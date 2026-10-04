/**
 * Integration tests for ControlServer.
 *
 * Each test function prints [PASS] or [FAIL] and returns bool.
 * A MultiThreadedExecutor runs in a background thread so that all ROS 2
 * topic/service callbacks are dispatched while the main thread blocks.
 *
 * Test coverage
 * ─────────────
 *  CS1  Single Joy type config registered — auto-selects first Sink as active
 *  CS2  Multiple Sinks, Joy + Twist types — each type independent active Sink
 *  CS3  Priority-based auto-selection — higher-priority Sink becomes active on register
 *  CS4  Active Sink manual override via setActiveSink()
 *  CS5  Fallback on Sink timeout — active switches to next-best ACTIVE Sink
 *  CS6  E-stop detection — emergencyStopCb fires; active Sink falls back
 *  CS7  Request-active — lower-priority Sink requests promotion; accepted if higher-pri
 *  CS8  EMERGENCY_STOP priority Sink — never selectable (auto or manual);
 *       normal messages from it do NOT fire emergencyStopCb (content-triggered only)
 *  CS9  outputCb receives messages from active Sink only
 *  CS10 No active Sink available — outputCb not called; emergencyStopCb fires
 *  CS11 Service mode — Joy source in SERVICE mode; outputCb fires via service-type cast
 *  CS12 Service mode — e-stop via service call; emergencyStopCb fires; fallback works
 *  CS13 Joy button messages (A/X/Y) delivered to outputCb with correct values
 *  CS14 Joy axes MOVE — axes[0..2] captured; vx/vy/vyaw mapping verified
 *  CS15 Twist single source — outputCb fires; linear.x/y + angular.z verified
 *  CS16 Twist e-stop — emergencyStopCb fires; active Sink falls back
 *  CS17 Twist request-active — HIGH promoted; LOW rejected
 *  CS18 Joy + Twist simultaneous — both outputCbs fire; values verified
 *  CS19 Fallback to LOW_FREQ Sink when ACTIVE unavailable
 *  CS20 Auto-disconnect on disconnect_timeout_ns
 *
 * Joy/Twist → unitree_api Request integration (full chain through
 * msgToOutSignal → SportClient → /api/sport/request):
 *  CS21 Joy buttons A/B/X/Y → StandUp/StandDown/StopMove/RecoveryStand api_ids
 *  CS22 PRESS edge — held button fires exactly once; re-press fires again
 *  CS23 Move dedup — identical (vx,vy,vyaw) not re-sent; changes are sent
 *  CS24 Idle joystick (all zeros) from startup → zero unitree requests
 *  CS25 Twist Move dedup — identical Twist not re-sent
 *  CS26 Joy e-stop → StopMove request published
 *  CS27 Per-axis Move — vx/vy/vyaw individually and combined (incl. negative
 *       values); unused axes (right V, L2 trigger) produce no output
 *
 * Joy mapping under test (real Xbox driver axis layout):
 *   axes[0]=vx, axes[1]=vy, axes[3]=vyaw  (axes[2]/[5] = L2/R2 triggers)
 *   buttons[0]=A(StandUp), [1]=B(StandDown), [2]=X(StopMove), [3]=Y(RecoveryStand)
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rv2_server_control/control_server.h"
#include "rv2_server_control/output_message_convert.h"
#include "rv2_control_signal_transport/control_signal_manager.h"

#include "unitree_api/msg/request.hpp"
#include "nlohmann/json.hpp"

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rv2_interfaces/srv/control_signal_joy.hpp>
#include <rv2_interfaces/srv/control_signal_twist.hpp>

using namespace rv2_interfaces;
using namespace rv2_interfaces::rv2_server_control;
using namespace std::chrono_literals;
using Joy   = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;

// Concrete Source types as registered with ControlSignalFactory
// (see control_signal_types.cpp). The factory always instantiates the
// service-bound specialisation for Joy/Twist — regardless of topic vs service
// mode — so getSource() must be down-cast to these exact types.
using JoySource   = ControlSignalSource<Joy,   rv2_interfaces::srv::ControlSignalJoy>;
using TwistSource = ControlSignalSource<Twist, rv2_interfaces::srv::ControlSignalTwist>;

// ── Shared helpers ────────────────────────────────────────────────────────────

[[maybe_unused]] static const char* stateName(ControlSignalState s)
{
    switch (s)
    {
        case ControlSignalState::UNKNOWN:      return "UNKNOWN";
        case ControlSignalState::ACTIVE:       return "ACTIVE";
        case ControlSignalState::LOW_FREQ:     return "LOW_FREQ";
        case ControlSignalState::TIMEOUT:      return "TIMEOUT";
        case ControlSignalState::DISCONNECTED: return "DISCONNECTED";
    }
    return "?";
}

// Priority tier literals matching the named-controller values of
// requester_priority.yaml controller_priority (local_xbox=80, remote_sfu=50,
// autonomous_agent=20). The old HIGH/MEDIUM/LOW constants were removed from
// ControlSignalConst.msg in favour of the yaml table + *_CUSTOM class ceilings.
static constexpr int8_t PRI_HIGH   = 80;
static constexpr int8_t PRI_MEDIUM = 50;
static constexpr int8_t PRI_LOW    = 20;

static msg::ControlSignalInfo makeInfo(
    const std::string& channel,
    const std::string& type,
    const std::string& mode,
    const std::string& targetCsm   = "",
    int8_t             priority    = PRI_MEDIUM,
    int64_t            timeoutNs   = 2'000'000'000LL,   // 2 s
    bool               useKA       = false,
    int64_t            kaIntervalNs = 300'000'000LL,    // 300 ms
    float              sendFreqHz  = 0.0f,              // 0 = disabled
    int64_t            disconnectTimeoutNs = 0)         // 0 = no auto-disconnect
{
    msg::ControlSignalInfo info;
    info.channel_name           = channel;
    info.control_signal_type    = type;
    info.control_signal_mode    = mode;
    info.target_csm_name        = targetCsm;
    info.priority               = priority;
    info.send_freq_hz           = sendFreqHz;
    info.timeout_ns             = timeoutNs;
    info.use_keep_alive         = useKA;
    info.keep_alive_interval_ns = kaIntervalNs;
    info.disconnect_timeout_ns  = disconnectTimeoutNs;
    // Required fields (validator rules 8/9): channel doubles as controller
    // identity in tests; all test controllers use the local custom class.
    info.controller_name        = channel;
    info.controller_priority_type =
        msg::ControlSignalConst::CONTROL_SIGNAL_PRIORITY_LOCAL_CUSTOM;
    return info;
}

static const std::string TYPE_JOY    = msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY;
static const std::string TYPE_TWIST  = msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST;
static const std::string MODE_TOPIC  = msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC;
static const std::string MODE_SERVICE = msg::ControlSignalConst::CONTROL_SIGNAL_MODE_SERVICE;

#define PASS(name)         RCLCPP_INFO (rclcpp::get_logger("test_cs"), "[PASS] %s", name)
#define FAIL(name, reason) do { RCLCPP_ERROR(rclcpp::get_logger("test_cs"), "[FAIL] " name ": %s", reason); return false; } while(0)

// Joy e-stop:       buttons[0..3] all == -99
// Joy request-act:  buttons[0..3] all ==  99
static bool joyIsEstop(const Joy& j)
{
    if (j.buttons.size() < 4) return false;
    return j.buttons[0]==-99 && j.buttons[1]==-99 && j.buttons[2]==-99 && j.buttons[3]==-99;
}
static bool joyIsReqActive(const Joy& j)
{
    if (j.buttons.size() < 4) return false;
    return j.buttons[0]==99 && j.buttons[1]==99 && j.buttons[2]==99 && j.buttons[3]==99;
}
static Joy makeJoy(float ax0, float ax1 = 0.f)
{
    Joy j;
    j.axes = {ax0, ax1};
    return j;
}
static Joy makeJoyEstop()
{
    Joy j;
    j.buttons = {-99,-99,-99,-99};
    return j;
}
static Joy makeJoyReqActive()
{
    Joy j;
    j.buttons = {99,99,99,99};
    return j;
}

// Twist e-stop / request-active
static bool twistIsEstop(const Twist& t)
{
    return t.linear.z == -99 && t.angular.x == -99 && t.angular.y == -99;
}
static bool twistIsReqActive(const Twist& t)
{
    return t.linear.z == 99 && t.angular.x == 99 && t.angular.y == 99;
}
static Twist makeTwist(double lx, double az = 0.0)
{
    Twist t;
    t.linear.x  = lx;
    t.angular.z = az;
    return t;
}

// ── Twist signal helpers ─────────────────────────────────────────────────────
//  Full-movement Twist (linear.x=vx, linear.y=vy, angular.z=vyaw)
static Twist makeTwistMove(double vx, double vy = 0.0, double vyaw = 0.0)
{
    Twist t;
    t.linear.x  = vx;
    t.linear.y  = vy;
    t.angular.z = vyaw;
    return t;
}
// Twist e-stop / request-active sentinels (linear.z/angular.x/angular.y == -99/99)
static Twist makeTwistEstop()
{
    Twist t;
    t.linear.z  = -99.0;
    t.angular.x = -99.0;
    t.angular.y = -99.0;
    return t;
}
static Twist makeTwistReqActive()
{
    Twist t;
    t.linear.z  = 99.0;
    t.angular.x = 99.0;
    t.angular.y = 99.0;
    return t;
}
// ── Joy button / axes helpers ─────────────────────────────────────────────────
//  makeJoyButton — allocates 17 buttons (docs/joy按鍵定義 layout) and sets
//                  buttons[btnIdx] = value; sticks at 0, L2/R2 released (+1.0).
//                  btnIdx < 0 → all buttons released.
static Joy makeJoyButton(int btnIdx, int value = 1)
{
    Joy j;
    j.buttons.assign(17, 0);
    j.axes = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    if (btnIdx >= 0 && btnIdx < 17)
        j.buttons[btnIdx] = value;
    return j;
}
//  makeJoyAxes — real Xbox driver layout: axes[0]=vx (left H), axes[1]=vy
//               (left V), axes[3]=vyaw (right H); axes[2]/[5] are the L2/R2
//               triggers, released = +1.0 (axes[5] < 0.5 is the e-stop
//               sentinel in control_signal_detect.h).
static Joy makeJoyAxes(float vx, float vy = 0.0f, float vyaw = 0.0f)
{
    Joy j;
    j.axes    = {vx, vy, 1.0f, vyaw, 0.0f, 1.0f, 0.0f, 0.0f};
    j.buttons.assign(17, 0);
    return j;
}
//  makeJoyAxesFull — like makeJoyAxes but with explicit unused axes
//                    (right stick vertical, L2 trigger) for no-output checks.
static Joy makeJoyAxesFull(float vx, float vy, float vyaw,
                           float rightV, float l2 = 1.0f)
{
    Joy j;
    j.axes    = {vx, vy, l2, vyaw, rightV, 1.0f, 0.0f, 0.0f};
    j.buttons.assign(17, 0);
    return j;
}

// Helper: build a TypeConfig<Joy> with counters wired to outputCb / emergencyStopCb
struct JoyTracker
{
    std::atomic<int>       outputCount{0};
    std::atomic<int>       estopCount{0};
    std::string            lastOutputChannel;
    Joy                    lastOutputMsg;
    std::mutex             mtx;

    ControlServer::TypeConfig<Joy> makeConfig()
    {
        ControlServer::TypeConfig<Joy> cfg;
        cfg.isEmergencyStop = joyIsEstop;
        cfg.isRequestActive = joyIsReqActive;
        cfg.outputCb = [this](const Joy& msg, const msg::ControlSignalInfo& info)
        {
            ++outputCount;
            std::lock_guard<std::mutex> lk(mtx);
            lastOutputChannel = info.channel_name;
            lastOutputMsg     = msg;
        };
        cfg.emergencyStopCb = [this](const msg::ControlSignalInfo&) { ++estopCount; };
        return cfg;
    }
};

struct TwistTracker
{
    std::atomic<int>  outputCount{0};
    std::atomic<int>  estopCount{0};
    std::string       lastOutputChannel;
    Twist             lastOutputMsg;
    std::mutex        mtx;

    ControlServer::TypeConfig<Twist> makeConfig()
    {
        ControlServer::TypeConfig<Twist> cfg;
        cfg.isEmergencyStop = twistIsEstop;
        cfg.isRequestActive = twistIsReqActive;
        cfg.outputCb = [this](const Twist& msg, const msg::ControlSignalInfo& info)
        {
            ++outputCount;
            std::lock_guard<std::mutex> lk(mtx);
            lastOutputChannel = info.channel_name;
            lastOutputMsg     = msg;
        };
        cfg.emergencyStopCb = [this](const msg::ControlSignalInfo&) { ++estopCount; };
        return cfg;
    }
};

// Register a Source on CSM and immediately call its remote registration.
// Returns the created Source pointer (cast to typed) or nullptr on failure.
static bool csmRegisterSource(
    ControlSignalManager& srcCsm,
    const msg::ControlSignalInfo& info,
    int timeoutMs = 3000)
{
    return srcCsm.registerSource(info, timeoutMs);
}


// ════════════════════════════════════════════════════════════════════════════
//  CS1 — Single Joy config; first Sink auto-selected as active
// ════════════════════════════════════════════════════════════════════════════
bool testCS1_SingleTypeFirstSinkActive(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs1_server", 50'000'000LL});

    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    // Source CSM — registers a Source and creates the remote Sink in our CS
    ControlSignalManager srcCsm(srcNode.get(), "cs1_source_csm");
    auto info = makeInfo("cs1/joy_a", TYPE_JOY, MODE_TOPIC, "cs1_server");
    if (!csmRegisterSource(srcCsm, info))
        FAIL("CS1", "registerSource failed");

    rclcpp::sleep_for(300ms);   // topic discovery

    // Send a normal Joy message via the Source
    auto* src = dynamic_cast<JoySource*>(srcCsm.getSource("cs1/joy_a").get());
    if (!src) FAIL("CS1", "getSource returned nullptr");

    bool ok;
    src->send(makeJoy(0.5f), ok);
    rclcpp::sleep_for(300ms);

    // After receiving the message the Sink is ACTIVE and the CS should select it
    auto active = cs.getActiveSinkChannel<Joy>();
    if (active != "cs1/joy_a")
        FAIL("CS1", ("expected active='cs1/joy_a', got='" + active + "'").c_str());

    // outputCb should have fired at least once (event-triggered on message arrival)
    rclcpp::sleep_for(200ms);
    if (tracker.outputCount.load() == 0)
        FAIL("CS1", "outputCb never called");

    PASS("CS1  Single Joy config — first Sink auto-selected as active");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS2 — Joy + Twist types registered simultaneously; independent active Sinks
// ════════════════════════════════════════════════════════════════════════════
bool testCS2_MultipleTypeConfigs(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs2_server", 50'000'000LL});

    JoyTracker   joyTracker;
    TwistTracker twistTracker;
    cs.registerTypeConfig(joyTracker.makeConfig());
    cs.registerTypeConfig(twistTracker.makeConfig());

    ControlSignalManager srcCsm(srcNode.get(), "cs2_source_csm");

    auto joyInfo = makeInfo("cs2/joy",   TYPE_JOY,   MODE_TOPIC, "cs2_server");
    auto twsInfo = makeInfo("cs2/twist", TYPE_TWIST, MODE_TOPIC, "cs2_server");
    if (!csmRegisterSource(srcCsm, joyInfo))   FAIL("CS2", "joy registerSource failed");
    if (!csmRegisterSource(srcCsm, twsInfo))   FAIL("CS2", "twist registerSource failed");

    rclcpp::sleep_for(300ms);

    auto* joySrc = dynamic_cast<JoySource*>(
        srcCsm.getSource("cs2/joy").get());
    auto* twsSrc = dynamic_cast<TwistSource*>(
        srcCsm.getSource("cs2/twist").get());
    if (!joySrc || !twsSrc) FAIL("CS2", "getSource returned nullptr");

    bool ok;
    joySrc->send(makeJoy(1.0f), ok);
    twsSrc->send(makeTwist(2.0), ok);
    rclcpp::sleep_for(300ms);

    auto joyActive = cs.getActiveSinkChannel<Joy>();
    auto twsActive = cs.getActiveSinkChannel<Twist>();
    if (joyActive != "cs2/joy")
        FAIL("CS2", ("Joy active expected 'cs2/joy', got '" + joyActive + "'").c_str());
    if (twsActive != "cs2/twist")
        FAIL("CS2", ("Twist active expected 'cs2/twist', got '" + twsActive + "'").c_str());

    rclcpp::sleep_for(200ms);
    if (joyTracker.outputCount.load() == 0)
        FAIL("CS2", "Joy outputCb never called");
    if (twistTracker.outputCount.load() == 0)
        FAIL("CS2", "Twist outputCb never called");

    PASS("CS2  Joy + Twist types — independent active Sinks, both outputCbs fired");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS3 — Priority auto-select after timeout: when LOW times out, a higher-
//        priority ACTIVE Sink takes over on its next message arrival
// ════════════════════════════════════════════════════════════════════════════
bool testCS3_PriorityAutoSelect(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNodeLow,
    rclcpp::Node::SharedPtr srcNodeHigh)
{
    ControlServer cs(csNode.get(), {"cs3_server", 50'000'000LL});
    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager csmLow (srcNodeLow.get(),  "cs3_csm_low");
    ControlSignalManager csmHigh(srcNodeHigh.get(), "cs3_csm_high");

    // LOW has a short 500 ms timeout; HIGH has the default 2 s timeout.
    auto infoLow  = makeInfo("cs3/joy_low",  TYPE_JOY, MODE_TOPIC, "cs3_server",
                             PRI_LOW,
                             500'000'000LL);   // 500 ms
    auto infoHigh = makeInfo("cs3/joy_high", TYPE_JOY, MODE_TOPIC, "cs3_server",
                             PRI_HIGH);

    if (!csmRegisterSource(csmLow,  infoLow))  FAIL("CS3", "low registerSource failed");
    if (!csmRegisterSource(csmHigh, infoHigh)) FAIL("CS3", "high registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* srcLow  = dynamic_cast<JoySource*>(
        csmLow.getSource("cs3/joy_low").get());
    auto* srcHigh = dynamic_cast<JoySource*>(
        csmHigh.getSource("cs3/joy_high").get());
    if (!srcLow || !srcHigh) FAIL("CS3", "getSource nullptr");

    bool ok;
    // LOW sends first — becomes active (no prior active).
    srcLow->send(makeJoy(0.1f), ok);
    rclcpp::sleep_for(200ms);

    if (cs.getActiveSinkChannel<Joy>() != "cs3/joy_low")
        FAIL("CS3", "expected 'cs3/joy_low' as initial active");

    // HIGH sends a normal message while LOW is ACTIVE → must NOT steal.
    srcHigh->send(makeJoy(0.9f), ok);
    rclcpp::sleep_for(200ms);

    if (cs.getActiveSinkChannel<Joy>() != "cs3/joy_low")
        FAIL("CS3", "HIGH normal message must not steal active from ACTIVE LOW sink");

    // Stop sending from LOW — let it time out (> 500 ms).
    rclcpp::sleep_for(600ms);

    // HIGH sends another message. Now LOW is TIMEOUT → auto-promote allowed → HIGH wins.
    srcHigh->send(makeJoy(0.9f), ok);
    rclcpp::sleep_for(200ms);

    auto active = cs.getActiveSinkChannel<Joy>();
    if (active != "cs3/joy_high")
        FAIL("CS3", ("expected 'cs3/joy_high' after LOW timeout, got '" + active + "'").c_str());

    PASS("CS3  Priority auto-select after timeout — HIGH promoted when LOW timed out");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS4 — Manual active Sink override via setActiveSink()
// ════════════════════════════════════════════════════════════════════════════
bool testCS4_ManualActiveSinkOverride(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNodeA,
    rclcpp::Node::SharedPtr srcNodeB)
{
    ControlServer cs(csNode.get(), {"cs4_server", 50'000'000LL});
    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager csmA(srcNodeA.get(), "cs4_csm_a");
    ControlSignalManager csmB(srcNodeB.get(), "cs4_csm_b");

    // Both with MEDIUM priority
    auto infoA = makeInfo("cs4/joy_a", TYPE_JOY, MODE_TOPIC, "cs4_server",
                          PRI_MEDIUM);
    auto infoB = makeInfo("cs4/joy_b", TYPE_JOY, MODE_TOPIC, "cs4_server",
                          PRI_MEDIUM);

    if (!csmRegisterSource(csmA, infoA)) FAIL("CS4", "A registerSource failed");
    if (!csmRegisterSource(csmB, infoB)) FAIL("CS4", "B registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* srcA = dynamic_cast<JoySource*>(csmA.getSource("cs4/joy_a").get());
    auto* srcB = dynamic_cast<JoySource*>(csmB.getSource("cs4/joy_b").get());
    if (!srcA || !srcB) FAIL("CS4", "getSource nullptr");

    bool ok;
    srcA->send(makeJoy(0.1f), ok);
    srcB->send(makeJoy(0.2f), ok);
    rclcpp::sleep_for(200ms);

    // setActiveSink on a non-existent channel must fail
    if (cs.setActiveSink<Joy>("cs4/nonexistent"))
        FAIL("CS4", "setActiveSink should fail for unknown channel");

    // Manually set B as active (overrides A which was first)
    if (!cs.setActiveSink<Joy>("cs4/joy_b"))
        FAIL("CS4", "setActiveSink('cs4/joy_b') returned false");

    if (cs.getActiveSinkChannel<Joy>() != "cs4/joy_b")
        FAIL("CS4", "expected 'cs4/joy_b' after manual override");

    // outputCb channel should now be joy_b
    tracker.outputCount = 0;
    rclcpp::sleep_for(200ms);
    {
        std::lock_guard<std::mutex> lk(tracker.mtx);
        if (tracker.outputCount > 0 && tracker.lastOutputChannel != "cs4/joy_b")
            FAIL("CS4", ("outputCb from wrong channel: " + tracker.lastOutputChannel).c_str());
    }

    PASS("CS4  Manual active Sink override via setActiveSink()");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS5 — Fallback on Sink timeout: active switches to next-best ACTIVE Sink
// ════════════════════════════════════════════════════════════════════════════
bool testCS5_FallbackOnTimeout(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNodeHigh,
    rclcpp::Node::SharedPtr srcNodeLow)
{
    // 500 ms timeout for the high-priority Sink
    ControlServer cs(csNode.get(), {"cs5_server", 50'000'000LL});
    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager csmHigh(srcNodeHigh.get(), "cs5_csm_high");
    ControlSignalManager csmLow (srcNodeLow.get(),  "cs5_csm_low");

    auto infoHigh = makeInfo("cs5/joy_high", TYPE_JOY, MODE_TOPIC, "cs5_server",
                             PRI_HIGH,
                             500'000'000LL);  // 500 ms timeout
    auto infoLow  = makeInfo("cs5/joy_low",  TYPE_JOY, MODE_TOPIC, "cs5_server",
                             PRI_LOW,
                             2'000'000'000LL);  // 2 s timeout

    if (!csmRegisterSource(csmHigh, infoHigh)) FAIL("CS5", "high registerSource failed");
    if (!csmRegisterSource(csmLow,  infoLow))  FAIL("CS5", "low registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* srcHigh = dynamic_cast<JoySource*>(csmHigh.getSource("cs5/joy_high").get());
    auto* srcLow  = dynamic_cast<JoySource*>(csmLow.getSource("cs5/joy_low").get());
    if (!srcHigh || !srcLow) FAIL("CS5", "getSource nullptr");

    bool ok;
    // Send HIGH first and wait for its message to be delivered.  Only after
    // HIGH is confirmed active do we send LOW, eliminating the race where LOW's
    // topic message arrives first and wins the "first active" slot.
    srcHigh->send(makeJoy(0.9f), ok);
    rclcpp::sleep_for(150ms);

    if (cs.getActiveSinkChannel<Joy>() != "cs5/joy_high")
        FAIL("CS5", "expected 'cs5/joy_high' as initial active");

    srcLow->send(makeJoy(0.1f), ok);
    rclcpp::sleep_for(100ms);

    // Stop sending from HIGH → wait for its Sink to timeout (> 500 ms)
    // LOW keeps being refreshed
    for (int i = 0; i < 4; ++i)
    {
        srcLow->send(makeJoy(0.1f), ok);
        rclcpp::sleep_for(200ms);
    }
    // By now: high Sink → TIMEOUT; low Sink → ACTIVE
    // The safety watchdog should have fallen back to 'joy_low'
    rclcpp::sleep_for(150ms);

    auto active = cs.getActiveSinkChannel<Joy>();
    if (active != "cs5/joy_low")
        FAIL("CS5", ("expected fallback to 'cs5/joy_low', got '" + active + "'").c_str());

    PASS("CS5  Fallback on timeout — active switched to next-best ACTIVE Sink");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS6 — E-stop detection: emergencyStopCb fires; active Sink falls back
// ════════════════════════════════════════════════════════════════════════════
bool testCS6_EmergencyStop(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNodeHigh,
    rclcpp::Node::SharedPtr srcNodeLow)
{
    ControlServer cs(csNode.get(), {"cs6_server", 50'000'000LL});
    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager csmHigh(srcNodeHigh.get(), "cs6_csm_high");
    ControlSignalManager csmLow (srcNodeLow.get(),  "cs6_csm_low");

    auto infoHigh = makeInfo("cs6/joy_high", TYPE_JOY, MODE_TOPIC, "cs6_server",
                             PRI_HIGH);
    auto infoLow  = makeInfo("cs6/joy_low",  TYPE_JOY, MODE_TOPIC, "cs6_server",
                             PRI_LOW);

    if (!csmRegisterSource(csmHigh, infoHigh)) FAIL("CS6", "high registerSource failed");
    if (!csmRegisterSource(csmLow,  infoLow))  FAIL("CS6", "low registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* srcHigh = dynamic_cast<JoySource*>(csmHigh.getSource("cs6/joy_high").get());
    auto* srcLow  = dynamic_cast<JoySource*>(csmLow.getSource("cs6/joy_low").get());
    if (!srcHigh || !srcLow) FAIL("CS6", "getSource nullptr");

    bool ok;
    // Send HIGH first and wait for delivery before sending LOW (avoids race on
    // first-active slot since auto-promote only fires when current active is TIMEOUT).
    srcHigh->send(makeJoy(0.9f), ok);
    rclcpp::sleep_for(150ms);

    if (cs.getActiveSinkChannel<Joy>() != "cs6/joy_high")
        FAIL("CS6", "expected 'cs6/joy_high' as initial active");

    srcLow->send(makeJoy(0.1f), ok);
    rclcpp::sleep_for(100ms);

    // HIGH source sends e-stop
    tracker.estopCount = 0;
    srcHigh->send(makeJoyEstop(), ok);
    rclcpp::sleep_for(300ms);

    if (tracker.estopCount.load() == 0)
        FAIL("CS6", "emergencyStopCb was not called on e-stop");

    // Active should have fallen back to LOW (next-best ACTIVE non-estop-priority Sink)
    auto active = cs.getActiveSinkChannel<Joy>();
    if (active != "cs6/joy_low")
        FAIL("CS6", ("expected fallback to 'cs6/joy_low' after e-stop, got '" + active + "'").c_str());

    PASS("CS6  E-stop detection — emergencyStopCb fired; active fell back to LOW Sink");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS7 — Request-active: lower-priority Sink requests promotion; accepted if higher-pri
// ════════════════════════════════════════════════════════════════════════════
bool testCS7_RequestActive(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNodeLow,
    rclcpp::Node::SharedPtr srcNodeHigh)
{
    ControlServer cs(csNode.get(), {"cs7_server", 50'000'000LL});
    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager csmLow (srcNodeLow.get(),  "cs7_csm_low");
    ControlSignalManager csmHigh(srcNodeHigh.get(), "cs7_csm_high");

    auto infoLow  = makeInfo("cs7/joy_low",  TYPE_JOY, MODE_TOPIC, "cs7_server",
                             PRI_LOW);
    auto infoHigh = makeInfo("cs7/joy_high", TYPE_JOY, MODE_TOPIC, "cs7_server",
                             PRI_HIGH);

    if (!csmRegisterSource(csmLow,  infoLow))  FAIL("CS7", "low registerSource failed");
    if (!csmRegisterSource(csmHigh, infoHigh)) FAIL("CS7", "high registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* srcLow  = dynamic_cast<JoySource*>(csmLow.getSource("cs7/joy_low").get());
    auto* srcHigh = dynamic_cast<JoySource*>(csmHigh.getSource("cs7/joy_high").get());
    if (!srcLow || !srcHigh) FAIL("CS7", "getSource nullptr");

    bool ok;
    // LOW sends first → becomes initial active
    srcLow->send(makeJoy(0.1f), ok);
    rclcpp::sleep_for(200ms);

    if (cs.getActiveSinkChannel<Joy>() != "cs7/joy_low")
        FAIL("CS7", "expected 'cs7/joy_low' as initial active");

    // HIGH sends a normal message (not request-active) → not promoted yet
    srcHigh->send(makeJoy(0.5f), ok);
    rclcpp::sleep_for(200ms);
    if (cs.getActiveSinkChannel<Joy>() != "cs7/joy_low")
        FAIL("CS7", "normal message from high should not steal active from low");

    // HIGH sends request-active → should be promoted (HIGH > LOW)
    srcHigh->send(makeJoyReqActive(), ok);
    rclcpp::sleep_for(200ms);
    auto active = cs.getActiveSinkChannel<Joy>();
    if (active != "cs7/joy_high")
        FAIL("CS7", ("expected 'cs7/joy_high' after request-active, got '" + active + "'").c_str());

    // LOW tries request-active → should NOT be promoted (LOW < HIGH)
    srcLow->send(makeJoyReqActive(), ok);
    rclcpp::sleep_for(200ms);
    if (cs.getActiveSinkChannel<Joy>() != "cs7/joy_high")
        FAIL("CS7", "LOW request-active should not steal active from HIGH");

    // EQUAL priority tries request-active → must also be rejected
    // (takeover requires priority STRICTLY greater than the incumbent's).
    ControlSignalManager csmEq(srcNodeLow.get(), "cs7_csm_eq");
    auto infoEq = makeInfo("cs7/joy_eq", TYPE_JOY, MODE_TOPIC, "cs7_server",
                           PRI_HIGH);
    if (!csmRegisterSource(csmEq, infoEq)) FAIL("CS7", "eq registerSource failed");
    rclcpp::sleep_for(300ms);
    auto* srcEq = dynamic_cast<JoySource*>(csmEq.getSource("cs7/joy_eq").get());
    if (!srcEq) FAIL("CS7", "eq getSource nullptr");

    srcEq->send(makeJoyReqActive(), ok);
    rclcpp::sleep_for(200ms);
    if (cs.getActiveSinkChannel<Joy>() != "cs7/joy_high")
        FAIL("CS7", "EQUAL-priority request-active must not steal active (strictly-greater rule)");

    PASS("CS7  Request-active — HIGH promoted; LOW and EQUAL rejected (strictly-greater rule)");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS8 — EMERGENCY_STOP priority Sink: never selectable (auto or manual);
//        a normal message from it does NOT fire emergencyStopCb
// ════════════════════════════════════════════════════════════════════════════
bool testCS8_EmergencyStopPrioritySink(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNodeNormal,
    rclcpp::Node::SharedPtr srcNodeEstop)
{
    ControlServer cs(csNode.get(), {"cs8_server", 50'000'000LL});
    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager csmNormal(srcNodeNormal.get(), "cs8_csm_normal");
    ControlSignalManager csmEstop (srcNodeEstop.get(),  "cs8_csm_estop");

    auto infoNormal = makeInfo("cs8/joy_normal", TYPE_JOY, MODE_TOPIC, "cs8_server",
                               PRI_MEDIUM);
    auto infoEstop  = makeInfo("cs8/joy_estop",  TYPE_JOY, MODE_TOPIC, "cs8_server",
                               msg::ControlSignalConst::CONTROL_SIGNAL_PRIORITY_EMERGENCY_STOP);

    if (!csmRegisterSource(csmNormal, infoNormal)) FAIL("CS8", "normal registerSource failed");
    if (!csmRegisterSource(csmEstop,  infoEstop))  FAIL("CS8", "estop registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* srcNormal = dynamic_cast<JoySource*>(csmNormal.getSource("cs8/joy_normal").get());
    auto* srcEstop  = dynamic_cast<JoySource*>(csmEstop.getSource("cs8/joy_estop").get());
    if (!srcNormal || !srcEstop) FAIL("CS8", "getSource nullptr");

    bool ok;
    srcNormal->send(makeJoy(0.5f), ok);
    rclcpp::sleep_for(200ms);

    // EMERGENCY_STOP priority Sink must never become active even after receiving msg
    auto active = cs.getActiveSinkChannel<Joy>();
    if (active == "cs8/joy_estop")
        FAIL("CS8", "EMERGENCY_STOP priority Sink must never be selected as active");
    if (active != "cs8/joy_normal")
        FAIL("CS8", ("expected active='cs8/joy_normal', got='" + active + "'").c_str());

    // A NORMAL message from the EMERGENCY_STOP-priority Sink must NOT fire
    // emergencyStopCb — e-stop is content-triggered (isEmergencyStop) only.
    // The Sink is merely never selectable as output.
    tracker.estopCount = 0;
    srcEstop->send(makeJoy(1.0f), ok);
    rclcpp::sleep_for(300ms);

    {
        std::lock_guard<std::mutex> lk(tracker.mtx);
        if (tracker.estopCount != 0)
            FAIL("CS8", "normal msg from ESTOP-priority Sink must not fire emergencyStopCb");
    }

    // Active should still be (or revert to) 'cs8/joy_normal'
    active = cs.getActiveSinkChannel<Joy>();
    if (active == "cs8/joy_estop")
        FAIL("CS8", "EMERGENCY_STOP priority Sink must not become active after normal msg");

    // Manual activation of an ESTOP-priority Sink must be refused as well.
    if (cs.setActiveSink<Joy>("cs8/joy_estop"))
        FAIL("CS8", "setActiveSink must refuse an EMERGENCY_STOP-priority Sink");
    if (cs.getActiveSinkChannel<Joy>() != "cs8/joy_normal")
        FAIL("CS8", "active must remain 'cs8/joy_normal' after refused manual activation");

    PASS("CS8  EMERGENCY_STOP priority Sink — never selectable (auto or manual); "
         "normal msgs don't fire e-stop");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS9 — outputCb only fires for the active Sink's messages
// ════════════════════════════════════════════════════════════════════════════
bool testCS9_OutputCbActiveOnly(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNodeA,
    rclcpp::Node::SharedPtr srcNodeB)
{
    ControlServer cs(csNode.get(), {"cs9_server", 50'000'000LL});
    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager csmA(srcNodeA.get(), "cs9_csm_a");
    ControlSignalManager csmB(srcNodeB.get(), "cs9_csm_b");

    auto infoA = makeInfo("cs9/joy_a", TYPE_JOY, MODE_TOPIC, "cs9_server",
                          PRI_HIGH);
    auto infoB = makeInfo("cs9/joy_b", TYPE_JOY, MODE_TOPIC, "cs9_server",
                          PRI_LOW);

    if (!csmRegisterSource(csmA, infoA)) FAIL("CS9", "A registerSource failed");
    if (!csmRegisterSource(csmB, infoB)) FAIL("CS9", "B registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* srcA = dynamic_cast<JoySource*>(csmA.getSource("cs9/joy_a").get());
    auto* srcB = dynamic_cast<JoySource*>(csmB.getSource("cs9/joy_b").get());
    if (!srcA || !srcB) FAIL("CS9", "getSource nullptr");

    bool ok;
    // Send A (HIGH) first and wait for delivery before sending B, to avoid the
    // race where B's topic message arrives first and wins the first-active slot.
    srcA->send(makeJoy(1.0f), ok);
    rclcpp::sleep_for(150ms);

    // A (HIGH) should be active
    if (cs.getActiveSinkChannel<Joy>() != "cs9/joy_a")
        FAIL("CS9", "expected 'cs9/joy_a' (HIGH) as active");

    srcB->send(makeJoy(0.2f), ok);
    rclcpp::sleep_for(100ms);

    // Output is event-triggered: send several more messages from BOTH sinks.
    // Only the active sink (A) must drive outputCb.
    tracker.outputCount = 0;
    for (int i = 0; i < 3; ++i)
    {
        srcA->send(makeJoy(1.0f), ok);
        srcB->send(makeJoy(0.2f), ok);
        rclcpp::sleep_for(100ms);
    }
    {
        std::lock_guard<std::mutex> lk(tracker.mtx);
        if (!tracker.lastOutputChannel.empty() && tracker.lastOutputChannel != "cs9/joy_a")
            FAIL("CS9", ("outputCb used wrong channel: " + tracker.lastOutputChannel).c_str());
    }
    if (tracker.outputCount.load() == 0)
        FAIL("CS9", "outputCb never called");

    PASS("CS9  outputCb only fires for active Sink (channel A, HIGH priority)");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS10 — No ACTIVE Sink: outputCb not called; emergencyStopCb fires
// ════════════════════════════════════════════════════════════════════════════
bool testCS10_NoActiveSink(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    // Very short timeout: 300 ms
    ControlServer cs(csNode.get(), {"cs10_server", 50'000'000LL});
    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager csmSrc(srcNode.get(), "cs10_csm_src");
    auto info = makeInfo("cs10/joy", TYPE_JOY, MODE_TOPIC, "cs10_server",
                         PRI_MEDIUM,
                         300'000'000LL);  // 300 ms timeout

    if (!csmRegisterSource(csmSrc, info)) FAIL("CS10", "registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* src = dynamic_cast<JoySource*>(csmSrc.getSource("cs10/joy").get());
    if (!src) FAIL("CS10", "getSource nullptr");

    bool ok;
    src->send(makeJoy(0.5f), ok);
    rclcpp::sleep_for(200ms);

    if (cs.getActiveSinkChannel<Joy>() != "cs10/joy")
        FAIL("CS10", "expected 'cs10/joy' as active after first message");

    // Stop sending — sink will timeout after 300 ms
    tracker.outputCount = 0;
    tracker.estopCount  = 0;
    rclcpp::sleep_for(700ms);   // well past timeout

    // By now the watchdog should have detected no usable Sink and fired emergencyStopCb
    if (tracker.estopCount.load() == 0)
        FAIL("CS10", "emergencyStopCb should fire when no ACTIVE Sink available");

    PASS("CS10  No active Sink — outputCb silent; emergencyStopCb fires");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS11 — SERVICE mode: Joy source; outputCb fires via ControlSignalSink<Joy,SrvT> cast
// ════════════════════════════════════════════════════════════════════════════
bool testCS11_ServiceModeOutputCb(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs11_server", 50'000'000LL});
    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager srcCsm(srcNode.get(), "cs11_source_csm");
    // timeout_ns is reused as the service-call wait timeout inside send().
    auto info = makeInfo("cs11/joy_svc", TYPE_JOY, MODE_SERVICE, "cs11_server",
                         PRI_MEDIUM,
                         500'000'000LL);  // 500 ms service-call timeout

    if (!csmRegisterSource(srcCsm, info))
        FAIL("CS11", "registerSource (service mode) failed");

    rclcpp::sleep_for(200ms);  // allow service server to be ready

    // Source must be the service-mode specialisation.
    auto* src = dynamic_cast<ControlSignalSource<Joy, rv2_interfaces::srv::ControlSignalJoy>*>(
        srcCsm.getSource("cs11/joy_svc").get());
    if (!src)
        FAIL("CS11", "getSource did not return ControlSignalSource<Joy,ControlSignalJoy>");

    // Send via service call — blocks until response (executor running in background thread).
    bool ok;
    if (!src->send(makeJoy(0.7f), ok))
        FAIL("CS11", "send() returned false (service call failed or timed out)");
    if (!ok)
        FAIL("CS11", "send() succeeded but server returned non-success response");
    rclcpp::sleep_for(200ms);

    // Sink should be ACTIVE and selected as the active channel.
    auto active = cs.getActiveSinkChannel<Joy>();
    if (active != "cs11/joy_svc")
        FAIL("CS11", ("expected active='cs11/joy_svc', got='" + active + "'").c_str());

    // outputCb must fire on message arrival — exercises the SrvT service path
    // (the typed Sink delivers the message that drives event-triggered output).
    rclcpp::sleep_for(200ms);
    if (tracker.outputCount.load() == 0)
        FAIL("CS11", "outputCb never called — service-mode message did not drive output");

    PASS("CS11  Service mode — outputCb fired via ControlSignalSink<Joy,SrvT> cast");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS12 — SERVICE mode: e-stop via service call; emergencyStopCb fires; fallback works
// ════════════════════════════════════════════════════════════════════════════
bool testCS12_ServiceModeEmergencyStop(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNodeHigh,
    rclcpp::Node::SharedPtr srcNodeLow)
{
    ControlServer cs(csNode.get(), {"cs12_server", 50'000'000LL});
    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager csmHigh(srcNodeHigh.get(), "cs12_csm_high");
    ControlSignalManager csmLow (srcNodeLow.get(),  "cs12_csm_low");

    // HIGH uses SERVICE mode; LOW uses TOPIC mode so it can keep refreshing freely.
    auto infoHigh = makeInfo("cs12/joy_high", TYPE_JOY, MODE_SERVICE, "cs12_server",
                             PRI_HIGH,
                             500'000'000LL);  // 500 ms service-call timeout
    auto infoLow  = makeInfo("cs12/joy_low",  TYPE_JOY, MODE_TOPIC,   "cs12_server",
                             PRI_LOW);

    if (!csmRegisterSource(csmHigh, infoHigh)) FAIL("CS12", "high registerSource failed");
    if (!csmRegisterSource(csmLow,  infoLow))  FAIL("CS12", "low registerSource failed");
    rclcpp::sleep_for(200ms);

    auto* srcHigh = dynamic_cast<ControlSignalSource<Joy, rv2_interfaces::srv::ControlSignalJoy>*>(
        csmHigh.getSource("cs12/joy_high").get());
    auto* srcLow  = dynamic_cast<JoySource*>(
        csmLow.getSource("cs12/joy_low").get());
    if (!srcHigh || !srcLow) FAIL("CS12", "getSource nullptr");

    bool ok;
    // LOW (topic) sends first → becomes initial active.
    srcLow->send(makeJoy(0.1f), ok);
    rclcpp::sleep_for(150ms);
    if (cs.getActiveSinkChannel<Joy>() != "cs12/joy_low")
        FAIL("CS12", "expected 'cs12/joy_low' as initial active");

    // HIGH sends a normal message (service) — LOW still ACTIVE, so HIGH must NOT steal.
    srcHigh->send(makeJoy(0.8f), ok);
    rclcpp::sleep_for(150ms);
    if (cs.getActiveSinkChannel<Joy>() != "cs12/joy_low")
        FAIL("CS12", "normal service-mode msg from HIGH must not steal active from ACTIVE LOW");

    // HIGH sends e-stop via service call — emergencyStopCb must fire.
    tracker.estopCount = 0;
    srcHigh->send(makeJoyEstop(), ok);
    rclcpp::sleep_for(300ms);

    if (tracker.estopCount.load() == 0)
        FAIL("CS12", "emergencyStopCb not called on service-mode e-stop");

    // e-stop sender must be excluded from fallback selection.
    auto active = cs.getActiveSinkChannel<Joy>();
    if (active == "cs12/joy_high")
        FAIL("CS12", "e-stop sender must not be re-selected as active after e-stop");
    if (active != "cs12/joy_low")
        FAIL("CS12", ("expected fallback to 'cs12/joy_low', got='" + active + "'").c_str());

    PASS("CS12  Service mode e-stop — emergencyStopCb fired; active fell back to LOW (topic)");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS13 — Joy button commands: StandUp / StopMove / RecoveryStand delivered
//         to outputCb with correct button values
// ════════════════════════════════════════════════════════════════════════════
bool testCS13_JoyButtonCommands(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs13_server", 50'000'000LL});
    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager srcCsm(srcNode.get(), "cs13_source_csm");
    auto info = makeInfo("cs13/joy", TYPE_JOY, MODE_TOPIC, "cs13_server");
    if (!csmRegisterSource(srcCsm, info))
        FAIL("CS13", "registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* src = dynamic_cast<JoySource*>(
        srcCsm.getSource("cs13/joy").get());
    if (!src) FAIL("CS13", "getSource nullptr");

    bool ok;
    // Activate the sink first with a plain MOVE message
    src->send(makeJoyAxes(0.5f), ok);
    rclcpp::sleep_for(200ms);
    if (cs.getActiveSinkChannel<Joy>() != "cs13/joy")
        FAIL("CS13", "sink not active after initial MOVE message");

    // ── buttons[0] = 1  (A → StandUp, api_id 1004) ───────────────────────────
    tracker.outputCount = 0;
    src->send(makeJoyButton(JOY_BTN_A), ok);
    rclcpp::sleep_for(200ms);
    {
        std::lock_guard<std::mutex> lk(tracker.mtx);
        if (tracker.outputCount == 0)
            FAIL("CS13", "outputCb not called for StandUp (buttons[0]=1)");
        const auto& btn = tracker.lastOutputMsg.buttons;
        if (static_cast<int>(btn.size()) < 1 || btn[JOY_BTN_A] != 1)
            FAIL("CS13", "captured msg: buttons[0] != 1 for StandUp");
    }

    // ── buttons[2] = 1  (X → StopMove, api_id 1003) ──────────────────────────
    tracker.outputCount = 0;
    src->send(makeJoyButton(JOY_BTN_X), ok);
    rclcpp::sleep_for(200ms);
    {
        std::lock_guard<std::mutex> lk(tracker.mtx);
        if (tracker.outputCount == 0)
            FAIL("CS13", "outputCb not called for StopMove (buttons[2]=1)");
        const auto& btn = tracker.lastOutputMsg.buttons;
        if (static_cast<int>(btn.size()) < 3 || btn[JOY_BTN_X] != 1)
            FAIL("CS13", "captured msg: buttons[2] != 1 for StopMove");
    }

    // ── buttons[3] = 1  (Y → RecoveryStand, api_id 1006) ─────────────────────
    tracker.outputCount = 0;
    src->send(makeJoyButton(JOY_BTN_Y), ok);
    rclcpp::sleep_for(200ms);
    {
        std::lock_guard<std::mutex> lk(tracker.mtx);
        if (tracker.outputCount == 0)
            FAIL("CS13", "outputCb not called for RecoveryStand (buttons[3]=1)");
        const auto& btn = tracker.lastOutputMsg.buttons;
        if (static_cast<int>(btn.size()) < 4 || btn[JOY_BTN_Y] != 1)
            FAIL("CS13", "captured msg: buttons[3] != 1 for RecoveryStand");
    }

    PASS("CS13  Joy button commands — A/X/Y button messages delivered to outputCb");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS14 — Joy axes MOVE: axes captured correctly; vx/vy/vyaw verified
//         vx = axes[0],  vy = axes[1],  vyaw = axes[3]  (joyToSportClientCmd)
// ════════════════════════════════════════════════════════════════════════════
bool testCS14_JoyAxesMoveConversion(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs14_server", 50'000'000LL});
    JoyTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager srcCsm(srcNode.get(), "cs14_source_csm");
    auto info = makeInfo("cs14/joy", TYPE_JOY, MODE_TOPIC, "cs14_server");
    if (!csmRegisterSource(srcCsm, info))
        FAIL("CS14", "registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* src = dynamic_cast<JoySource*>(
        srcCsm.getSource("cs14/joy").get());
    if (!src) FAIL("CS14", "getSource nullptr");

    bool ok;
    // axes[0]=0.7 (vx), axes[1]=0.2 (vy), axes[3]=0.3 (vyaw)
    const float VX = 0.7f, VY = 0.2f, VYAW = 0.3f;
    tracker.outputCount = 0;
    src->send(makeJoyAxes(VX, VY, VYAW), ok);
    rclcpp::sleep_for(300ms);

    {
        std::lock_guard<std::mutex> lk(tracker.mtx);
        if (tracker.outputCount == 0)
            FAIL("CS14", "outputCb not called");
        const auto& ax = tracker.lastOutputMsg.axes;
        if (static_cast<int>(ax.size()) < 4)
            FAIL("CS14", "captured Joy has fewer than 4 axes");
        // Direct mapping: axes[0]=vx, axes[1]=vy, axes[3]=vyaw (joyToSportClientCmd)
        if (std::abs(ax[JOY_AX_LEFT_H]  - VX)   > 1e-4f)
            FAIL("CS14", "axes[0] (vx) mismatch");
        if (std::abs(ax[JOY_AX_LEFT_V]  - VY)   > 1e-4f)
            FAIL("CS14", "axes[1] (vy) mismatch");
        if (std::abs(ax[JOY_AX_RIGHT_H] - VYAW) > 1e-4f)
            FAIL("CS14", "axes[2] (vyaw) mismatch");
    }

    PASS("CS14  Joy axes MOVE — axes captured; vx/vy/vyaw mapping verified");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS15 — Twist single source: outputCb fires; linear.x/y + angular.z verified
//         (mirrors twistToUnitreeRequest: vx=linear.x, vy=linear.y, vyaw=angular.z)
// ════════════════════════════════════════════════════════════════════════════
bool testCS15_TwistOutputCbConversion(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs15_server", 50'000'000LL});
    TwistTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager srcCsm(srcNode.get(), "cs15_source_csm");
    auto info = makeInfo("cs15/twist", TYPE_TWIST, MODE_TOPIC, "cs15_server");
    if (!csmRegisterSource(srcCsm, info))
        FAIL("CS15", "registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* src = dynamic_cast<TwistSource*>(
        srcCsm.getSource("cs15/twist").get());
    if (!src) FAIL("CS15", "getSource nullptr");

    bool ok;
    // Send full 3-DOF Twist; values map directly to unitree request:
    //   linear.x → vx,  linear.y → vy,  angular.z → vyaw
    const double VX = 2.5, VY = 0.4, VYAW = 1.2;
    tracker.outputCount = 0;
    src->send(makeTwistMove(VX, VY, VYAW), ok);
    rclcpp::sleep_for(300ms);

    {
        std::lock_guard<std::mutex> lk(tracker.mtx);
        if (tracker.outputCount == 0)
            FAIL("CS15", "outputCb not called");
        if (tracker.lastOutputChannel != "cs15/twist")
            FAIL("CS15", ("wrong output channel: " + tracker.lastOutputChannel).c_str());
        const auto& t = tracker.lastOutputMsg;
        if (std::abs(t.linear.x  - VX)   > 1e-6)
            FAIL("CS15", "linear.x mismatch");
        if (std::abs(t.linear.y  - VY)   > 1e-6)
            FAIL("CS15", "linear.y mismatch");
        if (std::abs(t.angular.z - VYAW) > 1e-6)
            FAIL("CS15", "angular.z mismatch");
    }

    PASS("CS15  Twist outputCb — linear.x/y and angular.z captured correctly");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS16 — Twist e-stop: emergencyStopCb fires; active Sink falls back
// ════════════════════════════════════════════════════════════════════════════
bool testCS16_TwistEmergencyStop(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNodeHigh,
    rclcpp::Node::SharedPtr srcNodeLow)
{
    ControlServer cs(csNode.get(), {"cs16_server", 50'000'000LL});
    TwistTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager csmHigh(srcNodeHigh.get(), "cs16_csm_high");
    ControlSignalManager csmLow (srcNodeLow.get(),  "cs16_csm_low");

    auto infoHigh = makeInfo("cs16/twist_high", TYPE_TWIST, MODE_TOPIC, "cs16_server",
                             PRI_HIGH);
    auto infoLow  = makeInfo("cs16/twist_low",  TYPE_TWIST, MODE_TOPIC, "cs16_server",
                             PRI_LOW);

    if (!csmRegisterSource(csmHigh, infoHigh)) FAIL("CS16", "high registerSource failed");
    if (!csmRegisterSource(csmLow,  infoLow))  FAIL("CS16", "low registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* srcHigh = dynamic_cast<TwistSource*>(
        csmHigh.getSource("cs16/twist_high").get());
    auto* srcLow  = dynamic_cast<TwistSource*>(
        csmLow.getSource("cs16/twist_low").get());
    if (!srcHigh || !srcLow) FAIL("CS16", "getSource nullptr");

    bool ok;
    srcHigh->send(makeTwistMove(1.0), ok);
    rclcpp::sleep_for(150ms);
    if (cs.getActiveSinkChannel<Twist>() != "cs16/twist_high")
        FAIL("CS16", "expected 'cs16/twist_high' as initial active");

    srcLow->send(makeTwistMove(0.2), ok);
    rclcpp::sleep_for(100ms);

    tracker.estopCount = 0;
    srcHigh->send(makeTwistEstop(), ok);
    rclcpp::sleep_for(300ms);

    if (tracker.estopCount.load() == 0)
        FAIL("CS16", "emergencyStopCb not called on Twist e-stop");

    auto active = cs.getActiveSinkChannel<Twist>();
    if (active != "cs16/twist_low")
        FAIL("CS16", ("expected fallback 'cs16/twist_low', got='" + active + "'").c_str());

    PASS("CS16  Twist e-stop — emergencyStopCb fired; active fell back to LOW Sink");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS17 — Twist request-active: HIGH promoted; LOW rejected (insufficient priority)
// ════════════════════════════════════════════════════════════════════════════
bool testCS17_TwistRequestActive(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNodeLow,
    rclcpp::Node::SharedPtr srcNodeHigh)
{
    ControlServer cs(csNode.get(), {"cs17_server", 50'000'000LL});
    TwistTracker tracker;
    cs.registerTypeConfig(tracker.makeConfig());

    ControlSignalManager csmLow (srcNodeLow.get(),  "cs17_csm_low");
    ControlSignalManager csmHigh(srcNodeHigh.get(), "cs17_csm_high");

    auto infoLow  = makeInfo("cs17/twist_low",  TYPE_TWIST, MODE_TOPIC, "cs17_server",
                             PRI_LOW);
    auto infoHigh = makeInfo("cs17/twist_high", TYPE_TWIST, MODE_TOPIC, "cs17_server",
                             PRI_HIGH);

    if (!csmRegisterSource(csmLow,  infoLow))  FAIL("CS17", "low registerSource failed");
    if (!csmRegisterSource(csmHigh, infoHigh)) FAIL("CS17", "high registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* srcLow  = dynamic_cast<TwistSource*>(
        csmLow.getSource("cs17/twist_low").get());
    auto* srcHigh = dynamic_cast<TwistSource*>(
        csmHigh.getSource("cs17/twist_high").get());
    if (!srcLow || !srcHigh) FAIL("CS17", "getSource nullptr");

    bool ok;
    // LOW sends first → becomes initial active
    srcLow->send(makeTwistMove(0.1), ok);
    rclcpp::sleep_for(200ms);
    if (cs.getActiveSinkChannel<Twist>() != "cs17/twist_low")
        FAIL("CS17", "expected 'cs17/twist_low' as initial active");

    // HIGH sends a normal message — not promoted (LOW is ACTIVE)
    srcHigh->send(makeTwistMove(0.5), ok);
    rclcpp::sleep_for(200ms);
    if (cs.getActiveSinkChannel<Twist>() != "cs17/twist_low")
        FAIL("CS17", "normal HIGH Twist must not steal active from ACTIVE LOW");

    // HIGH sends request-active → promoted (HIGH > LOW)
    srcHigh->send(makeTwistReqActive(), ok);
    rclcpp::sleep_for(200ms);
    if (cs.getActiveSinkChannel<Twist>() != "cs17/twist_high")
        FAIL("CS17", ("expected 'cs17/twist_high' after request-active, got='" +
                      cs.getActiveSinkChannel<Twist>() + "'").c_str());

    // LOW sends request-active → not promoted (LOW < HIGH)
    srcLow->send(makeTwistReqActive(), ok);
    rclcpp::sleep_for(200ms);
    if (cs.getActiveSinkChannel<Twist>() != "cs17/twist_high")
        FAIL("CS17", "LOW request-active must not override ACTIVE HIGH");

    PASS("CS17  Twist request-active — HIGH promoted; LOW rejected (insufficient priority)");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS18 — Joy + Twist simultaneous: both outputCbs fire; conversion values
//         verified for both types
//         Joy:   vx = axes[0],  vy = axes[1],  vyaw = axes[3]
//         Twist: vx = linear.x,  vy = linear.y,  vyaw = angular.z
// ════════════════════════════════════════════════════════════════════════════
bool testCS18_JoyTwistConversionValues(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs18_server", 50'000'000LL});
    JoyTracker   joyTracker;
    TwistTracker twistTracker;
    cs.registerTypeConfig(joyTracker.makeConfig());
    cs.registerTypeConfig(twistTracker.makeConfig());

    ControlSignalManager srcCsm(srcNode.get(), "cs18_source_csm");
    auto joyInfo = makeInfo("cs18/joy",   TYPE_JOY,   MODE_TOPIC, "cs18_server");
    auto twsInfo = makeInfo("cs18/twist", TYPE_TWIST, MODE_TOPIC, "cs18_server");
    if (!csmRegisterSource(srcCsm, joyInfo))
        FAIL("CS18", "joy registerSource failed");
    if (!csmRegisterSource(srcCsm, twsInfo))
        FAIL("CS18", "twist registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* joySrc = dynamic_cast<JoySource*>(
        srcCsm.getSource("cs18/joy").get());
    auto* twsSrc = dynamic_cast<TwistSource*>(
        srcCsm.getSource("cs18/twist").get());
    if (!joySrc || !twsSrc) FAIL("CS18", "getSource nullptr");

    bool ok;
    // Joy:   axes[0]=0.6 (vx), axes[1]=0.1 (vy), axes[3]=0.8 (vyaw)
    const float JVX=0.6f, JVY=0.1f, JVYAW=0.8f;
    joySrc->send(makeJoyAxes(JVX, JVY, JVYAW), ok);

    // Twist: linear.x=1.5, linear.y=0.25, angular.z=0.75
    //   → vx=1.5, vy=0.25, vyaw=0.75
    const double TVX=1.5, TVY=0.25, TVYAW=0.75;
    twsSrc->send(makeTwistMove(TVX, TVY, TVYAW), ok);
    rclcpp::sleep_for(400ms);

    // ── Verify Joy outputCb ───────────────────────────────────────────────────
    {
        std::lock_guard<std::mutex> lk(joyTracker.mtx);
        if (joyTracker.outputCount.load() == 0)
            FAIL("CS18", "Joy outputCb not called");
        const auto& ax = joyTracker.lastOutputMsg.axes;
        if (static_cast<int>(ax.size()) < 4)
            FAIL("CS18", "Joy: fewer than 4 axes captured");
        // Direct mapping: axes[0]=vx, axes[1]=vy, axes[3]=vyaw
        if (std::abs(ax[JOY_AX_LEFT_H]-JVX)>1e-4f ||
            std::abs(ax[JOY_AX_LEFT_V]-JVY)>1e-4f ||
            std::abs(ax[JOY_AX_RIGHT_H]-JVYAW)>1e-4f)
            FAIL("CS18", "Joy: captured axes mismatch");
    }

    // ── Verify Twist outputCb ─────────────────────────────────────────────────
    {
        std::lock_guard<std::mutex> lk(twistTracker.mtx);
        if (twistTracker.outputCount.load() == 0)
            FAIL("CS18", "Twist outputCb not called");
        const auto& t = twistTracker.lastOutputMsg;
        if (std::abs(t.linear.x  - TVX)   > 1e-6)
            FAIL("CS18", "Twist: linear.x mismatch");
        if (std::abs(t.linear.y  - TVY)   > 1e-6)
            FAIL("CS18", "Twist: linear.y mismatch");
        if (std::abs(t.angular.z - TVYAW) > 1e-6)
            FAIL("CS18", "Twist: angular.z mismatch");
    }

    PASS("CS18  Joy+Twist simultaneous — both outputCbs fired; conversion values verified");
    return true;
}

// ── CS19: Fallback to LOW_FREQ sink when ACTIVE is unavailable ────────────────
bool testCS19_FallbackToLowFreq(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs19_server", 50'000'000LL});
    JoyTracker    joyTracker;
    cs.registerTypeConfig(joyTracker.makeConfig());

    ControlSignalManager srcCsm(srcNode.get(), "cs19_source_csm");

    // Two Joy sinks: hi (HIGH priority) and md (MEDIUM priority), both at 10 Hz.
    auto hiInfo = makeInfo(
        "cs19/joy_hi",
        TYPE_JOY, MODE_TOPIC, "cs19_server",
        PRI_HIGH,
        2'000'000'000LL,  // timeout: 2 s
        false, 0,
        10.0f,            // send_freq_hz: 10 Hz (100 ms period)
        0);               // disconnect_timeout_ns
    auto mdInfo = makeInfo(
        "cs19/joy_md",
        TYPE_JOY, MODE_TOPIC, "cs19_server",
        PRI_MEDIUM,
        2'000'000'000LL,
        false, 0,
        10.0f,
        0);

    if (!csmRegisterSource(srcCsm, hiInfo))
        FAIL("CS19", "hi registerSource failed");
    if (!csmRegisterSource(srcCsm, mdInfo))
        FAIL("CS19", "md registerSource failed");
    rclcpp::sleep_for(300ms);   // topic discovery

    auto* hiSrc = dynamic_cast<JoySource*>(
        srcCsm.getSource("cs19/joy_hi").get());
    auto* mdSrc = dynamic_cast<JoySource*>(
        srcCsm.getSource("cs19/joy_md").get());
    if (!hiSrc || !mdSrc) FAIL("CS19", "getSource nullptr");

    bool ok;

    // Phase 1: send hi-priority message first and wait for it to be established as active.
    // By sending hi alone, its _onSinkMsg() fires before md's, so activeChannel = "cs19/joy_hi"
    // without any race with md's callback.
    hiSrc->send(makeJoy(0.5f), ok);
    rclcpp::sleep_for(50ms);    // 50ms: well within 100ms period, message delivered

    if (cs.getActiveSinkChannel<Joy>() != "cs19/joy_hi")
        FAIL("CS19", "expected hi to be selected initially");

    // Phase 2: send md-priority message.  By now hi is already the active channel;
    // md's _onSinkMsg() will see activeChannel = "cs19/joy_hi" and, since md has lower
    // priority (50 < 80), will not auto-promote — regardless of hi's current state.
    mdSrc->send(makeJoy(0.3f), ok);
    rclcpp::sleep_for(50ms);    // wait for md delivery

    // Phase 3: wait for both sinks to enter LOW_FREQ.
    // hi: ~100ms since last send (50 + 50ms) → already borderline; wait 100ms more → 200ms total
    // md: ~50ms since last send → wait 100ms more → 150ms total (> 100ms period → LOW_FREQ)
    rclcpp::sleep_for(100ms);

    // Both sinks should now be in LOW_FREQ; hi must remain selected (higher priority wins
    // even when both are LOW_FREQ — _selectBestLocked prefers hi, needFallback is false
    // for LOW_FREQ so activeChannel stays set to hi).
    if (cs.getActiveSinkChannel<Joy>() != "cs19/joy_hi")
        FAIL("CS19", "expected hi to remain selected even in LOW_FREQ due to higher priority");

    PASS("CS19  Fallback to LOW_FREQ — higher priority ACTIVE/LOW_FREQ selected over lower");
    return true;
}

// ── CS20: Auto-disconnect on disconnect_timeout_ns ─────────────────────────────
bool testCS20_AutoDisconnectTimeout(
    rclcpp::Node::SharedPtr tgtNode,
    rclcpp::Node::SharedPtr srcNode)
{
    // Use two direct CSMs (no ControlServer) so we can inspect tgtCsm's sinks.
    // statusTimerIntervalMs = 500 ms speeds up the disconnect detection.
    ControlSignalManager tgtCsm(tgtNode.get(), "cs20_tgt_csm", 500);
    ControlSignalManager srcCsm(srcNode.get(), "cs20_src_csm");

    // Joy source with:
    //   timeout_ns            = 500 ms
    //   disconnect_timeout_ns = 1000 ms (auto-disconnect after TIMEOUT > 1000 ms)
    auto info = makeInfo(
        "cs20/joy",
        TYPE_JOY, MODE_TOPIC, "cs20_tgt_csm",
        PRI_MEDIUM,
        500'000'000LL,     // timeout: 500 ms
        false, 0,
        0.0f,              // send_freq_hz: disabled
        1'000'000'000LL);  // disconnect_timeout_ns: 1 s

    if (!csmRegisterSource(srcCsm, info))
        FAIL("CS20", "registerSource failed");
    rclcpp::sleep_for(300ms);   // topic discovery

    auto* src = dynamic_cast<JoySource*>(
        srcCsm.getSource("cs20/joy").get());
    if (!src) FAIL("CS20", "getSource nullptr");

    // Send initial message → sink in tgtCsm becomes ACTIVE
    bool ok;
    src->send(makeJoy(0.5f), ok);
    rclcpp::sleep_for(100ms);

    auto sinkState = tgtCsm.getSinkState("cs20/joy");
    if (sinkState != ControlSignalState::ACTIVE)
        FAIL("CS20", ("expected sink ACTIVE, got " + std::string(stateName(sinkState))).c_str());

    // Stop sending; wait >500 ms → sink enters TIMEOUT
    rclcpp::sleep_for(700ms);
    sinkState = tgtCsm.getSinkState("cs20/joy");
    if (sinkState != ControlSignalState::TIMEOUT)
        FAIL("CS20", ("expected TIMEOUT after 700ms, got " + std::string(stateName(sinkState))).c_str());

    // CSM status timer (every 500 ms) removes TIMEOUT sinks after disconnect_timeout_ns (1000 ms).
    // Worst case: first TIMEOUT observed at T1, disconnect fires at T1+3×500ms = T1+1500ms.
    // Wait 3000 ms to give generous margin.
    rclcpp::sleep_for(3000ms);

    // Sink should either be erased (getSink returns nullptr) or marked DISCONNECTED.
    auto sinkPtr = tgtCsm.getSink("cs20/joy");
    if (sinkPtr)
    {
        auto finalState = sinkPtr->getState();
        if (finalState != ControlSignalState::DISCONNECTED)
            FAIL("CS20", ("expected DISCONNECTED, got " + std::string(stateName(finalState))).c_str());
    }
    // nullptr means the sink was already erased from the map — also correct.

    PASS("CS20  Auto-disconnect — sink enters TIMEOUT then DISCONNECTED after disconnect_timeout_ns");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  Joy/Twist → unitree_api Request integration helpers
//
//  Full output chain under test:
//    CSM Source → ControlServer routing → msgToOutSignal (joyToSportClientCmd /
//    Twist dedup) → SportClient → /api/sport/request (unitree_api::msg::Request)
// ════════════════════════════════════════════════════════════════════════════

/// Captures every unitree_api Request published on /api/sport/request.
struct RequestCapture
{
    std::vector<unitree_api::msg::Request> reqs;
    std::mutex mtx;
    rclcpp::Subscription<unitree_api::msg::Request>::SharedPtr sub;

    void attach(const rclcpp::Node::SharedPtr& node)
    {
        sub = node->create_subscription<unitree_api::msg::Request>(
            "/api/sport/request", rclcpp::QoS(100),
            [this](const unitree_api::msg::Request::SharedPtr m)
            {
                std::lock_guard<std::mutex> lk(mtx);
                reqs.push_back(*m);
            });
    }
    void clear()
    {
        std::lock_guard<std::mutex> lk(mtx);
        reqs.clear();
    }
    size_t total()
    {
        std::lock_guard<std::mutex> lk(mtx);
        return reqs.size();
    }
    size_t countApi(int64_t api_id)
    {
        std::lock_guard<std::mutex> lk(mtx);
        size_t n = 0;
        for (const auto& r : reqs)
            if (r.header.identity.api_id == api_id) ++n;
        return n;
    }
    /// Returns the parameter JSON of the LAST request with api_id, or null json.
    nlohmann::json lastParam(int64_t api_id)
    {
        std::lock_guard<std::mutex> lk(mtx);
        for (auto it = reqs.rbegin(); it != reqs.rend(); ++it)
            if (it->header.identity.api_id == api_id)
            {
                try { return nlohmann::json::parse(it->parameter); }
                catch (...) { return nlohmann::json(); }
            }
        return nlohmann::json();
    }
};

/// TypeConfig<Joy> wired exactly like control_server.cpp: real conversion + SportClient.
static ControlServer::TypeConfig<Joy> makeSportJoyConfig(SportClient& sc)
{
    ControlServer::TypeConfig<Joy> cfg;
    cfg.isEmergencyStop = joyIsEstop;
    cfg.isRequestActive = joyIsReqActive;
    cfg.outputCb = [&sc](const Joy& msg, const msg::ControlSignalInfo& info)
    {
        if (auto cmd = msgToOutSignal(info.channel_name, msg)) cmd(sc);
    };
    cfg.emergencyStopCb = [&sc](const msg::ControlSignalInfo&) { makeStopSignal()(sc); };
    return cfg;
}

static ControlServer::TypeConfig<Twist> makeSportTwistConfig(SportClient& sc)
{
    ControlServer::TypeConfig<Twist> cfg;
    cfg.isEmergencyStop = twistIsEstop;
    cfg.isRequestActive = twistIsReqActive;
    cfg.outputCb = [&sc](const Twist& msg, const msg::ControlSignalInfo& info)
    {
        if (auto cmd = msgToOutSignal(info.channel_name, msg)) cmd(sc);
    };
    cfg.emergencyStopCb = [&sc](const msg::ControlSignalInfo&) { makeStopSignal()(sc); };
    return cfg;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS21 — Joy buttons A/B/X/Y → StandUp/StandDown/StopMove/RecoveryStand
//         unitree_api Requests published with correct api_ids
// ════════════════════════════════════════════════════════════════════════════
bool testCS21_JoyButtonsToUnitreeRequest(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs21_server", 50'000'000LL});
    SportClient sport(csNode.get());
    RequestCapture cap;
    cap.attach(csNode);
    cs.registerTypeConfig(makeSportJoyConfig(sport));

    ControlSignalManager srcCsm(srcNode.get(), "cs21_source_csm");
    auto info = makeInfo("cs21/joy", TYPE_JOY, MODE_TOPIC, "cs21_server");
    if (!csmRegisterSource(srcCsm, info)) FAIL("CS21", "registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* src = dynamic_cast<JoySource*>(srcCsm.getSource("cs21/joy").get());
    if (!src) FAIL("CS21", "getSource nullptr");

    bool ok;
    // Activate the sink with a non-zero Move — must publish one Move request.
    src->send(makeJoyAxes(0.3f), ok);
    rclcpp::sleep_for(300ms);
    if (cs.getActiveSinkChannel<Joy>() != "cs21/joy")
        FAIL("CS21", "sink not active after initial MOVE message");
    if (cap.countApi(ROBOT_SPORT_API_ID_MOVE) == 0)
        FAIL("CS21", "activation Move request not published to /api/sport/request");
    cap.clear();

    struct Case { uint8_t btn; int32_t api; const char* name; };
    const Case cases[] = {
        {JOY_BTN_A, ROBOT_SPORT_API_ID_STANDUP,       "A→StandUp"},
        {JOY_BTN_B, ROBOT_SPORT_API_ID_STANDDOWN,     "B→StandDown"},
        {JOY_BTN_X, ROBOT_SPORT_API_ID_STOPMOVE,      "X→StopMove"},
        {JOY_BTN_Y, ROBOT_SPORT_API_ID_RECOVERYSTAND, "Y→RecoveryStand"},
    };
    for (const auto& c : cases)
    {
        src->send(makeJoyButton(c.btn), ok);   // press
        rclcpp::sleep_for(150ms);
        src->send(makeJoyButton(-1), ok);      // release
        rclcpp::sleep_for(150ms);
    }

    for (const auto& c : cases)
    {
        const size_t n = cap.countApi(c.api);
        if (n != 1)
            FAIL("CS21", (std::string(c.name) + ": expected exactly 1 request, got " +
                          std::to_string(n)).c_str());
    }
    // Exactly one zero-Move after the first release (axes returned 0,0,0);
    // subsequent identical zero axes must be deduplicated.
    if (cap.countApi(ROBOT_SPORT_API_ID_MOVE) != 1)
        FAIL("CS21", ("expected exactly 1 zero-Move during button sequence, got " +
                      std::to_string(cap.countApi(ROBOT_SPORT_API_ID_MOVE))).c_str());

    PASS("CS21  Joy buttons A/B/X/Y → StandUp/StandDown/StopMove/RecoveryStand requests");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS22 — PRESS edge: held button fires exactly once; re-press fires again
// ════════════════════════════════════════════════════════════════════════════
bool testCS22_ButtonPressEdgeOnce(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs22_server", 50'000'000LL});
    SportClient sport(csNode.get());
    RequestCapture cap;
    cap.attach(csNode);
    cs.registerTypeConfig(makeSportJoyConfig(sport));

    ControlSignalManager srcCsm(srcNode.get(), "cs22_source_csm");
    auto info = makeInfo("cs22/joy", TYPE_JOY, MODE_TOPIC, "cs22_server");
    if (!csmRegisterSource(srcCsm, info)) FAIL("CS22", "registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* src = dynamic_cast<JoySource*>(srcCsm.getSource("cs22/joy").get());
    if (!src) FAIL("CS22", "getSource nullptr");

    bool ok;
    // Hold A across 3 consecutive messages → PRESS edge only on the first.
    for (int i = 0; i < 3; ++i)
    {
        src->send(makeJoyButton(JOY_BTN_A), ok);
        rclcpp::sleep_for(120ms);
    }
    if (cap.countApi(ROBOT_SPORT_API_ID_STANDUP) != 1)
        FAIL("CS22", ("held A: expected exactly 1 StandUp, got " +
                      std::to_string(cap.countApi(ROBOT_SPORT_API_ID_STANDUP))).c_str());

    // Release, then press again → second PRESS edge → second StandUp.
    src->send(makeJoyButton(-1), ok);
    rclcpp::sleep_for(120ms);
    src->send(makeJoyButton(JOY_BTN_A), ok);
    rclcpp::sleep_for(200ms);

    if (cap.countApi(ROBOT_SPORT_API_ID_STANDUP) != 2)
        FAIL("CS22", ("re-press A: expected 2 StandUp total, got " +
                      std::to_string(cap.countApi(ROBOT_SPORT_API_ID_STANDUP))).c_str());

    PASS("CS22  PRESS edge — held button fired once; re-press fired again");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS23 — Move dedup: identical (vx,vy,vyaw) not re-sent; each change sent once
//         with correct x/y/z parameters
// ════════════════════════════════════════════════════════════════════════════
bool testCS23_MoveDedup(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs23_server", 50'000'000LL});
    SportClient sport(csNode.get());
    RequestCapture cap;
    cap.attach(csNode);
    cs.registerTypeConfig(makeSportJoyConfig(sport));

    ControlSignalManager srcCsm(srcNode.get(), "cs23_source_csm");
    auto info = makeInfo("cs23/joy", TYPE_JOY, MODE_TOPIC, "cs23_server");
    if (!csmRegisterSource(srcCsm, info)) FAIL("CS23", "registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* src = dynamic_cast<JoySource*>(srcCsm.getSource("cs23/joy").get());
    if (!src) FAIL("CS23", "getSource nullptr");

    bool ok;
    // Phase 1: identical Move repeated 3× → exactly 1 Move request.
    const float VX = 0.5f, VY = 0.2f, VYAW = 0.3f;
    for (int i = 0; i < 3; ++i)
    {
        src->send(makeJoyAxes(VX, VY, VYAW), ok);
        rclcpp::sleep_for(120ms);
    }
    if (cap.countApi(ROBOT_SPORT_API_ID_MOVE) != 1)
        FAIL("CS23", ("identical axes ×3: expected 1 Move, got " +
                      std::to_string(cap.countApi(ROBOT_SPORT_API_ID_MOVE))).c_str());

    // Verify the Move parameters (SportClient::Move encodes x/y/z JSON keys).
    {
        auto js = cap.lastParam(ROBOT_SPORT_API_ID_MOVE);
        if (js.is_null()) FAIL("CS23", "Move request parameter missing/unparsable");
        if (std::abs(js.value("x", 0.0f) - VX)   > 1e-4f ||
            std::abs(js.value("y", 0.0f) - VY)   > 1e-4f ||
            std::abs(js.value("z", 0.0f) - VYAW) > 1e-4f)
            FAIL("CS23", "Move parameter x/y/z mismatch");
    }

    // Phase 2: change vx → second Move.
    src->send(makeJoyAxes(0.8f, VY, VYAW), ok);
    rclcpp::sleep_for(200ms);
    if (cap.countApi(ROBOT_SPORT_API_ID_MOVE) != 2)
        FAIL("CS23", "changed vx: expected 2nd Move request");

    // Phase 3: return to zero → third Move (0,0,0), then repeated zeros deduped.
    for (int i = 0; i < 3; ++i)
    {
        src->send(makeJoyAxes(0.0f, 0.0f, 0.0f), ok);
        rclcpp::sleep_for(120ms);
    }
    if (cap.countApi(ROBOT_SPORT_API_ID_MOVE) != 3)
        FAIL("CS23", ("zeros: expected 3 Moves total (one final zero Move), got " +
                      std::to_string(cap.countApi(ROBOT_SPORT_API_ID_MOVE))).c_str());
    {
        auto js = cap.lastParam(ROBOT_SPORT_API_ID_MOVE);
        if (std::abs(js.value("x", -1.0f)) > 1e-6f ||
            std::abs(js.value("y", -1.0f)) > 1e-6f ||
            std::abs(js.value("z", -1.0f)) > 1e-6f)
            FAIL("CS23", "final Move must be (0,0,0)");
    }

    PASS("CS23  Move dedup — identical axes suppressed; each change sent exactly once");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS24 — Idle joystick (all zeros) from startup → ZERO unitree requests
//         while messages keep flowing (sink stays active)
// ════════════════════════════════════════════════════════════════════════════
bool testCS24_IdleJoyNoOutput(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs24_server", 50'000'000LL});
    SportClient sport(csNode.get());
    RequestCapture cap;
    cap.attach(csNode);
    cs.registerTypeConfig(makeSportJoyConfig(sport));

    ControlSignalManager srcCsm(srcNode.get(), "cs24_source_csm");
    auto info = makeInfo("cs24/joy", TYPE_JOY, MODE_TOPIC, "cs24_server");
    if (!csmRegisterSource(srcCsm, info)) FAIL("CS24", "registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* src = dynamic_cast<JoySource*>(srcCsm.getSource("cs24/joy").get());
    if (!src) FAIL("CS24", "getSource nullptr");

    bool ok;
    // Idle joystick stream: all axes 0, no buttons, 10 messages @ ~10 Hz.
    for (int i = 0; i < 10; ++i)
    {
        src->send(makeJoyAxes(0.0f, 0.0f, 0.0f), ok);
        rclcpp::sleep_for(100ms);
    }

    // Messages flowed (sink selected as active)...
    if (cs.getActiveSinkChannel<Joy>() != "cs24/joy")
        FAIL("CS24", "sink not active — idle messages did not flow");
    // ...but not a single unitree request was published.
    if (cap.total() != 0)
        FAIL("CS24", ("idle joystick published " + std::to_string(cap.total()) +
                      " requests; expected 0").c_str());

    PASS("CS24  Idle joystick — messages flow, zero unitree requests published");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS25 — Twist Move dedup: identical Twist not re-sent; change sent once
// ════════════════════════════════════════════════════════════════════════════
bool testCS25_TwistMoveDedup(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs25_server", 50'000'000LL});
    SportClient sport(csNode.get());
    RequestCapture cap;
    cap.attach(csNode);
    cs.registerTypeConfig(makeSportTwistConfig(sport));

    ControlSignalManager srcCsm(srcNode.get(), "cs25_source_csm");
    auto info = makeInfo("cs25/twist", TYPE_TWIST, MODE_TOPIC, "cs25_server");
    if (!csmRegisterSource(srcCsm, info)) FAIL("CS25", "registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* src = dynamic_cast<TwistSource*>(srcCsm.getSource("cs25/twist").get());
    if (!src) FAIL("CS25", "getSource nullptr");

    bool ok;
    // Identical Twist ×3 → exactly 1 Move.
    for (int i = 0; i < 3; ++i)
    {
        src->send(makeTwistMove(1.0, 0.5, 0.2), ok);
        rclcpp::sleep_for(120ms);
    }
    if (cap.countApi(ROBOT_SPORT_API_ID_MOVE) != 1)
        FAIL("CS25", ("identical Twist ×3: expected 1 Move, got " +
                      std::to_string(cap.countApi(ROBOT_SPORT_API_ID_MOVE))).c_str());
    {
        auto js = cap.lastParam(ROBOT_SPORT_API_ID_MOVE);
        if (std::abs(js.value("x", 0.0f) - 1.0f) > 1e-4f ||
            std::abs(js.value("y", 0.0f) - 0.5f) > 1e-4f ||
            std::abs(js.value("z", 0.0f) - 0.2f) > 1e-4f)
            FAIL("CS25", "Twist Move parameter x/y/z mismatch");
    }

    // Changed Twist → second Move.
    src->send(makeTwistMove(2.0, 0.5, 0.2), ok);
    rclcpp::sleep_for(200ms);
    if (cap.countApi(ROBOT_SPORT_API_ID_MOVE) != 2)
        FAIL("CS25", "changed Twist: expected 2nd Move request");

    PASS("CS25  Twist Move dedup — identical Twist suppressed; change sent once");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS26 — Joy e-stop content → StopMove request published on /api/sport/request
// ════════════════════════════════════════════════════════════════════════════
bool testCS26_EstopPublishesStopMove(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs26_server", 50'000'000LL});
    SportClient sport(csNode.get());
    RequestCapture cap;
    cap.attach(csNode);
    cs.registerTypeConfig(makeSportJoyConfig(sport));

    ControlSignalManager srcCsm(srcNode.get(), "cs26_source_csm");
    auto info = makeInfo("cs26/joy", TYPE_JOY, MODE_TOPIC, "cs26_server");
    if (!csmRegisterSource(srcCsm, info)) FAIL("CS26", "registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* src = dynamic_cast<JoySource*>(srcCsm.getSource("cs26/joy").get());
    if (!src) FAIL("CS26", "getSource nullptr");

    bool ok;
    src->send(makeJoyAxes(0.4f), ok);   // activate
    rclcpp::sleep_for(200ms);
    if (cs.getActiveSinkChannel<Joy>() != "cs26/joy")
        FAIL("CS26", "sink not active");
    cap.clear();

    src->send(makeJoyEstop(), ok);
    rclcpp::sleep_for(300ms);

    // emergencyStopCb → makeStopSignal → StopMove. The watchdog may re-fire the
    // e-stop while no fallback sink exists, so require >= 1 rather than == 1.
    if (cap.countApi(ROBOT_SPORT_API_ID_STOPMOVE) == 0)
        FAIL("CS26", "e-stop did not publish a StopMove request");

    PASS("CS26  Joy e-stop — StopMove request published on /api/sport/request");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  CS27 — Per-axis Move: vx / vy / vyaw individually and combined (incl.
//         negative values) map to the correct Move x/y/z parameters;
//         unused axes (right stick vertical, L2 trigger) produce no output
// ════════════════════════════════════════════════════════════════════════════
bool testCS27_PerAxisMove(
    rclcpp::Node::SharedPtr csNode,
    rclcpp::Node::SharedPtr srcNode)
{
    ControlServer cs(csNode.get(), {"cs27_server", 50'000'000LL});
    SportClient sport(csNode.get());
    RequestCapture cap;
    cap.attach(csNode);
    cs.registerTypeConfig(makeSportJoyConfig(sport));

    ControlSignalManager srcCsm(srcNode.get(), "cs27_source_csm");
    auto info = makeInfo("cs27/joy", TYPE_JOY, MODE_TOPIC, "cs27_server");
    if (!csmRegisterSource(srcCsm, info)) FAIL("CS27", "registerSource failed");
    rclcpp::sleep_for(300ms);

    auto* src = dynamic_cast<JoySource*>(srcCsm.getSource("cs27/joy").get());
    if (!src) FAIL("CS27", "getSource nullptr");

    bool ok;
    struct AxCase { float vx, vy, vyaw; const char* name; };
    const AxCase cases[] = {
        {0.5f,  0.0f,  0.0f, "vx only (left H)"},
        {0.0f,  0.4f,  0.0f, "vy only (left V)"},
        {0.0f,  0.0f, -0.7f, "vyaw only, negative (right H)"},
        {0.3f, -0.2f,  0.6f, "combined vx/vy/vyaw"},
    };

    size_t expectedMoves = 0;
    for (const auto& c : cases)
    {
        src->send(makeJoyAxes(c.vx, c.vy, c.vyaw), ok);
        rclcpp::sleep_for(200ms);
        ++expectedMoves;

        if (cap.countApi(ROBOT_SPORT_API_ID_MOVE) != expectedMoves)
            FAIL("CS27", (std::string(c.name) + ": expected " +
                          std::to_string(expectedMoves) + " Moves total, got " +
                          std::to_string(cap.countApi(ROBOT_SPORT_API_ID_MOVE))).c_str());

        auto js = cap.lastParam(ROBOT_SPORT_API_ID_MOVE);
        if (js.is_null())
            FAIL("CS27", (std::string(c.name) + ": Move parameter unparsable").c_str());
        if (std::abs(js.value("x", -99.0f) - c.vx)   > 1e-4f ||
            std::abs(js.value("y", -99.0f) - c.vy)   > 1e-4f ||
            std::abs(js.value("z", -99.0f) - c.vyaw) > 1e-4f)
            FAIL("CS27", (std::string(c.name) + ": Move x/y/z mismatch").c_str());
    }

    // Unused axes: wiggle right stick vertical and press L2 while the mapped
    // sticks hold the last values → values unchanged → NO new request.
    const auto& lastC = cases[3];
    cap.clear();
    src->send(makeJoyAxesFull(lastC.vx, lastC.vy, lastC.vyaw,
                              /*rightV=*/0.9f, /*l2=*/-1.0f), ok);
    rclcpp::sleep_for(200ms);
    src->send(makeJoyAxesFull(lastC.vx, lastC.vy, lastC.vyaw,
                              /*rightV=*/-0.5f, /*l2=*/-1.0f), ok);
    rclcpp::sleep_for(200ms);
    if (cap.total() != 0)
        FAIL("CS27", ("unused axes (right V, L2) produced " +
                      std::to_string(cap.total()) + " requests; expected 0").c_str());

    PASS("CS27  Per-axis Move — vx/vy/vyaw individual+combined verified; unused axes silent");
    return true;
}


// ════════════════════════════════════════════════════════════════════════════
//  main
// ════════════════════════════════════════════════════════════════════════════
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Each test uses its own dedicated nodes to avoid topic/service name collisions
    auto makeNode = [](const char* name)
    {
        return rclcpp::Node::make_shared(name);
    };

    // CS1
    auto n_cs1  = makeNode("n_cs1_cs");
    auto n_cs1s = makeNode("n_cs1_src");

    // CS2
    auto n_cs2   = makeNode("n_cs2_cs");
    auto n_cs2s  = makeNode("n_cs2_src");

    // CS3
    auto n_cs3   = makeNode("n_cs3_cs");
    auto n_cs3lo = makeNode("n_cs3_lo");
    auto n_cs3hi = makeNode("n_cs3_hi");

    // CS4
    auto n_cs4   = makeNode("n_cs4_cs");
    auto n_cs4a  = makeNode("n_cs4_a");
    auto n_cs4b  = makeNode("n_cs4_b");

    // CS5
    auto n_cs5   = makeNode("n_cs5_cs");
    auto n_cs5hi = makeNode("n_cs5_hi");
    auto n_cs5lo = makeNode("n_cs5_lo");

    // CS6
    auto n_cs6   = makeNode("n_cs6_cs");
    auto n_cs6hi = makeNode("n_cs6_hi");
    auto n_cs6lo = makeNode("n_cs6_lo");

    // CS7
    auto n_cs7   = makeNode("n_cs7_cs");
    auto n_cs7lo = makeNode("n_cs7_lo");
    auto n_cs7hi = makeNode("n_cs7_hi");

    // CS8
    auto n_cs8   = makeNode("n_cs8_cs");
    auto n_cs8n  = makeNode("n_cs8_normal");
    auto n_cs8e  = makeNode("n_cs8_estop");

    // CS9
    auto n_cs9   = makeNode("n_cs9_cs");
    auto n_cs9a  = makeNode("n_cs9_a");
    auto n_cs9b  = makeNode("n_cs9_b");

    // CS10
    auto n_cs10  = makeNode("n_cs10_cs");
    auto n_cs10s = makeNode("n_cs10_src");

    // CS11
    auto n_cs11  = makeNode("n_cs11_cs");
    auto n_cs11s = makeNode("n_cs11_src");

    // CS12
    auto n_cs12   = makeNode("n_cs12_cs");
    auto n_cs12hi = makeNode("n_cs12_hi");
    auto n_cs12lo = makeNode("n_cs12_lo");

    // CS13 — Joy button commands
    auto n_cs13  = makeNode("n_cs13_cs");
    auto n_cs13s = makeNode("n_cs13_src");

    // CS14 — Joy axes MOVE conversion
    auto n_cs14  = makeNode("n_cs14_cs");
    auto n_cs14s = makeNode("n_cs14_src");

    // CS15 — Twist outputCb conversion
    auto n_cs15  = makeNode("n_cs15_cs");
    auto n_cs15s = makeNode("n_cs15_src");

    // CS16 — Twist e-stop
    auto n_cs16   = makeNode("n_cs16_cs");
    auto n_cs16hi = makeNode("n_cs16_hi");
    auto n_cs16lo = makeNode("n_cs16_lo");

    // CS17 — Twist request-active
    auto n_cs17   = makeNode("n_cs17_cs");
    auto n_cs17lo = makeNode("n_cs17_lo");
    auto n_cs17hi = makeNode("n_cs17_hi");

    // CS18 — Joy+Twist simultaneous conversion values
    auto n_cs18  = makeNode("n_cs18_cs");
    auto n_cs18s = makeNode("n_cs18_src");

    // CS19 — LOW_FREQ fallback
    auto n_cs19   = makeNode("n_cs19_cs");
    auto n_cs19s  = makeNode("n_cs19_src");

    // CS20 — Auto-disconnect on timeout
    auto n_cs20   = makeNode("n_cs20_cs");
    auto n_cs20s  = makeNode("n_cs20_src");

    // CS21–CS26 — Joy/Twist → unitree_api Request integration
    auto n_cs21  = makeNode("n_cs21_cs");
    auto n_cs21s = makeNode("n_cs21_src");
    auto n_cs22  = makeNode("n_cs22_cs");
    auto n_cs22s = makeNode("n_cs22_src");
    auto n_cs23  = makeNode("n_cs23_cs");
    auto n_cs23s = makeNode("n_cs23_src");
    auto n_cs24  = makeNode("n_cs24_cs");
    auto n_cs24s = makeNode("n_cs24_src");
    auto n_cs25  = makeNode("n_cs25_cs");
    auto n_cs25s = makeNode("n_cs25_src");
    auto n_cs26  = makeNode("n_cs26_cs");
    auto n_cs26s = makeNode("n_cs26_src");
    auto n_cs27  = makeNode("n_cs27_cs");
    auto n_cs27s = makeNode("n_cs27_src");

    rclcpp::executors::MultiThreadedExecutor exec;
    for (auto& n : {n_cs1, n_cs1s,
                    n_cs2, n_cs2s,
                    n_cs3, n_cs3lo, n_cs3hi,
                    n_cs4, n_cs4a, n_cs4b,
                    n_cs5, n_cs5hi, n_cs5lo,
                    n_cs6, n_cs6hi, n_cs6lo,
                    n_cs7, n_cs7lo, n_cs7hi,
                    n_cs8, n_cs8n, n_cs8e,
                    n_cs9, n_cs9a, n_cs9b,
                    n_cs10, n_cs10s,
                    n_cs11, n_cs11s,
                    n_cs12, n_cs12hi, n_cs12lo,
                    n_cs13, n_cs13s,
                    n_cs14, n_cs14s,
                    n_cs15, n_cs15s,
                    n_cs16, n_cs16hi, n_cs16lo,
                    n_cs17, n_cs17lo, n_cs17hi,
                    n_cs18, n_cs18s,
                    n_cs19, n_cs19s,
                    n_cs20, n_cs20s,
                    n_cs21, n_cs21s,
                    n_cs22, n_cs22s,
                    n_cs23, n_cs23s,
                    n_cs24, n_cs24s,
                    n_cs25, n_cs25s,
                    n_cs26, n_cs26s,
                    n_cs27, n_cs27s})
        exec.add_node(n);

    std::thread execThread([&exec]() { exec.spin(); });

    RCLCPP_INFO(rclcpp::get_logger("test_cs"), "=== ControlServer Integration Tests ===");

    int passed = 0, failed = 0;
    auto run = [&](bool ok) { ok ? ++passed : ++failed; };

    run(testCS1_SingleTypeFirstSinkActive  (n_cs1,  n_cs1s));
    run(testCS2_MultipleTypeConfigs        (n_cs2,  n_cs2s));
    run(testCS3_PriorityAutoSelect         (n_cs3,  n_cs3lo, n_cs3hi));
    run(testCS4_ManualActiveSinkOverride   (n_cs4,  n_cs4a,  n_cs4b));
    run(testCS5_FallbackOnTimeout          (n_cs5,  n_cs5hi, n_cs5lo));
    run(testCS6_EmergencyStop              (n_cs6,  n_cs6hi, n_cs6lo));
    run(testCS7_RequestActive              (n_cs7,  n_cs7lo, n_cs7hi));
    run(testCS8_EmergencyStopPrioritySink  (n_cs8,  n_cs8n,  n_cs8e));
    run(testCS9_OutputCbActiveOnly         (n_cs9,  n_cs9a,  n_cs9b));
    run(testCS10_NoActiveSink              (n_cs10, n_cs10s));
    run(testCS11_ServiceModeOutputCb       (n_cs11, n_cs11s));
    run(testCS12_ServiceModeEmergencyStop  (n_cs12, n_cs12hi, n_cs12lo));
    run(testCS13_JoyButtonCommands         (n_cs13, n_cs13s));
    run(testCS14_JoyAxesMoveConversion     (n_cs14, n_cs14s));
    run(testCS15_TwistOutputCbConversion   (n_cs15, n_cs15s));
    run(testCS16_TwistEmergencyStop        (n_cs16, n_cs16hi, n_cs16lo));
    run(testCS17_TwistRequestActive        (n_cs17, n_cs17lo, n_cs17hi));
    run(testCS18_JoyTwistConversionValues  (n_cs18, n_cs18s));
    run(testCS19_FallbackToLowFreq         (n_cs19, n_cs19s));
    run(testCS20_AutoDisconnectTimeout     (n_cs20, n_cs20s));
    run(testCS21_JoyButtonsToUnitreeRequest(n_cs21, n_cs21s));
    run(testCS22_ButtonPressEdgeOnce       (n_cs22, n_cs22s));
    run(testCS23_MoveDedup                 (n_cs23, n_cs23s));
    run(testCS24_IdleJoyNoOutput           (n_cs24, n_cs24s));
    run(testCS25_TwistMoveDedup            (n_cs25, n_cs25s));
    run(testCS26_EstopPublishesStopMove    (n_cs26, n_cs26s));
    run(testCS27_PerAxisMove               (n_cs27, n_cs27s));

    RCLCPP_INFO(rclcpp::get_logger("test_cs"),
        "=== Results: %d passed, %d failed ===", passed, failed);

    exec.cancel();
    execThread.join();
    rclcpp::shutdown();
    return (failed == 0) ? 0 : 1;
}
