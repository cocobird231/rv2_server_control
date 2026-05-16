/**
 * Manual integration test for ControlSignalSource, ControlSignalSink, and ControlSignalManager.
 *
 * Each test function prints [PASS] or [FAIL] and returns bool.
 * The executor is running in a background thread for the whole duration so that
 * ROS 2 topic/service callbacks are dispatched while the main thread blocks.
 *
 * Test coverage:
 *  T1  Source topic mode            → initial state is UNKNOWN
 *  T2  Sink topic mode              → UNKNOWN → ACTIVE on first message received
 *  T3  Source service mode          → UNKNOWN → ACTIVE after successful send()
 *  T4  Sink timeout                 → ACTIVE → TIMEOUT after timeout_ns elapses
 *  T5  Keep-alive heartbeat         → Source becomes ACTIVE via Sink's wall-timer
 *  T6  CSM registerSource           → remote Sink is created; duplicate rejected
 *  T7  CSM control_signal_info_req  → source_list / sink_list are correct
 */

#include <thread>
#include <string>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rv2_server_control/control_signal_manager.h"

using namespace rv2_interfaces;
using namespace std::chrono_literals;

// ── Helpers ───────────────────────────────────────────────────────────────────

static const char* stateName(ControlSignalState s)
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

static msg::ControlSignalInfo makeInfo(
    const std::string& channel,
    const std::string& type,
    const std::string& mode,
    const std::string& targetCsm        = "",
    bool               useKeepAlive     = false,
    int64_t            keepAliveNs      = 300'000'000LL,   // 300 ms
    int64_t            timeoutNs        = 2'000'000'000LL,  // 2 s
    float              sendFreqHz       = 0.0f,             // 0 = disabled
    int64_t            disconnectTimeoutNs = 0              // 0 = no auto-disconnect
)
{
    msg::ControlSignalInfo info;
    info.channel_name           = channel;
    info.control_signal_type    = type;
    info.control_signal_mode    = mode;
    info.target_csm_name        = targetCsm;
    info.use_keep_alive         = useKeepAlive;
    info.keep_alive_interval_ns = keepAliveNs;
    info.send_freq_hz           = sendFreqHz;
    info.timeout_ns             = timeoutNs;
    info.disconnect_timeout_ns  = disconnectTimeoutNs;
    return info;
}

#define PASS(name)         RCLCPP_INFO (rclcpp::get_logger("test"), "[PASS] %s", name)
#define FAIL(name, reason) do { RCLCPP_ERROR(rclcpp::get_logger("test"), "[FAIL] " name ": %s", reason); return false; } while(0)

// ── T1: Source topic mode — always UNKNOWN ────────────────────────────────────
bool testSourceTopicUnknown(rclcpp::Node::SharedPtr node)
{
    auto info = makeInfo(
        "t1/joy",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC);

    ControlSignalSource<sensor_msgs::msg::Joy> src(node.get(), info);

    auto s = src.getState();
    if (s != ControlSignalState::UNKNOWN)
    {
        FAIL("T1", ("expected UNKNOWN, got " + std::string(stateName(s))).c_str());
    }
    PASS("T1  Source topic mode — initial UNKNOWN");
    return true;
}

// ── T2: Sink topic mode — UNKNOWN → ACTIVE on first message ──────────────────
bool testSinkTopicActiveOnReceive(rclcpp::Node::SharedPtr nodeA, rclcpp::Node::SharedPtr nodeB)
{
    auto info = makeInfo(
        "t2/joy",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC);

    ControlSignalSink  <sensor_msgs::msg::Joy> sink(nodeA.get(), info);
    ControlSignalSource<sensor_msgs::msg::Joy> src (nodeB.get(), info);

    if (sink.getState() != ControlSignalState::UNKNOWN)
    {
        FAIL("T2", ("expected initial UNKNOWN, got " + std::string(stateName(sink.getState()))).c_str());
    }

    rclcpp::sleep_for(300ms);   // wait for topic discovery

    bool cmdOk = false;
    sensor_msgs::msg::Joy joy;
    joy.axes = {0.5f, -0.5f};
    src.send(joy, cmdOk);

    rclcpp::sleep_for(200ms);   // wait for delivery

    auto s = sink.getState();
    if (s != ControlSignalState::ACTIVE)
    {
        FAIL("T2", ("expected ACTIVE after receive, got " + std::string(stateName(s))).c_str());
    }

    sensor_msgs::msg::Joy out;
    bool readOk = sink.read(out);
    if (!readOk || out.axes.empty() || out.axes[0] != 0.5f)
    {
        FAIL("T2", "read() returned wrong data or false");
    }

    PASS("T2  Sink topic mode — UNKNOWN→ACTIVE on receive");
    return true;
}

// ── T3: Source service mode — UNKNOWN → ACTIVE after successful send() ────────
bool testSourceServiceMode(rclcpp::Node::SharedPtr nodeA, rclcpp::Node::SharedPtr nodeB)
{
    // 500 ms request timeout; no keep-alive
    auto info = makeInfo(
        "t3/joy_svc",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_SERVICE,
        "", false, 0, 500'000'000LL);

    // Sink  = service server on nodeA
    // Source = service client on nodeB
    ControlSignalSink  <sensor_msgs::msg::Joy, srv::ControlSignalJoy> sink(nodeA.get(), info);
    ControlSignalSource<sensor_msgs::msg::Joy, srv::ControlSignalJoy> src (nodeB.get(), info);

    if (src.getState() != ControlSignalState::UNKNOWN)
    {
        FAIL("T3", "expected initial UNKNOWN for service source");
    }

    rclcpp::sleep_for(300ms);   // wait for service to be ready

    bool cmdOk = false;
    sensor_msgs::msg::Joy joy;
    joy.axes = {1.0f};
    bool sent = src.send(joy, cmdOk);
    if (!sent || !cmdOk)
    {
        FAIL("T3", ("send() failed — sent=" + std::to_string(sent) +
                    " cmdOk=" + std::to_string(cmdOk)).c_str());
    }

    auto srcState  = src.getState();
    auto sinkState = sink.getState();

    if (srcState != ControlSignalState::ACTIVE)
    {
        FAIL("T3", ("expected source ACTIVE after send, got " + std::string(stateName(srcState))).c_str());
    }
    if (sinkState != ControlSignalState::ACTIVE)
    {
        FAIL("T3", ("expected sink ACTIVE after service call, got " + std::string(stateName(sinkState))).c_str());
    }

    PASS("T3  Source service mode — UNKNOWN→ACTIVE after send()");
    return true;
}

// ── T4: Sink timeout — ACTIVE → TIMEOUT after timeout_ns ─────────────────────
bool testSinkTimeout(rclcpp::Node::SharedPtr nodeA, rclcpp::Node::SharedPtr nodeB)
{
    // 400 ms timeout
    auto info = makeInfo(
        "t4/twist",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC,
        "", false, 0, 400'000'000LL);

    ControlSignalSink  <geometry_msgs::msg::Twist> sink(nodeA.get(), info);
    ControlSignalSource<geometry_msgs::msg::Twist> src (nodeB.get(), info);

    rclcpp::sleep_for(300ms);   // topic discovery

    bool cmdOk = false;
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 1.0;
    src.send(twist, cmdOk);

    rclcpp::sleep_for(150ms);   // wait for delivery

    if (sink.getState() != ControlSignalState::ACTIVE)
    {
        FAIL("T4", "expected ACTIVE after first message");
    }

    // Wait longer than timeout_ns without sending → TIMEOUT
    rclcpp::sleep_for(600ms);

    auto s = sink.getState();
    if (s != ControlSignalState::TIMEOUT)
    {
        FAIL("T4", ("expected TIMEOUT, got " + std::string(stateName(s))).c_str());
    }

    PASS("T4  Sink timeout — ACTIVE→TIMEOUT after timeout_ns");
    return true;
}

// ── T5: Keep-alive — Source becomes ACTIVE from Sink's heartbeat ──────────────
bool testKeepAlive(rclcpp::Node::SharedPtr nodeA, rclcpp::Node::SharedPtr nodeB)
{
    // keep_alive_interval = 250 ms → threshold = 500 ms
    auto info = makeInfo(
        "t5/joy_ka",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC,
        "", true, 250'000'000LL, 2'000'000'000LL);

    // Sink on nodeA  → creates keep-alive publisher + wall timer
    // Source on nodeB → creates keep-alive subscriber
    ControlSignalSink  <sensor_msgs::msg::Joy> sink(nodeA.get(), info);
    ControlSignalSource<sensor_msgs::msg::Joy> src (nodeB.get(), info);

    if (src.getState() != ControlSignalState::UNKNOWN)
    {
        FAIL("T5", "expected initial UNKNOWN before keep-alive");
    }

    // Wait for at least 3 heartbeat ticks (≥ 750 ms) plus topic discovery
    rclcpp::sleep_for(1100ms);

    auto s = src.getState();
    if (s != ControlSignalState::ACTIVE)
    {
        FAIL("T5", ("expected ACTIVE after keep-alive, got " + std::string(stateName(s))).c_str());
    }

    PASS("T5  Keep-alive — Source becomes ACTIVE from Sink heartbeat");
    return true;
}

// ── T6: CSM registerSource — Sink created on remote CSM; duplicate rejected ───
bool testCSMRegistration(rclcpp::Node::SharedPtr nodeA, rclcpp::Node::SharedPtr nodeB)
{
    ControlSignalManager csmA(nodeA.get(), "csm_a");
    ControlSignalManager csmB(nodeB.get(), "csm_b");

    auto info = makeInfo(
        "t6/joy",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC,
        "csm_b");

    // Register source on CSM_A → calls csm_b/control_signal_reg → CSM_B creates Sink
    bool ok = csmA.registerSource(info, 3000);
    if (!ok)
    {
        FAIL("T6", "registerSource() returned false");
    }

    if (!csmA.getSource("t6/joy"))
    {
        FAIL("T6", "CSM_A: getSource() nullptr after registration");
    }
    if (!csmB.getSink("t6/joy"))
    {
        FAIL("T6", "CSM_B: getSink() nullptr — remote Sink was not created");
    }

    // Source state should be UNKNOWN (topic, no keep-alive, no messages yet)
    auto srcState = csmA.getSourceState("t6/joy");
    if (srcState != ControlSignalState::UNKNOWN)
    {
        FAIL("T6", ("expected source UNKNOWN before any send, got " + std::string(stateName(srcState))).c_str());
    }

    // Duplicate registration must be rejected
    bool dup = csmA.registerSource(info, 3000);
    if (dup)
    {
        FAIL("T6", "duplicate registerSource() should return false but returned true");
    }

    PASS("T6  CSM registerSource — Sink created on remote CSM; duplicate rejected");
    return true;
}

// ── T7: CSM control_signal_info_req — source_list / sink_list populated ───────
bool testCSMInfoReq(
    rclcpp::Node::SharedPtr nodeC,
    rclcpp::Node::SharedPtr nodeD,
    rclcpp::Node::SharedPtr nodeClient)
{
    ControlSignalManager csmC(nodeC.get(), "csm_c");
    ControlSignalManager csmD(nodeD.get(), "csm_d");

    auto info1 = makeInfo(
        "t7/joy",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC,
        "csm_d");
    auto info2 = makeInfo(
        "t7/twist",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC,
        "csm_d");

    if (!csmC.registerSource(info1, 3000)) { FAIL("T7", "registerSource info1 failed"); }
    if (!csmC.registerSource(info2, 3000)) { FAIL("T7", "registerSource info2 failed"); }

    // ── Query csm_c/control_signal_info_req — should return 2 sources, 0 sinks
    auto clientC = nodeClient->create_client<srv::ControlSignalInfoReq>(
        "csm_c/control_signal_info_req");
    if (!clientC->wait_for_service(3s))
    {
        FAIL("T7", "csm_c/control_signal_info_req not available");
    }

    auto futC = clientC->async_send_request(
        std::make_shared<srv::ControlSignalInfoReq::Request>());
    if (futC.wait_for(3s) != std::future_status::ready)
    {
        FAIL("T7", "csm_c info_req timed out");
    }
    auto resC = futC.get();
    if (resC->source_list.size() != 2)
    {
        FAIL("T7", ("csm_c: expected 2 sources, got " +
                    std::to_string(resC->source_list.size())).c_str());
    }
    if (!resC->sink_list.empty())
    {
        FAIL("T7", ("csm_c: expected 0 sinks, got " +
                    std::to_string(resC->sink_list.size())).c_str());
    }

    // ── Query csm_d/control_signal_info_req — should return 0 sources, 2 sinks
    auto clientD = nodeClient->create_client<srv::ControlSignalInfoReq>(
        "csm_d/control_signal_info_req");
    if (!clientD->wait_for_service(3s))
    {
        FAIL("T7", "csm_d/control_signal_info_req not available");
    }

    auto futD = clientD->async_send_request(
        std::make_shared<srv::ControlSignalInfoReq::Request>());
    if (futD.wait_for(3s) != std::future_status::ready)
    {
        FAIL("T7", "csm_d info_req timed out");
    }
    auto resD = futD.get();
    if (resD->sink_list.size() != 2)
    {
        FAIL("T7", ("csm_d: expected 2 sinks, got " +
                    std::to_string(resD->sink_list.size())).c_str());
    }
    if (!resD->source_list.empty())
    {
        FAIL("T7", ("csm_d: expected 0 sources, got " +
                    std::to_string(resD->source_list.size())).c_str());
    }

    PASS("T7  CSM info_req — source_list and sink_list populated correctly");
    return true;
}

// ── T8: Twist service mode — UNKNOWN → ACTIVE after send() ───────────────────────
bool testTwistServiceMode(rclcpp::Node::SharedPtr nodeA, rclcpp::Node::SharedPtr nodeB)
{
    // 500 ms request timeout; Sink = service server on nodeA, Source = client on nodeB
    auto info = makeInfo(
        "t8/twist_svc",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_SERVICE,
        "", false, 0, 500'000'000LL);

    ControlSignalSink  <geometry_msgs::msg::Twist,
                        srv::ControlSignalTwist> sink(nodeA.get(), info);
    ControlSignalSource<geometry_msgs::msg::Twist,
                        srv::ControlSignalTwist> src (nodeB.get(), info);

    if (src.getState() != ControlSignalState::UNKNOWN)
    {
        FAIL("T8", "expected initial UNKNOWN for Twist service source");
    }

    rclcpp::sleep_for(300ms);   // wait for service to be ready

    bool cmdOk = false;
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = 1.5;
    twist.linear.y  = 0.25;
    twist.angular.z = 0.75;
    bool sent = src.send(twist, cmdOk);
    if (!sent || !cmdOk)
    {
        FAIL("T8", ("send() failed — sent=" + std::to_string(sent) +
                    " cmdOk=" + std::to_string(cmdOk)).c_str());
    }

    if (src.getState() != ControlSignalState::ACTIVE)
    {
        FAIL("T8", ("expected source ACTIVE after send, got " +
                    std::string(stateName(src.getState()))).c_str());
    }
    if (sink.getState() != ControlSignalState::ACTIVE)
    {
        FAIL("T8", ("expected sink ACTIVE after service call, got " +
                    std::string(stateName(sink.getState()))).c_str());
    }

    geometry_msgs::msg::Twist out;
    bool readOk = sink.read(out);
    if (!readOk)
        FAIL("T8", "sink.read() returned false");
    if (std::abs(out.linear.x  - 1.5 ) > 1e-6)
        FAIL("T8", "read(): linear.x mismatch");
    if (std::abs(out.linear.y  - 0.25) > 1e-6)
        FAIL("T8", "read(): linear.y mismatch");
    if (std::abs(out.angular.z - 0.75) > 1e-6)
        FAIL("T8", "read(): angular.z mismatch");

    PASS("T8  Twist service mode — UNKNOWN→ACTIVE after send(); values round-trip correct");
    return true;
}

// ── T9: CSM setSinkMsgCallback — callback fires when Sink receives a message ────────
bool testCSMSinkMsgCallback(
    rclcpp::Node::SharedPtr nodeE,
    rclcpp::Node::SharedPtr nodeF)
{
    ControlSignalManager csmE(nodeE.get(), "csm_e");
    ControlSignalManager csmF(nodeF.get(), "csm_f");

    std::atomic<int> cbCount{0};
    std::string      cbChannel;

    // Register callback BEFORE source registers (tests retroactive application to new Sinks)
    csmF.setSinkMsgCallback<sensor_msgs::msg::Joy>(
        [&](const sensor_msgs::msg::Joy&, const msg::ControlSignalInfo& info)
        {
            ++cbCount;
            cbChannel = info.channel_name;
        });

    auto info = makeInfo(
        "t9/joy_cb",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC,
        "csm_f");
    if (!csmE.registerSource(info, 3000))
    {
        FAIL("T9", "registerSource() returned false");
    }

    rclcpp::sleep_for(300ms);   // topic discovery

    auto* src = dynamic_cast<ControlSignalSource<sensor_msgs::msg::Joy>*>(
        csmE.getSource("t9/joy_cb").get());
    if (!src)
        FAIL("T9", "getSource() returned nullptr");

    bool ok = false;
    sensor_msgs::msg::Joy joy;
    joy.axes = {0.5f, -0.3f};
    src->send(joy, ok);
    rclcpp::sleep_for(300ms);

    if (cbCount.load() == 0)
        FAIL("T9", "setSinkMsgCallback not triggered on message receipt");
    if (cbChannel != "t9/joy_cb")
        FAIL("T9", ("wrong channel in callback: got '" + cbChannel + "'").c_str());

    PASS("T9  CSM setSinkMsgCallback — callback fires on Sink message receipt");
    return true;
}

// ── T10: Multiple sources of different types in same CSM pair ──────────────────
//  CSM_G has a Joy source and a Twist source, both registered to CSM_H.
//  Verifies: source/sink counts, channel names, type strings, and that
//  Joy and Twist messages are delivered through the correct Sinks.
bool testCSMMultipleTypes(
    rclcpp::Node::SharedPtr nodeG,
    rclcpp::Node::SharedPtr nodeH)
{
    ControlSignalManager csmG(nodeG.get(), "csm_g");
    ControlSignalManager csmH(nodeH.get(), "csm_h");

    auto joyInfo = makeInfo(
        "t10/joy",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC,
        "csm_h");
    auto twsInfo = makeInfo(
        "t10/twist",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC,
        "csm_h");

    if (!csmG.registerSource(joyInfo, 3000))
        FAIL("T10", "registerSource Joy failed");
    if (!csmG.registerSource(twsInfo, 3000))
        FAIL("T10", "registerSource Twist failed");

    // G has 2 sources; H has 0 sources and 2 sinks
    auto srcList = csmG.getSourceInfoList();
    auto snkList = csmH.getSinkInfoList();
    if (srcList.size() != 2)
        FAIL("T10", ("expected 2 sources in G, got " + std::to_string(srcList.size())).c_str());
    if (snkList.size() != 2)
        FAIL("T10", ("expected 2 sinks in H, got " + std::to_string(snkList.size())).c_str());

    if (!csmG.getSource("t10/joy"))
        FAIL("T10", "G: getSource('t10/joy') nullptr");
    if (!csmG.getSource("t10/twist"))
        FAIL("T10", "G: getSource('t10/twist') nullptr");
    if (!csmH.getSink("t10/joy"))
        FAIL("T10", "H: getSink('t10/joy') nullptr");
    if (!csmH.getSink("t10/twist"))
        FAIL("T10", "H: getSink('t10/twist') nullptr");

    // Verify correct type strings in H's sink list
    for (const auto& si : snkList)
    {
        if (si.channel_name == "t10/joy" &&
            si.control_signal_type != msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY)
            FAIL("T10", "t10/joy sink has wrong type");
        if (si.channel_name == "t10/twist" &&
            si.control_signal_type != msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST)
            FAIL("T10", "t10/twist sink has wrong type");
    }

    rclcpp::sleep_for(300ms);   // topic discovery

    // Send a Joy message and verify it arrives at the Joy Sink
    auto* joySrc = dynamic_cast<ControlSignalSource<sensor_msgs::msg::Joy>*>(
        csmG.getSource("t10/joy").get());
    if (!joySrc) FAIL("T10", "getSource Joy cast failed");
    bool ok;
    sensor_msgs::msg::Joy joy;
    joy.axes = {1.0f};
    joySrc->send(joy, ok);
    rclcpp::sleep_for(200ms);
    if (csmH.getSinkState("t10/joy") != ControlSignalState::ACTIVE)
        FAIL("T10", "Joy Sink not ACTIVE after send");

    // Send a Twist message and verify it arrives at the Twist Sink
    auto* twsSrc = dynamic_cast<ControlSignalSource<geometry_msgs::msg::Twist>*>(
        csmG.getSource("t10/twist").get());
    if (!twsSrc) FAIL("T10", "getSource Twist cast failed");
    geometry_msgs::msg::Twist tw;
    tw.linear.x = 2.0;
    twsSrc->send(tw, ok);
    rclcpp::sleep_for(200ms);
    if (csmH.getSinkState("t10/twist") != ControlSignalState::ACTIVE)
        FAIL("T10", "Twist Sink not ACTIVE after send");

    PASS("T10  Multiple types — Joy + Twist in same CSM pair; correct counts, types, delivery");
    return true;
}

// ── T11: LOW_FREQ state — ACTIVE → LOW_FREQ when send interval > send_freq_hz ────────
bool testSinkLowFreq(rclcpp::Node::SharedPtr nodeA, rclcpp::Node::SharedPtr nodeB)
{
    // send_freq_hz = 5 Hz → expected interval = 200 ms
    // timeout_ns = 2000 ms → LOW_FREQ window = [200 ms, 2000 ms)
    const float sendFreqHz = 5.0f;
    const int64_t timeoutNs = 2'000'000'000LL;  // 2 s
    
    auto info = makeInfo(
        "t11/joy",
        msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY,
        msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC,
        "",     // targetCsm
        false,  // useKeepAlive
        0,      // keepAliveNs (0 = disabled)
        timeoutNs,
        sendFreqHz,
        0);     // disconnectTimeoutNs

    ControlSignalSink  <sensor_msgs::msg::Joy> sink(nodeA.get(), info);
    ControlSignalSource<sensor_msgs::msg::Joy> src (nodeB.get(), info);

    if (sink.getState() != ControlSignalState::UNKNOWN)
    {
        FAIL("T11", ("expected initial UNKNOWN, got " + std::string(stateName(sink.getState()))).c_str());
    }

    rclcpp::sleep_for(300ms);   // topic discovery

    // Send first message → ACTIVE
    bool cmdOk = false;
    sensor_msgs::msg::Joy joy;
    joy.axes = {0.5f, -0.5f};
    src.send(joy, cmdOk);
    rclcpp::sleep_for(80ms);    // well below 200ms period; delivery takes <10ms

    if (sink.getState() != ControlSignalState::ACTIVE)
    {
        FAIL("T11", ("expected ACTIVE after first message, got " + std::string(stateName(sink.getState()))).c_str());
    }

    // Wait 250 ms (> 200 ms send period but < 2000 ms timeout) → LOW_FREQ
    rclcpp::sleep_for(250ms);

    auto s = sink.getState();
    if (s != ControlSignalState::LOW_FREQ)
    {
        FAIL("T11", ("expected LOW_FREQ after 250ms, got " + std::string(stateName(s))).c_str());
    }

    // Send another message; wait well within the 200ms period → back to ACTIVE
    src.send(joy, cmdOk);
    rclcpp::sleep_for(80ms);    // well below 200ms period

    s = sink.getState();
    if (s != ControlSignalState::ACTIVE)
    {
        FAIL("T11", ("expected ACTIVE after message sent, got " + std::string(stateName(s))).c_str());
    }

    // Wait 2100 ms without sending → TIMEOUT (exceeds 2000 ms timeout)
    rclcpp::sleep_for(2100ms);

    s = sink.getState();
    if (s != ControlSignalState::TIMEOUT)
    {
        FAIL("T11", ("expected TIMEOUT after 2100ms, got " + std::string(stateName(s))).c_str());
    }

    PASS("T11  LOW_FREQ state — ACTIVE→LOW_FREQ when interval > send_freq_hz; recovers on message");
    return true;
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Two general-purpose nodes for direct Source/Sink tests (T1-T5, T8)
    auto nodeA = rclcpp::Node::make_shared("test_node_a");
    auto nodeB = rclcpp::Node::make_shared("test_node_b");

    // Dedicated nodes for CSM tests (each CSM needs its own node for unique service names)
    auto nodeCSM_A  = rclcpp::Node::make_shared("test_csm_a_node");
    auto nodeCSM_B  = rclcpp::Node::make_shared("test_csm_b_node");
    auto nodeCSM_C  = rclcpp::Node::make_shared("test_csm_c_node");
    auto nodeCSM_D  = rclcpp::Node::make_shared("test_csm_d_node");
    auto nodeClient = rclcpp::Node::make_shared("test_client_node");

    // T9 — setSinkMsgCallback
    auto nodeCSM_E  = rclcpp::Node::make_shared("test_csm_e_node");
    auto nodeCSM_F  = rclcpp::Node::make_shared("test_csm_f_node");

    // T10 — multiple types in same CSM
    auto nodeCSM_G  = rclcpp::Node::make_shared("test_csm_g_node");
    auto nodeCSM_H  = rclcpp::Node::make_shared("test_csm_h_node");

    // Multi-threaded executor — must be running before any blocking call
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(nodeA);
    exec.add_node(nodeB);
    exec.add_node(nodeCSM_A);
    exec.add_node(nodeCSM_B);
    exec.add_node(nodeCSM_C);
    exec.add_node(nodeCSM_D);
    exec.add_node(nodeClient);
    exec.add_node(nodeCSM_E);
    exec.add_node(nodeCSM_F);
    exec.add_node(nodeCSM_G);
    exec.add_node(nodeCSM_H);
    std::thread execThread([&exec]() { exec.spin(); });

    RCLCPP_INFO(rclcpp::get_logger("test"), "=== Control Signal Transport Tests ===");

    int passed = 0;
    int failed = 0;
    auto run = [&](bool ok) { ok ? ++passed : ++failed; };

    run(testSourceTopicUnknown        (nodeA));
    run(testSinkTopicActiveOnReceive  (nodeA, nodeB));
    run(testSourceServiceMode         (nodeA, nodeB));
    run(testSinkTimeout               (nodeA, nodeB));
    run(testKeepAlive                 (nodeA, nodeB));
    run(testCSMRegistration           (nodeCSM_A, nodeCSM_B));
    run(testCSMInfoReq                (nodeCSM_C, nodeCSM_D, nodeClient));
    run(testTwistServiceMode          (nodeA, nodeB));
    run(testCSMSinkMsgCallback        (nodeCSM_E, nodeCSM_F));
    run(testCSMMultipleTypes          (nodeCSM_G, nodeCSM_H));
    run(testSinkLowFreq               (nodeA, nodeB));

    RCLCPP_INFO(rclcpp::get_logger("test"),
        "=== Results: %d passed, %d failed ===", passed, failed);

    exec.cancel();
    execThread.join();
    rclcpp::shutdown();
    return (failed == 0) ? 0 : 1;
}
