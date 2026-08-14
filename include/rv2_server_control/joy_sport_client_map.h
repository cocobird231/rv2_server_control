/**
 * joy_sport_client_map.h
 *
 * Defines Joy axis/button index constants and implements the Joy →
 * SportClient command translation with per-channel state tracking.
 *
 * Layout follows docs/joy按鍵定義_數值範圍.json.
 *
 * ── Axis / Button layout ─────────────────────────────────────────────────────
 *  NOTE: the real joy driver reports the standard Xbox axis layout (verified
 *  on hardware: left stick worked at axes[0..1] while the doc's axes[2] for
 *  the right stick was actually the L2 trigger) — the doc JSON's 4-axis list
 *  is the logical mapping only.
 *
 *  axes[0]  Left joy horizontal   [-1.0 ~ +1.0]  → Move vx
 *  axes[1]  Left joy vertical     [-1.0 ~ +1.0]  → Move vy
 *  axes[2]  L2 trigger            [released=+1.0, pressed=-1.0]  (unused)
 *  axes[3]  Right joy horizontal  [-1.0 ~ +1.0]  → Move vyaw
 *  axes[4]  Right joy vertical    [-1.0 ~ +1.0]  (unused)
 *  axes[5]  R2 trigger            [released=+1.0, pressed=-1.0]  (e-stop, see
 *                                  control_signal_detect.h)
 *
 *  button[0]   A       → StandUp
 *  button[1]   B       → StandDown
 *  button[2]   X       → StopMove
 *  button[3]   Y       → RecoveryStand
 *  button[4]   L1      (unused)
 *  button[5]   R1      (unused)
 *  button[6]   L2      (unused)
 *  button[7]   R2      (unused)
 *  button[8]   Select  (unused)
 *  button[9]   Start   (unused)
 *  button[10]  Left joy button   (unused)
 *  button[11]  Right joy button  (unused)
 *  button[12]  D-pad up          (unused)
 *  button[13]  D-pad down        (unused)
 *  button[14]  D-pad left        (unused)
 *  button[15]  D-pad right       (unused)
 *  button[16]  Home              (reserved by system)
 *
 * ── Command mapping ───────────────────────────────────────────────────────────
 *
 *  Single-button commands (no combinations), triggered on PRESS edge:
 *    A  PRESS  →  StandUp
 *    B  PRESS  →  StandDown
 *    X  PRESS  →  StopMove
 *    Y  PRESS  →  RecoveryStand
 *
 *  Move (event-triggered, value-change gated):
 *    Move(vx, vy, vyaw)
 *      vx   = axes[0]  (left joy horizontal)
 *      vy   = axes[1]  (left joy vertical)
 *      vyaw = axes[3]  (right joy horizontal)
 *    Sent only when (vx, vy, vyaw) differs from the previously sent Move.
 *    Identical values (e.g. joystick idle at 0) produce NO command — the
 *    translation returns an empty SportClientCmd, which callers must check
 *    before invoking.
 */

#pragma once

#include <functional>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <joy_interpreter/joy_interpreter.hpp>

#include "ros2_b2_sport_client.h"
#include "unitree_api/msg/request.hpp"

namespace rv2_interfaces::rv2_server_control
{

// ── Alias (also declared here so joy_sport_client_map.h is self-contained) ───

using SportClientCmd = std::function<void(SportClient&)>;

// ══════════════════════════════════════════════════════════════════════════════
//  Axis / button index constants
// ══════════════════════════════════════════════════════════════════════════════

constexpr uint8_t JOY_AX_LEFT_H  = 0;   ///< Left joy horizontal  → Move vx   [-1.0 ~ +1.0]
constexpr uint8_t JOY_AX_LEFT_V  = 1;   ///< Left joy vertical    → Move vy   [-1.0 ~ +1.0]
constexpr uint8_t JOY_AX_L2      = 2;   ///< L2 trigger           (unused)    [released=+1.0]
constexpr uint8_t JOY_AX_RIGHT_H = 3;   ///< Right joy horizontal → Move vyaw [-1.0 ~ +1.0]
constexpr uint8_t JOY_AX_RIGHT_V = 4;   ///< Right joy vertical   (unused)    [-1.0 ~ +1.0]
constexpr uint8_t JOY_AX_R2      = 5;   ///< R2 trigger           (e-stop)    [released=+1.0]

constexpr uint8_t JOY_BTN_A         =  0;  ///< StandUp
constexpr uint8_t JOY_BTN_B         =  1;  ///< StandDown
constexpr uint8_t JOY_BTN_X         =  2;  ///< StopMove
constexpr uint8_t JOY_BTN_Y         =  3;  ///< RecoveryStand
constexpr uint8_t JOY_BTN_L1        =  4;
constexpr uint8_t JOY_BTN_R1        =  5;
constexpr uint8_t JOY_BTN_L2        =  6;
constexpr uint8_t JOY_BTN_R2        =  7;
constexpr uint8_t JOY_BTN_SELECT    =  8;
constexpr uint8_t JOY_BTN_START     =  9;
constexpr uint8_t JOY_BTN_LEFT_JOY  = 10;
constexpr uint8_t JOY_BTN_RIGHT_JOY = 11;
constexpr uint8_t JOY_BTN_PAD_UP    = 12;
constexpr uint8_t JOY_BTN_PAD_DOWN  = 13;
constexpr uint8_t JOY_BTN_PAD_LEFT  = 14;
constexpr uint8_t JOY_BTN_PAD_RIGHT = 15;
constexpr uint8_t JOY_BTN_HOME      = 16;


// ══════════════════════════════════════════════════════════════════════════════
//  Per-channel state
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Persistent state for one control-signal channel's Joy → sport translation.
 *
 * JoyInterpreter debounces the digital buttons and emits PRESS/RELEASE edge
 * events so each button command fires once per physical press.
 *
 * last_vx/last_vy/last_vyaw hold the values of the last Move command actually
 * sent on this channel; a new Move is emitted only when the values change.
 * Initialised to 0 so an idle joystick produces no output from startup.
 */
struct JoyChannelState
{
    joy_interpreter::JoyInterpreter interpreter;

    float last_vx   = 0.0f;   ///< vx of the last emitted Move
    float last_vy   = 0.0f;   ///< vy of the last emitted Move
    float last_vyaw = 0.0f;   ///< vyaw of the last emitted Move
};


// ══════════════════════════════════════════════════════════════════════════════
//  Joy → SportClient translation
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Translate one Joy message into a SportClientCmd, updating channel state.
 *
 * Single-button mapping, evaluated in priority order:
 *   A PRESS → StandUp, B PRESS → StandDown, X PRESS → StopMove,
 *   Y PRESS → RecoveryStand.
 *
 * Fully event-triggered: returns a command only when a button PRESS fired or
 * the Move values (vx, vy, vyaw) changed since the last emitted Move.
 * Otherwise returns an EMPTY SportClientCmd — callers MUST check it
 * (`if (cmd) cmd(sc);`) before invoking.
 */
inline SportClientCmd joyToSportClientCmd(
    JoyChannelState& st, const sensor_msgs::msg::Joy& joy)
{
    using Event = joy_interpreter::msg::JoyActionEvent;

    const rclcpp::Time now(joy.header.stamp);
    const auto& ax = joy.axes;
    const auto action = st.interpreter.update(joy, now);

    for (const auto& ev : action.events) {
        if (ev.action_type != Event::PRESS) continue;
        switch (ev.button_index) {
            case JOY_BTN_A:
                return [](SportClient& sc) {
                    unitree_api::msg::Request req;
                    sc.StandUp(req);
                };
            case JOY_BTN_B:
                return [](SportClient& sc) {
                    unitree_api::msg::Request req;
                    sc.StandDown(req);
                };
            case JOY_BTN_X:
                return [](SportClient& sc) {
                    unitree_api::msg::Request req;
                    sc.StopMove(req);
                };
            case JOY_BTN_Y:
                return [](SportClient& sc) {
                    unitree_api::msg::Request req;
                    sc.RecoveryStand(req);
                };
            default:
                break;
        }
    }

    // Move — emitted only when values changed since the last emitted Move.
    const float vx   = (ax.size() > JOY_AX_LEFT_H)  ?  ax[JOY_AX_LEFT_H]  : 0.0f;
    const float vy   = (ax.size() > JOY_AX_LEFT_V)  ?  ax[JOY_AX_LEFT_V]  : 0.0f;
    const float vyaw = (ax.size() > JOY_AX_RIGHT_H) ?  ax[JOY_AX_RIGHT_H] : 0.0f;

    if (vx == st.last_vx && vy == st.last_vy && vyaw == st.last_vyaw)
        return {};  // no value change → no event → no output

    st.last_vx   = vx;
    st.last_vy   = vy;
    st.last_vyaw = vyaw;
    return [vx, vy, vyaw](SportClient& sc) {
        unitree_api::msg::Request req;
        sc.Move(req, vx, vy, vyaw);
    };
}

}  // namespace rv2_interfaces::rv2_server_control
