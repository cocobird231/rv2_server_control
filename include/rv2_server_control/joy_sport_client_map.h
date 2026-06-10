/**
 * joy_sport_client_map.h
 *
 * Defines Joy axis/button index constants and implements the JoyAction →
 * SportClient command translation with per-channel state tracking.
 *
 * ── Axis / Button layout ─────────────────────────────────────────────────────
 *  axes[0]  Left joy horizontal  [left=+1.0,  right=-1.0]
 *  axes[1]  Left joy vertical    [up=+1.0,    down=-1.0]
 *  axes[2]  L2 trigger           [pressed=-1.0, released=+1.0]
 *  axes[3]  Right joy horizontal [left=+1.0,  right=-1.0]
 *  axes[4]  Right joy vertical   [up=+1.0,    down=-1.0]
 *  axes[5]  R2 trigger           [pressed=-1.0, released=+1.0]
 *  axes[6]  D-pad horizontal     [left=+1.0,  right=-1.0]
 *  axes[7]  D-pad vertical       [up=+1.0,    down=-1.0]
 *
 *  button[0]   A
 *  button[1]   B
 *  button[2]   X
 *  button[3]   Y
 *  button[4]   L1
 *  button[5]   R1
 *  button[6]   Select
 *  button[7]   Start
 *  button[8]   Xbox
 *  button[9]   Left joy button
 *  button[10]  Right joy button
 *
 * ── Command mapping ───────────────────────────────────────────────────────────
 *
 *  L2 long-press combinations (highest priority):
 *    L2+A  SINGLE_CLICK  →  StandDown / StandUp (low-stand toggle)
 *    L2+B  SINGLE_CLICK  →  Damp
 *    L2+X  SINGLE_CLICK  →  RecoveryStand
 *    L2+Start SINGLE_CLICK → BalanceStand (running mode)
 *
 *  Standalone commands:
 *    Start     PRESS         →  BalanceStand (unlock / stand)
 *    R1        DOUBLE_CLICK  →  ClassicWalk(true)
 *    L2        DOUBLE_CLICK  →  SpeedLevel(0)   (low speed)
 *    L1        DOUBLE_CLICK  →  SpeedLevel(2)   (high speed)
 *    X         SINGLE_CLICK  →  VisionWalk(true)
 *    Select    SINGLE_CLICK  →  ContinuousGait(true) (random pose / auto)
 *
 *  Default (every output tick, no event match):
 *    Move(vx, vy, vyaw)
 *      vx   = axes[LEFT_V]    (up positive)
 *      vy   = axes[LEFT_H]    (left positive)
 *      vyaw = axes[RIGHT_H]   (left positive → CCW rotation)
 */

#pragma once

#include <set>
#include <vector>
#include <functional>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <joy_interpreter/joy_interpreter.hpp>
#include <joy_interpreter/button_interpreter.hpp>

#include "ros2_b2_sport_client.h"
#include "unitree_api/msg/request.hpp"

namespace rv2_interfaces::rv2_server_control
{

// ── Alias (also declared here so joy_sport_client_map.h is self-contained) ───

using SportClientCmd = std::function<void(SportClient&)>;

// ══════════════════════════════════════════════════════════════════════════════
//  Axis / button index constants
// ══════════════════════════════════════════════════════════════════════════════

constexpr uint8_t JOY_AX_LEFT_H  = 0;   ///< Left joy horizontal  [left=+1.0,  right=-1.0]
constexpr uint8_t JOY_AX_LEFT_V  = 1;   ///< Left joy vertical    [up=+1.0,    down=-1.0]
constexpr uint8_t JOY_AX_L2      = 2;   ///< L2 trigger           [pressed=-1.0, released=+1.0]
constexpr uint8_t JOY_AX_RIGHT_H = 3;   ///< Right joy horizontal [left=+1.0,  right=-1.0]
constexpr uint8_t JOY_AX_RIGHT_V = 4;   ///< Right joy vertical   [up=+1.0,    down=-1.0]
constexpr uint8_t JOY_AX_R2      = 5;   ///< R2 trigger           [pressed=-1.0, released=+1.0]
constexpr uint8_t JOY_AX_PAD_H   = 6;   ///< D-pad horizontal     [left=+1.0,  right=-1.0]
constexpr uint8_t JOY_AX_PAD_V   = 7;   ///< D-pad vertical       [up=+1.0,    down=-1.0]

constexpr uint8_t JOY_BTN_A         =  0;
constexpr uint8_t JOY_BTN_B         =  1;
constexpr uint8_t JOY_BTN_X         =  2;
constexpr uint8_t JOY_BTN_Y         =  3;
constexpr uint8_t JOY_BTN_L1        =  4;
constexpr uint8_t JOY_BTN_R1        =  5;
constexpr uint8_t JOY_BTN_SELECT    =  6;
constexpr uint8_t JOY_BTN_START     =  7;
constexpr uint8_t JOY_BTN_XBOX      =  8;
constexpr uint8_t JOY_BTN_LEFT_JOY  =  9;
constexpr uint8_t JOY_BTN_RIGHT_JOY = 10;

/// Axis value at or below which an analog trigger is considered "pressed".
constexpr float JOY_TRIGGER_PRESS_THRESHOLD = -0.5f;

/// Virtual button indices for synthesized L2/R2 events (must not overlap 0–10).
constexpr uint8_t VBTN_L2 = 20;
constexpr uint8_t VBTN_R2 = 21;


// ══════════════════════════════════════════════════════════════════════════════
//  Per-channel state
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Persistent state for one control-signal channel's Joy → sport translation.
 *
 * JoyInterpreter processes the real digital buttons (0–10).
 * Two separate ButtonInterpreter instances synthesise FSM events (DOUBLE_CLICK,
 * LONG_PRESS, …) for the analog L2/R2 triggers by thresholding their axis values
 * so they behave identically to digital buttons for combination detection.
 *
 * Dependent-event state:
 *   held_buttons   — buttons in PRESS/LONG_PRESS state (cleared on RELEASE)
 *   l2_long_pressed — set by L2 LONG_PRESS, cleared by L2 RELEASE;
 *                     gates the L2+button combination commands
 *   low_stand_active — tracks the StandDown/StandUp toggle for L2+A
 */
struct JoyChannelState
{
    joy_interpreter::JoyInterpreter    interpreter;
    joy_interpreter::ButtonInterpreter l2_interp{VBTN_L2};
    joy_interpreter::ButtonInterpreter r2_interp{VBTN_R2};

    std::set<uint8_t> held_buttons;   ///< Digital buttons currently pressed (PRESS → RELEASE)
    bool l2_long_pressed = false;     ///< L2 trigger is long-held (LONG_PRESS fired, not RELEASED)
    bool low_stand_active = false;    ///< L2+A toggle: true = currently in StandDown (low stand)
};


// ══════════════════════════════════════════════════════════════════════════════
//  Joy → SportClient translation
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Translate one Joy message into a SportClientCmd, updating channel state.
 *
 * Two-pass design:
 *   Pass 1 — apply all state transitions (held_buttons, l2_long_pressed) so that
 *            combination detection in Pass 2 uses the fully-updated state.
 *   Pass 2 — evaluate commands in priority order and return the highest-priority match.
 *
 * The returned SportClientCmd is a lightweight callable; invoke it on a SportClient
 * to drive the robot.  Returns a Move() command on every tick even if no event fired.
 */
inline SportClientCmd joyToSportClientCmd(
    JoyChannelState& st, const sensor_msgs::msg::Joy& joy)
{
    using Event = joy_interpreter::msg::JoyActionEvent;

    const rclcpp::Time now(joy.header.stamp);
    const auto& ax = joy.axes;

    // ── Pass 1a: synthesise L2 events from axis threshold ─────────────────────
    const bool l2_pressed =
        (ax.size() > JOY_AX_L2) && (ax[JOY_AX_L2] <= JOY_TRIGGER_PRESS_THRESHOLD);
    const auto l2_events = st.l2_interp.update(l2_pressed, now);

    // ── Pass 1b: update main interpreter (digital buttons 0–10) ──────────────
    const auto action = st.interpreter.update(joy);

    // ── Pass 1c: apply state transitions ─────────────────────────────────────

    SportClientCmd l2_event_cmd;  // command produced by an L2 FSM event (e.g. DOUBLE_CLICK)

    for (const auto& ev : l2_events) {
        switch (ev.action_type) {
            case Event::LONG_PRESS:
                st.l2_long_pressed = true;
                break;
            case Event::RELEASE:
                st.l2_long_pressed = false;
                break;
            case Event::DOUBLE_CLICK:
                l2_event_cmd = [](SportClient& sc) {
                    unitree_api::msg::Request req;
                    sc.SpeedLevel(req, 0);   // low speed mode
                };
                break;
            default:
                break;
        }
    }

    for (const auto& ev : action.events) {
        if      (ev.action_type == Event::PRESS)   st.held_buttons.insert(ev.button_index);
        else if (ev.action_type == Event::RELEASE)  st.held_buttons.erase(ev.button_index);
    }

    // ── Pass 2: evaluate commands (highest priority first) ────────────────────

    // Priority 1 — L2 long-press + button single-click combinations.
    // Evaluated before standalone commands so that e.g. L2+X overrides standalone X.
    if (st.l2_long_pressed) {
        for (const auto& ev : action.events) {
            if (ev.action_type != Event::SINGLE_CLICK) continue;
            switch (ev.button_index) {
                case JOY_BTN_A: {
                    // Toggle between low stand (StandDown) and stand (StandUp).
                    if (!st.low_stand_active) {
                        st.low_stand_active = true;
                        return [](SportClient& sc) {
                            unitree_api::msg::Request req;
                            sc.StandDown(req);  // enter low stand
                        };
                    } else {
                        st.low_stand_active = false;
                        return [](SportClient& sc) {
                            unitree_api::msg::Request req;
                            sc.StandUp(req);    // return to normal stand
                        };
                    }
                }
                case JOY_BTN_B:
                    return [](SportClient& sc) {
                        unitree_api::msg::Request req;
                        sc.Damp(req);           // damping mode
                    };
                case JOY_BTN_X:
                    return [](SportClient& sc) {
                        unitree_api::msg::Request req;
                        sc.RecoveryStand(req);  // recovery from fall
                    };
                case JOY_BTN_START:
                    return [](SportClient& sc) {
                        unitree_api::msg::Request req;
                        sc.BalanceStand(req);   // running mode (balance stand unlock)
                    };
                default:
                    break;
            }
        }
    }

    // Priority 2 — L2 double-click (low speed) and L1 double-click (high speed).
    if (l2_event_cmd) return l2_event_cmd;

    for (const auto& ev : action.events) {
        if (ev.button_index == JOY_BTN_L1 && ev.action_type == Event::DOUBLE_CLICK) {
            return [](SportClient& sc) {
                unitree_api::msg::Request req;
                sc.SpeedLevel(req, 2);          // high speed mode
            };
        }
    }

    // Priority 3 — standalone single-press / single-click / double-click commands.
    for (const auto& ev : action.events) {
        switch (ev.button_index) {
            case JOY_BTN_START:
                // Standalone Start PRESS (not combined with L2) → unlock / balance stand.
                if (ev.action_type == Event::PRESS && !st.l2_long_pressed) {
                    return [](SportClient& sc) {
                        unitree_api::msg::Request req;
                        sc.BalanceStand(req);
                    };
                }
                break;

            case JOY_BTN_R1:
                if (ev.action_type == Event::DOUBLE_CLICK) {
                    return [](SportClient& sc) {
                        unitree_api::msg::Request req;
                        sc.ClassicWalk(req, true);  // classic gait mode
                    };
                }
                break;

            case JOY_BTN_X:
                // Standalone X SINGLE_CLICK (not combined with L2) → visual walk mode.
                if (ev.action_type == Event::SINGLE_CLICK && !st.l2_long_pressed) {
                    return [](SportClient& sc) {
                        unitree_api::msg::Request req;
                        sc.VisionWalk(req, true);   // visual walk mode
                    };
                }
                break;

            case JOY_BTN_SELECT:
                if (ev.action_type == Event::SINGLE_CLICK) {
                    return [](SportClient& sc) {
                        unitree_api::msg::Request req;
                        sc.ContinuousGait(req, true);  // random pose / continuous gait
                    };
                }
                break;

            default:
                break;
        }
    }

    // Priority 4 (default) — continuous Move every output tick.
    const float vx   = (ax.size() > JOY_AX_LEFT_V)  ?  ax[JOY_AX_LEFT_V]  : 0.0f;
    const float vy   = (ax.size() > JOY_AX_LEFT_H)  ?  ax[JOY_AX_LEFT_H]  : 0.0f;
    const float vyaw = (ax.size() > JOY_AX_RIGHT_H) ?  ax[JOY_AX_RIGHT_H] : 0.0f;
    return [vx, vy, vyaw](SportClient& sc) {
        unitree_api::msg::Request req;
        sc.Move(req, vx, vy, vyaw);
    };
}

}  // namespace rv2_interfaces::rv2_server_control
