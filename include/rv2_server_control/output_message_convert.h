/**
 * output_message_convert.h
 *
 * Converts ROS 2 control-signal messages to SportClientCmd — a callable
 * that drives a SportClient directly, with no intermediate Request object
 * exposed to callers.
 *
 * ── Interface ─────────────────────────────────────────────────────────────────
 *
 *  SportClientCmd  — alias for std::function<void(SportClient&)>
 *
 *  msgToOutSignal(id, msg)  — convert an input message to a SportClientCmd.
 *    id  : string channel identifier; used to maintain per-channel state
 *          (e.g. a JoyInterpreter instance per source).
 *    msg : incoming ROS 2 message.
 *
 *  makeStopSignal()  — returns a SportClientCmd that issues StopMove.
 *
 * ── Specialisations ───────────────────────────────────────────────────────────
 *
 *  Joy (sensor_msgs::msg::Joy)
 *    Each unique id owns a JoyInterpreter (static map, one instance per channel).
 *    Button events → sport commands (first PRESS event wins per timer tick):
 *      button[0] PRESS  →  Damp
 *      button[1] PRESS  →  StandUp
 *      button[2] PRESS  →  StandDown
 *      button[6] PRESS  →  StopMove
 *      button[7] PRESS  →  SwitchGait(0)
 *      button[8] PRESS  →  SwitchGait(1)
 *      button[9] PRESS  →  RecoveryStand
 *    Default (no button event): Move(vx, 0, vyaw)
 *      vx   = axes[0] - axes[1]
 *      vyaw = axes[2] - axes[3]
 *
 *  Twist (geometry_msgs::msg::Twist)
 *    Always returns Move(linear.x, linear.y, angular.z).
 *    id is unused (Twist has no per-channel interpreter state).
 *
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *  #include "rv2_server_control/output_message_convert.h"
 *
 *  // In outputCb — drives sportClient_ directly:
 *  msgToOutSignal(info.channel_name, joy_msg)(sportClient_);
 *
 *  // Emergency stop:
 *  makeStopSignal()(sportClient_);
 */

#pragma once

#include <functional>
#include <map>
#include <string>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <joy_interpreter/joy_interpreter.hpp>

#include "ros2_b2_sport_client.h"
#include "unitree_api/msg/request.hpp"

namespace rv2_interfaces::rv2_server_control
{

// ── Output type ───────────────────────────────────────────────────────────────

/** A callable that issues one sport command on the provided SportClient. */
using SportClientCmd = std::function<void(SportClient&)>;

// ── Primary template (no default implementation) ──────────────────────────────

template<typename inT>
SportClientCmd msgToOutSignal(const std::string& id, const inT& msg);

// ── Stop signal ───────────────────────────────────────────────────────────────

inline SportClientCmd makeStopSignal()
{
    return [](SportClient& sc) {
        unitree_api::msg::Request req;
        sc.StopMove(req);
    };
}

// ── Specialisation: Joy ───────────────────────────────────────────────────────

template<>
inline SportClientCmd msgToOutSignal<sensor_msgs::msg::Joy>(
    const std::string& id, const sensor_msgs::msg::Joy& joy)
{
    // One JoyInterpreter per channel — maintains FSM state across calls.
    static std::map<std::string, joy_interpreter::JoyInterpreter> interpreters;
    const auto action = interpreters[id].update(joy);

    using Event = joy_interpreter::msg::JoyActionEvent;
    for (const auto& ev : action.events)
    {
        if (ev.action_type != Event::PRESS) continue;
        switch (ev.button_index)
        {
            case 0: return [](SportClient& sc){ unitree_api::msg::Request req; sc.Damp(req); };
            case 1: return [](SportClient& sc){ unitree_api::msg::Request req; sc.StandUp(req); };
            case 2: return [](SportClient& sc){ unitree_api::msg::Request req; sc.StandDown(req); };
            case 6: return [](SportClient& sc){ unitree_api::msg::Request req; sc.StopMove(req); };
            case 7: return [](SportClient& sc){ unitree_api::msg::Request req; sc.SwitchGait(req, 0); };
            case 8: return [](SportClient& sc){ unitree_api::msg::Request req; sc.SwitchGait(req, 1); };
            case 9: return [](SportClient& sc){ unitree_api::msg::Request req; sc.RecoveryStand(req); };
            default: break;
        }
    }

    // Default: continuous Move from axes.
    const auto& ax  = action.axes;
    const float vx   = (ax.size() > 1) ? (ax[0] - ax[1]) : (ax.size() > 0 ? ax[0] : 0.0f);
    const float vyaw = (ax.size() > 3) ? (ax[2] - ax[3]) : 0.0f;
    return [vx, vyaw](SportClient& sc)
    {
        unitree_api::msg::Request req;
        sc.Move(req, vx, 0.0f, vyaw);
    };
}

// ── Specialisation: Twist ─────────────────────────────────────────────────────

template<>
inline SportClientCmd msgToOutSignal<geometry_msgs::msg::Twist>(
    const std::string& id, const geometry_msgs::msg::Twist& twist)
{
    (void)id;
    const float vx   = static_cast<float>(twist.linear.x);
    const float vy   = static_cast<float>(twist.linear.y);
    const float vyaw = static_cast<float>(twist.angular.z);
    return [vx, vy, vyaw](SportClient& sc)
    {
        unitree_api::msg::Request req;
        sc.Move(req, vx, vy, vyaw);
    };
}

}  // namespace rv2_interfaces::rv2_server_control
