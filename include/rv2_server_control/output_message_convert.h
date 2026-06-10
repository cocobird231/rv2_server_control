/**
 * output_message_convert.h
 *
 * Converts ROS 2 control-signal messages to SportClientCmd — a callable
 * that drives a SportClient directly.
 *
 * ── Interface ─────────────────────────────────────────────────────────────────
 *
 *  SportClientCmd  — alias for std::function<void(SportClient&)>
 *                    (defined in joy_sport_client_map.h)
 *
 *  msgToOutSignal(id, msg)  — convert a message to a SportClientCmd.
 *    id  : string channel identifier; used to maintain per-channel state
 *          (e.g. JoyInterpreter FSM, held-button tracking).
 *    msg : incoming ROS 2 message.
 *
 *  makeStopSignal()  — returns a SportClientCmd that issues StopMove.
 *
 * ── Specialisations ───────────────────────────────────────────────────────────
 *
 *  Joy (sensor_msgs::msg::Joy)
 *    Each channel id owns a JoyChannelState (static map).  See joy_sport_client_map.h
 *    for the full axis/button layout and command mapping table.
 *
 *  Twist (geometry_msgs::msg::Twist)
 *    Always returns Move(linear.x, linear.y, angular.z).
 *    id is unused.
 *
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *  #include "rv2_server_control/output_message_convert.h"
 *
 *  // In outputCb:
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

#include "rv2_server_control/joy_sport_client_map.h"

namespace rv2_interfaces::rv2_server_control
{

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
    static std::map<std::string, JoyChannelState> states;
    return joyToSportClientCmd(states[id], joy);
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
    return [vx, vy, vyaw](SportClient& sc) {
        unitree_api::msg::Request req;
        sc.Move(req, vx, vy, vyaw);
    };
}

}  // namespace rv2_interfaces::rv2_server_control
