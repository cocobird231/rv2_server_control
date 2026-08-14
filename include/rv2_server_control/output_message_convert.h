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
 *          (e.g. JoyInterpreter FSM, last emitted Move values).
 *    msg : incoming ROS 2 message.
 *    May return an EMPTY SportClientCmd when the message carries no new event
 *    (e.g. Move values identical to the last emitted Move).  Callers MUST
 *    check before invoking: `if (auto cmd = msgToOutSignal(...)) cmd(sc);`
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
 *    Returns Move(linear.x, linear.y, angular.z) only when the values differ
 *    from the last emitted Move on this channel id; otherwise returns an
 *    empty SportClientCmd (no new event).
 *
 * ── Usage ─────────────────────────────────────────────────────────────────────
 *  #include "rv2_server_control/output_message_convert.h"
 *
 *  // In outputCb (cmd may be empty — check before invoking):
 *  if (auto cmd = msgToOutSignal(info.channel_name, joy_msg)) cmd(sportClient_);
 *
 *  // Emergency stop (always valid):
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
    struct LastMove { float vx = 0.0f, vy = 0.0f, vyaw = 0.0f; };
    static std::map<std::string, LastMove> lastMoves;

    const float vx   = static_cast<float>(twist.linear.x);
    const float vy   = static_cast<float>(twist.linear.y);
    const float vyaw = static_cast<float>(twist.angular.z);

    auto& last = lastMoves[id];
    if (vx == last.vx && vy == last.vy && vyaw == last.vyaw)
        return {};  // no value change → no new event → no output

    last = {vx, vy, vyaw};
    return [vx, vy, vyaw](SportClient& sc) {
        unitree_api::msg::Request req;
        sc.Move(req, vx, vy, vyaw);
    };
}

}  // namespace rv2_interfaces::rv2_server_control
