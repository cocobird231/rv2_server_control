/**
 * output_message_convert.h
 *
 * Generic interface for converting ROS 2 control-signal messages to an output
 * signal type, and for generating stop signals.
 *
 * ── Generic interface ─────────────────────────────────────────────────────────
 *  msgToOutSignal<outT>(inT msg)  — convert an input message to outT
 *  makeStopSignal<outT>()         — produce a type-appropriate stop signal
 *
 * ── Specialisations: unitree_api::msg::Request ────────────────────────────────
 *  msgToOutSignal<Request>(const Joy &)    — Joy  → sport command
 *  msgToOutSignal<Request>(const Twist &)  — Twist → sport MOVE
 *  makeStopSignal<Request>()               — StopMove (api_id 1003)
 *
 *  Joy button-mode commands (first match wins):
 *    buttons[0] = 1  →  Damp          (api_id 1001)
 *    buttons[1] = 1  →  StandUp       (api_id 1004)
 *    buttons[2] = 1  →  StandDown     (api_id 1005)
 *    buttons[6] = 1  →  StopMove      (api_id 1003)
 *    buttons[7] = 1  →  SwitchGait 0  (api_id 1011, {"data":0})
 *    buttons[8] = 1  →  SwitchGait 1  (api_id 1011, {"data":1})
 *    buttons[9] = 1  →  RecoveryStand (api_id 1006)
 *  Default: MOVE (api_id 1008).
 *    vx = axes[0] - axes[1],  vy = 0,  vyaw = axes[2] - axes[3]
 *
 *  Twist MOVE (api_id 1008):
 *    linear.x → vx,  linear.y → vy,  angular.z → vyaw
 *
 * ── Usage ────────────────────────────────────────────────────────────────────
 *  #include "rv2_server_control/output_message_convert.h"
 *
 *  auto req  = rv2_server_control::msgToOutSignal<unitree_api::msg::Request>(joy_msg);
 *  auto stop = rv2_server_control::makeStopSignal<unitree_api::msg::Request>();
 */

#pragma once

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "unitree_api/msg/request.hpp"
#include "nlohmann/json.hpp"

namespace rv2_interfaces::rv2_server_control
{

// ── Unitree Sport API IDs ─────────────────────────────────────────────────────
//  Matches ROBOT_SPORT_API_ID_* constants in ros2_b2_sport_client.h.

static constexpr int64_t SPORT_API_ID_DAMP          = 1001;
static constexpr int64_t SPORT_API_ID_STOPMOVE       = 1003;
static constexpr int64_t SPORT_API_ID_STANDUP        = 1004;
static constexpr int64_t SPORT_API_ID_STANDDOWN      = 1005;
static constexpr int64_t SPORT_API_ID_RECOVERY_STAND = 1006;
static constexpr int64_t SPORT_API_ID_MOVE           = 1008;
static constexpr int64_t SPORT_API_ID_SWITCH_GAIT    = 1011;

// ── Primary templates (no default implementation) ─────────────────────────────

template<typename outT, typename inT>
outT msgToOutSignal(const inT & msg);

template<typename outT>
outT makeStopSignal();

// ── Specialisations: unitree_api::msg::Request + Joy ─────────────────────────

template<>
inline unitree_api::msg::Request msgToOutSignal<unitree_api::msg::Request, sensor_msgs::msg::Joy>(
    const sensor_msgs::msg::Joy & joy)
{
    const auto & btn = joy.buttons;
    const size_t nb  = btn.size();

    unitree_api::msg::Request req;

    // Button-mode commands (first match wins)
    if (nb > 0 && btn[0] == 1) { req.header.identity.api_id = SPORT_API_ID_DAMP;           return req; }
    if (nb > 1 && btn[1] == 1) { req.header.identity.api_id = SPORT_API_ID_STANDUP;        return req; }
    if (nb > 2 && btn[2] == 1) { req.header.identity.api_id = SPORT_API_ID_STANDDOWN;      return req; }
    if (nb > 6 && btn[6] == 1) { req.header.identity.api_id = SPORT_API_ID_STOPMOVE;       return req; }
    if (nb > 9 && btn[9] == 1) { req.header.identity.api_id = SPORT_API_ID_RECOVERY_STAND; return req; }
    if (nb > 7 && btn[7] == 1) {
        req.header.identity.api_id = SPORT_API_ID_SWITCH_GAIT;
        req.parameter = nlohmann::json{{"data", 0}}.dump();
        return req;
    }
    if (nb > 8 && btn[8] == 1) {
        req.header.identity.api_id = SPORT_API_ID_SWITCH_GAIT;
        req.parameter = nlohmann::json{{"data", 1}}.dump();
        return req;
    }

    // Default: MOVE from axes[0..3]
    const auto & ax  = joy.axes;
    const float  vx   = (ax.size() > 1) ? (ax[0] - ax[1]) : (ax.size() > 0 ? ax[0] : 0.0f);
    const float  vyaw = (ax.size() > 3) ? (ax[2] - ax[3]) : 0.0f;

    nlohmann::json js;
    js["x"] = vx;
    js["y"] = 0.0f;
    js["z"] = vyaw;

    req.header.identity.api_id = SPORT_API_ID_MOVE;
    req.parameter              = js.dump();
    return req;
}

// ── Specialisations: unitree_api::msg::Request + Twist ───────────────────────

template<>
inline unitree_api::msg::Request msgToOutSignal<unitree_api::msg::Request, geometry_msgs::msg::Twist>(
    const geometry_msgs::msg::Twist & twist)
{
    nlohmann::json js;
    js["x"] = static_cast<float>(twist.linear.x);
    js["y"] = static_cast<float>(twist.linear.y);
    js["z"] = static_cast<float>(twist.angular.z);

    unitree_api::msg::Request req;
    req.header.identity.api_id = SPORT_API_ID_MOVE;
    req.parameter              = js.dump();
    return req;
}

// ── Specialisation: makeStopSignal → unitree_api::msg::Request ────────────────

template<>
inline unitree_api::msg::Request makeStopSignal<unitree_api::msg::Request>()
{
    unitree_api::msg::Request req;
    req.header.identity.api_id = SPORT_API_ID_STOPMOVE;
    return req;
}

}  // namespace rv2_interfaces::rv2_server_control
