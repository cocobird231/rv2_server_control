/**
 * control_signal_detect.h
 *
 * Overloaded predicates for detecting emergency-stop and request-active
 * sentinel values embedded in ROS 2 control-signal messages.
 */

#pragma once

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace rv2_interfaces::rv2_server_control
{

// ── isEmergencyStop ───────────────────────────────────────────────────────────

template <typename MsgT>
inline bool isEmergencyStop(const MsgT & msg);

template <>
inline bool isEmergencyStop(const sensor_msgs::msg::Joy & joy)
{
    if (static_cast<int>(joy.buttons.size()) < 4) return false;
    return joy.buttons[3] == 1;  // Right pad button (index 3) is the e-stop command for Joy messages.
}

template <>
inline bool isEmergencyStop(const geometry_msgs::msg::Twist & twist)
{
    return twist.linear.z  == -99.0 &&
           twist.angular.x == -99.0 &&
           twist.angular.y == -99.0;
}

// ── isRequestActive ───────────────────────────────────────────────────────────

template <typename MsgT>
inline bool isRequestActive(const MsgT & msg);

template <>
inline bool isRequestActive(const sensor_msgs::msg::Joy & joy)
{
    if (static_cast<int>(joy.buttons.size()) < 4) return false;
    return joy.buttons[0] == 99 && joy.buttons[1] == 99 &&
           joy.buttons[2] == 99 && joy.buttons[3] == 99;
}

template <>
inline bool isRequestActive(const geometry_msgs::msg::Twist & twist)
{
    return twist.linear.z  == 99.0 &&
           twist.angular.x == 99.0 &&
           twist.angular.y == 99.0;
}

}  // namespace rv2_interfaces::rv2_server_control
