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

template <typename MsgT> inline bool isEmergencyStop(const MsgT& msg);

template <> inline bool isEmergencyStop(const sensor_msgs::msg::Joy& joy)
{
    if (static_cast<int>(joy.axes.size()) < 6)
        return false;
    return joy.axes[5] < 0.5;  // R2 trigger pressed to stop robot.
}

template <> inline bool isEmergencyStop(const geometry_msgs::msg::Twist& twist)
{
    return twist.linear.z == -99.0 && twist.angular.x == -99.0 && twist.angular.y == -99.0;
}

// ── isRequestActive ───────────────────────────────────────────────────────────

template <typename MsgT> inline bool isRequestActive(const MsgT& msg);

template <> inline bool isRequestActive(const sensor_msgs::msg::Joy& joy)
{
    if (joy.buttons.size() <= 11)
        return false;
    return joy.buttons[11] == 1;
}

template <> inline bool isRequestActive(const geometry_msgs::msg::Twist& twist)
{
    return twist.linear.z == 99.0 && twist.angular.x == 99.0 && twist.angular.y == 99.0;
}

}  // namespace rv2_interfaces::rv2_server_control
