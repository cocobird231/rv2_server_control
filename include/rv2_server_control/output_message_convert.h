/** @file output_message_convert.h
 * @brief Per-server Joy/Twist conversion state, reset on stop or owner change.
 */
#pragma once

#include <map>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "rv2_server_control/joy_sport_client_map.h"

namespace rv2_interfaces::rv2_server_control
{

inline SportClientCmd makeStopSignal()
{
    return [](SportClient& client)
    {
        unitree_api::msg::Request request;
        client.StopMove(request);
    };
}

/** No static process state: independent server instances cannot share dedup. */
class OutputMessageConverter
{
public:
    SportClientCmd convert(const std::string& channel, const sensor_msgs::msg::Joy& message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return joyToSportClientCmd(joy_[channel], message);
    }

    SportClientCmd convert(const std::string& channel, const geometry_msgs::msg::Twist& message)
    {
        const float vx = static_cast<float>(message.linear.x);
        const float vy = static_cast<float>(message.linear.y);
        const float yaw = static_cast<float>(message.angular.z);
        std::lock_guard<std::mutex> lock(mutex_);
        auto& last = twist_[channel];
        if (vx == last.vx && vy == last.vy && yaw == last.yaw)
            return {};
        last = {vx, vy, yaw};
        return [vx, vy, yaw](SportClient& client)
        {
            unitree_api::msg::Request request;
            client.Move(request, vx, vy, yaw);
        };
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        joy_.clear();
        twist_.clear();
    }

private:
    struct Move
    {
        float vx = 0;
        float vy = 0;
        float yaw = 0;
    };
    std::mutex mutex_;
    std::map<std::string, JoyChannelState> joy_;
    std::map<std::string, Move> twist_;
};

}  // namespace rv2_interfaces::rv2_server_control
