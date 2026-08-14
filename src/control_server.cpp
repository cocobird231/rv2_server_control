/**
 * This file implements the ControlServer class, which receives control signals
 * and routes them to sport client commands via msgToOutSignal().
 *
 * The control server owns a ControlSignalManager (CSM) that manages remote
 * Source connections. Output is event-triggered: outputCb runs on each message
 * received from the active Sink. A low-frequency safety watchdog re-selects a
 * fallback Sink and issues an emergency stop when signal is lost. Per-message-type
 * conversion is delegated to output_message_convert.h so that outputCb only needs
 * to invoke the sport client.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "rv2_server_control/control_server.h"
#include "rv2_server_control/control_signal_detect.h"
#include "rv2_server_control/output_message_convert.h"
#include "rclcpp_components/register_node_macro.hpp"



// ── ControlServerNode ─────────────────────────────────────────────────────────

class ControlServerNode : public rclcpp::Node
{
public:
    explicit ControlServerNode(const rclcpp::NodeOptions & options)
        : Node("control_server", options)
        , sportClient_(this)
    {
        // Declare parameters
        this->declare_parameter<std::string>("server_name",              "control_server");
        this->declare_parameter<int64_t>    ("watchdog_interval_ms",     100);
        this->declare_parameter<int64_t>    ("status_timer_interval_ms", 1000);

        const std::string serverName          = this->get_parameter("server_name").as_string();
        const int64_t     watchdogIntervalMs  = this->get_parameter("watchdog_interval_ms").as_int();
        const int64_t     statusTimerMs       = this->get_parameter("status_timer_interval_ms").as_int();

        // Build ControlServer config
        rv2_interfaces::rv2_server_control::ControlServer::Config cfg;
        cfg.name                  = serverName;
        cfg.watchdogIntervalNs    = watchdogIntervalMs * 1'000'000LL;
        cfg.statusTimerIntervalMs = statusTimerMs;

        server_ = std::make_unique<rv2_interfaces::rv2_server_control::ControlServer>(this, std::move(cfg));

        // Register Joy type config
        rv2_interfaces::rv2_server_control::ControlServer::TypeConfig<sensor_msgs::msg::Joy> joyCfg;
        joyCfg.outputCb = [this](const sensor_msgs::msg::Joy& joy,
                                 const rv2_interfaces::msg::ControlSignalInfo& info)
        {
            // cmd is empty when the message carries no new event (e.g. idle joystick).
            if (auto cmd = rv2_interfaces::rv2_server_control::msgToOutSignal(info.channel_name, joy))
                cmd(sportClient_);
        };

        joyCfg.emergencyStopCb = [this](const rv2_interfaces::msg::ControlSignalInfo& info)
        {
            rv2_interfaces::rv2_server_control::makeStopSignal()(sportClient_);
            RCLCPP_WARN(this->get_logger(),
                "[E-STOP/joy] STOP_MOVE sent. Triggered by ch='%s'",
                info.channel_name.empty() ? "(no active sink)" : info.channel_name.c_str());
        };

        server_->registerTypeConfig(std::move(joyCfg));

        // Register Twist type config
        rv2_interfaces::rv2_server_control::ControlServer::TypeConfig<geometry_msgs::msg::Twist> twistCfg;
        twistCfg.outputCb = [this](const geometry_msgs::msg::Twist& twist,
                                   const rv2_interfaces::msg::ControlSignalInfo& info)
        {
            // cmd is empty when the Move values did not change since the last output.
            if (auto cmd = rv2_interfaces::rv2_server_control::msgToOutSignal(info.channel_name, twist))
                cmd(sportClient_);
        };

        twistCfg.emergencyStopCb = [this](const rv2_interfaces::msg::ControlSignalInfo& info)
        {
            rv2_interfaces::rv2_server_control::makeStopSignal()(sportClient_);
            RCLCPP_WARN(this->get_logger(),
                "[E-STOP/twist] STOP_MOVE sent. Triggered by ch='%s'",
                info.channel_name.empty() ? "(no active sink)" : info.channel_name.c_str());
        };

        server_->registerTypeConfig(std::move(twistCfg));

        RCLCPP_INFO(this->get_logger(),
            "ControlServerNode started. Listening on '%s/control_signal_reg'. "
            "Publishing Unitree sport requests on each received control message "
            "(event-triggered); safety watchdog at %ld ms interval.",
            serverName.c_str(), static_cast<long>(watchdogIntervalMs));
    }

private:
    SportClient sportClient_;
    std::unique_ptr<rv2_interfaces::rv2_server_control::ControlServer> server_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(ControlServerNode)

