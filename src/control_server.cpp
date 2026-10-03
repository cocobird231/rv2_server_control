/** @file control_server.cpp
 * @brief R1 Joy/Twist control server with observable Unitree sport output.
 */
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "rv2_server_control/control_server.h"
#include "rv2_server_control/output_message_convert.h"

class ControlServerNode : public rclcpp::Node
{
    using Server = rv2_interfaces::rv2_server_control::ControlServer;
    using Info = rv2_interfaces::r1::ControlSignalInfo;

public:
    explicit ControlServerNode(const rclcpp::NodeOptions& options) :
        Node("control_server", options),
        sportClient_(this)
    {
        Server::Config config;
        config.name = declare_parameter<std::string>("server_name", "control_server");
        config.watchdogIntervalNs = milliseconds("watchdog_interval_ms", 50);
        config.managerOptions.statusIntervalMs = milliseconds("status_timer_interval_ms", 100) / 1'000'000;
        config.managerOptions.masterName = declare_parameter<std::string>("master_name", "csm_master");
        config.managerOptions.csmTimeoutNs = milliseconds("csm_timeout_ms", 600);
        config.managerOptions.csmDisconnectTimeoutNs = milliseconds("csm_disconnect_timeout_ms", 6000);
        logOutput_ = declare_parameter<bool>("log_output", true);
        server_ = std::make_unique<Server>(this, config);
        registerType<sensor_msgs::msg::Joy>("joy");
        registerType<geometry_msgs::msg::Twist>("twist");
        RCLCPP_INFO(get_logger(),
                    "R1 control server '%s': register at %s/control_signal_manage; status at %s/status; "
                    "sport requests at /api/sport/request",
                    config.name.c_str(),
                    config.name.c_str(),
                    config.name.c_str());
    }

private:
    int64_t milliseconds(const std::string& name, int64_t value)
    {
        const auto result = declare_parameter<int64_t>(name, value);
        if (result <= 0 || result > std::numeric_limits<int64_t>::max() / 1'000'000)
            throw std::invalid_argument(name + " must be positive and representable in nanoseconds");
        return result * 1'000'000;
    }

    template <typename MsgT> void registerType(const std::string& type)
    {
        Server::TypeConfig<MsgT> config;
        config.outputCb = [this, type](const MsgT& message, const Info& info)
        {
            if (auto command = converter_.convert(info.channel_name, message))
            {
                command(sportClient_);
                if (logOutput_)
                    RCLCPP_INFO(get_logger(),
                                "[OUTPUT/%s] controller=%s channel=%s priority=%d",
                                type.c_str(),
                                info.controller_name.c_str(),
                                info.channel_name.c_str(),
                                info.priority);
            }
        };
        config.emergencyStopCb = [this, type](const Info& info)
        {
            converter_.reset();
            rv2_interfaces::rv2_server_control::makeStopSignal()(sportClient_);
            RCLCPP_WARN(get_logger(),
                        "[STOP/%s] StopMove controller=%s channel=%s",
                        type.c_str(),
                        info.controller_name.c_str(),
                        info.channel_name.c_str());
        };
        config.activeChangedCb = [this](const std::string&)
        {
            converter_.reset();
        };
        server_->registerTypeConfig(std::move(config));
    }

    SportClient sportClient_;
    rv2_interfaces::rv2_server_control::OutputMessageConverter converter_;
    bool logOutput_ = true;
    // Destroy first: fences callbacks before converter and SportClient vanish.
    std::unique_ptr<Server> server_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(ControlServerNode)
