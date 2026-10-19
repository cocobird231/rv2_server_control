/** @file control_server.cpp
 * @brief R1 arbitration, latched wireless interruption and operational services.
 */
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <r1_interfaces/srv/control_server_status_req.hpp>
#include <r1_interfaces/srv/reset_interrupt_req.hpp>
#include <r1_interfaces/srv/switch_controller_req.hpp>
#include <unitree_go/msg/wireless_controller.hpp>

#include "rv2_server_control/control_server.h"
#include "rv2_server_control/output_message_convert.h"

class ControlServerNode : public rclcpp::Node
{
    using Server = rv2_interfaces::rv2_server_control::ControlServer;
    using Info = rv2_interfaces::r1::ControlSignalInfo;
    using StatusService = r1_interfaces::srv::ControlServerStatusReq;
    using SwitchService = r1_interfaces::srv::SwitchControllerReq;
    using ResetService = r1_interfaces::srv::ResetInterruptReq;

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
        config.resetOutputCb = [this]
        {
            converter_.reset();
        };
        config.interruptionStopCb = [this]
        {
            sendCommand(rv2_interfaces::rv2_server_control::makeStopSignal(), "");
            RCLCPP_WARN(get_logger(), "[INTERRUPTED] wireless input; StopMove sent, manual reset required");
        };
        server_ = std::make_unique<Server>(this, config);
        // All command callbacks and this observer execute under the Core mutex.
        // Record only after our publisher succeeds, never from the shared topic.
        sportClient_.setRequestObserver(
            [this](const unitree_api::msg::Request& request)
            {
                server_->recordOutput(
                    now(), outputController_, request.header.identity.api_id, request.parameter, request.binary);
            });
        registerType<sensor_msgs::msg::Joy>("joy");
        registerType<geometry_msgs::msg::Twist>("twist");
        const auto wirelessTopic = declare_parameter<std::string>("wireless_topic", "/wirelesscontroller");
        wireless_ = create_subscription<unitree_go::msg::WirelessController>(
            wirelessTopic,
            rclcpp::SensorDataQoS(),
            [this](unitree_go::msg::WirelessController::ConstSharedPtr)
            {
                server_->interrupt();
            });
        statusService_ = create_service<StatusService>(
            config.name + "/control_server_status",
            [this](StatusService::Request::SharedPtr, StatusService::Response::SharedPtr response)
            {
                try
                {
                    response->status = server_->getStatus();
                    response->status.stamp = now();
                    response->response = StatusService::Response::RESPONSE_SUCCESS;
                }
                catch (const std::exception& error)
                {
                    response->response = StatusService::Response::RESPONSE_ERROR;
                    response->reason = error.what();
                }
            });
        switchService_ = create_service<SwitchService>(
            config.name + "/switch_controller",
            [this](SwitchService::Request::SharedPtr request, SwitchService::Response::SharedPtr response)
            {
                try
                {
                    response->response = server_->switchController(request->controller_name);
                    response->reason = switchReason(response->response);
                }
                catch (const std::exception& error)
                {
                    response->response = SwitchService::Response::RESPONSE_ERROR;
                    response->reason = error.what();
                }
            });
        resetService_ = create_service<ResetService>(
            config.name + "/reset_interrupt",
            [this](ResetService::Request::SharedPtr, ResetService::Response::SharedPtr response)
            {
                try
                {
                    server_->resetInterrupt();
                    response->response = ResetService::Response::RESPONSE_SUCCESS;
                }
                catch (const std::exception& error)
                {
                    response->response = ResetService::Response::RESPONSE_ERROR;
                    response->reason = error.what();
                }
            });
        RCLCPP_INFO(get_logger(),
                    "R1 control server '%s': services at %s/{control_server_status,switch_controller,reset_interrupt}; "
                    "wireless interrupt at %s; sport requests at /api/sport/request",
                    config.name.c_str(),
                    config.name.c_str(),
                    wirelessTopic.c_str());
    }

private:
    static std::string switchReason(int8_t response)
    {
        using Response = SwitchService::Response;
        switch (response)
        {
        case Response::RESPONSE_SUCCESS:
            return {};
        case Response::RESPONSE_NOT_FOUND:
            return "controller is not registered";
        case Response::RESPONSE_NOT_ACTIVE:
            return "controller is not ACTIVE with fresh usable input";
        case Response::RESPONSE_UNSUPPORTED_TYPE:
            return "controller type has no output handler";
        case Response::RESPONSE_INTERRUPTED:
            return "wireless interruption requires manual reset";
        default:
            return "controller switch failed";
        }
    }

    int64_t milliseconds(const std::string& name, int64_t value)
    {
        const auto result = declare_parameter<int64_t>(name, value);
        if (result <= 0 || result > std::numeric_limits<int64_t>::max() / 1'000'000)
            throw std::invalid_argument(name + " must be positive and representable in nanoseconds");
        return result * 1'000'000;
    }

    void sendCommand(const rv2_interfaces::rv2_server_control::SportClientCmd& command, const std::string& controller)
    {
        outputController_ = controller;
        command(sportClient_);
    }

    template <typename MsgT> void registerType(const std::string& type)
    {
        Server::TypeConfig<MsgT> config;
        config.outputCb = [this, type](const MsgT& message, const Info& info)
        {
            if (auto command = converter_.convert(info.channel_name, message))
            {
                sendCommand(command, info.controller_name);
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
            sendCommand(rv2_interfaces::rv2_server_control::makeStopSignal(), info.controller_name);
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
    std::string outputController_;
    bool logOutput_ = true;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_;
    rclcpp::Service<StatusService>::SharedPtr statusService_;
    rclcpp::Service<SwitchService>::SharedPtr switchService_;
    rclcpp::Service<ResetService>::SharedPtr resetService_;
    // Destroy first: fences callbacks before converter and SportClient vanish.
    std::unique_ptr<Server> server_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(ControlServerNode)
