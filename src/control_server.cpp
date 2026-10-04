/**
 * This file implements the ControlServerNode, which receives control signals
 * and routes them to sport client commands via msgToOutSignal().
 *
 * The control server owns a ControlSignalManager (CSM) that manages remote
 * Source connections. Output is event-triggered: outputCb runs on each message
 * received from the active Sink. A low-frequency safety watchdog re-selects a
 * fallback Sink and issues an emergency stop when signal is lost. Per-message-type
 * conversion is delegated to output_message_convert.h so that outputCb only needs
 * to invoke the sport client.
 *
 * External intervention:
 *   Subscribes to the Unitree wireless controller topic (param 'wireless_topic',
 *   default '/wirelesscontroller'). Any incoming data means the physical remote
 *   is operating — the node raises the output-interrupted flag and suppresses
 *   ALL sport client output (including watchdog e-stop sends) until the flag
 *   is cleared via ControlServerReq REQUEST_CLEAR_INTERRUPT. New wireless data
 *   after a clear re-raises the flag immediately.
 *
 * ControlServerReq service ('<server_name>/control_server_req'):
 *   REQUEST_GET_INFO        — return a ControlServerStatus snapshot.
 *   REQUEST_SET_OUTPUT      — switch the output controller to <channel_name>;
 *                             allowed only when the requester's priority
 *                             (config/requester_priority.yaml) is strictly
 *                             greater than the current output controller's
 *                             registered priority.
 *   REQUEST_CLEAR_INTERRUPT — clear the external-intervention flag; any
 *                             requester listed in requester_priority.yaml may
 *                             perform this.
 *
 * A ControlServerStatus snapshot is also published periodically on
 * '<server_name>/control_server_status'.
 */

#include <algorithm>
#include <atomic>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <unitree_go/msg/wireless_controller.hpp>
#include <rv2_interfaces/msg/control_server_status.hpp>
#include <rv2_interfaces/msg/service_response_status_const.hpp>
#include <rv2_interfaces/srv/control_server_req.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "rv2_server_control/control_server.h"
#include "rv2_server_control/control_signal_detect.h"
#include "rv2_server_control/output_message_convert.h"
#include "rclcpp_components/register_node_macro.hpp"



// ── ControlServerNode ─────────────────────────────────────────────────────────

class ControlServerNode : public rclcpp::Node
{
    using ControlServerStatus = rv2_interfaces::msg::ControlServerStatus;
    using ControlServerReq    = rv2_interfaces::srv::ControlServerReq;
    using SrvResConst         = rv2_interfaces::msg::ServiceResponseStatusConst;
    using WirelessController  = unitree_go::msg::WirelessController;

public:
    explicit ControlServerNode(const rclcpp::NodeOptions & options)
        : Node("control_server", options)
        , sportClient_(this)
    {
        // Declare parameters
        this->declare_parameter<std::string>("server_name",              "control_server");
        this->declare_parameter<int64_t>    ("watchdog_interval_ms",     100);
        this->declare_parameter<int64_t>    ("status_timer_interval_ms", 1000);
        this->declare_parameter<std::string>("wireless_topic",           "/wirelesscontroller");
        this->declare_parameter<std::string>("requester_priority_file",  "");
        this->declare_parameter<int64_t>    ("status_publish_interval_ms", 1000);

        serverName_ = this->get_parameter("server_name").as_string();
        const int64_t watchdogIntervalMs = this->get_parameter("watchdog_interval_ms").as_int();
        const int64_t statusTimerMs      = this->get_parameter("status_timer_interval_ms").as_int();
        const std::string wirelessTopic  = this->get_parameter("wireless_topic").as_string();
        const int64_t statusPubMs        = this->get_parameter("status_publish_interval_ms").as_int();

        _loadRequesterPriorities(
            this->get_parameter("requester_priority_file").as_string());

        // Build ControlServer config
        rv2_interfaces::rv2_server_control::ControlServer::Config cfg;
        cfg.name                  = serverName_;
        cfg.watchdogIntervalNs    = watchdogIntervalMs * 1'000'000LL;
        cfg.statusTimerIntervalMs = statusTimerMs;

        server_ = std::make_unique<rv2_interfaces::rv2_server_control::ControlServer>(this, std::move(cfg));

        // Register Joy type config
        rv2_interfaces::rv2_server_control::ControlServer::TypeConfig<sensor_msgs::msg::Joy> joyCfg;
        joyCfg.outputCb = [this](const sensor_msgs::msg::Joy& joy,
                                 const rv2_interfaces::msg::ControlSignalInfo& info)
        {
            if (outputInterrupted_.load()) return;  // external intervention active
            // cmd is empty when the message carries no new event (e.g. idle joystick).
            if (auto cmd = rv2_interfaces::rv2_server_control::msgToOutSignal(info.channel_name, joy))
                cmd(sportClient_);
        };

        joyCfg.emergencyStopCb = [this](const rv2_interfaces::msg::ControlSignalInfo& info)
        {
            if (outputInterrupted_.load()) return;  // remote operator owns the robot
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
            if (outputInterrupted_.load()) return;
            // cmd is empty when the Move values did not change since the last output.
            if (auto cmd = rv2_interfaces::rv2_server_control::msgToOutSignal(info.channel_name, twist))
                cmd(sportClient_);
        };

        twistCfg.emergencyStopCb = [this](const rv2_interfaces::msg::ControlSignalInfo& info)
        {
            if (outputInterrupted_.load()) return;
            rv2_interfaces::rv2_server_control::makeStopSignal()(sportClient_);
            RCLCPP_WARN(this->get_logger(),
                "[E-STOP/twist] STOP_MOVE sent. Triggered by ch='%s'",
                info.channel_name.empty() ? "(no active sink)" : info.channel_name.c_str());
        };

        server_->registerTypeConfig(std::move(twistCfg));

        // ── External intervention: wireless controller topic ──────────────────
        wirelessSub_ = this->create_subscription<WirelessController>(
            wirelessTopic, rclcpp::QoS(10),
            [this](const WirelessController::SharedPtr) { _onWirelessData(); });

        // ── ControlServerReq service ──────────────────────────────────────────
        reqSrv_ = this->create_service<ControlServerReq>(
            serverName_ + "/control_server_req",
            [this](const std::shared_ptr<ControlServerReq::Request> req,
                   std::shared_ptr<ControlServerReq::Response> res)
            { _onControlServerReq(req, res); });

        // ── Status topic ──────────────────────────────────────────────────────
        statusPub_ = this->create_publisher<ControlServerStatus>(
            serverName_ + "/control_server_status", rclcpp::QoS(10));
        statusTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(statusPubMs),
            [this]() { statusPub_->publish(_makeStatus()); });

        RCLCPP_INFO(this->get_logger(),
            "ControlServerNode started. Listening on '%s/control_signal_reg'. "
            "Request service: '%s/control_server_req'. Wireless intervention "
            "topic: '%s'. Safety watchdog at %ld ms interval.",
            serverName_.c_str(), serverName_.c_str(), wirelessTopic.c_str(),
            static_cast<long>(watchdogIntervalMs));
    }

private:
    // ── Priority tables (requester_priority.yaml) ─────────────────────────────

    /**
     * Load the two priority tables from the YAML file:
     *   requester_priority:   <identity>: <priority int>   (GUIs / service requesters)
     *   controller_priority:  <controller>: <priority int> (named controllers)
     * Empty path falls back to the installed package config.
     *
     * requesterPriority_ holds the union of both tables (any listed identity
     * may act as a ControlServerReq requester); controllerPriority_ holds
     * only the controller table (used by the sink registration policy).
     */
    void _loadRequesterPriorities(std::string path)
    {
        if (path.empty())
        {
            path = ament_index_cpp::get_package_share_directory("rv2_server_control")
                   + "/config/requester_priority.yaml";
        }
        try
        {
            const YAML::Node root = YAML::LoadFile(path);
            const YAML::Node req  = root["requester_priority"];
            const YAML::Node ctrl = root["controller_priority"];
            if (!req || !req.IsMap())
                throw std::runtime_error("missing 'requester_priority' map");
            for (const auto& kv : req)
                requesterPriority_[kv.first.as<std::string>()] = kv.second.as<int>();
            // controller_priority is tolerated as missing (pre-refactor files):
            // requester permissions keep working; only the named-controller
            // override / reservation rules are then inactive.
            if (ctrl && ctrl.IsMap())
            {
                for (const auto& kv : ctrl)
                {
                    controllerPriority_[kv.first.as<std::string>()] = kv.second.as<int>();
                    requesterPriority_[kv.first.as<std::string>()]  = kv.second.as<int>();
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                    "'%s' has no 'controller_priority' map (pre-refactor file?). "
                    "Named-controller priority overrides are disabled.",
                    path.c_str());
            }
            RCLCPP_INFO(this->get_logger(),
                "Loaded %zu requester + %zu controller priorities from '%s'.",
                requesterPriority_.size() - controllerPriority_.size(),
                controllerPriority_.size(), path.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to load requester priority file '%s': %s. "
                "All mutating ControlServerReq requests will be rejected.",
                path.c_str(), e.what());
            requesterPriority_.clear();
            controllerPriority_.clear();
        }
    }

    // ── External intervention ─────────────────────────────────────────────────

    void _onWirelessData()
    {
        if (outputInterrupted_.exchange(true)) return;  // already interrupted

        interruptStampNs_ = this->now().nanoseconds();
        RCLCPP_WARN(this->get_logger(),
            "[INTERVENTION] Wireless controller data detected — output "
            "interrupted. Clear via ControlServerReq REQUEST_CLEAR_INTERRUPT.");
        // One StopMove on the rising edge so the robot does not keep executing
        // the last velocity command while the operator takes over.
        rv2_interfaces::rv2_server_control::makeStopSignal()(sportClient_);
        statusPub_->publish(_makeStatus());
    }

    // ── Status snapshot ───────────────────────────────────────────────────────

    ControlServerStatus _makeStatus()
    {
        ControlServerStatus st;
        st.server_name = serverName_;

        auto& csm = server_->csm();
        st.sink_list = csm.getSinkInfoList();
        st.sink_states.reserve(st.sink_list.size());
        for (const auto& info : st.sink_list)
            st.sink_states.push_back(
                static_cast<int8_t>(csm.getSinkState(info.controller_name)));

        // Active output controller: channel from the arbitration, canonical
        // controller name resolved from the (canonicalised) sink list.
        st.active_channel = server_->getActiveSinkAny().channel;
        st.active_controller_name = "";
        if (!st.active_channel.empty())
        {
            for (const auto& s : st.sink_list)
                if (s.channel_name == st.active_channel)
                {
                    st.active_controller_name =
                        s.controller_name.empty() ? s.channel_name
                                                  : s.controller_name;
                    break;
                }
        }

        st.output_interrupted       = outputInterrupted_.load();
        st.interrupt_source_channel =
            st.output_interrupted ? WIRELESS_REQUESTER : "";
        st.interrupt_stamp_ns       =
            st.output_interrupted ? interruptStampNs_ : 0;
        return st;
    }

    // ── ControlServerReq service callback ─────────────────────────────────────

    void _onControlServerReq(const std::shared_ptr<ControlServerReq::Request> req,
                             std::shared_ptr<ControlServerReq::Response> res)
    {
        res->response = SrvResConst::SRV_RES_SUCCESS;
        res->reason.clear();

        switch (req->request_type)
        {
            case ControlServerReq::Request::REQUEST_GET_INFO:
                break;  // snapshot below is the whole answer

            case ControlServerReq::Request::REQUEST_SET_OUTPUT:
                _handleSetOutput(*req, *res);
                break;

            case ControlServerReq::Request::REQUEST_CLEAR_INTERRUPT:
                _handleClearInterrupt(*req, *res);
                break;

            default:
                res->response = SrvResConst::SRV_RES_ERROR;
                res->reason   = "unknown request_type " +
                                std::to_string(req->request_type);
                break;
        }

        res->status = _makeStatus();
    }

    void _handleSetOutput(const ControlServerReq::Request& req,
                          ControlServerReq::Response& res)
    {
        const auto priIt = requesterPriority_.find(req.requester);
        if (priIt == requesterPriority_.end())
        {
            res.response = SrvResConst::SRV_RES_ERROR;
            res.reason   = "unknown requester '" + req.requester + "'";
            return;
        }

        // Permission: requester priority must be STRICTLY greater than the
        // current output controller's registered priority.
        const auto active = server_->getActiveSinkAny();
        if (!active.channel.empty() &&
            priIt->second <= static_cast<int>(active.priority))
        {
            res.response = SrvResConst::SRV_RES_ERROR;
            res.reason   = "insufficient permission: requester '" +
                           req.requester + "' (pri=" +
                           std::to_string(priIt->second) +
                           ") <= active controller '" + active.channel +
                           "' (pri=" + std::to_string(active.priority) + ")";
            RCLCPP_WARN(this->get_logger(), "[REQ/SET_OUTPUT] %s", res.reason.c_str());
            return;
        }

        // Resolve the canonical controller name to its transport channel.
        std::string channel;
        for (const auto& s : server_->csm().getSinkInfoList())
        {
            const std::string name =
                s.controller_name.empty() ? s.channel_name : s.controller_name;
            if (name == req.controller_name) { channel = s.channel_name; break; }
        }
        if (channel.empty() || !server_->setActiveSinkByName(channel))
        {
            res.response = SrvResConst::SRV_RES_ERROR;
            res.reason   = "unknown or non-activatable controller '" +
                           req.controller_name + "'";
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "[REQ/SET_OUTPUT] requester '%s' switched output controller to "
            "'%s' (channel '%s').",
            req.requester.c_str(), req.controller_name.c_str(), channel.c_str());
    }

    void _handleClearInterrupt(const ControlServerReq::Request& req,
                               ControlServerReq::Response& res)
    {
        // Nothing outranks the wireless intervention, so any requester listed
        // in requester_priority.yaml may clear the flag.
        if (requesterPriority_.find(req.requester) == requesterPriority_.end())
        {
            res.response = SrvResConst::SRV_RES_ERROR;
            res.reason   = "unknown requester '" + req.requester + "'";
            return;
        }

        if (!outputInterrupted_.exchange(false))
        {
            res.response = SrvResConst::SRV_RES_IGNORED;
            res.reason   = "no interruption was active";
            return;
        }

        interruptStampNs_ = 0;
        RCLCPP_WARN(this->get_logger(),
            "[REQ/CLEAR_INTERRUPT] requester '%s' cleared the external "
            "intervention flag — output resumed.", req.requester.c_str());
    }

    // ── Members ───────────────────────────────────────────────────────────────

    static constexpr const char* WIRELESS_REQUESTER = "wireless";

    SportClient sportClient_;
    std::unique_ptr<rv2_interfaces::rv2_server_control::ControlServer> server_;

    std::string serverName_;
    std::map<std::string, int> requesterPriority_;   ///< union of both yaml tables
    std::map<std::string, int> controllerPriority_;  ///< controller_priority table only

    std::atomic<bool> outputInterrupted_{false};
    int64_t           interruptStampNs_ = 0;

    rclcpp::Subscription<WirelessController>::SharedPtr wirelessSub_;
    rclcpp::Service<ControlServerReq>::SharedPtr        reqSrv_;
    rclcpp::Publisher<ControlServerStatus>::SharedPtr   statusPub_;
    rclcpp::TimerBase::SharedPtr                        statusTimer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(ControlServerNode)
