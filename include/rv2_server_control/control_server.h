/** @file control_server.h
 * @brief R1 sink arbitration and event-driven control output.
 */
#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeindex>

#include <rv2_control_signal_transport/r1/control_signal_manager.h>

#include "control_signal_detect.h"

namespace rv2_interfaces::rv2_server_control
{

using ControlSignalInfo = r1::ControlSignalInfo;

/**
 * One independent active source per message type. First usable source wins;
 * a healthy source keeps ownership until a strictly higher-priority request,
 * explicit selection, emergency stop, or loss of signal. All priorities 1..100
 * have the same meaning; transport reserves no emergency-stop priority.
 *
 * User callbacks run serially, including watchdog callbacks, and must be short
 * and nonblocking. Queries may be called from callbacks. Stop the executor
 * before destroying its parent node; this object's destructor fences callbacks.
 */
class ControlServer
{
public:
    struct Config
    {
        std::string name = "control_server";
        int64_t watchdogIntervalNs = 50'000'000;
        r1::ManagerOptions managerOptions{r1::RetryPolicy::Recommended()};
    };

    template <typename MsgT> struct TypeConfig
    {
        std::function<bool(const MsgT&)> isEmergencyStop = rv2_interfaces::rv2_server_control::isEmergencyStop<MsgT>;
        std::function<bool(const MsgT&)> isRequestActive = rv2_interfaces::rv2_server_control::isRequestActive<MsgT>;
        std::function<void(const MsgT&, const ControlSignalInfo&)> outputCb;
        std::function<void(const ControlSignalInfo&)> emergencyStopCb;
        /// A new owner needs fresh output conversion state, even at equal values.
        std::function<void(const std::string&)> activeChangedCb;
    };

    ControlServer(rclcpp::Node* node, const Config& cfg) :
        core_(std::make_shared<Core>(node->get_logger()))
    {
        if (cfg.name.empty() || cfg.watchdogIntervalNs <= 0)
            throw std::invalid_argument("server name and positive watchdog interval are required");
        manager_ = std::make_unique<r1::ControlSignalManager>(node, cfg.name, cfg.managerOptions);
        core_->manager = manager_.get();
        timer_ = node->create_wall_timer(std::chrono::nanoseconds(cfg.watchdogIntervalNs),
                                         [core = core_]
                                         {
                                             std::lock_guard<std::recursive_mutex> lock(core->mutex);
                                             if (core->stopping)
                                                 return;
                                             for (auto& [type, state] : core->types)
                                                 watch(*core, type, state);
                                         });
    }

    ~ControlServer()
    {
        timer_->cancel();
        // Copied ROS callbacks hold Core, never this. The lock waits for any
        // callback already dispatching, then prevents all further user output.
        std::lock_guard<std::recursive_mutex> lock(core_->mutex);
        core_->stopping = true;
        core_->manager = nullptr;
    }

    ControlServer(const ControlServer&) = delete;
    ControlServer& operator=(const ControlServer&) = delete;

    template <typename MsgT> void registerTypeConfig(TypeConfig<MsgT> config)
    {
        const auto key = r1::ControlSignalFactory::Instance().typeKey(typeid(MsgT));
        if (key.empty())
            throw std::invalid_argument("unregistered R1 control message type");
        std::lock_guard<std::recursive_mutex> lock(core_->mutex);
        auto& state = core_->types[key];
        state.stop = config.emergencyStopCb;
        state.changed = config.activeChangedCb;
        const auto core = core_;
        manager_->registerCallback<MsgT>(
            [core, key, config = std::move(config)](const MsgT& message, const ControlSignalInfo& info)
            {
                std::lock_guard<std::recursive_mutex> lock(core->mutex);
                if (core->stopping)
                    return;
                const auto endpoint = core->manager->getSink(info.controller_name);
                const auto endpointState = endpoint.state();
                if (!endpoint.valid() || !endpointState || *endpointState == r1::ControlSignalState::DISCONNECTED)
                    return;
                auto& state = core->types.at(key);
                const auto now = Clock::now();
                auto& record = state.records[info.channel_name];
                const auto previousState = record.handle.state();
                const bool replaced = state.active == info.channel_name && !record.info.controller_name.empty() &&
                                      (!record.handle.valid() || !previousState ||
                                       *previousState == r1::ControlSignalState::DISCONNECTED);
                if (replaced)
                {
                    stopPrevious(state);
                    if (state.changed)
                        state.changed(info.channel_name);
                }
                record.info = info;
                // Refresh every time: a retry replaces the endpoint, so retaining
                // the old weak handle would break same-name reconnections.
                record.handle = endpoint;
                record.received = now;
                record.blocked = false;

                if (config.isEmergencyStop && config.isEmergencyStop(message))
                {
                    record.blocked = true;
                    select(*core, key, state, best(state, now), false);
                    state.hadOutput = false;
                    if (state.stop)
                        state.stop(info);
                    return;
                }
                if (config.isRequestActive && config.isRequestActive(message))
                {
                    const auto active = state.records.find(state.active);
                    if (active == state.records.end() || !usable(active->second, now) ||
                        info.priority > active->second.info.priority)
                        select(*core, key, state, info.channel_name);
                    return;
                }
                const auto active = state.records.find(state.active);
                if (active == state.records.end() || !usable(active->second, now))
                    select(*core, key, state, best(state, now));
                if (state.active != info.channel_name)
                    return;
                // A valid first message precedes the Manager's next ACTIVE tick.
                // Arm the loss edge immediately, including one-packet sessions.
                state.hadOutput = true;
                state.lastOutput = info;
                if (config.outputCb)
                    config.outputCb(message, info);
            });
    }

    template <typename MsgT> std::string getActiveSink() const
    {
        const auto key = r1::ControlSignalFactory::Instance().typeKey(typeid(MsgT));
        std::lock_guard<std::recursive_mutex> lock(core_->mutex);
        const auto it = core_->types.find(key);
        return it == core_->types.end() ? std::string{} : it->second.active;
    }

    template <typename MsgT> bool setActiveSink(const std::string& channel)
    {
        const auto key = r1::ControlSignalFactory::Instance().typeKey(typeid(MsgT));
        std::lock_guard<std::recursive_mutex> lock(core_->mutex);
        const auto it = core_->types.find(key);
        if (it == core_->types.end())
            return false;
        const auto record = it->second.records.find(channel);
        if (record == it->second.records.end() || !usable(record->second, Clock::now()))
            return false;
        select(*core_, key, it->second, channel);
        return true;
    }

    r1::ControlSignalManager& csm() { return *manager_; }
    const r1::ControlSignalManager& csm() const { return *manager_; }

private:
    using Clock = std::chrono::steady_clock;
    struct Record
    {
        ControlSignalInfo info;
        r1::SinkHandle handle;
        Clock::time_point received{};
        bool blocked = false;
    };
    struct TypeState
    {
        std::map<std::string, Record> records;
        std::string active;
        bool hadOutput = false;
        ControlSignalInfo lastOutput;
        std::function<void(const ControlSignalInfo&)> stop;
        std::function<void(const std::string&)> changed;
    };
    struct Core
    {
        explicit Core(rclcpp::Logger loggerValue) :
            logger(std::move(loggerValue))
        {
        }
        std::recursive_mutex mutex;
        bool stopping = false;
        r1::ControlSignalManager* manager = nullptr;
        rclcpp::Logger logger;
        std::map<std::string, TypeState> types;
    };

    static bool usable(const Record& record, Clock::time_point now)
    {
        if (record.blocked || !record.handle.valid())
            return false;
        const auto state = record.handle.state();
        if (!state || *state == r1::ControlSignalState::DISCONNECTED)
            return false;
        // Use local receipt time for the consumer's safety watchdog. Manager
        // state is committed on its tick and can still say INITIAL/TIMEOUT
        // immediately after a fresh receive, or ACTIVE just after its deadline.
        return record.info.timeout_ns == 0 || now - record.received < std::chrono::nanoseconds(record.info.timeout_ns);
    }

    static std::string best(const TypeState& state, Clock::time_point now)
    {
        std::string channel;
        int priority = -1;
        for (const auto& [name, record] : state.records)
        {
            if (usable(record, now) && record.info.priority > priority)
            {
                channel = name;
                priority = record.info.priority;
            }
        }
        return channel;
    }

    static void stopPrevious(TypeState& state)
    {
        if (!state.hadOutput)
            return;
        state.hadOutput = false;
        if (state.stop)
            state.stop(state.lastOutput);
    }

    static void
    select(Core& core, const std::string& type, TypeState& state, const std::string& channel, bool stopOutgoing = true)
    {
        if (state.active == channel)
            return;
        // Release the old command before waiting for the new owner's first
        // ordinary packet. Neutral fallback and held request-active packets
        // must never leave the robot executing the previous owner's velocity.
        if (stopOutgoing)
            stopPrevious(state);
        state.active = channel;
        RCLCPP_INFO(
            core.logger, "[R1/%s] active channel: %s", type.c_str(), channel.empty() ? "(none)" : channel.c_str());
        if (state.changed)
            state.changed(channel);
    }

    static void watch(Core& core, const std::string& type, TypeState& state)
    {
        const auto now = Clock::now();
        for (auto it = state.records.begin(); it != state.records.end();)
        {
            if (!it->second.handle.valid())
                it = state.records.erase(it);
            else
                ++it;
        }
        const auto active = state.records.find(state.active);
        if (active != state.records.end() && usable(active->second, now))
            return;
        select(core, type, state, best(state, now));
        if (state.active.empty() && state.hadOutput)
        {
            state.hadOutput = false;
            RCLCPP_WARN(core.logger, "[R1/%s] signal lost; emergency stop", type.c_str());
            if (state.stop)
                state.stop(state.lastOutput);
        }
    }

    std::shared_ptr<Core> core_;
    std::unique_ptr<r1::ControlSignalManager> manager_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rv2_interfaces::rv2_server_control
