/** @file control_server.h
 * @brief R1 global sink arbitration and latched control interruption.
 */
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <r1_interfaces/msg/control_server_status.hpp>
#include <r1_interfaces/srv/switch_controller_req.hpp>
#include <rv2_control_signal_transport/r1/control_signal_manager.h>

#include "control_signal_detect.h"

namespace rv2_interfaces::rv2_server_control
{

using ControlSignalInfo = r1::ControlSignalInfo;

/**
 * One active source across all message types. First usable source wins;
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
        std::function<void()> interruptionStopCb;
        std::function<void()> resetOutputCb;
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
        core_->status.server_name = cfg.name;
        core_->interruptionStop = cfg.interruptionStopCb;
        core_->resetOutput = cfg.resetOutputCb;
        manager_ = std::make_unique<r1::ControlSignalManager>(node, cfg.name, cfg.managerOptions);
        core_->manager = manager_.get();
        timer_ = node->create_wall_timer(std::chrono::nanoseconds(cfg.watchdogIntervalNs),
                                         [core = core_]
                                         {
                                             std::lock_guard<std::recursive_mutex> lock(core->mutex);
                                             if (!core->stopping)
                                                 watch(*core);
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
                const auto generation = core->resumeGeneration.load();
                std::lock_guard<std::recursive_mutex> lock(core->mutex);
                if (core->stopping || generation != core->resumeGeneration.load())
                    return;
                const auto endpoint = core->manager->getSink(info.controller_name);
                const auto endpointState = endpoint.state();
                if (!endpoint.valid() || !endpointState || *endpointState == r1::ControlSignalState::DISCONNECTED)
                    return;
                auto& state = core->types.at(key);
                const auto now = Clock::now();
                auto& record = state.records[info.channel_name];
                const auto previousState = record.handle.state();
                const Owner incoming{key, info.channel_name};
                const bool replaced = core->active == incoming && !record.info.controller_name.empty() &&
                                      (!record.handle.valid() || !previousState ||
                                       *previousState == r1::ControlSignalState::DISCONNECTED);
                if (replaced && !core->status.interrupted)
                {
                    stopPrevious(*core);
                    if (state.changed)
                        state.changed(info.channel_name);
                }
                record.info = info;
                // A retry replaces the endpoint; never retain an old weak handle.
                record.handle = endpoint;
                record.received = now;
                record.blocked = false;
                if (core->status.interrupted)
                    return;

                if (config.isEmergencyStop && config.isEmergencyStop(message))
                {
                    record.blocked = true;
                    select(*core, best(*core, now), false);
                    core->hadOutput = false;
                    if (state.stop)
                        state.stop(info);
                    return;
                }
                if (config.isRequestActive && config.isRequestActive(message))
                {
                    const auto active = activeRecord(*core);
                    if (!active || !usable(*active, now) || info.priority > active->info.priority)
                        select(*core, incoming);
                    return;
                }
                const auto active = activeRecord(*core);
                if (!active || !usable(*active, now))
                    select(*core, best(*core, now));
                if (!(core->active == incoming))
                    return;
                // Arm the loss edge before the Manager's first ACTIVE tick.
                // This is callback eligibility, not proof of a published command.
                core->hadOutput = true;
                core->lastOutput = info;
                if (config.outputCb)
                    config.outputCb(message, info);
            });
    }

    template <typename MsgT> std::string getActiveSink() const
    {
        const auto key = r1::ControlSignalFactory::Instance().typeKey(typeid(MsgT));
        std::lock_guard<std::recursive_mutex> lock(core_->mutex);
        return core_->active.type == key ? core_->active.channel : std::string{};
    }

    /// Legacy C++ selection accepts fresh INITIAL/TIMEOUT records, unlike RPC.
    template <typename MsgT> bool setActiveSink(const std::string& channel)
    {
        const auto key = r1::ControlSignalFactory::Instance().typeKey(typeid(MsgT));
        std::lock_guard<std::recursive_mutex> lock(core_->mutex);
        if (core_->stopping || core_->status.interrupted)
            return false;
        const auto it = core_->types.find(key);
        if (it == core_->types.end())
            return false;
        const auto record = it->second.records.find(channel);
        if (record == it->second.records.end() || !usable(record->second, Clock::now()))
            return false;
        select(*core_, {key, channel});
        return true;
    }

    /// Select a registered, transport-ACTIVE and locally fresh controller.
    int8_t switchController(const std::string& controllerName)
    {
        using Response = r1_interfaces::srv::SwitchControllerReq::Response;
        std::lock_guard<std::recursive_mutex> lock(core_->mutex);
        if (core_->stopping)
            return Response::RESPONSE_ERROR;
        if (core_->status.interrupted)
            return Response::RESPONSE_INTERRUPTED;
        const auto registered = manager_->getSinkInfoList();
        const auto found = std::find_if(registered.begin(),
                                        registered.end(),
                                        [&](const ControlSignalInfo& info)
                                        {
                                            return info.controller_name == controllerName;
                                        });
        if (found == registered.end())
            return Response::RESPONSE_NOT_FOUND;
        const auto type = core_->types.find(found->type);
        if (type == core_->types.end())
            return Response::RESPONSE_UNSUPPORTED_TYPE;
        const auto endpoint = manager_->getSink(controllerName);
        const auto state = endpoint.state();
        const auto record = type->second.records.find(found->channel_name);
        if (!endpoint.valid() || !state || *state != r1::ControlSignalState::ACTIVE ||
            record == type->second.records.end() || record->second.info != *found ||
            !usable(record->second, Clock::now()) || record->second.handle.state() != r1::ControlSignalState::ACTIVE)
            return Response::RESPONSE_NOT_ACTIVE;
        select(*core_, {found->type, found->channel_name});
        return Response::RESPONSE_SUCCESS;
    }

    /// Latch before stopping, so all later controller callbacks are suppressed.
    bool interrupt()
    {
        std::lock_guard<std::recursive_mutex> lock(core_->mutex);
        if (core_->stopping || core_->status.interrupted)
            return false;
        core_->status.interrupted = true;
        core_->hadOutput = false;
        select(*core_, {}, false);
        if (core_->resetOutput)
            core_->resetOutput();
        if (core_->interruptionStop)
            core_->interruptionStop();
        return true;
    }

    /// Release the latch without replaying any packet observed before reset.
    void resetInterrupt()
    {
        std::lock_guard<std::recursive_mutex> lock(core_->mutex);
        if (core_->stopping || !core_->status.interrupted)
            return;
        ++core_->resumeGeneration;
        for (auto& [type, state] : core_->types)
            state.records.clear();
        core_->active = {};
        core_->hadOutput = false;
        core_->status.interrupted = false;
        if (core_->resetOutput)
            core_->resetOutput();
    }

    /// Return a coherent local snapshot; the caller supplies its ROS stamp.
    r1_interfaces::msg::ControlServerStatus getStatus() const
    {
        std::lock_guard<std::recursive_mutex> lock(core_->mutex);
        auto status = core_->status;
        const auto active = activeRecord(*core_);
        status.active_controller_name = active ? active->info.controller_name : std::string{};
        if (core_->manager)
            status.registered_controllers = core_->manager->getSinkInfoList();
        return status;
    }

    /// Called synchronously after actual publication, including safety StopMove.
    void recordOutput(const builtin_interfaces::msg::Time& stamp,
                      const std::string& controllerName,
                      int64_t apiId,
                      const std::string& parameter,
                      const std::vector<uint8_t>& binary)
    {
        std::lock_guard<std::recursive_mutex> lock(core_->mutex);
        core_->status.has_output = true;
        core_->status.output_stamp = stamp;
        core_->status.output_controller_name = controllerName;
        core_->status.output_api_id = apiId;
        core_->status.output_parameter = parameter;
        core_->status.output_binary = binary;
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
    struct Owner
    {
        std::string type;
        std::string channel;
        bool operator==(const Owner& other) const { return type == other.type && channel == other.channel; }
    };
    struct TypeState
    {
        std::map<std::string, Record> records;
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
        std::atomic<uint64_t> resumeGeneration{0};
        r1::ControlSignalManager* manager = nullptr;
        rclcpp::Logger logger;
        std::map<std::string, TypeState> types;
        Owner active;
        bool hadOutput = false;
        ControlSignalInfo lastOutput;
        std::function<void()> interruptionStop;
        std::function<void()> resetOutput;
        r1_interfaces::msg::ControlServerStatus status;
    };

    static const Record* activeRecord(const Core& core)
    {
        const auto type = core.types.find(core.active.type);
        if (type == core.types.end())
            return nullptr;
        const auto record = type->second.records.find(core.active.channel);
        return record == type->second.records.end() ? nullptr : &record->second;
    }

    static bool usable(const Record& record, Clock::time_point now)
    {
        if (record.blocked || !record.handle.valid())
            return false;
        const auto state = record.handle.state();
        if (!state || *state == r1::ControlSignalState::DISCONNECTED)
            return false;
        // A fresh receive can precede the Manager's INITIAL/TIMEOUT -> ACTIVE
        // tick, and a deadline can precede its ACTIVE -> TIMEOUT tick.
        return record.info.timeout_ns == 0 || now - record.received < std::chrono::nanoseconds(record.info.timeout_ns);
    }

    static Owner best(const Core& core, Clock::time_point now)
    {
        Owner selected;
        int priority = -1;
        for (const auto& [type, state] : core.types)
            for (const auto& [channel, record] : state.records)
                if (usable(record, now) && (record.info.priority > priority ||
                                            (record.info.priority == priority && channel < selected.channel)))
                {
                    selected = {type, channel};
                    priority = record.info.priority;
                }
        return selected;
    }

    static void stopPrevious(Core& core)
    {
        if (!core.hadOutput)
            return;
        core.hadOutput = false;
        const auto type = core.types.find(core.lastOutput.type);
        if (type != core.types.end() && type->second.stop)
            type->second.stop(core.lastOutput);
    }

    static void select(Core& core, const Owner& owner, bool stopOutgoing = true)
    {
        if (core.active == owner)
            return;
        if (stopOutgoing)
            stopPrevious(core);
        const auto previous = core.active;
        core.active = owner;
        if (!previous.type.empty() && previous.type != owner.type)
        {
            RCLCPP_INFO(core.logger, "[R1/%s] active channel: (none)", previous.type.c_str());
            if (const auto& changed = core.types.at(previous.type).changed)
                changed({});
        }
        if (!owner.type.empty())
        {
            RCLCPP_INFO(core.logger, "[R1/%s] active channel: %s", owner.type.c_str(), owner.channel.c_str());
            if (const auto& changed = core.types.at(owner.type).changed)
                changed(owner.channel);
        }
    }

    static void watch(Core& core)
    {
        const auto now = Clock::now();
        for (auto& [type, state] : core.types)
            for (auto it = state.records.begin(); it != state.records.end();)
            {
                if (!it->second.handle.valid())
                    it = state.records.erase(it);
                else
                    ++it;
            }
        if (core.status.interrupted)
            return;
        const auto active = activeRecord(core);
        if (active && usable(*active, now))
            return;
        select(core, best(core, now));
        if (core.active.type.empty())
            stopPrevious(core);
    }

    std::shared_ptr<Core> core_;
    std::unique_ptr<r1::ControlSignalManager> manager_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rv2_interfaces::rv2_server_control
