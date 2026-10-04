/**
 * ControlServer — receives control signals from multiple remote Sources (via Sinks
 * managed by an internal ControlSignalManager) and drives one or more motion outputs.
 *
 * Design overview
 * ───────────────
 * ControlServer owns a ControlSignalManager (CSM) internally.  Remote Source CSMs
 * call the hosted `<name>/control_signal_reg` service; the internal CSM creates
 * matching typed Sinks automatically.  Users register per-message-type behaviour
 * via TypeConfig<msgT>, which describes:
 *
 *   - isEmergencyStop  : predicate detecting an e-stop command in a message.
 *   - isRequestActive  : predicate detecting a "request to become active" command.
 *   - outputCb         : called (event-triggered) on every message received from the
 *                        active Sink — not on a fixed timer.
 *   - emergencyStopCb  : called on e-stop or when the active Sink loses signal and no
 *                        usable Sink remains.
 *
 * Multiple message types (Joy, Twist, String …) can be active simultaneously.
 * Each type maintains its own active-sink selection independently.
 *
 * Usage example
 * ─────────────
 *   ControlServer cs(node, {"my_server", 100'000'000LL});
 *
 *   ControlServer::TypeConfig<Joy> joyCfg;
 *   joyCfg.isEmergencyStop = [](const Joy& j){ ... };
 *   joyCfg.isRequestActive = [](const Joy& j){ ... };
 *   joyCfg.outputCb        = [](const Joy& j, const ControlSignalInfo& i){ ... };
 *   joyCfg.emergencyStopCb = [](const ControlSignalInfo& i){ ... };
 *   cs.registerTypeConfig(std::move(joyCfg));
 */

#pragma once

#include "control_signal_detect.h"

#include <rv2_control_signal_transport/control_signal_manager.h>

#include <map>
#include <mutex>
#include <string>
#include <functional>
#include <memory>
#include <typeindex>
#include <chrono>
#include <vector>

namespace rv2_interfaces::rv2_server_control
{


// ══════════════════════════════════════════════════════════════════════════════
//  ControlServer
// ══════════════════════════════════════════════════════════════════════════════

class ControlServer
{
public:
    // ── Top-level configuration ───────────────────────────────────────────────

    struct Config
    {
        /** Unique name prefix for hosted service names (e.g. "control_server"). */
        std::string name;

        /**
         * Safety-watchdog poll period in nanoseconds.
         * Output itself is event-triggered (outputCb fires on each message from the
         * active Sink). This timer only enforces safety: it re-selects a fallback
         * Sink when the active one stops being usable and fires emergencyStopCb once
         * when no usable Sink remains.
         */
        int64_t watchdogIntervalNs = 100'000'000LL;  // 100 ms

        /**
         * Period of the CSM low-frequency status / disconnect timer in milliseconds.
         * This timer checks Source/Sink states and removes entries that have
         * been continuously in TIMEOUT for longer than their disconnect_timeout_ns.
         */
        int64_t statusTimerIntervalMs = 1000;  // 1 s
    };

    // ── Per-message-type configuration ───────────────────────────────────────

    /**
     * @brief Per-type behaviour configuration for ControlServer.
     *
     * Instantiate one TypeConfig<msgT> per message type you want ControlServer
     * to handle, then pass it to registerTypeConfig().
     *
     * @tparam msgT  ROS 2 message type (sensor_msgs::msg::Joy, geometry_msgs::msg::Twist …)
     */
    template<typename msgT>
    struct TypeConfig
    {
        /**
         * Returns true if msg represents an emergency-stop command.
         * nullptr = e-stop detection via message content is disabled for this type.
         */
        std::function<bool(const msgT&)> isEmergencyStop;

        /**
         * Returns true if msg represents a "request to become active" command.
         * nullptr = request-active detection is disabled for this type.
         */
        std::function<bool(const msgT&)> isRequestActive;

        /**
         * Called (event-triggered) on every message received from the active Sink,
         * with that message and the Sink's ControlSignalInfo.
         * Not called when the message is classified as e-stop or request-active,
         * nor for messages arriving from a non-active Sink.
         */
        std::function<void(const msgT&, const msg::ControlSignalInfo&)> outputCb;

        /**
         * Called when an e-stop is detected or when no ACTIVE Sink is available.
         * The info argument identifies the triggering Sink, or is default-constructed
         * if no Sink was active.
         */
        std::function<void(const msg::ControlSignalInfo&)> emergencyStopCb;

        TypeConfig()
        {
            isEmergencyStop = rv2_interfaces::rv2_server_control::isEmergencyStop<msgT>;
            isRequestActive = rv2_interfaces::rv2_server_control::isRequestActive<msgT>;
        }
    };

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param node  Parent ROS 2 node (must outlive this object and be spinning).
     * @param cfg   Server configuration.
     */
    ControlServer(rclcpp::Node* node, Config cfg)
        : node_(node)
        , cfg_(std::move(cfg))
        , csm_(node, cfg_.name, cfg_.statusTimerIntervalMs)
    {
        watchdogTimer_ = node_->create_wall_timer(
            std::chrono::nanoseconds(cfg_.watchdogIntervalNs),
            [this]() { _watchdogTimerCb(); });

        RCLCPP_INFO(node_->get_logger(),
            "[ControlServer:%s] Started. Services: '%s/control_signal_reg', "
            "'%s/control_signal_info_req'",
            cfg_.name.c_str(), cfg_.name.c_str(), cfg_.name.c_str());
    }

    ControlServer(const ControlServer&)            = delete;
    ControlServer& operator=(const ControlServer&) = delete;

    // ── Type registration ─────────────────────────────────────────────────────

    /**
     * @brief Register per-type behaviour and activate Sink management for msgT.
     *
     * This must be called before remote Sources of type msgT connect.
     * It registers a CSM sink-message callback so that each newly received message
     * immediately drives the active-sink selection logic for the type.
     *
     * Replaces any previously registered TypeConfig for this msgT.
     *
     * @tparam msgT  ROS 2 message type.
     * @param  tcfg  Per-type configuration.
     */
    template<typename msgT>
    void registerTypeConfig(TypeConfig<msgT> tcfg)
    {
        const std::type_index tid     = typeid(msgT);
        const std::string     typeStr = ControlSignalManager::typeKeyFor<msgT>();

        // Initialise per-type runtime state.
        {
            std::lock_guard<std::mutex> lk(typeMtx_);
            typeStates_.emplace(tid, TypeState{});
        }

        // Register the CSM sink-message callback.  Fires on every received message
        // from any Sink of this type.
        csm_.setSinkMsgCallback<msgT>(
            [this, tcfg, tid](const msgT& msg, const msg::ControlSignalInfo& info)
            {
                _onSinkMsg(msg, info, tid, tcfg);
            });

        // Install the per-type safety-watchdog handler.
        _installWatchdogHandler(tcfg, tid);

        RCLCPP_INFO(node_->get_logger(),
            "[ControlServer:%s] Registered TypeConfig for '%s'",
            cfg_.name.c_str(), typeStr.c_str());
    }

    // ── Manual active-sink control ────────────────────────────────────────────

    /**
     * @brief Manually select the active Sink for the given message type by channel_name.
     * @return false if no Sink with that channel name and matching type is tracked,
     *         or if the Sink has EMERGENCY_STOP priority (reserved, never an output source).
     */
    template<typename msgT>
    bool setActiveSink(const std::string& channelName)
    {
        const std::type_index tid = typeid(msgT);
        std::lock_guard<std::mutex> lk(typeMtx_);
        auto it = typeStates_.find(tid);
        if (it == typeStates_.end()) return false;

        auto& st = it->second;
        auto recIt = st.sinkRecords.find(channelName);
        if (recIt == st.sinkRecords.end()) return false;
        if (recIt->second.priority ==
            msg::ControlSignalConst::CONTROL_SIGNAL_PRIORITY_EMERGENCY_STOP)
        {
            RCLCPP_WARN(node_->get_logger(),
                "[ControlServer:%s] Refused manual activation of e-stop-priority sink '%s'",
                cfg_.name.c_str(), channelName.c_str());
            return false;
        }
        st.activeChannel = channelName;
        RCLCPP_INFO(node_->get_logger(),
            "[ControlServer:%s] Active sink for type manually set to '%s'",
            cfg_.name.c_str(), channelName.c_str());
        return true;
    }

    /**
     * @brief Returns the channel name of the currently active Sink for msgT ("" if none).
     */
    template<typename msgT>
    std::string getActiveSinkChannel() const
    {
        const std::type_index tid = typeid(msgT);
        std::lock_guard<std::mutex> lk(typeMtx_);
        auto it = typeStates_.find(tid);
        if (it == typeStates_.end()) return {};
        return it->second.activeChannel;
    }

    /** Channel name + priority of an active Sink (type-erased view). */
    struct ActiveSinkInfo
    {
        std::string channel;   ///< empty = no active sink
        int8_t      priority = -1;
    };

    /**
     * @brief Single-output-controller view: the currently active Sink across
     *        ALL registered types. When more than one type has an active
     *        selection, the highest-priority one is reported.
     */
    ActiveSinkInfo getActiveSinkAny() const
    {
        std::lock_guard<std::mutex> lk(typeMtx_);
        ActiveSinkInfo best;
        for (const auto& [tid, st] : typeStates_)
        {
            if (st.activeChannel.empty()) continue;
            auto it = st.sinkRecords.find(st.activeChannel);
            const int8_t pri =
                (it != st.sinkRecords.end()) ? it->second.priority : -1;
            if (best.channel.empty() || pri > best.priority)
                best = ActiveSinkInfo{st.activeChannel, pri};
        }
        return best;
    }

    /**
     * @brief Manually select the active Sink by channel name only, searching
     *        all registered types (single-output-controller semantics: the
     *        owning type gets the channel as active, every other type's
     *        active selection is cleared).
     * @return false if no type tracks the channel, or the Sink has
     *         EMERGENCY_STOP priority.
     */
    bool setActiveSinkByName(const std::string& channelName)
    {
        std::lock_guard<std::mutex> lk(typeMtx_);
        TypeState* owner = nullptr;
        for (auto& [tid, st] : typeStates_)
        {
            auto it = st.sinkRecords.find(channelName);
            if (it == st.sinkRecords.end()) continue;
            if (it->second.priority ==
                msg::ControlSignalConst::CONTROL_SIGNAL_PRIORITY_EMERGENCY_STOP)
            {
                RCLCPP_WARN(node_->get_logger(),
                    "[ControlServer:%s] Refused manual activation of e-stop-priority sink '%s'",
                    cfg_.name.c_str(), channelName.c_str());
                return false;
            }
            owner = &st;
            break;
        }
        if (!owner) return false;

        for (auto& [tid, st] : typeStates_)
            st.activeChannel = (&st == owner) ? channelName : std::string{};
        RCLCPP_INFO(node_->get_logger(),
            "[ControlServer:%s] Active output controller set to '%s' (by name)",
            cfg_.name.c_str(), channelName.c_str());
        return true;
    }

    /**
     * @brief Expose the underlying ControlSignalManager.
     */
    const ControlSignalManager& csm() const { return csm_; }
    ControlSignalManager&       csm()       { return csm_; }

private:
    // ── Per-type runtime state ────────────────────────────────────────────────

    struct SinkRecord
    {
        std::shared_ptr<BaseControlSignalSink> sink;
        int8_t priority = 0;
    };

    struct TypeState
    {
        std::map<std::string, SinkRecord> sinkRecords;  // key: channel_name
        std::string                       activeChannel;
        bool                              hadUsableSink = false;  // for e-stop edge detection
    };

    // ── Members ───────────────────────────────────────────────────────────────

    rclcpp::Node*        node_;
    Config               cfg_;
    ControlSignalManager csm_;

    mutable std::mutex                   typeMtx_;
    std::map<std::type_index, TypeState> typeStates_;

    mutable std::mutex                watchdogHandlerMtx_;
    std::vector<std::function<void()>> watchdogHandlers_;

    rclcpp::TimerBase::SharedPtr watchdogTimer_;

    // ── Helpers ───────────────────────────────────────────────────────────────

    /** Best (highest-priority ACTIVE or LOW_FREQ, non-EMERGENCY_STOP) channel.
     *  ACTIVE sinks are preferred over LOW_FREQ sinks at the same priority level.
     *  @param excludeChannel  channel to skip (e.g. the sink that just triggered e-stop). */
    static std::string _selectBestLocked(const TypeState& st,
                                         const std::string& excludeChannel = "")
    {
        std::string best;
        int8_t bestPri = -1;
        bool   bestIsActive = false;   // whether best candidate is ACTIVE (not just LOW_FREQ)

        for (const auto& [ch, rec] : st.sinkRecords)
        {
            if (!excludeChannel.empty() && ch == excludeChannel) continue;
            if (rec.priority == msg::ControlSignalConst::CONTROL_SIGNAL_PRIORITY_EMERGENCY_STOP)
                continue;
            if (!rec.sink) continue;

            const ControlSignalState s = rec.sink->getState();
            const bool isUsable = (s == ControlSignalState::ACTIVE ||
                                   s == ControlSignalState::LOW_FREQ);
            if (!isUsable) continue;

            const bool isActive = (s == ControlSignalState::ACTIVE);

            // Higher priority always wins; at equal priority, ACTIVE beats LOW_FREQ.
            if (rec.priority > bestPri ||
                (rec.priority == bestPri && isActive && !bestIsActive))
            {
                bestPri      = rec.priority;
                bestIsActive = isActive;
                best         = ch;
            }
        }
        return best;
    }

    /**
     * Called from the CSM sink-message callback on every received message.
     * Drives e-stop / request-active logic and keeps SinkRecords in sync.
     */
    template<typename msgT>
    void _onSinkMsg(const msgT& msg,
                    const msg::ControlSignalInfo& info,
                    std::type_index tid,
                    const TypeConfig<msgT>& tcfg)
    {
        // Ensure SinkRecord exists for this channel.
        {
            std::lock_guard<std::mutex> lk(typeMtx_);
            auto& st  = typeStates_[tid];
            auto& rec = st.sinkRecords[info.channel_name];
            if (!rec.sink)
            {
                // CSM sinks are keyed by controller_name (sinkRecords stays
                // keyed by channel_name for the arbitration logic).
                rec.sink     = csm_.getSink(info.controller_name);
                rec.priority = info.priority;

                // Auto-select: first Sink when no active exists, or a higher-priority
                // newcomer when the current active is not healthy (TIMEOUT / UNKNOWN).
                // While the current active Sink is ACTIVE it keeps its slot; a non-active
                // Sink must send a request-active message to take over.
                const bool isEstopPriority =
                    (rec.priority ==
                     msg::ControlSignalConst::CONTROL_SIGNAL_PRIORITY_EMERGENCY_STOP);
                if (st.activeChannel.empty() && !isEstopPriority)
                {
                    st.activeChannel = info.channel_name;
                }
                else if (!isEstopPriority)
                {
                    auto activeIt = st.sinkRecords.find(st.activeChannel);
                    // Only auto-promote when the current active Sink is NOT healthy.
                    const bool currentIsActive =
                        (activeIt != st.sinkRecords.end() &&
                         activeIt->second.sink &&
                         activeIt->second.sink->getState() == ControlSignalState::ACTIVE);
                    if (!currentIsActive &&
                        activeIt != st.sinkRecords.end() &&
                        rec.priority > activeIt->second.priority)
                    {
                        st.activeChannel = info.channel_name;
                        RCLCPP_INFO(node_->get_logger(),
                            "[ControlServer:%s] Higher-priority Sink '%s' (pri=%d) auto-selected",
                            cfg_.name.c_str(), info.channel_name.c_str(),
                            static_cast<int>(rec.priority));
                    }
                }
            }
            else if (rec.priority != info.priority)
            {
                // Keep the stored priority in sync with the descriptor so that
                // auto-promote / selection (stored rec.priority) and request-active
                // (fresh info.priority) always agree on a single source of truth.
                RCLCPP_INFO(node_->get_logger(),
                    "[ControlServer:%s] Sink '%s' priority updated %d -> %d",
                    cfg_.name.c_str(), info.channel_name.c_str(),
                    static_cast<int>(rec.priority), static_cast<int>(info.priority));
                rec.priority = info.priority;
            }
        }

        // E-stop check.
        if (tcfg.isEmergencyStop && tcfg.isEmergencyStop(msg))
        {
            std::string best;
            {
                std::lock_guard<std::mutex> lk(typeMtx_);
                auto& st         = typeStates_[tid];
                // Exclude the e-stop sender so it is not re-selected as active.
                st.activeChannel = _selectBestLocked(st, info.channel_name);
                best             = st.activeChannel;
                // Keep watchdog edge-state consistent: a usable fallback means we
                // still have signal; no fallback means signal is lost (e-stop just
                // fired here, so suppress a redundant watchdog e-stop).
                st.hadUsableSink = !best.empty();
            }
            RCLCPP_WARN(node_->get_logger(),
                "[ControlServer:%s] E-stop from '%s'. Fallback to '%s'.",
                cfg_.name.c_str(), info.channel_name.c_str(),
                best.empty() ? "(none)" : best.c_str());
            if (tcfg.emergencyStopCb) tcfg.emergencyStopCb(info);
            return;
        }

        // Request-active check (only for non-active Sinks).
        if (tcfg.isRequestActive && tcfg.isRequestActive(msg))
        {
            std::lock_guard<std::mutex> lk(typeMtx_);
            auto& st = typeStates_[tid];
            if (info.channel_name != st.activeChannel &&
                info.priority !=
                    msg::ControlSignalConst::CONTROL_SIGNAL_PRIORITY_EMERGENCY_STOP)
            {
                int8_t activePri = -1;
                auto   activeIt  = st.sinkRecords.find(st.activeChannel);
                if (activeIt != st.sinkRecords.end())
                    activePri = activeIt->second.priority;

                if (info.priority > activePri)
                {
                    st.activeChannel = info.channel_name;
                    RCLCPP_INFO(node_->get_logger(),
                        "[ControlServer:%s] Request-active: switched to '%s' (pri=%d > %d)",
                        cfg_.name.c_str(), info.channel_name.c_str(),
                        static_cast<int>(info.priority), static_cast<int>(activePri));
                }
            }
            return;  // request-active messages not forwarded to outputCb
        }

        // ── Event-triggered output ─────────────────────────────────────────────
        // A normal control message has arrived.  Drive the output immediately
        // (rather than via a periodic timer) when, and only when, it comes from
        // the Sink currently selected as active for this type.
        std::string activeChannel;
        {
            std::lock_guard<std::mutex> lk(typeMtx_);
            auto it = typeStates_.find(tid);
            if (it == typeStates_.end()) return;
            activeChannel = it->second.activeChannel;
        }
        if (info.channel_name != activeChannel) return;

        if (tcfg.outputCb) tcfg.outputCb(msg, info);
    }

    /** Called every watchdogIntervalNs — dispatches to all per-type handlers. */
    void _watchdogTimerCb()
    {
        std::lock_guard<std::mutex> lk(watchdogHandlerMtx_);
        for (auto& h : watchdogHandlers_) h();
    }

    /**
     * Install a per-type safety-watchdog handler.
     *
     * Output is event-triggered (see _onSinkMsg); this handler does NOT read or
     * forward messages.  It only enforces safety on the watchdog timer:
     *   - if the active Sink is no longer usable, re-select the next-best usable
     *     Sink as active (output then resumes on that Sink's next message);
     *   - if no usable Sink remains, fire emergencyStopCb once on the
     *     loss-of-signal edge.
     */
    template<typename msgT>
    void _installWatchdogHandler(TypeConfig<msgT> tcfg, std::type_index tid)
    {
        std::lock_guard<std::mutex> lk(watchdogHandlerMtx_);
        watchdogHandlers_.push_back([this, tcfg, tid]()
        {
            // Snapshot the relevant TypeState under lock.
            std::string activeChannel;
            TypeState   snapshot;
            {
                std::lock_guard<std::mutex> lk2(typeMtx_);
                auto it = typeStates_.find(tid);
                if (it == typeStates_.end()) return;
                snapshot      = it->second;
                activeChannel = it->second.activeChannel;
            }

            // Is the currently-active Sink still usable?
            auto activeIt = snapshot.sinkRecords.find(activeChannel);
            const ControlSignalState activeSt =
                (activeIt != snapshot.sinkRecords.end() && activeIt->second.sink)
                    ? activeIt->second.sink->getState()
                    : ControlSignalState::UNKNOWN;
            const bool activeUsable =
                (activeSt == ControlSignalState::ACTIVE ||
                 activeSt == ControlSignalState::LOW_FREQ);

            if (activeUsable)
            {
                std::lock_guard<std::mutex> lk2(typeMtx_);
                auto it = typeStates_.find(tid);
                if (it != typeStates_.end()) it->second.hadUsableSink = true;
                return;
            }

            // Active Sink lost — try to fall back to the next-best usable Sink.
            const std::string best = _selectBestLocked(snapshot);
            if (!best.empty())
            {
                std::lock_guard<std::mutex> lk2(typeMtx_);
                auto it = typeStates_.find(tid);
                if (it != typeStates_.end())
                {
                    if (it->second.activeChannel != best)
                        RCLCPP_INFO(node_->get_logger(),
                            "[ControlServer:%s] Active Sink lost — fell back to '%s'.",
                            cfg_.name.c_str(), best.c_str());
                    it->second.activeChannel = best;
                    it->second.hadUsableSink = true;
                }
                return;
            }

            // No usable Sink at all — fire e-stop once on the loss-of-signal edge.
            bool prevHad = false;
            {
                std::lock_guard<std::mutex> lk2(typeMtx_);
                auto it = typeStates_.find(tid);
                if (it == typeStates_.end()) return;
                prevHad                  = it->second.hadUsableSink;
                it->second.hadUsableSink = false;
            }
            if (prevHad)
            {
                RCLCPP_WARN(node_->get_logger(),
                    "[ControlServer:%s] Active Sink lost and no usable Sink remains — e-stop.",
                    cfg_.name.c_str());
                if (tcfg.emergencyStopCb)
                    tcfg.emergencyStopCb(msg::ControlSignalInfo{});
            }
        });
    }
};


} // namespace rv2_interfaces::rv2_server_control
