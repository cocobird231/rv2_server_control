#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "rv2_server_control/control_server.h"
#include "rv2_server_control/output_message_convert.h"

using namespace std::chrono_literals;
namespace transport = rv2_interfaces::r1;
using Server = rv2_interfaces::rv2_server_control::ControlServer;
using Joy = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;
using SwitchResponse = r1_interfaces::srv::SwitchControllerReq::Response;

namespace
{
bool waitFor(const std::function<bool()>& predicate, std::chrono::milliseconds limit = 4000ms)
{
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (predicate())
            return true;
        std::this_thread::sleep_for(10ms);
    }
    return predicate();
}

Joy joy(float value = 0.5f)
{
    Joy result;
    result.axes = {value, 0, 1, 0, 0, 1};
    result.buttons.assign(12, 0);
    return result;
}

class ServerIntegration : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        static std::atomic<unsigned> serial{0};
        prefix_ = "server_case_" + std::to_string(++serial);
        sinkNode_ = std::make_shared<rclcpp::Node>(prefix_ + "_sink");
        sourceNode_ = std::make_shared<rclcpp::Node>(prefix_ + "_source");
        auto config = serverConfig();
        config.name = prefix_;
        config.interruptionStopCb = [this]
        {
            ++interruptionStops_;
            server_->recordOutput(sinkNode_->now(), "", 1003, "", {});
        };
        config.resetOutputCb = [this]
        {
            ++resets_;
            if (resetHook_)
                resetHook_();
        };
        server_ = std::make_unique<Server>(sinkNode_.get(), config);
        transport::ManagerOptions options(transport::RetryPolicy(20, 100, 0, 20, 4));
        options.statusIntervalMs = 20;
        sources_ = std::make_unique<transport::ControlSignalManager>(sourceNode_.get(), prefix_ + "_source", options);
        setJoyConfig();
        Server::TypeConfig<Twist> twistConfig;
        twistConfig.outputCb = [this](const Twist&, const transport::ControlSignalInfo&)
        {
            ++twistOutputs_;
        };
        twistConfig.emergencyStopCb = [this](const transport::ControlSignalInfo&)
        {
            ++twistStops_;
        };
        server_->registerTypeConfig(std::move(twistConfig));
        executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 4);
        executor_->add_node(sinkNode_);
        executor_->add_node(sourceNode_);
        spin_ = std::thread(
            [this]
            {
                executor_->spin();
            });
    }

    virtual Server::Config serverConfig() const
    {
        Server::Config config;
        config.watchdogIntervalNs = 20'000'000;
        config.managerOptions.statusIntervalMs = 20;
        return config;
    }

    void TearDown() override
    {
        executor_->cancel();
        if (spin_.joinable())
            spin_.join();
        server_.reset();
        sources_.reset();
        executor_.reset();
        sinkNode_.reset();
        sourceNode_.reset();
    }

    void setJoyConfig()
    {
        Server::TypeConfig<Joy> config;
        config.outputCb = [this](const Joy&, const transport::ControlSignalInfo& info)
        {
            std::lock_guard<std::mutex> lock(outputMutex_);
            outputs_.push_back(info.channel_name);
        };
        config.emergencyStopCb = [this](const transport::ControlSignalInfo&)
        {
            ++stops_;
        };
        server_->registerTypeConfig(std::move(config));
    }

    transport::SourceHandle add(const std::string& suffix,
                                int priority = 50,
                                int64_t timeoutMs = 600,
                                int64_t disconnectMs = 0,
                                const std::string& type = "joy",
                                const std::string& mode = "topic")
    {
        transport::ControlSignalInfo info;
        info.controller_name = prefix_ + "_" + suffix;
        info.channel_name = channel(suffix);
        info.target_manager_name = prefix_;
        info.type = type;
        info.mode = mode;
        info.priority = static_cast<int8_t>(priority);
        info.timeout_ns = timeoutMs * 1'000'000;
        info.disconnect_timeout_ns = disconnectMs * 1'000'000;
        transport::ControlSignalManager::RegisterResult result{transport::RegisterError::INVALID_CONTEXT, {}};
        EXPECT_TRUE(waitFor(
            [&]
            {
                result = sources_->registerSource(info, 1000);
                return result.code != transport::RegisterError::INVALID_CONTEXT;
            }));
        EXPECT_TRUE(result.handle.valid()) << static_cast<int>(result.code);
        EXPECT_TRUE(waitFor(
            [&]
            {
                return result.handle.ready();
            }));
        return result.handle;
    }

    template <typename Message>
    bool sendUntil(transport::SourceHandle& source, const Message& message, const std::function<bool()>& done)
    {
        return waitFor(
            [&]
            {
                source.send(message);
                return done();
            });
    }

    std::string channel(const std::string& suffix) const { return "/" + prefix_ + "/" + suffix; }
    size_t count(const std::string& suffix) const
    {
        std::lock_guard<std::mutex> lock(outputMutex_);
        return static_cast<size_t>(std::count(outputs_.begin(), outputs_.end(), channel(suffix)));
    }

    std::string prefix_;
    std::shared_ptr<rclcpp::Node> sinkNode_, sourceNode_;
    std::unique_ptr<Server> server_;
    std::unique_ptr<transport::ControlSignalManager> sources_;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_;
    mutable std::mutex outputMutex_;
    std::vector<std::string> outputs_;
    std::atomic<int> stops_{0}, twistOutputs_{0}, twistStops_{0};
    std::atomic<int> interruptionStops_{0}, resets_{0};
    std::function<void()> resetHook_;
};

TEST_F(ServerIntegration, Priority100IsNormalAndOwnershipIsExclusiveAcrossTypes)
{
    auto source = add("joy", 100, 3000);
    ASSERT_TRUE(sendUntil(source,
                          joy(),
                          [&]
                          {
                              return count("joy") > 0;
                          }));
    EXPECT_EQ(server_->getActiveSink<Joy>(), channel("joy"));
    EXPECT_EQ(stops_, 0);
    // Service replies fence every receive callback before the switch. A topic
    // stream may still have legitimate queued packets arriving after selection.
    auto twist = add("twist", 50, 3000, 0, "twist", "service");
    Twist message;
    message.linear.x = 0.3;
    ASSERT_TRUE(sendUntil(twist,
                          message,
                          [&]
                          {
                              return server_->csm().getSinkState(twist.controllerName()) ==
                                     transport::ControlSignalState::ACTIVE;
                          }));
    EXPECT_EQ(server_->getActiveSink<Joy>(), channel("joy"));
    EXPECT_TRUE(server_->getActiveSink<Twist>().empty());
    EXPECT_EQ(twistOutputs_, 0);
    ASSERT_EQ(server_->switchController(twist.controllerName()), SwitchResponse::RESPONSE_SUCCESS);
    EXPECT_TRUE(server_->getActiveSink<Joy>().empty());
    EXPECT_EQ(server_->getActiveSink<Twist>(), channel("twist"));
    EXPECT_EQ(stops_, 1);
    EXPECT_EQ(twistOutputs_, 0);  // manual selection never replays its cached data
    ASSERT_TRUE(sendUntil(twist,
                          message,
                          [&]
                          {
                              return twistOutputs_ > 0;
                          }));
}

TEST_F(ServerIntegration, TwistTopicInterruptResetAndSameValueRecoveryDoNotReplayPausedInput)
{
    auto converter = std::make_shared<rv2_interfaces::rv2_server_control::OutputMessageConverter>();
    auto converted = std::make_shared<std::atomic<int>>(0);
    resetHook_ = [converter]
    {
        converter->reset();
    };
    Server::TypeConfig<Twist> config;
    config.outputCb = [converter, converted](const Twist& message, const transport::ControlSignalInfo& info)
    {
        if (converter->convert(info.channel_name, message))
            ++*converted;
    };
    config.emergencyStopCb = [this, converter](const transport::ControlSignalInfo&)
    {
        ++twistStops_;
        converter->reset();
    };
    config.activeChangedCb = [converter](const std::string&)
    {
        converter->reset();
    };
    server_->registerTypeConfig(std::move(config));
    auto source = add("twist_topic", 50, 300, 0, "twist", "topic");
    Twist message;
    message.linear.x = 0.4;
    message.angular.z = -0.2;
    ASSERT_TRUE(sendUntil(source,
                          message,
                          [&]
                          {
                              return converted->load() == 1;
                          }));
    EXPECT_EQ(server_->getActiveSink<Twist>(), channel("twist_topic"));
    EXPECT_TRUE(server_->getActiveSink<Joy>().empty());
    ASSERT_TRUE(server_->interrupt());
    EXPECT_EQ(interruptionStops_, 1);
    ASSERT_EQ(source.send(message), transport::SendResult::OK);

    // Stop publishing and observe actual transport inactivity before reset;
    // this separates cached-data replay from ordinary queued topic delivery.
    ASSERT_TRUE(waitFor(
        [&]
        {
            return server_->csm().getSinkState(source.controllerName()) == transport::ControlSignalState::TIMEOUT;
        }));
    EXPECT_EQ(converted->load(), 1);
    EXPECT_EQ(twistStops_, 0);
    server_->resetInterrupt();
    std::this_thread::sleep_for(100ms);
    EXPECT_TRUE(server_->getActiveSink<Twist>().empty());
    EXPECT_EQ(converted->load(), 1);
    EXPECT_EQ(interruptionStops_, 1);
    ASSERT_TRUE(sendUntil(source,
                          message,
                          [&]
                          {
                              return converted->load() == 2;
                          }));
    EXPECT_EQ(server_->getActiveSink<Twist>(), channel("twist_topic"));

    // Silence after recovery produces one loss stop and no cached movement.
    ASSERT_TRUE(waitFor(
        [&]
        {
            return twistStops_ == 1;
        }));
    EXPECT_TRUE(server_->getActiveSink<Twist>().empty());
    std::this_thread::sleep_for(150ms);
    EXPECT_EQ(converted->load(), 2);
    EXPECT_EQ(twistStops_, 1);
}

TEST_F(ServerIntegration, RequestActiveRequiresHigherPriorityAndIsNotOutput)
{
    auto low = add("low", 20, 3000);
    ASSERT_TRUE(sendUntil(low,
                          joy(),
                          [&]
                          {
                              return count("low") > 0;
                          }));
    auto equal = add("equal", 20, 3000);
    auto high = add("high", 80, 3000);
    auto request = joy();
    request.buttons[11] = 1;
    for (int i = 0; i < 10; ++i)
    {
        equal.send(request);
        high.send(joy());
        std::this_thread::sleep_for(20ms);
    }
    EXPECT_EQ(server_->getActiveSink<Joy>(), channel("low"));
    EXPECT_EQ(count("high"), 0u);
    EXPECT_EQ(count("equal"), 0u);
    ASSERT_TRUE(sendUntil(high,
                          request,
                          [&]
                          {
                              return server_->getActiveSink<Joy>() == channel("high");
                          }));
    EXPECT_EQ(count("high"), 0u);
    EXPECT_EQ(stops_, 1);  // held takeover request must stop the previous command
    ASSERT_TRUE(sendUntil(high,
                          joy(),
                          [&]
                          {
                              return count("high") > 0;
                          }));
}

TEST_F(ServerIntegration, ActiveTimeoutFallsBackThenStopsOnlyOnce)
{
    auto active = add("active", 80, 200);
    ASSERT_TRUE(sendUntil(active,
                          joy(),
                          [&]
                          {
                              return count("active") > 0;
                          }));
    auto fallback = add("fallback", 20, 500);
    ASSERT_TRUE(sendUntil(fallback,
                          joy(0.3f),
                          [&]
                          {
                              return count("fallback") > 0;
                          }));
    EXPECT_EQ(server_->getActiveSink<Joy>(), channel("fallback"));
    EXPECT_EQ(stops_, 1);  // stop old command on fallback ownership transfer
    ASSERT_TRUE(waitFor(
        [&]
        {
            return stops_ == 2;
        }));
    EXPECT_TRUE(server_->getActiveSink<Joy>().empty());
    std::this_thread::sleep_for(200ms);
    EXPECT_EQ(stops_, 2);
    EXPECT_FALSE(server_->setActiveSink<Joy>(channel("active")));
}

class FirstPacketServer : public ServerIntegration
{
    Server::Config serverConfig() const override
    {
        auto config = ServerIntegration::serverConfig();
        config.managerOptions.statusIntervalMs = 200;
        return config;
    }
};

TEST_F(FirstPacketServer, OneServicePacketArmsLossEdgeBeforeStateTick)
{
    auto stateAtReceive = std::make_shared<std::atomic<int>>(-1);
    auto switchResult = std::make_shared<std::atomic<int>>(-1);
    Server::TypeConfig<Joy> config;
    config.outputCb = [this, stateAtReceive, switchResult](const Joy&, const transport::ControlSignalInfo& info)
    {
        const auto state = server_->csm().getSinkState(info.controller_name);
        stateAtReceive->store(state ? static_cast<int>(*state) : -1);
        switchResult->store(server_->switchController(info.controller_name));
    };
    config.emergencyStopCb = [this](const transport::ControlSignalInfo&)
    {
        ++stops_;
    };
    server_->registerTypeConfig(std::move(config));
    // Synchronize with an observed status tick, leaving the next 200ms window
    // for registration and the single packet's 50ms safety deadline.
    auto ticks = std::make_shared<std::atomic<int>>(0);
    auto status = sourceNode_->create_subscription<r1_interfaces::msg::ManagerStatus>(
        prefix_ + "/status",
        10,
        [ticks](r1_interfaces::msg::ManagerStatus::ConstSharedPtr)
        {
            ++*ticks;
        });
    ASSERT_TRUE(waitFor(
        [&]
        {
            return ticks->load() > 0;
        }));
    auto source = add("single", 50, 50, 0, "joy", "service");
    ASSERT_EQ(source.send(joy()), transport::SendResult::OK);
    EXPECT_EQ(stateAtReceive->load(), static_cast<int>(transport::ControlSignalState::INITIAL));
    EXPECT_EQ(switchResult->load(), SwitchResponse::RESPONSE_NOT_ACTIVE);
    ASSERT_TRUE(waitFor(
        [&]
        {
            return stops_ == 1;
        },
        150ms));
    std::this_thread::sleep_for(200ms);
    EXPECT_EQ(stops_, 1);
}

class SlowWatchdogServer : public ServerIntegration
{
    Server::Config serverConfig() const override
    {
        auto config = ServerIntegration::serverConfig();
        config.watchdogIntervalNs = 5'000'000'000;
        return config;
    }
};

TEST_F(SlowWatchdogServer, SameChannelRebuildResetsDedupBeforeWatchdog)
{
    const auto start = std::chrono::steady_clock::now();
    auto converter = std::make_shared<rv2_interfaces::rv2_server_control::OutputMessageConverter>();
    auto converted = std::make_shared<std::atomic<int>>(0);
    auto changes = std::make_shared<std::atomic<int>>(0);
    Server::TypeConfig<Joy> config;
    config.outputCb = [converter, converted](const Joy& message, const transport::ControlSignalInfo& info)
    {
        if (converter->convert(info.channel_name, message))
            ++*converted;
    };
    config.emergencyStopCb = [this, converter](const transport::ControlSignalInfo&)
    {
        ++stops_;
        converter->reset();
    };
    config.activeChangedCb = [converter, changes](const std::string&)
    {
        converter->reset();
        ++*changes;
    };
    server_->registerTypeConfig(std::move(config));
    auto source = add("same", 50, 150, 500);
    ASSERT_TRUE(sendUntil(source,
                          joy(),
                          [&]
                          {
                              return converted->load() == 1;
                          }));
    const auto oldSink = server_->csm().getSink(source.controllerName());
    ASSERT_TRUE(waitFor(
        [&]
        {
            return !oldSink.valid();
        },
        2000ms));
    ASSERT_EQ(stops_, 0);  // the 5s watchdog has not cleared the old record
    ASSERT_EQ(changes->load(), 1);
    if (source.valid())
    {
        EXPECT_TRUE(sources_->unregisterSource(source));
    }
    ASSERT_TRUE(waitFor(
        [&]
        {
            return !sources_->getSource(source.controllerName()).valid();
        }));
    auto replacement = add("same", 50, 150, 500);
    ASSERT_TRUE(sendUntil(replacement,
                          joy(),
                          [&]
                          {
                              return converted->load() == 2;
                          }));
    EXPECT_LT(std::chrono::steady_clock::now() - start, 4s);
    EXPECT_EQ(changes->load(), 2);
    EXPECT_EQ(stops_, 1);
}

TEST_F(ServerIntegration, EmergencyMessageStopsAndIsNotForwarded)
{
    auto source = add("estop", 50, 1000, 0, "joy", "service");
    ASSERT_EQ(source.send(joy()), transport::SendResult::OK);
    ASSERT_TRUE(waitFor(
        [&]
        {
            return count("estop") == 1;
        }));
    auto stop = joy();
    stop.axes[5] = -1;
    ASSERT_EQ(source.send(stop), transport::SendResult::OK);
    ASSERT_TRUE(waitFor(
        [&]
        {
            return stops_ == 1;
        }));
    EXPECT_EQ(count("estop"), 1u);
    EXPECT_TRUE(server_->getActiveSink<Joy>().empty());
    ASSERT_EQ(source.send(joy()), transport::SendResult::OK);
    ASSERT_TRUE(waitFor(
        [&]
        {
            return count("estop") == 2;
        }));
}

TEST_F(ServerIntegration, TerminalRemovalThenSameNameRegistrationRecovers)
{
    auto source = add("restart", 50, 150, 500);
    ASSERT_TRUE(sendUntil(source,
                          joy(),
                          [&]
                          {
                              return count("restart") > 0;
                          }));
    const auto oldSink = server_->csm().getSink(source.controllerName());
    ASSERT_TRUE(waitFor(
        [&]
        {
            return !oldSink.valid();
        }));
    EXPECT_EQ(stops_, 1);
    // Explicitly retire the old source intent before registering the same name.
    if (source.valid())
    {
        EXPECT_TRUE(sources_->unregisterSource(source));
    }
    ASSERT_TRUE(waitFor(
        [&]
        {
            return !sources_->getSource(source.controllerName()).valid();
        }));
    auto replacement = add("restart", 50, 150, 500);
    const auto before = count("restart");
    ASSERT_TRUE(sendUntil(replacement,
                          joy(),
                          [&]
                          {
                              return count("restart") > before;
                          }));
    EXPECT_EQ(server_->getActiveSink<Joy>(), channel("restart"));
    EXPECT_FALSE(oldSink.valid());
}

TEST_F(ServerIntegration, ReplacingTypeConfigDoesNotDuplicateWatchdog)
{
    setJoyConfig();
    setJoyConfig();
    auto source = add("replace", 50, 150);
    ASSERT_TRUE(sendUntil(source,
                          joy(),
                          [&]
                          {
                              return count("replace") > 0;
                          }));
    ASSERT_TRUE(waitFor(
        [&]
        {
            return stops_ == 1;
        }));
    std::this_thread::sleep_for(250ms);
    EXPECT_EQ(stops_, 1);
}

TEST_F(ServerIntegration, DisabledTimeoutKeepsSelectionUntilExplicitUnregister)
{
    auto source = add("persistent", 50, 0);
    ASSERT_TRUE(sendUntil(source,
                          joy(),
                          [&]
                          {
                              return count("persistent") > 0;
                          }));
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(stops_, 0);
    ASSERT_TRUE(sources_->unregisterSource(source));
    ASSERT_TRUE(waitFor(
        [&]
        {
            return stops_ == 1;
        }));
    EXPECT_TRUE(server_->getActiveSink<Joy>().empty());
}

TEST_F(ServerIntegration, TwistServiceSentinelAndManualSelection)
{
    auto source = add("twist", 50, 600, 0, "twist", "service");
    Twist message;
    message.linear.x = 0.4;
    ASSERT_EQ(source.send(message), transport::SendResult::OK);
    ASSERT_TRUE(waitFor(
        [&]
        {
            return twistOutputs_ == 1;
        }));
    EXPECT_TRUE(server_->setActiveSink<Twist>(channel("twist")));
    EXPECT_FALSE(server_->setActiveSink<Joy>(channel("twist")));
    message.linear.z = message.angular.x = message.angular.y = -99;
    ASSERT_EQ(source.send(message), transport::SendResult::OK);
    ASSERT_TRUE(waitFor(
        [&]
        {
            return twistStops_ == 1;
        }));
    EXPECT_EQ(twistOutputs_, 1);
}

TEST_F(ServerIntegration, CrossTypeTakeoverAndFallbackUseOnePriorityPolicy)
{
    auto low = add("joy_low", 20, 2000, 0, "joy", "service");
    ASSERT_EQ(low.send(joy()), transport::SendResult::OK);
    auto high = add("twist_high", 80, 300, 0, "twist", "service");
    Twist normal;
    normal.linear.x = 0.4;
    ASSERT_EQ(high.send(normal), transport::SendResult::OK);
    EXPECT_EQ(twistOutputs_, 0);
    auto request = normal;
    request.linear.z = request.angular.x = request.angular.y = 99;
    ASSERT_EQ(high.send(request), transport::SendResult::OK);
    EXPECT_TRUE(server_->getActiveSink<Joy>().empty());
    EXPECT_EQ(server_->getActiveSink<Twist>(), channel("twist_high"));
    EXPECT_EQ(stops_, 1);
    EXPECT_EQ(twistOutputs_, 0);
    ASSERT_EQ(high.send(normal), transport::SendResult::OK);
    EXPECT_EQ(twistOutputs_, 1);
    const auto before = count("joy_low");
    auto lowRequest = joy();
    lowRequest.buttons[11] = 1;
    ASSERT_EQ(low.send(lowRequest), transport::SendResult::OK);
    EXPECT_EQ(server_->getActiveSink<Twist>(), channel("twist_high"));
    ASSERT_TRUE(waitFor(
        [&]
        {
            return server_->getActiveSink<Joy>() == channel("joy_low");
        }));
    EXPECT_TRUE(server_->getActiveSink<Twist>().empty());
    EXPECT_EQ(twistStops_, 1);
    EXPECT_EQ(count("joy_low"), before);  // fallback does not replay
    ASSERT_EQ(low.send(joy()), transport::SendResult::OK);
    EXPECT_EQ(count("joy_low"), before + 1);
}

TEST_F(ServerIntegration, EqualPriorityFallbackUsesChannelOrderAcrossTypes)
{
    auto active = add("outgoing", 90, 300, 0, "joy", "service");
    auto alpha = add("alpha", 50, 3000, 0, "twist", "service");
    auto beta = add("beta", 50, 3000, 0, "joy", "service");
    ASSERT_EQ(active.send(joy()), transport::SendResult::OK);
    Twist message;
    message.linear.x = 0.4;
    ASSERT_EQ(alpha.send(message), transport::SendResult::OK);
    ASSERT_EQ(beta.send(joy()), transport::SendResult::OK);
    EXPECT_EQ(server_->getActiveSink<Joy>(), channel("outgoing"));
    ASSERT_TRUE(waitFor(
        [&]
        {
            return server_->getActiveSink<Twist>() == channel("alpha");
        }));
    EXPECT_TRUE(server_->getActiveSink<Joy>().empty());
    EXPECT_EQ(stops_, 1);
    EXPECT_EQ(count("beta"), 0u);
    EXPECT_EQ(twistOutputs_, 0);  // channel ordering wins over type ordering
    ASSERT_EQ(alpha.send(message), transport::SendResult::OK);
    EXPECT_EQ(twistOutputs_, 1);
}

TEST_F(ServerIntegration, StrictSwitchRejectsMissingInactiveAndUnsupportedWithoutSideEffects)
{
    auto active = add("active", 80, 3000, 0, "joy", "service");
    ASSERT_EQ(active.send(joy()), transport::SendResult::OK);
    auto initial = add("initial", 90, 3000);
    auto unsupported = add("text", 100, 3000, 0, "string");
    EXPECT_EQ(server_->switchController("absent"), SwitchResponse::RESPONSE_NOT_FOUND);
    EXPECT_EQ(server_->switchController(initial.controllerName()), SwitchResponse::RESPONSE_NOT_ACTIVE);
    EXPECT_EQ(server_->switchController(unsupported.controllerName()), SwitchResponse::RESPONSE_UNSUPPORTED_TYPE);
    EXPECT_EQ(server_->getStatus().active_controller_name, active.controllerName());
    EXPECT_EQ(stops_, 0);
    EXPECT_EQ(count("active"), 1u);
    const auto status = server_->getStatus();
    EXPECT_EQ(status.registered_controllers.size(), 3u);
    EXPECT_FALSE(status.has_output);  // output callbacks are not publish receipts
    ASSERT_TRUE(sources_->unregisterSource(initial));
    ASSERT_TRUE(waitFor(
        [&]
        {
            return !server_->csm().getSink(initial.controllerName()).valid();
        }));
    EXPECT_EQ(server_->switchController(initial.controllerName()), SwitchResponse::RESPONSE_NOT_FOUND);
}

TEST_F(FirstPacketServer, StrictSwitchRejectsLocallyStaleDataEvenBeforeTransportTimeoutTick)
{
    auto source = add("stale", 50, 80, 0, "joy", "service");
    ASSERT_TRUE(sendUntil(source,
                          joy(),
                          [&]
                          {
                              return server_->csm().getSinkState(source.controllerName()) ==
                                     transport::ControlSignalState::ACTIVE;
                          }));
    ASSERT_EQ(server_->switchController(source.controllerName()), SwitchResponse::RESPONSE_SUCCESS);
    std::this_thread::sleep_for(100ms);
    ASSERT_EQ(server_->csm().getSinkState(source.controllerName()), transport::ControlSignalState::ACTIVE);
    EXPECT_EQ(server_->switchController(source.controllerName()), SwitchResponse::RESPONSE_NOT_ACTIVE);
    ASSERT_TRUE(waitFor(
        [&]
        {
            return server_->csm().getSinkState(source.controllerName()) == transport::ControlSignalState::TIMEOUT;
        }));
    EXPECT_EQ(server_->switchController(source.controllerName()), SwitchResponse::RESPONSE_NOT_ACTIVE);
}

TEST_F(ServerIntegration, InterruptLatchesAllTypesAndResetRequiresNewData)
{
    auto source = add("joy", 20, 150, 0, "joy", "service");
    auto twist = add("twist", 100, 150, 0, "twist", "service");
    ASSERT_EQ(source.send(joy()), transport::SendResult::OK);
    const auto before = count("joy");
    ASSERT_TRUE(server_->interrupt());
    EXPECT_FALSE(server_->interrupt());
    EXPECT_EQ(interruptionStops_, 1);
    EXPECT_TRUE(server_->getActiveSink<Joy>().empty());
    EXPECT_TRUE(server_->getActiveSink<Twist>().empty());
    auto request = joy();
    request.buttons[11] = 1;
    auto emergency = joy();
    emergency.axes[5] = -1;
    Twist movement;
    movement.linear.x = 0.6;
    for (const auto& message : {joy(), request, emergency})
        ASSERT_EQ(source.send(message), transport::SendResult::OK);
    ASSERT_EQ(twist.send(movement), transport::SendResult::OK);
    auto newSource = add("new", 100, 150, 0, "joy", "service");
    ASSERT_EQ(newSource.send(joy()), transport::SendResult::OK);
    EXPECT_EQ(server_->switchController(twist.controllerName()), SwitchResponse::RESPONSE_INTERRUPTED);
    EXPECT_FALSE(server_->setActiveSink<Twist>(channel("twist")));
    std::this_thread::sleep_for(250ms);  // multiple watchdog ticks, no extra stop
    EXPECT_EQ(count("joy"), before);
    EXPECT_EQ(count("new"), 0u);
    EXPECT_EQ(twistOutputs_, 0);
    EXPECT_EQ(stops_, 0);
    EXPECT_EQ(twistStops_, 0);
    EXPECT_EQ(interruptionStops_, 1);
    const auto interrupted = server_->getStatus();
    EXPECT_TRUE(interrupted.interrupted);
    EXPECT_TRUE(interrupted.active_controller_name.empty());
    EXPECT_EQ(interrupted.registered_controllers.size(), 3u);
    EXPECT_TRUE(interrupted.has_output);
    EXPECT_EQ(interrupted.output_api_id, 1003);
    EXPECT_TRUE(interrupted.output_controller_name.empty());

    // Fresh traffic while interrupted still must not become reset-time fallback.
    ASSERT_EQ(twist.send(movement), transport::SendResult::OK);
    server_->resetInterrupt();
    server_->resetInterrupt();
    EXPECT_EQ(resets_, 2);  // once on interrupt, once on the effective reset
    EXPECT_FALSE(server_->getStatus().interrupted);
    EXPECT_TRUE(server_->getStatus().active_controller_name.empty());
    EXPECT_EQ(server_->switchController(twist.controllerName()), SwitchResponse::RESPONSE_NOT_ACTIVE);
    std::this_thread::sleep_for(100ms);
    EXPECT_TRUE(server_->getActiveSink<Twist>().empty());
    EXPECT_EQ(twistOutputs_, 0);
    EXPECT_EQ(interruptionStops_, 1);
    ASSERT_EQ(twist.send(movement), transport::SendResult::OK);
    EXPECT_EQ(twistOutputs_, 1);
    EXPECT_EQ(server_->getStatus().active_controller_name, twist.controllerName());
}

TEST_F(ServerIntegration, InterruptBeforeAnyOutputStillStopsAndSameValueRecoversAfterReset)
{
    auto converter = std::make_shared<rv2_interfaces::rv2_server_control::OutputMessageConverter>();
    resetHook_ = [converter]
    {
        converter->reset();
    };
    auto published = std::make_shared<std::atomic<int>>(0);
    Server::TypeConfig<Joy> config;
    config.outputCb = [this, converter, published](const Joy& message, const transport::ControlSignalInfo& info)
    {
        if (converter->convert(info.channel_name, message))
        {
            ++*published;
            builtin_interfaces::msg::Time stamp;
            stamp.sec = 12;
            stamp.nanosec = 34;
            server_->recordOutput(stamp, info.controller_name, 1008, "payload", {4, 5});
        }
    };
    server_->registerTypeConfig(std::move(config));
    EXPECT_EQ(server_->getStatus().server_name, prefix_);
    EXPECT_FALSE(server_->getStatus().has_output);
    ASSERT_TRUE(server_->interrupt());
    EXPECT_EQ(interruptionStops_, 1);
    server_->resetInterrupt();
    auto source = add("same", 50, 3000, 0, "joy", "service");
    ASSERT_EQ(source.send(joy()), transport::SendResult::OK);
    ASSERT_EQ(published->load(), 1);
    const auto status = server_->getStatus();
    EXPECT_EQ(status.output_stamp.sec, 12);
    EXPECT_EQ(status.output_stamp.nanosec, 34u);
    EXPECT_EQ(status.output_controller_name, source.controllerName());
    EXPECT_EQ(status.output_api_id, 1008);
    EXPECT_EQ(status.output_parameter, "payload");
    EXPECT_EQ(status.output_binary, (std::vector<uint8_t>{4, 5}));
    ASSERT_EQ(source.send(joy()), transport::SendResult::OK);
    EXPECT_EQ(published->load(), 1);
    EXPECT_EQ(server_->getStatus().output_stamp, status.output_stamp);
    ASSERT_TRUE(server_->interrupt());
    ASSERT_EQ(source.send(joy()), transport::SendResult::OK);
    EXPECT_EQ(published->load(), 1);
    server_->resetInterrupt();
    EXPECT_EQ(published->load(), 1);
    ASSERT_EQ(source.send(joy()), transport::SendResult::OK);
    EXPECT_EQ(published->load(), 2);
}

TEST_F(ServerIntegration, InterruptFencesInFlightOutputAndAllowsRecursiveStatusQueries)
{
    struct Gate
    {
        std::atomic<bool> entered{false}, release{false}, started{false}, completed{false};
        std::atomic<int> outputs{0};
    };
    struct ReleaseGuard
    {
        std::shared_ptr<Gate> gate = std::make_shared<Gate>();
        std::thread interrupt;
        ~ReleaseGuard()
        {
            gate->release = true;
            if (interrupt.joinable())
                interrupt.join();
        }
    } guard;
    const auto gate = guard.gate;
    Server::TypeConfig<Joy> config;
    config.outputCb = [this, gate](const Joy&, const transport::ControlSignalInfo& info)
    {
        gate->entered = true;
        while (!gate->release.load())
            std::this_thread::yield();
        server_->recordOutput(sinkNode_->now(), info.controller_name, 1008, "in-flight", {});
        EXPECT_EQ(server_->getStatus().output_controller_name, info.controller_name);
        ++gate->outputs;
    };
    server_->registerTypeConfig(std::move(config));
    auto source = add("inflight", 50, 3000);
    ASSERT_TRUE(sendUntil(source,
                          joy(),
                          [&]
                          {
                              return gate->entered.load();
                          }));
    guard.interrupt = std::thread(
        [&, gate]
        {
            gate->started = true;
            EXPECT_TRUE(server_->interrupt());
            gate->completed = true;
        });
    ASSERT_TRUE(waitFor(
        [&]
        {
            return gate->started.load();
        }));
    std::this_thread::sleep_for(50ms);
    EXPECT_FALSE(gate->completed);
    gate->release = true;
    guard.interrupt.join();
    EXPECT_TRUE(gate->completed);
    const auto before = gate->outputs.load();
    const auto status = server_->getStatus();
    EXPECT_TRUE(status.interrupted);
    EXPECT_EQ(status.output_api_id, 1003);  // safety stop follows the in-flight Move
    EXPECT_TRUE(status.output_controller_name.empty());
    for (int i = 0; i < 5; ++i)
    {
        source.send(joy(0.8f));
        std::this_thread::sleep_for(20ms);
    }
    EXPECT_EQ(gate->outputs.load(), before);
    EXPECT_EQ(interruptionStops_, 1);
    EXPECT_EQ(server_->getStatus().output_api_id, 1003);
}

TEST_F(ServerIntegration, DestructorFencesAnInFlightUserCallback)
{
    struct Gate
    {
        std::atomic<bool> entered{false}, release{false}, destroyed{false};
    };
    struct ReleaseGuard
    {
        std::shared_ptr<Gate> gate = std::make_shared<Gate>();
        std::thread destroy;
        ~ReleaseGuard()
        {
            gate->release = true;
            if (destroy.joinable())
                destroy.join();
        }
    } guard;
    const auto gate = guard.gate;
    Server::TypeConfig<Joy> config;
    config.outputCb = [gate](const Joy&, const transport::ControlSignalInfo&)
    {
        gate->entered = true;
        while (!gate->release.load())
            std::this_thread::yield();
    };
    server_->registerTypeConfig(std::move(config));
    auto source = add("shutdown", 50, 1000);
    ASSERT_TRUE(sendUntil(source,
                          joy(),
                          [&]
                          {
                              return gate->entered.load();
                          }));
    guard.destroy = std::thread(
        [&, gate]
        {
            server_.reset();
            gate->destroyed = true;
        });
    std::this_thread::sleep_for(50ms);
    EXPECT_FALSE(gate->destroyed);
    gate->release = true;
    guard.destroy.join();
    EXPECT_TRUE(gate->destroyed);
}
}  // namespace
