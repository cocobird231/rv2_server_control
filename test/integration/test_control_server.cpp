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

using namespace std::chrono_literals;
namespace transport = rv2_interfaces::r1;
using Server = rv2_interfaces::rv2_server_control::ControlServer;
using Joy = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;

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
        Server::Config config;
        config.name = prefix_;
        config.watchdogIntervalNs = 20'000'000;
        config.managerOptions.statusIntervalMs = 20;
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
};

TEST_F(ServerIntegration, Priority100IsNormalAndTypesAreIndependent)
{
    auto source = add("joy", 100);
    ASSERT_TRUE(sendUntil(source,
                          joy(),
                          [&]
                          {
                              return count("joy") > 0;
                          }));
    EXPECT_EQ(server_->getActiveSink<Joy>(), channel("joy"));
    EXPECT_EQ(stops_, 0);
    auto twist = add("twist", 50, 600, 0, "twist");
    Twist message;
    message.linear.x = 0.3;
    ASSERT_TRUE(sendUntil(twist,
                          message,
                          [&]
                          {
                              return twistOutputs_ > 0;
                          }));
    EXPECT_EQ(server_->getActiveSink<Joy>(), channel("joy"));
    EXPECT_EQ(server_->getActiveSink<Twist>(), channel("twist"));
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

TEST_F(ServerIntegration, OneServicePacketArmsLossEdgeBeforeStateTick)
{
    auto source = add("single", 50, 100, 0, "joy", "service");
    ASSERT_EQ(source.send(joy()), transport::SendResult::OK);
    ASSERT_TRUE(waitFor(
        [&]
        {
            return count("single") == 1;
        }));
    ASSERT_TRUE(waitFor(
        [&]
        {
            return stops_ == 1;
        }));
    std::this_thread::sleep_for(200ms);
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

TEST_F(ServerIntegration, DestructorFencesAnInFlightUserCallback)
{
    std::atomic<bool> entered{false}, release{false}, destroyed{false};
    Server::TypeConfig<Joy> config;
    config.outputCb = [&](const Joy&, const transport::ControlSignalInfo&)
    {
        entered = true;
        while (!release.load())
            std::this_thread::yield();
    };
    server_->registerTypeConfig(std::move(config));
    auto source = add("shutdown", 50, 1000);
    ASSERT_TRUE(sendUntil(source,
                          joy(),
                          [&]
                          {
                              return entered.load();
                          }));
    std::thread destroy(
        [&]
        {
            server_.reset();
            destroyed = true;
        });
    std::this_thread::sleep_for(50ms);
    EXPECT_FALSE(destroyed);
    release = true;
    destroy.join();
    EXPECT_TRUE(destroyed);
}
}  // namespace
