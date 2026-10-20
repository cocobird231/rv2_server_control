#include <gtest/gtest.h>

#include "rv2_server_control/control_signal_detect.h"
#include "rv2_server_control/output_message_convert.h"

namespace server = rv2_interfaces::rv2_server_control;
using Joy = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;

TEST(ControlPredicates, ShortJoyArraysAreSafe)
{
    for (size_t count = 0; count <= 11; ++count)
    {
        Joy joy;
        joy.buttons.assign(count, 1);
        EXPECT_FALSE(server::isRequestActive(joy));
    }
    Joy joy;
    joy.buttons.assign(12, 0);
    joy.buttons[11] = 1;
    EXPECT_TRUE(server::isRequestActive(joy));
    joy.axes.assign(5, -1);
    EXPECT_FALSE(server::isEmergencyStop(joy));
    joy.axes.push_back(-1);
    EXPECT_TRUE(server::isEmergencyStop(joy));
    joy.axes[5] = 1;
    EXPECT_FALSE(server::isEmergencyStop(joy));
}

TEST(ControlPredicates, TwistSentinelsRequireAllThreeFields)
{
    Twist message;
    message.linear.z = -99;
    EXPECT_FALSE(server::isEmergencyStop(message));
    message.angular.x = message.angular.y = -99;
    EXPECT_TRUE(server::isEmergencyStop(message));
    message.linear.z = message.angular.x = message.angular.y = 99;
    EXPECT_TRUE(server::isRequestActive(message));
    message.angular.y = 0;
    EXPECT_FALSE(server::isRequestActive(message));
}

TEST(OutputConversion, TwistDedupIsPerChannelAndPerInstance)
{
    server::OutputMessageConverter first, second;
    Twist message;
    EXPECT_FALSE(first.convert("one", message));
    message.linear.x = 0.5;
    EXPECT_TRUE(first.convert("one", message));
    EXPECT_FALSE(first.convert("one", message));
    EXPECT_TRUE(first.convert("two", message));
    EXPECT_TRUE(second.convert("one", message));
    first.reset();
    EXPECT_TRUE(first.convert("one", message));
    message.linear.x = 0;
    EXPECT_TRUE(first.convert("one", message));
    EXPECT_FALSE(first.convert("one", message));
}

TEST(OutputConversion, JoyDedupResetAndShortAxes)
{
    server::OutputMessageConverter converter;
    Joy message;
    EXPECT_FALSE(converter.convert("joy", message));
    message.axes = {0.2f};
    EXPECT_TRUE(converter.convert("joy", message));
    EXPECT_FALSE(converter.convert("joy", message));
    converter.reset();
    EXPECT_TRUE(converter.convert("joy", message));
    message.axes.clear();
    EXPECT_TRUE(converter.convert("joy", message));
    EXPECT_FALSE(converter.convert("joy", message));
}

TEST(OutputConversion, ButtonsTriggerOncePerPress)
{
    server::OutputMessageConverter converter;
    Joy message;
    message.buttons.assign(12, 0);
    EXPECT_FALSE(converter.convert("joy", message));
    for (int button = 0; button < 4; ++button)
    {
        message.header.stamp.sec += 1;
        message.buttons[button] = 1;
        EXPECT_TRUE(converter.convert("joy", message));
        EXPECT_FALSE(converter.convert("joy", message));
        message.header.stamp.sec += 1;
        message.buttons[button] = 0;
        EXPECT_FALSE(converter.convert("joy", message));
    }
}

TEST(OutputConversion, AdvancingJoyTimeDoesNotRepeatCommands)
{
    server::OutputMessageConverter converter;
    Joy message;
    message.axes = {0.25f};
    message.buttons.assign(12, 0);
    message.header.stamp.sec = 1;
    ASSERT_TRUE(converter.convert("joy", message));

    // A new timestamp with held axes is input activity, not a new Move.
    message.header.stamp.sec = 30;
    EXPECT_FALSE(converter.convert("joy", message));
    message.buttons[0] = 1;
    message.header.stamp.sec = 31;
    ASSERT_TRUE(converter.convert("joy", message));
    // Crossing the interpreter's long-press threshold must not repeat StandUp.
    message.header.stamp.sec = 60;
    EXPECT_FALSE(converter.convert("joy", message));
    message.header.stamp.sec = 90;
    EXPECT_FALSE(converter.convert("joy", message));
    message.buttons[0] = 0;
    message.header.stamp.sec = 91;
    EXPECT_FALSE(converter.convert("joy", message));

    message.buttons[1] = 1;
    message.header.stamp.sec = 92;
    ASSERT_TRUE(converter.convert("joy", message));
    message.buttons[1] = 0;
    message.header.stamp.nanosec = 10'000'000;
    EXPECT_FALSE(converter.convert("joy", message));
    // Expiring the single-click window also adds no robot command.
    message.header.stamp.sec = 120;
    EXPECT_FALSE(converter.convert("joy", message));
}
