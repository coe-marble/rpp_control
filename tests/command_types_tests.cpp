#include <type_traits>
#include <gtest/gtest.h>
#include <rpp_control/ros/command_types.hpp>

namespace
{

template<typename Types>
class CommandTypesTest : public ::testing::Test {};
using Modes = ::testing::Types<
  rpp_control::CommandTopicTypes<true>, rpp_control::CommandTopicTypes<false>>;
TYPED_TEST_SUITE(CommandTypesTest, Modes);

TYPED_TEST(CommandTypesTest, pose_payload_preserves_coordinates_without_copying)
{
    typename TypeParam::Pose message;
    if constexpr (std::is_same_v<typename TypeParam::Pose, geometry_msgs::msg::PoseStamped>) {
    message.header.frame_id = "jetski/map";
    message.pose.position.x = 12.5;
    message.pose.orientation.w = 1.0;
    EXPECT_EQ(&TypeParam::pose(message), &message.pose);
    } else {
    message.position.x = 12.5;
    message.orientation.w = 1.0;
    EXPECT_EQ(&TypeParam::pose(message), &message);
    }
    EXPECT_DOUBLE_EQ(TypeParam::pose(message).position.x, 12.5);
    EXPECT_DOUBLE_EQ(TypeParam::pose(message).orientation.w, 1.0);
}

TYPED_TEST(CommandTypesTest, twist_preserves_linear_and_angular_commands)
{
    typename TypeParam::Twist message;
    if constexpr (std::is_same_v<typename TypeParam::Twist, geometry_msgs::msg::TwistStamped>) {
    message.header.frame_id = "jetski/base_link";
    message.twist.linear.x = 3.0;
    message.twist.angular.z = -0.2;
    EXPECT_EQ(&TypeParam::twist(message), &message.twist);
    } else {
    message.linear.x = 3.0;
    message.angular.z = -0.2;
    EXPECT_EQ(&TypeParam::twist(message), &message);
    }
    EXPECT_DOUBLE_EQ(TypeParam::twist(message).linear.x, 3.0);
    EXPECT_DOUBLE_EQ(TypeParam::twist(message).angular.z, -0.2);
}

TYPED_TEST(CommandTypesTest, wrench_preserves_si_force_and_torque)
{
    typename TypeParam::Wrench message;
    if constexpr (std::is_same_v<typename TypeParam::Wrench, geometry_msgs::msg::WrenchStamped>) {
    message.wrench.force.x = 125.0;
    message.wrench.torque.z = -50.0;
    EXPECT_EQ(&TypeParam::wrench(message), &message.wrench);
    } else {
    message.force.x = 125.0;
    message.torque.z = -50.0;
    EXPECT_EQ(&TypeParam::wrench(message), &message);
    }
    EXPECT_DOUBLE_EQ(TypeParam::wrench(message).force.x, 125.0);
    EXPECT_DOUBLE_EQ(TypeParam::wrench(message).torque.z, -50.0);
}

TYPED_TEST(CommandTypesTest, exposes_timestamps_only_for_stamped_commands)
{
    typename TypeParam::Pose pose;
    typename TypeParam::Twist twist;
    typename TypeParam::Wrench wrench;
    if constexpr (std::is_same_v<typename TypeParam::Pose, geometry_msgs::msg::PoseStamped>) {
    EXPECT_EQ(TypeParam::stamp(pose), &pose.header.stamp);
    EXPECT_EQ(TypeParam::stamp(twist), &twist.header.stamp);
    EXPECT_EQ(TypeParam::stamp(wrench), &wrench.header.stamp);
    } else {
    EXPECT_EQ(TypeParam::stamp(pose), nullptr);
    EXPECT_EQ(TypeParam::stamp(twist), nullptr);
    EXPECT_EQ(TypeParam::stamp(wrench), nullptr);
    }
}

TEST(CommandTopicTypes, public_aliases_follow_the_shared_constant)
{
    EXPECT_TRUE((std::is_same_v<rpp_control::CmdPose,
    std::conditional_t<rpp_control::kUseStampedCommandTopics,
    geometry_msgs::msg::PoseStamped, geometry_msgs::msg::Pose>>));
    EXPECT_TRUE((std::is_same_v<rpp_control::CmdTwist,
    std::conditional_t<rpp_control::kUseStampedCommandTopics,
    geometry_msgs::msg::TwistStamped, geometry_msgs::msg::Twist>>));
    EXPECT_TRUE((std::is_same_v<rpp_control::CmdWrench,
    std::conditional_t<rpp_control::kUseStampedCommandTopics,
    geometry_msgs::msg::WrenchStamped, geometry_msgs::msg::Wrench>>));
}

}  // namespace
