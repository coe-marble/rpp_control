#include <rpp_control/ros/control_script_time.hpp>

#include <gtest/gtest.h>

#include <chrono>

namespace
{
namespace timing = rpp_control::time_helpers;
using namespace std::chrono_literals;

builtin_interfaces::msg::Time stamp(std::int32_t seconds, std::uint32_t nanoseconds = 0)
{
  builtin_interfaces::msg::Time result;
  result.sec = seconds;
  result.nanosec = nanoseconds;
  return result;
}

TEST(NavigationDeadline, timeout_starts_at_the_command_stamp)
{
  const rclcpp::Time received_at(10, 0, RCL_ROS_TIME);
  const auto deadline = timing::navigation_deadline(stamp(9, 750000000), received_at, 500ms);
  ASSERT_TRUE(deadline);
  EXPECT_EQ(*deadline, 10250000000ULL);
  EXPECT_FALSE(timing::deadline_expired(*deadline, 10249999999ULL));
  EXPECT_TRUE(timing::deadline_expired(*deadline, 10250000000ULL));
}

TEST(NavigationDeadline, reception_stamp_gives_a_full_timeout)
{
  const rclcpp::Time received_at(10, 0, RCL_ROS_TIME);
  const auto reception_stamp = static_cast<builtin_interfaces::msg::Time>(received_at);
  const auto deadline = timing::navigation_deadline(reception_stamp, received_at, 500ms);
  ASSERT_TRUE(deadline);
  EXPECT_EQ(*deadline, 10500000000ULL);
}

TEST(NavigationDeadline, expired_commands_are_rejected_at_the_deadline)
{
  const rclcpp::Time received_at(10, 0, RCL_ROS_TIME);
  EXPECT_FALSE(timing::navigation_deadline(stamp(9, 500000000), received_at, 500ms));
  EXPECT_FALSE(timing::navigation_deadline(stamp(9, 499999999), received_at, 500ms));
  EXPECT_TRUE(timing::navigation_deadline(stamp(9, 500000001), received_at, 500ms));
}

TEST(NavigationDeadline, future_and_malformed_stamps_are_rejected)
{
  const rclcpp::Time received_at(10, 0, RCL_ROS_TIME);
  EXPECT_FALSE(timing::navigation_deadline(stamp(10, 1), received_at, 500ms));
  EXPECT_FALSE(timing::navigation_deadline(stamp(-1), received_at, 500ms));
  EXPECT_FALSE(timing::navigation_deadline(stamp(9, 1000000000), received_at, 500ms));
}
}  // namespace
