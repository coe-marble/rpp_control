#include <cmath>
#include <limits>
#include <memory>
#include <gtest/gtest.h>
#include <rpp_control/ros/command_age_filter.hpp>
#include <rpp_control/ros/command_types.hpp>

namespace {

builtin_interfaces::msg::Time stamp(std::int32_t sec, std::uint32_t nanosec = 0)
{
    builtin_interfaces::msg::Time value;
    value.sec = sec;
    value.nanosec = nanosec;
    return value;
}

const rclcpp::Time now(10, 0, RCL_ROS_TIME);

TEST(CommandAgePolicy, accepts_fresh_but_rejects_exact_limit_and_older_messages)
{
    const rpp_control::CommandAgePolicy policy(true, 0.5);
    EXPECT_TRUE(policy.accepts(stamp(10), now));
    EXPECT_TRUE(policy.accepts(stamp(9, 500000001), now));
    EXPECT_FALSE(policy.accepts(stamp(9, 500000000), now));
    EXPECT_FALSE(policy.accepts(stamp(9, 499999999), now));
    EXPECT_FALSE(policy.accepts(stamp(1), now));
}

TEST(CommandAgePolicy, rejects_future_and_malformed_timestamps)
{
    const rpp_control::CommandAgePolicy policy(true, 0.5);
    EXPECT_FALSE(policy.accepts(stamp(10, 1), now));
    EXPECT_FALSE(policy.accepts(stamp(-1), now));
    EXPECT_FALSE(policy.accepts(stamp(9, 1000000000), now));
}

TEST(CommandAgePolicy, zero_stamp_and_ros_clock_jumps_follow_age_rules)
{
    const rpp_control::CommandAgePolicy policy(true, 0.5);
    EXPECT_TRUE(policy.accepts(stamp(0), rclcpp::Time(0, 0, RCL_ROS_TIME)));
    EXPECT_FALSE(policy.accepts(stamp(0), now));
    EXPECT_FALSE(policy.accepts(stamp(10), rclcpp::Time(9, 0, RCL_ROS_TIME)));
    EXPECT_FALSE(policy.accepts(stamp(10), rclcpp::Time(12, 0, RCL_ROS_TIME)));
}

TEST(CommandAgePolicy, disabled_filter_ignores_stamp_age_and_validity)
{
    const rpp_control::CommandAgePolicy policy(false, 0.5);
    EXPECT_TRUE(policy.accepts(stamp(0), now));
    EXPECT_TRUE(policy.accepts(stamp(20), now));
    EXPECT_TRUE(policy.accepts(stamp(-1, 1000000000), now));
}

TEST(CommandAgePolicy, validates_age_parameter_even_when_filter_is_disabled)
{
    for (double age : {0.0, -0.5, 1e-12, 1e20,
        std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_THROW(rpp_control::CommandAgePolicy(true, age), std::invalid_argument);
        EXPECT_THROW(rpp_control::CommandAgePolicy(false, age), std::invalid_argument);
    }
    EXPECT_NO_THROW(rpp_control::CommandAgePolicy(true, 1e-9));
}

class CommandAgeParameters : public ::testing::Test {
protected:
    void SetUp() override
    {
        context = std::make_shared<rclcpp::Context>();
        context->init(0, nullptr);
        rclcpp::NodeOptions options;
        options.context(context).enable_rosout(false)
            .start_parameter_services(false).start_parameter_event_publisher(false);
        node = std::make_shared<rclcpp::Node>("command_age_filter_test", options);
        filter = std::make_unique<rpp_control::CommandAgeFilter>(*node);
    }
    void TearDown() override
    {
        filter.reset();
        node.reset();
        context->shutdown("test finished");
    }
    std::shared_ptr<rclcpp::Context> context;
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<rpp_control::CommandAgeFilter> filter;
};

TEST_F(CommandAgeParameters, declares_enabled_filter_and_half_second_limit)
{
    EXPECT_TRUE(node->get_parameter("discard_old_stamped_messages").as_bool());
    EXPECT_DOUBLE_EQ(node->get_parameter("max_stamped_message_age_sec").as_double(), 0.5);
    const auto old = stamp(9);
    EXPECT_FALSE(filter->accepts(&old, now));
    EXPECT_TRUE(filter->accepts(nullptr, now));
}

TEST_F(CommandAgeParameters, valid_runtime_changes_apply_immediately)
{
    const auto old = stamp(9, 250000000);
    EXPECT_FALSE(filter->accepts(&old, now));
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("max_stamped_message_age_sec", 1.0)).successful);
    EXPECT_TRUE(filter->accepts(&old, now));
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("discard_old_stamped_messages", false)).successful);
    const auto very_old = stamp(0);
    EXPECT_TRUE(filter->accepts(&very_old, now));
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("discard_old_stamped_messages", true)).successful);
    EXPECT_FALSE(filter->accepts(&very_old, now));
}

TEST_F(CommandAgeParameters, invalid_atomic_change_preserves_both_parameters_and_policy)
{
    const auto result = node->set_parameters_atomically({
        rclcpp::Parameter("discard_old_stamped_messages", false),
        rclcpp::Parameter("max_stamped_message_age_sec", -1.0)});
    EXPECT_FALSE(result.successful);
    EXPECT_FALSE(result.reason.empty());
    EXPECT_TRUE(node->get_parameter("discard_old_stamped_messages").as_bool());
    EXPECT_DOUBLE_EQ(node->get_parameter("max_stamped_message_age_sec").as_double(), 0.5);
    const auto old = stamp(9);
    EXPECT_FALSE(filter->accepts(&old, now));
}

TEST_F(CommandAgeParameters, another_callback_veto_cannot_partially_disable_filtering)
{
    auto options = node->get_node_options();
    rclcpp::Node other("command_age_veto_test", options);
    bool veto_changes = false;
    // Register this callback first so the filter validates before the veto:
    // ROS invokes on-set callbacks in reverse registration order.
    const auto veto = other.add_on_set_parameters_callback(
        [&veto_changes](const std::vector<rclcpp::Parameter>&) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = !veto_changes;
            result.reason = "Rejected by another validator";
            return result;
        });
    rpp_control::CommandAgeFilter other_filter(other);
    veto_changes = true;
    EXPECT_FALSE(other.set_parameter(rclcpp::Parameter("discard_old_stamped_messages", false)).successful);
    const auto old = stamp(9);
    EXPECT_FALSE(other_filter.accepts(&old, now));
}

TEST_F(CommandAgeParameters, startup_parameter_overrides_are_honored_and_validated)
{
    rclcpp::NodeOptions options;
    options.context(context).enable_rosout(false)
        .start_parameter_services(false).start_parameter_event_publisher(false)
        .parameter_overrides({rclcpp::Parameter("discard_old_stamped_messages", false),
            rclcpp::Parameter("max_stamped_message_age_sec", 0.25)});
    rclcpp::Node configured("command_age_startup_test", options);
    rpp_control::CommandAgeFilter configured_filter(configured);
    const auto old = stamp(0);
    EXPECT_TRUE(configured_filter.accepts(&old, now));
    EXPECT_DOUBLE_EQ(configured.get_parameter("max_stamped_message_age_sec").as_double(), 0.25);
    options.parameter_overrides({rclcpp::Parameter("max_stamped_message_age_sec", 0.0)});
    rclcpp::Node invalid("command_age_invalid_test", options);
    EXPECT_THROW(rpp_control::CommandAgeFilter invalid_filter(invalid),
        rclcpp::exceptions::InvalidParameterValueException);
}

TEST_F(CommandAgeParameters, every_stamped_command_type_uses_the_same_filter)
{
    using Types = rpp_control::CommandTopicTypes<true>;
    Types::Pose pose;
    Types::Twist twist;
    Types::Wrench wrench;
    pose.header.stamp = twist.header.stamp = wrench.header.stamp = stamp(9);
    EXPECT_FALSE(filter->accepts(Types::stamp(pose), now));
    EXPECT_FALSE(filter->accepts(Types::stamp(twist), now));
    EXPECT_FALSE(filter->accepts(Types::stamp(wrench), now));
    pose.header.stamp = twist.header.stamp = wrench.header.stamp = stamp(9, 900000000);
    EXPECT_TRUE(filter->accepts(Types::stamp(pose), now));
    EXPECT_TRUE(filter->accepts(Types::stamp(twist), now));
    EXPECT_TRUE(filter->accepts(Types::stamp(wrench), now));
    using Plain = rpp_control::CommandTopicTypes<false>;
    EXPECT_TRUE(filter->accepts(Plain::stamp(Plain::Pose{}), now));
    EXPECT_TRUE(filter->accepts(Plain::stamp(Plain::Twist{}), now));
    EXPECT_TRUE(filter->accepts(Plain::stamp(Plain::Wrench{}), now));
}

}  // namespace
