#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace rpp_control
{

struct CommandAgePolicy
{
  bool discard_old;
  double max_age_seconds;
  rclcpp::Duration max_age;

  static rclcpp::Duration checked_duration(double seconds)
  {
    constexpr double max_seconds =
      static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9;
    if (!std::isfinite(seconds) || seconds <= 0.0 || seconds >= max_seconds) {
      throw std::invalid_argument(
                "max_stamped_message_age_sec must be finite, strictly positive, "
                "and representable as a ROS duration.");
    }
    const auto duration = rclcpp::Duration::from_seconds(seconds);
    if (duration.nanoseconds() < 1) {
      throw std::invalid_argument(
                "max_stamped_message_age_sec must be at least one nanosecond.");
    }
    return duration;
  }

  CommandAgePolicy(bool enabled, double seconds)
  : discard_old(enabled), max_age_seconds(seconds), max_age(checked_duration(seconds))
  {}

  bool accepts(const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & now) const
  {
    if (!discard_old) {return true;}
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {return false;}
    const rclcpp::Time sent(stamp, now.get_clock_type());
    const auto age = now - sent;
    return age.nanoseconds() >= 0 && age < max_age;
  }
};

// Parameter callbacks publish an immutable policy snapshot only after the
// complete transaction commits. Subscription callbacks read that snapshot;
// failed transactions cannot partially change filtering behavior.
class CommandAgeFilter {
public:
  explicit CommandAgeFilter(rclcpp::Node & node)
  : node_(node)
  {
    rcl_interfaces::msg::ParameterDescriptor enabled_descriptor;
    enabled_descriptor.description =
      "Discard old, future-dated or invalid stamped cmd_pos/cmd_vel/cmd_tau messages.";
    const bool enabled = node_.declare_parameter(
            "discard_old_stamped_messages", true, enabled_descriptor);

    rcl_interfaces::msg::ParameterDescriptor age_descriptor;
    age_descriptor.description =
      "Maximum stamped command age in seconds; the age must be strictly below this limit.";
    age_descriptor.additional_constraints =
      "Finite and strictly positive; at least one nanosecond; representable as a ROS duration.";
    rcl_interfaces::msg::FloatingPointRange age_range;
    age_range.from_value = 1e-9;
    age_range.to_value = std::nextafter(
            static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9, 0.0);
    age_range.step = 0.0;
    age_descriptor.floating_point_range.push_back(age_range);
    const double seconds = node_.declare_parameter(
            "max_stamped_message_age_sec", 0.5, age_descriptor);
    policy_.store(std::make_shared<const CommandAgePolicy>(enabled, seconds));

    validate_handle_ = node_.add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        if (!touches_policy(parameters)) {return result;}
        const auto current = policy_.load();
        bool enabled = current->discard_old;
        double seconds = current->max_age_seconds;
        try {
          for (const auto & parameter : parameters) {
            if (parameter.get_name() == "discard_old_stamped_messages") {
              enabled = parameter.as_bool();
            } else if (parameter.get_name() == "max_stamped_message_age_sec") {
              seconds = parameter.as_double();
            }
          }
          static_cast<void>(CommandAgePolicy(enabled, seconds));
        } catch (const std::exception & error) {
          result.successful = false;
          result.reason = error.what();
        }
        return result;
            });
    commit_handle_ = node_.add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        if (!touches_policy(parameters)) {return;}
        policy_.store(std::make_shared<const CommandAgePolicy>(
                    node_.get_parameter("discard_old_stamped_messages").as_bool(),
                    node_.get_parameter("max_stamped_message_age_sec").as_double()));
            });
  }

  bool accepts(const builtin_interfaces::msg::Time * stamp, const rclcpp::Time & now) const
  {
    if (stamp == nullptr) {return true;}
    return policy_.load()->accepts(*stamp, now);
  }

private:
  static bool touches_policy(const std::vector<rclcpp::Parameter> & parameters)
  {
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "discard_old_stamped_messages" ||
        parameter.get_name() == "max_stamped_message_age_sec") {return true;}
    }
    return false;
  }

  rclcpp::Node & node_;
  std::atomic<std::shared_ptr<const CommandAgePolicy>> policy_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr commit_handle_;
};

}  // namespace rpp_control
