#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <rpp_ros_interfaces/msg/numeric_sample.hpp>
#include <rclcpp/time.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

// ROS timestamps drive freshness and numeric dt; the steady clock schedules thread ticks.
namespace rpp_control::time_helpers
{
using Clock = std::chrono::steady_clock;
using Duration = std::chrono::nanoseconds;
inline constexpr std::uint64_t nanoseconds_per_second = 1000000000;

inline double seconds(Duration duration)
{
  return std::chrono::duration<double>(duration).count();
}

inline std::uint64_t nanoseconds(Duration duration)
{
  // Callers validate that the interval is positive before converting to unsigned time.
  return static_cast<std::uint64_t>(duration.count());
}

inline double frequency_hz(Duration interval)
{
  return 1.0 / seconds(interval);
}

inline Duration duration_from_seconds(const std::string & name, double value)
{
  if (!std::isfinite(value) || value < 1e-9 || value >= seconds(Duration::max())) {
    throw std::invalid_argument(name + " must be positive and representable in nanoseconds");
  }
  return std::chrono::duration_cast<Duration>(std::chrono::duration<double>(value));
}

inline Duration tick_interval_from_frequency(double frequency)
{
  if (!std::isfinite(frequency) || frequency <= 0.0) {
    throw std::invalid_argument("control_frequency_hz must be positive and finite");
  }
  return duration_from_seconds("Tick interval derived from control_frequency_hz", 1.0 / frequency);
}

inline void validate_control_periods(Duration tick_interval, Duration maximum_interval)
{
  if (maximum_interval < tick_interval) {
    throw std::invalid_argument("max_control_dt_sec must cover at least one control tick");
  }
}

inline bool valid_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0 && stamp.nanosec < nanoseconds_per_second;
}

inline std::uint64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  // Incoming messages are validated before this conversion.
  return static_cast<std::uint64_t>(stamp.sec) * nanoseconds_per_second + stamp.nanosec;
}

inline std::uint64_t stamp_ns(const rpp_ros_interfaces::msg::NumericSample & sample)
{
  return stamp_ns(sample.stamp);
}

inline std::optional<builtin_interfaces::msg::Time> stamp_from_ns(std::uint64_t time_ns)
{
  const auto seconds = time_ns / nanoseconds_per_second;
  if (seconds > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {
    return std::nullopt;
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(seconds);
  stamp.nanosec = static_cast<std::uint32_t>(time_ns % nanoseconds_per_second);
  return stamp;
}

inline std::optional<std::uint64_t> timestamp_ns(const rclcpp::Time & time)
{
  const auto value = time.nanoseconds();
  if (value < 0) {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(value);
}

inline bool valid_sample_stamp(
  const rpp_ros_interfaces::msg::NumericSample & sample, std::uint64_t now_ns)
{
  return valid_stamp(sample.stamp) && stamp_ns(sample) <= now_ns;
}

inline bool deadline_expired(std::uint64_t deadline_ns, std::uint64_t now_ns)
{
  return deadline_ns <= now_ns;
}

inline bool measurement_fresh(
  std::uint64_t measured_time,
  std::uint64_t ros_now_ns, Duration timeout)
{
  // Compare before unsigned subtraction so future stamps are never fresh.
  return measured_time <= ros_now_ns &&
         ros_now_ns - measured_time <= static_cast<std::uint64_t>(timeout.count());
}

inline std::optional<std::uint64_t> navigation_deadline(
  const builtin_interfaces::msg::Time & stamp, const rclcpp::Time & now,
  Duration timeout)
{
  const auto current_time = timestamp_ns(now);
  if (!current_time || !valid_stamp(stamp)) {return std::nullopt;}
  const auto submitted = stamp_ns(stamp);
  if (!measurement_fresh(submitted, *current_time, timeout)) {return std::nullopt;}
  const auto interval = nanoseconds(timeout);
  if (submitted > std::numeric_limits<std::uint64_t>::max() - interval) {return std::nullopt;}
  const auto deadline = submitted + interval;
  return deadline > *current_time ? std::optional<std::uint64_t>(deadline) : std::nullopt;
}

inline bool valid_control_interval(Duration interval, Duration maximum)
{
  return interval > Duration::zero() && interval <= maximum;
}

inline bool output_expired(
  std::optional<std::uint64_t> generated_at, std::optional<std::uint64_t> now_ns,
  Duration maximum_age)
{
  return !generated_at || !now_ns || *generated_at > *now_ns ||
         *now_ns - *generated_at > static_cast<std::uint64_t>(maximum_age.count());
}

inline Clock::time_point next_tick_deadline(
  Clock::time_point previous_deadline, Clock::time_point now, Duration interval)
{
  const auto next = previous_deadline + interval;
  return next <= now ? now + interval : next;
}

struct TickTiming
{
  std::optional<std::uint64_t> time_ns;
  Duration interval;
  bool reset_inputs;
};

class TickClock
{
public:
  TickClock(Duration default_interval, std::uint64_t route_revision)
  : default_interval_(default_interval),
    previous_route_revision_(route_revision)
  {}

  TickTiming update(const rclcpp::Time & ros_time, std::uint64_t route_revision)
  {
    const auto current_time = timestamp_ns(ros_time);
    const bool clock_moved_backwards =
      current_time && previous_ros_time_ && *current_time < *previous_ros_time_;

    // Scheduling uses the steady clock. Numeric dt follows ROS time, including pauses and jumps.
    auto interval = default_interval_;
    if (!current_time || clock_moved_backwards) {
      interval = Duration::zero();
    } else if (previous_ros_time_ && previous_route_revision_ == route_revision) {
      const auto elapsed_ns = *current_time - *previous_ros_time_;
      interval = Duration(static_cast<Duration::rep>(elapsed_ns));
    }
    previous_ros_time_ = current_time;
    previous_route_revision_ = route_revision;
    return {current_time, interval, clock_moved_backwards || !current_time};
  }

private:
  Duration default_interval_;
  std::optional<std::uint64_t> previous_ros_time_;
  std::uint64_t previous_route_revision_;
};
} // namespace rpp_control::time_helpers
