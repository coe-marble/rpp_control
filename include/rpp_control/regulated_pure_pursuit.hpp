#pragma once

#include <rpp_control/angle.hpp>
#include <rpp_control/planar_path_layout.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace rpp_control
{
struct PurePursuitParameters
{
  double desired_linear_vel = 1.0;
  double lookahead_dist = 1.0;
  double min_lookahead_dist = 0.3;
  double max_lookahead_dist = 2.0;
  double lookahead_time = 1.5;
  bool use_velocity_scaled_lookahead_dist = true;
  bool use_fixed_curvature_lookahead = false;
  double curvature_lookahead_dist = 1.0;
  bool interpolate_curvature_after_goal = false;
  double regulated_linear_scaling_min_radius = 1.0;
  double regulated_linear_scaling_min_speed = 0.1;
  double approach_velocity_scaling_dist = 1.0;
  double min_approach_linear_velocity = 0.05;
  double max_linear_accel = 1.0;
  double max_linear_decel = 1.0;
  double max_angular_vel = 1.0;
  double max_angular_accel = 2.0;
  double max_angular_decel = 2.0;
  bool use_rotate_to_heading = true;
  double rotate_to_heading_min_angle = 0.7853981633974483;
  double rotate_to_heading_angular_vel = 1.0;
  double goal_position_distance = 0.05;
  double heading_deadband = 0.05;

  void validate() const
  {
    const std::pair<const char *, double> positive[] = {
      {"desired_linear_vel", desired_linear_vel}, {"lookahead_dist", lookahead_dist},
      {"min_lookahead_dist", min_lookahead_dist}, {"max_lookahead_dist", max_lookahead_dist},
      {"lookahead_time", lookahead_time},
      {"curvature_lookahead_dist", curvature_lookahead_dist},
      {"regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius},
      {"approach_velocity_scaling_dist", approach_velocity_scaling_dist},
      {"max_linear_accel", max_linear_accel}, {"max_linear_decel", max_linear_decel},
      {"max_angular_vel", max_angular_vel}, {"max_angular_accel", max_angular_accel},
      {"max_angular_decel", max_angular_decel},
      {"rotate_to_heading_min_angle", rotate_to_heading_min_angle},
      {"rotate_to_heading_angular_vel", rotate_to_heading_angular_vel},
      {"goal_position_distance", goal_position_distance}};
    for (const auto & [name, value] : positive) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
      }
    }
    if (min_lookahead_dist > max_lookahead_dist ||
      !std::isfinite(regulated_linear_scaling_min_speed) || regulated_linear_scaling_min_speed < 0.0 ||
      regulated_linear_scaling_min_speed > desired_linear_vel ||
      !std::isfinite(min_approach_linear_velocity) || min_approach_linear_velocity < 0.0 ||
      min_approach_linear_velocity > desired_linear_vel ||
      rotate_to_heading_min_angle > pi / 2.0 || rotate_to_heading_angular_vel > max_angular_vel ||
      !std::isfinite(heading_deadband) || heading_deadband < 0.0 ||
      heading_deadband >= rotate_to_heading_min_angle) {
      throw std::invalid_argument(
        "Require min_lookahead_dist <= max_lookahead_dist, minimum speeds in [0,desired_linear_vel], "
        "rotate_to_heading_angular_vel <= max_angular_vel, rotate_to_heading_min_angle <= pi/2, "
        "and heading_deadband in [0,rotate_to_heading_min_angle)");
    }
  }

private:
  static constexpr double pi = 3.14159265358979323846;
};

// Geometry and velocity regulation only. The caller validates input layouts,
// finite values and masks. This class retains no path, goal, or control history.
class RegulatedPurePursuit
{
public:
  struct Command
  {
    double linear = 0.0;
    double angular = 0.0;
    std::uint8_t status = rpp_interfaces::contract::CODE_OK;
  };

  explicit RegulatedPurePursuit(PurePursuitParameters parameters = {}) : parameters_(parameters)
  {parameters_.validate();}

  template<class State, class Reference>
  Command compute(const State & state, const Reference & reference, const PlanarPathLayout & layout,
    double previous_linear, double previous_angular, double dt) const
  {
    const double count_value = reference[layout.count_offset()];
    const double remaining = reference[layout.remaining_distance_offset()];
    if (count_value < 0.0 || count_value > static_cast<double>(layout.capacity) ||
      std::floor(count_value) != count_value || remaining < 0.0) {
      return {0.0, 0.0, rpp_interfaces::contract::CODE_INVALID_INPUT};
    }
    const auto count = static_cast<std::size_t>(count_value);
    if (count == 0) {return {};}

    const auto cosine = std::cos(state[2]);
    const auto sine = std::sin(state[2]);
    const auto point = [&](std::size_t index) {
      const long double dx = static_cast<long double>(reference[layout.positions_offset() + 2 * index]) - state[0];
      const long double dy = static_cast<long double>(reference[layout.positions_offset() + 2 * index + 1]) - state[1];
      return Point{cosine * dx + sine * dy, -sine * dx + cosine * dy};
    };
    long double local_length = 0.0L;
    auto last = point(0);
    for (std::size_t index = 1; index < count; ++index) {
      const auto next = point(index);
      local_length += std::hypot(next.x - last.x, next.y - last.y);
      last = next;
    }
    const long double tolerance = 1e-6L * std::max(1.0L, local_length);
    if (static_cast<long double>(remaining) + tolerance < local_length) {
      return {0.0, 0.0, rpp_interfaces::contract::CODE_INVALID_INPUT};
    }
    const bool endpoint_visible = static_cast<long double>(remaining) <= local_length + tolerance;
    const auto endpoint_distance = std::hypot(last.x, last.y);
    const auto & params = parameters_;
    // A nearby endpoint on a loop is not the current target until the path is traversed.
    if (endpoint_visible && remaining <= params.goal_position_distance &&
      endpoint_distance <= params.goal_position_distance) {
      const auto error = wrap_angle(wrap_angle(reference[layout.orientations_offset() + count - 1]) - wrap_angle(state[2]));
      const double angular = params.use_rotate_to_heading ? rotation_speed(error) : 0.0;
      return limit_angular_velocity(linear_step(0.0, previous_linear, dt), angular, previous_angular, dt, true);
    }

    double lookahead = params.use_fixed_curvature_lookahead ? params.curvature_lookahead_dist : params.lookahead_dist;
    if (!params.use_fixed_curvature_lookahead && params.use_velocity_scaled_lookahead_dist) {
      const auto speed = std::hypot(static_cast<long double>(state[3]), static_cast<long double>(state[4]));
      lookahead = static_cast<double>(std::clamp(speed * params.lookahead_time,
        static_cast<long double>(params.min_lookahead_dist), static_cast<long double>(params.max_lookahead_dist)));
    }
    const auto carrot = lookahead_point(point, count, lookahead,
      params.use_fixed_curvature_lookahead && params.interpolate_curvature_after_goal && endpoint_visible);
    const auto angle = static_cast<double>(std::atan2(carrot.y, carrot.x));
    if (params.use_rotate_to_heading && std::abs(angle) > params.rotate_to_heading_min_angle) {
      return limit_angular_velocity(linear_step(0.0, previous_linear, dt), rotation_speed(angle),
        previous_angular, dt, true);
    }
    // Treat a perpendicular target consistently despite frame-rotation roundoff.
    if (std::abs(angle) > 1.5707963267948966 + 1e-9) {
      return {0.0, 0.0, rpp_interfaces::contract::CODE_INFEASIBLE};
    }
    const auto squared_distance = carrot.x * carrot.x + carrot.y * carrot.y;
    if (squared_distance < 1e-24L) {return {0.0, 0.0, rpp_interfaces::contract::CODE_INFEASIBLE};}
    const double curvature = static_cast<double>(2.0L * carrot.y / squared_distance);

    double linear = params.desired_linear_vel;
    const double absolute_curvature = std::abs(curvature);
    if (absolute_curvature > 0.0) {
      const double radius = 1.0 / absolute_curvature;
      if (radius < params.regulated_linear_scaling_min_radius) {
        linear = std::max(params.regulated_linear_scaling_min_speed,
          params.desired_linear_vel * (radius / params.regulated_linear_scaling_min_radius));
      }
      linear = std::min(linear, params.max_angular_vel / absolute_curvature);
    }
    if (endpoint_visible && remaining < params.approach_velocity_scaling_dist) {
      const auto scale = std::min(1.0L, endpoint_distance / params.approach_velocity_scaling_dist);
      linear = std::min(linear, std::max(params.min_approach_linear_velocity,
        static_cast<double>(linear * scale)));
      const auto braking_speed = std::sqrt(2.0L * params.max_linear_decel * endpoint_distance);
      linear = static_cast<double>(std::min(static_cast<long double>(linear), braking_speed));
    }
    // Ramp surge first so yaw follows the speed actually requested this tick.
    const double ramped_linear = linear_step(linear, previous_linear, dt);
    return limit_angular_velocity(ramped_linear, ramped_linear * curvature, previous_angular, dt,
      linear != params.desired_linear_vel || ramped_linear != linear);
  }

private:
  struct Point {long double x, y;};

  template<class PointAt>
  static Point lookahead_point(PointAt && point, std::size_t count, double distance, bool extend_endpoint)
  {
    const long double radius_squared = static_cast<long double>(distance) * distance;
    auto previous = point(0);
    if (previous.x * previous.x + previous.y * previous.y >= radius_squared) {return previous;}
    for (std::size_t index = 1; index < count; ++index) {
      const auto next = point(index);
      if (next.x * next.x + next.y * next.y >= radius_squared) {
        const auto dx = next.x - previous.x;
        const auto dy = next.y - previous.y;
        // Start is inside the circle; choose its forward exit along this segment.
        const auto fraction = std::clamp(circle_exit_fraction(previous, {dx, dy}, radius_squared), 0.0L, 1.0L);
        return {previous.x + fraction * dx, previous.y + fraction * dy};
      }
      previous = next;
    }
    // Extend only the global endpoint, using the last nonzero segment. A clipped
    // local section must never invent the direction of the remaining global path.
    if (extend_endpoint) {
      for (std::size_t index = count - 1; index > 0; --index) {
        const auto end = point(index);
        const auto start = point(index - 1);
        const auto length = std::hypot(end.x - start.x, end.y - start.y);
        if (length == 0.0L) {continue;}
        const Point direction{(end.x - start.x) / length, (end.y - start.y) / length};
        const auto distance_ahead = std::max(0.0L, circle_exit_fraction(previous, direction, radius_squared));
        return {previous.x + distance_ahead * direction.x, previous.y + distance_ahead * direction.y};
      }
    }
    return previous;
  }

  static long double circle_exit_fraction(Point start, Point direction, long double radius_squared)
  {
    const auto a = direction.x * direction.x + direction.y * direction.y;
    const auto b = 2.0L * (start.x * direction.x + start.y * direction.y);
    const auto c = start.x * start.x + start.y * start.y - radius_squared;
    const auto root = std::sqrt(std::max(0.0L, b * b - 4.0L * a * c));
    return b >= 0.0L ? (-2.0L * c) / (b + root) : (-b + root) / (2.0L * a);
  }

  double rotation_speed(double angle) const
  {
    if (std::abs(angle) <= parameters_.heading_deadband) {return 0.0;}
    const auto stopping_speed = std::sqrt(2.0L * parameters_.max_angular_decel * std::abs(angle));
    return std::copysign(static_cast<double>(std::min(stopping_speed,
      static_cast<long double>(parameters_.rotate_to_heading_angular_vel))), angle);
  }

  double linear_step(double target, double previous, double dt) const
  {
    const auto limited = std::clamp(static_cast<long double>(target),
      static_cast<long double>(previous) - static_cast<long double>(parameters_.max_linear_decel) * dt,
      static_cast<long double>(previous) + static_cast<long double>(parameters_.max_linear_accel) * dt);
    // Hard speed bounds take precedence when handover feedback is already outside them.
    return static_cast<double>(std::clamp(limited, 0.0L, static_cast<long double>(parameters_.desired_linear_vel)));
  }

  Command limit_angular_velocity(double linear, double angular, double previous_angular,
    double dt, bool limited) const
  {
    const auto capped_angular = std::clamp(angular, -parameters_.max_angular_vel, parameters_.max_angular_vel);
    long double angular_step;
    if (previous_angular != 0.0 && capped_angular != 0.0 &&
      std::signbit(previous_angular) != std::signbit(capped_angular)) {
      // A sign change first brakes to zero, then uses the remaining time to accelerate.
      const auto stop_time = std::abs(static_cast<long double>(previous_angular)) / parameters_.max_angular_decel;
      if (dt <= stop_time) {
        const auto remaining_speed = std::abs(static_cast<long double>(previous_angular)) -
          parameters_.max_angular_decel * dt;
        angular_step = std::copysign(remaining_speed, previous_angular);
      } else {
        const auto opposite_speed = std::min(std::abs(static_cast<long double>(capped_angular)),
          parameters_.max_angular_accel * (dt - stop_time));
        angular_step = std::copysign(opposite_speed, capped_angular);
      }
    } else {
      const auto rate = std::abs(capped_angular) > std::abs(previous_angular) ?
        parameters_.max_angular_accel : parameters_.max_angular_decel;
      const auto delta = static_cast<long double>(rate) * dt;
      angular_step = std::clamp(static_cast<long double>(capped_angular), previous_angular - delta, previous_angular + delta);
    }
    const auto ramped_angular = static_cast<double>(std::clamp(angular_step,
      -static_cast<long double>(parameters_.max_angular_vel), static_cast<long double>(parameters_.max_angular_vel)));
    limited = limited || capped_angular != angular || ramped_angular != angular;
    return {linear, ramped_angular,
      limited ? rpp_interfaces::contract::CODE_LIMITED : rpp_interfaces::contract::CODE_OK};
  }

  PurePursuitParameters parameters_;
};
}  // namespace rpp_control
