#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_interfaces/MissionMonitor.hpp>
#include <rpp_schema/rpp_interfaces/MissionMonitorDesc.hpp>
#include <rpp_interfaces/field_layout.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

class PoseMissionMonitor final : public rpp_interfaces::MissionMonitor
{
public:
  using ParameterDescription = rpp::params::ParameterDescription;
  using Description = rpp_schema::rpp_interfaces::MissionMonitorDesc;
  using Result = std::tuple<bool, std::uint8_t>;
  using CheckResult = std::tuple<bool, std::uint8_t, std::string>;

  RPP_PARAMETERS(
    ParameterDescription::create<std::string>("resource"),
    ParameterDescription::create<std::string>("frame_id", "map"),
    ParameterDescription::create<std::string>("child_frame_id", "base_link"),
    ParameterDescription::create<std::int64_t>("dimensions", 2),
    ParameterDescription::create<double>("position_tolerance", 0.25),
    ParameterDescription::create<double>("orientation_tolerance", 0.25),
    ParameterDescription::create<double>("dwell_time_sec", 0.0),
    ParameterDescription::create<double>("max_linear_speed", -1.0),
    ParameterDescription::create<double>("max_angular_speed", -1.0)
  )

  rpp::CallbackResult on_configure(const rpp::ComponentContext & context) override
  {
    if (active_) {
      return rpp::CallbackResult::failure("Deactivate the pose mission monitor before configuring");
    }
    const auto resource = context.get_parameter<std::string>("resource");
    const auto frame = context.get_parameter<std::string>("frame_id");
    const auto child = context.get_parameter<std::string>("child_frame_id");
    const auto dimensions = context.get_parameter<std::int64_t>("dimensions");
    const double position = context.get_parameter<double>("position_tolerance");
    const double orientation = context.get_parameter<double>("orientation_tolerance");
    const double dwell = context.get_parameter<double>("dwell_time_sec");
    const double linear_speed = context.get_parameter<double>("max_linear_speed");
    const double angular_speed = context.get_parameter<double>("max_angular_speed");
    if (resource.empty() || frame.empty() || child.empty() ||
      (dimensions != 2 && dimensions != 3) ||
      !std::isfinite(position) || position < 0.0 || !std::isfinite(orientation) ||
      orientation < 0.0 ||
      !std::isfinite(dwell) || dwell < 0.0 ||
      static_cast<long double>(dwell) * 1e9L > std::numeric_limits<std::uint64_t>::max() ||
      !valid_speed_limit(linear_speed) || !valid_speed_limit(angular_speed))
    {
      return rpp::CallbackResult::failure(
        "Provide a resource/frames, 2 or 3 dimensions, finite nonnegative tolerances/dwell, and speed limits >= 0 or -1 to disable");
    }
    dimensions_ = static_cast<std::size_t>(dimensions);
    const auto orientation_kind = dimensions_ ==
      2 ? rpp_interfaces::contract::COORDINATE_CONTINUOUS_ANGLE :
      rpp_interfaces::contract::COORDINATE_QUATERNION;
    const auto orientation_quantity = dimensions_ ==
      2 ? rpp_interfaces::contract::QUANTITY_ANGULAR_POSITION :
      rpp_interfaces::contract::QUANTITY_ORIENTATION;
    std::vector<rpp_interfaces::Field> fields{
      {resource + ".position", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_LINEAR_POSITION, static_cast<std::uint32_t>(dimensions_),
        frame, child, resource},
      {resource + ".orientation", orientation_kind, orientation_quantity,
        dimensions_ == 2 ? 1u : 4u, frame, child, resource}};
    goal_ = rpp_interfaces::FieldLayout(fields);
    previous_goal_.resize(goal_.size());
    linear_speed_offset_ = angular_speed_offset_ = missing_offset;
    std::size_t width = goal_.size();
    if (linear_speed >= 0.0) {
      linear_speed_offset_ = width;
      fields.push_back({resource + ".linearVelocity",
          rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
          rpp_interfaces::contract::QUANTITY_LINEAR_VELOCITY,
          static_cast<std::uint32_t>(dimensions_), child, child, resource});
      width += dimensions_;
    }
    if (angular_speed >= 0.0) {
      angular_speed_offset_ = width;
      fields.push_back({resource + ".angularVelocity",
          rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
          rpp_interfaces::contract::QUANTITY_ANGULAR_VELOCITY, dimensions_ == 2 ? 1u : 3u, child,
          child, resource});
    }
    state_ = rpp_interfaces::FieldLayout(std::move(fields));
    position_tolerance_ = position;
    orientation_tolerance_ = orientation;
    dwell_ns_ = static_cast<std::uint64_t>(static_cast<long double>(dwell) * 1e9L);
    max_linear_speed_ = linear_speed;
    max_angular_speed_ = angular_speed;
    configured_ = true;
    reset();
    return {};
  }

  Description::Const getDesc() override
  {
    if (!configured_) {throw std::logic_error("Configure the pose mission monitor first");}
    Description description;
    auto state = description.state();
    state_.write(state);
    auto goal = description.goal();
    goal_.write(goal);
    return Description::Const(std::move(description));
  }

  std::uint8_t resetHistory() override
  {
    if (!configured_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    reset();
    return rpp_interfaces::contract::CODE_OK;
  }

  CheckResult isOk(
    rpp::ListConst<double> state, rpp::ListConst<double> reference,
    std::uint64_t) override
  {
    if (!active_) {
      return {false, rpp_interfaces::contract::CODE_NOT_READY, "Pose mission monitor is inactive."};
    }
    if (state.size() != state_.size() || !reference.empty() || !state_.valid_row(state)) {
      return {false, rpp_interfaces::contract::CODE_INVALID_INPUT,
        "Invalid measured pose or unexpected reference fields."};
    }
    return {true, rpp_interfaces::contract::CODE_OK, ""};
  }

  Result isGoalReached(
    rpp::ListConst<double> state, rpp::ListConst<double> goal,
    std::uint64_t time_ns) override
  {
    if (!active_) {return {false, rpp_interfaces::contract::CODE_NOT_READY};}
    if (state.size() != state_.size() || goal.size() != goal_.size() ||
      !state_.valid_row(state) || !goal_.valid_row(goal))
    {
      return {false, rpp_interfaces::contract::CODE_INVALID_INPUT};
    }
    if (has_time_ && time_ns < last_time_ns_) {
      return {false, rpp_interfaces::contract::CODE_STALE_INPUT};
    }
    has_time_ = true;
    last_time_ns_ = time_ns;
    bool same_goal = has_goal_;
    for (std::size_t axis = 0; axis < goal.size(); ++axis) {
      same_goal = same_goal && previous_goal_[axis] == goal[axis];
      previous_goal_[axis] = goal[axis];
    }
    if (!same_goal) {arriving_ = false;}
    has_goal_ = true;
    double distance = 0.0;
    for (std::size_t axis = 0; axis < dimensions_; ++axis) {
      distance = std::hypot(distance,
        static_cast<double>(state[axis]) - static_cast<double>(goal[axis]));
    }
    double angle;
    if (dimensions_ == 2) {
      angle = std::abs(std::remainder(static_cast<double>(state[2]) - static_cast<double>(goal[2]),
        6.28318530717958647692));
    } else {
      double dot = 0.0;
      for (std::size_t axis = 3; axis < 7; ++axis) {dot += state[axis] * goal[axis];}
      angle = 2.0 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
    }
    const bool within = distance <= position_tolerance_ && angle <= orientation_tolerance_ &&
      speed_within(state, linear_speed_offset_, dimensions_, max_linear_speed_) &&
      speed_within(state, angular_speed_offset_, dimensions_ == 2 ? 1 : 3, max_angular_speed_);
    // Recheck every condition: a previous arrival must not remain latched after drift.
    if (!within) {arriving_ = false; return {false, rpp_interfaces::contract::CODE_OK};}
    if (!arriving_) {arrival_ns_ = time_ns; arriving_ = true;}
    return {time_ns - arrival_ns_ >= dwell_ns_, rpp_interfaces::contract::CODE_OK};
  }

  rpp::CallbackResult on_activate() override
  {
    if (!configured_) {
      return rpp::CallbackResult::failure("Configure the pose mission monitor first");
    }
    reset();
    active_ = true;
    return {};
  }
  rpp::CallbackResult on_deactivate() override {active_ = false; return {};}
  rpp::CallbackResult on_cleanup() override
  {
    active_ = configured_ = false;
    state_ = {};
    goal_ = {};
    previous_goal_.resize(0);
    reset();
    return {};
  }
  void reset() override {has_time_ = arriving_ = has_goal_ = false;}

private:
  static bool valid_speed_limit(double value)
  {
    return std::isfinite(value) && (value >= 0.0 || value == -1.0);
  }
  static bool speed_within(
    rpp::ListConst<double> state, std::size_t offset, std::size_t size,
    double limit)
  {
    if (offset == missing_offset) {return true;}
    double speed = 0.0;
    for (std::size_t axis = 0; axis < size; ++axis) {
      speed = std::hypot(speed, static_cast<double>(state[offset + axis]));
    }
    return speed <= limit;
  }
  static constexpr std::size_t missing_offset = std::numeric_limits<std::size_t>::max();
  rpp_interfaces::FieldLayout state_, goal_;
  rpp::List<double> previous_goal_;
  std::size_t dimensions_ = 2, linear_speed_offset_ = missing_offset,
    angular_speed_offset_ = missing_offset;
  double position_tolerance_ = 0.25, orientation_tolerance_ = 0.25;
  double max_linear_speed_ = -1.0, max_angular_speed_ = -1.0;
  std::uint64_t dwell_ns_ = 0, arrival_ns_ = 0, last_time_ns_ = 0;
  bool configured_ = false, active_ = false, has_time_ = false, arriving_ = false,
    has_goal_ = false;
};
