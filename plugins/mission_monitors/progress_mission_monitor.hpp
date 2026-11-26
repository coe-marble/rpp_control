#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_interfaces/MissionMonitor.hpp>
#include <rpp_schema/rpp_interfaces/MissionMonitorDesc.hpp>
#include <rpp_interfaces/field_layout.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

class ProgressMissionMonitor final : public rpp_interfaces::MissionMonitor
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
    ParameterDescription::create<double>("required_movement_radius", 0.5),
    ParameterDescription::create<double>("movement_time_allowance", 10.0),
    ParameterDescription::create<double>("max_tracking_error", -1.0)
  )

  rpp::CallbackResult on_configure(const rpp::ComponentContext & context) override
  {
    if (active_) {return rpp::CallbackResult::failure("Deactivate the mission monitor before configuring");}
    const auto resource = context.get_parameter<std::string>("resource");
    const auto frame = context.get_parameter<std::string>("frame_id");
    const auto child = context.get_parameter<std::string>("child_frame_id");
    const auto dimensions = context.get_parameter<std::int64_t>("dimensions");
    const double movement = context.get_parameter<double>("required_movement_radius");
    const double allowance = context.get_parameter<double>("movement_time_allowance");
    const double tracking = context.get_parameter<double>("max_tracking_error");
    if (resource.empty() || frame.empty() || child.empty() || (dimensions != 2 && dimensions != 3) ||
      !std::isfinite(movement) || movement <= 0.0 || !std::isfinite(allowance) || allowance <= 0.0 ||
      static_cast<long double>(allowance) * 1e9L < 1.0L ||
      static_cast<long double>(allowance) * 1e9L > std::numeric_limits<std::uint64_t>::max() ||
      !std::isfinite(tracking) || (tracking < 0.0 && tracking != -1.0)) {
      return rpp::CallbackResult::failure("Provide a resource/frames, 2 or 3 dimensions, positive movement/time allowance, and nonnegative tracking limit or -1 to disable");
    }
    dimensions_ = static_cast<std::size_t>(dimensions);
    state_ = rpp_interfaces::FieldLayout({{resource + ".position", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
      rpp_interfaces::contract::QUANTITY_LINEAR_POSITION, static_cast<std::uint32_t>(dimensions_), frame, child, resource}});
    reference_ = tracking >= 0.0 ? state_ : rpp_interfaces::FieldLayout{};
    movement_radius_ = movement;
    allowance_ns_ = static_cast<std::uint64_t>(static_cast<long double>(allowance) * 1e9L);
    tracking_limit_ = tracking;
    configured_ = true;
    reset();
    return {};
  }

  Description::Const getDesc() override
  {
    if (!configured_) {throw std::logic_error("Configure the mission monitor first");}
    Description description;
    auto state = description.state(); state_.write(state);
    auto reference = description.reference(); reference_.write(reference);
    return Description::Const(std::move(description));
  }

  std::uint8_t resetHistory() override
  {
    if (!configured_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    reset();
    return rpp_interfaces::contract::CODE_OK;
  }

  CheckResult isOk(rpp::ListConst<double> state, rpp::ListConst<double> reference, std::uint64_t time_ns) override
  {
    if (!active_) {return {false, rpp_interfaces::contract::CODE_NOT_READY, "Progress mission monitor is inactive."};}
    if (state.size() != state_.size() || reference.size() != reference_.size() ||
      !state_.valid_row(state) || !reference_.valid_row(reference)) {
      return {false, rpp_interfaces::contract::CODE_INVALID_INPUT, "Invalid measured state or reference values."};
    }
    if (has_baseline_ && time_ns < last_time_ns_) {
      return {false, rpp_interfaces::contract::CODE_STALE_INPUT, "Mission clock moved backwards."};
    }
    last_time_ns_ = time_ns;
    if (stop_reason_) {return {false, rpp_interfaces::contract::CODE_OK, stop_reason_};}
    double movement = 0.0, tracking_error = 0.0;
    for (std::size_t axis = 0; axis < dimensions_; ++axis) {
      movement = std::hypot(movement, static_cast<double>(state[axis]) - baseline_[axis]);
      if (tracking_limit_ >= 0.0) {
        tracking_error = std::hypot(tracking_error, static_cast<double>(state[axis]) - static_cast<double>(reference[axis]));
      }
    }
    if (!has_baseline_ || movement >= movement_radius_) {
      for (std::size_t axis = 0; axis < dimensions_; ++axis) {baseline_[axis] = state[axis];}
      baseline_ns_ = time_ns;
      has_baseline_ = true;
    }
    // Policy trips remain stopped until a new mission explicitly resets history.
    if (tracking_limit_ >= 0.0 && tracking_error > tracking_limit_) {
      stop_reason_ = "Tracking error exceeds the configured limit.";
    } else if (time_ns - baseline_ns_ > allowance_ns_) {
      stop_reason_ = "Required movement was not achieved within the allowed time.";
    }
    return {stop_reason_ == nullptr, rpp_interfaces::contract::CODE_OK, stop_reason_ ? stop_reason_ : ""};
  }

  Result isGoalReached(rpp::ListConst<double>, rpp::ListConst<double>, std::uint64_t) override
  {
    return {false, rpp_interfaces::contract::CODE_UNSUPPORTED};
  }

  rpp::CallbackResult on_activate() override
  {
    if (!configured_) {return rpp::CallbackResult::failure("Configure the mission monitor first");}
    reset();
    active_ = true;
    return {};
  }
  rpp::CallbackResult on_deactivate() override {active_ = false; return {};}
  rpp::CallbackResult on_cleanup() override
  {
    active_ = configured_ = false;
    state_ = {};
    reference_ = {};
    reset();
    return {};
  }
  void reset() override {has_baseline_ = false; stop_reason_ = nullptr;}

private:
  rpp_interfaces::FieldLayout state_, reference_;
  std::array<double, 3> baseline_{};
  std::size_t dimensions_ = 2;
  double movement_radius_ = 0.5, tracking_limit_ = -1.0;
  std::uint64_t allowance_ns_ = 10000000000, baseline_ns_ = 0, last_time_ns_ = 0;
  bool configured_ = false, active_ = false, has_baseline_ = false;
  const char * stop_reason_ = nullptr;
};
