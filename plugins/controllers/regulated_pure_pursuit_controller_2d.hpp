#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_interfaces/ControlLaw.hpp>
#include <rpp_control/auxiliary_state.hpp>
#include <rpp_control/control_plugin_contract.hpp>
#include <rpp_control/numeric_output_pool.hpp>
#include <rpp_control/regulated_pure_pursuit.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

class RegulatedPurePursuitController2D final : public rpp_interfaces::ControlLaw
{
public:
  using ParameterDescription = rpp::params::ParameterDescription;
  using Result = std::tuple<rpp::ListConst<double>, std::uint8_t>;

  explicit RegulatedPurePursuitController2D(bool validate_compute = true)
  : validate_compute_(validate_compute) {}

  RPP_PARAMETERS(
    ParameterDescription::create<std::string>("resource"),
    ParameterDescription::create<std::string>("frame_id", "map"),
    ParameterDescription::create<std::string>("child_frame_id", "base_link"),
    ParameterDescription::create<std::int64_t>("path_capacity", 256),
    ParameterDescription::create<double>("desired_linear_vel", 1.0),
    ParameterDescription::create<double>("lookahead_dist", 1.0),
    ParameterDescription::create<double>("min_lookahead_dist", 0.3),
    ParameterDescription::create<double>("max_lookahead_dist", 2.0),
    ParameterDescription::create<double>("lookahead_time", 1.5),
    ParameterDescription::create<bool>("use_velocity_scaled_lookahead_dist", true),
    ParameterDescription::create<bool>("use_fixed_curvature_lookahead", false),
    ParameterDescription::create<double>("curvature_lookahead_dist", 1.0),
    ParameterDescription::create<bool>("interpolate_curvature_after_goal", false),
    ParameterDescription::create<double>("regulated_linear_scaling_min_radius", 1.0),
    ParameterDescription::create<double>("regulated_linear_scaling_min_speed", 0.1),
    ParameterDescription::create<double>("approach_velocity_scaling_dist", 1.0),
    ParameterDescription::create<double>("min_approach_linear_velocity", 0.05),
    ParameterDescription::create<double>("max_linear_accel", 1.0),
    ParameterDescription::create<double>("max_linear_decel", 1.0),
    ParameterDescription::create<double>("max_angular_vel", 1.0),
    ParameterDescription::create<double>("max_angular_accel", 2.0),
    ParameterDescription::create<double>("max_angular_decel", 2.0),
    ParameterDescription::create<bool>("use_rotate_to_heading", true),
    ParameterDescription::create<double>("rotate_to_heading_min_angle", 0.7853981633974483),
    ParameterDescription::create<double>("rotate_to_heading_angular_vel", 1.0),
    ParameterDescription::create<double>("goal_position_distance", 0.05),
    ParameterDescription::create<double>("heading_deadband", 0.05)
  )

  rpp::CallbackResult on_configure(const rpp::ComponentContext & context) override
  {
    if (active_) {
      return rpp::CallbackResult::failure("Deactivate pure pursuit before configuring");
    }
    const auto resource = context.get_parameter<std::string>("resource");
    const auto frame = context.get_parameter<std::string>("frame_id");
    const auto child = context.get_parameter<std::string>("child_frame_id");
    const auto capacity = context.get_parameter<std::int64_t>("path_capacity");
    if (resource.empty() || frame.empty() || child.empty() || capacity < 2 ||
      capacity > static_cast<std::int64_t>(rpp_control::PlanarPathLayout::max_capacity))
    {
      return rpp::CallbackResult::failure(
        "Provide nonempty resource/frames and path_capacity in [2,512]");
    }
    auto parameters = read_parameters(context);
    try {
      parameters.validate();
    } catch (const std::invalid_argument & error) {
      return rpp::CallbackResult::failure(error.what());
    }
    const rpp_control::PlanarPathLayout layout(static_cast<std::size_t>(capacity));
    auto fields = layout.fields(resource, frame, child);
    rpp_interfaces::FieldLayout references(fields);
    fields.resize(2);
    fields.push_back({resource + ".linearVelocity", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_LINEAR_VELOCITY, 2, child, child, resource});
    fields.push_back({resource + ".angularVelocity", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_ANGULAR_VELOCITY, 1, child, child, resource});
    rpp_interfaces::FieldLayout state(fields);
    fields.erase(fields.begin(), fields.begin() + 2);
    rpp_interfaces::FieldLayout commands(std::move(fields));
    outputs_.prepare(3);
    layout_ = layout;
    state_ = std::move(state);
    references_ = std::move(references);
    commands_ = std::move(commands);
    pursuit_ = rpp_control::RegulatedPurePursuit(parameters);
    reset();
    configured_ = true;
    return {};
  }

  ControlLawDesc::Const getDesc() override
  {
    if (!configured_) {throw std::logic_error("Configure pure pursuit first");}
    return rpp_control::control_description(state_, references_, commands_);
  }

  Result compute(
    rpp::ListConst<double> state, rpp::ListConst<double> reference,
    rpp::ListConst<bool> enabled_axes, rpp::ListConst<bool> input_valid,
    rpp::ListConst<double> previous_control, bool previous_control_valid,
    std::uint64_t time_ns, std::uint64_t dt_ns) override
  {
    if (validate_compute_) {
      if (!active_) {return failure(rpp_interfaces::contract::CODE_NOT_READY);}
      if (dt_ns == 0 || state.size() != state_.size() || reference.size() != references_.size() ||
        (!enabled_axes.empty() && enabled_axes.size() != 3) ||
        (!input_valid.empty() && input_valid.size() != reference.size()) ||
        !state_.valid_row(state) || !references_.valid_row(reference) ||
        (previous_control_valid &&
        (previous_control.size() != 3 || !commands_.valid_row(previous_control))))
      {
        return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
      }
      if (has_time_ && time_ns < last_time_ns_) {
        return failure(rpp_interfaces::contract::CODE_STALE_INPUT);
      }
    }
    const auto buffer = outputs_.acquire();
    if (buffer == rpp_control::NumericOutputPool::unavailable) {
      return failure(rpp_interfaces::contract::CODE_NOT_READY);
    }
    bool available = enabled_axes.empty() || (enabled_axes[0] && enabled_axes[2]);
    for (std::size_t scalar = 0; scalar < input_valid.size(); ++scalar) {
      if (!input_valid[scalar]) {available = false; break;}
    }
    rpp_control::RegulatedPurePursuit::Command result;
    if (available) {
      const double previous_linear = previous_control_valid ? previous_control[0] : state[3];
      const double previous_angular = previous_control_valid ? previous_control[2] : state[5];
      result = pursuit_.compute(state, reference, layout_, previous_linear, previous_angular,
        static_cast<double>(dt_ns) * 1e-9);
      if (!rpp_control::control_success(result.status)) {return failure(result.status);}
      if (!std::isfinite(result.linear) || !std::isfinite(result.angular)) {
        return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
      }
    }
    // Missing references and disabled coupled axes stop immediately, with no ramp tail.
    auto & values = outputs_.values(buffer);
    values[0] = result.linear;
    values[1] = 0.0;
    values[2] = result.angular;
    last_time_ns_ = time_ns;
    has_time_ = true;
    return {outputs_.view(buffer), result.status};
  }

  rpp_control::AuxiliaryState::Snapshot::Const getAuxiliaryState() override
  {return rpp_control::AuxiliaryState{}.snapshot();}

  rpp::CallbackResult on_activate(const std::optional<HandoverState::Const> &) override
  {
    if (!configured_) {return rpp::CallbackResult::failure("Configure pure pursuit first");}
    reset();
    active_ = true;
    return {};
  }
  rpp::CallbackResult on_deactivate() override {active_ = false; reset(); return {};}
  rpp::CallbackResult on_cleanup() override
  {
    active_ = configured_ = false;
    reset();
    outputs_.clear();
    state_ = {};
    references_ = {};
    commands_ = {};
    return {};
  }
  void reset() override {has_time_ = false;}

private:
  static rpp_control::PurePursuitParameters read_parameters(const rpp::ComponentContext & context)
  {
    rpp_control::PurePursuitParameters parameters;
    parameters.desired_linear_vel = context.get_parameter<double>("desired_linear_vel");
    parameters.lookahead_dist = context.get_parameter<double>("lookahead_dist");
    parameters.min_lookahead_dist = context.get_parameter<double>("min_lookahead_dist");
    parameters.max_lookahead_dist = context.get_parameter<double>("max_lookahead_dist");
    parameters.lookahead_time = context.get_parameter<double>("lookahead_time");
    parameters.use_velocity_scaled_lookahead_dist =
      context.get_parameter<bool>("use_velocity_scaled_lookahead_dist");
    parameters.use_fixed_curvature_lookahead =
      context.get_parameter<bool>("use_fixed_curvature_lookahead");
    parameters.curvature_lookahead_dist = context.get_parameter<double>("curvature_lookahead_dist");
    parameters.interpolate_curvature_after_goal =
      context.get_parameter<bool>("interpolate_curvature_after_goal");
    parameters.regulated_linear_scaling_min_radius =
      context.get_parameter<double>("regulated_linear_scaling_min_radius");
    parameters.regulated_linear_scaling_min_speed =
      context.get_parameter<double>("regulated_linear_scaling_min_speed");
    parameters.approach_velocity_scaling_dist =
      context.get_parameter<double>("approach_velocity_scaling_dist");
    parameters.min_approach_linear_velocity =
      context.get_parameter<double>("min_approach_linear_velocity");
    parameters.max_linear_accel = context.get_parameter<double>("max_linear_accel");
    parameters.max_linear_decel = context.get_parameter<double>("max_linear_decel");
    parameters.max_angular_vel = context.get_parameter<double>("max_angular_vel");
    parameters.max_angular_accel = context.get_parameter<double>("max_angular_accel");
    parameters.max_angular_decel = context.get_parameter<double>("max_angular_decel");
    parameters.use_rotate_to_heading = context.get_parameter<bool>("use_rotate_to_heading");
    parameters.rotate_to_heading_min_angle =
      context.get_parameter<double>("rotate_to_heading_min_angle");
    parameters.rotate_to_heading_angular_vel =
      context.get_parameter<double>("rotate_to_heading_angular_vel");
    parameters.goal_position_distance = context.get_parameter<double>("goal_position_distance");
    parameters.heading_deadband = context.get_parameter<double>("heading_deadband");
    return parameters;
  }

  Result failure(std::uint8_t status) const {return {outputs_.empty(), status};}

  const bool validate_compute_;
  rpp_control::PlanarPathLayout layout_{256};
  rpp_control::RegulatedPurePursuit pursuit_;
  rpp_interfaces::FieldLayout state_, references_, commands_;
  rpp_control::NumericOutputPool outputs_;
  std::uint64_t last_time_ns_ = 0;
  bool configured_ = false, active_ = false, has_time_ = false;
};
