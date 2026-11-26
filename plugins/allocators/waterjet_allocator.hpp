#pragma once

#include <tuple>

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_interfaces/ActuationMapper.hpp>
#include <rpp_interfaces/field_layout.hpp>
#include <rpp_control/numeric_output_pool.hpp>
#include <rpp_control/control_plugin_contract.hpp>
#include <rpp_control/auxiliary_state.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

class WaterjetAllocator final : public rpp_interfaces::ActuationMapper
{
public:
  using CommandMapResult = std::tuple<rpp::ListConst<double>, rpp::ListConst<double>, std::uint8_t>;
  using FeedbackMapResult = std::tuple<rpp::ListConst<double>, std::uint8_t>;
  using ParameterDescription = rpp::params::ParameterDescription;
  RPP_PARAMETERS(
        ParameterDescription::create<double>("jet_x", -1.5),
        ParameterDescription::create<double>("jet_y", 0.0),
        ParameterDescription::create<double>("max_thrust", 6000.0),
        ParameterDescription::create<double>("max_angle", 0.5235987756),
        ParameterDescription::create<double>("mass", 450.0),
        ParameterDescription::create<double>("z_cg", 0.4),
        ParameterDescription::create<double>("metacentric_height", 0.3),
        ParameterDescription::create<double>("max_safe_roll_deg", 30.0),
        ParameterDescription::create<bool>("yaw_priority", true),
        ParameterDescription::create<std::string>("body_frame_id", "base_link"),
        ParameterDescription::create<std::string>("resource", "base"),
        ParameterDescription::create<std::string>("actuator_resource", "waterjet")
  )

  rpp::CallbackResult on_configure(const rpp::ComponentContext & context) override
  {
    if (active_) {return rpp::CallbackResult::failure("Deactivate the allocator before configuring");}
    Parameters parameters{
      context.get_parameter<double>("jet_x"), context.get_parameter<double>("jet_y"),
      context.get_parameter<double>("max_thrust"), context.get_parameter<double>("max_angle"),
      context.get_parameter<double>("mass"), context.get_parameter<double>("z_cg"),
      context.get_parameter<double>("metacentric_height"),
      context.get_parameter<double>("max_safe_roll_deg") * kPi / 180.0};
    validate_configuration(parameters);
    const auto frame = context.get_parameter<std::string>("body_frame_id");
    const auto resource = context.get_parameter<std::string>("resource");
    const auto actuator = context.get_parameter<std::string>("actuator_resource");
    if (frame.empty() || resource.empty() || actuator.empty()) {
      return rpp::CallbackResult::failure("Waterjet frame and resource names must be nonempty");
    }
    rpp_interfaces::FieldLayout state({
      {resource + ".linearVelocity", rpp_interfaces::contract::COORDINATE_EUCLIDEAN, rpp_interfaces::contract::QUANTITY_LINEAR_VELOCITY, 2, frame, frame, resource},
      {resource + ".angularVelocity", rpp_interfaces::contract::COORDINATE_EUCLIDEAN, rpp_interfaces::contract::QUANTITY_ANGULAR_VELOCITY, 1, frame, frame, resource}});
    rpp_interfaces::FieldLayout commands({
      {resource + ".force", rpp_interfaces::contract::COORDINATE_EUCLIDEAN, rpp_interfaces::contract::QUANTITY_FORCE, 2, frame, frame, resource},
      {resource + ".moment", rpp_interfaces::contract::COORDINATE_EUCLIDEAN, rpp_interfaces::contract::QUANTITY_MOMENT, 1, frame, frame, resource}});
    rpp_interfaces::FieldLayout actuators({
      {actuator + ".thrust", rpp_interfaces::contract::COORDINATE_EUCLIDEAN, rpp_interfaces::contract::QUANTITY_NORMALIZED, 1, frame, frame, actuator},
      {actuator + ".nozzle", rpp_interfaces::contract::COORDINATE_EUCLIDEAN, rpp_interfaces::contract::QUANTITY_NORMALIZED, 1, frame, frame, actuator}});
    rpp_control::NumericOutputPool actuator_outputs, realized_outputs;
    actuator_outputs.prepare(2);
    realized_outputs.prepare(3);
    const auto yaw_priority = context.get_parameter<bool>("yaw_priority");
    parameters_ = parameters;
    state_ = std::move(state); commands_ = std::move(commands); actuators_ = std::move(actuators);
    actuator_outputs_ = std::move(actuator_outputs);
    realized_outputs_ = std::move(realized_outputs);
    yaw_priority_ = yaw_priority;
    configured_ = true;
    return {};
  }

  ActuationMapperDesc::Const getDesc() override
  {
    if (!configured_) {throw std::logic_error("Configure the waterjet allocator first");}
    ActuationMapperDesc desc;
    auto state = desc.state();
    auto control = desc.control();
    auto actuators = desc.actuators();
    state_.write(state);
    commands_.write(control);
    actuators_.write(actuators);
    auto limits = desc.limits();
    limits.resize(1);
    auto limit = limits[0];
    auto fields = limit.fields();
    actuators_.write(fields);
    auto lower = limit.lower(); auto upper = limit.upper();
    lower.resize(2); upper.resize(2);
    for (std::size_t i = 0; i < 2; ++i) {lower[i] = -1.0; upper[i] = 1.0;}
    return ActuationMapperDesc::Const(std::move(desc));
  }

  CommandMapResult commandMap(
    rpp::ListConst<double> state, rpp::ListConst<double> command, std::uint64_t dtNs) override
  {
    if (!active_) {return failure(rpp_interfaces::contract::CODE_NOT_READY);}
    if (state.size() != 3 || command.size() != 3 || dtNs == 0 || !finite(state) || !finite(command)) {
      return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
    }
    const auto actuator_buffer = actuator_outputs_.acquire(), realized_buffer = realized_outputs_.acquire();
    if (actuator_buffer == rpp_control::NumericOutputPool::unavailable || realized_buffer == rpp_control::NumericOutputPool::unavailable) {
      return failure(rpp_interfaces::contract::CODE_NOT_READY);
    }
    const double tau_x = command[0];
    const double tau_y = command[1];
    const double tau_n = command[2];
    // Yaw priority reserves the nozzle's lateral force for yaw control.
    const double requested_y_force = yaw_priority_ &&
      std::abs(parameters_.jet_x) > 1e-6 ?
      (tau_n + parameters_.jet_y * tau_x) / parameters_.jet_x :
      tau_y;
    const double current_u = state[0];
    const double current_r = state[2];
    const double roll_moment_centripetal =
      parameters_.mass * current_u * current_r * parameters_.z_cg;
    constexpr double gravity = 9.81;
    const double max_restoring_moment = parameters_.mass * gravity *
      parameters_.metacentric_height * std::sin(parameters_.max_safe_roll_rad);

    if (!std::isfinite(max_restoring_moment) ||
      !std::isfinite(roll_moment_centripetal))
    {
      return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
    }

    const double max_allowable_fy_jet =
      max_restoring_moment > std::abs(roll_moment_centripetal) ?
      (max_restoring_moment - std::abs(roll_moment_centripetal)) / parameters_.z_cg :
      0.0;

    const double max_lateral_force = std::min(max_allowable_fy_jet,
            parameters_.max_thrust * std::sin(parameters_.max_angle));
    const double allocated_y_force = std::clamp(requested_y_force,
            -max_lateral_force, max_lateral_force);
    const double thrust_direction = tau_x < 0.0 ? -1.0 : 1.0;
    const double min_abs_x_force = std::abs(allocated_y_force) /
      std::tan(parameters_.max_angle);
    const double max_abs_x_force = std::sqrt(std::max(0.0,
            parameters_.max_thrust * parameters_.max_thrust -
              allocated_y_force * allocated_y_force));
    const double allocated_abs_x_force = std::min(
            std::max(std::abs(tau_x), min_abs_x_force), max_abs_x_force);
    double thrust = thrust_direction * std::hypot(
            allocated_abs_x_force, allocated_y_force);
    double angle = std::atan2(thrust_direction * allocated_y_force,
            allocated_abs_x_force);
    angle = std::clamp(angle, -parameters_.max_angle, parameters_.max_angle);
    if (!std::isfinite(thrust) || !std::isfinite(angle)) {
      return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
    }

    auto & actuators = actuator_outputs_.values(actuator_buffer);
    actuators[0] = std::clamp(thrust / parameters_.max_thrust, -1.0, 1.0);
    actuators[1] = std::clamp(angle / parameters_.max_angle, -1.0, 1.0);
    const auto estimated_wrench = physical_wrench(actuators[0], actuators[1]);
    auto & realizedEstimate = realized_outputs_.values(realized_buffer);
    bool limited = false;
    for (std::size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(estimated_wrench[i])) {return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);}
      realizedEstimate[i] = estimated_wrench[i];
      limited = limited || std::abs(estimated_wrench[i] - command[i]) >
        1e-9 * std::max({1.0, std::abs(estimated_wrench[i]), std::abs(command[i])});
    }
    return {actuator_outputs_.view(actuator_buffer), realized_outputs_.view(realized_buffer),
      limited ? rpp_interfaces::contract::CODE_LIMITED : rpp_interfaces::contract::CODE_OK};
  }

  FeedbackMapResult feedbackMap(rpp::ListConst<double> state, rpp::ListConst<double> actuators) override
  {
    if (!active_) {return {realized_outputs_.empty(), rpp_interfaces::contract::CODE_NOT_READY};}
    if (state.size() != 3 || actuators.size() != 2 || !finite(state) || !finite(actuators) ||
      std::abs(actuators[0]) > 1 || std::abs(actuators[1]) > 1)
    {
      return {realized_outputs_.empty(), rpp_interfaces::contract::CODE_INVALID_INPUT};
    }
    const auto buffer = realized_outputs_.acquire();
    if (buffer == rpp_control::NumericOutputPool::unavailable) {return {realized_outputs_.empty(), rpp_interfaces::contract::CODE_NOT_READY};}
    const auto mapped_wrench = physical_wrench(actuators[0], actuators[1]);
    auto & realized = realized_outputs_.values(buffer);
    for (std::size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(mapped_wrench[i])) {return {realized_outputs_.empty(), rpp_interfaces::contract::CODE_INVALID_INPUT};}
      realized[i] = mapped_wrench[i];
    }
    return {realized_outputs_.view(buffer), rpp_interfaces::contract::CODE_OK};
  }

  rpp_control::AuxiliaryState::Snapshot::Const getAuxiliaryState() override
  {
    return rpp_control::AuxiliaryState{}.snapshot();
  }

  rpp::CallbackResult on_activate(const std::optional<HandoverState::Const> & state) override
  {
    if (!configured_) {return rpp::CallbackResult::failure("Configure the waterjet allocator first");}
    if (state) {
      try {
        const auto measured = state->state();
        const auto previous_control = state->previousControl();
        rpp_control::validate_handover_values(measured);
        rpp_control::validate_handover_values(previous_control);
        rpp_control::validate_handover_values(state->auxiliaryState());
        if (!measured.state().empty()) {
          const rpp_interfaces::FieldLayout available_state(measured.desc().as_struct().fields);
          (void)rpp_interfaces::field_route(available_state, state_);
        }
        if (!previous_control.state().empty()) {
          const rpp_interfaces::FieldLayout available_controls(previous_control.desc().as_struct().fields);
          (void)rpp_control::handover_control_route(available_controls, commands_);
        }
        // This stateless mapper does not restore allocator auxiliary history.
      } catch (const std::invalid_argument & error) {
        return rpp::CallbackResult::failure(error.what());
      }
    }
    active_ = true;
    return {};
  }
  rpp::CallbackResult on_deactivate() override
  {
    active_ = false;
    return {};
  }
  rpp::CallbackResult on_cleanup() override
  {
    active_ = false; configured_ = false;
    actuator_outputs_.clear(); realized_outputs_.clear();
    state_ = rpp_interfaces::FieldLayout{}; commands_ = rpp_interfaces::FieldLayout{}; actuators_ = rpp_interfaces::FieldLayout{};
    return {};
  }

private:
  static constexpr double kPi = 3.14159265358979323846;
  struct Parameters {
    double jet_x, jet_y, max_thrust, max_angle, mass, z_cg, metacentric_height, max_safe_roll_rad;
  };
  Parameters parameters_{};
  rpp_interfaces::FieldLayout state_, commands_, actuators_;
  rpp_control::NumericOutputPool actuator_outputs_, realized_outputs_;
  bool active_ = false;
  bool configured_ = false, yaw_priority_ = true;

  CommandMapResult failure(std::uint8_t code) const
  {return {actuator_outputs_.empty(), realized_outputs_.empty(), code};}
  static bool finite(rpp::ListConst<double> values)
  {
    for (std::size_t i = 0; i < values.size(); ++i) {if (!std::isfinite(values[i])) {return false;}}
    return true;
  }
  std::array<double, 3> physical_wrench(double thrust_normalized, double angle_normalized) const noexcept
  {
    const auto thrust = thrust_normalized * parameters_.max_thrust;
    const auto angle = angle_normalized * parameters_.max_angle;
    const auto x = thrust * std::cos(angle), y = thrust * std::sin(angle);
    return {x, y, parameters_.jet_x * y - parameters_.jet_y * x};
  }
  static void validate_configuration(const Parameters & p)
  {
    constexpr double pi = kPi;
    for (const auto value : {p.jet_x, p.jet_y, p.max_thrust, p.max_angle, p.mass,
      p.z_cg, p.metacentric_height, p.max_safe_roll_rad})
    {
      if (!std::isfinite(value)) {throw std::invalid_argument("Waterjet allocator parameters must be finite");}
    }
    if (p.max_thrust <= 0 || p.max_angle <= 0 || p.max_angle > pi / 2 || p.mass <= 0 ||
      p.z_cg <= 0 || p.metacentric_height <= 0 || p.max_safe_roll_rad < 0 || p.max_safe_roll_rad >= pi / 2 ||
      !std::isfinite(p.max_thrust * p.max_thrust) ||
      !std::isfinite(p.mass * 9.81 * p.metacentric_height * std::sin(p.max_safe_roll_rad)))
    {
      throw std::invalid_argument("Waterjet allocator parameters are outside safe limits");
    }
  }
};
