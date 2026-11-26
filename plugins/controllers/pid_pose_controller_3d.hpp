#pragma once

#include <rpp_control/pid_control_law.hpp>

class PIDPoseController3D final : public rpp_control::PidControlLaw<PIDPoseController3D, 6>
{
public:
  explicit PIDPoseController3D(bool validate_compute = true)
  : PidControlLaw(validate_compute) {}

  using ParameterDescription = rpp::params::ParameterDescription;
  RPP_PARAMETERS(
    ParameterDescription::create<PidParameters>("pid_x"),
    ParameterDescription::create<PidParameters>("pid_y"),
    ParameterDescription::create<PidParameters>("pid_z"),
    ParameterDescription::create<PidParameters>("pid_k"),
    ParameterDescription::create<PidParameters>("pid_m"),
    ParameterDescription::create<PidParameters>("pid_n"),
    ParameterDescription::create<std::string>("frame_id", "map"),
    ParameterDescription::create<std::string>("child_frame_id", "base_link"),
    ParameterDescription::create<std::string>("resource", "base")
  )

  rpp::CallbackResult on_configure(const rpp::ComponentContext & context) override
  {
    const auto frame = context.get_parameter<std::string>("frame_id");
    const auto child = context.get_parameter<std::string>("child_frame_id");
    const auto resource = context.get_parameter<std::string>("resource");
    if (frame.empty() || child.empty() || resource.empty()) {
      return rpp::CallbackResult::failure("Pose PID frame names and resource must be nonempty");
    }
    FieldsDesc state{{
      {resource + ".position", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_LINEAR_POSITION, 3, frame, child, resource},
      {resource + ".orientation", rpp_interfaces::contract::COORDINATE_QUATERNION,
        rpp_interfaces::contract::QUANTITY_ORIENTATION, 4, frame, child, resource}
    }};
    FieldsDesc commands{{
      {resource + ".linearVelocity", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_LINEAR_VELOCITY, 3, child, child, resource},
      {resource + ".angularVelocity", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_ANGULAR_VELOCITY, 3, child, child, resource}
    }};
    return configure_pid(context, state, state, commands, {
      context.get_parameter<PidParameters>("pid_x"), context.get_parameter<PidParameters>("pid_y"),
      context.get_parameter<PidParameters>("pid_z"), context.get_parameter<PidParameters>("pid_k"),
      context.get_parameter<PidParameters>("pid_m"),
      context.get_parameter<PidParameters>("pid_n")});
  }

  bool axis_input_available(rpp::ListConst<bool> valid, std::size_t axis) const noexcept
  {
    if (valid.empty()) {return true;}
    if (axis < 3) {return valid[0] || valid[1] || valid[2];}
    return valid[3] && valid[4] && valid[5] && valid[6];
  }

  std::array<double, 6> calculate_error(
    rpp::ListConst<double> state, rpp::ListConst<double> references) const noexcept
  {
    // Both translation and rotation errors are expressed in the current child frame.
    const auto state_norm = std::sqrt(state[3] * state[3] + state[4] * state[4] +
      state[5] * state[5] + state[6] * state[6]);
    const double x = -state[3] / state_norm, y = -state[4] / state_norm;
    const double z = -state[5] / state_norm, w = state[6] / state_norm;
    const std::array<double, 3> d{
      references[0] - state[0], references[1] - state[1], references[2] - state[2]};
    const std::array<double, 3> t{
      2 * (y * d[2] - z * d[1]), 2 * (z * d[0] - x * d[2]), 2 * (x * d[1] - y * d[0])};
    std::array<double, 6> error{
      d[0] + w * t[0] + y * t[2] - z * t[1],
      d[1] + w * t[1] + z * t[0] - x * t[2],
      d[2] + w * t[2] + x * t[1] - y * t[0], 0, 0, 0};
    std::array<double, 3> v{
      w * references[3] + x * references[6] + y * references[5] - z * references[4],
      w * references[4] - x * references[5] + y * references[6] + z * references[3],
      w * references[5] + x * references[4] - y * references[3] + z * references[6]};
    auto qw = w * references[6] - x * references[3] - y * references[4] - z * references[5];
    bool negate = qw < 0;
    if (qw == 0) {
      for (const auto value : v) {if (value != 0) {negate = value < 0; break;}}
    }
    if (negate) {qw = -qw; for (auto & value : v) {value = -value;}}
    const auto norm = std::hypot(v[0], v[1], v[2]);
    if (norm > 0) {
      const auto scale = 2 * std::atan2(norm, qw) / norm;
      for (std::size_t i = 0; i < v.size(); ++i) {error[3 + i] = v[i] * scale;}
    }
    return error;
  }
};
