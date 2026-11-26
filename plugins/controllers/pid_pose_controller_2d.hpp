#pragma once

#include <rpp_control/pid_control_law.hpp>

class PIDPoseController2D final : public rpp_control::PidControlLaw<PIDPoseController2D, 3>
{
public:
  explicit PIDPoseController2D(bool validate_compute = true)
  : PidControlLaw(validate_compute) {}

  using ParameterDescription = rpp::params::ParameterDescription;
  RPP_PARAMETERS(
    ParameterDescription::create<PidParameters>("pid_x"),
    ParameterDescription::create<PidParameters>("pid_y"),
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
        rpp_interfaces::contract::QUANTITY_LINEAR_POSITION, 2, frame, child, resource},
      {resource + ".orientation", rpp_interfaces::contract::COORDINATE_CONTINUOUS_ANGLE,
        rpp_interfaces::contract::QUANTITY_ANGULAR_POSITION, 1, frame, child, resource}
    }};
    FieldsDesc commands{{
      {resource + ".linearVelocity", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_LINEAR_VELOCITY, 2, child, child, resource},
      {resource + ".angularVelocity", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_ANGULAR_VELOCITY, 1, child, child, resource}
    }};
    return configure_pid(context, state, state, commands, {
      context.get_parameter<PidParameters>("pid_x"),
      context.get_parameter<PidParameters>("pid_y"),
      context.get_parameter<PidParameters>("pid_n")}, {false, false, true});
  }

  bool axis_input_available(rpp::ListConst<bool> valid, std::size_t axis) const noexcept
  {
    if (valid.empty()) {return true;}
    // World-frame x/y errors both contribute to the two body-frame outputs.
    return axis < 2 ? valid[0] || valid[1] : valid[2];
  }

  std::array<double, 3> calculate_error(
    rpp::ListConst<double> state, rpp::ListConst<double> references) const noexcept
  {
    const auto dx = references[0] - state[0];
    const auto dy = references[1] - state[1];
    const auto c = std::cos(state[2]);
    const auto s = std::sin(state[2]);
    return {c * dx + s * dy, -s * dx + c * dy, rpp_control::wrap_angle(references[2] - state[2])};
  }
};
