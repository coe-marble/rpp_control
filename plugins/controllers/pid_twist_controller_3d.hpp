#pragma once

#include <rpp_control/pid_control_law.hpp>

class PIDTwistController3D final : public rpp_control::PidControlLaw<PIDTwistController3D, 6>
{
public:
  explicit PIDTwistController3D(bool validate_compute = true)
  : PidControlLaw(validate_compute) {}

  using ParameterDescription = rpp::params::ParameterDescription;
  RPP_PARAMETERS(
    ParameterDescription::create<PidParameters>("pid_x"),
    ParameterDescription::create<PidParameters>("pid_y"),
    ParameterDescription::create<PidParameters>("pid_z"),
    ParameterDescription::create<PidParameters>("pid_k"),
    ParameterDescription::create<PidParameters>("pid_m"),
    ParameterDescription::create<PidParameters>("pid_n"),
    ParameterDescription::create<std::string>("frame_id", "base_link"),
    ParameterDescription::create<std::string>("resource", "base"),
    ParameterDescription::create<bool>("scale_normalized_output", false),
    ParameterDescription::create<std::vector<double>>("positive_output_scales",
    std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}),
    ParameterDescription::create<std::vector<double>>("negative_output_scales",
    std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0})
  )

  rpp::CallbackResult on_configure(const rpp::ComponentContext & context) override
  {
    configure_output_scaling(context);
    const auto frame = context.get_parameter<std::string>("frame_id");
    const auto resource = context.get_parameter<std::string>("resource");
    if (frame.empty() || resource.empty()) {
      return rpp::CallbackResult::failure("Twist PID frame_id and resource must be nonempty");
    }
    FieldsDesc state{{
      {resource + ".linearVelocity", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_LINEAR_VELOCITY, 3, frame, frame, resource},
      {resource + ".angularVelocity", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_ANGULAR_VELOCITY, 3, frame, frame, resource}
    }};
    FieldsDesc commands{{
      {resource + ".force", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_FORCE, 3, frame, frame, resource},
      {resource + ".moment", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_MOMENT, 3, frame, frame, resource}
    }};
    return configure_pid(context, state, state, commands, {
      context.get_parameter<PidParameters>("pid_x"), context.get_parameter<PidParameters>("pid_y"),
      context.get_parameter<PidParameters>("pid_z"), context.get_parameter<PidParameters>("pid_k"),
      context.get_parameter<PidParameters>("pid_m"),
      context.get_parameter<PidParameters>("pid_n")});
  }

  bool axis_input_available(rpp::ListConst<bool> valid, std::size_t axis) const noexcept
  {return valid.empty() || valid[axis];}

  std::array<double, 6> calculate_error(
    rpp::ListConst<double> state, rpp::ListConst<double> references) const noexcept
  {
    std::array<double, 6> error{};
    for (std::size_t i = 0; i < error.size(); ++i) {error[i] = references[i] - state[i];}
    return error;
  }
};
