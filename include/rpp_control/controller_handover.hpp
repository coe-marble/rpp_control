#pragma once

#include <cmath>

#include <rpp_control/angle.hpp>


namespace rpp_control
{

// Optional native error history for one controlled axis. Controllers own its
// calculation, limits, and filtering; transport uses described auxiliary values.
struct ControllerAxisErrorInfo
{
  double integral_error = 0.0;
  double previous_error = 0.0;
  double previous_derivative = 0.0;
};

inline bool is_valid_controller_axis_error_info(const ControllerAxisErrorInfo & info)
{
  return std::isfinite(info.integral_error) &&
         std::isfinite(info.previous_error) &&
         std::isfinite(info.previous_derivative);
}

inline bool update_controller_axis_error_info(
  ControllerAxisErrorInfo & info, double reference, double state,
  double dt, bool integrate = true, bool angular = false, bool validate = true)
{
  if (validate && (!std::isfinite(reference) || !std::isfinite(state) ||
    !std::isfinite(dt) || dt <= 0.0))
  {
    return false;
  }
  auto error = reference - state;
  auto error_step = error - info.previous_error;
  // Angles: take the short way round, and no derivative jump when the error crosses +-pi.
  if (angular) {
    error = wrap_angle(error);
    error_step = wrap_angle(error - info.previous_error);
  }
  const auto derivative = error_step / dt;
  if (validate && (!std::isfinite(error) || !std::isfinite(derivative))) {return false;}

  const auto integral = integrate ? info.integral_error + error * dt : info.integral_error;
  if (validate && !std::isfinite(integral)) {return false;}

  info.integral_error = integral;
  info.previous_error = error;
  info.previous_derivative = derivative;
  return true;
}

}  // namespace rpp_control
