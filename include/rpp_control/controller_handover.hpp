#pragma once

#include <cmath>

#include <rpp_schema/rpp_control/ControllerAxisErrorInfo.hpp>

#include "control_defs.hpp"

namespace rpp_control
{

// Common retained error information for one controlled axis. Every controller
// updates this from its reference, measured value, and control period. A
// concrete controller may additionally apply its own limits or filtering.
struct ControllerAxisErrorInfo
{
  FP_TYPE integral_error = 0.0F;
  FP_TYPE previous_error = 0.0F;
  FP_TYPE previous_derivative = 0.0F;
};

inline bool is_valid_controller_axis_error_info(const ControllerAxisErrorInfo & info)
{
  return std::isfinite(info.integral_error) &&
         std::isfinite(info.previous_error) &&
         std::isfinite(info.previous_derivative);
}

inline ControllerAxisErrorInfo read_controller_axis_error_info(
  const rpp_schema::rpp_control::ControllerAxisErrorInfo::Const & source)
{
  return {
    static_cast<FP_TYPE>(source.integralError()),
    static_cast<FP_TYPE>(source.previousError()),
    static_cast<FP_TYPE>(source.previousDerivative())};
}

inline void write_controller_axis_error_info(
  rpp_schema::rpp_control::ControllerAxisErrorInfo destination,
  const ControllerAxisErrorInfo & source)
{
  destination.integralError() = source.integral_error;
  destination.previousError() = source.previous_error;
  destination.previousDerivative() = source.previous_derivative;
}

inline bool update_controller_axis_error_info(
  ControllerAxisErrorInfo & info, const FP_TYPE reference, const FP_TYPE state,
  const FP_TYPE dt, const bool integrate = true, const bool angular = false)
{
  if (!std::isfinite(reference) || !std::isfinite(state) ||
    !std::isfinite(dt) || dt <= 0.0)
  {
    return false;
  }
  auto error = reference - state;
  // An angle difference is wrapped to [-pi, pi] so the controller turns the short way.
  if (angular && std::isfinite(error)) {error = std::atan2(std::sin(error), std::cos(error));}
  auto error_delta = error - info.previous_error;
  // The step of an angle error is wrapped too, so crossing +-pi gives no derivative spike.
  if (angular && std::isfinite(error_delta)) {
    error_delta = std::atan2(std::sin(error_delta), std::cos(error_delta));
  }
  const auto derivative = error_delta / dt;
  if (!std::isfinite(error) || !std::isfinite(derivative)) {return false;}

  const auto integral = integrate ? info.integral_error + error * dt : info.integral_error;
  if (!std::isfinite(integral)) {return false;}

  info.integral_error = integral;
  info.previous_error = error;
  info.previous_derivative = derivative;
  return true;
}

}  // namespace rpp_control
