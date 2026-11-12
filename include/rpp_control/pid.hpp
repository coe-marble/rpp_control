#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <rpp_cpp/plugin.hpp>

#include "controller_handover.hpp"
#include "control_defs.hpp"


namespace rpp_control
{

RPP_PARAM_STRUCT(PIDParameters,
        RPP_MEMBER(FP_TYPE, kp, 0.0),
        RPP_MEMBER(FP_TYPE, ki, 0.0),
        RPP_MEMBER(FP_TYPE, kd, 0.0),
        RPP_MEMBER(FP_TYPE, output_limit, 0.0),
        RPP_MEMBER(FP_TYPE, integral_limit, 0.0),
        RPP_MEMBER(FP_TYPE, anti_windup_gain, 1.0),
        RPP_MEMBER(FP_TYPE, derivative_filter_coefficient, 0.0)
)


class PID
{
private:
  PIDParameters params_;
  ControllerAxisErrorInfo error_info_;

public:
  explicit PID(const PIDParameters & params)
  : params_(params)
  {
    validate_parameters(params_);
  }

  virtual ~PID() = default;

  void set_parameters(const PIDParameters & params)
  {
    validate_parameters(params);
    params_ = params;
  }

  // angular: ref and state are angles in rad and the error is wrapped to [-pi, pi].
  FP_TYPE step(FP_TYPE ref, FP_TYPE state, FP_TYPE dt, bool angular = false)
  {
    if (!std::isfinite(ref) || !std::isfinite(state) ||
      !std::isfinite(dt) || dt <= 0.0)
    {
      reset();
      return 0.0;
    }

    auto candidate = error_info_;
    if (!update_controller_axis_error_info(
                    candidate, ref, state, dt, params_.ki != 0.0, angular))
    {
      reset();
      return 0.0;
    }

    if (params_.integral_limit > 0.0) {
      candidate.integral_error = std::clamp(
                    candidate.integral_error, -params_.integral_limit,
                    params_.integral_limit);
    }
    if (params_.derivative_filter_coefficient > 0.0) {
      candidate.previous_derivative =
        params_.derivative_filter_coefficient * error_info_.previous_derivative +
        (1.0 - params_.derivative_filter_coefficient) *
        candidate.previous_derivative;
    }

    FP_TYPE output = params_.kp * candidate.previous_error +
      params_.ki * candidate.integral_error +
      params_.kd * candidate.previous_derivative;
    if (!std::isfinite(output)) {
      reset();
      return 0.0;
    }

    if (params_.output_limit > 0.0) {
      const FP_TYPE integral_contribution = params_.ki *
        (candidate.integral_error - error_info_.integral_error);
      const bool grows_upper_saturation =
        output > params_.output_limit &&
        integral_contribution > 0.0;
      const bool grows_lower_saturation =
        output < -params_.output_limit &&
        integral_contribution < 0.0;
      if (grows_upper_saturation || grows_lower_saturation) {
        candidate.integral_error = error_info_.integral_error +
          (1.0 - params_.anti_windup_gain) *
          (candidate.integral_error - error_info_.integral_error);
        output = params_.kp * candidate.previous_error +
          params_.ki * candidate.integral_error +
          params_.kd * candidate.previous_derivative;
      }
      output = std::clamp(output,
                    -params_.output_limit, params_.output_limit);
    }

    error_info_ = candidate;
    return output;
  }

  void reset()
  {
    error_info_ = {};
  }

  [[nodiscard]] ControllerAxisErrorInfo error_info() const
  {
    return error_info_;
  }

  void restore_error_info(const ControllerAxisErrorInfo & state)
  {
    if (!is_valid_controller_axis_error_info(state)) {
      throw std::invalid_argument("PID handover state must be finite.");
    }
    error_info_ = state;
  }

private:
  static void validate_parameters(const PIDParameters & params)
  {
    if (!std::isfinite(params.kp) || !std::isfinite(params.ki) ||
      !std::isfinite(params.kd) ||
      !std::isfinite(params.output_limit) ||
      !std::isfinite(params.integral_limit) ||
      !std::isfinite(params.anti_windup_gain) ||
      !std::isfinite(params.derivative_filter_coefficient))
    {
      throw std::invalid_argument("PID parameters must be finite.");
    }
    if (params.output_limit < 0.0 || params.integral_limit < 0.0 ||
      params.anti_windup_gain < 0.0 ||
      params.anti_windup_gain > 1.0)
    {
      throw std::invalid_argument(
                    "PID limits must be non-negative and anti_windup_gain must be in [0, 1].");
    }
    if (params.derivative_filter_coefficient < 0.0 ||
      params.derivative_filter_coefficient >= 1.0)
    {
      throw std::invalid_argument(
                    "PID derivative filter coefficient must be in [0, 1).");
    }
  }

};

} /// namespace rpp_control
