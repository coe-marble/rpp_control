#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <rpp_cpp/parameter_description.hpp>
#include <rpp_cpp/rpp_param_struct_headers.hpp>

#include "controller_handover.hpp"


namespace rpp_control
{

RPP_PARAM_STRUCT(PIDParameters,
        RPP_MEMBER(double, kp, 0.0),
        RPP_MEMBER(double, ki, 0.0),
        RPP_MEMBER(double, kd, 0.0),
        RPP_MEMBER(double, output_limit, 0.0),
        RPP_MEMBER(double, integral_limit, 0.0),
        RPP_MEMBER(double, anti_windup_gain, 1.0),
        RPP_MEMBER(double, derivative_filter_coefficient, 0.0)
)


class PID
{
private:
  PIDParameters params_{};
  bool validate_step_ = true;
  ControllerAxisErrorInfo error_info_;
  bool step_valid_ = false;
  bool step_limited_ = false;

public:
  PID() = default;

  explicit PID(const PIDParameters & params, bool validate_step = true)
  : params_(params), validate_step_(validate_step)
  {
    validate_parameters(params_);
  }

  ~PID() = default;

  void set_parameters(const PIDParameters & params)
  {
    validate_parameters(params);
    params_ = params;
  }

  // tracking_error is the previous applied minus requested command, in PID output units.
  // angular wraps the reference error to [-pi, pi].
  double step(
    double ref, double state, double dt, bool angular = false, double tracking_error = 0.0)
  {
    step_valid_ = false;
    step_limited_ = false;
    if (validate_step_ && (!std::isfinite(ref) || !std::isfinite(state) ||
      !std::isfinite(dt) || dt <= 0.0))
    {
      reset();
      return 0.0;
    }

    auto candidate = error_info_;
    if (params_.ki != 0.0 && params_.anti_windup_gain != 0.0 && tracking_error != 0.0) {
      candidate.integral_error += (params_.anti_windup_gain * tracking_error) / params_.ki;
      if (params_.integral_limit > 0.0) {
        candidate.integral_error = std::clamp(
          candidate.integral_error, -params_.integral_limit, params_.integral_limit);
      }
    }
    const double tracked_integral = candidate.integral_error;
    if (!update_controller_axis_error_info(
                    candidate, ref, state, dt, params_.ki != 0.0, angular, validate_step_))
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

    double output = params_.kp * candidate.previous_error +
      params_.ki * candidate.integral_error +
      params_.kd * candidate.previous_derivative;
    if (validate_step_ && !std::isfinite(output)) {
      reset();
      return 0.0;
    }

    if (params_.output_limit > 0.0) {
      step_limited_ = std::abs(output) > params_.output_limit;
      const double integral_contribution = params_.ki *
        (candidate.integral_error - tracked_integral);
      const bool grows_upper_saturation =
        output > params_.output_limit &&
        integral_contribution > 0.0;
      const bool grows_lower_saturation =
        output < -params_.output_limit &&
        integral_contribution < 0.0;
      if (grows_upper_saturation || grows_lower_saturation) {
        candidate.integral_error = tracked_integral +
          (1.0 - params_.anti_windup_gain) *
          (candidate.integral_error - tracked_integral);
        output = params_.kp * candidate.previous_error +
          params_.ki * candidate.integral_error +
          params_.kd * candidate.previous_derivative;
      }
      output = std::clamp(output,
                    -params_.output_limit, params_.output_limit);
    }

    error_info_ = candidate;
    step_valid_ = true;
    return output;
  }

  void reset()
  {
    error_info_ = {};
    step_valid_ = false;
    step_limited_ = false;
  }

  bool last_step_valid() const noexcept {return step_valid_;}
  bool last_step_limited() const noexcept {return step_limited_;}

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
    step_valid_ = false;
    step_limited_ = false;
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
