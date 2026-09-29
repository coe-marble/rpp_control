#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <rpp_cpp/plugin.hpp>

#include "control_defs.hpp"



namespace rpp_control {

    RPP_PARAM_STRUCT(PIDParameters,
        RPP_MEMBER(FP_TYPE, kp, 0.0),
        RPP_MEMBER(FP_TYPE, ki, 0.0),
        RPP_MEMBER(FP_TYPE, kd, 0.0),
        RPP_MEMBER(FP_TYPE, output_limit, 0.0),
        RPP_MEMBER(FP_TYPE, integral_limit, 0.0),
        RPP_MEMBER(FP_TYPE, derivative_filter_coefficient, 0.0)
    )


    class PID
    {
        PIDParameters params_;
        FP_TYPE integral_error_ = 0.0;
        FP_TYPE previous_error_ = 0.0;
        FP_TYPE previous_derivative_ = 0.0;
    public:
        explicit PID(const PIDParameters& params) : params_(params)
        {
            validate_parameters(params_);
        }

        virtual ~PID() = default;

        void set_parameters(const PIDParameters& params)
        {
            validate_parameters(params);
            params_ = params;
        }

        FP_TYPE step(FP_TYPE ref, FP_TYPE state, FP_TYPE dt)
        {
            if (!std::isfinite(ref) || !std::isfinite(state)
                || !std::isfinite(dt) || dt <= 0.0)
            {
                reset();
                return 0.0;
            }

            const FP_TYPE error = ref - state;
            if (!std::isfinite(error))
            {
                reset();
                return 0.0;
            }

            integral_error_ += error * dt;
            if (!std::isfinite(integral_error_))
            {
                reset();
                return 0.0;
            }

            if (params_.integral_limit > 0.0)
            {
                integral_error_ = std::clamp(integral_error_,
                    -params_.integral_limit, params_.integral_limit);
            }

            const FP_TYPE derivative = (error - previous_error_) / dt;
            if (!std::isfinite(derivative))
            {
                reset();
                return 0.0;
            }
            if (params_.derivative_filter_coefficient > 0.0)
            {
                previous_derivative_ =
                    params_.derivative_filter_coefficient * previous_derivative_
                    + (1.0 - params_.derivative_filter_coefficient) * derivative;
            }
            else
            {
                previous_derivative_ = derivative;
            }

            FP_TYPE output = params_.kp * error
                + params_.ki * integral_error_
                + params_.kd * previous_derivative_;
            if (!std::isfinite(output))
            {
                reset();
                return 0.0;
            }

            if (params_.output_limit > 0.0)
            {
                output = std::clamp(output,
                    -params_.output_limit, params_.output_limit);
            }

            previous_error_ = error;
            return output;
        }

        void reset()
        {
            integral_error_ = 0.0;
            previous_error_ = 0.0;
            previous_derivative_ = 0.0;
        }

    private:
        static void validate_parameters(const PIDParameters& params)
        {
            if (!std::isfinite(params.kp) || !std::isfinite(params.ki)
                || !std::isfinite(params.kd)
                || !std::isfinite(params.output_limit)
                || !std::isfinite(params.integral_limit)
                || !std::isfinite(params.derivative_filter_coefficient))
            {
                throw std::invalid_argument("PID parameters must be finite.");
            }
            if (params.output_limit < 0.0 || params.integral_limit < 0.0)
            {
                throw std::invalid_argument(
                    "PID output and integral limits must be non-negative.");
            }
            if (params.derivative_filter_coefficient < 0.0
                || params.derivative_filter_coefficient >= 1.0)
            {
                throw std::invalid_argument(
                    "PID derivative filter coefficient must be in [0, 1).");
            }
        }

    };

} /// namespace rpp_control