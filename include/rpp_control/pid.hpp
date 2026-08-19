#pragma once
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
            PID(const PIDParameters& params) : params_(params) {}
            virtual ~PID() = default;

            void set_parameters(const PIDParameters& params)
            {
                params_ = params;
            }

            FP_TYPE step(FP_TYPE ref, FP_TYPE state, FP_TYPE dt)
            {
                FP_TYPE error = ref - state;
                integral_error_ += error * dt;

                if (params_.integral_limit > 0.0)
                {
                    if (integral_error_ > params_.integral_limit)
                        integral_error_ = params_.integral_limit;
                    else if (integral_error_ < -params_.integral_limit)
                        integral_error_ = -params_.integral_limit;
                }

                if (params_.derivative_filter_coefficient > 0.0)
                {
                    FP_TYPE derivative = (error - previous_error_) / dt;
                    FP_TYPE filtered_derivative =
                        params_.derivative_filter_coefficient * previous_derivative_
                        + (1.0 - params_.derivative_filter_coefficient) * derivative;
                    previous_derivative_ = filtered_derivative;
                }
                else
                {
                    previous_derivative_ = (error - previous_error_) / dt;
                }

                FP_TYPE output = params_.kp * error
                    + params_.ki * integral_error_
                    + params_.kd * previous_derivative_;

                if (params_.output_limit > 0.0)
                {
                    if (output > params_.output_limit)
                        output = params_.output_limit;
                    else if (output < -params_.output_limit)
                        output = -params_.output_limit;
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

    };

} /// namespace rpp_control