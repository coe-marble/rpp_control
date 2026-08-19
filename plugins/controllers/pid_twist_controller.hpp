#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_control/TwistController2D.hpp>
#include <rpp_control/pid.hpp>
#include <memory>


using namespace rpp_control;

class PIDTwistController2D : public rpp_control::TwistController2D
{
    using ParameterDescription = rpp::params::ParameterDescription;
    RPP_PARAMETERS(
        ParameterDescription::create<PIDParameters>("pid_x"),
        ParameterDescription::create<PIDParameters>("pid_y"),
        ParameterDescription::create<PIDParameters>("pid_n")
    )

    std::unique_ptr<PID> pid_x_;
    std::unique_ptr<PID> pid_y_;
    std::unique_ptr<PID> pid_yaw_;

public:

    PIDTwistController2D() = default;
    virtual ~PIDTwistController2D() = default;

    void initialize(const rpp::ComponentContext& context) override
    {
        PIDParameters pid_x_params = context.get_parameter<PIDParameters>("pid_x");
        PIDParameters pid_y_params = context.get_parameter<PIDParameters>("pid_y");
        PIDParameters pid_yaw_params = context.get_parameter<PIDParameters>("pid_n");

        pid_x_ = std::make_unique<PID>(pid_x_params);
        pid_y_ = std::make_unique<PID>(pid_y_params);
        pid_yaw_ = std::make_unique<PID>(pid_yaw_params);
    }

    Wrench2D::Const step(Twist2D::Const ref_twist, Twist2D::Const state,
        Enabler2D::Const enabler, double dt) override
    {
        Wrench2D output;
        if (enabler.enableX())
        {
            output.force().x() = pid_x_->step(ref_twist.linear().x(), state.linear().x(), dt);
        }
        if (enabler.enableY())
        {
            output.force().y() = pid_y_->step(ref_twist.linear().y(), state.linear().y(), dt);
        }
        if (enabler.enableN())
        {
            output.torque() = pid_yaw_->step(ref_twist.angular(), state.angular(), dt);
        }
        return output;
    }

    void reset() override
    {
        // Implement reset logic if needed
        pid_x_->reset();
        pid_y_->reset();
        pid_yaw_->reset();
    }

};