#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_control/PoseController2D.hpp>
#include <rpp_control/pid.hpp>
#include <exception>
#include <memory>
#include <optional>
#include <utility>


using namespace rpp_control;

class PIDPoseController2D : public rpp_control::PoseController2D
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
    PIDPoseController2D() = default;
    virtual ~PIDPoseController2D() = default;

    rpp::CallbackResult on_configure(const rpp::ComponentContext& context) override
    {
        PIDParameters pid_x_params = context.get_parameter<PIDParameters>("pid_x");
        PIDParameters pid_y_params = context.get_parameter<PIDParameters>("pid_y");
        PIDParameters pid_yaw_params = context.get_parameter<PIDParameters>("pid_n");

        pid_x_ = std::make_unique<PID>(pid_x_params);
        pid_y_ = std::make_unique<PID>(pid_y_params);
        pid_yaw_ = std::make_unique<PID>(pid_yaw_params, true);
        return {};
    }

    Twist2D::Const step(Pose2D::Const ref_state, Pose2D::Const state,
        Enabler2D::Const enabler, double dt) override
    {
        Twist2D twist;
        if (enabler.enableX())
        {
            twist.linear().x() = pid_x_->step(ref_state.position().x(), state.position().x(), dt);
        }
        if (enabler.enableY())
        {
            twist.linear().y() = pid_y_->step(ref_state.position().y(), state.position().y(), dt);
        }
        if (enabler.enableN())
        {
            twist.angular() = pid_yaw_->step(ref_state.yaw(), state.yaw(), dt);
        }
        return twist;
    }

    void reset() override
    {
        if (pid_x_) {pid_x_->reset();}
        if (pid_y_) {pid_y_->reset();}
        if (pid_yaw_) {pid_yaw_->reset();}
    }

    rpp::CallbackResult on_cleanup() override
    {
        pid_x_.reset();
        pid_y_.reset();
        pid_yaw_.reset();
        return {};
    }

    rpp::CallbackResult on_activate(
        const std::optional<HandoverState::Const> & state) override
    {
        if (!state) {
            reset();
            return {};
        }
        const auto axes = state->axisErrors();
        if (axes.size() != 3) {
            return rpp::CallbackResult::failure(
                "PIDPoseController2D requires three controller axis error entries");
        }
        try {
            pid_x_->restore_error_info(read_controller_axis_error_info(axes[0]));
            pid_y_->restore_error_info(read_controller_axis_error_info(axes[1]));
            pid_yaw_->restore_error_info(read_controller_axis_error_info(axes[2]));
        } catch (const std::exception & error) {
            return rpp::CallbackResult::failure(error.what());
        }
        return {};
    }

    rpp::DeactivationResult<HandoverState::Const> on_deactivate() override
    {
        HandoverState handover;
        auto axes = handover.axisErrors();
        axes.resize(3);
        write_controller_axis_error_info(axes[0], pid_x_->error_info());
        write_controller_axis_error_info(axes[1], pid_y_->error_info());
        write_controller_axis_error_info(axes[2], pid_yaw_->error_info());
        return {{}, HandoverState::Const(std::move(handover))};
    }

};
