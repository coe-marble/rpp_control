#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_control/MotionController2D.hpp>
#include <rpp_plugin_types/rpp_control/PoseController2D.hpp>
#include <rpp_plugin_types/rpp_control/TwistController2D.hpp>
#include <rpp_schema/rpp_common/Twist2D.hpp>
#include <rpp_schema/rpp_common/Wrench2D.hpp>
#include <memory>

class CascadeController2D : public rpp_control::MotionController2D
{
    std::shared_ptr<rpp_control::PoseController2D> pose_controller_;
    std::shared_ptr<rpp_control::TwistController2D> twist_controller_;

public:
    RPP_COMPONENTS(
        { "pose_controller", "rpp_control::PoseController2D" },
        { "twist_controller", "rpp_control::TwistController2D" }
    )

    CascadeController2D() = default;
    virtual ~CascadeController2D() = default;


    void initialize(const rpp::ComponentContext& context) override
    {
        pose_controller_ = context
            .get_component<rpp_control::PoseController2D>("pose_controller");
        twist_controller_ = context
            .get_component<rpp_control::TwistController2D>("twist_controller");
    }


    Wrench2D::Const step(Odometry2D::Const ref_state, Odometry2D::Const state,
        EnablerOdometry2D::Const enabler, double dt) override
    {
        auto ref_twist_pose = pose_controller_->step(ref_state.pose(), state.pose(), enabler.pose(), dt);
        auto ref_tau = twist_controller_->step(std::move(ref_twist_pose), state.twist(), enabler.twist(), dt);
        return ref_tau;
    }

    void reset() override
    {
        pose_controller_->reset();
        twist_controller_->reset();
    }

};