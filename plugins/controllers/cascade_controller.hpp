#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_control/MotionController2D.hpp>
#include <rpp_plugin_types/rpp_control/PoseController2D.hpp>
#include <rpp_plugin_types/rpp_control/TwistController2D.hpp>
#include <rpp_schema/rpp_common/Pose2D.hpp>
#include <rpp_schema/rpp_common/Twist2D.hpp>
#include <rpp_schema/rpp_common/Wrench2D.hpp>

#include <memory>
#include <utility>

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
        const auto pose_twist = pose_controller_->step(
            ref_state.pose(), state.pose(), enabler.pose(), dt);

        // Disabled pose loops use externally supplied physical velocity
        // references.
        rpp_schema::rpp_common::Twist2D selected_twist;
        auto selected_linear = selected_twist.linear();
        selected_linear.x() = enabler.pose().enableX()
            ? pose_twist.linear().x() : ref_state.twist().linear().x();
        selected_linear.y() = enabler.pose().enableY()
            ? pose_twist.linear().y() : ref_state.twist().linear().y();
        selected_twist.angular() = enabler.pose().enableN()
            ? pose_twist.angular() : ref_state.twist().angular();

        return twist_controller_->step(
            std::move(selected_twist), state.twist(),
            enabler.twist(), dt);
    }

    void reset() override
    {
        pose_controller_->reset();
        twist_controller_->reset();
    }
};
