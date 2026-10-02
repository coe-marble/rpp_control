#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_control/control_defs.hpp>
#include <rpp_plugin_types/rpp_control/MotionController2D.hpp>
#include <rpp_plugin_types/rpp_control/PoseController2D.hpp>
#include <rpp_plugin_types/rpp_control/TwistController2D.hpp>
#include <rpp_schema/rpp_common/Pose2D.hpp>
#include <rpp_schema/rpp_common/Twist2D.hpp>
#include <rpp_schema/rpp_common/Wrench2D.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

class CascadeController2D : public rpp_control::MotionController2D
{
    using ParameterDescription = rpp::params::ParameterDescription;
    RPP_PARAMETERS(
        ParameterDescription::create<std::vector<double>>(
            "pose_scales", std::vector<double>{1.0, 1.0, 1.0}),
        ParameterDescription::create<std::vector<double>>(
            "twist_scales", std::vector<double>{1.0, 1.0, 1.0}),
        ParameterDescription::create<std::vector<double>>(
            "wrench_positive_scales", std::vector<double>{1.0, 1.0, 1.0}),
        ParameterDescription::create<std::vector<double>>(
            "wrench_negative_scales", std::vector<double>{1.0, 1.0, 1.0})
    )

    std::shared_ptr<rpp_control::PoseController2D> pose_controller_;
    std::shared_ptr<rpp_control::TwistController2D> twist_controller_;
    std::array<FP_TYPE, 3> pose_scales_{{1.0F, 1.0F, 1.0F}};
    std::array<FP_TYPE, 3> twist_scales_{{1.0F, 1.0F, 1.0F}};
    std::array<FP_TYPE, 3> wrench_positive_scales_{{1.0F, 1.0F, 1.0F}};
    std::array<FP_TYPE, 3> wrench_negative_scales_{{1.0F, 1.0F, 1.0F}};

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
        pose_scales_ = load_scales(context, "pose_scales");
        twist_scales_ = load_scales(context, "twist_scales");
        wrench_positive_scales_ = load_scales(
            context, "wrench_positive_scales");
        wrench_negative_scales_ = load_scales(
            context, "wrench_negative_scales");
    }

    Wrench2D::Const step(Odometry2D::Const ref_state, Odometry2D::Const state,
        EnablerOdometry2D::Const enabler, double dt) override
    {
        const auto normalized_ref_pose = normalize_pose(ref_state.pose());
        const auto normalized_pose = normalize_pose(state.pose());
        const auto pose_twist = pose_controller_->step(
            normalized_ref_pose, normalized_pose, enabler.pose(), dt);

        const auto normalized_ref_twist = normalize_twist(ref_state.twist());
        const auto normalized_twist = normalize_twist(state.twist());

        // Disabled pose loops use externally supplied physical velocity
        // references, normalized into the same PID coordinate system.
        rpp_schema::rpp_common::Twist2D selected_twist;
        auto selected_linear = selected_twist.linear();
        selected_linear.x() = enabler.pose().enableX()
            ? pose_twist.linear().x() : normalized_ref_twist.linear().x();
        selected_linear.y() = enabler.pose().enableY()
            ? pose_twist.linear().y() : normalized_ref_twist.linear().y();
        selected_twist.angular() = enabler.pose().enableN()
            ? pose_twist.angular() : normalized_ref_twist.angular();

        auto normalized_wrench = twist_controller_->step(
            std::move(selected_twist), normalized_twist,
            enabler.twist(), dt);
        return denormalize_wrench(std::move(normalized_wrench));
    }

    void reset() override
    {
        pose_controller_->reset();
        twist_controller_->reset();
    }

private:
    static std::array<FP_TYPE, 3> load_scales(
        const rpp::ComponentContext& context, const char* parameter_name)
    {
        const auto values = context.get_parameter<std::vector<double>>(
            parameter_name);
        if (values.size() != 3)
        {
            throw std::invalid_argument(
                std::string(parameter_name) + " must contain [x, y, yaw].");
        }

        std::array<FP_TYPE, 3> scales{};
        for (std::size_t index = 0; index < scales.size(); ++index)
        {
            scales[index] = static_cast<FP_TYPE>(values[index]);
            if (!std::isfinite(scales[index]) || scales[index] <= 0.0F)
            {
                throw std::invalid_argument(
                    std::string(parameter_name)
                    + " values must be finite and positive.");
            }
        }
        return scales;
    }

    rpp_schema::rpp_common::Pose2D normalize_pose(
        rpp_schema::rpp_common::Pose2D::Const pose) const
    {
        rpp_schema::rpp_common::Pose2D normalized;
        auto position = normalized.position();
        position.x() = pose.position().x() / pose_scales_[0];
        position.y() = pose.position().y() / pose_scales_[1];
        normalized.yaw() = pose.yaw() / pose_scales_[2];
        return normalized;
    }

    rpp_schema::rpp_common::Twist2D normalize_twist(
        rpp_schema::rpp_common::Twist2D::Const twist) const
    {
        rpp_schema::rpp_common::Twist2D normalized;
        auto linear = normalized.linear();
        linear.x() = twist.linear().x() / twist_scales_[0];
        linear.y() = twist.linear().y() / twist_scales_[1];
        normalized.angular() = twist.angular() / twist_scales_[2];
        return normalized;
    }

    static FP_TYPE denormalize_axis(
        const FP_TYPE normalized, const FP_TYPE positive_scale,
        const FP_TYPE negative_scale)
    {
        const FP_TYPE bounded = std::clamp(normalized, -1.0F, 1.0F);
        return bounded * (bounded >= 0.0F ? positive_scale : negative_scale);
    }

    rpp_schema::rpp_common::Wrench2D denormalize_wrench(
        rpp_schema::rpp_common::Wrench2D::Const wrench) const
    {
        rpp_schema::rpp_common::Wrench2D denormalized;
        auto force = denormalized.force();
        force.x() = denormalize_axis(
            wrench.force().x(), wrench_positive_scales_[0],
            wrench_negative_scales_[0]);
        force.y() = denormalize_axis(
            wrench.force().y(), wrench_positive_scales_[1],
            wrench_negative_scales_[1]);
        denormalized.torque() = denormalize_axis(
            wrench.torque(), wrench_positive_scales_[2],
            wrench_negative_scales_[2]);
        return denormalized;
    }
};
