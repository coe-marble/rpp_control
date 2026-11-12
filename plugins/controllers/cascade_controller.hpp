#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_control/MotionController2D.hpp>
#include <rpp_plugin_types/rpp_control/PoseController2D.hpp>
#include <rpp_plugin_types/rpp_control/TwistController2D.hpp>
#include <rpp_schema/rpp_common/Pose2D.hpp>
#include <rpp_schema/rpp_common/Twist2D.hpp>
#include <rpp_schema/rpp_common/Wrench2D.hpp>

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

class CascadeController2D : public rpp_control::MotionController2D
{
    const rpp::ComponentContext * context_ = nullptr;
    std::shared_ptr<rpp_control::PoseController2D> pose_controller_;
    std::shared_ptr<rpp_control::TwistController2D> twist_controller_;

public:
    RPP_COMPONENTS(
        { "pose_controller", "rpp_control::PoseController2D" },
        { "twist_controller", "rpp_control::TwistController2D" }
    )

    CascadeController2D() = default;
    virtual ~CascadeController2D() = default;

    rpp::CallbackResult on_configure(const rpp::ComponentContext& context) override
    {
        pose_controller_ = context
            .get_subcomponent_context("pose_controller")
            .get_instance<rpp_control::PoseController2D>();
        twist_controller_ = context
            .get_subcomponent_context("twist_controller")
            .get_instance<rpp_control::TwistController2D>();
        context_ = &context;
        return {};
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

    rpp::CallbackResult on_cleanup() override
    {
        pose_controller_.reset();
        twist_controller_.reset();
        context_ = nullptr;
        return {};
    }

    rpp::CallbackResult on_activate(
        const std::optional<HandoverState::Const> & state) override
    {
        if (!context_) {
            return rpp::CallbackResult::failure(
                "CascadeController2D must be configured before activation");
        }
        if (state) {
            auto valid_state = valid_parent_state(*state);
            if (!valid_state.success) {return valid_state;}
        }

        const auto & pose_context = context_->get_subcomponent_context("pose_controller");
        const auto & twist_context = context_->get_subcomponent_context("twist_controller");
        auto pose_result = pose_context.activate(
            state ? child_state(*state, 0) : rpp::HandoverState{});
        if (!pose_result.success) {return pose_result;}

        auto twist_result = twist_context.activate(
            state ? child_state(*state, 3) : rpp::HandoverState{});
        if (!twist_result.success) {
            (void)pose_context.deactivate();
            return twist_result;
        }
        return {};
    }

    rpp::DeactivationResult<HandoverState::Const> on_deactivate() override
    {
        if (!context_) {
            return {rpp::CallbackResult::failure(
              "CascadeController2D must be configured before deactivation"), std::nullopt};
        }
        const auto & pose_context = context_->get_subcomponent_context("pose_controller");
        const auto & twist_context = context_->get_subcomponent_context("twist_controller");

        auto twist_result = twist_context.deactivate();
        if (!twist_result.result.success) {return {std::move(twist_result.result), std::nullopt};}
        auto pose_result = pose_context.deactivate();
        if (!pose_result.result.success) {
            reactivate(twist_context, twist_result);
            return {std::move(pose_result.result), std::nullopt};
        }
        auto pose_state = valid_child_state(pose_result.state, "pose_controller");
        auto twist_state = valid_child_state(twist_result.state, "twist_controller");
        if (!pose_state.success || !twist_state.success) {
            reactivate(pose_context, pose_result);
            reactivate(twist_context, twist_result);
            return {!pose_state.success ? std::move(pose_state) : std::move(twist_state), std::nullopt};
        }

        HandoverState handover;
        auto axes = handover.axisErrors();
        axes.resize(6);
        copy_axis_errors(pose_result.state->get<HandoverState::Const>(), 0, axes, 0);
        copy_axis_errors(twist_result.state->get<HandoverState::Const>(), 0, axes, 3);
        return {{}, HandoverState::Const(std::move(handover))};
    }

private:
    template<typename AxisList>
    static void copy_axis_errors(
        const HandoverState::Const & source, const size_t source_offset,
        AxisList & destination, const size_t destination_offset)
    {
        const auto source_axes = source.axisErrors();
        for (size_t index = 0; index < 3; ++index) {
            destination[destination_offset + index].integralError() =
                source_axes[source_offset + index].integralError();
            destination[destination_offset + index].previousError() =
                source_axes[source_offset + index].previousError();
            destination[destination_offset + index].previousDerivative() =
                source_axes[source_offset + index].previousDerivative();
        }
    }

    static rpp::CallbackResult valid_parent_state(const HandoverState::Const & state)
    {
        const auto axes = state.axisErrors();
        if (axes.size() != 6) {
            return rpp::CallbackResult::failure(
              "CascadeController2D requires six controller axis error entries");
        }
        for (size_t index = 0; index < axes.size(); ++index) {
            const auto axis = axes[index];
            if (!std::isfinite(axis.integralError()) || !std::isfinite(axis.previousError())
                || !std::isfinite(axis.previousDerivative())) {
                return rpp::CallbackResult::failure(
                  "CascadeController2D handover axis errors must be finite");
            }
        }
        return {};
    }

    static rpp::CallbackResult valid_child_state(
        const std::optional<rpp::HandoverState> & state, const char * child_name)
    {
        if (!state || !state->holds<HandoverState::Const>()) {
            return rpp::CallbackResult::failure(
              std::string("CascadeController2D child did not return controller state: ") + child_name);
        }
        const auto axes = state->get<HandoverState::Const>().axisErrors();
        if (axes.size() != 3) {
            return rpp::CallbackResult::failure(
              std::string("CascadeController2D child returned invalid axis error count: ")
              + child_name);
        }
        for (size_t index = 0; index < axes.size(); ++index) {
            const auto axis = axes[index];
            if (!std::isfinite(axis.integralError()) || !std::isfinite(axis.previousError())
                || !std::isfinite(axis.previousDerivative())) {
                return rpp::CallbackResult::failure(
                  std::string("CascadeController2D child returned non-finite axis errors: ")
                  + child_name);
            }
        }
        return {};
    }

    static rpp::HandoverState child_state(
        const HandoverState::Const & parent, const size_t offset)
    {
        HandoverState handover;
        auto axes = handover.axisErrors();
        axes.resize(3);
        copy_axis_errors(parent, offset, axes, 0);
        return HandoverState::Const(std::move(handover));
    }

    static void reactivate(
        const rpp::SubcomponentContext & context,
        rpp::DeactivationResult<rpp::HandoverState> & deactivation)
    {
        (void)context.activate(
          deactivation.state ? std::move(*deactivation.state) : rpp::HandoverState{});
    }
};
