#pragma once

#include "motion_controller.hpp"

#include <array>
#include <string>
#include <vector>

#include <rpp_plugin_types/rpp_control/MotionController2D.hpp>
#include <rpp_schema/rpp_control/EnablerOdometry2D.hpp>

namespace rpp_control {

    struct MotionController2DTraits
    {
        using ControllerComponent = rpp_control::MotionController2D;
        using AllocatorComponent = rpp_control::MotionControllerAllocator2D;
        using InputMessage = ControllerComponent::Odometry2D;
        using OutputMessage = ControllerComponent::Wrench2D;
        using Command = rpp_schema::rpp_common::Command;
        using Enabler = rpp_schema::rpp_control::Enabler2D;
        using EnablerOdometry = rpp_schema::rpp_control::EnablerOdometry2D;
        using ControllerIO = ControllerIOT<3>;
        using Is3D = std::false_type;
        static constexpr DOF default_active_dofs = DOF_X | DOF_Y | DOF_N;

        static void make_input_message(
            const std::array<FP_TYPE, 3>& pose,
            const std::array<FP_TYPE, 3>& twist,
            InputMessage& message)
        {
            auto pose_msg = message.pose();
            auto position = pose_msg.position();
            position.x() = pose[0];
            position.y() = pose[1];
            pose_msg.yaw() = pose[2];

            auto twist_msg = message.twist();
            auto linear = twist_msg.linear();
            linear.x() = twist[0];
            linear.y() = twist[1];
            twist_msg.angular() = twist[2];
        }

        static OutputMessage::Const merge_wrench_reference(
            OutputMessage::Const internal_wrench,
            const std::array<FP_TYPE, 3>& external_wrench,
            const std::array<SignalStatus, 3>& selection)
        {
            OutputMessage merged_wrench;
            merged_wrench.force().x() = select_wrench_value(
                internal_wrench.force().x(), external_wrench[0], selection[0]);
            merged_wrench.force().y() = select_wrench_value(
                internal_wrench.force().y(), external_wrench[1], selection[1]);
            merged_wrench.torque() = select_wrench_value(
                internal_wrench.torque(), external_wrench[2], selection[2]);
            return merged_wrench;
        }

        static std::array<FP_TYPE, 3> wrench_values(
            const OutputMessage::Const& wrench)
        {
            return {
                static_cast<FP_TYPE>(wrench.force().x()),
                static_cast<FP_TYPE>(wrench.force().y()),
                static_cast<FP_TYPE>(wrench.torque())
            };
        }

        static OutputMessage::Const make_wrench(
            const std::array<FP_TYPE, 3>& values)
        {
            OutputMessage wrench;
            wrench.force().x() = values[0];
            wrench.force().y() = values[1];
            wrench.torque() = values[2];
            return wrench;
        }

        static bool is_finite_wrench(const OutputMessage::Const& wrench)
        {
            return std::isfinite(wrench.force().x())
                && std::isfinite(wrench.force().y())
                && std::isfinite(wrench.torque());
        }

        static bool is_finite_allocation(
            const std::tuple<Command::Const, OutputMessage::Const>& allocation)
        {
            if (!is_finite_wrench(std::get<1>(allocation)))
            {
                return false;
            }
            const auto commands = std::get<0>(allocation).data();
            for (size_t i = 0; i < commands.size(); ++i)
            {
                if (!std::isfinite(commands[i]))
                {
                    return false;
                }
            }
            return true;
        }

        static void parse_output_message(OutputMessage::Const ref_message,
            const std::tuple<Command::Const, OutputMessage::Const>& out_commands,
            std::array<FP_TYPE, 3>& wrench_ref, std::array<FP_TYPE, 3>& wrench,
            std::vector<FP_TYPE>& commands)
        {
            wrench_ref[0] = ref_message.force().x();
            wrench_ref[1] = ref_message.force().y();
            wrench_ref[2] = ref_message.torque();
            wrench[0] = std::get<1>(out_commands).force().x();
            wrench[1] = std::get<1>(out_commands).force().y();
            wrench[2] = std::get<1>(out_commands).torque();

            std::fill(commands.begin(), commands.end(), 0.0);
            size_t min_sz = std::min(std::get<0>(out_commands).data().size(), commands.size());
            for (size_t i = 0; i < min_sz; ++i)
            {
                commands[i] = std::get<0>(out_commands).data()[i];
            }
        }

        static void make_enabler_message(
            DOF active_dofs,
            bool use_selection,
            const std::array<SignalStatus, 3>& selection,
            Enabler&& enabler)
        {
            auto is_enabled = [&](DOF dof, size_t index) {
                return (active_dofs & dof) != 0
                    && (!use_selection || selection[index] == SIGNAL_INT);

            };
            enabler.enableX() = is_enabled(DOF_X, 0);
            enabler.enableY() = is_enabled(DOF_Y, 1);
            enabler.enableN() = is_enabled(DOF_N, 2);
        }

    private:
        static FP_TYPE select_wrench_value(const double internal_value,
            const FP_TYPE external_value, const SignalStatus selection)
        {
            if (selection == SIGNAL_INT)
            {
                return static_cast<FP_TYPE>(internal_value);
            }
            if (selection == SIGNAL_EXT)
            {
                return external_value;
            }
            return 0.0;
        }
    };


    class MotionController2DImpl final : public MotionControllerT<MotionController2DTraits>
    {
        public:
            using ParameterDescription = rpp::params::ParameterDescription;
            RPP_PARAMETERS(
                ParameterDescription::create<std::vector<std::string>>(
                    "active_dofs", std::vector<std::string>{"x", "y", "yaw"}),
                ParameterDescription::create<std::vector<double>>(
                    "max_wrench_rate", std::vector<double>{0.0, 0.0, 0.0}),
                ParameterDescription::create<std::vector<double>>(
                    "wrench_positive_scales",
                    std::vector<double>{1.0, 1.0, 1.0}),
                ParameterDescription::create<std::vector<double>>(
                    "wrench_negative_scales",
                    std::vector<double>{1.0, 1.0, 1.0}),
                ParameterDescription::create<std::string>(
                    "allocation_priority_dof", ""),
                ParameterDescription::create<std::string>(
                    "allocation_suppressed_dof", ""),
                ParameterDescription::create<double>(
                    "allocation_residual_activation_percent", 10.0),
                ParameterDescription::create<double>(
                    "allocation_residual_deactivation_percent", 2.5),
                ParameterDescription::create<std::string>(
                    "default_signal_tau", "DISABLED"),
                ParameterDescription::create<std::string>(
                    "default_signal_nu", "DISABLED")
            )

            RPP_COMPONENTS(
                {"controller", "rpp_control::MotionController2D"},
                {"allocator", "rpp_control::MotionControllerAllocator2D"}
            )

            explicit MotionController2DImpl(const rpp::ComponentContext& context)
                : MotionControllerT<MotionController2DTraits>(context)
            {
            }

        protected:
            void configure() override
            {
                const auto active_dof_names =
                    context().get_parameter<std::vector<std::string>>(
                        "active_dofs");
                DOF active_dofs = NO_DOF;
                for (const auto& dof_name : active_dof_names)
                {
                    const DOF dof = get_dof_by_name(dof_name);
                    if (dof == NO_DOF)
                    {
                        throw std::invalid_argument(
                            "Unknown active DOF: " + dof_name);
                    }
                    active_dofs = static_cast<DOF>(
                        static_cast<int>(active_dofs) | static_cast<int>(dof));
                }
                set_active_dofs(active_dofs);
                set_wrench_rate_limits(
                    context().get_parameter<std::vector<double>>(
                        "max_wrench_rate"));
                set_wrench_scales(
                    context().get_parameter<std::vector<double>>(
                        "wrench_positive_scales"),
                    context().get_parameter<std::vector<double>>(
                        "wrench_negative_scales"));
                set_wrench_selection(default_signal_selection(
                    context().get_parameter<std::string>("default_signal_tau")));
                set_twist_selection(default_signal_selection(
                    context().get_parameter<std::string>("default_signal_nu")));

                const auto priority_dof_name =
                    context().get_parameter<std::string>(
                        "allocation_priority_dof");
                const auto suppressed_dof_name =
                    context().get_parameter<std::string>(
                        "allocation_suppressed_dof");
                if (priority_dof_name.empty() != suppressed_dof_name.empty())
                {
                    throw std::invalid_argument(
                        "allocation_priority_dof and allocation_suppressed_dof must be configured together.");
                }
                if (priority_dof_name.empty())
                {
                    return;
                }

                AllocationSuppression suppression{};
                suppression.priority_dof = get_dof_by_name(priority_dof_name);
                suppression.suppressed_dof = get_dof_by_name(suppressed_dof_name);
                if (suppression.priority_dof == NO_DOF
                    || suppression.suppressed_dof == NO_DOF)
                {
                    throw std::invalid_argument(
                        "Allocation suppression DOFs must use supported DOF names.");
                }
                suppression.residual_activation_percent =
                    context().get_parameter<double>(
                        "allocation_residual_activation_percent");
                suppression.residual_deactivation_percent =
                    context().get_parameter<double>(
                        "allocation_residual_deactivation_percent");
                set_allocation_suppression(suppression);
            }

        private:
            static std::array<SignalStatus, 3> default_signal_selection(
                const std::string& value)
            {
                std::array<SignalStatus, 3> selection{};
                selection.fill(SIGNAL_DISABLED);
                if (value == "INT")
                {
                    selection.fill(SIGNAL_INT);
                }
                else if (value == "EXT")
                {
                    selection.fill(SIGNAL_EXT);
                }
                else if (value != "DISABLED")
                {
                    throw std::invalid_argument(
                        "Default signal parameters must be INT, EXT, or DISABLED.");
                }
                return selection;
            }
    };
}
