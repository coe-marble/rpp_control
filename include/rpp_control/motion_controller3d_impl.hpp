#pragma once

#include "motion_controller.hpp"

#include <rpp_plugin_types/rpp_control/MotionController3D.hpp>
#include <rpp_plugin_types/rpp_control/MotionControllerAllocator3D.hpp>
#include <rpp_schema/rpp_control/EnablerOdometry3D.hpp>

namespace rpp_control {

    struct MotionController3DTraits
    {
        using ControllerComponent = rpp_control::MotionController3D;
        using AllocatorComponent = rpp_control::MotionControllerAllocator3D;
        using InputMessage = ControllerComponent::Odometry3D;
        using OutputMessage = ControllerComponent::Wrench3D;
        using Command = rpp_schema::rpp_common::Command;
        using Enabler = rpp_schema::rpp_control::Enabler3D;
        using EnablerOdometry = rpp_schema::rpp_control::EnablerOdometry3D;
        using ControllerIO = ControllerIOT<6>;
        using Is3D = std::true_type;
        static constexpr DOF default_active_dofs =
            static_cast<DOF>(DOF_X | DOF_Y | DOF_Z | DOF_K | DOF_M | DOF_N);

        static void make_input_message(
            const std::array<FP_TYPE, 6>& pose,
            const std::array<FP_TYPE, 6>& twist,
            InputMessage& message)
        {
            auto pose_msg = message.pose();
            auto position = pose_msg.position();
            auto orientation = pose_msg.orientation();
            position.x() = pose[0];
            position.y() = pose[1];
            position.z() = pose[2];

            auto [qx, qy, qz, qw] = euler2quat(pose[3], pose[4], pose[5]);
            orientation.x() = qx;
            orientation.y() = qy;
            orientation.z() = qz;
            orientation.w() = qw;

            auto twist_msg = message.twist();
            auto linear = twist_msg.linear();
            auto angular = twist_msg.angular();
            linear.x() = twist[0];
            linear.y() = twist[1];
            linear.z() = twist[2];
            angular.x() = twist[3];
            angular.y() = twist[4];
            angular.z() = twist[5];
        }

        static OutputMessage::Const merge_wrench_reference(
            OutputMessage::Const internal_wrench,
            const std::array<FP_TYPE, 6>& external_wrench,
            const std::array<SignalStatus, 6>& selection)
        {
            OutputMessage merged_wrench;
            auto force = merged_wrench.force();
            force.x() = select_wrench_value(
                internal_wrench.force().x(), external_wrench[0], selection[0]);
            force.y() = select_wrench_value(
                internal_wrench.force().y(), external_wrench[1], selection[1]);
            force.z() = select_wrench_value(
                internal_wrench.force().z(), external_wrench[2], selection[2]);
            auto torque = merged_wrench.torque();
            torque.x() = select_wrench_value(
                internal_wrench.torque().x(), external_wrench[3], selection[3]);
            torque.y() = select_wrench_value(
                internal_wrench.torque().y(), external_wrench[4], selection[4]);
            torque.z() = select_wrench_value(
                internal_wrench.torque().z(), external_wrench[5], selection[5]);
            return merged_wrench;
        }

        static std::array<FP_TYPE, 6> wrench_values(
            const OutputMessage::Const& wrench)
        {
            return {
                wrench.force().x(), wrench.force().y(), wrench.force().z(),
                wrench.torque().x(), wrench.torque().y(), wrench.torque().z()
            };
        }

        static OutputMessage::Const make_wrench(
            const std::array<FP_TYPE, 6>& values)
        {
            OutputMessage wrench;
            auto force = wrench.force();
            auto torque = wrench.torque();
            force.x() = values[0];
            force.y() = values[1];
            force.z() = values[2];
            torque.x() = values[3];
            torque.y() = values[4];
            torque.z() = values[5];
            return wrench;
        }

        static bool is_finite_wrench(const OutputMessage::Const& wrench)
        {
            return std::isfinite(wrench.force().x())
                && std::isfinite(wrench.force().y())
                && std::isfinite(wrench.force().z())
                && std::isfinite(wrench.torque().x())
                && std::isfinite(wrench.torque().y())
                && std::isfinite(wrench.torque().z());
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

        static void parse_output_message(
            OutputMessage::Const ref_message,
            const std::tuple<Command::Const, OutputMessage::Const>& out_commands,
            std::array<FP_TYPE, 6>& wrench_ref,
            std::array<FP_TYPE, 6>& wrench,
            std::vector<FP_TYPE>& commands)
        {
            wrench_ref[0] = ref_message.force().x();
            wrench_ref[1] = ref_message.force().y();
            wrench_ref[2] = ref_message.force().z();
            wrench_ref[3] = ref_message.torque().x();
            wrench_ref[4] = ref_message.torque().y();
            wrench_ref[5] = ref_message.torque().z();
            wrench[0] = std::get<1>(out_commands).force().x();
            wrench[1] = std::get<1>(out_commands).force().y();
            wrench[2] = std::get<1>(out_commands).force().z();
            wrench[3] = std::get<1>(out_commands).torque().x();
            wrench[4] = std::get<1>(out_commands).torque().y();
            wrench[5] = std::get<1>(out_commands).torque().z();

            std::fill(commands.begin(), commands.end(), 0.0);
            size_t min_sz = std::min(
                std::get<0>(out_commands).data().size(), commands.size());
            for (size_t i = 0; i < min_sz; ++i)
            {
                commands[i] = std::get<0>(out_commands).data()[i];
            }
        }

        static void make_enabler_message(
            DOF active_dofs,
            bool use_selection,
            const std::array<SignalStatus, 6>& selection,
            Enabler&& enabler
        )
        {
            auto is_enabled = [&](DOF dof, size_t index) {
                return (active_dofs & dof) != 0
                && (!use_selection || selection[index] == SIGNAL_INT);

            };
            enabler.enableX() = is_enabled(DOF_X, 0);
            enabler.enableY() = is_enabled(DOF_Y, 1);
            enabler.enableZ() = is_enabled(DOF_Z, 2);
            enabler.enableK() = is_enabled(DOF_K, 3);
            enabler.enableM() = is_enabled(DOF_M, 4);
            enabler.enableN() = is_enabled(DOF_N, 5);
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

    class MotionController3DImpl final : public MotionControllerT<MotionController3DTraits>
    {
        public:
            RPP_COMPONENTS(
                {"controller", "rpp_control::MotionController3D"},
                {"allocator", "rpp_control::MotionControllerAllocator3D"}
            )

            explicit MotionController3DImpl(const rpp::ComponentContext& context)
                : MotionControllerT<MotionController3DTraits>(context)
            {
            }

    };
}
