#pragma once

#include "motion_controller.hpp"
#include <rpp_plugin_types/rpp_control/MotionController2D.hpp>
#include <rpp_schema/rpp_common/EnablerOdometry2D.hpp>

namespace rpp_control {

    struct MotionController2DTraits
    {
        using ControllerComponent = rpp_control::MotionController2D;
        using AllocatorComponent = rpp_control::MotionControllerAllocator2D;
        using InputMessage = ControllerComponent::Odometry2D;
        using OutputMessage = ControllerComponent::Wrench2D;
        using Command = rpp_schema::rpp_common::Command;
        using Enabler = rpp_schema::rpp_common::Enabler2D;
        using EnablerOdometry = rpp_schema::rpp_common::EnablerOdometry2D;
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
    };


    class MotionController2D final : public MotionControllerT<MotionController2DTraits>
    {
        public:
            RPP_COMPONENTS(
                {"controller", "rpp_control::MotionController2D"},
                {"allocator", "rpp_control::MotionControllerAllocator2D"}
            )

            explicit MotionController2D(const rpp::ComponentContext& context)
                : MotionControllerT<MotionController2DTraits>(context)
            {
            }
    };
}