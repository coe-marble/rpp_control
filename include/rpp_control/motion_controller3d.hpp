#pragma once

#include "motion_controller.hpp"

#include <rpp_plugin_types/rpp_control/MotionController3D.hpp>
#include <rpp_plugin_types/rpp_control/MotionControllerAllocator3D.hpp>
#include <rpp_schema/rpp_common/EnablerOdometry3D.hpp>

namespace rpp_control {

    struct MotionController3DTraits
    {
        using ControllerComponent = rpp_control::MotionController3D;
        using AllocatorComponent = rpp_control::MotionControllerAllocator3D;
        using InputMessage = ControllerComponent::Odometry3D;
        using OutputMessage = ControllerComponent::Wrench3D;
        using Command = rpp_schema::rpp_common::Command;
        using Enabler = rpp_schema::rpp_common::Enabler3D;
        using EnablerOdometry = rpp_schema::rpp_common::EnablerOdometry3D;
        using ControllerIO = ControllerIOT<6>;
        using Is3D = std::true_type;
        static constexpr DOF default_active_dofs =
            static_cast<DOF>(DOF_X | DOF_Y | DOF_Z | DOF_K | DOF_M | DOF_N);

        void make_input_message(
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

        void parse_output_message(
            OutputMessage::Const message,
            Command::Const out_commands,
            std::array<FP_TYPE, 6>& wrench,
            std::vector<FP_TYPE>& commands)
        {
            wrench[0] = message.force().x();
            wrench[1] = message.force().y();
            wrench[2] = message.force().z();
            wrench[3] = message.torque().x();
            wrench[4] = message.torque().y();
            wrench[5] = message.torque().z();

            size_t min_sz = std::min(out_commands.data().size(), commands.size());
            for (size_t i = 0; i < min_sz; ++i)
            {
                commands[i] = out_commands.data()[i];
            }
        }

        void make_enabler_message(
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


    };

    class MotionController3D final : public MotionControllerT<MotionController3DTraits>
    {
        public:
            RPP_COMPONENTS(
                {"controller", "rpp_control::MotionController3D"},
                {"allocator", "rpp_control::MotionControllerAllocator3D"}
            )

            explicit MotionController3D(const rpp::ComponentContext& context)
                : MotionControllerT<MotionController3DTraits>(context)
            {
            }

    };
}
