#pragma once

#include <type_traits>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace rpp_control {

// Build-time choice for cmd_pos, cmd_vel and cmd_tau. Stamped messages match
// current Nav2 velocity commands. Rebuild the controller and command publishers
// together after changing this constant.
inline constexpr bool kUseStampedCommandTopics = true;

template<bool Stamped>
struct CommandTopicTypes {
    template<typename Message>
    static const builtin_interfaces::msg::Time* stamp(const Message& message)
    {
        if constexpr (Stamped) return &message.header.stamp;
        else return nullptr;
    }

    using Pose = std::conditional_t<Stamped,
        geometry_msgs::msg::PoseStamped, geometry_msgs::msg::Pose>;
    using Twist = std::conditional_t<Stamped,
        geometry_msgs::msg::TwistStamped, geometry_msgs::msg::Twist>;
    using Wrench = std::conditional_t<Stamped,
        geometry_msgs::msg::WrenchStamped, geometry_msgs::msg::Wrench>;

    static const geometry_msgs::msg::Pose& pose(const Pose& message)
    {
        if constexpr (Stamped) return message.pose;
        else return message;
    }
    static const geometry_msgs::msg::Twist& twist(const Twist& message)
    {
        if constexpr (Stamped) return message.twist;
        else return message;
    }
    static const geometry_msgs::msg::Wrench& wrench(const Wrench& message)
    {
        if constexpr (Stamped) return message.wrench;
        else return message;
    }
};

using CommandTopics = CommandTopicTypes<kUseStampedCommandTopics>;
using CmdPose = CommandTopics::Pose;
using CmdTwist = CommandTopics::Twist;
using CmdWrench = CommandTopics::Wrench;

}  // namespace rpp_control
