#pragma once
#include "../control_defs.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <marble_control_msgs/msg/control_state.hpp>
#include <marble_control_msgs/msg/control_status.hpp>
#include <marble_control_msgs/msg/pose_reference.hpp>
#include <marble_control_msgs/msg/twist_reference.hpp>
#include <marble_control_msgs/msg/wrench_reference.hpp>
#include <marble_control_msgs/srv/request_control.hpp>
#include <marble_control_msgs/srv/request_external_reference.hpp>
#include <marble_control_msgs/srv/select_signal.hpp>


namespace rpp_control {

  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using WrenchStamped = geometry_msgs::msg::WrenchStamped;
  using Int32MultiArray = std_msgs::msg::Int32MultiArray;
  using Float32MultiArray = std_msgs::msg::Float32MultiArray;
  using NavigationStatus = nav_msgs::msg::Odometry;
  using WrenchReference = marble_control_msgs::msg::WrenchReference;
  using TwistReference = marble_control_msgs::msg::TwistReference;
  using PoseReference = marble_control_msgs::msg::PoseReference;
  using RequestControl = marble_control_msgs::srv::RequestControl;
  using ControlStatus = marble_control_msgs::msg::ControlStatus;
  using ControlState = marble_control_msgs::msg::ControlState;
  using RequestExternalReference = marble_control_msgs::srv::RequestExternalReference;
  using SelectSignal = marble_control_msgs::srv::SelectSignal;


}  // namespace rpp_control