#include <rpp_control/ros/motion_controller_ros.hpp>


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rpp_control::MotionControllerRos>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}