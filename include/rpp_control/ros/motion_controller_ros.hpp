#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rpp_cpp/plugin.hpp>
#include <rpp_cpp/context_builder.hpp>

#include "ros_defs.hpp"
#include "../utils.hpp"
#include "../motion_controller2d.hpp"
#include "../motion_controller3d.hpp"


namespace rpp_control {

    class MotionControllerRos : public rclcpp::Node {

    public:
        MotionControllerRos(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        ~MotionControllerRos() = default;

        std::variant<std::monostate,
            MotionController2D, MotionController3D> controller_;

        rclcpp::Subscription<WrenchReference>::SharedPtr wrench_ext_sub_;
        rclcpp::Subscription<TwistReference>::SharedPtr twist_ext_sub_;
        rclcpp::Subscription<PoseReference>::SharedPtr pose_ext_sub_;
        rclcpp::Subscription<NavigationStatus>::SharedPtr navigation_status_sub_;

        rclcpp::Subscription<PoseStamped>::SharedPtr pose_ref_sub_dev_;
        rclcpp::Subscription<TwistStamped>::SharedPtr twist_ref_sub_dev_;
        rclcpp::Subscription<WrenchStamped>::SharedPtr wrench_ref_sub_dev_;

        rclcpp::Publisher<ControlStatus>::SharedPtr status_pub_;
        rclcpp::Publisher<ControlState>::SharedPtr state_pub_;
        rclcpp::Publisher<Float32MultiArray>::SharedPtr pwm_out_pub_;

    private:
        void set_wrench_selection_(
            const std::array<SignalStatus, DOF_END_i>& wrench_selection);
        void set_twist_selection_(
            const std::array<SignalStatus, DOF_END_i>& twist_selection);

        // SUBSCRIPTION CALLBACKS
        void on_external_wrench_(WrenchReference::SharedPtr wrench_ref);
        void on_external_twist_(TwistReference::SharedPtr twist_ref);
        void on_external_pose_(PoseReference::SharedPtr eta_ref);
        void on_navigation_status_(NavigationStatus::SharedPtr nav_status);

        void on_external_wrench_dev_(WrenchStamped::SharedPtr wrench_ref);
        void on_external_twist_dev_(TwistStamped::SharedPtr twist_ref);
        void on_external_pose_dev_(PoseStamped::SharedPtr eta_ref);

        // SERVICES
        rclcpp::Service<RequestExternalReference>::SharedPtr
            request_external_ref_svc_;
        void request_external_ref_(
            RequestExternalReference::Request::SharedPtr request,
            RequestExternalReference::Response::SharedPtr response);

        rclcpp::Service<RequestControl>::SharedPtr
            request_control_svc_;
        void request_control_(
            RequestControl::Request::SharedPtr request,
            RequestControl::Response::SharedPtr response);
        rclcpp::Service<SelectSignal>::SharedPtr
            select_signal_svc_;
        void select_signal_(
            SelectSignal::Request::SharedPtr request,
            SelectSignal::Response::SharedPtr response);


        bool check_control_identity_token(const std::string& identity_token);
        bool check_ref_identity_token(const std::string& identity_token,
            ReferenceType ref_type, std::array<bool, 6> mask);


        template <typename T>
        static constexpr bool is_monostate()
        {
            return std::is_same_v<std::decay_t<T>, std::monostate>;
        }

        template <typename T>
        static constexpr bool is_3d_controller()
        {
            return std::is_same_v<std::decay_t<T>, MotionController3D>;
        }

        template <typename T>
        static constexpr bool is_2d_controller()
        {
            return std::is_same_v<std::decay_t<T>, MotionController2D>;
        }

        template <typename DataType, typename MaskType, typename Callback>
        void dispatch_3d_with_data(const std::array<DataType, 6>& data3d,
                        const std::array<MaskType, 6>& mask3d,
                        Callback&& func)
        {
            std::visit([&](auto& controller) {

                if constexpr (is_2d_controller<decltype(controller)>()) {
                    func(controller,
                        std::array<DataType, 3>{data3d[0], data3d[1], data3d[5]},
                        std::array<MaskType, 3>{mask3d[0], mask3d[1], mask3d[5]});
                }
                else if constexpr (is_3d_controller<decltype(controller)>()) {
                    func(controller, data3d, mask3d);
                }
            }, controller_);
        }

        template <typename MaskType, typename Callback>
        void dispatch_3d(const std::array<MaskType, 6>& mask3d,
                        Callback&& func)
        {
            std::visit([&](auto& controller) {
                if constexpr (is_2d_controller<decltype(controller)>()) {
                    func(controller,
                        std::array<MaskType, 3>{mask3d[0], mask3d[1], mask3d[5]});
                }
                else if constexpr (is_3d_controller<decltype(controller)>()) {
                    func(controller, mask3d);
                }
            }, controller_);
        }
    };

}