#include <rpp_control/ros/motion_controller_ros.hpp>



namespace rpp_control {

MotionControllerRos::MotionControllerRos(const rclcpp::NodeOptions &options)
    : Node("motion_controller", options),
        controller_(std::monostate{})
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    RCLCPP_INFO(this->get_logger(), "MotionControllerRos node has been started.");

    std::string type = this->declare_parameter("controller_type", "2D");

    std::string component_path = this->declare_parameter("component_path", "");

    if (component_path.empty())
    {
        throw std::runtime_error(
            "Parameter 'component_path' must be specified.");
    }

    auto context = rpp::ComponentContextBuilder()
        .build_component_from_path(component_path);

    if (type == "2D")
    {
        controller_.emplace<MotionController2DImpl>(context);
    }
    else if (type == "3D")
    {
        controller_.emplace<MotionController3DImpl>(context);
    }
    else
    {
        throw std::runtime_error(
            "Invalid controller_type parameter. Must be '2D' or '3D'.");
    }

    bool expose_developer_topics = this->declare_parameter(
        "expose_developer_topics", false);


    wrench_ext_sub_ = create_subscription<WrenchReference>(
        "wrench_ext", 1,
        std::bind(&MotionControllerRos::on_external_wrench_, this, _1));

    twist_ext_sub_ = create_subscription<TwistReference>(
        "twist_ext", 1,
        std::bind(&MotionControllerRos::on_external_twist_, this, _1));

    pose_ext_sub_ = create_subscription<PoseReference>(
        "pose_ext", 1,
        std::bind(&MotionControllerRos::on_external_pose_, this, _1));

    request_control_svc_ = create_service<RequestControl>(
        "request_control",
        std::bind(&MotionControllerRos::request_control_, this, _1, _2));

    request_external_ref_svc_ = create_service<RequestExternalReference>(
        "request_external_reference",
        std::bind(&MotionControllerRos::request_external_ref_, this, _1, _2));

    select_signal_svc_ = create_service<SelectSignal>(
        "select_signal",
        std::bind(&MotionControllerRos::select_signal_, this, _1, _2));

    if (expose_developer_topics)
    {
        pose_ref_sub_dev_ = create_subscription<PoseStamped>(
            "pose_ref", 1,
            std::bind(&MotionControllerRos::on_external_pose_dev_, this, _1));

        twist_ref_sub_dev_ = create_subscription<TwistStamped>(
            "twist_ref", 1,
            std::bind(&MotionControllerRos::on_external_twist_dev_, this, _1));

        wrench_ref_sub_dev_ = create_subscription<WrenchStamped>(
            "wrench_ref", 1,
            std::bind(&MotionControllerRos::on_external_wrench_dev_, this, _1));
    }

    // SET DEFAULT SIGNAL STATE FOR TAU
    rcl_interfaces::msg::ParameterDescriptor desc_default_sig;
    desc_default_sig.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    desc_default_sig.read_only = true;
    std::string default_sig;
    default_sig = this->declare_parameter(
        "default_signal_tau", "DISABLED", desc_default_sig);

    auto vec = std::array<SignalStatus, DOF_END_i> { };
    if (default_sig == "INT")
        vec.fill(SIGNAL_INT);
    else if (default_sig == "EXT")
        vec.fill(SIGNAL_EXT);
    set_wrench_selection_(vec);

    // SET DEFAULT SIGNAL STATE FOR NU
    default_sig = this->declare_parameter(
        "default_signal_nu", "DISABLED", desc_default_sig);

    if (default_sig == "INT")
        vec.fill(SIGNAL_INT);
    else if (default_sig == "EXT")
        vec.fill(SIGNAL_EXT);
    set_twist_selection_(vec);
}


bool MotionControllerRos::check_control_identity_token(
    const std::string& identity_token)
{
    FixedSizeString token_array;
    std::copy_n(identity_token.data(), std::min(identity_token.size(),
        token_array.size() - 1), token_array.begin());
    return std::visit([&token_array](auto& controller) {
        if constexpr (std::is_same_v<std::decay_t<decltype(controller)>, std::monostate>)
            return false; // No controller is active
        else
            return controller.check_control_identity_token(token_array);
    }, controller_);
}

bool MotionControllerRos::check_ref_identity_token(
    const std::string& identity_token,
    ReferenceType ref_type,
    std::array<bool, 6> mask)
{
    FixedSizeString token_array{};
    std::copy_n(identity_token.data(), std::min(identity_token.size(),
        token_array.size() - 1), token_array.begin());
    return std::visit([&ref_type, &mask, &token_array](auto& controller) {
        if constexpr (is_monostate<decltype(controller)>())
            return false; // No controller is active
        else
            return controller.check_ref_identity_token(
                token_array, ref_type, mask);
    }, controller_);
}

void MotionControllerRos::set_twist_selection_(
    const std::array<SignalStatus, DOF_END_i>& twist_selection)
{
    dispatch_3d(twist_selection,
        [](auto& active_controller, const auto& selection) {
            active_controller.set_twist_selection(selection);
    });
}

void MotionControllerRos::set_wrench_selection_(
    const std::array<SignalStatus, DOF_END_i>& wrench_selection)
{
    dispatch_3d(wrench_selection,
        [](auto& active_controller, const auto& selection) {
            active_controller.set_wrench_selection(selection);
    });
}


void MotionControllerRos::on_external_pose_(
    PoseReference::SharedPtr msg)
{
    if (!check_ref_identity_token(msg->identity_token,
        ReferenceType::POSE_REF, msg->mask))
    {
        RCLCPP_WARN(this->get_logger(),
            "Invalid identity token for external pose reference.");
        return;
    }

    double euler_x, euler_y, euler_z;
    std::tie(euler_x, euler_y, euler_z) = quat2euler(
        msg->reference.orientation.x, msg->reference.orientation.y,
        msg->reference.orientation.z, msg->reference.orientation.w);

    std::array<FP_TYPE, 6> data_3d{
        static_cast<FP_TYPE>(msg->reference.position.x),
        static_cast<FP_TYPE>(msg->reference.position.y),
        static_cast<FP_TYPE>(msg->reference.position.z),
        static_cast<FP_TYPE>(euler_x),
        static_cast<FP_TYPE>(euler_y),
        static_cast<FP_TYPE>(euler_z)
    };
    const auto& mask_3d = msg->mask;

    dispatch_3d_with_data(data_3d, mask_3d,
        [](auto& controller, const auto& d, const auto& m) {
            controller.set_current_pose_ref(d, m);
    });
}


void MotionControllerRos::on_external_twist_(
    TwistReference::SharedPtr msg)
{
    if (!check_ref_identity_token(msg->identity_token,
        ReferenceType::TWIST_REF, msg->mask))
    {
        RCLCPP_WARN(this->get_logger(),
            "Invalid identity token for external twist reference.");
        return;
    }

    std::array<FP_TYPE, 6> data_3d{
        static_cast<FP_TYPE>(msg->reference.linear.x),
        static_cast<FP_TYPE>(msg->reference.linear.y),
        static_cast<FP_TYPE>(msg->reference.linear.z),
        static_cast<FP_TYPE>(msg->reference.angular.x),
        static_cast<FP_TYPE>(msg->reference.angular.y),
        static_cast<FP_TYPE>(msg->reference.angular.z)
    };
    const auto& mask_3d = msg->mask;

    dispatch_3d_with_data(data_3d, mask_3d,
        [](auto& controller, const auto& d, const auto& m) {
            controller.set_current_twist_ref(d, m);
    });
}

void MotionControllerRos::on_external_wrench_(
    WrenchReference::SharedPtr msg)
{
    if (!check_ref_identity_token(msg->identity_token,
        ReferenceType::WRENCH_REF, msg->mask))
    {
        RCLCPP_WARN(this->get_logger(),
            "Invalid identity token for external wrench reference.");
        return;
    }
    std::array<FP_TYPE, 6> data_3d{
        static_cast<FP_TYPE>(msg->reference.force.x),
        static_cast<FP_TYPE>(msg->reference.force.y),
        static_cast<FP_TYPE>(msg->reference.force.z),
        static_cast<FP_TYPE>(msg->reference.torque.x),
        static_cast<FP_TYPE>(msg->reference.torque.y),
        static_cast<FP_TYPE>(msg->reference.torque.z)
    };
    const auto& mask_3d = msg->mask;

    dispatch_3d_with_data(data_3d, mask_3d,
        [](auto& controller, const auto& d, const auto& m) {
            controller.set_current_wrench_ref(d, m);
    });
}



void MotionControllerRos::on_navigation_status_(
    NavigationStatus::SharedPtr msg)
{
    std::visit([&](auto& controller) {
        if constexpr (is_2d_controller<decltype(controller)>())
        {
            auto yaw = quat_yaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            std::array<FP_TYPE, 3> pose{
                static_cast<FP_TYPE>(msg->pose.pose.position.x),
                static_cast<FP_TYPE>(msg->pose.pose.position.y),
                static_cast<FP_TYPE>(yaw)
            };
            std::array<FP_TYPE, 3> twist{
                static_cast<FP_TYPE>(msg->twist.twist.linear.x),
                static_cast<FP_TYPE>(msg->twist.twist.linear.y),
                static_cast<FP_TYPE>(msg->twist.twist.angular.z)
            };
            controller.set_feedback(
                pose, twist
            );
        }
        else if constexpr (is_3d_controller<decltype(controller)>())
        {
            double euler_x, euler_y, euler_z;
            std::tie(euler_x, euler_y, euler_z) = quat2euler(
                msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            std::array<FP_TYPE, 6> pose{
                static_cast<FP_TYPE>(msg->pose.pose.position.x),
                static_cast<FP_TYPE>(msg->pose.pose.position.y),
                static_cast<FP_TYPE>(msg->pose.pose.position.z),
                static_cast<FP_TYPE>(euler_x),
                static_cast<FP_TYPE>(euler_y),
                static_cast<FP_TYPE>(euler_z)
            };
            std::array<FP_TYPE, 6> twist{
                static_cast<FP_TYPE>(msg->twist.twist.linear.x),
                static_cast<FP_TYPE>(msg->twist.twist.linear.y),
                static_cast<FP_TYPE>(msg->twist.twist.linear.z),
                static_cast<FP_TYPE>(msg->twist.twist.angular.x),
                static_cast<FP_TYPE>(msg->twist.twist.angular.y),
                static_cast<FP_TYPE>(msg->twist.twist.angular.z)
            };
            controller.set_feedback(
                pose, twist
            );
        }
    }, controller_);
}


void MotionControllerRos::on_external_pose_dev_(
    PoseStamped::SharedPtr msg)
{
    double euler_x, euler_y, euler_z;
    std::tie(euler_x, euler_y, euler_z) = quat2euler(
        msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w);

    std::array<FP_TYPE, 6> data_3d{
        static_cast<FP_TYPE>(msg->pose.position.x),
        static_cast<FP_TYPE>(msg->pose.position.y),
        static_cast<FP_TYPE>(msg->pose.position.z),
        static_cast<FP_TYPE>(euler_x),
        static_cast<FP_TYPE>(euler_y),
        static_cast<FP_TYPE>(euler_z)
    };

    const auto& mask_3d = std::array<bool, 6>{
        true, true, true, true, true, true
    };

    dispatch_3d_with_data(data_3d, mask_3d,
        [](auto& controller, const auto& d, const auto& m) {
            if constexpr (is_2d_controller<decltype(controller)>())
            {
                controller.set_twist_selection(
                    {SIGNAL_INT, SIGNAL_INT, SIGNAL_INT}
                );
                controller.set_wrench_selection(
                    {SIGNAL_INT, SIGNAL_INT, SIGNAL_INT}
                );
            }
            else if constexpr (is_3d_controller<decltype(controller)>())
            {
                controller.set_twist_selection(
                    {SIGNAL_INT, SIGNAL_INT, SIGNAL_INT,
                     SIGNAL_INT, SIGNAL_INT, SIGNAL_INT}
                );
                controller.set_wrench_selection(
                    {SIGNAL_INT, SIGNAL_INT, SIGNAL_INT,
                     SIGNAL_INT, SIGNAL_INT, SIGNAL_INT}
                );
            }
            controller.set_current_pose_ref(d, m);
    });
}


void MotionControllerRos::on_external_twist_dev_(
    TwistStamped::SharedPtr msg)
{
    std::array<FP_TYPE, 6> data_3d{
        static_cast<FP_TYPE>(msg->twist.linear.x),
        static_cast<FP_TYPE>(msg->twist.linear.y),
        static_cast<FP_TYPE>(msg->twist.linear.z),
        static_cast<FP_TYPE>(msg->twist.angular.x),
        static_cast<FP_TYPE>(msg->twist.angular.y),
        static_cast<FP_TYPE>(msg->twist.angular.z)
    };
    const auto& mask_3d = std::array<bool, 6>{
        true, true, true, true, true, true
    };

    dispatch_3d_with_data(data_3d, mask_3d,
        [](auto& controller, const auto& d, const auto& m) {
            if constexpr (is_2d_controller<decltype(controller)>())
            {
                controller.set_twist_selection(
                    {SIGNAL_EXT, SIGNAL_EXT, SIGNAL_EXT}
                );
                controller.set_wrench_selection(
                    {SIGNAL_INT, SIGNAL_INT, SIGNAL_INT}
                );
            }
            else if constexpr (is_3d_controller<decltype(controller)>())
            {
                controller.set_twist_selection(
                    {SIGNAL_EXT, SIGNAL_EXT, SIGNAL_EXT,
                     SIGNAL_EXT, SIGNAL_EXT, SIGNAL_EXT}
                );
                controller.set_wrench_selection(
                    {SIGNAL_INT, SIGNAL_INT, SIGNAL_INT,
                     SIGNAL_INT, SIGNAL_INT, SIGNAL_INT}
                );
            }
            controller.set_current_twist_ref(d, m);
    });
}

void MotionControllerRos::on_external_wrench_dev_(
    WrenchStamped::SharedPtr msg)
{
    std::array<FP_TYPE, 6> data_3d{
        static_cast<FP_TYPE>(msg->wrench.force.x),
        static_cast<FP_TYPE>(msg->wrench.force.y),
        static_cast<FP_TYPE>(msg->wrench.force.z),
        static_cast<FP_TYPE>(msg->wrench.torque.x),
        static_cast<FP_TYPE>(msg->wrench.torque.y),
        static_cast<FP_TYPE>(msg->wrench.torque.z)
    };
    const auto& mask_3d = std::array<bool, 6>{
        true, true, true, true, true, true
    };

    dispatch_3d_with_data(data_3d, mask_3d,
        [](auto& controller, const auto& d, const auto& m) {
            if constexpr (is_2d_controller<decltype(controller)>())
            {
                controller.set_wrench_selection(
                    {SIGNAL_EXT, SIGNAL_EXT, SIGNAL_EXT}
                );
            }
            else if constexpr (is_3d_controller<decltype(controller)>())
            {
                controller.set_wrench_selection(
                    {SIGNAL_EXT, SIGNAL_EXT, SIGNAL_EXT,
                     SIGNAL_EXT, SIGNAL_EXT, SIGNAL_EXT}
                );
            }
            controller.set_current_wrench_ref(d, m);
    });
}


void MotionControllerRos::select_signal_(
    SelectSignal::Request::SharedPtr request,
    SelectSignal::Response::SharedPtr /*response*/)
{
    if (check_control_identity_token(request->identity_token))
    {
        set_wrench_selection_(
            reinterpret_cast<const std::array<SignalStatus, 6>&>(
                request->wrench_selection)
        );
        set_twist_selection_(
            reinterpret_cast<const std::array<SignalStatus, 6>&>(
                request->twist_selection)
        );
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),
            "Invalid identity token for select_signal service.");
    }
}


void MotionControllerRos::request_external_ref_(
    RequestExternalReference::Request::SharedPtr request,
    RequestExternalReference::Response::SharedPtr response)
{

    auto identity_name = FixedSizeString(request->identity);
    bool success = false;
    FixedSizeString token_array;
    std::visit([&identity_name, &request, &token_array, &success](auto& controller) {
        if constexpr (!is_monostate<decltype(controller)>())
        {
            std::tie(success, token_array) = controller.try_request_ref_identity_token(
                identity_name, reinterpret_cast<std::array<ReferenceType, 6>&>(request->ref_modes));
        }
    }, controller_);
    response->success = success;
    if (success) {
        response->identity_token = std::string(token_array.view());
        response->message = "External reference set granted.";
    }
    else
    {
        response->identity_token = "";
        response->message = "External reference denied.";
    }
}

void MotionControllerRos::request_control_(
    RequestControl::Request::SharedPtr request,
    RequestControl::Response::SharedPtr response)
{
    auto identity_name = FixedSizeString(request->identity);
    bool success = false;
    FixedSizeString new_token;
    std::visit([&identity_name, &success, &new_token](auto& controller) {
        if constexpr (!is_monostate<decltype(controller)>())
        {
            std::tie(success, new_token) =
                controller.try_request_control_identity_token(identity_name);
        }
    }, controller_);

    response->success = success;
    if (success) {
        response->identity_token = std::string(new_token.view());
        response->message = "Control granted.";
    }
    else
    {
        response->identity_token = "";
        response->message = "Control denied.";
    }
}

}

