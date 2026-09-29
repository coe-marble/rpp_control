#include <rpp_control/ros/motion_controller_ros.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <tuple>
#include <type_traits>

namespace {

bool to_signal_status(
    const std::uint8_t selection,
    rpp_control::SignalStatus& status)
{
    using SelectSignal = marble_control_msgs::srv::SelectSignal;
    switch (selection)
    {
    case SelectSignal::Request::DONTCARE:
        status = rpp_control::SIGNAL_DONTCARE;
        return true;
    case SelectSignal::Request::DISABLEAXIS:
        status = rpp_control::SIGNAL_DISABLED;
        return true;
    case SelectSignal::Request::EXTERNAL:
        status = rpp_control::SIGNAL_EXT;
        return true;
    case SelectSignal::Request::INTERNAL:
        status = rpp_control::SIGNAL_INT;
        return true;
    default:
        return false;
    }
}

bool to_reference_type(
    const std::uint8_t mode,
    rpp_control::ReferenceType& reference_type)
{
    using RequestExternalReference =
        marble_control_msgs::srv::RequestExternalReference;
    switch (mode)
    {
    case RequestExternalReference::Request::IGNORE:
        reference_type = rpp_control::IGNORE;
        return true;
    case RequestExternalReference::Request::TAU_REF:
        reference_type = rpp_control::WRENCH_REF;
        return true;
    case RequestExternalReference::Request::NU_REF:
        reference_type = rpp_control::TWIST_REF;
        return true;
    case RequestExternalReference::Request::ETA_REF:
        reference_type = rpp_control::POSE_REF;
        return true;
    default:
        return false;
    }
}

}  // namespace

namespace rpp_control {

MotionControllerRos::MotionControllerRos(const rclcpp::NodeOptions &options)
    : Node("motion_controller", options),
        controller_(std::monostate{})
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    RCLCPP_INFO(this->get_logger(), "MotionControllerRos node has been started.");

    std::string type = this->declare_parameter("controller_type", "2D");

    const std::string script_description_path = this->declare_parameter(
        "script_description_path", "");
    const std::string component_path = this->declare_parameter(
        "component_path", "");

    if (!script_description_path.empty() && !component_path.empty())
    {
        throw std::runtime_error(
            "Specify either 'script_description_path' or 'component_path', not both.");
    }
    if (script_description_path.empty() && component_path.empty())
    {
        throw std::runtime_error(
            "Parameter 'script_description_path' or 'component_path' must be specified.");
    }

    rpp::ComponentContextBuilder context_builder;
    if (!script_description_path.empty())
    {
        context_ = std::make_unique<rpp::ComponentContext>(
            context_builder.build_script_from_description_path(
                script_description_path));
    }
    else
    {
        context_ = std::make_unique<rpp::ComponentContext>(
            context_builder.build_component_from_path(component_path));
    }

    if (type == "2D")
    {
        controller_.emplace<MotionController2DImpl>(*context_);
    }
    else if (type == "3D")
    {
        controller_.emplace<MotionController3DImpl>(*context_);
    }
    else
    {
        throw std::runtime_error(
            "Invalid controller_type parameter. Must be '2D' or '3D'.");
    }

    std::visit([](auto& controller) {
        using Controller = std::decay_t<decltype(controller)>;
        if constexpr (!std::is_same_v<Controller, std::monostate>)
        {
            controller.initialize();
        }
    }, controller_);

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

    navigation_status_sub_ = create_subscription<NavigationStatus>(
        "navigation_status", 1,
        std::bind(&MotionControllerRos::on_navigation_status_, this, _1));

    status_pub_ = create_publisher<ControlStatus>("control_status", 1);
    state_pub_ = create_publisher<ControlState>("control_state", 1);
    pwm_out_pub_ = create_publisher<Float32MultiArray>("pwm_out", 1);

    request_control_svc_ = create_service<RequestControl>(
        "request_control",
        std::bind(&MotionControllerRos::request_control_, this, _1, _2));

    request_external_ref_svc_ = create_service<RequestExternalReference>(
        "request_external_reference",
        std::bind(&MotionControllerRos::request_external_ref_, this, _1, _2));

    release_control_svc_ = create_service<ReleaseControl>(
        "release_control",
        std::bind(&MotionControllerRos::release_control_, this, _1, _2));

    release_external_ref_svc_ = create_service<ReleaseExternalReference>(
        "release_external_reference",
        std::bind(&MotionControllerRos::release_external_ref_, this, _1, _2));

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

    rcl_interfaces::msg::ParameterDescriptor desc_default_sig;
    desc_default_sig.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    desc_default_sig.read_only = true;
    const auto default_selection = [this, &desc_default_sig](
        const std::string& parameter_name) {
        const auto value = this->declare_parameter(
            parameter_name, "DISABLED", desc_default_sig);
        std::array<SignalStatus, DOF_END_i> selection{};
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
            throw std::runtime_error(
                "Default signal parameters must be INT, EXT, or DISABLED.");
        }
        return selection;
    };
    set_wrench_selection_(default_selection("default_signal_tau"));
    set_twist_selection_(default_selection("default_signal_nu"));

    const double control_period = this->declare_parameter<double>(
        "control_period", 0.02);
    if (!std::isfinite(control_period) || control_period <= 0.0)
    {
        throw std::runtime_error("Parameter 'control_period' must be positive and finite.");
    }
    max_control_dt_ = this->declare_parameter<double>(
        "max_control_dt", control_period * 2.0);
    if (!std::isfinite(max_control_dt_) || max_control_dt_ < control_period)
    {
        throw std::runtime_error(
            "Parameter 'max_control_dt' must be finite and at least control_period.");
    }
    last_control_tick_ = std::chrono::steady_clock::now();
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(control_period)),
        std::bind(&MotionControllerRos::on_control_timer_, this));
}


void MotionControllerRos::on_control_timer_()
{
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(
        now - last_control_tick_).count();
    last_control_tick_ = now;

    bool control_active = false;
    if (std::isfinite(dt) && dt > 0.0 && dt <= max_control_dt_)
    {
        control_active = std::visit([dt](auto& controller) {
            using Controller = std::decay_t<decltype(controller)>;
            if constexpr (std::is_same_v<Controller, std::monostate>)
            {
                return false;
            }
            else
            {
                return controller.step(dt);
            }
        }, controller_);
    }
    else
    {
        std::visit([](auto& controller) {
            using Controller = std::decay_t<decltype(controller)>;
            if constexpr (!std::is_same_v<Controller, std::monostate>)
            {
                controller.stop();
            }
        }, controller_);
    }
    publish_control_result_(control_active, dt);
}

void MotionControllerRos::publish_control_result_(
    const bool control_active, const double dt)
{
    control_state_message_.header.stamp = this->now();
    control_state_message_.dt = std::isfinite(dt) && dt > 0.0 ? dt : 0.0;
    control_state_message_.eta.fill(0.0);
    control_state_message_.eta_ref.fill(0.0);
    control_state_message_.eta_error.fill(0.0);
    control_state_message_.eta_error_body.fill(0.0);
    control_state_message_.eta_is_integrating.fill(false);
    control_state_message_.nu.fill(0.0);
    control_state_message_.nu_ref.fill(0.0);
    control_state_message_.nu_error.fill(0.0);
    control_state_message_.nu_is_integrating.fill(false);
    control_state_message_.tau.fill(0.0);
    control_state_message_.tau_ref.fill(0.0);

    const auto copy_spatial = [](auto& destination, const auto& source) {
        destination.fill(0.0);
        if constexpr (std::tuple_size_v<std::decay_t<decltype(source)>> == 3)
        {
            destination[0] = source[0];
            destination[1] = source[1];
            destination[5] = source[2];
        }
        else
        {
            std::copy(source.begin(), source.end(), destination.begin());
        }
    };
    const auto copy_spatial_bool = [](auto& destination, const auto& source) {
        destination.fill(false);
        if constexpr (std::tuple_size_v<std::decay_t<decltype(source)>> == 3)
        {
            destination[0] = source[0];
            destination[1] = source[1];
            destination[5] = source[2];
        }
        else
        {
            std::copy(source.begin(), source.end(), destination.begin());
        }
    };

    std::visit([&](auto& controller) {
        using Controller = std::decay_t<decltype(controller)>;
        if constexpr (!std::is_same_v<Controller, std::monostate>)
        {
            typename Controller::ControllerIO::State state;
            controller.get_live_state(state);

            auto pose_error = state.pose_ref;
            auto twist_error = state.twist_ref;
            auto wrench_error = state.wrench_ref;
            for (size_t i = 0; i < pose_error.size(); ++i)
            {
                pose_error[i] -= state.pose[i];
                twist_error[i] -= state.twist[i];
                wrench_error[i] -= state.wrench[i];
            }

            copy_spatial(control_state_message_.eta, state.pose);
            copy_spatial(control_state_message_.eta_ref, state.pose_ref);
            copy_spatial(control_state_message_.eta_error, pose_error);
            copy_spatial(control_state_message_.nu, state.twist);
            copy_spatial(control_state_message_.nu_ref, state.twist_ref);
            copy_spatial(control_state_message_.nu_error, twist_error);
            copy_spatial(control_state_message_.tau, state.wrench);
            copy_spatial(control_state_message_.tau_ref, state.wrench_ref);
            copy_spatial_bool(control_state_message_.eta_is_integrating,
                state.has_pose_ext);

            std::array<bool, std::tuple_size_v<decltype(state.wrench_selection)>>
                wrench_internal{};
            for (size_t i = 0; i < wrench_internal.size(); ++i)
            {
                wrench_internal[i] = state.wrench_selection[i] == SIGNAL_INT;
            }
            copy_spatial_bool(control_state_message_.nu_is_integrating,
                wrench_internal);

            if (pwm_output_message_.data.size() != state.commands.size())
            {
                pwm_output_message_.data.resize(state.commands.size());
            }
            if (control_active)
            {
                std::copy(state.commands.begin(), state.commands.end(),
                    pwm_output_message_.data.begin());
            }
            else
            {
                std::fill(pwm_output_message_.data.begin(),
                    pwm_output_message_.data.end(), 0.0F);
            }
        }
    }, controller_);

    control_state_message_.pwm_out = pwm_output_message_;
    control_status_message_.header.stamp = control_state_message_.header.stamp;
    control_status_message_.name = "motion_controller";
    control_status_message_.status = control_active ? "ACTIVE" : "SAFE_STOP";
    state_pub_->publish(control_state_message_);
    pwm_out_pub_->publish(pwm_output_message_);
    status_pub_->publish(control_status_message_);
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
                controller.set_wrench_selection_internal_unless_external();
            }
            else if constexpr (is_3d_controller<decltype(controller)>())
            {
                controller.set_twist_selection(
                    {SIGNAL_INT, SIGNAL_INT, SIGNAL_INT,
                     SIGNAL_INT, SIGNAL_INT, SIGNAL_INT}
                );
                controller.set_wrench_selection_internal_unless_external();
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
                controller.set_wrench_selection_internal_unless_external();
            }
            else if constexpr (is_3d_controller<decltype(controller)>())
            {
                controller.set_twist_selection(
                    {SIGNAL_EXT, SIGNAL_EXT, SIGNAL_EXT,
                     SIGNAL_EXT, SIGNAL_EXT, SIGNAL_EXT}
                );
                controller.set_wrench_selection_internal_unless_external();
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
        std::array<SignalStatus, DOF_END_i> wrench_selection{};
        std::array<SignalStatus, DOF_END_i> twist_selection{};
        for (size_t i = 0; i < DOF_END_i; ++i)
        {
            if (!to_signal_status(request->wrench_selection[i],
                    wrench_selection[i])
                || !to_signal_status(request->twist_selection[i],
                    twist_selection[i]))
            {
                RCLCPP_WARN(this->get_logger(),
                    "Invalid signal selection value.");
                return;
            }
        }
        set_wrench_selection_(wrench_selection);
        set_twist_selection_(twist_selection);
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
    std::array<ReferenceType, DOF_END_i> ref_modes{};
    for (size_t i = 0; i < DOF_END_i; ++i)
    {
        if (!to_reference_type(request->ref_modes[i], ref_modes[i]))
        {
            response->success = false;
            response->identity_token = "";
            response->message = "Invalid external reference mode.";
            return;
        }
    }
    bool success = false;
    FixedSizeString token_array;
    std::visit([&identity_name, &ref_modes, &token_array, &success](auto& controller) {
        if constexpr (!is_monostate<decltype(controller)>())
        {
            std::tie(success, token_array) = controller.try_request_ref_identity_token(
                identity_name, ref_modes);
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

void MotionControllerRos::release_control_(
    ReleaseControl::Request::SharedPtr request,
    ReleaseControl::Response::SharedPtr response)
{
    const auto identity_token = FixedSizeString(request->identity_token);
    bool success = false;
    std::visit([&identity_token, &success](auto& controller) {
        if constexpr (!is_monostate<decltype(controller)>())
        {
            success = controller.release_control_identity_token(identity_token);
        }
    }, controller_);
    response->success = success;
    response->message = success
        ? "Control authority released."
        : "Control authority release denied.";
}

void MotionControllerRos::release_external_ref_(
    ReleaseExternalReference::Request::SharedPtr request,
    ReleaseExternalReference::Response::SharedPtr response)
{
    const auto identity_token = FixedSizeString(request->identity_token);
    bool success = false;
    std::visit([&identity_token, &success](auto& controller) {
        if constexpr (!is_monostate<decltype(controller)>())
        {
            success = controller.release_ref_identity_token(identity_token);
        }
    }, controller_);
    response->success = success;
    response->message = success
        ? "External reference authority released."
        : "External reference authority release denied.";
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

