#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include <rpp_plugin_types/rpp_control/MotionControllerAllocator2D.hpp>
#include <rpp_cpp/plugin.hpp>

class WaterjetAllocator : public rpp_control::MotionControllerAllocator2D
{
public:
    WaterjetAllocator() = default;
    virtual ~WaterjetAllocator() = default;

    using ParameterDescription = rpp::params::ParameterDescription;
    RPP_PARAMETERS(
        ParameterDescription::create<double>("jet_x", -1.5),
        ParameterDescription::create<double>("jet_y", 0.0),
        ParameterDescription::create<double>("max_thrust", 6000.0),
        ParameterDescription::create<double>("max_angle", 0.61),
        ParameterDescription::create<double>("mass", 450.0),
        ParameterDescription::create<double>("z_cg", 0.4),
        ParameterDescription::create<double>("filter_time_constant", 0.4),
        ParameterDescription::create<double>("metacentric_height", 0.3),
        ParameterDescription::create<double>("max_safe_roll_deg", 30.0)
    )

    void initialize(const rpp::ComponentContext& context) override
    {
        jet_x_ = context.get_parameter<double>("jet_x");
        jet_y_ = context.get_parameter<double>("jet_y");
        max_thrust_ = context.get_parameter<double>("max_thrust");
        max_angle_ = context.get_parameter<double>("max_angle");
        mass_ = context.get_parameter<double>("mass");
        z_cg_ = context.get_parameter<double>("z_cg");
        metacentric_height_ = context.get_parameter<double>("metacentric_height");
        max_safe_roll_rad_ = context.get_parameter<double>("max_safe_roll_deg")
            * kPi / 180.0;
        filter_time_constant_ = context.get_parameter<double>("filter_time_constant");
        validate_configuration();
    }

    std::tuple<Command::Const, Wrench2D::Const> allocate(
        Wrench2D::Const ref_wrench,
        Odometry2D::Const state,
        Enabler2D::Const enabler,
        double dt) override
    {
        if (!std::isfinite(ref_wrench.force().x())
            || !std::isfinite(ref_wrench.force().y())
            || !std::isfinite(ref_wrench.torque())
            || !std::isfinite(state.twist().linear().x())
            || !std::isfinite(state.twist().angular())
            || !std::isfinite(dt) || dt <= 0.0)
        {
            return safe_output();
        }

        const double tau_x = enabler.enableX() ? ref_wrench.force().x() : 0.0;
        const double tau_y = enabler.enableY() ? ref_wrench.force().y() : 0.0;
        const double tau_n = enabler.enableN() ? ref_wrench.torque() : 0.0;
        const double requested_y_force = std::abs(jet_x_) > 1e-6
            ? tau_y + (tau_n + jet_y_ * tau_x) / jet_x_
            : tau_y;
        const double current_u = state.twist().linear().x();
        const double current_r = state.twist().angular();
        const double roll_moment_centripetal =
            mass_ * current_u * current_r * z_cg_;
        constexpr double gravity = 9.81;
        const double max_restoring_moment = mass_ * gravity
            * metacentric_height_ * std::sin(max_safe_roll_rad_);

        if (!std::isfinite(max_restoring_moment)
            || !std::isfinite(roll_moment_centripetal))
        {
            return safe_output();
        }

        const double max_allowable_fy_jet =
            max_restoring_moment > std::abs(roll_moment_centripetal)
            ? (max_restoring_moment - std::abs(roll_moment_centripetal)) / z_cg_
            : 0.0;

        double thrust = std::hypot(tau_x, requested_y_force);
        double angle = thrust > 0.001
            ? std::atan2(requested_y_force, tau_x)
            : 0.0;
        thrust = std::min(thrust, max_thrust_);

        const double requested_fy_jet = thrust * std::sin(angle);
        if (std::abs(requested_fy_jet) > max_allowable_fy_jet)
        {
            const double safe_fy_jet =
                std::copysign(max_allowable_fy_jet, requested_fy_jet);
            const double fx_jet = thrust * std::cos(angle);
            angle = std::atan2(safe_fy_jet, fx_jet);
            thrust = std::min(std::hypot(fx_jet, safe_fy_jet), max_thrust_);
        }

        angle = std::clamp(angle, -max_angle_, max_angle_);
        if (!std::isfinite(thrust) || !std::isfinite(angle))
        {
            return safe_output();
        }

        Command out_command;
        out_command.data().resize(2);
        out_command.data()[0] = thrust;
        out_command.data()[1] = angle;

        Wrench2D allocated_wrench;
        allocated_wrench.force().x() = thrust * std::cos(angle);
        allocated_wrench.force().y() = thrust * std::sin(angle);
        allocated_wrench.torque() =
            (jet_x_ * thrust * std::sin(angle))
            - (jet_y_ * thrust * std::cos(angle));
        return std::make_tuple(std::move(out_command), std::move(allocated_wrench));
    }

    uint32_t outputSize() override
    {
        return 2;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    std::tuple<Command::Const, Wrench2D::Const> safe_output() const
    {
        Command command;
        command.data().resize(2);
        command.data()[0] = 0.0;
        command.data()[1] = 0.0;

        Wrench2D wrench;
        wrench.force().x() = 0.0;
        wrench.force().y() = 0.0;
        wrench.torque() = 0.0;
        return std::make_tuple(std::move(command), std::move(wrench));
    }

    void validate_configuration() const
    {
        const auto is_finite = [](const double value) {
            return std::isfinite(value);
        };
        if (!is_finite(jet_x_) || !is_finite(jet_y_)
            || !is_finite(max_thrust_) || !is_finite(max_angle_)
            || !is_finite(mass_) || !is_finite(z_cg_)
            || !is_finite(metacentric_height_)
            || !is_finite(max_safe_roll_rad_)
            || !is_finite(filter_time_constant_))
        {
            throw std::invalid_argument(
                "Waterjet allocator parameters must be finite.");
        }
        if (max_thrust_ < 0.0 || max_angle_ < 0.0
            || max_angle_ > kPi / 2.0 || mass_ <= 0.0 || z_cg_ <= 0.0
            || metacentric_height_ <= 0.0 || max_safe_roll_rad_ < 0.0
            || max_safe_roll_rad_ >= kPi / 2.0
            || filter_time_constant_ <= 0.0)
        {
            throw std::invalid_argument(
                "Waterjet allocator parameters are outside safe limits.");
        }
    }

    double jet_x_, jet_y_, max_thrust_, max_angle_;
    double mass_, z_cg_, metacentric_height_, max_safe_roll_rad_;
    double filter_time_constant_;
};
