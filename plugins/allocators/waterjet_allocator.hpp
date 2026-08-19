#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <algorithm>
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
            ParameterDescription::create<double>("max_safe_roll_deg", 30.0),
            ParameterDescription::create<double>("filter_time_constant", 0.4)
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
            max_safe_roll_rad_ = context.get_parameter<double>("max_safe_roll_deg") * M_PI / 180.0;
            filter_time_constant_ = context.get_parameter<double>("filter_time_constant");
            filter_thrust_alpha_ = 1.0 - std::exp(-1.0 / filter_time_constant_);
            filter_thrust_state_ = 0.0;
        }

        std::tuple<Command::Const, Wrench2D::Const> allocate(
            Wrench2D::Const ref_wrench,
            Odometry2D::Const state,
            Enabler2D::Const enabler,
            double dt) override
        {
            double tau_X = ref_wrench.force().x();
            double tau_Y = ref_wrench.force().y();

            double current_u = state.twist().linear().x();
            double current_r = state.twist().angular();

            double a_c = current_u * current_r;
            double F_centripetal = mass_ * a_c;

            double roll_moment_centripetal = F_centripetal * z_cg_;

            constexpr double g = 9.81;
            double max_restoring_moment = mass_ * g
                * metacentric_height_ * std::sin(max_safe_roll_rad_);

            double max_allowable_fy_jet = 0.0;
            if (max_restoring_moment > std::abs(roll_moment_centripetal)) {
                max_allowable_fy_jet = (max_restoring_moment - std::abs(roll_moment_centripetal)) / z_cg_;
            } else {
                max_allowable_fy_jet = 0.0;
            }

            double T = std::sqrt(tau_X * tau_X + tau_Y * tau_Y);
            double alpha = 0.0;

            if (T > 0.001) {
                alpha = std::atan2(tau_Y, tau_X);
            }

            if (T > max_thrust_) {
                T = max_thrust_;
            }

            double requested_fy_jet = T * std::sin(alpha);

            if (std::abs(requested_fy_jet) > max_allowable_fy_jet)
            {
                double safe_fy_jet = std::copysign(max_allowable_fy_jet, requested_fy_jet);
                double fx_jet = T * std::cos(alpha);

                alpha = std::atan2(safe_fy_jet, fx_jet);
                T = std::sqrt(fx_jet * fx_jet + safe_fy_jet * safe_fy_jet);
                if (T > max_thrust_)
                    T = max_thrust_;
            }

            alpha = std::clamp(alpha, -max_angle_, max_angle_);

            Command out_command;
            out_command.data().resize(2);
            out_command.data()[0] = T;
            out_command.data()[1] = alpha;

            Wrench2D allocated_wrench;
            allocated_wrench.force().x() = T * std::cos(alpha);
            allocated_wrench.force().y() = T * std::sin(alpha);
            allocated_wrench.torque() = (jet_x_ * T * std::sin(alpha)) - (jet_y_ * T * std::cos(alpha));
            return std::make_tuple(std::move(out_command), std::move(allocated_wrench));
        }

        uint32_t outputSize() override
        {
            return 2;
        }

    private:
        double jet_x_, jet_y_, max_thrust_, max_angle_;
        double mass_, z_cg_, metacentric_height_, max_safe_roll_rad_;
        double filter_thrust_alpha_, filter_time_constant_, filter_thrust_state_;
};
