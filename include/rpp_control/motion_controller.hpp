#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_control/MotionController2D.hpp>
#include <rpp_plugin_types/rpp_control/MotionControllerAllocator2D.hpp>

#include <rpp_schema/rpp_common/Command.hpp>
#include <rpp_schema/rpp_common/Odometry2D.hpp>
#include <rpp_schema/rpp_common/VectorPlanar.hpp>

#include "control_defs.hpp"
#include "utils.hpp"

namespace rpp_control {


    template <typename Traits>
    class MotionControllerT
    {
        public:
            using ControllerComponent = typename Traits::ControllerComponent;
            using AllocatorComponent = typename Traits::AllocatorComponent;
            using ControllerIO = typename Traits::ControllerIO;
            using Is3D = typename Traits::Is3D;
            using Enabler = typename Traits::Enabler;
            using EnablerOdometry = typename Traits::EnablerOdometry;
            using InputMessage = typename Traits::InputMessage;
            using OutputMessage = typename Traits::OutputMessage;

            using Dim = std::conditional_t<Is3D::value,
                std::integral_constant<size_t, 6>,
                std::integral_constant<size_t, 3>>;

            using PoseArray = std::array<FP_TYPE, Dim::value>;
            using TwistArray = std::array<FP_TYPE, Dim::value>;
            using WrenchArray = std::array<FP_TYPE, Dim::value>;

            explicit MotionControllerT(const rpp::ComponentContext& context)
                : context_(&context),
                  active_dofs_(Traits::default_active_dofs),
                  initialized_(false),
                  logger_(context.get_logger()),
                  clock_(context.get_clock()),
                  last_step_time_(clock_->now_seconds()),
                  latch_timeout_(0.5),
                  feedback_timeout_(1.0),
                  feedback_time_(last_step_time_),
                  current_wrench_selection_({}),
                  current_twist_selection_({}),
                  current_pose_ref_stamp_sec_({}),
                  current_twist_ref_stamp_sec_({}),
                  current_wrench_ref_stamp_sec_({}),
                  control_identity_token_()
            {
            }

            virtual ~MotionControllerT() = default;

            void initialize()
            {
                context_->initialize(); // initializes all subcomponents
                controller_ = context_->template get_component<ControllerComponent>("controller");
                allocator_ = context_->template get_component<AllocatorComponent>("allocator");

                auto num_commands = allocator_->outputSize();
                RPP_LOG_INFO(*logger_, "Allocator output size: %d", num_commands);
                state_live_.commands.resize(num_commands);



                if (!controller_)
                {
                    throw std::runtime_error("MotionController requires a controller component named 'controller'.");
                }

                if (!allocator_)
                {
                    throw std::runtime_error("MotionController requires an allocator component named 'allocator'.");
                }

                curr_state_msg_ = InputMessage{};
                ref_state_msg_ = InputMessage{};
                enabler_odom_msg_ = EnablerOdometry{};

                initialized_ = true;
            }

            bool step(double dt)
            {

                if (!initialized_)
                {
                    return false;
                }

                if (!(dt > 0.0) || !std::isfinite(dt))
                {
                    return false;
                }


                auto curr_time_sec = context_->get_clock()->now_seconds();
                latch_references(curr_time_sec);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    state_locked_ = state_live_; // make a deep copy of the live state to work with
                }

                if (!state_locked_.has_feedback)
                {
                    RPP_LOG_WARN(*logger_, "No feedback received. Stopping.");
                    return false;
                }

                if (!state_locked_.has_any_pose_ext
                    && !state_locked_.has_any_twist_ext
                    && !state_locked_.has_any_wrench_ext)
                {
                    RPP_LOG_WARN(*logger_, "No external references received. Stopping.");
                    return false;
                }

                Traits::make_input_message(
                    state_locked_.pose_ref, state_locked_.twist_ref, ref_state_msg_
                );
                Traits::make_input_message(
                    state_locked_.pose, state_locked_.twist, curr_state_msg_
                );

                Traits::make_enabler_message(
                    active_dofs_, true,
                    state_locked_.twist_selection, std::move(enabler_odom_msg_.pose())
                );
                Traits::make_enabler_message(
                    active_dofs_, true,
                    state_locked_.wrench_selection, std::move(enabler_odom_msg_.twist())
                );

                auto controller_command = controller_->step(
                    ref_state_msg_.as_const(),
                    curr_state_msg_.as_const(),
                    enabler_odom_msg_.as_const(), dt);

                // reuse the enabler_odom message to send to the allocator
                Traits::make_enabler_message(
                    active_dofs_, false,
                    state_locked_.twist_selection, enabler_odom_msg_.twist()
                );
                auto allocated_result = allocator_->allocate(
                    controller_command.shallow_copy(),
                    curr_state_msg_.as_const(),
                    enabler_odom_msg_.twist().as_const(),
                    dt
                );

                last_step_time_ = curr_time_sec;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    Traits::parse_output_message(
                        std::move(controller_command),
                        std::move(allocated_result),
                        state_live_.wrench_ref,
                        state_live_.wrench,
                        state_live_.commands
                    );
                }
                return true;
            }

            void start()
            {

            }


            void update_live_state_with_command()
            {
                state_live_.wrench = state_locked_.wrench_ref;
                state_live_.pwm = state_locked_.pwm;
            }

            void latch_references(double curr_time_sec)
            {
                state_live_.has_any_pose_ext = false;
                state_live_.has_any_twist_ext = false;
                state_live_.has_any_wrench_ext = false;

                state_live_.has_feedback = (feedback_time_ + feedback_timeout_) >= curr_time_sec;

                for (size_t i = 0; i < Dim::value; i++)
                {
                    bool has_pose_ext = current_pose_ref_stamp_sec_[i] > 0
                        && (current_pose_ref_stamp_sec_[i] + latch_timeout_) >= curr_time_sec;
                    bool has_twist_ext = current_twist_ref_stamp_sec_[i] > 0
                        && (current_twist_ref_stamp_sec_[i] + latch_timeout_) >= curr_time_sec;
                    bool has_wrench_ext = current_wrench_ref_stamp_sec_[i] > 0
                        &&(current_wrench_ref_stamp_sec_[i] + latch_timeout_) >= curr_time_sec;
                    if (!has_pose_ext)
                    {
                        state_live_.pose_ref[i] = 0.;
                    }
                    if (!has_twist_ext)
                    {
                        state_live_.twist_ref[i] = 0.;
                    }
                    if (!has_wrench_ext)
                    {
                        state_live_.wrench_ref[i] = 0.;
                    }

                    state_live_.has_pose_ext[i] = has_pose_ext;
                    state_live_.has_twist_ext[i] = has_twist_ext;
                    state_live_.has_wrench_ext[i] = has_wrench_ext;
                    state_live_.has_any_pose_ext |= has_pose_ext;
                    state_live_.has_any_twist_ext |= has_twist_ext;
                    state_live_.has_any_wrench_ext |= has_wrench_ext;
                }
            }

            void get_live_state(typename ControllerIO::State& state)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                state = state_live_;
            }

            void get_active_dofs(DOF& active_dofs)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                active_dofs = active_dofs_;
            }

            void set_feedback(const PoseArray& pose, const TwistArray& twist)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                state_live_.pose = pose;
                state_live_.twist = twist;
                state_live_.has_feedback = true;
                feedback_time_ = clock_->now_seconds();
            }

            void set_current_pose_ref(const PoseArray& pose_ref,
                const std::array<bool, Dim::value>& selection)
            {
                set_references(pose_ref, selection,
                    current_pose_ref_stamp_sec_, state_live_.pose_ref);
            }

            void set_current_twist_ref(const TwistArray& twist_ref,
                const std::array<bool, Dim::value>& selection)
            {
                set_references(twist_ref, selection,
                    current_twist_ref_stamp_sec_, state_live_.twist_ref);
            }

            void set_current_wrench_ref(const WrenchArray& wrench_ref,
                const std::array<bool, Dim::value>& selection)
            {
                set_references(wrench_ref, selection,
                    current_wrench_ref_stamp_sec_, state_live_.wrench_ref);
            }


            void set_references(const std::array<FP_TYPE, Dim::value>& ref_vec,
                const std::array<bool, Dim::value>& selection_vec,
                std::array<FP_TYPE, Dim::value>& current_ref_stamp_sec_,
                std::array<FP_TYPE, Dim::value>& io_ref_vec)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                auto curr_time_sec = clock_->now_seconds();
                for (size_t i = 0; i < Dim::value; i++)
                {
                    if (selection_vec[i]
                        && is_active_dof(get_dof_by_index<Dim::value>(i)))
                    {
                        io_ref_vec[i] = ref_vec[i];
                        current_ref_stamp_sec_[i] = curr_time_sec;
                    }
                }
            }

            void set_twist_selection(
                const std::array<SignalStatus, Dim::value>& twist_selection)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                for (size_t i = 0; i < Dim::value; i++)
                {
                    state_live_.twist_selection[i] = twist_selection[i];
                }
            }

            void set_wrench_selection(
                const std::array<SignalStatus, Dim::value>& wrench_selection)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                for (size_t i = 0; i < Dim::value; i++)
                {
                    state_live_.wrench_selection[i] = wrench_selection[i];
                }
            }

            bool is_active_dof(DOF dof) const
            {
                return (active_dofs_ & dof) != 0;
            }


            bool check_ref_identity_token(const FixedSizeString& identity_token,
                ReferenceType ref_type, std::array<bool, 6> mask)
            {
                // 0 is dontcare
                if (ref_type <= 0 || ref_type >= REF_END)
                    return false;
                for (int i = 0; i < 6; i++)
                {
                    if (mask[i])
                    {
                        if (!ref_identities[ref_type][i].is_set())
                            return false;
                        if (ref_identities[ref_type][i].token != identity_token)
                            return false;
                    }
                }
                return true;
            }

            bool check_control_identity_token(const FixedSizeString& identity_token)
            {
                return control_identity_token_.token == identity_token;
            }

            std::pair<bool, FixedSizeString> try_request_control_identity_token(const FixedSizeString& identity_name)
            {
                if (control_identity_token_.is_set())
                {
                    return {false, {}};
                }
                Identity::get_random_identity(
                    identity_name, "ctl", control_identity_token_);
                return {true, control_identity_token_.token};
            }

            std::pair<bool, FixedSizeString> try_request_ref_identity_token(
                const FixedSizeString& identity_name,
                std::array<ReferenceType, 6> mask)
            {
                for (size_t i = 0; i < 6; i++)
                {
                    auto ref_type = mask[i];
                    if (ref_type <= 0 || ref_type >= REF_END)
                        return {false, {}};
                    if (ref_identities[ref_type][i].is_set())
                        return {false, {}};
                    Identity::get_random_identity(identity_name,
                        "ref", ref_identities[ref_type][i]);
                }
                return {true, {}};
            }

        protected:
            const rpp::ComponentContext& context() const
            {
                return *context_;
            }

            std::mutex& mutex()
            {
                return mutex_;
            }

        private:

            void update_error(const typename ControllerIO::State& state, typename ControllerIO::Error& errors)
            {
                for (size_t i = 0; i < Dim::value; i++)
                {
                    errors.pose_error[i] = state.pose_ref[i] - state.pose[i];
                    errors.twist_error[i] = state.twist_ref[i] - state.twist[i];
                    errors.wrench_error[i] = state.wrench_ref[i] - state.wrench[i];
                }
            }

            void debug_output()
            {
                std::stringstream stream;
                this->debug_output(stream);
                RPP_LOG_DEBUG(*logger_, stream.str());
            }


            void debug_output(std::stringstream& stream)
            {
                std::lock_guard<std::mutex> lock(mutex_);

                std::stringstream twist_selection_list, wrench_selection_list;
                for (size_t i = 0; i < this->state_live_.twist_selection.size(); ++i)
                {
                    if (this->state_live_.twist_selection[i] == SIGNAL_DISABLED)
                        twist_selection_list << "DIS";
                    else if (this->state_live_.twist_selection[i] == SIGNAL_INT)
                        twist_selection_list << "INT";
                    else if (this->state_live_.twist_selection[i] == SIGNAL_EXT)
                        twist_selection_list << "EXT";

                    if (i != this->state_live_.twist_selection.size() - 1)
                        twist_selection_list << ",";
                }

                for (size_t i = 0; i < this->state_live_.wrench_selection.size(); ++i)
                {
                    if (this->state_live_.wrench_selection[i] == SIGNAL_DISABLED)
                        wrench_selection_list << "DIS";
                    else if (this->state_live_.wrench_selection[i] == SIGNAL_INT)
                        wrench_selection_list << "INT";
                    else if (this->state_live_.wrench_selection[i] == SIGNAL_EXT)
                        wrench_selection_list << "EXT";

                    if (i != this->state_live_.wrench_selection.size() - 1)
                        wrench_selection_list << ",";
                }

                stream << format("\n[MERGER] Nu selection: {%s}\n",
                            twist_selection_list.str().c_str());
                stream << format("[MERGER] Tau selection: {%s}\n",
                            wrench_selection_list.str().c_str());

                stream << format(
                    "[MERGER] Pose ext: {{ {%.2f} {%.2f} {%.2f} {%.2f} {%.2f} {%.2f} }}\n",
                            state_live_.pose_ref[0], state_live_.pose_ref[1], state_live_.pose_ref[2],
                            state_live_.pose_ref[3], state_live_.pose_ref[4], state_live_.pose_ref[5]);
                stream << format(
                    "[MERGER] Twist ext: {{ {%.2f} {%.2f} {%.2f} {%.2f} {%.2f} {%.2f} }}\n",
                            state_live_.twist_ref[0], state_live_.twist_ref[1], state_live_.twist_ref[2],
                            state_live_.twist_ref[3], state_live_.twist_ref[4], state_live_.twist_ref[5]);
                stream << format(
                    "[MERGER] Wrench ext: {{ {%.2f} {%.2f} {%.2f} {%.2f} {%.2f} {%.2f} }}\n",
                            state_live_.wrench_ref[0], state_live_.wrench_ref[1], state_live_.wrench_ref[2],
                            state_live_.wrench_ref[3], state_live_.wrench_ref[4], state_live_.wrench_ref[5]);
            }

            const rpp::ComponentContext* context_;
            DOF active_dofs_;
            bool initialized_;
            std::shared_ptr<rpp::RppLogger> logger_;
            std::shared_ptr<rpp::RppClock> clock_;
            double last_step_time_;
            double latch_timeout_;
            double feedback_timeout_;
            double feedback_time_;
            mutable std::mutex mutex_;
            typename ControllerIO::State state_live_;
            typename ControllerIO::State state_locked_;
            typename ControllerIO::Error errors_;

            std::shared_ptr<ControllerComponent> controller_;
            std::shared_ptr<AllocatorComponent> allocator_;

            std::array<SignalStatus, Dim::value> current_wrench_selection_;
            std::array<SignalStatus, Dim::value> current_twist_selection_;

            std::array<FP_TYPE, Dim::value> current_pose_ref_stamp_sec_;
            std::array<FP_TYPE, Dim::value> current_twist_ref_stamp_sec_;
            std::array<FP_TYPE, Dim::value> current_wrench_ref_stamp_sec_;

            Identity control_identity_token_;
            std::array<std::array<Identity, DOF_END_i>, REF_END> ref_identities = {{}};

            // pre-allocated messages to avoid dynamic allocation during step
            InputMessage ref_state_msg_;
            InputMessage curr_state_msg_;
            EnablerOdometry enabler_odom_msg_;
    };
}
