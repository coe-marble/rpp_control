#pragma once

#include <algorithm>
#include <array>
#include <chrono>
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

            struct AllocationSuppression
            {
                DOF priority_dof = NO_DOF;
                DOF suppressed_dof = NO_DOF;
                double priority_request_threshold = 0.0;
                double residual_activation_threshold = 0.0;
                double residual_deactivation_threshold = 0.0;
                bool active = false;
                WrenchArray previous_requested_wrench{};
            };

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
                RPP_LOG_DEBUG(*logger_,
                    "Initializing motion controller active_dofs=0x%x.",
                    static_cast<unsigned int>(active_dofs_));
                context_->initialize(); // initializes all subcomponents
                controller_ = context_->template get_component<ControllerComponent>("controller");
                allocator_ = context_->template get_component<AllocatorComponent>("allocator");

                if (!controller_)
                {
                    throw std::runtime_error("MotionController requires a controller component named 'controller'.");
                }

                if (!allocator_)
                {
                    throw std::runtime_error("MotionController requires an allocator component named 'allocator'.");
                }

                const auto num_commands = allocator_->outputSize();
                RPP_LOG_INFO(*logger_, "Allocator output size: %d", num_commands);
                state_live_.commands.resize(num_commands);

                curr_state_msg_ = InputMessage{};
                ref_state_msg_ = InputMessage{};
                enabler_odom_msg_ = EnablerOdometry{};

                initialized_ = true;
                RPP_LOG_DEBUG(*logger_, "Motion controller initialized.");
            }

            bool step(double dt)
            {
                std::lock_guard<std::mutex> step_lock(step_mutex_);
                if (!initialized_)
                {
                    clear_outputs();
                    return false;
                }

                if (!(dt > 0.0) || !std::isfinite(dt))
                {
                    return stop_with_warning("Invalid control-loop time step. Stopping.");
                }

                const auto curr_time_sec = clock_->now_seconds();
                if (!std::isfinite(curr_time_sec))
                {
                    return stop_with_warning("Invalid controller clock value. Stopping.");
                }
                latch_references(curr_time_sec);
                DOF control_dofs = NO_DOF;
                DOF allocation_dofs = NO_DOF;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    allocation_dofs = active_dofs_;
                    control_dofs = update_allocation_suppression_locked(
                        state_live_);
                    state_locked_ = state_live_;
                }

                if (!state_locked_.has_feedback)
                {
                    return stop_with_warning("No feedback received. Stopping.");
                }

                if (!state_locked_.has_any_pose_ext
                    && !state_locked_.has_any_twist_ext
                    && !state_locked_.has_any_wrench_ext)
                {
                    return stop_with_warning("No external references received. Stopping.");
                }

                if (!is_finite_array(state_locked_.pose)
                    || !is_finite_array(state_locked_.twist)
                    || !is_finite_array(state_locked_.pose_ref)
                    || !is_finite_array(state_locked_.twist_ref)
                    || !is_finite_array(state_locked_.wrench_ref))
                {
                    return stop_with_warning("Invalid control input. Stopping.");
                }

                auto twist_reference = state_locked_.twist_ref;
                for (size_t i = 0; i < Dim::value; ++i)
                {
                    if (state_locked_.twist_selection[i] != SIGNAL_EXT)
                    {
                        twist_reference[i] = 0.0;
                    }
                }

                Traits::make_input_message(
                    state_locked_.pose_ref, twist_reference, ref_state_msg_
                );
                Traits::make_input_message(
                    state_locked_.pose, state_locked_.twist, curr_state_msg_
                );

                Traits::make_enabler_message(
                    control_dofs, true,
                    state_locked_.twist_selection, std::move(enabler_odom_msg_.pose())
                );
                Traits::make_enabler_message(
                    control_dofs, true,
                    state_locked_.wrench_selection, std::move(enabler_odom_msg_.twist())
                );

                try
                {
                    auto controller_command = controller_->step(
                        ref_state_msg_.as_const(),
                        curr_state_msg_.as_const(),
                        enabler_odom_msg_.as_const(), dt);
                    auto requested_wrench = Traits::merge_wrench_reference(
                        std::move(controller_command), state_locked_.wrench_ref,
                        state_locked_.wrench_selection);
                    if (!Traits::is_finite_wrench(requested_wrench))
                    {
                        return stop_with_warning(
                            "Controller produced an invalid wrench. Stopping.");
                    }

                    Traits::make_enabler_message(
                        allocation_dofs, false,
                        state_locked_.wrench_selection, enabler_odom_msg_.twist()
                    );
                    auto allocated_result = allocator_->allocate(
                        requested_wrench.shallow_copy(),
                        curr_state_msg_.as_const(),
                        enabler_odom_msg_.twist().as_const(),
                        dt
                    );
                    if (!Traits::is_finite_allocation(allocated_result))
                    {
                        return stop_with_warning(
                            "Allocator produced an invalid command. Stopping.");
                    }

                    last_step_time_ = curr_time_sec;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        Traits::parse_output_message(
                            std::move(requested_wrench),
                            std::move(allocated_result),
                            state_live_.wrench_ref,
                            state_live_.wrench,
                            state_live_.commands
                        );
                        allocation_suppression_.previous_requested_wrench =
                            state_live_.wrench_ref;
                    }
                    RPP_LOG_DEBUG_THROTTLE(*logger_,
                        std::chrono::milliseconds(500),
                        "Control step completed time=%.3f dt=%.3f "
                        "control_dofs=0x%x allocation_dofs=0x%x "
                        "feedback=%d pose_ref=%d twist_ref=%d wrench_ref=%d.",
                        curr_time_sec, dt,
                        static_cast<unsigned int>(control_dofs),
                        static_cast<unsigned int>(allocation_dofs),
                        state_locked_.has_feedback,
                        state_locked_.has_any_pose_ext,
                        state_locked_.has_any_twist_ext,
                        state_locked_.has_any_wrench_ext);
                }
                catch (const std::exception& exception)
                {
                    RPP_LOG_ERROR_ONCE(*logger_,
                        "Control update failed: %s. Stopping.", exception.what());
                    clear_outputs();
                    reset_controller();
                    return false;
                }
                catch (...)
                {
                    RPP_LOG_ERROR_ONCE(*logger_,
                        "Control update failed with an unknown error. Stopping.");
                    clear_outputs();
                    reset_controller();
                    return false;
                }
                return true;
            }

            void start()
            {
                RPP_LOG_DEBUG(*logger_, "Motion controller started.");
            }

            void stop()
            {
                std::lock_guard<std::mutex> step_lock(step_mutex_);
                clear_outputs();
                reset_controller();
                RPP_LOG_DEBUG(*logger_, "Motion controller stopped.");
            }

            void latch_references(double curr_time_sec)
            {
                std::lock_guard<std::mutex> lock(mutex_);
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

            void set_active_dofs(const DOF active_dofs)
            {
                const auto requested = static_cast<int>(active_dofs);
                const auto supported = static_cast<int>(
                    Traits::default_active_dofs);
                if (requested == static_cast<int>(NO_DOF)
                    || (requested & ~supported) != 0)
                {
                    throw std::invalid_argument(
                        "Active DOFs must be a non-empty subset of the controller DOFs.");
                }

                std::lock_guard<std::mutex> lock(mutex_);
                active_dofs_ = active_dofs;
                RPP_LOG_DEBUG(*logger_, "Active DOFs set to 0x%x.",
                    static_cast<unsigned int>(active_dofs_));
                if ((active_dofs_ & allocation_suppression_.priority_dof)
                        == NO_DOF
                    || (active_dofs_ & allocation_suppression_.suppressed_dof)
                        == NO_DOF)
                {
                    allocation_suppression_.active = false;
                }
                for (size_t i = 0; i < Dim::value; ++i)
                {
                    if ((active_dofs_ & get_dof_by_index<Dim::value>(i)) != 0)
                    {
                        continue;
                    }
                    state_live_.pose_ref[i] = 0.0;
                    state_live_.twist_ref[i] = 0.0;
                    state_live_.wrench_ref[i] = 0.0;
                    state_live_.has_pose_ext[i] = false;
                    state_live_.has_twist_ext[i] = false;
                    state_live_.has_wrench_ext[i] = false;
                    current_pose_ref_stamp_sec_[i] = 0.0;
                    current_twist_ref_stamp_sec_[i] = 0.0;
                    current_wrench_ref_stamp_sec_[i] = 0.0;
                }
            }

            void set_allocation_suppression(
                const DOF priority_dof, const DOF suppressed_dof,
                const double priority_request_threshold,
                const double residual_activation_threshold,
                const double residual_deactivation_threshold)
            {
                if (!is_single_supported_dof(priority_dof)
                    || !is_single_supported_dof(suppressed_dof)
                    || priority_dof == suppressed_dof)
                {
                    throw std::invalid_argument(
                        "Allocation priority and suppressed DOFs must be distinct supported axes.");
                }
                if (!std::isfinite(priority_request_threshold)
                    || !std::isfinite(residual_activation_threshold)
                    || !std::isfinite(residual_deactivation_threshold)
                    || priority_request_threshold < 0.0
                    || residual_deactivation_threshold < 0.0
                    || residual_activation_threshold <= residual_deactivation_threshold)
                {
                    throw std::invalid_argument(
                        "Allocation suppression thresholds must be finite, non-negative, and have activation greater than deactivation.");
                }

                std::lock_guard<std::mutex> lock(mutex_);
                if ((active_dofs_ & priority_dof) == NO_DOF
                    || (active_dofs_ & suppressed_dof) == NO_DOF)
                {
                    throw std::invalid_argument(
                        "Allocation suppression DOFs must be active for this controller.");
                }
                allocation_suppression_.priority_dof = priority_dof;
                allocation_suppression_.suppressed_dof = suppressed_dof;
                allocation_suppression_.priority_request_threshold =
                    priority_request_threshold;
                allocation_suppression_.residual_activation_threshold =
                    residual_activation_threshold;
                allocation_suppression_.residual_deactivation_threshold =
                    residual_deactivation_threshold;
                allocation_suppression_.active = false;
                RPP_LOG_DEBUG(*logger_,
                    "Allocation suppression configured priority_dof=0x%x "
                    "suppressed_dof=0x%x request_threshold=%.3f "
                    "activation_threshold=%.3f deactivation_threshold=%.3f.",
                    static_cast<unsigned int>(priority_dof),
                    static_cast<unsigned int>(suppressed_dof),
                    priority_request_threshold,
                    residual_activation_threshold,
                    residual_deactivation_threshold);
            }

            void set_feedback(const PoseArray& pose, const TwistArray& twist)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!is_finite_array(pose) || !is_finite_array(twist))
                {
                    state_live_.has_feedback = false;
                    feedback_time_ = 0.0;
                    RPP_LOG_DEBUG_ONCE(*logger_,
                        "Ignoring non-finite feedback.");
                    return;
                }
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
                std::array<double, Dim::value>& current_ref_stamp_sec_,
                std::array<FP_TYPE, Dim::value>& io_ref_vec)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                const auto curr_time_sec = clock_->now_seconds();
                for (size_t i = 0; i < Dim::value; i++)
                {
                    if (selection_vec[i]
                        && is_active_dof(get_dof_by_index<Dim::value>(i)))
                    {
                        if (std::isfinite(ref_vec[i]) && std::isfinite(curr_time_sec))
                        {
                            io_ref_vec[i] = ref_vec[i];
                            current_ref_stamp_sec_[i] = curr_time_sec;
                        }
                        else
                        {
                            io_ref_vec[i] = 0.0;
                            current_ref_stamp_sec_[i] = 0.0;
                        }
                    }
                }
            }

            void set_twist_selection(
                const std::array<SignalStatus, Dim::value>& twist_selection)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                const bool changed =
                    state_live_.twist_selection != twist_selection;
                for (size_t i = 0; i < Dim::value; i++)
                {
                    state_live_.twist_selection[i] = twist_selection[i];
                }
                if (changed)
                {
                    RPP_LOG_DEBUG(*logger_, "Twist signal selection updated.");
                }
            }

            void set_wrench_selection(
                const std::array<SignalStatus, Dim::value>& wrench_selection)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                const bool changed =
                    state_live_.wrench_selection != wrench_selection;
                for (size_t i = 0; i < Dim::value; i++)
                {
                    state_live_.wrench_selection[i] = wrench_selection[i];
                }
                if (changed)
                {
                    RPP_LOG_DEBUG(*logger_, "Wrench signal selection updated.");
                }
            }

            void set_wrench_selection_internal_unless_external()
            {
                std::lock_guard<std::mutex> lock(mutex_);
                bool changed = false;
                for (auto& selection : state_live_.wrench_selection)
                {
                    if (selection != SIGNAL_EXT)
                    {
                        changed = changed || selection != SIGNAL_INT;
                        selection = SIGNAL_INT;
                    }
                }
                if (changed)
                {
                    RPP_LOG_DEBUG(*logger_,
                        "Wrench selection set to internal where not external.");
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
                std::lock_guard<std::mutex> lock(mutex_);
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
                std::lock_guard<std::mutex> lock(mutex_);
                return control_identity_token_.token == identity_token;
            }

            bool release_control_identity_token(const FixedSizeString& identity_token)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!control_identity_token_.is_set()
                    || control_identity_token_.token != identity_token)
                {
                    return false;
                }
                control_identity_token_ = Identity{};
                return true;
            }

            bool release_ref_identity_token(const FixedSizeString& identity_token)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                bool released = false;
                for (size_t ref_type = 1; ref_type < REF_END; ++ref_type)
                {
                    for (size_t i = 0; i < DOF_END_i; ++i)
                    {
                        size_t state_index = i;
                        bool has_state_index = true;
                        if constexpr (Dim::value == 3)
                        {
                            switch (i)
                            {
                            case DOF_X_i:
                                state_index = 0;
                                break;
                            case DOF_Y_i:
                                state_index = 1;
                                break;
                            case DOF_N_i:
                                state_index = 2;
                                break;
                            default:
                                has_state_index = false;
                            }
                        }
                        auto& identity = ref_identities[ref_type][i];
                        if (identity.is_set() && identity.token == identity_token)
                        {
                            identity = Identity{};
                            if (has_state_index && ref_type == POSE_REF)
                            {
                                current_pose_ref_stamp_sec_[state_index] = 0.0;
                                state_live_.pose_ref[state_index] = 0.0;
                                state_live_.has_pose_ext[state_index] = false;
                            }
                            else if (has_state_index && ref_type == TWIST_REF)
                            {
                                current_twist_ref_stamp_sec_[state_index] = 0.0;
                                state_live_.twist_ref[state_index] = 0.0;
                                state_live_.has_twist_ext[state_index] = false;
                            }
                            else if (has_state_index && ref_type == WRENCH_REF)
                            {
                                current_wrench_ref_stamp_sec_[state_index] = 0.0;
                                state_live_.wrench_ref[state_index] = 0.0;
                                state_live_.has_wrench_ext[state_index] = false;
                            }
                            released = true;
                        }
                    }
                }
                if (released)
                {
                    state_live_.has_any_pose_ext = false;
                    state_live_.has_any_twist_ext = false;
                    state_live_.has_any_wrench_ext = false;
                    state_live_.wrench.fill(0.0);
                    std::fill(state_live_.commands.begin(),
                        state_live_.commands.end(), 0.0);
                }
                return released;
            }

            std::pair<bool, FixedSizeString> try_request_control_identity_token(const FixedSizeString& identity_name)
            {
                std::lock_guard<std::mutex> lock(mutex_);
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
                std::lock_guard<std::mutex> lock(mutex_);
                bool has_reference = false;
                for (size_t i = 0; i < 6; i++)
                {
                    const auto ref_type = mask[i];
                    if (ref_type == IGNORE)
                        continue;
                    if (ref_type <= 0 || ref_type >= REF_END)
                        return {false, {}};
                    if (!is_active_dof(get_dof_by_index<6>(i)))
                        return {false, {}};
                    if (ref_identities[ref_type][i].is_set())
                        return {false, {}};
                    has_reference = true;
                }
                if (!has_reference)
                    return {false, {}};

                Identity identity;
                Identity::get_random_identity(identity_name, "ref", identity);
                for (size_t i = 0; i < 6; i++)
                {
                    const auto ref_type = mask[i];
                    if (ref_type != IGNORE)
                        ref_identities[ref_type][i] = identity;
                }
                return {true, identity.token};
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

            template <size_t Size>
            static bool is_finite_array(const std::array<FP_TYPE, Size>& values)
            {
                return std::all_of(values.begin(), values.end(),
                    [](const FP_TYPE value) { return std::isfinite(value); });
            }

            void clear_outputs()
            {
                std::lock_guard<std::mutex> lock(mutex_);
                state_live_.wrench.fill(0.0);
                std::fill(state_live_.commands.begin(),
                    state_live_.commands.end(), 0.0);
                state_live_.allocation_suppressed.fill(false);
                allocation_suppression_.active = false;
                allocation_suppression_.previous_requested_wrench.fill(0.0);
            }

            static bool is_single_supported_dof(const DOF dof)
            {
                const auto value = static_cast<int>(dof);
                const auto supported = static_cast<int>(
                    Traits::default_active_dofs);
                return value != 0 && (value & (value - 1)) == 0
                    && (value & ~supported) == 0;
            }

            static size_t dof_index(const DOF dof)
            {
                for (size_t index = 0; index < Dim::value; ++index)
                {
                    if (get_dof_by_index<Dim::value>(index) == dof)
                    {
                        return index;
                    }
                }
                return Dim::value;
            }

            DOF update_allocation_suppression_locked(
                typename ControllerIO::State& state)
            {
                state.allocation_suppressed.fill(false);
                if (allocation_suppression_.priority_dof == NO_DOF
                    || allocation_suppression_.suppressed_dof == NO_DOF
                    || (active_dofs_ & allocation_suppression_.priority_dof)
                        == NO_DOF
                    || (active_dofs_ & allocation_suppression_.suppressed_dof)
                        == NO_DOF)
                {
                    allocation_suppression_.active = false;
                    return active_dofs_;
                }

                const size_t priority_index = dof_index(
                    allocation_suppression_.priority_dof);
                const size_t suppressed_index = dof_index(
                    allocation_suppression_.suppressed_dof);
                if (priority_index == Dim::value || suppressed_index == Dim::value)
                {
                    allocation_suppression_.active = false;
                    return active_dofs_;
                }

                const FP_TYPE priority_request =
                    allocation_suppression_.previous_requested_wrench[
                        priority_index];
                const FP_TYPE suppressed_residual = std::abs(
                    allocation_suppression_.previous_requested_wrench[
                        suppressed_index]
                    - state.wrench[suppressed_index]);
                if (!std::isfinite(priority_request)
                    || !std::isfinite(suppressed_residual))
                {
                    allocation_suppression_.active = false;
                    return active_dofs_;
                }

                const bool priority_requested = std::abs(priority_request)
                    > allocation_suppression_.priority_request_threshold;
                const bool was_active = allocation_suppression_.active;
                const double threshold = allocation_suppression_.active
                    ? allocation_suppression_.residual_deactivation_threshold
                    : allocation_suppression_.residual_activation_threshold;
                allocation_suppression_.active = priority_requested
                    && suppressed_residual >= threshold;
                if (allocation_suppression_.active != was_active)
                {
                    RPP_LOG_DEBUG(*logger_,
                        "Allocation suppression %s priority_dof=0x%x "
                        "request=%.3f suppressed_dof=0x%x residual=%.3f "
                        "threshold=%.3f.",
                        allocation_suppression_.active ? "enabled" : "disabled",
                        static_cast<unsigned int>(
                            allocation_suppression_.priority_dof),
                        static_cast<double>(priority_request),
                        static_cast<unsigned int>(
                            allocation_suppression_.suppressed_dof),
                        static_cast<double>(suppressed_residual),
                        threshold);
                }
                if (!allocation_suppression_.active)
                {
                    return active_dofs_;
                }

                state.allocation_suppressed[suppressed_index] = true;
                return static_cast<DOF>(static_cast<int>(active_dofs_)
                    & ~static_cast<int>(allocation_suppression_.suppressed_dof));
            }

            bool stop_with_warning(const char* message)
            {
                clear_outputs();
                reset_controller();
                RPP_LOG_WARN_ONCE(*logger_, "%s", message);
                return false;
            }

            void reset_controller()
            {
                if (!controller_)
                {
                    return;
                }
                try
                {
                    controller_->reset();
                }
                catch (const std::exception& exception)
                {
                    RPP_LOG_ERROR_ONCE(*logger_,
                        "Controller reset failed: %s.", exception.what());
                }
                catch (...)
                {
                    RPP_LOG_ERROR_ONCE(*logger_, "Controller reset failed.");
                }
            }

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
            AllocationSuppression allocation_suppression_;
            std::mutex step_mutex_;
            mutable std::mutex mutex_;
            typename ControllerIO::State state_live_;
            typename ControllerIO::State state_locked_;
            typename ControllerIO::Error errors_;

            std::shared_ptr<ControllerComponent> controller_;
            std::shared_ptr<AllocatorComponent> allocator_;

            std::array<SignalStatus, Dim::value> current_wrench_selection_;
            std::array<SignalStatus, Dim::value> current_twist_selection_;

            std::array<double, Dim::value> current_pose_ref_stamp_sec_;
            std::array<double, Dim::value> current_twist_ref_stamp_sec_;
            std::array<double, Dim::value> current_wrench_ref_stamp_sec_;

            Identity control_identity_token_;
            std::array<std::array<Identity, DOF_END_i>, REF_END> ref_identities = {{}};

            // pre-allocated messages to avoid dynamic allocation during step
            InputMessage ref_state_msg_;
            InputMessage curr_state_msg_;
            EnablerOdometry enabler_odom_msg_;
    };
}
