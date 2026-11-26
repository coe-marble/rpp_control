#pragma once

#include <tuple>

#include <rpp_cpp/plugin.hpp>
#include <rpp_cpp/logger.hpp>
#include <rpp_plugin_types/rpp_interfaces/ControlLaw.hpp>
#include <rpp_control/control_plugin_contract.hpp>
#include <rpp_control/control_auxiliary_state.hpp>
#include <rpp_control/detail/field_values.hpp>
#include <rpp_control/numeric_output_pool.hpp>
#include <rpp_control/pid.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

namespace rpp_control
{
template<class Derived, std::size_t N>
class PidControlLaw : public rpp_interfaces::ControlLaw
{
public:
  using FieldsDesc = rpp_schema::rpp_interfaces::FieldListDesc_Native;
  using PidParameters = PIDParameters;
  using Result = std::tuple<rpp::ListConst<double>, std::uint8_t>;
  static_assert(N > 0 && N <= rpp_interfaces::FieldLayout::max_width);

  ControlLawDesc::Const getDesc() final
  {
    if (!configured_) {throw std::logic_error("Configure the PID controller first");}
    return control_description(state_, references_, commands_);
  }

  ControlAuxiliaryState::Snapshot::Const getAuxiliaryState() final
  {
    if (!configured_) {throw std::logic_error("Configure the PID controller first");}
    rpp::List<double> integral, previous_error, derivative;
    integral.resize(N);
    previous_error.resize(N);
    derivative.resize(N);
    for (std::size_t index = 0; index < N; ++index) {
      const auto history = pids_[index].error_info();
      integral[index] = history.integral_error;
      previous_error[index] = history.previous_error;
      derivative[index] = history.previous_derivative;
    }
    ControlAuxiliaryState auxiliary(last_auxiliary_time_);
    auxiliary.integral(auxiliary_axes_, integral)
      .previous_error(auxiliary_axes_, previous_error)
      .previous_derivative(auxiliary_axes_, derivative);
    if (requested_command_valid_) {
      rpp::List<double> requested;
      requested.resize(N);
      for (std::size_t index = 0; index < N; ++index) {requested[index] = requested_command_[index];}
      rpp::List<std::uint64_t> offsets;
      offsets.resize(1);
      offsets[0] = 0;
      auxiliary.horizon_inputs(commands_, offsets, requested);
    }
    return auxiliary.snapshot();
  }

  Result compute(
    rpp::ListConst<double> state, rpp::ListConst<double> references,
    rpp::ListConst<bool> enabledAxes, rpp::ListConst<bool> inputValid,
    rpp::ListConst<double> previousInput, bool previousInputValid,
    std::uint64_t timeNs, std::uint64_t dtNs) final
  {
    if (validate_compute_) {
      if (!active_) {return failure(rpp_interfaces::contract::CODE_NOT_READY);}
      if (dtNs == 0 || state.size() != state_size_ || references.size() != reference_size_ ||
        (!enabledAxes.empty() && enabledAxes.size() != N) ||
        (!inputValid.empty() && inputValid.size() != reference_size_) ||
        !state_.valid_row(state) || !references_.valid_row(references) ||
        (previousInputValid && (previousInput.size() != N || !detail::all_finite(previousInput))))
      {
        reset_pids();
        return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
      }
      if (has_time_ && timeNs < last_time_) {
        reset_pids();
        return failure(rpp_interfaces::contract::CODE_STALE_INPUT);
      }
    }
    const auto buffer = outputs_.acquire();
    if (buffer == NumericOutputPool::unavailable) {return failure(rpp_interfaces::contract::CODE_NOT_READY);}
    // Missing targets contribute zero instantaneous error. Derived controllers
    // decide which output axes can still track the remaining valid inputs.
    bool validity_changed = false;
    for (std::size_t index = 0; index < reference_size_; ++index) {
      const bool valid = inputValid.empty() || inputValid[index];
      validity_changed = validity_changed || previous_input_valid_[index] != valid;
      previous_input_valid_[index] = valid;
      effective_references_[index] = valid ?
        references[index] : state[reference_state_route_[index]];
    }
    // Pose errors rotate into body axes. Reset their accumulated history when
    // validity changes so an expired world-frame target leaves no stored effort.
    if (validity_changed) {reset_pids();}
    const auto & derived = static_cast<const Derived &>(*this);
    const auto errors = derived.calculate_error(state, effective_references_);
    const auto dt = static_cast<double>(dtNs) * 1e-9;
    auto & command = outputs_.values(buffer);
    bool limited = false;
    for (std::size_t i = 0; i < N; ++i) {
      if ((!enabledAxes.empty() && !enabledAxes[i]) || !derived.axis_input_available(inputValid, i)) {
        pids_[i].reset();
        command[i] = 0.0;
        requested_command_[i] = 0.0;
        previous_axis_enabled_[i] = false;
        continue;
      }
      double tracking_error = 0.0;
      // A newly enabled axis starts fresh instead of tracking feedback from its disabled tick.
      if (previousInputValid && requested_command_valid_ && previous_axis_enabled_[i]) {
        const double applied = previousInput[i];
        if (applied != requested_command_[i]) {
          tracking_error = pid_output_units(applied, i) - pid_output_units(requested_command_[i], i);
        }
      }
      double output = pids_[i].step(errors[i], 0.0, dt, angular_[i], tracking_error);
      if (scale_normalized_output_) {
        const double normalized = std::clamp(output, -1.0, 1.0);
        limited = limited || normalized != output;
        output = normalized * (normalized >= 0.0 ? positive_scales_[i] : negative_scales_[i]);
      }
      command[i] = output;
      requested_command_[i] = output;
      previous_axis_enabled_[i] = true;
      limited = limited || pids_[i].last_step_limited();
    }
    if (validate_compute_) {
      for (std::size_t i = 0; i < N; ++i) {
        if ((previous_axis_enabled_[i] && !pids_[i].last_step_valid()) ||
          !std::isfinite(static_cast<double>(command[i])))
        {
          reset_pids();
          return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
        }
      }
      last_time_ = timeNs;
      has_time_ = true;
    }
    requested_command_valid_ = true;
    last_auxiliary_time_ = timeNs;
    return {outputs_.view(buffer), limited ? rpp_interfaces::contract::CODE_LIMITED : rpp_interfaces::contract::CODE_OK};
  }

  rpp::CallbackResult on_activate(const std::optional<HandoverState::Const> & handover) final
  {
    if (!configured_) {return rpp::CallbackResult::failure("Configure the PID controller first");}
    std::array<ControllerAxisErrorInfo, N> restored{};
    std::optional<std::vector<std::size_t>> control_route;
    std::optional<std::vector<double>> requested_control;
    if (handover) {
      try {
        const auto measured = handover->state();
        const auto previous_control = handover->previousControl();
        const auto auxiliary = handover->auxiliaryState();
        validate_handover_values(measured);
        validate_handover_values(previous_control);
        validate_handover_values(auxiliary);
        const rpp_interfaces::FieldLayout available_state(measured.desc().as_struct().fields);
        if (!rpp_interfaces::validate_required_fields(available_state, state_)) {
          // compute() receives fresh state; this snapshot check is advisory at activation.
          RPP_LOG_WARN(*logger_,
            "PID handover state layout is missing or incompatible with required fields. Continuing activation.");
        }
        const rpp_interfaces::FieldLayout available_controls(previous_control.desc().as_struct().fields);
        control_route = handover_control_route(available_controls, commands_);
        const auto integral = ControlAuxiliaryState::read(
          "control.integral", auxiliary_axes_, auxiliary);
        auto previous_error = ControlAuxiliaryState::read(
          "control.previousError", auxiliary_axes_, auxiliary);
        if (!previous_error) {
          previous_error = ControlAuxiliaryState::read(
            "control.error", auxiliary_axes_, auxiliary);
        }
        const auto derivative = ControlAuxiliaryState::read(
          "control.previousDerivative", auxiliary_axes_, auxiliary);
        requested_control = ControlAuxiliaryState::read(
          "control.horizonInput.0", commands_, auxiliary);
        for (std::size_t index = 0; index < N; ++index) {
          if (integral) {restored[index].integral_error = (*integral)[index];}
          if (previous_error) {restored[index].previous_error = (*previous_error)[index];}
          if (derivative) {restored[index].previous_derivative = (*derivative)[index];}
        }
      } catch (const std::invalid_argument & error) {
        return rpp::CallbackResult::failure(error.what());
      }
    }
    reset();
    for (std::size_t i = 0; i < N; ++i) {pids_[i].restore_error_info(restored[i]);}
    if (requested_control) {
      for (std::size_t index = 0; index < N; ++index) {requested_command_[index] = (*requested_control)[index];}
      requested_command_valid_ = true;
    } else if (handover && control_route) {
      const auto previous_control = handover->previousControl().state();
      if (!previous_control.empty()) {
        for (std::size_t index = 0; index < N; ++index) {
          requested_command_[index] = previous_control[(*control_route)[index]];
        }
        requested_command_valid_ = true;
      }
    }
    previous_axis_enabled_.fill(requested_command_valid_);
    active_ = true;
    return {};
  }

  rpp::CallbackResult on_deactivate() final
  {
    active_ = false;
    return {};
  }

  rpp::CallbackResult on_cleanup() final
  {
    active_ = false;
    configured_ = false;
    reset();
    outputs_.clear();
    logger_.reset();
    state_ = rpp_interfaces::FieldLayout{};
    references_ = rpp_interfaces::FieldLayout{};
    commands_ = rpp_interfaces::FieldLayout{};
    effective_references_ = rpp::List<double>{};
    reference_state_route_.clear();
    previous_input_valid_.clear();
    auxiliary_axes_ = rpp_interfaces::FieldLayout{};
    state_size_ = reference_size_ = 0;
    return {};
  }

  void reset() final {reset_pids(); has_time_ = false;}

protected:
  // Unchecked callers must supply valid inputs, mask width and timing to an active controller.
  explicit PidControlLaw(bool validate_compute = true)
  : validate_compute_(validate_compute) {}

  void configure_output_scaling(const rpp::ComponentContext & context)
  {
    const auto positive = context.get_parameter<std::vector<double>>("positive_output_scales");
    const auto negative = context.get_parameter<std::vector<double>>("negative_output_scales");
    if (positive.size() != N || negative.size() != N) {throw std::invalid_argument("PID output scales must match its axis count");}
    for (std::size_t i = 0; i < N; ++i) {
      if (!std::isfinite(positive[i]) || positive[i] <= 0.0 || !std::isfinite(negative[i]) || negative[i] <= 0.0) {
        throw std::invalid_argument("PID output scales must be finite and positive");
      }
      positive_scales_[i] = positive[i]; negative_scales_[i] = negative[i];
    }
    scale_normalized_output_ = context.get_parameter<bool>("scale_normalized_output");
  }

  rpp::CallbackResult configure_pid(
    const rpp::ComponentContext & context,
    FieldsDesc state_desc, FieldsDesc reference_desc, FieldsDesc command_desc,
    const std::array<PidParameters, N> & parameters,
    std::array<bool, N> angular = std::array<bool, N>{})
  {
    if (active_) {return rpp::CallbackResult::failure("Deactivate before configuring the PID");}
    rpp_interfaces::FieldLayout state(std::move(state_desc.fields));
    rpp_interfaces::FieldLayout references(std::move(reference_desc.fields));
    rpp_interfaces::FieldLayout commands(std::move(command_desc.fields));
    if (state.size() == 0 || references.size() == 0 || commands.size() != N) {
      return rpp::CallbackResult::failure("PID inputs must be nonempty and commands must match its axis count");
    }
    for (const auto & command : commands.fields()) {
      if (command.kind != rpp_interfaces::contract::COORDINATE_EUCLIDEAN) {
        return rpp::CallbackResult::failure("PID commands must use Euclidean coordinates");
      }
    }
    std::array<PID, N> pids{};
    for (std::size_t i = 0; i < N; ++i) {pids[i] = PID(parameters[i], validate_compute_);}
    outputs_.prepare(N);
    logger_ = context.get_logger();
    state_ = std::move(state);
    references_ = std::move(references);
    commands_ = std::move(commands);
    auto auxiliary_axes = commands_.fields();
    for (auto & field : auxiliary_axes) {
      // Internal history stores numeric coefficients; names and frames identify the control axes.
      field.kind = rpp_interfaces::contract::COORDINATE_EUCLIDEAN;
      field.quantity = rpp_interfaces::contract::QUANTITY_NORMALIZED;
    }
    auxiliary_axes_ = rpp_interfaces::FieldLayout(std::move(auxiliary_axes));
    state_size_ = state_.size();
    reference_size_ = references_.size();
    reference_state_route_ = rpp_interfaces::field_route(state_, references_);
    effective_references_.resize(reference_size_);
    previous_input_valid_.assign(reference_size_, true);
    angular_ = angular;
    pids_ = std::move(pids);
    requested_command_valid_ = false;
    previous_axis_enabled_.fill(false);
    has_time_ = false;
    last_auxiliary_time_ = 0;
    configured_ = true;
    return {};
  }

private:
  const bool validate_compute_;
  std::shared_ptr<const rpp::RppLogger> logger_;
  std::vector<std::size_t> reference_state_route_;
  rpp::List<double> effective_references_;
  std::vector<bool> previous_input_valid_;
  rpp_interfaces::FieldLayout state_, references_, commands_;
  rpp_interfaces::FieldLayout auxiliary_axes_;
  std::size_t state_size_ = 0, reference_size_ = 0;
  std::array<PID, N> pids_{};
  std::array<bool, N> angular_{};
  std::array<double, N> positive_scales_{}, negative_scales_{};
  bool scale_normalized_output_ = false;
  std::array<double, N> requested_command_{};
  std::array<bool, N> previous_axis_enabled_{};
  bool requested_command_valid_ = false;
  NumericOutputPool outputs_;
  bool active_ = false;
  bool configured_ = false, has_time_ = false;
  std::uint64_t last_time_ = 0;
  std::uint64_t last_auxiliary_time_ = 0;

  double pid_output_units(double command, std::size_t axis) const
  {
    return scale_normalized_output_ ?
      command / (command >= 0.0 ? positive_scales_[axis] : negative_scales_[axis]) : command;
  }
  void reset_pids()
  {
    for (auto & pid : pids_) {pid.reset();}
    requested_command_valid_ = false;
    previous_axis_enabled_.fill(false);
    last_auxiliary_time_ = 0;
  }
  Result failure(std::uint8_t code) const {return {outputs_.empty(), code};}
};
}  // namespace rpp_control
