#pragma once

#include <tuple>

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_interfaces/ControlLaw.hpp>
#include <rpp_control/control_plugin_contract.hpp>
#include <rpp_control/control_auxiliary_state.hpp>
#include <rpp_control/detail/control_bindings.hpp>
#include <rpp_control/detail/field_values.hpp>
#include <rpp_interfaces/field_routing.hpp>
#include <rpp_control/numeric_output_pool.hpp>

#include <array>
#include <optional>
#include <stdexcept>

namespace rpp_control
{
template<std::size_t ControllerCount>
class CascadeControlLaw : public rpp_interfaces::ControlLaw
{
  static_assert(ControllerCount > 0 && ControllerCount <= 8);

  struct Stage : detail::ControlLawBinding
  {
    std::vector<std::size_t> external_route, selection_route;
    rpp::List<bool> reference_valid;
    bool applied_command_valid = false;
  };

public:
  using Result = std::tuple<rpp::ListConst<double>, std::uint8_t>;

  // Unchecked callers must supply valid inputs to an active cascade.
  explicit CascadeControlLaw(bool validate_compute = true)
  : validate_compute_(validate_compute) {}

  rpp::CallbackResult on_configure(const rpp::ComponentContext & context) final
  {
    if (active_) {return rpp::CallbackResult::failure("Deactivate the cascade before configuring");}
    std::array<Stage, ControllerCount> prepared;
    rpp_interfaces::FieldLayout state;
    rpp_interfaces::ReferenceHorizon outer_horizon;
    // ctl0 is innermost; execute from the highest required index towards ctl0.
    for (std::size_t i = 0; i < ControllerCount; ++i) {
      const auto slot_index = ControllerCount - 1 - i;
      const auto label = std::string("ctl") + std::to_string(slot_index);
      const auto & child = context.get_subcomponent_context(label);
      auto description = child.get_instance<rpp_interfaces::ControlLaw>()->getDesc();
      auto desc = description.as_struct();
      if (i > 0 && desc.horizonLength > 1) {
        return rpp::CallbackResult::failure(label + " must consume current references only");
      }
      if (i == 0) {
        outer_horizon = rpp_interfaces::ControlLawLayout(desc).horizon();
      }
      auto & stage = prepared[i];
      stage.name = label;
      stage.instance_id = child.get_instance_id();
      stage.set_layouts(std::move(desc));
      if (stage.commands.size() == 0) {
        return rpp::CallbackResult::failure(label + " must declare command outputs");
      }
      if (i > 0) {
        try {
          stage.reference_route = rpp_interfaces::field_route(prepared[i - 1].commands,
              stage.references);
        } catch (const std::exception & error) {
          return rpp::CallbackResult::failure("ctl" + std::to_string(slot_index + 1) + " -> " +
              label + ": " + error.what());
        }
      }
      state = rpp_interfaces::merge_fields(state, stage.state);
    }
    for (std::size_t i = 0; i < ControllerCount; ++i) {
      auto & stage = prepared[i];
      stage.bind_state(state);
      const auto reference_count = i == 0 ? outer_horizon.value_count() : stage.references.size();
      stage.reference_values.resize(reference_count);
      stage.reference_valid.resize(reference_count);
    }
    auto references = prepared.front().references;
    expose_inner_references_ = context.get_parameter<bool>("expose_inner_references");
    if (expose_inner_references_) {
      for (std::size_t i = 1; i < ControllerCount; ++i) {
        references = rpp_interfaces::merge_fields(references, prepared[i].references);
        std::vector<rpp_interfaces::Field> modes;
        for (std::size_t j = 0; j < prepared[i].references.fields().size(); ++j) {
          auto field = prepared[i].references.fields()[j];
          if (field.kind == rpp_interfaces::contract::COORDINATE_QUATERNION) {
            return rpp::CallbackResult::failure(
                "Inner reference selection requires Euclidean or angle fields");
          }
          field.name = prepared[i].name + "." + field.name + ".selection";
          field.kind = rpp_interfaces::contract::COORDINATE_EUCLIDEAN;
          field.quantity = rpp_interfaces::contract::QUANTITY_NORMALIZED;
          field.size = prepared[i].references.ranges()[j].size;
          modes.push_back(std::move(field));
        }
        const rpp_interfaces::FieldLayout selections(std::move(modes));
        references = rpp_interfaces::merge_fields(references, selections);
        prepared[i].selection_route = rpp_interfaces::field_route(references, selections);
      }
    }
    const auto samples = outer_horizon.sample_count();
    std::vector<std::uint64_t> offsets;
    if (outer_horizon.horizon_length() != 0) {
      if (references.size() > rpp_interfaces::ReferenceHorizon::max_values / samples) {
        return rpp::CallbackResult::failure("Cascade reference horizon exceeds capacity");
      }
      offsets.resize(references.size() * samples);
      for (std::size_t scalar = 0; scalar < references.size(); ++scalar) {
        // Added inner overrides use the first outer field's schedule; only sample zero is read.
        const auto outer_scalar = scalar < prepared.front().references.size() ? scalar : 0;
        for (std::size_t sample = 0; sample < samples; ++sample) {
          offsets[scalar * samples + sample] = outer_horizon.offset(outer_scalar, sample);
        }
      }
    }
    rpp_interfaces::ReferenceHorizon horizon(references, offsets, outer_horizon.horizon_length());
    for (std::size_t i = 0; i < ControllerCount; ++i) {
      if (i == 0 || expose_inner_references_) {
        const auto fields = rpp_interfaces::field_route(references, prepared[i].references);
        auto & route = prepared[i].external_route;
        route.reserve(i == 0 ? outer_horizon.value_count() : fields.size());
        for (const auto scalar : fields) {
          // Only the outermost stage consumes all samples in reference-major order.
          for (std::size_t sample = 0; sample < (i == 0 ? samples : 1); ++sample) {
            route.push_back(scalar * samples + sample);
          }
        }
      }
      for (auto & scalar : prepared[i].selection_route) {
        scalar *= samples;
      }
    }
    auto commands = prepared.back().commands;
    stages_ = std::move(prepared);
    state_ = std::move(state);
    references_ = std::move(references);
    horizon_ = std::move(horizon);
    commands_ = std::move(commands);
    state_size_ = state_.size();
    reference_size_ = horizon_.value_count();
    command_size_ = commands_.size();
    context_ = &context;
    last_auxiliary_time_ = 0;
    return {};
  }

  ControlLawDesc::Const getDesc() final
  {
    if (!context_) {throw std::logic_error("Configure the cascade first");}
    rpp_schema::rpp_interfaces::ControlLawDesc_Native definition;
    definition.state.fields = state_.fields();
    definition.reference.fields = references_.fields();
    definition.control.fields = commands_.fields();
    definition.horizonLength = horizon_.horizon_length();
    if (definition.horizonLength != 0) {
      definition.horizonOffsetsNs.resize(reference_size_);
      const auto samples = horizon_.sample_count();
      for (std::size_t scalar = 0; scalar < references_.size(); ++scalar) {
        for (std::size_t sample = 0; sample < samples; ++sample) {
          definition.horizonOffsetsNs[scalar * samples + sample] = horizon_.offset(scalar, sample);
        }
      }
    }
    ControlLawDesc description;
    write_control_description(description, definition);
    return ControlLawDesc::Const(std::move(description));
  }

  Result compute(
    rpp::ListConst<double> state, rpp::ListConst<double> references,
    rpp::ListConst<bool> enabledAxes, rpp::ListConst<bool> inputValid,
    rpp::ListConst<double> previous_applied_command, bool previous_applied_command_valid,
    std::uint64_t timeNs, std::uint64_t dtNs) final
  {
    if (validate_compute_) {
      if (!active_) {return failure(rpp_interfaces::contract::CODE_NOT_READY);}
      if (dtNs == 0 || state.size() != state_size_ || references.size() != reference_size_ ||
        (!enabledAxes.empty() && enabledAxes.size() != command_size_) ||
        (!inputValid.empty() && inputValid.size() != reference_size_) ||
        !detail::all_finite(state) || !detail::all_finite(references) ||
        !valid_selections(references) ||
        (previous_applied_command_valid &&
        (previous_applied_command.size() != command_size_ ||
        !detail::all_finite(previous_applied_command))))
      {
        return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
      }
    }
    std::array<rpp_interfaces::ControlLaw *, ControllerCount> controllers{};
    auto run = [&](const auto & active_controllers) -> Result {
        auto stage_command = empty_.empty();
        bool stage_command_available = false;
        std::uint8_t status = rpp_interfaces::contract::CODE_OK;
        for (std::size_t i = 0; i < ControllerCount; ++i) {
          auto & stage = stages_[i];
          detail::gather_fields(state, stage.state_route, stage.state_values);
          if (i == 0) {
            detail::gather_fields(references, stage.external_route, stage.reference_values);
            for (std::size_t axis = 0; axis < stage.external_route.size(); ++axis) {
              stage.reference_valid[axis] = inputValid.empty() ||
                inputValid[stage.external_route[axis]];
            }
          } else if (!expose_inner_references_) {
            detail::gather_fields(stage_command, stage.reference_route, stage.reference_values);
            for (std::size_t axis = 0; axis < stage.reference_valid.size(); ++axis) {
              stage.reference_valid[axis] = stage_command_available;
            }
          } else {
            for (std::size_t j = 0; j < stage.selection_route.size(); ++j) {
              const double mode = inputValid.empty() || inputValid[stage.selection_route[j]] ?
                references[stage.selection_route[j]] : rpp_interfaces::contract::REFERENCE_INTERNAL;
              if (mode == rpp_interfaces::contract::REFERENCE_EXTERNAL) {
                stage.reference_values[j] = references[stage.external_route[j]];
                stage.reference_valid[j] = inputValid.empty() ||
                  inputValid[stage.external_route[j]];
              } else if (mode == rpp_interfaces::contract::REFERENCE_DISABLED ||
                mode == rpp_interfaces::contract::REFERENCE_DONT_CARE)
              {
                stage.reference_values[j] = 0.0;
                stage.reference_valid[j] = true;
              } else {
                stage.reference_values[j] = stage_command[stage.reference_route[j]];
                stage.reference_valid[j] = stage_command_available;
              }
            }
          }
          const bool innermost = i + 1 == ControllerCount;
        // The caller supplies feedback in the final command layout; outer stages
        // track the references selected at their own cascade boundaries.
          const bool final_command_feedback = innermost && previous_applied_command_valid;
          const bool applied_command_valid = final_command_feedback || stage.applied_command_valid;
          const rpp::ListConst<double> applied_command =
            final_command_feedback ? previous_applied_command : stage.applied_command;
        // The public mask uses final command axes. Outer stages have their own command spaces.
          const rpp::ListConst<bool> stage_enabled_axes = innermost ?
            enabledAxes : rpp::ListConst<bool>(capnp::List<bool>::Reader{});
          auto [values, result_status] = active_controllers[i]->compute(
          stage.state_values, stage.reference_values, stage_enabled_axes, stage.reference_valid,
          applied_command, applied_command_valid, timeNs, dtNs);
          if (!control_success(result_status)) {return failure(result_status);}
        // The next stage indexes this list with a route built from the declared output layout.
          if (validate_compute_ && !innermost && values.size() != stage.commands.size()) {
            return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
          }
          if (result_status == rpp_interfaces::contract::CODE_LIMITED) {
            status = rpp_interfaces::contract::CODE_LIMITED;
          }
          stage_command = std::move(values);
          stage_command_available = stage.reference_valid.empty();
          for (std::size_t axis = 0; axis < stage.reference_valid.size(); ++axis) {
            stage_command_available = stage_command_available || stage.reference_valid[axis];
          }
        }
        return {std::move(stage_command), status};
      };
    auto [values, status] = with_controllers<0>(controllers, run);
    if (!control_success(status)) {
      clear_stage_feedback();
      return {std::move(values), status};
    }
    if (validate_compute_ && (values.size() != command_size_ || !detail::all_finite(values))) {
      clear_stage_feedback();
      return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
    }
    record_stage_feedback();
    last_auxiliary_time_ = timeNs;
    return {std::move(values), status};
  }

  ControlAuxiliaryState::Snapshot::Const getAuxiliaryState() final
  {
    if (!context_) {throw std::logic_error("Configure the cascade first");}
    ControlAuxiliaryState auxiliary(last_auxiliary_time_);
    for (const auto & stage : stages_) {
      auto [read, snapshot] = context_->with_active_component<rpp_interfaces::ControlLaw>(
        stage.name, {}, [](auto & controller) {
          return controller.getAuxiliaryState();
          }, stage.instance_id);
      if (!read) {
        throw std::runtime_error("Cannot read auxiliary state from '" + stage.name + "'");
      }
      auxiliary.merge(*snapshot);
      if (stage.applied_command_valid) {
        auxiliary.previous_control(stage.commands, stage.applied_command);
      }
    }
    return auxiliary.snapshot();
  }

  void reset() final
  {
    clear_stage_feedback();
    last_auxiliary_time_ = 0;
    if (!context_) {return;}
    for (const auto & stage : stages_) {
      (void)context_->with_active_component<rpp_interfaces::ControlLaw>(
        stage.name, {}, [](auto & child) {child.reset();}, stage.instance_id);
    }
  }

  rpp::CallbackResult on_activate(const std::optional<HandoverState::Const> & state) final
  {
    if (!context_) {return rpp::CallbackResult::failure("Configure the cascade first");}
    for (std::size_t i = 0; i < ControllerCount; ++i) {
      if (subcontext(i).get_instance_id() != stages_[i].instance_id) {
        return rpp::CallbackResult::failure("Reconfigure after replacing " + stages_[i].name);
      }
    }
    std::array<std::optional<std::vector<double>>, ControllerCount> applied_commands;
    if (state) {
      try {
        const auto previous_control = state->previousControl();
        validate_handover_values(previous_control);
        const rpp_interfaces::FieldLayout available_controls(
          previous_control.desc().as_struct().fields);
        const auto previous_values = previous_control.state();
        const auto auxiliary = state->auxiliaryState();
        for (std::size_t index = 0; index < ControllerCount; ++index) {
          const auto & commands = stages_[index].commands;
          auto & applied = applied_commands[index];
          const auto route = handover_control_route(available_controls, commands);
          if (route && !previous_values.empty()) {
            applied.emplace();
            for (const auto source : *route) {applied->push_back(previous_values[source]);}
          } else {
            applied = ControlAuxiliaryState::read(
              "control.previousControl", commands, auxiliary);
          }
        }
      } catch (const std::exception & error) {
        return rpp::CallbackResult::failure(error.what());
      }
    }
    for (std::size_t i = 0; i < ControllerCount; ++i) {
      // Every stage receives the same application state and selects its own fields.
      // Copy the wire value so a child can retain it after this callback returns.
      auto child_state = state ?
        rpp::HandoverState(HandoverState::Const::from_wire(*state)) : rpp::HandoverState{};
      auto result = subcontext(i).activate(std::move(child_state));
      if (!result.success) {
        for (std::size_t j = i; j-- > 0; ) {(void)subcontext(j).deactivate();}
        return rpp::CallbackResult::failure(stages_[i].name + ": " + result.message);
      }
    }
    clear_stage_feedback();
    for (std::size_t index = 0; index < ControllerCount; ++index) {
      if (!applied_commands[index]) {continue;}
      auto & stage = stages_[index];
      for (std::size_t axis = 0; axis < applied_commands[index]->size(); ++axis) {
        stage.applied_command[axis] = (*applied_commands[index])[axis];
      }
      stage.applied_command_valid = true;
    }
    active_ = true;
    return {};
  }

  rpp::CallbackResult on_deactivate() final
  {
    if (!context_) {return rpp::CallbackResult::failure("Configure the cascade first");}
    active_ = false;
    clear_stage_feedback();
    rpp::CallbackResult first_failure;
    for (std::size_t i = ControllerCount; i-- > 0; ) {
      auto result = subcontext(i).deactivate();
      if (!result.success && first_failure.success) {first_failure = std::move(result);}
    }
    return first_failure;
  }

  rpp::CallbackResult on_cleanup() final
  {
    active_ = false;
    context_ = nullptr;
    state_size_ = reference_size_ = command_size_ = 0;
    stages_ = std::array<Stage, ControllerCount>{};
    horizon_ = rpp_interfaces::ReferenceHorizon{};
    state_ = rpp_interfaces::FieldLayout{}; references_ = rpp_interfaces::FieldLayout{};
    commands_ = rpp_interfaces::FieldLayout{};
    return {};
  }

private:
  const bool validate_compute_;
  const rpp::ComponentContext * context_ = nullptr;
  bool expose_inner_references_ = false;
  std::array<Stage, ControllerCount> stages_;
  std::size_t state_size_ = 0, reference_size_ = 0, command_size_ = 0;
  rpp_interfaces::FieldLayout state_, references_, commands_;
  rpp_interfaces::ReferenceHorizon horizon_;
  NumericOutputPool empty_;
  bool active_ = false;
  std::uint64_t last_auxiliary_time_ = 0;

  const rpp::SubcomponentContext & subcontext(std::size_t index) const
  {return context_->get_subcomponent_context(stages_[index].name);}
  Result failure(std::uint8_t code) const {return {empty_.empty(), code};}

  void record_stage_feedback()
  {
    for (std::size_t i = 1; i < ControllerCount; ++i) {
      auto & outer = stages_[i - 1];
      const auto & inner = stages_[i];
      // A single validity flag describes the whole command.
      if (inner.reference_route.size() != outer.commands.size()) {continue;}
      outer.applied_command_valid = !validate_compute_ ||
        detail::all_finite(inner.reference_values);
      if (outer.applied_command_valid) {
        detail::scatter_fields(inner.reference_values, inner.reference_route,
            outer.applied_command);
      }
    }
  }

  void clear_stage_feedback()
  {
    for (auto & stage : stages_) {stage.applied_command_valid = false;}
  }

  // Nest stack-owned call guards so all children remain active throughout the update.
  template<std::size_t Index, typename Callback>
  Result with_controllers(
    std::array<rpp_interfaces::ControlLaw *, ControllerCount> & controllers, Callback & callback)
  {
    if constexpr (Index == ControllerCount) {
      return callback(controllers);
    } else {
      auto [success, result] = context_->with_active_component<rpp_interfaces::ControlLaw>(
        stages_[Index].name, {},
        [&](rpp_interfaces::ControlLaw & controller) -> Result {
          controllers[Index] = &controller;
          return with_controllers<Index + 1>(controllers, callback);
        }, stages_[Index].instance_id);
      if (!success) {return failure(rpp_interfaces::contract::CODE_NOT_READY);}
      return std::move(*result);
    }
  }
  bool valid_selections(const rpp::ListConst<double> & references) const
  {
    for (const auto & stage : stages_) {
      for (const auto index : stage.selection_route) {
        const double mode = references[index];
        if (mode != rpp_interfaces::contract::REFERENCE_EXTERNAL &&
          mode != rpp_interfaces::contract::REFERENCE_INTERNAL &&
          mode != rpp_interfaces::contract::REFERENCE_DISABLED &&
          mode != rpp_interfaces::contract::REFERENCE_DONT_CARE)
        {
          return false;
        }
      }
    }
    return true;
  }
};
}  // namespace rpp_control
