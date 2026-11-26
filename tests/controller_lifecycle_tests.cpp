#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <rpp_control/pid.hpp>
#include <rpp_control/control_plugin_contract.hpp>
#include <rpp_control/control_auxiliary_state.hpp>
#include <rpp_control/control_script.hpp>
#include <rpp_cpp/component_runtime_builder.hpp>
#include <rpp_plugin_types/rpp_interfaces/ControlLaw.hpp>
#include <rpp_plugin_types/rpp_interfaces/ReferenceGenerator.hpp>
#include <capnp_gen/rpp_interfaces/common/constants.capnp.h>

namespace
{
using ControllerState = rpp_schema::rpp_interfaces::ControllerHandoverState;
const rpp::ListConst<bool> all_axes_enabled(capnp::List<bool>::Reader{});
const rpp::ListConst<bool> all_inputs_valid(capnp::List<bool>::Reader{});
const rpp::ListConst<bool> update_all_values(capnp::List<bool>::Reader{});
constexpr auto goal_deadline = std::numeric_limits<std::uint64_t>::max();

rpp::ScriptRuntime build_control_runtime()
{
  rpp::RppDataManager data_manager(rpp::RPP_HOME);
  rpp::ComponentRuntimeBuilder builder(
    data_manager, rpp::ClockOptions(rpp::RPP_CLOCK_MOCK));
  // Construct the selected alternative in place to avoid GCC's temporary-variant warning.
  rpp::ScriptRequest request{};
  request.source.emplace<rpp::ScriptDescriptionPath>().path =
    "./tests/data/test_description/motion_controller2d.json";
  request.parts_folder = "./tests/data/test_ws_parts";
  return builder.build_script(request);
}

ControllerState::Const application_state(
  const rpp_interfaces::FieldLayout & state_fields,
  const rpp_interfaces::FieldLayout & control_fields,
  const std::vector<double> & integral_errors = {})
{
  ControllerState state;
  auto measured = state.state();
  auto measured_description = measured.desc();
  state_fields.write(measured_description);
  auto previous_control = state.previousControl();
  auto control_description = previous_control.desc();
  control_fields.write(control_description);
  auto measured_values = measured.state();
  measured_values.resize(state_fields.size());
  if (!integral_errors.empty()) {
    auto auxiliary_fields = control_fields.fields();
    for (auto & field : auxiliary_fields) {
      field.kind = rpp_interfaces::contract::COORDINATE_EUCLIDEAN;
      field.quantity = rpp_interfaces::contract::QUANTITY_NORMALIZED;
    }
    const rpp_interfaces::FieldLayout axes(std::move(auxiliary_fields));
    rpp::List<double> integral;
    integral.resize(integral_errors.size());
    for (std::size_t index = 0; index < integral_errors.size(); ++index) {
      integral[index] = integral_errors[index];
    }
    rpp_control::ControlAuxiliaryState auxiliary;
    auto auxiliary_state = state.auxiliaryState();
    auxiliary.integral(axes, integral).write_to(auxiliary_state);
  }
  return ControllerState::Const(std::move(state));
}

}  // namespace

TEST(ControlFieldValidation, ChecksRequiredFieldsWithoutBuildingARouteOrThrowing)
{
  using namespace rpp_interfaces::contract;
  const rpp_interfaces::FieldLayout available({
    {"base.angularVelocity", COORDINATE_EUCLIDEAN, QUANTITY_ANGULAR_VELOCITY, 1,
      "base_link", "base_link", "base"},
    {"base.linearVelocity", COORDINATE_EUCLIDEAN, QUANTITY_LINEAR_VELOCITY, 2,
      "base_link", "base_link", "base"}});
  const rpp_interfaces::FieldLayout required({available.fields()[1], available.fields()[0]});
  EXPECT_TRUE(rpp_interfaces::validate_required_fields(available, required));
  EXPECT_TRUE(rpp_interfaces::validate_required_fields(available, rpp_interfaces::FieldLayout{}));
  EXPECT_FALSE(rpp_interfaces::validate_required_fields(rpp_interfaces::FieldLayout{}, required));

  auto incompatible_fields = required.fields();
  incompatible_fields[0].frameId = "map";
  EXPECT_FALSE(rpp_interfaces::validate_required_fields(
    available, rpp_interfaces::FieldLayout(incompatible_fields)));
  incompatible_fields = required.fields();
  incompatible_fields[0].size = 3;
  EXPECT_FALSE(rpp_interfaces::validate_required_fields(
    available, rpp_interfaces::FieldLayout(incompatible_fields)));
}

TEST(ControllerLifecycle, MissingHandoverStateFieldsDoNotRejectPidActivation)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  const auto & cascade_context = runtime.get_context().get_subcomponent_context("controller",
    "CascadeController2_comp");
  ControllerState handover;
  // Each PID warns about the missing snapshot description, then accepts fresh compute inputs.
  ASSERT_TRUE(cascade_context.activate(ControllerState::Const(std::move(handover))).success);
  const auto cascade = cascade_context.get_instance<rpp_interfaces::ControlLaw>();
  const auto description = cascade->getDesc().as_struct();
  EXPECT_EQ(description.horizonLength, 0u);
  EXPECT_TRUE(description.horizonOffsetsNs.empty());
  rpp::List<double> measured, references, previous_control;
  measured.resize(rpp_interfaces::ControlLawLayout(description).state().size());
  references.resize(rpp_interfaces::ControlLawLayout(description).reference_size());
  const auto [command, status] = cascade->compute(
    measured, references, all_axes_enabled, all_inputs_valid, previous_control, false, 0,
    500000000);
  EXPECT_EQ(status, ::schema::rpp_interfaces::CODE_OK);
  EXPECT_EQ(command.size(), 3u);
  ASSERT_TRUE(cascade_context.deactivate().success);
}

TEST(ControllerLifecycle, PidErrorHistoryCanBeRestoredAndRejectsNonFiniteState)
{
  rpp_control::PIDParameters parameters{};
  parameters.kp = 1.0F;
  parameters.ki = 2.0F;
  parameters.kd = 3.0F;
  rpp_control::PID source(parameters);

  EXPECT_NEAR(source.step(1.0F, 0.0F, 0.5F), 8.0F, 1e-5);
  const auto exported = source.error_info();

  rpp_control::PID restored(parameters);
  restored.restore_error_info(exported);
  const auto restored_state = restored.error_info();
  EXPECT_DOUBLE_EQ(restored_state.integral_error, exported.integral_error);
  EXPECT_DOUBLE_EQ(restored_state.previous_error, exported.previous_error);
  EXPECT_DOUBLE_EQ(restored_state.previous_derivative, exported.previous_derivative);

  auto invalid = exported;
  invalid.previous_derivative = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(restored.restore_error_info(invalid), std::invalid_argument);
}

TEST(ControlAuxiliaryState, OptionalErrorAndHorizonInputsDescribeEveryStoredValue)
{
  using namespace rpp_interfaces::contract;
  const rpp_interfaces::FieldLayout axes({
    {"axis", COORDINATE_EUCLIDEAN, QUANTITY_NORMALIZED, 2, "", "", "control"}});
  rpp::List<double> error, inputs;
  error.resize(2);
  error[0] = 1.0; error[1] = -2.0;
  inputs.resize(4);
  for (std::size_t index = 0; index < inputs.size(); ++index) {
    inputs[index] = 10.0 + index;
                                                                                            }
  rpp::List<std::uint64_t> offsets;
  offsets.resize(2);
  offsets[0] = 0; offsets[1] = 100000000;

  rpp_control::ControlAuxiliaryState auxiliary(1000000000);
  auxiliary.error(axes, error).integral(axes).horizon_inputs(axes, offsets, inputs);
  const auto snapshot = auxiliary.snapshot();
  EXPECT_EQ(snapshot.stampNs(), 1000000000);
  const auto state = snapshot.state();
  EXPECT_EQ(state.state().size(), 6u);
  const auto description = state.desc().as_struct();
  ASSERT_EQ(description.fields.size(), 3u);
  EXPECT_EQ(description.fields[0].name, "control.error.axis");
  EXPECT_EQ(description.fields[1].name, "control.horizonInput.0.axis");
  EXPECT_EQ(description.fields[2].name, "control.horizonInput.100000000.axis");
  const auto restored = rpp_control::ControlAuxiliaryState::read(
    "control.error", axes, state);
  ASSERT_TRUE(restored);
  EXPECT_EQ(*restored, (std::vector<double>{1.0, -2.0}));
  EXPECT_FALSE(rpp_control::ControlAuxiliaryState::read(
    "control.integral", axes, state));

  offsets[1] = 0;
  EXPECT_THROW(auxiliary.horizon_inputs(axes, offsets, inputs), std::invalid_argument);
  const auto after_failure = auxiliary.snapshot();
  const auto retained = after_failure.state();
  EXPECT_EQ(retained.state().size(), 6u);
  EXPECT_EQ(after_failure.stampNs(), 1000000000);
}

TEST(ControlAuxiliaryState, MergingKeepsTheOldestContributingTimestamp)
{
  using namespace rpp_interfaces::contract;
  const rpp_interfaces::FieldLayout axes({
    {"axis", COORDINATE_EUCLIDEAN, QUANTITY_NORMALIZED, 1, "", "", "control"}});
  rpp::List<double> values;
  values.resize(1);
  values[0] = 2.0;
  rpp_control::ControlAuxiliaryState newer(200);
  newer.error(axes, values);
  rpp_control::ControlAuxiliaryState older(100);
  older.integral(axes, values);

  rpp_control::ControlAuxiliaryState combined;
  combined.merge(newer.snapshot());
  combined.merge(older.snapshot());
  combined.merge(rpp_control::ControlAuxiliaryState{}.snapshot());
  const auto snapshot = combined.snapshot();
  EXPECT_EQ(snapshot.stampNs(), 100);
  const auto state = snapshot.state();
  EXPECT_EQ(state.state().size(), 2u);
}

TEST(ControllerLifecycle, CascadeSelectsStageHistoryFromSharedApplicationFields)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  const auto & context = runtime.get_context();
  const auto & cascade_context = context.get_subcomponent_context("controller",
    "CascadeController2_comp");
  auto cascade = cascade_context.get_instance<rpp_interfaces::ControlLaw>();
  using namespace rpp_interfaces::contract;
  const rpp_interfaces::FieldLayout pose({
    {"base.position", COORDINATE_EUCLIDEAN, QUANTITY_LINEAR_POSITION, 2, "map", "base_link",
      "base"},
    {"base.orientation", COORDINATE_CONTINUOUS_ANGLE, QUANTITY_ANGULAR_POSITION, 1, "map",
      "base_link", "base"}});
  const rpp_interfaces::FieldLayout twist({
    {"base.linearVelocity", COORDINATE_EUCLIDEAN, QUANTITY_LINEAR_VELOCITY, 2, "base_link",
      "base_link", "base"},
    {"base.angularVelocity", COORDINATE_EUCLIDEAN, QUANTITY_ANGULAR_VELOCITY, 1, "base_link",
      "base_link", "base"}});
  const rpp_interfaces::FieldLayout wrench({
    {"base.force", COORDINATE_EUCLIDEAN, QUANTITY_FORCE, 2, "base_link", "base_link", "base"},
    {"base.moment", COORDINATE_EUCLIDEAN, QUANTITY_MOMENT, 1, "base_link", "base_link", "base"}});
  const auto measured_fields = rpp_interfaces::merge_fields(pose, twist);
  const auto control_fields = rpp_interfaces::merge_fields(twist, wrench);
  ASSERT_TRUE(cascade_context.activate(application_state(
    measured_fields, control_fields, {1.5, 0.0, 0.0, 2.5, 0.0, 0.0})).success);

  rpp::List<double> reference, measured, previous;
  reference.resize(9);
  for (std::size_t i = 6; i < 9; ++i) {
    reference[i] = ::schema::rpp_interfaces::REFERENCE_INTERNAL;
                                                                                                   }
  measured.resize(6);
  previous.resize(0);
  auto result = cascade->compute(measured, reference, all_axes_enabled, all_inputs_valid, previous,
    false, 0, 500000000);
  ASSERT_EQ(std::get<1>(result), ::schema::rpp_interfaces::CODE_OK);
  ASSERT_EQ(std::get<0>(result).size(), 3u);
  // The pose integral produces 3 m/s. The restored twist integral makes 29 N.
  EXPECT_DOUBLE_EQ(std::get<0>(result)[0], 29.0);

  const auto auxiliary_snapshot = cascade->getAuxiliaryState();
  const auto auxiliary_state = auxiliary_snapshot.state();
  EXPECT_EQ(auxiliary_state.state().size(), 27u);
  EXPECT_EQ(auxiliary_snapshot.stampNs(), 0);
  rpp_control::ControlAuxiliaryState collected;
  collected.merge(auxiliary_snapshot);
  ControllerState resumed;
  auto resumed_state = resumed.state();
  auto measured_description = resumed_state.desc();
  measured_fields.write(measured_description);
  auto resumed_control = resumed.previousControl();
  auto command_description = resumed_control.desc();
  control_fields.write(command_description);
  auto resumed_auxiliary = resumed.auxiliaryState();
  collected.write_to(resumed_auxiliary);
  ASSERT_TRUE(cascade_context.deactivate().success);
  ASSERT_TRUE(cascade_context.activate(ControllerState::Const(std::move(resumed))).success);
  auto resumed_result = cascade->compute(
    measured, reference, all_axes_enabled, all_inputs_valid, previous, false, 500000000, 500000000);
  ASSERT_EQ(std::get<1>(resumed_result), ::schema::rpp_interfaces::CODE_OK);
  EXPECT_DOUBLE_EQ(std::get<0>(resumed_result)[0], 14.0);
  const auto updated_auxiliary = cascade->getAuxiliaryState();
  EXPECT_EQ(updated_auxiliary.stampNs(), 500000000);

  ASSERT_TRUE(cascade_context.deactivate().success);
  // A factory can provide the measured state and final controls without PID history.
  // Outer-stage controls absent from that payload start without tracking history.
  ASSERT_TRUE(cascade_context.activate(application_state(measured_fields, wrench)).success);
  auto cold_result = cascade->compute(measured, reference, all_axes_enabled, all_inputs_valid,
    previous, false, 0, 500000000);
  EXPECT_DOUBLE_EQ(std::get<0>(cold_result)[0], 0.0);
  ASSERT_TRUE(cascade_context.deactivate().success);
}

TEST(ReferenceGenerator, ZeroHorizonReturnsOneCurrentValuePerReference)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  const auto & context = runtime.get_context().get_subcomponent_context("reference",
    "InterpolatingReference_comp");
  const auto generator = context.get_instance<rpp_interfaces::ReferenceGenerator>();
  const auto description = generator->getDesc().as_struct();
  const rpp_interfaces::FieldLayout state_layout(description.state.fields);
  const rpp_interfaces::FieldLayout reference_layout(description.reference.fields);
  // Reference-only selectors must not become measured-state requirements.
  EXPECT_EQ(state_layout.size(), 6u);
  EXPECT_EQ(reference_layout.size(), 9u);
  EXPECT_EQ(description.targetMask.size(), reference_layout.fields().size());
  const auto route = rpp_interfaces::field_route(reference_layout, state_layout);
  rpp::List<double> measured;
  measured.resize(state_layout.size());
  measured[0] = 3.0;
  measured[1] = 4.0;
  rpp::List<std::uint64_t> offsets;
  ASSERT_EQ(generator->bind(offsets, 0), ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(context.activate().success);
  auto [values, status] = generator->sample(measured, 0);
  ASSERT_EQ(status, ::schema::rpp_interfaces::CODE_OK);
  ASSERT_EQ(values.size(), reference_layout.size());
  for (std::size_t scalar = 0; scalar < route.size(); ++scalar) {
    EXPECT_DOUBLE_EQ(values[route[scalar]], measured[scalar]);
  }
  for (std::size_t scalar = state_layout.size(); scalar < reference_layout.size(); ++scalar) {
    EXPECT_DOUBLE_EQ(values[scalar], 0.0);
  }
  ASSERT_TRUE(context.deactivate().success);
}

TEST(ReferenceGenerator, SharedAndPerScalarHorizonsProduceReferenceMajorValues)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  const auto & context = runtime.get_context().get_subcomponent_context("reference",
    "InterpolatingReference_comp");
  const auto generator = context.get_instance<rpp_interfaces::ReferenceGenerator>();
  const auto description = generator->getDesc().as_struct();
  const auto state_width = rpp_interfaces::FieldLayout(description.state.fields).size();
  const auto reference_width = rpp_interfaces::FieldLayout(description.reference.fields).size();
  ASSERT_GE(reference_width, 2u);
  rpp::List<double> measured, waypoints;
  measured.resize(state_width);
  waypoints.resize(2 * reference_width);
  waypoints[reference_width] = 4.0;
  waypoints[reference_width + 1] = 8.0;
  rpp::List<std::uint64_t> times;
  times.resize(2);
  times[0] = 0;
  times[1] = 1000000000;
  constexpr std::uint32_t horizon_length = 3;

  for (const bool per_scalar : {false, true}) {
    SCOPED_TRACE(per_scalar);
    rpp::List<std::uint64_t> offsets;
    offsets.resize(per_scalar ? reference_width * horizon_length : horizon_length);
    for (std::size_t scalar = 0; scalar < (per_scalar ? reference_width : 1); ++scalar) {
      offsets[scalar * horizon_length] = 0;
      offsets[scalar * horizon_length + 1] = 250000000;
      offsets[scalar * horizon_length + 2] = 500000000;
    }
    if (per_scalar) {
      offsets[horizon_length + 1] = 500000000;
      offsets[horizon_length + 2] = 1000000000;
    }
    ASSERT_EQ(generator->bind(offsets, horizon_length), ::schema::rpp_interfaces::CODE_OK);
    ASSERT_EQ(generator->setTrajectory(waypoints, times, update_all_values),
      ::schema::rpp_interfaces::CODE_OK);
    ASSERT_TRUE(context.activate().success);
    auto [values, status] = generator->sample(measured, 0);
    ASSERT_EQ(status, ::schema::rpp_interfaces::CODE_OK);
    ASSERT_EQ(values.size(), reference_width * horizon_length);
    EXPECT_DOUBLE_EQ(values[0], 0.0);
    EXPECT_DOUBLE_EQ(values[1], 1.0);
    EXPECT_DOUBLE_EQ(values[2], 2.0);
    EXPECT_DOUBLE_EQ(values[3], 0.0);
    EXPECT_DOUBLE_EQ(values[4], per_scalar ? 4.0 : 2.0);
    EXPECT_DOUBLE_EQ(values[5], per_scalar ? 8.0 : 4.0);
    ASSERT_TRUE(context.deactivate().success);
  }
}

TEST(ReferenceGenerator, FailedRebindingPreservesThePreviousHorizon)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  const auto & context = runtime.get_context().get_subcomponent_context("reference",
    "InterpolatingReference_comp");
  const auto generator = context.get_instance<rpp_interfaces::ReferenceGenerator>();
  const auto description = generator->getDesc().as_struct();
  rpp::List<std::uint64_t> offsets;
  ASSERT_EQ(generator->bind(offsets, 0), ::schema::rpp_interfaces::CODE_OK);
  offsets.resize(2);
  offsets[0] = 0;
  offsets[1] = 10;
  EXPECT_EQ(generator->bind(offsets, 0), ::schema::rpp_interfaces::CODE_INVALID_INPUT);
  EXPECT_EQ(generator->bind(offsets, 3), ::schema::rpp_interfaces::CODE_INVALID_INPUT);
  ASSERT_TRUE(context.activate().success);
  EXPECT_EQ(generator->bind(offsets, 0), ::schema::rpp_interfaces::CODE_INVALID_INPUT);
  rpp::List<double> measured;
  measured.resize(rpp_interfaces::FieldLayout(description.state.fields).size());
  auto [values, status] = generator->sample(measured, 0);
  ASSERT_EQ(status, ::schema::rpp_interfaces::CODE_OK);
  EXPECT_EQ(values.size(), rpp_interfaces::FieldLayout(description.reference.fields).size());
  ASSERT_TRUE(context.deactivate().success);
}

TEST(ReferenceGenerator, ChecksOverflowAcrossEveryReferenceHorizon)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  const auto & context = runtime.get_context().get_subcomponent_context("reference",
    "InterpolatingReference_comp");
  const auto generator = context.get_instance<rpp_interfaces::ReferenceGenerator>();
  const auto description = generator->getDesc().as_struct();
  const auto reference_width = rpp_interfaces::FieldLayout(description.reference.fields).size();
  ASSERT_GE(reference_width, 2u);
  rpp::List<std::uint64_t> offsets;
  offsets.resize(reference_width * 2);
  for (std::size_t scalar = 0; scalar < reference_width; ++scalar) {
    offsets[scalar * 2] = 0;
    offsets[scalar * 2 + 1] = 10;
  }
  offsets[1] = std::numeric_limits<std::uint64_t>::max();
  ASSERT_EQ(generator->bind(offsets, 2), ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(context.activate().success);
  rpp::List<double> measured;
  measured.resize(rpp_interfaces::FieldLayout(description.state.fields).size());
  const auto result = generator->sample(measured, 1);
  EXPECT_EQ(std::get<1>(result), ::schema::rpp_interfaces::CODE_INVALID_INPUT);
  EXPECT_TRUE(std::get<0>(result).empty());
  ASSERT_TRUE(context.deactivate().success);
}

TEST(ReferenceGenerator, RebindingPreservesActiveStateAndTheAcceptedGoal)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  const auto & context = runtime.get_context().get_subcomponent_context("reference",
    "InterpolatingReference_comp");
  const auto generator = context.get_instance<rpp_interfaces::ReferenceGenerator>();
  const auto description = generator->getDesc().as_struct();
  const auto width = rpp_interfaces::FieldLayout(description.reference.fields).size();
  rpp::List<double> measured, goal;
  measured.resize(rpp_interfaces::FieldLayout(description.state.fields).size());
  goal.resize(width);
  goal[0] = 2.0;
  rpp::List<std::uint64_t> offsets;
  ASSERT_EQ(generator->bind(offsets, 0), ::schema::rpp_interfaces::CODE_OK);
  ASSERT_EQ(generator->setPoint(goal, update_all_values), ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(context.activate().success);
  offsets.resize(2);
  offsets[0] = 0;
  offsets[1] = 10;
  ASSERT_EQ(generator->bind(offsets, 2), ::schema::rpp_interfaces::CODE_OK);
  EXPECT_TRUE(context.is_active());
  const auto [values, status] = generator->sample(measured, 0);
  ASSERT_EQ(status, ::schema::rpp_interfaces::CODE_OK);
  ASSERT_EQ(values.size(), width * 2);
  EXPECT_DOUBLE_EQ(values[0], 2.0);
  EXPECT_DOUBLE_EQ(values[1], 2.0);
  ASSERT_TRUE(context.deactivate().success);
}

TEST(StateProvider, BuffersSamplesWithoutOverwritingRetainedViews)
{
  auto runtime = build_control_runtime();
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  auto provider = runtime.get_context()
    .get_subcomponent_context("state_provider", "StateProvider_comp")
    .get_instance<rpp_interfaces::StateProvider>();
  EXPECT_EQ(script.read_state(), ::schema::rpp_interfaces::CODE_NOT_READY);

  rpp::List<double> state;
  state.resize(script.state_layout().size());
  state[0] = 1.0;
  ASSERT_EQ(script.set_state(state, 100), ::schema::rpp_interfaces::CODE_OK);
  const auto [first, first_time, first_status] = provider->getState();
  ASSERT_EQ(first_status, ::schema::rpp_interfaces::CODE_OK);
  EXPECT_EQ(first_time, 100u);
  EXPECT_DOUBLE_EQ(first[0], 1.0);

  state[0] = 2.0;
  ASSERT_EQ(script.set_state(state, 200), ::schema::rpp_interfaces::CODE_OK);
  const auto [second, second_time, second_status] = provider->getState();
  ASSERT_EQ(second_status, ::schema::rpp_interfaces::CODE_OK);
  EXPECT_EQ(second_time, 200u);
  EXPECT_DOUBLE_EQ(first[0], 1.0);
  ASSERT_EQ(script.read_state(), ::schema::rpp_interfaces::CODE_OK);
  EXPECT_DOUBLE_EQ(script.state()[0], 2.0);

  state[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(script.set_state(state, 300), ::schema::rpp_interfaces::CODE_INVALID_INPUT);
  const auto [unchanged, unchanged_time, unchanged_status] = provider->getState();
  ASSERT_EQ(unchanged_status, ::schema::rpp_interfaces::CODE_OK);
  EXPECT_EQ(unchanged_time, 200u);
  EXPECT_DOUBLE_EQ(unchanged[0], 2.0);

  state[0] = 3.0;
  ASSERT_EQ(script.set_state(state, 300), ::schema::rpp_interfaces::CODE_OK);
  const auto [third, third_time, third_status] = provider->getState();
  ASSERT_EQ(third_status, ::schema::rpp_interfaces::CODE_OK);
  EXPECT_EQ(third_time, 300u);
  EXPECT_EQ(script.set_state(state, 400), ::schema::rpp_interfaces::CODE_BUDGET_EXCEEDED);
  EXPECT_DOUBLE_EQ(first[0], 1.0);
  EXPECT_DOUBLE_EQ(second[0], 2.0);
  EXPECT_DOUBLE_EQ(third[0], 3.0);

  script.deactivate();
  const auto [empty, empty_time, empty_status] = provider->getState();
  EXPECT_EQ(empty_status, ::schema::rpp_interfaces::CODE_NOT_READY);
  EXPECT_TRUE(empty.empty());
  EXPECT_EQ(empty_time, 0u);
  EXPECT_DOUBLE_EQ(first[0], 1.0);
}

TEST(ControlScriptLifecycle, ControllerDeactivationLeavesOtherComponentsActive)
{
  auto runtime = build_control_runtime();
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  const auto & context = runtime.get_context();
  const auto & controller = context.get_subcomponent_context("controller",
    "CascadeController2_comp");
  const auto & allocator = context.get_subcomponent_context("allocator", "WaterjetAllocator_comp");
  const auto & generator = context.get_subcomponent_context("reference",
    "InterpolatingReference_comp");
  rpp::List<double> goal;
  goal.resize(script.goal_layout("InterpolatingReference_comp").size());
  ASSERT_EQ(script.set_point("InterpolatingReference_comp", goal, update_all_values, goal_deadline),
    ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(script.step(0, 50000000).success());
  EXPECT_TRUE(script.has_previous_control());
  ASSERT_TRUE(controller.deactivate().success);
  script.rebuild_active_routes({});
  EXPECT_TRUE(allocator.is_active());
  EXPECT_TRUE(generator.is_active());
  EXPECT_FALSE(script.step(50000000, 50000000).success());
  EXPECT_FALSE(script.has_previous_control());
  for (std::size_t index = 0; index < script.actuators().size(); ++index) {
    EXPECT_DOUBLE_EQ(script.actuators()[index], 0.0);
  }
  ASSERT_TRUE(controller.activate().success);
  script.rebuild_active_routes({});
  EXPECT_TRUE(script.has_active_goal());
  EXPECT_TRUE(script.step(100000000, 50000000).success());
  EXPECT_TRUE(script.has_previous_control());
  script.deactivate();
}

TEST(ControlScriptLifecycle, GeneratorDeactivationPreservesDependentActivation)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  const auto & context = runtime.get_context();
  const auto & controller = context.get_subcomponent_context("controller",
    "CascadeController2_comp");
  const auto & allocator = context.get_subcomponent_context("allocator", "WaterjetAllocator_comp");
  const auto & generator = context.get_subcomponent_context("reference",
    "InterpolatingReference_comp");
  rpp::List<double> goal;
  goal.resize(script.goal_layout("InterpolatingReference_comp").size());
  ASSERT_EQ(script.set_point("InterpolatingReference_comp", goal, update_all_values, goal_deadline),
    ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(script.step(0, 50000000).success());
  ASSERT_TRUE(generator.deactivate().success);
  script.rebuild_active_routes({});
  EXPECT_TRUE(controller.is_active());
  EXPECT_TRUE(allocator.is_active());
  EXPECT_FALSE(script.step(50000000, 50000000).success());
  ASSERT_TRUE(generator.activate().success);
  script.rebuild_active_routes({});
  EXPECT_TRUE(script.step(100000000, 50000000).success());
  script.deactivate();
}

TEST(ControlScriptMapping, AcceptsExplicitControlValuesBeforeControllerComputation)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  script.commands()[0] = 100.0;
  script.commands()[1] = 0.0;
  script.commands()[2] = 0.0;
  const auto status = script.commandMap(50000000);
  EXPECT_TRUE(rpp_control::control_success(status));
  EXPECT_GT(static_cast<double>(script.actuators()[0]), 0.0);
  EXPECT_TRUE(script.has_previous_control());
  script.deactivate();
}

TEST(ControlScriptStep, AdvancesStagedPathsAndTrajectories)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  const auto goal_width = script.goal_layout("InterpolatingReference_comp").size();
  rpp::List<double> waypoints;
  waypoints.resize(2 * goal_width);
  waypoints[goal_width] = 2.0;

  ASSERT_EQ(script.set_path("InterpolatingReference_comp", waypoints, update_all_values,
    goal_deadline), ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(script.step(0, 500000000).success());
  EXPECT_DOUBLE_EQ(script.references()[0], 0.0);
  // The path spans one second. Repeated steps must advance it without restaging.
  ASSERT_TRUE(script.step(500000000, 500000000).success());
  EXPECT_DOUBLE_EQ(script.references()[0], 1.0);

  rpp::List<std::uint64_t> arrival_times;
  arrival_times.resize(2);
  arrival_times[0] = 0;
  arrival_times[1] = 2000000000;
  ASSERT_EQ(script.set_trajectory("InterpolatingReference_comp", waypoints, arrival_times,
    update_all_values, goal_deadline), ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(script.step(1000000000, 500000000).success());
  EXPECT_DOUBLE_EQ(script.references()[0], 0.0);
  ASSERT_TRUE(script.step(1500000000, 500000000).success());
  EXPECT_DOUBLE_EQ(script.references()[0], 0.5);
  script.deactivate();
}

TEST(ControlScriptGoals, PathHeartbeatPreservesProgressAndCannotReviveExpiry)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  const std::string name = "InterpolatingReference_comp";
  EXPECT_TRUE(script.goal_features(name) & rpp_interfaces::contract::GENERATOR_SET_PATH);
  EXPECT_EQ(script.goal_features("unknown"), 0u);
  const auto width = script.goal_layout(name).size();
  rpp::List<double> points;
  points.resize(2 * width);
  points[width] = 2.0;
  ASSERT_EQ(script.set_path(name, points, update_all_values, 1000000000),
    ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(script.step(0, 50000000).success());
  ASSERT_EQ(script.refresh_goal(name, update_all_values, 500000000, 2000000000),
    ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(script.step(500000000, 50000000).success());
  EXPECT_DOUBLE_EQ(script.references()[0], 1.0);
  ASSERT_TRUE(script.step(1000000000, 50000000).success());
  EXPECT_DOUBLE_EQ(script.references()[0], 2.0);
  EXPECT_EQ(script.refresh_goal(name, update_all_values, 2000000000, 3000000000),
    ::schema::rpp_interfaces::CODE_STALE_INPUT);
  script.deactivate();
}

TEST(MissionSupervision, OnlyTheSelectedMonitorDecidesArrival)
{
  auto runtime = build_control_runtime();
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  const std::string generator = "InterpolatingReference_comp";
  rpp::List<double> goal;
  goal.resize(script.goal_layout(generator).size());
  goal[0] = 2.0;
  ASSERT_EQ(script.set_point(generator, goal, update_all_values, goal_deadline),
    ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(script.step(0, 50000000).success());
  EXPECT_FALSE(script.has_goal_monitor("progress_monitor"));
  ASSERT_EQ(script.prepare_mission("goal_monitor", generator, goal, true),
    ::schema::rpp_interfaces::CODE_OK);
  const auto strict = script.supervise_mission(0);
  EXPECT_TRUE(strict.ok);
  EXPECT_FALSE(strict.reached);
  // Both pose monitors remain active; selecting the loose one changes only arrival.
  ASSERT_EQ(script.prepare_mission("loose_goal_monitor", generator, goal, false),
    ::schema::rpp_interfaces::CODE_OK);
  const auto loose = script.supervise_mission(1);
  EXPECT_TRUE(loose.ok);
  EXPECT_TRUE(loose.reached);
  // A stop vote takes precedence even when the selected arrival test would pass.
  const auto vetoed = script.supervise_mission(11000000000);
  EXPECT_FALSE(vetoed.ok);
  EXPECT_FALSE(vetoed.reached);
  script.deactivate();
}

TEST(MissionSupervision, ReplanningPreservesTheProgressWatchdog)
{
  auto runtime = build_control_runtime();
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  const std::string generator = "InterpolatingReference_comp";
  rpp::List<double> goal;
  goal.resize(script.goal_layout(generator).size());
  goal[0] = 2.0;
  ASSERT_EQ(script.set_point(generator, goal, update_all_values, goal_deadline),
    ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(script.step(0, 50000000).success());
  ASSERT_EQ(script.prepare_mission("goal_monitor", generator, goal, true),
    ::schema::rpp_interfaces::CODE_OK);
  EXPECT_TRUE(script.supervise_mission(0).ok);
  goal[0] = 3.0;
  ASSERT_EQ(script.prepare_mission("goal_monitor", generator, goal, false),
    ::schema::rpp_interfaces::CODE_OK);
  EXPECT_TRUE(script.supervise_mission(9000000000).ok);
  const auto stopped = script.supervise_mission(11000000000);
  EXPECT_FALSE(stopped.ok);
  EXPECT_FALSE(stopped.reached);
  EXPECT_EQ(stopped.status, ::schema::rpp_interfaces::CODE_OK);
  EXPECT_EQ(stopped.source, "progress_monitor");
  EXPECT_NE(stopped.message.find("movement"), std::string::npos);
  script.visit_mission_monitor_status(
    [&](const auto & name, const auto &, const auto & status, bool) {
      if (name == "progress_monitor") {EXPECT_EQ(status.message, stopped.message);}
    });
  // A new mission explicitly clears the stopped monitor's history.
  ASSERT_EQ(script.prepare_mission("goal_monitor", generator, goal, true),
    ::schema::rpp_interfaces::CODE_OK);
  const auto restarted = script.supervise_mission(11000000000);
  EXPECT_TRUE(restarted.ok);
  EXPECT_TRUE(restarted.message.empty());
  script.deactivate();
}

TEST(ControlScriptGoals, CancelClearsGoalsWithoutDeactivatingPlugins)
{
  auto runtime = build_control_runtime();
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  rpp::List<double> goal;
  goal.resize(script.goal_layout("InterpolatingReference_comp").size());
  ASSERT_EQ(script.set_point("InterpolatingReference_comp", goal, update_all_values, goal_deadline),
    ::schema::rpp_interfaces::CODE_OK);
  EXPECT_TRUE(script.has_active_goal());

  script.cancel();
  EXPECT_FALSE(script.has_active_goal());
  EXPECT_TRUE(script.all_active());
  EXPECT_TRUE(script.step(0, 50000000).success());
  script.deactivate();
}

TEST(ControlScriptGoals, MaskedValuesKeepIndependentDeadlinesAndLifecycleOwnership)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  rpp::List<double> goal;
  goal.resize(script.goal_layout("InterpolatingReference_comp").size());
  goal[3] = 1.0;
  for (std::size_t axis = 6; axis < 9; ++axis) {
    goal[axis] = ::schema::rpp_interfaces::REFERENCE_EXTERNAL;
                                                                                                          }
  ASSERT_EQ(script.set_point("InterpolatingReference_comp", goal, update_all_values, 10000000000),
    ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(script.step(0, 500000000).success());

  rpp::List<bool> mask;
  mask.resize(goal.size());
  mask[5] = true;
  goal[3] = 999.0;  // Not selected: the forward target must stay at 1 m/s.
  goal[0] = std::numeric_limits<double>::quiet_NaN();  // Ignored by this masked update.
  goal[5] = 1.0;
  ASSERT_EQ(script.set_point("InterpolatingReference_comp", goal, mask, 1000000000),
    ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(script.step(500000000, 500000000).success());
  EXPECT_DOUBLE_EQ(script.references()[3], 1.0);
  EXPECT_NE(static_cast<double>(script.commands()[2]), 0.0);

  ASSERT_TRUE(script.step(1000000000, 500000000).success());
  EXPECT_DOUBLE_EQ(script.commands()[2], 0.0);
  EXPECT_NE(static_cast<double>(script.commands()[0]), 0.0);
  EXPECT_TRUE(script.has_active_goal());
  EXPECT_TRUE(script.all_active());

  ASSERT_TRUE(script.step(10000000000, 500000000).success());
  EXPECT_FALSE(script.has_active_goal());
  EXPECT_TRUE(script.all_active());
  for (std::size_t axis = 0; axis < script.actuators().size(); ++axis) {
    EXPECT_DOUBLE_EQ(script.actuators()[axis], 0.0);
                                                                                                                        }
  script.deactivate();
}

TEST(ControlScriptGoals, UpdatingOneValuePreservesOtherValuesTrajectoryOrigins)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  const auto width = script.goal_layout("InterpolatingReference_comp").size();
  rpp::List<double> waypoints;
  waypoints.resize(2 * width);
  waypoints[width + 3] = 2.0;
  rpp::List<std::uint64_t> times;
  times.resize(2);
  times[1] = 2000000000;
  ASSERT_EQ(script.set_trajectory("InterpolatingReference_comp", waypoints, times,
    update_all_values, goal_deadline),
    ::schema::rpp_interfaces::CODE_OK);
  ASSERT_EQ(script.sample(0), ::schema::rpp_interfaces::CODE_OK);
  ASSERT_EQ(script.sample(500000000), ::schema::rpp_interfaces::CODE_OK);
  EXPECT_DOUBLE_EQ(script.references()[3], 0.5);

  rpp::List<double> point;
  point.resize(width);
  point[5] = 1.0;
  rpp::List<bool> mask;
  mask.resize(width);
  mask[5] = true;
  ASSERT_EQ(script.set_point("InterpolatingReference_comp", point, mask, goal_deadline),
    ::schema::rpp_interfaces::CODE_OK);
  ASSERT_EQ(script.sample(1000000000), ::schema::rpp_interfaces::CODE_OK);
  EXPECT_DOUBLE_EQ(script.references()[3], 1.0);
  EXPECT_DOUBLE_EQ(script.references()[5], 1.0);
  script.deactivate();
}

TEST(ControlScriptGoals, RouteRebuildKeepsOriginalDeadlineAndRejectsMalformedUpdates)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  rpp::List<double> point;
  point.resize(script.goal_layout("InterpolatingReference_comp").size());
  ASSERT_EQ(script.set_point("InterpolatingReference_comp", point, update_all_values, 1000000000),
    ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(script.step(0, 500000000).success());
  script.rebuild_active_routes({});
  rpp::List<bool> malformed;
  malformed.resize(1);
  EXPECT_EQ(script.set_point("InterpolatingReference_comp", point, malformed, goal_deadline),
    ::schema::rpp_interfaces::CODE_INVALID_INPUT);
  EXPECT_EQ(script.set_point("unknown", point, update_all_values, goal_deadline),
    ::schema::rpp_interfaces::CODE_INVALID_INPUT);
  ASSERT_TRUE(script.step(500000000, 500000000).success());
  EXPECT_TRUE(script.has_active_goal());
  ASSERT_TRUE(script.step(1000000000, 500000000).success());
  EXPECT_FALSE(script.has_active_goal());
  EXPECT_TRUE(script.all_active());
  script.deactivate();
}

TEST(ControlScriptTime, AcceptsUnsignedTimestampsAndReportsBackwardSamples)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  rpp::List<double> goal;
  goal.resize(script.goal_layout("InterpolatingReference_comp").size());
  ASSERT_EQ(script.set_point("InterpolatingReference_comp", goal, update_all_values, goal_deadline),
    ::schema::rpp_interfaces::CODE_OK);

  constexpr std::uint64_t time_ns = (std::uint64_t{1} << 63) + 100;
  ASSERT_TRUE(script.step(time_ns, 500000000).success());
  const auto snapshot = script.controller_auxiliary_state().snapshot();
  EXPECT_EQ(snapshot.stampNs(), time_ns);
  EXPECT_FALSE(snapshot.state().state().empty());
  const auto allocator_snapshot = script.allocator_auxiliary_state().snapshot();
  EXPECT_TRUE(allocator_snapshot.state().state().empty());

  const auto stale_step = script.step(time_ns - 1, 500000000);
  EXPECT_FALSE(stale_step.success());
  EXPECT_EQ(stale_step.status, ::schema::rpp_interfaces::CODE_STALE_INPUT);
  script.deactivate();
}

TEST(ControlScriptTime, AcceptsTrajectoryDurationsAcrossTheUnsignedRange)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  const auto goal_width = script.goal_layout("InterpolatingReference_comp").size();
  rpp::List<double> waypoints;
  waypoints.resize(2 * goal_width);
  waypoints[goal_width] = 2.0;
  rpp::List<std::uint64_t> arrival_times;
  arrival_times.resize(2);
  arrival_times[0] = 0;
  arrival_times[1] = std::numeric_limits<std::uint64_t>::max();

  ASSERT_EQ(script.set_trajectory("InterpolatingReference_comp", waypoints, arrival_times,
    update_all_values, goal_deadline), ::schema::rpp_interfaces::CODE_OK);
  ASSERT_EQ(script.sample(0), ::schema::rpp_interfaces::CODE_OK);
  ASSERT_EQ(script.sample(std::numeric_limits<std::uint64_t>::max() - 1),
    ::schema::rpp_interfaces::CODE_OK);
  EXPECT_NEAR(script.references()[0], 2.0, 1e-12);
  script.deactivate();
}

TEST(ControlScriptTime, RejectsHorizonOverflowBeforeAddingOffsets)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  const auto & generator_context = runtime.get_context().get_subcomponent_context("reference",
    "InterpolatingReference_comp");
  const auto generator = generator_context.get_instance<rpp_interfaces::ReferenceGenerator>();
  rpp::List<std::uint64_t> offsets;
  offsets.resize(2);
  offsets[0] = 0;
  offsets[1] = 10;
  ASSERT_EQ(generator->bind(offsets, 2), ::schema::rpp_interfaces::CODE_OK);
  const auto description = generator->getDesc().as_struct();
  rpp::List<double> state;
  state.resize(rpp_interfaces::FieldLayout(description.state.fields).size());
  rpp::List<double> goal;
  goal.resize(rpp_interfaces::FieldLayout(description.reference.fields).size());
  ASSERT_EQ(generator->setPoint(goal, update_all_values), ::schema::rpp_interfaces::CODE_OK);
  ASSERT_TRUE(generator_context.activate().success);

  constexpr auto last_time = std::numeric_limits<std::uint64_t>::max();
  EXPECT_EQ(std::get<1>(generator->sample(state, last_time - 10)),
    ::schema::rpp_interfaces::CODE_OK);
  EXPECT_EQ(std::get<1>(generator->sample(state, last_time - 9)),
    ::schema::rpp_interfaces::CODE_INVALID_INPUT);
  EXPECT_EQ(std::get<1>(generator->sample(state, last_time - 11)),
    ::schema::rpp_interfaces::CODE_STALE_INPUT);
  ASSERT_TRUE(generator_context.deactivate().success);
}

TEST(ControlScriptStep, ReportsTheFailingStageAndKeepsSafeOutput)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  rpp::List<double> goal;
  goal.resize(script.goal_layout("InterpolatingReference_comp").size());

  EXPECT_FALSE(script.has_previous_control());
  ASSERT_EQ(script.set_point("InterpolatingReference_comp", goal, update_all_values, goal_deadline),
    ::schema::rpp_interfaces::CODE_OK);
  const auto successful_step = script.step(0, 500000000);
  ASSERT_TRUE(successful_step.success());
  EXPECT_TRUE(script.has_previous_control());
  const auto result = script.step(500000000, 0);
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.failed_stage, rpp_control::ControlScript::StepStage::Compute);
  EXPECT_EQ(result.status, ::schema::rpp_interfaces::CODE_INVALID_INPUT);
  EXPECT_FALSE(script.has_previous_control());
  for (std::size_t index = 0; index < script.actuators().size(); ++index) {
    EXPECT_DOUBLE_EQ(script.actuators()[index], 0.0);
  }
  script.deactivate();
}

TEST(ControlScriptFeedback, UsesTheMapperEstimateAfterActuatorLimits)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  // Request external surge velocity while disabling the other axes.
  script.references()[3] = 1.0;
  script.references()[6] = ::schema::rpp_interfaces::REFERENCE_EXTERNAL;
  script.set_actuator_limit(0, 0.0, 0.0);

  constexpr std::uint64_t dt_ns = 500000000;
  EXPECT_FALSE(script.has_previous_control());
  ASSERT_EQ(script.compute(0, dt_ns), ::schema::rpp_interfaces::CODE_OK);
  EXPECT_DOUBLE_EQ(script.commands()[0], 8.0);
  ASSERT_EQ(script.commandMap(dt_ns), ::schema::rpp_interfaces::CODE_LIMITED);
  EXPECT_TRUE(script.has_previous_control());
  EXPECT_DOUBLE_EQ(script.realized()[0], 0.0);

  // The mapper estimates zero force after the thrust clamp. Tracking that
  // difference corrects the integral; without feedback the next output is 3 N.
  ASSERT_EQ(script.compute(dt_ns, dt_ns), ::schema::rpp_interfaces::CODE_OK);
  EXPECT_DOUBLE_EQ(script.commands()[0], -5.0);

  script.reset();
  EXPECT_FALSE(script.has_previous_control());
  ASSERT_EQ(script.compute(2 * dt_ns, dt_ns), ::schema::rpp_interfaces::CODE_OK);
  EXPECT_DOUBLE_EQ(script.commands()[0], 8.0);
  ASSERT_EQ(script.commandMap(dt_ns), ::schema::rpp_interfaces::CODE_LIMITED);
  script.clear_outputs();
  EXPECT_FALSE(script.has_previous_control());
  ASSERT_EQ(script.compute(3 * dt_ns, dt_ns), ::schema::rpp_interfaces::CODE_OK);
  EXPECT_DOUBLE_EQ(script.commands()[0], 3.0);
  script.deactivate();
}

TEST(ControlScriptBindings, PreservesApplicationCapsWhenRebuildingActiveRoutes)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  script.set_actuator_limit(0, 0.0, 0.0);

  script.rebuild_active_routes({});
  rpp::List<double> goal;
  goal.resize(script.goal_layout("InterpolatingReference_comp").size());
  goal[3] = 1.0;
  goal[6] = ::schema::rpp_interfaces::REFERENCE_EXTERNAL;
  ASSERT_EQ(script.set_point("InterpolatingReference_comp", goal, update_all_values, goal_deadline),
    ::schema::rpp_interfaces::CODE_OK);
  const auto result = script.step(0, 500000000);
  ASSERT_TRUE(result.success());
  EXPECT_EQ(result.status, ::schema::rpp_interfaces::CODE_LIMITED);
  EXPECT_DOUBLE_EQ(script.actuators()[0], 0.0);
  EXPECT_DOUBLE_EQ(script.realized()[0], 0.0);

  script.deactivate();
  EXPECT_FALSE(script.all_active());
}

TEST(ControlScriptBindings, RoutesAxisFlagsAndPreservesThemWhenRebuildingActiveRoutes)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  rpp_control::ControlScript script(runtime);
  script.configure();
  script.activate();
  ASSERT_EQ(script.enabled_axes().size(), 3u);
  // Disable only sideways force within the two-scalar base.force field.
  script.enabled_axes()[1] = false;
  script.references()[3] = 1.0;
  script.references()[4] = 2.0;
  script.references()[6] = ::schema::rpp_interfaces::REFERENCE_EXTERNAL;
  script.references()[7] = ::schema::rpp_interfaces::REFERENCE_EXTERNAL;

  ASSERT_EQ(script.compute(0, 500000000), ::schema::rpp_interfaces::CODE_OK);
  EXPECT_DOUBLE_EQ(script.commands()[0], 8.0);
  EXPECT_DOUBLE_EQ(script.commands()[1], 0.0);

  script.rebuild_active_routes({});
  EXPECT_TRUE(script.enabled_axes()[0]);
  EXPECT_FALSE(script.enabled_axes()[1]);
  EXPECT_TRUE(script.enabled_axes()[2]);

  // Invalid mask widths fail at the input boundary.
  script.enabled_axes().resize(2);
  EXPECT_EQ(script.compute(500000000, 500000000), ::schema::rpp_interfaces::CODE_INVALID_INPUT);
  script.deactivate();
}

TEST(ControllerLifecycle, DisabledPidAxesClearHistoryAndRestartWithoutDisabledFeedback)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  const auto & cascade_context = runtime.get_context().get_subcomponent_context("controller",
    "CascadeController2_comp");
  ASSERT_TRUE(cascade_context.activate().success);
  auto cascade = cascade_context.get_instance<rpp_interfaces::ControlLaw>();

  rpp::List<double> reference, measured, applied;
  reference.resize(9);
  measured.resize(6);
  applied.resize(3);
  reference[3] = 1.0;
  reference[4] = 2.0;
  reference[5] = 1.5;
  measured[5] = 0.5;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    reference[6 + axis] = ::schema::rpp_interfaces::REFERENCE_EXTERNAL;
  }

  constexpr std::uint64_t dt_ns = 500000000;
  {
    auto [command, status] = cascade->compute(
      measured, reference, all_axes_enabled, all_inputs_valid, applied, false, 0, dt_ns);
    ASSERT_EQ(status, ::schema::rpp_interfaces::CODE_OK);
    EXPECT_DOUBLE_EQ(command[0], 8.0);
    EXPECT_DOUBLE_EQ(command[1], 16.0);
    EXPECT_DOUBLE_EQ(command[2], 8.0);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      applied[axis] = command[axis];
    }
  }

  rpp::List<bool> enabled_axes;
  enabled_axes.resize(3);
  enabled_axes[0] = true;
  enabled_axes[1] = true;
  enabled_axes[2] = false;
  applied[2] = 1000.0;
  {
    auto [command, status] = cascade->compute(
      measured, reference, enabled_axes, all_inputs_valid, applied, true, dt_ns, dt_ns);
    ASSERT_EQ(status, ::schema::rpp_interfaces::CODE_OK);
    EXPECT_DOUBLE_EQ(command[0], 3.0);
    EXPECT_DOUBLE_EQ(command[1], 6.0);
    EXPECT_DOUBLE_EQ(command[2], 0.0);
    for (std::size_t axis = 0; axis < 2; ++axis) {
      applied[axis] = command[axis];
    }
  }

  auto history_fields =
    rpp_interfaces::ControlLawLayout(cascade->getDesc().as_struct()).control().fields();
  for (auto & field : history_fields) {
    field.quantity = rpp_interfaces::contract::QUANTITY_NORMALIZED;
                                                                                                      }
  const rpp_interfaces::FieldLayout history_axes(std::move(history_fields));
  const auto snapshot = cascade->getAuxiliaryState();
  for (const auto * history_name : {
    "control.integral", "control.previousError", "control.previousDerivative"})
  {
    const auto history = rpp_control::ControlAuxiliaryState::read(history_name, history_axes,
      snapshot.state());
    ASSERT_TRUE(history);
    EXPECT_DOUBLE_EQ((*history)[2], 0.0);
  }

  enabled_axes[2] = true;
  {
    // The large feedback from the disabled tick must not seed the new integral.
    auto [command, status] = cascade->compute(
      measured, reference, enabled_axes, all_inputs_valid, applied, true, 2 * dt_ns, dt_ns);
    ASSERT_EQ(status, ::schema::rpp_interfaces::CODE_OK);
    EXPECT_DOUBLE_EQ(command[2], 8.0);
  }

  for (std::size_t axis = 0; axis < 3; ++axis) {
    enabled_axes[axis] = false;
                                                                           }
  {
    auto [command, status] = cascade->compute(
      measured, reference, enabled_axes, all_inputs_valid, applied, true, 3 * dt_ns, dt_ns);
    ASSERT_EQ(status, ::schema::rpp_interfaces::CODE_OK);
    for (std::size_t axis = 0; axis < command.size(); ++axis) {
      EXPECT_DOUBLE_EQ(command[axis], 0.0);
    }
  }
  {
    // An empty mask restores all axes, each with fresh history.
    auto [command, status] = cascade->compute(
      measured, reference, all_axes_enabled, all_inputs_valid, applied, true, 4 * dt_ns, dt_ns);
    ASSERT_EQ(status, ::schema::rpp_interfaces::CODE_OK);
    EXPECT_DOUBLE_EQ(command[0], 8.0);
    EXPECT_DOUBLE_EQ(command[1], 16.0);
    EXPECT_DOUBLE_EQ(command[2], 8.0);
  }
  enabled_axes.resize(2);
  EXPECT_EQ(std::get<1>(cascade->compute(
    measured, reference, enabled_axes, all_inputs_valid, applied, false, 5 * dt_ns, dt_ns)),
    ::schema::rpp_interfaces::CODE_INVALID_INPUT);
  ASSERT_TRUE(cascade_context.deactivate().success);
}

TEST(ControllerLifecycle, CascadeTracksEachStagesAppliedCommand)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  const auto & cascade_context = runtime.get_context().get_subcomponent_context("controller",
    "CascadeController2_comp");
  ASSERT_TRUE(cascade_context.activate().success);
  auto cascade = cascade_context.get_instance<rpp_interfaces::ControlLaw>();

  rpp::List<double> reference, measured, applied;
  reference.resize(9);
  measured.resize(6);
  applied.resize(3);
  reference[0] = 1.0;
  reference[3] = 2.0;
  reference[6] = ::schema::rpp_interfaces::REFERENCE_EXTERNAL;
  reference[7] = ::schema::rpp_interfaces::REFERENCE_INTERNAL;
  reference[8] = ::schema::rpp_interfaces::REFERENCE_INTERNAL;

  constexpr std::uint64_t dt = 500000000;
  {
    auto [command, status] = cascade->compute(measured, reference, all_axes_enabled,
      all_inputs_valid, applied, false, 0, dt);
    ASSERT_EQ(status, ::schema::rpp_interfaces::CODE_OK);
    ASSERT_EQ(command.size(), 3u);
    EXPECT_DOUBLE_EQ(command[0], 16.0);
  }

  applied[0] = 4.0;
  auto [command, status] = cascade->compute(measured, reference, all_axes_enabled, all_inputs_valid,
    applied, true, dt, dt);
  ASSERT_EQ(status, ::schema::rpp_interfaces::CODE_OK);
  ASSERT_EQ(command.size(), 3u);
  EXPECT_DOUBLE_EQ(command[0], -6.0);

  const auto description = cascade->getDesc().as_struct();
  const auto auxiliary_snapshot = cascade->getAuxiliaryState();
  rpp_control::ControlAuxiliaryState collected;
  collected.merge(auxiliary_snapshot);
  ControllerState restored;
  auto measured_state = restored.state();
  auto state_description = measured_state.desc();
  rpp_interfaces::ControlLawLayout(description).state().write(state_description);
  auto previous_control = restored.previousControl();
  auto control_description = previous_control.desc();
  rpp_interfaces::ControlLawLayout(description).control().write(control_description);
  auto control_values = previous_control.state();
  control_values.resize(applied.size());
  for (std::size_t index = 0; index < applied.size(); ++index) {
    control_values[index] = applied[index];
                                                                                                       }
  auto auxiliary_state = restored.auxiliaryState();
  collected.write_to(auxiliary_state);
  ASSERT_TRUE(cascade_context.deactivate().success);
  ASSERT_TRUE(cascade_context.activate(ControllerState::Const(std::move(restored))).success);

  // Select the pose stage's command next, making its tracking history observable.
  reference[6] = ::schema::rpp_interfaces::REFERENCE_INTERNAL;
  auto [next_command, next_status] = cascade->compute(
    measured, reference, all_axes_enabled, all_inputs_valid, applied, true, 2 * dt, dt);
  ASSERT_EQ(next_status, ::schema::rpp_interfaces::CODE_OK);
  EXPECT_DOUBLE_EQ(next_command[0], 14.0);
  ASSERT_TRUE(cascade_context.deactivate().success);
}
