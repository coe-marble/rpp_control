#include "gtest/gtest.h"
#include "rpp_control/legacy/motion_controller2d_impl.hpp"
#include "rpp_control/legacy/motion_controller3d_impl.hpp"
#include "rpp_cpp/component_runtime_builder.hpp"

#include <stdexcept>
#include <limits>

class TestMotionController : public ::testing::Test {

public:
  const double EPSILON = 1e-5;

protected:
  static rpp::ScriptRuntime build_runtime()
  {
    rpp::RppDataManager data_manager(rpp::RPP_HOME);
    rpp::ComponentRuntimeBuilder builder(
      data_manager, rpp::ClockOptions(rpp::RPP_CLOCK_MOCK));
    return builder.build_script({
      rpp::ScriptDescriptionPath{
        "./tests/data/test_description/motion_controller2d.json"},
      std::nullopt, "./tests/data/test_ws_parts"});
  }

  static std::shared_ptr<rpp::RppClockMock> mock_clock(
    const rpp::ComponentContext & context)
  {
    auto read_only_clock = std::dynamic_pointer_cast<const rpp::RppClockMock>(
            context.get_clock());
    if (!read_only_clock) {
      throw std::runtime_error("Test runtime did not create a mock clock");
    }
    return std::const_pointer_cast<rpp::RppClockMock>(
            std::move(read_only_clock));
  }

};

TEST_F(TestMotionController, ThrustCapIsIncludedInReportedRealization)
{
    auto runtime = build_runtime();
    ASSERT_TRUE(runtime.configure().success);
    auto clock = mock_clock(runtime.get_context());
    clock->set_time(100.0);
    rpp_control::MotionController2DImpl controller(runtime);
    controller.configure();
    controller.set_output_limit(0.2);
    controller.set_feedback({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    controller.set_current_wrench_ref({6000.0, 0.0, 0.0}, {true, false, false});
    controller.set_wrench_selection({rpp_control::SIGNAL_EXT,
        rpp_control::SIGNAL_DISABLED, rpp_control::SIGNAL_DISABLED});
    ASSERT_TRUE(controller.step(0.02));
    const auto & state = controller.live_state();
    ASSERT_EQ(state.commands.size(), 2u);
    EXPECT_NEAR(state.commands[0], 0.2, EPSILON);
    EXPECT_NEAR(state.wrench[0], 1200.0, EPSILON);
}

TEST_F(TestMotionController, InvalidFeedbackClearsPreviouslyActiveOutput)
{
    auto runtime = build_runtime();
    ASSERT_TRUE(runtime.configure().success);
    auto clock = mock_clock(runtime.get_context());
    clock->set_time(100.0);
    rpp_control::MotionController2DImpl controller(runtime);
    controller.configure();
    controller.set_feedback({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    controller.set_current_wrench_ref({6000.0, 0.0, 0.0}, {true, false, false});
    controller.set_wrench_selection({rpp_control::SIGNAL_EXT,
        rpp_control::SIGNAL_DISABLED, rpp_control::SIGNAL_DISABLED});
    ASSERT_TRUE(controller.step(0.02));
    controller.set_feedback({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
        {0.0, 0.0, 0.0});
    EXPECT_FALSE(controller.step(0.02));
    for (const auto value : controller.live_state().commands) {EXPECT_DOUBLE_EQ(value, 0.0);}
}

TEST_F(TestMotionController, TestInstance2DAndSetReferences) {

    auto runtime = build_runtime();
    ASSERT_TRUE(runtime.configure().success);
    const auto & context = runtime.get_context();

    auto clock = mock_clock(context);

    clock->set_time(100.0);

    auto controller_2d = std::make_unique<rpp_control::MotionController2DImpl>(runtime);

    EXPECT_TRUE(controller_2d != nullptr);

    EXPECT_TRUE(controller_2d->is_active_dof(rpp_control::DOF_X));
    EXPECT_TRUE(controller_2d->is_active_dof(rpp_control::DOF_Y));
    EXPECT_TRUE(controller_2d->is_active_dof(rpp_control::DOF_N));
    EXPECT_FALSE(controller_2d->is_active_dof(rpp_control::DOF_Z));

    auto state = rpp_control::MotionController2DImpl::State{};

    EXPECT_FALSE(state.has_any_pose_ext);
    EXPECT_FALSE(state.has_any_twist_ext);
    EXPECT_FALSE(state.has_any_wrench_ext);
    controller_2d->set_feedback({1.0, 2.0, 3.0}, {0.1, 0.2, 0.3});
    controller_2d->latch_references(clock->now_seconds());
    state = controller_2d->live_state();
    EXPECT_TRUE(state.has_feedback);

    controller_2d->set_current_pose_ref({4.0, 5.0, 6.0}, {true, false, true});
    controller_2d->set_current_twist_ref({0.4, 0.5, 0.6}, {false, true, true});
    controller_2d->set_current_wrench_ref({0.7, 0.8, 0.9}, {true, true, false});

    // It should be false before latch_is_called
    EXPECT_FALSE(state.has_pose_ext[0]);
    clock->set_time(100.1);
    auto now = clock->now_seconds();
    controller_2d->latch_references(now);

    state = controller_2d->live_state();

    EXPECT_TRUE(state.has_pose_ext[0]);
    EXPECT_FALSE(state.has_pose_ext[1]);
    EXPECT_TRUE(state.has_pose_ext[2]);
    EXPECT_FALSE(state.has_twist_ext[0]);
    EXPECT_TRUE(state.has_twist_ext[1]);
    EXPECT_TRUE(state.has_twist_ext[2]);
    EXPECT_TRUE(state.has_wrench_ext[0]);
    EXPECT_TRUE(state.has_wrench_ext[1]);
    EXPECT_FALSE(state.has_wrench_ext[2]);

    EXPECT_NEAR(state.pose_ref[0], 4.0, EPSILON);
    EXPECT_NEAR(state.pose_ref[1], 0.0, EPSILON);
    EXPECT_NEAR(state.pose_ref[2], 6.0, EPSILON);
    EXPECT_NEAR(state.twist_ref[0], 0.0, EPSILON);
    EXPECT_NEAR(state.twist_ref[1], 0.5, EPSILON);
    EXPECT_NEAR(state.twist_ref[2], 0.6, EPSILON);
    EXPECT_NEAR(state.wrench_ref[0], 0.7, EPSILON);
    EXPECT_NEAR(state.wrench_ref[1], 0.8, EPSILON);
    EXPECT_NEAR(state.wrench_ref[2], 0.0, EPSILON);

    using SignalStatus = rpp_control::SignalStatus;

    controller_2d->set_wrench_selection(
    {SignalStatus::SIGNAL_EXT,
      SignalStatus::SIGNAL_INT,
      SignalStatus::SIGNAL_DISABLED});
    controller_2d->set_twist_selection(
    {SignalStatus::SIGNAL_INT,
      SignalStatus::SIGNAL_EXT,
      SignalStatus::SIGNAL_DISABLED});
    controller_2d->latch_references(now);
    state = controller_2d->live_state();

    EXPECT_EQ(state.twist_selection[0], SignalStatus::SIGNAL_INT);
    EXPECT_EQ(state.twist_selection[1], SignalStatus::SIGNAL_EXT);
    EXPECT_EQ(state.twist_selection[2], SignalStatus::SIGNAL_DISABLED);
    EXPECT_EQ(state.wrench_selection[0], SignalStatus::SIGNAL_EXT);
    EXPECT_EQ(state.wrench_selection[1], SignalStatus::SIGNAL_INT);
    EXPECT_EQ(state.wrench_selection[2], SignalStatus::SIGNAL_DISABLED);
}

TEST_F(TestMotionController, TestInstance2DAndStep) {
    auto runtime = build_runtime();
    ASSERT_TRUE(runtime.configure().success);
    const auto & context = runtime.get_context();

    auto clock = mock_clock(context);
    clock->set_time(100.0);

    auto controller_2d = std::make_unique<rpp_control::MotionController2DImpl>(runtime);

    EXPECT_TRUE(controller_2d != nullptr);

    bool step_result = controller_2d->step(0.01);
    EXPECT_FALSE(step_result); // Not configured yet.

    controller_2d->configure();

    step_result = controller_2d->step(0.01);
    EXPECT_FALSE(step_result); // No feedback received.

    clock->elapse(0.01);

    controller_2d->set_feedback({1.0, 2.0, 3.0}, {0.1, 0.2, 0.3});
    EXPECT_FALSE(controller_2d->step(0.01)); // No external references received.

    controller_2d->set_current_pose_ref({4.0, 5.0, 6.0}, {true, false, true});
    EXPECT_FALSE(controller_2d->step(-0.01));  // Wrong dt value.

    controller_2d->set_twist_selection(
    {rpp_control::SignalStatus::SIGNAL_INT,
      rpp_control::SignalStatus::SIGNAL_INT,
      rpp_control::SignalStatus::SIGNAL_INT});

    controller_2d->set_wrench_selection(
    {rpp_control::SignalStatus::SIGNAL_INT,
      rpp_control::SignalStatus::SIGNAL_INT,
      rpp_control::SignalStatus::SIGNAL_INT});

    EXPECT_TRUE(controller_2d->step(0.01)); // Should succeed now.

    auto state = rpp_control::MotionController2DImpl::State{};
    state = controller_2d->live_state();

    EXPECT_TRUE(state.has_feedback);
    EXPECT_TRUE(state.has_any_pose_ext);
    EXPECT_FALSE(state.has_any_twist_ext);
    EXPECT_FALSE(state.has_any_wrench_ext);

    EXPECT_NEAR(state.pose_ref[0], 4.0, EPSILON);
    EXPECT_NEAR(state.pose_ref[1], 0.0, EPSILON);
    EXPECT_NEAR(state.pose_ref[2], 6.0, EPSILON);
    EXPECT_NEAR(state.twist_ref[0], 0.0, EPSILON);
    EXPECT_NEAR(state.twist_ref[1], 0.0, EPSILON);
    EXPECT_NEAR(state.twist_ref[2], 0.0, EPSILON);
    EXPECT_NEAR(state.wrench_ref[0], 0.0, EPSILON);
    EXPECT_NEAR(state.wrench_ref[1], 0.0, EPSILON);
    EXPECT_NEAR(state.wrench_ref[2], 0.0, EPSILON);

    EXPECT_NEAR(state.pose[0], 1.0, EPSILON);
    EXPECT_NEAR(state.pose[1], 2.0, EPSILON);
    EXPECT_NEAR(state.pose[2], 3.0, EPSILON);
    EXPECT_NEAR(state.twist[0], 0.1, EPSILON);
    EXPECT_NEAR(state.twist[1], 0.2, EPSILON);
    EXPECT_NEAR(state.twist[2], 0.3, EPSILON);
    EXPECT_TRUE(state.wrench[0] != 0.0);
    EXPECT_TRUE(state.wrench[1] != 0.0);
    EXPECT_TRUE(state.wrench[2] != 0.0);

    EXPECT_TRUE(state.commands.size() == 2);
    EXPECT_TRUE(state.commands[0] != 0.0);
    EXPECT_TRUE(state.commands[1] != 0.0);
}

TEST_F(TestMotionController, ConfigureCanLeaveChildComponentsInactive) {
    auto runtime = build_runtime();
    ASSERT_TRUE(runtime.configure().success);
    const auto & context = runtime.get_context();

    rpp_control::MotionController2DImpl::Options options{};
    options.activate_on_start = false;
    rpp_control::MotionController2DImpl controller(runtime, options);
    controller.configure();

    EXPECT_FALSE(context.get_subcomponent_context("controller").is_active());
    EXPECT_FALSE(context.get_subcomponent_context("allocator").is_active());
}

TEST_F(TestMotionController, TestAllocationFeedbackSuppressesConflictingAxis) {
    auto runtime = build_runtime();
    ASSERT_TRUE(runtime.configure().success);
    const auto & context = runtime.get_context();

    auto clock = mock_clock(context);
    clock->set_time(100.0);

    auto controller_2d = std::make_unique<rpp_control::MotionController2DImpl>(
        runtime);
    auto allocation_suppression =
    rpp_control::MotionController2DImpl::AllocationSuppression{};
    allocation_suppression.priority_dof = rpp_control::DOF_N;
    allocation_suppression.suppressed_dof = rpp_control::DOF_X;
    controller_2d->set_allocation_suppression(allocation_suppression);
    controller_2d->configure();
    controller_2d->set_feedback({1.0, 2.0, 3.0}, {0.1, 0.2, 0.3});
    controller_2d->set_current_wrench_ref(
    {3000.0, 0.0, 3000.0}, {true, false, true});
    controller_2d->set_wrench_selection(
    {rpp_control::SIGNAL_EXT,
      rpp_control::SIGNAL_DISABLED,
      rpp_control::SIGNAL_EXT});

    EXPECT_TRUE(controller_2d->step(0.01));

    clock->elapse(0.01);
    EXPECT_TRUE(controller_2d->step(0.01));

    auto state = rpp_control::MotionController2DImpl::State{};
    state = controller_2d->live_state();
    EXPECT_TRUE(state.allocation_suppressed[0]);
    EXPECT_NEAR(state.wrench_ref[0], state.wrench[0], EPSILON);
    EXPECT_LT(std::abs(state.wrench[2]), std::abs(state.wrench_ref[2]));
}

TEST_F(TestMotionController, TestZeroSurgeCommandSuppressesSurgeForYawRate) {
    auto runtime = build_runtime();
    ASSERT_TRUE(runtime.configure().success);
    const auto & context = runtime.get_context();

    auto clock = mock_clock(context);
    clock->set_time(100.0);

    auto controller_2d = std::make_unique<rpp_control::MotionController2DImpl>(
        runtime);
    auto allocation_suppression =
    rpp_control::MotionController2DImpl::AllocationSuppression{};
    allocation_suppression.priority_dof = rpp_control::DOF_N;
    allocation_suppression.suppressed_dof = rpp_control::DOF_X;
    controller_2d->set_allocation_suppression(allocation_suppression);
    controller_2d->configure();
    controller_2d->set_feedback({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    controller_2d->set_current_twist_ref(
    {0.0, 0.0, 0.05}, {true, false, true});
    controller_2d->set_twist_selection(
    {rpp_control::SIGNAL_EXT,
      rpp_control::SIGNAL_DISABLED,
      rpp_control::SIGNAL_EXT});
    controller_2d->set_wrench_selection(
    {rpp_control::SIGNAL_INT,
      rpp_control::SIGNAL_DISABLED,
      rpp_control::SIGNAL_INT});

    EXPECT_TRUE(controller_2d->step(0.01));

    auto state = rpp_control::MotionController2DImpl::State{};
    state = controller_2d->live_state();
    EXPECT_TRUE(state.allocation_suppressed[0]);
    EXPECT_NEAR(state.wrench_ref[0], 0.0, EPSILON);
}

TEST_F(TestMotionController, TestRequestedWrenchRateLimit) {
    auto runtime = build_runtime();
    ASSERT_TRUE(runtime.configure().success);
    const auto & context = runtime.get_context();

    auto clock = mock_clock(context);
    clock->set_time(100.0);

    auto controller_2d = std::make_unique<rpp_control::MotionController2DImpl>(
        runtime);
    controller_2d->configure();
    controller_2d->set_wrench_rate_limits({1000.0, 0.0, 0.0});
    controller_2d->set_feedback({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    controller_2d->set_current_wrench_ref(
    {6000.0, 0.0, 0.0}, {true, false, false});
    controller_2d->set_wrench_selection(
    {rpp_control::SIGNAL_EXT,
      rpp_control::SIGNAL_DISABLED,
      rpp_control::SIGNAL_DISABLED});

    EXPECT_TRUE(controller_2d->step(0.1));
    auto state = rpp_control::MotionController2DImpl::State{};
    state = controller_2d->live_state();
    EXPECT_NEAR(state.wrench_ref[0], 6000.0, EPSILON);
    EXPECT_NEAR(state.wrench[0], 100.0, EPSILON);

    clock->elapse(0.1);
    EXPECT_TRUE(controller_2d->step(0.1));
    state = controller_2d->live_state();
    EXPECT_NEAR(state.wrench_ref[0], 6000.0, EPSILON);
    EXPECT_NEAR(state.wrench[0], 200.0, EPSILON);
}

TEST_F(TestMotionController, TestSuppressedSurgeDoesNotReverseYawWrench) {
    auto runtime = build_runtime();
    ASSERT_TRUE(runtime.configure().success);
    const auto & context = runtime.get_context();

    auto clock = mock_clock(context);
    clock->set_time(100.0);

    auto controller_2d = std::make_unique<rpp_control::MotionController2DImpl>(
        runtime);
    auto allocation_suppression =
    rpp_control::MotionController2DImpl::AllocationSuppression{};
    allocation_suppression.priority_dof = rpp_control::DOF_N;
    allocation_suppression.suppressed_dof = rpp_control::DOF_X;
    controller_2d->set_allocation_suppression(allocation_suppression);
    controller_2d->configure();
    controller_2d->set_feedback({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    controller_2d->set_current_twist_ref(
    {0.0, 0.0, 0.1}, {false, false, true});
    controller_2d->set_current_wrench_ref(
    {0.0, 0.0, 3000.0}, {true, false, true});
    controller_2d->set_twist_selection(
    {rpp_control::SIGNAL_DISABLED,
      rpp_control::SIGNAL_DISABLED,
      rpp_control::SIGNAL_EXT});
    controller_2d->set_wrench_selection(
    {rpp_control::SIGNAL_EXT,
      rpp_control::SIGNAL_DISABLED,
      rpp_control::SIGNAL_EXT});

    EXPECT_TRUE(controller_2d->step(0.01));
    clock->elapse(0.01);
    EXPECT_TRUE(controller_2d->step(0.01));

    controller_2d->set_current_wrench_ref(
    {0.0, 0.0, -3000.0}, {true, false, true});
    clock->elapse(0.01);
    EXPECT_TRUE(controller_2d->step(0.01));

    auto state = rpp_control::MotionController2DImpl::State{};
    state = controller_2d->live_state();
    EXPECT_TRUE(state.allocation_suppressed[0]);
    EXPECT_NEAR(state.wrench_ref[2], -3000.0, EPSILON);
    EXPECT_NEAR(state.wrench[2], 0.0, EPSILON);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

    // setenv("TEST_DATA_DIR", TEST_DATA_DIR, 1);

    // Pokrećemo GoogleTest najnormalnije
  return RUN_ALL_TESTS();
}
