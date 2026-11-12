#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include <rpp_control/pid.hpp>
#include <rpp_cpp/component_runtime_builder.hpp>
#include <rpp_plugin_types/rpp_control/MotionController2D.hpp>
#include <rpp_schema/rpp_common/Odometry2D.hpp>
#include <rpp_schema/rpp_control/EnablerOdometry2D.hpp>

namespace
{
using ControllerState = rpp_schema::rpp_control::ControllerHandoverState::Const;

rpp::ScriptRuntime<> build_control_runtime()
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

std::array<double, 18> axis_values(const ControllerState & state)
{
  const auto axes = state.axisErrors();
  EXPECT_EQ(axes.size(), 6u);
  std::array<double, 18> values{};
  for (size_t index = 0; index < 6; ++index) {
    values[index * 3] = axes[index].integralError();
    values[index * 3 + 1] = axes[index].previousError();
    values[index * 3 + 2] = axes[index].previousDerivative();
  }
  return values;
}

void set_all_axes(rpp_schema::rpp_control::EnablerOdometry2D & enabler)
{
  auto pose = enabler.pose();
  pose.enableX() = true;
  pose.enableY() = true;
  pose.enableN() = true;
  auto twist = enabler.twist();
  twist.enableX() = true;
  twist.enableY() = true;
  twist.enableN() = true;
}
}  // namespace

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
  EXPECT_FLOAT_EQ(restored_state.integral_error, exported.integral_error);
  EXPECT_FLOAT_EQ(restored_state.previous_error, exported.previous_error);
  EXPECT_FLOAT_EQ(restored_state.previous_derivative, exported.previous_derivative);

  auto invalid = exported;
  invalid.previous_derivative = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(restored.restore_error_info(invalid), std::invalid_argument);
}

TEST(ControllerLifecycle, CascadeTransfersPoseAndTwistPidHistories)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  const auto & context = runtime.get_context();
  const auto & cascade_context = context.get_subcomponent_context("controller");
  ASSERT_TRUE(cascade_context.activate().success);

  auto cascade = cascade_context.get_instance<rpp_control::MotionController2D>();
  rpp_schema::rpp_common::Odometry2D reference;
  reference.pose().position().x() = 1.0;
  reference.pose().position().y() = 2.0;
  reference.pose().yaw() = 3.0;
  reference.twist().linear().x() = 4.0;
  reference.twist().linear().y() = 5.0;
  reference.twist().angular() = 6.0;
  rpp_schema::rpp_common::Odometry2D measured;
  rpp_schema::rpp_control::EnablerOdometry2D enabled;
  set_all_axes(enabled);

  (void)cascade->step(reference.as_const(), measured.as_const(), enabled.as_const(), 0.5);
  auto stopped = cascade_context.deactivate();
  ASSERT_TRUE(stopped.result.success);
  ASSERT_TRUE(stopped.state.has_value());
  ASSERT_TRUE(stopped.state->holds<ControllerState>());
  const auto first = axis_values(stopped.state->get<ControllerState>());
  EXPECT_GT(first[0], 0.0);
  EXPECT_GT(first[9], 0.0);

  ASSERT_TRUE(cascade_context.activate(std::move(*stopped.state)).success);
  auto restored = cascade_context.deactivate();
  ASSERT_TRUE(restored.result.success);
  ASSERT_TRUE(restored.state.has_value());
  ASSERT_TRUE(restored.state->holds<ControllerState>());
  EXPECT_EQ(axis_values(restored.state->get<ControllerState>()), first);
}
