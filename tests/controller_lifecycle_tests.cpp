#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include <gtest/gtest.h>

#include <rpp_control/pid.hpp>
#include <rpp_cpp/component_runtime_builder.hpp>
#include <rpp_plugin_types/rpp_control/MotionController2D.hpp>
#include <rpp_plugin_types/rpp_control/PoseController2D.hpp>
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
  return builder.build_script({
    rpp::ScriptDescriptionPath{"./tests/data/test_description/motion_controller2d.json"},
    std::nullopt, "./tests/data/test_ws_parts"});
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

TEST(ControllerLifecycle, AngularPidUsesShortestErrorAndDerivativeAcrossWrap)
{
  constexpr double pi = 3.14159265358979323846;
  rpp_control::PIDParameters parameters{};
  parameters.kp = 1.0F;
  rpp_control::PID pid(parameters, true);
  EXPECT_NEAR(pid.step(170.0 * pi / 180.0, -170.0 * pi / 180.0, 0.02),
    -20.0 * pi / 180.0, 1e-5);

  parameters.kp = 0.0F;
  parameters.kd = 1.0F;
  pid.set_parameters(parameters);
  pid.reset();
  (void)pid.step(179.0 * pi / 180.0, 0.0, 1.0);
  EXPECT_NEAR(pid.step(-179.0 * pi / 180.0, 0.0, 1.0),
    2.0 * pi / 180.0, 1e-5);

  // Linear channels retain ordinary subtraction.
  rpp_control::PID linear(parameters);
  EXPECT_NEAR(linear.step(170.0 * pi / 180.0, -170.0 * pi / 180.0, 1.0),
    340.0 * pi / 180.0, 1e-5);
}

TEST(ControllerLifecycle, PoseControllerYawUsesShortTurnAcrossPi)
{
  auto runtime = build_control_runtime();
  ASSERT_TRUE(runtime.configure().success);
  const auto & cascade = runtime.get_context().get_subcomponent_context("controller");
  ASSERT_TRUE(cascade.activate().success);
  auto pose = cascade.get_subcomponent_context("pose_controller")
    .get_instance<rpp_control::PoseController2D>();
  ASSERT_NE(pose, nullptr);
  rpp_control::PoseController2D::Pose2D reference;
  rpp_control::PoseController2D::Pose2D measured;
  rpp_control::PoseController2D::Enabler2D enabled;
  constexpr double pi = 3.14159265358979323846;
  reference.yaw() = 170.0 * pi / 180.0;
  measured.yaw() = -170.0 * pi / 180.0;
  enabled.enableN() = true;
  const auto output = pose->step(
    reference.as_const(), measured.as_const(), enabled.as_const(), 0.02);
  EXPECT_LT(output.angular(), 0.0);
}
