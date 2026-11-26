#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include <rpp_control/regulated_pure_pursuit.hpp>

namespace
{
class PurePursuitTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    parameters.use_velocity_scaled_lookahead_dist = false;
    parameters.use_rotate_to_heading = false;
    parameters.regulated_linear_scaling_min_radius = 0.1;
    parameters.max_linear_accel = 100.0;
    parameters.max_angular_accel = 100.0;
    parameters.max_angular_vel = 10.0;
  }

  std::vector<double> path(std::initializer_list<std::array<double, 3>> points,
    std::optional<double> remaining = std::nullopt) const
  {
    std::vector<double> values(layout.size(), 0.0);
    std::size_t index = 0;
    double length = 0.0;
    std::array<double, 3> previous{};
    for (const auto & pose : points) {
      values[layout.positions_offset() + 2 * index] = pose[0];
      values[layout.positions_offset() + 2 * index + 1] = pose[1];
      values[layout.orientations_offset() + index] = pose[2];
      if (index > 0) {length += std::hypot(pose[0] - previous[0], pose[1] - previous[1]);}
      previous = pose;
      ++index;
    }
    values[layout.count_offset()] = static_cast<double>(points.size());
    values[layout.remaining_distance_offset()] = remaining.value_or(length);
    return values;
  }

  rpp_control::RegulatedPurePursuit::Command compute(const std::vector<double> & reference,
    double previous_linear = 0.0, double previous_angular = 0.0, double dt = 1.0) const
  {
    return rpp_control::RegulatedPurePursuit(parameters).compute(
      state, reference, layout, previous_linear, previous_angular, dt);
  }

  rpp_control::PurePursuitParameters parameters;
  rpp_control::PlanarPathLayout layout{8};
  std::array<double, 6> state{};
};
}  // namespace

TEST_F(PurePursuitTest, InterpolatedLookaheadDoesNotDependOnWaypointDensity)
{
  const auto sparse = compute(path({{0, 0, 0}, {2, 2, 0}}));
  const auto dense = compute(path({{0, 0, 0}, {0.5, 0.5, 0}, {1, 1, 0}, {2, 2, 0}}));
  EXPECT_DOUBLE_EQ(sparse.linear, 1.0);
  EXPECT_NEAR(sparse.angular, std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(dense.angular, sparse.angular, 1e-12);
}

TEST_F(PurePursuitTest, UsesTheMeasuredPoseToTransformThePathToBodyCoordinates)
{
  state = {100, 200, 1.5707963267948966, 0, 0, 0};
  const auto command = compute(path({{100, 200, 0}, {100, 210, 0}}));
  EXPECT_DOUBLE_EQ(command.linear, 1.0);
  EXPECT_NEAR(command.angular, 0.0, 1e-12);
}

TEST_F(PurePursuitTest, AdaptiveLookaheadRespectsItsMinimumAndMaximum)
{
  parameters.use_velocity_scaled_lookahead_dist = true;
  parameters.lookahead_time = 1.0;
  parameters.min_lookahead_dist = 0.5;
  parameters.max_lookahead_dist = 2.0;
  const auto reference = path({{0, 0, 0}, {4, 4, 0}});
  const auto slow = compute(reference);
  state[3] = 10.0;
  const auto fast = compute(reference);
  EXPECT_NEAR(slow.angular, 2.0 * std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(fast.angular, std::sqrt(2.0) / 2.0, 1e-12);
}

TEST_F(PurePursuitTest, APerpendicularTargetDoesNotBecomeBehindDueToRoundoff)
{
  state[2] = 1.5707963267948966;
  const auto command = compute(path({{0, 0, 0}, {-2, 0, 0}}));
  EXPECT_EQ(command.status, rpp_interfaces::contract::CODE_OK);
  EXPECT_DOUBLE_EQ(command.linear, 1.0);
  EXPECT_NEAR(command.angular, 2.0, 1e-12);
}

TEST_F(PurePursuitTest, CurvatureAndAngularLimitsReduceForwardSpeed)
{
  const auto reference = path({{0, 0, 0}, {2, 2, 0}});
  parameters.regulated_linear_scaling_min_radius = 2.0;
  const auto curved = compute(reference);
  EXPECT_NEAR(curved.linear, 1.0 / (2.0 * std::sqrt(2.0)), 1e-12);
  EXPECT_NEAR(curved.angular, 0.5, 1e-12);
  EXPECT_EQ(curved.status, rpp_interfaces::contract::CODE_LIMITED);
  parameters.regulated_linear_scaling_min_radius = 0.1;
  parameters.max_angular_vel = 0.25;
  parameters.rotate_to_heading_angular_vel = 0.25;
  const auto capped = compute(reference);
  EXPECT_NEAR(capped.linear, 0.25 / std::sqrt(2.0), 1e-12);
  EXPECT_NEAR(capped.angular, 0.25, 1e-12);
}

TEST_F(PurePursuitTest, FixedCurvatureLookaheadControlsSteeringIndependentlyOfAdaptiveDistance)
{
  const auto reference = path({{0, 0, 0}, {4, 4, 0}});
  parameters.lookahead_dist = 2.0;
  parameters.use_fixed_curvature_lookahead = true;
  parameters.curvature_lookahead_dist = 0.5;
  EXPECT_NEAR(compute(reference).angular, 2.0 * std::sqrt(2.0), 1e-12);
  parameters.use_fixed_curvature_lookahead = false;
  EXPECT_NEAR(compute(reference).angular, std::sqrt(2.0) / 2.0, 1e-12);
}

TEST_F(PurePursuitTest, CurvatureExtrapolationDoesNotExtendAClippedLocalSection)
{
  parameters.use_fixed_curvature_lookahead = true;
  parameters.curvature_lookahead_dist = 1.0;
  parameters.interpolate_curvature_after_goal = true;
  parameters.approach_velocity_scaling_dist = 0.1;
  const auto endpoint = compute(path({{0, 0.5, 0}, {0.5, 0.5, 0}}));
  const auto clipped = compute(path({{0, 0.5, 0}, {0.5, 0.5, 0}}, 10.0));
  EXPECT_NEAR(endpoint.angular, 1.0, 1e-12);
  EXPECT_NEAR(clipped.angular, 2.0, 1e-12);
}

TEST_F(PurePursuitTest, OnlyApproachesTheGlobalEndpoint)
{
  const auto goal = compute(path({{0, 0, 0}, {0.2, 0, 0}}));
  const auto clipped = compute(path({{0, 0, 0}, {0.2, 0, 0}}, 10.0));
  EXPECT_NEAR(goal.linear, 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(clipped.linear, 1.0);
  const auto loop = compute(path({{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 0}}));
  EXPECT_DOUBLE_EQ(loop.linear, 1.0);
}

TEST_F(PurePursuitTest, AlignsFinalHeadingAndRotatesBeforeFollowingABehindTarget)
{
  parameters.use_rotate_to_heading = true;
  const auto final = compute(path({{0, 0, 1.5707963267948966}}));
  EXPECT_DOUBLE_EQ(final.linear, 0.0);
  EXPECT_DOUBLE_EQ(final.angular, 1.0);
  const auto behind = compute(path({{-1, 0, 0}}));
  EXPECT_DOUBLE_EQ(behind.linear, 0.0);
  EXPECT_DOUBLE_EQ(behind.angular, 1.0);
  parameters.use_rotate_to_heading = false;
  EXPECT_EQ(compute(path({{-1, 0, 0}})).status, rpp_interfaces::contract::CODE_INFEASIBLE);
}

TEST_F(PurePursuitTest, LimitsCommandChangesUsingThePreviousAppliedVelocity)
{
  parameters.max_linear_accel = 0.5;
  parameters.max_linear_decel = 0.25;
  parameters.max_angular_accel = 0.2;
  const auto accelerating = compute(path({{0, 0, 0}, {2, 2, 0}}), 0.0, 0.0, 0.1);
  EXPECT_NEAR(accelerating.linear, 0.05, 1e-12);
  EXPECT_NEAR(accelerating.angular, 0.02, 1e-12);
  const auto braking = compute(path({{0, 0, 0}, {0.1, 0, 0}}), 1.0, 0.0, 0.1);
  EXPECT_NEAR(braking.linear, 0.975, 1e-12);
}

TEST_F(PurePursuitTest, EmptyPathsStopImmediatelyAndInvalidMetadataIsRejected)
{
  EXPECT_DOUBLE_EQ(compute(path({}), 1.0, 1.0, 0.01).linear, 0.0);
  auto reference = path({{0, 0, 0}, {2, 0, 0}});
  reference[layout.count_offset()] = 1.5;
  EXPECT_EQ(compute(reference).status, rpp_interfaces::contract::CODE_INVALID_INPUT);
  reference[layout.count_offset()] = 9;
  EXPECT_EQ(compute(reference).status, rpp_interfaces::contract::CODE_INVALID_INPUT);
  reference[layout.count_offset()] = 2;
  reference[layout.remaining_distance_offset()] = -1;
  EXPECT_EQ(compute(reference).status, rpp_interfaces::contract::CODE_INVALID_INPUT);
  reference[layout.remaining_distance_offset()] = 1;
  EXPECT_EQ(compute(reference).status, rpp_interfaces::contract::CODE_INVALID_INPUT);
}

TEST_F(PurePursuitTest, YawReversalBrakesBeforeAcceleratingInTheOppositeDirection)
{
  parameters.max_angular_accel = 0.4;
  parameters.max_angular_decel = 0.2;
  const auto braking = compute(path({{0, 0, 0}, {10, 0, 0}}), 1.0, 1.0, 0.1);
  EXPECT_NEAR(braking.angular, 0.98, 1e-12);
  const auto reversed = compute(path({{0, 0, 0}, {2, -2, 0}}), 1.0, 0.1, 1.0);
  EXPECT_NEAR(reversed.angular, -0.2, 1e-12);
}

TEST_F(PurePursuitTest, RejectsInvalidParametersBeforeNumericExecution)
{
  parameters.max_linear_accel = 0;
  EXPECT_THROW(rpp_control::RegulatedPurePursuit{parameters}, std::invalid_argument);
  parameters.max_linear_accel = 1;
  parameters.min_lookahead_dist = 3;
  EXPECT_THROW(rpp_control::RegulatedPurePursuit{parameters}, std::invalid_argument);
  parameters.min_lookahead_dist = 0.3;
  parameters.desired_linear_vel = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(rpp_control::RegulatedPurePursuit{parameters}, std::invalid_argument);
}
