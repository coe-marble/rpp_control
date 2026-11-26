#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <tuple>

#include <gtest/gtest.h>

#include <rpp_control/planar_path_layout.hpp>
#include <rpp_cpp/component_runtime_builder.hpp>
#include <rpp_plugin_types/rpp_interfaces/ControlLaw.hpp>

namespace
{
class PurePursuitControllerTest : public ::testing::Test
{
protected:
  PurePursuitControllerTest()
  : data_manager_(rpp::RPP_HOME), runtime_(build_runtime(data_manager_)) {}

  static rpp::ComponentRuntime build_runtime(rpp::RppDataManager & data_manager)
  {
    rpp::ComponentRuntimeBuilder builder(data_manager, rpp::ClockOptions(rpp::RPP_CLOCK_MOCK));
    return builder.build_component({"./tests/data/pure_pursuit", ""});
  }

  void SetUp() override
  {
    ASSERT_TRUE(runtime_.configure().success);
    controller_ = runtime_.get_instance<rpp_interfaces::ControlLaw>();
    ASSERT_TRUE(runtime_.activate().success);
    state_.resize(6);
    reference_.resize(layout_.size());
    reference_[layout_.positions_offset() + 2] = 10.0;
    reference_[layout_.count_offset()] = 2.0;
    reference_[layout_.remaining_distance_offset()] = 10.0;
    enabled_.resize(3);
    for (std::size_t axis = 0; axis < 3; ++axis) {enabled_[axis] = true;}
    valid_.resize(layout_.size());
    for (std::size_t scalar = 0; scalar < valid_.size(); ++scalar) {valid_[scalar] = true;}
    previous_.resize(3);
    previous_[0] = 1.0;
  }

  auto compute(std::uint64_t time_ns = 0, std::uint64_t dt_ns = 100000000)
  {
    return controller_->compute(state_, reference_, enabled_, valid_, previous_, true, time_ns,
        dt_ns);
  }

  rpp::RppDataManager data_manager_;
  rpp::ComponentRuntime runtime_;
  std::shared_ptr<rpp_interfaces::ControlLaw> controller_;
  rpp_control::PlanarPathLayout layout_{4};
  rpp::List<double> state_, reference_, previous_;
  rpp::List<bool> enabled_, valid_;
};
}  // namespace

TEST_F(PurePursuitControllerTest, DescribesCurrentPathInputsAndBodyVelocityOutputs)
{
  const auto description = controller_->getDesc().as_struct();
  EXPECT_EQ(description.horizonLength, 0u);
  EXPECT_TRUE(description.horizonOffsetsNs.empty());
  EXPECT_EQ(description.reference.fields.size(), 6u);
  EXPECT_EQ(description.state.fields.size(), 4u);
  ASSERT_EQ(description.control.fields.size(), 2u);
  EXPECT_EQ(description.control.fields[0].name, "base.linearVelocity");
  EXPECT_EQ(description.control.fields[0].frameId, "base_link");
  EXPECT_EQ(description.control.fields[1].name, "base.angularVelocity");
  const auto [command, status] = compute();
  ASSERT_EQ(status, rpp_interfaces::contract::CODE_OK);
  ASSERT_EQ(command.size(), 3u);
  EXPECT_DOUBLE_EQ(command[0], 1.0);
  EXPECT_DOUBLE_EQ(command[1], 0.0);
  EXPECT_DOUBLE_EQ(command[2], 0.0);
}

TEST_F(PurePursuitControllerTest, ExpiredPathOrDisabledSteeringStopsWithoutARampTail)
{
  previous_[2] = 1.0;
  valid_[layout_.positions_offset()] = false;
  const auto [expired, expired_status] = compute();
  ASSERT_EQ(expired_status, rpp_interfaces::contract::CODE_OK);
  EXPECT_DOUBLE_EQ(expired[0], 0.0);
  EXPECT_DOUBLE_EQ(expired[2], 0.0);
  valid_[layout_.positions_offset()] = true;
  enabled_[2] = false;
  const auto [disabled, disabled_status] = compute(1);
  ASSERT_EQ(disabled_status, rpp_interfaces::contract::CODE_OK);
  EXPECT_DOUBLE_EQ(disabled[0], 0.0);
  EXPECT_DOUBLE_EQ(disabled[2], 0.0);
}

TEST_F(PurePursuitControllerTest, RejectsInvalidTimeAndMalformedReferences)
{
  EXPECT_EQ(std::get<1>(compute(10, 0)), rpp_interfaces::contract::CODE_INVALID_INPUT);
  ASSERT_EQ(std::get<1>(compute(10)), rpp_interfaces::contract::CODE_OK);
  EXPECT_EQ(std::get<1>(compute(9)), rpp_interfaces::contract::CODE_STALE_INPUT);
  reference_[layout_.count_offset()] = 1.5;
  EXPECT_EQ(std::get<1>(compute(11)), rpp_interfaces::contract::CODE_INVALID_INPUT);
  reference_[layout_.count_offset()] = 2;
  state_[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(std::get<1>(compute(11)), rpp_interfaces::contract::CODE_INVALID_INPUT);
}

TEST_F(PurePursuitControllerTest, RetainedOutputSurvivesLaterCallsAndCleanup)
{
  const auto [first, first_status] = compute();
  ASSERT_EQ(first_status, rpp_interfaces::contract::CODE_OK);
  valid_[layout_.positions_offset()] = false;
  const auto [stopped, stopped_status] = compute(1);
  ASSERT_EQ(stopped_status, rpp_interfaces::contract::CODE_OK);
  EXPECT_DOUBLE_EQ(stopped[0], 0.0);
  EXPECT_DOUBLE_EQ(first[0], 1.0);
  ASSERT_TRUE(runtime_.deactivate().success);
  ASSERT_TRUE(runtime_.cleanup().success);
  EXPECT_DOUBLE_EQ(first[0], 1.0);
}
