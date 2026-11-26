#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <memory>
#include <string>
#include <tuple>

#include <gtest/gtest.h>

#include <rpp_control/planar_path_layout.hpp>
#include <rpp_cpp/component_runtime_builder.hpp>
#include <rpp_plugin_types/rpp_interfaces/ReferenceGenerator.hpp>
#include <capnp_gen/rpp_interfaces/reference/constants.capnp.h>

namespace
{
class PlanarPathReferenceTest : public ::testing::Test
{
protected:
  explicit PlanarPathReferenceTest(const std::string & path = "./tests/data/planar_path_reference")
  : data_manager_(rpp::RPP_HOME), runtime_(build_runtime(data_manager_, path)) {}

  static rpp::ComponentRuntime build_runtime(
    rpp::RppDataManager & data_manager,
    const std::string & path)
  {
    rpp::ComponentRuntimeBuilder builder(data_manager, rpp::ClockOptions(rpp::RPP_CLOCK_MOCK));
    return builder.build_component({path, ""});
  }

  void SetUp() override
  {
    ASSERT_TRUE(runtime_.configure().success);
    generator_ = runtime_.get_instance<rpp_interfaces::ReferenceGenerator>();
    ASSERT_EQ(generator_->bind(no_offsets_, 0), rpp_interfaces::contract::CODE_OK);
    ASSERT_TRUE(runtime_.activate().success);
    state_.resize(3);
    width_ = rpp_interfaces::FieldLayout(generator_->getDesc().as_struct().reference.fields).size();
    mask_.resize(width_);
    for (std::size_t scalar = 0; scalar < mask_.size(); ++scalar) {
      mask_[scalar] = scalar < 3 || scalar >= layout_.size();
    }
  }

  rpp::List<double> goal(std::initializer_list<double> poses) const
  {
    rpp::List<double> values;
    values.resize(poses.size() / 3 * width_);
    std::size_t scalar = 0;
    for (double value : poses) {
      values[(scalar / 3) * width_ + scalar % 3] = value;
      ++scalar;
    }
    return values;
  }

  rpp::RppDataManager data_manager_;
  rpp::ComponentRuntime runtime_;
  std::shared_ptr<rpp_interfaces::ReferenceGenerator> generator_;
  rpp_control::PlanarPathLayout layout_{4};
  std::size_t width_ = 0;
  rpp::List<double> state_;
  rpp::List<bool> mask_;
  const rpp::ListConst<std::uint64_t> no_offsets_{capnp::List<std::uint64_t>::Reader{}};
};
}  // namespace

TEST_F(PlanarPathReferenceTest, EmitsTheSpatialPathWithCountPaddingAndGlobalRemainingDistance)
{
  const auto values = goal({0, 0, 0, 10, 0, 0, 10, 10, 1});
  ASSERT_EQ(generator_->setPath(values, mask_), rpp_interfaces::contract::CODE_OK);
  state_[0] = 4; state_[1] = 2;
  const auto [reference, status] = generator_->sample(state_, 0);
  ASSERT_EQ(status, rpp_interfaces::contract::CODE_OK);
  ASSERT_EQ(reference.size(), layout_.size());
  EXPECT_DOUBLE_EQ(reference[0], 4);
  EXPECT_DOUBLE_EQ(reference[1], 0);
  EXPECT_DOUBLE_EQ(reference[layout_.count_offset()], 3);
  EXPECT_DOUBLE_EQ(reference[layout_.remaining_distance_offset()], 16);
  EXPECT_DOUBLE_EQ(reference[layout_.positions_offset() + 2], 10);
  EXPECT_DOUBLE_EQ(reference[layout_.positions_offset() + 3], 0);
  EXPECT_DOUBLE_EQ(reference[layout_.positions_offset() + 4], 10);
  EXPECT_DOUBLE_EQ(reference[layout_.positions_offset() + 5], 10);
  EXPECT_DOUBLE_EQ(reference[layout_.positions_offset() + 6], 10);
  EXPECT_DOUBLE_EQ(reference[layout_.positions_offset() + 7], 10);
  EXPECT_DOUBLE_EQ(reference[layout_.orientations_offset() + 3], 1);
  state_[0] = 10; state_[1] = 4;
  const auto [later, later_status] = generator_->sample(state_, 1000000000000);
  ASSERT_EQ(later_status, rpp_interfaces::contract::CODE_OK);
  EXPECT_DOUBLE_EQ(later[0], 10);
  EXPECT_DOUBLE_EQ(later[1], 4);
  EXPECT_DOUBLE_EQ(reference[0], 4);  // Retained views are never overwritten.
}

TEST_F(PlanarPathReferenceTest, RejectsPartialGoalsAndTimedBindingWithoutLosingTheAcceptedPath)
{
  const auto values = goal({0, 0, 0, 10, 0, 0});
  ASSERT_EQ(generator_->setPath(values, mask_), rpp_interfaces::contract::CODE_OK);
  rpp::List<std::uint64_t> offsets;
  offsets.resize(2); offsets[0] = 0; offsets[1] = 1000000000;
  EXPECT_EQ(generator_->bind(offsets, 2), rpp_interfaces::contract::CODE_UNSUPPORTED);
  EXPECT_EQ(generator_->setTrajectory(values, offsets, mask_),
    rpp_interfaces::contract::CODE_UNSUPPORTED);
  mask_[2] = false;
  EXPECT_EQ(generator_->setPath(values, mask_), rpp_interfaces::contract::CODE_INVALID_INPUT);
  state_[0] = 5;
  const auto [reference, status] = generator_->sample(state_, 100);
  ASSERT_EQ(status, rpp_interfaces::contract::CODE_OK);
  EXPECT_DOUBLE_EQ(reference[0], 5);
  EXPECT_EQ(std::get<1>(generator_->sample(state_, 99)),
    rpp_interfaces::contract::CODE_STALE_INPUT);
  state_[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(std::get<1>(generator_->sample(state_, 101)),
    rpp_interfaces::contract::CODE_INVALID_INPUT);
}

TEST_F(PlanarPathReferenceTest, ReplanningRestartsPruningAndCancellationClearsThePath)
{
  ASSERT_EQ(generator_->setPath(goal({0, 0, 0, 10, 0, 0}), mask_),
    rpp_interfaces::contract::CODE_OK);
  state_[0] = 8;
  ASSERT_EQ(std::get<1>(generator_->sample(state_, 0)), rpp_interfaces::contract::CODE_OK);
  ASSERT_EQ(generator_->setPath(goal({0, 0, 0, 5, 0, 0}), mask_),
    rpp_interfaces::contract::CODE_OK);
  state_[0] = 1;
  const auto [reference, status] = generator_->sample(state_, 1);
  ASSERT_EQ(status, rpp_interfaces::contract::CODE_OK);
  EXPECT_DOUBLE_EQ(reference[0], 1);
  ASSERT_EQ(generator_->cancel(), rpp_interfaces::contract::CODE_OK);
  const auto [cancelled, cancelled_status] = generator_->sample(state_, 2);
  ASSERT_EQ(cancelled_status, rpp_interfaces::contract::CODE_OK);
  EXPECT_DOUBLE_EQ(cancelled[layout_.count_offset()], 0);
  EXPECT_DOUBLE_EQ(cancelled[layout_.remaining_distance_offset()], 0);
}

TEST_F(PlanarPathReferenceTest, AdvertisesPoseTargetsAndReadOnlyDerivedPathFields)
{
  const auto description = generator_->getDesc().as_struct();
  EXPECT_EQ(description.features,
    rpp_interfaces::contract::GENERATOR_SET_POINT | rpp_interfaces::contract::GENERATOR_SET_PATH);
  ASSERT_EQ(description.targetMask.size(), 6u);
  for (std::size_t field = 0; field < description.targetMask.size(); ++field) {
    EXPECT_EQ(description.targetMask[field], field < 2);
  }
  EXPECT_EQ(description.state.fields.size(), 2u);
  ASSERT_EQ(generator_->setPoint(goal({2, 3, 1}), mask_), rpp_interfaces::contract::CODE_OK);
  const auto [reference, status] = generator_->sample(state_, 0);
  ASSERT_EQ(status, rpp_interfaces::contract::CODE_OK);
  EXPECT_DOUBLE_EQ(reference[layout_.count_offset()], 1);
  EXPECT_DOUBLE_EQ(reference[0], 2);
  EXPECT_DOUBLE_EQ(reference[1], 3);
  EXPECT_DOUBLE_EQ(reference[2], 1);
}

namespace
{
class PathOverridesTest : public PlanarPathReferenceTest
{
protected:
  PathOverridesTest()
  : PlanarPathReferenceTest("./tests/data/path_reference_overrides") {}
};
}  // namespace

TEST_F(PathOverridesTest, UpdatingAnInnerReferenceDoesNotRestartTheAcceptedPath)
{
  ASSERT_EQ(generator_->setPath(goal({0, 0, 0, 10, 0, 0}), mask_),
    rpp_interfaces::contract::CODE_OK);
  state_[0] = 8;
  ASSERT_EQ(std::get<1>(generator_->sample(state_, 0)), rpp_interfaces::contract::CODE_OK);
  auto override = goal({0, 0, 0});
  override[layout_.size()] = 2.5;
  for (std::size_t scalar = 0; scalar < mask_.size(); ++scalar) {
    mask_[scalar] = scalar == layout_.size();
                                                                                                          }
  ASSERT_EQ(generator_->setPoint(override, mask_), rpp_interfaces::contract::CODE_OK);
  state_[0] = 2;
  const auto [reference, status] = generator_->sample(state_, 1);
  ASSERT_EQ(status, rpp_interfaces::contract::CODE_OK);
  EXPECT_DOUBLE_EQ(reference[0], 8.0);
  EXPECT_DOUBLE_EQ(reference[layout_.remaining_distance_offset()], 2.0);
  EXPECT_DOUBLE_EQ(reference[layout_.size()], 2.5);
  ASSERT_TRUE(generator_->getDesc().as_struct().targetMask.back());
  ASSERT_EQ(generator_->cancel(), rpp_interfaces::contract::CODE_OK);
  const auto [cancelled, cancelled_status] = generator_->sample(state_, 2);
  ASSERT_EQ(cancelled_status, rpp_interfaces::contract::CODE_OK);
  EXPECT_DOUBLE_EQ(cancelled[layout_.size()], 0.0);
}

TEST_F(PathOverridesTest, RejectsVaryingOverridesBeforeReplacingThePath)
{
  ASSERT_EQ(generator_->setPath(goal({0, 0, 0, 10, 0, 0}), mask_),
    rpp_interfaces::contract::CODE_OK);
  state_[0] = 8;
  ASSERT_EQ(std::get<1>(generator_->sample(state_, 0)), rpp_interfaces::contract::CODE_OK);
  auto update = goal({0, 0, 0, 20, 0, 0});
  update[width_ + layout_.size()] = 1.0;
  EXPECT_EQ(generator_->setPath(update, mask_), rpp_interfaces::contract::CODE_INVALID_INPUT);
  const auto [reference, status] = generator_->sample(state_, 1);
  ASSERT_EQ(status, rpp_interfaces::contract::CODE_OK);
  EXPECT_DOUBLE_EQ(reference[layout_.remaining_distance_offset()], 2.0);
}
