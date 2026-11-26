#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include <rpp_control/planar_path.hpp>

namespace
{
class PlanarPathTest : public ::testing::Test
{
protected:
  std::vector<rpp_control::PlanarPath::Pose> view(std::size_t capacity = 8, double length = 100.0)
  {
    std::vector<rpp_control::PlanarPath::Pose> points;
    path.visit_pruned(capacity, length, [&](std::size_t, const auto & point) {points.push_back(point);});
    return points;
  }

  rpp_control::PlanarPath path;
};
}  // namespace

TEST_F(PlanarPathTest, ProjectsOntoSegmentsAndPreservesTheRemainingVertices)
{
  ASSERT_TRUE(path.set_path(std::vector<double>{0, 0, 0, 10, 0, 1, 10, 10, 2}, 3, 8));
  path.prune(4, 2, 30);
  const auto points = view();
  ASSERT_EQ(points.size(), 3u);
  EXPECT_DOUBLE_EQ(points[0].x, 4);
  EXPECT_DOUBLE_EQ(points[0].y, 0);
  EXPECT_DOUBLE_EQ(points[0].yaw, 0.4);
  EXPECT_DOUBLE_EQ(points[1].x, 10);
  EXPECT_DOUBLE_EQ(points[1].y, 0);
  EXPECT_DOUBLE_EQ(points[2].x, 10);
  EXPECT_DOUBLE_EQ(points[2].y, 10);
  EXPECT_DOUBLE_EQ(path.remaining_distance(), 16);
}

TEST_F(PlanarPathTest, HeadingInterpolationTakesTheShortestRotation)
{
  ASSERT_TRUE(path.set_path(std::vector<double>{0, 0, 3.1, 10, 0, -3.1}, 3, 8));
  path.prune(5, 0, 20);
  const auto points = view();
  EXPECT_NEAR(std::abs(points[0].yaw), 3.14159265358979323846, 1e-12);
}

TEST_F(PlanarPathTest, ProgressNeverMovesBackward)
{
  ASSERT_TRUE(path.set_path(std::vector<double>{0, 0, 0, 10, 0, 0, 20, 0, 0}, 3, 8));
  path.prune(15, 1, 30);
  path.prune(3, 0, 30);
  EXPECT_DOUBLE_EQ(view()[0].x, 15);
  EXPECT_DOUBLE_EQ(path.remaining_distance(), 5);
}

TEST_F(PlanarPathTest, SearchWindowDoesNotJumpToTheEndOfALoop)
{
  ASSERT_TRUE(path.set_path(std::vector<double>{
    0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0}, 3, 8));
  path.prune(5, 0, 20);
  EXPECT_DOUBLE_EQ(view()[0].x, 5);
  EXPECT_DOUBLE_EQ(path.remaining_distance(), 45);
}

TEST_F(PlanarPathTest, ClipsByArcLengthAndCapacityWithoutChangingGlobalRemainingDistance)
{
  ASSERT_TRUE(path.set_path(std::vector<double>{0, 0, 0, 10, 0, 0, 20, 0, 0, 30, 0, 0}, 3, 8));
  path.prune(5, 0, 30);
  const auto clipped = view(8, 12);
  ASSERT_EQ(clipped.size(), 3u);
  EXPECT_DOUBLE_EQ(clipped[0].x, 5);
  EXPECT_DOUBLE_EQ(clipped[1].x, 10);
  EXPECT_DOUBLE_EQ(clipped[2].x, 17);
  const auto bounded = view(2, 100);
  ASSERT_EQ(bounded.size(), 2u);
  EXPECT_DOUBLE_EQ(bounded.back().x, 10);
  EXPECT_DOUBLE_EQ(path.remaining_distance(), 25);
}

TEST_F(PlanarPathTest, InvalidReplacementPreservesProgressAndAValidReplacementResetsIt)
{
  ASSERT_TRUE(path.set_path(std::vector<double>{0, 0, 0, 10, 0, 0}, 3, 8));
  path.prune(8, 0, 20);
  EXPECT_FALSE(path.set_path(std::vector<double>{0, 0, std::numeric_limits<double>::quiet_NaN()}, 3, 8));
  EXPECT_FALSE(path.set_path(std::vector<double>{0, 0, 0, 1}, 3, 8));
  EXPECT_FALSE(path.set_path(std::vector<double>{0, 0, 0, 1, 0, 0}, 3, 1));
  EXPECT_DOUBLE_EQ(view()[0].x, 8);
  const auto largest = std::numeric_limits<double>::max();
  EXPECT_FALSE(path.set_path(std::vector<double>{-largest, 0, 0, largest, 0, 0}, 3, 8));
  EXPECT_DOUBLE_EQ(path.remaining_distance(), 2);
  ASSERT_TRUE(path.set_path(std::vector<double>{0, 0, 0, 5, 0, 0}, 3, 8));
  path.prune(1, 0, 20);
  EXPECT_DOUBLE_EQ(view()[0].x, 1);
}

TEST_F(PlanarPathTest, RepeatedPositionsAndAnExhaustedPathKeepTheFinalHeading)
{
  ASSERT_TRUE(path.set_path(std::vector<double>{0, 0, 0, 0, 0, 1, 10, 0, 2, 10, 0, 3}, 3, 8));
  path.prune(12, 0, 20);
  const auto points = view();
  ASSERT_EQ(points.size(), 1u);
  EXPECT_DOUBLE_EQ(points[0].x, 10);
  EXPECT_DOUBLE_EQ(points[0].yaw, 3);
  EXPECT_DOUBLE_EQ(path.remaining_distance(), 0);
  ASSERT_TRUE(path.set_path(std::vector<double>{2, 3, 1}, 3, 8));
  path.prune(0, 0, 20);
  ASSERT_EQ(view().size(), 1u);
  EXPECT_DOUBLE_EQ(view()[0].x, 2);
  EXPECT_DOUBLE_EQ(view()[0].y, 3);
}
