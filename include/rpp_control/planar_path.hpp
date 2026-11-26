#pragma once

#include <rpp_control/angle.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rpp_control
{
// Geometry only: storage changes on submission; pruning and visiting do not allocate.
class PlanarPath
{
public:
  struct Pose
  {
    double x, y, yaw;
  };

  template<class Values>
  bool set_path(const Values & values, std::size_t stride, std::size_t max_points)
  {
    if (stride < 3 || values.empty() || values.size() % stride != 0 ||
      values.size() / stride > max_points) {return false;}

    std::vector<Waypoint> prepared;
    prepared.reserve(values.size() / stride);
    for (std::size_t offset = 0; offset < values.size(); offset += stride) {
      Pose pose{values[offset], values[offset + 1], values[offset + 2]};
      if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
        return false;
      }
      pose.yaw = wrap_angle(pose.yaw);
      double distance = 0.0;
      if (!prepared.empty()) {
        const auto & previous = prepared.back();
        distance = previous.distance + std::hypot(pose.x - previous.pose.x,
            pose.y - previous.pose.y);
        if (!std::isfinite(distance)) {return false;}
      }
      prepared.push_back({pose, distance});
    }
    // Invalid updates leave both the accepted path and its progress untouched.
    points_.swap(prepared);
    segment_ = 0;
    progress_ = 0.0;
    first_ = points_.front().pose;
    return true;
  }

  bool empty() const noexcept {return points_.empty();}
  void clear() {points_.clear(); segment_ = 0; progress_ = 0.0;}
  double remaining_distance() const noexcept {return points_.back().distance - progress_;}

  // The caller supplies finite coordinates and a finite positive search distance.
  // Search forward in arc length so a nearby return leg cannot skip a whole loop.
  void prune(double x, double y, double max_search_distance) noexcept
  {
    const auto search_end = progress_ + std::min(max_search_distance, remaining_distance());
    auto best_distance = distance_to(first_, x, y);
    auto best_progress = progress_;
    auto best_segment = segment_;
    for (std::size_t index = segment_; index + 1 < points_.size(); ++index) {
      const auto & start = points_[index];
      const auto & end = points_[index + 1];
      if (start.distance > search_end) {break;}
      const auto length = end.distance - start.distance;
      if (length <= 0.0) {continue;}
      const auto lower = std::max(progress_, start.distance) - start.distance;
      const auto upper = std::min(search_end, end.distance) - start.distance;
      if (lower > upper) {continue;}
      // Extended arithmetic keeps distant finite states from overflowing projection.
      const long double dx = static_cast<long double>(x) - start.pose.x;
      const long double dy = static_cast<long double>(y) - start.pose.y;
      const auto along = dx * ((end.pose.x - start.pose.x) / length) +
        dy * ((end.pose.y - start.pose.y) / length);
      const auto candidate = start.distance + static_cast<double>(
        std::clamp(along, static_cast<long double>(lower), static_cast<long double>(upper)));
      const auto pose = interpolate(index, candidate);
      const auto distance = distance_to(pose, x, y);
      if (distance < best_distance) {
        best_distance = distance;
        best_progress = candidate;
        best_segment = index;
      }
    }
    progress_ = best_progress;
    segment_ = best_segment;
    first_ = progress_ == points_.back().distance ? points_.back().pose :
      interpolate(segment_, progress_);
  }

  // Emit the projection followed by original vertices, clipping only the last segment.
  // The count excludes unused capacity. The visitor writes directly to the result buffer.
  template<class Visitor>
  std::size_t visit_pruned(std::size_t capacity, double max_length, Visitor && visitor) const
  {
    if (empty() || capacity == 0) {return 0;}
    visitor(0, first_);
    std::size_t count = 1;
    const auto end_distance = progress_ + std::min(max_length, remaining_distance());
    for (std::size_t index = segment_ + 1; index < points_.size() && count < capacity; ++index) {
      const auto & point = points_[index];
      if (point.distance <= progress_) {continue;}
      if (point.distance > end_distance) {
        if (end_distance > progress_) {visitor(count++, interpolate(index - 1, end_distance));}
        break;
      }
      visitor(count++, point.pose);
      if (point.distance == end_distance) {break;}
    }
    return count;
  }

private:
  struct Waypoint
  {
    Pose pose;
    double distance;
  };

  static long double distance_to(const Pose & pose, double x, double y) noexcept
  {
    return std::hypot(static_cast<long double>(pose.x) - x, static_cast<long double>(pose.y) - y);
  }

  Pose interpolate(std::size_t segment, double distance) const noexcept
  {
    const auto & start = points_[segment];
    if (segment + 1 == points_.size()) {return start.pose;}
    const auto & end = points_[segment + 1];
    const auto length = end.distance - start.distance;
    if (length <= 0.0) {return start.pose;}
    const auto fraction = std::clamp((distance - start.distance) / length, 0.0, 1.0);
    return {
      (1.0 - fraction) * start.pose.x + fraction * end.pose.x,
      (1.0 - fraction) * start.pose.y + fraction * end.pose.y,
      wrap_angle(start.pose.yaw + fraction * wrap_angle(end.pose.yaw - start.pose.yaw))};
  }

  std::vector<Waypoint> points_;
  std::size_t segment_ = 0;
  double progress_ = 0.0;
  Pose first_{0.0, 0.0, 0.0};
};
}  // namespace rpp_control
