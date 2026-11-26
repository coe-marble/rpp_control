#pragma once

#include <rpp_interfaces/field_layout.hpp>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace rpp_control
{
// Shared by the path generator and its consuming controller; these are spatial
// samples inside one flat reference, independent of a controller's time horizon.
struct PlanarPathLayout
{
  static constexpr std::size_t max_capacity = 512;

  explicit PlanarPathLayout(std::size_t capacity) : capacity(capacity)
  {
    if (capacity < 2 || capacity > max_capacity) {
      throw std::invalid_argument("Planar path capacity must be between 2 and 512 points");
    }
  }

  std::vector<rpp_interfaces::Field> fields(
    const std::string & resource, const std::string & frame, const std::string & child) const
  {
    using namespace rpp_interfaces::contract;
    const auto count = static_cast<std::uint32_t>(capacity);
    return {
      {resource + ".position", COORDINATE_EUCLIDEAN, QUANTITY_LINEAR_POSITION, 2, frame, child, resource},
      {resource + ".orientation", COORDINATE_CONTINUOUS_ANGLE, QUANTITY_ANGULAR_POSITION, 1, frame, child, resource},
      {resource + ".path.position", COORDINATE_EUCLIDEAN, QUANTITY_LINEAR_POSITION, 2 * count, frame, child, resource},
      {resource + ".path.orientation", COORDINATE_CONTINUOUS_ANGLE, QUANTITY_ANGULAR_POSITION, count, frame, child, resource},
      {resource + ".path.count", COORDINATE_EUCLIDEAN, QUANTITY_NORMALIZED, 1, frame, child, resource},
      {resource + ".path.remainingDistance", COORDINATE_EUCLIDEAN, QUANTITY_LINEAR_POSITION, 1, frame, child, resource}};
  }

  std::size_t positions_offset() const noexcept {return 3;}
  std::size_t orientations_offset() const noexcept {return 3 + 2 * capacity;}
  std::size_t count_offset() const noexcept {return 3 + 3 * capacity;}
  std::size_t remaining_distance_offset() const noexcept {return 4 + 3 * capacity;}
  std::size_t size() const noexcept {return 5 + 3 * capacity;}

  std::size_t capacity;
};
}  // namespace rpp_control
