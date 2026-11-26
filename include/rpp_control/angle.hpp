#pragma once

#include <cmath>

namespace rpp_control
{
inline double wrap_angle(double angle) noexcept
{
  constexpr double pi = 3.14159265358979323846;
  angle = std::fmod(angle, 2 * pi);
  if (angle > pi) {angle -= 2 * pi;}
  if (angle < -pi) {angle += 2 * pi;}
  return angle;
}
}  // namespace rpp_control
