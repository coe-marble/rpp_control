#pragma once

#include <rpp_interfaces/field_routing.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

namespace rpp_control::detail
{
using rpp_interfaces::gather_fields;
using rpp_interfaces::scatter_fields;

template<typename Values>
inline bool all_finite(const Values & values)
{
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!std::isfinite(static_cast<double>(values[i]))) {return false;}
  }
  return true;
}
}  // namespace rpp_control::detail
