#pragma once

#include <rpp_control/auxiliary_state.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace rpp_control
{
// Controller history uses stable control.* field names on top of generic snapshot assembly.
class ControlAuxiliaryState : public AuxiliaryState
{
public:
  explicit ControlAuxiliaryState(std::uint64_t stamp_ns = 0)
  : AuxiliaryState(stamp_ns)
  {}

  ControlAuxiliaryState & error(const rpp_interfaces::FieldLayout & axes, Values values = {})
  {
    return append("control.error", axes, std::move(values));
  }

  ControlAuxiliaryState & integral(const rpp_interfaces::FieldLayout & axes, Values values = {})
  {
    return append("control.integral", axes, std::move(values));
  }

  ControlAuxiliaryState & previous_error(
    const rpp_interfaces::FieldLayout & axes, Values values = {})
  {
    return append("control.previousError", axes, std::move(values));
  }

  ControlAuxiliaryState & previous_derivative(
    const rpp_interfaces::FieldLayout & axes, Values values = {})
  {
    return append("control.previousDerivative", axes, std::move(values));
  }

  ControlAuxiliaryState & previous_control(
    const rpp_interfaces::FieldLayout & inputs, Values values = {})
  {
    return append("control.previousControl", inputs, std::move(values));
  }

  ControlAuxiliaryState & horizon_inputs(
    const rpp_interfaces::FieldLayout & inputs,
    rpp::ListConst<std::uint64_t> offsets_ns, Values values = {})
  {
    if (!values) {
      return *this;
    }
    if (inputs.size() == 0 || offsets_ns.empty() || offsets_ns[0] != 0 ||
      values->size() % inputs.size() != 0 || values->size() / inputs.size() != offsets_ns.size())
    {
      throw std::invalid_argument("Warm-start inputs must contain one row per horizon offset");
    }
    auto prepared = *this;
    for (std::size_t row = 0; row < offsets_ns.size(); ++row) {
      if (row > 0 && offsets_ns[row] <= offsets_ns[row - 1]) {
        throw std::invalid_argument("Warm-start horizon offsets must increase strictly");
      }
      prepared.append_row("control.horizonInput." + std::to_string(offsets_ns[row]), inputs,
        *values, row * inputs.size());
    }
    // Rejecting a later horizon row must leave the previously assembled state intact.
    *this = std::move(prepared);
    return *this;
  }

  ControlAuxiliaryState & append(
    const std::string & name, const rpp_interfaces::FieldLayout & axes, Values values = {})
  {
    AuxiliaryState::append(name, axes, std::move(values));
    return *this;
  }
};
} // namespace rpp_control
