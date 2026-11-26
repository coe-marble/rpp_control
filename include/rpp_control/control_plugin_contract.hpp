#pragma once

#include <rpp_interfaces/field_routing.hpp>
#include <rpp_interfaces/control_law_layout.hpp>
#include <rpp_schema/rpp_interfaces/ControlLawDesc.hpp>
#include <rpp_schema/rpp_interfaces/StateWithDescription.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rpp_control
{
inline void write_control_description(
  rpp_schema::rpp_interfaces::ControlLawDesc & desc,
  const rpp_schema::rpp_interfaces::ControlLawDesc_Native & source)
{
  const rpp_interfaces::ControlLawLayout prepared(source);
  auto state = desc.state();
  auto reference = desc.reference();
  auto control = desc.control();
  prepared.state().write(state);
  prepared.reference().write(reference);
  prepared.control().write(control);
  desc.horizonLength() = source.horizonLength;
  auto offsets = desc.horizonOffsetsNs();
  offsets.resize(source.horizonOffsetsNs.size());
  for (std::size_t index = 0; index < source.horizonOffsetsNs.size(); ++index) {
    offsets[index] = source.horizonOffsetsNs[index];
  }
}

inline rpp_schema::rpp_interfaces::ControlLawDesc::Const control_description(
  const rpp_interfaces::FieldLayout & state_layout, const rpp_interfaces::FieldLayout & reference_layout,
  const rpp_interfaces::FieldLayout & control_layout)
{
  rpp_schema::rpp_interfaces::ControlLawDesc_Native definition;
  definition.state.fields = state_layout.fields();
  definition.reference.fields = reference_layout.fields();
  definition.control.fields = control_layout.fields();
  definition.horizonLength = 0;
  rpp_schema::rpp_interfaces::ControlLawDesc desc;
  write_control_description(desc, definition);
  return rpp_schema::rpp_interfaces::ControlLawDesc::Const(std::move(desc));
}

inline bool same_field_layout(
  const rpp_interfaces::FieldLayout & saved, const rpp_interfaces::FieldLayout & expected)
{
  if (saved.fields().size() != expected.fields().size()) {return false;}
  for (std::size_t i = 0; i < saved.fields().size(); ++i) {
    if (!rpp_interfaces::same_field(saved, i, expected, i)) {return false;}
  }
  return true;
}

inline void validate_handover_values(
  const rpp_schema::rpp_interfaces::StateWithDescription::Const & state)
{
  const auto values = state.state();
  const rpp_interfaces::FieldLayout layout(state.desc().as_struct().fields);
  if (!values.empty() && (values.size() != layout.size() || !layout.valid_row(values))) {
    throw std::invalid_argument("Handover values do not match their field description");
  }
}

inline std::optional<std::vector<std::size_t>> handover_control_route(
  const rpp_interfaces::FieldLayout & available, const rpp_interfaces::FieldLayout & required)
{
  bool complete = true;
  for (std::size_t index = 0; index < required.fields().size(); ++index) {
    const auto found = rpp_interfaces::find_field(available, required.fields()[index].name);
    if (!found) {
      complete = false;
    } else if (!rpp_interfaces::same_field(available, *found, required, index)) {
      throw std::invalid_argument("Incompatible handover control field: " + required.fields()[index].name);
    }
  }
  // A newly activated stage may produce controls absent from the previous graph.
  // It starts without control history when its complete command is unavailable.
  if (!complete) {return std::nullopt;}
  return rpp_interfaces::field_route(available, required);
}

inline bool control_success(std::uint8_t status) noexcept
{
  return status == rpp_interfaces::contract::CODE_OK || status == rpp_interfaces::contract::CODE_LIMITED;
}
}  // namespace rpp_control
