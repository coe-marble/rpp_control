#pragma once

#include <rpp_cpp/list_definitions.hpp>
#include <rpp_interfaces/field_routing.hpp>
#include <rpp_interfaces/control_law_layout.hpp>
#include <rpp_schema/rpp_interfaces/ControlLawDesc.hpp>

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace rpp_control::detail
{
struct ComponentDescription
{
  std::string name, instance_id;
  rpp_interfaces::FieldLayout state;
};

struct ComponentBinding : ComponentDescription
{
  std::vector<std::size_t> state_route;
  rpp::List<double> state_values;

  void bind_state(const rpp_interfaces::FieldLayout & source)
  {
    state_route = rpp_interfaces::field_route(source, state);
    state_values.resize(state.size());
  }
};

struct ControlLawBinding : ComponentBinding
{
  rpp_interfaces::FieldLayout references, commands;
  std::vector<std::size_t> reference_route;
  rpp::List<double> reference_values, applied_command;

  void set_layouts(rpp_schema::rpp_interfaces::ControlLawDesc_Native && description)
  {
    const rpp_interfaces::ControlLawLayout layout(description);
    state = layout.state();
    references = layout.reference();
    commands = layout.control();
    applied_command.resize(commands.size());
  }
};
}  // namespace rpp_control::detail
