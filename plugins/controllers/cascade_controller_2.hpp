#pragma once

#include <rpp_control/cascade_control_law.hpp>

class CascadeController2 final : public rpp_control::CascadeControlLaw<2>
{
public:
  using CascadeControlLaw::CascadeControlLaw;

  RPP_PARAMETERS(
    rpp::params::ParameterDescription::create<bool>("expose_inner_references", false))
  RPP_COMPONENTS(
    {"ctl0", "rpp_interfaces::ControlLaw"},
    {"ctl1", "rpp_interfaces::ControlLaw"}
  )
};
