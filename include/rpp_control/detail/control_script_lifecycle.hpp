#pragma once

// Component activation, deactivation, instance guards, and lifecycle status selection.
namespace rpp_control
{
template<bool Concurrent>
void BasicControlScript<Concurrent>::activate()
{
  if (!report_binding_error_) {
    context_.activate_subcomponents().require_success("Control script activation");
  } else {
    for (const auto & slot : context_.list_subcomponents()) {
      for (const auto & component : runtime_.get_subcomponent_statuses(slot)) {
        if (component.state != rpp::LifecycleState::INACTIVE ||
          binding_rejected(slot, component.name))
        {
          continue;
        }
        const auto result = runtime_.activate_component(slot, component.name);
        if (!result.success) {
          report_binding_error_(slot + std::string(" '") + component.name + "': " + result.message);
        }
      }
    }
  }

  // Rebuilding replaces the stored safe vector, so pass an independent copy.
  const auto safe_values = safe_actuator_values_;
  rebuild_active_routes(safe_values);
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::deactivate()
{
  cancel();
  clear_outputs();
  context_.deactivate_subcomponents().require_success("Control script deactivation");
}

template<bool Concurrent>
bool BasicControlScript<Concurrent>::all_active() const
{
  if (!configured_ || !state_source_active() || !rejected_bindings_.empty()) {
    return false;
  }
  for (const auto & generator : reference_generators_) {
    if (generator.selected &&
      (!generator.routed ||
      !component_active<rpp_interfaces::ReferenceGenerator>(reference_slot_, generator)))
    {
      return false;
    }
  }
  for (const auto & controller : controllers_) {
    if (!controller.routed ||
      !component_active<rpp_interfaces::ControlLaw>(controller_slot_, controller))
    {
      return false;
    }
  }
  for (const auto & allocator : allocators_) {
    if (!allocator.routed ||
      !component_active<rpp_interfaces::ActuationMapper>(allocator_slot_, allocator))
    {
      return false;
    }
  }
  return true;
}

template<bool Concurrent>
template<class Plugin>
bool BasicControlScript<Concurrent>::component_active(
  const std::string & slot, const ComponentRoutes & component) const
{
  return component_active<Plugin>(slot, component.name, component.instance_id);
}

template<bool Concurrent>
template<class Plugin>
bool BasicControlScript<Concurrent>::component_active(
  const std::string & slot, const std::string & name, const std::string & instance_id) const
{
  // Match the prepared instance so a replacement under the same name cannot use stale routes.
  const auto result =
    context_.template with_active_component<Plugin>(slot, name, [](auto &) {}, instance_id);
  return result.first;
}

template<bool Concurrent>
bool BasicControlScript<Concurrent>::state_source_active() const
{
  return !state_provider_name_.empty() &&
         component_active<rpp_interfaces::StateProvider>(
      state_slot_, state_provider_name_, state_provider_instance_);
}

template<bool Concurrent>
template<class Include>
auto BasicControlScript<Concurrent>::components_matching(const Include & include) const
-> ComponentStatuses
{
  ComponentStatuses components;
  for (const auto & slot : context_.list_subcomponents()) {
    auto & selected = components[slot];
    for (const auto & component : runtime_.get_subcomponent_statuses(slot)) {
      if (include(component)) {
        selected.push_back(component);
      }
    }
  }
  return components;
}

template<bool Concurrent>
auto BasicControlScript<Concurrent>::active_components() const -> ComponentStatuses
{
  return components_matching(
    [](const auto & component) {return component.state == rpp::LifecycleState::ACTIVE;});
}

template<bool Concurrent>
template<class Plugin>
auto BasicControlScript<Concurrent>::component_instance(
  const std::string & slot, const std::string & name) -> std::shared_ptr<Plugin>
{
  return context_.get_subcomponent_context(slot, name).template get_instance<Plugin>();
}
} // namespace rpp_control
