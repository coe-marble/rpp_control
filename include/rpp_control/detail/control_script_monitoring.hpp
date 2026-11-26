#pragma once

// Mission preparation and executor-side evaluation. Numeric step() stays unchanged.
namespace rpp_control
{
template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::prepare_mission(
  const std::string & goal_monitor,
  const std::string & generator, rpp::ListConst<double> final_target, bool new_mission)
{
  if (!configured_) {return rpp_interfaces::contract::CODE_NOT_READY;}
  return supervision_.prepare(goal_monitor, supervision_.monitor_names(), state_layout(),
      goal_layout(generator),
    generated_reference_layout(), final_target, route_revision_, new_mission);
}

template<bool Concurrent>
auto BasicControlScript<Concurrent>::supervise_mission(std::uint64_t time_ns)
-> typename MissionSupervision<Concurrent>::Evaluation
{
  return supervision_.evaluate(state(), references(), time_ns, route_revision_);
}
}  // namespace rpp_control
