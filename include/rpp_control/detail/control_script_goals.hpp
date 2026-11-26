#pragma once

#include <chrono>

// Goal submission, validation, expansion, cancellation, and deadlines.
namespace rpp_control
{
template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::set_point(const std::string & name,
  rpp::ListConst<double> values, rpp::ListConst<bool> mask, std::uint64_t deadline_ns)
{
  return stage_goal(name, values, mask, deadline_ns, rpp_interfaces::contract::GENERATOR_SET_POINT);
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::set_path(const std::string & name,
  rpp::ListConst<double> values, rpp::ListConst<bool> mask, std::uint64_t deadline_ns)
{
  return stage_goal(name, values, mask, deadline_ns, rpp_interfaces::contract::GENERATOR_SET_PATH);
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::set_trajectory(const std::string & name,
  rpp::ListConst<double> values, rpp::ListConst<std::uint64_t> times_ns, rpp::ListConst<bool> mask,
  std::uint64_t deadline_ns)
{
  return stage_goal(
    name, values, mask, deadline_ns, rpp_interfaces::contract::GENERATOR_SET_TRAJECTORY, &times_ns);
}

template<bool Concurrent>
void BasicControlScript<Concurrent>::cancel()
{
  for (auto & generator : reference_generators_) {
    std::fill(generator.deadlines.begin(), generator.deadlines.end(), 0);
    (void)generator.plugin->cancel();
  }
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::cancel(const std::string & name)
{
  const auto found = std::find_if(reference_generators_.begin(), reference_generators_.end(),
    [&](const auto & generator) {return generator.name == name;});
  if (found == reference_generators_.end()) {return rpp_interfaces::contract::CODE_INVALID_INPUT;}
  std::fill(found->deadlines.begin(), found->deadlines.end(), 0);
  return found->plugin->cancel();
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::refresh_goal(const std::string & name,
  rpp::ListConst<bool> mask, std::uint64_t now_ns, std::uint64_t deadline_ns)
{
  const auto found = std::find_if(reference_generators_.begin(), reference_generators_.end(),
    [&](const auto & generator) {return generator.name == name;});
  if (found == reference_generators_.end() || deadline_ns <= now_ns ||
    (!mask.empty() && mask.size() != found->deadlines.size())) {
    return rpp_interfaces::contract::CODE_INVALID_INPUT;
  }
  for (std::size_t axis = 0; axis < found->deadlines.size(); ++axis) {
    if ((mask.empty() || mask[axis]) && found->deadlines[axis] <= now_ns) {
      // A heartbeat never revives an expired goal or restarts its plugin trajectory.
      return rpp_interfaces::contract::CODE_STALE_INPUT;
    }
  }
  for (std::size_t axis = 0; axis < found->deadlines.size(); ++axis) {
    if (mask.empty() || mask[axis]) {found->deadlines[axis] = deadline_ns;}
  }
  return rpp_interfaces::contract::CODE_OK;
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::goal_features(const std::string & name) const
{
  const auto found = std::find_if(reference_generators_.begin(), reference_generators_.end(),
    [&](const auto & generator) {return generator.name == name;});
  return found == reference_generators_.end() ? 0 : found->features;
}

template<bool Concurrent>
bool BasicControlScript<Concurrent>::generator_has_goal(const ReferenceGeneratorBinding & generator)
{
  // Generators without targets produce continuous references while selected.
  return generator.targets.size() == 0 ||
    std::any_of(generator.deadlines.begin(), generator.deadlines.end(),
      [](auto deadline) { return deadline != 0; });
}

template<bool Concurrent>
bool BasicControlScript<Concurrent>::has_active_goal() const
{
  return std::any_of(reference_generators_.begin(), reference_generators_.end(),
           [](const auto & generator) {
             return generator.selected && generator_has_goal(generator);
           }) ||
    std::any_of(controllers_.begin(), controllers_.end(), [](const auto & controller) {
      return controller.layout.reference().size() == 0 && controller.commands_active;
    });
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::stage_goal(const std::string & name,
  rpp::ListConst<double> values, rpp::ListConst<bool> mask, std::uint64_t deadline_ns,
  std::uint8_t feature, const rpp::ListConst<std::uint64_t> * times_ns)
{
  const auto reject = [&](std::uint8_t code, const char * reason) noexcept {
    try {
      RPP_LOG_WARN_THROTTLE(*context_.get_logger(), std::chrono::seconds(1),
        "Goal for generator '%s' rejected (code %u): %s",
        name.c_str(), static_cast<unsigned>(code), reason);
    } catch (...) {
      // Logging must preserve the rejection result, including allocation failures.
    }
    return code;
  };

  if (!configured_) {
    return reject(rpp_interfaces::contract::CODE_NOT_READY, "control script is not configured");
  }
  const auto found = std::find_if(reference_generators_.begin(), reference_generators_.end(),
    [&](const auto & candidate) { return candidate.name == name; });
  if (found == reference_generators_.end()) {
    return reject(rpp_interfaces::contract::CODE_INVALID_INPUT, "unknown reference generator");
  }
  auto & generator = *found;
  const auto & layout = generator.targets;
  if (!generator.routed) {
    return reject(rpp_interfaces::contract::CODE_NOT_READY, "generator routes are unavailable");
  }
  if (!(generator.features & feature)) {
    return reject(rpp_interfaces::contract::CODE_UNSUPPORTED, "goal operation is not supported");
  }

  if (deadline_ns == 0) {
    return reject(rpp_interfaces::contract::CODE_INVALID_INPUT, "deadline must be nonzero");
  }
  if (layout.size() == 0) {
    return reject(rpp_interfaces::contract::CODE_INVALID_INPUT, "generator has no target fields");
  }
  if (values.empty() || values.size() > rpp_interfaces::ReferenceHorizon::max_values ||
    values.size() % layout.size() != 0) {
    return reject(rpp_interfaces::contract::CODE_INVALID_INPUT,
      "values must contain complete target rows within capacity");
  }
  if (!mask.empty() && mask.size() != layout.size()) {
    return reject(rpp_interfaces::contract::CODE_INVALID_INPUT,
      "mask width does not match the target layout");
  }

  const auto rows = values.size() / layout.size();
  if (feature == rpp_interfaces::contract::GENERATOR_SET_POINT && rows != 1) {
    return reject(rpp_interfaces::contract::CODE_INVALID_INPUT, "point goals require one row");
  }
  for (std::size_t row = 0; row < rows; ++row) {
    if (!layout.valid_update(values, mask, row * layout.size())) {
      return reject(rpp_interfaces::contract::CODE_INVALID_INPUT,
        "invalid coordinate values or incomplete quaternion update");
    }
  }

  if (times_ns) {
    if (times_ns->size() != rows || (*times_ns)[0] != 0) {
      return reject(rpp_interfaces::contract::CODE_INVALID_INPUT,
        "trajectory times must match the rows and start at zero");
    }
    for (std::size_t row = 1; row < rows; ++row) {
      if ((*times_ns)[row] <= (*times_ns)[row - 1]) {
        return reject(rpp_interfaces::contract::CODE_INVALID_INPUT,
          "trajectory times must increase strictly");
      }
    }
    for (std::size_t scalar = 0; scalar < layout.size(); ++scalar) {
      if (!mask.empty() && !mask[scalar]) {continue;}
      const auto last_offset = generator.horizon.offset(generator.target_reference_route[scalar],
        generator.horizon.sample_count() - 1);
      if ((*times_ns)[rows - 1] < last_offset) {
        return reject(rpp_interfaces::contract::CODE_INVALID_INPUT,
          "trajectory does not cover the bound reference horizon");
      }
    }
  }

  GeneratorGoal goal;
  const auto preparation_status = prepare_goal(generator, values, mask, rows, goal);
  if (!control_success(preparation_status)) {
    return reject(preparation_status,
      preparation_status == rpp_interfaces::contract::CODE_BUDGET_EXCEEDED
        ? "goal allocation failed" : "expanded goal exceeds generator capacity");
  }
  if (times_ns) {
    goal.arrival_times_ns.resize(rows);
    for (std::size_t row = 0; row < rows; ++row) {
      goal.arrival_times_ns[row] = (*times_ns)[row];
    }
  }

  // A failed setter may have changed plugin state. Invalidate only the submitted fields.
  for (std::size_t scalar = 0; scalar < layout.size(); ++scalar) {
    if (mask.empty() || mask[scalar]) {
      generator.deadlines[scalar] = 0;
    }
  }

  // Plugin exceptions are fatal to the caller; only returned statuses are rejections.
  std::uint8_t code;
  if (feature == rpp_interfaces::contract::GENERATOR_SET_POINT) {
    code = generator.plugin->setPoint(goal.values, goal.mask);
  } else if (feature == rpp_interfaces::contract::GENERATOR_SET_PATH) {
    code = generator.plugin->setPath(goal.values, goal.mask);
  } else {
    code = generator.plugin->setTrajectory(goal.values, goal.arrival_times_ns, goal.mask);
  }
  if (!control_success(code)) {
    return reject(code, "reference generator rejected the goal");
  }

  // Commit deadlines only after the generator accepts the update.
  for (std::size_t scalar = 0; scalar < layout.size(); ++scalar) {
    if (mask.empty() || mask[scalar]) {
      generator.deadlines[scalar] = deadline_ns;
      auto & timing = generator.trajectory_timing[scalar];
      timing = {};
      if (times_ns) {
        timing.timed = timing.pending = true;
        timing.duration_ns = (*times_ns)[rows - 1];
      }
    }
  }
  generator.timing_pending = std::any_of(generator.trajectory_timing.begin(),
    generator.trajectory_timing.end(), [](const auto & timing) {return timing.pending;});
  return code;
}

template<bool Concurrent>
std::string BasicControlScript<Concurrent>::trajectory_warning() const
{
  for (const auto & generator : reference_generators_) {
    if (!generator.selected || !control_success(generator.status)) {continue;}
    for (std::size_t scalar = 0; scalar < generator.trajectory_timing.size(); ++scalar) {
      const auto & timing = generator.trajectory_timing[scalar];
      if (!timing.timed || timing.pending || generator.deadlines[scalar] == 0 ||
        generator.last_sample_ns < timing.origin_ns) {continue;}
      const auto elapsed = generator.last_sample_ns - timing.origin_ns;
      const auto offset = generator.horizon.offset(generator.target_reference_route[scalar],
        generator.horizon.sample_count() - 1);
      if (elapsed > timing.duration_ns || offset > timing.duration_ns - elapsed) {
        return generator.name;
      }
    }
  }
  return {};
}

template<bool Concurrent>
std::uint8_t BasicControlScript<Concurrent>::prepare_goal(const ReferenceGeneratorBinding & generator,
  rpp::ListConst<double> values, rpp::ListConst<bool> mask, std::size_t rows,
  GeneratorGoal & goal) const
{
  if (generator.outputs.size() == 0 ||
    rows > rpp_interfaces::ReferenceHorizon::max_values / generator.outputs.size()) {
    return rpp_interfaces::contract::CODE_INVALID_INPUT;
  }

  try {
    goal.values.resize(rows * generator.outputs.size());
    goal.mask.resize(generator.outputs.size());
  } catch (const std::bad_alloc &) {
    return rpp_interfaces::contract::CODE_BUDGET_EXCEEDED;
  }
  for (std::size_t scalar = 0; scalar < generator.outputs.size(); ++scalar) {
    goal.mask[scalar] = false;
  }

  // Goal rows are waypoint-major, unlike the reference-major sampled horizon.
  for (std::size_t scalar = 0; scalar < generator.targets.size(); ++scalar) {
    const auto destination = generator.target_reference_route[scalar];
    goal.mask[destination] = mask.empty() || mask[scalar];
    if (!mask.empty() && !mask[scalar]) {
      continue;
    }
    for (std::size_t row = 0; row < rows; ++row) {
      goal.values[row * generator.outputs.size() + destination] =
        values[row * generator.targets.size() + scalar];
    }
  }
  return rpp_interfaces::contract::CODE_OK;
}
} // namespace rpp_control
