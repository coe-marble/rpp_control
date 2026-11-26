#pragma once

#include <tuple>

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_interfaces/ReferenceGenerator.hpp>
#include <capnp_gen/rpp_interfaces/reference/constants.capnp.h>
#include <rpp_control/angle.hpp>
#include <rpp_interfaces/field_layout.hpp>
#include <rpp_interfaces/reference_horizon.hpp>
#include <rpp_control/numeric_output_pool.hpp>
#include <rpp_schema/rpp_interfaces/ReferenceGeneratorDesc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <new>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

class InterpolatingReference final : public rpp_interfaces::ReferenceGenerator
{
public:
  using ParameterDescription = rpp::params::ParameterDescription;
  using FieldsDesc = rpp_schema::rpp_interfaces::FieldListDesc_Native;
  using Description = rpp_schema::rpp_interfaces::ReferenceGeneratorDesc;
  using Sample = std::tuple<rpp::ListConst<double>, std::uint8_t>;

  RPP_PARAMETERS(
    ParameterDescription::create<FieldsDesc>("state"),
    ParameterDescription::create<std::vector<bool>>("targetMask"),
    ParameterDescription::create<FieldsDesc>("references"),
    ParameterDescription::create<std::int64_t>("max_points", 256),
    ParameterDescription::create<std::int64_t>("max_horizon_samples", 256),
    ParameterDescription::create<std::uint64_t>("path_segment_ns", 1000000000)
  )

  rpp::CallbackResult on_configure(const rpp::ComponentContext & context) override
  {
    if (active_) {throw std::logic_error("Deactivate before configuring reference generation");}
    const auto max_points = context.get_parameter<std::int64_t>("max_points");
    const auto max_horizon_samples = context.get_parameter<std::int64_t>("max_horizon_samples");
    const auto path_segment_ns = context.get_parameter<std::uint64_t>("path_segment_ns");
    if (max_points < 1 || max_points > 65536 ||
      max_horizon_samples < 1 || max_horizon_samples > 4096 || path_segment_ns < 1)
    {
      return rpp::CallbackResult::failure("Invalid reference capacity or path segment duration");
    }
    rpp_interfaces::FieldLayout state(context.get_parameter<FieldsDesc>("state").fields);
    rpp_interfaces::FieldLayout output(context.get_parameter<FieldsDesc>("references").fields);
    auto mask = context.get_parameter<std::vector<bool>>("targetMask");
    if (state.size() == 0 || output.size() == 0 || mask.size() != output.fields().size()) {
      return rpp::CallbackResult::failure("Provide nonempty state/references and one targetMask flag per reference field");
    }
    if (static_cast<std::size_t>(max_points) > max_values / output.size()) {
      return rpp::CallbackResult::failure("Reference waypoint storage exceeds capacity");
    }
    std::vector<Selection> selected;
    for (std::size_t i = 0; i < output.fields().size(); ++i) {
      const auto & reference = output.fields()[i];
      if (!mask[i]) {
        return rpp::CallbackResult::failure("Interpolating references must be selected as goal fields");
      }
      const auto & destination = output.ranges()[i];
      const auto found = std::find_if(state.fields().begin(), state.fields().end(),
        [&](const rpp_interfaces::Field & field) {return field.name == reference.name;});
      if (found == state.fields().end()) {
        // Reference-only fields, such as cascade selectors, need no measured source.
        selected.push_back({missing_source, destination.offset, destination.size, reference.kind});
        continue;
      }
      const auto index = static_cast<std::size_t>(found - state.fields().begin());
      const auto & source = state.ranges()[index];
      if (found->kind != reference.kind || found->quantity != reference.quantity ||
        found->frameId != reference.frameId || found->childFrameId != reference.childFrameId ||
        found->resource != reference.resource || source.size != destination.size)
      {
        return rpp::CallbackResult::failure("Reference is incompatible with its measured field: " + reference.name);
      }
      selected.push_back({source.offset, destination.offset, source.size, found->kind});
    }
    clear_goals();
    state_ = std::move(state);
    output_ = std::move(output);
    selected_ = std::move(selected);
    tracks_.clear();
    for (const auto & selection : selected_) {
      if (selection.kind == rpp_interfaces::contract::COORDINATE_QUATERNION) {
        tracks_.push_back({selection, {}, 0, false});
      } else {
        for (std::size_t axis = 0; axis < selection.size; ++axis) {
          const auto source = selection.source == missing_source ? missing_source : selection.source + axis;
          tracks_.push_back({{source, selection.destination + axis, 1, selection.kind}, {}, 0, false});
        }
      }
    }
    mask_ = std::move(mask);
    max_points_ = static_cast<std::size_t>(max_points);
    max_horizon_samples_ = static_cast<std::size_t>(max_horizon_samples);
    segment_ns_ = path_segment_ns;
    horizon_ = rpp_interfaces::ReferenceHorizon{};
    outputs_.clear();
    bound_ = false;
    configured_ = true;
    return {};
  }

  Description::Const getDesc() override
  {
    if (!configured_) {throw std::logic_error("Configure the reference generator first");}
    Description desc;
    auto state = desc.state();
    state_.write(state);
    auto reference = desc.reference();
    output_.write(reference);
    auto mask = desc.targetMask();
    mask.resize(mask_.size());
    for (std::size_t i = 0; i < mask_.size(); ++i) {mask[i] = mask_[i];}
    desc.features() = static_cast<std::uint8_t>(
      rpp_interfaces::contract::GENERATOR_SET_POINT | rpp_interfaces::contract::GENERATOR_SET_PATH | rpp_interfaces::contract::GENERATOR_SET_TRAJECTORY);
    return Description::Const(std::move(desc));
  }

  std::uint8_t bind(
    rpp::ListConst<std::uint64_t> horizonOffsetsNs, std::uint32_t horizonLength) override
  {
    if (!configured_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    if (horizonLength > max_horizon_samples_) {
      return rpp_interfaces::contract::CODE_INVALID_INPUT;
    }
    try {
      rpp_interfaces::ReferenceHorizon prepared(output_, horizonOffsetsNs, horizonLength);
      outputs_.prepare(prepared.value_count());
      horizon_ = std::move(prepared);
      bound_ = true;
      return rpp_interfaces::contract::CODE_OK;
    } catch (const std::invalid_argument &) {
      return rpp_interfaces::contract::CODE_INVALID_INPUT;
    } catch (const std::bad_alloc &) {
      return rpp_interfaces::contract::CODE_BUDGET_EXCEEDED;
    }
  }

  rpp::CallbackResult on_activate() override
  {
    if (!configured_ || !bound_) {
      return rpp::CallbackResult::failure("Bind the controller horizon before activation");
    }
    active_ = true;
    return {};
  }
  rpp::CallbackResult on_deactivate() override
  {
    active_ = false;
    return {};
  }
  rpp::CallbackResult on_cleanup() override
  {
    on_deactivate();
    clear_goals();
    tracks_.clear();
    configured_ = false;
    bound_ = false;
    horizon_ = rpp_interfaces::ReferenceHorizon{};
    outputs_.clear();
    state_ = rpp_interfaces::FieldLayout{};
    output_ = rpp_interfaces::FieldLayout{};
    selected_.clear();
    mask_.clear();
    return {};
  }

  std::uint8_t setPoint(rpp::ListConst<double> state, rpp::ListConst<bool> updateMask) override
  {
    if (!configured_ || !bound_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    if (state.size() != output_.size()) {return rpp_interfaces::contract::CODE_INVALID_INPUT;}
    return stage(state, nullptr, updateMask);
  }
  std::uint8_t setPath(rpp::ListConst<double> states, rpp::ListConst<bool> updateMask) override
  {
    if (!configured_ || !bound_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    return stage(states, nullptr, updateMask);
  }
  std::uint8_t setTrajectory(rpp::ListConst<double> states,
    rpp::ListConst<std::uint64_t> times, rpp::ListConst<bool> updateMask) override
  {
    if (!configured_ || !bound_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    return stage(states, &times, updateMask);
  }
  std::uint8_t cancel() override
  {
    if (!configured_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    clear_goals();
    return rpp_interfaces::contract::CODE_OK;
  }

  Sample sample(rpp::ListConst<double> state, std::uint64_t time_ns) override
  {
    if (!active_ || !bound_) {return failure(rpp_interfaces::contract::CODE_NOT_READY);}
    if (state.size() != state_.size() || !state_.valid_row(state)) {
      return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
    }
    if (has_sample_time_ && time_ns < last_sample_ns_) {
      return failure(rpp_interfaces::contract::CODE_STALE_INPUT);
    }
    if (horizon_.maximum_offset() > std::numeric_limits<std::uint64_t>::max() - time_ns) {
      return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
    }
    for (const auto & track : tracks_) {
      if (track.goal && !track.changed && time_ns < track.start_ns) {
        return failure(rpp_interfaces::contract::CODE_STALE_INPUT);
      }
    }
    const auto buffer = outputs_.acquire();
    if (buffer == rpp_control::NumericOutputPool::unavailable) {return failure(rpp_interfaces::contract::CODE_NOT_READY);}
    auto & values = outputs_.values(buffer);
    const auto sample_count = horizon_.sample_count();
    for (auto & track : tracks_) {
      if (track.changed) {
        track.start_ns = time_ns;
        track.changed = false;
      }
      if (!track.goal) {
        // Unset references retain valid coordinates for the caller's validity mask.
        for (std::size_t sample = 0; sample < sample_count; ++sample) {
          for (std::size_t axis = 0; axis < track.selection.size; ++axis) {
            double value = 0.0;
            if (track.selection.source != missing_source) {
              value = state[track.selection.source + axis];
            } else if (track.selection.kind == rpp_interfaces::contract::COORDINATE_QUATERNION && axis == 3) {
              value = 1.0;
            }
            values[(track.selection.destination + axis) * sample_count + sample] = value;
          }
        }
        continue;
      }
      for (std::size_t sample = 0; sample < sample_count; ++sample) {
        const auto sample_ns = time_ns + horizon_.offset(track.selection.destination, sample);
        // Tracks contain one scalar or four synchronized quaternion coefficients.
        std::array<double, 4> interpolated{};
        interpolate(*track.goal, sample_ns - track.start_ns, track.selection, interpolated);
        for (std::size_t axis = 0; axis < track.selection.size; ++axis) {
          values[(track.selection.destination + axis) * sample_count + sample] = interpolated[axis];
        }
      }
    }
    last_sample_ns_ = time_ns;
    has_sample_time_ = true;
    return {outputs_.view(buffer), rpp_interfaces::contract::CODE_OK};
  }

private:
  struct Selection
  {
    std::size_t source;
    std::size_t destination;
    std::size_t size;
    std::uint8_t kind;
  };
  struct Goal
  {
    std::vector<double> points;
    std::vector<std::uint64_t> times;
  };
  struct Track
  {
    Selection selection;
    std::shared_ptr<const Goal> goal;
    std::uint64_t start_ns;
    bool changed;
  };
  static constexpr std::size_t max_values = 1024 * 1024;
  static constexpr std::size_t missing_source = std::numeric_limits<std::size_t>::max();

  static double linear_interpolation(double a, double b, double alpha) noexcept
  {
    if ((a <= 0 && b >= 0) || (a >= 0 && b <= 0)) {return (1 - alpha) * a + alpha * b;}
    return std::clamp(a + alpha * (b - a), std::min(a, b), std::max(a, b));
  }
  rpp_interfaces::FieldLayout state_;
  rpp_interfaces::FieldLayout output_;
  std::vector<Selection> selected_;
  std::vector<bool> mask_;
  rpp_interfaces::ReferenceHorizon horizon_;
  rpp_control::NumericOutputPool outputs_;
  bool configured_ = false;
  bool bound_ = false;
  bool active_ = false;
  std::vector<Track> tracks_;
  bool has_sample_time_ = false;
  std::uint64_t last_sample_ns_ = 0;
  std::size_t max_points_ = 0;
  std::size_t max_horizon_samples_ = 0;
  std::uint64_t segment_ns_ = 0;

  Sample failure(std::uint8_t status) const {return {outputs_.empty(), status};}

  void clear_goals()
  {
    for (auto & track : tracks_) {track.goal.reset(); track.changed = false;}
    has_sample_time_ = false;
  }

  std::uint8_t stage(
    rpp::ListConst<double> states, const rpp::ListConst<std::uint64_t> * times,
    rpp::ListConst<bool> update_mask)
  {
    if ((!update_mask.empty() && update_mask.size() != output_.size()) ||
      states.empty() || states.size() % output_.size() != 0) {return rpp_interfaces::contract::CODE_INVALID_INPUT;}
    const auto count = states.size() / output_.size();
    if (count > max_points_) {return rpp_interfaces::contract::CODE_INVALID_INPUT;}
    if (times) {
      if (times->size() != count || (*times)[0] != 0) {return rpp_interfaces::contract::CODE_INVALID_INPUT;}
      for (std::size_t i = 0; i < count; ++i) {
        if (i > 0 && (*times)[i] <= (*times)[i - 1])
        {
          return rpp_interfaces::contract::CODE_INVALID_INPUT;
        }
      }
      for (const auto & track : tracks_) {
        if (!update_mask.empty() && !update_mask[track.selection.destination]) {continue;}
        if ((*times)[count - 1] < horizon_.offset(track.selection.destination, horizon_.sample_count() - 1)) {
          return rpp_interfaces::contract::CODE_INVALID_INPUT;
        }
      }
    } else if (count - 1 > std::numeric_limits<std::uint64_t>::max() / segment_ns_) {
      return rpp_interfaces::contract::CODE_INVALID_INPUT;
    }
    for (std::size_t row = 0; row < count; ++row) {
      if (!output_.valid_update(states, update_mask, row * output_.size())) {return rpp_interfaces::contract::CODE_INVALID_INPUT;}
    }
    try {
      auto prepared = std::make_shared<Goal>();
      prepared->points.resize(count * output_.size());
      prepared->times.resize(count);
      for (std::size_t row = 0; row < count; ++row) {
        prepared->times[row] = times ? (*times)[row] : row * segment_ns_;
        write_row(*prepared, states, row, update_mask);
      }
      // Partial updates retain older curves. Bound the aggregate retained storage
      // as well as each individual request, counting shared curves only once.
      std::size_t retained_values = prepared->points.size();
      std::set<const Goal *> retained;
      for (const auto & track : tracks_) {
        if (update_mask.empty() || update_mask[track.selection.destination] || !track.goal) {continue;}
        if (retained.insert(track.goal.get()).second) {retained_values += track.goal->points.size();}
        if (retained_values > max_values) {return rpp_interfaces::contract::CODE_BUDGET_EXCEEDED;}
      }
      for (auto & track : tracks_) {
        if (!update_mask.empty() && !update_mask[track.selection.destination]) {continue;}
        track.goal = prepared;
        track.changed = true;
      }
      return rpp_interfaces::contract::CODE_OK;
    } catch (const std::bad_alloc &) {
      return rpp_interfaces::contract::CODE_BUDGET_EXCEEDED;
    }
  }

  void write_row(Goal & prepared, rpp::ListConst<double> states, std::size_t row,
    rpp::ListConst<bool> update_mask) const
  {
    for (const auto & selection : selected_) {
      const auto destination = row * output_.size() + selection.destination;
      if (selection.kind == rpp_interfaces::contract::COORDINATE_QUATERNION &&
        !update_mask.empty() && !update_mask[selection.destination]) {continue;}
      for (std::size_t i = 0; i < selection.size; ++i) {
        if (update_mask.empty() || update_mask[selection.destination + i]) {prepared.points[destination + i] = states[destination + i];}
      }
      if (selection.kind == rpp_interfaces::contract::COORDINATE_CONTINUOUS_ANGLE) {
        for (std::size_t i = 0; i < selection.size; ++i) {
          prepared.points[destination + i] = rpp_control::wrap_angle(prepared.points[destination + i]);
        }
      } else if (selection.kind == rpp_interfaces::contract::COORDINATE_QUATERNION) {
        double norm = 0.0;
        double dot = 0.0;
        for (std::size_t i = 0; i < 4; ++i) {
          const auto value = prepared.points[destination + i];
          norm += value * value;
          if (row > 0) {dot += value * prepared.points[destination - output_.size() + i];}
        }
        const auto scale = (row > 0 && dot < 0.0 ? -1.0 : 1.0) / std::sqrt(norm);
        for (std::size_t i = 0; i < 4; ++i) {prepared.points[destination + i] *= scale;}
      }
    }
  }

  void interpolate(const Goal & goal, std::uint64_t time, const Selection & selection,
    std::array<double, 4> & values) const
  {
    const auto width = output_.size();
    if (time >= goal.times.back()) {
      const auto source = (goal.times.size() - 1) * width + selection.destination;
      for (std::size_t i = 0; i < selection.size; ++i) {
        values[i] = goal.points[source + i];
      }
      return;
    }
    const auto upper = std::upper_bound(goal.times.begin(), goal.times.end(), time);
    const auto right = static_cast<std::size_t>(upper - goal.times.begin());
    const auto left = right - 1;
    const auto alpha = static_cast<double>(time - goal.times[left]) /
      static_cast<double>(goal.times[right] - goal.times[left]);
    const auto a = left * width + selection.destination;
    const auto b = right * width + selection.destination;
    switch (selection.kind) {
      case rpp_interfaces::contract::COORDINATE_CONTINUOUS_ANGLE:
        for (std::size_t i = 0; i < selection.size; ++i) {
          values[i] = rpp_control::wrap_angle(goal.points[a + i] +
            alpha * rpp_control::wrap_angle(goal.points[b + i] - goal.points[a + i]));
        }
        break;
      case rpp_interfaces::contract::COORDINATE_QUATERNION: {
        double dot = 0.0;
        for (std::size_t i = 0; i < 4; ++i) {dot += goal.points[a + i] * goal.points[b + i];}
        dot = std::clamp(dot, 0.0, 1.0);
        double wa = 1.0 - alpha;
        double wb = alpha;
        if (dot < 0.9995) {
          const auto angle = std::acos(dot);
          const auto denominator = std::sin(angle);
          wa = std::sin((1.0 - alpha) * angle) / denominator;
          wb = std::sin(alpha * angle) / denominator;
        }
        double q[4];
        double norm = 0.0;
        for (std::size_t i = 0; i < 4; ++i) {
          q[i] = wa * goal.points[a + i] + wb * goal.points[b + i];
          norm += q[i] * q[i];
        }
        const auto scale = 1.0 / std::sqrt(norm);
        for (std::size_t i = 0; i < 4; ++i) {values[i] = q[i] * scale;}
        break;
      }
      default:
        for (std::size_t i = 0; i < selection.size; ++i) {
          values[i] = linear_interpolation(goal.points[a + i], goal.points[b + i], alpha);
        }
    }
  }
};
