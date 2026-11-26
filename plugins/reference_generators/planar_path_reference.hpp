#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_interfaces/ReferenceGenerator.hpp>
#include <capnp_gen/rpp_interfaces/reference/constants.capnp.h>
#include <rpp_control/numeric_output_pool.hpp>
#include <rpp_control/planar_path.hpp>
#include <rpp_control/planar_path_layout.hpp>
#include <rpp_schema/rpp_interfaces/ReferenceGeneratorDesc.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <new>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

class PlanarPathReference final : public rpp_interfaces::ReferenceGenerator
{
public:
  using ParameterDescription = rpp::params::ParameterDescription;
  using Description = rpp_schema::rpp_interfaces::ReferenceGeneratorDesc;
  using FieldsDesc = rpp_schema::rpp_interfaces::FieldListDesc_Native;
  using Sample = std::tuple<rpp::ListConst<double>, std::uint8_t>;

  RPP_PARAMETERS(
    ParameterDescription::create<std::string>("resource"),
    ParameterDescription::create<std::string>("frame_id", "map"),
    ParameterDescription::create<std::string>("child_frame_id", "base_link"),
    ParameterDescription::create<std::int64_t>("path_capacity", 256),
    ParameterDescription::create<std::int64_t>("max_points", 1024),
    ParameterDescription::create<double>("max_search_distance", 20.0),
    ParameterDescription::create<double>("max_path_length", 100.0),
    ParameterDescription::create<FieldsDesc>("additional_targets")
  )

  rpp::CallbackResult on_configure(const rpp::ComponentContext & context) override
  {
    if (active_) {
      return rpp::CallbackResult::failure("Deactivate the path reference generator before configuring");
    }
    const auto resource = context.get_parameter<std::string>("resource");
    const auto frame = context.get_parameter<std::string>("frame_id");
    const auto child = context.get_parameter<std::string>("child_frame_id");
    const auto capacity = context.get_parameter<std::int64_t>("path_capacity");
    const auto max_points = context.get_parameter<std::int64_t>("max_points");
    const auto search_distance = context.get_parameter<double>("max_search_distance");
    const auto path_length = context.get_parameter<double>("max_path_length");
    if (resource.empty() || frame.empty() || child.empty() || capacity < 2 ||
      capacity > static_cast<std::int64_t>(rpp_control::PlanarPathLayout::max_capacity) ||
      max_points < 1 || max_points > 65536 || !std::isfinite(search_distance) ||
      search_distance <= 0.0 || !std::isfinite(path_length) || path_length <= 0.0) {
      return rpp::CallbackResult::failure(
        "Provide resource/frames, path_capacity in [2,512], max_points in [1,65536], "
        "and finite positive search/path distances in metres");
    }
    rpp_control::PlanarPathLayout layout(static_cast<std::size_t>(capacity));
    auto fields = layout.fields(resource, frame, child);
    rpp_interfaces::FieldLayout state({fields[0], fields[1]});
    const auto additional = context.get_parameter<FieldsDesc>("additional_targets");
    for (const auto & field : additional.fields) {
      if (field.kind != rpp_interfaces::contract::COORDINATE_EUCLIDEAN &&
        field.kind != rpp_interfaces::contract::COORDINATE_CONTINUOUS_ANGLE) {
        return rpp::CallbackResult::failure("additional_targets supports Euclidean and angle fields");
      }
      fields.push_back(field);
    }
    rpp_interfaces::FieldLayout output(std::move(fields));
    rpp::List<double> held_targets;
    held_targets.resize(output.size() - layout.size());
    rpp::List<bool> target_mask;
    target_mask.resize(output.size());
    for (std::size_t scalar = 0; scalar < output.size(); ++scalar) {
      target_mask[scalar] = scalar < 3 || scalar >= layout.size();
    }
    layout_ = layout;
    output_ = std::move(output);
    state_ = std::move(state);
    held_targets_ = std::move(held_targets);
    all_targets_mask_ = std::move(target_mask);
    max_points_ = static_cast<std::size_t>(max_points);
    max_search_distance_ = search_distance;
    max_path_length_ = path_length;
    path_.clear();
    outputs_.clear();
    has_sample_time_ = bound_ = false;
    configured_ = true;
    return {};
  }

  Description::Const getDesc() override
  {
    if (!configured_) {throw std::logic_error("Configure the path reference generator first");}
    Description description;
    auto state = description.state();
    state_.write(state);
    auto reference = description.reference();
    output_.write(reference);
    auto mask = description.targetMask();
    mask.resize(output_.fields().size());
    for (std::size_t field = 0; field < mask.size(); ++field) {mask[field] = field < 2 || field >= 6;}
    description.features() = static_cast<std::uint8_t>(
      rpp_interfaces::contract::GENERATOR_SET_POINT | rpp_interfaces::contract::GENERATOR_SET_PATH);
    return Description::Const(std::move(description));
  }

  std::uint8_t bind(rpp::ListConst<std::uint64_t> offsets, std::uint32_t horizon_length) override
  {
    if (!configured_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    // A spatial polyline has no arrival times. Do not pretend it is a timed horizon.
    if (horizon_length != 0) {return rpp_interfaces::contract::CODE_UNSUPPORTED;}
    if (!offsets.empty()) {return rpp_interfaces::contract::CODE_INVALID_INPUT;}
    try {
      outputs_.prepare(output_.size());
    } catch (const std::bad_alloc &) {
      return rpp_interfaces::contract::CODE_BUDGET_EXCEEDED;
    }
    bound_ = true;
    return rpp_interfaces::contract::CODE_OK;
  }

  rpp::CallbackResult on_activate() override
  {
    if (!configured_ || !bound_) {
      return rpp::CallbackResult::failure("Bind the current-only path reference before activation");
    }
    active_ = true;
    return {};
  }
  rpp::CallbackResult on_deactivate() override {active_ = false; return {};}
  rpp::CallbackResult on_cleanup() override
  {
    active_ = configured_ = bound_ = has_sample_time_ = false;
    path_.clear();
    outputs_.clear();
    state_ = {};
    output_ = {};
    held_targets_.resize(0);
    all_targets_mask_.resize(0);
    return {};
  }

  std::uint8_t setPoint(rpp::ListConst<double> values, rpp::ListConst<bool> mask) override
  {
    if (!configured_ || !bound_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    if (values.size() != output_.size()) {return rpp_interfaces::contract::CODE_INVALID_INPUT;}
    return setPath(values, mask);
  }

  std::uint8_t setPath(rpp::ListConst<double> values, rpp::ListConst<bool> mask) override
  {
    if (!configured_ || !bound_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    const rpp::ListConst<bool> selected = mask.empty() ? all_targets_mask_ : mask;
    if (selected.size() != output_.size() || values.empty() || values.size() % output_.size() != 0 ||
      values.size() / output_.size() > max_points_ ||
      selected[0] != selected[1] || selected[1] != selected[2]) {
      return rpp_interfaces::contract::CODE_INVALID_INPUT;
    }
    // Derived path fields cannot be submitted as targets. Pose updates are atomic;
    // cascade overrides can change independently without restarting path progress.
    for (std::size_t scalar = 3; scalar < layout_.size(); ++scalar) {
      if (selected[scalar]) {return rpp_interfaces::contract::CODE_INVALID_INPUT;}
    }
    for (std::size_t row = 0; row < values.size(); row += output_.size()) {
      if (!output_.valid_update(values, selected, row)) {return rpp_interfaces::contract::CODE_INVALID_INPUT;}
      for (std::size_t scalar = layout_.size(); scalar < output_.size(); ++scalar) {
        if (selected[scalar] && values[row + scalar] != values[scalar]) {
          return rpp_interfaces::contract::CODE_INVALID_INPUT;
        }
      }
    }
    try {
      if (selected[0] && !path_.set_path(values, output_.size(), max_points_)) {
        return rpp_interfaces::contract::CODE_INVALID_INPUT;
      }
    } catch (const std::bad_alloc &) {
      return rpp_interfaces::contract::CODE_BUDGET_EXCEEDED;
    }
    for (std::size_t scalar = layout_.size(); scalar < output_.size(); ++scalar) {
      if (selected[scalar]) {held_targets_[scalar - layout_.size()] = values[scalar];}
    }
    return rpp_interfaces::contract::CODE_OK;
  }

  std::uint8_t setTrajectory(rpp::ListConst<double>, rpp::ListConst<std::uint64_t>, rpp::ListConst<bool>) override
  {return rpp_interfaces::contract::CODE_UNSUPPORTED;}

  std::uint8_t cancel() override
  {
    if (!configured_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    path_.clear();
    for (std::size_t scalar = 0; scalar < held_targets_.size(); ++scalar) {held_targets_[scalar] = 0.0;}
    has_sample_time_ = false;
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
    const auto buffer = outputs_.acquire();
    if (buffer == rpp_control::NumericOutputPool::unavailable) {
      return failure(rpp_interfaces::contract::CODE_NOT_READY);
    }
    auto & values = outputs_.values(buffer);
    rpp_control::PlanarPath::Pose last{state[0], state[1], state[2]};
    std::size_t count = 0;
    if (!path_.empty()) {
      path_.prune(state[0], state[1], max_search_distance_);
      count = path_.visit_pruned(layout_.capacity, max_path_length_,
        [&](std::size_t index, const auto & pose) {
          write_pose(values, index, pose);
          last = pose;
        });
    }
    // Padding stays finite; consumers must use count, never infer the end from padding.
    for (std::size_t index = count; index < layout_.capacity; ++index) {write_pose(values, index, last);}
    values[0] = values[layout_.positions_offset()];
    values[1] = values[layout_.positions_offset() + 1];
    values[2] = values[layout_.orientations_offset()];
    values[layout_.count_offset()] = static_cast<double>(count);
    values[layout_.remaining_distance_offset()] = path_.empty() ? 0.0 : path_.remaining_distance();
    for (std::size_t scalar = 0; scalar < held_targets_.size(); ++scalar) {
      values[layout_.size() + scalar] = held_targets_[scalar];
    }
    last_sample_ns_ = time_ns;
    has_sample_time_ = true;
    return {outputs_.view(buffer), rpp_interfaces::contract::CODE_OK};
  }

private:
  Sample failure(std::uint8_t status) const {return {outputs_.empty(), status};}

  void write_pose(rpp::List<double> & values, std::size_t index,
    const rpp_control::PlanarPath::Pose & pose) const
  {
    values[layout_.positions_offset() + 2 * index] = pose.x;
    values[layout_.positions_offset() + 2 * index + 1] = pose.y;
    values[layout_.orientations_offset() + index] = pose.yaw;
  }

  rpp_control::PlanarPathLayout layout_{256};
  rpp_control::PlanarPath path_;
  rpp_interfaces::FieldLayout state_, output_;
  rpp::List<double> held_targets_;
  rpp::List<bool> all_targets_mask_;
  rpp_control::NumericOutputPool outputs_;
  std::size_t max_points_ = 1024;
  double max_search_distance_ = 20.0, max_path_length_ = 100.0;
  std::uint64_t last_sample_ns_ = 0;
  bool configured_ = false, bound_ = false, active_ = false, has_sample_time_ = false;
};
