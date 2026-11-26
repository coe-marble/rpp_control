#pragma once

#include <rpp_cpp/list_definitions.hpp>
#include <rpp_interfaces/field_routing.hpp>
#include <rpp_schema/rpp_interfaces/StateWithDescription.hpp>
#include <rpp_schema/rpp_interfaces/StateWithDescriptionStamped.hpp>

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rpp_control
{
// Assemble described plugin-owned auxiliary snapshots outside the control tick.
// Missing inputs contribute neither values nor fields to the auxiliary description.
class AuxiliaryState
{
public:
  using Values = std::optional<rpp::ListConst<double>>;
  using State = rpp_schema::rpp_interfaces::StateWithDescription;
  using Snapshot = rpp_schema::rpp_interfaces::StateWithDescriptionStamped;

  explicit AuxiliaryState(std::uint64_t stamp_ns = 0)
  : local_stamp_ns_(stamp_ns)
  {}

  AuxiliaryState & append(
    const std::string & name, const rpp_interfaces::FieldLayout & axes, Values values = {})
  {
    if (!values) {
      return *this;
    }
    if (values->size() != axes.size()) {
      throw std::invalid_argument("Auxiliary values do not match their field description: " + name);
    }
    append_row(name, axes, *values, 0);
    return *this;
  }

  void write_to(State & state) const
  {
    auto description = state.desc();
    layout_.write(description);
    auto values = state.state();
    values.resize(values_.size());
    for (std::size_t index = 0; index < values_.size(); ++index) {
      values[index] = values_[index];
    }
  }

  void write_to(Snapshot & snapshot) const
  {
    auto state = snapshot.state();
    write_to(state);
    snapshot.stampNs() = oldest_stamp_ns_.value_or(local_stamp_ns_);
  }

  Snapshot::Const snapshot() const
  {
    Snapshot result;
    write_to(result);
    return Snapshot::Const(std::move(result));
  }

  void merge(const Snapshot::Const & snapshot)
  {
    const auto state = snapshot.state();
    const auto values = state.state();
    const rpp_interfaces::FieldLayout source(state.desc().as_struct().fields);
    if (values.size() != source.size() || !source.valid_row(values)) {
      throw std::invalid_argument("Auxiliary values do not match their field description");
    }
    append_values(source, values, 0, snapshot.stampNs());
  }

  static std::optional<std::vector<double>> read(
    const std::string & name, const rpp_interfaces::FieldLayout & axes, const State::Const & state)
  {
    const auto values = state.state();
    const rpp_interfaces::FieldLayout available(state.desc().as_struct().fields);
    if (values.size() != available.size() || !available.valid_row(values)) {
      throw std::invalid_argument("Auxiliary values do not match their field description");
    }
    const auto required = scoped_layout(name, axes);
    bool present = false;
    for (const auto & field : required.fields()) {
      present = present || rpp_interfaces::find_field(available, field.name).has_value();
    }
    if (!present) {
      return std::nullopt;
    }
    const auto route = rpp_interfaces::field_route(available, required);
    std::vector<double> result;
    result.reserve(route.size());
    for (const auto index : route) {
      result.push_back(values[index]);
    }
    return result;
  }

private:
  rpp_interfaces::FieldLayout layout_;
  std::vector<double> values_;
  std::uint64_t local_stamp_ns_ = 0;
  std::optional<std::uint64_t> oldest_stamp_ns_;

  static rpp_interfaces::FieldLayout scoped_layout(
    const std::string & name, const rpp_interfaces::FieldLayout & axes)
  {
    if (name.empty()) {
      throw std::invalid_argument("Auxiliary state name must be nonempty");
    }
    auto fields = axes.fields();
    for (auto & field : fields) {
      field.name = name + "." + field.name;
    }
    return rpp_interfaces::FieldLayout(std::move(fields));
  }

protected:
  void append_row(
    const std::string & name, const rpp_interfaces::FieldLayout & axes,
    rpp::ListConst<double> values, std::size_t offset)
  {
    if (!axes.valid_row(values, offset)) {
      throw std::invalid_argument("Invalid auxiliary values: " + name);
    }
    const auto scoped = scoped_layout(name, axes);
    append_values(scoped, values, offset, local_stamp_ns_);
  }

private:
  void append_values(
    const rpp_interfaces::FieldLayout & source, rpp::ListConst<double> values,
    std::size_t offset, std::uint64_t stamp_ns)
  {
    for (const auto & field : source.fields()) {
      if (rpp_interfaces::find_field(layout_, field.name)) {
        throw std::invalid_argument("Repeated auxiliary field: " + field.name);
      }
    }
    auto combined = rpp_interfaces::merge_fields(layout_, source);
    values_.reserve(values_.size() + source.size());
    for (std::size_t index = 0; index < source.size(); ++index) {
      values_.push_back(values[offset + index]);
    }
    layout_ = std::move(combined);
    // A merged snapshot conservatively reports the oldest contributing data.
    if (source.size() != 0 && (!oldest_stamp_ns_ || stamp_ns < *oldest_stamp_ns_)) {
      oldest_stamp_ns_ = stamp_ns;
    }
  }
};
} // namespace rpp_control
