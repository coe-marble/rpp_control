#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_interfaces/ControlLaw.hpp>
#include <rpp_control/auxiliary_state.hpp>
#include <rpp_control/control_plugin_contract.hpp>
#include <rpp_control/numeric_output_pool.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <vector>

// An explicit final cascade stage: select physical wrench references, enforce
// available axes, and limit effort changes before the allocator receives them.
class WrenchController2D final : public rpp_interfaces::ControlLaw
{
public:
  explicit WrenchController2D(bool validate_compute = true)
  : validate_compute_(validate_compute) {}

  using ParameterDescription = rpp::params::ParameterDescription;
  using Result = std::tuple<rpp::ListConst<double>, std::uint8_t>;
  RPP_PARAMETERS(
    ParameterDescription::create<std::string>("frame_id", "base_link"),
    ParameterDescription::create<std::string>("resource", "base"),
    ParameterDescription::create<bool>("enable_x", true),
    ParameterDescription::create<bool>("enable_y", true),
    ParameterDescription::create<bool>("enable_yaw", true),
    ParameterDescription::create<std::vector<double>>("max_wrench_rate",
    std::vector<double>{0.0, 0.0, 0.0})
  )

  rpp::CallbackResult on_configure(const rpp::ComponentContext & context) override
  {
    if (active_) {
      return rpp::CallbackResult::failure("Deactivate the wrench controller before configuring");
    }
    const auto frame = context.get_parameter<std::string>("frame_id");
    const auto resource = context.get_parameter<std::string>("resource");
    const auto rates = context.get_parameter<std::vector<double>>("max_wrench_rate");
    if (frame.empty() || resource.empty() || rates.size() != 3 ||
      !std::all_of(rates.begin(), rates.end(), [](double rate) {
        return std::isfinite(rate) && rate >= 0.0;
                                                                                                          }))
    {
      return rpp::CallbackResult::failure(
        "Wrench frame/resource must be nonempty; rates must contain three finite nonnegative values");
    }
    wrench_ = rpp_interfaces::FieldLayout({
      {resource + ".force", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_FORCE, 2, frame, frame, resource},
      {resource + ".moment", rpp_interfaces::contract::COORDINATE_EUCLIDEAN,
        rpp_interfaces::contract::QUANTITY_MOMENT, 1, frame, frame, resource}});
    enabled_ = {context.get_parameter<bool>("enable_x"), context.get_parameter<bool>("enable_y"),
      context.get_parameter<bool>("enable_yaw")};
    std::copy(rates.begin(), rates.end(), rates_.begin());
    outputs_.prepare(3);
    configured_ = true;
    return {};
  }

  ControlLawDesc::Const getDesc() override
  {
    if (!configured_) {throw std::logic_error("Configure the wrench controller first");}
    return rpp_control::control_description({}, wrench_, wrench_);
  }

  Result compute(
    rpp::ListConst<double> state, rpp::ListConst<double> reference,
    rpp::ListConst<bool> enabled_axes, rpp::ListConst<bool> input_valid,
    rpp::ListConst<double> previous_control, bool previous_control_valid,
    std::uint64_t, std::uint64_t dt_ns) override
  {
    if (validate_compute_) {
      if (!active_) {return failure(rpp_interfaces::contract::CODE_NOT_READY);}
      if (!state.empty() || reference.size() != 3 || dt_ns == 0 ||
        (!enabled_axes.empty() && enabled_axes.size() != 3) ||
        (!input_valid.empty() && input_valid.size() != 3) ||
        !wrench_.valid_row(reference) ||
        (previous_control_valid &&
        (previous_control.size() != 3 || !wrench_.valid_row(previous_control))))
      {
        return failure(rpp_interfaces::contract::CODE_INVALID_INPUT);
      }
    }
    const auto buffer = outputs_.acquire();
    if (buffer == rpp_control::NumericOutputPool::unavailable) {
      return failure(rpp_interfaces::contract::CODE_NOT_READY);
    }
    auto & command = outputs_.values(buffer);
    bool limited = false;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      // Expired and disabled axes become zero immediately, without a rate-limited tail.
      if (!enabled_[axis] || (!enabled_axes.empty() && !enabled_axes[axis]) ||
        (!input_valid.empty() && !input_valid[axis])) {command[axis] = 0.0; continue;}
      double value = reference[axis];
      if (rates_[axis] > 0.0) {
        const double previous =
          previous_control_valid ? static_cast<double>(previous_control[axis]) : 0.0;
        const double delta = rates_[axis] * (static_cast<double>(dt_ns) * 1e-9);
        value = std::clamp(value, previous - delta, previous + delta);
        limited = limited || value != reference[axis];
      }
      command[axis] = value;
    }
    return {outputs_.view(buffer),
      limited ? rpp_interfaces::contract::CODE_LIMITED : rpp_interfaces::contract::CODE_OK};
  }

  rpp_control::AuxiliaryState::Snapshot::Const getAuxiliaryState() override
  {return rpp_control::AuxiliaryState{}.snapshot();}

  rpp::CallbackResult on_activate(const std::optional<HandoverState::Const> &) override
  {
    if (!configured_) {
      return rpp::CallbackResult::failure("Configure the wrench controller first");
    }
    active_ = true;
    return {};
  }
  rpp::CallbackResult on_deactivate() override {active_ = false; return {};}
  rpp::CallbackResult on_cleanup() override
  {
    active_ = configured_ = false;
    outputs_.clear();
    wrench_ = rpp_interfaces::FieldLayout{};
    return {};
  }
  void reset() override {}

private:
  Result failure(std::uint8_t code) const {return {outputs_.empty(), code};}
  rpp_interfaces::FieldLayout wrench_;
  rpp_control::NumericOutputPool outputs_;
  std::array<bool, 3> enabled_{};
  std::array<double, 3> rates_{};
  const bool validate_compute_;
  bool configured_ = false, active_ = false;
};
