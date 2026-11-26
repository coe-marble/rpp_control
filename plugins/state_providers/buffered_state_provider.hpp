#pragma once

#include <rpp_cpp/plugin.hpp>
#include <rpp_plugin_types/rpp_interfaces/StateProvider.hpp>
#include <rpp_interfaces/field_layout.hpp>
#include <rpp_control/numeric_output_pool.hpp>

#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>

class BufferedStateProvider final : public rpp_interfaces::StateProvider
{
public:
  using ParameterDescription = rpp::params::ParameterDescription;
  using FieldsDesc = rpp_schema::rpp_interfaces::FieldListDesc_Native;
  RPP_PARAMETERS(
    ParameterDescription::create<FieldsDesc>("state")
  )

  rpp::CallbackResult on_configure(const rpp::ComponentContext & context) override
  {
    rpp_interfaces::FieldLayout prepared(context.get_parameter<FieldsDesc>("state").fields);
    if (prepared.size() == 0) {return rpp::CallbackResult::failure("State layout is empty");}
    samples_.prepare(prepared.size());
    layout_ = std::move(prepared);
    current_sample_.reset();
    time_ns_ = 0;
    configured_ = true;
    return {};
  }

  rpp_interfaces::FieldList::Const getDesc() override
  {
    if (!configured_) {throw std::logic_error("Configure the state provider first");}
    return layout_.description();
  }

  std::tuple<rpp::ListConst<double>, std::uint64_t, std::uint8_t> getState() override
  {
    if (!configured_ || !current_sample_) {
      return {samples_.empty(), 0, rpp_interfaces::contract::CODE_NOT_READY};
    }
    return {samples_.view(*current_sample_), time_ns_, rpp_interfaces::contract::CODE_OK};
  }

  std::uint8_t setState(rpp::ListConst<double> state, std::uint64_t timeNs) override
  {
    if (!configured_) {return rpp_interfaces::contract::CODE_NOT_READY;}
    if (state.size() != layout_.size() || !layout_.valid_row(state)) {
      return rpp_interfaces::contract::CODE_INVALID_INPUT;
    }
    const auto available = samples_.acquire();
    if (available == rpp_control::NumericOutputPool::unavailable) {
      return rpp_interfaces::contract::CODE_BUDGET_EXCEEDED;
    }
    auto & destination = samples_.values(available);
    for (std::size_t index = 0; index < state.size(); ++index) {
      destination[index] = state[index];
    }
    current_sample_ = available;
    time_ns_ = timeNs;
    return rpp_interfaces::contract::CODE_OK;
  }

  rpp::CallbackResult on_deactivate() override
  {
    current_sample_.reset();
    return {};
  }

  rpp::CallbackResult on_cleanup() override
  {
    configured_ = false;
    current_sample_.reset();
    samples_.clear();
    time_ns_ = 0;
    layout_ = rpp_interfaces::FieldLayout{};
    return {};
  }

private:
  rpp_interfaces::FieldLayout layout_;
  rpp_control::NumericOutputPool samples_;
  std::optional<std::size_t> current_sample_;
  std::uint64_t time_ns_ = 0;
  bool configured_ = false;
};
