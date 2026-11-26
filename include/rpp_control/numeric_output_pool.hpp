#pragma once

#include <rpp_cpp/list_definitions.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <stdexcept>

namespace rpp_control
{
// Result views pin their buffer. A retained view is never overwritten, even on cleanup.
class NumericOutputPool
{
  struct Buffer
  {
    rpp::List<double> values;
    explicit Buffer(std::size_t size) {values.resize(size);}
  };

public:
  static constexpr std::size_t capacity = 3;
  static constexpr std::size_t unavailable = capacity;
  NumericOutputPool()
  : empty_(std::make_shared<Buffer>(0)) {}

  void prepare(std::size_t size)
  {
    if (size > 1024 * 1024) {throw std::invalid_argument("Reference output is too large");}
    std::array<std::shared_ptr<Buffer>, capacity> prepared;
    for (auto & buffer : prepared) {buffer = std::make_shared<Buffer>(size);}
    buffers_.swap(prepared);
  }

  std::size_t acquire() const noexcept
  {
    for (std::size_t i = 0; i < capacity; ++i) {
      if (buffers_[i] && buffers_[i].use_count() == 1) {return i;}
    }
    return unavailable;
  }
  rpp::List<double> & values(std::size_t index) {return buffers_[index]->values;}
  rpp::ListConst<double> view(std::size_t index) const
  {
    return {buffers_[index]->values, buffers_[index]};
  }
  rpp::ListConst<double> empty() const {return {empty_->values, empty_};}
  void clear() noexcept {for (auto & buffer : buffers_) {buffer.reset();}}

private:
  std::array<std::shared_ptr<Buffer>, capacity> buffers_;
  std::shared_ptr<Buffer> empty_;
};
}  // namespace rpp_control
