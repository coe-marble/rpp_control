#pragma once

#include <iomanip>
#include <map>
#include <optional>
#include <random>
#include <sstream>
#include <string>

namespace rpp_control
{
// Executor-owned authority. Expiry and enable/disable never transfer ownership.
class ReferenceClaims
{
public:
  struct Claim
  {
    std::string instance_id, identity, token;
  };

  std::optional<std::string> claim(
    const std::string & interface,
    const std::string & instance_id, const std::string & identity)
  {
    if (claims_.count(interface)) {return std::nullopt;}
    std::random_device random;
    std::ostringstream token;
    token << std::hex << std::setfill('0');
    for (unsigned index = 0; index < 4; ++index) {token << std::setw(8) << random();}
    auto value = token.str();
    claims_.emplace(interface, Claim{instance_id, identity, value});
    return value;
  }

  bool authorizes(const std::string & interface, const std::string & token) const
  {
    const auto found = claims_.find(interface);
    if (found == claims_.end()) {return false;}
    return !token.empty() && token == found->second.token;
  }

  bool claimed(const std::string & interface) const {return claims_.count(interface) != 0;}

  std::optional<std::string> interface_for(const std::string & token) const
  {
    if (token.empty()) {return std::nullopt;}
    for (const auto & [interface, claim] : claims_) {
      if (claim.token == token) {return interface;}
    }
    return std::nullopt;
  }

  void release(const std::string & interface) {claims_.erase(interface);}

  template<class InstanceLookup>
  void retain(const InstanceLookup & instance_for)
  {
    for (auto claim = claims_.begin(); claim != claims_.end(); ) {
      if (instance_for(claim->first) != claim->second.instance_id) {
        claim = claims_.erase(claim);
      } else {++claim;}
    }
  }

private:
  std::map<std::string, Claim> claims_;
};
}  // namespace rpp_control
