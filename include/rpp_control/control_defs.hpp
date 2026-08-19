#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <string>
#include <string_view>
#include <vector>

#include <rpp_schema/rpp_common/Command.hpp>
#include <rpp_schema/rpp_common/Odometry2D.hpp>
#include <rpp_schema/rpp_common/Twist2D.hpp>
#include <rpp_schema/rpp_common/VectorPlanar.hpp>

#define FP_TYPE float

namespace rpp_control {



/**
 * Indices of the DOFS
 */
enum DOF_i {
  DOF_X_i = 0,
  DOF_Y_i,
  DOF_Z_i,
  DOF_K_i,
  DOF_M_i,
  DOF_N_i,
  DOF_END_i
};

/**
 * DOF flags
 */
enum DOF { NO_DOF = 0, DOF_X = 1, DOF_Y = 2, DOF_Z = 4, DOF_K = 8, DOF_M = 16, DOF_N = 32 };

enum ReferenceType {IGNORE=0, POSE_REF, TWIST_REF, WRENCH_REF, REF_END };

enum SignalStatus { SIGNAL_DISABLED=0, SIGNAL_EXT=1, SIGNAL_INT=2, SIGNAL_DONTCARE=3 };

const std::array<std::string, DOF_END_i> DOF_NAMES{{"x", "y", "z", "roll", "pitch", "yaw"}};

/**
 * State data that is read and written frequently during control.
 */
template <size_t Dim>
struct ControllerIOStateT
{
  ControllerIOStateT() :
      has_any_pose_ext(false),
      has_any_twist_ext(false),
      has_any_wrench_ext(false),
      has_feedback(false),
      has_pose_ext{},
      has_twist_ext{},
      has_wrench_ext{},
      pose_ref{},
      twist_ref{},
      wrench_ref{},
      pose{},
      twist{},
      wrench{},
      commands{},
      twist_selection{},
      wrench_selection{}
  {}

  bool has_any_pose_ext;
  bool has_any_twist_ext;
  bool has_any_wrench_ext;
  bool has_feedback;
  std::array<bool, Dim> has_pose_ext;
  std::array<bool, Dim> has_twist_ext;
  std::array<bool, Dim> has_wrench_ext;

  std::array<FP_TYPE, Dim> pose_ref;
  std::array<FP_TYPE, Dim> twist_ref;
  std::array<FP_TYPE, Dim> wrench_ref;

  std::array<FP_TYPE, Dim> pose;
  std::array<FP_TYPE, Dim> twist;
  std::array<FP_TYPE, Dim> wrench;

  std::vector<FP_TYPE> commands;
  std::array<SignalStatus, Dim> twist_selection;
  std::array<SignalStatus, Dim> wrench_selection;
};

/**
 * Derived data that should not be deep-copied every step.
 */
template <size_t Dim>
struct ControllerIOErrorT
{
  ControllerIOErrorT() :
      pose_error{},
      twist_error{},
      wrench_error{},
      pose_error_body{}
  {}

  std::array<FP_TYPE, Dim> pose_error;
  std::array<FP_TYPE, Dim> twist_error;
  std::array<FP_TYPE, Dim> wrench_error;
  std::array<FP_TYPE, Dim> pose_error_body;

};

template <size_t Dim>
struct ControllerIOT
{
  using State = ControllerIOStateT<Dim>;
  using Error = ControllerIOErrorT<Dim>;

  State state;
  Error errors;
};



struct FixedSizeString
{
    constexpr static size_t MAX_LEN = 32;
    std::array<char, MAX_LEN> data_;

    FixedSizeString() : data_{} {}

    FixedSizeString(std::string_view str) : data_{}
    {
        size_t len = std::min(str.size(), data_.size() - 1);
        std::copy_n(str.data(), len, data_.begin());
        data_[len] = '\0';
    }

    void fill(std::string_view str)
    {
        size_t len = std::min(str.size(), data_.size() - 1);
        std::copy_n(str.data(), len, data_.begin());
        data_[len] = '\0';
    }

    void fill(char c)
    {
        std::fill(data_.begin(), data_.end(), c);
        data_[data_.size() - 1] = '\0';
    }

    size_t size() const
    {
        return std::string_view(data_.data()).size();
    }

    std::string_view view() const
    {
        return std::string_view(data_.data());
    }

    std::array<char, MAX_LEN>::iterator begin() { return data_.begin(); }
    std::array<char, MAX_LEN>::iterator end() { return data_.end(); }

    bool is_empty() const
    {
        return data_[0] == '\0';
    }

    std::string to_string() const
    {
        return std::string(data_.data());
    }

    bool operator==(const FixedSizeString& other) const {
        return std::string_view(data_.data()) == std::string_view(other.data_.data());
    }

    bool operator!=(const FixedSizeString& other) const {
        return !(*this == other);
    }
};

struct Identity
{
    FixedSizeString token;
    FixedSizeString name;

    Identity() : token{}, name{}
    {
        token.fill('\0');
        name.fill('\0');
    }

    Identity(const FixedSizeString& token, const FixedSizeString& name)
        : token{token}, name{name} {}

    static void get_random_identity(
        const FixedSizeString& name, std::string_view prefix, Identity& out_identity)
    {
        out_identity.name = name;
        out_identity.token.fill('\0');
        int random_num = std::rand() % 40000 + 10000;

        std::snprintf(out_identity.token.begin(),
                      out_identity.token.size(),
                      "%.*s%d",
                      static_cast<int>(prefix.size()),
                      prefix.data(),
                      random_num);

    }

    bool is_set() const
    {
        return !token.view().empty();
    }
};


[[maybe_unused]] inline constexpr DOF operator|(DOF a, DOF b) {
  return static_cast<DOF>(static_cast<int>(a) | static_cast<int>(b));
}

[[maybe_unused]] inline constexpr DOF operator&(DOF a, DOF b) {
  return static_cast<DOF>(static_cast<int>(a) & static_cast<int>(b));
}

[[maybe_unused]] inline constexpr DOF operator|=(DOF& a, DOF b) {
  return a = a | b;
}

[[maybe_unused]] inline constexpr DOF operator&=(DOF& a, DOF b) {
  return a = a & b;
}

[[maybe_unused]] inline constexpr DOF operator~(DOF a) {
  return static_cast<DOF>(~static_cast<int>(a));
}



template <size_t Dim>
inline constexpr DOF get_dof_by_index(int i)
{
    if constexpr (Dim == 6)
    {
        return static_cast<DOF>(1 << i);
    }
    else if constexpr (Dim == 3)
    {
        switch (i)
        {
            case 0: return DOF_X;
            case 1: return DOF_Y;
            case 2: return DOF_N;
            default: return NO_DOF;
        }
    }
    else
    {
        static_assert(Dim == 3 || Dim == 6, "Dim must be either 3 or 6");
    }
    return NO_DOF;
}

[[maybe_unused]] inline constexpr bool is_dof_angular(DOF dof) { return dof & (DOF_K | DOF_M | DOF_N); }
[[maybe_unused]] inline constexpr bool is_dof_linear(DOF dof) { return dof & (DOF_X | DOF_Y | DOF_Z); }

template <size_t Dim>
inline constexpr bool is_dof_angular(int i) { return get_dof_by_index<Dim>(i) & (DOF_K | DOF_M | DOF_N); }

template <size_t Dim>
inline constexpr bool is_dof_linear(int i) { return get_dof_by_index<Dim>(i) & (DOF_X | DOF_Y | DOF_Z); }

[[maybe_unused]] static DOF get_dof_by_name(const std::string& dof)
{

  std::string lowercase(dof);
  std::transform(lowercase.begin(), lowercase.end(), lowercase.begin(),
      [](unsigned char c){ return std::tolower(c); }
  );

  auto itr = std::find(DOF_NAMES.begin(), DOF_NAMES.end(), lowercase);
  if (itr == DOF_NAMES.end())
  {
    return NO_DOF;
  }

  int idx = std::distance(DOF_NAMES.begin(), itr);

  return static_cast<DOF>(get_dof_by_index<6>(idx));
}

[[maybe_unused]] static DOF get_dofs_by_name(const std::vector<std::string>& names)
{
  auto ret_dofs = 0;
  for (const auto& name : names)
  {
    ret_dofs |= get_dof_by_name(name);
  }
  return static_cast<DOF>(ret_dofs);
}

[[maybe_unused]] static std::vector<std::string> get_dof_names(DOF dof)
{
  std::vector<std::string> ret_dofs;
  for (int i = 0; i < DOF_END_i; ++i)
  {
    if (get_dof_by_index<6>(i) & dof)
    {
      ret_dofs.push_back(DOF_NAMES[i]);
    }
  }
  return ret_dofs;
}


}  // namespace rpp_control