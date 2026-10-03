#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

#include "gtest/gtest.h"
#include "rpp_cpp/context_builder.hpp"
#include "rpp_plugin_types/rpp_control/MotionControllerAllocator2D.hpp"

namespace
{

constexpr std::size_t k_grid_size = 10;
constexpr double k_jet_x = -1.5;
constexpr double k_jet_y = 0.0;
constexpr double k_max_thrust = 6000.0;
constexpr double k_max_angle = 0.5235987756;
constexpr double k_mass = 450.0;
constexpr double k_center_of_gravity_height = 0.4;
constexpr double k_metacentric_height = 0.3;
constexpr double k_max_safe_roll_deg = 30.0;
constexpr double k_pi = 3.14159265358979323846;
constexpr double k_gravity = 9.81;
constexpr double k_tolerance = 1e-9;

double maximum_lateral_force()
{
  const double restoring_moment = k_mass * k_gravity * k_metacentric_height
    * std::sin(k_max_safe_roll_deg * k_pi / 180.0);
  return std::min(
    restoring_moment / k_center_of_gravity_height,
    k_max_thrust * std::sin(k_max_angle));
}

double maximum_safe_yaw_wrench()
{
  return std::abs(k_jet_x) * maximum_lateral_force();
}

double normalized_grid_value(const std::size_t index)
{
  return -1.0 + 2.0 * static_cast<double>(index)
    / static_cast<double>(k_grid_size - 1);
}

TEST(WaterjetAllocatorSafety, sweeps_normalized_surge_and_yaw_grid)
{
  const std::string data_directory = "./tests/data/waterjet_allocator_safety";
  auto context = rpp::ComponentContextBuilder(rpp::RPP_CLOCK_MOCK)
    .build_script_from_description_path(
      data_directory + "/waterjet_allocator.json",
      std::nullopt,
      data_directory + "/parts");
  context.initialize();

  using Allocator = rpp_control::MotionControllerAllocator2D;
  const auto allocator = context.get_component<Allocator>("allocator");
  ASSERT_NE(allocator, nullptr);
  EXPECT_EQ(allocator->outputSize(), 2U);

  std::ofstream csv(std::filesystem::current_path()
    / "waterjet_allocator_safety_grid.csv");
  ASSERT_TRUE(csv.is_open());
  csv << std::fixed << std::setprecision(12);
  csv << "surge_normalized,yaw_normalized,reference_surge_n,reference_yaw_nm,"
      << "thrust_command_normalized,nozzle_command_normalized,"
      << "realized_surge_n,realized_sway_n,realized_yaw_nm,"
      << "surge_error_n,yaw_error_nm,saturated\n";

  for (std::size_t surge_index = 0; surge_index < k_grid_size; ++surge_index)
  {
    for (std::size_t yaw_index = 0; yaw_index < k_grid_size; ++yaw_index)
    {
      const double surge_normalized = normalized_grid_value(surge_index);
      const double yaw_normalized = normalized_grid_value(yaw_index);
      const double reference_surge = surge_normalized * k_max_thrust;
      const double reference_yaw = yaw_normalized * maximum_safe_yaw_wrench();

      Allocator::Wrench2D reference;
      reference.force().x() = reference_surge;
      reference.force().y() = 0.0;
      reference.torque() = reference_yaw;

      Allocator::Odometry2D state;
      state.twist().linear().x() = 0.0;
      state.twist().linear().y() = 0.0;
      state.twist().angular() = 0.0;

      Allocator::Enabler2D enabler;
      enabler.enableX() = true;
      enabler.enableY() = false;
      enabler.enableN() = true;

      const auto allocation = allocator->allocate(
        std::move(reference), std::move(state), std::move(enabler), 0.1);
      const auto & commands = std::get<0>(allocation).data();
      const auto & realized = std::get<1>(allocation);
      ASSERT_EQ(commands.size(), 2U);

      const double thrust_command = commands[0];
      const double nozzle_command = commands[1];
      const double realized_surge = realized.force().x();
      const double realized_sway = realized.force().y();
      const double realized_yaw = realized.torque();
      const double realized_thrust = std::hypot(realized_surge, realized_sway);
      const bool saturated = std::abs(thrust_command) >= 1.0 - k_tolerance
        || std::abs(nozzle_command) >= 1.0 - k_tolerance;

      SCOPED_TRACE(
        "surge_normalized=" + std::to_string(surge_normalized)
        + ", yaw_normalized=" + std::to_string(yaw_normalized));
      EXPECT_TRUE(std::isfinite(thrust_command));
      EXPECT_TRUE(std::isfinite(nozzle_command));
      EXPECT_TRUE(std::isfinite(realized_surge));
      EXPECT_TRUE(std::isfinite(realized_sway));
      EXPECT_TRUE(std::isfinite(realized_yaw));
      EXPECT_LE(std::abs(thrust_command), 1.0 + k_tolerance);
      EXPECT_LE(std::abs(nozzle_command), 1.0 + k_tolerance);
      EXPECT_LE(realized_thrust, k_max_thrust + k_tolerance);
      EXPECT_LE(std::abs(realized_sway), maximum_lateral_force() + k_tolerance);
      EXPECT_NEAR(
        realized_yaw,
        k_jet_x * realized_sway - k_jet_y * realized_surge,
        k_tolerance);

      csv << surge_normalized << ',' << yaw_normalized << ','
          << reference_surge << ',' << reference_yaw << ','
          << thrust_command << ',' << nozzle_command << ','
          << realized_surge << ',' << realized_sway << ',' << realized_yaw << ','
          << realized_surge - reference_surge << ','
          << realized_yaw - reference_yaw << ',' << saturated << '\n';
    }
  }
}

}  // namespace
