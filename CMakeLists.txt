cmake_minimum_required(VERSION 3.15)
project(rpp_control)

find_package(ament_cmake REQUIRED)
find_package(rpp_cpp REQUIRED)
find_package(rpp_interfaces REQUIRED)

# Native plugin implementations are registered and compiled by RPP.
add_library(rpp_control INTERFACE)
target_compile_features(rpp_control INTERFACE cxx_std_17)
target_link_libraries(rpp_control INTERFACE rpp_cpp::rpp_cpp rpp_interfaces::rpp_interfaces)

target_include_directories(rpp_control INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY plugins DESTINATION share/${PROJECT_NAME})
install(FILES plugins.json DESTINATION share/${PROJECT_NAME})
install(DIRECTORY .rppws DESTINATION share/${PROJECT_NAME})
install(TARGETS rpp_control EXPORT rpp_controlTargets)
ament_export_targets(rpp_controlTargets)
ament_export_include_directories(include)
ament_export_dependencies(rpp_cpp rpp_interfaces)

option(RPP_CONTROL_BUILD_ROS2 "Build the ROS 2 control nodes" ON)
if(RPP_CONTROL_BUILD_ROS2)
  find_package(Threads REQUIRED)
  find_package(rpp_ros2 REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(nav2_msgs REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rpp_ros_interfaces REQUIRED)

  add_executable(control_script_node src/ros/control_script_node.cpp
    src/ros/control_script_monitors.cpp
    src/ros/control_script_references.cpp src/ros/navigation_interface.cpp
    src/ros/navigation_follow_path.cpp)
  target_compile_features(control_script_node PRIVATE cxx_std_20)
  target_link_libraries(control_script_node PRIVATE Threads::Threads rpp_control rpp_ros2::rpp_ros2
    rclcpp::rclcpp rclcpp_action::rclcpp_action ${nav2_msgs_TARGETS}
    ${rcl_interfaces_TARGETS} ${std_srvs_TARGETS} ${geometry_msgs_TARGETS} ${rpp_ros_interfaces_TARGETS})
  install(TARGETS control_script_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_controller_lifecycle tests/controller_lifecycle_tests.cpp)
  target_link_libraries(test_controller_lifecycle rpp_control)
  ament_add_gtest(test_waterjet_allocator_safety tests/waterjet_allocator_safety_tests.cpp)
  target_link_libraries(test_waterjet_allocator_safety rpp_control)
  ament_add_gtest(test_reference_claims tests/reference_claims_tests.cpp)
  target_link_libraries(test_reference_claims rpp_control)
  ament_add_gtest(test_planar_path tests/planar_path_tests.cpp)
  target_link_libraries(test_planar_path rpp_control)
  ament_add_gtest(test_planar_path_reference tests/planar_path_reference_tests.cpp)
  target_link_libraries(test_planar_path_reference rpp_control)
  ament_add_gtest(test_pure_pursuit
    tests/regulated_pure_pursuit_tests.cpp tests/pure_pursuit_controller_tests.cpp)
  target_link_libraries(test_pure_pursuit rpp_control)

  if(RPP_CONTROL_BUILD_ROS2)
    ament_add_gtest(test_control_script_time tests/control_script_time_tests.cpp)
    target_link_libraries(test_control_script_time rpp_control rclcpp::rclcpp
      ${builtin_interfaces_TARGETS} ${rpp_ros_interfaces_TARGETS})
  endif()

  add_custom_target(copy_test_directory ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
      "${CMAKE_CURRENT_BINARY_DIR}/tests/data"
  )
  add_dependencies(test_controller_lifecycle copy_test_directory)
  add_dependencies(test_waterjet_allocator_safety copy_test_directory)
  add_dependencies(test_planar_path_reference copy_test_directory)
  add_dependencies(test_pure_pursuit copy_test_directory)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
