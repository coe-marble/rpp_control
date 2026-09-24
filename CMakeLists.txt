cmake_minimum_required(VERSION 3.8)
project(rpp_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rpp_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(marble_control_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(rpp_control SHARED
  src/motion_controller_ros.cpp
)
target_link_libraries(rpp_control
  PUBLIC
  rclcpp::rclcpp
  rpp_cpp::rpp_cpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${marble_control_msgs_TARGETS}
)
set(HOME_DIR "$ENV{HOME}")
target_include_directories(rpp_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${HOME_DIR}/.rpp/interfaces/cpp>
    $<INSTALL_INTERFACE:include>
)

add_executable(motion_controller_node
  src/motion_controller_node.cpp
)

target_link_libraries(motion_controller_node
  rpp_control
)

install(
  DIRECTORY include/
  DESTINATION include
)


install(TARGETS rpp_control motion_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_export_include_directories(include)

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()


  ament_add_gtest(test_motion_controller tests/motion_controller_tests.cpp)

  target_include_directories(test_motion_controller PRIVATE include)
  target_link_libraries(test_motion_controller
    rpp_control
  )
  # copy test data to build directory
  add_custom_target(copy_test_directory ALL
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/data"
            "${CMAKE_CURRENT_BINARY_DIR}/tests/data"
    )

  add_dependencies(test_motion_controller copy_test_directory)


  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
