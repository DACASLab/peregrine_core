cmake_minimum_required(VERSION 3.8)
project(peregrine_autopilot_abstraction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(peregrine_interfaces REQUIRED)

# Header-only library for msg_version.hpp
add_library(msg_version INTERFACE)
target_include_directories(msg_version INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


add_library(px4_abstraction_component SHARED
  src/px4_hardware_abstraction.cpp
)

target_include_directories(px4_abstraction_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(px4_abstraction_component
  msg_version
)

ament_target_dependencies(px4_abstraction_component
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  px4_msgs
  peregrine_interfaces
)

rclcpp_components_register_node(px4_abstraction_component
  PLUGIN "px4_hardware_abstraction::PX4APIManager"
  EXECUTABLE px4_abstraction_node
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install header-only library
install(TARGETS msg_version
  EXPORT msg_version_targets
)

# Export include directories
ament_export_include_directories(include)

# Export targets
ament_export_targets(msg_version_targets HAS_LIBRARY_TARGET)
ament_export_targets(export_px4_abstraction_component HAS_LIBRARY_TARGET)
install(TARGETS px4_abstraction_component
        EXPORT export_px4_abstraction_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
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
