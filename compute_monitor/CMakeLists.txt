cmake_minimum_required(VERSION 3.8)
project(compute_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(compute_monitor_component SHARED
  src/compute_monitor_node.cpp
)

target_include_directories(compute_monitor_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(compute_monitor_component PUBLIC cxx_std_17)

ament_target_dependencies(compute_monitor_component
  peregrine_interfaces
  rclcpp
  rclcpp_components
)

rclcpp_components_register_node(compute_monitor_component
  PLUGIN "compute_monitor::ComputeMonitorNode"
  EXECUTABLE compute_monitor_node
)

install(TARGETS compute_monitor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(peregrine_interfaces rclcpp rclcpp_components)

ament_package()
