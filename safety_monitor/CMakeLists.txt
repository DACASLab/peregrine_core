cmake_minimum_required(VERSION 3.8)
project(safety_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  safety_monitor_parameters
  src/safety_monitor_parameters.yaml
)

add_library(safety_monitor_component SHARED
  src/battery_checker.cpp
  src/gps_checker.cpp
  src/geofence_checker.cpp
  src/envelope_checker.cpp
  src/rule_engine.cpp
  src/safety_action_executor.cpp
  src/safety_monitor_node.cpp
)

target_include_directories(safety_monitor_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(safety_monitor_component safety_monitor_parameters)

target_compile_features(safety_monitor_component PUBLIC cxx_std_17)

ament_target_dependencies(safety_monitor_component
  geometry_msgs
  peregrine_interfaces
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
)

rclcpp_components_register_node(safety_monitor_component
  PLUGIN "safety_monitor::SafetyMonitorNode"
  EXECUTABLE safety_monitor_node
)

install(TARGETS safety_monitor_component
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
ament_export_dependencies(geometry_msgs peregrine_interfaces rclcpp rclcpp_components rclcpp_lifecycle sensor_msgs tf2_geometry_msgs tf2_ros generate_parameter_library)

ament_package()
