cmake_minimum_required(VERSION 3.8)
project(multi_agent_coordinator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  multi_agent_coordinator_parameters
  src/multi_agent_coordinator_parameters.yaml
)

add_library(multi_agent_coordinator_component SHARED
  src/multi_agent_coordinator_node.cpp
  src/bvc_calculator.cpp
  src/neighbor_tracker.cpp
)

target_include_directories(multi_agent_coordinator_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(multi_agent_coordinator_component
  multi_agent_coordinator_parameters
  Eigen3::Eigen
)

target_compile_features(multi_agent_coordinator_component PUBLIC cxx_std_17)

ament_target_dependencies(multi_agent_coordinator_component
  peregrine_interfaces
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

rclcpp_components_register_node(multi_agent_coordinator_component
  PLUGIN "multi_agent_coordinator::MultiAgentCoordinatorNode"
  EXECUTABLE multi_agent_coordinator_node
)

install(TARGETS multi_agent_coordinator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Pure-geometry test: link only the geometry translation unit, no ROS needed.
  ament_add_gtest(test_bvc_calculator
    test/test_bvc_calculator.cpp
    src/bvc_calculator.cpp
  )
  target_include_directories(test_bvc_calculator PRIVATE include)
  target_link_libraries(test_bvc_calculator Eigen3::Eigen)
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  peregrine_interfaces rclcpp rclcpp_components geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs generate_parameter_library
)

ament_package()
