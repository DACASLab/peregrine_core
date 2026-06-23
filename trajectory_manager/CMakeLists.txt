cmake_minimum_required(VERSION 3.8)
project(trajectory_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(frame_transforms REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

generate_parameter_library(
  trajectory_manager_parameters
  src/trajectory_manager_parameters.yaml
)

add_library(trajectory_manager_component SHARED
  src/trajectory_manager_node.cpp
  src/generators.cpp
  src/path_smoothing.cpp
  src/coverage_planner.cpp
)

target_include_directories(trajectory_manager_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(trajectory_manager_component
  trajectory_manager_parameters
  Eigen3::Eigen
)

target_compile_features(trajectory_manager_component PUBLIC cxx_std_17)

ament_target_dependencies(trajectory_manager_component
  frame_transforms
  peregrine_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  nav_msgs
  std_msgs
  visualization_msgs
)

rclcpp_components_register_node(trajectory_manager_component
  PLUGIN "trajectory_manager::TrajectoryManagerNode"
  EXECUTABLE trajectory_manager_node
)

install(TARGETS trajectory_manager_component
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

  add_executable(test_coverage_sweep test/test_coverage_sweep.cpp)
  target_link_libraries(test_coverage_sweep trajectory_manager_component Eigen3::Eigen)
  ament_target_dependencies(test_coverage_sweep peregrine_interfaces rclcpp)
  install(TARGETS test_coverage_sweep DESTINATION lib/${PROJECT_NAME})

  add_executable(test_separation test/test_separation.cpp)
  target_link_libraries(test_separation trajectory_manager_component Eigen3::Eigen)
  ament_target_dependencies(test_separation peregrine_interfaces rclcpp)
  install(TARGETS test_separation DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_dependencies(frame_transforms peregrine_interfaces rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle generate_parameter_library nav_msgs std_msgs visualization_msgs)

ament_package()
