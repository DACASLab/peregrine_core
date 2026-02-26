cmake_minimum_required(VERSION 3.8)
project(trajectory_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(trajectory_manager_component SHARED
  src/trajectory_manager_node.cpp
  src/generators.cpp
)

target_include_directories(trajectory_manager_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(trajectory_manager_component PUBLIC cxx_std_17)

ament_target_dependencies(trajectory_manager_component
  peregrine_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
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

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(peregrine_interfaces rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle)

ament_package()
