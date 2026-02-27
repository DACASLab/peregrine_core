cmake_minimum_required(VERSION 3.8)
project(rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(flight_visualizer_component SHARED
  src/flight_visualizer_node.cpp
)

target_include_directories(flight_visualizer_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(flight_visualizer_component PUBLIC cxx_std_17)

ament_target_dependencies(flight_visualizer_component
  geometry_msgs
  nav_msgs
  peregrine_interfaces
  rclcpp
  rclcpp_components
  std_msgs
  visualization_msgs
)

rclcpp_components_register_nodes(flight_visualizer_component "rviz_plugins::FlightVisualizerNode")

add_executable(flight_visualizer_node src/main.cpp)
target_link_libraries(flight_visualizer_node flight_visualizer_component)
target_compile_features(flight_visualizer_node PUBLIC cxx_std_17)
ament_target_dependencies(flight_visualizer_node rclcpp)

install(TARGETS
  flight_visualizer_component
  flight_visualizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  peregrine_interfaces
  rclcpp
  rclcpp_components
  std_msgs
  visualization_msgs
)

ament_package()
