cmake_minimum_required(VERSION 3.8)
project(frame_transforms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(frame_transformer_component SHARED
  src/frame_transformer.cpp
)

target_include_directories(frame_transformer_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(frame_transformer_component PUBLIC cxx_std_17)

ament_target_dependencies(frame_transformer_component
  Eigen3
  geometry_msgs
  nav_msgs
  peregrine_interfaces
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

rclcpp_components_register_node(frame_transformer_component
  PLUGIN "frame_transforms::FrameTransformer"
  EXECUTABLE frame_transformer_node
)

install(TARGETS frame_transformer_component
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
ament_export_dependencies(Eigen3 geometry_msgs nav_msgs peregrine_interfaces rclcpp sensor_msgs tf2_ros)

ament_package()
