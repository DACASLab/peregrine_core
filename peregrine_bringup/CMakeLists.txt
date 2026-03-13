cmake_minimum_required(VERSION 3.8)
project(peregrine_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_capture_tracking_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(mocap_relay_component SHARED src/mocap_relay.cpp)
target_include_directories(mocap_relay_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mocap_relay_component PUBLIC cxx_std_17)
ament_target_dependencies(mocap_relay_component
  geometry_msgs motion_capture_tracking_interfaces rclcpp rclcpp_components)

rclcpp_components_register_node(mocap_relay_component
  PLUGIN "peregrine_bringup::MocapRelay"
  EXECUTABLE mocap_relay_node)

install(TARGETS mocap_relay_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
