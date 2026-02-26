cmake_minimum_required(VERSION 3.8)
project(hardware_abstraction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(frame_transforms REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(hardware_abstraction_component SHARED
  src/px4_hardware_abstraction.cpp
)

target_include_directories(hardware_abstraction_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(hardware_abstraction_component PUBLIC cxx_std_17)

ament_target_dependencies(hardware_abstraction_component
  Eigen3
  frame_transforms
  nav_msgs
  peregrine_interfaces
  px4_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_node(hardware_abstraction_component
  PLUGIN "hardware_abstraction::PX4HardwareAbstraction"
  EXECUTABLE hardware_abstraction_node
)

install(TARGETS hardware_abstraction_component
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
ament_export_dependencies(Eigen3 frame_transforms nav_msgs peregrine_interfaces px4_msgs rclcpp sensor_msgs)

ament_package()
