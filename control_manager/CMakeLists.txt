cmake_minimum_required(VERSION 3.8)
project(control_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  control_manager_parameters
  src/control_manager_parameters.yaml
)

add_library(control_manager_component SHARED
  src/control_manager_node.cpp
  src/px4_passthrough_controller.cpp
  src/se3_controller.cpp
)

target_include_directories(control_manager_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(control_manager_component PUBLIC cxx_std_17)

target_link_libraries(control_manager_component Eigen3::Eigen control_manager_parameters)

ament_target_dependencies(control_manager_component
  peregrine_interfaces
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
)

rclcpp_components_register_node(control_manager_component
  PLUGIN "control_manager::ControlManagerNode"
  EXECUTABLE control_manager_node
)

install(TARGETS control_manager_component
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
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_so3_utils test/test_so3_utils.cpp)
  target_include_directories(test_so3_utils PRIVATE include)
  target_link_libraries(test_so3_utils Eigen3::Eigen)

  ament_add_gtest(test_se3_controller test/test_se3_controller.cpp)
  target_include_directories(test_se3_controller PRIVATE include)
  target_link_libraries(test_se3_controller control_manager_component Eigen3::Eigen)
  ament_target_dependencies(test_se3_controller peregrine_interfaces)
endif()

pluginlib_export_plugin_description_file(control_manager plugins.xml)

ament_export_include_directories(include)
ament_export_dependencies(peregrine_interfaces pluginlib rclcpp rclcpp_components rclcpp_lifecycle generate_parameter_library)

ament_package()
