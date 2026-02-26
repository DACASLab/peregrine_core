cmake_minimum_required(VERSION 3.8)
project(uav_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(uav_manager_component SHARED
  src/action_orchestrator.cpp
  src/health_aggregator.cpp
  src/supervisor_state_machine.cpp
  src/transition_guard.cpp
  src/uav_manager_node.cpp
)

target_include_directories(uav_manager_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(uav_manager_component PUBLIC cxx_std_17)

ament_target_dependencies(uav_manager_component
  peregrine_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
)

# Register the component plugin for composition.
# IMPORTANT: uav_manager uses a ReentrantCallbackGroup for action servers.
# It MUST be loaded into component_container_mt (MultiThreadedExecutor),
# NOT component_container (SingleThreadedExecutor).
rclcpp_components_register_node(uav_manager_component
  PLUGIN "uav_manager::UavManagerNode"
  EXECUTABLE uav_manager_node
)

install(TARGETS uav_manager_component
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
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_supervisor_state_machine
    test/test_supervisor_state_machine.cpp
    src/supervisor_state_machine.cpp
    src/transition_guard.cpp
    src/health_aggregator.cpp
  )
  target_include_directories(test_supervisor_state_machine PRIVATE include)
  ament_target_dependencies(test_supervisor_state_machine rclcpp)

  ament_add_gtest(test_health_aggregator
    test/test_health_aggregator.cpp
    src/health_aggregator.cpp
  )
  target_include_directories(test_health_aggregator PRIVATE include)
  ament_target_dependencies(test_health_aggregator rclcpp)
endif()

ament_export_include_directories(include)
ament_export_dependencies(peregrine_interfaces rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle)

ament_package()
