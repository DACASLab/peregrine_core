cmake_minimum_required(VERSION 3.8)
project(peregrine_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Peregrine-specific BT nodes (concrete action/condition/service nodes)
add_library(peregrine_bt_nodes SHARED
  src/condition_nodes.cpp
  src/action_nodes.cpp
  src/service_nodes.cpp
  src/utility_nodes.cpp
  src/register_nodes.cpp
)

target_include_directories(peregrine_bt_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(peregrine_bt_nodes PUBLIC cxx_std_17)

ament_target_dependencies(peregrine_bt_nodes
  behaviortree_cpp
  behaviortree_ros2
  btcpp_ros2_interfaces
  peregrine_interfaces
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
)

# BT executor node (standalone executable, not composable)
add_executable(bt_executor_node src/bt_executor_node.cpp)

target_include_directories(bt_executor_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(bt_executor_node peregrine_bt_nodes)

target_compile_features(bt_executor_node PUBLIC cxx_std_17)

ament_target_dependencies(bt_executor_node
  behaviortree_cpp
  behaviortree_ros2
  peregrine_interfaces
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
)

install(TARGETS peregrine_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bt_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(peregrine_bt_nodes)
ament_export_dependencies(
  behaviortree_cpp
  behaviortree_ros2
  btcpp_ros2_interfaces
  peregrine_interfaces
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
)

ament_package()
