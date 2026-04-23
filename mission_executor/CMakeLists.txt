cmake_minimum_required(VERSION 3.8)
project(mission_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(mission_executor_lib
  src/mission_executor_node.cpp
)
target_include_directories(mission_executor_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mission_executor_lib PUBLIC cxx_std_17)
ament_target_dependencies(mission_executor_lib behaviortree_cpp peregrine_interfaces rclcpp rclcpp_action)

add_executable(mission_executor_node src/main.cpp)
target_link_libraries(mission_executor_node mission_executor_lib)
ament_target_dependencies(mission_executor_node behaviortree_cpp peregrine_interfaces rclcpp rclcpp_action)

install(TARGETS mission_executor_lib mission_executor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch trees config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(behaviortree_cpp peregrine_interfaces rclcpp rclcpp_action)
ament_package()
