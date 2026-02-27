cmake_minimum_required(VERSION 3.8)
project(tui_status)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Curses REQUIRED)
find_package(peregrine_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(tui_status_node
  src/main.cpp
  src/tui_node.cpp
  src/renderer.cpp
  src/alert_buffer.cpp
)

target_include_directories(tui_status_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CURSES_INCLUDE_DIRS}
)

target_compile_features(tui_status_node PUBLIC cxx_std_17)

target_link_libraries(tui_status_node
  ${CURSES_LIBRARIES}
)

ament_target_dependencies(tui_status_node
  peregrine_interfaces
  rclcpp
)

install(TARGETS tui_status_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
ament_export_dependencies(peregrine_interfaces rclcpp)

ament_package()
