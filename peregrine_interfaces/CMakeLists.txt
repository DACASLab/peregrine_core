cmake_minimum_required(VERSION 3.8)
project(peregrine_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate (or refresh) interface file list cmake include
execute_process(
  COMMAND bash -c "QUIET=1 ${CMAKE_CURRENT_SOURCE_DIR}/generate_interface_lists.bash"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE _gen_if_res
)
if(NOT _gen_if_res EQUAL 0)
  message(FATAL_ERROR "Failed to generate interface file lists")
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/interface_files.cmake OPTIONAL)

set(_all_interface_files ${MSG_FILES} ${SRV_FILES})
if(_all_interface_files)
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${_all_interface_files}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
