cmake_minimum_required(VERSION 3.8)
project(peregrine_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Keep the interface list deterministic whenever message/service files are added.
execute_process(
  COMMAND bash -c "QUIET=1 ${CMAKE_CURRENT_SOURCE_DIR}/generate_interface_lists.bash"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE _gen_if_res
)
if(NOT _gen_if_res EQUAL 0)
  message(FATAL_ERROR "Failed to generate interface file lists")
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/interface_files.cmake OPTIONAL)

set(_all_interface_files ${MSG_FILES} ${SRV_FILES} ${ACTION_FILES})
if(_all_interface_files)
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${_all_interface_files}
    DEPENDENCIES std_msgs geometry_msgs action_msgs builtin_interfaces
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
