cmake_minimum_required(VERSION 3.8)
project(frame_transforms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


# --- FRAME TRANSFORMER LIBRARY ---
add_library(
  frame_transformer SHARED
  src/frame_transformer.cpp
  src/px4_transforms.cpp
)
target_include_directories(frame_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(frame_transformer
  rclcpp
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  Eigen3
)

# --- TF CONNECTOR COMPONENT ---
add_library(tf_connector_component SHARED src/tf_connector.cpp)
target_link_libraries(tf_connector_component frame_transformer)
ament_target_dependencies(tf_connector_component
  rclcpp
  rclcpp_components
  tf2_ros
  geometry_msgs
)

# --- TRANSFORM BROADCASTER COMPONENT ---
add_library(transform_broadcaster_component SHARED src/transform_broadcaster.cpp)
target_link_libraries(transform_broadcaster_component frame_transformer)
ament_target_dependencies(transform_broadcaster_component
  rclcpp
  rclcpp_components
  tf2_ros
  geometry_msgs
)


rclcpp_components_register_node(tf_connector_component
  PLUGIN "frame_transforms::TFConnectorComponent"
  EXECUTABLE tf_connector_node
)

rclcpp_components_register_node(transform_broadcaster_component
  PLUGIN "frame_transforms::TransformBroadcasterComponent"
  EXECUTABLE transform_broadcaster_node
)


# Install targets
install(TARGETS
frame_transformer
tf_connector_component
transform_broadcaster_component
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

install(DIRECTORY include/
DESTINATION include
)

# Add example executables
add_subdirectory(examples)

# # === Install launch files ===
# install(
#   DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# === Install example launch files ===
install(
  DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}/examples
  FILES_MATCHING PATTERN "*.launch.py"
)


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

ament_package()
