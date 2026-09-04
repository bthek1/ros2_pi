cmake_minimum_required(VERSION 3.16)
project(pimesh_camera)

# C++17, not C++20: this package builds on the Pi under Jazzy, with g++ 13.3.0,
# and Jazzy's baseline is 17. The dev box builds it too — as a compile check,
# since it has no camera.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pimesh_msgs REQUIRED)

# The V4L2 half is a plain library with no ROS in it, so it can be reasoned
# about — and later tested — without spinning a node.
add_library(v4l2_capture src/v4l2_capture.cpp)
target_include_directories(v4l2_capture PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Namespaced targets, NOT ament_target_dependencies(). That command is gone in
# Lyrical (the dev box) and still present in Jazzy (the Pi); `rclcpp::rclcpp`
# and the generated `<pkg>_TARGETS` variables exist in both. This package has to
# compile under two ROS distros, so it uses the form both of them have.
add_library(camera_node_component SHARED src/camera_node.cpp)
target_include_directories(camera_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camera_node_component PUBLIC
  v4l2_capture
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${pimesh_msgs_TARGETS})
rclcpp_components_register_node(camera_node_component
  PLUGIN "pimesh_camera::CameraNode"
  EXECUTABLE camera_node_composed)

# The standalone executable the Pi actually runs. It exists so that a capture
# failure becomes a non-zero exit code rather than a silently idle process.
add_executable(camera_node src/camera_node_main.cpp)
target_link_libraries(camera_node PRIVATE camera_node_component rclcpp::rclcpp)

install(TARGETS v4l2_capture camera_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
