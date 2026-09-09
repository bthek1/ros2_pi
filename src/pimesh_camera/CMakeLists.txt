cmake_minimum_required(VERSION 3.16)
project(pimesh_camera)

# C++17, and here it is not a convention but the constraint: this is the package
# the Pi builds, Jazzy's baseline is 17, and a C++20 feature would compile on
# this box and fail at the far end of an rsync. STANDARD_REQUIRED makes a
# missing standard a configure error rather than a silent downgrade; EXTENSIONS
# OFF asks for -std=c++17 over -std=gnu++17, so a GNU extension cannot creep in
# unnoticed and take the Pi's build with it.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(camera_components SHARED
  src/camera_node.cpp
  src/v4l2_capture.cpp
)
target_include_directories(camera_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Plain target_link_libraries against the exported targets, never
# ament_target_dependencies(): that macro was deprecated in Jazzy and *removed*
# in Lyrical, so it builds on the Pi and stops the dev box dead with "Unknown
# CMake command". This spelling is correct on both.
target_link_libraries(camera_components PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
)

rclcpp_components_register_nodes(camera_components
  "pimesh_camera::CameraNode"
)

add_executable(camera_node src/camera_node_main.cpp)
target_link_libraries(camera_node PRIVATE camera_components rclcpp::rclcpp)

# The gate's instrument, not part of the pipeline. It is here rather than in a
# test directory because it is a ROS node that has to be built by the same
# toolchain against the same message types, and because the thing it measures
# is this package's claim.
add_executable(capture_probe src/capture_probe_main.cpp)
target_link_libraries(capture_probe PRIVATE rclcpp::rclcpp ${sensor_msgs_TARGETS})

install(DIRECTORY include/ DESTINATION include)

# The library where the component loader looks, the executables where `ros2 run`
# looks. Two lookups, two destinations; swapping them breaks exactly one.
install(TARGETS camera_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS camera_node capture_probe
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(camera_components)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs)

ament_package()
