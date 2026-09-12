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
find_package(ament_index_cpp REQUIRED)
# yaml-cpp, and the spelling matters. ROS ships a `yaml_cpp_vendor` package on
# both distros, but it does *different things*: Lyrical's appends a bare
# `yaml-cpp` to yaml_cpp_vendor_TARGETS, Jazzy's redirects yaml-cpp_DIR at a
# vendored copy. The system package's own config exports
# `yaml-cpp::yaml-cpp` identically on both (checked 2026-09-12), so this goes
# straight at it and skips the vendor entirely.
find_package(yaml-cpp REQUIRED)

add_library(camera_components SHARED
  src/calibration.cpp
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
  ament_index_cpp::ament_index_cpp
  yaml-cpp::yaml-cpp
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

# --- Tests ------------------------------------------------------------------
#
# BUILD_TESTING is on by default under `colcon test` and off for a plain build,
# so this costs the Pi nothing on the ordinary path. Every test here runs
# *without a camera* — that is a requirement rather than an accident, because
# the dev box has no capture device and a test suite that only runs on the Pi is
# one that stops being run.
#
# What needs hardware is covered by tools/gates/capture.sh instead: the frame
# rate, the two-launch stamp comparison against a real sensor, and the busy
# device. The split is deliberate — these are fast, hermetic and assert on
# logic; the gate is slow, physical and asserts on numbers.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The stamp arithmetic, which is the claim this whole package exists to make.
  ament_add_gtest(test_stamp test/test_stamp.cpp)
  if(TARGET test_stamp)
    target_include_directories(test_stamp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  # The calibration loader's refusals. Every one of these is a file that parses
  # as YAML and is wrong anyway, which is the only interesting category: a
  # syntax error announces itself, and a 1280x720 calibration on a 1080p stream
  # does not.
  ament_add_gtest(test_calibration test/test_calibration.cpp)
  if(TARGET test_calibration)
    target_link_libraries(test_calibration camera_components)
  endif()

  ament_add_gtest(test_camera_info test/test_camera_info.cpp)
  if(TARGET test_camera_info)
    target_include_directories(test_camera_info PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_camera_info ${sensor_msgs_TARGETS})
  endif()

  # The refusal paths. Links the real library, because the thing under test is
  # the ioctl sequence's error handling and a reimplementation of it would test
  # itself.
  ament_add_gtest(test_v4l2_capture test/test_v4l2_capture.cpp)
  if(TARGET test_v4l2_capture)
    target_link_libraries(test_v4l2_capture camera_components)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(camera_components)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs ament_index_cpp yaml-cpp)

ament_package()
