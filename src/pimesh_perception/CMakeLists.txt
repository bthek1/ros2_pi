cmake_minimum_required(VERSION 3.16)
project(pimesh_perception)

# C++17. This package is dev-box only — the Pi builds `pimesh_msgs` and
# `pimesh_camera` and nothing else — so Lyrical's C++20 would work here. It
# stays at 17 so that moving a helper into a shared package later does not
# quietly become a Pi build failure.
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
# The apt OpenCV, 4.10.0, WITHOUT the CUDA module. `cv::cuda::` will not link
# and nothing here may reach for it — see CLAUDE.md.
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc features2d)
# P3. Eigen is header-only and already a transitive dependency of tf2 and PCL,
# so this costs nothing to link and buys readable 3x3 geometry — the rotation
# core reads like the maths it implements instead of like index arithmetic.
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

# P4: ONNX Runtime, the GPU build, installed by `just install-onnxruntime` into
# a user-owned prefix. NOT a system package and NOT findable by pkg-config's
# default search path, so the location is passed in explicitly — `just build`
# sets ONNXRUNTIME_ROOT. Without it this package still builds, minus depth_node,
# which is deliberate: a fresh clone should compile before it has downloaded
# 2 GB of CUDA runtime.
set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH
  "root of an ONNX Runtime GPU install (see just install-onnxruntime)")
set(PIMESH_HAVE_ORT FALSE)
if(ONNXRUNTIME_ROOT AND EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
  set(PIMESH_HAVE_ORT TRUE)
  message(STATUS "pimesh: ONNX Runtime at ${ONNXRUNTIME_ROOT}")
else()
  message(WARNING
    "pimesh: no ONNX Runtime — SKIPPING depth_node. "
    "Run `just install-onnxruntime`, then rebuild.")
endif()

# The ROS-free half, so the logic can be tested without a node or a camera.
# `mailbox.hpp` is header-only (a template); the rest need a TU.
#
# This library is where P3's real content lives. `rotation.cpp` is driven by
# test_rotation with synthetic ray bundles and known rotations — a thing the
# node could never be tested for, because the node needs a camera and a room.
add_library(perception_core
  src/jpeg.cpp src/orb_tracker.cpp src/rotation.cpp src/depth_convert.cpp)
target_include_directories(perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(perception_core PUBLIC ${OpenCV_LIBS} Eigen3::Eigen)

# Namespaced targets, not ament_target_dependencies() — same reason as
# pimesh_camera: that command is gone in Lyrical. This package is dev-box only,
# but the house form is the one that works on both distros.
add_library(decode_node_component SHARED src/decode_node.cpp)
target_include_directories(decode_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(decode_node_component PUBLIC
  perception_core
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${pimesh_msgs_TARGETS})
rclcpp_components_register_node(decode_node_component
  PLUGIN "pimesh_perception::DecodeNode"
  EXECUTABLE decode_node_composed)

# P3: ORB features and the rotation-only odometer.
add_library(keypoint_node_component SHARED src/keypoint_node.cpp)
target_include_directories(keypoint_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(keypoint_node_component PUBLIC
  perception_core
  rclcpp::rclcpp
  rclcpp_components::component
  tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${pimesh_msgs_TARGETS})
rclcpp_components_register_node(keypoint_node_component
  PLUGIN "pimesh_perception::KeypointNode"
  EXECUTABLE keypoint_node_composed)

# P4: monocular depth on the GPU. The only target in the workspace that needs
# something outside apt and ROS, hence the guard.
if(PIMESH_HAVE_ORT)
  add_library(depth_core src/depth_model.cpp)
  target_include_directories(depth_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ONNXRUNTIME_ROOT}/include)
  # perception_core holds the pure preprocessing and the inverse-depth
  # conversion — the parts of this stage that are testable without a GPU.
  target_link_libraries(depth_core PUBLIC
    perception_core ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so)

  add_library(depth_node_component SHARED src/depth_node.cpp)
  target_include_directories(depth_node_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(depth_node_component PUBLIC
    depth_core
    rclcpp::rclcpp
    rclcpp_components::component
    ${sensor_msgs_TARGETS}
    ${pimesh_msgs_TARGETS})
  rclcpp_components_register_node(depth_node_component
    PLUGIN "pimesh_perception::DepthNode"
    EXECUTABLE depth_node_composed)

  add_executable(depth_node src/depth_node_main.cpp)
  target_link_libraries(depth_node PRIVATE depth_node_component rclcpp::rclcpp)

  install(TARGETS depth_core depth_node_component
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  install(TARGETS depth_node DESTINATION lib/${PROJECT_NAME})
endif()

# The zero-copy probe. A diagnostic, loaded only when the launch is asked for
# it (`probe:=true`) and by `just gate-ipc`.
add_library(ipc_probe_node_component SHARED src/ipc_probe_node.cpp)
target_include_directories(ipc_probe_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ipc_probe_node_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS})
rclcpp_components_register_node(ipc_probe_node_component
  PLUGIN "pimesh_perception::IpcProbeNode"
  EXECUTABLE ipc_probe_node_composed)

# Standalone runner, for looking at one stage with nothing else in the picture.
add_executable(decode_node src/decode_node_main.cpp)
target_link_libraries(decode_node PRIVATE decode_node_component rclcpp::rclcpp)

add_executable(keypoint_node src/keypoint_node_main.cpp)
target_link_libraries(keypoint_node PRIVATE keypoint_node_component rclcpp::rclcpp)

install(TARGETS perception_core decode_node_component keypoint_node_component
  ipc_probe_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS decode_node keypoint_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Nothing here may need a camera, the Pi, or the network. The mailbox is
  # pure logic; the decode is exercised against JPEGs made on the spot with
  # cv::imencode.
  ament_add_gtest(test_mailbox test/test_mailbox.cpp)
  target_link_libraries(test_mailbox perception_core)

  ament_add_gtest(test_jpeg test/test_jpeg.cpp)
  target_link_libraries(test_jpeg perception_core)

  # Synthetic ray bundles rotated by a known R. No camera, no room, no ROS.
  ament_add_gtest(test_rotation test/test_rotation.cpp)
  target_link_libraries(test_rotation perception_core)

  # Synthetic images with drawn corners, so detection and both matchings can
  # be exercised without the Pi.
  ament_add_gtest(test_orb_tracker test/test_orb_tracker.cpp)
  target_link_libraries(test_orb_tracker perception_core)

  # The two pure steps either side of the depth model. In perception_core and
  # not depth_core on purpose: these run on a machine with no ONNX Runtime and
  # no GPU.
  ament_add_gtest(test_depth_convert test/test_depth_convert.cpp)
  target_link_libraries(test_depth_convert perception_core)
endif()

ament_package()
