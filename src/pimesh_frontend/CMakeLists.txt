cmake_minimum_required(VERSION 3.16)
project(pimesh_frontend)

# C++17, not 20, even though nothing in this package runs on the Pi.
# tools/gates/build.sh builds the whole workspace at both ends, so a C++20
# feature here stops the Pi's build — and the cost of finding that out is an
# rsync and a five-minute compile on four cores.
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
find_package(geometry_msgs REQUIRED)
# P7's pose stream. nav_msgs/Odometry is a *sequence* where TF is a tree queried
# by time, and it is what an RViz Odometry display draws a trail from — so the
# trajectory is viewable without a nav_msgs/Path publisher existing anywhere.
find_package(nav_msgs REQUIRED)
find_package(pimesh_msgs REQUIRED)
# The single-slot mailbox every node here drops frames through, the cv::Mat
# header laid over an image without copying it, and the percentile the probes
# report. Header-only, so this is include directories and nothing to link.
find_package(pimesh_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# The apt OpenCV: 4.10.0 here, 4.6.0 on the Pi, and **neither has the CUDA
# module** — `cv::cuda::` does not link on this box, so nothing here reaches for
# it. The components are named rather than taking the whole of OpenCV: it makes
# the dependency legible, and `features2d` appearing in this list is the line that
# says where ORB comes from.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d)

add_library(frontend_components SHARED
  src/decode_node.cpp
  probes/ipc_probe.cpp
  src/keyframe_store.cpp
  src/keypoint_node.cpp
  probes/odom_probe.cpp
  src/odometry_node.cpp
  src/orb_tracker.cpp
  src/rgbd_odometry.cpp
  src/rotation_fit.cpp
)
target_include_directories(frontend_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pimesh_core_INCLUDE_DIRS}
)
# Plain target_link_libraries against exported targets, never
# ament_target_dependencies(): removed in Lyrical, present in Jazzy, so the macro
# builds on the Pi and stops the dev box with "Unknown CMake command".
target_link_libraries(frontend_components PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${pimesh_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
  opencv_features2d
  opencv_calib3d
)

# Plural, so no main is generated: the thin ones are written by hand in src/.
rclcpp_components_register_nodes(frontend_components
  "pimesh_frontend::DecodeNode"
  "pimesh_frontend::IpcProbe"
  "pimesh_frontend::OdomProbe"
  "pimesh_frontend::KeypointNode"
  "pimesh_frontend::OdometryNode"
)

foreach(node decode_node keypoint_node odometry_node)
  add_executable(${node} apps/${node}_main.cpp)
  target_link_libraries(${node} PRIVATE frontend_components rclcpp::rclcpp)
endforeach()

# The instrument tools/gates/keypoints.sh measures with. A C++ subscriber rather
# than `ros2 topic hz`, for the reason pimesh_camera's capture_probe gives at
# length: a Python subscriber to 500-feature messages at 50 Hz has scheduling
# error the size of the thing being measured, and it has nothing to say about
# what is *in* the messages. This one reports the matched-keypoint fraction off
# the published track ids, which is the number the gate compares against the
# predecessor's.
add_executable(keypoint_probe apps/keypoint_probe.cpp)
target_include_directories(keypoint_probe PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${pimesh_core_INCLUDE_DIRS})
target_link_libraries(keypoint_probe PRIVATE rclcpp::rclcpp ${pimesh_msgs_TARGETS})

install(DIRECTORY include/ DESTINATION include)

# The library where the component loader looks, the executables where `ros2 run`
# looks. Two lookups, two destinations.
install(TARGETS frontend_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS decode_node keypoint_node odometry_node keypoint_probe
  DESTINATION lib/${PROJECT_NAME}
)

# --- Tests ------------------------------------------------------------------
#
# No camera, no Pi, no GPU: everything here is logic that can be got wrong
# silently. The mailbox's drop accounting and the bgr8 layout arithmetic are both
# the kind of mistake that produces a plausible-looking image and a wrong number.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Reading a Keypoints message back into geometry. Header-only, no node, no ROS
  # graph — and it is a header at all because it was three loops inside
  # odometry_node's subscription callback, where nothing could call it. Two of
  # those loops were bounded by one array's length while indexing another, so a
  # message with ragged parallel arrays read past the end of a vector; that is the
  # one assertion in this package about undefined behaviour rather than about a
  # plausible wrong answer.
  ament_add_gtest(test_keypoints_view test/test_keypoints_view.cpp)
  if(TARGET test_keypoints_view)
    target_include_directories(test_keypoints_view PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include ${pimesh_core_INCLUDE_DIRS})
    target_link_libraries(test_keypoints_view ${pimesh_msgs_TARGETS} opencv_core)
  endif()

  # The geometry. Every failure here is silent by construction: a rotation fit
  # returns a plausible matrix for mismatched pairs, a missing determinant
  # correction returns a mirror, and a basis change applied the wrong way round
  # turns a yaw into a roll and still produces a TF frame that moves when you pan.
  ament_add_gtest(test_rotation_fit test/test_rotation_fit.cpp)
  if(TARGET test_rotation_fit)
    target_link_libraries(test_rotation_fit frontend_components)
  endif()

  # The tracker, against synthetic frames whose true motion is known. It is the
  # one part of this package that can be wrong by *degree* rather than outright —
  # a matcher that is 20% worse still tracks, still draws green circles, and shows
  # up only as a mesh that drifts.
  ament_add_gtest(test_orb_tracker test/test_orb_tracker.cpp)
  if(TARGET test_orb_tracker)
    target_link_libraries(test_orb_tracker frontend_components opencv_imgproc)
  endif()

  # P7's geometry, and the suite that closes the loop test_rotation_fit left open.
  # It simulates a camera with a *known* motion, generates what it would have seen,
  # runs the fit, and asserts the composed pose is that motion — which is the check
  # that was missing while the rotation-only path composed the inverse of it for six
  # days, publishing a frame that turned left when the camera panned right. Nothing
  # failed then and nothing would now: every other property of that pose was right.
  ament_add_gtest(test_rgbd_odometry test/test_rgbd_odometry.cpp)
  if(TARGET test_rgbd_odometry)
    target_link_libraries(test_rgbd_odometry frontend_components)
  endif()

  # The keyframe store's admission rule. Nothing consumes the store yet, which is
  # exactly why it needs a test: a threshold that never fires and one that fires on
  # every frame both produce a running pipeline, and only the memory figure would
  # ever say which.
  ament_add_gtest(test_keyframe_store test/test_keyframe_store.cpp)
  if(TARGET test_keyframe_store)
    target_link_libraries(test_keyframe_store frontend_components)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(frontend_components)
ament_export_dependencies(
  rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs pimesh_msgs
  pimesh_core tf2 tf2_ros OpenCV)

ament_package()
