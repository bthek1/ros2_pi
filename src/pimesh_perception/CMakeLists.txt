cmake_minimum_required(VERSION 3.16)
project(pimesh_perception)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# The apt OpenCV: 4.10.0 here, 4.6.0 on the Pi, and **neither has the CUDA
# module** — `cv::cuda::` does not link on this box, so nothing here reaches for
# it. The components are named rather than taking the whole of OpenCV: it makes
# the dependency legible, and `features2d` appearing in this list is the line that
# says where ORB comes from.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d)

# --- ONNX Runtime, and the whole cross-machine story in one if() -------------
#
# Dev box only, and *optional*, which is the load-bearing word. The Pi builds this
# entire workspace — tools/gates/build.sh builds it at both ends — and the GPU
# stack is x86-64 CUDA that will never exist there. So this is a find, not a
# REQUIRED find, and what changes with it is only which DepthEngine gets compiled:
# depth_node itself, its registration and its parameters build identically on both
# machines. See include/pimesh_perception/depth_engine.hpp for why that matters —
# pimesh_bringup's test_transforms asserts every launch plugin is registered, and
# gates/test.sh asserts the same suites pass at both ends with zero skips.
#
# The prefix is not written down here: tools/fetch-gpu-stack.sh --print-prefix is
# the one place that knows it, so PIMESH_GPU_PREFIX moves the install and nothing
# else has to be told.
execute_process(
  COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/fetch-gpu-stack.sh --print-prefix
  OUTPUT_VARIABLE PIMESH_GPU_PREFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
find_library(ONNXRUNTIME_LIB onnxruntime
  PATHS "${PIMESH_GPU_PREFIX}/lib" NO_DEFAULT_PATH)
find_path(ONNXRUNTIME_INCLUDE onnxruntime_cxx_api.h
  PATHS "${PIMESH_GPU_PREFIX}/include" NO_DEFAULT_PATH)

if(ONNXRUNTIME_LIB AND ONNXRUNTIME_INCLUDE)
  set(PIMESH_HAVE_ONNXRUNTIME TRUE)
  message(STATUS "pimesh_perception: ONNX Runtime at ${PIMESH_GPU_PREFIX} — depth_node will infer")
else()
  set(PIMESH_HAVE_ONNXRUNTIME FALSE)
  message(STATUS
    "pimesh_perception: no ONNX Runtime — depth_node builds but refuses to run. "
    "This is expected on the Pi; on the dev box run bash tools/fetch-gpu-stack.sh")
endif()

add_library(perception_components SHARED
  src/decode_node.cpp
  src/depth_node.cpp
  src/depth_probe.cpp
  src/ipc_probe.cpp
  src/keyframe_store.cpp
  src/keypoint_node.cpp
  src/odom_probe.cpp
  src/odometry_node.cpp
  src/orb_tracker.cpp
  src/rgbd_odometry.cpp
  src/rotation_fit.cpp
  # Exactly one of these two is compiled, and the choice is the only thing the
  # GPU stack's presence changes about this library.
  $<$<BOOL:${PIMESH_HAVE_ONNXRUNTIME}>:src/depth_engine_ort.cpp>
  $<$<NOT:$<BOOL:${PIMESH_HAVE_ONNXRUNTIME}>>:src/depth_engine_null.cpp>
)
target_include_directories(perception_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Plain target_link_libraries against exported targets, never
# ament_target_dependencies(): removed in Lyrical, present in Jazzy, so the macro
# builds on the Pi and stops the dev box with "Unknown CMake command".
target_link_libraries(perception_components PUBLIC
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

# --- The depth build, and the linker flag that decides whether it uses the GPU --
#
# The default model path is baked in here rather than computed at runtime.
# CMAKE_CURRENT_SOURCE_DIR is <ws>/src/pimesh_perception, so two `..` reach the
# workspace; the alternative — walking up from get_package_share_directory() at
# runtime — is four segments of assumption about a layout --symlink-install
# already bends. models/ is git-ignored and 99 MB, so it is read from the source
# tree and never installed into share/.
get_filename_component(PIMESH_WS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
target_compile_definitions(perception_components PRIVATE
  PIMESH_DEFAULT_MODEL_PATH="${PIMESH_WS_DIR}/models/depth_anything_v2_small.onnx")

if(PIMESH_HAVE_ONNXRUNTIME)
  target_include_directories(perception_components PRIVATE ${ONNXRUNTIME_INCLUDE})
  target_link_libraries(perception_components PRIVATE ${ONNXRUNTIME_LIB} ${CMAKE_DL_LIBS})

  # The prefix, baked in, so depth_engine_ort.cpp can dlopen the CUDA provider by
  # absolute path before ONNX Runtime tries to. See that file for why that is
  # load-bearing and not belt-and-braces: the linker flag below gets the provider's
  # *dependencies* found only when the RPATH in the dlopen chain belongs to
  # something that has one, and in a component container it does not.
  target_compile_definitions(perception_components PRIVATE
    PIMESH_GPU_PREFIX="${PIMESH_GPU_PREFIX}")

  # **`--disable-new-dtags` is the difference between the GPU and the CPU, and
  # nothing about the build says so otherwise.**
  #
  # libonnxruntime_providers_cuda.so is *dlopened* by libonnxruntime.so and
  # carries no RPATH or RUNPATH of its own. DT_RUNPATH — which is what CMake and
  # every modern linker emit by default — is **not inherited down a dlopen
  # chain**, while the older DT_RPATH is. So with the default flags the provider
  # cannot find libcublas.so.13 sitting in the very directory it was loaded from,
  # ONNX Runtime falls back to the CPU, and there is no error anywhere: the only
  # symptom is a pipeline four times slower than it should be.
  #
  # Measured 2026-09-15, same source and same libraries, one flag apart:
  # RUNPATH gave CPUExecutionProvider at 236.62 ms, RPATH gave
  # CUDAExecutionProvider at 51.20 ms. tools/gates/gpu-stack.sh keeps a control
  # run asserting the default-flags build does *not* reach CUDA, so this cannot
  # quietly stop being load-bearing.
  #
  # BUILD_WITH_INSTALL_RPATH because colcon --symlink-install runs the library
  # from a path CMake would otherwise rewrite the RPATH for on install.
  set_target_properties(perception_components PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${PIMESH_GPU_PREFIX}/lib")
  target_link_options(perception_components PRIVATE
    "-Wl,--disable-new-dtags"
    "-Wl,-rpath,${PIMESH_GPU_PREFIX}/lib")
endif()

# Plural, so no main is generated: the thin ones are written by hand in src/.
rclcpp_components_register_nodes(perception_components
  "pimesh_perception::DecodeNode"
  "pimesh_perception::DepthNode"
  "pimesh_perception::DepthProbe"
  "pimesh_perception::IpcProbe"
  "pimesh_perception::OdomProbe"
  "pimesh_perception::KeypointNode"
  "pimesh_perception::OdometryNode"
)

foreach(node decode_node depth_node keypoint_node odometry_node)
  add_executable(${node} src/${node}_main.cpp)
  target_link_libraries(${node} PRIVATE perception_components rclcpp::rclcpp)
endforeach()

# The instrument tools/gates/keypoints.sh measures with. A C++ subscriber rather
# than `ros2 topic hz`, for the reason pimesh_camera's capture_probe gives at
# length: a Python subscriber to 500-feature messages at 50 Hz has scheduling
# error the size of the thing being measured, and it has nothing to say about
# what is *in* the messages. This one reports the matched-keypoint fraction off
# the published track ids, which is the number the gate compares against the
# predecessor's.
add_executable(keypoint_probe src/keypoint_probe_main.cpp)
target_include_directories(keypoint_probe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(keypoint_probe PRIVATE rclcpp::rclcpp ${pimesh_msgs_TARGETS})

install(DIRECTORY include/ DESTINATION include)

# The library where the component loader looks, the executables where `ros2 run`
# looks. Two lookups, two destinations.
install(TARGETS perception_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS decode_node depth_node keypoint_node odometry_node keypoint_probe
  DESTINATION lib/${PROJECT_NAME}
)

# --- Tests ------------------------------------------------------------------
#
# No camera, no Pi, no GPU: everything here is logic that can be got wrong
# silently. The mailbox's drop accounting and the bgr8 layout arithmetic are both
# the kind of mistake that produces a plausible-looking image and a wrong number.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The numbers every probe in this package reports through. Header-only, no ROS,
  # no GPU — and until stats.hpp existed there was nothing to point a test at: the
  # percentile lived four times over in anonymous namespaces inside translation
  # units with a node in them. Every failure here is a plausible wrong number
  # rather than a crash, which is the whole category this directory is for.
  ament_add_gtest(test_stats test/test_stats.cpp)
  if(TARGET test_stats)
    target_include_directories(test_stats PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  ament_add_gtest(test_mailbox test/test_mailbox.cpp)
  if(TARGET test_mailbox)
    target_include_directories(test_mailbox PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  # Reading a Keypoints message back into geometry. Header-only, no node, no ROS
  # graph — and it is a header at all because it was three loops inside
  # odometry_node's subscription callback, where nothing could call it. Two of
  # those loops were bounded by one array's length while indexing another, so a
  # message with ragged parallel arrays read past the end of a vector; that is the
  # one assertion in this package about undefined behaviour rather than about a
  # plausible wrong answer.
  ament_add_gtest(test_keypoints_view test/test_keypoints_view.cpp)
  if(TARGET test_keypoints_view)
    target_include_directories(test_keypoints_view PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_keypoints_view ${pimesh_msgs_TARGETS} opencv_core)
  endif()

  ament_add_gtest(test_image_buffer test/test_image_buffer.cpp)
  if(TARGET test_image_buffer)
    target_include_directories(test_image_buffer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_image_buffer ${sensor_msgs_TARGETS} opencv_core)
  endif()

  # The geometry. Every failure here is silent by construction: a rotation fit
  # returns a plausible matrix for mismatched pairs, a missing determinant
  # correction returns a mirror, and a basis change applied the wrong way round
  # turns a yaw into a roll and still produces a TF frame that moves when you pan.
  ament_add_gtest(test_rotation_fit test/test_rotation_fit.cpp)
  if(TARGET test_rotation_fit)
    target_link_libraries(test_rotation_fit perception_components)
  endif()

  # The arithmetic either side of the depth model — the whole of P4 that can be
  # got wrong in silence. No ONNX Runtime, no GPU, no camera, so this suite runs
  # identically on both machines, which is the reason depth_model.hpp is a
  # separate header from the engine behind it. A channel order swapped or a
  # reciprocal taken before the clamp both produce a depth map that renders as a
  # plausible room and is numerically nonsense.
  ament_add_gtest(test_depth_model test/test_depth_model.cpp)
  if(TARGET test_depth_model)
    target_include_directories(test_depth_model PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_depth_model opencv_core opencv_imgproc)
  endif()

  # The tracker, against synthetic frames whose true motion is known. It is the
  # one part of this package that can be wrong by *degree* rather than outright —
  # a matcher that is 20% worse still tracks, still draws green circles, and shows
  # up only as a mesh that drifts.
  ament_add_gtest(test_orb_tracker test/test_orb_tracker.cpp)
  if(TARGET test_orb_tracker)
    target_link_libraries(test_orb_tracker perception_components opencv_imgproc)
  endif()

  # P7's geometry, and the suite that closes the loop test_rotation_fit left open.
  # It simulates a camera with a *known* motion, generates what it would have seen,
  # runs the fit, and asserts the composed pose is that motion — which is the check
  # that was missing while the rotation-only path composed the inverse of it for six
  # days, publishing a frame that turned left when the camera panned right. Nothing
  # failed then and nothing would now: every other property of that pose was right.
  ament_add_gtest(test_rgbd_odometry test/test_rgbd_odometry.cpp)
  if(TARGET test_rgbd_odometry)
    target_link_libraries(test_rgbd_odometry perception_components)
  endif()

  # The keyframe store's admission rule. Nothing consumes the store yet, which is
  # exactly why it needs a test: a threshold that never fires and one that fires on
  # every frame both produce a running pipeline, and only the memory figure would
  # ever say which.
  ament_add_gtest(test_keyframe_store test/test_keyframe_store.cpp)
  if(TARGET test_keyframe_store)
    target_link_libraries(test_keyframe_store perception_components)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(perception_components)
ament_export_dependencies(
  rclcpp rclcpp_components sensor_msgs geometry_msgs nav_msgs pimesh_msgs tf2 tf2_ros OpenCV)

ament_package()
