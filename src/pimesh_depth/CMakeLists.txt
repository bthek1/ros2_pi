cmake_minimum_required(VERSION 3.16)
project(pimesh_depth)

# C++17, not 20, even though the inference in this package only ever runs on the
# dev box. tools/gates/build.sh builds the whole workspace at both ends, so a
# C++20 feature here stops the Pi's build — and Lyrical raises this box to C++20
# on its own, which means the Pi is the only machine that would ever notice.
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
find_package(pimesh_msgs REQUIRED)
# The mailbox this node drops two frames in three through, the cv::Mat header it
# lays over an image without copying it, and the percentile its stats line
# reports. Header-only: there is no library to link, only include directories.
find_package(pimesh_core REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# --- ONNX Runtime, and the whole cross-machine story in one if() -------------
#
# Dev box only, and *optional*, which is the load-bearing word. The Pi builds this
# entire workspace — tools/gates/build.sh builds it at both ends — and the GPU
# stack is x86-64 CUDA that will never exist there. So this is a find, not a
# REQUIRED find, and what changes with it is only which DepthEngine gets compiled:
# depth_node itself, its registration and its parameters build identically on both
# machines. See include/pimesh_depth/depth_engine.hpp for why that matters —
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
  message(STATUS "pimesh_depth: ONNX Runtime at ${PIMESH_GPU_PREFIX} — depth_node will infer")
else()
  set(PIMESH_HAVE_ONNXRUNTIME FALSE)
  message(STATUS
    "pimesh_depth: no ONNX Runtime — depth_node builds but refuses to run. "
    "This is expected on the Pi; on the dev box run bash tools/fetch-gpu-stack.sh")
endif()

add_library(depth_components SHARED
  src/depth_node.cpp
  probes/depth_probe.cpp
  probes/scale_probe.cpp
  # Exactly one of these two is compiled, and the choice is the only thing the
  # GPU stack's presence changes about this library.
  $<$<BOOL:${PIMESH_HAVE_ONNXRUNTIME}>:src/depth_engine_ort.cpp>
  $<$<NOT:$<BOOL:${PIMESH_HAVE_ONNXRUNTIME}>>:src/depth_engine_null.cpp>
)
target_include_directories(depth_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pimesh_core_INCLUDE_DIRS}
)
# Plain target_link_libraries against exported targets, never
# ament_target_dependencies(): removed in Lyrical, present in Jazzy, so the macro
# builds on the Pi and stops the dev box with "Unknown CMake command".
target_link_libraries(depth_components PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${pimesh_msgs_TARGETS}
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
)

# --- The depth build, and the linker flag that decides whether it uses the GPU --
#
# The default model path is baked in here rather than computed at runtime.
# CMAKE_CURRENT_SOURCE_DIR is <ws>/src/pimesh_depth, so two `..` reach the
# workspace; the alternative — walking up from get_package_share_directory() at
# runtime — is four segments of assumption about a layout --symlink-install
# already bends. models/ is git-ignored and 99 MB, so it is read from the source
# tree and never installed into share/.
get_filename_component(PIMESH_WS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
target_compile_definitions(depth_components PRIVATE
  PIMESH_DEFAULT_MODEL_PATH="${PIMESH_WS_DIR}/models/depth_anything_v2_small.onnx")

if(PIMESH_HAVE_ONNXRUNTIME)
  target_include_directories(depth_components PRIVATE ${ONNXRUNTIME_INCLUDE})
  target_link_libraries(depth_components PRIVATE ${ONNXRUNTIME_LIB} ${CMAKE_DL_LIBS})

  # The prefix, baked in, so depth_engine_ort.cpp can dlopen the CUDA provider by
  # absolute path before ONNX Runtime tries to. See that file for why that is
  # load-bearing and not belt-and-braces: the linker flag below gets the provider's
  # *dependencies* found only when the RPATH in the dlopen chain belongs to
  # something that has one, and in a component container it does not.
  target_compile_definitions(depth_components PRIVATE
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
  set_target_properties(depth_components PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${PIMESH_GPU_PREFIX}/lib")
  target_link_options(depth_components PRIVATE
    "-Wl,--disable-new-dtags"
    "-Wl,-rpath,${PIMESH_GPU_PREFIX}/lib")
endif()

# Plural, so no main is generated: the thin one is written by hand in src/.
rclcpp_components_register_nodes(depth_components
  "pimesh_depth::DepthNode"
  "pimesh_depth::DepthProbe"
  "pimesh_depth::ScaleProbe"
)

add_executable(depth_node apps/depth_node_main.cpp)
target_link_libraries(depth_node PRIVATE depth_components rclcpp::rclcpp)

install(DIRECTORY include/ DESTINATION include)

# The library where the component loader looks, the executable where `ros2 run`
# looks. Two lookups, two destinations.
install(TARGETS depth_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS depth_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Tests ------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The arithmetic either side of the network — the whole of P4 that can be got
  # wrong in silence. No ONNX Runtime, no GPU, no camera, so this suite runs
  # identically on both machines, which is the reason depth_model.hpp is a
  # separate header from the engine behind it. A channel order swapped, planes
  # interleaved instead of planar, or a reciprocal taken before the clamp each
  # produce a depth map that renders as a plausible room and is numerically
  # nonsense. It also covers the colour preview's mapping, where one sign decides
  # whether near is bright or the 6 m clip is.
  ament_add_gtest(test_depth_model test/test_depth_model.cpp)
  if(TARGET test_depth_model)
    target_include_directories(test_depth_model PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include ${pimesh_core_INCLUDE_DIRS})
    target_link_libraries(test_depth_model opencv_core opencv_imgproc)
  endif()

  # P12's instrument. tools/gates/scale.sh divides a tape measure by the number
  # this returns and calls the result `depth_scale` — the constant under every
  # distance this project reports — and there is no second opinion on it
  # anywhere. Every way of getting it wrong returns a plausible distance: a patch
  # that is not centred, a NaN in the sort, and above all far-clip values counted
  # as distances, which makes the implied scale come out too small with nothing
  # saying so. Synthetic cv::Mats, no model and no GPU, so it runs on the Pi.
  ament_add_gtest(test_depth_patch test/test_depth_patch.cpp)
  if(TARGET test_depth_patch)
    target_include_directories(test_depth_patch PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include ${pimesh_core_INCLUDE_DIRS})
    target_link_libraries(test_depth_patch opencv_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(depth_components)
ament_export_dependencies(
  rclcpp rclcpp_components sensor_msgs pimesh_msgs pimesh_core OpenCV)

ament_package()
