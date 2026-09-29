cmake_minimum_required(VERSION 3.16)
project(pimesh_backend)

# C++17, not 20. This package builds on the Pi like every other one here, and
# Lyrical's ament_cmake_ros_core raises the dev box to C++20 on its own — so a
# C++20 feature is caught by the Pi's build and by nothing else.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
# P15's solver. **From ROS itself at both ends** — ros-lyrical-libg2o and
# ros-jazzy-libg2o, both 2020.5.29, both exporting these targets under identical
# names (measured 2026-09-23, and a probe built and solved identically on both
# machines 2026-09-29) — so this costs the cross-distro rule nothing, and needs no
# depth_engine_null.cpp-style conditional engine.
#
# **Built with asserts on**, which is the thing to know before calling it: a
# problem with no free pose aborts the process rather than returning an error.
# solve_local_ba() refuses those before g2o sees them.
find_package(g2o REQUIRED)

add_library(backend SHARED
  src/local_ba.cpp
  src/local_mapper.cpp
  src/map.cpp
  src/triangulation.cpp
)
target_include_directories(backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Plain target_link_libraries against exported targets, never
# ament_target_dependencies(): removed in Lyrical, present in Jazzy.
target_link_libraries(backend PUBLIC opencv_core)
# PRIVATE: no header here includes g2o, so nothing downstream needs its targets —
# and a consumer that had to find_package(g2o) to link odometry_node would be one
# more place for the two distros to differ.
target_link_libraries(backend PRIVATE
  g2o::core g2o::stuff g2o::types_sba g2o::types_slam3d g2o::solver_eigen)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS backend EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- Tests ------------------------------------------------------------------
#
# No ROS, no camera, no GPU. Every failure these catch is one a running pipeline
# would not show: a map that never culls has *better* counts, a covisibility
# query that always returns one keyframe is P7's tracker under a new name, and a
# triangulation accepted at one degree of parallax is a point metres out along
# its ray with a reprojection error of a fraction of a pixel.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_triangulation test/test_triangulation.cpp)
  if(TARGET test_triangulation)
    target_link_libraries(test_triangulation backend)
  endif()

  ament_add_gtest(test_map test/test_map.cpp)
  if(TARGET test_map)
    target_link_libraries(test_map backend)
  endif()

  # P15's solver, on synthetic windows with a known answer. The failures it exists
  # for are a BA that converges to a lower cost and a worse geometry, one that
  # never moves anything, and one that aborts the process on a window g2o cannot
  # take — all three of which a running pipeline reports as "fine" or as a crash
  # somewhere else.
  ament_add_gtest(test_local_ba test/test_local_ba.cpp)
  if(TARGET test_local_ba)
    target_link_libraries(test_local_ba backend)
  endif()

  # The backend thread's queue: that it refuses when full rather than dropping,
  # and that a keyframe offered is a keyframe processed.
  ament_add_gtest(test_local_mapper test/test_local_mapper.cpp)
  if(TARGET test_local_mapper)
    target_link_libraries(test_local_mapper backend)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV)

ament_package()
