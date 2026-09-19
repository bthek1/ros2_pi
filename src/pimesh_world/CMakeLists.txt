cmake_minimum_required(VERSION 3.16)
project(pimesh_world)

# C++17, not 20. Nothing in this package runs on the Pi and tools/gates/build.sh
# builds it there anyway, so a C++20 feature here stops a build on the machine
# least able to afford a five-minute compile. Note that this setting is *raised*
# on the dev box — Lyrical's ament_cmake_ros_core exports an INTERFACE target
# requiring cxx_std_20 — so the Pi's build is the only thing that enforces it.
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
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pimesh_msgs REQUIRED)
find_package(pimesh_perception REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(world_components SHARED
  src/fusion_node.cpp
  src/marching_cubes.cpp
  src/mesh_cleanup.cpp
  src/mesh_node.cpp
  src/mesh_io.cpp
  src/scale_aligner.cpp
  src/shared_volume.cpp
  src/tsdf_volume.cpp
)
target_include_directories(world_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# pimesh_perception exports include directories rather than a CMake target, and
# what this package takes from it is header-only — Mailbox, percentile, and the
# cv::Mat view over a bgr8 message. So this is an include path and not a link.
target_include_directories(world_components PUBLIC ${pimesh_perception_INCLUDE_DIRS})

# Plain target_link_libraries against exported targets, never
# ament_target_dependencies(): removed in Lyrical, present in Jazzy, so the macro
# builds on the Pi and stops the dev box with "Unknown CMake command".
target_link_libraries(world_components PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${pimesh_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  opencv_core
  opencv_imgproc
)

# Plural, so no main is generated: the thin ones are written by hand in src/.
rclcpp_components_register_nodes(world_components
  "pimesh_world::FusionNode"
  "pimesh_world::MeshNode"
)

foreach(node fusion_node mesh_node)
  add_executable(${node} src/${node}_main.cpp)
  target_link_libraries(${node} PRIVATE world_components rclcpp::rclcpp)
endforeach()

install(DIRECTORY include/ DESTINATION include)

# The library where the component loader looks, the executables where `ros2 run`
# looks. Two lookups, two destinations.
install(TARGETS world_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS fusion_node mesh_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Tests ------------------------------------------------------------------
#
# Everything here is geometry and statistics with no ROS in it, which is the
# whole reason TsdfVolume and ScaleAligner are classes rather than code inside a
# node. Every mistake available in them is silent: a sign flip on the signed
# distance, a projection that uses ray length where the depth image holds z, a
# median that is quietly the mean. Each produces a volume that meshes into a
# plausible room and is wrong, and none of them crashes.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_tsdf_volume test/test_tsdf_volume.cpp)
  if(TARGET test_tsdf_volume)
    target_link_libraries(test_tsdf_volume world_components)
  endif()

  ament_add_gtest(test_scale_aligner test/test_scale_aligner.cpp)
  if(TARGET test_scale_aligner)
    target_link_libraries(test_scale_aligner world_components)
  endif()

  # The 256-entry lookup table, checked the only way copied constant data can be:
  # against a property the whole of it has to satisfy at once. A sphere fully
  # inside the known region has no boundary edges, and one wrong case leaves one.
  ament_add_gtest(test_marching_cubes test/test_marching_cubes.cpp)
  if(TARGET test_marching_cubes)
    target_link_libraries(test_marching_cubes world_components)
  endif()

  # The cleanup policy and the triangle budget. Every failure available in them is
  # a mesh that renders: a filled frontier is a sealed box, which looks *more*
  # finished than a correct scan, and decimation that shrinks a surface in from
  # its edges looks like walls receding.
  ament_add_gtest(test_mesh_cleanup test/test_mesh_cleanup.cpp)
  if(TARGET test_mesh_cleanup)
    target_link_libraries(test_mesh_cleanup world_components)
  endif()

  # The rendezvous between the two nodes, and the chunked copy between them.
  # Every failure available here produces a mesh: a registry that hands out a
  # fresh volume per lookup leaves the mesher on an empty one forever, and an
  # empty volume is what an unswept room looks like.
  ament_add_gtest(test_shared_volume test/test_shared_volume.cpp)
  if(TARGET test_shared_volume)
    target_link_libraries(test_shared_volume world_components)
  endif()

  # The saved surface, read back both ways. P6's evidence is a triangle count
  # taken off a PLY file through read_ply, so writer and reader being wrong
  # *together* is invisible to the gate and to a round trip alike — this suite
  # also reads the bytes against the layout the header promises.
  ament_add_gtest(test_mesh_io test/test_mesh_io.cpp)
  if(TARGET test_mesh_io)
    target_link_libraries(test_mesh_io world_components)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(world_components)
ament_export_dependencies(
  rclcpp rclcpp_components sensor_msgs geometry_msgs std_srvs visualization_msgs
  pimesh_msgs pimesh_perception tf2 tf2_ros OpenCV)

ament_package()
