cmake_minimum_required(VERSION 3.16)
project(pimesh_dataset)

# C++17, like every package here. The dev box raises itself to C++20 through
# Lyrical's ament_ros_defaults whatever this says, so the Pi's build is the only
# thing that catches a C++20 spelling — which is the reason this package builds
# at both ends despite nothing on the Pi ever replaying a dataset.
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
# The calibration loader and the CameraInfo builder, shared rather than copied —
# see package.xml for why this edge is the right way round.
find_package(pimesh_camera REQUIRED)
# imgcodecs alone: the only OpenCV this package uses is one imread at startup, to
# learn the frame size the calibration is checked against.
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

add_library(dataset_components SHARED
  src/dataset_reader.cpp
  src/dataset_node.cpp
)
target_include_directories(dataset_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pimesh_camera_INCLUDE_DIRS}
)
# Plain target_link_libraries against exported targets, never
# ament_target_dependencies(): removed in Lyrical, so that spelling builds on the
# Pi and stops the dev box dead.
target_link_libraries(dataset_components PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${pimesh_msgs_TARGETS}
  # **Not a namespaced target**: pimesh_camera exports with
  # ament_export_libraries() rather than install(EXPORT), so what a consumer gets
  # is a resolved path in ${pimesh_camera_LIBRARIES} and not pimesh_camera::x.
  # Taking the whole library brings CameraNode and V4l2Capture along with the
  # loader, which costs a .so in the container and nothing else — neither has any
  # static initialisation and nothing here constructs them. Splitting the Pi's
  # package in two to make this edge narrower would be the dev box's convenience
  # paid for on the sensor head.
  ${pimesh_camera_LIBRARIES}
  opencv_core
  opencv_imgcodecs
)

rclcpp_components_register_nodes(dataset_components
  "pimesh_dataset::DatasetNode"
)

add_executable(dataset_node apps/dataset_node_main.cpp)
target_link_libraries(dataset_node PRIVATE dataset_components rclcpp::rclcpp)

install(DIRECTORY include/ DESTINATION include)

# The library where the component loader looks, the executable where `ros2 run`
# looks. Two lookups, two destinations.
install(TARGETS dataset_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS dataset_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Tests ------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The index parser and the timestamp arithmetic. Both are pure functions over a
  # path and a string, which is what lets this suite run on a machine with no
  # dataset on it — and the timestamp one is the reason the suite exists at all:
  # `stod(text) * 1e9` is the obvious spelling, it is wrong by a few hundred
  # nanoseconds, and nothing anywhere would ever report it.
  ament_add_gtest(test_dataset_reader test/test_dataset_reader.cpp)
  if(TARGET test_dataset_reader)
    target_link_libraries(test_dataset_reader dataset_components)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(dataset_components)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs pimesh_msgs pimesh_camera OpenCV)

ament_package()
