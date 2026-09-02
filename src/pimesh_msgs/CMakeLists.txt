cmake_minimum_required(VERSION 3.16)
project(pimesh_msgs)

# C++17, not C++20: this package builds under Jazzy on the Pi as well as
# Lyrical here, and Jazzy's baseline is 17. See CLAUDE.md on the ABI split.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Keypoints.msg
  msg/MeshStats.msg
  msg/PipelineStats.msg
  srv/ResetMap.srv
  srv/SaveMesh.srv
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
