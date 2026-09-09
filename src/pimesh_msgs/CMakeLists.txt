cmake_minimum_required(VERSION 3.16)
project(pimesh_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# One call, every interface. The generator writes C, C++, Python and the
# typesupport libraries for whichever RMW this machine has — which is the whole
# reason the Pi compiles this package instead of receiving it. A .msg is
# portable; the .so the generator makes out of it is not.
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Keypoints.msg
  msg/MeshStats.msg
  msg/PipelineStats.msg
  srv/ResetMap.srv
  srv/SaveMesh.srv
  DEPENDENCIES std_msgs geometry_msgs
)

# Without this a downstream package finds the headers and fails to link the
# typesupport. rosidl_generate_interfaces does not imply it.
ament_export_dependencies(rosidl_default_runtime)

ament_package()
