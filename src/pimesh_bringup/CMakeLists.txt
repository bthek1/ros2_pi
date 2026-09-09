cmake_minimum_required(VERSION 3.16)
project(pimesh_bringup)

find_package(ament_cmake REQUIRED)

# There is no target here and there is not meant to be. This package is launch
# files, parameter YAML and RViz configs — the things that describe how the
# workspace is assembled, which is a different kind of fact from the code that
# gets assembled. Keeping them out of the node packages is what stops a
# parameter default and a launch default drifting apart in two files.
#
# share/${PROJECT_NAME} is where FindPackageShare() and `ros2 launch <pkg>
# <file>` look. Omitting an install() here does not fail the build; it fails the
# launch, later, with "not found in the share directory".
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
