cmake_minimum_required(VERSION 3.16)
project(pimesh_hello)

# C++17, stated outright rather than inherited.
#
# Not 20: the Pi builds this exact file under Jazzy, whose baseline is 17, and a
# feature that compiles here and not there is only discovered at the far end of
# an rsync. STANDARD_REQUIRED turns a missing standard into a configure error
# instead of a silent downgrade; EXTENSIONS OFF asks for -std=c++17 rather than
# -std=gnu++17, so a GNU extension cannot creep in unnoticed.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# The component library. SHARED is not a style choice: a container loads this
# with dlopen at runtime, so there has to be a .so to open.
add_library(hello_components SHARED
  src/hello_node.cpp
  src/echo_node.cpp
)
target_include_directories(hello_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Modern CMake targets, not ament_target_dependencies().
#
# That macro was deprecated in Jazzy and **removed in Lyrical** — the dev box
# fails with `Unknown CMake command`, and the Pi would have built the same file
# happily, which is the worst possible way to find out. Both distros export
# these targets, so this is the form that is correct on both. `std_msgs::std_msgs`
# is an aggregate over the generated typesupport libraries.
target_link_libraries(hello_components PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  std_msgs::std_msgs
)

# Writes the class name into the ament index, which is how a container finds a
# component it was only ever told the name of. The plural form registers without
# generating a main — this package writes its own, in hello_node_main.cpp,
# because a hand-written entry point is the thing P1 is demonstrating.
rclcpp_components_register_nodes(hello_components
  "pimesh_hello::HelloNode"
  "pimesh_hello::EchoNode"
)

# The standalone binaries: `ros2 run pimesh_hello hello_node` / `echo_node`.
# Every component gets one. A node that only works inside a container is as
# awkward to debug as one that only works outside it.
foreach(node hello_node echo_node)
  add_executable(${node} src/${node}_main.cpp)
  target_link_libraries(${node} PRIVATE hello_components rclcpp::rclcpp)
endforeach()

install(DIRECTORY include/ DESTINATION include)

# The library goes to lib/ where the loader looks; the executable goes to
# lib/${PROJECT_NAME}/ where `ros2 run` looks. Two different destinations for
# two different lookups, and swapping them makes exactly one of them fail.
install(TARGETS hello_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS hello_node echo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files and parameter YAML go to share/, which is where
# FindPackageShare() and `ros2 launch <pkg> <file>` look. Omitting this does not
# fail the build — it fails the launch, later, with "not found in the share
# directory".
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(hello_components)
ament_export_dependencies(rclcpp rclcpp_components std_msgs)

ament_package()
