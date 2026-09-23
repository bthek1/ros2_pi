cmake_minimum_required(VERSION 3.16)
project(pimesh_dashboard)

# C++17, not 20. Nothing in this package runs on the Pi, but tools/gates/build.sh
# builds the whole workspace at both ends and a C++20 feature here stops that
# build — the dev box compiles at -std=c++20 regardless because Lyrical's
# ament_cmake_ros_core raises it, so the Pi is the only place this is enforced.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pimesh_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# --- The protocol, with no ROS in it ----------------------------------------
#
# A library of its own so the tests can reach it without a ROS graph, a socket or
# a browser. Everything in it is a pure function over bytes, which is what makes
# `test_websocket` able to pin a handshake against RFC 6455's own example.
add_library(dashboard_protocol STATIC
  src/websocket.cpp
  src/web_server.cpp
)
target_include_directories(dashboard_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_property(TARGET dashboard_protocol PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(dashboard_components SHARED src/dashboard_node.cpp)
target_include_directories(dashboard_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Plain target_link_libraries against exported targets, never
# ament_target_dependencies(): removed in Lyrical, present in Jazzy, so the macro
# builds on the Pi and stops the dev box with "Unknown CMake command".
target_link_libraries(dashboard_components PUBLIC
  dashboard_protocol
  rclcpp::rclcpp
  rclcpp_components::component
  ament_index_cpp::ament_index_cpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${pimesh_msgs_TARGETS}
)

# Plural, so no main is generated: the thin one is written by hand in src/.
rclcpp_components_register_nodes(dashboard_components "pimesh_dashboard::DashboardNode")

add_executable(dashboard_node apps/dashboard_node_main.cpp)
target_link_libraries(dashboard_node PRIVATE dashboard_components rclcpp::rclcpp)

# The instrument tools/gates/dashboard.sh measures with. No ROS in it: it is a
# TCP client, and a ROS node here would put a participant on the domain that the
# measurement would then include. See the file's header for why it is not a real
# headless browser.
add_executable(dashboard_probe apps/dashboard_probe.cpp)
target_link_libraries(dashboard_probe PRIVATE dashboard_protocol)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS dashboard_protocol dashboard_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS dashboard_node dashboard_probe DESTINATION lib/${PROJECT_NAME})

# The page and its assets, vendored and versioned in-repo. No CDN: the LAN may
# have no internet, and a monitoring tool that breaks when DNS does is not one.
install(DIRECTORY web/ DESTINATION share/${PROJECT_NAME}/web)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The wire format. Every failure in here is silent by construction: a wrong
  # accept key is a browser that closes the connection with no error, a wrong
  # mask is a message that arrives as mojibake, and a length written in the wrong
  # width is a frame boundary quietly in the wrong place. The SHA-1 is pinned
  # against the published vectors for the reason FNV-1a had to be — a hash with a
  # wrong constant behaves perfectly and is still the wrong hash.
  ament_add_gtest(test_websocket test/test_websocket.cpp)
  if(TARGET test_websocket)
    target_link_libraries(test_websocket dashboard_protocol)
  endif()

  # Path resolution and content types, which is the half of an HTTP server that
  # can be wrong without anything crashing. `GET /../../../../etc/passwd` is a
  # request no browser sends and curl --path-as-is does, and this page is bound
  # to the LAN on purpose.
  ament_add_gtest(test_web_paths test/test_web_paths.cpp)
  if(TARGET test_web_paths)
    target_link_libraries(test_web_paths dashboard_protocol)
  endif()

  # The two functions every byte of the page is built out of, which lived in an
  # anonymous namespace inside dashboard_node.cpp until 2026-09-21 where nothing
  # could call them. Both fail by *blanking the page*: JSON.parse throws on the
  # first illegal byte, onStats never runs, and the table holds its last value
  # with the socket up and the pipeline fine — which reads as a stalled pipeline.
  # Header-only and ROS-free, so it links against nothing.
  ament_add_gtest(test_json test/test_json.cpp)
  if(TARGET test_json)
    target_include_directories(test_json PRIVATE include)
  endif()

  # The binary mesh layout, pinned byte for byte. Its reader is hand-written
  # JavaScript in web/app.js and cannot be linked against, so the two ends agree
  # only because somebody typed 4, 12 and 3 twice — and a stride read one byte
  # out still yields floats, so the page draws believable geometry in the wrong
  # places rather than failing.
  ament_add_gtest(test_mesh_payload test/test_mesh_payload.cpp)
  if(TARGET test_mesh_payload)
    target_include_directories(test_mesh_payload PRIVATE include)
    target_link_libraries(
      test_mesh_payload ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})
  endif()

  # The other half of that contract, and of the JSON one: what web/app.js and
  # dashboard_probe.cpp actually read, checked against what dashboard_node.cpp
  # actually sends. It reads both files as text because the reader is in a
  # browser — inelegant, and the only thing that can check it at all.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_dashboard_contract test/test_dashboard_contract.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(dashboard_protocol dashboard_components)
ament_export_dependencies(
  rclcpp rclcpp_components ament_index_cpp sensor_msgs nav_msgs visualization_msgs
  geometry_msgs std_msgs pimesh_msgs)

ament_package()
