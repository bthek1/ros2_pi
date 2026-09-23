cmake_minimum_required(VERSION 3.16)
project(pimesh_core)

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
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

# **No library target, and that is the point.** Everything here is a header, so
# there is nothing to link and nothing for a consumer to get wrong: a package
# that finds this one gets ${pimesh_core_INCLUDE_DIRS} and that is the whole
# contract.
install(DIRECTORY include/ DESTINATION include)

# --- Tests ------------------------------------------------------------------
#
# These three suites are the reason this package is worth existing rather than
# being a directory. Every one of them covers a helper that spent months in an
# anonymous namespace inside a translation unit with a node in it, where no test
# could call it however much anyone wanted to — and the first run of test_stats
# found the FNV-1a offset basis had been wrong since milestone A, in both copies,
# without anything ever being wrong.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The percentile and the hash every gate's number comes out of. A percentile
  # that is quietly the maximum reports a plausible figure forever.
  ament_add_gtest(test_stats test/test_stats.cpp)
  if(TARGET test_stats)
    target_include_directories(test_stats PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  # Newest-wins and its drop accounting. A mailbox that silently keeps the
  # oldest frame is a pipeline that works and lags.
  ament_add_gtest(test_mailbox test/test_mailbox.cpp)
  if(TARGET test_mailbox)
    target_include_directories(test_mailbox PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  # The bgr8 layout arithmetic, that a cv::Mat over a message shares its memory,
  # and the 32FC1 twin every distance the TSDF integrates passes through. It also
  # pins the *pair*: the two disagree about step == 0, which is defensible and is
  # asserted as a pair so that making them consistent is a decision somebody takes
  # rather than a tidy-up.
  ament_add_gtest(test_image_buffer test/test_image_buffer.cpp)
  if(TARGET test_image_buffer)
    target_include_directories(test_image_buffer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_image_buffer ${sensor_msgs_TARGETS} opencv_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(sensor_msgs OpenCV)

ament_package()
