cmake_minimum_required(VERSION 3.16)
project(pimesh_bringup)

find_package(ament_cmake REQUIRED)

# Launch and config only — no compiled targets here on purpose. Everything a
# session needs to be reproducible lives in these two directories.
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
