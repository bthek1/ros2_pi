cmake_minimum_required(VERSION 3.16)
project(pimesh_instruments)

find_package(ament_cmake REQUIRED)

# Nothing is built or installed here. This package is a test harness for the
# scripts in tools/, which are not a package and cannot have tests of their own —
# colcon only runs tests that live inside one.
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # P9's straightness instrument, checked against a synthetic distortion.
  # tools/gates/calibration.sh prints a number in pixels and asserts on it; this
  # is what makes that number worth asserting on. It needs no camera, no printed
  # board and no saved frames — a chessboard is projected through a known K and D
  # with cv2.projectPoints and the measurement has to recover them — so it was
  # finished before the physical session and is what caught the transposed-grid
  # and near-vertical-line failures there.
  ament_add_pytest_test(test_straightness test/test_straightness.py)

  # The mesh gate's instrument. tools/gates/mesh.sh closes P6 by pointing at three
  # PNGs and asserting what fraction of each is surface; a renderer that ignored
  # its azimuth would write three copies of one picture and satisfy that check
  # three times over.
  ament_add_pytest_test(test_mesh_render test/test_mesh_render.py)

  # P3's outside opinion. gates/keypoints.sh asserts that keypoint_node's matched
  # fraction is within 5 points of this reimplementation of the predecessor's
  # algorithm — so the reimplementation decides the gate. Two of the ways it can
  # be wrong move the number *up* and make the gate agree better: many-to-one
  # matching, and scoring a featureless frame as zero.
  ament_add_pytest_test(test_orb_reference test/test_orb_reference.py)
endif()

ament_package()
