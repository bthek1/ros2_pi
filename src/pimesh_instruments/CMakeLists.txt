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

  # #13's P19. gates/lost.sh asserts how many depth frames LOST and OK took against
  # an injected blackout, and this is what counts them. Three of its failure modes
  # flatter the tracker: UNKNOWN read as OK, an earlier LOST credited to the
  # blackout, and stamps compared as doubles past 2^53.
  ament_add_pytest_test(test_lost_timeline test/test_lost_timeline.py)

  # #13's P20. gates/relocalise.sh asserts how far a relocalised pose is from motion
  # capture, through a Sim(3) fitted on the saved map's keyframes. Left in the map's
  # own units the error would read at half its size on fr1/desk, and a relocalised
  # pose let into its own alignment has nothing to disagree with.
  ament_add_pytest_test(test_reloc_truth test/test_reloc_truth.py)

  # P16's judge. gates/place.sh closes the phase on the precision this computes
  # against motion capture, and every way it can be wrong flatters the detector:
  # an unjudged closure counted true, a relative pose compared inverted, a
  # precision of 1.000 printed over no closures at all.
  ament_add_pytest_test(test_place_truth test/test_place_truth.py)
endif()

ament_package()
