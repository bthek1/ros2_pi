// The volume, against surfaces whose distance is known by construction.
//
// **Everything this file asserts is a mistake that does not crash.** A sign flip
// on the signed distance, a projection that uses ray length where a depth image
// holds z, a block coordinate that truncates towards zero instead of flooring, a
// colour average that walks a grey wall darker — each of them produces a volume
// that meshes into a room-shaped thing, and not one of them produces an error.
// That is the entire category tools/gates/fusion.sh cannot see, because a gate
// measures a running system and every one of these runs perfectly.

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "opencv2/core.hpp"
#include "pimesh_world/tsdf_volume.hpp"

using pimesh_world::TsdfVolume;

namespace
{

constexpr int kWidth = 640;
constexpr int kHeight = 480;

cv::Matx33d test_k()
{
  return cv::Matx33d(500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0);
}

/// A fronto-parallel plane: every pixel the same z.
///
/// This is the shape that separates z from ray length. A depth image of a plane
/// perpendicular to the optical axis is *constant*, so a corner pixel reads the
/// same 2.000 m as the centre one even though the corner is 2.58 m away along its
/// ray. Any code that confuses the two agrees perfectly at the principal point
/// and is 30% wrong in the corners.
cv::Mat plane_at(float z)
{
  return cv::Mat(kHeight, kWidth, CV_32FC1, cv::Scalar(z));
}

cv::Mat colour_of(int b, int g, int r)
{
  return cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(b, g, r));
}

/// Rotation about the optical frame's y axis (down), i.e. a pan.
cv::Matx33d yaw(double radians)
{
  const double c = std::cos(radians);
  const double s = std::sin(radians);
  return cv::Matx33d(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
}

float centre_ray(const TsdfVolume & volume, const cv::Affine3d & pose)
{
  return volume.raycast_ray(pose, cv::Vec3f(0.0F, 0.0F, 1.0F));
}

TsdfVolume::Options default_options()
{
  TsdfVolume::Options options;
  options.voxel_size_m = 0.015F;
  options.truncation_voxels = 4;
  options.min_weight = 3.0F;
  return options;
}

}  // namespace

// --- The block grid ----------------------------------------------------------

TEST(TsdfBlockKey, RoundTripsThroughNegativeCoordinates)
{
  // **The negative half is the half that matters.** The map frame's origin is
  // wherever the camera started, so it is in the middle of the room and half of
  // every block coordinate is negative. A key that packs without sign-extending
  // puts the room's left half on top of a region 2 million blocks away.
  const std::vector<int> values {0, 1, -1, 7, -7, 1000, -1000, 1048575, -1048576};
  for (const int x : values) {
    for (const int y : values) {
      for (const int z : values) {
        int bx = 0;
        int by = 0;
        int bz = 0;
        TsdfVolume::key_to_block(TsdfVolume::block_key(x, y, z), bx, by, bz);
        EXPECT_EQ(bx, x);
        EXPECT_EQ(by, y);
        EXPECT_EQ(bz, z);
      }
    }
  }
}

TEST(TsdfBlockKey, BlocksEitherSideOfTheOriginAreDifferentBlocks)
{
  // Truncation towards zero — `int(x / size)` — makes block 0 twice as wide as
  // every other block and fuses a slab either side of each axis. The failure is a
  // seam of doubled surface through the middle of the room, which reads as pose
  // drift.
  EXPECT_NE(TsdfVolume::block_key(0, 0, 0), TsdfVolume::block_key(-1, 0, 0));
  EXPECT_NE(TsdfVolume::block_key(0, 0, 0), TsdfVolume::block_key(0, -1, 0));
  EXPECT_NE(TsdfVolume::block_key(0, 0, 0), TsdfVolume::block_key(0, 0, -1));
}

TEST(TsdfVolumeGeometry, VoxelCentresTileTheBlockWithoutGapsOrOverlap)
{
  TsdfVolume volume(default_options());
  const float vs = volume.voxel_size();
  const std::int64_t key = TsdfVolume::block_key(-2, 3, -1);
  // The 512 centres of one block must be 512 distinct points on a regular grid
  // whose extent is exactly one block. An index decomposition that transposes two
  // axes still produces 512 distinct points and a volume that is mirrored.
  for (int index = 0; index < TsdfVolume::kBlockVoxels; ++index) {
    const cv::Vec3f centre = volume.voxel_centre(key, index);
    const int i = index % TsdfVolume::kBlockSide;
    const int j = (index / TsdfVolume::kBlockSide) % TsdfVolume::kBlockSide;
    const int k = index / (TsdfVolume::kBlockSide * TsdfVolume::kBlockSide);
    EXPECT_NEAR(centre[0], -2 * volume.block_size() + (i + 0.5F) * vs, 1e-5);
    EXPECT_NEAR(centre[1], 3 * volume.block_size() + (j + 0.5F) * vs, 1e-5);
    EXPECT_NEAR(centre[2], -1 * volume.block_size() + (k + 0.5F) * vs, 1e-5);
  }
}

// --- Integration and read-back ----------------------------------------------

TEST(TsdfIntegrate, APlaneComesBackOutAtTheDistanceItWentIn)
{
  TsdfVolume volume(default_options());
  const cv::Mat depth = plane_at(2.0F);
  const cv::Affine3d identity = cv::Affine3d::Identity();

  for (int i = 0; i < 4; ++i) {
    volume.integrate(depth, cv::Mat(), test_k(), identity);
  }

  const float hit = centre_ray(volume, identity);
  ASSERT_GT(hit, 0.0F) << "the ray found no surface at all";
  // Within two voxels: the read-back quantises to voxel centres and interpolates
  // the crossing between two march samples.
  EXPECT_NEAR(hit, 2.0F, 2.0F * volume.voxel_size());
}

TEST(TsdfIntegrate, ACornerRayReportsZAndNotRayLength)
{
  // The assertion that separates the two conventions. The corner pixel of this
  // plane is 2.58 m away along its own ray and 2.00 m away in z; the depth image
  // it was integrated from said 2.00, and what comes back must say 2.00 too.
  TsdfVolume volume(default_options());
  const cv::Mat depth = plane_at(2.0F);
  const cv::Affine3d identity = cv::Affine3d::Identity();
  for (int i = 0; i < 4; ++i) {
    volume.integrate(depth, cv::Mat(), test_k(), identity);
  }

  const cv::Matx33d k = test_k();
  const float nx = static_cast<float>((40.0 - k(0, 2)) / k(0, 0));
  const float ny = static_cast<float>((40.0 - k(1, 2)) / k(1, 1));
  const float ray_length = std::sqrt(nx * nx + ny * ny + 1.0F) * 2.0F;
  ASSERT_GT(ray_length, 2.4F) << "this corner is not oblique enough to tell the two apart";

  const float hit = volume.raycast_ray(cv::Affine3d::Identity(), cv::Vec3f(nx, ny, 1.0F));
  ASSERT_GT(hit, 0.0F);
  EXPECT_NEAR(hit, 2.0F, 3.0F * volume.voxel_size());
  EXPECT_LT(hit, 2.2F) << "this is ray length, not z: every wall will come out bowed";
}

TEST(TsdfIntegrate, TheSurfaceIsWhereTheWorldSaysAndNotWhereTheCameraWas)
{
  // Integrate from one pose, read from another. The plane sits at world z = 2;
  // a camera slid sideways still sees it at z = 2, and a camera panned by 20
  // degrees sees it at 2 / cos(20) = 2.128 along its own axis. Both numbers come
  // out of the world transform being applied in the right direction — the wrong
  // direction is an inverse, which for a pure translation is a surface that moves
  // the wrong way and for a rotation is a room that swings twice as far.
  TsdfVolume volume(default_options());
  const cv::Mat depth = plane_at(2.0F);
  for (int i = 0; i < 4; ++i) {
    volume.integrate(depth, cv::Mat(), test_k(), cv::Affine3d::Identity());
  }

  const cv::Affine3d slid(cv::Matx33d::eye(), cv::Vec3d(0.3, 0.0, 0.0));
  const float slid_hit = centre_ray(volume, slid);
  ASSERT_GT(slid_hit, 0.0F) << "the slid camera found nothing — the pose moved the map";
  EXPECT_NEAR(slid_hit, 2.0F, 3.0F * volume.voxel_size());

  const double angle = 20.0 * CV_PI / 180.0;
  const cv::Affine3d panned(yaw(angle), cv::Vec3d(0.0, 0.0, 0.0));
  const float panned_hit = centre_ray(volume, panned);
  ASSERT_GT(panned_hit, 0.0F);
  EXPECT_NEAR(panned_hit, 2.0F / std::cos(angle), 4.0F * volume.voxel_size());
}

TEST(TsdfIntegrate, TheSignedDistanceIsPositiveInFrontOfTheSurface)
{
  // The sign convention, asserted directly rather than through a mesh. Getting it
  // backwards turns every surface inside out: marching cubes still finds the same
  // zero crossing and winds every triangle the other way, so the room renders as
  // a shell lit from inside and nothing about it is obviously wrong.
  TsdfVolume volume(default_options());
  const cv::Mat depth = plane_at(2.0F);
  for (int i = 0; i < 4; ++i) {
    volume.integrate(depth, cv::Mat(), test_k(), cv::Affine3d::Identity());
  }

  float sdf = 0.0F;
  float weight = 0.0F;
  ASSERT_TRUE(volume.sample(cv::Vec3f(0.0F, 0.0F, 1.97F), sdf, weight))
    << "nothing allocated just in front of the surface";
  EXPECT_GT(sdf, 0.0F) << "in front of the wall must be positive";

  ASSERT_TRUE(volume.sample(cv::Vec3f(0.0F, 0.0F, 2.03F), sdf, weight));
  EXPECT_LT(sdf, 0.0F) << "behind the wall must be negative";
}

TEST(TsdfIntegrate, NothingIsWrittenMoreThanATruncationBehindTheSurface)
{
  // A camera has no opinion about what is behind a wall. Writing free space there
  // is what erodes a surface from its far side over a long sweep, and it looks
  // like the mesh thinning rather than like a bug.
  TsdfVolume volume(default_options());
  const cv::Mat depth = plane_at(2.0F);
  for (int i = 0; i < 4; ++i) {
    volume.integrate(depth, cv::Mat(), test_k(), cv::Affine3d::Identity());
  }
  float sdf = 0.0F;
  float weight = 0.0F;
  EXPECT_FALSE(volume.sample(cv::Vec3f(0.0F, 0.0F, 2.0F + 4.0F * volume.truncation_m()),
    sdf, weight));
}

TEST(TsdfIntegrate, TheWeightThresholdKeepsSingleObservationsOutOfTheSurface)
{
  // The noise floor. One frame's flying pixel at an object edge allocates a voxel
  // with weight 1; meshing it is the fringe of debris this exists to exclude. The
  // gap between the two counters is what MeshStats publishes.
  TsdfVolume volume(default_options());
  const cv::Mat depth = plane_at(2.0F);

  volume.integrate(depth, cv::Mat(), test_k(), cv::Affine3d::Identity());
  EXPECT_GT(volume.voxels_allocated(), 0U) << "nothing was allocated at all";
  EXPECT_EQ(volume.voxels_above_weight(), 0U)
    << "one observation must not be enough to mesh a voxel";
  EXPECT_EQ(centre_ray(volume, cv::Affine3d::Identity()), 0.0F)
    << "the ray-caster must not see a surface the mesher would refuse";

  volume.integrate(depth, cv::Mat(), test_k(), cv::Affine3d::Identity());
  volume.integrate(depth, cv::Mat(), test_k(), cv::Affine3d::Identity());
  EXPECT_GT(volume.voxels_above_weight(), 0U);
  EXPECT_LT(volume.voxels_above_weight(), volume.voxels_allocated())
    << "every allocated voxel passing the threshold means the threshold is doing nothing";
}

TEST(TsdfIntegrate, TheRunningAverageSaturatesRatherThanGrowingForever)
{
  TsdfVolume::Options options = default_options();
  options.max_weight = 5.0F;
  TsdfVolume volume(options);
  const cv::Mat depth = plane_at(2.0F);
  for (int i = 0; i < 40; ++i) {
    volume.integrate(depth, cv::Mat(), test_k(), cv::Affine3d::Identity());
  }
  float sdf = 0.0F;
  float weight = 0.0F;
  ASSERT_TRUE(volume.sample(cv::Vec3f(0.0F, 0.0F, 1.99F), sdf, weight));
  EXPECT_FLOAT_EQ(weight, 5.0F)
    << "an unbounded weight is a map that has stopped listening";
}

TEST(TsdfIntegrate, ASurfaceThatMovesInsideTheBandIsFollowed)
{
  // The consequence of the saturating weight, stated as behaviour: forty frames
  // of a wall at 2.00 m followed by forty at 2.04 m must end up reporting 2.04. A
  // volume whose weight grows without bound reports something in between forever.
  TsdfVolume::Options options = default_options();
  options.max_weight = 8.0F;
  TsdfVolume volume(options);
  for (int i = 0; i < 40; ++i) {
    volume.integrate(plane_at(2.00F), cv::Mat(), test_k(), cv::Affine3d::Identity());
  }
  for (int i = 0; i < 40; ++i) {
    volume.integrate(plane_at(2.04F), cv::Mat(), test_k(), cv::Affine3d::Identity());
  }
  const float hit = centre_ray(volume, cv::Affine3d::Identity());
  ASSERT_GT(hit, 0.0F);
  EXPECT_NEAR(hit, 2.04F, 3.0F * volume.voxel_size());
}

TEST(TsdfIntegrate, ASurfaceThatJumpsFurtherThanTheBandLeavesAGhost)
{
  // **A limitation, pinned as a test rather than left to be discovered.** Only
  // the blocks this frame's truncation band names are updated, so a surface that
  // moves further than a truncation is not corrected — it is *joined*, and the
  // old one stands. This is how every voxel-hashing integrator behaves and it is
  // the price of not walking the whole frustum every frame: nothing carves free
  // space between the camera and the new surface.
  //
  // It matters here for one specific reason. The depth model's frame-to-frame
  // wobble is +/-4%, which at 2.5 m is +/-10 cm against a 6 cm truncation — so
  // *unaligned* frames genuinely land outside each other's bands and stack into
  // shingles instead of averaging into a wall. That is the mechanism
  // ScaleAligner exists to remove, and it is why tools/gates/fusion.sh runs the
  // clip both ways rather than trusting that alignment helps.
  TsdfVolume::Options options = default_options();
  options.max_weight = 8.0F;
  TsdfVolume volume(options);
  for (int i = 0; i < 40; ++i) {
    volume.integrate(plane_at(2.0F), cv::Mat(), test_k(), cv::Affine3d::Identity());
  }
  for (int i = 0; i < 40; ++i) {
    volume.integrate(plane_at(2.3F), cv::Mat(), test_k(), cv::Affine3d::Identity());
  }
  const float hit = centre_ray(volume, cv::Affine3d::Identity());
  ASSERT_GT(hit, 0.0F);
  EXPECT_NEAR(hit, 2.0F, 3.0F * volume.voxel_size())
    << "the first surface should still be standing — if this now reports 2.3 the "
       "integrator has started carving free space and the comment above is stale";

  // And the new surface is there too, behind it: both were integrated, neither
  // was erased.
  float sdf = 0.0F;
  float weight = 0.0F;
  EXPECT_TRUE(volume.sample(cv::Vec3f(0.0F, 0.0F, 2.29F), sdf, weight));
  EXPECT_GT(sdf, 0.0F);
}

TEST(TsdfIntegrate, ColourIsAveragedTowardsWhatWasSeenMostOften)
{
  // Two frames of red and eight of blue must come out blue, and the arithmetic
  // must not walk: a running average that truncates instead of rounding loses
  // half a level per frame and a grey wall goes black over a thousand of them.
  TsdfVolume volume(default_options());
  const cv::Mat depth = plane_at(2.0F);
  for (int i = 0; i < 2; ++i) {
    volume.integrate(depth, colour_of(0, 0, 255), test_k(), cv::Affine3d::Identity());
  }
  for (int i = 0; i < 8; ++i) {
    volume.integrate(depth, colour_of(255, 0, 0), test_k(), cv::Affine3d::Identity());
  }

  // Find the voxel closest to the surface straight ahead.
  const cv::Vec3f target(0.0F, 0.0F, 2.0F);
  float best = 1e9F;
  TsdfVolume::Voxel found;
  for (const auto & entry : volume.blocks()) {
    for (int i = 0; i < TsdfVolume::kBlockVoxels; ++i) {
      const auto & voxel = entry.second.voxels[i];
      if (voxel.weight < 1.0F) {continue;}
      const cv::Vec3f centre = volume.voxel_centre(entry.first, i);
      const float d = static_cast<float>(cv::norm(centre - target));
      if (d < best) {
        best = d;
        found = voxel;
      }
    }
  }
  ASSERT_LT(best, 0.05F) << "no voxel near the surface point";
  EXPECT_GT(found.b, found.r) << "eight blue frames must outvote two red ones";
  EXPECT_GT(found.b, 190) << "the running average is losing levels, not converging";

  TsdfVolume grey(default_options());
  for (int i = 0; i < 300; ++i) {
    grey.integrate(depth, colour_of(128, 128, 128), test_k(), cv::Affine3d::Identity());
  }
  const auto & block = grey.blocks().begin()->second;
  for (int i = 0; i < TsdfVolume::kBlockVoxels; ++i) {
    if (block.voxels[i].weight > 0.0F) {
      EXPECT_NEAR(block.voxels[i].b, 128, 1)
        << "300 frames of one colour must not drift the average";
      break;
    }
  }
}

TEST(TsdfIntegrate, NonFiniteAndOutOfRangeDepthIsIgnoredRatherThanIntegrated)
{
  // One NaN folded into a TSDF poisons voxels that were fine, and a reading at
  // the model's far clip is "no idea" rather than a wall six metres away.
  TsdfVolume volume(default_options());
  cv::Mat depth = plane_at(2.0F);
  depth.at<float>(100, 100) = std::numeric_limits<float>::quiet_NaN();
  depth.at<float>(101, 100) = std::numeric_limits<float>::infinity();
  depth.at<float>(102, 100) = -1.0F;
  depth.at<float>(103, 100) = 99.0F;
  for (int i = 0; i < 4; ++i) {
    volume.integrate(depth, cv::Mat(), test_k(), cv::Affine3d::Identity());
  }

  for (const auto & entry : volume.blocks()) {
    for (int i = 0; i < TsdfVolume::kBlockVoxels; ++i) {
      const auto & voxel = entry.second.voxels[i];
      ASSERT_TRUE(std::isfinite(voxel.sdf)) << "a NaN reached the volume";
      ASSERT_TRUE(std::isfinite(voxel.weight));
    }
  }
  const float hit = centre_ray(volume, cv::Affine3d::Identity());
  EXPECT_NEAR(hit, 2.0F, 3.0F * volume.voxel_size());
}

TEST(TsdfIntegrate, AnEmptyOrWrongTypeDepthMapIsRefusedRatherThanGuessedAt)
{
  TsdfVolume volume(default_options());
  EXPECT_EQ(
    volume.integrate(cv::Mat(), cv::Mat(), test_k(), cv::Affine3d::Identity()).voxels_updated,
    0U);
  cv::Mat wrong(kHeight, kWidth, CV_16UC1, cv::Scalar(2000));
  EXPECT_EQ(
    volume.integrate(wrong, cv::Mat(), test_k(), cv::Affine3d::Identity()).voxels_updated, 0U);
  EXPECT_EQ(volume.block_count(), 0U);
}

TEST(TsdfIntegrate, ABlockNothingWasWrittenIntoIsNotAllocated)
{
  // The allocation pass is deliberately generous — it samples a band around a ray
  // and rounds outwards — so some of what it names turns out to have no valid
  // depth behind it. Inserting those anyway would inflate voxels_allocated, which
  // MeshStats publishes and a person reads as "how much room have I seen".
  //
  // **The scene is a sparse depth map, because a dense one does not exercise
  // this at all.** Measured while writing the test: a full plane, a depth step
  // and a slanted wall each filter *nothing* — every block the band names has a
  // voxel projecting onto a valid pixel. A map valid only on scattered pixels
  // filters 171 of 276, and that is the shape real monocular depth has wherever
  // the far clip has taken out most of a region.
  TsdfVolume volume(default_options());
  cv::Mat sparse = cv::Mat::zeros(kHeight, kWidth, CV_32FC1);
  for (int v = 0; v < kHeight; v += 40) {
    for (int u = 0; u < kWidth; u += 40) {
      sparse.at<float>(v, u) = 2.0F;
    }
  }

  const auto result = volume.integrate(sparse, cv::Mat(), test_k(), cv::Affine3d::Identity());

  EXPECT_EQ(volume.block_count(), result.blocks_new)
    << "the accounting disagrees with the map itself";
  EXPECT_LT(result.blocks_new, result.blocks_touched)
    << "every block the allocation pass named got written, so nothing is being "
       "filtered and voxels_allocated is an over-count";
}

TEST(TsdfIntegrate, TheCeilingStopsNewBlocksAndSaysSoRatherThanStoppingQuietly)
{
  // A map that has quietly stopped growing and a camera pointed at a wall it has
  // already mapped look identical from outside. So past the ceiling: existing
  // blocks keep updating — the mapped parts of the room stay live — nothing new
  // is taken on, and the refusal is counted so the node can warn.
  TsdfVolume::Options options = default_options();
  options.max_blocks = 40;
  TsdfVolume volume(options);
  const cv::Mat depth = plane_at(2.0F);

  const auto first = volume.integrate(depth, cv::Mat(), test_k(), cv::Affine3d::Identity());
  EXPECT_LE(volume.block_count(), options.max_blocks);
  EXPECT_GT(first.blocks_refused, 0U) << "a 40-block ceiling was not reached by a whole plane";

  // And the part that *is* mapped goes on being refined: weight climbs past the
  // threshold on the blocks that made it in, so a surface still appears.
  for (int i = 0; i < 4; ++i) {
    volume.integrate(depth, cv::Mat(), test_k(), cv::Affine3d::Identity());
  }
  EXPECT_EQ(volume.block_count(), options.max_blocks);
  EXPECT_GT(volume.voxels_above_weight(), 0U)
    << "a full volume stopped updating what it already holds, not just what it does not";
}

TEST(TsdfVolumeReset, ClearThrowsTheMapAwayCompletely)
{
  TsdfVolume volume(default_options());
  for (int i = 0; i < 4; ++i) {
    volume.integrate(plane_at(2.0F), cv::Mat(), test_k(), cv::Affine3d::Identity());
  }
  ASSERT_GT(volume.block_count(), 0U);
  volume.clear();
  EXPECT_EQ(volume.block_count(), 0U);
  EXPECT_EQ(volume.voxels_allocated(), 0U);
  EXPECT_EQ(centre_ray(volume, cv::Affine3d::Identity()), 0.0F);
}

TEST(TsdfRaycast, AnImageOfAPlaneComesBackFlat)
{
  // The whole-image entry point, which is what the aligner measures against. A
  // plane must come back as a plane: a ray-caster that steps past the crossing in
  // some directions and not others produces a surface with holes in it, and the
  // aligner's median then runs on whichever pixels survived.
  TsdfVolume volume(default_options());
  for (int i = 0; i < 4; ++i) {
    volume.integrate(plane_at(2.0F), cv::Mat(), test_k(), cv::Affine3d::Identity());
  }

  const cv::Size small(kWidth / 16, kHeight / 16);
  const double sx = static_cast<double>(small.width) / kWidth;
  const double sy = static_cast<double>(small.height) / kHeight;
  const cv::Matx33d k = test_k();
  const cv::Matx33d k_small(
    k(0, 0) * sx, 0.0, k(0, 2) * sx, 0.0, k(1, 1) * sy, k(1, 2) * sy, 0.0, 0.0, 1.0);

  cv::Mat expected;
  volume.raycast(k_small, cv::Affine3d::Identity(), small, expected);
  ASSERT_EQ(expected.type(), CV_32FC1);
  ASSERT_EQ(expected.size(), small);

  int hits = 0;
  double worst = 0.0;
  for (int v = 0; v < expected.rows; ++v) {
    for (int u = 0; u < expected.cols; ++u) {
      const float value = expected.at<float>(v, u);
      if (value <= 0.0F) {continue;}
      ++hits;
      worst = std::max(worst, std::fabs(static_cast<double>(value) - 2.0));
    }
  }
  EXPECT_GT(hits, expected.total() * 0.9)
    << "the ray-cast is full of holes: " << hits << " of " << expected.total();
  EXPECT_LT(worst, 4.0 * volume.voxel_size())
    << "the plane came back bent by " << worst << " m";
}

TEST(TsdfRaycast, AnEmptyVolumeReportsNothingRatherThanZeroDistance)
{
  // 0 means "no surface", and the aligner treats it as invalid. The failure this
  // guards is a ray-caster that returns its near plane when it finds nothing: the
  // aligner would then compute a ratio against 0.15 m everywhere and scale the
  // whole frame into the camera.
  TsdfVolume volume(default_options());
  cv::Mat expected;
  volume.raycast(test_k(), cv::Affine3d::Identity(), cv::Size(16, 12), expected);
  ASSERT_EQ(expected.size(), cv::Size(16, 12));
  EXPECT_EQ(cv::countNonZero(expected), 0);
}
