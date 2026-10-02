// P18's instrument (#12): a rebuild scored against motion capture.
//
// The instrument decides tools/gates/rebuild.sh, and the reason it exists is that
// the first one could not tell a right surface from a wrong one. So the property
// that matters most is pinned directly: over the *same* frames, an arm whose poses
// carry drift must score worse against ground truth than an arm whose poses do not —
// with the ground truth in a different frame and at a different scale, so the
// alignment has real work to do. And Umeyama is pinned against a known similarity,
// because an alignment that is wrong is a reference surface in the wrong place.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <random>
#include <vector>

#include <unistd.h>

#include "pimesh_mapping/ground_truth.hpp"
#include "pimesh_mapping/rebuild.hpp"

using pimesh_mapping::ArmScore;
using pimesh_mapping::Corrections;
using pimesh_mapping::FrameMemory;
using pimesh_mapping::RememberedFrame;
using pimesh_mapping::Sim3;
using pimesh_mapping::TsdfVolume;
using pimesh_mapping::apply;
using pimesh_mapping::groundtruth_at;
using pimesh_mapping::score_arm;
using pimesh_mapping::umeyama;

namespace
{

constexpr std::int64_t kSecond = 1000000000LL;

/// Axis-angle to a rotation, through cv::Affine3d's own conversion — core, not
/// calib3d, which this package does not link.
cv::Matx33d rot(const cv::Vec3d & axis_angle)
{
  return cv::Affine3d(axis_angle, cv::Vec3d(0, 0, 0)).rotation();
}

double rotation_deg(const cv::Matx33d & r)
{
  const double c = std::max(-1.0, std::min(1.0, (cv::trace(r) - 1.0) / 2.0));
  return std::acos(c) * 180.0 / CV_PI;
}

}  // namespace

// --- Umeyama ------------------------------------------------------------------------

TEST(Umeyama, RecoversAKnownSimilarityExactly)
{
  std::mt19937 gen(3);
  std::uniform_real_distribution<double> u(-2.0, 2.0);
  const double s = 0.5;
  const cv::Matx33d r = rot(cv::Vec3d(0.3, -0.8, 0.2));
  const cv::Vec3d t(1.0, -2.0, 0.5);
  std::vector<cv::Vec3d> from, to;
  for (int i = 0; i < 20; ++i) {
    from.emplace_back(u(gen), u(gen), u(gen));
    to.push_back(s * (r * from.back()) + t);
  }
  const Sim3 fit = umeyama(from, to);
  ASSERT_TRUE(fit.ok);
  EXPECT_NEAR(fit.s, s, 1e-9);
  // The matrices, not an angle between them: acos near 1 cannot resolve much below
  // sqrt(2e-16) rad, ~8e-7 degrees, and the first version's 1e-6-degree bound sat on
  // that floor — the Pi's aarch64 rounding landed at 1.2e-6 and failed a correct fit.
  EXPECT_LT(cv::norm(cv::Mat(fit.r - r)), 1e-9);
  EXPECT_LT(cv::norm(fit.t - t), 1e-9);
}

TEST(Umeyama, NeverReturnsAReflection)
{
  // A mirrored point set is fitted best by a reflection, which is not a rotation. The
  // guard flips the smallest singular direction instead; the result must have
  // determinant +1 whatever the fit costs.
  std::vector<cv::Vec3d> from{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
  std::vector<cv::Vec3d> to;
  for (const auto & p : from) {to.emplace_back(-p[0], p[1], p[2]);}
  const Sim3 fit = umeyama(from, to);
  ASSERT_TRUE(fit.ok);
  EXPECT_NEAR(cv::determinant(cv::Mat(fit.r)), 1.0, 1e-9);
}

TEST(Umeyama, RefusesWhatItCannotDetermine)
{
  // Two points, and points on a line: both leave a rotation free, and any answer
  // would be a guess presented as an alignment.
  EXPECT_FALSE(umeyama({{0, 0, 0}, {1, 0, 0}}, {{0, 0, 0}, {1, 0, 0}}).ok);
  std::vector<cv::Vec3d> line;
  for (int i = 0; i < 10; ++i) {line.emplace_back(i * 0.1, 0.0, 0.0);}
  EXPECT_FALSE(umeyama(line, line).ok);
}

TEST(Sim3Apply, MovesPositionsAsPointsAndOnlyRotatesOrientations)
{
  Sim3 sim;
  sim.s = 2.0;
  sim.r = rot(cv::Vec3d(0, 0, CV_PI / 2));
  sim.t = cv::Vec3d(1, 0, 0);
  sim.ok = true;
  const cv::Affine3d pose(rot(cv::Vec3d(0.1, 0, 0)), cv::Vec3d(1, 0, 0));
  const cv::Affine3d out = apply(sim, pose);
  // Position: 2 * Rz(90)(1,0,0) + (1,0,0) = (1,2,0).
  EXPECT_LT(cv::norm(out.translation() - cv::Vec3d(1, 2, 0)), 1e-12);
  EXPECT_LT(rotation_deg(out.rotation().t() * (sim.r * pose.rotation())), 1e-9);
}

TEST(GroundTruthAt, TakesTheNearestSampleWithinTolerance)
{
  const std::vector<std::pair<std::int64_t, cv::Affine3d>> gt{
    {1000 * 1000000LL, cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(1, 0, 0))},
    {1010 * 1000000LL, cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(2, 0, 0))}};
  cv::Affine3d p;
  ASSERT_TRUE(groundtruth_at(gt, 1004 * 1000000LL, p));
  EXPECT_EQ(p.translation()[0], 1.0);
  ASSERT_TRUE(groundtruth_at(gt, 1006 * 1000000LL, p));
  EXPECT_EQ(p.translation()[0], 2.0);
  EXPECT_FALSE(groundtruth_at(gt, 1100 * 1000000LL, p)) << "90 ms from the nearest is no match";
}

// --- The arm score ------------------------------------------------------------------

namespace
{

cv::Matx33d test_k()
{
  return cv::Matx33d(500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0);
}

/// A wall 2 m ahead, walked past twice by a camera at six positions on a small grid.
/// Odometry on the second pass has drifted 0.3 m toward the wall, so its arm puts a
/// ghost wall at 1.7 m; the corrected arm has the truth. Ground truth is the truth in
/// another frame — rotated, shifted and at twice the scale — the way a motion-capture
/// world relates to this pipeline's map.
struct Walk
{
  FrameMemory memory {FrameMemory::Config{100, 4}};
  std::vector<cv::Affine3d> truth, drifted;
  std::vector<std::pair<std::int64_t, cv::Affine3d>> groundtruth;

  Walk()
  {
    const cv::Mat wall(480, 640, CV_32FC1, cv::Scalar(2.0F));
    Sim3 world;
    world.s = 2.0;
    world.r = rot(cv::Vec3d(0.2, 0.5, -0.1));
    world.t = cv::Vec3d(3, -1, 0.5);
    world.ok = true;
    int n = 0;
    for (int pass = 0; pass < 2; ++pass) {
      for (int i = 0; i < 6; ++i) {
        const cv::Affine3d pose(cv::Matx33d::eye(), cv::Vec3d(0.3 * (i % 3), 0.2 * (i / 3), 0.0));
        const cv::Affine3d odom = pass == 0 ? pose :
          cv::Affine3d(cv::Matx33d::eye(), pose.translation() + cv::Vec3d(0, 0, 0.3));
        const std::int64_t stamp = (pass * 10 + i) * kSecond;
        memory.offer(stamp, wall, cv::Mat(), test_k(), odom, cv::Affine3d::Identity());
        truth.push_back(pose);
        drifted.push_back(odom);
        groundtruth.emplace_back(stamp, apply(world, pose));
        ++n;
      }
    }
  }
};

}  // namespace

TEST(ScoreArm, AnArmCarryingDriftScoresWorseThanOneWithoutOverTheSameFrames)
{
  // The property the instrument exists for, and the one its predecessor lacked. Both
  // arms are the same twelve frames; only the poses differ. The drifted arm must be
  // further from the ground-truth surface **and** the corrected one must be close to
  // it — so this cannot pass by both being bad, or by the instrument ignoring poses.
  const Walk walk;
  const TsdfVolume::Options options;
  const ArmScore corrected = score_arm(walk.memory.frames(), walk.truth, walk.groundtruth,
    options, 4, 0.2, 0.05);
  const ArmScore drifted = score_arm(walk.memory.frames(), walk.drifted, walk.groundtruth,
    options, 4, 0.2, 0.05);
  ASSERT_TRUE(corrected.alignment.ok && drifted.alignment.ok);
  EXPECT_EQ(corrected.judged, 12u);
  // The alignment undid the ground truth's frame: scale 1/2 back onto the map.
  EXPECT_NEAR(corrected.alignment.s, 0.5, 1e-6);
  EXPECT_LT(corrected.ate_m, 1e-6);
  EXPECT_GT(drifted.ate_m, 0.05);
  ASSERT_GE(corrected.surface.gap_m, 0.0);
  ASSERT_GE(drifted.surface.gap_m, 0.0);
  EXPECT_LT(corrected.surface.gap_m, 0.02);
  EXPECT_GT(drifted.surface.gap_m, corrected.surface.gap_m + 0.05);
}

TEST(ScoreArm, FramesWithNoGroundTruthAreNotJudgedAndSaySo)
{
  Walk walk;
  walk.groundtruth.resize(6);   // the second pass has no motion capture
  const ArmScore s = score_arm(walk.memory.frames(), walk.truth, walk.groundtruth,
    TsdfVolume::Options{}, 4, 0.2, 0.05);
  EXPECT_EQ(s.frames, 12u);
  EXPECT_EQ(s.judged, 6u);
}

// --- The memory dump ----------------------------------------------------------------

TEST(MemoryDump, RoundTripsEverythingTheOfflineRebuildReads)
{
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / ("pimesh_dump_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  Walk walk;
  const Corrections corrections{{0, cv::Affine3d::Identity()},
    {10 * kSecond, cv::Affine3d(rot(cv::Vec3d(0, 0.01, 0)), cv::Vec3d(0, 0, -0.3))}};
  TsdfVolume::Options options;
  options.voxel_size_m = 0.02F;
  options.max_blocks = 1234;
  ASSERT_EQ(pimesh_mapping::write_memory(dir.string(), walk.memory.frames(), corrections, options, 4), "");
  EXPECT_FALSE(fs::exists(dir.string() + ".partial")) << "the partial directory is renamed, not left";

  std::vector<RememberedFrame> frames;
  Corrections back;
  TsdfVolume::Options read_options;
  int downsample = 0;
  ASSERT_EQ(pimesh_mapping::read_memory(dir.string(), frames, back, read_options, downsample), "");
  ASSERT_EQ(frames.size(), walk.memory.size());
  EXPECT_EQ(downsample, 4);
  EXPECT_EQ(read_options.voxel_size_m, 0.02F);
  EXPECT_EQ(read_options.max_blocks, 1234u);
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const RememberedFrame & a = walk.memory.frames()[i];
    const RememberedFrame & b = frames[i];
    EXPECT_EQ(a.stamp_ns, b.stamp_ns);
    EXPECT_EQ(cv::norm(a.depth_mm, b.depth_mm, cv::NORM_INF), 0.0);
    EXPECT_EQ(cv::norm(a.k - b.k), 0.0);
    EXPECT_EQ(cv::norm(a.odom_from_camera.translation() - b.odom_from_camera.translation()), 0.0)
      << "full precision, not six figures";
    EXPECT_TRUE(b.bgr.empty());
  }
  ASSERT_EQ(back.size(), 2u);
  EXPECT_EQ(back[1].first, 10 * kSecond);
  EXPECT_EQ(cv::norm(back[1].second.translation() - corrections[1].second.translation()), 0.0);

  // A missing image is an error, not a shorter memory.
  fs::remove(dir / "d000003.png");
  EXPECT_NE(pimesh_mapping::read_memory(dir.string(), frames, back, read_options, downsample), "");
  fs::remove_all(dir);
}
