// The backend thread's queue, and that a keyframe offered is a keyframe processed.
//
// **The queue refusing is the behaviour, not a failure.** Every other queue in this
// project is newest-wins and drops; a dropped keyframe is a hole in the map, so this
// one says no and the caller keeps the keyframe. A queue that silently dropped
// instead would produce a map with fewer keyframes and nothing else wrong with it.

#include <gtest/gtest.h>

#include <chrono>
#include <random>
#include <thread>
#include <vector>

#include "pimesh_backend/local_mapper.hpp"
#include "pimesh_backend/triangulation.hpp"

using pimesh_backend::KeyframeInput;
using pimesh_backend::LocalMapper;
using pimesh_backend::Map;

namespace
{

cv::Matx33d camera_k()
{
  return cv::Matx33d(517.3, 0.0, 318.6, 0.0, 516.5, 255.3, 0.0, 0.0, 1.0);
}

/// A keyframe from a camera at `x` on a line, seeing a fixed room with track ids
/// equal to point indices. `matched` is left empty of claims; the tests that need
/// associations let the map's own track index supply them via lookup.
KeyframeInput keyframe_at(double x, const Map * map = nullptr)
{
  std::mt19937 gen(3);
  std::uniform_real_distribution<double> px(-1.0, 1.0);
  std::uniform_real_distribution<double> py(-0.6, 0.6);
  std::uniform_real_distribution<double> pz(2.0, 4.0);
  std::uniform_int_distribution<int> byte(0, 255);
  KeyframeInput in;
  in.map_from_camera = cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(x, 0.0, 0.0));
  in.k = camera_k();
  std::vector<cv::Mat> rows;
  for (int i = 0; i < 150; ++i) {
    const cv::Vec3d p(px(gen), py(gen), pz(gen));
    cv::Mat d(1, 32, CV_8U);
    for (int b = 0; b < 32; ++b) {d.at<std::uint8_t>(0, b) = static_cast<std::uint8_t>(byte(gen));}
    cv::Point2d pixel;
    if (!pimesh_backend::project(in.map_from_camera, in.k, p, pixel)) {continue;}
    if (pixel.x < 0 || pixel.x >= 640 || pixel.y < 0 || pixel.y >= 480) {continue;}
    in.pixels.emplace_back(static_cast<float>(pixel.x), static_cast<float>(pixel.y));
    in.track_ids.push_back(i);
    rows.push_back(d);
    in.network_points.push_back(in.map_from_camera.inv() * p);
    in.has_depth.push_back(1);
  }
  cv::vconcat(rows, in.descriptors);
  in.matched.assign(in.pixels.size(), pimesh_backend::kNoPoint);
  if (map != nullptr) {
    const auto views = map->lookup_tracks(in.track_ids);
    for (std::size_t i = 0; i < views.size(); ++i) {in.matched[i] = views[i].id;}
  }
  return in;
}

}  // namespace

TEST(LocalMapper, RefusesWhenFullRatherThanDropping)
{
  Map map(Map::Config{});
  LocalMapper::Config config;
  config.queue = 2;
  LocalMapper mapper(map, config);
  // Not started: nothing drains the queue.
  EXPECT_TRUE(mapper.try_push(keyframe_at(0.0)));
  EXPECT_TRUE(mapper.try_push(keyframe_at(0.1)));
  EXPECT_FALSE(mapper.try_push(keyframe_at(0.2)));
  EXPECT_EQ(mapper.stats().refused_full, 1u);
  mapper.start();
  mapper.flush();
  // The two it accepted are in the map; the one it refused is the caller's to
  // offer again, and offering it again succeeds.
  EXPECT_EQ(map.stats().keyframes, 2u);
  EXPECT_TRUE(mapper.try_push(keyframe_at(0.2)));
  mapper.flush();
  EXPECT_EQ(map.stats().keyframes, 3u);
  EXPECT_EQ(mapper.stats().processed, 3u);
}

TEST(LocalMapper, WithoutBundleAdjustmentItNeverSolves)
{
  Map map(Map::Config{});
  LocalMapper mapper(map, LocalMapper::Config{});
  mapper.start();
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(mapper.try_push(keyframe_at(0.1 * i, &map)));
    mapper.flush();
  }
  const auto stats = mapper.stats();
  EXPECT_EQ(stats.processed, 4u);
  EXPECT_EQ(stats.ba_runs, 0u);
  EXPECT_EQ(stats.ba_refused, 0u);
  EXPECT_GT(stats.associated, 0u);
}

TEST(LocalMapper, WithBundleAdjustmentEveryKeyframeAfterTheFirstIsSolved)
{
  // **The plan's second false green for P15: a BA that never runs.** A window that is
  // always empty, a solver that returns at iteration 0 — each produces a run
  // identical to the control. So: solves counted, iterations above zero, and a
  // window of more than one keyframe.
  Map map(Map::Config{});
  LocalMapper::Config config;
  config.bundle_adjust = true;
  LocalMapper mapper(map, config);
  mapper.start();
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(mapper.try_push(keyframe_at(0.08 * i, &map)));
    mapper.flush();
  }
  const auto stats = mapper.stats();
  // The first keyframe is alone and fixed — the map's origin — so it is refused, and
  // said to be; every later one is solved.
  EXPECT_EQ(stats.ba_refused, 1u);
  EXPECT_EQ(stats.ba_runs, 4u);
  EXPECT_GT(stats.iterations, 0u);
  EXPECT_GT(stats.window_free + stats.window_fixed, stats.ba_runs);
  EXPECT_EQ(stats.ba_ms.size(), stats.ba_runs);
}

TEST(LocalMapper, TheThreadIsNiced)
{
  // Raising a thread's nice value needs no privilege, so this holds on both
  // machines under any user. If it ever reports false, BA is competing with
  // depth_node for the CPU at equal priority, which is mesh_node's measured
  // 17.8 -> 14.6 Hz arriving by a second door.
  Map map(Map::Config{});
  LocalMapper mapper(map, LocalMapper::Config{});
  mapper.start();
  ASSERT_TRUE(mapper.try_push(keyframe_at(0.0)));
  mapper.flush();
  EXPECT_TRUE(mapper.stats().niced);
}

TEST(LocalMapper, StopsPromptlyWithKeyframesStillQueued)
{
  // odometry_node's destructor joins this thread while the container shuts down. A
  // stop that waited for the queue to drain would hold up the whole container's
  // teardown behind bundle adjustment, and a stop that never woke a thread idle on
  // its condition variable would hang it outright — gates/teardown.sh would see the
  // container as a straggler. Destroyed with two keyframes queued and one being
  // processed, it must return.
  Map map(Map::Config{});
  LocalMapper::Config config;
  config.bundle_adjust = true;
  {
    LocalMapper mapper(map, config);
    mapper.start();
    ASSERT_TRUE(mapper.try_push(keyframe_at(0.0)));
    ASSERT_TRUE(mapper.try_push(keyframe_at(0.1)));
  }
  SUCCEED();
}

TEST(LocalMapper, AnIdleThreadStopsToo)
{
  // **Idle means parked on the condition variable**, and the first version of this
  // test never got it there: destroyed straight after start(), the worker had not
  // reached its wait yet, saw the stop flag on the way in, and left — so a
  // destructor that set the flag and never notified passed it. Caught by mutation.
  // One keyframe processed, the queue flushed, and a moment for the worker to go
  // back to sleep; *then* the destructor has something to wake.
  Map map(Map::Config{});
  {
    LocalMapper mapper(map, LocalMapper::Config{});
    mapper.start();
    ASSERT_TRUE(mapper.try_push(keyframe_at(0.0)));
    mapper.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  SUCCEED();
}

