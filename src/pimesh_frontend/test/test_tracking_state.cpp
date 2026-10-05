// #13's P19: the OK / LOST transitions.
//
// **Both ways of getting this wrong produce a pipeline that runs.** A monitor that
// goes LOST too eagerly refuses a fifth of `bags/desk1` and the mesh is merely
// thinner; one that never goes LOST fuses through every blackout and the mesh is
// merely ghostlier. Neither crashes, neither silences a topic, and the second has
// a perfect "no frame fused while lost" record — which is why the transitions are
// pinned here rather than left to a gate.

#include <gtest/gtest.h>

#include <stdexcept>

#include "pimesh_frontend/tracking_state.hpp"

using pimesh_frontend::Tracking;
using pimesh_frontend::TrackingMonitor;

namespace
{

TrackingMonitor monitor(std::size_t lost_after, std::size_t recover_after)
{
  return TrackingMonitor(TrackingMonitor::Config{lost_after, recover_after});
}

void feed(TrackingMonitor & m, bool posed, std::size_t times)
{
  for (std::size_t i = 0; i < times; ++i) {m.observe(posed);}
}

}  // namespace

TEST(TrackingMonitor, StartsOk)
{
  // Before any frame the pose is the origin, which is a definition rather than an
  // estimate — there is nothing to be lost from yet.
  const TrackingMonitor m = monitor(5, 2);
  EXPECT_EQ(m.state(), Tracking::Ok);
  EXPECT_EQ(m.frames(), 0u);
}

TEST(TrackingMonitor, ARunShorterThanTheThresholdIsNotLost)
{
  TrackingMonitor m = monitor(5, 2);
  feed(m, false, 4);
  EXPECT_EQ(m.state(), Tracking::Ok);
  EXPECT_EQ(m.holds(), 4u);
}

TEST(TrackingMonitor, LostOnExactlyTheThresholdthHold)
{
  TrackingMonitor m = monitor(5, 2);
  for (int i = 0; i < 4; ++i) {EXPECT_FALSE(m.observe(false)) << "hold " << i + 1;}
  EXPECT_TRUE(m.observe(false)) << "the fifth consecutive hold must enter LOST";
  EXPECT_EQ(m.state(), Tracking::Lost);
  EXPECT_EQ(m.entered_lost(), 1u);
  EXPECT_EQ(m.lost_frames(), 1u) << "the frame that entered LOST is a LOST frame";
}

TEST(TrackingMonitor, ScatteredHoldsNeverAddUp)
{
  // bags/desk1 holds ~20% of its depth frames, as singles while the camera moves
  // fast. A counter that accumulated rather than reset would declare LOST every
  // few seconds of a run in which nothing was wrong.
  TrackingMonitor m = monitor(5, 2);
  for (int i = 0; i < 1000; ++i) {
    m.observe(false);
    m.observe(i % 3 != 0);   // runs of one or two holds, never five
    m.observe(true);
  }
  EXPECT_EQ(m.state(), Tracking::Ok);
  EXPECT_EQ(m.entered_lost(), 0u);
  EXPECT_LE(m.longest_hold_run(), 2u);
}

TEST(TrackingMonitor, LostIsReachablePastTheStallRule)
{
  // odometry_node resets its own hold counter every max_hold_frames (5) when it
  // takes a new reference. A monitor driven from that counter could never reach a
  // threshold above 5 — so this one counts for itself, and a long blackout is LOST
  // whatever the stall rule does underneath it.
  TrackingMonitor m = monitor(12, 2);
  feed(m, false, 11);
  EXPECT_EQ(m.state(), Tracking::Ok);
  m.observe(false);
  EXPECT_EQ(m.state(), Tracking::Lost);
}

TEST(TrackingMonitor, RecoveryNeedsConsecutiveFits)
{
  TrackingMonitor m = monitor(5, 3);
  feed(m, false, 5);
  ASSERT_EQ(m.state(), Tracking::Lost);
  // Two fits, a hold, two fits: never three in a row.
  feed(m, true, 2);
  m.observe(false);
  feed(m, true, 2);
  EXPECT_EQ(m.state(), Tracking::Lost) << "a hold between fits restarts the count";
  EXPECT_TRUE(m.observe(true)) << "the third consecutive fit recovers";
  EXPECT_EQ(m.state(), Tracking::Ok);
  EXPECT_EQ(m.recovered(), 1u);
}

TEST(TrackingMonitor, FitsWhileLostAreStillLostFrames)
{
  // A fit that has not yet earned recovery is a frame fusion_node must refuse. The
  // count is what the gate compares against fusion's refusals, so it has to
  // include them.
  TrackingMonitor m = monitor(2, 3);
  feed(m, false, 2);   // LOST on the second: 1 lost frame
  feed(m, true, 2);    // still LOST: 3
  m.observe(true);     // recovers on this one: not a lost frame
  EXPECT_EQ(m.state(), Tracking::Ok);
  EXPECT_EQ(m.lost_frames(), 3u);
}

TEST(TrackingMonitor, ANeverRecoveringMonitorStaysLost)
{
  // tools/gates/lost.sh's control: recover_after_fits past the length of the clip.
  // It must stay LOST through any number of fits — that is the run whose OK
  // fraction the gate's floor has to be seen to fail.
  TrackingMonitor m = monitor(5, 1000000);
  feed(m, false, 5);
  feed(m, true, 5000);
  EXPECT_EQ(m.state(), Tracking::Lost);
  EXPECT_EQ(m.recovered(), 0u);
}

TEST(TrackingMonitor, ReLosesAfterRecovering)
{
  TrackingMonitor m = monitor(3, 2);
  feed(m, false, 3);
  feed(m, true, 2);
  ASSERT_EQ(m.state(), Tracking::Ok);
  feed(m, false, 3);
  EXPECT_EQ(m.state(), Tracking::Lost);
  EXPECT_EQ(m.entered_lost(), 2u);
  EXPECT_EQ(m.recovered(), 1u);
}

TEST(TrackingMonitor, ZeroThresholdsAreRefused)
{
  EXPECT_THROW(monitor(0, 2), std::invalid_argument);
  EXPECT_THROW(monitor(5, 0), std::invalid_argument);
}

TEST(TrackingMonitor, NamesAreTheOnesTheGateGreps)
{
  // tools/gates/lost.sh and the dashboard read these strings; the message carries
  // a number, and the log lines carry these.
  EXPECT_STREQ(pimesh_frontend::tracking_name(Tracking::Ok), "OK");
  EXPECT_STREQ(pimesh_frontend::tracking_name(Tracking::Lost), "LOST");
}


// --- #13's P20: with a loaded map, only a relocalisation leaves LOST ---------------

namespace
{

TrackingMonitor relocalising(std::size_t lost_after)
{
  TrackingMonitor::Config c{lost_after, 2};
  c.recover_by_relocalisation = true;
  return TrackingMonitor(c);
}

}  // namespace

TEST(TrackingMonitorRelocalising, StartsLost)
{
  // A fresh session with a saved map has an odom frame unrelated to it. OK would say
  // "posed in the map" about a pose that is posed in nothing of the kind.
  const TrackingMonitor m = relocalising(5);
  EXPECT_EQ(m.state(), Tracking::Lost);
  EXPECT_EQ(m.entered_lost(), 0u) << "starting LOST is not entering it";
}

TEST(TrackingMonitorRelocalising, FitsAloneNeverRecover)
{
  TrackingMonitor m = relocalising(5);
  feed(m, true, 10000);
  EXPECT_EQ(m.state(), Tracking::Lost);
  EXPECT_EQ(m.lost_frames(), 10000u);
}

TEST(TrackingMonitorRelocalising, ARelocalisationRecoversOnce)
{
  TrackingMonitor m = relocalising(5);
  feed(m, true, 3);
  EXPECT_TRUE(m.relocalised());
  EXPECT_EQ(m.state(), Tracking::Ok);
  EXPECT_FALSE(m.relocalised()) << "a late answer while OK must change nothing";
  EXPECT_EQ(m.recovered(), 1u);
  EXPECT_EQ(m.relocalisations(), 1u);
}

TEST(TrackingMonitorRelocalising, ALaterBlackoutNeedsAnotherRelocalisation)
{
  TrackingMonitor m = relocalising(5);
  m.relocalised();
  feed(m, false, 5);
  ASSERT_EQ(m.state(), Tracking::Lost);
  feed(m, true, 50);
  EXPECT_EQ(m.state(), Tracking::Lost) << "odometric recovery is not recovery in the map";
  EXPECT_TRUE(m.relocalised());
  EXPECT_EQ(m.entered_lost(), 1u);
}

TEST(TrackingMonitorRelocalising, WithoutAMapRelocalisingIsInert)
{
  // P19's mode: relocalised() is still callable and still means "leave LOST", but
  // nothing calls it without a map, and fits recover as before.
  TrackingMonitor m = monitor(5, 2);
  EXPECT_FALSE(m.relocalised());
  feed(m, false, 5);
  feed(m, true, 2);
  EXPECT_EQ(m.state(), Tracking::Ok);
  EXPECT_EQ(m.relocalisations(), 0u);
}
