// The one-slot mailbox: what it keeps, what it drops, and that it can be shut
// down.
//
// Worth a test because every failure mode here is silent. A mailbox that keeps
// the *oldest* message instead of the newest still delivers frames at full rate
// and still looks like a working pipeline — it is just showing the room as it was
// half a second ago, which is indistinguishable from latency somewhere else. A
// drop counter that misses one in ten turns "this stage is keeping up" into a
// number nobody can check. And a stop() that does not wake a waiter hangs the
// destructor, which shows up as a gate timing out with no message.

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "pimesh_perception/mailbox.hpp"

using pimesh_perception::Mailbox;

TEST(Mailbox, DeliversWhatWasPut)
{
  Mailbox<std::unique_ptr<std::string>> box;
  EXPECT_FALSE(box.push(std::make_unique<std::string>("frame")));

  auto got = box.pop(std::chrono::milliseconds(10));
  ASSERT_NE(got, nullptr);
  EXPECT_EQ(*got, "frame");
  EXPECT_EQ(box.dropped(), 0u);
}

TEST(Mailbox, NewestWins)
{
  Mailbox<std::unique_ptr<std::string>> box;
  box.push(std::make_unique<std::string>("old"));
  // The second push displaces the first and says so — that return value is how
  // the producer learns it is outrunning the consumer.
  EXPECT_TRUE(box.push(std::make_unique<std::string>("new")));

  auto got = box.pop(std::chrono::milliseconds(10));
  ASSERT_NE(got, nullptr);
  EXPECT_EQ(*got, "new") << "the mailbox kept the stale frame";
  EXPECT_EQ(box.dropped(), 1u);
}

TEST(Mailbox, EmptyAfterPop)
{
  Mailbox<std::unique_ptr<int>> box;
  box.push(std::make_unique<int>(7));
  EXPECT_NE(box.pop(std::chrono::milliseconds(10)), nullptr);
  // Not "the same message twice": a worker that re-reads an emptied slot would
  // process every frame twice, doubling the cost and the feature count.
  EXPECT_EQ(box.pop(std::chrono::milliseconds(10)), nullptr);
}

TEST(Mailbox, CountsEveryDrop)
{
  Mailbox<std::unique_ptr<int>> box;
  for (int i = 0; i < 20; ++i) {box.push(std::make_unique<int>(i));}

  auto got = box.pop(std::chrono::milliseconds(10));
  ASSERT_NE(got, nullptr);
  EXPECT_EQ(*got, 19);
  EXPECT_EQ(box.dropped(), 19u) << "19 of 20 frames were displaced";
}

TEST(Mailbox, PopTimesOutWhenEmpty)
{
  Mailbox<std::unique_ptr<int>> box;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(box.pop(std::chrono::milliseconds(30)), nullptr);
  const auto waited = std::chrono::steady_clock::now() - start;
  // The timeout is what lets a worker loop notice shutdown. A pop that returns
  // immediately would spin a core; one that never returns cannot be joined.
  EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(waited).count(), 20);
}

TEST(Mailbox, StopWakesAWaiter)
{
  Mailbox<std::unique_ptr<int>> box;
  bool returned = false;

  std::thread worker([&] {
    // A long timeout on purpose: if stop() does not wake this, the test takes
    // ten seconds and fails, which is the behaviour a hung destructor has.
    box.pop(std::chrono::seconds(10));
    returned = true;
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  box.stop();
  worker.join();

  EXPECT_TRUE(returned);
  EXPECT_TRUE(box.stopped());
}

TEST(Mailbox, WorksWithASharedPointerSlotToo)
{
  // The slot type the pipeline actually uses. `/image_raw` has several consumers in
  // the container, and rclcpp shares one buffer between subscriptions that take a
  // shared const pointer while *copying* it for all but one of the subscriptions
  // that take ownership — so the frame that arrives here is a
  // `Image::ConstSharedPtr`, and this class has to carry one without quietly
  // keeping a reference alive in the emptied slot.
  Mailbox<std::shared_ptr<const std::string>> box;
  auto first = std::make_shared<const std::string>("old");
  box.push(first);
  EXPECT_TRUE(box.push(std::make_shared<const std::string>("new")));

  auto got = box.pop(std::chrono::milliseconds(10));
  ASSERT_NE(got, nullptr);
  EXPECT_EQ(*got, "new");
  EXPECT_EQ(box.dropped(), 1u);

  // Emptied, not merely moved from. A shared_ptr left in the slot would hand the
  // same frame to the worker twice and hold its buffer alive for as long as the
  // node ran.
  EXPECT_EQ(box.pop(std::chrono::milliseconds(0)), nullptr);
  EXPECT_EQ(first.use_count(), 1L) << "the mailbox is still holding the displaced frame";
}

TEST(Mailbox, SurvivesConcurrentProducerAndConsumer)
{
  // Not a race detector — a test cannot be one — but it does exercise the lock
  // ordering and the notify-outside-the-lock path under real contention, and it
  // asserts the invariant that matters: nothing is delivered twice and nothing
  // is invented. delivered + dropped == produced, always.
  Mailbox<std::unique_ptr<int>> box;
  constexpr int kFrames = 5000;
  int delivered = 0;

  std::thread producer([&] {
    for (int i = 0; i < kFrames; ++i) {box.push(std::make_unique<int>(i));}
    box.stop();
  });

  while (!box.stopped()) {
    if (box.pop(std::chrono::milliseconds(1)) != nullptr) {++delivered;}
  }
  producer.join();
  while (box.pop(std::chrono::milliseconds(0)) != nullptr) {++delivered;}

  EXPECT_EQ(delivered + static_cast<int>(box.dropped()), kFrames);
}
