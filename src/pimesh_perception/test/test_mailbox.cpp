// The mailbox is the pipeline's back-pressure policy in twenty lines, so it is
// tested for the property that matters: under load it must lose the OLD frame,
// never the new one, and it must say how many it lost.

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "pimesh_perception/mailbox.hpp"

using pimesh_perception::Mailbox;
using namespace std::chrono_literals;

TEST(Mailbox, DeliversWhatWasPut)
{
  Mailbox<int> box;
  EXPECT_FALSE(box.put(7));
  int got = 0;
  ASSERT_TRUE(box.take(got));
  EXPECT_EQ(got, 7);
}

TEST(Mailbox, NewestValueWinsAndTheOldOneIsGone)
{
  Mailbox<int> box;
  box.put(1);
  box.put(2);
  box.put(3);
  int got = 0;
  ASSERT_TRUE(box.take(got));
  EXPECT_EQ(got, 3) << "an old frame was delivered — the queue grew";
  EXPECT_TRUE(box.empty()) << "one slot means one slot";
}

TEST(Mailbox, CountsEveryFrameItDropped)
{
  Mailbox<int> box;
  EXPECT_FALSE(box.put(1)) << "the first put displaces nothing";
  EXPECT_TRUE(box.put(2));
  EXPECT_TRUE(box.put(3));
  EXPECT_EQ(box.dropped(), 2u);

  int got = 0;
  box.take(got);
  box.put(4);
  EXPECT_EQ(box.dropped(), 2u) << "putting into an empty slot is not a drop";
}

TEST(Mailbox, MovesTheValueRatherThanCopyingIt)
{
  // A move-only payload is the proof: this would not compile if the mailbox
  // copied. The real one carries a shared_ptr to a 2.7 MB frame.
  Mailbox<std::unique_ptr<int>> box;
  box.put(std::make_unique<int>(42));
  std::unique_ptr<int> got;
  ASSERT_TRUE(box.take(got));
  ASSERT_NE(got, nullptr);
  EXPECT_EQ(*got, 42);
}

TEST(Mailbox, TakeBlocksUntilAValueArrives)
{
  Mailbox<int> box;
  std::atomic<bool> woke{false};
  int got = 0;

  std::thread worker([&] {
      if (box.take(got)) {
        woke.store(true);
      }
    });

  std::this_thread::sleep_for(50ms);
  EXPECT_FALSE(woke.load()) << "take() returned before anything was put";

  box.put(99);
  worker.join();
  EXPECT_TRUE(woke.load());
  EXPECT_EQ(got, 99);
}

TEST(Mailbox, CloseWakesAWaitingTaker)
{
  // The shutdown path. Without this the worker thread parks forever on the
  // condition variable and the node's destructor never joins it.
  Mailbox<int> box;
  std::atomic<bool> returned_false{false};

  std::thread worker([&] {
      int got = 0;
      returned_false.store(!box.take(got));
    });

  std::this_thread::sleep_for(50ms);
  box.close();
  worker.join();
  EXPECT_TRUE(returned_false.load());
}

TEST(Mailbox, DeliversNothingOnceClosedEvenWithAValueWaiting)
{
  Mailbox<int> box;
  box.put(5);
  box.close();
  int got = 0;
  EXPECT_FALSE(box.take(got)) << "a shutdown is not the time to process one more frame";
  EXPECT_FALSE(box.put(6)) << "a closed mailbox accepts nothing";
}

TEST(Mailbox, CloseIsIdempotent)
{
  Mailbox<int> box;
  box.close();
  box.close();
  EXPECT_TRUE(box.closed());
}

TEST(Mailbox, SurvivesAProducerAndConsumerRunningFlatOut)
{
  // Not a timing assertion — a race detector. Every frame is either taken or
  // counted as dropped, and the two must add up to what was put.
  Mailbox<int> box;
  std::atomic<uint64_t> taken{0};
  constexpr int kFrames = 20000;

  std::thread consumer([&] {
      int got = 0;
      while (box.take(got)) {
        taken.fetch_add(1);
      }
    });

  for (int i = 0; i < kFrames; ++i) {
    box.put(i);
  }
  // Drain, then close: anything still in the slot is either taken or dropped
  // by the close, and both are accounted for below.
  while (!box.empty()) {
    std::this_thread::sleep_for(1ms);
  }
  box.close();
  consumer.join();

  EXPECT_EQ(taken.load() + box.dropped(), static_cast<uint64_t>(kFrames))
    << "a frame was neither delivered nor counted as dropped";
}
