#ifndef PIMESH_PERCEPTION__MAILBOX_HPP_
#define PIMESH_PERCEPTION__MAILBOX_HPP_

#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <utility>

namespace pimesh_perception
{

/// One slot, newest wins: the shape every expensive stage in this pipeline hangs
/// off.
///
/// The rule it implements is the repo's "no work in a subscription callback
/// beyond a bounded copy". A callback that does the work instead blocks the
/// executor for as long as the work takes, and the only thing that can then
/// absorb the difference between a 59 Hz camera and a 13 Hz consumer is a queue
/// — which is not a buffer, it is latency with a nice name. At 59 Hz in and
/// 13 Hz out, a queue grows by 46 frames a second until the process dies; a
/// single slot drops 46 frames a second and stays current. **Dropping is the
/// design**, which is why `dropped()` is counted and logged rather than hidden:
/// a stage silently discarding 80% of its input and a stage keeping up look
/// identical from outside, and exactly one of them is fine.
///
/// A unique_ptr in and a unique_ptr out, because that is the thing being handed
/// around: under intra-process comms the pointer the callback was given is the
/// publisher's own buffer, and moving it through here keeps that true all the
/// way to the worker. Copying the message into a slot would work perfectly and
/// throw away what P2 exists to prove.
template<typename T>
class Mailbox
{
public:
  /// Put a message in the slot, discarding whatever was there.
  ///
  /// Returns true if something was displaced — the caller's cue that it is
  /// falling behind. Never blocks: the producer here is a DDS callback thread,
  /// and a producer that waits for a consumer is back to a queue.
  bool push(std::unique_ptr<T> msg)
  {
    bool displaced = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      displaced = slot_ != nullptr;
      if (displaced) {++dropped_;}
      slot_ = std::move(msg);
    }
    // Notify outside the lock: waking a thread that then immediately blocks on
    // the mutex we still hold is two context switches for nothing.
    ready_.notify_one();
    return displaced;
  }

  /// Take the message, waiting up to `timeout` for one. Null if none arrived, or
  /// if stop() has been called.
  ///
  /// The timeout is not impatience — it is what lets the worker loop notice
  /// shutdown without the producer having to wake it. A wait with no deadline
  /// here is a thread that cannot be joined.
  template<typename Duration>
  std::unique_ptr<T> pop(Duration timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait_for(lock, timeout, [this] {return slot_ != nullptr || stopped_;});
    return std::move(slot_);
  }

  /// Wake every waiter and stay awake. Called from the destructor's path, so
  /// that a worker sitting in pop() returns instead of being joined on forever.
  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    ready_.notify_all();
  }

  bool stopped() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return stopped_;
  }

  /// How many messages have been displaced, ever. The honest denominator for
  /// "this stage is keeping up".
  std::size_t dropped() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::unique_ptr<T> slot_;
  std::size_t dropped_ {0};
  bool stopped_ {false};
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__MAILBOX_HPP_
