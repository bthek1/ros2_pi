// A single-slot mailbox: the newest value wins, the unread one is dropped.
//
// This is the shape every expensive stage in this pipeline uses, and it is the
// reason the pipeline does not die under load. A subscription callback runs on
// the executor's thread; if it did the work, the executor would block and the
// queue behind it would grow without bound. So the callback does one bounded
// thing — hand the frame over — and a worker thread does the milliseconds.
//
// The queue is deliberately ONE deep. Depth runs at ~13 Hz against a camera
// that delivers 30-60, so most frames must be dropped; the only question is
// which. Dropping the OLDEST is the answer for a live reconstruction: a frame
// the camera took 400 ms ago has no value once a newer one exists. A growing
// queue would instead trade latency for completeness, which is exactly backwards
// here.
//
// It is a template with no ROS in it so it can be tested without a node, and
// it counts what it dropped so `dropped_mailbox` in PipelineStats is measured
// rather than estimated.

#ifndef PIMESH_PERCEPTION__MAILBOX_HPP_
#define PIMESH_PERCEPTION__MAILBOX_HPP_

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

namespace pimesh_perception
{

template<typename T>
class Mailbox
{
public:
  /// Store a value, discarding whatever was waiting. Returns true if this
  /// call dropped an unread value — the caller counts those, it does not.
  bool put(T value)
  {
    bool displaced = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) {
        return false;
      }
      displaced = slot_.has_value();
      if (displaced) {
        ++dropped_;
      }
      slot_ = std::move(value);
    }
    // Notify OUTSIDE the lock: waking a taker that then blocks on the mutex we
    // still hold is a pointless context switch on every single frame.
    cv_.notify_one();
    return displaced;
  }

  /// Block until a value arrives or the mailbox closes. Returns false only
  /// when closed — a closed mailbox delivers nothing, including anything left
  /// in the slot, because a shutdown is not the time to process one more frame.
  bool take(T & out)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] {return closed_ || slot_.has_value();});
    if (closed_) {
      return false;
    }
    out = std::move(*slot_);
    slot_.reset();
    return true;
  }

  /// Wake every waiting taker and refuse further traffic. Idempotent, so a
  /// destructor and an explicit shutdown may both call it.
  void close()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
      slot_.reset();
    }
    cv_.notify_all();
  }

  bool closed() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return !slot_.has_value();
  }

  /// How many unread values have been overwritten since construction. This is
  /// `dropped_mailbox`, and it is expected to be large: it is the design
  /// working, not a fault.
  uint64_t dropped() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<T> slot_;
  bool closed_{false};
  uint64_t dropped_{0};
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__MAILBOX_HPP_
