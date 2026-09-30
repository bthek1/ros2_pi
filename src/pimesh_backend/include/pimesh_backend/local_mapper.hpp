#ifndef PIMESH_BACKEND__LOCAL_MAPPER_HPP_
#define PIMESH_BACKEND__LOCAL_MAPPER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "pimesh_backend/local_ba.hpp"
#include "pimesh_backend/map.hpp"

namespace pimesh_backend
{

/// The backend thread: takes keyframes from tracking, puts them into the map, and
/// (with bundle adjustment on) re-solves the window around each one.
///
/// **Two conventions of this project broken on purpose, and both are #11's.**
///
///   - **The queue is not a newest-wins mailbox.** Every other queue here drops the
///     older frame, because every other stage is per-frame and a dropped frame is
///     one frame. A dropped *keyframe* is a hole in the map. So this is a bounded
///     queue that says *no* when full — `try_push` returns false and the caller keeps
///     the keyframe and offers it again — which is backpressure on admission, not a
///     drop. The keyframe admission test is already a rate limiter, a keyframe every
///     half second or so, so the bound is small.
///   - **The thread is niced**, for mesh_node's measured reason: an equal-priority
///     CPU-heavy thread took depth_node from 17.8 to 14.6 Hz with its per-frame cost
///     unchanged — not more work, just not scheduled. Linux nice is per thread, so
///     this pushes down bundle adjustment and nothing else in the container.
class LocalMapper
{
public:
  struct Config
  {
    std::size_t queue {2};
    /// Bundle-adjust the window around each keyframe. false is gates/ba.sh's control:
    /// the same thread doing the same insert, triangulate and cull, and skipping only
    /// the solve.
    bool bundle_adjust {false};
    std::size_t window_keyframes {10};
    std::size_t max_fixed_keyframes {20};
    BaConfig ba;
    int nice {10};
  };

  struct Stats
  {
    std::uint64_t processed {0};
    std::uint64_t refused_full {0};
    std::uint64_t ba_runs {0};
    std::uint64_t ba_refused {0};
    std::uint64_t iterations {0};
    std::uint64_t window_free {0};
    std::uint64_t window_fixed {0};
    std::uint64_t window_points {0};
    std::uint64_t window_edges {0};
    std::uint64_t outliers_dropped {0};
    double chi2_before {0.0};
    double chi2_after {0.0};
    /// Over every free keyframe that had a scale vertex: how many, and the sum of
    /// `|s - 1|`. Their ratio is `ba_scale_dev` on the stats line — 0 with scale
    /// modelling off, and the number that says whether the vertices absorbed the
    /// breathing P14 measured as `align_dev`.
    std::uint64_t scale_solved {0};
    double scale_dev_sum {0.0};
    /// Per solve, in milliseconds, for a p95 — and per keyframe for the whole of what
    /// the thread did with it.
    std::vector<double> ba_ms;
    std::vector<double> keyframe_ms;
    /// Aggregated InsertReports.
    std::uint64_t associated {0};
    std::uint64_t created {0};
    std::uint64_t refused_duplicate {0};
    std::uint64_t refused_reprojection {0};
    std::uint64_t refused_bad {0};
    std::uint64_t aligned {0};
    double align_sum {0.0};
    double align_dev_sum {0.0};
    bool niced {false};
  };

  LocalMapper(Map & map, const Config & config);
  ~LocalMapper();
  LocalMapper(const LocalMapper &) = delete;
  LocalMapper & operator=(const LocalMapper &) = delete;

  /// Start the thread. Separate from the constructor so a test can fill the queue
  /// with nothing draining it and watch it refuse.
  void start();
  /// Offer a keyframe. False when the queue is full — the caller keeps it.
  bool try_push(const KeyframeInput & input);
  /// Block until everything offered so far has been processed. For tests.
  void flush();
  Stats stats() const;

private:
  void run();
  void process(const KeyframeInput & input);

  Map & map_;
  Config config_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::condition_variable drained_;
  std::deque<KeyframeInput> queue_;
  bool busy_ {false};
  bool stop_ {false};
  std::thread worker_;
  Stats stats_;
};

}  // namespace pimesh_backend

#endif  // PIMESH_BACKEND__LOCAL_MAPPER_HPP_
