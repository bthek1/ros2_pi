#include "pimesh_backend/local_mapper.hpp"

#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace pimesh_backend
{

LocalMapper::LocalMapper(Map & map, const Config & config)
: map_(map), config_(config) {}

LocalMapper::~LocalMapper()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  if (worker_.joinable()) {worker_.join();}
}

void LocalMapper::start()
{
  if (worker_.joinable()) {return;}
  worker_ = std::thread([this] {this->run();});
}

bool LocalMapper::try_push(const KeyframeInput & input)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= std::max<std::size_t>(1, config_.queue)) {
      ++stats_.refused_full;
      return false;
    }
    queue_.push_back(input);
  }
  wake_.notify_one();
  return true;
}

void LocalMapper::flush()
{
  std::unique_lock<std::mutex> lock(mutex_);
  drained_.wait(lock, [this] {return (queue_.empty() && !busy_) || stop_;});
}

LocalMapper::Stats LocalMapper::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void LocalMapper::run()
{
  // Per thread on Linux: `who = 0` with PRIO_PROCESS is the calling thread, not the
  // process — the same spelling mesh_node uses, for the same reason. Raising nice
  // needs no privilege.
  const bool niced = config_.nice == 0 || setpriority(PRIO_PROCESS, 0, config_.nice) == 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.niced = niced;
  }
  for (;;) {
    KeyframeInput input;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this] {return stop_ || !queue_.empty();});
      if (stop_) {break;}
      input = std::move(queue_.front());
      queue_.pop_front();
      busy_ = true;
    }
    process(input);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      busy_ = false;
    }
    drained_.notify_all();
  }
  drained_.notify_all();
}

void LocalMapper::process(const KeyframeInput & input)
{
  const auto start = std::chrono::steady_clock::now();
  const InsertReport inserted = map_.insert(input);
  if (inserted.ok) {
    map_.triangulate(inserted.id);
    map_.cull(inserted.id);
  }

  BaResult solved;
  BaProblem problem;
  double ba_ms = -1.0;
  if (inserted.ok && config_.bundle_adjust) {
    const auto ba_start = std::chrono::steady_clock::now();
    // Snapshot under the map's lock, solve without it, write back under it: tracking
    // reads the map at the depth rate and must never wait on a solve.
    problem = map_.ba_window(inserted.id, config_.window_keyframes, config_.max_fixed_keyframes);
    solved = solve_local_ba(problem, config_.ba);
    Map::ApplyReport applied;
    if (solved.ran) {applied = map_.apply_ba(problem, solved);}
    ba_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ba_start)
      .count();
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.outliers_dropped += applied.observations_dropped;
  }
  const double keyframe_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

  std::lock_guard<std::mutex> lock(mutex_);
  ++stats_.processed;
  stats_.keyframe_ms.push_back(keyframe_ms);
  stats_.associated += inserted.associated;
  stats_.created += inserted.created;
  stats_.refused_duplicate += inserted.refused_duplicate;
  stats_.refused_reprojection += inserted.refused_reprojection;
  stats_.refused_bad += inserted.refused_bad;
  if (inserted.scale_pairs > 0) {
    ++stats_.aligned;
    stats_.align_sum += inserted.scale;
    stats_.align_dev_sum += std::abs(inserted.scale_median - 1.0);
  }
  if (ba_ms >= 0.0) {
    if (solved.ran) {
      ++stats_.ba_runs;
      stats_.iterations += static_cast<std::uint64_t>(solved.iterations);
      stats_.window_free += solved.free_keyframes;
      stats_.window_fixed += solved.fixed_keyframes;
      stats_.window_points += problem.points.size();
      stats_.window_edges += solved.edges;
      stats_.chi2_before += solved.chi2_before;
      stats_.chi2_after += solved.chi2_after;
      for (std::size_t i = 0; i < problem.keyframes.size(); ++i) {
        if (problem.keyframes[i].fixed || !(solved.depth_scale[i] > 0.0)) {continue;}
        ++stats_.scale_solved;
        stats_.scale_dev_sum += std::abs(solved.depth_scale[i] - 1.0);
      }
      stats_.ba_ms.push_back(ba_ms);
    } else {
      ++stats_.ba_refused;
    }
  }
}

}  // namespace pimesh_backend
