#include "pimesh_frontend/relocaliser.hpp"

#include <sys/resource.h>

#include <chrono>
#include <utility>

namespace pimesh_frontend
{

Relocaliser::Relocaliser(const Config & config, std::deque<Keyframe> map)
: config_(config), map_(std::move(map))
{
  config_.place.across_sessions = true;
}

Relocaliser::~Relocaliser()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  if (worker_.joinable()) {worker_.join();}
}

void Relocaliser::start()
{
  if (worker_.joinable()) {return;}
  worker_ = std::thread([this] {this->run();});
}

void Relocaliser::submit(Keyframe query)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_) {++stats_.skipped;}
    pending_ = std::move(query);
    ++stats_.submitted;
  }
  wake_.notify_one();
}

std::vector<Relocalisation> Relocaliser::take_results()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Relocalisation> out;
  out.swap(results_);
  return out;
}

void Relocaliser::flush()
{
  std::unique_lock<std::mutex> lock(mutex_);
  drained_.wait(lock, [this] {return (!pending_ && !busy_) || stop_;});
}

Relocaliser::Stats Relocaliser::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void Relocaliser::run()
{
  // Per thread: nice applies to this search and not to the pose worker that feeds it.
  const bool niced = config_.nice == 0 || setpriority(PRIO_PROCESS, 0, config_.nice) == 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.niced = niced;
  }
  for (;;) {
    Keyframe query;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this] {return stop_ || pending_.has_value();});
      if (stop_) {break;}
      query = std::move(*pending_);
      pending_.reset();
      busy_ = true;
    }

    const auto start = std::chrono::steady_clock::now();
    const PlaceResult found = find_place(map_, query, config_.place);
    Relocalisation r;
    r.query_stamp_ns = query.stamp_ns;
    r.accepted = found.accepted;
    r.searched = found.searched;
    r.odom_from_camera = query.odom_from_camera;
    if (found.verified > 0) {
      r.candidate_stamp_ns = found.best.candidate_stamp_ns;
      r.inliers = found.best.inliers;
      r.matches = found.best.matches;
    }
    if (found.accepted) {
      for (const Keyframe & kf : map_) {
        if (kf.stamp_ns == found.best.candidate_stamp_ns) {
          r.map_from_camera = relocalised_pose(kf.odom_from_camera, found.best.query_from_candidate);
          break;
        }
      }
    }
    r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.queries;
      if (r.accepted) {++stats_.accepted;}
      stats_.query_ms.push_back(r.ms);
      results_.push_back(std::move(r));
      busy_ = false;
    }
    drained_.notify_all();
  }
  drained_.notify_all();
}

}  // namespace pimesh_frontend
