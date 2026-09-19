#include "pimesh_perception/keyframe_store.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

namespace pimesh_perception
{

std::size_t Keyframe::bytes() const
{
  return static_cast<std::size_t>(descriptors.total() * descriptors.elemSize()) +
         bearings.size() * sizeof(cv::Vec3d) +
         track_ids.size() * sizeof(std::int32_t) +
         landmarks.size() * sizeof(cv::Vec3d) +
         landmark_row.size() * sizeof(std::int32_t);
}

double angle_between(const cv::Matx33d & a, const cv::Matx33d & b)
{
  const cv::Matx33d relative = a.t() * b;
  const double trace = relative(0, 0) + relative(1, 1) + relative(2, 2);
  return std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
}

bool KeyframeStore::would_insert(const cv::Affine3d & pose) const
{
  if (frames_.empty()) {return true;}

  const cv::Affine3d & last = frames_.back().odom_from_camera;
  const double moved = cv::norm(cv::Vec3d(pose.translation()) - cv::Vec3d(last.translation()));
  if (moved >= config_.min_translation_m) {return true;}

  return angle_between(last.rotation(), pose.rotation()) >= config_.min_angle_rad;
}

bool KeyframeStore::maybe_insert(Keyframe frame)
{
  if (!would_insert(frame.odom_from_camera)) {return false;}

  frames_.push_back(std::move(frame));
  // Oldest first, which is the wrong policy for a relocaliser and the right one
  // for a bound that must never be reached in a session this project actually
  // runs. When loop closure consumes this, the eviction rule becomes a decision
  // with a measurement behind it; until then it is a ceiling, not a strategy, and
  // `dropped()` is what says the ceiling was ever hit.
  while (frames_.size() > config_.max_keyframes) {
    frames_.pop_front();
    ++dropped_;
  }
  return true;
}

std::size_t KeyframeStore::bytes() const
{
  std::size_t total = 0;
  for (const Keyframe & frame : frames_) {total += frame.bytes();}
  return total;
}

void KeyframeStore::clear()
{
  frames_.clear();
  dropped_ = 0;
}

}  // namespace pimesh_perception
