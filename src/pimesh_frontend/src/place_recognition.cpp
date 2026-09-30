#include "pimesh_frontend/place_recognition.hpp"

#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <utility>

#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"

namespace pimesh_frontend
{
namespace
{

struct Scored
{
  std::size_t index {0};
  /// (query row, candidate row), one-to-one.
  std::vector<std::pair<int, int>> pairs;
};

/// Ratio-tested Hamming matches from the query's descriptors into one candidate's,
/// **one-to-one**: a candidate corner claimed by two query corners keeps the closer.
/// Without that a single strongly textured corner in the candidate can collect
/// matches from half the query and look like a well-matched view.
std::vector<std::pair<int, int>> match_descriptors(
  const cv::Mat & query, const cv::Mat & candidate, const PlaceConfig & config)
{
  std::vector<std::pair<int, int>> out;
  if (query.empty() || candidate.empty()) {return out;}
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> knn;
  matcher.knnMatch(query, candidate, knn, 2);
  std::vector<int> best_query(static_cast<std::size_t>(candidate.rows), -1);
  std::vector<float> best_distance(static_cast<std::size_t>(candidate.rows), 1e9f);
  for (const std::vector<cv::DMatch> & m : knn) {
    if (m.empty() || m[0].distance > static_cast<float>(config.max_hamming)) {continue;}
    if (m.size() > 1 && m[0].distance >= static_cast<float>(config.ratio) * m[1].distance) {continue;}
    const auto train = static_cast<std::size_t>(m[0].trainIdx);
    if (m[0].distance < best_distance[train]) {
      best_distance[train] = m[0].distance;
      best_query[train] = m[0].queryIdx;
    }
  }
  for (std::size_t t = 0; t < best_query.size(); ++t) {
    if (best_query[t] >= 0) {out.emplace_back(best_query[t], static_cast<int>(t));}
  }
  return out;
}

/// Geometric verification: the candidate's landmarks against the query's bearings,
/// through one rigid pose.
PlaceMatch verify(
  const Keyframe & candidate, const Keyframe & query,
  const std::vector<std::pair<int, int>> & pairs, const PlaceConfig & config)
{
  PlaceMatch match;
  match.candidate_stamp_ns = candidate.stamp_ns;
  match.matches = pairs.size();

  // landmark_row is a parallel index, landmark -> descriptor row; invert it.
  std::vector<int> landmark_of(static_cast<std::size_t>(candidate.descriptors.rows), -1);
  for (std::size_t i = 0; i < candidate.landmark_row.size() && i < candidate.landmarks.size(); ++i) {
    const int row = candidate.landmark_row[i];
    if (row >= 0 && static_cast<std::size_t>(row) < landmark_of.size()) {
      landmark_of[static_cast<std::size_t>(row)] = static_cast<int>(i);
    }
  }
  std::vector<cv::Point3d> object;
  std::vector<cv::Point2d> image;
  for (const auto & [q, c] : pairs) {
    const int li = landmark_of[static_cast<std::size_t>(c)];
    if (li < 0 || static_cast<std::size_t>(q) >= query.bearings.size()) {continue;}
    const cv::Vec3d & b = query.bearings[static_cast<std::size_t>(q)];
    if (b[2] <= 1e-6) {continue;}
    const cv::Vec3d & p = candidate.landmarks[static_cast<std::size_t>(li)];
    object.emplace_back(p[0], p[1], p[2]);
    // The bearing's point on the normalised image plane: the pixel with K divided
    // out, so the camera matrix below is the identity.
    image.emplace_back(b[0] / b[2], b[1] / b[2]);
  }
  match.with_landmark = object.size();
  const std::size_t seed_floor = std::max<std::size_t>(6, config.min_ransac_inliers);
  if (object.size() < seed_floor) {return match;}

  const double tolerance = config.reprojection_px / std::max(1.0, config.focal_px);
  cv::Mat rvec, tvec;
  std::vector<int> inliers;
  const bool ok = cv::solvePnPRansac(
    object, image, cv::Matx33d::eye(), cv::noArray(), rvec, tvec, false,
    config.ransac_iterations, tolerance, 0.99, inliers);
  if (!ok) {return match;}
  match.ransac_inliers = inliers.size();
  match.query_from_candidate = cv::Affine3d(cv::Vec3d(rvec), cv::Vec3d(tvec));
  if (inliers.size() < seed_floor) {return match;}

  // --- The guided search -----------------------------------------------------------
  //
  // Every candidate landmark, through the seed pose, against every query corner near
  // where it lands. One-to-one on the query side: a corner claimed by two landmarks
  // keeps the closer descriptor.
  const double radius = config.guided_radius_px / std::max(1.0, config.focal_px);
  std::vector<cv::Point2d> query_plane(query.bearings.size(), cv::Point2d(1e9, 1e9));
  for (std::size_t q = 0; q < query.bearings.size(); ++q) {
    const cv::Vec3d & b = query.bearings[q];
    if (b[2] > 1e-6) {query_plane[q] = cv::Point2d(b[0] / b[2], b[1] / b[2]);}
  }
  const int n_query = std::min<int>(query.descriptors.rows, static_cast<int>(query.bearings.size()));
  std::vector<int> claimed_by(static_cast<std::size_t>(std::max(0, n_query)), -1);
  std::vector<int> claimed_distance(claimed_by.size(), 1 << 30);
  const cv::Affine3d seed = match.query_from_candidate;
  for (std::size_t li = 0; li < candidate.landmarks.size() && li < candidate.landmark_row.size(); ++li) {
    const int row = candidate.landmark_row[li];
    if (row < 0 || row >= candidate.descriptors.rows) {continue;}
    const cv::Vec3d p = seed * candidate.landmarks[li];
    if (p[2] <= 1e-6) {continue;}
    const cv::Point2d at(p[0] / p[2], p[1] / p[2]);
    int best_q = -1;
    int best_d = config.guided_max_hamming + 1;
    for (int q = 0; q < n_query; ++q) {
      const cv::Point2d d = query_plane[static_cast<std::size_t>(q)] - at;
      if (d.x * d.x + d.y * d.y > radius * radius) {continue;}
      const int h = static_cast<int>(cv::norm(
        candidate.descriptors.row(row), query.descriptors.row(q), cv::NORM_HAMMING));
      if (h < best_d) {best_d = h; best_q = q;}
    }
    if (best_q < 0 || best_d >= claimed_distance[static_cast<std::size_t>(best_q)]) {continue;}
    claimed_distance[static_cast<std::size_t>(best_q)] = best_d;
    claimed_by[static_cast<std::size_t>(best_q)] = static_cast<int>(li);
  }
  std::vector<cv::Point3d> guided_object;
  std::vector<cv::Point2d> guided_image;
  for (std::size_t q = 0; q < claimed_by.size(); ++q) {
    if (claimed_by[q] < 0) {continue;}
    const cv::Vec3d & p = candidate.landmarks[static_cast<std::size_t>(claimed_by[q])];
    guided_object.emplace_back(p[0], p[1], p[2]);
    guided_image.push_back(query_plane[q]);
  }

  // --- Refine on everything found, then count --------------------------------------
  //
  // Two passes: fit to all the guided pairs, then refit to the ones that pass, so
  // one gross mismatch in the guided set cannot drag the final pose. The count is
  // over the final pose, with the same pixel tolerance as the seed.
  auto count_inliers = [&](const cv::Mat & rv, const cv::Mat & tv, std::vector<int> & which) {
      which.clear();
      if (guided_object.empty()) {return;}
      std::vector<cv::Point2d> projected;
      cv::projectPoints(guided_object, rv, tv, cv::Matx33d::eye(), cv::noArray(), projected);
      for (std::size_t i = 0; i < projected.size(); ++i) {
        const cv::Point2d d = projected[i] - guided_image[i];
        if (d.x * d.x + d.y * d.y <= tolerance * tolerance) {which.push_back(static_cast<int>(i));}
      }
    };
  if (guided_object.size() >= seed_floor) {
    cv::Mat rv = rvec.clone(), tv = tvec.clone();
    std::vector<int> pass;
    for (int round = 0; round < 2; ++round) {
      count_inliers(rv, tv, pass);
      if (pass.size() < seed_floor) {break;}
      std::vector<cv::Point3d> o;
      std::vector<cv::Point2d> im;
      for (int i : pass) {
        o.push_back(guided_object[static_cast<std::size_t>(i)]);
        im.push_back(guided_image[static_cast<std::size_t>(i)]);
      }
      cv::solvePnP(o, im, cv::Matx33d::eye(), cv::noArray(), rv, tv, true, cv::SOLVEPNP_ITERATIVE);
    }
    count_inliers(rv, tv, pass);
    if (pass.size() >= match.ransac_inliers) {
      match.inliers = pass.size();
      match.query_from_candidate = cv::Affine3d(cv::Vec3d(rv), cv::Vec3d(tv));
    } else {
      match.inliers = match.ransac_inliers;
    }
  } else {
    match.inliers = match.ransac_inliers;
  }
  match.odom_query_from_candidate = query.odom_from_camera.inv() * candidate.odom_from_camera;
  match.odom_rotation_disagreement_deg = angle_between(
    match.query_from_candidate.rotation(), match.odom_query_from_candidate.rotation()) * 180.0 / CV_PI;
  return match;
}

/// Track ids two keyframes have in common. Both lists are small (~500) and unsorted,
/// so sort copies and walk them.
std::size_t shared_tracks(const std::vector<std::int32_t> & a, std::vector<std::int32_t> b_sorted)
{
  std::vector<std::int32_t> a_sorted(a);
  std::sort(a_sorted.begin(), a_sorted.end());
  std::size_t n = 0;
  auto i = a_sorted.begin();
  auto j = b_sorted.begin();
  while (i != a_sorted.end() && j != b_sorted.end()) {
    if (*i < *j) {
      ++i;
    } else if (*j < *i) {
      ++j;
    } else {
      ++n;
      ++i;
      ++j;
    }
  }
  return n;
}

}  // namespace

PlaceResult find_place(
  const std::deque<Keyframe> & database, const Keyframe & query, const PlaceConfig & config)
{
  PlaceResult result;
  result.query_stamp_ns = query.stamp_ns;
  const auto gap_ns = static_cast<std::int64_t>(config.min_gap_s * 1e9);

  std::vector<std::int32_t> query_tracks(query.track_ids);
  std::sort(query_tracks.begin(), query_tracks.end());
  std::vector<Scored> scored;
  for (std::size_t i = 0; i < database.size(); ++i) {
    const Keyframe & kf = database[i];
    if (kf.stamp_ns > query.stamp_ns - gap_ns) {
      ++result.too_recent;
      continue;
    }
    if (shared_tracks(kf.track_ids, query_tracks) > config.max_shared_tracks) {
      ++result.still_tracked;
      continue;
    }
    ++result.searched;
    Scored s;
    s.index = i;
    s.pairs = match_descriptors(query.descriptors, kf.descriptors, config);
    if (s.pairs.size() >= config.min_matches) {scored.push_back(std::move(s));}
  }
  // Most matches first; ties to the newer keyframe, so the order does not depend on
  // anything but the data.
  std::sort(scored.begin(), scored.end(), [](const Scored & a, const Scored & b) {
      return a.pairs.size() != b.pairs.size() ? a.pairs.size() > b.pairs.size() : a.index > b.index;
    });
  if (scored.size() > config.candidates) {scored.resize(config.candidates);}

  for (const Scored & s : scored) {
    const PlaceMatch m = verify(database[s.index], query, s.pairs, config);
    if (result.verified == 0 || m.inliers > result.best.inliers) {result.best = m;}
    ++result.verified;
  }
  // `verified`, not the best's stamp, says whether `best` is anything: a stamp of 0 is
  // a real stamp in a dataset, and the first version of this read it as "no
  // candidate" and refused a revisit with 205 inliers.
  result.accepted = result.verified > 0 && result.best.inliers >= config.min_inliers;
  return result;
}

// --- The thread -------------------------------------------------------------------

PlaceRecognizer::PlaceRecognizer(const Config & config)
: config_(config) {}

PlaceRecognizer::~PlaceRecognizer()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  if (worker_.joinable()) {worker_.join();}
}

void PlaceRecognizer::start()
{
  if (worker_.joinable()) {return;}
  worker_ = std::thread([this] {this->run();});
}

void PlaceRecognizer::submit(Keyframe keyframe)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.push_back(std::move(keyframe));
    ++stats_.submitted;
  }
  wake_.notify_one();
}

void PlaceRecognizer::flush()
{
  std::unique_lock<std::mutex> lock(mutex_);
  drained_.wait(lock, [this] {return (pending_.empty() && !busy_) || stop_;});
}

std::vector<PlaceResult> PlaceRecognizer::take_results()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<PlaceResult> out;
  out.swap(results_);
  return out;
}

PlaceRecognizer::Stats PlaceRecognizer::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void PlaceRecognizer::run()
{
  const bool niced = config_.nice == 0 || setpriority(PRIO_PROCESS, 0, config_.nice) == 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.niced = niced;
  }
  for (;;) {
    std::vector<Keyframe> batch;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this] {return stop_ || !pending_.empty();});
      if (stop_) {break;}
      batch.swap(pending_);
      busy_ = true;
      stats_.skipped += batch.size() - 1;
    }
    // The query is searched against the database *before* it joins it, so it can
    // never find itself whatever the gap is set to.
    Keyframe query = std::move(batch.back());
    batch.pop_back();
    for (Keyframe & kf : batch) {database_.push_back(std::move(kf));}
    while (database_.size() > config_.max_keyframes) {database_.pop_front();}

    const auto start = std::chrono::steady_clock::now();
    PlaceResult result = find_place(database_, query, config_.place);
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    database_.push_back(std::move(query));
    while (database_.size() > config_.max_keyframes) {database_.pop_front();}

    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.queries;
      stats_.verified += result.verified;
      if (result.accepted) {++stats_.accepted;}
      stats_.database = database_.size();
      stats_.query_ms.push_back(ms);
      results_.push_back(std::move(result));
      busy_ = false;
    }
    drained_.notify_all();
  }
  drained_.notify_all();
}

}  // namespace pimesh_frontend
