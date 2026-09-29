#include "pimesh_backend/map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "pimesh_backend/local_ba.hpp"
#include "pimesh_backend/triangulation.hpp"

namespace pimesh_backend
{

int hamming(const cv::Mat & a, const cv::Mat & b)
{
  return static_cast<int>(cv::norm(a, b, cv::NORM_HAMMING));
}

InsertReport Map::insert(const KeyframeInput & input)
{
  InsertReport report;
  const std::size_t n = input.pixels.size();
  // **The whole keyframe, or none of it.** Every per-feature vector is parallel to
  // the pixels by contract, and nothing but this line checks the contract.
  if (input.track_ids.size() != n || input.network_points.size() != n ||
    input.has_depth.size() != n || input.matched.size() != n ||
    static_cast<std::size_t>(input.descriptors.rows) != n ||
    (n > 0 && input.descriptors.type() != CV_8U))
  {
    return report;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  auto keyframe = std::make_shared<MapKeyframe>();
  keyframe->id = next_keyframe_++;
  keyframe->stamp_ns = input.stamp_ns;
  keyframe->map_from_camera = input.map_from_camera;
  keyframe->k = input.k;
  keyframe->pixels = input.pixels;
  keyframe->track_ids = input.track_ids;
  keyframe->descriptors = input.descriptors.clone();
  keyframe->points.resize(n);
  keyframes_[keyframe->id] = keyframe;
  if (first_keyframe_ == kNoKeyframe) {first_keyframe_ = keyframe->id;}
  report.id = keyframe->id;
  report.ok = true;

  // --- Pass one: the associations tracking claims -----------------------------------
  //
  // Decided before any point is created, because the accepted ones are also what the
  // new depth map's scale is measured against — see pass two.
  std::vector<std::uint8_t> create(n, 0);
  std::vector<double> ratios;
  std::vector<std::pair<std::size_t, std::shared_ptr<MapPoint>>> accepted;
  for (std::size_t i = 0; i < n; ++i) {
    const auto row = static_cast<std::int32_t>(i);
    const std::int32_t track = input.track_ids[i];
    create[i] = input.has_depth[i];
    if (input.matched[i] == kNoPoint) {continue;}

    const auto found = points_.find(input.matched[i]);
    if (found == points_.end() || found->second->bad) {
      // Culled since tracking matched it. The feature is still a feature, so it may
      // become a point of its own.
      ++report.refused_bad;
      continue;
    }
    if (found->second->observations.count(keyframe->id) != 0) {
      // **A second feature in this keyframe claiming a point the first already
      // took.** Refused outright, and it does *not* become a new point: two features
      // claiming one landmark is a repeated texture or a matcher error, and a new
      // point here would be a duplicate of the one refused.
      ++report.refused_duplicate;
      create[i] = 0;
      continue;
    }
    const std::shared_ptr<MapPoint> & point = found->second;
    cv::Point2d where;
    const bool in_front = project(input.map_from_camera, input.k, point->position, where);
    if (!in_front ||
      std::hypot(where.x - input.pixels[i].x, where.y - input.pixels[i].y) > config_.associate_px)
    {
      // The geometry disagrees with the claim. The feature is most likely something
      // else, so it may become its own point.
      ++report.refused_reprojection;
      continue;
    }
    add_observation_(point, *keyframe, row);
    point->track_id = track;
    if (std::find(point->tracks.begin(), point->tracks.end(), track) == point->tracks.end()) {
      point->tracks.push_back(track);
    }
    tracks_[track] = point;
    ++report.associated;
    create[i] = 0;
    accepted.emplace_back(i, point);
    if (input.has_depth[i] && input.network_points[i][2] > 0.0) {
      const double z_map = (input.map_from_camera.inv() * point->position)[2];
      ratios.push_back(z_map / input.network_points[i][2]);
    }
  }

  // --- The new depth map, on the map's scale --------------------------------------
  //
  // **The same fact as ScaleHandling::DivideOut in rgbd_odometry.hpp, arriving at the
  // map.** Depth Anything V2's scale breathes a few percent from frame to frame. P7
  // never saw it as an inconsistency because it posed each frame against *one*
  // keyframe's points, all from one depth map; a map holds points from many, and a
  // 4% difference between two keyframes' scales at 3 m is 12 cm of disagreement
  // inside the map that every PnP against it has to split. The camera pose was
  // solved against the map, so it is on the map's scale already; the points this
  // keyframe is about to create are on *its depth map's*. The associated points say
  // what the ratio is — their depth as the new camera sees them, against the
  // network's reading for the same features — and the median of it is applied to
  // every point created below.
  if (config_.align_scale && ratios.size() >= config_.align_min_pairs) {
    std::nth_element(
      ratios.begin(), ratios.begin() + static_cast<std::ptrdiff_t>(ratios.size() / 2), ratios.end());
    const double median = ratios[ratios.size() / 2];
    // **Part of the way, never all of it — and this is the line the whole map's scale
    // hangs on.** Correcting fully makes the new points agree with the map exactly,
    // which sounds like the goal and is a random walk: the map's scale is then
    // inherited keyframe to keyframe, each inheritance carries the error of the
    // median it was measured with, and nothing ever pulls it back. Measured on TUM
    // fr1/desk, 2026-09-29: full correction finished one run with a fitted Sim(3)
    // scale of **0.699** where every other run sat at 0.99-1.06 — 30% of walk over
    // 33 keyframes. Not correcting at all has no walk, because every keyframe is
    // re-anchored to the network's own scale, which is noisy but absolute; it leaves
    // the map a patchwork of scales that differ by ~10% keyframe to keyframe.
    //
    // A gain below 1 is a high-pass in log-scale: the breathing between neighbouring
    // keyframes is mostly removed, and the network's absolute scale still wins over
    // time, so the map's scale is mean-reverting rather than a walk. It is exactly
    // the property test_scale_aligner pins on fusion_node's aligner — corrections
    // whose product stays bounded under a constant bias — arriving at a second
    // consumer of the same network.
    const double corrected = std::exp(config_.align_gain * std::log(median));
    // Clamped: a ratio far from 1 is a keyframe tracked badly rather than a network
    // that breathed, and scaling a whole depth map by it would spread one bad pose
    // over every point it creates.
    report.scale = std::clamp(corrected, 1.0 - config_.align_max_step, 1.0 + config_.align_max_step);
    report.scale_median = median;
    report.scale_pairs = ratios.size();
  }

  // --- This keyframe's readings of the points it re-observed ------------------------
  //
  // After the scale is known, so they go in on the map's scale like everything this
  // keyframe creates.
  for (const auto & [i, point] : accepted) {
    if (!input.has_depth[i]) {continue;}
    point->readings[keyframe->id] = report.scale * input.network_points[i];
    update_position_(*point);
  }

  // --- Pass two: every unclaimed feature with a depth reading becomes a point -------
  for (std::size_t i = 0; i < n; ++i) {
    if (!create[i]) {continue;}
    const auto row = static_cast<std::int32_t>(i);
    const std::int32_t track = input.track_ids[i];
    auto point = std::make_shared<MapPoint>();
    point->id = next_point_++;
    // `network_position` is what the network said, unscaled — it is the reference the
    // depth ratio is measured against, and aligning it would hide the very breathing
    // that ratio exists to show.
    point->network_position = input.map_from_camera * input.network_points[i];
    point->position = input.map_from_camera * (report.scale * input.network_points[i]);
    point->readings[keyframe->id] = report.scale * input.network_points[i];
    point->origin = input.map_from_camera.translation();
    point->first_keyframe = keyframe->id;
    point->track_id = track;
    point->tracks.push_back(track);
    points_[point->id] = point;
    add_observation_(point, *keyframe, row);
    // The track now belongs to this point, whatever it belonged to before: a track
    // whose previous point was refused above is a track the geometry has just said
    // is not that point.
    tracks_[track] = point;
    recent_.push_back(point);
    ++report.created;
  }
  return report;
}

void Map::add_observation_(
  const std::shared_ptr<MapPoint> & point, MapKeyframe & keyframe, std::int32_t row)
{
  // The covisibility edges first, against every keyframe that already sees it.
  for (const auto & [other, unused] : point->observations) {
    (void)unused;
    if (other == keyframe.id) {continue;}
    const auto it = keyframes_.find(other);
    if (it == keyframes_.end()) {continue;}
    ++keyframe.covisible[other];
    ++it->second->covisible[keyframe.id];
  }
  point->observations[keyframe.id] = row;
  keyframe.points[static_cast<std::size_t>(row)] = point;
  refresh_descriptor_(*point);
}

void Map::remove_observation_(MapPoint & point, KeyframeId keyframe_id)
{
  const auto obs = point.observations.find(keyframe_id);
  if (obs == point.observations.end()) {return;}
  const std::int32_t row = obs->second;
  point.observations.erase(obs);
  if (point.readings.erase(keyframe_id) != 0 && !point.readings.empty()) {update_position_(point);}

  const auto here = keyframes_.find(keyframe_id);
  if (here != keyframes_.end()) {
    auto & slot = here->second->points[static_cast<std::size_t>(row)];
    if (slot.get() == &point) {slot.reset();}
  }
  for (const auto & [other, unused] : point.observations) {
    (void)unused;
    const auto there = keyframes_.find(other);
    if (here != keyframes_.end()) {
      auto & edges = here->second->covisible;
      const auto e = edges.find(other);
      if (e != edges.end() && --e->second <= 0) {edges.erase(e);}
    }
    if (there != keyframes_.end()) {
      auto & edges = there->second->covisible;
      const auto e = edges.find(keyframe_id);
      if (e != edges.end() && --e->second <= 0) {edges.erase(e);}
    }
  }
}

void Map::cull_point_(const std::shared_ptr<MapPoint> & point)
{
  if (point->bad) {return;}
  point->bad = true;
  while (!point->observations.empty()) {
    remove_observation_(*point, point->observations.begin()->first);
  }
  for (std::int32_t track : point->tracks) {
    const auto it = tracks_.find(track);
    if (it != tracks_.end() && it->second == point) {tracks_.erase(it);}
  }
  points_.erase(point->id);
  ++culled_points_;
}

void Map::cull_keyframe_(MapKeyframe & keyframe)
{
  if (keyframe.bad || keyframe.id == first_keyframe_) {return;}
  for (std::size_t row = 0; row < keyframe.points.size(); ++row) {
    const std::shared_ptr<MapPoint> point = keyframe.points[row];
    if (!point || point->bad) {continue;}
    remove_observation_(*point, keyframe.id);
    // A point nothing observes any more is not a point.
    if (point->observations.empty()) {cull_point_(point);}
  }
  keyframe.bad = true;
  keyframe.covisible.clear();
  ++culled_keyframes_;
  keyframes_.erase(keyframe.id);
}

void Map::update_position_(MapPoint & point) const
{
  if (config_.use_triangulation && point.triangulated) {
    point.position = point.triangulated_position;
    return;
  }
  if (point.optimised || point.readings.empty()) {return;}
  cv::Vec3d sum(0.0, 0.0, 0.0);
  std::size_t used = 0;
  for (auto it = point.readings.rbegin();
    it != point.readings.rend() && used < std::max<std::size_t>(1, config_.reading_window); ++it)
  {
    const auto kf = keyframes_.find(it->first);
    if (kf == keyframes_.end()) {continue;}
    sum += kf->second->map_from_camera * it->second;
    ++used;
  }
  if (used > 0) {point.position = sum * (1.0 / static_cast<double>(used));}
}

void Map::refresh_descriptor_(MapPoint & point) const
{
  // The observation whose descriptor has the smallest median distance to all the
  // others — ORB-SLAM's choice, and it is a *choice of one real descriptor*, never
  // an average: the bitwise mean of two ORB descriptors describes neither corner.
  // Over the newest eight observations, which bounds the cost at 28 comparisons
  // and follows a point whose appearance drifts as the viewpoint does.
  std::vector<cv::Mat> rows;
  for (auto it = point.observations.rbegin(); it != point.observations.rend() && rows.size() < 8;
    ++it)
  {
    const auto kf = keyframes_.find(it->first);
    if (kf == keyframes_.end()) {continue;}
    rows.push_back(kf->second->descriptors.row(it->second));
  }
  if (rows.empty()) {return;}
  if (rows.size() == 1) {
    point.descriptor = rows.front().clone();
    return;
  }
  std::size_t best = 0;
  int best_median = std::numeric_limits<int>::max();
  for (std::size_t i = 0; i < rows.size(); ++i) {
    std::vector<int> distances;
    for (std::size_t j = 0; j < rows.size(); ++j) {
      if (i != j) {distances.push_back(hamming(rows[i], rows[j]));}
    }
    std::nth_element(
      distances.begin(), distances.begin() + static_cast<std::ptrdiff_t>(distances.size() / 2),
      distances.end());
    const int median = distances[distances.size() / 2];
    if (median < best_median) {
      best_median = median;
      best = i;
    }
  }
  point.descriptor = rows[best].clone();
}

TriangulateReport Map::triangulate(KeyframeId keyframe_id)
{
  TriangulateReport report;
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = keyframes_.find(keyframe_id);
  if (it == keyframes_.end()) {return report;}
  const double min_parallax = config_.min_parallax_deg * CV_PI / 180.0;

  for (const std::shared_ptr<MapPoint> & point : it->second->points) {
    if (!point || point->bad || point->observations.size() < 2) {continue;}
    std::vector<View> views;
    views.reserve(point->observations.size());
    for (const auto & [kf_id, row] : point->observations) {
      const auto kf = keyframes_.find(kf_id);
      if (kf == keyframes_.end()) {continue;}
      views.push_back(
        View{kf->second->map_from_camera, kf->second->k,
          kf->second->pixels[static_cast<std::size_t>(row)]});
    }
    ++report.attempted;
    const Triangulation t = pimesh_backend::triangulate(views, min_parallax, config_.triangulate_px);
    if (!t.ok) {
      if (std::string(t.refusal) == "parallax") {
        ++report.low_parallax;
      } else {
        ++report.rejected;
      }
      continue;
    }
    point->triangulated_position = t.point;
    point->triangulated = true;
    update_position_(*point);
    point->triangulation_error_px = t.mean_error_px;
    ++report.triangulated;
    report.error_px.push_back(t.mean_error_px);
    const double network = cv::norm(point->network_position - point->origin);
    if (network > 0.0) {
      report.depth_ratio.push_back(cv::norm(t.point - point->origin) / network);
    }
  }
  return report;
}

CullReport Map::cull(KeyframeId keyframe_id)
{
  CullReport report;
  std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t before_points = culled_points_;
  const std::size_t before_keyframes = culled_keyframes_;

  // --- Recent points ------------------------------------------------------------
  //
  // **Judged while young, then left alone.** A point is on probation for a couple of
  // keyframes after it is created: if tracking kept failing to find it where it
  // should be, or no later keyframe saw it, it goes. After that it is established
  // and only a keyframe cull (or, from P15, bundle adjustment) removes it. The
  // alternative — judging every point every keyframe — culls the far side of the
  // room the moment the camera turns away from it, which is the map forgetting
  // exactly what it exists to remember.
  std::deque<std::shared_ptr<MapPoint>> keep;
  for (const std::shared_ptr<MapPoint> & point : recent_) {
    if (point->bad) {continue;}
    ++report.points_judged;
    const KeyframeId age = keyframe_id - point->first_keyframe;
    const double ratio = static_cast<double>(point->found) / static_cast<double>(point->visible);
    if (ratio < config_.min_found_ratio) {
      cull_point_(point);
    } else if (age >= static_cast<KeyframeId>(config_.cull_after_keyframes) &&
      point->observations.size() < config_.min_observations)
    {
      cull_point_(point);
    } else if (age <= static_cast<KeyframeId>(config_.cull_after_keyframes)) {
      keep.push_back(point);
    }
  }
  recent_ = std::move(keep);

  // --- Redundant keyframes ------------------------------------------------------
  //
  // Among the keyframes covisible with this one, any whose points are almost all
  // seen by three or more *other* keyframes adds nothing the map does not already
  // have — and a map that keeps every keyframe makes every later covisibility
  // query, projection search and (from P15) bundle adjustment pay for them.
  const auto self = keyframes_.find(keyframe_id);
  if (self != keyframes_.end()) {
    std::vector<KeyframeId> candidates;
    for (const auto & [other, weight] : self->second->covisible) {
      (void)weight;
      candidates.push_back(other);
    }
    for (KeyframeId candidate : candidates) {
      if (candidate == first_keyframe_ || candidate == keyframe_id) {continue;}
      const auto kf = keyframes_.find(candidate);
      if (kf == keyframes_.end()) {continue;}
      ++report.keyframes_judged;
      std::size_t live = 0;
      std::size_t redundant = 0;
      for (const std::shared_ptr<MapPoint> & point : kf->second->points) {
        if (!point || point->bad) {continue;}
        ++live;
        if (point->observations.size() >= config_.redundant_observers + 1) {++redundant;}
      }
      if (live > 0 && static_cast<double>(redundant) >= config_.redundant_fraction * live) {
        cull_keyframe_(*kf->second);
      }
    }
  }

  report.points = culled_points_ - before_points;
  report.keyframes = culled_keyframes_ - before_keyframes;
  judged_points_ += report.points_judged;
  judged_keyframes_ += report.keyframes_judged;
  return report;
}

BaProblem Map::ba_window(KeyframeId newest, std::size_t window, std::size_t max_fixed) const
{
  BaProblem problem;
  std::lock_guard<std::mutex> lock(mutex_);
  const KeyframeId ref = resolve_reference_(newest);
  if (ref == kNoKeyframe) {return problem;}
  const MapKeyframe & kf = *keyframes_.at(ref);

  // --- The free keyframes: the newest and its most covisible -----------------------
  std::vector<std::pair<int, KeyframeId>> neighbours;
  for (const auto & [other, weight] : kf.covisible) {neighbours.emplace_back(weight, other);}
  std::sort(neighbours.begin(), neighbours.end(), [](const auto & a, const auto & b) {
      return a.first != b.first ? a.first > b.first : a.second > b.second;
    });
  std::map<KeyframeId, std::size_t> index;
  auto add_keyframe = [&](KeyframeId id, bool fixed) {
      const MapKeyframe & k = *keyframes_.at(id);
      index[id] = problem.keyframes.size();
      // The map's first keyframe is its origin and is never moved, whatever window
      // it lands in: moving it would move the frame every pose is expressed in.
      problem.keyframes.push_back(BaKeyframe{id, k.map_from_camera, k.k, fixed || id == first_keyframe_});
    };
  add_keyframe(ref, false);
  for (const auto & [weight, other] : neighbours) {
    (void)weight;
    if (problem.keyframes.size() >= std::max<std::size_t>(1, window)) {break;}
    add_keyframe(other, false);
  }

  // --- The points they see ---------------------------------------------------------
  std::map<PointId, std::size_t> point_index;
  std::vector<std::shared_ptr<MapPoint>> points;
  const std::size_t free_count = problem.keyframes.size();
  for (std::size_t i = 0; i < free_count; ++i) {
    for (const std::shared_ptr<MapPoint> & p : keyframes_.at(problem.keyframes[i].id)->points) {
      if (!p || p->bad || point_index.count(p->id) != 0) {continue;}
      point_index[p->id] = problem.points.size();
      problem.points.push_back(BaPoint{p->id, p->position});
      points.push_back(p);
    }
  }

  // --- The fixed keyframes: everyone else who sees those points, most-shared first -
  std::map<KeyframeId, int> outside;
  for (const std::shared_ptr<MapPoint> & p : points) {
    for (const auto & [id, row] : p->observations) {
      (void)row;
      if (index.count(id) == 0) {++outside[id];}
    }
  }
  std::vector<std::pair<int, KeyframeId>> fixed(outside.size());
  std::transform(outside.begin(), outside.end(), fixed.begin(), [](const auto & e) {
      return std::make_pair(e.second, e.first);
    });
  std::sort(fixed.begin(), fixed.end(), [](const auto & a, const auto & b) {
      return a.first != b.first ? a.first > b.first : a.second > b.second;
    });
  for (std::size_t i = 0; i < fixed.size() && i < max_fixed; ++i) {add_keyframe(fixed[i].second, true);}

  // **A window with nothing fixed floats.** Monocular BA is only defined up to a
  // similarity of the whole window; with no fixed keyframe the solver is free to
  // slide, turn and scale it, and on a map whose older keyframes are all culled or
  // uncovisible that is the case. The oldest keyframe in the window is held instead.
  bool any_fixed = false;
  for (const BaKeyframe & k : problem.keyframes) {any_fixed = any_fixed || k.fixed;}
  if (!any_fixed && problem.keyframes.size() > 1) {
    auto oldest = std::min_element(problem.keyframes.begin(), problem.keyframes.end(),
        [](const BaKeyframe & a, const BaKeyframe & b) {return a.id < b.id;});
    oldest->fixed = true;
  }

  // --- Every observation between them ------------------------------------------------
  for (std::size_t pi = 0; pi < points.size(); ++pi) {
    const MapPoint & p = *points[pi];
    for (const auto & [id, row] : p.observations) {
      const auto k = index.find(id);
      if (k == index.end()) {continue;}
      const MapKeyframe & mk = *keyframes_.at(id);
      BaObservation o;
      o.keyframe = k->second;
      o.point = pi;
      o.row = row;
      o.pixel = mk.pixels[static_cast<std::size_t>(row)];
      const auto reading = p.readings.find(id);
      o.depth = (reading != p.readings.end()) ? reading->second[2] : 0.0;
      problem.observations.push_back(o);
    }
  }
  return problem;
}

Map::ApplyReport Map::apply_ba(const BaProblem & problem, const BaResult & result)
{
  ApplyReport report;
  if (!result.ran || result.map_from_camera.size() != problem.keyframes.size() ||
    result.positions.size() != problem.points.size() ||
    result.outlier.size() != problem.observations.size())
  {
    return report;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t i = 0; i < problem.keyframes.size(); ++i) {
    if (problem.keyframes[i].fixed) {continue;}
    const auto kf = keyframes_.find(problem.keyframes[i].id);
    if (kf == keyframes_.end() || kf->second->bad) {continue;}
    kf->second->map_from_camera = result.map_from_camera[i];
    ++report.keyframes;
  }
  for (std::size_t i = 0; i < problem.points.size(); ++i) {
    const auto p = points_.find(problem.points[i].id);
    if (p == points_.end() || p->second->bad) {continue;}
    p->second->position = result.positions[i];
    p->second->optimised = true;
    ++report.points;
  }
  for (std::size_t i = 0; i < problem.observations.size(); ++i) {
    if (!result.outlier[i]) {continue;}
    const BaObservation & o = problem.observations[i];
    const auto p = points_.find(problem.points[o.point].id);
    if (p == points_.end() || p->second->bad) {continue;}
    const std::shared_ptr<MapPoint> point = p->second;
    const KeyframeId kf = problem.keyframes[o.keyframe].id;
    if (point->observations.count(kf) == 0) {continue;}
    remove_observation_(*point, kf);
    ++report.observations_dropped;
    if (point->observations.empty()) {cull_point_(point);}
  }
  return report;
}

KeyframeId Map::resolve_reference_(KeyframeId reference) const
{
  const auto it = keyframes_.find(reference);
  if (it != keyframes_.end() && !it->second->bad) {return reference;}
  return keyframes_.empty() ? kNoKeyframe : keyframes_.rbegin()->first;
}

LocalMap Map::local_map(KeyframeId reference) const
{
  LocalMap out;
  std::lock_guard<std::mutex> lock(mutex_);
  const KeyframeId ref = resolve_reference_(reference);
  if (ref == kNoKeyframe) {return out;}
  const MapKeyframe & kf = *keyframes_.at(ref);

  std::vector<std::pair<int, KeyframeId>> neighbours;
  for (const auto & [other, weight] : kf.covisible) {
    if (weight >= config_.min_covisibility) {neighbours.emplace_back(weight, other);}
  }
  // Heaviest first, and ties broken towards the newer keyframe so the order — and
  // therefore which keyframes make the cut — does not depend on hash order.
  std::sort(neighbours.begin(), neighbours.end(), [](const auto & a, const auto & b) {
      return a.first != b.first ? a.first > b.first : a.second > b.second;
    });
  out.keyframes.push_back(ref);
  for (const auto & [weight, other] : neighbours) {
    (void)weight;
    if (out.keyframes.size() > config_.local_keyframes) {break;}
    out.keyframes.push_back(other);
  }

  std::unordered_set<PointId> seen;
  for (KeyframeId id : out.keyframes) {
    const auto it = keyframes_.find(id);
    if (it == keyframes_.end()) {continue;}
    for (const std::shared_ptr<MapPoint> & point : it->second->points) {
      if (!point || point->bad || !seen.insert(point->id).second) {continue;}
      out.points.push_back(PointView{point->id, point->position, point->descriptor});
    }
  }
  return out;
}

std::vector<PointView> Map::lookup_tracks(const std::vector<std::int32_t> & track_ids) const
{
  std::vector<PointView> out(track_ids.size());
  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t i = 0; i < track_ids.size(); ++i) {
    const auto it = tracks_.find(track_ids[i]);
    if (it == tracks_.end() || it->second->bad) {continue;}
    out[i] = PointView{it->second->id, it->second->position, it->second->descriptor};
  }
  return out;
}

void Map::record_tracking(const std::vector<PointId> & visible, const std::vector<PointId> & found)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (PointId id : visible) {
    const auto it = points_.find(id);
    if (it != points_.end()) {++it->second->visible;}
  }
  for (PointId id : found) {
    const auto it = points_.find(id);
    if (it != points_.end()) {++it->second->found;}
  }
}

MapStats Map::stats() const
{
  MapStats out;
  std::lock_guard<std::mutex> lock(mutex_);
  out.keyframes = keyframes_.size();
  out.points = points_.size();
  for (const auto & [id, point] : points_) {
    (void)id;
    if (point->observations.size() >= 3) {++out.points_3plus;}
    if (!point->triangulated) {continue;}
    ++out.triangulated;
    out.triangulation_error_px.push_back(point->triangulation_error_px);
    const double network = cv::norm(point->network_position - point->origin);
    if (network > 0.0) {
      out.depth_ratio.push_back(cv::norm(point->triangulated_position - point->origin) / network);
    }
  }
  out.culled_points = culled_points_;
  out.culled_keyframes = culled_keyframes_;
  out.judged_points = judged_points_;
  out.judged_keyframes = judged_keyframes_;
  return out;
}

KeyframeId Map::newest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return keyframes_.empty() ? kNoKeyframe : keyframes_.rbegin()->first;
}

std::vector<cv::Vec3d> Map::positions() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<cv::Vec3d> out;
  out.reserve(points_.size());
  for (const auto & [id, point] : points_) {
    (void)id;
    out.push_back(point->position);
  }
  return out;
}

void Map::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  keyframes_.clear();
  points_.clear();
  tracks_.clear();
  recent_.clear();
  next_keyframe_ = 0;
  next_point_ = 0;
  first_keyframe_ = kNoKeyframe;
  culled_points_ = 0;
  culled_keyframes_ = 0;
  judged_points_ = 0;
  judged_keyframes_ = 0;
}

}  // namespace pimesh_backend
