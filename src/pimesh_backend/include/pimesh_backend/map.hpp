#ifndef PIMESH_BACKEND__MAP_HPP_
#define PIMESH_BACKEND__MAP_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"

namespace pimesh_backend
{

struct BaProblem;
struct BaResult;

using KeyframeId = std::int64_t;
using PointId = std::int64_t;

/// "No map point", as its own spelling. **Not 0**: 0 is the first point's id, and a
/// feature that matched nothing must not read as a feature that matched the first
/// point in the map.
constexpr PointId kNoPoint = -1;
constexpr KeyframeId kNoKeyframe = -1;

/// One point in space, **observed by many keyframes** — the noun this project did
/// not have until P14.
///
/// Before it, a landmark was a pixel pushed out along its ray by the depth network,
/// stored inside the keyframe that saw it, and it died with that keyframe: P7 posed
/// every frame against the newest keyframe's landmarks and nothing else. A
/// `MapPoint` outlives the keyframe that created it, collects every keyframe that
/// later sees it, and can have its position re-solved from all of them.
struct MapPoint
{
  PointId id {kNoPoint};
  /// Where it is, in the map frame: the mean of the newest `reading_window` depth
  /// readings in `readings` — or, only with `use_triangulation`, the triangulated
  /// position once there is one. See Map::Config::reading_window for why the
  /// *newest* reading and not the first.
  cv::Vec3d position {0.0, 0.0, 0.0};
  /// Every observing keyframe's depth reading for this point, **in that keyframe's
  /// optical frame**, scaled onto the map's scale. Only observations whose feature had
  /// a usable depth reading have one. Camera frame rather than map frame so that a
  /// reading moves with its keyframe when bundle adjustment moves the keyframe — a
  /// reading placed in the map at insertion would be left where the old pose put it.
  std::map<KeyframeId, cv::Vec3d> readings;
  /// Bundle adjustment has solved this point's position. From then on a new reading
  /// is a measurement for the next solve, not a replacement for the last one.
  bool optimised {false};
  /// The triangulation, kept whether or not it moves the point: it is the
  /// independent opinion `depth_ratio` is measured from.
  cv::Vec3d triangulated_position {0.0, 0.0, 0.0};
  /// The network's reading, kept and never moved, and where the camera was when it
  /// was taken. The two together are what `|triangulated - network|` is measured
  /// from — the first independent opinion this project has about its depth model.
  cv::Vec3d network_position {0.0, 0.0, 0.0};
  cv::Vec3d origin {0.0, 0.0, 0.0};
  KeyframeId first_keyframe {kNoKeyframe};
  /// A 1x32 CV_8U row: the observation's descriptor that is closest, by median
  /// Hamming distance, to all the others. What a projection search matches against.
  cv::Mat descriptor;
  /// The tracker's id for the feature the most recent keyframe saw. A *hint*, not an
  /// identity: the map point is the identity, and a track id is how the tracker
  /// happens to be following it at the moment.
  std::int32_t track_id {-1};
  /// Every track id that has ever been this point, so a cull can scrub each of them
  /// from the track index rather than leave an entry pointing at a dead point.
  std::vector<std::int32_t> tracks;
  /// keyframe -> the row of that keyframe's features this point is. **One entry per
  /// keyframe, by construction** — a std::map keyed on the keyframe cannot hold two —
  /// and the reverse (one point per feature) is enforced by `insert`.
  std::map<KeyframeId, std::int32_t> observations;
  /// Tracking's opinion: how many frames predicted this point should be visible, and
  /// in how many of them it was actually found. The ratio is ORB-SLAM's point cull.
  std::uint32_t visible {1};
  std::uint32_t found {1};
  bool triangulated {false};
  double triangulation_error_px {0.0};
  /// **Culled, not deleted.** Anything holding a shared_ptr to this — a keyframe's
  /// feature table, a snapshot taken a moment ago — still has a valid object and
  /// reads this flag, rather than dereferencing freed memory because another thread
  /// culled it under the lock.
  bool bad {false};
};

/// One keyframe as the map holds it: a pose, the features, and which map point each
/// feature is.
struct MapKeyframe
{
  KeyframeId id {kNoKeyframe};
  std::int64_t stamp_ns {0};
  /// `map <- camera_optical_frame`.
  cv::Affine3d map_from_camera {cv::Affine3d::Identity()};
  cv::Matx33d k {cv::Matx33d::eye()};
  std::vector<cv::Point2f> pixels;
  std::vector<std::int32_t> track_ids;
  cv::Mat descriptors;
  /// Parallel to `pixels`: the map point each feature is, or null.
  std::vector<std::shared_ptr<MapPoint>> points;
  /// The covisibility graph's edges out of this keyframe: other keyframe -> the
  /// number of map points both observe. Maintained on every observation added and
  /// removed, never recomputed.
  std::map<KeyframeId, int> covisible;
  bool bad {false};
};

/// What a keyframe arrives with from tracking. Every per-feature vector is parallel
/// to `pixels`; `Map::insert` refuses the whole keyframe if they are not, for
/// keypoints_well_formed()'s reason — a ragged input converted as far as it goes
/// pairs features with the wrong points, which is a plausible wrong answer.
struct KeyframeInput
{
  std::int64_t stamp_ns {0};
  cv::Affine3d map_from_camera {cv::Affine3d::Identity()};
  cv::Matx33d k {cv::Matx33d::eye()};
  std::vector<cv::Point2f> pixels;
  std::vector<std::int32_t> track_ids;
  /// N rows of 32 bytes.
  cv::Mat descriptors;
  /// Where the depth network put each feature, in this camera's optical frame, and
  /// whether it had a usable reading at all.
  std::vector<cv::Vec3d> network_points;
  std::vector<std::uint8_t> has_depth;
  /// The map point tracking associated each feature with, or kNoPoint. A claim to be
  /// checked, not a fact: `insert` reprojects the point into this keyframe and
  /// refuses the association if it does not land near the feature.
  std::vector<PointId> matched;
};

struct InsertReport
{
  bool ok {false};
  KeyframeId id {kNoKeyframe};
  /// Features that became an observation of an existing point.
  std::size_t associated {0};
  /// Features that became a new point.
  std::size_t created {0};
  /// Associations refused, by reason. Counted separately because they mean
  /// different things: a duplicate is two features claiming one point (a matcher
  /// bug or a repeated texture), a reprojection refusal is a claim the geometry
  /// disagrees with, and a bad point is one culled since tracking matched it.
  std::size_t refused_duplicate {0};
  std::size_t refused_reprojection {0};
  std::size_t refused_bad {0};
  /// The factor this keyframe's depth map was scaled by before it created points,
  /// and how many associations it was measured from. Exactly 1.0 with 0 pairs when
  /// alignment is off or had too little to go on — the two are told apart by
  /// `scale_pairs`, not by the scale.
  double scale {1.0};
  std::size_t scale_pairs {0};
  /// The ratio measured, before the gain: the breathing between this depth map and
  /// the map, which is what `scale` corrects part of.
  double scale_median {1.0};
};

struct TriangulateReport
{
  std::size_t attempted {0};
  std::size_t triangulated {0};
  std::size_t low_parallax {0};
  std::size_t rejected {0};
  /// Per point triangulated: its mean reprojection error, and the ratio of its
  /// distance from the camera that created it to the network's distance for the same
  /// point. 1.0 would mean the network and the geometry agree exactly.
  std::vector<double> error_px;
  std::vector<double> depth_ratio;
};

struct CullReport
{
  std::size_t points {0};
  std::size_t keyframes {0};
  /// How many were *judged*, whatever the verdict. **A cull that culled nothing and
  /// a cull that never looked have the same `points` and `keyframes`**, and the
  /// plan's first false green is a map that never culls; these are what tell the
  /// two apart.
  std::size_t points_judged {0};
  std::size_t keyframes_judged {0};
};

/// A map point as tracking sees it: a copy, taken under the lock, that stays valid
/// however the map changes after it was taken.
struct PointView
{
  PointId id {kNoPoint};
  cv::Vec3d position {0.0, 0.0, 0.0};
  cv::Mat descriptor;
};

/// The part of the map tracking projects into a frame: the reference keyframe, the
/// keyframes covisible with it, and every live point any of them observes.
struct LocalMap
{
  std::vector<KeyframeId> keyframes;
  std::vector<PointView> points;
};

struct MapStats
{
  std::size_t keyframes {0};
  std::size_t points {0};
  /// Live points observed by at least three keyframes — the plan's measure of a map
  /// that is a map rather than a pile of single sightings.
  std::size_t points_3plus {0};
  std::size_t triangulated {0};
  std::size_t culled_points {0};
  std::size_t culled_keyframes {0};
  std::size_t judged_points {0};
  std::size_t judged_keyframes {0};
  /// Over the live triangulated points *as they stand now*, not every attempt ever
  /// made: a point re-triangulated at each of ten keyframes would otherwise be
  /// counted ten times, and the median would describe how often the busiest points
  /// were revisited rather than the map.
  std::vector<double> triangulation_error_px;
  std::vector<double> depth_ratio;
};

/// Map points and the keyframes that observe them, under **one mutex**.
///
/// **One mutex, and every public method takes it.** Tracking reads the map at the
/// depth rate and (from P15) a backend thread rewrites it at the keyframe rate; a
/// lock per point would make every read a thousand lock acquisitions and every
/// write a lock-ordering question. Tracking never holds a reference into the map
/// across the lock: it asks for a snapshot — `local_map`, `lookup_tracks` — and gets
/// copies. The map is small enough that copying the part in view is cheaper than
/// thinking about it.
///
/// **Culling flags; it does not delete.** A culled point or keyframe is marked `bad`
/// and dropped from the map's indexes, but the object lives until the last
/// shared_ptr to it goes — so a keyframe's feature table can still hold a culled
/// point and simply read the flag. That is what makes culling safe to do in the
/// middle of an operation that is iterating something else.
class Map
{
public:
  struct Config
  {
    /// An association tracking claims is refused if the point reprojects further
    /// than this from the feature in the keyframe it is being added to. Looser than
    /// the PnP inlier threshold because the keyframe pose is the *published* one,
    /// after the low-pass, and it lags the fitted pose by up to a few centimetres.
    double associate_px {4.0};
    /// A triangulation is refused if any view reprojects further than this.
    double triangulate_px {2.0};
    /// Below this parallax a point keeps the network's reading. See triangulate()
    /// for the arithmetic behind 3 degrees.
    double min_parallax_deg {3.0};
    /// A point's position is the mean of this many of its newest depth readings.
    ///
    /// **1, and the reason is a measurement.** The first version kept a point where
    /// the keyframe that created it put it, and tracking against those first
    /// readings was measured on TUM fr1/desk (2026-09-29) at a Sim(3) ATE of
    /// 0.47-0.54 m over three runs against 0.25-0.35 m for P7's tracker — with the
    /// projection search switched *off*, so the only difference was which reading a
    /// track id resolved to. P7 re-reads every landmark from the newest keyframe's
    /// depth map; the network's error depends on viewpoint and distance, and the
    /// newest keyframe is the viewpoint nearest the frame being tracked. With a
    /// window of 1 a map point sits exactly where P7's reference landmark would, so
    /// the map *contains* P7's tracker and adds to it only what P7 did not have.
    std::size_t reading_window {1};
    /// Let a triangulation move the point. Off by default: a triangulated position
    /// is derived from the poses, which were solved against the points, so it has no
    /// scale anchor of its own — the network's reading is the only absolute scale
    /// this pipeline has, and a point that stops using it is a point that can drift.
    bool use_triangulation {false};
    /// Put each new keyframe's depth map on the map's scale before it creates
    /// points, from the median depth ratio over its accepted associations. See
    /// insert() for why a map of many depth maps needs this where P7 did not.
    bool align_scale {true};
    std::size_t align_min_pairs {20};
    /// The fraction of the measured log-ratio applied: 1 corrects fully (and lets the
    /// map's scale random-walk), 0 not at all (and leaves the map a patchwork of
    /// scales). See insert().
    double align_gain {0.5};
    /// The largest correction one keyframe may apply, either way.
    double align_max_step {0.15};
    /// A new point is judged this many keyframes after the one that created it.
    std::size_t cull_after_keyframes {2};
    /// ...and culled then if fewer keyframes than this have observed it.
    std::size_t min_observations {2};
    /// ...or at any time while recent, if tracking found it in fewer than this
    /// fraction of the frames that predicted it.
    double min_found_ratio {0.25};
    /// A keyframe is redundant, and culled, when at least this fraction of its points
    /// are observed by at least `redundant_observers` *other* keyframes.
    double redundant_fraction {0.9};
    std::size_t redundant_observers {3};
    /// The local map: the reference and up to this many of its most covisible
    /// keyframes...
    std::size_t local_keyframes {10};
    /// ...sharing at least this many points with it.
    int min_covisibility {15};
  };

  explicit Map(const Config & config)
  : config_(config) {}

  /// Add a keyframe: its associations become observations (each checked), its
  /// unassociated features with a depth reading become new points, and the
  /// covisibility graph is updated. Does not triangulate or cull — those are
  /// separate steps so a caller can time and count them separately, and so the
  /// backend thread in P15 can run them without tracking waiting.
  InsertReport insert(const KeyframeInput & input);

  /// Re-solve the position of every point this keyframe observes that has at least
  /// two observations, from all of them.
  TriangulateReport triangulate(KeyframeId keyframe);

  /// Cull recent points that were not re-observed or not found, and keyframes in
  /// this one's covisible set that other keyframes already describe. The first
  /// keyframe and `keyframe` itself are never culled.
  CullReport cull(KeyframeId keyframe);

  /// The reference keyframe, its most covisible neighbours and every live point
  /// they observe. If the reference has been culled, the newest keyframe stands in
  /// for it rather than returning nothing.
  LocalMap local_map(KeyframeId reference) const;

  /// The bundle-adjustment window around `newest`: it and up to `window - 1` of its
  /// most covisible keyframes, free; every other keyframe that sees their points, up
  /// to `max_fixed`, held fixed; and every observation between them. A snapshot, so
  /// it is solved without the lock.
  BaProblem ba_window(KeyframeId newest, std::size_t window, std::size_t max_fixed) const;

  struct ApplyReport
  {
    std::size_t keyframes {0};
    std::size_t points {0};
    std::size_t observations_dropped {0};
  };
  /// Write a solution back: poses of free keyframes and positions of points that
  /// still exist, and drop the observations the solve called outliers. Anything
  /// culled while the solve ran is skipped, not resurrected.
  ApplyReport apply_ba(const BaProblem & problem, const BaResult & result);

  /// The live point each track id currently belongs to, parallel to `track_ids`,
  /// `id == kNoPoint` where there is none.
  std::vector<PointView> lookup_tracks(const std::vector<std::int32_t> & track_ids) const;

  /// Tracking's report on one frame: the points it predicted would be in view, and
  /// the ones it actually used. Feeds the found-ratio cull.
  void record_tracking(const std::vector<PointId> & visible, const std::vector<PointId> & found);

  MapStats stats() const;
  KeyframeId newest() const;
  std::vector<cv::Vec3d> positions() const;
  void clear();

private:
  // All of these assume mutex_ is held.
  void add_observation_(const std::shared_ptr<MapPoint> & point, MapKeyframe & keyframe, std::int32_t row);
  void remove_observation_(MapPoint & point, KeyframeId keyframe);
  void cull_point_(const std::shared_ptr<MapPoint> & point);
  void cull_keyframe_(MapKeyframe & keyframe);
  void refresh_descriptor_(MapPoint & point) const;
  void update_position_(MapPoint & point) const;
  KeyframeId resolve_reference_(KeyframeId reference) const;

  Config config_;
  mutable std::mutex mutex_;
  std::map<KeyframeId, std::shared_ptr<MapKeyframe>> keyframes_;
  std::unordered_map<PointId, std::shared_ptr<MapPoint>> points_;
  /// track id -> the point it currently belongs to. Rewritten whenever a keyframe
  /// associates or creates a point for a track, and scrubbed when a point is culled.
  std::unordered_map<std::int32_t, std::shared_ptr<MapPoint>> tracks_;
  /// Points young enough to be judged by the recent-point cull.
  std::deque<std::shared_ptr<MapPoint>> recent_;
  KeyframeId next_keyframe_ {0};
  PointId next_point_ {0};
  KeyframeId first_keyframe_ {kNoKeyframe};
  std::size_t culled_points_ {0};
  std::size_t culled_keyframes_ {0};
  std::size_t judged_points_ {0};
  std::size_t judged_keyframes_ {0};
};

/// Hamming distance between two 32-byte descriptor rows.
int hamming(const cv::Mat & a, const cv::Mat & b);

}  // namespace pimesh_backend

#endif  // PIMESH_BACKEND__MAP_HPP_
