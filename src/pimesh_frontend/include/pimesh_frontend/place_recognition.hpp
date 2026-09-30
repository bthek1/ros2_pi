#ifndef PIMESH_FRONTEND__PLACE_RECOGNITION_HPP_
#define PIMESH_FRONTEND__PLACE_RECOGNITION_HPP_

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"
#include "pimesh_backend/pose_graph.hpp"
#include "pimesh_frontend/keyframe_store.hpp"

namespace pimesh_frontend
{

/// Place recognition (#12's P16): the keyframe store's second reader.
///
/// Every earlier stage asks "where is the camera relative to where it just was".
/// This asks "has the camera been *here* before" — of every keyframe, not the
/// newest — and it is the only question whose answer can remove drift rather than
/// bound it per step.
///
/// **The whole difficulty is refusing.** A false closure does not degrade a map, it
/// destroys it: the pose graph will pull two different places together with all the
/// confidence of a real loop. And a descriptor match has no opinion on whether two
/// views are the same place — repeated texture, a second identical monitor, the
/// same poster seen from across the room all match. That is P7's `solvePnPRansac`
/// lesson one level up, so a candidate is believed only after **geometry** agrees:
/// the candidate's depth-backed 3D landmarks, projected through a single rigid pose,
/// have to land on the query's corners.
struct PlaceConfig
{
  /// A candidate must be at least this much older than the query. Anything younger
  /// is odometry's business — the tracker is still following it — and "recognising"
  /// the view from half a second ago is a closure of zero length that says nothing.
  double min_gap_s {3.0};
  /// **And a candidate the tracker still connects to the query is not a revisit.**
  /// Features tracked continuously from one keyframe to the next keep their track
  /// ids, so a candidate sharing more than this many ids with the query is a view
  /// odometry already relates — ORB-SLAM excludes the query's covisible keyframes
  /// for the same reason. Measured on bags/desk1, 2026-09-30, with the time gap
  /// alone: 7 of 16 queries "closed" onto the keyframe just before them, 2-5 s
  /// back, because desk1's keyframes arrive that far apart. Correct as places,
  /// useless as loops: an edge the odometry chain already has.
  std::size_t max_shared_tracks {5};
  /// A descriptor match: Hamming distance at most this many bits (of 256), and the
  /// best at most `ratio` of the second best — Lowe's test, which throws away a
  /// corner that matches two places about equally well, the repeated-texture case.
  int max_hamming {50};
  double ratio {0.8};
  /// How many of the best-scoring candidates are geometrically verified. Verifying
  /// is the expensive half, so only the top few by match count get it.
  std::size_t candidates {3};
  /// Descriptor matches a candidate needs before it is worth a PnP.
  std::size_t min_matches {25};
  /// Inliers the RANSAC pose needs to be worth refining. Only a seed: what decides
  /// is `min_inliers`, counted after the guided search below.
  std::size_t min_ransac_inliers {12};
  /// **Guided matching**, ORB-SLAM's second look. With a pose in hand every one of the
  /// candidate's landmarks is projected into the query and matched against the
  /// corners within `guided_radius_px` of where it lands, at a looser Hamming limit
  /// than the blind search could afford — the geometry now does the rejecting the
  /// ratio test did. A right pose finds many more; a wrong one projects onto
  /// nothing. Measured before it existed, 2026-09-30: bags/walk1's revisit of its
  /// own start was the best candidate at 15-18 RANSAC inliers while bags/desk1's
  /// best refused candidates sat at 22-27, so no threshold on the blind count could
  /// accept the one and refuse the other.
  double guided_radius_px {8.0};
  int guided_max_hamming {64};
  /// **The acceptance test.** Inliers after the guided search and a refinement over
  /// everything it found, and their reprojection tolerance in pixels. `focal_px`
  /// converts pixels into the normalised image plane the bearings live in.
  std::size_t min_inliers {30};
  double reprojection_px {3.0};
  double focal_px {500.0};
  int ransac_iterations {300};
};

/// One verified candidate.
struct PlaceMatch
{
  std::int64_t candidate_stamp_ns {0};
  /// Descriptor matches, and how many of those had a landmark in the candidate —
  /// only those can take part in the PnP.
  std::size_t matches {0};
  std::size_t with_landmark {0};
  /// Inliers of the RANSAC seed, and of the refined pose after the guided search.
  /// `inliers` is the one acceptance is decided on; the gap between the two is what
  /// the guided search bought.
  std::size_t ransac_inliers {0};
  std::size_t inliers {0};
  /// Maps a point in the candidate's optical frame into the query's. The edge a pose
  /// graph (P17) will carry between the two keyframes.
  cv::Affine3d query_from_candidate {cv::Affine3d::Identity()};
  /// How far the closure's relative rotation is from odometry's for the same two
  /// keyframes, in degrees. Odometry drifts in translation far faster than in
  /// rotation, so a closure that disagrees with it by tens of degrees is either a
  /// catastrophic drift or a wrong place. Measured and logged; whether it should
  /// refuse anything is gates/place.sh's question.
  double odom_rotation_disagreement_deg {-1.0};
  /// Odometry's own `query_from_candidate` for the same two keyframes — what a pose
  /// graph would believe without this closure. Logged so gates/place.sh can ask the
  /// question P17 depends on: is the closure closer to the truth than odometry is?
  cv::Affine3d odom_query_from_candidate {cv::Affine3d::Identity()};
};

struct PlaceResult
{
  std::int64_t query_stamp_ns {0};
  /// Keyframes descriptor-scored, and keyframes skipped for being inside the gap.
  std::size_t searched {0};
  std::size_t too_recent {0};
  /// Keyframes skipped because the tracker still connects them to the query.
  std::size_t still_tracked {0};
  /// PnPs attempted.
  std::size_t verified {0};
  /// The best candidate by inliers among those verified — reported **whether or not**
  /// it was accepted, because the margin between the best refused candidate and the
  /// threshold is the number that says how close this came to a false closure.
  /// Meaningful only when `verified > 0` — not when its stamp is non-zero, because 0
  /// is a legitimate stamp and "nothing verified" must not share its spelling.
  PlaceMatch best;
  bool accepted {false};
};

/// Search `database` for the place `query` shows. Pure: no thread, no lock, no
/// state. The query need not be in the database, and if it is, the gap excludes it.
PlaceResult find_place(
  const std::deque<Keyframe> & database, const Keyframe & query, const PlaceConfig & config);

/// The search, on its own thread, over its own copy of the keyframes.
///
/// **Two queues with two different drop rules, and the difference is the point.** A
/// keyframe added to the database is never dropped — a keyframe missing from it is a
/// place that can never be recognised, a hole rather than a missed update. A
/// *query* may be skipped when the thread falls behind: that is one missed chance
/// to notice a revisit that the next keyframe will get again. So a backlog is
/// appended in full and only its newest keyframe is queried, and the skips are
/// counted.
///
/// Niced, per thread, for mesh_node's reason: a CPU-heavy thread at equal priority
/// took depth_node from 17.8 to 14.6 Hz with its per-frame cost unchanged.
class PlaceRecognizer
{
public:
  struct Config
  {
    PlaceConfig place;
    /// The database's ceiling; the oldest keyframe goes first. The same bound as the
    /// keyframe store's, so the two describe the same history.
    std::size_t max_keyframes {500};
    int nice {10};
    /// #12's P17: add each accepted closure to a pose graph over every keyframe and
    /// optimise it, so `map_from_odom()` stops being identity. false is P17's control
    /// — the same search, the same graph with no loop edge, which is the odometry
    /// chain exactly.
    bool close_loops {false};
    pimesh_backend::PoseGraphConfig graph;
  };

  struct Stats
  {
    std::uint64_t submitted {0};
    std::uint64_t queries {0};
    /// Queries skipped because a newer keyframe was already waiting.
    std::uint64_t skipped {0};
    std::uint64_t accepted {0};
    std::uint64_t verified {0};
    std::size_t database {0};
    std::vector<double> query_ms;
    bool niced {false};
    /// The pose graph: loop edges added, solves run, closures the solved graph could
    /// not reconcile, and the last solve's correction at the newest keyframe.
    std::uint64_t loops {0};
    std::uint64_t solves {0};
    std::uint64_t inconsistent {0};
    std::vector<double> solve_ms;
    double correction_m {0.0};
    double correction_deg {0.0};
  };

  explicit PlaceRecognizer(const Config & config);
  ~PlaceRecognizer();
  PlaceRecognizer(const PlaceRecognizer &) = delete;
  PlaceRecognizer & operator=(const PlaceRecognizer &) = delete;

  void start();
  /// Never blocks on a search and never refuses.
  void submit(Keyframe keyframe);
  /// Block until everything submitted has been added and the backlog's query run.
  void flush();
  /// Every query's result since the last call, in order. The caller logs them.
  std::vector<PlaceResult> take_results();
  Stats stats() const;
  /// `map <- odom` as of the last solve; identity until a closure is optimised, and
  /// always identity with `close_loops` off.
  cv::Affine3d map_from_odom() const;
  /// Every keyframe's stamp and pose in `map`, for the gate's ATE. Call after
  /// flush() for the whole session.
  std::vector<std::pair<std::int64_t, cv::Affine3d>> trajectory() const;
  /// Every keyframe's `map <- odom` correction as of the last solve, in order —
  /// PoseGraph::corrections(), copied out under the lock. What P18's rebuild reads.
  std::vector<std::pair<std::int64_t, cv::Affine3d>> corrections() const;

private:
  void run();

  Config config_;
  std::deque<Keyframe> database_;
  /// Worker-thread only, except through the two accessors, which copy under mutex_.
  pimesh_backend::PoseGraph graph_;
  cv::Affine3d map_from_odom_ {cv::Affine3d::Identity()};
  std::vector<std::pair<std::int64_t, cv::Affine3d>> trajectory_;
  std::vector<std::pair<std::int64_t, cv::Affine3d>> corrections_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::condition_variable drained_;
  std::vector<Keyframe> pending_;
  std::vector<PlaceResult> results_;
  Stats stats_;
  bool busy_ {false};
  bool stop_ {false};
  std::thread worker_;
};

}  // namespace pimesh_frontend

#endif  // PIMESH_FRONTEND__PLACE_RECOGNITION_HPP_
