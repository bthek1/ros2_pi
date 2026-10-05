#ifndef PIMESH_FRONTEND__RELOCALISER_HPP_
#define PIMESH_FRONTEND__RELOCALISER_HPP_

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "opencv2/core/affine.hpp"
#include "pimesh_frontend/keyframe_store.hpp"
#include "pimesh_frontend/place_recognition.hpp"

namespace pimesh_frontend
{

/// Where the query camera is in the saved map, from a verified match.
///
/// `query_from_candidate` maps points in the candidate camera's frame into the query
/// camera's — `verify` projects `seed * candidate.landmarks` onto the query's image.
/// So the candidate camera, seen from the query, is that transform, and the query in
/// the map is the candidate's map pose composed with its **inverse**.
///
/// **A one-line function with its own test, for P7's reason.** Composed the other way
/// it returns a pose that is right whenever the two views coincide and moves the wrong
/// way whenever they do not — which a relocalisation onto a nearly identical view
/// would never show. test_place_recognition pins the direction on an asymmetric pair.
inline cv::Affine3d relocalised_pose(
  const cv::Affine3d & map_from_candidate, const cv::Affine3d & query_from_candidate)
{
  return map_from_candidate * query_from_candidate.inv();
}

/// One query's answer.
struct Relocalisation
{
  std::int64_t query_stamp_ns {0};
  bool accepted {false};
  std::int64_t candidate_stamp_ns {-1};
  std::size_t searched {0};
  std::size_t inliers {0};
  std::size_t matches {0};
  /// The query camera in the saved map. Meaningful only when accepted.
  cv::Affine3d map_from_camera {cv::Affine3d::Identity()};
  /// The query camera in this session's odom, as submitted — the other half of
  /// `map <- odom`, carried with the answer so the caller does not have to remember
  /// which pose it asked about.
  cv::Affine3d odom_from_camera {cv::Affine3d::Identity()};
  double ms {0.0};
};

/// #13's P20: on LOST, search a map saved by an earlier session.
///
/// **P16's search, pointed at someone else's keyframes.** The database is a loaded
/// map whose keyframes carry their pose in that map's frame (`odom_from_camera`, see
/// map_io.hpp); the query is the frame being tracked, with its pose in this session's
/// odom. `find_place` runs with `across_sessions`, so neither stamps nor track ids —
/// both meaningless across two recordings — exclude anything.
///
/// **Its own thread, newest query wins.** A search over a few hundred keyframes is
/// tens of milliseconds and the pose worker has 57; a query submitted while one is
/// running replaces any still waiting, because the freshest frame is the one whose
/// answer is worth having — and a backlog of LOST frames is a backlog of answers that
/// arrive after the camera has moved on.
class Relocaliser
{
public:
  struct Config
  {
    PlaceConfig place;
    /// The search is background work; see PlaceRecognizer::Config::nice.
    int nice {10};
  };

  struct Stats
  {
    std::uint64_t submitted {0};
    std::uint64_t queries {0};
    /// Queries replaced by a newer one before the thread reached them.
    std::uint64_t skipped {0};
    std::uint64_t accepted {0};
    std::vector<double> query_ms;
    bool niced {false};
  };

  /// `map`: keyframes whose `odom_from_camera` is their pose in the saved map.
  Relocaliser(const Config & config, std::deque<Keyframe> map);
  ~Relocaliser();
  Relocaliser(const Relocaliser &) = delete;
  Relocaliser & operator=(const Relocaliser &) = delete;

  void start();
  /// `query.odom_from_camera` is its pose in this session's odom.
  void submit(Keyframe query);
  /// Every answer since the last call, oldest first.
  std::vector<Relocalisation> take_results();
  /// Blocks until nothing is waiting or running. For tests.
  void flush();
  Stats stats() const;
  std::size_t size() const {return map_.size();}

private:
  void run();

  Config config_;
  const std::deque<Keyframe> map_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::condition_variable drained_;
  std::optional<Keyframe> pending_;
  std::vector<Relocalisation> results_;
  Stats stats_;
  bool busy_ {false};
  bool stop_ {false};
  std::thread worker_;
};

}  // namespace pimesh_frontend

#endif  // PIMESH_FRONTEND__RELOCALISER_HPP_
