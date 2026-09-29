#ifndef PIMESH_FRONTEND__LOCAL_MAP_MATCH_HPP_
#define PIMESH_FRONTEND__LOCAL_MAP_MATCH_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"
#include "pimesh_backend/map.hpp"

namespace pimesh_frontend
{

struct ProjectionConfig
{
  /// How far from its predicted pixel a map point's match may be. The prediction
  /// comes from a PnP over the track-id matches, so it is good to a few pixels when
  /// that solve succeeded and to however far the camera moved in one depth interval
  /// when it did not.
  double radius_px {12.0};
  /// ORB's Hamming distance, in bits of 256, above which two corners are not the
  /// same corner. 50 is ORB-SLAM's "low" threshold — the strict one.
  int max_hamming {50};
  /// The best candidate must beat the second best by this factor, or the match is
  /// ambiguous and refused. A repeated texture has two candidates at the same
  /// distance, and picking one of them is a coin flip dressed as a match.
  double ratio {0.8};
};

struct ProjectionMatch
{
  /// Index into the `points` handed to search_by_projection().
  std::size_t point {0};
  /// Index into the corners.
  std::size_t corner {0};
  int distance {0};
};

struct ProjectionSearch
{
  std::vector<ProjectionMatch> matches;
  /// Points that projected inside the image — what the map's found ratio counts as
  /// "predicted visible". A point that projected off the sensor was not missed.
  std::vector<std::size_t> in_view;
};

/// Find this frame's corners for map points the tracker has no track id for, by
/// projecting them with a predicted pose and matching descriptors near where they
/// land.
///
/// **This is what the map is for.** Track ids carry a feature from frame to frame
/// while the tracker can see it; the moment it cannot — occluded, blurred, off the
/// edge for a second — the id dies and P7's tracker never sees that landmark again.
/// A map point outlives the id, and this is how it is found again: not by identity
/// but by *being where the map says it is and looking like what the map says it
/// looks like*.
///
/// **Each corner is claimed once and each point once**, closest descriptor first —
/// the same rule the tracker's pooled window follows, for the same reason: without
/// it two map points can both claim one corner, the map then records one corner as
/// two landmarks, and the map's duplicate refusal is the only thing left between
/// that and a PnP with a corner counted twice.
///
/// `skip_point` and `corner_taken` are the ones stage one already matched by track
/// id; neither is considered here.
ProjectionSearch search_by_projection(
  const std::vector<pimesh_backend::PointView> & points,
  const std::vector<std::uint8_t> & skip_point,
  const cv::Affine3d & map_from_camera, const cv::Matx33d & k, const cv::Size & image,
  const std::vector<cv::Point2f> & corners, const cv::Mat & corner_descriptors,
  const std::vector<std::uint8_t> & corner_taken,
  const ProjectionConfig & config);

}  // namespace pimesh_frontend

#endif  // PIMESH_FRONTEND__LOCAL_MAP_MATCH_HPP_
