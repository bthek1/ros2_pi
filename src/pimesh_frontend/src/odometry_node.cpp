// The pose: where the camera is, from corners and a depth map.
//
// Split out of keypoint_node.cpp on 2026-09-23. Everything here ran there from
// P3 (the rotation) and P7 (the 6-DoF solve); what changed is where the corners
// come from — a topic rather than a member — and that this node's stats line is
// its own rather than the second of two rows.

#include "pimesh_frontend/nodes/odometry_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pimesh_core/image_buffer.hpp"
#include "pimesh_frontend/keyframe_store.hpp"
#include "pimesh_frontend/map_io.hpp"
#include "pimesh_frontend/tum_trajectory.hpp"
#include "pimesh_frontend/keypoints_view.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "pimesh_core/stats.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

using pimesh_core::depth_mat_over;

namespace pimesh_frontend
{
namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & text)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = text;
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describe_int(
  const std::string & text, std::int64_t low, std::int64_t high)
{
  auto descriptor = describe(text);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = low;
  range.to_value = high;
  descriptor.integer_range.push_back(range);
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describe_double(
  const std::string & text, double low, double high)
{
  auto descriptor = describe(text);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = low;
  range.to_value = high;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

/// One phrase for whichever solver produced the numbers, so the log line does not
/// print a residual in metres under a solver that measures pixels.
std::string describe_fit(bool pnp, double residual_px, double residual_m, double scale)
{
  char buffer[128];
  if (pnp) {
    std::snprintf(buffer, sizeof(buffer), " at %.2f px reprojection", residual_px);
  } else {
    std::snprintf(buffer, sizeof(buffer), " at %.4f m (depth scale %.4f)", residual_m, scale);
  }
  return std::string(buffer);
}

const char * regime_name(OdometryRegime regime)
{
  return regime == OdometryRegime::SixDof ? "sixdof" : "rotation_only";
}

}  // namespace

OdometryNode::OdometryNode(const rclcpp::NodeOptions & options)
: Node("odometry_node", options),
  keyframes_(KeyframeStore::Config{}),
  last_log_(0, 0, RCL_ROS_TIME)
{
  // --- The regime -----------------------------------------------------------
  //
  // A string rather than a bool, because the two are different estimators and not
  // one estimator with a feature switched off — and because a bool named
  // `use_depth` would read as a fallback, which it is not. An unknown value is
  // refused rather than defaulted: silently running the control regime while a
  // config file asks for the real one is precisely the class of failure this
  // project keeps finding.
  const std::string regime_text = declare_parameter(
    "odometry", std::string("sixdof"),
    describe(
      "sixdof | rotation_only. sixdof pairs each frame's corners with its own "
      "depth map and fits a rigid transform; rotation_only fits bearing rays and "
      "publishes zero translation, which is what P3 did and what "
      "tools/gates/odom.sh measures against."));
  if (regime_text == "sixdof") {
    regime_ = OdometryRegime::SixDof;
  } else if (regime_text == "rotation_only") {
    regime_ = OdometryRegime::RotationOnly;
  } else {
    throw rclcpp::exceptions::InvalidParameterValueException(
            "odometry must be 'sixdof' or 'rotation_only', not '" + regime_text + "'");
  }

  min_pairs_ = static_cast<std::size_t>(
    declare_parameter(
      "min_matched_pairs", 8,
      describe_int(
        "Consecutive pairs needed before a rotation is trusted. Below this the "
        "fit is interpolating noise.", 3, 500)));
  max_residual_rad_ = declare_parameter(
    "max_residual_rad", 0.03,
    describe_double(
      "Mean ray residual ceiling. 0.03 rad is 1.7 degrees, ~28 px at fx=953.",
      0.001, 1.0));
  reject_fraction_ = declare_parameter(
    "reject_fraction", 0.2,
    describe_double(
      "Fraction of worst pairs discarded before each refit. Never below the "
      "min_matched_pairs floor — rejecting until a fit looks good is how a robust "
      "estimator becomes a way of manufacturing agreement.", 0.0, 0.6));

  // --- P7's gates, and they are the point-cloud versions of the three above ---
  const std::string solver = declare_parameter(
    "pose_solver", std::string("pnp"),
    describe(
      "pnp | points. pnp poses each frame from the keyframe's 3D landmarks and "
      "this frame's *pixels*, so only one depth map is involved and the "
      "translation comes from reprojection geometry. points fits a rigid "
      "transform between two unprojected clouds, which is the obvious thing to do "
      "and is the control: see PnpFit in rgbd_odometry.hpp for the four "
      "measurements that ended with it."));
  if (solver == "pnp") {
    solve_pnp_ = true;
  } else if (solver == "points") {
    solve_pnp_ = false;
  } else {
    throw rclcpp::exceptions::InvalidParameterValueException(
            "pose_solver must be 'pnp' or 'points', not '" + solver + "'");
  }
  reprojection_px_ = declare_parameter(
    "pnp_inlier_px", 3.0,
    describe_double(
      "RANSAC inlier threshold for the pose solve, in pixels. Generous beside the "
      "0.4955 px this camera's calibration reproject at, because the 3D points it "
      "is fitting come from a monocular depth network and carry its error, not the "
      "lens's.", 0.5, 30.0));
  max_reprojection_px_ = declare_parameter(
    "max_reprojection_px", 2.0,
    describe_double(
      "Mean inlier reprojection error past which the pose is refused and the last "
      "one held. **In pixels, which is the point**: the metres this pipeline works "
      "in are arbitrary until depth_scale is pinned with a tape measure, so a gate "
      "in metres is a gate on an unknown unit.", 0.1, 20.0));
  translation_tau_s_ = declare_parameter(
    "translation_tau_s", 0.5,
    describe_double(
      "Time constant of the low-pass on the measured position, in seconds; 0 "
      "disables it. The hand moves centimetres a second while the estimate's "
      "error is white at the depth rate and ~30 times larger per sample, so the "
      "two are separable by band even though no better estimator would separate "
      "them by accuracy.", 0.0, 10.0));
  max_speed_m_s_ = declare_parameter(
    "max_speed_m_s", 2.0,
    describe_double(
      "Speed past which a pose is refused as implausible rather than published. "
      "In the map's arbitrary units — landmarks are clipped at max_depth_m, so "
      "2.0 is about a third of the visible depth per second, which no hand-held "
      "sweep reaches. This is the gate a reprojection error cannot supply: PnP "
      "reports how well a pose explains the pixels, never whether the 3D points "
      "behind them were where the depth network said.", 0.01, 1000.0));
  max_turn_rad_s_ = declare_parameter(
    "max_turn_rad_s", 6.0,
    describe_double(
      "The same refusal for rotation. 6 rad/s is ~340 deg/s, past which a rolling "
      "shutter would have smeared the frame beyond tracking anyway.", 0.01, 100.0));
  max_hold_frames_ = static_cast<std::size_t>(
    declare_parameter(
      "max_hold_frames", 5,
      describe_int(
        "Consecutive depth frames without a usable fit before the reference view "
        "is replaced at the held pose. Without it, losing track once loses it for "
        "the rest of the session — measured as pose_ok frozen at 188 while held "
        "climbed past 560.", 1, 200)));

  // --- The tracking state (#13's P19) -----------------------------------------
  //
  // Separate from max_hold_frames on purpose, and counted separately: that one is
  // when to give up on a *reference view*, this one is when to stop trusting the
  // *pose*. They default to the same 5, and the monitor keeps its own run length
  // because the stall rule resets the node's every 5 — see tracking_state.hpp.
  TrackingMonitor::Config tracking_config;
  tracking_config.lost_after_holds = static_cast<std::size_t>(
    declare_parameter(
      "lost_after_holds", 5,
      describe_int(
        "Consecutive depth frames without a fit before the state is LOST and "
        "fusion_node stops integrating. A run, never one: bags/desk1 holds ~20% of "
        "its depth frames as scattered singles, and LOST on the first would refuse a "
        "fifth of the room.", 1, 1000)));
  tracking_config.recover_after_fits = static_cast<std::size_t>(
    declare_parameter(
      "recover_after_fits", 2,
      describe_int(
        "Consecutive fits needed to leave LOST. tools/gates/lost.sh sets it past the "
        "clip's length for its never-recovers control.", 1, 100000000)));
  tracking_ = std::make_unique<TrackingMonitor>(tracking_config);
  const std::string tracking_topic = declare_parameter(
    "tracking_state_topic", std::string("/tracking/state"),
    describe(
      "Where OK / LOST goes, one message per depth frame. fusion_node's "
      "tracking_state_topic must be the same string, and test_transforms asserts "
      "it: a mismatch leaves fusion with no state for any frame, which it refuses."));
  keyframe_min_shared_ = static_cast<std::size_t>(
    declare_parameter(
      "keyframe_min_shared", 60,
      describe_int(
        "Landmarks still shared with the reference view, below which a new "
        "reference is taken whatever the angle and distance thresholds say. A "
        "keyframe the camera has turned away from satisfies both of those right up "
        "until it does not, and its last surviving correspondences are all in one "
        "corner of the frame.", 8, 500)));
  min_landmark_pairs_ = static_cast<std::size_t>(
    declare_parameter(
      "min_landmark_pairs", 12,
      describe_int(
        "Landmarks common to two depth-backed frames before a 6-DoF transform is "
        "trusted. Higher than min_matched_pairs because three points already "
        "determine a rigid transform exactly, so a handful of them fits its own "
        "noise perfectly and reports a residual of zero.", 4, 500)));
  max_point_residual_m_ = declare_parameter(
    "max_point_residual_m", 0.05,
    describe_double(
      "Mean landmark residual ceiling, in metres of the map's own units. Note "
      "those units are arbitrary until depth_scale is pinned with a tape measure "
      "— this number is a threshold on self-consistency, not on real distance.",
      0.001, 2.0));
  point_reject_fraction_ = declare_parameter(
    "point_reject_fraction", 0.3,
    describe_double(
      "Fraction of worst landmark pairs discarded before each refit. Higher than "
      "reject_fraction because a landmark outlier is unbounded: a ray can be at "
      "most 180 degrees wrong, a depth reading off the far side of an occlusion "
      "edge is metres out.", 0.0, 0.6));
  // **Measured, not reasoned — see ScaleHandling in rgbd_odometry.hpp.** false is
  // the control: it reported an 89.5 m path over a 45 s desk sweep and a surface
  // worse than rotation-only odometry, because a rigid fit has nowhere to put the
  // depth network's few percent of frame-to-frame scale wobble except into
  // translation along the view axis.
  scale_handling_ = declare_parameter(
    "divide_out_depth_scale", true,
    describe(
      "Estimate the scale difference between two frames' landmarks and discard "
      "it, keeping only rotation and translation. Depth Anything V2 estimates "
      "*relative* depth and its scale breathes; fusion_node's aligner exists for "
      "the same reason and hits its 15% clamp on 1 frame in 7 of bags/desk1.")) ?
    ScaleHandling::DivideOut : ScaleHandling::Rigid;
  depth_patch_ = static_cast<int>(
    declare_parameter(
      "depth_patch", 3,
      describe_int(
        "Side of the square window a keypoint's depth is taken as the median of. "
        "1 is a single pixel, which is the wrong reading to take at a corner: ORB "
        "puts features on edges by construction, and an edge is where the depth "
        "map steps between foreground and background.", 1, 15)));
  depth_patch_spread_ = declare_parameter(
    "depth_patch_spread", 0.1,
    describe_double(
      "Relative spread across that window past which the reading is refused as "
      "straddling a depth step. Relative rather than absolute, because depth "
      "error grows with distance.", 0.005, 1.0));
  min_depth_m_ = declare_parameter(
    "min_depth_m", 0.2,
    describe_double("Readings nearer than this are the model's noise floor.", 0.01, 5.0));
  max_depth_m_ = declare_parameter(
    "max_depth_m", 6.0,
    describe_double(
      "Readings past this are the model's 'far away or no idea' and must match "
      "depth_node's max_range_m. A landmark at the clip plane is a landmark on a "
      "surface that does not exist.", 0.5, 100.0));
  history_frames_ = static_cast<std::size_t>(
    declare_parameter(
      "history_frames", 90,
      describe_int(
        "Frames of ORB output kept waiting for their depth map. depth_node costs "
        "~55 ms, so a depth map arrives ~3 frames after its image; 90 frames is "
        "~1.5 s at 59 Hz, which is headroom of a different order rather than a "
        "tight fit.", 4, 600)));

  const double keyframe_angle_deg = declare_parameter(
    "keyframe_angle_deg", 18.0,
    describe_double(
      "View change admitting a new keyframe. Either this or keyframe_distance_m, "
      "not both: a camera panning on the spot travels no distance and still sees "
      "a different room.", 1.0, 90.0));
  const double keyframe_distance_m = declare_parameter(
    "keyframe_distance_m", 0.3,
    describe_double("Motion admitting a new keyframe.", 0.01, 10.0));
  const std::int64_t max_keyframes = declare_parameter(
    "max_keyframes", 500,
    describe_int(
      "Ceiling on the keyframe store. ~37 kB each — the descriptors alone are 16 "
      "kB — so 500 is ~18 MB. The newest keyframe is what each frame is posed "
      "against; matching against *every* one of them, to recognise a place seen "
      "minutes ago, is the deferred loop-closure work.", 1, 100000));
  keyframes_ = KeyframeStore(
    KeyframeStore::Config{
      keyframe_angle_deg * CV_PI / 180.0,
      keyframe_distance_m,
      static_cast<std::size_t>(max_keyframes)});

  // --- The map (#11's P14) ---------------------------------------------------
  // **false by default, and that is a measurement, not caution.** On TUM fr1/desk,
  // 2026-09-29, no configuration of local-map tracking beat this tracker's Sim(3)
  // ATE over 24 runs and seven variants; the best overlapped it. The mechanism is
  // on the `stats map` line as `align_dev`: Depth Anything's scale differs by ~15%
  // between consecutive keyframes, P7's tracker never mixes two depth maps, and a
  // map mixes them in every PnP by construction. Defaulting to the worse tracker
  // would silently move every other gate onto it.
  local_map_ = declare_parameter(
    "local_map", false,
    describe(
      "Track against the local map — the reference keyframe and every keyframe "
      "covisible with it, projected into the frame — rather than against the "
      "newest keyframe's landmarks alone. false is P7's tracker and is "
      "tools/gates/map.sh's control; the map is built either way, so the two runs "
      "do the same work and differ only in what tracking reads."));
  projection_.radius_px = declare_parameter(
    "map_search_radius_px", 12.0,
    describe_double(
      "How far from its predicted pixel a map point's corner may be, in pixels.",
      1.0, 100.0));
  projection_.max_hamming = static_cast<int>(
    declare_parameter(
      "map_search_max_hamming", 50,
      describe_int(
        "ORB Hamming distance, in bits of 256, past which two corners are not the "
        "same corner. 50 is ORB-SLAM's strict threshold.", 1, 256)));
  projection_.ratio = declare_parameter(
    "map_search_ratio", 0.8,
    describe_double(
      "The best candidate must beat the second best by this factor, or the match "
      "is ambiguous and refused.", 0.1, 1.0));
  pimesh_backend::Map::Config map_config;
  map_config.min_parallax_deg = declare_parameter(
    "map_min_parallax_deg", 3.0,
    describe_double(
      "Below this parallax a map point keeps the depth network's reading rather "
      "than being triangulated. Depth error per pixel of noise is ~d/(f*parallax), "
      "so at 1 degree and f=525 a triangulation is ~11% of the distance out per "
      "pixel — worse than the network it would replace.", 0.1, 45.0));
  map_config.associate_px = declare_parameter(
    "map_associate_px", 4.0,
    describe_double(
      "An association tracking claims is refused if the point reprojects further "
      "than this from the feature in the keyframe it is added to. Looser than "
      "pnp_inlier_px because the keyframe pose is the published one, after the "
      "low-pass.", 0.5, 50.0));
  map_config.align_scale = declare_parameter(
    "map_align_scale", true,
    describe(
      "Scale each new keyframe's depth map onto the map's scale, from the median "
      "depth ratio over the points it re-observed, before it creates points. The "
      "network's scale breathes a few percent a frame; P7 never mixed two depth "
      "maps so never saw it, and a map is nothing but many depth maps mixed."));
  map_config.align_gain = declare_parameter(
    "map_align_gain", 0.5,
    describe_double(
      "The fraction of that log-ratio applied. 1 makes each keyframe inherit the "
      "map's scale exactly, which is a random walk — measured as a fitted Sim(3) "
      "scale of 0.699 on one fr1/desk run against 0.99-1.06 on the rest. 0 leaves "
      "the map a patchwork of depth-map scales. Between them the network's "
      "absolute scale still wins over time.", 0.0, 1.0));
  map_config.reading_window = static_cast<std::size_t>(
    declare_parameter(
      "map_reading_window", 1,
      describe_int(
        "A map point sits at the mean of this many of its newest depth readings. 1 "
        "puts it exactly where P7's reference landmark would be; see "
        "Map::Config::reading_window for the measurement behind that default.",
        1, 50)));
  map_config.local_keyframes = static_cast<std::size_t>(
    declare_parameter(
      "map_local_keyframes", 10,
      describe_int(
        "Most-covisible keyframes, beside the reference, whose points make the "
        "local map.", 1, 200)));
  map_ = std::make_unique<pimesh_backend::Map>(map_config);

  // --- The backend thread (#11's P15) -------------------------------------------
  pimesh_backend::LocalMapper::Config mapper_config;
  // **false by default until gates/ba.sh says otherwise**, for local_map's reason:
  // a default is what every other gate runs, and it should be the configuration that
  // has been measured to be better rather than the one most recently written.
  local_ba_ = declare_parameter(
    "local_ba", false,
    describe(
      "Bundle-adjust the covisible window around each new keyframe on the backend "
      "thread — poses and points together, g2o, Huber kernel, with each depth "
      "reading as a prior. false is tools/gates/ba.sh's control: the same thread "
      "doing the same insert, triangulate and cull, skipping only the solve. Only "
      "affects tracking with local_map:=true, since P7's tracker never reads the "
      "map."));
  mapper_config.bundle_adjust = local_ba_;
  mapper_config.window_keyframes = static_cast<std::size_t>(
    declare_parameter(
      "ba_window_keyframes", 10,
      describe_int(
        "Keyframes bundle adjustment moves: the newest and its most covisible. "
        "Every other keyframe that sees their points is held fixed.", 2, 100)));
  mapper_config.max_fixed_keyframes = static_cast<std::size_t>(
    declare_parameter(
      "ba_max_fixed_keyframes", 20,
      describe_int(
        "Ceiling on the fixed keyframes around the window, most-shared first. They "
        "anchor the solve and cost only their edges.", 1, 200)));
  mapper_config.ba.depth_sigma_rel = declare_parameter(
    "ba_depth_sigma_rel", 0.15,
    describe_double(
      "One sigma of a depth reading, relative to the depth. 0.15 is P14's "
      "measurement of how far consecutive keyframes' depth maps disagree (align_dev). "
      "0 drops the prior and makes this monocular BA, whose only scale anchor is "
      "the fixed keyframes.", 0.0, 10.0));
  mapper_config.ba.depth_scale_sigma = declare_parameter(
    "ba_depth_scale_sigma", 0.0,
    describe_double(
      "One sigma of each keyframe's depth-map scale, as a log. Positive gives every "
      "keyframe a scale all its readings share — P14 measured the depth network's "
      "error as a whole-map breathing (align_dev, 15-21%), which an independent prior "
      "per reading cannot represent. 0 is the per-reading model gates/ba.sh measured "
      "on 2026-09-30.", 0.0, 2.0));
  mapper_config.ba.depth_point_sigma_rel = declare_parameter(
    "ba_depth_point_sigma_rel", 0.05,
    describe_double(
      "One sigma of a depth reading, relative to the depth, once its keyframe's scale "
      "is modelled — the within-frame warp. Used only with ba_depth_scale_sigma > 0. "
      "P12 measured 5.3% of within-frame spread over a patch of wall.", 0.001, 10.0));
  mapper_config.queue = static_cast<std::size_t>(
    declare_parameter(
      "backend_queue", 2,
      describe_int(
        "Keyframes waiting for the backend. **Not newest-wins**: a full queue refuses "
        "and the keyframe is offered again next frame, because a dropped keyframe is "
        "a hole in the map. Keyframes arrive every half second or so, so it is small.",
        1, 50)));
  mapper_config.nice = static_cast<int>(
    declare_parameter(
      "backend_nice", 10,
      describe_int(
        "Nice value of the backend thread alone. Measured on mesh_node: a CPU-heavy "
        "thread at equal priority took depth_node from 17.8 to 14.6 Hz with its "
        "per-frame cost unchanged.", 0, 19)));
  // --- Place recognition (#12's P16) -----------------------------------------------
  //
  // **On by default, because it changes nothing.** It reads the keyframe store and
  // reports; no pose, map or transform depends on what it finds until P17 gives a
  // closure somewhere to go. Its cost is a niced thread, which gates/place.sh
  // measures.
  place_recognition_ = declare_parameter(
    "place_recognition", true,
    describe(
      "Search every keyframe older than place_min_gap_s for the place each new "
      "keyframe shows, and verify a candidate by PnP before reporting it. Reports "
      "only: nothing downstream reads a closure yet."));
  place_config_.place.min_gap_s = declare_parameter(
    "place_min_gap_s", 3.0,
    describe_double(
      "A candidate keyframe must be at least this much older than the query. Younger "
      "is the tracker's business.", 0.0, 3600.0));
  place_config_.place.min_inliers = static_cast<std::size_t>(
    declare_parameter(
      "place_min_inliers", 30,
      describe_int(
        "PnP inliers a candidate needs to be accepted as the same place — the one "
        "number standing between a descriptor match and a closure.", 6, 1000)));
  place_config_.max_keyframes = static_cast<std::size_t>(max_keyframes);
  // --- Loop closure (#12's P17) ---------------------------------------------------
  //
  // **false by default until gates/loop.sh says otherwise**, for local_map's reason.
  // Off, `map -> odom` is still published — identity — because the launch file no
  // longer runs a static publisher for that edge while the pipeline is up: one
  // authority per edge.
  loop_closure_ = declare_parameter(
    "loop_closure", false,
    describe(
      "Add each place-recognition closure to a pose graph over every keyframe, "
      "optimise it, and publish the correction as map -> odom. Needs "
      "place_recognition. false is tools/gates/loop.sh's control: map -> odom stays "
      "identity."));
  place_config_.close_loops = loop_closure_ && place_recognition_;
  if (loop_closure_ && !place_recognition_) {
    RCLCPP_WARN(
      get_logger(),
      "loop_closure:=true with place_recognition:=false — there will be no closures "
      "to add, and map -> odom stays identity.");
  }
  map_frame_ = declare_parameter(
    "map_frame", std::string("map"),
    describe("Parent of odom_frame in the TF tree; the frame the pose graph corrects into."));
  keyframe_trajectory_path_ = declare_parameter(
    "keyframe_trajectory_path", std::string(""),
    describe(
      "Where to write every keyframe's pose in map_frame, TUM format, once a stats "
      "window — the camera optical pose, the frame TUM's ground truth is in. Empty "
      "writes nothing. For tools/gates/loop.sh."));
  // --- A saved map (#13's P20) ---------------------------------------------------
  map_save_path_ = declare_parameter(
    "map_save_path", std::string(""),
    describe(
      "Write the keyframe store here — each keyframe at its pose in map_frame — every "
      "5 s while keyframes are being added, and once more at shutdown. Empty saves "
      "nothing. See map_io.hpp for the format and for why map points are not in it."));
  map_load_path_ = declare_parameter(
    "map_load_path", std::string(""),
    describe(
      "Load a map saved by an earlier session. The session then starts LOST and leaves "
      "LOST only by relocalising into it: map -> odom is set from the recovered pose, "
      "and fusion_node integrates nothing until then. Empty loads nothing."));
  if (!map_load_path_.empty()) {
    if (!map_save_path_.empty()) {
      throw rclcpp::exceptions::InvalidParameterValueException(
              "map_load_path and map_save_path together would extend a map across "
              "sessions, which is not built yet — see milestone-i-future.md");
    }
    if (loop_closure_) {
      throw rclcpp::exceptions::InvalidParameterValueException(
              "map_load_path with loop_closure:=true would give map -> odom two "
              "authorities, the relocalisation and the pose graph");
    }
    if (regime_ != OdometryRegime::SixDof) {
      throw rclcpp::exceptions::InvalidParameterValueException(
              "map_load_path needs odometry:=sixdof — a relocalisation is verified against "
              "depth landmarks, which rotation_only never reads");
    }
    const auto started = std::chrono::steady_clock::now();
    const std::string why = load_keyframes(map_load_path_, loaded_map_);
    if (!why.empty()) {
      // Fatal, for the calibration loader's reason: a map somebody asked for and did
      // not get is not a session without a map, it is a session that would say LOST
      // for ever about a room it was told it knew.
      throw std::runtime_error("map_load_path: " + why);
    }
    std::error_code ec;
    const auto bytes = std::filesystem::file_size(map_load_path_, ec);
    reloc_stats_.loaded = loaded_map_.size();
    reloc_stats_.load_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    reloc_stats_.file_mb = ec ? -1.0 : static_cast<double>(bytes) / 1e6;
    TrackingMonitor::Config relocalising = tracking_->config();
    relocalising.recover_by_relocalisation = true;
    tracking_ = std::make_unique<TrackingMonitor>(relocalising);
    RCLCPP_INFO(
      get_logger(),
      "map loaded: keyframes=%zu file_mb=%.2f load_ms=%.1f from %s — starting LOST until "
      "a relocalisation into it", reloc_stats_.loaded, reloc_stats_.file_mb,
      reloc_stats_.load_ms, map_load_path_.c_str());
  }

  mapper_ = std::make_unique<pimesh_backend::LocalMapper>(*map_, mapper_config);
  mapper_->start();
  map_points_period_s_ = declare_parameter(
    "map_points_period_s", 1.0,
    describe_double(
      "Shortest interval between two /map/points publications. The cloud is only "
      "rebuilt when a keyframe has changed the map.", 0.1, 60.0));

  publish_tf_ = declare_parameter(
    "publish_tf", true,
    describe("Publish odom -> base_link. Rotation only in the rotation_only regime."));
  odom_frame_ = declare_parameter("odom_frame", std::string("odom"), describe("Parent frame."));
  base_frame_ = declare_parameter(
    "base_frame", std::string("base_link"), describe("Child frame — the body, not the camera."));

  const std::string keypoints_topic = declare_parameter(
    "keypoints_topic", std::string("/keypoints"),
    describe(
      "ORB output from keypoint_node, in-process. Must name keypoint_node's own "
      "published topic: a mismatch is a node that never sees a corner and holds "
      "its pose for the whole session, which looks exactly like a camera that is "
      "not moving."));
  const std::string depth_topic = declare_parameter(
    "depth_topic", std::string("/depth"),
    describe(
      "32FC1 metres from depth_node, in-process. Must name depth_node's own "
      "depth_topic: a mismatch is a node that never sees a depth map and holds "
      "its pose for the whole session, which looks exactly like a camera that is "
      "not moving."));

  const double stats_period_s = declare_parameter(
    "stats_period_s", 5.0,
    describe_double("How often to log the summary line.", 0.5, 120.0));

  // --- QoS ------------------------------------------------------------------
  rclcpp::QoS image_qos(rclcpp::KeepLast(1));
  image_qos.reliable();

  // CameraInfo is transient-local at the publisher, so this matches it — and that
  // is what lets this node start after the camera and still get the intrinsics
  // without waiting for a new message. A VOLATILE reader here would silently wait
  // forever on a camera that had already said everything it was going to say.
  rclcpp::QoS info_qos(rclcpp::KeepLast(1));
  info_qos.reliable().transient_local();

  // The pose, as a message as well as a TF edge. Not a duplicate: TF is a tree
  // every consumer queries by time, and an Odometry stream is a *sequence* — which
  // is what an RViz Odometry display with a large `Keep` draws a trail from, and
  // what makes the trajectory visible without a nav_msgs/Path publisher existing
  // anywhere in this project. KeepLast(50) rather than 1, because a trail whose
  // samples are dropped is a trail with holes in it.
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(rclcpp::KeepLast(50)));

  // **KeepLast(1) would be wrong here and it is the one QoS in this node that
  // differs from the image path's.** Every keypoints message matters: the depth
  // rendezvous looks each one up by exact stamp, and in rotation_only the pairs
  // chain, so a message the middleware dropped to keep the queue at one is a
  // rotation increment that silently never happened. Depth below keeps 1, because
  // a depth map is a measurement against a keyframe rather than a link in a chain.
  rclcpp::QoS keypoints_qos(rclcpp::KeepLast(120));
  keypoints_qos.reliable();

  // A *shared const* pointer on both, never a unique_ptr: `/keypoints` has
  // keypoint_probe beside this node and `/depth` has fusion_node, so each is a
  // fan-out, and rclcpp copies the buffer for every ownership-taking subscription
  // but the last. See the note in the header.
  keypoints_sub_ = create_subscription<pimesh_msgs::msg::Keypoints>(
    keypoints_topic, keypoints_qos,
    [this](pimesh_msgs::msg::Keypoints::ConstSharedPtr msg) {this->on_keypoints(std::move(msg));});
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", info_qos,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {this->on_camera_info(msg);});
  // Subscribed in both regimes. In rotation_only nothing is done with the frames
  // beyond counting them, and that is on purpose: the control run then puts the
  // same load on /depth as the measured one, so a rate difference between the two
  // is a difference in the estimator rather than in how many consumers the
  // topic had.
  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic, image_qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {this->on_depth(std::move(msg));});

  // The basis for the optical -> body change of frame comes from TF, not from a
  // quaternion written here. There is exactly one publisher of that edge in this
  // project and exactly one place its numbers live; a second copy is how two
  // conventions end up one rotation apart with nothing failing.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  stats_pub_ = create_publisher<pimesh_msgs::msg::PipelineStats>("/pipeline/stats", 10);

  // **Reliable and deep, because fusion_node looks each one up by exact stamp.** A
  // state the middleware dropped is a frame fusion refuses for want of one — the
  // safe direction, but a frame lost all the same. 60 is ~3.5 s of depth.
  tracking_pub_ = create_publisher<pimesh_msgs::msg::TrackingState>(
    tracking_topic, rclcpp::QoS(rclcpp::KeepLast(60)).reliable());

  // Latched, so a viewer started after the last keyframe still gets the map. A
  // VOLATILE reader against this writer is compatible and simply misses the stored
  // one — see the note on /world/mesh in CLAUDE.md for why that mismatch is legal.
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.reliable().transient_local();
  map_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/map/points", map_qos);
  // **A Path whose poses are corrections, not camera poses** — each is `map <- odom`
  // stamped at a keyframe. nav_msgs/Path is the nearest standard shape (stamped
  // poses, in order) and RViz can draw it, but reading one as a trajectory would be
  // reading it wrong: identity everywhere means "no correction", not "the camera
  // stood at the origin". Latched, for fusion_node's rebuild (#12's P18), and
  // published only after a solve.
  corrections_pub_ = create_publisher<nav_msgs::msg::Path>("/pose_graph/corrections", map_qos);

  pose_worker_ = std::thread([this] {this->pose_work();});

  last_log_ = now();
  stats_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(stats_period_s)),
    [this] {this->log_stats();});

  // The regime, stated at startup, because it is a property of this node that
  // every consumer of its TF has to know and no message carries.
  RCLCPP_INFO(
    get_logger(), "regime=%s: %s", regime_name(regime_),
    regime_ == OdometryRegime::SixDof ?
    "6-DoF from depth-backed landmarks on exact RGB-D pairs; the pose is published "
    "at the depth rate, at each depth map's own stamp." :
    "Bearing rays only — translation is identically zero. This is P3's estimator, "
    "kept as tools/gates/odom.sh's control run.");
  if (regime_ == OdometryRegime::SixDof) {
    RCLCPP_INFO(
      get_logger(),
      "each frame is measured against a keyframe, not against its predecessor. Pose "
      "gated on >= %zu shared landmarks and residual < %.3f m, from %d px patches with "
      "spread < %.0f%% over [%.2f, %.2f] m; a new keyframe at %.0f deg, %.2f m, or "
      "fewer than %zu shared. Solver: %s.",
      min_landmark_pairs_, max_point_residual_m_, depth_patch_,
      100.0 * depth_patch_spread_, min_depth_m_, max_depth_m_,
      keyframe_angle_deg, keyframe_distance_m, keyframe_min_shared_,
      solve_pnp_ ? "PnP on the keyframe's landmarks and this frame's pixels" :
      (scale_handling_ == ScaleHandling::DivideOut ?
      "a similarity fit between two point clouds" :
      "a rigid fit between two point clouds"));
  } else {
    RCLCPP_INFO(
      get_logger(),
      "pose gated on >= %zu pairs and residual < %.3f rad.",
      min_pairs_, max_residual_rad_);
  }
}

OdometryNode::~OdometryNode()
{
  depth_mailbox_.stop();
  if (pose_worker_.joinable()) {pose_worker_.join();}
  // The last word, after the worker has stopped adding keyframes: whatever the 5 s
  // cadence had not written yet.
  if (!map_save_path_.empty()) {
    if (places_) {places_->flush();}
    save_map("shutdown");
  }
  relocaliser_.reset();
  places_.reset();
}

void OdometryNode::save_map(const char * why)
{
  const auto started = std::chrono::steady_clock::now();
  // Each keyframe at its pose in the map frame: the pose graph's corrected pose where
  // loop closure has one, else the current map <- odom (identity without closure).
  std::unordered_map<std::int64_t, cv::Affine3d> corrected;
  cv::Affine3d map_from_odom = cv::Affine3d::Identity();
  if (places_) {
    map_from_odom = places_->map_from_odom();
    if (loop_closure_) {
      for (const auto & [stamp, pose] : places_->trajectory()) {corrected[stamp] = pose;}
    }
  }
  std::deque<Keyframe> out;
  for (const Keyframe & kf : keyframes_.frames()) {
    Keyframe copy = kf;
    const auto found = corrected.find(kf.stamp_ns);
    copy.odom_from_camera =
      found != corrected.end() ? found->second : map_from_odom * kf.odom_from_camera;
    out.push_back(std::move(copy));
  }
  const std::string error = save_keyframes(map_save_path_, out);
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  if (!error.empty()) {
    RCLCPP_ERROR(get_logger(), "map not saved (%s): %s", why, error.c_str());
    return;
  }
  std::error_code ec;
  const auto bytes = std::filesystem::file_size(map_save_path_, ec);
  {
    std::lock_guard<std::mutex> lock(reloc_mutex_);
    ++reloc_stats_.saves;
    reloc_stats_.saved_keyframes = out.size();
    reloc_stats_.save_ms = ms;
    reloc_stats_.saved_mb = ec ? -1.0 : static_cast<double>(bytes) / 1e6;
  }
  RCLCPP_INFO(
    get_logger(), "map saved (%s): keyframes=%zu file_mb=%.2f save_ms=%.1f to %s", why,
    out.size(), ec ? -1.0 : static_cast<double>(bytes) / 1e6, ms, map_save_path_.c_str());
}

void OdometryNode::apply_relocalisations()
{
  if (!relocaliser_) {return;}
  for (const Relocalisation & r : relocaliser_->take_results()) {
    if (!r.accepted) {continue;}
    bool applied = false;
    std::uint64_t lost_frames = 0;
    {
      std::lock_guard<std::mutex> lock(tracking_mutex_);
      applied = tracking_->relocalised();
      lost_frames = tracking_->lost_frames();
    }
    if (!applied) {
      // An answer to a query asked while LOST, arriving after an earlier answer
      // already fixed the map frame. Moving it again would be a jump in map -> odom
      // with nothing new to justify it.
      std::lock_guard<std::mutex> lock(reloc_mutex_);
      ++reloc_stats_.ignored;
      continue;
    }
    // **Set, not accumulated** — P7's keyframe lesson one frame up: the map pose of
    // this frame is measured against the saved keyframe directly, so map <- odom is
    // that measurement composed with this frame's odom pose, once.
    const cv::Affine3d map_from_odom = r.map_from_camera * r.odom_from_camera.inv();
    {
      std::lock_guard<std::mutex> lock(reloc_mutex_);
      reloc_map_from_odom_ = map_from_odom;
      ++reloc_stats_.applied;
    }
    const cv::Vec3d t(r.map_from_camera.translation());
    const cv::Vec4d q = quaternion_from_rotation(r.map_from_camera.rotation());
    // Read by tools/gates/relocalise.sh and scored against motion capture: the query
    // camera's pose in the saved map, TUM order (x y z qx qy qz qw).
    RCLCPP_INFO(
      get_logger(),
      "relocalised query_ns=%lld candidate_ns=%lld inliers=%zu matches=%zu searched=%zu "
      "ms=%.1f lost_frames=%lu pose=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
      static_cast<long long>(r.query_stamp_ns), static_cast<long long>(r.candidate_stamp_ns),
      r.inliers, r.matches, r.searched, r.ms, static_cast<unsigned long>(lost_frames),
      t[0], t[1], t[2], q[0], q[1], q[2], q[3]);
  }
}

void OdometryNode::on_keypoints(pimesh_msgs::msg::Keypoints::ConstSharedPtr msg)
{
  ++keypoint_frames_;

  // --- Refuse a message whose parallel arrays disagree ------------------------
  //
  // The message documents every per-feature array as the same length and nothing
  // enforces it at runtime, so this is the one place it is checked. Refused
  // whole rather than converted as far as it goes: a half-read frame produces
  // corners paired with the wrong track ids, which is a plausible wrong answer,
  // and the accessors in keypoints_view.hpp clamp only so that a caller who
  // forgot this check cannot read off the end.
  if (!keypoints_well_formed(*msg)) {
    ++malformed_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "keypoints message with mismatched array lengths (x=%zu y=%zu ids=%zu "
      "prev_x=%zu descriptors=%zu for %u bytes each) — refused. %lu so far.",
      msg->x.size(), msg->y.size(), msg->track_id.size(), msg->prev_x.size(),
      msg->descriptors.size(), msg->descriptor_bytes,
      static_cast<unsigned long>(malformed_.load()));
    return;
  }

  // --- The bounded copy ------------------------------------------------------
  //
  // Done here in the callback rather than on a worker, and it is the one thing in
  // this node that is allowed to be: it is a fixed-size copy of ~500 points and a
  // 16 kB descriptor block, with no decode, no inference and no solve in it. The
  // alternative — a mailbox and a thread — would be *worse than slow*, because a
  // mailbox is newest-wins: every message it dropped would be a frame whose ORB
  // output the depth rendezvous below can never find.
  //
  // The conversions themselves are in keypoints_view.hpp so that a test can call
  // them; they were three loops here, and two of them read past the end of a
  // vector on a ragged message.
  auto record = std::make_shared<FrameRecord>();
  record->stamp_ns = stamp_ns(msg->header.stamp);
  record->optical_frame = msg->header.frame_id;
  record->pixels = keypoint_pixels(*msg);
  record->ids = msg->track_id;
  record->descriptors = copy_keypoint_descriptors(*msg);
  record->image_size = cv::Size(
    static_cast<int>(msg->image_width), static_cast<int>(msg->image_height));

  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    history_.push_back(std::move(record));
    while (history_.size() > history_frames_) {history_.pop_front();}
  }

  if (regime_ != OdometryRegime::RotationOnly) {return;}

  // --- The control regime's estimator ----------------------------------------
  //
  // Also in the callback, and unlike the copy above this is a *fit* — Kabsch over
  // a few hundred bearing pairs with up to three refits. It is here because it
  // cannot be anywhere else: the pairs in each message span the **detector's**
  // previous frame, so this chain composes correctly only if every message is
  // processed in order and none is skipped. A newest-wins mailbox would silently
  // drop increments and under-rotate; a growing queue is what this pipeline is
  // built to avoid.
  //
  // **Measured 2026-09-23 over bags/desk1: 0.12 ms per frame at 56.6 Hz**, against
  // the 6.8 ms keypoint_node spends on ORB for the same frame — so it is ~0.7% of
  // one core and two orders off the things this project puts on worker threads.
  // It is published as `rotation_cost` in the stats line rather than assumed,
  // because a claim about work done in a callback should be a number somebody can
  // read; if it ever stops being small, that is where it will show.
  //
  // It is also the regime nothing but tools/gates/odom.sh runs.
  const auto start = std::chrono::steady_clock::now();

  update_pose(keypoint_consecutive_pairs(*msg), msg->header.frame_id);
  publish_pose(rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));

  rotation_cost_sum_ms_ = rotation_cost_sum_ms_.load() +
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void OdometryNode::on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  depth_mailbox_.push(std::move(msg));
}

void OdometryNode::on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  // P is the *projection* matrix and K the intrinsics; for a monocular camera with
  // no rectification they agree, and reading K is reading what the calibration
  // file says rather than what a rectifier would have produced.
  std::lock_guard<std::mutex> lock(k_mutex_);
  const bool first = !have_k_;
  k_ = cv::Matx33d(
    msg->k[0], msg->k[1], msg->k[2],
    msg->k[3], msg->k[4], msg->k[5],
    msg->k[6], msg->k[7], msg->k[8]);
  have_k_ = (k_(0, 0) != 0.0 && k_(1, 1) != 0.0);
  if (first && have_k_) {
    RCLCPP_INFO(
      get_logger(), "intrinsics fx=%.1f fy=%.1f cx=%.1f cy=%.1f from %s",
      k_(0, 0), k_(1, 1), k_(0, 2), k_(1, 2), msg->header.frame_id.c_str());
  }
}

void OdometryNode::pose_work()
{
  while (!depth_mailbox_.stopped()) {
    auto msg = depth_mailbox_.pop(std::chrono::milliseconds(100));
    if (msg) {process_depth(std::move(msg));}
  }
}

std::shared_ptr<const OdometryNode::FrameRecord> OdometryNode::record_at(std::int64_t want)
{
  std::lock_guard<std::mutex> lock(history_mutex_);
  // Newest first: a depth map is ~3 frames behind its image, so the match is near
  // the back and the search is a handful of comparisons rather than 90.
  for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
    if ((*it)->stamp_ns == want) {return *it;}
  }
  return nullptr;
}
void OdometryNode::process_depth(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  ++depth_frames_;
  if (regime_ != OdometryRegime::SixDof) {
    // rotation_only estimates on the image stamps, not these; the state at a depth
    // stamp is whether that chain is holding. Published all the same, because
    // fusion_node refuses a frame it has no state for, and gates/odom.sh's control
    // run integrates in this regime.
    track(!holding_.load(), msg->header.stamp);
    return;
  }

  const auto start = std::chrono::steady_clock::now();

  cv::Matx33d k;
  {
    std::lock_guard<std::mutex> lock(k_mutex_);
    if (!have_k_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no /camera_info yet — holding pose. Unprojection needs K, and a pose from "
        "invented intrinsics is confidently wrong rather than absent.");
      ++shift_held_;
      track(false, msg->header.stamp);
      return;
    }
    k = k_;
  }

  const cv::Mat depth = depth_mat_over(*msg);
  if (depth.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "depth frame with encoding '%s' %ux%u step %u is not usable 32FC1",
      msg->encoding.c_str(), msg->width, msg->height, msg->step);
    ++shift_held_;
    track(false, msg->header.stamp);
    return;
  }

  const std::int64_t when = stamp_ns(msg->header.stamp);
  const std::shared_ptr<const FrameRecord> record = record_at(when);
  if (!record) {
    // Not a warning per frame: at startup the depth maps of frames that went past
    // before this node had a tracker land here, and so does anything that outran
    // the history. The counter is what says whether it is a startup transient or a
    // history too shallow to hold the depth stage's latency.
    ++depth_unmatched_;
    ++shift_held_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "no ORB output at the depth map's own stamp — %lu so far. The history is %zu "
      "frames; a persistent count means depth has fallen further behind than that.",
      static_cast<unsigned long>(depth_unmatched_.load()), history_frames_);
    track(false, msg->header.stamp);
    return;
  }

  if (!ensure_basis(record->optical_frame)) {
    ++shift_held_;
    track(false, msg->header.stamp);
    return;
  }

  // --- This frame's landmarks -------------------------------------------------
  //
  // Sorted by track id, so the intersection with the reference below is a linear
  // merge. Built as tuples and sorted once rather than inserted into a map: ~250
  // entries, and a flat vector is both faster and the shape the merge wants.
  struct Entry
  {
    std::int32_t id;
    cv::Vec3d point;
    cv::Vec3d ray;
    cv::Point2f pixel;
  };
  std::vector<Entry> entries;
  entries.reserve(record->pixels.size());
  // Kept in *detection* order beside the sorted view, because the keyframe store
  // indexes its landmarks by the descriptor row they came from and sorted-by-id
  // order is not that.
  std::vector<std::int32_t> landmark_rows;
  std::vector<cv::Vec3d> landmark_points;
  landmark_rows.reserve(record->pixels.size());
  landmark_points.reserve(record->pixels.size());

  for (std::size_t i = 0; i < record->pixels.size(); ++i) {
    double metres = 0.0;
    if (!sample_depth(
        depth, record->pixels[i].x, record->pixels[i].y, depth_patch_,
        min_depth_m_, max_depth_m_, depth_patch_spread_, metres))
    {
      continue;
    }
    const cv::Vec3d point = unproject(k, record->pixels[i].x, record->pixels[i].y, metres);
    entries.push_back(
      Entry{record->ids[i], point, bearing(k, record->pixels[i].x, record->pixels[i].y),
        record->pixels[i]});
    landmark_rows.push_back(static_cast<std::int32_t>(i));
    landmark_points.push_back(point);
  }
  std::sort(
    entries.begin(), entries.end(),
    [](const Entry & l, const Entry & r) {return l.id < r.id;});

  RgbdView view;
  view.stamp_ns = when;
  view.ids.reserve(entries.size());
  view.points.reserve(entries.size());
  view.bearings.reserve(entries.size());
  view.pixels.reserve(entries.size());
  for (const Entry & entry : entries) {
    view.ids.push_back(entry.id);
    view.points.push_back(entry.point);
    view.bearings.push_back(entry.ray);
    view.pixels.push_back(entry.pixel);
  }
  landmark_sum_ = landmark_sum_.load() + static_cast<double>(view.points.size());

  // --- The correspondence, by track id, against the *reference* view -----------
  //
  // **By id, not by re-matching descriptors.** The tracker has already decided
  // which corner is which, once, against a ten-frame window that forgives the
  // detection churn a second matching pass would trip over — and a second opinion
  // about identity would be a second place for the two to disagree, silently.
  //
  // Against the reference rather than the previous frame: see `reference_` in the
  // header for the measurement that decided it. The pooled window is also what
  // makes it possible at all — a track id survives a feature flickering out for a
  // frame, which is how a landmark is still recognisable half a second later.
  //
  // **The current frame is matched by its keypoints, not by its landmarks.** PnP
  // wants the reference's 3D point and this frame's pixel, and a pixel needs no
  // depth of its own — so the correspondence set is built against every corner
  // ORB found here, roughly twice as many as have a usable depth reading. Only the
  // `points` control needs both sides in 3D.
  std::vector<cv::Vec3d> from;          // reference landmarks, reference optical frame
  std::vector<cv::Point2f> seen;        // where they are on this frame's sensor
  std::vector<cv::Vec3d> to;            // this frame's landmarks — the control only
  std::vector<cv::Vec3d> from_points;   // their reference twins — the control only
  if (reference_.valid()) {
    // This frame's corners, sorted by id, for the linear merge. Every corner, not
    // only the ones with depth.
    std::vector<std::pair<std::int32_t, cv::Point2f>> corners;
    corners.reserve(record->ids.size());
    for (std::size_t i = 0; i < record->ids.size(); ++i) {
      corners.emplace_back(record->ids[i], record->pixels[i]);
    }
    std::sort(
      corners.begin(), corners.end(),
      [](const auto & l, const auto & r) {return l.first < r.first;});

    std::size_t a = 0;
    std::size_t b = 0;
    while (a < reference_.ids.size() && b < corners.size()) {
      if (reference_.ids[a] < corners[b].first) {
        ++a;
      } else if (corners[b].first < reference_.ids[a]) {
        ++b;
      } else {
        from.push_back(reference_.points[a]);
        seen.push_back(corners[b].second);
        ++a;
        ++b;
      }
    }

    if (!solve_pnp_) {
      std::size_t x = 0;
      std::size_t y = 0;
      while (x < reference_.ids.size() && y < view.ids.size()) {
        if (reference_.ids[x] < view.ids[y]) {
          ++x;
        } else if (view.ids[y] < reference_.ids[x]) {
          ++y;
        } else {
          from_points.push_back(reference_.points[x]);
          to.push_back(view.points[y]);
          ++x;
          ++y;
        }
      }
    }
  }
  // The orientation as it stands, read once — used by the control path, as the
  // prediction the local-map search projects with when nothing better exists, and
  // as the pose to hold when nothing can be fitted.
  cv::Affine3d pose_now;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_now = pose_;
  }
  const cv::Affine3d odom_from_camera_now = pose_now * base_from_optical_;

  // --- The map, stage one: by track id -------------------------------------------
  //
  // Every corner whose track id the map knows, against that point's position in the
  // map frame. Done in both settings of `local_map`: in the control it is only the
  // association a new keyframe is built with, so the map the control maintains is
  // the map the tracker *would* have had.
  //
  // **Deduplicated by point**, because a point can carry two track ids — an old one
  // the tracker has not yet retired and a new one the projection search attached —
  // and a corner counted twice in a PnP is a correspondence weighted double with
  // nothing to say so.
  const std::size_t n_features = record->pixels.size();
  std::vector<pimesh_backend::PointId> feature_point(n_features, pimesh_backend::kNoPoint);
  std::vector<std::uint8_t> corner_taken(n_features, 0);
  std::vector<cv::Vec3d> map_object;
  std::vector<cv::Point2f> map_image;
  std::vector<pimesh_backend::PointId> map_ids;
  std::unordered_set<pimesh_backend::PointId> matched_points;
  {
    const std::vector<pimesh_backend::PointView> by_track = map_->lookup_tracks(record->ids);
    for (std::size_t i = 0; i < n_features; ++i) {
      const pimesh_backend::PointId id = by_track[i].id;
      if (id == pimesh_backend::kNoPoint || !matched_points.insert(id).second) {continue;}
      feature_point[i] = id;
      corner_taken[i] = 1;
      map_object.push_back(by_track[i].position);
      map_image.push_back(record->pixels[i]);
      map_ids.push_back(id);
    }
  }
  const std::size_t by_track = map_object.size();

  // --- The map, stage two: the local map, by projection ------------------------------
  //
  // **This is the phase.** The first stage finds only what the tracker still has an
  // id for; this one finds map points the tracker has lost — projected with the pose
  // the first stage predicts, matched on descriptor near where they land. It is what
  // lets a landmark outlive the keyframe that first saw it *and* the track id it was
  // first seen under.
  PnpFit map_fit;
  std::size_t by_projection = 0;
  std::size_t local_keyframes = 0;
  if (local_map_) {
    const PnpFit predicted_fit = fit_pose_pnp(
      k, map_object, map_image, min_landmark_pairs_, reprojection_px_, max_reprojection_px_);
    const cv::Affine3d predicted =
      predicted_fit.ok ? camera_step(predicted_fit.motion()) : odom_from_camera_now;

    const pimesh_backend::LocalMap local = map_->local_map(map_->newest());
    local_keyframes = local.keyframes.size();
    std::vector<std::uint8_t> skip(local.points.size(), 0);
    for (std::size_t i = 0; i < local.points.size(); ++i) {
      skip[i] = matched_points.count(local.points[i].id) != 0 ? 1 : 0;
    }
    const ProjectionSearch search = search_by_projection(
      local.points, skip, predicted, k, record->image_size, record->pixels,
      record->descriptors, corner_taken, projection_);
    for (const ProjectionMatch & m : search.matches) {
      const pimesh_backend::PointView & point = local.points[m.point];
      feature_point[m.corner] = point.id;
      map_object.push_back(point.position);
      map_image.push_back(record->pixels[m.corner]);
      map_ids.push_back(point.id);
    }
    by_projection = search.matches.size();

    map_fit = fit_pose_pnp(
      k, map_object, map_image, min_landmark_pairs_, reprojection_px_, max_reprojection_px_);
    // The combined solve failing where the track-id one succeeded means the
    // projection matches dragged it off; the track-id answer stands. Its inlier
    // indices are still valid — stage one's entries are the first `by_track` of the
    // combined arrays.
    if (!map_fit.ok && predicted_fit.ok) {map_fit = predicted_fit;}

    // What tracking tells the map: every point it expected to see, and the ones it
    // found. A point matched and then rejected by RANSAC was predicted and not found.
    std::vector<pimesh_backend::PointId> visible(map_ids.begin(), map_ids.begin() +
      static_cast<std::ptrdiff_t>(by_track));
    for (std::size_t p : search.in_view) {
      if (!skip[p]) {visible.push_back(local.points[p].id);}
    }
    std::vector<pimesh_backend::PointId> found;
    if (map_fit.ok) {
      for (int index : map_fit.inlier_index) {
        found.push_back(map_ids[static_cast<std::size_t>(index)]);
      }
    }
    map_->record_tracking(visible, found);

    std::lock_guard<std::mutex> lock(map_stats_mutex_);
    local_keyframe_samples_.push_back(static_cast<double>(local_keyframes));
    by_track_sum_ += by_track;
    by_projection_sum_ += by_projection;
    ++map_tracked_frames_;
  }

  // Landmarks this frame shares with what it is measured against: the reference
  // view's in P7's tracker, and the points the map still knows by track id in the
  // local-map one. It is the number the "lost" keyframe rule reads.
  const std::size_t shared = local_map_ ? by_track : (solve_pnp_ ? from.size() : to.size());
  pair_sum_ = pair_sum_.load() + static_cast<double>(shared);

  // --- The solve ---------------------------------------------------------------
  bool advanced = false;
  double residual_m = 0.0;
  double residual_px = 0.0;
  double fitted_scale = 1.0;
  std::size_t pairs_used = 0;
  cv::Affine3d odom_from_camera;
  bool fit_ok = false;

  if (local_map_) {
    // **Map coordinates, so there is no reference to compose with.** P7's pose is
    // the reference keyframe's composed with the fitted step; here the 3D points are
    // already in the map frame, so the fit is the camera's pose outright.
    if (map_fit.ok) {
      odom_from_camera = camera_step(map_fit.motion());
      residual_px = map_fit.residual_px;
      pairs_used = map_fit.inliers;
      fit_ok = true;
      reprojection_sum_px_ = reprojection_sum_px_.load() + map_fit.residual_px;
      inlier_sum_ = inlier_sum_.load() + static_cast<double>(map_fit.inliers);
    }
  } else if (reference_.valid() && shared >= min_landmark_pairs_) {
    if (solve_pnp_) {
      const PnpFit fit = fit_pose_pnp(
        k, from, seen, min_landmark_pairs_, reprojection_px_, max_reprojection_px_);
      if (fit.ok) {
        odom_from_camera = reference_.odom_from_camera * camera_step(fit.motion());
        residual_px = fit.residual_px;
        pairs_used = fit.inliers;
        fit_ok = true;
        reprojection_sum_px_ = reprojection_sum_px_.load() + fit.residual_px;
        inlier_sum_ = inlier_sum_.load() + static_cast<double>(fit.inliers);
      }
    } else {
      // The control: a rigid fit between two unprojected clouds, which is the
      // obvious thing to do and is what four measurements said does not work here.
      const RigidFit fit = fit_rigid_robust(
        from_points, to, min_landmark_pairs_,
        max_point_residual_m_, point_reject_fraction_, 2, scale_handling_);
      if (fit.ok) {
        odom_from_camera = reference_.odom_from_camera * camera_step(fit.motion());
        residual_m = fit.residual_m;
        fitted_scale = fit.scale;
        pairs_used = fit.pairs_used;
        fit_ok = true;
      }
    }
  }

  // --- Is that a motion a camera could have made? ------------------------------
  //
  // Checked against the *last accepted* pose and the time since it, not against
  // the reference: a reference can be a second old, and what is implausible is a
  // speed rather than a displacement.
  // **`have_shift_` is bookkeeping for this gate, not for the filter**, and that
  // distinction cost a run: it started life inside the `translation_tau_s > 0`
  // branch, so with the filter off the gate below was skipped entirely and
  // reported `implausible=0` over a trajectory with a 6.95 m step in it. A guard
  // that is only armed when an unrelated feature is enabled is worse than no
  // guard, because its counter reads as evidence.
  if (fit_ok && have_shift_) {
    const double dt = static_cast<double>(when - last_shift_ns_) * 1e-9;
    if (dt > 0.0) {
      const cv::Affine3d posed = odom_from_camera * base_from_optical_.inv();
      const double moved =
        cv::norm(cv::Vec3d(posed.translation()) - cv::Vec3d(pose_now.translation()));
      const double turned = angle_between(pose_now.rotation(), posed.rotation());
      if (moved > max_speed_m_s_ * dt || turned > max_turn_rad_s_ * dt) {
        ++implausible_;
        RCLCPP_WARN(
          get_logger(),
          "refusing a pose that moved %.2f m and turned %.2f rad in %.0f ms — %.1f m/s and "
          "%.1f rad/s against ceilings of %.1f and %.1f. The fit was confident (%zu inliers "
          "at %.2f px); a reprojection error says nothing about whether the landmarks were "
          "where the depth network put them.",
          moved, turned, dt * 1e3, moved / dt, turned / dt, max_speed_m_s_, max_turn_rad_s_,
          pairs_used, residual_px);
        fit_ok = false;
      }
    }
  }

  if (fit_ok) {
    if (holding_) {
      RCLCPP_INFO(
        get_logger(),
        "regime=sixdof: recovered, %zu of %zu shared landmarks%s",
        pairs_used, shared, describe_fit(solve_pnp_, residual_px, residual_m, fitted_scale).c_str());
      holding_ = false;
    }
    holds_ = 0;
    // **Set, not accumulated.** The fit answers where this camera is *relative to
    // the reference*, so the pose is the reference's composed with it — once. An
    // increment applied to the running pose would add the whole keyframe-to-now
    // transform again at every frame, which is the mistake that made a simulated
    // keyframe run come out worse than frame-to-frame.
    cv::Affine3d posed = odom_from_camera * base_from_optical_.inv();

    // The low-pass, on the measured position. `alpha` is derived from the actual
    // interval between accepted measurements rather than assumed to be 1/17 s, so
    // a run of holds does not leave the filter lagging by however long they took:
    // after a 1 s gap at tau = 0.5 s, alpha is 0.86 and the filter has all but
    // caught up, which is the behaviour a fixed alpha would get wrong in exactly
    // the situation that matters.
    if (translation_tau_s_ > 0.0) {
      cv::Vec3d smoothed(posed.translation());
      if (have_shift_) {
        const double dt = static_cast<double>(when - last_shift_ns_) * 1e-9;
        const double alpha = (dt > 0.0) ? 1.0 - std::exp(-dt / translation_tau_s_) : 1.0;
        cv::Vec3d previous;
        {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          previous = cv::Vec3d(pose_.translation());
        }
        smoothed = previous + alpha * (smoothed - previous);
      }
      posed = cv::Affine3d(posed.rotation(), smoothed);
      // The reference is anchored on the *filtered* pose, so what the next frame
      // is measured against is the pose that was actually published. Anchoring on
      // the raw one would make the filter a display convenience with the noise
      // still in the map.
      odom_from_camera = posed * base_from_optical_;
    }
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      trajectory_m_ = trajectory_m_.load() +
        cv::norm(cv::Vec3d(posed.translation()) - cv::Vec3d(pose_.translation()));
      pose_ = posed;
    }
    last_shift_ns_ = when;
    have_shift_ = true;
    point_residual_sum_ = point_residual_sum_.load() + residual_m;
    scale_sum_ = scale_sum_.load() + fitted_scale;
    ++shift_ok_;
    // The pose is published at every depth stamp, accepted or held, because
    // fusion_node looks it up at exactly these stamps and waits up to its
    // tf_timeout_ms for each. A gap here is not a missing sample; it is a depth
    // frame dropped rather than integrated.
    advanced = true;
  } else {
    if (reference_.valid() && !holding_) {
      RCLCPP_INFO(
        get_logger(),
        "regime=hold: %zu shared landmarks with the reference view (floor %zu), and no "
        "pose under %.2f px — keeping the last one",
        shared, min_landmark_pairs_,
        solve_pnp_ ? max_reprojection_px_ : max_point_residual_m_);
      holding_ = true;
    }
    ++shift_held_;
    ++holds_;
    odom_from_camera = odom_from_camera_now;
  }

  // An answer from the relocaliser lands before this frame's state is decided, so a
  // relocalisation takes effect at this stamp — state and map -> odom together.
  apply_relocalisations();
  // The state first, then the pose: fusion_node waits for the transform and reads
  // the state it finds, so the state has to be on the wire by then.
  track(fit_ok, msg->header.stamp);
  publish_pose(rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));

  // --- While LOST with a saved map: ask it where this is (#13's P20) ----------------
  //
  // Only a frame with a *fitted* pose is asked about. The answer is turned into
  // map <- odom through this frame's odom pose, and a held pose is wrong by however
  // far the camera moved since it was last fitted — which would be baked into every
  // pose after it.
  if (!map_load_path_.empty() && fit_ok) {
    bool lost = false;
    {
      std::lock_guard<std::mutex> lock(tracking_mutex_);
      lost = tracking_->state() == Tracking::Lost;
    }
    if (lost) {
      if (!relocaliser_) {
        Relocaliser::Config config;
        config.place = place_config_.place;
        config.place.focal_px = k(0, 0);
        relocaliser_ = std::make_unique<Relocaliser>(config, std::move(loaded_map_));
        relocaliser_->start();
        RCLCPP_INFO(
          get_logger(), "relocaliser up: keyframes=%zu min_inliers=%zu focal_px=%.1f",
          relocaliser_->size(), config.place.min_inliers, config.place.focal_px);
      }
      Keyframe query;
      query.stamp_ns = when;
      query.odom_from_camera = odom_from_camera;
      query.descriptors = record->descriptors.clone();
      query.track_ids = record->ids;
      query.bearings.reserve(record->pixels.size());
      for (const cv::Point2f & pixel : record->pixels) {
        query.bearings.push_back(bearing(k, pixel.x, pixel.y));
      }
      query.landmark_row = landmark_rows;
      query.landmarks = landmark_points;
      relocaliser_->submit(std::move(query));
      std::lock_guard<std::mutex> lock(reloc_mutex_);
      ++reloc_stats_.submitted;
    }
  }

  // --- Retiring the reference --------------------------------------------------
  //
  // Four ways, and the last two are not about geometry at all.
  //
  // The shared-landmark floor: a keyframe the camera has turned away from
  // satisfies both thresholds right up until it does not, and its last surviving
  // correspondences sit in one corner of the frame, which is exactly the geometry
  // a translation fit is worst on.
  //
  // **And a run of holds, which is a deadlock rather than a refinement.** The
  // reference was only replaced on a successful fit in the first version of this,
  // so the first time tracking was lost it was lost for good: measured on
  // bags/desk1 as `pose_ok` frozen at 188 while `held` climbed past 560. Taking a
  // new reference at the held pose bakes in whatever drift the lost stretch cost,
  // and that is the right trade — it is a pose that stopped being updated against
  // one that stopped being updated *forever*.
  const bool lost = advanced && shared < keyframe_min_shared_;
  const bool moved = advanced && keyframes_.would_insert(odom_from_camera);
  const bool stuck = !advanced && holds_ >= max_hold_frames_;
  if (!reference_.valid() || lost || moved || stuck) {
    if (stuck) {
      ++keyframes_on_stall_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%zu depth frames without a usable fit — taking a new reference view here. The "
        "translation since the last good one is lost, not estimated.", holds_);
      holds_ = 0;
      // The speed gate measures from the last *accepted* pose, so a stall has to
      // move its clock too. Leaving it behind makes `dt` grow with the stall and
      // the ceiling grow with it, which is the gate quietly relaxing exactly when
      // tracking is worst.
      last_shift_ns_ = when;
      have_shift_ = true;
    } else if (lost && !moved) {
      ++keyframes_on_loss_;
    }

    // --- Into the map, through the backend thread ------------------------------
    //
    // The same event as P7's keyframe, so the map's keyframes and the reference
    // views are one sequence. Every feature arrives with the map point tracking
    // associated it with, and the map checks each claim against the geometry
    // before it becomes an observation; every unclaimed feature with a depth
    // reading becomes a new point.
    //
    // **Handed over, not inserted here.** P14 did this synchronously, at 0.8-1.0 ms
    // a keyframe; P15's bundle adjustment is not a millisecond, so the whole of the
    // map's upkeep moved onto LocalMapper's thread. The keyframe becomes *pending*
    // and is offered below, on this frame and every one after until the queue takes
    // it: a full queue defers a keyframe, it does not drop it.
    {
      pimesh_backend::KeyframeInput input;
      input.stamp_ns = when;
      input.map_from_camera = odom_from_camera;
      input.k = k;
      input.pixels = record->pixels;
      input.track_ids = record->ids;
      input.descriptors = record->descriptors;
      input.network_points.assign(n_features, cv::Vec3d(0.0, 0.0, 0.0));
      input.has_depth.assign(n_features, 0);
      for (std::size_t j = 0; j < landmark_rows.size(); ++j) {
        const auto row = static_cast<std::size_t>(landmark_rows[j]);
        input.network_points[row] = landmark_points[j];
        input.has_depth[row] = 1;
      }
      input.matched = feature_point;
      if (pending_keyframe_) {
        // The one real drop in this path, and it is counted: a keyframe still waiting
        // when the next one is due. gates/ba.sh asserts it stays at zero.
        ++keyframes_dropped_;
      }
      pending_keyframe_ = std::move(input);
    }

    Keyframe frame;
    frame.stamp_ns = when;
    frame.odom_from_camera = odom_from_camera;
    frame.descriptors = record->descriptors;
    frame.track_ids = record->ids;
    frame.bearings.reserve(record->pixels.size());
    for (const cv::Point2f & pixel : record->pixels) {
      frame.bearings.push_back(bearing(k, pixel.x, pixel.y));
    }
    frame.landmark_row = std::move(landmark_rows);
    frame.landmarks = std::move(landmark_points);
    // A keyframe the store admits is a keyframe place recognition searches from and,
    // afterwards, searches *for*. A deep copy: the descriptors are the tracker's
    // matrix, and the search thread must not share memory with the pose thread.
    std::optional<Keyframe> for_places;
    if (place_recognition_ && keyframes_.would_insert(frame.odom_from_camera)) {
      for_places = frame;
      for_places->descriptors = frame.descriptors.clone();
    }
    const bool inserted = keyframes_.maybe_insert(std::move(frame));
    // #13's P20: the map file, at most every 5 s of stamps while keyframes arrive. On
    // this thread because the store is this thread's; ~40 kB a keyframe, measured and
    // printed as save_ms rather than assumed cheap.
    if (inserted && !map_save_path_.empty() && keyframes_.size() != keyframes_at_last_save_ &&
      when - last_map_save_ns_ >= 5000000000LL)
    {
      last_map_save_ns_ = when;
      keyframes_at_last_save_ = keyframes_.size();
      save_map("periodic");
    }
    if (inserted && for_places) {
      if (!places_) {
        place_config_.place.focal_px = k(0, 0);
        places_ = std::make_unique<PlaceRecognizer>(place_config_);
        places_->start();
        // Read by gates/place.sh, so the instrument judges availability by the rule
        // the node used rather than by a second copy of it.
        RCLCPP_INFO(
          get_logger(),
          "place recognition up: min_gap_s=%.2f max_shared_tracks=%zu min_ransac_inliers=%zu "
          "min_inliers=%zu focal_px=%.1f max_keyframes=%zu",
          place_config_.place.min_gap_s, place_config_.place.max_shared_tracks,
          place_config_.place.min_ransac_inliers, place_config_.place.min_inliers,
          place_config_.place.focal_px, place_config_.max_keyframes);
      }
      places_->submit(std::move(*for_places));
    }

    view.odom_from_camera = odom_from_camera;
    reference_ = std::move(view);
  }

  // --- Offer a pending keyframe to the backend ------------------------------------
  //
  // Every depth frame, until the queue takes it. Refused means the backend is still
  // busy with the ones before; the keyframe is a keyframe at *its* stamp and pose
  // whenever it gets in, so waiting a frame costs nothing but the wait.
  if (pending_keyframe_) {
    if (mapper_->try_push(*pending_keyframe_)) {
      pending_keyframe_.reset();
      if (when - last_map_points_ns_ >= static_cast<std::int64_t>(map_points_period_s_ * 1e9)) {
        last_map_points_ns_ = when;
        publish_map_points(rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));
      }
    } else {
      ++keyframes_deferred_;
    }
  }

  // --- What place recognition found since the last frame ---------------------------
  //
  // One line per query, at the keyframe rate (~2 Hz), because gates/place.sh judges
  // every one of them against ground truth — the refused as well as the accepted,
  // since a revisit that was missed is the recall and a refusal's margin is the
  // distance to a false closure. The rotation is a Rodrigues vector.
  if (places_) {
    for (const PlaceResult & r : places_->take_results()) {
      const cv::Vec3d rvec = r.best.query_from_candidate.rvec();
      const cv::Vec3d t = r.best.query_from_candidate.translation();
      const cv::Vec3d orvec = r.best.odom_query_from_candidate.rvec();
      const cv::Vec3d ot = r.best.odom_query_from_candidate.translation();
      RCLCPP_INFO(
        get_logger(),
        "place query_ns=%lld accepted=%d searched=%zu too_recent=%zu still_tracked=%zu verified=%zu "
        "match_ns=%lld matches=%zu with_landmark=%zu ransac_inliers=%zu inliers=%zu "
        "rx=%.5f ry=%.5f rz=%.5f tx=%.4f ty=%.4f tz=%.4f odom_rot_dis_deg=%.2f "
        "orx=%.5f ory=%.5f orz=%.5f otx=%.4f oty=%.4f otz=%.4f",
        static_cast<long long>(r.query_stamp_ns), r.accepted ? 1 : 0, r.searched, r.too_recent,
        r.still_tracked, r.verified, r.verified > 0 ? static_cast<long long>(r.best.candidate_stamp_ns) : -1LL,
        r.best.matches, r.best.with_landmark, r.best.ransac_inliers, r.best.inliers,
        rvec[0], rvec[1], rvec[2], t[0], t[1], t[2], r.best.odom_rotation_disagreement_deg,
        orvec[0], orvec[1], orvec[2], ot[0], ot[1], ot[2]);
    }
  }

  publish_corrections();

  pose_cost_sum_ms_ = pose_cost_sum_ms_.load() +
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

bool OdometryNode::ensure_basis(const std::string & optical_frame)
{
  std::lock_guard<std::mutex> lock(basis_mutex_);
  if (have_basis_) {return true;}

  try {
    // TimePointZero: "the latest available", which for a latched static
    // transform is the only sensible request — asking for it *at* the frame's
    // stamp would compare the Pi's clock with this machine's and wait for a
    // transform that is already there.
    const auto tf = tf_buffer_->lookupTransform(base_frame_, optical_frame, tf2::TimePointZero);
    const auto & q = tf.transform.rotation;
    const auto & t = tf.transform.translation;
    tf2::Matrix3x3 m(tf2::Quaternion(q.x, q.y, q.z, q.w));
    base_from_optical_ = cv::Affine3d(
      cv::Matx33d(
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]),
      cv::Vec3d(t.x, t.y, t.z));
    have_basis_ = true;
    RCLCPP_INFO(
      get_logger(), "basis %s <- %s from TF", base_frame_.c_str(), optical_frame.c_str());
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "no %s <- %s transform yet (%s) — holding pose",
      base_frame_.c_str(), optical_frame.c_str(), ex.what());
    return false;
  }
}

void OdometryNode::update_pose(
  const std::vector<PixelPair> & pairs, const std::string & optical_frame)
{
  cv::Matx33d k;
  {
    std::lock_guard<std::mutex> lock(k_mutex_);
    if (!have_k_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no /camera_info yet — holding pose. Rays need K, and a pose from invented "
        "intrinsics is confidently wrong rather than absent.");
      ++pose_held_;
      return;
    }
    k = k_;
  }

  if (!ensure_basis(optical_frame)) {
    ++pose_held_;
    return;
  }

  if (pairs.size() < min_pairs_) {
    if (!holding_) {
      RCLCPP_INFO(
        get_logger(), "regime=hold: %zu consecutive pairs, need %zu",
        pairs.size(), min_pairs_);
      holding_ = true;
    }
    ++pose_held_;
    return;
  }

  std::vector<cv::Vec3d> from;
  std::vector<cv::Vec3d> to;
  from.reserve(pairs.size());
  to.reserve(pairs.size());
  for (const PixelPair & pair : pairs) {
    from.push_back(bearing(k, pair.previous.x, pair.previous.y));
    to.push_back(bearing(k, pair.current.x, pair.current.y));
  }

  const RotationFit fit =
    fit_rotation_robust(from, to, min_pairs_, max_residual_rad_, reject_fraction_);

  if (!fit.ok) {
    if (!holding_) {
      RCLCPP_INFO(
        get_logger(),
        "regime=hold: %zu/%zu pairs, residual %.4f rad (ceiling %.4f) — keeping the last pose",
        fit.pairs_used, fit.pairs_in, fit.residual_rad, max_residual_rad_);
      holding_ = true;
    }
    ++pose_held_;
    return;
  }

  if (holding_) {
    RCLCPP_INFO(
      get_logger(), "regime=rotation_only: recovered, %zu pairs at %.4f rad",
      fit.pairs_used, fit.residual_rad);
    holding_ = false;
  }

  // **`camera_step` is the correction this line waited six days for.** The fit maps
  // the previous frame's rays onto this frame's — it is the motion of the *rays*,
  // which is the camera's motion inverted. Composing the fit itself, as this did
  // from P3 until 2026-09-19, publishes a pose that turns left when the camera pans
  // right; nothing failed, because a TF frame that moves when you pan looks
  // correct. See rgbd_odometry.hpp and test_rgbd_odometry's CameraStep suite.
  //
  // The increment is in the optical frame and the pose is in the body frame, so
  // the basis changes first — and it multiplies on the right, because this is a
  // rotation of the camera relative to where it was, not relative to odom.
  const cv::Matx33d step = change_basis(
    base_from_optical_.rotation(), camera_step(fit.rotation));
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_ = cv::Affine3d(pose_.rotation() * step, pose_.translation());
  }
  residual_sum_ = residual_sum_.load() + fit.residual_rad;
  ++pose_ok_;
}

void OdometryNode::track(bool posed, const builtin_interfaces::msg::Time & stamp)
{
  auto out = std::make_unique<pimesh_msgs::msg::TrackingState>();
  out->header.stamp = stamp;
  out->header.frame_id = base_frame_;
  out->posed = posed;
  {
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    const bool changed = tracking_->observe(posed);
    out->state = tracking_->state() == Tracking::Ok ?
      pimesh_msgs::msg::TrackingState::OK : pimesh_msgs::msg::TrackingState::LOST;
    out->holds = static_cast<std::uint32_t>(tracking_->holds());
    // One line per transition, with the stamp, because tools/gates/lost.sh measures
    // how many depth frames each took against a blackout it injected by stamp.
    if (changed && tracking_->state() == Tracking::Lost) {
      RCLCPP_WARN(
        get_logger(),
        "tracking LOST stamp_ns=%lld after %zu depth frames without a fit — fusion_node "
        "stops integrating until %zu fits in a row",
        static_cast<long long>(rclcpp::Time(stamp).nanoseconds()), tracking_->holds(),
        tracking_->config().recover_after_fits);
    } else if (changed) {
      RCLCPP_INFO(
        get_logger(),
        "tracking OK stamp_ns=%lld after %zu fits in a row; %llu depth frames LOST so far",
        static_cast<long long>(rclcpp::Time(stamp).nanoseconds()), tracking_->fits(),
        static_cast<unsigned long long>(tracking_->lost_frames()));
    }
  }
  tracking_pub_->publish(std::move(out));
}

void OdometryNode::publish_pose(const rclcpp::Time & stamp)
{
  {
    std::lock_guard<std::mutex> lock(basis_mutex_);
    if (!have_basis_) {return;}
  }

  cv::Affine3d pose;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose = pose_;
  }
  const cv::Vec3d t(pose.translation());
  const cv::Vec4d q = quaternion_from_rotation(pose.rotation());

  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    // The *frame's* stamp, not now(): this transform describes where the camera was
    // when those pixels were captured. A pose stamped with the moment the estimator
    // finished is a pose that claims the camera was somewhere it had already left.
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = t[0];
    tf.transform.translation.y = t[1];
    tf.transform.translation.z = t[2];
    tf.transform.rotation.x = q[0];
    tf.transform.rotation.y = q[1];
    tf.transform.rotation.z = q[2];
    tf.transform.rotation.w = q[3];

    // `map -> odom`, **with the same stamp** — never now(), and never a stamp older
    // than one already sent. On a bag, now() is minutes ahead of the frames, and the
    // next frame-stamped transform would then be older than the newest tf2 holds:
    // TF_OLD_DATA at the frame rate, from inside the buffer's lock, which is the
    // stall CLAUDE.md records. A correction therefore takes effect at the next frame
    // and is never back-dated.
    cv::Affine3d correction = places_ ? places_->map_from_odom() : cv::Affine3d::Identity();
    // With a saved map the relocalisation is the one authority for this edge (the
    // constructor refuses loop closure beside it). Identity until the first one —
    // the state is LOST until then, so nothing downstream reads through it.
    if (!map_load_path_.empty()) {
      std::lock_guard<std::mutex> lock(reloc_mutex_);
      correction = reloc_map_from_odom_;
    }
    const cv::Vec3d ct(correction.translation());
    const cv::Vec4d cq = quaternion_from_rotation(correction.rotation());
    geometry_msgs::msg::TransformStamped map_tf;
    map_tf.header.stamp = stamp;
    map_tf.header.frame_id = map_frame_;
    map_tf.child_frame_id = odom_frame_;
    map_tf.transform.translation.x = ct[0];
    map_tf.transform.translation.y = ct[1];
    map_tf.transform.translation.z = ct[2];
    map_tf.transform.rotation.x = cq[0];
    map_tf.transform.rotation.y = cq[1];
    map_tf.transform.rotation.z = cq[2];
    map_tf.transform.rotation.w = cq[3];
    tf_broadcaster_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{map_tf, tf});
  }

  auto odom = std::make_unique<nav_msgs::msg::Odometry>();
  odom->header.stamp = stamp;
  odom->header.frame_id = odom_frame_;
  odom->child_frame_id = base_frame_;
  odom->pose.pose.position.x = t[0];
  odom->pose.pose.position.y = t[1];
  odom->pose.pose.position.z = t[2];
  odom->pose.pose.orientation.x = q[0];
  odom->pose.pose.orientation.y = q[1];
  odom->pose.pose.orientation.z = q[2];
  odom->pose.pose.orientation.w = q[3];
  // A large diagonal, not a -1 and not zeros. This node published `-1` in
  // element 0 from P7 until 2026-09-23, on the belief that nav_msgs documents it
  // the way sensor_msgs/Imu does; it does not, and the result was a matrix that
  // is not positive semidefinite, which RViz reported at the pose rate for the
  // length of every session. See unconstrained_covariance() for the whole story
  // and for why zeros is the worse of the two legal answers.
  //
  // `twist` stays zero-*valued* — nothing here estimates a velocity — and
  // carries the same unconstrained covariance, which is what says so.
  const auto covariance = unconstrained_covariance();
  std::copy(covariance.begin(), covariance.end(), odom->pose.covariance.begin());
  std::copy(covariance.begin(), covariance.end(), odom->twist.covariance.begin());
  odom_pub_->publish(std::move(odom));
}

void OdometryNode::publish_map_points(const rclcpp::Time & stamp)
{
  if (map_points_pub_->get_subscription_count() == 0 &&
    map_points_pub_->get_intra_process_subscription_count() == 0)
  {
    // Latched, but nothing is listening: the next keyframe will publish again, and
    // building a cloud of ten thousand points for nobody is the kind of work a
    // viewer's topic must never cost the pipeline.
    return;
  }
  const std::vector<cv::Vec3d> points = map_->positions();
  auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud->header.stamp = stamp;
  // odom, not map: the map -> odom edge is static identity until milestone H's pose
  // graph, and these positions are in the frame the poses are in.
  cloud->header.frame_id = odom_frame_;
  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(*cloud, "z");
  for (const cv::Vec3d & p : points) {
    *x = static_cast<float>(p[0]);
    *y = static_cast<float>(p[1]);
    *z = static_cast<float>(p[2]);
    ++x;
    ++y;
    ++z;
  }
  map_points_pub_->publish(std::move(cloud));
}

double OdometryNode::net_displacement()
{
  // **Reported beside the path length because the two answer different
  // questions, and only together do they say anything.** A path length is the sum
  // of every step, so an estimator whose steps are pure noise reports a *large*
  // one — 89.5 m over a 45 s desk sweep, in the run that made P7's estimator get
  // rewritten. The net displacement of that same run was a fraction of it, which
  // is the signature of a random walk rather than of a camera going somewhere.
  // P7's claim is about where the camera got to; the path length is what says how
  // smoothly it got there.
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return cv::norm(cv::Vec3d(pose_.translation()));
}

void OdometryNode::publish_corrections()
{
  if (!places_) {return;}
  const std::uint64_t solves = places_->stats().solves;
  if (solves == corrections_published_for_) {return;}
  corrections_published_for_ = solves;
  auto path = std::make_unique<nav_msgs::msg::Path>();
  path->header.frame_id = map_frame_;
  for (const auto & [stamp_ns, correction] : places_->corrections()) {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
    p.header.frame_id = odom_frame_;
    const cv::Vec3d t(correction.translation());
    const cv::Vec4d q = quaternion_from_rotation(correction.rotation());
    p.pose.position.x = t[0];
    p.pose.position.y = t[1];
    p.pose.position.z = t[2];
    p.pose.orientation.x = q[0];
    p.pose.orientation.y = q[1];
    p.pose.orientation.z = q[2];
    p.pose.orientation.w = q[3];
    path->poses.push_back(p);
  }
  if (path->poses.empty()) {return;}
  path->header.stamp = path->poses.back().header.stamp;
  corrections_pub_->publish(std::move(path));
}

void OdometryNode::write_keyframe_trajectory()
{
  if (keyframe_trajectory_path_.empty() || !places_) {return;}
  std::vector<TumPose> poses;
  for (const auto & [stamp_ns, map_from_camera] : places_->trajectory()) {
    const cv::Vec3d t(map_from_camera.translation());
    const cv::Vec4d q = quaternion_from_rotation(map_from_camera.rotation());
    poses.push_back(TumPose{stamp_ns, t[0], t[1], t[2], q[0], q[1], q[2], q[3]});
  }
  // Written beside the target and renamed over it: a session killed mid-write leaves
  // the last complete trajectory, not half of this one — a truncated file is a
  // shorter trajectory to evo, and a shorter trajectory is a plausible one.
  const std::string partial = keyframe_trajectory_path_ + ".partial";
  const std::string why = write_tum_trajectory(partial, poses);
  if (!why.empty()) {
    RCLCPP_DEBUG(get_logger(), "keyframe trajectory not written: %s", why.c_str());
    return;
  }
  if (std::rename(partial.c_str(), keyframe_trajectory_path_.c_str()) != 0) {
    RCLCPP_WARN(get_logger(), "could not move the keyframe trajectory into %s",
      keyframe_trajectory_path_.c_str());
  }
}

void OdometryNode::log_stats()
{
  write_keyframe_trajectory();
  const rclcpp::Time stamp = now();
  const double span_s = (stamp - last_log_).seconds();
  if (span_s <= 0.0) {return;}

  const std::uint64_t keypoints_now = keypoint_frames_.load();
  const std::uint64_t keypoints_delta = keypoints_now - last_keypoint_frames_;
  const std::uint64_t depth_now = depth_frames_.load();
  const std::uint64_t depth_delta = depth_now - last_depth_frames_;
  last_keypoint_frames_ = keypoints_now;
  last_depth_frames_ = depth_now;
  last_log_ = stamp;

  if (keypoints_delta == 0) {
    RCLCPP_WARN(
      get_logger(), "stats no keypoints in %.1fs — is keypoint_node running?", span_s);
    return;
  }

  const std::uint64_t ok = pose_ok_.load();
  const std::uint64_t held = pose_held_.load();
  const std::uint64_t shifted = shift_ok_.load();
  const std::uint64_t shift_held = shift_held_.load();
  const double shifted_d = static_cast<double>(shifted);
  const double depth_total = static_cast<double>(depth_now);
  const double keypoints_total = static_cast<double>(keypoints_now);

  // Poses published per second over *this window*. In sixdof that is one per
  // depth frame, in rotation_only one per keypoints frame.
  const double pose_rate_hz = (regime_ == OdometryRegime::SixDof) ?
    static_cast<double>(depth_delta) / span_s :
    static_cast<double>(keypoints_delta) / span_s;

  // One line, and every number in it is needed to read any of the others.
  //
  // `regime=` is first so that a gate reading this line never has to infer which
  // estimator produced the numbers after it, and `traj=` is the headline of P7:
  // rotation_only reports exactly 0.000 there whatever the camera did.
  RCLCPP_INFO(
    get_logger(),
    "stats regime=%s rate=%.1fHz kp_in=%.1fHz pose_ok=%lu held=%lu reject_rate=%.3f "
    "residual=%.4frad rotation_cost=%.2fms traj=%.4fm net=%.4fm depth_in=%lu "
    "depth_lost=%lu shift_ok=%lu shift_held=%lu landmarks=%.0f shared=%.0f "
    "inliers=%.0f reproj_px=%.3f point_residual=%.4fm depth_scale=%.4f "
    "pose_cost=%.2fms keyframes=%zu keyframe_kb=%.1f kf_on_loss=%lu kf_on_stall=%lu "
    "implausible=%lu malformed=%lu",
    regime_name(regime_),
    pose_rate_hz,
    static_cast<double>(keypoints_delta) / span_s,
    static_cast<unsigned long>(ok), static_cast<unsigned long>(held),
    (ok + held > 0) ? static_cast<double>(held) / static_cast<double>(ok + held) : 0.0,
    (ok > 0) ? residual_sum_.load() / static_cast<double>(ok) : 0.0,
    (keypoints_total > 0.0) ? rotation_cost_sum_ms_.load() / keypoints_total : 0.0,
    trajectory_m_.load(),
    net_displacement(),
    static_cast<unsigned long>(depth_now),
    static_cast<unsigned long>(depth_unmatched_.load()),
    static_cast<unsigned long>(shifted), static_cast<unsigned long>(shift_held),
    (depth_total > 0.0) ? landmark_sum_.load() / depth_total : 0.0,
    (depth_total > 0.0) ? pair_sum_.load() / depth_total : 0.0,
    (shifted > 0) ? inlier_sum_.load() / shifted_d : 0.0,
    (shifted > 0) ? reprojection_sum_px_.load() / shifted_d : 0.0,
    (shifted > 0) ? point_residual_sum_.load() / shifted_d : 0.0,
    (shifted > 0) ? scale_sum_.load() / shifted_d : 1.0,
    (depth_total > 0.0) ? pose_cost_sum_ms_.load() / depth_total : 0.0,
    keyframes_.size(),
    static_cast<double>(keyframes_.bytes()) / 1024.0,
    static_cast<unsigned long>(keyframes_on_loss_.load()),
    static_cast<unsigned long>(keyframes_on_stall_.load()),
    static_cast<unsigned long>(implausible_.load()),
    static_cast<unsigned long>(malformed_.load()));

  // --- The map, on a line of its own (#11's P14) -----------------------------------
  //
  // A second line rather than more fields on the first, and prefixed `stats map` so
  // that nothing parsing `stats regime=` can pick a number off it — the
  // gates/keypoints.sh lesson, where a second node logging the same prefix put
  // `cost_mean=0.00` under an 8 ms budget. Every figure tools/gates/map.sh asserts
  // or prints is on it.
  {
    const pimesh_backend::MapStats map = map_->stats();
    const pimesh_backend::LocalMapper::Stats backend = mapper_->stats();
    std::vector<double> local;
    double by_track_mean = 0.0;
    double by_projection_mean = 0.0;
    {
      std::lock_guard<std::mutex> lock(map_stats_mutex_);
      local = local_keyframe_samples_;
      if (map_tracked_frames_ > 0) {
        by_track_mean = static_cast<double>(by_track_sum_) / static_cast<double>(map_tracked_frames_);
        by_projection_mean =
          static_cast<double>(by_projection_sum_) / static_cast<double>(map_tracked_frames_);
      }
    }
    // **-1, not 0, for a figure nothing measured.** An empty local-map distribution
    // printed as a median of 0 reads as "tracked against nothing", and a triangulation
    // error of 0.000 reads as perfect; neither is what an empty sample says. An
    // unmeasured value and a good one must not have the same spelling.
    auto median = [](const std::vector<double> & v) {
        return v.empty() ? -1.0 : pimesh_core::percentile(v, 0.5);
      };
    auto p95 = [](const std::vector<double> & v) {
        return v.empty() ? -1.0 : pimesh_core::percentile(v, 0.95);
      };
    auto mean = [](double sum, std::uint64_t n) {return n > 0 ? sum / static_cast<double>(n) : -1.0;};
    RCLCPP_INFO(
      get_logger(),
      "stats map local_map=%s keyframes=%zu points=%zu obs3_frac=%.3f triangulated=%zu "
      "tri_err_px=%.3f depth_ratio=%.3f depth_ratio_p05=%.3f depth_ratio_p95=%.3f "
      "culled_points=%zu culled_keyframes=%zu judged_points=%zu judged_keyframes=%zu "
      "cull_runs=%lu "
      "local_kf_p50=%.1f local_kf_p95=%.1f local_kf_n=%zu by_track=%.1f by_projection=%.1f "
      "inserted=%lu associated=%lu created=%lu refused_dup=%lu refused_reproj=%lu "
      "refused_bad=%lu aligned=%lu align_mean=%.4f align_dev=%.4f map_cost=%.2fms",
      local_map_ ? "true" : "false",
      map.keyframes, map.points,
      map.points > 0 ? static_cast<double>(map.points_3plus) / static_cast<double>(map.points) : -1.0,
      map.triangulated,
      median(map.triangulation_error_px),
      median(map.depth_ratio),
      map.depth_ratio.empty() ? -1.0 : pimesh_core::percentile(map.depth_ratio, 0.05),
      p95(map.depth_ratio),
      map.culled_points, map.culled_keyframes, map.judged_points, map.judged_keyframes,
      static_cast<unsigned long>(backend.processed),
      median(local), p95(local), local.size(),
      by_track_mean, by_projection_mean,
      static_cast<unsigned long>(backend.processed), static_cast<unsigned long>(backend.associated),
      static_cast<unsigned long>(backend.created),
      static_cast<unsigned long>(backend.refused_duplicate),
      static_cast<unsigned long>(backend.refused_reprojection),
      static_cast<unsigned long>(backend.refused_bad),
      static_cast<unsigned long>(backend.aligned),
      mean(backend.align_sum, backend.aligned), mean(backend.align_dev_sum, backend.aligned),
      median(backend.keyframe_ms));

    // --- The backend thread (#11's P15), on a third line --------------------------
    //
    // Every figure tools/gates/ba.sh asserts on. `ba_runs` and `ba_iter` are the
    // plan's second false green — a BA that never runs is a run identical to the
    // control — and `kf_dropped` is the queue's promise that a keyframe is deferred
    // rather than lost.
    RCLCPP_INFO(
      get_logger(),
      "stats backend local_ba=%s ba_runs=%lu ba_refused=%lu ba_iter=%.1f ba_free=%.1f "
      "ba_fixed=%.1f ba_points=%.0f ba_edges=%.0f ba_ms=%.2f ba_ms_p95=%.2f "
      "ba_chi2_before=%.1f ba_chi2_after=%.1f ba_outliers=%lu kf_processed=%lu "
      "kf_deferred=%lu kf_refused_full=%lu kf_dropped=%lu keyframe_ms=%.2f "
      "keyframe_ms_p95=%.2f niced=%s ba_scale_dev=%.4f ba_scale_n=%lu",
      local_ba_ ? "true" : "false",
      static_cast<unsigned long>(backend.ba_runs), static_cast<unsigned long>(backend.ba_refused),
      mean(static_cast<double>(backend.iterations), backend.ba_runs),
      mean(static_cast<double>(backend.window_free), backend.ba_runs),
      mean(static_cast<double>(backend.window_fixed), backend.ba_runs),
      mean(static_cast<double>(backend.window_points), backend.ba_runs),
      mean(static_cast<double>(backend.window_edges), backend.ba_runs),
      median(backend.ba_ms), p95(backend.ba_ms),
      mean(backend.chi2_before, backend.ba_runs), mean(backend.chi2_after, backend.ba_runs),
      static_cast<unsigned long>(backend.outliers_dropped),
      static_cast<unsigned long>(backend.processed),
      static_cast<unsigned long>(keyframes_deferred_.load()),
      static_cast<unsigned long>(backend.refused_full),
      static_cast<unsigned long>(keyframes_dropped_.load()),
      median(backend.keyframe_ms), p95(backend.keyframe_ms),
      backend.niced ? "true" : "false",
      mean(backend.scale_dev_sum, backend.scale_solved),
      static_cast<unsigned long>(backend.scale_solved));
  }

  // --- Place recognition (#12's P16), on a line of its own -------------------------
  //
  // Cumulative. `queries` against `submitted` is how far behind the search thread
  // is; -1 for a cost nothing measured.
  {
    PlaceRecognizer::Stats places;
    if (places_) {places = places_->stats();}
    auto median = [](const std::vector<double> & v) {
        return v.empty() ? -1.0 : pimesh_core::percentile(v, 0.5);
      };
    auto p95 = [](const std::vector<double> & v) {
        return v.empty() ? -1.0 : pimesh_core::percentile(v, 0.95);
      };
    RCLCPP_INFO(
      get_logger(),
      "stats place enabled=%s submitted=%lu queries=%lu skipped=%lu accepted=%lu "
      "verified=%lu database=%zu query_ms=%.2f query_ms_p95=%.2f niced=%s "
      "loop_closure=%s loops=%lu solves=%lu inconsistent=%lu solve_ms=%.2f solve_ms_p95=%.2f "
      "correction_m=%.4f correction_deg=%.3f",
      place_recognition_ ? "true" : "false",
      static_cast<unsigned long>(places.submitted), static_cast<unsigned long>(places.queries),
      static_cast<unsigned long>(places.skipped), static_cast<unsigned long>(places.accepted),
      static_cast<unsigned long>(places.verified), places.database,
      median(places.query_ms), p95(places.query_ms), places.niced ? "true" : "false",
      place_config_.close_loops ? "true" : "false",
      static_cast<unsigned long>(places.loops), static_cast<unsigned long>(places.solves),
      static_cast<unsigned long>(places.inconsistent), median(places.solve_ms), p95(places.solve_ms),
      places.correction_m, places.correction_deg);
  }

  // --- The tracking state (#13's P19) ------------------------------------------
  //
  // Cumulative, and read by tools/gates/lost.sh. `frames` is the denominator the
  // monitor saw — every depth frame — so `lost_frames / frames` is the share of the
  // session fusion_node was told to refuse.
  Tracking tracking_now;
  {
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    tracking_now = tracking_->state();
    RCLCPP_INFO(
      get_logger(),
      "stats tracking state=%s frames=%lu lost_frames=%lu entered_lost=%lu recovered=%lu "
      "longest_hold_run=%zu lost_after_holds=%zu recover_after_fits=%zu",
      tracking_name(tracking_now), static_cast<unsigned long>(tracking_->frames()),
      static_cast<unsigned long>(tracking_->lost_frames()),
      static_cast<unsigned long>(tracking_->entered_lost()),
      static_cast<unsigned long>(tracking_->recovered()), tracking_->longest_hold_run(),
      tracking_->config().lost_after_holds, tracking_->config().recover_after_fits);
  }

  // --- The saved map and relocalisation (#13's P20) -------------------------------
  {
    Relocaliser::Stats searched;
    if (relocaliser_) {searched = relocaliser_->stats();}
    RelocStats r;
    {
      std::lock_guard<std::mutex> lock(reloc_mutex_);
      r = reloc_stats_;
    }
    auto pct = [](const std::vector<double> & v, double p) {
        return v.empty() ? -1.0 : pimesh_core::percentile(v, p);
      };
    RCLCPP_INFO(
      get_logger(),
      "stats reloc map_loaded=%s keyframes=%zu load_ms=%.1f file_mb=%.2f submitted=%lu "
      "queries=%lu skipped=%lu accepted=%lu applied=%lu ignored=%lu query_ms=%.2f "
      "query_ms_p95=%.2f niced=%s saves=%lu saved_keyframes=%zu save_ms=%.1f saved_mb=%.2f",
      map_load_path_.empty() ? "false" : "true", r.loaded, r.load_ms, r.file_mb,
      static_cast<unsigned long>(r.submitted), static_cast<unsigned long>(searched.queries),
      static_cast<unsigned long>(searched.skipped), static_cast<unsigned long>(searched.accepted),
      static_cast<unsigned long>(r.applied), static_cast<unsigned long>(r.ignored),
      pct(searched.query_ms, 0.5), pct(searched.query_ms, 0.95), searched.niced ? "true" : "false",
      static_cast<unsigned long>(r.saves), r.saved_keyframes, r.save_ms, r.saved_mb);
  }

  // --- The same numbers, on a topic, for P8's dashboard ------------------------
  //
  // **One row, where this was the second of two until the split.** The node
  // boundary and the stage boundary are the same thing now, so the dashboard's
  // `odometry` row is a node's own account of itself rather than a slice of
  // another node's. See PipelineStats.msg on why `stage` is a string.
  auto os = std::make_unique<pimesh_msgs::msg::PipelineStats>();
  os->header.stamp = stamp;
  os->stage = "odometry";
  const std::uint64_t posed = (regime_ == OdometryRegime::SixDof) ? shifted : ok;
  const std::uint64_t refused = (regime_ == OdometryRegime::SixDof) ? shift_held : held;
  os->rate_hz = static_cast<float>(pose_rate_hz);
  os->latency_ms = static_cast<float>(
    (regime_ == OdometryRegime::SixDof) ?
    ((depth_total > 0.0) ? pose_cost_sum_ms_.load() / depth_total : 0.0) :
    ((keypoints_total > 0.0) ? rotation_cost_sum_ms_.load() / keypoints_total : 0.0));
  os->latency_p95_ms = 0.0F;
  os->frames_in = posed + refused;
  os->frames_out = posed;
  // **Neither counter, and that is the honest answer.** A held pose is not a
  // frame dropped by design and it is not one lost in transport: the frame
  // arrived, was read, and the estimator declined to answer. `detail` carries it
  // rather than either column, because a refusal counted as a drop would make a
  // node that is working correctly on a blank wall look like one losing data.
  os->dropped_by_design = 0;
  os->dropped_in_transport = depth_unmatched_.load();
  char detail[192];
  std::snprintf(
    detail, sizeof(detail),
    "%s %s held=%lu traj=%.2fm net=%.2fm reproj=%.2fpx shared=%.0f keyframes=%zu",
    tracking_name(tracking_now), regime_name(regime_), static_cast<unsigned long>(refused),
    trajectory_m_.load(), net_displacement(),
    (shifted > 0) ? reprojection_sum_px_.load() / shifted_d : 0.0,
    (depth_total > 0.0) ? pair_sum_.load() / depth_total : 0.0,
    keyframes_.size());
  os->detail = detail;
  stats_pub_->publish(std::move(os));

  // A sixdof run with no depth is a session that will publish no pose at all, and
  // the symptom — a TF tree with a missing edge — sends people to look at the
  // static transforms. Said once per window rather than once, because the cause is
  // usually that depth_node is still loading its model.
  if (regime_ == OdometryRegime::SixDof && depth_now == 0) {
    RCLCPP_WARN(
      get_logger(),
      "regime=sixdof and nothing has arrived on the depth topic — no pose will be "
      "published until it does. Is depth_node running, and does its depth_topic "
      "match this node's?");
  }
}

}  // namespace pimesh_frontend

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_frontend::OdometryNode)
