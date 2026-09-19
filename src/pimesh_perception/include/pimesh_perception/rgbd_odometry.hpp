#ifndef PIMESH_PERCEPTION__RGBD_ODOMETRY_HPP_
#define PIMESH_PERCEPTION__RGBD_ODOMETRY_HPP_

#include <cstddef>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"

namespace pimesh_perception
{

/// The 3D point a pixel and a depth reading name, in the optical frame.
///
/// **`z`, not ray length**, and the distinction is not pedantic: the depth maps
/// this project produces hold the perpendicular distance to the image plane, the
/// same convention `TsdfVolume` integrates under (`test_tsdf_volume` pins it — the
/// two agree perfectly at the principal point and disagree by 30% in the corners).
/// Treating a depth value as a distance along the ray would push every corner of
/// the frame outward and show up as a room that bulges.
cv::Vec3d unproject(const cv::Matx33d & k, double x, double y, double z);

/// One depth reading at a keypoint, or a refusal.
///
/// **A corner is the worst place in the image to read a depth map**, which is the
/// whole reason this is a function rather than `depth.at<float>(y, x)`. ORB puts
/// its features on edges and occlusion boundaries by construction, and that is
/// exactly where a monocular depth network's output steps between foreground and
/// background — so a single-pixel read at a corner lands on either side of the
/// step at random. The landmark then jumps metres between frames while looking
/// like a perfectly ordinary correspondence, and a rigid fit has no way to tell
/// that from real motion.
///
/// So: the median over a small patch, and a refusal when the patch is not flat.
/// `max_spread` is relative — a 10% spread at 1 m is 10 cm and at 5 m is 50 cm,
/// which is the right shape for a depth error that grows with distance.
///
/// Returns false, leaving `metres` untouched, when the patch is off the image, has
/// too few finite samples, falls outside [`min_m`, `max_m`], or straddles a step.
bool sample_depth(
  const cv::Mat & depth_32f, double x, double y, int patch,
  double min_m, double max_m, double max_spread, double & metres);

/// How the fit treats a difference of *scale* between the two clouds.
///
/// **This is the decision that makes 6-DoF odometry work on a monocular depth
/// network at all, and it was measured rather than reasoned.** Depth Anything V2
/// estimates *relative* depth, and its overall scale wobbles a few percent from
/// frame to frame — `fusion_node`'s scale aligner exists for exactly that, and on
/// `bags/desk1` it hits its own 15% clamp on 102 of 718 frames.
///
/// Write the measured landmarks as `Q_k = s_k * P_k`, with `P` the true geometry
/// and `s_k` the network's scale for frame k. True motion is
/// `P_cur = R P_prev + t`, so what the landmarks actually satisfy is
///
///     Q_cur = (s_cur / s_prev) * R * Q_prev + s_cur * t
///
/// — a **similarity**, whose scale factor is the breathing and whose translation
/// is the motion. Forcing the scale to 1 does not remove it; it makes the
/// translation absorb it, as `(a - 1)` times the landmark centroid's depth. At a
/// few percent and a centroid 3-5 m out that is 0.1-0.2 m of invented motion
/// along the view axis **every step**, in whichever direction the network happened
/// to breathe.
///
/// Measured on `bags/desk1`, 2026-09-19, and the difference is not subtle:
/// `Rigid` reported a **89.5 m** path over a 45 s desk sweep and made the surface
/// *worse* than rotation-only odometry (paired-surface gap 0.71 m against 0.25 m).
/// `DivideOut` is what P7 ships.
///
/// `Rigid` stays reachable because a threshold nobody has watched fail is not an
/// assertion: `tools/gates/odom.sh` is what compares them.
enum class ScaleHandling
{
  /// Force the scale to 1. Correct for a stereo or ToF sensor, wrong for this one.
  Rigid,
  /// Estimate the scale and **throw it away**, keeping only rotation and
  /// translation. What a camera cannot distinguish from a uniform scaling is a
  /// uniform scaling — and a forward translation is not one: points at 1 m and at
  /// 5 m move by the same vector, not by the same ratio, so real motion survives
  /// this and the breathing does not. A scene with no depth spread at all — a flat
  /// wall filling the frame — is the case where the two become degenerate, and
  /// there the scale wins. That is a real loss of forward motion in the one
  /// geometry where the alternative is 0.2 m of noise a step.
  DivideOut,
};

/// What a rigid fit came back with, and whether it should be believed.
///
/// `rotation` and `translation` describe the motion of the **points**:
/// `to[i] ~= scale * rotation * from[i] + translation`. That is not the camera's
/// motion — see camera_step(), which is where the inverse lives, written once and
/// named.
struct RigidFit
{
  /// True only if every gate passed. False means **hold the last pose**, the same
  /// policy the rotation-only path has had since P3: a wrong pose is worse than a
  /// stale one, because the TSDF bakes it into every voxel it touches and nothing
  /// downstream can tell which frames were guesses.
  bool ok {false};
  cv::Matx33d rotation {cv::Matx33d::eye()};
  cv::Vec3d translation {0.0, 0.0, 0.0};
  /// Mean distance, in metres, between a transformed source point and its target,
  /// over the pairs that survived rejection.
  double residual_m {0.0};
  /// The scale the two clouds differ by — the depth network's breathing, not
  /// motion. 1.0 exactly under `ScaleHandling::Rigid`. Reported because it is the
  /// number that says whether dividing it out was doing anything.
  double scale {1.0};
  std::size_t pairs_in {0};
  std::size_t pairs_used {0};
  std::size_t refits {0};

  /// The **metric** part, with the scale deliberately left out: this is what gets
  /// composed into a pose. Putting the scale in here would grow the map by a few
  /// percent a frame, which over a minute is a room the wrong size by orders of
  /// magnitude.
  cv::Affine3d motion() const {return cv::Affine3d(rotation, translation);}

  /// Where this fit says `point` went, scale included. The residual is measured
  /// against this and not against motion(), because the scale is part of the model
  /// of what the *landmarks* did even though it is no part of what the camera did.
  cv::Vec3d map(const cv::Vec3d & point) const {return scale * (rotation * point) + translation;}
};

/// The rigid transform that best maps `from` onto `to` — Kabsch about the
/// centroids, one SVD, no iteration.
///
/// The difference from fit_rotation() in rotation_fit.hpp is one line and it is
/// the whole of P7: these are **points in space**, not directions from a common
/// origin, so they have a centroid and removing it is what lets a translation
/// exist at all. Bearing rays have no centroid to remove, which is why the
/// rotation-only fit cannot see translation even in principle.
///
/// `scale` is estimated when `ScaleHandling::DivideOut` is asked for and left at
/// exactly 1.0 otherwise — see that enum for why this one is not a matter of
/// taste on a monocular relative-depth network.
RigidFit fit_rigid(
  const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to,
  ScaleHandling scale = ScaleHandling::DivideOut);

/// Mean distance, in metres, between `fit.map(from[i])` and `to[i]`.
double mean_residual_m(
  const RigidFit & fit,
  const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to);

/// Fit, throw away the worst pairs, fit again, then decide.
///
/// The same robust estimator shape as fit_rotation_robust(), and for the same
/// reason — a handful of confidently wrong correspondences drag a least-squares
/// fit by more than the motion being measured. It bites harder here: a rotation
/// fit's outliers are bounded by the frame (a ray can be at most 180 degrees
/// wrong), while a landmark whose depth came off the far side of an occlusion
/// edge is metres out and drags the translation with it.
///
/// The gates, and both are about refusing to answer:
///   - at least `min_pairs` surviving pairs;
///   - mean residual under `max_residual_m`.
/// Failing either sets `ok = false`, which the caller must read as *hold*.
RigidFit fit_rigid_robust(
  const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to,
  std::size_t min_pairs = 12, double max_residual_m = 0.05,
  double reject_fraction = 0.3, std::size_t refits = 2,
  ScaleHandling scale = ScaleHandling::DivideOut);

/// What a PnP solve came back with.
///
/// **This is P7's estimator, and the three that came before it are why.** The
/// obvious way to get a 6-DoF step out of RGB-D is to unproject both frames and
/// fit a rigid transform between the two point clouds. Measured on `bags/desk1`,
/// that does not work here, and the reason is not the estimator:
///
///   - both clouds carry the depth network's error, and it is *structured* — a
///     smooth warp over the frame, not per-pixel noise — so it does not average
///     down over three hundred landmarks the way independent noise would;
///   - the network's overall scale breathes a few percent a frame, which a rigid
///     fit can only absorb as translation along the view axis;
///   - and the signal is tiny beside all that: a hand sweep moves ~3 mm between
///     two depth frames, against a measured ~90 mm of per-sample estimate error.
///
/// Measured, in order: rigid 3D-3D frame to frame reported an **89.5 m** path over
/// a 45 s desk sweep; dividing the scale out left it at **117 m**; measuring
/// against a keyframe instead of the previous frame brought it to **44 m**; and a
/// low-pass on the position brought it to **8.4 m** with **3.6 m** of net drift.
/// Every one of those was still worse than publishing no translation at all, as
/// far as the surface could tell.
///
/// PnP changes the measurement rather than the filtering. The 3D points come from
/// the **keyframe's** depth map and the 2D points are the current frame's
/// *pixels*, so the current frame contributes no depth at all: one source of depth
/// error instead of two, no scale ratio to estimate between them, and the
/// translation determined by where the corners actually landed on the sensor —
/// which is the same mechanism monocular SLAM has always used and is exact to a
/// fraction of a pixel. It also gets ~2x the correspondences, because a current
/// keypoint needs no usable depth of its own to be used.
///
/// The residual is in **pixels**, which is the other reason to prefer it: the
/// metres this pipeline works in are arbitrary until `depth_scale` is pinned with
/// a tape measure, so a gate in metres is a gate on an unknown unit, and one in
/// pixels is a gate on the sensor.
struct PnpFit
{
  bool ok {false};
  /// The motion of the **points**: a point in the reference camera's frame maps to
  /// `rotation * point + translation` in this one. camera_step() is what turns it
  /// into where the camera went.
  cv::Matx33d rotation {cv::Matx33d::eye()};
  cv::Vec3d translation {0.0, 0.0, 0.0};
  /// Mean reprojection error of the inliers, in pixels.
  double residual_px {0.0};
  std::size_t pairs_in {0};
  std::size_t inliers {0};

  cv::Affine3d motion() const {return cv::Affine3d(rotation, translation);}
};

/// Solve the camera's pose from 3D landmarks and where they landed on the sensor.
///
/// `object` are points in the reference camera's optical frame; `image` are the
/// pixels the same features occupy in the frame being posed; `k` its intrinsics.
///
/// **No distortion coefficients, and that is measured rather than lazy.** The C922
/// at 720p has essentially none — over 243 marker-confirmed frames the correlation
/// between how far the board reached from the image centre and how bent its rows
/// were came out at −0.160, where a real radial distortion would make it strongly
/// positive — and every fit off a real set lands |k1| < 0.02 with the sign flipping
/// as frames are added. `bearing()` in rotation_fit.hpp ignores it for the same
/// reason. **Re-measure before assuming this holds at another resolution.**
///
/// RANSAC rather than a plain solve, because one landmark whose depth came off the
/// far side of an occlusion edge is metres out and a least-squares pose has no
/// defence against it. `reprojection_px` is the inlier threshold; `min_inliers` and
/// `max_residual_px` are the gates, and failing either means **hold**.
PnpFit fit_pose_pnp(
  const cv::Matx33d & k,
  const std::vector<cv::Vec3d> & object, const std::vector<cv::Point2f> & image,
  std::size_t min_inliers = 12, double reprojection_px = 3.0,
  double max_residual_px = 2.0, int iterations = 200);

/// What a translation-only fit came back with, the rotation having been settled
/// elsewhere.
///
/// **Splitting the two is worth a factor of three in translation error, measured
/// 2026-09-19.** A rotation is determined by *where the corners are in the image*
/// — pixels, refined to a fraction of one, with no depth in the answer at all. A
/// translation cannot be had without depth. Solving them together lets the depth
/// noise into the rotation and, worse, lets the fit trade one against the other;
/// solving the rotation from bearing rays first and the translation from the
/// landmarks afterwards uses each measurement where it is strongest. Over a
/// simulated hand sweep with 1.5% depth noise the per-step translation error fell
/// from 0.051 m to 0.016 m against a true 0.018 m of motion.
struct TranslationFit
{
  bool ok {false};
  cv::Vec3d translation {0.0, 0.0, 0.0};
  /// The depth network's breathing, as in RigidFit — 1.0 exactly under
  /// `ScaleHandling::Rigid`.
  double scale {1.0};
  double residual_m {0.0};
  std::size_t pairs_in {0};
  std::size_t pairs_used {0};
  std::size_t refits {0};

  cv::Vec3d map(const cv::Matx33d & rotation, const cv::Vec3d & point) const
  {
    return scale * (rotation * point) + translation;
  }
  cv::Affine3d motion(const cv::Matx33d & rotation) const
  {
    return cv::Affine3d(rotation, translation);
  }
};

/// Given the rotation, the translation (and the scale) that best carry `from`
/// onto `to`. Closed form, then the same reject-worst refits as everything else
/// here — an outlier landmark is metres out and a mean of three hundred of them
/// is not robust to one.
TranslationFit fit_translation_robust(
  const cv::Matx33d & rotation,
  const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to,
  std::size_t min_pairs = 12, double max_residual_m = 0.05,
  double reject_fraction = 0.3, std::size_t refits = 2,
  ScaleHandling scale = ScaleHandling::DivideOut);

/// The camera's movement, given the motion the *points* appeared to undergo.
///
/// A fit answers `P_cur = M * P_prev` for one world point seen in two frames. The
/// camera composes the other way round — the point did not move, the camera did —
/// so `T_world_cur = T_world_prev * M^-1`, and this function is that inverse.
///
/// **It exists as a named function because its absence was invisible for six
/// days.** From P3 until 2026-09-19 `update_pose()` composed the fitted rotation
/// itself, so the published `odom -> base_link` turned *left* when the camera
/// panned right. Nothing failed: a TF frame that moves when you pan looks correct
/// in RViz, the residual gate is indifferent to the sign, and a TSDF built from
/// consistently mirrored poses still produces a surface. `test_rgbd_odometry`
/// closes the loop that `test_rotation_fit` left open — it simulates a camera with
/// a known motion, runs the fit, and asserts the composed pose is the motion that
/// was simulated rather than merely a rotation of the right size.
cv::Affine3d camera_step(const cv::Affine3d & point_motion);
cv::Matx33d camera_step(const cv::Matx33d & point_rotation);

/// Change the frame a rigid motion is expressed in: `M_b = E * M_a * E^-1`.
///
/// The rotation-only overload in rotation_fit.hpp is the same identity with the
/// translation dropped. Keeping the full transform matters as soon as
/// `base_link -> camera_optical_frame` acquires a lever arm: with a non-zero
/// offset, a pure camera rotation is a body rotation *plus* a translation, and
/// dropping it would model a camera swung about the wrist as one spinning on the
/// spot. That edge is identity today (the C922 is hand-held), which is precisely
/// why this has to be right before it is ever mounted — an identity basis hides
/// the mistake completely.
cv::Affine3d change_basis(const cv::Affine3d & basis, const cv::Affine3d & motion);

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__RGBD_ODOMETRY_HPP_
