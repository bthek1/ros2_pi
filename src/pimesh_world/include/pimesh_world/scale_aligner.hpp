#ifndef PIMESH_WORLD__SCALE_ALIGNER_HPP_
#define PIMESH_WORLD__SCALE_ALIGNER_HPP_

#include <cstddef>
#include <deque>

#include "opencv2/core.hpp"

namespace pimesh_world
{

/// What the aligner concluded about one frame.
struct ScaleResult
{
  /// Multiply the incoming depth by this before integrating. Exactly 1.0 when
  /// there was nothing to align against, which is a refusal and not a correction.
  double scale {1.0};
  /// Fraction of pixels where both the ray-cast and the incoming depth are valid.
  double overlap {0.0};
  /// The raw median expected/incoming over that overlap, before the high-pass.
  double ratio {1.0};
  /// The rolling-median baseline the ratio was measured against.
  double baseline {1.0};
  /// True only when a correction was actually computed and applied.
  bool aligned {false};
  /// True when the correction hit `max_correction` and was cut short.
  bool clamped {false};
};

/// Median of `expected / incoming` over the pixels where both are valid.
///
/// Returns the overlap fraction through `overlap` and `false` when that fraction
/// is below `min_overlap` — a mostly-new view has nothing to conform to, and the
/// honest answer there is "I don't know" rather than a ratio computed from the
/// handful of pixels that did overlap.
///
/// **Median, not mean, and that is the whole robustness story.** The overlap
/// contains genuinely new geometry (where the ray-cast hit an older surface the
/// camera has since moved past) and depth-model failures (where the incoming
/// frame is simply wrong). Both are outliers in the ratio distribution and both
/// would drag a mean; the median ignores them as long as they are under half the
/// pixels, which they are or the pose is wrong and no scale will save the frame.
bool depth_ratio(
  const cv::Mat & expected_m, const cv::Mat & incoming_m,
  double min_overlap, double & ratio, double & overlap);

/// Median absolute disagreement, in metres, between two depth images over their
/// valid overlap.
///
/// **This is the paired-surface number**, the one `tools/gates/fusion.sh` prints
/// with alignment on and off: given a surface the volume already holds and a new
/// view of the same wall, how far apart do the two say the wall is? The
/// predecessor's went 7.8 cm to 5.7 cm when the aligner was switched on.
///
/// Median again, and for the same reason: the parts of the frame that see new
/// geometry disagree by metres and are not what is being asked about.
bool surface_gap(
  const cv::Mat & expected_m, const cv::Mat & incoming_m,
  double min_overlap, double & gap_m, double & overlap);

/// The fraction of the incoming frame the map already agrees with, to within
/// `tolerance` **as a fraction of the distance**.
///
/// **The denominator is every valid pixel of the incoming frame, not the
/// overlap**, and that is the whole reason this exists beside `surface_gap`. A
/// gap taken over the overlap is conditioned on the ray-cast having found
/// something, so two maps with different amounts of surface in them are not being
/// asked the same question: a map full of spurious shingles finds *a* surface
/// almost everywhere, and whichever shingle is nearest may agree with the frame
/// by luck. Measured on bags/desk1, the median gap over the overlap was 1.3329 m
/// aligned against 1.3291 m unaligned — indistinguishable — while the overlap
/// itself was 0.858 against 0.748. The statistic was answering a different
/// question for each run.
///
/// **Relative rather than absolute**, because `depth_scale` is unpinned: until a
/// tape measure fixes it (P5, and it needs a person) every distance here is
/// plausibly shaped and the wrong size, and a tolerance in centimetres would be a
/// tolerance in unknown units.
bool surface_agreement(
  const cv::Mat & expected_m, const cv::Mat & incoming_m,
  double tolerance, double & fraction);

/// A high-pass corrector for the monocular depth model's frame-to-frame scale
/// wobble.
///
/// **Why this exists at all.** Depth Anything's output scale is not stable: the
/// predecessor measured ±4% frame to frame on a *static* scene, which at 2.5 m is
/// ±10 cm — seven of our voxels. A TSDF fed that averages it into layered
/// shingles instead of one wall, and no amount of weight fixes it, because the
/// observations genuinely disagree.
///
/// **Why it is a high-pass rather than "conform each frame to the map", which is
/// the obvious thing and is unstable.** Conforming directly puts the ray-caster
/// in the feedback loop, and any *constant* error in that loop compounds: the
/// predecessor measured its renderer reading systematically ~1.25 voxels far, and
/// the wall walked away at about +1% a frame until the map was meaningless. So
/// the correction applied is only the frame's *deviation* from a rolling median
/// of recent ratios. Per-frame wobble is fast and gets cancelled; renderer bias
/// and slow drift are slow, land in the baseline, and are never fed back. Over
/// the window the corrections have median exactly 1 by construction — no net push
/// on the map, which is the property that makes this safe to run in a loop.
///
/// **It does not make the map the right size and must never be asked to.**
/// Absolute scale belongs to `depth_node`'s tape-measured `depth_scale`; this
/// only makes consecutive frames agree with each other. A room that is
/// consistently 2.7x too big is exactly what this looks like when it is working.
class ScaleAligner
{
public:
  struct Options
  {
    /// Below this valid-overlap fraction the frame is mostly new geometry and
    /// there is nothing to conform to. Refuse rather than guess.
    double min_overlap {0.2};
    /// The clamp on one frame's correction. A wrong pose or a failed depth map
    /// can produce a ratio of 2 with a perfectly healthy-looking overlap, and
    /// applying it folds the map in a way no later frame undoes.
    double max_correction {0.15};
    /// How many recent ratios the baseline is the median of. 50 frames is ~3 s at
    /// the depth rate: long enough that per-frame wobble cannot move it, short
    /// enough to follow a real change in the scene.
    std::size_t window {50};
  };

  /// Two constructors for the same reason TsdfVolume has two: a default argument
  /// of `Options()` needs the nested class's default member initializers before
  /// the enclosing class is complete.
  ScaleAligner()
  : ScaleAligner(Options()) {}
  explicit ScaleAligner(const Options & options)
  : options_(options) {}

  ScaleResult scale_for(const cv::Mat & expected_m, const cv::Mat & incoming_m);

  void reset() {ratios_.clear();}
  std::size_t history() const {return ratios_.size();}
  const Options & options() const {return options_;}

private:
  Options options_;
  std::deque<double> ratios_;
};

}  // namespace pimesh_world

#endif  // PIMESH_WORLD__SCALE_ALIGNER_HPP_
