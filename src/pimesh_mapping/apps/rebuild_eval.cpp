// #12's P18 instrument: score a rebuild against motion capture.
//
// Reads a memory dump fusion_node wrote after its last rebuild, and scores two arms
// over the same remembered frames:
//
//   corrected   each frame at the pose graph's final correction applied to its odometry
//   control     each frame at its odometry pose alone — what no loop closure would give
//
// Each arm is rebuilt, ground truth is Sim(3)-aligned onto that arm, the same frames
// are rebuilt again at the aligned ground-truth poses as its reference, and the two
// volumes are ray-cast from those poses and compared. The depth readings are identical
// in all of them; only the poses differ. Prints `key=value` lines; exits non-zero only
// when it could not read its inputs, because a figure it could not compute is printed
// as -1 rather than guessed.
//
// Usage: rebuild_eval <memory_dump_dir> <groundtruth.txt>

#include <cstdio>
#include <string>
#include <vector>

#include "pimesh_mapping/ground_truth.hpp"
#include "pimesh_mapping/rebuild.hpp"

using pimesh_mapping::ArmScore;
using pimesh_mapping::Corrections;
using pimesh_mapping::RememberedFrame;

namespace
{

void print_arm(const char * name, const ArmScore & s)
{
  std::printf("%s_frames=%zu\n", name, s.frames);
  std::printf("%s_judged=%zu\n", name, s.judged);
  std::printf("%s_aligned=%d\n", name, s.alignment.ok ? 1 : 0);
  std::printf("%s_scale=%.4f\n", name, s.alignment.ok ? s.alignment.s : -1.0);
  std::printf("%s_ate_m=%.4f\n", name, s.ate_m);
  std::printf("%s_gap_m=%.4f\n", name, s.surface.gap_m);
  std::printf("%s_agree=%.4f\n", name, s.surface.agree);
  std::printf("%s_views=%zu\n", name, s.surface.views);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::fprintf(stderr, "usage: %s <memory_dump_dir> <groundtruth.txt>\n", argv[0]);
    return 2;
  }
  std::vector<RememberedFrame> frames;
  Corrections corrections;
  pimesh_mapping::TsdfVolume::Options options;
  int downsample = 0;
  const std::string why = pimesh_mapping::read_memory(argv[1], frames, corrections, options, downsample);
  if (!why.empty()) {
    std::fprintf(stderr, "error=%s\n", why.c_str());
    return 2;
  }
  const auto groundtruth = pimesh_mapping::load_tum_groundtruth(argv[2]);
  if (groundtruth.size() < 3) {
    std::fprintf(stderr, "error=fewer than three ground-truth poses in %s\n", argv[2]);
    return 2;
  }

  std::vector<cv::Affine3d> corrected, control;
  for (const RememberedFrame & f : frames) {
    corrected.push_back(pimesh_mapping::correction_at(corrections, f.stamp_ns) * f.odom_from_camera);
    control.push_back(f.odom_from_camera);
  }
  // fusion_node's own defaults for the comparison, so the offline number means what
  // the live `agree` does: 20% of the reference's area must overlap, and agreement is
  // within 5% of depth.
  constexpr double kMinOverlap = 0.2;
  constexpr double kTolerance = 0.05;
  const ArmScore c = pimesh_mapping::score_arm(frames, corrected, groundtruth, options, downsample,
    kMinOverlap, kTolerance);
  const ArmScore u = pimesh_mapping::score_arm(frames, control, groundtruth, options, downsample,
    kMinOverlap, kTolerance);
  std::printf("frames=%zu\n", frames.size());
  std::printf("corrections=%zu\n", corrections.size());
  std::printf("downsample=%d\n", downsample);
  print_arm("corrected", c);
  print_arm("control", u);
  return 0;
}
