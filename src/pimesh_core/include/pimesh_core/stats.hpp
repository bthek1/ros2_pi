#ifndef PIMESH_PERCEPTION__STATS_HPP_
#define PIMESH_PERCEPTION__STATS_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace pimesh_core
{

/// The two helpers every probe in this package had its own copy of.
///
/// **They were identical, four times over, and that is the reason this header
/// exists rather than tidiness.** `percentile` appeared in `depth_node.cpp`,
/// `depth_probe.cpp` and `keypoint_probe_main.cpp`; FNV-1a appeared twice more
/// under two different names. Every one of those copies produces a number that
/// gets printed by a gate and quoted in a doc, and not one of them was reachable
/// by a test — they sat in anonymous namespaces inside translation units with a
/// ROS node in them. A number nobody can test is a number that is right by
/// assertion.
///
/// `pimesh_camera` keeps its own copy of both, deliberately. It is the Pi's
/// package — the sensor head runs it and nothing else — and a dependency edge from
/// it to this package would point the wrong way down the pipeline for the sake of
/// twelve lines. The duplication there is a known cost, written down here rather
/// than discovered later.

/// The value at rank `fraction` of `values`, by nearest rank, with no
/// interpolation.
///
/// Taken **by value** on purpose: it reorders what it is given (`nth_element` is a
/// partial sort) and the callers hand it live accumulators they go on using.
/// Changing this to a reference would silently shuffle a probe's sample buffer
/// between reports, which is the kind of change that looks like a tidy-up.
///
/// **No interpolation, deliberately.** These summarise windows of tens to a few
/// hundred samples — per-frame costs, arrival intervals, matched fractions — and
/// interpolating between two neighbours invents a number that no frame actually
/// cost. Every value this returns is one a real sample had.
///
/// **Two properties worth knowing before quoting the result**, both asserted in
/// test/test_stats.cpp so they cannot drift into being accidents:
///
///  - **On a small sample, `percentile(v, 0.95)` is the maximum.** The index is
///    `floor(fraction * n)`, so any `n <= 20` puts 0.95 at the last element. That
///    is the honest answer — with 12 samples there is no 95th percentile distinct
///    from the largest — but it means a "p95" over a short window is a worst case,
///    and should be read as one.
///  - **It sits one rank above the textbook nearest-rank definition when
///    `fraction * n` is an exact integer**, and agrees everywhere else. Nearest
///    rank takes the `ceil(fraction * n)`-th smallest; this takes the
///    `floor(fraction * n) + 1`-th. For n = 90 and 0.95 both give the 86th; for
///    n = 100 they give the 96th and the 95th. The difference is one sample in the
///    tail of a window and it is written down rather than corrected, because every
///    number this project has published — `interval_p95_ms`, `cost_p95`,
///    `matched_p05`, `offset_p95_ms` — was measured with this convention, and
///    silently changing it would move published figures without moving anything
///    real.
inline double percentile(std::vector<double> values, double fraction)
{
  // Empty is 0.0 rather than NaN because every caller prints this straight into a
  // gate's key=value output, and a NaN there fails an `awk` comparison in a way
  // that reads as a broken gate rather than as an empty window. The callers that
  // could legitimately see an empty window guard it themselves.
  if (values.empty()) {return 0.0;}
  const std::size_t index = std::min(
    values.size() - 1,
    static_cast<std::size_t>(fraction * static_cast<double>(values.size())));
  std::nth_element(
    values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
  return values[index];
}

/// FNV-1a over a byte range: a cheap 64-bit summary that differs when any byte
/// does.
///
/// Not a cryptographic choice — nothing here is adversarial. It is used for two
/// things, and both are equality questions rather than security ones: counting
/// *distinct* camera frames (a driver replaying a buffer publishes at full rate
/// with identical payloads, which is a real rate at the DDS layer and a fake one
/// as a statement about the sensor), and asking whether `/depth/rgb` is
/// byte-identical to the frame its depth map was inferred on without pinning
/// 2.7 MB per in-flight stamp.
///
/// The offset basis and prime are the standard 64-bit FNV-1a constants, **in hex**,
/// and they are checked against the published reference vectors in
/// test/test_stats.cpp.
///
/// **That test found them wrong the first time it ran, 2026-09-15.** Both copies
/// of this function — here and in `pimesh_camera`'s `capture_probe` — carried the
/// basis in decimal as `1469598103934665603`, which is the correct
/// `14695981039346656037` with its last digit missing: a truncated paste, present
/// since milestone A. Nothing had ever noticed, and nothing could have. A wrong
/// basis is still a perfectly serviceable hash — it avalanches the same way and it
/// answers "are these two byte arrays equal" correctly every time — so every
/// number this project has published off the back of it (`duplicate payloads`,
/// `/depth/rgb identical`) was and remains right. It simply was not FNV-1a, while
/// two comments said it was.
///
/// Hex, because that is the form the constant is published in and a dropped digit
/// in it does not silently make a different valid-looking number. This is the
/// cheapest possible example of the rule this project keeps relearning: a wrong
/// thing that works is invisible until something compares it against the outside
/// world.
inline std::uint64_t fnv1a(const std::uint8_t * data, std::size_t size)
{
  std::uint64_t hash = 0xcbf29ce484222325ULL;
  for (std::size_t i = 0; i < size; ++i) {
    hash ^= data[i];
    hash *= 0x100000001b3ULL;
  }
  return hash;
}

inline std::uint64_t fnv1a(const std::vector<std::uint8_t> & bytes)
{
  return fnv1a(bytes.data(), bytes.size());
}

}  // namespace pimesh_core

#endif  // PIMESH_PERCEPTION__STATS_HPP_
