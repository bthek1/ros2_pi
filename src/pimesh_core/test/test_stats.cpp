// The two helpers every probe in this package reports its numbers through.
//
// **These had no tests because they had no home.** `percentile` existed four
// times and FNV-1a twice, each in an anonymous namespace inside a translation
// unit with a ROS node in it — unreachable by any test, and free to drift apart.
// Every gate in this project prints a number that came out of one of them:
// `cost_p95`, `interval_p95_ms`, `matched_p05`, `offset_p95_ms`, and the
// `/depth/rgb` byte-identity count that closes P4's third claim.
//
// Nothing here needs a GPU, a camera or a running node, so this suite runs
// identically on both machines.
//
// The property worth stating up front: **every failure in this file is a wrong
// number rather than a crash.** A percentile off by one rank, or an FNV constant
// with a mistyped digit, produces output that is the right shape, the right
// magnitude, and quoted in a doc as measured fact.

#include <cstdint>
#include <numeric>
#include <utility>
#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "pimesh_core/stats.hpp"

using pimesh_core::fnv1a;
using pimesh_core::percentile;

// --- percentile: the edges ---------------------------------------------------

TEST(Percentile, EmptyIsZeroRatherThanNaN)
{
  // Every caller prints this straight into a gate's key=value output, and a NaN
  // there fails an awk comparison in a way that reads as a broken gate rather
  // than as a window in which nothing arrived.
  EXPECT_DOUBLE_EQ(percentile({}, 0.95), 0.0);
  EXPECT_DOUBLE_EQ(percentile({}, 0.0), 0.0);
}

TEST(Percentile, OneSampleIsThatSampleAtEveryFraction)
{
  for (const double fraction : {0.0, 0.05, 0.5, 0.95, 1.0}) {
    EXPECT_DOUBLE_EQ(percentile({7.5}, fraction), 7.5) << "fraction " << fraction;
  }
}

TEST(Percentile, FractionZeroIsTheMinimumAndOneIsTheMaximum)
{
  const std::vector<double> values{5.0, 1.0, 9.0, 3.0, 7.0};
  EXPECT_DOUBLE_EQ(percentile(values, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(percentile(values, 1.0), 9.0);
}

TEST(Percentile, DoesNotDependOnTheInputOrder)
{
  // nth_element reorders, so a version that accidentally returned values[index]
  // of the *original* ordering would still pass a single hand-picked case.
  std::vector<double> ascending(100);
  std::iota(ascending.begin(), ascending.end(), 1.0);          // 1 .. 100

  std::vector<double> shuffled = ascending;
  std::mt19937 generator(12345);
  std::shuffle(shuffled.begin(), shuffled.end(), generator);

  for (const double fraction : {0.0, 0.05, 0.5, 0.9, 0.95, 1.0}) {
    EXPECT_DOUBLE_EQ(percentile(ascending, fraction), percentile(shuffled, fraction))
      << "fraction " << fraction;
  }
}

TEST(Percentile, DoesNotDisturbTheCallersVector)
{
  // **Taken by value on purpose, and this is the test that keeps it that way.**
  // The callers hand it live accumulators they go on appending to between reports
  // — depth_probe's intervals_, keypoint_probe's matched_. Changing the signature
  // to a reference is a one-character tidy-up that would partially sort a probe's
  // sample buffer on every report, quietly reordering data that later reports are
  // still measuring.
  std::vector<double> values{5.0, 1.0, 9.0, 3.0, 7.0};
  const std::vector<double> before = values;

  percentile(values, 0.95);

  EXPECT_EQ(values, before) << "percentile reordered the caller's samples";
}

// --- percentile: the convention, pinned --------------------------------------

TEST(Percentile, IsTheFloorRankAndNeverInterpolates)
{
  // index = floor(fraction * n), so with n = 10 every returned value is one a real
  // sample had. An implementation that interpolated would return 5.5 here for 0.5
  // and 9.55 for 0.95 — plausible numbers that no frame ever cost.
  std::vector<double> values(10);
  std::iota(values.begin(), values.end(), 1.0);                // 1 .. 10

  EXPECT_DOUBLE_EQ(percentile(values, 0.5), 6.0);              // index 5
  EXPECT_DOUBLE_EQ(percentile(values, 0.25), 3.0);             // index 2
  EXPECT_DOUBLE_EQ(percentile(values, 0.95), 10.0);            // index 9, the max
}

TEST(Percentile, OnASmallSampleP95IsTheMaximum)
{
  // Documented in stats.hpp and asserted here so it stays a decision rather than
  // becoming a surprise: with n <= 20, floor(0.95 * n) is n - 1. That is the
  // honest answer — twelve samples have no 95th percentile distinct from the
  // largest — but it means a "p95" over a short window **is a worst case** and has
  // to be read as one. gates/depth.sh reports the worst window's p95 and labels it
  // an upper bound for exactly this reason.
  for (std::size_t n = 1; n <= 20; ++n) {
    std::vector<double> values(n);
    std::iota(values.begin(), values.end(), 1.0);
    EXPECT_DOUBLE_EQ(percentile(values, 0.95), static_cast<double>(n))
      << "n = " << n << " should give the maximum";
  }
  // 21 is the first size at which p95 is not simply the largest sample.
  std::vector<double> twenty_one(21);
  std::iota(twenty_one.begin(), twenty_one.end(), 1.0);
  EXPECT_DOUBLE_EQ(percentile(twenty_one, 0.95), 20.0);
}

TEST(Percentile, SitsOneRankAboveTextbookNearestRankOnExactBoundaries)
{
  // The one place this differs from the textbook definition, pinned so that
  // "our p95" stays a known quantity rather than an accident.
  //
  // Nearest rank takes the ceil(fraction * n)-th smallest; this takes the
  // floor(fraction * n) + 1-th. They agree unless fraction * n is an exact
  // integer. n = 100 is the case where it is.
  std::vector<double> hundred(100);
  std::iota(hundred.begin(), hundred.end(), 1.0);              // 1 .. 100
  EXPECT_DOUBLE_EQ(percentile(hundred, 0.95), 96.0);           // textbook: 95.0

  // n = 90: 0.95 * 90 = 85.5, not an integer, so both definitions give the 86th.
  std::vector<double> ninety(90);
  std::iota(ninety.begin(), ninety.end(), 1.0);
  EXPECT_DOUBLE_EQ(percentile(ninety, 0.95), 86.0);
}

TEST(Percentile, NeverReadsPastTheEndAtFractionOneOrBeyond)
{
  // floor(1.0 * n) is n, which indexes one past the last element — the clamp to
  // n - 1 is the only thing between this and undefined behaviour, and a fraction
  // above 1.0 is not rejected anywhere upstream.
  std::vector<double> values(7);
  std::iota(values.begin(), values.end(), 1.0);

  EXPECT_DOUBLE_EQ(percentile(values, 1.0), 7.0);
  EXPECT_DOUBLE_EQ(percentile(values, 1.5), 7.0);
  EXPECT_DOUBLE_EQ(percentile(values, 100.0), 7.0);
}

// --- fnv1a -------------------------------------------------------------------

TEST(Fnv1a, MatchesTheReferenceVectors)
{
  // The published 64-bit FNV-1a vectors. A mistyped digit in the offset basis or
  // the prime would still hash, still look random, and still answer "are these
  // two byte arrays equal" correctly — so nothing this project does with it would
  // ever notice. Pinned anyway: a hash that is *nearly* FNV-1a is one nobody can
  // compare against anything else.
  EXPECT_EQ(fnv1a(std::vector<std::uint8_t>{}), 0xcbf29ce484222325ULL);
  EXPECT_EQ(fnv1a(std::vector<std::uint8_t>{'a'}), 0xaf63dc4c8601ec8cULL);
  EXPECT_EQ(
    fnv1a(std::vector<std::uint8_t>{'f', 'o', 'o', 'b', 'a', 'r'}),
    0x85944171f73967e8ULL);
}

TEST(Fnv1a, OneFlippedByteChangesTheHash)
{
  // This is the whole property both callers rely on: depth_probe asks whether
  // /depth/rgb is byte-identical to the frame its depth was inferred on, and
  // capture_probe counts distinct camera frames. A hash that ignored a byte would
  // report a perfect match over a stream that was silently wrong.
  std::vector<std::uint8_t> bytes(4096, 0x5A);
  const std::uint64_t original = fnv1a(bytes);

  for (const std::size_t index : {std::size_t{0}, std::size_t{2048}, bytes.size() - 1}) {
    std::vector<std::uint8_t> altered = bytes;
    altered[index] ^= 0x01;
    EXPECT_NE(fnv1a(altered), original) << "a flipped byte at " << index << " was invisible";
  }
}

TEST(Fnv1a, LengthIsPartOfTheHash)
{
  // Zero bytes are the case a length-blind hash gets wrong: appending them must
  // change the answer, or a truncated image would match its full-length original.
  EXPECT_NE(
    fnv1a(std::vector<std::uint8_t>{0x00}),
    fnv1a(std::vector<std::uint8_t>{0x00, 0x00}));
  EXPECT_NE(
    fnv1a(std::vector<std::uint8_t>{1, 2, 3}),
    fnv1a(std::vector<std::uint8_t>{1, 2, 3, 0}));
}

TEST(Fnv1a, IsOrderSensitive)
{
  // A rearranged image is not the same image. A hash built by summing or xoring
  // bytes would call these equal.
  EXPECT_NE(
    fnv1a(std::vector<std::uint8_t>{1, 2, 3, 4}),
    fnv1a(std::vector<std::uint8_t>{4, 3, 2, 1}));
}

TEST(Fnv1a, ThePointerOverloadAgreesWithTheVectorOne)
{
  // depth_probe hashes a message's `data` vector; anything hashing a raw buffer
  // must get the same answer or the two could never be compared.
  const std::vector<std::uint8_t> bytes{9, 8, 7, 6, 5};
  EXPECT_EQ(fnv1a(bytes.data(), bytes.size()), fnv1a(bytes));
}

// --- quartiles ---------------------------------------------------------------

TEST(Quartiles, AgreeWithPercentileAtEverySampleCount)
{
  // **The pair that would otherwise be true when written and false a year
  // later.** `quartiles` exists because three `percentile` calls take three
  // copies of the vector, and `centred_patch_stats` makes them per frame over
  // tens of thousands of depth samples. The two must return the same ranks —
  // both spell the convention with `percentile_index`, and this is what says so
  // out loud rather than trusting the reader to notice.
  //
  // Swept across sizes on purpose: the convention has an off-by-one character to
  // it (`floor(fraction * n)`, one rank above nearest-rank when the product is an
  // integer), so a reimplementation agrees on most counts and disagrees on a few.
  // n = 4, 8, 12, 20, 100 are exactly the ones where `fraction * n` is an integer
  // for at least one quartile.
  for (std::size_t n = 1; n <= 200; ++n) {
    std::vector<double> values;
    values.reserve(n);
    // Deliberately unsorted, and not a permutation of 0..n-1: a helper that
    // happened to return the index rather than the value would pass over
    // 0, 1, 2, ...
    for (std::size_t i = 0; i < n; ++i) {
      values.push_back(static_cast<double>(((i * 37) % n) * 3 + 1));
    }
    // Moved in from an explicit copy rather than passed by value directly: GCC
    // 15 at -O2 emits a spurious -Wfree-nonheap-object for the copy it would
    // otherwise construct and destroy inside this loop's inlined body, and a
    // warning nobody can act on is a warning everybody learns to scroll past.
    // That `quartiles` does not disturb a caller's vector is asserted on its own
    // below, where it is the claim rather than an incidental.
    std::vector<double> scratch = values;
    const pimesh_core::Quartiles q = pimesh_core::quartiles(std::move(scratch));
    const double p25 = percentile(values, 0.25);
    const double p50 = percentile(values, 0.5);
    const double p75 = percentile(values, 0.75);
    EXPECT_DOUBLE_EQ(q.q1, p25) << "n = " << n;
    EXPECT_DOUBLE_EQ(q.median, p50) << "n = " << n;
    EXPECT_DOUBLE_EQ(q.q3, p75) << "n = " << n;
  }
}

TEST(Quartiles, DoNotReorderTheCallersVector)
{
  // Same contract as `percentile`: taken by value, because the callers hand it
  // live accumulators they go on using. A reference here would shuffle a probe's
  // sample buffer between reports, which is the kind of change that looks like a
  // tidy-up.
  std::vector<double> values{5.0, 1.0, 4.0, 2.0, 3.0};
  const std::vector<double> before = values;
  (void)pimesh_core::quartiles(values);
  EXPECT_EQ(values, before);
}

TEST(Quartiles, AreAllZeroOnAnEmptyInput)
{
  // `percentile`'s reason, and the same warning attaches: a caller that could see
  // an empty input has to branch on the *count*, because 0.0 is a plausible
  // value for every quantity this project measures.
  const pimesh_core::Quartiles q = pimesh_core::quartiles({});
  EXPECT_DOUBLE_EQ(q.q1, 0.0);
  EXPECT_DOUBLE_EQ(q.median, 0.0);
  EXPECT_DOUBLE_EQ(q.q3, 0.0);
}

TEST(Quartiles, OrderTheThreeTheWayTheirNamesSay)
{
  // q1 <= median <= q3, which a transposed pair of indices would break while
  // every individual value stayed a real sample. An IQR computed as q3 - q1 would
  // then come out negative, and a spread that is negative reads as zero spread to
  // anything comparing it against a ceiling.
  std::vector<double> values;
  for (int i = 0; i < 37; ++i) {values.push_back(static_cast<double>((i * 11) % 37));}
  const pimesh_core::Quartiles q = pimesh_core::quartiles(values);
  EXPECT_LE(q.q1, q.median);
  EXPECT_LE(q.median, q.q3);
}
