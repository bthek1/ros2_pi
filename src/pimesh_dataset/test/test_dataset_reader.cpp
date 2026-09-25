// The dataset index parser, and the timestamp arithmetic underneath it.
//
// **Why this suite exists** — the same reason every other suite in this
// workspace does: a wrong version of this code produces a plausible result
// rather than a crash.
//
//   - `std::stod(text) * 1e9` is the obvious way to turn "1305031452.791720"
//     into nanoseconds. It is wrong by a few hundred ns and *nothing downstream
//     would ever say so*: evo associates within 10 ms, the pipeline is internally
//     consistent because it is wrong the same way everywhere, and the only
//     symptom is a trajectory whose stamps are not quite the dataset's. The first
//     test below is the one that separates the two spellings.
//   - An index whose timestamps repeat produces two frames at one stamp, and
//     `odometry_node` looks a frame up by *exact stamp* to pair it with a depth
//     map. That is not a duplicate frame; it is a second answer to a lookup.
//   - A listed image that is not on disk gives `imdecode` failures at 30 Hz, all
//     of them throttled, none of them naming the index that is wrong.
//
// No ROS, no dataset, no camera: everything here is a pure function over a string
// or over a directory this file creates, so it runs identically on the Pi.

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
#include "pimesh_dataset/dataset_reader.hpp"

using pimesh_dataset::FrameList;
using pimesh_dataset::parse_tum_timestamp;
using pimesh_dataset::read_tum_index;

namespace
{

/// A throwaway sequence tree that removes itself. `rgb.txt` plus whatever image
/// files the caller asks for, created empty — the reader checks that a listed
/// path *is a file*, and never opens it.
///
/// `images` is one whitespace-separated string rather than a container of names,
/// which reads no worse at the call sites and keeps a `std::vector<std::string>`
/// temporary out of every test body: GCC 15 at -O2 emits a spurious
/// `-Wfree-nonheap-object` for one inlined into a gtest `TestBody`, and a
/// warning nobody can act on is a warning everybody learns to scroll past.
class TempSequence
{
public:
  TempSequence(const std::string & index, const std::string & images)
  : root_(std::filesystem::temp_directory_path() /
      ("pimesh_dataset_" + std::to_string(::rand())))
  {
    std::filesystem::create_directories(root_ / "rgb");
    std::ofstream(root_ / "rgb.txt") << index;
    std::istringstream names(images);
    std::string name;
    while (names >> name) {std::ofstream(root_ / "rgb" / name) << "not-a-png";}
  }
  ~TempSequence() {std::filesystem::remove_all(root_);}
  std::string path() const {return root_.string();}

private:
  std::filesystem::path root_;
};

/// TUM's own three header lines, so the fixtures exercise the comment skip.
const char kHeader[] =
  "# color images\n"
  "# file: 'rgbd_dataset_freiburg1_desk.bag'\n"
  "# timestamp filename\n";

}  // namespace

// --- The timestamp ----------------------------------------------------------

TEST(TumTimestamp, IsExactWhereADoubleWouldNotBe)
{
  // The first frame of TUM fr1/desk, and the number this whole function is for.
  std::int64_t ns = 0;
  ASSERT_TRUE(parse_tum_timestamp("1305031452.791720", ns));
  EXPECT_EQ(ns, 1305031452791720000LL);

  // And the control: what the obvious spelling gives. If these two ever agree,
  // the assertion above has stopped being about anything — a compiler with wider
  // intermediates, a different rounding mode, and this suite would be passing
  // over the bug it was written for.
  const auto naive = static_cast<std::int64_t>(std::stod("1305031452.791720") * 1e9);
  EXPECT_NE(naive, ns)
    << "stod * 1e9 happens to be exact here, so this suite is no longer "
       "demonstrating why the integer path exists";
}

TEST(TumTimestamp, PadsTheFractionToNanosecondsRatherThanReadingItAsAnInteger)
{
  std::int64_t ns = 0;
  // "1.5" is one and a half seconds, not one second and five nanoseconds. Off by
  // 1e8 and monotonic, so a whole clip would replay at the right *order* and the
  // wrong rate.
  ASSERT_TRUE(parse_tum_timestamp("1.5", ns));
  EXPECT_EQ(ns, 1500000000LL);

  ASSERT_TRUE(parse_tum_timestamp("1.000000001", ns));
  EXPECT_EQ(ns, 1000000001LL);

  ASSERT_TRUE(parse_tum_timestamp("7", ns));
  EXPECT_EQ(ns, 7000000000LL);
}

TEST(TumTimestamp, RefusesWhatItCannotRepresentOrParse)
{
  std::int64_t ns = -1;
  // Sub-nanosecond. Truncating it would be the quiet answer, and truncating is
  // exactly what makes two distinct frames share a stamp.
  EXPECT_FALSE(parse_tum_timestamp("1.0000000001", ns));
  EXPECT_FALSE(parse_tum_timestamp("", ns));
  EXPECT_FALSE(parse_tum_timestamp(".", ns));
  EXPECT_FALSE(parse_tum_timestamp("1.2.3", ns));
  EXPECT_FALSE(parse_tum_timestamp("-1.5", ns));
  EXPECT_FALSE(parse_tum_timestamp("1e9", ns));
  EXPECT_FALSE(parse_tum_timestamp("nan", ns));
  // Year 2262 and beyond does not fit in int64 nanoseconds.
  EXPECT_FALSE(parse_tum_timestamp("99999999999.0", ns));
  // Untouched by every refusal above, which is the contract the header states.
  EXPECT_EQ(ns, -1);
}

// --- The index --------------------------------------------------------------

TEST(TumIndex, ReadsTheFormatTumActuallyWrites)
{
  TempSequence seq(
    std::string(kHeader) +
    "1305031452.791720 rgb/a.png\n"
    "1305031452.823674 rgb/b.png\n",
    "a.png b.png");

  const FrameList list = read_tum_index(seq.path(), "rgb.txt");
  ASSERT_TRUE(list.ok) << list.why;
  ASSERT_EQ(list.frames.size(), 2u);
  EXPECT_EQ(list.frames[0].stamp_ns, 1305031452791720000LL);
  EXPECT_EQ(list.frames[1].stamp_ns, 1305031452823674000LL);
  // Resolved against the sequence directory, because that is what the paths in
  // the file are relative to — a reader that returned them verbatim would work
  // only when run from inside the dataset.
  EXPECT_EQ(list.frames[0].path, seq.path() + "/rgb/a.png");
}

TEST(TumIndex, RefusesTimestampsThatDoNotStrictlyIncrease)
{
  // Equal: two frames at one stamp, which is two answers to an exact-stamp
  // lookup rather than a duplicate.
  TempSequence same(
    std::string(kHeader) + "1.0 rgb/a.png\n1.0 rgb/b.png\n", "a.png b.png");
  EXPECT_FALSE(read_tum_index(same.path(), "rgb.txt").ok);

  // Backwards: the `--loop` failure, arriving through a file instead.
  TempSequence back(
    std::string(kHeader) + "2.0 rgb/a.png\n1.0 rgb/b.png\n", "a.png b.png");
  const FrameList list = read_tum_index(back.path(), "rgb.txt");
  EXPECT_FALSE(list.ok);
  EXPECT_NE(list.why.find("does not increase"), std::string::npos) << list.why;
  // A refusal returns *no* frames, not the ones it read before giving up — the
  // `ok`-is-the-only-field-you-may-branch-on contract.
  EXPECT_TRUE(list.frames.empty());
}

TEST(TumIndex, RefusesAFrameThatIsNotOnDisk)
{
  TempSequence seq(
    std::string(kHeader) + "1.0 rgb/a.png\n2.0 rgb/missing.png\n", "a.png");
  const FrameList list = read_tum_index(seq.path(), "rgb.txt");
  EXPECT_FALSE(list.ok);
  EXPECT_NE(list.why.find("missing.png"), std::string::npos) << list.why;
}

TEST(TumIndex, RefusesALineThatIsNotTwoFields)
{
  TempSequence one(std::string(kHeader) + "1.0\n", "");
  EXPECT_FALSE(read_tum_index(one.path(), "rgb.txt").ok);

  // Three fields is the interesting one: a path with a space in it silently
  // becomes a truncated path, and the file check below would then report the
  // wrong reason. Refusing the line says what is actually wrong.
  TempSequence three(std::string(kHeader) + "1.0 rgb/a.png extra\n", "a.png");
  const FrameList list = read_tum_index(three.path(), "rgb.txt");
  EXPECT_FALSE(list.ok);
  EXPECT_NE(list.why.find("<timestamp> <path>"), std::string::npos) << list.why;
}

TEST(TumIndex, RefusesAnIndexWithNothingInIt)
{
  // Header lines only — what a truncated download or a mis-named index looks
  // like. Not an error to read, and zero frames is not a small number of frames.
  TempSequence empty(kHeader, "");
  const FrameList list = read_tum_index(empty.path(), "rgb.txt");
  EXPECT_FALSE(list.ok);
  EXPECT_NE(list.why.find("lists no frames"), std::string::npos) << list.why;
}

TEST(TumIndex, NamesTheFileItCouldNotFind)
{
  const FrameList list = read_tum_index("/nonexistent/sequence", "rgb.txt");
  EXPECT_FALSE(list.ok);
  // The path is set even on failure: "no such file" without it is the error
  // message that sends somebody looking in the wrong directory.
  EXPECT_EQ(list.index_path, "/nonexistent/sequence/rgb.txt");
  EXPECT_NE(list.why.find("/nonexistent/sequence/rgb.txt"), std::string::npos);
}
