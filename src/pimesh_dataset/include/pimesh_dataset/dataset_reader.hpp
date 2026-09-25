#ifndef PIMESH_DATASET__DATASET_READER_HPP_
#define PIMESH_DATASET__DATASET_READER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace pimesh_dataset
{

/// One frame of a TUM-format sequence: when it was captured, and where it is.
struct DatasetFrame
{
  /// Nanoseconds since the Unix epoch, **exact**. See parse_tum_timestamp.
  std::int64_t stamp_ns {0};
  /// Absolute path to the image file.
  std::string path;
};

/// A parsed index, or the reason it is not one.
///
/// `ok` is the only field a caller may branch on, the same contract
/// `pimesh_camera::Calibration` uses: the rest is meaningful when it is true and
/// left at its default when it is false, so a caller that forgets to check gets
/// an empty list rather than a partial one.
struct FrameList
{
  bool ok {false};
  /// Why not, in a form fit for a log line or an exception. Empty when `ok`.
  std::string why;
  /// The index file that was read. Set even on failure — "no such file" is
  /// useless without it.
  std::string index_path;
  std::vector<DatasetFrame> frames;
};

/// Parse a TUM timestamp — `"1305031452.791720"` — into exact nanoseconds.
///
/// **Exact, and that word is the whole reason this is a function.** The obvious
/// spelling is `std::stod(text) * 1e9`, and it is wrong in a way nothing
/// downstream would ever report: a double carries ~15-16 significant digits, a
/// 2011 Unix timestamp spends 10 of them before the point, and the product lands
/// a few hundred nanoseconds from the decimal that was written down. Every
/// consumer accepts it, the trajectory `evo` reads is off by a fraction of a
/// microsecond against a 10 ms association window, and nothing anywhere says so
/// — but `odometry_node` pairs a frame with its depth map by **exact stamp**,
/// and a stamp that is the result of a float round trip is a stamp that has to
/// survive the same round trip identically everywhere. Splitting the decimal
/// text and scaling the two halves as integers is not cleverness; it is the only
/// version of this that is true.
///
/// Refusals, all of them silent-if-unchecked failures:
///  - empty, or anything that is not `digits[.digits]`
///  - a fraction longer than 9 digits — sub-nanosecond, which a ROS stamp cannot
///    hold, and truncating it would make two distinct frames share a stamp
///  - a value past what an `int64` of nanoseconds can represent (year 2262)
///
/// \return true and set `ns` on success; false leaves `ns` untouched.
bool parse_tum_timestamp(const std::string & text, std::int64_t & ns);

/// Read a TUM-format frame index — `rgb.txt` or `depth.txt`.
///
/// The format is three `#` comment lines and then `<timestamp> <relative path>`
/// per line. Paths are resolved against `dataset_dir`, which is what the file's
/// own `rgb/1305031452.791720.png` is relative to.
///
/// Validated rather than trusted, and each refusal is a failure that would
/// otherwise be quiet:
///
///  - the index is missing or unreadable
///  - a line does not have exactly two fields
///  - a timestamp does not parse (see above)
///  - **timestamps do not strictly increase** — two frames at one stamp is not a
///    duplicate frame, it is a second answer to a lookup this pipeline does by
///    exact stamp, and a replay that goes backwards is the `--loop` trap that
///    froze the TF tree on 2026-09-13
///  - **a listed image is not on disk** — 613 `stat` calls at startup against
///    613 `imdecode` failures at 30 Hz, which is the same information delivered
///    at the point where somebody can act on it
///  - the index lists nothing at all
FrameList read_tum_index(const std::string & dataset_dir, const std::string & index_name);

}  // namespace pimesh_dataset

#endif  // PIMESH_DATASET__DATASET_READER_HPP_
