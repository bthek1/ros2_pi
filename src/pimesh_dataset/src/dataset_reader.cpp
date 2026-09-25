#include "pimesh_dataset/dataset_reader.hpp"

#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace pimesh_dataset
{
namespace
{

bool all_digits(const std::string & text)
{
  if (text.empty()) {return false;}
  for (const char c : text) {
    if (std::isdigit(static_cast<unsigned char>(c)) == 0) {return false;}
  }
  return true;
}

}  // namespace

bool parse_tum_timestamp(const std::string & text, std::int64_t & ns)
{
  const std::size_t dot = text.find('.');
  const std::string whole = (dot == std::string::npos) ? text : text.substr(0, dot);
  std::string frac = (dot == std::string::npos) ? std::string() : text.substr(dot + 1);

  if (!all_digits(whole)) {return false;}
  // A second '.' lands in `frac` and fails the digit check, which is the refusal
  // we want without a special case for it.
  if (dot != std::string::npos && !all_digits(frac)) {return false;}
  if (frac.size() > 9) {return false;}

  // The seconds, as an integer, with the overflow checked *before* the multiply
  // rather than after — signed overflow is undefined behaviour, so a check on
  // the product is a check on a value the optimiser is entitled to assume cannot
  // exist.
  std::int64_t seconds = 0;
  for (const char c : whole) {
    const int digit = c - '0';
    if (seconds > (std::numeric_limits<std::int64_t>::max() - digit) / 10) {return false;}
    seconds = seconds * 10 + digit;
  }
  constexpr std::int64_t kNano = 1000000000LL;
  if (seconds > std::numeric_limits<std::int64_t>::max() / kNano) {return false;}

  // Right-pad to nanoseconds: "791720" is 791720000 ns, not 791720.
  frac.append(9 - frac.size(), '0');
  std::int64_t nanos = 0;
  for (const char c : frac) {nanos = nanos * 10 + (c - '0');}

  const std::int64_t total = seconds * kNano;
  if (total > std::numeric_limits<std::int64_t>::max() - nanos) {return false;}
  ns = total + nanos;
  return true;
}

FrameList read_tum_index(const std::string & dataset_dir, const std::string & index_name)
{
  FrameList out;
  const std::filesystem::path root(dataset_dir);
  out.index_path = (root / index_name).string();

  std::ifstream in(out.index_path);
  if (!in) {
    out.why = "no such file: " + out.index_path;
    return out;
  }

  std::string line;
  std::size_t line_no = 0;
  std::int64_t previous = std::numeric_limits<std::int64_t>::min();
  while (std::getline(in, line)) {
    ++line_no;
    // TUM's three header lines, and blank lines, which a hand-edited index grows.
    const std::size_t first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') {continue;}

    std::istringstream fields(line);
    std::string stamp_text;
    std::string relative;
    std::string extra;
    if (!(fields >> stamp_text >> relative) || (fields >> extra)) {
      out.why = index_name + ":" + std::to_string(line_no) +
        " is not '<timestamp> <path>': '" + line + "'";
      out.frames.clear();
      return out;
    }

    DatasetFrame frame;
    if (!parse_tum_timestamp(stamp_text, frame.stamp_ns)) {
      out.why = index_name + ":" + std::to_string(line_no) +
        " has an unusable timestamp '" + stamp_text + "'";
      out.frames.clear();
      return out;
    }
    if (frame.stamp_ns <= previous) {
      out.why = index_name + ":" + std::to_string(line_no) + " timestamp " + stamp_text +
        " does not increase — two frames at one stamp are two answers to a lookup "
        "this pipeline does by exact stamp";
      out.frames.clear();
      return out;
    }
    previous = frame.stamp_ns;

    frame.path = (root / relative).string();
    std::error_code ec;
    if (!std::filesystem::is_regular_file(frame.path, ec)) {
      out.why = index_name + ":" + std::to_string(line_no) + " lists '" + relative +
        "', which is not a file under " + dataset_dir;
      out.frames.clear();
      return out;
    }
    out.frames.push_back(std::move(frame));
  }

  if (out.frames.empty()) {
    out.why = out.index_path + " lists no frames";
    return out;
  }

  out.ok = true;
  return out;
}

}  // namespace pimesh_dataset
