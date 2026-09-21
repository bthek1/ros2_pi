// The two functions the whole page is built out of, and both fail silently.
//
// `stats_json()` and `pose_json()` are `quote` and `number` and nothing else.
// When either produces a byte JSON does not allow, `JSON.parse` throws in
// `onmessage`, `onStats` never runs, and the panel holds whatever it last
// showed — with the socket still up, the pipeline still running, and not one
// log line at either end. A frozen stage table is exactly what a *stalled
// pipeline* looks like, so the bug would be read as a symptom of the thing the
// dashboard exists to watch.
//
// Both dangerous inputs come from outside this node. `detail` is free-form text
// another node wrote with `snprintf`, and a rate is whatever dividing by an
// elapsed span produced — including, for a window of zero frames over a span
// that rounded to zero, a NaN.
//
// Pinned against literal expected strings rather than round-tripped through a
// parser written here. `test_mesh_io` is why: a writer and a reader that are
// wrong together round-trip perfectly, and the reader that matters is in a
// browser and cannot be linked against.

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "pimesh_dashboard/json.hpp"

using pimesh_dashboard::json::number;
using pimesh_dashboard::json::quote;

namespace
{

/// Whether `document` could be parsed at all, checked the only way that does not
/// mean writing a second implementation of the thing under test: walk it as a
/// browser's tokeniser would and refuse what RFC 8259 refuses inside a string.
/// It does not validate structure — it answers the one question that decides
/// whether the page goes blank.
bool string_bytes_are_legal(const std::string & document)
{
  bool in_string = false;
  for (std::size_t i = 0; i < document.size(); ++i) {
    const unsigned char c = static_cast<unsigned char>(document[i]);
    if (in_string) {
      if (c == '\\') {
        ++i;             // Whatever follows is escaped; skip it.
        continue;
      }
      if (c == '"') {in_string = false; continue;}
      if (c < 0x20) {return false;}   // A bare control character ends the parse.
    } else if (c == '"') {
      in_string = true;
    }
  }
  return !in_string;     // An unterminated string is the truncation case.
}

}  // namespace

// ---------------------------------------------------------------------------
// quote
// ---------------------------------------------------------------------------

TEST(Quote, WrapsOrdinaryTextAndAddsNothingElse)
{
  EXPECT_EQ(quote(""), "\"\"");
  EXPECT_EQ(quote("capture"), "\"capture\"");
  EXPECT_EQ(quote("odom"), "\"odom\"");
  // The real thing camera_node writes, which is the common case and must survive
  // untouched: an `=` and a `,` are ordinary bytes inside a JSON string.
  EXPECT_EQ(
    quote("calibrated kernel_drops=0"), "\"calibrated kernel_drops=0\"");
}

TEST(Quote, EscapesTheTwoCharactersThatWouldEndTheStringEarly)
{
  EXPECT_EQ(quote("a\"b"), "\"a\\\"b\"");
  EXPECT_EQ(quote("a\\b"), "\"a\\\\b\"");
  // A backslash at the very end is the subtle one: emitted raw it would escape
  // the closing quote and swallow the rest of the document.
  EXPECT_EQ(quote("trailing\\"), "\"trailing\\\\\"");
}

TEST(Quote, EscapesTheWhitespaceControlsByTheirShortForms)
{
  EXPECT_EQ(quote("a\nb"), "\"a\\nb\"");
  EXPECT_EQ(quote("a\rb"), "\"a\\rb\"");
  EXPECT_EQ(quote("a\tb"), "\"a\\tb\"");
}

TEST(Quote, EscapesEveryOtherControlCharacterAsFourHexDigits)
{
  // The spelling is pinned, not just the fact of escaping: `\u7` and `\U0007`
  // are both rejected by a JSON parser, and both are plausible format strings.
  EXPECT_EQ(quote(std::string(1, '\x07')), "\"\\u0007\"");
  EXPECT_EQ(quote(std::string(1, '\x0b')), "\"\\u000b\"");
  EXPECT_EQ(quote(std::string(1, '\x1f')), "\"\\u001f\"");
  // NUL inside a std::string is a byte like any other, and reaches here whenever
  // a detail was built in a fixed buffer.
  EXPECT_EQ(quote(std::string(1, '\0')), "\"\\u0000\"");
}

TEST(Quote, LeavesTheFirstPrintableByteAlone)
{
  // 0x1f is escaped and 0x20 is a space. An off-by-one at this boundary escapes
  // every space in every detail string — legal JSON, and an unreadable table.
  EXPECT_EQ(quote(" "), "\" \"");
  EXPECT_EQ(quote("two words"), "\"two words\"");
  // 0x7f is DEL, a control character in ASCII and an ordinary one to JSON.
  EXPECT_EQ(quote(std::string(1, '\x7f')), "\"\x7f\"");
}

TEST(Quote, PassesUtf8Through)
{
  // JSON is UTF-8, so these need no escaping — and escaping them per byte, which
  // is what treating `char` as signed and testing `< 0x20` the wrong way would
  // do, produces mojibake out of a detail string somebody bothered to write.
  const std::string degrees = "tilt 20\xc2\xb0";       // U+00B0
  EXPECT_EQ(quote(degrees), "\"" + degrees + "\"");
  const std::string arrow = "odom \xe2\x86\x92 base_link";   // U+2192
  EXPECT_EQ(quote(arrow), "\"" + arrow + "\"");
}

TEST(Quote, SurvivesEveryByteAValueCouldHave)
{
  // The property behind all of the above, over the whole domain: whatever a node
  // writes into `detail`, the result is a string literal a parser gets to the end
  // of. This is the assertion that would catch a newly added escape case being
  // written wrongly, which the enumerated tests above cannot.
  std::string every_byte;
  for (int i = 0; i < 256; ++i) {every_byte += static_cast<char>(i);}
  const std::string quoted = quote(every_byte);
  EXPECT_TRUE(string_bytes_are_legal(quoted));

  // And one byte at a time, so a failure names the byte.
  for (int i = 0; i < 256; ++i) {
    const std::string one = quote(std::string(1, static_cast<char>(i)));
    EXPECT_TRUE(string_bytes_are_legal(one)) << "byte " << i;
  }
}

TEST(Quote, RefusesTheInjectionThatWouldRewriteTheDocument)
{
  // The concrete failure: a detail string that closes its own value and appends
  // a field. Escaped, it is inert text; unescaped, the object it sits in gains a
  // key and the page shows a stage that does not exist.
  const std::string hostile = "ok\",\"rate_hz\":9999,\"detail\":\"";
  const std::string document = "{\"detail\":" + quote(hostile) + "}";
  EXPECT_TRUE(string_bytes_are_legal(document));
  EXPECT_EQ(document.find("\"rate_hz\""), std::string::npos)
    << "the injected key reached the document as syntax";
}

// ---------------------------------------------------------------------------
// number
// ---------------------------------------------------------------------------

TEST(Number, PrintsFourSignificantFigures)
{
  EXPECT_EQ(number(0.0), "0");
  EXPECT_EQ(number(17.4667), "17.47");
  EXPECT_EQ(number(53.6615), "53.66");
  EXPECT_EQ(number(-1.5), "-1.5");
  EXPECT_EQ(number(0.016341), "0.01634");
}

TEST(Number, StaysAValidJsonNumberAtEveryMagnitude)
{
  // %g's exponent form is a legal JSON number; %f-style output of the same value
  // would be seventeen digits of noise in a column somebody reads at a glance.
  EXPECT_EQ(number(1234567.0), "1.235e+06");
  EXPECT_EQ(number(0.00001234), "1.234e-05");
  EXPECT_EQ(number(1e300), "1e+300");
}

TEST(Number, TurnsWhatJsonCannotSpellIntoNull)
{
  // **This is the case that blanks the page.** `%g` prints these as `nan` and
  // `inf`, which are bare identifiers where a value belongs: the document fails
  // to parse and every panel stops, because one stage divided by zero for one
  // window.
  EXPECT_EQ(number(std::numeric_limits<double>::quiet_NaN()), "null");
  EXPECT_EQ(number(std::numeric_limits<double>::infinity()), "null");
  EXPECT_EQ(number(-std::numeric_limits<double>::infinity()), "null");
  // The arithmetic that actually produces them here.
  EXPECT_EQ(number(0.0 / 0.0), "null");
  EXPECT_EQ(number(1.0 / 0.0), "null");
}

TEST(Number, NeverEmitsAnIdentifierWhateverItIsGiven)
{
  const std::vector<double> values = {
    0.0, -0.0, 1.0, -1.0, 1e-320, 1e308, -1e308,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::denorm_min(),
  };
  for (double v : values) {
    const std::string text = number(v);
    EXPECT_EQ(text.find("nan"), std::string::npos) << v;
    EXPECT_EQ(text.find("inf"), std::string::npos) << v;
    EXPECT_EQ(text.find("NAN"), std::string::npos) << v;
    EXPECT_EQ(text.find("INF"), std::string::npos) << v;
    EXPECT_FALSE(text.empty()) << v;
  }
}

TEST(Number, NullIsDistinguishableFromZeroAtTheOtherEnd)
{
  // The page renders `null` as an em dash and `0` as 0.0 (`fmt` in web/app.js
  // tests for null, undefined and !isFinite explicitly). Collapsing the two here
  // would report a stage that produced no measurement as a stage measured at
  // zero — the `cost_mean=0.00` failure this workspace has now had three times.
  EXPECT_NE(number(std::numeric_limits<double>::quiet_NaN()), number(0.0));
  EXPECT_EQ(number(0.0), "0");
}

// ---------------------------------------------------------------------------
// The two together, in the shape the node actually emits
// ---------------------------------------------------------------------------

TEST(StatsRow, BuildsAParseableRowOutOfHostileInputs)
{
  // One row of `stats_json()`, assembled exactly as dashboard_node.cpp does, with
  // every field carrying the worst value it can: a detail that tries to close the
  // string, and a rate that is not a number.
  std::string row = "{\"stage\":" + quote("dep\"th") +
    ",\"rate_hz\":" + number(std::numeric_limits<double>::quiet_NaN()) +
    ",\"latency_ms\":" + number(55.0973) +
    ",\"detail\":" + quote("model\tloaded\nprovider=CUDA") +
    ",\"stale\":false}";

  EXPECT_TRUE(string_bytes_are_legal(row));
  EXPECT_NE(row.find("\"rate_hz\":null"), std::string::npos);
  EXPECT_NE(row.find("\"latency_ms\":55.1"), std::string::npos);
  EXPECT_NE(row.find("dep\\\"th"), std::string::npos);
  EXPECT_EQ(row.find('\t'), std::string::npos) << "a raw tab reached the document";
  EXPECT_EQ(row.find('\n'), std::string::npos) << "a raw newline reached the document";
}
