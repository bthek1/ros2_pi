#ifndef PIMESH_DASHBOARD__JSON_HPP_
#define PIMESH_DASHBOARD__JSON_HPP_

#include <cmath>
#include <cstdio>
#include <string>

namespace pimesh_dashboard
{
namespace json
{

/// The two functions every byte of text this dashboard renders passes through.
///
/// **They lived in an anonymous namespace inside `dashboard_node.cpp` until
/// 2026-09-21, where no test could call them**, which is the same gap
/// `percentile`, FNV-1a and `depth_mat_over` each sat in before somebody looked.
/// The question CLAUDE.md asks of a helper is not whether it is interesting but
/// whether a test could reach it if it wanted to, and for these two the answer
/// was no while `stats_json()` and `pose_json()` — the entire contents of the
/// page — were built out of nothing else.
///
/// They are worth a suite for the usual reason: **both fail by blanking the
/// page rather than by erroring.** `JSON.parse` throws on the first bad byte,
/// `onStats` never returns, and the panel simply stops updating with a message
/// in a console nobody has open. There is no log line at either end, the
/// WebSocket stays up, the pipeline is unaffected, and the stage table holds
/// whatever it last showed — which reads as a *stalled pipeline* rather than as
/// a formatting bug. Both of the inputs that do it come from outside this node:
/// `detail` is free-form text another node wrote, and a rate is whatever
/// dividing by an elapsed time produced.
///
/// Header-only and ROS-free on purpose, the same arrangement `depth_model.hpp`
/// has for the arithmetic either side of the network: the suite needs no node,
/// no socket and no browser, so it runs identically on both machines.

/// `text` as a JSON string literal, quotes included.
///
/// The dangerous input is `detail`, which arrives off a topic as whatever
/// another node's `snprintf` produced and goes into the document verbatim. One
/// unescaped `"` ends the string early and the rest of the object becomes
/// syntax; one raw newline is a control character inside a string literal,
/// which RFC 8259 forbids and every browser parser rejects.
///
/// Bytes at or above 0x80 pass through untouched, which is correct: JSON is
/// UTF-8 and escaping them would mangle any non-ASCII detail a node writes.
inline std::string quote(const std::string & text)
{
  std::string out = "\"";
  for (char c : text) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          // Every other control character, by the one escape JSON always allows.
          // A bare 0x07 in a string is as fatal to the parse as a bare quote and
          // is far likelier to reach here unnoticed, since it prints as nothing.
          char buffer[8];
          std::snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned char>(c));
          out += buffer;
        } else {
          out += c;
        }
    }
  }
  return out + "\"";
}

/// `value` as a JSON number, or `null` where JSON has no spelling for it.
///
/// **NaN and the infinities are a parse error and not a bad value.** JSON has no
/// literal for either, so printing what `%g` gives (`nan`, `inf`) puts a bare
/// identifier where a number belongs and the whole document fails — one stage
/// dividing by zero for one window blanks every panel. `null` is the honest
/// answer and the page already renders it as `—` (`fmt` in `web/app.js` tests
/// for it explicitly), so the two ends agree that a missing number is missing
/// rather than zero.
///
/// `%.4g` because these are numbers a person reads off a table at a glance: four
/// significant figures covers 17.47 Hz and 0.0163 ms alike, and a rate printed
/// to seventeen digits would be a column of noise. It stays valid JSON at every
/// magnitude — `%g`'s exponent form (`1.235e+06`) is a legal JSON number.
inline std::string number(double value)
{
  if (!std::isfinite(value)) {return "null";}
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.4g", value);
  return buffer;
}

}  // namespace json
}  // namespace pimesh_dashboard

#endif  // PIMESH_DASHBOARD__JSON_HPP_
