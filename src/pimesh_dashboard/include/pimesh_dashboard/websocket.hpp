#ifndef PIMESH_DASHBOARD__WEBSOCKET_HPP_
#define PIMESH_DASHBOARD__WEBSOCKET_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/// The wire format, with no ROS and no sockets in it.
///
/// **Hand-written rather than vendored, and the reason is the same one that keeps
/// `cv::aruco` out of `pimesh_perception`.** A single-header WebSocket library
/// would be a third dependency that has to exist, behave identically and be
/// packaged on **both** Ubuntu 24.04/Jazzy and 26.04/Lyrical — and this workspace
/// has already lost an afternoon to an API that existed at both ends and meant
/// different things. RFC 6455's server half is a SHA-1, a base64, and a frame
/// header; all three are small, all three are exactly testable against published
/// vectors, and none of them can drift between machines.
///
/// Everything here is a pure function over bytes. That is what lets
/// `test_websocket` cover it without a socket, a browser or a ROS graph — and
/// this is a protocol whose failures are otherwise invisible: a wrong accept key
/// is a browser that closes the connection with no error, a wrong mask is a
/// message that arrives as mojibake, and a length field written in the wrong
/// width is a frame boundary that is silently in the wrong place.
namespace pimesh_dashboard
{

/// SHA-1 of `data`, as 20 bytes.
///
/// Only ever used for the handshake below, where RFC 6455 names it — it is not a
/// security property and nothing here depends on SHA-1 being strong.
///
/// **Pinned against the published vectors in `test_websocket`**, which is the
/// lesson this project paid for once already: the FNV-1a offset basis sat wrong
/// for a whole milestone because a hash with the wrong constant still avalanches,
/// still compares equal to itself, and answers every question asked of it
/// correctly. A hash is the one kind of code whose behaviour cannot tell you it is
/// the wrong hash.
std::array<std::uint8_t, 20> sha1(const std::vector<std::uint8_t> & data);
std::array<std::uint8_t, 20> sha1(const std::string & text);

/// Standard base64, with padding.
std::string base64(const std::uint8_t * data, std::size_t size);

/// The `Sec-WebSocket-Accept` value for a client's `Sec-WebSocket-Key`.
///
/// `base64(sha1(key + magic))`, with the magic GUID from RFC 6455 section 1.3.
/// Getting it wrong does not produce an error anywhere: the browser simply closes
/// the connection, and the page reports "connection failed" the same way it would
/// for a port that is not listening.
std::string accept_key(const std::string & client_key);

/// The four opcodes this server has any use for.
enum class Opcode : std::uint8_t
{
  Continuation = 0x0,
  Text = 0x1,
  Binary = 0x2,
  Close = 0x8,
  Ping = 0x9,
  Pong = 0xA,
};

/// One frame taken off the wire.
struct Frame
{
  Opcode opcode {Opcode::Binary};
  bool final {true};
  std::vector<std::uint8_t> payload;
};

/// What a parse attempt concluded.
enum class Parse
{
  /// A whole frame came out; `consumed` bytes of the buffer belong to it.
  Ok,
  /// Not enough bytes yet. **Not an error** — this is the ordinary state of a
  /// stream socket, and treating a short read as a protocol failure is how a
  /// server ends up closing connections under load and nowhere else.
  Incomplete,
  /// Malformed beyond recovery: the connection has to close.
  Invalid,
};

/// Read one frame from the front of `buffer`.
///
/// **Client frames are always masked** and this rejects an unmasked one, as RFC
/// 6455 requires of a server. That is not pedantry: accepting unmasked frames is
/// what makes a WebSocket endpoint usable as a cache-poisoning gadget by a page
/// that cannot speak the protocol properly, which is the entire reason masking
/// exists.
Parse parse_frame(
  const std::vector<std::uint8_t> & buffer, Frame & frame, std::size_t & consumed);

/// Build a frame to send. **Server frames are never masked**, also per the RFC.
///
/// The length is written in the *shortest* of the three widths the protocol
/// allows — 7 bits, 16 or 64 — because a browser that receives a 64-bit length for
/// a 5-byte payload is entitled to close the connection, and some do.
std::vector<std::uint8_t> build_frame(
  Opcode opcode, const std::uint8_t * payload, std::size_t size, bool final = true);

std::vector<std::uint8_t> build_frame(Opcode opcode, const std::string & text);

}  // namespace pimesh_dashboard

#endif  // PIMESH_DASHBOARD__WEBSOCKET_HPP_
