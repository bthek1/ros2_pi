// The instrument tools/gates/dashboard.sh measures with: a WebSocket client that
// connects, counts what arrives on each channel, and says so.
//
// **A C++ client and not a headless browser**, which is a deliberate reading of
// what P8 asks for. The phase says "with a headless browser client attached",
// and the claim under test is *what an attached client costs the pipeline* — so
// the client has to be real enough to hold a socket open, read every channel and
// be killable mid-run, and no more. A real browser would add a 200 MB process, a
// GPU context and a page render to the thing being measured, which is the error
// this project keeps writing down: an instrument whose own cost is the size of
// the effect.
//
// It is also the only way to check the *other* half. A browser that renders the
// page tells you the page renders; it cannot tell you whether the server's frame
// lengths are right, because a browser that disagrees simply closes the socket.
// This one reports the bytes.
//
// No ROS in it at all — it is a TCP client, and building it as a ROS node would
// put a participant on the domain that the measurement would then include.

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "pimesh_dashboard/web_server.hpp"
#include "pimesh_dashboard/websocket.hpp"

using pimesh_dashboard::accept_key;
using pimesh_dashboard::base64;
using pimesh_dashboard::Frame;
using pimesh_dashboard::Opcode;
using pimesh_dashboard::Parse;
using pimesh_dashboard::parse_frame;

namespace
{

const char * channel_name(std::uint8_t tag)
{
  switch (tag) {
    case 1: return "stats";
    case 2: return "rgb";
    case 3: return "depth";
    case 4: return "pose";
    case 5: return "mesh";
    default: return "unknown";
  }
}

int connect_to(const std::string & host, int port)
{
  addrinfo hints {};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo * result = nullptr;
  if (::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &result) != 0) {
    return -1;
  }
  int fd = -1;
  for (addrinfo * it = result; it != nullptr; it = it->ai_next) {
    fd = ::socket(it->ai_family, it->ai_socktype, it->ai_protocol);
    if (fd < 0) {continue;}
    if (::connect(fd, it->ai_addr, it->ai_addrlen) == 0) {break;}
    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(result);
  if (fd >= 0) {
    int yes = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
  }
  return fd;
}

/// The value of the first `"stale":` in a JSON document, or -1 if there is none.
///
/// **A substring search and not a JSON parser**, and the choice is the usual one
/// in this workspace: a parser would be a dependency, and what is needed here is
/// one boolean out of a document this repo also writes. It is a probe reading a
/// format defined twenty lines away, not a client reading the internet.
int first_stale(const std::string & json)
{
  const auto at = json.find("\"stale\":");
  if (at == std::string::npos) {return -1;}
  return json.compare(at + 8, 4, "true") == 0 ? 1 : 0;
}

/// The first `"age_s":` in a JSON document, or -1 if there is none.
double first_age(const std::string & json)
{
  const auto at = json.find("\"age_s\":");
  if (at == std::string::npos) {return -1.0;}
  return std::atof(json.c_str() + at + 8);
}

/// How many `"stale":true` there are, which for the stats document is how many
/// stage rows have gone quiet.
std::size_t count_stale(const std::string & json)
{
  std::size_t n = 0;
  for (std::size_t at = json.find("\"stale\":true"); at != std::string::npos;
    at = json.find("\"stale\":true", at + 1))
  {
    ++n;
  }
  return n;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string host = "localhost";
  int port = 8080;
  double seconds = 30.0;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--host" && i + 1 < argc) {host = argv[++i];}
    else if (arg == "--port" && i + 1 < argc) {port = std::atoi(argv[++i]);}
    else if (arg == "--seconds" && i + 1 < argc) {seconds = std::atof(argv[++i]);}
    else {
      std::fprintf(stderr, "usage: dashboard_probe [--host H] [--port P] [--seconds S]\n");
      return 2;
    }
  }

  const int fd = connect_to(host, port);
  if (fd < 0) {
    std::printf("dashboard_probe result connected=0 reason=no_connection\n");
    return 1;
  }

  // A fresh 16-byte nonce, base64'd, as RFC 6455 requires. Random rather than
  // constant because the *server's* answer is checked against it below — a server
  // that echoes a fixed value back would pass a constant key and fail this.
  std::random_device entropy;
  std::uint8_t nonce[16];
  for (std::uint8_t & byte : nonce) {byte = static_cast<std::uint8_t>(entropy() & 0xFF);}
  const std::string key = base64(nonce, sizeof(nonce));

  const std::string request =
    "GET /ws HTTP/1.1\r\nHost: " + host + ":" + std::to_string(port) + "\r\n"
    "Upgrade: websocket\r\nConnection: Upgrade\r\n"
    "Sec-WebSocket-Key: " + key + "\r\nSec-WebSocket-Version: 13\r\n\r\n";
  if (::send(fd, request.data(), request.size(), MSG_NOSIGNAL) < 0) {
    std::printf("dashboard_probe result connected=0 reason=handshake_send\n");
    ::close(fd);
    return 1;
  }

  std::vector<std::uint8_t> buffer;
  bool upgraded = false;
  std::map<std::uint8_t, std::uint64_t> counts;
  std::map<std::uint8_t, std::uint64_t> bytes;
  std::uint64_t handshake_ok = 0;

  // --- Staleness, timed from the last sample that was *not* stale -------------
  //
  // The claim is "a feed with no arrival for N seconds says STALE rather than
  // freezing on its last value looking healthy", and the only honest way to time
  // that is from the last moment the page said it was fine. Timing it from when
  // the *publisher* was killed would fold this probe's polling interval and the
  // gate's shell into the number.
  //
  // **The number reported is the server's own `age_s` at the moment the flag
  // flipped**, not the wall time between two samples of it. The first version
  // measured the latter and reported 0.10 s — which is true, and is the probe's
  // 10 Hz sampling interval rather than anything about staleness. What the claim
  // is actually about is how long after the data stopped the page admits it, and
  // only the server knows when the data stopped.
  bool seen_fresh_pose = false;
  double pose_stale_delay = -1.0;
  std::size_t stale_rows = 0;
  std::size_t stage_rows = 0;
  // The server's own count of frames it dropped because a client was not reading
  // fast enough. **Read back through the socket rather than out of a log**: it is
  // the pacing rule reporting itself, and the page shows the same number.
  long ws_dropped = -1;

  const auto started = std::chrono::steady_clock::now();
  auto first = started;
  bool have_first = false;

  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count()
    < seconds)
  {
    pollfd waiting{fd, POLLIN, 0};
    const int ready = ::poll(&waiting, 1, 200);
    if (ready < 0) {break;}
    if (ready == 0) {continue;}

    std::uint8_t chunk[65536];
    const ssize_t got = ::recv(fd, chunk, sizeof(chunk), 0);
    if (got <= 0) {break;}
    buffer.insert(buffer.end(), chunk, chunk + got);

    if (!upgraded) {
      const std::string text(buffer.begin(), buffer.end());
      const auto end = text.find("\r\n\r\n");
      if (end == std::string::npos) {continue;}
      // **The accept value is checked, not assumed.** A server that completes a
      // handshake with the wrong key is a server no browser will talk to, and a
      // probe that did not check would report a healthy connection for one.
      const std::string want = "sec-websocket-accept: " + accept_key(key);
      std::string lowered = text.substr(0, end);
      for (char & c : lowered) {c = static_cast<char>(::tolower(c));}
      std::string wanted = want;
      for (char & c : wanted) {c = static_cast<char>(::tolower(c));}
      handshake_ok = (lowered.find(wanted) != std::string::npos) ? 1 : 0;
      buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(end + 4));
      upgraded = true;
      if (handshake_ok == 0) {break;}
    }

    for (;;) {
      Frame frame;
      std::size_t consumed = 0;
      // **The server's frames are unmasked**, which the shared parser refuses —
      // correctly, because it is a *server's* parser and a server must reject an
      // unmasked client frame. So the mask bit is set here before parsing, which
      // is the smallest honest way to reuse one implementation for both
      // directions rather than writing a second one to drift from it.
      if (buffer.size() < 2) {break;}
      std::vector<std::uint8_t> masked = buffer;
      masked[1] = static_cast<std::uint8_t>(masked[1] | 0x80);
      const std::size_t header = (masked[1] & 0x7F) < 126 ? 2 :
        ((masked[1] & 0x7F) == 126 ? 4 : 10);
      masked.insert(masked.begin() + static_cast<long>(header), 4, 0x00);
      const Parse status = parse_frame(masked, frame, consumed);
      if (status != Parse::Ok) {break;}
      buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(consumed - 4));

      if (frame.opcode == Opcode::Close) {break;}
      if (frame.payload.empty()) {continue;}
      const std::uint8_t tag = frame.payload[0];
      ++counts[tag];
      bytes[tag] += frame.payload.size();

      if (tag == 4 || tag == 1) {
        const std::string json(frame.payload.begin() + 1, frame.payload.end());
        if (tag == 4) {
          const int stale = first_stale(json);
          if (stale == 0) {
            seen_fresh_pose = true;
          } else if (stale == 1 && seen_fresh_pose && pose_stale_delay < 0.0) {
            pose_stale_delay = first_age(json);
          }
        } else {
          const auto at = json.find("\"ws_dropped\":");
          if (at != std::string::npos) {ws_dropped = std::atol(json.c_str() + at + 14);}
          stale_rows = count_stale(json);
          stage_rows = 0;
          for (std::size_t at = json.find("\"stage\":"); at != std::string::npos;
            at = json.find("\"stage\":", at + 1))
          {
            ++stage_rows;
          }
        }
      }
      if (!have_first) {
        first = std::chrono::steady_clock::now();
        have_first = true;
      }
    }
  }
  ::close(fd);

  const double span = have_first ?
    std::chrono::duration<double>(std::chrono::steady_clock::now() - first).count() : 0.0;

  // One keyed line, the shape every other probe in this workspace prints.
  std::printf(
    "dashboard_probe result connected=1 handshake=%lu span_s=%.2f",
    static_cast<unsigned long>(handshake_ok), span);
  std::uint64_t total = 0;
  for (std::uint8_t tag = 1; tag <= 5; ++tag) {
    const std::uint64_t n = counts.count(tag) ? counts[tag] : 0;
    total += n;
    std::printf(
      " %s=%lu %s_hz=%.2f %s_kb=%.1f", channel_name(tag), static_cast<unsigned long>(n),
      channel_name(tag), (span > 0.0) ? static_cast<double>(n) / span : 0.0,
      channel_name(tag),
      static_cast<double>(bytes.count(tag) ? bytes[tag] : 0) / 1024.0);
  }
  std::printf(
    " total=%lu stage_rows=%lu stale_rows=%lu ws_dropped=%ld pose_stale_delay_s=%.2f\n",
    static_cast<unsigned long>(total), static_cast<unsigned long>(stage_rows),
    static_cast<unsigned long>(stale_rows), ws_dropped, pose_stale_delay);
  return (handshake_ok == 1 && total > 0) ? 0 : 1;
}
