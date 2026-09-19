// The wire format, against published vectors.
//
// **Every failure this suite catches is silent.** A wrong `Sec-WebSocket-Accept`
// is a browser that closes the connection with no error anywhere — the page says
// "connection failed", exactly as it would for a port nothing is listening on. A
// mask applied the wrong way round is a message that arrives as mojibake. A
// length field written in the wrong width is a frame boundary quietly in the
// wrong place, which shows up as the *next* message being garbage.
//
// And the SHA-1 is pinned against FIPS 180-4's own vectors for the reason this
// project already paid for once: the FNV-1a offset basis was wrong for a whole
// milestone, because a hash with a wrong constant avalanches just as well,
// compares equal to itself every time, and answers correctly every question that
// is asked of it. Testing a hash's *behaviour* can never tell you it is the wrong
// hash; only an outside reference can.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "pimesh_dashboard/websocket.hpp"

using pimesh_dashboard::accept_key;
using pimesh_dashboard::base64;
using pimesh_dashboard::build_frame;
using pimesh_dashboard::Frame;
using pimesh_dashboard::Opcode;
using pimesh_dashboard::Parse;
using pimesh_dashboard::parse_frame;
using pimesh_dashboard::sha1;

namespace
{

std::string hex(const std::array<std::uint8_t, 20> & digest)
{
  static const char * digits = "0123456789abcdef";
  std::string out;
  for (std::uint8_t byte : digest) {
    out += digits[byte >> 4];
    out += digits[byte & 0x0F];
  }
  return out;
}

/// What a browser sends: the same frame, masked with a key of its choosing.
std::vector<std::uint8_t> client_frame(
  Opcode opcode, const std::string & payload, const std::uint8_t mask[4])
{
  std::vector<std::uint8_t> out;
  out.push_back(static_cast<std::uint8_t>(0x80 | static_cast<int>(opcode)));
  if (payload.size() < 126) {
    out.push_back(static_cast<std::uint8_t>(0x80 | payload.size()));
  } else if (payload.size() <= 0xFFFF) {
    out.push_back(0x80 | 126);
    out.push_back(static_cast<std::uint8_t>((payload.size() >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>(payload.size() & 0xFF));
  } else {
    // **The helper has to speak all three widths or the test lies.** Without this
    // branch the 65536 case built a frame whose length field said 0, the parser
    // correctly returned an empty payload, and the failure looked like a parser
    // bug. A fixture that cannot express the case is not testing it.
    out.push_back(0x80 | 127);
    for (int i = 7; i >= 0; --i) {
      out.push_back(
        static_cast<std::uint8_t>((static_cast<std::uint64_t>(payload.size()) >> (i * 8)) & 0xFF));
    }
  }
  out.insert(out.end(), mask, mask + 4);
  for (std::size_t i = 0; i < payload.size(); ++i) {
    out.push_back(static_cast<std::uint8_t>(payload[i]) ^ mask[i % 4]);
  }
  return out;
}

}  // namespace

// --- SHA-1, against the standard's own vectors -------------------------------

TEST(Sha1, MatchesThePublishedVectors)
{
  // FIPS 180-4 appendix A. These are the reference values, not values this
  // implementation produced and somebody then wrote down — which is the whole
  // distinction, and the one the FNV-1a basis failed for a milestone.
  EXPECT_EQ(hex(sha1(std::string("abc"))), "a9993e364706816aba3e25717850c26c9cd0d89d");
  EXPECT_EQ(
    hex(sha1(std::string("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"))),
    "84983e441c3bd26ebaae4aa1f95129e5e54670f1");
  EXPECT_EQ(hex(sha1(std::string(""))), "da39a3ee5e6b4b0d3255bfef95601890afd80709");
}

TEST(Sha1, HandlesEveryPaddingBoundary)
{
  // The length is appended in the last 8 bytes of the final 64-byte block, so a
  // message of 55, 56 or 64 bytes exercises three different paths: fits, forces
  // an extra block, and lands exactly on a boundary. An implementation that gets
  // the padding wrong is usually right for short inputs and wrong here.
  //
  // **These came off `hashlib`, not off this implementation.** The 56-byte one
  // was written from memory first and was wrong, and the suite failed on it while
  // the code was correct — which is the reference-vector habit doing exactly its
  // job from the other direction. Never paste a value this code produced.
  EXPECT_EQ(
    hex(sha1(std::string(55, 'a'))), "c1c8bbdc22796e28c0e15163d20899b65621d65a");
  EXPECT_EQ(
    hex(sha1(std::string(56, 'a'))), "c2db330f6083854c99d4b5bfb6e8f29f201be699");
  EXPECT_EQ(
    hex(sha1(std::string(64, 'a'))), "0098ba824b5c16427bd7a1122a5a442a25ec644d");
}

TEST(Base64, MatchesRfc4648AndPadsCorrectly)
{
  auto encode = [](const std::string & text) {
      return base64(reinterpret_cast<const std::uint8_t *>(text.data()), text.size());
    };
  // RFC 4648 section 10. The padding cases are the ones that matter: without it a
  // decoder cannot tell one trailing byte from two.
  EXPECT_EQ(encode(""), "");
  EXPECT_EQ(encode("f"), "Zg==");
  EXPECT_EQ(encode("fo"), "Zm8=");
  EXPECT_EQ(encode("foo"), "Zm9v");
  EXPECT_EQ(encode("foob"), "Zm9vYg==");
  EXPECT_EQ(encode("fooba"), "Zm9vYmE=");
  EXPECT_EQ(encode("foobar"), "Zm9vYmFy");
}

TEST(AcceptKey, MatchesTheExampleInRfc6455)
{
  // RFC 6455 section 1.3, verbatim: this exact key must produce this exact
  // accept value. It is the one check that proves the magic GUID is right, and
  // getting the GUID wrong fails in complete silence — the browser closes the
  // connection and the page reports the same thing it would for a dead port.
  EXPECT_EQ(accept_key("dGhlIHNhbXBsZSBub25jZQ=="), "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
}

// --- Framing ------------------------------------------------------------------

TEST(Frames, ReadsAMaskedClientFrame)
{
  const std::uint8_t mask[4] = {0x37, 0xFA, 0x21, 0x3D};
  const auto wire = client_frame(Opcode::Text, "Hello", mask);

  Frame frame;
  std::size_t consumed = 0;
  ASSERT_EQ(parse_frame(wire, frame, consumed), Parse::Ok);
  EXPECT_EQ(consumed, wire.size());
  EXPECT_EQ(frame.opcode, Opcode::Text);
  EXPECT_TRUE(frame.final);
  EXPECT_EQ(std::string(frame.payload.begin(), frame.payload.end()), "Hello");
}

TEST(Frames, RefusesAnUnmaskedClientFrame)
{
  // A server MUST reject this. Masking is the reason a WebSocket endpoint cannot
  // be driven by a page that only knows how to send plain HTTP, and accepting an
  // unmasked frame quietly gives that up.
  std::vector<std::uint8_t> wire{0x81, 0x05, 'H', 'e', 'l', 'l', 'o'};
  Frame frame;
  std::size_t consumed = 0;
  EXPECT_EQ(parse_frame(wire, frame, consumed), Parse::Invalid);
}

TEST(Frames, AShortReadIsIncompleteAndNotAnError)
{
  // The ordinary state of a stream socket. Treating a partial frame as a
  // protocol failure is how a server ends up closing connections under load and
  // nowhere else — the bug that only appears when the payload gets big.
  const std::uint8_t mask[4] = {1, 2, 3, 4};
  const auto whole = client_frame(Opcode::Binary, std::string(200, 'x'), mask);
  Frame frame;
  std::size_t consumed = 0;
  for (std::size_t cut : {std::size_t(0), std::size_t(1), std::size_t(3), whole.size() - 1}) {
    const std::vector<std::uint8_t> partial(whole.begin(), whole.begin() + static_cast<long>(cut));
    EXPECT_EQ(parse_frame(partial, frame, consumed), Parse::Incomplete) << cut;
  }
  EXPECT_EQ(parse_frame(whole, frame, consumed), Parse::Ok);
  EXPECT_EQ(frame.payload.size(), 200u);
}

TEST(Frames, ReadsTwoFramesOutOfOneBuffer)
{
  // A browser coalesces; the parser has to say how much it took so the caller can
  // erase exactly that and go round again. Getting `consumed` wrong by even a
  // byte desynchronises every frame after it, and the symptom is the *second*
  // message being garbage while the first was fine.
  const std::uint8_t mask[4] = {9, 8, 7, 6};
  auto buffer = client_frame(Opcode::Text, "one", mask);
  const auto second = client_frame(Opcode::Text, "two", mask);
  buffer.insert(buffer.end(), second.begin(), second.end());

  Frame frame;
  std::size_t consumed = 0;
  ASSERT_EQ(parse_frame(buffer, frame, consumed), Parse::Ok);
  EXPECT_EQ(std::string(frame.payload.begin(), frame.payload.end()), "one");
  buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(consumed));
  ASSERT_EQ(parse_frame(buffer, frame, consumed), Parse::Ok);
  EXPECT_EQ(std::string(frame.payload.begin(), frame.payload.end()), "two");
  EXPECT_EQ(consumed, buffer.size());
}

TEST(Frames, ServerFramesAreNeverMasked)
{
  // Setting the mask bit on a server frame makes every browser drop the
  // connection. The bit is the high one of the second byte.
  const auto frame = build_frame(Opcode::Text, std::string("hello"));
  ASSERT_GE(frame.size(), 2u);
  EXPECT_EQ(frame[1] & 0x80, 0) << "a server frame must not be masked";
  EXPECT_EQ(frame[0], 0x81) << "FIN set, opcode text";
  EXPECT_EQ(frame.size(), 2u + 5u) << "no mask bytes on a server frame";
}

TEST(Frames, UsesTheShortestLengthEncodingThatFits)
{
  // Legal to write a 64-bit length for a five-byte payload, and legal for the
  // peer to reject it — which is the worst kind of choice, because it works
  // everywhere until it does not.
  const std::vector<std::uint8_t> small(10, 0);
  const std::vector<std::uint8_t> medium(200, 0);
  const std::vector<std::uint8_t> large(70000, 0);

  EXPECT_EQ(build_frame(Opcode::Binary, small.data(), small.size())[1], 10);
  EXPECT_EQ(build_frame(Opcode::Binary, medium.data(), medium.size())[1], 126);
  EXPECT_EQ(build_frame(Opcode::Binary, large.data(), large.size())[1], 127);

  EXPECT_EQ(build_frame(Opcode::Binary, small.data(), small.size()).size(), 2u + 10u);
  EXPECT_EQ(build_frame(Opcode::Binary, medium.data(), medium.size()).size(), 4u + 200u);
  EXPECT_EQ(build_frame(Opcode::Binary, large.data(), large.size()).size(), 10u + 70000u);
}

TEST(Frames, ABigPayloadSurvivesTheRoundTrip)
{
  // The 16-bit boundary is where a length written in the wrong width stops being
  // an academic point: 65535 fits, 65536 does not, and the frame after a wrong
  // one is where the damage shows.
  for (std::size_t size : {std::size_t(125), std::size_t(126), std::size_t(65535),
      std::size_t(65536)})
  {
    std::string payload(size, '\0');
    for (std::size_t i = 0; i < size; ++i) {payload[i] = static_cast<char>(i & 0xFF);}
    const std::uint8_t mask[4] = {0xAA, 0x55, 0x00, 0xFF};
    const auto wire = client_frame(Opcode::Binary, payload, mask);
    Frame frame;
    std::size_t consumed = 0;
    ASSERT_EQ(parse_frame(wire, frame, consumed), Parse::Ok) << size;
    ASSERT_EQ(frame.payload.size(), size) << size;
    EXPECT_EQ(std::string(frame.payload.begin(), frame.payload.end()), payload) << size;
  }
}

TEST(Frames, ControlOpcodesSurviveTheRoundTrip)
{
  // The page sends a close and answers a ping; both are read by the same parser
  // as a data frame, and an opcode masked off by the wrong nibble would turn a
  // close into a continuation and leave the socket open forever.
  const std::uint8_t mask[4] = {3, 1, 4, 1};
  for (Opcode opcode : {Opcode::Close, Opcode::Ping, Opcode::Pong}) {
    const auto wire = client_frame(opcode, "", mask);
    Frame frame;
    std::size_t consumed = 0;
    ASSERT_EQ(parse_frame(wire, frame, consumed), Parse::Ok);
    EXPECT_EQ(frame.opcode, opcode);
    EXPECT_TRUE(frame.payload.empty());
  }
}
