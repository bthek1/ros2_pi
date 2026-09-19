#include "pimesh_dashboard/websocket.hpp"

#include <algorithm>
#include <cstring>

namespace pimesh_dashboard
{
namespace
{

std::uint32_t rotate_left(std::uint32_t value, int bits)
{
  return (value << bits) | (value >> (32 - bits));
}

}  // namespace

std::array<std::uint8_t, 20> sha1(const std::vector<std::uint8_t> & data)
{
  // FIPS 180-4, written out. The constants are in the form the standard publishes
  // them — hex, not decimal — which is this project's rule for anything that has a
  // reference vector: the FNV-1a basis was wrong for a whole milestone because it
  // had been pasted as a decimal literal with its last digit missing, and a hash
  // with a wrong constant behaves perfectly.
  std::uint32_t h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};

  std::vector<std::uint8_t> message = data;
  const std::uint64_t bits = static_cast<std::uint64_t>(data.size()) * 8U;
  message.push_back(0x80);
  while (message.size() % 64 != 56) {message.push_back(0x00);}
  for (int i = 7; i >= 0; --i) {
    message.push_back(static_cast<std::uint8_t>((bits >> (i * 8)) & 0xFF));
  }

  for (std::size_t chunk = 0; chunk < message.size(); chunk += 64) {
    std::uint32_t w[80];
    for (int i = 0; i < 16; ++i) {
      w[i] = (static_cast<std::uint32_t>(message[chunk + i * 4 + 0]) << 24) |
        (static_cast<std::uint32_t>(message[chunk + i * 4 + 1]) << 16) |
        (static_cast<std::uint32_t>(message[chunk + i * 4 + 2]) << 8) |
        static_cast<std::uint32_t>(message[chunk + i * 4 + 3]);
    }
    for (int i = 16; i < 80; ++i) {
      w[i] = rotate_left(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }

    std::uint32_t a = h[0];
    std::uint32_t b = h[1];
    std::uint32_t c = h[2];
    std::uint32_t d = h[3];
    std::uint32_t e = h[4];

    for (int i = 0; i < 80; ++i) {
      std::uint32_t f = 0;
      std::uint32_t k = 0;
      if (i < 20) {
        f = (b & c) | ((~b) & d);
        k = 0x5A827999u;
      } else if (i < 40) {
        f = b ^ c ^ d;
        k = 0x6ED9EBA1u;
      } else if (i < 60) {
        f = (b & c) | (b & d) | (c & d);
        k = 0x8F1BBCDCu;
      } else {
        f = b ^ c ^ d;
        k = 0xCA62C1D6u;
      }
      const std::uint32_t temp = rotate_left(a, 5) + f + e + k + w[i];
      e = d;
      d = c;
      c = rotate_left(b, 30);
      b = a;
      a = temp;
    }

    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
  }

  std::array<std::uint8_t, 20> out {};
  for (int i = 0; i < 5; ++i) {
    out[i * 4 + 0] = static_cast<std::uint8_t>((h[i] >> 24) & 0xFF);
    out[i * 4 + 1] = static_cast<std::uint8_t>((h[i] >> 16) & 0xFF);
    out[i * 4 + 2] = static_cast<std::uint8_t>((h[i] >> 8) & 0xFF);
    out[i * 4 + 3] = static_cast<std::uint8_t>(h[i] & 0xFF);
  }
  return out;
}

std::array<std::uint8_t, 20> sha1(const std::string & text)
{
  return sha1(std::vector<std::uint8_t>(text.begin(), text.end()));
}

std::string base64(const std::uint8_t * data, std::size_t size)
{
  static const char * alphabet =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((size + 2) / 3 * 4);
  std::size_t i = 0;
  while (i + 2 < size) {
    const std::uint32_t n = (static_cast<std::uint32_t>(data[i]) << 16) |
      (static_cast<std::uint32_t>(data[i + 1]) << 8) | data[i + 2];
    out += alphabet[(n >> 18) & 63];
    out += alphabet[(n >> 12) & 63];
    out += alphabet[(n >> 6) & 63];
    out += alphabet[n & 63];
    i += 3;
  }
  // The tail, and the padding is not decoration: a decoder that is handed three
  // base64 characters with no '=' cannot tell one trailing byte from two.
  if (i + 1 == size) {
    const std::uint32_t n = static_cast<std::uint32_t>(data[i]) << 16;
    out += alphabet[(n >> 18) & 63];
    out += alphabet[(n >> 12) & 63];
    out += "==";
  } else if (i + 2 == size) {
    const std::uint32_t n = (static_cast<std::uint32_t>(data[i]) << 16) |
      (static_cast<std::uint32_t>(data[i + 1]) << 8);
    out += alphabet[(n >> 18) & 63];
    out += alphabet[(n >> 12) & 63];
    out += alphabet[(n >> 6) & 63];
    out += '=';
  }
  return out;
}

std::string accept_key(const std::string & client_key)
{
  // The magic GUID is RFC 6455 section 1.3, verbatim. It is not a secret and not
  // configurable; it exists so that a server which echoes the key back without
  // understanding the protocol cannot accidentally complete a handshake.
  static const char * kMagic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  const auto digest = sha1(client_key + kMagic);
  return base64(digest.data(), digest.size());
}

Parse parse_frame(
  const std::vector<std::uint8_t> & buffer, Frame & frame, std::size_t & consumed)
{
  if (buffer.size() < 2) {return Parse::Incomplete;}

  const std::uint8_t first = buffer[0];
  const std::uint8_t second = buffer[1];
  frame.final = (first & 0x80) != 0;
  frame.opcode = static_cast<Opcode>(first & 0x0F);
  const bool masked = (second & 0x80) != 0;
  // A server MUST reject an unmasked client frame. Masking is why a WebSocket
  // endpoint cannot be driven by a page that only knows how to send HTTP.
  if (!masked) {return Parse::Invalid;}

  std::uint64_t length = second & 0x7F;
  std::size_t offset = 2;
  if (length == 126) {
    if (buffer.size() < offset + 2) {return Parse::Incomplete;}
    length = (static_cast<std::uint64_t>(buffer[offset]) << 8) | buffer[offset + 1];
    offset += 2;
  } else if (length == 127) {
    if (buffer.size() < offset + 8) {return Parse::Incomplete;}
    length = 0;
    for (int i = 0; i < 8; ++i) {
      length = (length << 8) | buffer[offset + static_cast<std::size_t>(i)];
    }
    offset += 8;
    // The high bit must be zero per the RFC, and a length past what this server
    // will ever be sent is a frame to refuse rather than to allocate for. The
    // browser only ever sends this server small control messages.
    if (length > (1ULL << 32)) {return Parse::Invalid;}
  }

  if (buffer.size() < offset + 4) {return Parse::Incomplete;}
  std::uint8_t mask[4];
  std::memcpy(mask, buffer.data() + offset, 4);
  offset += 4;

  if (buffer.size() < offset + length) {return Parse::Incomplete;}
  frame.payload.resize(static_cast<std::size_t>(length));
  for (std::uint64_t i = 0; i < length; ++i) {
    frame.payload[static_cast<std::size_t>(i)] =
      buffer[offset + static_cast<std::size_t>(i)] ^ mask[i % 4];
  }
  consumed = offset + static_cast<std::size_t>(length);
  return Parse::Ok;
}

std::vector<std::uint8_t> build_frame(
  Opcode opcode, const std::uint8_t * payload, std::size_t size, bool final)
{
  std::vector<std::uint8_t> out;
  out.reserve(size + 10);
  out.push_back(static_cast<std::uint8_t>((final ? 0x80 : 0x00) | static_cast<int>(opcode)));
  // The shortest width the length fits in. A 64-bit length on a five-byte payload
  // is legal to write and legal for the peer to reject, which is the worst kind of
  // choice: it works everywhere until it does not.
  if (size < 126) {
    out.push_back(static_cast<std::uint8_t>(size));
  } else if (size <= 0xFFFF) {
    out.push_back(126);
    out.push_back(static_cast<std::uint8_t>((size >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>(size & 0xFF));
  } else {
    out.push_back(127);
    for (int i = 7; i >= 0; --i) {
      out.push_back(static_cast<std::uint8_t>((static_cast<std::uint64_t>(size) >> (i * 8)) & 0xFF));
    }
  }
  // No mask byte and no mask: a server frame is never masked, and setting the bit
  // would make every browser drop the connection.
  out.insert(out.end(), payload, payload + size);
  return out;
}

std::vector<std::uint8_t> build_frame(Opcode opcode, const std::string & text)
{
  return build_frame(
    opcode, reinterpret_cast<const std::uint8_t *>(text.data()), text.size(), true);
}

}  // namespace pimesh_dashboard
