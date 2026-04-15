// ── fake_codec.hpp ──────────────────────────────────────────────────────────
// Minimal PacketCodec implementation used by rtc_communication unit tests.
// Exists solely to drive Transceiver<Codec> through realistic send/recv cycles
// without pulling in any robot-specific packet definitions.
//
// Satisfies the rtc::PacketCodec concept (see packet_codec.hpp):
//   - RecvPacket / SendPacket : trivially copyable wire structs
//   - State                   : decoded application-level snapshot
//   - static Decode(span, State&) -> bool
// ─────────────────────────────────────────────────────────────────────────────
#ifndef RTC_COMMUNICATION_TEST_FAKE_CODEC_HPP_
#define RTC_COMMUNICATION_TEST_FAKE_CODEC_HPP_

#include "rtc_communication/packet_codec.hpp"

#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

namespace rtc::test {

#pragma pack(push, 1)
struct FakePacket {
  uint32_t sequence;
  int32_t  payload;
};
#pragma pack(pop)

static_assert(std::is_trivially_copyable_v<FakePacket>);

struct FakeCodec {
  using RecvPacket = FakePacket;
  using SendPacket = FakePacket;
  using State      = FakePacket;

  // Decodes a full FakePacket from buf. Returns false on short buffer.
  [[nodiscard]] static bool Decode(
      std::span<const uint8_t> buf, State& out) noexcept {
    if (buf.size() < sizeof(FakePacket)) return false;
    std::memcpy(&out, buf.data(), sizeof(FakePacket));
    return true;
  }
};

static_assert(rtc::PacketCodec<FakeCodec>,
              "FakeCodec must satisfy the PacketCodec concept");

}  // namespace rtc::test

#endif  // RTC_COMMUNICATION_TEST_FAKE_CODEC_HPP_
