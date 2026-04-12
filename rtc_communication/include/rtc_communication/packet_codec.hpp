#ifndef RTC_COMMUNICATION_PACKET_CODEC_HPP_
#define RTC_COMMUNICATION_PACKET_CODEC_HPP_

// Generic packet codec concept and helpers.
//
// Defines the PacketCodec concept: any type that provides:
//   - RecvPacket   : packed struct for the wire-format receive packet
//   - SendPacket   : packed struct for the wire-format send packet
//   - State        : application-level state type (decoded output)
//   - static Decode(span<const uint8_t>, State&) -> bool
//
// Encode is NOT enforced by the concept — codecs may provide a static
// Encode() method, but it is optional. The generic EncodePacket() helper
// covers the common trivially-copyable memcpy case.
//
// Domain-specific codecs satisfy this concept.

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

namespace rtc {

// -- Concept: PacketCodec ----------------------------------------------------
// A codec type C must expose:
//   C::RecvPacket  -- trivially copyable packed struct
//   C::SendPacket  -- trivially copyable packed struct
//   C::State       -- decoded state type
//   C::Decode(span<const uint8_t>, State&) -> bool
// Optional (not enforced by concept):
//   C::Encode(const SendPacket&, span<uint8_t, sizeof(SendPacket)>) -> void

template <typename C>
concept PacketCodec = requires {
  typename C::RecvPacket;
  typename C::SendPacket;
  typename C::State;
} && std::is_trivially_copyable_v<typename C::RecvPacket>
  && std::is_trivially_copyable_v<typename C::SendPacket>
  && requires(std::span<const uint8_t> buf, typename C::State& state) {
    { C::Decode(buf, state) } -> std::same_as<bool>;
  }

// -- Generic decode helper ---------------------------------------------------
// Decodes a raw buffer into a packed struct T via memcpy.
// Returns false if the buffer is smaller than sizeof(T).
template <typename T>
  requires std::is_trivially_copyable_v<T>
[[nodiscard]] inline bool DecodePacket(
    std::span<const uint8_t> buf, T& out) noexcept {
  if (buf.size() < sizeof(T)) return false;
  std::memcpy(&out, buf.data(), sizeof(T));
  return true;
}

// -- Generic encode helper ---------------------------------------------------
// Encodes a packed struct T into a raw byte buffer via memcpy.
template <typename T>
  requires std::is_trivially_copyable_v<T>
inline void EncodePacket(
    const T& pkt,
    std::span<uint8_t, sizeof(T)> out) noexcept {
  std::memcpy(out.data(), &pkt, sizeof(T));
}

}  // namespace rtc

#endif  // RTC_COMMUNICATION_PACKET_CODEC_HPP_
