#ifndef UDP_HAND_DRIVER_UDP_HAND_CONSTANTS_HPP_
#define UDP_HAND_DRIVER_UDP_HAND_CONSTANTS_HPP_

#include <rtc_base/filters/bessel_filter.hpp>
#include <rtc_base/filters/sliding_trend_detector.hpp>
#include <rtc_base/types/types.hpp>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace udp_hand_driver {

// ── Hand identity ────────────────────────────────────────────────────────────

inline constexpr int kNumHandMotors = 10;

inline const std::vector<std::string> kDefaultHandMotorNames = {
    "thumb_cmc_aa", "thumb_cmc_fe",  "thumb_mcp_fe",  "index_mcp_aa",  "index_mcp_fe",
    "index_dip_fe", "middle_mcp_aa", "middle_mcp_fe", "middle_dip_fe", "ring_mcp_fe"};

inline const std::vector<std::string> kDefaultFingertipNames = {"thumb", "index", "middle", "ring"};

// ── Hand UDP packet layout (16 uint32 values per fingertip) ──────────────────
// Packet schema is fixed by the hand firmware. These constants describe the
// layout exclusively used inside udp_hand_driver — they are NOT part of
// rtc_*/integrated_bringup compile-time contracts. Cross-package contracts use
// rtc_msgs/FingertipSensor.msg named fields instead.
//
//   barometer[8] + reserved[5] (skipped) + tof[3]
// Only 11 useful values are stored (reserved is discarded).
inline constexpr int kBarometerCount = 8;
inline constexpr int kReservedCount = 5;  // in packet only, not stored
inline constexpr int kTofCount = 3;
inline constexpr int kSensorDataPerPacket = kBarometerCount + kReservedCount + kTofCount;  // 16
inline constexpr int kSensorValuesPerFingertip = kBarometerCount + kTofCount;              // 11

// Default fingertip count (assm_v1 hand: 4 fingers). Runtime count is
// configured via YAML; this is the fallback when YAML omits the field.
inline constexpr int kDefaultNumFingertips = 4;
inline constexpr int kNumFingertips = kDefaultNumFingertips;  // 4 (legacy alias)

// Compile-time upper bound on fingertips for this hand domain. Sized so the
// std::array fields on the RT path (FT inference, baro/tof channels, packet
// scratchpads) stay heap-free; raise the constant — never branch on it — if
// a future hand exceeds 8 fingertips. Lives in this package because fingertip
// is hand-domain vocabulary; rtc_base stays robot-agnostic.
inline constexpr int kMaxFingertips = 8;

// Capacity-derived totals (hand-domain — fingertip is the natural unit here).
inline constexpr int kMaxHandSensors = kMaxFingertips * kSensorValuesPerFingertip;  // 88
inline constexpr int kNumHandSensors = kNumFingertips * kSensorValuesPerFingertip;  // 44

// ── Fingertip F/T inference (3-head ONNX model) ──────────────────────────────
// Output layout: [contact_prob(1), F(3), u(3)] = 7
// (output0 = contact logit→sigmoid, output1 = F, output2 = u)
inline constexpr int kFTValuesPerFingertip = 7;
inline constexpr int kFTInputSize = 2 * kBarometerCount;  // baro(8) + delta(8) = 16
inline constexpr int kFTHistoryLength = 12;               // FIFO history rows for ONNX input

// ── Fingertip F/T inference state (SeqLock-compatible: trivially_copyable) ───
struct FingertipFTState {
  static constexpr int kMaxFTValues = kMaxFingertips * kFTValuesPerFingertip;  // 56
  std::array<float, kMaxFTValues> ft_data{};
  std::array<bool, kMaxFingertips> per_fingertip_valid{};
  int num_fingertips{0};
  bool valid{false};
};

// ── Hand sensor filtering channel capacities ─────────────────────────────────
inline constexpr std::size_t kMaxBaroChannels = 64;  // kMaxFingertips × kBarometerCount
inline constexpr std::size_t kMaxTofChannels = 24;   // kMaxFingertips × kTofCount

using BesselFilterBaro = rtc::BesselFilterN<kMaxBaroChannels>;
using BesselFilterTof = rtc::BesselFilterN<kMaxTofChannels>;

// Per-fingertip barometer drift detector (8 baro channels, 5 s window @ 500 Hz)
using BarometerTrendDetector = rtc::SlidingTrendDetector<kBarometerCount, 2500>;

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_UDP_HAND_CONSTANTS_HPP_
