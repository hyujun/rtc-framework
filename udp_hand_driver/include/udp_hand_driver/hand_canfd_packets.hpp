#ifndef UDP_HAND_DRIVER_HAND_CANFD_PACKETS_HPP_
#define UDP_HAND_DRIVER_HAND_CANFD_PACKETS_HPP_

// proto2 wire format: the hand over CAN FD (epic #895, H1-F04-2a).
//
// This header is the code-side single source of the wire contract (D23): the
// tables below are the contract, the static_asserts pin them, and the two
// vendor documents (FDCAN1_Communication_Protocol.md "Proto",
// FDCAN1_Frame.md "Frame") are what they were transcribed from. It is byte
// pure — no socket, no ROS, no <linux/can.h>, no other driver header — so the
// driver's CAN comm, the fake firmware and the unit tests all build the same
// bytes from it and nothing else.
//
// ── CAN identifier (29-bit extended, Proto §2 · Frame §2.1) ─────────────────
//   bit 28..26  priority   (3)  0 = highest; every hand frame uses 0
//   bit 25..22  msg type   (4)  0 CMD · 1 REQ · 2 RESP
//   bit 21..14  target     (8)  0x00 host · 0x01 right hand · 0x02 left hand · 0xFF broadcast
//   bit 13..6   source     (8)  0x00 host · 0x01 right hand · 0x02 left hand
//   bit  5..0   frame idx  (6)  CMD 0 position · 1 FF current · 2 target current · 3 hand side
//                               RESP 0 · 1 · 2 (the telemetry burst)
//
//   host → hand   CMD   [0][0][node][0][idx]   right 0x00004000+idx   left 0x00008000+idx
//                 REQ   [0][1][node][0][0]     right 0x00404000       left 0x00408000
//   hand → host   echo  [0][0][0][node][idx]   right 0x00000040+idx   left 0x00000080+idx
//                 RESP  [0][2][0][node][idx]   right 0x00800040+idx   left 0x00800080+idx
//
//   Arbitration (lower id wins, W2): echo < CMD < REQ < RESP right < RESP left.
//   Values here are the bare 29-bit identifier. The socket adds CAN_EFF_FLAG
//   (bit 31) to every one of them and marks error frames with CAN_ERR_FLAG
//   (bit 29), whose low bits are an error class, not a hand id — an echo of
//   Frame 0 (0x40) and a bus-off error frame (0x20000040) differ only in that
//   flag, so a receiver must test IsErrorFrame() BEFORE masking an id down to
//   29 bits. kRespIdMask (0x1FFFFFC0) is the receive-filter mask that selects
//   one node's RESP burst regardless of frame index (W1).
//
// ── Payloads (every data frame DLC 64, little-endian, IEEE 754 float32) ─────
//   CMD Frame 0 (target position, rad) and Frame 1 (feedforward current, A)
//   share one layout:    0..39 float[10] M0..M9       40..63 reserved (0x00)
//   The MCU applies a CMD, then echoes its 64 bytes back verbatim (same idx,
//   target host, source node) — the host's write validation (D41). Frame 2
//   (target current → current mode) and Frame 3 (hand side → flash) are not
//   built here (D27: Frame 0 + 1 only); only their ids can be formed.
//   REQ carries no data (DLC 0, D47) and is answered by the RESP burst idx 0, 1, 2:
//   RESP F0   0..39  pos   float[10]  rad          40..63  vel M0..M5  float[6]  rad/s
//   RESP F1   0..15  vel M6..M9  float[4]  rad/s   16..55  cur   float[10]  A
//            56..63  thumb fx fy fz px  int16[4]
//   RESP F2   0..1  thumb py int16 · 2..3 thumb temp int16 · 4 thumb contact uint8
//             5..17 index TactileRaw · 18..30 middle · 31..43 ring · 44..63 reserved
//   TactileRaw (13 B): fx fy fz px py temp int16[6] + contact uint8. The thumb's
//   is split across F1 (fx fy fz px) and F2 (py temp contact) — ThumbRaw().
//
// ── Tactile scaling (Proto §4.3 — the SSoT for the signs; the §4.2 table
//   omits the flips on fy, fz and py). The wire's contact point is in mm;
//   this header hands it out in metres (SI at the boundary, AGENTS §10). ─────
//   fx [N] = +raw × 25 / 32767        fy [N] = −raw × 25 / 32767
//   fz [N] = −raw × 100 / 32767       px [m] = +raw × 0.32767 / 32767
//   py [m] = −raw × 0.32767 / 32767   temp [°C] = raw      contact = raw != 0
//
//   There is no sequence number, CRC, cycle counter, mode readback or fault
//   flag anywhere on the wire (W3, D31) — RespSetAssembler completes a set on
//   its three frame indices alone, in any arrival order (D57).
//
// RT contract: everything here is noexcept and allocation-free (the comm calls
// it from the SCHED_FIFO CommLoop tick). The assembler takes the clock as an
// argument instead of reading it, so the set deadline (D42) is testable.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <type_traits>

namespace udp_hand_driver::canfd {

// ── CAN identifier ──────────────────────────────────────────────────────────

/// 4-bit message-type field (Proto §2.1).
enum class MsgType : uint8_t { kCmd = 0x0, kReq = 0x1, kResp = 0x2 };

/// Node ids (Frame §2.1). A hand's id is set in its flash (W5); the host is 0.
inline constexpr uint8_t kNodeHost = 0x00;
inline constexpr uint8_t kNodeRightHand = 0x01;
inline constexpr uint8_t kNodeLeftHand = 0x02;
inline constexpr uint8_t kNodeBroadcast = 0xFF;

/// Which hand a node is. This is the only spelling the host side exposes
/// (the `hand_side` parameter, D51): the integer node id stays inside this
/// header, derived by NodeIdOf().
enum class HandSide : uint8_t { kRight, kLeft };

/// Node id of a hand (D51: right → kNodeRightHand, left → kNodeLeftHand).
[[nodiscard]] constexpr uint8_t NodeIdOf(HandSide side) noexcept {
  return side == HandSide::kLeft ? kNodeLeftHand : kNodeRightHand;
}

/// Parse the `hand_side` parameter's value ("right" / "left", exact). False
/// and `out` untouched for anything else.
[[nodiscard]] constexpr bool ParseHandSide(std::string_view text, HandSide& out) noexcept {
  if (text == "right") {
    out = HandSide::kRight;
    return true;
  }
  if (text == "left") {
    out = HandSide::kLeft;
    return true;
  }
  return false;
}

static_assert(NodeIdOf(HandSide::kRight) == kNodeRightHand, "D51: right hand is node 1");
static_assert(NodeIdOf(HandSide::kLeft) == kNodeLeftHand, "D51: left hand is node 2");

/// The only priority the hand protocol uses (0 = wins arbitration).
inline constexpr uint8_t kPriorityRealtime = 0;

/// Bit positions of the id fields (Proto §2).
inline constexpr unsigned kPriorityShift = 26;
inline constexpr unsigned kMsgTypeShift = 22;
inline constexpr unsigned kTargetShift = 14;
inline constexpr unsigned kSourceShift = 6;

/// 29-bit identifier (= CAN_EFF_MASK). Strips the socket's EFF / RTR / ERR flags.
inline constexpr uint32_t kExtIdMask = 0x1FFFFFFFu;
/// Error-frame flag as SocketCAN sets it on can_id (= CAN_ERR_FLAG, bit 29).
inline constexpr uint32_t kErrorFrameFlag = 0x20000000u;
inline constexpr uint32_t kFrameIndexMask = 0x3Fu;
/// Receive-filter mask selecting one node's RESP burst whatever the frame index (W1).
inline constexpr uint32_t kRespIdMask = kExtIdMask & ~kFrameIndexMask;  // 0x1FFFFFC0

/// True for a SocketCAN error frame; its id is an error class, not a hand id,
/// and must be rejected before the 29-bit mask is applied.
[[nodiscard]] constexpr bool IsErrorFrame(uint32_t socket_can_id) noexcept {
  return (socket_can_id & kErrorFrameFlag) != 0;
}

/// Proto §2.2 FDCAN_EXT_ID_MAKE — the bare 29-bit identifier (no CAN_EFF_FLAG).
/// Every field is masked to its width.
[[nodiscard]] constexpr uint32_t MakeCanId(uint8_t priority, MsgType type, uint8_t target,
                                           uint8_t source, uint8_t frame_idx) noexcept {
  return ((static_cast<uint32_t>(priority) & 0x07u) << kPriorityShift) |
         ((static_cast<uint32_t>(type) & 0x0Fu) << kMsgTypeShift) |
         ((static_cast<uint32_t>(target) & 0xFFu) << kTargetShift) |
         ((static_cast<uint32_t>(source) & 0xFFu) << kSourceShift) |
         (static_cast<uint32_t>(frame_idx) & kFrameIndexMask);
}

/// host → `node`: command frame `frame_idx` (CmdFrame).
[[nodiscard]] constexpr uint32_t CmdId(uint8_t node, uint8_t frame_idx) noexcept {
  return MakeCanId(kPriorityRealtime, MsgType::kCmd, node, kNodeHost, frame_idx);
}

/// `node` → host: echo of the command frame `frame_idx` (same payload).
[[nodiscard]] constexpr uint32_t EchoId(uint8_t node, uint8_t frame_idx) noexcept {
  return MakeCanId(kPriorityRealtime, MsgType::kCmd, kNodeHost, node, frame_idx);
}

/// host → `node`: telemetry request (DLC 0).
[[nodiscard]] constexpr uint32_t ReqId(uint8_t node) noexcept {
  return MakeCanId(kPriorityRealtime, MsgType::kReq, node, kNodeHost, 0);
}

/// `node` → host: telemetry response frame `frame_idx` (0, 1, 2).
[[nodiscard]] constexpr uint32_t RespId(uint8_t node, uint8_t frame_idx) noexcept {
  return MakeCanId(kPriorityRealtime, MsgType::kResp, kNodeHost, node, frame_idx);
}

// Field extractors. Each reads only its own bits, so they accept a bare
// 29-bit id or a socket can_id that still carries the flag bits — but an
// error frame's bits mean something else entirely (see IsErrorFrame).

/// Priority field (bits 28..26).
[[nodiscard]] constexpr uint8_t PriorityOf(uint32_t id) noexcept {
  return static_cast<uint8_t>((id >> kPriorityShift) & 0x07u);
}

/// Raw 4-bit type field (bits 25..22) — may hold a value with no MsgType enumerator.
[[nodiscard]] constexpr uint8_t MsgTypeOf(uint32_t id) noexcept {
  return static_cast<uint8_t>((id >> kMsgTypeShift) & 0x0Fu);
}

/// Target node (bits 21..14).
[[nodiscard]] constexpr uint8_t TargetOf(uint32_t id) noexcept {
  return static_cast<uint8_t>((id >> kTargetShift) & 0xFFu);
}

/// Source node (bits 13..6).
[[nodiscard]] constexpr uint8_t SourceOf(uint32_t id) noexcept {
  return static_cast<uint8_t>((id >> kSourceShift) & 0xFFu);
}

/// Frame index (bits 5..0).
[[nodiscard]] constexpr uint8_t FrameIndexOf(uint32_t id) noexcept {
  return static_cast<uint8_t>(id & kFrameIndexMask);
}

// The vendor documents' worked ids (Proto §3 · §4, Frame §2.2 · §2.3 · §4.1).
static_assert(CmdId(kNodeRightHand, 0) == 0x00004000u, "Frame §2.2 CMD Frame 0, right hand");
static_assert(CmdId(kNodeRightHand, 1) == 0x00004001u, "Frame §2.2 CMD Frame 1, right hand");
static_assert(CmdId(kNodeLeftHand, 0) == 0x00008000u, "Frame §2.2 CMD Frame 0, left hand");
static_assert(CmdId(kNodeBroadcast, 3) == 0x003FC003u, "Frame §2.2 CMD Frame 3, broadcast");
static_assert(EchoId(kNodeRightHand, 0) == 0x00000040u, "Proto §3.6 echo Frame 0, right hand");
static_assert(EchoId(kNodeLeftHand, 1) == 0x00000081u, "Proto §3.6 echo Frame 1, left hand");
static_assert(ReqId(kNodeRightHand) == 0x00404000u, "Proto §3.1 REQ, right hand");
static_assert(ReqId(kNodeLeftHand) == 0x00408000u, "Frame §2.2 REQ, left hand");
static_assert(RespId(kNodeRightHand, 0) == 0x00800040u, "Proto §4.1 RESP Frame 0, right hand");
static_assert(RespId(kNodeRightHand, 2) == 0x00800042u, "Proto §4.3 RESP Frame 2, right hand");
static_assert(RespId(kNodeLeftHand, 1) == 0x00800081u, "Frame §4.1 RESP Frame 1, left hand");
static_assert(kRespIdMask == 0x1FFFFFC0u, "W1 RESP filter mask");
static_assert((RespId(kNodeLeftHand, 2) & kRespIdMask) == RespId(kNodeLeftHand, 0),
              "the RESP mask must erase only the frame index");
static_assert((kErrorFrameFlag & kExtIdMask) == 0, "CAN_ERR_FLAG lies above the 29-bit id");

// ── Frames ──────────────────────────────────────────────────────────────────

/// DLC of every data frame.
inline constexpr std::size_t kFrameLen = 64;
/// Motors and fingertips the proto2 wire carries (W1: 10 motors, 4 fingertips,
/// same hand as proto1b). The wire count is pinned here, independent of the
/// driver's kNumHandMotors / kDefaultNumFingertips; test_hand_canfd_packets
/// asserts the two agree.
inline constexpr std::size_t kWireMotorCount = 10;
inline constexpr std::size_t kWireFingertipCount = 4;
/// RESP F0 carries vel M0..M5, RESP F1 the rest.
inline constexpr std::size_t kVelLoCount = 6;
inline constexpr std::size_t kVelHiCount = kWireMotorCount - kVelLoCount;
inline constexpr std::size_t kCmdReservedLen = kFrameLen - kWireMotorCount * sizeof(float);  // 24
inline constexpr std::size_t kRespFrame2ReservedLen = 20;

/// Command frame index (CmdId's `frame_idx`). Only kPosition / kFeedforward
/// payloads are built by this header (D27).
enum class CmdFrame : uint8_t {
  kPosition = 0,       ///< target position, rad — switches the MCU to position mode
  kFeedforward = 1,    ///< feedforward current, A — added as torque T = kt × i
  kTargetCurrent = 2,  ///< target current, A — switches the MCU to current mode (unused)
  kHandSide = 3,       ///< data[0] = 1 right / 2 left → flash (unused)
};

/// Frames in one RESP burst (idx 0 .. kNumRespFrames-1).
inline constexpr std::size_t kNumRespFrames = 3;
/// Set-complete mask: FrameBit(i) for every i < kNumRespFrames.
inline constexpr uint8_t kRespSetMask = static_cast<uint8_t>((1u << kNumRespFrames) - 1u);

/// Bit of RESP frame `frame_idx` in a set mask; 0 for an index outside the
/// burst (no bit to set — callers reject such frames before using the mask).
[[nodiscard]] constexpr uint8_t FrameBit(uint8_t frame_idx) noexcept {
  return frame_idx < kNumRespFrames ? static_cast<uint8_t>(1u << frame_idx) : uint8_t{0};
}

static_assert(kRespSetMask == (FrameBit(0) | FrameBit(1) | FrameBit(2)), "mask covers the burst");
static_assert(FrameBit(static_cast<uint8_t>(kNumRespFrames)) == 0, "no bit past the burst");

/// Fingertip order of Telemetry::tactile (= the driver's default fingertip names).
enum class Fingertip : std::size_t { kThumb = 0, kIndex = 1, kMiddle = 2, kRing = 3 };

#pragma pack(push, 1)

/// CMD Frame 0 / Frame 1 payload (64 B). Plain arrays, not std::array: the
/// struct is packed and taking a packed member's address is what the compiler
/// warns about — element access is not.
struct CmdMotorFrame {
  float values[kWireMotorCount];  ///< Frame 0: rad · Frame 1: A, firmware motor order
  uint8_t reserved[kCmdReservedLen];
};

/// RESP F0 (64 B): position M0..M9 + velocity M0..M5.
struct RespFrame0 {
  float pos[kWireMotorCount];  ///< rad
  float vel_lo[kVelLoCount];   ///< rad/s, M0..M5
};

/// One fingertip's raw tactile record (13 B) as laid out in RESP F2.
struct TactileRaw {
  int16_t fx;
  int16_t fy;
  int16_t fz;
  int16_t px;
  int16_t py;
  int16_t temp;
  uint8_t contact;
};

/// RESP F1 (64 B): velocity M6..M9 + current M0..M9 + thumb tactile head.
struct RespFrame1 {
  float vel_hi[kVelHiCount];   ///< rad/s, M6..M9
  float cur[kWireMotorCount];  ///< A
  int16_t thumb_fx;
  int16_t thumb_fy;
  int16_t thumb_fz;
  int16_t thumb_px;
};

/// RESP F2 (64 B): thumb tactile tail + index / middle / ring records.
struct RespFrame2 {
  int16_t thumb_py;
  int16_t thumb_temp;
  uint8_t thumb_contact;
  TactileRaw index;
  TactileRaw middle;
  TactileRaw ring;
  uint8_t reserved[kRespFrame2ReservedLen];
};

#pragma pack(pop)

static_assert(sizeof(CmdMotorFrame) == kFrameLen, "CMD Frame 0/1 is 64 bytes");
static_assert(sizeof(RespFrame0) == kFrameLen, "RESP F0 is 64 bytes");
static_assert(sizeof(RespFrame1) == kFrameLen, "RESP F1 is 64 bytes");
static_assert(sizeof(RespFrame2) == kFrameLen, "RESP F2 is 64 bytes");
static_assert(sizeof(TactileRaw) == 13, "Proto §4.3: 13 bytes per fingertip");
static_assert(offsetof(RespFrame0, vel_lo) == 40, "Proto §4.1: vel at 40");
static_assert(offsetof(RespFrame1, cur) == 16, "Proto §4.2: cur at 16");
static_assert(offsetof(RespFrame1, thumb_fx) == 56, "Proto §4.2: thumb fx at 56");
static_assert(offsetof(RespFrame2, thumb_contact) == 4, "Proto §4.3: thumb contact at 4");
static_assert(offsetof(RespFrame2, index) == 5, "Proto §4.3: index at 5");
static_assert(offsetof(RespFrame2, middle) == 18, "Proto §4.3: middle at 18");
static_assert(offsetof(RespFrame2, ring) == 31, "Proto §4.3: ring at 31");
static_assert(offsetof(RespFrame2, reserved) == 44, "Proto §4.3: reserved at 44");
static_assert(std::is_trivially_copyable_v<CmdMotorFrame> &&
                  std::is_trivially_copyable_v<RespFrame0> &&
                  std::is_trivially_copyable_v<RespFrame1> &&
                  std::is_trivially_copyable_v<RespFrame2>,
              "wire frames are memcpy'd");

/// Build a Frame 0 (rad) or Frame 1 (A) payload; reserved bytes zero.
[[nodiscard]] inline CmdMotorFrame MakeCmdMotorFrame(
    const std::array<float, kWireMotorCount>& values) noexcept {
  CmdMotorFrame frame{};
  for (std::size_t i = 0; i < kWireMotorCount; ++i) {
    frame.values[i] = values[i];
  }
  return frame;
}

/// Copy a 64-byte frame struct onto the wire buffer (`out` holds kFrameLen bytes).
template <typename Frame>
inline void SerializeFrame(const Frame& frame, uint8_t* out) noexcept {
  static_assert(sizeof(Frame) == kFrameLen, "only 64-byte wire frames");
  std::memcpy(out, &frame, kFrameLen);
}

/// Copy a received payload into a frame struct. False unless len == 64.
template <typename Frame>
[[nodiscard]] inline bool DecodeFrame(const uint8_t* data, std::size_t len, Frame& out) noexcept {
  static_assert(sizeof(Frame) == kFrameLen, "only 64-byte wire frames");
  if (len != kFrameLen)
    return false;
  std::memcpy(&out, data, kFrameLen);
  return true;
}

// ── Tactile decode (Proto §4.3) ─────────────────────────────────────────────

/// int16 full scale of every tactile channel.
inline constexpr float kTactileFullScale = 32767.0f;
/// Physical value at full scale.
inline constexpr float kForceXyRangeN = 25.0f;
inline constexpr float kForceZRangeN = 100.0f;
inline constexpr float kLocationRangeM = 0.32767f;  ///< the wire's 327.67 mm

/// One fingertip in host units. Which of these the driver publishes is the
/// consumer's choice (D29 / D30 keep fx fy fz + contact); the decode is
/// complete so the wire is readable end to end.
struct Tactile {
  float fx_n{0.0f};
  float fy_n{0.0f};
  float fz_n{0.0f};
  float px_m{0.0f};  ///< contact point, metres (wire: mm)
  float py_m{0.0f};
  float temp_c{0.0f};
  bool contact{false};
};

/// Proto §4.3 formulas, sign flips on fy / fz / py included, location in metres.
[[nodiscard]] constexpr Tactile DecodeTactile(TactileRaw raw) noexcept {
  Tactile t{};
  t.fx_n = static_cast<float>(raw.fx) * kForceXyRangeN / kTactileFullScale;
  t.fy_n = -static_cast<float>(raw.fy) * kForceXyRangeN / kTactileFullScale;
  t.fz_n = -static_cast<float>(raw.fz) * kForceZRangeN / kTactileFullScale;
  t.px_m = static_cast<float>(raw.px) * kLocationRangeM / kTactileFullScale;
  t.py_m = -static_cast<float>(raw.py) * kLocationRangeM / kTactileFullScale;
  t.temp_c = static_cast<float>(raw.temp);
  t.contact = raw.contact != 0;
  return t;
}

/// Reassemble the thumb record that the wire splits across F1 and F2.
[[nodiscard]] constexpr TactileRaw ThumbRaw(const RespFrame1& f1, const RespFrame2& f2) noexcept {
  TactileRaw r{};
  r.fx = f1.thumb_fx;
  r.fy = f1.thumb_fy;
  r.fz = f1.thumb_fz;
  r.px = f1.thumb_px;
  r.py = f2.thumb_py;
  r.temp = f2.thumb_temp;
  r.contact = f2.thumb_contact;
  return r;
}

// ── Telemetry set ───────────────────────────────────────────────────────────

/// The three RESP frames of one REQ, in index order.
struct RespSet {
  RespFrame0 frame0;
  RespFrame1 frame1;
  RespFrame2 frame2;
};

static_assert(sizeof(RespSet) == kNumRespFrames * kFrameLen, "RespSet is the raw burst");

/// One hand's telemetry in host units (firmware motor order; fingertips in
/// Fingertip order).
struct Telemetry {
  std::array<float, kWireMotorCount> pos{};  ///< rad
  std::array<float, kWireMotorCount> vel{};  ///< rad/s
  std::array<float, kWireMotorCount> cur{};  ///< A
  std::array<Tactile, kWireFingertipCount> tactile{};
};

/// Stitch a complete RespSet into Telemetry (velocity halves, thumb halves).
inline void DecodeTelemetry(const RespSet& set, Telemetry& out) noexcept {
  for (std::size_t i = 0; i < kWireMotorCount; ++i) {
    out.pos[i] = set.frame0.pos[i];
    out.cur[i] = set.frame1.cur[i];
  }
  for (std::size_t i = 0; i < kVelLoCount; ++i) {
    out.vel[i] = set.frame0.vel_lo[i];
  }
  for (std::size_t i = 0; i < kVelHiCount; ++i) {
    out.vel[kVelLoCount + i] = set.frame1.vel_hi[i];
  }
  out.tactile[static_cast<std::size_t>(Fingertip::kThumb)] =
      DecodeTactile(ThumbRaw(set.frame1, set.frame2));
  out.tactile[static_cast<std::size_t>(Fingertip::kIndex)] = DecodeTactile(set.frame2.index);
  out.tactile[static_cast<std::size_t>(Fingertip::kMiddle)] = DecodeTactile(set.frame2.middle);
  out.tactile[static_cast<std::size_t>(Fingertip::kRing)] = DecodeTactile(set.frame2.ring);
}

// ── RESP set assembler (D42 · D43 · D44) ────────────────────────────────────

/// What Feed() did with one received frame.
enum class FeedResult : uint8_t {
  kStored,      ///< a frame of this node's set, stored; set still incomplete
  kComplete,    ///< this frame completed the set — set() is now defined
  kDuplicate,   ///< idx already held — replaces that copy before completion, dropped after
  kOutsideSet,  ///< not a RESP from this node (error frame, echo, other hand, CMD, REQ) — dropped
  kBadIndex,    ///< RESP from this node with idx >= kNumRespFrames — dropped
  kBadLength,   ///< RESP from this node with len != kFrameLen — dropped
};

/// Counters for the comm stats. Single writer (the thread feeding the
/// assembler); snapshot by value.
struct AssemblerStats {
  uint64_t sets_completed{0};
  /// A partial set was dropped by Reset / Begin (the deadline passed: D43
  /// cycle failure).
  uint64_t sets_abandoned{0};
  /// An idx already in the set: replaced before completion, dropped after.
  uint64_t duplicate{0};
  uint64_t outside_set{0};
  uint64_t bad_index{0};
  uint64_t bad_length{0};
};

/// Collects one node's RESP burst (idx 0, 1, 2) into a RespSet.
///
/// Lifecycle per cycle: Begin(deadline) right after the REQ is sent → Feed()
/// every frame the socket yields until kComplete or Expired(now) → on expiry
/// Reset() (drops the partial set; the comm drains the socket and fails the
/// cycle — D43 / D44, no retry inside the tick). Membership is by id only:
/// not an error frame and (id & kRespIdMask) == RespId(node, 0). The set
/// completes when all three indices are held, whatever their arrival order
/// (D57); a repeated idx replaces the earlier copy until then, and once
/// complete the set is frozen until Begin / Reset. With no counter on the
/// wire (D31), a tail of the previous burst that arrives more than a tick
/// late can complete a mixed set; this tick's leftover frames then show up
/// in the next DrainStale's stale_drained. One instance per hand; no shared
/// state.
class RespSetAssembler {
 public:
  using Clock = std::chrono::steady_clock;

  explicit constexpr RespSetAssembler(uint8_t node) noexcept : expected_base_(RespId(node, 0)) {}

  /// Start a new set with its deadline; a pending partial set counts as abandoned.
  void Begin(Clock::time_point deadline) noexcept {
    Reset();
    deadline_ = deadline;
  }

  /// Move the deadline of the set in progress without touching what is
  /// stored (D59 deadline chain: the set stage's deadline is re-based on the
  /// moment the echo stage closes, after frames may already have arrived).
  void SetDeadline(Clock::time_point deadline) noexcept { deadline_ = deadline; }

  /// Drop whatever is pending (partial → abandoned). Deadline untouched.
  void Reset() noexcept {
    AbandonIfPartial();
    mask_ = 0;
  }

  /// Offer one received frame. `can_id` is accepted as the socket delivers it
  /// (EFF / ERR flags still set) or as a bare 29-bit id; `data` holds `len`
  /// bytes. Error frames are rejected before the id is interpreted.
  [[nodiscard]] FeedResult Feed(uint32_t can_id, const uint8_t* data, std::size_t len) noexcept {
    if (IsErrorFrame(can_id)) {
      ++stats_.outside_set;
      return FeedResult::kOutsideSet;
    }
    const uint32_t id = can_id & kExtIdMask;
    if ((id & kRespIdMask) != expected_base_) {
      ++stats_.outside_set;
      return FeedResult::kOutsideSet;
    }
    const uint8_t idx = FrameIndexOf(id);
    if (idx >= kNumRespFrames) {
      ++stats_.bad_index;
      return FeedResult::kBadIndex;
    }
    if (len != kFrameLen) {
      ++stats_.bad_length;
      return FeedResult::kBadLength;
    }
    if ((mask_ & FrameBit(idx)) != 0) {
      ++stats_.duplicate;
      if (mask_ != kRespSetMask)
        Store(idx, data);
      return FeedResult::kDuplicate;
    }
    Store(idx, data);
    mask_ = static_cast<uint8_t>(mask_ | FrameBit(idx));
    if (mask_ == kRespSetMask) {
      ++stats_.sets_completed;
      return FeedResult::kComplete;
    }
    return FeedResult::kStored;
  }

  [[nodiscard]] bool complete() const noexcept { return mask_ == kRespSetMask; }

  /// FrameBit(i) set for every frame received in the current set.
  [[nodiscard]] uint8_t mask() const noexcept { return mask_; }

  /// Defined only while complete(); later Feeds leave it as it is.
  [[nodiscard]] const RespSet& set() const noexcept { return set_; }

  [[nodiscard]] bool Expired(Clock::time_point now) const noexcept { return now >= deadline_; }

  /// Time left before the set deadline; zero once expired.
  [[nodiscard]] Clock::duration Remaining(Clock::time_point now) const noexcept {
    return now >= deadline_ ? Clock::duration::zero() : deadline_ - now;
  }

  [[nodiscard]] Clock::time_point deadline() const noexcept { return deadline_; }

  [[nodiscard]] uint8_t node() const noexcept { return SourceOf(expected_base_); }

  [[nodiscard]] const AssemblerStats& stats() const noexcept { return stats_; }

  void ResetStats() noexcept { stats_ = AssemblerStats{}; }

 private:
  void AbandonIfPartial() noexcept {
    if (mask_ != 0 && mask_ != kRespSetMask)
      ++stats_.sets_abandoned;
  }

  void Store(uint8_t idx, const uint8_t* data) noexcept {
    switch (idx) {
      case 0:
        std::memcpy(&set_.frame0, data, kFrameLen);
        break;
      case 1:
        std::memcpy(&set_.frame1, data, kFrameLen);
        break;
      case 2:
        std::memcpy(&set_.frame2, data, kFrameLen);
        break;
      default:
        break;  // unreachable: Feed rejects idx >= kNumRespFrames
    }
  }

  uint32_t expected_base_;
  uint8_t mask_{0};
  Clock::time_point deadline_{};
  RespSet set_{};
  AssemblerStats stats_{};
};

}  // namespace udp_hand_driver::canfd

#endif  // UDP_HAND_DRIVER_HAND_CANFD_PACKETS_HPP_
