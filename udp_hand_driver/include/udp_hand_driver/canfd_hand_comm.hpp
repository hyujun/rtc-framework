#ifndef UDP_HAND_DRIVER_CANFD_HAND_COMM_HPP_
#define UDP_HAND_DRIVER_CANFD_HAND_COMM_HPP_

// CanFdHandComm: HandComm over CAN FD for the proto2 hand (epic #895, H1-F04-2c).
//
// One rtc::CanFdTransport (29-bit ids, BRS on, RECV_OWN_MSGS off) carries the
// whole cycle built from hand_canfd_packets.hpp — the wire contract stays in
// that header, this one only decides what to send when and how to count what
// comes back. Every receive goes through ONE window (RunWindow), whose two
// stages open with what was sent:
//
//   RunCycle                  the cycle transaction (D49 (b) · D59 · D60):
//                             CMD Frame 0 (pos, rad) + Frame 1 (ff, A) when a
//                             command came in, then REQ (DLC 0, D47), back to
//                             back; one receive window collects the two echoes
//                             (D41) and the RESP burst idx 0 1 2 in any order
//                             (D57) under the deadline chain below
//                                                     → kWriteEcho · kJointRead,
//                             and on a sensor cycle kBulkSensor too (D49: 1:1
//                             with proto1's per-kind stats)
//   WriteCommandWithEcho      the echo stage alone (no REQ)       → kWriteEcho
//   ReadCycle                 the set stage alone (REQ, no CMD) → kJointRead
//   WriteCommandFireAndForget E-Stop: Frame 1 zeros, then Frame 0 zeros (D26),
//                             nothing awaited — not a window
//   DrainStale                non-blocking drain of the queue (≤ kMaxDrainPerCall)
//                             + assembler reset
//
// Deadline chain (D42 · D59). Each stage has one deadline of length T =
// recv_timeout_ms: the echo stage closes at CMD + T (or on both echoes), the
// set stage at (echo stage close) + T — re-based with SetDeadline when the
// echoes arrive, left at the nominal CMD + 2T when they do not — so the worst
// case is 2T whatever the hand does, the configure-time check 2T < slot stays
// the bound, and a tick without a write has the set stage alone (REQ + T).
// The hand answers each frame in turn, so the pipelined RESP burst lands at
// about CMD + 1.3 ms; a set deadline of REQ + T would miss it every cycle.
//
// Frame classification (D41 · D58 · D59). Every frame the filter lets through:
//   ① an error message frame (CAN_ERR_FLAG, subscribed through error_mask) →
//      CanFdCommStats::error_frames / bus_off only;
//   ② this node's echo with idx 0 or 1 not yet seen while the echo stage is
//      open → accepted in any order; a payload that differs from what was sent
//      only bumps echo_payload_mismatch (the hand applied its own copy —
//      nothing to re-send);
//   ③ this node's RESP while the set stage is open → RespSetAssembler::Feed
//      (duplicates, bad index / length counted by it, D57);
//   ④ anything else (an echo idx ≥ 2 or a repeated idx, a RESP outside an
//      open set stage, a late echo after its stage closed, another type) →
//      cmd_mismatch + last_unexpected_* only, attributed to kWriteEcho while
//      the echo stage is open and to kJointRead otherwise — however many
//      arrive, the open stages keep waiting (D58). Nothing is re-sent (D41).
// recv_ok counts the first class-② / ③ frame of each open stage (≤ 2 per
// cycle); every stage ends on exactly one terminal counter (ok, timeout or
// error), so the per-kind attempt denominators hold. A failed CMD send fails
// the echo stage before any wait (recv_error of kWriteEcho beside the tx errno
// counter; the REQ still goes out); a failed REQ send fails the set stage the
// same way (the echo stage still waits); a recv error ends every open stage
// with an error. On this link a dead peer / downed interface surfaces on send
// (ENETDOWN, ENOBUFS), which is the role ECONNREFUSED plays on UDP.
//
// The set stage takes every RESP of this node that arrives after DrainStale,
// so a RESP tail more than a tick late can complete a mixed set (D57) on a
// write tick as on a read-only one — accepted in #919, no arrival-time guard.
//
// Feedforward units (D50): the controller hands Nm, the wire carries A;
// FeedforwardToAmps() divides by the per-motor torque constant
// Config::feedforward_kt (Nm/A, default 1.0 = pass-through). This is the one
// place that conversion lives.
//
// Statistics: the shared HandCommStatsPair (UdpHandCommStats, same JSON keys
// and meaning: a frame-level arrival is recv_ok, a transaction result is
// per_kind) plus CanFdCommStats for what only this link has (tx errno classes,
// error frames, echo payload mismatches, the assembler's counters) behind its
// own SeqLock. canfd_stats() is not part of HandComm; the node reaches it
// through a non-owning pointer to the concrete type it constructed.
//
// Receive filter: rx id EchoId(node, 0) under kRxMask — priority, target (host)
// and source (this node) bits only — so this node's echoes and RESP frames pass
// and the other hand's traffic, our own CMD / REQ and everything else is
// dropped in the kernel. Error frames bypass that filter and are subscribed
// separately (CanTransportConfig::error_mask).
//
// RT: every HandComm RT method is noexcept and allocation-free; the frames,
// the assembler and the stats are members, the clock is steady_clock, there is
// no RCLCPP call (failures are return values — the node logs) and no
// process-global state: one instance per hand.

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_communication/can/canfd_transport.hpp"
#include "udp_hand_driver/hand_canfd_packets.hpp"
#include "udp_hand_driver/hand_comm.hpp"
#include "udp_hand_driver/udp_hand_constants.hpp"

#include <linux/can.h>
#include <linux/can/error.h>
#include <time.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <utility>

namespace udp_hand_driver {

/// Link configuration (the node fills it from parameters at on_configure).
struct CanFdHandCommConfig {
  std::string interface_name{"can0"};
  canfd::HandSide hand_side{canfd::HandSide::kRight};  ///< D51: the only hand selector
  /// Per-stage receive deadline (echo stage · RESP set stage), D42.
  double recv_timeout_ms{2.0};
  /// Per-motor torque constant kt [Nm/A] (firmware motor order), D50. Must be
  /// > 0 with a finite reciprocal — Open() refuses otherwise (NUM-2). 1.0
  /// passes Nm through as A.
  std::array<float, kNumHandMotors> feedforward_kt{
      {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
  /// SO_PRIORITY for the socket; < 0 leaves the kernel default.
  int socket_priority{-1};
};

/// What only the CAN FD link counts. Single writer (CommLoop thread); readers
/// get a SeqLock snapshot through CanFdHandComm::canfd_stats().
struct CanFdCommStats {
  // Failed SendFrame() by errno class (every CMD / REQ / E-Stop frame).
  uint64_t tx_eagain{0};    ///< socket send buffer full
  uint64_t tx_enobufs{0};   ///< controller tx queue full
  uint64_t tx_enetdown{0};  ///< interface down
  uint64_t tx_other{0};
  // Error message frames received (error_mask subscription), any stage.
  uint64_t error_frames{0};
  uint64_t bus_off{0};  ///< of which CAN_ERR_BUSOFF
  /// Echo frames (D41 class ②) whose 64 bytes differ from the command sent.
  uint64_t echo_payload_mismatch{0};
  /// RespSetAssembler counters, copied in at publish time.
  canfd::AssemblerStats assembler{};
};

class CanFdHandComm final : public HandComm {
 public:
  using Clock = std::chrono::steady_clock;

  /// Max non-blocking reads per DrainStale() call: the controller's 10-frame
  /// Rx FIFO plus a late echo pair, with headroom. A residual backlog is picked
  /// up next cycle.
  static constexpr int kMaxDrainPerCall = 16;

  /// Receive-filter mask: priority + target + source (type and index bits
  /// cleared), so one filter admits this node's echo and RESP streams.
  static constexpr uint32_t kRxMask = 0x1C3FFFC0u;

  /// Error classes subscribed. CAN_ERR_BUSERROR is left out on purpose: the
  /// kernel header warns it may flood, and every flooded frame costs the RT
  /// loop one recv.
  static constexpr can_err_mask_t kErrorMask =
      CAN_ERR_TX_TIMEOUT | CAN_ERR_CRTL | CAN_ERR_BUSOFF | CAN_ERR_RESTARTED;

  /// errno class of a failed SendFrame() (the tx_* counters).
  enum class TxFailure : uint8_t { kEagain, kEnobufs, kEnetdown, kOther };

  [[nodiscard]] static constexpr TxFailure ClassifyTxErrno(int err) noexcept {
    switch (err) {
      case EAGAIN:
        return TxFailure::kEagain;
      case ENOBUFS:
        return TxFailure::kEnobufs;
      case ENETDOWN:
        return TxFailure::kEnetdown;
      default:
        return TxFailure::kOther;
    }
  }

  // Not noexcept: the transport config copies interface_name (non-RT path).
  explicit CanFdHandComm(CanFdHandCommConfig cfg)
      : cfg_(std::move(cfg)),
        node_(canfd::NodeIdOf(cfg_.hand_side)),
        transport_(MakeTransportConfig(cfg_, node_)),
        assembler_(node_),
        echo_base_(canfd::EchoId(node_, 0)),
        resp_base_(canfd::RespId(node_, 0)) {
    tx_frame0_.can_id = canfd::CmdId(node_, 0) | CAN_EFF_FLAG;
    tx_frame1_.can_id = canfd::CmdId(node_, 1) | CAN_EFF_FLAG;
    tx_frame0_.len = tx_frame1_.len = static_cast<__u8>(canfd::kFrameLen);
    tx_frame0_.flags = tx_frame1_.flags = CANFD_BRS;
    tx_req_.can_id = canfd::ReqId(node_) | CAN_EFF_FLAG;
    tx_req_.len = 0;  // D47: the REQ carries no data
    tx_req_.flags = CANFD_BRS;
  }

  /// The wrapped transport, read-only (socket-level observations in tests).
  [[nodiscard]] const rtc::CanFdTransport& transport() const noexcept { return transport_; }

  [[nodiscard]] const CanFdHandCommConfig& config() const noexcept { return cfg_; }

  [[nodiscard]] uint8_t node() const noexcept { return node_; }

  /// Nm → A through Config::feedforward_kt (D50). Defined after Open().
  [[nodiscard]] float FeedforwardToAmps(std::size_t motor, float ff_nm) const noexcept {
    return ff_nm * inv_kt_[motor];
  }

  /// D50 / NUM-2 acceptance of one torque constant: finite and > 0 (which also
  /// rejects NaN) with a finite reciprocal — a subnormal kt passes > 0 but 1/kt
  /// is +inf, and 0 × inf = NaN would put NaN into the E-Stop feedforward
  /// frame; kt = +inf (a YAML value past FLT_MAX) would pass > 0 with 1/kt = 0
  /// and silently send every feedforward as 0 A. One predicate for Open() and
  /// the node's configure-time check, so the two cannot disagree.
  [[nodiscard]] static bool IsValidFeedforwardKt(float kt) noexcept {
    return std::isfinite(kt) && (kt > 0.0f) && std::isfinite(1.0f / kt);
  }

  // ── Non-RT ────────────────────────────────────────────────────────────────

  /// Validates feedforward_kt (> 0, NUM-2), precomputes the per-stage timeout
  /// and opens the socket (filter, error subscription, optional priority).
  [[nodiscard]] bool Open() noexcept override {
    for (std::size_t i = 0; i < inv_kt_.size(); ++i) {
      const float kt = cfg_.feedforward_kt[i];
      if (!IsValidFeedforwardKt(kt))
        return false;
      inv_kt_[i] = 1.0f / kt;
    }
    if (!(cfg_.recv_timeout_ms > 0.0))
      return false;
    stage_timeout_ = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double, std::milli>(cfg_.recv_timeout_ms));
    if (!transport_.Open())
      return false;
    if (cfg_.socket_priority >= 0 && !transport_.SetPriority(cfg_.socket_priority)) {
      transport_.Close();
      return false;
    }
    return true;
  }

  void Close() noexcept override { transport_.Close(); }

  /// proto2 has no sensor-mode handshake: the RESP burst always carries the
  /// tactile records.
  [[nodiscard]] bool InitializeSensors() noexcept override { return true; }

  // set_verify_response_mode / set_verify_bulk_sensor_mode / set_joint_io_mode:
  // no MODE byte on this wire — the HandComm defaults (no-ops) apply.

  void ResetCommStats() noexcept override {
    stats_.Reset();
    canfd_working_ = {};
    assembler_.ResetStats();
    canfd_seqlock_.Store(canfd_working_);
  }

  [[nodiscard]] UdpHandCommStats comm_stats() const noexcept override { return stats_.Load(); }

  /// CAN-only counters, consistent snapshot (published with PublishCommStats).
  [[nodiscard]] CanFdCommStats canfd_stats() const noexcept { return canfd_seqlock_.Load(); }

  [[nodiscard]] uint64_t recv_error_count() const noexcept override {
    return stats_.recv_error_count();
  }

  [[nodiscard]] double recv_timeout_ms() const noexcept override { return cfg_.recv_timeout_ms; }

  [[nodiscard]] bool SupportsIndividualMode() const noexcept override { return false; }

  // ── RT ────────────────────────────────────────────────────────────────────

  int DrainStale() noexcept override {
    if (!transport_.is_open())
      return 0;
    const int drained = DrainQueued();
    stats_.CountStaleDrained(drained);
    assembler_.Reset();
    return drained;
  }

  [[nodiscard]] bool WriteCommandWithEcho(const HandMotorArray& pos,
                                          const HandMotorArray& ff_nm) noexcept override {
    RTC_TRACE_SCOPE("hand_write_echo");
    if (!transport_.is_open())
      return false;
    BuildCommandFrames(pos, ff_nm);
    return RunWindow(/*send_cmd=*/true, /*send_req=*/false, /*sensors=*/false).write_ok;
  }

  void WriteCommandFireAndForget(const HandMotorArray& pos,
                                 const HandMotorArray& ff_nm) noexcept override {
    if (!transport_.is_open())
      return;
    BuildCommandFrames(pos, ff_nm);
    // D26: the feedforward goes to zero first, then the position.
    (void)SendCounted(tx_frame1_);
    (void)SendCounted(tx_frame0_);
  }

  /// REQ → RESP set (the set stage alone). No motor-space read exists on this
  /// wire (motor_ok stays false, kMotor stamped at once); the set fills the
  /// joint-space triplet and, on a sensor cycle, the raw 192-byte RespSet for
  /// SensorProtocol2.
  void ReadCycle(const HandReadPlan& plan, HandReadBuffers& bufs,
                 HandReadOutcome& out) noexcept override {
    out = HandReadOutcome{};
    out.StampPhase(ReadPhase::kMotor);
    if (!transport_.is_open()) {
      out.StampPhase(ReadPhase::kJoint);
      return;
    }
    RTC_TRACE_SCOPE("hand_read_joint");
    const WindowResult r = RunWindow(/*send_cmd=*/false, /*send_req=*/true, plan.sensors);
    FinishRead(r, plan, bufs, out);
  }

  /// The cycle transaction (D49 (b) · D59 · D60): CMD pair (when a command
  /// came in) and REQ sent back to back, then one receive window for the
  /// echoes and the RESP set. kMotor is stamped at the window start, kJoint
  /// at set completion (or at the window end when the set failed).
  void RunCycle(const HandReadPlan& plan, const HandMotorArray* cmd_pos,
                const HandMotorArray* cmd_ff, HandReadBuffers& bufs,
                HandReadOutcome& out) noexcept override {
    RTC_TRACE_SCOPE("hand_cycle_window");
    out = HandReadOutcome{};
    const bool write = (cmd_pos != nullptr) && (cmd_ff != nullptr);
    out.StampPhase(ReadPhase::kMotor);
    if (!transport_.is_open()) {
      if (write)
        out.write_end = Clock::now();
      out.StampPhase(ReadPhase::kJoint);
      return;
    }
    if (write)
      BuildCommandFrames(*cmd_pos, *cmd_ff);
    const WindowResult r = RunWindow(write, /*send_req=*/true, plan.sensors);
    out.write_ok = r.write_ok;
    out.write_end = r.write_end;
    FinishRead(r, plan, bufs, out);
  }

  [[nodiscard]] UdpHandCommStats& comm_stats_mut() noexcept override { return stats_.working(); }

  void PublishCommStats() noexcept override {
    stats_.Publish();
    canfd_working_.assembler = assembler_.stats();
    canfd_seqlock_.Store(canfd_working_);
  }

  /// MsgType in the high nibble, frame index (low 4 bits) in the low nibble of
  /// the last class-④ frame — there is no CMD byte on this wire.
  [[nodiscard]] uint8_t last_unexpected_cmd() const noexcept override {
    return stats_.working().last_unexpected_cmd;
  }

  [[nodiscard]] uint16_t last_unexpected_len() const noexcept override {
    return stats_.working().last_unexpected_len;
  }

 private:
  [[nodiscard]] static rtc::CanFdTransportConfig MakeTransportConfig(const CanFdHandCommConfig& cfg,
                                                                     uint8_t node) {
    rtc::CanFdTransportConfig t{};
    t.interface_name = cfg.interface_name;
    t.extended_frame = true;
    t.bitrate_switch = true;
    t.receive_own_messages = false;
    t.rx_can_id = canfd::EchoId(node, 0);
    t.rx_can_mask = kRxMask;
    t.error_mask = kErrorMask;
    // SO_RCVTIMEO is not used by the frame API (ppoll deadlines); keep it sane.
    t.recv_timeout_ms = std::max(1, static_cast<int>(cfg.recv_timeout_ms + 0.5));
    return t;
  }

  [[nodiscard]] static timespec ToTimespec(Clock::duration d) noexcept {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(ns / 1'000'000'000);
    ts.tv_nsec = static_cast<long>(ns % 1'000'000'000);
    return ts;
  }

  /// Forensic "cmd" of a frame that is not what the stage waits for.
  [[nodiscard]] static uint8_t UnexpectedCmdOf(uint32_t id) noexcept {
    return static_cast<uint8_t>((canfd::MsgTypeOf(id) << 4) | (canfd::FrameIndexOf(id) & 0x0Fu));
  }

  void BuildCommandFrames(const HandMotorArray& pos, const HandMotorArray& ff_nm) noexcept {
    std::array<float, canfd::kWireMotorCount> ff_a{};
    for (std::size_t i = 0; i < ff_a.size(); ++i)
      ff_a[i] = FeedforwardToAmps(i, ff_nm[i]);
    canfd::SerializeFrame(canfd::MakeCmdMotorFrame(pos), tx_frame0_.data);
    canfd::SerializeFrame(canfd::MakeCmdMotorFrame(ff_a), tx_frame1_.data);
  }

  /// SendFrame with the tx errno counters; false on failure (errno kept).
  [[nodiscard]] bool SendCounted(const canfd_frame& frame) noexcept {
    if (transport_.SendFrame(frame))
      return true;
    switch (ClassifyTxErrno(errno)) {
      case TxFailure::kEagain:
        ++canfd_working_.tx_eagain;
        break;
      case TxFailure::kEnobufs:
        ++canfd_working_.tx_enobufs;
        break;
      case TxFailure::kEnetdown:
        ++canfd_working_.tx_enetdown;
        break;
      case TxFailure::kOther:
        ++canfd_working_.tx_other;
        break;
    }
    return false;
  }

  /// Class ①: counts and consumes an error message frame. False for a data frame.
  [[nodiscard]] bool ConsumeErrorFrame(const canfd_frame& frame) noexcept {
    if (!canfd::IsErrorFrame(frame.can_id))
      return false;
    ++canfd_working_.error_frames;
    if ((frame.can_id & CAN_ERR_BUSOFF) != 0)
      ++canfd_working_.bus_off;
    return true;
  }

  /// What one receive window reported (see RunWindow).
  struct WindowResult {
    bool write_ok{false};           ///< both echoes arrived (echo stage only)
    bool set_ok{false};             ///< the RESP set completed (set stage only)
    Clock::time_point write_end{};  ///< echo stage close (echoes, deadline or error)
    Clock::time_point set_end{};    ///< set stage close (complete, deadline or error)
  };

  /// Class of one received data frame (classification in the header comment).
  enum class RxClass : uint8_t { kEcho, kResp, kOther };

  [[nodiscard]] RxClass ClassifyRx(uint32_t id) const noexcept {
    if ((id & ~canfd::kFrameIndexMask) == echo_base_)
      return RxClass::kEcho;
    if ((id & canfd::kRespIdMask) == resp_base_)
      return RxClass::kResp;
    return RxClass::kOther;
  }

  /// The receive window (D59). Sends what the flags ask for — the CMD pair
  /// (tx_frame0_/1_ already built) and / or the REQ — then receives until
  /// every stage that opened has closed. Deadline chain: echo stage CMD + T;
  /// set stage (echo close) + T, nominal CMD + 2T until the echoes arrive, REQ
  /// + T when no echo stage opened. ppoll waits for the nearest open deadline.
  [[nodiscard]] WindowResult RunWindow(bool send_cmd, bool send_req, bool sensors) noexcept {
    WindowResult r{};
    // ── Send stage ───────────────────────────────────────────────────────
    bool echo_open = false;
    Clock::time_point echo_deadline{};
    if (send_cmd) {
      if (SendCounted(tx_frame0_) && SendCounted(tx_frame1_)) {
        echo_open = true;
        echo_deadline = Clock::now() + stage_timeout_;
      } else {
        stats_.CountRecvError(RequestKind::kWriteEcho);
        r.write_end = Clock::now();
      }
    }
    bool set_open = false;
    if (send_req) {
      if (SendCounted(tx_req_)) {
        set_open = true;
        // Upper bound of the chain; pulled in by SetDeadline when the echoes
        // arrive. Begin() resets the assembler, so it is called once here.
        assembler_.Begin((echo_open ? echo_deadline : Clock::now()) + stage_timeout_);
      } else {
        CountSetError(sensors);
        r.set_end = Clock::now();
      }
    }

    // ── Receive window ───────────────────────────────────────────────────
    uint8_t echo_got = 0;
    bool echo_first = true;
    bool set_first = true;
    while (echo_open || set_open) {
      const Clock::time_point now = Clock::now();
      if (echo_open && now >= echo_deadline) {
        stats_.CountRecvTimeout(RequestKind::kWriteEcho);
        echo_open = false;
        r.write_end = now;  // the set deadline stays at the nominal CMD + 2T
      }
      if (set_open && assembler_.Expired(now)) {
        CountSetTimeout(sensors);
        AbandonSet();
        set_open = false;
        r.set_end = now;
      }
      if (!echo_open && !set_open)
        break;
      Clock::time_point next = set_open ? assembler_.deadline() : echo_deadline;
      if (echo_open && echo_deadline < next)
        next = echo_deadline;
      const ssize_t n = transport_.RecvFrameWithTimeout(rx_, ToTimespec(next - now));
      if (n == 0)
        continue;  // deadline or a spurious wake: the loop head decides
      if (n < 0) {
        const Clock::time_point t_err = Clock::now();
        if (echo_open) {
          stats_.CountRecvError(RequestKind::kWriteEcho);
          echo_open = false;
          r.write_end = t_err;
        }
        if (set_open) {
          CountSetError(sensors);
          AbandonSet();
          set_open = false;
          r.set_end = t_err;
        }
        break;
      }
      if (ConsumeErrorFrame(rx_))
        continue;  // ①
      const uint32_t id = rx_.can_id & canfd::kExtIdMask;
      const uint8_t idx = canfd::FrameIndexOf(id);
      const RxClass cls = ClassifyRx(id);
      if (cls == RxClass::kEcho && echo_open) {
        if (echo_first) {
          stats_.CountRecvOk();
          echo_first = false;
        }
        if (idx < 2 && (echo_got & canfd::FrameBit(idx)) == 0) {  // ②
          echo_got = static_cast<uint8_t>(echo_got | canfd::FrameBit(idx));
          const canfd_frame& sent = (idx == 0) ? tx_frame0_ : tx_frame1_;
          if (rx_.len != canfd::kFrameLen ||
              std::memcmp(rx_.data, sent.data, canfd::kFrameLen) != 0) {
            ++canfd_working_.echo_payload_mismatch;
          }
          if (echo_got == (canfd::FrameBit(0) | canfd::FrameBit(1))) {
            ++stats_.PerKind(RequestKind::kWriteEcho).ok;
            echo_open = false;
            r.write_ok = true;
            r.write_end = Clock::now();
            if (set_open)
              assembler_.SetDeadline(r.write_end + stage_timeout_);  // chain
          }
          continue;
        }
        // ④ (idx ≥ 2 or a repeated idx) — falls through to the mismatch below
      } else if (cls == RxClass::kResp && set_open) {  // ③
        if (set_first) {
          stats_.CountRecvOk();
          set_first = false;
        }
        switch (assembler_.Feed(rx_.can_id, rx_.data, rx_.len)) {
          case canfd::FeedResult::kComplete:
            ++stats_.PerKind(RequestKind::kJointRead).ok;
            if (sensors)
              ++stats_.PerKind(RequestKind::kBulkSensor).ok;
            set_open = false;
            r.set_ok = true;
            r.set_end = Clock::now();
            break;
          case canfd::FeedResult::kStored:
          case canfd::FeedResult::kDuplicate:
            break;  // the assembler's own counters record these
          case canfd::FeedResult::kBadLength:
            stats_.CountShortOrDecode(RequestKind::kJointRead, rx_.len);
            break;
          case canfd::FeedResult::kOutsideSet:  // unreachable: ClassifyRx matched the base
          case canfd::FeedResult::kBadIndex:
            stats_.CountCmdMismatch(RequestKind::kJointRead, UnexpectedCmdOf(id), rx_.len);
            break;
        }
        continue;
      }
      // ④
      stats_.CountCmdMismatch(echo_open ? RequestKind::kWriteEcho : RequestKind::kJointRead,
                              UnexpectedCmdOf(id), rx_.len);
    }
    return r;
  }

  /// Copy a completed set into the read buffers and stamp kJoint at the set
  /// stage's close (completion, or the window end when it failed).
  void FinishRead(const WindowResult& r, const HandReadPlan& plan, HandReadBuffers& bufs,
                  HandReadOutcome& out) noexcept {
    if (r.set_ok) {
      const canfd::RespSet& set = assembler_.set();
      FillJoint(set, bufs.joint);
      out.joint_ok = true;
      if (plan.sensors) {
        static_assert(sizeof(canfd::RespSet) <= sizeof(bufs.sensor_raw),
                      "sensor_raw must hold a RespSet");
        std::memcpy(bufs.sensor_raw.data(), &set, sizeof(canfd::RespSet));
        out.sensor_recvd = static_cast<ssize_t>(sizeof(canfd::RespSet));
      }
    }
    out.phase_end[static_cast<std::size_t>(ReadPhase::kJoint)] = r.set_end;
  }

  void CountSetTimeout(bool sensors) noexcept {
    stats_.CountRecvTimeout(RequestKind::kJointRead);
    if (sensors)
      ++stats_.PerKind(RequestKind::kBulkSensor).timeout;
  }

  void CountSetError(bool sensors) noexcept {
    stats_.CountRecvError(RequestKind::kJointRead);
    if (sensors)
      ++stats_.PerKind(RequestKind::kBulkSensor).error;
  }

  void AbandonSet() noexcept {
    assembler_.Reset();
    stats_.CountStaleDrained(DrainQueued());
  }

  /// Non-blocking reads until the queue is empty or kMaxDrainPerCall; error
  /// frames are classified, data frames counted as drained.
  [[nodiscard]] int DrainQueued() noexcept {
    int drained = 0;
    for (int i = 0; i < kMaxDrainPerCall; ++i) {
      if (transport_.RecvFrameNonBlocking(rx_) <= 0)
        break;
      if (!ConsumeErrorFrame(rx_))
        ++drained;
    }
    return drained;
  }

  static void FillJoint(const canfd::RespSet& set, MotorTriplet& joint) noexcept {
    constexpr std::size_t n = std::min(canfd::kWireMotorCount, packets::kMotorDataCount);
    for (std::size_t i = 0; i < n; ++i) {
      joint.pos[i] = set.frame0.pos[i];
      joint.cur[i] = set.frame1.cur[i];
      joint.vel[i] = (i < canfd::kVelLoCount) ? set.frame0.vel_lo[i]
                                              : set.frame1.vel_hi[i - canfd::kVelLoCount];
    }
  }

  CanFdHandCommConfig cfg_;
  uint8_t node_;
  rtc::CanFdTransport transport_;
  canfd::RespSetAssembler assembler_;
  uint32_t echo_base_;  ///< EchoId(node, 0): this node's echoes under ~kFrameIndexMask
  uint32_t resp_base_;  ///< RespId(node, 0): this node's RESP burst under kRespIdMask

  std::array<float, kNumHandMotors> inv_kt_{};  ///< 1 / feedforward_kt, set in Open()
  Clock::duration stage_timeout_{};

  // Scratch reused every cycle (allocation-free RT path). The ids / lengths /
  // flags are fixed in the constructor; only the payloads change.
  canfd_frame tx_frame0_{};
  canfd_frame tx_frame1_{};
  canfd_frame tx_req_{};
  canfd_frame rx_{};

  HandCommStatsPair stats_;
  static_assert(std::is_trivially_copyable_v<CanFdCommStats>,
                "SeqLock payload must be trivially copyable");
  CanFdCommStats canfd_working_{};
  rtc::SeqLock<CanFdCommStats> canfd_seqlock_{};
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_CANFD_HAND_COMM_HPP_
