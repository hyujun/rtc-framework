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
//                             (D41, `cmd_ack` echo only) and the RESP burst idx
//                             0 1 2 in any order (D57) under the deadline chain
//                             below
//                                                     → kWriteEcho · kJointRead,
//                             and on a sensor cycle kBulkSensor too (D49: 1:1
//                             with proto1's per-kind stats)
//   WriteCommandWithEcho      the echo stage alone (no REQ)       → kWriteEcho
//                             (`cmd_ack` none: the CMD pair's send alone)
//   ReadCycle                 the set stage alone (REQ, no CMD) → kJointRead
//   SendCommandOnly ·         the shared-request split (#956, `req_mode`
//   SendBroadcastReq ·        broadcast, `cmd_ack` none only): the CMD pair
//   ReceiveSet                alone → kWriteEcho; one REQ to target 0xFF
//                             that every hand answers; this hand's set stage
//                             against a bound anchored at that REQ (REQ + T,
//                             + 2T after any hand's write) → kJointRead
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
// Command acknowledgement (#945 D61 · D63 · D64), CanFdHandCommConfig::cmd_ack
// — it must match the firmware:
//   echo  the SDO-like firmware echoes every CMD frame: the echo stage and the
//         chain above.
//   none  the PDO-like firmware applies a CMD silently: no echo stage opens, a
//         write is ok (kWriteEcho ok, HandReadOutcome::write_ok) when both CMD
//         frames were sent — write_rx stays false, since nothing came back
//         (D68) — and the set stage keeps the nominal CMD + 2T (from the end of
//         the CMD sends) when Frame 0 went out, since the hand still works
//         through the CMD before the REQ; REQ + T otherwise. The 2T bound is the
//         same in both modes.
// A mismatch fails no stage outright, it shows in counters: echo against a
// silent firmware times every echo stage out; none against an echoing firmware
// sees each echo as class ④ and counts it in echo_unexpected.
//
// Frame classification (D41 · D58 · D59). Every frame the filter lets through:
//   ① an error message frame (CAN_ERR_FLAG, subscribed through error_mask) →
//      CanFdCommStats::error_frames / bus_off only;
//   ② this node's echo with idx 0 or 1 not yet seen while the echo stage is
//      open (never in `cmd_ack` none) → accepted in any order; a payload that differs from what was
//      sent only bumps echo_payload_mismatch (the hand applied its own copy — nothing to re-send);
//   ③ this node's RESP while the set stage is open → RespSetAssembler::Feed
//      (duplicates, bad index / length counted by it, D57);
//   ④ anything else (an echo idx ≥ 2 or a repeated idx, a RESP outside an
//      open set stage, a late echo after its stage closed, another type) →
//      cmd_mismatch + last_unexpected_* only, attributed to kWriteEcho while
//      the echo stage is open and to kJointRead otherwise — however many
//      arrive, the open stages keep waiting (D58). Nothing is re-sent (D41).
//      An echo while no echo stage is open (late, a read-only tick, or any
//      echo under `cmd_ack` none) also bumps CanFdCommStats::echo_unexpected.
// recv_ok counts the first frame of each open stage's id class — an echo id
// for the echo stage, a RESP id for the set stage, whatever its index or
// length (a frame-level arrival, as on UDP) — so ≤ 2 per cycle; every stage ends on exactly one
// terminal counter (ok, timeout or error), so the per-kind attempt denominators hold. A failed CMD
// send fails the echo stage before any wait (recv_error of kWriteEcho beside the tx errno counter;
// the REQ still goes out); a failed REQ send fails the set stage the same way (the echo stage still
// waits); a recv error ends every open stage with an error. On this link a dead peer / downed
// interface surfaces on send (ENETDOWN, ENOBUFS), which is the role ECONNREFUSED plays on UDP.
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
// error frames, echo payload mismatches and unexpected echoes, the assembler's counters) behind its
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
  /// Whether the firmware echoes CMD frames (#945 D61) — see the header comment.
  canfd::CmdAck cmd_ack{canfd::CmdAck::kEcho};
  /// Per-stage receive deadline T (echo stage · RESP set stage), D42; under
  /// `cmd_ack` none the set stage after a write gets the nominal 2T (D64).
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
  /// Echo frames of this node received while no echo stage was open (late
  /// echoes, echoes on a read-only tick, every echo under `cmd_ack` none —
  /// the sign of an echoing firmware behind `cmd_ack` none, D63). Each is
  /// also a class-④ cmd_mismatch.
  uint64_t echo_unexpected{0};
  /// RespSetAssembler counters, copied in at publish time.
  canfd::AssemblerStats assembler{};
};

/// A `cmd_ack` setting the firmware disagrees with, as the counters show it
/// (#945 D63). The node evaluates it on each stats save and warns once per
/// kind; it never switches the mode itself.
enum class CmdAckMismatch : uint8_t {
  kNone,
  kFirmwareSilent,  ///< `cmd_ack` echo, but no write ever saw its echoes
  kFirmwareEchoes,  ///< `cmd_ack` none, but echoes arrive
};

/// Writes an all-timeout echo record needs before it reads as a silent firmware
/// rather than a start-up transient.
inline constexpr uint64_t kCmdAckMismatchMinWrites = 100;

/// echo: ≥ kCmdAckMismatchMinWrites echo timeouts and not one echoed write,
/// while the RESP set still completes (a live hand — a dead link times out
/// both) and no echo arrived late (echoes after T mean T is too short, not a
/// silent firmware); a send error here and there does not hide it. none: any
/// echo_unexpected.
[[nodiscard]] constexpr CmdAckMismatch CmdAckMismatchDecision(
    canfd::CmdAck ack, const UdpHandCommStats::PerKind& write_echo, uint64_t joint_read_ok,
    uint64_t echo_unexpected) noexcept {
  if (ack == canfd::CmdAck::kNone)
    return echo_unexpected > 0 ? CmdAckMismatch::kFirmwareEchoes : CmdAckMismatch::kNone;
  return (write_echo.ok == 0 && write_echo.timeout >= kCmdAckMismatchMinWrites &&
          joint_read_ok > 0 && echo_unexpected == 0)
             ? CmdAckMismatch::kFirmwareSilent
             : CmdAckMismatch::kNone;
}

class CanFdHandComm final : public HandComm {
 public:
  using Clock = std::chrono::steady_clock;

  /// Max non-blocking reads per DrainStale() call: the controller's 10-frame
  /// Rx FIFO plus a late (or, under a `cmd_ack` mismatch, unawaited) echo
  /// pair, with headroom. A residual backlog is picked
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
    tx_req_bc_ = tx_req_;
    tx_req_bc_.can_id = canfd::ReqId(canfd::kNodeBroadcast) | CAN_EFF_FLAG;
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

  /// The shared-request split has no echo stage, so it serves `cmd_ack` none
  /// only (#956 D81).
  [[nodiscard]] bool SupportsBroadcastReq() const noexcept override {
    return cfg_.cmd_ack == canfd::CmdAck::kNone;
  }

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
    out.write_rx = r.write_ok && cfg_.cmd_ack == canfd::CmdAck::kEcho;  // D68
    out.write_end = r.write_end;
    FinishRead(r, plan, bufs, out);
  }

  /// Shared-request tick, step 1 (#956): the CMD pair alone. The write is
  /// the send, as under `cmd_ack` none in RunCycle (the only mode this split
  /// serves): kWriteEcho ok when both frames went out, error otherwise.
  [[nodiscard]] bool SendCommandOnly(const HandMotorArray& pos, const HandMotorArray& ff_nm,
                                     HandReadOutcome& out) noexcept override {
    RTC_TRACE_SCOPE("hand_cmd_send");
    if (!transport_.is_open()) {
      out.write_end = Clock::now();
      return false;
    }
    BuildCommandFrames(pos, ff_nm);
    const bool frame0_sent = SendCounted(tx_frame0_);
    const bool sent = frame0_sent && SendCounted(tx_frame1_);
    out.write_end = Clock::now();
    if (sent) {
      ++stats_.PerKind(RequestKind::kWriteEcho).ok;
      out.write_ok = true;
    } else {
      stats_.CountRecvError(RequestKind::kWriteEcho);
    }
    out.write_rx = false;  // D68: nothing came back
    return frame0_sent;
  }

  /// Shared-request tick, step 2: the broadcast REQ (`0x007FC000`), sent on
  /// this comm's socket for every hand. A failed send counts in this comm's
  /// tx_* only; each hand's ReceiveSet counts its set error.
  [[nodiscard]] bool SendBroadcastReq(Clock::time_point& t_sent) noexcept override {
    const bool sent = transport_.is_open() && SendCounted(tx_req_bc_);
    t_sent = Clock::now();
    return sent;
  }

  /// Shared-request tick, step 3: this hand's RESP set (the set stage alone)
  /// against the bound `req.t_sent` + T, or + 2T after a write (D83).
  ///
  /// Why the anchor is the broadcast send and not this call: the hands answer
  /// the one REQ concurrently, so a later hand's set is on the bus — and in
  /// this socket's queue — while an earlier hand's window is still open;
  /// anchoring at this call would hand every later hand a bound up to a whole
  /// window too late. A write tick puts the CMD frames on the bus before the
  /// REQ: the rig probe (#956, 2026-10-25) measured first CMD F0 → last F2 at
  /// 1.919 ms max, inside 2T = 2.8 ms at the probe's T 1.4.
  ///
  /// Frames already queued are fed first, whatever the clock says (S5): when
  /// an earlier hand's window ran past this bound, the set that arrived in
  /// time must not time out unread. Only then does the bound decide. The loop
  /// calls every hand's ReceiveSet back to back (no tail in between), so what
  /// is queued at this call had arrived by the earlier window's close. The
  /// pre-feed reads at most kMaxDrainPerCall frames, so a stream of
  /// duplicates or error frames cannot keep it from reaching the bound.
  void ReceiveSet(const HandReadPlan& plan, HandReadBuffers& bufs, HandReadOutcome& out,
                  const SharedRequest& req) noexcept override {
    RTC_TRACE_SCOPE("hand_set_window");
    ResetReadFields(out);
    out.phase_end[static_cast<std::size_t>(ReadPhase::kMotor)] = req.t_sent;
    if (!transport_.is_open()) {
      out.StampPhase(ReadPhase::kJoint);
      return;
    }
    Window w{};
    w.sensors = plan.sensors;
    if (!req.sent) {
      CountSetError(plan.sensors);  // S4: no REQ went out — an error, not a timeout
      w.r.set_end = Clock::now();
      FinishRead(w.r, plan, bufs, out);
      return;
    }
    w.set_open = true;
    assembler_.Begin(req.t_sent + (req.after_write ? 2 : 1) * stage_timeout_);
    for (int i = 0; i < kMaxDrainPerCall && w.set_open; ++i) {
      const ssize_t n = transport_.RecvFrameNonBlocking(rx_);
      if (n == 0)
        break;
      if (n < 0) {
        CloseOnRecvError(w);
        break;
      }
      HandleFrame(w);
    }
    ReceiveUntilClosed(w);
    FinishRead(w.r, plan, bufs, out);
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

  /// Relative ppoll timeout; a past deadline is 0 (ppoll refuses a negative one).
  [[nodiscard]] static timespec ToTimespec(Clock::duration d) noexcept {
    if (d < Clock::duration::zero())
      d = Clock::duration::zero();
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
    bool write_ok{false};           ///< both echoes arrived (none: both CMD frames sent)
    bool set_ok{false};             ///< the RESP set completed (set stage only)
    Clock::time_point write_end{};  ///< echo stage close (echoes, deadline or error; none: sent)
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

  /// The open stages of one receive window and what they reported so far
  /// (RunWindow · ReceiveSet).
  struct Window {
    bool echo_open{false};
    bool set_open{false};
    bool sensors{false};  ///< the set stage also closes a kBulkSensor attempt
    bool echo_first{true};
    bool set_first{true};
    uint8_t echo_got{0};
    Clock::time_point echo_deadline{};
    WindowResult r{};
  };

  /// The receive window (D59). Sends what the flags ask for — the CMD pair
  /// (tx_frame0_/1_ already built) and / or the REQ — then receives until
  /// every stage that opened has closed. Deadline chain: echo stage CMD + T;
  /// set stage (echo close) + T, nominal CMD + 2T until the echoes arrive, REQ
  /// + T when no CMD went out. `cmd_ack` none opens no echo stage: the write
  /// is the send, the set stage keeps CMD + 2T when Frame 0 went out (D64).
  /// ppoll waits for the nearest open deadline.
  [[nodiscard]] WindowResult RunWindow(bool send_cmd, bool send_req, bool sensors) noexcept {
    Window w{};
    w.sensors = sensors;
    // ── Send stage ───────────────────────────────────────────────────────
    // The set stage's upper bound, decided by what the CMD did: echo stage
    // close + T (pulled in by SetDeadline when the echoes arrive); under
    // cmd_ack none the end of the CMD sends + 2T once Frame 0 went out (D64);
    // left at the epoch — REQ + T — when no CMD reached the bus.
    Clock::time_point set_bound{};
    if (send_cmd) {
      const bool frame0_sent = SendCounted(tx_frame0_);
      const bool sent = frame0_sent && SendCounted(tx_frame1_);
      const Clock::time_point t_sent = Clock::now();
      if (cfg_.cmd_ack == canfd::CmdAck::kNone) {
        if (sent) {
          ++stats_.PerKind(RequestKind::kWriteEcho).ok;
          w.r.write_ok = true;
        } else {
          stats_.CountRecvError(RequestKind::kWriteEcho);
        }
        w.r.write_end = t_sent;
        if (frame0_sent)
          set_bound = t_sent + 2 * stage_timeout_;
      } else if (sent) {
        w.echo_open = true;
        w.echo_deadline = t_sent + stage_timeout_;
        set_bound = w.echo_deadline + stage_timeout_;
      } else {
        stats_.CountRecvError(RequestKind::kWriteEcho);
        w.r.write_end = t_sent;
      }
    }
    if (send_req) {
      if (SendCounted(tx_req_)) {
        w.set_open = true;
        // Begin() resets the assembler, so it is called once here.
        assembler_.Begin(set_bound != Clock::time_point{} ? set_bound
                                                          : Clock::now() + stage_timeout_);
      } else {
        CountSetError(sensors);
        w.r.set_end = Clock::now();
      }
    }
    ReceiveUntilClosed(w);
    return w.r;
  }

  /// The receive loop of a window: each pass closes what has expired, then
  /// waits for the nearest open deadline and hands the frame to HandleFrame.
  void ReceiveUntilClosed(Window& w) noexcept {
    while (w.echo_open || w.set_open) {
      const Clock::time_point now = Clock::now();
      if (w.echo_open && now >= w.echo_deadline) {
        stats_.CountRecvTimeout(RequestKind::kWriteEcho);
        w.echo_open = false;
        w.r.write_end = now;  // the set deadline stays at the nominal CMD + 2T
      }
      if (w.set_open && assembler_.Expired(now)) {
        CountSetTimeout(w.sensors);
        AbandonSet();
        w.set_open = false;
        w.r.set_end = now;
      }
      if (!w.echo_open && !w.set_open)
        break;
      Clock::time_point next = w.set_open ? assembler_.deadline() : w.echo_deadline;
      if (w.echo_open && w.echo_deadline < next)
        next = w.echo_deadline;
      const ssize_t n = transport_.RecvFrameWithTimeout(rx_, ToTimespec(next - now));
      if (n == 0)
        continue;  // deadline or a spurious wake: the loop head decides
      if (n < 0) {
        CloseOnRecvError(w);
        break;
      }
      HandleFrame(w);
    }
  }

  /// A recv error ends every open stage with an error.
  void CloseOnRecvError(Window& w) noexcept {
    const Clock::time_point t_err = Clock::now();
    if (w.echo_open) {
      stats_.CountRecvError(RequestKind::kWriteEcho);
      w.echo_open = false;
      w.r.write_end = t_err;
    }
    if (w.set_open) {
      CountSetError(w.sensors);
      AbandonSet();
      w.set_open = false;
      w.r.set_end = t_err;
    }
  }

  /// Classify and account the frame in rx_ against the open stages (header
  /// comment, classes ① – ④).
  void HandleFrame(Window& w) noexcept {
    if (ConsumeErrorFrame(rx_))
      return;  // ①
    const uint32_t id = rx_.can_id & canfd::kExtIdMask;
    const uint8_t idx = canfd::FrameIndexOf(id);
    const RxClass cls = ClassifyRx(id);
    if (cls == RxClass::kEcho && w.echo_open) {
      if (w.echo_first) {
        stats_.CountRecvOk();
        w.echo_first = false;
      }
      if (idx < 2 && (w.echo_got & canfd::FrameBit(idx)) == 0) {  // ②
        w.echo_got = static_cast<uint8_t>(w.echo_got | canfd::FrameBit(idx));
        const canfd_frame& sent = (idx == 0) ? tx_frame0_ : tx_frame1_;
        if (rx_.len != canfd::kFrameLen ||
            std::memcmp(rx_.data, sent.data, canfd::kFrameLen) != 0) {
          ++canfd_working_.echo_payload_mismatch;
        }
        if (w.echo_got == (canfd::FrameBit(0) | canfd::FrameBit(1))) {
          ++stats_.PerKind(RequestKind::kWriteEcho).ok;
          w.echo_open = false;
          w.r.write_ok = true;
          w.r.write_end = Clock::now();
          if (w.set_open)
            assembler_.SetDeadline(w.r.write_end + stage_timeout_);  // chain
        }
        return;
      }
      // ④ (idx ≥ 2 or a repeated idx) — falls through to the mismatch below
    } else if (cls == RxClass::kResp && w.set_open) {  // ③
      if (w.set_first) {
        stats_.CountRecvOk();
        w.set_first = false;
      }
      switch (assembler_.Feed(rx_.can_id, rx_.data, rx_.len)) {
        case canfd::FeedResult::kComplete:
          ++stats_.PerKind(RequestKind::kJointRead).ok;
          if (w.sensors)
            ++stats_.PerKind(RequestKind::kBulkSensor).ok;
          w.set_open = false;
          w.r.set_ok = true;
          w.r.set_end = Clock::now();
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
      return;
    }
    // ④
    if (cls == RxClass::kEcho && !w.echo_open)
      ++canfd_working_.echo_unexpected;
    stats_.CountCmdMismatch(w.echo_open ? RequestKind::kWriteEcho : RequestKind::kJointRead,
                            UnexpectedCmdOf(id), rx_.len);
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
  canfd_frame tx_req_bc_{};  ///< the broadcast REQ (#956) — the only frame to target 0xFF
  canfd_frame rx_{};

  HandCommStatsPair stats_;
  static_assert(std::is_trivially_copyable_v<CanFdCommStats>,
                "SeqLock payload must be trivially copyable");
  CanFdCommStats canfd_working_{};
  rtc::SeqLock<CanFdCommStats> canfd_seqlock_{};
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_CANFD_HAND_COMM_HPP_
