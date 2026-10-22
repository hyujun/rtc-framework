#ifndef UDP_HAND_DRIVER_HAND_COMM_HPP_
#define UDP_HAND_DRIVER_HAND_COMM_HPP_

// HandComm: the hand communication-cycle seam (epic #895, decision D25).
//
// UdpHandController drives one comm cycle per CommLoop tick: drain → gated
// write (+ echo) → reads → publish. Which bytes go over which link is the only
// thing that differs between hand protocols, so that is what this interface
// abstracts — a whole cycle's worth of transactions, not a byte pipe:
//
//   - UdpHandComm (udp_hand_comm.hpp): proto1a / proto1b over UDP. Wraps the
//     pre-existing UdpHandTransport; a cycle is the same request/response
//     sequence the controller used to issue directly.
//   - CanFdHandComm (canfd_hand_comm.hpp): proto2 over CAN FD. Sends the
//     command as two frames and reads one REQ → three-frame RESP burst per
//     cycle; it cannot be split into per-request calls without caching, which
//     is why the read side is one ReadCycle() and not three virtuals.
//
// Ownership of the bookkeeping stays with the controller: ReadCycle reports
// per-channel outcomes and the phase boundaries inside the read; the
// controller builds the attempted / ok masks, the per-channel failure streaks
// and the timing profile from them, stamps the end of the read itself, and
// decodes the raw bulk-sensor response through its SensorProtocol. The comm
// knows nothing about the outcome ring.
//
// Thread contract (same as the transport it replaces): the "RT" group runs on
// the SCHED_FIFO CommLoop thread every tick and must be noexcept and
// allocation-free in every implementation (virtual dispatch is the only cost
// added by the seam); the "non-RT" group runs on the lifecycle / Start / Stop
// path before the loop launches or after it joined. comm_stats() may be read
// from any thread — it is a SeqLock snapshot published by PublishCommStats().
//
// The pre-existing UDP type names are kept (UdpHandCommStats, RequestKind);
// they moved here unchanged so this header does not pull in the socket headers.

#include "rtc_base/threading/seqlock.hpp"
#include "udp_hand_driver/udp_hand_constants.hpp"
#include "udp_hand_driver/udp_hand_packets.hpp"

#include <sys/types.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace udp_hand_driver {

// ── Communication mode ───────────────────────────────────────────────────────
// Which read sequence a cycle issues. Individual mode is a UDP-only legacy; a
// comm that does not implement it says so through SupportsIndividualMode() and
// the controller refuses to start in that mode.
enum class HandCommunicationMode {
  kIndividual,  // per-motor pos + vel reads, per-finger sensor reads
  kBulk,        // one all-motors read (per space) + one all-sensors read
};

// Request kind: the comm-neutral vocabulary of the per-kind statistics and of
// the controller's cycle-outcome masks (values are bit positions — keep them
// dense from 0). The names are the stats JSON `per_request` keys, so every
// link maps its own transactions onto these six; the UDP proto1 command bytes
// behind each one are documented in UdpHandComm. The caller disambiguates
// joint- vs motor-space reads, which share a wire format.
enum class RequestKind : uint8_t {
  kWriteEcho = 0,   // command write + echo
  kMotorRead = 1,   // motor-space read (pos / vel / cur)
  kJointRead = 2,   // joint-space read
  kSensorRead = 3,  // per-finger sensor read (individual mode)
  kBulkSensor = 4,  // all-sensors read (bulk mode)
  kSetMode = 5,     // sensor-mode handshake (InitializeSensors)
};

inline constexpr int kNumRequestKinds = 6;

// Stable short names for logs / stats JSON, indexed by RequestKind.
inline constexpr std::array<const char*, kNumRequestKinds> kRequestKindNames = {
    "write_echo", "motor_read", "joint_read", "sensor_read", "bulk_sensor", "set_mode"};

// Bit for a RequestKind in the cycle-outcome attempted/ok masks.
[[nodiscard]] constexpr uint8_t RequestKindBit(RequestKind kind) noexcept {
  return static_cast<uint8_t>(1u << static_cast<unsigned>(kind));
}

// Communication statistics (recv success/timeout/error and total cycles).
// Single writer (CommLoop); other threads read the SeqLock snapshot.
struct UdpHandCommStats {
  uint64_t recv_ok{0};
  uint64_t recv_timeout{0};
  uint64_t recv_error{0};
  uint64_t cmd_mismatch{0};
  uint64_t mode_mismatch{0};
  uint64_t total_cycles{0};
  uint64_t comm_decimation_skip_count{0};  // cycles skipped by comm_decimation
  // Datagrams dropped by the cycle-start drain (DrainStale): a 1-cycle desync
  // leaves the previous request's late response queued, which this cycle would
  // otherwise consume as a cmd_mismatch. Pre-request stage — not attributable
  // to a RequestKind, so aggregate-only (no per_kind entry).
  uint64_t stale_drained{0};

  // Per-request-kind breakdown. Unlike aggregate recv_ok (any first-attempt
  // packet arrival), per-kind `ok` counts request-level success — the method
  // returned validated data. `short_or_decode` counts short packets and codec
  // decode failures (both drop the packet and retry).
  struct PerKind {
    uint64_t ok{0};
    uint64_t timeout{0};
    uint64_t error{0};
    uint64_t cmd_mismatch{0};
    uint64_t mode_mismatch{0};
    uint64_t short_or_decode{0};
  };

  std::array<PerKind, kNumRequestKinds> per_kind{};

  // Forensics for the most recent rejected packet: the CMD byte of the last
  // cmd_mismatch (a stale echo of the *previous* request implies desync), and
  // the byte count of the last rejected recv (mismatch or short).
  uint8_t last_unexpected_cmd{0};
  uint16_t last_unexpected_len{0};
};

// ── Stats pair ───────────────────────────────────────────────────────────────
// The working copy / published snapshot pair every HandComm keeps, with the
// counting vocabulary the links share, so a second link does not fork the
// bookkeeping (P5). Single writer (the CommLoop thread) mutates `working()` and
// publishes it once per cycle; any thread reads `Load()`. Hot-path methods are
// counter increments only — noexcept, allocation-free (RT-safe).
//
// Which event each link maps onto which counter is the link's business
// (UdpHandTransport: a datagram; CanFdHandComm: a frame / a transaction) — the
// JSON keys and their meaning in UdpHandCommStats stay the same.
class HandCommStatsPair {
 public:
  [[nodiscard]] UdpHandCommStats& working() noexcept { return working_; }

  [[nodiscard]] const UdpHandCommStats& working() const noexcept { return working_; }

  [[nodiscard]] UdpHandCommStats::PerKind& PerKind(RequestKind kind) noexcept {
    return working_.per_kind[static_cast<std::size_t>(kind)];
  }

  /// Consistent snapshot for off-loop readers (SeqLock retry loop: no torn
  /// per_kind / aggregate or cmd / len pair).
  [[nodiscard]] UdpHandCommStats Load() const noexcept { return seqlock_.Load(); }

  /// Publish the working copy (~400 B memcpy + 2 fences; once per cycle).
  void Publish() noexcept { seqlock_.Store(working_); }

  /// Zero everything and publish the cleared snapshot (restart begins clean —
  /// a reader between restart and the first cycle sees zeros, not stale data).
  void Reset() noexcept {
    working_ = {};
    recv_error_count_.store(0, std::memory_order_relaxed);
    seqlock_.Store(working_);
  }

  /// Receive failures (timeouts + errors) since the last reset. Any thread.
  [[nodiscard]] uint64_t recv_error_count() const noexcept {
    return recv_error_count_.load(std::memory_order_relaxed);
  }

  // ── Counting (CommLoop thread) ───────────────────────────────────────────

  /// First-attempt packet / frame arrival, validity-independent.
  void CountRecvOk() noexcept { ++working_.recv_ok; }

  void CountRecvTimeout(RequestKind kind) noexcept {
    recv_error_count_.fetch_add(1, std::memory_order_relaxed);
    ++working_.recv_timeout;
    ++PerKind(kind).timeout;
  }

  void CountRecvError(RequestKind kind) noexcept {
    recv_error_count_.fetch_add(1, std::memory_order_relaxed);
    ++working_.recv_error;
    ++PerKind(kind).error;
  }

  /// Classify a failed blocking recv by errno: EAGAIN / EWOULDBLOCK is a
  /// timeout, anything else a socket error.
  void CountRecvFailErrno(RequestKind kind) noexcept {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      CountRecvTimeout(kind);
    } else {
      CountRecvError(kind);
    }
  }

  /// A packet / frame that is not the one this request waits for. Records the
  /// rejected cmd + len into the working copy; the whole-struct publish keeps
  /// the pair consistent for cross-thread readers.
  void CountCmdMismatch(RequestKind kind, uint8_t got_cmd, ssize_t recvd) noexcept {
    ++working_.cmd_mismatch;
    ++PerKind(kind).cmd_mismatch;
    working_.last_unexpected_cmd = got_cmd;
    working_.last_unexpected_len = static_cast<uint16_t>(recvd);
  }

  void CountModeMismatch(RequestKind kind) noexcept {
    ++working_.mode_mismatch;
    ++PerKind(kind).mode_mismatch;
  }

  /// Short packet or codec decode failure. Updates only the len, preserving
  /// the last cmd byte.
  void CountShortOrDecode(RequestKind kind, ssize_t recvd) noexcept {
    ++PerKind(kind).short_or_decode;
    working_.last_unexpected_len = static_cast<uint16_t>(recvd);
  }

  void CountStaleDrained(int drained) noexcept {
    working_.stale_drained += static_cast<uint64_t>(drained);
  }

 private:
  static_assert(std::is_trivially_copyable_v<UdpHandCommStats>,
                "SeqLock payload must be trivially copyable");
  // CommLoop-thread-only working copy: mutated in place every cycle, then
  // published as one consistent snapshot at cycle end.
  UdpHandCommStats working_{};
  // Reader-visible snapshot. Single writer = CommLoop via Publish() / Reset();
  // readers = stats timer + failure detector (Load()).
  rtc::SeqLock<UdpHandCommStats> seqlock_{};
  std::atomic<uint64_t> recv_error_count_{0};
};

// ── Read-cycle contract ──────────────────────────────────────────────────────

using HandMotorArray = std::array<float, kNumHandMotors>;

// Bit / slot helpers for the per-finger sensor channel (bit i = finger i).
static_assert(kMaxFingertips <= 8, "finger masks are 8 bits");

[[nodiscard]] constexpr uint8_t FingerBit(int finger) noexcept {
  return static_cast<uint8_t>(1u << static_cast<unsigned>(finger));
}

/// Offset of finger `finger`'s values in a kMaxHandSensors-sized array.
[[nodiscard]] constexpr std::ptrdiff_t FingerOffset(int finger) noexcept {
  return static_cast<std::ptrdiff_t>(finger) * kSensorValuesPerFingertip;
}

// What the controller wants read this cycle. Every field but `sensors` is a
// per-run constant (hoisted in PrepareRun from the SensorProtocol / config);
// `sensors` flips per cycle with the sensor decimation. Passing the constants
// with every call keeps the comm stateless about the protocol.
struct HandReadPlan {
  bool motor_space_read{
      true};              ///< bulk: issue the motor-space read (SensorProtocol::HasMotorSpaceRead)
  bool sensors{false};    ///< this cycle is a sensor cycle
  int num_fingertips{0};  ///< individual: fingers polled
  std::size_t sensor_response_size{0};  ///< bulk: expected all-sensors response bytes
};

/// Position / velocity / current of one motor space (the wire width, not the
/// published kNumHandMotors — the controller copies the leading motors).
struct MotorTriplet {
  std::array<float, packets::kMotorDataCount> pos{};
  std::array<float, packets::kMotorDataCount> vel{};
  std::array<float, packets::kMotorDataCount> cur{};
};

// Controller-owned scratch the comm fills in place (preallocated, reused every
// cycle — no per-tick zeroing). A buffer's content is defined only for the
// channel whose outcome flag is set; the controller copies from there into the
// published state.
struct HandReadBuffers {
  MotorTriplet motor;  ///< motor-space (bulk: pos/vel/cur; individual: pos + vel)
  MotorTriplet joint;  ///< joint-space (bulk: pos/vel/cur; individual: pos)
  /// Bulk raw all-sensors response (sized for the largest response); decoded
  /// by the controller's SensorProtocol, which owns the per-version layout.
  std::array<uint8_t, packets::kMaxPacketSize> sensor_raw{};
  /// Individual per-finger decoded values, finger i at FingerOffset(i).
  std::array<int32_t, kMaxHandSensors> sensor_values{};
};

/// Boundaries INSIDE a read cycle that the timing profiler splits on. The end
/// of the whole read is stamped by the controller after its own decode / cache
/// copies, so the comm never stamps a sensor phase.
///   bulk       stamps kMotor (after the motor-space read) and kJoint
///   individual stamps kMotor (pos), kJoint (joint pos) and kVel
enum class ReadPhase : uint8_t { kMotor = 0, kJoint = 1, kVel = 2 };

inline constexpr std::size_t kNumReadPhases = 3;

// Per-channel outcome of one read cycle. Trivially resettable (`out = {}`): a
// phase an implementation does not stamp stays at the epoch, and the
// controller clamps each boundary to be no earlier than the previous one, so
// an unstamped phase reads as a zero-length span rather than garbage.
struct HandReadOutcome {
  using Clock = std::chrono::steady_clock;

  bool motor_ok{false};       ///< motor-space read validated (bulk pos/vel/cur, individual pos)
  bool joint_ok{false};       ///< joint-space read validated (bulk pos/vel/cur, individual pos)
  bool vel_ok{false};         ///< individual only: motor-space velocity read validated
  ssize_t sensor_recvd{-1};   ///< bulk: raw bytes received (−1 = none / failed)
  uint8_t finger_ok_mask{0};  ///< individual: FingerBit(i) set when finger i validated
  packets::JointMode received_motor_mode{
      packets::JointMode::kMotor};  ///< MODE echoed by the motor-space read
  std::array<Clock::time_point, kNumReadPhases> phase_end{};

  void StampPhase(ReadPhase phase) noexcept {
    phase_end[static_cast<std::size_t>(phase)] = Clock::now();
  }

  [[nodiscard]] Clock::time_point PhaseEnd(ReadPhase phase) const noexcept {
    return phase_end[static_cast<std::size_t>(phase)];
  }

  [[nodiscard]] bool FingerOk(int finger) const noexcept {
    return (finger_ok_mask & FingerBit(finger)) != 0;
  }
};

// ── Interface ────────────────────────────────────────────────────────────────

class HandComm {
 public:
  virtual ~HandComm() = default;

  // ── Non-RT (lifecycle / Start / Stop path) ───────────────────────────────

  /// Open the link. Returns false on failure (logged by the implementation).
  [[nodiscard]] virtual bool Open() noexcept = 0;

  /// Close the link. Idempotent.
  virtual void Close() noexcept = 0;

  /// Protocol-specific sensor handshake after Open (UDP: NN → RAW mode with
  /// retries). Returns false if the hand did not acknowledge.
  [[nodiscard]] virtual bool InitializeSensors() noexcept = 0;

  /// MODE-byte validation gates (SensorProtocol::VerifiesResponseMode /
  /// VerifiesBulkSensorResponseMode). Links without a MODE byte leave the
  /// defaults (no-ops).
  virtual void set_verify_response_mode(bool /*v*/) noexcept {}

  virtual void set_verify_bulk_sensor_mode(bool /*v*/) noexcept {}

  /// Joint-space I/O MODE the command write and the joint-space read carry
  /// (SensorProtocol::JointIoMode — 1a kJoint, 1b kMotor). Links without a
  /// MODE byte leave the default (no-op).
  virtual void set_joint_io_mode(packets::JointMode /*mode*/) noexcept {}

  /// Zero all counters + forensics and publish the cleared snapshot (restart
  /// begins clean — see UdpHandTransport::ResetCommStats).
  virtual void ResetCommStats() noexcept = 0;

  /// Consistent snapshot for off-loop readers (stats timer, failure detector).
  [[nodiscard]] virtual UdpHandCommStats comm_stats() const noexcept = 0;

  /// Receive errors (not timeouts) since the last reset. Any thread.
  [[nodiscard]] virtual uint64_t recv_error_count() const noexcept = 0;

  /// Per-stage receive deadline the implementation was configured with.
  [[nodiscard]] virtual double recv_timeout_ms() const noexcept = 0;

  /// Whether ReadCycleIndividual() is implemented (UDP only).
  /// UdpHandController::PrepareRun() refuses to start `individual` on a comm
  /// that returns false, so a non-UDP link never runs the default below.
  [[nodiscard]] virtual bool SupportsIndividualMode() const noexcept { return false; }

  // ── RT (CommLoop thread, every tick) — noexcept, allocation-free ─────────

  /// Drop anything left queued from a prior cycle before this cycle's first
  /// request (bounded, non-blocking). Returns the count dropped; accumulates
  /// into comm_stats_mut().stale_drained. Link not open → 0.
  virtual int DrainStale() noexcept = 0;

  /// Send the position command (rad) with per-motor feedforward torque (Nm) and
  /// wait for the hand's echo. Returns true when the echo validated. Links
  /// without a feedforward channel ignore `ff_nm` (UDP proto1).
  [[nodiscard]] virtual bool WriteCommandWithEcho(const HandMotorArray& pos,
                                                  const HandMotorArray& ff_nm) noexcept = 0;

  /// Send the command without waiting for any echo — the terminal E-Stop
  /// zero-write (both arrays zero). Must not block beyond the send itself.
  virtual void WriteCommandFireAndForget(const HandMotorArray& pos,
                                         const HandMotorArray& ff_nm) noexcept = 0;

  /// One bulk read cycle: motor-space (if planned) → joint-space → sensors (if
  /// planned). Fills `bufs` for the channels that validated and reports them in
  /// `out` (reset by the implementation) with the kMotor / kJoint boundaries.
  /// Every per-request failure is already counted in comm_stats_mut().
  virtual void ReadCycle(const HandReadPlan& plan, HandReadBuffers& bufs,
                         HandReadOutcome& out) noexcept = 0;

  /// One individual-mode read cycle: pos → joint pos → vel → per-finger
  /// sensors. Default = unsupported: nothing attempted, every flag false
  /// (SupportsIndividualMode() gates it at Start).
  virtual void ReadCycleIndividual(const HandReadPlan& /*plan*/, HandReadBuffers& /*bufs*/,
                                   HandReadOutcome& out) noexcept {
    out = HandReadOutcome{};
  }

  /// Mutable working copy for the CommLoop's own per-cycle writes (total_cycles
  /// stamp). CommLoop thread only — published via PublishCommStats().
  [[nodiscard]] virtual UdpHandCommStats& comm_stats_mut() noexcept = 0;

  /// Publish the working copy as the reader-visible snapshot (once per cycle).
  virtual void PublishCommStats() noexcept = 0;

  /// Last-unexpected forensics for the CommLoop's throttled onset log
  /// (same-thread reads of the working copy).
  [[nodiscard]] virtual uint8_t last_unexpected_cmd() const noexcept = 0;
  [[nodiscard]] virtual uint16_t last_unexpected_len() const noexcept = 0;
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_HAND_COMM_HPP_
