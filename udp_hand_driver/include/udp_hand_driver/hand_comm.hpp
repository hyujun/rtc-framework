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
//     sequence the controller used to issue directly (bulk 0x10 / 0x10 / 0x19,
//     individual 0x11 / 0x11 / 0x12 / 0x14..0x17).
//   - A CAN FD implementation (proto2, epic #895 F04-2c) sends the command as
//     two frames and reads one REQ → three-frame RESP burst per cycle; it
//     cannot be split into per-request calls without caching, which is why
//     the read side is one ReadCycle() and not three virtuals.
//
// Ownership of the bookkeeping stays with the controller: ReadCycle reports
// per-channel outcomes and phase timestamps; the controller builds the
// attempted / ok masks, the per-channel failure streaks and the timing
// profile from them, and decodes the raw bulk-sensor response through its
// SensorProtocol. The comm knows nothing about the outcome ring.
//
// Thread contract (same as the transport it replaces): the "RT" group runs on
// the SCHED_FIFO CommLoop thread every tick and must be noexcept and
// allocation-free in every implementation (virtual dispatch is the only cost
// added by the seam); the "non-RT" group runs on the lifecycle / Start / Stop
// path before the loop launches or after it joined. comm_stats() may be read
// from any thread — it is a SeqLock snapshot published by PublishCommStats().
//
// Public names: the pre-existing UDP types keep their names (decision D1);
// UdpHandCommStats / RequestKind moved here unchanged so the abstract header
// does not pull in the socket headers.

#include "udp_hand_driver/udp_hand_constants.hpp"
#include "udp_hand_driver/udp_hand_packets.hpp"

#include <sys/types.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace udp_hand_driver {

// ── Communication mode ───────────────────────────────────────────────────────
// Which read sequence a cycle issues. Individual mode is a UDP-only legacy
// (decision D25); a comm that does not support it reports so through
// HandComm::SupportsIndividualMode() and the node rejects the combination at
// configure (D16).
enum class HandCommunicationMode {
  kIndividual,  // 0x11 pos + 0x12 vel + 0x14~0x17 sensors
  kBulk,        // 0x10 all motors + 0x19 all sensors
};

// Request kind for per-kind statistics attribution. The caller disambiguates
// joint- vs motor-space reads (same request method / cmd byte, different MODE),
// which the transport cannot infer from the wire format alone. Values are bit
// positions in the controller's cycle-outcome masks — keep them dense from 0.
// A non-UDP protocol maps its own transactions onto these kinds so the stats
// JSON `per_request` keys stay stable (epic #895 G-1).
enum class RequestKind : uint8_t {
  kWriteEcho = 0,   // 0x01 write position + echo
  kMotorRead = 1,   // 0x10/0x11/0x12 motor-space read
  kJointRead = 2,   // 0x10/0x11 joint-space read
  kSensorRead = 3,  // 0x14~0x17 individual sensor read
  kBulkSensor = 4,  // 0x19 bulk sensor read
  kSetMode = 5,     // 0x04 set sensor mode
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

// ── Read-cycle contract ──────────────────────────────────────────────────────

using HandMotorArray = std::array<float, kNumHandMotors>;

// What the controller wants read this cycle. Every field but `sensors` is a
// per-run constant (hoisted in PrepareRun from the SensorProtocol / config);
// `sensors` flips per cycle with the sensor decimation.
struct HandReadPlan {
  bool motor_space_read{true};  ///< bulk: issue the kMotor read (SensorProtocol::HasMotorSpaceRead)
  bool sensors{false};          ///< this cycle is a sensor cycle
  int num_fingertips{0};        ///< individual: fingers polled (0x14 + i)
  std::size_t sensor_response_size{
      0};  ///< bulk: expected 0x19 response bytes (SensorProtocol::ResponseSize)
};

// Controller-owned scratch the comm fills in place (preallocated, reused every
// cycle — no per-tick zeroing). A buffer's content is defined only for the
// channel whose outcome flag is set; the controller copies from there into the
// published state.
struct HandReadBuffers {
  // Motor-space (kMotor): bulk 0x10 pos/vel/cur, individual 0x11 pos + 0x12 vel.
  std::array<float, packets::kMotorDataCount> motor_pos{};
  std::array<float, packets::kMotorDataCount> motor_vel{};
  std::array<float, packets::kMotorDataCount> motor_cur{};
  // Joint-space (the protocol's joint I/O MODE): bulk 0x10 pos/vel/cur,
  // individual 0x11 pos.
  std::array<float, packets::kMotorDataCount> joint_pos{};
  std::array<float, packets::kMotorDataCount> joint_vel{};
  std::array<float, packets::kMotorDataCount> joint_cur{};
  // Bulk raw 0x19 response (sized for the largest response); decoded by the
  // controller's SensorProtocol, which owns the per-version layout.
  std::array<uint8_t, packets::kMaxPacketSize> sensor_raw{};
  // Individual per-finger decoded values, finger i at
  // [i * kSensorValuesPerFingertip, (i + 1) * kSensorValuesPerFingertip).
  std::array<int32_t, kMaxHandSensors> sensor_values{};
};

// Per-channel outcome of one read cycle plus the phase boundary timestamps the
// timing profiler consumes. Bulk and individual fill disjoint flag groups.
struct HandReadOutcome {
  using Clock = std::chrono::steady_clock;

  // Bulk channels.
  bool motor_space_ok{false};  ///< 0x10 kMotor read validated (only if planned)
  bool joint_space_ok{false};  ///< 0x10 joint-space read validated
  ssize_t sensor_recvd{-1};    ///< 0x19 raw bytes received (−1 = none / failed)
  packets::JointMode received_motor_mode{
      packets::JointMode::kMotor};  ///< MODE echoed by the motor-space read

  // Individual channels.
  bool pos_ok{false};         ///< 0x11 kMotor
  bool joint_pos_ok{false};   ///< 0x11 joint-space
  bool vel_ok{false};         ///< 0x12 kMotor
  uint8_t finger_ok_mask{0};  ///< bit i = finger i's 0x14+i read validated

  // End of each read phase, in issue order:
  //   bulk       [0] motor-space  [1] joint-space  [2] sensors  [3] = [2]
  //   individual [0] pos          [1] joint pos    [2] vel      [3] sensors
  // A phase not planned this cycle still stamps its end (zero-length span), so
  // the controller's phase arithmetic never reads an unset time_point.
  std::array<Clock::time_point, 4> phase_end{};
};

static_assert(kMaxFingertips <= 8, "HandReadOutcome::finger_ok_mask is 8 bits");

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
  /// VerifiesBulkSensorResponseMode). Protocols without a MODE byte ignore them.
  virtual void set_verify_response_mode(bool v) noexcept = 0;
  virtual void set_verify_bulk_sensor_mode(bool v) noexcept = 0;

  /// Joint-space I/O MODE the command write and the joint-space read carry
  /// (SensorProtocol::JointIoMode — 1a kJoint, 1b kMotor). Protocols without a
  /// MODE byte ignore it.
  virtual void set_joint_io_mode(packets::JointMode mode) noexcept = 0;

  /// Zero all counters + forensics and publish the cleared snapshot (restart
  /// begins clean — see UdpHandTransport::ResetCommStats).
  virtual void ResetCommStats() noexcept = 0;

  /// Consistent snapshot for off-loop readers (stats timer, failure detector).
  [[nodiscard]] virtual UdpHandCommStats comm_stats() const noexcept = 0;

  /// Receive errors (not timeouts) since the last reset. Any thread.
  [[nodiscard]] virtual uint64_t recv_error_count() const noexcept = 0;

  /// Per-stage receive deadline the implementation was configured with.
  [[nodiscard]] virtual double recv_timeout_ms() const noexcept = 0;

  /// Whether ReadCycleIndividual() is implemented (UDP only, decision D25). The
  /// node rejects `individual` for a comm that returns false (D16).
  [[nodiscard]] virtual bool SupportsIndividualMode() const noexcept { return false; }

  // ── RT (CommLoop thread, every tick) — noexcept, allocation-free ─────────

  /// Drop anything left queued from a prior cycle before this cycle's first
  /// request (bounded, non-blocking). Returns the count dropped; accumulates
  /// into comm_stats_mut().stale_drained. Link not open → 0.
  virtual int DrainStale() noexcept = 0;

  /// Send the position command (rad) with per-motor feedforward torque (Nm) and
  /// wait for the hand's echo. Returns true when the echo validated. Protocols
  /// without a feedforward channel ignore `ff_nm` (UDP proto1); the CAN FD
  /// implementation converts it to current (epic #895 D34 · D50).
  [[nodiscard]] virtual bool WriteCommandWithEcho(const HandMotorArray& pos,
                                                  const HandMotorArray& ff_nm) noexcept = 0;

  /// Send the command without waiting for any echo — the terminal E-Stop
  /// zero-write (both arrays zero). Must not block beyond the send itself.
  virtual void WriteCommandFireAndForget(const HandMotorArray& pos,
                                         const HandMotorArray& ff_nm) noexcept = 0;

  /// One bulk read cycle: motor-space (if planned) → joint-space → sensors (if
  /// planned). Fills `bufs` for the channels that validated and reports them in
  /// `out` with the phase timestamps. Every per-request failure is already
  /// counted in comm_stats_mut() by the implementation.
  virtual void ReadCycle(const HandReadPlan& plan, HandReadBuffers& bufs,
                         HandReadOutcome& out) noexcept = 0;

  /// One individual-mode read cycle: pos → joint pos → vel → per-finger
  /// sensors. Default = unsupported: nothing attempted, all phase stamps set,
  /// every flag false (SupportsIndividualMode() is the configure-time gate).
  virtual void ReadCycleIndividual(const HandReadPlan& /*plan*/, HandReadBuffers& /*bufs*/,
                                   HandReadOutcome& out) noexcept {
    out = HandReadOutcome{};
    out.phase_end.fill(HandReadOutcome::Clock::now());
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
