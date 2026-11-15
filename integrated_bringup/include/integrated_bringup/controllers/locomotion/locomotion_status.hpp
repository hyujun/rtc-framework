// The locomotion_controller's status topic (diagnostic_msgs/DiagnosticStatus):
// the pure functions that turn the tick's diagnostic row, the command filter's
// output and the MPC thread's counters into the message's level and values.
// The ROS side (the publisher and the message) is in support/owned_topics.hpp.
//
// The RT tick stores its LocomotionDiagLogPod row (SeqLock); the command timer
// stores the filter's output (SeqLock); the controller's 1 Hz timer stores the
// MPC thread's counters (SeqLock — the publish thread never touches the
// thread). The publish thread loads the three and writes every value with
// snprintf into a fixed buffer (reals as %.6g) — no formatting on the RT side.
// The keys are fixed at configure (LocomotionStatusKeyName), in
// LocomotionStatusKey order, so a consumer can index them without this header.
// `level`: OK while the plan is followed (stand / walk) or the controller
// stands without an MPC; WARN while it waits for a state, holds a torque
// (degraded) or has an MPC but no usable plan (idle); ERROR under the fault
// latch or the E-STOP.
#ifndef INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_LOCOMOTION_STATUS_HPP_
#define INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_LOCOMOTION_STATUS_HPP_

#include "integrated_bringup/logging/locomotion_diag_log_pod.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rtc::mpc {
struct FilteredCommand;  // rtc_mpc/reference/command_filter.hpp
}  // namespace rtc::mpc

namespace integrated_bringup {

/// The MPC thread's counters as the 1 Hz timer last read them (0 without a thread).
struct LocomotionMpcCounters {
  std::uint64_t solves{0};
  std::uint64_t failed{0};
  std::uint64_t deadline_miss{0};
  std::uint64_t withheld{0};
  std::uint64_t overrun{0};
};

static_assert(std::is_trivially_copyable_v<LocomotionMpcCounters>,
              "LocomotionMpcCounters travels through a SeqLock");

/// The message's values, in order. kCount is the array size, not a key.
enum class LocomotionStatusKey : std::uint8_t {
  kSource = 0,  ///< idle / stand / walk / fault
  kMode,        ///< waiting / nominal / degraded / fault
  kFaultCause,
  kEstop,
  kTickCount,
  kPlanId,
  kPolicyAgeMs,
  kConverged,
  kInitialSolve,
  kContactMask,
  kScheduleEpoch,
  kStateEpoch,
  kPlausibilityRejects,
  kSolves,
  kFailedSolves,
  kDeadlineMiss,
  kWithheld,
  kOverrun,
  kWbcTickUs,
  kQpIterations,
  kCmdVx,
  kCmdVy,
  kCmdWz,
  kCmdHeight,
  kCmdValid,
  kCmdFresh,
  kCmdStaleTicks,        ///< filter ticks the input was stale
  kCmdNonFiniteTicks,    ///< filter ticks with a non-finite used axis (the axis kept its value)
  kCmdIgnoredAxisTicks,  ///< filter ticks with a non-zero angular.x / angular.y
  kCount,
};

inline constexpr std::size_t kLocomotionStatusKeyCount =
    static_cast<std::size_t>(LocomotionStatusKey::kCount);

/// The longest formatted value, with its terminator.
inline constexpr std::size_t kLocomotionStatusValueChars = 32;

/// Static key string (never allocates).
[[nodiscard]] const char* LocomotionStatusKeyName(LocomotionStatusKey key) noexcept;

/// Static names of the enums (never allocate).
[[nodiscard]] const char* LocomotionSourceName(std::uint8_t ref_source) noexcept;
[[nodiscard]] const char* LocomotionModeName(std::uint8_t mode) noexcept;
[[nodiscard]] const char* LocomotionFaultCauseName(std::uint8_t cause) noexcept;

/// diagnostic_msgs/DiagnosticStatus level (OK 0 / WARN 1 / ERROR 2) — the rule
/// in the file comment. `mpc_enabled`: this configuration runs an MPC.
[[nodiscard]] std::uint8_t LocomotionStatusLevel(const LocomotionDiagLogPod& d,
                                                 bool mpc_enabled) noexcept;

/// Write the value of `key` into `buf` (at least kLocomotionStatusValueChars),
/// NUL-terminated; a longer value is cut. `cmd` is the command filter's last
/// output (all zero without a command path). policy_age_ms is −1 while no plan
/// is followed (plan_id 0).
void FormatLocomotionStatusValue(LocomotionStatusKey key, const LocomotionDiagLogPod& d,
                                 const rtc::mpc::FilteredCommand& cmd,
                                 const LocomotionMpcCounters& counters, char* buf,
                                 std::size_t size) noexcept;

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_CONTROLLERS_LOCOMOTION_LOCOMOTION_STATUS_HPP_
