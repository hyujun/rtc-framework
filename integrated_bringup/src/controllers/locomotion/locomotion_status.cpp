// See locomotion_status.hpp.
#include "integrated_bringup/controllers/locomotion/locomotion_status.hpp"

#include "rtc_mpc/reference/command_filter.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cstdio>

namespace integrated_bringup {

namespace {

using Key = LocomotionStatusKey;
using Mode = LocomotionDiagLogPod::Mode;
using Source = LocomotionDiagLogPod::ReferenceSource;
using Cause = LocomotionDiagLogPod::FaultCause;

constexpr const char* kKeyNames[kLocomotionStatusKeyCount] = {
    "source",
    "mode",
    "fault_cause",
    "estop",
    "pose_guard_mask",
    "tick_count",
    "plan_id",
    "policy_age_ms",
    "converged",
    "initial_solve",
    "contact_mask",
    "schedule_epoch",
    "state_epoch",
    "plausibility_rejects",
    "plan_holds",
    "solves",
    "failed_solves",
    "deadline_miss",
    "withheld",
    "overrun",
    "wbc_tick_us",
    "qp_iterations",
    "cmd_vx",
    "cmd_vy",
    "cmd_wz",
    "cmd_height",
    "cmd_valid",
    "cmd_fresh",
    "cmd_stale_ticks",
    "cmd_nonfinite_ticks",
    "cmd_ignored_axis_ticks",
};

}  // namespace

const char* LocomotionStatusKeyName(LocomotionStatusKey key) noexcept {
  const auto i = static_cast<std::size_t>(key);
  return i < kLocomotionStatusKeyCount ? kKeyNames[i] : "unknown";
}

const char* LocomotionSourceName(std::uint8_t ref_source) noexcept {
  switch (static_cast<Source>(ref_source)) {
    case Source::kIdle:
      return "idle";
    case Source::kStand:
      return "stand";
    case Source::kWalk:
      return "walk";
    case Source::kFault:
      return "fault";
  }
  return "unknown";
}

const char* LocomotionModeName(std::uint8_t mode) noexcept {
  switch (static_cast<Mode>(mode)) {
    case Mode::kWaiting:
      return "waiting";
    case Mode::kNominal:
      return "nominal";
    case Mode::kDegraded:
      return "degraded";
    case Mode::kFault:
      return "fault";
  }
  return "unknown";
}

const char* LocomotionFaultCauseName(std::uint8_t cause) noexcept {
  switch (static_cast<Cause>(cause)) {
    case Cause::kNone:
      return "none";
    case Cause::kSolveHealth:
      return "solve_health";
    case Cause::kSource:
      return "source";
    case Cause::kNonFinite:
      return "non_finite";
    case Cause::kModelEpoch:
      return "model_epoch";
    case Cause::kPlanLoss:
      return "plan_loss";
    case Cause::kPoseGuard:
      return "pose_guard";
  }
  return "unknown";
}

std::uint8_t LocomotionStatusLevel(const LocomotionDiagLogPod& d, bool mpc_enabled) noexcept {
  using Msg = diagnostic_msgs::msg::DiagnosticStatus;
  if (d.fault_latched || d.estop || static_cast<Mode>(d.mode) == Mode::kFault) {
    return Msg::ERROR;
  }
  if (static_cast<Mode>(d.mode) != Mode::kNominal) {
    return Msg::WARN;  // waiting for a state, or holding the last torque
  }
  if (mpc_enabled && static_cast<Source>(d.ref_source) == Source::kIdle) {
    return Msg::WARN;  // an MPC, but no usable plan: the latched pose
  }
  return Msg::OK;
}

void FormatLocomotionStatusValue(LocomotionStatusKey key, const LocomotionDiagLogPod& s,
                                 const rtc::mpc::FilteredCommand& cmd,
                                 const LocomotionMpcCounters& c, char* buf,
                                 std::size_t size) noexcept {
  if (buf == nullptr || size == 0) {
    return;
  }
  const auto str = [&](const char* v) { std::snprintf(buf, size, "%s", v); };
  const auto u64 = [&](std::uint64_t v) {
    std::snprintf(buf, size, "%llu", static_cast<unsigned long long>(v));
  };
  const auto flag = [&](bool v) { str(v ? "1" : "0"); };
  const auto real = [&](double v) { std::snprintf(buf, size, "%.6g", v); };
  switch (key) {
    case Key::kSource:
      return str(LocomotionSourceName(s.ref_source));
    case Key::kMode:
      return str(LocomotionModeName(s.mode));
    case Key::kFaultCause:
      return str(LocomotionFaultCauseName(s.fault_cause));
    case Key::kEstop:
      return flag(s.estop);
    case Key::kPoseGuardMask:
      return u64(s.pose_guard_mask);
    case Key::kTickCount:
      return u64(s.tick_count);
    case Key::kPlanId:
      return u64(s.plan_id);
    case Key::kPolicyAgeMs:
      return real(s.plan_id == 0 ? -1.0 : s.policy_age_us * 1e-3);
    case Key::kConverged:
      return flag(s.mpc_converged);
    case Key::kInitialSolve:
      return flag(s.mpc_initial_solve);
    case Key::kContactMask:
      return u64(s.contact_mask);
    case Key::kScheduleEpoch:
      return u64(s.schedule_epoch);
    case Key::kStateEpoch:
      return u64(s.mpc_state_epoch);
    case Key::kPlausibilityRejects:
      return u64(s.plausibility_rejects);
    case Key::kPlanHolds:
      return u64(s.plan_holds);
    case Key::kSolves:
      return u64(c.solves);
    case Key::kFailedSolves:
      return u64(c.failed);
    case Key::kDeadlineMiss:
      return u64(c.deadline_miss);
    case Key::kWithheld:
      return u64(c.withheld);
    case Key::kOverrun:
      return u64(c.overrun);
    case Key::kWbcTickUs:
      return real(s.wbc_tick_us);
    case Key::kQpIterations:
      return u64(static_cast<std::uint64_t>(s.qp_iterations < 0 ? 0 : s.qp_iterations));
    case Key::kCmdVx:
      return real(cmd.v_x);
    case Key::kCmdVy:
      return real(cmd.v_y);
    case Key::kCmdWz:
      return real(cmd.omega_z);
    case Key::kCmdHeight:
      return real(cmd.base_height);
    case Key::kCmdValid:
      return flag(cmd.valid);
    case Key::kCmdFresh:
      return flag(cmd.fresh);
    case Key::kCmdStaleTicks:
      return u64(cmd.stale_count);
    case Key::kCmdNonFiniteTicks:
      return u64(cmd.nonfinite_count);
    case Key::kCmdIgnoredAxisTicks:
      return u64(cmd.ignored_axis_count);
    case Key::kCount:
      break;
  }
  str("");
}

}  // namespace integrated_bringup
