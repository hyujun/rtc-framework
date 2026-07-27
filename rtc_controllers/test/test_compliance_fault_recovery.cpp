// ── #260: external recovery path for a latched compliance fault ─────────────
//
// The three compliance controllers latch SAFE_STOP on a critical fault and never
// auto-recover (§10.6 "자동 복구 금지"). Until #260 the only way out was a process
// restart: ResetFault() existed but nothing outside the controller could reach
// it, because the CM only ever holds an RTControllerInterface&.
//
// What this suite pins is the part of that recovery the CM depends on, and it
// drives ALL THREE controllers through the same checks rather than one of them:
// the #277 slice found the same defect in five controllers after a table listed
// three, so a contract that is supposed to be uniform is asserted uniformly.
//
//   1. ResetFault() / HasLatchedFault() dispatch through the BASE interface.
//      A concrete-class-only ResetFault() compiles and passes every existing
//      per-controller test while being unreachable from the CM — which is
//      exactly the state #260 reports. The base reference here is the oracle.
//   2. HasLatchedFault() tracks the latch, so /rtc_cm/reset_fault can report
//      what actually happened instead of "request delivered".
//   3. E-8 separation, BOTH directions: ClearEstop() does not clear the fault
//      latch, and ResetFault() does not clear the E-STOP.
//   4. An UNCONSUMED reset request does not survive an activation boundary.
//      Compute() runs only while active, so a request left on an inactive
//      controller would be consumed on the first tick after activation and
//      unlatch a fault nobody re-authorised then — the laundering
//      BeginBiasCalibration() already refuses.
//
// The per-controller SAFE_STOP latch itself (fault → latch → persists) is
// already covered in each controller's own suite; this file re-latches only as
// the precondition it needs.
#include "rtc_controllers/direct/cascaded_compliance_controller.hpp"
#include "rtc_controllers/direct/task_impedance_controller.hpp"
#include "rtc_controllers/indirect/task_admittance_controller.hpp"
#include "test_urdf_path.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <vector>

namespace {

using rtc::CascadedComplianceController;
using rtc::TaskAdmittanceController;
using rtc::TaskImpedanceController;
using rtc::compliance::ComplianceState;

std::string Urdf6() {
  return rtc::test::TestUrdfPath("serial_6dof.urdf");
}

std::string Urdf7() {
  return rtc::test::TestUrdfPath("serial_7dof.urdf");
}

rtc::ControllerState MakeState(int nj, const std::vector<double>& q, double dt = 0.002) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = dt;
  state.devices[0].num_channels = nj;
  state.devices[0].valid = true;
  for (int i = 0; i < nj; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = q[static_cast<std::size_t>(i)];
    state.devices[0].velocities[static_cast<std::size_t>(i)] = 0.0;
  }
  return state;
}

// The four checks of the #260 contract, run against the BASE interface — the
// only handle the controller manager ever has. `tick` advances one Compute()
// WITHOUT re-applying whatever caused the latch, so a reset that is honoured is
// observable; `reactivate` is the CM's on_activate hook.
void ExpectResetFaultContract(rtc::RTControllerInterface& base, const std::function<void()>& tick,
                              const std::function<void()>& reactivate) {
  ASSERT_TRUE(base.HasLatchedFault())
      << "precondition: the controller was expected to be latched in SAFE_STOP";

  // (3a) A global E-STOP clear must not release the controller-local latch.
  // Run the E-STOP for real rather than clearing one that was never set: the
  // production path is TriggerEstop() → ticks that take the estop early return
  // (a different Compute() exit, with its own diagnostic store) → ClearEstop().
  // Clearing false over false would exercise none of that, and for the
  // admittance controller ClearEstop() additionally interacts with the hold the
  // SAFE_STOP path re-seeds every tick.
  base.TriggerEstop();
  tick();
  ASSERT_TRUE(base.HasLatchedFault())
      << "the E-STOP tick published a state that hides the SAFE_STOP latch";
  base.ClearEstop();
  tick();
  EXPECT_TRUE(base.HasLatchedFault()) << "ClearEstop() released a controller-local SAFE_STOP (E-8)";

  // (4) A reset that has not been consumed yet must not cross an activation
  // boundary. The request is delivered while the controller is NOT ticking,
  // then activation intervenes before any Compute() sees it.
  base.ResetFault();
  reactivate();
  tick();
  EXPECT_TRUE(base.HasLatchedFault())
      << "activation consumed a stale reset request and laundered the fault (E-8)";

  // (2) A reset delivered to a ticking controller clears the latch, and
  // HasLatchedFault() reports it — this is what makes the service response
  // honest rather than "delivered".
  base.ResetFault();
  tick();
  EXPECT_FALSE(base.HasLatchedFault()) << "ResetFault() through the base interface did not clear "
                                          "the latch";

  // (3b) The other direction: ResetFault() must not have cleared the E-STOP.
  // Checked last so the trigger cannot interfere with the recovery above.
  base.TriggerEstop();
  base.ResetFault();
  tick();
  EXPECT_TRUE(base.IsEstopped()) << "ResetFault() cleared the global E-STOP latch (E-8)";
}

// ── TaskImpedanceController ────────────────────────────────────────────────
// Latch recipe: a commanded step far past pose_error_limit. Subsequent ticks
// re-seed X_d to the measured pose, so the CAUSE clears while the LATCH stays —
// which is what lets `tick` be cause-free below.
TEST(ComplianceFaultRecovery, TaskImpedanceResetFaultReachableThroughBaseInterface) {
  const std::vector<double> q(6, 0.25);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  gains.pose_error_limit = 0.05;
  TaskImpedanceController ctrl(Urdf6(), gains, TaskImpedanceController::TaskSelection::kFullSe3);
  auto state = MakeState(6, q);

  const auto seed = ctrl.Compute(state);
  ASSERT_FALSE(ctrl.HasLatchedFault()) << "latched before any fault was applied";

  const std::array<double, 6> far{seed.actual_task_positions[0] + 0.5,
                                  seed.actual_task_positions[1],
                                  seed.actual_task_positions[2],
                                  0.0,
                                  0.0,
                                  0.0};
  ctrl.SetDeviceTarget(0, std::span<const double>(far.data(), far.size()));
  (void)ctrl.Compute(state);
  ASSERT_TRUE(ctrl.HasLatchedFault()) << "the pose-error step did not latch SAFE_STOP";
  ASSERT_EQ(ctrl.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kSafeStop))
      << "HasLatchedFault() disagrees with the diagnostic state it mirrors";

  ExpectResetFaultContract(
      ctrl, [&] { (void)ctrl.Compute(state); },
      [&] { ctrl.ResetTargetInitializationForTesting(); });
}

// ── CascadedComplianceController ───────────────────────────────────────────
TEST(ComplianceFaultRecovery, CascadedComplianceResetFaultReachableThroughBaseInterface) {
  const std::vector<double> q(6, 0.3);
  CascadedComplianceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  gains.max_torque_rate = 1e12;
  gains.joint_limit_kp = 0.0;
  gains.joint_limit_kd = 0.0;
  gains.joint_limit_margin = 0.0;
  gains.pose_error_limit = 0.05;
  CascadedComplianceController ctrl(Urdf6(), gains);
  auto state = MakeState(6, q);

  const auto seed = ctrl.Compute(state);
  ASSERT_FALSE(ctrl.HasLatchedFault()) << "latched before any fault was applied";

  const std::array<double, 6> far{seed.actual_task_positions[0] + 0.5,
                                  seed.actual_task_positions[1],
                                  seed.actual_task_positions[2],
                                  0.0,
                                  0.0,
                                  0.0};
  ctrl.SetDeviceTarget(0, std::span<const double>(far.data(), far.size()));
  (void)ctrl.Compute(state);
  ASSERT_TRUE(ctrl.HasLatchedFault()) << "the pose-error step did not latch SAFE_STOP";
  ASSERT_EQ(ctrl.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kSafeStop))
      << "HasLatchedFault() disagrees with the diagnostic state it mirrors";

  ExpectResetFaultContract(
      ctrl, [&] { (void)ctrl.Compute(state); },
      [&] { ctrl.ResetTargetInitializationForTesting(); });
}

// ── TaskAdmittanceController ───────────────────────────────────────────────
TEST(ComplianceFaultRecovery, TaskAdmittanceResetFaultReachableThroughBaseInterface) {
  const std::vector<double> q(7, 0.2);
  TaskAdmittanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  gains.pose_error_limit = 0.05;
  TaskAdmittanceController ctrl(Urdf7(), gains);
  auto state = MakeState(7, q);

  const auto seed = ctrl.Compute(state);
  ASSERT_FALSE(ctrl.HasLatchedFault()) << "latched before any fault was applied";

  const std::array<double, 6> far{seed.actual_task_positions[0] + 0.5,
                                  seed.actual_task_positions[1],
                                  seed.actual_task_positions[2],
                                  0.0,
                                  0.0,
                                  0.0};
  ctrl.SetDeviceTarget(0, std::span<const double>(far.data(), far.size()));
  (void)ctrl.Compute(state);
  ASSERT_TRUE(ctrl.HasLatchedFault()) << "the pose-error step did not latch SAFE_STOP";
  ASSERT_EQ(ctrl.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kSafeStop))
      << "HasLatchedFault() disagrees with the diagnostic state it mirrors";

  ExpectResetFaultContract(
      ctrl, [&] { (void)ctrl.Compute(state); },
      [&] { ctrl.ResetTargetInitializationForTesting(); });
}

// ── The default is "no latch", not "always recovered" ──────────────────────
// A controller with no fault latch of its own must read as never latched, so
// /rtc_cm/reset_fault reports a no-op for it instead of claiming a clear.
TEST(ComplianceFaultRecovery, BaseDefaultReportsNoLatchedFault) {
  const std::vector<double> q(6, 0.1);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  TaskImpedanceController ctrl(Urdf6(), gains, TaskImpedanceController::TaskSelection::kFullSe3);
  rtc::RTControllerInterface& base = ctrl;

  // Never ticked: the diagnostic snapshot has never been stored, and the answer
  // must be "not latched" rather than a stale or default-constructed SAFE_STOP.
  EXPECT_FALSE(base.HasLatchedFault()) << "an un-ticked controller reported a latched fault";

  auto state = MakeState(6, q);
  (void)ctrl.Compute(state);
  EXPECT_FALSE(base.HasLatchedFault()) << "a nominal tick reported a latched fault";

  // ResetFault() on an unlatched controller must not latch one. What it must
  // not do to the FSM state underneath (demote a running controller to HOLDING,
  // restart the DEGRADED dwell) HasLatchedFault() cannot see — that is pinned
  // where it is observable, at the state machine itself:
  // test_compliance_core.cpp `ResetFaultOnAnUnlatchedStateIsANoOp`.
  base.ResetFault();
  (void)ctrl.Compute(state);
  EXPECT_FALSE(base.HasLatchedFault());
}

}  // namespace
