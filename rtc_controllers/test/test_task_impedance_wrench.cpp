// ── TaskImpedanceController × external wrench (spec §3.2.1, §6.3, §10.6) ─────
// Slice-2 surface: the SetExternalWrench input contract, the conditioning chain
// as the controller drives it (frame transform, bias calibration, staleness
// fade), the §6.3 inertia-shaping law including the T4.1 continuity boundary,
// and the zero-allocation gate with the wrench path live.
//
// The model-free halves of these mechanisms (staleness arithmetic, the four
// conditioning stages, the state-machine lattice) are pinned in
// test_compliance_core.cpp; what needs a robot here is everything that touches
// the Jacobian, Λ_S or a frame placement.
//
// Zero-allocation is checked by GLOBAL operator-new interposition (same reason
// as test_task_impedance_controller.cpp: Compute() lives in another TU, so a
// same-TU Eigen guard would observe nothing).
#include "rtc_controllers/direct/task_impedance_controller.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "test_urdf_path.hpp"
#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/rt_model_handle.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <vector>

// That interposition is rtc::testing::ScopedAllocGate
// (rtc_controllers/testing/alloc_gate.hpp) — shared with every other suite that
// gates an RT path, rather than a per-file copy of the operator-new replacement.

namespace {

using rtc::TaskImpedanceController;
using rtc::compliance::ComplianceState;
using rtc::compliance::kWrenchDim;
using rtc::compliance::Wrench6;
using Sel = rtc::TaskImpedanceController::TaskSelection;

constexpr double kDt = 0.002;
constexpr double kRateHz = 1.0 / kDt;

std::string Urdf6() {
  return rtc::test::TestUrdfPath("serial_6dof.urdf");
}

std::string Urdf7() {
  return rtc::test::TestUrdfPath("serial_7dof.urdf");
}

rtc::ControllerState MakeState(int nj, const std::vector<double>& q,
                               const std::vector<double>& qd) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = kDt;
  state.devices[0].num_channels = nj;
  state.devices[0].valid = true;
  for (int i = 0; i < nj; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] =
        (i < static_cast<int>(q.size())) ? q[static_cast<std::size_t>(i)] : 0.0;
    state.devices[0].velocities[static_cast<std::size_t>(i)] =
        (i < static_cast<int>(qd.size())) ? qd[static_cast<std::size_t>(i)] : 0.0;
  }
  return state;
}

void Send(TaskImpedanceController& c, const Wrench6& w) {
  c.SetExternalWrench(std::span<const double, kWrenchDim>(w.data(), kWrenchDim));
}

// A wrench source with no deadband, no filter and no bias calibration, so tests
// that are about something ELSE (frames, §6.3, staleness) see the raw value
// unchanged. Each stage has its own test; mixing them in here would only make
// failures ambiguous.
std::string TransparentWrenchYaml(const std::string& extra = {}) {
  return R"(
external_wrench:
  enabled: true
  filter_enabled: false
  bias_calibration_samples: 0
  deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  max: [1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6]
)" + extra;
}

// ── Input contract ──────────────────────────────────────────────────────────

// A wrench arriving at a controller with no source configured must be dropped:
// the default is A=NONE, and a stray producer must not be able to inject force
// into a controller that was never told to listen.
TEST(TaskImpedanceWrench, DisabledByDefaultAndDropsInput) {
  TaskImpedanceController::Gains gains;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  EXPECT_FALSE(ctrl.external_wrench_enabled());
  EXPECT_EQ(ctrl.formulation(), TaskImpedanceController::Formulation::kJacobianTranspose);

  auto state = MakeState(6, std::vector<double>(6, 0.3), std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);
  Send(ctrl, Wrench6{100.0, 0, 0, 0, 0, 0});
  (void)ctrl.Compute(state);
  const auto diag = ctrl.GetDiagnosticsForTesting();
  EXPECT_FALSE(diag.wrench_valid);
  for (double v : diag.wrench_lwa)
    EXPECT_EQ(v, 0.0);
}

// ── Frame + sign (§3.2.1, compliance-conventions §2) ────────────────────────

// End-to-end Ad^{-T}: a pure FORCE applied at a sensor that sits away from the
// tip must show up in the LWA wrench with a moment (p_sensor − p_tip) × f. This
// is the transform that Ad^T instead of Ad^{-T} gets wrong, and it is wrong in a
// way no rotation-only test can see.
TEST(TaskImpedanceWrench, LeverArmProducesExpectedLwaMoment) {
  const std::vector<double> q(6, 0.3);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  // link_5 is proximal to the tip, so p_sensor − p_tip is genuinely nonzero.
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml("  sensor_frame: link_5\n")));
  ASSERT_TRUE(ctrl.external_wrench_enabled());

  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);  // seed

  const Wrench6 w{3.0, -2.0, 5.0, 0.0, 0.0, 0.0};  // pure force, sensor frame
  Send(ctrl, w);
  (void)ctrl.Compute(state);
  const auto diag = ctrl.GetDiagnosticsForTesting();
  ASSERT_TRUE(diag.wrench_valid);
  ASSERT_DOUBLE_EQ(diag.wrench_fade, 1.0);

  // Independent reference: same model, same frames, expanded block form
  //   f_A = R·f_S,   τ_A = (p_S − p_tip) × (R·f_S)
  rtc_urdf_bridge::ModelConfig cfg;
  cfg.urdf_path = Urdf6();
  cfg.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(cfg);
  rtc_urdf_bridge::RtModelHandle handle(builder.GetFullModel());
  handle.ComputeJacobians(std::span<const double>(q.data(), q.size()));
  const auto tip = static_cast<pinocchio::FrameIndex>(builder.GetFullModel()->nframes - 1);
  const auto sid = handle.GetFrameId("link_5");
  ASSERT_NE(sid, 0u);
  const pinocchio::SE3& s_pl = handle.GetFramePlacement(sid);
  const pinocchio::SE3& t_pl = handle.GetFramePlacement(tip);
  const Eigen::Vector3d lever = s_pl.translation() - t_pl.translation();
  ASSERT_GT(lever.norm(), 1e-3) << "fixture gives no lever arm — test would be vacuous";
  const Eigen::Vector3d f_s(w[0], w[1], w[2]);
  const Eigen::Vector3d f_a = s_pl.rotation() * f_s;
  const Eigen::Vector3d t_a = lever.cross(f_a);

  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(diag.wrench_lwa[static_cast<std::size_t>(i)], f_a(i), 1e-9) << "force axis " << i;
    EXPECT_NEAR(diag.wrench_lwa[static_cast<std::size_t>(i) + 3], t_a(i), 1e-9)
        << "moment axis " << i << " — Ad^T instead of Ad^{-T}?";
  }
  // Non-vacuity: the moment really is nonzero, so a dropped p×f term would fail.
  EXPECT_GT(t_a.norm(), 1e-3);
}

// A world-aligned sensor at the tip is the identity case: what the caller sends
// is what the law sees, with the documented sign ("+ = environment on robot").
TEST(TaskImpedanceWrench, DefaultSensorFrameIsTheTipBodyFrame) {
  const std::vector<double> q(6, 0.0);  // zero pose: tip rotation is identity here
  TaskImpedanceController::Gains gains;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));  // no sensor_frame ⇒ tip

  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);
  Send(ctrl, Wrench6{7.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  (void)ctrl.Compute(state);
  const auto diag = ctrl.GetDiagnosticsForTesting();
  ASSERT_TRUE(diag.wrench_valid);

  rtc_urdf_bridge::ModelConfig cfg;
  cfg.urdf_path = Urdf6();
  cfg.root_joint_type = "fixed";
  rtc_urdf_bridge::PinocchioModelBuilder builder(cfg);
  rtc_urdf_bridge::RtModelHandle handle(builder.GetFullModel());
  handle.ComputeJacobians(std::span<const double>(q.data(), q.size()));
  const auto tip = static_cast<pinocchio::FrameIndex>(builder.GetFullModel()->nframes - 1);
  const Eigen::Vector3d expect =
      handle.GetFramePlacement(tip).rotation() * Eigen::Vector3d(7, 0, 0);
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(diag.wrench_lwa[static_cast<std::size_t>(i)], expect(i), 1e-9);
  // Sensor AT the tip ⇒ zero lever arm ⇒ a pure force carries no moment.
  for (int i = 3; i < 6; ++i)
    EXPECT_NEAR(diag.wrench_lwa[static_cast<std::size_t>(i)], 0.0, 1e-9);
}

TEST(TaskImpedanceWrench, ConfigureRejectsUnknownSensorFrame) {
  TaskImpedanceController::Gains gains;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  EXPECT_THROW(ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml("  sensor_frame: no_such_link\n"))),
               std::runtime_error);
  // The rejected configure left the wrench path off, not half-enabled.
  EXPECT_FALSE(ctrl.external_wrench_enabled());
}

// ── Bias calibration (§3.2.1) ───────────────────────────────────────────────

// Activation runs BIAS_CALIBRATING for N samples, emits NOTHING while it does
// (an uncalibrated offset must not reach the control law), then commits the
// average and lets the ramp start.
TEST(TaskImpedanceWrench, BiasCalibrationSuppressesWrenchThenCommits) {
  const std::vector<double> q(6, 0.3);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(R"(
external_wrench:
  enabled: true
  filter_enabled: false
  bias_calibration_samples: 5
  deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
)"));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));

  const Wrench6 offset{2.0, -1.0, 0.5, 0.0, 0.0, 0.0};
  // A sample already sitting in the slot when the controller activates is
  // DISOWNED, not consumed (#236 D22 / §10.6): from inside there is no way to
  // tell a reading published a microsecond ago from the last one a producer
  // emitted before it died, so neither is trusted. Nothing is lost — the
  // calibration DEBT survives (`bias_calibrated` stays false) and the next
  // sample re-enters BIAS_CALIBRATING one tick later.
  Send(ctrl, offset);
  (void)ctrl.Compute(state);
  {
    const auto d = ctrl.GetDiagnosticsForTesting();
    EXPECT_FALSE(d.wrench_valid) << "the pre-activation sample was consumed, not disowned";
    EXPECT_FALSE(d.bias_calibrated) << "disowning the sample also dropped the owed calibration";
  }

  // The first sample published AFTER activation re-enters BIAS_CALIBRATING and
  // counts as the first of the five.
  Send(ctrl, offset);
  (void)ctrl.Compute(state);
  EXPECT_EQ(ctrl.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kBiasCalibrating));

  // Four more samples reach the required five; the fifth commits the bias. Every
  // one of these ticks must still emit zero.
  for (int k = 0; k < 5; ++k) {
    Send(ctrl, offset);
    (void)ctrl.Compute(state);
    const auto d = ctrl.GetDiagnosticsForTesting();
    for (double v : d.wrench_lwa)
      EXPECT_EQ(v, 0.0) << "uncalibrated wrench leaked into the law at sample " << k;
  }

  // Bias committed == the constant offset it saw at rest.
  const auto& bias = ctrl.GetWrenchBiasForTesting();
  for (std::size_t i = 0; i < kWrenchDim; ++i)
    EXPECT_NEAR(bias[i], offset[i], 1e-12) << "component " << i;
  EXPECT_NE(ctrl.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kBiasCalibrating));

  // Post-calibration, the SAME raw value now reads as zero external load.
  Send(ctrl, offset);
  (void)ctrl.Compute(state);
  for (double v : ctrl.GetDiagnosticsForTesting().wrench_lwa)
    EXPECT_NEAR(v, 0.0, 1e-9);

  // ...and a real change above the bias survives.
  Wrench6 pushed = offset;
  pushed[0] += 10.0;
  Send(ctrl, pushed);
  (void)ctrl.Compute(state);
  double norm = 0.0;
  for (double v : ctrl.GetDiagnosticsForTesting().wrench_lwa)
    norm += v * v;
  EXPECT_GT(std::sqrt(norm), 9.0);
}

// §3.2.1 asks for an N-SAMPLE average. The RT loop runs faster than any F/T
// source, so "N ticks" and "N samples" are different quantities: here the producer
// publishes once every three ticks, exactly the 3:1 ratio a 500 Hz loop has
// against a ~166 Hz sensor. A per-tick count would commit after four TICKS —
// having folded the first reading in three times — and the committed average
// makes the two hypotheses numerically distinct, so this cannot pass either way.
TEST(TaskImpedanceWrench, BiasAveragesNewSamplesNotTicks) {
  const std::vector<double> q(6, 0.3);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(R"(
external_wrench:
  enabled: true
  filter_enabled: false
  bias_calibration_samples: 4
  deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
)"));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);  // seed; no sample yet

  // Four readings, each held for three ticks. 3 × 0.002 s = 6 ms stays well inside
  // the 50 ms default timeout, so none of these is stale.
  constexpr int kTicksPerSample = 3;
  for (int s = 1; s <= 4; ++s) {
    Send(ctrl, Wrench6{static_cast<double>(s), 0.0, 0.0, 0.0, 0.0, 0.0});
    for (int t = 0; t < kTicksPerSample; ++t)
      (void)ctrl.Compute(state);
  }

  EXPECT_TRUE(ctrl.GetDiagnosticsForTesting().bias_calibrated);
  // Per SAMPLE: (1+2+3+4)/4 = 2.5. Per TICK it would have committed at tick 4 on
  // (1+1+1+2)/4 = 1.25 — the value this assertion rules out.
  EXPECT_NEAR(ctrl.GetWrenchBiasForTesting()[0], 2.5, 1e-12)
      << "bias averaged control ticks instead of sensor readings";
}

// Cold start: the controller activates BEFORE the F/T driver publishes anything.
// The FSM gate has to be released (otherwise the controller sits in
// BIAS_CALIBRATING forever, masking every later wrench fault), but the §3.2.1
// calibration is still owed and must run when data finally appears — releasing
// the gate must not discard the work. `bias_calibrated` is what makes the
// difference observable at all: without it, "never calibrated" and "calibrated"
// look identical from outside.
TEST(TaskImpedanceWrench, BiasCalibrationRunsWhenTheProducerStartsLate) {
  const std::vector<double> q(6, 0.3);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(R"(
external_wrench:
  enabled: true
  filter_enabled: false
  bias_calibration_samples: 4
  deadband: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
)"));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));

  // Ten ticks with no producer at all. The FSM must not be stuck, and the bias
  // must not be declared done.
  for (int k = 0; k < 10; ++k) {
    (void)ctrl.Compute(state);
    const auto d = ctrl.GetDiagnosticsForTesting();
    EXPECT_FALSE(d.wrench_valid) << "tick " << k;
    EXPECT_FALSE(d.bias_calibrated) << "a calibration that never ran reported itself done";
    EXPECT_NE(d.state, static_cast<std::uint8_t>(ComplianceState::kBiasCalibrating))
        << "stranded in BIAS_CALIBRATING with no producer (tick " << k << ")";
  }

  // The driver comes up. The owed calibration runs now: the wrench stays
  // suppressed while the four samples average, and the offset must NOT leak.
  const Wrench6 offset{2.0, -1.0, 0.5, 0.0, 0.0, 0.0};
  for (int k = 0; k < 4; ++k) {
    Send(ctrl, offset);
    (void)ctrl.Compute(state);
    const auto d = ctrl.GetDiagnosticsForTesting();
    for (double v : d.wrench_lwa)
      EXPECT_EQ(v, 0.0) << "uncalibrated wrench leaked into the law at sample " << k;
  }

  const auto committed = ctrl.GetDiagnosticsForTesting();
  EXPECT_TRUE(committed.bias_calibrated) << "four samples arrived but no bias was committed";
  const auto& bias = ctrl.GetWrenchBiasForTesting();
  for (std::size_t i = 0; i < kWrenchDim; ++i)
    EXPECT_NEAR(bias[i], offset[i], 1e-12) << "component " << i;

  // Post-calibration the same raw value reads as zero external load.
  Send(ctrl, offset);
  (void)ctrl.Compute(state);
  for (double v : ctrl.GetDiagnosticsForTesting().wrench_lwa)
    EXPECT_NEAR(v, 0.0, 1e-9);
}

// ── Staleness (§10.6) ───────────────────────────────────────────────────────

// The MUST of §10.6: a dead source fades the wrench to ZERO and degrades — it
// does NOT hold the last value, and it does NOT stop the robot. Both halves are
// asserted, because each failure mode is a different real bug (a held value
// pushes the arm forever; a SAFE_STOP stops it on every estimator hiccup).
TEST(TaskImpedanceWrench, StaleWrenchFadesToZeroAndDegradesWithoutSafeStop) {
  const std::vector<double> q(6, 0.3);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  gains.wrench_timeout = 0.01;       // 5 ticks
  gains.wrench_fadeout_time = 0.02;  // 10 further ticks
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);  // seed (bias_calibration_samples: 0 ⇒ committed)

  const Wrench6 push{20.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Send(ctrl, push);
  (void)ctrl.Compute(state);
  const auto fresh = ctrl.GetDiagnosticsForTesting();
  ASSERT_TRUE(fresh.wrench_valid);
  ASSERT_FALSE(fresh.wrench_stale);
  const double live =
      std::abs(fresh.wrench_lwa[0]) + std::abs(fresh.wrench_lwa[1]) + std::abs(fresh.wrench_lwa[2]);
  ASSERT_GT(live, 1.0) << "wrench never reached the law — the fade assertion would be vacuous";

  // Producer dies. Walk past the timeout and the fade-out ramp.
  double prev = live;
  bool saw_partial_fade = false;
  for (int k = 0; k < 40; ++k) {
    (void)ctrl.Compute(state);
    const auto d = ctrl.GetDiagnosticsForTesting();
    const double mag =
        std::abs(d.wrench_lwa[0]) + std::abs(d.wrench_lwa[1]) + std::abs(d.wrench_lwa[2]);
    EXPECT_LE(mag, prev + 1e-12) << "wrench grew while stale, tick " << k;
    if (mag > 1e-9 && mag < live - 1e-9)
      saw_partial_fade = true;
    prev = mag;
    EXPECT_NE(d.state, static_cast<std::uint8_t>(ComplianceState::kSafeStop))
        << "a dead wrench source stopped the robot (tick " << k << ")";
  }
  EXPECT_TRUE(saw_partial_fade) << "wrench stepped to zero instead of ramping (torque jump)";

  const auto end = ctrl.GetDiagnosticsForTesting();
  EXPECT_TRUE(end.wrench_stale);
  for (double v : end.wrench_lwa)
    EXPECT_EQ(v, 0.0) << "expired wrench held instead of faded to zero (§10.6 MUST)";
  EXPECT_EQ(end.state, static_cast<std::uint8_t>(ComplianceState::kDegraded));
  EXPECT_TRUE(end.control_valid) << "DEGRADED must keep controlling, not hand off to the hold";

  // Source returns → recovers on its own, no ResetFault().
  for (int k = 0; k < 400; ++k) {
    Send(ctrl, push);
    (void)ctrl.Compute(state);
  }
  const auto recovered = ctrl.GetDiagnosticsForTesting();
  EXPECT_FALSE(recovered.wrench_stale);
  EXPECT_TRUE(rtc::compliance::IsRunning(static_cast<ComplianceState>(recovered.state)));
}

// #236 D22 — the defect the WrenchPipeline migration fixes. The controller's own
// copy of the wrench path RE-DATED the sample in the slot on (re)activation
// (`ResetTiming`), so the last reading of a producer that died BEFORE the
// controller was deactivated came back as a FRESH full-magnitude wrench on the
// tick after re-activation: §10.6 says an expired wrench goes to zero and is
// never held, and re-activation was the one path that resurrected it. The shared
// pipeline DISOWNS the sample instead (`Invalidate`).
//
// The fade above cannot catch this: it only walks the age forward, and the bug
// lives in the reset that puts the age back.
TEST(TaskImpedanceWrench, ReactivationDoesNotReviveADeadProducersLastReading) {
  const std::vector<double> q(6, 0.3);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  gains.wrench_timeout = 0.01;       // 5 ticks
  gains.wrench_fadeout_time = 0.02;  // 10 further ticks
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);  // seed (bias_calibration_samples: 0 ⇒ committed)

  const Wrench6 push{20.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Send(ctrl, push);
  (void)ctrl.Compute(state);
  ASSERT_GT(std::abs(ctrl.GetDiagnosticsForTesting().wrench_lwa[0]), 1.0)
      << "wrench never reached the law — the revival assertion would be vacuous";

  // Producer dies; walk past timeout + fade-out so the wrench is fully expired.
  for (int k = 0; k < 40; ++k)
    (void)ctrl.Compute(state);
  ASSERT_TRUE(ctrl.GetDiagnosticsForTesting().wrench_stale);

  // Re-activation (what on_activate does in the CM). NOTHING is published after
  // this point — the producer is still dead.
  ctrl.ResetTargetInitializationForTesting();
  for (int k = 0; k < 5; ++k) {
    (void)ctrl.Compute(state);
    const auto d = ctrl.GetDiagnosticsForTesting();
    EXPECT_FALSE(d.wrench_valid) << "a dead producer's last reading was re-dated as fresh at tick "
                                 << k;
    for (double v : d.wrench_lwa)
      EXPECT_EQ(v, 0.0) << "resurrected wrench entered the law at tick " << k;
  }

  // Not a dead end: a genuinely new sample is accepted again (the disown covers
  // the generation present at reset, not the input for good).
  Send(ctrl, push);
  (void)ctrl.Compute(state);  // bias_samples == 0 ⇒ commits, emits zero
  Send(ctrl, push);
  (void)ctrl.Compute(state);
  const auto back = ctrl.GetDiagnosticsForTesting();
  EXPECT_TRUE(back.wrench_valid);
  EXPECT_GT(std::abs(back.wrench_lwa[0]), 1.0) << "the input never recovered after the disown";
}

// F3 (PR #256) reaches the impedance controller with the pipeline: a producer
// sending garbage is otherwise INVISIBLE — the finiteness gate drops the sample,
// so nothing arrives and the last good reading just keeps ageing, which reads
// exactly like a producer that stopped. The count is what separates the two.
TEST(TaskImpedanceWrench, RejectedSampleCountSurfacesInDiagnostics) {
  const std::vector<double> q(6, 0.3);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);

  Send(ctrl, Wrench6{5.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  (void)ctrl.Compute(state);
  const auto good = ctrl.GetDiagnosticsForTesting();
  ASSERT_EQ(good.wrench_rejected, 0u);
  ASSERT_GT(
      std::abs(good.wrench_lwa[0]) + std::abs(good.wrench_lwa[1]) + std::abs(good.wrench_lwa[2]),
      1.0)
      << "no wrench in the law — the retention assertion below would be vacuous";

  const double nan_v = std::numeric_limits<double>::quiet_NaN();
  Send(ctrl, Wrench6{nan_v, 0.0, 0.0, 0.0, 0.0, 0.0});
  Send(ctrl, Wrench6{0.0, std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0, 0.0});
  (void)ctrl.Compute(state);
  const auto d = ctrl.GetDiagnosticsForTesting();
  EXPECT_EQ(d.wrench_rejected, 2u) << "non-finite samples were counted wrong (or let through)";
  // Still inside wrench_timeout, so the retained reading is unfaded: the garbage
  // was dropped at the input, not conditioned into the law.
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_TRUE(std::isfinite(d.wrench_lwa[i])) << "non-finite wrench reached the control law";
    EXPECT_NEAR(d.wrench_lwa[i], good.wrench_lwa[i], 1e-12)
        << "a rejected sample disturbed the last good reading, component " << i;
  }
}

// Contact detection rides the conditioned LWA force with hysteresis, and drives
// the RUNNING_FREE_SPACE ⇄ RUNNING_CONTACT split.
TEST(TaskImpedanceWrench, ContactStateFollowsForceWithHysteresis) {
  const std::vector<double> q(6, 0.3);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  gains.contact_threshold = 10.0;
  gains.contact_release_ratio = 0.5;  // release below 5 N
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml()));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);

  auto tick = [&](double fx) {
    Send(ctrl, Wrench6{fx, 0, 0, 0, 0, 0});
    (void)ctrl.Compute(state);
    return ctrl.GetDiagnosticsForTesting();
  };

  EXPECT_FALSE(tick(2.0).in_contact);
  EXPECT_FALSE(tick(9.0).in_contact);
  EXPECT_TRUE(tick(15.0).in_contact);
  EXPECT_EQ(ctrl.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kRunningContact));
  // Between release and enter the latch holds — no chatter.
  EXPECT_TRUE(tick(7.0).in_contact);
  EXPECT_FALSE(tick(2.0).in_contact);
  EXPECT_EQ(ctrl.GetDiagnosticsForTesting().state,
            static_cast<std::uint8_t>(ComplianceState::kRunning));
}

// A zero contact_threshold is not "a hysteresis with a very small band" — it is
// enter == exit == 0, the bare comparator §10.6 forbids, and every real reading
// exceeds 0 N so it would latch contact on sensor noise and never release. The
// release-ratio clamp cannot rescue it because the ratio SCALES the threshold.
// Contact detection is therefore off at a non-positive threshold.
TEST(TaskImpedanceWrench, NonPositiveContactThresholdDisablesContactDetection) {
  const std::vector<double> q(6, 0.3);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml("  contact_threshold: 0.0\n")));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  (void)ctrl.Compute(state);

  // A tiny reading (noise-scale) and a large one must BOTH read as no contact,
  // and the FSM must stay on the free-space side of the split.
  for (const double fx : {1e-9, 50.0}) {
    Send(ctrl, Wrench6{fx, 0, 0, 0, 0, 0});
    (void)ctrl.Compute(state);
    const auto d = ctrl.GetDiagnosticsForTesting();
    EXPECT_FALSE(d.in_contact) << "contact latched at a zero threshold, fx = " << fx;
    EXPECT_NE(d.state, static_cast<std::uint8_t>(ComplianceState::kRunningContact));
  }
}

// ── §6.3 inertia shaping ────────────────────────────────────────────────────

// Two controllers, identical in everything but the formulation, driven with the
// identical state and target. `desired_inertia` omitted ⇒ Λ_d := Λ_S ⇒ B = I,
// which is not short-circuited: the identity comes out of the real Λ_d
// factorisation and solve, so this exercises the §6.3 path rather than an `if`.
void ExpectT41Equivalence(const std::string& urdf, int nj, Sel sel,
                          const TaskImpedanceController::Gains& gains) {
  const std::vector<double> q(static_cast<std::size_t>(nj), 0.3);
  TaskImpedanceController a(urdf, gains, sel);
  TaskImpedanceController b(urdf, gains, sel);
  a.SetControlRate(kRateHz);
  b.SetControlRate(kRateHz);
  a.LoadConfig(YAML::Load("formulation: jacobian_transpose"));
  b.LoadConfig(YAML::Load("formulation: inertia_shaping"));
  ASSERT_EQ(b.formulation(), TaskImpedanceController::Formulation::kInertiaShaping);

  auto state = MakeState(nj, q, std::vector<double>(static_cast<std::size_t>(nj), 0.05));
  const auto seed_a = a.Compute(state);
  const auto seed_b = b.Compute(state);

  // A nonzero task error, or f_task == 0 makes the comparison vacuous.
  const std::array<double, 6> target{seed_a.actual_task_positions[0] + 0.03,
                                     seed_a.actual_task_positions[1] - 0.02,
                                     seed_a.actual_task_positions[2] + 0.01,
                                     0.05,
                                     -0.03,
                                     0.02};
  a.SetDeviceTarget(0, std::span<const double>(target.data(), target.size()));
  b.SetDeviceTarget(0, std::span<const double>(target.data(), target.size()));
  const auto out_a = a.Compute(state);
  const auto out_b = b.Compute(state);
  (void)seed_b;

  const auto diag_b = b.GetDiagnosticsForTesting();
  ASSERT_TRUE(diag_b.control_valid) << "inertia-shaping path degenerated at this pose";
  ASSERT_FALSE(diag_b.inertia_clamped) << "the max_inertia_ratio clamp fired at Λ_d = Λ_S";
  ASSERT_FALSE(diag_b.inertia_solve_failed)
      << "Λ_d = Λ_S failed to factor — the equivalence would hold trivially via the "
         "B = I degrade path instead of through the §6.3 solve";
  ASSERT_GT(diag_b.sigma_min, gains.singularity_threshold)
      << "pose too close to singular for a meaningful comparison";

  double spread = 0.0;
  for (int i = 0; i < nj; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_NEAR(out_b.devices[0].commands[ui], out_a.devices[0].commands[ui], 1e-9)
        << "§6.3 with Λ_d = Λ_S diverged from §6.2 at joint " << i;
    spread += std::abs(out_a.devices[0].commands[ui]);
  }
  EXPECT_GT(spread, 1e-3) << "both outputs were ~zero — T4.1 would pass vacuously";
}

// Nullspace INACTIVE (both posture gains zero): §6.2 forms no M at all, while
// §6.3 must form Λ_S regardless. This is the widened-gate case — the branch
// PR #241's F2 narrowed and slice 2 re-opens for inertia shaping only — and it
// carries T4.1 at the same time.
//
// serial_6dof is unusable for this: all six of its joints turn about +Z with
// origins on the Z axis, so its Jacobian is rank-1 and Λ_S is singular at EVERY
// pose. §6.5 says that must latch SAFE_STOP, and it does (asserted separately
// below) — so the equivalence has to be measured on a full-rank arm.
TEST(TaskImpedanceWrench, T41NaturalInertiaMatchesJacobianTransposeWithoutNullspace) {
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  gains.nullspace_kp = 0.0;
  gains.nullspace_kd = 0.0;  // ⇒ nullspace_active == false even though nv > m
  ExpectT41Equivalence(Urdf7(), 7, Sel::kFullSe3, gains);
}

// Redundant (nv > m): both sides run the nullspace projector, so this pins that
// shaping does not perturb τ_null either.
TEST(TaskImpedanceWrench, T41NaturalInertiaMatchesJacobianTransposeRedundant) {
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  gains.nullspace_kp = 30.0;
  gains.nullspace_kd = 12.0;
  ExpectT41Equivalence(Urdf7(), 7, Sel::kFullSe3, gains);
}

// With Λ_d ≠ Λ_S the external wrench term (B − I)(−f_ext) becomes live, so the
// SAME wrench that is inert under §6.2 now changes the torque. That difference
// is the whole point of A≠NONE — if it were zero the formulation would be
// decorative.
TEST(TaskImpedanceWrench, ShapedInertiaMakesTheWrenchTermBite) {
  const std::vector<double> q(7, 0.3);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  TaskImpedanceController shaped(Urdf7(), gains, Sel::kFullSe3);
  TaskImpedanceController neutral(Urdf7(), gains, Sel::kFullSe3);
  shaped.SetControlRate(kRateHz);
  neutral.SetControlRate(kRateHz);
  shaped.LoadConfig(YAML::Load(TransparentWrenchYaml(
      "formulation: inertia_shaping\ndesired_inertia: [0.5, 0.5, 0.5, 0.05, 0.05, 0.05]\n"
      "max_inertia_ratio: 100.0\n")));
  neutral.LoadConfig(YAML::Load(TransparentWrenchYaml("formulation: inertia_shaping\n")));

  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  (void)shaped.Compute(state);
  (void)neutral.Compute(state);

  const Wrench6 push{25.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Send(shaped, push);
  Send(neutral, push);
  const auto out_shaped = shaped.Compute(state);
  const auto out_neutral = neutral.Compute(state);

  ASSERT_TRUE(shaped.GetDiagnosticsForTesting().control_valid);
  ASSERT_TRUE(neutral.GetDiagnosticsForTesting().control_valid);
  double diff = 0.0;
  for (int i = 0; i < 7; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    diff += std::abs(out_shaped.devices[0].commands[ui] - out_neutral.devices[0].commands[ui]);
  }
  EXPECT_GT(diff, 1e-3) << "f_ext had no effect under Λ_d ≠ Λ_S — the §6.3 term is dead";

  // Λ_d = Λ_S (neutral) is the case where f_ext CANNOT matter: (B − I) == 0.
  TaskImpedanceController quiet(Urdf7(), gains, Sel::kFullSe3);
  quiet.SetControlRate(kRateHz);
  quiet.LoadConfig(YAML::Load(TransparentWrenchYaml("formulation: inertia_shaping\n")));
  (void)quiet.Compute(state);
  const auto no_wrench = quiet.Compute(state);
  for (int i = 0; i < 7; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_NEAR(out_neutral.devices[0].commands[ui], no_wrench.devices[0].commands[ui], 1e-9)
        << "a wrench moved the neutral (B = I) law, where (B − I)(−f_ext) must be 0";
  }
}

// The DIRECTION of the §6.3 wrench term, which ShapedInertiaMakesTheWrenchTermBite
// above deliberately does not constrain (it only asks that the term be alive).
//
// Λ_d → ∞ is the limit where the answer is fixed by physics alone and needs no
// convention argument: an infinitely heavy desired inertia must make the arm
// IMMOBILE under an external push, so the commanded task force has to cancel the
// wrench exactly — f_cmd → −f_ext as B = Λ_S Λ_d⁻¹ → 0. (Getting the sign wrong
// gives f_cmd → +f_ext, i.e. an arm that yields TWICE as fast as the unshaped one
// precisely when it was configured to be immovable.) The pose error is held at
// zero so f_task drops out and the bracket reduces to the wrench term alone.
//
// The oracle for Jᵀf_ext avoids re-deriving the Jacobian in the test: a
// jacobian_transpose controller with K_p = I on translation, all other gains 0
// and a target displaced by exactly f_ext_LWA produces f_task ≡ f_ext_LWA, so its
// own torque delta IS Jᵀf_ext. Both controllers share the model, the pose and the
// gravity term, which therefore cancels out of both deltas.
TEST(TaskImpedanceWrench, HeavyDesiredInertiaCancelsTheExternalWrench) {
  const std::vector<double> q(7, 0.3);
  const std::vector<double> qd(7, 0.0);

  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  gains.nullspace_kp = 0.0;
  gains.nullspace_kd = 0.0;        // no τ_null on either side
  gains.kp_pos = {1.0, 1.0, 1.0};  // unit stiffness ⇒ f_task == e_trans exactly
  gains.kd_pos = {0.0, 0.0, 0.0};
  gains.kp_rot = {0.0, 0.0, 0.0};  // the oracle's target rotation must not bite
  gains.kd_rot = {0.0, 0.0, 0.0};
  gains.pose_error_limit = 1e9;  // the oracle's target is metres away by design
  gains.max_torque_rate = 1e12;  // measure the law, not the slew limiter

  auto state = MakeState(7, q, qd);

  // ── Shaped side: B → 0, pose error 0 ⇒ τ − ĝ == Jᵀ(B − I)f_ext ≈ −Jᵀf_ext ──
  TaskImpedanceController shaped(Urdf7(), gains, Sel::kFullSe3);
  shaped.SetControlRate(kRateHz);
  shaped.LoadConfig(YAML::Load(
      TransparentWrenchYaml("formulation: inertia_shaping\n"
                            "desired_inertia: [1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6]\n")));
  // Seed tick: X_d = X_meas (⇒ e ≡ 0 from here on) and no sample has arrived yet,
  // so this tick's torque is exactly ĝ — the wrench-free baseline.
  const auto base_shaped = shaped.Compute(state);

  const Wrench6 push{2.0, -1.5, 1.0, 0.0, 0.0, 0.0};  // pure force, sensor = tip
  Send(shaped, push);
  (void)shaped.Compute(state);  // the tick that consumes the FIRST sample
  Send(shaped, push);
  const auto out_shaped = shaped.Compute(state);  // wrench live

  const auto diag_shaped = shaped.GetDiagnosticsForTesting();
  ASSERT_TRUE(diag_shaped.control_valid);
  ASSERT_TRUE(diag_shaped.wrench_valid);
  ASSERT_DOUBLE_EQ(diag_shaped.wrench_fade, 1.0);
  ASSERT_FALSE(diag_shaped.inertia_clamped)
      << "‖B − I‖∞ ≈ 1 at B ≈ 0, so the default ratio 3.0 must not clamp";
  // Sensor frame == tip ⇒ the Ad^{-T} lever arm is zero, so a pure force stays a
  // pure force and the unit-K_p oracle below can reproduce it exactly.
  for (std::size_t i = 3; i < 6; ++i)
    ASSERT_NEAR(diag_shaped.wrench_lwa[i], 0.0, 1e-12);

  // ── Oracle side: f_task := f_ext_LWA ⇒ τ − ĝ == Jᵀf_ext ────────────────────
  TaskImpedanceController oracle(Urdf7(), gains, Sel::kFullSe3);
  oracle.SetControlRate(kRateHz);
  oracle.LoadConfig(YAML::Load("formulation: jacobian_transpose"));
  const auto base_oracle = oracle.Compute(state);  // seed; e ≡ 0 ⇒ τ == ĝ

  const std::array<double, 6> target{
      base_oracle.actual_task_positions[0] + diag_shaped.wrench_lwa[0],
      base_oracle.actual_task_positions[1] + diag_shaped.wrench_lwa[1],
      base_oracle.actual_task_positions[2] + diag_shaped.wrench_lwa[2],
      0.0,
      0.0,
      0.0};
  oracle.SetDeviceTarget(0, std::span<const double>(target.data(), target.size()));
  const auto out_oracle = oracle.Compute(state);
  ASSERT_TRUE(oracle.GetDiagnosticsForTesting().control_valid);

  double scale = 0.0;
  for (int i = 0; i < 7; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    scale = std::max(
        scale, std::abs(out_oracle.devices[0].commands[ui] - base_oracle.devices[0].commands[ui]));
  }
  ASSERT_GT(scale, 1e-3) << "Jᵀf_ext is ~zero at this pose — the comparison is vacuous";

  for (int i = 0; i < 7; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double shaped_delta =
        out_shaped.devices[0].commands[ui] - base_shaped.devices[0].commands[ui];
    const double oracle_delta =
        out_oracle.devices[0].commands[ui] - base_oracle.devices[0].commands[ui];
    EXPECT_NEAR(shaped_delta, -oracle_delta, 1e-3 * scale)
        << "joint " << i << ": Λ_d → ∞ must CANCEL the external wrench (f_cmd → −f_ext), "
        << "but the commanded torque moved with it — the (B − I)f_ext sign is inverted";
  }
}

// §5.2 MUST: an aggressively light Λ_d must be clamped, and the clamp must be
// reported rather than applied silently.
TEST(TaskImpedanceWrench, MaxInertiaRatioClampsAndReports) {
  const std::vector<double> q(7, 0.3);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  TaskImpedanceController ctrl(Urdf7(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(TransparentWrenchYaml(
      "formulation: inertia_shaping\ndesired_inertia: [1.0e-4, 1.0e-4, 1.0e-4, 1.0e-5, 1.0e-5, "
      "1.0e-5]\nmax_inertia_ratio: 2.0\n")));
  auto state = MakeState(7, q, std::vector<double>(7, 0.0));
  const auto seed = ctrl.Compute(state);
  const std::array<double, 6> target{seed.actual_task_positions[0] + 0.02,
                                     seed.actual_task_positions[1],
                                     seed.actual_task_positions[2],
                                     0.0,
                                     0.0,
                                     0.0};
  ctrl.SetDeviceTarget(0, std::span<const double>(target.data(), target.size()));
  Send(ctrl, Wrench6{30.0, 0, 0, 0, 0, 0});
  const auto out = ctrl.Compute(state);
  const auto diag = ctrl.GetDiagnosticsForTesting();
  EXPECT_TRUE(diag.inertia_clamped) << "Λ_d ≈ 0 sailed past max_inertia_ratio";
  EXPECT_FALSE(diag.inertia_solve_failed)
      << "this must be the RATIO clamp, not a Λ_d factorisation failure — the two "
         "shared one flag once, which let this assertion pass for the wrong reason";
  EXPECT_TRUE(diag.control_valid) << "the clamp must keep the law running, not degrade it";
  for (int i = 0; i < 7; ++i)
    EXPECT_TRUE(std::isfinite(out.devices[0].commands[static_cast<std::size_t>(i)]));

  // Non-vacuity: at the SAME configuration a ratio wide enough for this Λ_d does
  // not clamp, so the assertion above is about the bound and not about the pose.
  TaskImpedanceController wide(Urdf7(), gains, Sel::kFullSe3);
  wide.SetControlRate(kRateHz);
  wide.LoadConfig(YAML::Load(TransparentWrenchYaml("formulation: inertia_shaping\n")));
  (void)wide.Compute(state);
  (void)wide.Compute(state);
  EXPECT_FALSE(wide.GetDiagnosticsForTesting().inertia_clamped);
}

// The widened gate, stated as behaviour: using Λ_S re-exposes §6.5 singularity
// risk, so on a rank-deficient arm §6.3 MUST latch SAFE_STOP while §6.2 — which
// never forms Λ at all — keeps controlling. serial_6dof is exactly that arm (six
// coaxial +Z joints ⇒ σ_min ≡ 0), which makes this the sharpest available
// statement that the F2 narrowing was WIDENED rather than reverted: the same
// controller, same fixture, differs only by formulation.
TEST(TaskImpedanceWrench, InertiaShapingReArmsSingularityFaultThatJacobianTransposeIsImmuneTo) {
  const std::vector<double> q(6, 0.3);
  TaskImpedanceController::Gains gains;
  gains.activation_ramp_time = 0.0;
  gains.nullspace_kp = 0.0;
  gains.nullspace_kd = 0.0;

  TaskImpedanceController jt(Urdf6(), gains, Sel::kFullSe3);
  jt.SetControlRate(kRateHz);
  jt.LoadConfig(YAML::Load("formulation: jacobian_transpose"));
  auto state = MakeState(6, q, std::vector<double>(6, 0.0));
  for (int k = 0; k < 3; ++k)
    (void)jt.Compute(state);
  const auto d_jt = jt.GetDiagnosticsForTesting();
  EXPECT_TRUE(d_jt.control_valid) << "§6.2 must stay immune to task singularities (F2)";
  EXPECT_NE(d_jt.state, static_cast<std::uint8_t>(ComplianceState::kSafeStop));

  TaskImpedanceController shaped(Urdf6(), gains, Sel::kFullSe3);
  shaped.SetControlRate(kRateHz);
  shaped.LoadConfig(YAML::Load("formulation: inertia_shaping"));
  for (int k = 0; k < 3; ++k)
    (void)shaped.Compute(state);
  const auto d_sh = shaped.GetDiagnosticsForTesting();
  EXPECT_EQ(d_sh.sigma_min, 0.0) << "fixture is not singular — test would be vacuous";
  EXPECT_EQ(d_sh.state, static_cast<std::uint8_t>(ComplianceState::kSafeStop))
      << "§6.3 used Λ_S at a singular pose without re-arming the σ_min fault (§6.5)";
}

TEST(TaskImpedanceWrench, ConfigureRejectsNonPositiveDesiredInertia) {
  TaskImpedanceController::Gains gains;
  TaskImpedanceController ctrl(Urdf6(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  EXPECT_THROW(
      ctrl.LoadConfig(YAML::Load(
          "formulation: inertia_shaping\ndesired_inertia: [1.0, 0.0, 1.0, 1.0, 1.0, 1.0]")),
      std::runtime_error);
  EXPECT_THROW(ctrl.LoadConfig(YAML::Load("formulation: inertia_shaping\ndesired_inertia: [1.0]")),
               std::runtime_error);
  EXPECT_THROW(ctrl.LoadConfig(YAML::Load("formulation: nonsense")), std::runtime_error);
}

// ── Zero allocation with the wrench path live (T7.1) ────────────────────────

TEST(TaskImpedanceWrench, ComputeIsAllocationFreeWithWrenchAndShaping) {
  const std::vector<double> q(7, 0.15);
  TaskImpedanceController::Gains gains;
  gains.nullspace_kp = 40.0;  // nullspace + DLS + Λ_S + wrench, all at once
  TaskImpedanceController ctrl(Urdf7(), gains, Sel::kFullSe3);
  ctrl.SetControlRate(kRateHz);
  ctrl.LoadConfig(YAML::Load(R"(
formulation: inertia_shaping
desired_inertia: [2.0, 2.0, 2.0, 0.2, 0.2, 0.2]
external_wrench:
  enabled: true
  filter_enabled: true
  bias_calibration_samples: 3
  sensor_frame: link_5
)"));
  auto state = MakeState(7, q, std::vector<double>(7, 0.02));
  Send(ctrl, Wrench6{1.0, 2.0, 3.0, 0.1, 0.2, 0.3});

  // The FIRST Compute (spec T7.1) — which is also the seed + BIAS_CALIBRATING
  // entry tick, the heaviest one. RAII rather than a bare arm/disarm pair: an
  // ASSERT_* added inside the region returns from the test, and a bare disarm
  // line would then never run.
  {
    rtc::testing::ScopedAllocGate gate;
    (void)ctrl.Compute(state);
    EXPECT_EQ(gate.count(), 0u) << "first Compute() with the wrench path allocated";
  }

  // Then a steady-state tick that runs conditioning, the filter and §6.3.
  for (int k = 0; k < 5; ++k) {
    Send(ctrl, Wrench6{1.0, 2.0, 3.0, 0.1, 0.2, 0.3});
    (void)ctrl.Compute(state);
  }
  Send(ctrl, Wrench6{4.0, 5.0, 6.0, 0.4, 0.5, 0.6});
  {
    rtc::testing::ScopedAllocGate gate;
    (void)ctrl.Compute(state);
    EXPECT_EQ(gate.count(), 0u) << "steady-state Compute() with conditioning + §6.3 allocated";
  }
  EXPECT_TRUE(ctrl.GetDiagnosticsForTesting().control_valid);
}

}  // namespace
