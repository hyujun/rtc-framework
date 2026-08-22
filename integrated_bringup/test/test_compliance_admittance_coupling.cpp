// ── The §7 admittance law, as this binding wires it (#469 S3) ────────────────
//
// S2 shipped demo_compliance_controller as a rename-copy with no law of its
// own; S3 gives it one. What is asserted here is the WIRING, not the law:
// rtc_controllers owns the ODE and tests it (admittance_integrator, wrench
// pipeline, state machine), and re-deriving those numbers here would test them
// twice while testing the binding not at all.
//
// So each half gets its own exact oracle, joined at the probe the tick stages:
//
//   published wrench ──▶ WrenchPipeline ──▶ wrench_lwa ──▶ ×α ──▶ Integrator ──▶ x̃
//                       [ frame / sign / staleness ]      [ bit-identical to a
//                                                           local integrator ]
//
// WHY THE WRENCH IS INJECTED RATHER THAN GROWN FROM FINGERTIP FORCES. The
// source adapter (`FromPullEstimate`) has an exhaustive suite of its own
// (#469 S1) and the estimate it packs is pinned against a known load in sim
// (#177 crit#4). Driving this file through fingertip FK would re-assert both
// and leave the law's plumbing — α, dt, the params snapshot, the frame the
// deviation composes onto — asserted by nothing. The one thing injection skips
// is the publish CALL SITE, so that gets an assertion of its own at the bottom.
//
// BIAS FIRST. `bias_samples` defaults to 100 (§3.2.1 MUST), so a fixture that
// published its step force from tick 0 would calibrate the bias TO that force
// and measure a conditioned wrench of exactly zero — green, and about nothing.
// Every program below publishes a no-load wrench until calibration commits.
#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/demo_compliance_controller.hpp"
#include "shipped_config_test_fixture.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace {

using integrated_bringup::DemoComplianceController;
using rtc::ControllerOutput;
using rtc::ControllerState;
using rtc::compliance::ComplianceState;
using rtc::compliance::Wrench6;

using integrated_bringup::testfx::kArmDof;
using integrated_bringup::testfx::kArmHome;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::MakeIiwa7LeapDeviceConfigs;
using integrated_bringup::testfx::MakeIiwa7LeapState;
using integrated_bringup::testfx::MergeShippedShared;
using integrated_bringup::testfx::SharedIiwa7LeapBuilder;
using integrated_bringup::testfx::SharedIiwa7LeapModelConfig;
using integrated_bringup::testfx::ShippedControllerNode;

constexpr const char* kProfile = "iiwa7_leap";
// 100 bias samples + the 0.5 s activation ramp, with slack. Deliberately not
// "just enough": a fixture tuned to the exact boundary turns a change in either
// default into a mystery failure here rather than at its own test.
constexpr int kSettleTicks = 400;
// ω_n = √(K/Λ) = √(200/2) = 10 rad/s, ζ = K_d/(2√(KΛ)) = 1 — critically damped,
// so ~5 time constants is 0.5 s. 800 ticks is 1.6 s.
constexpr int kResponseTicks = 800;

YAML::Node ComplianceConfig() {
  YAML::Node cfg = ShippedControllerNode(kProfile, "demo_compliance_controller");
  MergeShippedShared(cfg, kProfile);
  return cfg;
}

std::unique_ptr<DemoComplianceController> BringUp(const YAML::Node& cfg) {
  auto ctrl = std::make_unique<DemoComplianceController>("", DemoComplianceController::Gains{});
  ctrl->SetSystemModelConfig(SharedIiwa7LeapModelConfig());
  ctrl->SetSharedModelBuilder(SharedIiwa7LeapBuilder());
  ctrl->SetControlRate(1.0 / kDt);
  ctrl->LoadConfig(cfg);
  ctrl->SetDeviceNameConfigs(MakeIiwa7LeapDeviceConfigs());
  // ACTIVATE. Without this every SetDeviceTarget below is dropped by the #196
  // activation-generation gate — a target queued while Inactive is stale by
  // definition — and a fixture that dispatched goals into that hole would be
  // comparing two arms holding still.
  const rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                         "inactive");
  EXPECT_EQ(ctrl->on_activate(inactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  return ctrl;
}

/// Bitwise, not near-equal: the two integrators run the same code on the same
/// input, so a tolerance would absorb precisely the plumbing error this is
/// looking for (a dropped α, a dt off by one tick, a stale params snapshot).
bool SameBits(double a, double b) {
  return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

/// A wrench with `fx` and nothing else. The single-axis shape is the point: a
/// binding that transposed or mixed the frame produces motion on another axis,
/// and a diagonal force would hide that in the norm.
Wrench6 ForceX(double fx) {
  return Wrench6{{fx, 0.0, 0.0, 0.0, 0.0, 0.0}};
}

/// Drives the controller the way production does, including WHERE the wrench
/// acts. The application point is the previous tick's control-frame origin,
/// because that is what a source can actually know: the pull adapter reads the
/// virtual TCP that ComputeControl computed, and publishes from
/// ComputeSecondary — one tick behind, by construction.
///
/// This matters more than it looks. The first version of this fixture published
/// at the world origin, and the pipeline dutifully transported the force across
/// a 0.3 m lever arm into a 3.3 N·m torque that drove the angular deviation
/// straight into its §7.5 box. That is the pipeline being RIGHT; the fixture
/// was wrong. `offset` exists so one test can put the lever arm back
/// deliberately and check the transport itself.
struct Driver {
  DemoComplianceController* ctrl;
  ControllerState state = MakeIiwa7LeapState();
  std::array<double, kArmDof> arm_meas = kArmHome;
  std::uint64_t iteration = 0;
  Eigen::Vector3d offset{Eigen::Vector3d::Zero()};
  Eigen::Vector3d apply_point{Eigen::Vector3d::Zero()};

  /// Run `n` ticks publishing `w` on each one, closing the joint loop so the
  /// arm actually travels. Returns the last tick's probe.
  DemoComplianceController::ComplianceProbe Run(int n, const Wrench6& w) {
    DemoComplianceController::ComplianceProbe probe;
    for (int k = 0; k < n; ++k) {
      for (int i = 0; i < kArmDof; ++i) {
        state.devices[0].positions[static_cast<std::size_t>(i)] =
            arm_meas[static_cast<std::size_t>(i)];
      }
      state.iteration = ++iteration;
      state.t_relative_s = static_cast<double>(iteration) * kDt;
      ctrl->PublishExternalWrenchForTesting(w, apply_point + offset);
      const ControllerOutput out = ctrl->Compute(state);
      probe = ctrl->GetComplianceProbeForTesting();
      apply_point = probe.task_origin;
      for (int i = 0; i < kArmDof && i < out.devices[0].num_channels; ++i) {
        arm_meas[static_cast<std::size_t>(i)] =
            out.devices[0].commands[static_cast<std::size_t>(i)];
      }
    }
    return probe;
  }

  /// Same, but publishing NOTHING — the withheld case (§10.6 / D-A7).
  DemoComplianceController::ComplianceProbe RunSilent(int n) {
    DemoComplianceController::ComplianceProbe probe;
    for (int k = 0; k < n; ++k) {
      for (int i = 0; i < kArmDof; ++i) {
        state.devices[0].positions[static_cast<std::size_t>(i)] =
            arm_meas[static_cast<std::size_t>(i)];
      }
      state.iteration = ++iteration;
      state.t_relative_s = static_cast<double>(iteration) * kDt;
      const ControllerOutput out = ctrl->Compute(state);
      probe = ctrl->GetComplianceProbeForTesting();
      apply_point = probe.task_origin;
      for (int i = 0; i < kArmDof && i < out.devices[0].num_channels; ++i) {
        arm_meas[static_cast<std::size_t>(i)] =
            out.devices[0].commands[static_cast<std::size_t>(i)];
      }
    }
    return probe;
  }
};

// ── The pipeline half: frame, sign, magnitude ───────────────────────────────

TEST(ComplianceAdmittanceCoupling, APublishedForceReachesTheLawOnItsOwnAxis) {
  auto ctrl = BringUp(ComplianceConfig());
  Driver d{ctrl.get()};

  d.Run(kSettleTicks, Wrench6{});  // bias calibrates against no load
  const auto probe = d.Run(kResponseTicks, ForceX(10.0));

  ASSERT_TRUE(probe.engaged) << "the arm lane never ran — every assertion below would be vacuous";
  ASSERT_EQ(probe.alpha, 1.0) << "the §10.7 ramp never completed in " << kSettleTicks << " ticks";

  // The conditioned wrench is the published one: the pull adapter hands over a
  // base-frame vector and this path rotates nothing, so a transpose or a
  // borrowed sensor rotation shows up here rather than three layers down.
  EXPECT_NEAR(probe.wrench_lwa[0], 10.0, 0.2) << "fx did not survive conditioning";
  EXPECT_NEAR(probe.wrench_lwa[1], 0.0, 1e-9) << "fy appeared from nowhere";
  EXPECT_NEAR(probe.wrench_lwa[2], 0.0, 1e-9) << "fz appeared from nowhere";
  // The torque is a LEVER ARM, not a leak: the application point is one tick
  // behind the control frame, so the residual arm is one tick of travel. At
  // steady state that is a settled arm and the moment collapses with it. Bound
  // rather than pinned — this asserts the transport is not inventing a moment,
  // and the test below asserts it computes the right one when there IS an arm.
  EXPECT_LT(probe.wrench_lwa.tail<3>().norm(), 1e-3) << "a settled arm still carried a moment";

  // §7.2 steady state of Λẍ + K_dẋ + K_px = F is x = F/K_p. The default K_p is
  // 200 N/m, so 10 N ⇒ 0.05 m — inside the 0.15 m displacement box, which
  // matters: past it the barrier would be what the number measures.
  EXPECT_NEAR(probe.deviation[0], 10.0 / 200.0, 2e-3)
      << "the compliant frame did not settle at F/K";
  EXPECT_NEAR(probe.deviation[1], 0.0, 1e-6) << "deviation leaked onto y";
  EXPECT_NEAR(probe.deviation[2], 0.0, 1e-6) << "deviation leaked onto z";
  EXPECT_LT(probe.deviation.tail<3>().norm(), 1e-3)
      << "an angular deviation grew from a pure force";
}

TEST(ComplianceAdmittanceCoupling, TheApplicationPointIsTransportedToTheTaskFrame) {
  // `p_sensor` and `p_task` are two different arguments to the same call, and
  // swapping them flips the moment's sign while leaving the force untouched —
  // invisible to every other test here. Offset along +Z with a force along +X
  // gives r x f = (0, d*fx, 0): one component, one sign, one magnitude.
  constexpr double kOffsetZ = 0.2;
  constexpr double kFx = 10.0;
  auto ctrl = BringUp(ComplianceConfig());
  Driver d{ctrl.get()};
  d.Run(kSettleTicks, Wrench6{});
  d.offset = Eigen::Vector3d(0.0, 0.0, kOffsetZ);
  const auto probe = d.Run(kResponseTicks, ForceX(kFx));

  EXPECT_NEAR(probe.wrench_lwa[3], 0.0, 1e-2) << "tau_x";
  EXPECT_NEAR(probe.wrench_lwa[4], kOffsetZ * kFx, 5e-2)
      << "tau_y is not (p_apply - p_task) x f — check which argument is the sensor and which is "
         "the task frame";
  EXPECT_NEAR(probe.wrench_lwa[5], 0.0, 1e-2) << "tau_z";
}

TEST(ComplianceAdmittanceCoupling, TheSignFollowsTheForce) {
  // The one failure this whole sprint inherits: an inverted default flipped
  // every contact gate on the 2026-07-22 p1b run. Asserted as a RELATION
  // between two runs rather than against a remembered number, so it cannot be
  // satisfied by a fixture that happens to push the right way.
  auto pos = BringUp(ComplianceConfig());
  Driver dp{pos.get()};
  dp.Run(kSettleTicks, Wrench6{});
  const double x_pos = dp.Run(kResponseTicks, ForceX(10.0)).deviation[0];

  auto neg = BringUp(ComplianceConfig());
  Driver dn{neg.get()};
  dn.Run(kSettleTicks, Wrench6{});
  const double x_neg = dn.Run(kResponseTicks, ForceX(-10.0)).deviation[0];

  EXPECT_GT(x_pos, 0.0);
  EXPECT_LT(x_neg, 0.0);
  EXPECT_NEAR(x_pos, -x_neg, 1e-9) << "the law is not symmetric — a sign is being applied twice";
}

// ── The integrator half: exactly the core, driven by exactly this α ─────────

TEST(ComplianceAdmittanceCoupling, TheDeviationIsExactlyWhatTheCoreIntegratorHolds) {
  auto ctrl = BringUp(ComplianceConfig());
  const auto& params = ctrl->GetAdmittanceParamsForTesting().admittance;
  rtc::compliance::AdmittanceIntegrator oracle;

  Driver d{ctrl.get()};
  // The load starts as soon as the bias average has committed (100 samples,
  // §3.2.1) and NOT after the 0.5 s ramp — see the counter below for why that
  // difference is the whole test.
  constexpr int kBiasTicks = 150;
  constexpr int kTotalTicks = kSettleTicks + kResponseTicks;
  int compared = 0;
  int ramping_under_load = 0;
  for (int k = 0; k < kTotalTicks; ++k) {
    const Wrench6 w = (k < kBiasTicks) ? Wrench6{} : ForceX(10.0);
    const auto probe = d.Run(1, w);
    ASSERT_TRUE(probe.engaged) << "held at tick " << k;

    // The probe stages the wrench and α BEFORE the step, so feeding the oracle
    // here reproduces the same call the controller just made.
    oracle.Step(params, probe.alpha * probe.wrench_lwa, kDt);
    const Eigen::Matrix<double, 6, 1> expected = oracle.deviation();
    for (int i = 0; i < 6; ++i) {
      ASSERT_TRUE(SameBits(probe.deviation[i], expected[i]))
          << "tick " << k << " component " << i << ": " << probe.deviation[i] << " vs "
          << expected[i];
    }
    ++compared;
    if (probe.alpha < 1.0 && probe.wrench_lwa.norm() > 0.0) {
      ++ramping_under_load;
    }
  }
  EXPECT_EQ(compared, kTotalTicks);
  // THE POSITIVE CONTROL FOR α, and it has to say `under_load`. A first draft
  // counted partial-ramp ticks alone and passed with the α term deleted from
  // the controller: the ramp had finished before the force arrived, so every
  // tick with α < 1 multiplied a wrench of exactly zero and 0 == α·0 either
  // way. The mutant is only visible on ticks that are ramping AND loaded.
  EXPECT_GT(ramping_under_load, 0)
      << "no tick was both ramping and loaded, so α is multiplied by zero and untested";
}

// ── The composition: does X_c actually reach the arm? ───────────────────────

TEST(ComplianceAdmittanceCoupling, TheArmTracksTheCompliantFrameAndNotTheTrajectory) {
  // Everything above is satisfied by a controller that computes a deviation
  // perfectly and then never composes it onto the reference — the arm would sit
  // on the trajectory while the probe reported a textbook x̃. This is the test
  // that fails when the composition, or the feedforward, is dropped: the SAME
  // program with and without a force, compared at the control frame.
  auto loaded = BringUp(ComplianceConfig());
  Driver dl{loaded.get()};
  const Eigen::Vector3d start = dl.Run(kSettleTicks, Wrench6{}).tcp_position;

  // Tracked tick by tick, not just at the end. The FEEDFORWARD half of row 13
  // (ν_c riding the IK's ff term) is invisible at steady state — ν_c is zero
  // there and K_ik alone closes the gap — so a controller that dropped it would
  // reach the same final position by a lag it recovers from. During the
  // transient that lag is ẋ̃/K_ik: the response peaks near ẋ̃ = 0.018 m/s and
  // K_ik is 5 /s, so ~3.7 mm of following error with the term dropped against
  // ~0 with it. The bound below sits between the two.
  double worst_lag = 0.0;
  double peak_speed = 0.0;
  DemoComplianceController::ComplianceProbe with_force;
  for (int k = 0; k < kResponseTicks; ++k) {
    with_force = dl.Run(1, ForceX(10.0));
    worst_lag = std::max(
        worst_lag, std::abs((with_force.tcp_position.x() - start.x()) - with_force.deviation[0]));
    peak_speed = std::max(peak_speed, std::abs(with_force.velocity[0]));
  }
  ASSERT_GT(peak_speed, 5e-3) << "the compliant frame never moved fast enough for a lag to show";
  EXPECT_LT(worst_lag, 1.5e-3) << "the arm lagged the compliant frame by " << worst_lag
                               << " m — the ν_c feedforward is not reaching the IK";

  auto free_run = BringUp(ComplianceConfig());
  Driver df{free_run.get()};
  df.Run(kSettleTicks, Wrench6{});
  const auto without = df.Run(kResponseTicks, Wrench6{});

  ASSERT_TRUE(without.deviation.isZero(0.0)) << "the control run was not force-free";

  const Eigen::Vector3d moved = with_force.tcp_position - without.tcp_position;
  // The arm converges onto X_c, so the displacement it realises is the
  // deviation itself — asserted as a RELATION to x̃ rather than a remembered
  // number, so it stays true when the gains change.
  EXPECT_NEAR(moved.x(), with_force.deviation[0], 5e-3)
      << "the compliant frame moved but the arm did not follow it";
  EXPECT_GT(moved.x(), 0.01) << "the whole displacement is within the assertion's tolerance";
  EXPECT_LT(std::abs(moved.y()), 5e-3);
  EXPECT_LT(std::abs(moved.z()), 5e-3);
}

// ── §10.6: withholding, not zeroing ─────────────────────────────────────────

TEST(ComplianceAdmittanceCoupling, AWithheldWrenchGoesStaleAndDegradesRatherThanHolding) {
  auto ctrl = BringUp(ComplianceConfig());
  Driver d{ctrl.get()};
  d.Run(kSettleTicks, Wrench6{});
  const auto loaded = d.Run(kResponseTicks, ForceX(10.0));
  ASSERT_GT(loaded.deviation[0], 0.01) << "nothing was displaced, so nothing can be seen to fade";
  ASSERT_FALSE(loaded.status.stale);

  // Long enough to pass `timeout` (0.05 s) AND `fadeout_time` (0.1 s).
  const auto faded = d.RunSilent(200);
  EXPECT_TRUE(faded.status.stale) << "an aged sample was not reported stale";
  EXPECT_EQ(faded.state, ComplianceState::kDegraded)
      << "wrench_timeout must degrade — never SAFE_STOP (§3.2)";
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(faded.wrench_lwa[i], 0.0, 1e-9)
        << "component " << i << " was HELD at its last value instead of fading to zero";
  }
  // With K_p > 0 the frame relaxes back once the force is gone — that is the
  // spring, and it is the observable difference between "faded" and "frozen".
  EXPECT_LT(faded.deviation[0], loaded.deviation[0] * 0.5);
}

// ── AC2: every held branch collapses the frame, and none accrues ────────────

TEST(ComplianceAdmittanceCoupling, AnEstopCollapsesTheCompliantFrameAndDoesNotAccrue) {
  auto ctrl = BringUp(ComplianceConfig());
  Driver d{ctrl.get()};
  d.Run(kSettleTicks, Wrench6{});
  ASSERT_GT(d.Run(kResponseTicks, ForceX(10.0)).deviation[0], 0.01);

  ctrl->TriggerEstop();
  const auto held = d.Run(50, ForceX(10.0));
  EXPECT_FALSE(held.engaged);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(held.deviation[i], 0.0)
        << "component " << i
        << ": the deviation survived the E-STOP and would be commanded on "
           "resume";
  }
  // The force kept arriving throughout; a lane that accrued while held would
  // show it here.
  EXPECT_EQ(held.velocity.norm(), 0.0);
}

TEST(ComplianceAdmittanceCoupling, AnUnreadableArmCollapsesTheCompliantFrame) {
  auto ctrl = BringUp(ComplianceConfig());
  Driver d{ctrl.get()};
  d.Run(kSettleTicks, Wrench6{});
  ASSERT_GT(d.Run(kResponseTicks, ForceX(10.0)).deviation[0], 0.01);

  // The F5 device-readability gate's branch: fewer channels than the model has
  // DOF. ComputeControl holds, and holding must reset for the same reason
  // E-STOP does.
  d.state.devices[0].num_channels = kArmDof - 2;
  const auto held = d.Run(20, ForceX(10.0));
  EXPECT_FALSE(held.engaged);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(held.deviation[i], 0.0) << "component " << i;
  }
}

TEST(ComplianceAdmittanceCoupling, ASafeStopHoldsPositionAndCollapsesTheFrame) {
  // sigma_below_critical, forced by raising the threshold above any sigma_min
  // the home posture can produce. Chosen over pose_error_limit because the CLIK
  // tracks the compliant frame CLOSELY — that is the law working — so the pose
  // error stays small no matter how hard the arm is pushed, and a fixture built
  // on it would be tuning a tolerance rather than testing a fault.
  //
  // AND THE ARM IS GIVEN SOMEWHERE TO GO. A first draft asserted `q_cmd ==
  // q_meas` on a controller that had no target: it held either way, and the
  // whole SAFE_STOP branch could be deleted with the test still green. The
  // claim is only meaningful against a run that DOES move, so the same target
  // is dispatched to a fault-free twin and the two are compared.
  // A task-space goal (x, y, z, r, p, y) — device 0's target on this binding is
  // a POSE, not joint angles. The shipped iiwa7_leap trajectory_speed is
  // 0.01 m/s, so travel is measured in millimetres per second and the run has
  // to be long enough to produce one.
  const std::array<double, 6> target = {0.45, 0.10, 0.55, 0.0, 0.0, 0.0};
  constexpr int kTravelTicks = 800;  // 1.6 s ⇒ ~16 mm at the shipped speed

  auto healthy = BringUp(ComplianceConfig());
  healthy->SetDeviceTarget(0, std::span<const double>(target));
  Driver dh{healthy.get()};
  const Eigen::Vector3d start = dh.Run(1, Wrench6{}).tcp_position;
  const auto moving = dh.Run(kTravelTicks, Wrench6{});
  ASSERT_NE(moving.state, ComplianceState::kSafeStop);
  const double travelled = (moving.tcp_position - start).norm();
  // Sub-millimetre, and that is the shipped configuration rather than a weak
  // fixture: trajectory_speed is 0.01 m/s and a quintic profile spends its first
  // second accelerating. What the assertion needs is separation from the faulted
  // run, which holds to 1e-9 rad — four orders of magnitude below this.
  ASSERT_GT(travelled, 1e-4) << "the control arm never moved, so 'it stopped' asserts nothing";

  YAML::Node cfg = ComplianceConfig();
  cfg["singularity_critical"] = 100.0;
  auto ctrl = BringUp(cfg);
  ctrl->SetDeviceTarget(0, std::span<const double>(target));
  Driver d{ctrl.get()};
  const auto stopped = d.Run(kTravelTicks + 1, Wrench6{});

  ASSERT_EQ(stopped.state, ComplianceState::kSafeStop)
      << "the fixture never tripped the fault it is about";
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(stopped.deviation[i], 0.0)
        << "component " << i << ": a latched SAFE_STOP left a displacement waiting to be commanded";
  }

  // Position hold: q_cmd == q_meas, on an arm that was mid-travel toward a
  // target it will now never reach. Asserted through the OUTPUT rather than the
  // probe, because "the arm stopped" is a claim about the command and dq = 0 is
  // only the mechanism.
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(d.arm_meas[static_cast<std::size_t>(i)], kArmHome[static_cast<std::size_t>(i)],
                1e-9)
        << "joint " << i << " travelled under a latched SAFE_STOP while its healthy twin covered "
        << travelled << " m";
  }
  for (int i = 0; i < kArmDof; ++i) {
    d.state.devices[0].positions[static_cast<std::size_t>(i)] =
        d.arm_meas[static_cast<std::size_t>(i)];
  }
  d.state.iteration = ++d.iteration;
  const ControllerOutput out = ctrl->Compute(d.state);
  ASSERT_EQ(out.devices[0].num_channels, kArmDof);
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                d.arm_meas[static_cast<std::size_t>(i)], 1e-12)
        << "joint " << i << " kept moving under SAFE_STOP";
  }
}

// ── The seam injection skips: the publish call site ─────────────────────────

TEST(ComplianceAdmittanceCoupling, TheTickItselfFeedsThePipelineFromTheSource) {
  // Every test above injects the wrench, so all of them pass with the publish
  // site in ComputeSecondary deleted. This one uses no injection at all: the
  // pipeline can only become `valid` if the controller's own lane published
  // into it. The pull estimator is disabled in this fixture (no fingertip
  // forces), so what is asserted is the complement — WITHOUT a source, the law
  // never runs on a wrench nobody produced.
  auto ctrl = BringUp(ComplianceConfig());
  Driver d{ctrl.get()};
  const auto probe = d.RunSilent(kSettleTicks);

  ASSERT_TRUE(probe.engaged);
  EXPECT_FALSE(probe.status.valid)
      << "the pipeline reported a sample although nothing published one";
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(probe.deviation[i], 0.0)
        << "component " << i << ": the frame moved with no wrench behind it";
  }
}

}  // namespace
