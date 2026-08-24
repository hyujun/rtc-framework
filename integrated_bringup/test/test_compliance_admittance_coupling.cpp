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
#include <limits>
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

TEST(ComplianceAdmittanceCoupling, WithoutASourceTheLawRunsOnNoWrenchAtAll) {
  // The complement of the test below. No injection and no fingertip forces, so
  // nothing anywhere can hand the pipeline a sample — and the frame must not
  // move. Necessary but NOT sufficient for the publish call site: this half
  // passes with that site deleted, which is why it no longer carries the name.
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

TEST(ComplianceAdmittanceCoupling, TheTickItselfFeedsThePipelineFromTheSource) {
  // THE POSITIVE CONTROL for the one seam injection cannot reach. Every other
  // test in this file publishes through PublishExternalWrenchForTesting, so all
  // of them — and the test above with them — stay green with the
  // `wrench_pipeline_.Publish(...)` block in ComputeSecondary deleted. The
  // production symptom of that deletion is an arm that silently never complies:
  // no fault, no log, no CSV column out of range.
  //
  // So: no injection at all. A pinch is staged on the fingertip lane and the
  // ONLY path from there to the pipeline is the controller's own tick —
  // fingertip decode → FK → pull estimate → FromPullEstimate → Publish.
  //
  // THE VIRTUAL TCP IS TURNED ON, and that is not fixture convenience.
  // `FromPullEstimate` withholds unless `vtcp.valid` (an invalid virtual TCP
  // resolves to the identity, whose origin is a finite and completely wrong
  // place to hang a force), and the iiwa7_leap SIM profile ships
  // `virtual_tcp_mode: disabled` — so on that profile this source can never
  // publish, whatever the fingertips report. Both hardware profiles
  // (ur5e_p1a/p1b) ship `centroid`, which is what this asks for: the shipped
  // hardware configuration of the lane, run on the model the suite has.
  YAML::Node cfg = ComplianceConfig();
  cfg["virtual_tcp_mode"] = "centroid";
  auto ctrl = BringUp(cfg);
  Driver d{ctrl.get()};

  // DeriveFingertipCounts bounds the whole decode loop by this; leave it at the
  // bare fixture's 0 and every force below is written and never read (the trap
  // test_compliance_diag_log's StagePinch documents).
  auto& dev1 = d.state.devices[1];
  dev1.num_inference_groups = 4;
  // A COMPRESSIVE pinch, which is a statement about direction and not only
  // magnitude: the estimator activates a contact on f_n = −n̂·f_obj, where n̂ is
  // the FK-resolved pinch axis, so a force orthogonal to it registers nothing at
  // any magnitude. −y in the fingertip LINK frame is what projects onto that
  // axis at this hand's home pose (the ±z that SetFingertipForce writes is very
  // nearly orthogonal to it, and no sign of it ever activates the thumb —
  // `required_roles: [thumb]` then rejects every tick). If the hand model or its
  // home pose changes, this fails with the reason code below rather than
  // silently going quiet.
  for (int f = 0; f < 4; ++f) {
    dev1.inference_enable[static_cast<std::size_t>(f)] = true;
    const int base =
        f * static_cast<int>(integrated_bringup::kHandInferenceValuesPerFingertipCapacity);
    for (int c = 0; c < 4; ++c) {
      dev1.inference_data[static_cast<std::size_t>(base + c)] = 0.0F;
    }
    dev1.inference_data[static_cast<std::size_t>(base + 2)] = -5.0F;  // f_y, link frame
  }

  const auto probe = d.RunSilent(kSettleTicks);

  ASSERT_TRUE(probe.engaged) << "the arm lane never ran — the assertion below would be vacuous";
  EXPECT_TRUE(probe.status.valid)
      << "no sample ever reached the pipeline: the tick did not publish the estimate its own "
         "source produced, so the law would never see a real wrench in production "
         "(PullInvalidReason="
      << static_cast<int>(probe.invalid_reason) << ")";
}

// ── The controller-local fault latch (#260, §10.6) ──────────────────────────

TEST(ComplianceAdmittanceCoupling, ALatchedFaultSurvivesReactivationAndExitsOnlyThroughResetFault) {
  // THE CAUSE HAS TO BE ABLE TO GO AWAY. With a permanent one every tick
  // re-latches, "still latched" asserts nothing, and an on_activate that
  // launders the latch passes just as well. So the fault is raised by a
  // configured threshold and then RECONFIGURED away — LoadConfig is a non-RT
  // reconfigure entry point, and this is the one thing in reach that removes a
  // critical cause without also removing the latch.
  YAML::Node faulted = ComplianceConfig();
  faulted["singularity_critical"] = 100.0;  // above any sigma_min this posture has
  auto ctrl = BringUp(faulted);
  // A GOAL, so the arm holds a posture with a sigma_min to speak of. Without one
  // this fixture's closed loop hands the un-seeded command straight back as the
  // measurement and the arm sits at the zero configuration — where the iiwa is
  // straight and sigma_min is exactly 0, so every threshold latches and the
  // reconfigure below could never remove the cause.
  const std::array<double, 6> target = {0.45, 0.10, 0.55, 0.0, 0.0, 0.0};
  ctrl->SetDeviceTarget(0, std::span<const double>(target));
  Driver d{ctrl.get()};

  ASSERT_EQ(d.Run(2, Wrench6{}).state, ComplianceState::kSafeStop)
      << "the fixture never tripped the fault it is about";
  // #260's observable half. Without the override the CM's /rtc_cm/reset_fault
  // reads the base class's `false` and reports "nothing latched" at an arm the
  // law has frozen.
  EXPECT_TRUE(ctrl->HasLatchedFault()) << "the latch is invisible off the RT thread";

  // The key is absent from the shipped YAML (the §7 block takes core defaults),
  // and an absent key leaves the parser's previous value standing — so the
  // restore has to name it.
  YAML::Node healed = ComplianceConfig();
  healed["singularity_critical"] = 0.005;
  ctrl->LoadConfig(healed);
  // LoadConfig is a CONFIGURE-time entry point and re-running it drops the
  // integrated command back to its zero init; on this closed-loop fixture the
  // next tick would hand that back as the measurement and teleport the arm into
  // the singular zero configuration, re-raising the very cause being removed.
  // A fresh goal re-seeds the command from the measurement, which is what the
  // production configure path gets from the arm self-init.
  ctrl->SetDeviceTarget(0, std::span<const double>(target));
  ASSERT_EQ(d.Run(2, Wrench6{}).state, ComplianceState::kSafeStop)
      << "SAFE_STOP is latched: removing the cause must not release it";

  // A request made below Active dies at the activation boundary, and the
  // activation itself does not clear anything: §10.6 forbids automatic
  // recovery, and a deactivate/activate cycle is a controller switch, not an
  // operator re-authorising a fault.
  const rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  const rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                         "inactive");
  ASSERT_EQ(ctrl->on_deactivate(active), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ctrl->ResetFault();  // unconsumed — no tick runs while Inactive
  ASSERT_EQ(ctrl->on_activate(inactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);

  const auto after_cycle = d.Run(2, Wrench6{});
  EXPECT_EQ(after_cycle.state, ComplianceState::kSafeStop)
      << "re-activation laundered a latched SAFE_STOP: the arm resumed compliant motion with the "
         "fault cause never diagnosed and no record that it fired";
  EXPECT_TRUE(ctrl->HasLatchedFault());

  // The one exit. Consumed at the head of the next tick, on the RT thread.
  ctrl->ResetFault();
  const auto after_reset = d.Run(2, Wrench6{});
  EXPECT_NE(after_reset.state, ComplianceState::kSafeStop)
      << "the reset request never reached the state machine";
  EXPECT_FALSE(ctrl->HasLatchedFault());
}

TEST(ComplianceAdmittanceCoupling, AHeldTickReportsTheLatchedStateRatherThanHolding) {
  // The mirror the diagnostic row carries is not a place to write a literal.
  // A held tick does not step the FSM, so what it must report is the state the
  // FSM is actually in — and forcing HOLDING there reported a latched SAFE_STOP
  // as HOLDING on every row of the outage, which is the one row an operator
  // reads to find out that the controller is latched at all.
  YAML::Node cfg = ComplianceConfig();
  cfg["singularity_critical"] = 100.0;
  auto ctrl = BringUp(cfg);
  Driver d{ctrl.get()};
  ASSERT_EQ(d.Run(2, Wrench6{}).state, ComplianceState::kSafeStop);

  d.state.devices[0].num_channels = kArmDof - 2;  // F5 gate: the arm goes unreadable
  const auto held = d.Run(5, Wrench6{});
  ASSERT_FALSE(held.engaged) << "the hold path never ran";
  EXPECT_EQ(held.state, ComplianceState::kSafeStop)
      << "a held tick reported HOLDING while the state machine was latched";
}

// ── Holds that are not the F5 gate (#469 review) ────────────────────────────

TEST(ComplianceAdmittanceCoupling, AMidFunctionHoldExpiresTheWrenchLikeEveryOtherHold) {
  // The non-finite-Jacobian hold returns from the MIDDLE of ComputeControl,
  // downstream of the engagement flag and upstream of the pipeline Update. That
  // combination is what made it invisible: `engaged` stayed true, so no falling
  // edge ran, and `Update()` never ran, so the wrench age — which advances
  // inside Read(), not on the clock — simply stopped. A producer that died
  // during the hold would then be read on the resume tick as `age = dt,
  // fade = 1.0`: a seconds-old force applied at full weight, which is §10.6's
  // last-value hold reached by standing still.
  auto ctrl = BringUp(ComplianceConfig());
  Driver d{ctrl.get()};
  d.Run(kSettleTicks, Wrench6{});
  const auto running = d.Run(kResponseTicks, ForceX(10.0));
  ASSERT_TRUE(running.engaged);
  ASSERT_TRUE(running.status.valid) << "the fixture never got a wrench into the pipeline";
  ASSERT_GT(running.deviation[0], 0.01) << "the law was not actually running";

  // A joint state that has stopped being a number: the Jacobian goes non-finite
  // and CLIK holds. Deliberately NOT the F5 gate — the device is perfectly
  // readable here, which is exactly why this path needed its own answer.
  d.arm_meas[0] = std::numeric_limits<double>::quiet_NaN();
  const auto held = d.RunSilent(10);

  EXPECT_FALSE(held.engaged) << "the mid-function hold never disengaged the law";
  EXPECT_FALSE(held.status.valid)
      << "the pipeline still owns the pre-hold sample: its age froze for the whole hold instead of "
         "expiring, so the resume tick would hand a stale wrench to the law at full weight";
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(held.deviation[i], 0.0)
        << "component " << i << ": the deviation survived a hold and would be commanded on resume";
  }
}

TEST(ComplianceAdmittanceCoupling, AControlFrameHoldExpiresTheWrenchLikeEveryOtherHold) {
  // The frame-transition hold, the second of the two mid-function returns and
  // the one that can last kComplianceVtcpFrameWaitTicks — 1000 ticks, two whole
  // seconds of an arm that is NOT being driven by the law. It is reached with an
  // external goal authored in one control frame while another is live, so the
  // goal fires unchanged when its frame comes back (#292).
  YAML::Node cfg = ComplianceConfig();
  cfg["virtual_tcp_mode"] = "centroid";
  auto ctrl = BringUp(cfg);
  const std::array<double, 6> target = {0.45, 0.10, 0.55, 0.0, 0.0, 0.0};
  ctrl->SetDeviceTarget(0, std::span<const double>(target));  // EXTERNAL: not a hold seed
  Driver d{ctrl.get()};
  d.Run(kSettleTicks, Wrench6{});
  const auto running = d.Run(200, ForceX(10.0));
  ASSERT_TRUE(running.engaged);
  ASSERT_TRUE(running.status.valid) << "the fixture never got a wrench into the pipeline";

  // The control frame changes under the standing goal: the virtual TCP is turned
  // off at runtime, which is a live gains write and exactly the transition
  // ClassifyFrameTransition calls kHoldExternal.
  auto g = ctrl->get_gains();
  g.vtcp.mode = integrated_bringup::VirtualTcpMode::kDisabled;
  ctrl->set_gains(g);

  const auto held = d.RunSilent(20);
  EXPECT_FALSE(held.engaged) << "the frame hold never disengaged the law";
  EXPECT_FALSE(held.status.valid)
      << "the pipeline still owns the pre-hold sample: through a hold that can run for "
      << "1000 ticks its age would not advance at all, and the resume tick would apply a "
      << "two-second-old wrench at full weight";
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(held.deviation[i], 0.0) << "component " << i << ": the deviation survived the hold";
  }
}

TEST(ComplianceAdmittanceCoupling, AHoldClearsTheSourceVerdictItCanNoLongerRefresh) {
  // ComputeSecondary is the only writer of the source verdict and it does not
  // run under E-STOP at all, so without an explicit clear the members freeze at
  // whatever the last living estimate said. The first tick after the halt then
  // feeds the FSM a `quality_low` / `invalid_reason` describing an estimate that
  // no longer exists.
  auto ctrl = BringUp(ComplianceConfig());
  Driver d{ctrl.get()};
  d.state.devices[1].num_inference_groups = 4;
  for (int f = 0; f < 3; ++f) {
    integrated_bringup::testfx::SetFingertipForce(d.state, f, 3.0F);
  }
  const auto with_source = d.Run(50, Wrench6{});
  ASSERT_NE(with_source.invalid_reason, 0)
      << "the pull lane never produced a verdict, so this test cannot show it being cleared";

  ctrl->TriggerEstop();
  const auto halted = d.Run(5, Wrench6{});
  ASSERT_FALSE(halted.engaged);
  EXPECT_EQ(halted.invalid_reason, 0)
      << "the verdict outlived the estimate it describes and would gate the FSM on resume";
  EXPECT_FALSE(halted.quality_low);
}

// ── §10.7 ramp vs §10.6 lattice ─────────────────────────────────────────────

TEST(ComplianceAdmittanceCoupling, ABiasReEntryReArmsTheActivationRamp) {
  // THE SHIPPED ORDER OF EVENTS, not a contrived one: the pull estimator
  // withholds while there is no grasp, so the pipeline releases the bias gate
  // with the debt still owed and the controller runs to RUNNING against no
  // wrench at all. Minutes later a grasp starts publishing, the average is
  // re-entered and committed — and THAT is the first tick a conditioned wrench
  // exists. A ramp that ran to completion during the idle stretch is no ramp:
  // §10.7's "does not step the arm" would be spent on ticks with nothing to
  // ramp, and the first real force would land at alpha = 1.
  auto ctrl = BringUp(ComplianceConfig());
  Driver d{ctrl.get()};

  const auto idle = d.RunSilent(kSettleTicks);
  ASSERT_EQ(idle.alpha, 1.0) << "the no-source path must still reach RUNNING (D-A7)";
  ASSERT_FALSE(idle.status.valid);

  // Data arrives. The pipeline re-enters BIAS_CALIBRATING and suppresses its
  // output for the whole average, so the ramp must be back at its START when the
  // first conditioned wrench finally appears — not merely below 1. Measured ON
  // the commit tick, because that is the only tick where the two candidate
  // behaviours are far apart: a ramp that merely restarted when the data arrived
  // has already spent the whole average accruing (~0.4 at these rates), while a
  // ramp the average suspends is still at zero plus this tick's dt.
  double alpha_at_commit = -1.0;
  for (int k = 0; k < 200 && alpha_at_commit < 0.0; ++k) {
    const auto p = d.Run(1, ForceX(10.0));
    if (p.status.bias_calibrated) {
      alpha_at_commit = p.alpha;
    }
  }
  ASSERT_GE(alpha_at_commit, 0.0)
      << "the average never committed — this test is not about what it thinks";
  EXPECT_LT(alpha_at_commit, 0.01)
      << "the ramp had already run during the bias average, so the first wrench the law ever "
         "conditioned entered at alpha="
      << alpha_at_commit << " instead of at the start of the ramp";

  // And it still finishes.
  EXPECT_EQ(d.Run(kSettleTicks, ForceX(10.0)).alpha, 1.0);
}

// ── Config validation ───────────────────────────────────────────────────────

TEST(ComplianceAdmittanceCoupling, ANonPositiveDegradedRecoveryTimeIsRejected) {
  // `std::max(0.0, v)` stood here and admitted exactly the value it was written
  // to exclude: a zero dwell promotes DEGRADED → RUNNING on the first
  // fault-free tick, so a wrench flickering in and out of `stale` chatters
  // between the two every tick.
  for (const double bad : {0.0, -0.1}) {
    YAML::Node cfg = ComplianceConfig();
    cfg["degraded_recovery_time"] = bad;
    auto ctrl = std::make_unique<DemoComplianceController>("", DemoComplianceController::Gains{});
    ctrl->SetSystemModelConfig(SharedIiwa7LeapModelConfig());
    ctrl->SetSharedModelBuilder(SharedIiwa7LeapBuilder());
    ctrl->SetControlRate(1.0 / kDt);
    try {
      ctrl->LoadConfig(cfg);
      ADD_FAILURE() << "degraded_recovery_time = " << bad << " was accepted";
    } catch (const std::runtime_error& e) {
      // Named, so this cannot pass on an unrelated throw from the same call.
      EXPECT_NE(std::string(e.what()).find("degraded_recovery_time"), std::string::npos)
          << "threw for a different reason: " << e.what();
    }
  }
}

}  // namespace
