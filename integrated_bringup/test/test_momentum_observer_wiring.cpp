// Wiring for #135 Layer 1 — the layer that turns a DeviceState plus a Pinocchio
// handle into the four nv-vectors the momentum observer consumes.
//
// Two kinds of case here, and the second is why this file uses a real URDF at
// all. The gate cases are pure logic. The dynamics cases run the observer
// against an ACTUAL arm sub-model, because the failure this wiring exists to
// prevent — mixing the model's internal joint order with the device order —
// produces a finite, smooth, wrong residual that no gate and no NaN check would
// catch. Only an end-to-end oracle ("a known external torque comes back out")
// separates a correct assembly from a plausible one.
//
// WHICH arm is not a free choice: iiwa7_leap, because its URDF ships in
// `robot_descriptions` (this repo) while the ur5e_p1b profile resolves
// `hand_description`, which lives outside it. This file used to sit on the
// ur5e_p1b fixture and every model-backed case here threw "package
// 'hand_description' not found" in CI — 13 of 14 red, invisible because
// integrated_bringup runs in the continue-on-error best-effort lane. The oracle
// above therefore never once ran off this machine. Any fixture used here must
// resolve only in-repo packages; a case that genuinely needs the closed chain
// belongs in test_wbc_closed_chain_projection_sharing.cpp instead.
//
// The move also un-vacuums one gate: on the 6-DOF ur5e, Jᵀ is square, so the
// payload least-squares is exactly determined and `fit_error` is identically
// zero — `max_fit_error` could never fire. iiwa7's 7 DOF make it
// over-determined, which is the case the gate exists for.
#include "integrated_bringup/support/momentum_observer_wiring.hpp"

#include "iiwa7_leap_test_fixture.hpp"

#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace rub = rtc_urdf_bridge;
using integrated_bringup::MomentumObserverInputsReadable;
using integrated_bringup::MomentumObserverWiring;

constexpr int kArmDof = integrated_bringup::testfx::kArmDof;  // 7
constexpr double kDt = 0.001;

// The external torque the cases below must recover, and the swing torque the
// moving case adds. Distinct per joint on purpose: a uniform vector would let a
// wiring that permutes the residual pass by symmetry, which is the single
// failure this file exists to catch.
//
// Deduced extent + static_assert, not std::array<double, kArmDof>: an array
// whose size is FIXED accepts a short initialiser list and zero-fills, which is
// exactly how four of these lists stayed 6 long while kArmDof became 7. Each
// then indexed one past its own end — consistent garbage in the quasi-static
// cases, and a diverged plant in the integrating one. Sized this way, a list
// that does not match the arm is a compile error.
constexpr double kQuasiStaticTauExt[]{2.0, -3.0, 1.0, 0.5, -0.25, 0.75, -1.5};
static_assert(std::size(kQuasiStaticTauExt) == static_cast<std::size_t>(kArmDof),
              "one quasi-static external torque per DOF");

// Small distal components: the moving arm is gravity-compensated and undamped,
// so a constant torque is a constant acceleration and a light distal joint
// reaches the divergence guard long before the observer's answer can be read.
constexpr double kMovingTauExt[]{0.5, -0.8, 0.3, 0.2, 0.05, -0.03, 0.02};
static_assert(std::size(kMovingTauExt) == static_cast<std::size_t>(kArmDof),
              "one moving-case external torque per DOF");

// ── Gate fixtures (no model) ─────────────────────────────────────────────────

/// A device reporting all three lanes cleanly over `dof` slots.
rtc::DeviceState FullyReadableDevice(int dof = kArmDof) {
  rtc::DeviceState dev;
  dev.valid = true;
  dev.num_channels = dof;
  dev.hole_mask = 0;
  dev.velocity_hole_mask = 0;
  dev.effort_hole_mask = 0;
  for (int i = 0; i < dof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    dev.positions[u] = 0.1 * i;
    dev.velocities[u] = 0.0;
    dev.efforts[u] = 0.0;
  }
  return dev;
}

rtc::ControllerState StateWith(const rtc::DeviceState& dev) {
  rtc::ControllerState s;
  s.num_devices = 1;
  s.devices[0] = dev;
  s.dt = kDt;
  return s;
}

// ── Model fixtures ───────────────────────────────────────────────────────────

std::shared_ptr<const pinocchio::Model> ArmModel() {
  return integrated_bringup::testfx::SharedIiwa7LeapBuilder()->GetReducedModel("iiwa7");
}

/// Arm joint names in the model's own order.
std::vector<std::string> ArmJointNames(const rub::RtModelHandle& h) {
  return h.GetPinocchioJointNames();
}

double MaxAbsDiff(const std::vector<double>& a, const std::vector<double>& b) {
  double m = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i)
    m = std::max(m, std::fabs(a[i] - b[i]));
  return m;
}

/// g(q) in DEVICE order — what a resting arm's joint torque sensor would read
/// while holding this posture against gravity with nothing in its hand.
std::vector<double> GravityInDeviceOrder(rub::RtModelHandle& h, const std::vector<double>& q_dev) {
  h.ComputeGeneralizedGravity(q_dev);
  std::vector<double> g_dev(static_cast<std::size_t>(kArmDof), 0.0);
  h.ReorderOutput(h.GetGeneralizedGravity(), std::span<double>(g_dev.data(), g_dev.size()));
  return g_dev;
}

}  // namespace

// ── The lane gate ────────────────────────────────────────────────────────────

// All three lanes, because [MO-3a] reads all three. Each sub-case punches ONE
// hole, so a gate that forgot a lane fails exactly the sub-case for that lane
// rather than failing everything or nothing.
TEST(MomentumObserverGate, AsksEveryLaneItReads) {
  EXPECT_TRUE(MomentumObserverInputsReadable(FullyReadableDevice(), kArmDof));

  {
    rtc::DeviceState dev = FullyReadableDevice();
    dev.valid = false;
    EXPECT_FALSE(MomentumObserverInputsReadable(dev, kArmDof)) << "device not reporting";
  }
  {
    rtc::DeviceState dev = FullyReadableDevice();
    dev.hole_mask = 1ULL << 2;
    EXPECT_FALSE(MomentumObserverInputsReadable(dev, kArmDof)) << "position hole";
  }
  {
    rtc::DeviceState dev = FullyReadableDevice();
    dev.velocity_hole_mask = 1ULL << 4;
    EXPECT_FALSE(MomentumObserverInputsReadable(dev, kArmDof)) << "velocity hole";
  }
  {
    rtc::DeviceState dev = FullyReadableDevice();
    dev.effort_hole_mask = 1ULL << 0;
    EXPECT_FALSE(MomentumObserverInputsReadable(dev, kArmDof)) << "effort hole";
  }
  {
    // The whole-lane case the effort lane actually hits in the field: a driver
    // that never publishes JointState.effort at all.
    rtc::DeviceState dev = FullyReadableDevice();
    dev.effort_hole_mask = ~0ULL;
    EXPECT_FALSE(MomentumObserverInputsReadable(dev, kArmDof)) << "effort lane absent";
  }
  {
    rtc::DeviceState dev = FullyReadableDevice();
    dev.num_channels = kArmDof - 1;
    EXPECT_FALSE(MomentumObserverInputsReadable(dev, kArmDof)) << "too narrow";
  }
  {
    // A hole ABOVE the model width must not close the gate — the observer only
    // indexes [0, dof).
    rtc::DeviceState dev = FullyReadableDevice();
    dev.num_channels = kArmDof + 2;
    dev.effort_hole_mask = 1ULL << (kArmDof + 1);
    EXPECT_TRUE(MomentumObserverInputsReadable(dev, kArmDof)) << "hole outside the model width";
  }
}

// ── Configure validation ─────────────────────────────────────────────────────

TEST(MomentumObserverWiringTest, ConfigureRejectsBadArguments) {
  rub::RtModelHandle h(ArmModel());
  const std::vector<std::string> names = ArmJointNames(h);
  const std::vector<double> gains(kArmDof, 30.0);
  MomentumObserverWiring w;

  EXPECT_THROW(integrated_bringup::ConfigureMomentumObserverWiring(nullptr, names, 0, gains, w),
               std::invalid_argument);
  EXPECT_THROW(integrated_bringup::ConfigureMomentumObserverWiring(ArmModel(), names, -1, gains, w),
               std::invalid_argument);
  EXPECT_THROW(integrated_bringup::ConfigureMomentumObserverWiring(
                   ArmModel(), names, rtc::ControllerState::kMaxDevices, gains, w),
               std::invalid_argument);
  // dof must equal nv — truncating would observe part of the arm and report the
  // rest of it as external torque.
  {
    std::vector<std::string> short_names(names.begin(), names.end() - 1);
    EXPECT_THROW(
        integrated_bringup::ConfigureMomentumObserverWiring(ArmModel(), short_names, 0, gains, w),
        std::invalid_argument);
  }
  // A name the model does not carry must be rejected rather than silently
  // falling back to the memcpy reorder — that fallback is exactly the
  // unverified identity assumption this wiring owns its handle to avoid.
  {
    std::vector<std::string> wrong = names;
    wrong.back() = "not_a_joint_on_this_arm";
    EXPECT_THROW(
        integrated_bringup::ConfigureMomentumObserverWiring(ArmModel(), wrong, 0, gains, w),
        std::invalid_argument);
  }
  EXPECT_FALSE(w.configured) << "a rejected configure left a tickable wiring";
  EXPECT_FALSE(w.enabled());

  EXPECT_NO_THROW(
      integrated_bringup::ConfigureMomentumObserverWiring(ArmModel(), names, 0, gains, w));
  EXPECT_TRUE(w.configured);
  EXPECT_TRUE(w.enabled());
  EXPECT_EQ(w.dof, kArmDof);
  ASSERT_NE(w.handle, nullptr);
  EXPECT_EQ(w.handle->nv(), kArmDof);

  // A configure that fails AFTER a successful one must not leave the previous
  // observer tickable — the caller's next tick would otherwise run against a
  // model it thinks it replaced.
  EXPECT_THROW(integrated_bringup::ConfigureMomentumObserverWiring(nullptr, names, 0, gains, w),
               std::invalid_argument);
  EXPECT_FALSE(w.enabled());
}

// ── AC5 glue half — a closed gate freezes the observer ───────────────────────

TEST(MomentumObserverWiringTest, ClosedLaneGateHoldsTheObserverAndFreezesTheResidual) {
  rub::RtModelHandle h(ArmModel());
  MomentumObserverWiring w;
  integrated_bringup::ConfigureMomentumObserverWiring(ArmModel(), ArmJointNames(h), 0,
                                                      std::vector<double>(kArmDof, 40.0), w);

  const std::vector<double> q_dev(integrated_bringup::testfx::kArmHome.begin(),
                                  integrated_bringup::testfx::kArmHome.end());
  const std::vector<double> g_dev = GravityInDeviceOrder(h, q_dev);
  const std::vector<double> tau_ext(std::begin(kQuasiStaticTauExt),
                                   std::end(kQuasiStaticTauExt));

  rtc::DeviceState dev = FullyReadableDevice();
  for (int i = 0; i < kArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    dev.positions[u] = q_dev[u];
    dev.velocities[u] = 0.0;
    dev.efforts[u] = g_dev[u] - tau_ext[u];
  }

  rtc::ControllerState state = StateWith(dev);
  for (int k = 0; k < 3000; ++k)
    ASSERT_TRUE(integrated_bringup::UpdateMomentumObserver(state, w)) << "tick " << k;

  const std::vector<double> frozen = w.residual_device;
  ASSERT_GT(MaxAbsDiff(frozen, std::vector<double>(kArmDof, 0.0)), 0.5)
      << "residual never grew, so freezing it proves nothing";

  // Effort lane loses one slot — everything else stays perfect.
  state.devices[0].effort_hole_mask = 1ULL << 3;
  for (int k = 0; k < 100; ++k) {
    EXPECT_FALSE(integrated_bringup::UpdateMomentumObserver(state, w));
    EXPECT_FALSE(w.observer.valid());
    EXPECT_EQ(w.observer.invalid_reason(), rtc::estimation::MomentumInvalidReason::kHeld);
    EXPECT_EQ(w.residual_device, frozen) << "held tick moved the residual";
  }

  // Velocity lane alone closes it too.
  state.devices[0].effort_hole_mask = 0;
  state.devices[0].velocity_hole_mask = 1ULL << 1;
  EXPECT_FALSE(integrated_bringup::UpdateMomentumObserver(state, w));
  EXPECT_EQ(w.observer.invalid_reason(), rtc::estimation::MomentumInvalidReason::kHeld);
  EXPECT_EQ(w.residual_device, frozen);
}

// ── Dynamics assembly, quasi-static ──────────────────────────────────────────

// An arm holding its own weight and nothing else must report nothing. This is
// the case that catches a dropped or sign-flipped gravity term: g(q) at this
// posture is tens of N·m, so getting it wrong cannot hide.
TEST(MomentumObserverWiringTest, GravityHoldAloneProducesNoResidual) {
  rub::RtModelHandle h(ArmModel());
  MomentumObserverWiring w;
  integrated_bringup::ConfigureMomentumObserverWiring(ArmModel(), ArmJointNames(h), 0,
                                                      std::vector<double>(kArmDof, 40.0), w);

  const std::vector<double> q_dev(integrated_bringup::testfx::kArmHome.begin(),
                                  integrated_bringup::testfx::kArmHome.end());
  const std::vector<double> g_dev = GravityInDeviceOrder(h, q_dev);
  ASSERT_GT(MaxAbsDiff(g_dev, std::vector<double>(kArmDof, 0.0)), 5.0)
      << "posture is gravity-neutral, so this case would be vacuous";

  rtc::DeviceState dev = FullyReadableDevice();
  for (int i = 0; i < kArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    dev.positions[u] = q_dev[u];
    dev.velocities[u] = 0.0;
    dev.efforts[u] = g_dev[u];
  }

  const rtc::ControllerState state = StateWith(dev);
  for (int k = 0; k < 3000; ++k)
    ASSERT_TRUE(integrated_bringup::UpdateMomentumObserver(state, w));

  EXPECT_LT(MaxAbsDiff(w.residual_device, std::vector<double>(kArmDof, 0.0)), 1e-9);
}

// The positive control: a KNOWN external torque must come back out, in the
// caller's device order.
TEST(MomentumObserverWiringTest, KnownExternalTorqueIsRecovered) {
  rub::RtModelHandle h(ArmModel());
  MomentumObserverWiring w;
  integrated_bringup::ConfigureMomentumObserverWiring(ArmModel(), ArmJointNames(h), 0,
                                                      std::vector<double>(kArmDof, 40.0), w);

  const std::vector<double> q_dev(integrated_bringup::testfx::kArmHome.begin(),
                                  integrated_bringup::testfx::kArmHome.end());
  const std::vector<double> g_dev = GravityInDeviceOrder(h, q_dev);
  const std::vector<double> tau_ext(std::begin(kQuasiStaticTauExt),
                                   std::end(kQuasiStaticTauExt));

  rtc::DeviceState dev = FullyReadableDevice();
  for (int i = 0; i < kArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    dev.positions[u] = q_dev[u];
    dev.velocities[u] = 0.0;
    dev.efforts[u] = g_dev[u] - tau_ext[u];
  }

  const rtc::ControllerState state = StateWith(dev);
  for (int k = 0; k < 3000; ++k)
    ASSERT_TRUE(integrated_bringup::UpdateMomentumObserver(state, w));

  EXPECT_LT(MaxAbsDiff(w.residual_device, tau_ext), 1e-9);
}

// ── The coordinate-order oracle ──────────────────────────────────────────────

// THE case this wiring exists for. The device presents its joints in an order
// the model does not use, so every reorder in UpdateMomentumObserver has to be
// present and pointing the right way for the answer to come back right. A
// mixed-order assembly still produces a finite, smooth residual — it just
// converges to the wrong vector — so the assertion is on WHICH torque comes
// out, per joint, not on whether one does.
TEST(MomentumObserverWiringTest, PermutedDeviceOrderStillRecoversTheTorquePerJoint) {
  rub::RtModelHandle h(ArmModel());

  std::vector<std::string> names = ArmJointNames(h);
  ASSERT_EQ(names.size(), static_cast<std::size_t>(kArmDof));
  // Reverse the model's order — a permutation with no fixed point except the
  // middle pair, so a forgotten reorder cannot pass by accident.
  std::reverse(names.begin(), names.end());
  ASSERT_TRUE(h.SetJointOrder(names));
  ASSERT_TRUE(h.HasJointReorder());

  MomentumObserverWiring w;
  // The wiring pins `names` on ITS OWN handle — nothing about `h` reaches it.
  integrated_bringup::ConfigureMomentumObserverWiring(ArmModel(), names, 0,
                                                      std::vector<double>(kArmDof, 40.0), w);
  ASSERT_TRUE(w.handle->HasJointReorder()) << "wiring did not pin the device order";

  // Posture and torques are now stated in the REVERSED (device) order.
  std::vector<double> q_dev(integrated_bringup::testfx::kArmHome.begin(),
                            integrated_bringup::testfx::kArmHome.end());
  std::reverse(q_dev.begin(), q_dev.end());

  const std::vector<double> g_dev = GravityInDeviceOrder(h, q_dev);
  const std::vector<double> tau_ext(std::begin(kQuasiStaticTauExt),
                                   std::end(kQuasiStaticTauExt));

  rtc::DeviceState dev = FullyReadableDevice();
  for (int i = 0; i < kArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    dev.positions[u] = q_dev[u];
    dev.velocities[u] = 0.0;
    dev.efforts[u] = g_dev[u] - tau_ext[u];
  }

  const rtc::ControllerState state = StateWith(dev);
  for (int k = 0; k < 3000; ++k)
    ASSERT_TRUE(integrated_bringup::UpdateMomentumObserver(state, w));

  EXPECT_LT(MaxAbsDiff(w.residual_device, tau_ext), 1e-9)
      << "residual is not in the caller's joint order";
}

// ── Moving arm: the momentum term actually participates ──────────────────────

// Every case above holds q̇ == 0, which makes p identically constant — and
// [MO-3b] only ever reads p - p_0, so a WRONG momentum would cancel and never
// show. Here the arm is simulated forward under its own dynamics with a constant
// external torque applied, so p genuinely moves and M q̇ has to be right.
//
// Plant and observer discretise differently (explicit Euler forward simulation
// vs the observer's backward-Euler [MO-3b]), so the recovered torque carries an
// O(dt) bias rather than converging to machine precision. The tolerance below is
// that bias, not slack: it is checked against the no-load run in the same test,
// which must stay an order of magnitude smaller.
//
// The horizon is 0.6 s — 36 observer time constants at K_I = 60, and short
// enough that the plant stays in its linear regime. It cannot simply be made
// longer: the arm is gravity-compensated and undamped, so a constant external
// torque accelerates it without bound and an undamped explicit-Euler integrator
// eventually overflows (it did, at 4 s). Adding plant damping instead would put
// an unmodelled force into [MO-1] and the observer would correctly report it as
// external torque, destroying the oracle. Hence: short horizon, and an explicit
// boundedness guard so a future divergence reads as "the plant diverged" rather
// than as a mysterious tolerance failure.
TEST(MomentumObserverWiringTest, RecoversTorqueWhileTheArmIsMoving) {
  // Scaled to the joints' inertias, unlike the quasi-static cases above. The arm
  // here is gravity-compensated and undamped, so a constant torque is a constant
  // acceleration and a distal joint reaches the 50 rad/s guard below in a
  // fraction of a second — long before the observer's answer can be read. The
  // distal components are therefore small: what this case needs is a MOVING
  // arm, not a strongly loaded one. One entry per DOF, and the length is
  // asserted rather than assumed — a list sized for a different arm indexes
  // past its own end here, which reads as a diverged plant rather than as the
  // out-of-bounds read it is.
  const std::vector<double> tau_ext(std::begin(kMovingTauExt), std::end(kMovingTauExt));

  auto run = [&](const std::vector<double>& applied_ext) {
    rub::RtModelHandle h(ArmModel());
    MomentumObserverWiring w;
    integrated_bringup::ConfigureMomentumObserverWiring(ArmModel(), ArmJointNames(h), 0,
                                                        std::vector<double>(kArmDof, 60.0), w);

    std::vector<double> q(integrated_bringup::testfx::kArmHome.begin(),
                          integrated_bringup::testfx::kArmHome.end());
    std::vector<double> v(static_cast<std::size_t>(kArmDof), 0.0);
    std::vector<double> tau_total(static_cast<std::size_t>(kArmDof), 0.0);

    rtc::DeviceState dev = FullyReadableDevice();
    rtc::ControllerState state = StateWith(dev);

    for (int k = 0; k < 300; ++k) {
      // Commanded torque: gravity compensation plus a swing on joint 1. This is
      // what the joint torque sensor reports (tau_m); the external torque is
      // NOT part of it.
      const std::vector<double> g_dev = GravityInDeviceOrder(h, q);
      std::vector<double> tau_m = g_dev;
      tau_m[1] += 4.0 * std::sin(2.0 * 3.14159265358979 * 0.5 * (k * kDt));

      for (int i = 0; i < kArmDof; ++i) {
        const auto u = static_cast<std::size_t>(i);
        tau_total[u] = tau_m[u] + applied_ext[u];
        dev.positions[u] = q[u];
        dev.velocities[u] = v[u];
        dev.efforts[u] = tau_m[u];
      }
      state.devices[0] = dev;
      integrated_bringup::UpdateMomentumObserver(state, w);

      // Plant: q̈ = ABA(q, v, tau_m + tau_ext), explicit Euler.
      h.ComputeForwardDynamics(q, v, tau_total);
      std::vector<double> a_dev(static_cast<std::size_t>(kArmDof), 0.0);
      h.ReorderOutput(h.GetDdq(), std::span<double>(a_dev.data(), a_dev.size()));
      for (int i = 0; i < kArmDof; ++i) {
        const auto u = static_cast<std::size_t>(i);
        v[u] += a_dev[u] * kDt;
        q[u] += v[u] * kDt;
        // Guard the ORACLE, not the observer: past this the forward simulation
        // has left the regime where it models an arm at all.
        if (!std::isfinite(q[u]) || !std::isfinite(v[u]) || std::fabs(v[u]) > 50.0) {
          ADD_FAILURE() << "plant diverged at tick " << k << " joint " << i << " v=" << v[u];
          return w.residual_device;
        }
      }
    }
    return w.residual_device;
  };

  const std::vector<double> zero(kArmDof, 0.0);
  const std::vector<double> r_unloaded = run(zero);
  const std::vector<double> r_loaded = run(tau_ext);

  const double unloaded_err = MaxAbsDiff(r_unloaded, zero);
  const double loaded_err = MaxAbsDiff(r_loaded, tau_ext);

  // The moving, unloaded arm must still report ~nothing: this is the term that
  // a wrong M q̇ or a wrong C^T q̇ would corrupt, and it has no gravity torque
  // to hide behind.
  EXPECT_LT(unloaded_err, 0.05) << "moving unloaded arm produced a residual (err="
                                << unloaded_err << ")";
  EXPECT_LT(loaded_err, 0.25) << "external torque not recovered while moving (err=" << loaded_err
                              << ")";
  EXPECT_GT(MaxAbsDiff(r_loaded, zero), 0.2) << "loaded run produced no residual at all";
}

// ── Layer 2A: payload estimation through the wiring (#135) ──────────────────
//
// These run the FULL path — device lanes → reorder → observer → Jacobian →
// [WRENCH-A]/[MASS-A] — because the failure this layer is most exposed to is
// invisible to a unit test of the estimator: GetFrameJacobian's columns are in
// Pinocchio order while the wiring's residual() accessor is in DEVICE order,
// and pairing those two yields a finite, smooth, wrong wrench. Only a fixture
// whose two orders DIFFER can tell the two wirings apart, so the tests below
// deliberately pin a reversed device order.

namespace {

constexpr char kPayloadFrame[] = "ee_link";

/// A wiring with Layer 2A armed. `joint_names` fixes the DEVICE order, which
/// the tests vary on purpose.
integrated_bringup::MomentumObserverParams PayloadParams(double gain = 10.0) {
  integrated_bringup::MomentumObserverParams p;
  p.has_block = true;
  p.enabled = true;
  p.gains.assign(1, gain);
  p.payload.has_block = true;
  p.payload.enabled = true;
  p.payload.frame = kPayloadFrame;
  p.payload.max_arm_velocity = 1e-3;
  p.payload.max_peripheral_velocity = 1e-4;
  p.payload.settle_time_constants = 5.0;
  p.payload.sigma0 = 1e-3;
  p.payload.lambda_max = 0.05;
  p.payload.min_sigma = 1e-4;
  p.payload.max_fit_error = 1e-6;
  p.payload.min_gravity = 1e-3;
  return p;
}

/// τ_m a resting arm reports while `w` is applied to it at the payload frame.
/// From the equation of motion at rest, g = τ_m + τ_ext with τ_ext = J_pᵀw, so
/// τ_m = g − J_pᵀw. Built in DEVICE order, which is what a device reports.
std::vector<double> MeasuredTorqueUnderPayload(rub::RtModelHandle& h,
                                               const std::vector<double>& q_dev,
                                               const Eigen::Matrix<double, 6, 1>& w) {
  const std::vector<double> g_dev = GravityInDeviceOrder(h, q_dev);

  h.ComputeJacobians(q_dev);
  Eigen::MatrixXd J(6, kArmDof);
  h.GetFrameJacobian(h.GetFrameId(kPayloadFrame), pinocchio::LOCAL_WORLD_ALIGNED, J);
  const Eigen::VectorXd tau_ext_pin = J.transpose() * w;

  std::vector<double> tau_ext_dev(static_cast<std::size_t>(kArmDof), 0.0);
  h.ReorderOutput(tau_ext_pin, std::span<double>(tau_ext_dev.data(), tau_ext_dev.size()));

  std::vector<double> tau(static_cast<std::size_t>(kArmDof), 0.0);
  for (std::size_t i = 0; i < tau.size(); ++i)
    tau[i] = g_dev[i] - tau_ext_dev[i];
  return tau;
}

/// Drive the wiring to steady state under a constant payload and return it.
void SettleUnderPayload(MomentumObserverWiring& wir, const std::vector<double>& q_dev,
                        const std::vector<double>& tau_dev, int ticks = 4000) {
  rtc::DeviceState dev = FullyReadableDevice();
  for (int i = 0; i < kArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    dev.positions[u] = q_dev[u];
    dev.velocities[u] = 0.0;
    dev.efforts[u] = tau_dev[u];
  }
  const rtc::ControllerState st = StateWith(dev);
  for (int k = 0; k < ticks; ++k)
    (void)integrated_bringup::UpdateMomentumObserver(st, wir);
}

}  // namespace

// THE positive control, and the order test in one. A 3 kg payload hanging from
// the arm tip must come back as +3 kg through the whole wiring, with the DEVICE order
// reversed relative to the model so that any implementation pairing the
// device-order residual with the Pinocchio-order Jacobian reports a different
// (and wrong) mass.
TEST(PayloadEstimatorWiring, RecoversHangingMassUnderReversedDeviceOrder) {
  auto model = ArmModel();
  rub::RtModelHandle probe(model);
  std::vector<std::string> names = ArmJointNames(probe);
  std::reverse(names.begin(), names.end());  // device order != model order
  ASSERT_TRUE(probe.SetJointOrder(names));

  std::vector<double> q_dev(static_cast<std::size_t>(kArmDof));
  for (int i = 0; i < kArmDof; ++i)
    q_dev[static_cast<std::size_t>(i)] = integrated_bringup::testfx::kArmHome[static_cast<std::size_t>(i)];

  const double mass = 3.0;
  const Eigen::Vector3d g_world = model->gravity.linear();
  Eigen::Matrix<double, 6, 1> w_true;
  w_true.head<3>() = mass * g_world;
  w_true.tail<3>().setZero();

  const std::vector<double> tau = MeasuredTorqueUnderPayload(probe, q_dev, w_true);

  MomentumObserverWiring wir;
  integrated_bringup::BuildMomentumObserverWiring(PayloadParams(), model, names, 0, wir);
  ASSERT_TRUE(wir.payload_enabled());

  SettleUnderPayload(wir, q_dev, tau);

  ASSERT_TRUE(wir.payload.valid())
      << "reason " << static_cast<int>(wir.payload.invalid_reason());
  EXPECT_NEAR(wir.payload.estimate().mass, mass, 1e-3)
      << "a reversed device order must not change the recovered mass";
  EXPECT_GT(wir.payload.estimate().mass, 0.0);
}

// An unloaded arm must read ~0 kg, not "some mass" — the negative control for
// the same path.
TEST(PayloadEstimatorWiring, UnloadedArmReportsNoPayload) {
  auto model = ArmModel();
  rub::RtModelHandle probe(model);
  const std::vector<std::string> names = ArmJointNames(probe);
  ASSERT_TRUE(probe.SetJointOrder(names));

  std::vector<double> q_dev(static_cast<std::size_t>(kArmDof));
  for (int i = 0; i < kArmDof; ++i)
    q_dev[static_cast<std::size_t>(i)] = integrated_bringup::testfx::kArmHome[static_cast<std::size_t>(i)];
  const std::vector<double> tau = GravityInDeviceOrder(probe, q_dev);  // holding only itself

  MomentumObserverWiring wir;
  integrated_bringup::BuildMomentumObserverWiring(PayloadParams(), model, names, 0, wir);
  SettleUnderPayload(wir, q_dev, tau);

  ASSERT_TRUE(wir.payload.valid());
  EXPECT_NEAR(wir.payload.estimate().mass, 0.0, 1e-3);
}

// D14: a moving hand is an external torque to an arm sub-model that pins the
// hand's joints, so the gate must close on a SECOND device's motion even though
// the arm itself is still. Device 1 exists only in this test.
TEST(PayloadEstimatorWiring, PeripheralDeviceMotionClosesTheGate) {
  auto model = ArmModel();
  rub::RtModelHandle probe(model);
  const std::vector<std::string> names = ArmJointNames(probe);
  ASSERT_TRUE(probe.SetJointOrder(names));

  std::vector<double> q_dev(static_cast<std::size_t>(kArmDof));
  for (int i = 0; i < kArmDof; ++i)
    q_dev[static_cast<std::size_t>(i)] = integrated_bringup::testfx::kArmHome[static_cast<std::size_t>(i)];
  const std::vector<double> tau = GravityInDeviceOrder(probe, q_dev);

  MomentumObserverWiring wir;
  integrated_bringup::BuildMomentumObserverWiring(PayloadParams(), model, names, 0, wir);
  SettleUnderPayload(wir, q_dev, tau);
  ASSERT_TRUE(wir.payload.valid()) << "precondition: gate open with one still device";

  rtc::DeviceState arm = FullyReadableDevice();
  for (int i = 0; i < kArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    arm.positions[u] = q_dev[u];
    arm.efforts[u] = tau[u];
  }
  rtc::ControllerState st = StateWith(arm);
  st.num_devices = 2;
  st.devices[1] = FullyReadableDevice(4);
  st.devices[1].velocities[2] = 0.05;  // a finger moving, arm untouched

  (void)integrated_bringup::UpdateMomentumObserver(st, wir);

  EXPECT_FALSE(wir.payload.valid());
  EXPECT_EQ(wir.payload.invalid_reason(), rtc::estimation::PayloadInvalidReason::kHandMoving);
  EXPECT_NEAR(wir.payload.estimate().mass, 0.0, 1e-3) << "the last estimate must freeze, not zero";
}

TEST(PayloadEstimatorWiring, ArmMotionClosesTheGate) {
  auto model = ArmModel();
  rub::RtModelHandle probe(model);
  const std::vector<std::string> names = ArmJointNames(probe);
  ASSERT_TRUE(probe.SetJointOrder(names));

  std::vector<double> q_dev(static_cast<std::size_t>(kArmDof));
  for (int i = 0; i < kArmDof; ++i)
    q_dev[static_cast<std::size_t>(i)] = integrated_bringup::testfx::kArmHome[static_cast<std::size_t>(i)];
  const std::vector<double> tau = GravityInDeviceOrder(probe, q_dev);

  MomentumObserverWiring wir;
  integrated_bringup::BuildMomentumObserverWiring(PayloadParams(), model, names, 0, wir);
  SettleUnderPayload(wir, q_dev, tau);
  ASSERT_TRUE(wir.payload.valid());

  rtc::DeviceState arm = FullyReadableDevice();
  for (int i = 0; i < kArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    arm.positions[u] = q_dev[u];
    arm.efforts[u] = tau[u];
  }
  arm.velocities[1] = 0.5;
  (void)integrated_bringup::UpdateMomentumObserver(StateWith(arm), wir);

  EXPECT_EQ(wir.payload.invalid_reason(), rtc::estimation::PayloadInvalidReason::kArmMoving);
}

// The residual re-converges from zero after a re-seed, so an estimate read too
// early measures the filter rather than the load.
TEST(PayloadEstimatorWiring, EstimateIsWithheldUntilTheResidualHasSettled) {
  auto model = ArmModel();
  rub::RtModelHandle probe(model);
  const std::vector<std::string> names = ArmJointNames(probe);
  ASSERT_TRUE(probe.SetJointOrder(names));

  std::vector<double> q_dev(static_cast<std::size_t>(kArmDof));
  for (int i = 0; i < kArmDof; ++i)
    q_dev[static_cast<std::size_t>(i)] = integrated_bringup::testfx::kArmHome[static_cast<std::size_t>(i)];
  const std::vector<double> tau = GravityInDeviceOrder(probe, q_dev);

  MomentumObserverWiring wir;
  integrated_bringup::BuildMomentumObserverWiring(PayloadParams(), model, names, 0, wir);

  SettleUnderPayload(wir, q_dev, tau, 10);  // 10 ticks << 5 tau (= 500 at K_I=10, dt=1ms)
  EXPECT_FALSE(wir.payload.valid());
  EXPECT_EQ(wir.payload.invalid_reason(), rtc::estimation::PayloadInvalidReason::kNotConverged);
}

// A frame the model does not carry must be refused loudly: pinocchio returns
// frame 0 (universe) for an unknown name, which would project onto an all-zero
// Jacobian for the life of the run and look like "no payload, ever".
TEST(PayloadEstimatorWiring, UnknownPayloadFrameIsRejected) {
  auto model = ArmModel();
  rub::RtModelHandle probe(model);
  const std::vector<std::string> names = ArmJointNames(probe);

  auto params = PayloadParams();
  params.payload.frame = "no_such_frame";
  MomentumObserverWiring wir;
  EXPECT_THROW(integrated_bringup::BuildMomentumObserverWiring(params, model, names, 0, wir),
               std::invalid_argument);
}

// Absent sub-block ⇒ Layer 1b behaviour exactly: residual, no payload.
TEST(PayloadEstimatorWiring, AbsentBlockLeavesTheObserverUntouched) {
  auto model = ArmModel();
  rub::RtModelHandle probe(model);
  const std::vector<std::string> names = ArmJointNames(probe);

  integrated_bringup::MomentumObserverParams params;
  params.has_block = true;
  params.enabled = true;
  params.gains.assign(1, 10.0);

  MomentumObserverWiring wir;
  integrated_bringup::BuildMomentumObserverWiring(params, model, names, 0, wir);
  EXPECT_TRUE(wir.enabled());
  EXPECT_FALSE(wir.payload_enabled());
}

// ── Layer 2A: the payload estimate must freeze with the residual ────────────
//
// Three ways a tick can stop producing a residual, and all three must take the
// payload estimate down with it. The estimate is a function of `r`, so a tick
// that did not advance `r` carries no new payload evidence — leaving
// payload_valid standing republishes the LAST estimate as if it had been
// measured now, and a consumer polling the topic cannot tell the two apart.
//
// Each way is its own case because they run through different code: the
// caller's explicit hold (E-STOP), the observer rejecting its own tick, and the
// input-lane gate refusing to feed it at all. The third is one branch earlier
// than the second and needs its own hold.

namespace {

/// A wiring settled under a known hanging payload, so `payload.valid()` is true
/// and the estimate is non-trivial before whatever each case does to it.
void ArmWithASettledPayload(MomentumObserverWiring& wir, std::vector<double>& q_dev_out,
                            std::vector<double>& tau_out) {
  auto model = ArmModel();
  rub::RtModelHandle probe(model);
  const std::vector<std::string> names = ArmJointNames(probe);

  q_dev_out.assign(integrated_bringup::testfx::kArmHome.begin(),
                   integrated_bringup::testfx::kArmHome.end());

  Eigen::Matrix<double, 6, 1> w_true;
  w_true.head<3>() = 3.0 * model->gravity.linear();
  w_true.tail<3>().setZero();
  tau_out = MeasuredTorqueUnderPayload(probe, q_dev_out, w_true);

  integrated_bringup::BuildMomentumObserverWiring(PayloadParams(), model, names, 0, wir);
  SettleUnderPayload(wir, q_dev_out, tau_out);
}

/// The settled device, ready for one more tick.
rtc::DeviceState SettledDevice(const std::vector<double>& q_dev, const std::vector<double>& tau) {
  rtc::DeviceState dev = FullyReadableDevice();
  for (int i = 0; i < kArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    dev.positions[u] = q_dev[u];
    dev.velocities[u] = 0.0;
    dev.efforts[u] = tau[u];
  }
  return dev;
}

}  // namespace

TEST(PayloadEstimatorWiring, ExplicitHoldFreezesThePayloadWithTheResidual) {
  MomentumObserverWiring wir;
  std::vector<double> q_dev;
  std::vector<double> tau;
  ArmWithASettledPayload(wir, q_dev, tau);
  ASSERT_TRUE(wir.payload.valid()) << "nothing to freeze";
  const double before = wir.payload.estimate().mass;

  integrated_bringup::HoldMomentumObserver(wir);

  EXPECT_FALSE(wir.payload.valid()) << "an E-STOP tick kept republishing the last payload";
  EXPECT_EQ(wir.payload.invalid_reason(), rtc::estimation::PayloadInvalidReason::kObserverInvalid);
  // Frozen, not cleared: the last value stays readable so a log carries what was
  // there when the hold began. Only the validity flag changes.
  EXPECT_DOUBLE_EQ(wir.payload.estimate().mass, before);
}

// The gate the observer applies to ITSELF — a tick whose inputs it cannot
// integrate. Reached through dt = 0, which UpdateMomentumObserver forwards.
TEST(PayloadEstimatorWiring, AnObserverThatRejectsItsOwnTickTakesThePayloadDown) {
  MomentumObserverWiring wir;
  std::vector<double> q_dev;
  std::vector<double> tau;
  ArmWithASettledPayload(wir, q_dev, tau);
  ASSERT_TRUE(wir.payload.valid());

  rtc::ControllerState st = StateWith(SettledDevice(q_dev, tau));
  st.dt = 0.0;

  EXPECT_FALSE(integrated_bringup::UpdateMomentumObserver(st, wir));
  EXPECT_FALSE(wir.observer.valid());
  EXPECT_FALSE(wir.payload.valid())
      << "the observer rejected the tick but the payload stayed valid";
  EXPECT_EQ(wir.payload.invalid_reason(), rtc::estimation::PayloadInvalidReason::kObserverInvalid);
}

// The lane gate — the device stopped reporting a lane [MO-3a]. This never
// reaches the observer's own Update(), so the payload hold cannot ride along
// with the one down there.
TEST(PayloadEstimatorWiring, AClosedLaneGateTakesThePayloadDownToo) {
  MomentumObserverWiring wir;
  std::vector<double> q_dev;
  std::vector<double> tau;
  ArmWithASettledPayload(wir, q_dev, tau);
  ASSERT_TRUE(wir.payload.valid());

  rtc::DeviceState dev = SettledDevice(q_dev, tau);
  dev.effort_hole_mask = 1ULL << 3;  // one slot stops reporting torque
  const rtc::ControllerState st = StateWith(dev);

  EXPECT_FALSE(integrated_bringup::UpdateMomentumObserver(st, wir));
  EXPECT_FALSE(wir.observer.valid()) << "the lane gate did not freeze the residual";
  EXPECT_FALSE(wir.payload.valid())
      << "the residual froze but the payload kept reporting the last estimate";
  EXPECT_EQ(wir.payload.invalid_reason(), rtc::estimation::PayloadInvalidReason::kObserverInvalid);
}

// on_activate drops the latches. A payload estimate surviving a deactivate /
// activate cycle would have been measured against the previous session's state.
TEST(PayloadEstimatorWiring, ResetRtStateClearsThePayloadEstimate) {
  MomentumObserverWiring wir;
  std::vector<double> q_dev;
  std::vector<double> tau;
  ArmWithASettledPayload(wir, q_dev, tau);
  ASSERT_TRUE(wir.payload.valid());
  ASSERT_GT(wir.payload.estimate().mass, 1.0);

  integrated_bringup::ResetMomentumObserverRtState(wir);

  EXPECT_FALSE(wir.payload.valid());
  EXPECT_DOUBLE_EQ(wir.payload.estimate().mass, 0.0) << "a stale mass survived the reset";
}
