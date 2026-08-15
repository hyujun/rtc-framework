// Wiring for #135 Layer 1 — the layer that turns a DeviceState plus a Pinocchio
// handle into the four nv-vectors the momentum observer consumes.
//
// Two kinds of case here, and the second is why this file uses a real URDF at
// all. The gate cases are pure logic. The dynamics cases run the observer
// against an ACTUAL ur5e arm sub-model, because the failure this wiring exists
// to prevent — mixing the model's internal joint order with the device order —
// produces a finite, smooth, wrong residual that no gate and no NaN check would
// catch. Only an end-to-end oracle ("a known external torque comes back out")
// separates a correct assembly from a plausible one.
#include "integrated_bringup/support/momentum_observer_wiring.hpp"

#include "ur5e_p1b_test_fixture.hpp"

#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace rub = rtc_urdf_bridge;
using integrated_bringup::MomentumObserverInputsReadable;
using integrated_bringup::MomentumObserverWiring;

constexpr int kArmDof = integrated_bringup::testfx::kUr5eArmDof;  // 6
constexpr double kDt = 0.001;

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
  return integrated_bringup::testfx::SharedUr5eP1bBuilder()->GetReducedModel("ur5e");
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
  const std::vector<double> gains(kArmDof, 30.0);
  MomentumObserverWiring w;

  EXPECT_THROW(integrated_bringup::ConfigureMomentumObserverWiring(h, -1, kArmDof, gains, w),
               std::invalid_argument);
  EXPECT_THROW(integrated_bringup::ConfigureMomentumObserverWiring(
                   h, rtc::ControllerState::kMaxDevices, kArmDof, gains, w),
               std::invalid_argument);
  // dof must equal nv — truncating would observe part of the arm and report the
  // rest of it as external torque.
  EXPECT_THROW(integrated_bringup::ConfigureMomentumObserverWiring(h, 0, kArmDof - 1, gains, w),
               std::invalid_argument);
  EXPECT_FALSE(w.configured);

  EXPECT_NO_THROW(integrated_bringup::ConfigureMomentumObserverWiring(h, 0, kArmDof, gains, w));
  EXPECT_TRUE(w.configured);
  EXPECT_EQ(w.dof, kArmDof);
  EXPECT_EQ(h.nv(), kArmDof);
}

// ── AC5 glue half — a closed gate freezes the observer ───────────────────────

TEST(MomentumObserverWiringTest, ClosedLaneGateHoldsTheObserverAndFreezesTheResidual) {
  rub::RtModelHandle h(ArmModel());
  MomentumObserverWiring w;
  integrated_bringup::ConfigureMomentumObserverWiring(h, 0, kArmDof,
                                                      std::vector<double>(kArmDof, 40.0), w);

  const std::vector<double> q_dev(integrated_bringup::testfx::kUr5eHome.begin(),
                                  integrated_bringup::testfx::kUr5eHome.end());
  const std::vector<double> g_dev = GravityInDeviceOrder(h, q_dev);
  const std::vector<double> tau_ext{2.0, -3.0, 1.0, 0.5, -0.25, 0.75};

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
  integrated_bringup::ConfigureMomentumObserverWiring(h, 0, kArmDof,
                                                      std::vector<double>(kArmDof, 40.0), w);

  const std::vector<double> q_dev(integrated_bringup::testfx::kUr5eHome.begin(),
                                  integrated_bringup::testfx::kUr5eHome.end());
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
  integrated_bringup::ConfigureMomentumObserverWiring(h, 0, kArmDof,
                                                      std::vector<double>(kArmDof, 40.0), w);

  const std::vector<double> q_dev(integrated_bringup::testfx::kUr5eHome.begin(),
                                  integrated_bringup::testfx::kUr5eHome.end());
  const std::vector<double> g_dev = GravityInDeviceOrder(h, q_dev);
  const std::vector<double> tau_ext{2.0, -3.0, 1.0, 0.5, -0.25, 0.75};

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
  integrated_bringup::ConfigureMomentumObserverWiring(h, 0, kArmDof,
                                                      std::vector<double>(kArmDof, 40.0), w);

  // Posture and torques are now stated in the REVERSED (device) order.
  std::vector<double> q_dev(integrated_bringup::testfx::kUr5eHome.begin(),
                            integrated_bringup::testfx::kUr5eHome.end());
  std::reverse(q_dev.begin(), q_dev.end());

  const std::vector<double> g_dev = GravityInDeviceOrder(h, q_dev);
  const std::vector<double> tau_ext{2.0, -3.0, 1.0, 0.5, -0.25, 0.75};

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
  // acceleration: 0.75 N·m on wrist_3 (~2e-3 kg·m²) reaches 50 rad/s in 0.12 s
  // and the integrator leaves the arm's regime long before the observer's answer
  // can be read. The distal components are therefore small — what this case
  // needs is a MOVING arm, not a strongly loaded one.
  const std::vector<double> tau_ext{0.5, -0.8, 0.3, 0.05, -0.03, 0.02};

  auto run = [&](const std::vector<double>& applied_ext) {
    rub::RtModelHandle h(ArmModel());
    MomentumObserverWiring w;
    integrated_bringup::ConfigureMomentumObserverWiring(h, 0, kArmDof,
                                                        std::vector<double>(kArmDof, 60.0), w);

    std::vector<double> q(integrated_bringup::testfx::kUr5eHome.begin(),
                          integrated_bringup::testfx::kUr5eHome.end());
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
