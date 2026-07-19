// test_dof_dynamics.cpp — #172 Phase 1-2 "deeper AC" coverage that the all-Z
// serial_6dof.urdf fixture structurally cannot provide:
//
//   (1) 7-DOF generalization — P / JointPD / CLIK must index nv = 7 gain,
//       target and dynamics buffers without the out-of-bounds read that the
//       former hard-coded 6-DOF storage caused (serial_7dof.urdf, kMaxRobotDOF
//       capacity = 12).
//
//   (2) Non-Z dynamics magnitude — on serial_6dof every joint axis is Z, so
//       g(q) ≡ 0 and C(q,v) ≈ 0 and the existing tests can only assert
//       finiteness. planar_3r.urdf (Y-axis joints) has substantial g and C, so
//       torque OSC / JointPD gravity + Coriolis compensation is checked in
//       *magnitude* against the RNEA identity computed by an independent
//       RtModelHandle (the same identity asserted in rtc_urdf_bridge).
//
//   (3) Device order vs URDF order — the core controllers currently map device
//       channel i → URDF joint i with no name-based reorder (that is Phase 3).
//       This pins that identity contract and proves a device/URDF order
//       mismatch is *observable* (not silently absorbed), anchoring Phase 3.
//
// Reference dynamics come from a freshly-built RtModelHandle (bridge = model
// provider, controller = consumer), never from the controller under test.

#include "rtc_controllers/direct/joint_pd_controller.hpp"
#include "rtc_controllers/direct/operational_space_controller.hpp"
#include "rtc_controllers/indirect/clik_controller.hpp"
#include "rtc_controllers/indirect/p_controller.hpp"
#include "test_urdf_path.hpp"
#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/rt_model_handle.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace rub = rtc_urdf_bridge;

std::string UrdfPath(const std::string& name) {
  return rtc::test::TestUrdfPath(name);
}

rtc::ControllerState MakeState(int nj, double dt = 0.002) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = dt;
  state.devices[0].num_channels = nj;
  state.devices[0].valid = true;
  return state;
}

// Independent model handle for the same fixture — the ground-truth dynamics
// provider the controllers are validated against.
std::unique_ptr<rub::RtModelHandle> MakeHandle(const std::string& urdf) {
  rub::ModelConfig cfg;
  cfg.urdf_path = urdf;
  cfg.root_joint_type = "fixed";
  rub::PinocchioModelBuilder builder(cfg);
  return std::make_unique<rub::RtModelHandle>(builder.GetFullModel());
}

Eigen::VectorXd RefGravity(const std::string& urdf, const std::vector<double>& q) {
  auto h = MakeHandle(urdf);
  h->ComputeGeneralizedGravity(q);
  return h->GetGeneralizedGravity();
}

// A device config that lifts the torque clamp so the end-to-end command equals
// the analytic dynamics term (default fallback would clip ~N·m torques).
std::map<std::string, rtc::DeviceNameConfig> UnclampedTorqueConfig(int nj) {
  rtc::DeviceNameConfig cfg;
  cfg.device_name = "arm";
  rtc::DeviceJointLimits lim;
  lim.max_torque.assign(static_cast<std::size_t>(nj), 1.0e6);
  lim.max_velocity.assign(static_cast<std::size_t>(nj), 1.0e6);
  cfg.joint_limits = lim;
  std::map<std::string, rtc::DeviceNameConfig> m;
  m.emplace("arm", std::move(cfg));
  return m;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// (1) 7-DOF generalization — no out-of-bounds gain/target/dynamics access
// ═══════════════════════════════════════════════════════════════════════════════

TEST(SevenDof, PControllerUsesSeventhGain) {
  // Regression for the former array<double,6> kp: the 7th joint's gain (kp[6])
  // must be a real value, not an out-of-bounds read. Default kp[6] = 80.0.
  rtc::PController ctrl(UrdfPath("serial_7dof.urdf"));
  auto state = MakeState(7);
  (void)ctrl.Compute(state);

  std::array<double, 7> target{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5};
  ctrl.SetDeviceTarget(0, target);
  auto out = ctrl.Compute(state);

  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.devices[0].num_channels, 7);
  // command[6] = pos + kp[6]·error·dt = 0 + 80·0.5·0.002 = 0.08.
  EXPECT_NEAR(out.devices[0].commands[6], 80.0 * 0.5 * 0.002, 1e-9);
  for (std::size_t i = 0; i < 7; ++i) {
    EXPECT_TRUE(std::isfinite(out.devices[0].commands[i])) << "ch " << i;
  }
}

TEST(SevenDof, PControllerLoadConfigRequiresSevenGains) {
  // The YAML kp length must equal the model DOF (nv = 7); a 6-length kp that
  // silently passed before is now a fail-fast configuration error.
  rtc::PController ctrl(UrdfPath("serial_7dof.urdf"));
  YAML::Node six = YAML::Load("kp: [1, 2, 3, 4, 5, 6]");
  EXPECT_THROW(ctrl.LoadConfig(six), std::runtime_error);

  rtc::PController ok(UrdfPath("serial_7dof.urdf"));
  YAML::Node seven = YAML::Load("kp: [1, 2, 3, 4, 5, 6, 7]");
  EXPECT_NO_THROW(ok.LoadConfig(seven));
  const auto g = ok.get_gains();
  EXPECT_DOUBLE_EQ(g.kp[6], 7.0);
}

TEST(SevenDof, JointPdComputesSevenChannels) {
  rtc::JointPDController ctrl(UrdfPath("serial_7dof.urdf"));
  auto state = MakeState(7);
  state.devices[0].positions = {0.1, 0.2, -0.3, 0.4, -0.2, 0.3, -0.1};
  (void)ctrl.Compute(state);

  std::array<double, 7> target{0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4};
  ctrl.SetDeviceTarget(0, target);
  auto out = ctrl.Compute(state);

  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);
  EXPECT_EQ(out.devices[0].num_channels, 7);
  for (std::size_t i = 0; i < 7; ++i) {
    EXPECT_TRUE(std::isfinite(out.devices[0].commands[i])) << "ch " << i;
  }
}

TEST(SevenDof, ClikComputesSevenChannels) {
  // 7-DOF redundant arm: CLIK's null-space projection spans nv = 7. Output must
  // stay finite across the whole redundant joint vector.
  rtc::ClikController::Gains gains;
  gains.enable_null_space = true;
  rtc::ClikController ctrl(UrdfPath("serial_7dof.urdf"), gains);
  auto state = MakeState(7);
  state.devices[0].positions = {0.1, 0.4, -0.2, 0.6, 0.1, -0.3, 0.2};
  (void)ctrl.Compute(state);
  auto out = ctrl.Compute(state);

  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.command_type, rtc::CommandType::kPosition);
  for (std::size_t i = 0; i < 7; ++i) {
    EXPECT_TRUE(std::isfinite(out.devices[0].commands[i])) << "ch " << i;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// (2) Non-Z dynamics — RNEA identity anchor + controller magnitude checks
// ═══════════════════════════════════════════════════════════════════════════════

TEST(PlanarDynamics, RneaIdentityGivesNonTrivialGravityAndCoriolis) {
  // Anchor: on the Y-axis planar arm the model's g(q) and C(q,v)·v are both
  // substantial, and g(q) == RNEA(q,0,0), nle(q,v) == RNEA(q,v,0). This is what
  // makes the controller magnitude assertions below meaningful (serial_6dof's
  // all-Z axes make both terms ~0, so they can only be finiteness-checked).
  const std::string urdf = UrdfPath("planar_3r.urdf");
  auto h = MakeHandle(urdf);
  ASSERT_EQ(h->nv(), 3);

  const std::vector<double> q_flat{0.0, 0.0, 0.0};  // horizontal → peak gravity
  const std::vector<double> zero{0.0, 0.0, 0.0};

  h->ComputeGeneralizedGravity(q_flat);
  const Eigen::VectorXd g = h->GetGeneralizedGravity();
  h->ComputeInverseDynamics(q_flat, zero, zero);
  const Eigen::VectorXd tau_grav = h->GetTau();
  EXPECT_LT((g - tau_grav).norm(), 1e-9) << "RNEA(q,0,0) must equal g(q)";
  EXPECT_GT(g.norm(), 1.0) << "planar arm gravity must be substantial (non-Z axes)";

  const std::vector<double> q_bent{0.3, -0.6, 0.4};
  const std::vector<double> v{0.5, -0.4, 0.6};
  h->ComputeGeneralizedGravity(q_bent);
  const Eigen::VectorXd g_bent = h->GetGeneralizedGravity();
  h->ComputeNonLinearEffects(q_bent, v);
  const Eigen::VectorXd nle = h->GetNonLinearEffects();
  h->ComputeInverseDynamics(q_bent, v, zero);
  const Eigen::VectorXd tau_nle = h->GetTau();
  EXPECT_LT((nle - tau_nle).norm(), 1e-9) << "RNEA(q,v,0) must equal nle(q,v)";
  const Eigen::VectorXd cv = nle - g_bent;  // Coriolis/centrifugal C(q,v)·v
  EXPECT_GT(cv.norm(), 1e-3) << "Coriolis term must be non-trivial at a bent config";
}

TEST(PlanarDynamics, JointPdGravityMatchesModelMagnitude) {
  const std::string urdf = UrdfPath("planar_3r.urdf");
  const std::vector<double> q{0.2, -0.5, 0.3};
  const Eigen::VectorXd g_ref = RefGravity(urdf, q);
  ASSERT_GT(g_ref.norm(), 0.1) << "fixture must produce real gravity";

  rtc::JointPDController::Gains gains;
  gains.enable_gravity_compensation = true;
  gains.enable_coriolis_compensation = false;
  rtc::JointPDController ctrl(urdf, gains);
  ctrl.SetDeviceNameConfigs(UnclampedTorqueConfig(3));  // lift torque clamp

  auto state = MakeState(3);
  for (std::size_t i = 0; i < 3; ++i)
    state.devices[0].positions[i] = q[i];
  (void)ctrl.Compute(state);  // seed hold target (zero PD error next tick)
  auto out = ctrl.Compute(state);

  EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);
  // Accessor mirrors g(q) exactly (pre-clamp).
  const auto grav = ctrl.gravity_torques();
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(grav[i], g_ref[static_cast<Eigen::Index>(i)], 1e-6) << "grav[" << i << "]";
  }
  // At the seeded hold pose the whole torque command IS the gravity term.
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], g_ref[static_cast<Eigen::Index>(i)], 1e-6)
        << "cmd[" << i << "]";
  }
}

TEST(PlanarDynamics, JointPdCoriolisMatchesModelMagnitude) {
  // Gravity OFF, Coriolis ON, at the seeded hold pose with a non-zero velocity:
  // PD error is zero, so the entire torque command equals C(q,v)·v.
  const std::string urdf = UrdfPath("planar_3r.urdf");
  const std::vector<double> q{0.3, -0.6, 0.4};
  const std::vector<double> v{0.5, -0.4, 0.6};

  auto h = MakeHandle(urdf);
  h->ComputeGeneralizedGravity(q);
  const Eigen::VectorXd g_ref = h->GetGeneralizedGravity();
  h->ComputeNonLinearEffects(q, v);
  const Eigen::VectorXd cv_ref = h->GetNonLinearEffects() - g_ref;
  ASSERT_GT(cv_ref.norm(), 1e-3) << "fixture must produce real Coriolis";

  rtc::JointPDController::Gains gains;
  gains.enable_gravity_compensation = false;
  gains.enable_coriolis_compensation = true;
  rtc::JointPDController ctrl(urdf, gains);
  ctrl.SetDeviceNameConfigs(UnclampedTorqueConfig(3));

  auto state = MakeState(3);
  for (std::size_t i = 0; i < 3; ++i) {
    state.devices[0].positions[i] = q[i];
    state.devices[0].velocities[i] = v[i];
  }
  (void)ctrl.Compute(state);  // seed hold target
  auto out = ctrl.Compute(state);

  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], cv_ref[static_cast<Eigen::Index>(i)], 1e-6)
        << "cmd[" << i << "]";
  }
}

TEST(PlanarDynamics, OscHoldTorqueEqualsGravity) {
  // Torque OSC at the self-seeded hold pose (zero task error, zero velocity,
  // zero posture torque with default null_kp = 0): τ = Jᵀ·0 + h + Nᵀ·0 = g(q).
  const std::string urdf = UrdfPath("planar_3r.urdf");
  const std::vector<double> q{0.3, -0.6, 0.4};
  const Eigen::VectorXd g_ref = RefGravity(urdf, q);
  ASSERT_GT(g_ref.norm(), 0.1);

  rtc::OperationalSpaceController::Gains gains;
  rtc::OperationalSpaceController ctrl(urdf, gains);
  ctrl.SetDeviceNameConfigs(UnclampedTorqueConfig(3));

  auto state = MakeState(3);
  for (std::size_t i = 0; i < 3; ++i)
    state.devices[0].positions[i] = q[i];
  (void)ctrl.Compute(state);  // seeds task target from current FK pose
  auto out = ctrl.Compute(state);

  EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], g_ref[static_cast<Eigen::Index>(i)], 1e-3)
        << "cmd[" << i << "] should be the gravity compensation";
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// (3) Device order vs URDF order — identity contract is observable (Phase 3 anchor)
// ═══════════════════════════════════════════════════════════════════════════════

TEST(JointOrder, GravityIsChannelOrderSensitive) {
  // The controllers read device channel i as URDF joint i — there is no
  // name-based reorder yet (Phase 3). On a per-joint-asymmetric-gravity fixture
  // this makes a device/URDF order mismatch observable: a permuted joint config
  // yields the model gravity AT THE PERMUTED CONFIG, not the original gravity
  // permuted. So feeding channels in the wrong order would produce detectably
  // wrong torque — which is exactly what Phase 3's reorder must fix.
  const std::string urdf = UrdfPath("planar_3r.urdf");

  const std::array<double, 3> qa{0.4, -0.3, 0.6};
  const std::array<double, 3> qb{0.6, -0.3, 0.4};  // swap channels 0 and 2

  auto gravity_of = [&](const std::array<double, 3>& cfg) {
    rtc::JointPDController::Gains g;
    g.enable_gravity_compensation = true;
    rtc::JointPDController ctrl(urdf, g);
    auto st = MakeState(3);
    for (std::size_t i = 0; i < 3; ++i)
      st.devices[0].positions[i] = cfg[i];
    (void)ctrl.Compute(st);
    (void)ctrl.Compute(st);
    return ctrl.gravity_torques();
  };

  const auto ga = gravity_of(qa);
  const auto gb = gravity_of(qb);

  // Identity contract: gravity for the permuted config equals the model's g at
  // that permuted config (channels consumed positionally, in URDF order).
  const Eigen::VectorXd ref_b = RefGravity(urdf, {qb[0], qb[1], qb[2]});
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(gb[i], ref_b[static_cast<Eigen::Index>(i)], 1e-6) << "gb[" << i << "]";
  }
  // The permutation is NOT gravity-symmetric → the mismatch is detectable
  // (a name-based reorder would be required to make the two agree).
  const double delta = std::abs(ga[0] - gb[0]) + std::abs(ga[2] - gb[2]);
  EXPECT_GT(delta, 1e-2) << "permuting joint order must change g → mismatch observable";
}
