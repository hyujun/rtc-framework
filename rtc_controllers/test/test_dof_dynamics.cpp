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

#include "rtc_controllers/direct/cascaded_compliance_controller.hpp"
#include "rtc_controllers/direct/joint_pd_controller.hpp"
#include "rtc_controllers/direct/operational_space_controller.hpp"
#include "rtc_controllers/direct/task_impedance_controller.hpp"
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

// Independent gravity reference via the RNEA identity g(q) == RNEA(q, 0, 0).
// Deliberately a DIFFERENT code path from the controller's
// ComputeGeneralizedGravity so a shared sign/convention bug cannot hide behind
// a circular comparison.
Eigen::VectorXd RefGravity(const std::string& urdf, const std::vector<double>& q) {
  auto h = MakeHandle(urdf);
  const std::vector<double> zero(q.size(), 0.0);
  h->ComputeInverseDynamics(q, zero, zero);
  return h->GetTau();
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

TEST(SevenDof, JointPdSeventhChannelGainsAndGravity) {
  // Concrete OOB guards on the 7th channel (index 6), stronger than finiteness:
  //   (a) LoadConfig writes and get_gains() reads kp[6]/kd[6];
  //   (b) gravity compensation reproduces the model's g(q) at every index
  //       including 6. serial_7dof has mixed Z/Y axes so g(q) is non-zero.
  const std::string urdf = UrdfPath("serial_7dof.urdf");

  rtc::JointPDController cfgctrl(urdf);
  cfgctrl.LoadConfig(
      YAML::Load("kp: [1, 2, 3, 4, 5, 6, 7]\nkd: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]\n"
                 "enable_gravity_compensation: true"));
  EXPECT_DOUBLE_EQ(cfgctrl.get_gains().kp[6], 7.0);
  EXPECT_DOUBLE_EQ(cfgctrl.get_gains().kd[6], 0.7);

  const std::vector<double> q{0.1, 0.5, -0.3, 0.4, -0.2, 0.3, -0.1};
  const Eigen::VectorXd g_ref = RefGravity(urdf, q);
  ASSERT_GT(g_ref.norm(), 0.1) << "mixed-axis 7-DOF must load gravity";

  rtc::JointPDController::Gains gains;
  gains.enable_gravity_compensation = true;
  rtc::JointPDController ctrl(urdf, gains);
  ctrl.SetDeviceNameConfigs(UnclampedTorqueConfig(7));
  auto state = MakeState(7);
  for (std::size_t i = 0; i < 7; ++i)
    state.devices[0].positions[i] = q[i];
  (void)ctrl.Compute(state);  // seed hold target (zero PD error next tick)
  auto out = ctrl.Compute(state);

  EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);
  EXPECT_EQ(out.devices[0].num_channels, 7);
  const auto grav = ctrl.gravity_torques();
  for (std::size_t i = 0; i < 7; ++i) {
    EXPECT_NEAR(grav[i], g_ref[static_cast<Eigen::Index>(i)], 1e-6) << "grav[" << i << "]";
    EXPECT_NEAR(out.devices[0].commands[i], g_ref[static_cast<Eigen::Index>(i)], 1e-6)
        << "cmd[" << i << "]";
  }
}

TEST(SevenDof, ClikSevenChannelsHoldAtCurrent) {
  // 7-DOF redundant arm: CLIK's null-space projection spans nv = 7. Stronger
  // than finiteness — at the self-seeded hold (task target = current FK pose)
  // the IK command must stay near the CURRENT joint angles across all 7
  // channels, so a wrong value at the 7th channel (index 6) would be caught.
  rtc::ClikController::Gains gains;
  gains.enable_null_space = true;
  rtc::ClikController ctrl(UrdfPath("serial_7dof.urdf"), gains);
  auto state = MakeState(7);
  const std::array<double, 7> q{0.1, 0.4, -0.2, 0.6, 0.1, -0.3, 0.2};
  for (std::size_t i = 0; i < 7; ++i)
    state.devices[0].positions[i] = q[i];
  (void)ctrl.Compute(state);  // seed hold target from current pose
  auto out = ctrl.Compute(state);

  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.command_type, rtc::CommandType::kPosition);
  EXPECT_EQ(out.devices[0].num_channels, 7);
  for (std::size_t i = 0; i < 7; ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], q[i], 0.05) << "ch " << i;
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
  // NOTE: at hold F = Λ·a_task = 0, so this checks the gravity term only; the
  // operational-space mapping (Λ, M⁻¹Jᵀ, JᵀF) is exercised by the companion
  // OscTaskForceMovesBeyondGravity test below (non-zero task error → F ≠ 0).
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

TEST(PlanarDynamics, OscTaskForceMovesBeyondGravity) {
  // Drives the operational-space law itself (not just gravity-at-rest): with a
  // persistent task-space position error the task force F = Λ·a_task ≠ 0, so
  // τ = Jᵀ·F + g must differ from pure gravity g(q) by the Jᵀ·F term. This
  // exercises Λ⁻¹, the M⁻¹Jᵀ Cholesky solve and the JᵀF mapping — all of which
  // are exactly zero at the hold pose and would be masked by the degenerate-M
  // τ = h fallback. null_kd is inert here (nv = 3 ≤ 6 → null-space gated off).
  const std::string urdf = UrdfPath("planar_3r.urdf");
  const std::vector<double> q{0.3, -0.6, 0.4};  // bent → well-conditioned J
  const Eigen::VectorXd g_ref = RefGravity(urdf, q);

  rtc::OperationalSpaceController::Gains gains;
  gains.trajectory_speed = 1.0;
  rtc::OperationalSpaceController ctrl(urdf, gains);
  ctrl.SetDeviceNameConfigs(UnclampedTorqueConfig(3));

  auto state = MakeState(3);
  for (std::size_t i = 0; i < 3; ++i)
    state.devices[0].positions[i] = q[i];
  (void)ctrl.Compute(state);  // seed task target from current FK pose
  auto out0 = ctrl.Compute(state);

  // Offset the task target in x; the state stays fixed so a persistent task
  // error builds up as the trajectory advances toward the (unreachable) goal.
  std::array<double, 6> target{};
  for (std::size_t i = 0; i < 6; ++i)
    target[i] = out0.actual_task_positions[i];
  target[0] += 0.15;
  ctrl.SetDeviceTarget(0, target);

  rtc::ControllerOutput out{};
  for (int k = 0; k < 40; ++k)
    out = ctrl.Compute(state);

  double dnorm = 0.0;
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isfinite(out.devices[0].commands[i])) << "cmd[" << i << "]";
    dnorm += std::abs(out.devices[0].commands[i] - g_ref[static_cast<Eigen::Index>(i)]);
  }
  EXPECT_GT(dnorm, 1e-2)
      << "with a task error τ = JᵀF + g must exceed pure gravity (operational-space law is live)";
}

// ═══════════════════════════════════════════════════════════════════════════════
// (3) Device order vs URDF order — identity contract is observable (Phase 3 anchor)
// ═══════════════════════════════════════════════════════════════════════════════

TEST(JointOrder, GravityIsChannelOrderSensitive) {
  // Pins the CURRENT contract: the controllers consume device channel i as URDF
  // joint i, positionally, with no name-based reorder. Two facts are asserted:
  //   (1) gravity for a config equals the model's g at that config (positional
  //       consumption), and
  //   (2) the fixture's gravity is order-asymmetric (permuting joints changes g).
  // Together these mean that IF a device delivered joints in a different order
  // than the URDF, the dynamics would be wrong — i.e. joint order matters.
  //
  // This test injects NO device joint_state_names, so no reorder map is built
  // (HasJointReorder()==false) and the controller keeps consuming channels
  // positionally — the no-regression path. The Phase 3 hard-correctness case
  // (permuted device order → output reordered back to device order) is exercised
  // separately in GravityReorderedToDeviceOrder / OscTorqueReorderedToDeviceOrder
  // below; assertion (2) here establishes the order-asymmetry precondition they
  // rely on (a symmetric fixture could pass reorder tests trivially).
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

// ═══════════════════════════════════════════════════════════════════════════════
// (3b) Phase 3 (A2) hard-correctness — permuted device order → device-order output
// ═══════════════════════════════════════════════════════════════════════════════
//
// The device declares joint_state_names in an order that is a genuine permutation
// of the URDF/Pinocchio order [joint_1, joint_2, joint_3]. RtModelHandle::
// SetJointOrder (wired into every controller's OnDeviceConfigsSet) must then
// (a) make the model consume device-order q/v correctly and (b) scatter each
// model-derived torque term back to the device channel order, so channel i always
// drives its own physical joint. Without the reorder these outputs would be the
// Pinocchio-order value at index i (the characterization test above) — which
// differs here because the fixture's dynamics are order-asymmetric.
//
// COVERAGE NOTE: these exercise the direct-torque reorder paths (JointPD gravity
// + Coriolis via Reorder{Input,Output}; OSC main τ via ReorderOutput). The OSC/
// CLIK *null-space* input-gather (ReorderInput on tau0_/null_err_) is gated on
// nv > 6 (OSC) / enable_null_space (CLIK); the 3-DOF planar fixture does not
// activate the OSC gate, so that specific path is covered by the ReorderInput
// round-trip unit test in rtc_urdf_bridge, not end-to-end here.

namespace {

// Device order = the 3-cycle [joint_3, joint_1, joint_2] of the URDF order.
// device channel → Pinocchio index:  0→2 (joint_3), 1→0 (joint_1), 2→1 (joint_2).
const std::array<const char*, 3> kPermutedNames{"joint_3", "joint_1", "joint_2"};
constexpr std::array<int, 3> kDeviceToPin{2, 0, 1};

std::map<std::string, rtc::DeviceNameConfig> PermutedDeviceConfig() {
  rtc::DeviceNameConfig cfg;
  cfg.device_name = "arm";
  cfg.joint_state_names = {kPermutedNames[0], kPermutedNames[1], kPermutedNames[2]};
  rtc::DeviceJointLimits lim;
  lim.max_torque.assign(3, 1.0e6);
  lim.max_velocity.assign(3, 1.0e6);
  cfg.joint_limits = lim;
  std::map<std::string, rtc::DeviceNameConfig> m;
  m.emplace("arm", std::move(cfg));
  return m;
}

// Independent Coriolis reference: C(q,v)·v = RNEA(q,v,0) − RNEA(q,0,0) (Pinocchio
// order). A different path from the controller's ComputeCoriolisMatrix so a shared
// convention bug cannot hide behind a circular comparison.
Eigen::VectorXd RefCoriolis(const std::string& urdf, const std::vector<double>& q,
                            const std::vector<double>& v) {
  auto h = MakeHandle(urdf);
  const std::vector<double> zero(q.size(), 0.0);
  h->ComputeInverseDynamics(q, v, zero);
  const Eigen::VectorXd nle = h->GetTau();  // C·v + g
  h->ComputeInverseDynamics(q, zero, zero);
  const Eigen::VectorXd g = h->GetTau();  // g
  return nle - g;
}

}  // namespace

TEST(JointOrder, GravityReorderedToDeviceOrder) {
  // JointPD gravity under a permuted device order: gravity_torques() must come
  // back in DEVICE channel order — the hard-correctness upgrade of the
  // characterization test above (which pins the no-reorder path).
  const std::string urdf = UrdfPath("planar_3r.urdf");
  const std::array<double, 3> q_pin{0.4, -0.3, 0.6};  // Pinocchio order [j1,j2,j3]
  const Eigen::VectorXd g_pin = RefGravity(urdf, {q_pin[0], q_pin[1], q_pin[2]});

  rtc::JointPDController::Gains g;
  g.enable_gravity_compensation = true;
  rtc::JointPDController ctrl(urdf, g);
  ctrl.SetDeviceNameConfigs(PermutedDeviceConfig());

  auto st = MakeState(3);
  for (std::size_t i = 0; i < 3; ++i)
    st.devices[0].positions[i] = q_pin[static_cast<std::size_t>(kDeviceToPin[i])];
  (void)ctrl.Compute(st);
  (void)ctrl.Compute(st);
  const auto gtorq = ctrl.gravity_torques();

  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(gtorq[i], g_pin[kDeviceToPin[i]], 1e-6)
        << "device channel " << i << " (" << kPermutedNames[i] << ")";
  }
  // Guard against a vacuous pass: reorder actually changed channel 0 away from
  // the positional (Pinocchio-order) value. Needs an order-asymmetric fixture.
  EXPECT_GT(std::abs(gtorq[0] - g_pin[0]), 1e-2)
      << "channel 0 must receive joint_3's gravity, not joint_1's (positional)";
}

TEST(JointOrder, CoriolisReorderedToDeviceOrder) {
  // JointPD Coriolis under a permuted device order exercises the in-controller
  // ReorderInput (gather v to Pinocchio for C·v) + ReorderOutput (scatter result
  // to device order). Zero PD gains so the torque command equals C·v alone.
  const std::string urdf = UrdfPath("planar_3r.urdf");
  const std::array<double, 3> q_pin{0.4, -0.3, 0.6};
  const std::array<double, 3> v_pin{0.5, -0.7, 0.2};
  const Eigen::VectorXd cor_pin =
      RefCoriolis(urdf, {q_pin[0], q_pin[1], q_pin[2]}, {v_pin[0], v_pin[1], v_pin[2]});

  rtc::JointPDController::Gains g;
  g.enable_gravity_compensation = false;
  g.enable_coriolis_compensation = true;
  for (auto& k : g.kp)
    k = 0.0;
  for (auto& k : g.kd)
    k = 0.0;
  rtc::JointPDController ctrl(urdf, g);  // command_type defaults to kTorque
  ctrl.SetDeviceNameConfigs(PermutedDeviceConfig());

  auto st = MakeState(3);
  for (std::size_t i = 0; i < 3; ++i) {
    st.devices[0].positions[i] = q_pin[static_cast<std::size_t>(kDeviceToPin[i])];
    st.devices[0].velocities[i] = v_pin[static_cast<std::size_t>(kDeviceToPin[i])];
  }
  (void)ctrl.Compute(st);
  auto out = ctrl.Compute(st);

  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], cor_pin[kDeviceToPin[i]], 1e-6)
        << "device channel " << i << " (" << kPermutedNames[i] << ")";
  }
  EXPECT_GT(std::abs(out.devices[0].commands[0] - cor_pin[0]), 1e-2)
      << "channel 0 must receive joint_3's Coriolis term, not joint_1's (positional)";
}

TEST(JointOrder, OscTorqueReorderedToDeviceOrder) {
  // OSC hold torque (task error ≈ 0 → τ = h = g at rest) under a permuted device
  // order: the emitted torque command must be g mapped to DEVICE channel order,
  // proving the main τ = JᵀF + h path is scattered by ReorderOutput.
  const std::string urdf = UrdfPath("planar_3r.urdf");
  const std::array<double, 3> q_pin{0.3, -0.6, 0.4};
  const Eigen::VectorXd g_pin = RefGravity(urdf, {q_pin[0], q_pin[1], q_pin[2]});
  ASSERT_GT(g_pin.norm(), 0.1);

  rtc::OperationalSpaceController::Gains gains;
  rtc::OperationalSpaceController ctrl(urdf, gains);
  ctrl.SetDeviceNameConfigs(PermutedDeviceConfig());

  auto st = MakeState(3);
  for (std::size_t i = 0; i < 3; ++i)
    st.devices[0].positions[i] = q_pin[static_cast<std::size_t>(kDeviceToPin[i])];
  (void)ctrl.Compute(st);  // seeds task target from current FK pose
  auto out = ctrl.Compute(st);

  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], g_pin[kDeviceToPin[i]], 1e-3)
        << "device channel " << i << " (" << kPermutedNames[i] << ")";
  }
  EXPECT_GT(std::abs(out.devices[0].commands[0] - g_pin[0]), 1e-2)
      << "channel 0 must receive joint_3's gravity, not joint_1's (positional)";
}

// ═══════════════════════════════════════════════════════════════════════════════
// (3c) Phase 3 (A2) hard-correctness — permuted device order → task twist ν
// ═══════════════════════════════════════════════════════════════════════════════
//
// The three tests above cover terms formed in DEVICE order and scattered out
// (gravity, Coriolis, τ). The opposite direction — a device-order vector that
// must be GATHERED before it meets a Pinocchio-order matrix — has one consumer
// nothing pinned: the task twist ν = J·q̇. GetFrameJacobian returns J in
// Pinocchio COLUMN order (ComputeJacobians gathers q on the way in), so feeding
// it the device-order q̇ pairs every column with another joint's velocity. The
// product stays finite and the pose error is untouched, so the only symptom is a
// wrong −K_d·ν damping torque — silent, and worse the faster the arm moves.
//
// Both §6.2-law controllers are pinned because they share the expression
// (compliance/impedance_law.hpp). At the seeding tick e ≡ 0 (X_d := the measured
// pose) and ν_d = 0, so with K_p = 0 the emitted command collapses to one
// analytic term:  τ_dev[i] = (−Jᵀ K_d J q̇ + g)[pin(i)].

namespace {

constexpr double kRefKd = 12.0;  // one damping value on all six task axes

// −Jᵀ K_d J q̇ + g in PINOCCHIO order, from an independent handle.
Eigen::VectorXd RefDampingPlusGravity(const std::string& urdf, const std::vector<double>& q_pin,
                                      const std::vector<double>& qd_pin) {
  auto h = MakeHandle(urdf);
  const auto nv = static_cast<Eigen::Index>(q_pin.size());
  h->ComputeJacobians(q_pin);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, nv);
  h->GetFrameJacobian(static_cast<pinocchio::FrameIndex>(h->GetModel().nframes - 1),
                      pinocchio::LOCAL_WORLD_ALIGNED, J);
  Eigen::Map<const Eigen::VectorXd> qd(qd_pin.data(), nv);
  const Eigen::Matrix<double, 6, 1> f_task = -kRefKd * (J * qd);
  return J.transpose() * f_task + RefGravity(urdf, q_pin);
}

// Pinocchio-order q̇ (asymmetric, so a permutation is observable) and its
// device-order presentation.
const std::vector<double> kQPin{0.3, -0.6, 0.4};
const std::vector<double> kQdPin{0.7, -0.4, 0.9};

void FillPermutedState(rtc::ControllerState& st) {
  for (std::size_t i = 0; i < 3; ++i) {
    const auto p = static_cast<std::size_t>(kDeviceToPin[i]);
    st.devices[0].positions[i] = kQPin[p];
    st.devices[0].velocities[i] = kQdPin[p];
  }
}

}  // namespace

TEST(JointOrder, TaskImpedanceTaskTwistUsesReorderedVelocity) {
  const std::string urdf = UrdfPath("planar_3r.urdf");
  const Eigen::VectorXd ref = RefDampingPlusGravity(urdf, kQPin, kQdPin);

  rtc::TaskImpedanceController::Gains g;
  g.impedance.kp_pos = {0.0, 0.0, 0.0};
  g.impedance.kp_rot = {0.0, 0.0, 0.0};
  g.impedance.kd_pos = {kRefKd, kRefKd, kRefKd};
  g.impedance.kd_rot = {kRefKd, kRefKd, kRefKd};
  g.activation_ramp_time = 0.0;  // α ≡ 1
  g.max_torque_rate = 1.0e12;    // §10.5 inert: the command IS the control law
  g.joint_limit_kp = 0.0;
  g.joint_limit_kd = 0.0;
  g.joint_limit_margin = 0.0;
  rtc::TaskImpedanceController ctrl(urdf, g);
  ctrl.SetDeviceNameConfigs(PermutedDeviceConfig());

  auto st = MakeState(3);
  FillPermutedState(st);
  (void)ctrl.Compute(st);  // seeds X_d from the measured pose ⇒ e ≡ 0
  const auto out = ctrl.Compute(st);

  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], ref[kDeviceToPin[i]], 1e-6)
        << "device channel " << i << " (" << kPermutedNames[i] << ")";
  }
  // Mutation check: with the device-order q̇ fed straight to J the command is a
  // materially different vector, so this test cannot pass vacuously.
  const Eigen::VectorXd ref_unreordered = RefDampingPlusGravity(urdf, kQPin, {0.9, 0.7, -0.4});
  EXPECT_GT((ref - ref_unreordered).norm(), 1e-2) << "fixture must make the permutation observable";
}

TEST(JointOrder, CascadedComplianceTaskTwistUsesReorderedVelocity) {
  // Same law, same reference — the cascade's inner loop. Its outer loop is inert
  // here (no wrench published ⇒ f_ext = 0 ⇒ X_c ≡ X_d, ν_c = 0), so the inner
  // impedance term is again the only live one.
  const std::string urdf = UrdfPath("planar_3r.urdf");
  const Eigen::VectorXd ref = RefDampingPlusGravity(urdf, kQPin, kQdPin);

  rtc::CascadedComplianceController::Gains g;
  g.impedance.kp_pos = {0.0, 0.0, 0.0};
  g.impedance.kp_rot = {0.0, 0.0, 0.0};
  g.impedance.kd_pos = {kRefKd, kRefKd, kRefKd};
  g.impedance.kd_rot = {kRefKd, kRefKd, kRefKd};
  g.activation_ramp_time = 0.0;
  g.max_torque_rate = 1.0e12;
  g.joint_limit_kp = 0.0;
  g.joint_limit_kd = 0.0;
  g.joint_limit_margin = 0.0;
  rtc::CascadedComplianceController ctrl(urdf, g);
  ctrl.SetDeviceNameConfigs(PermutedDeviceConfig());

  auto st = MakeState(3);
  FillPermutedState(st);
  (void)ctrl.Compute(st);
  const auto out = ctrl.Compute(st);

  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], ref[kDeviceToPin[i]], 1e-6)
        << "device channel " << i << " (" << kPermutedNames[i] << ")";
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// (4) Phase 3 (A1) submodel selection — controller DOF == primary device group
// ═══════════════════════════════════════════════════════════════════════════════
//
// arm_with_mimic.urdf is an arm (base_link → link_1..link_5, 5 revolute) plus a
// gripper (finger_left prismatic + finger_right prismatic, a mimic of finger_left).
// Pinocchio's full model counts the mimic joint as an independent DOF, so the full
// model DOF is nv = 7 (5 arm + finger_left + finger_right). Sub_model "arm" (root
// base_link → tip link_5) locks BOTH gripper joints out → nv = 5. When the
// controller's primary device group is "arm", MaybeSelectSubModel (in LoadConfig)
// must switch handle_ to that reduced model so the controller DOF equals the arm's
// channel count. No system config → the full model is kept (regression guard).

namespace {

rub::ModelConfig ArmHandSystemConfig() {
  rub::ModelConfig sys;
  sys.urdf_path = UrdfPath("arm_with_mimic.urdf");
  sys.root_joint_type = "fixed";
  sys.sub_models.push_back({"arm", "base_link", "link_5"});  // name == primary device group
  return sys;
}

// Controller YAML with a single device group "arm" so GetPrimaryDeviceName()=="arm",
// plus optional trailing gains lines.
YAML::Node ArmControllerYaml(const std::string& gains_yaml) {
  return YAML::Load(std::string(R"(
topics:
  arm:
    subscribe:
      - topic: "arm/joint_goal"
        role: "target"
)") + gains_yaml);
}

}  // namespace

TEST(SubModelSelection, JointPdSelectsArmSubmodelNv5) {
  // With the system config injected, LoadConfig validates kp/kd against the
  // SELECTED submodel nv (5), not the full model (6).
  const auto sys = ArmHandSystemConfig();

  rtc::JointPDController ctrl(sys.urdf_path);  // ctor builds the full model (nv=6)
  ctrl.SetSystemModelConfig(sys);
  EXPECT_NO_THROW(ctrl.LoadConfig(ArmControllerYaml("kp: [1, 2, 3, 4, 5]\nkd: [1, 2, 3, 4, 5]\n")))
      << "5-length gains must match the selected arm submodel (nv=5)";

  // A full-model-length (7) gain vector must now be REJECTED — proof the active nv
  // is the submodel's 5, not the full model's 7.
  rtc::JointPDController ctrl_full(sys.urdf_path);
  ctrl_full.SetSystemModelConfig(sys);
  EXPECT_THROW(ctrl_full.LoadConfig(ArmControllerYaml("kp: [1, 2, 3, 4, 5, 6, 7]\n")),
               std::runtime_error);
}

TEST(SubModelSelection, PSelectsArmSubmodelNv5) {
  const auto sys = ArmHandSystemConfig();

  rtc::PController ctrl(sys.urdf_path);
  ctrl.SetSystemModelConfig(sys);
  EXPECT_NO_THROW(ctrl.LoadConfig(ArmControllerYaml("kp: [1, 2, 3, 4, 5]\n")));

  rtc::PController ctrl_full(sys.urdf_path);
  ctrl_full.SetSystemModelConfig(sys);
  EXPECT_THROW(ctrl_full.LoadConfig(ArmControllerYaml("kp: [1, 2, 3, 4, 5, 6, 7]\n")),
               std::runtime_error);
}

TEST(SubModelSelection, NoSystemConfigKeepsFullModelNv7) {
  // No system config → no submodel selection → full model nv = 7 retained
  // (regression guard: single-URDF robots are unaffected). Now 7-length gains are
  // accepted and 5-length rejected — the inverse of the selected case.
  const std::string urdf = UrdfPath("arm_with_mimic.urdf");

  rtc::JointPDController ctrl(urdf);
  EXPECT_NO_THROW(
      ctrl.LoadConfig(ArmControllerYaml("kp: [1, 2, 3, 4, 5, 6, 7]\nkd: [1, 2, 3, 4, 5, 6, 7]\n")));

  rtc::JointPDController ctrl5(urdf);
  EXPECT_THROW(ctrl5.LoadConfig(ArmControllerYaml("kp: [1, 2, 3, 4, 5]\n")), std::runtime_error);
}

TEST(SubModelSelection, ClikAndOscRunOnArmSubmodel) {
  // CLIK/OSC share the identical MaybeSelectSubModel + InitFromModel buffer-rebuild
  // logic but expose no kp-length proxy. Assert instead that after submodel
  // selection they Compute FINITE output for a 5-channel arm device — which
  // exercises every nv=5-sized buffer (J, M, Λ, τ, dq) and so validates that the
  // rebuild resized them from the full-model 6. (nv-exactness itself is pinned by
  // the JointPD/P kp-length tests above; this covers the CLIK/OSC rebuild path.)
  const auto sys = ArmHandSystemConfig();

  auto st5 = MakeState(5);
  for (std::size_t i = 0; i < 5; ++i)
    st5.devices[0].positions[i] = 0.1 * static_cast<double>(i + 1);

  rtc::ClikController clik(sys.urdf_path, rtc::ClikController::Gains{});
  clik.SetSystemModelConfig(sys);
  clik.LoadConfig(ArmControllerYaml(""));
  (void)clik.Compute(st5);
  const auto co = clik.Compute(st5);
  for (std::size_t i = 0; i < 5; ++i)
    EXPECT_TRUE(std::isfinite(co.devices[0].commands[i])) << "clik cmd[" << i << "]";

  rtc::OperationalSpaceController osc(sys.urdf_path, rtc::OperationalSpaceController::Gains{});
  osc.SetSystemModelConfig(sys);
  osc.LoadConfig(ArmControllerYaml(""));
  (void)osc.Compute(st5);
  const auto oo = osc.Compute(st5);
  for (std::size_t i = 0; i < 5; ++i)
    EXPECT_TRUE(std::isfinite(oo.devices[0].commands[i])) << "osc cmd[" << i << "]";
}
