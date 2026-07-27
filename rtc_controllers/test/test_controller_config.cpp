// Unit tests for the *configuration / registry-hook* surface of the four core
// rtc_controllers (P, JointPD, CLIK, OSC). test_core_controllers.cpp covers the
// per-tick control law; this file covers the previously-untested off-RT setup
// paths:
//   - LoadConfig(YAML)            — gains / command_type parsing + null no-op
//   - OnDeviceConfigsSet()        — tip_link / joint_limits / safe_position
//                                   resolution (driven via SetDeviceNameConfigs)
//   - SetDeviceTarget bounds guard
//   - multi-device passthrough init + secondary-device target drain
//   - 6-DOF task target paths (CLIK / OSC) + OSC π-rotation segment split
//   - gravity / Coriolis compensation branches (JointPD / OSC)
//   - off-RT-only setters (ClearEstop re-seed, SetHandEstop)
//
// All paths are node-free: base LoadConfig only parses the optional "topics"
// section, and SetDeviceNameConfigs() invokes OnDeviceConfigsSet() directly.
//
// Gravity/Coriolis NOTE: serial_6dof.urdf has all-Z revolute axes, so under the
// default -Z gravity the generalized gravity torque is identically zero and the
// constant mass matrix makes Coriolis ≈ 0 (same property asserted in
// rtc_urdf_bridge test_rt_model_handle). These tests therefore exercise the
// compensation *branches* and assert finiteness/validity — the dynamics
// magnitudes are validated against the RNEA identity in rtc_urdf_bridge.

#include "rtc_controllers/direct/joint_pd_controller.hpp"
#include "rtc_controllers/direct/operational_space_controller.hpp"
#include "rtc_controllers/indirect/clik_controller.hpp"
#include "rtc_controllers/indirect/p_controller.hpp"
#include "test_urdf_path.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <map>
#include <string>

namespace {

std::string GetTestUrdfPath() {
  if (const char* env = std::getenv("RTC_TEST_URDF_PATH")) {
    return env;
  }
  return rtc::test::TestUrdfPath("serial_6dof.urdf");
}

rtc::ControllerState MakeState(int nj = 6, double dt = 0.002) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = dt;
  state.devices[0].num_channels = nj;
  state.devices[0].valid = true;
  return state;
}

// num_devices=2: primary arm (6ch) + a secondary joint device.
rtc::ControllerState MakeTwoDeviceState(int nj1 = 4, bool dev1_valid = true) {
  auto state = MakeState();
  state.num_devices = 2;
  state.devices[1].num_channels = nj1;
  state.devices[1].valid = dev1_valid;
  return state;
}

// Single-device config map keyed by `name`. Empty optionals exercise the
// fallback paths in each controller's OnDeviceConfigsSet().
std::map<std::string, rtc::DeviceNameConfig> MakeConfigMap(const std::string& name,
                                                           rtc::DeviceNameConfig cfg) {
  cfg.device_name = name;
  std::map<std::string, rtc::DeviceNameConfig> m;
  m.emplace(name, std::move(cfg));
  return m;
}

bool AllFinite(const std::array<double, rtc::kMaxDeviceChannels>& a, int n) {
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(a[static_cast<std::size_t>(i)])) {
      return false;
    }
  }
  return true;
}

// An *undefined* node (operator bool == false) — the "no YAML config" case CM
// hands a controller. A default-constructed Node is a defined Null, so it does
// NOT take the `if (!cfg) return;` early-out; this does.
YAML::Node UndefinedNode() {
  return YAML::Node(YAML::NodeType::Undefined);
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// PController
// ═══════════════════════════════════════════════════════════════════════════════

TEST(PControllerConfig, LoadConfigParsesKpAndCommandType) {
  rtc::PController ctrl(GetTestUrdfPath());
  YAML::Node cfg = YAML::Load(R"(
kp: [10.0, 11.0, 12.0, 13.0, 14.0, 15.0]
command_type: torque
)");
  ctrl.LoadConfig(cfg);

  const auto g = ctrl.get_gains();
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(g.kp[i], 10.0 + static_cast<double>(i), 1e-12) << "kp[" << i << "]";
  }
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kTorque);
}

TEST(PControllerConfig, LoadConfigNullNodeKeepsDefaults) {
  rtc::PController ctrl(GetTestUrdfPath());
  const auto before = ctrl.get_gains();
  ctrl.LoadConfig(UndefinedNode());
  const auto after = ctrl.get_gains();
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(before.kp[i], after.kp[i]) << "kp[" << i << "]";
  }
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kPosition);
}

TEST(PControllerConfig, LoadConfigWrongSizeKpThrows) {
  // #172: a `kp` length that does not match the model DOF (nv=6 for the test
  // model) is a fail-fast configuration error, not a silently-ignored value.
  // (Previously the mismatch was dropped, hiding a DOF-generalization bug.)
  rtc::PController ctrl(GetTestUrdfPath());
  YAML::Node cfg = YAML::Load("kp: [1.0, 2.0, 3.0]");
  EXPECT_THROW(ctrl.LoadConfig(cfg), std::runtime_error);
}

TEST(PControllerConfig, LoadConfigMatchingSizeKpApplied) {
  // A `kp` of exactly nv entries is accepted and applied.
  rtc::PController ctrl(GetTestUrdfPath());
  YAML::Node cfg = YAML::Load("kp: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]");
  ctrl.LoadConfig(cfg);
  const auto g = ctrl.get_gains();
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(g.kp[i], static_cast<double>(i + 1));
  }
}

TEST(PControllerConfig, OnDeviceConfigsSetTipLinkChangesTcpFrame) {
  // Default tip = last frame (tool_link); overriding to an earlier link moves
  // the FK frame, so the reported TCP differs at a bent configuration.
  rtc::PController def(GetTestUrdfPath());
  rtc::PController tip(GetTestUrdfPath());

  rtc::DeviceNameConfig cfg;
  rtc::DeviceUrdfConfig urdf;
  urdf.tip_link = "link_3";
  cfg.urdf = urdf;
  tip.SetDeviceNameConfigs(MakeConfigMap("arm", cfg));

  auto state = MakeState();
  state.devices[0].positions[1] = 0.5;
  state.devices[0].positions[2] = 0.5;

  (void)def.Compute(state);
  (void)tip.Compute(state);
  const auto out_def = def.Compute(state);
  const auto out_tip = tip.Compute(state);

  double diff = 0.0;
  for (int i = 0; i < 3; ++i) {
    diff += std::abs(out_def.actual_task_positions[static_cast<std::size_t>(i)] -
                     out_tip.actual_task_positions[static_cast<std::size_t>(i)]);
  }
  EXPECT_GT(diff, 1e-3) << "tip_link override should move the FK TCP frame";
}

TEST(PControllerConfig, OnDeviceConfigsSetJointLimitsClampCommand) {
  // A tiny per-joint velocity limit clamps the P-control command symmetrically.
  rtc::PController ctrl(GetTestUrdfPath());
  rtc::DeviceNameConfig cfg;
  rtc::DeviceJointLimits lim;
  lim.max_velocity.assign(6, 0.01);
  cfg.joint_limits = lim;
  ctrl.SetDeviceNameConfigs(MakeConfigMap("arm", cfg));

  auto state = MakeState();
  (void)ctrl.Compute(state);
  std::array<double, 6> target{5.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // large error
  ctrl.SetDeviceTarget(0, target);
  auto out = ctrl.Compute(state);

  // Unclamped command would be 120*5*0.002 = 1.2; the 0.01 limit clamps it.
  EXPECT_NEAR(out.devices[0].commands[0], 0.01, 1e-9);
}

TEST(PControllerConfig, OnDeviceConfigsSetEmptyMapUsesDefaultLimits) {
  // Empty config map → GetDeviceNameConfig(primary) is null → default limits.
  // The default 2.0 rad/s limit leaves the 1.2 command unclamped.
  rtc::PController ctrl(GetTestUrdfPath());
  ctrl.SetDeviceNameConfigs({});

  auto state = MakeState();
  (void)ctrl.Compute(state);
  std::array<double, 6> target{5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ctrl.SetDeviceTarget(0, target);
  auto out = ctrl.Compute(state);
  EXPECT_NEAR(out.devices[0].commands[0], 1.2, 1e-9);
}

TEST(PControllerConfig, SetDeviceTargetIgnoresOutOfRangeIndex) {
  rtc::PController ctrl(GetTestUrdfPath());
  auto state = MakeState();
  (void)ctrl.Compute(state);

  std::array<double, 6> target{0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  ctrl.SetDeviceTarget(-1, target);                                 // negative → ignored
  ctrl.SetDeviceTarget(rtc::ControllerState::kMaxDevices, target);  // OOB → ignored
  auto out = ctrl.Compute(state);

  // No valid target was queued, so the command holds at the seeded position.
  EXPECT_NEAR(out.devices[0].commands[0], 0.0, 1e-9);
}

TEST(PControllerConfig, MultiDevicePassthrough) {
  rtc::PController ctrl(GetTestUrdfPath());
  auto state = MakeTwoDeviceState(/*nj1=*/4);
  (void)ctrl.Compute(state);

  std::array<double, 4> hand_target{0.1, 0.2, 0.3, 0.4};
  ctrl.SetDeviceTarget(1, hand_target);
  auto out = ctrl.Compute(state);

  EXPECT_EQ(out.devices[1].num_channels, 4);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(out.devices[1].commands[i], hand_target[i], 1e-12) << "ch " << i;
  }
}

TEST(PControllerConfig, MultiDeviceInvalidSecondarySkippedInInit) {
  // Invalid secondary device is skipped during target-slot self-init; Compute
  // must still produce valid primary output.
  rtc::PController ctrl(GetTestUrdfPath());
  auto state = MakeTwoDeviceState(/*nj1=*/4, /*dev1_valid=*/false);
  (void)ctrl.Compute(state);
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(AllFinite(out.devices[0].commands, 6));
}

// ═══════════════════════════════════════════════════════════════════════════════
// JointPDController
// ═══════════════════════════════════════════════════════════════════════════════

TEST(JointPDConfig, LoadConfigParsesGainsAndFlags) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  YAML::Node cfg = YAML::Load(R"(
kp: [1, 2, 3, 4, 5, 6]
kd: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
enable_gravity_compensation: true
enable_coriolis_compensation: true
trajectory_speed: 2.5
command_type: torque
)");
  ctrl.LoadConfig(cfg);

  const auto g = ctrl.get_gains();
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(g.kp[i], static_cast<double>(i + 1), 1e-12);
    EXPECT_NEAR(g.kd[i], 0.1 * static_cast<double>(i + 1), 1e-12);
  }
  EXPECT_TRUE(g.enable_gravity_compensation);
  EXPECT_TRUE(g.enable_coriolis_compensation);
  EXPECT_DOUBLE_EQ(g.trajectory_speed, 2.5);
  // #172: dynamics compensation (N·m) is only valid on a torque command.
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kTorque);
}

TEST(JointPDConfig, DynamicsCompensationInPositionModeThrows) {
  // #172: enabling gravity/Coriolis compensation (N·m) while the command type
  // is position/velocity mixes units — reject it fail-fast at config time.
  rtc::JointPDController ctrl(GetTestUrdfPath());
  YAML::Node cfg = YAML::Load(R"(
enable_gravity_compensation: true
command_type: position
)");
  EXPECT_THROW(ctrl.LoadConfig(cfg), std::runtime_error);
}

TEST(JointPDConfig, LoadConfigClampsTrajectorySpeedFloor) {
  // trajectory_speed is floored at 1e-6 to avoid division-by-zero in Compute.
  rtc::JointPDController ctrl(GetTestUrdfPath());
  YAML::Node cfg = YAML::Load("trajectory_speed: 0.0");
  ctrl.LoadConfig(cfg);
  EXPECT_GE(ctrl.get_gains().trajectory_speed, 1e-6);
}

TEST(JointPDConfig, LoadConfigNullNodeKeepsDefaults) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  const auto before = ctrl.get_gains();
  ctrl.LoadConfig(UndefinedNode());
  EXPECT_DOUBLE_EQ(ctrl.get_gains().kp[0], before.kp[0]);
}

TEST(JointPDConfig, OnDeviceConfigsSetSafePositionDrivesEstop) {
  // safe_position from device config becomes the E-STOP target. With a torque
  // controller, the PD error (safe - current) drives the sign of the torque.
  rtc::JointPDController ctrl(GetTestUrdfPath());
  rtc::DeviceNameConfig cfg;
  cfg.safe_position = {0.4, 0.0, 0.0, 0.0, 0.0, 0.0};
  ctrl.SetDeviceNameConfigs(MakeConfigMap("arm", cfg));

  auto state = MakeState();
  state.devices[0].positions[0] = 0.0;  // below safe_position[0] → positive error
  (void)ctrl.Compute(state);
  ctrl.TriggerEstop();
  auto out = ctrl.Compute(state);

  EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);
  EXPECT_GT(out.devices[0].commands[0], 0.0)
      << "E-STOP should drive toward safe_position (0.4 > 0.0)";
  // goal_positions echo the safe position in E-STOP.
  EXPECT_NEAR(out.devices[0].goal_positions[0], 0.4, 1e-12);
}

TEST(JointPDConfig, OnDeviceConfigsSetTorqueLimitClamps) {
  // max_torque limit clamps the E-STOP torque (torque command path).
  rtc::JointPDController ctrl(GetTestUrdfPath());
  rtc::DeviceNameConfig cfg;
  rtc::DeviceJointLimits lim;
  lim.max_torque.assign(6, 0.05);
  cfg.joint_limits = lim;
  cfg.safe_position = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ctrl.SetDeviceNameConfigs(MakeConfigMap("arm", cfg));

  auto state = MakeState();
  (void)ctrl.Compute(state);
  ctrl.TriggerEstop();
  auto out = ctrl.Compute(state);  // default torque command_type
  EXPECT_LE(std::abs(out.devices[0].commands[0]), 0.05 + 1e-9)
      << "torque should be clamped to max_torque";
}

TEST(JointPDConfig, GravityCompensationBranchProducesFiniteTorque) {
  // Enable gravity comp → UpdateDynamics computes g(q) and Compute adds it.
  // serial_6dof gravity torque is ~0 (all-Z axes), so assert finiteness, not
  // magnitude; the diagnostic accessor must mirror the cached vector.
  rtc::JointPDController::Gains g;
  g.enable_gravity_compensation = true;
  rtc::JointPDController ctrl(GetTestUrdfPath(), g);

  auto state = MakeState();
  state.devices[0].positions = {0.0, 0.5, -0.3, 0.2, -0.1, 0.4};
  (void)ctrl.Compute(state);
  auto out = ctrl.Compute(state);

  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(AllFinite(out.devices[0].commands, 6));
  const auto grav = ctrl.gravity_torques();
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_TRUE(std::isfinite(grav[i])) << "gravity_torques[" << i << "]";
  }
  // tcp_position() is cached during UpdateDynamics' FK.
  const auto tcp = ctrl.tcp_position();
  for (double c : tcp) {
    EXPECT_TRUE(std::isfinite(c));
  }
}

TEST(JointPDConfig, CoriolisCompensationBranchProducesFiniteTorque) {
  rtc::JointPDController::Gains g;
  g.enable_coriolis_compensation = true;
  rtc::JointPDController ctrl(GetTestUrdfPath(), g);

  auto state = MakeState();
  state.devices[0].positions = {0.1, 0.5, -0.3, 0.2, -0.1, 0.4};
  state.devices[0].velocities = {0.3, -0.2, 0.4, 0.1, -0.5, 0.2};  // nonzero v → C·v
  (void)ctrl.Compute(state);
  auto out = ctrl.Compute(state);

  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(AllFinite(out.devices[0].commands, 6));
}

TEST(JointPDConfig, PositionCommandTypeAddsVelocityFeedforward) {
  // In position mode the trajectory velocity is fed forward into the command;
  // in torque mode it is not. Verify the branch executes and stays finite.
  rtc::JointPDController ctrl(GetTestUrdfPath());
  YAML::Node cfg = YAML::Load("command_type: position");
  ctrl.LoadConfig(cfg);
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kPosition);

  auto state = MakeState();
  (void)ctrl.Compute(state);
  std::array<double, 6> target{0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  ctrl.SetDeviceTarget(0, target);
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(AllFinite(out.devices[0].commands, 6));
}

TEST(JointPDConfig, ClearEstopReseedsAndResumes) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  ctrl.OnDeviceConfigsSet();  // populate safe_position_ fallback
  auto state = MakeState();
  state.devices[0].positions[0] = 0.5;
  (void)ctrl.Compute(state);

  ctrl.TriggerEstop();
  EXPECT_TRUE(ctrl.IsEstopped());
  (void)ctrl.Compute(state);

  ctrl.ClearEstop();
  EXPECT_FALSE(ctrl.IsEstopped());
  // ClearEstop forces target re-seed on the next tick; output stays valid.
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(AllFinite(out.devices[0].commands, 6));
}

TEST(JointPDConfig, SetHandEstopIsAccepted) {
  // SetHandEstop is a write-only latch (no observable control effect here);
  // cover the setter and confirm normal operation continues.
  rtc::JointPDController ctrl(GetTestUrdfPath());
  auto state = MakeState();
  (void)ctrl.Compute(state);
  ctrl.SetHandEstop(true);
  ctrl.SetHandEstop(false);
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);
}

TEST(JointPDConfig, SetDeviceTargetIgnoresOutOfRangeIndex) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  auto state = MakeState();
  (void)ctrl.Compute(state);
  std::array<double, 6> target{0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  ctrl.SetDeviceTarget(-1, target);
  ctrl.SetDeviceTarget(rtc::ControllerState::kMaxDevices, target);
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);
}

TEST(JointPDConfig, MultiDevicePassthrough) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  auto state = MakeTwoDeviceState(/*nj1=*/4);
  (void)ctrl.Compute(state);

  std::array<double, 4> hand{0.1, 0.2, 0.3, 0.4};
  ctrl.SetDeviceTarget(1, hand);
  auto out = ctrl.Compute(state);
  EXPECT_EQ(out.devices[1].num_channels, 4);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(out.devices[1].commands[i], hand[i], 1e-12) << "ch " << i;
  }
}

TEST(JointPDConfig, MultiDeviceInvalidSecondarySkippedInInit) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  auto state = MakeTwoDeviceState(/*nj1=*/4, /*dev1_valid=*/false);
  (void)ctrl.Compute(state);
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(AllFinite(out.devices[0].commands, 6));
}

// ═══════════════════════════════════════════════════════════════════════════════
// ClikController
// ═══════════════════════════════════════════════════════════════════════════════

TEST(ClikConfig, LoadConfigParsesAllGains) {
  rtc::ClikController::Gains init;
  rtc::ClikController ctrl(GetTestUrdfPath(), init);
  YAML::Node cfg = YAML::Load(R"(
kp_translation: [2.0, 3.0, 4.0]
kp_rotation: [1.1, 1.2, 1.3]
max_damping: 0.02
singularity_threshold: 0.03
null_kp: 0.8
enable_null_space: true
control_6dof: true
trajectory_speed: 0.2
trajectory_angular_speed: 0.6
max_traj_velocity: 0.8
max_traj_angular_velocity: 1.5
command_type: position
)");
  ctrl.LoadConfig(cfg);

  const auto g = ctrl.get_gains();
  EXPECT_NEAR(g.kp_translation[0], 2.0, 1e-12);
  EXPECT_NEAR(g.kp_translation[2], 4.0, 1e-12);
  EXPECT_NEAR(g.kp_rotation[1], 1.2, 1e-12);
  EXPECT_NEAR(g.max_damping, 0.02, 1e-12);
  EXPECT_NEAR(g.singularity_threshold, 0.03, 1e-12);
  EXPECT_NEAR(g.null_kp, 0.8, 1e-12);
  EXPECT_TRUE(g.enable_null_space);
  EXPECT_TRUE(g.control_6dof);
  EXPECT_NEAR(g.trajectory_speed, 0.2, 1e-12);
  EXPECT_NEAR(g.trajectory_angular_speed, 0.6, 1e-12);
  EXPECT_NEAR(g.max_traj_velocity, 0.8, 1e-12);
  EXPECT_NEAR(g.max_traj_angular_velocity, 1.5, 1e-12);
}

TEST(ClikConfig, LoadConfigNullNodeKeepsDefaults) {
  rtc::ClikController::Gains init;
  rtc::ClikController ctrl(GetTestUrdfPath(), init);
  const auto before = ctrl.get_gains();
  ctrl.LoadConfig(UndefinedNode());
  EXPECT_DOUBLE_EQ(ctrl.get_gains().max_damping, before.max_damping);
  EXPECT_DOUBLE_EQ(ctrl.get_gains().singularity_threshold, before.singularity_threshold);
}

// Both ends of the §6.5 ramp are floored, and the retired key is inert.
//
// σ₀ is the half the migration left unclamped: `max_damping` was floored from
// the start, but a `singularity_threshold: 0` sails through into
// AdaptiveDampingSquared, whose `sigma0 <= 0` short-circuit returns λ² = 0 for
// EVERY σ_min. That is not a wider or narrower shell — it is no shell, i.e. the
// one guard the pre-migration constant λ provided unconditionally. The other
// three task-space controllers already clamped this key, so this is a
// convergence defect and not a new rule.
TEST(ClikConfig, LoadConfigFloorsBothEndsOfTheDlsRamp) {
  rtc::ClikController::Gains init;
  rtc::ClikController ctrl(GetTestUrdfPath(), init);
  ctrl.LoadConfig(YAML::Load(R"(
max_damping: 0.0
singularity_threshold: 0.0
)"));
  const auto g = ctrl.get_gains();
  EXPECT_GT(g.max_damping, 0.0) << "λ_max = 0 removes the damping the ramp ramps TO";
  EXPECT_GT(g.singularity_threshold, 0.0) << "σ₀ = 0 makes AdaptiveDampingSquared return 0 always";

  // Negative is the same failure with a sign — the short-circuit is `<= 0`.
  ctrl.LoadConfig(YAML::Load("singularity_threshold: -1.0"));
  EXPECT_GT(ctrl.get_gains().singularity_threshold, 0.0);
}

// ── #277: the null-space posture gain is floored at 0, in both places ───────
//
// q̇₀ = K_pⁿ·(q_target − q) with K_pⁿ < 0 drives the posture AWAY from its
// target, and N keeps that off the Cartesian task — so it is a silent drift,
// not a fault. This controller's gate is `enable_null_space && !control_6dof`
// and never reads the gain, so unlike its four siblings there is no gate to
// observe: the point-of-use floor is pinned by the OUTPUT instead, against an
// explicit zero gain.
TEST(ClikConfig, NegativePostureGainIsFlooredAndTheFloorSurvivesSetGains) {
  const std::string urdf7 = rtc::test::TestUrdfPath("serial_7dof.urdf");
  rtc::ClikController::Gains init;
  init.enable_null_space = true;
  init.control_6dof = false;  // the posture branch only runs on the 3-DOF task

  rtc::ClikController cfg_only(urdf7, init);
  cfg_only.LoadConfig(YAML::Load("null_kp: -0.8\n"));
  EXPECT_DOUBLE_EQ(cfg_only.get_gains().null_kp, 0.0)
      << "a divergent posture gain reached the gains POD";
  cfg_only.LoadConfig(YAML::Load("null_kp: 0.8\n"));
  EXPECT_DOUBLE_EQ(cfg_only.get_gains().null_kp, 0.8) << "a floor, not a rewrite";

  // Three identical histories differing only in the gain written straight into
  // the SeqLock — the path set_gains() opens past configure (NUM-1). The seed
  // tick self-initialises null_target ← q, so the arm has to MOVE afterwards for
  // the posture error to be nonzero; otherwise every gain gives the same output
  // and the comparison below would hold for a controller with no floor at all.
  auto run = [&](double null_kp) {
    rtc::ClikController c(urdf7, init);
    auto seed = MakeState(7);
    for (std::size_t i = 0; i < 7; ++i) {
      seed.devices[0].positions[i] = 0.1;
    }
    (void)c.Compute(seed);

    auto g = c.get_gains();
    g.null_kp = null_kp;
    c.set_gains(g);

    auto moved = MakeState(7);
    for (std::size_t i = 0; i < 7; ++i) {
      moved.devices[0].positions[i] = 0.3;  // q_target − q = −0.2 on every channel
    }
    return c.Compute(moved);
  };

  const auto zero = run(0.0);
  const auto negative = run(-0.8);
  const auto positive = run(0.8);

  for (std::size_t i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(negative.devices[0].commands[i], zero.devices[0].commands[i])
        << "ch " << i << ": a negative posture gain reached the law";
  }

  // Non-vacuity: the posture term DOES move this output, so the equality above
  // is the floor doing its job and not a branch that never ran.
  bool positive_differs = false;
  for (std::size_t i = 0; i < 7; ++i) {
    if (positive.devices[0].commands[i] != zero.devices[0].commands[i]) {
      positive_differs = true;
    }
  }
  EXPECT_TRUE(positive_differs)
      << "a positive posture gain changed nothing — the comparison pins nothing";

  // The absent NODE — the widest form of "the key is absent", and the one the
  // controller manager hands a controller with no YAML. LoadConfig returns early
  // there, so a floor placed only after that return never runs and get_gains()
  // keeps reporting a gain Compute() refuses to use.
  auto bypassed = cfg_only.get_gains();
  bypassed.null_kp = -0.8;
  cfg_only.set_gains(bypassed);
  cfg_only.LoadConfig(UndefinedNode());
  EXPECT_DOUBLE_EQ(cfg_only.get_gains().null_kp, 0.0)
      << "the `if (!cfg)` early return skipped the loader floor";
}

TEST(ClikConfig, RetiredDampingKeyIsIgnoredNotMapped) {
  // `damping` set the CONSTANT λ that #236 S3b deleted. LoadConfig ignores keys
  // it does not know, so the risk this pins is silence: an old config parses
  // clean and the controller runs on defaults the operator never chose. It must
  // not be mapped onto max_damping either — a constant and the ceiling of a
  // σ_min-adaptive ramp are different quantities.
  rtc::ClikController::Gains init;
  rtc::ClikController ctrl(GetTestUrdfPath(), init);
  const auto before = ctrl.get_gains();
  ctrl.LoadConfig(YAML::Load("damping: 0.01"));
  const auto after = ctrl.get_gains();
  EXPECT_DOUBLE_EQ(after.max_damping, before.max_damping);
  EXPECT_DOUBLE_EQ(after.singularity_threshold, before.singularity_threshold);
}

TEST(ClikConfig, OnDeviceConfigsSetResolvesTipAndSafePosition) {
  rtc::ClikController::Gains init;
  rtc::ClikController def(GetTestUrdfPath(), init);
  rtc::ClikController tip(GetTestUrdfPath(), init);

  rtc::DeviceNameConfig cfg;
  rtc::DeviceUrdfConfig urdf;
  urdf.tip_link = "link_3";
  cfg.urdf = urdf;
  rtc::DeviceJointLimits lim;
  lim.max_velocity.assign(6, 1.0);
  cfg.joint_limits = lim;
  cfg.safe_position = {0.2, 0.1, 0.0, 0.0, 0.0, 0.0};
  tip.SetDeviceNameConfigs(MakeConfigMap("arm", cfg));

  auto state = MakeState();
  state.devices[0].positions[1] = 0.5;
  state.devices[0].positions[2] = 0.5;
  (void)def.Compute(state);
  (void)tip.Compute(state);
  const auto out_def = def.Compute(state);
  const auto out_tip = tip.Compute(state);

  double diff = 0.0;
  for (int i = 0; i < 3; ++i) {
    diff += std::abs(out_def.actual_task_positions[static_cast<std::size_t>(i)] -
                     out_tip.actual_task_positions[static_cast<std::size_t>(i)]);
  }
  EXPECT_GT(diff, 1e-3) << "tip_link override should move the FK TCP frame";
}

TEST(ClikConfig, SixDofTargetUpdatesOrientationGoal) {
  // control_6dof=true: a 6-value SetDeviceTarget sets both position and
  // orientation; task_goal_positions[3..5] reflect the requested RPY.
  rtc::ClikController::Gains g;
  g.control_6dof = true;
  rtc::ClikController ctrl(GetTestUrdfPath(), g);

  auto state = MakeState();
  state.devices[0].positions = {0.1, 0.4, -0.2, 0.3, 0.1, 0.2};
  (void)ctrl.Compute(state);
  auto out0 = ctrl.Compute(state);

  std::array<double, 6> target{};
  target[0] = out0.actual_task_positions[0] + 0.05;
  target[1] = out0.actual_task_positions[1];
  target[2] = out0.actual_task_positions[2];
  target[3] = 0.3;   // roll
  target[4] = -0.2;  // pitch
  target[5] = 0.5;   // yaw
  ctrl.SetDeviceTarget(0, target);
  auto out1 = ctrl.Compute(state);

  EXPECT_NEAR(out1.task_goal_positions[3], 0.3, 1e-6);
  EXPECT_NEAR(out1.task_goal_positions[4], -0.2, 1e-6);
  EXPECT_NEAR(out1.task_goal_positions[5], 0.5, 1e-6);
}

TEST(ClikConfig, MultiDeviceSecondaryPassthrough) {
  rtc::ClikController::Gains init;
  rtc::ClikController ctrl(GetTestUrdfPath(), init);
  auto state = MakeTwoDeviceState(/*nj1=*/3);
  (void)ctrl.Compute(state);

  std::array<double, 3> hand{0.7, 0.8, 0.9};
  ctrl.SetDeviceTarget(1, hand);
  auto out = ctrl.Compute(state);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(out.devices[1].commands[i], hand[i], 1e-12) << "ch " << i;
  }
}

TEST(ClikConfig, MultiDeviceInvalidSecondarySkippedInInit) {
  rtc::ClikController::Gains init;
  rtc::ClikController ctrl(GetTestUrdfPath(), init);
  auto state = MakeTwoDeviceState(/*nj1=*/3, /*dev1_valid=*/false);
  (void)ctrl.Compute(state);
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(AllFinite(out.devices[0].commands, 6));
}

TEST(ClikConfig, SetDeviceTargetIgnoresOutOfRangeIndex) {
  rtc::ClikController::Gains init;
  rtc::ClikController ctrl(GetTestUrdfPath(), init);
  auto state = MakeState();
  (void)ctrl.Compute(state);
  std::array<double, 6> target{};
  ctrl.SetDeviceTarget(-1, target);
  ctrl.SetDeviceTarget(rtc::ControllerState::kMaxDevices, target);
  ctrl.SetHandEstop(true);  // write-only latch, cover the setter
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);
}

TEST(ClikConfig, EstopThenClearResumes) {
  rtc::ClikController::Gains init;
  rtc::ClikController ctrl(GetTestUrdfPath(), init);
  ctrl.OnDeviceConfigsSet();  // populate safe_position_ / max_joint_velocity_
  auto state = MakeState();
  state.devices[0].positions[0] = 0.5;
  (void)ctrl.Compute(state);

  ctrl.TriggerEstop();
  EXPECT_TRUE(ctrl.IsEstopped());
  auto estop_out = ctrl.Compute(state);
  EXPECT_EQ(estop_out.command_type, rtc::CommandType::kPosition);
  // E-STOP drives joint 0 toward safe_position (0.0) from 0.5.
  EXPECT_LT(estop_out.devices[0].commands[0], 0.5);

  ctrl.ClearEstop();
  EXPECT_FALSE(ctrl.IsEstopped());
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(AllFinite(out.devices[0].commands, 6));
}

TEST(ClikConfig, SixDofPiRotationSplitsTrajectory) {
  // control_6dof + a ~π rotation target trips the segment-split defense; run
  // past the segment duration to exercise the mid-segment transition.
  rtc::ClikController::Gains g;
  g.control_6dof = true;
  g.trajectory_speed = 5.0;
  g.trajectory_angular_speed = 5.0;
  rtc::ClikController ctrl(GetTestUrdfPath(), g);

  auto state = MakeState();
  (void)ctrl.Compute(state);
  auto out0 = ctrl.Compute(state);

  std::array<double, 6> target{};
  for (int i = 0; i < 3; ++i) {
    target[static_cast<std::size_t>(i)] = out0.actual_task_positions[static_cast<std::size_t>(i)];
  }
  target[3] = out0.actual_task_positions[3] + M_PI;  // ~π roll → split path
  target[4] = out0.actual_task_positions[4];
  target[5] = out0.actual_task_positions[5];
  ctrl.SetDeviceTarget(0, target);

  for (int i = 0; i < 4000; ++i) {
    auto out = ctrl.Compute(state);
    ASSERT_TRUE(out.valid) << "iter " << i;
    ASSERT_TRUE(AllFinite(out.devices[0].commands, 6)) << "iter " << i;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// OperationalSpaceController
// ═══════════════════════════════════════════════════════════════════════════════

TEST(OscConfig, LoadConfigParsesAllGains) {
  rtc::OperationalSpaceController::Gains init;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), init);
  YAML::Node cfg = YAML::Load(R"(
kp_pos: [3.0, 3.0, 3.0]
kd_pos: [0.2, 0.2, 0.2]
kp_rot: [1.0, 1.0, 1.0]
kd_rot: [0.1, 0.1, 0.1]
max_damping: 0.05
singularity_threshold: 0.03
enable_gravity_compensation: true
trajectory_speed: 0.15
trajectory_angular_speed: 0.7
max_traj_velocity: 0.6
max_traj_angular_velocity: 1.2
command_type: torque
)");
  ctrl.LoadConfig(cfg);

  const auto g = ctrl.get_gains();
  EXPECT_NEAR(g.kp_pos[0], 3.0, 1e-12);
  EXPECT_NEAR(g.kd_pos[1], 0.2, 1e-12);
  EXPECT_NEAR(g.kp_rot[2], 1.0, 1e-12);
  EXPECT_NEAR(g.kd_rot[0], 0.1, 1e-12);
  EXPECT_NEAR(g.max_damping, 0.05, 1e-12);
  EXPECT_NEAR(g.singularity_threshold, 0.03, 1e-12);
  EXPECT_TRUE(g.enable_gravity_compensation);
  EXPECT_NEAR(g.trajectory_speed, 0.15, 1e-12);
  EXPECT_NEAR(g.max_traj_angular_velocity, 1.2, 1e-12);
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kTorque);
}

TEST(OscConfig, LoadConfigNullNodeKeepsDefaults) {
  rtc::OperationalSpaceController::Gains init;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), init);
  const auto before = ctrl.get_gains();
  ctrl.LoadConfig(UndefinedNode());
  EXPECT_DOUBLE_EQ(ctrl.get_gains().max_damping, before.max_damping);
  EXPECT_DOUBLE_EQ(ctrl.get_gains().singularity_threshold, before.singularity_threshold);
}

// Same two guarantees as the CLIK pair above — the two controllers #236 S2b+S3b
// migrated share the defect and share the fix.
TEST(OscConfig, LoadConfigFloorsBothEndsOfTheDlsRamp) {
  rtc::OperationalSpaceController::Gains init;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), init);
  ctrl.LoadConfig(YAML::Load(R"(
max_damping: 0.0
singularity_threshold: 0.0
command_type: torque
)"));
  const auto g = ctrl.get_gains();
  EXPECT_GT(g.max_damping, 0.0) << "λ_max = 0 removes the damping the ramp ramps TO";
  EXPECT_GT(g.singularity_threshold, 0.0) << "σ₀ = 0 makes AdaptiveDampingSquared return 0 always";

  ctrl.LoadConfig(YAML::Load("singularity_threshold: -1.0"));
  EXPECT_GT(ctrl.get_gains().singularity_threshold, 0.0);
}

// ── #277: both posture gains floored, in LoadConfig and at the point of use ──
//
// τ₀ = K_pⁿ·(q_safe − q) − K_dⁿ·q̇ projected through Nᵀ. A negative K_pⁿ pushes
// away from the configured safe posture and a negative K_dⁿ injects energy into
// the redundant DOFs, and Nᵀ keeps either from disturbing the Cartesian task —
// so the posture drifts with every number finite and no fault raised.
//
// The redundant fixture is mandatory here: the gate is `nv > 6`, so on
// serial_6dof it can never open and the second half would be vacuous.
TEST(OscConfig, PostureGainsAreFlooredAndTheFloorSurvivesSetGains) {
  rtc::OperationalSpaceController::Gains init;
  rtc::OperationalSpaceController ctrl(rtc::test::TestUrdfPath("serial_7dof.urdf"), init);

  ctrl.LoadConfig(YAML::Load(R"(
null_kp: -8.0
null_kd: -1.0
command_type: torque
)"));
  EXPECT_DOUBLE_EQ(ctrl.get_gains().null_kp, 0.0)
      << "a divergent posture stiffness reached the gains POD";
  EXPECT_DOUBLE_EQ(ctrl.get_gains().null_kd, 0.0)
      << "negative damping injects energy into the null space";

  // A key the YAML never mentions is floored too. The value then comes from the
  // constructor or a previous set_gains(), and a floor folded into the parse
  // would skip it entirely — leaving get_gains() reporting a gain Compute()
  // refuses to run. (TaskImpedanceController is where that gap actually bites:
  // its §6.1 TRANSLATION_ONLY guard reads the gain on exactly this path.)
  auto bypassed = ctrl.get_gains();
  bypassed.null_kp = -8.0;
  ctrl.set_gains(bypassed);
  ctrl.LoadConfig(YAML::Load("command_type: torque\n"));
  EXPECT_DOUBLE_EQ(ctrl.get_gains().null_kp, 0.0)
      << "an absent key left a negative posture gain in the POD";

  // A floor, not a rewrite — and this pair is what opens the gate below.
  ctrl.LoadConfig(YAML::Load("null_kp: 8.0\nnull_kd: 1.0\n"));
  ASSERT_DOUBLE_EQ(ctrl.get_gains().null_kp, 8.0);
  ASSERT_DOUBLE_EQ(ctrl.get_gains().null_kd, 1.0);

  auto state = MakeState(7);
  for (std::size_t i = 0; i < 7; ++i) {
    state.devices[0].positions[i] = 0.2;  // away from the singular zero posture
  }
  (void)ctrl.Compute(state);
  ASSERT_TRUE(ctrl.nullspace_active())
      << "the fixture never opens the posture gate — the negative case proves nothing";

  // The set_gains() bypass (NUM-1). The gate is `kp != 0 || kd != 0`, so a floor
  // applied only to the law's arguments would hold it OPEN on a negative gain
  // and project an all-zero τ₀ through Nᵀ; floored before the gate, a negative
  // gain reads exactly as zero does.
  auto g = ctrl.get_gains();
  g.null_kp = -8.0;
  g.null_kd = -1.0;
  ctrl.set_gains(g);
  (void)ctrl.Compute(state);
  EXPECT_FALSE(ctrl.nullspace_active())
      << "a negative posture gain held the gate open — the floor is configure-only";

  // The absent NODE — the widest form of "the key is absent", and the one the
  // controller manager actually hands a controller with no YAML. LoadConfig
  // returns early there, so a floor placed only after that return covers neither
  // gain and get_gains() keeps reporting one Compute() refuses to run. The POD
  // is still the negative pair set_gains() just wrote.
  ASSERT_DOUBLE_EQ(ctrl.get_gains().null_kp, -8.0) << "the bypass above did not take";
  ctrl.LoadConfig(UndefinedNode());
  EXPECT_DOUBLE_EQ(ctrl.get_gains().null_kp, 0.0)
      << "the `if (!cfg)` early return skipped the loader floor";
  EXPECT_DOUBLE_EQ(ctrl.get_gains().null_kd, 0.0);
}

TEST(OscConfig, RetiredDampingKeyIsIgnoredNotMapped) {
  rtc::OperationalSpaceController::Gains init;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), init);
  const auto before = ctrl.get_gains();
  ctrl.LoadConfig(YAML::Load("damping: 0.01"));
  const auto after = ctrl.get_gains();
  EXPECT_DOUBLE_EQ(after.max_damping, before.max_damping);
  EXPECT_DOUBLE_EQ(after.singularity_threshold, before.singularity_threshold);
}

TEST(OscConfig, OnDeviceConfigsSetResolvesTipLink) {
  rtc::OperationalSpaceController::Gains init;
  rtc::OperationalSpaceController def(GetTestUrdfPath(), init);
  rtc::OperationalSpaceController tip(GetTestUrdfPath(), init);

  rtc::DeviceNameConfig cfg;
  rtc::DeviceUrdfConfig urdf;
  urdf.tip_link = "link_3";
  cfg.urdf = urdf;
  rtc::DeviceJointLimits lim;
  lim.max_velocity.assign(6, 1.0);
  cfg.joint_limits = lim;
  cfg.safe_position = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
  tip.SetDeviceNameConfigs(MakeConfigMap("arm", cfg));

  auto state = MakeState();
  state.devices[0].positions[1] = 0.5;
  state.devices[0].positions[2] = 0.5;
  (void)def.Compute(state);
  (void)tip.Compute(state);
  const auto out_def = def.Compute(state);
  const auto out_tip = tip.Compute(state);

  double diff = 0.0;
  for (int i = 0; i < 3; ++i) {
    diff += std::abs(out_def.actual_task_positions[static_cast<std::size_t>(i)] -
                     out_tip.actual_task_positions[static_cast<std::size_t>(i)]);
  }
  EXPECT_GT(diff, 1e-3);
}

TEST(OscConfig, GravityCompensationBranchProducesFiniteOutput) {
  rtc::OperationalSpaceController::Gains g;
  g.enable_gravity_compensation = true;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), g);

  auto state = MakeState();
  state.devices[0].positions = {0.1, 0.5, -0.3, 0.2, -0.1, 0.4};
  (void)ctrl.Compute(state);
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(AllFinite(out.devices[0].commands, 6));
}

TEST(OscConfig, SixDofPoseTargetNormalTrajectory) {
  // 6-value target with a modest rotation → normal (non-split) task trajectory.
  rtc::OperationalSpaceController::Gains g;
  g.trajectory_speed = 1.0;
  g.trajectory_angular_speed = 1.0;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), g);

  auto state = MakeState();
  state.devices[0].positions = {0.1, 0.4, -0.2, 0.3, 0.1, 0.2};
  (void)ctrl.Compute(state);
  auto out0 = ctrl.Compute(state);

  std::array<double, 6> target{};
  for (int i = 0; i < 3; ++i) {
    target[static_cast<std::size_t>(i)] = out0.actual_task_positions[static_cast<std::size_t>(i)];
  }
  target[0] += 0.05;
  target[3] = out0.actual_task_positions[3] + 0.2;  // small rotation delta
  target[4] = out0.actual_task_positions[4];
  target[5] = out0.actual_task_positions[5];
  ctrl.SetDeviceTarget(0, target);

  for (int i = 0; i < 50; ++i) {
    auto out = ctrl.Compute(state);
    ASSERT_TRUE(AllFinite(out.devices[0].commands, 6)) << "iter " << i;
  }
}

TEST(OscConfig, SixDofPiRotationSplitsTrajectory) {
  // A ~π rotation target trips the segment-split defense; running past the
  // segment duration exercises the mid-segment transition. Assert validity
  // through the whole segmented motion.
  rtc::OperationalSpaceController::Gains g;
  g.trajectory_speed = 5.0;          // short translational duration
  g.trajectory_angular_speed = 5.0;  // short angular duration → fast completion
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), g);

  auto state = MakeState();
  (void)ctrl.Compute(state);
  auto out0 = ctrl.Compute(state);

  std::array<double, 6> target{};
  for (int i = 0; i < 3; ++i) {
    target[static_cast<std::size_t>(i)] = out0.actual_task_positions[static_cast<std::size_t>(i)];
  }
  // Request a roll of current + π → angular distance ≈ π → split path.
  target[3] = out0.actual_task_positions[3] + M_PI;
  target[4] = out0.actual_task_positions[4];
  target[5] = out0.actual_task_positions[5];
  ctrl.SetDeviceTarget(0, target);

  for (int i = 0; i < 4000; ++i) {
    auto out = ctrl.Compute(state);
    ASSERT_TRUE(out.valid) << "iter " << i;
    ASSERT_TRUE(AllFinite(out.devices[0].commands, 6)) << "iter " << i;
  }
}

TEST(OscConfig, MultiDeviceSecondaryPassthrough) {
  rtc::OperationalSpaceController::Gains init;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), init);
  auto state = MakeTwoDeviceState(/*nj1=*/3);
  (void)ctrl.Compute(state);

  std::array<double, 3> hand{0.3, 0.6, 0.9};
  ctrl.SetDeviceTarget(1, hand);
  auto out = ctrl.Compute(state);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(out.devices[1].commands[i], hand[i], 1e-12) << "ch " << i;
  }
}

TEST(OscConfig, MultiDeviceInvalidSecondarySkippedInInit) {
  rtc::OperationalSpaceController::Gains init;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), init);
  auto state = MakeTwoDeviceState(/*nj1=*/3, /*dev1_valid=*/false);
  (void)ctrl.Compute(state);
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(AllFinite(out.devices[0].commands, 6));
}

TEST(OscConfig, ClearEstopReseedsAndSetHandEstop) {
  rtc::OperationalSpaceController::Gains init;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), init);
  ctrl.OnDeviceConfigsSet();
  auto state = MakeState();
  state.devices[0].positions[0] = 0.5;
  (void)ctrl.Compute(state);

  ctrl.TriggerEstop();
  (void)ctrl.Compute(state);
  ctrl.ClearEstop();
  EXPECT_FALSE(ctrl.IsEstopped());
  ctrl.SetHandEstop(true);  // write-only latch
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);
  EXPECT_TRUE(AllFinite(out.devices[0].commands, 6));
}

TEST(OscConfig, SetDeviceTargetIgnoresOutOfRangeIndex) {
  rtc::OperationalSpaceController::Gains init;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), init);
  auto state = MakeState();
  (void)ctrl.Compute(state);
  std::array<double, 6> target{};
  ctrl.SetDeviceTarget(-1, target);
  ctrl.SetDeviceTarget(rtc::ControllerState::kMaxDevices, target);
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);
}
