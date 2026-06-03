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

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <map>
#include <string>

namespace {

std::string GetTestUrdfPath() {
  const char* env = std::getenv("RTC_TEST_URDF_PATH");
  if (env != nullptr) {
    return env;
  }
  std::string path = __FILE__;
  auto pos = path.rfind("/rtc_controllers/");
  if (pos != std::string::npos) {
    return path.substr(0, pos) + "/rtc_urdf_bridge/test/urdf/serial_6dof.urdf";
  }
  return "serial_6dof.urdf";
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

TEST(PControllerConfig, LoadConfigWrongSizeKpIgnored) {
  // PController requires exactly 6 kp entries; a 3-entry list is rejected.
  rtc::PController ctrl(GetTestUrdfPath());
  const auto before = ctrl.get_gains();
  YAML::Node cfg = YAML::Load("kp: [1.0, 2.0, 3.0]");
  ctrl.LoadConfig(cfg);
  EXPECT_DOUBLE_EQ(ctrl.get_gains().kp[0], before.kp[0]);
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
  ctrl.SetDeviceTarget(-1, target);                            // negative → ignored
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
command_type: position
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
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kPosition);
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
damping: 0.02
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
  EXPECT_NEAR(g.damping, 0.02, 1e-12);
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
  EXPECT_DOUBLE_EQ(ctrl.get_gains().damping, before.damping);
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
  target[3] = 0.3;  // roll
  target[4] = -0.2;  // pitch
  target[5] = 0.5;  // yaw
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
damping: 0.05
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
  EXPECT_NEAR(g.damping, 0.05, 1e-12);
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
  EXPECT_DOUBLE_EQ(ctrl.get_gains().damping, before.damping);
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
