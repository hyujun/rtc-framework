// Unit tests for the four core rtc_controllers:
//   PController, JointPDController, ClikController, OperationalSpaceController
//
// Uses the serial_6dof.urdf (6-DOF serial arm, joints joint_1..joint_6,
// revolute Z-axis, base_link -> link_1 -> ... -> link_6 -> tool_link).

#include <gtest/gtest.h>

#include "rtc_controllers/direct/joint_pd_controller.hpp"
#include "rtc_controllers/direct/operational_space_controller.hpp"
#include "rtc_controllers/indirect/clik_controller.hpp"
#include "rtc_controllers/indirect/p_controller.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

// ═══════════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════════

static std::string GetTestUrdfPath() {
  const char *env = std::getenv("RTC_TEST_URDF_PATH");
  if (env)
    return env;
  // Resolve relative to this source file
  std::string path = __FILE__;
  auto pos = path.rfind("/rtc_controllers/");
  if (pos != std::string::npos) {
    return path.substr(0, pos) + "/rtc_urdf_bridge/test/urdf/serial_6dof.urdf";
  }
  return "serial_6dof.urdf"; // last resort
}

static rtc::ControllerState MakeState(int nj = 6, double dt = 0.002) {
  rtc::ControllerState state{};
  state.num_devices = 1;
  state.dt = dt;
  state.devices[0].num_channels = nj;
  state.devices[0].valid = true;
  // Positions/velocities/efforts default to zero (home position)
  return state;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PController Tests
// ═══════════════════════════════════════════════════════════════════════════════

TEST(PController, ComputeAtTarget) {
  rtc::PController ctrl(GetTestUrdfPath());
  auto state = MakeState();

  // Target == position (both zero) -> command should be near current position
  ctrl.InitializeHoldPosition(state);
  auto out = ctrl.Compute(state);

  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.command_type, rtc::CommandType::kPosition);

  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                state.devices[0].positions[static_cast<std::size_t>(i)], 1e-6)
        << "Joint " << i << " command should match position at target";
  }
}

TEST(PController, ComputeWithError) {
  rtc::PController ctrl(GetTestUrdfPath());
  auto state = MakeState();

  ctrl.InitializeHoldPosition(state);

  // Set a target offset for joint 0
  std::array<double, 6> target{0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  ctrl.SetDeviceTarget(0, target);

  auto out = ctrl.Compute(state);

  // Command for joint 0 should be pos + kp * error * dt
  // With default kp[0]=120, error=0.5, dt=0.002: 0 + 120 * 0.5 * 0.002 = 0.12
  const double expected = 0.0 + 120.0 * 0.5 * 0.002;
  EXPECT_NEAR(out.devices[0].commands[0], expected, 1e-6);

  // Other joints should remain near zero
  for (int i = 1; i < 6; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)], 0.0, 1e-6)
        << "Joint " << i;
  }
}

TEST(PController, SetDeviceTarget) {
  rtc::PController ctrl(GetTestUrdfPath());
  auto state = MakeState();

  ctrl.InitializeHoldPosition(state);

  std::array<double, 6> target{0.1, -0.2, 0.3, -0.1, 0.05, -0.15};
  ctrl.SetDeviceTarget(0, target);
  auto out = ctrl.Compute(state);

  // All joints should move toward target
  for (int i = 0; i < 6; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double error = target[ui] - state.devices[0].positions[ui];
    if (std::abs(error) > 1e-9) {
      // Command should be on the same side as the target
      EXPECT_GT(out.devices[0].commands[ui] * error, 0.0)
          << "Joint " << i << " should move toward target";
    }
  }
}

TEST(PController, InitializeHoldPosition) {
  rtc::PController ctrl(GetTestUrdfPath());
  auto state = MakeState();
  // Set non-zero positions
  state.devices[0].positions[0] = 0.3;
  state.devices[0].positions[1] = -0.2;

  ctrl.InitializeHoldPosition(state);
  auto out = ctrl.Compute(state);

  // Should hold current position (command ~= position)
  for (int i = 0; i < 6; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_NEAR(out.devices[0].commands[ui], state.devices[0].positions[ui],
                1e-6)
        << "Joint " << i;
  }
}

TEST(PController, Estop) {
  rtc::PController ctrl(GetTestUrdfPath());
  auto state = MakeState();
  state.devices[0].positions[0] = 0.5;

  ctrl.InitializeHoldPosition(state);
  // Set a different target
  std::array<double, 6> target{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ctrl.SetDeviceTarget(0, target);

  ctrl.TriggerEstop();
  EXPECT_TRUE(ctrl.IsEstopped());

  auto out = ctrl.Compute(state);
  // E-STOP: commands should equal current positions (hold in place)
  for (int i = 0; i < 6; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_NEAR(out.devices[0].commands[ui], state.devices[0].positions[ui],
                1e-6)
        << "Joint " << i << " E-STOP should hold current position";
  }
}

TEST(PController, EstopClear) {
  rtc::PController ctrl(GetTestUrdfPath());
  auto state = MakeState();

  ctrl.InitializeHoldPosition(state);
  std::array<double, 6> target{0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  ctrl.SetDeviceTarget(0, target);

  ctrl.TriggerEstop();
  EXPECT_TRUE(ctrl.IsEstopped());

  ctrl.ClearEstop();
  EXPECT_FALSE(ctrl.IsEstopped());

  // After clearing estop, normal control should resume
  auto out = ctrl.Compute(state);
  // Joint 0 should move toward target (command != current position)
  EXPECT_GT(out.devices[0].commands[0], 1e-6)
      << "After ClearEstop, control should resume toward target";
}

TEST(PController, SetGetGainsRoundTrip) {
  rtc::PController ctrl(GetTestUrdfPath());

  rtc::PController::Gains g;
  for (std::size_t i = 0; i < 6; ++i) {
    g.kp[i] = 50.0 + 5.0 * static_cast<double>(i);
  }
  ctrl.set_gains(g);
  const auto rb = ctrl.get_gains();
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(rb.kp[i], g.kp[i], 1e-12) << "Gain " << i;
  }
}

TEST(PController, Name) {
  rtc::PController ctrl(GetTestUrdfPath());
  EXPECT_EQ(ctrl.Name(), "PController");
}

TEST(PController, ZeroDt) {
  rtc::PController ctrl(GetTestUrdfPath());
  auto state = MakeState(6, 0.0); // dt = 0

  ctrl.InitializeHoldPosition(state);
  std::array<double, 6> target{0.3, 0.0, 0.0, 0.0, 0.0, 0.0};
  ctrl.SetDeviceTarget(0, target);

  // Should not crash, should produce a valid output
  auto out = ctrl.Compute(state);
  EXPECT_TRUE(out.valid);

  // With dt=0, P-control law: pos + kp * error * 0 = pos = 0
  // So commands should be near zero for all joints (since pos is 0 and dt is 0)
  EXPECT_NEAR(out.devices[0].commands[0], 0.0, 1e-6);
}

// ═══════════════════════════════════════════════════════════════════════════════
// JointPDController Tests
// ═══════════════════════════════════════════════════════════════════════════════

TEST(JointPD, ComputeAtTarget) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  auto state = MakeState();

  ctrl.InitializeHoldPosition(state);
  auto out = ctrl.Compute(state);

  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);

  // At target with zero velocity, PD output should be near zero
  // (no gravity comp by default, and error = 0)
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)], 0.0, 1e-3)
        << "Joint " << i << " torque should be ~0 at target";
  }
}

TEST(JointPD, ComputeWithError) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  auto state = MakeState();

  ctrl.InitializeHoldPosition(state);

  // Set a target offset from current position
  std::array<double, 6> target{0.3, 0.0, 0.0, 0.0, 0.0, 0.0};
  ctrl.SetDeviceTarget(0, target);

  // Run Compute twice to allow trajectory to be initialized and PD to produce
  // nonzero error (first call picks up new_target_, second sees trajectory)
  auto out1 = ctrl.Compute(state);
  auto out2 = ctrl.Compute(state);

  // At least one of these should have nonzero torque on joint 0
  // The trajectory interpolates, so after init joint 0 target != current
  double max_cmd = std::max(std::abs(out1.devices[0].commands[0]),
                            std::abs(out2.devices[0].commands[0]));
  EXPECT_GT(max_cmd, 1e-6)
      << "PD should produce nonzero torque when target != position";
}

TEST(JointPD, SetDeviceTarget) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  auto state = MakeState();

  ctrl.InitializeHoldPosition(state);

  std::array<double, 6> target{0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
  ctrl.SetDeviceTarget(0, target);

  // Run several cycles to simulate convergence direction
  rtc::ControllerOutput prev_out{};
  bool moved_toward_target = false;
  for (int iter = 0; iter < 5; ++iter) {
    auto out = ctrl.Compute(state);
    // Torque on joint 0 should be positive (pushing toward target=0.1 from
    // pos=0)
    if (out.devices[0].commands[0] > 1e-6) {
      moved_toward_target = true;
    }
    prev_out = out;
  }
  EXPECT_TRUE(moved_toward_target) << "JointPD should produce positive torque "
                                      "to move toward positive target";
}

TEST(JointPD, InitializeHoldPosition) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  auto state = MakeState();
  state.devices[0].positions[0] = 0.5;
  state.devices[0].positions[1] = -0.3;

  ctrl.InitializeHoldPosition(state);
  auto out = ctrl.Compute(state);

  // At hold position, PD error ~0, torque ~0
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)], 0.0, 1e-3)
        << "Joint " << i << " should hold with near-zero torque";
  }
}

TEST(JointPD, Estop) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  // OnDeviceConfigsSet not called -> safe_position_ will be empty when
  // ComputeEstop is called. Need to call it for safe_position_ to be set.
  ctrl.OnDeviceConfigsSet();

  auto state = MakeState();
  state.devices[0].positions[0] = 0.5;

  ctrl.InitializeHoldPosition(state);

  ctrl.TriggerEstop();
  EXPECT_TRUE(ctrl.IsEstopped());

  auto out = ctrl.Compute(state);
  EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);

  // In E-STOP mode, the controller drives toward safe_position_ (zeros by
  // default). safe_position_[0]=0.0, current=0.5, so PD error = 0.0 - 0.5 =
  // -0.5
  // => torque should be negative for joint 0
  EXPECT_LT(out.devices[0].commands[0], 0.0)
      << "E-STOP should produce torque driving toward safe position (zero)";
}

TEST(JointPD, SetGetGainsRoundTrip) {
  rtc::JointPDController ctrl(GetTestUrdfPath());

  rtc::JointPDController::Gains gains;
  for (int i = 0; i < 6; ++i) {
    gains.kp[static_cast<std::size_t>(i)] = 50.0 + static_cast<double>(i);
    gains.kd[static_cast<std::size_t>(i)] = 10.0 + static_cast<double>(i);
  }
  gains.enable_gravity_compensation = true;
  gains.enable_coriolis_compensation = false;
  gains.trajectory_speed = 2.0;

  ctrl.set_gains(gains);
  const auto rb = ctrl.get_gains();
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(rb.kp[static_cast<std::size_t>(i)],
                gains.kp[static_cast<std::size_t>(i)], 1e-12);
    EXPECT_NEAR(rb.kd[static_cast<std::size_t>(i)],
                gains.kd[static_cast<std::size_t>(i)], 1e-12);
  }
  EXPECT_TRUE(rb.enable_gravity_compensation);
  EXPECT_FALSE(rb.enable_coriolis_compensation);
  EXPECT_DOUBLE_EQ(rb.trajectory_speed, 2.0);
}

TEST(JointPD, CommandType) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kTorque);
}

TEST(JointPD, Name) {
  rtc::JointPDController ctrl(GetTestUrdfPath());
  EXPECT_EQ(ctrl.Name(), "JointPDController");
}

// ═══════════════════════════════════════════════════════════════════════════════
// ClikController Tests
// ═══════════════════════════════════════════════════════════════════════════════

TEST(Clik, ComputeAtTarget) {
  rtc::ClikController::Gains gains;
  rtc::ClikController ctrl(GetTestUrdfPath(), gains);

  auto state = MakeState();

  // Initialize hold -> target becomes current FK position
  ctrl.InitializeHoldPosition(state);
  auto out = ctrl.Compute(state);

  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.command_type, rtc::CommandType::kPosition);

  // At target, commands should be near current joint positions (zero)
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                state.devices[0].positions[static_cast<std::size_t>(i)], 0.01)
        << "Joint " << i << " should stay near current position when at target";
  }
}

TEST(Clik, ComputeWithTaskError) {
  rtc::ClikController::Gains gains;
  gains.kp_translation = {5.0, 5.0, 5.0};
  gains.trajectory_speed = 1.0;
  rtc::ClikController ctrl(GetTestUrdfPath(), gains);

  auto state = MakeState();

  ctrl.InitializeHoldPosition(state);
  auto out0 = ctrl.Compute(state);

  // Set target offset from current TCP position (3-DOF mode: x,y,z)
  std::array<double, 6> target{};
  target[0] = out0.actual_task_positions[0] + 0.1;
  target[1] = out0.actual_task_positions[1] + 0.05;
  target[2] = out0.actual_task_positions[2] - 0.05;
  ctrl.SetDeviceTarget(0, target);

  // After SetDeviceTarget, the goal_positions in output should reflect
  // the new target, proving the controller received and processed it.
  auto out1 = ctrl.Compute(state);
  bool goal_updated = false;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(out1.task_goal_positions[static_cast<std::size_t>(i)] -
                 out0.actual_task_positions[static_cast<std::size_t>(i)]) >
        1e-6) {
      goal_updated = true;
    }
  }
  EXPECT_TRUE(goal_updated)
      << "CLIK task_goal_positions should update after SetDeviceTarget";
}

TEST(Clik, InitializeHoldPosition) {
  rtc::ClikController::Gains gains;
  rtc::ClikController ctrl(GetTestUrdfPath(), gains);

  auto state = MakeState();
  state.devices[0].positions[0] = 0.3;
  state.devices[0].positions[1] = -0.2;

  ctrl.InitializeHoldPosition(state);
  auto out = ctrl.Compute(state);

  // After InitializeHoldPosition, commands should hold near current joints
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                state.devices[0].positions[static_cast<std::size_t>(i)], 0.05)
        << "Joint " << i;
  }
}

TEST(Clik, Estop) {
  rtc::ClikController::Gains gains;
  rtc::ClikController ctrl(GetTestUrdfPath(), gains);
  // Populate safe_position_ and max_joint_velocity_ via OnDeviceConfigsSet
  ctrl.OnDeviceConfigsSet();

  auto state = MakeState();
  state.devices[0].positions[0] = 0.5;

  ctrl.InitializeHoldPosition(state);

  ctrl.TriggerEstop();
  EXPECT_TRUE(ctrl.IsEstopped());

  auto out = ctrl.Compute(state);
  EXPECT_EQ(out.command_type, rtc::CommandType::kPosition);

  // In E-STOP, CLIK drives toward safe_position_ (zeros by default).
  // safe_position_[0]=0 and current=0.5, so command[0] should be < 0.5
  // (moving toward safe position)
  EXPECT_LT(out.devices[0].commands[0], 0.5)
      << "E-STOP should move joint 0 toward safe position (0.0)";
}

TEST(Clik, SetGetGainsRoundTrip) {
  rtc::ClikController::Gains init;
  rtc::ClikController ctrl(GetTestUrdfPath(), init);

  rtc::ClikController::Gains gains;
  gains.kp_translation = {2.0, 3.0, 4.0};
  gains.kp_rotation = {1.5, 1.5, 1.5};
  gains.damping = 0.02;
  gains.null_kp = 0.8;
  gains.enable_null_space = true;
  gains.control_6dof = false;
  gains.trajectory_speed = 0.2;
  gains.trajectory_angular_speed = 0.6;
  gains.max_traj_velocity = 0.8;
  gains.max_traj_angular_velocity = 1.5;

  ctrl.set_gains(gains);
  const auto rb = ctrl.get_gains();
  EXPECT_NEAR(rb.kp_translation[0], 2.0, 1e-12);
  EXPECT_NEAR(rb.kp_rotation[2], 1.5, 1e-12);
  EXPECT_NEAR(rb.damping, 0.02, 1e-12);
  EXPECT_TRUE(rb.enable_null_space);
  EXPECT_FALSE(rb.control_6dof);
  EXPECT_NEAR(rb.trajectory_speed, 0.2, 1e-12);
}

TEST(Clik, CommandType) {
  rtc::ClikController::Gains gains;
  rtc::ClikController ctrl(GetTestUrdfPath(), gains);
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kPosition);
}

TEST(Clik, Name) {
  rtc::ClikController::Gains gains;
  rtc::ClikController ctrl(GetTestUrdfPath(), gains);
  EXPECT_EQ(ctrl.Name(), "ClikController");
}

// ═══════════════════════════════════════════════════════════════════════════════
// OperationalSpaceController Tests
// ═══════════════════════════════════════════════════════════════════════════════

TEST(OSC, ComputeAtTarget) {
  rtc::OperationalSpaceController::Gains gains;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), gains);

  auto state = MakeState();

  ctrl.InitializeHoldPosition(state);
  auto out = ctrl.Compute(state);

  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);

  // At target, commands should be near current joint positions (since OSC
  // integrates: q_cmd = q + clamp(dq, +-v_max) * dt, and dq should be ~0)
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                state.devices[0].positions[static_cast<std::size_t>(i)], 0.01)
        << "Joint " << i << " should stay near current position when at target";
  }
}

TEST(OSC, ComputeWithPosError) {
  rtc::OperationalSpaceController::Gains gains;
  gains.kp_pos = {5.0, 5.0, 5.0};
  gains.kd_pos = {0.1, 0.1, 0.1};
  gains.trajectory_speed = 1.0; // fast trajectory for testing
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), gains);

  auto state = MakeState();
  // Use non-zero joint config for well-conditioned Jacobian
  ctrl.InitializeHoldPosition(state);
  auto out0 = ctrl.Compute(state);

  // Set target offset from current TCP pose
  std::array<double, 6> target{};
  target[0] = out0.actual_task_positions[0] + 0.1;
  target[1] = out0.actual_task_positions[1] + 0.05;
  target[2] = out0.actual_task_positions[2] - 0.05;
  target[3] = out0.actual_task_positions[3];
  target[4] = out0.actual_task_positions[4];
  target[5] = out0.actual_task_positions[5];
  ctrl.SetDeviceTarget(0, target);

  // After SetDeviceTarget, task_goal_positions should reflect the new target
  auto out1 = ctrl.Compute(state);
  bool goal_updated = false;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(out1.task_goal_positions[static_cast<std::size_t>(i)] -
                 out0.actual_task_positions[static_cast<std::size_t>(i)]) >
        1e-6) {
      goal_updated = true;
    }
  }
  EXPECT_TRUE(goal_updated)
      << "OSC task_goal_positions should update after SetDeviceTarget";
}

TEST(OSC, InitializeHoldPosition) {
  rtc::OperationalSpaceController::Gains gains;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), gains);

  auto state = MakeState();
  state.devices[0].positions[0] = 0.3;
  state.devices[0].positions[1] = -0.2;

  ctrl.InitializeHoldPosition(state);
  auto out = ctrl.Compute(state);

  // After InitializeHoldPosition, commands should stay near current joints
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                state.devices[0].positions[static_cast<std::size_t>(i)], 0.05)
        << "Joint " << i;
  }
}

TEST(OSC, Estop) {
  rtc::OperationalSpaceController::Gains gains;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), gains);
  // Populate safe_position_ and max_joint_velocity_ via OnDeviceConfigsSet
  ctrl.OnDeviceConfigsSet();

  auto state = MakeState();
  state.devices[0].positions[0] = 0.5;

  ctrl.InitializeHoldPosition(state);

  ctrl.TriggerEstop();
  EXPECT_TRUE(ctrl.IsEstopped());

  auto out = ctrl.Compute(state);
  EXPECT_EQ(out.command_type, rtc::CommandType::kTorque);

  // In E-STOP, OSC drives toward safe_position_ (zeros by default).
  // safe_position_[0]=0 and current=0.5, so command[0] should be < 0.5
  EXPECT_LT(out.devices[0].commands[0], 0.5)
      << "E-STOP should move joint 0 toward safe position (0.0)";
}

TEST(OSC, SetGetGainsRoundTrip) {
  rtc::OperationalSpaceController::Gains init;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), init);

  rtc::OperationalSpaceController::Gains gains;
  gains.kp_pos = {3.0, 3.0, 3.0};
  gains.kd_pos = {0.2, 0.2, 0.2};
  gains.kp_rot = {1.0, 1.0, 1.0};
  gains.kd_rot = {0.1, 0.1, 0.1};
  gains.damping = 0.05;
  gains.enable_gravity_compensation = true;
  gains.trajectory_speed = 0.15;
  gains.trajectory_angular_speed = 0.7;
  gains.max_traj_velocity = 0.6;
  gains.max_traj_angular_velocity = 1.2;

  ctrl.set_gains(gains);
  const auto rb = ctrl.get_gains();
  EXPECT_NEAR(rb.kp_pos[0], 3.0, 1e-12);
  EXPECT_NEAR(rb.kd_pos[1], 0.2, 1e-12);
  EXPECT_NEAR(rb.kp_rot[2], 1.0, 1e-12);
  EXPECT_NEAR(rb.damping, 0.05, 1e-12);
  EXPECT_TRUE(rb.enable_gravity_compensation);
  EXPECT_DOUBLE_EQ(rb.trajectory_speed, 0.15);
}

TEST(OSC, CommandType) {
  rtc::OperationalSpaceController::Gains gains;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), gains);
  EXPECT_EQ(ctrl.GetCommandType(), rtc::CommandType::kTorque);
}

TEST(OSC, Name) {
  rtc::OperationalSpaceController::Gains gains;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), gains);
  EXPECT_EQ(ctrl.Name(), "OperationalSpaceController");
}
