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

TEST(PController, UpdateGains) {
  rtc::PController ctrl(GetTestUrdfPath());

  std::vector<double> new_gains{50.0, 55.0, 60.0, 65.0, 70.0, 75.0};
  ctrl.UpdateGainsFromMsg(new_gains);

  auto readback = ctrl.GetCurrentGains();
  ASSERT_EQ(readback.size(), 6u);
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(readback[i], new_gains[i], 1e-12) << "Gain " << i;
  }
}

TEST(PController, UpdateGainsTooShort) {
  rtc::PController ctrl(GetTestUrdfPath());

  auto original = ctrl.GetCurrentGains();
  // Send fewer than 6 values -> gains should be unchanged
  std::vector<double> short_gains{10.0, 20.0};
  ctrl.UpdateGainsFromMsg(short_gains);

  auto readback = ctrl.GetCurrentGains();
  ASSERT_EQ(readback.size(), original.size());
  for (std::size_t i = 0; i < original.size(); ++i) {
    EXPECT_NEAR(readback[i], original[i], 1e-12)
        << "Gain " << i << " should be unchanged";
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

TEST(JointPD, UpdateGains) {
  rtc::JointPDController ctrl(GetTestUrdfPath());

  // Layout: [kp x6, kd x6, gravity(0/1), coriolis(0/1), trajectory_speed]
  std::vector<double> new_gains(15, 0.0);
  for (int i = 0; i < 6; ++i) {
    new_gains[static_cast<std::size_t>(i)] =
        50.0 + static_cast<double>(i); // kp
    new_gains[static_cast<std::size_t>(i + 6)] =
        10.0 + static_cast<double>(i); // kd
  }
  new_gains[12] = 1.0; // enable_gravity
  new_gains[13] = 0.0; // disable_coriolis
  new_gains[14] = 2.0; // trajectory_speed

  ctrl.UpdateGainsFromMsg(new_gains);
  auto readback = ctrl.GetCurrentGains();

  ASSERT_EQ(readback.size(), 15u);
  for (std::size_t i = 0; i < 15; ++i) {
    EXPECT_NEAR(readback[i], new_gains[i], 1e-12) << "Gain index " << i;
  }
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

TEST(Clik, UpdateGains) {
  rtc::ClikController::Gains gains;
  rtc::ClikController ctrl(GetTestUrdfPath(), gains);

  // Layout: [kp_translation x3, kp_rotation x3, damping, null_kp,
  //          enable_null_space(0/1), control_6dof(0/1),
  //          trajectory_speed, trajectory_angular_speed,
  //          max_traj_velocity, max_traj_angular_velocity] = 14
  std::vector<double> new_gains{
      2.0,  3.0, 4.0, // kp_translation
      1.5,  1.5, 1.5, // kp_rotation
      0.02,           // damping
      0.8,            // null_kp
      1.0,            // enable_null_space
      0.0,            // control_6dof
      0.2,            // trajectory_speed
      0.6,            // trajectory_angular_speed
      0.8,            // max_traj_velocity
      1.5             // max_traj_angular_velocity
  };

  ctrl.UpdateGainsFromMsg(new_gains);
  auto readback = ctrl.GetCurrentGains();

  ASSERT_EQ(readback.size(), 14u);
  for (std::size_t i = 0; i < 14; ++i) {
    EXPECT_NEAR(readback[i], new_gains[i], 1e-12) << "Gain index " << i;
  }
}

TEST(Clik, UpdateGainsTooShort) {
  rtc::ClikController::Gains gains;
  rtc::ClikController ctrl(GetTestUrdfPath(), gains);

  auto original = ctrl.GetCurrentGains();
  // Send fewer than 10 values -> gains should be unchanged
  std::vector<double> short_gains{1.0, 2.0, 3.0, 4.0, 5.0};
  ctrl.UpdateGainsFromMsg(short_gains);

  auto readback = ctrl.GetCurrentGains();
  ASSERT_EQ(readback.size(), original.size());
  for (std::size_t i = 0; i < original.size(); ++i) {
    EXPECT_NEAR(readback[i], original[i], 1e-12)
        << "Gain " << i << " should be unchanged";
  }
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

TEST(OSC, UpdateGains) {
  rtc::OperationalSpaceController::Gains gains;
  rtc::OperationalSpaceController ctrl(GetTestUrdfPath(), gains);

  // Layout: [kp_pos x3, kd_pos x3, kp_rot x3, kd_rot x3, damping,
  //          enable_gravity(0/1), trajectory_speed, trajectory_angular_speed,
  //          max_traj_velocity, max_traj_angular_velocity] = 18
  std::vector<double> new_gains{
      3.0,  3.0, 3.0, // kp_pos
      0.2,  0.2, 0.2, // kd_pos
      1.0,  1.0, 1.0, // kp_rot
      0.1,  0.1, 0.1, // kd_rot
      0.05,           // damping
      1.0,            // enable_gravity
      0.15,           // trajectory_speed
      0.7,            // trajectory_angular_speed
      0.6,            // max_traj_velocity
      1.2             // max_traj_angular_velocity
  };

  ctrl.UpdateGainsFromMsg(new_gains);
  auto readback = ctrl.GetCurrentGains();

  ASSERT_EQ(readback.size(), 18u);
  for (std::size_t i = 0; i < 18; ++i) {
    EXPECT_NEAR(readback[i], new_gains[i], 1e-12) << "Gain index " << i;
  }
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
