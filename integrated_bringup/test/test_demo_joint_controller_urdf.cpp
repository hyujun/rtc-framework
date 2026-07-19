// ── DemoJointController URDF-backed unit tests (#149) ────────────────────────
//
// Complements test_demo_joint_controller (empty-URDF sister): that fixture
// covers the grasp/sensor logic, but every model-dependent path — arm quintic
// trajectory with a registered TCP frame, hand fingertip FK + TF composition,
// virtual TCP, ToF snapshot population, E-STOP FK/TF-alive — stays dark
// without a Pinocchio model. This file drives those on the shared iiwa7_leap
// fixture (see iiwa7_leap_test_fixture.hpp). No ROS node / DDS involved.

#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/hand_sensor_layout.hpp"
#include "integrated_bringup/support/virtual_tcp.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <memory>

namespace {

using integrated_bringup::DemoJointController;
using integrated_bringup::kHandSensorValuesPerFingertipCapacity;
using integrated_bringup::VirtualTcpMode;
using rtc::ControllerOutput;
using rtc::ControllerState;

using integrated_bringup::testfx::kArmDof;
using integrated_bringup::testfx::kArmHome;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::kHandDof;
using integrated_bringup::testfx::MakeIiwa7LeapDeviceConfigs;
using integrated_bringup::testfx::MakeIiwa7LeapState;
using integrated_bringup::testfx::SetFingertipForce;
using integrated_bringup::testfx::SharedIiwa7LeapBuilder;
using integrated_bringup::testfx::SharedIiwa7LeapModelConfig;

// Required keys (estop / fsm) + test-friendly trajectory speeds. All-zero safe
// position so the E-STOP test observes motion away from kArmHome.
const char* const kJointYaml = R"(
robot_trajectory_speed: 2.0
hand_trajectory_speed: 3.0
robot_max_traj_velocity: 3.14
hand_max_traj_velocity: 6.28
estop:
  arm_safe_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fsm:
  contact_stop_release_eps: 0.005
  contact_stop_lpf_cutoff_hz: 20.0
command_type: "position"
topics:
  iiwa7:
    subscribe:
      - topic: "iiwa7/joint_goal"
        role: "target"
  leap:
    subscribe:
      - topic: "leap/joint_goal"
        role: "target"
)";

class JointControllerUrdfTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ctrl_ = std::make_unique<DemoJointController>("");
    // Production bring-up sequence — see iiwa7_leap_test_fixture.hpp.
    ctrl_->SetSystemModelConfig(SharedIiwa7LeapModelConfig());
    ctrl_->SetSharedModelBuilder(SharedIiwa7LeapBuilder());
    ctrl_->SetControlRate(1.0 / kDt);
    ctrl_->LoadConfig(YAML::Load(kJointYaml));
    ctrl_->SetDeviceNameConfigs(MakeIiwa7LeapDeviceConfigs());

    state_ = MakeIiwa7LeapState();
    // First tick: self-init (hold seeds from measured state).
    last_out_ = ctrl_->Compute(state_);
  }

  // Closed-loop tick: the plant is a perfect position tracker.
  ControllerOutput RunTicks(int n) {
    for (int i = 0; i < n; ++i) {
      state_.iteration += 1;
      last_out_ = ctrl_->Compute(state_);
      for (int j = 0; j < kArmDof; ++j) {
        state_.devices[0].positions[static_cast<std::size_t>(j)] =
            last_out_.devices[0].commands[static_cast<std::size_t>(j)];
      }
      if (last_out_.num_devices > 1 && last_out_.devices[1].num_channels == kHandDof) {
        for (int j = 0; j < kHandDof; ++j) {
          state_.devices[1].positions[static_cast<std::size_t>(j)] =
              last_out_.devices[1].commands[static_cast<std::size_t>(j)];
        }
      }
    }
    return last_out_;
  }

  std::unique_ptr<DemoJointController> ctrl_;
  ControllerState state_;
  ControllerOutput last_out_;
};

// Self-init holds the measured pose; with a model present the arm-tip TF and
// all four (serial-FK) fingertip TFs publish as valid.
TEST_F(JointControllerUrdfTest, SelfInitHoldsPoseAndPublishesTf) {
  auto out = RunTicks(5);
  EXPECT_TRUE(out.valid);
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                kArmHome[static_cast<std::size_t>(i)], 1e-6)
        << "arm joint " << i;
  }
  EXPECT_TRUE(out.arm_tip_pose_valid);
  EXPECT_FALSE(out.virtual_tcp_pose_valid);
  for (std::size_t f = 0; f < 4; ++f) {
    EXPECT_TRUE(out.task_link_pose_valid[f]) << "fingertip " << f;
  }
  // actual_task_positions carries the (base-relative) FK translation — a real
  // pose, not the identity fallback.
  const double norm2 = out.actual_task_positions[0] * out.actual_task_positions[0] +
                       out.actual_task_positions[1] * out.actual_task_positions[1] +
                       out.actual_task_positions[2] * out.actual_task_positions[2];
  EXPECT_GT(norm2, 1e-4);
}

// Joint-space quintic trajectory: a device-0 target reaches the goal exactly
// (rest-to-rest endpoints) within duration + margin.
TEST_F(JointControllerUrdfTest, ArmTrajectoryReachesJointTarget) {
  std::array<double, kArmDof> target = kArmHome;
  target[0] += 0.3;
  target[2] -= 0.25;
  ctrl_->SetDeviceTarget(0, target);

  // max_dist 0.3 rad @ speed 2.0 → duration ~0.18s (90 ticks); run 200.
  auto out = RunTicks(200);
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                target[static_cast<std::size_t>(i)], 1e-6)
        << "arm joint " << i;
  }
  // goal_positions lane mirrors the target slot.
  EXPECT_NEAR(out.devices[0].goal_positions[0], target[0], 1e-12);
}

// Constant-offset virtual TCP publishes and sits |offset| from the arm tip.
TEST_F(JointControllerUrdfTest, VirtualTcpConstantOffsetPublishes) {
  auto gains = ctrl_->get_gains();
  gains.vtcp.mode = VirtualTcpMode::kConstant;
  gains.vtcp.offset = {0.0, 0.0, 0.05};
  ctrl_->set_gains(gains);

  auto out = RunTicks(3);

  ASSERT_TRUE(out.virtual_tcp_pose_valid);
  ASSERT_TRUE(out.arm_tip_pose_valid);
  const double dx = out.virtual_tcp_pose.position[0] - out.arm_tip_pose.position[0];
  const double dy = out.virtual_tcp_pose.position[1] - out.arm_tip_pose.position[1];
  const double dz = out.virtual_tcp_pose.position[2] - out.arm_tip_pose.position[2];
  EXPECT_NEAR(std::sqrt(dx * dx + dy * dy + dz * dz), 0.05, 1e-6);
}

// A real raw sensor lane populates the ToF snapshot (the empty-URDF sister can
// only assert the negative gate — populate needs hand_handle_ FK).
TEST_F(JointControllerUrdfTest, ToFSnapshotPopulatedFromSensorLane) {
  auto& dev1 = state_.devices[1];
  dev1.num_inference_groups = 3;
  dev1.num_sensor_channels = 3 * static_cast<int>(kHandSensorValuesPerFingertipCapacity);
  for (int f = 0; f < 3; ++f) {
    SetFingertipForce(state_, f, 0.5f);
    const std::size_t base =
        static_cast<std::size_t>(f) * kHandSensorValuesPerFingertipCapacity + 8;
    dev1.sensor_data[base + 1] = 25;  // ToF A [mm]
    dev1.sensor_data[base + 2] = 50;  // ToF B [mm]
  }

  RunTicks(2);

  const auto& tof = ctrl_->GetToFSnapshotForTesting();
  ASSERT_TRUE(tof.populated);
  EXPECT_EQ(tof.num_fingers, 3);
  EXPECT_NEAR(tof.distances[0], 0.025, 1e-9);
  EXPECT_NEAR(tof.distances[1], 0.050, 1e-9);
  EXPECT_TRUE(tof.valid[0]);
}

// E-STOP: arm steps toward the (zero) safe position at the per-joint velocity
// limit; FK-based task-space log + arm-tip TF stay alive; fingertip/vtcp TFs
// are withheld; the hand holds its measured position.
TEST_F(JointControllerUrdfTest, EstopDrivesTowardSafePositionKeepsTf) {
  for (int i = 0; i < kHandDof; ++i) {
    state_.devices[1].positions[static_cast<std::size_t>(i)] = 0.4;
  }
  ctrl_->TriggerEstop();

  auto out = RunTicks(250);

  EXPECT_LT(out.devices[0].commands[1], 0.5);  // home 0.7 → toward 0.0
  EXPECT_GT(out.devices[0].commands[1], 0.0);
  EXPECT_NEAR(out.devices[0].goal_positions[1], 0.0, 1e-12);  // safe position
  EXPECT_TRUE(out.arm_tip_pose_valid);
  EXPECT_FALSE(out.virtual_tcp_pose_valid);
  for (std::size_t f = 0; f < out.task_link_pose_valid.size(); ++f) {
    EXPECT_FALSE(out.task_link_pose_valid[f]);
  }
  EXPECT_NEAR(out.devices[1].commands[0], 0.4, 1e-9);

  ctrl_->ClearEstop();
  out = RunTicks(3);
  EXPECT_TRUE(out.valid);
}

}  // namespace
