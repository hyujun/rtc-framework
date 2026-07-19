// ── DemoTaskController unit tests (#149) ─────────────────────────────────────
//
// Two tiers, no ROS node / DDS involved (deterministic; no domain isolation
// needed):
//
//   1. URDF-backed fixture (iiwa7_leap serial profile, mirroring
//      test_task_arm_cache_equivalence's model topology): drives the CM
//      bring-up sequence by hand (SetSystemModelConfig → SetSharedModelBuilder
//      → SetControlRate → LoadConfig → SetDeviceNameConfigs) so the combined
//      -model cache is fresh and ComputeControl's CLIK / trajectory / grasp /
//      contact_stop / ToF paths actually run — unlike the joint/wbc sister
//      tests, DemoTaskController gates ALL of ComputeControl behind the cache
//      (compute.cpp E-stop/cache-readiness check), so an empty-URDF controller
//      exercises none of it.
//   2. Model-less tests: E-STOP wire path without arm_handle_ and the
//      LoadConfig YAML validation throws.
//
// The arm is closed-loop simulated as a perfect position tracker (measured ←
// commanded each tick); the hand is optionally pinned (external object) for
// the contact_stop scenarios, mirroring test_demo_joint_controller.

#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/controllers/hand_sensor_layout.hpp"
#include "integrated_bringup/support/virtual_tcp.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math.hpp>
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

namespace {

using integrated_bringup::DemoTaskController;
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

// ── Controller YAML (inline — required keys + test-friendly speeds) ──────────
// estop.arm_safe_position is all-zero so the E-STOP test observes motion away
// from kArmHome. Trajectory speeds are faster than the shipped iiwa7_leap
// config to keep tick counts small; the control law is speed-agnostic.
const char* const kTaskYaml = R"(
kp_translation: [5.0, 5.0, 5.0]
kp_rotation: [2.0, 2.0, 2.0]
damping: 0.01
null_kp: 0.5
enable_null_space: true
control_6dof: false
trajectory_speed: 0.5
trajectory_angular_speed: 2.0
hand_trajectory_speed: 3.0
max_traj_velocity: 1.0
max_traj_angular_velocity: 4.0
hand_max_traj_velocity: 6.28
estop:
  arm_safe_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fsm:
  pi_rotation_margin: 0.15
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
logs:
  - msg_type: rtc_msgs/DeviceStateLog
    instance: iiwa7_state
)";

// ZYX (yaw·pitch·roll) rotation — matches DrainTargetSlot's 6-DOF target
// composition and pinocchio::rpy::matrixToRpy.
Eigen::Matrix3d RpyToMatrix(double r, double p, double y) {
  return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

double GeodesicAngle(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  return Eigen::AngleAxisd(a.transpose() * b).angle();
}

// ── URDF-backed fixture ──────────────────────────────────────────────────────

class TaskControllerUrdfTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ctrl_ = std::make_unique<DemoTaskController>("", DemoTaskController::Gains{});
    // Production bring-up sequence (rt_controller_node_params.cpp): system
    // model + shared builder → PreConfigure(LoadConfig) → control rate →
    // device configs (triggers OnDeviceConfigsSet: reorder map + TCP frame
    // registration on the combined cache).
    ctrl_->SetSystemModelConfig(SharedIiwa7LeapModelConfig());
    ctrl_->SetSharedModelBuilder(SharedIiwa7LeapBuilder());
    ctrl_->SetControlRate(1.0 / kDt);
    ctrl_->LoadConfig(YAML::Load(kTaskYaml));
    ctrl_->SetDeviceNameConfigs(MakeIiwa7LeapDeviceConfigs());

    state_ = MakeIiwa7LeapState();
    // First tick: cache Update + arm/hand hold self-init.
    last_out_ = ctrl_->Compute(state_);
  }

  // Closed-loop tick: the plant is a perfect position tracker. The hand can be
  // pinned (feedback_hand=false) to emulate an object blocking the fingers.
  ControllerOutput RunTicks(int n, bool feedback_hand = true) {
    for (int i = 0; i < n; ++i) {
      state_.iteration += 1;
      last_out_ = ctrl_->Compute(state_);
      for (int j = 0; j < kArmDof; ++j) {
        state_.devices[0].positions[static_cast<std::size_t>(j)] =
            last_out_.devices[0].commands[static_cast<std::size_t>(j)];
      }
      if (feedback_hand && last_out_.num_devices > 1 &&
          last_out_.devices[1].num_channels == kHandDof) {
        for (int j = 0; j < kHandDof; ++j) {
          state_.devices[1].positions[static_cast<std::size_t>(j)] =
              last_out_.devices[1].commands[static_cast<std::size_t>(j)];
        }
      }
    }
    return last_out_;
  }

  Eigen::Matrix3d ActualRotation() const {
    return RpyToMatrix(last_out_.actual_task_positions[3], last_out_.actual_task_positions[4],
                       last_out_.actual_task_positions[5]);
  }

  std::unique_ptr<DemoTaskController> ctrl_;
  ControllerState state_;
  ControllerOutput last_out_;
};

// ── Self-init & hold ─────────────────────────────────────────────────────────

// First tick seeds the TCP hold from the fresh combined-model cache; with no
// target the arm must hold the measured pose and publish a valid arm-tip TF +
// all four (serial-FK) fingertip TFs.
TEST_F(TaskControllerUrdfTest, SelfInitHoldsMeasuredPoseAndPublishesTf) {
  auto out = RunTicks(5);
  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.num_devices, 2);
  EXPECT_EQ(out.devices[0].goal_type, rtc::GoalType::kTask);
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                kArmHome[static_cast<std::size_t>(i)], 1e-6)
        << "arm joint " << i << " drifted while holding";
  }
  EXPECT_TRUE(out.arm_tip_pose_valid);
  EXPECT_FALSE(out.virtual_tcp_pose_valid);
  for (std::size_t f = 0; f < 4; ++f) {
    EXPECT_TRUE(out.task_link_pose_valid[f]) << "fingertip " << f;
  }
  // No sensor stream → no fingertips, no grasp.
  EXPECT_EQ(ctrl_->GetGraspStateForTesting().num_fingertips, 0);
  EXPECT_FALSE(ctrl_->GetGraspStateForTesting().grasp_detected);
  EXPECT_FALSE(ctrl_->GetToFSnapshotForTesting().populated);
}

// ── CLIK tracking ────────────────────────────────────────────────────────────

// 3-DOF mode: a translation-only TCP target (values[0..2]) with null-space
// references (values[3..6]) converges through the quintic trajectory + damped
// pseudoinverse + null-space secondary task.
TEST_F(TaskControllerUrdfTest, Clik3DofConvergesToTranslationTarget) {
  const auto tcp0 = ctrl_->tcp_position();
  std::array<double, kArmDof> target{};
  target[0] = tcp0[0] + 0.03;
  target[1] = tcp0[1] - 0.02;
  target[2] = tcp0[2] + 0.02;
  for (int i = 3; i < kArmDof; ++i) {
    target[static_cast<std::size_t>(i)] = kArmHome[static_cast<std::size_t>(i)];
  }
  ctrl_->SetDeviceTarget(0, target);

  RunTicks(600);

  const auto tcp = ctrl_->tcp_position();
  EXPECT_NEAR(tcp[0], target[0], 1e-3);
  EXPECT_NEAR(tcp[1], target[1], 1e-3);
  EXPECT_NEAR(tcp[2], target[2], 1e-3);
  const auto err = ctrl_->position_error();
  EXPECT_LT(std::abs(err[0]), 1e-3);
  // Publish-side goal mirrors the commanded TCP target.
  EXPECT_NEAR(last_out_.task_goal_positions[0], target[0], 1e-12);
}

// 6-DOF mode: position + orientation target (rpy). The goal rotation is the
// current tool rotation composed with a 0.3 rad Z rotation — well under the
// π-split margin, so the single-segment path runs.
TEST_F(TaskControllerUrdfTest, Clik6DofConvergesToPoseTarget) {
  auto gains = ctrl_->get_gains();
  gains.control_6dof = true;
  ctrl_->set_gains(gains);

  RunTicks(2);  // one output with actual_task_positions under 6-DOF read path
  const Eigen::Matrix3d r0 = ActualRotation();
  const Eigen::Matrix3d r_goal =
      r0 * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d rpy_goal = pinocchio::rpy::matrixToRpy(r_goal);

  const auto tcp0 = ctrl_->tcp_position();
  const std::array<double, 6> target = {tcp0[0] + 0.02, tcp0[1],     tcp0[2] - 0.02,
                                        rpy_goal[0],    rpy_goal[1], rpy_goal[2]};
  ctrl_->SetDeviceTarget(0, target);

  RunTicks(1200);

  const auto tcp = ctrl_->tcp_position();
  EXPECT_NEAR(tcp[0], target[0], 2e-3);
  EXPECT_NEAR(tcp[2], target[2], 2e-3);
  EXPECT_LT(GeodesicAngle(ActualRotation(), r_goal), 0.02);
  // 6-DOF publish goal carries the target orientation rpy.
  EXPECT_NEAR(last_out_.task_goal_positions[5], rpy_goal[2], 1e-9);
}

// Near-π rotation triggers the split-trajectory defense (two rest-to-rest
// segments through the midpoint) and the segment-transition handover.
TEST_F(TaskControllerUrdfTest, PiRotationSplitTrajectoryConverges) {
  auto gains = ctrl_->get_gains();
  gains.control_6dof = true;
  ctrl_->set_gains(gains);

  RunTicks(2);
  const Eigen::Matrix3d r0 = ActualRotation();
  const double angle = M_PI - 0.05;  // > π - pi_rotation_margin (0.15) → split
  const Eigen::Matrix3d r_goal =
      r0 * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d rpy_goal = pinocchio::rpy::matrixToRpy(r_goal);

  const auto tcp0 = ctrl_->tcp_position();
  const std::array<double, 6> target = {tcp0[0],     tcp0[1],     tcp0[2],
                                        rpy_goal[0], rpy_goal[1], rpy_goal[2]};
  ctrl_->SetDeviceTarget(0, target);

  RunTicks(2400);

  EXPECT_LT(GeodesicAngle(ActualRotation(), r_goal), 0.05);
  const auto tcp = ctrl_->tcp_position();
  EXPECT_NEAR(tcp[0], target[0], 5e-3);
}

// ── Hand trajectory ──────────────────────────────────────────────────────────

// With no contact, the hand quintic trajectory drives every finger to the
// commanded target (contact_stop stays disengaged).
TEST_F(TaskControllerUrdfTest, HandTrajectoryReachesTarget) {
  std::array<double, kHandDof> target{};
  target.fill(0.5);
  ctrl_->SetDeviceTarget(1, target);

  auto out = RunTicks(250);
  for (int i = 0; i < kHandDof; ++i) {
    EXPECT_NEAR(out.devices[1].commands[static_cast<std::size_t>(i)], 0.5, 1e-3)
        << "hand joint " << i;
  }
  // Publish lanes mirror the computed hand trajectory.
  EXPECT_NEAR(out.devices[1].trajectory_positions[0], 0.5, 1e-3);
  EXPECT_NEAR(out.devices[1].goal_positions[0], 0.5, 1e-12);
}

// ── Grasp detection (force-only capability) ──────────────────────────────────

// leap declares has_native_contact=false → detection falls back to |F| >
// grasp_force_threshold and publishes a derived binary contact_flag. A
// disabled fingertip (inference_enable=false) must contribute nothing.
TEST_F(TaskControllerUrdfTest, ForceOnlyGraspDetectionDerivesContactFlag) {
  state_.devices[1].num_inference_groups = 3;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 5.0f);
  SetFingertipForce(state_, 1, 5.0f);
  state_.devices[1].inference_enable[2] = false;

  RunTicks(2);

  const auto& gs = ctrl_->GetGraspStateForTesting();
  EXPECT_EQ(gs.num_fingertips, 3);
  EXPECT_EQ(gs.num_active_contacts, 2);
  EXPECT_TRUE(gs.grasp_detected);
  EXPECT_NEAR(gs.force_magnitude[0], 5.0f, 1e-4f);
  EXPECT_NEAR(gs.max_force, 5.0f, 1e-4f);
  // Derived binary flag despite the zero native slot.
  EXPECT_NEAR(gs.contact_flag[0], 1.0f, 1e-5f);
  EXPECT_TRUE(gs.inference_valid[0]);
  EXPECT_FALSE(gs.inference_valid[2]);
  EXPECT_NEAR(gs.contact_flag[2], 0.0f, 1e-6f);
}

// ── contact_stop (freeze / latch / release gate) ─────────────────────────────

namespace {
constexpr double kHandStart = 0.3;
constexpr double kHandGoal = 0.8;
}  // namespace

class TaskContactStopTest : public TaskControllerUrdfTest {
 protected:
  // Hand at 0.3 with 2 fingertips in firm contact, commanded toward 0.8. The
  // hand is pinned at 0.3 (object blocks it) — feedback_hand=false in ticks.
  void PrimeContact() {
    for (int i = 0; i < kHandDof; ++i) {
      state_.devices[1].positions[static_cast<std::size_t>(i)] = kHandStart;
    }
    state_.devices[1].num_inference_groups = 2;
    state_.devices[1].num_sensor_channels = 0;
    SetFingertipForce(state_, 0, 5.0f);
    SetFingertipForce(state_, 1, 5.0f);

    std::array<double, kHandDof> target{};
    target.fill(kHandGoal);
    ctrl_->SetDeviceTarget(1, target);
  }
};

// Contact present → the hand freezes at the LPF'd measured position instead of
// following the trajectory to the 0.8 goal.
TEST_F(TaskContactStopTest, ContactFreezesHandAtMeasuredPosition) {
  PrimeContact();
  auto out = RunTicks(300, /*feedback_hand=*/false);

  EXPECT_TRUE(ctrl_->GetGraspStateForTesting().grasp_detected);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-4);
}

// Latch: after contact drops (object slips) the hold persists — the command
// must NOT snap to the goal-advanced trajectory.
TEST_F(TaskContactStopTest, LatchHoldsAfterContactLost) {
  PrimeContact();
  RunTicks(150, /*feedback_hand=*/false);

  SetFingertipForce(state_, 0, 0.0f);
  SetFingertipForce(state_, 1, 0.0f);
  auto out = RunTicks(300, /*feedback_hand=*/false);

  EXPECT_FALSE(ctrl_->GetGraspStateForTesting().grasp_detected);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-3);
  EXPECT_LT(out.devices[1].commands[0], 0.5);
}

// Release-phase gate: an opening goal on every gate joint (idx {1,4,7}, signs
// {+1,-1,-1} — built-in shared defaults) drops the latch even while contact
// force persists, and the trajectory drives the gated joints off the hold.
TEST_F(TaskContactStopTest, ReleaseGateClearsLatchDespiteContact) {
  PrimeContact();
  RunTicks(150, /*feedback_hand=*/false);

  std::array<double, kHandDof> open{};
  open.fill(kHandStart);
  open[1] = kHandStart + 0.3;   // +1 → target > actual = loosening
  open[4] = kHandStart - 0.25;  // -1 → target < actual = loosening
  open[7] = kHandStart - 0.25;  // -1
  ctrl_->SetDeviceTarget(1, open);
  auto out = RunTicks(300, /*feedback_hand=*/false);

  EXPECT_GT(out.devices[1].commands[1], kHandStart + 0.1);
  EXPECT_LT(out.devices[1].commands[4], kHandStart - 0.1);
}

// Hand E-STOP (independent of the arm E-STOP) drops the contact latch so a
// post-clear resume honours fresh trajectory goals instead of the stale hold.
TEST_F(TaskContactStopTest, HandEstopDropsLatch) {
  PrimeContact();
  RunTicks(150, /*feedback_hand=*/false);

  SetFingertipForce(state_, 0, 0.0f);  // contact gone, latch persists…
  SetFingertipForce(state_, 1, 0.0f);
  RunTicks(50, /*feedback_hand=*/false);

  ctrl_->SetHandEstop(true);          // …until the hand E-STOP clears it
  EXPECT_FALSE(ctrl_->IsEstopped());  // controller-level E-STOP untouched
  RunTicks(2, /*feedback_hand=*/false);
  ctrl_->SetHandEstop(false);

  auto out = RunTicks(400, /*feedback_hand=*/false);
  // Latch dropped → trajectory resumed toward the 0.8 goal.
  EXPECT_GT(out.devices[1].commands[0], 0.5);
}

// Hand-state dropout while latched: the last hold position is kept (safety)
// and the controller survives the invalid-device tick.
TEST_F(TaskContactStopTest, HandDropoutWhileLatchedHoldsPosition) {
  PrimeContact();
  RunTicks(150, /*feedback_hand=*/false);

  state_.devices[1].valid = false;
  auto out = RunTicks(5, /*feedback_hand=*/false);
  EXPECT_EQ(out.devices[1].num_channels, 0);  // no wire command without a device

  state_.devices[1].valid = true;
  out = RunTicks(5, /*feedback_hand=*/false);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-3);
}

// ── ToF snapshot ─────────────────────────────────────────────────────────────

// A real raw sensor lane (baro+ToF stride) alongside ≥3 inference groups
// populates the 3-finger × 2-sensor snapshot with metre-converted distances
// and fingertip poses.
TEST_F(TaskControllerUrdfTest, ToFSnapshotPopulatedFromSensorLane) {
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
  EXPECT_EQ(tof.sensors_per_finger, 2);
  EXPECT_NEAR(tof.distances[0], 0.025, 1e-9);
  EXPECT_NEAR(tof.distances[1], 0.050, 1e-9);
  EXPECT_TRUE(tof.valid[0]);
  EXPECT_TRUE(tof.valid[1]);
  // Fingertip pose rides along (unit quaternion).
  const auto& q = tof.tip_poses[0].quaternion;
  EXPECT_NEAR(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3], 1.0, 1e-9);
}

// A force-only stream (no raw sensor lane) must NOT publish a junk snapshot.
TEST_F(TaskControllerUrdfTest, ToFSnapshotWithheldWithoutSensorLane) {
  state_.devices[1].num_inference_groups = 4;
  state_.devices[1].num_sensor_channels = 0;
  for (int f = 0; f < 4; ++f) {
    SetFingertipForce(state_, f, 0.5f);
  }
  RunTicks(2);
  EXPECT_FALSE(ctrl_->GetToFSnapshotForTesting().populated);
}

// ── Virtual TCP ──────────────────────────────────────────────────────────────

// Constant-offset virtual TCP: the control point shifts by |offset| from the
// arm tip and is published as a valid TF source.
TEST_F(TaskControllerUrdfTest, VirtualTcpConstantOffsetPublishes) {
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

// ── E-STOP ───────────────────────────────────────────────────────────────────

// E-STOP drives the arm toward the YAML safe position (all-zero here) at the
// per-joint velocity limit while keeping the arm-tip TF alive and withholding
// vtcp/fingertip TFs; the hand holds its measured position.
TEST_F(TaskControllerUrdfTest, EstopDrivesTowardSafePositionKeepsTf) {
  for (int i = 0; i < kHandDof; ++i) {
    state_.devices[1].positions[static_cast<std::size_t>(i)] = 0.4;
  }
  ctrl_->TriggerEstop();
  EXPECT_TRUE(ctrl_->IsEstopped());

  auto out = RunTicks(250);

  // Joint 1 (home 0.7, safe 0.0) has moved measurably toward zero.
  EXPECT_LT(out.devices[0].commands[1], 0.5);
  EXPECT_GT(out.devices[0].commands[1], 0.0);
  EXPECT_EQ(out.devices[0].goal_type, rtc::GoalType::kJoint);
  EXPECT_TRUE(out.arm_tip_pose_valid);
  EXPECT_FALSE(out.virtual_tcp_pose_valid);
  for (std::size_t f = 0; f < out.task_link_pose_valid.size(); ++f) {
    EXPECT_FALSE(out.task_link_pose_valid[f]);
  }
  // Hand holds its measured position under E-STOP.
  EXPECT_NEAR(out.devices[1].commands[0], 0.4, 1e-9);

  ctrl_->ClearEstop();
  EXPECT_FALSE(ctrl_->IsEstopped());
  out = RunTicks(3);
  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.devices[0].goal_type, rtc::GoalType::kTask);
}

// ── Single-device configuration ──────────────────────────────────────────────

// num_devices == 1: every devices[1] block is skipped and the hand self-init
// latches immediately — no crash, valid arm output.
TEST_F(TaskControllerUrdfTest, SingleDeviceStateSkipsHandPaths) {
  state_.num_devices = 1;
  auto out = RunTicks(5);
  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.num_devices, 1);
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                kArmHome[static_cast<std::size_t>(i)], 1e-6);
  }
}

// ── Model-less tier ──────────────────────────────────────────────────────────

// Without LoadConfig/model the E-STOP wire path must still produce a safe
// command from the zero-initialized safe position, skipping the arm-tip FK.
TEST(TaskControllerNoModelTest, EstopComputeWithoutModelIsSafe) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  ctrl.TriggerEstop();

  ControllerState state = MakeIiwa7LeapState();
  const auto out = ctrl.Compute(state);

  EXPECT_FALSE(out.arm_tip_pose_valid);  // no arm_handle_ → TF withheld
  for (int i = 0; i < kArmDof; ++i) {
    const double cmd = out.devices[0].commands[static_cast<std::size_t>(i)];
    EXPECT_TRUE(std::isfinite(cmd));
    // One clamped step from measured toward the (zero) safe position.
    EXPECT_NEAR(cmd, kArmHome[static_cast<std::size_t>(i)], 2.0 * kDt + 1e-9);
  }
  // Hand holds measured (zeros).
  EXPECT_NEAR(out.devices[1].commands[0], 0.0, 1e-12);
}

// ── LoadConfig validation ────────────────────────────────────────────────────

YAML::Node MinimalValidYaml() {
  return YAML::Load(R"(
estop:
  arm_safe_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fsm:
  pi_rotation_margin: 0.15
  contact_stop_release_eps: 0.005
)");
}

TEST(TaskControllerLoadConfigTest, MinimalYamlLoads) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  EXPECT_NO_THROW(ctrl.LoadConfig(MinimalValidYaml()));
}

TEST(TaskControllerLoadConfigTest, MissingEstopThrows) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  cfg.remove("estop");
  EXPECT_THROW(ctrl.LoadConfig(cfg), std::runtime_error);
}

TEST(TaskControllerLoadConfigTest, MissingFsmThrows) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  cfg.remove("fsm");
  EXPECT_THROW(ctrl.LoadConfig(cfg), std::runtime_error);
}

TEST(TaskControllerLoadConfigTest, PiRotationMarginOutOfRangeThrows) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  cfg["fsm"]["pi_rotation_margin"] = 2.0;  // > π/2
  EXPECT_THROW(ctrl.LoadConfig(cfg), std::runtime_error);
}

TEST(TaskControllerLoadConfigTest, ReleaseEpsOutOfRangeThrows) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  cfg["fsm"]["contact_stop_release_eps"] = 0.5;  // > 0.1
  EXPECT_THROW(ctrl.LoadConfig(cfg), std::runtime_error);
}

TEST(TaskControllerLoadConfigTest, LpfCutoffAboveNyquistThrows) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  ctrl.SetControlRate(500.0);
  auto cfg = MinimalValidYaml();
  cfg["fsm"]["contact_stop_lpf_cutoff_hz"] = 400.0;  // ≥ control_rate/2
  EXPECT_THROW(ctrl.LoadConfig(cfg), std::runtime_error);
}

TEST(TaskControllerLoadConfigTest, UnknownLogMsgTypeThrows) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  cfg["logs"] = YAML::Load(R"([{msg_type: rtc_msgs/Bogus}])");
  EXPECT_THROW(ctrl.LoadConfig(cfg), std::runtime_error);
}

TEST(TaskControllerLoadConfigTest, OversizedSafePositionThrows) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  YAML::Node big(YAML::NodeType::Sequence);
  for (int i = 0; i < 33; ++i) {  // > kDemoTaskMaxArmDof
    big.push_back(0.0);
  }
  cfg["estop"]["arm_safe_position"] = big;
  EXPECT_THROW(ctrl.LoadConfig(cfg), std::runtime_error);
}

}  // namespace
