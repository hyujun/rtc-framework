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
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "integrated_bringup/support/virtual_tcp.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"  // kMinMaxDamping

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

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
arm_dof: 7
kp_translation: [5.0, 5.0, 5.0]
kp_rotation: [2.0, 2.0, 2.0]
singularity_threshold: 0.02
max_damping: 0.05
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
  # #310: pins the TaskDiagLog msg_type string against LoadConfig's closed set.
  # The deployed YAMLs carry this entry, so a drift between the literal here and
  # kTaskDiagLogMsgType would otherwise only surface as a configure throw at
  # launch — this fixture turns that into a test failure.
  - msg_type: integrated_bringup/TaskDiagLog
    instance: task_diag
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
  /// YAML LoadConfig receives. Overridden by fixtures that need extra blocks
  /// (e.g. `force_pi_grasp`); the default keeps every existing test byte-identical.
  virtual std::string ControllerYaml() const { return kTaskYaml; }

  /// Grasp mode the controller runs under from its FIRST tick, applied after
  /// LoadConfig. Overridden to build a history with no force_pi tick in it — some
  /// published grasp fields are FSM latches, so "never published" and "published
  /// then switched away from" are different states. The default returns what
  /// LoadConfig resolved, so the base fixture stays a no-op rather than pinning
  /// the mode behind the YAML's back.
  virtual integrated_bringup::GraspHandMode InitialMode() const {
    return ctrl_->get_gains().grasp_hand_mode;
  }

  void SetUp() override {
    ctrl_ = std::make_unique<DemoTaskController>("", DemoTaskController::Gains{});
    // Production bring-up sequence (rt_controller_node_params.cpp): system
    // model + shared builder → PreConfigure(LoadConfig) → control rate →
    // device configs (triggers OnDeviceConfigsSet: reorder map + TCP frame
    // registration on the combined cache).
    ctrl_->SetSystemModelConfig(SharedIiwa7LeapModelConfig());
    ctrl_->SetSharedModelBuilder(SharedIiwa7LeapBuilder());
    ctrl_->SetControlRate(1.0 / kDt);
    ctrl_->LoadConfig(YAML::Load(ControllerYaml()));
    ctrl_->SetDeviceNameConfigs(MakeIiwa7LeapDeviceConfigs());
    {
      auto g = ctrl_->get_gains();
      g.grasp_hand_mode = InitialMode();
      ctrl_->set_gains(g);
    }

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

// ── §6.5 DLS at the binding (#282) ───────────────────────────────────────────
//
// The numerics of the migration — the two tiers and their oracle — live in
// test_task_dls_convergence.cpp. What has to be pinned HERE is that the shipped
// controller reaches that law at all, on the surfaces that bypass configure.
//
// Each of these drives one tick from a fixed state and reads the arm command,
// which is desired_q_ after integrating dq_. The plant is a perfect tracker, so
// a single tick from an identical start makes the command a direct readout of
// the joint velocity the law produced.

namespace {

// One tick from a chosen arm state under the given gains; returns the arm
// command. Fresh controller each time so no trajectory/seed state carries over
// between the gain settings being compared.
//
// `six_dof` is a parameter rather than kTaskYaml's value because all three
// shipped robot configs run `control_6dof: true`. A suite that only drove the
// YAML's 3-DOF default would leave the lane that actually ships uncovered and
// stay green for a mutation of the 6-D arm of the ternary alone.
//
// `extended` puts the arm at q = 0 — iiwa7 fully extended, where every joint
// axis is either on the tool's line or parallel to its neighbours' and the
// Jacobian collapses to σ_min = EXACTLY 0 (the same pose sits at the bottom of
// test_task_dls_convergence.cpp's recorded σ_min range). It is the only pose at
// which the σ₀ floor is observable: everywhere else σ_min far exceeds both a
// sub-floor σ₀ and its floor, and λ² is 0 on either side of the comparison.
std::array<double, kArmDof> CommandUnderGains(
    bool six_dof, bool extended, const std::function<void(DemoTaskController::Gains&)>& mutate) {
  DemoTaskController ctrl("", DemoTaskController::Gains{});
  ctrl.SetSystemModelConfig(SharedIiwa7LeapModelConfig());
  ctrl.SetSharedModelBuilder(SharedIiwa7LeapBuilder());
  ctrl.SetControlRate(1.0 / kDt);
  ctrl.LoadConfig(YAML::Load(kTaskYaml));
  ctrl.SetDeviceNameConfigs(MakeIiwa7LeapDeviceConfigs());

  auto state = MakeIiwa7LeapState();
  if (extended) {
    for (int i = 0; i < kArmDof; ++i) {
      state.devices[0].positions[static_cast<std::size_t>(i)] = 0.0;
    }
  }
  static_cast<void>(ctrl.Compute(state));  // seed the hold target from the measured pose

  // set_gains() writes the POD straight into the SeqLock: no LoadConfig, no
  // parameter callback. This is exactly the surface the point-of-use halves of
  // NUM-1 (λ_max) and NUM-2 (σ₀) exist for.
  auto g = ctrl.get_gains();
  g.control_6dof = six_dof;
  mutate(g);
  ctrl.set_gains(g);

  // One tick under the selected branch before the goal is authored: the 6-DOF
  // read path is what fills the rpy entries of actual_task_positions, and the
  // pose target below is expressed relative to them.
  state.iteration += 1;
  const ControllerOutput seed = ctrl.Compute(state);

  // A target the law has to work against — otherwise the task velocity is zero
  // and every damping setting produces the same (zero) command, which would
  // make every test below vacuous.
  const auto tcp0 = ctrl.tcp_position();
  if (six_dof) {
    const std::array<double, 6> target = {tcp0[0] + 0.05,
                                          tcp0[1] - 0.04,
                                          tcp0[2] + 0.03,
                                          seed.actual_task_positions[3],
                                          seed.actual_task_positions[4],
                                          seed.actual_task_positions[5] + 0.2};
    ctrl.SetDeviceTarget(0, target);
  } else {
    std::array<double, kArmDof> target{};
    target[0] = tcp0[0] + 0.05;
    target[1] = tcp0[1] - 0.04;
    target[2] = tcp0[2] + 0.03;
    for (int i = 3; i < kArmDof; ++i) {
      target[static_cast<std::size_t>(i)] = kArmHome[static_cast<std::size_t>(i)] + 0.2;
    }
    ctrl.SetDeviceTarget(0, target);
  }

  // Ticked WITHOUT feeding the command back into the state, for two reasons.
  // The pose stays exactly where it was put, so `extended`'s rank-deficient
  // Jacobian is still rank-deficient on the tick that is measured — a perfect
  // tracker would walk straight off it. And the measurement stays the
  // trajectory's start, so the task error is the trajectory's own displacement:
  // one tick would leave it at zero (a rest-to-rest quintic has zero position
  // AND zero velocity at t = 0), which made the whole comparison read the
  // null-space term alone and collapse to nothing on the 6-DOF lane, which has
  // no posture task at all. kTrialTicks lands mid-trajectory, where the error
  // and the feedforward are both largest.
  constexpr int kTrialTicks = 40;
  ControllerOutput out{};
  for (int t = 0; t < kTrialTicks; ++t) {
    state.iteration += 1;
    out = ctrl.Compute(state);
  }

  std::array<double, kArmDof> cmd{};
  for (int i = 0; i < kArmDof; ++i) {
    cmd[static_cast<std::size_t>(i)] = out.devices[0].commands[static_cast<std::size_t>(i)];
  }
  return cmd;
}

// Non-finite input short-circuits to infinity instead of being skipped.
// `std::max` is `a < b ? b : a` and `x < NaN` is false, so a naive running max
// reports **0** for two commands that are entirely NaN — which would make every
// `MaxAbsDiff(...) == 0` assertion below pass on a controller that had gone
// non-finite. That is the same laundering FloorMaxDamping exists to prevent,
// reproduced inside the sensor meant to detect it.
[[nodiscard]] double MaxAbsDiff(const std::array<double, kArmDof>& a,
                                const std::array<double, kArmDof>& b) {
  double m = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double d = std::abs(a[i] - b[i]);
    if (!std::isfinite(d)) {
      return std::numeric_limits<double>::infinity();
    }
    m = std::max(m, d);
  }
  return m;
}

}  // namespace

// Every case below runs on BOTH task dimensions. The 3-DOF lane is kTaskYaml's
// default; the 6-DOF lane is what config/{ur5e_p1a,ur5e_p1b,iiwa7_leap} ship.
class TaskControllerDlsBinding : public ::testing::TestWithParam<bool> {
 protected:
  [[nodiscard]] std::array<double, kArmDof> Command(
      const std::function<void(DemoTaskController::Gains&)>& mutate, bool extended = false) const {
    return CommandUnderGains(GetParam(), extended, mutate);
  }
};

INSTANTIATE_TEST_SUITE_P(BothTaskDimensions, TaskControllerDlsBinding, ::testing::Bool(),
                         [](const ::testing::TestParamInfo<bool>& info) {
                           return info.param ? "SixDof" : "ThreeDof";
                         });

// Tier 1, in the binding's own units. At a well-conditioned pose the §6.5 ramp
// damps by EXACTLY zero, so λ_max is not read at all and changing it must leave
// the command bit-identical. Under the retired constant-λ law every λ change
// moved the command, so this is a direct discriminator between the two laws —
// not a restatement of AdaptiveDampingSquared's unit test, which cannot see
// whether this controller calls it.
TEST_P(TaskControllerDlsBinding, LambdaMaxIsInertOutsideTheSingularityShell) {
  const auto a = Command([](DemoTaskController::Gains& g) { g.max_damping = 0.05; });
  const auto b = Command([](DemoTaskController::Gains& g) { g.max_damping = 0.5; });
  EXPECT_EQ(MaxAbsDiff(a, b), 0.0)
      << "λ_max changed the command at a well-conditioned pose — the binding is not running the "
         "§6.5 ramp (which is exactly 0 there), it is running something that always damps";
}

// The partner: inflate σ₀ past the pose's σ_min so the whole workspace is inside
// the shell, and λ_max must now move the command. Without this the test above
// would also pass on a controller that ignored λ_max entirely.
TEST_P(TaskControllerDlsBinding, LambdaMaxMovesTheCommandInsideTheShell) {
  const auto a = Command([](DemoTaskController::Gains& g) {
    g.singularity_threshold = 100.0;
    g.max_damping = 0.05;
  });
  const auto b = Command([](DemoTaskController::Gains& g) {
    g.singularity_threshold = 100.0;
    g.max_damping = 0.5;
  });
  EXPECT_GT(MaxAbsDiff(a, b), 1e-9)
      << "inside the shell λ² = λ_max²(1 − (σ_min/σ₀)²), so λ_max must reach the command";
}

// NUM-1's point-of-use half, which this controller is the first in-tree consumer
// of (modification-guide §Adding a New Controller item 5). A λ_max below
// kMinMaxDamping arriving through set_gains() never passes LoadConfig or the
// parameter callback, so only the tick's own FloorMaxDamping can catch it: the
// command must equal the FLOORED one, not the raw one. σ₀ is inflated so λ² is
// non-zero and the floor is observable at all.
TEST_P(TaskControllerDlsBinding, PointOfUseFloorsLambdaMaxFromSetGains) {
  const auto below = Command([](DemoTaskController::Gains& g) {
    g.singularity_threshold = 100.0;
    g.max_damping = 1e-9;  // below kMinMaxDamping
  });
  const auto at_floor = Command([](DemoTaskController::Gains& g) {
    g.singularity_threshold = 100.0;
    g.max_damping = rtc::compliance::kMinMaxDamping;
  });
  const auto unfloored = Command([](DemoTaskController::Gains& g) {
    g.singularity_threshold = 100.0;
    g.max_damping = 1e-3;  // a value the floor does NOT change
  });

  EXPECT_EQ(MaxAbsDiff(below, at_floor), 0.0)
      << "a sub-floor λ_max from set_gains() must be raised to kMinMaxDamping on the tick";
  EXPECT_GT(MaxAbsDiff(at_floor, unfloored), 1e-12)
      << "vacuity guard: λ_max must be observable at all in this configuration, otherwise the "
         "equality above would hold for any floor or none";
}

// NUM-2's point-of-use half, the σ₀ partner of the test above. σ₀ ≤ 0 arriving
// through set_gains() is NOT a stricter shell — AdaptiveDampingSquared
// short-circuits on `sigma0 <= 0.0` and returns λ² = 0 for every σ_min, so §6.5
// is disarmed at every pose INCLUDING a genuinely singular one, where J Jᵀ is
// only positive semi-definite and Eigen's LLT still reports Success. Nothing
// else in the chain reports it either; the arm just gets an undamped J⁺.
//
// Read the assertion as the mutation it is written against: with the floor, σ₀
// becomes kMinSigma0 > σ_min = 0, the pose is inside the shell, and λ_max
// reaches the command. Delete the floor and λ² is 0, which makes λ_max inert and
// collapses this difference to exactly zero — the same signature
// LambdaMaxIsInertOutsideTheSingularityShell asserts on purpose one pose over.
TEST_P(TaskControllerDlsBinding, PointOfUseFloorsSigma0FromSetGains) {
  const auto a = Command(
      [](DemoTaskController::Gains& g) {
        g.singularity_threshold = 0.0;  // disarms §6.5 everywhere if it survives
        g.max_damping = 0.05;
      },
      /*extended=*/true);
  const auto b = Command(
      [](DemoTaskController::Gains& g) {
        g.singularity_threshold = 0.0;
        g.max_damping = 0.5;
      },
      /*extended=*/true);

  EXPECT_GT(MaxAbsDiff(a, b), 1e-9)
      << "λ_max was inert at a rank-deficient pose under σ₀ = 0, i.e. λ² = 0: the σ₀ ≤ 0 that "
         "set_gains() wrote reached AdaptiveDampingSquared unfloored and disarmed §6.5 exactly "
         "where it is needed";
  for (const double v : a) {
    EXPECT_TRUE(std::isfinite(v)) << "the damped solve must still produce a finite command";
  }
}

// A non-finite λ_max is NOT laundered into a plausible one — FloorMaxDamping
// passes it through so the corruption stays visible downstream instead of the
// controller quietly running a damping shell that looks armed.
TEST_P(TaskControllerDlsBinding, NonFiniteLambdaMaxIsNotLaunderedIntoTheFloor) {
  const auto nan_gain = Command([](DemoTaskController::Gains& g) {
    g.singularity_threshold = 100.0;
    g.max_damping = std::numeric_limits<double>::quiet_NaN();
  });
  const auto floored = Command([](DemoTaskController::Gains& g) {
    g.singularity_threshold = 100.0;
    g.max_damping = rtc::compliance::kMinMaxDamping;
  });
  bool any_non_finite = false;
  for (const double v : nan_gain) {
    any_non_finite = any_non_finite || !std::isfinite(v);
  }
  EXPECT_TRUE(any_non_finite)
      << "std::max(1e-4, NaN) == 1e-4 would have laundered this into a finite command; the "
         "non-finite value must survive to the downstream finite check";
  EXPECT_GT(MaxAbsDiff(nan_gain, floored), 0.0);
}

// The σ₀ twin of the case above, and the reason FloorSigma0 is a symbol rather
// than a `std::max` at each site: `max(1e-6, NaN) == 1e-6` is a shell of radius
// 1e-6, which no reachable pose enters — so the corrupt gain reads back as a
// controller that is armed and simply never triggers, with no fault and no log.
// Passed through, the NaN reaches λ² and then the command, where
// ValidateControllerOutput rejects the tick at the actuator boundary.
TEST_P(TaskControllerDlsBinding, NonFiniteSigma0IsNotLaunderedIntoTheFloor) {
  const auto nan_gain = Command([](DemoTaskController::Gains& g) {
    g.singularity_threshold = std::numeric_limits<double>::quiet_NaN();
    g.max_damping = 0.05;
  });
  bool any_non_finite = false;
  for (const double v : nan_gain) {
    any_non_finite = any_non_finite || !std::isfinite(v);
  }
  EXPECT_TRUE(any_non_finite)
      << "a NaN σ₀ was laundered into 1e-6 — the operator's corrupt gain has been traded for a "
         "silently disarmed singularity guard";
}

// The ok=false lane. A NaN joint state reaches FK and makes the Jacobian
// non-finite; DifferentialIk then reports ok=false and keeps the PREVIOUS
// tick's J+ and N on purpose. Solving with those would command a
// plausible-looking velocity derived from a stale inverse, so the binding holds
// explicitly instead — dq_ = 0 and desired_q_ tracks the measurement, the same
// shape #292's kHoldExternal uses.
//
// The observable is NaN CONTAINMENT. The pre-#282 inline had no guard: a single
// NaN column makes every entry of J J^T non-finite, so the materialised inverse
// and therefore all seven joint commands went NaN. With the hold, only the joint
// whose own measurement is NaN carries it (that is the hold copying the state,
// not the law spreading it) and the other six stay pinned to where they were.
TEST_F(TaskControllerUrdfTest, NonFiniteJacobianHoldsInsteadOfSolvingWithAStaleInverse) {
  const auto tcp0 = ctrl_->tcp_position();
  std::array<double, kArmDof> target{};
  target[0] = tcp0[0] + 0.05;
  target[1] = tcp0[1];
  target[2] = tcp0[2];
  for (int i = 3; i < kArmDof; ++i) {
    target[static_cast<std::size_t>(i)] = kArmHome[static_cast<std::size_t>(i)];
  }
  ctrl_->SetDeviceTarget(0, target);
  RunTicks(20);  // in motion, so a stale J+ would produce visible drift

  std::array<double, kArmDof> before{};
  for (int i = 0; i < kArmDof; ++i) {
    before[static_cast<std::size_t>(i)] = state_.devices[0].positions[static_cast<std::size_t>(i)];
  }
  // Vacuity guard: the arm must actually be moving, or "it held" is free.
  ASSERT_GT(std::abs(before[0] - kArmHome[0]) + std::abs(before[1] - kArmHome[1]), 1e-4)
      << "fixture drifted: the arm never left home, so holding proves nothing";
  const auto traj_before = last_out_.trajectory_task_positions;

  constexpr std::size_t kNanJoint = 2;
  state_.devices[0].positions[kNanJoint] = std::numeric_limits<double>::quiet_NaN();
  state_.iteration += 1;
  const auto out = ctrl_->Compute(state_);

  // The hold must not advance the clock it is holding against. The §6.5 solve
  // sits AHEAD of `traj_state_ = trajectory_.compute(...)` / `trajectory_time_
  // += dt` for exactly this: with the hold on the far side of them the reference
  // kept running for the whole outage while the arm stood still, and whatever
  // resumed the law would have been handed the entire accumulated gap as task
  // error — a max-velocity dash to catch up, the jerk #292's hold-then-resume
  // shape exists to avoid. kHoldExternal returns ahead of the same two lines.
  for (std::size_t i = 0; i < traj_before.size(); ++i) {
    EXPECT_DOUBLE_EQ(out.trajectory_task_positions[i], traj_before[i])
        << "trajectory reference component " << i
        << " advanced on a tick the control law could not be evaluated on";
  }

  for (int i = 0; i < kArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    const double cmd = out.devices[0].commands[u];
    if (u == kNanJoint) {
      EXPECT_FALSE(std::isfinite(cmd))
          << "joint " << i << " IS the non-finite measurement; laundering it would hide the fault";
      continue;
    }
    EXPECT_TRUE(std::isfinite(cmd))
        << "joint " << i
        << " went non-finite: the NaN spread through J+ instead of being contained by the hold";
    EXPECT_DOUBLE_EQ(cmd, before[u])
        << "joint " << i << " moved on a tick the law could not be evaluated on";
  }
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

// The mode reaches the RT tick through the Gains SeqLock snapshot, not a plain
// member (B-1 RT handoff): the tick loads Gains once, so the mode and the
// thresholds it is read against can never be observed half-applied. This drives
// the branch through the PUBLIC gains path only, away from the kContactStop
// default — asking for contact_stop would prove nothing, since that is what an
// ignored Gains field leaves behind anyway.
TEST_F(TaskContactStopTest, GraspModeTravelsThroughTheGainsSnapshot) {
  auto g = ctrl_->get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kNone;
  ctrl_->set_gains(g);

  PrimeContact();
  auto out = RunTicks(300, /*feedback_hand=*/false);

  // Contact is observed either way — a tick that ignored the Gains field would
  // freeze the hand at 0.3 instead of driving it toward the 0.8 goal.
  ASSERT_TRUE(ctrl_->GetGraspStateForTesting().grasp_detected);
  EXPECT_GT(out.devices[1].commands[0], 0.5);
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

// ── contact_stop force LPF ───────────────────────────────────────────────────

// The freeze decision reads LPF'd force while the published GraspState keeps
// the raw value. Both halves matter: without the first, a single noisy sample
// can latch a freeze; without the second, every BT consumer of
// GraspState.max_force silently inherits the filter's group delay.
TEST_F(TaskContactStopTest, FilteredForceLagsRawWhilePublishedStaysRaw) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  // Valid but unloaded, long enough for the filter to settle at 0 — the step
  // below therefore starts from a primed filter rather than from a seed.
  SetFingertipForce(state_, 0, 0.0f);
  SetFingertipForce(state_, 1, 0.0f);
  RunTicks(60);

  SetFingertipForce(state_, 0, 5.0f);
  SetFingertipForce(state_, 1, 5.0f);
  RunTicks(1);

  const auto gs = ctrl_->GetGraspStateForTesting();
  const auto agg = ctrl_->GetContactStopAggregatesForTesting();
  EXPECT_NEAR(gs.max_force, 5.0f, 1e-4f) << "published aggregate must stay raw";
  EXPECT_LT(agg.max_force, 5.0f * 0.9f) << "contact_stop aggregate must lag the step";

  // …and converge to the same steady state, so the filter costs delay, not gain.
  RunTicks(120);
  EXPECT_NEAR(ctrl_->GetContactStopAggregatesForTesting().max_force, 5.0f, 1e-2f);
}

// Task-side wiring of the shared-guard / own-bank change. The laws it protects
// (rectification bias, wire spike, NaN) are pinned once in
// test_demo_joint_controller.cpp; what this pins is that THIS controller's
// Force-PI feed is the filtered bank rather than grasp_state_.force_magnitude.
// The swap lives in two files and only a per-file assertion catches one of them
// being missed.
TEST_F(TaskContactStopTest, ForcePiFeedIsTheFilteredBankNotTheRawMagnitude) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 0.0f);
  SetFingertipForce(state_, 1, 0.0f);
  RunTicks(60);

  SetFingertipForce(state_, 0, 5.0f);
  SetFingertipForce(state_, 1, 5.0f);
  // Past the guard's max_hold_ticks, so what remains between raw and feed is the
  // filter and nothing else.
  RunTicks(5);

  EXPECT_NEAR(ctrl_->GetGraspStateForTesting().max_force, 5.0f, 1e-4f)
      << "published aggregate must stay raw";
  EXPECT_LT(ctrl_->GetForcePiFeedForTesting()[0], 4.0f)
      << "the servo feed must lag the step — a raw feed would already be at 5 N";

  RunTicks(200);
  EXPECT_NEAR(ctrl_->GetForcePiFeedForTesting()[0], 5.0f, 1e-2f)
      << "…and converge, so the filter costs delay, not gain";
}

// An invalid fingertip must not advance its filter with the zeroed force
// ReadState writes: an IIR would carry that phantom sample for several ticks
// after the lane recovers. Re-entry seeds to the live force instead — erring
// toward an early freeze, which is the fail-safe direction for this latch.
TEST_F(TaskContactStopTest, InvalidFingertipFreezesFilterAndReEntrySeeds) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 5.0f);
  SetFingertipForce(state_, 1, 5.0f);
  RunTicks(150);
  ASSERT_NEAR(ctrl_->GetContactStopAggregatesForTesting().max_force, 5.0f, 1e-2f);

  state_.devices[1].inference_enable[0] = false;
  state_.devices[1].inference_enable[1] = false;
  RunTicks(5);
  EXPECT_NEAR(ctrl_->GetContactStopAggregatesForTesting().max_force, 0.0f, 1e-6f)
      << "invalid fingertips report no force";

  // One tick back at a much lower force: seeded, so no residue of the old 5 N.
  SetFingertipForce(state_, 0, 0.2f);
  SetFingertipForce(state_, 1, 0.2f);
  RunTicks(1);
  EXPECT_NEAR(ctrl_->GetContactStopAggregatesForTesting().max_force, 0.2f, 1e-4f);
}

// ── contact_stop force delta-spike guard ─────────────────────────────────────
//
// Wiring only (the guard's laws live in test_fingertip_force_guard.cpp): the LPF
// must be fed the GUARDED triplet while the raw lane stays the driver's value.
// Duplicated from the joint controller's suite on purpose — the two controllers
// hold independent guard arrays, and a wiring mistake in one is invisible in the
// other.

// The 260730_1017 p1b failure mode: a 2-tick fz spike (6.406 then 6.058 N) on an
// otherwise ~0.02 N lane, which fed straight into the Bessel LPF left ~0.170 N of
// filtered tail behind it.
TEST_F(TaskContactStopTest, TwoTickForceSpikeIsHeldOutOfTheFilterWhileRawIsUntouched) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 0.02f);
  SetFingertipForce(state_, 1, 0.02f);
  RunTicks(80);
  ASSERT_NEAR(ctrl_->GetContactStopAggregatesForTesting().max_force, 0.02f, 1e-3f)
      << "filter must be settled before the spike";

  SetFingertipForce(state_, 0, 6.406f);
  RunTicks(1);
  EXPECT_TRUE(ctrl_->WasForceGuardRejectedForTesting(0));
  EXPECT_NEAR(ctrl_->GetGraspStateForTesting().max_force, 6.406f, 1e-3f)
      << "published aggregate must stay raw";
  EXPECT_NEAR(ctrl_->GetGuardedFingertipForceForTesting()[2], 0.02f, 1e-4f);

  SetFingertipForce(state_, 0, 6.058f);
  RunTicks(1);
  EXPECT_TRUE(ctrl_->WasForceGuardRejectedForTesting(0))
      << "the second spike tick is compared against last_accepted, not the first spike";

  SetFingertipForce(state_, 0, 0.04f);
  RunTicks(1);
  EXPECT_FALSE(ctrl_->WasForceGuardRejectedForTesting(0));
  EXPECT_NEAR(ctrl_->GetGuardedFingertipForceForTesting()[2], 0.04f, 1e-4f);

  for (int i = 0; i < 20; ++i) {
    RunTicks(1);
    EXPECT_LT(ctrl_->GetContactStopAggregatesForTesting().max_force, 0.06f) << "tick " << i;
  }
}

// Without the consecutive-hold cap a genuine 6 N contact would be rejected on
// every tick (|6 - 0| > 4 forever) and the freeze would never trigger.
TEST_F(TaskContactStopTest, SustainedForceStepReachesTheFilterAfterTheHoldCap) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 0.0f);
  SetFingertipForce(state_, 1, 0.0f);
  RunTicks(60);

  SetFingertipForce(state_, 0, 6.0f);
  SetFingertipForce(state_, 1, 6.0f);
  for (int i = 0; i < 2; ++i) {
    RunTicks(1);
    EXPECT_TRUE(ctrl_->WasForceGuardRejectedForTesting(0)) << "hold tick " << i;
  }
  RunTicks(1);
  EXPECT_FALSE(ctrl_->WasForceGuardRejectedForTesting(0))
      << "hold cap must let a real step through";
  EXPECT_NEAR(ctrl_->GetGuardedFingertipForceForTesting()[2], 6.0f, 1e-4f);

  RunTicks(200);
  EXPECT_NEAR(ctrl_->GetContactStopAggregatesForTesting().max_force, 6.0f, 1e-2f);
}

// NaN gets no hold cap: one admitted sample poisons the IIR delay line for good.
TEST_F(TaskContactStopTest, NonFiniteForceNeverReachesTheFilter) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 0.5f);
  SetFingertipForce(state_, 1, 0.5f);
  RunTicks(80);
  ASSERT_NEAR(ctrl_->GetContactStopAggregatesForTesting().max_force, 0.5f, 1e-3f);

  SetFingertipForce(state_, 0, std::numeric_limits<float>::quiet_NaN());
  for (int i = 0; i < 20; ++i) {
    RunTicks(1);
    EXPECT_TRUE(ctrl_->WasForceGuardRejectedForTesting(0)) << "tick " << i;
    const float agg = ctrl_->GetContactStopAggregatesForTesting().max_force;
    EXPECT_TRUE(std::isfinite(agg)) << "tick " << i;
    EXPECT_NEAR(agg, 0.5f, 1e-3f) << "tick " << i;
  }

  SetFingertipForce(state_, 0, std::numeric_limits<float>::infinity());
  RunTicks(1);
  EXPECT_TRUE(ctrl_->WasForceGuardRejectedForTesting(0));
  EXPECT_TRUE(std::isfinite(ctrl_->GetContactStopAggregatesForTesting().max_force));

  SetFingertipForce(state_, 0, 0.6f);
  RunTicks(1);
  EXPECT_FALSE(ctrl_->WasForceGuardRejectedForTesting(0));
  EXPECT_NEAR(ctrl_->GetGuardedFingertipForceForTesting()[2], 0.6f, 1e-4f);
}

// ── "contact_stop never runs the PI law" — at the compute level ─────────────
//
// Today this guarantee is held by a BUILD-TIME fact: BuildGraspController only
// constructs the Force-PI controller when the type is "force_pi", so under
// contact_stop `grasp_controller_` is null and the `grasp_controller_ && mode ==
// kForcePi` branch cannot be taken whatever the mode says. That is about to stop
// being true — making the build mode-independent is the next commit — so the
// guarantee needs an owner in ComputeSecondary BEFORE it loses the one it has.
//
// Sister block to the joint controller's; the branch is written out once per
// controller, so it needs a copy per controller.
namespace {
// Appended to kTaskYaml (a flat map, so top-level keys concatenate). Supplies the
// two conditions BuildGraspController ANDs: the whitelisted type AND a
// `force_pi_grasp` block. f_target=2.5 is the PI law's fingerprint in the
// published GraspState — FillLogOutput copies it only on the force_pi branch.
const char* const kForcePiBlock = R"(
grasp_controller_type: "force_pi"
force_pi_grasp:
  f_target: 2.5
  fingers:
    thumb:
      q_open:  [0.0, 0.0, 0.0]
      q_close: [0.5, 1.0, 0.7]
    index:
      q_open:  [0.0, 0.0, 0.0]
      q_close: [0.5, 1.0, 0.7]
    middle:
      q_open:  [0.0, 0.0, 0.0]
      q_close: [0.5, 1.0, 0.7]
)";
}  // namespace

class TaskForcePiBuiltTest : public TaskContactStopTest {
 protected:
  std::string ControllerYaml() const override { return std::string(kTaskYaml) + kForcePiBlock; }

  integrated_bringup::GraspHandMode InitialMode() const override {
    // Precondition check lives here rather than in a test body: if the block
    // stopped building the PI controller, every assertion would pass for the
    // wrong reason. LoadConfig has already run when this is called.
    EXPECT_EQ(ctrl_->get_gains().grasp_hand_mode, integrated_bringup::GraspHandMode::kForcePi);
    return integrated_bringup::GraspHandMode::kForcePi;
  }
};

class TaskContactStopWithPiBuiltTest : public TaskForcePiBuiltTest {
 protected:
  integrated_bringup::GraspHandMode InitialMode() const override {
    (void)TaskForcePiBuiltTest::InitialMode();  // keep the build precondition
    return integrated_bringup::GraspHandMode::kContactStop;
  }
};

// Control arm. With the PI controller built and the mode left at force_pi, the
// contact_stop freeze must NOT run: the FSM sits in kIdle (nothing commanded a
// grasp), so the PI law overwrites no finger and the trajectory reaches its goal.
TEST_F(TaskForcePiBuiltTest, ForcePiModeSkipsTheFreezeAndPublishesPiDiagnostics) {
  PrimeContact();
  auto out = RunTicks(300, /*feedback_hand=*/false);

  ASSERT_TRUE(ctrl_->GetGraspStateForTesting().grasp_detected);
  EXPECT_GT(out.devices[1].commands[0], 0.5) << "contact_stop freeze ran under force_pi";
  EXPECT_NEAR(ctrl_->GetGraspStateForTesting().grasp_target_force, 2.5F, 1e-5F);
}

// The pin. Same built PI controller, mode moved to contact_stop after the fixture
// ticked in force_pi — the runtime-switch shape. The hand must freeze at the
// measured position, reachable only through the `else if` branch; a tick that let
// a non-null grasp_controller_ decide on its own would sail to the 0.8 goal.
TEST_F(TaskForcePiBuiltTest, ContactStopModeFreezesEvenWhenThePiControllerExists) {
  auto g = ctrl_->get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kContactStop;
  ctrl_->set_gains(g);

  PrimeContact();
  auto out = RunTicks(300, /*feedback_hand=*/false);

  ASSERT_TRUE(ctrl_->GetGraspStateForTesting().grasp_detected);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-4);
}

// Same branch, config-time shape: contact_stop in force from tick 1. Distinct
// history through the same gate, and the only shape in which the published-
// diagnostics half is meaningful — grasp_phase / grasp_target_force / finger_s are
// FSM latches, so a fixture that ticked force_pi first would leave a stale 2.5 N
// behind however correct the gate is.
TEST_F(TaskContactStopWithPiBuiltTest, ContactStopFromTheFirstTickFreezesAndPublishesNothing) {
  PrimeContact();
  auto out = RunTicks(300, /*feedback_hand=*/false);

  const auto& gs = ctrl_->GetGraspStateForTesting();
  ASSERT_TRUE(gs.grasp_detected);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-4);
  EXPECT_NEAR(gs.grasp_target_force, 0.0F, 1e-9F)
      << "PI target force published while the PI law was not running";
  EXPECT_EQ(gs.grasp_phase, 0U);
  for (std::size_t f = 0; f < 3; ++f) {
    EXPECT_NEAR(gs.finger_s[f], 0.0F, 1e-9F) << "finger " << f;
  }
}

// ── Mode-switch race closure (B-4a) ─────────────────────────────────────────
//
// Sibling block to the joint controller's; the rationale lives there.

// The latch mirror the non-RT quiet gate reads. Its own test rather than a rider
// on the freeze assertions: a tick that freezes the hand but never mirrors leaves
// the gate believing the hand is quiet, and no command assertion can see that.
TEST_F(TaskContactStopWithPiBuiltTest, LatchMirrorTracksTheLatchForTheQuietGate) {
  EXPECT_FALSE(ctrl_->GetContactLatchedMirrorForTesting()) << "mirror set before any contact";

  PrimeContact();
  RunTicks(150, /*feedback_hand=*/false);
  EXPECT_TRUE(ctrl_->GetContactLatchedMirrorForTesting())
      << "hold engaged but the gate cannot see it";

  // Release intent on every gate joint drops the latch with contact still
  // present — the mirror has to follow that path too, not just the engage path.
  std::array<double, kHandDof> open{};
  open.fill(kHandStart);
  open[1] = kHandStart + 0.3;
  open[4] = kHandStart - 0.25;
  open[7] = kHandStart - 0.25;
  ctrl_->SetDeviceTarget(1, open);
  RunTicks(300, /*feedback_hand=*/false);
  EXPECT_FALSE(ctrl_->GetContactLatchedMirrorForTesting());
}

// The gate's OTHER input, and the one that would otherwise go unverified until a
// live robot: the FSM phase. GraspController::Update() mutates the FSM on the RT
// tick, so the callback reads a mirror instead of phase() — and a mirror that never
// leaves 0 is indistinguishable from an idle hand, which is exactly the state the
// gate treats as "safe to switch".
TEST_F(TaskForcePiBuiltTest, PhaseMirrorLeavesIdleOnceAGraspIsCommanded) {
  ASSERT_EQ(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle));

  ASSERT_TRUE(ctrl_->CommandGraspForTesting(2.0)) << "fixture built no PI controller";
  // CommandGrasp only arms the transition; the FSM steps inside Update(), which
  // only the force_pi branch calls — so the mirror can only move via a tick.
  PrimeContact();
  RunTicks(5, /*feedback_hand=*/false);

  EXPECT_NE(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle))
      << "the quiet gate would admit a mode change mid-grasp";
}

// The race the quiet gate cannot close by itself: the callback checks the mirror,
// contact engages, and only then does the mode Store land. The tick that observes
// the new mode stops running the hold, and hand_computed_ falls back to a
// trajectory still aimed at the PRE-CONTACT goal — a resumed squeeze on a held
// object. set_gains() here IS the race: it skips the gate the way a gate with a
// hole would.
TEST_F(TaskContactStopWithPiBuiltTest, SwitchingAwayWhileLatchedDoesNotSnapBackToTheOldGoal) {
  PrimeContact();
  auto out = RunTicks(300, /*feedback_hand=*/false);
  ASSERT_NEAR(out.devices[1].commands[0], kHandStart, 1e-4) << "fixture never latched";
  ASSERT_TRUE(ctrl_->GetContactLatchedMirrorForTesting());

  auto g = ctrl_->get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kForcePi;
  ctrl_->set_gains(g);

  out = RunTicks(1, /*feedback_hand=*/false);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-3) << "snapped back on the switch tick";
  out = RunTicks(300, /*feedback_hand=*/false);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-3) << "ramped toward the old goal after";

  // The goal itself was re-seeded, not just the command — otherwise the next
  // re-plan walks back to 0.8 and the hold above was only a reprieve.
  EXPECT_NEAR(out.devices[1].goal_positions[0], kHandStart, 1e-3);
  EXPECT_FALSE(ctrl_->GetContactLatchedMirrorForTesting());
}

// ── Quiet-gate mirrors across a re-configure (review C3) ─────────────────────
// Rationale on the joint twins.

TEST_F(TaskContactStopWithPiBuiltTest, ReconfigureClearsTheLatchMirror) {
  PrimeContact();
  (void)RunTicks(300, /*feedback_hand=*/false);
  ASSERT_TRUE(ctrl_->GetContactLatchedMirrorForTesting()) << "fixture never latched";

  ctrl_->LoadConfig(YAML::Load(ControllerYaml()));

  EXPECT_FALSE(ctrl_->GetContactLatchedMirrorForTesting())
      << "re-configure left the gate believing the old hand was still holding";
  EXPECT_EQ(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle));
}

// #424 — same contract as the joint controller's
// PublishedStiffnessEstimateTracksTheControllersOwn, and it has to stay
// symmetric: the fill is duplicated in the two compute paths, so a fix applied
// to one of them is exactly the drift this pair exists to catch. Rationale for
// the lifted pin and the creeping force lives there.
TEST_F(TaskForcePiBuiltTest, PublishedStiffnessEstimateTracksTheControllersOwn) {
  ASSERT_TRUE(ctrl_->SetGraspKEstMaxForTesting(400.0)) << "fixture built no PI controller";
  ASSERT_TRUE(ctrl_->CommandGraspForTesting(2.0));
  PrimeContact();

  constexpr int kTicks = 800;
  for (int i = 0; i < kTicks; ++i) {
    const float f = 0.5F + 0.001F * static_cast<float>(i);  // 0.5 N/s at 500 Hz
    SetFingertipForce(state_, 0, f);
    SetFingertipForce(state_, 1, f);
    (void)RunTicks(1, /*feedback_hand=*/false);
  }

  const auto fs = ctrl_->GetGraspFingerStatesForTesting();
  ASSERT_FALSE(fs.empty());
  const auto published = ctrl_->GetPublishedGraspStateForTesting();

  bool left_seed = false;
  for (std::size_t i = 0; i < fs.size(); ++i) {
    EXPECT_FLOAT_EQ(published.finger_stiffness_est[i], static_cast<float>(fs[i].K_contact_est))
        << "finger " << i;
    if (fs[i].K_contact_est != 1.0) {
      left_seed = true;
    }
  }
  EXPECT_TRUE(left_seed) << "estimate never left its 1.0 seed, so the equality above cannot "
                            "tell a live mirror from a constant";
}

// Symmetric with the joint controller's EstopClearsAPopulatedForcePiDiagnostic;
// rationale for why the base-fixture E-STOP test cannot cover this lives there.
TEST_F(TaskForcePiBuiltTest, EstopClearsAPopulatedForcePiDiagnostic) {
  ASSERT_TRUE(ctrl_->SetGraspKEstMaxForTesting(400.0)) << "fixture built no PI controller";
  ASSERT_TRUE(ctrl_->CommandGraspForTesting(2.0));
  PrimeContact();
  for (int i = 0; i < 800; ++i) {
    const float f = 0.5F + 0.001F * static_cast<float>(i);
    SetFingertipForce(state_, 0, f);
    SetFingertipForce(state_, 1, f);
    (void)RunTicks(1, /*feedback_hand=*/false);
  }

  const auto fs = ctrl_->GetGraspFingerStatesForTesting();
  ASSERT_FALSE(fs.empty());
  const auto live = ctrl_->GetPublishedGraspStateForTesting();
  bool any_nonzero = false;
  for (std::size_t i = 0; i < fs.size(); ++i) {
    if (live.finger_s[i] != 0.0F || live.finger_filtered_force[i] != 0.0F ||
        live.finger_force_error[i] != 0.0F || live.finger_stiffness_est[i] != 0.0F) {
      any_nonzero = true;
    }
  }
  ASSERT_TRUE(any_nonzero) << "nothing was populated, so the E-STOP assertions below "
                              "would hold against a controller that never filled at all";

  ctrl_->TriggerEstop();
  (void)RunTicks(1, /*feedback_hand=*/false);

  const auto published = ctrl_->GetPublishedGraspStateForTesting();
  for (std::size_t i = 0; i < published.finger_s.size(); ++i) {
    EXPECT_FLOAT_EQ(published.finger_s[i], 0.0F) << "finger " << i;
    EXPECT_FLOAT_EQ(published.finger_filtered_force[i], 0.0F) << "finger " << i;
    EXPECT_FLOAT_EQ(published.finger_force_error[i], 0.0F) << "finger " << i;
    EXPECT_FLOAT_EQ(published.finger_stiffness_est[i], 0.0F) << "finger " << i;
  }
}

TEST_F(TaskForcePiBuiltTest, ReconfigureClearsThePhaseMirror) {
  ASSERT_TRUE(ctrl_->CommandGraspForTesting(2.0));
  PrimeContact();
  (void)RunTicks(5, /*feedback_hand=*/false);
  ASSERT_NE(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle));

  ctrl_->LoadConfig(YAML::Load(ControllerYaml()));

  EXPECT_EQ(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle))
      << "re-configure left the gate waiting on the old FSM";
  EXPECT_FALSE(ctrl_->GetContactLatchedMirrorForTesting());
}

// ── FSM ownership when the PI law is not running (review C2) ─────────────────
// Same four properties as the joint controller, same order. Rationale lives there;
// the pair has to stay symmetric because the fix is in shared code
// (GraspController::Reset) driven from two independent compute paths.

TEST_F(TaskForcePiBuiltTest, LeavingForcePiMidGraspReleasesTheQuietGate) {
  ASSERT_TRUE(ctrl_->CommandGraspForTesting(2.0)) << "fixture built no PI controller";
  PrimeContact();
  (void)RunTicks(5, /*feedback_hand=*/false);
  ASSERT_NE(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle))
      << "fixture never left Idle — the lock cannot be constructed";

  auto g = ctrl_->get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kContactStop;
  ctrl_->set_gains(g);
  (void)RunTicks(1, /*feedback_hand=*/false);

  EXPECT_EQ(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle))
      << "the quiet gate is still waiting on an FSM nobody steps";
}

// `none` is the other away-mode, and ur5e_p1b ships block + "none" — a reset keyed
// on "the freeze runs" rather than "the PI law does not" would lock that config.
TEST_F(TaskForcePiBuiltTest, LeavingForcePiForNoneAlsoReleasesTheQuietGate) {
  ASSERT_TRUE(ctrl_->CommandGraspForTesting(2.0));
  PrimeContact();
  (void)RunTicks(5, /*feedback_hand=*/false);
  ASSERT_NE(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle));

  auto g = ctrl_->get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kNone;
  ctrl_->set_gains(g);
  (void)RunTicks(1, /*feedback_hand=*/false);

  EXPECT_EQ(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle))
      << "mode none left the gate locked";
}

TEST_F(TaskForcePiBuiltTest, ReturningToForcePiDoesNotResumeTheInterruptedGrasp) {
  ASSERT_TRUE(ctrl_->CommandGraspForTesting(2.0));
  PrimeContact();
  (void)RunTicks(5, /*feedback_hand=*/false);

  auto g = ctrl_->get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kContactStop;
  ctrl_->set_gains(g);
  (void)RunTicks(1, /*feedback_hand=*/false);

  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kForcePi;
  ctrl_->set_gains(g);
  (void)RunTicks(50, /*feedback_hand=*/false);

  EXPECT_EQ(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle))
      << "the old grasp resumed itself on the way back";
}

TEST_F(TaskContactStopWithPiBuiltTest, AGraspArmedUnderAnotherLawIsDisarmed) {
  ASSERT_TRUE(ctrl_->CommandGraspForTesting(2.0));
  PrimeContact();
  (void)RunTicks(1, /*feedback_hand=*/false);

  auto g = ctrl_->get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kForcePi;
  ctrl_->set_gains(g);
  (void)RunTicks(50, /*feedback_hand=*/false);

  EXPECT_EQ(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle))
      << "a request armed under another law fired on the switch";
}

TEST_F(TaskForcePiBuiltTest, ActivationResetsTheFsmAndArmsTheServiceGate) {
  const rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                         "inactive");
  // Activate FIRST — see the joint twin: asserting `false` without a preceding
  // activation only re-reads the initial value and lets a dropped on_deactivate
  // store survive.
  EXPECT_FALSE(ctrl_->IsActiveForTesting()) << "active before any activation";
  ASSERT_EQ(ctrl_->on_activate(inactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(ctrl_->IsActiveForTesting()) << "activation did not open the service gate";

  ASSERT_TRUE(ctrl_->CommandGraspForTesting(2.0));
  PrimeContact();
  (void)RunTicks(5, /*feedback_hand=*/false);
  ASSERT_NE(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle));

  ASSERT_EQ(ctrl_->on_deactivate(inactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(ctrl_->IsActiveForTesting()) << "grasp_command still accepted while Inactive";
  ASSERT_EQ(ctrl_->on_activate(inactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);

  EXPECT_TRUE(ctrl_->IsActiveForTesting());
  EXPECT_EQ(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle))
      << "re-activation resumed a mid-grasp FSM";
  (void)RunTicks(50, /*feedback_hand=*/false);
  EXPECT_EQ(ctrl_->GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle));
}

// The regression this controller actually had: the mode comparison was hoisted
// into Compute(), beside the tick's single gains Load and therefore AHEAD of the
// `!estop_active_` gate on ComputeSecondary — while the closure that is the edge's
// only consumer sits behind it. A switch landing on an E-STOP tick advanced the
// cache with nobody to act on it, and the next live tick saw no edge: the hold
// stopped and the hand resumed a trajectory still aimed at the pre-contact goal.
// The justification in the header was itself false (FillLogOutput E-STOP-early-
// returns, so it never reads the cache on those ticks). Same test on the joint
// controller keeps the pair symmetric.
TEST_F(TaskContactStopWithPiBuiltTest, EstopTickDoesNotSwallowTheModeEdge) {
  PrimeContact();
  auto out = RunTicks(300, /*feedback_hand=*/false);
  ASSERT_NEAR(out.devices[1].commands[0], kHandStart, 1e-4) << "fixture never latched";
  ASSERT_TRUE(ctrl_->GetContactLatchedMirrorForTesting());

  ctrl_->TriggerEstop();
  auto g = ctrl_->get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kForcePi;
  ctrl_->set_gains(g);
  (void)RunTicks(1, /*feedback_hand=*/false);
  ctrl_->ClearEstop();

  out = RunTicks(1, /*feedback_hand=*/false);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-3)
      << "the E-STOP tick ate the edge — first live tick snapped back";
  out = RunTicks(300, /*feedback_hand=*/false);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-3);
  EXPECT_NEAR(out.devices[1].goal_positions[0], kHandStart, 1e-3) << "goal never re-seeded";
  EXPECT_FALSE(ctrl_->GetContactLatchedMirrorForTesting());
}

// The reachable half of the closure's guard: on a QUIET switch — the normal path,
// the one the gate admits — the closure must stay out of the way. Keyed on the
// mode edge alone it would discard the operator's pending target every time.
TEST_F(TaskContactStopWithPiBuiltTest, QuietSwitchKeepsTheOperatorsGoal) {
  for (int i = 0; i < kHandDof; ++i) {
    state_.devices[1].positions[static_cast<std::size_t>(i)] = kHandStart;
  }
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;  // no contact → no latch
  std::array<double, kHandDof> target{};
  target.fill(kHandGoal);
  ctrl_->SetDeviceTarget(1, target);
  RunTicks(50, /*feedback_hand=*/false);
  ASSERT_FALSE(ctrl_->GetContactLatchedMirrorForTesting()) << "fixture latched without contact";

  auto g = ctrl_->get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kForcePi;
  ctrl_->set_gains(g);
  auto out = RunTicks(300, /*feedback_hand=*/false);

  EXPECT_NEAR(out.devices[1].goal_positions[0], kHandGoal, 1e-9)
      << "quiet switch discarded the goal";
  EXPECT_NEAR(out.devices[1].commands[0], kHandGoal, 1e-3);
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

  // Turning the vtcp on moved the control point but must not have moved the ARM
  // (#292): the hold seed follows the frame, so the offset is never spent as
  // task error. Added, not relaxed — the offset assertions above are unchanged.
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                kArmHome[static_cast<std::size_t>(i)], 1e-6)
        << "arm joint " << i << " moved when the control frame changed";
  }
}

// ── Control-frame contract (#292) ────────────────────────────────────────────
//
// These pin the WIRING. The transition law itself is pinned exhaustively by
// ClassifyFrameTransition's tests in test_virtual_tcp.cpp, which is the only
// place it can be: iiwa7_leap's _base.yaml leaves `extended`/`closure_path`
// commented out and ships no closure sidecar, so this fixture's hand FK is
// serial — all four fingertips are valid from tick 1 and the member mask never
// moves. kReplanExternal and kBindExternal-after-a-mask-change are unreachable
// here, and a controller-level test pretending otherwise would pass without
// executing anything.
//
// The lever for "the control frame is tool0 while the intended frame is vtcp"
// is device 1's validity: RunHandForwardKinematics' serial path requires a
// valid hand device, so such a tick produces no fingertip poses and leaves
// vtcp_valid_ false while gains.vtcp.mode stays enabled. That is held for a
// different reason than a closed-chain walk-in tick, but it is the same shape
// from ComputeControl's side, which is the side under test.

// The observed p1b defect, reduced: the control point moves from tool0 to a
// virtual TCP 0.2 m and ~90° away in one tick, with the measured joints
// unchanged. SetUp's first Compute ran with the default Gains (vtcp disabled),
// so the self-init latched a tool0 hold — enabling the vtcp now IS the
// first-valid edge.
TEST_F(TaskControllerUrdfTest, VtcpFirstValidDoesNotInjectArmMotion) {
  auto gains = ctrl_->get_gains();
  gains.control_6dof = true;  // p1b runs 6-DOF CLIK, which is what made it violent
  gains.vtcp.mode = VirtualTcpMode::kConstant;
  gains.vtcp.offset = {0.0, 0.0, 0.2};              // ≈ the measured 0.2098 m jump
  gains.vtcp.orientation = {1.5708, 0.0, -1.5708};  // p1b's virtual_tcp_orientation
  ctrl_->set_gains(gains);

  auto out = RunTicks(1);  // the edge tick itself
  ASSERT_TRUE(out.virtual_tcp_pose_valid);
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                kArmHome[static_cast<std::size_t>(i)], 1e-9)
        << "arm joint " << i << " moved on the first-valid edge";
  }
  // The goal was re-seeded into the frame now being controlled, so the two
  // agree — the condition whose absence let computePoseError see 0.2 m of error.
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(out.task_goal_positions[i], out.actual_task_positions[i], 1e-9)
        << "task goal and actual disagree in component " << i;
  }

  out = RunTicks(20);  // and no drift accumulates afterwards
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                kArmHome[static_cast<std::size_t>(i)], 1e-6)
        << "arm joint " << i << " drifted after the frame transition";
  }
}

// A goal that arrives while the control frame is still tool0 is held, not
// reinterpreted, and fires unchanged once its frame arrives.
TEST_F(TaskControllerUrdfTest, ExternalTargetBeforeFirstValidIsHeldThenExecuted) {
  auto gains = ctrl_->get_gains();
  gains.vtcp.mode = VirtualTcpMode::kCentroid;
  ctrl_->set_gains(gains);

  state_.devices[1].valid = false;  // hand FK yields nothing → control frame is tool0
  RunTicks(1);
  ASSERT_FALSE(last_out_.virtual_tcp_pose_valid);

  const auto tcp0 = ctrl_->tcp_position();
  std::array<double, kArmDof> target{};
  target[0] = tcp0[0] + 0.03;
  target[1] = tcp0[1] - 0.02;
  target[2] = tcp0[2] + 0.02;
  for (int i = 3; i < kArmDof; ++i) {
    target[static_cast<std::size_t>(i)] = kArmHome[static_cast<std::size_t>(i)];
  }
  ctrl_->SetDeviceTarget(0, target);

  auto out = RunTicks(30);
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                kArmHome[static_cast<std::size_t>(i)], 1e-9)
        << "arm joint " << i << " moved while the goal's frame was unavailable";
  }

  state_.devices[1].valid = true;  // frame arrives → the preserved goal executes
  RunTicks(1200);
  ASSERT_TRUE(last_out_.virtual_tcp_pose_valid);
  const auto tcp = ctrl_->tcp_position();
  EXPECT_NEAR(tcp[0], target[0], 2e-3);
  EXPECT_NEAR(tcp[1], target[1], 2e-3);
  EXPECT_NEAR(tcp[2], target[2], 2e-3);
}

// Losing the frame mid-flight holds the arm and preserves the goal verbatim —
// it is never re-read as a goal in whatever frame happens to be live.
TEST_F(TaskControllerUrdfTest, VtcpLossHoldsExternalGoalAndDoesNotReinterpret) {
  auto gains = ctrl_->get_gains();
  gains.vtcp.mode = VirtualTcpMode::kCentroid;
  ctrl_->set_gains(gains);
  RunTicks(1);
  ASSERT_TRUE(last_out_.virtual_tcp_pose_valid);

  const auto tcp0 = ctrl_->tcp_position();
  std::array<double, kArmDof> target{};
  target[0] = tcp0[0] + 0.03;
  target[1] = tcp0[1];
  target[2] = tcp0[2] + 0.02;
  for (int i = 3; i < kArmDof; ++i) {
    target[static_cast<std::size_t>(i)] = kArmHome[static_cast<std::size_t>(i)];
  }
  ctrl_->SetDeviceTarget(0, target);
  RunTicks(100);  // part-way through the trajectory

  const std::array<double, 3> goal_before = {last_out_.task_goal_positions[0],
                                             last_out_.task_goal_positions[1],
                                             last_out_.task_goal_positions[2]};
  std::array<double, kArmDof> cmd_before{};
  for (int i = 0; i < kArmDof; ++i) {
    cmd_before[static_cast<std::size_t>(i)] =
        last_out_.devices[0].commands[static_cast<std::size_t>(i)];
  }

  state_.devices[1].valid = false;
  auto out = RunTicks(50);
  ASSERT_FALSE(out.virtual_tcp_pose_valid);
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                cmd_before[static_cast<std::size_t>(i)], 1e-9)
        << "arm joint " << i << " moved while the goal's frame was gone";
  }
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(out.task_goal_positions[i], goal_before[i], 1e-12)
        << "goal component " << i << " was rewritten while its frame was gone";
  }

  state_.devices[1].valid = true;
  RunTicks(1200);
  const auto tcp = ctrl_->tcp_position();
  EXPECT_NEAR(tcp[0], target[0], 2e-3);
  EXPECT_NEAR(tcp[1], target[1], 2e-3);
  EXPECT_NEAR(tcp[2], target[2], 2e-3);
}

// A frame that flickers every single tick still produces no motion while the
// goal is a hold seed — each tick re-seeds to zero error, which is why the
// design needs no latch or hysteresis anywhere.
TEST_F(TaskControllerUrdfTest, VtcpFlickerWithHoldSeedProducesNoMotion) {
  auto gains = ctrl_->get_gains();
  gains.vtcp.mode = VirtualTcpMode::kCentroid;
  ctrl_->set_gains(gains);

  for (int k = 0; k < 40; ++k) {
    state_.devices[1].valid = (k % 2 == 0);
    RunTicks(1);
    for (int i = 0; i < kArmDof; ++i) {
      ASSERT_NEAR(last_out_.devices[0].commands[static_cast<std::size_t>(i)],
                  kArmHome[static_cast<std::size_t>(i)], 1e-9)
          << "arm joint " << i << " moved on flicker tick " << k;
    }
  }
}

// R1: a goal whose frame never returns expires and the arm goes back to holding
// the frame it DOES have, instead of being frozen for the rest of the session.
TEST_F(TaskControllerUrdfTest, ExternalGoalExpiresInsteadOfFreezingTheArm) {
  auto gains = ctrl_->get_gains();
  gains.vtcp.mode = VirtualTcpMode::kCentroid;
  ctrl_->set_gains(gains);

  state_.devices[1].valid = false;
  RunTicks(1);
  ASSERT_FALSE(last_out_.virtual_tcp_pose_valid);

  const auto tcp0 = ctrl_->tcp_position();
  std::array<double, kArmDof> target{};
  target[0] = tcp0[0] + 0.05;
  target[1] = tcp0[1];
  target[2] = tcp0[2];
  for (int i = 3; i < kArmDof; ++i) {
    target[static_cast<std::size_t>(i)] = kArmHome[static_cast<std::size_t>(i)];
  }
  ctrl_->SetDeviceTarget(0, target);

  // Still inside the budget: the goal is intact and distinct from the actual
  // pose, so the expiry below is observably a change and not a no-op.
  auto out = RunTicks(10);
  EXPECT_NEAR(out.task_goal_positions[0], target[0], 1e-12);
  EXPECT_GT(std::abs(out.task_goal_positions[0] - out.actual_task_positions[0]), 1e-3);

  out = RunTicks(static_cast<int>(integrated_bringup::kVtcpFrameWaitTicks) + 5);
  // Expired: the goal is now the hold seed at the frame actually being
  // controlled, so goal and actual agree again...
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(out.task_goal_positions[i], out.actual_task_positions[i], 1e-9)
        << "component " << i << " did not fall back to a hold seed";
  }
  // ...and the arm never moved to get there.
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                kArmHome[static_cast<std::size_t>(i)], 1e-6)
        << "arm joint " << i << " moved across goal expiry";
  }
}

// `virtual_tcp_mode: disabled` must behave exactly as before: both frames are
// permanently tool0, so every tick classifies as kNone and nothing new runs.
TEST_F(TaskControllerUrdfTest, VirtualTcpDisabledPathUnchanged) {
  ASSERT_EQ(ctrl_->get_gains().vtcp.mode, VirtualTcpMode::kDisabled);
  EXPECT_FALSE(ctrl_->GetControlFrameIdForTesting().is_vtcp);

  auto out = RunTicks(20);
  EXPECT_FALSE(out.virtual_tcp_pose_valid);
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                kArmHome[static_cast<std::size_t>(i)], 1e-6);
  }

  // An external goal still executes with no frame ceremony in the way.
  const auto tcp0 = ctrl_->tcp_position();
  std::array<double, kArmDof> target{};
  target[0] = tcp0[0] + 0.03;
  target[1] = tcp0[1];
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
}

// The member mask reaches behaviour only through ClassifyFrameTransition, which
// ignores it in the two cases below — so a wrong derivation would leave no other
// trace. Read it directly.
TEST_F(TaskControllerUrdfTest, ControlFrameIdReportsMemberMaskPerMode) {
  auto gains = ctrl_->get_gains();
  gains.vtcp.mode = VirtualTcpMode::kCentroid;
  ctrl_->set_gains(gains);
  RunTicks(1);
  auto id = ctrl_->GetControlFrameIdForTesting();
  EXPECT_TRUE(id.is_vtcp);
  EXPECT_EQ(id.member_mask, 0b1111) << "all four serial-FK fingertips participate";

  // kConstant does not read the fingertips, so its mask is pinned: a fingertip
  // dropping out must not flip a constant-offset vtcp's frame identity.
  gains.vtcp.mode = VirtualTcpMode::kConstant;
  gains.vtcp.offset = {0.0, 0.0, 0.05};
  ctrl_->set_gains(gains);
  RunTicks(1);
  id = ctrl_->GetControlFrameIdForTesting();
  EXPECT_TRUE(id.is_vtcp);
  EXPECT_EQ(id.member_mask, 0xFF);

  // No hand FK this tick → no frame and no members. A mask surviving from the
  // previous tick here would be a stale participating set.
  state_.devices[1].valid = false;
  RunTicks(1);
  id = ctrl_->GetControlFrameIdForTesting();
  EXPECT_FALSE(id.is_vtcp);
  EXPECT_EQ(id.member_mask, 0);
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

// ── Over-reported device channels (#265 S-A / #172) ──────────────────────────

// `num_channels` is wire-derived — WriteJointStateToCache sets it from the
// JointState message length, not from the device config's joint_state_names —
// so it is independent of the arm model's nv. A state topic carrying extra
// joints (the exact regression the sim configs' `state_joint_names` comments
// warn about) makes nc0 > nv, and every arm loop that indexes desired_q_ / dq_
// / traj_dq_ (nv-wide Eigen vectors) by a device channel index must intersect
// the two. Channels past the model carry no model joint: they hold at the
// measured position, never the fresh-zero that means "go to the origin" on a
// position lane.
TEST_F(TaskControllerUrdfTest, OverReportedChannelsHoldMeasuredAndStayInModelBounds) {
  constexpr int kExtra = 3;
  constexpr std::array<double, kExtra> kSpare = {0.31, -0.42, 0.53};
  state_.devices[0].num_channels = kArmDof + kExtra;
  for (int k = 0; k < kExtra; ++k) {
    state_.devices[0].positions[static_cast<std::size_t>(kArmDof + k)] =
        kSpare[static_cast<std::size_t>(k)];
  }

  // RunTicks feeds back only the first kArmDof channels, so the spare slots keep
  // the measured values written above.
  auto out = RunTicks(5);

  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.devices[0].num_channels, kArmDof + kExtra);
  // Model-backed channels are unaffected: the arm still holds its measured pose,
  // exactly as in SelfInitHoldsMeasuredPoseAndPublishesTf (nc0 == nv).
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(out.devices[0].commands[static_cast<std::size_t>(i)],
                kArmHome[static_cast<std::size_t>(i)], 1e-6)
        << "arm joint " << i << " drifted once the device over-reported channels";
  }
  // Channels beyond the model hold at measured — not 0.0.
  for (int k = 0; k < kExtra; ++k) {
    EXPECT_DOUBLE_EQ(out.devices[0].commands[static_cast<std::size_t>(kArmDof + k)],
                     kSpare[static_cast<std::size_t>(k)])
        << "spare channel " << k << " was not held at its measured position";
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
arm_dof: 7
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

// The grasp mode rides the Gains snapshot (B-1 RT handoff), so LoadConfig has to
// resolve it BEFORE the Store it shares with every other gain — assigned after,
// the value lands in a local nothing reads again and the RT tick silently runs
// the default law. Only a non-default YAML value can tell those apart: both the
// Gains default and the shared-config default are contact_stop. Sister pin to
// JointControllerLoadConfigTest.GraspControllerTypeReachesGains — the ordering is
// written out once per controller, so it needs a copy per controller.
TEST(TaskControllerLoadConfigTest, GraspControllerTypeReachesGains) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  cfg["grasp_controller_type"] = "none";
  ASSERT_NO_THROW(ctrl.LoadConfig(cfg));
  EXPECT_EQ(ctrl.get_gains().grasp_hand_mode, integrated_bringup::GraspHandMode::kNone);
}

// ── #277 / NUM-6: the sixth posture-gain consumer ───────────────────────────
//
// q̇₀ = K_p·(q_null − q) with K_p < 0 drives the null-space posture AWAY from its
// target, and the (I − J⁺J) projector keeps that off the Cartesian task — so it
// is a silent drift, never a fault. This controller is the most exposed of the
// six: `null_kp` is a declared ROS parameter, so `ros2 param set` and the BT
// SetGains node reach it at runtime, where the five rtc_controllers siblings are
// only reachable through a YAML reload or a direct set_gains() handle.
TEST(TaskControllerLoadConfigTest, NegativeNullKpIsFlooredByTheLoader) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  cfg["null_kp"] = -0.5;
  ASSERT_NO_THROW(ctrl.LoadConfig(cfg));
  EXPECT_DOUBLE_EQ(ctrl.get_gains().null_kp, 0.0)
      << "a divergent posture gain reached the gains POD";

  // A floor, not a rewrite.
  cfg["null_kp"] = 0.5;
  ASSERT_NO_THROW(ctrl.LoadConfig(cfg));
  EXPECT_DOUBLE_EQ(ctrl.get_gains().null_kp, 0.5);

  // Unconditional, so an ABSENT key cannot leave a negative value from the
  // constructor or a previous `ros2 param set` in the POD.
  auto g = ctrl.get_gains();
  g.null_kp = -0.5;
  ctrl.set_gains(g);
  auto no_key = MinimalValidYaml();
  ASSERT_FALSE(no_key["null_kp"]);
  ASSERT_NO_THROW(ctrl.LoadConfig(no_key));
  EXPECT_DOUBLE_EQ(ctrl.get_gains().null_kp, 0.0)
      << "an absent key left a negative posture gain in the POD";
}

// ── #282: the retired `damping` key is reported, never mapped ───────────────
//
// The five rtc_controllers schemas retired the same key on the ground that a
// constant λ and the ceiling of a σ_min-adaptive ramp are not the same quantity,
// so any mapping would be a guess (rtc_controllers/README.md). This controller
// ships in three robot configs, so a stale key must not stop a bring-up either —
// it is ignored, and the ramp runs on ITS OWN keys.
TEST(TaskControllerLoadConfigTest, RetiredDampingKeyIsIgnoredAndNotMappedOntoMaxDamping) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  cfg["damping"] = 0.9;  // a value nothing should adopt
  ASSERT_NO_THROW(ctrl.LoadConfig(cfg));

  const auto g = ctrl.get_gains();
  EXPECT_DOUBLE_EQ(g.max_damping, DemoTaskController::Gains{}.max_damping)
      << "the retired constant λ was mapped onto λ_max — that mapping is the guess this "
         "contract exists to refuse";
  EXPECT_DOUBLE_EQ(g.singularity_threshold, DemoTaskController::Gains{}.singularity_threshold);
}

// Both §6.5 keys are read, and both are floored by the loader. The floors are
// the shared symbols (NUM-1 / NUM-2), so this also pins that a future edit
// cannot quietly swap them for a hand-written std::max.
TEST(TaskControllerLoadConfigTest, DlsKeysAreParsedAndFlooredByTheLoader) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  cfg["max_damping"] = 0.07;
  cfg["singularity_threshold"] = 0.03;
  ASSERT_NO_THROW(ctrl.LoadConfig(cfg));
  EXPECT_DOUBLE_EQ(ctrl.get_gains().max_damping, 0.07);
  EXPECT_DOUBLE_EQ(ctrl.get_gains().singularity_threshold, 0.03);

  // λ_max below kMinMaxDamping: floored. σ₀ ≤ 0 does not narrow the shell, it
  // makes AdaptiveDampingSquared return 0 for every σ_min — i.e. it disarms
  // §6.5 everywhere — so it is floored rather than accepted.
  cfg["max_damping"] = 1e-9;
  cfg["singularity_threshold"] = -1.0;
  ASSERT_NO_THROW(ctrl.LoadConfig(cfg));
  EXPECT_DOUBLE_EQ(ctrl.get_gains().max_damping, rtc::compliance::kMinMaxDamping);
  EXPECT_DOUBLE_EQ(ctrl.get_gains().singularity_threshold, rtc::compliance::kMinSigma0);

  // A floor is not a clamp: a non-finite value passes through UNCHANGED so the
  // downstream finite check still sees it. `std::max(1e-4, NaN) == 1e-4` would
  // have laundered it into a plausible value.
  cfg["max_damping"] = ".nan";
  ASSERT_NO_THROW(ctrl.LoadConfig(cfg));
  EXPECT_TRUE(std::isnan(ctrl.get_gains().max_damping))
      << "a non-finite λ_max was laundered into the floor";

  // Same for σ₀, and this is the half a hand-written `std::max(kMinSigma0, ·)`
  // got wrong until #282's review: `1e-6 < NaN` is false, so it returned 1e-6 —
  // a shell no reachable pose enters, i.e. the operator's corrupt gain silently
  // traded for a §6.5 that never engages.
  cfg["max_damping"] = 0.05;
  cfg["singularity_threshold"] = ".nan";
  ASSERT_NO_THROW(ctrl.LoadConfig(cfg));
  EXPECT_TRUE(std::isnan(ctrl.get_gains().singularity_threshold))
      << "a non-finite σ₀ was laundered into the floor";
}

// ── #302: the two surfaces of this controller must agree on gain length ─────
//
// OnGainParametersSet already rejected `v.size() != 3` ("kp_translation requires
// 3 values"), so `ros2 param set` and the BT SetGains node could not install a
// mis-shaped gain. LoadConfig used `min(size, 3)` and accepted from a file
// exactly what the parameter surface refuses — the same key, the same
// controller, opposite verdicts.
//
// Message-matched, not a bare EXPECT_THROW: `[1, 2, 3, 4]` reaches no
// past-the-end index, so nothing else in this loader would have thrown, and for
// the short case yaml-cpp must not be allowed to satisfy the assertion for a
// reason unrelated to length.
TEST(TaskControllerLoadConfigTest, MisShapedCartesianGainSequenceThrows) {
  auto load_error = [](const YAML::Node& cfg) {
    DemoTaskController ctrl{"", DemoTaskController::Gains{}};
    try {
      ctrl.LoadConfig(cfg);
    } catch (const std::exception& e) {
      return std::string(e.what());
    }
    return std::string{};
  };

  auto over_long = MinimalValidYaml();
  over_long["kp_translation"] = std::vector<double>{1.0, 2.0, 3.0, 4.0};
  EXPECT_NE(load_error(over_long).find("kp_translation"), std::string::npos);
  EXPECT_NE(load_error(over_long).find("3-entry"), std::string::npos);

  auto short_seq = MinimalValidYaml();
  short_seq["kp_rotation"] = std::vector<double>{1.0, 2.0};
  EXPECT_NE(load_error(short_seq).find("kp_rotation"), std::string::npos);

  auto scalar = MinimalValidYaml();
  scalar["kp_translation"] = 0.5;
  EXPECT_NE(load_error(scalar).find("kp_translation"), std::string::npos);
}

TEST(TaskControllerLoadConfigTest, ExactWidthCartesianGainsAreAppliedAndAbsentKeysKeepDefaults) {
  // Non-vacuity under the case above, and the specific regression the `min(size,
  // 3)` spelling produced: a two-entry sequence used to write entries 0 and 1
  // from YAML and leave entry 2 at its default, so the POD held a gain no
  // operator ever wrote and get_gains() reported it as if deliberate. Asserting
  // the whole triple is what makes a partial fill visible.
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  const auto defaults = ctrl.get_gains();

  auto cfg = MinimalValidYaml();
  cfg["kp_translation"] = std::vector<double>{7.0, 8.0, 9.0};
  ASSERT_NO_THROW(ctrl.LoadConfig(cfg));
  EXPECT_DOUBLE_EQ(ctrl.get_gains().kp_translation[0], 7.0);
  EXPECT_DOUBLE_EQ(ctrl.get_gains().kp_translation[2], 9.0);
  EXPECT_DOUBLE_EQ(ctrl.get_gains().kp_rotation[2], defaults.kp_rotation[2])
      << "an absent key must stay at its default, not become a config error";

  // A rejected reconfigure must not leave a half-written gain behind either.
  auto bad = MinimalValidYaml();
  bad["kp_translation"] = std::vector<double>{1.0, 2.0};
  EXPECT_THROW(ctrl.LoadConfig(bad), std::runtime_error);
  EXPECT_DOUBLE_EQ(ctrl.get_gains().kp_translation[0], 7.0)
      << "a mis-shaped sequence partially overwrote the live gain";
  EXPECT_DOUBLE_EQ(ctrl.get_gains().kp_translation[2], 9.0);
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

// The guard keys are optional (absent → Gains defaults), so a typo in the key
// name silently keeps 4.0 N / 2 ticks and the session looks configured. These
// three tests are what separate "the YAML was honoured" from "the YAML parsed".
TEST(TaskControllerLoadConfigTest, ForceGuardKeysReachGains) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  cfg["fsm"]["contact_stop_force_guard_delta_n"] = 2.5;
  cfg["fsm"]["contact_stop_force_guard_max_hold_ticks"] = 5;
  ctrl.LoadConfig(cfg);
  EXPECT_DOUBLE_EQ(ctrl.get_gains().contact_stop_force_guard_delta_n, 2.5);
  EXPECT_EQ(ctrl.get_gains().contact_stop_force_guard_max_hold_ticks, 5);
}

TEST(TaskControllerLoadConfigTest, ForceGuardDeltaOutOfRangeThrows) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  // 0 would reject every triplet whose axis moved at all — not a usable guard.
  cfg["fsm"]["contact_stop_force_guard_delta_n"] = 0.0;
  EXPECT_THROW(ctrl.LoadConfig(cfg), std::runtime_error);
}

TEST(TaskControllerLoadConfigTest, ForceGuardHoldTicksOutOfRangeThrows) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  cfg["fsm"]["contact_stop_force_guard_max_hold_ticks"] = -1;
  EXPECT_THROW(ctrl.LoadConfig(cfg), std::runtime_error);
}

TEST(TaskControllerLoadConfigTest, UnknownLogMsgTypeThrows) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  cfg["logs"] = YAML::Load(R"([{msg_type: rtc_msgs/Bogus}])");
  EXPECT_THROW(ctrl.LoadConfig(cfg), std::runtime_error);
}

// Since #340 this lands on the length cross-check (arm_dof stays 7) rather than
// on the old range check the length itself used to trip.
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

// ── #340: `arm_dof` is the DoF source, and the cross-check is finally live ────
//
// arm_dof_ used to BE the length of estop.arm_safe_position, so the two could
// not disagree and every throw inside ParseArmSafePosition was unreachable from
// production. Each config below is one the old code accepted in silence.
//
// The message is asserted, not just the throw: LoadConfig has many other ways
// to fail and a bare EXPECT_THROW would go green on any of them.

// Applies `mutate` to MinimalValidYaml() and returns LoadConfig's error
// message, or "" on success.
std::string TaskLoadConfigError(const std::function<void(YAML::Node&)>& mutate) {
  DemoTaskController ctrl{"", DemoTaskController::Gains{}};
  auto cfg = MinimalValidYaml();
  mutate(cfg);
  try {
    ctrl.LoadConfig(cfg);
  } catch (const std::exception& e) {
    return e.what();
  }
  return "";
}

TEST(TaskArmDofSourceTest, UnmutatedFixtureLoads) {
  EXPECT_EQ(TaskLoadConfigError([](YAML::Node&) {}), "");
}

TEST(TaskArmDofSourceTest, SafePositionLengthDisagreeingWithArmDofThrows) {
  const std::string msg = TaskLoadConfigError([](YAML::Node& cfg) { cfg["arm_dof"] = 6; });
  EXPECT_NE(msg.find("demo_task_controller"), std::string::npos) << msg;
  EXPECT_NE(msg.find("arm_safe_position"), std::string::npos) << msg;
}

TEST(TaskArmDofSourceTest, MissingArmDofThrowsWithMigrationInstructions) {
  const std::string msg = TaskLoadConfigError([](YAML::Node& cfg) { cfg.remove("arm_dof"); });
  EXPECT_NE(msg.find("demo_task_controller"), std::string::npos) << msg;
  EXPECT_NE(msg.find("arm_dof: 7"), std::string::npos)
      << "the message must name the value implied by the safe-position length: " << msg;
}

TEST(TaskArmDofSourceTest, MissingEstopSectionThrowsFromTheSharedParser) {
  const std::string msg = TaskLoadConfigError([](YAML::Node& cfg) { cfg.remove("estop"); });
  EXPECT_NE(msg.find("demo_task_controller"), std::string::npos) << msg;
  EXPECT_NE(msg.find("estop"), std::string::npos) << msg;
}

TEST(TaskArmDofSourceTest, ArmDofAboveTheControllerCapThrows) {
  const std::string msg = TaskLoadConfigError(
      [](YAML::Node& cfg) { cfg["arm_dof"] = integrated_bringup::kDemoTaskMaxArmDof + 1; });
  EXPECT_NE(msg.find("arm_dof"), std::string::npos) << msg;
  EXPECT_NE(msg.find("out of range"), std::string::npos) << msg;
}

// ── E-STOP telemetry freshness (#234 P-1) ──────────────────────────────────
//
// The CM stamps a publish snapshot every tick and the publish thread re-Loads
// the controller-owned SeqLock, so an E-STOP tick that stores nothing ships
// the pre-E-STOP body under the current stamp. Read through the SeqLock, not
// the staging buffer: that is the only way to tell "filled but never stored"
// from "published".
TEST_F(TaskControllerUrdfTest, EstopTickPublishesThisTicksBody) {
  state_.devices[1].num_inference_groups = 3;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 5.0f);
  SetFingertipForce(state_, 1, 5.0f);
  RunTicks(2);
  ASSERT_TRUE(ctrl_->GetPublishedGraspStateForTesting().grasp_detected);

  ctrl_->TriggerEstop();
  SetFingertipForce(state_, 0, 0.0f);
  SetFingertipForce(state_, 1, 0.0f);
  RunTicks(2);

  // NOTE: this fixture never loads the force_pi block, so grasp_hand_mode is the
  // kContactStop default and the Force-PI fill below never ran in the first
  // place. The zeros asserted here therefore pin "never filled", not "cleared"
  // — the clearing itself is pinned by EstopClearsAPopulatedForcePiDiagnostic
  // on the force_pi fixture, which is where a missing fill() actually shows up.
  const auto published = ctrl_->GetPublishedGraspStateForTesting();
  EXPECT_EQ(published.num_active_contacts, 0);
  EXPECT_FALSE(published.grasp_detected);
  EXPECT_NEAR(published.force_magnitude[0], 0.0f, 1e-4f);
  EXPECT_FALSE(published.pull.valid);
  EXPECT_FALSE(published.pull.slip_risk);
  for (std::size_t i = 0; i < published.finger_s.size(); ++i) {
    EXPECT_FLOAT_EQ(published.finger_s[i], 0.0f) << "finger " << i;
    EXPECT_FLOAT_EQ(published.finger_filtered_force[i], 0.0f) << "finger " << i;
    EXPECT_FLOAT_EQ(published.finger_force_error[i], 0.0f) << "finger " << i;
    // The stiffness estimate is the one field here that reads as live when
    // frozen: it barely moves tick to tick, so a stale sample looks exactly
    // like a fresh one. 0.0 is unreachable while the servo runs, which is what
    // makes it a usable not-computed marker (#424).
    EXPECT_FLOAT_EQ(published.finger_stiffness_est[i], 0.0f) << "finger " << i;
  }
}

}  // namespace

// ── grasp_diag.csv (#428), task lane ────────────────────────────────────────
//
// Symmetric to the joint controller's JointGraspDiagLogTest. The push site is
// duplicated across the two controllers' Compute() tails, so a fix or a
// deletion applied to only one of them is exactly the drift this pair exists to
// catch — the same reason the #424 stiffness-mirror tests come in a pair.
//
// The task lane has a structural difference worth pinning: it has NO separate
// E-STOP early-return, so its valid=0 rows come from ComputeSecondary being
// skipped rather than from an explicit estop push. Both must reach the file.

namespace {

namespace grasp_diag_fs = std::filesystem;

class TaskScopedSessionDir {
 public:
  TaskScopedSessionDir() {
    if (const char* prev = std::getenv("RTC_SESSION_DIR")) {
      had_prev_ = true;
      prev_value_ = prev;
    }
    auto base = grasp_diag_fs::temp_directory_path() / "rtc_grasp_diag_task_test";
    grasp_diag_fs::create_directories(base);
    dir_ = base / ("s_" + std::to_string(reinterpret_cast<std::uintptr_t>(this) & 0xFFFFFFFFU));
    grasp_diag_fs::create_directories(dir_);
    ::setenv("RTC_SESSION_DIR", dir_.c_str(), 1);
  }

  ~TaskScopedSessionDir() {
    if (had_prev_) {
      ::setenv("RTC_SESSION_DIR", prev_value_.c_str(), 1);
    } else {
      ::unsetenv("RTC_SESSION_DIR");
    }
    std::error_code ec;
    grasp_diag_fs::remove_all(dir_, ec);
  }

  TaskScopedSessionDir(const TaskScopedSessionDir&) = delete;
  TaskScopedSessionDir& operator=(const TaskScopedSessionDir&) = delete;

 private:
  grasp_diag_fs::path dir_;
  bool had_prev_{false};
  std::string prev_value_;
};

std::vector<std::string> GraspDiagReadLines(const grasp_diag_fs::path& p) {
  std::vector<std::string> out;
  std::ifstream in(p);
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty()) {
      out.push_back(line);
    }
  }
  return out;
}

std::vector<std::string> GraspDiagSplit(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  return out;
}

std::size_t GraspDiagColumn(const std::vector<std::string>& header, const std::string& name) {
  for (std::size_t i = 0; i < header.size(); ++i) {
    if (header[i] == name) {
      return i;
    }
  }
  return header.size();
}

class TaskGraspDiagLogTest : public TaskForcePiBuiltTest {
 protected:
  TaskScopedSessionDir session_;
  rtc::ControllerLogSet log_set_{"task_diag_test"};
  std::vector<std::string> finger_names_{"thumb", "index"};

  struct Entry {
    std::string msg_type;
    std::string instance;
  };

  void BindGraspDiag() {
    const std::vector<Entry> entries{{std::string(integrated_bringup::kGraspDiagLogMsgType),
                                      std::string(integrated_bringup::kGraspDiagLogInstance)}};
    integrated_bringup::LogRegistrationContext ctx{
        .logger = rclcpp::get_logger("grasp_diag_task_test"),
        .log_set = log_set_,
        .grasp_diag_enabled = true,
        .grasp_diag_finger_names = finger_names_,
    };
    auto reg = integrated_bringup::RegisterControllerLogs(entries, ctx);
    ASSERT_EQ(reg.status, integrated_bringup::LogRegistrationStatus::kSuccess);
    bound_ = static_cast<bool>(reg.handles.grasp_diag);
    ctrl_->SetGraspDiagLogHandleForTesting(std::move(reg.handles.grasp_diag));
  }

  grasp_diag_fs::path CsvPath() const {
    for (const auto& ch : log_set_.Channels()) {
      if (ch.first == integrated_bringup::kGraspDiagLogInstance) {
        return ch.second;
      }
    }
    return {};
  }

  bool bound_{false};
};

}  // namespace

TEST_F(TaskGraspDiagLogTest, EveryTickProducesOneRowAndKEstTracksTheControllersOwn) {
  BindGraspDiag();
  ASSERT_TRUE(bound_) << "channel did not register — the rest of this test is vacuous";
  ASSERT_TRUE(ctrl_->SetGraspKEstMaxForTesting(400.0));
  ASSERT_TRUE(ctrl_->CommandGraspForTesting(2.0));
  PrimeContact();

  // 800, matching the sibling #424 test: the task lane's hand feedback path
  // moves the estimate more slowly than the joint lane's, and at 400 the
  // estimate is still sitting on its 1.0 seed — which would make the equality
  // above pass against a column hardcoded to the seed.
  constexpr int kTicks = 800;
  for (int i = 0; i < kTicks; ++i) {
    const float f = 0.5F + 0.001F * static_cast<float>(i);
    SetFingertipForce(state_, 0, f);
    SetFingertipForce(state_, 1, f);
    (void)RunTicks(1, /*feedback_hand=*/false);
    // Production drains on a 10 Hz timer; a single trailing drain would overrun
    // the 512-deep ring and silently truncate the tail.
    if (i % 100 == 99) {
      log_set_.DrainAll();
    }
  }
  log_set_.DrainAll();
  ASSERT_EQ(log_set_.TotalDropCount(), 0U);

  const auto lines = GraspDiagReadLines(CsvPath());
  EXPECT_EQ(lines.size(), static_cast<std::size_t>(kTicks) + 1U) << "header + one row per tick";

  const auto header = GraspDiagSplit(lines[0]);
  const auto last = GraspDiagSplit(lines.back());
  const auto fs_states = ctrl_->GetGraspFingerStatesForTesting();
  ASSERT_FALSE(fs_states.empty());

  bool left_seed = false;
  for (std::size_t i = 0; i < fs_states.size() && i < finger_names_.size(); ++i) {
    const auto col = GraspDiagColumn(header, "k_est_" + finger_names_[i]);
    ASSERT_LT(col, header.size());
    EXPECT_FLOAT_EQ(std::stof(last[col]), static_cast<float>(fs_states[i].K_contact_est))
        << "finger " << finger_names_[i];
    if (fs_states[i].K_contact_est != 1.0) {
      left_seed = true;
    }
  }
  EXPECT_TRUE(left_seed) << "estimate never left its seed — the equality cannot tell a live "
                            "column from a constant";
}

// The task lane reaches valid=0 by a different route than the joint lane: there
// is no E-STOP early return here, ComputeSecondary is simply skipped, so the
// per-tick `ran` flag is the only thing marking the row. Pin that the row still
// appears and still reports not-computed.
TEST_F(TaskGraspDiagLogTest, EstopTickStillLeavesAValidZeroRow) {
  BindGraspDiag();
  ASSERT_TRUE(bound_);
  ASSERT_TRUE(ctrl_->CommandGraspForTesting(2.0));
  PrimeContact();
  for (int i = 0; i < 50; ++i) {
    SetFingertipForce(state_, 0, 0.5F + 0.001F * static_cast<float>(i));
    SetFingertipForce(state_, 1, 0.5F + 0.001F * static_cast<float>(i));
    (void)RunTicks(1, /*feedback_hand=*/false);
  }
  ctrl_->TriggerEstop();
  (void)RunTicks(1, /*feedback_hand=*/false);
  log_set_.DrainAll();

  const auto lines = GraspDiagReadLines(CsvPath());
  ASSERT_GE(lines.size(), 3U);
  const auto header = GraspDiagSplit(lines[0]);
  const auto valid_col = GraspDiagColumn(header, "valid");
  ASSERT_LT(valid_col, header.size());
  EXPECT_EQ(GraspDiagSplit(lines.back())[valid_col], "0");
  EXPECT_EQ(GraspDiagSplit(lines[lines.size() - 2U])[valid_col], "1")
      << "the preceding servo tick must be valid, or this proves nothing";
}

// ── §7.3 joint tail: the arm command is bounded by the joint limits (#478) ───
//
// This binding published `desired_q_` straight to the wire until #478 — the
// hand half of the same function clamped its device and both sibling bindings
// (`demo_joint`, `demo_wbc`) clamped theirs. The tail's algebra and its ORDER
// belong to rtc_controllers (test_joint_command_tail); what is asserted here is
// that this binding CALLS it, on the arm device, with its configured limits.
namespace {

/// The shipped arm configs with the position band narrowed to `home ± half`.
std::map<std::string, rtc::DeviceNameConfig> NarrowArmBandForTask(double half) {
  auto configs = MakeIiwa7LeapDeviceConfigs();
  auto& limits = configs["iiwa7"].joint_limits.value();
  for (int i = 0; i < kArmDof; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    limits.position_lower[ui] = kArmHome[ui] - half;
    limits.position_upper[ui] = kArmHome[ui] + half;
  }
  return configs;
}

/// Runs the same CLIK program under the given device configs and returns the
/// arm's final joint command. Only the configs differ between the two calls
/// below, which is what makes the second one attributable to the band.
std::array<double, kArmDof> RunToTargetUnderConfigs(
    const std::map<std::string, rtc::DeviceNameConfig>& configs, double* worst_step) {
  DemoTaskController ctrl("", DemoTaskController::Gains{});
  ctrl.SetSystemModelConfig(SharedIiwa7LeapModelConfig());
  ctrl.SetSharedModelBuilder(SharedIiwa7LeapBuilder());
  ctrl.SetControlRate(1.0 / kDt);
  ctrl.LoadConfig(YAML::Load(kTaskYaml));
  ctrl.SetDeviceNameConfigs(configs);

  auto state = MakeIiwa7LeapState();
  auto out = ctrl.Compute(state);  // seed the hold target from the measured pose

  const auto tcp0 = ctrl.tcp_position();
  std::array<double, kArmDof> target{};
  target[0] = tcp0[0] + 0.10;  // well past what a 0.005 rad band can reach
  target[1] = tcp0[1] - 0.08;
  target[2] = tcp0[2] + 0.06;
  for (int i = 3; i < kArmDof; ++i) {
    target[static_cast<std::size_t>(i)] = kArmHome[static_cast<std::size_t>(i)];
  }
  ctrl.SetDeviceTarget(0, target);

  std::array<double, kArmDof> prev = kArmHome;
  *worst_step = 0.0;
  for (int k = 0; k < 1200; ++k) {
    state.iteration += 1;
    out = ctrl.Compute(state);
    for (int i = 0; i < kArmDof; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      const double q = out.devices[0].commands[ui];
      state.devices[0].positions[ui] = q;
      *worst_step = std::max(
          *worst_step, std::abs(q - prev[ui]) /
                           MakeIiwa7LeapDeviceConfigs()["iiwa7"].joint_limits->max_velocity[ui]);
      prev[ui] = q;
    }
  }
  std::array<double, kArmDof> last{};
  for (int i = 0; i < kArmDof; ++i) {
    last[static_cast<std::size_t>(i)] = out.devices[0].commands[static_cast<std::size_t>(i)];
  }
  return last;
}

}  // namespace

TEST(TaskControllerJointTail, TheArmCommandStaysInsideTheConfiguredBand) {
  constexpr double kHalf = 0.005;

  // POSITIVE CONTROL FIRST: with the shipped band the same program must travel
  // FURTHER than kHalf on some joint, or the bounded run below proves nothing.
  double free_step = 0.0;
  const auto free_run = RunToTargetUnderConfigs(MakeIiwa7LeapDeviceConfigs(), &free_step);
  double free_excursion = 0.0;
  for (int i = 0; i < kArmDof; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    free_excursion = std::max(free_excursion, std::abs(free_run[ui] - kArmHome[ui]));
  }
  ASSERT_GT(free_excursion, kHalf)
      << "the unbounded run never left the band, so the bounded run proves nothing";

  double bounded_step = 0.0;
  const auto bounded = RunToTargetUnderConfigs(NarrowArmBandForTask(kHalf), &bounded_step);
  for (int i = 0; i < kArmDof; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_GE(bounded[ui], kArmHome[ui] - kHalf - 1e-12) << "arm joint " << i << " below its band";
    EXPECT_LE(bounded[ui], kArmHome[ui] + kHalf + 1e-12) << "arm joint " << i << " above its band";
  }
  // The rebound bounds the step that SURVIVES the clamp — the half a velocity
  // clamp on the solve output cannot reach, since the clamp runs after it.
  EXPECT_LE(bounded_step, kDt + 1e-12) << "a bounded command stepped past max_velocity·dt";
}
