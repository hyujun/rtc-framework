// ── DemoDualArmController: several task frames of one device group ──────────
//
// What this file pins, on a synthetic fixture this repository owns
// (rtc_urdf_bridge/test/urdf/dual_arm_trunk_hand.urdf — a trunk with two arms
// as one device group and a hand on the right arm):
//   - standstill: with no goal the command does not move;
//   - coupling: a left-hand goal taken in the TORSO frame does not move the
//     trunk, and the same goal taken in the pelvis frame does;
//   - both hands at once, against the stationary point of the weighted
//     least-squares problem the QP is;
//   - a goal expressed in another frame than its task's base frame is
//     converted into the base frame once, on the tick it is applied;
//   - the reference's feed-forward is in the base frame's axes;
//   - failed solves latch a controller-local fault; an E-STOP freezes the
//     command and re-seeds it from the measurement on release;
//   - an unreadable device freezes or silences, never commands;
//   - the transforms are the MEASURED poses.
//
// Oracles are forward kinematics on the FULL model addressed by joint name
// (dualarm_test_rig.hpp) and, for the two-hand case, a Gauss–Newton solve of
// the stationarity condition written out here — neither shares a cache, a
// reorder map or a frame index with the controller.
//
// Every case that says "X is refused / frozen / kept" carries the same rig's
// passing case next to it, and the cases that separate two behaviours which
// coincide under an ideal servo (re-seeded vs. kept) offset the measurement.

#include "dualarm_test_rig.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "rtc_tsid/kinematics/se3_error.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace dualarm_rig;  // NOLINT(google-build-using-namespace)
using integrated_bringup::DualArmDiagLogPod;
using integrated_bringup::DualArmTestAccess;
using integrated_bringup::TaskGoal;
using integrated_bringup::TaskGoalIngress;
using integrated_bringup::TaskGoalReject;
using Hold = DualArmDiagLogPod::Hold;
using GoalDrop = DualArmDiagLogPod::GoalDrop;

constexpr std::size_t kRight = 0;  // task order of the default rig
constexpr std::size_t kLeft = 1;

/// A double in scientific notation (std::to_string prints six decimals, which
/// turns every value this file records into "0.000000").
std::string Sci(double value) {
  std::ostringstream os;
  os.precision(6);
  os << std::scientific << value;
  return os.str();
}

double MaxAbsDiff(const std::vector<double>& a, const std::vector<double>& b) {
  double worst = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    worst = std::max(worst, std::abs(a[i] - b[i]));
  }
  return worst;
}

/// `pose` moved by `delta` along the BASE frame's axes.
pinocchio::SE3 Shifted(const pinocchio::SE3& pose, const Eigen::Vector3d& delta) {
  return {pose.rotation(), pose.translation() + delta};
}

/// Every tick's output has to be one the controller manager would accept.
void ExpectOutputAccepted(const Harness& h) {
  const auto verdict = rtc::ValidateControllerOutput(h.out, h.state);
  EXPECT_TRUE(verdict.Ok()) << rtc::OutputRejectReasonToString(verdict.reason) << " on tick "
                            << h.iteration;
}

/// Runs until every task's reference has stopped moving, then `settle` more.
void RunUntilReferencesRest(Harness& h, int settle, int limit = 20000) {
  for (int i = 0; i < limit; ++i) {
    (void)h.Tick();
    const auto& pod = h.ctrl->LastTick();
    bool moving = false;
    for (std::size_t k = 0; k < pod.num_tasks; ++k) {
      moving = moving || pod.tasks[k].traj_active;
    }
    if (!moving) {
      break;
    }
  }
  h.Run(settle);
}

// ── The fixture can tell the cases apart ────────────────────────────────────

TEST(DualArmFixture, CanTellTheCasesApart) {
  const pinocchio::Model& model = *Builder()->GetFullModel();
  EXPECT_EQ(model.nv, kBodyDof + kHandDof);
  for (const char* frame : {"world", kRoot, kLeftBase, kLeftFrame, kRightFrame, kHandMount}) {
    EXPECT_TRUE(model.existFrame(frame)) << frame;
  }
  // world → pelvis: a rotation AND an offset, so a goal in one and the same
  // goal in the other differ in every component.
  const pinocchio::SE3 world_from_pelvis = FramePose(kBodyQ, kHandQ, "world", kRoot);
  EXPECT_GT(world_from_pelvis.translation().norm(), 0.1);
  EXPECT_GT(Eigen::AngleAxisd(world_from_pelvis.rotation()).angle(), 0.2);
  // Each task frame is rotated against its base frame at the rig's pose: a
  // twist in the frame's own axes and the same twist in the base axes differ.
  EXPECT_GT(Eigen::AngleAxisd(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame).rotation()).angle(),
            0.5);
  EXPECT_GT(Eigen::AngleAxisd(FramePose(kBodyQ, kHandQ, kLeftBase, kLeftFrame).rotation()).angle(),
            0.5);
  // The left arm hangs off the torso: moving a trunk joint moves the left
  // tool in the pelvis and leaves it where it was in the torso.
  std::vector<double> trunk_moved = kBodyQ;
  trunk_moved[0] += 0.2;
  EXPECT_GT(PositionGap(FramePose(kBodyQ, kHandQ, kRoot, kLeftFrame),
                        FramePose(trunk_moved, kHandQ, kRoot, kLeftFrame)),
            1e-2);
  EXPECT_LT(PositionGap(FramePose(kBodyQ, kHandQ, kLeftBase, kLeftFrame),
                        FramePose(trunk_moved, kHandQ, kLeftBase, kLeftFrame)),
            1e-12);
  // The start pose is off every limit band, so no box is active at rest.
  std::vector<double> lower;
  std::vector<double> upper;
  std::vector<double> torque;
  BodyLimits(lower, upper, torque);
  for (std::size_t i = 0; i < kBodyQ.size(); ++i) {
    EXPECT_GT(kBodyQ[i], lower[i] + 0.2) << i;
    EXPECT_LT(kBodyQ[i], upper[i] - 0.2) << i;
  }
}

// ── Task goal ingress (support/owned_topics) ────────────────────────────────

TEST(TaskGoalIngressTest, ValidatesAndStamps) {
  const std::vector<std::string> frames = {"world", "pelvis"};
  TaskGoalIngress ingress;
  rtc_msgs::msg::RobotTarget msg;
  msg.goal_type = "task";
  msg.task_target = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

  // An empty frame id is the task's own base frame: slot −1.
  EXPECT_EQ(integrated_bringup::DeliverTaskGoal(msg, frames, 7, ingress), TaskGoalReject::kNone);
  TaskGoal goal = ingress.box.Load();
  EXPECT_EQ(goal.frame_slot, -1);
  EXPECT_EQ(goal.sequence, 1U);
  EXPECT_EQ(goal.generation, 7U);
  EXPECT_DOUBLE_EQ(goal.pose[5], 0.6);

  // A listed frame comes out as its index, and the sequence moves.
  msg.header.frame_id = "pelvis";
  EXPECT_EQ(integrated_bringup::DeliverTaskGoal(msg, frames, 8, ingress), TaskGoalReject::kNone);
  goal = ingress.box.Load();
  EXPECT_EQ(goal.frame_slot, 1);
  EXPECT_EQ(goal.sequence, 2U);
  EXPECT_EQ(ingress.accepted.load(), 2U);

  // Refusals leave the box alone and are counted by reason.
  msg.header.frame_id = "torso_link";
  EXPECT_EQ(integrated_bringup::DeliverTaskGoal(msg, frames, 8, ingress),
            TaskGoalReject::kUnknownFrame);
  msg.header.frame_id = "world";
  msg.task_target[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(integrated_bringup::DeliverTaskGoal(msg, frames, 8, ingress),
            TaskGoalReject::kNonFinite);
  msg.task_target[2] = 0.3;
  msg.goal_type = "joint";
  EXPECT_EQ(integrated_bringup::DeliverTaskGoal(msg, frames, 8, ingress),
            TaskGoalReject::kGoalType);
  EXPECT_EQ(ingress.box.Load().sequence, 2U);
  EXPECT_EQ(ingress.RejectCount(TaskGoalReject::kUnknownFrame), 1U);
  EXPECT_EQ(ingress.RejectCount(TaskGoalReject::kNonFinite), 1U);
  EXPECT_EQ(ingress.RejectCount(TaskGoalReject::kGoalType), 1U);
  EXPECT_EQ(ingress.TotalRejects(), 3U);
}

// ── Configuration ───────────────────────────────────────────────────────────

TEST(DualArmConfigTest, ParsesTheRigAndRefusesWhatItCannotRun) {
  EXPECT_NO_THROW((void)integrated_bringup::ParseDualArmConfig(YAML::Load(Yaml())));

  const auto refused = [](const Knobs& knobs) {
    try {
      (void)integrated_bringup::ParseDualArmConfig(YAML::Load(Yaml(knobs)));
    } catch (const std::runtime_error& e) {
      return std::string(e.what());
    }
    return std::string();
  };
  Knobs knobs;
  knobs.kind = "position_axis";
  EXPECT_NE(refused(knobs).find("kind"), std::string::npos);
  knobs = {};
  knobs.accel_constraint = "kinematic";
  EXPECT_NE(refused(knobs).find("accel_constraint"), std::string::npos);
  knobs = {};
  knobs.eta_tau = 1.21;
  EXPECT_NE(refused(knobs).find("eta_tau"), std::string::npos);
  knobs = {};
  knobs.eta_tau = 1.2;  // the top of the range is inside it
  EXPECT_TRUE(refused(knobs).empty()) << refused(knobs);
  knobs = {};
  knobs.left_base = kLeftFrame;  // a task relative to itself
  EXPECT_NE(refused(knobs).find("same frame"), std::string::npos);
  knobs = {};
  knobs.right_base = "";  // no "empty means world"
  EXPECT_NE(refused(knobs).find("base_frame"), std::string::npos) << refused(knobs);
  knobs = {};
  knobs.extra_posture_joint = kLeftJoints[0];  // one joint in two groups
  EXPECT_NE(refused(knobs).find("already in a posture group"), std::string::npos);
  knobs = {};
  knobs.linear_speed = 0.0;
  EXPECT_NE(refused(knobs).find("linear_speed"), std::string::npos);

  // A missing key is named, not defaulted.
  YAML::Node node = YAML::Load(Yaml());
  node["fault"].remove("track_err_max");
  try {
    (void)integrated_bringup::ParseDualArmConfig(node);
    ADD_FAILURE() << "a missing key was accepted";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("fault.track_err_max"), std::string::npos) << e.what();
  }
}

TEST(DualArmConfigTest, TheRuntimeRefusesWhatTheModelOrDevicesCannotGive) {
  EXPECT_TRUE(BringUp()->ConfigError().empty()) << BringUp()->ConfigError();

  const auto error = [](const Knobs& knobs) { return BringUp(knobs)->ConfigError(); };
  Knobs knobs;
  knobs.right_frame = "no_such_frame";
  EXPECT_NE(error(knobs).find("not a frame"), std::string::npos) << error(knobs);
  knobs = {};
  knobs.left_base = "no_such_base";
  EXPECT_NE(error(knobs).find("not a frame"), std::string::npos) << error(knobs);
  knobs = {};
  knobs.target_frames = {"world", "no_such_target"};
  EXPECT_NE(error(knobs).find("target_frames"), std::string::npos) << error(knobs);
  knobs = {};
  knobs.extra_posture_joint = kHandJoints[0];  // a hand joint is not a body joint
  EXPECT_NE(error(knobs).find("not a joint of device group"), std::string::npos) << error(knobs);
  knobs = {};
  knobs.limit_margin = 1.3;  // wider than half the trunk range
  EXPECT_NE(error(knobs).find("leaves no range"), std::string::npos) << error(knobs);
  knobs = {};
  knobs.gain_linear = 1.0 / kDt + 1.0;  // K·h > 1
  EXPECT_NE(error(knobs).find("exceeds 1/dt"), std::string::npos) << error(knobs);
  knobs = {};
  knobs.gain_linear = 1.0 / kDt;  // K·h == 1 is inside
  EXPECT_TRUE(error(knobs).empty()) << error(knobs);

  // A device group with no torque limits cannot carry the torque rows.
  auto devices = Devices();
  devices["body"].joint_limits->max_torque.clear();
  EXPECT_NE(BringUp({}, &devices)->ConfigError().find("max_torque"), std::string::npos);

  // The config loaded a second time AFTER the device configs keeps a runtime.
  auto ctrl = BringUp();
  ctrl->LoadConfig(YAML::Load(Yaml()));
  EXPECT_TRUE(ctrl->ConfigError().empty()) << ctrl->ConfigError();
}

// ── Standstill (formulation §4 item 5) ──────────────────────────────────────

TEST(DualArmStandstill, NoGoalNoMotion) {
  Harness h;
  (void)h.Tick();
  ASSERT_TRUE(h.ctrl->LastTick().reseeded);
  const std::vector<double> q0 = h.Command();
  EXPECT_LT(MaxAbsDiff(q0, kBodyQ), 1e-15) << "the seed is the measurement";

  double worst = 0.0;
  int failures = 0;
  for (int i = 0; i < 1000; ++i) {
    (void)h.Tick();
    const auto& pod = h.ctrl->LastTick();
    ASSERT_TRUE(pod.clik_ran) << "tick " << i;
    failures += pod.converged ? 0 : 1;
    // The premises "at rest" stands on: no joint that cannot be held, and no
    // torque row or braking bound doing the work.
    EXPECT_EQ(pod.brake_static_infeasible, 0U);
    worst = std::max(worst, MaxAbsDiff(h.Command(), q0));
    ExpectOutputAccepted(h);
  }
  EXPECT_EQ(failures, 0);
  EXPECT_LE(worst, 1e-9) << "max |q_c(t) - q_c(0)| over 1000 ticks [rad]";
  RecordProperty("standstill_max_dq_rad", Sci(worst));
  EXPECT_FALSE(h.ctrl->HasLatchedFault());
}

TEST(DualArmStandstill, ReactivationSeedsFromTheMeasurementAgain) {
  Harness h;
  h.Run(50);
  // The robot is somewhere else when the controller is activated again.
  for (std::size_t i = 0; i < h.body_q.size(); ++i) {
    h.body_q[i] += 0.02 * (i % 2 == 0 ? 1.0 : -1.0);
  }
  const std::vector<double> moved = h.body_q;
  ASSERT_EQ(h.ctrl->on_activate(rclcpp_lifecycle::State()),
            rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  (void)h.Tick();
  EXPECT_TRUE(h.ctrl->LastTick().reseeded);
  EXPECT_LT(MaxAbsDiff(h.Command(), moved), 1e-9);
  // ... and the solve accepts the new command state: a seed that left the
  // previous anchor in place would fail its command check until the latch.
  for (int i = 0; i < 20; ++i) {
    (void)h.Tick();
    EXPECT_TRUE(h.ctrl->LastTick().converged) << "tick " << i;
    EXPECT_FALSE(h.ctrl->LastTick().command_mismatch);
  }
  EXPECT_FALSE(h.ctrl->HasLatchedFault());
  EXPECT_LT(MaxAbsDiff(h.Command(), moved), 1e-9);
}

// ── Coupling (formulation §4 item 6) ────────────────────────────────────────

/// Moves the left hand 5 cm and returns the largest change of the three trunk
/// commands seen on the way.
double TrunkTravelForALeftHandGoal(const Knobs& knobs, bool expect_premises) {
  Harness h(knobs);
  (void)h.Tick();
  const std::vector<double> q0 = h.Command();
  const std::string base = knobs.left_base;
  const pinocchio::SE3 start = FramePose(kBodyQ, kHandQ, base, kLeftFrame);
  EXPECT_EQ(h.ctrl->DeliverTaskGoal(kLeft, TaskGoalMsg(Shifted(start, {0.03, 0.03, 0.03}))),
            TaskGoalReject::kNone);
  double trunk = 0.0;
  for (int i = 0; i < 1500; ++i) {
    (void)h.Tick();
    const auto& pod = h.ctrl->LastTick();
    EXPECT_TRUE(pod.converged) << "tick " << i;
    if (expect_premises) {
      // What "the trunk has no part in this task" stands on: nothing but the
      // task rows and the posture term decided this tick's solution.
      EXPECT_EQ(pod.accel_rows_binding, 0) << "tick " << i;
      EXPECT_EQ(pod.fb_saturated, 0U) << "tick " << i;
      EXPECT_EQ(pod.tasks[kRight].goal_sequence, 0U);
    }
    const std::vector<double> q = h.Command();
    for (std::size_t j = 0; j < 3; ++j) {
      trunk = std::max(trunk, std::abs(q[j] - q0[j]));
    }
  }
  // The hand did move: an arm that stayed put would also leave the trunk alone.
  EXPECT_GT(PositionGap(FramePose(h.Command(), kHandQ, base, kLeftFrame), start), 0.04);
  return trunk;
}

TEST(DualArmCoupling, ATorsoRelativeLeftHandDoesNotMoveTheTrunk) {
  const double relative = TrunkTravelForALeftHandGoal({}, /*expect_premises=*/true);
  EXPECT_LE(relative, 1e-9) << "trunk command change with the left hand relative to the torso";
  RecordProperty("trunk_travel_torso_relative_rad", Sci(relative));

  // The control: the same goal with the pelvis as the task's base frame. The
  // trunk joints are then in the task's columns, and the solve uses them.
  Knobs pelvis_based;
  pelvis_based.left_base = kRoot;
  const double world_like = TrunkTravelForALeftHandGoal(pelvis_based, /*expect_premises=*/false);
  EXPECT_GE(world_like, 1e-3) << "trunk command change with the left hand relative to the pelvis";
  RecordProperty("trunk_travel_pelvis_relative_rad", Sci(world_like));
}

// ── Both hands at once ──────────────────────────────────────────────────────

struct TaskSpec {
  std::string base;
  std::string frame;
  pinocchio::SE3 goal;  // in `base`
  double weight;
};

/// The stationary point of the weighted least-squares problem the QP is, by
/// Gauss–Newton from `body_q` (the hand stays at kHandQ). At rest the solve
/// returns v = 0 exactly where
///   Σ_k w_k·J_kᵀ·(K ⊙ e_k) + W_n·K_q·(q_des − q) = 0,
/// with J_k the task's Jacobian in its base frame's axes. Written from the
/// formulas, on the full model by joint name.
std::vector<double> StationaryPoint(std::vector<double> body_q, const std::vector<double>& q_des,
                                    const std::vector<TaskSpec>& tasks, const Knobs& knobs) {
  const pinocchio::Model& model = *Builder()->GetFullModel();
  pinocchio::Data data(model);
  const auto body = BodyJoints();
  std::vector<Eigen::Index> col(body.size());
  for (std::size_t i = 0; i < body.size(); ++i) {
    col[i] = model.joints[model.getJointId(body[i])].idx_v();
  }
  Eigen::VectorXd w_posture(kBodyDof);
  for (int i = 0; i < kBodyDof; ++i) {
    w_posture[i] = (i < 3) ? knobs.posture_weight_trunk : knobs.posture_weight_arm;
  }
  Eigen::Matrix<double, 6, 1> gain;
  gain << knobs.gain_linear, knobs.gain_linear, knobs.gain_linear, knobs.gain_angular,
      knobs.gain_angular, knobs.gain_angular;

  for (int iter = 0; iter < 200; ++iter) {
    const Eigen::VectorXd q = FullQ(body_q, kHandQ);
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(kBodyDof, kBodyDof);
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(kBodyDof);
    for (const auto& task : tasks) {
      const auto fid = model.getFrameId(task.frame);
      const auto bid = model.getFrameId(task.base);
      Eigen::MatrixXd j_t = Eigen::MatrixXd::Zero(6, model.nv);
      Eigen::MatrixXd j_b = Eigen::MatrixXd::Zero(6, model.nv);
      pinocchio::getFrameJacobian(model, data, fid, pinocchio::LOCAL_WORLD_ALIGNED, j_t);
      pinocchio::getFrameJacobian(model, data, bid, pinocchio::LOCAL_WORLD_ALIGNED, j_b);
      const Eigen::Matrix3d r_b = data.oMf[bid].rotation();
      const Eigen::Vector3d arm = data.oMf[fid].translation() - data.oMf[bid].translation();
      Eigen::MatrixXd j_rel(6, kBodyDof);
      for (int i = 0; i < kBodyDof; ++i) {
        const Eigen::Vector3d ang = j_t.block<3, 1>(3, col[static_cast<std::size_t>(i)]) -
                                    j_b.block<3, 1>(3, col[static_cast<std::size_t>(i)]);
        const Eigen::Vector3d lin = j_t.block<3, 1>(0, col[static_cast<std::size_t>(i)]) -
                                    j_b.block<3, 1>(0, col[static_cast<std::size_t>(i)]) +
                                    arm.cross(j_b.block<3, 1>(3, col[static_cast<std::size_t>(i)]));
        j_rel.block<3, 1>(0, i) = r_b.transpose() * lin;
        j_rel.block<3, 1>(3, i) = r_b.transpose() * ang;
      }
      const Eigen::Matrix<double, 6, 1> error =
          rtc::tsid::ComputeTaskPoseError(data.oMf[bid].actInv(data.oMf[fid]), task.goal);
      grad += task.weight * j_rel.transpose() * gain.cwiseProduct(error);
      hess += task.weight * j_rel.transpose() * gain.asDiagonal() * j_rel;
    }
    for (int i = 0; i < kBodyDof; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      grad[i] += w_posture[i] * knobs.posture_gain * (q_des[ui] - body_q[ui]);
      hess(i, i) += w_posture[i] * knobs.posture_gain + 1e-12;
    }
    const Eigen::VectorXd step = hess.ldlt().solve(grad);
    for (int i = 0; i < kBodyDof; ++i) {
      body_q[static_cast<std::size_t>(i)] += step[i];
    }
    if (step.cwiseAbs().maxCoeff() < 1e-13) {
      break;
    }
  }
  return body_q;
}

TEST(DualArmTwoHands, BothGoalsOnOneTickReachTheWeightedLeastSquaresPoint) {
  const Knobs knobs;
  Harness h(knobs);
  (void)h.Tick();
  const pinocchio::SE3 right_goal =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.04, -0.03, 0.05});
  const pinocchio::SE3 left_goal =
      Shifted(FramePose(kBodyQ, kHandQ, kLeftBase, kLeftFrame), {-0.03, 0.04, 0.03});
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(right_goal)), TaskGoalReject::kNone);
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kLeft, TaskGoalMsg(left_goal)), TaskGoalReject::kNone);

  int failures = 0;
  bool both_moved_together = false;
  for (int i = 0; i < 30000; ++i) {
    (void)h.Tick();
    const auto& pod = h.ctrl->LastTick();
    failures += pod.converged ? 0 : 1;
    both_moved_together =
        both_moved_together || (pod.tasks[kRight].traj_active && pod.tasks[kLeft].traj_active);
  }
  EXPECT_EQ(failures, 0);
  EXPECT_TRUE(both_moved_together) << "the two references were never in motion on one tick";
  EXPECT_EQ(h.ctrl->LastTick().tasks[kRight].goal_sequence, 1U);
  EXPECT_EQ(h.ctrl->LastTick().tasks[kLeft].goal_sequence, 1U);

  const std::vector<double> q_c = h.Command();
  const std::vector<TaskSpec> tasks = {{kRoot, kRightFrame, right_goal, knobs.weight_right},
                                       {kLeftBase, kLeftFrame, left_goal, knobs.weight_left}};
  const std::vector<double> q_star = StationaryPoint(q_c, kBodyQ, tasks, knobs);
  EXPECT_LE(MaxAbsDiff(q_c, q_star), 1e-6)
      << "command against the weighted least-squares stationary point [rad]";
  RecordProperty("wls_stationary_gap_rad", Sci(MaxAbsDiff(q_c, q_star)));
  RecordProperty("wls_right_err_m",
                 Sci(PositionGap(FramePose(q_c, kHandQ, kRoot, kRightFrame), right_goal)));
  RecordProperty("wls_left_err_m",
                 Sci(PositionGap(FramePose(q_c, kHandQ, kLeftBase, kLeftFrame), left_goal)));
  // Not vacuous: the stationary point is not where the arm started.
  EXPECT_GT(MaxAbsDiff(q_star, kBodyQ), 1e-2);
}

TEST(DualArmTwoHands, WithNoPostureGainBothGoalsAreReached) {
  Knobs knobs;
  knobs.posture_gain = 0.0;  // the posture term then only damps
  Harness h(knobs);
  (void)h.Tick();
  const pinocchio::SE3 right_goal =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.04, -0.03, 0.05});
  const pinocchio::SE3 left_goal =
      Shifted(FramePose(kBodyQ, kHandQ, kLeftBase, kLeftFrame), {-0.03, 0.04, 0.03});
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(right_goal)), TaskGoalReject::kNone);
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kLeft, TaskGoalMsg(left_goal)), TaskGoalReject::kNone);
  int failures = 0;
  for (int i = 0; i < 4000; ++i) {
    (void)h.Tick();
    failures += h.ctrl->LastTick().converged ? 0 : 1;
  }
  EXPECT_EQ(failures, 0);
  const std::vector<double> q_c = h.Command();
  const pinocchio::SE3 right = FramePose(q_c, kHandQ, kRoot, kRightFrame);
  const pinocchio::SE3 left = FramePose(q_c, kHandQ, kLeftBase, kLeftFrame);
  EXPECT_LE(PositionGap(right, right_goal), 1e-4);
  EXPECT_LE(RotationGap(right, right_goal), 1e-3);
  EXPECT_LE(PositionGap(left, left_goal), 1e-4);
  EXPECT_LE(RotationGap(left, left_goal), 1e-3);
  RecordProperty("kq0_right_err_m", Sci(PositionGap(right, right_goal)));
  RecordProperty("kq0_left_err_m", Sci(PositionGap(left, left_goal)));
}

// ── Goals in another frame than the task's base frame ───────────────────────

TEST(DualArmGoalFrames, TheSameGoalInWorldAndInTheBaseFrameCommandTheSameMotion) {
  // The right hand's base frame is the pelvis. One physical goal, two ways of
  // writing it down.
  const pinocchio::SE3 in_pelvis =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.03, 0.02, -0.04});
  const pinocchio::SE3 in_world = FramePose(kBodyQ, kHandQ, "world", kRoot).act(in_pelvis);
  ASSERT_GT(PositionGap(in_world, in_pelvis), 0.1) << "the two expressions must differ";

  Harness direct;
  Harness converted;
  Harness named_base;
  (void)direct.Tick();
  (void)converted.Tick();
  (void)named_base.Tick();
  ASSERT_EQ(direct.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(in_pelvis)), TaskGoalReject::kNone);
  ASSERT_EQ(converted.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(in_world, "world")),
            TaskGoalReject::kNone);
  ASSERT_EQ(named_base.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(in_pelvis, kRoot)),
            TaskGoalReject::kNone);
  (void)direct.Tick();
  (void)converted.Tick();
  (void)named_base.Tick();
  // The goal the controller holds, in the base frame.
  const auto& goal_direct = DualArmTestAccess::Tasks(*direct.ctrl)[kRight].goal;
  const auto& goal_converted = DualArmTestAccess::Tasks(*converted.ctrl)[kRight].goal;
  EXPECT_LE(PositionGap(goal_direct, goal_converted), 1e-12);
  EXPECT_LE(RotationGap(goal_direct, goal_converted), 1e-12);
  EXPECT_LE(PositionGap(goal_direct, in_pelvis), 1e-12) << "an empty frame id is the base frame";

  double worst = 0.0;
  double worst_named = 0.0;
  for (int i = 0; i < 1500; ++i) {
    (void)direct.Tick();
    (void)converted.Tick();
    (void)named_base.Tick();
    worst = std::max(worst, MaxAbsDiff(direct.Command(), converted.Command()));
    worst_named = std::max(worst_named, MaxAbsDiff(direct.Command(), named_base.Command()));
  }
  EXPECT_LE(worst, 1e-9) << "world-expressed against base-expressed, max over the motion [rad]";
  RecordProperty("frame_conversion_max_dq_rad", Sci(worst));
  EXPECT_LE(worst_named, 1e-9);
  EXPECT_GT(MaxAbsDiff(direct.Command(), kBodyQ), 1e-2) << "the goal did move the arm";
}

TEST(DualArmGoalFrames, AnUnknownFrameIsRefusedAndCounted) {
  Harness h;
  (void)h.Tick();
  const pinocchio::SE3 goal =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.03, 0.0, 0.0});
  // `hand_base` is a frame of the model, but not one the controller was told
  // goals may come in.
  EXPECT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(goal, kHandMount)),
            TaskGoalReject::kUnknownFrame);
  h.Run(50);
  const auto& pod = h.ctrl->LastTick();
  EXPECT_EQ(pod.tasks[kRight].goal_sequence, 0U);
  EXPECT_EQ(
      pod.tasks[kRight].ingress_counts[static_cast<std::size_t>(TaskGoalReject::kUnknownFrame)],
      1U);
  EXPECT_LE(MaxAbsDiff(h.Command(), kBodyQ), 1e-9) << "a refused goal moved the arm";
}

TEST(DualArmGoalFrames, AGoalInAMovingFrameIsConvertedOnceWhenItIsApplied) {
  // The left hand's base frame is the torso; the goal is written in `world`.
  // The torso moves afterwards. Converted once, the goal is a pose fixed in
  // the torso and travels with it; converted every tick, it would stay put in
  // the world and its torso-relative pose would change instead.
  Harness h;
  (void)h.Tick();
  const pinocchio::SE3 in_torso =
      Shifted(FramePose(kBodyQ, kHandQ, kLeftBase, kLeftFrame), {0.02, 0.02, 0.0});
  const pinocchio::SE3 in_world = FramePose(kBodyQ, kHandQ, "world", kLeftBase).act(in_torso);
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kLeft, TaskGoalMsg(in_world, "world")), TaskGoalReject::kNone);
  RunUntilReferencesRest(h, 500);
  const pinocchio::SE3 goal_before = DualArmTestAccess::Tasks(*h.ctrl)[kLeft].goal;
  EXPECT_LE(PositionGap(goal_before, in_torso), 1e-9);
  const pinocchio::SE3 world_before = FramePose(h.Command(), kHandQ, "world", kLeftFrame);

  // Move the trunk through the posture target (it is in the right-hand task's
  // null space, so the solve follows it while the right hand stays).
  std::vector<double> posture = kBodyQ;
  posture[0] += 0.5;
  h.ctrl->SetDeviceTarget(0, posture);
  h.Run(6000);
  const std::vector<double> q = h.Command();
  ASSERT_GE(std::abs(q[0] - kBodyQ[0]), 0.05) << "the trunk has to have moved for this to judge";

  const pinocchio::SE3 goal_after = DualArmTestAccess::Tasks(*h.ctrl)[kLeft].goal;
  EXPECT_EQ(PositionGap(goal_before, goal_after), 0.0) << "the goal was converted again";
  EXPECT_EQ(RotationGap(goal_before, goal_after), 0.0);
  // The hand kept its torso-relative pose and was carried through the world.
  EXPECT_LE(PositionGap(FramePose(q, kHandQ, kLeftBase, kLeftFrame), in_torso), 2e-3);
  EXPECT_GT(PositionGap(FramePose(q, kHandQ, "world", kLeftFrame), world_before), 1e-2);
}

TEST(DualArmGoalFrames, AGoalHalfATurnAwayIsRefused) {
  Harness h;
  (void)h.Tick();
  const pinocchio::SE3 start = FramePose(kBodyQ, kHandQ, kRoot, kRightFrame);
  const pinocchio::SE3 flipped(
      start.rotation() * Eigen::AngleAxisd(3.1, Eigen::Vector3d::UnitX()).toRotationMatrix(),
      start.translation());
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(flipped)), TaskGoalReject::kNone);
  h.Run(20);
  const auto& pod = h.ctrl->LastTick();
  EXPECT_EQ(pod.tasks[kRight].drop_counts[static_cast<std::size_t>(GoalDrop::kUnusable)], 1U);
  EXPECT_EQ(pod.tasks[kRight].goal_sequence, 0U);
  EXPECT_LE(MaxAbsDiff(h.Command(), kBodyQ), 1e-9);

  // The control: a quarter turn is taken.
  const pinocchio::SE3 quarter(
      start.rotation() * Eigen::AngleAxisd(1.5, Eigen::Vector3d::UnitX()).toRotationMatrix(),
      start.translation());
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(quarter)), TaskGoalReject::kNone);
  h.Run(20);
  EXPECT_EQ(h.ctrl->LastTick().tasks[kRight].goal_sequence, 2U);
}

TEST(DualArmGoalFrames, AFinitePoseTooFarToPlanIsRefused) {
  // Every component is finite, so the ingress takes it — but its distance
  // overflows, and a trajectory of infinite duration has NaN coefficients.
  Harness h;
  (void)h.Tick();
  pinocchio::SE3 far = FramePose(kBodyQ, kHandQ, kRoot, kRightFrame);
  far.translation()[0] = 1e200;
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(far)), TaskGoalReject::kNone);
  for (int i = 0; i < 20; ++i) {
    (void)h.Tick();
    EXPECT_TRUE(h.ctrl->LastTick().converged) << "tick " << i;
  }
  const auto& pod = h.ctrl->LastTick();
  EXPECT_EQ(pod.tasks[kRight].drop_counts[static_cast<std::size_t>(GoalDrop::kUnusable)], 1U);
  EXPECT_EQ(pod.tasks[kRight].goal_sequence, 0U);
  EXPECT_TRUE(std::isfinite(pod.tasks[kRight].ref_pos[0]));
  EXPECT_LE(MaxAbsDiff(h.Command(), kBodyQ), 1e-9);
}

TEST(DualArmGoalFrames, AGoalFromBeforeARebuildIsNotApplied) {
  // The ingress outlives a runtime rebuild; the goal in it names a frame slot
  // of the runtime it was sent to. No activation here, so the generation gate
  // does not help: the rebuild itself has to leave the old goal behind.
  auto ctrl = BringUp();
  const pinocchio::SE3 goal =
      FramePose(kBodyQ, kHandQ, "world", kRoot)
          .act(Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.05, 0.0, 0.0}));
  ASSERT_EQ(ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(goal, "torso_link")), TaskGoalReject::kNone);
  Knobs fewer_frames;
  fewer_frames.target_frames = {"world"};  // slot 2 no longer exists
  ctrl->LoadConfig(YAML::Load(Yaml(fewer_frames)));
  ASSERT_TRUE(ctrl->ConfigError().empty()) << ctrl->ConfigError();
  Harness h(std::move(ctrl));
  h.Run(50);
  EXPECT_EQ(h.ctrl->LastTick().tasks[kRight].goal_sequence, 0U);
  EXPECT_LE(MaxAbsDiff(h.Command(), kBodyQ), 1e-9) << "a goal from the previous runtime moved it";
}

// ── Feed-forward axes ───────────────────────────────────────────────────────

TEST(DualArmFeedForward, TheReferenceTwistAloneCarriesTheFrame) {
  // No feedback at all: task gains 0, posture gain 0. What moves the frame is
  // the feed-forward twist, so it has to be in the axes the solve reads it in
  // — the base frame's. In the frame's own axes (rotated > 0.5 rad against
  // the base here) the hand would leave in another direction.
  Knobs knobs;
  knobs.gain_linear = 0.0;
  knobs.gain_angular = 0.0;
  knobs.posture_gain = 0.0;
  Harness h(knobs);
  (void)h.Tick();
  const pinocchio::SE3 start = FramePose(kBodyQ, kHandQ, kRoot, kRightFrame);
  const pinocchio::SE3 goal(
      start.rotation() * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix(),
      start.translation() + Eigen::Vector3d(0.05, 0.0, 0.03));
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  RunUntilReferencesRest(h, 10);
  const pinocchio::SE3 reached = FramePose(h.Command(), kHandQ, kRoot, kRightFrame);
  // Open-loop integration of a quintic over ~0.6 s: millimetres of drift are
  // expected, centimetres are a wrong axis.
  EXPECT_LE(PositionGap(reached, goal), 3e-3);
  RecordProperty("feedforward_only_err_m", Sci(PositionGap(reached, goal)));
  EXPECT_LE(RotationGap(reached, goal), 2e-2);
  EXPECT_GT(PositionGap(start, goal), 0.05);
}

// ── The device-group lanes ──────────────────────────────────────────────────

TEST(DualArmGroupLanes, AJointGoalHasToNameTheWholeGroupAndATaskGoalIsRefused) {
  Harness h;
  (void)h.Tick();
  const auto goal_before = DualArmTestAccess::Tasks(*h.ctrl)[kRight].goal;

  // The base forwards a task goal on a group topic as six "joint" values.
  const std::vector<double> six = {0.1, 0.2, 0.3, 0.0, 0.0, 0.0};
  h.ctrl->SetDeviceTaskTarget(0, six);
  EXPECT_EQ(h.ctrl->GroupGoalRejectCount(), 1U);
  // A positional goal shorter than the group.
  h.ctrl->SetDeviceTarget(0, six);
  EXPECT_EQ(h.ctrl->GroupGoalRejectCount(), 2U);
  h.Run(20);
  EXPECT_LE(MaxAbsDiff(h.Command(), kBodyQ), 1e-9) << "a refused goal moved the body";

  // The control: all 17 joints is a posture target. It lands in the posture
  // slot and leaves the task goals alone.
  std::vector<double> posture = kBodyQ;
  posture[6] += 0.3;  // left elbow
  h.ctrl->SetDeviceTarget(0, posture);
  EXPECT_EQ(h.ctrl->GroupGoalRejectCount(), 2U);
  h.Run(2000);
  const auto& cache = DualArmTestAccess::Cache(*h.ctrl);
  EXPECT_DOUBLE_EQ(DualArmTestAccess::PostureTarget(*h.ctrl)[cache.ext_to_pin_q(6)], posture[6]);
  EXPECT_EQ(PositionGap(DualArmTestAccess::Tasks(*h.ctrl)[kRight].goal, goal_before), 0.0);
  EXPECT_GT(MaxAbsDiff(h.Command(), kBodyQ), 1e-3) << "the posture target was not followed";
  EXPECT_EQ(h.ctrl->GetTargetUnhandledCount(), 0U);
}

TEST(DualArmGroupLanes, TheHandFollowsItsJointGoalAndIsLockedInTheSolve) {
  Harness h;
  (void)h.Tick();
  (void)h.Tick();
  const std::vector<double> hand_goal = {0.8, 0.1, 0.6, 0.9};
  h.ctrl->SetDeviceTarget(1, hand_goal);
  h.Run(600);
  EXPECT_LE(MaxAbsDiff(h.hand_q, hand_goal), 1e-9);
  // The body did not move to follow the hand: the task frames hang off the
  // hand mount, upstream of every hand joint.
  EXPECT_LE(MaxAbsDiff(h.Command(), kBodyQ), 1e-9);

  // A goal past the device limits stops at them.
  h.ctrl->SetDeviceTarget(1, std::vector<double>{2.5, -1.0, 0.5, 0.5});
  h.Run(600);
  EXPECT_NEAR(h.hand_q[0], 1.5, 1e-9);
  EXPECT_NEAR(h.hand_q[1], 0.0, 1e-9);
}

TEST(DualArmGroupLanes, AHandGoalBehindTheMotionDoesNotOvershoot) {
  Harness h;
  (void)h.Tick();
  (void)h.Tick();
  h.ctrl->SetDeviceTarget(1, std::vector<double>{1.5, 1.5, 1.5, 1.5});  // the upper limit
  for (int i = 0; i < 2000 && h.hand_q[0] < 1.2; ++i) {
    (void)h.Tick();
  }
  ASSERT_GE(h.hand_q[0], 1.2) << "the hand has to be on its way for this to judge";
  // A new goal just behind where the hand is, while it is moving fast. It is
  // applied on the next tick, after that tick's step of the old trajectory.
  const double goal = h.hand_q[0] - 0.05;
  h.ctrl->SetDeviceTarget(1, std::vector<double>{goal, 0.3, 0.1, 0.4});
  const double before = h.hand_q[0];
  (void)h.Tick();
  const double at_restart = h.hand_q[0];
  ASSERT_GT(at_restart, before) << "the hand was not moving when the goal arrived";
  double peak = at_restart;
  for (int i = 0; i < 400; ++i) {
    (void)h.Tick();
    peak = std::max(peak, h.hand_q[0]);
    for (const double q : h.hand_q) {
      EXPECT_GE(q, 0.0);
      EXPECT_LE(q, 1.5);
    }
  }
  // It restarts from rest: it never travels on past where it was restarted.
  EXPECT_LE(peak, at_restart + 1e-12);
  EXPECT_NEAR(h.hand_q[0], goal, 1e-9);
}

TEST(DualArmGroupLanes, APostureGoalBeforeAPendingSeedIsCountedNotLost) {
  Harness h;
  h.Run(5);
  // A stop has ended, but the body has not reported since: the seed is pending.
  h.ctrl->TriggerEstop();
  (void)h.Tick();
  h.ctrl->ClearEstop();
  h.body_valid = false;
  std::vector<double> posture = kBodyQ;
  posture[6] += 0.3;
  h.ctrl->SetDeviceTarget(0, posture);
  (void)h.Tick();
  EXPECT_EQ(h.ctrl->GroupGoalRejectCount(), 1U);
  h.body_valid = true;
  h.Run(200);
  EXPECT_LE(MaxAbsDiff(h.Command(), kBodyQ), 1e-9) << "the posture goal was applied after the seed";
}

TEST(DualArmGroupLanes, AGoalSentWhileInactiveIsDroppedAtActivation) {
  auto ctrl = BringUp();
  const pinocchio::SE3 goal =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.05, 0.0, 0.0});
  // Before the activation: the subscription is alive while Inactive.
  ASSERT_EQ(ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  ASSERT_EQ(ctrl->on_activate(rclcpp_lifecycle::State()),
            rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  Harness h(std::move(ctrl));
  h.Run(200);
  const auto& pod = h.ctrl->LastTick();
  EXPECT_EQ(pod.tasks[kRight].drop_counts[static_cast<std::size_t>(GoalDrop::kStale)], 1U);
  EXPECT_EQ(pod.tasks[kRight].goal_sequence, 0U);
  EXPECT_LE(MaxAbsDiff(h.Command(), kBodyQ), 1e-9) << "a goal from before the activation moved it";

  // The control: the same goal sent after the activation is taken.
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  h.Run(200);
  EXPECT_EQ(h.ctrl->LastTick().tasks[kRight].goal_sequence, 2U);
  EXPECT_GT(MaxAbsDiff(h.Command(), kBodyQ), 1e-3);
}

// ── Failed solves and the fault latch ───────────────────────────────────────

/// Makes every solve fail from the next tick on: a non-finite reference pose
/// is not refused at the call — it reaches the solve and takes the failure
/// branch, the one that rewrites the solve's own outputs.
void PoisonReference(Harness& h, std::size_t k) {
  auto& task = DualArmTestAccess::Tasks(*h.ctrl)[k];
  task.traj_active = false;
  task.goal.translation()[0] = std::numeric_limits<double>::quiet_NaN();
}

TEST(DualArmFault, FiveFailedSolvesLatchAndOnlyTheResetServiceReleases) {
  Harness h;
  h.Run(10);
  const std::vector<double> held = h.Command();
  PoisonReference(h, kRight);
  for (int i = 0; i < 4; ++i) {
    (void)h.Tick();
    EXPECT_TRUE(h.ctrl->LastTick().clik_ran);
    EXPECT_TRUE(h.ctrl->LastTick().reached_solve);
    EXPECT_FALSE(h.ctrl->LastTick().converged);
    EXPECT_FALSE(h.ctrl->HasLatchedFault()) << "latched after " << i + 1 << " failures";
    EXPECT_EQ(h.ctrl->LastTick().qp_fail_streak, i + 1);
    EXPECT_EQ(MaxAbsDiff(h.Command(), held), 0.0) << "a failed solve's output reached the command";
    ExpectOutputAccepted(h);
  }
  (void)h.Tick();
  EXPECT_TRUE(h.ctrl->HasLatchedFault());
  EXPECT_EQ(h.ctrl->LastTick().fault_cause, DualArmDiagLogPod::FaultCause::kQpFailStreak);
  EXPECT_EQ(MaxAbsDiff(h.Command(), held), 0.0);

  // Latched: no solve, the command held, goals refused — and the measurement
  // is moved away from it, so "held" and "followed the measurement" differ.
  for (auto& offset : h.body_offset) {
    offset = 0.02;
  }
  const pinocchio::SE3 goal =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kLeftFrame), {0.05, 0.0, 0.0});
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kLeft, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  for (int i = 0; i < 20; ++i) {
    (void)h.Tick();
    const auto& pod = h.ctrl->LastTick();
    EXPECT_FALSE(pod.clik_ran);
    EXPECT_EQ(pod.hold, Hold::kFault);
    EXPECT_EQ(MaxAbsDiff(h.Command(), held), 0.0);
    for (int j = 0; j < kBodyDof; ++j) {
      EXPECT_EQ(h.out.devices[0].commands[static_cast<std::size_t>(j)],
                held[static_cast<std::size_t>(j)]);
    }
    ExpectOutputAccepted(h);
  }
  EXPECT_EQ(h.ctrl->LastTick().tasks[kLeft].drop_counts[static_cast<std::size_t>(GoalDrop::kHeld)],
            1U);

  // The reset service: cleared on the next tick. The command starts from the
  // MEASUREMENT (offset), not from what was held, every goal is "here", and
  // the solve runs again.
  std::vector<double> measured = held;
  for (auto& value : measured) {
    value += 0.02;
  }
  h.ctrl->ResetFault();
  (void)h.Tick();
  EXPECT_FALSE(h.ctrl->HasLatchedFault());
  EXPECT_TRUE(h.ctrl->LastTick().reseeded);
  EXPECT_TRUE(h.ctrl->LastTick().converged);
  EXPECT_EQ(h.ctrl->LastTick().qp_fail_streak, 0);
  EXPECT_EQ(h.ctrl->LastTick().tasks[kLeft].goal_sequence, 0U) << "the refused goal came back";
  EXPECT_LE(MaxAbsDiff(h.Command(), measured), 1e-9) << "the command was not re-seeded";
  std::fill(h.body_offset.begin(), h.body_offset.end(), 0.0);
  for (int i = 0; i < 50; ++i) {
    (void)h.Tick();
    EXPECT_TRUE(h.ctrl->LastTick().converged) << "tick " << i;
  }
  EXPECT_FALSE(h.ctrl->HasLatchedFault());
  EXPECT_LE(MaxAbsDiff(h.Command(), measured), 1e-9);
}

TEST(DualArmFault, ACallTheSolveRefusesCountsAsAFailure) {
  // A non-finite gain passes through the tick's clamp unchanged (a clamp does
  // not remove a NaN) and the solve refuses the call before it solves.
  Harness h;
  h.Run(10);
  const std::vector<double> held = h.Command();
  auto gains = h.ctrl->GetGains();
  gains.task_gain_linear[kLeft] = std::numeric_limits<double>::quiet_NaN();
  h.ctrl->SetGains(gains);
  for (int i = 0; i < 5; ++i) {
    (void)h.Tick();
    EXPECT_TRUE(h.ctrl->LastTick().rejected_input);
    EXPECT_FALSE(h.ctrl->LastTick().reached_solve);
    ExpectOutputAccepted(h);
  }
  EXPECT_TRUE(h.ctrl->HasLatchedFault());
  EXPECT_EQ(MaxAbsDiff(h.Command(), held), 0.0);
}

TEST(DualArmFault, AnEstopClearDoesNotReleaseItAndTheResetWorksDuringAnEstop) {
  Harness h;
  h.Run(10);
  PoisonReference(h, kRight);
  h.Run(5);
  ASSERT_TRUE(h.ctrl->HasLatchedFault());

  // A global clear is not the reset.
  h.ctrl->TriggerEstop();
  (void)h.Tick();
  h.ctrl->ClearEstop();
  h.Run(3);
  EXPECT_TRUE(h.ctrl->HasLatchedFault()) << "an E-STOP clear released a controller fault";
  EXPECT_FALSE(h.ctrl->LastTick().clik_ran);

  // The manager's reset service works while the global latch is up and reads
  // the latch back two ticks later: the request must not wait for the clear.
  h.ctrl->TriggerEstop();
  (void)h.Tick();
  h.ctrl->ResetFault();
  (void)h.Tick();
  EXPECT_FALSE(h.ctrl->HasLatchedFault()) << "the reset waited for the E-STOP to clear";
  EXPECT_TRUE(h.ctrl->LastTick().estop);
  h.ctrl->ClearEstop();
  h.Run(20);
  EXPECT_FALSE(h.ctrl->HasLatchedFault());
  EXPECT_TRUE(h.ctrl->LastTick().converged);
}

TEST(DualArmFault, TheHandIsHeldUnderTheLatchToo) {
  Harness h;
  h.Run(10);
  h.ctrl->SetDeviceTarget(1, std::vector<double>{1.2, 1.2, 1.2, 1.2});
  h.Run(20);
  ASSERT_GT(h.hand_q[0], kHandQ[0] + 1e-3) << "the hand has to be moving";
  ASSERT_LT(h.hand_q[0], 1.19);
  PoisonReference(h, kRight);
  h.Run(5);
  ASSERT_TRUE(h.ctrl->HasLatchedFault());
  const std::vector<double> held = h.hand_q;
  h.Run(200);
  EXPECT_EQ(MaxAbsDiff(h.hand_q, held), 0.0) << "the hand kept moving under a latched fault";
}

TEST(DualArmFault, AJointThatCannotBeHeldIsReportedByTheSolveNotFailed) {
  // A joint whose torque limit is below its gravity load does not make the
  // solve fail: letting the joint accelerate keeps the torque row. What says
  // so is the solve's own flag, which the log carries per joint — a standstill
  // that reads this flag as zero is one where no joint is in that state.
  auto devices = Devices();
  devices["body"].joint_limits->max_torque[6] = 1e-3;  // left elbow
  Harness h(BringUp({}, &devices));
  ASSERT_TRUE(h.ctrl->ConfigError().empty()) << h.ctrl->ConfigError();
  h.Run(3);
  const auto& pod = h.ctrl->LastTick();
  EXPECT_TRUE(pod.converged);
  const int elbow = DualArmTestAccess::Cache(*h.ctrl).ext_to_pin_v(6);
  EXPECT_NE((pod.brake_static_infeasible >> elbow) & 1U, 0U);
  EXPECT_GT(MaxAbsDiff(h.Command(), kBodyQ), 0.0) << "the joint is expected to sag";
}

TEST(DualArmFault, ATrackingErrorOverItsThresholdLatches) {
  Knobs knobs;
  knobs.track_err_max = 0.01;
  Harness h(knobs);
  h.Run(20);
  EXPECT_FALSE(h.ctrl->HasLatchedFault());
  // The plant stops following: the reference moves, the measurement does not.
  h.servo = false;
  const pinocchio::SE3 goal =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.08, 0.0, 0.0});
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  // With the measurement frozen the command check sees the same q each tick
  // only because the solve is evaluated at its own command, not at the
  // measurement — so it keeps integrating, and the gap grows.
  h.Run(600);
  EXPECT_TRUE(h.ctrl->HasLatchedFault());
  EXPECT_EQ(h.ctrl->LastTick().fault_cause, DualArmDiagLogPod::FaultCause::kTrackError);

  // The control: the threshold at 0 (the shipped value) never latches on it.
  Harness off;
  off.Run(20);
  off.servo = false;
  ASSERT_EQ(off.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  off.Run(600);
  EXPECT_FALSE(off.ctrl->HasLatchedFault());
  EXPECT_GT(off.ctrl->LastTick().track_err, 0.01);
}

// ── E-STOP ──────────────────────────────────────────────────────────────────

TEST(DualArmEstop, TheCommandFreezesAndIsReseededFromTheMeasurementOnRelease) {
  Harness h;
  (void)h.Tick();
  const pinocchio::SE3 goal =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.08, 0.02, 0.0});
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  h.Run(150);
  ASSERT_TRUE(h.ctrl->LastTick().tasks[kRight].traj_active) << "stop it while it is moving";
  const std::vector<double> at_stop = h.Command();
  ASSERT_GT(MaxAbsDiff(at_stop, kBodyQ), 1e-3);

  // Stopped, with the reference still on its way: nothing integrates. The
  // plant is frozen (the manager holds the wire) and the measurement is moved
  // away from the last command.
  h.ctrl->TriggerEstop();
  h.servo = false;
  for (std::size_t i = 0; i < h.body_offset.size(); ++i) {
    h.body_offset[i] = (i % 2 == 0) ? 0.02 : -0.015;
  }
  const pinocchio::SE3 later = Shifted(goal, {0.0, 0.05, 0.0});
  for (int i = 0; i < 100; ++i) {
    if (i == 10) {
      // A goal issued during the stop must not land when it clears.
      ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(later)), TaskGoalReject::kNone);
    }
    (void)h.Tick();
    const auto& pod = h.ctrl->LastTick();
    EXPECT_TRUE(pod.estop);
    EXPECT_FALSE(pod.clik_ran) << "the solve ran under an E-STOP, tick " << i;
    EXPECT_EQ(pod.hold, Hold::kEstop);
    EXPECT_FALSE(pod.tasks[kRight].valid) << "a held tick reported a pose error";
    EXPECT_EQ(MaxAbsDiff(h.Command(), at_stop), 0.0) << "the command moved under an E-STOP";
    // What goes out is the measurement of that tick.
    for (int j = 0; j < kBodyDof; ++j) {
      EXPECT_EQ(h.out.devices[0].commands[static_cast<std::size_t>(j)],
                h.state.devices[0].positions[static_cast<std::size_t>(j)]);
    }
    ExpectOutputAccepted(h);
  }

  // Released: the command starts where the robot IS, and nothing resumes.
  h.ctrl->ClearEstop();
  std::vector<double> measured = h.body_q;
  for (std::size_t i = 0; i < measured.size(); ++i) {
    measured[i] += h.body_offset[i];
  }
  ASSERT_GT(MaxAbsDiff(measured, at_stop), 1e-2) << "the offset is what tells the two apart";
  (void)h.Tick();
  const auto& pod = h.ctrl->LastTick();
  EXPECT_TRUE(pod.reseeded);
  EXPECT_TRUE(pod.clik_ran);
  EXPECT_TRUE(pod.converged);
  EXPECT_EQ(pod.tasks[kRight].goal_sequence, 0U) << "a goal survived the stop";
  EXPECT_FALSE(pod.tasks[kRight].traj_active);
  EXPECT_LE(MaxAbsDiff(h.Command(), measured), 1e-9) << "the command was not re-seeded";

  // From here the servo follows again, from the re-seeded command.
  h.body_q = measured;
  std::fill(h.body_offset.begin(), h.body_offset.end(), 0.0);
  h.servo = true;
  for (int i = 0; i < 300; ++i) {
    (void)h.Tick();
    EXPECT_TRUE(h.ctrl->LastTick().converged) << "tick " << i;
    EXPECT_FALSE(h.ctrl->LastTick().command_mismatch) << "tick " << i;
  }
  EXPECT_FALSE(h.ctrl->HasLatchedFault());
  EXPECT_LE(MaxAbsDiff(h.Command(), measured), 1e-9) << "the motion resumed after the release";
  EXPECT_EQ(h.ctrl->LastTick().tasks[kRight].drop_counts[static_cast<std::size_t>(GoalDrop::kHeld)],
            1U);
}

TEST(DualArmEstop, ATriggerAndClearBetweenTwoTicksStillReseeds) {
  Harness h;
  (void)h.Tick();
  const pinocchio::SE3 goal =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.08, 0.0, 0.0});
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  h.Run(100);
  ASSERT_TRUE(h.ctrl->LastTick().tasks[kRight].traj_active);
  h.ctrl->TriggerEstop();
  h.ctrl->ClearEstop();  // the tick never sees the flag raised
  (void)h.Tick();
  EXPECT_FALSE(h.ctrl->LastTick().estop);
  EXPECT_TRUE(h.ctrl->LastTick().reseeded);
  EXPECT_FALSE(h.ctrl->LastTick().tasks[kRight].traj_active) << "the motion carried on";
  EXPECT_EQ(h.ctrl->LastTick().tasks[kRight].goal_sequence, 0U);
}

TEST(DualArmEstop, TheFlagAloneStopsTheSolve) {
  // TriggerEstop() raises the flag and then moves the epoch, from another
  // thread. A tick can land between the two: it reads the flag up and an
  // epoch that has not moved, so nothing has asked for a re-seed yet. The
  // flag by itself has to stop the solve on that tick.
  Harness h;
  (void)h.Tick();
  const pinocchio::SE3 goal =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.08, 0.0, 0.0});
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  h.Run(100);
  ASSERT_TRUE(h.ctrl->LastTick().tasks[kRight].traj_active);
  const std::vector<double> at_stop = h.Command();

  h.ctrl->TriggerEstop();
  DualArmTestAccess::MarkEstopEpochServiced(*h.ctrl);
  (void)h.Tick();
  EXPECT_TRUE(h.ctrl->LastTick().estop);
  EXPECT_FALSE(h.ctrl->LastTick().clik_ran) << "the solve ran on a tick that saw the E-STOP flag";
  EXPECT_EQ(MaxAbsDiff(h.Command(), at_stop), 0.0);
}

// ── Unreadable devices ──────────────────────────────────────────────────────

TEST(DualArmReadability, AnUnreadableBodyFreezesTheSolveAndSilencesTheGroup) {
  Harness h;
  (void)h.Tick();
  const pinocchio::SE3 goal =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.08, 0.0, 0.0});
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  h.Run(100);
  const std::vector<double> before = h.Command();
  const pinocchio::SE3 ref_before = DualArmTestAccess::Tasks(*h.ctrl)[kRight].ref_pose;

  h.body_valid = false;
  for (int i = 0; i < 50; ++i) {
    (void)h.Tick();
    const auto& pod = h.ctrl->LastTick();
    EXPECT_FALSE(pod.clik_ran);
    EXPECT_EQ(pod.hold, Hold::kUnreadable);
    EXPECT_EQ(h.out.devices[0].num_channels, 0) << "an unreadable group was commanded";
    EXPECT_GT(h.out.devices[1].num_channels, 0) << "the hand was silenced with it";
    EXPECT_FALSE(h.out.arm_tip_pose_valid) << "a transform was published from no measurement";
    EXPECT_EQ(MaxAbsDiff(h.Command(), before), 0.0);
    ExpectOutputAccepted(h);
  }
  // Frozen, not advanced open-loop: the reference is where it was.
  EXPECT_EQ(PositionGap(DualArmTestAccess::Tasks(*h.ctrl)[kRight].ref_pose, ref_before), 0.0);

  // Readable again: the motion continues from where it stopped, with no jump.
  h.body_valid = true;
  (void)h.Tick();
  EXPECT_TRUE(h.ctrl->LastTick().clik_ran);
  EXPECT_TRUE(h.ctrl->LastTick().converged);
  EXPECT_FALSE(h.ctrl->LastTick().reseeded);
  EXPECT_LT(MaxAbsDiff(h.Command(), before), 5e-3);
  h.Run(1500);
  EXPECT_LE(PositionGap(FramePose(h.Command(), kHandQ, kRoot, kRightFrame), goal), 2e-3);
}

TEST(DualArmReadability, ANonFiniteMeasurementIsNotSeededFromNorCommanded) {
  // The readability gate judges width and freshness, not values.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  {
    Harness h;
    h.body_offset[4] = nan;  // from the first tick on
    for (int i = 0; i < 10; ++i) {
      (void)h.Tick();
      EXPECT_FALSE(h.ctrl->LastTick().reseeded) << "seeded from a NaN measurement";
      EXPECT_EQ(h.out.devices[0].num_channels, 0);
      ExpectOutputAccepted(h);
    }
    h.body_offset[4] = 0.0;
    (void)h.Tick();
    EXPECT_TRUE(h.ctrl->LastTick().reseeded);
    EXPECT_LE(MaxAbsDiff(h.Command(), kBodyQ), 1e-15);
  }
  // After a stop, with the measurement gone non-finite: the stop's own hold
  // must not put it on the wire, and the release must not seed from it.
  Harness h;
  h.Run(10);
  const std::vector<double> before = h.Command();
  h.ctrl->TriggerEstop();
  h.servo = false;
  h.body_offset[2] = nan;
  for (int i = 0; i < 5; ++i) {
    (void)h.Tick();
    EXPECT_EQ(h.out.devices[0].num_channels, 0);
    ExpectOutputAccepted(h);
  }
  h.ctrl->ClearEstop();
  for (int i = 0; i < 5; ++i) {
    (void)h.Tick();
    EXPECT_FALSE(h.ctrl->LastTick().reseeded);
    EXPECT_EQ(h.out.devices[0].num_channels, 0);
    ExpectOutputAccepted(h);
  }
  for (const double q : h.Command()) {
    EXPECT_TRUE(std::isfinite(q));
  }
  h.body_offset[2] = 0.0;
  h.servo = true;
  (void)h.Tick();
  EXPECT_TRUE(h.ctrl->LastTick().reseeded);
  EXPECT_LE(MaxAbsDiff(h.Command(), before), 1e-9);
}

TEST(DualArmReadability, AnUnreadableHandSilencesTheHandOnly) {
  Harness h;
  h.Run(5);
  h.hand_valid = false;
  for (int i = 0; i < 20; ++i) {
    (void)h.Tick();
    EXPECT_TRUE(h.ctrl->LastTick().clik_ran) << "the body stopped for the hand";
    EXPECT_EQ(h.out.devices[1].num_channels, 0);
    EXPECT_EQ(h.out.devices[0].num_channels, kBodyDof);
    ExpectOutputAccepted(h);
  }
}

TEST(DualArmReadability, TheSeedWaitsForAReadableTick) {
  Harness h;
  h.body_valid = false;
  for (int i = 0; i < 10; ++i) {
    (void)h.Tick();
    EXPECT_FALSE(h.ctrl->LastTick().reseeded);
    EXPECT_EQ(h.ctrl->LastTick().hold, Hold::kNotSeeded);
    EXPECT_EQ(h.out.devices[0].num_channels, 0);
    ExpectOutputAccepted(h);
  }
  h.body_valid = true;
  (void)h.Tick();
  EXPECT_TRUE(h.ctrl->LastTick().reseeded);
  EXPECT_LE(MaxAbsDiff(h.Command(), kBodyQ), 1e-15);
}

// ── Per-task error, gains ───────────────────────────────────────────────────

TEST(DualArmDiagnostics, TheErrorOfTaskZeroIsTheCoresInEitherTaskOrder) {
  for (const bool left_first : {false, true}) {
    Knobs knobs;
    knobs.left_first = left_first;
    Harness h(knobs);
    (void)h.Tick();
    const std::size_t right = left_first ? 1 : 0;
    const std::size_t left = left_first ? 0 : 1;
    const pinocchio::SE3 right_goal =
        Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.05, 0.0, 0.02});
    const pinocchio::SE3 left_goal =
        Shifted(FramePose(kBodyQ, kHandQ, kLeftBase, kLeftFrame), {0.0, 0.04, 0.0});
    ASSERT_EQ(h.ctrl->DeliverTaskGoal(right, TaskGoalMsg(right_goal)), TaskGoalReject::kNone);
    ASSERT_EQ(h.ctrl->DeliverTaskGoal(left, TaskGoalMsg(left_goal)), TaskGoalReject::kNone);
    for (int i = 0; i < 200; ++i) {
      (void)h.Tick();
      // Same formula, same inputs as the core's own report of task 0.
      EXPECT_EQ(DualArmTestAccess::Tasks(*h.ctrl)[0].err_norm,
                DualArmTestAccess::Clik(*h.ctrl).TcpErrorNorm())
          << "left_first=" << left_first << " tick " << i;
    }
    EXPECT_GT(DualArmTestAccess::Tasks(*h.ctrl)[0].err_norm, 0.0);
    EXPECT_GT(DualArmTestAccess::Tasks(*h.ctrl)[1].err_norm, 0.0);
    const auto& pod = h.ctrl->LastTick();
    EXPECT_NEAR(std::hypot(pod.tasks[0].err_lin, pod.tasks[0].err_ang),
                DualArmTestAccess::Tasks(*h.ctrl)[0].err_norm, 1e-15);
  }
}

TEST(DualArmGains, AreBoundedWhereTheTickUsesThem) {
  Harness h;
  (void)h.Tick();
  auto gains = h.ctrl->GetGains();
  gains.task_gain_linear[kRight] = 10.0 / kDt;  // far past 1/dt
  gains.task_gain_angular[kRight] = -3.0;
  h.ctrl->SetGains(gains);
  (void)h.Tick();
  const auto& task = DualArmTestAccess::FrameTasks(*h.ctrl)[kRight];
  EXPECT_DOUBLE_EQ(task.gain[0], 1.0 / kDt);
  EXPECT_DOUBLE_EQ(task.gain[3], 0.0);
  EXPECT_TRUE(h.ctrl->LastTick().converged);

  // A speed of zero is floored where it divides, not divided by.
  gains = h.ctrl->GetGains();
  gains.linear_speed = 0.0;
  gains.angular_speed = 0.0;
  h.ctrl->SetGains(gains);
  const pinocchio::SE3 goal =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.01, 0.0, 0.0});
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(kRight, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  (void)h.Tick();
  EXPECT_TRUE(h.ctrl->LastTick().tasks[kRight].traj_active);
  EXPECT_TRUE(std::isfinite(h.ctrl->LastTick().tasks[kRight].ref_pos[0]));
  EXPECT_TRUE(h.ctrl->LastTick().converged);
}

// ── Transforms: the measured poses ──────────────────────────────────────────

TEST(DualArmTransforms, AreTheMeasuredPosesNotTheCommandedOnes) {
  Harness h;
  h.Run(5);
  // Measurement and command apart: the tick that publishes must read the first.
  for (std::size_t i = 0; i < h.body_offset.size(); ++i) {
    h.body_offset[i] = (i % 3 == 0) ? 0.03 : -0.02;
  }
  h.servo = false;
  (void)h.Tick();
  std::vector<double> measured = h.body_q;
  for (std::size_t i = 0; i < measured.size(); ++i) {
    measured[i] += h.body_offset[i];
  }
  const std::vector<double> commanded = h.Command();
  ASSERT_GT(MaxAbsDiff(measured, commanded), 1e-2);

  ASSERT_TRUE(h.out.arm_tip_pose_valid);
  const pinocchio::SE3 tip = ToSe3(h.out.arm_tip_pose);
  EXPECT_LE(PositionGap(tip, FramePose(measured, h.hand_q, kRoot, kHandMount)), 1e-12);
  EXPECT_LE(RotationGap(tip, FramePose(measured, h.hand_q, kRoot, kHandMount)), 1e-7);
  EXPECT_GT(PositionGap(tip, FramePose(commanded, h.hand_q, kRoot, kHandMount)), 1e-4);

  // Slots: the fingertips first, then one per task frame.
  for (std::size_t f = 0; f < kTips.size(); ++f) {
    ASSERT_TRUE(h.out.task_link_pose_valid[f]) << kTips[f];
    EXPECT_LE(PositionGap(ToSe3(h.out.task_link_poses[f]),
                          FramePose(measured, h.hand_q, kRoot, kTips[f])),
              1e-12)
        << kTips[f];
  }
  const std::vector<std::string> task_frames = {kRightFrame, kLeftFrame};
  const std::vector<std::string> task_bases = {kRoot, kLeftBase};
  for (std::size_t k = 0; k < task_frames.size(); ++k) {
    const std::size_t slot = kTips.size() + k;
    ASSERT_TRUE(h.out.task_link_pose_valid[slot]) << task_frames[k];
    EXPECT_LE(PositionGap(ToSe3(h.out.task_link_poses[slot]),
                          FramePose(measured, h.hand_q, kRoot, task_frames[k])),
              1e-12)
        << task_frames[k];
    // The log's measured pose is in the task's BASE frame.
    const auto& row = h.ctrl->LastTick().tasks[k];
    ASSERT_TRUE(row.meas_valid);
    EXPECT_LE(PositionGap(ToSe3(row.meas_pos, row.meas_quat),
                          FramePose(measured, h.hand_q, task_bases[k], task_frames[k])),
              1e-12);
    // ... and its commanded pose is the one at the command the solve read.
    ASSERT_TRUE(row.valid);
    EXPECT_GT(PositionGap(ToSe3(row.cmd_pos, row.cmd_quat), ToSe3(row.meas_pos, row.meas_quat)),
              1e-4);
  }
}

// ── Lifecycle on a real node ────────────────────────────────────────────────

class DualArmLifecycle : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  /// The controller manager's order: PreConfigure, device configs, on_configure.
  static rtc::RTControllerInterface::CallbackReturn Configure(
      DemoDualArmController& ctrl, const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
      const Knobs& knobs) {
    const YAML::Node yaml = YAML::Load(Yaml(knobs));
    if (ctrl.PreConfigure(node, yaml) != rtc::RTControllerInterface::CallbackReturn::SUCCESS) {
      return rtc::RTControllerInterface::CallbackReturn::FAILURE;
    }
    ctrl.SetDeviceNameConfigs(Devices(knobs.with_hand));
    return ctrl.on_configure(rclcpp_lifecycle::State(), node, yaml);
  }
};

TEST_F(DualArmLifecycle, ConfiguresItsTopicsAndRefusesAnUnresolvedFrame) {
  using Ret = rtc::RTControllerInterface::CallbackReturn;
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dualarm_refuse");
    auto ctrl = Make();
    Knobs knobs;
    knobs.left_frame = "no_such_frame";
    EXPECT_EQ(Configure(*ctrl, node, knobs), Ret::FAILURE);
  }
  {
    // A configure that fails AFTER it created its subscriptions leaves none
    // behind for the next attempt to duplicate.
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dualarm_half");
    auto ctrl = Make();
    Knobs knobs;
    knobs.log_entry_without_instance = true;
    EXPECT_EQ(Configure(*ctrl, node, knobs), Ret::FAILURE);
    EXPECT_TRUE(DualArmTestAccess::Topics(*ctrl).task_goal_subs.empty());
    EXPECT_EQ(DualArmTestAccess::Topics(*ctrl).num_tf_slots, 0);
  }
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dualarm_ok");
  auto ctrl = Make();
  ASSERT_EQ(Configure(*ctrl, node, {}), Ret::SUCCESS);

  const auto& topics = DualArmTestAccess::Topics(*ctrl);
  // One goal subscription per task, on the default (non-RT) callback group,
  // depth 1.
  ASSERT_EQ(topics.task_goal_subs.size(), 2U);
  const auto default_group = node->get_node_base_interface()->get_default_callback_group();
  for (const auto& sub : topics.task_goal_subs) {
    EXPECT_EQ(sub->get_actual_qos().depth(), 1U);
    const auto found = default_group->find_subscription_ptrs_if(
        [&sub](const rclcpp::SubscriptionBase::SharedPtr& candidate) { return candidate == sub; });
    EXPECT_NE(found, nullptr) << sub->get_topic_name() << " is not on the default callback group";
  }
  EXPECT_NE(std::string(topics.task_goal_subs[0]->get_topic_name()).find("right_hand/task_goal"),
            std::string::npos);
  EXPECT_NE(std::string(topics.task_goal_subs[1]->get_topic_name()).find("left_hand/task_goal"),
            std::string::npos);

  // Transform slots: the hand mount, the fingertips, then the task frames —
  // all under the body group's root link.
  const std::vector<std::string> want = {"hand_base_actual", "finger_a_tip_actual",
                                         "finger_b_tip_actual", "catch_frame_actual",
                                         "left_tool_actual"};
  ASSERT_EQ(static_cast<std::size_t>(topics.num_tf_slots), want.size());
  for (std::size_t i = 0; i < want.size(); ++i) {
    EXPECT_EQ(topics.tf_slots[i].child_frame_id, want[i]);
    EXPECT_EQ(topics.tf_slots[i].parent_frame_id, kRoot);
    EXPECT_TRUE(topics.tf_slots[i].slot_valid);
  }

  // Parameters: a gain past 1/dt is refused, a speed of zero is floored.
  const auto refused =
      node->set_parameter(rclcpp::Parameter("tasks.right_hand.gain_linear", 1.0 / kDt + 1.0));
  EXPECT_FALSE(refused.successful);
  EXPECT_DOUBLE_EQ(ctrl->GetGains().task_gain_linear[kRight], 10.0);
  EXPECT_TRUE(
      node->set_parameter(rclcpp::Parameter("tasks.right_hand.gain_linear", 20.0)).successful);
  EXPECT_DOUBLE_EQ(ctrl->GetGains().task_gain_linear[kRight], 20.0);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("posture.trunk.gain", 0.5)).successful);
  EXPECT_DOUBLE_EQ(ctrl->GetGains().posture_gain[0], 0.5);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("trajectory.linear_speed",
                                                     std::numeric_limits<double>::quiet_NaN()))
                   .successful);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("trajectory.linear_speed", 0.0)).successful);
  EXPECT_DOUBLE_EQ(ctrl->GetGains().linear_speed, 1e-6);

  // A cleanup → configure cycle comes up again (fresh cache, fresh slots).
  EXPECT_EQ(ctrl->on_cleanup(rclcpp_lifecycle::State()), Ret::SUCCESS);
  EXPECT_EQ(Configure(*ctrl, node, {}), Ret::SUCCESS);
  EXPECT_EQ(DualArmTestAccess::Topics(*ctrl).task_goal_subs.size(), 2U);
  EXPECT_EQ(static_cast<std::size_t>(DualArmTestAccess::Topics(*ctrl).num_tf_slots), want.size());
}

}  // namespace
