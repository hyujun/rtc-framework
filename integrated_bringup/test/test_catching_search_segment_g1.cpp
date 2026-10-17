// The search on a chain, the segments on the device: the shipped g1_p1b models
// through the planner (dynamic_catching E3-F01, #640 — Sprint Contract 3, 4, 5).
//
// On G1 the catch SEARCH runs on `g1_catch` (pelvis → the right palm: waist 3 +
// right arm 7, 10 joints, the left arm locked) and the SEGMENTS are planned on
// `g1` (the whole arm device, 17 joints). rtc_controllers' own suites pin the
// widening rule on synthetic arms with one joint taken out; this file runs it
// on the two models the profile declares.
//
//   pair       G1SearchSegmentPair.* — each of the four combinations (grid, nlp
//              × mpc, mpc_docking) publishes a plan of the chain's joints and a
//              17-joint segment in ONE planner wake: the first wake its search
//              plans on (the grid search settles for `n_settle` snapshots of a
//              track first and publishes "no plan" meanwhile). The joints the
//              chain lacks start, and are referenced, on the reported command.
//   agreement  G1SearchSegmentAgreement.* — "two solves of the same problem at
//              two widths differ only by solver convergence". With the joints
//              the chain lacks at rest ON THE POSE THE CHAIN MODEL LOCKS THEM
//              AT, no torque row active and NO TORQUE COST, the chain's joints
//              of the 17-joint solution are the 10-joint solution's: 1e-4 rad
//              at every node and 0.1 mm of the catch frame at the catch node
//              (decision D10 — the two numbers were fixed before this was run
//              and are not tuned to it), and the mpc_docking solve that starts
//              from the widened chain solution ends in one SQP iteration. Two
//              tests:
//                MpcOnTheTreeSolvesTheChainsProblem — the mpc planner on each
//                  model (its cost has no torque term);
//                WithNoTorqueCostMpcDockingOnTheTreeIsTheChainsProblem — the
//                  docking cores with their torque cost off, so that no term
//                  reads a joint the chain lacks.
//              "No torque cost" is part of the premise, not a convenience
//              (user decision, 2026-10-14): a torque cost on the tree weighs
//              the torque of the joints the chain lacks, and the chain's own
//              torque depends on where those joints are, so the tree's
//              optimum moves them and the two problems differ. Measured with
//              the docking fixture's cost (`r_tau` 1e-4 on every joint of each
//              model) on this scene: the joints the chain lacks move 5e-2 rad,
//              the chain's 3e-3 rad against 1e-4, three SQP iterations against
//              one — the torque rows inactive all the while. Where G1's torque
//              cost goes is E3-F02's (#641) to decide.
//              With no torque row active and no torque cost, the dynamics is in
//              neither problem: that the chain model IS the tree with its other
//              joints on the locked pose is checked directly, by the two
//              models' inverse dynamics (ChainTorqueGap), not by a solve.
//   times      G1SearchSegmentTimes.* — one candidate of the nlp search on the
//              chain, and the first solve of each segment planner on the tree,
//              against the real clock. RECORDED, not judged: G1 has no budget
//              yet (E3-F05), and the host is whatever this runs on.
//
// WHAT IS BORROWED. g1_p1b ships no catching configuration (the wait pose, the
// budgets and the hand's docking values are E3-F05's). The wait pose is the
// fixture's (kG1Home); the docking cores take the docking fixture's synthetic
// capture set (mpc_docking::BaseParams); the grid search's and the mpc
// planner's values, the nlp search's lattice and the two planners' grids are
// read from the shipped ur5e_p1b catching profile — the same hand on another
// arm. The search's cores judge kinematics only (`torque_rows` false): the
// chain's inverse dynamics is not the robot's when the left arm is elsewhere.
//
// CLOCK. The pair and the agreement run on a PINNED clock: a solve takes none
// of the test's time, so no budget or lead decides a value. The times run on
// the real steady clock, with budgets that cut nothing.
#include "g1_p1b_test_fixture.hpp"
#include "rtc_base/threading/seqlock.hpp"
#include "rtc_base/types/types.hpp"
#include "rtc_controllers/catching/catch_pose_ik_params.hpp"
#include "rtc_controllers/catching/docking_params.hpp"
#include "rtc_controllers/catching/grid_catch_search.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_planner.hpp"
#include "rtc_controllers/catching/mpc_segment_planner.hpp"
#include "rtc_controllers/catching/nlp_catch_search.hpp"
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "rtc_controllers/catching/planner_params.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"
#include "rtc_controllers/testing/grid_catch_search_fixture.hpp"
#include "rtc_controllers/testing/mpc_docking_fixture.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "shipped_config_test_fixture.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace dk = rtc::testing::mpc_docking;
namespace fx = integrated_bringup::testfx;
using rtc::catching::BallPrediction;
using rtc::catching::CatchPoseIkOptions;
using rtc::catching::CatchSolution;
using rtc::catching::CovarianceSnapshot;
using rtc::catching::CycleOutcome;
using rtc::catching::CycleOutcomeName;
using rtc::catching::GridCatchSearch;
using rtc::catching::GridCatchSearchConstants;
using rtc::catching::GridCatchSearchModel;
using rtc::catching::InJointSet;
using rtc::catching::JointSet;
using rtc::catching::JointSetSlot;
using rtc::catching::kMaxSegmentNv;
using rtc::catching::MpcDockingSegmentCoreInput;
using rtc::catching::MpcDockingSegmentCoreResult;
using rtc::catching::MpcDockingSegmentPlanner;
using rtc::catching::MpcDockingSegmentPlannerConstants;
using rtc::catching::MpcDockingSegmentPlannerModel;
using rtc::catching::MpcDockingSegmentPlannerParams;
using rtc::catching::MpcSegmentCoreInput;
using rtc::catching::MpcSegmentPlanner;
using rtc::catching::MpcSegmentPlannerConstants;
using rtc::catching::MpcSegmentPlannerModel;
using rtc::catching::MpcSegmentPlannerParams;
using rtc::catching::NlpCatchSearch;
using rtc::catching::NlpCatchSearchConstants;
using rtc::catching::NlpCatchSearchModel;
using rtc::catching::NlpCatchSearchParams;
using rtc::catching::NlpRejectName;
using rtc::catching::NodeTrajectoryFollower;
using rtc::catching::NowReal;
using rtc::catching::PlannerCycle;
using rtc::catching::PlannerCycleIo;
using rtc::catching::PlannerCycleRecord;
using rtc::catching::PlannerParams;
using rtc::catching::PlannerRtState;
using rtc::catching::PlanSnapshot;
using rtc::catching::ReportedSegments;
using rtc::catching::SearchStats;
using rtc::catching::SegmentKind;
using rtc::catching::SegmentNodeSample;
using rtc::catching::SegmentNodeTimeNs;
using rtc::catching::SegmentOutcome;
using rtc::catching::SegmentOutcomeName;
using rtc::catching::SegmentRecord;
using rtc::catching::SegmentSnapshot;
using rtc::catching::TrajectorySnapshot;
using rtc::catching::ValidateSegmentNodes;
using rtc::testing::BitsEqual;

using DevicePose = std::array<double, fx::kG1Dof>;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kPinnedNow = 10'000 * kMs;
constexpr std::uint64_t kActivation = 3;
constexpr std::uint64_t kTrack = 7;
constexpr double kControlDt = 0.002;
// Between two wakes of the pinned clock: a vision period.
constexpr std::int64_t kWakePeriodNs = 33 * kMs;

// The agreement's tolerances — decision D10 (#640), fixed before the first
// run. A value over either is reported, never a reason to move the number.
constexpr double kAgreeJointRad = 1e-4;  // chain joints, every node [rad]
constexpr double kAgreeCatchM = 1e-4;    // catch frame at the catch node [m]

// The ball: 0.8 m/s (the docking fixture's reference closing speed) down the
// capture axis of the AIM posture below, on that posture's entrance plane
// kStarAfterT0S after the nlp search's earliest start instant — mid-window for
// the nlp lattice and inside the grid search's lead window.
constexpr double kBallSpeed = 0.8;
constexpr double kBallSigma = 0.002;
constexpr double kStarAfterT0S = 0.65;

// The aim posture: the wait pose with the chain's joints moved by these
// (DEVICE order; the left arm, which the chain lacks, by nothing). The throw
// is aimed at ITS capture axis, so that catching it moves the arm — a ball
// down the wait pose's own axis is caught by standing still, and two solves
// that both stand still agree whatever the model.
constexpr DevicePose kAimOffset = {0.12,  0.0,   0.06,  // waist yaw, roll, pitch
                                   0.0,   0.0,   0.0,  0.0,  0.0, 0.0,  0.0,   // left arm
                                   -0.20, -0.10, 0.10, 0.25, 0.0, 0.10, 0.0};  // right arm
// How far the chosen catch pose has to be from the wait pose for the scene to
// show anything [rad, the largest chain joint].
constexpr double kMinReachRad = 0.05;

// ── The clock ────────────────────────────────────────────────────────────────

std::atomic<std::int64_t> g_pinned_now{0};  // 0 = the real steady clock

std::int64_t SuiteClock() noexcept {
  const std::int64_t pinned = g_pinned_now.load(std::memory_order_relaxed);
  return pinned != 0 ? pinned : rtc::SteadyNowNs();
}

struct PinnedClock {
  explicit PinnedClock(std::int64_t now) { g_pinned_now.store(now); }

  ~PinnedClock() { g_pinned_now.store(0); }

  void Set(std::int64_t now) { g_pinned_now.store(now); }

  PinnedClock(const PinnedClock&) = delete;
  PinnedClock& operator=(const PinnedClock&) = delete;
};

// ── What is borrowed from the shipped ur5e_p1b catching profile ──────────────

struct Borrowed {
  PlannerParams planner;                   // the grid search and the mpc planner
  CatchPoseIkOptions grid_ik;              // planner.search.grid.ik
  CatchPoseIkOptions nlp_ik;               // planner.search.nlp.ik
  GridCatchSearchConstants grid_consts;    // the keys outside planner.*
  NlpCatchSearchParams nlp;                // the lattice, the grid, the budget — not `core`
  MpcDockingSegmentPlannerParams docking;  // the grid, the publish thresholds — not `core`
  double t_arm_s{0.0};
};

Borrowed LoadBorrowed() {
  constexpr int kBorrowedNv = 6;  // the profile's own arm: its per-joint lists
  const YAML::Node c =
      fx::ShippedControllerNode("ur5e_p1b", "demo_catching_controller")["catching"];
  Borrowed b;
  b.planner = rtc::catching::ParsePlannerParams(c);
  b.grid_ik = rtc::catching::ParseCatchPoseIkParams(c).options;
  b.nlp_ik = rtc::catching::ParseCatchPoseIkParams(c, nullptr, "nlp").options;
  b.nlp = rtc::catching::ParseNlpSearchParams(c, kBorrowedNv);
  b.docking = rtc::catching::ParseMpcDockingSegmentParams(c, kBorrowedNv);
  b.t_arm_s = c["joint_cmd"]["lag"]["T_arm"].as<double>();
  const YAML::Node grid = c["planner"]["search"]["grid"];
  GridCatchSearchConstants& pc = b.grid_consts;
  pc.eta_v = grid["gamma"]["eta_v"].as<double>();
  pc.v_max = grid["reference"]["v_max"].as<double>();
  pc.a_dec = grid["stop"]["a_dec"].as<double>();
  pc.t_arm_s = b.t_arm_s;
  pc.t_close_lead = c["robot"]["hand"]["T_close_lead"].as<double>();
  pc.t_close_total = c["robot"]["hand"]["T_close_e2e"].as<double>() + 0.5 * kControlDt;
  pc.ball_mass = c["core"]["ball"]["mass"].as<double>();
  pc.ref_omega = grid["reference"]["omega"].as<double>();
  pc.ref_zeta = grid["reference"]["zeta"].as<double>();
  pc.ref_a_max = grid["reference"]["a_max"].as<double>();
  pc.control_dt = kControlDt;
  pc.follows_segments = true;  // every combination here runs a segment planner
  return b;
}

// Budgets. kShipped: the borrowed profile's — on the pinned clock they cut
// nothing. kGenerous: for the real clock, so that a loaded host times the
// work and not a deadline.
enum class Budget { kShipped, kGenerous };
constexpr double kGenerousSearchS = 2.0;
constexpr double kGenerousSolveS = 1.0;

// ── The scene: the two models, the command the arm rests on ──────────────────

struct Throw {
  TrajectorySnapshot traj;
  CovarianceSnapshot cov;
};

struct Scene {
  fx::ShippedArm search;                                   // g1_catch, 10
  fx::ShippedArm segment;                                  // g1, 17
  std::unique_ptr<rtc_urdf_bridge::RtModelHandle> handle;  // the search's, for the IK
  Borrowed borrowed;
  DevicePose command{};       // where the arm rests: the wait pose, DEVICE order
  JointSet chain{0};          // the search model's joints, as device indices
  std::vector<int> outside;   // the device joints the chain lacks
  std::vector<int> slot_map;  // search model joint m → its column in a chain solution
  // E3-F02: the tree's segment planners built with planner.groups declared
  // (waist, left arm, right arm — device order) and no key of a group's own.
  bool groups_declared{false};
  // ... and, with the groups, the whole-body terms of the tree's mpc planner:
  // the left arm's rest weight and the angular-momentum rate (s = 10 N·m).
  double wb_rest_left{0.0};
  double wb_ang_pre{0.0};
  double wb_ang_post{0.0};
  // ... and the left arm's own move blocking: two pre-catch intervals per
  // block (the mpc planner's stop part in [2, 2, 3] too).
  bool wb_coarse_left{false};

  [[nodiscard]] rtc::catching::JointGroups DeclaredGroups() const {
    rtc::catching::JointGroups g;
    if (groups_declared) {
      g.n_groups = 3;
      for (int d = 0; d < fx::kG1Dof; ++d) {
        g.group_of_device[static_cast<std::size_t>(d)] =
            static_cast<std::int8_t>(d < 3 ? 0 : (d < 10 ? 1 : 2));
      }
    }
    return g;
  }

  // gtest ASSERTs: call as ASSERT_NO_FATAL_FAILURE(scene->Build(pose)).
  void Build(const DevicePose& pose) {
    command = pose;
    std::vector<fx::ShippedArm> models =
        fx::LoadShippedModels("g1_p1b", "_base.yaml", fx::MakeG1P1bModelConfig(),
                              {fx::kG1SearchModel, fx::kG1SegmentModel}, "g1",
                              std::vector<double>(pose.begin(), pose.end()));
    ASSERT_EQ(models.size(), 2U);
    search = std::move(models[0]);
    segment = std::move(models[1]);
    ASSERT_TRUE(search.rig.arm.model);
    ASSERT_TRUE(segment.rig.arm.model);
    ASSERT_EQ(search.rig.arm.model->nv, fx::kG1CatchDof);
    ASSERT_EQ(segment.rig.arm.model->nv, fx::kG1Dof);
    ASSERT_EQ(search.n_dev, fx::kG1Dof);
    ASSERT_EQ(segment.n_dev, fx::kG1Dof);
    ASSERT_TRUE(rtc::catching::MakeJointSet(search.device_of_model, fx::kG1Dof, chain));
    JointSet whole = 0;
    ASSERT_TRUE(rtc::catching::MakeJointSet(segment.device_of_model, fx::kG1Dof, whole));
    ASSERT_EQ(whole, rtc::catching::FullJointSet(fx::kG1Dof));
    // waist 3 and right arm 7 of (waist, left arm, right arm).
    ASSERT_EQ(chain, JointSet{0x1FC07});
    for (int d = 0; d < fx::kG1Dof; ++d) {
      if (!InJointSet(chain, d)) {
        outside.push_back(d);
      }
    }
    ASSERT_EQ(outside.size(), 7U);
    for (const int d : search.device_of_model) {
      slot_map.push_back(JointSetSlot(chain, d));
    }
    handle = std::make_unique<rtc_urdf_bridge::RtModelHandle>(search.rig.arm.model);
    borrowed = LoadBorrowed();
  }

  [[nodiscard]] int SearchNv() const { return static_cast<int>(search.rig.arm.model->nv); }

  [[nodiscard]] int SegmentNv() const { return static_cast<int>(segment.rig.arm.model->nv); }

  // The grid search's and the cycle's parameters: the borrowed ones, on this
  // device's wait pose.
  [[nodiscard]] PlannerParams CycleParams(Budget budget) const {
    PlannerParams p = borrowed.planner;
    p.enabled = true;
    p.wait_pose.fill(0.0);
    std::copy(command.begin(), command.end(), p.wait_pose.begin());
    p.wait_pose_n = fx::kG1Dof;
    // One weight per joint of THIS device: the borrowed profile's uniform value.
    const double weight = p.mpc_segment.jerk_weight.empty() ? 1.0 : p.mpc_segment.jerk_weight[0];
    p.mpc_segment.jerk_weight.assign(static_cast<std::size_t>(fx::kG1Dof), weight);
    if (budget == Budget::kGenerous) {
      p.mpc_segment.budget_first_s = kGenerousSolveS;
      p.mpc_segment.budget_replan_s = kGenerousSolveS;
    }
    return p;
  }

  // The arm at rest on the command, following no plan.
  [[nodiscard]] PlannerRtState RestingRt(std::int64_t now, std::uint64_t iteration = 500) const {
    PlannerRtState rt = rtc::testing::TrackingRtState(kActivation, command);
    rt.rt_iteration = iteration;
    rt.rt_state_ns = now - kMs;
    rt.reset_epoch = 1;
    rt.track_seen = true;
    rt.track_generation = kTrack;
    return rt;
  }

  // The earliest instant the nlp search lets a segment of this wake start.
  [[nodiscard]] std::int64_t SearchStartNs(std::int64_t now, Budget budget) const {
    const double budget_s = budget == Budget::kGenerous ? kGenerousSearchS : borrowed.nlp.budget_s;
    return now + static_cast<std::int64_t>(
                     std::llround((borrowed.t_arm_s + budget_s + borrowed.nlp.start_lead_s) * 1e9));
  }

  // The aim posture, search-model order.
  [[nodiscard]] Eigen::VectorXd AimPosture() const {
    Eigen::VectorXd q(SearchNv());
    for (int m = 0; m < SearchNv(); ++m) {
      const auto d = static_cast<std::size_t>(search.device_of_model[static_cast<std::size_t>(m)]);
      q[m] = command[d] + kAimOffset[d];
    }
    return q;
  }

  // The ball down the aim posture's capture axis, on its entrance plane at
  // `star_ns`; kCap samples `spacing_ns` apart from `first_ns`, as snapshot
  // `seq` of the one track.
  [[nodiscard]] std::unique_ptr<Throw> AimedThrow(std::int64_t first_ns, std::int64_t star_ns,
                                                  std::int64_t spacing_ns,
                                                  std::uint64_t seq = 1) const {
    const dk::HandState h = dk::HandAt(search.rig, AimPosture(), Eigen::VectorXd::Zero(SearchNv()));
    const Eigen::Vector3d v_b = -kBallSpeed * h.R.col(2);
    const Eigen::Vector3d p_e = h.p + h.R * Eigen::Vector3d(0.0, 0.0, search.rig.params.s_ent);
    const double t_star_s = static_cast<double>(star_ns - first_ns) / 1e9;
    auto t = std::make_unique<Throw>();
    t->traj = rtc::testing::LineTrajectory(p_e - v_b * t_star_s, v_b, first_ns, spacing_ns,
                                           rtc::catching::kCap, 0, seq, kTrack, kActivation,
                                           first_ns - 5 * kMs);
    t->cov = rtc::testing::IsotropicCovariance(t->traj, kBallSigma);
    return t;
  }

  // How far the aim posture's capture axis passes from the wait pose's catch
  // frame [m]: the least the catch frame has to travel to be on the ball's line.
  [[nodiscard]] double AimLineOffsetM() const {
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(SearchNv());
    const dk::HandState aim = dk::HandAt(search.rig, AimPosture(), zero);
    const dk::HandState home = dk::HandAt(search.rig, search.rig.arm.q_nominal, zero);
    const Eigen::Vector3d axis = aim.R.col(2);
    const Eigen::Vector3d r = home.p - aim.p;
    return (r - axis * axis.dot(r)).norm();
  }
};

[[nodiscard]] BallPrediction BallOf(const Throw& t) {
  return BallPrediction{&t.traj, &t.cov, true};
}

const ReportedSegments& NoSegments() {
  static const ReportedSegments none{};
  return none;
}

// ── The four objects, each on its model ──────────────────────────────────────

std::unique_ptr<GridCatchSearch> MakeGrid(const Scene& s, Budget budget) {
  const fx::ShippedArm& arm = s.search;
  GridCatchSearchModel pm;
  pm.handle = s.handle.get();
  pm.catch_frame = arm.rig.arm.frame;
  pm.nv = s.SearchNv();
  pm.n_dev = arm.n_dev;
  for (int m = 0; m < pm.nv; ++m) {
    const auto u = static_cast<std::size_t>(m);
    pm.device_of_model[u] = arm.device_of_model[u];
    pm.qdot_max[u] = arm.rig.limits.qd_max[m];
    // The D-16 box. G1 declares none (`robot.arm.qdd_max` is a catching key);
    // this is the value rtc_controllers' cycle suite gives its arm.
    pm.qddot_max[u] = 20.0;
  }
  pm.accel_box = true;
  return rtc::catching::MakeGridCatchSearch(pm, s.borrowed.grid_consts, s.CycleParams(budget),
                                            s.borrowed.grid_ik, &SuiteClock);
}

// Whether the docking cores' cost weighs torque (`r_tau`; the fixture's
// BaseParams weighs every joint of the model it is sized for). The torque
// ROWS are another switch (`torque_rows`).
enum class TorqueCost {
  kEveryJoint,  // the fixture's: every joint of each model
  kNone,        // no torque cost — the agreement's premise
};

void ApplyTorqueCost(TorqueCost mode, rtc::catching::MpcDockingSegmentCoreParams& core) {
  if (mode == TorqueCost::kNone) {
    core.r_tau.resize(0);  // empty = off
  }
}

std::unique_ptr<NlpCatchSearch> MakeNlp(const Scene& s, Budget budget, std::string* error,
                                        TorqueCost torque_cost = TorqueCost::kEveryJoint) {
  const fx::ShippedArm& arm = s.search;
  NlpCatchSearchModel model;
  model.arm = arm.rig.arm.model;
  model.handle = s.handle.get();
  model.catch_frame = arm.rig.arm.frame;
  model.nv = s.SearchNv();
  model.n_dev = arm.n_dev;
  for (int m = 0; m < model.nv; ++m) {
    const auto u = static_cast<std::size_t>(m);
    model.device_of_model[u] = arm.device_of_model[u];
  }
  NlpCatchSearchConstants constants;
  constants.t_arm_s = s.borrowed.t_arm_s;
  constants.control_dt = kControlDt;
  NlpCatchSearchParams params = s.borrowed.nlp;
  params.wait_pose.fill(0.0);
  std::copy(s.command.begin(), s.command.end(), params.wait_pose.begin());
  params.wait_pose_n = arm.n_dev;
  // ONE candidate a wake: what the borrowed profile ships, and what the times
  // below are of.
  params.max_solves = 1;
  if (budget == Budget::kGenerous) {
    params.budget_s = kGenerousSearchS;
    params.solve_budget_s = kGenerousSolveS;
  }
  params.core = arm.rig.params;  // BaseParams(10), q_nom = the wait pose
  // The G1 decision: the search judges kinematics only.
  params.core.torque_rows = false;
  ApplyTorqueCost(torque_cost, params.core);
  params.limits = arm.rig.limits;
  auto search = std::make_unique<NlpCatchSearch>();
  if (!search->Configure(model, constants, params, s.borrowed.nlp_ik, &SuiteClock, error)) {
    return nullptr;
  }
  return search;
}

// The mpc planner on `arm`, whose model joint m is device joint
// `device_of_model[m]` of the device the RT reports to THIS planner.
std::unique_ptr<MpcSegmentPlanner> MakeMpcOn(const Scene& s, const fx::ShippedArm& arm,
                                             std::span<const int> device_of_model, Budget budget,
                                             std::string* error) {
  MpcSegmentPlannerModel model;
  model.arm = arm.rig.arm.model;
  model.catch_frame = arm.rig.arm.frame;
  model.nv = static_cast<int>(arm.rig.arm.model->nv);
  for (int m = 0; m < model.nv; ++m) {
    const auto u = static_cast<std::size_t>(m);
    model.device_of_model[u] = device_of_model[u];
    model.q_min[u] = arm.rig.limits.q_min[m];
    model.q_max[u] = arm.rig.limits.q_max[m];
    model.qdot_max[u] = arm.rig.limits.qd_max[m];
    model.tau_max[u] = arm.rig.limits.tau_max[m];
  }
  MpcSegmentPlannerParams params = s.CycleParams(budget).mpc_segment;
  params.jerk_weight.resize(static_cast<std::size_t>(model.nv));
  if (&arm == &s.segment) {
    model.groups = s.DeclaredGroups();
    params.n_groups = model.groups.n_groups;
    if (s.groups_declared) {
      params.groups[1].w_rest = s.wb_rest_left;
      params.ang_w_pre = s.wb_ang_pre;
      params.ang_w_post = s.wb_ang_post;
      params.ang_scale = 10.0;
      std::copy(s.command.begin(), s.command.end(), model.rest_pose.begin());
      model.rest_pose_n = fx::kG1Dof;
      if (s.wb_coarse_left) {
        params.groups[1].pre_block = 2;
        params.groups[1].stop_blocks = {2, 2, 3};
        params.groups[1].n_stop_blocks = 3;
      }
    }
  }
  MpcSegmentPlannerConstants consts;
  consts.eta_v = s.borrowed.grid_consts.eta_v;
  consts.t_arm_s = s.borrowed.t_arm_s;
  consts.control_dt = kControlDt;
  consts.v_eps = params.v_eps;
  return rtc::catching::MakeMpcSegmentPlanner(model, consts, params, &SuiteClock, error);
}

std::unique_ptr<MpcSegmentPlanner> MakeMpc(const Scene& s, Budget budget, std::string* error) {
  return MakeMpcOn(s, s.segment, s.segment.device_of_model, budget, error);
}

std::unique_ptr<MpcDockingSegmentPlanner> MakeDocking(
    const Scene& s, Budget budget, std::string* error,
    TorqueCost torque_cost = TorqueCost::kEveryJoint) {
  const fx::ShippedArm& arm = s.segment;
  MpcDockingSegmentPlannerModel model;
  model.arm = arm.rig.arm.model;
  model.catch_frame = arm.rig.arm.frame;
  model.nv = s.SegmentNv();
  for (int m = 0; m < model.nv; ++m) {
    const auto u = static_cast<std::size_t>(m);
    model.device_of_model[u] = arm.device_of_model[u];
    model.qd_rating[u] = arm.rig.limits.qd_max[m];
  }
  model.limits = arm.rig.limits;
  std::copy(s.command.begin(), s.command.end(), model.warm_pose.begin());
  MpcDockingSegmentPlannerConstants consts;
  consts.t_arm_s = s.borrowed.t_arm_s;
  consts.control_dt = kControlDt;
  MpcDockingSegmentPlannerParams params = s.borrowed.docking;
  if (budget == Budget::kGenerous) {
    params.budget_first_s = kGenerousSolveS;
  }
  // BaseParams(17), q_nom = the command; a segment keeps its torque rows.
  params.core = arm.rig.params;
  model.groups = s.DeclaredGroups();
  params.n_groups = model.groups.n_groups;
  if (s.groups_declared) {
    params.groups[1].w_rest_stop = s.wb_rest_left;
    params.core.w_ang_pre = s.wb_ang_pre;
    params.core.w_ang_post = s.wb_ang_post;
    params.core.ang_scale = 10.0;
    if (s.wb_coarse_left) {
      params.groups[1].pre_block = 2;
    }
  }
  ApplyTorqueCost(torque_cost, params.core);
  return rtc::catching::MakeMpcDockingSegmentPlanner(model, consts, params, &SuiteClock, error);
}

// ── Reading segments ─────────────────────────────────────────────────────────

[[nodiscard]] std::size_t Entry(int k, int column) {
  return static_cast<std::size_t>(k * kMaxSegmentNv + column);
}

// The catch frame of `arm` at node `k` of `seg`, whose column of model joint m
// is `column_of_model[m]`.
[[nodiscard]] Eigen::Vector3d FramePointAt(const fx::ShippedArm& arm, const SegmentSnapshot& seg,
                                           std::span<const int> column_of_model, int k) {
  const pinocchio::Model& model = *arm.rig.arm.model;
  Eigen::VectorXd q(model.nv);
  for (int m = 0; m < model.nv; ++m) {
    q[m] = seg.q[Entry(k, column_of_model[static_cast<std::size_t>(m)])];
  }
  pinocchio::Data data(model);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacement(model, data, arm.rig.arm.frame);
  return data.oMf[arm.rig.arm.frame].translation();
}

// How far the chain's joints of a segment of the DEVICE (`wide`, 17 columns in
// device order) are from a solution of the CHAIN (`narrow`: the column of
// device joint d is its rank in the chain's set).
struct ChainGap {
  bool comparable{false};            // same grid, same node count
  double q{0.0}, qd{0.0}, qdd{0.0};  // largest over nodes and chain joints
  int q_node{-1}, q_device{-1};      // where the largest |Δq| is
  double q_to_catch{0.0};            // the largest |Δq| up to the catch node
  double catch_m{0.0};               // catch frame at the catch node [m]
  double outside_q{0.0};             // how far `wide` moves a joint the chain lacks [rad]
  double reach{0.0};  // largest chain joint travel of `narrow`, node 0 → catch node
};

[[nodiscard]] ChainGap GapOf(const Scene& s, const SegmentSnapshot& wide,
                             const SegmentSnapshot& narrow) {
  ChainGap g;
  g.comparable = wide.nv == s.SegmentNv() && narrow.nv == s.SearchNv() &&
                 wide.n_nodes == narrow.n_nodes && wide.n_pre == narrow.n_pre &&
                 wide.t0_ns == narrow.t0_ns && wide.t_c_ns == narrow.t_c_ns &&
                 wide.dt_ns == narrow.dt_ns && wide.dt_pre_ns == narrow.dt_pre_ns &&
                 wide.dt_catch_ns == narrow.dt_catch_ns;
  if (!g.comparable) {
    return g;
  }
  for (int k = 0; k <= wide.n_nodes; ++k) {
    for (int d = 0; d < s.SegmentNv(); ++d) {
      if (!InJointSet(s.chain, d)) {
        g.outside_q = std::max(g.outside_q, std::fabs(wide.q[Entry(k, d)] - wide.q[Entry(0, d)]));
        continue;
      }
      const int slot = JointSetSlot(s.chain, d);
      const double dq = std::fabs(wide.q[Entry(k, d)] - narrow.q[Entry(k, slot)]);
      if (dq > g.q) {
        g.q = dq;
        g.q_node = k;
        g.q_device = d;
      }
      if (k <= narrow.n_pre) {
        g.q_to_catch = std::max(g.q_to_catch, dq);
      }
      g.qd = std::max(g.qd, std::fabs(wide.qd[Entry(k, d)] - narrow.qd[Entry(k, slot)]));
      g.qdd = std::max(g.qdd, std::fabs(wide.qdd[Entry(k, d)] - narrow.qdd[Entry(k, slot)]));
      if (k == narrow.n_pre) {
        g.reach = std::max(g.reach, std::fabs(narrow.q[Entry(k, slot)] - narrow.q[Entry(0, slot)]));
      }
    }
  }
  g.catch_m = (FramePointAt(s.segment, wide, s.segment.device_of_model, wide.n_pre) -
               FramePointAt(s.search, narrow, s.slot_map, narrow.n_pre))
                  .norm();
  return g;
}

void RecordSci(const std::string& name, double value) {
  // RecordProperty(double) goes through to_string (6 fixed decimals).
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3e", value);
  ::testing::Test::RecordProperty(name, buf);
}

void RecordGap(const std::string& tag, const ChainGap& g) {
  RecordSci(tag + "_max_abs_dq_rad", g.q);
  RecordSci(tag + "_max_abs_dq_up_to_the_catch_node_rad", g.q_to_catch);
  RecordSci(tag + "_max_abs_dqd_rad_s", g.qd);
  RecordSci(tag + "_max_abs_dqdd_rad_s2", g.qdd);
  ::testing::Test::RecordProperty(tag + "_max_abs_dq_node", g.q_node);
  ::testing::Test::RecordProperty(tag + "_max_abs_dq_device_joint", g.q_device);
  RecordSci(tag + "_catch_frame_m", g.catch_m);
  RecordSci(tag + "_outside_joint_travel_rad", g.outside_q);
  RecordSci(tag + "_chain_reach_rad", g.reach);
}

[[nodiscard]] std::string Describe(const SegmentRecord& r) {
  std::ostringstream os;
  os << "\noutcome " << SegmentOutcomeName(r.outcome) << ", core " << r.core_reason_name << ", k "
     << r.k << ", nodes " << r.n_nodes << ", from_search " << r.from_search
     << ", start_from_search " << r.start_from_search << ", iterations " << r.iterations
     << ", solve " << r.solve_ns << " ns, slack " << r.slack_max << ", tau ratio "
     << r.tau_ratio_max << ", speed ratio " << r.speed_ratio_max << ", catch err "
     << r.catch_pos_err << ", infeasible group " << r.docking.infeasible_group_name;
  return os.str();
}

// The grid search's judgement rejects of a wake (search_stats.hpp: JudgeReject).
[[nodiscard]] std::string Describe(const SearchStats& st) {
  static constexpr std::array<const char*, rtc::catching::kJudgeRejectCount> kNames = {
      "passed", "input", "ik", "manipulability", "not_evaluated", "too_far"};
  std::ostringstream os;
  os << "\nsearch: nlp reason " << NlpRejectName(st.nlp.reason) << ", candidates";
  for (std::size_t i = 0; i < kNames.size(); ++i) {
    os << " " << kNames[i] << " " << st.judge_rejects[i];
  }
  return os.str();
}

[[nodiscard]] std::string Describe(const PlannerCycleRecord& r) {
  std::ostringstream os;
  os << "cycle " << CycleOutcomeName(r.outcome) << ", search valid " << r.search_valid
     << ", plan reason " << static_cast<int>(r.reason) << Describe(r.search) << Describe(r.segment);
  return os.str();
}

// ── (3) The four combinations: a pair in one wake ────────────────────────────

enum class SearchKind { kGrid, kNlp };
enum class SegmentKindOf { kMpc, kMpcDocking };

struct Boxes {
  rtc::SeqLock<TrajectorySnapshot> traj;
  rtc::SeqLock<CovarianceSnapshot> cov;
  rtc::SeqLock<PlannerRtState> rt;
  rtc::SeqLock<PlanSnapshot> plan;
  rtc::SeqLock<SegmentSnapshot> segment;
};

struct Wiring {
  Boxes boxes;
  PlannerCycle cycle;
  // Owned by the cycle once installed; kept typed, as a controller's configure
  // keeps them, to read what the cores were handed.
  const NlpCatchSearch* nlp{nullptr};
  const MpcSegmentPlanner* mpc{nullptr};
  const MpcDockingSegmentPlanner* docking{nullptr};
};

void ExpectAPairInOneWake(SearchKind search_kind, SegmentKindOf segment_kind) {
  PinnedClock pin(kPinnedNow);
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Build(fx::kG1Home));
  // The scene shows something only if the ball is not where the hand waits.
  ASSERT_GT(s->AimLineOffsetM(), 0.05);

  auto w = std::make_unique<Wiring>();
  w->cycle.Configure(s->CycleParams(Budget::kShipped));
  w->cycle.SetClock(&SuiteClock);
  ASSERT_TRUE(w->cycle.Bind(PlannerCycleIo{&w->boxes.traj, &w->boxes.cov, &w->boxes.rt,
                                           &w->boxes.plan, &w->boxes.segment}));
  std::string err;
  if (search_kind == SearchKind::kGrid) {
    auto search = MakeGrid(*s, Budget::kShipped);
    ASSERT_NE(search, nullptr);
    w->cycle.InstallSearch(std::move(search));
  } else {
    auto search = MakeNlp(*s, Budget::kShipped, &err);
    ASSERT_NE(search, nullptr) << err;
    w->nlp = search.get();
    w->cycle.InstallSearch(std::move(search));
  }
  if (segment_kind == SegmentKindOf::kMpc) {
    auto planner = MakeMpc(*s, Budget::kShipped, &err);
    ASSERT_NE(planner, nullptr) << err;
    w->mpc = planner.get();
    w->cycle.InstallSegmentPlanner(std::move(planner));
  } else {
    auto planner = MakeDocking(*s, Budget::kShipped, &err);
    ASSERT_NE(planner, nullptr) << err;
    w->docking = planner.get();
    w->cycle.InstallSegmentPlanner(std::move(planner));
  }

  // The RT stand-in: the arm at rest on the wait pose, 17 joints reported. The
  // ball is one throw, and each wake reads a newer snapshot of it. The grid
  // search plans nothing until it has seen `n_settle` snapshots of a track
  // (the borrowed profile's value): those wakes publish "no plan" and no
  // segment. The nlp search has no such rule and plans on the first.
  const int settle_wakes = search_kind == SearchKind::kGrid ? s->borrowed.planner.n_settle : 0;
  const std::int64_t star = s->SearchStartNs(kPinnedNow, Budget::kShipped) +
                            static_cast<std::int64_t>(std::llround(kStarAfterT0S * 1e9));
  PlannerRtState rt{};
  auto rec = std::make_unique<PlannerCycleRecord>();
  for (int wake = 0; wake <= settle_wakes; ++wake) {
    SCOPED_TRACE("wake " + std::to_string(wake + 1));
    const std::int64_t now = kPinnedNow + wake * kWakePeriodNs;
    pin.Set(now);
    const std::unique_ptr<Throw> ball =
        s->AimedThrow(now, star, 50 * kMs, static_cast<std::uint64_t>(wake + 1));
    w->boxes.cov.Store(ball->cov);
    w->boxes.traj.Store(ball->traj);
    rt = s->RestingRt(now, 500 + static_cast<std::uint64_t>(wake));
    ASSERT_EQ(rt.nv, fx::kG1Dof);
    w->boxes.rt.Store(rt);
    *rec = w->cycle.Run(NowReal{now});
    if (wake < settle_wakes) {
      ASSERT_EQ(rec->outcome, CycleOutcome::kPublished) << Describe(*rec);
      ASSERT_FALSE(rec->plan_valid) << Describe(*rec);
      EXPECT_TRUE(rec->search.settling);
      EXPECT_EQ(rec->segment.outcome, SegmentOutcome::kOff);
      EXPECT_FALSE(w->boxes.segment.Load().valid);
    }
  }

  // ── The wake the search plans on: ONE wake, a pair ──
  ASSERT_EQ(rec->outcome, CycleOutcome::kPublished) << Describe(*rec);
  ASSERT_TRUE(rec->plan_valid) << Describe(*rec);
  EXPECT_TRUE(rec->search_valid);
  ASSERT_EQ(rec->segment.outcome, SegmentOutcome::kPublished) << Describe(*rec);
  EXPECT_EQ(rec->segment.kind, SegmentKind::kFirst);
  ::testing::Test::RecordProperty("wakes_until_the_pair", settle_wakes + 1);
  ::testing::Test::RecordProperty("search_rank_gate_mask", rec->search.chosen_rank_mask);

  // ── The plan: of the chain's joints ──
  const PlanSnapshot plan = w->boxes.plan.Load();
  ASSERT_TRUE(plan.valid);
  EXPECT_EQ(plan.plan_id, rec->plan_id);
  EXPECT_EQ(plan.nv, fx::kG1CatchDof);
  EXPECT_EQ(plan.joint_set, s->chain);
  double reach = 0.0;
  for (std::size_t d = 0; d < plan.q_star.size(); ++d) {
    if (InJointSet(s->chain, static_cast<int>(d))) {
      reach = std::max(reach, std::fabs(plan.q_star[d] - s->command[d]));
    } else {
      EXPECT_EQ(plan.q_star[d], 0.0) << "q_star entry " << d << " is of no joint of the plan";
    }
  }
  EXPECT_GT(reach, kMinReachRad) << "the catch pose is the wait pose: the wake moved nothing";
  RecordSci("plan_reach_rad", reach);

  // ── The segment: of the device ──
  const auto seg = std::make_unique<SegmentSnapshot>(w->boxes.segment.Load());
  ASSERT_TRUE(seg->valid);
  EXPECT_EQ(seg->nv, fx::kG1Dof);
  EXPECT_EQ(seg->plan_id, plan.plan_id);
  EXPECT_EQ(seg->t_c_ns, plan.t_c_ns);
  EXPECT_EQ(seg->segment_seq, rec->segment.segment_seq);
  ASSERT_TRUE(ValidateSegmentNodes(*seg));
  const int n_pre = seg->n_pre;
  ASSERT_GE(n_pre, 1);
  EXPECT_EQ(rec->segment.k, -n_pre);
  EXPECT_EQ(rec->segment.n_nodes, seg->n_nodes);
  ::testing::Test::RecordProperty("segment_n_pre", n_pre);
  ::testing::Test::RecordProperty("segment_n_nodes", seg->n_nodes);
  ::testing::Test::RecordProperty("segment_iterations", rec->segment.iterations);
  RecordSci("segment_tau_ratio_max", rec->segment.tau_ratio_max);

  // The RT's sampler reads it: the joint evaluation at every node instant is
  // that node, and the follower on the device's model gives a pose there.
  NodeTrajectoryFollower follower;
  ASSERT_TRUE(follower.Init(s->segment.rig.arm.model, s->segment.rig.arm.frame,
                            s->segment.device_of_model));
  for (int k = 0; k <= seg->n_nodes; ++k) {
    SCOPED_TRACE("node " + std::to_string(k));
    const std::int64_t t = SegmentNodeTimeNs(*seg, k);
    std::array<double, kMaxSegmentNv> q{};
    std::array<double, kMaxSegmentNv> qd{};
    std::array<double, kMaxSegmentNv> qdd{};
    ASSERT_TRUE(NodeTrajectoryFollower::SampleJoints(*seg, t, q, qd, qdd));
    for (int d = 0; d < fx::kG1Dof; ++d) {
      const auto u = static_cast<std::size_t>(d);
      EXPECT_NEAR(q[u], seg->q[Entry(k, d)], 1e-12) << "joint " << d;
      EXPECT_NEAR(qd[u], seg->qd[Entry(k, d)], 1e-9) << "joint " << d;
    }
    auto sample = std::make_unique<SegmentNodeSample>();
    ASSERT_TRUE(follower.Sample(*seg, t, *sample));
    EXPECT_TRUE(sample->placement.translation().allFinite());
    EXPECT_TRUE(sample->twist.allFinite());
  }

  // ── The joints the chain lacks: node 0 is the command, at rest ──
  double outside_travel = 0.0;
  for (const int d : s->outside) {
    SCOPED_TRACE("device joint " + std::to_string(d));
    const auto u = static_cast<std::size_t>(d);
    EXPECT_TRUE(BitsEqual(seg->q[Entry(0, d)], rt.q_cmd[u]))
        << seg->q[Entry(0, d)] << " vs " << rt.q_cmd[u];
    EXPECT_EQ(seg->qd[Entry(0, d)], 0.0);
    EXPECT_EQ(seg->qdd[Entry(0, d)], 0.0);
    for (int k = 0; k <= seg->n_nodes; ++k) {
      outside_travel = std::max(outside_travel, std::fabs(seg->q[Entry(k, d)] - rt.q_cmd[u]));
    }
  }
  RecordSci("outside_joint_travel_rad", outside_travel);

  // ── ... and what the planner referenced them to is that command ──
  const int n_model = s->SegmentNv();
  if (segment_kind == SegmentKindOf::kMpc) {
    EXPECT_FALSE(rec->segment.from_search);
    EXPECT_FALSE(rec->segment.start_from_search);
    const MpcSegmentCoreInput& in = w->mpc->ApproachCoreInput(n_pre);
    ASSERT_EQ(in.q_ref.rows(), n_model);
    ASSERT_EQ(in.q_ref.cols(), seg->n_nodes + 1);
    for (int m = 0; m < n_model; ++m) {
      const int d = s->segment.device_of_model[static_cast<std::size_t>(m)];
      if (InJointSet(s->chain, d)) {
        continue;
      }
      SCOPED_TRACE("device joint " + std::to_string(d));
      EXPECT_TRUE(BitsEqual(in.q0[m], rt.q_cmd[static_cast<std::size_t>(d)]));
      for (Eigen::Index k = 0; k < in.q_ref.cols(); ++k) {
        EXPECT_TRUE(BitsEqual(in.q_ref(m, k), rt.q_cmd[static_cast<std::size_t>(d)])) << k;
        EXPECT_EQ(in.qd_ref(m, k), 0.0) << k;
        EXPECT_EQ(in.qdd_ref(m, k), 0.0) << k;
      }
    }
  } else {
    // A chain's solution is never published as it is: the segment is this
    // planner's own solve.
    EXPECT_FALSE(rec->segment.from_search);
    // The nlp search hands its solution in and the solve starts from it,
    // widened; the grid search solves none.
    EXPECT_EQ(rec->segment.start_from_search, search_kind == SearchKind::kNlp);
    EXPECT_FALSE(rec->segment.start_from_memory);
    const MpcDockingSegmentCoreInput* in = w->docking->LastInputForTesting(n_pre);
    ASSERT_NE(in, nullptr);
    EXPECT_TRUE(in->catch_target_valid);
    EXPECT_EQ(in->initial_valid, search_kind == SearchKind::kNlp);
    for (int m = 0; m < n_model; ++m) {
      const int d = s->segment.device_of_model[static_cast<std::size_t>(m)];
      const auto u = static_cast<std::size_t>(d);
      SCOPED_TRACE("device joint " + std::to_string(d));
      EXPECT_TRUE(BitsEqual(in->q0[m], rt.q_cmd[u]));
      if (InJointSet(s->chain, d)) {
        // The chain's joints aim at the plan's catch pose.
        EXPECT_TRUE(BitsEqual(in->q_catch_target[m], plan.q_star[u]));
        continue;
      }
      EXPECT_TRUE(BitsEqual(in->q_catch_target[m], in->q0[m]));
      if (in->initial_valid) {
        for (Eigen::Index k = 0; k < in->q_init.cols(); ++k) {
          EXPECT_TRUE(BitsEqual(in->q_init(m, k), in->q0[m])) << k;
          EXPECT_EQ(in->qd_init(m, k), 0.0) << k;
          EXPECT_EQ(in->qdd_init(m, k), 0.0) << k;
        }
      }
    }
    if (search_kind == SearchKind::kNlp) {
      // The solve ran on the search's own grid.
      const CatchSolution* solution = w->nlp->Solution();
      ASSERT_NE(solution, nullptr);
      EXPECT_EQ(solution->joint_set, s->chain);
      EXPECT_EQ(solution->seg.nv, fx::kG1CatchDof);
      EXPECT_EQ(solution->seg.n_pre, n_pre);
      EXPECT_EQ(solution->seg.t0_ns, seg->t0_ns);
    }
  }
}

TEST(G1SearchSegmentPair, GridAndMpcPublishADeviceSegmentInOneWake) {
  RTC_SKIP_IF_PACKAGE_MISSING("hand_description");
  ExpectAPairInOneWake(SearchKind::kGrid, SegmentKindOf::kMpc);
}

TEST(G1SearchSegmentPair, GridAndMpcDockingPublishADeviceSegmentInOneWake) {
  RTC_SKIP_IF_PACKAGE_MISSING("hand_description");
  ExpectAPairInOneWake(SearchKind::kGrid, SegmentKindOf::kMpcDocking);
}

TEST(G1SearchSegmentPair, NlpAndMpcPublishADeviceSegmentInOneWake) {
  RTC_SKIP_IF_PACKAGE_MISSING("hand_description");
  ExpectAPairInOneWake(SearchKind::kNlp, SegmentKindOf::kMpc);
}

TEST(G1SearchSegmentPair, NlpAndMpcDockingPublishADeviceSegmentInOneWake) {
  RTC_SKIP_IF_PACKAGE_MISSING("hand_description");
  ExpectAPairInOneWake(SearchKind::kNlp, SegmentKindOf::kMpcDocking);
}

// ── (4) The agreement ────────────────────────────────────────────────────────

// The pose the chain model has the joints it lacks locked at: the builder's
// reference configuration (PinocchioModelBuilder::MakeReferenceConfig — the
// model's neutral pose, and the fixture's ModelConfig sets no
// lock_reference_config), which for these revolute joints is zero.
// ChainTorqueGap below checks it against the two models rather than trusting
// that reading.
[[nodiscard]] DevicePose LockedOutsidePose() {
  constexpr int kLeftArmFirst = 3;  // the device is (waist 3, left arm 7, right arm 7)
  constexpr int kLeftArmJoints = 7;
  DevicePose pose = fx::kG1Home;
  for (int d = kLeftArmFirst; d < kLeftArmFirst + kLeftArmJoints; ++d) {
    pose[static_cast<std::size_t>(d)] = 0.0;
  }
  return pose;
}

// The largest difference between the chain model's inverse dynamics and the
// tree model's, on the chain's joints [N·m], with the tree's other joints at
// rest on `pose` and the chain's joints in a state that exercises M, C and g.
[[nodiscard]] double ChainTorqueGap(const Scene& s, const DevicePose& pose) {
  const pinocchio::Model& chain = *s.search.rig.arm.model;
  const pinocchio::Model& tree = *s.segment.rig.arm.model;
  Eigen::VectorXd q_t = Eigen::VectorXd::Zero(tree.nv);
  Eigen::VectorXd v_t = Eigen::VectorXd::Zero(tree.nv);
  Eigen::VectorXd a_t = Eigen::VectorXd::Zero(tree.nv);
  Eigen::VectorXd q_c(chain.nv);
  Eigen::VectorXd v_c(chain.nv);
  Eigen::VectorXd a_c(chain.nv);
  std::vector<int> tree_of_device(static_cast<std::size_t>(fx::kG1Dof), -1);
  for (int m = 0; m < tree.nv; ++m) {
    const auto d = static_cast<std::size_t>(s.segment.device_of_model[static_cast<std::size_t>(m)]);
    tree_of_device[d] = m;
    q_t[m] = pose[d];
  }
  for (int m = 0; m < chain.nv; ++m) {
    const auto d = static_cast<std::size_t>(s.search.device_of_model[static_cast<std::size_t>(m)]);
    q_c[m] = pose[d] + kAimOffset[d];
    v_c[m] = 0.7 * std::sin(1.3 * m + 0.2);
    a_c[m] = 4.0 * std::cos(0.9 * m + 0.5);
    const int t = tree_of_device[d];
    q_t[t] = q_c[m];
    v_t[t] = v_c[m];
    a_t[t] = a_c[m];
  }
  pinocchio::Data data_c(chain);
  pinocchio::Data data_t(tree);
  const Eigen::VectorXd tau_c = pinocchio::rnea(chain, data_c, q_c, v_c, a_c);
  const Eigen::VectorXd tau_t = pinocchio::rnea(tree, data_t, q_t, v_t, a_t);
  double gap = 0.0;
  for (int m = 0; m < chain.nv; ++m) {
    const auto d = static_cast<std::size_t>(s.search.device_of_model[static_cast<std::size_t>(m)]);
    gap = std::max(gap, std::fabs(tau_c[m] - tau_t[tree_of_device[d]]));
  }
  return gap;
}

// The scene of the agreement, checked: the joints the chain lacks rest where
// the chain model locks them, so that the two models are one mechanism for as
// long as those joints stand still.
void BuildAgreementScene(Scene& s) {
  ASSERT_NO_FATAL_FAILURE(s.Build(LockedOutsidePose()));
  ASSERT_GT(s.AimLineOffsetM(), 0.05);
  // What that pose changes of the wait pose is exactly the joints the chain
  // lacks: the chain starts where every other test here starts it.
  for (int d = 0; d < fx::kG1Dof; ++d) {
    const auto u = static_cast<std::size_t>(d);
    if (InJointSet(s.chain, d)) {
      ASSERT_EQ(s.command[u], fx::kG1Home[u]) << d;
    } else {
      ASSERT_EQ(s.command[u], 0.0) << d;
    }
  }
  const double locked = ChainTorqueGap(s, LockedOutsidePose());
  const double elsewhere = ChainTorqueGap(s, fx::kG1Home);
  RecordSci("chain_torque_gap_at_the_locked_pose_Nm", locked);
  RecordSci("chain_torque_gap_at_the_wait_pose_Nm", elsewhere);
  ASSERT_LT(locked, 1e-9) << "the chain model does not lock its other joints at this pose";
  // The check has teeth: with the left arm elsewhere the chain's joints carry
  // other torques.
  ASSERT_GT(elsewhere, 1e-3);
  // The cores' nominal posture is the start, for every joint: the posture
  // cost of a joint that stands still is zero in both problems.
  for (int m = 0; m < s.SegmentNv(); ++m) {
    const auto d = static_cast<std::size_t>(s.segment.device_of_model[static_cast<std::size_t>(m)]);
    ASSERT_EQ(s.segment.rig.params.q_nom[m], s.command[d]);
  }
}

void ExpectWithinD10(const ChainGap& gap, const char* what) {
  ASSERT_TRUE(gap.comparable) << what << ": the two solves are not on one grid";
  EXPECT_LE(gap.q, kAgreeJointRad)
      << what << ": device joint " << gap.q_device << " at node " << gap.q_node;
  EXPECT_LE(gap.catch_m, kAgreeCatchM) << what;
}

// mpc_docking at the two widths, with no torque cost in either: the nlp
// search's solve of its one candidate on the chain (10 joints, no torque rows),
// and the planner's solves of that plan on the tree (17 joints, torque rows
// on) — started from the chain's solution widened, and from the plan alone.
struct DockingAtBothWidths {
  int chain_iterations{-1};    // the search's solve
  SegmentRecord from_chain{};  // the tree's solve started from the widened chain solution
  bool from_chain_converged{false};
  ChainGap from_chain_gap{};
  SegmentRecord from_plan{};  // the tree's solve handed no solution
  ChainGap from_plan_gap{};
};

// gtest ASSERTs (the preconditions): call inside ASSERT_NO_FATAL_FAILURE.
void SolveDockingAtBothWidths(const Scene& s, DockingAtBothWidths& out) {
  const TorqueCost torque_cost = TorqueCost::kNone;
  const std::string tag = "no_torque_cost";
  // ── The 10-joint solve: the search's one candidate ──
  std::string err;
  const std::unique_ptr<NlpCatchSearch> search = MakeNlp(s, Budget::kShipped, &err, torque_cost);
  ASSERT_NE(search, nullptr) << err;
  const PlannerRtState rt = s.RestingRt(kPinnedNow);
  const std::int64_t star = s.SearchStartNs(kPinnedNow, Budget::kShipped) +
                            static_cast<std::int64_t>(std::llround(kStarAfterT0S * 1e9));
  const std::unique_ptr<Throw> ball = s.AimedThrow(kPinnedNow, star, 50 * kMs);
  auto stats = std::make_unique<SearchStats>();
  PlanSnapshot plan =
      search->Plan(ball->traj, ball->cov, true, rt, NoSegments(), NowReal{kPinnedNow}, 0, *stats);
  ASSERT_TRUE(plan.valid) << "the search chose nothing: " << NlpRejectName(stats->nlp.reason);
  ASSERT_NE(search->Solution(), nullptr);
  const auto chain_solution = std::make_unique<CatchSolution>(*search->Solution());
  ASSERT_TRUE(chain_solution->converged);
  ASSERT_TRUE(chain_solution->feasible);
  ASSERT_EQ(chain_solution->joint_set, s.chain);
  ASSERT_EQ(chain_solution->seg.nv, fx::kG1CatchDof);
  ASSERT_EQ(chain_solution->seg.dt_catch_ns, 0);
  plan.plan_id = 1;  // the cycle's to give
  for (const NlpCatchSearch::CandidateRecord& c : search->Candidates()) {
    if (c.solved && c.t_c_ns == plan.t_c_ns) {
      out.chain_iterations = c.iterations;
    }
  }
  ::testing::Test::RecordProperty(tag + "_chain_iterations", out.chain_iterations);
  ::testing::Test::RecordProperty(tag + "_n_pre", chain_solution->seg.n_pre);

  // ── The 17-joint solve, started from that solution widened ──
  const std::unique_ptr<MpcDockingSegmentPlanner> planner =
      MakeDocking(s, Budget::kShipped, &err, torque_cost);
  ASSERT_NE(planner, nullptr) << err;
  auto wide = std::make_unique<SegmentSnapshot>();
  ASSERT_TRUE(
      planner->PlanFirst(rt, plan, BallOf(*ball), chain_solution.get(), *wide, out.from_chain))
      << Describe(out.from_chain);
  ASSERT_EQ(out.from_chain.outcome, SegmentOutcome::kReady) << Describe(out.from_chain);
  EXPECT_FALSE(out.from_chain.from_search);
  ASSERT_TRUE(out.from_chain.start_from_search) << Describe(out.from_chain);
  // On the search's own grid: the same problem, node for node.
  ASSERT_EQ(-out.from_chain.k, chain_solution->seg.n_pre);
  const MpcDockingSegmentCoreResult* res = planner->LastResult(-out.from_chain.k);
  ASSERT_NE(res, nullptr);
  out.from_chain_converged = res->converged;
  // The precondition: no torque row of the tree is active — the left arm's
  // included (the rows are on, every |τ/τ_max| is below 1).
  ASSERT_TRUE(std::isfinite(out.from_chain.tau_ratio_max));
  ASSERT_LT(out.from_chain.tau_ratio_max, 1.0);
  out.from_chain_gap = GapOf(s, *wide, chain_solution->seg);
  ASSERT_TRUE(out.from_chain_gap.comparable);
  // The scene moves the arm: two solves that stand still agree on anything.
  ASSERT_GT(out.from_chain_gap.reach, kMinReachRad);
  const std::string warm = tag + "_tree_from_the_chain_solution";
  RecordSci(warm + "_tau_ratio_max", out.from_chain.tau_ratio_max);
  ::testing::Test::RecordProperty(warm + "_sqp_iterations", out.from_chain.iterations);
  ::testing::Test::RecordProperty(warm + "_qp_solves", out.from_chain.docking.qp_solves);
  ::testing::Test::RecordProperty(warm + "_init_qp_used", res->init_qp_used ? 1 : 0);
  ::testing::Test::RecordProperty(warm + "_converged", res->converged ? 1 : 0);
  RecordGap(warm, out.from_chain_gap);

  // ── The 17-joint solve from the plan alone: no solution handed in ──
  const std::unique_ptr<MpcDockingSegmentPlanner> cold_planner =
      MakeDocking(s, Budget::kShipped, &err, torque_cost);
  ASSERT_NE(cold_planner, nullptr) << err;
  ASSERT_TRUE(cold_planner->PlanFirst(rt, plan, BallOf(*ball), nullptr, *wide, out.from_plan))
      << Describe(out.from_plan);
  EXPECT_FALSE(out.from_plan.start_from_search);
  ASSERT_TRUE(std::isfinite(out.from_plan.tau_ratio_max));
  ASSERT_LT(out.from_plan.tau_ratio_max, 1.0);
  out.from_plan_gap = GapOf(s, *wide, chain_solution->seg);
  ASSERT_TRUE(out.from_plan_gap.comparable);
  const std::string cold = tag + "_tree_from_the_plan";
  RecordSci(cold + "_tau_ratio_max", out.from_plan.tau_ratio_max);
  ::testing::Test::RecordProperty(cold + "_sqp_iterations", out.from_plan.iterations);
  RecordGap(cold, out.from_plan_gap);
}

// What Sprint Contract 4 says of mpc_docking, on a solved pair of widths.
void ExpectTheChainsProblem(const DockingAtBothWidths& r) {
  // ONE SQP iteration: the widened chain solution already solves the tree's
  // problem, so the solve linearises once and takes no step.
  EXPECT_TRUE(r.from_chain_converged);
  EXPECT_EQ(r.from_chain.iterations, 1) << Describe(r.from_chain);
  ExpectWithinD10(r.from_chain_gap, "the tree's solve from the widened chain solution");
  // ... and a solve of the tree's problem from elsewhere ends on it too.
  ExpectWithinD10(r.from_plan_gap, "the tree's solve from the plan alone");
}

// The premise: the joints the chain lacks at rest on the locked pose, the
// chain's torque rows off, the tree's on and inactive, and NO torque cost in
// either model. Then no term of the tree's problem reads a joint the chain
// lacks — its own torque, or the chain's torque through it — and the two
// problems are one: this is what says the widening and the joint maps are
// right. (With a torque cost they are not one — the file's header has the
// measurement.)
TEST(G1SearchSegmentAgreement, WithNoTorqueCostMpcDockingOnTheTreeIsTheChainsProblem) {
  RTC_SKIP_IF_PACKAGE_MISSING("hand_description");
  const PinnedClock pin(kPinnedNow);
  // E3-F02: with planner.groups declared (no weights) the tree's problem is
  // the same one — the agreement holds either way.
  for (const bool groups : {false, true}) {
    SCOPED_TRACE(groups ? "groups declared" : "no groups");
    auto s = std::make_unique<Scene>();
    ASSERT_NO_FATAL_FAILURE(BuildAgreementScene(*s));
    s->groups_declared = groups;
    auto r = std::make_unique<DockingAtBothWidths>();
    ASSERT_NO_FATAL_FAILURE(SolveDockingAtBothWidths(*s, *r));
    ExpectTheChainsProblem(*r);
    // Nothing moves a joint the chain lacks.
    EXPECT_LE(r->from_chain_gap.outside_q, 1e-9);
    EXPECT_LE(r->from_plan_gap.outside_q, 1e-9);
  }
}

// mpc: the same plan handed to the planner on the chain (10 joints, a device of
// its own ten) and to the planner on the tree (17).
// `groups`: the tree's planner with planner.groups declared (E3-F02, no
// weights) — the same problem, so the same agreement.
void RunMpcAgreement(bool groups) {
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(BuildAgreementScene(*s));
  s->groups_declared = groups;

  // The plan: the grid search's, on the chain. It plans nothing until it has
  // seen `n_settle` snapshots of the track: the same throw, read that many
  // times more.
  const std::unique_ptr<GridCatchSearch> search = MakeGrid(*s, Budget::kShipped);
  ASSERT_NE(search, nullptr);
  const PlannerRtState rt = s->RestingRt(kPinnedNow);
  const std::int64_t star = s->SearchStartNs(kPinnedNow, Budget::kShipped) +
                            static_cast<std::int64_t>(std::llround(kStarAfterT0S * 1e9));
  const std::unique_ptr<Throw> ball = s->AimedThrow(kPinnedNow, star, 50 * kMs);
  auto stats = std::make_unique<SearchStats>();
  PlanSnapshot plan{};
  for (int seen = 0; seen <= s->borrowed.planner.n_settle; ++seen) {
    ball->traj.token.snapshot_sequence = static_cast<std::uint64_t>(seen + 1);
    ball->cov.token = ball->traj.token;
    plan =
        search->Plan(ball->traj, ball->cov, true, rt, NoSegments(), NowReal{kPinnedNow}, 0, *stats);
  }
  ASSERT_TRUE(plan.valid) << Describe(*stats);
  ASSERT_EQ(plan.joint_set, s->chain);
  plan.plan_id = 1;

  // ── On the tree ──
  std::string err;
  const std::unique_ptr<MpcSegmentPlanner> tree = MakeMpc(*s, Budget::kShipped, &err);
  ASSERT_NE(tree, nullptr) << err;
  auto wide = std::make_unique<SegmentSnapshot>();
  SegmentRecord wide_rec{};
  ASSERT_TRUE(tree->PlanFirst(rt, plan, BallOf(*ball), nullptr, *wide, wide_rec))
      << Describe(wide_rec);

  // ── On the chain: the same plan and report, in a device of the chain's ten
  // joints (device joint d of the 17 is column JointSetSlot(chain, d) there) ──
  const std::unique_ptr<MpcSegmentPlanner> chain =
      MakeMpcOn(*s, s->search, s->slot_map, Budget::kShipped, &err);
  ASSERT_NE(chain, nullptr) << err;
  std::array<double, fx::kG1CatchDof> command10{};
  PlanSnapshot plan10 = plan;
  plan10.q_star.fill(0.0);
  plan10.joint_set = rtc::catching::FullJointSet(fx::kG1CatchDof);
  for (int d = 0; d < fx::kG1Dof; ++d) {
    if (InJointSet(s->chain, d)) {
      const auto slot = static_cast<std::size_t>(JointSetSlot(s->chain, d));
      command10[slot] = s->command[static_cast<std::size_t>(d)];
      plan10.q_star[slot] = plan.q_star[static_cast<std::size_t>(d)];
    }
  }
  PlannerRtState rt10 = rtc::testing::TrackingRtState(kActivation, command10);
  rt10.rt_iteration = rt.rt_iteration;
  rt10.rt_state_ns = rt.rt_state_ns;
  rt10.reset_epoch = rt.reset_epoch;
  rt10.track_seen = rt.track_seen;
  rt10.track_generation = rt.track_generation;
  auto narrow = std::make_unique<SegmentSnapshot>();
  SegmentRecord narrow_rec{};
  ASSERT_TRUE(chain->PlanFirst(rt10, plan10, BallOf(*ball), nullptr, *narrow, narrow_rec))
      << Describe(narrow_rec);
  ASSERT_EQ(narrow_rec.k, wide_rec.k);

  // The precondition: no torque row is active in either — the rows are soft
  // (|τ/τ_max| ≤ η_τ + s), so "inactive" is below η_τ with no slack.
  const double eta_tau = tree->Params().eta_tau;
  for (const SegmentRecord* r : {&wide_rec, &narrow_rec}) {
    ASSERT_TRUE(std::isfinite(r->tau_ratio_max));
    ASSERT_LT(r->tau_ratio_max, eta_tau);
    ASSERT_LT(r->slack_max, 1e-6);
  }
  RecordSci("tree_tau_ratio_max", wide_rec.tau_ratio_max);
  RecordSci("chain_tau_ratio_max", narrow_rec.tau_ratio_max);
  RecordSci("eta_tau", eta_tau);
  ::testing::Test::RecordProperty("tree_qp_iterations", wide_rec.iterations);
  ::testing::Test::RecordProperty("chain_qp_iterations", narrow_rec.iterations);
  ::testing::Test::RecordProperty("n_pre", -wide_rec.k);
  // Neither reference was cut: the two planners were given one problem.
  EXPECT_EQ(wide_rec.ref_scaled, narrow_rec.ref_scaled);
  EXPECT_EQ(wide_rec.ref_clamped, narrow_rec.ref_clamped);
  EXPECT_EQ(wide_rec.x0_clamped, narrow_rec.x0_clamped);

  const ChainGap gap = GapOf(*s, *wide, *narrow);
  RecordGap("tree_vs_chain", gap);
  ASSERT_GT(gap.reach, kMinReachRad);
  ExpectWithinD10(gap, "the mpc planner on the tree against the one on the chain");
}

TEST(G1SearchSegmentAgreement, MpcOnTheTreeSolvesTheChainsProblem) {
  RTC_SKIP_IF_PACKAGE_MISSING("hand_description");
  const PinnedClock pin(kPinnedNow);
  for (const bool groups : {false, true}) {
    SCOPED_TRACE(groups ? "groups declared" : "no groups");
    ASSERT_NO_FATAL_FAILURE(RunMpcAgreement(groups));
  }
}

// ── (5) Solve times: recorded, not judged ────────────────────────────────────

// Median and largest of a set of durations, as integer properties [µs].
void RecordUs(const std::string& name, std::vector<std::int64_t> ns) {
  ::testing::Test::RecordProperty(name + "_n", static_cast<int>(ns.size()));
  if (ns.empty()) {
    return;
  }
  std::sort(ns.begin(), ns.end());
  ::testing::Test::RecordProperty(name + "_median_us", static_cast<int>(ns[ns.size() / 2] / 1000));
  ::testing::Test::RecordProperty(name + "_max_us", static_cast<int>(ns.back() / 1000));
}

[[nodiscard]] int MedianOf(std::vector<int> v) {
  if (v.empty()) {
    return -1;
  }
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

// ── E3-F02 (#641): whole catches with the whole-body terms (G1, a check) ────
// The grid search on the chain, the mpc planner on the tree with the groups
// declared, the left arm's rest weight and the angular rate (D15's values);
// an RT stand-in that follows the first plan and takes every segment of it.
// The judging logic is the synthetic tree's (rtc_controllers
// test_catching_wholebody_cycle.cpp); this runs it on the G1 model.

struct WholeBodyWake {
  double rate_max{0.0};  // max ‖k̇_G‖ over the segment's nodes 1..N [N·m]
  double left_end{0.0};  // max |q_N − q_rest| over the left arm [rad]
  rtc::catching::SegmentKind kind{rtc::catching::SegmentKind::kFirst};
};

[[nodiscard]] WholeBodyWake LookAt(const Scene& s, const SegmentSnapshot& seg) {
  const pinocchio::Model& model = *s.segment.rig.arm.model;
  pinocchio::Data data(model);
  WholeBodyWake w;
  for (int k = 1; k <= seg.n_nodes; ++k) {
    Eigen::VectorXd q(model.nv), v(model.nv), a(model.nv);
    for (int m = 0; m < model.nv; ++m) {
      const auto e = Entry(k, s.segment.device_of_model[static_cast<std::size_t>(m)]);
      q[m] = seg.q[e];
      v[m] = seg.qd[e];
      a[m] = seg.qdd[e];
    }
    const Eigen::MatrixXd ag = pinocchio::computeCentroidalMap(model, data, q).bottomRows<3>();
    const double eps = 1e-6;
    const Eigen::MatrixXd ap =
        pinocchio::computeCentroidalMap(model, data, q + eps * v).bottomRows<3>();
    const Eigen::MatrixXd am =
        pinocchio::computeCentroidalMap(model, data, q - eps * v).bottomRows<3>();
    w.rate_max = std::max(w.rate_max, (ag * a + (ap - am) / (2.0 * eps) * v).norm());
  }
  for (int d = 3; d < 10; ++d) {  // the left arm, device order
    w.left_end = std::max(w.left_end, std::abs(seg.q[Entry(seg.n_nodes, d)] -
                                               s.command[static_cast<std::size_t>(d)]));
  }
  return w;
}

[[nodiscard]] std::vector<WholeBodyWake> RunWholeBodyCatch(double w_rest_left) {
  std::vector<WholeBodyWake> out;
  PinnedClock pin(kPinnedNow);
  auto s = std::make_unique<Scene>();
  s->Build(fx::kG1Home);
  s->groups_declared = true;
  s->wb_rest_left = w_rest_left;
  s->wb_ang_pre = 0.001;
  s->wb_ang_post = 0.1;
  auto w = std::make_unique<Wiring>();
  w->cycle.Configure(s->CycleParams(Budget::kShipped));
  w->cycle.SetClock(&SuiteClock);
  EXPECT_TRUE(w->cycle.Bind(PlannerCycleIo{&w->boxes.traj, &w->boxes.cov, &w->boxes.rt,
                                           &w->boxes.plan, &w->boxes.segment}));
  auto search = MakeGrid(*s, Budget::kShipped);
  EXPECT_NE(search, nullptr);
  w->cycle.InstallSearch(std::move(search));
  std::string err;
  auto planner = MakeMpc(*s, Budget::kShipped, &err);
  EXPECT_NE(planner, nullptr) << err;
  if (planner == nullptr) {
    return out;
  }
  w->cycle.InstallSegmentPlanner(std::move(planner));
  const std::int64_t star = s->SearchStartNs(kPinnedNow, Budget::kShipped) +
                            static_cast<std::int64_t>(std::llround(kStarAfterT0S * 1e9));
  // The RT stand-in's following state.
  bool following = false;
  std::uint32_t plan_id = 0;
  std::int64_t t_c = std::numeric_limits<std::int64_t>::max();
  std::uint32_t last_seq = 0, pending_seq = 0, active_seq = 0;
  std::int64_t pending_t0 = 0;
  bool pending = false, active = false;
  for (int wake = 0; wake < 200; ++wake) {
    const std::int64_t now = kPinnedNow + wake * kWakePeriodNs;
    if (now >= t_c) {
      break;
    }
    pin.Set(now);
    const std::unique_ptr<Throw> ball =
        s->AimedThrow(kPinnedNow, star, 50 * kMs, static_cast<std::uint64_t>(wake + 1));
    w->boxes.cov.Store(ball->cov);
    w->boxes.traj.Store(ball->traj);
    PlannerRtState rt = s->RestingRt(now, 500 + static_cast<std::uint64_t>(wake));
    if (!following) {
      const PlanSnapshot p = w->boxes.plan.Load();
      if (p.valid) {
        following = true;
        plan_id = p.plan_id;
        t_c = p.t_c_ns;
      }
    }
    if (following) {
      const SegmentSnapshot d = w->boxes.segment.Load();
      if (d.valid && d.plan_id == plan_id && d.segment_seq > last_seq) {
        last_seq = d.segment_seq;
        pending = true;
        pending_seq = d.segment_seq;
        pending_t0 = d.t0_ns;
      }
      if (pending &&
          now + static_cast<std::int64_t>(s->borrowed.t_arm_s * 1e9) + kMs >= pending_t0) {
        active = true;
        active_seq = pending_seq;
        pending = false;
      }
      const std::int64_t lead = t_c - now;
      rt.mode = static_cast<std::uint8_t>(lead <= 200 * kMs ? rtc::catching::Mode::kCommitted
                                                            : rtc::catching::Mode::kApproach);
      rt.plan_active = true;
      rt.plan_id = plan_id;
      rt.plan_t_c_ns = t_c;
      rt.segment_pending = pending;
      rt.segment_pending_seq = pending ? pending_seq : 0;
      rt.segment_active = active;
      rt.segment_seq = active ? active_seq : 0;
    }
    w->boxes.rt.Store(rt);
    const PlannerCycleRecord rec = w->cycle.Run(NowReal{now});
    if (rec.segment.outcome == SegmentOutcome::kPublished) {
      out.push_back(LookAt(*s, w->boxes.segment.Load()));
      out.back().kind = rec.segment.kind;
    }
  }
  return out;
}

TEST(G1WholeBodyCycle, TheRateStaysNearTheFirstWakesAndTheLeftArmEndsAtRest) {
  RTC_SKIP_IF_PACKAGE_MISSING("hand_description");
  const std::vector<WholeBodyWake> t = RunWholeBodyCatch(1.0);
  ASSERT_GE(t.size(), 6U) << "the first segment and five replans";
  RecordProperty("segments", static_cast<int>(t.size()));
  const double first = t.front().rate_max;
  ASSERT_GT(first, 0.0);
  for (std::size_t i = 0; i < t.size(); ++i) {
    EXPECT_LE(t[i].rate_max, 1.1 * first) << "segment " << i;
  }
  EXPECT_LE(t.back().left_end, 1e-2);
  RecordProperty("left_end_last", std::to_string(t.back().left_end));
}

TEST(G1WholeBodyCycle, WithoutItsRestWeightTheLeftArmIsNotBroughtBack) {
  RTC_SKIP_IF_PACKAGE_MISSING("hand_description");
  const std::vector<WholeBodyWake> t = RunWholeBodyCatch(0.0);
  ASSERT_GE(t.size(), 6U);
  double last = 0.0;
  for (std::size_t i = 0; i < t.size(); ++i) {
    if (i > 0 && t[i].kind == rtc::catching::SegmentKind::kFirst) {
      continue;
    }
    // Near saturation (the jerk cost against the angular term) the end node
    // wobbles: a return is more than 1 % of the drift reached.
    EXPECT_GE(t[i].left_end, 0.99 * last) << "segment " << i;
    last = t[i].left_end;
  }
  EXPECT_GT(t.back().left_end, t.front().left_end);
  EXPECT_GT(t.back().left_end, 10.0 * 1e-2) << "ten times the held arm's tolerance";
  RecordProperty("left_end_first", std::to_string(t.front().left_end));
  RecordProperty("left_end_last", std::to_string(t.back().left_end));
}

TEST(G1SearchSegmentTimes, RecordsTheSearchsCandidateAndTheFirstSolvesOnTheTree) {
  RTC_SKIP_IF_PACKAGE_MISSING("hand_description");
  auto s = std::make_unique<Scene>();
  ASSERT_NO_FATAL_FAILURE(s->Build(fx::kG1Home));
  std::string err;
  const std::unique_ptr<NlpCatchSearch> search = MakeNlp(*s, Budget::kGenerous, &err);
  ASSERT_NE(search, nullptr) << err;
  const std::unique_ptr<MpcSegmentPlanner> mpc = MakeMpc(*s, Budget::kGenerous, &err);
  ASSERT_NE(mpc, nullptr) << err;
  // Two planners: one that is always handed the search's solution, one never.
  const std::unique_ptr<MpcDockingSegmentPlanner> docking_from_chain =
      MakeDocking(*s, Budget::kGenerous, &err);
  ASSERT_NE(docking_from_chain, nullptr) << err;
  const std::unique_ptr<MpcDockingSegmentPlanner> docking_cold =
      MakeDocking(*s, Budget::kGenerous, &err);
  ASSERT_NE(docking_cold, nullptr) << err;

  enum Which { kMpc = 0, kDockingFromChain = 1, kDockingCold = 2 };

  struct Timed {
    std::vector<std::int64_t> ns;
    std::vector<int> iterations;
    std::vector<int> n_pre;
  };

  std::array<Timed, 3> first{};
  Timed candidate;
  constexpr int kRepeats = 7;
  std::uint32_t plan_id = 0;
  auto out = std::make_unique<SegmentSnapshot>();
  auto stats = std::make_unique<SearchStats>();
  for (int repeat = 0; repeat < kRepeats; ++repeat) {
    for (const Which which : {kMpc, kDockingFromChain, kDockingCold}) {
      SCOPED_TRACE("repeat " + std::to_string(repeat) + ", solve " + std::to_string(which));
      // A new throw and a new trial each time: every search solve is its
      // candidate's first (no memory), on the real clock.
      const std::int64_t now = rtc::SteadyNowNs();
      const std::int64_t star = s->SearchStartNs(now, Budget::kGenerous) +
                                static_cast<std::int64_t>(std::llround(kStarAfterT0S * 1e9));
      const std::unique_ptr<Throw> ball = s->AimedThrow(now, star, 100 * kMs);
      PlannerRtState rt = s->RestingRt(now, 500 + static_cast<std::uint64_t>(3 * repeat + which));
      search->ResetTrial();
      PlanSnapshot plan =
          search->Plan(ball->traj, ball->cov, true, rt, NoSegments(), NowReal{now}, 0, *stats);
      ASSERT_TRUE(plan.valid) << "the search chose nothing: " << NlpRejectName(stats->nlp.reason);
      // The timed search call solved ONE candidate, from the IK pose.
      ASSERT_EQ(static_cast<int>(stats->nlp.n_solved), 1);
      const CatchSolution* solution = search->Solution();
      ASSERT_NE(solution, nullptr);
      bool found = false;
      for (const NlpCatchSearch::CandidateRecord& c : search->Candidates()) {
        if (c.solved && c.solve_ns > 0) {
          ASSERT_FALSE(found) << "more than one candidate was solved";
          found = true;
          EXPECT_EQ(c.start, NlpCatchSearch::Start::kIkTarget);
          candidate.ns.push_back(c.solve_ns);
          candidate.iterations.push_back(c.iterations);
          candidate.n_pre.push_back(c.n_pre);
        }
      }
      ASSERT_TRUE(found);
      plan.plan_id = ++plan_id;
      // The same report, read again after the search: the planners refuse one
      // older than 50 ms, and how long the search took on this host is not
      // what is timed next.
      rt.rt_state_ns = rtc::SteadyNowNs() - kMs;
      SegmentRecord rec{};
      bool ok = false;
      if (which == kMpc) {
        mpc->ResetTrial();
        ok = mpc->PlanFirst(rt, plan, BallOf(*ball), solution, *out, rec);
      } else if (which == kDockingFromChain) {
        docking_from_chain->ResetTrial();
        ok = docking_from_chain->PlanFirst(rt, plan, BallOf(*ball), solution, *out, rec);
        EXPECT_TRUE(rec.start_from_search) << Describe(rec);
      } else {
        docking_cold->ResetTrial();
        ok = docking_cold->PlanFirst(rt, plan, BallOf(*ball), nullptr, *out, rec);
        EXPECT_FALSE(rec.start_from_search) << Describe(rec);
        EXPECT_FALSE(rec.start_from_memory) << Describe(rec);
      }
      // The timed call is a first solve of the device's 17 joints that ended
      // on a publishable segment.
      EXPECT_TRUE(ok) << Describe(rec);
      EXPECT_EQ(rec.kind, SegmentKind::kFirst);
      EXPECT_EQ(out->nv, fx::kG1Dof);
      EXPECT_GT(rec.solve_ns, 0);
      EXPECT_FALSE(rec.from_search);
      Timed& t = first[static_cast<std::size_t>(which)];
      t.ns.push_back(rec.solve_ns);
      t.iterations.push_back(rec.iterations);
      t.n_pre.push_back(-rec.k);
    }
  }
  const auto record = [](const std::string& name, const Timed& t) {
    RecordUs(name, t.ns);
    ::testing::Test::RecordProperty(name + "_median_iterations", MedianOf(t.iterations));
    ::testing::Test::RecordProperty(name + "_median_n_pre", MedianOf(t.n_pre));
  };
  record("nlp_candidate_chain10_cold", candidate);
  record("mpc_first_tree17", first[kMpc]);
  record("mpc_docking_first_tree17_from_the_chain_solution", first[kDockingFromChain]);
  record("mpc_docking_first_tree17_cold", first[kDockingCold]);
#ifdef RTC_TEST_BUILD_TYPE
  ::testing::Test::RecordProperty("build_type", RTC_TEST_BUILD_TYPE);
#endif
}

// ── E3-F02 (#641): the 17-joint time baseline (a record, not a verdict) ──────
// Run only with RTC_WHOLEBODY_TIMING set, alone (ctest -j1). Four variants of
// the tree's planners — the whole-body terms off / on × no own blocking / the
// left arm's — solve the same plan and the same chain solution, in turns
// (ABCD, DCBA, …), on three throws (the catch 0.55, 0.65, 0.75 s past the
// search's earliest start), 1 + 20 times each (the first is warm-up). Per
// variant and solve: the median and the max of the solve time, the
// iterations, the time per QP iteration, the QP's dimensions, the cold
// retries, and the same wake's search + first solve.

struct TimingColumn {
  std::vector<std::int64_t> ns;
  std::vector<int> iterations, qp_iterations, n_pre;
  std::vector<double> qp_us;
  int cold{0}, init_qp{0};
  int nz{0}, n_eq{0}, n_in{0}, active{0};
};

void PrintColumn(const std::string& name, TimingColumn c) {
  if (c.ns.empty()) {
    std::printf("[timing] %-36s (no solve)\n", name.c_str());
    return;
  }
  std::vector<double> per_it;
  for (std::size_t i = 0; i < c.ns.size(); ++i) {
    const int it = c.qp_iterations[i] > 0 ? c.qp_iterations[i] : c.iterations[i];
    per_it.push_back(it > 0 ? c.qp_us[i] / it : 0.0);
  }
  std::sort(c.ns.begin(), c.ns.end());
  std::sort(per_it.begin(), per_it.end());
  std::printf(
      "[timing] %-36s n=%zu median=%.1f ms max=%.1f ms it_med=%d qp_it_med=%d us/it_med=%.1f "
      "nz=%d n_eq=%d n_in=%d active=%d cold=%d init_qp=%d n_pre_med=%d\n",
      name.c_str(), c.ns.size(), static_cast<double>(c.ns[c.ns.size() / 2]) * 1e-6,
      static_cast<double>(c.ns.back()) * 1e-6, MedianOf(c.iterations), MedianOf(c.qp_iterations),
      per_it[per_it.size() / 2], c.nz, c.n_eq, c.n_in, c.active, c.cold, c.init_qp,
      MedianOf(c.n_pre));
  RecordUs(name, c.ns);
}

// Rows of `qp` at a bound at `x` (finite bounds, to 1e-7).
int ActiveRows(const rtc::tsid::QPData& qp, const Eigen::VectorXd& x) {
  if (x.size() < qp.C.cols()) {
    return -1;
  }
  const Eigen::VectorXd cx = qp.C * x.head(qp.C.cols());
  int active = 0;
  for (Eigen::Index r = 0; r < cx.size(); ++r) {
    const bool lo = std::isfinite(qp.l[r]) && std::abs(cx[r] - qp.l[r]) <= 1e-7;
    const bool hi = std::isfinite(qp.u[r]) && std::abs(cx[r] - qp.u[r]) <= 1e-7;
    active += lo || hi ? 1 : 0;
  }
  return active;
}

TEST(G1WholeBodyTimes, RecordsTheBaselineWithAndWithoutTheTerms) {
  RTC_SKIP_IF_PACKAGE_MISSING("hand_description");
  if (std::getenv("RTC_WHOLEBODY_TIMING") == nullptr) {
    GTEST_SKIP() << "a record: set RTC_WHOLEBODY_TIMING and run alone (ctest -j1)";
  }

  struct Variant {
    const char* name;
    bool terms;
    bool coarse;
    std::unique_ptr<Scene> scene;
    std::unique_ptr<MpcSegmentPlanner> mpc;
    std::unique_ptr<MpcDockingSegmentPlanner> dock_chain, dock_cold;
    std::array<TimingColumn, 4> col;  // mpc first, mpc replan, docking from chain, docking cold
  };

  std::array<Variant, 4> v{Variant{"off", false, false, {}, {}, {}, {}, {}},
                           Variant{"terms", true, false, {}, {}, {}, {}, {}},
                           Variant{"blocks", false, true, {}, {}, {}, {}, {}},
                           Variant{"terms_blocks", true, true, {}, {}, {}, {}, {}}};
  std::string err;
  for (Variant& x : v) {
    x.scene = std::make_unique<Scene>();
    ASSERT_NO_FATAL_FAILURE(x.scene->Build(fx::kG1Home));
    x.scene->groups_declared = true;
    x.scene->wb_coarse_left = x.coarse;
    if (x.terms) {
      x.scene->wb_rest_left = 1.0;
      x.scene->wb_ang_pre = 0.001;
      x.scene->wb_ang_post = 0.1;
    }
    x.mpc = MakeMpc(*x.scene, Budget::kGenerous, &err);
    ASSERT_NE(x.mpc, nullptr) << x.name << ": " << err;
    x.dock_chain = MakeDocking(*x.scene, Budget::kGenerous, &err);
    ASSERT_NE(x.dock_chain, nullptr) << x.name << ": " << err;
    x.dock_cold = MakeDocking(*x.scene, Budget::kGenerous, &err);
    ASSERT_NE(x.dock_cold, nullptr) << x.name << ": " << err;
  }
  Scene& s0 = *v[0].scene;
  const std::unique_ptr<NlpCatchSearch> search = MakeNlp(s0, Budget::kGenerous, &err);
  ASSERT_NE(search, nullptr) << err;
  std::vector<std::int64_t> same_wake;
  auto out = std::make_unique<SegmentSnapshot>();
  auto stats = std::make_unique<SearchStats>();
  std::uint32_t plan_id = 0;
  constexpr int kRepeats = 20;
  for (const double star_after : {0.55, 0.65, 0.75}) {
    for (int rep = 0; rep <= kRepeats; ++rep) {
      const bool warm_up = rep == 0;
      for (int turn = 0; turn < 4; ++turn) {
        Variant& x = v[static_cast<std::size_t>(rep % 2 == 0 ? turn : 3 - turn)];
        // A new throw on the real clock, the search's one candidate, then
        // the variant's four solves on that plan.
        const std::int64_t now = rtc::SteadyNowNs();
        const std::int64_t star = s0.SearchStartNs(now, Budget::kGenerous) +
                                  static_cast<std::int64_t>(std::llround(star_after * 1e9));
        const std::unique_ptr<Throw> ball = s0.AimedThrow(now, star, 100 * kMs);
        PlannerRtState rt = s0.RestingRt(now, 500);
        search->ResetTrial();
        PlanSnapshot plan =
            search->Plan(ball->traj, ball->cov, true, rt, NoSegments(), NowReal{now}, 0, *stats);
        ASSERT_TRUE(plan.valid) << NlpRejectName(stats->nlp.reason);
        const CatchSolution* solution = search->Solution();
        std::int64_t search_ns = 0;
        for (const NlpCatchSearch::CandidateRecord& c : search->Candidates()) {
          search_ns += c.solved ? c.solve_ns : 0;
        }
        plan.plan_id = ++plan_id;
        const auto note = [&](TimingColumn& c, const SegmentRecord& rec, bool ok) {
          EXPECT_TRUE(ok) << x.name << " " << Describe(rec);
          if (warm_up || !ok) {
            return;
          }
          c.ns.push_back(rec.solve_ns);
          c.iterations.push_back(rec.iterations);
          c.qp_iterations.push_back(rec.docking.ran ? rec.docking.qp_iterations : rec.iterations);
          c.qp_us.push_back(rec.docking.ran ? rec.docking.qp_us : rec.mpc_qp_us);
          c.n_pre.push_back(-rec.k);
          c.cold += rec.solver_retried ? 1 : 0;
          c.init_qp += rec.start_from_search ? 0 : (rec.docking.ran ? 1 : 0);
        };
        // mpc: the first solve, then a re-solve of its grid point on a newer
        // snapshot (the replan the RT following it asks for).
        {
          SegmentRecord rec{};
          x.mpc->ResetTrial();
          rt.rt_state_ns = rtc::SteadyNowNs() - kMs;
          const bool ok = x.mpc->PlanFirst(rt, plan, BallOf(*ball), solution, *out, rec);
          note(x.col[0], rec, ok);
          if (!warm_up && ok) {
            same_wake.push_back(search_ns + rec.solve_ns);
            const int n_pre = out->n_pre;
            const rtc::tsid::QPData& qp = x.mpc->ApproachCore(n_pre).MainQp();
            x.col[0].nz = static_cast<int>(qp.H.rows());
            x.col[0].n_eq = static_cast<int>(qp.A.rows());
            x.col[0].n_in = static_cast<int>(qp.C.rows());
          }
          if (ok) {
            out->segment_seq = 1;
            out->publish_ns = rtc::SteadyNowNs();
            x.mpc->NotePublished(*out);
            PlannerRtState following = s0.RestingRt(rtc::SteadyNowNs(), 501);
            following.mode = static_cast<std::uint8_t>(rtc::catching::Mode::kApproach);
            following.plan_active = true;
            following.plan_id = plan.plan_id;
            following.plan_t_c_ns = plan.t_c_ns;
            following.segment_active = true;
            following.segment_seq = 1;
            following.rt_state_ns = rtc::SteadyNowNs() - kMs;
            const std::unique_ptr<Throw> newer = s0.AimedThrow(now, star, 100 * kMs, 2);
            SegmentRecord again{};
            const bool replanned = x.mpc->Replan(following, BallOf(*newer), *out, again);
            note(x.col[1], again, replanned);
          }
        }
        {
          SegmentRecord rec{};
          x.dock_chain->ResetTrial();
          rt.rt_state_ns = rtc::SteadyNowNs() - kMs;
          const bool ok = x.dock_chain->PlanFirst(rt, plan, BallOf(*ball), solution, *out, rec);
          note(x.col[2], rec, ok);
          if (!warm_up && ok) {
            const auto& core = x.dock_chain->CoreFor(out->n_pre);
            x.col[2].nz = static_cast<int>(core.LastQp().H.rows());
            x.col[2].n_eq = static_cast<int>(core.LastQp().A.rows());
            x.col[2].n_in = static_cast<int>(core.LastQp().C.rows());
            x.col[2].active = ActiveRows(core.LastQp(), core.LastQpSolution());
          }
        }
        {
          SegmentRecord rec{};
          x.dock_cold->ResetTrial();
          rt.rt_state_ns = rtc::SteadyNowNs() - kMs;
          const bool ok = x.dock_cold->PlanFirst(rt, plan, BallOf(*ball), nullptr, *out, rec);
          note(x.col[3], rec, ok);
        }
      }
    }
  }
  for (Variant& x : v) {
    PrintColumn(std::string("mpc_first_") + x.name, x.col[0]);
    PrintColumn(std::string("mpc_replan_") + x.name, x.col[1]);
    PrintColumn(std::string("docking_from_chain_") + x.name, x.col[2]);
    PrintColumn(std::string("docking_cold_") + x.name, x.col[3]);
  }
  RecordUs("search_plus_mpc_first_same_wake", same_wake);
  std::sort(same_wake.begin(), same_wake.end());
  if (!same_wake.empty()) {
    std::printf("[timing] search + mpc first, same wake: median %.1f ms, max %.1f ms\n",
                static_cast<double>(same_wake[same_wake.size() / 2]) * 1e-6,
                static_cast<double>(same_wake.back()) * 1e-6);
  }
}

}  // namespace
