// StageReference · HorizonReference · StageReferenceBuilder (reference/): the
// W2-F02 [SPRINT] PR B gates. The base knots are the pre-registered simulation's
// (the original target-trajectory arithmetic in Python, full precision), the
// contact poses have two independent oracles (a closed form for the frame
// hung off the root, a separate pinocchio Data FK for the finger), the swing
// heights are PR A's BuildSwing on the window's own intervals, and the arm
// swing phase is a tile loop. Every refusal sits next to an accepted twin.
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/reference/contact_schedule.hpp"
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "rtc_mpc/reference/stage_reference_builder.hpp"
#include "rtc_mpc/reference/swing_trajectory.hpp"
#include "test_utils/loco_reference_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

namespace rtc::mpc {
namespace {

namespace tu = test_utils;

constexpr double kTol = 1e-12;
constexpr int kNodes = 51;
constexpr double kDt = 0.02;
constexpr double kT0 = 1e4 + 0.123;
constexpr double kInf = std::numeric_limits<double>::infinity();
const double kNaN = std::numeric_limits<double>::quiet_NaN();

// Registered (pre-registration sim, full precision).
constexpr double kPsi0 = 1.3;
constexpr double kVmX = 0.15192759855888655, kVmY = 0.16409665186455447;
constexpr double kWm = 0.09937856379324261;
constexpr double kP1X = 0.36626241801531134, kP1Y = 0.03541896960246338, kPsi1 = 1.439782497327635;
constexpr double kP2X = 0.37748049674648365, kP2Y = 0.18797766227378, kPsi2 = 1.529782497327635;
constexpr double kInvalidPX = 0.3531746594956103, kInvalidPY = -0.14256617184740594,
                 kInvalidPsi = 1.334782497327635;
constexpr double kV35X = 0.5090559355672153, kV35Y = 0.029360764019132554;
constexpr double kV50X = 0.5096345531575305, kV50Y = -0.016511275781242243;

struct Rig {
  pinocchio::Model model;
  RobotModelHandler handler;
  GaitTable table;
  GaitScheduler scheduler;
  std::vector<double> q_default;
  std::vector<double> q_snapshot;
  ReferenceConfig cfg;
  StageReferenceBuilder builder;

  Rig() : model(tu::BuildLocoPanda()) {
    EXPECT_EQ(handler.Init(model, YAML::Load(tu::kLocoHandlerYaml)), RobotModelInitError::kNoError);
    table = tu::LocoGaitTable();
    EXPECT_EQ(scheduler.Init(table, 1.0), GaitSchedulerError::kNoError);
    q_default = tu::RandomInsideLimits(model, 20261112U, 0.1);
    q_snapshot = tu::RandomInsideLimits(model, 777U, 0.05);
    q_snapshot[7] = 0.011;  // the fingers
    q_snapshot[8] = 0.034;
    q_default[7] = 0.02;
    q_default[8] = 0.01;
  }

  // The walk running since t0 − 1.27 and the window rebuilt at t0.
  void Walk(double t0, bool arm_swing = false, bool run_policy = false) {
    EXPECT_EQ(
        LoadReferenceConfig(tu::ReferenceYaml(model, q_default, arm_swing,
                                              {{"panda_joint2", 1.0}, {"panda_joint4", -1.0}}),
                            tu::ActuatedJointNames(model), cfg),
        ReferenceError::kNoError);
    EXPECT_EQ(builder.Init(handler, cfg, tu::ShippedSwing(), FlatSurface(0.0), kNodes, kDt),
              ReferenceError::kNoError);
    tu::WalkFrom(scheduler, t0, run_policy);
  }

  MPCStateSnapshot Snapshot() const { return tu::RegisteredSnapshot(model, q_snapshot); }
};

bool UrdfMissing() {
  return !std::filesystem::exists(tu::kPandaUrdf);
}

double Lerp(double a, double b, double alpha) {
  return a + (b - a) * alpha;
}

// The base knot at node k of the registered fixture (f = 0.7, H = 1).
void RegisteredBase(int k, bool valid, double& x, double& y, double& psi) {
  const double t = k * kDt;
  const double p1x = valid ? kP1X : kInvalidPX, p1y = valid ? kP1Y : kInvalidPY;
  const double psi1 = valid ? kPsi1 : kInvalidPsi;
  const double p2x = valid ? kP2X : kInvalidPX, p2y = valid ? kP2Y : kInvalidPY;
  const double psi2 = valid ? kPsi2 : kInvalidPsi;
  if (t <= 0.7) {
    const double a = t / 0.7;
    x = Lerp(0.3, p1x, a);
    y = Lerp(-0.2, p1y, a);
    psi = Lerp(kPsi0, psi1, a);
  } else {
    const double a = (t - 0.7) / 0.3;
    x = Lerp(p1x, p2x, a);
    y = Lerp(p1y, p2y, a);
    psi = Lerp(psi1, psi2, a);
  }
}

// ── Types ─────────────────────────────────────────────────────────────────────

TEST(StageReferenceTypes, PodsCarryNoPaddingAndFitTwoSlots) {
  EXPECT_TRUE(std::is_trivially_copyable_v<StageReference>);
  EXPECT_TRUE(std::is_trivially_copyable_v<HorizonReference>);
  EXPECT_LE(sizeof(HorizonReference), 256U * 1024U);
  EXPECT_EQ(sizeof(HorizonReference) % alignof(double), 0U);
}

// ── Init ──────────────────────────────────────────────────────────────────────

TEST(StageReferenceBuilder, InitCapacityAndModelRefusalsHaveAcceptedTwins) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  Rig rig;
  ASSERT_EQ(LoadReferenceConfig(tu::ReferenceYaml(rig.model, rig.q_default, false, {}),
                                tu::ActuatedJointNames(rig.model), rig.cfg),
            ReferenceError::kNoError);
  const SwingConfig swing = tu::ShippedSwing();
  {
    StageReferenceBuilder b;
    EXPECT_EQ(b.Init(rig.handler, rig.cfg, swing, FlatSurface(0.0), kMaxHorizon + 1, kDt),
              ReferenceError::kNoError);
    EXPECT_EQ(b.n_nodes(), 65);
  }
  {
    StageReferenceBuilder b;
    EXPECT_EQ(b.Init(rig.handler, rig.cfg, swing, FlatSurface(0.0), kMaxHorizon + 2, kDt),
              ReferenceError::kCapacity);
    EXPECT_FALSE(b.initialized());
    EXPECT_EQ(b.Init(rig.handler, rig.cfg, swing, FlatSurface(0.0), 1, kDt),
              ReferenceError::kCapacity);
    EXPECT_EQ(b.Init(rig.handler, rig.cfg, swing, FlatSurface(0.0), kNodes, 0.0),
              ReferenceError::kInvalidTimeStep);
    EXPECT_EQ(b.Init(rig.handler, rig.cfg, swing, FlatSurface(0.0), kNodes, kNaN),
              ReferenceError::kInvalidTimeStep);
    EXPECT_EQ(b.Init(rig.handler, rig.cfg, swing, FlatSurface(0.0), kNodes, 1e-10),
              ReferenceError::kInvalidTimeStep);
    ReferenceConfig nan_gain = rig.cfg;
    nan_gain.arm_swing_gain = kNaN;
    EXPECT_EQ(b.Init(rig.handler, nan_gain, swing, FlatSurface(0.0), kNodes, kDt),
              ReferenceError::kNonFinite);
    SwingConfig bad = swing;
    bad.height = 0.0;
    EXPECT_EQ(b.Init(rig.handler, rig.cfg, bad, FlatSurface(0.0), kNodes, kDt),
              ReferenceError::kInvalidSwingConfig);
    EXPECT_EQ(b.Init(rig.handler, rig.cfg, swing, FlatSurface(kNaN), kNodes, kDt),
              ReferenceError::kInvalidTerrain);
    RobotModelHandler none;
    EXPECT_EQ(b.Init(none, rig.cfg, swing, FlatSurface(0.0), kNodes, kDt),
              ReferenceError::kHandlerNotInitialised);
  }
  // A fixed-base model is refused; a floating one with the same joints passes.
  {
    pinocchio::Model fixed;
    pinocchio::urdf::buildModel(tu::kPandaUrdf, fixed);
    RobotModelHandler h;
    ASSERT_EQ(h.Init(fixed, YAML::Load(tu::kPandaTwoFingerYaml)), RobotModelInitError::kNoError);
    ReferenceConfig cfg;
    ASSERT_EQ(LoadReferenceConfig(tu::ReferenceYaml(fixed, rig.q_default, false, {}),
                                  tu::ActuatedJointNames(fixed), cfg),
              ReferenceError::kNoError);
    StageReferenceBuilder b;
    EXPECT_EQ(b.Init(h, cfg, swing, FlatSurface(0.0), kNodes, kDt), ReferenceError::kFixedBase);
  }
  // More contacts than the layout holds: nine point contacts.
  {
    YAML::Node y = YAML::Load(tu::kLocoHandlerYaml);
    y["contact_frames"] = YAML::Node(YAML::NodeType::Sequence);
    for (int i = 0; i <= 8; ++i) {
      YAML::Node c;
      c["name"] = "panda_link" + std::to_string(i);
      c["dim"] = 3;
      y["contact_frames"].push_back(c);
    }
    RobotModelHandler h;
    ASSERT_EQ(h.Init(rig.model, y), RobotModelInitError::kNoError);
    ASSERT_EQ(h.n_contacts(), 9);
    StageReferenceBuilder b;
    EXPECT_EQ(b.Init(h, rig.cfg, swing, FlatSurface(0.0), kNodes, kDt), ReferenceError::kCapacity);
  }
  // q_default at the limit passes; outside it is refused.
  {
    std::vector<double> at_limit = rig.q_default;
    at_limit[0] = rig.model.upperPositionLimit[7];
    ReferenceConfig cfg;
    ASSERT_EQ(LoadReferenceConfig(tu::ReferenceYaml(rig.model, at_limit, false, {}),
                                  tu::ActuatedJointNames(rig.model), cfg),
              ReferenceError::kNoError);
    StageReferenceBuilder b;
    EXPECT_EQ(b.Init(rig.handler, cfg, swing, FlatSurface(0.0), kNodes, kDt),
              ReferenceError::kNoError);
    std::vector<double> outside = rig.q_default;
    outside[0] = rig.model.upperPositionLimit[7] + 1e-3;
    ASSERT_EQ(LoadReferenceConfig(tu::ReferenceYaml(rig.model, outside, false, {}),
                                  tu::ActuatedJointNames(rig.model), cfg),
              ReferenceError::kNoError);
    StageReferenceBuilder b2;
    EXPECT_EQ(b2.Init(rig.handler, cfg, swing, FlatSurface(0.0), kNodes, kDt),
              ReferenceError::kOutOfLimits);
    EXPECT_FALSE(b2.initialized());
  }
}

// ── Nodes · base · joints ─────────────────────────────────────────────────────

TEST(StageReferenceBuilder, NodeTimesModesAndTheRegisteredBaseKnots) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  Rig rig;
  rig.Walk(kT0);
  const MPCStateSnapshot snap = rig.Snapshot();
  const FilteredCommand cmd = tu::RegisteredCommand();
  ASSERT_EQ(rig.builder.Build(rig.scheduler, snap, cmd), ReferenceError::kNoError);
  const HorizonReference& h = rig.builder.active();
  EXPECT_EQ(h.n_stages, 51U);
  EXPECT_EQ(h.n_contacts, 2U);
  EXPECT_EQ(h.t0, kT0);
  EXPECT_EQ(h.dt, kDt);
  EXPECT_EQ(h.reference_id, 1U);
  EXPECT_EQ(h.schedule_id, rig.scheduler.schedule().schedule_id);
  EXPECT_EQ(h.schedule_epoch, rig.scheduler.schedule().epoch);
  EXPECT_EQ(h.observation_sequence, 7U);
  EXPECT_EQ(h.command_sequence, 11U);
  EXPECT_EQ(h.model_epoch, 3U);
  EXPECT_EQ(h.current_gait, 1U);
  EXPECT_EQ(h.nq, 16U);
  EXPECT_EQ(h.nv, 15U);
  EXPECT_EQ(h.n_contact_vars, 9U);
  std::array<std::uint32_t, kNodes> modes{};
  ASSERT_TRUE(SampleNodes(rig.scheduler.schedule(), kT0, kDt, kNodes - 1, modes));
  bool saw_single_support = false;
  for (int k = 0; k < kNodes; ++k) {
    const StageReference& st = h.stages[k];
    SCOPED_TRACE(k);
    const double expect_time = kT0 + k * kDt;
    ASSERT_EQ(std::memcmp(&st.time, &expect_time, sizeof(double)), 0);
    ASSERT_EQ(st.mode, modes[k]);
    ASSERT_EQ(st.n_stance, static_cast<std::uint32_t>(std::popcount(st.mode)));
    saw_single_support = saw_single_support || st.n_stance == 1;
    double x = 0, y = 0, psi = 0;
    RegisteredBase(k, true, x, y, psi);
    ASSERT_NEAR(st.q[0], x, kTol);
    ASSERT_NEAR(st.q[1], y, kTol);
    ASSERT_NEAR(st.q[2] - 0.75, 0.0, 1e-15);
    ASSERT_EQ(st.q[3], 0.0);
    ASSERT_EQ(st.q[4], 0.0);
    const double norm =
        std::sqrt(st.q[3] * st.q[3] + st.q[4] * st.q[4] + st.q[5] * st.q[5] + st.q[6] * st.q[6]);
    ASSERT_LE(std::fabs(norm - 1.0), 4.5e-16);
    const double dot = st.q[5] * std::sin(0.5 * psi) + st.q[6] * std::cos(0.5 * psi);
    ASSERT_GE(std::fabs(dot), 1.0 - 1e-15);
    // WD-63: the commanded velocity seen from the node's own yaw.
    const double c = std::cos(kPsi0 - psi), s = std::sin(kPsi0 - psi);
    ASSERT_NEAR(st.v[0], c * 0.5 - s * 0.1, kTol);
    ASSERT_NEAR(st.v[1], s * 0.5 + c * 0.1, kTol);
    ASSERT_EQ(st.v[2], 0.0);
    ASSERT_EQ(st.v[3], 0.0);
    ASSERT_EQ(st.v[4], 0.0);
    ASSERT_EQ(st.v[5], 0.3);
    for (int i = 7; i < 16; ++i) {
      ASSERT_EQ(std::memcmp(&st.q[i], &rig.q_default[i - 7], sizeof(double)), 0) << i;
    }
    for (int i = 6; i < 15; ++i) {
      ASSERT_EQ(st.v[i], 0.0);
    }
  }
  EXPECT_TRUE(saw_single_support);
  // The registered knots themselves (a yaw-only mutant lands at (0.3767553, 0.1886305)).
  EXPECT_NEAR(h.stages[35].q[0], kP1X, 1e-9);
  EXPECT_NEAR(h.stages[35].q[1], kP1Y, 1e-9);
  EXPECT_NEAR(2.0 * std::atan2(h.stages[35].q[5], h.stages[35].q[6]), kPsi1, 1e-9);
  EXPECT_NEAR(h.stages[50].q[0], kP2X, 1e-9);
  EXPECT_NEAR(h.stages[50].q[1], kP2Y, 1e-9);
  EXPECT_NEAR(2.0 * std::atan2(h.stages[50].q[5], h.stages[50].q[6]), kPsi2, 1e-9);
  EXPECT_GT(std::hypot(h.stages[50].q[0] - 0.3767553, h.stages[50].q[1] - 0.1886305), 1e-4);
  EXPECT_NEAR(h.stages[35].v[0], kV35X, 1e-9);
  EXPECT_NEAR(h.stages[35].v[1], kV35Y, 1e-9);
  EXPECT_NEAR(h.stages[50].v[0], kV50X, 1e-9);
  EXPECT_NEAR(h.stages[50].v[1], kV50Y, 1e-9);
  // The measured world velocity the knots were built from (full R, not yaw).
  const Eigen::Matrix3d R = tu::Rzyx(1.3, -0.05, 0.1);
  const Eigen::Vector3d vm = R * Eigen::Vector3d(0.2, -0.1, 0.03);
  EXPECT_NEAR(vm[0], kVmX, 1e-12);
  EXPECT_NEAR(vm[1], kVmY, 1e-12);
  EXPECT_NEAR((R * Eigen::Vector3d(0.02, -0.01, 0.1))[2], kWm, 1e-12);
}

TEST(StageReferenceBuilder, AnInvalidCommandIntegratesOnlyTheMeasuredVelocity) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  Rig rig;
  rig.Walk(kT0, true);
  ASSERT_EQ(rig.builder.Build(rig.scheduler, rig.Snapshot(), tu::RegisteredCommand(false)),
            ReferenceError::kNoError);
  const HorizonReference& h = rig.builder.active();
  EXPECT_NEAR(h.stages[50].q[0], kInvalidPX, 1e-9);
  EXPECT_NEAR(h.stages[50].q[1], kInvalidPY, 1e-9);
  EXPECT_NEAR(2.0 * std::atan2(h.stages[50].q[5], h.stages[50].q[6]), kInvalidPsi, 1e-9);
  for (int k = 0; k < kNodes; ++k) {
    double x = 0, y = 0, psi = 0;
    RegisteredBase(k, false, x, y, psi);
    ASSERT_NEAR(h.stages[k].q[0], x, kTol) << k;
    ASSERT_NEAR(h.stages[k].q[1], y, kTol) << k;
    ASSERT_NEAR(h.stages[k].q[2] - 0.75, 0.0, 1e-15);
    for (int i = 0; i < 6; ++i) {
      ASSERT_EQ(h.stages[k].v[i], 0.0) << k << " " << i;
    }
    for (int i = 7; i < 16; ++i) {  // arm swing on, but v_x reads 0: Δq = 0
      ASSERT_EQ(std::memcmp(&h.stages[k].q[i], &rig.q_default[i - 7], sizeof(double)), 0);
    }
  }
}

TEST(StageReferenceBuilder, YawInterpolatesUnwrappedPastPi) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  Rig rig;
  rig.Walk(kT0);
  MPCStateSnapshot snap = tu::RegisteredSnapshot(rig.model, rig.q_snapshot, 3.05, 0.0, 0.0);
  for (int i = 0; i < 6; ++i) {
    snap.v[i] = 0.0;
  }
  FilteredCommand cmd = tu::RegisteredCommand();
  cmd.v_y = 0.0;
  cmd.omega_z = 0.5;
  ASSERT_EQ(rig.builder.Build(rig.scheduler, snap, cmd), ReferenceError::kNoError);
  const HorizonReference& h = rig.builder.active();
  // ψ(0.7H) = 3.225, ψ(H) = 3.375 > π; node 43 (t = 0.86) sits between them on the
  // unwrapped line (a wrap-then-interpolate mutant lands near 0.18).
  const double psi_43 =
      3.2249999999999996 + (0.86 - 0.7) / 0.3 * (3.3749999999999996 - 3.2249999999999996);
  const double q5 = h.stages[43].q[5], q6 = h.stages[43].q[6];
  EXPECT_GE(std::fabs(q5 * std::sin(0.5 * psi_43) + q6 * std::cos(0.5 * psi_43)), 1.0 - 1e-15);
  EXPECT_NEAR(2.0 * std::atan2(h.stages[50].q[5], h.stages[50].q[6]), 3.375, 1e-9);
  // The wrap-then-interpolate mutant's node 43 (≈ 0.158 rad) is far from this one.
  EXPECT_LT(std::fabs(q5 * std::sin(0.5 * 0.158) + q6 * std::cos(0.5 * 0.158)), 0.5);
}

TEST(StageReferenceBuilder, BaseBelowTheTerrainIsRefusedAndTheActiveHorizonStays) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  Rig rig;
  rig.Walk(kT0);
  ASSERT_EQ(rig.builder.Build(rig.scheduler, rig.Snapshot(), tu::RegisteredCommand()),
            ReferenceError::kNoError);
  const auto before = std::make_unique<HorizonReference>(rig.builder.active());
  FilteredCommand low = tu::RegisteredCommand();
  low.base_height = 0.0;  // == the flat terrain
  EXPECT_EQ(rig.builder.Build(rig.scheduler, rig.Snapshot(), low),
            ReferenceError::kBaseBelowTerrain);
  EXPECT_EQ(std::memcmp(before.get(), &rig.builder.active(), sizeof(HorizonReference)), 0);
  low.base_height = 1e-3;
  EXPECT_EQ(rig.builder.Build(rig.scheduler, rig.Snapshot(), low), ReferenceError::kNoError);
}

// ── Contacts ──────────────────────────────────────────────────────────────────

TEST(StageReferenceBuilder, ContactPosesComeFromTheMeasuredKinematicsAndTheSwingPlan) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  Rig rig;
  rig.Walk(kT0);
  const MPCStateSnapshot snap = rig.Snapshot();
  ASSERT_EQ(rig.builder.Build(rig.scheduler, snap, tu::RegisteredCommand()),
            ReferenceError::kNoError);
  const HorizonReference& h = rig.builder.active();

  // Oracle (a): the root frame hung off the base, closed form p_b + R_b·offset.
  const Eigen::Matrix3d Rb = tu::Rzyx(1.3, -0.05, 0.1);
  const Eigen::Vector3d pa =
      Eigen::Vector3d(0.3, -0.2, 0.71) +
      Rb * Eigen::Vector3d(tu::kFootOffsetX, tu::kFootOffsetY, tu::kFootOffsetZ);
  const Eigen::Matrix3d Ra =
      Rb * Eigen::AngleAxisd(tu::kFootYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  // Oracle (b): a separate Data FK of the finger at the snapshot configuration.
  Eigen::VectorXd q(rig.model.nq);
  for (int i = 0; i < rig.model.nq; ++i) {
    q[i] = snap.q[i];
  }
  pinocchio::Data data(rig.model);
  pinocchio::framesForwardKinematics(rig.model, data, q);
  const pinocchio::SE3& oMb = data.oMf[rig.model.getFrameId("panda_leftfinger")];
  // Guards: the two frames are apart and both yaws are well defined.
  ASSERT_GE(std::hypot(pa[0] - oMb.translation()[0], pa[1] - oMb.translation()[1]), 0.02);
  ASSERT_GE(std::hypot(Ra(0, 0), Ra(1, 0)), 0.5);
  ASSERT_GE(std::hypot(oMb.rotation()(0, 0), oMb.rotation()(1, 0)), 0.5);
  // The FK is of the snapshot, not of q_default.
  Eigen::VectorXd qd = q;
  for (std::size_t i = 0; i < rig.q_default.size(); ++i) {
    qd[7 + i] = rig.q_default[i];
  }
  pinocchio::Data data_d(rig.model);
  pinocchio::framesForwardKinematics(rig.model, data_d, qd);
  const Eigen::Vector3d pb_default =
      data_d.oMf[rig.model.getFrameId("panda_leftfinger")].translation();
  ASSERT_GE((pb_default.head<2>() - oMb.translation().head<2>()).norm(), 1e-3);

  const double yaw_a = std::atan2(Ra(1, 0), Ra(0, 0));
  const double yaw_b = std::atan2(oMb.rotation()(1, 0), oMb.rotation()(0, 0));
  const SwingConfig swing = tu::ShippedSwing();
  const ContactSchedule& w = rig.scheduler.schedule();
  for (std::size_t c = 0; c < 2; ++c) {
    const double ox = c == 0 ? pa[0] : oMb.translation()[0];
    const double oy = c == 0 ? pa[1] : oMb.translation()[1];
    const double oyaw = c == 0 ? yaw_a : yaw_b;
    // PR A's plan on the window's own stretches, the one under way closed at SwingStart.
    std::array<SwingInterval, kMaxSwingsPerContact> ivs{};
    const std::size_t n = SwingIntervals(w, c, ivs);
    ASSERT_LE(n, kMaxSwingsPerContact);
    std::vector<SwingTrajectory> plans;
    for (std::size_t i = 0; i < n; ++i) {
      SwingInterval iv = ivs[i];
      if (iv.open_start) {
        double t_lo = 0.0;
        bool clamped = false;
        ASSERT_TRUE(rig.scheduler.SwingStart(c, w.t0, t_lo, clamped));
        EXPECT_FALSE(clamped);
        iv.lift_off = t_lo;
        iv.open_start = false;
      }
      SwingTrajectory s = SwingTrajectory::Stance(0.0);
      ASSERT_EQ(BuildSwing(iv, swing, 0.0, s), SwingError::kNoError);
      plans.push_back(s);
    }
    int swing_nodes = 0;
    for (int k = 0; k < kNodes; ++k) {
      SCOPED_TRACE("contact " + std::to_string(c) + " node " + std::to_string(k));
      const StageReference& st = h.stages[k];
      const double* pose = &st.contact_pose[7 * c];
      ASSERT_NEAR(pose[0], ox, kTol);
      ASSERT_NEAR(pose[1], oy, kTol);
      ASSERT_EQ(pose[3], 0.0);
      ASSERT_EQ(pose[4], 0.0);
      ASSERT_NEAR(pose[5], std::sin(0.5 * oyaw), kTol);
      ASSERT_NEAR(pose[6], std::cos(0.5 * oyaw), kTol);
      const bool stance = (st.mode & (1U << c)) != 0;
      if (stance) {
        ASSERT_EQ(pose[2], 0.0);  // the terrain, bitwise
        ASSERT_EQ(st.contact_twist[6 * c + 2], 0.0);
        ASSERT_EQ(st.contact_acceleration[6 * c + 2], 0.0);
        ASSERT_EQ(st.impact_proximity[c], 1.0);
      } else {
        ++swing_nodes;
        std::size_t pick = 0;
        for (std::size_t i = 1; i < n; ++i) {
          if (ivs[i].lift_off <= st.time) {
            pick = i;
          }
        }
        const SwingSample sample = plans[pick].Evaluate(st.time);
        ASSERT_NEAR(pose[2], sample.z, kTol);
        ASSERT_NEAR(st.contact_twist[6 * c + 2], sample.z_dot, kTol);
        ASSERT_NEAR(st.contact_acceleration[6 * c + 2], sample.z_ddot, kTol);
        ASSERT_NEAR(st.impact_proximity[c], sample.impact, kTol);
        ASSERT_GE(pose[2], -0.001 - kTol);  // never below the touch-down height
      }
      for (int i = 0; i < 6; ++i) {
        if (i != 2) {
          ASSERT_EQ(st.contact_twist[6 * c + i], 0.0);
          ASSERT_EQ(st.contact_acceleration[6 * c + i], 0.0);
        }
      }
    }
    EXPECT_GT(swing_nodes, 0);
  }
}

// Walking 2 s with a Build every tick: the same absolute instant must read the
// same swing (the stretch under way is closed at its real lift-off, so it does
// not get re-planned every tick); just before touch-down the sample stays
// bounded. dt = 1/64 and t0 = 0 keep the node times exact across ticks.
TEST(StageReferenceBuilder, ConsecutiveBuildsAgreeOnOverlappingNodes) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  Rig rig;
  const double dt = 1.0 / 64.0;
  ASSERT_EQ(LoadReferenceConfig(tu::ReferenceYaml(rig.model, rig.q_default, false, {}),
                                tu::ActuatedJointNames(rig.model), rig.cfg),
            ReferenceError::kNoError);
  ASSERT_EQ(rig.builder.Init(rig.handler, rig.cfg, tu::ShippedSwing(), FlatSurface(0.0), 65, dt),
            ReferenceError::kNoError);
  tu::WalkFrom(rig.scheduler, 0.0);
  const MPCStateSnapshot snap = rig.Snapshot();
  const FilteredCommand cmd = tu::RegisteredCommand();
  // Registered bound: the largest |ż| PR A's plan reaches on this gait's
  // stretches (0.13 s and 0.15 s swings, scaled).
  double max_rate = 0.0;
  for (const double T : {0.13, 0.15}) {
    SwingTrajectory s = SwingTrajectory::Stance(0.0);
    ASSERT_EQ(BuildSwing(SwingInterval{0.0, T, false, false}, tu::ShippedSwing(), 0.0, s),
              SwingError::kNoError);
    for (int i = 0; i <= 10000; ++i) {
      max_rate = std::max(max_rate, std::fabs(s.Evaluate(T * i / 10000.0).z_dot));
    }
  }
  auto prev = std::make_unique<HorizonReference>();
  int compared = 0;
  int touchdown_checks = 0;
  for (int tick = 0; tick <= 128; ++tick) {
    const double t = tick * dt;
    ASSERT_EQ(rig.scheduler.Advance(t, cmd, PlanarVelocity{}), GaitSchedulerError::kNoError);
    ASSERT_EQ(rig.scheduler.current_gait(), 1U);
    ASSERT_FALSE(rig.scheduler.switch_pending());
    ASSERT_EQ(rig.builder.Build(rig.scheduler, snap, cmd), ReferenceError::kNoError);
    const HorizonReference& h = rig.builder.active();
    if (tick > 0) {
      for (std::uint32_t k = 1; k < prev->n_stages; ++k) {
        const StageReference& a = prev->stages[k];
        const StageReference& b = h.stages[k - 1];
        ASSERT_EQ(std::memcmp(&a.time, &b.time, sizeof(double)), 0) << tick << " " << k;
        for (std::size_t c = 0; c < 2; ++c) {
          ASSERT_NEAR(a.contact_pose[7 * c + 2], b.contact_pose[7 * c + 2], kTol)
              << tick << " " << k;
          ASSERT_NEAR(a.contact_twist[6 * c + 2], b.contact_twist[6 * c + 2], kTol)
              << tick << " " << k;
          ASSERT_NEAR(a.impact_proximity[c], b.impact_proximity[c], kTol) << tick << " " << k;
        }
        ++compared;
      }
    }
    *prev = h;
    // A Build 1e-6 s before a touch-down: node 0 is still in the air, bounded.
    double t_event = 0.0;
    std::uint32_t after = 0;
    if (NextEvent(rig.scheduler.schedule(), t, t_event, after) && after == 0b11U &&
        t_event - 1e-6 > t && t_event - 1e-6 < t + dt) {
      ASSERT_EQ(rig.scheduler.Advance(t_event - 1e-6, cmd, PlanarVelocity{}),
                GaitSchedulerError::kNoError);
      ASSERT_EQ(rig.builder.Build(rig.scheduler, snap, cmd), ReferenceError::kNoError);
      const StageReference& st = rig.builder.active().stages[0];
      const std::size_t c = (st.mode & 1U) == 0 ? 0 : 1;
      ASSERT_EQ(st.mode & (1U << c), 0U);
      ASSERT_TRUE(std::isfinite(st.contact_twist[6 * c + 2]));
      ASSERT_TRUE(std::isfinite(st.contact_acceleration[6 * c + 2]));
      ASSERT_LE(std::fabs(st.contact_twist[6 * c + 2]), max_rate + kTol);
      ++touchdown_checks;
      // Back on the tick grid for the next comparison.
      ASSERT_EQ(rig.scheduler.Advance(t + dt, cmd, PlanarVelocity{}), GaitSchedulerError::kNoError);
      ASSERT_EQ(rig.builder.Build(rig.scheduler, snap, cmd), ReferenceError::kNoError);
      *prev = rig.builder.active();
      ++tick;
    }
  }
  EXPECT_GT(compared, 1000);
  EXPECT_GT(touchdown_checks, 0);
}

// ── Wrench ────────────────────────────────────────────────────────────────────

TEST(StageReferenceBuilder, StanceContactsShareTheWeightAndFlightNodesCarryNone) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  Rig rig;
  rig.Walk(kT0);
  const double weight = pinocchio::computeTotalMass(rig.model) * 9.81;
  ASSERT_EQ(rig.builder.Build(rig.scheduler, rig.Snapshot(), tu::RegisteredCommand()),
            ReferenceError::kNoError);
  const WbLayout& layout = rig.handler.layout();
  ASSERT_EQ(layout.contact_u_offset[0], 0);
  ASSERT_EQ(layout.contact_u_offset[1], 6);
  bool single = false, both = false;
  for (int k = 0; k < kNodes; ++k) {
    const StageReference& st = rig.builder.active().stages[k];
    SCOPED_TRACE(k);
    for (std::size_t c = 0; c < 2; ++c) {
      const auto off = static_cast<std::size_t>(layout.contact_u_offset[c]);
      const int dim = layout.contact_dim[c];
      const bool stance = (st.mode & (1U << c)) != 0;
      for (int i = 0; i < dim; ++i) {
        if (i == 2 && stance) {
          ASSERT_NEAR(st.wrench[off + 2], weight / st.n_stance, 1e-12 * weight);
        } else {
          ASSERT_EQ(st.wrench[off + static_cast<std::size_t>(i)], 0.0);
        }
      }
    }
    single = single || st.n_stance == 1;
    both = both || st.n_stance == 2;
    for (std::size_t i = 9; i < kMaxContactVars; ++i) {
      ASSERT_EQ(st.wrench[i], 0.0);
    }
  }
  EXPECT_TRUE(single && both);
  // Flight (hop): every wrench entry zero on the flight nodes.
  rig.scheduler.RequestGait(2);
  ASSERT_EQ(rig.scheduler.Advance(kT0 + 0.01, tu::RegisteredCommand(), PlanarVelocity{}),
            GaitSchedulerError::kNoError);
  const double t_s = rig.scheduler.switch_time();
  ASSERT_EQ(rig.scheduler.Advance(t_s + 0.05, tu::RegisteredCommand(), PlanarVelocity{}),
            GaitSchedulerError::kNoError);
  ASSERT_EQ(rig.scheduler.current_gait(), 2U);
  ASSERT_EQ(rig.builder.Build(rig.scheduler, rig.Snapshot(), tu::RegisteredCommand()),
            ReferenceError::kNoError);
  int flight = 0;
  for (int k = 0; k < kNodes; ++k) {
    const StageReference& st = rig.builder.active().stages[k];
    if (st.mode == 0) {
      ++flight;
      for (const double w : st.wrench) {
        ASSERT_EQ(w, 0.0) << k;
      }
      ASSERT_EQ(st.n_stance, 0U);
    }
  }
  EXPECT_GT(flight, 0);
}

// A switch into hop completing at t0 puts contact b in swing with no lift-off
// to plan from (SwingStart clamps at the take-over): the open-start plan
// stands — node 0 sits at the apex, descending to touch-down.
TEST(StageReferenceBuilder, AClampedSwingStartKeepsTheOpenStartPlan) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  Rig rig;
  rig.Walk(kT0);
  rig.scheduler.RequestGait(2);
  ASSERT_EQ(rig.scheduler.Advance(kT0 + 0.01, tu::RegisteredCommand(), PlanarVelocity{}),
            GaitSchedulerError::kNoError);
  const double t_s = rig.scheduler.switch_time();
  ASSERT_EQ(rig.scheduler.Advance(t_s, tu::RegisteredCommand(), PlanarVelocity{}),
            GaitSchedulerError::kNoError);
  ASSERT_EQ(rig.scheduler.current_gait(), 2U);
  double t_lo = 0.0;
  bool clamped = false;
  ASSERT_TRUE(rig.scheduler.SwingStart(1, t_s, t_lo, clamped));
  ASSERT_TRUE(clamped);
  ASSERT_EQ(rig.builder.Build(rig.scheduler, rig.Snapshot(), tu::RegisteredCommand()),
            ReferenceError::kNoError);
  const StageReference& st = rig.builder.active().stages[0];
  ASSERT_EQ(st.mode & 0b10U, 0U);
  EXPECT_NEAR(st.contact_pose[7 + 2], (0.0 - 0.001) + 0.08, kTol);  // z_td + h
  EXPECT_NEAR(st.contact_twist[6 + 2], 0.0, kTol);
  EXPECT_NEAR(st.impact_proximity[1], 0.005, kTol);
}

// ── Arm swing ─────────────────────────────────────────────────────────────────

double TilePhase(double anchor, double period, double t) {
  long k = 0;
  while (anchor + static_cast<double>(k + 1) * period <= t) {
    ++k;
  }
  while (anchor + static_cast<double>(k) * period > t) {
    --k;
  }
  return (t - (anchor + static_cast<double>(k) * period)) / period;
}

TEST(StageReferenceBuilder, ArmSwingFollowsTheGaitPhaseAndSparesTheTerminalNode) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  const auto idx2 = 7 + 1;  // panda_joint2 is the second joint after the root
  const auto idx4 = 7 + 3;
  Rig off;
  off.Walk(kT0, false);
  ASSERT_EQ(off.builder.Build(off.scheduler, off.Snapshot(), tu::RegisteredCommand()),
            ReferenceError::kNoError);
  Rig on;
  on.Walk(kT0, true);
  ASSERT_EQ(on.builder.Build(on.scheduler, on.Snapshot(), tu::RegisteredCommand()),
            ReferenceError::kNoError);
  GaitPhaseInfo info{};
  ASSERT_TRUE(on.scheduler.PhaseInfo(info));
  ASSERT_FALSE(info.pending);
  bool moved = false;
  for (int k = 0; k < kNodes; ++k) {
    SCOPED_TRACE(k);
    const StageReference& a = off.builder.active().stages[k];
    const StageReference& b = on.builder.active().stages[k];
    for (int i = 7; i < 16; ++i) {
      ASSERT_EQ(std::memcmp(&a.q[i], &on.q_default[i - 7], sizeof(double)), 0);
      if (i != idx2 && i != idx4) {
        ASSERT_EQ(std::memcmp(&b.q[i], &on.q_default[i - 7], sizeof(double)), 0);
      }
    }
    if (k == kNodes - 1) {
      ASSERT_EQ(std::memcmp(&b.q[idx2], &on.q_default[1], sizeof(double)), 0);
      ASSERT_EQ(std::memcmp(&b.q[idx4], &on.q_default[3], sizeof(double)), 0);
      continue;
    }
    const double phi = TilePhase(info.anchor, info.period, b.time);
    const double dq = 0.15 * std::sin(2.0 * std::numbers::pi * (phi - 0.15)) * 0.5;
    ASSERT_NEAR(b.q[idx2] - on.q_default[1], dq, kTol);
    ASSERT_NEAR(b.q[idx4] - on.q_default[3], -dq, kTol);
    moved = moved || std::fabs(dq) > 1e-3;
  }
  EXPECT_TRUE(moved);
  // A pending switch: nodes at or after t_s take the target gait's phase.
  on.scheduler.RequestGait(2);
  ASSERT_EQ(on.scheduler.Advance(kT0 + 0.02, tu::RegisteredCommand(), PlanarVelocity{}),
            GaitSchedulerError::kNoError);
  ASSERT_TRUE(on.scheduler.PhaseInfo(info));
  ASSERT_TRUE(info.pending);
  ASSERT_LT(info.t_switch, kT0 + 0.02 + 1.0);
  ASSERT_EQ(on.builder.Build(on.scheduler, on.Snapshot(), tu::RegisteredCommand()),
            ReferenceError::kNoError);
  int after = 0;
  for (int k = 0; k < kNodes - 1; ++k) {
    const StageReference& b = on.builder.active().stages[k];
    const double phi = b.time >= info.t_switch
                           ? TilePhase(info.next_anchor, info.next_period, b.time)
                           : TilePhase(info.anchor, info.period, b.time);
    const double dq = 0.15 * std::sin(2.0 * std::numbers::pi * (phi - 0.15)) * 0.5;
    ASSERT_NEAR(b.q[idx2] - on.q_default[1], dq, kTol) << k;
    after += b.time >= info.t_switch;
  }
  EXPECT_GT(after, 0);
  // The swung value never leaves the joint's limits: q_default at the upper
  // limit of joint2 (+1), a positive swing is clamped there, a negative one moves.
  {
    Rig lim;
    lim.q_default[1] = lim.model.upperPositionLimit[8];
    lim.Walk(kT0, true);
    ASSERT_EQ(lim.builder.Build(lim.scheduler, lim.Snapshot(), tu::RegisteredCommand()),
              ReferenceError::kNoError);
    bool clamped_once = false, moved_down = false;
    for (int k = 0; k < kNodes - 1; ++k) {
      const double q2 = lim.builder.active().stages[k].q[idx2];
      ASSERT_LE(q2, lim.model.upperPositionLimit[8]) << k;
      clamped_once = clamped_once || q2 == lim.model.upperPositionLimit[8];
      moved_down = moved_down || q2 < lim.model.upperPositionLimit[8] - 1e-3;
    }
    EXPECT_TRUE(clamped_once);
    EXPECT_TRUE(moved_down);
  }
  // A single-mode gait (stance) swings nothing.
  Rig stance;
  ASSERT_EQ(LoadReferenceConfig(tu::ReferenceYaml(stance.model, stance.q_default, true,
                                                  {{"panda_joint2", 1.0}, {"panda_joint4", -1.0}}),
                                tu::ActuatedJointNames(stance.model), stance.cfg),
            ReferenceError::kNoError);
  ASSERT_EQ(stance.builder.Init(stance.handler, stance.cfg, tu::ShippedSwing(), FlatSurface(0.0),
                                kNodes, kDt),
            ReferenceError::kNoError);
  stance.scheduler.SetPolicyEnabled(false);
  ASSERT_EQ(stance.scheduler.Advance(kT0, tu::RegisteredCommand(), PlanarVelocity{}),
            GaitSchedulerError::kNoError);
  ASSERT_EQ(stance.builder.Build(stance.scheduler, stance.Snapshot(), tu::RegisteredCommand()),
            ReferenceError::kNoError);
  for (int k = 0; k < kNodes; ++k) {
    ASSERT_EQ(std::memcmp(&stance.builder.active().stages[k].q[idx2], &stance.q_default[1],
                          sizeof(double)),
              0);
  }
}

// ── Slots ─────────────────────────────────────────────────────────────────────

TEST(StageReferenceBuilder, SlotsSwapOnSuccessAndStayOnFailure) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  Rig rig;
  rig.Walk(kT0);
  // Init: the active slot is all zero (the other is written before it is ever seen).
  const auto zero = std::make_unique<HorizonReference>();
  EXPECT_EQ(std::memcmp(zero.get(), &rig.builder.active(), sizeof(HorizonReference)), 0);
  const MPCStateSnapshot snap = rig.Snapshot();
  const FilteredCommand cmd = tu::RegisteredCommand();
  ASSERT_EQ(rig.builder.Build(rig.scheduler, snap, cmd), ReferenceError::kNoError);
  const HorizonReference* first = &rig.builder.active();
  const auto copy1 = std::make_unique<HorizonReference>(*first);
  EXPECT_EQ(copy1->reference_id, 1U);
  ASSERT_EQ(rig.builder.Build(rig.scheduler, snap, cmd), ReferenceError::kNoError);
  const HorizonReference* second = &rig.builder.active();
  EXPECT_NE(first, second);
  EXPECT_EQ(second->reference_id, 2U);
  // Build #1's bytes are still there (the other slot), untouched.
  EXPECT_EQ(std::memcmp(copy1.get(), first, sizeof(HorizonReference)), 0);
  // Same input: everything but reference_id identical.
  auto copy2 = std::make_unique<HorizonReference>(*second);
  copy2->reference_id = copy1->reference_id;
  EXPECT_EQ(std::memcmp(copy1.get(), copy2.get(), sizeof(HorizonReference)), 0);
  ASSERT_EQ(rig.builder.Build(rig.scheduler, snap, cmd), ReferenceError::kNoError);
  EXPECT_EQ(&rig.builder.active(), first);  // alternating
  EXPECT_EQ(rig.builder.active().reference_id, 3U);

  // The failure set: error, active bytes and reference_id unchanged.
  const auto before = std::make_unique<HorizonReference>(rig.builder.active());
  const HorizonReference* active = &rig.builder.active();
  MPCStateSnapshot bad_q = snap;
  bad_q.q[9] = kNaN;
  EXPECT_EQ(rig.builder.Build(rig.scheduler, bad_q, cmd), ReferenceError::kNonFiniteInput);
  MPCStateSnapshot bad_v = snap;
  bad_v.v[2] = kInf;
  EXPECT_EQ(rig.builder.Build(rig.scheduler, bad_v, cmd), ReferenceError::kNonFiniteInput);
  MPCStateSnapshot bad_dim = snap;
  bad_dim.nv = 14;
  EXPECT_EQ(rig.builder.Build(rig.scheduler, bad_dim, cmd), ReferenceError::kSnapshotMismatch);
  MPCStateSnapshot bad_quat = snap;
  bad_quat.q[6] *= 1.01;
  EXPECT_EQ(rig.builder.Build(rig.scheduler, bad_quat, cmd), ReferenceError::kSnapshotMismatch);
  FilteredCommand bad_cmd = cmd;
  bad_cmd.omega_z = kNaN;
  EXPECT_EQ(rig.builder.Build(rig.scheduler, snap, bad_cmd), ReferenceError::kNonFiniteInput);
  MPCStateSnapshot huge_v = snap;
  for (int i = 3; i < 6; ++i) {
    huge_v.v[i] = std::numeric_limits<double>::max();  // finite; (R·ω)_z overflows the yaw knot
  }
  EXPECT_EQ(rig.builder.Build(rig.scheduler, huge_v, cmd), ReferenceError::kNonFiniteInput);
  // A window too short for the horizon (a scheduler with a 0.3 s horizon).
  GaitScheduler shortw;
  ASSERT_EQ(shortw.Init(rig.table, 0.3), GaitSchedulerError::kNoError);
  ASSERT_EQ(shortw.Advance(kT0, cmd, PlanarVelocity{}), GaitSchedulerError::kNoError);
  EXPECT_EQ(rig.builder.Build(shortw, snap, cmd), ReferenceError::kWindowTooShort);
  GaitScheduler never;
  ASSERT_EQ(never.Init(rig.table, 1.0), GaitSchedulerError::kNoError);
  EXPECT_EQ(rig.builder.Build(never, snap, cmd), ReferenceError::kNoSchedule);
  StageReferenceBuilder uninit;
  EXPECT_EQ(uninit.Build(rig.scheduler, snap, cmd), ReferenceError::kNotInitialised);
  EXPECT_EQ(&rig.builder.active(), active);
  EXPECT_EQ(std::memcmp(before.get(), active, sizeof(HorizonReference)), 0);
  EXPECT_EQ(rig.builder.active().reference_id, 3U);
  static_assert(noexcept(rig.builder.Build(rig.scheduler, snap, cmd)));
}

// ── YAML ──────────────────────────────────────────────────────────────────────

const std::vector<std::string> kAB = {"joint_a", "joint_b"};

constexpr const char* kRefYaml = R"(
intermediate_fraction: 0.7
q_default:
  joint_a: 0.1
  joint_b: -0.2
arm_swing:
  enabled: true
  gain: 0.15
  phase_offset: 0.15
  joints:
    - {name: joint_a, sign: 1}
    - {name: joint_b, sign: -1}
)";

ReferenceConfig Sentinel() {
  ReferenceConfig c;
  c.intermediate_fraction = -1.0;
  return c;
}

TEST(ReferenceConfigYaml, TheTemplateAndTheRegisteredValuesLoad) {
  ReferenceConfig c = Sentinel();
  ASSERT_EQ(LoadReferenceConfig(YAML::Load(kRefYaml), kAB, c), ReferenceError::kNoError);
  EXPECT_EQ(c.intermediate_fraction, 0.7);
  ASSERT_EQ(c.q_default.size(), 2U);
  EXPECT_EQ(c.q_default[0].first, "joint_a");
  EXPECT_EQ(c.q_default[1].second, -0.2);
  EXPECT_TRUE(c.arm_swing_enabled);
  EXPECT_EQ(c.arm_swing_gain, 0.15);
  ASSERT_EQ(c.arm_swing_joints.size(), 2U);
  EXPECT_EQ(c.arm_swing_joints[1].sign, -1.0);
  const YAML::Node root =
      YAML::LoadFile(std::string(RTC_MPC_CONFIG_DIR) + "/reference_default.yaml");
  ReferenceConfig t = Sentinel();
  EXPECT_EQ(LoadReferenceConfig(root["reference"], kAB, t), ReferenceError::kNoError);
  EXPECT_FALSE(t.arm_swing_enabled);
}

TEST(ReferenceConfigYaml, RefusalsHaveAcceptedTwinsAndLeaveTheConfigUntouched) {
  struct Case {
    const char* what;
    std::function<void(YAML::Node&)> edit;
    ReferenceError expected;
  };

  const std::vector<Case> cases = {
      {"unknown key", [](YAML::Node& n) { n["fraction"] = 0.7; }, ReferenceError::kUnknownKey},
      {"unknown arm key", [](YAML::Node& n) { n["arm_swing"]["amplitude"] = 0.1; },
       ReferenceError::kUnknownKey},
      {"missing joint", [](YAML::Node& n) { n["q_default"].remove("joint_b"); },
       ReferenceError::kMissingJoint},
      {"unknown joint", [](YAML::Node& n) { n["q_default"]["joint_c"] = 0.0; },
       ReferenceError::kUnknownJoint},
      {"non-finite q", [](YAML::Node& n) { n["q_default"]["joint_a"] = YAML::Load(".nan"); },
       ReferenceError::kNonFinite},
      {"non-finite gain", [](YAML::Node& n) { n["arm_swing"]["gain"] = YAML::Load(".inf"); },
       ReferenceError::kNonFinite},
      {"enabled, no joints", [](YAML::Node& n) { n["arm_swing"]["joints"] = YAML::Load("[]"); },
       ReferenceError::kEmptyArmSwingJoints},
      {"sign 2", [](YAML::Node& n) { n["arm_swing"]["joints"][0]["sign"] = 2; },
       ReferenceError::kInvalidSign},
      {"sign 0", [](YAML::Node& n) { n["arm_swing"]["joints"][0]["sign"] = 0; },
       ReferenceError::kInvalidSign},
      {"fraction 0", [](YAML::Node& n) { n["intermediate_fraction"] = 0.0; },
       ReferenceError::kInvalidFraction},
      {"fraction 1", [](YAML::Node& n) { n["intermediate_fraction"] = 1.0; },
       ReferenceError::kInvalidFraction},
      {"disabled, unknown swing joint",
       [](YAML::Node& n) {
         n["arm_swing"]["enabled"] = false;
         n["arm_swing"]["joints"][0]["name"] = "joint_c";
       },
       ReferenceError::kUnknownJoint},
      {"swing joint twice", [](YAML::Node& n) { n["arm_swing"]["joints"][1]["name"] = "joint_a"; },
       ReferenceError::kDuplicateJoint},
      {"missing arm_swing", [](YAML::Node& n) { n.remove("arm_swing"); },
       ReferenceError::kMissingKey},
      {"enabled not a bool", [](YAML::Node& n) { n["arm_swing"]["enabled"] = "maybe"; },
       ReferenceError::kWrongType},
      {"q_default a list", [](YAML::Node& n) { n["q_default"] = YAML::Load("[0.1, 0.2]"); },
       ReferenceError::kNotAMap},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.what);
    YAML::Node n = YAML::Load(kRefYaml);
    c.edit(n);
    ReferenceConfig cfg = Sentinel();
    EXPECT_EQ(LoadReferenceConfig(n, kAB, cfg), c.expected);
    EXPECT_EQ(cfg.intermediate_fraction, -1.0);
  }
  // Accepted twins.
  {
    YAML::Node n = YAML::Load(kRefYaml);
    n["arm_swing"]["enabled"] = false;
    n["arm_swing"]["joints"] = YAML::Load("[]");
    ReferenceConfig cfg = Sentinel();
    EXPECT_EQ(LoadReferenceConfig(n, kAB, cfg), ReferenceError::kNoError);
  }
  {
    YAML::Node n = YAML::Load(kRefYaml);
    n["intermediate_fraction"] = 0.6;
    ReferenceConfig cfg = Sentinel();
    EXPECT_EQ(LoadReferenceConfig(n, kAB, cfg), ReferenceError::kNoError);
    EXPECT_EQ(cfg.intermediate_fraction, 0.6);
  }
  ReferenceConfig none = Sentinel();
  EXPECT_EQ(LoadReferenceConfig(YAML::Node(), kAB, none), ReferenceError::kNotAMap);
  EXPECT_EQ(none.intermediate_fraction, -1.0);
}

// intermediate_fraction is read, not hardcoded: at 0.6 the first knot moves.
TEST(StageReferenceBuilder, TheIntermediateFractionIsTheConfigs) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  Rig rig;
  rig.Walk(kT0);
  YAML::Node y = tu::ReferenceYaml(rig.model, rig.q_default, false, {});
  y["intermediate_fraction"] = 0.6;
  ReferenceConfig cfg;
  ASSERT_EQ(LoadReferenceConfig(y, tu::ActuatedJointNames(rig.model), cfg),
            ReferenceError::kNoError);
  StageReferenceBuilder b;
  ASSERT_EQ(b.Init(rig.handler, cfg, tu::ShippedSwing(), FlatSurface(0.0), kNodes, kDt),
            ReferenceError::kNoError);
  ASSERT_EQ(b.Build(rig.scheduler, rig.Snapshot(), tu::RegisteredCommand()),
            ReferenceError::kNoError);
  // knot 1 at 0.6 H: p + 0.6·(v_m + v_c)/2, with v_c = R_z(1.3)·(0.5, 0.1).
  const double vcx = std::cos(1.3) * 0.5 - std::sin(1.3) * 0.1;
  const double vcy = std::sin(1.3) * 0.5 + std::cos(1.3) * 0.1;
  EXPECT_NEAR(b.active().stages[30].q[0], 0.3 + 0.6 * (kVmX + vcx) / 2.0, 1e-9);
  EXPECT_NEAR(b.active().stages[30].q[1], -0.2 + 0.6 * (kVmY + vcy) / 2.0, 1e-9);
  EXPECT_NEAR(b.active().stages[50].q[0], 0.3 + 0.6 * (kVmX + vcx) / 2.0 + 0.4 * vcx, 1e-9);
}

}  // namespace
}  // namespace rtc::mpc
