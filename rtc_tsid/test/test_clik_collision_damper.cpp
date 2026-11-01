/// @file test_clik_collision_damper.cpp
/// @brief ClikReferenceGenerator's collision damper (Config::collision_damper,
///        #642) on the synthetic 17-DoF tree.
///
/// Judged against something the damper did not produce — the test's own
/// CapsuleWorkspace over the same spec, read at the COMMAND state the solve
/// is evaluated at:
///   1. off, or on with every pair outside d_influence, the outputs are the
///      ones without it (bit for bit on the far case);
///   2. one hand sent through the other: without the damper the command
///      state's pair distance goes inside d_safe, with it the distance never
///      does (to the solver's tolerance) and the rows are active and binding;
///   3. a pair already inside d_safe is asked to separate at ξ — no faster —
///      so the distance grows by about ξ·dt a tick;
///   4. a pair whose gradient vanishes on every column keeps an inert row
///      (two spheres kept coincident by their links);
///   5. a row the velocity box cannot meet fails the call on its failure
///      branch (damper_rows_violated or an unconverged solve), outputs held;
///   6. Init refuses an inconsistent configuration by name.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#pragma GCC diagnostic pop

#include "clik_test_util.hpp"
#include "rtc_tsid/kinematics/clik_reference.hpp"
#include "rtc_urdf_bridge/collision/capsule_workspace.hpp"
#include "tree_fixture.hpp"

namespace rtc::tsid {
namespace {

using Clik = ClikReferenceGenerator;
using Vec6 = Eigen::Matrix<double, 6, 1>;
namespace rub = rtc_urdf_bridge;

constexpr int kNv = test::kTreeNv;
constexpr double kDt = 0.002;
constexpr double kDInfluence = 0.08;
constexpr double kDSafe = 0.02;
constexpr double kXi = 0.2;

// The tree's arms: upper arm (link 3 → joint 4, 0.24 m), forearm (link 5 →
// joint 6, 0.16 m), hand (link 7 to the tip, 0.1 m); the torso up to the arm
// roots. tree_17dof.urdf's offsets.
const char* const kSpec = R"(
collision_capsules:
  capsules:
    - {name: torso, link: torso, p0: [0, 0, -0.1], p1: [0, 0, 0.35], radius: 0.1}
    - {name: a_upper, link: arm_a_link_3, p0: [0, 0, 0], p1: [0.02, 0, -0.24], radius: 0.05}
    - {name: a_fore, link: arm_a_link_5, p0: [0, 0, 0], p1: [0, 0, -0.16], radius: 0.04}
    - {name: a_hand, link: arm_a_link_7, p0: [0, 0, 0], p1: [0, 0, -0.1], radius: 0.04}
    - {name: b_upper, link: arm_b_link_3, p0: [0, 0, 0], p1: [0.02, 0, -0.24], radius: 0.05}
    - {name: b_fore, link: arm_b_link_5, p0: [0, 0, 0], p1: [0, 0, -0.16], radius: 0.04}
    - {name: b_hand, link: arm_b_link_7, p0: [0, 0, 0], p1: [0, 0, -0.1], radius: 0.04}
  pairs:
    - [a_upper, b_upper]
    - [a_upper, b_fore]
    - [a_upper, b_hand]
    - [a_fore, b_upper]
    - [a_fore, b_fore]
    - [a_fore, b_hand]
    - [a_hand, b_upper]
    - [a_hand, b_fore]
    - [a_hand, b_hand]
    - [a_fore, torso]
    - [a_hand, torso]
    - [b_fore, torso]
    - [b_hand, torso]
)";

class ClikDamperTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = test::LoadTreeModel();
    test::InitContactFreeCache(cache_, model_);
    tip_a_ = cache_.RegisterFrame("tip_a", model_->getFrameId("tip_a"));
    tip_b_ = cache_.RegisterFrame("tip_b", model_->getFrameId("tip_b"));
    all_ = test::VelocityIndices(*model_, "trunk_");
    for (const char* p : {"arm_a_", "arm_b_"}) {
      const std::vector<int> idx = test::VelocityIndices(*model_, p);
      all_.insert(all_.end(), idx.begin(), idx.end());
    }
    ASSERT_EQ(static_cast<int>(all_.size()), kNv);
    q_home_ = test::TreeHomePosture(*model_);
    spec_ = rub::CapsuleModel::ParseYaml(kSpec, "damper");
    // The test's own measure of the pairs, at whatever state it is asked.
    data_ = std::make_unique<pinocchio::Data>(*model_);
    ASSERT_EQ(ws_.Bind(spec_, *model_, {}), rub::CapsuleBindStatus::kOk) << ws_.BindError();
  }

  [[nodiscard]] Clik::Config BaseConfig() const {
    Clik::Config cfg;
    cfg.arm_v_idx = all_;
    cfg.damping_sq = 1e-4;
    cfg.v_limit = 3.0;
    cfg.w_arm = 1e-3;
    cfg.w_hand = 0.0;
    cfg.evaluate_at_command = true;
    cfg.relative_tasks = true;
    cfg.max_frame_tasks = 2;
    cfg.max_iter = 50;
    return cfg;
  }

  [[nodiscard]] Clik::Config DamperConfig(double d_i = kDInfluence, double d_s = kDSafe,
                                          double xi = kXi) const {
    Clik::Config cfg = BaseConfig();
    cfg.collision_damper.enabled = true;
    cfg.collision_damper.capsules = spec_;
    cfg.collision_damper.model = model_;
    cfg.collision_damper.d_influence = d_i;
    cfg.collision_damper.d_safe = d_s;
    cfg.collision_damper.xi = xi;
    return cfg;
  }

  /// tip_a toward `target` (world), tip_b held where it is at q_home.
  std::array<Clik::FrameTask, 2> TasksToward(const Eigen::Vector3d& target) {
    cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
    const auto& rf_a = cache_.registered_frames[static_cast<size_t>(tip_a_)];
    const auto& rf_b = cache_.registered_frames[static_cast<size_t>(tip_b_)];
    std::array<Clik::FrameTask, 2> tasks;
    tasks[0].kind = Clik::TaskKind::kSe3;
    tasks[0].frame_idx = tip_a_;
    tasks[0].placement_des = rf_a.oMf;
    tasks[0].placement_des.translation() = target;
    tasks[0].gain = Vec6::Constant(8.0);
    tasks[0].fb_lin_max = 0.5;
    tasks[0].fb_ang_max = 1.0;
    tasks[1].kind = Clik::TaskKind::kSe3;
    tasks[1].frame_idx = tip_b_;
    tasks[1].placement_des = rf_b.oMf;
    tasks[1].gain = Vec6::Constant(8.0);
    return tasks;
  }

  [[nodiscard]] Clik::MultiFrameInput Input(std::span<const Clik::FrameTask> tasks) const {
    Clik::MultiFrameInput in;
    in.tasks = tasks;
    in.q_posture_des = &q_home_;
    in.dt = kDt;
    return in;
  }

  /// The smallest pair distance of the spec at q, by the test's workspace.
  double MinDistance(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(*model_, *data_, q);
    EXPECT_TRUE(ws_.UpdatePlacements(*model_, *data_));
    EXPECT_EQ(ws_.EvaluateDistances(), 0U);
    return ws_.MinDistance();
  }

  double PairDistance(const Eigen::VectorXd& q, std::size_t pair) {
    pinocchio::forwardKinematics(*model_, *data_, q);
    EXPECT_TRUE(ws_.UpdatePlacements(*model_, *data_));
    EXPECT_EQ(ws_.EvaluateDistances(), 0U);
    return ws_.Distance(pair);
  }

  struct Run {
    double min_distance{std::numeric_limits<double>::infinity()};
    int ticks_with_rows{0};
    int ticks_binding{0};
    int failures{0};
    Eigen::VectorXd q_end;
  };

  /// The closed loop in command mode: the cache at the command state, the
  /// solve, the command state moved to QRef (an ideal servo).
  Run Loop(Clik& gen, const Clik::MultiFrameInput& in, int ticks) {
    Run r;
    Eigen::VectorXd q = q_home_;
    Eigen::VectorXd v = Eigen::VectorXd::Zero(kNv);
    for (int k = 0; k < ticks; ++k) {
      cache_.Update(q, v);
      const bool ok = gen.Compute(cache_, in);
      if (!ok) {
        ++r.failures;
      }
      q = gen.QRef();
      v = gen.VRef();
      r.min_distance = std::min(r.min_distance, MinDistance(q));
      r.ticks_with_rows += gen.LastSolve().damper_rows > 0 ? 1 : 0;
      r.ticks_binding += gen.LastSolve().damper_rows_binding > 0 ? 1 : 0;
    }
    r.q_end = q;
    return r;
  }

  std::shared_ptr<pinocchio::Model> model_;
  PinocchioCache cache_;
  int tip_a_{-1};
  int tip_b_{-1};
  std::vector<int> all_;
  Eigen::VectorXd q_home_;
  std::shared_ptr<const rub::CapsuleModel> spec_;
  std::unique_ptr<pinocchio::Data> data_;
  rub::CapsuleWorkspace ws_;
};

}  // namespace

// ── 1. Off, and on with nothing near, are the same solve ─────────────────────

TEST_F(ClikDamperTest, OffAndFarPairsLeaveTheSolveUntouched) {
  // A reach that keeps the arms apart: tip_a a little outward.
  cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
  const Eigen::Vector3d away =
      cache_.registered_frames[static_cast<size_t>(tip_a_)].oMf.translation() +
      Eigen::Vector3d(0.05, 0.05, 0.0);
  const auto tasks = TasksToward(away);
  const Clik::MultiFrameInput in = Input(tasks);
  Clik off;
  off.Init(kNv, BaseConfig());
  Clik on;
  on.Init(kNv, DamperConfig());
  Eigen::VectorXd q = q_home_;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(kNv);
  for (int k = 0; k < 50; ++k) {
    cache_.Update(q, v);
    ASSERT_TRUE(off.Compute(cache_, in)) << k;
    cache_.Update(q, v);
    ASSERT_TRUE(on.Compute(cache_, in)) << k;
    ASSERT_GE(MinDistance(q), kDInfluence) << "the far case must stay outside d_influence";
    EXPECT_EQ(on.LastSolve().damper_rows, 0);
    EXPECT_TRUE(std::isfinite(on.LastSolve().damper_min_distance));
    EXPECT_TRUE(std::isnan(off.LastSolve().damper_min_distance));
    EXPECT_TRUE(test::BitEqual(on.VRef(), off.VRef())) << "tick " << k;
    EXPECT_TRUE(test::BitEqual(on.QRef(), off.QRef())) << "tick " << k;
    q = on.QRef();
    v = on.VRef();
  }
}

// ── 2. One hand through the other ────────────────────────────────────────────

TEST_F(ClikDamperTest, TheDamperKeepsTheCommandStateOutsideDSafeWhereTheControlGoesIn) {
  cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
  // tip_a sent to where tip_b's hand is, and a little beyond: the hands meet.
  const Eigen::Vector3d p_b =
      cache_.registered_frames[static_cast<size_t>(tip_b_)].oMf.translation();
  const Eigen::Vector3d p_a =
      cache_.registered_frames[static_cast<size_t>(tip_a_)].oMf.translation();
  const Eigen::Vector3d target = p_b + 0.2 * (p_b - p_a);
  const auto tasks = TasksToward(target);
  const Clik::MultiFrameInput in = Input(tasks);

  Clik off;
  off.Init(kNv, BaseConfig());
  const Run control = Loop(off, in, 600);
  EXPECT_LT(control.min_distance, kDSafe) << "the control never came close — the gate is vacuous";

  Clik on;
  on.Init(kNv, DamperConfig());
  const Run damped = Loop(on, in, 600);
  EXPECT_EQ(damped.failures, 0);
  EXPECT_GE(damped.min_distance, kDSafe - 1e-3) << "the command state went inside d_safe";
  EXPECT_GE(damped.ticks_with_rows, 1);
  EXPECT_GE(damped.ticks_binding, 1) << "no tick sat on a damper row — the rows did nothing";
  std::cout << "[damper] control min distance " << control.min_distance
            << " m, damped min distance " << damped.min_distance << " m, ticks with rows "
            << damped.ticks_with_rows << ", binding " << damped.ticks_binding << "\n";
}

// ── 3. Inside d_safe the row asks for ξ (d_s − d)/(d_i − d_s), capped at ξ ──

TEST_F(ClikDamperTest, APairInsideDSafeSeparatesAtTheRowsSpeedAndNoFasterThanXi) {
  // Bring the hands together without the damper, then turn it on from there
  // with the task still pulling them together: the nearest pair's row binds,
  // so the first damped tick separates it by the row's right-hand side times
  // dt — ξ (d_s − d)/(d_i − d_s) on a wide band, ξ itself on a band so narrow
  // that the formula would ask for more (the cap).
  cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
  const Eigen::Vector3d p_b =
      cache_.registered_frames[static_cast<size_t>(tip_b_)].oMf.translation();
  const Eigen::Vector3d p_a =
      cache_.registered_frames[static_cast<size_t>(tip_a_)].oMf.translation();
  const auto tasks = TasksToward(p_b + 0.2 * (p_b - p_a));
  const Clik::MultiFrameInput in = Input(tasks);
  Clik off;
  off.Init(kNv, BaseConfig());
  Eigen::VectorXd q = q_home_;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(kNv);
  for (int k = 0; k < 1000 && MinDistance(q) > kDSafe - 0.01; ++k) {
    cache_.Update(q, v);
    ASSERT_TRUE(off.Compute(cache_, in)) << k;
    q = off.QRef();
    v = off.VRef();
  }
  const double d0 = MinDistance(q);
  ASSERT_LT(d0, kDSafe - 0.005) << "the hands never met";

  struct Band {
    const char* what;
    double d_influence;
  };

  for (const Band& band : {Band{"wide band (uncapped)", kDInfluence},
                           Band{"narrow band (capped at xi)", kDSafe + 0.005}}) {
    Clik on;
    on.Init(kNv, DamperConfig(band.d_influence, kDSafe, kXi));
    cache_.Update(q, Eigen::VectorXd::Zero(kNv));
    ASSERT_TRUE(on.Compute(cache_, in)) << band.what;
    EXPECT_GE(on.LastSolve().damper_rows, 1) << band.what;
    EXPECT_GE(on.LastSolve().damper_rows_binding, 1) << band.what;
    const double rhs = std::min(kXi, kXi * (kDSafe - d0) / (band.d_influence - kDSafe));
    const double d1 = MinDistance(on.QRef());
    // The nearest pair's row binds: Δd = rhs·dt to first order (the
    // linearisation's second order over one 2 ms step is below 1e-5).
    EXPECT_NEAR(d1 - d0, rhs * kDt, 0.1 * rhs * kDt + 1e-5) << band.what << " d0 " << d0;
    std::cout << "[damper " << band.what << "] d0 " << d0 << " Δd " << d1 - d0 << " rhs·dt "
              << rhs * kDt << " (ξ·dt " << kXi * kDt << ")\n";
  }
}

// ── 4. A vanishing gradient keeps an inert row ──────────────────────────────

TEST_F(ClikDamperTest, AnUnmeasurablePairIsInert) {
  // Two spheres kept coincident by their links: arm_a_link_6's at the
  // origin of joint arm_a_7, arm_a_link_7's at its own origin. Their distance
  // is −(r1 + r2) at every q, with no normal — the row stays inert and the
  // solve is the one without it.
  const pinocchio::JointIndex j7 = model_->getJointId("arm_a_7");
  const Eigen::Vector3d c6 = model_->jointPlacements[j7].translation();
  const std::string yaml = std::string(R"(
collision_capsules:
  capsules:
    - {name: s6, link: arm_a_link_6, p0: [)") +
                           std::to_string(c6.x()) + ", " + std::to_string(c6.y()) + ", " +
                           std::to_string(c6.z()) + "], p1: [" + std::to_string(c6.x()) + ", " +
                           std::to_string(c6.y()) + ", " + std::to_string(c6.z()) +
                           R"(], radius: 0.03}
    - {name: s7, link: arm_a_link_7, p0: [0, 0, 0], p1: [0, 0, 0], radius: 0.03}
  pairs: [[s6, s7]]
)";
  Clik::Config cfg = DamperConfig();
  cfg.collision_damper.capsules = rub::CapsuleModel::ParseYaml(yaml, "coincident");
  Clik on;
  on.Init(kNv, cfg);
  Clik off;
  off.Init(kNv, BaseConfig());
  cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
  const Eigen::Vector3d away =
      cache_.registered_frames[static_cast<size_t>(tip_a_)].oMf.translation() +
      Eigen::Vector3d(0.05, 0.05, 0.0);
  const auto tasks = TasksToward(away);
  const Clik::MultiFrameInput in = Input(tasks);
  cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
  ASSERT_TRUE(on.Compute(cache_, in));
  // Coincident spheres: coal's nearest points coincide and its normal is zero,
  // which the workspace reports as an invalid pair (no distance, no gradient)
  // — the row stays inert and the diagnostics count the pair.
  EXPECT_EQ(on.LastSolve().damper_pairs_invalid, 1);
  EXPECT_TRUE(std::isnan(on.LastSolve().damper_min_distance));
  EXPECT_EQ(on.LastSolve().damper_rows, 0) << "a pair with no column to move stays inert";
  cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
  ASSERT_TRUE(off.Compute(cache_, in));
  EXPECT_TRUE(test::BitEqual(on.VRef(), off.VRef()));
}

// ── 5. A row the box cannot meet fails the call ─────────────────────────────

TEST_F(ClikDamperTest, ARowTheBoxCannotMeetFailsTheCallOnItsFailureBranch) {
  // The hands brought inside d_safe, then a damper asking them apart at 50 m/s
  // under a 0.5 rad/s box: no admissible velocity. The call fails on the
  // failure branch (q_ref held, v_ref = 0), and says which block broke.
  cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
  const Eigen::Vector3d p_b =
      cache_.registered_frames[static_cast<size_t>(tip_b_)].oMf.translation();
  const Eigen::Vector3d p_a =
      cache_.registered_frames[static_cast<size_t>(tip_a_)].oMf.translation();
  const auto tasks = TasksToward(p_b + 0.2 * (p_b - p_a));
  const Clik::MultiFrameInput in = Input(tasks);
  Clik off;
  off.Init(kNv, BaseConfig());
  Eigen::VectorXd q = q_home_;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(kNv);
  for (int k = 0; k < 1000 && MinDistance(q) > kDSafe - 0.01; ++k) {
    cache_.Update(q, v);
    ASSERT_TRUE(off.Compute(cache_, in)) << k;
    q = off.QRef();
    v = off.VRef();
  }
  ASSERT_LT(MinDistance(q), kDSafe - 0.005);
  Clik::Config cfg = DamperConfig(kDInfluence, kDSafe, 50.0);
  cfg.v_limit = 0.5;
  Clik on;
  on.Init(kNv, cfg);
  cache_.Update(q, Eigen::VectorXd::Zero(kNv));
  const bool ok = on.Compute(cache_, in);
  EXPECT_FALSE(ok);
  const auto& s = on.LastSolve();
  EXPECT_TRUE(s.reached_solve);
  EXPECT_GE(s.damper_rows, 1);
  EXPECT_TRUE(s.damper_rows_violated || !s.converged) << "status " << s.status;
  EXPECT_FALSE(s.accel_rows_violated);
  for (int i = 0; i < kNv; ++i) {
    EXPECT_EQ(on.VRef()[i], 0.0) << i;
    EXPECT_EQ(on.QRef()[i], q[i]) << i;
  }
}

// ── 6. Init refuses an inconsistent damper by name ───────────────────────────

TEST_F(ClikDamperTest, InitRefusesAnInconsistentDamper) {
  const auto refusal = [&](const Clik::Config& cfg) -> std::string {
    try {
      Clik gen;
      gen.Init(kNv, cfg);
    } catch (const std::runtime_error& e) {
      return e.what();
    }
    return {};
  };
  {
    Clik::Config cfg = DamperConfig();
    cfg.collision_damper.capsules.reset();
    EXPECT_NE(refusal(cfg).find("capsule spec"), std::string::npos);
  }
  {
    Clik::Config cfg = DamperConfig();
    cfg.collision_damper.model.reset();
    EXPECT_NE(refusal(cfg).find("the model"), std::string::npos);
  }
  {
    Clik::Config cfg = DamperConfig(0.02, 0.02, 0.2);  // d_i == d_s
    EXPECT_NE(refusal(cfg).find("d_influence - d_safe >="), std::string::npos);
  }
  {
    // A band below the floor: 1/(d_i − d_s) would overflow the rhs.
    Clik::Config cfg = DamperConfig(0.02 + 1e-9, 0.02, 0.2);
    EXPECT_NE(refusal(cfg).find("d_influence - d_safe >="), std::string::npos);
  }
  {
    Clik::Config cfg = DamperConfig(0.08, 0.02, 0.0);  // xi 0
    EXPECT_NE(refusal(cfg).find("xi > 0"), std::string::npos);
  }
  {
    Clik::Config cfg = DamperConfig(0.08, std::numeric_limits<double>::quiet_NaN(), 0.2);
    EXPECT_FALSE(refusal(cfg).empty());
  }
  {
    // Off but with a field set: refused, like brake_margin without the bound.
    Clik::Config cfg = BaseConfig();
    cfg.collision_damper.xi = 0.2;
    EXPECT_NE(refusal(cfg).find("not enabled"), std::string::npos);
  }
  {
    // A spec the model refuses: the workspace's message, by name.
    Clik::Config cfg = DamperConfig();
    cfg.collision_damper.capsules = rub::CapsuleModel::ParseYaml(R"(
collision_capsules:
  capsules:
    - {name: ghost, link: no_such_link, p0: [0, 0, 0], p1: [0, 0, 0.1], radius: 0.03}
    - {name: a7, link: arm_a_link_7, p0: [0, 0, 0], p1: [0, 0, 0.1], radius: 0.03}
  pairs: [[ghost, a7]])",
                                                                 "bad");
    EXPECT_NE(refusal(cfg).find("no_such_link"), std::string::npos);
  }
  {
    // A model of another size.
    Clik::Config cfg = DamperConfig();
    auto other = std::make_shared<pinocchio::Model>(*model_);
    other->addJoint(0, pinocchio::JointModelRX(), pinocchio::SE3::Identity(), "extra");
    cfg.collision_damper.model = other;
    EXPECT_NE(refusal(cfg).find("nv"), std::string::npos);
  }
}

}  // namespace rtc::tsid
