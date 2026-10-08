/// @file test_clik_pink_reference.cpp
/// @brief The two-frame CLIK against pink's solution of the same problems
///        (E2-F04, #636; formulation §2.2).
///
/// pink is an independent published implementation of a velocity-level QP IK
/// with a world frame task, a relative frame task, a posture task and
/// configuration / velocity limits — the structure of the multi-frame
/// Compute(). Its solutions are recorded in golden/clik_pink_reference.inc by
/// golden/gen_pink_reference.py (whose docstring has the mapping between the
/// two formulations and the versions); this suite feeds the recorded inputs to
/// the CLIK and compares the joint velocities. Nothing here depends on pink
/// being installed, so the suite never skips.
///
/// Two tiers, because one thing does differ — pink's task Jacobian carries the
/// Jlog6 factor of the exact error derivative and the CLIK's velocity law is
/// first order:
///   tier 1  pink as published, at pose errors of 1 mm / 1 mrad, where the
///           factor is I + O(error): the tolerance scales with the velocity;
///   tier 2  pink with the factor removed, at 0.2 m / 0.5 rad: the same
///           problem, compared at the solvers' own accuracy.

#include "panda_fixture.hpp"
#include "rtc_tsid/kinematics/clik_reference.hpp"
#include "tree_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace rtc::tsid {
namespace {

using Clik = ClikReferenceGenerator;
using Vec6 = Eigen::Matrix<double, 6, 1>;

struct PinkCase {
  int problem;  // 0 = the 9-DoF arm, 1 = the 17-DoF tree
  int tier;
  int nv;
  int active;  // limit rows active in pink's solution
  double q[17];
  double q_posture[17];
  double target_world[12];  // rotation (row-major), translation
  double target_rel[12];
  double v[17];
};

#include "golden/clik_pink_reference.inc"

struct Problem {
  std::shared_ptr<const pinocchio::Model> model;
  const char* world_frame;
  const char* rel_frame;
  const char* rel_root;
};

[[nodiscard]] Problem ProblemOf(int index) {
  if (index == 0) {
    return {test::SharedPandaModel(), "panda_hand", "panda_link5", "panda_link1"};
  }
  static const std::shared_ptr<const pinocchio::Model> tree = test::LoadTreeModel();
  return {tree, "tip_b", "tip_a", "torso"};
}

[[nodiscard]] pinocchio::SE3 Pose(const double* p) {
  pinocchio::SE3 pose;
  pose.rotation() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(p);
  pose.translation() = Eigen::Map<const Eigen::Vector3d>(p + 9);
  return pose;
}

[[nodiscard]] std::string Sci(double x) {
  std::array<char, 32> buf{};
  std::snprintf(buf.data(), buf.size(), "%.3e", x);
  return buf.data();
}

/// The CLIK's velocity for one recorded case. `rel_as_world` hands the
/// relative task's target to a world task instead — the wrong problem, for
/// the control below.
[[nodiscard]] Eigen::VectorXd SolveCase(const PinkCase& c, bool rel_as_world, bool& ok) {
  const Problem problem = ProblemOf(c.problem);
  const int nv = problem.model->nv;
  EXPECT_EQ(nv, c.nv);

  PinocchioCache cache;
  ContactManagerConfig contact_cfg;
  contact_cfg.max_contacts = 0;
  cache.Init(problem.model, rtc::tsid::ContactFrameIds(contact_cfg));
  const int world_frame =
      cache.RegisterFrame(problem.world_frame, problem.model->getFrameId(problem.world_frame));
  const int rel_frame =
      cache.RegisterFrame(problem.rel_frame, problem.model->getFrameId(problem.rel_frame));
  const int rel_root =
      cache.RegisterFrame(problem.rel_root, problem.model->getFrameId(problem.rel_root));

  Clik::Config cfg;
  for (int i = 0; i < nv; ++i) {
    cfg.arm_v_idx.push_back(i);
  }
  cfg.posture_groups = {{cfg.arm_v_idx, kPinkPostureCost * kPinkPostureCost}};
  cfg.damping_sq = kPinkDamping;
  cfg.q_min = problem.model->lowerPositionLimit;
  cfg.q_max = problem.model->upperPositionLimit;
  cfg.v_limit_per_joint = problem.model->velocityLimit;
  cfg.max_frame_tasks = 2;
  cfg.relative_tasks = true;
  Clik gen;
  gen.Init(nv, cfg);
  EXPECT_TRUE(gen.SetPostureGroupGain(0, kPinkPostureGain / kPinkDt));

  std::array<Clik::FrameTask, 2> tasks;
  tasks[0].kind = Clik::TaskKind::kSe3;
  tasks[0].frame_idx = world_frame;
  tasks[0].placement_des = Pose(c.target_world);
  tasks[0].gain = Vec6::Constant(kPinkWorldGain / kPinkDt);
  tasks[0].weight = kPinkWorldCost * kPinkWorldCost;
  tasks[1].kind = Clik::TaskKind::kSe3;
  tasks[1].frame_idx = rel_frame;
  tasks[1].base_frame_idx = rel_as_world ? -1 : rel_root;
  tasks[1].placement_des = Pose(c.target_rel);
  tasks[1].gain = Vec6::Constant(kPinkRelGain / kPinkDt);
  tasks[1].weight = kPinkRelCost * kPinkRelCost;

  const Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(c.q, nv);
  const Eigen::VectorXd q_posture = Eigen::Map<const Eigen::VectorXd>(c.q_posture, nv);
  cache.Update(q, Eigen::VectorXd::Zero(nv));
  Clik::MultiFrameInput in;
  in.tasks = tasks;
  in.q_posture_des = &q_posture;
  in.dt = kPinkDt;
  ok = gen.Compute(cache, in);
  return gen.VRef();
}

struct TierResult {
  int cases{0};
  int active{0};
  double worst_error{0.0};         // max |v_clik − v_pink| over the tier
  double worst_over_tolerance{0};  // max of error / tolerance
  double worst_control{0.0};       // the same error for the wrong problem
};

/// `tolerance(v_pink_norm)` is the tier's tolerance on the max-abs error.
template <typename Tolerance>
[[nodiscard]] TierResult RunTier(int tier, Tolerance&& tolerance) {
  TierResult r;
  for (const PinkCase& c : kPinkCases) {
    if (c.tier != tier) {
      continue;
    }
    const Eigen::VectorXd v_pink = Eigen::Map<const Eigen::VectorXd>(c.v, c.nv);
    bool ok = false;
    const Eigen::VectorXd v = SolveCase(c, false, ok);
    EXPECT_TRUE(ok) << "problem " << c.problem << " tier " << tier << " case " << r.cases;
    const double error = (v - v_pink).cwiseAbs().maxCoeff();
    r.worst_error = std::max(r.worst_error, error);
    r.worst_over_tolerance = std::max(r.worst_over_tolerance, error / tolerance(v_pink.norm()));
    bool control_ok = false;
    const Eigen::VectorXd v_control = SolveCase(c, true, control_ok);
    if (control_ok) {
      r.worst_control = std::max(r.worst_control, (v_control - v_pink).cwiseAbs().maxCoeff());
    }
    r.active += c.active;
    ++r.cases;
  }
  return r;
}

TEST(ClikPinkReferenceTest, RecordedSetCoversBothModelsAndBothTiers) {
  std::array<std::array<int, 3>, 2> count{};  // [problem][tier]
  for (const PinkCase& c : kPinkCases) {
    ASSERT_TRUE(c.problem == 0 || c.problem == 1);
    ASSERT_TRUE(c.tier == 1 || c.tier == 2);
    ++count[static_cast<size_t>(c.problem)][static_cast<size_t>(c.tier)];
  }
  for (const auto& per_problem : count) {
    EXPECT_GE(per_problem[1], 4);
    EXPECT_GE(per_problem[2], 4);
  }
}

TEST(ClikPinkReferenceTest, SmallErrorsMatchPinkAsPublished) {
  // pink's Jacobian is Jlog6(error)·J and the CLIK's is J: at 1 mm / 1 mrad
  // the factor is within ~1e-3 of the identity, so the two velocities agree to
  // that fraction of the velocity, plus the solvers' accuracy.
  const TierResult r = RunTier(1, [](double v_norm) { return 1e-5 + 2e-3 * v_norm; });
  RecordProperty("cases", r.cases);
  RecordProperty("worst_error", Sci(r.worst_error));
  RecordProperty("worst_error_over_tolerance", Sci(r.worst_over_tolerance));
  EXPECT_LE(r.worst_over_tolerance, 1.0);
  EXPECT_GT(r.active, 0) << "premise: some case sits on a limit";
}

TEST(ClikPinkReferenceTest, LargeErrorsMatchFirstOrderPink) {
  // The same problem on both sides: only the two solvers' accuracy is left.
  const TierResult r = RunTier(2, [](double) { return 1e-5; });
  RecordProperty("cases", r.cases);
  RecordProperty("worst_error", Sci(r.worst_error));
  RecordProperty("worst_control_error", Sci(r.worst_control));
  EXPECT_LE(r.worst_over_tolerance, 1.0);
  EXPECT_GT(r.active, 0) << "premise: some case sits on a limit";
  // The reference tells a relative task from a world one: the relative
  // target handed to a world task is off by orders of magnitude.
  EXPECT_GT(r.worst_control, 1e-2);
}

}  // namespace
}  // namespace rtc::tsid
