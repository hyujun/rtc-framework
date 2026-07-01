// ── test_loop_closed_chain — closure error / projection / Jc rank·cond /
//    constraintDynamics smoke / 통합 loader (Phase 7·8·9·10 + §4d·§4e) ────────
#include "rtc_urdf_bridge/closed_chain_model.hpp"
#include "rtc_urdf_bridge/loop_projection.hpp"
#include "rtc_urdf_bridge/loop_verification.hpp"
#include "test_urdf_path.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/constrained-dynamics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <cmath>

namespace rub = rtc_urdf_bridge;
using rtc::test::TestUrdfPath;

namespace {
rub::ClosedChainModel LoadCcm() {
  return rub::BuildClosedChainModelFromExtendedUrdf(TestUrdfPath("four_bar_tree.urdf"),
                                                    TestUrdfPath("four_bar.closure.yaml"));
}
}  // namespace

// ── Phase 7: nominal q(=0) 에서 closure error 작음 (조립 상태로 설계) ──────────
TEST(LoopClosedChain, ClosureErrorSmallAtNominal) {
  const rub::ClosedChainModel ccm = LoadCcm();
  pinocchio::Data data(ccm.model);
  const Eigen::VectorXd q0 = pinocchio::neutral(ccm.model);

  const auto errors = rub::ComputeClosureErrors(ccm.model, data, ccm.constraints, q0);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors.front().dim, 3);
  EXPECT_LT(errors.front().norm, 1e-9);
}

// ── q0 는 대칭 조립 특이형상: x-translation row 가 0 → rank 결손 감지 ──────────
// (평면/대칭 4-bar 과구속의 실제 증상. §5b 특이형상 경고 대상.)
TEST(LoopClosedChain, NominalConfigDetectedAsSingular) {
  const rub::ClosedChainModel ccm = LoadCcm();
  pinocchio::Data data(ccm.model);
  const Eigen::VectorXd q0 = pinocchio::neutral(ccm.model);

  const rub::JacobianReport rep =
      rub::AnalyzeConstraintJacobian(ccm.model, data, ccm.constraints, q0);
  EXPECT_EQ(rep.rows, 3);
  EXPECT_LT(rep.smallest_singular_value, 1e-6);  // 특이 (rank < rows)
  EXPECT_LT(rep.rank, rep.rows);
}

// ── Phase 9: 일반(operating) 형상에서 Jc full-rank + Delassus 조건수 유한 ──────
TEST(LoopClosedChain, GenericConfigFullRankFiniteCondition) {
  const rub::ClosedChainModel ccm = LoadCcm();
  pinocchio::Data data(ccm.model);
  Eigen::VectorXd q(ccm.model.nq);
  q << 0.3, 0.4, -0.2, 0.5;  // 비대칭 일반 형상 (rank 는 q 만의 함수, 조립 불필요)

  const rub::JacobianReport rep =
      rub::AnalyzeConstraintJacobian(ccm.model, data, ccm.constraints, q);
  EXPECT_EQ(rep.rows, 3);
  EXPECT_EQ(rep.rank, 3);
  EXPECT_TRUE(rep.full_rank);
  EXPECT_GT(rep.smallest_singular_value, 1e-6);
  EXPECT_TRUE(std::isfinite(rep.delassus_condition));
}

// ── Phase 8: 임의 q → projection 후 ‖φ‖ < tol ───────────────────────────────
TEST(LoopClosedChain, PositionProjectionConverges) {
  const rub::ClosedChainModel ccm = LoadCcm();
  pinocchio::Data data(ccm.model);
  Eigen::VectorXd q_seed(ccm.model.nq);
  q_seed << 0.25, 0.0, 0.0, 0.0;  // joint_a 를 구동한 뒤 나머지를 사영

  rub::ProjectionOptions opts;
  opts.tolerance = 1e-10;
  opts.max_iterations = 200;
  const rub::ProjectionResult res =
      rub::ProjectToConstraint(ccm.model, data, ccm.constraints, q_seed, opts);

  EXPECT_TRUE(res.converged);
  EXPECT_LT(res.final_error, 1e-8);

  // 사영 결과에서 실제 closure error 도 작아야 한다.
  const auto errors = rub::ComputeClosureErrors(ccm.model, data, ccm.constraints, res.q);
  EXPECT_LT(errors.front().norm, 1e-7);
}

// ── Phase 8b: velocity projection — Jc v_proj ≈ 0 ────────────────────────────
TEST(LoopClosedChain, VelocityProjectionSatisfiesConstraint) {
  const rub::ClosedChainModel ccm = LoadCcm();
  pinocchio::Data data(ccm.model);
  Eigen::VectorXd q(ccm.model.nq);
  q << 0.3, 0.4, -0.2, 0.5;
  Eigen::VectorXd v(ccm.model.nv);
  v << 1.0, -0.5, 0.7, 0.2;

  const Eigen::VectorXd v_proj = rub::ProjectVelocity(ccm.model, data, ccm.constraints, q, v);
  const rub::ConstraintKinematics kin =
      rub::ComputeConstraintKinematics(ccm.model, data, ccm.constraints, q);
  EXPECT_LT((kin.Jc * v_proj).norm(), 1e-6);
}

// ── Phase 10: constraintDynamics smoke — NaN 없는 qdd 반환 ────────────────────
// 주의: neutral(=q0) 은 대칭 조립 특이형상 → KKT 특이 → NaN. 실제 operating 형상
// (joint_a 구동 후 loop-consistent 로 사영) 에서 평가한다.
TEST(LoopClosedChain, ConstraintDynamicsProducesFiniteAcceleration) {
  const rub::ClosedChainModel ccm = LoadCcm();
  pinocchio::Data data(ccm.model);

  // 비특이 operating 형상으로 사영
  Eigen::VectorXd q_seed(ccm.model.nq);
  q_seed << 0.25, 0.0, 0.0, 0.0;
  rub::ProjectionOptions opts;
  opts.tolerance = 1e-10;
  opts.max_iterations = 200;
  const rub::ProjectionResult proj =
      rub::ProjectToConstraint(ccm.model, data, ccm.constraints, q_seed, opts);
  ASSERT_TRUE(proj.converged);

  std::vector<pinocchio::RigidConstraintData> cdatas;
  cdatas.reserve(ccm.constraints.size());
  for (const auto& cm : ccm.constraints) {
    cdatas.emplace_back(cm);
  }
  pinocchio::initConstraintDynamics(ccm.model, data, ccm.constraints, cdatas);

  const Eigen::VectorXd v = Eigen::VectorXd::Zero(ccm.model.nv);
  const Eigen::VectorXd tau = Eigen::VectorXd::Zero(ccm.model.nv);
  pinocchio::constraintDynamics(ccm.model, data, proj.q, v, tau, ccm.constraints, cdatas);

  EXPECT_TRUE(data.ddq.allFinite());
  EXPECT_EQ(data.ddq.size(), ccm.model.nv);
}

// ── §4d/§4e: 통합 loader — actuated ids, q_ref loop-consistent ───────────────
TEST(LoopClosedChain, IntegratedLoaderExposesActuationAndConsistentQref) {
  const rub::ClosedChainModel ccm = LoadCcm();

  ASSERT_EQ(ccm.actuated_joint_ids.size(), 1u);
  EXPECT_EQ(ccm.actuated_joint_ids.front(), ccm.model.getJointId("joint_a"));

  EXPECT_TRUE(ccm.q_ref_converged);
  pinocchio::Data data(ccm.model);
  const auto errors = rub::ComputeClosureErrors(ccm.model, data, ccm.constraints, ccm.q_ref);
  EXPECT_LT(errors.front().norm, 1e-7);
}
