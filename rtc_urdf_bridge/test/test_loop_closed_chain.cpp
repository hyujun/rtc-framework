// ── test_loop_closed_chain — closure error / projection / Jc rank·cond /
//    constraintDynamics smoke / 통합 loader (Phase 7·8·9·10 + §4d·§4e) /
//    strict-vs-acceptance tolerance 분리 (#250) ─────────────────────────────
#include "closure_test_fixtures.hpp"
#include "rtc_urdf_bridge/closed_chain_model.hpp"
#include "rtc_urdf_bridge/constraint_builder.hpp"
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
#include <limits>
#include <stdexcept>

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
  EXPECT_TRUE(ccm.q_ref_acceptable);  // strict 충족 ⇒ 수용 (#250)
  pinocchio::Data data(ccm.model);
  const auto errors = rub::ComputeClosureErrors(ccm.model, data, ccm.constraints, ccm.q_ref);
  EXPECT_LT(errors.front().norm, 1e-7);
}

// ── #250: 로드 타임 q_ref 파이프라인의 rejected 경로 wiring — 기하적으로 닫을 수 없는
//    closure (c2 anchor 를 링크 길이 합 밖으로 이동) 는 q_ref_converged / q_ref_acceptable
//    둘 다 false 로 보고돼 소비자가 q_ref 를 정상 결과로 오인하지 않는다.
//    (floor(acceptable-but-not-converged) 분류 자체는 solver 레벨 테스트가 고정한다 —
//    four_bar 는 joint_ab 가 y축이라 z-불일치가 전역 보상 가능하고, crank_rocker 는
//    neutral 이 사영 정체점이라 loader 진입점(neutral 시작)으로는 floor 를 만들 수 없다.)
TEST(LoopClosedChain, BuildClosedChainDataReportsRejectionWhenLoopCannotClose) {
  const rub::ClosureSpec spec = rub::LoadClosureYaml(TestUrdfPath("four_bar.closure.yaml"));

  pinocchio::Model model = LoadCcm().model;
  // 링크 길이 합(≈0.6 m)을 초과하는 anchor 이동 → 어떤 형상에서도 닫히지 않는다.
  model.frames[model.getFrameId("c2")].placement.translation().z() += 10.0;

  const rub::ClosedChainData data = rub::BuildClosedChainData(model, spec);
  EXPECT_FALSE(data.q_ref_converged);
  EXPECT_FALSE(data.q_ref_acceptable);
  EXPECT_TRUE(data.q_ref.allFinite());
}

// ═══ #250: strict convergence vs acceptance 분리 ═════════════════════════════
// residual floor fixture: FlooredCrankRocker(δ) — 평면 기구 closure anchor 의 z-불일치는
// 어떤 관절 운동으로도 보상 불가라 ‖φ‖ floor == δ (closure_test_fixtures.hpp 참조).

namespace {
// crank 를 비특이 구간(0.2)에 두고 전체 열 사영 (q_ref 시나리오와 동일한 진입점).
rub::ProjectionResult ProjectFrom(const rub::ClosedChainModel& cc, pinocchio::Data& data,
                                  double crank, const rub::ProjectionOptions& opts = {}) {
  Eigen::VectorXd q_init = pinocchio::neutral(cc.model);
  q_init[cc.model.idx_qs[cc.model.getJointId("j_crank")]] = crank;
  return rub::ProjectToConstraint(cc.model, data, cc.constraints, q_init, opts);
}
}  // namespace

// ── 상태 1 (strictly converged): floor 없는 기구 — converged 와 acceptable 둘 다 true. ──
TEST(LoopClosedChain, AcceptanceStrictConvergenceSetsBothFlags) {
  const rub::ClosedChainModel cc = rtc::test::CrankRocker();
  pinocchio::Data data(cc.model);
  const rub::ProjectionResult res = ProjectFrom(cc, data, 0.2);

  EXPECT_TRUE(res.converged);
  EXPECT_TRUE(res.acceptable);
  EXPECT_LT(res.final_error, 1e-10);
}

// ── 상태 2 (accepted with residual floor): strict 미달이지만 1e-6 이내 — 사용 가능. ──
//    stall 감지(#250 D3)로 max_iterations(100) 를 소진하지 않아야 한다 — floor 에서
//    개선율 ≈ 0 이라 조기 종료. (이게 없으면 floor 로봇의 모든 호출이 100-iter 를 태운다.)
TEST(LoopClosedChain, AcceptanceResidualFloorAcceptedNotConverged) {
  const rub::ClosedChainModel cc = rtc::test::FlooredCrankRocker(3e-8);
  pinocchio::Data data(cc.model);
  const rub::ProjectionResult res = ProjectFrom(cc, data, 0.2);

  EXPECT_FALSE(res.converged);
  EXPECT_TRUE(res.acceptable);
  EXPECT_NEAR(res.final_error, 3e-8, 3e-9);  // floor == 주입한 z-offset
  EXPECT_LT(res.iterations, 30) << "stall 미감지 — floor 에서 max_iterations 소진";
  EXPECT_TRUE(res.q.allFinite());
}

// ── 상태 3 (rejected): floor 가 acceptance(1e-6) 초과 — 둘 다 false. ─────────────
TEST(LoopClosedChain, AcceptanceResidualAboveAcceptanceRejected) {
  const rub::ClosedChainModel cc = rtc::test::FlooredCrankRocker(1e-4);
  pinocchio::Data data(cc.model);
  const rub::ProjectionResult res = ProjectFrom(cc, data, 0.2);

  EXPECT_FALSE(res.converged);
  EXPECT_FALSE(res.acceptable);
  EXPECT_GT(res.final_error, 1e-6);
}

// ── 회귀 (#250 핵심 금지사항): acceptance 는 stopping 기준이 아니다. ─────────────
//    초기 residual 이 이미 acceptance(1e-6) 미만이어도 solver 는 strict 를 향해 실제로
//    반복해 residual 을 유의미하게 개선해야 한다 — acceptance 를 solver tolerance 로
//    꽂으면 첫 검사에서 성공 처리되어 이 개선(실측 0.58 µm → 0.03 µm 상당)이 사라진다.
TEST(LoopClosedChain, AcceptanceIsNotAStoppingCriterion) {
  const rub::ClosedChainModel cc = rtc::test::FlooredCrankRocker(3e-8);
  pinocchio::Data data(cc.model);

  // floor 해를 만든 뒤 passive 를 미세 교란 → 초기 residual 을 (floor, acceptance) 구간에.
  const rub::ProjectionResult base = ProjectFrom(cc, data, 0.2);
  ASSERT_TRUE(base.acceptable);
  Eigen::VectorXd q_perturbed = base.q;
  q_perturbed[cc.model.idx_qs[cc.model.getJointId("j_coupler")]] += 1.5e-6;
  q_perturbed[cc.model.idx_qs[cc.model.getJointId("j_rocker")]] -= 1.5e-6;

  const rub::ConstraintKinematics kin0 =
      rub::ComputeConstraintKinematics(cc.model, data, cc.constraints, q_perturbed);
  const double initial = kin0.phi.norm();
  ASSERT_GT(initial, 1e-7) << "교란이 너무 작다 — 픽스처 무력";
  ASSERT_LT(initial, 1e-6) << "교란이 acceptance 를 넘었다 — 픽스처 무력";

  const rub::ProjectionResult res =
      rub::ProjectToConstraint(cc.model, data, cc.constraints, q_perturbed);
  EXPECT_GE(res.iterations, 1) << "초기 residual < acceptance 인데 반복 없이 반환";
  EXPECT_LT(res.final_error, initial / 5.0) << "projection 이 residual 을 개선하지 않았다";
  EXPECT_TRUE(res.acceptable);
}

// ── 비유한 residual 은 절대 acceptable 이 아니다. ────────────────────────────────
TEST(LoopClosedChain, AcceptanceNonFiniteResidualRejected) {
  const rub::ClosedChainModel cc = rtc::test::CrankRocker();
  pinocchio::Data data(cc.model);
  Eigen::VectorXd q_init = pinocchio::neutral(cc.model);
  q_init[cc.model.idx_qs[cc.model.getJointId("j_coupler")]] =
      std::numeric_limits<double>::quiet_NaN();

  const rub::ProjectionResult res =
      rub::ProjectToConstraint(cc.model, data, cc.constraints, q_init);
  EXPECT_FALSE(res.converged);
  EXPECT_FALSE(res.acceptable);
  EXPECT_FALSE(std::isfinite(res.final_error));
}

// ── #250 D4: CONTACT_6D 는 acceptance 미적용 — strict 로만 수용된다. ─────────────
//    6D residual 은 병진(m)·회전(rad) 혼합 norm 이라 1e-6 m 임계를 적용할 수 없다.
//    동일 기하·동일 z-불일치(3e-8)에서 3D 는 수용, 6D 는 거부됨을 고정한다.
//    (max_iterations=0 으로 solver 반복 없이 종료-후 분류만 검사 — 6D 로 닫는 crank_rocker
//    는 과구속이라 수렴 자체가 목적이 아니다.)
TEST(LoopClosedChain, AcceptanceContact6DStaysStrict) {
  const rub::ClosedChainModel cc = rtc::test::CrankRocker();

  // neutral 에서 c1(world [0.5,0,0])·c2(world [0.7,0,0]) 둘 다 자세 identity — c2 anchor 를
  // in-plane 으로 c1 에 겹치게 옮기고 z 로 3e-8 만 어긋내면 residual 이 정확히 [0,0,±3e-8].
  rub::ClosedChainInfo info;
  info.name = "pin_check";
  info.frame_1 = "c1";
  info.frame_2 = "c2";
  info.reference_frame = rub::LoopReferenceFrame::kLocal;

  rub::ProjectionOptions opts;
  opts.max_iterations = 0;  // 분류만 검사
  const Eigen::VectorXd q0 = pinocchio::neutral(cc.model);
  pinocchio::Data data(cc.model);

  for (const bool is_6d : {false, true}) {
    info.is_6d = is_6d;
    auto constraints = rub::BuildRigidConstraints(cc.model, {info});
    constraints.front().joint2_placement.translation() += Eigen::Vector3d(-0.2, 0.0, 3e-8);

    const rub::ProjectionResult res =
        rub::ProjectToConstraint(cc.model, data, constraints, q0, opts);
    EXPECT_FALSE(res.converged) << "is_6d=" << is_6d;
    EXPECT_NEAR(res.final_error, 3e-8, 3e-9) << "is_6d=" << is_6d;
    EXPECT_EQ(res.acceptable, !is_6d)
        << "is_6d=" << is_6d << " — 3D 는 수용, 6D 는 strict 유지여야 한다";
  }
}

// ── 옵션 validation: 비유한·비양수·acceptance<strict 는 invalid_argument. ─────────
TEST(LoopClosedChain, AcceptanceInvalidOptionsThrow) {
  const rub::ClosedChainModel cc = rtc::test::CrankRocker();
  pinocchio::Data data(cc.model);
  const Eigen::VectorXd q0 = pinocchio::neutral(cc.model);

  rub::ProjectionOptions bad_tol;
  bad_tol.tolerance = -1e-10;
  EXPECT_THROW((void)rub::ProjectToConstraint(cc.model, data, cc.constraints, q0, bad_tol),
               std::invalid_argument);

  rub::ProjectionOptions bad_acc;
  bad_acc.acceptance_tolerance = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((void)rub::ProjectToConstraint(cc.model, data, cc.constraints, q0, bad_acc),
               std::invalid_argument);

  rub::ProjectionOptions inverted;  // acceptance 가 strict 보다 작음 — 의미 역전
  inverted.tolerance = 1e-6;
  inverted.acceptance_tolerance = 1e-10;
  EXPECT_THROW((void)rub::ProjectToConstraint(cc.model, data, cc.constraints, q0, inverted),
               std::invalid_argument);
}
