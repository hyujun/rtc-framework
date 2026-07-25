// ── test_loop_projection_passive — ProjectPassiveToConstraint (actuated 고정, ────
//    passive 만 사영). Extended-URDF 폐쇄 체인 시각화(actuated 스트림 → loop-consistent
//    full q)의 core 수학. crank_rocker(비특이 crank-rocker) + four_bar(대칭 특이) 픽스처. ──
#include "closure_test_fixtures.hpp"
#include "rtc_urdf_bridge/closed_chain_model.hpp"
#include "rtc_urdf_bridge/loop_projection.hpp"
#include "test_urdf_path.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace rub = rtc_urdf_bridge;
using rtc::test::CrankRocker;
using rtc::test::FourBar;

// ── 성공기준 1: actuated(j_crank) 를 고정한 채 passive 를 풀어 loop 를 닫는다. ───────
//    crank-rocker 의 비특이 구간에서 cold seed(neutral) 로도 수렴. actuated 는 불변.
TEST(LoopProjectionPassive, FixesActuatedAndClosesLoop) {
  const rub::ClosedChainModel cc = CrankRocker();
  pinocchio::Data data(cc.model);
  const auto q_idx = cc.model.idx_qs[cc.model.getJointId("j_crank")];

  // 비특이 조립 구간 (probe 실측). neutral(=crank 0) 은 특이라 제외.
  for (const double crank : {-0.2, -0.1, 0.1, 0.2, 0.3}) {
    Eigen::VectorXd q_init = pinocchio::neutral(cc.model);
    q_init[q_idx] = crank;
    const rub::ProjectionResult res = rub::ProjectPassiveToConstraint(
        cc.model, data, cc.constraints, q_init, cc.actuated_joint_ids);

    EXPECT_TRUE(res.converged) << "crank=" << crank;
    EXPECT_LT(res.final_error, 1e-8) << "crank=" << crank;
    EXPECT_TRUE(res.q.allFinite()) << "crank=" << crank;
    // actuated tangent=0 → integrate 시 j_crank 는 정확히 불변.
    EXPECT_NEAR(res.q[q_idx], crank, 1e-12) << "crank=" << crank;
  }
}

// ── 성공기준 2: warm-start(직전 해 seed) 로 cold-seed 가 놓치는 구간까지 연속 추종. ──
//    crank 를 0.05 스텝으로 sweep. 각 스텝 수렴 + 연속(‖Δq‖ 유계, branch-flip 없음).
TEST(LoopProjectionPassive, WarmStartContinuity) {
  const rub::ClosedChainModel cc = CrankRocker();
  pinocchio::Data data(cc.model);
  const auto q_idx = cc.model.idx_qs[cc.model.getJointId("j_crank")];

  // 첫 점은 cold-수렴 가능한 crank=0.05 에서 시작.
  Eigen::VectorXd q = pinocchio::neutral(cc.model);
  const double start = 0.05;
  q[q_idx] = start;
  rub::ProjectionResult prev =
      rub::ProjectPassiveToConstraint(cc.model, data, cc.constraints, q, cc.actuated_joint_ids);
  ASSERT_TRUE(prev.converged);
  q = prev.q;

  int steps = 0;
  for (double crank = start + 0.05; crank <= 0.6001; crank += 0.05) {
    q[q_idx] = crank;  // 직전 해 + actuated 갱신 = warm seed
    const rub::ProjectionResult res =
        rub::ProjectPassiveToConstraint(cc.model, data, cc.constraints, q, cc.actuated_joint_ids);

    // cold-seed 는 crank≈0.35/0.55 에서 실패하지만 warm 은 통과해야 한다.
    EXPECT_TRUE(res.converged) << "crank=" << crank;
    EXPECT_LT(res.final_error, 1e-8) << "crank=" << crank;
    EXPECT_NEAR(res.q[q_idx], crank, 1e-12) << "crank=" << crank;
    // 연속성: 0.05 rad actuated 스텝에 대한 전체 q 변화가 유계 (branch-flip 시 큰 점프).
    EXPECT_LT((res.q - q).norm(), 0.3) << "crank=" << crank << " (discontinuous jump)";

    q = res.q;
    ++steps;
  }
  EXPECT_GE(steps, 11);  // 0.10..0.60 전 구간 통과
}

namespace {

// 조립된 시작 형상 (crank=start 에서 cold 수렴 가능한 비특이 구간).
Eigen::VectorXd AssembledAt(const rub::ClosedChainModel& cc, pinocchio::Data& data, double crank) {
  const auto q_idx = cc.model.idx_qs[cc.model.getJointId("j_crank")];
  Eigen::VectorXd q = pinocchio::neutral(cc.model);
  q[q_idx] = crank;
  const rub::ProjectionResult res =
      rub::ProjectPassiveToConstraint(cc.model, data, cc.constraints, q, cc.actuated_joint_ids);
  EXPECT_TRUE(res.converged) << "시작 형상 조립 실패 (crank=" << crank << ")";
  return res.q;
}

// 물리적으로 옳은 분기의 기준해: q_from 에서 crank=to 까지 **미세** continuation.
// (분기는 residual 이 아니라 homotopy 경로가 결정하므로, 충분히 잘게 쪼갠 경로가 ground truth.)
Eigen::VectorXd FineContinuation(const rub::ClosedChainModel& cc, pinocchio::Data& data,
                                 const Eigen::VectorXd& q_from, double to) {
  constexpr double kFineStep = 0.01;
  const auto q_idx = cc.model.idx_qs[cc.model.getJointId("j_crank")];
  const double from = q_from[q_idx];
  const int n = std::max(1, static_cast<int>(std::ceil(std::abs(to - from) / kFineStep)));
  Eigen::VectorXd q = q_from;
  for (int k = 1; k <= n; ++k) {
    q[q_idx] = from + (to - from) * static_cast<double>(k) / static_cast<double>(n);
    const rub::ProjectionResult res =
        rub::ProjectPassiveToConstraint(cc.model, data, cc.constraints, q, cc.actuated_joint_ids);
    EXPECT_TRUE(res.converged) << "기준해 continuation 실패 (k=" << k << ")";
    q = res.q;
  }
  return q;
}

}  // namespace

// ── 성공기준 4: 큰 actuated 점프에서 조립 분기 이탈 — continuation 이 막는다 (#248). ──
//   점 구속 loop 은 조립 분기가 여러 개이고 **모두 φ=0 을 만족**하므로 residual 로는 옳은
//   분기를 고를 수 없다. 분기를 결정하는 것은 homotopy 경로다.
TEST(LoopProjectionPassive, ContinuationSurvivesLargeActuatedJump) {
  const rub::ClosedChainModel cc = CrankRocker();
  pinocchio::Data data(cc.model);
  const auto q_idx = cc.model.idx_qs[cc.model.getJointId("j_crank")];

  constexpr double kStart = 0.05;
  constexpr double kTarget = 1.2;  // Δ=1.15 rad — 기본 증분(0.05)의 23배 점프

  const Eigen::VectorXd q_start = AssembledAt(cc, data, kStart);
  const Eigen::VectorXd q_truth = FineContinuation(cc, data, q_start, kTarget);

  Eigen::VectorXd q_target = q_start;
  q_target[q_idx] = kTarget;

  // (a) **회귀 감지 (mutation guard)**: continuation 없는 단일 사영은 기준 분기를 벗어난다.
  //     이 EXPECT 가 깨지면 픽스처가 더는 결함을 재현하지 못한다는 뜻이므로 (b) 는 vacuous 가
  //     된다 — 그 경우 픽스처를 고쳐야지 (b) 를 약화하면 안 된다.
  const rub::ProjectionResult naive = rub::ProjectPassiveToConstraint(
      cc.model, data, cc.constraints, q_target, cc.actuated_joint_ids);
  EXPECT_GT((naive.q - q_truth).cwiseAbs().maxCoeff(), 1.0)
      << "단일 사영이 분기를 유지했다 — 이 픽스처는 더 이상 #248 을 재현하지 못한다";

  // (b) continuation 은 기준 분기를 유지한다.
  const rub::ProjectionResult fixed = rub::ProjectPassiveWithContinuation(
      cc.model, data, cc.constraints, q_start, q_target, cc.actuated_joint_ids);
  EXPECT_TRUE(fixed.converged);
  EXPECT_LT(fixed.final_error, 1e-8);
  EXPECT_LT((fixed.q - q_truth).cwiseAbs().maxCoeff(), 1e-6) << "분기 이탈";
  EXPECT_NEAR(fixed.q[q_idx], kTarget, 1e-12) << "actuated 는 정확히 고정";
}

// ── 성공기준 5: max_actuated_increment ≤ 0 은 continuation 을 끄고 단일 사영과 동일. ──
//   escape hatch 계약 고정 — 끄면 (a) 와 같은 이탈 동작으로 돌아간다.
TEST(LoopProjectionPassive, ContinuationDisabledMatchesSingleProjection) {
  const rub::ClosedChainModel cc = CrankRocker();
  pinocchio::Data data(cc.model);
  const auto q_idx = cc.model.idx_qs[cc.model.getJointId("j_crank")];

  const Eigen::VectorXd q_start = AssembledAt(cc, data, 0.05);
  Eigen::VectorXd q_target = q_start;
  q_target[q_idx] = 1.2;

  const rub::ProjectionResult single = rub::ProjectPassiveToConstraint(
      cc.model, data, cc.constraints, q_target, cc.actuated_joint_ids);
  const rub::ProjectionResult off = rub::ProjectPassiveWithContinuation(
      cc.model, data, cc.constraints, q_start, q_target, cc.actuated_joint_ids, {},
      /*max_actuated_increment=*/0.0);

  EXPECT_EQ(off.converged, single.converged);
  EXPECT_LT((off.q - single.q).cwiseAbs().maxCoeff(), 1e-12);
}

// ── 성공기준 6: 중간 sub-step 이 미수렴이면 **즉시 중단**하고 그 결과를 돌린다 (리뷰 ①). ──
//   미수렴 해를 warm-start 로 이어가면 homotopy 보장이 깨진 경로를 따라가는데, 마지막
//   sub-step 만 수렴하면 converged=true 로 보고돼 소비자가 그 형상을 커밋한다 — 단일 사영
//   시절에는 없던 실패 은폐 경로다.
TEST(LoopProjectionPassive, ContinuationStopsAtFirstNonConvergedSubstep) {
  const rub::ClosedChainModel cc = CrankRocker();
  pinocchio::Data data(cc.model);
  const auto q_idx = cc.model.idx_qs[cc.model.getJointId("j_crank")];

  constexpr double kStart = 0.05;
  constexpr double kTarget = 1.2;  // Δ=1.15 rad → sub-step 여러 개

  const Eigen::VectorXd q_start = AssembledAt(cc, data, kStart);
  Eigen::VectorXd q_target = q_start;
  q_target[q_idx] = kTarget;

  // 도달 불가 tolerance → **모든** sub-step 이 미수렴. (residual 바닥은 ~1e-16.)
  rub::ProjectionOptions opts;
  opts.tolerance = 1e-300;
  opts.max_iterations = 10;

  const rub::ProjectionResult res = rub::ProjectPassiveWithContinuation(
      cc.model, data, cc.constraints, q_start, q_target, cc.actuated_joint_ids, opts);

  ASSERT_FALSE(res.converged) << "도달 불가 tolerance 인데 수렴 보고 — 픽스처가 무력하다";
  // 첫 sub-step 에서 멈췄어야 한다. 계속 진행했다면 actuated 가 q_target(1.2)까지 갔을 것이다.
  EXPECT_GT(res.q[q_idx], kStart) << "첫 sub-step 은 진행했어야 한다";
  EXPECT_LE(res.q[q_idx], kStart + rub::kDefaultActuatedIncrement + 1e-12)
      << "미수렴인데도 다음 sub-step 으로 진행했다 (actuated=" << res.q[q_idx]
      << ") — 실패가 마지막 sub-step 결과에 가려진다";
}

// ── 성공기준 3: 특이 조립형상(four_bar 대칭)에서 미수렴이라도 NaN 을 내지 않는다. ───
//    소비자(P2 노드)는 미수렴 시 직전 해 hold — 그 전제로 결과가 항상 finite 여야 한다.
TEST(LoopProjectionPassive, SingularAssemblyStaysFinite) {
  const rub::ClosedChainModel cc = FourBar();  // q_ref_singular=true
  pinocchio::Data data(cc.model);
  const auto q_idx = cc.model.idx_qs[cc.model.getJointId("joint_a")];

  // 특이 neutral 근방에서 actuated 를 변위 → 근방에 해 없음(미수렴)이 정상.
  for (const double theta : {-0.2, -0.05, 0.05, 0.2}) {
    Eigen::VectorXd q_init = cc.q_ref;
    q_init[q_idx] = theta;
    const rub::ProjectionResult res = rub::ProjectPassiveToConstraint(
        cc.model, data, cc.constraints, q_init, cc.actuated_joint_ids);

    EXPECT_TRUE(res.q.allFinite()) << "theta=" << theta;           // NaN 금지
    EXPECT_NEAR(res.q[q_idx], theta, 1e-12) << "theta=" << theta;  // actuated 여전히 고정
  }
}
