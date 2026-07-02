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
