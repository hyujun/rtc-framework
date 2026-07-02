// ── test_closed_chain_handle — closed-chain 축약 동역학 질의 (M/g/h/J/FK) ─────
//   serial degeneracy · reduced round-trip(핵심) · gravity 유한차분 · 특이 flag.
#include "closure_test_fixtures.hpp"
#include "rtc_urdf_bridge/closed_chain_handle.hpp"
#include "rtc_urdf_bridge/loop_verification.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/constrained-dynamics.hpp>
#include <pinocchio/algorithm/energy.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/proximal.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Cholesky>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;

namespace {

// 독립 관절 순서로 q_ref(또는 임의 full q)에서 스칼라 q_a 추출.
std::vector<double> ActuatedSlice(const rub::ClosedChainHandle& handle,
                                  const pinocchio::Model& model, const Eigen::VectorXd& q_full) {
  std::vector<double> q_a;
  for (const auto& name : handle.GetIndependentJointNames()) {
    const auto jid = model.getJointId(name);
    q_a.push_back(q_full[model.idx_qs[jid]]);
  }
  return q_a;
}

// 독립 관절 순서로 full v-space 벡터에서 스칼라 성분 추출.
std::vector<double> IndepVSlice(const rub::ClosedChainHandle& handle, const pinocchio::Model& model,
                                const Eigen::VectorXd& v_full) {
  std::vector<double> out;
  for (const auto& name : handle.GetIndependentJointNames()) {
    const auto jid = model.getJointId(name);
    out.push_back(v_full[model.idx_vs[jid]]);
  }
  return out;
}

}  // namespace

// ── serial 등가: 구속 없으면 축약값 == RtModelHandle full 값 ──────────────────
TEST(ClosedChainHandle, SerialDegeneracyMatchesRtModelHandle) {
  // four_bar 의 spanning-tree 모델(구속 분리) 을 구속 없이 사용 → 순수 serial 4-DoF.
  const rub::ClosedChainModel ccm = rtc::test::FourBar();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  rub::ClosedChainHandle handle(model, {}, {}, {});  // 구속/actuated 없음 → 항등
  rub::RtModelHandle rt(model);

  ASSERT_EQ(handle.nv_independent(), model->nv);
  ASSERT_EQ(handle.constraint_dim(), 0);

  Eigen::VectorXd q(model->nq);
  Eigen::VectorXd v(model->nv);
  for (int i = 0; i < model->nq; ++i)
    q[i] = 0.1 * (i + 1);
  for (int i = 0; i < model->nv; ++i)
    v[i] = -0.2 * (i + 1);
  const std::vector<double> qv(q.data(), q.data() + q.size());
  const std::vector<double> vv(v.data(), v.data() + v.size());

  handle.Update(qv, vv);

  rt.ComputeMassMatrix(qv);
  EXPECT_LT((handle.GetMassMatrix() - rt.GetMassMatrix()).norm(), 1e-9);
  rt.ComputeGeneralizedGravity(qv);
  EXPECT_LT((handle.GetGeneralizedGravity() - rt.GetGeneralizedGravity()).norm(), 1e-9);
  rt.ComputeNonLinearEffects(qv, vv);
  EXPECT_LT((handle.GetNonLinearEffects() - rt.GetNonLinearEffects()).norm(), 1e-9);

  // FK + Jacobian (마지막 revolute 관절의 child frame).
  const auto fid = static_cast<pinocchio::FrameIndex>(model->nframes - 1);
  rt.ComputeForwardKinematics(qv);
  EXPECT_LT((handle.GetFramePosition(fid) - rt.GetFramePosition(fid)).norm(), 1e-9);

  rt.ComputeJacobians(qv);
  Eigen::MatrixXd J_rt(6, model->nv);
  Eigen::MatrixXd J_cc(6, model->nv);
  rt.GetFrameJacobian(fid, pinocchio::LOCAL_WORLD_ALIGNED, J_rt);
  handle.GetFrameJacobian(fid, pinocchio::LOCAL_WORLD_ALIGNED, J_cc);
  EXPECT_LT((J_rt - J_cc).norm(), 1e-9);
}

// ── 핵심: 축약 역동역학이 pinocchio constraintDynamics 왕복과 일치 ────────────
//   M_a·a_I + h_a == Gᵀ·τ_full,  a_I = (constraintDynamics ddq) 의 독립 성분.
//   → M_a·G·h_a(drift 항 포함) 를 pinocchio 순동역학으로 검증. crank-rocker(비특이).
TEST(ClosedChainHandle, CrankRockerReducedRoundTrip) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  // 구속 q_ref(neutral 사영)는 crank_rocker 에서 특이/미수렴 → neutral seed 로 두고
  // 비특이 조립 구간(crank≈0.2)에서 질의한다. n_a = nv − rank(Jc) (planar contact_3d
  // 는 rank 2 < m 3 이므로 nv−m 이 아님).
  rub::ClosedChainHandle handle(model, ccm.constraints, ccm.actuated_joint_ids, {});

  const int n_a = handle.nv_independent();
  ASSERT_GT(n_a, 0);

  const std::vector<double> q_a(static_cast<std::size_t>(n_a), 0.2);  // 비특이 crank
  std::vector<double> v_a(static_cast<std::size_t>(n_a));
  for (int i = 0; i < n_a; ++i)
    v_a[static_cast<std::size_t>(i)] = 0.37 * (i + 1) - 0.2;

  const rub::ClosedChainHandle::Status st = handle.Update(q_a, v_a);
  ASSERT_TRUE(st.converged);
  ASSERT_FALSE(st.singular) << "crank=0.2 는 비특이 조립 구간이어야 한다";

  const Eigen::VectorXd q_full = handle.GetFullConfiguration();
  const Eigen::MatrixXd G = handle.GetReductionMap();
  Eigen::VectorXd v_a_vec(n_a);
  for (int i = 0; i < n_a; ++i)
    v_a_vec[i] = v_a[static_cast<std::size_t>(i)];
  const Eigen::VectorXd v_full = G * v_a_vec;

  // v_full 은 구속-정합 (Jc v_full ≈ 0).
  {
    pinocchio::Data d(*model);
    const auto ck = rub::ComputeConstraintKinematics(*model, d, ccm.constraints, q_full);
    EXPECT_LT((ck.Jc * v_full).norm(), 1e-8);
  }

  // 오라클: 임의 τ_full → constraintDynamics → ddq (구속-정합).
  Eigen::VectorXd tau_full(model->nv);
  for (int i = 0; i < model->nv; ++i)
    tau_full[i] = 0.5 * (i + 1) - 1.1;

  pinocchio::Data data(*model);
  std::vector<pinocchio::RigidConstraintData> cdatas;
  cdatas.reserve(ccm.constraints.size());
  for (const auto& cm : ccm.constraints)
    cdatas.emplace_back(cm);
  pinocchio::initConstraintDynamics(*model, data, ccm.constraints, cdatas);
  // planar contact_3d 는 redundant(rank 2 < 3) → Delassus 특이. proximal mu>0 로 정칙화
  // (pinocchio 문서: 중복 구속 시 mu~1e-12 권장).
  pinocchio::ProximalSettings prox(1e-12, 1e-10, 50);
  const Eigen::VectorXd ddq = pinocchio::constraintDynamics(*model, data, q_full, v_full, tau_full,
                                                            ccm.constraints, cdatas, prox);

  // a_I = ddq 의 독립 성분.
  const std::vector<double> a_I_v = IndepVSlice(handle, *model, ddq);
  Eigen::VectorXd a_I(n_a);
  for (int i = 0; i < n_a; ++i)
    a_I[i] = a_I_v[static_cast<std::size_t>(i)];

  const Eigen::VectorXd lhs = handle.GetMassMatrix() * a_I + handle.GetNonLinearEffects();
  const Eigen::VectorXd rhs = G.transpose() * tau_full;
  // 비-공허성 가드: G≡0 / stale zero 버퍼면 lhs=rhs=0 으로 통과하므로 RHS·M_a 가
  // 유의미하게 non-trivial 임을 먼저 확인한다.
  ASSERT_GT(rhs.norm(), 1e-3) << "Gᵀτ 가 0 — 축약 map 이 채워지지 않음(공허 통과 방지)";
  ASSERT_GT(handle.GetMassMatrix().norm(), 1e-6) << "M_a 가 0 — 재계산 안 됨";
  EXPECT_LT((lhs - rhs).norm(), 1e-6) << "reduced EOM 이 constraintDynamics 와 불일치";
}

// ── g_a 검증: 위치에너지의 q_a 유한차분 == 축약 중력 (drift 무관) ─────────────
TEST(ClosedChainHandle, CrankRockerGravityMatchesPotentialGradient) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle handle(model, ccm.constraints, ccm.actuated_joint_ids, {});

  const int n_a = handle.nv_independent();
  const std::vector<double> q_a(static_cast<std::size_t>(n_a), 0.2);  // 비특이 crank
  ASSERT_TRUE(handle.Update(q_a).converged);
  const Eigen::VectorXd g_a = handle.GetGeneralizedGravity();

  // v_a 없음 → h_a == g_a.
  EXPECT_LT((handle.GetNonLinearEffects() - g_a).norm(), 1e-12);

  // U(q_a) = potentialEnergy(사영된 full q). dU/dq_a_k ≈ 중앙차분.
  pinocchio::Data d(*model);
  const double eps = 1e-6;
  for (int k = 0; k < n_a; ++k) {
    std::vector<double> qp = q_a, qm = q_a;
    qp[static_cast<std::size_t>(k)] += eps;
    qm[static_cast<std::size_t>(k)] -= eps;
    handle.Update(qp);
    const double Up = pinocchio::computePotentialEnergy(*model, d, handle.GetFullConfiguration());
    handle.Update(qm);
    const double Um = pinocchio::computePotentialEnergy(*model, d, handle.GetFullConfiguration());
    const double dU = (Up - Um) / (2.0 * eps);
    // g = dU/dq (일반화 중력은 위치에너지의 gradient). 부호: g_a_k ≈ dU/dq_a_k.
    EXPECT_NEAR(g_a[k], dU, 1e-4) << "독립 좌표 " << k;
  }
}

// ── M_a SPD + 운동에너지 정합: ½vᵀM_a v == ½v_fullᵀ M v_full ──────────────────
TEST(ClosedChainHandle, CrankRockerMassMatrixKineticEnergy) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle handle(model, ccm.constraints, ccm.actuated_joint_ids, {});

  const int n_a = handle.nv_independent();
  const std::vector<double> q_a(static_cast<std::size_t>(n_a), 0.2);  // 비특이 crank
  std::vector<double> v_a(static_cast<std::size_t>(n_a));
  for (int i = 0; i < n_a; ++i)
    v_a[static_cast<std::size_t>(i)] = 0.4 * (i + 1);
  handle.Update(q_a, v_a);

  const Eigen::MatrixXd M_a = handle.GetMassMatrix();
  EXPECT_LT((M_a - M_a.transpose()).norm(), 1e-10);  // 대칭 (impl 이 강제 대칭화)
  // 실제 positive-definite 검증: LLT 성공 = 전 방향 SPD (단일 벡터 positivity 로는 불충분).
  Eigen::LLT<Eigen::MatrixXd> llt(M_a);
  EXPECT_EQ(llt.info(), Eigen::Success) << "M_a 가 positive-definite 아님";
  Eigen::VectorXd v_a_vec(n_a);
  for (int i = 0; i < n_a; ++i)
    v_a_vec[i] = v_a[static_cast<std::size_t>(i)];

  // 운동에너지 일관성: T_red == T_full 은 M_a == GᵀMG 를 확인한다 (stale M_a-vs-G 검출).
  // 독립 검증은 아니다(정의상 항등) — M_a 정확성 자체는 round-trip 테스트가 담당한다.
  const Eigen::MatrixXd G = handle.GetReductionMap();
  const Eigen::VectorXd v_full = G * v_a_vec;
  pinocchio::Data d(*model);
  const double T_full =
      pinocchio::computeKineticEnergy(*model, d, handle.GetFullConfiguration(), v_full);
  const double T_red = 0.5 * (v_a_vec.transpose() * M_a * v_a_vec)(0, 0);
  EXPECT_NEAR(T_full, T_red, 1e-9);
}

// ── 특이 조립형상: NaN 대신 singular flag (four_bar q_ref 는 대칭 특이) ───────
TEST(ClosedChainHandle, FourBarSingularConfigFlaggedNoNaN) {
  const rub::ClosedChainModel ccm = rtc::test::FourBar();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle handle(model, ccm.constraints, ccm.actuated_joint_ids, ccm.q_ref);

  const std::vector<double> q_a = ActuatedSlice(handle, *model, ccm.q_ref);
  std::vector<double> v_a(static_cast<std::size_t>(handle.nv_independent()), 0.1);
  const rub::ClosedChainHandle::Status st = handle.Update(q_a, v_a);

  EXPECT_TRUE(st.singular) << "four_bar q_ref 는 대칭 특이 조립형상이어야 한다";
  EXPECT_TRUE(handle.GetMassMatrix().allFinite());
  EXPECT_TRUE(handle.GetGeneralizedGravity().allFinite());
  EXPECT_TRUE(handle.GetNonLinearEffects().allFinite());
  EXPECT_TRUE(handle.GetReductionMap().allFinite());
}
