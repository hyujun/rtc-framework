// ── test_rt_closed_chain_handle — RT-safe fixed-step loop projection FK ────────
//   #121 Phase 2b. RtClosedChainHandle 는 ClosedChainHandle(non-RT, 수렴 while+SVD) 의
//   FK 부분을 warm-start + 고정 K Newton + preallocated LDLT 로 RT-safe 재구현한다.
//   검증: (1) 500Hz 궤적에서 fully-converged FK 등가(<1mm), (2) 축약 J_a 등가,
//   (3) serial 항등(구속 없음 → 개방 체인 FK 와 동일), (4) size 불일치 → hold,
//   (5) 특이 조립형상 → singular flag + 유한 유지.
#include "closure_test_fixtures.hpp"
#include "rtc_urdf_bridge/closed_chain_handle.hpp"
#include "rtc_urdf_bridge/loop_verification.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "rtc_urdf_bridge/types.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/constrained-dynamics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/proximal.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Cholesky>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;

namespace {

// 독립 관절 순서로 full q(또는 v)에서 스칼라 성분 추출 (RT 핸들 입력 순서 =
// GetIndependentJointNames).
std::vector<double> IndepQSlice(const rub::RtClosedChainHandle& handle,
                                const pinocchio::Model& model, const Eigen::VectorXd& q_full) {
  std::vector<double> q_a;
  for (const auto& name : handle.GetIndependentJointNames()) {
    const auto jid = model.getJointId(name);
    q_a.push_back(q_full[static_cast<Eigen::Index>(model.idx_qs[jid])]);
  }
  return q_a;
}

}  // namespace

// ── (1) warm-start + 고정 K=2 사영이 fully-converged FK 를 <1mm 로 추종 ──────────
//   Phase 2a 스파이크(ProjectPassiveToConstraint, max_iter=K)와 동일 궤적을 RtClosedChainHandle
//   로 재현해 프로덕션 클래스가 동일 정확도임을 확인한다.
TEST(RtClosedChainHandle, WarmStartFixedStepTracksConverged) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  rub::ClosedChainHandle ref(model, ccm.constraints, ccm.actuated_joint_ids, {});
  ASSERT_EQ(ref.nv_independent(), 1);

  // ref 를 비특이 crank 중심으로 수렴시켜 warm-start seed 확보.
  ref.Update(std::vector<double>{0.2});
  const Eigen::VectorXd seed = ref.GetFullConfiguration();

  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids, seed,
                              /*num_iterations=*/2);
  ASSERT_EQ(rt.nv_independent(), 1);
  ASSERT_EQ(rt.constraint_dim(), ref.constraint_dim());

  const auto fid_c1 = rt.GetFrameId("c1");
  ASSERT_NE(fid_c1, 0u);
  ASSERT_TRUE(rt.IsFrameDownstreamOfLoop(fid_c1)) << "c1 은 loop-terminal 프레임";

  constexpr double kDt = 1.0 / 500.0;  // 500 Hz
  constexpr int kTicks = 1500;         // 3 s
  constexpr double kCenter = 0.2;
  constexpr double kAmp = 0.08;   // [0.12, 0.28] 비특이 band
  constexpr double kPeakV = 3.0;  // rad/s (가장 빠른 케이스)
  const double freq = kPeakV / (2.0 * M_PI * kAmp);

  double max_err = 0.0;
  int n = 0;
  for (int t = 0; t < kTicks; ++t) {
    const double q_a = kCenter + kAmp * std::sin(2.0 * M_PI * freq * (t * kDt));
    const rub::RtClosedChainHandle::Status st_rt = rt.Update(std::vector<double>{q_a});
    const rub::ClosedChainHandle::Status st_ref = ref.Update(std::vector<double>{q_a});
    ASSERT_FALSE(st_rt.held) << "비특이 궤적에서 hold 되면 안 된다";
    if (!st_ref.converged || st_ref.singular) {
      continue;  // 특이 형상은 비교 대상 아님 (RT 는 hold 정책)
    }
    const Eigen::Vector3d p_rt = rt.GetFramePosition(fid_c1);
    const Eigen::Vector3d p_ref = ref.GetFramePosition(fid_c1);
    max_err = std::max(max_err, (p_rt - p_ref).norm());
    ++n;
  }
  ASSERT_GT(n, 100);
  EXPECT_LT(max_err, 1e-3) << "고정 K=2 RT 사영이 fully-converged FK 를 <1mm 로 추종해야 한다";
}

// ── (2) 축약 프레임 Jacobian J_a 가 fully-converged 핸들과 일치 ─────────────────
//   RT G 는 SVD damped-pinv 대신 정규방정식 left-pinv 를 쓰므로 비특이 조립형상에서 수치
//   등가여야 한다.
TEST(RtClosedChainHandle, ReducedJacobianMatchesConverged) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  rub::ClosedChainHandle ref(model, ccm.constraints, ccm.actuated_joint_ids, {});
  ref.Update(std::vector<double>{0.2});
  const Eigen::VectorXd seed = ref.GetFullConfiguration();

  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids, seed, 2);

  const rub::RtClosedChainHandle::Status st = rt.Update(std::vector<double>{0.2});
  ASSERT_FALSE(st.held);
  ASSERT_FALSE(st.singular) << "crank=0.2 는 비특이";
  ASSERT_LT(st.closure_error, 1e-6) << "warm-start + K=2 로 잘 닫혀야 한다";

  const int n_a = rt.nv_independent();
  const auto fid_c1 = rt.GetFrameId("c1");
  Eigen::MatrixXd J_rt(6, n_a);
  Eigen::MatrixXd J_ref(6, n_a);
  rt.GetFrameJacobian(fid_c1, pinocchio::LOCAL_WORLD_ALIGNED, J_rt);
  ref.GetFrameJacobian(fid_c1, pinocchio::LOCAL_WORLD_ALIGNED, J_ref);
  EXPECT_LT((J_rt - J_ref).norm(), 1e-5)
      << "정규방정식 left-pinv G 가 SVD-pinv 와 비특이 형상에서 일치해야 한다";

  // G 도 일치 (독립 블록 = I, 종속 블록 = 사영).
  EXPECT_LT((rt.GetReductionMap() - ref.GetReductionMap()).norm(), 1e-5);
}

// ── (3) serial 항등: 구속 없으면 개방 체인 FK/Jacobian 과 byte-근사 동일 ─────────
TEST(RtClosedChainHandle, SerialModelIdentity) {
  const rub::ClosedChainModel ccm = rtc::test::FourBar();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  ASSERT_EQ(model->nq, model->nv) << "fixture 는 all-revolute (nq==nv) 전제";

  rub::RtClosedChainHandle rt(model, {}, {}, {});  // 구속/actuated 없음 → 항등
  rub::RtModelHandle open(model);
  ASSERT_EQ(rt.nv_independent(), model->nv);
  ASSERT_EQ(rt.constraint_dim(), 0);

  std::vector<double> q_a(static_cast<std::size_t>(model->nv));
  for (int i = 0; i < model->nv; ++i) {
    q_a[static_cast<std::size_t>(i)] = 0.13 * (i + 1);
  }
  const rub::RtClosedChainHandle::Status st = rt.Update(q_a);
  ASSERT_FALSE(st.held);
  ASSERT_FALSE(st.singular);

  // 주입한 q_a 가 그대로 full configuration (사영 없음).
  const Eigen::VectorXd& q_full = rt.GetFullConfiguration();
  for (int i = 0; i < model->nv; ++i) {
    EXPECT_NEAR(q_full[i], q_a[static_cast<std::size_t>(i)], 1e-12);
  }

  // FK / Jacobian 이 개방 체인 핸들과 동일.
  const auto fid = static_cast<pinocchio::FrameIndex>(model->nframes - 1);
  open.ComputeForwardKinematics(q_a);
  EXPECT_LT((rt.GetFramePosition(fid) - open.GetFramePosition(fid)).norm(), 1e-12);

  open.ComputeJacobians(q_a);
  Eigen::MatrixXd J_open(6, model->nv);
  Eigen::MatrixXd J_rt(6, model->nv);  // n_a == nv (항등)
  open.GetFrameJacobian(fid, pinocchio::LOCAL_WORLD_ALIGNED, J_open);
  rt.GetFrameJacobian(fid, pinocchio::LOCAL_WORLD_ALIGNED, J_rt);
  EXPECT_LT((J_rt - J_open).norm(), 1e-12) << "G=I → J_a == J_full";

  // 항등이면 loop 하류 프레임이 없다.
  EXPECT_FALSE(rt.IsFrameDownstreamOfLoop(fid));
}

// ── (4) q_a size 불일치 → RT 에서 throw 대신 hold (직전 해 유지) ────────────────
TEST(RtClosedChainHandle, SizeMismatchHolds) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle ref(model, ccm.constraints, ccm.actuated_joint_ids, {});
  ref.Update(std::vector<double>{0.2});
  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids,
                              ref.GetFullConfiguration(), 2);

  const rub::RtClosedChainHandle::Status ok = rt.Update(std::vector<double>{0.2});
  ASSERT_FALSE(ok.held);
  const Eigen::VectorXd q_good = rt.GetFullConfiguration();
  const Eigen::Vector3d p_good = rt.GetFramePosition(rt.GetFrameId("c1"));

  // 잘못된 크기 (n_a=1 인데 2개) → held, full q / FK 불변.
  const rub::RtClosedChainHandle::Status bad = rt.Update(std::vector<double>{0.2, 0.3});
  EXPECT_TRUE(bad.held);
  EXPECT_LT((rt.GetFullConfiguration() - q_good).norm(), 1e-15);
  EXPECT_LT((rt.GetFramePosition(rt.GetFrameId("c1")) - p_good).norm(), 1e-15);
}

// ── (5) 특이 조립형상 → singular flag + FK 유한 유지 (damped, NaN 없음) ─────────
//   four_bar 는 대칭 특이 4-bar (neutral 근방 q_ref 특이). RT 핸들은 hold 를 강제하지 않고
//   singular 를 flag 하며(소비자 정책), 출력은 damped 라 유한해야 한다.
TEST(RtClosedChainHandle, SingularAssemblyFlagsSingular) {
  const rub::ClosedChainModel ccm = rtc::test::FourBar();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids, ccm.q_ref, 2);

  const int n_a = rt.nv_independent();
  ASSERT_GT(n_a, 0);
  // 특이 형상 = q_ref (대칭 조립). q_ref 에서 독립 관절 슬라이스를 q_a 로 쓴다 (non-RT 테스트
  // 동일).
  std::vector<double> q_a;
  for (const auto& name : rt.GetIndependentJointNames()) {
    const auto jid = model->getJointId(name);
    q_a.push_back(ccm.q_ref[model->idx_qs[jid]]);
  }

  const rub::RtClosedChainHandle::Status st = rt.Update(q_a);
  EXPECT_TRUE(st.singular) << "대칭 4-bar 조립형상은 특이로 flag 돼야 한다";
  EXPECT_FALSE(st.held) << "damped 라 유한 — NaN hold 는 아님";

  // FK / G 가 유한 (NaN 누출 없음).
  const auto fid = static_cast<pinocchio::FrameIndex>(model->nframes - 1);
  EXPECT_TRUE(rt.GetFramePosition(fid).allFinite());
  EXPECT_TRUE(rt.GetReductionMap().allFinite());
  EXPECT_TRUE(rt.GetFullConfiguration().allFinite());
}

// ── (6) q_seed 크기 misconfig → 조용한 neutral fallback 대신 throw (review #5) ────
//   빈 seed 는 명시적 neutral 신호(허용). 비어있지 않은데 크기≠nq 면 대칭 링키지에서 neutral 이
//   특이 조립 → 영구 저하로 이어지므로 생성자가 throw 해 config 단계에서 표면화한다.
TEST(RtClosedChainHandle, SeedSizeMismatchThrows) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  // 빈 seed → neutral, throw 없음.
  EXPECT_NO_THROW(
      rub::RtClosedChainHandle(model, ccm.constraints, ccm.actuated_joint_ids, Eigen::VectorXd{}));

  // 크기 nq+1 → throw.
  Eigen::VectorXd bad = Eigen::VectorXd::Zero(model->nq + 1);
  EXPECT_THROW(rub::RtClosedChainHandle(model, ccm.constraints, ccm.actuated_joint_ids, bad),
               std::invalid_argument);
}

// ── (7) identity 경로 비유한 q_a → held (직전 해 유지, NaN 누출 없음) (review #2) ──
//   비-identity 경로의 allFinite guard 와 대칭. 측정 NaN 이 held=false 로 새어나가면 안 된다.
TEST(RtClosedChainHandle, IdentityNonFiniteHolds) {
  const rub::ClosedChainModel ccm = rtc::test::FourBar();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::RtClosedChainHandle rt(model, {}, {}, {});  // 구속 없음 → identity

  std::vector<double> q_a(static_cast<std::size_t>(model->nv), 0.1);
  ASSERT_FALSE(rt.Update(q_a).held);  // 유효 tick → 캐시
  const Eigen::VectorXd q_good = rt.GetFullConfiguration();
  const auto fid = static_cast<pinocchio::FrameIndex>(model->nframes - 1);
  const Eigen::Vector3d p_good = rt.GetFramePosition(fid);

  // NaN 주입 → held, full q / FK 불변 + 유한 유지.
  q_a[0] = std::numeric_limits<double>::quiet_NaN();
  const rub::RtClosedChainHandle::Status bad = rt.Update(q_a);
  EXPECT_TRUE(bad.held) << "identity 경로도 비유한 q 는 hold";
  EXPECT_TRUE(rt.GetFramePosition(fid).allFinite()) << "NaN 누출 없음";
  EXPECT_LT((rt.GetFullConfiguration() - q_good).norm(), 1e-15);
  EXPECT_LT((rt.GetFramePosition(fid) - p_good).norm(), 1e-15);
}

// ── (8) 범위 밖 frame_id getter → OOB 대신 항등/0 (review #9) ───────────────────
TEST(RtClosedChainHandle, OutOfRangeFrameGettersSafe) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids, {});

  const auto oob = static_cast<pinocchio::FrameIndex>(model->nframes + 100);
  EXPECT_TRUE(rt.GetFramePlacement(oob).isIdentity());
  EXPECT_EQ(rt.GetFramePosition(oob).norm(), 0.0);
  EXPECT_TRUE(rt.GetFrameRotation(oob).isIdentity());
}

// ── (9) 축약 동역학 M_a/g_a/h_a 가 non-RT ClosedChainHandle 과 일치 (crank_rocker 비특이) ──
//   RT 는 SVD damped-pinv 대신 정규방정식 left-pinv(=수치 등가) 로 G/Jc_D⁺ 를 쓰므로
//   비특이 조립형상에서 M_a=GᵀMG, g_a=Gᵀg, h_a=Gᵀrnea(...) 가 non-RT 와 일치해야 한다.
TEST(RtClosedChainHandle, ReducedDynamicsMatchesConverged) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  rub::ClosedChainHandle ref(model, ccm.constraints, ccm.actuated_joint_ids, {});
  ref.Update(std::vector<double>{0.2});
  const Eigen::VectorXd seed = ref.GetFullConfiguration();

  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids, seed, 2);
  const int n_a = rt.nv_independent();
  ASSERT_EQ(n_a, ref.nv_independent());

  const std::vector<double> q_a(static_cast<std::size_t>(n_a), 0.2);  // 비특이 crank
  std::vector<double> v_a(static_cast<std::size_t>(n_a));
  for (int i = 0; i < n_a; ++i)
    v_a[static_cast<std::size_t>(i)] = 0.37 * (i + 1) - 0.2;

  const rub::ClosedChainHandle::Status rst = ref.Update(q_a, v_a);
  ASSERT_TRUE(rst.converged);
  ASSERT_FALSE(rst.singular) << "crank=0.2 는 비특이 조립 구간";

  const rub::RtClosedChainHandle::Status st = rt.Update(q_a);
  ASSERT_FALSE(st.held);
  ASSERT_FALSE(st.singular);
  const rub::RtClosedChainHandle::Status dst = rt.UpdateDynamics(v_a);
  ASSERT_FALSE(dst.held);

  EXPECT_LT((rt.GetMassMatrix() - ref.GetMassMatrix()).norm(), 1e-5) << "M_a parity";
  EXPECT_LT((rt.GetGeneralizedGravity() - ref.GetGeneralizedGravity()).norm(), 1e-5)
      << "g_a parity";
  EXPECT_LT((rt.GetNonLinearEffects() - ref.GetNonLinearEffects()).norm(), 1e-4) << "h_a parity";

  // v_a 미제공 → h_a == g_a (bias 없음).
  ASSERT_FALSE(rt.Update(q_a).held);
  ASSERT_FALSE(rt.UpdateDynamics({}).held);
  EXPECT_LT((rt.GetNonLinearEffects() - rt.GetGeneralizedGravity()).norm(), 1e-12);
}

// ── (10) 축약 역동역학 round-trip: M_a·a_I + h_a == Gᵀτ (constraintDynamics 오라클) ──
//   non-RT CrankRockerReducedRoundTrip 를 RtClosedChainHandle 로 재현 — RT 축약 EOM 이
//   pinocchio 순동역학과 일치함을 독립 검증 (spec [SPRINT] 1b).
TEST(RtClosedChainHandle, ReducedRoundTripConstraintDynamics) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  rub::ClosedChainHandle ref(model, ccm.constraints, ccm.actuated_joint_ids, {});
  ref.Update(std::vector<double>{0.2});
  const Eigen::VectorXd seed = ref.GetFullConfiguration();

  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids, seed, 2);
  const int n_a = rt.nv_independent();
  ASSERT_GT(n_a, 0);

  const std::vector<double> q_a(static_cast<std::size_t>(n_a), 0.2);
  std::vector<double> v_a(static_cast<std::size_t>(n_a));
  for (int i = 0; i < n_a; ++i)
    v_a[static_cast<std::size_t>(i)] = 0.37 * (i + 1) - 0.2;

  const rub::RtClosedChainHandle::Status st = rt.Update(q_a);
  ASSERT_FALSE(st.held);
  ASSERT_FALSE(st.singular);
  ASSERT_LT(st.closure_error, 1e-6) << "warm-start + K=2 로 잘 닫혀야 한다";
  ASSERT_FALSE(rt.UpdateDynamics(v_a).held);

  const Eigen::VectorXd q_full = rt.GetFullConfiguration();
  const Eigen::MatrixXd G = rt.GetReductionMap();
  Eigen::VectorXd v_a_vec(n_a);
  for (int i = 0; i < n_a; ++i)
    v_a_vec[i] = v_a[static_cast<std::size_t>(i)];
  const Eigen::VectorXd v_full = G * v_a_vec;

  // v_full 은 구속-정합 (Jc v_full ≈ 0).
  {
    pinocchio::Data d(*model);
    const auto ck = rub::ComputeConstraintKinematics(*model, d, ccm.constraints, q_full);
    EXPECT_LT((ck.Jc * v_full).norm(), 1e-6);
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
  pinocchio::ProximalSettings prox(1e-12, 1e-10, 50);  // redundant contact_3d → mu>0 정칙화
  const Eigen::VectorXd ddq = pinocchio::constraintDynamics(*model, data, q_full, v_full, tau_full,
                                                            ccm.constraints, cdatas, prox);

  // a_I = ddq 의 독립 성분.
  Eigen::VectorXd a_I(n_a);
  {
    int k = 0;
    for (const auto& name : rt.GetIndependentJointNames()) {
      const auto jid = model->getJointId(name);
      a_I[k++] = ddq[static_cast<Eigen::Index>(model->idx_vs[jid])];
    }
  }

  const Eigen::VectorXd lhs = rt.GetMassMatrix() * a_I + rt.GetNonLinearEffects();
  const Eigen::VectorXd rhs = G.transpose() * tau_full;
  ASSERT_GT(rhs.norm(), 1e-3) << "Gᵀτ 가 0 — 축약 map 미충전(공허 통과 방지)";
  ASSERT_GT(rt.GetMassMatrix().norm(), 1e-6) << "M_a 가 0 — 재계산 안 됨";
  EXPECT_LT((lhs - rhs).norm(), 1e-5) << "RT reduced EOM 이 constraintDynamics 와 불일치";
}

// ── (11) serial 항등 동역학: 구속 없으면 M_a==crba(model), g_a==g, h_a==nle ──────
TEST(RtClosedChainHandle, SerialModelDynamicsIdentity) {
  const rub::ClosedChainModel ccm = rtc::test::FourBar();  // all-revolute tree
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::RtClosedChainHandle rt(model, {}, {}, {});  // 구속/actuated 없음 → 항등
  const int n = model->nv;
  ASSERT_EQ(rt.nv_independent(), n);

  std::vector<double> q_a(static_cast<std::size_t>(n));
  std::vector<double> v_a(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    q_a[static_cast<std::size_t>(i)] = 0.13 * (i + 1);
    v_a[static_cast<std::size_t>(i)] = 0.05 * (i + 1);
  }
  ASSERT_FALSE(rt.Update(q_a).held);
  ASSERT_FALSE(rt.UpdateDynamics(v_a).held);

  Eigen::VectorXd qv(n);
  Eigen::VectorXd vv(n);
  for (int i = 0; i < n; ++i) {
    qv[i] = q_a[static_cast<std::size_t>(i)];
    vv[i] = v_a[static_cast<std::size_t>(i)];
  }
  pinocchio::Data d(*model);
  pinocchio::crba(*model, d, qv);
  d.M.triangularView<Eigen::StrictlyLower>() =
      d.M.transpose().triangularView<Eigen::StrictlyLower>();
  EXPECT_LT((rt.GetMassMatrix() - d.M).norm(), 1e-10) << "G=I → M_a == crba(model)";
  pinocchio::computeGeneralizedGravity(*model, d, qv);
  EXPECT_LT((rt.GetGeneralizedGravity() - d.g).norm(), 1e-10);
  pinocchio::nonLinearEffects(*model, d, qv, vv);
  EXPECT_LT((rt.GetNonLinearEffects() - d.nle).norm(), 1e-10) << "identity → h_a == nle";
}

// ── (12) 특이 조립형상 동역학: singular flag + M_a/g_a/h_a 유한 (damped, NaN 없음) ──
TEST(RtClosedChainHandle, SingularAssemblyDynamicsFinite) {
  const rub::ClosedChainModel ccm = rtc::test::FourBar();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids, ccm.q_ref, 2);

  const std::vector<double> q_a = IndepQSlice(rt, *model, ccm.q_ref);
  const std::vector<double> v_a(static_cast<std::size_t>(rt.nv_independent()), 0.1);

  const rub::RtClosedChainHandle::Status st = rt.Update(q_a);
  EXPECT_TRUE(st.singular) << "대칭 4-bar q_ref 는 특이 조립형상";
  const rub::RtClosedChainHandle::Status dst = rt.UpdateDynamics(v_a);
  EXPECT_FALSE(dst.held) << "damped → 유한, NaN hold 아님";

  EXPECT_TRUE(rt.GetMassMatrix().allFinite());
  EXPECT_TRUE(rt.GetGeneralizedGravity().allFinite());
  EXPECT_TRUE(rt.GetNonLinearEffects().allFinite());
}

// ── (13) Update 가 held 면 UpdateDynamics 도 직전 동역학 hold (byte-불변) ─────────
TEST(RtClosedChainHandle, DynamicsHeldWhenUpdateHeld) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle ref(model, ccm.constraints, ccm.actuated_joint_ids, {});
  ref.Update(std::vector<double>{0.2});
  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids,
                              ref.GetFullConfiguration(), 2);
  const int n_a = rt.nv_independent();
  const std::vector<double> q_a(static_cast<std::size_t>(n_a), 0.2);
  const std::vector<double> v_a(static_cast<std::size_t>(n_a), 0.1);

  ASSERT_FALSE(rt.Update(q_a).held);
  ASSERT_FALSE(rt.UpdateDynamics(v_a).held);
  const Eigen::MatrixXd M_good = rt.GetMassMatrix();
  const Eigen::VectorXd h_good = rt.GetNonLinearEffects();

  // 잘못된 크기 Update → held → UpdateDynamics 미갱신 (직전값 유지).
  const rub::RtClosedChainHandle::Status bad = rt.Update(std::vector<double>{0.2, 0.3});
  ASSERT_TRUE(bad.held);
  const rub::RtClosedChainHandle::Status dyn = rt.UpdateDynamics(v_a);
  EXPECT_TRUE(dyn.held);
  EXPECT_LT((rt.GetMassMatrix() - M_good).norm(), 1e-15);
  EXPECT_LT((rt.GetNonLinearEffects() - h_good).norm(), 1e-15);
}

// ── (14) arm + closed-chain hand 축약 동역학 (topology archetype: arm 무영향 + hand loop) ──
//   현재 build-only 이던 arm_with_four_bar_hand 픽스처에 동역학 테스트 신설 (spec [SPRINT] 1).
//   독립 집합 = 모든 movable − loop-passive lock (full-model index) → dep = hand passive 만
//   (dep ≤ m well-posed). Phase 2 WBC 배선이 동일 로직을 쓴다.
TEST(RtClosedChainHandle, ArmWithFourBarHandReducedDynamics) {
  rub::ModelConfig cfg;
  cfg.urdf_path = rtc::test::TestUrdfPath("arm_with_four_bar_hand.urdf.xacro");
  cfg.root_joint_type = "fixed";
  cfg.closure_yaml_path = rtc::test::TestUrdfPath("four_bar.closure.yaml");
  const rub::PinocchioModelBuilder builder(cfg);

  const auto full = builder.GetFullModel();
  ASSERT_NE(full, nullptr);
  ASSERT_FALSE(builder.GetConstraintModels().empty()) << "arm+hand 은 loop 구속을 가져야 한다";

  // 독립 집합 = movable − **loop-passive** {joint_ab, joint_c, joint_cd}. builder 의
  // passive-lock(movable−actuated) 은 arm_joint 도 잠그지만(사이드카가 standalone hand 기준
  // 이라 arm 미선언), arm_joint 는 loop 밖 독립 DoF 다 → dep = 3(hand passive) = m 로 well-posed.
  // (실로봇 사이드카는 arm/hand 를 모두 actuated 로 선언하므로 movable−actuated 가 곧
  // loop-passive.)
  const std::vector<std::string> loop_passive{"joint_ab", "joint_c", "joint_cd"};
  std::vector<pinocchio::JointIndex> actuated_ids;
  for (int jid = 1; jid < full->njoints; ++jid) {
    const auto jidx = static_cast<std::size_t>(jid);
    if (full->nvs[jidx] == 0) {
      continue;  // fixed
    }
    if (std::find(loop_passive.begin(), loop_passive.end(), full->names[jidx]) ==
        loop_passive.end()) {
      actuated_ids.push_back(static_cast<pinocchio::JointIndex>(jid));
    }
  }
  ASSERT_EQ(actuated_ids.size(), 2u) << "독립 = {arm_joint, joint_a}";

  const Eigen::VectorXd& q_ref = builder.GetClosureReferenceConfig();
  rub::ClosedChainHandle ref(full, builder.GetConstraintModels(), actuated_ids, q_ref);
  rub::RtClosedChainHandle rt(full, builder.GetConstraintModels(), actuated_ids, q_ref, 2);
  const int n_a = rt.nv_independent();
  ASSERT_EQ(n_a, static_cast<int>(actuated_ids.size()));
  ASSERT_EQ(ref.nv_independent(), n_a);

  const std::vector<double> q_a = IndepQSlice(rt, *full, q_ref);
  std::vector<double> v_a(static_cast<std::size_t>(n_a));
  for (int i = 0; i < n_a; ++i)
    v_a[static_cast<std::size_t>(i)] = 0.02 * (i + 1);

  const rub::ClosedChainHandle::Status ref_st = ref.Update(q_a, v_a);
  const rub::RtClosedChainHandle::Status st = rt.Update(q_a);
  ASSERT_FALSE(st.held);
  ASSERT_FALSE(rt.UpdateDynamics(v_a).held);

  // 항상: 차원 + 유한 + 대칭 (특이/비특이 무관).
  EXPECT_EQ(rt.GetMassMatrix().rows(), n_a);
  EXPECT_EQ(rt.GetMassMatrix().cols(), n_a);
  EXPECT_TRUE(rt.GetMassMatrix().allFinite());
  EXPECT_TRUE(rt.GetGeneralizedGravity().allFinite());
  EXPECT_TRUE(rt.GetNonLinearEffects().allFinite());
  EXPECT_LT((rt.GetMassMatrix() - rt.GetMassMatrix().transpose()).norm(), 1e-9) << "M_a 대칭";

  // 비특이 조립일 때만 non-RT parity + SPD (특이는 damped 라 값이 갈릴 수 있음).
  if (!st.singular && !ref_st.singular && ref_st.converged) {
    EXPECT_LT((rt.GetMassMatrix() - ref.GetMassMatrix()).norm(), 1e-4) << "M_a parity";
    EXPECT_LT((rt.GetGeneralizedGravity() - ref.GetGeneralizedGravity()).norm(), 1e-4);
    EXPECT_LT((rt.GetNonLinearEffects() - ref.GetNonLinearEffects()).norm(), 1e-3);
    Eigen::LLT<Eigen::MatrixXd> llt(rt.GetMassMatrix().eval());
    EXPECT_EQ(llt.info(), Eigen::Success) << "비특이 → M_a SPD";
  }
}

// ── (15) loop-consistent dJv 가 재사영 중앙차분 oracle 과 일치 (#173 L2-exact) ─────
//   dJv = J̇_a·v_a 를 fully-converged 재사영 J_a(q_a ± ε·v_a) 의 중앙차분과 대조한다. 이
//   oracle 은 G(q) 의 변화까지 포함하므로 full-tree frozen-loop FD 가 아니라 실제 축약
//   Jacobian 의 drift 를 검증한다. tolerance 는 고정-K 사영·damped-pinv 잔차가 FD 오차
//   바닥을 높이므로 순수 pinocchio FD 보다 여유를 둔다 (issue #173 리뷰).
TEST(RtClosedChainHandle, DriftMatchesReprojectedFiniteDifference) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  rub::ClosedChainHandle ref(model, ccm.constraints, ccm.actuated_joint_ids, {});
  ref.Update(std::vector<double>{0.2});
  const Eigen::VectorXd seed = ref.GetFullConfiguration();

  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids, seed, 2);
  const int n_a = rt.nv_independent();
  const std::vector<double> q_a(static_cast<std::size_t>(n_a), 0.2);
  std::vector<double> v_a(static_cast<std::size_t>(n_a));
  for (int i = 0; i < n_a; ++i)
    v_a[static_cast<std::size_t>(i)] = 0.37 * (i + 1) - 0.2;

  const rub::RtClosedChainHandle::Status st = rt.Update(q_a);
  ASSERT_FALSE(st.held);
  ASSERT_FALSE(st.singular);
  ASSERT_FALSE(rt.UpdateDynamics(v_a).held);

  const pinocchio::FrameIndex fid = rt.GetFrameId("c1");
  ASSERT_NE(fid, static_cast<pinocchio::FrameIndex>(0));
  ASSERT_TRUE(rt.IsFrameDownstreamOfLoop(fid)) << "c1 은 loop-하류 (drift 가 비자명한 프레임)";
  Eigen::Matrix<double, 6, 1> dJv;
  rt.GetFrameClassicalAccelerationDrift(fid, pinocchio::LOCAL_WORLD_ALIGNED, dJv);
  ASSERT_TRUE(dJv.allFinite());
  ASSERT_GT(dJv.norm(), 1e-8) << "v≠0 → loop-consistent drift 비영 (공허 통과 방지)";

  // oracle: fully-converged 재사영 J_a(q_a ± ε v_a) 중앙차분 → J̇_a·v_a.
  const double eps = 1e-5;
  std::vector<double> q_p(q_a);
  std::vector<double> q_m(q_a);
  for (int i = 0; i < n_a; ++i) {
    q_p[static_cast<std::size_t>(i)] += eps * v_a[static_cast<std::size_t>(i)];
    q_m[static_cast<std::size_t>(i)] -= eps * v_a[static_cast<std::size_t>(i)];
  }
  rub::ClosedChainHandle hp(model, ccm.constraints, ccm.actuated_joint_ids, seed);
  rub::ClosedChainHandle hm(model, ccm.constraints, ccm.actuated_joint_ids, seed);
  ASSERT_TRUE(hp.Update(q_p).converged);
  ASSERT_TRUE(hm.Update(q_m).converged);
  Eigen::MatrixXd J_p(6, n_a);
  Eigen::MatrixXd J_m(6, n_a);
  hp.GetFrameJacobian(fid, pinocchio::LOCAL_WORLD_ALIGNED, J_p);
  hm.GetFrameJacobian(fid, pinocchio::LOCAL_WORLD_ALIGNED, J_m);
  Eigen::VectorXd v_vec(n_a);
  for (int i = 0; i < n_a; ++i)
    v_vec[i] = v_a[static_cast<std::size_t>(i)];
  const Eigen::Matrix<double, 6, 1> dJv_fd = ((J_p - J_m) / (2.0 * eps)) * v_vec;

  EXPECT_LT((dJv - dJv_fd).norm(), 1e-4)
      << "loop-consistent dJv ≠ 재사영 FD oracle: dJv=" << dJv.transpose()
      << ", fd=" << dJv_fd.transpose();
}

// ── (16) serial 항등 drift: 구속 없으면 pinocchio classical accel(q, v, a=0) 과 일치 ──
TEST(RtClosedChainHandle, SerialDriftMatchesPinocchio) {
  const rub::ClosedChainModel ccm = rtc::test::FourBar();  // all-revolute tree
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  ASSERT_EQ(model->nq, model->nv) << "fixture 는 all-revolute (nq==nv) 전제";

  rub::RtClosedChainHandle rt(model, {}, {}, {});  // 구속/actuated 없음 → 항등
  const int nv = model->nv;
  std::vector<double> q_a(static_cast<std::size_t>(nv));
  std::vector<double> v_a(static_cast<std::size_t>(nv));
  for (int i = 0; i < nv; ++i) {
    q_a[static_cast<std::size_t>(i)] = 0.13 * (i + 1);
    v_a[static_cast<std::size_t>(i)] = 0.07 * (i + 1) - 0.1;
  }
  ASSERT_FALSE(rt.Update(q_a).held);
  ASSERT_FALSE(rt.UpdateDynamics(v_a).held);

  const auto fid = static_cast<pinocchio::FrameIndex>(model->nframes - 1);
  Eigen::Matrix<double, 6, 1> dJv;
  rt.GetFrameClassicalAccelerationDrift(fid, pinocchio::LOCAL_WORLD_ALIGNED, dJv);

  // oracle: 개방 체인 2차 FK (zero accel) + classical accel — G=I 라 v_full=v_a 직접.
  pinocchio::Data d(*model);
  const Eigen::VectorXd& q_full = rt.GetFullConfiguration();
  Eigen::VectorXd v_full(nv);
  for (int i = 0; i < nv; ++i)
    v_full[i] = v_a[static_cast<std::size_t>(i)];
  pinocchio::forwardKinematics(*model, d, q_full, v_full, Eigen::VectorXd::Zero(nv).eval());
  const Eigen::Matrix<double, 6, 1> dJv_ref =
      pinocchio::getFrameClassicalAcceleration(*model, d, fid, pinocchio::LOCAL_WORLD_ALIGNED)
          .toVector();

  ASSERT_GT(dJv_ref.norm(), 1e-8) << "v≠0 → 개방 체인 drift 비영 (공허 통과 방지)";
  EXPECT_LT((dJv - dJv_ref).norm(), 1e-12) << "serial 항등 → pinocchio classical accel 과 일치";
}

// ── (17) v_a 미제공 시 drift = 0 (#173 F2: no-velocity 경로 stale a_drift 누출 회귀) ──
TEST(RtClosedChainHandle, DriftZeroWithoutVelocity) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle ref(model, ccm.constraints, ccm.actuated_joint_ids, {});
  ref.Update(std::vector<double>{0.2});
  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids,
                              ref.GetFullConfiguration(), 2);
  const int n_a = rt.nv_independent();
  const std::vector<double> q_a(static_cast<std::size_t>(n_a), 0.2);
  const std::vector<double> v_a(static_cast<std::size_t>(n_a), 0.37);
  const pinocchio::FrameIndex fid = rt.GetFrameId("c1");

  // 먼저 비영 속도로 drift 를 채워 stale 상태를 만든다.
  ASSERT_FALSE(rt.Update(q_a).held);
  ASSERT_FALSE(rt.UpdateDynamics(v_a).held);
  Eigen::Matrix<double, 6, 1> dJv;
  rt.GetFrameClassicalAccelerationDrift(fid, pinocchio::LOCAL_WORLD_ALIGNED, dJv);
  ASSERT_GT(dJv.norm(), 1e-8) << "선행 tick 은 비영 drift (stale 재료)";

  // no-velocity tick → 직전 a_drift 가 잔존하면 비영 누출 (F2). 정확히 0 이어야 한다.
  ASSERT_FALSE(rt.Update(q_a).held);
  ASSERT_FALSE(rt.UpdateDynamics({}).held);
  rt.GetFrameClassicalAccelerationDrift(fid, pinocchio::LOCAL_WORLD_ALIGNED, dJv);
  EXPECT_EQ(dJv.norm(), 0.0) << "v=0 → drift 정확히 0 (stale a_drift 누출 없음)";
}

// ── (18) drift getter 와 J/oMf getter 의 data_ 상태 동시 유효 (#173 F1) + OOB/held ──
//   RebuildReducedDynamics 말미 3단 복원(computeJointJacobians → 2차 forwardKinematics →
//   updateFramePlacements) 뒤 세 getter 를 인터리브 호출해도 서로 오염되지 않아야 한다
//   (WBC provider 가 프레임마다 J → oMf → dJv 순회 호출).
TEST(RtClosedChainHandle, DriftGetterKeepsKinematicsGettersValid) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle ref(model, ccm.constraints, ccm.actuated_joint_ids, {});
  ref.Update(std::vector<double>{0.2});
  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids,
                              ref.GetFullConfiguration(), 2);
  const int n_a = rt.nv_independent();
  const std::vector<double> q_a(static_cast<std::size_t>(n_a), 0.2);
  const std::vector<double> v_a(static_cast<std::size_t>(n_a), 0.37);
  const pinocchio::FrameIndex fid = rt.GetFrameId("c1");

  // Update 직후 (UpdateDynamics 전) 의 J/oMf 스냅샷.
  ASSERT_FALSE(rt.Update(q_a).held);
  Eigen::MatrixXd J_before(6, n_a);
  rt.GetFrameJacobian(fid, pinocchio::LOCAL_WORLD_ALIGNED, J_before);
  const pinocchio::SE3 oMf_before = rt.GetFramePlacement(fid);

  // UpdateDynamics 후 인터리브: J → drift → oMf → drift. 동일 q_full 상태이므로 J/oMf 는
  // Update 직후와 일치해야 하고 (drift 유한차분 오염 복원), drift 는 재호출에 불변이어야 한다.
  ASSERT_FALSE(rt.UpdateDynamics(v_a).held);
  Eigen::MatrixXd J_after(6, n_a);
  rt.GetFrameJacobian(fid, pinocchio::LOCAL_WORLD_ALIGNED, J_after);
  Eigen::Matrix<double, 6, 1> dJv1;
  rt.GetFrameClassicalAccelerationDrift(fid, pinocchio::LOCAL_WORLD_ALIGNED, dJv1);
  const pinocchio::SE3 oMf_after = rt.GetFramePlacement(fid);
  Eigen::Matrix<double, 6, 1> dJv2;
  rt.GetFrameClassicalAccelerationDrift(fid, pinocchio::LOCAL_WORLD_ALIGNED, dJv2);

  EXPECT_LT((J_after - J_before).norm(), 1e-15) << "UpdateDynamics 후에도 J getter 유효 (F1)";
  EXPECT_LT((oMf_after.translation() - oMf_before.translation()).norm(), 1e-15);
  EXPECT_LT((oMf_after.rotation() - oMf_before.rotation()).norm(), 1e-15);
  EXPECT_EQ((dJv1 - dJv2).norm(), 0.0) << "getter 인터리브에 drift 불변";

  // OOB frame → 0 기록 (GetFramePlacement OOB 규약 대칭).
  Eigen::Matrix<double, 6, 1> dJv_oob;
  dJv_oob.setConstant(1.0);
  rt.GetFrameClassicalAccelerationDrift(static_cast<pinocchio::FrameIndex>(1u << 20),
                                        pinocchio::LOCAL_WORLD_ALIGNED, dJv_oob);
  EXPECT_EQ(dJv_oob.norm(), 0.0) << "범위 밖 frame → 0";

  // held tick (크기 불일치) → 직전 유효 drift 유지 (문서 계약: 소비자 last-good 정책 전제).
  ASSERT_TRUE(rt.Update(std::vector<double>{0.2, 0.3}).held);
  EXPECT_TRUE(rt.UpdateDynamics(v_a).held);
  Eigen::Matrix<double, 6, 1> dJv_held;
  rt.GetFrameClassicalAccelerationDrift(fid, pinocchio::LOCAL_WORLD_ALIGNED, dJv_held);
  EXPECT_TRUE(dJv_held.allFinite());
  EXPECT_LT((dJv_held - dJv1).norm(), 1e-15) << "held → 직전 유효 drift 반환";
}

// ── (19) 비유한 v_a → UpdateDynamics held (직전 동역학·drift byte-불변, NaN 미누출) ──────
//   Update 의 q allFinite guard 와 대칭 (#173 감사). guard 없으면 NaN v_full 이 h_a/a_drift →
//   data_.v/a 로 스며드는데 held=false 라 소비자가 fresh 로 오인해 last-good snapshot 을 오염.
TEST(RtClosedChainHandle, DynamicsHeldOnNonFiniteVelocity) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle ref(model, ccm.constraints, ccm.actuated_joint_ids, {});
  ref.Update(std::vector<double>{0.2});
  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids,
                              ref.GetFullConfiguration(), 2);
  const int n_a = rt.nv_independent();
  const std::vector<double> q_a(static_cast<std::size_t>(n_a), 0.2);
  const std::vector<double> v_a(static_cast<std::size_t>(n_a), 0.37);
  const pinocchio::FrameIndex fid = rt.GetFrameId("c1");

  // 유효 tick 으로 last-good 동역학·drift 를 채운다.
  ASSERT_FALSE(rt.Update(q_a).held);
  ASSERT_FALSE(rt.UpdateDynamics(v_a).held);
  const Eigen::MatrixXd M_good = rt.GetMassMatrix();
  const Eigen::VectorXd h_good = rt.GetNonLinearEffects();
  Eigen::Matrix<double, 6, 1> dJv_good;
  rt.GetFrameClassicalAccelerationDrift(fid, pinocchio::LOCAL_WORLD_ALIGNED, dJv_good);
  ASSERT_GT(dJv_good.norm(), 1e-8);

  // 유효 q + 비유한 v → held 격상, 동역학·drift 미갱신 (NaN 누출 없음).
  ASSERT_FALSE(rt.Update(q_a).held) << "q 는 유효 — kinematics 는 fresh";
  std::vector<double> v_bad(v_a);
  v_bad[0] = std::numeric_limits<double>::quiet_NaN();
  const rub::RtClosedChainHandle::Status dyn = rt.UpdateDynamics(v_bad);
  EXPECT_TRUE(dyn.held) << "비유한 v_a → held (Update 의 q guard 와 대칭)";
  EXPECT_LT((rt.GetMassMatrix() - M_good).norm(), 1e-15) << "M_a byte-불변";
  EXPECT_LT((rt.GetNonLinearEffects() - h_good).norm(), 1e-15) << "h_a byte-불변";
  Eigen::Matrix<double, 6, 1> dJv_after;
  rt.GetFrameClassicalAccelerationDrift(fid, pinocchio::LOCAL_WORLD_ALIGNED, dJv_after);
  EXPECT_TRUE(dJv_after.allFinite()) << "drift 에 NaN 미누출";
  EXPECT_LT((dJv_after - dJv_good).norm(), 1e-15) << "drift 직전 유효값 유지";
}
