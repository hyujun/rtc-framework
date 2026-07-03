// ── test_rt_closed_chain_handle — RT-safe fixed-step loop projection FK ────────
//   #121 Phase 2b. RtClosedChainHandle 는 ClosedChainHandle(non-RT, 수렴 while+SVD) 의
//   FK 부분을 warm-start + 고정 K Newton + preallocated LDLT 로 RT-safe 재구현한다.
//   검증: (1) 500Hz 궤적에서 fully-converged FK 등가(<1mm), (2) 축약 J_a 등가,
//   (3) serial 항등(구속 없음 → 개방 체인 FK 와 동일), (4) size 불일치 → hold,
//   (5) 특이 조립형상 → singular flag + 유한 유지.
#include "closure_test_fixtures.hpp"
#include "rtc_urdf_bridge/closed_chain_handle.hpp"
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

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

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;

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
