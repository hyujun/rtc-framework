// ── test_closed_chain_fk_measurement — frozen-loop FK vs closed-chain FK 실측 ──
//   #121 Phase 1 측정 하네스. `urdf.extended: true` 컨트롤러가 겪는 근사 오차를 정량화한다:
//   컨트롤러는 loop-passive 관절을 reference 형상에 **고정**한 reduced/tree 모델로 FK 를
//   구한다 (frozen-loop). ClosedChainHandle 은 actuated q_a 로부터 passive 를 loop 구속에
//   사영해 loop-consistent FK 를 구한다. 이 둘의 EE 포즈 차이를 crank-rocker(비특이 구간)
//   에서 sweep 하며 측정하고, topology 판정(actuated-상류 프레임은 무영향)을 실측으로 확인한다.
//
//   측정 결과는 Phase 2 의 RT 격리 방식(저주기 갱신 허용 오차·singular hold) 결정에 쓰인다.
//   fixture 선택: arm_with_four_bar_hand 가 topology archetype(arm 끝=무영향, hand 내부 loop)
//   이지만 four_bar 는 q_ref 대칭 특이라 FK 크기 측정이 불안정하다. crank-rocker 는 비특이
//   crank 구간을 제공하므로 크기 측정에 쓰고, 특이/미존재 topology 는 Phase 0
//   test_closed_chain_handle(FrameDownstreamOfLoop*) 에서 별도 검증한다.
#include "closure_test_fixtures.hpp"
#include "rtc_urdf_bridge/closed_chain_handle.hpp"
#include "rtc_urdf_bridge/loop_projection.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;

namespace {

// 정렬된 μs 샘플에서 percentile(선형 보간 없이 nearest-rank) 추출.
double PercentileUs(const std::vector<double>& sorted_us, double pct) {
  if (sorted_us.empty()) {
    return 0.0;
  }
  const auto n = static_cast<double>(sorted_us.size());
  auto idx = static_cast<std::size_t>(std::ceil(pct / 100.0 * n)) - 1;
  idx = std::min(idx, sorted_us.size() - 1);
  return sorted_us[idx];
}

// N 회 Update 를 반복하며 per-call μs 를 수집(수렴·비특이만) → min/mean/median/p95/p99/max 리포트.
void BenchUpdate(rub::ClosedChainHandle& handle, bool with_velocity, const char* label) {
  constexpr int kWarm = 20;
  constexpr int kN = 2000;
  for (int i = 0; i < kWarm; ++i) {
    handle.Update(std::vector<double>{0.2});  // warm-up (캐시/할당 안정화)
  }
  std::vector<double> us;
  us.reserve(kN);
  const std::vector<double> v_a = with_velocity ? std::vector<double>{0.5} : std::vector<double>{};
  for (int i = 0; i < kN; ++i) {
    const double q_a = 0.15 + 0.01 * (i % 11);  // [0.15, 0.25] 비특이 crank 구간
    const auto t0 = std::chrono::steady_clock::now();
    const rub::ClosedChainHandle::Status st = handle.Update(std::vector<double>{q_a}, v_a);
    const auto t1 = std::chrono::steady_clock::now();
    if (!st.converged || st.singular) {
      continue;
    }
    us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
  }
  std::sort(us.begin(), us.end());
  const double mean = us.empty() ? 0.0 : std::accumulate(us.begin(), us.end(), 0.0) / us.size();
  std::printf(
      "[timing] %-22s n=%zu  min=%.2f  mean=%.2f  median=%.2f  p95=%.2f  p99=%.2f  max=%.2f (us)\n",
      label, us.size(), us.empty() ? 0.0 : us.front(), mean, PercentileUs(us, 50),
      PercentileUs(us, 95), PercentileUs(us, 99), us.empty() ? 0.0 : us.back());
}

// full model q 에서 단일-DoF actuated 관절 슬롯만 q_a 로 덮어쓴다 (passive 는 고정 유지).
// = 컨트롤러의 frozen-loop reduced 모델 FK 와 동치 (locked passive 를 reference 에 baked).
Eigen::VectorXd FrozenConfig(const pinocchio::Model& model, const Eigen::VectorXd& q_frozen_base,
                             const std::string& actuated_joint, double q_a) {
  Eigen::VectorXd q = q_frozen_base;
  const auto jid = model.getJointId(actuated_joint);
  q[model.idx_qs[jid]] = q_a;  // crank-rocker j_crank 은 bounded revolute → nq==1
  return q;
}

}  // namespace

// ── 핵심 측정: frozen-loop FK vs closed-chain FK (crank-rocker, 비특이 구간) ────
TEST(ClosedChainFkMeasurement, FrozenVsClosedChainCrankRocker) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle handle(model, ccm.constraints, ccm.actuated_joint_ids, {});
  ASSERT_EQ(handle.nv_independent(), 1) << "crank-rocker 독립 좌표는 j_crank 하나";
  const std::string actuated = handle.GetIndependentJointNames().front();

  // 프레임: c1(coupler tip, loop-terminal — passive 상류) vs crank_link(actuated 상류).
  const auto fid_c1 = handle.GetFrameId("c1");
  const auto fid_crank = handle.GetFrameId("crank_link");
  ASSERT_NE(fid_c1, 0u);
  ASSERT_NE(fid_crank, 0u);
  // Phase 0 topology 판정과 측정 대상의 정합 확인.
  ASSERT_TRUE(handle.IsFrameDownstreamOfLoop(fid_c1)) << "c1 은 passive 상류 → closed FK 필요";
  ASSERT_FALSE(handle.IsFrameDownstreamOfLoop(fid_crank)) << "crank_link 는 actuated 상류만";

  // 운영점(q_base=0.2, 비특이)에서 loop-consistent 형상을 확립 → frozen 모델의 lock 기준.
  constexpr double kBase = 0.2;
  ASSERT_TRUE(handle.Update(std::vector<double>{kBase}).converged);
  ASSERT_FALSE(handle.GetStatus().singular) << "crank≈0.2 는 비특이여야 한다";
  const Eigen::VectorXd q_star = handle.GetFullConfiguration();  // passive = 운영점 lock 값

  // frozen FK 용 독립 Data (handle 내부 Data 와 분리).
  pinocchio::Data frozen_data(*model);

  // actuated q_a 를 운영점 주위로 sweep 하며 EE 포즈 차이 측정.
  double max_c1_diff = 0.0;
  double max_crank_diff = 0.0;
  double c1_diff_at_base = -1.0;
  int measured = 0;
  std::printf("\n[measure] crank-rocker frozen-loop vs closed-chain EE (m)\n");
  std::printf("[measure]   q_a      c1_diff(loop-term)   crank_diff(upstream)\n");
  for (int step = -6; step <= 6; ++step) {
    const double q_a = kBase + 0.025 * step;  // [0.05, 0.35] — 비특이 crank 구간
    const rub::ClosedChainHandle::Status st = handle.Update(std::vector<double>{q_a});
    if (!st.converged || st.singular) {
      continue;  // 특이/미수렴 형상은 측정 제외 (Phase 2 hold 정책 대상)
    }
    const Eigen::Vector3d closed_c1 = handle.GetFramePosition(fid_c1);
    const Eigen::Vector3d closed_crank = handle.GetFramePosition(fid_crank);

    const Eigen::VectorXd q_frozen = FrozenConfig(*model, q_star, actuated, q_a);
    pinocchio::forwardKinematics(*model, frozen_data, q_frozen);
    pinocchio::updateFramePlacements(*model, frozen_data);
    const Eigen::Vector3d frozen_c1 = frozen_data.oMf[fid_c1].translation();
    const Eigen::Vector3d frozen_crank = frozen_data.oMf[fid_crank].translation();

    const double c1_diff = (closed_c1 - frozen_c1).norm();
    const double crank_diff = (closed_crank - frozen_crank).norm();
    max_c1_diff = std::max(max_c1_diff, c1_diff);
    max_crank_diff = std::max(max_crank_diff, crank_diff);
    if (std::abs(q_a - kBase) < 1e-12) {
      c1_diff_at_base = c1_diff;
    }
    ++measured;
    std::printf("[measure]  %+.3f      %.6e         %.6e\n", q_a, c1_diff, crank_diff);
  }
  std::printf("[measure] max c1_diff=%.6e  max crank_diff=%.6e  (n=%d)\n\n", max_c1_diff,
              max_crank_diff, measured);

  ASSERT_GE(measured, 5) << "비특이 구간 측정점이 충분해야 한다";

  // (1) topology: actuated-상류 프레임은 frozen==closed (passive 무관) → 차이 ~0.
  EXPECT_LT(max_crank_diff, 1e-9)
      << "crank_link 는 loop-passive 하류가 아니므로 frozen/closed FK 가 같아야 한다";

  // (2) 운영점에서는 frozen lock == loop-consistent → c1 차이 ~0.
  ASSERT_GE(c1_diff_at_base, 0.0) << "운영점 측정 누락";
  EXPECT_LT(c1_diff_at_base, 1e-9) << "운영점(lock 기준)에서는 두 FK 가 일치해야 한다";

  // (3) 운영점을 벗어나면 loop-terminal 프레임 FK 가 유의하게 갈라진다 (근사 오차 존재).
  //     이 크기가 Phase 2 저주기 갱신 허용 오차 판단의 근거다.
  EXPECT_GT(max_c1_diff, 1e-3)
      << "loop-terminal FK 는 운영점을 벗어나면 frozen 근사와 갈라져야 한다 (측정 목적)";
}

// ── 대조: closed-chain FK == closure_state_publisher 재구성 (동일 사영) ────────
//   ClosedChainHandle::Update 와 ClosureStatePublisher 는 같은 ProjectPassiveToConstraint 를
//   쓴다. 동일 seed(passive=warm-start, actuated=측정값)로 두 재구성이 <1e-6 로 일치함을
//   확인해, handle FK 가 publisher 의 loop-consistent 재구성과 같은 값임을 보증한다.
TEST(ClosedChainFkMeasurement, ClosedChainMatchesPublisherReconstruction) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle handle(model, ccm.constraints, ccm.actuated_joint_ids, {});
  const std::string actuated = handle.GetIndependentJointNames().front();

  pinocchio::Data pub_data(*model);
  const rub::ProjectionOptions opts;  // handle 기본과 동일

  // warm-start seed 를 handle 과 publisher 가 각자 유지하도록 나란히 갱신하며 sweep.
  Eigen::VectorXd pub_seed = pinocchio::neutral(*model);
  for (int step = 0; step <= 8; ++step) {
    const double q_a = 0.10 + 0.03 * step;  // 비특이 crank 구간 오름차순
    const rub::ClosedChainHandle::Status st = handle.Update(std::vector<double>{q_a});
    if (!st.converged || st.singular) {
      continue;
    }
    // publisher 흐름 재현: 직전 해(pub_seed)의 actuated 슬롯만 측정값으로 덮고 passive 사영.
    pub_seed[model->idx_qs[model->getJointId(actuated)]] = q_a;
    const rub::ProjectionResult res = rub::ProjectPassiveToConstraint(
        *model, pub_data, ccm.constraints, pub_seed, ccm.actuated_joint_ids, opts);
    ASSERT_TRUE(res.converged) << "q_a=" << q_a << " publisher 사영 미수렴";
    pub_seed = res.q;  // warm-start 유지

    EXPECT_LT((handle.GetFullConfiguration() - res.q).norm(), 1e-6)
        << "q_a=" << q_a << " 에서 handle FK 와 publisher 재구성이 갈라졌다";
  }
}

// ── 연산시간: ClosedChainHandle::Update (kinematics/dynamics) per-call latency ──
//   Phase 2 격리 방식(non-RT 재사영 주기) 확정 근거이자 "Update 는 RT 불가" 계약의 실측.
//   Update 는 passive 사영(반복 Newton)+SVD(G 축약 map)+crba/rnea(축약 M/g/h)로 heap 할당·
//   반복을 포함한다. 두 경로를 잰다: (a) q_a 만(kinematics+gravity), (b) q_a+v_a(비선형효과
//   h_a — drift γ 중앙차분 포함, 최대 비용).
//   ⚠ 절대 수치는 머신 종속(빌드/부하/affinity)이라 order-of-magnitude 만 유효. RT-불가 결론은
//     이 규모 + 할당/반복이라는 알고리즘 사실에 근거한다. crank-rocker(nv=3, m=3)는 소형이므로
//     실제 다-DoF 폐쇄 손은 SVD/crba 가 nv 로 증가해 더 크다 (하한 추정).
TEST(ClosedChainFkMeasurement, UpdateComputeTime) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle handle(model, ccm.constraints, ccm.actuated_joint_ids, {});

  std::printf("\n[timing] ClosedChainHandle::Update per-call (crank-rocker nv=%d, m=%d)\n",
              handle.nv_full(), handle.constraint_dim());
  BenchUpdate(handle, /*with_velocity=*/false, "kinematics+gravity");
  BenchUpdate(handle, /*with_velocity=*/true, "full dynamics(+h_a)");
  std::printf(
      "[timing] RT budget @500Hz=2000us, @1kHz=1000us → non-RT 재사영 주기 사이징 참고\n\n");

  // machine-종속 flaky 방지: 하드 assert 없이 리포트만. 규모가 RT tick(수백 us~ms)과 비교
  // 가능함을 육안 확인용. (회귀 시 O(n) 폭주만 별도 감시하려면 median 상한을 아주 느슨히 둔다.)
  SUCCEED();
}
