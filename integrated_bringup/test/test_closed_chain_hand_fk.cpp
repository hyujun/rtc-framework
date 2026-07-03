// ── test_closed_chain_hand_fk — 컨트롤러용 closed-chain hand FK 래퍼 (#121 2c) ─
//   ClosedChainHandFk 의 신규 로직 검증: closure 감지·topology 게이트·이름 브릿지(frame+q_a)·
//   fingertip hand-root 상대 FK 가 fully-converged ClosedChainHandle oracle 과 일치.
//   (내부 RtClosedChainHandle 의 사영 정확도 자체는 rtc_urdf_bridge 에서 이미 검증.)
#include "closure_test_fixtures.hpp"  // rtc_urdf_bridge/test (include dir via CMake)
#include "integrated_bringup/support/closed_chain_hand_fk.hpp"
#include "rtc_urdf_bridge/closed_chain_handle.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace ib = integrated_bringup;
namespace rub = rtc_urdf_bridge;

namespace {

// 1-device ControllerState 로 device0.positions[0]=q 를 채운다 (crank-rocker: 독립=j_crank).
rtc::ControllerState MakeState(double q) {
  rtc::ControllerState state;
  state.num_devices = 1;
  state.devices[0].num_channels = 1;
  state.devices[0].valid = true;
  state.devices[0].positions[0] = q;
  return state;
}

}  // namespace

// ── 활성 경로: fingertip(c1) hand-root 상대 FK == converged oracle (<1mm) ────────
TEST(ClosedChainHandFk, ActiveFingertipMatchesConvergedOracle) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  // 비특이 crank 중심으로 수렴시켜 warm-start seed 확보.
  rub::ClosedChainHandle oracle(model, ccm.constraints, ccm.actuated_joint_ids, {});
  oracle.Update(std::vector<double>{0.2});
  const Eigen::VectorXd seed = oracle.GetFullConfiguration();

  ib::ClosedChainHandFk fk;
  const std::vector<std::vector<std::string>> dev_names = {{"j_crank"}};
  const std::vector<std::string> tips = {"c1"};
  const auto res = fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, seed, dev_names,
                                tips, /*hand_root=*/"");
  ASSERT_EQ(res, ib::HandFkWiringResult::kActive);
  ASSERT_TRUE(fk.active());

  const auto fid_c1 = oracle.GetFrameId("c1");
  ASSERT_NE(fid_c1, 0u);

  constexpr double kDt = 1.0 / 500.0;
  constexpr int kTicks = 1500;
  constexpr double kCenter = 0.2, kAmp = 0.08, kPeakV = 3.0;
  const double freq = kPeakV / (2.0 * M_PI * kAmp);

  double max_err = 0.0;
  int n = 0;
  for (int t = 0; t < kTicks; ++t) {
    const double q = kCenter + kAmp * std::sin(2.0 * M_PI * freq * (t * kDt));
    fk.Update(MakeState(q));
    const auto st_ref = oracle.Update(std::vector<double>{q});
    ASSERT_FALSE(fk.status().held);
    if (!st_ref.converged || st_ref.singular) {
      continue;
    }
    pinocchio::SE3 T_ft;
    ASSERT_TRUE(fk.GetFingertipHandRootPose(0, T_ft));
    const Eigen::Vector3d p_ref = oracle.GetFramePosition(fid_c1);
    max_err = std::max(max_err, (T_ft.translation() - p_ref).norm());
    ++n;
  }
  ASSERT_GT(n, 100);
  EXPECT_LT(max_err, 1e-3);
}

// ── hand-root 상대 표현: oMroot.actInv(oMtip) 와 일치 ──────────────────────────
TEST(ClosedChainHandFk, HandRootRelativePose) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  rub::ClosedChainHandle oracle(model, ccm.constraints, ccm.actuated_joint_ids, {});
  oracle.Update(std::vector<double>{0.2});
  const Eigen::VectorXd seed = oracle.GetFullConfiguration();

  ib::ClosedChainHandFk fk;
  const auto res = fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, seed, {{"j_crank"}},
                                std::vector<std::string>{"c1"},
                                /*hand_root=*/"base_link");
  ASSERT_EQ(res, ib::HandFkWiringResult::kActive);

  fk.Update(MakeState(0.2));
  oracle.Update(std::vector<double>{0.2});

  const auto fid_c1 = oracle.GetFrameId("c1");
  const auto fid_root = oracle.GetFrameId("base_link");
  ASSERT_NE(fid_root, 0u);
  const pinocchio::SE3 expected =
      oracle.GetFramePlacement(fid_root).actInv(oracle.GetFramePlacement(fid_c1));

  pinocchio::SE3 got;
  ASSERT_TRUE(fk.GetFingertipHandRootPose(0, got));
  EXPECT_LT((got.translation() - expected.translation()).norm(), 1e-6);
  EXPECT_LT((got.rotation() - expected.rotation()).norm(), 1e-6);
}

// ── topology 게이트: loop-상류 프레임(crank_link)은 보정 불필요 → 비활성 ────────
TEST(ClosedChainHandFk, UpstreamFingertipInactive) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  ib::ClosedChainHandFk fk;
  const auto res = fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, ccm.q_ref,
                                {{"j_crank"}}, std::vector<std::string>{"crank_link"},
                                /*hand_root=*/"");
  EXPECT_EQ(res, ib::HandFkWiringResult::kInactiveNoDownstream);
  EXPECT_FALSE(fk.active());
}

// ── 브릿지 미완성: 독립 관절이 device joint_state_names 에 없음 → 비활성 ────────
TEST(ClosedChainHandFk, BridgeIncompleteInactive) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  ib::ClosedChainHandFk fk;
  const auto res = fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, ccm.q_ref,
                                {{"some_other_joint"}}, std::vector<std::string>{"c1"},
                                /*hand_root=*/"");
  EXPECT_EQ(res, ib::HandFkWiringResult::kInactiveBridgeIncomplete);
  EXPECT_FALSE(fk.active());
  EXPECT_EQ(fk.missing_joint(), "j_crank");
}

// ── closure 없음(plain URDF): 구속 비면 비활성 → serial 경로 (byte-for-byte) ────
TEST(ClosedChainHandFk, NoClosureInactive) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  ib::ClosedChainHandFk fk;
  const auto res = fk.Configure(model, /*constraints=*/{}, /*actuated=*/{}, /*seed=*/{},
                                {{"j_crank"}}, std::vector<std::string>{"c1"}, /*hand_root=*/"");
  EXPECT_EQ(res, ib::HandFkWiringResult::kInactiveNoClosure);
  EXPECT_FALSE(fk.active());

  // 비활성 Update / query 는 안전한 no-op.
  fk.Update(MakeState(0.2));
  pinocchio::SE3 out;
  EXPECT_FALSE(fk.GetFingertipHandRootPose(0, out));
}
