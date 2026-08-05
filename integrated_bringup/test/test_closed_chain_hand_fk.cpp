// ── test_closed_chain_hand_fk — 컨트롤러용 closed-chain hand FK 래퍼 (#121 2c) ─
//   ClosedChainHandFk 의 신규 로직 검증: closure 감지·topology 게이트·이름 브릿지(frame+q_a)·
//   fingertip hand-root 상대 FK 가 fully-converged ClosedChainHandle oracle 과 일치, 그리고
//   리뷰 수정(#1 혼합 손 서비스, #2 생성 실패 graceful, #3 hand_root 필수, #4/#5 status/validity
//   소비→hold). (내부 RtClosedChainHandle 의 사영 정확도 자체는 rtc_urdf_bridge 에서 검증.)
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

// crank-rocker: 독립 = j_crank (device0/ch0). base_link 을 hand-root 로 사용(활성 필수 조건).
constexpr const char* kHandRoot = "base_link";

// 1-device ControllerState 로 device0.positions[0]=q 를 채운다.
rtc::ControllerState MakeState(double q, bool valid = true) {
  rtc::ControllerState state;
  state.num_devices = 1;
  state.devices[0].num_channels = 1;
  state.devices[0].valid = valid;
  state.devices[0].positions[0] = q;
  return state;
}

// warm-start seed = 비특이 crank 중심 수렴 형상.
Eigen::VectorXd ConvergedSeed(const rub::ClosedChainModel& ccm,
                              const std::shared_ptr<pinocchio::Model>& model) {
  rub::ClosedChainHandle h(model, ccm.constraints, ccm.actuated_joint_ids, {});
  h.Update(std::vector<double>{0.2});
  return h.GetFullConfiguration();
}

}  // namespace

// ── 활성 경로: fingertip(c1) hand-root 상대 FK == converged oracle (<1mm) ────────
TEST(ClosedChainHandFk, ActiveFingertipMatchesConvergedOracle) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  const Eigen::VectorXd seed = ConvergedSeed(ccm, model);

  rub::ClosedChainHandle oracle(model, ccm.constraints, ccm.actuated_joint_ids, {});
  oracle.Update(std::vector<double>{0.2});

  ib::ClosedChainHandFk fk;
  const auto res = fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, seed, {{"j_crank"}},
                                std::vector<std::string>{"c1"}, kHandRoot);
  ASSERT_EQ(res, ib::HandFkWiringResult::kActive);

  const auto fid_c1 = oracle.GetFrameId("c1");
  const auto fid_root = oracle.GetFrameId(kHandRoot);
  ASSERT_NE(fid_c1, 0u);
  ASSERT_NE(fid_root, 0u);

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
    if (!st_ref.converged || st_ref.singular) {
      continue;
    }
    pinocchio::SE3 T_ft;
    ASSERT_TRUE(fk.GetFingertipHandRootPose(0, T_ft));
    // oracle 도 hand-root(base_link) 상대로 비교.
    const pinocchio::SE3 ref =
        oracle.GetFramePlacement(fid_root).actInv(oracle.GetFramePlacement(fid_c1));
    max_err = std::max(max_err, (T_ft.translation() - ref.translation()).norm());
    ++n;
  }
  ASSERT_GT(n, 100);
  EXPECT_LT(max_err, 1e-3);
}

// ── #1: 활성 시 혼합 손 — loop-상류 fingertip 도 (serial 등가로) 서비스된다 ──────
TEST(ClosedChainHandFk, MixedHandServesUpstreamFingertip) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  const Eigen::VectorXd seed = ConvergedSeed(ccm, model);
  rub::ClosedChainHandle oracle(model, ccm.constraints, ccm.actuated_joint_ids, {});
  oracle.Update(std::vector<double>{0.2});

  ib::ClosedChainHandFk fk;
  // c1 = loop 하류, crank_link = actuated 상류(비하류) — 둘 다 서비스돼야 한다.
  const auto res = fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, seed, {{"j_crank"}},
                                std::vector<std::string>{"c1", "crank_link"}, kHandRoot);
  ASSERT_EQ(res, ib::HandFkWiringResult::kActive);

  fk.Update(MakeState(0.2));
  oracle.Update(std::vector<double>{0.2});

  const auto fid_root = oracle.GetFrameId(kHandRoot);
  pinocchio::SE3 p_c1, p_crank;
  EXPECT_TRUE(fk.GetFingertipHandRootPose(0, p_c1)) << "loop-하류 fingertip 서비스";
  EXPECT_TRUE(fk.GetFingertipHandRootPose(1, p_crank)) << "#1: 비하류 fingertip 도 서비스";

  const pinocchio::SE3 ref_crank = oracle.GetFramePlacement(fid_root).actInv(
      oracle.GetFramePlacement(oracle.GetFrameId("crank_link")));
  EXPECT_LT((p_crank.translation() - ref_crank.translation()).norm(), 1e-9)
      << "비하류 fingertip 은 full-model FK(serial 등가)";
}

// ── #4/#5: 신뢰 불가 tick(소스 device invalid)은 직전 유효 pose 를 hold ──────────
TEST(ClosedChainHandFk, UntrustworthyTickHoldsLastGood) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  const Eigen::VectorXd seed = ConvergedSeed(ccm, model);

  ib::ClosedChainHandFk fk;
  ASSERT_EQ(fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, seed, {{"j_crank"}},
                         std::vector<std::string>{"c1"}, kHandRoot),
            ib::HandFkWiringResult::kActive);

  fk.Update(MakeState(0.2));  // 신뢰 tick → 캐시 채움
  pinocchio::SE3 good;
  ASSERT_TRUE(fk.GetFingertipHandRootPose(0, good));

  // 소스 device invalid → 신뢰 불가 → 직전 pose 유지.
  fk.Update(MakeState(0.25, /*valid=*/false));
  pinocchio::SE3 held;
  ASSERT_TRUE(fk.GetFingertipHandRootPose(0, held));
  EXPECT_LT((held.translation() - good.translation()).norm(), 1e-15) << "hold 직전 유효 해";
}

// ── #284 후속: 소스 슬롯이 이번 메시지에 안 써졌으면 그것도 신뢰 불가다 ────────────
//   위 케이스와 같은 결론을 **다른 축**으로 요구한다. device 는 `valid` 이고 채널 수도 충분해
//   pre-#284 두 항은 통과하는데, 그 슬롯은 reorder map 이 건드리지 않아 positions[] 에 직전
//   값이 남아 있다. 그 값은 유한하고 그럴듯하므로 사영은 조용히 성공하고, warm-start seed 가
//   stale 형상으로 commit 된다 — 이 래퍼가 sources_ok 로 막으려던 바로 그 오염이 새 축으로
//   들어온 것이다. 값을 0.25 로 *바꿔* 두는 것이 이 테스트의 급소다: hold 가 아니라 갱신이
//   일어났다면 pose 가 움직이므로 단언이 진다.
TEST(ClosedChainHandFk, HoledSourceSlotHoldsLastGood) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  const Eigen::VectorXd seed = ConvergedSeed(ccm, model);

  ib::ClosedChainHandFk fk;
  ASSERT_EQ(fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, seed, {{"j_crank"}},
                         std::vector<std::string>{"c1"}, kHandRoot),
            ib::HandFkWiringResult::kActive);

  fk.Update(MakeState(0.2));
  pinocchio::SE3 good;
  ASSERT_TRUE(fk.GetFingertipHandRootPose(0, good));

  rtc::ControllerState holed = MakeState(0.25);
  holed.devices[0].hole_mask = 1ULL << 0;  // 이 tick 에 슬롯 0 은 안 써졌다
  ASSERT_TRUE(holed.devices[0].valid) << "폭·유효성 축은 통과해야 이 테스트가 구멍 축을 잰다";
  ASSERT_GE(holed.devices[0].num_channels, 1);

  fk.Update(holed);
  pinocchio::SE3 held;
  ASSERT_TRUE(fk.GetFingertipHandRootPose(0, held));
  EXPECT_LT((held.translation() - good.translation()).norm(), 1e-15)
      << "구멍 난 슬롯의 직전 값을 측정값으로 사영했다";
}

// ── review #3: loop-untrustworthy tick 에도 비하류 fingertip 은 계속 서비스된다 ────
//   비하류(serial 등가) tip 의 pose 는 actuated q 만의 함수라 loop 미수렴/특이와 무관. threshold=0
//   으로 모든 tick 을 loop-untrustworthy 로 만들어 하류=hold / 비하류=live 를 확인한다.
TEST(ClosedChainHandFk, NonDownstreamServedWhenLoopUntrustworthy) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  const Eigen::VectorXd seed = ConvergedSeed(ccm, model);
  rub::ClosedChainHandle oracle(model, ccm.constraints, ccm.actuated_joint_ids, {});
  oracle.Update(std::vector<double>{0.2});

  ib::ClosedChainHandFk fk;
  // c1 = loop 하류, crank_link = 비하류. closure_error_threshold=0 → loop 은 항상 untrustworthy.
  ASSERT_EQ(fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, seed, {{"j_crank"}},
                         std::vector<std::string>{"c1", "crank_link"}, kHandRoot,
                         /*closure_error_threshold=*/0.0),
            ib::HandFkWiringResult::kActive);

  fk.Update(MakeState(0.2));
  pinocchio::SE3 p_c1, p_crank;
  EXPECT_FALSE(fk.GetFingertipHandRootPose(0, p_c1)) << "하류 c1: loop-untrustworthy → hold(무효)";
  ASSERT_TRUE(fk.GetFingertipHandRootPose(1, p_crank))
      << "#3: 비하류 crank_link 는 finite tick 이면 live";

  const auto fid_root = oracle.GetFrameId(kHandRoot);
  const pinocchio::SE3 ref_crank = oracle.GetFramePlacement(fid_root).actInv(
      oracle.GetFramePlacement(oracle.GetFrameId("crank_link")));
  EXPECT_LT((p_crank.translation() - ref_crank.translation()).norm(), 1e-9);
}

// ── #3: 유효 closure+하류지만 hand_root 미해결 → 비활성 (serial fallback) ────────
TEST(ClosedChainHandFk, NoHandRootInactive) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  ib::ClosedChainHandFk fk;
  EXPECT_EQ(fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, ccm.q_ref, {{"j_crank"}},
                         std::vector<std::string>{"c1"}, /*hand_root=*/""),
            ib::HandFkWiringResult::kInactiveNoHandRoot);
  EXPECT_FALSE(fk.active());
  EXPECT_EQ(fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, ccm.q_ref, {{"j_crank"}},
                         std::vector<std::string>{"c1"}, /*hand_root=*/"no_such_link"),
            ib::HandFkWiringResult::kInactiveNoHandRoot);
}

// ── #2: ill-posed closure(actuated 없음 → n_a==0)는 생성 실패를 graceful 처리 ────
TEST(ClosedChainHandFk, ConstructionFailedInactive) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  ib::ClosedChainHandFk fk;
  // 구속 有인데 actuated_joint_ids 비움 → RtClosedChainHandle 생성자 throw → catch.
  EXPECT_EQ(fk.Configure(model, ccm.constraints, /*actuated=*/{}, ccm.q_ref, {{"j_crank"}},
                         std::vector<std::string>{"c1"}, kHandRoot),
            ib::HandFkWiringResult::kInactiveConstructionFailed);
  EXPECT_FALSE(fk.active());
}

// ── topology 게이트: 어떤 fingertip 도 loop 하류 아님 → 비활성 ─────────────────
TEST(ClosedChainHandFk, NoDownstreamInactive) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  ib::ClosedChainHandFk fk;
  EXPECT_EQ(fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, ccm.q_ref, {{"j_crank"}},
                         std::vector<std::string>{"crank_link"}, kHandRoot),
            ib::HandFkWiringResult::kInactiveNoDownstream);
  EXPECT_FALSE(fk.active());
}

// ── 브릿지 미완성: 독립 관절이 device joint_state_names 에 없음 → 비활성 ────────
TEST(ClosedChainHandFk, BridgeIncompleteInactive) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  ib::ClosedChainHandFk fk;
  const auto res = fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, ccm.q_ref,
                                {{"some_other_joint"}}, std::vector<std::string>{"c1"}, kHandRoot);
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
                                {{"j_crank"}}, std::vector<std::string>{"c1"}, kHandRoot);
  EXPECT_EQ(res, ib::HandFkWiringResult::kInactiveNoClosure);
  EXPECT_FALSE(fk.active());

  // 비활성 Update / query 는 안전한 no-op.
  fk.Update(MakeState(0.2));
  pinocchio::SE3 out;
  EXPECT_FALSE(fk.GetFingertipHandRootPose(0, out));
}
