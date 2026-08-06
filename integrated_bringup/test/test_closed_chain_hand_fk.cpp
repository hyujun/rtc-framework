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
#include <cstdint>
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

// ── (#175) borrowed 모드 — 사영은 남이 돌리고 이 래퍼는 정책만 적용한다 ──────────

// 두 모드가 **같은 궤적에서 비트 동일한 fingertip pose** 를 낸다 (#175 exact 항목).
//   owning 은 자기 핸들을 사영하고 borrowed 는 외부 핸들의 결과를 읽는데, 둘에 같은 q 를 먹이면
//   결과가 갈릴 이유가 없어야 한다 — 갈린다면 정책 코드가 두 벌로 갈라진 것이다.
//   ⚠ 이 테스트가 덮지 **못하는** 축: 두 모드의 입력 provenance 차이. 여기서는 같은 q 를 손으로
//   먹이므로 그 축은 원리적으로 관측되지 않는다 (컨트롤러 레벨 테스트의 몫).
TEST(ClosedChainHandFk, BorrowedMatchesOwningBitForBit) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  const Eigen::VectorXd seed = ConvergedSeed(ccm, model);

  ib::ClosedChainHandFk owning;
  ASSERT_EQ(owning.Configure(model, ccm.constraints, ccm.actuated_joint_ids, seed, {{"j_crank"}},
                             std::vector<std::string>{"c1", "crank_link"}, kHandRoot),
            ib::HandFkWiringResult::kActive);
  EXPECT_TRUE(owning.owns_projection());

  // borrowed: 사영은 테스트가 소유한 외부 핸들이 돌린다 (프로덕션에서는 provider).
  rub::RtClosedChainHandle external(model, ccm.constraints, ccm.actuated_joint_ids, seed);
  ib::ClosedChainHandFk borrowed;
  ASSERT_EQ(
      borrowed.ConfigureBorrowed(external, std::vector<std::string>{"c1", "crank_link"}, kHandRoot),
      ib::HandFkWiringResult::kActive);
  EXPECT_FALSE(borrowed.owns_projection()) << "borrowed 는 핸들을 소유하지 않는다";

  constexpr int kTicks = 200;
  constexpr double kCenter = 0.2, kAmp = 0.05;
  for (int t = 0; t < kTicks; ++t) {
    const double q = kCenter + kAmp * std::sin(0.05 * t);
    owning.Update(MakeState(q));
    // 빌려준 쪽이 사영한다 → 그 직후 status 가 운동학 스냅샷이다.
    const auto kin = external.Update(std::vector<double>{q});
    borrowed.UpdateFromProjection(external, /*projection_fresh=*/true, kin);

    for (std::size_t f = 0; f < 2; ++f) {
      pinocchio::SE3 a, b;
      ASSERT_EQ(owning.GetFingertipHandRootPose(f, a), borrowed.GetFingertipHandRootPose(f, b))
          << "tick " << t << " fingertip " << f << ": 유효성 판정이 갈렸다";
      EXPECT_EQ(a.translation(), b.translation()) << "tick " << t << " fingertip " << f;
      EXPECT_EQ(a.rotation(), b.rotation()) << "tick " << t << " fingertip " << f;
    }
  }

  // 계측 축(#175): owning 은 tick 마다 사영하고 borrowed 는 한 번도 사영하지 않는다.
  EXPECT_EQ(owning.projection_count(), static_cast<std::uint32_t>(kTicks));
  EXPECT_EQ(borrowed.projection_count(), 0U) << "borrowed 가 사영하면 2→1 이 무너진다";
}

// borrowed 모드도 정책은 같다: untrustworthy 사영이면 직전 유효 pose 를 hold ────────
TEST(ClosedChainHandFk, BorrowedHoldsWhenProjectionNotFresh) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  const Eigen::VectorXd seed = ConvergedSeed(ccm, model);

  rub::RtClosedChainHandle external(model, ccm.constraints, ccm.actuated_joint_ids, seed);
  ib::ClosedChainHandFk fk;
  ASSERT_EQ(fk.ConfigureBorrowed(external, std::vector<std::string>{"c1"}, kHandRoot),
            ib::HandFkWiringResult::kActive);

  const auto kin0 = external.Update(std::vector<double>{0.2});
  fk.UpdateFromProjection(external, true, kin0);
  pinocchio::SE3 good;
  ASSERT_TRUE(fk.GetFingertipHandRootPose(0, good));

  // 사영은 정상인데 **입력 provenance 가 아니다** → 캐시 hold. 이것이 컨트롤러가 cache-hold
  // 블록으로 돌린 사영을 fingertip 이 fresh 로 채택하지 않게 만드는 지점이다 (#175).
  //
  // 스텝은 seed clamp(kDefaultActuatedIncrement = 0.05) 안에 둔다 — 넘기면 사영이 walk-in 으로
  // held 가 되어 아래 "provenance 복귀 = 즉시 반영" 이 held 때문에 통과/실패하게 되고, 재려던
  // provenance 축이 아니라 클램프 축을 재게 된다.
  constexpr double kStep = 0.23;  // |0.23 - 0.2| = 0.03 < 0.05
  const auto kin1 = external.Update(std::vector<double>{kStep});
  ASSERT_FALSE(kin1.held) << "클램프에 걸리면 이 테스트는 provenance 축을 재지 못한다";
  fk.UpdateFromProjection(external, /*projection_fresh=*/false, kin1);
  pinocchio::SE3 held;
  ASSERT_TRUE(fk.GetFingertipHandRootPose(0, held));
  EXPECT_EQ(held.translation(), good.translation())
      << "provenance 가 아니면 사영이 신선해도 채택하면 안 된다";

  // provenance 가 돌아오면 즉시 반영된다 (hold 가 latch 되지 않는다).
  const auto kin2 = external.Update(std::vector<double>{kStep});
  ASSERT_FALSE(kin2.held);
  fk.UpdateFromProjection(external, true, kin2);
  pinocchio::SE3 fresh;
  ASSERT_TRUE(fk.GetFingertipHandRootPose(0, fresh));
  EXPECT_NE(fresh.translation(), good.translation());

  // 운동학 스냅샷이 held 면 (예: 사영 입력 비유한) 역시 직전 값을 지킨다.
  const auto kin_held = external.Update(std::vector<double>{std::nan("")});
  ASSERT_TRUE(kin_held.held);
  fk.UpdateFromProjection(external, true, kin_held);
  pinocchio::SE3 after_nan;
  ASSERT_TRUE(fk.GetFingertipHandRootPose(0, after_nan));
  EXPECT_EQ(after_nan.translation(), fresh.translation()) << "held 사영은 채택하지 않는다";
}

// 프레임 해결 실패는 borrowed 에서도 같은 사유로 비활성이 된다 (owning 과 대칭) ──────
TEST(ClosedChainHandFk, BorrowedFrameResolutionFailures) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  const Eigen::VectorXd seed = ConvergedSeed(ccm, model);
  rub::RtClosedChainHandle external(model, ccm.constraints, ccm.actuated_joint_ids, seed);

  {
    ib::ClosedChainHandFk fk;
    EXPECT_EQ(fk.ConfigureBorrowed(external, std::vector<std::string>{"c1"}, "no_such_link"),
              ib::HandFkWiringResult::kInactiveNoHandRoot);
    EXPECT_FALSE(fk.active());
  }
  {
    ib::ClosedChainHandFk fk;
    // crank_link 은 loop 비하류 — 하류가 하나도 없으면 serial 이 이미 정확하다.
    EXPECT_EQ(fk.ConfigureBorrowed(external, std::vector<std::string>{"crank_link"}, kHandRoot),
              ib::HandFkWiringResult::kInactiveNoDownstream);
    EXPECT_FALSE(fk.active());
  }
}

// ── borrowed 래퍼는 공용 dispatch 로 구동되지 않는다 (PR #374 리뷰) ──────────────
//   `RunHandForwardKinematics` 는 owning 전용이다. borrowed 래퍼를 넘기면 사영할 수단이 없어
//   `Update(state)` 가 즉시 반환하는데(Release 는 assert 컴파일 아웃), 그런데도 true 를 돌리면
//   호출측은 FK 가 돈 것으로 알고 **얼어붙은 pose 캐시를 유효한 TF 로 발행**한다. 증상이 "TF 가
//   안 움직인다" 뿐인 조용한 stale 이므로 withhold(false)로 떨어뜨린다.
TEST(ClosedChainHandFk, SharedDispatchWithholdsForABorrowedWrapper) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  const Eigen::VectorXd seed = ConvergedSeed(ccm, model);
  rub::RtClosedChainHandle external(model, ccm.constraints, ccm.actuated_joint_ids, seed);

  ib::ClosedChainHandFk borrowed;
  ASSERT_EQ(borrowed.ConfigureBorrowed(external, std::vector<std::string>{"c1"}, kHandRoot),
            ib::HandFkWiringResult::kActive);
  ASSERT_TRUE(borrowed.active());
  ASSERT_FALSE(borrowed.owns_projection());

  // hand_handle 은 dispatch 진입 조건일 뿐이다 (closed 활성 경로는 그것을 쓰지 않는다).
  rub::RtModelHandle hand_handle(model);
  Eigen::VectorXd hand_q = Eigen::VectorXd::Zero(model->nq);
  EXPECT_FALSE(RunHandForwardKinematics(borrowed, &hand_handle, hand_q, MakeState(0.2)))
      << "borrowed 를 공용 dispatch 로 구동하면 조용한 stale 이 된다 — withhold 여야 한다";
  EXPECT_EQ(borrowed.projection_count(), 0U) << "dispatch 가 borrowed 를 사영시켰다";

  // owning 은 같은 dispatch 로 정상 구동된다 (대조군 — 위 단언이 '무조건 false' 가 아님을 고정).
  ib::ClosedChainHandFk owning;
  ASSERT_EQ(owning.Configure(model, ccm.constraints, ccm.actuated_joint_ids, seed, {{"j_crank"}},
                             std::vector<std::string>{"c1"}, kHandRoot),
            ib::HandFkWiringResult::kActive);
  EXPECT_TRUE(RunHandForwardKinematics(owning, &hand_handle, hand_q, MakeState(0.2)));
  EXPECT_EQ(owning.projection_count(), 1U);
}

// ── 재배선은 계측 카운터도 되돌린다 (PR #374 리뷰) ──────────────────────────────
//   "borrowed 는 절대 사영하지 않는다"(2→1 의 wrapper 축 증거)가 이전 배선의 잔량 때문에 깨져
//   보이면 안 된다 — 소비자는 **현재 배선**의 사영 횟수를 묻는다.
TEST(ClosedChainHandFk, RewiringResetsTheProjectionCounter) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  const Eigen::VectorXd seed = ConvergedSeed(ccm, model);

  ib::ClosedChainHandFk fk;
  ASSERT_EQ(fk.Configure(model, ccm.constraints, ccm.actuated_joint_ids, seed, {{"j_crank"}},
                         std::vector<std::string>{"c1"}, kHandRoot),
            ib::HandFkWiringResult::kActive);
  constexpr int kTicks = 3;
  for (int t = 0; t < kTicks; ++t) {
    fk.Update(MakeState(0.2 + 0.001 * t));
  }
  ASSERT_EQ(fk.projection_count(), static_cast<std::uint32_t>(kTicks));

  rub::RtClosedChainHandle external(model, ccm.constraints, ccm.actuated_joint_ids, seed);
  ASSERT_EQ(fk.ConfigureBorrowed(external, std::vector<std::string>{"c1"}, kHandRoot),
            ib::HandFkWiringResult::kActive);
  EXPECT_EQ(fk.projection_count(), 0U)
      << "owning 시절의 잔량이 남아 borrowed 계측 단언이 잘못된 이유로 진다";

  const auto kin = external.Update(std::vector<double>{0.2});
  fk.UpdateFromProjection(external, true, kin);
  EXPECT_EQ(fk.projection_count(), 0U) << "borrowed 는 사영하지 않는다";
}
