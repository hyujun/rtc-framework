// ── test_rt_closed_chain_alloc — RtClosedChainHandle::Update 힙 할당 0 (RT-1) ──
//   `Update` 는 RT tick 에서 돌므로 warm-up 이후 전역 new 가 한 번도 불리면 안 된다
//   (invariants.md RT-1). #248 이 seed clamp / passive 이탈 가드를 hot path 에 추가하며
//   `pinocchio::difference` 호출이 새로 들어왔으므로, "사전 할당 버퍼만 쓴다" 는 계약을
//   추정이 아니라 센서로 고정한다. 별도 TU 인 이유는 alloc_counter.hpp 가 전역
//   operator new/delete 를 **정의**하기 때문 (TU 당 1회).
#include "alloc_counter.hpp"
#include "closure_test_fixtures.hpp"
#include "rtc_urdf_bridge/closed_chain_model.hpp"
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace rub = rtc_urdf_bridge;
using rtc::urdf::test::AllocCounter;

namespace {

// clamp 가 걸리는 큰 점프와 걸리지 않는 작은 스텝을 모두 통과시켜, 두 분기 어느 쪽도
// 할당하지 않음을 확인한다.
void ExpectNoAllocOverTicks(rub::RtClosedChainHandle& rt, double q_target, int ticks) {
  const std::vector<double> q_a{q_target};
  static_cast<void>(rt.Update(q_a));  // warm-up (첫 호출의 lazy 초기화가 있다면 여기서 소진)

  AllocCounter::Arm();
  for (int t = 0; t < ticks; ++t) {
    static_cast<void>(rt.Update(q_a));
  }
  AllocCounter::Disarm();

  EXPECT_EQ(AllocCounter::alloc_count.load(), 0)
      << "Update() 가 " << ticks << " tick 동안 힙 할당을 " << AllocCounter::alloc_count.load()
      << "회 수행 (RT-1 위반)";
}

}  // namespace

TEST(RtClosedChainAlloc, UpdateDoesNotAllocate) {
  const rub::ClosedChainModel ccm = rtc::test::CrankRocker();
  auto model = std::make_shared<pinocchio::Model>(ccm.model);

  // 조립된 seed 확보 (비특이 crank 구간). 여기까지는 non-RT 라 할당 허용.
  rub::RtClosedChainHandle warm(model, ccm.constraints, ccm.actuated_joint_ids, Eigen::VectorXd{},
                                /*num_iterations=*/2);
  for (int t = 0; t < 40; ++t) {
    static_cast<void>(warm.Update(std::vector<double>{0.2}));
  }

  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids, Eigen::VectorXd{},
                              /*num_iterations=*/2);
  for (int t = 0; t < 40; ++t) {  // seed 를 0.2 형상으로 walk-in (clamp 경유)
    static_cast<void>(rt.Update(std::vector<double>{0.2}));
  }

  // (a) 작은 스텝 — clamp 미발동 경로.
  ExpectNoAllocOverTicks(rt, 0.21, 50);

  // (b) 큰 점프 — clamp 발동 + walk-in 경로 (#248 에서 추가된 분기).
  ExpectNoAllocOverTicks(rt, 1.2, 50);
}
