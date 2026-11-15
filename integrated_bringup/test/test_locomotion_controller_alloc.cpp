// locomotion_controller's RT tick allocates nothing on any of its paths:
// nominal (the QP), degraded (a short source drop), fault (damping), the reset
// and E-STOP services, and a re-latch. Own binary: the gate replaces the global
// `operator new` (alloc_gate.hpp contract item 1).
//
// WHAT THIS GATE SEES: global `operator new` across translation units, so the
// QP solver's and pinocchio's heap use counts too. It does not see Eigen's
// aligned_malloc, which none of the fixed-size types here reach.
#include "locomotion_controller_rig.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace {

using namespace loco_rig;  // NOLINT(google-build-using-namespace) — the shared rig

class LocomotionAlloc : public LocomotionControllerTest {
 protected:
  // Ticks one call under the gate; the stamps are written before it.
  std::size_t GatedTick(bool stale) {
    const int64_t now = NowNs();
    state_.iteration += 1;
    state_.devices[kBody].joint_recv_steady_ns = stale ? now - 1'000'000'000 : now;
    state_.devices[kBody].base_recv_steady_ns = now;
    state_.devices[kHand].joint_recv_steady_ns = now;
    rtc::testing::ScopedAllocGate gate;
    static_cast<void>(ctrl_->Compute(state_));
    return gate.count();
  }
};

TEST_F(LocomotionAlloc, EveryTickPathAllocatesNothing) {
  BringUp();
  Tick();  // warm the solver and the latches outside the gate
  Tick();
  std::size_t worst = 0;
  for (int k = 0; k < 50; ++k) {  // nominal: the QP
    worst = std::max(worst, GatedTick(false));
  }
  ASSERT_EQ(LastMode(), Mode::kNominal);
  const int n = cfg_["fault"]["max_source_bad_ticks"].as<int>();
  for (int k = 0; k <= n + 3; ++k) {  // degraded, then the latch and damping
    worst = std::max(worst, GatedTick(true));
  }
  ASSERT_EQ(LastMode(), Mode::kFault);
  ctrl_->ResetFault();                        // off-RT hook, outside the gate
  worst = std::max(worst, GatedTick(false));  // the reset and the re-latch
  ASSERT_EQ(LastMode(), Mode::kNominal);
  ctrl_->TriggerEstop();
  worst = std::max(worst, GatedTick(false));
  ctrl_->ClearEstop();
  worst = std::max(worst, GatedTick(false));
  for (int k = 0; k < 10; ++k) {
    worst = std::max(worst, GatedTick(false));
  }
  ASSERT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(worst, 0U);
}

TEST(LocomotionAllocGate, PositiveControlTheGateIsArmed) {
  rtc::testing::ScopedAllocGate gate;
  std::vector<double> v;
  v.reserve(64);
  EXPECT_GT(gate.count(), 0U) << "the gate is inert — every zero above proves nothing";
}

}  // namespace
