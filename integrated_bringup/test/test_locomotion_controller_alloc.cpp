// locomotion_controller's RT tick allocates nothing on any of its paths:
// nominal (the QP), degraded (a short source drop), fault (damping), the reset
// and E-STOP services (the hold's waiting ticks, the clear's guard refusals,
// the first passing tick), and a re-latch; with the MPC, the stand on its plan
// and the fall back to the latched pose. Own binary: the gate replaces the global
// `operator new` (alloc_gate.hpp contract item 1).
//
// WHAT THIS GATE SEES: global `operator new` across translation units, so the
// QP solver's and pinocchio's heap use counts too. It does not see Eigen's
// aligned_malloc, which none of the fixed-size types here reach.
#include "locomotion_controller_rig.hpp"
#include "locomotion_mpc_stub.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <thread>
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

  // Gated ticks at about the control rate (the MPC's plans and their age are
  // wall time) until `done` or 1000 ticks; the most any tick allocated goes
  // into `worst`.
  void GatedTickUntil(const std::function<bool()>& done, std::size_t& worst) {
    for (int k = 0; k < 1000 && !done(); ++k) {
      worst = std::max(worst, GatedTick(false));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
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
  for (int k = 0; k < 10; ++k) {  // the hold: waiting, models updated, logs written
    worst = std::max(worst, GatedTick(false));
  }
  ASSERT_EQ(LastMode(), Mode::kWaiting);
  // The clear onto a pitched base: the guard refuses, the controller waits;
  // then the base comes up and the first passing tick latches.
  auto& pose = state_.devices[kBody].base_pose;
  pose[4] = std::sin(0.5 * 0.7);
  pose[6] = std::cos(0.5 * 0.7);
  ctrl_->ClearEstop();
  for (int k = 0; k < 10; ++k) {
    worst = std::max(worst, GatedTick(false));
  }
  ASSERT_EQ(LastMode(), Mode::kWaiting);
  ASSERT_NE(ctrl_->LastDiag().pose_guard_mask, 0U);
  pose[4] = 0.0;
  pose[6] = 1.0;
  worst = std::max(worst, GatedTick(false));  // the guard passes: the latch
  for (int k = 0; k < 10; ++k) {
    worst = std::max(worst, GatedTick(false));
  }
  ASSERT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(worst, 0U);
}

// The MPC path: the state write, the sample, the plausibility check, the plan
// copy and the contact mask, refused plans answered by the plan before them,
// then the plan lost and the pose re-latched. The
// plans come from a stub on the MPC thread (the gate counts the ticking thread
// only).
TEST_F(LocomotionAlloc, TheMpcPathAllocatesNothing) {
  Configure(MpcConfig(), rclcpp::NodeOptions{}, "test_locomotion_mpc_alloc");
  StandingStub* const s = SpawnStandingStub(*ctrl_);
  ASSERT_NE(s, nullptr);
  Activate();
  MakeStanding();
  // Until the first stand, outside the gate (the plan arrives from another thread).
  ASSERT_TRUE(TickUntil(
      [this] { return ctrl_->Source() == LocomotionController::ReferenceSource::kStand; }));
  ASSERT_EQ(ctrl_->Source(), LocomotionController::ReferenceSource::kStand);
  std::size_t worst = 0;
  for (int k = 0; k < 30; ++k) {
    worst = std::max(worst, GatedTick(false));
  }
  EXPECT_EQ(ctrl_->Source(), LocomotionController::ReferenceSource::kStand);
  s->joint_offset = 1e3;  // every new plan refused: held, then aged out (still: re-latched)
  GatedTickUntil(
      [this] {
        return ctrl_->LastDiag().plan_holds > 0 &&
               ctrl_->Source() == LocomotionController::ReferenceSource::kIdle;
      },
      worst);
  EXPECT_GT(ctrl_->LastDiag().plan_holds, 0U);
  EXPECT_EQ(ctrl_->Source(), LocomotionController::ReferenceSource::kIdle);
  s->joint_offset = 0.0;
  ASSERT_TRUE(TickUntil(
      [this] { return ctrl_->Source() == LocomotionController::ReferenceSource::kStand; }));
  s->fail = true;  // the plan ages out (max_policy_age_s): the fall back and the re-latch
  GatedTickUntil([this] { return ctrl_->Source() == LocomotionController::ReferenceSource::kIdle; },
                 worst);
  for (int k = 0; k < 20; ++k) {  // the re-latched pose, still under the gate
    worst = std::max(worst, GatedTick(false));
  }
  EXPECT_EQ(ctrl_->Source(), LocomotionController::ReferenceSource::kIdle);
  EXPECT_EQ(worst, 0U);
}

TEST(LocomotionAllocGate, PositiveControlTheGateIsArmed) {
  rtc::testing::ScopedAllocGate gate;
  std::vector<double> v;
  v.reserve(64);
  EXPECT_GT(gate.count(), 0U) << "the gate is inert — every zero above proves nothing";
}

}  // namespace
