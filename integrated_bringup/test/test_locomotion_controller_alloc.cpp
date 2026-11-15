// locomotion_controller's RT tick allocates nothing on any of its paths:
// nominal (the QP), degraded (a short source drop), fault (damping), the reset
// and E-STOP services, and a re-latch; with the MPC, the stand on its plan and
// the fall back to the latched pose. Own binary: the gate replaces the global
// `operator new` (alloc_gate.hpp contract item 1).
//
// WHAT THIS GATE SEES: global `operator new` across translation units, so the
// QP solver's and pinocchio's heap use counts too. It does not see Eigen's
// aligned_malloc, which none of the fixed-size types here reach.
#include "locomotion_controller_rig.hpp"
#include "locomotion_mpc_stub.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
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

// The MPC path: the state write, the sample, the plausibility check, the plan
// copy and the contact mask, then the plan lost and the pose re-latched. The
// plans come from a stub on the MPC thread (the gate counts the ticking thread
// only).
TEST_F(LocomotionAlloc, TheMpcPathAllocatesNothing) {
  cfg_ = MpcConfig();
  rclcpp::NodeOptions opts;
  opts.use_global_arguments(false);
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_locomotion_mpc_alloc", "", opts);
  ctrl_ = std::make_unique<LocomotionController>("");
  ctrl_->SetSystemModelConfig(fx::ShippedModelConfig(kProfile));
  ctrl_->SetSharedModelBuilder(fx::ShippedModelBuilder(kProfile));
  ctrl_->SetControlRate(fx::kShippedProfileControlRateHz);
  ASSERT_EQ(ctrl_->PreConfigure(node_, cfg_), CallbackReturn::SUCCESS);
  ctrl_->SetDeviceNameConfigs(ShippedDevices(cfg_));
  ASSERT_EQ(ctrl_->on_configure(rclcpp_lifecycle::State{}, node_, cfg_), CallbackReturn::SUCCESS);
  const double weight =
      pinocchio::computeTotalMass(ctrl_->FullModel()) * ctrl_->FullModel().gravity.linear().norm();
  auto stub = std::make_unique<StandingStub>(weight);
  StandingStub* const s = stub.get();
  ASSERT_TRUE(integrated_bringup::LocomotionControllerTestAccess::Spawn(*ctrl_, std::move(stub)));
  Activate();
  MakeStanding();
  Tick();
  Tick();
  // Until the first stand, outside the gate (the plan arrives from another thread).
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (ctrl_->Source() != LocomotionController::ReferenceSource::kStand &&
         std::chrono::steady_clock::now() < end) {
    Tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_EQ(ctrl_->Source(), LocomotionController::ReferenceSource::kStand);
  std::size_t worst = 0;
  for (int k = 0; k < 30; ++k) {
    worst = std::max(worst, GatedTick(false));
  }
  EXPECT_EQ(ctrl_->Source(), LocomotionController::ReferenceSource::kStand);
  s->fail = true;  // the plan ages out: the fall back and the re-latch
  for (int k = 0; k < 120; ++k) {
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
