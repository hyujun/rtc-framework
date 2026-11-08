// ── HandJointController: allocation on the RT tick (RT-1) ───────────────────
//
// Compute() is the control tick. Two gates, in this one translation unit:
//   - the operator-new gate sees every C++ allocation of the process;
//   - the C-level gate (rtc_base's malloc_gate) also sees what an operator-new
//     counter cannot: Eigen's aligned allocator and allocations made inside
//     libpinocchio, which is compiled elsewhere.
//
// Claimed: after the first tick (the seed and the walk-in), every tick of the
// closed-chain path — fresh projection, held projection, unreadable device,
// E-STOP, a goal being applied, the lane switched off — makes NO allocation
// under either gate. The closed-chain handle, the borrowed fingertip FK and
// the law are all preallocated at configure; this is where that is measured.
// The positive control proves both gates fire on an allocation made inside a
// library, which is the kind the claim is about.
#include "hand_joint_test_rig.hpp"
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

namespace {

using namespace hand_joint_rig;  // NOLINT(google-build-using-namespace)

struct Counts {
  std::size_t op_new{0};
  std::size_t c_malloc{0};
};

Counts GatedTick(HandJointController& ctrl, const ControllerState& state) {
  Counts counts;
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    const ControllerOutput out = ctrl.Compute(state);
    (void)out;
    counts.op_new = new_gate.count();
    counts.c_malloc = malloc_gate.count();
  }
  return counts;
}

TEST(HandJointAlloc, TheGatesSeeAnAllocationMadeInsideALibrary) {
  const Rig rig = ClosedRig();
  const pinocchio::Model& model = *rig.builder->GetFullModel();
  std::size_t op_new = 0;
  std::size_t c_malloc = 0;
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    const pinocchio::Data data(model);  // allocates inside libpinocchio / Eigen
    (void)data;
    op_new = new_gate.count();
    c_malloc = malloc_gate.count();
  }
  EXPECT_GT(op_new + c_malloc, 0u) << "neither gate saw a library allocation — the claims "
                                      "below would be vacuous";
}

TEST(HandJointAlloc, EveryTickAfterTheFirstIsAllocationFree) {
  const Rig rig = ClosedRig();
  auto ctrl = BringUp(rig);
  ASSERT_EQ(ctrl->ConfigError(), "");
  // Seed + walk-in, ungated: the first tick's right to settle is not the claim.
  (void)ctrl->Compute(MakeState(rig.q, true, 1));

  std::uint64_t t = 2;
  auto expect_clean = [&](const ControllerState& state, const char* what) {
    const Counts c = GatedTick(*ctrl, state);
    EXPECT_EQ(c.op_new, 0u) << what << ": operator new on the tick (RT-1)";
    EXPECT_EQ(c.c_malloc, 0u) << what << ": malloc on the tick (RT-1)";
  };
  // Walk-in ticks (held) and the fresh ticks after them.
  for (int i = 0; i < 60; ++i) {
    expect_clean(MakeState(rig.q, true, t++), "walk-in / fresh");
  }
  ASSERT_TRUE(ctrl->LastTick().model_fresh) << "the rig did not converge in 60 ticks";
  // Unreadable device.
  expect_clean(MakeState(rig.q, false, t++), "unreadable");
  // Non-finite measurement (held projection).
  std::vector<double> bad = rig.q;
  bad[0] = std::numeric_limits<double>::quiet_NaN();
  expect_clean(MakeState(bad, true, t++), "non-finite");
  // A goal: pushed outside the gate (the ingress is non-RT), applied inside.
  std::vector<double> goal = rig.q;
  for (double& v : goal) {
    v += 0.05;
  }
  ctrl->SetDeviceTarget(0, goal);
  for (int i = 0; i < 20; ++i) {
    expect_clean(MakeState(rig.q, true, t++), "goal");
  }
  // Lane off.
  auto gains = ctrl->GetGains();
  gains.ff_enabled = false;
  ctrl->SetGains(gains);
  expect_clean(MakeState(rig.q, true, t++), "lane off");
  // E-STOP and release.
  ctrl->TriggerEstop();
  expect_clean(MakeState(rig.q, true, t++), "estop");
  ctrl->ClearEstop();
  expect_clean(MakeState(rig.q, true, t++), "release (reseed)");
}

TEST(HandJointAlloc, ThePlainPathIsAllocationFreeToo) {
  const Rig rig = PlainRig();
  auto ctrl = BringUp(rig);
  ASSERT_EQ(ctrl->ConfigError(), "");
  (void)ctrl->Compute(MakeState(rig.q, true, 1));
  for (std::uint64_t t = 2; t < 12; ++t) {
    const Counts c = GatedTick(*ctrl, MakeState(rig.q, true, t));
    EXPECT_EQ(c.op_new, 0u);
    EXPECT_EQ(c.c_malloc, 0u);
  }
}

}  // namespace
