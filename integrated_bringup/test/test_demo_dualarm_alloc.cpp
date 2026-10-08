// ── DemoDualArmController: allocation on the RT tick (RT-1) ─────────────────
//
// Compute() is the control tick. Two gates, in this one translation unit:
//   - the operator-new gate sees every C++ allocation of the process;
//   - the C-level gate (rtc_base's malloc_gate) also sees what an operator-new
//     counter cannot: Eigen's aligned allocator, and allocations made inside
//     libpinocchio and the solver library, which are compiled elsewhere.
//
// What is CLAIMED, and what is only RECORDED:
//   - every tick, in every branch, makes no operator-new allocation;
//   - every tick that does NOT run the QP solve — E-STOP, latched fault,
//     unreadable body, not seeded yet, a call the solve refuses before it
//     solves — makes no C-level allocation either;
//   - so does the goal-application step on its own (frame conversion,
//     trajectory start, reference advance). It shares a tick with the solve,
//     so it is measured through the controller's test seam;
//   - a tick that DOES solve: the solver backend's C-level allocations are
//     counted and recorded as a test property, not asserted. They are the
//     solver's (issue #654), not this binding's.
//
// The positive controls prove each gate fires on an allocation made INSIDE a
// library, which is the kind the claims above are about.

#include "dualarm_test_rig.hpp"
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace dualarm_rig;  // NOLINT(google-build-using-namespace)
using integrated_bringup::DualArmTestAccess;
using integrated_bringup::TaskGoal;
using integrated_bringup::TaskGoalReject;

struct Counts {
  std::size_t op_new{0};
  std::size_t c_malloc{0};
};

/// One tick with both gates around Compute() alone (the harness's own
/// bookkeeping — vectors, the state copy — stays outside).
Counts GatedTick(Harness& h) {
  h.FillState();
  Counts counts;
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    h.out = h.ctrl->Compute(h.state);
    counts.op_new = new_gate.count();
    counts.c_malloc = malloc_gate.count();
  }
  h.AfterCompute();
  return counts;
}

pinocchio::SE3 Shifted(const pinocchio::SE3& pose, const Eigen::Vector3d& delta) {
  return {pose.rotation(), pose.translation() + delta};
}

TEST(DualArmAlloc, TheGatesSeeAnAllocationMadeInsideALibrary) {
  const pinocchio::Model& model = *Builder()->GetFullModel();
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
  EXPECT_GT(c_malloc, 0U) << "the C-level gate did not see a library allocation";
  // Eigen's aligned allocator does not go through operator new: this is the
  // difference the two gates are here for.
  RecordProperty("positive_control_op_new", static_cast<int>(op_new));
  RecordProperty("positive_control_c_malloc", static_cast<int>(c_malloc));
}

TEST(DualArmAlloc, TicksThatSolveMakeNoOperatorNewAllocation) {
  Harness h;
  // Warm-up outside the gate: the first solves size the backend's workspace.
  h.Run(20);

  std::size_t worst_new = 0;
  std::size_t worst_malloc = 0;
  const auto record = [&](const Counts& counts) {
    worst_new = std::max(worst_new, counts.op_new);
    worst_malloc = std::max(worst_malloc, counts.c_malloc);
  };

  // At rest.
  for (int i = 0; i < 50; ++i) {
    record(GatedTick(h));
    ASSERT_TRUE(h.ctrl->LastTick().converged);
  }
  // The tick that takes a goal in ANOTHER frame than the task's base frame
  // (the conversion path), and the motion after it — both tasks moving.
  const pinocchio::SE3 right_in_world =
      FramePose(kBodyQ, kHandQ, "world", kRoot)
          .act(Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.04, 0.0, 0.03}));
  const pinocchio::SE3 left_goal =
      Shifted(FramePose(kBodyQ, kHandQ, kLeftBase, kLeftFrame), {0.0, 0.03, 0.0});
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(0, TaskGoalMsg(right_in_world, "world")),
            TaskGoalReject::kNone);
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(1, TaskGoalMsg(left_goal)), TaskGoalReject::kNone);
  // Group goals on the same tick: the posture target and the hand trajectory.
  std::vector<double> posture = kBodyQ;
  posture[6] += 0.2;
  h.ctrl->SetDeviceTarget(0, posture);
  h.ctrl->SetDeviceTarget(1, std::vector<double>{0.6, 0.2, 0.5, 0.7});
  for (int i = 0; i < 400; ++i) {
    record(GatedTick(h));
    ASSERT_TRUE(h.ctrl->LastTick().clik_ran);
  }
  EXPECT_EQ(h.ctrl->LastTick().tasks[0].goal_sequence, 1U);
  EXPECT_EQ(h.ctrl->LastTick().tasks[1].goal_sequence, 1U);
  // An unreadable hand: the solve still runs.
  h.hand_valid = false;
  for (int i = 0; i < 10; ++i) {
    record(GatedTick(h));
    ASSERT_TRUE(h.ctrl->LastTick().clik_ran);
  }
  h.hand_valid = true;
  // The re-seed tick after an E-STOP (seed, then solve, in one tick).
  h.ctrl->TriggerEstop();
  (void)h.Tick();
  h.ctrl->ClearEstop();
  record(GatedTick(h));
  ASSERT_TRUE(h.ctrl->LastTick().reseeded);
  ASSERT_TRUE(h.ctrl->LastTick().clik_ran);
  // A solve that fails after it was reached (the core's failure branch).
  DualArmTestAccess::Tasks(*h.ctrl)[0].goal.translation()[0] =
      std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 3; ++i) {
    record(GatedTick(h));
    ASSERT_TRUE(h.ctrl->LastTick().reached_solve);
    ASSERT_FALSE(h.ctrl->LastTick().converged);
  }

  EXPECT_EQ(worst_new, 0U) << "operator new on a tick that solves";
  // The solver backend's own allocations: recorded, not asserted (#654).
  RecordProperty("solve_tick_c_malloc_max", static_cast<int>(worst_malloc));
}

TEST(DualArmAlloc, TicksThatDoNotSolveAllocateNothing) {
  const auto expect_none = [](Harness& h, const char* what) {
    const Counts counts = GatedTick(h);
    EXPECT_FALSE(h.ctrl->LastTick().reached_solve) << what << ": this tick was meant not to solve";
    EXPECT_EQ(counts.op_new, 0U) << what << ": operator new";
    EXPECT_EQ(counts.c_malloc, 0U) << what << ": C-level allocation";
  };

  // No readable tick since activation: nothing is seeded.
  {
    Harness h;
    h.body_valid = false;
    for (int i = 0; i < 5; ++i) {
      expect_none(h, "not seeded");
    }
  }
  Harness h;
  h.Run(20);
  const pinocchio::SE3 goal =
      Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.05, 0.0, 0.0});
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(0, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  h.Run(50);

  // E-STOP, with a goal and group goals arriving during it (discarded).
  h.ctrl->TriggerEstop();
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(1, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  h.ctrl->SetDeviceTarget(1, std::vector<double>{0.6, 0.2, 0.5, 0.7});
  for (int i = 0; i < 10; ++i) {
    expect_none(h, "E-STOP");
  }
  h.ctrl->ClearEstop();
  h.Run(5);

  // An unreadable body device.
  h.body_valid = false;
  for (int i = 0; i < 10; ++i) {
    expect_none(h, "unreadable body");
  }
  h.body_valid = true;
  h.Run(5);

  // A call the solve refuses before it solves (a non-finite gain): five of
  // them, the fifth raising the latch — then the latched hold, with a goal
  // arriving under it.
  auto gains = h.ctrl->GetGains();
  gains.task_gain_linear[0] = std::numeric_limits<double>::quiet_NaN();
  h.ctrl->SetGains(gains);
  for (int i = 0; i < 5; ++i) {
    expect_none(h, "refused call");
    EXPECT_TRUE(h.ctrl->LastTick().rejected_input);
  }
  ASSERT_TRUE(h.ctrl->HasLatchedFault());
  ASSERT_EQ(h.ctrl->DeliverTaskGoal(0, TaskGoalMsg(goal)), TaskGoalReject::kNone);
  for (int i = 0; i < 10; ++i) {
    expect_none(h, "latched fault");
  }
}

TEST(DualArmAlloc, ApplyingAGoalAllocatesNothing) {
  // The step that turns an accepted goal into a reference: the conversion into
  // the base frame, the trajectory's coefficients, and the reference advance
  // with its twist. It runs on the tick the goal is applied, next to a solve,
  // so it is measured on its own here.
  Harness h;
  h.Run(20);  // the cache now holds the command state the conversion reads
  const pinocchio::SE3 right_in_world =
      FramePose(kBodyQ, kHandQ, "world", kRoot)
          .act(Shifted(FramePose(kBodyQ, kHandQ, kRoot, kRightFrame), {0.04, 0.0, 0.03}));
  const pinocchio::SE3 left_in_world =
      FramePose(kBodyQ, kHandQ, "world", kLeftBase)
          .act(Shifted(FramePose(kBodyQ, kHandQ, kLeftBase, kLeftFrame), {0.0, 0.03, 0.0}));
  TaskGoal goals[2];
  for (std::size_t k = 0; k < 2; ++k) {
    const auto msg = TaskGoalMsg(k == 0 ? right_in_world : left_in_world, "world");
    goals[k].pose = msg.task_target;
    goals[k].frame_slot = 0;  // "world" in the rig's target_frames
    goals[k].sequence = 1;
  }
  ASSERT_EQ(h.ctrl->TargetFrameNames()[0], "world");

  std::size_t op_new = 0;
  std::size_t c_malloc = 0;
  bool applied[2] = {false, false};
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    for (std::size_t k = 0; k < 2; ++k) {
      applied[k] = DualArmTestAccess::ApplyTaskGoal(*h.ctrl, k, goals[k]);
      for (int i = 0; i < 5; ++i) {
        DualArmTestAccess::AdvanceReference(*h.ctrl, k, kDt);
      }
    }
    op_new = new_gate.count();
    c_malloc = malloc_gate.count();
  }
  EXPECT_TRUE(applied[0]);
  EXPECT_TRUE(applied[1]);
  EXPECT_EQ(op_new, 0U);
  EXPECT_EQ(c_malloc, 0U) << "C-level allocation while applying a goal";
  // The step did its work: the references are on their way.
  EXPECT_TRUE(DualArmTestAccess::Tasks(*h.ctrl)[0].traj_active);
  EXPECT_GT(DualArmTestAccess::Tasks(*h.ctrl)[1].twist_ff.norm(), 0.0);
}

}  // namespace
