/// @file test_clik_multiframe_malloc.cpp
/// @brief C-level allocation count of ClikReferenceGenerator's multi-frame
///        Compute() (E2-F04, #636; RT-1).
///
/// Compute() is on the RT tick. An operator-new counter cannot carry an
/// "allocates nothing" claim for it: Eigen allocates through its own aligned
/// allocator, and most of the call runs inside librtc_tsid and libpinocchio,
/// compiled elsewhere. rtc_base's malloc gate defines the malloc family in
/// this executable, so it counts those too.
///
/// What is claimed, and what is only recorded:
///   - everything Compute() does BEFORE the QP solve allocates nothing. The
///     sensor is a call that fails at the last step before the solve (a
///     non-finite coupling term in the acceleration rows), after the input
///     checks, the relative Jacobians, the cost, the box, the braking bound and
///     the rows were all built — and the failure branch itself;
///   - the solve is ProxQP's. Its allocations are counted and RECORDED as a
///     test property, not asserted (the boundary the MPC cores draw, MD-22;
///     the solver's defaults are #654's).
///
/// One translation unit: the gate defines non-inline C functions, and
/// alloc_counter.hpp the global operator new.

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include "alloc_counter.hpp"
#include "clik_test_util.hpp"
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_tsid/kinematics/clik_reference.hpp"
#include "tree_fixture.hpp"

namespace rtc::tsid {
namespace {

using Clik = ClikReferenceGenerator;
using Mode = Clik::AccelConstraint;
using Vec6 = Eigen::Matrix<double, 6, 1>;

class ClikMultiFrameMallocTest : public ::testing::Test {
 protected:
  static constexpr int kNv = test::kTreeNv;
  static constexpr double kDt = 0.002;

  void SetUp() override {
    model_ = test::LoadTreeModel();
    ASSERT_EQ(model_->nv, kNv);
    test::InitContactFreeCache(cache_, model_);
    tip_a_ = cache_.RegisterFrame("tip_a", model_->getFrameId("tip_a"));
    tip_b_ = cache_.RegisterFrame("tip_b", model_->getFrameId("tip_b"));
    torso_ = cache_.RegisterFrame("torso", model_->getFrameId("torso"));
    trunk_ = test::VelocityIndices(*model_, "trunk_");
    arm_a_ = test::VelocityIndices(*model_, "arm_a_");
    arm_b_ = test::VelocityIndices(*model_, "arm_b_");
    for (int i = 0; i < kNv; ++i) {
      all_.push_back(i);
    }
    q_home_ = test::TreeHomePosture(*model_);
    qd_ff_ = Eigen::VectorXd::Constant(kNv, 0.01);
    twist_ff_ = Vec6::Constant(0.01);
  }

  /// Every option that adds work to Compute(), except the acceleration form.
  [[nodiscard]] Clik::Config BaseConfig() const {
    Clik::Config cfg;
    cfg.arm_v_idx = all_;
    cfg.damping_sq = 1e-6;
    cfg.q_min = model_->lowerPositionLimit;
    cfg.q_max = model_->upperPositionLimit;
    cfg.v_limit_per_joint = model_->upperVelocityLimit;
    cfg.w_smooth = 1e-3;
    cfg.evaluate_at_command = true;
    cfg.max_frame_tasks = 2;
    cfg.posture_groups = {{trunk_, 3e-2}, {arm_a_, 1e-2}, {arm_b_, 2e-2}};
    return cfg;
  }

  [[nodiscard]] Clik::Config DynamicConfig() const {
    Clik::Config cfg = BaseConfig();
    cfg.relative_tasks = true;
    cfg.accel_constraint = Mode::kDynamic;
    cfg.tau_max = model_->upperEffortLimit;
    cfg.eta_tau = 0.8;
    cfg.brake_from_torque = true;
    return cfg;
  }

  [[nodiscard]] Clik::Config KinematicConfig() const {
    Clik::Config cfg = BaseConfig();
    cfg.accel_constraint = Mode::kKinematic;
    cfg.task_accel_max_linear = 8.0;
    cfg.task_accel_max_angular = 20.0;
    return cfg;
  }

  /// tip_b in the world (SE3, with feed-forward and a feedback cap) and tip_a
  /// either in the torso frame (SE3) or in the world (position + axis).
  [[nodiscard]] std::array<Clik::FrameTask, 2> Tasks(bool second_relative) {
    cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
    const auto& rf_a = cache_.registered_frames[static_cast<size_t>(tip_a_)];
    const auto& rf_b = cache_.registered_frames[static_cast<size_t>(tip_b_)];
    const auto& rf_t = cache_.registered_frames[static_cast<size_t>(torso_)];
    std::array<Clik::FrameTask, 2> tasks;
    tasks[0].kind = Clik::TaskKind::kSe3;
    tasks[0].frame_idx = tip_b_;
    tasks[0].placement_des = rf_b.oMf;
    tasks[0].placement_des.translation() += Eigen::Vector3d(0.05, -0.03, 0.04);
    tasks[0].gain = Vec6::Constant(8.0);
    tasks[0].twist_ff = &twist_ff_;
    tasks[0].fb_lin_max = 0.1;
    tasks[0].fb_ang_max = 0.1;
    tasks[1].frame_idx = tip_a_;
    tasks[1].gain = Vec6::Constant(8.0);
    tasks[1].weight = 0.4;
    if (second_relative) {
      tasks[1].kind = Clik::TaskKind::kSe3;
      tasks[1].base_frame_idx = torso_;
      tasks[1].placement_des = rf_t.oMf.actInv(rf_a.oMf);
      tasks[1].placement_des.translation() += Eigen::Vector3d(0.03, 0.03, -0.03);
      tasks[1].fb_lin_max = 0.1;
    } else {
      tasks[1].kind = Clik::TaskKind::kPositionAxis;
      tasks[1].target.position = rf_a.oMf.translation() + Eigen::Vector3d(0.03, 0.03, -0.03);
      tasks[1].target.axis = rf_a.oMf.rotation().col(2);
      tasks[1].gain_axis = 8.0;
      tasks[1].fb_ang_max = 0.1;
    }
    return tasks;
  }

  [[nodiscard]] Clik::MultiFrameInput Input(std::span<const Clik::FrameTask> tasks) const {
    Clik::MultiFrameInput in;
    in.tasks = tasks;
    in.q_posture_des = &q_home_;
    in.qd_posture_ff = &qd_ff_;
    in.dt = kDt;
    return in;
  }

  /// A few solved ticks in command mode; leaves the cache at the next state.
  void WarmUp(Clik& gen, const Clik::MultiFrameInput& in) {
    for (int g = 0; g < 3; ++g) {
      ASSERT_TRUE(gen.SetPostureGroupGain(g, 0.5));
    }
    Eigen::VectorXd q = q_home_;
    Eigen::VectorXd v = Eigen::VectorXd::Zero(kNv);
    for (int k = 0; k < 5; ++k) {
      cache_.Update(q, v);
      ASSERT_TRUE(gen.Compute(cache_, in)) << k;
      q = gen.QRef();
      v = gen.VRef();
    }
    cache_.Update(q, v);
  }

  struct Counts {
    bool ok{false};
    std::size_t c_malloc{0};
    std::int64_t operator_new{0};
  };

  [[nodiscard]] Counts Measure(Clik& gen, const Clik::MultiFrameInput& in) {
    Counts c;
    test::AllocCounter::Arm();
    {
      rtc::testing::ScopedMallocGate gate;
      c.ok = gen.Compute(cache_, in);
      c.c_malloc = gate.count();
    }
    test::AllocCounter::Disarm();
    c.operator_new = test::AllocCounter::alloc_count.load();
    return c;
  }

  std::shared_ptr<pinocchio::Model> model_;
  PinocchioCache cache_;
  int tip_a_{-1};
  int tip_b_{-1};
  int torso_{-1};
  std::vector<int> trunk_;
  std::vector<int> arm_a_;
  std::vector<int> arm_b_;
  std::vector<int> all_;
  Eigen::VectorXd q_home_;
  Eigen::VectorXd qd_ff_;
  Vec6 twist_ff_;
};

// Contract item 4 of the gate: the positive control allocates INSIDE a
// library, or a zero below would prove only what an operator-new counter does.
TEST_F(ClikMultiFrameMallocTest, TheGateSeesAllocationsMadeInsideALibrary) {
  std::size_t count = 0;
  {
    rtc::testing::ScopedMallocGate gate;
    const pinocchio::Data data(*model_);
    count = gate.count();
  }
  EXPECT_GT(count, 0U);
}

TEST_F(ClikMultiFrameMallocTest, EverythingBeforeTheSolveAllocatesNothingWithTorqueRows) {
  Clik gen;
  gen.Init(kNv, DynamicConfig());
  const std::array<Clik::FrameTask, 2> tasks = Tasks(true);
  const Clik::MultiFrameInput in = Input(tasks);
  WarmUp(gen, in);
  ASSERT_FALSE(HasFatalFailure());

  // An OFF-diagonal inertia term: the braking bound reads the diagonal only,
  // so the call gets through it and fails in the torque rows — the last step
  // before the solve.
  cache_.M(arm_a_[0], arm_a_[1]) = std::numeric_limits<double>::quiet_NaN();
  const Counts c = Measure(gen, in);
  EXPECT_FALSE(c.ok);
  EXPECT_TRUE(gen.LastSolve().non_finite);
  EXPECT_FALSE(gen.LastSolve().reached_solve) << "premise: the solver did not run";
  EXPECT_FALSE(gen.LastSolve().rejected_input) << "premise: not refused at the input checks";
  EXPECT_FALSE(gen.LastSolve().command_mismatch);
  EXPECT_EQ(gen.LastSolve().tasks, 2);
  EXPECT_NE(gen.LastSolve().fb_saturated, 0U) << "premise: the cap path ran";
  EXPECT_EQ(c.c_malloc, 0U);
  EXPECT_EQ(c.operator_new, 0);
}

TEST_F(ClikMultiFrameMallocTest, EverythingBeforeTheSolveAllocatesNothingWithTaskRows) {
  Clik gen;
  gen.Init(kNv, KinematicConfig());
  const std::array<Clik::FrameTask, 2> tasks = Tasks(false);
  const Clik::MultiFrameInput in = Input(tasks);
  WarmUp(gen, in);
  ASSERT_FALSE(HasFatalFailure());

  // A non-finite drift of the second task's frame: its stacked rows are the
  // last thing built before the solve.
  cache_.registered_frames[static_cast<size_t>(tip_a_)].dJv(1) =
      std::numeric_limits<double>::quiet_NaN();
  const Counts c = Measure(gen, in);
  EXPECT_FALSE(c.ok);
  EXPECT_TRUE(gen.LastSolve().non_finite);
  EXPECT_FALSE(gen.LastSolve().reached_solve) << "premise: the solver did not run";
  EXPECT_EQ(gen.LastSolve().tasks, 2);
  EXPECT_EQ(c.c_malloc, 0U);
  EXPECT_EQ(c.operator_new, 0);
}

TEST_F(ClikMultiFrameMallocTest, AFullCallAllocatesOnlyInsideTheQpSolver) {
  struct Case {
    const char* name;
    Clik::Config cfg;
    bool second_relative;
  };

  Clik::Config box = BaseConfig();
  box.relative_tasks = true;
  const std::array<Case, 3> cases = {{{"dynamic", DynamicConfig(), true},
                                      {"kinematic", KinematicConfig(), false},
                                      {"box", box, true}}};
  for (const Case& c : cases) {
    Clik gen;
    gen.Init(kNv, c.cfg);
    const std::array<Clik::FrameTask, 2> tasks = Tasks(c.second_relative);
    const Clik::MultiFrameInput in = Input(tasks);
    WarmUp(gen, in);
    ASSERT_FALSE(HasFatalFailure()) << c.name;
    const Counts counts = Measure(gen, in);
    EXPECT_TRUE(counts.ok) << c.name;
    EXPECT_TRUE(gen.LastSolve().reached_solve) << c.name;
    // The C++ side of the whole call, solver included.
    EXPECT_EQ(counts.operator_new, 0) << c.name;
    // The C side: what is left once the pre-solve path is known to be zero
    // (the two tests above) is the solver's. Recorded, not asserted.
    RecordProperty(std::string("c_malloc_full_call_") + c.name, static_cast<int>(counts.c_malloc));
  }
}

}  // namespace
}  // namespace rtc::tsid
