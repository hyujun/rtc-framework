// The inekf state source's Update() allocates nothing — neither through
// `operator new` nor through a C malloc inside pinocchio, Eigen or another
// library — on every path: before the start, the start, a nominal step, a held
// sample, the stale hold, a gap and its reset, skew, a refused step, a missing
// IMU. Own binary: both gates replace process-wide entry points (alloc_gate.hpp,
// malloc_gate.hpp contract item 1).
#include "integrated_bringup/controllers/locomotion/inekf_state_source.hpp"
#include "locomotion_controller_rig.hpp"
#include "locomotion_mpc_stub.hpp"
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <string>

namespace {

using namespace loco_rig;  // NOLINT(google-build-using-namespace) — the shared rig
using loco::InekfStateSource;
using loco::MeasuredRobotState;

constexpr int64_t kT = 2'000'000;

class InekfSourceAlloc : public LocomotionControllerTest {
 protected:
  void SetUp() override {
    Configure(ShippedConfig());
    MakeStanding();
    const std::vector<int> body_to_full =
        integrated_bringup::LocomotionControllerTestAccess::ReducedToFull(*ctrl_);
    InekfStateSource::Params p;
    p.body_max_age_ns = 10'000'000;
    p.hand_max_age_ns = 50'000'000;
    p.imu_site = 0;
    p.config = loco::ParseInekfSourceConfig(cfg_["estimator"]["inekf"], "inekf");
    src_ = std::make_unique<InekfStateSource>(
        p,
        std::make_unique<loco::ImuFrameKinematics>(
            std::make_shared<const pinocchio::Model>(ctrl_->ReducedModel()),
            std::vector<std::string>{"left_sole", "right_sole"}, "imu_in_pelvis"),
        body_to_full);
  }

  // `ticks` gated Update() calls (the lanes written outside the gate); the
  // most either gate counted on one call goes into worst_.
  void Gated(int ticks, uint64_t k = 1, int64_t joint_after_ns = 0, uint32_t planned = 0b11U) {
    for (int t = 0; t < ticks; ++t) {
      now_ += kT;
      auto& body = state_.devices[kBody];
      body.imu_sequence[0] += k;
      body.imu_recv_steady_ns[0] = now_;
      body.joint_recv_steady_ns = now_ + joint_after_ns;
      state_.devices[kHand].joint_recv_steady_ns = now_;
      const loco::SourceTickInput in{planned};
      const auto& idx = ctrl_->Indices();
      const int64_t read = now_ + std::max<int64_t>(0, joint_after_ns);
      std::size_t news = 0;
      std::size_t mallocs = 0;
      {
        rtc::testing::ScopedAllocGate new_gate;
        rtc::testing::ScopedMallocGate malloc_gate;
        static_cast<void>(src_->Update(state_, idx, in, read, out_));
        news = new_gate.count();
        mallocs = malloc_gate.count();
      }
      worst_new_ = std::max(worst_new_, news);
      worst_malloc_ = std::max(worst_malloc_, mallocs);
    }
  }

  std::unique_ptr<InekfStateSource> src_;
  MeasuredRobotState out_{};
  int64_t now_{1'000'000'000'000};
  std::size_t worst_new_{0};
  std::size_t worst_malloc_{0};
};

TEST_F(InekfSourceAlloc, TheGatesSeeAnAllocationInsideALibrary) {
  std::size_t news = 0;
  std::size_t mallocs = 0;
  {
    rtc::testing::ScopedAllocGate new_gate;
    rtc::testing::ScopedMallocGate malloc_gate;
    const pinocchio::Data data(ctrl_->ReducedModel());  // allocates inside libpinocchio / Eigen
    static_cast<void>(data);
    news = new_gate.count();
    mallocs = malloc_gate.count();
  }
  EXPECT_GT(news + mallocs, 0U);
  EXPECT_GT(mallocs, 0U);
}

TEST_F(InekfSourceAlloc, EveryUpdatePathAllocatesNothing) {
  auto& imu = state_.devices[kBody].imu[0];
  const auto still = imu;
  imu[0] = 0.1;  // shaking: before the start
  Gated(20);
  EXPECT_FALSE(out_.estimator.initialized);
  imu = still;
  Gated(1);  // the start
  ASSERT_TRUE(out_.valid);
  Gated(40);  // nominal: propagate, contacts inserted, confirmed, updated
  ASSERT_EQ(0b11U, out_.estimator.estimated_mask);
  Gated(20, 1, 3 * kT);   // skew: the update skipped
  Gated(20, 1, 12 * kT);  // skew past the invalid bound
  Gated(20);
  Gated(20, 0);  // a held sample, then the stale hold
  EXPECT_EQ(loco::StateSourceStatus::kImuStale, out_.status);
  Gated(1);  // the sample after the stall: a reset
  EXPECT_FALSE(out_.estimator.initialized);
  Gated(20);
  ASSERT_TRUE(out_.valid);
  Gated(1, 11);  // a gap: the reset
  EXPECT_FALSE(out_.estimator.initialized);
  Gated(20);  // the restart and its steps
  ASSERT_TRUE(out_.valid);
  imu[0] = 400.0;  // refused steps, then their reset
  Gated(20);
  imu = still;
  Gated(20, 1, 0, 0U);                         // no planned contact: the anchors released
  state_.devices[kBody].imu_valid[0] = false;  // a missing IMU
  Gated(20);
  EXPECT_EQ(loco::StateSourceStatus::kImuMissing, out_.status);
  EXPECT_EQ(0U, worst_new_);
  EXPECT_EQ(0U, worst_malloc_);
}

}  // namespace
