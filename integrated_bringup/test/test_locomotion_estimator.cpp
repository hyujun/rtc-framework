// locomotion_controller with `estimator.source: inekf` on the installed g1_loco
// profile: the start on the activation tick, the start timeout, a restart of
// the estimate (a new state epoch) before and after the latch, the same q / v
// to the WBC and the MPC, the planned mask handed to the source, and the
// estimator's columns under the E-STOP and the fault.
#include "locomotion_controller_rig.hpp"
#include "locomotion_mpc_stub.hpp"

#include <gtest/gtest.h>

#include <bit>
#include <map>
#include <vector>

namespace {

using namespace loco_rig;  // NOLINT(google-build-using-namespace) — the shared rig
using integrated_bringup::LocomotionControllerTestAccess;
using loco::StateSourceStatus;
using ReferenceSource = LocomotionController::ReferenceSource;

YAML::Node InekfConfig(bool mpc) {
  YAML::Node cfg = mpc ? MpcConfig() : YAML::Clone(ShippedConfig());
  cfg["estimator"]["source"] = "inekf";
  return cfg;
}

bool SameBits(double a, double b) {
  return std::bit_cast<uint64_t>(a) == std::bit_cast<uint64_t>(b);
}

class LocomotionEstimatorTest : public LocomotionControllerTest {
 protected:
  // An IMU that is not still (the start gate refuses every sample).
  void Shake(double rate) { state_.devices[kBody].imu[0][0] = rate; }

  // The next tick reads an IMU sample 11 periods after the last: a reset.
  void SkipImuSamples() { state_.devices[kBody].imu_sequence[0] += 10; }

  const loco::MeasuredRobotState& Measured() const { return ctrl_->LastMeasuredState(); }

  Cause LastCause() const { return static_cast<Cause>(ctrl_->LastDiag().fault_cause); }
};

// ── start ───────────────────────────────────────────────────────────────────

TEST_F(LocomotionEstimatorTest, TheActivationTickStartsTheEstimateAndLatches) {
  BringUp(InekfConfig(false));
  ASSERT_NE(nullptr, dynamic_cast<const loco::InekfStateSource*>(ctrl_->StateSource()));
  state_.devices[kBody].base_pose[0] = 5.0;  // a truth lane the estimate must not follow
  last_out_ = Tick();
  EXPECT_TRUE(Measured().valid) << loco::ToString(Measured().status);
  EXPECT_TRUE(Measured().estimator.initialized);
  EXPECT_EQ(loco::StateSourceKind::kEstimated, Measured().kind);
  EXPECT_EQ(Mode::kNominal, LastMode());
  // The truth lane is never read: the base is over the soles' mean (the gauge).
  EXPECT_LT(std::abs(Measured().q[0]), 0.1);
  TickFor(20);
  EXPECT_EQ(Mode::kNominal, LastMode());
  EXPECT_EQ(0b11U, Measured().estimator.estimated_mask) << "both soles in stance";
}

TEST_F(LocomotionEstimatorTest, AWindowOfThreeWaitsTwoTicksWithoutAFault) {
  YAML::Node cfg = InekfConfig(false);
  cfg["estimator"]["inekf"]["init"]["window_ticks"] = 3;
  BringUp(cfg);
  for (int t = 1; t <= 2; ++t) {
    last_out_ = Tick();
    EXPECT_EQ(Mode::kWaiting, LastMode()) << t;
    EXPECT_EQ(StateSourceStatus::kNotInitialized, Measured().status) << t;
    EXPECT_FALSE(ctrl_->HasLatchedFault());
  }
  last_out_ = Tick();
  EXPECT_EQ(Mode::kNominal, LastMode());
}

TEST_F(LocomotionEstimatorTest, AnEstimateThatDoesNotStartInTimeLatchesTheSourceFault) {
  BringUp(InekfConfig(false));
  const auto timeout =
      static_cast<int>(std::lround(cfg_["estimator"]["inekf"]["init"]["timeout_s"].as<double>() *
                                   fx::kShippedProfileControlRateHz));
  Shake(0.1);
  for (int t = 1; t < timeout; ++t) {
    last_out_ = Tick();
    ASSERT_EQ(Mode::kWaiting, LastMode()) << t;
    ASSERT_FALSE(ctrl_->HasLatchedFault()) << t;
  }
  last_out_ = Tick();
  EXPECT_EQ(Mode::kFault, LastMode());
  EXPECT_EQ(Cause::kSource, LastCause());
  ExpectDamping(last_out_);
}

TEST_F(LocomotionEstimatorTest, AnEstimateWithoutAnImuTimesOutToo) {
  BringUp(InekfConfig(false));
  const auto timeout =
      static_cast<int>(std::lround(cfg_["estimator"]["inekf"]["init"]["timeout_s"].as<double>() *
                                   fx::kShippedProfileControlRateHz));
  state_.devices[kBody].imu_valid[0] = false;
  for (int t = 1; t < timeout; ++t) {
    last_out_ = Tick();
    ASSERT_EQ(StateSourceStatus::kImuMissing, Measured().status);
    ASSERT_FALSE(ctrl_->HasLatchedFault()) << t;
  }
  last_out_ = Tick();
  EXPECT_EQ(Cause::kSource, LastCause());
}

TEST_F(LocomotionEstimatorTest, TheStartTimeoutDoesNotRunUnderTheEstopAndRestartsAtTheClear) {
  BringUp(InekfConfig(false));
  const auto timeout =
      static_cast<int>(std::lround(cfg_["estimator"]["inekf"]["init"]["timeout_s"].as<double>() *
                                   fx::kShippedProfileControlRateHz));
  Shake(0.1);
  ctrl_->TriggerEstop();
  for (int t = 0; t < timeout + 20; ++t) {
    last_out_ = Tick();
  }
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  ctrl_->ClearEstop();
  for (int t = 1; t < timeout; ++t) {
    last_out_ = Tick();
    ASSERT_FALSE(ctrl_->HasLatchedFault()) << t;
  }
  last_out_ = Tick();
  EXPECT_EQ(Cause::kSource, LastCause());
}

// ── a restart of the estimate ───────────────────────────────────────────────

TEST_F(LocomotionEstimatorTest, ARestartAfterTheLatchFaultsOnThatTickAndRestartsTheMpcStream) {
  Configure(InekfConfig(true));
  ASSERT_NE(nullptr, SpawnStandingStub(*ctrl_));
  Activate();
  MakeStanding();
  ASSERT_TRUE(TickUntil([&] { return ctrl_->Source() == ReferenceSource::kStand; }));
  for (int rep = 0; rep < 3; ++rep) {
    const std::uint32_t stream = LocomotionControllerTestAccess::StateEpoch(*ctrl_);
    const std::uint32_t epoch = Measured().state_epoch;
    SkipImuSamples();
    last_out_ = Tick();
    EXPECT_EQ(epoch + 1, Measured().state_epoch) << rep;
    EXPECT_EQ(stream + 1, LocomotionControllerTestAccess::StateEpoch(*ctrl_)) << rep;
    EXPECT_EQ(Mode::kFault, LastMode()) << rep;
    EXPECT_EQ(Cause::kSource, LastCause()) << rep;
    EXPECT_FALSE(ctrl_->LastPlanSample().valid) << rep;  // nothing of the old state followed
    TickFor(5);
    EXPECT_EQ(Mode::kFault, LastMode());
    // The reset re-latches on the restarted estimate; no plan of the old
    // stream is followed again.
    ctrl_->ResetFault();
    ASSERT_TRUE(TickUntil([&] { return ctrl_->Source() == ReferenceSource::kStand; })) << rep;
    EXPECT_EQ(LocomotionControllerTestAccess::StateEpoch(*ctrl_),
              ctrl_->LastPlanSample().state_epoch);
  }
}

TEST_F(LocomotionEstimatorTest, ARestartBeforeTheLatchOnlyRestartsTheStream) {
  Configure(InekfConfig(true));
  ASSERT_NE(nullptr, SpawnStandingStub(*ctrl_));
  Activate();
  MakeStanding();
  ctrl_->TriggerEstop();  // nothing latches while it holds; the estimate still runs
  TickFor(5);
  ASSERT_TRUE(Measured().estimator.initialized);
  const std::uint32_t stream = LocomotionControllerTestAccess::StateEpoch(*ctrl_);
  SkipImuSamples();
  last_out_ = Tick();
  EXPECT_EQ(stream + 1, LocomotionControllerTestAccess::StateEpoch(*ctrl_));
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_EQ(Mode::kWaiting, LastMode());
}

TEST_F(LocomotionEstimatorTest, TheTruthSourceNeverChangesItsEpochAfterTheActivation) {
  BringUp();
  TickFor(10);
  const std::uint32_t epoch = Measured().state_epoch;
  SkipImuSamples();  // truth reads no IMU
  TickFor(10);
  EXPECT_EQ(epoch, Measured().state_epoch);
  EXPECT_EQ(Mode::kNominal, LastMode());
  EXPECT_EQ(0U, ctrl_->LastDiag().estimator.reset_count);
}

// ── same state to the WBC and the MPC ───────────────────────────────────────

TEST_F(LocomotionEstimatorTest, TheWbcAndTheMpcGetTheSameStateBitForBit) {
  Configure(InekfConfig(true));
  StandingStub* stub = SpawnStandingStub(*ctrl_);
  ASSERT_NE(nullptr, stub);
  Activate();
  MakeStanding();

  struct Seen {
    Eigen::VectorXd q;
    Eigen::VectorXd v;
  };

  std::map<std::uint64_t, Seen> by_sequence;
  std::vector<int> body_to_full;
  {
    const auto full_order = loco::JointOrder(ctrl_->FullModel());
    for (const auto& name : loco::JointOrder(ctrl_->ReducedModel())) {
      body_to_full.push_back(static_cast<int>(
          std::find(full_order.begin(), full_order.end(), name) - full_order.begin()));
    }
  }
  int mismatches = 0;
  for (int t = 0; t < 200; ++t) {
    state_.devices[kBody].joint_sequence = 1000 + static_cast<std::uint64_t>(t);
    last_out_ = Tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (!Measured().valid) {
      continue;
    }
    const auto& qp = LocomotionControllerTestAccess::QpState(*ctrl_);
    const auto& m = Measured();
    for (std::size_t i = 0; i < 7; ++i) {
      mismatches += SameBits(qp.q(static_cast<Eigen::Index>(i)), m.q[i]) ? 0 : 1;
    }
    for (std::size_t i = 0; i < 6; ++i) {
      mismatches += SameBits(qp.v(static_cast<Eigen::Index>(i)), m.v[i]) ? 0 : 1;
    }
    for (std::size_t j = 0; j < body_to_full.size(); ++j) {
      const auto a = static_cast<std::size_t>(body_to_full[j]);
      mismatches += SameBits(qp.q(static_cast<Eigen::Index>(7 + j)), m.q[7 + a]) ? 0 : 1;
      mismatches += SameBits(qp.v(static_cast<Eigen::Index>(6 + j)), m.v[6 + a]) ? 0 : 1;
    }
    EXPECT_EQ(m.sequence, ctrl_->LastDiag().source_sequence);
    by_sequence[m.sequence] = Seen{qp.q, qp.v};
  }
  EXPECT_EQ(0, mismatches);
  std::size_t compared = 0;
  {
    const std::lock_guard<std::mutex> lock(stub->snapshot_mutex);
    const std::size_t n = std::min(stub->snapshot_count, StandingStub::kSnapshots);
    for (std::size_t k = 0; k < n; ++k) {
      const auto& s = stub->snapshots[k];
      const auto it = by_sequence.find(s.source_sequence);
      if (it == by_sequence.end()) {
        continue;
      }
      ++compared;
      for (Eigen::Index i = 0; i < it->second.q.size(); ++i) {
        EXPECT_TRUE(SameBits(it->second.q(i), s.q[static_cast<std::size_t>(i)])) << "q" << i;
      }
      for (Eigen::Index i = 0; i < it->second.v.size(); ++i) {
        EXPECT_TRUE(SameBits(it->second.v(i), s.v[static_cast<std::size_t>(i)])) << "v" << i;
      }
    }
  }
  EXPECT_GE(compared, 3U) << "the MPC solved on the ticks' states";
  RecordProperty("snapshots_compared", static_cast<int>(compared));
}

TEST_F(LocomotionEstimatorTest, TheSourceIsGivenTheLastTicksContactMask) {
  Configure(InekfConfig(true));
  StandingStub* stub = SpawnStandingStub(*ctrl_);
  ASSERT_NE(nullptr, stub);
  Activate();
  MakeStanding();
  last_out_ = Tick();
  EXPECT_EQ(0b11U, ctrl_->LastDiag().estimator.planned_mask) << "all contacts after activation";
  ASSERT_TRUE(TickUntil([&] { return ctrl_->Source() == ReferenceSource::kStand; }));
  stub->mask = 0b01U;  // a swing of the second contact in the plan
  std::uint32_t previous = ctrl_->LastDiag().contact_mask;
  int changes = 0;
  for (int t = 0; t < 300; ++t) {
    last_out_ = Tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const auto& d = ctrl_->LastDiag();
    if (d.fault_latched) {
      break;
    }
    EXPECT_EQ(previous, d.estimator.planned_mask) << t;
    changes += d.contact_mask != previous ? 1 : 0;
    previous = d.contact_mask;
  }
  EXPECT_GE(changes, 1) << "the plan's mask reached the controller";
}

// ── telemetry under the E-STOP and the fault ────────────────────────────────

TEST_F(LocomotionEstimatorTest, TheEstimatorColumnsKeepRunningUnderTheEstopAndTheFault) {
  BringUp(InekfConfig(false));
  TickFor(5);
  ASSERT_EQ(Mode::kNominal, LastMode());
  ctrl_->TriggerEstop();
  std::uint64_t seq = ctrl_->LastDiag().estimator.imu_sequence;
  const std::uint32_t epoch = ctrl_->LastDiag().estimator.reset_count;
  for (int t = 0; t < 50; ++t) {
    last_out_ = Tick();
    const auto& d = ctrl_->LastDiag();
    EXPECT_EQ(seq + 1, d.estimator.imu_sequence) << t;
    EXPECT_EQ(epoch, d.estimator.reset_count) << t;
    EXPECT_TRUE(d.estimator.initialized) << t;
    EXPECT_EQ(Mode::kWaiting, LastMode());
    seq = d.estimator.imu_sequence;
  }
  ctrl_->ClearEstop();
  TickFor(3);
  ASSERT_EQ(Mode::kNominal, LastMode());

  SkipImuSamples();
  last_out_ = Tick();
  ASSERT_EQ(Mode::kFault, LastMode());
  seq = ctrl_->LastDiag().estimator.imu_sequence;
  const std::uint32_t resets = ctrl_->LastDiag().estimator.reset_count;
  for (int t = 0; t < 50; ++t) {
    last_out_ = Tick();
    const auto& d = ctrl_->LastDiag();
    EXPECT_EQ(seq + 1, d.estimator.imu_sequence) << t;
    EXPECT_EQ(resets, d.estimator.reset_count) << t;
    EXPECT_EQ(Mode::kFault, LastMode());
    seq = d.estimator.imu_sequence;
  }
  ctrl_->ResetFault();
  TickFor(2);
  EXPECT_EQ(Mode::kNominal, LastMode());
}

// ── configure ───────────────────────────────────────────────────────────────

TEST_F(LocomotionEstimatorTest, ConfigureRefusesAFrameOffTheRootAnotherPeriodOrATypo) {
  const auto refused = [&](const std::function<void(YAML::Node&)>& spoil, const char* what) {
    YAML::Node cfg = InekfConfig(false);
    spoil(cfg);
    auto ctrl = MakeController();
    EXPECT_ANY_THROW(ctrl->LoadConfig(cfg)) << what;
  };
  refused([](YAML::Node& c) { c["estimator"]["inekf"]["imu_frame"] = "imu_in_torso"; },
          "a frame on the waist");
  refused([](YAML::Node& c) { c["estimator"]["inekf"]["imu_frame"] = "no_such_frame"; },
          "an unknown frame");
  refused([](YAML::Node& c) { c["estimator"]["inekf"]["imu"]["sample_period_s"] = 0.001; },
          "a sample period other than the control period");
  refused([](YAML::Node& c) { c["estimator"]["source"] = "InEKF"; }, "a misspelt source");
  // The installed section with either source configures.
  auto ctrl = MakeController();
  EXPECT_NO_THROW(ctrl->LoadConfig(InekfConfig(false)));
  auto truth = MakeController();
  YAML::Node cfg = ShippedConfig();
  cfg["estimator"]["inekf"]["imu_frame"] = "imu_in_torso";  // checked under truth too
  EXPECT_ANY_THROW(truth->LoadConfig(cfg));
}

}  // namespace
