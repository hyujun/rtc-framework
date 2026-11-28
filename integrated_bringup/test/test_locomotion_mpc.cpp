// locomotion_controller with its MPC (mpc.enabled): the reference sources
// (idle → stand on a usable plan, back to idle or the plan-loss fault when the
// plan is lost), the plausibility check, the contact mask, the state stream's
// restarts (reset, E-STOP, activation), the thread's lifetime, the hand hold,
// the layout-profile refusal, and the `mpc:` schema. The plans come from a stub
// handler on the real MPC thread (one producer, as in the run); one smoke test
// runs the installed MPC itself.
#include "locomotion_mpc_stub.hpp"
#include "rtc_controllers/testing/bit_compare.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using namespace loco_rig;  // NOLINT(google-build-using-namespace) — the shared rig
using integrated_bringup::LocomotionControllerTestAccess;
using integrated_bringup::LocomotionDiagLogPod;
using Source = LocomotionController::ReferenceSource;
using namespace std::chrono_literals;

class LocomotionMpcTest : public LocomotionControllerTest {
 protected:
  // The rig's bring-up with the MPC on and the stub's thread spawned before
  // the activation (which then only resumes it).
  void BringUpWithStub(YAML::Node cfg = MpcConfig()) {
    Configure(cfg, rclcpp::NodeOptions{}, "test_locomotion_mpc");
    ASSERT_TRUE(ctrl_->MpcEnabled());
    stub_ = SpawnStandingStub(*ctrl_);
    ASSERT_NE(stub_, nullptr);
    Activate();
    MakeStanding();
  }

  bool Standing() { return ctrl_->Source() == Source::kStand; }

  /// The base height MakeStanding stands the rig at — what the shipped
  /// profile's `mpc.activation_height: measured` puts the command at.
  double StandHeight() const { return state_.devices[kBody].base_pose[2]; }

  void ReachStand() {
    ASSERT_TRUE(TickUntil([this] { return Standing(); })) << SourceName();
  }

  std::string SourceName() const {
    std::string s = "source " + std::to_string(static_cast<int>(ctrl_->Source())) + ", plan " +
                    std::to_string(ctrl_->LastDiag().plan_id) + ", rejects " +
                    std::to_string(ctrl_->LastDiag().plausibility_rejects);
    const auto& m = ctrl_->MpcManager();
    s += ", received " + std::to_string(m.HasEverReceivedSolution()) +
         ", refused epoch/state/age/dim " + std::to_string(m.EpochRejections()) + "/" +
         std::to_string(m.StateEpochRejections()) + "/" + std::to_string(m.PolicyAgeRejections()) +
         "/" + std::to_string(m.DimensionRejections());
    if (const auto* t = dynamic_cast<const rtc::mpc::HandlerMPCThread*>(ctrl_->MpcThread())) {
      s += ", solves " + std::to_string(t->TotalSolves()) + " failed " +
           std::to_string(t->FailedSolves()) + " (last " + std::to_string(t->LastSolveErrorCode()) +
           ") withheld " + std::to_string(t->WithheldCount()) + " cold withheld " +
           std::to_string(t->ColdWithheldCount()) + " deadline " +
           std::to_string(t->DeadlineMissCount());
    }
    return s;
  }

  StandingStub* stub_{nullptr};
};

// ── The reference sources ────────────────────────────────────────────────────

TEST_F(LocomotionMpcTest, AConvergedStandingPlanIsFollowedBlockForBlock) {
  BringUpWithStub();
  ReachStand();
  const auto& ref = ctrl_->Reference();
  const auto& plan = ctrl_->LastPlanSample();
  ASSERT_TRUE(plan.valid);
  ASSERT_EQ(plan.nq, ref.q_des.size());
  for (Eigen::Index i = 0; i < ref.q_des.size(); ++i) {
    EXPECT_EQ(ref.q_des(i), plan.q_ref[static_cast<std::size_t>(i)]) << i;
  }
  for (Eigen::Index i = 0; i < ref.v_des.size(); ++i) {
    EXPECT_EQ(ref.v_des(i), plan.v_ref[static_cast<std::size_t>(i)]) << i;
    EXPECT_EQ(ref.a_des(i), plan.a_ref[static_cast<std::size_t>(i)]) << i;  // the stub: no gain
  }
  for (Eigen::Index i = 0; i < ref.lambda_des.size(); ++i) {
    EXPECT_EQ(ref.lambda_des(i), plan.wrench_ref[static_cast<std::size_t>(i)]) << i;
  }
  EXPECT_EQ(LastMode(), Mode::kNominal);
  const auto& d = ctrl_->LastDiag();
  EXPECT_EQ(d.ref_source, static_cast<std::uint8_t>(Source::kStand));
  EXPECT_EQ(d.plan_id, plan.plan_id);
  EXPECT_TRUE(d.mpc_converged);
  EXPECT_EQ(d.mpc_solve_us, 1.234);
  EXPECT_GE(d.policy_age_us, 0.0);
  EXPECT_EQ(d.contact_mask, 0b11U);
  // The body's position target is the QP's own state, carried on from the
  // tick before (header, "The servo target") — not the plan's pose.
  const auto before = SnapshotServoTarget();
  last_out_ = Tick();
  ExpectServoTargetStep(before, last_out_);
}

// One budget for every solve: a published plan that did not converge is still a
// usable plan, so idle -> stand needs only the plan and its contacts.
TEST_F(LocomotionMpcTest, AnUnconvergedPublishedPlanIsStoodOn) {
  BringUpWithStub();
  stub_->converged = false;
  ReachStand();
  EXPECT_EQ(ctrl_->Source(), Source::kStand) << SourceName();
  EXPECT_FALSE(ctrl_->LastDiag().mpc_converged);
  EXPECT_EQ(LastMode(), Mode::kNominal);
}

TEST_F(LocomotionMpcTest, AnUnconvergedOnlinePlanKeepsTheStand) {
  BringUpWithStub();
  ReachStand();
  stub_->converged = false;
  const auto latched_before = ctrl_->LastDiag().fault_count;
  TickFor(150);
  EXPECT_EQ(ctrl_->Source(), Source::kStand) << SourceName();
  EXPECT_FALSE(ctrl_->LastDiag().mpc_converged);
  EXPECT_EQ(ctrl_->LastDiag().fault_count, latched_before);
}

// A plan that is finite but implausible is never followed: idle, re-latched,
// counted — and it is not a fault.
TEST_F(LocomotionMpcTest, AnImplausiblePlanFallsBackToIdleWithoutAFault) {
  const std::pair<const char*, std::function<void(StandingStub&)>> spoils[] = {
      {"joint far from the measurement", [](StandingStub& s) { s.joint_offset = 1e3; }},
      {"zero quaternion", [](StandingStub& s) { s.zero_quaternion = true; }},
      {"three times the weight", [](StandingStub& s) { s.fz_scale = 3.5; }},
      {"a contact the model does not have", [](StandingStub& s) { s.mask = 0b111U; }},
      {"no support at all", [](StandingStub& s) { s.mask = 0U; }},
      {"a third of the weight", [](StandingStub& s) { s.fz_scale = 0.3; }},
      {"the base rolled past the bound", [](StandingStub& s) { s.base_tilt = 0.6; }},
      {"the base above the height range", [](StandingStub& s) { s.base_dz = 0.5; }},
      {"the base faster than the bound", [](StandingStub& s) { s.base_vx = 4.0; }},
      {"a sole pulling on the floor", [](StandingStub& s) { s.pull = 200.0; }},
  };
  for (const auto& [what, spoil] : spoils) {
    SCOPED_TRACE(what);
    BringUpWithStub();
    ReachStand();
    const auto rejects = ctrl_->LastDiag().plausibility_rejects;
    spoil(*stub_);
    ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kIdle; })) << SourceName();
    EXPECT_GT(ctrl_->LastDiag().plausibility_rejects, rejects);
    EXPECT_FALSE(ctrl_->HasLatchedFault());
    EXPECT_EQ(LastMode(), Mode::kNominal);
    TickFor(30);
    EXPECT_EQ(ctrl_->Source(), Source::kIdle) << "an implausible plan stays refused";
    // The re-latched pose is the measured one.
    const auto pose = StandPose();
    for (std::size_t j = 0; j < pose.size(); ++j) {
      EXPECT_NEAR(ctrl_->Reference().q_des(7 + static_cast<Eigen::Index>(j)), pose[j], 1e-12) << j;
    }
    ctrl_.reset();
  }
}

// A plan's joint accelerations are not judged (Plausible): a plan whose joint
// rate flips sign each node — far past any joint's acceleration rating — is
// followed, not refused.
TEST_F(LocomotionMpcTest, APlanWithLargeJointAccelerationsIsFollowed) {
  BringUpWithStub();
  ReachStand();
  const auto rejects = ctrl_->LastDiag().plausibility_rejects;
  stub_->joint_rate = 30.0;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->LastDiag().sample.a_joint_max > 1e3; }))
      << "the stub's plan carries the large acceleration";
  const auto followed = ctrl_->LastDiag().plan_id;
  // Later plans, each with the same acceleration, are taken in turn.
  ASSERT_TRUE(TickUntil([&] { return ctrl_->LastDiag().plan_id > followed; })) << SourceName();
  EXPECT_EQ(ctrl_->Source(), Source::kStand) << SourceName();
  EXPECT_EQ(ctrl_->LastDiag().plausibility_rejects, rejects);
  EXPECT_FALSE(ctrl_->HasLatchedFault());
}

// ── A fresh plan refused in stand / walk ────────────────────────────────────

class LocomotionMpcHoldTest : public LocomotionMpcTest {
 protected:
  // Standing on a plan with the stub closed: the held plan is the last one
  // taken and no other is in flight. The age bound is lifted so that the
  // test, not the clock, decides when the next plan comes.
  void StandOnAClosedStub() {
    YAML::Node cfg = MpcConfig();
    cfg["mpc"]["max_policy_age_s"] = 0.0;  // the horizon bound still applies
    BringUpWithStub(cfg);
    ReachStand();
    stub_->gate = 0;
    TickFor(60);  // a solve in flight lands and is taken
    const auto published = ctrl_->MpcManager().GetSolveStats().count;
    TickFor(30);
    ASSERT_EQ(ctrl_->MpcManager().GetSolveStats().count, published) << "the stub is closed";
    ASSERT_EQ(ctrl_->Source(), Source::kStand) << SourceName();
    held_ = ctrl_->LastDiag().plan_id;
    ASSERT_NE(held_, 0U);
  }

  // Let one solve through and wait until its plan is published (the
  // manager counts a publish after the buffer write); the next tick takes it.
  void ReleaseOnePlan() {
    const auto published = ctrl_->MpcManager().GetSolveStats().count;
    stub_->gate = 1;
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while ((stub_->gate.load() != 0 || ctrl_->MpcManager().GetSolveStats().count == published) &&
           std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_EQ(stub_->gate.load(), 0);
    ASSERT_EQ(ctrl_->MpcManager().GetSolveStats().count, published + 1);
  }

  void TearDown() override {
    if (stub_ != nullptr) {
      stub_->gate = -1;
    }
    LocomotionMpcTest::TearDown();
  }

  std::uint64_t held_{0};
};

TEST_F(LocomotionMpcHoldTest, ARefusedFreshPlanKeepsThePlanBeforeItAndTheNextPlanReplacesIt) {
  StandOnAClosedStub();
  state_.devices[kBody].base_twist[0] = 0.3;  // [m/s] moving: losing the plan would be the fault
  const auto d0 = ctrl_->LastDiag();
  stub_->joint_offset = 1e3;
  ReleaseOnePlan();
  last_out_ = Tick();
  const auto& d = ctrl_->LastDiag();
  EXPECT_EQ(d.plausibility_rejects, d0.plausibility_rejects + 1);
  EXPECT_EQ(d.plan_holds, d0.plan_holds + 1);
  EXPECT_GT(d.sample.plan_id, held_) << "the refused plan is logged";
  EXPECT_EQ(d.sample.check, 5U);
  EXPECT_EQ(d.plan_id, held_) << "the plan before it is followed";
  EXPECT_EQ(ctrl_->Source(), Source::kStand);
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  const std::uint64_t refused = d.sample.plan_id;
  TickFor(20);
  EXPECT_EQ(ctrl_->LastDiag().plan_id, held_) << "the refused plan stays refused";
  EXPECT_FALSE(ctrl_->HasLatchedFault());

  stub_->joint_offset = 0.0;
  ReleaseOnePlan();
  last_out_ = Tick();
  EXPECT_GT(ctrl_->LastDiag().plan_id, refused);
  EXPECT_EQ(ctrl_->Source(), Source::kStand);
  EXPECT_EQ(ctrl_->LastDiag().plan_holds, d0.plan_holds + 1);
}

// A plan followed for a while and refused later gives way the same way: the
// plan offset within the bound is followed, then the measurement moves so
// that it is past the bound and the plan before it is not.
TEST_F(LocomotionMpcHoldTest, APlanRefusedAfterItWasFollowedGivesWayToThePlanBeforeIt) {
  StandOnAClosedStub();
  state_.devices[kBody].base_twist[0] = 0.3;
  const int c = LocomotionControllerTestAccess::ChannelOfFirstReducedJoint(*ctrl_);
  ASSERT_GE(c, 0);
  stub_->joint_offset = 0.4;  // [rad] within joint_delta_rad (0.5)
  ReleaseOnePlan();
  TickFor(5);
  const auto d0 = ctrl_->LastDiag();
  ASSERT_GT(d0.plan_id, held_) << "the offset plan is followed";
  state_.devices[kBody].positions[static_cast<std::size_t>(c)] -=
      0.2;  // 0.6 from it, 0.2 from the one before
  last_out_ = Tick();
  const auto& d = ctrl_->LastDiag();
  EXPECT_EQ(d.sample.check, 5U);
  EXPECT_EQ(d.plan_holds, d0.plan_holds + 1);
  EXPECT_EQ(d.plan_id, held_);
  EXPECT_EQ(ctrl_->Source(), Source::kStand);
  EXPECT_FALSE(ctrl_->HasLatchedFault()) << SourceName();
}

// Only a plan this controller followed is held: a stand entered from idle on
// plan B has no plan before it, although the manager still holds the plan idle
// sampled before B (single support here, so idle did not stand on it).
TEST_F(LocomotionMpcHoldTest, APlanIdleOnlySampledIsNotHeld) {
  YAML::Node cfg = MpcConfig();
  cfg["mpc"]["max_policy_age_s"] = 0.0;
  Configure(cfg, rclcpp::NodeOptions{}, "test_locomotion_mpc");
  stub_ = SpawnStandingStub(*ctrl_);
  ASSERT_NE(stub_, nullptr);
  stub_->gate = 0;
  Activate();
  MakeStanding();
  TickFor(30);
  stub_->mask = 0b01U;
  ReleaseOnePlan();
  TickFor(5);
  ASSERT_EQ(ctrl_->Source(), Source::kIdle);
  ASSERT_NE(ctrl_->MpcManager().ActivePlanId(), 0U);
  stub_->mask = 0b11U;
  stub_->joint_offset = 0.4;
  ReleaseOnePlan();
  TickFor(5);
  ASSERT_EQ(ctrl_->Source(), Source::kStand) << SourceName();
  state_.devices[kBody].base_twist[0] = 0.3;
  const int c = LocomotionControllerTestAccess::ChannelOfFirstReducedJoint(*ctrl_);
  ASSERT_GE(c, 0);
  state_.devices[kBody].positions[static_cast<std::size_t>(c)] -= 0.2;
  last_out_ = Tick();
  EXPECT_TRUE(ctrl_->HasLatchedFault()) << SourceName();
  EXPECT_EQ(static_cast<Cause>(ctrl_->LastDiag().fault_cause), Cause::kPlanLoss);
  EXPECT_EQ(ctrl_->LastDiag().plan_holds, 0U);
}

// The plan before it is checked again on that tick: if the measurement moved
// away from it too, the plan is lost then, not a tick later.
TEST_F(LocomotionMpcHoldTest, ThePlanBeforeARefusedOneIsCheckedAgainBeforeItIsHeld) {
  StandOnAClosedStub();
  state_.devices[kBody].base_twist[0] = 0.3;
  const auto holds = ctrl_->LastDiag().plan_holds;
  ReleaseOnePlan();
  state_.devices[kBody].positions[0] += 0.6;  // [rad] past joint_delta_rad from both plans
  last_out_ = Tick();
  EXPECT_TRUE(ctrl_->HasLatchedFault()) << SourceName();
  EXPECT_EQ(static_cast<Cause>(ctrl_->LastDiag().fault_cause), Cause::kPlanLoss);
  EXPECT_EQ(ctrl_->LastDiag().plan_holds, holds);
}

// Refusals in a row: the plan before them is held until it ages out, then the
// lost-plan path runs as before (here: moving, so the fault).
TEST_F(LocomotionMpcTest, RefusalsInARowHoldThePlanOnlyUntilItAgesOut) {
  BringUpWithStub();
  ReachStand();
  state_.devices[kBody].base_twist[0] = 0.3;
  const auto age_rejects = ctrl_->MpcManager().PolicyAgeRejections();
  stub_->joint_offset = 1e3;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->HasLatchedFault(); })) << SourceName();
  EXPECT_EQ(static_cast<Cause>(ctrl_->LastDiag().fault_cause), Cause::kPlanLoss);
  EXPECT_GT(ctrl_->LastDiag().plan_holds, 0U);
  EXPECT_GT(ctrl_->MpcManager().PolicyAgeRejections(), age_rejects);
}

// Idle → stand is unchanged: a refused plan in idle holds nothing.
TEST_F(LocomotionMpcTest, InIdleARefusedPlanHoldsNothing) {
  Configure(MpcConfig(), rclcpp::NodeOptions{}, "test_locomotion_mpc");
  stub_ = SpawnStandingStub(*ctrl_);
  ASSERT_NE(stub_, nullptr);
  stub_->mask = 0b01U;  // plausible plans idle does not stand on
  Activate();
  MakeStanding();
  ASSERT_TRUE(TickUntil([this] { return ctrl_->MpcManager().HasEverReceivedSolution(); }));
  TickFor(30);
  ASSERT_EQ(ctrl_->Source(), Source::kIdle);
  stub_->joint_offset = 1e3;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->LastDiag().plausibility_rejects >= 3; }))
      << SourceName();
  EXPECT_EQ(ctrl_->LastDiag().plan_holds, 0U);
  EXPECT_EQ(ctrl_->Source(), Source::kIdle);
  EXPECT_FALSE(ctrl_->HasLatchedFault());
}

TEST_F(LocomotionMpcTest, ALostPlanReLatchesTheMeasuredPoseOnAStillDoubleSupport) {
  BringUpWithStub();
  ReachStand();
  const auto age_rejects = ctrl_->MpcManager().PolicyAgeRejections();
  stub_->fail = true;  // no new plan: the held one ages out (max_policy_age_s)
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kIdle; })) << SourceName();
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_GT(ctrl_->MpcManager().PolicyAgeRejections(), age_rejects);
  EXPECT_EQ(ctrl_->LastDiag().plan_id, 0U);
  const auto& c = LocomotionControllerTestAccess::Contacts(*ctrl_);
  EXPECT_EQ(c.active_count, 2);
  stub_->fail = false;
  ReachStand();  // and back
}

TEST_F(LocomotionMpcTest, ALostPlanOffAStillDoubleSupportLatchesThePlanLossFault) {
  BringUpWithStub();
  ReachStand();
  state_.devices[kBody].base_twist[0] = 0.3;  // [m/s] the base moving
  stub_->fail = true;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->HasLatchedFault(); })) << SourceName();
  EXPECT_EQ(static_cast<Cause>(ctrl_->LastDiag().fault_cause), Cause::kPlanLoss);
  EXPECT_EQ(ctrl_->Source(), Source::kFault);
  ExpectDamping(last_out_);
}

// ── The contact mask ─────────────────────────────────────────────────────────

TEST_F(LocomotionMpcTest, ASwingMaskTakesTheContactOutAndBackInFully) {
  BringUpWithStub();
  ReachStand();
  stub_->mask = 0b01U;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kWalk; })) << SourceName();
  {
    const auto& c = LocomotionControllerTestAccess::Contacts(*ctrl_);
    EXPECT_EQ(c.contacts[0].activation, 1.0);
    EXPECT_EQ(c.contacts[1].activation, 0.0);
    EXPECT_FALSE(c.contacts[1].active);
    EXPECT_EQ(c.active_count, 1);
    EXPECT_EQ(ctrl_->LastDiag().contact_mask, 0b01U);
  }
  stub_->mask = 0b11U;
  ReachStand();
  const auto& c = LocomotionControllerTestAccess::Contacts(*ctrl_);
  EXPECT_EQ(c.contacts[1].activation, 1.0);
  EXPECT_TRUE(c.contacts[1].active);
  EXPECT_EQ(c.active_count, 2);
}

// ── Restarts of the state stream ─────────────────────────────────────────────

TEST_F(LocomotionMpcTest, AResetRestartsTheStreamAndTheGait) {
  BringUpWithStub();
  ReachStand();
  state_.devices[kBody].base_twist[0] = 0.3;
  stub_->fail = true;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->HasLatchedFault(); }));
  state_.devices[kBody].base_twist[0] = 0.0;
  stub_->fail = false;
  const auto epoch = LocomotionControllerTestAccess::StateEpoch(*ctrl_);
  ctrl_->ResetFault();
  last_out_ = Tick();
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_EQ(LocomotionControllerTestAccess::StateEpoch(*ctrl_), epoch + 1);
  EXPECT_EQ(ctrl_->Source(), Source::kIdle);
  EXPECT_FALSE(ctrl_->LastPlanSample().valid) << "no plan of the old stream is sampled";
  EXPECT_EQ(ctrl_->LastDiag().mpc_state_epoch, epoch + 1);
  ReachStand();
  EXPECT_EQ(ctrl_->LastPlanSample().state_epoch, epoch + 1);
}

TEST_F(LocomotionMpcTest, EitherEstopEdgeRestartsTheStreamAndNoStandWhileStopped) {
  BringUpWithStub();
  ReachStand();
  const auto epoch = LocomotionControllerTestAccess::StateEpoch(*ctrl_);
  ctrl_->TriggerEstop();
  last_out_ = Tick();
  EXPECT_EQ(LocomotionControllerTestAccess::StateEpoch(*ctrl_), epoch + 1);
  EXPECT_EQ(ctrl_->Source(), Source::kIdle);
  TickFor(100);
  EXPECT_EQ(ctrl_->Source(), Source::kIdle) << "no stand while the E-STOP holds";
  ExpectDamping(last_out_);
  ctrl_->ClearEstop();
  last_out_ = Tick();
  EXPECT_EQ(LocomotionControllerTestAccess::StateEpoch(*ctrl_), epoch + 2);
  ReachStand();
}

TEST_F(LocomotionMpcTest, AReactivationSamplesNoPlanOfTheLastSession) {
  BringUpWithStub();
  ReachStand();
  const auto epoch = LocomotionControllerTestAccess::StateEpoch(*ctrl_);
  Deactivate();
  EXPECT_FALSE(ctrl_->MpcManager().Enabled());
  Activate();
  EXPECT_TRUE(ctrl_->MpcManager().Enabled());
  last_out_ = Tick();
  EXPECT_EQ(LocomotionControllerTestAccess::StateEpoch(*ctrl_), epoch + 1);
  EXPECT_EQ(ctrl_->Source(), Source::kIdle);
  EXPECT_FALSE(ctrl_->LastPlanSample().valid);
  ReachStand();
  EXPECT_EQ(ctrl_->LastPlanSample().state_epoch, epoch + 1);
}

// ── The thread ───────────────────────────────────────────────────────────────

TEST_F(LocomotionMpcTest, CleanupStopsTheThreadWithinOnePeriodAndOneSolve) {
  BringUpWithStub();
  ReachStand();
  constexpr int kSolveUs = 50'000;
  stub_->sleep_us = kSolveUs;
  TickFor(20);
  Deactivate();
  const auto t0 = std::chrono::steady_clock::now();
  ASSERT_EQ(ctrl_->on_cleanup(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  const double took = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_EQ(ctrl_->MpcThread(), nullptr);
  EXPECT_LE(took, kNodeDt + kSolveUs * 1e-6 + 1.0);
  stub_ = nullptr;
}

TEST_F(LocomotionMpcTest, UnderTheMpcTheHandsHoldTheirDefaultPose) {
  BringUpWithStub();
  auto& hand = state_.devices[kHand];
  for (int c = 0; c < hand.num_channels; ++c) {
    hand.positions[static_cast<std::size_t>(c)] = 0.3;
  }
  last_out_ = Tick();
  const auto q_default = cfg_["reference"]["q_default"];
  const auto names = ShippedDevices(cfg_).at("p1b").joint_state_names;
  const auto& out = last_out_.devices[kHand];
  ASSERT_EQ(out.num_channels, static_cast<int>(names.size()));
  for (std::size_t c = 0; c < names.size(); ++c) {
    EXPECT_EQ(out.commands[c], q_default[names[c]].as<double>()) << names[c];
  }
}

TEST_F(LocomotionMpcTest, TheMpcOffLayoutRefusesTheActivation) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({rclcpp::Parameter("rt_layout_profile", "mpc_off")});
  Configure(MpcConfig(), opts, "test_locomotion_mpc_off");
  EXPECT_EQ(ctrl_->on_activate(rclcpp_lifecycle::State{}), CallbackReturn::FAILURE);
  EXPECT_EQ(ctrl_->MpcThread(), nullptr) << "refused before anything was spawned";
}

// The installed MPC itself: the thread starts, its first plan (the standing
// solve) reaches the sampler, and the thread pauses and stops. No timing is
// asserted — whether the online solves then meet their publish deadline is the
// host's (the control PCs decide the budgets), so standing on them is not
// asserted either.
TEST_F(LocomotionMpcTest, SmokeTheInstalledMpcDeliversAPlan) {
  cfg_ = MpcConfig();
  BringUp(cfg_);
  ASSERT_NE(ctrl_->MpcThread(), nullptr);
  ASSERT_TRUE(TickUntil([this] { return ctrl_->MpcManager().HasEverReceivedSolution(); }, 120s))
      << SourceName();
  EXPECT_FALSE(ctrl_->HasLatchedFault()) << SourceName();
  EXPECT_EQ(LastMode(), Mode::kNominal);
  TickFor(20);
  Deactivate();
  ASSERT_EQ(ctrl_->on_cleanup(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_EQ(ctrl_->MpcThread(), nullptr);
}

// ── Without the MPC ──────────────────────────────────────────────────────────

void ExpectSameOutput(const rtc::ControllerOutput& a, const rtc::ControllerOutput& b) {
  ASSERT_EQ(a.num_devices, b.num_devices);
  EXPECT_EQ(a.valid, b.valid);
  EXPECT_EQ(a.command_type, b.command_type);
  const auto same = [](const auto& x, const auto& y) {
    return std::memcmp(x.data(), y.data(), sizeof(x)) == 0;
  };
  for (int d = 0; d < a.num_devices; ++d) {
    const auto& x = a.devices[static_cast<std::size_t>(d)];
    const auto& y = b.devices[static_cast<std::size_t>(d)];
    SCOPED_TRACE(d);
    EXPECT_EQ(x.num_channels, y.num_channels);
    EXPECT_EQ(x.command_type, y.command_type);
    EXPECT_EQ(x.goal_type, y.goal_type);
    EXPECT_EQ(x.has_pd_gains, y.has_pd_gains);
    EXPECT_TRUE(same(x.commands, y.commands));
    EXPECT_TRUE(same(x.feedforward, y.feedforward));
    EXPECT_TRUE(same(x.pd_kp, y.pd_kp));
    EXPECT_TRUE(same(x.pd_kd, y.pd_kd));
    EXPECT_TRUE(same(x.goal_positions, y.goal_positions));
    EXPECT_TRUE(same(x.target_positions, y.target_positions));
    EXPECT_TRUE(same(x.trajectory_positions, y.trajectory_positions));
    EXPECT_TRUE(same(x.trajectory_velocities, y.trajectory_velocities));
  }
}

// `mpc:` absent and mpc.enabled false are the same controller, bit for bit.
TEST_F(LocomotionMpcTest, NoMpcSectionAndADisabledOneGiveTheSameOutput) {
  YAML::Node without = YAML::Clone(ShippedConfig());
  without.remove("mpc");
  BringUp(without);
  ASSERT_FALSE(ctrl_->MpcEnabled());
  std::vector<rtc::ControllerOutput> a;
  for (int k = 0; k < 10; ++k) {
    state_.devices[kBody].positions[0] += 1e-3;
    a.push_back(Tick());
  }
  BringUp(ShippedConfig());  // enabled: false
  ASSERT_FALSE(ctrl_->MpcEnabled());
  ASSERT_EQ(ctrl_->MpcThread(), nullptr);
  for (int k = 0; k < 10; ++k) {
    state_.devices[kBody].positions[0] += 1e-3;
    ExpectSameOutput(a[static_cast<std::size_t>(k)], Tick());
  }
  EXPECT_EQ(ctrl_->LastDiag().plan_id, 0U);
  EXPECT_EQ(ctrl_->LastDiag().mpc_state_epoch, 0U) << "the MPC stream never started";
}

// ── The schema ───────────────────────────────────────────────────────────────

TEST(LocomotionMpcConfig, TheConsumerKeysAreRequiredAndChecked) {
  const auto refused = [](const std::function<void(YAML::Node&)>& spoil, const char* what) {
    ExpectRefused(MpcConfig(), spoil, what);
  };
  refused([](YAML::Node& c) { c["mpc"].remove("enabled"); }, "no enabled");
  refused([](YAML::Node& c) { c["mpc"]["riccati"].remove("enabled"); }, "no riccati.enabled");
  refused([](YAML::Node& c) { c["mpc"].remove("max_policy_age_s"); }, "no max_policy_age_s");
  refused([](YAML::Node& c) { c["mpc"].remove("period_nodes"); }, "no period_nodes");
  refused([](YAML::Node& c) { c["mpc"]["sanity"].remove("wrench_scale"); }, "no wrench_scale");
  refused([](YAML::Node& c) { c["mpc"]["period_nodes"] = 0; }, "period 0");
  refused([](YAML::Node& c) { c["mpc"]["max_policy_age_s"] = -0.1; }, "negative age");
  refused([](YAML::Node& c) { c["mpc"]["sanity"]["base_speed"] = 0.0; }, "zero bound");
  refused([](YAML::Node& c) { c["mpc"]["enabled"] = "maybe"; }, "not a bool");
  refused([](YAML::Node& c) { c["mpc"]["enable"] = true; }, "misspelt key");
  refused([](YAML::Node& c) { c["mpc"]["riccati"]["gain"] = 1.0; }, "unknown riccati key");
  refused([](YAML::Node& c) { c["mpc"]["sanity"]["joint_accel"] = 500.0; }, "retired sanity key");
  const auto parsed = ParseLocomotionControllerConfig(MpcConfig());
  EXPECT_TRUE(parsed.mpc.present);
  EXPECT_TRUE(parsed.mpc.enabled);
  EXPECT_TRUE(parsed.mpc.riccati_enabled);
  EXPECT_EQ(parsed.mpc.period_nodes, 2);
  EXPECT_EQ(parsed.mpc.max_policy_age_s, 0.35);
  // The other loco profile ships no `mpc:`: no MPC.
  const auto b7 = ParseLocomotionControllerConfig(fx::ShippedControllerNode("b7_loco", kKey));
  EXPECT_FALSE(b7.mpc.present);
  EXPECT_FALSE(b7.mpc.enabled);
}

TEST_F(LocomotionMpcTest, TheStageReferenceMustNameTheHandsAndNoStrangers) {
  const auto refused = [](const std::function<void(YAML::Node&)>& spoil, const char* what) {
    YAML::Node cfg = MpcConfig();
    spoil(cfg);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_loco_qdefault");
    EXPECT_NE(MakeController()->PreConfigure(node, cfg), CallbackReturn::SUCCESS) << what;
  };
  refused([](YAML::Node& c) { c["reference"]["q_default"]["no_such_joint"] = 0.0; }, "stranger");
  refused([](YAML::Node& c) { c["reference"]["q_default"].remove("thumb_mcp_joint"); },
          "hand joint missing");
  refused([](YAML::Node& c) { c["reference"]["q_default"].remove("left_knee_joint"); },
          "body joint missing");
  refused([](YAML::Node& c) { c.remove("gait"); }, "no gait");
  refused([](YAML::Node& c) { c["reference"]["q_default"]["thumb_mcp_joint"] = 0.4; },
          "a hand hold off the dynamics model's lock");
  refused(
      [](YAML::Node& c) {
        c["command"]["sample_rate_hz"] = 2000.0;
        c["command"]["cutoff_hz"] = 2.4;
      },
      "a command timer faster than a millisecond");
  // Positive control: the installed sections build.
  BringUpWithStub();
}

// ── The velocity command and the status topic ────────────────────────────────
//
// The controller's node is spun here (the CM's nrt_callback executor in a run):
// the Twist callback, the command timer and the status subscription all run on
// it, between ticks.

using integrated_bringup::LocomotionStatusKey;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

class LocomotionCommandTest : public LocomotionMpcTest {
 protected:
  static constexpr const char* kTwistTopic = "/g1/cmd_vel";
  static constexpr const char* kStatusTopic = "/g1/locomotion_status";

  void SetUp() override {
    helper_ = std::make_shared<rclcpp::Node>("test_locomotion_command_helper");
    twist_pub_ = helper_->create_publisher<geometry_msgs::msg::Twist>(kTwistTopic, rclcpp::QoS(1));
    status_sub_ = helper_->create_subscription<DiagnosticStatus>(
        kStatusTopic, rclcpp::QoS(1), [this](DiagnosticStatus::ConstSharedPtr m) {
          last_status_ = *m;
          ++status_count_;
        });
    executor_.add_node(helper_);
  }

  void TearDown() override {
    StopSpinning();
    executor_.remove_node(helper_);
  }

  void BringUpSpinning(YAML::Node cfg = MpcConfig()) {
    BringUpWithStub(cfg);
    SpinController();
  }

  void SpinController() {
    spun_ = node_->get_node_base_interface();
    executor_.add_node(spun_);
  }

  void StopSpinning() {
    if (spun_) {
      executor_.remove_node(spun_);
      spun_.reset();
    }
  }

  // Spin the executor and tick the controller for `d`, publishing `twist` at
  // 50 Hz when given.
  void SpinFor(std::chrono::milliseconds d, const geometry_msgs::msg::Twist* twist = nullptr) {
    const auto end = std::chrono::steady_clock::now() + d;
    auto next_pub = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < end) {
      if (twist != nullptr && std::chrono::steady_clock::now() >= next_pub) {
        twist_pub_->publish(*twist);
        next_pub += 20ms;
      }
      executor_.spin_some(1ms);
      if (ctrl_) {
        last_out_ = Tick();
      }
      std::this_thread::sleep_for(2ms);
    }
  }

  static geometry_msgs::msg::Twist Twist(double vx, double vy = 0.0, double wz = 0.0) {
    geometry_msgs::msg::Twist t;
    t.linear.x = vx;
    t.linear.y = vy;
    t.angular.z = wz;
    return t;
  }

  // One snapshot publish, then the message the helper received.
  bool PublishStatus() {
    const auto before = status_count_;
    rtc::PublishSnapshot snap{};
    snap.stamp_ns = NowNs();
    ctrl_->PublishNonRtSnapshot(snap);
    const auto end = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < end && status_count_ == before) {
      executor_.spin_some(1ms);
      std::this_thread::sleep_for(2ms);
    }
    return status_count_ != before;
  }

  const std::string& Value(LocomotionStatusKey key) const {
    return last_status_.values[static_cast<std::size_t>(key)].value;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr spun_;  ///< the controller node, when added
  rclcpp::Node::SharedPtr helper_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<DiagnosticStatus>::SharedPtr status_sub_;
  DiagnosticStatus last_status_{};
  int status_count_{0};
};

TEST_F(LocomotionCommandTest, ATwistReachesTheFilterAndThePlanner) {
  BringUpSpinning();
  ASSERT_TRUE(ctrl_->VelocityCommandSubscribed());
  EXPECT_EQ(helper_->count_subscribers(kTwistTopic), 1U);
  ReachStand();
  const auto seq0 = stub_->horizon_command_sequence.load();
  const auto twist = Twist(0.3);
  SpinFor(800ms, &twist);
  EXPECT_GT(ctrl_->VelocityCommandsReceived(), 10U);
  const auto cmd = ctrl_->LastCommand();
  EXPECT_TRUE(cmd.valid);
  EXPECT_TRUE(cmd.fresh);
  EXPECT_NEAR(cmd.v_x, 0.3, 0.02) << "the Bessel stage settled on the command";
  EXPECT_EQ(cmd.v_y, 0.0);
  EXPECT_EQ(cmd.base_height, StandHeight());
  EXPECT_GT(cmd.sequence, 20U) << "one filter step per timer period (100 Hz nominal; a loaded host "
                                  "skips periods)";
  EXPECT_EQ(cmd.nonfinite_count, 0U);
  // The planner integrated it: the horizon's node 0 base velocity is the command.
  EXPECT_GT(stub_->horizon_command_sequence.load(), seq0);
  EXPECT_NEAR(stub_->horizon_base_vx.load(), cmd.v_x, 0.02);
}

TEST_F(LocomotionCommandTest, AStaleTwistDecaysToRestAndStaysValid) {
  BringUpSpinning();
  const auto twist = Twist(0.3);
  SpinFor(600ms, &twist);
  ASSERT_NEAR(ctrl_->LastCommand().v_x, 0.3, 0.02);
  SpinFor(1200ms);  // nothing published: stale after command.t_stale (0.5 s)
  const auto cmd = ctrl_->LastCommand();
  EXPECT_TRUE(cmd.valid) << "a sample was seen since the reset";
  EXPECT_FALSE(cmd.fresh);
  EXPECT_GT(cmd.stale_count, 0U);
  EXPECT_NEAR(cmd.v_x, 0.0, 1e-3);
  EXPECT_EQ(cmd.base_height, StandHeight()) << "the height stays where the activation put it";
}

TEST_F(LocomotionCommandTest, ANonFiniteAxisKeepsTheLastAcceptedValue) {
  BringUpSpinning();
  const auto good = Twist(0.3);
  SpinFor(600ms, &good);
  ASSERT_NEAR(ctrl_->LastCommand().v_x, 0.3, 0.02);
  const auto bad = Twist(std::numeric_limits<double>::quiet_NaN(), 0.2);
  SpinFor(600ms, &bad);
  const auto cmd = ctrl_->LastCommand();
  EXPECT_TRUE(cmd.fresh);
  EXPECT_NEAR(cmd.v_x, 0.3, 0.02) << "the NaN axis holds the last accepted value";
  EXPECT_NEAR(cmd.v_y, 0.2, 0.02) << "the other axes pass";
  EXPECT_GT(cmd.nonfinite_count, 0U);
  ASSERT_TRUE(PublishStatus());
  EXPECT_EQ(Value(LocomotionStatusKey::kCmdNonFiniteTicks), std::to_string(cmd.nonfinite_count))
      << "the operator's NaN shows on the status topic";
}

// The diagnostic log carries what the timer last handed over, the phase
// manager's gait with it, and the sampled plan's base velocity.
TEST_F(LocomotionCommandTest, TheDiagLogCarriesTheCommandTheGaitAndThePlanVelocity) {
  BringUpSpinning();
  ReachStand();
  const auto twist = Twist(0.3, 0.1, 0.2);
  SpinFor(800ms, &twist);
  const auto cmd = ctrl_->LastCommand();
  const auto& d = ctrl_->LastDiag();
  // The timer may step between the tick's read and LastCommand: one 10 ms
  // filter step apart at most, on a Bessel output that has settled.
  EXPECT_NEAR(d.cmd_vx, cmd.v_x, 0.01);
  EXPECT_NEAR(d.cmd_vy, cmd.v_y, 0.01);
  EXPECT_NEAR(d.cmd_wz, cmd.omega_z, 0.01);
  EXPECT_EQ(d.cmd_h, cmd.base_height);
  EXPECT_TRUE(d.cmd_valid);
  EXPECT_GE(d.gait_latest, 0) << "the phase manager's gait index";
  const auto& plan = ctrl_->LastPlanSample();
  ASSERT_TRUE(plan.valid);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(d.ref_base_v[i], plan.v_ref[i]) << i;
  }
  SpinFor(1200ms);  // stale after command.t_stale
  EXPECT_FALSE(ctrl_->LastDiag().cmd_valid) << "a decaying command is not a fresh one";
}

TEST_F(LocomotionCommandTest, WithoutATopicKeyNothingIsSubscribed) {
  YAML::Node cfg = MpcConfig();
  cfg.remove("velocity_command");
  // The shipped profile's measured activation height runs the filter's timer
  // by itself (AMeasuredActivationHeightNeedsNoCommandTopic); without it:
  cfg["mpc"]["activation_height"] = "config";
  BringUpSpinning(cfg);
  EXPECT_FALSE(ctrl_->VelocityCommandSubscribed());
  EXPECT_EQ(helper_->count_subscribers(kTwistTopic), 0U);
  const auto twist = Twist(0.3);
  SpinFor(200ms, &twist);
  EXPECT_EQ(ctrl_->VelocityCommandsReceived(), 0U);
  EXPECT_EQ(ctrl_->LastCommand().sequence, 0U) << "no timer either";
}

TEST_F(LocomotionCommandTest, WithoutAnMpcTheTopicKeyIsInert) {
  YAML::Node cfg = ShippedConfig();  // mpc.enabled false
  ASSERT_TRUE(cfg["velocity_command"]);
  BringUp(cfg);
  EXPECT_FALSE(ctrl_->MpcEnabled());
  EXPECT_FALSE(ctrl_->VelocityCommandSubscribed());
  EXPECT_EQ(helper_->count_subscribers(kTwistTopic), 0U);
  TickFor(3);
  const auto& d = ctrl_->LastDiag();
  EXPECT_FALSE(d.cmd_valid);
  EXPECT_EQ(d.gait_latest, -1) << "no phase manager";
  EXPECT_TRUE(std::isnan(d.ref_base_v[0])) << "no plan";
}

TEST_F(LocomotionCommandTest, AnActivationResetsTheCommand) {
  BringUpSpinning();
  const auto twist = Twist(0.3);
  SpinFor(600ms, &twist);
  ASSERT_NEAR(ctrl_->LastCommand().v_x, 0.3, 0.02);
  Deactivate();
  const auto seq = ctrl_->LastCommand().sequence;
  SpinFor(100ms, &twist);
  EXPECT_EQ(ctrl_->LastCommand().sequence, seq) << "no filter step while inactive";
  Activate();
  const auto cmd = ctrl_->LastCommand();
  EXPECT_EQ(cmd.sequence, 0U);
  EXPECT_FALSE(cmd.valid);
  EXPECT_EQ(cmd.v_x, 0.0);
  EXPECT_EQ(cmd.base_height, cfg_["command"]["base_height"].as<double>());
  SpinFor(600ms, &twist);  // and the path is up again
  EXPECT_NEAR(ctrl_->LastCommand().v_x, 0.3, 0.02);
}

// mpc.activation_height: measured under free_run: the tick hands the height
// to the wall timer, the filter's one writer.
TEST_F(LocomotionCommandTest, AMeasuredActivationHeightReachesTheFilterThroughTheTimer) {
  YAML::Node cfg = MpcConfig();
  cfg["mpc"]["activation_height"] = "measured";
  BringUpSpinning(cfg);
  const double stands = state_.devices[kBody].base_pose[2];
  ASSERT_NE(stands, cfg["command"]["base_height"].as<double>());
  ReachStand();
  const auto twist = Twist(0.3);
  SpinFor(600ms, &twist);
  const auto cmd = ctrl_->LastCommand();
  EXPECT_EQ(cmd.base_height, stands);
  EXPECT_NEAR(cmd.v_x, 0.3, 0.02) << "the command path runs on";
  EXPECT_EQ(ctrl_->LastDiag().cmd_h, stands);
}

TEST_F(LocomotionCommandTest, AMeasuredActivationHeightNeedsNoCommandTopic) {
  YAML::Node cfg = MpcConfig();
  cfg.remove("velocity_command");
  cfg["mpc"]["activation_height"] = "measured";
  BringUpSpinning(cfg);
  EXPECT_FALSE(ctrl_->VelocityCommandSubscribed());
  SpinFor(300ms);
  EXPECT_EQ(ctrl_->LastCommand().base_height, state_.devices[kBody].base_pose[2])
      << "the timer runs for the height alone";
}

TEST_F(LocomotionCommandTest, TheStatusTopicCarriesTheSourceThePlanAndTheCommand) {
  BringUpSpinning();
  ReachStand();
  const auto twist = Twist(0.3);
  SpinFor(1200ms, &twist);  // past one 1 Hz read of the MPC counters
  ASSERT_TRUE(PublishStatus());
  EXPECT_EQ(last_status_.level, DiagnosticStatus::OK);
  EXPECT_EQ(last_status_.message, "stand");
  EXPECT_EQ(last_status_.name, "locomotion_controller");
  EXPECT_EQ(last_status_.hardware_id, cfg_["model"]["body_group"].as<std::string>());
  ASSERT_EQ(last_status_.values.size(), integrated_bringup::kLocomotionStatusKeyCount);
  for (std::size_t i = 0; i < last_status_.values.size(); ++i) {
    EXPECT_EQ(last_status_.values[i].key,
              integrated_bringup::LocomotionStatusKeyName(static_cast<LocomotionStatusKey>(i)));
  }
  EXPECT_EQ(Value(LocomotionStatusKey::kSource), "stand");
  EXPECT_EQ(Value(LocomotionStatusKey::kMode), "nominal");
  EXPECT_EQ(Value(LocomotionStatusKey::kFaultCause), "none");
  EXPECT_EQ(Value(LocomotionStatusKey::kPlanId), std::to_string(ctrl_->LastDiag().plan_id));
  EXPECT_EQ(Value(LocomotionStatusKey::kContactMask), "3");
  EXPECT_EQ(Value(LocomotionStatusKey::kConverged), "1");
  EXPECT_GT(std::stoull(Value(LocomotionStatusKey::kSolves)), 0U) << "the 1 Hz counter read";
  EXPECT_EQ(Value(LocomotionStatusKey::kFailedSolves), "0");
  EXPECT_NEAR(std::stod(Value(LocomotionStatusKey::kCmdVx)), 0.3, 0.02);
  EXPECT_EQ(Value(LocomotionStatusKey::kCmdValid), "1");
  EXPECT_EQ(Value(LocomotionStatusKey::kCmdFresh), "1");
  char height[integrated_bringup::kLocomotionStatusValueChars];
  std::snprintf(height, sizeof(height), "%.6g", StandHeight());
  EXPECT_EQ(Value(LocomotionStatusKey::kCmdHeight), height);
  EXPECT_EQ(Value(LocomotionStatusKey::kCmdNonFiniteTicks), "0");
  EXPECT_NE(Value(LocomotionStatusKey::kPolicyAgeMs), "-1");
  // The fault: the plan lost off a still double support.
  state_.devices[kBody].base_twist[0] = 0.3;
  stub_->fail = true;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->HasLatchedFault(); }));
  ASSERT_TRUE(PublishStatus());
  EXPECT_EQ(last_status_.level, DiagnosticStatus::ERROR);
  EXPECT_EQ(last_status_.message, "fault");
  EXPECT_EQ(Value(LocomotionStatusKey::kFaultCause), "plan_loss");
  EXPECT_EQ(Value(LocomotionStatusKey::kMode), "fault");
}

TEST_F(LocomotionCommandTest, TheStatusPublisherIsSilentBeforeActivation) {
  Configure(MpcConfig(), rclcpp::NodeOptions{}, "test_locomotion_mpc");
  SpinController();
  SpinFor(200ms);  // the helper's subscription matches
  EXPECT_FALSE(PublishStatus()) << "a lifecycle publisher publishes only while active";
}

TEST(LocomotionStatus, TheLevelFollowsTheModeAndTheSource) {
  using integrated_bringup::LocomotionStatusLevel;
  LocomotionDiagLogPod s{};
  s.mode = static_cast<std::uint8_t>(Mode::kNominal);
  s.ref_source = static_cast<std::uint8_t>(Source::kIdle);
  EXPECT_EQ(LocomotionStatusLevel(s, false), DiagnosticStatus::OK)
      << "no MPC: the latched pose is the plan";
  EXPECT_EQ(LocomotionStatusLevel(s, true), DiagnosticStatus::WARN)
      << "an MPC without a usable plan";
  s.ref_source = static_cast<std::uint8_t>(Source::kStand);
  EXPECT_EQ(LocomotionStatusLevel(s, true), DiagnosticStatus::OK);
  s.ref_source = static_cast<std::uint8_t>(Source::kWalk);
  EXPECT_EQ(LocomotionStatusLevel(s, true), DiagnosticStatus::OK);
  s.mode = static_cast<std::uint8_t>(Mode::kDegraded);
  EXPECT_EQ(LocomotionStatusLevel(s, true), DiagnosticStatus::WARN);
  s.mode = static_cast<std::uint8_t>(Mode::kWaiting);
  EXPECT_EQ(LocomotionStatusLevel(s, true), DiagnosticStatus::WARN);
  s.mode = static_cast<std::uint8_t>(Mode::kNominal);
  s.estop = true;
  EXPECT_EQ(LocomotionStatusLevel(s, true), DiagnosticStatus::ERROR);
  s.estop = false;
  s.fault_latched = true;
  s.ref_source = static_cast<std::uint8_t>(Source::kFault);
  EXPECT_EQ(LocomotionStatusLevel(s, true), DiagnosticStatus::ERROR);
}

TEST(LocomotionStatus, EveryKeyHasADistinctNameAndEveryValueFormats) {
  std::set<std::string> names;
  LocomotionDiagLogPod s{};
  s.policy_age_us = 12345.0;
  rtc::mpc::FilteredCommand cmd{};
  cmd.v_x = 0.25;
  cmd.nonfinite_count = 3;
  cmd.ignored_axis_count = 4;
  integrated_bringup::LocomotionMpcCounters c{};
  c.solves = 7;
  for (std::size_t i = 0; i < integrated_bringup::kLocomotionStatusKeyCount; ++i) {
    const auto key = static_cast<LocomotionStatusKey>(i);
    const std::string name = integrated_bringup::LocomotionStatusKeyName(key);
    EXPECT_FALSE(name.empty());
    EXPECT_NE(name, "unknown");
    EXPECT_TRUE(names.insert(name).second) << name;
    char buf[integrated_bringup::kLocomotionStatusValueChars];
    integrated_bringup::FormatLocomotionStatusValue(key, s, cmd, c, buf, sizeof(buf));
    EXPECT_GT(std::strlen(buf), 0U) << name;
  }
  const auto value = [&](LocomotionStatusKey key) {
    char buf[integrated_bringup::kLocomotionStatusValueChars];
    integrated_bringup::FormatLocomotionStatusValue(key, s, cmd, c, buf, sizeof(buf));
    return std::string(buf);
  };
  EXPECT_EQ(value(LocomotionStatusKey::kPolicyAgeMs), "-1") << "no plan (plan_id 0)";
  s.plan_id = 9;
  EXPECT_EQ(value(LocomotionStatusKey::kPolicyAgeMs), "12.345");
  EXPECT_EQ(value(LocomotionStatusKey::kCmdVx), "0.25");
  EXPECT_EQ(value(LocomotionStatusKey::kSolves), "7");
  EXPECT_EQ(value(LocomotionStatusKey::kCmdNonFiniteTicks), "3");
  EXPECT_EQ(value(LocomotionStatusKey::kCmdIgnoredAxisTicks), "4");
}

TEST(LocomotionCommandConfig, TheTopicKeyIsOptionalAndChecked) {
  const auto refused = [](const std::function<void(YAML::Node&)>& spoil, const char* what) {
    ExpectRefused(MpcConfig(), spoil, what);
  };
  refused([](YAML::Node& c) { c["velocity_command"]["topic"] = ""; }, "empty topic");
  refused([](YAML::Node& c) { c["velocity_command"].remove("topic"); }, "no topic");
  refused([](YAML::Node& c) { c["velocity_command"]["qos"] = 1; }, "unknown key");
  refused([](YAML::Node& c) { c["velocity_command"] = "g1/cmd_vel"; }, "not a map");
  const auto parsed = ParseLocomotionControllerConfig(MpcConfig());
  EXPECT_EQ(parsed.velocity_command_topic, "g1/cmd_vel");
  YAML::Node without = MpcConfig();
  without.remove("velocity_command");
  EXPECT_TRUE(ParseLocomotionControllerConfig(without).velocity_command_topic.empty());
}

TEST(LocomotionDiagLog, TheHeaderAndARowHaveTheSameColumns) {
  const std::vector<std::string> contacts{"a", "b"};
  const std::vector<std::string> joints{"j0", "j1", "j2"};
  const auto cols = integrated_bringup::LocomotionDiagLogColumnsFor(contacts, joints);
  std::ostringstream header;
  integrated_bringup::WriteLocomotionDiagLogHeader(header, contacts, joints, cols);
  std::ostringstream row;
  integrated_bringup::WriteLocomotionDiagLogRow(row, LocomotionDiagLogPod{}, cols);
  const auto count = [](const std::string& s) { return std::count(s.begin(), s.end(), ','); };
  EXPECT_EQ(count(header.str()), count(row.str()));
  EXPECT_NE(header.str().find(",ref_source,plan_id,policy_age_us,"), std::string::npos);
  EXPECT_NE(header.str().find(",cmd_vx,cmd_vy,cmd_wz,cmd_h,cmd_valid,gait_latest,ref_base_vx,"
                              "ref_base_vy,ref_base_vz,meas_contact_valid,plan_holds,"),
            std::string::npos);
  // The lane's force is simulator truth: it is in the truth log, not here.
  EXPECT_EQ(header.str().find("meas_fz"), std::string::npos);
  // The replay inputs, then the MPC lock-step, then the support-height pin,
  // appended last; an empty pod reads "no sample, never received, free-run,
  // nothing waited, no pin".
  const std::string tail =
      ",cmd_input,cmd_deadman,est_imu_valid,est_imu_age_us,mpc_sync_lockstep,mpc_trigger_tick,"
      "mpc_wait_us,mpc_wait_count,mpc_pending_superseded,est_ground_a,est_ground_b,"
      "est_ground_nu_a,est_ground_nu_b,est_ground_nis_a,est_ground_nis_b,est_gated_pins";
  ASSERT_GE(header.str().size(), tail.size());
  EXPECT_EQ(header.str().substr(header.str().size() - tail.size()), tail);
  const std::string row_tail = ",0,-1,0,0,0,0,0,0,0,0,0,0,0,0";
  EXPECT_EQ(row.str().substr(row.str().size() - row_tail.size()), row_tail);
}

// The measured contact force: group c of the body's inference lane, fz at
// value 3, logged only while that group is fresh and the lane has the model's
// contact count.
class LocomotionContactForceLogTest : public LocomotionControllerTest {
 protected:
  void BringUpWithLane(int values_per_group) {
    YAML::Node cfg = ShippedConfig();
    auto devices = ShippedDevices(cfg);
    rtc::DeviceSensorLayout layout;
    layout.inference_values_per_group = values_per_group;
    devices.at("g1").sensor_layout = layout;
    ConfigureWithDevices(cfg, devices);
    Activate();
    MakeStanding();
  }

  void SetLane(int groups, int stride) {
    auto& body = state_.devices[kBody];
    body.num_inference_groups = groups;
    for (int g = 0; g < groups; ++g) {
      const auto at = static_cast<std::size_t>(g * stride);
      body.inference_data[at + 3] = -150.0F - static_cast<float>(g);
      body.inference_enable[static_cast<std::size_t>(g)] = true;
    }
  }
};

TEST_F(LocomotionContactForceLogTest, AFreshGroupSetsItsBitAndAStaleOneClearsIt) {
  BringUpWithLane(7);
  SetLane(2, 7);
  TickFor(1);
  const auto& d = ctrl_->LastDiag();
  EXPECT_EQ(d.meas_contact_valid, 0b11U);
  state_.devices[kBody].inference_enable[1] = false;  // the value is held, the flag drops
  TickFor(1);
  EXPECT_EQ(ctrl_->LastDiag().meas_contact_valid, 0b01U);
}

TEST_F(LocomotionContactForceLogTest, ALaneThatDoesNotFitTheContactsHasNoBit) {
  BringUpWithLane(7);
  SetLane(3, 7);  // a group count other than the model's two contacts
  TickFor(1);
  EXPECT_EQ(ctrl_->LastDiag().meas_contact_valid, 0U);
}

TEST_F(LocomotionContactForceLogTest, ALaneTooNarrowForAForceHasNoBit) {
  BringUpWithLane(3);
  SetLane(2, 3);
  TickFor(1);
  EXPECT_EQ(ctrl_->LastDiag().meas_contact_valid, 0U);
}

TEST_F(LocomotionContactForceLogTest, NoSensorLayoutIsNoLane) {
  BringUp();  // the rig's device configs carry no sensor_layout
  SetLane(2, 7);
  TickFor(1);
  EXPECT_EQ(ctrl_->LastDiag().meas_contact_valid, 0U);
}

// ── R-tier columns (locomotion-controller.md "Log tiers") ──────────────────

// wrench_des is the λ target the QP was handed on that tick, per contact at
// its own offset; NaN on a tick the QP did not run.
void ExpectWrenchDesIsTheQpTarget(const LocomotionController& ctrl, const YAML::Node& cfg) {
  const auto& d = ctrl.LastDiag();
  const Eigen::VectorXd& lambda = ctrl.Reference().lambda_des;
  Eigen::Index offset = 0;
  const YAML::Node contacts = cfg["model"]["contacts"];
  for (std::size_t c = 0; c < contacts.size(); ++c) {
    const int dim = contacts[c]["dim"].as<int>();
    for (int k = 0; k < 6; ++k) {
      const double got = d.wrench_des[c * 6 + static_cast<std::size_t>(k)];
      if (k < dim) {
        EXPECT_TRUE(rtc::testing::BitsEqual(got, lambda(offset + k))) << c << ' ' << k;
      } else {
        EXPECT_TRUE(std::isnan(got)) << "a point contact has no moment: " << c << ' ' << k;
      }
    }
    offset += dim;
  }
}

void ExpectWrenchDesNaN(const LocomotionController& ctrl) {
  for (const double x : ctrl.LastDiag().wrench_des) {
    EXPECT_TRUE(std::isnan(x));
  }
}

TEST_F(LocomotionMpcTest, WrenchDesIsTheStandPlansTargetTheQpWasHanded) {
  BringUpWithStub();
  ReachStand();
  for (int k = 0; k < 20; ++k) {
    last_out_ = Tick();
    ASSERT_EQ(ctrl_->Source(), Source::kStand);
    ASSERT_EQ(LastMode(), Mode::kNominal);
    ExpectWrenchDesIsTheQpTarget(*ctrl_, cfg_);
  }
  // The MPC's own record: its last attempt, published, after the plan the
  // WBC follows at the latest; seq never goes back.
  const auto& m = ctrl_->LastDiag().mpc_last;
  EXPECT_GT(m.seq, 0U);
  EXPECT_EQ(m.outcome, rtc::mpc::SolveOutcome::kPublished);
  EXPECT_GE(m.plan_id, ctrl_->LastDiag().plan_id);
  std::uint64_t seq = m.seq;
  for (int k = 0; k < 20; ++k) {
    last_out_ = Tick();
    EXPECT_GE(ctrl_->LastDiag().mpc_last.seq, seq);
    seq = ctrl_->LastDiag().mpc_last.seq;
  }
}

// Idle (no MPC): the lever-rule static share, with one point contact (dim 3)
// beside a 6-dim one so a wrong offset shows.
TEST_F(LocomotionMpcTest, WrenchDesUnderIdleWithAPointContact) {
  YAML::Node cfg = YAML::Clone(ShippedConfig());
  cfg["model"]["contacts"][0]["dim"] = 3;
  Configure(cfg, rclcpp::NodeOptions{}, "test_locomotion_wrench_des");
  Activate();
  MakeStanding();
  for (int k = 0; k < 5; ++k) {
    last_out_ = Tick();
  }
  ASSERT_EQ(LastMode(), Mode::kNominal);
  ASSERT_EQ(ctrl_->Source(), Source::kIdle);
  ExpectWrenchDesIsTheQpTarget(*ctrl_, cfg);
  EXPECT_EQ(ctrl_->LastDiag().mpc_last.seq, 0U) << "no MPC, no record";
  EXPECT_EQ(ctrl_->LastDiag().mpc_last.outcome, rtc::mpc::SolveOutcome::kNone);
}

TEST_F(LocomotionMpcTest, WrenchDesIsNaNWhenTheQpDidNotRun) {
  Configure(ShippedConfig(), rclcpp::NodeOptions{}, "test_locomotion_wrench_nan");
  Activate();
  last_out_ = ctrl_->Compute(state_);  // no lane yet: waiting
  ASSERT_EQ(LastMode(), Mode::kWaiting);
  ExpectWrenchDesNaN(*ctrl_);
  MakeStanding();
  last_out_ = Tick();
  ASSERT_EQ(LastMode(), Mode::kNominal);
  last_out_ = StaleTick();  // a short source drop: the hold
  ASSERT_EQ(LastMode(), Mode::kDegraded);
  ExpectWrenchDesNaN(*ctrl_);
  for (int k = 0; k < 8; ++k) {
    last_out_ = StaleTick();
  }
  ASSERT_EQ(LastMode(), Mode::kFault);
  last_out_ = Tick();  // latched: damping, no QP
  ASSERT_EQ(LastMode(), Mode::kFault);
  ExpectWrenchDesNaN(*ctrl_);
}

// servo_kp / servo_kd are kp (q_cmd − q) and −kd q̇ from the tick's own inputs,
// each one; their sum is tau_servo to rounding; under damping the kp term is 0.
TEST_F(LocomotionMpcTest, TheServoSplitIsItsTwoTerms) {
  Configure(ShippedConfig(), rclcpp::NodeOptions{}, "test_locomotion_servo_split");
  Activate();
  MakeStanding();
  const auto kp = cfg_["pd_feedforward"]["kp"].as<std::vector<double>>();
  const auto kd = cfg_["pd_feedforward"]["kd"].as<std::vector<double>>();
  auto& body = state_.devices[kBody];
  for (int c = 0; c < body.num_channels; ++c) {
    body.velocities[static_cast<std::size_t>(c)] = 0.01 * (c + 1) * (c % 2 == 0 ? 1.0 : -1.0);
  }
  const auto check = [&](bool damping) {
    const auto& d = ctrl_->LastDiag();
    const auto& out = last_out_.devices[kBody];
    for (std::size_t i = 0; i < kp.size(); ++i) {
      const double q = body.positions[i];
      const double v = body.velocities[i];
      EXPECT_TRUE(rtc::testing::BitsEqual(d.servo_kp[i], kp[i] * (out.commands[i] - q))) << i;
      EXPECT_TRUE(rtc::testing::BitsEqual(d.servo_kd[i], -kd[i] * v)) << i;
      const double sum = d.servo_kp[i] + d.servo_kd[i];
      EXPECT_LE(std::abs(d.tau_servo[i] - sum),
                1e-12 * std::max(1.0, std::abs(d.servo_kp[i]) + std::abs(d.servo_kd[i])))
          << i;
      if (damping) {
        EXPECT_EQ(d.servo_kp[i], 0.0) << "damping commands the measured q: " << i;
      }
    }
  };
  for (int k = 0; k < 3; ++k) {
    last_out_ = Tick();
  }
  ASSERT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(ctrl_->LastDiag().state_source, 0U) << "truth";
  check(false);
  for (int k = 0; k < 9; ++k) {
    last_out_ = StaleTick();
  }
  last_out_ = Tick();
  ASSERT_EQ(LastMode(), Mode::kFault);
  check(true);
}

TEST_F(LocomotionMpcTest, TheStateSourceColumnIsOneUnderTheEstimator) {
  YAML::Node cfg = YAML::Clone(ShippedConfig());
  cfg["estimator"]["source"] = "inekf";
  Configure(cfg, rclcpp::NodeOptions{}, "test_locomotion_state_source");
  Activate();
  MakeStanding();
  last_out_ = Tick();
  EXPECT_EQ(ctrl_->LastDiag().state_source, 1U);
}

// ── velocity_command.type joy (and the Twist path's sample, unchanged) ──────

// The Twist callback's whole sample, bit for bit: the six axes as sent (also
// NaN, ±inf, −0.0, a denormal and 1e300 — the filter, not the callback, judges
// them), sequence + 1, the receive time between the steady clock before and
// after the call, and the source fields at their Twist values.
TEST_F(LocomotionCommandTest, TheTwistCallbackStoresTheWholeSampleUnchanged) {
  BringUpSpinning();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const std::array<std::array<double, 6>, 8> fixtures{{
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
      {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0},
      {nan, 0.5, -0.25, nan, 0.0, nan},
      {inf, -inf, 0.125, inf, -inf, 2.0},
      {-0.0, -0.0, -0.0, -0.0, -0.0, -0.0},
      {4.9e-324, -4.9e-324, 2.2e-308, 0.0, 0.0, 1e-310},
      {1e300, -1e300, 1e300, -1e300, 1e300, -1e300},
  }};
  for (std::size_t i = 0; i < fixtures.size(); ++i) {
    SCOPED_TRACE(i);
    const auto& f = fixtures[i];
    geometry_msgs::msg::Twist t;
    t.linear.x = f[0];
    t.linear.y = f[1];
    t.linear.z = f[2];
    t.angular.x = f[3];
    t.angular.y = f[4];
    t.angular.z = f[5];
    const auto before = LocomotionControllerTestAccess::RawCommand(*ctrl_);
    const std::int64_t t0 = rtc::SteadyNowNs();
    LocomotionControllerTestAccess::OnTwist(*ctrl_, t);
    const std::int64_t t1 = rtc::SteadyNowNs();
    const auto s = LocomotionControllerTestAccess::RawCommand(*ctrl_);
    const std::array<double, 6> got{s.twist.linear_x,  s.twist.linear_y,  s.twist.linear_z,
                                    s.twist.angular_x, s.twist.angular_y, s.twist.angular_z};
    EXPECT_EQ(std::memcmp(got.data(), f.data(), sizeof(got)), 0) << "the axes bit for bit";
    EXPECT_EQ(s.sequence, before.sequence + 1);
    EXPECT_GE(s.recv_steady_ns, t0);
    EXPECT_LE(s.recv_steady_ns, t1);
    EXPECT_EQ(s.source.input, 0U);
    EXPECT_EQ(s.source.deadman, -1);
    EXPECT_EQ(s.source.joy_rejects, 0U);
  }
}

class LocomotionJoyCommandTest : public LocomotionCommandTest {
 protected:
  static constexpr const char* kJoyTopic = "/g1/joy";
  static constexpr std::size_t kLefty = 1;
  static constexpr std::size_t kDeadman = 9;  // leftshoulder

  void SetUp() override {
    LocomotionCommandTest::SetUp();
    joy_pub_ = helper_->create_publisher<sensor_msgs::msg::Joy>(kJoyTopic, rclcpp::QoS(1));
  }

  static YAML::Node JoyMpcConfig() {
    YAML::Node cfg = MpcConfig();
    SetJoyCommand(cfg);
    return cfg;
  }

  static sensor_msgs::msg::Joy Joy(float lefty, int deadman) {
    sensor_msgs::msg::Joy j;
    j.axes.assign(6, 0.0F);
    j.buttons.assign(21, 0);
    j.axes[kLefty] = lefty;
    j.buttons[kDeadman] = deadman;
    return j;
  }

  // Spin, tick and publish `joy` at 50 Hz until `done` holds or `deadline`
  // passes; whether it held.
  bool SpinUntil(const std::function<bool()>& done, std::chrono::milliseconds deadline,
                 const sensor_msgs::msg::Joy* joy) {
    const auto end = std::chrono::steady_clock::now() + deadline;
    auto next_pub = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < end) {
      if (joy != nullptr && std::chrono::steady_clock::now() >= next_pub) {
        joy_pub_->publish(*joy);
        next_pub += 20ms;
      }
      executor_.spin_some(1ms);
      last_out_ = Tick();
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return done();
  }

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
};

// The Joy callback's sample: every message — a zero one too (released,
// too short, a NaN axis) — refreshes the receive time on the steady clock and
// the sequence; header.stamp is not read (an hour off either way changes
// nothing); deadman and rejects ride in the sample.
TEST_F(LocomotionJoyCommandTest, TheJoyCallbackStoresEveryMessageWithItsReceiveTime) {
  BringUpSpinning(JoyMpcConfig());
  auto nan_axis = Joy(std::numeric_limits<float>::quiet_NaN(), 1);
  auto too_short = Joy(0.25F, 1);
  too_short.buttons.resize(5);

  struct Case {
    sensor_msgs::msg::Joy msg;
    double vx;
    int deadman;
    std::uint64_t reject;
  };

  const std::vector<Case> cases{
      {Joy(0.25F, 1), 0.5, 1, 0}, {Joy(0.25F, 0), 0.0, 0, 0}, {Joy(0.25F, 2), 0.0, 0, 0},
      {nan_axis, 0.0, 1, 1},      {too_short, 0.0, -1, 1},
  };
  for (const std::int32_t stamp_sec : {0, 3600, -3600}) {
    for (std::size_t i = 0; i < cases.size(); ++i) {
      SCOPED_TRACE(std::to_string(stamp_sec) + " / " + std::to_string(i));
      sensor_msgs::msg::Joy msg = cases[i].msg;
      msg.header.stamp.sec = stamp_sec;
      const auto before = LocomotionControllerTestAccess::RawCommand(*ctrl_);
      const std::int64_t t0 = rtc::SteadyNowNs();
      LocomotionControllerTestAccess::OnJoy(*ctrl_, msg);
      const std::int64_t t1 = rtc::SteadyNowNs();
      const auto s = LocomotionControllerTestAccess::RawCommand(*ctrl_);
      EXPECT_EQ(s.twist.linear_x, cases[i].vx);
      EXPECT_EQ(s.twist.linear_y, 0.0);
      EXPECT_EQ(s.twist.angular_z, 0.0);
      EXPECT_EQ(s.sequence, before.sequence + 1);
      EXPECT_GE(s.recv_steady_ns, t0);
      EXPECT_LE(s.recv_steady_ns, t1);
      EXPECT_EQ(s.source.input, 1U);
      EXPECT_EQ(s.source.deadman, cases[i].deadman);
      EXPECT_EQ(s.source.joy_rejects, before.source.joy_rejects + cases[i].reject);
    }
  }
}

TEST_F(LocomotionJoyCommandTest, OnlyTheConfiguredSubscriptionExists) {
  BringUpSpinning(JoyMpcConfig());
  ASSERT_TRUE(
      SpinUntil([this] { return helper_->count_subscribers(kJoyTopic) == 1U; }, 2000ms, nullptr));
  EXPECT_EQ(helper_->count_subscribers(kTwistTopic), 0U);
  EXPECT_TRUE(ctrl_->VelocityCommandSubscribed());
  EXPECT_EQ(ctrl_->LastCommandSource().input, 1U) << "tagged joy before the first message";
  EXPECT_EQ(ctrl_->LastCommandSource().deadman, 0);
}

TEST_F(LocomotionJoyCommandTest, ACleanupReleasesTheJoySubscription) {
  BringUpSpinning(JoyMpcConfig());
  ASSERT_TRUE(
      SpinUntil([this] { return helper_->count_subscribers(kJoyTopic) == 1U; }, 2000ms, nullptr));
  Deactivate();
  ASSERT_EQ(ctrl_->on_cleanup(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_FALSE(ctrl_->VelocityCommandSubscribed());
  EXPECT_TRUE(
      SpinUntil([this] { return helper_->count_subscribers(kJoyTopic) == 0U; }, 2000ms, nullptr));
}

TEST_F(LocomotionJoyCommandTest, TheTwistModeHasNoJoySubscription) {
  BringUpSpinning();
  SpinFor(200ms);
  EXPECT_EQ(helper_->count_subscribers(kTwistTopic), 1U);
  EXPECT_EQ(helper_->count_subscribers(kJoyTopic), 0U);
  EXPECT_EQ(ctrl_->LastCommandSource().input, 0U);
  EXPECT_EQ(ctrl_->LastCommandSource().deadman, -1);
}

TEST_F(LocomotionJoyCommandTest, WithoutAnMpcNothingIsSubscribed) {
  YAML::Node cfg = JoyMpcConfig();
  cfg["mpc"]["enabled"] = false;
  Configure(cfg, rclcpp::NodeOptions{}, "test_locomotion_mpc");
  SpinController();
  SpinFor(200ms);
  EXPECT_FALSE(ctrl_->VelocityCommandSubscribed());
  EXPECT_EQ(helper_->count_subscribers(kJoyTopic), 0U);
  EXPECT_EQ(helper_->count_subscribers(kTwistTopic), 0U);
  EXPECT_EQ(ctrl_->LastCommandSource().input, 0U) << "no Joy subscription: not tagged joy";
  EXPECT_EQ(ctrl_->LastCommandSource().deadman, -1);
}

TEST_F(LocomotionJoyCommandTest, AJoyReachesTheFilterAndThePlanner) {
  BringUpSpinning(JoyMpcConfig());
  ReachStand();
  const auto seq0 = stub_->horizon_command_sequence.load();
  const auto joy = Joy(0.15F, 1);  // 0.15 × v_max 2.0 = 0.3 m/s
  ASSERT_TRUE(
      SpinUntil([this] { return std::abs(ctrl_->LastCommand().v_x - 0.3) <= 0.02; }, 2000ms, &joy))
      << ctrl_->LastCommand().v_x;
  const auto cmd = ctrl_->LastCommand();
  EXPECT_TRUE(cmd.fresh);
  EXPECT_EQ(cmd.v_y, 0.0);
  const auto source = ctrl_->LastCommandSource();
  EXPECT_EQ(source.input, 1U);
  EXPECT_EQ(source.deadman, 1);
  EXPECT_EQ(source.joy_rejects, 0U);
  ASSERT_TRUE(SpinUntil(
      [&] {
        return stub_->horizon_command_sequence.load() > seq0 &&
               std::abs(stub_->horizon_base_vx.load() - ctrl_->LastCommand().v_x) <= 0.02;
      },
      2000ms, &joy))
      << "the planner's node 0 base velocity follows the command";
  last_out_ = Tick();
  EXPECT_EQ(ctrl_->LastDiag().cmd_input, 1U);
  EXPECT_EQ(ctrl_->LastDiag().cmd_deadman, 1);
}

TEST_F(LocomotionJoyCommandTest, ReleasingTheDeadmanStopsAsAFreshCommand) {
  BringUpSpinning(JoyMpcConfig());
  const auto go = Joy(0.15F, 1);
  ASSERT_TRUE(
      SpinUntil([this] { return std::abs(ctrl_->LastCommand().v_x - 0.3) <= 0.02; }, 2000ms, &go));
  const auto stale0 = ctrl_->LastCommand().stale_count;
  const auto released = Joy(0.15F, 0);  // the stick still pushed, the deadman let go
  ASSERT_TRUE(
      SpinUntil([this] { return std::abs(ctrl_->LastCommand().v_x) < 1e-3; }, 2000ms, &released));
  const auto cmd = ctrl_->LastCommand();
  EXPECT_TRUE(cmd.fresh) << "a released deadman is a command, not silence";
  EXPECT_EQ(cmd.stale_count - stale0, 0U);
  EXPECT_EQ(ctrl_->LastCommandSource().deadman, 0);
  ASSERT_TRUE(PublishStatus());
  EXPECT_EQ(Value(LocomotionStatusKey::kCmdInput), "1");
  EXPECT_EQ(Value(LocomotionStatusKey::kCmdDeadman), "0");
}

TEST_F(LocomotionJoyCommandTest, ASilentPadStopsThroughStale) {
  BringUpSpinning(JoyMpcConfig());
  const auto go = Joy(0.15F, 1);
  ASSERT_TRUE(
      SpinUntil([this] { return std::abs(ctrl_->LastCommand().v_x - 0.3) <= 0.02; }, 2000ms, &go));
  const auto stale0 = ctrl_->LastCommand().stale_count;
  // nothing published with the deadman still held: stale after command.t_stale
  ASSERT_TRUE(SpinUntil(
      [this] {
        const auto c = ctrl_->LastCommand();
        return std::abs(c.v_x) < 1e-3 && !c.fresh;
      },
      2000ms, nullptr));
  EXPECT_GT(ctrl_->LastCommand().stale_count - stale0, 0U);
}

TEST_F(LocomotionJoyCommandTest, AnUnreadableJoyIsZeroAndCounted) {
  BringUpSpinning(JoyMpcConfig());
  const auto go = Joy(0.15F, 1);
  ASSERT_TRUE(
      SpinUntil([this] { return std::abs(ctrl_->LastCommand().v_x - 0.3) <= 0.02; }, 2000ms, &go));
  auto bad = Joy(std::numeric_limits<float>::quiet_NaN(), 1);
  ASSERT_TRUE(SpinUntil(
      [this] {
        return std::abs(ctrl_->LastCommand().v_x) < 1e-3 &&
               ctrl_->LastCommandSource().joy_rejects > 0U;
      },
      2000ms, &bad))
      << "unlike a Twist, a NaN axis does not hold the last value";
  EXPECT_TRUE(ctrl_->LastCommand().fresh);
  const auto rejects = ctrl_->LastCommandSource().joy_rejects;
  ASSERT_TRUE(PublishStatus());
  EXPECT_EQ(Value(LocomotionStatusKey::kCmdJoyRejects), std::to_string(rejects));
}

TEST_F(LocomotionJoyCommandTest, AReactivationStartsFromRestInJoyMode) {
  BringUpSpinning(JoyMpcConfig());
  const auto go = Joy(0.15F, 1);
  ASSERT_TRUE(
      SpinUntil([this] { return std::abs(ctrl_->LastCommand().v_x - 0.3) <= 0.02; }, 2000ms, &go));
  Deactivate();
  SpinFor(100ms);
  Activate();
  const auto cmd = ctrl_->LastCommand();
  EXPECT_EQ(cmd.sequence, 0U);
  EXPECT_FALSE(cmd.valid);
  EXPECT_EQ(cmd.v_x, 0.0) << "nothing from the last session";
  EXPECT_EQ(ctrl_->LastCommandSource().input, 1U);
  EXPECT_EQ(ctrl_->LastCommandSource().deadman, 0);
  EXPECT_EQ(ctrl_->LastCommandSource().joy_rejects, 0U);
  EXPECT_EQ(helper_->count_subscribers(kJoyTopic), 1U);
  ASSERT_TRUE(
      SpinUntil([this] { return std::abs(ctrl_->LastCommand().v_x - 0.3) <= 0.02; }, 2000ms, &go));
}

TEST_F(LocomotionCommandTest, TheStatusTopicCarriesTheTwistSourceKeys) {
  BringUpSpinning();
  const auto twist = Twist(0.3);
  SpinFor(300ms, &twist);
  ASSERT_TRUE(PublishStatus());
  ASSERT_EQ(last_status_.values.size(), integrated_bringup::kLocomotionStatusKeyCount);
  EXPECT_EQ(last_status_.values[static_cast<std::size_t>(LocomotionStatusKey::kCmdInput)].key,
            "cmd_input");
  EXPECT_EQ(last_status_.values[static_cast<std::size_t>(LocomotionStatusKey::kCmdDeadman)].key,
            "cmd_deadman");
  EXPECT_EQ(last_status_.values[static_cast<std::size_t>(LocomotionStatusKey::kCmdJoyRejects)].key,
            "cmd_joy_rejects");
  EXPECT_EQ(Value(LocomotionStatusKey::kCmdInput), "0");
  EXPECT_EQ(Value(LocomotionStatusKey::kCmdDeadman), "-1");
  EXPECT_EQ(Value(LocomotionStatusKey::kCmdJoyRejects), "0");
  last_out_ = Tick();
  EXPECT_EQ(ctrl_->LastDiag().cmd_input, 0U);
  EXPECT_EQ(ctrl_->LastDiag().cmd_deadman, -1);
}

}  // namespace
