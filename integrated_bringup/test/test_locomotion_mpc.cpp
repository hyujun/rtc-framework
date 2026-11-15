// locomotion_controller with its MPC (mpc.enabled): the reference sources
// (idle → stand on a usable plan, back to idle or the plan-loss fault when the
// plan is lost), the plausibility check, the contact mask, the state stream's
// restarts (reset, E-STOP, activation), the thread's lifetime, the hand hold,
// the layout-profile refusal, and the `mpc:` schema. The plans come from a stub
// handler on the real MPC thread (one producer, as in the run); one smoke test
// runs the installed MPC itself.
#include "locomotion_mpc_stub.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

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
           ") initial ticks " + std::to_string(t->InitialSolveTicks()) + " withheld " +
           std::to_string(t->WithheldCount()) + " cold withheld " +
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
    EXPECT_EQ(ref.a_des(i), plan.a_ref[static_cast<std::size_t>(i)]) << i;  // riccati off
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
  // The body's position target is the plan's.
  const auto& body = last_out_.devices[kBody];
  for (int j = 0; j < body.num_channels; ++j) {
    EXPECT_EQ(body.commands[static_cast<std::size_t>(j)],
              plan.q_ref[static_cast<std::size_t>(7 + j)])
        << j;
  }
}

TEST_F(LocomotionMpcTest, AnUnconvergedStandingPlanIsNotStoodOn) {
  BringUpWithStub();
  stub_->converged = false;
  stub_->initial = true;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->MpcManager().HasEverReceivedSolution(); }));
  TickFor(100);
  EXPECT_EQ(ctrl_->Source(), Source::kIdle);
  EXPECT_EQ(LastMode(), Mode::kNominal);
  stub_->converged = true;  // positive control
  ReachStand();
}

TEST_F(LocomotionMpcTest, AnUnconvergedOnlinePlanKeepsTheStand) {
  BringUpWithStub();
  ReachStand();
  stub_->initial = false;
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
      {"a joint acceleration out of bounds", [](StandingStub& s) { s.joint_rate = 30.0; }},
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
    const auto& body = last_out_.devices[kBody];
    const auto pose = StandPose();
    for (std::size_t j = 0; j < pose.size(); ++j) {
      EXPECT_NEAR(body.commands[j], pose[j], 1e-12) << j;
    }
    ctrl_.reset();
  }
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
    YAML::Node cfg = MpcConfig();
    spoil(cfg);
    EXPECT_THROW(static_cast<void>(ParseLocomotionControllerConfig(cfg)), std::invalid_argument)
        << what;
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
  const auto parsed = ParseLocomotionControllerConfig(MpcConfig());
  EXPECT_TRUE(parsed.mpc.present);
  EXPECT_TRUE(parsed.mpc.enabled);
  EXPECT_FALSE(parsed.mpc.riccati_enabled);
  EXPECT_EQ(parsed.mpc.period_nodes, 1);
  EXPECT_EQ(parsed.mpc.max_policy_age_s, 0.14);
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
  // Positive control: the installed sections build.
  BringUpWithStub();
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
}

}  // namespace
