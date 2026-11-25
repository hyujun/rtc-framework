// locomotion_controller on the installed g1_loco profile and the CM's own model:
// configure → activate, a standing tick the QP solves on the first try, the
// fault latch and its three exits (the reset service, activation, the cause
// staying), the E-STOP edges, and the configuration refusals. Every profile that
// ships the controller's YAML must load it (the model-free half).
#include "locomotion_controller_rig.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#pragma GCC diagnostic pop

#include <functional>
#include <stdexcept>

namespace {

using namespace loco_rig;  // NOLINT(google-build-using-namespace) — the shared rig

// ── Configuration ───────────────────────────────────────────────────────────

TEST_F(LocomotionControllerTest, ConfiguresWithTheFullAndTheReducedModel) {
  BringUp();
  // Kinematics on body + hands, dynamics on the body alone.
  EXPECT_EQ(ctrl_->FullModel().nv, 6 + 23 + 10);
  EXPECT_EQ(ctrl_->ReducedModel().nv, 6 + 23);
  const auto& idx = ctrl_->Indices();
  EXPECT_EQ(idx.num_joints, 33);
  EXPECT_EQ(idx.body.num_channels, 23);
  ASSERT_EQ(idx.num_hands, 1);
  EXPECT_EQ(idx.hands[0].num_channels, 10);
  // Every channel lands on the joint of its own name.
  const auto order = loco::JointOrder(ctrl_->FullModel());
  const auto devices = ShippedDevices(cfg_);
  for (int c = 0; c < idx.body.num_channels; ++c) {
    EXPECT_EQ(order[static_cast<std::size_t>(idx.body.joint[static_cast<std::size_t>(c)])],
              devices.at("g1").joint_state_names[static_cast<std::size_t>(c)]);
  }
  for (int c = 0; c < idx.hands[0].num_channels; ++c) {
    EXPECT_EQ(order[static_cast<std::size_t>(idx.hands[0].joint[static_cast<std::size_t>(c)])],
              devices.at("p1b").joint_state_names[static_cast<std::size_t>(c)]);
  }
}

TEST_F(LocomotionControllerTest, AHandJointMovesOnlyTheHandFramesOfTheFullModel) {
  BringUp();
  Tick();
  const auto sole0 = ctrl_->FullFramePlacement("left_sole");
  const auto torso0 = ctrl_->FullFramePlacement("torso_link");
  const auto tip0 = ctrl_->FullFramePlacement("l_index_tip_bracket");
  ASSERT_TRUE(sole0 && torso0 && tip0);
  const Eigen::VectorXd tau0 = ctrl_->LastSolve()->tau;

  state_.devices[kHand].positions[5] = -0.6;  // index_mcp_fe
  Tick();
  EXPECT_TRUE(ctrl_->FullFramePlacement("left_sole")->isApprox(*sole0, 0.0));
  EXPECT_TRUE(ctrl_->FullFramePlacement("torso_link")->isApprox(*torso0, 0.0));
  EXPECT_GT((ctrl_->FullFramePlacement("l_index_tip_bracket")->translation() - tip0->translation())
                .norm(),
            1e-3);
  // The dynamics model holds the hand at neutral: the hand pose is not an input.
  EXPECT_EQ(ctrl_->ReducedModel().nv, 29);
  static_cast<void>(tau0);
}

// ── Standing ────────────────────────────────────────────────────────────────

TEST_F(LocomotionControllerTest, TheFirstTickAfterActivationSolvesTheStance) {
  BringUp();
  const rtc::ControllerOutput out = Tick();
  ASSERT_EQ(LastMode(), Mode::kNominal);
  const rtc::tsid::CommandOutput* s = ctrl_->LastSolve();
  ASSERT_NE(s, nullptr);
  EXPECT_TRUE(s->tau_valid);
  EXPECT_EQ(s->rows_dropped, 0);
  // At rest on the reference: the QP's acceleration is solver noise.
  const double eps_abs = cfg_["whole_body_qp"]["solver"]["eps_abs"].as<double>();
  EXPECT_LE(s->a_opt.cwiseAbs().maxCoeff(), 100.0 * eps_abs);

  // Equations of motion, independently: S^T τ + Σ J_cᵀ λ = M a + h on the
  // dynamics model at the measured state.
  const pinocchio::Model& red = ctrl_->ReducedModel();
  pinocchio::Data d(red);
  const auto& m = ctrl_->LastMeasuredState();
  Eigen::VectorXd q = pinocchio::neutral(red);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(red.nv);
  for (int i = 0; i < 7; ++i) {
    q(i) = m.q[static_cast<std::size_t>(i)];
  }
  const auto& idx = ctrl_->Indices();
  for (int c = 0; c < idx.body.num_channels; ++c) {
    q(7 + c) = state_.devices[kBody].positions[static_cast<std::size_t>(c)];
  }
  Eigen::VectorXd rhs = pinocchio::rnea(red, d, q, v, s->a_opt);
  pinocchio::computeJointJacobians(red, d, q);
  pinocchio::updateFramePlacements(red, d);
  Eigen::VectorXd lhs = Eigen::VectorXd::Zero(red.nv);
  lhs.tail(red.nv - 6) = s->tau;
  Eigen::Index offset = 0;
  for (const char* sole : {"left_sole", "right_sole"}) {
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, red.nv);
    pinocchio::getFrameJacobian(red, d, red.getFrameId(sole), pinocchio::LOCAL_WORLD_ALIGNED, J);
    lhs += J.transpose() * s->lambda_opt.segment(offset, 6);
    offset += 6;
  }
  EXPECT_LE((lhs - rhs).cwiseAbs().maxCoeff(), 1e-6 * rhs.cwiseAbs().maxCoeff());

  // The wire: pd_feedforward on the body with the QP's torque, the target
  // started from the measurement at the latch (header, "The servo target"),
  // the shipped gains claimed; the hand held where it was.
  EXPECT_EQ(out.command_type, rtc::CommandType::kPdFeedforward);
  const auto& body = out.devices[kBody];
  ASSERT_EQ(body.num_channels, 23);
  for (std::size_t i = 0; i < 23; ++i) {
    EXPECT_EQ(body.feedforward[i], s->tau(static_cast<Eigen::Index>(i))) << i;
  }
  ExpectServoTargetFromMeasurement(out);
  ExpectShippedGains(out);
  const auto& hand = out.devices[kHand];
  EXPECT_EQ(hand.command_type, rtc::CommandType::kPosition);
  EXPECT_FALSE(hand.has_pd_gains);
  ASSERT_EQ(hand.num_channels, 10);
  for (std::size_t i = 0; i < 10; ++i) {
    EXPECT_EQ(hand.commands[i], state_.devices[kHand].positions[i]) << i;
  }
}

TEST_F(LocomotionControllerTest, TheReferenceStaysLatchedWhileTheRobotMoves) {
  BringUp();
  Tick();
  const double q0 = state_.devices[kBody].positions[3];
  state_.devices[kBody].positions[3] += 0.05;  // the knee sags
  Tick();
  EXPECT_EQ(ctrl_->Reference().q_des(7 + 3), q0);  // the reference does not follow
}

TEST_F(LocomotionControllerTest, TheHandsHoldThePoseTheyHadAtTheLatch) {
  BringUp();
  state_.devices[kHand].positions[2] = 0.3;
  Tick();
  state_.devices[kHand].positions[2] = 0.7;  // the hand drifts (or is pushed)
  const rtc::ControllerOutput out = Tick();
  EXPECT_EQ(out.devices[kHand].commands[2], 0.3);  // held, not following
}

TEST_F(LocomotionControllerTest, AHandNotSeenAtTheLatchIsHeldWhereItFirstReports) {
  BringUp();
  state_.devices[kHand].valid = false;  // the hand has not reported yet
  rtc::ControllerOutput out = Tick();
  ASSERT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(out.devices[kHand].num_channels, 0);  // no hold at a made-up pose
  state_.devices[kHand].valid = true;
  state_.devices[kHand].positions[4] = 0.2;
  out = Tick();
  ASSERT_EQ(out.devices[kHand].num_channels, 10);
  EXPECT_EQ(out.devices[kHand].commands[4], 0.2);
  state_.devices[kHand].positions[4] = 0.5;
  out = Tick();
  EXPECT_EQ(out.devices[kHand].commands[4], 0.2);  // and held there
}

// ── The wbc servo target (header, "The servo target") ───────────────────────

YAML::Node WbcTargetConfig(double time_constant_s = 0.0) {
  YAML::Node cfg = ShippedConfig();
  cfg["pd_feedforward"]["target"] = "wbc";
  cfg["pd_feedforward"]["target_time_constant_s"] = time_constant_s;
  return cfg;
}

TEST_F(LocomotionControllerTest, TheWbcTargetStartsAtTheLatchFromTheMeasuredPose) {
  BringUp(WbcTargetConfig());
  const rtc::ControllerOutput out = Tick();
  ASSERT_EQ(LastMode(), Mode::kNominal);
  ExpectServoTargetFromMeasurement(out);
}

// The QP's reference stays where it was latched; the wbc target is the QP's own
// joint state, so it starts from where the knee is, not from the latch.
TEST_F(LocomotionControllerTest, TheWbcTargetFollowsTheRobotWhileTheReferenceStaysLatched) {
  BringUp(WbcTargetConfig());
  Tick();
  const double q0 = state_.devices[kBody].positions[3];
  state_.devices[kBody].positions[3] += 0.05;  // the knee sags
  const auto before = SnapshotServoTarget();
  const rtc::ControllerOutput out = Tick();
  EXPECT_EQ(ctrl_->Reference().q_des(7 + 3), q0);  // the reference does not follow
  ExpectServoTargetStep(before, out);
  EXPECT_GT(out.devices[kBody].commands[3], q0 + 0.05 - 1e-3);
}

// τ > 0: the target keeps exp(−dt/τ) of its gap to the measurement — a sag the
// QP did not ask for is not taken over at once, so the servo resists it.
TEST_F(LocomotionControllerTest, ALongerWbcTargetTimeConstantKeepsPartOfTheGap) {
  BringUp(WbcTargetConfig(0.05));
  Tick();
  const double q0 = state_.devices[kBody].positions[3];
  state_.devices[kBody].positions[3] += 0.05;  // the knee sags
  const auto before = SnapshotServoTarget();
  const rtc::ControllerOutput out = Tick();
  ExpectServoTargetStep(before, out);
  const double k = std::exp(-state_.dt / 0.05);
  // The kept share of the gap, up to the tick's own (small) motion.
  EXPECT_NEAR(out.devices[kBody].commands[3], q0 + 0.05 * (1.0 - k), 1e-3);
  EXPECT_LT(out.devices[kBody].commands[3], q0 + 0.05 - 1e-3);
}

// The target never leaves the joint's range; where it is clamped its velocity
// stops, so it does not keep pushing past the limit.
TEST_F(LocomotionControllerTest, TheWbcTargetIsClampedToTheJointRange) {
  BringUp(WbcTargetConfig());
  Tick();
  const double hi = ctrl_->ReducedModel().upperPositionLimit(7 + 3);
  state_.devices[kBody].positions[3] = hi - 1e-4;
  state_.devices[kBody].velocities[3] = 1.0;  // one tick of it passes the limit
  const rtc::ControllerOutput out = Tick();
  EXPECT_EQ(out.devices[kBody].commands[3], hi);
  EXPECT_EQ(ctrl_->ServoTargetVelocity()[3], 0.0);
}

// A tick without a valid state holds the target: the measurement it would be
// pulled toward is not this tick's.
TEST_F(LocomotionControllerTest, AStaleTickHoldsTheWbcTarget) {
  BringUp(WbcTargetConfig());
  Tick();
  const rtc::ControllerOutput held = Tick();
  const rtc::ControllerOutput out = StaleTick();
  ASSERT_EQ(LastMode(), Mode::kDegraded);
  for (std::size_t i = 0; i < 23; ++i) {
    EXPECT_EQ(out.devices[kBody].commands[i], held.devices[kBody].commands[i]) << i;
  }
}

// ── Degraded and fault ──────────────────────────────────────────────────────

TEST_F(LocomotionControllerTest, AShortSourceDropHoldsTheTorqueAndALongOneLatches) {
  BringUp();
  Tick();
  const Eigen::VectorXd tau = ctrl_->LastSolve()->tau;
  const int n = cfg_["fault"]["max_source_bad_ticks"].as<int>();
  for (int k = 0; k < n; ++k) {
    const rtc::ControllerOutput out = StaleTick();
    ASSERT_EQ(LastMode(), Mode::kDegraded) << k;
    EXPECT_FALSE(ctrl_->HasLatchedFault()) << k;
    for (std::size_t i = 0; i < 23; ++i) {
      EXPECT_EQ(out.devices[kBody].feedforward[i], tau(static_cast<Eigen::Index>(i))) << i;
    }
  }
  for (std::size_t i = 0; i < 23; ++i) {
    state_.devices[kBody].velocities[i] = 0.1 * static_cast<double>(i) - 1.0;
  }
  const rtc::ControllerOutput out = StaleTick();
  EXPECT_EQ(LastMode(), Mode::kFault);
  EXPECT_TRUE(ctrl_->HasLatchedFault());
  EXPECT_EQ(static_cast<Cause>(ctrl_->LastDiag().fault_cause), Cause::kSource);
  ExpectDamping(out);
  // Latched: a good state does not release it on its own.
  ExpectDamping(Tick());
  EXPECT_EQ(LastMode(), Mode::kFault);
}

TEST_F(LocomotionControllerTest, TheResetServiceReleasesTheLatchAndReLatchesTheReference) {
  BringUp();
  Tick();
  const int n = cfg_["fault"]["max_source_bad_ticks"].as<int>();
  for (int k = 0; k <= n; ++k) {
    StaleTick();
  }
  ASSERT_TRUE(ctrl_->HasLatchedFault());
  state_.devices[kBody].positions[0] += 0.02;  // the robot settled somewhere else
  ctrl_->ResetFault();
  const rtc::ControllerOutput out = Tick();
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(ctrl_->Reference().q_des(7 + 0), state_.devices[kBody].positions[0]);
  ExpectServoTargetFromMeasurement(out);
}

TEST_F(LocomotionControllerTest, AResetWhileTheCauseStaysDoesNotCommandTheQp) {
  BringUp();
  Tick();
  const int n = cfg_["fault"]["max_source_bad_ticks"].as<int>();
  for (int k = 0; k <= n; ++k) {
    StaleTick();
  }
  ASSERT_TRUE(ctrl_->HasLatchedFault());
  ctrl_->ResetFault();
  // Still no valid state: no reference to hold, so damping, never the QP.
  for (int k = 0; k < 3; ++k) {
    StaleTick();
    EXPECT_EQ(LastMode(), Mode::kWaiting) << k;
  }
}

TEST_F(LocomotionControllerTest, ActivationDropsALatchFromTheLastSession) {
  BringUp();
  Tick();
  const int n = cfg_["fault"]["max_source_bad_ticks"].as<int>();
  for (int k = 0; k <= n; ++k) {
    StaleTick();
  }
  ASSERT_TRUE(ctrl_->HasLatchedFault());
  Deactivate();
  Activate();
  // The latch is the old session's; the first tick of this one starts over.
  Tick();
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(ctrl_->LastDiag().tick_count, 1U);
}

TEST_F(LocomotionControllerTest, ActivationWithTheCauseStillThereWaitsOnDamping) {
  BringUp();
  Tick();
  const int n = cfg_["fault"]["max_source_bad_ticks"].as<int>();
  for (int k = 0; k <= n; ++k) {
    StaleTick();
  }
  Deactivate();
  Activate();
  const rtc::ControllerOutput out = StaleTick();
  EXPECT_EQ(LastMode(), Mode::kWaiting);
  ExpectDamping(out);
}

// ── E-STOP ──────────────────────────────────────────────────────────────────

TEST_F(LocomotionControllerTest, EstopDampsAndTheClearReLatchesFromWhereTheRobotIs) {
  BringUp();
  Tick();
  ctrl_->TriggerEstop();
  ExpectDamping(Tick());
  state_.devices[kBody].positions[2] += 0.03;
  ctrl_->ClearEstop();
  const rtc::ControllerOutput out = Tick();
  EXPECT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(ctrl_->Reference().q_des(7 + 2), state_.devices[kBody].positions[2]);
  ExpectServoTargetFromMeasurement(out);
}

// ── Refusals ────────────────────────────────────────────────────────────────

TEST(LocomotionControllerConfig, RefusesEachMalformedRuntimeSection) {
  const auto refused = [](const std::function<void(YAML::Node&)>& spoil, const char* what) {
    ExpectRefused(ShippedConfig(), spoil, what);
  };
  refused([](YAML::Node& c) { c["command_type"] = "torque"; }, "torque");
  refused([](YAML::Node& c) { c["pd_feedforward"]["kp"][0] = -1.0; }, "negative kp");
  refused([](YAML::Node& c) { c["pd_feedforward"]["kd"][1] = ".nan"; }, "NaN kd");
  refused([](YAML::Node& c) { c["pd_feedforward"].remove("kd"); }, "missing kd");
  refused([](YAML::Node& c) { c["pd_feedforward"]["extra"] = 1; }, "unknown key");
  refused([](YAML::Node& c) { c["pd_feedforward"].remove("target"); }, "missing target");
  refused([](YAML::Node& c) { c["pd_feedforward"]["target"] = "measured"; }, "unknown target");
  refused([](YAML::Node& c) { c["pd_feedforward"].remove("target_time_constant_s"); },
          "missing target_time_constant_s");
  refused([](YAML::Node& c) { c["pd_feedforward"]["target_time_constant_s"] = -0.01; },
          "negative target_time_constant_s");
  refused([](YAML::Node& c) { c["pd_feedforward"]["target_time_constant_s"] = ".inf"; },
          "infinite target_time_constant_s");
  refused([](YAML::Node& c) { c["fault"]["max_source_bad_ticks"] = 0; }, "zero bad ticks");
  refused([](YAML::Node& c) { c["fault"].remove("damping_kd"); }, "missing damping");
  refused([](YAML::Node& c) { c["estimator"]["source"] = "imu"; }, "estimator source");
  refused([](YAML::Node& c) { c.remove("whole_body_qp"); }, "no whole_body_qp");
  EXPECT_NO_THROW(static_cast<void>(ParseLocomotionControllerConfig(ShippedConfig())));
}

TEST_F(LocomotionControllerTest, RefusesTopicsOutOfDeviceOrderAndGainsOfTheWrongLength) {
  {
    YAML::Node cfg = YAML::Clone(ShippedConfig());
    YAML::Node topics;
    topics["p1b"] = YAML::Clone(cfg["topics"]["p1b"]);
    topics["g1"] = YAML::Clone(cfg["topics"]["g1"]);
    cfg["topics"] = topics;
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_loco_order");
    LocomotionController ctrl("");
    ctrl.SetSystemModelConfig(fx::ShippedModelConfig(kProfile));
    ctrl.SetSharedModelBuilder(fx::ShippedModelBuilder(kProfile));
    ctrl.SetControlRate(fx::kShippedProfileControlRateHz);
    EXPECT_NE(ctrl.PreConfigure(node, cfg), CallbackReturn::SUCCESS);
  }
  {
    YAML::Node cfg = YAML::Clone(ShippedConfig());
    cfg["pd_feedforward"]["kp"].push_back(10.0);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_loco_len");
    LocomotionController ctrl("");
    ctrl.SetSystemModelConfig(fx::ShippedModelConfig(kProfile));
    ctrl.SetSharedModelBuilder(fx::ShippedModelBuilder(kProfile));
    ctrl.SetControlRate(fx::kShippedProfileControlRateHz);
    ASSERT_EQ(ctrl.PreConfigure(node, cfg), CallbackReturn::SUCCESS);
    ctrl.SetDeviceNameConfigs(ShippedDevices(cfg));
    EXPECT_EQ(ctrl.on_configure(rclcpp_lifecycle::State{}, node, cfg), CallbackReturn::FAILURE);
    EXPECT_NE(ctrl.ConfigError().find("one gain per body joint"), std::string::npos)
        << ctrl.ConfigError();
  }
}

// Every profile that ships the YAML loads it; the gain lists match the body roster.
TEST(LocomotionControllerConfig, EveryProfileThatShipsTheYamlLoadsIt) {
  for (const char* profile : {"g1_loco", "b7_loco"}) {
    SCOPED_TRACE(profile);
    const YAML::Node cfg = fx::ShippedControllerNode(profile, kKey);
    LocomotionControllerConfig parsed;
    ASSERT_NO_THROW(parsed = ParseLocomotionControllerConfig(cfg));
    const auto devices = fx::ShippedSimConfigs(profile, cfg);
    const auto& roster = devices.at(parsed.model.body_group).joint_state_names;
    EXPECT_EQ(parsed.pd_kp.size(), roster.size());
    EXPECT_EQ(parsed.pd_kd.size(), roster.size());
    EXPECT_EQ(parsed.damping_kd.size(), roster.size());
    ASSERT_EQ(cfg["topics"].size(), 1 + parsed.model.hand_groups.size());
  }
}

}  // namespace
