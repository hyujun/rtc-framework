// ── DemoJointController grasp-detection unit tests ───────────────────────────
//
// Exercises the force-derived grasp detection, the fingertip-count derivation
// (num_inference_groups preferred, sensor-stride fallback), the ToF snapshot
// sensor-lane gate, and the grasp_controller_type=="none" no-op branch — all
// without a real URDF. The controller is built with an empty urdf_path; the
// arm_handle_ FK is guarded (mirrors DemoWbcController) so only the
// grasp/sensor logic runs. arm/task-space output is not asserted here.

#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/controllers/hand_sensor_layout.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace {

using integrated_bringup::DemoJointController;
using integrated_bringup::kHandInferenceValuesPerFingertipCapacity;
using integrated_bringup::kHandSensorValuesPerFingertipCapacity;
using rtc::ControllerOutput;
using rtc::ControllerState;

// Arm: 6 joints, hand: 10 joints — both valid, at zero unless overridden.
ControllerState MakeState(double dt = 0.002) {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = dt;
  state.iteration = 1;

  auto& dev0 = state.devices[0];
  dev0.num_channels = 6;
  dev0.valid = true;

  auto& dev1 = state.devices[1];
  dev1.num_channels = 10;
  dev1.valid = true;

  return state;
}

// Write a fingertip force (Fz) + raw native-contact slot into inference_data.
// Leaves the count fields (num_inference_groups / num_sensor_channels) to the
// caller so each test pins the exact stream shape it targets.
void SetFingertipForce(ControllerState& s, int f, float fz, float contact = 0.0f) {
  auto& dev1 = s.devices[1];
  dev1.inference_enable[static_cast<std::size_t>(f)] = true;
  const int base = f * static_cast<int>(kHandInferenceValuesPerFingertipCapacity);
  dev1.inference_data[static_cast<std::size_t>(base)] = contact;   // native contact (slot 0)
  dev1.inference_data[static_cast<std::size_t>(base + 1)] = 0.0f;  // Fx
  dev1.inference_data[static_cast<std::size_t>(base + 2)] = 0.0f;  // Fy
  dev1.inference_data[static_cast<std::size_t>(base + 3)] = fz;    // Fz
}

class JointGraspTest : public ::testing::Test {
 protected:
  DemoJointController ctrl_{""};
  ControllerState state_ = MakeState();

  void SetUp() override {
    // First Compute() runs the controller-internal self-init (fallback DoF from
    // device channel counts, hold-trajectory seeding).
    (void)ctrl_.Compute(state_);
  }
};

// ── Fingertip-count derivation + force-only detection ──────────────────────

// p1b real-hw shape: force-only inference lane (num_sensor_channels=0), a fixed
// native contact_flag=0, capability flags false. Grasp detection must fall back
// to |F| > grasp_force_threshold and still report contacts + a derived binary
// contact_flag.
TEST_F(JointGraspTest, ForceOnlyFallbackDetectsContactWithZeroContactFlag) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 5.0f, /*contact=*/0.0f);
  SetFingertipForce(state_, 1, 5.0f, /*contact=*/0.0f);
  (void)ctrl_.Compute(state_);

  const auto& gs = ctrl_.GetGraspStateForTesting();
  EXPECT_EQ(gs.num_fingertips, 2);
  EXPECT_EQ(gs.num_active_contacts, 2);
  EXPECT_TRUE(gs.grasp_detected);
  EXPECT_NEAR(gs.force_magnitude[0], 5.0f, 1e-4f);
  // Derived binary contact_flag (1.0) even though the raw native slot was 0.
  EXPECT_NEAR(gs.contact_flag[0], 1.0f, 1e-5f);
  EXPECT_NEAR(gs.contact_flag[1], 1.0f, 1e-5f);
}

// Legacy mock shape: no inference groups, fingertip count derived from the
// sensor channel stride. This path must keep working after the derivation
// switched to prefer num_inference_groups.
TEST_F(JointGraspTest, ChannelStrideFallbackStillWorks) {
  state_.devices[1].num_inference_groups = 0;
  state_.devices[1].num_sensor_channels =
      2 * static_cast<int>(kHandSensorValuesPerFingertipCapacity);
  SetFingertipForce(state_, 0, 5.0f);
  SetFingertipForce(state_, 1, 5.0f);
  (void)ctrl_.Compute(state_);

  const auto& gs = ctrl_.GetGraspStateForTesting();
  EXPECT_EQ(gs.num_fingertips, 2);
  EXPECT_EQ(gs.num_active_contacts, 2);
  EXPECT_TRUE(gs.grasp_detected);
}

// Neither lane populated → zero fingertips, no grasp, no crash.
TEST_F(JointGraspTest, ZeroChannelsZeroGroupsReportsZeroFingertips) {
  state_.devices[1].num_inference_groups = 0;
  state_.devices[1].num_sensor_channels = 0;
  (void)ctrl_.Compute(state_);

  const auto& gs = ctrl_.GetGraspStateForTesting();
  EXPECT_EQ(gs.num_fingertips, 0);
  EXPECT_EQ(gs.num_active_contacts, 0);
  EXPECT_FALSE(gs.grasp_detected);
}

// ── ToF snapshot sensor-lane gate ──────────────────────────────────────────

// The ToF snapshot is gated by the raw sensor-lane fingertip count, NOT the
// inference-group count, so a force-only stream never publishes a junk all-zero
// snapshot. (populated stays false regardless because hand_handle_ is null in
// this fixture; the gate counter is the observable contract.)
TEST_F(JointGraspTest, ToFSnapshotGateUsesSensorLaneCount) {
  state_.devices[1].num_inference_groups = 4;
  state_.devices[1].num_sensor_channels = 0;
  for (int f = 0; f < 4; ++f) {
    state_.devices[1].inference_enable[static_cast<std::size_t>(f)] = true;
  }
  (void)ctrl_.Compute(state_);

  EXPECT_EQ(ctrl_.GetGraspStateForTesting().num_fingertips, 4);
  EXPECT_EQ(ctrl_.GetNumSensorFingertipsForTesting(), 0);
  EXPECT_FALSE(ctrl_.GetToFSnapshotForTesting().populated);

  // A real sensor lane makes the gate counter track the channel stride.
  state_.devices[1].num_sensor_channels =
      4 * static_cast<int>(kHandSensorValuesPerFingertipCapacity);
  (void)ctrl_.Compute(state_);
  EXPECT_EQ(ctrl_.GetNumSensorFingertipsForTesting(), 4);
}

// ── grasp_controller_type branch behaviour ─────────────────────────────────

// Drive the hand from 0.3 toward 0.8 with a strong contact present. With the
// hand device at 0.3 and 2 fingertips reporting 5 N (> threshold), the two hand
// modes must diverge: "none" lets the trajectory pass through; "contact_stop"
// freezes the hand at its current position.
namespace {
constexpr double kHandStart = 0.3;
constexpr double kHandGoal = 0.8;

void PrimeHandMotion(DemoJointController& ctrl, ControllerState& state) {
  for (int i = 0; i < 10; ++i) {
    state.devices[1].positions[static_cast<std::size_t>(i)] = kHandStart;
  }
  state.devices[1].num_inference_groups = 2;
  state.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state, 0, 5.0f);
  SetFingertipForce(state, 1, 5.0f);

  std::array<double, 10> tgt{};
  tgt.fill(kHandGoal);
  ctrl.SetDeviceTarget(1, tgt);
}

ControllerOutput RunHandTicks(DemoJointController& ctrl, ControllerState& state, int n) {
  ControllerOutput out{};
  for (int i = 0; i < n; ++i) {
    state.iteration = static_cast<uint64_t>(i + 2);
    out = ctrl.Compute(state);
  }
  return out;
}
}  // namespace

TEST_F(JointGraspTest, NoneTypeSkipsHandIntervention) {
  ctrl_.SetGraspControllerTypeForTesting("none");
  PrimeHandMotion(ctrl_, state_);
  // 300 ticks × 2 ms = 0.6 s > trajectory duration (~0.5 s): motion completes.
  auto out = RunHandTicks(ctrl_, state_, 300);

  // Contact is still observed (GraspState publishing is unaffected by "none").
  EXPECT_TRUE(ctrl_.GetGraspStateForTesting().grasp_detected);
  // Hand followed the trajectory well past the 0.3 start — no freeze.
  EXPECT_GT(out.devices[1].commands[0], 0.5);
}

// The mode reaches the RT tick through the Gains SeqLock snapshot, not through a
// plain member (B-1 RT handoff): the tick loads Gains once, so the mode and the
// thresholds it is read against can never be observed half-applied.
//
// This is the pin for that handoff, and the two halves fail for different
// reasons. The storage half catches a writer that stores the mode somewhere the
// tick does not read (SetGraspControllerTypeForTesting keeping its own copy);
// the behaviour half catches a tick that reads somewhere the writers do not
// write. Only the behaviour half needs a non-default mode to be conclusive:
// asking for kContactStop proves nothing, since that is what an ignored Gains
// field would leave behind anyway.
TEST_F(JointGraspTest, GraspModeTravelsThroughTheGainsSnapshot) {
  ctrl_.SetGraspControllerTypeForTesting("force_pi");
  EXPECT_EQ(ctrl_.get_gains().grasp_hand_mode, integrated_bringup::GraspHandMode::kForcePi);
  ctrl_.SetGraspControllerTypeForTesting("contact_stop");
  EXPECT_EQ(ctrl_.get_gains().grasp_hand_mode, integrated_bringup::GraspHandMode::kContactStop);

  // Behaviour half: steer the branch through the PUBLIC gains path only, away
  // from the kContactStop default.
  auto g = ctrl_.get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kNone;
  ctrl_.set_gains(g);
  PrimeHandMotion(ctrl_, state_);
  auto out = RunHandTicks(ctrl_, state_, 300);
  // Contact is observed either way — a tick that ignored the Gains field would
  // still freeze the hand at 0.3 instead of letting it reach the 0.8 goal.
  ASSERT_TRUE(ctrl_.GetGraspStateForTesting().grasp_detected);
  EXPECT_GT(out.devices[1].commands[0], 0.5);
}

TEST_F(JointGraspTest, ContactStopFreezesHandAtCurrentPosition) {
  // Default grasp_controller_type is "contact_stop" (control group).
  PrimeHandMotion(ctrl_, state_);
  auto out = RunHandTicks(ctrl_, state_, 300);

  EXPECT_TRUE(ctrl_.GetGraspStateForTesting().grasp_detected);
  // Hand frozen at the current actual position (0.3), not the 0.8 goal. The
  // hold runs through the position LPF; with a constant measured 0.3 the filter
  // settles to 0.3 (DC pass-through), so a tight tolerance still holds.
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-4);
}

// #162 latch: once contact engages, the hold must persist after contact is lost
// (object slips) until an explicit release. Previously the freeze was unlatched
// and re-evaluated per tick, so a single no-contact tick snapped the command
// back to the (goal-advanced) trajectory.
TEST_F(JointGraspTest, ContactStopLatchHoldsAfterContactLost) {
  PrimeHandMotion(ctrl_, state_);
  (void)RunHandTicks(ctrl_, state_, 100);  // engage + latch, held near 0.3

  // Contact disappears with no release command issued.
  SetFingertipForce(state_, 0, 0.0f);
  SetFingertipForce(state_, 1, 0.0f);
  auto out = RunHandTicks(ctrl_, state_, 300);

  EXPECT_FALSE(ctrl_.GetGraspStateForTesting().grasp_detected);
  // Latched: hand keeps its held position, does NOT snap to the 0.8 goal.
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-3);
  EXPECT_LT(out.devices[1].commands[0], 0.5);
}

// #162 release gate still drops the latch even while contact force persists:
// the gate infers release *intent* from the goal direction, not contact loss.
TEST_F(JointGraspTest, ContactStopReleaseGateClearsLatch) {
  PrimeHandMotion(ctrl_, state_);
  (void)RunHandTicks(ctrl_, state_, 100);  // latched at ~0.3, contact still 5 N

  // Command an opening motion that satisfies every release gate
  // (idx {1,4,7}, signs {+1,-1,-1}); contact is deliberately kept present.
  std::array<double, 10> open{};
  open.fill(kHandStart);
  open[1] = kHandStart + 0.3;  // sign +1 → target > actual ⇒ loosening
  open[4] = kHandStart - 0.3;  // sign -1 → target < actual ⇒ loosening
  open[7] = kHandStart - 0.3;  // sign -1 → target < actual ⇒ loosening
  ctrl_.SetDeviceTarget(1, open);
  auto out = RunHandTicks(ctrl_, state_, 300);

  // Latch dropped: gated joints track the opening trajectory off the hold.
  EXPECT_GT(out.devices[1].commands[1], kHandStart + 0.1);
  EXPECT_LT(out.devices[1].commands[4], kHandStart - 0.1);
}

// #162 the latched hold is the LPF output, not the raw measured position: a step
// in the measured position must be tracked with lag, not instantaneously.
TEST_F(JointGraspTest, ContactStopHoldUsesFilteredPosition) {
  PrimeHandMotion(ctrl_, state_);
  (void)RunHandTicks(ctrl_, state_, 200);  // filter settled to 0.3, latched

  // Object pushes finger 0: measured jumps 0.3 → 0.5, contact kept present.
  state_.devices[1].positions[0] = 0.5;
  auto out = RunHandTicks(ctrl_, state_, 3);  // a few ticks of the step

  // LPF lag: the hold has moved off 0.3 but has not jumped to the raw 0.5.
  EXPECT_GT(out.devices[1].commands[0], kHandStart + 1e-4);
  EXPECT_LT(out.devices[1].commands[0], 0.5 - 1e-3);
}

// ── "contact_stop never runs the PI law" — at the compute level ─────────────
//
// Today this guarantee is held by a BUILD-TIME fact: BuildGraspController only
// constructs the Force-PI controller when the type is "force_pi", so under
// contact_stop `grasp_controller_` is null and the `grasp_controller_ && mode ==
// kForcePi` branch cannot be taken whatever the mode says. That is about to stop
// being true — making the build mode-independent is the next commit — so the
// guarantee needs an owner in ComputeControl BEFORE it loses the one it has.
//
// The fixture is the state that used to be unreachable: a controller that DID
// build the PI controller, then had its mode moved to contact_stop. Only step 1's
// Gains handoff makes that constructible from a test at all.
namespace {
// Minimal LoadConfig payload that BUILDS the Force-PI controller — the
// whitelisted type AND a `force_pi_grasp` block, the two conditions
// BuildGraspController ANDs. `arm_safe_position` is 6 wide to match MakeState's
// 6 arm channels so the arm stays readable. Under an empty config_variant the
// shared demo_shared.yaml path does not resolve, so the built-in defaults plus
// this block are the whole configuration (3 fingers, joint map {0,1,2}/{3,4,5}/
// {6,7,8}) — nothing here depends on an installed YAML.
const char* const kForcePiYaml = R"(
estop:
  arm_safe_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fsm:
  contact_stop_release_eps: 0.005
grasp_controller_type: "force_pi"
force_pi_grasp:
  f_target: 2.5
  fingers:
    thumb:
      q_open:  [0.0, 0.0, 0.0]
      q_close: [0.5, 1.0, 0.7]
    index:
      q_open:  [0.0, 0.0, 0.0]
      q_close: [0.5, 1.0, 0.7]
    middle:
      q_open:  [0.0, 0.0, 0.0]
      q_close: [0.5, 1.0, 0.7]
)";

class JointForcePiBuiltTest : public ::testing::Test {
 protected:
  DemoJointController ctrl_{""};
  ControllerState state_ = MakeState();

  // Mode the controller runs under from its FIRST tick. Overridden below to get a
  // history with no force_pi tick in it at all — some of the published grasp
  // fields are FSM latches, so "never published" and "published then switched
  // away from" are different states and only one of them is this file's business.
  virtual integrated_bringup::GraspHandMode InitialMode() const {
    return integrated_bringup::GraspHandMode::kForcePi;
  }

  void SetUp() override {
    ctrl_.SetControlRate(1.0 / 0.002);
    ctrl_.LoadConfig(YAML::Load(kForcePiYaml));
    // Precondition, not an assumption: if the block stopped building the PI
    // controller, every assertion below would pass for the wrong reason.
    ASSERT_EQ(ctrl_.get_gains().grasp_hand_mode, integrated_bringup::GraspHandMode::kForcePi);
    auto g = ctrl_.get_gains();
    g.grasp_hand_mode = InitialMode();
    ctrl_.set_gains(g);
    (void)ctrl_.Compute(state_);
  }
};

class JointContactStopWithPiBuiltTest : public JointForcePiBuiltTest {
 protected:
  integrated_bringup::GraspHandMode InitialMode() const override {
    return integrated_bringup::GraspHandMode::kContactStop;
  }
};
}  // namespace

// Control arm. With the PI controller built and the mode left at force_pi, the
// contact_stop freeze must NOT run: the FSM sits in kIdle (nothing has commanded
// a grasp), so the PI law writes no finger and the trajectory reaches its goal.
// The published target_force is the PI law's fingerprint — FillLogOutput only
// copies it on the force_pi branch, and 2.5 comes from the YAML above.
TEST_F(JointForcePiBuiltTest, ForcePiModeSkipsTheFreezeAndPublishesPiDiagnostics) {
  PrimeHandMotion(ctrl_, state_);
  auto out = RunHandTicks(ctrl_, state_, 300);

  ASSERT_TRUE(ctrl_.GetGraspStateForTesting().grasp_detected);
  EXPECT_GT(out.devices[1].commands[0], 0.5) << "contact_stop freeze ran under force_pi";
  EXPECT_NEAR(ctrl_.GetGraspStateForTesting().grasp_target_force, 2.5F, 1e-5F);
}

// The pin. Same built PI controller, mode moved to contact_stop: the hand must
// freeze at the measured position, which is only reachable through the `else if`
// branch. A tick that let a non-null grasp_controller_ decide on its own would
// run the PI branch here and the hand would sail to the 0.8 goal.
TEST_F(JointForcePiBuiltTest, ContactStopModeFreezesEvenWhenThePiControllerExists) {
  auto g = ctrl_.get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kContactStop;
  ctrl_.set_gains(g);

  PrimeHandMotion(ctrl_, state_);
  auto out = RunHandTicks(ctrl_, state_, 300);

  ASSERT_TRUE(ctrl_.GetGraspStateForTesting().grasp_detected);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-4);
}

// Same guarantee, OTHER consumer of the mode. FillLogOutput copies the FSM phase
// / target force / per-finger PI state out of grasp_controller_, and it reads the
// mode from grasp_hand_mode_cached_ rather than from the Gains snapshot the
// control law branched on. That second read site is why this is its own
// assertion: publishing a target force the control law never applied would make
// the CSV and the GUI describe a law that did not run.
//
// The fixture runs contact_stop from tick 1 on purpose. These fields are FSM
// latches — nothing rewrites them on a tick that does not take the force_pi
// branch — so a fixture that ticked force_pi first would leave a stale 2.5 N here
// no matter how correct the gate is, and the assertion would be about latch
// clearing rather than about publishing.
TEST_F(JointContactStopWithPiBuiltTest, ContactStopModePublishesNoPiDiagnostics) {
  PrimeHandMotion(ctrl_, state_);
  (void)RunHandTicks(ctrl_, state_, 300);

  const auto& gs = ctrl_.GetGraspStateForTesting();
  EXPECT_NEAR(gs.grasp_target_force, 0.0F, 1e-9F)
      << "PI target force published while the PI law was not running";
  EXPECT_EQ(gs.grasp_phase, 0U);
  for (std::size_t f = 0; f < 3; ++f) {
    EXPECT_NEAR(gs.finger_s[f], 0.0F, 1e-9F) << "finger " << f;
  }
}

// The freeze half again, but with contact_stop in force from the first tick — the
// shape a controller that STARTED in contact_stop with the block present will
// have once the build stops looking at the mode. The sister test above covers the
// runtime-switch shape; this one covers the config-time shape, and they exercise
// different histories through the same branch.
TEST_F(JointContactStopWithPiBuiltTest, ContactStopFromTheFirstTickStillFreezes) {
  PrimeHandMotion(ctrl_, state_);
  auto out = RunHandTicks(ctrl_, state_, 300);

  ASSERT_TRUE(ctrl_.GetGraspStateForTesting().grasp_detected);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-4);
}

// ── Mode-switch race closure (B-4a) ─────────────────────────────────────────

// The latch mirror the non-RT quiet gate reads. Its own test rather than a rider
// on the freeze assertions: a tick that freezes the hand but never mirrors leaves
// the gate believing the hand is quiet, and no command assertion can see that.
TEST_F(JointContactStopWithPiBuiltTest, LatchMirrorTracksTheLatchForTheQuietGate) {
  EXPECT_FALSE(ctrl_.GetContactLatchedMirrorForTesting()) << "mirror set before any contact";

  PrimeHandMotion(ctrl_, state_);
  (void)RunHandTicks(ctrl_, state_, 100);
  EXPECT_TRUE(ctrl_.GetContactLatchedMirrorForTesting())
      << "hold engaged but the gate cannot see it";

  // Release intent on every gate joint (idx {1,4,7}, signs {+1,-1,-1}) drops the
  // latch with contact still present — the mirror has to follow that path too,
  // not just the engage path.
  std::array<double, 10> open{};
  open.fill(kHandStart);
  open[1] = kHandStart + 0.3;
  open[4] = kHandStart - 0.3;
  open[7] = kHandStart - 0.3;
  ctrl_.SetDeviceTarget(1, open);
  (void)RunHandTicks(ctrl_, state_, 300);
  EXPECT_FALSE(ctrl_.GetContactLatchedMirrorForTesting());
}

// The gate's OTHER input, and the one that would otherwise go unverified until a
// live robot: the FSM phase. GraspController::Update() mutates the FSM on the RT
// tick, so the callback reads a mirror instead of phase() — and a mirror that never
// leaves 0 is indistinguishable from an idle hand, which is precisely the state the
// gate treats as "safe to switch". So drive the FSM off Idle and watch the mirror
// follow.
TEST_F(JointForcePiBuiltTest, PhaseMirrorLeavesIdleOnceAGraspIsCommanded) {
  ASSERT_EQ(ctrl_.GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle));

  ASSERT_TRUE(ctrl_.CommandGraspForTesting(2.0)) << "fixture built no PI controller";
  // CommandGrasp only arms the transition; the FSM steps inside Update(), which
  // only the force_pi branch calls — so the mirror can only move via a tick.
  PrimeHandMotion(ctrl_, state_);
  (void)RunHandTicks(ctrl_, state_, 5);

  EXPECT_NE(ctrl_.GetGraspPhaseMirrorForTesting(),
            static_cast<uint8_t>(rtc::grasp::GraspPhase::kIdle))
      << "the quiet gate would admit a mode change mid-grasp";
}

// The race the quiet gate cannot close by itself: the callback checks the mirror,
// contact engages, and only then does the mode Store land. The tick that observes
// the new mode stops running the hold — and hand_computed_ falls back to a
// trajectory still aimed at the PRE-CONTACT goal, which by now has run to
// completion at 0.8. On a held object that is a resumed squeeze.
//
// set_gains() here IS the race: it skips the gate exactly the way a gate with a
// hole would. Without the closure the command snaps from the 0.3 hold to 0.8.
TEST_F(JointContactStopWithPiBuiltTest, SwitchingAwayWhileLatchedDoesNotSnapBackToTheOldGoal) {
  PrimeHandMotion(ctrl_, state_);
  auto out = RunHandTicks(ctrl_, state_, 300);
  ASSERT_NEAR(out.devices[1].commands[0], kHandStart, 1e-4) << "fixture never latched";
  ASSERT_TRUE(ctrl_.GetContactLatchedMirrorForTesting());

  auto g = ctrl_.get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kForcePi;
  ctrl_.set_gains(g);

  // First tick after the switch, then well past the old trajectory's duration.
  out = RunHandTicks(ctrl_, state_, 1);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-3) << "snapped back on the switch tick";
  out = RunHandTicks(ctrl_, state_, 300);
  EXPECT_NEAR(out.devices[1].commands[0], kHandStart, 1e-3) << "ramped toward the old goal after";

  // The goal itself was re-seeded, not just the command — otherwise the next
  // re-plan (any new target, or the trajectory being re-initialised) walks back
  // to 0.8 and the hold above was only a reprieve.
  EXPECT_NEAR(out.devices[1].goal_positions[0], kHandStart, 1e-3);
  // And the closure dropped the latch: it handed ownership of the hand to the
  // new mode rather than leaving a contact_stop latch behind under force_pi.
  EXPECT_FALSE(ctrl_.GetContactLatchedMirrorForTesting());
}

// The other half of the closure's guard, and the half that is actually reachable:
// on a QUIET switch — the normal path, the one the gate admits — the closure must
// stay out of the way. A closure keyed on the mode edge alone would re-seed the
// goal to wherever the hand happens to be and silently discard the operator's
// pending target on every legitimate switch.
//
// (The `!run_contact_stop` term of the guard has no reachable test: a latch can
// only be set while the hold is the law, and the closure drops it on the way out,
// so "latched while contact_stop is the law again" cannot be constructed. It is
// defence-in-depth, like the WARN.)
TEST_F(JointContactStopWithPiBuiltTest, QuietSwitchKeepsTheOperatorsGoal) {
  // No contact anywhere, so the latch never engages.
  for (int i = 0; i < 10; ++i) {
    state_.devices[1].positions[static_cast<std::size_t>(i)] = kHandStart;
  }
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  std::array<double, 10> tgt{};
  tgt.fill(kHandGoal);
  ctrl_.SetDeviceTarget(1, tgt);
  (void)RunHandTicks(ctrl_, state_, 50);
  ASSERT_FALSE(ctrl_.GetContactLatchedMirrorForTesting()) << "fixture latched without contact";

  auto g = ctrl_.get_gains();
  g.grasp_hand_mode = integrated_bringup::GraspHandMode::kForcePi;
  ctrl_.set_gains(g);
  auto out = RunHandTicks(ctrl_, state_, 300);

  EXPECT_NEAR(out.devices[1].goal_positions[0], kHandGoal, 1e-9)
      << "quiet switch discarded the goal";
  EXPECT_NEAR(out.devices[1].commands[0], kHandGoal, 1e-3);
}

// ── Deferred hand hold-seed (device-1 startup race) ─────────────────────────
// If the hand device (device 1) is not yet valid on the controller's first
// Compute() tick, its hold target must NOT latch to the zero-initialized
// targets[1] — otherwise every finger is commanded toward 0 the moment the
// device becomes valid (the reported ur5e-fine / p1b-collapse asymmetry). The
// hand self-init is deferred (split from the arm flag) and re-seeds from the
// measured pose on the first valid tick.
TEST(JointHandSeedTest, DeferredHandSeedHoldsMeasuredWhenDeviceValidLate) {
  DemoJointController ctrl{""};

  // Tick 1: arm valid, hand NOT valid yet, at a non-zero rest pose (0.5 rad).
  ControllerState state = MakeState();
  state.devices[1].valid = false;
  for (int i = 0; i < 10; ++i) {
    state.devices[1].positions[static_cast<std::size_t>(i)] = 0.5;
  }
  (void)ctrl.Compute(state);  // arm seeds immediately; hand seed deferred

  // Tick 2: the hand now streams a valid measured state.
  state.devices[1].valid = true;
  state.iteration = 2;
  const auto out = ctrl.Compute(state);

  // Hand command holds the measured 0.5 rest pose, not a 0-collapse.
  ASSERT_GT(out.num_devices, 1);
  for (int i = 0; i < 10; ++i) {
    EXPECT_NEAR(out.devices[1].commands[static_cast<std::size_t>(i)], 0.5, 1e-6)
        << "finger " << i << " collapsed toward 0 instead of holding measured pose";
  }
}

// ── E-STOP telemetry freshness (#234 P-1) ──────────────────────────────────
//
// The CM stamps a publish snapshot every tick and the publish thread re-Loads
// the controller-owned SeqLock, so an E-STOP tick that stores nothing ships
// the pre-E-STOP body under the current stamp. These tests read through the
// SeqLock (GetPublishedGraspStateForTesting), not the staging buffer, because
// that is the only way to tell "filled but never stored" from "published".

TEST_F(JointGraspTest, EstopTickPublishesThisTicksBody) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 5.0f);
  SetFingertipForce(state_, 1, 5.0f);
  (void)ctrl_.Compute(state_);
  ASSERT_TRUE(ctrl_.GetPublishedGraspStateForTesting().grasp_detected);

  // The object is released *during* the E-STOP: the published body must follow
  // the sensors, which it can only do if this tick stored anything at all.
  ctrl_.TriggerEstop();
  SetFingertipForce(state_, 0, 0.0f);
  SetFingertipForce(state_, 1, 0.0f);
  state_.iteration = 2;
  (void)ctrl_.Compute(state_);

  const auto published = ctrl_.GetPublishedGraspStateForTesting();
  EXPECT_EQ(published.num_active_contacts, 0);
  EXPECT_FALSE(published.grasp_detected);
  EXPECT_NEAR(published.force_magnitude[0], 0.0f, 1e-4f);
  // Never a live-looking pull estimate under an E-STOP stamp.
  EXPECT_FALSE(published.pull.valid);
  EXPECT_FALSE(published.pull.slip_risk);
}

TEST_F(JointGraspTest, EstopTickClearsControlLawDerivedDiagnostics) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 5.0f);
  SetFingertipForce(state_, 1, 5.0f);
  (void)ctrl_.Compute(state_);

  ctrl_.TriggerEstop();
  state_.iteration = 2;
  (void)ctrl_.Compute(state_);

  // The Force-PI servo was not stepped this tick, so its per-finger telemetry
  // is neutralized rather than frozen at the last pre-E-STOP sample.
  const auto published = ctrl_.GetPublishedGraspStateForTesting();
  for (std::size_t i = 0; i < published.finger_s.size(); ++i) {
    EXPECT_FLOAT_EQ(published.finger_s[i], 0.0f) << "finger " << i;
    EXPECT_FLOAT_EQ(published.finger_filtered_force[i], 0.0f) << "finger " << i;
    EXPECT_FLOAT_EQ(published.finger_force_error[i], 0.0f) << "finger " << i;
  }
}

// ── contact_stop force LPF ───────────────────────────────────────────────────

// The freeze decision reads LPF'd force while the published GraspState keeps
// the raw value. Both halves matter: without the first, a single noisy sample
// can latch a freeze; without the second, every BT consumer of
// GraspState.max_force silently inherits the filter's group delay.
TEST_F(JointGraspTest, FilteredForceLagsRawWhilePublishedStaysRaw) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  // Valid but unloaded, long enough for the filter to settle at 0 — the step
  // below therefore starts from a primed filter rather than from a seed.
  SetFingertipForce(state_, 0, 0.0f);
  SetFingertipForce(state_, 1, 0.0f);
  for (int i = 0; i < 60; ++i) {
    state_.iteration = i + 1;
    (void)ctrl_.Compute(state_);
  }

  SetFingertipForce(state_, 0, 5.0f);
  SetFingertipForce(state_, 1, 5.0f);
  state_.iteration = 61;
  (void)ctrl_.Compute(state_);

  EXPECT_NEAR(ctrl_.GetGraspStateForTesting().max_force, 5.0f, 1e-4f)
      << "published aggregate must stay raw";
  EXPECT_LT(ctrl_.GetContactStopAggregatesForTesting().max_force, 5.0f * 0.9f)
      << "contact_stop aggregate must lag the step";

  for (int i = 0; i < 120; ++i) {
    state_.iteration = 62 + i;
    (void)ctrl_.Compute(state_);
  }
  EXPECT_NEAR(ctrl_.GetContactStopAggregatesForTesting().max_force, 5.0f, 1e-2f);
}

// An invalid fingertip must not advance its filter with the zeroed force
// ReadState writes: an IIR would carry that phantom sample for several ticks
// after the lane recovers. Re-entry seeds to the live force instead — erring
// toward an early freeze, which is the fail-safe direction for this latch.
TEST_F(JointGraspTest, InvalidFingertipFreezesFilterAndReEntrySeeds) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 5.0f);
  SetFingertipForce(state_, 1, 5.0f);
  for (int i = 0; i < 150; ++i) {
    state_.iteration = i + 1;
    (void)ctrl_.Compute(state_);
  }
  ASSERT_NEAR(ctrl_.GetContactStopAggregatesForTesting().max_force, 5.0f, 1e-2f);

  state_.devices[1].inference_enable[0] = false;
  state_.devices[1].inference_enable[1] = false;
  for (int i = 0; i < 5; ++i) {
    state_.iteration = 151 + i;
    (void)ctrl_.Compute(state_);
  }
  EXPECT_NEAR(ctrl_.GetContactStopAggregatesForTesting().max_force, 0.0f, 1e-6f)
      << "invalid fingertips report no force";

  // One tick back at a much lower force: seeded, so no residue of the old 5 N.
  SetFingertipForce(state_, 0, 0.2f);
  SetFingertipForce(state_, 1, 0.2f);
  state_.iteration = 160;
  (void)ctrl_.Compute(state_);
  EXPECT_NEAR(ctrl_.GetContactStopAggregatesForTesting().max_force, 0.2f, 1e-4f);
}

// ── contact_stop force delta-spike guard ─────────────────────────────────────
//
// Wiring only: the guard's own laws are pinned in test_fingertip_force_guard.cpp.
// What these tests establish is that the LPF is fed the GUARDED triplet, that the
// raw lane is untouched, and that the per-tick verdict is observable.

// The 260730_1017 p1b failure mode, reproduced: a 2-tick fz spike (6.406 then
// 6.058 N) on an otherwise ~0.02 N lane. Fed straight into the Bessel LPF it
// left ~0.170 N of filtered tail — 17 % of a 1 N contact threshold, from noise.
TEST_F(JointGraspTest, TwoTickForceSpikeIsHeldOutOfTheFilterWhileRawIsUntouched) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 0.02f);
  SetFingertipForce(state_, 1, 0.02f);
  for (int i = 0; i < 80; ++i) {
    state_.iteration = i + 1;
    (void)ctrl_.Compute(state_);
  }
  const float settled = ctrl_.GetContactStopAggregatesForTesting().max_force;
  ASSERT_NEAR(settled, 0.02f, 1e-3f) << "filter must be settled before the spike";

  SetFingertipForce(state_, 0, 6.406f);
  state_.iteration = 81;
  (void)ctrl_.Compute(state_);
  EXPECT_TRUE(ctrl_.WasForceGuardRejectedForTesting(0));
  // Raw is untouched: the published GraspState still reports the wire value, so
  // grasp detection and every BT consumer see exactly what the driver sent.
  EXPECT_NEAR(ctrl_.GetGraspStateForTesting().max_force, 6.406f, 1e-3f);
  // The LPF was fed the held triplet, not the spike.
  EXPECT_NEAR(ctrl_.GetGuardedFingertipForceForTesting()[2], 0.02f, 1e-4f);

  SetFingertipForce(state_, 0, 6.058f);
  state_.iteration = 82;
  (void)ctrl_.Compute(state_);
  EXPECT_TRUE(ctrl_.WasForceGuardRejectedForTesting(0))
      << "the second spike tick is compared against last_accepted, not the first spike";
  EXPECT_NEAR(ctrl_.GetGuardedFingertipForceForTesting()[2], 0.02f, 1e-4f);

  SetFingertipForce(state_, 0, 0.04f);
  state_.iteration = 83;
  (void)ctrl_.Compute(state_);
  EXPECT_FALSE(ctrl_.WasForceGuardRejectedForTesting(0));
  EXPECT_NEAR(ctrl_.GetGuardedFingertipForceForTesting()[2], 0.04f, 1e-4f);

  // No tail: without the guard the filtered aggregate peaks near 0.17 N here.
  for (int i = 0; i < 20; ++i) {
    state_.iteration = 84 + i;
    (void)ctrl_.Compute(state_);
    EXPECT_LT(ctrl_.GetContactStopAggregatesForTesting().max_force, 0.06f) << "tick " << i;
  }
}

// The other half of the contract: a step that persists is real. Without the
// consecutive-hold cap the guard would reject a genuine 6 N contact on EVERY
// tick (|6 - 0| > 4 forever) and contact_stop could never fire.
TEST_F(JointGraspTest, SustainedForceStepReachesTheFilterAfterTheHoldCap) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 0.0f);
  SetFingertipForce(state_, 1, 0.0f);
  for (int i = 0; i < 60; ++i) {
    state_.iteration = i + 1;
    (void)ctrl_.Compute(state_);
  }

  SetFingertipForce(state_, 0, 6.0f);
  SetFingertipForce(state_, 1, 6.0f);
  for (int i = 0; i < 2; ++i) {
    state_.iteration = 61 + i;
    (void)ctrl_.Compute(state_);
    EXPECT_TRUE(ctrl_.WasForceGuardRejectedForTesting(0)) << "hold tick " << i;
    EXPECT_NEAR(ctrl_.GetGuardedFingertipForceForTesting()[2], 0.0f, 1e-6f) << "hold tick " << i;
  }
  state_.iteration = 63;
  (void)ctrl_.Compute(state_);
  EXPECT_FALSE(ctrl_.WasForceGuardRejectedForTesting(0)) << "hold cap must let a real step through";
  EXPECT_NEAR(ctrl_.GetGuardedFingertipForceForTesting()[2], 6.0f, 1e-4f);

  for (int i = 0; i < 200; ++i) {
    state_.iteration = 64 + i;
    (void)ctrl_.Compute(state_);
  }
  EXPECT_NEAR(ctrl_.GetContactStopAggregatesForTesting().max_force, 6.0f, 1e-2f);
}

// NaN has no hold cap: admitted once, it poisons the IIR delay line for good and
// the downstream max/threshold arithmetic then reads as a plausible number
// instead of failing loudly.
TEST_F(JointGraspTest, NonFiniteForceNeverReachesTheFilter) {
  state_.devices[1].num_inference_groups = 2;
  state_.devices[1].num_sensor_channels = 0;
  SetFingertipForce(state_, 0, 0.5f);
  SetFingertipForce(state_, 1, 0.5f);
  for (int i = 0; i < 80; ++i) {
    state_.iteration = i + 1;
    (void)ctrl_.Compute(state_);
  }
  ASSERT_NEAR(ctrl_.GetContactStopAggregatesForTesting().max_force, 0.5f, 1e-3f);

  SetFingertipForce(state_, 0, std::numeric_limits<float>::quiet_NaN());
  for (int i = 0; i < 20; ++i) {
    state_.iteration = 81 + i;
    (void)ctrl_.Compute(state_);
    EXPECT_TRUE(ctrl_.WasForceGuardRejectedForTesting(0)) << "tick " << i;
    const float agg = ctrl_.GetContactStopAggregatesForTesting().max_force;
    EXPECT_TRUE(std::isfinite(agg)) << "tick " << i;
    EXPECT_NEAR(agg, 0.5f, 1e-3f) << "tick " << i;
  }

  SetFingertipForce(state_, 0, std::numeric_limits<float>::infinity());
  state_.iteration = 200;
  (void)ctrl_.Compute(state_);
  EXPECT_TRUE(ctrl_.WasForceGuardRejectedForTesting(0));
  EXPECT_TRUE(std::isfinite(ctrl_.GetContactStopAggregatesForTesting().max_force));

  // Recovery: a finite in-band sample is accepted again immediately.
  SetFingertipForce(state_, 0, 0.6f);
  state_.iteration = 201;
  (void)ctrl_.Compute(state_);
  EXPECT_FALSE(ctrl_.WasForceGuardRejectedForTesting(0));
  EXPECT_NEAR(ctrl_.GetGuardedFingertipForceForTesting()[2], 0.6f, 1e-4f);
}

}  // namespace
