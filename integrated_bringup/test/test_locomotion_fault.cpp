// locomotion_controller's fault and E-STOP behaviour on the installed g1_loco
// profile: the tick under the E-STOP (waiting — nothing latched, solved or sent
// to the MPC, everything logged), the two edges and the packed E-STOP word, the
// pose guard as a pure function and at every entry (activation, reset, clear,
// a lost plan — with its settle time and its fault), the damping law against
// its oracle bit for bit, the hands held from their first readable tick, the
// lever rule's bounds, the plan lost while a foot swings, the configuration
// refusals of fault.pose_guard, and the gain tripwire behind the CM's E-STOP
// hold (the locomotion gains must stay below demo_joint's: the hold keeps the
// last gains on the simulator's wire, and a soft collapse under them is the
// documented behaviour).
#include "integrated_bringup/controllers/locomotion/pose_guard.hpp"
#include "locomotion_mpc_stub.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace loco_rig;  // NOLINT(google-build-using-namespace) — the shared rig
using integrated_bringup::LocomotionControllerTestAccess;
using integrated_bringup::LocomotionDiagLogPod;
using Source = LocomotionController::ReferenceSource;
using loco::EvaluatePoseGuard;
using loco::PoseGuardInput;
using loco::PoseGuardLimits;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// ── The pure function ───────────────────────────────────────────────────────

// The shipped limit, as the controller parses it (not π/6: 0.5236 is the YAML value).
double ShippedTilt() {
  return ParseLocomotionControllerConfig(ShippedConfig()).pose_guard_base_tilt_rad;
}

// A rotation of `angle` about the x axis, as (x, y, z, w).
std::array<double, 4> RollQuaternion(double angle) {
  return {std::sin(0.5 * angle), 0.0, 0.0, std::cos(0.5 * angle)};
}

PoseGuardInput UprightInput(const double* z, int nc) {
  PoseGuardInput in;
  in.base_valid = true;
  in.q_xyzw = {0.0, 0.0, 0.0, 1.0};
  in.v_base = {0.0, 0.0, 0.0};
  in.contact_z = z;
  in.num_contacts = nc;
  return in;
}

PoseGuardLimits StanceLimits(double tilt) {
  PoseGuardLimits lim;
  lim.cos_tilt = std::cos(tilt);
  lim.check_stance = true;
  lim.max_spread_m = integrated_bringup::kRelatchFootSpreadM;
  lim.max_speed = integrated_bringup::kRelatchBaseSpeed;
  return lim;
}

TEST(PoseGuard, TheTiltLimitSeparatesAStandFromAFallAndALyingBody) {
  const double tilt = ShippedTilt();
  PoseGuardLimits lim;
  lim.cos_tilt = std::cos(tilt);
  const double z[2] = {0.0, 0.0};
  PoseGuardInput in = UprightInput(z, 2);
  EXPECT_EQ(EvaluatePoseGuard(in, lim), 0U);
  in.q_xyzw = RollQuaternion(5.0 * M_PI / 180.0);
  EXPECT_EQ(EvaluatePoseGuard(in, lim), 0U) << "5 degrees is a stand";
  in.q_xyzw = RollQuaternion(45.0 * M_PI / 180.0);
  EXPECT_EQ(EvaluatePoseGuard(in, lim), loco::kPoseGuardTilt) << "45 degrees is a fall";
  in.q_xyzw = RollQuaternion(M_PI / 2.0);
  EXPECT_EQ(EvaluatePoseGuard(in, lim), loco::kPoseGuardTilt) << "lying on the side";
  // A yaw of any size is not a tilt.
  in.q_xyzw = {0.0, 0.0, std::sin(1.0), std::cos(1.0)};
  EXPECT_EQ(EvaluatePoseGuard(in, lim), 0U);
}

TEST(PoseGuard, TheBoundaryIsTheParsedLimitToANanoradian) {
  // φ = θ ∓ 2e-9 about x gives R22 − cos θ = ±2e-9 · sin θ (≈ ±1e-9 at the
  // shipped 30°), which the fixture asserts about itself before asking the
  // guard. No exact-equality case: cos(θ ∓ 2e-9) lands on either side of
  // cos θ, never on it.
  const double tilt = ShippedTilt();
  PoseGuardLimits lim;
  lim.cos_tilt = std::cos(tilt);
  const double z[2] = {0.0, 0.0};
  for (const double sign : {-1.0, +1.0}) {
    const double phi = tilt + sign * 2e-9;
    PoseGuardInput in = UprightInput(z, 2);
    in.q_xyzw = RollQuaternion(phi);
    const double r22 = 1.0 - 2.0 * in.q_xyzw[0] * in.q_xyzw[0];
    ASSERT_NEAR(r22 - lim.cos_tilt, -sign * 2e-9 * std::sin(tilt), 1e-12)
        << "the fixture is not at the boundary";
    EXPECT_EQ(EvaluatePoseGuard(in, lim), sign < 0 ? 0U : loco::kPoseGuardTilt) << sign;
  }
}

TEST(PoseGuard, EveryNonFiniteOrNonUnitInputFailsAndNamesItsBit) {
  PoseGuardLimits stance = StanceLimits(ShippedTilt());
  const double z[2] = {0.0, 0.0};
  {
    PoseGuardInput in = UprightInput(z, 2);
    in.q_xyzw = {0.0, 0.0, 0.0, 0.0};  // the zero quaternion: no rotation at all
    // R22 of the zero quaternion reads 1 (upright!): the norm bit must catch it.
    EXPECT_EQ(EvaluatePoseGuard(in, stance), loco::kPoseGuardQuaternion);
  }
  {
    PoseGuardInput in = UprightInput(z, 2);
    in.q_xyzw = {0.0, 0.0, 0.0, 1.0 + 2e-6};  // off unit by more than the tolerance
    EXPECT_EQ(EvaluatePoseGuard(in, stance), loco::kPoseGuardQuaternion);
    in.q_xyzw = {0.0, 0.0, 0.0, 1.0 + 2e-7};  // inside it
    EXPECT_EQ(EvaluatePoseGuard(in, stance), 0U);
  }
  {
    PoseGuardInput in = UprightInput(z, 2);
    in.q_xyzw = {kNaN, 0.0, 0.0, 1.0};
    const auto mask = EvaluatePoseGuard(in, stance);
    EXPECT_TRUE(mask & loco::kPoseGuardNonFinite) << mask;
    EXPECT_TRUE(mask & loco::kPoseGuardQuaternion) << mask;
    EXPECT_TRUE(mask & loco::kPoseGuardTilt) << "a NaN fails the positive comparison";
  }
  {
    PoseGuardInput in = UprightInput(z, 2);
    in.v_base = {0.0, kInf, 0.0};
    const auto mask = EvaluatePoseGuard(in, stance);
    EXPECT_TRUE(mask & loco::kPoseGuardNonFinite) << mask;
    EXPECT_TRUE(mask & loco::kPoseGuardSpeed) << mask;
  }
  {
    const double one_nan[2] = {0.0, kNaN};
    PoseGuardInput in = UprightInput(one_nan, 2);
    const auto mask = EvaluatePoseGuard(in, stance);
    EXPECT_TRUE(mask & loco::kPoseGuardNonFinite) << mask;
    const double all_nan[2] = {kNaN, kNaN};
    in.contact_z = all_nan;
    EXPECT_TRUE(EvaluatePoseGuard(in, stance) & loco::kPoseGuardNonFinite);
    EXPECT_TRUE(EvaluatePoseGuard(in, stance) & loco::kPoseGuardSpread)
        << "NaN heights are no stance";
    // Without the stance checks the heights are not read at all.
    PoseGuardLimits upright_only;
    upright_only.cos_tilt = stance.cos_tilt;
    in.contact_z = nullptr;
    in.num_contacts = 0;
    EXPECT_EQ(EvaluatePoseGuard(in, upright_only), 0U);
  }
  {
    PoseGuardInput in = UprightInput(z, 0);
    EXPECT_EQ(EvaluatePoseGuard(in, stance), loco::kPoseGuardSpread) << "no contact to stand on";
  }
  {
    PoseGuardInput in = UprightInput(z, 2);
    in.base_valid = false;
    EXPECT_EQ(EvaluatePoseGuard(in, stance), loco::kPoseGuardBaseInvalid);
  }
}

TEST(PoseGuard, TheStanceChecksAreTheLostPlansConstantsInThePositiveForm) {
  PoseGuardLimits stance = StanceLimits(ShippedTilt());
  const double level[2] = {0.0, 0.015};  // exactly the spread limit: a stand
  PoseGuardInput in = UprightInput(level, 2);
  EXPECT_EQ(EvaluatePoseGuard(in, stance), 0U);
  const double lifted[2] = {0.0, 0.0150001};
  in.contact_z = lifted;
  EXPECT_EQ(EvaluatePoseGuard(in, stance), loco::kPoseGuardSpread);
  in.contact_z = level;
  in.v_base = {0.1, 0.0, 0.0};  // exactly the speed limit: moving
  EXPECT_EQ(EvaluatePoseGuard(in, stance), loco::kPoseGuardSpeed);
  in.v_base = {0.0999999, 0.0, 0.0};
  EXPECT_EQ(EvaluatePoseGuard(in, stance), 0U);
  in.v_base = {0.0, 0.0, 0.0};
  in.q_xyzw = RollQuaternion(45.0 * M_PI / 180.0);
  EXPECT_EQ(EvaluatePoseGuard(in, stance), loco::kPoseGuardTilt) << "the tilt joins the two";
}

TEST(PoseGuard, TheLimitAcceptsAHorizontalLimitAndATinyOne) {
  const double z[1] = {0.0};
  PoseGuardLimits lim;
  lim.cos_tilt = std::cos(M_PI / 2.0);  // a positive rounding residue, not 0
  PoseGuardInput in = UprightInput(z, 1);
  in.q_xyzw = RollQuaternion(91.0 * M_PI / 180.0);
  EXPECT_EQ(EvaluatePoseGuard(in, lim), loco::kPoseGuardTilt) << "tipped past horizontal";
  in.q_xyzw = RollQuaternion(89.0 * M_PI / 180.0);
  EXPECT_EQ(EvaluatePoseGuard(in, lim), 0U);
  lim.cos_tilt = std::cos(1e-9);
  in.q_xyzw = {0.0, 0.0, 0.0, 1.0};
  EXPECT_EQ(EvaluatePoseGuard(in, lim), 0U) << "a tiny limit accepts the exact upright";
  in.q_xyzw = RollQuaternion(1e-6);
  EXPECT_EQ(EvaluatePoseGuard(in, lim), loco::kPoseGuardTilt);
}

// ── The rig ─────────────────────────────────────────────────────────────────

class LocomotionFaultTest : public LocomotionControllerTest {
 protected:
  void BringUpWithStub(YAML::Node cfg = MpcConfig()) {
    Configure(cfg, rclcpp::NodeOptions{}, "test_locomotion_fault_mpc");
    ASSERT_TRUE(ctrl_->MpcEnabled());
    stub_ = SpawnStandingStub(*ctrl_);
    ASSERT_NE(stub_, nullptr);
    Activate();
    MakeStanding();
  }

  void ReachStand() {
    ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kStand; }));
  }

  std::uint32_t SettleTicks() const {
    return LocomotionControllerTestAccess::GuardSettleTicks(*ctrl_);
  }

  // The base pitched by `angle` about its y axis (x, y, z, w). The two soles
  // share their x, so a pitch leaves their heights equal: the stance checks
  // see a level double support and only the tilt bit moves.
  void PitchBase(double angle) {
    auto& p = state_.devices[kBody].base_pose;
    p[3] = 0.0;
    p[4] = std::sin(0.5 * angle);
    p[5] = 0.0;
    p[6] = std::cos(0.5 * angle);
  }

  void UprightBase() { PitchBase(0.0); }

  Cause LastCause() const { return static_cast<Cause>(ctrl_->LastDiag().fault_cause); }

  // The source fault: one stale tick more than the controller tolerates.
  void LatchSourceFault() {
    const int n = cfg_["fault"]["max_source_bad_ticks"].as<int>();
    for (int k = 0; k <= n; ++k) {
      StaleTick();
    }
    ASSERT_EQ(LastCause(), Cause::kSource);
  }

  // From an entry onto a refused pose: settle-time − 1 ticks waiting in damping
  // with `mask` as the verdict and nothing solved, then the guard's fault on
  // the next tick. last_out_ is the latching tick's output.
  void ExpectGuardFaultAfterSettle(std::uint32_t mask = loco::kPoseGuardTilt) {
    const std::uint32_t n = SettleTicks();
    for (std::uint32_t k = 1; k < n; ++k) {
      last_out_ = Tick();
      ASSERT_EQ(LastMode(), Mode::kWaiting) << k;
      ASSERT_FALSE(ctrl_->HasLatchedFault()) << k;
      ASSERT_EQ(GuardMask(), mask) << k;
      ASSERT_EQ(ctrl_->LastSolve(), nullptr) << k;
    }
    ExpectDamping(last_out_);
    last_out_ = Tick();  // the n-th refusal
    ASSERT_EQ(LastMode(), Mode::kFault);
    ASSERT_TRUE(ctrl_->HasLatchedFault());
    ASSERT_EQ(LastCause(), Cause::kPoseGuard);
    ASSERT_EQ(GuardMask(), mask) << "the verdict stays readable under the latch";
    ExpectDamping(last_out_);
  }

  std::uint32_t GuardMask() const { return ctrl_->LastDiag().pose_guard_mask; }

  // The body roster index of a joint, from the shipped device config.
  int BodyChannel(const std::string& joint) const {
    const std::vector<std::string> roster = ShippedDevices(cfg_).at("g1").joint_state_names;
    const auto it = std::find(roster.begin(), roster.end(), joint);
    if (it == roster.end()) {
      throw std::runtime_error("no body joint " + joint);
    }
    return static_cast<int>(it - roster.begin());
  }

  // Test-side forward kinematics of the rig's state on the controller's full
  // model: the soles' and the CoM's world positions.
  struct Feet {
    Eigen::Vector3d left;
    Eigen::Vector3d right;
    Eigen::Vector3d com;
  };

  Feet Kinematics() const {
    const pinocchio::Model& full = ctrl_->FullModel();
    const auto& idx = ctrl_->Indices();
    const auto& body = state_.devices[kBody];
    Eigen::VectorXd q = pinocchio::neutral(full);
    for (int i = 0; i < 7; ++i) {
      q(i) = body.base_pose[static_cast<std::size_t>(i)];
    }
    for (int c = 0; c < idx.body.num_channels; ++c) {
      q(7 + idx.body.joint[static_cast<std::size_t>(c)]) =
          body.positions[static_cast<std::size_t>(c)];
    }
    pinocchio::Data data(full);
    pinocchio::framesForwardKinematics(full, data, q);
    pinocchio::centerOfMass(full, data, q, false);
    return {data.oMf[full.getFrameId("left_sole")].translation(),
            data.oMf[full.getFrameId("right_sole")].translation(), data.com[0]};
  }

  // Both soles back on the floor after a joint change.
  void Relevel() {
    const Feet k = Kinematics();
    state_.devices[kBody].base_pose[2] -= std::min(k.left.z(), k.right.z());
  }

  // Bisect `set(a)` on [lo, hi] until `measure()` is within `tol` of `target`
  // (monotone in `a` over the bracket).
  void Bisect(const std::function<void(double)>& set, const std::function<double()>& measure,
              double target, double lo, double hi, double tol = 1e-6) {
    set(lo);
    const double f_lo = measure();
    set(hi);
    const double f_hi = measure();
    ASSERT_TRUE((f_lo - target) * (f_hi - target) < 0.0)
        << "target " << target << " not bracketed: " << f_lo << " .. " << f_hi;
    const bool rising = f_hi > f_lo;
    for (int it = 0; it < 80; ++it) {
      const double mid = 0.5 * (lo + hi);
      set(mid);
      const double f = measure();
      if (std::abs(f - target) < tol) {
        return;
      }
      if ((f < target) == rising) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    FAIL() << "bisection did not converge on " << target;
  }

  StandingStub* stub_{nullptr};
};

// ── Under the E-STOP ────────────────────────────────────────────────────────

TEST_F(LocomotionFaultTest, UnderTheEstopTheTickWaitsLogsAndFeedsNothingToTheMpc) {
  BringUpWithStub();
  ReachStand();
  ctrl_->TriggerEstop();
  last_out_ = Tick();  // the edge: the stream restarts, nothing is latched
  const std::uint64_t sequence = ctrl_->MpcManager().ReadState().sequence;
  const std::uint64_t tick0 = ctrl_->LastDiag().tick_count;
  for (int k = 0; k < 200; ++k) {
    state_.devices[kBody].positions[3] += 1e-4;  // the robot sags under the hold
    last_out_ = Tick();
    ASSERT_EQ(LastMode(), Mode::kWaiting) << k;
    ASSERT_TRUE(ctrl_->LastDiag().estop) << k;
    ASSERT_FALSE(ctrl_->LastDiag().fault_latched) << k;
    ASSERT_EQ(ctrl_->LastSolve(), nullptr) << k << " the QP ran under the E-STOP";
    ASSERT_FALSE(ctrl_->LastJointLog().seeded) << k << " a reference was latched";
    ASSERT_EQ(ctrl_->MpcManager().ReadState().sequence, sequence)
        << k << " a state reached the MPC";
    ASSERT_EQ(ctrl_->LastDiag().tick_count, tick0 + 1U + static_cast<std::uint64_t>(k)) << "logged";
    ASSERT_EQ(ctrl_->Source(), Source::kIdle) << k;
  }
  // The models still follow the measurement: the log's CoM moved with the sag.
  EXPECT_NE(ctrl_->LastDiag().com[2], 0.0);
  ExpectDamping(last_out_);
  EXPECT_TRUE(ctrl_->LastJointLog().model_fresh);
  // The status level reads ERROR under the hold.
  EXPECT_EQ(integrated_bringup::LocomotionStatusLevel(ctrl_->LastDiag(), true),
            diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST_F(LocomotionFaultTest, UnderTheEstopABadSourceOnlyWaitsButAnotherModelEpochLatches) {
  BringUp();
  Tick();
  ctrl_->TriggerEstop();
  Tick();
  const int n = cfg_["fault"]["max_source_bad_ticks"].as<int>();
  for (int k = 0; k <= n + 1; ++k) {  // more than the latch count: no reference, no rule
    StaleTick();
    ASSERT_EQ(LastMode(), Mode::kWaiting) << k;
    ASSERT_FALSE(ctrl_->HasLatchedFault()) << k;
  }
  EXPECT_EQ(ctrl_->LastDiag().source_bad_ticks, static_cast<std::uint32_t>(n + 2));
  // The model epoch is checked in ReadState, before the hold's branch.
  LocomotionControllerTestAccess::BumpModelEpoch(*ctrl_);
  Tick();
  EXPECT_TRUE(ctrl_->HasLatchedFault());
  EXPECT_EQ(LastCause(), Cause::kModelEpoch);
  EXPECT_EQ(LastMode(), Mode::kFault);
}

// ── The edges and the packed word ───────────────────────────────────────────

TEST_F(LocomotionFaultTest, TheEstopWordPacksTheFlagAndTheEdgeCount) {
  BringUp();
  Tick();
  const auto word0 = LocomotionControllerTestAccess::EstopWord(*ctrl_);
  EXPECT_EQ(word0 & 1U, 0U);
  ctrl_->TriggerEstop();
  const auto word1 = LocomotionControllerTestAccess::EstopWord(*ctrl_);
  EXPECT_EQ(word1, (word0 | 1U) + 2U);
  EXPECT_TRUE(ctrl_->IsEstopped());
  ctrl_->TriggerEstop();  // repeated: no edge
  EXPECT_EQ(LocomotionControllerTestAccess::EstopWord(*ctrl_), word1);
  ctrl_->ClearEstop();
  const auto word2 = LocomotionControllerTestAccess::EstopWord(*ctrl_);
  EXPECT_EQ(word2, (word1 & ~1U) + 2U);
  EXPECT_FALSE(ctrl_->IsEstopped());
  ctrl_->ClearEstop();  // repeated: no edge
  EXPECT_EQ(LocomotionControllerTestAccess::EstopWord(*ctrl_), word2);
}

TEST_F(LocomotionFaultTest, ATriggerAndAClearBetweenTwoTicksStillRestartTheReferenceOnce) {
  BringUpWithStub();
  ReachStand();
  const auto epoch = LocomotionControllerTestAccess::StateEpoch(*ctrl_);
  const double q0 = state_.devices[kBody].positions[2];
  state_.devices[kBody].positions[2] = q0 + 0.02;  // the robot moved while nobody looked
  ctrl_->TriggerEstop();
  ctrl_->ClearEstop();
  last_out_ = Tick();
  EXPECT_FALSE(ctrl_->LastDiag().estop);
  EXPECT_EQ(LocomotionControllerTestAccess::StateEpoch(*ctrl_), epoch + 1) << "one restart";
  EXPECT_EQ(LastMode(), Mode::kNominal) << "the guard passed the standing robot at once";
  EXPECT_EQ(last_out_.devices[kBody].commands[2], q0 + 0.02) << "re-latched where the robot is";
  EXPECT_EQ(ctrl_->Source(), Source::kIdle);
  EXPECT_EQ(ctrl_->LastDiag().fault_count, 0U);
}

// ── The clear edge: the guard ───────────────────────────────────────────────

TEST_F(LocomotionFaultTest, AClearLatchesTheTickItPassesTheGuardFromThatTicksMeasurement) {
  BringUp();
  Tick();
  ctrl_->TriggerEstop();
  Tick();
  state_.devices[kBody].positions[4] += 0.03;
  state_.devices[kBody].positions[7] -= 0.02;
  ctrl_->ClearEstop();
  const rtc::ControllerOutput out = Tick();
  EXPECT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(GuardMask(), 0U);
  const auto& body = out.devices[kBody];
  for (std::size_t i = 0; i < 23; ++i) {
    EXPECT_EQ(body.commands[i], state_.devices[kBody].positions[i]) << i;
  }
  EXPECT_TRUE(ctrl_->LastDiag().tau_valid);
  ExpectShippedGains(out);
}

TEST_F(LocomotionFaultTest, AClearOntoATiltedBaseWaitsForTheSettleTimeThenLatchesTheGuardFault) {
  BringUp();
  Tick();
  ctrl_->TriggerEstop();
  Tick();
  PitchBase(45.0 * M_PI / 180.0);
  ctrl_->ClearEstop();
  ASSERT_GT(SettleTicks(), 1U) << "the loop below needs a wait of at least two ticks";
  ExpectGuardFaultAfterSettle();
  EXPECT_EQ(ctrl_->Source(), Source::kFault);
  // Standing the robot up does not release the latch by itself.
  UprightBase();
  Tick();
  EXPECT_EQ(LastMode(), Mode::kFault);
  // A reset with the pose fixed: the guard passes, nominal on that tick.
  ctrl_->ResetFault();
  last_out_ = Tick();
  EXPECT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(GuardMask(), 0U);
  EXPECT_FALSE(ctrl_->HasLatchedFault());
}

TEST_F(LocomotionFaultTest, StandingUpInsideTheSettleTimeLatchesWithNoFault) {
  BringUp();
  Tick();
  ctrl_->TriggerEstop();
  Tick();
  PitchBase(45.0 * M_PI / 180.0);
  ctrl_->ClearEstop();
  for (std::uint32_t k = 0; k < SettleTicks() / 2; ++k) {
    Tick();
    ASSERT_EQ(LastMode(), Mode::kWaiting) << k;
  }
  PitchBase(5.0 * M_PI / 180.0);  // up, a little off vertical
  const rtc::ControllerOutput out = Tick();
  EXPECT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(GuardMask(), 0U);
  EXPECT_EQ(ctrl_->LastDiag().fault_count, 0U);
  EXPECT_EQ(out.devices[kBody].commands[0], state_.devices[kBody].positions[0]);
}

TEST_F(LocomotionFaultTest, TheSettleClockRestartsAtEveryEntryAndDoesNotRunUnderTheEstop) {
  BringUp();
  Tick();
  PitchBase(45.0 * M_PI / 180.0);
  ctrl_->TriggerEstop();
  const std::uint32_t n = SettleTicks();
  for (std::uint32_t k = 0; k < 2 * n; ++k) {  // tilted under the hold for twice the settle time
    Tick();
    ASSERT_EQ(LastMode(), Mode::kWaiting) << k;
    ASSERT_EQ(GuardMask(), 0U) << k << " the guard was asked under the E-STOP";
  }
  ctrl_->ClearEstop();
  ExpectGuardFaultAfterSettle();  // the full settle time again after the clear
  // A reset with the pose still bad: the clock starts over, the fault returns
  // after the settle time — HasLatchedFault reads false in between (the CM's
  // reset reply says "still present" from that flag on the first tick).
  ctrl_->ResetFault();
  ExpectGuardFaultAfterSettle();
  EXPECT_EQ(ctrl_->LastDiag().fault_count, 2U);
}

// ── The other entries ───────────────────────────────────────────────────────

TEST_F(LocomotionFaultTest, AnActivationOntoATiltedBaseWaitsThenLatchesAndOntoASlightTiltStands) {
  Configure(ShippedConfig());
  Activate();
  MakeStanding();
  PitchBase(45.0 * M_PI / 180.0);
  ExpectGuardFaultAfterSettle();
  EXPECT_EQ(ctrl_->LastDiag().tick_count, static_cast<std::uint64_t>(SettleTicks()));
  // The activation drops the latch (header) and asks the guard again.
  Deactivate();
  Activate();
  PitchBase(5.0 * M_PI / 180.0);
  Tick();
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(ctrl_->LastDiag().tick_count, 1U) << "the first tick of the activation latched";
}

TEST_F(LocomotionFaultTest, AResetOntoATiltedBaseWaitsThenLatchesTheGuardFault) {
  BringUp();
  Tick();
  LatchSourceFault();
  PitchBase(45.0 * M_PI / 180.0);
  ctrl_->ResetFault();
  ExpectGuardFaultAfterSettle();
  EXPECT_EQ(ctrl_->LastDiag().fault_count, 2U);
}

TEST_F(LocomotionFaultTest, AnEntryWithoutAValidStateWaitsWithoutAClock) {
  BringUp();
  Tick();
  LatchSourceFault();
  ASSERT_TRUE(ctrl_->HasLatchedFault());
  ctrl_->ResetFault();
  const std::uint32_t n = SettleTicks();
  for (std::uint32_t k = 0; k < 3 * n; ++k) {  // far past the settle time, never asked
    last_out_ = StaleTick();
    ASSERT_EQ(LastMode(), Mode::kWaiting) << k;
    ASSERT_FALSE(ctrl_->HasLatchedFault()) << k;
    ASSERT_EQ(GuardMask(), 0U) << k;
  }
  ExpectDamping(last_out_);
  Tick();  // the state returns: the guard passes the standing robot at once
  EXPECT_EQ(LastMode(), Mode::kNominal);
}

// ── The fault latch and the E-STOP are separate ─────────────────────────────

TEST_F(LocomotionFaultTest, AFaultLatchedBeforeTheEstopOutlivesTheClear) {
  BringUp();
  Tick();
  LatchSourceFault();
  ctrl_->TriggerEstop();
  last_out_ = Tick();
  EXPECT_TRUE(ctrl_->HasLatchedFault());
  EXPECT_EQ(LastMode(), Mode::kFault);
  ExpectDamping(last_out_);
  ctrl_->ClearEstop();
  last_out_ = Tick();
  EXPECT_TRUE(ctrl_->HasLatchedFault()) << "a clear is not a reset";
  EXPECT_EQ(LastMode(), Mode::kFault);
  EXPECT_EQ(LastCause(), Cause::kSource);
  ExpectDamping(last_out_);
  EXPECT_EQ(ctrl_->LastDiag().fault_count, 1U);
}

TEST_F(LocomotionFaultTest, AResetUnderTheEstopReleasesTheLatchAndTheGuardRunsAfterTheClear) {
  BringUp();
  Tick();
  LatchSourceFault();
  ctrl_->TriggerEstop();
  Tick();
  ctrl_->ResetFault();
  Tick();
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_EQ(LastMode(), Mode::kWaiting) << "released, but the hold keeps it waiting";
  EXPECT_EQ(GuardMask(), 0U) << "not asked under the hold";
  PitchBase(45.0 * M_PI / 180.0);
  ctrl_->ClearEstop();
  Tick();
  EXPECT_EQ(LastMode(), Mode::kWaiting);
  EXPECT_EQ(GuardMask(), loco::kPoseGuardTilt) << "asked on the first valid tick after the clear";
  UprightBase();
  Tick();
  EXPECT_EQ(LastMode(), Mode::kNominal);
}

TEST_F(LocomotionFaultTest, AnActivationUnderTheEstopWaitsAndSolvesNothingUntilTheClear) {
  BringUpWithStub();
  ReachStand();
  ctrl_->TriggerEstop();
  Tick();
  Deactivate();
  Activate();
  const std::uint64_t sequence = ctrl_->MpcManager().ReadState().sequence;
  for (int k = 0; k < 50; ++k) {
    Tick();
    ASSERT_EQ(LastMode(), Mode::kWaiting) << k;
    ASSERT_EQ(ctrl_->LastSolve(), nullptr) << k;
    ASSERT_EQ(ctrl_->MpcManager().ReadState().sequence, sequence) << k;
  }
  ctrl_->ClearEstop();
  Tick();
  EXPECT_EQ(LastMode(), Mode::kNominal);
  EXPECT_GT(ctrl_->MpcManager().ReadState().sequence, sequence);
  ReachStand();
}

TEST_F(LocomotionFaultTest, RequestsMadeWhileInactiveAreConsumedByTheActivation) {
  BringUp();
  Tick();
  LatchSourceFault();
  Deactivate();
  ctrl_->ResetFault();  // to a controller that is not ticking: dropped
  ctrl_->TriggerEstop();
  ctrl_->ClearEstop();  // an E-STOP that came and went while inactive
  Activate();
  const rtc::ControllerOutput out = Tick();
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_FALSE(ctrl_->LastDiag().estop);
  EXPECT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(ctrl_->LastDiag().tick_count, 1U);
  EXPECT_EQ(ctrl_->LastDiag().fault_count, 1U) << "no second latch, no false restart";
  EXPECT_EQ(out.devices[kBody].commands[0], state_.devices[kBody].positions[0]);
}

TEST_F(LocomotionFaultTest, FaultThenEstopThenResetThenClearEndsInTheGuardsVerdict) {
  BringUp();
  Tick();
  LatchSourceFault();
  ASSERT_TRUE(ctrl_->HasLatchedFault());
  ctrl_->TriggerEstop();
  Tick();
  ASSERT_TRUE(ctrl_->HasLatchedFault());
  ASSERT_TRUE(ctrl_->LastDiag().estop);
  ctrl_->ResetFault();
  Tick();
  ASSERT_FALSE(ctrl_->HasLatchedFault());
  ASSERT_TRUE(ctrl_->LastDiag().estop);
  ASSERT_EQ(LastMode(), Mode::kWaiting);
  ctrl_->ClearEstop();
  const rtc::ControllerOutput out = Tick();
  EXPECT_FALSE(ctrl_->LastDiag().estop);
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(LastCause(), Cause::kNone);
  EXPECT_EQ(GuardMask(), 0U);
  EXPECT_TRUE(ctrl_->LastJointLog().seeded);
  ExpectShippedGains(out);
}

// ── The lost plan: the stance checks ────────────────────────────────────────

TEST_F(LocomotionFaultTest, ALostPlanOnALevelStillStandIsReLatchedWithAPassingMask) {
  BringUpWithStub();
  ReachStand();
  stub_->fail = true;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kIdle; }));
  EXPECT_FALSE(ctrl_->HasLatchedFault());
  EXPECT_EQ(GuardMask(), 0U);
  EXPECT_EQ(LastMode(), Mode::kNominal);
}

TEST_F(LocomotionFaultTest, ALostPlanWithAFootLiftedLatchesPlanLossOnTheSpreadBit) {
  BringUpWithStub();
  ReachStand();
  // Bend one knee: that sole lifts off the other's height.
  state_.devices[kBody].positions[static_cast<std::size_t>(BodyChannel("left_knee_joint"))] += 0.5;
  Tick();
  const auto& d = ctrl_->LastDiag();
  const double spread = std::abs(d.foot_p[2] - d.foot_p[5]);
  ASSERT_GT(spread, integrated_bringup::kRelatchFootSpreadM) << "the fixture lifts no foot";
  stub_->fail = true;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->HasLatchedFault(); }));
  EXPECT_EQ(LastCause(), Cause::kPlanLoss);
  EXPECT_EQ(GuardMask(), loco::kPoseGuardSpread);
  ExpectDamping(last_out_);
}

TEST_F(LocomotionFaultTest, ALostPlanOnAMovingBaseLatchesPlanLossOnTheSpeedBit) {
  BringUpWithStub();
  ReachStand();
  state_.devices[kBody].base_twist[0] = 0.2;
  stub_->fail = true;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->HasLatchedFault(); }));
  EXPECT_EQ(LastCause(), Cause::kPlanLoss);
  EXPECT_EQ(GuardMask(), loco::kPoseGuardSpeed);
}

TEST_F(LocomotionFaultTest, ALostPlanOnAPitchedBaseLatchesPlanLossOnTheTiltBitAlone) {
  BringUpWithStub();
  ReachStand();
  // A pitch about the base y axis: the soles share their x, so their heights
  // stay equal and the base is still — only the tilt refuses the re-latch.
  PitchBase(45.0 * M_PI / 180.0);
  Tick();
  const auto& d = ctrl_->LastDiag();
  ASSERT_LE(std::abs(d.foot_p[2] - d.foot_p[5]), integrated_bringup::kRelatchFootSpreadM);
  ASSERT_LT(std::hypot(d.base_v[0], d.base_v[1], d.base_v[2]),
            integrated_bringup::kRelatchBaseSpeed);
  stub_->fail = true;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->HasLatchedFault(); }));
  EXPECT_EQ(LastCause(), Cause::kPlanLoss) << "immediate — no settle time for a lost plan";
  EXPECT_EQ(GuardMask(), loco::kPoseGuardTilt);
}

TEST_F(LocomotionFaultTest, APlanLostWhileAFootSwingsDampsAndKeepsTheHandsAndTheStream) {
  BringUpWithStub();
  ReachStand();
  state_.devices[kHand].positions[2] = 0.3;
  Tick();
  // The right foot in swing: the plan's mask says so and the robot lifts it.
  stub_->mask = 0b01U;
  ASSERT_TRUE(TickUntil([this] { return ctrl_->Source() == Source::kWalk; }));
  state_.devices[kBody].positions[static_cast<std::size_t>(BodyChannel("right_knee_joint"))] += 0.5;
  state_.devices[kHand].positions[2] = 0.9;  // the hand is pushed meanwhile
  Tick();
  const double hand_target = last_out_.devices[kHand].commands[2];
  const auto epoch = LocomotionControllerTestAccess::StateEpoch(*ctrl_);
  stub_->fail = true;  // the plan is gone mid-step
  ASSERT_TRUE(TickUntil([this] { return ctrl_->HasLatchedFault(); }));
  EXPECT_EQ(LastCause(), Cause::kPlanLoss);
  EXPECT_TRUE(GuardMask() & loco::kPoseGuardSpread);
  const std::uint64_t sequence = ctrl_->MpcManager().ReadState().sequence;
  const auto lim = ShippedDevices(cfg_).at("g1").joint_limits->max_torque;
  for (int k = 0; k < 30; ++k) {
    last_out_ = Tick();
    const auto& body = last_out_.devices[kBody];
    ASSERT_EQ(LastMode(), Mode::kFault) << k;
    for (std::size_t i = 0; i < 23; ++i) {
      ASSERT_TRUE(std::isfinite(body.feedforward[i])) << k << ' ' << i;
      ASSERT_LE(std::abs(body.feedforward[i]), lim[i]) << k << ' ' << i;
    }
    ASSERT_EQ(last_out_.devices[kHand].commands[2], hand_target) << k << " the hand hold moved";
    ASSERT_EQ(ctrl_->MpcManager().ReadState().sequence, sequence) << k;
    ASSERT_EQ(LocomotionControllerTestAccess::StateEpoch(*ctrl_), epoch) << k;
  }
  ExpectDamping(last_out_);
  // The reset is the one thing that moves the stream on.
  state_.devices[kBody].positions[static_cast<std::size_t>(BodyChannel("right_knee_joint"))] -= 0.5;
  ctrl_->ResetFault();
  Tick();
  EXPECT_EQ(LocomotionControllerTestAccess::StateEpoch(*ctrl_), epoch + 1);
}

// ── The damping law against its oracle ──────────────────────────────────────

TEST_F(LocomotionFaultTest, TheDampingLawMatchesItsOracleBitForBitOnEveryLaneShape) {
  BringUp();
  Tick();
  ctrl_->TriggerEstop();
  auto& dev = state_.devices[kBody];
  const auto lim = devices_.at("g1").joint_limits->max_torque;
  // The rig's oracle (ExpectDamping) plus the goal / trajectory lanes.
  const auto expect_oracle = [&](const rtc::ControllerOutput& out) {
    ExpectDamping(out);
    const auto& body = out.devices[kBody];
    for (std::size_t i = 0; i < 23; ++i) {
      ASSERT_EQ(body.goal_positions[i], dev.positions[i]) << i;
      ASSERT_EQ(body.target_positions[i], dev.positions[i]) << i;
      ASSERT_EQ(body.trajectory_positions[i], dev.positions[i]) << i;
      ASSERT_EQ(body.trajectory_velocities[i], 0.0) << i;
    }
    ASSERT_EQ(body.goal_type, rtc::GoalType::kJoint);
    ASSERT_EQ(out.command_type, rtc::CommandType::kPdFeedforward);
  };
  // Finite velocities, one of them large enough to clamp.
  for (std::size_t i = 0; i < 23; ++i) {
    dev.velocities[i] = 0.3 * static_cast<double>(i) - 2.0;
  }
  dev.velocities[5] = 1e6;
  {
    const rtc::ControllerOutput out = Tick();
    expect_oracle(out);
    EXPECT_EQ(out.devices[kBody].feedforward[5], -lim[5]) << "clamped to the torque limit";
  }
  // One NaN channel among finite ones: that channel reads 0 → −0.0 feedforward.
  dev.velocities[7] = kNaN;
  {
    const rtc::ControllerOutput out = Tick();
    expect_oracle(out);
    EXPECT_TRUE(std::signbit(out.devices[kBody].feedforward[7]));
    EXPECT_EQ(out.devices[kBody].feedforward[7], 0.0);
  }
  // The velocity lane unreadable: every channel −0.0.
  dev.velocity_hole_mask = 1U << 3;
  {
    const rtc::ControllerOutput out = Tick();
    expect_oracle(out);
    for (std::size_t i = 0; i < 23; ++i) {
      EXPECT_TRUE(std::signbit(out.devices[kBody].feedforward[i])) << i;
    }
  }
  dev.velocity_hole_mask = 0;
  // The device unreadable: nothing honest to command.
  dev.valid = false;
  {
    const rtc::ControllerOutput out = Tick();
    EXPECT_EQ(out.devices[kBody].num_channels, 0);
    EXPECT_TRUE(out.devices[kBody].has_pd_gains);
  }
  dev.valid = true;
}

TEST_F(LocomotionFaultTest, WithoutATorqueLimitTheDampingIsUnclamped) {
  // A roster without joint_limits.max_torque: the clamp is ±inf.
  YAML::Node cfg = ShippedConfig();
  auto devices = ShippedDevices(cfg);
  devices.at("g1").joint_limits.reset();
  ConfigureWithDevices(cfg, devices, rclcpp::NodeOptions{}, "test_locomotion_fault_nolim");
  Activate();
  MakeStanding();
  Tick();
  ctrl_->TriggerEstop();
  auto& dev = state_.devices[kBody];
  const auto kd = cfg_["fault"]["damping_kd"].as<std::vector<double>>();
  dev.velocities[5] = 1e6;
  dev.velocities[6] = -1e6;
  const rtc::ControllerOutput out = Tick();
  EXPECT_EQ(out.devices[kBody].feedforward[5], -kd[5] * 1e6);
  EXPECT_EQ(out.devices[kBody].feedforward[6], kd[6] * 1e6);
  EXPECT_TRUE(std::isfinite(out.devices[kBody].feedforward[5]));
}

// ── The hands ───────────────────────────────────────────────────────────────

TEST_F(LocomotionFaultTest, AHandIsHeldFromItsFirstReadableTickWhileTheBodyWaits) {
  Configure(ShippedConfig());
  Activate();
  MakeStanding();
  PitchBase(45.0 * M_PI / 180.0);  // the body waits on the guard
  state_.devices[kHand].positions[2] = 0.3;
  rtc::ControllerOutput out = Tick();
  ASSERT_EQ(LastMode(), Mode::kWaiting);
  ASSERT_EQ(out.devices[kHand].num_channels, 10);
  EXPECT_EQ(out.devices[kHand].commands[2], 0.3) << "held on the first readable tick";
  std::array<double, rtc::kMaxDeviceChannels> hold = out.devices[kHand].commands;
  for (int k = 0; k < 20; ++k) {
    state_.devices[kHand].positions[2] += 0.05;  // pushed while waiting
    out = Tick();
    for (std::size_t c = 0; c < 10; ++c) {
      ASSERT_EQ(std::bit_cast<std::uint64_t>(out.devices[kHand].commands[c]),
                std::bit_cast<std::uint64_t>(hold[c]))
          << k << ' ' << c;
    }
  }
  // The guard passes: the body latches, the hand does not jump to where it was pushed.
  UprightBase();
  out = Tick();
  ASSERT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(out.devices[kHand].commands[2], 0.3);
  // The E-STOP's edges re-latch the hand where it is (the robot was not ours in between).
  ctrl_->TriggerEstop();
  state_.devices[kHand].positions[2] = 0.7;
  out = Tick();
  EXPECT_EQ(out.devices[kHand].commands[2], 0.7);
  ctrl_->ClearEstop();
  state_.devices[kHand].positions[2] = 0.8;
  out = Tick();
  EXPECT_EQ(out.devices[kHand].commands[2], 0.8);
  state_.devices[kHand].positions[2] = 0.9;
  out = Tick();
  EXPECT_EQ(out.devices[kHand].commands[2], 0.8) << "and held there";
}

TEST_F(LocomotionFaultTest, AHandHoldDoesNotMoveThroughTheGuardFaultAndItsReset) {
  Configure(ShippedConfig());
  Activate();
  MakeStanding();
  PitchBase(45.0 * M_PI / 180.0);
  state_.devices[kHand].positions[4] = 0.25;
  Tick();
  const std::uint32_t n = SettleTicks();
  for (std::uint32_t k = 0; k < n + 5; ++k) {  // through the latch of kPoseGuard
    state_.devices[kHand].positions[4] = 0.25 + 0.001 * static_cast<double>(k);
    last_out_ = Tick();
    ASSERT_EQ(last_out_.devices[kHand].commands[4], 0.25) << k;
  }
  ASSERT_EQ(LastCause(), Cause::kPoseGuard);
  ctrl_->ResetFault();  // an entry: the hands re-latch where they are
  UprightBase();
  last_out_ = Tick();
  EXPECT_EQ(LastMode(), Mode::kNominal);
  EXPECT_EQ(last_out_.devices[kHand].commands[4], state_.devices[kHand].positions[4]);
}

// ── The lever rule ──────────────────────────────────────────────────────────

// The CoM moved forward of the foot line by leaning both hips: the lever rule
// splits the weight by projection (here half and half: the feet share their x)
// and the centres of pressure carry the forward offset — within 0.1 m of each
// contact's origin, saturated beyond.
TEST_F(LocomotionFaultTest, TheLeverRuleBoundsTheCentreOfPressureToTheSole) {
  BringUp();
  const int left = BodyChannel("left_hip_pitch_joint");
  const int right = BodyChannel("right_hip_pitch_joint");
  const std::vector<double> stand = StandPose();
  const auto lean = [&](double a) {
    state_.devices[kBody].positions[static_cast<std::size_t>(left)] = stand[left] + a;
    state_.devices[kBody].positions[static_cast<std::size_t>(right)] = stand[right] + a;
    Relevel();
  };
  const auto offset = [&]() {
    const Feet k = Kinematics();
    return k.com.x() - 0.5 * (k.left.x() + k.right.x());
  };
  const double weight =
      pinocchio::computeTotalMass(ctrl_->FullModel()) * ctrl_->FullModel().gravity.linear().norm();
  for (const double target : {0.05, 0.10, 0.20}) {
    SCOPED_TRACE(target);
    Bisect(lean, offset, target, 0.0, 1.2, 1e-7);
    Deactivate();  // a fresh latch from this pose
    Activate();
    last_out_ = Tick();
    const auto& d = ctrl_->LastDiag();
    const double realised = d.com[0] - 0.5 * (d.foot_p[0] + d.foot_p[3]);
    ASSERT_NEAR(realised, target, 1e-6) << "the fixture did not realise the CoM offset";
    const Eigen::VectorXd& l = ctrl_->Reference().lambda_des;
    ASSERT_EQ(l.size(), 12);
    double fz_sum = 0.0;
    for (int c = 0; c < 2; ++c) {
      const double fz = l(6 * c + 2);
      const double tx = l(6 * c + 3);
      const double ty = l(6 * c + 4);
      EXPECT_TRUE(std::isfinite(fz) && std::isfinite(tx) && std::isfinite(ty)) << c;
      EXPECT_GT(fz, 0.0) << c;
      EXPECT_LE(std::abs(tx), 0.1 * fz + 1e-9) << c;
      EXPECT_LE(std::abs(ty), 0.1 * fz + 1e-9) << c;
      EXPECT_EQ(l(6 * c + 5), 0.0) << c;
      if (target > 0.1) {
        EXPECT_EQ(std::abs(ty), 0.1 * fz) << c << " the forward offset saturates its axis";
      } else {
        EXPECT_NEAR(std::abs(ty), target * fz, 1e-6 * fz) << c;
      }
      fz_sum += fz;
    }
    EXPECT_NEAR(fz_sum, weight, 1e-9 * weight);
    EXPECT_EQ(LastMode(), Mode::kNominal);
    for (int k = 0; k < 50; ++k) {
      Tick();
      ASSERT_TRUE(ctrl_->LastDiag().tau_valid) << target << ' ' << k;
      ASSERT_EQ(ctrl_->LastDiag().health, 0U) << target << ' ' << k;
    }
  }
}

// One foot moved in toward the other by its hip roll: feet closer than 5 cm
// share the weight half and half (the projection onto a near-zero line is
// noise); farther apart, the projection — which, with the CoM near the fixed
// foot, is far from half.
TEST_F(LocomotionFaultTest, TheLeverRuleSplitsHalfAndHalfBelowFiveCentimetresOfSeparation) {
  BringUp();
  const int roll = BodyChannel("right_hip_roll_joint");
  const std::vector<double> stand = StandPose();
  const auto move = [&](double a) {
    state_.devices[kBody].positions[static_cast<std::size_t>(roll)] = stand[roll] + a;
    Relevel();
  };
  const auto separation = [&]() {
    const Feet k = Kinematics();
    return (k.right.head<2>() - k.left.head<2>()).norm();
  };
  const double weight =
      pinocchio::computeTotalMass(ctrl_->FullModel()) * ctrl_->FullModel().gravity.linear().norm();
  move(0.0);
  const double apart = separation();
  ASSERT_GT(apart, 0.1) << "the stand pose has the feet apart";
  // The roll that brings the right foot over the left: a coarse scan for the
  // closest approach (the separation is V-shaped in the roll), then the
  // bisection runs on the monotone side between the stand and that roll.
  const double sign = (move(0.2), separation()) < apart ? 1.0 : -1.0;
  double closest = 0.0;
  double closest_sep = apart;
  for (int k = 1; k <= 600; ++k) {
    const double a = sign * 0.001 * static_cast<double>(k);
    move(a);
    if (separation() < closest_sep) {
      closest_sep = separation();
      closest = a;
    }
  }
  // One roll joint sweeps a circle: the soles' x offset keeps the closest
  // approach a couple of centimetres apart, which is still well inside the
  // half-and-half band (a 1 mm case would need a second joint).
  ASSERT_LT(closest_sep, 0.03) << "the right foot never comes near the left";
  for (const double target : {closest_sep, 0.049, 0.051}) {
    SCOPED_TRACE(target);
    if (target == closest_sep) {
      move(closest);
    } else {
      Bisect(move, separation, target, 0.0, closest, 1e-8);
    }
    Deactivate();
    Activate();
    Tick();
    const Feet k = Kinematics();
    const Eigen::Vector2d p0 = k.left.head<2>();
    const Eigen::Vector2d p1 = k.right.head<2>();
    const Eigen::Vector2d d = p1 - p0;
    const Eigen::VectorXd& l = ctrl_->Reference().lambda_des;
    if (target < 0.05) {
      EXPECT_EQ(l(2), 0.5 * weight);
      EXPECT_EQ(l(8), 0.5 * weight);
    } else {
      const double t = std::clamp((k.com.head<2>() - p0).dot(d) / d.squaredNorm(), 0.0, 1.0);
      ASSERT_GT(std::abs(t - 0.5), 0.1) << "the projection is not telling here: t " << t;
      EXPECT_NEAR(l(2), (1.0 - t) * weight, 1e-9 * weight);
      EXPECT_NEAR(l(8), t * weight, 1e-9 * weight);
    }
    EXPECT_NEAR(l(2) + l(8), weight, 1e-9 * weight);
    for (int c = 0; c < 2; ++c) {
      EXPECT_LE(std::abs(l(6 * c + 3)), 0.1 * l(6 * c + 2) + 1e-9) << c;
      EXPECT_LE(std::abs(l(6 * c + 4)), 0.1 * l(6 * c + 2) + 1e-9) << c;
    }
  }
}

TEST_F(LocomotionFaultTest, PointContactsShareTheWeightWithoutAMoment) {
  {
    YAML::Node cfg = YAML::Clone(ShippedConfig());
    for (auto c : cfg["model"]["contacts"]) {
      c["dim"] = 3;
    }
    BringUp(cfg);
    Tick();
    const Eigen::VectorXd& l = ctrl_->Reference().lambda_des;
    ASSERT_GE(l.size(), 6);
    const double w = pinocchio::computeTotalMass(ctrl_->FullModel()) *
                     ctrl_->FullModel().gravity.linear().norm();
    EXPECT_NEAR(l(2) + l(5), w, 1e-9 * w);
    EXPECT_EQ(l(0), 0.0);
    EXPECT_EQ(l(1), 0.0);
    EXPECT_EQ(l(3), 0.0);
    EXPECT_EQ(l(4), 0.0);
    for (Eigen::Index i = 6; i < l.size(); ++i) {
      EXPECT_EQ(l(i), 0.0) << i;
    }
  }
}

// ── Configuration ───────────────────────────────────────────────────────────

TEST(LocomotionFaultConfig, RefusesEveryMalformedPoseGuard) {
  const auto refused = [](const std::function<void(YAML::Node&)>& spoil, const char* what) {
    ExpectRefused(ShippedConfig(), spoil, what);
  };
  refused([](YAML::Node& c) { c["fault"].remove("pose_guard"); }, "missing pose_guard");
  refused([](YAML::Node& c) { c["fault"]["pose_guard"].remove("base_tilt_rad"); },
          "missing base_tilt_rad");
  refused([](YAML::Node& c) { c["fault"]["pose_guard"]["base_speed"] = 0.1; }, "unknown key");
  refused([](YAML::Node& c) { c["fault"]["pose_guard"]["base_tilt_rad"] = 0.0; }, "zero");
  refused([](YAML::Node& c) { c["fault"]["pose_guard"]["base_tilt_rad"] = -0.1; }, "negative");
  refused([](YAML::Node& c) { c["fault"]["pose_guard"]["base_tilt_rad"] = 1.5708; }, "> pi/2");
  refused([](YAML::Node& c) { c["fault"]["pose_guard"]["base_tilt_rad"] = ".nan"; }, "NaN");
  refused([](YAML::Node& c) { c["fault"]["pose_guard"]["base_tilt_rad"] = ".inf"; }, "inf");
  refused([](YAML::Node& c) { c["fault"]["pose_guard"]["base_tilt_rad"] = "wide"; }, "a word");
  refused([](YAML::Node& c) { c["fault"]["pose_guard"] = 0.5; }, "a scalar, not a map");
  {
    YAML::Node cfg = YAML::Clone(ShippedConfig());
    cfg["fault"]["pose_guard"]["base_tilt_rad"] = 1.5707963267948966;  // pi/2 exactly: accepted
    EXPECT_NO_THROW(static_cast<void>(ParseLocomotionControllerConfig(cfg)));
    cfg["fault"]["pose_guard"]["base_tilt_rad"] = 1e-9;
    EXPECT_NO_THROW(static_cast<void>(ParseLocomotionControllerConfig(cfg)));
  }
}

TEST(LocomotionFaultConfig, BothShippedProfilesPinThirtyDegrees) {
  for (const char* profile : {"g1_loco", "b7_loco"}) {
    SCOPED_TRACE(profile);
    const auto parsed = ParseLocomotionControllerConfig(fx::ShippedControllerNode(profile, kKey));
    EXPECT_EQ(parsed.pose_guard_base_tilt_rad, 0.5236);
  }
}

// The CM's E-STOP hold sends no gains, so the simulator keeps the LAST gains
// on its wire — under this controller its own low ones, under which a held
// pose settles slowly (documented). This tripwire fires when the two gain sets
// stop being ordered that way: then that paragraph is wrong.
TEST(LocomotionFaultConfig, TheLocomotionGainsStayBelowDemoJointsOnEveryJoint) {
  const YAML::Node loco = fx::ShippedControllerNode("g1_loco", kKey);
  const YAML::Node joint = fx::ShippedControllerNode("g1_loco", "demo_joint_controller");
  const auto loco_kp = loco["pd_feedforward"]["kp"].as<std::vector<double>>();
  const auto loco_kd = loco["pd_feedforward"]["kd"].as<std::vector<double>>();
  const auto joint_kp = joint["pd_feedforward"]["kp"].as<std::vector<double>>();
  const auto joint_kd = joint["pd_feedforward"]["kd"].as<std::vector<double>>();
  ASSERT_EQ(loco_kp.size(), joint_kp.size());
  ASSERT_EQ(loco_kd.size(), joint_kd.size());
  for (std::size_t i = 0; i < loco_kp.size(); ++i) {
    EXPECT_LT(loco_kp[i], joint_kp[i]) << i;
    EXPECT_LT(loco_kd[i], joint_kd[i]) << i;
  }
}

}  // namespace
