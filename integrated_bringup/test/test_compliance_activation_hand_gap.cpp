// ── Activation vs. a standing hand gap (#504) ────────────────────────────────
//
// On a hand driven by POSITION commands and no grasp servo — which is what
// `grasp_controller_type: "none"` ships on both p1b and iiwa7_leap — the grip
// force IS the gap between the commanded finger pose and the measured one. The
// fingers are commanded THROUGH the object; what the object refuses to let them
// reach is what it feels.
//
// `ResetTargetInitialization()` re-seeds the hand target from the MEASURED pose
// on every activation, so that gap collapses to zero the moment a controller is
// activated. On the arm that is the whole point (an activation must not inherit
// the previous session's goal). On the hand it silently drops whatever was being
// held: 2026-09-04 on p1b hardware the fingertip forces fell from ~1.5 N to
// 0.3-0.9 N across a joint -> compliance switch, the pull estimator went invalid
// for the entire activation, and the symptom sixteen seconds later was "the arm
// does not move". The sim reproduction measured the same collapse: sum |f| 18.5
// N -> 0.000 N, both fingertips under `contact_on_threshold` within 238 ms.
//
// #504 decided direction (a): the contract is NOT changed. `on_activate` keeps
// re-seeding the hand, and the supported procedure is instead
//
//     activate compliance FIRST, then send the hand target that closes on the
//     object
//
// — a path that exists in the shipped code already (the hand target arrives on
// device 1 of the same mailbox as the arm's, and the hand control law is the one
// DemoJointController uses). So what has to be pinned is not a fix but the
// PROPERTY THE PROCEDURE RESTS ON, in both directions at once:
//
//     supported order   (activate -> target)  -> the gap stands
//     unsupported order (target -> activate)  -> the gap collapses
//
// BOTH IN ONE TEST, deliberately. There is no pre-fix revision to produce a red,
// so the second half is the positive control for the first: without it, "the gap
// stands" could pass on a controller that never re-seeds at all, and the whole
// reason the procedure exists would go unobserved. They share one target, one
// measured pose and one helper so the two cannot drift into measuring different
// things.
//
// NOT THE SAME AS test_activation_generation_gate.cpp (#196 §3). That one pins
// that a target which arrived while INACTIVE is DROPPED — a target that was
// never consumed. This one is about a target that WAS consumed, is standing as a
// commanded gap, and is erased by the re-seed. The #196 gate would be satisfied
// by a controller that kept the gap; the property here is the opposite one.
//
// iiwa7_leap, not p1b, because it is the profile the URDF fixture models and
// DemoComplianceController will not run its non-E-STOP path model-less. The
// re-seed is robot-agnostic — it is `hand_target_initialized_` and the device-1
// hold seed, neither of which knows how many fingers it has.
// ─────────────────────────────────────────────────────────────────────────────
#include "iiwa7_leap_test_fixture.hpp"
#include "integrated_bringup/controllers/demo_compliance_controller.hpp"
#include "shipped_config_test_fixture.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>

#ifndef RTC_DEMO_SHARED_CONFIG_DIR
#error "RTC_DEMO_SHARED_CONFIG_DIR must be defined by CMake"
#endif

namespace {

using integrated_bringup::DemoComplianceController;
using rtc::ControllerOutput;
using rtc::ControllerState;

using integrated_bringup::testfx::kArmDof;
using integrated_bringup::testfx::kArmHome;
using integrated_bringup::testfx::kDt;
using integrated_bringup::testfx::kHandDof;
using integrated_bringup::testfx::MakeIiwa7LeapDeviceConfigs;
using integrated_bringup::testfx::MakeIiwa7LeapState;
using integrated_bringup::testfx::MergeShippedShared;
using integrated_bringup::testfx::SharedIiwa7LeapBuilder;
using integrated_bringup::testfx::SharedIiwa7LeapModelConfig;
using integrated_bringup::testfx::ShippedControllerNode;

const rclcpp_lifecycle::State kInactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                        "inactive");

// Where the fingers are. Every joint a different value: a measured pose of all
// zeros (the fixture default) would let a command that ignored the measurement
// entirely still read back as "seeded at the measurement".
constexpr double kFingerRestBase = 0.11;
constexpr double kFingerRestStep = 0.017;

// How far past the measurement the target closes. Signs and magnitudes are the
// shape of the p1b hardware table in #504 (+0.0025 / -0.0393 / -0.0917): a
// single uniform delta would pass on a controller that mixed up joint order or
// dropped the sign, which is exactly the kind of drift the gap is made of.
constexpr std::array<double, 4> kCloseDelta = {0.0025, -0.0393, -0.0917, 0.0461};

// > the hand trajectory duration for these deltas, so the "stands" half is the
// converged gap and not a ramp caught in the middle.
constexpr int kSettleTicks = 600;  // 600 x 2 ms = 1.2 s
constexpr double kTolerance = 1e-6;

double FingerRest(std::size_t i) {
  return kFingerRestBase + kFingerRestStep * static_cast<double>(i);
}

double FingerTarget(std::size_t i) {
  return FingerRest(i) + kCloseDelta[i % kCloseDelta.size()];
}

// The fixture state with the hand parked at the rest pose above.
ControllerState MakeGraspingState() {
  ControllerState state = MakeIiwa7LeapState();
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandDof); ++i) {
    state.devices[1].positions[i] = FingerRest(i);
  }
  return state;
}

std::unique_ptr<DemoComplianceController> BringUp() {
  const std::string profile = "iiwa7_leap";
  YAML::Node cfg = ShippedControllerNode(profile, "demo_compliance_controller");
  MergeShippedShared(cfg, profile);

  auto ctrl = std::make_unique<DemoComplianceController>("", DemoComplianceController::Gains{});
  ctrl->SetSystemModelConfig(SharedIiwa7LeapModelConfig());
  ctrl->SetSharedModelBuilder(SharedIiwa7LeapBuilder());
  ctrl->SetControlRate(1.0 / kDt);
  ctrl->LoadConfig(cfg);
  ctrl->SetDeviceNameConfigs(MakeIiwa7LeapDeviceConfigs());
  return ctrl;
}

// commanded - measured, per finger. This is the quantity the object feels.
using HandGap = std::array<double, static_cast<std::size_t>(kHandDof)>;

HandGap GapOf(const ControllerOutput& out, const ControllerState& state) {
  HandGap gap{};
  for (std::size_t i = 0; i < static_cast<std::size_t>(kHandDof); ++i) {
    gap[i] = out.devices[1].commands[i] - state.devices[1].positions[i];
  }
  return gap;
}

double MaxAbs(const HandGap& gap) {
  double m = 0.0;
  for (const double g : gap) {
    m = std::max(m, std::abs(g));
  }
  return m;
}

// Tick the controller `n` times from `iteration`, returning the last output.
ControllerOutput TickFor(DemoComplianceController& ctrl, ControllerState& state, int n,
                     std::uint64_t& iteration) {
  ControllerOutput out{};
  for (int k = 0; k < n; ++k) {
    state.iteration = ++iteration;
    state.t_relative_s = static_cast<double>(iteration) * kDt;
    out = ctrl.Compute(state);
  }
  return out;
}

void SendCloseTarget(DemoComplianceController& ctrl) {
  std::array<double, static_cast<std::size_t>(kHandDof)> target{};
  for (std::size_t i = 0; i < target.size(); ++i) {
    target[i] = FingerTarget(i);
  }
  ctrl.SetDeviceTarget(1, std::span<const double>(target));
}

}  // namespace

// The whole of #504's acceptance in one place: the same close command, applied
// on either side of the activation boundary, is a grasp or is nothing.
TEST(ComplianceActivationHandGap, ReSeedDecidesWhetherTheCloseCommandSurvives) {
  // ── Supported order: activate FIRST, then close ───────────────────────────
  HandGap supported{};
  {
    auto ctrl = BringUp();
    ControllerState state = MakeGraspingState();
    std::uint64_t iteration = 0;

    ASSERT_EQ(ctrl->on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);

    // Two ticks: the device-1 hold seed is deferred until the hand is first
    // readable, so the first tick may still be commanding the zero-init.
    const ControllerOutput seeded = TickFor(*ctrl, state, 2, iteration);
    ASSERT_GT(seeded.num_devices, 1);
    EXPECT_NEAR(MaxAbs(GapOf(seeded, state)), 0.0, kTolerance)
        << "activation left a gap standing — the hold seed did not come from the measurement, "
           "and the rest of this test would be measuring that instead";

    SendCloseTarget(*ctrl);
    const ControllerOutput closed = TickFor(*ctrl, state, kSettleTicks, iteration);
    supported = GapOf(closed, state);
  }

  // ── Unsupported order: close FIRST, then activate ─────────────────────────
  HandGap unsupported{};
  double before_reactivation = 0.0;
  {
    auto ctrl = BringUp();
    ControllerState state = MakeGraspingState();
    std::uint64_t iteration = 0;

    ASSERT_EQ(ctrl->on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
    (void)TickFor(*ctrl, state, 2, iteration);

    SendCloseTarget(*ctrl);
    const ControllerOutput closed = TickFor(*ctrl, state, kSettleTicks, iteration);
    before_reactivation = MaxAbs(GapOf(closed, state));

    // The switch. On hardware the gap was built by DemoJointController and this
    // is compliance coming up; here one instance stands for both, because the
    // re-seed is per-activation and reads only the measurement — it cannot tell
    // which controller commanded the pose it is replacing.
    ASSERT_EQ(ctrl->on_deactivate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);
    ASSERT_EQ(ctrl->on_activate(kInactive), rtc::RTControllerInterface::CallbackReturn::SUCCESS);

    const ControllerOutput after = TickFor(*ctrl, state, 2, iteration);
    unsupported = GapOf(after, state);
  }

  // Positive control first: if the close command never opened a gap in the
  // second sequence either, "the unsupported order collapses it" is vacuous.
  EXPECT_GT(before_reactivation, 0.01)
      << "the close command never opened a gap, so the collapse below proves nothing";

  // Supported order: the gap stands, joint by joint and with the commanded sign.
  for (std::size_t i = 0; i < supported.size(); ++i) {
    EXPECT_NEAR(supported[i], kCloseDelta[i % kCloseDelta.size()], 1e-4)
        << "finger " << i
        << ": target sent after activation did not hold its gap — the grip this procedure "
           "depends on is not there";
  }

  // Unsupported order: the same command, erased.
  EXPECT_NEAR(MaxAbs(unsupported), 0.0, kTolerance)
      << "activation did NOT collapse a standing hand gap. That is a better world than the one "
         "#504 documents, but the shipped procedure (activate compliance, THEN close) was chosen "
         "because it does — re-open #504 before relaxing this.";

  // And the two orders must actually differ, which is the sentence #504's
  // procedure is: neither assertion above catches a fixture that made both
  // sequences identical.
  EXPECT_GT(MaxAbs(supported), MaxAbs(unsupported) + 0.01)
      << "both orders produced the same hand gap — the activation boundary is not being crossed";
}
