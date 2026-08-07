// ── DemoWbcController launch-profile activation gate (issue #350) ───────────
//
// The MPC cores are reserved by the layout manifest on every tier, but the MPC
// thread itself is conditional: `mpc.enabled` lives in the controller YAML and
// SpawnMpcThreadIfNeeded runs in on_activate. #350 lets a launch opt out of the
// reservation, which shrinks the cset shield and hands those cores back to the
// system cpuset.
//
// That creates one dangerous combination — the shield was built for MPC-off but
// the controller's config still says mpc.enabled: true — and it is reachable in
// normal use: the robot launches pass `enable_mpc` to the shield without also
// overriding the controller parameter, and a controller switch can bring in an
// MPC-enabled config mid-session. Activating there would put SCHED_FIFO 60 on a
// core arbitrary user work now shares.
//
// Two properties are pinned here:
//   * the unknown-profile default reserves rather than refuses, so a typo
//     cannot take out activation on a box whose shield still holds the cores;
//   * the refusal happens before on_activate's first side effect. The
//     controller manager leaves the previous controller active on a failed
//     on_activate and never calls on_deactivate for the failed target, so
//     anything ordered before the FAILURE stays half-activated for good.

#include "integrated_bringup/controllers/demo_wbc_controller.hpp"

#include <lifecycle_msgs/msg/state.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string_view>

namespace {

using integrated_bringup::DemoWbcController;
using rtc::RTControllerInterface;

// on_activate only reads `prev` to forward it; the transition it names does not
// matter to the gate.
rclcpp_lifecycle::State InactiveState() {
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
}

// Drive the real config path so `mpc_enabled_` is set the way a deployment sets
// it — from YAML, through ConfigureMpc. That call sits *after* LoadConfig's
// model-availability guard, so a controller built without a URDF returns early
// and never reads `mpc.enabled` at all; the UR5e fixture exists to get past that
// guard (its base_link -> tool0 chain is what LoadConfig's fallback branch asks
// for), not because anything here is robot-specific.
//
// The shipped config is used verbatim with one key flipped. Hand-building a
// minimal YAML does not work: LoadConfig rejects a config missing any of
// arm_dof / integration / estop / ... long before it reads `mpc.enabled`, so a
// synthetic node would have to carry a copy of the real schema and would then
// rot against it.
//
// The controller is neither copyable nor movable, so it is configured in place.
void Configure(DemoWbcController& ctrl, bool mpc_enabled, std::string_view profile) {
  YAML::Node cfg = YAML::LoadFile(RTC_WBC_CONFIG_PATH)["demo_wbc_controller"];
  // The tsid block names hand contact frames (thumb_tip_link, ...) that only
  // exist in the combined arm+hand model. Dropping it keeps the fixture on the
  // arm-only URDF; TSID has no bearing on which threads this run may spawn.
  cfg.remove("tsid");
  cfg["mpc"]["enabled"] = mpc_enabled;
  ctrl.LoadConfig(cfg);
  ctrl.SetLayoutProfile(profile);
}

// A controller whose LoadConfig can reach ConfigureMpc.
std::unique_ptr<DemoWbcController> MakeController() {
  return std::make_unique<DemoWbcController>(RTC_UR5E_URDF_PATH);
}

TEST(WbcLayoutProfileGate, OnlyTheDeclaredOptOutDropsMpc) {
  EXPECT_TRUE(DemoWbcController::LayoutProfileDropsMpc(DemoWbcController::kMpcOffLayoutProfile));

  // Everything else reserves. The empty string is the realistic one — it is
  // what a node reports when no launch set the parameter at all.
  EXPECT_FALSE(DemoWbcController::LayoutProfileDropsMpc(DemoWbcController::kDefaultLayoutProfile));
  EXPECT_FALSE(DemoWbcController::LayoutProfileDropsMpc(""));
  EXPECT_FALSE(DemoWbcController::LayoutProfileDropsMpc("mpc_offf"));
  EXPECT_FALSE(DemoWbcController::LayoutProfileDropsMpc("MPC_OFF"));
  EXPECT_FALSE(DemoWbcController::LayoutProfileDropsMpc("off"));
}

TEST(WbcLayoutProfileGate, RefusesMpcEnabledConfigUnderTheOptOutProfile) {
  auto owned = MakeController();
  auto& ctrl = *owned;
  Configure(ctrl, /*mpc_enabled=*/true, DemoWbcController::kMpcOffLayoutProfile);

  EXPECT_EQ(ctrl.on_activate(InactiveState()), RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(WbcLayoutProfileGate, TsidOnlyConfigActivatesUnderTheOptOutProfile) {
  // The profile refuses a *configuration*, not a controller type (decision
  // D13). DemoWbcController is a WBC with optional MPC, and its TSID-only form
  // is the one an MPC-off launch is meant to run.
  auto owned = MakeController();
  auto& ctrl = *owned;
  Configure(ctrl, /*mpc_enabled=*/false, DemoWbcController::kMpcOffLayoutProfile);

  EXPECT_NE(ctrl.on_activate(InactiveState()), RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(WbcLayoutProfileGate, MpcEnabledConfigActivatesUnderTheDefaultProfile) {
  // The bit-invariance half: with the reservation in place nothing about
  // activation changed, so a launch that never mentions enable_mpc behaves
  // exactly as it did before #350.
  auto owned = MakeController();
  auto& ctrl = *owned;
  Configure(ctrl, /*mpc_enabled=*/true, DemoWbcController::kDefaultLayoutProfile);

  EXPECT_NE(ctrl.on_activate(InactiveState()), RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(WbcLayoutProfileGate, ProfileIsNotLatchedByARefusal) {
  // A refused activation must leave the controller reusable: the operator's fix
  // is to relaunch, and if the refusal stuck the second launch would fail too
  // for a reason that no longer exists.
  auto owned = MakeController();
  auto& ctrl = *owned;
  Configure(ctrl, /*mpc_enabled=*/true, DemoWbcController::kMpcOffLayoutProfile);
  ASSERT_EQ(ctrl.on_activate(InactiveState()), RTControllerInterface::CallbackReturn::FAILURE);

  ctrl.SetLayoutProfile(DemoWbcController::kDefaultLayoutProfile);
  EXPECT_NE(ctrl.on_activate(InactiveState()), RTControllerInterface::CallbackReturn::FAILURE);
}

}  // namespace
