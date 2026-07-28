// ── CombinedModelCache: the F5 gate on the shared model scatter (#236 S7b) ───
//
// `ExtractFullState` is where three controllers turn a device reading into the
// Pinocchio configuration every FK / Jacobian / QP consumer runs on, and where
// the #265 audit found the claim that motivates decision B: `min(arm_dof, nc0)`
// prevents the out-of-range read and prevents nothing else. `q_curr_full_`
// PERSISTS across ticks, so the slots the bound skips keep whatever they held
// before — zero on the first tick — and the model runs at a partially-ZERO
// configuration that is numerically identical to reading the unreported
// channels outright.
//
// This file is the only place that can show that, because the claim is about a
// buffer that survives between calls. It needs a real model (the reorder map is
// built from joint names), so unlike test_device_readability_gate.cpp it loads
// a URDF — but every assertion is a configuration vector, never a pose, so the
// §3.3 vacuity trap does not apply.

#include "integrated_bringup/support/combined_model_cache.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include "rtc_base/types/types.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/types.hpp"

namespace rub = rtc_urdf_bridge;

namespace {

using integrated_bringup::CombinedModelCache;
using rtc::ControllerState;

constexpr int kArmDof = 7;  // iiwa7

const std::vector<std::string>& ArmJointNames() {
  // devices.iiwa7.joint_state_names from config/iiwa7_leap/sim.yaml.
  static const std::vector<std::string> names{"A1", "A2", "A3", "A4", "A5", "A6", "A7"};
  return names;
}

double MeasuredArm(std::size_t i) {
  return 0.11 + 0.07 * static_cast<double>(i);
}

double SecondReading(std::size_t i) {
  return -0.41 - 0.03 * static_cast<double>(i);
}

ControllerState MakeState(int arm_channels, double (*value)(std::size_t)) {
  ControllerState state{};
  state.num_devices = 1;
  state.dt = 0.002;
  auto& dev0 = state.devices[0];
  dev0.num_channels = arm_channels;
  dev0.valid = true;
  // Reported channels only — the rest stay zero, which is exactly how an
  // unreported joint presents itself: finite, plausible, and wrong.
  for (std::size_t i = 0; i < static_cast<std::size_t>(arm_channels); ++i) {
    dev0.positions[i] = value(i);
    dev0.velocities[i] = 0.0;
  }
  return state;
}

class CombinedModelCacheGateTest : public ::testing::Test {
 protected:
  CombinedModelCache cache_;
  std::shared_ptr<rub::PinocchioModelBuilder> builder_;
  rclcpp::Logger logger_ = rclcpp::get_logger("combined_model_cache_gate_test");

  void SetUp() override {
    rub::ModelConfig cfg;
    cfg.urdf_path = ament_index_cpp::get_package_share_directory("robot_descriptions") +
                    "/robots/iiwa7_leap/urdf/iiwa7_with_leap_right.urdf.xacro";
    cfg.root_joint_type = "fixed";
    cfg.sub_models.push_back({"iiwa7", "link_0", "ee_link"});
    cfg.tree_models.push_back(
        {"wbc",
         "link_0",
         {"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"}});
    ASSERT_NO_THROW(builder_ = std::make_shared<rub::PinocchioModelBuilder>(cfg));

    ASSERT_TRUE(cache_.InitModel(*builder_, {}, "[test]", logger_));
    cache_.BuildReorderMap(&ArmJointNames(), nullptr, kArmDof, "[test]", logger_);
    ASSERT_TRUE(cache_.reorder_valid()) << "fixture precondition: the arm joint names must map";
  }

  // The value the model currently holds for external arm index `i`.
  double ModelValue(int i) const { return cache_.q()[cache_.ext_to_pin_q(i)]; }
};

TEST_F(CombinedModelCacheGateTest, AReadableDeviceIsScattered) {
  // Positive control: without this the "unchanged" assertions below would pass
  // on a helper that never writes anything at all.
  ControllerState full = MakeState(kArmDof, MeasuredArm);
  cache_.ExtractFullState(full, kArmDof, 0);

  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(ModelValue(i), MeasuredArm(static_cast<std::size_t>(i)), 1e-12) << "joint " << i;
  }
}

TEST_F(CombinedModelCacheGateTest, ANarrowDeviceScattersNothingRatherThanAPrefix) {
  // Seed the persistent buffer with a full reading.
  ControllerState full = MakeState(kArmDof, MeasuredArm);
  cache_.ExtractFullState(full, kArmDof, 0);

  // Now a narrower reading arrives. The bound alone would scatter its first
  // nc0 entries and leave the rest at the previous tick's values, handing the
  // model a configuration that is part new measurement and part history with
  // nothing to mark the seam. The gate refuses the whole update instead.
  ControllerState narrow = MakeState(kArmDof - 3, SecondReading);
  cache_.ExtractFullState(narrow, kArmDof, 0);

  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(ModelValue(i), MeasuredArm(static_cast<std::size_t>(i)), 1e-12)
        << "joint " << i << " took part of an unreadable device's reading";
  }
}

TEST_F(CombinedModelCacheGateTest, ANarrowFirstReadingLeavesTheModelAtItsInitialisedValue) {
  // The first-tick form of the same hazard, and the one that actually reaches
  // hardware: nothing has been scattered yet, so the slots past nc0 are the
  // zeros SelectModel allocated. Scattering the prefix would put the model at a
  // partially-ZERO configuration — the exact state §3.7 exists to prevent.
  ControllerState narrow = MakeState(kArmDof - 3, MeasuredArm);
  cache_.ExtractFullState(narrow, kArmDof, 0);

  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(ModelValue(i), 0.0, 1e-12)
        << "joint " << i << " was scattered from a device that never reported it";
  }
}

TEST_F(CombinedModelCacheGateTest, AnUnreportedDeviceScattersNothing) {
  ControllerState full = MakeState(kArmDof, MeasuredArm);
  cache_.ExtractFullState(full, kArmDof, 0);

  ControllerState invalid = MakeState(kArmDof, SecondReading);
  invalid.devices[0].valid = false;
  cache_.ExtractFullState(invalid, kArmDof, 0);

  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(ModelValue(i), MeasuredArm(static_cast<std::size_t>(i)), 1e-12) << "joint " << i;
  }
}

TEST_F(CombinedModelCacheGateTest, AWideDeviceIsScatteredAndBounded) {
  // Over-reporting stays a normal input: the model takes its arm_dof_ joints
  // and the bound keeps the loop off the end of the reorder map.
  ControllerState wide = MakeState(rtc::kMaxDeviceChannels, MeasuredArm);
  cache_.ExtractFullState(wide, kArmDof, 0);

  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(ModelValue(i), MeasuredArm(static_cast<std::size_t>(i)), 1e-12) << "joint " << i;
  }
}

// ── The HAND half: the deferral #236 S7b left open, resolved (#291) ───────────
//
// S7b gated the arm half here and left the hand on `valid` + ModelChannelBound,
// with a comment saying that tightening it "would change which ticks refresh the
// hand model — a separate decision". This is that decision, and the tests below
// are what it now means. The reasoning is NOT "the bound was enough" — the file
// header explains at length why a bound over a persistent buffer never is — it
// is that a partial hand refresh produces a SPLICED configuration: fingers
// [0, nc1) from this tick, fingers [nc1, hand_dof) from whenever they were last
// written. Every consumer of this block (fingertip FK, the closed-chain
// constraint blocks) is a kinematic function of the whole hand, so a coherent
// configuration one tick old is a defensible input and an incoherent splice is
// not.

constexpr int kHandDof = 16;  // leap right

const std::vector<std::string>& HandJointNames() {
  // devices.leap.joint_state_names from config/iiwa7_leap/sim.yaml — thumb
  // first, matching the shipped controller-facing order.
  static const std::vector<std::string> names{"12", "13", "14", "15", "0", "1", "2",  "3",
                                              "4",  "5",  "6",  "7",  "8", "9", "10", "11"};
  return names;
}

double MeasuredHand(std::size_t i) {
  return -0.23 - 0.05 * static_cast<double>(i);
}

double SecondHandReading(std::size_t i) {
  return 0.37 + 0.02 * static_cast<double>(i);
}

ControllerState MakeTwoDeviceState(int arm_channels, int hand_channels,
                                   double (*arm_value)(std::size_t),
                                   double (*hand_value)(std::size_t)) {
  ControllerState state = MakeState(arm_channels, arm_value);
  state.num_devices = 2;
  auto& dev1 = state.devices[1];
  dev1.num_channels = hand_channels;
  dev1.valid = true;
  for (std::size_t i = 0; i < static_cast<std::size_t>(hand_channels); ++i) {
    dev1.positions[i] = hand_value(i);
    dev1.velocities[i] = 0.0;
  }
  return state;
}

class CombinedModelCacheHandGateTest : public ::testing::Test {
 protected:
  CombinedModelCache cache_;
  std::shared_ptr<rub::PinocchioModelBuilder> builder_;
  rclcpp::Logger logger_ = rclcpp::get_logger("combined_model_cache_hand_gate_test");

  void SetUp() override {
    rub::ModelConfig cfg;
    cfg.urdf_path = ament_index_cpp::get_package_share_directory("robot_descriptions") +
                    "/robots/iiwa7_leap/urdf/iiwa7_with_leap_right.urdf.xacro";
    cfg.root_joint_type = "fixed";
    cfg.sub_models.push_back({"iiwa7", "link_0", "ee_link"});
    cfg.tree_models.push_back(
        {"wbc",
         "link_0",
         {"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"}});
    ASSERT_NO_THROW(builder_ = std::make_shared<rub::PinocchioModelBuilder>(cfg));

    ASSERT_TRUE(cache_.InitModel(*builder_, {}, "[test]", logger_));
    cache_.BuildReorderMap(&ArmJointNames(), &HandJointNames(), kArmDof + kHandDof, "[test]",
                           logger_);
    ASSERT_TRUE(cache_.reorder_valid())
        << "fixture precondition: arm AND hand joint names must both map";
  }

  double ModelValue(int i) const { return cache_.q()[cache_.ext_to_pin_q(i)]; }

  double HandModelValue(int i) const { return cache_.q()[cache_.ext_to_pin_q(kArmDof + i)]; }
};

TEST_F(CombinedModelCacheHandGateTest, AReadableHandIsScattered) {
  // Positive control: without it, every "unchanged" assertion below would pass
  // on a helper that stopped writing the hand block entirely.
  ControllerState full = MakeTwoDeviceState(kArmDof, kHandDof, MeasuredArm, MeasuredHand);
  cache_.ExtractFullState(full, kArmDof, kHandDof);

  for (int i = 0; i < kHandDof; ++i) {
    EXPECT_NEAR(HandModelValue(i), MeasuredHand(static_cast<std::size_t>(i)), 1e-12)
        << "finger " << i;
  }
}

TEST_F(CombinedModelCacheHandGateTest, ANarrowHandScattersNothingRatherThanAPrefix) {
  ControllerState full = MakeTwoDeviceState(kArmDof, kHandDof, MeasuredArm, MeasuredHand);
  cache_.ExtractFullState(full, kArmDof, kHandDof);

  // A narrower hand arrives. With ModelChannelBound alone the first six fingers
  // would take SecondHandReading while the last ten kept MeasuredHand — a
  // posture the robot was never in. The gate refuses the whole hand update.
  ControllerState narrow = MakeTwoDeviceState(kArmDof, 6, MeasuredArm, SecondHandReading);
  cache_.ExtractFullState(narrow, kArmDof, kHandDof);

  for (int i = 0; i < kHandDof; ++i) {
    EXPECT_NEAR(HandModelValue(i), MeasuredHand(static_cast<std::size_t>(i)), 1e-12)
        << "finger " << i << " took part of an unreadable hand's reading";
  }
}

TEST_F(CombinedModelCacheHandGateTest, ANarrowFirstHandReadingLeavesTheHandBlockAtZero) {
  // The first-tick form, and the one that reaches hardware: nothing has been
  // scattered yet, so a prefix scatter would hand the model six real fingers
  // and ten at the origin.
  ControllerState narrow = MakeTwoDeviceState(kArmDof, 6, MeasuredArm, MeasuredHand);
  cache_.ExtractFullState(narrow, kArmDof, kHandDof);

  for (int i = 0; i < kHandDof; ++i) {
    EXPECT_NEAR(HandModelValue(i), 0.0, 1e-12)
        << "finger " << i << " was scattered from a hand that never reported it";
  }
}

TEST_F(CombinedModelCacheHandGateTest, AnUnreadableHandDoesNotBlockTheArmScatter) {
  // The asymmetry the gate placement encodes, and §3.7's secondary-passthrough
  // rule expressed in the model lane: an unreadable ARM returns early and so
  // invalidates the hand block too (they share one model), but an unreadable
  // HAND must leave the arm scatter intact. Getting this backwards would turn a
  // finger-count mismatch into a frozen arm model.
  ControllerState narrow = MakeTwoDeviceState(kArmDof, 6, MeasuredArm, MeasuredHand);
  cache_.ExtractFullState(narrow, kArmDof, kHandDof);

  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_NEAR(ModelValue(i), MeasuredArm(static_cast<std::size_t>(i)), 1e-12)
        << "arm joint " << i << " was withheld because the HAND was narrow";
  }
}

TEST_F(CombinedModelCacheHandGateTest, AWideHandIsScatteredAndBounded) {
  // Over-reporting stays a normal input on this axis too.
  ControllerState wide = MakeTwoDeviceState(kArmDof, kHandDof + 8, MeasuredArm, MeasuredHand);
  cache_.ExtractFullState(wide, kArmDof, kHandDof);

  for (int i = 0; i < kHandDof; ++i) {
    EXPECT_NEAR(HandModelValue(i), MeasuredHand(static_cast<std::size_t>(i)), 1e-12)
        << "finger " << i;
  }
}

}  // namespace
