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
        {"wbc", "link_0", {"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"}});
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

}  // namespace
