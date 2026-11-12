// ── test_g1_loco_configure.cpp ───────────────────────────────────────────────
// The shipped g1_loco profile brings demo_joint_controller up (#977).
//
// The profile is the first whose primary group is a whole body on a tree with
// four branches, whose hand is a closed-chain surrogate and whose command type
// is pd_feedforward. Every one of those is read off the SHIPPED files — the CM
// merge of _base.yaml + sim.yaml for the model and the devices, the controller
// YAML through the CM's loader, demo_shared merged in — so a profile edit that
// production would refuse at on_configure is refused here, not at launch.
//
// The control case at the bottom is the premise this profile changed under: a
// free-flyer root on the same model is locked as a passive joint by the closure
// builder, the hand's closed-chain handle refuses, and on_configure FAILS. The
// shipped `root_joint_type: "fixed"` is what makes the case above succeed, and
// this is the test that would have said so before a launch did.
// ──────────────────────────────────────────────────────────────────────────────
#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/support/combined_model_cache.hpp"
#include "rtc_urdf_bridge/free_flyer_dims.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "shipped_catching_bringup_fixture.hpp"
#include "shipped_config_test_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace rub = rtc_urdf_bridge;
namespace fx = integrated_bringup::testfx;
using integrated_bringup::DemoJointController;
using rtc::ControllerOutput;
using rtc::ControllerState;

constexpr const char *kProfile = "g1_loco";
constexpr const char *kController = "demo_joint_controller";
constexpr int kBodyDof = 23;
constexpr int kHandDof = 10;
constexpr int kActuated = kBodyDof + kHandDof;
constexpr int kHandPassive =
    10; // the five four-bar loops, two passive joints each
constexpr double kDt = 1.0 / fx::kShippedProfileControlRateHz;

/// The controller's YAML as the CM hands it over: the profile's file through
/// the CM loader, the shared block merged in.
YAML::Node ShippedJointConfig() {
  YAML::Node cfg = fx::ShippedControllerNode(kProfile, kController);
  fx::MergeShippedShared(cfg, kProfile);
  return cfg;
}

/// Device configs of the two groups the controller claims, from the shipped
/// YAML, tagged as the simulator.
std::map<std::string, rtc::DeviceNameConfig>
ShippedDevices(const YAML::Node &cfg) {
  std::map<std::string, rtc::DeviceNameConfig> configs =
      fx::ShippedSimConfigs(kProfile, cfg);
  for (auto &[group, device] : configs) {
    rtc::DeviceBackendBinding backend;
    backend.type = "mujoco_native";
    device.backend = backend;
  }
  return configs;
}

/// A measured state at the profile's E-STOP safe position (the `stand`
/// keyframe), hand open.
ControllerState StandState(const YAML::Node &cfg) {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = kDt;
  state.iteration = 1;
  const auto safe = cfg["estop"]["arm_safe_position"].as<std::vector<double>>();
  auto &body = state.devices[0];
  body.num_channels = static_cast<int>(safe.size());
  body.valid = true;
  body.hole_mask = 0;
  for (std::size_t i = 0; i < safe.size(); ++i) {
    body.positions[i] = safe[i];
  }
  auto &hand = state.devices[1];
  hand.num_channels = kHandDof;
  hand.valid = true;
  hand.hole_mask = 0;
  return state;
}

class G1LocoConfigure : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  /// The CM's bring-up order: model + builder + rate, PreConfigure (node +
  /// LoadConfig), device configs, on_configure.
  std::unique_ptr<DemoJointController>
  BringUp(const rub::ModelConfig &model,
          std::shared_ptr<rub::PinocchioModelBuilder> builder,
          const char *node_name) {
    cfg_ = ShippedJointConfig();
    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    node_ =
        std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, "", opts);
    auto ctrl = std::make_unique<DemoJointController>("");
    ctrl->SetSystemModelConfig(model);
    ctrl->SetSharedModelBuilder(std::move(builder));
    ctrl->SetControlRate(fx::kShippedProfileControlRateHz);
    EXPECT_EQ(ctrl->PreConfigure(node_, cfg_),
              DemoJointController::CallbackReturn::SUCCESS)
        << node_name;
    ctrl->SetDeviceNameConfigs(ShippedDevices(cfg_));
    return ctrl;
  }

  DemoJointController::CallbackReturn Configure(DemoJointController &ctrl) {
    return ctrl.on_configure(rclcpp_lifecycle::State{}, node_, cfg_);
  }

  YAML::Node cfg_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

// ── The shipped profile ──────────────────────────────────────────────────────

TEST_F(G1LocoConfigure,
       TheShippedModelIsAFixedRootWholeBodyWithAClosedChainHand) {
  const rub::ModelConfig model = fx::ShippedModelConfig(kProfile);
  EXPECT_EQ(model.root_joint_type, "fixed");
  EXPECT_TRUE(model.sub_models.empty()) << "no serial arm: the body is a tree";
  ASSERT_EQ(model.tree_models.size(), 2U);
  EXPECT_EQ(model.tree_models[0].name, "g1");
  EXPECT_EQ(model.tree_models[0].root_link, "pelvis");
  EXPECT_EQ(model.tree_models[1].name, "p1b");
  EXPECT_EQ(model.tree_models[1].root_link, "base_adapter");
  // The hand mount is where the body tree's right branch ends.
  const auto &body_tips = model.tree_models[0].tip_links;
  EXPECT_NE(std::find(body_tips.begin(), body_tips.end(), "base_adapter"),
            body_tips.end());

  const auto builder = fx::ShippedModelBuilder(kProfile);
  ASSERT_NE(builder->GetFullModel(), nullptr);
  EXPECT_EQ(builder->GetFullModel()->nq, kActuated + kHandPassive);
  ASSERT_NE(builder->GetTreeModel("g1"), nullptr);
  EXPECT_EQ(builder->GetTreeModel("g1")->nv, kBodyDof);
  // The closure sidecar lists every driven joint: the actuated model is the
  // whole body plus the hand's ten, nothing locked that a device drives.
  ASSERT_NE(builder->GetActuatedModel(), nullptr)
      << "the hand's loops must be loaded";
  EXPECT_EQ(builder->GetActuatedModel()->nv, kActuated);
}

TEST_F(G1LocoConfigure, TheShippedControllerConfigures) {
  const rub::ModelConfig model = fx::ShippedModelConfig(kProfile);
  auto ctrl =
      BringUp(model, fx::ShippedModelBuilder(kProfile), "g1_loco_configure");
  ASSERT_EQ(Configure(*ctrl), DemoJointController::CallbackReturn::SUCCESS)
      << "hand FK wiring: " << ctrl->HandFkWiringErrorForTesting()
      << "\narm tip: " << ctrl->ArmTipConfigError();
  EXPECT_EQ(ctrl->HandFkWiringErrorForTesting(), "");
  EXPECT_EQ(ctrl->GetCommandType(), rtc::CommandType::kPdFeedforward)
      << "the profile ships command_type: pd_feedforward";
  EXPECT_EQ(ctrl->OwnedStateFrameIdForTesting(), "pelvis");

  // TF slots: the hand mount, the four fingertips and the virtual TCP, all in
  // the tree root — the same shape as g1_p1b's, on the other body.
  const std::vector<std::pair<std::string, std::string>> expected = {
      {"pelvis", "base_adapter_actual"},
      {"pelvis", "l_thumb_tip_bracket_actual"},
      {"pelvis", "l_index_tip_bracket_actual"},
      {"pelvis", "l_middle_tip_bracket_actual"},
      {"pelvis", "l_ring_tip_bracket_actual"},
      {"pelvis", "virtual_tcp_actual"},
  };
  EXPECT_EQ(ctrl->TfSlotFramesForTesting(), expected);
}

// Ticks at the keyframe: the hold seeds from the measured pose and every pose
// lane the sim bring-up reads is valid — a pd_feedforward output with the 23
// body commands at the stance and zero feedforward.
//
// The fingertip poses take a few ticks: the hand's closed-chain projection
// seeds from the neutral configuration and walks its actuated vector — all 33
// joints, the body's among them — toward the measurement at most
// kDefaultActuatedIncrement (0.05 rad) per tick (NUM-5), holding the previous
// pose until the loops close. The stance's knee is 0.6 rad from neutral, so the
// fingertips are trustworthy after ~12 ticks; 40 leaves a margin and is still
// 80 ms of a 500 Hz run.
TEST_F(G1LocoConfigure, TheConfiguredControllerHoldsTheStanceWithValidPoses) {
  const rub::ModelConfig model = fx::ShippedModelConfig(kProfile);
  auto ctrl = BringUp(model, fx::ShippedModelBuilder(kProfile), "g1_loco_hold");
  ASSERT_EQ(Configure(*ctrl), DemoJointController::CallbackReturn::SUCCESS)
      << ctrl->HandFkWiringErrorForTesting();

  ControllerState state = StandState(cfg_);
  ControllerOutput out = ctrl->Compute(state);
  for (int i = 0; i < 40; ++i) {
    state.iteration += 1;
    out = ctrl->Compute(state);
  }
  ASSERT_TRUE(out.valid);
  EXPECT_EQ(out.command_type, rtc::CommandType::kPdFeedforward);
  ASSERT_EQ(out.num_devices, 2);
  EXPECT_EQ(out.devices[0].num_channels, kBodyDof);
  EXPECT_EQ(out.devices[1].num_channels, kHandDof);
  const auto safe =
      cfg_["estop"]["arm_safe_position"].as<std::vector<double>>();
  for (std::size_t i = 0; i < safe.size(); ++i) {
    EXPECT_NEAR(out.devices[0].commands[i], safe[i], 1e-9)
        << "body joint " << i;
    EXPECT_DOUBLE_EQ(out.devices[0].feedforward[i], 0.0) << "body joint " << i;
  }
  EXPECT_TRUE(out.arm_tip_pose_valid);
  for (std::size_t f = 0; f < 4; ++f) {
    EXPECT_TRUE(out.task_link_pose_valid[f]) << "fingertip " << f;
  }
}

// ── The control model's cache on a floating root (W1-F04)
// ─────────────────────
//
// The cache scatters device joints only, so a free-flyer root keeps the value
// SelectModel seeded: neutral (a unit quaternion), not zero. With the base at
// identity the joint block of M and g and every registered frame's pose equal
// the fixed-root model's.
TEST_F(G1LocoConfigure,
       AFloatingCacheSeedsANeutralBaseAndMatchesTheFixedJointBlock) {
  const YAML::Node cfg = ShippedJointConfig();
  const auto devices = ShippedDevices(cfg);
  const std::vector<std::string> &body = devices.at("g1").joint_state_names;
  const std::vector<std::string> &hand = devices.at("p1b").joint_state_names;
  const std::vector<std::string> frames = {
      "base_adapter",        "left_sole",           "right_sole",
      "l_thumb_tip_bracket", "l_index_tip_bracket", "l_ring_tip_bracket"};
  const rclcpp::Logger logger = rclcpp::get_logger("g1_loco_cache_test");
  const ControllerState state = StandState(cfg);

  struct Built {
    std::shared_ptr<rub::PinocchioModelBuilder> builder;
    std::unique_ptr<integrated_bringup::CombinedModelCache> cache;
  };

  const auto build = [&](const std::string &root) {
    rub::ModelConfig model = fx::ShippedModelConfig(kProfile);
    model.root_joint_type = root;
    Built b{std::make_shared<rub::PinocchioModelBuilder>(model),
            std::make_unique<integrated_bringup::CombinedModelCache>()};
    EXPECT_TRUE(b.cache->InitModel(*b.builder, {}, "[test]", logger)) << root;
    b.cache->BuildReorderMap(&body, &hand, kActuated, "[test]", logger);
    EXPECT_TRUE(b.cache->reorder_valid()) << root;
    for (const auto &f : frames) {
      static_cast<void>(
          b.cache->cache().RegisterFrame(f, b.cache->model()->getFrameId(f)));
    }
    b.cache->ExtractFullState(state, kBodyDof, kHandDof);
    b.cache->Update();
    return b;
  };
  const Built fixed = build("fixed");
  const Built floating = build("floating");
  const pinocchio::Model &ml = *floating.cache->model();
  ASSERT_EQ(ml.nv, kActuated + rub::kFreeFlyerNv)
      << "the floating control model is the actuated one, plus the base";
  ASSERT_EQ(fixed.cache->model()->nv, kActuated);

  const Eigen::VectorXd neutral = pinocchio::neutral(ml);
  for (int i = 0; i < rub::kFreeFlyerNq; ++i) {
    EXPECT_EQ(floating.cache->q()[i], neutral[i]) << "base slot " << i;
  }
  const auto &cl = floating.cache->cache();
  const auto &cx = fixed.cache->cache();
  EXPECT_LE((cl.M.bottomRightCorner(kActuated, kActuated) - cx.M)
                .cwiseAbs()
                .maxCoeff(),
            1e-12);
  EXPECT_LE((cl.g.tail(kActuated) - cx.g).cwiseAbs().maxCoeff(), 1e-12);
  ASSERT_EQ(cl.registered_frames.size(), frames.size());
  for (std::size_t k = 0; k < frames.size(); ++k) {
    EXPECT_EQ(cl.registered_frames[k].name, frames[k]);
    const pinocchio::SE3 &a = cl.registered_frames[k].oMf;
    const pinocchio::SE3 &b = cx.registered_frames[k].oMf;
    EXPECT_LE((a.translation() - b.translation()).norm(), 1e-12) << frames[k];
    EXPECT_LE((a.rotation() - b.rotation()).norm(), 1e-12) << frames[k];
  }

  // The identity reorder fallback (no primary device config) starts after the
  // base.
  integrated_bringup::CombinedModelCache identity;
  ASSERT_TRUE(identity.SelectModel(*floating.builder, "[test]", logger));
  identity.BuildReorderMap(nullptr, nullptr, kActuated, "[test]", logger);
  EXPECT_EQ(identity.ext_to_pin_q(0), rub::kFreeFlyerNq);
  EXPECT_EQ(identity.ext_to_pin_v(0), rub::kFreeFlyerNv);
  EXPECT_EQ(identity.ext_to_pin_q(kActuated - 1), ml.nq - 1);
}

// ── The premise: a floating root refuses ─────────────────────────────────────
//
// The same model with `root_joint_type: "floating"`. The closure builder locks
// every joint the sidecar does not list as actuated, the free-flyer among them,
// and the hand's closed-chain handle refuses (more dependent DoF than
// constraint rows) — the serial fallback cannot carry the hand's device joints,
// so on_configure FAILS. This is why the shipped profile keeps a fixed root and
// takes the floating base from the simulator's lane instead.
TEST_F(G1LocoConfigure, AFloatingRootOnTheSameModelRefusesConfigure) {
  rub::ModelConfig model = fx::ShippedModelConfig(kProfile);
  model.root_joint_type = "floating";
  auto builder = std::make_shared<rub::PinocchioModelBuilder>(model);
  auto ctrl = BringUp(model, builder, "g1_loco_floating_root");
  EXPECT_EQ(Configure(*ctrl), DemoJointController::CallbackReturn::FAILURE);
  EXPECT_NE(ctrl->HandFkWiringErrorForTesting(), "")
      << "the refusal must come from the hand FK wiring, not from somewhere "
         "else";
}

} // namespace
