// ── DemoInferenceController: binding contract ────────────────────────────────
//
// Driven with an injected fake engine, so every success path is exercised
// without a .onnx file — which is the point of the injection seam, since no
// policy exists yet and a unit test should not need one even once it does.
//
// The cases are grouped around the two things that can go wrong quietly:
//   * the policy runs on the wrong cadence (its action then covers a different
//     amount of simulated time than it was trained for), and
//   * a bad tick ships a command anyway instead of holding.

#include "inference_fake_engine.hpp"
#include "integrated_bringup/controllers/demo_inference_controller.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using integrated_bringup::DemoInferenceController;
using rtc::ControllerOutput;
using rtc::ControllerState;

constexpr int kArmDof = 6;
constexpr int kHandDof = 10;
constexpr int kFingertips = 4;
constexpr int kInputElements = 20;  // 6 arm + 10 hand + 4 force norms
constexpr int kInferenceStride = 7;

using integrated_bringup::testfx::FakeEngine;

std::map<std::string, rtc::DeviceNameConfig> MakeDeviceConfigs() {
  rtc::DeviceNameConfig arm;
  arm.device_name = "arm";
  for (int i = 0; i < kArmDof; ++i) {
    arm.joint_state_names.push_back("arm_j" + std::to_string(i));
  }
  rtc::DeviceJointLimits arm_lim;
  arm_lim.position_lower.assign(kArmDof, -6.28);
  arm_lim.position_upper.assign(kArmDof, 6.28);
  arm_lim.max_velocity.assign(kArmDof, 2.0);
  arm.joint_limits = arm_lim;

  rtc::DeviceNameConfig hand;
  hand.device_name = "hand";
  for (int i = 0; i < kHandDof; ++i) {
    hand.joint_state_names.push_back("hand_j" + std::to_string(i));
  }
  for (int f = 0; f < kFingertips; ++f) {
    hand.sensor_names.push_back("ft" + std::to_string(f));
  }
  rtc::DeviceJointLimits hand_lim;
  // Mirrors the real p1b shape: several joints flex NEGATIVE, upper bound 0.
  hand_lim.position_lower.assign(kHandDof, -1.5708);
  hand_lim.position_upper.assign(kHandDof, 0.0);
  hand_lim.max_velocity.assign(kHandDof, 10.0);
  hand.joint_limits = hand_lim;

  rtc::DeviceSensorLayout layout;
  layout.inference_values_per_group = kInferenceStride;
  hand.sensor_layout = layout;

  return {{"arm", arm}, {"hand", hand}};
}

std::string MakeYaml(int decimation = 10, const std::string& model_path = "fake_policy.onnx",
                     bool allow_missing = false) {
  return R"(
command_type: "position"
topics:
  arm:
    subscribe:
      - topic: "arm/joint_goal"
        role: "target"
  hand:
    subscribe:
      - topic: "hand/joint_goal"
        role: "target"
inference:
  model_path: ")" +
         model_path + R"("
  allow_missing_model: )" +
         std::string(allow_missing ? "true" : "false") + R"(
  decimation: )" +
         std::to_string(decimation) + R"(
  inputs:
    - name: "obs"
      shape: [1, 20]
      features:
        - "arm.position"
        - "hand.position"
        - "hand.fingertip_force_norm"
  outputs:
    - name: "arm_action"
      shape: [1, 6]
    - name: "posture"
      shape: [1, 1]
  output_features:
    - { tensor: "arm_action", role: "joint_target",   device: "arm" }
    - { tensor: "posture",    role: "posture_scalar", device: "hand" }
  hand_posture:
    open:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    close: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
)";
}

/// A schema that adds the object pose lane (27 elements). Link features are
/// deliberately absent: they need a real URDF, and the object lane's contract —
/// matching, staleness, frame checking — is independent of it. So is the
/// frame change: `source_frame_link` names the policy frame itself, which needs
/// no model. The URDF-backed rotation is test_demo_inference_urdf.cpp's.
std::string MakeObjectYaml(double timeout_sec = 0.02, const std::string& match_mode = "prefix",
                           const std::string& frame_match = "pool_",
                           const std::string& source_frame_id = "world") {
  return R"(
command_type: "position"
topics:
  arm:
    subscribe:
      - topic: "arm/joint_goal"
        role: "target"
  hand:
    subscribe:
      - topic: "hand/joint_goal"
        role: "target"
inference:
  model_path: "fake_policy.onnx"
  decimation: 1
  inputs:
    - name: "obs"
      shape: [1, 27]
      features:
        - "arm.position"
        - "hand.position"
        - "hand.fingertip_force_norm"
        - "object.position"
        - "object.orientation_xyzw"
  outputs:
    - name: "arm_action"
      shape: [1, 6]
    - name: "posture"
      shape: [1, 1]
  output_features:
    - { tensor: "arm_action", role: "joint_target",   device: "arm" }
    - { tensor: "posture",    role: "posture_scalar", device: "hand" }
  policy_frame: "world_link"
  object_pose:
    topic: "/sim/object_transforms"
    match_mode: ")" +
         match_mode + R"("
    frame_match: ")" +
         frame_match + R"("
    source_frame_id: ")" +
         source_frame_id + R"("
    source_frame_link: "world_link"
    timeout_sec: )" +
         std::to_string(timeout_sec) + R"(
  hand_posture:
    open:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    close: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
)";
}

/// One TFMessage carrying `names` at distinct positions.
tf2_msgs::msg::TFMessage MakeTfMessage(const std::vector<std::string>& names,
                                       const std::string& frame_id = "world") {
  tf2_msgs::msg::TFMessage msg;
  double k = 1.0;
  for (const auto& n : names) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = frame_id;
    tf.child_frame_id = n;
    tf.transform.translation.x = k;
    tf.transform.translation.y = 2.0 * k;
    tf.transform.translation.z = 3.0 * k;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    msg.transforms.push_back(tf);
    k += 1.0;
  }
  return msg;
}

ControllerState MakeState(double dt = 0.002) {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = dt;
  state.iteration = 1;
  auto& arm = state.devices[0];
  arm.num_channels = kArmDof;
  arm.valid = true;
  auto& hand = state.devices[1];
  hand.num_channels = kHandDof;
  hand.valid = true;
  hand.num_inference_groups = kFingertips;
  return state;
}

/// Drives the REAL three-pass bring-up: PreConfigure→LoadConfig (Pass 1),
/// SetDeviceNameConfigs (Pass 2), on_configure (Pass 3).
///
/// The node is not scenery. Half of this controller's validation — every rule
/// that compares the policy's declared widths against the device rosters —
/// only runs in Pass 3, so a fixture that stopped at LoadConfig would exercise
/// none of it and would still be able to reach Compute(). The order here is the
/// contract, not a convenience.
struct Harness {
  FakeEngine* engine{nullptr};
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::unique_ptr<DemoInferenceController> ctrl;
  rtc::RTControllerInterface::CallbackReturn configure_result{
      rtc::RTControllerInterface::CallbackReturn::FAILURE};

  explicit Harness(
      const std::string& yaml = MakeYaml(), bool stub_engine = false,
      const std::vector<std::size_t>& in_sizes = {static_cast<std::size_t>(kInputElements)},
      const std::vector<std::size_t>& out_sizes = {6, 1}) {
    auto fake = std::make_unique<FakeEngine>(in_sizes, out_sizes);
    if (stub_engine) {
      fake->stub = true;
    }
    engine = fake.get();
    for (const auto n : out_sizes) {
      engine->next_output.emplace_back(n, 0.0F);
    }
    ctrl = std::make_unique<DemoInferenceController>("", std::move(fake));

    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    static int seq = 0;
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "inference_harness_" + std::to_string(seq++), "", opts);

    const YAML::Node cfg = YAML::Load(yaml);
    ctrl->LoadConfig(cfg);
    ctrl->SetDeviceNameConfigs(MakeDeviceConfigs());
    ctrl->OnDeviceConfigsSet();
    configure_result = ctrl->on_configure(rclcpp_lifecycle::State{}, node, cfg);
  }

  /// Activate too, for cases that drive Compute(). Kept separate so a case can
  /// assert on a configure refusal without also demanding activation.
  [[nodiscard]] bool Activate() {
    return ctrl->on_activate(rclcpp_lifecycle::State{}) ==
           rtc::RTControllerInterface::CallbackReturn::SUCCESS;
  }
};

/// gtest environment: one rclcpp context for the whole binary.
class RclcppEnv : public ::testing::Environment {
 public:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

const auto* const kEnv = ::testing::AddGlobalTestEnvironment(new RclcppEnv);

/// Mark every fingertip group fresh with a known force triple.
void SetFingertipForce(ControllerState& s, int f, float fx, float fy, float fz) {
  auto& hand = s.devices[1];
  hand.inference_enable[static_cast<std::size_t>(f)] = true;
  const auto base = static_cast<std::size_t>(f * kInferenceStride);
  hand.inference_data[base] = 0.0F;
  hand.inference_data[base + 1] = fx;
  hand.inference_data[base + 2] = fy;
  hand.inference_data[base + 3] = fz;
}

}  // namespace

// ── Schema ──────────────────────────────────────────────────────────────────

TEST(DemoInferenceConfig, ResolvesFeatureWidthsFromTheDeviceRosters) {
  Harness h;
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  const auto& io = h.ctrl->IoParamsForTesting();
  ASSERT_EQ(io.inputs.size(), 1U);
  EXPECT_EQ(io.inputs[0].name, "obs") << "the .onnx tensor name the engine binds by";
  const auto& segs = io.inputs[0].segments;
  ASSERT_EQ(segs.size(), 3U);
  EXPECT_EQ(segs[0].count, kArmDof);
  EXPECT_EQ(segs[1].count, kHandDof);
  EXPECT_EQ(segs[2].count, kFingertips);
  EXPECT_EQ(io.decimation, 10);
}

TEST(DemoInferenceConfig, RejectsATorqueCommandType) {
  // The heads carry radians. Reinterpreting them as newton-metres would parse
  // cleanly and then be wrong at the actuator boundary.
  std::string yaml = MakeYaml();
  yaml.replace(yaml.find("\"position\""), std::string("\"position\"").size(), "\"torque\"");
  DemoInferenceController ctrl(
      "", std::make_unique<FakeEngine>(std::vector<std::size_t>{kInputElements},
                                       std::vector<std::size_t>{6, 1}));
  EXPECT_THROW(ctrl.LoadConfig(YAML::Load(yaml)), std::invalid_argument);
}

TEST(DemoInferenceConfig, RejectsMismatchedPostureLengths) {
  // Now a Pass 3 refusal rather than a LoadConfig throw. `hand_posture` moved
  // out of Pass 1 because whether it is required at all depends on the declared
  // role, and roles need the device rosters (#511 D-9). The rule itself is
  // unchanged — this is the same assertion at the stage that now owns it, not
  // a weakened one (PROC-6).
  std::string yaml = MakeYaml();
  yaml.replace(
      yaml.find("close: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]"),
      std::string("close: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]").size(),
      "close: [-1.0, -1.0]");
  Harness h{yaml};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceConfig, RejectsAHandSliceWiderThanOneElement) {
  // Widen the posture TENSOR rather than the slice. A slice wider than its own
  // tensor is refused one layer down by the schema parser, so pointing this
  // case at that would have tested the parser twice and left the binding's own
  // "the scalar is exactly one element" check unexercised — the check that
  // stands between a two-wide head and a posture blend reading half a command.
  std::string yaml = MakeYaml();
  yaml.replace(yaml.find("- name: \"posture\"\n      shape: [1, 1]"),
               std::string("- name: \"posture\"\n      shape: [1, 1]").size(),
               "- name: \"posture\"\n      shape: [1, 2]");
  Harness h{yaml};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceConfig, RejectsAnArmSliceThatDisagreesWithTheDeviceRoster) {
  // The retrain failure this controller exists to catch: the policy still says
  // 5 joints, the robot has 6. Both numbers are plausible on their own.
  std::string yaml = MakeYaml();
  // Legal against the tensor (5 of 6 elements), so the schema parser passes it
  // and the binding's own roster cross-check is what has to refuse it.
  yaml.replace(yaml.find("role: \"joint_target\",   device: \"arm\" }"),
               std::string("role: \"joint_target\",   device: \"arm\" }").size(),
               "role: \"joint_target\", device: \"arm\", count: 5 }");
  Harness h{yaml};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

// ── Output roles: declared, never inferred (#511 B-2, D-4, D-5) ────────────

namespace {

/// A schema whose two devices take the SAME role — the natural shape of a
/// multi-output policy, and the config the pre-#511 suffix loop resolved by
/// keeping whichever entry came last. Widths differ (6 vs 10) so a swap cannot
/// hide behind a coincidence.
std::string MakeTwoJointTargetYaml() {
  return R"(
command_type: "position"
topics:
  arm:
    subscribe:
      - topic: "arm/joint_goal"
        role: "target"
  hand:
    subscribe:
      - topic: "hand/joint_goal"
        role: "target"
inference:
  model_path: "fake_policy.onnx"
  decimation: 1
  inputs:
    - name: "obs"
      shape: [1, 20]
      features:
        - "arm.position"
        - "hand.position"
        - "hand.fingertip_force_norm"
  outputs:
    - name: "arm_action"
      shape: [1, 6]
    - name: "hand_action"
      shape: [1, 10]
  output_features:
    - { tensor: "arm_action",  role: "joint_target", device: "arm" }
    - { tensor: "hand_action", role: "joint_target", device: "hand" }
)";
}

}  // namespace

TEST(DemoInferenceRoles, TwoDevicesInTheSameRoleEachDriveTheirOwn) {
  // The #511 B-2 regression. Under the suffix loop both entries ended in
  // `target_position`, the loop had no `break`, and the hand's slice won — so
  // the ARM was driven by the hand head. Every commanded value stayed finite
  // and inside the joint limits, which is why nothing downstream could see it.
  Harness h{MakeTwoJointTargetYaml(), false, {static_cast<std::size_t>(kInputElements)}, {6, 10}};
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());

  // Distinct constants per tensor, so "which head drove which device" is
  // readable straight off the command.
  h.engine->next_output = {std::vector<float>(6, 0.25F), std::vector<float>(10, -0.75F)};

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  const auto out = h.ctrl->Compute(state);
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());

  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_DOUBLE_EQ(out.devices[0].target_positions[static_cast<std::size_t>(i)], 0.25)
        << "arm joint " << i << " must come from the tensor its own entry names";
  }
  for (int i = 0; i < kHandDof; ++i) {
    EXPECT_DOUBLE_EQ(out.devices[1].target_positions[static_cast<std::size_t>(i)], -0.75)
        << "hand joint " << i;
  }
}

TEST(DemoInferenceRoles, ConfiguresWithoutHandPostureWhenTheHandTakesJointTargets) {
  // #511 D-9. `hand_posture` used to be required unconditionally, so a policy
  // that commands the hand joints directly failed configure over two lists it
  // would never read. MakeTwoJointTargetYaml declares no hand_posture at all.
  Harness h{MakeTwoJointTargetYaml(), false, {static_cast<std::size_t>(kInputElements)}, {6, 10}};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
}

TEST(DemoInferenceRoles, RejectsAnUnknownDevice) {
  // `banana.target_position` used to bind to the arm, because the old loop only
  // ever looked at the suffix.
  std::string yaml = MakeTwoJointTargetYaml();
  yaml.replace(yaml.find("device: \"arm\""), std::string("device: \"arm\"").size(),
               "device: \"banana\"");
  Harness h{yaml, false, {static_cast<std::size_t>(kInputElements)}, {6, 10}};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceRoles, RejectsAnUnknownRole) {
  std::string yaml = MakeTwoJointTargetYaml();
  yaml.replace(yaml.find("role: \"joint_target\", device: \"hand\""),
               std::string("role: \"joint_target\", device: \"hand\"").size(),
               "role: \"wrist_wiggle\", device: \"hand\"");
  Harness h{yaml, false, {static_cast<std::size_t>(kInputElements)}, {6, 10}};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceRoles, RejectsTwoRolesAimedAtTheHand) {
  // The schema layer refuses the same role twice on one device; this is the
  // other shape — two DIFFERENT roles for one device, i.e. two commands for the
  // same joints with no defensible order to apply them in.
  std::string yaml = MakeTwoJointTargetYaml();
  yaml.replace(
      yaml.find("    - { tensor: \"hand_action\", role: \"joint_target\", device: \"hand\" }"),
      std::string("    - { tensor: \"hand_action\", role: \"joint_target\", device: \"hand\" }")
          .size(),
      "    - { tensor: \"hand_action\", role: \"joint_target\", device: \"hand\" }\n"
      "    - { tensor: \"arm_action\", role: \"posture_scalar\", device: \"hand\", offset: 0, "
      "count: 1 }");
  Harness h{yaml, false, {static_cast<std::size_t>(kInputElements)}, {6, 10}};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceRoles, RejectsAMissingArmRole) {
  // Without a primary joint_target the arm receives nothing and the controller
  // can only ever hold — a config that brings up "successfully" and moves
  // nothing is worse than one that refuses.
  std::string yaml = MakeTwoJointTargetYaml();
  yaml.replace(
      yaml.find("    - { tensor: \"arm_action\",  role: \"joint_target\", device: \"arm\" }\n"),
      std::string("    - { tensor: \"arm_action\",  role: \"joint_target\", device: \"arm\" }\n")
          .size(),
      "");
  Harness h{yaml, false, {static_cast<std::size_t>(kInputElements)}, {6, 10}};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

// ── Recurrent state (#511 P5) ──────────────────────────────────────────────

namespace {

/// `obs` + `h_in` in, `arm_action` + `posture` + `h_out` out, `h_out` feeding
/// `h_in`. `reset_after_hold_sec` is left at whatever the case wants.
std::string MakeRecurrentYaml(const std::string& reset_line = "") {
  return R"(
command_type: "position"
topics:
  arm:
    subscribe:
      - topic: "arm/joint_goal"
        role: "target"
  hand:
    subscribe:
      - topic: "hand/joint_goal"
        role: "target"
inference:
  model_path: "fake_policy.onnx"
  decimation: 1
)" + reset_line +
         R"(  inputs:
    - name: "obs"
      shape: [1, 20]
      features:
        - "arm.position"
        - "hand.position"
        - "hand.fingertip_force_norm"
    - name: "h_in"
      shape: [1, 4]
      source: recurrent
  outputs:
    - name: "arm_action"
      shape: [1, 6]
    - name: "posture"
      shape: [1, 1]
    - name: "h_out"
      shape: [1, 4]
      feeds: "h_in"
  output_features:
    - { tensor: "arm_action", role: "joint_target",   device: "arm" }
    - { tensor: "posture",    role: "posture_scalar", device: "hand" }
  hand_posture:
    open:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    close: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
)";
}

/// A recurrent harness with the state head wired: `h_out` always emits @p v.
Harness MakeRecurrentHarness(const std::string& reset_line = "") {
  Harness h{MakeRecurrentYaml(reset_line),
            false,
            {static_cast<std::size_t>(kInputElements), 4U},
            {6, 1, 4}};
  return h;
}

}  // namespace

TEST(DemoInferenceRecurrent, TheStateHeadFeedsTheNextStepsStateInput) {
  auto h = MakeRecurrentHarness();
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>(1, 0.0F),
                           std::vector<float>{1.0F, 2.0F, 3.0F, 4.0F}};

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  // Activation zeroes the state, so the FIRST step must have run on zeros.
  ASSERT_EQ(h.engine->last_inputs.size(), 2U);
  for (const float v : h.engine->last_inputs[1]) {
    EXPECT_FLOAT_EQ(v, 0.0F) << "activation must reset the state before the first evaluation";
  }

  // The second step sees what the first emitted.
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  EXPECT_FLOAT_EQ(h.engine->last_inputs[1][0], 1.0F);
  EXPECT_FLOAT_EQ(h.engine->last_inputs[1][3], 4.0F);
}

TEST(DemoInferenceRecurrent, ANonFiniteStateIsZeroedRatherThanFrozen) {
  // #511 C-4. Freezing a NaN would make every later step non-finite, every tick
  // hold, and the hold path is silent — the robot would stop with nothing in the
  // log and no recovery short of re-activation.
  auto h = MakeRecurrentHarness();
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());

  auto state = MakeState();
  h.engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>(1, 0.0F),
                           std::vector<float>{5.0F, 6.0F, 7.0F, 8.0F}};
  static_cast<void>(h.ctrl->Compute(state));
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FLOAT_EQ(h.engine->last_inputs[1][0], 5.0F) << "state is flowing before the fault";

  // The action stays perfectly valid; only the STATE head goes bad.
  h.engine->next_output[2][1] = std::numeric_limits<float>::quiet_NaN();
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_FALSE(h.ctrl->LastTickHeldForTesting())
      << "a bad state must not reject the action that was already accepted";

  static_cast<void>(h.ctrl->Compute(state));
  for (const float v : h.engine->last_inputs[1]) {
    EXPECT_FLOAT_EQ(v, 0.0F) << "the whole state tensor must be reset, not partially copied";
  }
}

TEST(DemoInferenceRecurrent, ARejectedActionDoesNotAdvanceTheState) {
  // #511 C-5. The `ok` gate already means "the whole action was accepted", and
  // the feedback sits behind it: advancing the policy's memory on a step whose
  // command was thrown away would keep its story moving while the robot stood
  // still, and every later action would be finite and about a history that did
  // not happen.
  auto h = MakeRecurrentHarness();
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>(1, 0.0F),
                           std::vector<float>{2.0F, 2.0F, 2.0F, 2.0F}};

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FLOAT_EQ(h.engine->last_inputs[1][0], 2.0F);

  // The ARM head goes bad while the state head offers a perfectly finite new
  // value. The action is rejected; the state must not take that value.
  h.engine->next_output[0][3] = std::numeric_limits<float>::quiet_NaN();
  h.engine->next_output[2] = std::vector<float>{6.0F, 6.0F, 6.0F, 6.0F};
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_TRUE(h.ctrl->LastTickHeldForTesting()) << "a non-finite arm target must hold";

  h.engine->next_output[0][3] = 0.0F;
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  EXPECT_FLOAT_EQ(h.engine->last_inputs[1][0], 2.0F)
      << "the state must still be the last one that produced an ACCEPTED action";
}

TEST(DemoInferenceRecurrent, ALongHoldResetsTheStateOnResume) {
  // dt is 2 ms and the threshold 6 ms, so four held ticks cross it.
  auto h = MakeRecurrentHarness("  reset_after_hold_sec: 0.006\n");
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>(1, 0.0F),
                           std::vector<float>{9.0F, 9.0F, 9.0F, 9.0F}};

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FLOAT_EQ(h.engine->last_inputs[1][0], 9.0F);

  state.devices[0].hole_mask = 0b10U;  // arm lane unreadable → hold
  for (int t = 0; t < 4; ++t) {
    static_cast<void>(h.ctrl->Compute(state));
  }
  ASSERT_TRUE(h.ctrl->LastTickHeldForTesting());

  state.devices[0].hole_mask = 0U;
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  for (const float v : h.engine->last_inputs[1]) {
    EXPECT_FLOAT_EQ(v, 0.0F) << "a state describing a moment that has passed must not resume";
  }
}

TEST(DemoInferenceRecurrent, AShortHoldKeepsTheState) {
  // The mirror of the case above, and the reason the policy is a threshold and
  // not "reset on every hold": one hole in a device lane is not rare, and
  // erasing memory on each would gut the recurrence.
  auto h = MakeRecurrentHarness("  reset_after_hold_sec: 0.5\n");
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>(1, 0.0F),
                           std::vector<float>{9.0F, 9.0F, 9.0F, 9.0F}};

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FLOAT_EQ(h.engine->last_inputs[1][0], 9.0F);

  state.devices[0].hole_mask = 0b10U;
  static_cast<void>(h.ctrl->Compute(state));  // 2 ms of hold, well under 500 ms
  ASSERT_TRUE(h.ctrl->LastTickHeldForTesting());

  state.devices[0].hole_mask = 0U;
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  EXPECT_FLOAT_EQ(h.engine->last_inputs[1][0], 9.0F) << "a transient hold must not erase memory";
}

TEST(DemoInferenceRecurrent, ANegativeThresholdNeverResetsOutsideActivation) {
  auto h = MakeRecurrentHarness("  reset_after_hold_sec: -1.0\n");
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>(1, 0.0F),
                           std::vector<float>{3.0F, 3.0F, 3.0F, 3.0F}};

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  static_cast<void>(h.ctrl->Compute(state));
  state.devices[0].hole_mask = 0b10U;
  for (int t = 0; t < 200; ++t) {  // 400 ms of hold
    static_cast<void>(h.ctrl->Compute(state));
  }
  state.devices[0].hole_mask = 0U;
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_FLOAT_EQ(h.engine->last_inputs[1][0], 3.0F);
}

TEST(DemoInferenceRecurrent, ZeroThresholdResetsAfterAnyHold) {
  auto h = MakeRecurrentHarness("  reset_after_hold_sec: 0.0\n");
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>(1, 0.0F),
                           std::vector<float>{4.0F, 4.0F, 4.0F, 4.0F}};

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FLOAT_EQ(h.engine->last_inputs[1][0], 4.0F);

  state.devices[0].hole_mask = 0b10U;
  static_cast<void>(h.ctrl->Compute(state));  // one held tick is enough
  state.devices[0].hole_mask = 0U;
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_FLOAT_EQ(h.engine->last_inputs[1][0], 0.0F);
}

TEST(DemoInferenceRecurrent, ReactivationResetsTheStateRegardlessOfTheThreshold) {
  // The gap across a deactivation is not a hold, and no threshold governs it.
  auto h = MakeRecurrentHarness("  reset_after_hold_sec: -1.0\n");
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>(1, 0.0F),
                           std::vector<float>{7.0F, 7.0F, 7.0F, 7.0F}};

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FLOAT_EQ(h.engine->last_inputs[1][0], 7.0F);

  ASSERT_EQ(h.ctrl->on_deactivate(rclcpp_lifecycle::State{}),
            rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  static_cast<void>(h.ctrl->Compute(state));
  for (const float v : h.engine->last_inputs[1]) {
    EXPECT_FLOAT_EQ(v, 0.0F);
  }
}

TEST(DemoInferenceRecurrent, TheStateTensorIsNotPackedWithObservationFeatures) {
  // The observation walk is flat over features and the state tensor declares
  // none, so nothing should reach it from PackObservation. Pinned because a
  // walk that had kept a policy-wide cursor would spill into it — and the
  // policy would read part of its observation as a hidden state.
  auto h = MakeRecurrentHarness();
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>(1, 0.0F),
                           std::vector<float>(4, 0.0F)};

  auto state = MakeState();
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 1.0 + i;
  }
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_EQ(h.engine->last_inputs[0].size(), static_cast<std::size_t>(kInputElements));
  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][0], 1.0F);
  for (const float v : h.engine->last_inputs[1]) {
    EXPECT_FLOAT_EQ(v, 0.0F);
  }
}

// ── Multi-input: one address space per tensor ──────────────────────────────

namespace {

/// Two input tensors. The arm/hand joints go in `obs` and the fingertip forces
/// in `aux`, so the second tensor's offsets restart at 0 — the property that
/// separates a real N-tensor walk from one that kept a policy-wide prefix sum.
std::string MakeTwoInputYaml(const std::string& aux_lane = "") {
  return R"(
command_type: "position"
topics:
  arm:
    subscribe:
      - topic: "arm/joint_goal"
        role: "target"
  hand:
    subscribe:
      - topic: "hand/joint_goal"
        role: "target"
inference:
  model_path: "fake_policy.onnx"
  decimation: 1
  inputs:
    - name: "obs"
      shape: [1, 16]
      features: ["arm.position", "hand.position"]
    - name: "aux"
      shape: [1, 4]
      features: ["hand.fingertip_force_norm"]
)" + aux_lane +
         R"(  outputs:
    - name: "arm_action"
      shape: [1, 6]
    - name: "posture"
      shape: [1, 1]
  output_features:
    - { tensor: "arm_action", role: "joint_target",   device: "arm" }
    - { tensor: "posture",    role: "posture_scalar", device: "hand" }
  hand_posture:
    open:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    close: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
)";
}

}  // namespace

TEST(DemoInferenceMultiInput, EachTensorIsPackedIntoItsOwnBuffer) {
  Harness h{MakeTwoInputYaml(), false, {16U, 4U}};
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());

  auto state = MakeState();
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 0.1 * (i + 1);
  }
  for (int i = 0; i < kHandDof; ++i) {
    state.devices[1].positions[static_cast<std::size_t>(i)] = -0.01 * (i + 1);
  }
  SetFingertipForce(state, 0, 3.0F, 4.0F, 0.0F);  // ‖f‖ = 5

  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  ASSERT_EQ(h.engine->last_inputs.size(), 2U);
  ASSERT_EQ(h.engine->last_inputs[0].size(), 16U);
  ASSERT_EQ(h.engine->last_inputs[1].size(), 4U);

  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][0], 0.1F);
  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][6], -0.01F) << "hand joints follow the arm in tensor 0";

  // The whole point: the force lane starts at 0 of ITS tensor, not at 16 of a
  // policy-wide flattening. A binding that kept the flat offset would write
  // past the 4-element buffer and hold forever instead.
  EXPECT_FLOAT_EQ(h.engine->last_inputs[1][0], 5.0F);
  for (int f = 1; f < kFingertips; ++f) {
    EXPECT_FLOAT_EQ(h.engine->last_inputs[1][static_cast<std::size_t>(f)], 0.0F);
  }
}

TEST(DemoInferenceMultiInput, TheAffineLaneNormalisesOnlyItsOwnTensor) {
  // `aux` scales by 2 and `obs` declares no lane at all. A binding that applied
  // one tensor's lane to every buffer would leave both tensors finite and in a
  // plausible range — the observation would simply be normalised with the wrong
  // constants, which is precisely the failure the schema is built to prevent.
  const std::string yaml =
      MakeTwoInputYaml("      offset: [0.0, 0.0, 0.0, 0.0]\n      scale: [2.0, 2.0, 2.0, 2.0]\n");
  Harness h{yaml, false, {16U, 4U}};
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());

  auto state = MakeState();
  state.devices[0].positions[0] = 0.5;
  SetFingertipForce(state, 0, 3.0F, 4.0F, 0.0F);  // ‖f‖ = 5

  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  ASSERT_EQ(h.engine->last_inputs.size(), 2U);
  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][0], 0.5F) << "tensor 0 declares no lane — identity";
  EXPECT_FLOAT_EQ(h.engine->last_inputs[1][0], 10.0F) << "tensor 1 scales by its own lane";
}

TEST(DemoInferenceMultiInput, HoldsWhenTheEngineDeclaresFewerTensorsThanTheSchema) {
  // The engine offers one tensor, the schema declares two. ONNX Runtime
  // allocates every declared input, so a binding that packed only what it could
  // reach would leave the policy observing an untouched allocation as if it
  // were this tick's robot.
  Harness h{MakeTwoInputYaml(), false, {16U}};
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
  EXPECT_EQ(h.engine->run_count, 0) << "the model must not run on a half-packed observation";
}

TEST(DemoInferenceConfig, RejectsAnEmptyModelPathWithoutTheOptIn) {
  Harness h{MakeYaml(10, "", /*allow_missing=*/false)};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceConfig, AcceptsAnEmptyModelPathWithTheOptIn) {
  Harness h{MakeYaml(10, "", /*allow_missing=*/true)};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
}

TEST(DemoInferenceConfig, RejectsAnEngineThatDidNotInitialise) {
  // The stub engine's Init() is a silent no-op, so without the
  // is_initialized() gate this configure would succeed and the controller would
  // hold position forever with nothing in the log to explain it.
  Harness h{MakeYaml(), /*stub_engine=*/true};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceConfig, RejectsAClosePostureOutsideTheJointBand) {
  // The concrete trap: poses_p1b.yaml's "close" is positive on joints whose
  // upper limit is 0, so it would be clamped back to open with no diagnostic.
  std::string yaml = MakeYaml();
  yaml.replace(
      yaml.find("close: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]"),
      std::string("close: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]").size(),
      "close: [0.79, 0.79, 0.79, 0.79, 0.79, 0.79, 0.79, 0.79, 0.79, 0.79]");
  Harness h{yaml};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

// ── Observation packing ─────────────────────────────────────────────────────

TEST(DemoInferenceObservation, PacksJointsAndForceMagnitudesInYamlOrder) {
  Harness h;
  auto state = MakeState();
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 0.1 * (i + 1);
  }
  for (int i = 0; i < kHandDof; ++i) {
    state.devices[1].positions[static_cast<std::size_t>(i)] = -0.01 * (i + 1);
  }
  SetFingertipForce(state, 0, 3.0F, 4.0F, 0.0F);  // ‖f‖ = 5

  static_cast<void>(h.ctrl->Compute(state));

  ASSERT_EQ(h.engine->last_inputs[0].size(), static_cast<std::size_t>(kInputElements));
  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][0], 0.1F);
  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][5], 0.6F);
  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][6], -0.01F);
  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][16], 5.0F)
      << "force feature must be the 3-axis magnitude";
}

TEST(DemoInferenceObservation, StaleFingertipGroupContributesZero) {
  // The recorded decision: "no contact" and "no reading" are the same
  // observation to this policy. Holding the last force instead would let a
  // dropped lane keep reporting a grasp that has ended.
  Harness h;
  auto state = MakeState();
  SetFingertipForce(state, 1, 0.0F, 0.0F, 9.0F);
  state.devices[1].inference_enable[1] = false;  // lane went stale

  static_cast<void>(h.ctrl->Compute(state));

  ASSERT_EQ(h.engine->last_inputs[0].size(), static_cast<std::size_t>(kInputElements));
  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][17], 0.0F);
}

// ── Decimation ──────────────────────────────────────────────────────────────

TEST(DemoInferenceDecimation, RunsExactlyOncePerDecimationTicks) {
  Harness h;
  auto state = MakeState();
  for (int t = 0; t < 100; ++t) {
    static_cast<void>(h.ctrl->Compute(state));
  }
  EXPECT_EQ(h.engine->run_count, 10) << "100 ticks at decimation 10 is 10 policy evaluations";
  EXPECT_EQ(h.ctrl->InferenceCountForTesting(), 10U);
}

TEST(DemoInferenceDecimation, HoldsTheActionBetweenEvaluations) {
  Harness h;
  auto state = MakeState();
  h.engine->next_output = {std::vector<float>(6, 0.5F), std::vector<float>{0.0F}};

  const auto first = h.ctrl->Compute(state);
  const double after_first = first.devices[0].target_positions[0];

  // Change what the engine WOULD return; the held ticks must not see it.
  h.engine->next_output[0].assign(6, -0.5F);
  for (int t = 1; t < 10; ++t) {
    const auto out = h.ctrl->Compute(state);
    EXPECT_EQ(h.engine->run_count, 1) << "tick " << t << " must not re-run the policy";
    EXPECT_DOUBLE_EQ(out.devices[0].target_positions[0], after_first);
  }
  static_cast<void>(h.ctrl->Compute(state));  // tick 11 — back on the grid
  EXPECT_EQ(h.engine->run_count, 2);
}

TEST(DemoInferenceDecimation, DecimationOfOneRunsEveryTick) {
  Harness h{MakeYaml(/*decimation=*/1)};
  auto state = MakeState();
  for (int t = 0; t < 5; ++t) {
    static_cast<void>(h.ctrl->Compute(state));
  }
  EXPECT_EQ(h.engine->run_count, 5);
}

// ── Hold paths ──────────────────────────────────────────────────────────────

TEST(DemoInferenceHold, HoldsWhenRunFails) {
  Harness h;
  auto state = MakeState();
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 0.25;
  }
  h.engine->run_result = false;

  const auto out = h.ctrl->Compute(state);
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
  for (int i = 0; i < kArmDof; ++i) {
    EXPECT_DOUBLE_EQ(out.devices[0].commands[static_cast<std::size_t>(i)], 0.25);
  }
}

TEST(DemoInferenceHold, AFailedRunDoesNotReplayTheStaleOutputBuffer) {
  // A failed Run() leaves the engine's output buffer holding SOMETHING — the
  // previous result, or whatever the partially-executed graph wrote. The return
  // value is the only thing separating a fresh action from that, so the buffer
  // is deliberately loaded with a new, attractive value here: if the controller
  // read it anyway, the arm would set off toward 0.9.
  //
  // The wait until tick 11 is not padding. At decimation 10 the ticks in
  // between do not call Run() at all, so a failure injected at tick 2 would
  // prove nothing; the next EVALUATION is where the contract lives.
  Harness h;
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  auto state = MakeState();
  h.engine->next_output = {std::vector<float>(6, 0.4F), std::vector<float>{0.0F}};

  static_cast<void>(h.ctrl->Compute(state));  // tick 1: evaluates, action = 0.4
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  ASSERT_EQ(h.engine->run_count, 1);

  for (int t = 2; t <= 10; ++t) {
    static_cast<void>(h.ctrl->Compute(state));  // held action, no evaluation
  }
  ASSERT_EQ(h.engine->run_count, 1) << "decimation must not have evaluated in between";

  h.engine->next_output[0].assign(6, 0.9F);
  h.engine->run_result = false;

  const auto out = h.ctrl->Compute(state);  // tick 11: evaluates, fails
  EXPECT_EQ(h.engine->run_count, 2);
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
  EXPECT_DOUBLE_EQ(out.devices[0].commands[0], 0.0)
      << "held at the measured position, not stepped toward the buffer's 0.9";

  // And the failure must not be papered over by replaying the last good action
  // for the rest of the decimation window.
  for (int t = 12; t <= 20; ++t) {
    const auto held = h.ctrl->Compute(state);
    EXPECT_TRUE(h.ctrl->LastTickHeldForTesting()) << "tick " << t;
    EXPECT_DOUBLE_EQ(held.devices[0].commands[0], 0.0);
  }
}

TEST(DemoInferenceHold, HoldsOnANonFiniteArmOutput) {
  Harness h;
  auto state = MakeState();
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 0.3;
  }
  h.engine->next_output = {std::vector<float>(6, 0.1F), std::vector<float>{0.0F}};
  h.engine->next_output[0][3] = std::numeric_limits<float>::quiet_NaN();

  const auto out = h.ctrl->Compute(state);
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
  // Whole-robot hold: joint 0 was finite but is held too, because half an
  // action is not a smaller version of the right one.
  EXPECT_DOUBLE_EQ(out.devices[0].commands[0], 0.3);
}

TEST(DemoInferenceHold, HoldsOnANonFinitePostureScalar) {
  Harness h;
  auto state = MakeState();
  h.engine->next_output = {std::vector<float>(6, 0.1F),
                           std::vector<float>{std::numeric_limits<float>::infinity()}};
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
}

TEST(DemoInferenceHold, HoldsWhenTheArmLaneHasAHole) {
  Harness h;
  auto state = MakeState();
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 0.2;
  }
  state.devices[0].hole_mask = 0b100U;  // slot 2 was not written this message

  const auto out = h.ctrl->Compute(state);
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
  EXPECT_EQ(h.engine->run_count, 0) << "the model must not be run on an unreadable observation";
  EXPECT_DOUBLE_EQ(out.devices[0].commands[0], 0.2);
}

TEST(DemoInferenceHold, HoldsWhenADeviceIsInvalid) {
  Harness h;
  auto state = MakeState();
  state.devices[1].valid = false;
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
  EXPECT_EQ(h.engine->run_count, 0);
}

TEST(DemoInferenceHold, HoldModeNeverRunsThePolicy) {
  // `allow_missing_model` with an empty path: the wiring comes up and every
  // tick holds, which is what lets the controller be brought up and smoke
  // tested before a policy file exists.
  Harness h{MakeYaml(10, "", /*allow_missing=*/true)};
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  auto state = MakeState();
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 0.15;
  }
  for (int t = 0; t < 25; ++t) {
    const auto out = h.ctrl->Compute(state);
    EXPECT_TRUE(h.ctrl->LastTickHeldForTesting()) << "tick " << t;
    EXPECT_DOUBLE_EQ(out.devices[0].commands[0], 0.15);
  }
  EXPECT_EQ(h.engine->run_count, 0);
}

// ── Output shaping ──────────────────────────────────────────────────────────

TEST(DemoInferenceOutput, BlendsThePostureBetweenOpenAndClose) {
  Harness h;
  auto state = MakeState();
  h.engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>{1.0F}};

  static_cast<void>(h.ctrl->Compute(state));
  const auto out = h.ctrl->Compute(state);
  // target_positions carries the unbounded policy action; commands carries the
  // rate-limited one. The action is what the blend produced.
  for (int i = 0; i < kHandDof; ++i) {
    EXPECT_DOUBLE_EQ(out.devices[1].target_positions[static_cast<std::size_t>(i)], -1.0);
  }
}

TEST(DemoInferenceOutput, ClampsAnOutOfRangeScalarInsteadOfHolding) {
  Harness h;
  auto state = MakeState();
  h.engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>{1.7F}};

  const auto out = h.ctrl->Compute(state);
  EXPECT_FALSE(h.ctrl->LastTickHeldForTesting()) << "a clamped scalar still ships a command";
  EXPECT_TRUE(h.ctrl->LastScalarClampedForTesting());
  EXPECT_DOUBLE_EQ(out.devices[1].target_positions[0], -1.0) << "clamped to the close posture";
}

TEST(DemoInferenceOutput, RateLimitsALargeArmStep) {
  // max_velocity 2.0 rad/s at dt = 2 ms ⇒ at most 4 mrad of motion per tick, no
  // matter how far away the policy's absolute target is.
  Harness h;
  auto state = MakeState();
  h.engine->next_output = {std::vector<float>(6, 3.0F), std::vector<float>{0.0F}};

  const auto out = h.ctrl->Compute(state);
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  EXPECT_NEAR(out.devices[0].commands[0], 0.004, 1e-9);
  EXPECT_DOUBLE_EQ(out.devices[0].target_positions[0], 3.0)
      << "the raw policy target stays visible for diagnostics";
}

TEST(DemoInferenceOutput, ClampsToThePositionBand) {
  // The hand's upper bound is 0.0 on every joint here — the same shape as the
  // real p1b, where a positive "close" posture would be pulled back to open.
  Harness h;
  auto state = MakeState();
  h.engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>{0.0F}};
  static_cast<void>(h.ctrl->Compute(state));
  const auto out = h.ctrl->Compute(state);
  for (int i = 0; i < kHandDof; ++i) {
    EXPECT_LE(out.devices[1].commands[static_cast<std::size_t>(i)], 0.0);
    EXPECT_GE(out.devices[1].commands[static_cast<std::size_t>(i)], -1.5708);
  }
}

TEST(DemoInferenceOutput, IdentityIsPositionCommandType) {
  Harness h;
  auto state = MakeState();
  const auto out = h.ctrl->Compute(state);
  EXPECT_EQ(out.command_type, rtc::CommandType::kPosition);
  EXPECT_EQ(h.ctrl->Name(), "DemoInferenceController");
}

// ── Object pose lane ────────────────────────────────────────────────────────

namespace {
constexpr std::size_t kObjectInputElements = 27;

/// Feed one TFMessage straight into the controller's callback. That is the
/// production entry point (the subscription lambda calls exactly this), so no
/// DDS round-trip is needed to exercise the matching and staleness rules.
Harness MakeObjectHarness(const std::string& yaml) {
  return Harness{yaml, /*stub_engine=*/false, {kObjectInputElements}};
}
}  // namespace

TEST(DemoInferenceObject, ExactlyOneMatchIsAcceptedAndPacked) {
  auto h = MakeObjectHarness(MakeObjectYaml());
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  h.ctrl->InjectObjectTransformsForTesting(MakeTfMessage({"pool_apple_object"}));

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  ASSERT_EQ(h.engine->last_inputs[0].size(), kObjectInputElements);
  // Layout: 6 arm + 10 hand + 4 force + 3 object position + 4 object quat.
  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][20], 1.0F);
  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][21], 2.0F);
  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][22], 3.0F);
  EXPECT_FLOAT_EQ(h.engine->last_inputs[0][26], 1.0F) << "quaternion w is the LAST element (xyzw)";
}

TEST(DemoInferenceObject, TwoMatchesAreRefusedRatherThanPickingTheFirst) {
  // The sim publishes every non-parked free body in one message. "Take the
  // first" would silently track a different object the day the scene gains one,
  // and every downstream number would stay plausible.
  auto h = MakeObjectHarness(MakeObjectYaml());
  h.ctrl->InjectObjectTransformsForTesting(
      MakeTfMessage({"pool_apple_object", "pool_duck_object"}));

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
  EXPECT_EQ(h.engine->run_count, 0);
}

TEST(DemoInferenceObject, APoseWithNoOrientationIsNotAPose) {
  // An all-zero quaternion is what a broken publisher sends, and normalising it
  // MANUFACTURES NaN — which then packs, runs, and only maybe gets caught at the
  // command screen. The lane must refuse it like any other unusable observation.
  auto h = MakeObjectHarness(MakeObjectYaml());
  auto msg = MakeTfMessage({"pool_apple_object"});
  msg.transforms[0].transform.rotation.x = 0.0;
  msg.transforms[0].transform.rotation.y = 0.0;
  msg.transforms[0].transform.rotation.z = 0.0;
  msg.transforms[0].transform.rotation.w = 0.0;
  h.ctrl->InjectObjectTransformsForTesting(msg);

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
  EXPECT_EQ(h.engine->run_count, 0) << "nothing may reach the engine";
}

TEST(DemoInferenceObject, ANonFinitePoseIsRefusedBeforeItIsPacked) {
  auto h = MakeObjectHarness(MakeObjectYaml());
  auto msg = MakeTfMessage({"pool_apple_object"});
  msg.transforms[0].transform.translation.y = std::numeric_limits<double>::quiet_NaN();
  h.ctrl->InjectObjectTransformsForTesting(msg);

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
  EXPECT_EQ(h.engine->run_count, 0);
}

TEST(DemoInferenceObject, NoMatchHolds) {
  auto h = MakeObjectHarness(MakeObjectYaml());
  h.ctrl->InjectObjectTransformsForTesting(MakeTfMessage({"table", "robot_base"}));
  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
}

TEST(DemoInferenceObject, HoldsBeforeAnyMessageArrives) {
  auto h = MakeObjectHarness(MakeObjectYaml());
  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
  EXPECT_EQ(h.engine->run_count, 0) << "never run the policy on an object pose we do not have";
}

TEST(DemoInferenceObject, GoesStaleAfterTheConfiguredTimeout) {
  // Age is accumulated in dt, so the budget is expressed on the simulation's
  // own time axis rather than a wall clock.
  //
  // The budget is deliberately 0.021 s and not 0.020 s: at dt = 2 ms the latter
  // puts the threshold exactly on an accumulated sum, where ten additions of
  // 0.002 land a few ulp above 0.02 and the verdict is decided by float noise
  // rather than by the rule. A full millisecond of margin on each side makes
  // this a test of the timeout and not of rounding.
  auto h = MakeObjectHarness(MakeObjectYaml(/*timeout_sec=*/0.021));
  h.ctrl->InjectObjectTransformsForTesting(MakeTfMessage({"pool_apple_object"}));
  auto state = MakeState();  // dt = 2 ms

  for (int t = 0; t <= 10; ++t) {  // ages 0 .. 0.020, all inside 0.021
    static_cast<void>(h.ctrl->Compute(state));
    EXPECT_FALSE(h.ctrl->LastTickHeldForTesting()) << "tick " << t << " is inside the budget";
  }
  static_cast<void>(h.ctrl->Compute(state));  // age 0.022 — past it
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting()) << "past timeout_sec the pose must go stale";

  // And it stays stale: nothing re-validates a pose that never arrived again.
  for (int t = 0; t < 5; ++t) {
    static_cast<void>(h.ctrl->Compute(state));
    EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
  }
}

TEST(DemoInferenceObject, AFreshMessageResetsTheAge) {
  auto h = MakeObjectHarness(MakeObjectYaml(/*timeout_sec=*/0.021));
  auto state = MakeState();
  for (int cycle = 0; cycle < 3; ++cycle) {
    h.ctrl->InjectObjectTransformsForTesting(MakeTfMessage({"pool_apple_object"}));
    for (int t = 0; t < 8; ++t) {
      static_cast<void>(h.ctrl->Compute(state));
      ASSERT_FALSE(h.ctrl->LastTickHeldForTesting()) << "cycle " << cycle << " tick " << t;
    }
  }
}

TEST(DemoInferenceObject, AReactivationDoesNotInheritThePreviousActivationsPose) {
  // Age is accrued in tick `dt` and the tick does not run while inactive, so a
  // deactivation gap costs the lane nothing: a pose from before it would come
  // back looking one tick old however long the publisher has been silent since.
  auto h = MakeObjectHarness(MakeObjectYaml(/*timeout_sec=*/0.021));
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.ctrl->InjectObjectTransformsForTesting(MakeTfMessage({"pool_apple_object"}));

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting()) << "the pose is fresh on this activation";

  ASSERT_EQ(h.ctrl->on_deactivate(rclcpp_lifecycle::State{}),
            rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());

  // The publisher stopped during the gap. The lane cannot vouch for what it
  // holds — the age it would report was measured against a different run — so
  // it must hold instead of handing the policy a pose of unknown age.
  for (int t = 0; t < 20; ++t) {
    static_cast<void>(h.ctrl->Compute(state));
    ASSERT_TRUE(h.ctrl->LastTickHeldForTesting()) << "tick " << t << " after re-activation";
  }
  EXPECT_EQ(h.ctrl->LastHoldReasonForTesting(), integrated_bringup::InferenceHoldReason::kObject)
      << "and it must say WHICH lane is missing, not fold into a generic hold";

  // It recovers the moment the publisher speaks again — the gate is freshness,
  // not a one-way door.
  h.ctrl->InjectObjectTransformsForTesting(MakeTfMessage({"pool_apple_object"}));
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_FALSE(h.ctrl->LastTickHeldForTesting());
}

TEST(DemoInferenceObject, TransformsInTheWrongSourceFrameAreIgnored) {
  auto h = MakeObjectHarness(MakeObjectYaml(0.02, "prefix", "pool_", /*source_frame_id=*/"world"));
  h.ctrl->InjectObjectTransformsForTesting(
      MakeTfMessage({"pool_apple_object"}, /*frame_id=*/"camera_optical"));
  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting())
      << "a pose in an unexpected frame is finite and plausible — and rotated";
}

TEST(DemoInferenceObject, ExactMatchModeDoesNotAcceptAPrefix) {
  auto h = MakeObjectHarness(
      MakeObjectYaml(0.02, /*match_mode=*/"exact", /*frame_match=*/"pool_apple_object"));
  h.ctrl->InjectObjectTransformsForTesting(MakeTfMessage({"pool_apple_object_marker"}));
  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());

  h.ctrl->InjectObjectTransformsForTesting(MakeTfMessage({"pool_apple_object"}));
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_FALSE(h.ctrl->LastTickHeldForTesting());
}

TEST(DemoInferenceObject, RejectsAZeroTimeout) {
  auto h = MakeObjectHarness(MakeObjectYaml(/*timeout_sec=*/0.0));
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceObject, RejectsAnUnknownMatchMode) {
  auto h = MakeObjectHarness(MakeObjectYaml(0.02, /*match_mode=*/"regex"));
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

// ── Hold is a LATCH, not a follower ─────────────────────────────────────────
//
// These exist because the sim caught what the fixture could not: every case
// above feeds a state whose measured positions never change, and against a
// static state "command the measured position" and "command the latched
// position" are indistinguishable. On the real ur5e_p1b the difference is a
// zero-stiffness follower that sags 0.2 rad under gravity.

TEST(DemoInferenceHoldLatch, HoldCommandsTheEntryPositionNotTheDriftingOne) {
  Harness h{MakeYaml(10, "", /*allow_missing=*/true)};  // hold mode: never runs a policy
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  auto state = MakeState();
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 0.5;
  }

  const auto first = h.ctrl->Compute(state);
  ASSERT_TRUE(h.ctrl->LastTickHeldForTesting());
  EXPECT_DOUBLE_EQ(first.devices[0].commands[0], 0.5);

  // Now the arm sags, exactly as gravity makes it: the measured position walks
  // away tick by tick. The command must NOT walk with it.
  double sag = 0.5;
  for (int t = 0; t < 50; ++t) {
    sag -= 0.002;
    for (int i = 0; i < kArmDof; ++i) {
      state.devices[0].positions[static_cast<std::size_t>(i)] = sag;
    }
    const auto out = h.ctrl->Compute(state);
    EXPECT_DOUBLE_EQ(out.devices[0].commands[0], 0.5)
        << "tick " << t << ": the hold followed the measurement instead of latching it";
  }
  EXPECT_LT(sag, 0.41) << "the fixture must actually have moved, or this proves nothing";
}

TEST(DemoInferenceHoldLatch, AnAcceptedActionClearsTheLatch) {
  Harness h;  // real (fake) policy, decimation 10
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  auto state = MakeState();
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 0.2;
  }
  h.engine->run_result = false;  // force a hold on the first evaluation
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_TRUE(h.ctrl->LastTickHeldForTesting());

  // Policy recovers; the arm must resume from the measurement, not stay pinned
  // to the latched value.
  h.engine->run_result = true;
  h.engine->next_output = {std::vector<float>(6, 1.0F), std::vector<float>{0.0F}};
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 0.30;
  }
  const auto out = h.ctrl->Compute(state);
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  EXPECT_GT(out.devices[0].commands[0], 0.30) << "must move toward the policy target from 0.30";
  EXPECT_NEAR(out.devices[0].commands[0], 0.304, 1e-9) << "one tick of the 2 rad/s rate bound";
}

TEST(DemoInferenceHoldLatch, ALatchEnteredWhileUnreadableUsesTheLastGoodState) {
  // A hold entered BECAUSE the device went unreadable must not latch the
  // unreadable reading — that is the one number known to be untrustworthy.
  Harness h;
  auto state = MakeState();
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 0.25;
  }
  static_cast<void>(h.ctrl->Compute(state));  // readable: records 0.25
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());

  state.devices[0].hole_mask = 0b1000U;  // slot 3 stale
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 9.9;  // garbage behind the hole
  }
  const auto out = h.ctrl->Compute(state);
  ASSERT_TRUE(h.ctrl->LastTickHeldForTesting());
  EXPECT_DOUBLE_EQ(out.devices[0].commands[0], 0.25)
      << "latched from the last readable state, not from the unreadable one";
}

// ── By-name placement, fill, constant, joint convention (§5 criterion 2) ────
//
// The export this binding was extended for lists its joint table in its own
// order, fills 16 of 46 slots, and defines some joint axes opposite to the
// robot's URDF. The fixture below reproduces all three with the roster fixture:
// the tensor lists the hand BEFORE the arm and each of them BACKWARDS, so
// "packed by position" and "packed by name" cannot coincide, and the
// convention flips one hand joint and offsets one arm joint.

namespace {

constexpr float kPad = -7.0F;     // the filler every uncovered slot must hold
constexpr float kPoison = 99.0F;  // what a stale buffer holds before packing
constexpr double kArmJ1Offset = 0.25;

std::string MakeNamedYaml(const std::string& inference_extra = "") {
  return R"(
command_type: "position"
topics:
  arm:
    subscribe:
      - topic: "arm/joint_goal"
        role: "target"
  hand:
    subscribe:
      - topic: "hand/joint_goal"
        role: "target"
inference:
  model_path: "fake_policy.onnx"
  decimation: 1
  joint_convention:
    hand_j2: { sign: -1 }
    arm_j1: { offset: 0.25 }
)" + inference_extra +
         R"(  inputs:
    - name: "joint_pos"
      shape: [1, 20]
      element_names: [pad_a, hand_j9, hand_j8, hand_j7, hand_j6, hand_j5, hand_j4, hand_j3,
                      hand_j2, hand_j1, hand_j0, arm_j5, arm_j4, arm_j3, arm_j2, arm_j1, arm_j0,
                      pad_b, pad_c, pad_d]
      features: ["arm.position", "hand.position"]
      fill: [-7.0]
    - name: "joint_vel"
      shape: [1, 16]
      element_names: [hand_j0, hand_j1, hand_j2, hand_j3, hand_j4, hand_j5, hand_j6, hand_j7,
                      hand_j8, hand_j9, arm_j0, arm_j1, arm_j2, arm_j3, arm_j4, arm_j5]
      features: ["arm.velocity", "hand.velocity"]
    - name: "thumb_force"
      shape: [1, 1, 1, 3]
      features: ["hand.ft0.force_norm"]
      fill: [0.0]
    - name: "root_quat"
      shape: [1, 4]
      source: constant
      values: [0.0, 0.0, 0.0, 1.0]
    - name: "arm_in"
      shape: [1, 6]
      source: recurrent
      seed: "arm.position"
    - name: "syn_in"
      shape: [1, 1]
      source: recurrent
  outputs:
    - name: "arm_action"
      shape: [1, 6]
      element_names: [arm_j0, arm_j1, arm_j2, arm_j3, arm_j4, arm_j5]
    - name: "hand_action"
      shape: [1, 10]
      element_names: [hand_j3, hand_j7, hand_j0, hand_j9, hand_j1, hand_j5, hand_j8, hand_j2,
                      hand_j6, hand_j4]
    - { name: "arm_out", shape: [1, 6], feeds: "arm_in" }
    - { name: "syn_out", shape: [1, 1], feeds: "syn_in" }
    - { name: "gain", shape: [1, 6] }
  output_features:
    - { tensor: "arm_action",  role: "joint_target", device: "arm" }
    - { tensor: "hand_action", role: "joint_target", device: "hand" }
)";
}

const std::vector<std::size_t> kNamedIn = {20, 16, 3, 4, 6, 1};
const std::vector<std::size_t> kNamedOut = {6, 10, 6, 1, 6};

enum NamedIn : std::size_t { kJointPos, kJointVel, kThumbForce, kRootQuat, kArmIn, kSynIn };

enum NamedOut : std::size_t { kArmAction, kHandAction, kArmOut, kSynOut, kGain };

Harness MakeNamedHarness(const std::string& extra = "") {
  return Harness{MakeNamedYaml(extra), false, kNamedIn, kNamedOut};
}

/// Position of @p name in a tensor's element_names — the test addresses the
/// tensor the way the policy does, by name, never by an index literal.
std::size_t IndexOf(const std::vector<std::string>& names, const std::string& name) {
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i] == name) {
      return i;
    }
  }
  ADD_FAILURE() << "no element named '" << name << "'";
  return names.size();
}

/// Distinct per-joint positions and velocities, so a swap shows up as a value.
ControllerState MakeDistinctState() {
  auto s = MakeState();
  for (int i = 0; i < kArmDof; ++i) {
    s.devices[0].positions[static_cast<std::size_t>(i)] = 0.1 * (i + 1);
    s.devices[0].velocities[static_cast<std::size_t>(i)] = 1.0 * (i + 1);
  }
  for (int i = 0; i < kHandDof; ++i) {
    s.devices[1].positions[static_cast<std::size_t>(i)] = -0.01 * (i + 1);
    s.devices[1].velocities[static_cast<std::size_t>(i)] = 0.5 * (i + 1);
  }
  return s;
}

}  // namespace

TEST(DemoInferenceNamed, PlacesJointLanesByNameThroughTheConvention) {
  auto h = MakeNamedHarness();
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.engine->PoisonInputs(kPoison);

  const auto state = MakeDistinctState();
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());

  const auto& names = h.ctrl->IoParamsForTesting().inputs[kJointPos].element_names;
  const auto& pos = h.engine->last_inputs[kJointPos];
  for (int i = 0; i < kArmDof; ++i) {
    const double expect = 0.1 * (i + 1) + (i == 1 ? kArmJ1Offset : 0.0);
    EXPECT_FLOAT_EQ(pos[IndexOf(names, "arm_j" + std::to_string(i))], static_cast<float>(expect))
        << "arm_j" << i;
  }
  for (int i = 0; i < kHandDof; ++i) {
    const double expect = -0.01 * (i + 1) * (i == 2 ? -1.0 : 1.0);
    EXPECT_FLOAT_EQ(pos[IndexOf(names, "hand_j" + std::to_string(i))], static_cast<float>(expect))
        << "hand_j" << i;
  }
  for (const char* pad : {"pad_a", "pad_b", "pad_c", "pad_d"}) {
    EXPECT_FLOAT_EQ(pos[IndexOf(names, pad)], kPad)
        << pad << ": an uncovered slot must hold the filler, not what the buffer held";
  }
}

TEST(DemoInferenceNamed, AVelocityTakesTheSignButNotTheOffset) {
  // The offset is a position datum; a velocity has none to shift.
  auto h = MakeNamedHarness();
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  static_cast<void>(h.ctrl->Compute(MakeDistinctState()));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());

  const auto& names = h.ctrl->IoParamsForTesting().inputs[kJointVel].element_names;
  const auto& vel = h.engine->last_inputs[kJointVel];
  EXPECT_FLOAT_EQ(vel[IndexOf(names, "arm_j1")], 2.0F);
  EXPECT_FLOAT_EQ(vel[IndexOf(names, "arm_j4")], 5.0F);
  EXPECT_FLOAT_EQ(vel[IndexOf(names, "hand_j2")], -1.5F);
  EXPECT_FLOAT_EQ(vel[IndexOf(names, "hand_j9")], 5.0F);
}

TEST(DemoInferenceNamed, AnUnreadableVelocityLaneHolds) {
  // The position gate says nothing about velocities (#446). A driver that omits
  // them leaves zeros — a robot at rest, as far as the policy could tell.
  auto h = MakeNamedHarness();
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  auto state = MakeDistinctState();
  state.devices[1].velocity_hole_mask = 0b100U;
  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_TRUE(h.ctrl->LastTickHeldForTesting());
  EXPECT_EQ(h.engine->run_count, 0);
}

TEST(DemoInferenceNamed, AGroupForceNormLandsInTheFirstSlotAndTheRestIsFill) {
  auto h = MakeNamedHarness();
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.engine->PoisonInputs(kPoison);
  auto state = MakeDistinctState();
  SetFingertipForce(state, 0, 3.0F, 0.0F, 4.0F);    // ‖f‖ = 5
  SetFingertipForce(state, 1, 30.0F, 0.0F, 40.0F);  // another group — must not leak in
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  const auto& f = h.engine->last_inputs[kThumbForce];
  EXPECT_FLOAT_EQ(f[0], 5.0F);
  EXPECT_FLOAT_EQ(f[1], 0.0F);
  EXPECT_FLOAT_EQ(f[2], 0.0F);
}

TEST(DemoInferenceNamed, AConstantTensorIsRewrittenEveryEvaluation) {
  auto h = MakeNamedHarness();
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.engine->PoisonInputs(kPoison);
  static_cast<void>(h.ctrl->Compute(MakeDistinctState()));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  const auto& q = h.engine->last_inputs[kRootQuat];
  EXPECT_FLOAT_EQ(q[0], 0.0F);
  EXPECT_FLOAT_EQ(q[1], 0.0F);
  EXPECT_FLOAT_EQ(q[2], 0.0F);
  EXPECT_FLOAT_EQ(q[3], 1.0F);
}

// ── Named heads (§5 criterion 4) ────────────────────────────────────────────

TEST(DemoInferenceNamed, ANamedHeadIsGatheredIntoDeviceOrderAndTheConventionUndone) {
  auto h = MakeNamedHarness();
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());

  const auto& io = h.ctrl->IoParamsForTesting();
  const auto& hand_names = io.outputs[kHandAction].element_names;
  std::vector<float> hand_head(kNamedOut[kHandAction]);
  for (std::size_t k = 0; k < hand_head.size(); ++k) {
    hand_head[k] = -0.1F * static_cast<float>(k + 1);  // distinct per EXPORT position
  }
  std::vector<float> arm_head(kNamedOut[kArmAction]);
  for (std::size_t k = 0; k < arm_head.size(); ++k) {
    arm_head[k] = 0.05F * static_cast<float>(k + 1);
  }
  h.engine->next_output = {
      arm_head, hand_head, std::vector<float>(6, 0.0F), {0.0F}, std::vector<float>(6, 111.0F)};

  const auto state = MakeDistinctState();
  const auto out = h.ctrl->Compute(state);
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());

  for (int j = 0; j < kHandDof; ++j) {
    const auto k = IndexOf(hand_names, "hand_j" + std::to_string(j));
    const double sign = (j == 2) ? -1.0 : 1.0;
    EXPECT_DOUBLE_EQ(out.devices[1].target_positions[static_cast<std::size_t>(j)],
                     sign * static_cast<double>(hand_head[k]))
        << "hand_j" << j << " must come from the element NAMED after it, sign undone";
  }
  for (int j = 0; j < kArmDof; ++j) {
    const double offset = (j == 1) ? kArmJ1Offset : 0.0;
    EXPECT_DOUBLE_EQ(out.devices[0].target_positions[static_cast<std::size_t>(j)],
                     static_cast<double>(arm_head[static_cast<std::size_t>(j)]) - offset)
        << "arm_j" << j;
  }

  // Tensors that are declared but not commands must not reach a command.
  h.engine->next_output[kGain].assign(6, -222.0F);
  h.engine->next_output[kSynOut] = {1.5F};
  const auto again = h.ctrl->Compute(state);
  for (int j = 0; j < kHandDof; ++j) {
    EXPECT_DOUBLE_EQ(again.devices[1].target_positions[static_cast<std::size_t>(j)],
                     out.devices[1].target_positions[static_cast<std::size_t>(j)]);
  }
  for (int j = 0; j < kArmDof; ++j) {
    EXPECT_DOUBLE_EQ(again.devices[0].target_positions[static_cast<std::size_t>(j)],
                     out.devices[0].target_positions[static_cast<std::size_t>(j)]);
  }
}

TEST(DemoInferenceNamed, RejectsANamedHeadThatMissesADeviceJoint) {
  std::string yaml = MakeNamedYaml();
  yaml.replace(yaml.find("hand_j6, hand_j4]"), std::string("hand_j6, hand_j4]").size(),
               "hand_j6, hand_jX]");
  Harness h{yaml, false, kNamedIn, kNamedOut};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE)
      << "a device joint the head does not name would keep whatever it last held";
}

// ── Recurrent seed (§5 criterion 5) ─────────────────────────────────────────

TEST(DemoInferenceNamed, TheIntegratorIsSeededFromTheMeasuredJointsAndReseededAfterALongHold) {
  auto h = MakeNamedHarness("  reset_after_hold_sec: 0.006\n");
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  h.engine->next_output = {std::vector<float>(6, 0.0F),
                           std::vector<float>(10, 0.0F),
                           std::vector<float>(6, 9.0F),
                           {0.75F},
                           std::vector<float>(6, 0.0F)};

  auto state = MakeDistinctState();
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  for (int i = 0; i < kArmDof; ++i) {
    const double expect = 0.1 * (i + 1) + (i == 1 ? kArmJ1Offset : 0.0);
    EXPECT_FLOAT_EQ(h.engine->last_inputs[kArmIn][static_cast<std::size_t>(i)],
                    static_cast<float>(expect))
        << "the first step's integrator is the measured arm (policy convention), not zero";
  }
  EXPECT_FLOAT_EQ(h.engine->last_inputs[kSynIn][0], 0.0F) << "an unseeded state starts at zero";

  static_cast<void>(h.ctrl->Compute(state));
  EXPECT_FLOAT_EQ(h.engine->last_inputs[kArmIn][0], 9.0F) << "then the feedback owns it";
  EXPECT_FLOAT_EQ(h.engine->last_inputs[kSynIn][0], 0.75F);

  // A long hold while the arm moves: the resumed integrator must describe the
  // arm as it is now, not where it was nor zero.
  state.devices[0].hole_mask = 0b1U;
  for (int t = 0; t < 4; ++t) {
    static_cast<void>(h.ctrl->Compute(state));
  }
  ASSERT_TRUE(h.ctrl->LastTickHeldForTesting());
  state.devices[0].hole_mask = 0U;
  for (int i = 0; i < kArmDof; ++i) {
    state.devices[0].positions[static_cast<std::size_t>(i)] = 0.7;
  }
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  EXPECT_FLOAT_EQ(h.engine->last_inputs[kArmIn][0], 0.7F);
  EXPECT_FLOAT_EQ(h.engine->last_inputs[kArmIn][1], static_cast<float>(0.7 + kArmJ1Offset));
  EXPECT_FLOAT_EQ(h.engine->last_inputs[kSynIn][0], 0.0F);
}

TEST(DemoInferenceNamed, ASeedIsTakenOnTheStepThatActuallyRuns) {
  // The reset stays armed until an action is accepted, so an attempt that holds
  // does not leave the next attempt seeded from a measurement it never used.
  auto h = MakeNamedHarness();
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(h.Activate());
  auto state = MakeDistinctState();
  h.engine->run_result = false;
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_TRUE(h.ctrl->LastTickHeldForTesting());

  h.engine->run_result = true;
  state.devices[0].positions[0] = 0.33;
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  EXPECT_FLOAT_EQ(h.engine->last_inputs[kArmIn][0], 0.33F);
}

// ── Command tail base = previous command (§5 criterion 7, D2) ───────────────

TEST(DemoInferenceOutput, TheCommandAdvancesFromThePreviousCommandNotTheMeasurement) {
  // A servo that lags its command must not be throttled by its own lag. With a
  // measured base the command could lead the joint by one tick of v_max at
  // most, so a joint that has not moved yet would be commanded 4 mrad ahead
  // forever. Here the joint never moves at all.
  Harness h;
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  auto state = MakeState();  // measured 0.0 on every tick
  h.engine->next_output = {std::vector<float>(6, 3.0F), std::vector<float>{0.0F}};
  for (int k = 1; k <= 20; ++k) {
    const auto out = h.ctrl->Compute(state);
    ASSERT_FALSE(h.ctrl->LastTickHeldForTesting()) << "tick " << k;
    EXPECT_NEAR(out.devices[0].commands[0], 0.004 * k, 1e-9) << "tick " << k;
  }
}

TEST(DemoInferenceOutput, AfterAHoldTheCommandRestartsFromTheMeasurement) {
  // Neither the lead the command had built up nor the hold latch: a joint that
  // drifted during the hold is not dragged back to where the hold began.
  Harness h;
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  auto state = MakeState();
  h.engine->next_output = {std::vector<float>(6, 3.0F), std::vector<float>{0.0F}};
  for (int k = 1; k <= 10; ++k) {
    static_cast<void>(h.ctrl->Compute(state));
  }
  state.devices[0].hole_mask = 0b1U;
  const auto held = h.ctrl->Compute(state);
  ASSERT_TRUE(h.ctrl->LastTickHeldForTesting());
  EXPECT_DOUBLE_EQ(held.devices[0].commands[0], 0.0) << "latched at the last readable 0.0";

  state.devices[0].hole_mask = 0U;
  state.devices[0].positions[0] = 0.02;
  const auto out = h.ctrl->Compute(state);
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  EXPECT_NEAR(out.devices[0].commands[0], 0.024, 1e-9);
}

// ── Refusals: legacy keys, convention, reach gate, frames ──────────────────

namespace {

/// LoadConfig must throw, and say so with @p needle — "it threw" alone cannot
/// tell the migration message from an unrelated parse failure.
void ExpectLoadConfigRefuses(const std::string& yaml, const std::string& needle) {
  DemoInferenceController ctrl(
      "", std::make_unique<FakeEngine>(std::vector<std::size_t>{kInputElements},
                                       std::vector<std::size_t>{6, 1}));
  try {
    ctrl.LoadConfig(YAML::Load(yaml));
    ADD_FAILURE() << "LoadConfig accepted a config carrying '" << needle << "'";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find(needle), std::string::npos) << e.what();
  }
}

std::string WithInference(const std::string& yaml, const std::string& lines) {
  std::string out = yaml;
  const std::string anchor = "inference:\n";
  out.insert(out.find(anchor) + anchor.size(), lines);
  return out;
}

}  // namespace

TEST(DemoInferenceConfig, RejectsTheLegacyFrameKeysNamingTheMigration) {
  ExpectLoadConfigRefuses(WithInference(MakeYaml(), "  palm:\n    link: \"l_palm_link\"\n"),
                          "inference.palm is gone");
  ExpectLoadConfigRefuses(
      WithInference(MakeYaml(), "  base_pose_in_world:\n    rpy: [0.0, 0.0, 3.14]\n"),
      "base_pose_in_world is gone");
  ExpectLoadConfigRefuses(
      WithInference(MakeYaml(), "  object_pose:\n    reference_frame: \"base\"\n"),
      "reference_frame is gone");
}

TEST(DemoInferenceConfig, RejectsAConventionForAJointNeitherDeviceHas) {
  std::string yaml = MakeNamedYaml();
  yaml.replace(yaml.find("hand_j2: { sign: -1 }"), std::string("hand_j2: { sign: -1 }").size(),
               "hand_j99: { sign: -1 }");
  Harness h{yaml, false, kNamedIn, kNamedOut};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceConfig, RejectsAConventionSignThatIsAScale) {
  std::string yaml = MakeNamedYaml();
  yaml.replace(yaml.find("hand_j2: { sign: -1 }"), std::string("hand_j2: { sign: -1 }").size(),
               "hand_j2: { sign: -2 }");
  Harness h{yaml, false, kNamedIn, kNamedOut};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceConfig, RejectsAnUnknownConventionKey) {
  std::string yaml = MakeNamedYaml();
  yaml.replace(yaml.find("hand_j2: { sign: -1 }"), std::string("hand_j2: { sign: -1 }").size(),
               "hand_j2: { sign: -1, scale: 2.0 }");
  Harness h{yaml, false, kNamedIn, kNamedOut};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceConfig, RejectsAReachPhaseWithoutAGate) {
  std::string yaml = MakeYaml();
  yaml.replace(yaml.find("shape: [1, 20]"), std::string("shape: [1, 20]").size(), "shape: [1, 21]");
  yaml.replace(yaml.find("        - \"hand.fingertip_force_norm\"\n"),
               std::string("        - \"hand.fingertip_force_norm\"\n").size(),
               "        - \"hand.fingertip_force_norm\"\n        - \"reach.phase\"\n");
  Harness h{yaml, false, {21U}};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceConfig, RejectsAGateThatNoFeatureReads) {
  const std::string yaml = WithInference(MakeYaml(),
                                         "  reach_gate:\n"
                                         "    tips:\n"
                                         "      - { link: \"a\", force_group: \"ft0\", "
                                         "contact_obj: [0.0, 0.0, 0.0] }\n"
                                         "      - { link: \"b\", force_group: \"ft1\", "
                                         "contact_obj: [0.0, 0.0, 0.0] }\n");
  Harness h{yaml};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE)
      << "a block of trained constants that looks configured and does nothing";
}

TEST(DemoInferenceConfig, RejectsAnObjectFeatureWithoutAPolicyFrame) {
  std::string yaml = MakeObjectYaml();
  yaml.replace(yaml.find("  policy_frame: \"world_link\"\n"),
               std::string("  policy_frame: \"world_link\"\n").size(), "");
  Harness h{yaml, false, {27U}};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceConfig, RejectsAnObjectLaneThatDoesNotNameItsUrdfFrame) {
  std::string yaml = MakeObjectYaml();
  yaml.replace(yaml.find("    source_frame_link: \"world_link\"\n"),
               std::string("    source_frame_link: \"world_link\"\n").size(), "");
  Harness h{yaml, false, {27U}};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceConfig, RejectsALinkFeatureWithNoSystemUrdf) {
  std::string yaml = WithInference(MakeYaml(), "  policy_frame: \"base_link\"\n");
  yaml.replace(yaml.find("shape: [1, 20]"), std::string("shape: [1, 20]").size(), "shape: [1, 23]");
  yaml.replace(
      yaml.find("        - \"hand.fingertip_force_norm\"\n"),
      std::string("        - \"hand.fingertip_force_norm\"\n").size(),
      "        - \"hand.fingertip_force_norm\"\n        - \"link.l_palm_link.position\"\n");
  Harness h{yaml, false, {23U}};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

// ── model_path expansion (D4) ───────────────────────────────────────────────

namespace {

/// Sets or clears one environment variable for the life of a case.
class ScopedEnv {
 public:
  ScopedEnv(const char* name, const char* value) : name_(name) {
    if (const char* old = std::getenv(name)) {
      had_ = true;
      old_ = old;
    }
    if (value != nullptr) {
      ::setenv(name, value, 1);
    } else {
      ::unsetenv(name);
    }
  }

  ~ScopedEnv() {
    if (had_) {
      ::setenv(name_.c_str(), old_.c_str(), 1);
    } else {
      ::unsetenv(name_.c_str());
    }
  }

  ScopedEnv(const ScopedEnv&) = delete;
  ScopedEnv& operator=(const ScopedEnv&) = delete;

 private:
  std::string name_;
  std::string old_;
  bool had_{false};
};

}  // namespace

TEST(DemoInferenceModelPath, ExpandsAnEnvironmentVariable) {
  const ScopedEnv env("RTC_TEST_POLICY_DIR", "/opt/policies");
  Harness h{MakeYaml(10, "${RTC_TEST_POLICY_DIR}/policy.onnx")};
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  EXPECT_EQ(h.engine->last_model_path, "/opt/policies/policy.onnx");
  EXPECT_FALSE(h.ctrl->HoldModeForTesting());
}

TEST(DemoInferenceModelPath, AnUnsetVariableIsHoldModeWhenOptedIn) {
  const ScopedEnv env("RTC_TEST_POLICY_DIR", nullptr);
  Harness h{MakeYaml(10, "${RTC_TEST_POLICY_DIR}/policy.onnx", /*allow_missing=*/true)};
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(h.ctrl->HoldModeForTesting());
  EXPECT_EQ(h.engine->init_count, 0) << "no engine load is attempted without a path";
}

TEST(DemoInferenceModelPath, AnUnsetVariableFailsWithoutTheOptIn) {
  const ScopedEnv env("RTC_TEST_POLICY_DIR", nullptr);
  Harness h{MakeYaml(10, "${RTC_TEST_POLICY_DIR}/policy.onnx", /*allow_missing=*/false)};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceModelPath, ASetPathThatDoesNotLoadFailsEvenWhenOptedIn) {
  // The operator pointed at a file; a typo in it is not "no policy here".
  const ScopedEnv env("RTC_TEST_POLICY_DIR", "/opt/policies");
  Harness h{MakeYaml(10, "${RTC_TEST_POLICY_DIR}/policy.onnx", /*allow_missing=*/true),
            /*stub_engine=*/true};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
}

TEST(DemoInferenceModelPath, ExpandsALeadingTildeOnly) {
  const ScopedEnv home("HOME", "/home/tester");
  std::string out;
  std::string missing;
  ASSERT_TRUE(DemoInferenceController::ExpandModelPath("~/p/x.onnx", out, missing));
  EXPECT_EQ(out, "/home/tester/p/x.onnx");
  ASSERT_TRUE(DemoInferenceController::ExpandModelPath("/a/~b/x.onnx", out, missing));
  EXPECT_EQ(out, "/a/~b/x.onnx") << "a tilde inside a path is a file-name character";
}

TEST(DemoInferenceModelPath, NamesTheFirstMissingVariable) {
  const ScopedEnv a("RTC_TEST_A", "/x");
  const ScopedEnv b("RTC_TEST_B", nullptr);
  std::string out;
  std::string missing;
  EXPECT_FALSE(
      DemoInferenceController::ExpandModelPath("${RTC_TEST_A}/${RTC_TEST_B}/m.onnx", out, missing));
  EXPECT_EQ(missing, "RTC_TEST_B");
}

TEST(DemoInferenceModelPath, RefusesAMalformedReference) {
  std::string out;
  std::string missing;
  EXPECT_THROW(static_cast<void>(
                   DemoInferenceController::ExpandModelPath("${RTC_TEST_A/m.onnx", out, missing)),
               std::invalid_argument);
  EXPECT_THROW(
      static_cast<void>(DemoInferenceController::ExpandModelPath("${1BAD}/m.onnx", out, missing)),
      std::invalid_argument);
}

TEST(DemoInferenceConfig, RejectsARosterWiderThanTheFixedCapacity) {
  // OnDeviceConfigsSet keeps at most kMaxArmDof joints, but `arm.position` is as
  // wide as the WHOLE roster — packing it would ship a tail the tick never wrote.
  // Every OTHER check is satisfied on purpose (the arm head slices the clamped
  // DOF, the observation sums to the full roster), so only the capacity guard
  // stands between this config and a controller that configures and then
  // holds forever.
  constexpr int kWide = DemoInferenceController::kMaxArmDof + 1;
  auto cfgs = MakeDeviceConfigs();
  auto& arm = cfgs.at("arm");
  arm.joint_state_names.clear();
  for (int i = 0; i < kWide; ++i) {
    arm.joint_state_names.push_back("arm_j" + std::to_string(i));
  }
  std::string yaml = MakeYaml();
  const int obs = kWide + kHandDof + kFingertips;
  yaml.replace(yaml.find("shape: [1, 20]"), std::string("shape: [1, 20]").size(),
               "shape: [1, " + std::to_string(obs) + "]");
  yaml.replace(yaml.find("shape: [1, 6]"), std::string("shape: [1, 6]").size(),
               "shape: [1, " + std::to_string(DemoInferenceController::kMaxArmDof) + "]");
  auto ctrl = std::make_unique<DemoInferenceController>(
      "", std::make_unique<FakeEngine>(
              std::vector<std::size_t>{static_cast<std::size_t>(obs)},
              std::vector<std::size_t>{
                  static_cast<std::size_t>(DemoInferenceController::kMaxArmDof), 1}));
  rclcpp::NodeOptions opts;
  opts.use_global_arguments(false);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("inference_roster_cap", "", opts);
  const YAML::Node cfg = YAML::Load(yaml);
  ctrl->LoadConfig(cfg);
  ctrl->SetDeviceNameConfigs(cfgs);
  EXPECT_EQ(ctrl->on_configure(rclcpp_lifecycle::State{}, node, cfg),
            rtc::RTControllerInterface::CallbackReturn::FAILURE);
}
