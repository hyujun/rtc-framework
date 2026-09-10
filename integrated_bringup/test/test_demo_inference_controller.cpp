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
#include <limits>
#include <map>
#include <memory>
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

/// Deterministic stand-in for `rtc::OnnxEngine`.
///
/// Holds real buffers so the controller's pack/unpack path is the production
/// one; only the "model" is fake. `run_result` and the output values are
/// writable so a case can make a single tick fail without touching the
/// controller.
class FakeEngine final : public rtc::InferenceEngine {
 public:
  FakeEngine(std::size_t in_size, std::vector<std::size_t> out_sizes)
      : input_(in_size, 0.0F), out_sizes_(std::move(out_sizes)) {
    for (const auto n : out_sizes_) {
      outputs_.emplace_back(n, 0.0F);
    }
  }

  void Init(const rtc::ModelConfig& /*config*/) override {
    // `stub` reproduces exactly what a build without ONNX Runtime does: Init()
    // returns normally, throws nothing, and leaves is_initialized() false.
    initialized_ = !stub;
  }

  [[nodiscard]] bool Run() noexcept override {
    ++run_count;
    last_input = input_;
    for (std::size_t h = 0; h < outputs_.size(); ++h) {
      for (std::size_t i = 0; i < outputs_[h].size(); ++i) {
        outputs_[h][i] =
            (h < next_output.size() && i < next_output[h].size()) ? next_output[h][i] : 0.0F;
      }
    }
    return run_result;
  }

  // Single-input while the controller's schema is: index 0 is the only tensor,
  // and anything else answers nullptr/0 rather than folding onto slot 0 — a
  // controller reading a second tensor should hold, not silently re-read the
  // first.
  float* input_buffer(int /*model_idx*/, int input_idx) noexcept override {
    return (input_idx == 0) ? input_.data() : nullptr;
  }

  [[nodiscard]] const float* output_buffer(int /*model_idx*/,
                                           int output_idx) const noexcept override {
    const auto h = static_cast<std::size_t>(output_idx);
    return (h < outputs_.size()) ? outputs_[h].data() : nullptr;
  }

  [[nodiscard]] std::size_t input_size(int /*model_idx*/, int input_idx) const noexcept override {
    return (input_idx == 0) ? input_.size() : 0;
  }

  [[nodiscard]] int num_inputs(int /*model_idx*/) const noexcept override { return 1; }

  [[nodiscard]] std::size_t output_size(int /*model_idx*/, int output_idx) const noexcept override {
    const auto h = static_cast<std::size_t>(output_idx);
    return (h < outputs_.size()) ? outputs_[h].size() : 0;
  }

  [[nodiscard]] int num_outputs(int /*model_idx*/) const noexcept override {
    return static_cast<int>(outputs_.size());
  }

  [[nodiscard]] bool is_initialized() const noexcept override { return initialized_; }

  [[nodiscard]] int num_models() const noexcept override { return 1; }

  // ── Knobs ────────────────────────────────────────────────────────────────
  bool stub{false};
  bool run_result{true};
  int run_count{0};
  std::vector<std::vector<float>> next_output;
  std::vector<float> last_input;

 private:
  bool initialized_{false};
  std::vector<float> input_;
  std::vector<std::size_t> out_sizes_;
  std::vector<std::vector<float>> outputs_;
};

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
    - { name: "arm.target_position", tensor: "arm_action" }
    - { name: "hand.posture_scalar", tensor: "posture" }
  hand_posture:
    open:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    close: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
)";
}

/// A schema that adds the object pose lane (27 elements). Palm features are
/// deliberately absent: they need a real URDF, and the object lane's contract —
/// matching, staleness, frame checking — is independent of it.
std::string MakeObjectYaml(double timeout_sec = 0.02, const std::string& match_mode = "prefix",
                           const std::string& frame_match = "pool_",
                           const std::string& source_frame_id = "world",
                           const std::string& reference_frame = "world") {
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
    - { name: "arm.target_position", tensor: "arm_action" }
    - { name: "hand.posture_scalar", tensor: "posture" }
  base_pose_in_world:
    position: [0.0, 0.0, 0.0]
    rpy: [0.0, 0.0, 3.14159265358979]
  object_pose:
    topic: "/sim/object_transforms"
    match_mode: ")" +
         match_mode + R"("
    frame_match: ")" +
         frame_match + R"("
    source_frame_id: ")" +
         source_frame_id + R"("
    reference_frame: ")" +
         reference_frame + R"("
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

  explicit Harness(const std::string& yaml = MakeYaml(), bool stub_engine = false,
                   std::size_t input_elements = static_cast<std::size_t>(kInputElements)) {
    auto fake = std::make_unique<FakeEngine>(input_elements, std::vector<std::size_t>{6, 1});
    if (stub_engine) {
      fake->stub = true;
    }
    engine = fake.get();
    engine->next_output = {std::vector<float>(6, 0.0F), std::vector<float>(1, 0.0F)};
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
      "", std::make_unique<FakeEngine>(kInputElements, std::vector<std::size_t>{6, 1}));
  EXPECT_THROW(ctrl.LoadConfig(YAML::Load(yaml)), std::invalid_argument);
}

TEST(DemoInferenceConfig, RejectsMismatchedPostureLengths) {
  std::string yaml = MakeYaml();
  yaml.replace(
      yaml.find("close: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]"),
      std::string("close: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]").size(),
      "close: [-1.0, -1.0]");
  DemoInferenceController ctrl(
      "", std::make_unique<FakeEngine>(kInputElements, std::vector<std::size_t>{6, 1}));
  EXPECT_THROW(ctrl.LoadConfig(YAML::Load(yaml)), std::invalid_argument);
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
  yaml.replace(yaml.find("\"arm.target_position\", tensor: \"arm_action\""),
               std::string("\"arm.target_position\", tensor: \"arm_action\"").size(),
               "\"arm.target_position\", tensor: \"arm_action\", count: 5");
  Harness h{yaml};
  EXPECT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::FAILURE);
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

  ASSERT_EQ(h.engine->last_input.size(), static_cast<std::size_t>(kInputElements));
  EXPECT_FLOAT_EQ(h.engine->last_input[0], 0.1F);
  EXPECT_FLOAT_EQ(h.engine->last_input[5], 0.6F);
  EXPECT_FLOAT_EQ(h.engine->last_input[6], -0.01F);
  EXPECT_FLOAT_EQ(h.engine->last_input[16], 5.0F) << "force feature must be the 3-axis magnitude";
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

  ASSERT_EQ(h.engine->last_input.size(), static_cast<std::size_t>(kInputElements));
  EXPECT_FLOAT_EQ(h.engine->last_input[17], 0.0F);
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
  return Harness{yaml, /*stub_engine=*/false, kObjectInputElements};
}
}  // namespace

TEST(DemoInferenceObject, ExactlyOneMatchIsAcceptedAndPacked) {
  auto h = MakeObjectHarness(MakeObjectYaml());
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  h.ctrl->InjectObjectTransformsForTesting(MakeTfMessage({"pool_apple_object"}));

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  ASSERT_EQ(h.engine->last_input.size(), kObjectInputElements);
  // Layout: 6 arm + 10 hand + 4 force + 3 object position + 4 object quat.
  EXPECT_FLOAT_EQ(h.engine->last_input[20], 1.0F);
  EXPECT_FLOAT_EQ(h.engine->last_input[21], 2.0F);
  EXPECT_FLOAT_EQ(h.engine->last_input[22], 3.0F);
  EXPECT_FLOAT_EQ(h.engine->last_input[26], 1.0F) << "quaternion w is the LAST element (xyzw)";
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

TEST(DemoInferenceObject, BaseReferenceFrameAppliesTheMountingRotation) {
  // base_pose_in_world is a 180 deg yaw, so a world point (1, 2, 3) is
  // (-1, -2, 3) in base. Getting this backwards produces a pose that is
  // plausible everywhere except in the policy's reasoning.
  auto h = MakeObjectHarness(
      MakeObjectYaml(0.02, "prefix", "pool_", "world", /*reference_frame=*/"base"));
  ASSERT_EQ(h.configure_result, rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  h.ctrl->InjectObjectTransformsForTesting(MakeTfMessage({"pool_apple_object"}));

  auto state = MakeState();
  static_cast<void>(h.ctrl->Compute(state));
  ASSERT_FALSE(h.ctrl->LastTickHeldForTesting());
  ASSERT_EQ(h.engine->last_input.size(), kObjectInputElements);
  EXPECT_NEAR(h.engine->last_input[20], -1.0F, 1e-5);
  EXPECT_NEAR(h.engine->last_input[21], -2.0F, 1e-5);
  EXPECT_NEAR(h.engine->last_input[22], 3.0F, 1e-5) << "z is unchanged by a yaw";
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
