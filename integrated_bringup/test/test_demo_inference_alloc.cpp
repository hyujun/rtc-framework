// ── DemoInferenceController RT-1 zero-allocation gate ────────────────────────
//
// WHAT THIS GATE CAN AND CANNOT SEE, because the difference decides what the
// green below is worth.
//
//   ScopedAllocGate counts global `operator new`. It works across translation
//   units, so it does cover the REAL Compute() compiled into the
//   integrated_bringup library — that is a genuine measurement.
//
//   It does NOT see Eigen. Eigen's aligned_malloc calls std::malloc directly,
//   and the gate that catches that (ScopedNoMalloc) only reaches Eigen code
//   compiled in THIS translation unit. Compute() is in the library, so pointing
//   that gate at it would be fail-open: a guaranteed green proving nothing.
//
//   It does NOT see the real model either. Every case below injects FakeEngine,
//   so what is gated is the controller's own path AROUND inference, never ONNX
//   Runtime — and a real Run() DOES allocate, on every call, IoBinding or not.
//   test_demo_inference_real_model.cpp counts it against the real policy (local
//   only: it needs the policy file); a policy-step tick there allocates exactly
//   what Run() does, which is the cross-check that this suite's zero is the
//   whole of the controller's share.
//
// The consequence is stated rather than papered over. The roster-fixture cases
// cover the feature-packing, decimation, unpack, blend and command-tail path.
// The shipped-config cases add the real model — named placement, fill,
// constants, seeds, both velocity lanes, per-group forces, link poses through
// the combined-model cache AND the closed-chain projection, and the reach
// gate — and what they certify there is exactly "no global operator new": a
// std::vector or std::string temporary anywhere on that path would be caught,
// an Eigen dynamic-size temporary (std::malloc) would not.

// The positive control is not decoration. Without it every assertion here could
// be green because the gate was inert.

#include "integrated_bringup/controllers/demo_inference_controller.hpp"
#include "rtc_base/logging/thread_csv_producer.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "session_dir_test_fixture.hpp"
#include "shipped_config_test_fixture.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

using integrated_bringup::DemoInferenceController;
using rtc::ControllerState;

constexpr int kArmDof = 6;
constexpr int kHandDof = 10;
constexpr int kFingertips = 4;
constexpr std::size_t kInputElements = 20;
constexpr int kInferenceStride = 7;

/// Minimal engine: real buffers, no model. Its Run() must not allocate either,
/// since it is called from inside the gated region.
class FakeEngine final : public rtc::InferenceEngine {
 public:
  explicit FakeEngine(const std::vector<std::size_t>& in_sizes = {kInputElements},
                      const std::vector<std::size_t>& out_sizes = {6, 1}) {
    for (const auto n : in_sizes) {
      inputs_.emplace_back(n, 0.0F);
    }
    for (const auto n : out_sizes) {
      outputs_.emplace_back(n, 0.1F);
    }
  }

  void Init(const rtc::ModelConfig& /*config*/) override { initialized_ = true; }

  [[nodiscard]] bool Run() noexcept override { return true; }

  // Out-of-range answers nullptr/0 rather than folding onto slot 0, so a
  // controller that addressed a tensor this "model" does not have would hold
  // rather than silently re-read another one.
  float* input_buffer(int /*m*/, int input_idx) noexcept override {
    const auto t = static_cast<std::size_t>(input_idx);
    return (t < inputs_.size()) ? inputs_[t].data() : nullptr;
  }

  [[nodiscard]] const float* output_buffer(int /*m*/, int output_idx) const noexcept override {
    const auto h = static_cast<std::size_t>(output_idx);
    return (h < outputs_.size()) ? outputs_[h].data() : nullptr;
  }

  [[nodiscard]] std::size_t input_size(int /*m*/, int input_idx) const noexcept override {
    const auto t = static_cast<std::size_t>(input_idx);
    return (t < inputs_.size()) ? inputs_[t].size() : 0;
  }

  [[nodiscard]] std::size_t output_size(int /*m*/, int output_idx) const noexcept override {
    const auto h = static_cast<std::size_t>(output_idx);
    return (h < outputs_.size()) ? outputs_[h].size() : 0;
  }

  [[nodiscard]] int num_inputs(int /*m*/) const noexcept override {
    return static_cast<int>(inputs_.size());
  }

  [[nodiscard]] int num_outputs(int /*m*/) const noexcept override {
    return static_cast<int>(outputs_.size());
  }

  [[nodiscard]] bool is_initialized() const noexcept override { return initialized_; }

  [[nodiscard]] int num_models() const noexcept override { return 1; }

 private:
  bool initialized_{false};
  std::vector<std::vector<float>> inputs_;
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
  hand_lim.position_lower.assign(kHandDof, -1.5708);
  hand_lim.position_upper.assign(kHandDof, 0.0);
  hand_lim.max_velocity.assign(kHandDof, 10.0);
  hand.joint_limits = hand_lim;

  rtc::DeviceSensorLayout layout;
  layout.inference_values_per_group = kInferenceStride;
  hand.sensor_layout = layout;

  return {{"arm", arm}, {"hand", hand}};
}

constexpr const char* kYaml = R"(
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
  decimation: 10
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

/// The same observation split across TWO input tensors. Same features, same
/// widths, so any allocation the split introduces is the split's own — the
/// span table, the per-tensor buffer lookup and the per-tensor affine call.
constexpr const char* kTwoInputYaml = R"(
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
  decimation: 10
  inputs:
    - name: "obs"
      shape: [1, 16]
      features:
        - "arm.position"
        - "hand.position"
    - name: "aux"
      shape: [1, 4]
      features:
        - "hand.fingertip_force_norm"
      offset: [0.0, 0.0, 0.0, 0.0]
      scale:  [1.0, 1.0, 1.0, 1.0]
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

/// A recurrent schema: `h_out` feeds `h_in` every accepted step, so the gated
/// region below covers the reset check, the feedback copy and its finiteness
/// screen on top of everything the feed-forward cases already cover.
constexpr const char* kRecurrentYaml = R"(
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
  decimation: 10
  inputs:
    - name: "obs"
      shape: [1, 20]
      features:
        - "arm.position"
        - "hand.position"
        - "hand.fingertip_force_norm"
    - name: "h_in"
      shape: [1, 8]
      source: recurrent
  outputs:
    - name: "arm_action"
      shape: [1, 6]
    - name: "posture"
      shape: [1, 1]
    - name: "h_out"
      shape: [1, 8]
      feeds: "h_in"
  output_features:
    - { tensor: "arm_action", role: "joint_target",   device: "arm" }
    - { tensor: "posture",    role: "posture_scalar", device: "hand" }
  hand_posture:
    open:  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    close: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
)";

ControllerState MakeState() {
  ControllerState state{};
  state.num_devices = 2;
  state.dt = 0.002;
  state.iteration = 1;
  state.devices[0].num_channels = kArmDof;
  state.devices[0].valid = true;
  state.devices[1].num_channels = kHandDof;
  state.devices[1].valid = true;
  state.devices[1].num_inference_groups = kFingertips;
  for (int f = 0; f < kFingertips; ++f) {
    state.devices[1].inference_enable[static_cast<std::size_t>(f)] = true;
  }
  return state;
}

// The shipped config opens its CSV logs at configure. This keeps them out of the
// workspace's real session tree — and is what lets registration succeed, which
// puts the bound log push inside the shipped gates below.
const auto* const kSession =
    ::testing::AddGlobalTestEnvironment(new integrated_bringup::testfx::IsolatedSessionDir);

class InferenceAllocGate : public ::testing::Test {
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

  void SetUp() override { Build(kYaml, {kInputElements}); }

  /// Bring one controller all the way to ACTIVE. Split out of SetUp so a case
  /// can rebuild on a different schema without duplicating the three-pass
  /// order, which is itself part of what these cases exercise.
  void Build(const char* yaml, const std::vector<std::size_t>& in_sizes,
             const std::vector<std::size_t>& out_sizes = {6, 1}) {
    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("inference_alloc_gate", "", opts);
    ctrl_ = std::make_unique<DemoInferenceController>(
        "", std::make_unique<FakeEngine>(in_sizes, out_sizes));
    const YAML::Node cfg = YAML::Load(yaml);
    ctrl_->LoadConfig(cfg);
    ctrl_->SetDeviceNameConfigs(MakeDeviceConfigs());
    ctrl_->OnDeviceConfigsSet();
    ASSERT_EQ(ctrl_->on_configure(rclcpp_lifecycle::State{}, node_, cfg),
              rtc::RTControllerInterface::CallbackReturn::SUCCESS);
    ASSERT_EQ(ctrl_->on_activate(rclcpp_lifecycle::State{}),
              rtc::RTControllerInterface::CallbackReturn::SUCCESS);
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::unique_ptr<DemoInferenceController> ctrl_;
};

}  // namespace

TEST_F(InferenceAllocGate, GatePositiveControl) {
  // `new int(7)` does NOT work here: C++ permits eliding a new-expression, and
  // the optimiser takes that permission, so the gate reads zero and the whole
  // file goes vacuously green. Calling `::operator new` directly is a plain
  // function call — not elidable — and `volatile` stops the result being
  // reasoned away.
  std::size_t seen = 0;
  {
    const rtc::testing::ScopedAllocGate gate;
    void* volatile p = ::operator new(64);
    seen = gate.count();
    ::operator delete(const_cast<void*>(p));
  }
  EXPECT_GT(seen, 0U) << "the allocation gate never fired — it is inert, and every "
                         "other case in this file would be vacuously green";
}

TEST_F(InferenceAllocGate, ComputeDoesNotAllocateOnEvaluationOrHeldTicks) {
  auto state = MakeState();

  // Warm up outside the gate exactly as the real loop does: the first tick
  // takes the evaluation branch, and anything one-shot happens there.
  for (int t = 0; t < 25; ++t) {
    static_cast<void>(ctrl_->Compute(state));
  }
  ASSERT_FALSE(ctrl_->LastTickHeldForTesting());

  std::size_t allocations = 0;
  {
    const rtc::testing::ScopedAllocGate gate;
    // 100 ticks at decimation 10 covers both branches: ten evaluations
    // (pack → run → unpack → blend → tail) and ninety held ticks.
    for (int t = 0; t < 100; ++t) {
      static_cast<void>(ctrl_->Compute(state));
    }
    allocations = gate.count();
  }
  EXPECT_EQ(allocations, 0U) << "the RT tick allocated";
}

TEST_F(InferenceAllocGate, TheHoldPathDoesNotAllocateEither) {
  // The hold path runs when things are already going wrong, which is the worst
  // time to discover it allocates.
  auto state = MakeState();
  for (int t = 0; t < 5; ++t) {
    static_cast<void>(ctrl_->Compute(state));
  }
  state.devices[0].hole_mask = 0b10U;  // arm lane has a hole → hold every tick

  static_cast<void>(ctrl_->Compute(state));
  ASSERT_TRUE(ctrl_->LastTickHeldForTesting());

  std::size_t allocations = 0;
  {
    const rtc::testing::ScopedAllocGate gate;
    for (int t = 0; t < 100; ++t) {
      static_cast<void>(ctrl_->Compute(state));
    }
    allocations = gate.count();
  }
  EXPECT_EQ(allocations, 0U) << "the hold path allocated";
}

TEST_F(InferenceAllocGate, AMultiTensorObservationDoesNotAllocateEither) {
  // The N-tensor walk is where an allocation would be easiest to introduce and
  // hardest to notice: a `std::vector<std::span<float>>` gathering the engine's
  // buffers each tick would look perfectly ordinary, cost one allocation per
  // policy step, and never show up as a wrong number anywhere.
  Build(kTwoInputYaml, {16U, 4U});
  auto state = MakeState();

  for (int t = 0; t < 25; ++t) {
    static_cast<void>(ctrl_->Compute(state));
  }
  ASSERT_FALSE(ctrl_->LastTickHeldForTesting())
      << "the two-tensor schema must actually run, or this case gates the hold path twice";

  std::size_t allocations = 0;
  {
    const rtc::testing::ScopedAllocGate gate;
    for (int t = 0; t < 100; ++t) {
      static_cast<void>(ctrl_->Compute(state));
    }
    allocations = gate.count();
  }
  EXPECT_EQ(allocations, 0U) << "the multi-tensor RT tick allocated";
}

TEST_F(InferenceAllocGate, TheRecurrentFeedbackPathDoesNotAllocate) {
  // The state copy is engine buffer → engine buffer and must stay that way. A
  // staging `std::vector` for the hidden state would be the obvious way to
  // write it, would cost one allocation per policy step, and would never show
  // up as a wrong number — the actions would be identical.
  Build(kRecurrentYaml, {kInputElements, 8U}, {6, 1, 8});
  auto state = MakeState();

  for (int t = 0; t < 25; ++t) {
    static_cast<void>(ctrl_->Compute(state));
  }
  ASSERT_FALSE(ctrl_->LastTickHeldForTesting())
      << "the recurrent schema must actually run, or this case gates the hold path twice";

  std::size_t allocations = 0;
  {
    const rtc::testing::ScopedAllocGate gate;
    for (int t = 0; t < 100; ++t) {
      static_cast<void>(ctrl_->Compute(state));
    }
    allocations = gate.count();
  }
  EXPECT_EQ(allocations, 0U) << "the recurrent RT tick allocated";
}

TEST_F(InferenceAllocGate, TheDiagLogPushDoesNotAllocate) {
  // Bound to a real producer: an unbound handle returns at PushLogs' first
  // check, and a gate around that would pass without seeing the push at all.
  using integrated_bringup::InferenceDiagLogPod;
  rtc::ThreadCsvProducer<InferenceDiagLogPod, 512> producer;
  ctrl_->SetInferenceDiagLogHandleForTesting(rtc::LogHandle<InferenceDiagLogPod>(&producer));
  auto state = MakeState();
  for (int t = 0; t < 25; ++t) {
    static_cast<void>(ctrl_->Compute(state));
  }
  ASSERT_FALSE(ctrl_->LastTickHeldForTesting());
  static_cast<void>(producer.Drain([](const InferenceDiagLogPod&) {}));

  std::size_t allocations = 0;
  {
    const rtc::testing::ScopedAllocGate gate;
    for (int t = 0; t < 100; ++t) {
      static_cast<void>(ctrl_->Compute(state));
    }
    allocations = gate.count();
  }
  const std::size_t rows = producer.Drain([](const InferenceDiagLogPod&) {});
  EXPECT_EQ(allocations, 0U) << "the diag log push allocated";
  EXPECT_EQ(rows, 100U) << "every gated tick must have pushed its row, or the gate did not see "
                           "the push path";
}

// ── The shipped policy path on the real ur5e_p1b model ─────────────────────

namespace {

namespace fx = integrated_bringup::testfx;

std::vector<std::size_t> ShippedNumels(const YAML::Node& tensors) {
  std::vector<std::size_t> out;
  for (const auto& t : tensors) {
    std::size_t n = 1;
    for (const auto& d : t["shape"]) {
      n *= d.as<std::size_t>();
    }
    out.push_back(n);
  }
  return out;
}

class ShippedInferenceAllocGate : public InferenceAllocGate {
 protected:
  void SetUp() override {
    YAML::Node cfg = fx::ShippedControllerNode("ur5e_p1b", "demo_inference_controller");
    cfg["inference"]["model_path"] = "fake_policy.onnx";
    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("inference_alloc_shipped", "", opts);
    ctrl_ = std::make_unique<DemoInferenceController>(
        "", std::make_unique<FakeEngine>(ShippedNumels(cfg["inference"]["inputs"]),
                                         ShippedNumels(cfg["inference"]["outputs"])));
    ctrl_->SetSystemModelConfig(fx::SharedUr5eP1bModelConfig());
    ctrl_->SetSharedModelBuilder(fx::SharedUr5eP1bBuilder());
    ctrl_->SetControlRate(1.0 / fx::kUr5eDt);
    ctrl_->LoadConfig(cfg);
    auto devices = fx::MakeUr5eP1bDeviceConfigs();
    rtc::DeviceSensorLayout layout;
    layout.inference_values_per_group = kInferenceStride;
    devices.at("p1b").sensor_layout = layout;
    ctrl_->SetDeviceNameConfigs(devices);
    ASSERT_EQ(ctrl_->on_configure(rclcpp_lifecycle::State{}, node_, cfg),
              rtc::RTControllerInterface::CallbackReturn::SUCCESS);
    ASSERT_EQ(ctrl_->on_activate(rclcpp_lifecycle::State{}),
              rtc::RTControllerInterface::CallbackReturn::SUCCESS);

    // The object sample: world (1.0, 0.0, 0.5), identity. Delivered OUTSIDE
    // the gate — the callback is non-RT and allowed to allocate.
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "world";
    tf.child_frame_id = "pool_pole_object";
    tf.transform.translation.x = 1.0;
    tf.transform.translation.z = 0.5;
    tf.transform.rotation.w = 1.0;
    object_msg_.transforms.push_back(tf);

    state_ = fx::MakeUr5eP1bState();
    for (int f = 0; f < 4; ++f) {
      state_.devices[1].inference_enable[static_cast<std::size_t>(f)] = true;
      state_.devices[1].inference_data[static_cast<std::size_t>((f * kInferenceStride) + 3)] =
          (f < 3) ? 1.0F : 0.0F;  // thumb, index, middle pressing: the grasp branch runs
    }
  }

  /// Warm up outside the gate until the policy runs: the closed-chain
  /// projection walks in from its reference seed first, and those ticks hold.
  ::testing::AssertionResult WarmUp() {
    for (int t = 0; t < 600; ++t) {
      ctrl_->InjectObjectTransformsForTesting(object_msg_);
      static_cast<void>(ctrl_->Compute(state_));
      if (!ctrl_->LastTickHeldForTesting() && t > 20) {
        return ::testing::AssertionSuccess();
      }
    }
    return ::testing::AssertionFailure() << "the shipped policy never ran";
  }

  tf2_msgs::msg::TFMessage object_msg_;
  ControllerState state_{};
};

}  // namespace

TEST_F(ShippedInferenceAllocGate, TheShippedPolicyPathDoesNotAllocate) {
  ASSERT_TRUE(WarmUp());
  ctrl_->InjectObjectTransformsForTesting(object_msg_);  // fresh sample, outside the gate

  std::size_t allocations = 0;
  {
    const rtc::testing::ScopedAllocGate gate;
    // 50 ticks = 10 policy steps at decimation 5, and 0.1 s of object age —
    // inside its 0.2 s timeout, so every step runs the full observation.
    for (int t = 0; t < 50; ++t) {
      static_cast<void>(ctrl_->Compute(state_));
    }
    allocations = gate.count();
  }
  ASSERT_FALSE(ctrl_->LastTickHeldForTesting()) << "the gated ticks must have run the policy";
  EXPECT_EQ(allocations, 0U) << "the shipped policy tick allocated";
}

TEST_F(ShippedInferenceAllocGate, ASeededResetAfterALongHoldDoesNotAllocate) {
  // The reset path — seed extraction into the integrator, zeroing the other
  // states, clearing the reach trigger — only runs after activation or a long
  // hold, so the case above never reaches it inside its gate.
  ASSERT_TRUE(WarmUp());
  ctrl_->InjectObjectTransformsForTesting(object_msg_);

  std::size_t allocations = 0;
  bool resumed = false;
  {
    const rtc::testing::ScopedAllocGate gate;
    state_.devices[0].hole_mask = 0b1U;
    for (int t = 0; t < 55; ++t) {  // 0.11 s > reset_after_hold_sec 0.1
      static_cast<void>(ctrl_->Compute(state_));
    }
    state_.devices[0].hole_mask = 0U;
    for (int t = 0; t < 5; ++t) {
      static_cast<void>(ctrl_->Compute(state_));
    }
    resumed = !ctrl_->LastTickHeldForTesting();
    allocations = gate.count();
  }
  ASSERT_TRUE(resumed) << "the resume must have run the policy, or the reset path was not gated";
  EXPECT_EQ(allocations, 0U) << "the reset/seed path allocated";
}
