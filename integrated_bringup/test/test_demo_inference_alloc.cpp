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
//   Runtime: a real Run() may allocate internally even on top of IoBinding.
//   That one cannot be measured before a model exists, so it is listed as an
//   on-arrival check in demo_inference_controller.yaml's model_path block
//   rather than left to be rediscovered.
//
// The consequence is stated rather than papered over: the feature-packing,
// decimation, unpack, blend and command-tail path IS covered here, and the
// palm-FK path (Pinocchio + Eigen inside the library) IS NOT. The cases below
// therefore drive a schema with no palm feature — gating a path this harness
// cannot observe would be worse than not gating it, because it would read as
// coverage.
//
// The positive control is not decoration. Without it every assertion here could
// be green because the gate was inert.

#include "integrated_bringup/controllers/demo_inference_controller.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>

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
  FakeEngine() : input_(kInputElements, 0.0F), head0_(6, 0.1F), head1_(1, 0.5F) {}

  void Init(const rtc::ModelConfig& /*config*/) override { initialized_ = true; }

  [[nodiscard]] bool Run() noexcept override { return true; }

  float* input_buffer(int /*m*/) noexcept override { return input_.data(); }

  const float* output_buffer(int /*m*/, int output_idx) const noexcept override {
    return (output_idx == 0) ? head0_.data() : head1_.data();
  }

  [[nodiscard]] std::size_t input_size(int /*m*/) const noexcept override { return input_.size(); }

  [[nodiscard]] std::size_t output_size(int /*m*/, int output_idx) const noexcept override {
    return (output_idx == 0) ? head0_.size() : head1_.size();
  }

  [[nodiscard]] int num_outputs(int /*m*/) const noexcept override { return 2; }

  [[nodiscard]] bool is_initialized() const noexcept override { return initialized_; }

  [[nodiscard]] int num_models() const noexcept override { return 1; }

 private:
  bool initialized_{false};
  std::vector<float> input_;
  std::vector<float> head0_;
  std::vector<float> head1_;
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
  input_shape: [1, 20]
  output_shapes: [[1, 6], [1, 1]]
  input_features:
    - "arm.position"
    - "hand.position"
    - "hand.fingertip_force_norm"
  output_features:
    - { name: "arm.target_position", head: 0, offset: 0, count: 6 }
    - { name: "hand.posture_scalar", head: 1, offset: 0, count: 1 }
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

  void SetUp() override {
    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("inference_alloc_gate", "", opts);
    ctrl_ = std::make_unique<DemoInferenceController>("", std::make_unique<FakeEngine>());
    const YAML::Node cfg = YAML::Load(kYaml);
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
