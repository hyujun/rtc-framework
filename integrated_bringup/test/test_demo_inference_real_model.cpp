// ── DemoInferenceController on the REAL policy and ONNX Runtime (local only) ──
//
// Every other inference suite injects a fake engine, so none of them has seen
// the shipped config meet the model it was written for. This one does, which
// needs the policy file — and the file stays out of the repo (public: the model,
// its exports and anything computed from them are never committed). It is found
// the way production finds it: the shipped `model_path` expands
// `${RTC_POLICY_DIR}`, and nothing here edits that key.
//
//   RTC_POLICY_DIR unset → every model case SKIPS. A skip is UNVERIFIED, not
//                          passed: CI never has the file, so in CI this suite
//                          proves nothing and says so.
//   RTC_POLICY_DIR set   → the file must load. A wrong directory FAILS rather
//                          than skips, as production refuses a path that is set
//                          but broken.
//
// What it pins:
//   * configure: the shipped YAML against the real .onnx through the real
//     engine, every tensor bound by name; one misspelled input name fails
//     configure and the engine's table names both spellings.
//   * the first real actions, from the training pregrasp: finite; the arm
//     within one step of the graph's slew of the MEASURED arm (a zero seed
//     would put it within one step of zero); the hand at the measured hand
//     (the synergy starts at 0); then, every step, both integrators inside
//     their per-step bound.
//
// What it only produces or measures — recorded, never asserted:
//   * a parity dump (RTC_INFERENCE_DUMP_DIR): the exact input tensors of
//     several evaluations and the outputs this engine produced, raw float32 +
//     a manifest, for an offline run of the Python ORT of the same version.
//     The dump is a derivative of the policy, so a directory inside the repo
//     is refused.
//   * Run() and policy-tick latency, and the global operator-new count of
//     Run() and of a policy tick, as RecordProperty entries in the test XML.
//     Not gates: this box is not the control PC and this process is not
//     SCHED_FIFO on a shielded core. The numbers inform a decision.
//
// The operator-new count is a LOWER bound on what Run() allocates. The gate
// replaces the plain `operator new` (libstdc++ routes the nothrow form through
// it); the aligned form and ORT's own malloc / posix_memalign (its arena) go
// around it. A non-zero count is therefore conclusive, a zero is not.

#include "inference_shipped_fixture.hpp"
#include "integrated_bringup/controllers/demo_inference_controller.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_inference/onnx/onnx_engine.hpp"
#include "shipped_config_test_fixture.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <rcutils/logging.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fx = integrated_bringup::testfx;
using integrated_bringup::DemoInferenceController;
using rtc::ControllerState;
using CR = rtc::RTControllerInterface::CallbackReturn;
using Clock = std::chrono::steady_clock;

/// The graph's per-step bound on both of its integrators (arm target and
/// synergy): `x_out = x_in + clamp(target − x_in, ±0.005)`.
constexpr double kPolicySlew = 0.005;
/// float32 round-off on values of order 1–3.
constexpr double kFloatTol = 1e-5;
/// The synergy's range: (clamp(a, ±1) + 1) · g with g ∈ [0, 1], floored at g.
constexpr double kSynergyMax = 2.0;

std::string EnvOrEmpty(const char* name) {
  const char* v = std::getenv(name);
  return (v != nullptr) ? std::string(v) : std::string();
}

// ── The recording seam ──────────────────────────────────────────────────────
//
// The real engine behind a pass-through. While `recording` is on, Run()
// snapshots the inputs it is about to evaluate and the outputs it produced. The
// snapshot has to be taken HERE: right after Run() the controller writes the
// recurrent outputs back into the input buffers, so reading the engine after
// Compute() shows the NEXT step's state, not the one the model saw. The
// snapshot buffers are sized at Init, so recording copies without allocating.
class RecordingEngine final : public rtc::InferenceEngine {
 public:
  explicit RecordingEngine(std::unique_ptr<rtc::OnnxEngine> inner) : inner_(std::move(inner)) {}

  void Init(const rtc::ModelConfig& config) override {
    inner_->Init(config);
    seen_inputs.assign(config.inputs.size(), {});
    produced_outputs.assign(config.outputs.size(), {});
    for (std::size_t t = 0; t < seen_inputs.size(); ++t) {
      seen_inputs[t].assign(inner_->input_size(0, static_cast<int>(t)), 0.0F);
    }
    for (std::size_t h = 0; h < produced_outputs.size(); ++h) {
      produced_outputs[h].assign(inner_->output_size(0, static_cast<int>(h)), 0.0F);
    }
  }

  [[nodiscard]] bool Run() noexcept override {
    if (recording) {
      for (std::size_t t = 0; t < seen_inputs.size(); ++t) {
        const float* src = inner_->input_buffer(0, static_cast<int>(t));
        std::copy(src, src + seen_inputs[t].size(), seen_inputs[t].begin());
      }
    }
    const bool ok = inner_->Run();
    if (recording && ok) {
      for (std::size_t h = 0; h < produced_outputs.size(); ++h) {
        const float* src = inner_->output_buffer(0, static_cast<int>(h));
        std::copy(src, src + produced_outputs[h].size(), produced_outputs[h].begin());
      }
      ++recorded_runs;
    }
    return ok;
  }

  float* input_buffer(int model_idx, int input_idx) noexcept override {
    return inner_->input_buffer(model_idx, input_idx);
  }

  [[nodiscard]] const float* output_buffer(int model_idx, int output_idx) const noexcept override {
    return inner_->output_buffer(model_idx, output_idx);
  }

  [[nodiscard]] std::size_t input_size(int model_idx, int input_idx) const noexcept override {
    return inner_->input_size(model_idx, input_idx);
  }

  [[nodiscard]] std::size_t output_size(int model_idx, int output_idx) const noexcept override {
    return inner_->output_size(model_idx, output_idx);
  }

  [[nodiscard]] int num_inputs(int model_idx) const noexcept override {
    return inner_->num_inputs(model_idx);
  }

  [[nodiscard]] int num_outputs(int model_idx) const noexcept override {
    return inner_->num_outputs(model_idx);
  }

  [[nodiscard]] bool is_initialized() const noexcept override { return inner_->is_initialized(); }

  [[nodiscard]] int num_models() const noexcept override { return inner_->num_models(); }

  [[nodiscard]] rtc::OnnxEngine& inner() noexcept { return *inner_; }

  bool recording{false};
  std::uint64_t recorded_runs{0};
  std::vector<std::vector<float>> seen_inputs;
  std::vector<std::vector<float>> produced_outputs;

 private:
  std::unique_ptr<rtc::OnnxEngine> inner_;
};

// ── ERROR-line capture ──────────────────────────────────────────────────────
//
// The engine's I/O table is the observable of a misspelled name, and it
// reaches the operator only as the text of one RCLCPP_ERROR — so that is where
// the assertion goes. Full length (the table for 25 tensors is several KiB, past
// any fixed buffer), and chained to the previous handler so a local run still
// shows the controller's log.
class ErrorLog {
 public:
  static void Install() {
    Clear();
    Previous() = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&ErrorLog::Handler);
  }

  static void Restore() {
    if (Previous() != nullptr) {
      rcutils_logging_set_output_handler(Previous());
      Previous() = nullptr;
    }
  }

  static void Clear() {
    const std::lock_guard<std::mutex> lock(Mutex());
    Lines().clear();
  }

  static std::string Joined() {
    const std::lock_guard<std::mutex> lock(Mutex());
    std::string out;
    for (const auto& line : Lines()) {
      out.append(line).append("\n");
    }
    return out;
  }

 private:
  static void Handler(const rcutils_log_location_t* location, int severity, const char* name,
                      rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
    if (severity >= RCUTILS_LOG_SEVERITY_ERROR) {
      va_list sizing;
      va_copy(sizing, *args);
      const int n = std::vsnprintf(nullptr, 0, format, sizing);
      va_end(sizing);
      if (n > 0) {
        std::string text(static_cast<std::size_t>(n) + 1, '\0');
        va_list copy;
        va_copy(copy, *args);
        std::vsnprintf(text.data(), text.size(), format, copy);
        va_end(copy);
        text.resize(static_cast<std::size_t>(n));
        const std::lock_guard<std::mutex> lock(Mutex());
        Lines().push_back(std::move(text));
      }
    }
    if (Previous() != nullptr) {
      va_list forward;
      va_copy(forward, *args);
      Previous()(location, severity, name, timestamp, format, &forward);
      va_end(forward);
    }
  }

  static std::vector<std::string>& Lines() {
    static std::vector<std::string> lines;
    return lines;
  }

  static std::mutex& Mutex() {
    static std::mutex m;
    return m;
  }

  static rcutils_logging_output_handler_t& Previous() {
    static rcutils_logging_output_handler_t prev = nullptr;
    return prev;
  }
};

/// Percentile of an unsorted sample (nearest rank), in the sample's unit.
double Percentile(std::vector<double> v, double p) {
  std::sort(v.begin(), v.end());
  const auto rank = static_cast<std::size_t>(std::ceil(p * static_cast<double>(v.size())));
  return v[std::min(v.size() - 1, (rank > 0) ? rank - 1 : 0)];
}

/// Record p50 / p99 / max of a latency sample [µs] under `key`, and print it.
void RecordLatency(const std::string& key, const std::vector<double>& us) {
  const double p50 = Percentile(us, 0.50);
  const double p99 = Percentile(us, 0.99);
  const double max = *std::max_element(us.begin(), us.end());
  std::ostringstream s;
  s.precision(1);
  s << std::fixed << "p50 " << p50 << " us, p99 " << p99 << " us, max " << max << " us (n "
    << us.size() << ")";
  ::testing::Test::RecordProperty(key, s.str());
  std::printf("[measured] %s: %s\n", key.c_str(), s.str().c_str());
}

void RecordValue(const std::string& key, double value) {
  std::ostringstream s;
  s.precision(4);
  s << std::fixed << value;
  ::testing::Test::RecordProperty(key, s.str());
  std::printf("[measured] %s: %s\n", key.c_str(), s.str().c_str());
}

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

/// The production bring-up order on the real model, the SHIPPED controller
/// config untouched, and the real engine behind the recording seam.
class RealPolicy : public ::testing::Test {
 protected:
  void SetUp() override {
    if (EnvOrEmpty("RTC_POLICY_DIR").empty()) {
      GTEST_SKIP() << "UNVERIFIED — RTC_POLICY_DIR is not set, so the real policy was not loaded "
                      "and nothing in this case ran. Export it as the directory holding the "
                      ".onnx the shipped config names.";
    }
    cfg_ = fx::ShippedControllerNode("ur5e_p1b", "demo_inference_controller");
  }

  void TearDown() override { ErrorLog::Restore(); }

  CR Configure(const YAML::Node& cfg) {
    auto engine = std::make_unique<RecordingEngine>(std::make_unique<rtc::OnnxEngine>());
    engine_ = engine.get();
    ctrl_ = std::make_unique<DemoInferenceController>("", std::move(engine));

    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    static int seq = 0;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "inference_real_" + std::to_string(seq++), "", opts);

    ctrl_->SetSystemModelConfig(fx::SharedUr5eP1bModelConfig());
    ctrl_->SetSharedModelBuilder(fx::SharedUr5eP1bBuilder());
    ctrl_->SetControlRate(1.0 / fx::kUr5eDt);
    ctrl_->LoadConfig(cfg);
    ctrl_->SetDeviceNameConfigs(fx::MakeInferenceDeviceConfigs());
    ErrorLog::Install();
    const auto ret = ctrl_->on_configure(rclcpp_lifecycle::State{}, node_, cfg);
    ErrorLog::Restore();
    return ret;
  }

  /// Configure + activate the shipped config; the controller's errors are the
  /// failure message.
  ::testing::AssertionResult BringUp() {
    if (Configure(cfg_) != CR::SUCCESS) {
      return ::testing::AssertionFailure()
             << "the shipped config did not configure against the real policy:\n"
             << ErrorLog::Joined();
    }
    if (ctrl_->on_activate(rclcpp_lifecycle::State{}) != CR::SUCCESS) {
      return ::testing::AssertionFailure() << "activation failed";
    }
    return ::testing::AssertionSuccess();
  }

  [[nodiscard]] std::size_t In(const std::string& name) const {
    const auto& ins = ctrl_->IoParamsForTesting().inputs;
    for (std::size_t t = 0; t < ins.size(); ++t) {
      if (ins[t].name == name) {
        return t;
      }
    }
    ADD_FAILURE() << "no input tensor '" << name << "'";
    return ins.size();
  }

  [[nodiscard]] std::size_t Out(const std::string& name) const {
    const auto& outs = ctrl_->IoParamsForTesting().outputs;
    for (std::size_t t = 0; t < outs.size(); ++t) {
      if (outs[t].name == name) {
        return t;
      }
    }
    ADD_FAILURE() << "no output tensor '" << name << "'";
    return outs.size();
  }

  /// What the model saw / produced on the last recorded evaluation.
  [[nodiscard]] const std::vector<float>& Seen(const std::string& name) const {
    return engine_->seen_inputs[In(name)];
  }

  [[nodiscard]] const std::vector<float>& Produced(const std::string& name) const {
    return engine_->produced_outputs[Out(name)];
  }

  void InjectObjectInPolicyFrame(const Eigen::Vector3d& p_pf, const Eigen::Quaterniond& q_pf) {
    object_msg_ = fx::MakeObjectTf(fx::WorldFromPolicyFrame(p_pf), fx::WorldFromPolicyFrame(q_pf));
    Republish();
  }

  void Republish() {
    if (!object_msg_.transforms.empty()) {
      ctrl_->InjectObjectTransformsForTesting(object_msg_);
    }
  }

  /// Tick until a policy step is accepted (the closed-chain projection walks in
  /// from its reference seed first, and those ticks hold by design).
  ::testing::AssertionResult RunUntilAccepted(const ControllerState& state, int max_ticks = 600) {
    const auto before = ctrl_->InferenceCountForTesting();
    for (int t = 0; t < max_ticks; ++t) {
      Republish();
      last_out_ = ctrl_->Compute(state);
      if (!ctrl_->LastTickHeldForTesting() && ctrl_->InferenceCountForTesting() > before) {
        return ::testing::AssertionSuccess();
      }
    }
    return ::testing::AssertionFailure()
           << "no policy step accepted in " << max_ticks << " ticks (held throughout)";
  }

  /// Tick until the NEXT policy evaluation, which must be accepted.
  ::testing::AssertionResult RunOnePolicyStep(const ControllerState& state) {
    const auto before = ctrl_->InferenceCountForTesting();
    for (int t = 0; t < 50 && ctrl_->InferenceCountForTesting() == before; ++t) {
      Republish();
      last_out_ = ctrl_->Compute(state);
    }
    if (ctrl_->InferenceCountForTesting() == before) {
      return ::testing::AssertionFailure() << "no policy evaluation in 50 ticks";
    }
    if (ctrl_->LastTickHeldForTesting()) {
      return ::testing::AssertionFailure() << "the policy step was held";
    }
    return ::testing::AssertionSuccess();
  }

  YAML::Node cfg_;
  RecordingEngine* engine_{nullptr};
  std::unique_ptr<DemoInferenceController> ctrl_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rtc::ControllerOutput last_out_{};
  tf2_msgs::msg::TFMessage object_msg_;
};

/// The trained nominal object: the pole standing on the floor, its z axis
/// down, in the policy frame.
const Eigen::Vector3d kNominalObject(0.5, 0.05, 0.075);

Eigen::Quaterniond NominalObjectOrientation() {
  return Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
}

}  // namespace

// ── §5 criterion 1: the shipped config loads the real model ────────────────

TEST_F(RealPolicy, TheShippedConfigLoadsTheRealModel) {
  ASSERT_TRUE(BringUp());
  EXPECT_FALSE(ctrl_->HoldModeForTesting()) << "a loaded policy is not the no-model hold mode";
  ASSERT_TRUE(engine_->is_initialized());

  const auto in_sizes = fx::Numels(cfg_["inference"]["inputs"]);
  const auto out_sizes = fx::Numels(cfg_["inference"]["outputs"]);
  ASSERT_EQ(engine_->num_inputs(0), 16);
  ASSERT_EQ(engine_->num_outputs(0), 9);
  for (std::size_t t = 0; t < in_sizes.size(); ++t) {
    EXPECT_EQ(engine_->input_size(0, static_cast<int>(t)), in_sizes[t]) << "input " << t;
  }
  for (std::size_t h = 0; h < out_sizes.size(); ++h) {
    EXPECT_EQ(engine_->output_size(0, static_cast<int>(h)), out_sizes[h]) << "output " << h;
  }
#ifdef HAS_ONNXRUNTIME
  RecordProperty("onnxruntime_version", OrtGetApiBase()->GetVersionString());
#endif
}

TEST_F(RealPolicy, OneMisspelledInputNameFailsConfigureAndTheTableNamesBoth) {
  // Same shape, different name: only a by-name binding can tell. The row this
  // renames is not a recurrent input, so the schema accepts it and the refusal
  // has to come from the engine's comparison against the model itself.
  YAML::Node cfg = YAML::Clone(cfg_);
  bool renamed = false;
  for (auto t : cfg["inference"]["inputs"]) {
    if (t["name"].as<std::string>() == "robot_joint_vel") {
      t["name"] = "robot_joint_velocity";
      renamed = true;
    }
  }
  ASSERT_TRUE(renamed);

  ASSERT_EQ(Configure(cfg), CR::FAILURE);
  const std::string log = ErrorLog::Joined();
  EXPECT_NE(log.find("I/O does not match the declared schema"), std::string::npos) << log;

  // Exactly one row of each kind, naming the right spelling on each side — a
  // table that flagged every row would be as useless as none.
  int absent_rows = 0;
  int undeclared_rows = 0;
  std::istringstream lines(log);
  for (std::string line; std::getline(lines, line);) {
    if (line.find("declared in the config, absent from the model") != std::string::npos) {
      ++absent_rows;
      EXPECT_NE(line.find("robot_joint_velocity"), std::string::npos) << line;
    }
    if (line.find("the model has it, the config does not declare it") != std::string::npos) {
      ++undeclared_rows;
      EXPECT_NE(line.find("robot_joint_vel"), std::string::npos) << line;
      EXPECT_EQ(line.find("robot_joint_velocity"), std::string::npos) << line;
    }
  }
  EXPECT_EQ(absent_rows, 1) << log;
  EXPECT_EQ(undeclared_rows, 1) << log;
}

// ── The first real actions, from the training pregrasp ─────────────────────

TEST_F(RealPolicy, TheFirstRealActionsStartFromTheMeasuredRobot) {
  ASSERT_TRUE(BringUp());
  const auto state = fx::MakePregraspState();
  InjectObjectInPolicyFrame(kNominalObject, NominalObjectOrientation());
  engine_->recording = true;
  ASSERT_TRUE(RunUntilAccepted(state));

  // The seeds the model actually saw.
  for (std::size_t i = 0; i < fx::kC18.size(); ++i) {
    EXPECT_NEAR(Seen("arm_applied_target_in")[i], fx::kC18[i], kFloatTol) << "joint " << i;
  }
  EXPECT_FLOAT_EQ(Seen("synergy_in")[0], 0.0F);

  const auto& outs = ctrl_->IoParamsForTesting().outputs;
  for (const auto& head : outs) {
    for (const float v : Produced(head.name)) {
      ASSERT_TRUE(std::isfinite(v)) << head.name;
    }
  }

  // No jump at engagement. The arm target moves at most one slew step from its
  // integrator, which was seeded with the measured arm; the hand command is
  // C20 + synergy · C19 with the synergy one step from zero, i.e. the measured
  // hand to within 0.005 · max|C19| ≈ 0.3 mrad.
  for (std::size_t i = 0; i < fx::kC18.size(); ++i) {
    EXPECT_LE(std::abs(last_out_.devices[0].target_positions[i] - fx::kC18[i]),
              kPolicySlew + kFloatTol)
        << "arm joint " << i;
  }
  for (int j = 0; j < fx::kP1bHandDof; ++j) {
    const auto k = static_cast<std::size_t>(j);
    EXPECT_NEAR(last_out_.devices[1].target_positions[k], state.devices[1].positions[k], 1e-3)
        << "hand joint " << j;
  }
  const double syn0 = Produced("synergy_out")[0];
  EXPECT_GE(syn0, 0.0);
  EXPECT_LE(syn0, kPolicySlew + kFloatTol);
  RecordValue("first_step_reach_phase", Seen("reach_phase")[0]);

  // Then every step: both integrators inside their per-step bound, the arm
  // head identical to the arm state it feeds back, the synergy in range.
  constexpr int kSteps = 40;
  double max_arm_excursion = 0.0;
  for (int step = 1; step <= kSteps; ++step) {
    ASSERT_TRUE(RunOnePolicyStep(state)) << "step " << step;
    const auto& applied_in = Seen("arm_applied_target_in");
    const auto& arm = Produced("arm_action");
    const auto& applied_out = Produced("arm_applied_target_out");
    for (std::size_t i = 0; i < arm.size(); ++i) {
      EXPECT_LE(std::abs(arm[i] - applied_in[i]), kPolicySlew + kFloatTol)
          << "step " << step << " joint " << i;
      EXPECT_FLOAT_EQ(arm[i], applied_out[i]) << "step " << step << " joint " << i;
      max_arm_excursion = std::max(max_arm_excursion, std::abs(arm[i] - fx::kC18[i]));
    }
    const double syn_in = Seen("synergy_in")[0];
    const double syn_out = Produced("synergy_out")[0];
    EXPECT_GE(syn_out, 0.0) << "step " << step;
    EXPECT_LE(syn_out, kSynergyMax + kFloatTol) << "step " << step;
    EXPECT_LE(std::abs(syn_out - syn_in), kPolicySlew + kFloatTol) << "step " << step;
  }
  RecordValue("step40_synergy", Produced("synergy_out")[0]);
  RecordValue("step40_reach_phase", Seen("reach_phase")[0]);
  RecordValue("max_arm_excursion_from_c18_rad", max_arm_excursion);
}

// ── §5 criterion 9: tensors for the offline Python ORT comparison ──────────

TEST_F(RealPolicy, DumpsTheTensorsOfSeveralEvaluationsForAnOfflineComparison) {
  namespace fs = std::filesystem;
  const std::string dump_env = EnvOrEmpty("RTC_INFERENCE_DUMP_DIR");
  if (dump_env.empty()) {
    GTEST_SKIP() << "RTC_INFERENCE_DUMP_DIR is not set — no parity tensors were dumped";
  }
  const fs::path dir = fs::weakly_canonical(fs::path(dump_env));
  const fs::path repo = fs::weakly_canonical(fs::path(RTC_DEMO_SHARED_CONFIG_DIR) / ".." / "..");
  const bool inside_repo =
      std::mismatch(repo.begin(), repo.end(), dir.begin(), dir.end()).first == repo.end();
  ASSERT_FALSE(inside_repo) << dir << " is inside the repository. The dump is derived from the "
                            << "policy and must never be committable — point it elsewhere.";
  fs::create_directories(dir);

  ASSERT_TRUE(BringUp());
  engine_->recording = true;

  // A robot that moves and touches, so every lane carries something the model
  // reads: joints on slow sines with matching velocities, the object drifting a
  // centimetre, and after the first steps a grasp by force — thumb plus two
  // more, which is what latches the reach hold and so moves the synergy (thumb
  // plus ONE leaves the whole synergy branch at zero, untested).
  constexpr double kW = 2.0 * M_PI * 0.5;
  const auto state_at = [&](int tick) {
    auto s = fx::MakePregraspState();
    const double t = static_cast<double>(tick) * fx::kUr5eDt;
    for (int i = 0; i < fx::kUr5eArmDof; ++i) {
      const auto k = static_cast<std::size_t>(i);
      const double ph = 0.7 * static_cast<double>(i);
      s.devices[0].positions[k] += 0.02 * std::sin((kW * t) + ph);
      s.devices[0].velocities[k] = 0.02 * kW * std::cos((kW * t) + ph);
    }
    for (int j = 0; j < fx::kP1bHandDof; ++j) {
      const auto k = static_cast<std::size_t>(j);
      const double ph = 1.0 + (0.3 * static_cast<double>(j));
      s.devices[1].positions[k] += 0.01 * std::sin((kW * t) + ph);
      s.devices[1].velocities[k] = 0.01 * kW * std::cos((kW * t) + ph);
    }
    if (tick > 60) {
      for (const int g : {0, 1, 2}) {  // thumb, index, middle
        const auto base = static_cast<std::size_t>(g * fx::kInferenceStride);
        s.devices[1].inference_enable[static_cast<std::size_t>(g)] = true;
        s.devices[1].inference_data[base + 1] = 0.3F;
        s.devices[1].inference_data[base + 3] = 0.8F;
      }
    }
    return s;
  };

  constexpr int kDumpSteps = 60;
  const auto& ins = ctrl_->IoParamsForTesting().inputs;
  const auto& outs = ctrl_->IoParamsForTesting().outputs;
  const auto write = [&](const fs::path& p, const std::vector<float>& v) {
    std::ofstream f(p, std::ios::binary);
    f.write(reinterpret_cast<const char*>(v.data()),
            static_cast<std::streamsize>(v.size() * sizeof(float)));
    return static_cast<bool>(f);
  };

  int dumped = 0;
  std::uint64_t seen_runs = engine_->recorded_runs;
  for (int tick = 0; tick < 2000 && dumped < kDumpSteps; ++tick) {
    const double t = static_cast<double>(tick) * fx::kUr5eDt;
    InjectObjectInPolicyFrame(
        kNominalObject + Eigen::Vector3d(0.01 * std::sin(kW * t), 0.01 * std::cos(kW * t), 0.0),
        NominalObjectOrientation());
    last_out_ = ctrl_->Compute(state_at(tick));
    if (engine_->recorded_runs == seen_runs) {
      continue;
    }
    seen_runs = engine_->recorded_runs;
    for (std::size_t i = 0; i < ins.size(); ++i) {
      ASSERT_TRUE(
          write(dir / ("step" + std::to_string(dumped) + "_in" + std::to_string(i) + ".f32"),
                engine_->seen_inputs[i]));
    }
    for (std::size_t h = 0; h < outs.size(); ++h) {
      ASSERT_TRUE(
          write(dir / ("step" + std::to_string(dumped) + "_out" + std::to_string(h) + ".f32"),
                engine_->produced_outputs[h]));
    }
    ++dumped;
  }
  ASSERT_EQ(dumped, kDumpSteps) << "the policy did not evaluate often enough to dump";

  std::ofstream manifest(dir / "manifest.txt");
#ifdef HAS_ONNXRUNTIME
  manifest << "ort_version " << OrtGetApiBase()->GetVersionString() << "\n";
#endif
  manifest << "steps " << dumped << "\n";
  const auto shape_line = [&](const char* kind, std::size_t idx, const auto& tensor) {
    manifest << kind << " " << idx << " " << tensor.name;
    for (const auto d : tensor.shape) {
      manifest << " " << d;
    }
    manifest << "\n";
  };
  for (std::size_t i = 0; i < ins.size(); ++i) {
    shape_line("input", i, ins[i]);
  }
  for (std::size_t h = 0; h < outs.size(); ++h) {
    shape_line("output", h, outs[h]);
  }
  ASSERT_TRUE(static_cast<bool>(manifest));
  RecordProperty("dump_dir", dir.string());
}

// ── Measurements: latency and heap traffic of the real engine ──────────────

TEST_F(RealPolicy, MeasuresTheLatencyOfTheRealModel) {
  ASSERT_TRUE(BringUp());
  const auto state = fx::MakePregraspState();
  InjectObjectInPolicyFrame(kNominalObject, NominalObjectOrientation());
  ASSERT_TRUE(RunUntilAccepted(state));

  auto& ort = engine_->inner();
  for (int k = 0; k < 100; ++k) {
    ASSERT_TRUE(ort.Run());
  }
  constexpr int kRuns = 1000;
  std::vector<double> binding_us;
  std::vector<double> direct_us;
  binding_us.reserve(kRuns);
  direct_us.reserve(kRuns);
  const int model = 0;
  for (int k = 0; k < kRuns; ++k) {
    const auto t0 = Clock::now();
    const bool ok = ort.Run();  // IoBinding — what the controller's tick calls
    const auto t1 = Clock::now();
    const bool ok_direct = ort.RunModels(&model, 1);  // direct Session::Run
    const auto t2 = Clock::now();
    ASSERT_TRUE(ok && ok_direct);
    binding_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    direct_us.push_back(std::chrono::duration<double, std::micro>(t2 - t1).count());
  }
  RecordLatency("run_iobinding", binding_us);
  RecordLatency("run_direct", direct_us);

  // The whole tick, which is what has to fit in one control period: Run()
  // executes synchronously inside the tick that owns the policy step, so the
  // budget is that one tick, not `decimation` of them.
  std::vector<double> policy_tick_us;
  std::vector<double> other_tick_us;
  for (int t = 0; t < 5 * kRuns; ++t) {
    Republish();
    const auto before = ctrl_->InferenceCountForTesting();
    const auto t0 = Clock::now();
    last_out_ = ctrl_->Compute(state);
    const auto t1 = Clock::now();
    ASSERT_FALSE(ctrl_->LastTickHeldForTesting()) << "tick " << t;
    const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    (ctrl_->InferenceCountForTesting() > before ? policy_tick_us : other_tick_us).push_back(us);
  }
  RecordLatency("tick_with_policy_step", policy_tick_us);
  RecordLatency("tick_without_policy_step", other_tick_us);
}

TEST(RealPolicyAllocGate, GatePositiveControl) {
  // Without this every count below could be zero because the gate was inert.
  // A new-EXPRESSION is elidable and the optimiser elides it (a first draft
  // here read zero); `::operator new` is a plain call, and `volatile` keeps the
  // result from being reasoned away — the form test_demo_inference_alloc uses.
  std::size_t seen = 0;
  {
    const rtc::testing::ScopedAllocGate gate;
    void* volatile p = ::operator new(64);
    seen = gate.count();
    ::operator delete(const_cast<void*>(p));
  }
  EXPECT_GT(seen, 0U) << "the allocation gate never fired — every count below would be vacuous";
}

TEST_F(RealPolicy, CountsTheHeapAllocationsOfTheRealEngine) {
  ASSERT_TRUE(BringUp());
  const auto state = fx::MakePregraspState();
  InjectObjectInPolicyFrame(kNominalObject, NominalObjectOrientation());
  ASSERT_TRUE(RunUntilAccepted(state));
  auto& ort = engine_->inner();
  for (int k = 0; k < 20; ++k) {  // past any first-run arena growth
    ASSERT_TRUE(ort.Run());
  }

  constexpr int kCalls = 10;
  const int model = 0;
  std::size_t binding = 0;
  std::size_t direct = 0;
  std::size_t ticks = 0;
  bool all_ok = true;
  {
    const rtc::testing::ScopedAllocGate gate;
    for (int k = 0; k < kCalls; ++k) {
      all_ok = ort.Run() && all_ok;
    }
    binding = gate.count();
  }
  {
    const rtc::testing::ScopedAllocGate gate;
    for (int k = 0; k < kCalls; ++k) {
      all_ok = ort.RunModels(&model, 1) && all_ok;
    }
    direct = gate.count();
  }
  ASSERT_TRUE(all_ok);

  Republish();  // fresh sample, outside the gate (the callback is non-RT)
  const auto before = ctrl_->InferenceCountForTesting();
  {
    const rtc::testing::ScopedAllocGate gate;
    for (int t = 0; t < 5 * kCalls; ++t) {  // 0.1 s of object age: inside its timeout
      static_cast<void>(ctrl_->Compute(state));
    }
    ticks = gate.count();
  }
  ASSERT_FALSE(ctrl_->LastTickHeldForTesting()) << "the gated ticks must have run the policy";
  const auto steps = ctrl_->InferenceCountForTesting() - before;
  ASSERT_EQ(steps, static_cast<std::uint64_t>(kCalls));

  RecordValue("operator_new_per_run_iobinding", static_cast<double>(binding) / kCalls);
  RecordValue("operator_new_per_run_direct", static_cast<double>(direct) / kCalls);
  RecordValue("operator_new_per_policy_step_tick", static_cast<double>(ticks) / kCalls);
}
