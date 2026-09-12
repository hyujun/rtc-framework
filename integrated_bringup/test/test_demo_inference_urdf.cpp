// ── DemoInferenceController on the real ur5e_p1b model + the SHIPPED config ──
//
// The roster-fixture suite (test_demo_inference_controller.cpp) pins the
// binding's contracts on made-up joint names. What it cannot see is whether the
// shipped config, read against the real URDF, observes the robot the way the
// policy was trained to: that needs the model (link poses, the closed-chain
// fingertips, the base ↔ base_link half turn) and the shipped YAML itself.
//
// The engine is still the fake — sized from the shipped declaration, not from
// a literal — so everything here runs in CI without the policy file. The only
// edit made to the shipped config is `model_path`, so the fake engine is
// "loaded" instead of the policy directory being required.
//
// ORACLE. Arm at the policy's pregrasp arm constant (C18) and hand at the
// policy's synergy-0 hand constant (C20) mapped through the training asset's
// joint signs: MuJoCo (ur5e_with_proto_1b.xml, mj_kinematics, 2026-09-11) puts
// the palm and fingertips at the positions below, in the MJCF body `base`
// frame (= URDF `base_link`). The passive joints there were the recorded
// pregrasp's own values, whose loop gaps are 0.09–1.73 mm; the binding solves
// them from the actuated joints instead, hence ±2 mm on the tips and ±1 mm on
// the palm (upstream of every loop).
//
// The constants below are kept in the frame they were MEASURED in and crossed
// into the policy frame (URDF `base`) with `fx::BaseFromBaseLink` at the point
// of comparison, rather than re-typed with their signs flipped: a transcribed
// number no longer says where it came from, and this suite exists because the
// frame is exactly the thing that is easy to get wrong.

#include "inference_fake_engine.hpp"
#include "inference_shipped_fixture.hpp"
#include "integrated_bringup/controllers/demo_inference_controller.hpp"
#include "rtc_base/logging/thread_csv_producer.hpp"
#include "session_dir_test_fixture.hpp"
#include "shipped_config_test_fixture.hpp"
#include "ur5e_p1b_test_fixture.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fx = integrated_bringup::testfx;
using integrated_bringup::DemoInferenceController;
using integrated_bringup::testfx::FakeEngine;
using rtc::ControllerState;
using CR = rtc::RTControllerInterface::CallbackReturn;

// Policy constants and the pregrasp state: inference_shipped_fixture.hpp.
using fx::IndexOfName;
using fx::kC18;
using fx::kC20;
using fx::kGripperOrder;
using fx::kInferenceStride;
using fx::kPolicySign;
using fx::MakeObjectTf;
using fx::MakePregraspState;
using fx::Numels;

// Recorded contact points, object frame (cylinder centre), thumb/index/middle/ring.
constexpr std::array<std::array<double, 3>, 4> kC0 = {{{-0.013565, 0.037512, 0.005946},
                                                       {-0.032927, -0.022462, 0.011726},
                                                       {-0.010342, -0.038602, 0.017704},
                                                       {0.01168, -0.038178, -0.002864}}};

// ── MuJoCo oracle (policy frame) ────────────────────────────────────────────
constexpr std::array<double, 3> kPalmPos = {0.6053, 0.0639, 0.2619};
constexpr std::array<double, 4> kPalmQuatXyzw = {0.8892, 0.0761, -0.1590, -0.4223};
const std::map<std::string, std::array<double, 3>> kTipPos = {
    {"l_thumb_tip_link", {0.5906, 0.0844, 0.0973}},
    {"l_index_tip_link", {0.6156, -0.0073, 0.0890}},
    {"l_middle_tip_link", {0.5895, -0.0266, 0.0824}},
    {"l_ring_tip_link", {0.5618, -0.0308, 0.1002}}};
/// The fingertip BRACKETS at the same posture, measured the same way. The reach
/// gate reads these, not the tip links 17.5 mm behind them: the recorded contact
/// points sit on the face that touches, and that face is the bracket's
/// neighbourhood (11.0 mm) rather than the tip link's (25.6 mm).
const std::map<std::string, std::array<double, 3>> kBracketPos = {
    {"l_thumb_tip_bracket", {0.5884, 0.0804, 0.0807}},
    {"l_index_tip_bracket", {0.6098, -0.0020, 0.0734}},
    {"l_middle_tip_bracket", {0.5842, -0.0209, 0.0667}},
    {"l_ring_tip_bracket", {0.5566, -0.0251, 0.0851}}};
constexpr double kPalmTol = 1e-3;
constexpr double kTipTol = 2e-3;

/// The trained nominal object IN THE POLICY FRAME (which is also the sim
/// world): the pole standing on a z = 0 floor under the reset posture's palm.
/// Read in `base_link` the same pole is at (0.5, 0.05, 0.075) — a frame slip
/// does not look like an error, it looks like an object on the other side of
/// the robot.
const Eigen::Vector3d kNominalObject(-0.5, -0.05, 0.075);

void SetForce(ControllerState& s, int group, float fz) {
  auto& hand = s.devices[1];
  hand.inference_enable[static_cast<std::size_t>(group)] = fz != 0.0F;
  const auto base = static_cast<std::size_t>(group * kInferenceStride);
  hand.inference_data[base + 3] = fz;
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
// The shipped config opens its CSV logs at configure — keep them out of the
// workspace's real session tree.
const auto* const kSession = static_cast<const fx::IsolatedSessionDir*>(
    ::testing::AddGlobalTestEnvironment(new fx::IsolatedSessionDir));

/// The production bring-up order on the real model, driven with the SHIPPED
/// controller config (model_path aside).
class ShippedInference : public ::testing::Test {
 protected:
  void SetUp() override {
    cfg_ = fx::ShippedControllerNode("ur5e_p1b", "demo_inference_controller");
    shipped_model_path_ = cfg_["inference"]["model_path"].as<std::string>("");
    cfg_["inference"]["model_path"] = "fake_policy.onnx";
    AdjustShippedConfig(cfg_);

    in_sizes_ = Numels(cfg_["inference"]["inputs"]);
    out_sizes_ = Numels(cfg_["inference"]["outputs"]);
    auto fake = std::make_unique<FakeEngine>(in_sizes_, out_sizes_);
    engine_ = fake.get();
    for (const auto n : out_sizes_) {
      engine_->next_output.emplace_back(n, 0.0F);
    }
    ctrl_ = std::make_unique<DemoInferenceController>("", std::move(fake));

    rclcpp::NodeOptions opts;
    opts.use_global_arguments(false);
    static int seq = 0;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "inference_urdf_" + std::to_string(seq++), "", opts);

    ctrl_->SetSystemModelConfig(fx::SharedUr5eP1bModelConfig());
    ctrl_->SetSharedModelBuilder(fx::SharedUr5eP1bBuilder());
    ctrl_->SetControlRate(1.0 / fx::kUr5eDt);
    ctrl_->LoadConfig(cfg_);
    ctrl_->SetDeviceNameConfigs(fx::MakeInferenceDeviceConfigs());
    configured_ = ctrl_->on_configure(rclcpp_lifecycle::State{}, node_, cfg_) == CR::SUCCESS;
    ASSERT_TRUE(configured_) << "the shipped config must configure against the real model";
    ASSERT_EQ(ctrl_->on_activate(rclcpp_lifecycle::State{}), CR::SUCCESS);
  }

  /// Hook for the one case that needs the shipped config with a single key
  /// moved (the object lane's source frame). Everything else in this suite runs
  /// the config exactly as it ships, which is the point of the suite.
  virtual void AdjustShippedConfig(YAML::Node& /*cfg*/) {}

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

  /// Tick until a policy step is accepted (the closed-chain projection walks in
  /// from its reference seed first, and those ticks hold by design). Bounded,
  /// so a projection that never converges fails here instead of hanging.
  ::testing::AssertionResult RunUntilAccepted(const ControllerState& state, int max_ticks = 600) {
    const auto before = ctrl_->InferenceCountForTesting();
    for (int t = 0; t < max_ticks; ++t) {
      Republish();
      last_out_ = ctrl_->Compute(state);
      if (!ctrl_->LastTickHeldForTesting() && ctrl_->InferenceCountForTesting() > before) {
        return ::testing::AssertionSuccess() << t + 1 << " ticks";
      }
    }
    return ::testing::AssertionFailure()
           << "no policy step accepted in " << max_ticks << " ticks (held throughout)";
  }

  /// Tick until the NEXT policy evaluation (one decimation period when running).
  void RunOnePolicyStep(const ControllerState& state) {
    const auto before = ctrl_->InferenceCountForTesting();
    for (int t = 0; t < 50 && ctrl_->InferenceCountForTesting() == before; ++t) {
      Republish();
      last_out_ = ctrl_->Compute(state);
    }
    ASSERT_GT(ctrl_->InferenceCountForTesting(), before);
    ASSERT_FALSE(ctrl_->LastTickHeldForTesting());
  }

  [[nodiscard]] const std::vector<float>& Tensor(const std::string& name) const {
    return engine_->last_inputs[In(name)];
  }

  /// Publish the object in `world` and keep republishing it on every tick the
  /// run helpers drive, the way the sim does — the lane goes stale after
  /// `timeout_sec` (100 ticks here), which a walk-in can outlast.
  ///
  /// Under the shipped config `world` IS the policy frame (both name URDF
  /// `base`), so these coordinates are also what the policy is shown. The one
  /// case where they differ overrides `source_frame_link` and says so.
  void InjectObjectInWorld(const Eigen::Vector3d& p_world, const Eigen::Quaterniond& q_world) {
    object_msg_ = MakeObjectTf(p_world, q_world);
    Republish();
  }

  void Republish() {
    if (!object_msg_.transforms.empty()) {
      ctrl_->InjectObjectTransformsForTesting(object_msg_);
    }
  }

  YAML::Node cfg_;
  std::string shipped_model_path_;
  std::vector<std::size_t> in_sizes_;
  std::vector<std::size_t> out_sizes_;
  FakeEngine* engine_{nullptr};
  std::unique_ptr<DemoInferenceController> ctrl_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  rtc::ControllerOutput last_out_{};
  tf2_msgs::msg::TFMessage object_msg_;
  bool configured_{false};
};

}  // namespace

// ── The shipped config itself ───────────────────────────────────────────────
//
// Values AND presence, separately: a key that went missing would otherwise be
// read at its default, and several defaults (decimation 1, identity sign, the
// reach constants) are plausible enough to pass a value check by accident.

TEST_F(ShippedInference, DeclaresTheExportsTensorsByNameInOrder) {
  const auto& io = ctrl_->IoParamsForTesting();
  const std::vector<std::string> inputs = {"reach_phase",
                                           "object_root_pos_w",
                                           "object_root_quat_w",
                                           "arm_applied_target_in",
                                           "synergy_in",
                                           "robot_joint_pos",
                                           "robot_joint_vel",
                                           "robot_body_pos_w",
                                           "robot_body_quat_w",
                                           "robot_root_pos_w",
                                           "robot_root_quat_w",
                                           "thumb_tip_contact_force_matrix_w",
                                           "index_tip_contact_force_matrix_w",
                                           "middle_tip_contact_force_matrix_w",
                                           "ring_tip_contact_force_matrix_w",
                                           "last_action_in"};
  const std::vector<std::string> outputs = {
      "arm_action",          "gripper_action",          "arm_applied_target_out",
      "synergy_out",         "last_action_out",         "arm_action_kp_gains",
      "arm_action_kd_gains", "gripper_action_kp_gains", "gripper_action_kd_gains"};
  ASSERT_EQ(io.inputs.size(), inputs.size());
  ASSERT_EQ(io.outputs.size(), outputs.size());
  for (std::size_t t = 0; t < inputs.size(); ++t) {
    EXPECT_EQ(io.inputs[t].name, inputs[t]);
  }
  for (std::size_t t = 0; t < outputs.size(); ++t) {
    EXPECT_EQ(io.outputs[t].name, outputs[t]);
  }
  EXPECT_EQ(io.recurrent_links.size(), 3U);
  EXPECT_EQ(io.inputs[In("robot_joint_pos")].Numel(), 46U);
  EXPECT_EQ(io.inputs[In("robot_joint_vel")].Numel(), 41U);
  EXPECT_EQ(io.inputs[In("robot_body_pos_w")].stride, 3);
  EXPECT_EQ(io.inputs[In("robot_body_quat_w")].stride, 4);
  EXPECT_EQ(io.inputs[In("arm_applied_target_in")].seed_feature, "ur5e.position");
}

TEST_F(ShippedInference, CarriesTheTrainedCadenceFrameAndModelLocation) {
  const auto inf = cfg_["inference"];
  ASSERT_TRUE(inf["decimation"]) << "decimation must be spelled out, not defaulted to 1";
  EXPECT_EQ(ctrl_->IoParamsForTesting().decimation, 5) << "500 Hz / 5 = the trained 100 Hz";
  ASSERT_TRUE(inf["policy_frame"]);
  EXPECT_EQ(inf["policy_frame"].as<std::string>(), "base")
      << "shown base_link poses the policy anti-tracks the object and never reaches it";
  ASSERT_TRUE(inf["object_pose"]["source_frame_link"]);
  // Same frame as policy_frame, so the shipped object lane applies no rotation.
  // The rotation itself is pinned below, on a config whose source frame moves.
  EXPECT_EQ(inf["object_pose"]["source_frame_link"].as<std::string>(), "base");
  EXPECT_EQ(inf["object_pose"]["source_frame_id"].as<std::string>(), "world");
  EXPECT_EQ(shipped_model_path_, "${RTC_POLICY_DIR}/ObjectHandGraspDeployMulti5-Export-v0.onnx")
      << "the model stays out of the repo and is found through the environment";
  ASSERT_TRUE(inf["allow_missing_model"]);
  EXPECT_TRUE(inf["allow_missing_model"].as<bool>());
}

TEST_F(ShippedInference, CarriesTheTrainingAssetsJointSigns) {
  const auto conv = cfg_["inference"]["joint_convention"];
  ASSERT_TRUE(conv && conv.IsMap());
  for (const auto& [joint, sign] : kPolicySign) {
    if (sign < 0.0) {
      ASSERT_TRUE(conv[joint]) << joint << " must be declared";
      ASSERT_TRUE(conv[joint]["sign"]) << joint;
      EXPECT_EQ(conv[joint]["sign"].as<double>(), -1.0) << joint;
    } else {
      EXPECT_FALSE(conv[joint]) << joint << " shares the URDF's axis and must stay identity";
    }
  }
}

TEST_F(ShippedInference, CarriesTheReachGateTheModelWasTrainedWith) {
  const auto gate = cfg_["inference"]["reach_gate"];
  ASSERT_TRUE(gate);
  // The BRACKETS, not the tip links: the gate measures to the recorded contact
  // points, which sit on the face that touches (see the YAML's own note).
  const std::vector<std::string> links = {"l_thumb_tip_bracket", "l_index_tip_bracket",
                                          "l_middle_tip_bracket", "l_ring_tip_bracket"};
  const std::vector<std::string> groups = {"thumb", "index", "middle", "ring"};
  ASSERT_EQ(gate["tips"].size(), links.size());
  for (std::size_t i = 0; i < links.size(); ++i) {
    const auto tip = gate["tips"][i];
    EXPECT_EQ(tip["link"].as<std::string>(), links[i]) << "the opposing digit comes first";
    EXPECT_EQ(tip["force_group"].as<std::string>(), groups[i]);
    for (std::size_t c = 0; c < 3; ++c) {
      EXPECT_DOUBLE_EQ(tip["contact_obj"][c].as<double>(), kC0[i][c])
          << "tip " << i << " axis " << c << " — the model's own contact point, cylinder centre";
    }
  }
  for (const char* key :
       {"tip_std", "force_threshold", "min_fingers", "hold_on_steps", "hold_off_steps"}) {
    EXPECT_TRUE(gate[key]) << key << " must be spelled out";
  }
  EXPECT_DOUBLE_EQ(gate["tip_std"].as<double>(), 0.030);
  EXPECT_DOUBLE_EQ(gate["force_threshold"].as<double>(), 0.5);
  EXPECT_EQ(gate["min_fingers"].as<int>(), 2);
  EXPECT_EQ(gate["hold_on_steps"].as<int>(), 5);
  EXPECT_EQ(gate["hold_off_steps"].as<int>(), 100);
}

TEST_F(ShippedInference, ServesTheFingertipsThroughTheClosedChain) {
  EXPECT_TRUE(ctrl_->ClosedChainLinksActiveForTesting())
      << "the fingertips are downstream of loop-passive joints; the serial pose is wrong";
}

// ── A projection that never walks in (the quiet failure) ───────────────────

TEST_F(ShippedInference, TheOrdinaryWalkInIsNotWorthWarningAbout) {
  using integrated_bringup::InferenceHoldReason;
  EXPECT_EQ(cfg_["inference"]["closed_chain_warn_ticks"].as<int>(), 250)
      << "the shipped bound must clear the measured walk-in (43 ticks) by a wide margin";
  const auto state = MakePregraspState();
  InjectObjectInWorld(kNominalObject, Eigen::Quaterniond::Identity());
  for (int t = 0; t < 10; ++t) {
    Republish();
    last_out_ = ctrl_->Compute(state);
  }
  ASSERT_EQ(ctrl_->LastHoldReasonForTesting(), InferenceHoldReason::kClosedChain)
      << "this case is about the walk-in, so it has to still be in one";
  ctrl_->PollDiagnosticsForTesting();
  ASSERT_TRUE(RunUntilAccepted(state));
  ctrl_->PollDiagnosticsForTesting();
  EXPECT_EQ(ctrl_->ClosedChainWarningsForTesting(), 0)
      << "bring-up holds by design; warning on it teaches the operator to ignore the warning";
}

/// The same bring-up under a bound the ordinary walk-in crosses — the shape of
/// a projection that is stuck, without having to build one.
class ShippedInferenceWithAnEagerStallWarning : public ShippedInference {
 protected:
  void AdjustShippedConfig(YAML::Node& cfg) override {
    cfg["inference"]["closed_chain_warn_ticks"] = 5;
  }
};

TEST_F(ShippedInferenceWithAnEagerStallWarning, AHeldProjectionIsReportedOncePerActivation) {
  using integrated_bringup::InferenceHoldReason;
  const auto state = MakePregraspState();
  InjectObjectInWorld(kNominalObject, Eigen::Quaterniond::Identity());
  for (int t = 0; t < 10; ++t) {
    Republish();
    last_out_ = ctrl_->Compute(state);
  }
  ASSERT_EQ(ctrl_->LastHoldReasonForTesting(), InferenceHoldReason::kClosedChain);
  for (int poll = 0; poll < 5; ++poll) {
    ctrl_->PollDiagnosticsForTesting();
  }
  EXPECT_EQ(ctrl_->ClosedChainWarningsForTesting(), 1)
      << "the timer polls at 10 Hz; a stall that logs on every poll is a log flood";

  ASSERT_EQ(ctrl_->on_deactivate(rclcpp_lifecycle::State{}), CR::SUCCESS);
  ASSERT_EQ(ctrl_->on_activate(rclcpp_lifecycle::State{}), CR::SUCCESS);
  EXPECT_EQ(ctrl_->ClosedChainWarningsForTesting(), 0)
      << "a new activation is a new robot situation and gets a new budget";
}

// ── §5 criterion 3: frame, convention and closed chain against MuJoCo ──────

TEST_F(ShippedInference, ObservesTheTrainingPregraspWhereMuJoCoPutsIt) {
  const auto state = MakePregraspState();
  InjectObjectInWorld(kNominalObject, Eigen::Quaterniond::Identity());
  ASSERT_TRUE(RunUntilAccepted(state));

  const auto& io = ctrl_->IoParamsForTesting();

  // Joint table: the arm at C18 and the hand at C20 — i.e. the DEVICE values
  // (s ⊙ C20) came back through the convention. An identity convention reads
  // s ⊙ C20 here and fails on eight joints.
  {
    const auto& names = io.inputs[In("robot_joint_pos")].element_names;
    const auto& pos = Tensor("robot_joint_pos");
    const std::vector<std::string> arm = {"shoulder_pan_joint", "shoulder_lift_joint",
                                          "elbow_joint",        "wrist_1_joint",
                                          "wrist_2_joint",      "wrist_3_joint"};
    for (std::size_t i = 0; i < arm.size(); ++i) {
      EXPECT_NEAR(pos[IndexOfName(names, arm[i])], kC18[i], 1e-6) << arm[i];
    }
    for (std::size_t k = 0; k < kGripperOrder.size(); ++k) {
      EXPECT_NEAR(pos[IndexOfName(names, kGripperOrder[k])], kC20[k], 1e-6)
          << kGripperOrder[k] << " must arrive in the POLICY's joint convention";
    }
    for (const auto& n : names) {
      const bool observed =
          std::find(arm.begin(), arm.end(), n) != arm.end() ||
          std::find(kGripperOrder.begin(), kGripperOrder.end(), n) != kGripperOrder.end();
      if (!observed) {
        EXPECT_FLOAT_EQ(pos[IndexOfName(names, n)], 0.0F) << n << " is not observed";
      }
    }
  }

  // Body table: palm and fingertips where MuJoCo puts them. The oracle was
  // measured in MJCF `base` (= URDF `base_link`) and the policy frame is URDF
  // `base`, so every expectation crosses the half turn on the way in.
  {
    const auto& names = io.inputs[In("robot_body_pos_w")].element_names;
    const auto& pos = Tensor("robot_body_pos_w");
    const auto row = [&](const std::string& link) { return 3 * IndexOfName(names, link); };
    const auto in_policy_frame = [](const std::array<double, 3>& p) {
      return fx::BaseFromBaseLink(Eigen::Vector3d(p[0], p[1], p[2]));
    };
    const Eigen::Vector3d palm = in_policy_frame(kPalmPos);
    for (std::size_t c = 0; c < 3; ++c) {
      EXPECT_NEAR(pos[row("l_palm_link") + c], palm[static_cast<Eigen::Index>(c)], kPalmTol)
          << "palm axis " << c;
    }
    for (const auto& [link, p] : kTipPos) {
      const Eigen::Vector3d tip = in_policy_frame(p);
      for (std::size_t c = 0; c < 3; ++c) {
        EXPECT_NEAR(pos[row(link) + c], tip[static_cast<Eigen::Index>(c)], kTipTol)
            << link << " axis " << c;
      }
    }
    EXPECT_FLOAT_EQ(pos[row("tool0")], 0.0F) << "an unobserved row holds the filler";

    const auto& qnames = io.inputs[In("robot_body_quat_w")].element_names;
    const auto& quat = Tensor("robot_body_quat_w");
    const std::size_t prow = 4 * IndexOfName(qnames, "l_palm_link");
    // The oracle carries four decimals, so compare ROTATION ANGLE against the
    // normalised oracle rather than components (q ≡ −q).
    const Eigen::Quaterniond oracle =
        fx::BaseFromBaseLink(Eigen::Quaterniond(kPalmQuatXyzw[3], kPalmQuatXyzw[0],
                                                kPalmQuatXyzw[1], kPalmQuatXyzw[2])
                                 .normalized())
            .normalized();
    const Eigen::Quaterniond seen =
        Eigen::Quaterniond(quat[prow + 3], quat[prow], quat[prow + 1], quat[prow + 2]).normalized();
    EXPECT_LT(seen.angularDistance(oracle), 5e-3) << "palm orientation [rad]";
    const std::size_t trow = 4 * IndexOfName(qnames, "tool0");
    EXPECT_FLOAT_EQ(quat[trow + 3], 1.0F) << "an unobserved row holds the identity filler";
  }

  // Root: the policy frame's own origin.
  EXPECT_FLOAT_EQ(Tensor("robot_root_pos_w")[0], 0.0F);
  EXPECT_FLOAT_EQ(Tensor("robot_root_quat_w")[3], 1.0F);
}

/// The shipped profile publishes the object in the policy frame itself, so it
/// never exercises the rotation. This moves ONLY the object lane's source frame
/// to `base_link` — the shape of a perception stack that publishes in a
/// different frame than the policy reads — and leaves everything else shipped.
class ShippedInferenceWithObjectSourceInBaseLink : public ShippedInference {
 protected:
  void AdjustShippedConfig(YAML::Node& cfg) override {
    cfg["inference"]["object_pose"]["source_frame_link"] = "base_link";
  }
};

TEST_F(ShippedInferenceWithObjectSourceInBaseLink,
       TheObjectPoseIsRotatedIntoThePolicyFrameByTheModel) {
  // The point (1, 2, 3) read in base_link is (−1, −2, 3) in the policy frame
  // (URDF `base`), a half turn about z away. The rotation used to be typed into
  // the YAML (`base_pose_in_world`); it comes from the URDF now, and this is the
  // same assertion against that source — a hand-typed constant would not follow
  // the policy frame when it moves, which is exactly what it just did.
  const auto state = MakePregraspState();
  InjectObjectInWorld({1.0, 2.0, 3.0}, Eigen::Quaterniond::Identity());
  ASSERT_TRUE(RunUntilAccepted(state));
  const auto& p = Tensor("object_root_pos_w");
  EXPECT_NEAR(p[0], -1.0F, 1e-5);
  EXPECT_NEAR(p[1], -2.0F, 1e-5);
  EXPECT_NEAR(p[2], 3.0F, 1e-5) << "z is unchanged by a yaw";
  const auto& q = Tensor("object_root_quat_w");
  EXPECT_NEAR(std::abs(q[2]), 1.0F, 1e-5) << "identity in world is a half turn about z here";
}

// ── §5 criterion 4 on the shipped heads ─────────────────────────────────────

TEST_F(ShippedInference, TheGripperHeadReachesTheHandInDeviceOrderWithSignsUndone) {
  const auto state = MakePregraspState();
  InjectObjectInWorld(kNominalObject, Eigen::Quaterniond::Identity());

  std::vector<float> grip(out_sizes_[Out("gripper_action")]);
  for (std::size_t k = 0; k < grip.size(); ++k) {
    grip[k] = 0.01F * static_cast<float>(k + 1);
  }
  std::vector<float> arm(out_sizes_[Out("arm_action")]);
  for (std::size_t k = 0; k < arm.size(); ++k) {
    arm[k] = static_cast<float>(kC18[k]) + (0.001F * static_cast<float>(k));
  }
  engine_->next_output[Out("gripper_action")] = grip;
  engine_->next_output[Out("arm_action")] = arm;
  ASSERT_TRUE(RunUntilAccepted(state));

  const auto devices = fx::MakeUr5eP1bDeviceConfigs();
  const auto& hand_names = devices.at("p1b").joint_state_names;
  const auto& head_names = ctrl_->IoParamsForTesting().outputs[Out("gripper_action")].element_names;
  for (std::size_t j = 0; j < hand_names.size(); ++j) {
    const auto k = IndexOfName(head_names, hand_names[j]);
    EXPECT_NEAR(last_out_.devices[1].target_positions[j],
                kPolicySign.at(hand_names[j]) * static_cast<double>(grip[k]), 1e-9)
        << hand_names[j];
  }
  for (std::size_t i = 0; i < arm.size(); ++i) {
    EXPECT_NEAR(last_out_.devices[0].target_positions[i], static_cast<double>(arm[i]), 1e-9);
  }

  // The gain heads and the state heads are not commands.
  const auto before = last_out_;
  engine_->next_output[Out("gripper_action_kp_gains")].assign(10, 999.0F);
  engine_->next_output[Out("arm_action_kp_gains")].assign(6, -999.0F);
  engine_->next_output[Out("synergy_out")] = {2.0F};
  RunOnePolicyStep(state);
  for (std::size_t j = 0; j < hand_names.size(); ++j) {
    EXPECT_DOUBLE_EQ(last_out_.devices[1].target_positions[j],
                     before.devices[1].target_positions[j]);
  }
}

// ── §5 criterion 5 on the shipped recurrent tensors ─────────────────────────

TEST_F(ShippedInference, TheArmIntegratorStartsAtTheMeasuredArm) {
  const auto state = MakePregraspState();
  InjectObjectInWorld(kNominalObject, Eigen::Quaterniond::Identity());
  ASSERT_TRUE(RunUntilAccepted(state));
  const auto& applied = Tensor("arm_applied_target_in");
  for (std::size_t i = 0; i < kC18.size(); ++i) {
    EXPECT_NEAR(applied[i], kC18[i], 1e-6) << "joint " << i << ": zero would drive the arm home";
  }
  EXPECT_FLOAT_EQ(Tensor("synergy_in")[0], 0.0F);
  for (const float v : Tensor("last_action_in")) {
    EXPECT_FLOAT_EQ(v, 0.0F);
  }

  engine_->next_output[Out("arm_applied_target_out")].assign(6, 0.123F);
  RunOnePolicyStep(state);
  RunOnePolicyStep(state);
  EXPECT_FLOAT_EQ(Tensor("arm_applied_target_in")[0], 0.123F) << "then the feedback owns it";
}

// ── Reach gate: wiring against the model (the formula is unit-tested) ──────

TEST_F(ShippedInference, ReachPhaseMeasuresFromTheFingertipBracketsNotTheTipLinks) {
  const auto state = MakePregraspState();
  // First step: object far away, to learn where the observed tip links are.
  InjectObjectInWorld({2.0, 2.0, 2.0}, Eigen::Quaterniond::Identity());
  ASSERT_TRUE(RunUntilAccepted(state));
  EXPECT_LT(Tensor("reach_phase")[0], 1e-6F) << "two metres away is no proximity";

  const auto& io = ctrl_->IoParamsForTesting();
  const auto& names = io.inputs[In("robot_body_pos_w")].element_names;
  const std::vector<std::string> tip_links = {"l_thumb_tip_link", "l_index_tip_link",
                                              "l_middle_tip_link", "l_ring_tip_link"};
  const std::vector<std::string> brackets = {"l_thumb_tip_bracket", "l_index_tip_bracket",
                                             "l_middle_tip_bracket", "l_ring_tip_bracket"};
  std::array<Eigen::Vector3d, 4> tip{};
  std::array<Eigen::Vector3d, 4> bracket{};
  for (std::size_t i = 0; i < 4; ++i) {
    const auto r = 3 * IndexOfName(names, tip_links[i]);
    const auto& pos = Tensor("robot_body_pos_w");
    tip[i] = Eigen::Vector3d(pos[r], pos[r + 1], pos[r + 2]);
    const auto& b = kBracketPos.at(brackets[i]);
    bracket[i] = fx::BaseFromBaseLink(Eigen::Vector3d(b[0], b[1], b[2]));
  }

  // Place the object where the recorded contact points sit closest to the
  // BRACKETS (the pole upside down, as trained — Rx(pi) in the policy frame):
  // the least-squares origin for a fixed orientation is the mean of p_i − R·c_i.
  const Eigen::Quaterniond q_obj(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  const auto contact = [&](std::size_t i) {
    return Eigen::Vector3d(kC0[i][0], kC0[i][1], kC0[i][2]);
  };
  Eigen::Vector3d p_obj = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < 4; ++i) {
    p_obj += bracket[i] - (q_obj * contact(i));
  }
  p_obj /= 4.0;
  double d_bracket = 0.0;
  double d_tip = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    const Eigen::Vector3d target = p_obj + (q_obj * contact(i));
    d_bracket += (bracket[i] - target).norm();
    d_tip += (tip[i] - target).norm();
  }
  d_bracket /= 4.0;
  d_tip /= 4.0;

  InjectObjectInWorld(p_obj, q_obj);
  RunOnePolicyStep(state);
  const double gate = Tensor("reach_phase")[0];
  ASSERT_GT(gate, 0.05) << "the fixture must put the gate on its ramp, or this proves nothing";
  ASSERT_LT(gate, 0.999);

  // Invert the gate to recover the distance the controller actually measured:
  // gate = exp(−(d/sigma)²) with no tactile hold armed, so d = sigma·sqrt(−ln gate).
  // Comparing DISTANCES rather than gate values is what makes this test about
  // the frame: the two candidates are 17.5 mm apart, far outside the oracle's
  // own couple of millimetres.
  constexpr double kSigma = 0.030;
  const double d_seen = kSigma * std::sqrt(-std::log(gate));
  EXPECT_NEAR(d_seen, d_bracket, 3e-3)
      << "the gate must measure from the brackets (oracle " << d_bracket << " m)";
  EXPECT_GT(std::abs(d_seen - d_tip), 10e-3)
      << "measuring from the tip links instead would read " << d_tip << " m";
}

TEST_F(ShippedInference, TheTactileHoldCountsPolicyStepsAndReadsTheRightFingers) {
  auto state = MakePregraspState();
  InjectObjectInWorld({2.0, 2.0, 2.0}, Eigen::Quaterniond::Identity());
  ASSERT_TRUE(RunUntilAccepted(state));

  // Thumb + index + middle pressing, ring not: a grasp only if the FIRST tip
  // reads the thumb group. A reversed mapping would put the silent ring first.
  const auto devices = fx::MakeInferenceDeviceConfigs();
  const auto& groups = devices.at("p1b").sensor_names;
  SetForce(state, static_cast<int>(IndexOfName(groups, "thumb")), 1.0F);
  SetForce(state, static_cast<int>(IndexOfName(groups, "index")), 1.0F);
  SetForce(state, static_cast<int>(IndexOfName(groups, "middle")), 1.0F);

  // Four policy steps = twenty ticks. Counting ticks would have latched by now.
  for (int step = 1; step <= 4; ++step) {
    RunOnePolicyStep(state);
    EXPECT_FALSE(ctrl_->ReachHoldForTesting()) << "grasped step " << step;
  }
  RunOnePolicyStep(state);
  EXPECT_TRUE(ctrl_->ReachHoldForTesting()) << "the fifth consecutive grasped step latches";
  // The gate observed on that step is already the latched one.
  EXPECT_FLOAT_EQ(Tensor("reach_phase")[0], 1.0F);

  // Release: the hold survives 99 empty steps and drops on the 100th.
  for (const char* g : {"thumb", "index", "middle"}) {
    SetForce(state, static_cast<int>(IndexOfName(groups, g)), 0.0F);
  }
  for (int step = 1; step <= 99; ++step) {
    RunOnePolicyStep(state);
  }
  EXPECT_TRUE(ctrl_->ReachHoldForTesting());
  RunOnePolicyStep(state);
  EXPECT_FALSE(ctrl_->ReachHoldForTesting());
  EXPECT_LT(Tensor("reach_phase")[0], 1e-6F) << "no ratchet: the gate falls back to proximity";
}

TEST_F(ShippedInference, AReactivationDoesNotInheritTheTactileHold) {
  // The latch is cross-tick state like the recurrent buffers, and the gap makes
  // the same statement about it: the hand has been out of this controller's
  // hands since, so a grasp it latched before cannot speak for now. Every held
  // tick of the new activation (the closed-chain walk-in is dozens of them)
  // reports `reach_hold` on its diagnostic row before the first accepted step.
  auto state = MakePregraspState();
  InjectObjectInWorld({2.0, 2.0, 2.0}, Eigen::Quaterniond::Identity());
  ASSERT_TRUE(RunUntilAccepted(state));

  const auto devices = fx::MakeInferenceDeviceConfigs();
  const auto& groups = devices.at("p1b").sensor_names;
  for (const char* g : {"thumb", "index", "middle"}) {
    SetForce(state, static_cast<int>(IndexOfName(groups, g)), 1.0F);
  }
  for (int step = 0; step < 5; ++step) {
    RunOnePolicyStep(state);
  }
  ASSERT_TRUE(ctrl_->ReachHoldForTesting()) << "the fixture must latch, or this proves nothing";

  ASSERT_EQ(ctrl_->on_deactivate(rclcpp_lifecycle::State{}), CR::SUCCESS);
  ASSERT_EQ(ctrl_->on_activate(rclcpp_lifecycle::State{}), CR::SUCCESS);
  EXPECT_FALSE(ctrl_->ReachHoldForTesting()) << "the new activation starts with no grasp behind it";
}

TEST_F(ShippedInference, AStaleObjectPoseHolds) {
  // The reach gate and the object tensors both need it; without a sample the
  // policy must not run.
  const auto state = MakePregraspState();
  for (int t = 0; t < 20; ++t) {
    static_cast<void>(ctrl_->Compute(state));
    EXPECT_TRUE(ctrl_->LastTickHeldForTesting());
  }
  EXPECT_EQ(engine_->run_count, 0);
}

// ── Observability: the CSV lane and hold reasons (Phase 4) ─────────────────
//
// A hold commands the same latched position whatever caused it, so without a
// recorded reason a run that held on every tick reads like a policy that chose
// to stand still. These pin that each hold is named, that a policy step is
// marked, and that what the row reports is what the policy was actually shown.

TEST_F(ShippedInference, ShipsItsCsvLogs) {
  const YAML::Node logs = cfg_["logs"];
  ASSERT_TRUE(logs && logs.IsSequence()) << "the shipped config must declare its `logs:`";
  std::vector<std::pair<std::string, std::string>> seen;
  for (const auto& e : logs) {
    seen.emplace_back(e["msg_type"].as<std::string>(), e["instance"].as<std::string>());
  }
  const std::vector<std::pair<std::string, std::string>> expected = {
      {"rtc_msgs/DeviceStateLog", "ur5e_state"},
      {"rtc_msgs/DeviceStateLog", "p1b_state"},
      {"integrated_bringup/InferenceDiagLog", "inference_diag"}};
  EXPECT_EQ(seen, expected);
}

TEST_F(ShippedInference, TheDiagFileIsOpenedUnderTheSessionWithTheReachTipColumns) {
  namespace fs = std::filesystem;
  const fs::path dir = kSession->Dir() / "controllers" / "demo_inference_controller";
  const fs::path diag = dir / "inference_diag.csv";
  ASSERT_TRUE(fs::exists(diag)) << "configure must open " << diag;
  EXPECT_TRUE(fs::exists(dir / "ur5e_state.csv"));
  EXPECT_TRUE(fs::exists(dir / "p1b_state.csv"));
  // Earlier tests in this binary append to the same file; only rows written
  // from here on belong to this controller.
  const auto offset = fs::file_size(diag);

  const auto state = MakePregraspState();
  constexpr int kTicks = 40;  // below the 512-row ring: nothing may drop before the drain
  for (int t = 0; t < kTicks; ++t) {
    static_cast<void>(ctrl_->Compute(state));
  }
  ASSERT_EQ(ctrl_->on_deactivate(rclcpp_lifecycle::State{}), CR::SUCCESS);  // drains

  std::ifstream in(diag);
  std::string header;
  ASSERT_TRUE(std::getline(in, header));
  EXPECT_EQ(header.rfind("t_relative_s,", 0), 0U) << header;
  EXPECT_NE(header.find(",force_thumb,force_index,force_middle,force_ring"), std::string::npos)
      << "the tip columns are the reach gate's force groups, in its tip order: " << header;
  in.clear();
  in.seekg(static_cast<std::streamoff>(offset));
  int rows = 0;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line.rfind("t_relative_s", 0) == 0) {
      continue;
    }
    ++rows;
    // Third column is `held`.
    std::istringstream fields(line);
    std::string field;
    for (int c = 0; c < 3; ++c) {
      std::getline(fields, field, ',');
    }
    EXPECT_EQ(field, "1") << "no object was published, so every tick holds: " << line;
  }
  EXPECT_EQ(rows, kTicks) << "one row per tick";
}

TEST_F(ShippedInference, EveryHeldTickNamesItsReason) {
  using integrated_bringup::InferenceHoldReason;
  const auto state = MakePregraspState();

  // No object yet. The projection first walks in from its reference seed —
  // those ticks hold on the closed chain, whose fingertip poses are not yet
  // this configuration's — and then, with the chain converged, the missing
  // object is what holds.
  InferenceHoldReason reason = InferenceHoldReason::kNone;
  for (int t = 0; t < 600 && reason != InferenceHoldReason::kObject; ++t) {
    last_out_ = ctrl_->Compute(state);
    ASSERT_TRUE(ctrl_->LastTickHeldForTesting());
    reason = ctrl_->LastHoldReasonForTesting();
    ASSERT_TRUE(reason == InferenceHoldReason::kClosedChain ||
                reason == InferenceHoldReason::kObject)
        << "tick " << t << " held for reason " << static_cast<int>(reason);
  }
  EXPECT_EQ(reason, InferenceHoldReason::kObject);
  EXPECT_GT(ctrl_->HoldCountForTesting(InferenceHoldReason::kClosedChain), 0U)
      << "the walk-in from the reference seed must be visible as closed-chain holds";
  EXPECT_EQ(ctrl_->HoldCountForTesting(InferenceHoldReason::kUnreadable), 0U);

  // A held tick's goal is the latch, not a policy target it does not have.
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(last_out_.devices[0].goal_positions[i], last_out_.devices[0].commands[i]);
  }

  // An unreadable arm is named as such, not as the object it also lacks.
  auto holed = state;
  holed.devices[0].hole_mask = 0b1U;
  last_out_ = ctrl_->Compute(holed);
  EXPECT_EQ(ctrl_->LastHoldReasonForTesting(), InferenceHoldReason::kUnreadable);
}

TEST_F(ShippedInference, TheDiagRowReportsWhatThePolicyWasShown) {
  using integrated_bringup::InferenceDiagLogPod;
  using integrated_bringup::InferenceHoldReason;
  rtc::ThreadCsvProducer<InferenceDiagLogPod, 512> producer;
  ctrl_->SetInferenceDiagLogHandleForTesting(rtc::LogHandle<InferenceDiagLogPod>(&producer));
  std::vector<InferenceDiagLogPod> rows;
  const auto drain = [&] {
    static_cast<void>(producer.Drain([&](const InferenceDiagLogPod& p) { rows.push_back(p); }));
  };

  auto state = MakePregraspState();
  // Distinct sub-threshold forces, so each column is traceable to its group
  // without arming the tactile hold.
  const auto sensors = fx::MakeInferenceDeviceConfigs().at("p1b").sensor_names;
  const std::vector<std::pair<std::string, float>> forces = {
      {"thumb", 0.1F}, {"index", 0.2F}, {"middle", 0.3F}, {"ring", 0.4F}};
  for (const auto& [group, f] : forces) {
    SetForce(state, static_cast<int>(IndexOfName(sensors, group)), f);
  }
  InjectObjectInWorld(kNominalObject, Eigen::Quaterniond::Identity());
  // A NON-zero arm head, distinct per joint: with the default zeros the policy
  // target coincides with an unset `goal_positions`, and both the goal and the
  // lag checks below would pass with those lanes not written at all.
  std::array<double, 6> arm_target{};
  auto& arm_head = engine_->next_output[Out("arm_action")];
  for (std::size_t i = 0; i < arm_target.size(); ++i) {
    arm_target[i] = kC18[i] + (0.01 * static_cast<double>(i + 1));
    arm_head[i] = static_cast<float>(arm_target[i]);
  }

  const auto before = ctrl_->InferenceCountForTesting();
  bool accepted = false;
  for (int t = 0; t < 600 && !accepted; ++t) {
    Republish();
    last_out_ = ctrl_->Compute(state);
    drain();
    accepted = !ctrl_->LastTickHeldForTesting() && ctrl_->InferenceCountForTesting() > before;
  }
  ASSERT_TRUE(accepted);
  ASSERT_FALSE(rows.empty());
  const auto step = rows.back();

  EXPECT_FALSE(step.held);
  EXPECT_EQ(step.hold_reason, static_cast<std::uint8_t>(InferenceHoldReason::kNone));
  EXPECT_TRUE(step.policy_step);
  EXPECT_EQ(step.inference_count, ctrl_->InferenceCountForTesting());
  EXPECT_NEAR(step.reach_phase, Tensor("reach_phase")[0], 1e-6)
      << "the row must carry the gate value the policy was fed";
  EXPECT_TRUE(std::isfinite(step.tip_distance));
  EXPECT_GT(step.tip_distance, 0.0);
  EXPECT_TRUE(step.object_valid);
  for (std::size_t c = 0; c < 3; ++c) {
    EXPECT_NEAR(step.object_position[c], kNominalObject[static_cast<Eigen::Index>(c)], 1e-9)
        << "policy frame, axis " << c;
  }
  EXPECT_FALSE(step.closed_held);
  EXPECT_LT(step.closure_error, DemoInferenceController::kClosureErrorThreshold);
  ASSERT_EQ(step.num_tips, 4U);
  for (std::size_t i = 0; i < forces.size(); ++i) {
    EXPECT_NEAR(step.tip_force[i], forces[i].second, 1e-6) << forces[i].first;
  }
  // The arm stands at C18 and the target is C18 + 0.01·(i+1), so the lag is the
  // last joint's 0.06 — and goal_positions carries the target itself, not the
  // rate-bounded command (which may only step one tick of v_max from C18).
  EXPECT_NEAR(step.arm_lag_max, 0.06, 1e-6);
  for (std::size_t i = 0; i < arm_target.size(); ++i) {
    EXPECT_NEAR(last_out_.devices[0].goal_positions[i], arm_target[i], 1e-6)
        << "goal_positions carries the policy's own target, joint " << i;
  }

  // Between steps the action is replayed: not held, not a step, and the gate
  // value the policy saw stays on the row.
  last_out_ = ctrl_->Compute(state);
  drain();
  const auto replay = rows.back();
  EXPECT_FALSE(replay.held);
  EXPECT_FALSE(replay.policy_step);
  EXPECT_DOUBLE_EQ(replay.reach_phase, step.reach_phase);
}

// ── The shipped pole overlay (sim_overlays/inference_pole.yaml) ─────────────
//
// It exists to put the sim where the policy was trained, and every value in it
// is derived from something this suite already pins — so derive them again here
// instead of trusting the file.

namespace {

YAML::Node PoleOverlay() {
  const std::string path =
      std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/ur5e_p1b/sim_overlays/inference_pole.yaml";
  return YAML::LoadFile(path)["mujoco_simulator"]["ros__parameters"];
}

}  // namespace

TEST(ShippedPoleOverlay, StartsTheHandAtThePolicysSynergyZeroShapeInThisHandsConvention) {
  const YAML::Node overlay = PoleOverlay();
  // The sim's command order comes from the profile's own sim config, not from
  // a literal, so a reordered roster fails here instead of shipping a hand
  // whose joints are shuffled.
  const std::string sim_path =
      std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/ur5e_p1b/mujoco_simulator.yaml";
  const auto order = YAML::LoadFile(sim_path)["mujoco_simulator"]["ros__parameters"]
                                             ["robot_response"]["p1b"]["command_joint_names"]
                                                 .as<std::vector<std::string>>();
  const auto q0 = overlay["robot_response"]["p1b"]["initial_qpos"].as<std::vector<double>>();
  ASSERT_EQ(q0.size(), order.size());
  for (std::size_t j = 0; j < order.size(); ++j) {
    const double expected = kPolicySign.at(order[j]) * kC20[IndexOfName(kGripperOrder, order[j])];
    EXPECT_NEAR(q0[j], expected, 1e-6) << order[j];
  }
  EXPECT_EQ(overlay["robot_response"]["ur5e"]["initial_qpos"].as<std::vector<double>>().size(), 6U);
}

TEST(ShippedPoleOverlay, PutsThePoleAtTheTrainingNominalInThePolicyFrame) {
  const YAML::Node overlay = PoleOverlay();
  const YAML::Node pool = overlay["object_pool"];
  EXPECT_EQ(pool["directory"].as<std::string>(), "package://robot_descriptions/objects");
  EXPECT_EQ(pool["objects"].as<std::vector<std::string>>(), std::vector<std::string>{"pole"});
  EXPECT_EQ(pool["selection"].as<std::string>(), "fixed");
  EXPECT_EQ(pool["pose"].as<std::string>(), "fixed");
  const std::string model = overlay["model_path"].as<std::string>();
  EXPECT_EQ(model.substr(model.size() - std::string("/ur5e_p1b/mjcf/scene.xml").size()),
            "/ur5e_p1b/mjcf/scene.xml")
      << "the training scene is a floor at z = 0, not the table scene";

  const auto p = pool["position"].as<std::vector<double>>();
  const auto rpy = pool["rpy"].as<std::vector<double>>();
  ASSERT_EQ(p.size(), 3U);
  ASSERT_EQ(rpy.size(), 3U);
  // The sim world IS the policy frame (both are URDF `base`), so what the
  // overlay spawns is literally what the policy will be shown — the pole under
  // the reset posture's palm, not on the far side of the robot.
  const Eigen::Vector3d p_pf(p[0], p[1], p[2]);
  EXPECT_NEAR(p_pf.x(), kNominalObject.x(), 1e-9);
  EXPECT_NEAR(p_pf.y(), kNominalObject.y(), 1e-9);
  // The centre rests at 0.075 (h 0.15 on a z = 0 floor); a few mm above so the
  // spawn does not start interpenetrating, and no more than that.
  EXPECT_GT(p_pf.z(), kNominalObject.z());
  EXPECT_LE(p_pf.z(), kNominalObject.z() + 0.010);
  // ZYX: R = Rz(yaw) Ry(pitch) Rx(roll). The training pole's axis points DOWN.
  const Eigen::Matrix3d r_world = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
                                      .toRotationMatrix();
  const Eigen::Vector3d axis_pf = r_world * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(axis_pf.z(), -1.0, 1e-9) << "object z axis must point down in the policy frame";
}
