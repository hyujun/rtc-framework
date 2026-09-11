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

#include "inference_fake_engine.hpp"
#include "integrated_bringup/controllers/demo_inference_controller.hpp"
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
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace fx = integrated_bringup::testfx;
using integrated_bringup::DemoInferenceController;
using integrated_bringup::testfx::FakeEngine;
using rtc::ControllerState;
using CR = rtc::RTControllerInterface::CallbackReturn;

// ── Policy constants (ONNX initializers, policy frame / policy convention) ──
constexpr std::array<double, 6> kC18 = {-0.011404, -1.148533, 1.720851,
                                        -2.153859, -2.254593, 2.771491};
// C20 in the gripper head's order (index → ring → thumb → middle).
const std::vector<std::string> kGripperOrder = {
    "index_mcp_aa_joint",  "index_mcp_fe_joint", "index_dip_fe_joint", "ring_mcp_fe_joint",
    "thumb_cmc_aa_joint",  "thumb_cmc_fe_joint", "thumb_mcp_joint",    "thumb_dip_fe_joint",
    "middle_mcp_fe_joint", "middle_dip_fe_joint"};
constexpr std::array<double, 10> kC20 = {0.064476,  0.557459, 0.440156, 0.45242,  -1.426097,
                                         -0.429778, 0.049478, 0.609149, 0.529181, 0.511366};
// The training asset's joint signs (q_policy = s · q_device). The oracle's
// premise, stated here independently of the shipped YAML so the YAML is what
// is being checked.
const std::map<std::string, double> kPolicySign = {
    {"thumb_cmc_aa_joint", -1.0}, {"thumb_cmc_fe_joint", 1.0},   {"thumb_mcp_joint", 1.0},
    {"thumb_dip_fe_joint", -1.0}, {"index_mcp_aa_joint", -1.0},  {"index_mcp_fe_joint", -1.0},
    {"index_dip_fe_joint", -1.0}, {"middle_mcp_fe_joint", -1.0}, {"middle_dip_fe_joint", -1.0},
    {"ring_mcp_fe_joint", -1.0}};
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
constexpr double kPalmTol = 1e-3;
constexpr double kTipTol = 2e-3;

constexpr int kInferenceStride = 7;

std::vector<std::size_t> Numels(const YAML::Node& tensors) {
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

std::size_t IndexOfName(const std::vector<std::string>& names, const std::string& name) {
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i] == name) {
      return i;
    }
  }
  ADD_FAILURE() << "no element named '" << name << "'";
  return names.size();
}

/// The fixture's device configs plus what the p1b hand needs here and the
/// fixture does not carry: the inference lane stride (force features) and the
/// hand's position band (the command tail), both as _base.yaml ships them.
std::map<std::string, rtc::DeviceNameConfig> MakeDevices() {
  auto devices = fx::MakeUr5eP1bDeviceConfigs();
  auto& hand = devices.at("p1b");
  rtc::DeviceSensorLayout layout;
  layout.inference_values_per_group = kInferenceStride;
  hand.sensor_layout = layout;
  rtc::DeviceJointLimits lim;
  lim.max_velocity.assign(fx::kP1bHandDof, 10.384);
  lim.position_lower = {0.0,        -1.5707963, -1.5707963, -1.5707963, -0.34906585,
                        -1.5707963, -1.5707963, -1.5707963, -1.5707963, -1.5707963};
  lim.position_upper = {2.356194487, 1.5707963, 1.5707963, 0.0, 0.523598776,
                        0.0,         0.0,       0.0,       0.0, 0.0};
  hand.joint_limits = lim;
  return devices;
}

/// Arm at C18, hand at s ⊙ C20 in DEVICE order — the training pregrasp.
ControllerState MakePregraspState() {
  auto state = fx::MakeUr5eP1bState();
  for (std::size_t i = 0; i < kC18.size(); ++i) {
    state.devices[0].positions[i] = kC18[i];
  }
  const auto devices = fx::MakeUr5eP1bDeviceConfigs();
  const auto& hand_names = devices.at("p1b").joint_state_names;
  for (std::size_t j = 0; j < hand_names.size(); ++j) {
    const auto k = IndexOfName(kGripperOrder, hand_names[j]);
    state.devices[1].positions[j] = kPolicySign.at(hand_names[j]) * kC20[k];
  }
  return state;
}

tf2_msgs::msg::TFMessage MakeObjectTf(const Eigen::Vector3d& p_world,
                                      const Eigen::Quaterniond& q_world) {
  tf2_msgs::msg::TFMessage msg;
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "world";
  tf.child_frame_id = "pool_pole_object";
  tf.transform.translation.x = p_world.x();
  tf.transform.translation.y = p_world.y();
  tf.transform.translation.z = p_world.z();
  tf.transform.rotation.x = q_world.x();
  tf.transform.rotation.y = q_world.y();
  tf.transform.rotation.z = q_world.z();
  tf.transform.rotation.w = q_world.w();
  msg.transforms.push_back(tf);
  return msg;
}

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

/// The production bring-up order on the real model, driven with the SHIPPED
/// controller config (model_path aside).
class ShippedInference : public ::testing::Test {
 protected:
  void SetUp() override {
    cfg_ = fx::ShippedControllerNode("ur5e_p1b", "demo_inference_controller");
    shipped_model_path_ = cfg_["inference"]["model_path"].as<std::string>("");
    cfg_["inference"]["model_path"] = "fake_policy.onnx";

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
    ctrl_->SetDeviceNameConfigs(MakeDevices());
    configured_ = ctrl_->on_configure(rclcpp_lifecycle::State{}, node_, cfg_) == CR::SUCCESS;
    ASSERT_TRUE(configured_) << "the shipped config must configure against the real model";
    ASSERT_EQ(ctrl_->on_activate(rclcpp_lifecycle::State{}), CR::SUCCESS);
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
  void InjectObjectInWorld(const Eigen::Vector3d& p_world, const Eigen::Quaterniond& q_world) {
    object_msg_ = MakeObjectTf(p_world, q_world);
    Republish();
  }

  /// The object at a pose given in the POLICY frame, published in `world`
  /// (= URDF `base`, a half turn about z away).
  void InjectObjectInPolicyFrame(const Eigen::Vector3d& p_pf, const Eigen::Quaterniond& q_pf) {
    const Eigen::Quaterniond world_from_pf(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    InjectObjectInWorld(world_from_pf * p_pf, world_from_pf * q_pf);
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
  EXPECT_EQ(inf["policy_frame"].as<std::string>(), "base_link");
  ASSERT_TRUE(inf["object_pose"]["source_frame_link"]);
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
  const std::vector<std::string> links = {"l_thumb_tip_link", "l_index_tip_link",
                                          "l_middle_tip_link", "l_ring_tip_link"};
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

// ── §5 criterion 3: frame, convention and closed chain against MuJoCo ──────

TEST_F(ShippedInference, ObservesTheTrainingPregraspWhereMuJoCoPutsIt) {
  const auto state = MakePregraspState();
  InjectObjectInPolicyFrame({0.5, 0.05, 0.075}, Eigen::Quaterniond::Identity());
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

  // Body table: palm and fingertips where MuJoCo puts them, in base_link.
  {
    const auto& names = io.inputs[In("robot_body_pos_w")].element_names;
    const auto& pos = Tensor("robot_body_pos_w");
    const auto row = [&](const std::string& link) { return 3 * IndexOfName(names, link); };
    for (std::size_t c = 0; c < 3; ++c) {
      EXPECT_NEAR(pos[row("l_palm_link") + c], kPalmPos[c], kPalmTol) << "palm axis " << c;
    }
    for (const auto& [link, p] : kTipPos) {
      for (std::size_t c = 0; c < 3; ++c) {
        EXPECT_NEAR(pos[row(link) + c], p[c], kTipTol) << link << " axis " << c;
      }
    }
    EXPECT_FLOAT_EQ(pos[row("tool0")], 0.0F) << "an unobserved row holds the filler";

    const auto& qnames = io.inputs[In("robot_body_quat_w")].element_names;
    const auto& quat = Tensor("robot_body_quat_w");
    const std::size_t prow = 4 * IndexOfName(qnames, "l_palm_link");
    // The oracle carries four decimals, so compare ROTATION ANGLE against the
    // normalised oracle rather than components (q ≡ −q).
    const Eigen::Quaterniond oracle =
        Eigen::Quaterniond(kPalmQuatXyzw[3], kPalmQuatXyzw[0], kPalmQuatXyzw[1], kPalmQuatXyzw[2])
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

TEST_F(ShippedInference, TheObjectPoseIsRotatedIntoThePolicyFrameByTheModel) {
  // The world point (1, 2, 3) is (−1, −2, 3) in base_link: `world` is URDF
  // `base`, a half turn about z from base_link. The rotation used to be typed
  // into the YAML (`base_pose_in_world`); it now comes from the URDF, and this
  // is the same assertion against the new source.
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
  InjectObjectInPolicyFrame({0.5, 0.05, 0.075}, Eigen::Quaterniond::Identity());

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
  InjectObjectInPolicyFrame({0.5, 0.05, 0.075}, Eigen::Quaterniond::Identity());
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

TEST_F(ShippedInference, ReachPhaseIsTheProximityOfTheTipsToTheObjectsContactPoints) {
  const auto state = MakePregraspState();
  // First step: object far away, to learn where the tips are.
  InjectObjectInPolicyFrame({2.0, 2.0, 2.0}, Eigen::Quaterniond::Identity());
  ASSERT_TRUE(RunUntilAccepted(state));
  const auto& io = ctrl_->IoParamsForTesting();
  const auto& names = io.inputs[In("robot_body_pos_w")].element_names;
  const std::vector<std::string> tips = {"l_thumb_tip_link", "l_index_tip_link",
                                         "l_middle_tip_link", "l_ring_tip_link"};
  std::array<Eigen::Vector3d, 4> tip{};
  for (std::size_t i = 0; i < tips.size(); ++i) {
    const auto r = 3 * IndexOfName(names, tips[i]);
    const auto& pos = Tensor("robot_body_pos_w");
    tip[i] = Eigen::Vector3d(pos[r], pos[r + 1], pos[r + 2]);
  }
  EXPECT_LT(Tensor("reach_phase")[0], 1e-6F) << "two metres away is no proximity";

  // Now place the object where the tips are closest to their contact points
  // (the pole upside down, as trained): the least-squares object origin for a
  // fixed orientation is the mean of tip_i − R·c_i.
  const Eigen::Quaterniond q_obj(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
  Eigen::Vector3d p_obj = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < 4; ++i) {
    p_obj += tip[i] - q_obj * Eigen::Vector3d(kC0[i][0], kC0[i][1], kC0[i][2]);
  }
  p_obj /= 4.0;
  double mean_d = 0.0;
  for (std::size_t i = 0; i < 4; ++i) {
    mean_d += (tip[i] - (p_obj + q_obj * Eigen::Vector3d(kC0[i][0], kC0[i][1], kC0[i][2]))).norm();
  }
  mean_d /= 4.0;
  const double expected = std::exp(-(mean_d / 0.030) * (mean_d / 0.030));
  ASSERT_GT(expected, 0.05) << "the fixture must put the gate on its ramp, or this proves nothing";
  ASSERT_LT(expected, 0.999);

  InjectObjectInPolicyFrame(p_obj, q_obj);
  RunOnePolicyStep(state);
  EXPECT_NEAR(Tensor("reach_phase")[0], expected, 1e-4)
      << "mean tip distance " << mean_d << " m against the model's contact points";
}

TEST_F(ShippedInference, TheTactileHoldCountsPolicyStepsAndReadsTheRightFingers) {
  auto state = MakePregraspState();
  InjectObjectInPolicyFrame({2.0, 2.0, 2.0}, Eigen::Quaterniond::Identity());
  ASSERT_TRUE(RunUntilAccepted(state));

  // Thumb + index + middle pressing, ring not: a grasp only if the FIRST tip
  // reads the thumb group. A reversed mapping would put the silent ring first.
  const auto devices = MakeDevices();
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
