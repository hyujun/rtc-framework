// locomotion_controller.model.* (humanoid_model_config.hpp): every refusal next to the
// same rig accepted, so a check that accepts everything (or nothing) is red; then the
// shipped g1_loco files through the CM's own loader, the CM model and the shipped
// rosters; then the handoff to rtc_mpc's RobotModelHandler.
//
// The model is the g1_loco CM model (G1 + p1b surrogate hand; floating root, W1-F04),
// built the way the CM builds it
// (shipped_catching_bringup_fixture.hpp). No ROS node.
#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"
#include "rtc_controller_manager/controller_config_loader.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/model/wb_layout.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "shipped_catching_bringup_fixture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fx = integrated_bringup::testfx;
namespace loco = integrated_bringup::locomotion;

constexpr const char* kProfile = "g1_loco";
constexpr const char* kContext = "locomotion_controller.model";

// The rig every refusal starts from — accepted as is (the AcceptedRig test).
constexpr const char* kRig = R"(
body_group: "g1"
base_frame: "universe"
torso_frame: "torso_link"
end_effectors:
  - {name: "left_hand", frame: "left_wrist_yaw_link"}
  - {name: "right_hand", frame: "base_adapter"}
contacts:
  - {name: "left_foot", frame: "left_sole", dim: 6}
  - {name: "right_foot", frame: "right_sole", dim: 6}
hand_groups: ["p1b"]
)";

YAML::Node Rig() {
  return YAML::Load(kRig);
}

const pinocchio::Model& CmModel() {
  return *fx::ShippedModelBuilder(kProfile)->GetFullModel();
}

std::vector<std::string> BodyOrder() {
  return loco::JointOrder(*fx::ShippedModelBuilder(kProfile)->GetTreeModel("g1"));
}

// The shipped rosters (_base.yaml + sim.yaml devices.<group>.joint_state_names), read
// once; callers that edit them take a copy.
const std::map<std::string, rtc::DeviceNameConfig>& ShippedRosters() {
  static const std::map<std::string, rtc::DeviceNameConfig> shipped = [] {
    std::map<std::string, rtc::DeviceNameConfig> rosters;
    const YAML::Node devices = fx::ShippedSimParams(kProfile)["devices"];
    for (auto it = devices.begin(); it != devices.end(); ++it) {
      rtc::DeviceNameConfig cfg;
      cfg.device_name = it->first.as<std::string>();
      if (it->second["joint_state_names"]) {
        cfg.joint_state_names = it->second["joint_state_names"].as<std::vector<std::string>>();
      }
      rosters[cfg.device_name] = std::move(cfg);
    }
    return rosters;
  }();
  return shipped;
}

loco::DeviceLookup LookupIn(const std::map<std::string, rtc::DeviceNameConfig>& rosters) {
  return [&rosters](const std::string& group) -> const rtc::DeviceNameConfig* {
    const auto it = rosters.find(group);
    return it == rosters.end() ? nullptr : &it->second;
  };
}

// All three passes on `node`, the CM model and `rosters`.
void RunAll(const YAML::Node& node, const std::map<std::string, rtc::DeviceNameConfig>& rosters) {
  const loco::HumanoidModelConfig cfg = loco::LoadHumanoidModelConfig(node, kContext);
  loco::ValidateAgainstModel(cfg, CmModel(), kContext);
  loco::ValidateDevices(cfg, BodyOrder(), LookupIn(rosters), kContext);
}

// `edit` applied to the rig, on `rosters`, must be refused with a message containing
// `expect` (so the RIGHT check fired). The passing twin of every refusal is the
// unedited rig on the shipped rosters (AcceptedRig).
void ExpectRefused(const std::function<void(YAML::Node&)>& edit, const std::string& expect,
                   const std::map<std::string, rtc::DeviceNameConfig>& rosters = ShippedRosters()) {
  YAML::Node node = Rig();
  edit(node);
  try {
    RunAll(node, rosters);
    ADD_FAILURE() << "accepted; expected a refusal containing: " << expect;
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find(expect), std::string::npos)
        << "refused for another reason: " << e.what() << "\nexpected: " << expect;
  }
}

YAML::Node Contact(const std::string& name, const std::string& frame, int dim) {
  YAML::Node c;
  c["name"] = name;
  c["frame"] = frame;
  c["dim"] = dim;
  return c;
}

TEST(HumanoidModelConfig, AcceptedRig) {
  const loco::HumanoidModelConfig cfg = loco::LoadHumanoidModelConfig(Rig(), kContext);
  EXPECT_EQ(cfg.body_group, "g1");
  ASSERT_EQ(cfg.end_effectors.size(), 2U);
  EXPECT_EQ(cfg.end_effectors[1].frame, "base_adapter");
  ASSERT_EQ(cfg.contacts.size(), 2U);
  EXPECT_EQ(cfg.contacts[0].frame, "left_sole");
  EXPECT_EQ(cfg.contacts[1].dim, 6);
  EXPECT_EQ(cfg.hand_groups, std::vector<std::string>{"p1b"});
  EXPECT_NO_THROW(RunAll(Rig(), ShippedRosters()));
}

// ── Pass 1, schema ──────────────────────────────────────────────────────────

TEST(HumanoidModelConfig, RefusesAnUnknownKeyAndWordsItApartFromAMissingOne) {
  // A misspelling is both unknown and missing; it is reported as unknown.
  ExpectRefused(
      [](YAML::Node& n) {
        n["body_groups"] = n["body_group"];
        n.remove("body_group");
      },
      "model: unknown key 'body_groups'");
  ExpectRefused([](YAML::Node& n) { n["contacts"][1]["mu"] = 0.7; },
                "model.contacts[1]: unknown key 'mu'");
  ExpectRefused([](YAML::Node& n) { n.remove("hand_groups"); },
                "model: required key 'hand_groups' is missing");
  ExpectRefused([](YAML::Node& n) { n["end_effectors"][0].remove("frame"); },
                "model.end_effectors[0]: required key 'frame' is missing");
  ExpectRefused([](YAML::Node& n) { n["contacts"][0].remove("dim"); },
                "model.contacts[0]: required key 'dim' is missing");
}

TEST(HumanoidModelConfig, RefusesANonNameAndANonList) {
  ExpectRefused([](YAML::Node& n) { n["torso_frame"] = YAML::Load("[torso_link]"); },
                "model.torso_frame: expected a non-empty name");
  ExpectRefused([](YAML::Node& n) { n["contacts"][0]["frame"] = ""; },
                "model.contacts[0].frame: expected a non-empty name");
  ExpectRefused([](YAML::Node& n) { n["contacts"] = "left_sole"; },
                "model.contacts: expected a list");
  ExpectRefused([](YAML::Node& n) { n["hand_groups"] = YAML::Load("[{a: 1}]"); },
                "model.hand_groups[0]: expected a non-empty name");
}

TEST(HumanoidModelConfig, RefusesAContactCountOutsideOneToTheMpcCapacity) {
  ExpectRefused([](YAML::Node& n) { n["contacts"] = YAML::Load("[]"); },
                "model.contacts: expected 1 to");
  ExpectRefused(
      [](YAML::Node& n) {
        YAML::Node many(YAML::NodeType::Sequence);
        for (int i = 0; i <= rtc::mpc::kMaxContacts; ++i) {
          many.push_back(Contact("c" + std::to_string(i), "f" + std::to_string(i), 3));
        }
        n["contacts"] = many;
      },
      "model.contacts: expected 1 to");
  // Within the count but over the force-variable capacity: 5 surfaces = 30 > 24.
  ExpectRefused(
      [](YAML::Node& n) {
        YAML::Node five(YAML::NodeType::Sequence);
        for (int i = 0; i < 5; ++i) {
          five.push_back(Contact("c" + std::to_string(i), "f" + std::to_string(i), 6));
        }
        n["contacts"] = five;
      },
      "force variables");
}

TEST(HumanoidModelConfig, RefusesADimOtherThanThreeOrSix) {
  for (const char* dim : {"0", "7", "6.0", "surface"}) {
    ExpectRefused([dim](YAML::Node& n) { n["contacts"][1]["dim"] = YAML::Load(dim); },
                  "model.contacts[1].dim: expected 3 (point) or 6 (surface)");
  }
  // 3 is accepted (point contact) — the twin of the refusals above.
  YAML::Node point = Rig();
  point["contacts"][1]["dim"] = 3;
  EXPECT_EQ(loco::LoadHumanoidModelConfig(point, kContext).contacts[1].dim, 3);
}

TEST(HumanoidModelConfig, RefusesAnEndEffectorCountOutsideOneToTwo) {
  ExpectRefused([](YAML::Node& n) { n["end_effectors"] = YAML::Load("[]"); },
                "model.end_effectors: expected 1 to 2");
  ExpectRefused(
      [](YAML::Node& n) {
        YAML::Node third;
        third["name"] = "third_hand";
        third["frame"] = "torso_link";
        n["end_effectors"].push_back(third);
      },
      "model.end_effectors: expected 1 to 2");
  // One end-effector is accepted.
  YAML::Node one = Rig();
  one["end_effectors"] = YAML::Load(R"([{name: "left_hand", frame: "left_wrist_yaw_link"}])");
  EXPECT_NO_THROW(RunAll(one, ShippedRosters()));
}

TEST(HumanoidModelConfig, RefusesANameOrFrameUsedTwiceAcrossContactsAndEndEffectors) {
  ExpectRefused([](YAML::Node& n) { n["contacts"][1]["name"] = "left_foot"; },
                "model.contacts[1].name: name 'left_foot' is used twice");
  ExpectRefused([](YAML::Node& n) { n["contacts"][0]["name"] = "right_hand"; },
                "model.contacts[0].name: name 'right_hand' is used twice");
  ExpectRefused([](YAML::Node& n) { n["contacts"][1]["frame"] = "left_sole"; },
                "model.contacts[1].frame: frame 'left_sole' is used twice");
  ExpectRefused([](YAML::Node& n) { n["contacts"][0]["frame"] = "base_adapter"; },
                "model.contacts[0].frame: frame 'base_adapter' is used twice");
  ExpectRefused([](YAML::Node& n) { n["torso_frame"] = "left_sole"; },
                "model.torso_frame: frame 'left_sole' is also a contact or end-effector frame");
}

TEST(HumanoidModelConfig, RefusesADeviceGroupNamedTwice) {
  ExpectRefused([](YAML::Node& n) { n["hand_groups"] = YAML::Load("[g1]"); },
                "model.hand_groups[0]: device group 'g1' is used twice");
  ExpectRefused([](YAML::Node& n) { n["hand_groups"] = YAML::Load("[p1b, p1b]"); },
                "model.hand_groups[1]: device group 'p1b' is used twice");
}

// ── Pass 1, model-derived ───────────────────────────────────────────────────

TEST(HumanoidModelConfig, RefusesAFrameTheModelDoesNotHave) {
  ExpectRefused([](YAML::Node& n) { n["contacts"][0]["frame"] = "left_ankle_sole"; },
                "model.contacts[0].frame: frame 'left_ankle_sole' is not in the model");
  ExpectRefused([](YAML::Node& n) { n["torso_frame"] = "torso"; },
                "model.torso_frame: frame 'torso' is not in the model");
  ExpectRefused([](YAML::Node& n) { n["end_effectors"][1]["frame"] = "right_hand_palm"; },
                "model.end_effectors[1].frame: frame 'right_hand_palm' is not in the model");
}

// The shipped CM model has a floating root (W1-F04) — the model this rule is for; the
// same urdf: block with a fixed root is the model a pelvis base is valid on.
const pinocchio::Model& FixedCmModel() {
  static const std::shared_ptr<const pinocchio::Model> model = [] {
    rtc_urdf_bridge::ModelConfig cfg = fx::ShippedModelConfig(kProfile);
    cfg.root_joint_type = "fixed";
    return rtc_urdf_bridge::PinocchioModelBuilder(cfg).GetFullModel();
  }();
  return *model;
}

TEST(HumanoidModelConfig, RefusesANonUniverseBaseOnTheFloatingModel) {
  YAML::Node pelvis = Rig();
  pelvis["base_frame"] = "pelvis";
  const loco::HumanoidModelConfig on_pelvis = loco::LoadHumanoidModelConfig(pelvis, kContext);
  const loco::HumanoidModelConfig on_universe = loco::LoadHumanoidModelConfig(Rig(), kContext);
  // Floating: universe passes, pelvis is refused.
  EXPECT_NO_THROW(loco::ValidateAgainstModel(on_universe, CmModel(), kContext));
  try {
    loco::ValidateAgainstModel(on_pelvis, CmModel(), kContext);
    ADD_FAILURE() << "a floating model with base_frame pelvis was accepted";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("model.base_frame: the model has a floating base"),
              std::string::npos)
        << e.what();
  }
  // Fixed: pelvis is a valid base.
  EXPECT_NO_THROW(loco::ValidateAgainstModel(on_pelvis, FixedCmModel(), kContext));
}

TEST(HumanoidModelConfig, RefusesAContactOnTheBaseFrame) {
  ExpectRefused([](YAML::Node& n) { n["contacts"][0]["frame"] = "universe"; },
                "a contact cannot be the base frame 'universe'");
}

// ── Pass 3, device-derived ──────────────────────────────────────────────────

TEST(HumanoidModelConfig, RefusesAMissingDeviceGroup) {
  ExpectRefused([](YAML::Node& n) { n["body_group"] = "b7"; },
                "model.body_group: no device group 'b7'");
  ExpectRefused([](YAML::Node& n) { n["hand_groups"] = YAML::Load("[p2_right]"); },
                "model.hand_groups[0]: no device group 'p2_right'");
  const loco::HumanoidModelConfig cfg = loco::LoadHumanoidModelConfig(Rig(), kContext);
  EXPECT_THROW(loco::ValidateDevices(cfg, BodyOrder(), nullptr, kContext), std::invalid_argument);
}

TEST(HumanoidModelConfig, RefusesAnEmptyRosterInsteadOfTakingTheModelOrder) {
  std::map<std::string, rtc::DeviceNameConfig> rosters = ShippedRosters();
  rosters["g1"].joint_state_names.clear();
  ExpectRefused([](YAML::Node&) {}, "has no joint_state_names", rosters);
}

TEST(HumanoidModelConfig, RefusesAHandGroupWithoutARoster) {
  std::map<std::string, rtc::DeviceNameConfig> rosters = ShippedRosters();
  rosters["p1b"].joint_state_names.clear();
  ExpectRefused([](YAML::Node&) {},
                "model.hand_groups[0]: device group 'p1b' has no joint_state_names", rosters);
}

TEST(HumanoidModelConfig, HandlerYamlRefusesAConfigWithoutAnEndEffector) {
  loco::HumanoidModelConfig cfg = loco::LoadHumanoidModelConfig(Rig(), kContext);
  EXPECT_NO_THROW((void)loco::ToRobotModelHandlerYaml(cfg));
  cfg.end_effectors.clear();
  EXPECT_THROW((void)loco::ToRobotModelHandlerYaml(cfg), std::invalid_argument);
}

TEST(HumanoidModelConfig, RefusesARosterOutOfTheModelOrderAndSaysWhere) {
  std::map<std::string, rtc::DeviceNameConfig> rosters = ShippedRosters();
  auto& names = rosters["g1"].joint_state_names;
  ASSERT_GT(names.size(), 5U);
  std::swap(names[3], names[5]);
  const std::vector<std::string> order = BodyOrder();
  ExpectRefused([](YAML::Node&) {},
                "(index 3: expected '" + order[3] + "', actual '" + order[5] + "')", rosters);
  std::map<std::string, rtc::DeviceNameConfig> shorter = ShippedRosters();
  shorter["g1"].joint_state_names.pop_back();
  ExpectRefused([](YAML::Node&) {},
                "(index " + std::to_string(order.size() - 1) + ": expected '" + order.back() +
                    "', actual '<end>')",
                shorter);
}

// ── Shipped files and the rtc_mpc handoff ───────────────────────────────────

YAML::Node InstalledModelSection() {
  const std::string main = ament_index_cpp::get_package_share_directory("integrated_bringup") +
                           "/config/g1_loco/controllers/locomotion_controller.yaml";
  return rtc::LoadControllerConfig(main, "locomotion_controller")["model"];
}

TEST(HumanoidModelConfig, TheInstalledG1LocoFilesPassAllThreePasses) {
  const YAML::Node model = InstalledModelSection();
  ASSERT_TRUE(model.IsMap()) << "the include: merge did not bring model.* in";
  EXPECT_NO_THROW(RunAll(model, ShippedRosters()));
}

TEST(HumanoidModelConfig, TheHandlerYamlInitsRobotModelHandlerWithTheContactOrder) {
  const loco::HumanoidModelConfig cfg =
      loco::LoadHumanoidModelConfig(InstalledModelSection(), kContext);
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(CmModel(), loco::ToRobotModelHandlerYaml(cfg)),
            rtc::mpc::RobotModelInitError::kNoError);
  // WD-36: the handler's one end-effector is end_effectors[0].
  EXPECT_EQ(handler.end_effector_frame_id(),
            static_cast<int>(CmModel().getFrameId(cfg.end_effectors.front().frame)));
  ASSERT_EQ(handler.n_contacts(), static_cast<int>(cfg.contacts.size()));
  for (std::size_t i = 0; i < cfg.contacts.size(); ++i) {
    const auto& info = handler.contact_frames()[i];
    EXPECT_EQ(info.name, cfg.contacts[i].frame) << i;
    EXPECT_EQ(info.frame_id, static_cast<int>(CmModel().getFrameId(cfg.contacts[i].frame))) << i;
    EXPECT_EQ(info.dim, cfg.contacts[i].dim) << i;
  }
}

}  // namespace
