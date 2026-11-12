// g1_loco: the body's joint order is one order in all three places that state it
// (W1-F03 WG-1). The controller indexes the body by position, so a swap anywhere is a
// mirrored or crossed command, not an error:
//   (a) the CM model — tree model `g1` of the shipped urdf: block, configuration order
//   (b) the device rosters — _base.yaml devices.g1.joint_state_names and the simulator's
//       robot_response.g1.command_joint_names (mujoco_simulator.yaml)
//   (c) the plant — the MJCF <actuator> children in document order (MuJoCo numbers
//       actuators by appearance; test_g1_floating_stance pins actuator name == joint
//       name), and its `stand` keyframe's ctrl / qpos lengths
// The comparison is CompareJointOrder, the same one ValidateDevices refuses a roster
// with at configure (humanoid_model_config.hpp).
//
// b7_loco is NOT covered: its model is a private asset, and its MJCF interleaves the
// hands between the arms (the simulator matches by name), so the same test would be
// red by construction. Its order is checked by hand at bring-up.
#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"
#include "shipped_catching_bringup_fixture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>

#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fx = integrated_bringup::testfx;
namespace loco = integrated_bringup::locomotion;

constexpr const char* kProfile = "g1_loco";
constexpr const char* kBody = "g1";

std::vector<std::string> ModelOrder() {
  return loco::JointOrder(*fx::ShippedModelBuilder(kProfile)->GetTreeModel(kBody));
}

int ModelNv() {
  return fx::ShippedModelBuilder(kProfile)->GetTreeModel(kBody)->nv;
}

std::vector<std::string> DeviceRoster() {
  return fx::ShippedSimParams(kProfile)["devices"][kBody]["joint_state_names"]
      .as<std::vector<std::string>>();
}

std::vector<std::string> SimulatorRoster() {
  const YAML::Node sim =
      YAML::LoadFile(fx::ShippedProfilesDir() + "/" + kProfile + "/mujoco_simulator.yaml");
  return sim["mujoco_simulator"]["ros__parameters"]["robot_response"][kBody]["command_joint_names"]
      .as<std::vector<std::string>>();
}

std::string PlantMjcf() {
  return ament_index_cpp::get_package_share_directory("robot_descriptions") +
         "/robots/g1/mjcf/g1_29dof_floating.xml";
}

struct Plant {
  std::vector<std::string> actuator_joints;
  std::size_t stand_ctrl{0};
  std::size_t stand_qpos{0};
};

std::size_t CountNumbers(const char* text) {
  std::istringstream in(text == nullptr ? "" : text);
  std::size_t n = 0;
  double v = 0.0;
  while (in >> v) {
    ++n;
  }
  return n;
}

Plant ReadPlant() {
  tinyxml2::XMLDocument doc;
  EXPECT_EQ(doc.LoadFile(PlantMjcf().c_str()), tinyxml2::XML_SUCCESS) << PlantMjcf();
  Plant plant;
  const tinyxml2::XMLElement* root = doc.FirstChildElement("mujoco");
  if (root == nullptr) {
    return plant;
  }
  if (const auto* actuators = root->FirstChildElement("actuator")) {
    for (const auto* a = actuators->FirstChildElement(); a != nullptr;
         a = a->NextSiblingElement()) {
      const char* joint = a->Attribute("joint");
      plant.actuator_joints.emplace_back(joint == nullptr ? "<no joint>" : joint);
    }
  }
  if (const auto* keys = root->FirstChildElement("keyframe")) {
    for (const auto* k = keys->FirstChildElement("key"); k != nullptr;
         k = k->NextSiblingElement("key")) {
      if (const char* name = k->Attribute("name");
          name != nullptr && std::string(name) == "stand") {
        plant.stand_ctrl = CountNumbers(k->Attribute("ctrl"));
        plant.stand_qpos = CountNumbers(k->Attribute("qpos"));
      }
    }
  }
  return plant;
}

void ExpectSameOrder(const std::vector<std::string>& expected,
                     const std::vector<std::string>& actual, const char* what) {
  const auto mismatch = loco::CompareJointOrder(expected, actual);
  EXPECT_FALSE(mismatch.has_value())
      << what << " differs from the model order at " << loco::Describe(*mismatch);
}

TEST(G1LocoJointOrder, TheModelHasTheBodysDofAndNoEmptySource) {
  // Non-empty and the body's size, so an empty source cannot pass as "equal".
  EXPECT_EQ(static_cast<int>(ModelOrder().size()), ModelNv());
  EXPECT_EQ(ModelNv(), 23);
  EXPECT_EQ(DeviceRoster().size(), ModelOrder().size());
  EXPECT_EQ(SimulatorRoster().size(), ModelOrder().size());
  EXPECT_EQ(ReadPlant().actuator_joints.size(), ModelOrder().size());
}

TEST(G1LocoJointOrder, TheRostersAreTheModelOrder) {
  ExpectSameOrder(ModelOrder(), DeviceRoster(), "_base.yaml devices.g1.joint_state_names");
  ExpectSameOrder(ModelOrder(), SimulatorRoster(),
                  "mujoco_simulator.yaml robot_response.g1.command_joint_names");
}

TEST(G1LocoJointOrder, ThePlantsActuatorsAreTheModelOrder) {
  const Plant plant = ReadPlant();
  ExpectSameOrder(ModelOrder(), plant.actuator_joints, "the MJCF <actuator> order");
  // The keyframe the simulator resets to drives every actuator, over the plant's nq
  // (free joint 7 + the actuated joints).
  EXPECT_EQ(plant.stand_ctrl, ModelOrder().size());
  EXPECT_EQ(plant.stand_qpos, 7 + ModelOrder().size());
}

// Positive control: the shipped sources agree, so a comparison that saw nothing would
// also be green. Two swapped names must be found, at the first swapped index.
TEST(G1LocoJointOrder, ASwappedPairIsReportedWithIndexExpectedAndActual) {
  const std::vector<std::string> model = ModelOrder();
  std::vector<std::string> swapped = DeviceRoster();
  ASSERT_GT(swapped.size(), 12U);
  std::swap(swapped[6], swapped[12]);  // the right leg's first joint with the waist's
  const auto mismatch = loco::CompareJointOrder(model, swapped);
  ASSERT_TRUE(mismatch.has_value());
  EXPECT_EQ(mismatch->index, 6U);
  EXPECT_EQ(mismatch->expected, model[6]);
  EXPECT_EQ(mismatch->actual, model[12]);
  EXPECT_EQ(loco::Describe(*mismatch),
            "index 6: expected '" + model[6] + "', actual '" + model[12] + "'");
}

}  // namespace
