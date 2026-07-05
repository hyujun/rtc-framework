/// Unit tests for XML tree validation (Tier 3).
///
/// Validates that all shipped BT XML trees parse correctly against
/// the registered node types. Acts as a regression guard.

#include "ur5e_bt_coordinator/bt_node_registration.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <set>
#include <string>

using namespace rtc_bt;

namespace {

/// Extract the set of `hand_pose.<name>` / `arm_pose.<name>` keys declared in a
/// poses YAML file whose key starts with `prefix` (e.g. "hand_pose." /
/// "arm_pose.") — simple line scan, enough for the flat key:value layout the
/// coordinator's poses files use; avoids a yaml-cpp test dependency.
std::set<std::string> ReadPoseKeys(const std::filesystem::path& path, const std::string& prefix) {
  std::set<std::string> keys;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    const auto first = line.find_first_not_of(" \t");
    if (first == std::string::npos)
      continue;
    const std::string trimmed = line.substr(first);
    if (trimmed.rfind(prefix, 0) != 0)
      continue;
    const auto colon = trimmed.find(':');
    if (colon == std::string::npos)
      continue;
    keys.insert(trimmed.substr(0, colon));
  }
  return keys;
}

}  // namespace

class TreeValidationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Register via the same free function the coordinator uses (all
    // capabilities present → every node type), so the validated node set can
    // never drift from what the node actually registers.
    std::shared_ptr<BtRosBridge> null_bridge;
    rtc_bt::RegisterBtNodes(factory_, null_bridge);

    // Resolve trees + config directories
    try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("ur5e_bt_coordinator");
      trees_dir_ = std::filesystem::path(pkg_share) / "trees";
      config_dir_ = std::filesystem::path(pkg_share) / "config";
    } catch (...) {
      // Fallback to source tree
      auto src = std::filesystem::path(__FILE__).parent_path().parent_path();
      trees_dir_ = src / "trees";
      config_dir_ = src / "config";
    }
  }

  /// Validate a tree file and return the total node count.
  std::size_t ValidateTree(const std::string& filename) {
    auto path = trees_dir_ / filename;
    EXPECT_TRUE(std::filesystem::exists(path)) << "Tree not found: " << path;
    if (!std::filesystem::exists(path))
      return 0;

    auto tree = factory_.createTreeFromFile(path.string());
    std::size_t total = 0;
    for (const auto& subtree : tree.subtrees) {
      total += subtree->nodes.size();
    }
    return total;
  }

  BT::BehaviorTreeFactory factory_;
  std::filesystem::path trees_dir_;
  std::filesystem::path config_dir_;
};

TEST_F(TreeValidationTest, CommonMotions) {
  // common_motions.xml is a subtree library (no main_tree_to_execute).
  // Validate it parses without errors via registerBehaviorTreeFromFile.
  auto path = trees_dir_ / "common_motions.xml";
  ASSERT_TRUE(std::filesystem::exists(path)) << "Tree not found: " << path;
  EXPECT_NO_THROW(factory_.registerBehaviorTreeFromFile(path.string()));
}

TEST_F(TreeValidationTest, HandMotions) {
  auto count = ValidateTree("hand_motions.xml");
  EXPECT_GT(count, 0u) << "hand_motions.xml should have nodes";
}

TEST_F(TreeValidationTest, PickAndPlaceContactStop) {
  auto count = ValidateTree("pick_and_place_contact_stop.xml");
  EXPECT_GT(count, 0u) << "pick_and_place_contact_stop.xml should have nodes";
}

TEST_F(TreeValidationTest, PickAndPlaceForcePI) {
  auto count = ValidateTree("pick_and_place_force_pi.xml");
  EXPECT_GT(count, 0u) << "pick_and_place_force_pi.xml should have nodes";
}

TEST_F(TreeValidationTest, ShapeInspect) {
  auto count = ValidateTree("shape_inspect.xml");
  EXPECT_GT(count, 0u) << "shape_inspect.xml should have nodes";
}

TEST_F(TreeValidationTest, TowelUnfold) {
  auto count = ValidateTree("towel_unfold.xml");
  EXPECT_GT(count, 0u) << "towel_unfold.xml should have nodes";
}

TEST_F(TreeValidationTest, VisionApproach) {
  auto count = ValidateTree("vision_approach.xml");
  EXPECT_GT(count, 0u) << "vision_approach.xml should have nodes";
}

TEST_F(TreeValidationTest, SearchMotion) {
  auto count = ValidateTree("search_motion.xml");
  EXPECT_GT(count, 0u) << "search_motion.xml should have nodes";
}

TEST_F(TreeValidationTest, ShapeInspectSimple) {
  auto count = ValidateTree("shape_inspect_simple.xml");
  EXPECT_GT(count, 0u) << "shape_inspect_simple.xml should have nodes";
}

TEST_F(TreeValidationTest, InvalidXmlThrows) {
  const std::string bad_xml =
      R"(<root BTCPP_format="4"><BehaviorTree ID="Bad">
           <UnknownNodeType foo="bar"/>
         </BehaviorTree></root>)";

  EXPECT_THROW(factory_.createTreeFromText(bad_xml), std::exception);
}

// Pose-name contract (Open Q3 / Sprint item 3): poses_p1b.yaml overlays
// poses.yaml and overrides hand_pose.* with the proto_1b layout. It must
// override *every* default hand pose — any it misses would silently keep the
// assm-layout values from poses.yaml (wrong joint order for p1b). It must also
// NOT redefine arm_pose.* (the UR5e arm poses are single-sourced in poses.yaml)
// — an arm copy here would drift undetectably. Together these keep every pose a
// tree references resolvable under both variants without editing the trees.
TEST_F(TreeValidationTest, PoseNameParityDefaultVsP1b) {
  const auto default_hand = ReadPoseKeys(config_dir_ / "poses.yaml", "hand_pose.");
  const auto p1b_hand = ReadPoseKeys(config_dir_ / "poses_p1b.yaml", "hand_pose.");
  const auto p1b_arm = ReadPoseKeys(config_dir_ / "poses_p1b.yaml", "arm_pose.");

  ASSERT_FALSE(default_hand.empty()) << "no hand_pose keys parsed from poses.yaml";
  EXPECT_EQ(default_hand, p1b_hand)
      << "poses_p1b.yaml must override exactly the hand_pose.* names poses.yaml "
         "defines (values differ, key set may not — else an assm-layout pose leaks)";
  EXPECT_TRUE(p1b_arm.empty())
      << "poses_p1b.yaml must not redefine arm_pose.* — arm poses are shared via poses.yaml";
}
