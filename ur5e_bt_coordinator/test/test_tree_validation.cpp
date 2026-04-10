/// Unit tests for XML tree validation (Tier 3).
///
/// Validates that all shipped BT XML trees parse correctly against
/// the registered node types. Acts as a regression guard.

#include "ur5e_bt_coordinator/action_nodes/compute_offset_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/compute_sweep_trajectory.hpp"
#include "ur5e_bt_coordinator/action_nodes/compute_tilt_sequence.hpp"
#include "ur5e_bt_coordinator/action_nodes/get_current_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/flex_extend_finger.hpp"
#include "ur5e_bt_coordinator/action_nodes/grasp_control.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_finger.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_opposition.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_to_joints.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_to_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_gains.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_hand_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_pose_z.hpp"
#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"
#include "ur5e_bt_coordinator/action_nodes/track_trajectory.hpp"
#include "ur5e_bt_coordinator/action_nodes/trigger_shape_estimation.hpp"
#include "ur5e_bt_coordinator/action_nodes/ur5e_hold_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/wait_duration.hpp"
#include "ur5e_bt_coordinator/action_nodes/wait_shape_result.hpp"
#include "ur5e_bt_coordinator/condition_nodes/check_shape_type.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_force_above.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_grasp_phase.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_grasped.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_object_detected.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_vision_target_ready.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <filesystem>

using namespace rtc_bt;

class TreeValidationTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    std::shared_ptr<BtRosBridge> null_bridge;

    // Action nodes
    factory_.registerNodeType<MoveToPose>("MoveToPose", null_bridge);
    factory_.registerNodeType<MoveToJoints>("MoveToJoints", null_bridge);
    factory_.registerNodeType<GraspControl>("GraspControl", null_bridge);
    factory_.registerNodeType<TrackTrajectory>("TrackTrajectory", null_bridge);
    factory_.registerNodeType<SetGains>("SetGains", null_bridge);
    factory_.registerNodeType<SwitchController>("SwitchController", null_bridge);
    factory_.registerNodeType<ComputeOffsetPose>("ComputeOffsetPose");
    factory_.registerNodeType<SetPoseZ>("SetPoseZ");
    factory_.registerNodeType<ComputeSweepTrajectory>("ComputeSweepTrajectory");
    factory_.registerNodeType<ComputeTiltSequence>("ComputeTiltSequence");
    factory_.registerNodeType<GetCurrentPose>("GetCurrentPose", null_bridge);
    factory_.registerNodeType<WaitDuration>("WaitDuration");
    factory_.registerNodeType<MoveFinger>("MoveFinger", null_bridge);
    factory_.registerNodeType<FlexExtendFinger>("FlexExtendFinger", null_bridge);
    factory_.registerNodeType<SetHandPose>("SetHandPose", null_bridge);
    factory_.registerNodeType<UR5eHoldPose>("UR5eHoldPose", null_bridge);
    factory_.registerNodeType<MoveOpposition>("MoveOpposition", null_bridge);
    factory_.registerNodeType<TriggerShapeEstimation>("TriggerShapeEstimation", null_bridge);
    factory_.registerNodeType<WaitShapeResult>("WaitShapeResult", null_bridge);

    // Condition nodes
    factory_.registerNodeType<IsForceAbove>("IsForceAbove", null_bridge);
    factory_.registerNodeType<IsGrasped>("IsGrasped", null_bridge);
    factory_.registerNodeType<IsGraspPhase>("IsGraspPhase", null_bridge);
    factory_.registerNodeType<IsObjectDetected>("IsObjectDetected", null_bridge);
    factory_.registerNodeType<IsVisionTargetReady>("IsVisionTargetReady", null_bridge);
    factory_.registerNodeType<CheckShapeType>("CheckShapeType", null_bridge);

    // Resolve trees directory
    try {
      std::string pkg_share =
          ament_index_cpp::get_package_share_directory("ur5e_bt_coordinator");
      trees_dir_ = std::filesystem::path(pkg_share) / "trees";
    } catch (...) {
      // Fallback to source tree
      trees_dir_ = std::filesystem::path(__FILE__).parent_path().parent_path() / "trees";
    }
  }

  /// Validate a tree file and return the total node count.
  std::size_t ValidateTree(const std::string& filename)
  {
    auto path = trees_dir_ / filename;
    EXPECT_TRUE(std::filesystem::exists(path)) << "Tree not found: " << path;
    if (!std::filesystem::exists(path)) return 0;

    auto tree = factory_.createTreeFromFile(path.string());
    std::size_t total = 0;
    for (const auto& subtree : tree.subtrees) {
      total += subtree->nodes.size();
    }
    return total;
  }

  BT::BehaviorTreeFactory factory_;
  std::filesystem::path trees_dir_;
};

TEST_F(TreeValidationTest, CommonMotions)
{
  // common_motions.xml is a subtree library (no main_tree_to_execute).
  // Validate it parses without errors via registerBehaviorTreeFromFile.
  auto path = trees_dir_ / "common_motions.xml";
  ASSERT_TRUE(std::filesystem::exists(path)) << "Tree not found: " << path;
  EXPECT_NO_THROW(factory_.registerBehaviorTreeFromFile(path.string()));
}

TEST_F(TreeValidationTest, HandMotions)
{
  auto count = ValidateTree("hand_motions.xml");
  EXPECT_GT(count, 0u) << "hand_motions.xml should have nodes";
}

TEST_F(TreeValidationTest, PickAndPlace)
{
  auto count = ValidateTree("pick_and_place.xml");
  EXPECT_GT(count, 0u) << "pick_and_place.xml should have nodes";
}

TEST_F(TreeValidationTest, ShapeInspect)
{
  auto count = ValidateTree("shape_inspect.xml");
  EXPECT_GT(count, 0u) << "shape_inspect.xml should have nodes";
}

TEST_F(TreeValidationTest, TowelUnfold)
{
  auto count = ValidateTree("towel_unfold.xml");
  EXPECT_GT(count, 0u) << "towel_unfold.xml should have nodes";
}

TEST_F(TreeValidationTest, VisionApproach)
{
  auto count = ValidateTree("vision_approach.xml");
  EXPECT_GT(count, 0u) << "vision_approach.xml should have nodes";
}

TEST_F(TreeValidationTest, SearchMotion)
{
  auto count = ValidateTree("search_motion.xml");
  EXPECT_GT(count, 0u) << "search_motion.xml should have nodes";
}

TEST_F(TreeValidationTest, InvalidXmlThrows)
{
  const std::string bad_xml =
      R"(<root BTCPP_format="4"><BehaviorTree ID="Bad">
           <UnknownNodeType foo="bar"/>
         </BehaviorTree></root>)";

  EXPECT_THROW(factory_.createTreeFromText(bad_xml), std::exception);
}
