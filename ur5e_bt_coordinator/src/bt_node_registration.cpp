#include "ur5e_bt_coordinator/bt_node_registration.hpp"

// Action nodes
#include "ur5e_bt_coordinator/action_nodes/compute_offset_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/compute_sweep_trajectory.hpp"
#include "ur5e_bt_coordinator/action_nodes/compute_tilt_sequence.hpp"
#include "ur5e_bt_coordinator/action_nodes/flex_extend_finger.hpp"
#include "ur5e_bt_coordinator/action_nodes/get_current_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/grasp_control.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_finger.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_opposition.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_to_joints.hpp"
#include "ur5e_bt_coordinator/action_nodes/move_to_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/process_search_data.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_gains.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_hand_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/set_pose_z.hpp"
#include "ur5e_bt_coordinator/action_nodes/start_tof_collection.hpp"
#include "ur5e_bt_coordinator/action_nodes/stop_tof_collection.hpp"
#include "ur5e_bt_coordinator/action_nodes/switch_controller.hpp"
#include "ur5e_bt_coordinator/action_nodes/track_trajectory.hpp"
#include "ur5e_bt_coordinator/action_nodes/trigger_shape_estimation.hpp"
#include "ur5e_bt_coordinator/action_nodes/ur5e_hold_pose.hpp"
#include "ur5e_bt_coordinator/action_nodes/wait_duration.hpp"
#include "ur5e_bt_coordinator/action_nodes/wait_shape_result.hpp"

// Condition nodes
#include "ur5e_bt_coordinator/condition_nodes/check_shape_type.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_force_above.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_grasp_phase.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_grasped.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_object_detected.hpp"
#include "ur5e_bt_coordinator/condition_nodes/is_vision_target_ready.hpp"

namespace rtc_bt {

void RegisterBtNodes(BT::BehaviorTreeFactory& factory, const std::shared_ptr<BtRosBridge>& bridge,
                     const RobotCapabilities& capabilities) {
  // ── Always-available nodes (motion / compute / vision) ────────────────
  factory.registerNodeType<MoveToPose>("MoveToPose", bridge);
  factory.registerNodeType<MoveToJoints>("MoveToJoints", bridge);
  factory.registerNodeType<TrackTrajectory>("TrackTrajectory", bridge);
  factory.registerNodeType<SetGains>("SetGains", bridge);
  factory.registerNodeType<SwitchController>("SwitchController", bridge);
  factory.registerNodeType<ComputeOffsetPose>("ComputeOffsetPose");
  factory.registerNodeType<SetPoseZ>("SetPoseZ");
  factory.registerNodeType<ComputeSweepTrajectory>("ComputeSweepTrajectory");
  factory.registerNodeType<ComputeTiltSequence>("ComputeTiltSequence");
  factory.registerNodeType<GetCurrentPose>("GetCurrentPose", bridge);
  factory.registerNodeType<WaitDuration>("WaitDuration");
  factory.registerNodeType<IsObjectDetected>("IsObjectDetected", bridge);
  factory.registerNodeType<IsVisionTargetReady>("IsVisionTargetReady", bridge);

  // ── Hand demo nodes (pose-driven — no grasp sensing required) ─────────
  factory.registerNodeType<MoveFinger>("MoveFinger", bridge);
  factory.registerNodeType<FlexExtendFinger>("FlexExtendFinger", bridge);
  factory.registerNodeType<SetHandPose>("SetHandPose", bridge);
  factory.registerNodeType<UR5eHoldPose>("UR5eHoldPose", bridge);
  factory.registerNodeType<MoveOpposition>("MoveOpposition", bridge);

  // ── Capability: grasp-force sensing ───────────────────────────────────
  if (capabilities.has_grasp_sensing) {
    factory.registerNodeType<GraspControl>("GraspControl", bridge);
    factory.registerNodeType<IsForceAbove>("IsForceAbove", bridge);
    factory.registerNodeType<IsGraspPhase>("IsGraspPhase", bridge);
    factory.registerNodeType<IsGrasped>("IsGrasped", bridge);
  }

  // ── Capability: ToF proximity sensing ─────────────────────────────────
  if (capabilities.has_tof) {
    factory.registerNodeType<StartToFCollection>("StartToFCollection", bridge);
    factory.registerNodeType<StopToFCollection>("StopToFCollection", bridge);
    factory.registerNodeType<ProcessSearchData>("ProcessSearchData", bridge);
  }

  // ── Capability: shape estimation ──────────────────────────────────────
  if (capabilities.has_shape) {
    factory.registerNodeType<TriggerShapeEstimation>("TriggerShapeEstimation", bridge);
    factory.registerNodeType<WaitShapeResult>("WaitShapeResult", bridge);
    factory.registerNodeType<CheckShapeType>("CheckShapeType", bridge);
  }
}

}  // namespace rtc_bt
