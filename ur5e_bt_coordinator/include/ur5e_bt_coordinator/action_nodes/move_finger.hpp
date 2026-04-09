// file: include/ur5e_bt_coordinator/action_nodes/move_finger.hpp
#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/hand_pose_config.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace rtc_bt {

/// 특정 손가락을 명명된 포즈로 이동시키는 BT Action Node.
///
/// RT 컨트롤러와 동일한 quintic trajectory duration 공식으로 소요 시간을
/// 추정하여 완료를 판정한다.
///
/// Input ports:
///   - finger_name (string): 손가락 이름 ("thumb" | "index" | "middle" | "ring")
///   - pose (string): 명명된 포즈 (hand_pose_config에서 lookup)
///   - hand_trajectory_speed (double): trajectory speed [rad/s] (기본 1.0)
///   - current_gains (vector<double>): SwitchController에서 로드한 gains
///     (hand_max_traj_velocity를 추출하여 duration 추정에 사용)
class MoveFinger : public BT::StatefulActionNode {
public:
  MoveFinger(const std::string& name, const BT::NodeConfig& config,
             std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
  double duration_{0.01};
  HandPose target_pose_{};
  std::vector<int> joint_indices_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace rtc_bt
