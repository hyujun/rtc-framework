// file: include/ur5e_bt_coordinator/action_nodes/move_opposition.hpp
#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/hand_pose_config.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace rtc_bt {

/// 엄지 + 대상 손가락 opposition 동작을 수행하는 BT Action Node.
///
/// thumb 관절과 target 손가락 관절을 목표 포즈로 이동하고,
/// 나머지 손가락은 home으로 리셋하여 잔류 문제를 방지한다.
/// RT 컨트롤러와 동일한 quintic trajectory duration 공식으로 완료를 판정한다.
///
/// Input ports:
///   - thumb_pose (string): 엄지 포즈 이름 (예: "thumb_index_oppose")
///   - target_finger (string): 대상 손가락 이름 ("index" | "middle" | "ring")
///   - target_pose (string): 대상 손가락 포즈 이름 (예: "index_oppose")
///   - hand_trajectory_speed (double): trajectory speed [rad/s] (기본 1.0)
///
/// Duration 추정의 max_vel은 bt_utils.hpp::kDefaultHandMaxTrajVelocity 사용.
class MoveOpposition : public BT::StatefulActionNode {
public:
  MoveOpposition(const std::string &name, const BT::NodeConfig &config,
                 std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
  double duration_{0.01};
  std::chrono::steady_clock::time_point start_time_;
};

} // namespace rtc_bt
