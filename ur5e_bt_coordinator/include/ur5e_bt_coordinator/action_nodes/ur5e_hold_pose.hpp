// file: include/ur5e_bt_coordinator/action_nodes/ur5e_hold_pose.hpp
#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/hand_pose_config.hpp"

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <string>

namespace rtc_bt {

/// UR5e 팔을 목표 자세로 이동 후 유지하는 BT Action Node.
///
/// 하위 UR5e 컨트롤러가 quintic trajectory를 내부 생성하여 이동한다.
/// 이 노드는 목표 관절각만 전달하고, Parallel 부모에 의해 halt될 때까지
/// 영구 RUNNING을 반환한다.
///
/// Input ports:
///   - pose (string): 명명된 UR5e 포즈 (예: "demo_pose")
class UR5eHoldPose : public BT::StatefulActionNode {
public:
  UR5eHoldPose(const std::string& name, const BT::NodeConfig& config,
               std::shared_ptr<BtRosBridge> bridge);

  /// @brief BT 포트 정의
  static BT::PortsList providedPorts();

  /// @brief UR5e 목표 관절각을 컨트롤러에 전달
  BT::NodeStatus onStart() override;

  /// @brief 항상 RUNNING 반환 (halt될 때까지 자세 유지)
  BT::NodeStatus onRunning() override;

  /// @brief halt 로깅
  void onHalted() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
};

}  // namespace rtc_bt
