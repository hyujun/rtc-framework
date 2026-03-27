// file: include/ur5e_bt_coordinator/action_nodes/move_finger.hpp
#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/hand_pose_config.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <string>

namespace rtc_bt {

/// 특정 손가락을 명명된 포즈로 이동시키는 BT Action Node.
///
/// 하위 Hand 컨트롤러가 quintic polynomial trajectory를 내부 생성하므로,
/// 이 노드는 목표 관절각 + duration만 전달하고 시간 기반으로 완료를 판정한다.
///
/// Input ports:
///   - finger_name (string): 손가락 이름 ("thumb" | "index" | "middle" | "ring")
///   - pose (string): 명명된 포즈 (hand_pose_config에서 lookup)
///   - duration (double): trajectory 실행 시간 [s]
class MoveFinger : public BT::StatefulActionNode {
public:
  MoveFinger(const std::string& name, const BT::NodeConfig& config,
             std::shared_ptr<BtRosBridge> bridge);

  /// @brief BT 포트 정의
  static BT::PortsList providedPorts();

  /// @brief 포즈 lookup 후 Hand 컨트롤러에 목표 전달
  BT::NodeStatus onStart() override;

  /// @brief 경과 시간 확인, duration 도달 시 SUCCESS
  BT::NodeStatus onRunning() override;

  /// @brief halt 로깅
  void onHalted() override;

private:
  std::shared_ptr<BtRosBridge> bridge_;
  double duration_{1.0};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace rtc_bt
