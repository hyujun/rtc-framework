// file: include/ur5e_bt_coordinator/action_nodes/set_hand_pose.hpp
#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/hand_pose_config.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <string>

namespace rtc_bt {

/// Hand 전체 10관절을 명명된 포즈로 이동시키는 BT Action Node.
///
/// MoveFinger와 동일 패턴이지만, 특정 손가락이 아닌 전체 10-DoF를 대상으로 한다.
///
/// Input ports:
///   - pose (string): 명명된 포즈 (예: "home", "full_flex")
///   - duration (double): trajectory 실행 시간 [s]
class SetHandPose : public BT::StatefulActionNode {
public:
  SetHandPose(const std::string& name, const BT::NodeConfig& config,
              std::shared_ptr<BtRosBridge> bridge);

  /// @brief BT 포트 정의
  static BT::PortsList providedPorts();

  /// @brief 10-DoF 목표 관절각을 컨트롤러에 전달
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
