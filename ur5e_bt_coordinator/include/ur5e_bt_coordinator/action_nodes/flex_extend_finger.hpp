// file: include/ur5e_bt_coordinator/action_nodes/flex_extend_finger.hpp
#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/hand_pose_config.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <string>

namespace rtc_bt {

/// 특정 손가락의 flex → extend 1회 cycle을 수행하는 BT Action Node.
///
/// duration/2 동안 flex 포즈로 이동, 이후 duration/2 동안 home(extend)으로 복귀.
/// 하위 컨트롤러가 quintic trajectory를 수행하므로 BT 노드에서는 보간하지 않는다.
///
/// Input ports:
///   - finger_name (string): 손가락 이름
///   - duration (double): flex+extend 전체 시간 [s] (min: kMinDuration)
class FlexExtendFinger : public BT::StatefulActionNode {
public:
  FlexExtendFinger(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<BtRosBridge> bridge);

  /// @brief BT 포트 정의
  static BT::PortsList providedPorts();

  /// @brief flex 타겟 전송 (Phase 1 시작)
  BT::NodeStatus onStart() override;

  /// @brief 시간 기반 phase 전환: FLEX → EXTEND → SUCCESS
  BT::NodeStatus onRunning() override;

  /// @brief halt 로깅
  void onHalted() override;

private:
  enum class Phase { kFlex, kExtend };

  std::shared_ptr<BtRosBridge> bridge_;
  std::string finger_name_;
  double duration_{1.0};
  Phase phase_{Phase::kFlex};
  bool extend_sent_{false};
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace rtc_bt
