// file: include/ur5e_bt_coordinator/action_nodes/flex_extend_finger.hpp
#pragma once

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/hand_pose_config.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace rtc_bt {

/// 특정 손가락의 flex → extend 1회 cycle을 수행하는 BT Action Node.
///
/// flex 포즈로 이동 후 trajectory 완료 시 home(extend)으로 복귀한다.
/// 각 phase의 소요 시간은 RT 컨트롤러와 동일한 quintic trajectory duration
/// 공식으로 추정한다.
///
/// Input ports:
///   - finger_name (string): 손가락 이름
///   - hand_trajectory_speed (double): trajectory speed [rad/s] (기본 1.0)
///   - hand_max_traj_velocity (double): max trajectory velocity [rad/s] (기본 2.0)
class FlexExtendFinger : public BT::StatefulActionNode {
public:
  FlexExtendFinger(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<BtRosBridge> bridge);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { kFlex, kExtend };

  std::shared_ptr<BtRosBridge> bridge_;
  std::string finger_name_;
  double speed_{1.0};
  double max_vel_{2.0};
  Phase phase_{Phase::kFlex};
  double flex_duration_{0.01};
  double extend_duration_{0.01};
  HandPose flex_target_{};
  HandPose home_target_{};
  std::vector<int> joint_indices_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace rtc_bt
