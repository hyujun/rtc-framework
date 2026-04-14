#include "ur5e_bt_coordinator/action_nodes/start_tof_collection.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("start_tof_collection"); }
}  // namespace

StartToFCollection::StartToFCollection(
    const std::string& name, const BT::NodeConfig& config,
    std::shared_ptr<BtRosBridge> bridge)
  : BT::SyncActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList StartToFCollection::providedPorts()
{
  return {};
}

BT::NodeStatus StartToFCollection::tick()
{
  bridge_->StartToFCollection();
  RCLCPP_INFO(logger(), "ToF collection started");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
