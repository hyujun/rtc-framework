#include "ur5e_bt_coordinator/action_nodes/stop_tof_collection.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("stop_tof_collection"); }
}  // namespace

StopToFCollection::StopToFCollection(
    const std::string& name, const BT::NodeConfig& config,
    std::shared_ptr<BtRosBridge> bridge)
  : BT::SyncActionNode(name, config), bridge_(std::move(bridge))
{}

BT::PortsList StopToFCollection::providedPorts()
{
  return {
    BT::OutputPort<int>("count", "Number of ToF snapshots collected"),
  };
}

BT::NodeStatus StopToFCollection::tick()
{
  bridge_->StopToFCollection();
  auto count = static_cast<int>(bridge_->GetCollectedToFCount());
  setOutput("count", count);
  RCLCPP_INFO(logger(), "ToF collection stopped: %d snapshots", count);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
