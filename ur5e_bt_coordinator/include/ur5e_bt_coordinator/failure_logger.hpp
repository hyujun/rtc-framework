#pragma once

/// Real-time BT status-change logger that forwards FAILURE transitions to
/// rclcpp logging. Inherits from BT::StatusChangeLogger which auto-subscribes
/// to every node in the tree on construction.
///
/// Logs one line per FAILURE transition with:
///   - Node display name (from XML `name=` attribute or auto-generated)
///   - Registered type (e.g. "MoveToPose", "IsGraspPhase")
///   - Node UID (stable across ticks, useful for matching Groot2 traces)
///   - Previous status (IDLE/RUNNING/SUCCESS)
///
/// This complements per-node RCLCPP_ERROR/WARN messages: those explain *why*
/// a specific node failed, while this logger guarantees every FAILURE in the
/// tree is surfaced even if the node itself is silent.

#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <behaviortree_cpp/tree_node.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <string>

namespace rtc_bt {

class FailureLogger : public BT::StatusChangeLogger {
public:
  FailureLogger(BT::TreeNode* root_node, rclcpp::Logger logger)
    : BT::StatusChangeLogger(root_node), logger_(logger) {}

  void callback(BT::Duration /*timestamp*/,
                const BT::TreeNode& node,
                BT::NodeStatus prev_status,
                BT::NodeStatus status) override
  {
    if (status != BT::NodeStatus::FAILURE) return;
    if (prev_status == BT::NodeStatus::FAILURE) return;  // avoid duplicates

    const std::string display_name =
        node.name().empty() ? "<anon>" : node.name();
    RCLCPP_ERROR(
      logger_,
      "[BT FAIL] %s (type=%s uid=%u) %s -> FAILURE",
      display_name.c_str(),
      node.registrationName().c_str(),
      static_cast<unsigned>(node.UID()),
      BT::toStr(prev_status).c_str());
  }

  void flush() override {}

private:
  rclcpp::Logger logger_;
};

}  // namespace rtc_bt
