/// Offline BT tree validation tool.
///
/// Validates a BehaviorTree XML file against the registered node types
/// without launching a ROS2 node or connecting to any topics.
///
/// Usage:
///   ros2 run ur5e_bt_coordinator validate_tree <tree_file.xml>
///   ros2 run ur5e_bt_coordinator validate_tree trees/pick_and_place_contact_stop.xml
///   ros2 run ur5e_bt_coordinator validate_tree trees/pick_and_place_force_pi.xml
///
/// Exit codes:
///   0 = valid, 1 = invalid, 2 = file not found

#include "ur5e_bt_coordinator/bt_node_registration.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <filesystem>
#include <iostream>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: validate_tree <tree_file.xml>\n"
              << "\n"
              << "Validates a BehaviorTree XML file against registered node types.\n"
              << "The file can be:\n"
              << "  - An absolute path\n"
              << "  - A relative path (resolved from package share/trees/)\n"
              << "\n"
              << "Exit codes: 0=valid, 1=invalid, 2=file not found\n";
    return 1;
  }

  std::string input = argv[1];
  std::filesystem::path tree_path;

  if (std::filesystem::path(input).is_absolute()) {
    tree_path = input;
  } else {
    try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("ur5e_bt_coordinator");
      tree_path = std::filesystem::path(pkg_share) / "trees" / input;
    } catch (...) {
      // If package not found, try relative to CWD
      tree_path = input;
    }
  }

  if (!std::filesystem::exists(tree_path)) {
    std::cerr << "[ERROR] File not found: " << tree_path << "\n";
    return 2;
  }

  std::cout << "[validate_tree] Checking: " << tree_path << "\n";

  // Register every node type with a null bridge (validation only, no ROS comms),
  // reusing the single production registration path so this tool can never drift
  // out of sync with the node set. All capabilities on = validate against the
  // full superset (incl. ToF/shape nodes), so a tree using any registered node
  // resolves regardless of a robot's runtime capability gating.
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<rtc_bt::BtRosBridge> null_bridge;  // nullptr
  rtc_bt::RegisterBtNodes(factory, null_bridge, rtc_bt::RobotCapabilities{true, true, true});

  try {
    auto tree = factory.createTreeFromFile(tree_path.string());

    // Count nodes
    std::size_t total_nodes = 0;
    for (const auto& subtree : tree.subtrees) {
      total_nodes += subtree->nodes.size();
    }

    std::cout << "[validate_tree] VALID ✓\n"
              << "  Subtrees: " << tree.subtrees.size() << "\n"
              << "  Total nodes: " << total_nodes << "\n";

    // List registered node types for reference
    std::cout << "\n  Registered node types:\n";
    auto manifests = factory.manifests();
    for (const auto& [name, manifest] : manifests) {
      std::cout << "    - " << name;
      if (!manifest.ports.empty()) {
        std::cout << " (";
        bool first = true;
        for (const auto& [port_name, port_info] : manifest.ports) {
          if (!first)
            std::cout << ", ";
          std::cout << port_name;
          first = false;
        }
        std::cout << ")";
      }
      std::cout << "\n";
    }

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[validate_tree] INVALID ✗\n"
              << "  Error: " << e.what() << "\n";
    return 1;
  }
}
