#pragma once
/// The one-snippet tree every node test builds: `xml` (a node, or a small
/// Sequence of them) as the whole body of a BehaviorTree. ROS-free, so the
/// Tier 1 tests can include it without test_helpers.hpp.

#include <behaviortree_cpp/bt_factory.h>

#include <string>

namespace rtc_bt::test {

inline BT::Tree CreateSnippetTree(BT::BehaviorTreeFactory& factory, const std::string& xml) {
  return factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" + xml +
                                    R"(</BehaviorTree></root>)");
}

}  // namespace rtc_bt::test
