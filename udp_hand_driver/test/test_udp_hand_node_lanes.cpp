// Aux-lane wiring tests for UdpHandNode (issue #345).
//
// The lane split moves the blocking file-I/O timers (timing CSV drain, stats
// JSON save) off the node's default executor thread — which also serves the
// joint-command subscription — onto a dedicated `hand_aux_io` executor thread.
// The split is load-bearing but *silent* when it breaks: if the aux callback
// group is auto-added to the executor that adds the node, everything still
// runs, on one thread again, and no existing test notices. These tests pin the
// properties that make the split real.
//
// main()'s two-executor wiring itself lives in udp_hand_node.cpp and cannot be
// linked here (it owns main()), so the tests exercise the same rclcpp mechanism
// the wiring depends on: a group the node does not surrender to add_node(), and
// a second executor that can therefore claim it.

#include "udp_hand_driver/udp_hand_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace {

class UdpHandNodeLaneTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  std::shared_ptr<UdpHandNode> MakeNode() { return std::make_shared<UdpHandNode>(); }
};

// The group must exist before any lifecycle transition: main() hands it to the
// aux executor at startup, long before configure. Creating it in on_configure
// would also break reconfiguration — the executor would keep holding the old,
// now-orphaned group and the aux timers would silently stop firing.
TEST_F(UdpHandNodeLaneTest, AuxGroupExistsAfterConstruction) {
  auto node = MakeNode();
  ASSERT_NE(node->GetAuxCallbackGroup(), nullptr);
}

// The load-bearing flag. If this is ever flipped to the rclcpp default (true),
// main()'s `main_executor.add_node()` claims the aux group, both lanes collapse
// onto one thread, and the CSV burst is back in front of the command
// subscription — with every other test still green.
TEST_F(UdpHandNodeLaneTest, AuxGroupIsNotAutomaticallyAddedWithNode) {
  auto node = MakeNode();
  EXPECT_FALSE(node->GetAuxCallbackGroup()->automatically_add_to_executor_with_node());
}

// Behavioural counterpart of the flag test: adding the node to an executor must
// leave the aux group unclaimed, so a second executor can take it. rclcpp
// throws if a group is added to two executors, so a no-throw here IS the
// assertion that the split is achievable.
TEST_F(UdpHandNodeLaneTest, AddNodeLeavesAuxGroupForASecondExecutor) {
  auto node = MakeNode();
  rclcpp::executors::SingleThreadedExecutor main_executor;
  rclcpp::executors::SingleThreadedExecutor aux_executor;

  main_executor.add_node(node->get_node_base_interface());
  EXPECT_NO_THROW(aux_executor.add_callback_group(node->GetAuxCallbackGroup(),
                                                  node->get_node_base_interface()));

  aux_executor.remove_callback_group(node->GetAuxCallbackGroup());
  main_executor.remove_node(node->get_node_base_interface());
}

// Order independence: main() claims the aux group *before* add_node, so the
// reverse order must hold too. A future rclcpp that swept unassociated groups
// on add_node regardless of the flag would fail here first.
TEST_F(UdpHandNodeLaneTest, AuxGroupSurvivesBeingClaimedBeforeAddNode) {
  auto node = MakeNode();
  rclcpp::executors::SingleThreadedExecutor main_executor;
  rclcpp::executors::SingleThreadedExecutor aux_executor;

  aux_executor.add_callback_group(node->GetAuxCallbackGroup(), node->get_node_base_interface());
  EXPECT_NO_THROW(main_executor.add_node(node->get_node_base_interface()));

  main_executor.remove_node(node->get_node_base_interface());
  aux_executor.remove_callback_group(node->GetAuxCallbackGroup());
}

// The aux slot default mirrors the shell SSoT get_os_cores() == 0. main() reads
// this before spinning, which is why it is declared in the constructor rather
// than on_configure.
TEST_F(UdpHandNodeLaneTest, AuxCpuSlotDefaultsToTheOsSlot) {
  auto node = MakeNode();
  EXPECT_EQ(node->AuxCpuSlot(), 0);
}

TEST_F(UdpHandNodeLaneTest, AuxCpuSlotHonoursParameterOverride) {
  auto node = MakeNode();
  node->set_parameter(rclcpp::Parameter("aux_cpu_slot", 3));
  EXPECT_EQ(node->AuxCpuSlot(), 3);
}

// A negative slot is the repo-wide "do not pin" sentinel (thread_config.hpp
// cpu_core == -1); ApplyThreadConfig skips the affinity step entirely for it.
// Phase 4 (use_cpu_affinity:=false) relies on this passing through unclamped.
TEST_F(UdpHandNodeLaneTest, AuxCpuSlotPassesThroughTheNoPinSentinel) {
  auto node = MakeNode();
  node->set_parameter(rclcpp::Parameter("aux_cpu_slot", -1));
  EXPECT_EQ(node->AuxCpuSlot(), -1);
}

// The thread config the aux lane is built from: CFS (never preempts the
// CommLoop) and a name that survives pthread_setname_np's 15-char truncation,
// which is how verify_rt_runtime.sh finds the thread in /proc/*/comm.
TEST_F(UdpHandNodeLaneTest, AuxThreadConfigIsNonRealtimeAndNameable) {
  EXPECT_EQ(udp_hand_driver::kHandAuxIoConfig.sched_policy, SCHED_OTHER);
  EXPECT_EQ(udp_hand_driver::kHandAuxIoConfig.sched_priority, 0);
  EXPECT_STREQ(udp_hand_driver::kHandAuxIoConfig.name, "hand_aux_io");
  EXPECT_LE(std::string(udp_hand_driver::kHandAuxIoConfig.name).size(), 15u);
  // Inherit by default; main() overwrites cpu_core from aux_cpu_slot.
  EXPECT_EQ(udp_hand_driver::kHandAuxIoConfig.cpu_core, -1);
}

}  // namespace
