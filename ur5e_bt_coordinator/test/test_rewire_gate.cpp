/// Regression coverage for the #158 rewire gap: the tick timer starts in
/// on_activate whether or not the latched /rtc_cm/active_controller_name has
/// arrived, so tick 1 could run before RewireControllerTopics had created the
/// controller-owned target publishers. UR5eHoldPose — the first child of
/// FullDemo's Parallel in hand_motions.xml — publishes an arm target straight
/// from onStart, which dereferenced a null publisher and SIGSEGV'd.
///
/// These drive the real BtCoordinatorNode lifecycle (configure → activate →
/// timer-driven TickCallback), unlike test_lifecycle_config which stays at the
/// mechanism level: the gate lives in TickCallback, so the tick path itself is
/// the unit under test. The tree is a single-leaf UR5eHoldPose written to a
/// temp file (LoadTree takes absolute paths), not one of the large composite
/// trees, so configure stays cheap.
///
/// DDS-free: the active-controller transition is injected through the bridge's
/// own handler via CoordinatorTickInjector + BridgeStateInjector, and the
/// arm-target observation rides intra-process comms within one context.
///
/// The two halves matter for different reasons (exec-plan D2): dropping the
/// gate brings back the SIGSEGV, while a gate that re-checks per tick — or a
/// bare null guard with no gate — would swallow UR5eHoldPose's one-shot
/// onStart publish and turn the crash into a silent hang with the arm never
/// moving. TickPublishesArmTargetOnceWired is what catches that.

#include "inject_fixture.hpp"
#include "ur5e_bt_coordinator/bt_coordinator_node.hpp"
#include <rtc_msgs/msg/robot_target.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace rtc_bt::test {

/// Test-only seam (friend of BtCoordinatorNode): exposes the node's own bridge
/// and tree so the tick path can be driven with a controllable rewire state.
struct CoordinatorTickInjector {
  explicit CoordinatorTickInjector(std::shared_ptr<BtCoordinatorNode> node_in)
      : node(std::move(node_in)) {}

  /// The bridge the node created in on_configure — the same instance the tree's
  /// nodes hold, so injecting here is what the real callback would do.
  [[nodiscard]] std::shared_ptr<BtRosBridge> Bridge() const { return node->bridge_; }

  /// Root status is IDLE until the first tick actually reaches the tree, which
  /// is precisely what the gate defers.
  [[nodiscard]] BT::NodeStatus RootStatus() const {
    return node->tree_ ? node->tree_->rootNode()->status() : BT::NodeStatus::IDLE;
  }

  std::shared_ptr<BtCoordinatorNode> node;
};

namespace {

constexpr const char* kController = "demo_task_controller";
constexpr double kTickRateHz = 200.0;  // 5ms/tick — many ticks per Spin below

class RewireGateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tree_path_ = std::filesystem::temp_directory_path() / "rewire_gate_tree.xml";
    std::ofstream(tree_path_) << R"(<root BTCPP_format="4"><BehaviorTree ID="T">)"
                                 R"(<UR5eHoldPose pose="demo_pose"/>)"
                                 R"(</BehaviorTree></root>)";

    rclcpp::NodeOptions opts;
    opts.use_intra_process_comms(true);
    // hand_pose.* is a dynamic param, so it only exists via auto-declare.
    opts.automatically_declare_parameters_from_overrides(true);
    opts.parameter_overrides({
        rclcpp::Parameter("tree_file", tree_path_.string()),
        rclcpp::Parameter("hand_group", "p1b"),
        rclcpp::Parameter("tick_rate_hz", kTickRateHz),
        // The watchdog only logs; keep it out of the way of the tick timer.
        rclcpp::Parameter("watchdog_interval_s", 0.0),
        // A non-default hand_group seeds no compile-time hand poses and
        // configure rejects an empty pose map. The tree only holds an arm
        // pose (demo_pose, a kUR5ePoses default), so one entry is enough.
        rclcpp::Parameter("hand_pose.home", std::vector<double>(kDefaultHandDof, 0.0)),
    });
    node_ = std::make_shared<BtCoordinatorNode>(opts);
    injector_ = std::make_unique<CoordinatorTickInjector>(node_);

    exec_.add_node(node_->get_node_base_interface());
    ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    // Observe the arm target the way a controller would. Created before
    // activate so no publish can be missed; intra-process keeps it DDS-free.
    arm_target_sub_ = node_->create_subscription<rtc_msgs::msg::RobotTarget>(
        std::string("/") + kController + "/ur5e/joint_goal", rclcpp::QoS{10},
        [this](rtc_msgs::msg::RobotTarget::SharedPtr) { ++arm_targets_seen_; });
  }

  void TearDown() override {
    exec_.remove_node(node_->get_node_base_interface());
    injector_.reset();
    node_.reset();
    std::error_code ec;
    std::filesystem::remove(tree_path_, ec);
  }

  /// Let the tick timer fire repeatedly. At 200Hz a 100ms spin is ~20 ticks —
  /// well past the single tick that used to be enough to crash.
  void SpinTicks(std::chrono::milliseconds duration = 100ms) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      exec_.spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  void InjectActiveController(const std::string& name) {
    BridgeStateInjector(injector_->Bridge()).ActiveController(name);
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<BtCoordinatorNode> node_;
  std::unique_ptr<CoordinatorTickInjector> injector_;
  rclcpp::Subscription<rtc_msgs::msg::RobotTarget>::SharedPtr arm_target_sub_;
  std::filesystem::path tree_path_;
  int arm_targets_seen_ = 0;
};

// ── Acceptance 1, first half: no crash, no progress before the rewire ───────

TEST_F(RewireGateTest, TicksBeforeRewireDoNotCrashOrAdvanceTree) {
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_FALSE(injector_->Bridge()->IsControllerWired());

  SpinTicks();  // pre-fix: SIGSEGV on tick 1 via UR5eHoldPose::onStart

  // The tree must not have started: onStart would publish an arm target into
  // an unbound topic, and UR5eHoldPose only ever publishes that once.
  EXPECT_EQ(injector_->RootStatus(), BT::NodeStatus::IDLE);
  EXPECT_EQ(arm_targets_seen_, 0);
}

// ── Acceptance 1, second half: the one-shot publish survives the gate ───────

TEST_F(RewireGateTest, TickPublishesArmTargetOnceWired) {
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  SpinTicks(50ms);
  ASSERT_EQ(arm_targets_seen_, 0);

  InjectActiveController(kController);
  ASSERT_TRUE(injector_->Bridge()->IsControllerWired());

  SpinTicks();

  // onStart ran after the gate opened and its single publish reached the wire —
  // the gate deferred it rather than swallowing it (D2).
  EXPECT_EQ(injector_->RootStatus(), BT::NodeStatus::RUNNING);
  EXPECT_GT(arm_targets_seen_, 0);
}

// ── D1: the latch never re-closes ──────────────────────────────────────────

TEST_F(RewireGateTest, WiredLatchSurvivesControllerSwitch) {
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  InjectActiveController(kController);
  ASSERT_TRUE(injector_->Bridge()->IsControllerWired());

  // A switch rebinds the pubs to the new controller; an empty name is ignored
  // by the rewire outright. Neither may re-close the gate on a running tree.
  InjectActiveController("other_controller");
  EXPECT_TRUE(injector_->Bridge()->IsControllerWired());
  InjectActiveController("");
  EXPECT_TRUE(injector_->Bridge()->IsControllerWired());
}

}  // namespace
}  // namespace rtc_bt::test
