/// Regression coverage for the #158 rewire gap: the tick timer starts in
/// on_activate whether or not the latched /rtc_cm/active_controller_name has
/// arrived, so tick 1 could run before RewireControllerTopics had created the
/// controller-owned target publishers. UR5eHoldPose — the first child of
/// FullDemo's Parallel in hand_motions.xml — publishes an arm target straight
/// from onStart, which dereferenced a null publisher and SIGSEGV'd.
///
/// These drive the real BtCoordinatorNode lifecycle (configure → activate →
/// TickCallback / StepCallback), unlike test_lifecycle_config which stays at
/// the mechanism level: the gate lives on the tick path, so the tick path
/// itself is the unit under test. The tree is a single-leaf UR5eHoldPose
/// written to a temp file (LoadTree takes absolute paths), not one of the large
/// composite trees, so configure stays cheap.
///
/// DDS-free: the active-controller transition is injected through the bridge's
/// own handler via CoordinatorTickInjector + BridgeStateInjector, and the
/// arm-target observation rides intra-process comms within one context.
///
/// The halves matter for different reasons (exec-plan D2):
///   - dropping the gate brings back the SIGSEGV
///     (TicksBeforeRewireDoNotCrashOrAdvanceTree)
///   - a gate that re-checks per tick, or a bare null guard with no gate, would
///     swallow UR5eHoldPose's one-shot onStart publish and turn the crash into
///     a silent hang with the arm never moving (TickPublishesArmTargetOnceWired)
///   - ~/step is a second tick path; a gate on the timer alone leaves the same
///     gap reachable by service call (StepGateTest)

#include "inject_fixture.hpp"
#include "ur5e_bt_coordinator/bt_coordinator_node.hpp"
#include <rtc_msgs/msg/robot_target.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>

#include <gtest/gtest.h>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

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
  /// Auto-tick fixture by default. StepGateTest flips this so on_activate
  /// cancels the tick timer and ~/step becomes the only path to the tree.
  [[nodiscard]] virtual bool StepMode() const { return false; }

  /// Long enough that the default fixtures never trip the controller-wait
  /// escalation; ControllerWaitTimeoutTest shortens it to exercise it.
  [[nodiscard]] virtual double ControllerWaitTimeoutS() const { return 10.0; }

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
        rclcpp::Parameter("step_mode", StepMode()),
        rclcpp::Parameter("controller_wait_timeout_s", ControllerWaitTimeoutS()),
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

    // ~/step client. Same executor as the server, so CallStep must pump it
    // instead of blocking on the future.
    step_client_node_ = std::make_shared<rclcpp::Node>("step_client");
    step_client_ = step_client_node_->create_client<std_srvs::srv::Trigger>("/bt_coordinator/step");
    exec_.add_node(step_client_node_);
  }

  void TearDown() override {
    exec_.remove_node(step_client_node_->get_node_base_interface());
    exec_.remove_node(node_->get_node_base_interface());
    step_client_.reset();
    step_client_node_.reset();
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

  /// Call ~/step and return the response, or nullptr if it never resolves.
  /// The service and the client share exec_, so pump it while polling — a
  /// blocking wait here would deadlock exactly the way SwitchController's
  /// docs warn about.
  std_srvs::srv::Trigger::Response::SharedPtr CallStep(std::chrono::milliseconds timeout = 2000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!step_client_->service_is_ready()) {
      if (std::chrono::steady_clock::now() >= deadline)
        return nullptr;
      exec_.spin_some();
      std::this_thread::sleep_for(1ms);
    }
    auto future = step_client_->async_send_request(std::make_shared<Trigger::Request>()).share();
    while (std::chrono::steady_clock::now() < deadline) {
      exec_.spin_some();
      if (future.wait_for(0s) == std::future_status::ready)
        return future.get();
      std::this_thread::sleep_for(1ms);
    }
    return nullptr;
  }

  void InjectActiveController(const std::string& name) {
    BridgeStateInjector(injector_->Bridge()).ActiveController(name);
  }

  using Trigger = std_srvs::srv::Trigger;

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<BtCoordinatorNode> node_;
  std::unique_ptr<CoordinatorTickInjector> injector_;
  rclcpp::Subscription<rtc_msgs::msg::RobotTarget>::SharedPtr arm_target_sub_;
  rclcpp::Node::SharedPtr step_client_node_;
  rclcpp::Client<Trigger>::SharedPtr step_client_;
  std::filesystem::path tree_path_;
  int arm_targets_seen_ = 0;
};

/// ~/step is the tick path's twin. step_mode cancels the tick timer in
/// on_activate, so anything the tree does here came through the service.
class StepGateTest : public RewireGateTest {
 protected:
  [[nodiscard]] bool StepMode() const override { return true; }
};

/// Threshold well below the spins below, so the escalation has certainly run.
class ControllerWaitTimeoutTest : public RewireGateTest {
 protected:
  [[nodiscard]] double ControllerWaitTimeoutS() const override { return 0.05; }
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

// ── The gate covers the ~/step path too, not just the timer ────────────────

TEST_F(StepGateTest, StepRefusesToTickBeforeRewire) {
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_FALSE(injector_->Bridge()->IsControllerWired());

  auto response = CallStep();
  ASSERT_NE(response, nullptr) << "~/step never answered";

  // Pre-fix this stepped the tree with a null target publisher: a SIGSEGV
  // before PublishArmTarget's null guard, and a silently dropped one-shot
  // publish after it — the arm never moves and nothing reports why.
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "No active controller wired yet");
  EXPECT_EQ(injector_->RootStatus(), BT::NodeStatus::IDLE);
  EXPECT_EQ(arm_targets_seen_, 0);
}

TEST_F(StepGateTest, StepTicksOnceWired) {
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  InjectActiveController(kController);
  ASSERT_TRUE(injector_->Bridge()->IsControllerWired());

  auto response = CallStep();
  ASSERT_NE(response, nullptr) << "~/step never answered";

  // The timer is cancelled in step_mode, so this tick — and the arm target it
  // published — can only have come from the service.
  EXPECT_TRUE(response->success);
  EXPECT_EQ(injector_->RootStatus(), BT::NodeStatus::RUNNING);
  EXPECT_GT(arm_targets_seen_, 0);
}

// ── The target pubs honour the lifecycle, not just the gate ────────────────

TEST_F(RewireGateTest, DeactivatedNodePublishesNoArmTarget) {
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  InjectActiveController(kController);
  SpinTicks();
  ASSERT_GT(arm_targets_seen_, 0) << "baseline: an ACTIVE node must publish";

  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  const int before = arm_targets_seen_;

  // on_deactivate cancels the tick timer, so drive the bridge directly — the
  // question is whether the publisher itself refuses, not whether anything
  // still calls it. An INACTIVE LifecycleNode must put nothing on the wire.
  //
  // The pubs are created by the rewire, which runs after on_activate, so they
  // miss the managed-entity sweep that on_activate performs. They must be
  // activated explicitly at creation; without that they are LifecyclePublishers
  // that were never enabled, and only reach the wire because the declared type
  // is the rclcpp::Publisher base whose publish() is non-virtual. That slicing
  // is what this test refuses to let the code depend on.
  injector_->Bridge()->PublishArmTarget(Pose6D{0.4, 0.0, 0.3, 0.0, 0.0, 0.0});
  SpinTicks(50ms);

  EXPECT_EQ(arm_targets_seen_, before);
}

// ── The wait escalates but never gives up ──────────────────────────────────

TEST_F(ControllerWaitTimeoutTest, WaitTimeoutDiagnosesButKeepsWaiting) {
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  SpinTicks();  // 100ms ≫ the 50ms threshold → the ERROR has fired by now
  EXPECT_EQ(injector_->RootStatus(), BT::NodeStatus::IDLE);

  // The escalation is diagnostic only. Abandoning the wait (failing the tree,
  // transitioning to error) would mean a CM that is merely slow to come up
  // could never recover on its own — so a late controller must still open the
  // gate and let the tree run.
  InjectActiveController(kController);
  SpinTicks();

  EXPECT_EQ(injector_->RootStatus(), BT::NodeStatus::RUNNING);
  EXPECT_GT(arm_targets_seen_, 0);
}

}  // namespace
}  // namespace rtc_bt::test
