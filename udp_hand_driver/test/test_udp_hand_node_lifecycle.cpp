// Lifecycle behaviour tests for UdpHandNode in sil_mode=loopmodel (issue #345).
//
// test_udp_hand_node_lanes pins the lane *wiring* and the pull-model arithmetic
// on an unconfigured node; nothing there runs on_configure/on_activate, so the
// NRT publish lane (publish_timer_ -> PollAndPublish -> PublishState) was pinned
// only as arithmetic, never as behaviour. These tests drive the node through
// configure -> activate with the in-process fake hand (sil_mode=loopmodel — the
// same fake path test_udp_hand_controller exercises at the controller level) and
// assert the lane end to end: the CommLoop stores SeqLock snapshots, the poll
// timer publishes them on the executor thread, deactivate quiesces the topics,
// and a reactivation resumes them.
//
// The suite runs under a dedicated ROS_DOMAIN_ID (CMakeLists — the isolation
// idiom rtc_controller_manager's full-pipeline tests use) because colcon test
// runs packages in parallel and cross-package discovery churn breaks pub/sub
// matching waits. RTC_SESSION_DIR points at a per-run temp dir so the stats
// JSON / timing CSV side effects land nowhere real and stay assertable.

#include "udp_hand_driver/udp_hand_node.hpp"
#include <rtc_msgs/msg/calibration_command.hpp>
#include <rtc_msgs/msg/calibration_status.hpp>
#include <rtc_msgs/msg/hand_sensor_state.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

namespace {

using namespace std::chrono_literals;

rclcpp::NodeOptions LoopmodelOptions() {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("sil_mode", "loopmodel"),
      // 2 kHz instead of the 500 Hz default: comm cycles accumulate 4x faster,
      // so the publish-count carry path (kCycleLogInterval cycles) is reached
      // well inside the spin budget even on a loaded CI runner. The CommLoop
      // falls back to CFS when SCHED_FIFO is unavailable (EPERM in CI), which
      // sustains this rate — the tick body is microseconds of LPF math.
      rclcpp::Parameter("loop_rate_hz", 2000.0),
      // publish_rate == loop rate -> link_decimation_ == 1, so link_status
      // publishes from the first completed cycle instead of every 5th.
      rclcpp::Parameter("publish_rate", 2000.0),
  });
  return options;
}

class UdpHandNodeLifecycleTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    char tmpl[] = "/tmp/udp_hand_node_lifecycle_XXXXXX";
    const char* created = mkdtemp(tmpl);
    ASSERT_NE(created, nullptr);
    session_dir_ = created;
    // The node writes <session>/device/hand_udp_stats.json (SaveCommStats) and
    // <session>/timing/hand_udp_timing_log.csv (on_activate). ResolveSessionDir
    // honours this env var first.
    setenv("RTC_SESSION_DIR", created, 1);
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
    std::error_code ec;
    std::filesystem::remove_all(session_dir_, ec);
  }

  template <typename Pred>
  bool SpinUntil(Pred&& pred, std::chrono::nanoseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!pred()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      exec_.spin_once(20ms);
    }
    return true;
  }

  void SpinFor(std::chrono::nanoseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      exec_.spin_once(20ms);
    }
  }

  static std::filesystem::path session_dir_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};

std::filesystem::path UdpHandNodeLifecycleTest::session_dir_;

// The publish lane, end to end: loopmodel configure/activate brings up the real
// CommLoop thread, the poll timer turns SeqLock progress into topic traffic,
// deactivate freezes the sequence so the lane goes quiet, and cleanup releases
// the controller under the still-live poll timer (its null guard is what keeps
// the unconfigured node safe — ReleaseResources deliberately leaves the default
// lane's timer to be replaced by the next configure).
TEST_F(UdpHandNodeLifecycleTest, PublishLaneRunsEndToEndAcrossTheLifecycle) {
  auto node = std::make_shared<UdpHandNode>(LoopmodelOptions());
  auto probe = std::make_shared<rclcpp::Node>("hand_lifecycle_probe");

  int joint_count = 0;
  int motor_count = 0;
  int sensor_count = 0;
  int link_count = 0;
  bool last_link = false;
  sensor_msgs::msg::JointState last_joint;

  // The three state publishers are best_effort — a reliable subscription would
  // not match them.
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  auto joint_sub = probe->create_subscription<sensor_msgs::msg::JointState>(
      "/hand/joint_states", state_qos, [&](sensor_msgs::msg::JointState::SharedPtr msg) {
        ++joint_count;
        last_joint = *msg;
      });
  auto motor_sub = probe->create_subscription<sensor_msgs::msg::JointState>(
      "/hand/motor_states", state_qos,
      [&](sensor_msgs::msg::JointState::SharedPtr) { ++motor_count; });
  auto sensor_sub = probe->create_subscription<rtc_msgs::msg::HandSensorState>(
      "/hand/sensor_states", state_qos,
      [&](rtc_msgs::msg::HandSensorState::SharedPtr) { ++sensor_count; });
  auto link_sub = probe->create_subscription<std_msgs::msg::Bool>(
      "/hand/link_status", rclcpp::QoS(1).transient_local(),
      [&](std_msgs::msg::Bool::SharedPtr msg) {
        ++link_count;
        last_link = msg->data;
      });

  exec_.add_node(node->get_node_base_interface());
  exec_.add_node(probe);

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(SpinUntil(
      [&] { return joint_count >= 5 && motor_count >= 5 && sensor_count >= 1 && link_count >= 1; },
      10s));

  // Fake mode has no recv failures, so the strict per-channel link health must
  // report up; the joint message carries the firmware-slot name order.
  EXPECT_TRUE(last_link);
  EXPECT_EQ(last_joint.name, udp_hand_driver::kDefaultHandMotorNames);
  EXPECT_EQ(last_joint.position.size(), static_cast<std::size_t>(udp_hand_driver::kNumHandMotors));

  // Keep the lane running long enough for the publish-count carry
  // (kCycleLogInterval == 500 comm cycles; 0.25 s nominal at 2 kHz) to be
  // crossed with a wide margin. No assertion is tied to reaching it — the spin
  // only widens the exercised path.
  SpinFor(1200ms);

  // The timing CSV opened on activation (hand_udp_timing_opened_).
  EXPECT_TRUE(std::filesystem::exists(session_dir_ / "timing" / "hand_udp_timing_log.csv"));

  // Deactivate stops the CommLoop, so the SeqLock sequence freezes and every
  // subsequent poll takes the no-progress early return; LifecyclePublishers are
  // gated on top of that. The topics must go quiet.
  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  SpinFor(100ms);  // let in-flight deliveries settle
  const int joint_after_deactivate = joint_count;
  SpinFor(300ms);
  EXPECT_EQ(joint_count, joint_after_deactivate);

  // Cleanup resets the controller while the poll timer keeps firing until the
  // next configure replaces it — the spin proves the null-controller guard
  // holds (no crash, no publish).
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  SpinFor(100ms);
  EXPECT_EQ(joint_count, joint_after_deactivate);

  // on_cleanup persisted the forensic counters before releasing the controller.
  EXPECT_TRUE(std::filesystem::exists(session_dir_ / "device" / "hand_udp_stats.json"));

  exec_.remove_node(probe);
  exec_.remove_node(node->get_node_base_interface());
}

// Reactivation contract: on_activate recreates the per-activation timers and
// restarts the CommLoop, and the poll lane resumes against the same controller
// (last_published_seq_ is behind after the pause; the poll must absorb the jump
// and keep publishing rather than stall).
TEST_F(UdpHandNodeLifecycleTest, DeactivateActivateCycleResumesPublishing) {
  auto node = std::make_shared<UdpHandNode>(LoopmodelOptions());
  auto probe = std::make_shared<rclcpp::Node>("hand_reactivation_probe");

  int joint_count = 0;
  auto joint_sub = probe->create_subscription<sensor_msgs::msg::JointState>(
      "/hand/joint_states", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      [&](sensor_msgs::msg::JointState::SharedPtr) { ++joint_count; });

  exec_.add_node(node->get_node_base_interface());
  exec_.add_node(probe);

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil([&] { return joint_count >= 3; }, 10s));

  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  SpinFor(100ms);

  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  const int base = joint_count;
  ASSERT_TRUE(SpinUntil([&] { return joint_count >= base + 3; }, 10s));

  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  exec_.remove_node(probe);
  exec_.remove_node(node->get_node_base_interface());
}

// Calibration requests the CommLoop cannot serve must be consumed without
// harming the RT loop. loopmodel leaves ft_inferencer disabled, so a barometer
// START exercises the dispatcher's no-inferencer branch, and a bogus
// sensor_type its default branch — the regression this guards is a dispatch
// that dereferences the missing inferencer (or floods the RT thread with
// unthrottled logs, the RT-3 violation #345 fixed). Afterwards the loop must
// still be publishing and the status topic must keep reporting the barometer
// channel as failed (= not calibratable), never as started.
TEST_F(UdpHandNodeLifecycleTest, CalibrationRequestsWithoutAnInferencerAreConsumedSafely) {
  auto node = std::make_shared<UdpHandNode>(LoopmodelOptions());
  auto probe = std::make_shared<rclcpp::Node>("hand_calibration_probe");

  int joint_count = 0;
  auto joint_sub = probe->create_subscription<sensor_msgs::msg::JointState>(
      "/hand/joint_states", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      [&](sensor_msgs::msg::JointState::SharedPtr) { ++joint_count; });

  int status_count = 0;
  uint8_t last_status_state = 0xFF;
  auto status_sub = probe->create_subscription<rtc_msgs::msg::CalibrationStatus>(
      "/hand/calibration/status", rclcpp::QoS(4).reliable().transient_local(),
      [&](rtc_msgs::msg::CalibrationStatus::SharedPtr msg) {
        ++status_count;
        last_status_state = msg->state;
      });

  auto calib_pub = probe->create_publisher<rtc_msgs::msg::CalibrationCommand>(
      "/hand/calibration/command", rclcpp::QoS(1).reliable());

  exec_.add_node(node->get_node_base_interface());
  exec_.add_node(probe);

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Reliable QoS makes delivery certain once the command channel is matched.
  ASSERT_TRUE(SpinUntil([&] { return calib_pub->get_subscription_count() > 0; }, 10s));

  // Sequential, not back-to-back: the pending-request handoff is last-wins, so
  // the second command would overwrite the first before the CommLoop consumed
  // it. Each spin spans hundreds of 2 kHz cycles — the dispatch has long since
  // run by the time the next command goes out.
  rtc_msgs::msg::CalibrationCommand cmd;
  cmd.sensor_type = udp_hand_driver::calibration::kSensorBarometer;
  cmd.action = udp_hand_driver::calibration::kActionStart;
  cmd.sample_count = 25;
  calib_pub->publish(cmd);
  SpinFor(200ms);

  cmd.sensor_type = 0xEE;  // no such sensor — the dispatcher's default branch
  calib_pub->publish(cmd);
  SpinFor(200ms);

  const int joint_base = joint_count;
  ASSERT_TRUE(SpinUntil([&] { return status_count >= 2 && joint_count >= joint_base + 3; }, 10s));
  EXPECT_EQ(last_status_state, udp_hand_driver::calibration::kStateFailed);

  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  exec_.remove_node(probe);
  exec_.remove_node(node->get_node_base_interface());
}

// The aux lane as *behaviour*, not wiring. test_udp_hand_node_lanes proves
// cb_group_aux_ can be claimed by a second executor; nothing proved that a timer
// created inside that group during on_activate is ever serviced. Both aux timers
// are created after the group has already been handed out, and the group is built
// with automatically_add_to_executor_with_node=false, so the node's own executor
// never touches them — if that hand-off broke, the stats/timing drain would die
// silently in production while every other test stayed green (issue #345).
//
// Discriminator: the timing CSV. While the node is Active the 1 Hz aux timer is
// its only writer (on_deactivate's straggler drain is the other caller, and this
// test asserts before deactivating), and ThreadTimingCsvLogger::Log flushes per
// row — so growth means the timer fired on aux_thread.
TEST_F(UdpHandNodeLifecycleTest, AuxTimerFiresOnItsOwnExecutor) {
  auto node = std::make_shared<UdpHandNode>(LoopmodelOptions());

  // Claim the aux group before add_node, the ordering test_udp_hand_node_lanes
  // pins as safe, then service it from a thread of its own — the hand_aux_io
  // lane's shape in main().
  rclcpp::executors::SingleThreadedExecutor aux_exec;
  aux_exec.add_callback_group(node->GetAuxCallbackGroup(), node->get_node_base_interface());
  exec_.add_node(node->get_node_base_interface());

  std::thread aux_thread([&aux_exec] { aux_exec.spin(); });

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  const auto csv = session_dir_ / "timing" / "hand_udp_timing_log.csv";
  ASSERT_TRUE(std::filesystem::exists(csv));
  const auto size_before_drain = std::filesystem::file_size(csv);

  // Deliberately sleep instead of SpinUntil: pumping exec_ here would service the
  // node's default callback group too, so a drain timer wrongly created outside
  // cb_group_aux_ would still run and the growth below would prove nothing. With
  // only aux_thread spinning, growth can come from nothing but the aux lane. The
  // CommLoop keeps filling the SPSC on its own thread either way.
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  bool drained = false;
  while (!drained && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(50ms);
    drained = std::filesystem::file_size(csv) > size_before_drain;
  }
  EXPECT_TRUE(drained) << "the 1 Hz aux timer never drained the timing CSV — cb_group_aux_ "
                          "is not being serviced by the aux executor";

  // Tear the runtime down while the aux executor is still spinning: this is the
  // window StopRuntime()'s quiesce barrier exists for. Note loopmodel disables
  // the failure detector (enable_fd && !use_fake_hand_), so the detector-reset
  // ordering inside that barrier is exercised only on real/firmware SIL paths.
  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  aux_exec.cancel();
  aux_thread.join();
  aux_exec.remove_callback_group(node->GetAuxCallbackGroup());
  exec_.remove_node(node->get_node_base_interface());
}

// shutdown() from Active must run the deactivate path. rclcpp_lifecycle dispatches
// a transition callback only after the machine has entered the transition state,
// so on_shutdown's former `get_current_state() == PRIMARY_STATE_ACTIVE` guard was
// never true: shutdown-from-Active skipped deactivate entirely (issue #345).
//
// Discriminator: the timing CSV again, made timing-independent by never spinning
// the aux executor here — with cb_group_aux_ unserviced the 1 Hz drain cannot run
// at all, so the file cannot grow while Active and the only remaining writer is
// on_deactivate's straggler drain. Growth across shutdown() therefore proves the
// deactivate path ran.
TEST_F(UdpHandNodeLifecycleTest, ShutdownFromActiveRunsTheDeactivatePath) {
  auto node = std::make_shared<UdpHandNode>(LoopmodelOptions());
  auto probe = std::make_shared<rclcpp::Node>("hand_shutdown_probe");

  int joint_count = 0;
  auto joint_sub = probe->create_subscription<sensor_msgs::msg::JointState>(
      "/hand/joint_states", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      [&](sensor_msgs::msg::JointState::SharedPtr) { ++joint_count; });

  exec_.add_node(node->get_node_base_interface());
  exec_.add_node(probe);

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Several published states means thousands of 2 kHz CommLoop ticks have pushed
  // timing samples, so the SPSC certainly holds undrained rows.
  ASSERT_TRUE(SpinUntil([&] { return joint_count >= 5; }, 10s));

  const auto csv = session_dir_ / "timing" / "hand_udp_timing_log.csv";
  ASSERT_TRUE(std::filesystem::exists(csv));
  const auto size_before_shutdown = std::filesystem::file_size(csv);

  ASSERT_EQ(node->shutdown().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);

  EXPECT_GT(std::filesystem::file_size(csv), size_before_shutdown)
      << "shutdown() from Active never drained the timing CSV — on_deactivate was "
         "skipped, so the runtime was released without the deactivate path";

  exec_.remove_node(probe);
  exec_.remove_node(node->get_node_base_interface());
}

// Configure must REJECT a non-positive loop_rate_hz, not absorb it. The NRT
// publish timer's period is 1 / (rate * kPublishPollOversampling), so rate == 0
// makes that +inf and duration_cast<nanoseconds> performs an out-of-range
// double -> int64 conversion (UB); PrepareRun()'s own guard runs at on_activate,
// after the timer was already built (issue #345).
//
// What this test does and does not pin, measured rather than assumed: replacing
// the guard with a silent fallback — the `(rate > 0.0) ? rate : default` shape
// used by the two sibling rate computations in the same function — turns this
// red, which is the regression worth guarding. Deleting the guard outright
// leaves it green, because rcl_timer_init2 also rejects the resulting negative
// period and the transition still ends Unconfigured; the guard's remaining value
// there is avoiding the UB and emitting an actionable message instead of
// "timer period must be non-negative".
TEST_F(UdpHandNodeLifecycleTest, NonPositiveLoopRateIsRejectedAtConfigure) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("sil_mode", "loopmodel"),
      rclcpp::Parameter("loop_rate_hz", 0.0),
  });
  auto node = std::make_shared<UdpHandNode>(options);
  exec_.add_node(node->get_node_base_interface());

  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
      << "configure() accepted loop_rate_hz = 0 and built a publish timer from an "
         "infinite period";

  exec_.remove_node(node->get_node_base_interface());
}

}  // namespace
