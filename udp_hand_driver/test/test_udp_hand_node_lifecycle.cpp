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

#include "silent_loopback_socket.hpp"
#include "udp_hand_driver/udp_hand_node.hpp"
#include <rtc_msgs/msg/calibration_command.hpp>
#include <rtc_msgs/msg/calibration_status.hpp>
#include <rtc_msgs/msg/hand_sensor_state.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifndef UDP_HAND_TEST_FIXTURE_DIR
#error "UDP_HAND_TEST_FIXTURE_DIR must be set by CMake (test/fixtures)"
#endif

namespace {

using namespace std::chrono_literals;

// Dotted key paths of every object member in a JSON document made of objects
// and scalars only (SaveCommStats emits no arrays). Root-level keys have no
// prefix; nested ones read `comm_stats.per_request.write_echo.ok`. A string is
// a key when the next non-blank character is ':'; the writer emits no escaped
// quotes, so a plain scan is exact for this file.
std::set<std::string> JsonKeyPaths(const std::string& json) {
  std::set<std::string> out;
  std::vector<std::string> stack;
  std::string pending;
  for (std::size_t i = 0; i < json.size(); ++i) {
    const char c = json[i];
    if (c == '"') {
      const auto end = json.find('"', i + 1);
      if (end == std::string::npos) {
        break;
      }
      const std::string s = json.substr(i + 1, end - i - 1);
      i = end;
      std::size_t j = i + 1;
      while (j < json.size() && std::isspace(static_cast<unsigned char>(json[j])) != 0) {
        ++j;
      }
      if (j < json.size() && json[j] == ':') {
        std::string path;
        for (const auto& k : stack) {
          if (!k.empty()) {
            path += k + '.';
          }
        }
        path += s;
        out.insert(path);
        pending = s;
      }
    } else if (c == '{') {
      stack.push_back(pending);  // root pushes "" (skipped when joining)
      pending.clear();
    } else if (c == '}') {
      if (!stack.empty()) {
        stack.pop_back();
      }
    }
  }
  return out;
}

std::string ReadFile(const std::filesystem::path& p) {
  std::ifstream ifs(p);
  std::stringstream ss;
  ss << ifs.rdbuf();
  return ss.str();
}

// One non-empty, non-`#` line per entry.
std::set<std::string> ReadFixtureLines(const std::filesystem::path& p) {
  std::set<std::string> out;
  std::ifstream ifs(p);
  std::string line;
  while (std::getline(ifs, line)) {
    if (!line.empty() && line[0] != '#') {
      out.insert(line);
    }
  }
  return out;
}

std::string FirstLine(const std::string& text) {
  const auto nl = text.find('\n');
  return nl == std::string::npos ? text : text.substr(0, nl);
}

// Every topic below is subscribed by its absolute /hand/... name while the node
// itself declares only relative defaults (D53). The namespace is supplied the
// way the standalone launch supplies it, so a default that silently regressed
// to an absolute name would still resolve to the same topic — which is why
// RelativeTopicDefaultsResolveUnderTheNamespace pins the parameter values too.
constexpr const char* kTestNamespace = "/hand";

rclcpp::NodeOptions LoopmodelOptions() {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", std::string("__ns:=") + kTestNamespace});
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
    // The node writes <session>/device/hand_stats.json (SaveCommStats) and
    // <session>/timing/hand_timing_log.csv (on_activate). ResolveSessionDir
    // honours this env var first.
    setenv("RTC_SESSION_DIR", created, 1);
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
    std::error_code ec;
    std::filesystem::remove_all(session_dir_, ec);
  }

  // Stays local instead of moving to the shared rtc::testing::WaitUntil (#356):
  // that helper only sleeps, and this fixture has no other spinner — the node's
  // publish lane runs on exec_, so a sleep-poll wait here would never observe
  // progress and every test would time out. Pumping the executor is a different
  // primitive from sleeping, and which one a wait uses is load-bearing in this
  // file: the aux-lane test below deliberately sleeps instead of calling this,
  // because pumping exec_ there would service the default callback group and
  // void the assertion.
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

// D53: the eight topic parameters default to relative names and the namespace
// alone decides where they land. Both halves are asserted — the resolved names
// on the graph, because that is the runtime contract (G-1), and the raw
// parameter values, because a default rewritten back to "/hand/joint_states"
// would resolve to the same topic under this namespace and pass the first half.
TEST_F(UdpHandNodeLifecycleTest, RelativeTopicDefaultsResolveUnderTheNamespace) {
  auto node = std::make_shared<UdpHandNode>(LoopmodelOptions());
  exec_.add_node(node->get_node_base_interface());

  EXPECT_STREQ(node->get_namespace(), kTestNamespace);
  EXPECT_EQ(node->get_fully_qualified_name(), std::string(kTestNamespace) + "/udp_hand_node");

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const std::map<std::string, std::string> expected_relative = {
      {"command_topic", "joint_command"},
      {"joint_state_topic", "joint_states"},
      {"motor_state_topic", "motor_states"},
      {"sensor_topic", "sensor_states"},
      {"link_status_topic", "link_status"},
      {"estop_topic", "estop"},
      {"calibration_command_topic", "calibration/command"},
      {"calibration_status_topic", "calibration/status"},
  };
  for (const auto& [name, relative] : expected_relative) {
    EXPECT_EQ(node->get_parameter(name).as_string(), relative)
        << name << " default is no longer the relative name";
  }

  // The resolved graph names: publishers for the five outputs, subscriptions
  // for the three inputs. The monitor topic derives from sensor_topic.
  const auto resolved = [&](const std::string& relative) {
    return std::string(kTestNamespace) + "/" + relative;
  };
  for (const char* out : {"joint_states", "motor_states", "sensor_states", "sensor_states/monitor",
                          "link_status", "calibration/status"}) {
    EXPECT_EQ(node->count_publishers(resolved(out)), 1u) << "no publisher on " << resolved(out);
  }
  for (const char* in : {"joint_command", "calibration/command", "estop"}) {
    EXPECT_EQ(node->count_subscribers(resolved(in)), 1u) << "no subscription on " << resolved(in);
  }

  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

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

  // The timing CSV opened on activation (hand_timing_opened_).
  EXPECT_TRUE(std::filesystem::exists(session_dir_ / "timing" / "hand_timing_log.csv"));

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
  EXPECT_TRUE(std::filesystem::exists(session_dir_ / "device" / "hand_stats.json"));

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

  const auto csv = session_dir_ / "timing" / "hand_timing_log.csv";
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

  const auto csv = session_dir_ / "timing" / "hand_timing_log.csv";
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

// Forensic-artifact schema gate (G-1 of epic #895, #897): the key set of
// <session>/device/hand_stats.json must stay a SUPERSET of the checked-in
// fixture, and the <session>/timing/hand_timing_log.csv header must match
// it exactly. Analysis scripts and the hand-stats readers key on these names, so
// a transport seam or a new protocol may ADD keys but never drop or rename one.
// The fixtures were generated from this very run on the pre-seam tree (main
// d7d38af7); regenerate them only in a commit that deliberately changes the
// schema (new keys: append the line; the CSV header is rtc_base's
// RtTickTimingPayload schema shared with CM / MPC). Loopmodel runs the node's
// default (individual) comm mode, so the individual-mode timing keys are the
// ones pinned; the bulk-only keys are not in this fixture.
TEST_F(UdpHandNodeLifecycleTest, StatsJsonKeysAndTimingCsvHeaderMatchFixture) {
  auto node = std::make_shared<UdpHandNode>(LoopmodelOptions());
  exec_.add_node(node->get_node_base_interface());

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  // Enough cycles that every sensor-cycle-gated timing block (sensor_proc_us)
  // has samples and appears in the JSON.
  SpinFor(500ms);
  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  exec_.remove_node(node->get_node_base_interface());

  const std::filesystem::path fixtures = UDP_HAND_TEST_FIXTURE_DIR;

  // ── hand_stats.json: key superset ──────────────────────────────────
  const auto json_path = session_dir_ / "device" / "hand_stats.json";
  ASSERT_TRUE(std::filesystem::exists(json_path));
  const auto actual_keys = JsonKeyPaths(ReadFile(json_path));
  const auto expected_keys = ReadFixtureLines(fixtures / "hand_stats_keys.txt");
  ASSERT_FALSE(expected_keys.empty())
      << "fixture missing or empty: " << (fixtures / "hand_stats_keys.txt");
  std::vector<std::string> missing;
  for (const auto& k : expected_keys) {
    if (actual_keys.count(k) == 0) {
      missing.push_back(k);
    }
  }
  std::string actual_dump;
  for (const auto& k : actual_keys) {
    actual_dump += "  " + k + "\n";
  }
  EXPECT_TRUE(missing.empty()) << "hand_stats.json dropped " << missing.size()
                               << " fixture key(s); first: " << missing.front()
                               << "\nactual keys:\n"
                               << actual_dump;

  // ── hand_timing_log.csv: header identical ──────────────────────────
  const auto csv_path = session_dir_ / "timing" / "hand_timing_log.csv";
  ASSERT_TRUE(std::filesystem::exists(csv_path));
  const std::string actual_header = FirstLine(ReadFile(csv_path));
  const std::string expected_header = FirstLine(ReadFile(fixtures / "hand_timing_log_header.txt"));
  ASSERT_FALSE(expected_header.empty())
      << "fixture missing or empty: " << (fixtures / "hand_timing_log_header.txt");
  EXPECT_EQ(actual_header, expected_header);
}

// ── protocol_version "2" (CAN FD, epic #895 — F04-2d) ──────────────────────
// Every configure-time rejection below must end in Unconfigured *before* a
// controller or publisher exists. No test here reconfigures the same node:
// on_configure re-declares its parameters, so cleanup → configure throws on a
// redeclaration (pre-existing, out of this feature's scope).

rclcpp::NodeOptions Proto2Options(std::vector<rclcpp::Parameter> extra) {
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("sil_mode", "off"),
      rclcpp::Parameter("protocol_version", "2"),
      rclcpp::Parameter("communication_mode", "bulk"),
      // No such device on any PC (rig PCs do have can0); keeps the interface
      // check the one that fires only where the test says it does.
      rclcpp::Parameter("can_interface", "rtcnone0"),
      // The shipped YAML value. The C++ declare default (10 ms) cannot satisfy
      // D42 at 500 Hz, so a proto2 node always needs this key set.
      rclcpp::Parameter("recv_timeout_ms", 0.4),
  };
  params.insert(params.end(), extra.begin(), extra.end());
  options.parameter_overrides(params);
  return options;
}

// D11 / D16: the CAN FD link is bulk-only; "2" + individual is a configuration
// error, not a silent fallback.
TEST_F(UdpHandNodeLifecycleTest, Proto2RejectsIndividualCommunicationMode) {
  auto node = std::make_shared<UdpHandNode>(Proto2Options({
      rclcpp::Parameter("sil_mode", "loopmodel"),
      rclcpp::Parameter("communication_mode", "individual"),
  }));
  exec_.add_node(node->get_node_base_interface());
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  exec_.remove_node(node->get_node_base_interface());
}

// A missing CAN interface is refused at configure on the real-hardware path
// (an Open() failure at activate would be too late and too vague) …
TEST_F(UdpHandNodeLifecycleTest, Proto2RejectsAMissingCanInterfaceOutsideLoopmodel) {
  auto node = std::make_shared<UdpHandNode>(Proto2Options({}));
  exec_.add_node(node->get_node_base_interface());
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  exec_.remove_node(node->get_node_base_interface());
}

// … while loopmodel never opens the socket, so the same missing interface is
// accepted there and the node reaches Inactive (the dev-box path).
TEST_F(UdpHandNodeLifecycleTest, Proto2LoopmodelAcceptsAMissingCanInterface) {
  auto node = std::make_shared<UdpHandNode>(Proto2Options({
      rclcpp::Parameter("sil_mode", "loopmodel"),
  }));
  exec_.add_node(node->get_node_base_interface());
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  exec_.remove_node(node->get_node_base_interface());
}

// An interface name the kernel could never hold is refused in every sil_mode.
TEST_F(UdpHandNodeLifecycleTest, Proto2RejectsAnOverlongCanInterfaceName) {
  auto node = std::make_shared<UdpHandNode>(Proto2Options({
      rclcpp::Parameter("sil_mode", "loopmodel"),
      rclcpp::Parameter("can_interface", std::string(64, 'x')),
  }));
  exec_.add_node(node->get_node_base_interface());
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  exec_.remove_node(node->get_node_base_interface());
}

// D51: hand_side is parsed on every protocol (it names the forensic files).
TEST_F(UdpHandNodeLifecycleTest, InvalidHandSideIsRejectedOnEveryProtocol) {
  for (const char* proto : {"1a", "2"}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("sil_mode", "loopmodel"),
        rclcpp::Parameter("protocol_version", proto),
        rclcpp::Parameter("communication_mode", "bulk"),
        rclcpp::Parameter("hand_side", "middle"),
    });
    auto node = std::make_shared<UdpHandNode>(options);
    exec_.add_node(node->get_node_base_interface());
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        << "protocol " << proto;
    exec_.remove_node(node->get_node_base_interface());
  }
}

// D50 / NUM-2: feedforward_kt must be 10 values, each > 0 with a finite
// reciprocal — the same predicate CanFdHandComm::Open() applies.
TEST_F(UdpHandNodeLifecycleTest, Proto2RejectsAnUnusableFeedforwardKt) {
  const std::vector<std::vector<double>> bad = {
      std::vector<double>(9, 1.0),                                  // wrong size
      {1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},           // zero
      {1.0, 1.0, 1.0, -2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},          // negative
      {1.0, 1.0, 1.0, 1e-40, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},         // 1/kt overflows
      {1.0, 1.0, 1.0, std::nan(""), 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},  // NaN
  };
  for (std::size_t i = 0; i < bad.size(); ++i) {
    auto node = std::make_shared<UdpHandNode>(Proto2Options({
        rclcpp::Parameter("sil_mode", "loopmodel"),
        rclcpp::Parameter("feedforward_kt", bad[i]),
    }));
    exec_.add_node(node->get_node_base_interface());
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        << "case " << i;
    exec_.remove_node(node->get_node_base_interface());
  }
}

// D42: both receive stages must fit the comm slot — 2 × recv_timeout_ms <
// 1000 / loop_rate_hz × comm_decimation. 500 Hz gives a 2 ms slot: 1.0 ms is
// out (2.0 < 2.0 fails), 0.9 ms is in; decimation 2 widens the slot to 4 ms.
TEST_F(UdpHandNodeLifecycleTest, Proto2RejectsARecvTimeoutThatCannotFitTheSlot) {
  struct Case {
    double recv_timeout_ms;
    int comm_decimation;
    bool accepted;
  };

  const Case cases[] = {
      {1.0, 1, false}, {0.9, 1, true}, {0.0, 1, false}, {-0.4, 1, false}, {1.5, 2, true},
  };
  for (const auto& c : cases) {
    auto node = std::make_shared<UdpHandNode>(Proto2Options({
        rclcpp::Parameter("sil_mode", "loopmodel"),
        rclcpp::Parameter("loop_rate_hz", 500.0),
        rclcpp::Parameter("recv_timeout_ms", c.recv_timeout_ms),
        rclcpp::Parameter("comm_decimation", c.comm_decimation),
    }));
    exec_.add_node(node->get_node_base_interface());
    const auto state = node->configure().id();
    if (c.accepted) {
      EXPECT_EQ(state, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
          << c.recv_timeout_ms << " ms / decimation " << c.comm_decimation;
      ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
    } else {
      EXPECT_EQ(state, lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
          << c.recv_timeout_ms << " ms / decimation " << c.comm_decimation;
    }
    exec_.remove_node(node->get_node_base_interface());
  }
}

std::string Dump(const std::set<std::string>& keys) {
  std::string out;
  for (const auto& k : keys) {
    out += "  " + k + "\n";
  }
  return out;
}

// Forensic schema of a proto2 run: the stats JSON key set must EQUAL
// fixtures/hand_stats_keys_proto2.txt (bulk timing keys + comm_stats.canfd.*
// + the keys every protocol emits) and the timing CSV header must still match
// the shared fixture. Exact, not superset: the proto1 fixture pins the
// individual-mode keys, so it cannot double as this gate, and G-2's SIL run
// compares its JSON against this same file. loopmodel never opens the CAN
// socket, but the comm is constructed, so the canfd block is present with
// zero counters — the schema this pins is the one the hardware run writes.
TEST_F(UdpHandNodeLifecycleTest, Proto2StatsJsonKeysMatchTheProto2FixtureExactly) {
  auto node = std::make_shared<UdpHandNode>(Proto2Options({
      rclcpp::Parameter("sil_mode", "loopmodel"),
      rclcpp::Parameter("loop_rate_hz", 2000.0),
      rclcpp::Parameter("publish_rate", 2000.0),
      // D42 at 2 kHz: the slot is 0.5 ms, so the shipped 0.4 ms is refused.
      rclcpp::Parameter("recv_timeout_ms", 0.2),
  }));
  exec_.add_node(node->get_node_base_interface());

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  SpinFor(500ms);  // sensor-cycle-gated blocks (sensor_proc_us) need samples
  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  exec_.remove_node(node->get_node_base_interface());

  const std::filesystem::path fixtures = UDP_HAND_TEST_FIXTURE_DIR;
  const auto json_path = session_dir_ / "device" / "hand_stats.json";
  ASSERT_TRUE(std::filesystem::exists(json_path));
  const std::string json = ReadFile(json_path);
  const auto actual_keys = JsonKeyPaths(json);
  const auto expected_keys = ReadFixtureLines(fixtures / "hand_stats_keys_proto2.txt");
  ASSERT_FALSE(expected_keys.empty())
      << "fixture missing or empty: " << (fixtures / "hand_stats_keys_proto2.txt")
      << "\nactual keys:\n"
      << Dump(actual_keys);
  EXPECT_EQ(actual_keys, expected_keys) << "actual keys:\n" << Dump(actual_keys);

  // The tags say what ran; the proto1 superset still holds (nothing dropped).
  EXPECT_NE(json.find("\"protocol_version\": \"2\""), std::string::npos);
  EXPECT_NE(json.find("\"transport\": \"canfd\""), std::string::npos);
  for (const auto& k : ReadFixtureLines(fixtures / "hand_stats_keys.txt")) {
    if (k.rfind("timing_stats.read_", 0) == 0) {
      continue;  // individual-mode timing keys — bulk writes its own set
    }
    EXPECT_EQ(actual_keys.count(k), 1u) << "proto1 key dropped on proto2: " << k;
  }

  const auto csv_path = session_dir_ / "timing" / "hand_timing_log.csv";
  ASSERT_TRUE(std::filesystem::exists(csv_path));
  EXPECT_EQ(FirstLine(ReadFile(csv_path)),
            FirstLine(ReadFile(fixtures / "hand_timing_log_header.txt")));
}

// D39 through the node: the command callback's feedforward branches, observed
// through the two counters the stats JSON carries. Four messages in order —
// no feedforward (silent), a matching size (accepted), a wrong size (zeroed +
// counted), a matching size with a NaN (that entry zeroed + counted). The
// loopmodel fake ignores the feedforward value itself, so what this pins is
// branch selection and the per-message accounting, not the wire payload (the
// controller seam test covers that the staged ff reaches the comm).
TEST_F(UdpHandNodeLifecycleTest, JointCommandFeedforwardBranchesAreCounted) {
  auto node = std::make_shared<UdpHandNode>(LoopmodelOptions());
  auto probe = std::make_shared<rclcpp::Node>("hand_feedforward_probe");
  // The node's command subscription is best_effort depth 1.
  auto cmd_pub = probe->create_publisher<rtc_msgs::msg::JointCommand>("/hand/joint_command",
                                                                      rclcpp::QoS(1).best_effort());

  exec_.add_node(node->get_node_base_interface());
  exec_.add_node(probe);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil([&] { return cmd_pub->get_subscription_count() > 0; }, 10s));

  rtc_msgs::msg::JointCommand msg;
  msg.joint_names = udp_hand_driver::kDefaultHandMotorNames;
  msg.values.assign(static_cast<std::size_t>(udp_hand_driver::kNumHandMotors), 0.1);
  const auto send = [&](std::vector<double> ff) {
    msg.feedforward = std::move(ff);
    cmd_pub->publish(msg);
    SpinFor(150ms);  // best_effort: give DDS time to deliver before the next one
  };
  send({});  // absent → silent
  send(std::vector<double>(static_cast<std::size_t>(udp_hand_driver::kNumHandMotors), 0.5));
  send({1.0, 2.0, 3.0});  // size mismatch
  std::vector<double> with_nan(static_cast<std::size_t>(udp_hand_driver::kNumHandMotors), 0.5);
  with_nan[4] = std::nan("");
  send(with_nan);  // non-finite entry

  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  exec_.remove_node(probe);
  exec_.remove_node(node->get_node_base_interface());

  const std::string json = ReadFile(session_dir_ / "device" / "hand_stats.json");
  EXPECT_NE(json.find("\"feedforward_size_mismatch\": 1,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"feedforward_nonfinite\": 1,"), std::string::npos) << json;
}

// D51: an explicit hand_side suffixes both forensic artifacts; the default
// names above stay as they were, so one-hand deployments see no rename.
TEST_F(UdpHandNodeLifecycleTest, ExplicitHandSideSuffixesTheForensicFiles) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("sil_mode", "loopmodel"),
      rclcpp::Parameter("hand_side", "left"),
      rclcpp::Parameter("loop_rate_hz", 2000.0),
  });
  auto node = std::make_shared<UdpHandNode>(options);
  exec_.add_node(node->get_node_base_interface());

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  SpinFor(200ms);
  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  exec_.remove_node(node->get_node_base_interface());

  EXPECT_TRUE(std::filesystem::exists(session_dir_ / "device" / "hand_stats_left.json"));
  EXPECT_TRUE(std::filesystem::exists(session_dir_ / "timing" / "hand_timing_log_left.csv"));
  // The suffixed run is a proto1 UDP run: tagged as such.
  const std::string json = ReadFile(session_dir_ / "device" / "hand_stats_left.json");
  EXPECT_NE(json.find("\"protocol_version\": \"1a\""), std::string::npos);
  EXPECT_NE(json.find("\"transport\": \"udp\""), std::string::npos);
  EXPECT_EQ(json.find("\"canfd\""), std::string::npos);
}

// F08 (#905): the runtime names are link-neutral. The CommLoop thread is
// `hand_comm` (kHandCommThreadConfig; the name is set before the affinity and
// scheduler steps, so it survives an EPERM on SCHED_FIFO), and the forensic
// artifacts carry no `hand_udp_` prefix. The external-layout verifier
// (repo_scripts verify_rt_runtime.sh) and the gate tool key on these names;
// its unit tests use synthetic records, so this is the only check against the
// binary.
TEST_F(UdpHandNodeLifecycleTest, CommThreadAndArtifactsCarryTheNeutralNames) {
  auto node = std::make_shared<UdpHandNode>(LoopmodelOptions());
  exec_.add_node(node->get_node_base_interface());

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  SpinFor(200ms);

  std::map<std::string, int> thread_names;
  for (const auto& task : std::filesystem::directory_iterator("/proc/self/task")) {
    std::ifstream comm(task.path() / "comm");
    std::string name;
    std::getline(comm, name);
    ++thread_names[name];
  }
  EXPECT_EQ(thread_names["hand_comm"], 1) << "CommLoop thread not named hand_comm while active";
  EXPECT_EQ(thread_names["hand_udp_recv"], 0) << "pre-F08 thread name still in use";

  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  exec_.remove_node(node->get_node_base_interface());

  EXPECT_TRUE(std::filesystem::exists(session_dir_ / "device" / "hand_stats.json"));
  EXPECT_TRUE(std::filesystem::exists(session_dir_ / "timing" / "hand_timing_log.csv"));
  for (const auto* sub : {"device", "timing"}) {
    for (const auto& entry : std::filesystem::directory_iterator(session_dir_ / sub)) {
      const std::string name = entry.path().filename().string();
      EXPECT_NE(name.rfind("hand_udp_", 0), 0u) << "pre-F08 artifact written: " << entry.path();
    }
  }
}

// ── E-Stop input (E-8) ───────────────────────────────────────────────────────
// A loopback UDP socket the test owns and never answers stands in for the hand
// (test/silent_loopback_socket.hpp): sil_mode=off opens the real UdpHandComm
// against it, so this is the production wire end to end — not the scripted seam
// test_udp_hand_controller uses.
bool IsWritePosition(const std::vector<std::uint8_t>& d) {
  return d.size() == udp_hand_driver::packets::kMotorPacketSize &&
         d[1] == static_cast<std::uint8_t>(udp_hand_driver::packets::Command::kWritePosition);
}

// The estop_topic input reaches the CommLoop latch and the wire. With no hand
// answering, every cycle is read requests that time out and no write ever goes
// out (no command is sent, and the write gate holds until a first read
// succeeds), so the only write-position datagram the socket can ever see is the
// terminal E-Stop zero-write: exactly one, all-zero payload, and silence after
// it because the loop has exited. A `false` must not trip it, and
// deactivate → activate must clear the latch so the reads resume — the
// documented way back.
TEST_F(UdpHandNodeLifecycleTest, EstopTopicZeroWritesOnceAndStopsTheLoop) {
  udp_hand_driver::test_support::SilentLoopbackSocket hand;
  ASSERT_TRUE(hand.ok());

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", std::string("__ns:=") + kTestNamespace});
  options.parameter_overrides({
      rclcpp::Parameter("sil_mode", "off"),
      rclcpp::Parameter("target_ip", "127.0.0.1"),
      rclcpp::Parameter("target_port", hand.port()),
      // Three timed-out reads per cycle must fit the 500 Hz slot.
      rclcpp::Parameter("recv_timeout_ms", 0.3),
      rclcpp::Parameter("loop_rate_hz", 500.0),
      // The detector would otherwise declare the (deliberately dead) link a
      // failure; it is not what this test is about.
      rclcpp::Parameter("enable_failure_detector", false),
  });
  auto node = std::make_shared<UdpHandNode>(options);
  auto probe = std::make_shared<rclcpp::Node>("hand_estop_probe");
  auto estop_pub =
      probe->create_publisher<std_msgs::msg::Bool>("/hand/estop", rclcpp::QoS(1).reliable());
  exec_.add_node(node->get_node_base_interface());
  exec_.add_node(probe);

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil([&] { return estop_pub->get_subscription_count() > 0; }, 10s));

  std::vector<std::vector<std::uint8_t>> rx;
  // The loop is running: read requests are arriving.
  ASSERT_TRUE(SpinUntil(
      [&] {
        hand.Drain(rx);
        return !rx.empty();
      },
      10s));
  std_msgs::msg::Bool msg;
  msg.data = false;
  estop_pub->publish(msg);
  SpinFor(200ms);
  hand.Drain(rx);
  EXPECT_TRUE(std::none_of(rx.begin(), rx.end(), IsWritePosition))
      << "a write-position datagram went out before any E-Stop";
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  rx.clear();

  msg.data = true;
  estop_pub->publish(msg);
  ASSERT_TRUE(SpinUntil(
      [&] {
        hand.Drain(rx);
        return std::any_of(rx.begin(), rx.end(), IsWritePosition);
      },
      10s));
  // Silence after the zero-write: the loop has exited, nothing else may send.
  SpinFor(300ms);
  hand.Drain(rx);
  const auto first_write = std::find_if(rx.begin(), rx.end(), IsWritePosition);
  ASSERT_NE(first_write, rx.end());
  EXPECT_EQ(std::count_if(rx.begin(), rx.end(), IsWritePosition), 1);
  EXPECT_EQ(std::next(first_write), rx.end()) << "datagrams were sent after the E-Stop zero-write";
  // id, cmd, mode, then ten float positions — all zero (D26: zero position).
  EXPECT_TRUE(std::all_of(first_write->begin() + 3, first_write->end(),
                          [](std::uint8_t b) { return b == 0; }));
  // The node stays ACTIVE with a stopped loop: resuming is the operator's call.
  EXPECT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // A second `true` on a stopped loop changes nothing.
  estop_pub->publish(msg);
  SpinFor(200ms);
  rx.clear();
  hand.Drain(rx);
  EXPECT_TRUE(rx.empty());

  // deactivate → activate clears the latch: reads resume, no write.
  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  rx.clear();
  ASSERT_TRUE(SpinUntil(
      [&] {
        hand.Drain(rx);
        return !rx.empty();
      },
      10s));
  SpinFor(100ms);
  hand.Drain(rx);
  EXPECT_TRUE(std::none_of(rx.begin(), rx.end(), IsWritePosition))
      << "the E-Stop latch survived deactivate → activate";

  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  exec_.remove_node(probe);
  exec_.remove_node(node->get_node_base_interface());
}

}  // namespace
