// Every state lane of every shipped backend honours DeviceBackend::
// CloseStateLane() (#941).
//
// rtc_controller_manager owns the gate and the teardown that closes it
// (test_state_lane_teardown there). What it cannot see is whether a concrete
// backend actually routed its subscription callbacks through the gate — a lane
// created with a bare `[this]` lambda compiles, runs, and is destroyed under an
// in-flight callback exactly as before. This file is the enforcement: one case
// per lane, each proving that a message delivered AFTER the lane was closed
// leaves that lane's observable untouched.
//
// Each negative assertion stands behind a witness subscription on the same
// topic, pumped by the same executor: when the witness has seen the message,
// the executor has received it, so "the cache did not change" means the lane
// rejected it, not that nothing arrived.
//
// Lanes (one subscription each, all on the state callback group):
//   mujoco_native     joint (/joint_states)           wrench (fingertip topics)
//   ur_driver_native  joint
//   udp_hand_native   joint · motor · sensor

#include "integrated_bringup/backends/mujoco_native_backend.hpp"
#include "integrated_bringup/backends/udp_hand_native_backend.hpp"
#include "integrated_bringup/backends/ur_driver_native_backend.hpp"
#include "integrated_bringup/controllers/hand_sensor_layout.hpp"
#include "rtc_controller_manager/device_backend.hpp"
#include "rtc_controller_manager/device_state_cache.hpp"
#include <rtc_msgs/msg/hand_sensor_state.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;

constexpr std::size_t kInferStride = integrated_bringup::kHandInferenceValuesPerFingertipCapacity;
const std::vector<std::string> kJointNames = {"j0", "j1", "j2"};

// One topic the test publishes on and watches: a RELIABLE publisher (matches
// the BEST_EFFORT and RELIABLE readers alike) plus the witness reader.
template <typename MsgT>
struct Lane {
  typename rclcpp::Publisher<MsgT>::SharedPtr pub;
  typename rclcpp::Subscription<MsgT>::SharedPtr witness;
  std::atomic<int> seen{0};
};

// The node that hosts the backend, the node that plays the device, and the
// one executor that pumps both — on this thread, so every assertion knows
// exactly what has been dispatched (PROC-8: a TU-local pump).
class Harness {
 public:
  explicit Harness(const std::string& tag,
                   const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : node(std::make_shared<rclcpp_lifecycle::LifecycleNode>("sl_" + tag, options)),
        group(node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
        device(std::make_shared<rclcpp::Node>("sl_device_" + tag)) {
    exec.add_node(node->get_node_base_interface());
    exec.add_node(device->get_node_base_interface());
  }

  ~Harness() {
    exec.remove_node(device->get_node_base_interface());
    exec.remove_node(node->get_node_base_interface());
  }

  // Call after the backend is configured: waits until the publisher sees both
  // readers (the backend's and the witness), so the first message is not lost
  // to discovery.
  template <typename MsgT>
  std::unique_ptr<Lane<MsgT>> OpenLane(const std::string& topic) {
    auto lane = std::make_unique<Lane<MsgT>>();
    rclcpp::QoS qos{1};
    qos.reliable();
    lane->pub = device->create_publisher<MsgT>(topic, qos);
    auto* seen = &lane->seen;
    lane->witness = device->create_subscription<MsgT>(
        topic, qos, [seen](typename MsgT::ConstSharedPtr) { seen->fetch_add(1); });
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (lane->pub->get_subscription_count() < 2 && std::chrono::steady_clock::now() < deadline) {
      exec.spin_some(10ms);
    }
    EXPECT_GE(lane->pub->get_subscription_count(), 2U) << topic << ": readers never matched";
    return lane;
  }

  // Open lane: publishes once and pumps until the witness has the message AND
  // the backend's own observable (`decoded`) reports it. The witness alone is
  // not enough on the positive side — the backend's reader is a different DDS
  // reader of the same sample and may be served a spin later.
  template <typename MsgT, typename Pred>
  [[nodiscard]] bool DeliverUntil(Lane<MsgT>& lane, const MsgT& msg, Pred decoded) {
    const int before = lane.seen.load();
    lane.pub->publish(msg);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (lane.seen.load() == before || !decoded()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      exec.spin_some(10ms);
      std::this_thread::sleep_for(500us);
    }
    return true;
  }

  // Closed lane: publishes once, pumps until the witness has it, then keeps
  // pumping for a fixed wall-clock period. `spin_some` returns at once when
  // nothing is ready, so a count of calls would be no grace at all; a
  // deadline is. After this, a message the backend's reader still had not
  // been served would have had to lag the witness by the whole period.
  template <typename MsgT>
  [[nodiscard]] bool DeliverToClosedLane(Lane<MsgT>& lane, const MsgT& msg) {
    const int before = lane.seen.load();
    lane.pub->publish(msg);
    const auto witness_deadline = std::chrono::steady_clock::now() + 2s;
    while (lane.seen.load() == before) {
      if (std::chrono::steady_clock::now() >= witness_deadline) {
        return false;
      }
      exec.spin_some(10ms);
    }
    const auto grace_deadline = std::chrono::steady_clock::now() + 100ms;
    while (std::chrono::steady_clock::now() < grace_deadline) {
      exec.spin_some(5ms);
      std::this_thread::sleep_for(1ms);
    }
    return true;
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  rclcpp::CallbackGroup::SharedPtr group;
  std::shared_ptr<rclcpp::Node> device;
  rclcpp::executors::SingleThreadedExecutor exec;
};

sensor_msgs::msg::JointState JointStateAt(double base) {
  sensor_msgs::msg::JointState msg;
  msg.name = kJointNames;
  msg.position = {base, base + 1.0, base + 2.0};
  return msg;
}

rtc_msgs::msg::HandSensorState SensorStateWithForce(float fx) {
  rtc_msgs::msg::HandSensorState msg;
  msg.fingertips.resize(1);
  msg.fingertips[0].inference_enable = true;
  msg.fingertips[0].f[0] = fx;
  return msg;
}

geometry_msgs::msg::WrenchStamped WrenchWithForce(double fx) {
  geometry_msgs::msg::WrenchStamped msg;
  msg.wrench.force.x = fx;
  return msg;
}

// The joint lane is the same shape on all three backends: first message lands
// in ReadState and stamps LastStateStamp; after CloseStateLane neither moves.
template <typename BackendT>
void ExpectJointLaneClosed(Harness& h, BackendT& backend, const std::string& topic) {
  auto lane = h.OpenLane<sensor_msgs::msg::JointState>(topic);
  ASSERT_TRUE(h.DeliverUntil(*lane, JointStateAt(1.0), [&backend] {
    rtc::DeviceStateCache c{};
    return backend.ReadState(c) && c.positions[0] == 1.0;
  })) << "open joint lane never decoded the first message";
  rtc::DeviceStateCache before{};
  ASSERT_TRUE(backend.ReadState(before));
  ASSERT_DOUBLE_EQ(before.positions[0], 1.0);
  const auto stamp_before = backend.LastStateStamp();
  ASSERT_NE(stamp_before, std::chrono::steady_clock::time_point{});

  ASSERT_TRUE(backend.CloseStateLane());

  ASSERT_TRUE(h.DeliverToClosedLane(*lane, JointStateAt(10.0)))
      << "second joint state never delivered";
  rtc::DeviceStateCache after{};
  static_cast<void>(backend.ReadState(after));
  EXPECT_DOUBLE_EQ(after.positions[0], 1.0) << "closed joint lane still wrote the cache (#941)";
  EXPECT_EQ(backend.LastStateStamp(), stamp_before) << "closed joint lane still stamped (#941)";
}

// ── mujoco_native ────────────────────────────────────────────────────────────

constexpr const char* kMujocoGroup = "sl_mj";
const std::vector<std::string> kMujocoWrenchTopics = {"/sl_mj/ft0/contact_wrench"};

rclcpp::NodeOptions MujocoOptions() {
  rclcpp::NodeOptions options;
  const std::string prefix = std::string("devices.") + kMujocoGroup + ".backend";
  options.parameter_overrides({
      rclcpp::Parameter(prefix + ".fingertip_wrench_topics", kMujocoWrenchTopics),
      // High enough that the reads below never trip the staleness verdict;
      // what is asserted is the sample, not the verdict.
      rclcpp::Parameter(prefix + ".max_consecutive_missed_ticks", static_cast<int64_t>(1000)),
  });
  return options;
}

rtc::DeviceBackendConfig MujocoConfig(const std::string& tag) {
  rtc::DeviceBackendConfig cfg;
  cfg.group_name = kMujocoGroup;
  cfg.type = "mujoco_native";
  cfg.state_topic = "/sl_mj/" + tag + "/joint_states";
  cfg.command_topic = "/sl_mj/" + tag + "/joint_command";
  cfg.joint_command_names = kJointNames;
  return cfg;
}

TEST(StateLaneClose, MujocoJointLaneIgnoresMessagesAfterClose) {
  Harness h("mj_joint", MujocoOptions());
  const auto cfg = MujocoConfig("joint");
  rtc::MujocoNativeBackend backend;
  backend.Configure(h.node.get(), cfg, h.group);
  ExpectJointLaneClosed(h, backend, cfg.state_topic);
}

TEST(StateLaneClose, MujocoWrenchLaneIgnoresMessagesAfterClose) {
  Harness h("mj_wrench", MujocoOptions());
  rtc::MujocoNativeBackend backend;
  backend.Configure(h.node.get(), MujocoConfig("wrench"), h.group);

  auto lane = h.OpenLane<geometry_msgs::msg::WrenchStamped>(kMujocoWrenchTopics[0]);
  ASSERT_TRUE(h.DeliverUntil(*lane, WrenchWithForce(1.0), [&backend] {
    rtc::DeviceStateCache c{};
    backend.ReadSensorState(c);
    return c.inference_sequence[0] == 1U;
  })) << "open wrench lane never decoded the first message";
  rtc::DeviceStateCache before{};
  backend.ReadSensorState(before);
  ASSERT_EQ(before.inference_sequence[0], 1U);
  ASSERT_FLOAT_EQ(before.inference_data[1], 1.0F);  // slot 1 of fingertip 0 = fx

  ASSERT_TRUE(backend.CloseStateLane());

  ASSERT_TRUE(h.DeliverToClosedLane(*lane, WrenchWithForce(10.0)))
      << "second wrench never delivered";
  rtc::DeviceStateCache after{};
  backend.ReadSensorState(after);
  EXPECT_EQ(after.inference_sequence[0], 1U) << "closed wrench lane still counted a sample (#941)";
  EXPECT_FLOAT_EQ(after.inference_data[1], 1.0F)
      << "closed wrench lane still wrote the mirror (#941)";
}

// ── ur_driver_native ─────────────────────────────────────────────────────────

TEST(StateLaneClose, UrJointLaneIgnoresMessagesAfterClose) {
  Harness h("ur_joint");
  rtc::DeviceBackendConfig cfg;
  cfg.group_name = "sl_ur";
  cfg.type = "ur_driver_native";
  cfg.state_topic = "/sl_ur/joint_states";
  cfg.command_topic = "/sl_ur/forward_position_controller/commands";
  cfg.joint_command_names = kJointNames;
  rtc::UrDriverNativeBackend backend;
  backend.Configure(h.node.get(), cfg, h.group);
  ExpectJointLaneClosed(h, backend, cfg.state_topic);
}

// ── udp_hand_native ──────────────────────────────────────────────────────────

rtc::DeviceBackendConfig UdpConfig(const std::string& tag) {
  rtc::DeviceBackendConfig cfg;
  cfg.group_name = "sl_udp";
  cfg.type = "udp_hand_native";
  cfg.state_topic = "/sl_udp/" + tag + "/joint_states";
  cfg.command_topic = "/sl_udp/" + tag + "/joint_command";
  cfg.motor_topic = "/sl_udp/" + tag + "/motor_states";
  cfg.sensor_topic = "/sl_udp/" + tag + "/sensor_states";
  cfg.joint_command_names = kJointNames;
  return cfg;
}

// Layout BEFORE Configure (header contract); OnSensorState returns early
// without one, which would make the sensor-lane case vacuous.
rtc::DeviceSensorLayout UdpLayout() {
  rtc::DeviceSensorLayout layout;
  layout.primary_count_per_group = 8;
  layout.secondary_count_per_group = 3;
  layout.values_per_group = 11;
  layout.inference_values_per_group = static_cast<int>(kInferStride);
  return layout;
}

TEST(StateLaneClose, UdpJointLaneIgnoresMessagesAfterClose) {
  Harness h("udp_joint");
  const auto cfg = UdpConfig("joint");
  rtc::UdpHandNativeBackend backend;
  backend.SetSensorLayout(UdpLayout());
  backend.Configure(h.node.get(), cfg, h.group);
  ExpectJointLaneClosed(h, backend, cfg.state_topic);
}

TEST(StateLaneClose, UdpMotorLaneIgnoresMessagesAfterClose) {
  Harness h("udp_motor");
  const auto cfg = UdpConfig("motor");
  rtc::UdpHandNativeBackend backend;
  backend.SetSensorLayout(UdpLayout());
  backend.Configure(h.node.get(), cfg, h.group);

  auto lane = h.OpenLane<sensor_msgs::msg::JointState>(cfg.motor_topic);
  ASSERT_TRUE(h.DeliverUntil(*lane, JointStateAt(1.0), [&backend] {
    rtc::DeviceStateCache c{};
    backend.ReadMotorState(c);
    return c.num_motor_channels == 3;
  })) << "open motor lane never decoded the first message";
  rtc::DeviceStateCache before{};
  backend.ReadMotorState(before);
  ASSERT_EQ(before.num_motor_channels, 3);
  ASSERT_DOUBLE_EQ(before.motor_positions[0], 1.0);

  ASSERT_TRUE(backend.CloseStateLane());

  ASSERT_TRUE(h.DeliverToClosedLane(*lane, JointStateAt(10.0)))
      << "second motor state never delivered";
  rtc::DeviceStateCache after{};
  backend.ReadMotorState(after);
  EXPECT_DOUBLE_EQ(after.motor_positions[0], 1.0)
      << "closed motor lane still wrote the cache (#941)";
}

TEST(StateLaneClose, UdpSensorLaneIgnoresMessagesAfterClose) {
  Harness h("udp_sensor");
  const auto cfg = UdpConfig("sensor");
  rtc::UdpHandNativeBackend backend;
  backend.SetSensorLayout(UdpLayout());
  backend.Configure(h.node.get(), cfg, h.group);

  auto lane = h.OpenLane<rtc_msgs::msg::HandSensorState>(cfg.sensor_topic);
  ASSERT_TRUE(h.DeliverUntil(*lane, SensorStateWithForce(1.0F), [&backend] {
    rtc::DeviceStateCache c{};
    backend.ReadSensorState(c);
    return c.inference_sequence[0] == 1U;
  })) << "open sensor lane never decoded the first message";
  rtc::DeviceStateCache before{};
  backend.ReadSensorState(before);
  ASSERT_EQ(before.inference_sequence[0], 1U);
  ASSERT_FLOAT_EQ(before.inference_data[1], 1.0F);  // slot 1 of fingertip 0 = f[0]

  ASSERT_TRUE(backend.CloseStateLane());

  ASSERT_TRUE(h.DeliverToClosedLane(*lane, SensorStateWithForce(10.0F)))
      << "second sensor state never delivered";
  rtc::DeviceStateCache after{};
  backend.ReadSensorState(after);
  EXPECT_EQ(after.inference_sequence[0], 1U) << "closed sensor lane still counted a sample (#941)";
  EXPECT_FLOAT_EQ(after.inference_data[1], 1.0F)
      << "closed sensor lane still wrote the cache (#941)";
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
