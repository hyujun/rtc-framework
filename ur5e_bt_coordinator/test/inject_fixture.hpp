#pragma once
/// DDS-free test fixture for ur5e_bt_coordinator unit tests (issue #154).
///
/// Provides:
///   - BridgeStateInjector: thin friend of BtRosBridge that forwards fully
///     formed messages into the extracted On* subscription handlers — the
///     exact production callback path (same mutexes, parsing, health stamps).
///     It never touches bridge fields directly, so the inject path cannot
///     drift from the subscription path.
///   - InjectTestFixture: bridge node + BtRosBridge + injector only. No mock
///     controllers, no fixture publishers, no background spin thread, no
///     delivery retry loops — state lands synchronously, removing the FastDDS
///     discovery/delivery churn that made the former RosTestFixture-based
///     binaries flaky.
///
/// Helper names/signatures deliberately mirror RosTestFixture
/// (test_helpers.hpp) so test bodies switch tiers by swapping the base class
/// only — keeping assertion parity reviewable (exec-plan D5). Tests whose
/// subject is the real service/rebind path (switch_controller, set_gains,
/// grasp_control, service_singlethread) stay on RosTestFixture.

#include "hand_joint_names.hpp"
#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/bt_types.hpp"
#include <rtc_msgs/msg/grasp_state.hpp>
#include <rtc_msgs/msg/to_f_snapshot.hpp>
#include <rtc_msgs/msg/wbc_state.hpp>
#include <shape_estimation_msgs/msg/shape_estimate.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace rtc_bt::test {

/// Test-only state-inject seam (friend of BtRosBridge). Each method wraps its
/// payload in the wire message type and calls the corresponding On* handler
/// synchronously — handler-mediated only, no field pokes (exec-plan D2).
struct BridgeStateInjector {
  explicit BridgeStateInjector(std::shared_ptr<BtRosBridge> bridge_in)
      : bridge(std::move(bridge_in)) {}

  void ArmJointState(sensor_msgs::msg::JointState msg) const {
    bridge->OnArmJointState(std::make_shared<sensor_msgs::msg::JointState>(std::move(msg)));
  }

  void HandJointState(sensor_msgs::msg::JointState msg) const {
    bridge->OnHandJointState(std::make_shared<sensor_msgs::msg::JointState>(std::move(msg)));
  }

  void WorldTarget(geometry_msgs::msg::Polygon msg) const {
    bridge->OnWorldTarget(std::make_shared<geometry_msgs::msg::Polygon>(std::move(msg)));
  }

  /// Runs the full production transition: RewireControllerTopics(name) first
  /// (creating the arm/hand target pubs + param/grasp clients), then the
  /// active_controller_ update — same ordering guarantee as the real callback.
  void ActiveController(const std::string& name) const {
    std_msgs::msg::String msg;
    msg.data = name;
    bridge->OnActiveController(std::make_shared<std_msgs::msg::String>(std::move(msg)));
  }

  void Estop(bool active) const {
    std_msgs::msg::Bool msg;
    msg.data = active;
    bridge->OnEstop(std::make_shared<std_msgs::msg::Bool>(msg));
  }

  void ShapeEstimate(const shape_estimation_msgs::msg::ShapeEstimate& msg) const {
    bridge->OnShapeEstimate(std::make_shared<shape_estimation_msgs::msg::ShapeEstimate>(msg));
  }

  void GraspState(rtc_msgs::msg::GraspState msg) const {
    bridge->OnGraspState(std::make_shared<rtc_msgs::msg::GraspState>(std::move(msg)));
  }

  void WbcState(rtc_msgs::msg::WbcState msg) const {
    bridge->OnWbcState(std::make_shared<rtc_msgs::msg::WbcState>(std::move(msg)));
  }

  void Transforms(tf2_msgs::msg::TFMessage msg) const {
    bridge->OnTransforms(std::make_shared<tf2_msgs::msg::TFMessage>(std::move(msg)));
  }

  void ToFSnapshot(const rtc_msgs::msg::ToFSnapshot& msg) const {
    bridge->OnToFSnapshot(std::make_shared<rtc_msgs::msg::ToFSnapshot>(msg));
  }

  std::shared_ptr<BtRosBridge> bridge;
};

/// One rclcpp context per binary, formalized as a gtest global Environment
/// (init before the first test, shutdown after the last) instead of the
/// leave-running-until-exit pattern the DDS tier uses.
class RosContextEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

// static (not inline): each inject binary is a single TU including this header,
// and a namespace-scope non-inline variable's dynamic init is guaranteed to run
// before main's first odr-use of the TU ([basic.start.dynamic]/4) — an inline
// variable that is never odr-used forfeits that guarantee (/5).
static const ::testing::Environment* const kRosContextEnv =
    ::testing::AddGlobalTestEnvironment(new RosContextEnvironment);

/// DDS-free fixture: bridge node + BtRosBridge + BridgeStateInjector. State
/// helpers are synchronous handler calls, so there is no delivery to wait for —
/// Spin() only elapses the requested wall-clock time.
class InjectTestFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "bt_test_node", rclcpp::NodeOptions().use_intra_process_comms(true));
    bridge_ = std::make_shared<BtRosBridge>(node_);
    injector_ = std::make_unique<BridgeStateInjector>(bridge_);

    // Default active = demo_task_controller (parity with RosTestFixture).
    // This must really run RewireControllerTopics: PublishArmTarget /
    // PublishHandTarget dereference the rewire-created target pubs without a
    // null check, and move_to_* / hand nodes publish targets mid-tick
    // (exec-plan D3). Publisher/client creation is synchronous and
    // discovery-free; publishing with no subscriber is a no-op.
    SetActiveAlias("demo_task_controller");
  }

  void TearDown() override {
    injector_.reset();
    bridge_.reset();
    node_.reset();
  }

  /// Inject the active-controller transition through the production callback.
  void SetActiveAlias(const std::string& name) { injector_->ActiveController(name); }

  /// Injection is synchronous, so there is no delivery to wait for — but the
  /// duration still elapses on the wall clock (bare Spin() defaults to 0ms).
  /// Keeps cross-tier semantics: a test that uses Spin(dur) to age health
  /// stamps or advance duration-based nodes behaves identically on the e2e
  /// tier, whose Spin really sleeps.
  void Spin(std::chrono::milliseconds duration = std::chrono::milliseconds(0)) {
    if (duration.count() > 0) {
      std::this_thread::sleep_for(duration);
    }
  }

  // ── State injection helpers (signatures mirror RosTestFixture) ──────────

  void PublishArmState(const Pose6D& tcp, const std::vector<double>& joints) {
    sensor_msgs::msg::JointState js;
    js.position.assign(joints.begin(), joints.end());
    injector_->ArmJointState(std::move(js));

    // TCP pose travels the production route too: a `base → tool0_actual`
    // transform through OnTransforms → tf_buffer_, read back via GetTcpPose's
    // quaternion↔RPY round-trip.
    geometry_msgs::msg::TransformStamped tfs;
    tfs.header.stamp = node_->now();
    tfs.header.frame_id = "base";
    tfs.child_frame_id = "tool0_actual";
    tfs.transform.translation.x = tcp.x;
    tfs.transform.translation.y = tcp.y;
    tfs.transform.translation.z = tcp.z;
    // RPY → quaternion (ZYX, matches GetTcpPose's inverse mapping).
    const double cr = std::cos(tcp.roll * 0.5);
    const double sr = std::sin(tcp.roll * 0.5);
    const double cp = std::cos(tcp.pitch * 0.5);
    const double sp = std::sin(tcp.pitch * 0.5);
    const double cy = std::cos(tcp.yaw * 0.5);
    const double sy = std::sin(tcp.yaw * 0.5);
    tfs.transform.rotation.w = cr * cp * cy + sr * sp * sy;
    tfs.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    tfs.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    tfs.transform.rotation.z = cr * cp * sy - sr * sp * cy;
    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(tfs);
    injector_->Transforms(std::move(tf_msg));
  }

  void PublishHandState(const std::vector<double>& joints) {
    // Hand nodes map finger→joint via joint_states name prefixes
    // (FingerJointIndices), so name must be populated, not just position.
    const auto& names = AssmV1JointNames();
    sensor_msgs::msg::JointState js;
    js.position.assign(joints.begin(), joints.end());
    const std::size_t n = std::min(joints.size(), names.size());
    js.name.assign(names.begin(), names.begin() + static_cast<long>(n));
    injector_->HandJointState(std::move(js));
  }

  void PublishGraspState(const CachedGraspState& gs) {
    rtc_msgs::msg::GraspState msg;
    msg.num_active_contacts = gs.num_active_contacts;
    msg.max_force = gs.max_force;
    msg.grasp_detected = gs.grasp_detected;
    msg.force_threshold = gs.force_threshold;
    msg.min_fingertips = gs.min_fingertips;
    msg.grasp_phase = gs.grasp_phase;
    msg.grasp_target_force = gs.grasp_target_force;
    for (const auto& ft : gs.fingertips) {
      msg.fingertip_names.push_back(ft.name);
      msg.force_magnitude.push_back(ft.force_magnitude);
      msg.contact_flag.push_back(ft.contact_flag);
      msg.inference_valid.push_back(ft.inference_valid);
    }
    msg.finger_s = gs.finger_s;
    msg.finger_filtered_force = gs.finger_filtered_force;
    msg.finger_force_error = gs.finger_force_error;
    injector_->GraspState(std::move(msg));
  }

  void PublishWorldTarget(double x, double y, double z) {
    geometry_msgs::msg::Polygon msg;
    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(x);
    pt.y = static_cast<float>(y);
    pt.z = static_cast<float>(z);
    msg.points.push_back(pt);
    injector_->WorldTarget(std::move(msg));
  }

  void PublishEstop(bool active) { injector_->Estop(active); }

  void PublishShapeEstimate(uint8_t shape_type, double confidence, uint32_t num_points = 100) {
    shape_estimation_msgs::msg::ShapeEstimate msg;
    msg.shape_type = shape_type;
    msg.confidence = confidence;
    msg.num_points_used = num_points;
    injector_->ShapeEstimate(msg);
  }

  // ── Data members ────────────────────────────────────────────────────────

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<BtRosBridge> bridge_;
  std::unique_ptr<BridgeStateInjector> injector_;
};

}  // namespace rtc_bt::test
