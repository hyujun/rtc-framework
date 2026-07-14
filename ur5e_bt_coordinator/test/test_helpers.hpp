#pragma once
/// Test helpers for ur5e_bt_coordinator unit tests.
///
/// Provides:
///   - RosTestFixture: RAII rclcpp::init/shutdown with a test ROS2 node +
///                     BtRosBridge + three mock controller LifecycleNodes
///                     (one per demo controller name) hosting the parameter
///                     API + grasp_command srv. Tests publish the desired
///                     active controller name and the bridge rebinds its
///                     AsyncParametersClient + grasp_command_client_ to the
///                     matching mock automatically.

#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/bt_types.hpp"
#include <rtc_msgs/msg/grasp_state.hpp>
#include <rtc_msgs/srv/grasp_command.hpp>
#include <shape_estimation_msgs/msg/shape_estimate.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace rtc_bt::test {

/// Default poll budget — matches SetGains' kSrvTimeoutS so a rebind wait cannot
/// expire before the tick's own service wait would.
inline constexpr std::chrono::milliseconds kDefaultWaitTimeout{2000};

/// Delivery-match tolerances for Publish* blocking waits (exact-ish: values are
/// round-tripped through message fields, not computed).
inline constexpr double kJointMatchTol = 1e-6;  // rad
inline constexpr double kPoseMatchTol = 1e-4;   // m

/// Poll `condition` until it holds or `timeout` expires; returns whether it
/// held. Deliberately does NOT call rclcpp::spin_some — the fixture's dedicated
/// background thread is the sole spinner (see RosTestFixture::spin_thread_).
/// Spinning here too would drive the same executor concurrently, which rclcpp
/// does not permit; the removed SpinUntil made exactly that mistake. Callers
/// observe delivery through the bridge's thread-safe cached-state getters, which
/// the background spin populates.
inline bool WaitUntil(const std::function<bool()>& condition,
                      std::chrono::milliseconds timeout = kDefaultWaitTimeout) {
  const auto start = std::chrono::steady_clock::now();
  while (!condition()) {
    if (std::chrono::steady_clock::now() - start > timeout)
      return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

/// Publish via `publish`, then wait (polling `observed`) for the bridge to cache
/// it, re-publishing only if a whole poll window passes without delivery.
/// Returns whether it was observed before the timeout.
///
/// The fixture's state publishers are volatile (non-transient_local), so a lone
/// sample sent before the bridge's freshly-created subscription has matched is
/// dropped with no redelivery — and the bridge rebinds those subs late, inside
/// the active-controller callback. The first publish therefore races discovery
/// and can be silently lost (the flake the old fixed-sleep masked as a stale
/// read); a bounded re-publish covers that. Re-publishing is deliberately GENTLE
/// (at most once per poll window, not a tight loop): flooding the topic backs up
/// rmw and induces its own drops, which manifested as later publishes in a rapid
/// sequence (IsGraspPhase_AllPhases) never landing at all. Idempotent: the state
/// topics are latest-value, so a repeat is a no-op once the sample has settled.
inline bool PublishUntilObserved(const std::function<void()>& publish,
                                 const std::function<bool()>& observed,
                                 std::chrono::milliseconds timeout = kDefaultWaitTimeout) {
  constexpr std::chrono::milliseconds kRepublishWindow{100};
  const auto start = std::chrono::steady_clock::now();
  for (;;) {
    publish();
    if (WaitUntil(observed, kRepublishWindow))
      return true;
    if (std::chrono::steady_clock::now() - start >= timeout)
      return false;
  }
}

/// Mock per-controller LifecycleNode hosting a representative gain parameter
/// set + grasp_command srv. Mirrors the real demo controllers' surface so
/// the bridge's AsyncParametersClient + grasp_command_client_ can round-trip
/// against it.
struct MockController {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Service<rtc_msgs::srv::GraspCommand>::SharedPtr grasp_srv;
  uint8_t last_grasp_cmd{0};
  double last_grasp_force{0.0};
  std::atomic<int> grasp_cmd_calls{0};
};

/// RAII test fixture that manages rclcpp lifecycle and provides:
///   - bridge node    (`node_`)               — hosts BtRosBridge
///   - mock controllers (`mock_joint_`, `mock_task_`, `mock_wbc_`) at
///     /<name>/<name>, each with its name's parameter subset declared and
///     a grasp_command srv server attached.
///   - background spin drives all four nodes so bridge clients' futures
///     resolve during BT ticks.
class RosTestFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    // Initialize the ROS context ONCE for the whole test binary, not per test.
    // Each test still creates/destroys its own nodes, but tearing the context
    // (DDS participant) up and down ~20 times per binary churns FastDDS enough
    // that discovery/delivery latency spikes past the wait budget in later tests.
    // Left running until the process exits (standard rclcpp test pattern).
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "bt_test_node", rclcpp::NodeOptions().use_intra_process_comms(true));
    bridge_ = std::make_shared<BtRosBridge>(node_);

    SpawnMock("demo_joint_controller", mock_joint_);
    SpawnMock("demo_task_controller", mock_task_);
    SpawnMock("demo_wbc_controller", mock_wbc_);

    // State injection publishers — pick demo_task_controller as the default
    // active for state-injection tests; SetActiveAlias() can override later.
    const std::string ctrl_ns = "/demo_task_controller";
    // Phase 4: arm/hand state는 controller-agnostic /rtc_cm/<group>/joint_states
    // 로 이동. TCP pose는 tf2 broadcaster (`base → tool0_actual`) 로 주입.
    arm_joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/rtc_cm/ur5e/joint_states", rclcpp::QoS{10});
    hand_joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/rtc_cm/hand/joint_states", rclcpp::QoS{10});
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    grasp_state_pub_ = node_->create_publisher<rtc_msgs::msg::GraspState>(
        ctrl_ns + "/hand/grasp_state", rclcpp::QoS{10});
    world_target_pub_ =
        node_->create_publisher<geometry_msgs::msg::Polygon>("/world_target_info", rclcpp::QoS{10});
    active_ctrl_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/rtc_cm/active_controller_name", rclcpp::QoS{1}.transient_local());
    estop_pub_ =
        node_->create_publisher<std_msgs::msg::Bool>("/system/estop_status", rclcpp::QoS{10});
    shape_estimate_pub_ = node_->create_publisher<shape_estimation_msgs::msg::ShapeEstimate>(
        "/shape/estimate", rclcpp::QoS{10});

    // Background spin drives bridge + all mock nodes.
    spin_running_.store(true);
    spin_thread_ = std::thread([this]() {
      while (spin_running_.load()) {
        rclcpp::spin_some(node_->get_node_base_interface());
        rclcpp::spin_some(mock_joint_.node->get_node_base_interface());
        rclcpp::spin_some(mock_task_.node->get_node_base_interface());
        rclcpp::spin_some(mock_wbc_.node->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });

    // Default active = demo_task_controller (covers most parameter cases).
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    SetActiveAlias("demo_task_controller");
  }

  void TearDown() override {
    spin_running_.store(false);
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    mock_joint_.grasp_srv.reset();
    mock_task_.grasp_srv.reset();
    mock_wbc_.grasp_srv.reset();
    bridge_.reset();
    mock_joint_.node.reset();
    mock_task_.node.reset();
    mock_wbc_.node.reset();
    node_.reset();
    // Intentionally do NOT rclcpp::shutdown() here — the context is shared
    // across the binary's tests (see SetUp). Repeated shutdown/init churns DDS.
  }

  /// Publish the active controller name and block (via the now-synchronous
  /// PublishActiveController) until the bridge has processed the transition —
  /// GetActiveController reflects `name`, set by the /rtc_cm/active_controller_name
  /// callback immediately before it rebinds the param/grasp clients through
  /// RewireControllerTopics. Replaces a fixed 150ms sleep that raced delivery —
  /// the flake source behind the set_gains failures (rebind not yet applied →
  /// tick hit the previous controller's clients).
  void SetActiveAlias(const std::string& name) { PublishActiveController(name); }

  /// Sleep wrapper preserved for legacy test bodies. Delivery is now guaranteed
  /// by the blocking Publish* helpers, so this is a residual settle, not a
  /// synchronization point.
  void Spin(std::chrono::milliseconds duration = std::chrono::milliseconds(50)) {
    std::this_thread::sleep_for(duration);
  }

  // ── State injection helpers ─────────────────────────────────────────────

  void PublishArmState(const Pose6D& tcp, const std::vector<double>& joints) {
    // Phase 4: TCP pose via tf2, joint via sensor_msgs/JointState. Retry until
    // the bridge caches both the joint sample (empty→populated is an unambiguous
    // delivery edge, so this holds even for all-zero joints) and the tf pose
    // (translation carries the delivery signal for pose tests; rotation arrives
    // atomically in the same tf message). Zero-valued targets may match the
    // pre-delivery default early, but every such test's assertion is
    // delivery-independent (still-far / not-yet-converged → RUNNING).
    PublishUntilObserved(
        [this, &tcp, &joints]() {
          sensor_msgs::msg::JointState js;
          js.position.assign(joints.begin(), joints.end());
          arm_joint_pub_->publish(js);

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
          tf_broadcaster_->sendTransform(tfs);
        },
        [this, &tcp, &joints]() {
          const auto cur = bridge_->GetArmJointPositions();
          if (cur.size() != joints.size())
            return false;
          for (std::size_t i = 0; i < joints.size(); ++i)
            if (std::fabs(cur[i] - joints[i]) > kJointMatchTol)
              return false;
          const Pose6D p = bridge_->GetTcpPose();
          return std::fabs(p.x - tcp.x) < kPoseMatchTol && std::fabs(p.y - tcp.y) < kPoseMatchTol &&
                 std::fabs(p.z - tcp.z) < kPoseMatchTol;
        });
  }

  void PublishHandState(const std::vector<double>& joints) {
    // Hand nodes map finger→joint via joint_states name prefixes
    // (FingerJointIndices), so name must be populated, not just position.
    // assm_v1 10-DoF order (thumb:3 / index:3 / middle:3 / ring:1).
    static const std::vector<std::string> assm_v1_joint_names = {
        "thumb_cmc_aa", "thumb_cmc_fe",  "thumb_mcp_fe",  "index_mcp_aa",  "index_mcp_fe",
        "index_dip_fe", "middle_mcp_aa", "middle_mcp_fe", "middle_dip_fe", "ring_mcp_fe"};
    // Retry until the bridge caches the hand sample (empty→populated edge).
    PublishUntilObserved(
        [this, &joints]() {
          sensor_msgs::msg::JointState js;
          js.position.assign(joints.begin(), joints.end());
          const std::size_t n = std::min(joints.size(), assm_v1_joint_names.size());
          js.name.assign(assm_v1_joint_names.begin(),
                         assm_v1_joint_names.begin() + static_cast<long>(n));
          hand_joint_pub_->publish(js);
        },
        [this, &joints]() {
          const auto cur = bridge_->GetHandJointPositions();
          if (cur.size() != joints.size())
            return false;
          for (std::size_t i = 0; i < joints.size(); ++i)
            if (std::fabs(cur[i] - joints[i]) > kJointMatchTol)
              return false;
          return true;
        });
  }

  void PublishGraspState(const CachedGraspState& gs) {
    // Retry until the bridge caches this sample, matched on the scalar fields the
    // condition-node tests key on. An all-default grasp state is
    // indistinguishable from the pre-delivery default and returns immediately,
    // but those tests assert a negative (not-grasped / idle) that the default
    // also satisfies, so no assertion is weakened.
    PublishUntilObserved(
        [this, &gs]() {
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
          grasp_state_pub_->publish(msg);
        },
        [this, &gs]() {
          const auto cur = bridge_->GetGraspState();
          return cur.grasp_phase == gs.grasp_phase &&
                 cur.num_active_contacts == gs.num_active_contacts &&
                 cur.grasp_detected == gs.grasp_detected &&
                 cur.min_fingertips == gs.min_fingertips &&
                 cur.fingertips.size() == gs.fingertips.size() &&
                 std::fabs(cur.max_force - gs.max_force) < kJointMatchTol &&
                 std::fabs(cur.force_threshold - gs.force_threshold) < kJointMatchTol;
        });
  }

  void PublishWorldTarget(double x, double y, double z) {
    // GetWorldTargetPose's valid flag is a delivery signal independent of value.
    PublishUntilObserved(
        [this, x, y, z]() {
          geometry_msgs::msg::Polygon msg;
          geometry_msgs::msg::Point32 pt;
          pt.x = static_cast<float>(x);
          pt.y = static_cast<float>(y);
          pt.z = static_cast<float>(z);
          msg.points.push_back(pt);
          world_target_pub_->publish(msg);
        },
        [this, x, y, z]() {
          Pose6D p;
          return bridge_->GetWorldTargetPose(p) && std::fabs(p.x - x) < kPoseMatchTol &&
                 std::fabs(p.y - y) < kPoseMatchTol && std::fabs(p.z - z) < kPoseMatchTol;
        });
  }

  void PublishActiveController(const std::string& name) {
    // active_ctrl_pub_ is transient_local, so a late-matching bridge sub still
    // latches the last name — but retry anyway for symmetry and to also cover
    // the rebind completing (GetActiveController is set only after the bridge's
    // RewireControllerTopics finishes; see bt_ros_bridge active_ctrl_sub_).
    PublishUntilObserved(
        [this, &name]() {
          std_msgs::msg::String msg;
          msg.data = name;
          active_ctrl_pub_->publish(msg);
        },
        [this, &name]() { return bridge_->GetActiveController() == name; });
  }

  void PublishEstop(bool active) {
    PublishUntilObserved(
        [this, active]() {
          std_msgs::msg::Bool msg;
          msg.data = active;
          estop_pub_->publish(msg);
        },
        [this, active]() { return bridge_->IsEstopped() == active; });
  }

  void PublishShapeEstimate(uint8_t shape_type, double confidence, uint32_t num_points = 100) {
    // GetShapeEstimate's valid flag is a delivery signal independent of value.
    PublishUntilObserved(
        [this, shape_type, confidence, num_points]() {
          shape_estimation_msgs::msg::ShapeEstimate msg;
          msg.shape_type = shape_type;
          msg.confidence = confidence;
          msg.num_points_used = num_points;
          shape_estimate_pub_->publish(msg);
        },
        [this, shape_type, num_points]() {
          shape_estimation_msgs::msg::ShapeEstimate out;
          return bridge_->GetShapeEstimate(out) && out.shape_type == shape_type &&
                 out.num_points_used == num_points;
        });
  }

  // ── Data members ────────────────────────────────────────────────────────

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<BtRosBridge> bridge_;

  MockController mock_joint_;
  MockController mock_task_;
  MockController mock_wbc_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hand_joint_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<rtc_msgs::msg::GraspState>::SharedPtr grasp_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr world_target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_ctrl_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<shape_estimation_msgs::msg::ShapeEstimate>::SharedPtr shape_estimate_pub_;

 private:
  void SpawnMock(const std::string& name, MockController& out) {
    const std::string ns = "/" + name;
    const auto opts = rclcpp::NodeOptions().use_global_arguments(false);
    out.node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, ns, opts);

    // Declare the union of demo controllers' parameter names. Tests only
    // touch the ones relevant to the alias they activate; the rest are
    // declared but unused.
    out.node->declare_parameter<double>("trajectory_speed", 0.1);
    out.node->declare_parameter<double>("trajectory_angular_speed", 0.5);
    out.node->declare_parameter<double>("hand_trajectory_speed", 1.0);
    out.node->declare_parameter<double>("robot_trajectory_speed", 0.5);
    out.node->declare_parameter<double>("arm_trajectory_speed", 0.5);
    out.node->declare_parameter<std::vector<double>>("kp_translation",
                                                     std::vector<double>{5.0, 5.0, 5.0});
    out.node->declare_parameter<std::vector<double>>("kp_rotation",
                                                     std::vector<double>{2.0, 2.0, 2.0});
    out.node->declare_parameter<double>("damping", 0.01);
    out.node->declare_parameter<double>("null_kp", 0.5);
    out.node->declare_parameter<bool>("enable_null_space", false);
    out.node->declare_parameter<bool>("control_6dof", true);
    out.node->declare_parameter<double>("grasp_contact_threshold", 0.5);
    out.node->declare_parameter<double>("grasp_force_threshold", 1.0);
    out.node->declare_parameter<int64_t>("grasp_min_fingertips", 2);
    out.node->declare_parameter<double>("se3_weight", 100.0);
    out.node->declare_parameter<double>("force_weight", 10.0);
    out.node->declare_parameter<double>("posture_weight", 1.0);
    out.node->declare_parameter<bool>("mpc_enable", false);
    out.node->declare_parameter<double>("riccati_gain_scale", 1.0);

    out.grasp_srv = out.node->create_service<rtc_msgs::srv::GraspCommand>(
        "grasp_command", [&out](const std::shared_ptr<rtc_msgs::srv::GraspCommand::Request> req,
                                std::shared_ptr<rtc_msgs::srv::GraspCommand::Response> resp) {
          out.last_grasp_cmd = req->command;
          out.last_grasp_force = req->target_force;
          out.grasp_cmd_calls.fetch_add(1);
          resp->ok = true;
          resp->message = "mock ok";
        });
  }

  std::atomic<bool> spin_running_{false};
  std::thread spin_thread_;
};

}  // namespace rtc_bt::test
