#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"

#include <cmath>

namespace rtc_bt {

namespace {
auto bridge_log() { return ::rtc_bt::logging::BridgeLogger(); }
auto poses_log() { return ::rtc_bt::logging::PosesLogger(); }
} // namespace

BtRosBridge::BtRosBridge(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : node_(std::move(node)) {
  // Initialize pose maps from compile-time defaults
  hand_poses_ = kHandPoses;
  arm_poses_ = kUR5ePoses;

  // ── Subscribers (all RELIABLE QoS) ──────────────────────────────────────
  //
  // Phase 4: controller-owned topics (arm_gui / hand_gui / grasp_state /
  // tof_snapshot / arm_target / hand_target) live under
  // /<active_controller_name>/... and are rebound on every
  // /ur5e/active_controller_name transition via RewireControllerTopics.
  // Manager-owned topics stay at their fixed paths below.

  world_target_sub_ = node_->create_subscription<geometry_msgs::msg::Polygon>(
      "/world_target_info", rclcpp::QoS{10},
      [this](geometry_msgs::msg::Polygon::SharedPtr msg) {
        if (msg->points.empty())
          return;

        // Check if all coordinates are zero (data not ready)
        bool all_zero = true;
        for (const auto &pt : msg->points) {
          if (pt.x != 0.0f || pt.y != 0.0f || pt.z != 0.0f) {
            all_zero = false;
            break;
          }
        }

        {
          std::lock_guard lock(state_mutex_);
          if (all_zero) {
            world_target_valid_ = false;
          } else {
            // points[0] = position (x, y, z) only
            world_target_pose_.x = static_cast<double>(msg->points[0].x);
            world_target_pose_.y = static_cast<double>(msg->points[0].y);
            world_target_pose_.z = static_cast<double>(msg->points[0].z);
            world_target_pose_.roll = 0.0;
            world_target_pose_.pitch = 0.0;
            world_target_pose_.yaw = 0.0;
            world_target_valid_ = true;
          }
        }
        {
          std::lock_guard lock(health_mutex_);
          world_target_last_ = std::chrono::steady_clock::now();
          world_target_received_ = true;
        }
      });

  active_ctrl_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/ur5e/active_controller_name", rclcpp::QoS{1}.transient_local(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        {
          std::lock_guard lock(state_mutex_);
          active_controller_ = msg->data;
        }
        RewireControllerTopics(msg->data);
      });

  estop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/system/estop_status", rclcpp::QoS{10},
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        {
          std::lock_guard lock(state_mutex_);
          estopped_ = msg->data;
        }
        {
          std::lock_guard lock(health_mutex_);
          estop_last_ = std::chrono::steady_clock::now();
          estop_received_ = true;
        }
      });

  current_gains_sub_ =
      node_->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/ur5e/current_gains", rclcpp::QoS{10},
          [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            std::lock_guard lock(state_mutex_);
            cached_gains_ = msg->data;
            cached_gains_valid_ = true;
            RCLCPP_DEBUG(bridge_log(), "received current_gains (%zu values)",
                         msg->data.size());
          });

  // ── Publishers (manager-owned: fixed paths) ─────────────────────────────

  gains_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ur5e/controller_gains", rclcpp::QoS{10});

  select_ctrl_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/ur5e/controller_type", rclcpp::QoS{10});

  request_gains_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/ur5e/request_gains", rclcpp::QoS{10});

  // ── Shape estimation ──────────────────────────────────────────────────

  shape_estimate_sub_ =
      node_->create_subscription<shape_estimation_msgs::msg::ShapeEstimate>(
          "/shape/estimate", rclcpp::QoS{10},
          [this](shape_estimation_msgs::msg::ShapeEstimate::SharedPtr msg) {
            std::lock_guard lock(state_mutex_);
            shape_estimate_ = *msg;
            shape_estimate_valid_ = true;
          });

  shape_trigger_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/shape/trigger", rclcpp::QoS{10});

  shape_clear_client_ =
      node_->create_client<std_srvs::srv::Trigger>("/shape/clear");

  // ── /rtc_cm/* service clients (Phase 4) ───────────────────────────────
  switch_controller_client_ =
      node_->create_client<rtc_msgs::srv::SwitchController>(
          "/rtc_cm/switch_controller");
  list_controllers_client_ =
      node_->create_client<rtc_msgs::srv::ListControllers>(
          "/rtc_cm/list_controllers");

  RCLCPP_INFO(bridge_log(), "initialized");
}

// ── Cached state accessors ────────────────────────────────────────────────

Pose6D BtRosBridge::GetTcpPose() const {
  std::lock_guard lock(state_mutex_);
  return tcp_pose_;
}

std::vector<double> BtRosBridge::GetArmJointPositions() const {
  std::lock_guard lock(state_mutex_);
  return arm_joint_positions_;
}

std::vector<double> BtRosBridge::GetHandJointPositions() const {
  std::lock_guard lock(state_mutex_);
  return hand_joint_positions_;
}

CachedGraspState BtRosBridge::GetGraspState() const {
  std::lock_guard lock(state_mutex_);
  return grasp_state_;
}

bool BtRosBridge::GetObjectPose(Pose6D &pose) const {
  return GetWorldTargetPose(pose);
}

bool BtRosBridge::GetWorldTargetPose(Pose6D &pose) const {
  std::lock_guard lock(state_mutex_);
  if (!world_target_valid_)
    return false;
  pose = world_target_pose_;
  return true;
}

std::string BtRosBridge::GetActiveController() const {
  std::lock_guard lock(state_mutex_);
  return active_controller_;
}

bool BtRosBridge::IsEstopped() const {
  std::lock_guard lock(state_mutex_);
  return estopped_;
}

// ── Gains query ──────────────────────────────────────────────────────────

void BtRosBridge::RequestCurrentGains() {
  std_msgs::msg::Bool msg;
  msg.data = true;
  request_gains_pub_->publish(msg);
  RCLCPP_DEBUG(bridge_log(), "request_current_gains published");
}

std::vector<double> BtRosBridge::GetCachedGains() const {
  std::lock_guard lock(state_mutex_);
  return cached_gains_;
}

bool BtRosBridge::HasCachedGains() const {
  std::lock_guard lock(state_mutex_);
  return cached_gains_valid_;
}

void BtRosBridge::ClearCachedGains() {
  std::lock_guard lock(state_mutex_);
  cached_gains_.clear();
  cached_gains_valid_ = false;
}

// ── Publishers ────────────────────────────────────────────────────────────

void BtRosBridge::PublishArmTarget(const Pose6D &target) {
  rtc_msgs::msg::RobotTarget msg;
  msg.header.stamp = node_->now();
  msg.goal_type = "task";
  msg.task_target = {target.x,    target.y,     target.z,
                     target.roll, target.pitch, target.yaw};
  arm_target_pub_->publish(msg);
}

void BtRosBridge::PublishArmJointTarget(const std::vector<double> &target) {
  rtc_msgs::msg::RobotTarget msg;
  msg.header.stamp = node_->now();
  msg.goal_type = "joint";
  msg.joint_target.assign(target.begin(), target.end());
  arm_target_pub_->publish(msg);
}

void BtRosBridge::PublishHandTarget(const std::vector<double> &target) {
  {
    std::lock_guard lock(state_mutex_);
    last_hand_target_ = target;
  }
  rtc_msgs::msg::RobotTarget msg;
  msg.header.stamp = node_->now();
  msg.goal_type = "joint";
  msg.joint_target.assign(target.begin(), target.end());
  hand_target_pub_->publish(msg);
}

std::vector<double> BtRosBridge::GetLastHandTarget() const {
  std::lock_guard lock(state_mutex_);
  return last_hand_target_;
}

void BtRosBridge::PublishGains(const std::vector<double> &gains) {
  std_msgs::msg::Float64MultiArray msg;
  msg.data = gains;
  gains_pub_->publish(msg);
}

void BtRosBridge::PublishSelectController(const std::string &name) {
  std_msgs::msg::String msg;
  msg.data = name;
  RCLCPP_DEBUG(bridge_log(), "select_controller: %s", name.c_str());
  select_ctrl_pub_->publish(msg);
}

bool BtRosBridge::RequestSwitchController(const std::string &name,
                                          double timeout_s,
                                          std::string &message) {
  if (!switch_controller_client_->service_is_ready()) {
    // Single 200ms grace period so the client survives a brief CM start race.
    if (!switch_controller_client_->wait_for_service(
            std::chrono::milliseconds(200))) {
      message = "switch_controller service unavailable";
      return false;
    }
  }
  auto req = std::make_shared<rtc_msgs::srv::SwitchController::Request>();
  req->activate_controllers = {name};
  req->strictness = rtc_msgs::srv::SwitchController::Request::STRICT;
  // Mirror caller's timeout into the service field (server currently treats
  // the field as informational — D-A4 sync helper bounds latency to ~ms).
  const auto t_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_s));
  req->timeout.sec = static_cast<int32_t>(t_ns.count() / 1'000'000'000LL);
  req->timeout.nanosec = static_cast<uint32_t>(t_ns.count() % 1'000'000'000LL);

  auto fut = switch_controller_client_->async_send_request(req);
  const auto wait_ms = static_cast<int64_t>(timeout_s * 1000.0);
  if (fut.wait_for(std::chrono::milliseconds(wait_ms)) !=
      std::future_status::ready) {
    message = "switch_controller timeout (" + std::to_string(timeout_s) + "s)";
    return false;
  }
  auto resp = fut.get();
  message = resp->message;
  return resp->ok;
}

// ── Shape estimation ──────────────────────────────────────────────────────

void BtRosBridge::PublishShapeTrigger(const std::string &command) {
  std_msgs::msg::String msg;
  msg.data = command;
  RCLCPP_INFO(bridge_log(), "shape_trigger: %s", command.c_str());
  shape_trigger_pub_->publish(msg);
}

void BtRosBridge::CallShapeClear() {
  if (!shape_clear_client_->service_is_ready()) {
    RCLCPP_WARN(bridge_log(), "/shape/clear service not available");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  shape_clear_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto result = future.get();
        RCLCPP_INFO(bridge_log(), "/shape/clear: %s",
                    result->success ? "OK" : result->message.c_str());
      });
}

bool BtRosBridge::GetShapeEstimate(
    shape_estimation_msgs::msg::ShapeEstimate &out) const {
  std::lock_guard lock(state_mutex_);
  if (!shape_estimate_valid_)
    return false;
  out = shape_estimate_;
  return true;
}

void BtRosBridge::ClearShapeEstimate() {
  std::lock_guard lock(state_mutex_);
  shape_estimate_ = shape_estimation_msgs::msg::ShapeEstimate{};
  shape_estimate_valid_ = false;
}

// ── ToF data collection ──────────────────────────────────────────────────

void BtRosBridge::StartToFCollection() {
  std::lock_guard lock(tof_mutex_);
  tof_buffer_.clear();
  tof_buffer_.reserve(8192);
  tof_collecting_.store(true, std::memory_order_relaxed);
  RCLCPP_INFO(bridge_log(), "ToF collection started (buffer cleared)");
}

void BtRosBridge::StopToFCollection() {
  tof_collecting_.store(false, std::memory_order_relaxed);
  std::lock_guard lock(tof_mutex_);
  RCLCPP_INFO(bridge_log(), "ToF collection stopped (%zu snapshots)",
              tof_buffer_.size());
}

const std::vector<rtc_msgs::msg::ToFSnapshot> &
BtRosBridge::GetCollectedToFData() const {
  // Caller must ensure collection is stopped before reading.
  return tof_buffer_;
}

std::size_t BtRosBridge::GetCollectedToFCount() const {
  std::lock_guard lock(tof_mutex_);
  return tof_buffer_.size();
}

// ── Pose library ──────────────────────────────────────────────────────────

void BtRosBridge::LoadPoseOverrides(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
  // Discover hand_pose.* parameters
  auto hand_result = node->list_parameters({"hand_pose"}, 1);
  int hand_count = 0;
  for (const auto &param_name : hand_result.names) {
    // param_name = "hand_pose.home" → pose_name = "home"
    const std::string prefix = "hand_pose.";
    if (param_name.size() <= prefix.size())
      continue;
    std::string pose_name = param_name.substr(prefix.size());

    try {
      auto vals = node->get_parameter(param_name).as_double_array();
      if (vals.size() != static_cast<std::size_t>(kHandDofCount)) {
        RCLCPP_WARN(poses_log(),
                    "hand_pose.%s has %zu values (expected %d), skipped",
                    pose_name.c_str(), vals.size(), kHandDofCount);
        continue;
      }
      HandPose pose{};
      for (int i = 0; i < kHandDofCount; ++i) {
        pose[i] = vals[i] * kDeg2Rad;
      }
      hand_poses_[pose_name] = pose;
      ++hand_count;
    } catch (const std::exception &e) {
      RCLCPP_WARN(poses_log(), "failed to load hand_pose.%s: %s",
                  pose_name.c_str(), e.what());
    }
  }

  // Discover arm_pose.* parameters
  auto arm_result = node->list_parameters({"arm_pose"}, 1);
  int arm_count = 0;
  for (const auto &param_name : arm_result.names) {
    const std::string prefix = "arm_pose.";
    if (param_name.size() <= prefix.size())
      continue;
    std::string pose_name = param_name.substr(prefix.size());

    try {
      auto vals = node->get_parameter(param_name).as_double_array();
      if (vals.size() != static_cast<std::size_t>(kArmDofCount)) {
        RCLCPP_WARN(poses_log(),
                    "arm_pose.%s has %zu values (expected %d), skipped",
                    pose_name.c_str(), vals.size(), kArmDofCount);
        continue;
      }
      ArmPose pose{};
      for (int i = 0; i < kArmDofCount; ++i) {
        pose[i] = vals[i] * kDeg2Rad;
      }
      arm_poses_[pose_name] = pose;
      ++arm_count;
    } catch (const std::exception &e) {
      RCLCPP_WARN(poses_log(), "failed to load arm_pose.%s: %s",
                  pose_name.c_str(), e.what());
    }
  }

  RCLCPP_INFO(poses_log(),
              "loaded %d hand poses, %d arm poses (total: %zu hand, %zu arm)",
              hand_count, arm_count, hand_poses_.size(), arm_poses_.size());
}

const HandPose &BtRosBridge::GetHandPose(const std::string &name) const {
  auto it = hand_poses_.find(name);
  if (it == hand_poses_.end()) {
    throw BT::RuntimeError("PoseLibrary: unknown hand pose: " + name);
  }
  return it->second;
}

const ArmPose &BtRosBridge::GetArmPose(const std::string &name) const {
  auto it = arm_poses_.find(name);
  if (it == arm_poses_.end()) {
    throw BT::RuntimeError("PoseLibrary: unknown arm pose: " + name);
  }
  return it->second;
}

// ── Topic health watchdog ─────────────────────────────────────────────────

std::vector<TopicHealth> BtRosBridge::GetTopicHealth(double timeout_s) const {
  std::lock_guard lock(health_mutex_);
  auto now = std::chrono::steady_clock::now();

  auto make_health = [&](const std::string &name, bool received,
                         TimePoint last) {
    TopicHealth h;
    h.name = name;
    h.received = received;
    if (received) {
      h.seconds_since_last = std::chrono::duration<double>(now - last).count();
      h.healthy = (h.seconds_since_last <= timeout_s);
    }
    return h;
  };

  return {
      make_health("/ur5e/gui_position", arm_gui_received_, arm_gui_last_),
      make_health("/hand/gui_position", hand_gui_received_, hand_gui_last_),
      make_health("/hand/grasp_state", grasp_state_received_,
                  grasp_state_last_),
      make_health("/world_target_info", world_target_received_,
                  world_target_last_),
      make_health("/system/estop_status", estop_received_, estop_last_),
  };
}

bool BtRosBridge::AreTopicsHealthy(double timeout_s) const {
  std::lock_guard lock(health_mutex_);
  auto now = std::chrono::steady_clock::now();

  auto is_ok = [&](bool received, TimePoint last) {
    if (!received)
      return false;
    return std::chrono::duration<double>(now - last).count() <= timeout_s;
  };

  // Critical topics: arm + hand position feedback
  return is_ok(arm_gui_received_, arm_gui_last_) &&
         is_ok(hand_gui_received_, hand_gui_last_);
}

// ── Phase 4: dynamic rewiring for controller-owned topics ────────────────
// Invoked from the active_ctrl_sub_ callback. Idempotent — skips when the
// controller name has not changed since the last call.
void BtRosBridge::RewireControllerTopics(const std::string &ctrl_name) {
  if (ctrl_name.empty())
    return;
  {
    std::lock_guard lock(controller_topics_mutex_);
    if (ctrl_name == rewired_controller_)
      return;
    rewired_controller_ = ctrl_name;
  }

  const std::string ns = "/" + ctrl_name;

  // Drop previous sub/pub handles before recreating to avoid two live
  // subscribers holding references to the same state maps.
  arm_gui_sub_.reset();
  hand_gui_sub_.reset();
  grasp_state_sub_.reset();
  tof_snapshot_sub_.reset();
  arm_target_pub_.reset();
  hand_target_pub_.reset();

  arm_gui_sub_ = node_->create_subscription<rtc_msgs::msg::GuiPosition>(
      ns + "/ur5e/gui_position", rclcpp::QoS{10},
      [this](rtc_msgs::msg::GuiPosition::SharedPtr msg) {
        {
          std::lock_guard lock(state_mutex_);
          tcp_pose_.x = msg->task_positions[0];
          tcp_pose_.y = msg->task_positions[1];
          tcp_pose_.z = msg->task_positions[2];
          tcp_pose_.roll = msg->task_positions[3];
          tcp_pose_.pitch = msg->task_positions[4];
          tcp_pose_.yaw = msg->task_positions[5];
          arm_joint_positions_.assign(msg->joint_positions.begin(),
                                      msg->joint_positions.end());
        }
        {
          std::lock_guard lock(health_mutex_);
          arm_gui_last_ = std::chrono::steady_clock::now();
          arm_gui_received_ = true;
        }
      });

  hand_gui_sub_ = node_->create_subscription<rtc_msgs::msg::GuiPosition>(
      ns + "/hand/gui_position", rclcpp::QoS{10},
      [this](rtc_msgs::msg::GuiPosition::SharedPtr msg) {
        {
          std::lock_guard lock(state_mutex_);
          hand_joint_positions_.assign(msg->joint_positions.begin(),
                                       msg->joint_positions.end());
        }
        {
          std::lock_guard lock(health_mutex_);
          hand_gui_last_ = std::chrono::steady_clock::now();
          hand_gui_received_ = true;
        }
      });

  grasp_state_sub_ = node_->create_subscription<rtc_msgs::msg::GraspState>(
      ns + "/hand/grasp_state", rclcpp::QoS{10},
      [this](rtc_msgs::msg::GraspState::SharedPtr msg) {
        {
          std::lock_guard lock(state_mutex_);
          const auto n = msg->force_magnitude.size();
          grasp_state_.fingertips.resize(n);
          for (std::size_t i = 0; i < n; ++i) {
            auto &ft = grasp_state_.fingertips[i];
            ft.name = (i < msg->fingertip_names.size())
                          ? msg->fingertip_names[i]
                          : "";
            ft.force_magnitude = msg->force_magnitude[i];
            ft.contact_flag =
                (i < msg->contact_flag.size()) ? msg->contact_flag[i] : 0.0f;
            ft.inference_valid = (i < msg->inference_valid.size())
                                     ? msg->inference_valid[i]
                                     : false;
          }
          grasp_state_.num_active_contacts = msg->num_active_contacts;
          grasp_state_.max_force = msg->max_force;
          grasp_state_.grasp_detected = msg->grasp_detected;
          grasp_state_.force_threshold = msg->force_threshold;
          grasp_state_.min_fingertips = msg->min_fingertips;
          grasp_state_.grasp_phase = msg->grasp_phase;
          grasp_state_.grasp_target_force = msg->grasp_target_force;
          grasp_state_.finger_s.assign(msg->finger_s.begin(),
                                       msg->finger_s.end());
          grasp_state_.finger_filtered_force.assign(
              msg->finger_filtered_force.begin(),
              msg->finger_filtered_force.end());
          grasp_state_.finger_force_error.assign(
              msg->finger_force_error.begin(), msg->finger_force_error.end());
        }
        {
          std::lock_guard lock(health_mutex_);
          grasp_state_last_ = std::chrono::steady_clock::now();
          grasp_state_received_ = true;
        }
      });

  {
    auto tof_qos = rclcpp::SensorDataQoS();
    tof_qos.keep_last(100);
    tof_snapshot_sub_ = node_->create_subscription<rtc_msgs::msg::ToFSnapshot>(
        ns + "/tof/snapshot", tof_qos,
        [this](rtc_msgs::msg::ToFSnapshot::SharedPtr msg) {
          if (!tof_collecting_.load(std::memory_order_relaxed))
            return;
          std::lock_guard lock(tof_mutex_);
          tof_buffer_.push_back(*msg);
        });
  }

  arm_target_pub_ = node_->create_publisher<rtc_msgs::msg::RobotTarget>(
      ns + "/ur5e/joint_goal", rclcpp::QoS{10});
  hand_target_pub_ = node_->create_publisher<rtc_msgs::msg::RobotTarget>(
      ns + "/hand/joint_goal", rclcpp::QoS{10});

  RCLCPP_INFO(bridge_log(), "rewired controller-owned topics to '%s'",
              ctrl_name.c_str());
}

} // namespace rtc_bt
