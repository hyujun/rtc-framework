#include "ur5e_bt_coordinator/bt_ros_bridge.hpp"

#include "ur5e_bt_coordinator/bt_logging.hpp"

#include <lifecycle_msgs/msg/state.hpp>

#include <cmath>

namespace rtc_bt {

namespace {
auto bridge_log() {
  return ::rtc_bt::logging::BridgeLogger();
}

auto poses_log() {
  return ::rtc_bt::logging::PosesLogger();
}
}  // namespace

BtRosBridge::BtRosBridge(rclcpp_lifecycle::LifecycleNode::SharedPtr node, RobotProfile profile)
    : node_(std::move(node)),
      topic_namer_(std::move(profile.topics)),
      arm_dof_(profile.arm_dof),
      hand_dof_(profile.hand_dof) {
  // Seed compile-time default poses only for the default variant. A non-default
  // group (e.g. hand_group=p1b) carries robot-specific joint semantics, so it
  // must supply its own poses via YAML (hand_pose.* / arm_pose.*); LoadPose
  // Overrides errors out if none are provided (open-question 2a).
  static const TopicNamer kDefaultTopics{};
  if (topic_namer_.arm_group == kDefaultTopics.arm_group) {
    arm_poses_ = kUR5ePoses;
  }
  if (topic_namer_.hand_group == kDefaultTopics.hand_group) {
    hand_poses_ = kHandPoses;
  }

  // Default finger-index strategy (Seam C). LoadFingerMap() may swap this to an
  // ExplicitFingerResolver during on_configure when finger_map.* params exist.
  finger_resolver_ = std::make_unique<PrefixFingerResolver>();

  // Pre-rewire health labels for controller-owned topics: relative form
  // (ns="") until the first controller activates and RewireControllerTopics
  // rebinds them to the live namespaced path.
  grasp_state_topic_ = topic_namer_.GraspState("");
  wbc_state_topic_ = topic_namer_.WbcState("");

  // ── Subscribers (all RELIABLE QoS) ──────────────────────────────────────
  //
  // Phase 4: controller-owned topics (grasp_state / wbc_state / tof_snapshot /
  // arm_target / hand_target) live under /<active_controller_name>/... and
  // are rebound on every /rtc_cm/active_controller_name transition via
  // RewireControllerTopics. arm/hand joint state는 controller-agnostic
  // /rtc_cm/<group>/joint_states 로 이동했고 (rewire 불필요). TCP pose 의
  // `base → tool0_actual` 는 active controller 가 `<ns>/transforms` 로만
  // 발행하므로 (전용 /tf publisher 없음) transforms_sub_ 가 rewire 되며
  // tf_buffer_ 에 직접 feed 한다 — bare listener 만으로는 못 받는다.
  // Manager-owned topics stay at their fixed paths.

  // tf2 buffer + bare listener (kept for any /tf_static). The controller's
  // `base → tool0_actual` frames arrive via transforms_sub_ (RewireControllerTopics),
  // not the listener — the controller has no /tf publisher.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Per-group joint states (CM publishes always — independent of active ctrl).
  arm_joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      topic_namer_.ArmJointStates(), rclcpp::QoS{1},
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnArmJointState(std::move(msg)); });
  hand_joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      topic_namer_.HandJointStates(), rclcpp::QoS{1},
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnHandJointState(std::move(msg)); });

  world_target_sub_ = node_->create_subscription<geometry_msgs::msg::Polygon>(
      "/world_target_info", rclcpp::QoS{1},
      [this](geometry_msgs::msg::Polygon::SharedPtr msg) { OnWorldTarget(std::move(msg)); });

  active_ctrl_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/rtc_cm/active_controller_name", rclcpp::QoS{1}.transient_local(),
      [this](std_msgs::msg::String::SharedPtr msg) { OnActiveController(std::move(msg)); });

  estop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/system/estop_status", rclcpp::QoS{1},
      [this](std_msgs::msg::Bool::SharedPtr msg) { OnEstop(std::move(msg)); });

  // ── Shape estimation ──────────────────────────────────────────────────

  shape_estimate_sub_ = node_->create_subscription<shape_estimation_msgs::msg::ShapeEstimate>(
      "/shape/estimate", rclcpp::QoS{1},
      [this](shape_estimation_msgs::msg::ShapeEstimate::SharedPtr msg) {
        OnShapeEstimate(std::move(msg));
      });

  shape_trigger_pub_ =
      node_->create_publisher<std_msgs::msg::String>("/shape/trigger", rclcpp::QoS{1});

  shape_clear_client_ = node_->create_client<std_srvs::srv::Trigger>("/shape/clear");

  // ── /rtc_cm/* service clients (Phase 4) ───────────────────────────────
  switch_controller_client_ =
      node_->create_client<rtc_msgs::srv::SwitchController>("/rtc_cm/switch_controller");
  list_controllers_client_ =
      node_->create_client<rtc_msgs::srv::ListControllers>("/rtc_cm/list_controllers");

  RCLCPP_INFO(bridge_log(), "initialized");
}

// ── Subscription handlers ─────────────────────────────────────────────────
// Extracted callback bodies (see the header note): the subscription lambdas
// above and in RewireControllerTopics forward here 1:1, and the test-only
// BridgeStateInjector calls the same handlers directly.

void BtRosBridge::OnArmJointState(sensor_msgs::msg::JointState::SharedPtr msg) {
  {
    std::lock_guard lock(state_mutex_);
    arm_joint_positions_.assign(msg->position.begin(), msg->position.end());
  }
  {
    std::lock_guard lock(health_mutex_);
    arm_gui_last_ = std::chrono::steady_clock::now();
    arm_gui_received_ = true;
  }
}

void BtRosBridge::OnHandJointState(sensor_msgs::msg::JointState::SharedPtr msg) {
  {
    std::lock_guard lock(state_mutex_);
    hand_joint_positions_.assign(msg->position.begin(), msg->position.end());
    hand_joint_names_.assign(msg->name.begin(), msg->name.end());
  }
  {
    std::lock_guard lock(health_mutex_);
    hand_gui_last_ = std::chrono::steady_clock::now();
    hand_gui_received_ = true;
  }
}

void BtRosBridge::OnWorldTarget(geometry_msgs::msg::Polygon::SharedPtr msg) {
  if (msg->points.empty())
    return;

  // Check if all coordinates are zero (data not ready)
  bool all_zero = true;
  for (const auto& pt : msg->points) {
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
}

void BtRosBridge::OnActiveController(std_msgs::msg::String::SharedPtr msg) {
  // Rebind the controller-owned topics/clients FIRST, then publish the
  // name — so a reader that observes GetActiveController() == msg->data is
  // guaranteed the param/grasp clients are already bound (they are created
  // inside RewireControllerTopics). Setting the name first would expose a
  // window where the controller reads as active but its clients are still
  // null / point at the previous controller.
  RewireControllerTopics(msg->data);
  {
    std::lock_guard lock(state_mutex_);
    active_controller_ = msg->data;
  }
}

void BtRosBridge::OnEstop(std_msgs::msg::Bool::SharedPtr msg) {
  {
    std::lock_guard lock(state_mutex_);
    estopped_ = msg->data;
  }
  {
    std::lock_guard lock(health_mutex_);
    estop_last_ = std::chrono::steady_clock::now();
    estop_received_ = true;
  }
}

void BtRosBridge::OnShapeEstimate(shape_estimation_msgs::msg::ShapeEstimate::SharedPtr msg) {
  std::lock_guard lock(state_mutex_);
  shape_estimate_ = *msg;
  shape_estimate_valid_ = true;
}

void BtRosBridge::OnTransforms(tf2_msgs::msg::TFMessage::SharedPtr msg) {
  // #292: this message is also the evidence the task-frame selection runs on.
  // The controller broadcasts kVirtualTcpFrame only on ticks it is actually
  // controlling that point, so one sighting is conclusive; the fallback has to
  // out-wait the settle window because the closed-chain FK walk-in publishes
  // tool0 alone. Once latched, the frame is frozen until the next rewire — a
  // control point that drops out for a tick must not move what convergence is
  // measured against.
  bool latched_virtual_tcp = false;
  {
    std::lock_guard lock(state_mutex_);
    if (tf_child_frame_.empty()) {
      bool saw_virtual_tcp = false;
      bool saw_fallback = false;
      for (const auto& tf : msg->transforms) {
        if (tf.child_frame_id == kVirtualTcpFrame) {
          saw_virtual_tcp = true;
        } else if (tf.child_frame_id == tf_fallback_child_frame_) {
          saw_fallback = true;
        }
      }
      if (saw_virtual_tcp) {
        tf_child_frame_ = kVirtualTcpFrame;
        latched_virtual_tcp = true;
      } else if (saw_fallback && ++tf_fallback_seen_ >= kTaskFrameSettleMsgs) {
        tf_child_frame_ = tf_fallback_child_frame_;
      }
      // A message with neither frame is not evidence about the task frame, so
      // it neither advances nor resets the window.
    }
  }

  if (latched_virtual_tcp) {
    // NOT a mismatch — #294 resolved the premise this line was written on.
    //
    // #292 left it a WARN because the tree's absolute waypoints were assumed to
    // be tool0-authored, which would make a vtcp control point a silent
    // `T_tcp_vtcp` error. The audit found no such waypoint: every MoveToPose
    // target in trees/*.xml is a `{blackboard_var}`, and the two things that
    // fill those vars are (1) the GetCurrentPose → ComputeOffsetPose chain,
    // frame-consistent by construction since #292 made the read follow the
    // active controller, and (2) `{object_pose}`, whose position comes from
    // /world_target_info and whose orientation is the current control frame's.
    // The vision publisher is out of tree; its (x, y, z) was confirmed to be
    // the OBJECT CENTRE, so commanding the fingertip centroid there is what a
    // grasp wants — closer to the intent than putting the wrist flange on it.
    //
    // It stays as a line, at INFO, for the two things it still tells an
    // operator: which point every relative offset in the tree is measured from,
    // and where an absolute waypoint would land if one were ever added. Neither
    // is a fault, and a warning that fires on every default UR5e run stops
    // being read — which would cost more than it saves the day one is real.
    RCLCPP_INFO(node_->get_logger(),
                "active controller controls '%s', not '%s': poses are read and commanded at the "
                "virtual TCP, so the vision target places the fingertip centroid at the object "
                "centre and tree offsets are measured from there (#294).",
                kVirtualTcpFrame, tf_fallback_child_frame_.c_str());
  }

  for (const auto& tf : msg->transforms) {
    tf_buffer_->setTransform(tf, "bt_ros_bridge", /*is_static=*/false);
  }
}

void BtRosBridge::OnGraspState(rtc_msgs::msg::GraspState::SharedPtr msg) {
  {
    std::lock_guard lock(state_mutex_);
    const auto n = msg->force_magnitude.size();
    grasp_state_.fingertips.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      auto& ft = grasp_state_.fingertips[i];
      ft.name = (i < msg->fingertip_names.size()) ? msg->fingertip_names[i] : "";
      ft.force_magnitude = msg->force_magnitude[i];
      // contact_flag passthrough — sensor A producers send native
      // sigmoid prob [0..1], sensor B producers send derived binary
      // 1.0/0.0. Downstream BT nodes test with `> 0.5f` either way.
      ft.contact_flag = (i < msg->contact_flag.size()) ? msg->contact_flag[i] : 0.0f;
      ft.inference_valid = (i < msg->inference_valid.size()) ? msg->inference_valid[i] : false;
    }
    grasp_state_.num_active_contacts = msg->num_active_contacts;
    grasp_state_.max_force = msg->max_force;
    grasp_state_.grasp_detected = msg->grasp_detected;
    grasp_state_.force_threshold = msg->force_threshold;
    grasp_state_.min_fingertips = msg->min_fingertips;
    grasp_state_.grasp_phase = msg->grasp_phase;
    grasp_state_.grasp_target_force = msg->grasp_target_force;
    grasp_state_.finger_s.assign(msg->finger_s.begin(), msg->finger_s.end());
    grasp_state_.finger_filtered_force.assign(msg->finger_filtered_force.begin(),
                                              msg->finger_filtered_force.end());
    grasp_state_.finger_force_error.assign(msg->finger_force_error.begin(),
                                           msg->finger_force_error.end());
    grasp_state_.finger_stiffness_est.assign(msg->finger_stiffness_est.begin(),
                                             msg->finger_stiffness_est.end());
    grasp_state_.received_at = std::chrono::steady_clock::now();
  }
  {
    std::lock_guard lock(health_mutex_);
    grasp_state_last_ = std::chrono::steady_clock::now();
    grasp_state_received_ = true;
  }
}

void BtRosBridge::OnWbcState(rtc_msgs::msg::WbcState::SharedPtr msg) {
  {
    std::lock_guard lock(state_mutex_);
    const auto n = msg->force_magnitude.size();
    wbc_state_.fingertips.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      auto& ft = wbc_state_.fingertips[i];
      ft.name = (i < msg->fingertip_names.size()) ? msg->fingertip_names[i] : "";
      ft.force_magnitude = msg->force_magnitude[i];
      // contact_flag passthrough — same sensor-A/B capability semantics
      // as GraspState branch above (see WbcState.msg).
      ft.contact_flag = (i < msg->contact_flag.size()) ? msg->contact_flag[i] : 0.0f;
      ft.displacement = (i < msg->displacement.size()) ? msg->displacement[i] : 0.0f;
    }
    wbc_state_.num_active_contacts = msg->num_active_contacts;
    wbc_state_.max_force = msg->max_force;
    wbc_state_.grasp_target_force = msg->grasp_target_force;
    wbc_state_.grasp_detected = msg->grasp_detected;
    wbc_state_.min_fingertips = msg->min_fingertips;
    wbc_state_.phase = msg->phase;
    wbc_state_.tsid_solve_us = msg->tsid_solve_us;
    wbc_state_.tsid_solver_ok = msg->tsid_solver_ok;
    wbc_state_.qp_fail_count = msg->qp_fail_count;
  }
  {
    std::lock_guard lock(health_mutex_);
    wbc_state_last_ = std::chrono::steady_clock::now();
    wbc_state_received_ = true;
  }
}

void BtRosBridge::OnToFSnapshot(rtc_msgs::msg::ToFSnapshot::SharedPtr msg) {
  if (!tof_collecting_.load(std::memory_order_relaxed))
    return;
  std::lock_guard lock(tof_mutex_);
  tof_buffer_.push_back(*msg);
}

// ── Cached state accessors ────────────────────────────────────────────────

// Runs the tf2 lookup and refreshes tcp_pose_ / tcp_pose_valid_, returning
// whether this call produced a live reading. Both public accessors go through
// it so neither depends on the other having been called first — an
// IsTcpPoseValid() that merely reported a remembered flag would answer about
// whenever GetTcpPose() last ran, which is a trap for any caller that checks
// validity before reading (#292).
bool BtRosBridge::RefreshTcpPose() const {
  std::string child;
  {
    std::lock_guard lock(state_mutex_);
    child = tf_child_frame_;
  }
  // Unsettled frame: there is nothing to look up. Guessing a frame here would
  // produce a pose that looks live and is measured against the wrong control
  // point, which is worse for a convergence test than having no pose at all.
  if (tf_buffer_ && !child.empty()) {
    try {
      const auto tfs = tf_buffer_->lookupTransform(tf_parent_frame_, child, tf2::TimePointZero);
      const double qw = tfs.transform.rotation.w;
      const double qx = tfs.transform.rotation.x;
      const double qy = tfs.transform.rotation.y;
      const double qz = tfs.transform.rotation.z;
      const double roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
      const double sinp = 2.0 * (qw * qy - qz * qx);
      const double pitch =
          (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
      const double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      const Pose6D pose{tfs.transform.translation.x,
                        tfs.transform.translation.y,
                        tfs.transform.translation.z,
                        roll,
                        pitch,
                        yaw};
      // Cache the fresh lookup so a later lookup failure (transform gap, halt)
      // falls back to this last-known pose rather than a zero-initialized one.
      std::lock_guard lock(state_mutex_);
      tcp_pose_ = pose;
      tcp_pose_valid_ = true;
      return true;
    } catch (const tf2::TransformException&) {
      // fall through: the cached pose stays, its freshness does not
    }
  }
  std::lock_guard lock(state_mutex_);
  // #292: a failed lookup means the cache is no longer a live reading. The pose
  // itself is kept (callers that only want a last-known value keep working),
  // but it stops claiming to be current — the case that matters is right after
  // a rewire, where the cache still holds the PREVIOUS controller's pose and a
  // convergence test would otherwise succeed against a pose nobody is holding.
  tcp_pose_valid_ = false;
  return false;
}

Pose6D BtRosBridge::GetTcpPose() const {
  // Phase 4: tf2 lookup of `base → <active controller's task frame>`. Returns
  // the cached value when the lookup fails (e.g. no transform yet at startup)
  // so callers see a last-known pose rather than zeros — pair with
  // IsTcpPoseValid() to tell a live reading from a stale one (#292).
  RefreshTcpPose();
  std::lock_guard lock(state_mutex_);
  return tcp_pose_;
}

bool BtRosBridge::IsTcpPoseValid() const {
  return RefreshTcpPose();
}

std::string BtRosBridge::GetTaskFrame() const {
  std::lock_guard lock(state_mutex_);
  return tf_child_frame_;
}

std::vector<double> BtRosBridge::GetArmJointPositions() const {
  std::lock_guard lock(state_mutex_);
  return arm_joint_positions_;
}

std::vector<double> BtRosBridge::GetHandJointPositions() const {
  std::lock_guard lock(state_mutex_);
  return hand_joint_positions_;
}

bool BtRosBridge::IsHandStateReady() const {
  std::lock_guard lock(state_mutex_);
  // names/positions consistency + the configured hand DoF as a floor (issue
  // #161 comment §3). OnHandJointState assigns names and positions from one
  // message so the widths agree in practice; asserting it here keeps a
  // malformed first message (name/position width mismatch, which the BT nodes
  // would index-mismatch on) out of "ready". The DoF check is `>=`, not `==`:
  // an empty or partial (< hand_dof_) startup message must not read ready, but
  // a message carrying *more* joints than configured is safe — the resolver and
  // the finger nodes already skip out-of-range indices (see the OOB regression
  // in test_hand_nodes) — so it must not hang the gate forever.
  return !hand_joint_names_.empty() && hand_joint_positions_.size() == hand_joint_names_.size() &&
         static_cast<int>(hand_joint_names_.size()) >= hand_dof_;
}

std::vector<int> BtRosBridge::GetFingerJointIndices(const std::string& key) const {
  std::lock_guard lock(state_mutex_);
  return finger_resolver_->Resolve(hand_joint_names_, key);
}

CachedGraspState BtRosBridge::GetGraspState() const {
  std::lock_guard lock(state_mutex_);
  return grasp_state_;
}

CachedWbcState BtRosBridge::GetWbcState() const {
  std::lock_guard lock(state_mutex_);
  return wbc_state_;
}

bool BtRosBridge::GetObjectPose(Pose6D& pose) const {
  return GetWorldTargetPose(pose);
}

bool BtRosBridge::GetWorldTargetPose(Pose6D& pose) const {
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

bool BtRosBridge::IsControllerWired() const {
  // Unlocked read, symmetric with the Publish* dereferences below: the rewire
  // writes these and the tick reads them, and both run on the single-threaded
  // executor's mutually-exclusive callback group (see the THREADING note on
  // RewireControllerTopics).
  return arm_target_pub_ != nullptr && hand_target_pub_ != nullptr;
}

// ── Publishers ────────────────────────────────────────────────────────────

void BtRosBridge::PublishArmTarget(const Pose6D& target) {
  if (!arm_target_pub_) {
    RCLCPP_WARN_THROTTLE(bridge_log(), *node_->get_clock(), logging::kThrottleSlowMs,
                         "arm target dropped: no active controller wired yet");
    return;
  }
  rtc_msgs::msg::RobotTarget msg;
  msg.header.stamp = node_->now();
  msg.goal_type = "task";
  msg.task_target = {target.x, target.y, target.z, target.roll, target.pitch, target.yaw};
  arm_target_pub_->publish(msg);
}

void BtRosBridge::PublishArmJointTarget(const std::vector<double>& target) {
  if (!arm_target_pub_) {
    RCLCPP_WARN_THROTTLE(bridge_log(), *node_->get_clock(), logging::kThrottleSlowMs,
                         "arm joint target dropped: no active controller wired yet");
    return;
  }
  rtc_msgs::msg::RobotTarget msg;
  msg.header.stamp = node_->now();
  msg.goal_type = "joint";
  msg.joint_target.assign(target.begin(), target.end());
  arm_target_pub_->publish(msg);
}

void BtRosBridge::PublishHandTarget(const std::vector<double>& target) {
  // Guard before the state write: last_hand_target_ is the incremental base for
  // grasp commands (bt_utils.hpp), so it must track what actually went out.
  if (!hand_target_pub_) {
    RCLCPP_WARN_THROTTLE(bridge_log(), *node_->get_clock(), logging::kThrottleSlowMs,
                         "hand target dropped: no active controller wired yet");
    return;
  }
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

BtRosBridge::SwitchRequestHandle BtRosBridge::RequestSwitchControllerAsync(const std::string& name,
                                                                           std::string& message) {
  // Non-blocking readiness probe (0ms). Not-ready is retriable by the caller.
  if (!switch_controller_client_->service_is_ready()) {
    message = "switch_controller service not ready";
    return {};  // invalid future
  }
  auto req = std::make_shared<rtc_msgs::srv::SwitchController::Request>();
  req->activate_controllers = {name};
  req->strictness = rtc_msgs::srv::SwitchController::Request::STRICT;
  // timeout field is informational to the server; the BT node bounds latency.
  auto handle = switch_controller_client_->async_send_request(req);
  message.clear();
  const int64_t request_id = handle.request_id;
  return {handle.future.share(), request_id};
}

void BtRosBridge::CancelSwitchControllerRequest(int64_t request_id) {
  if (request_id == 0)
    return;
  switch_controller_client_->remove_pending_request(request_id);
}

// ── Shape estimation ──────────────────────────────────────────────────────

void BtRosBridge::PublishShapeTrigger(const std::string& command) {
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
      request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto result = future.get();
        RCLCPP_INFO(bridge_log(), "/shape/clear: %s",
                    result->success ? "OK" : result->message.c_str());
      });
}

bool BtRosBridge::GetShapeEstimate(shape_estimation_msgs::msg::ShapeEstimate& out) const {
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
  RCLCPP_INFO(bridge_log(), "ToF collection stopped (%zu snapshots)", tof_buffer_.size());
}

const std::vector<rtc_msgs::msg::ToFSnapshot>& BtRosBridge::GetCollectedToFData() const {
  // Caller must ensure collection is stopped before reading.
  return tof_buffer_;
}

std::size_t BtRosBridge::GetCollectedToFCount() const {
  std::lock_guard lock(tof_mutex_);
  return tof_buffer_.size();
}

// ── Pose library ──────────────────────────────────────────────────────────

void BtRosBridge::LoadPoseOverrides(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
  // Discover hand_pose.* parameters
  auto hand_result = node->list_parameters({"hand_pose"}, 1);
  int hand_count = 0;
  for (const auto& param_name : hand_result.names) {
    // param_name = "hand_pose.home" → pose_name = "home"
    const std::string prefix = "hand_pose.";
    if (param_name.size() <= prefix.size())
      continue;
    std::string pose_name = param_name.substr(prefix.size());

    try {
      auto vals = node->get_parameter(param_name).as_double_array();
      if (vals.size() != static_cast<std::size_t>(hand_dof_)) {
        RCLCPP_WARN(poses_log(), "hand_pose.%s has %zu values (expected %d), skipped",
                    pose_name.c_str(), vals.size(), hand_dof_);
        continue;
      }
      HandPose pose(static_cast<std::size_t>(hand_dof_), 0.0);
      for (int i = 0; i < hand_dof_; ++i) {
        pose[i] = vals[i] * kDeg2Rad;
      }
      hand_poses_[pose_name] = pose;
      ++hand_count;
    } catch (const std::exception& e) {
      RCLCPP_WARN(poses_log(), "failed to load hand_pose.%s: %s", pose_name.c_str(), e.what());
    }
  }

  // Discover arm_pose.* parameters
  auto arm_result = node->list_parameters({"arm_pose"}, 1);
  int arm_count = 0;
  for (const auto& param_name : arm_result.names) {
    const std::string prefix = "arm_pose.";
    if (param_name.size() <= prefix.size())
      continue;
    std::string pose_name = param_name.substr(prefix.size());

    try {
      auto vals = node->get_parameter(param_name).as_double_array();
      if (vals.size() != static_cast<std::size_t>(arm_dof_)) {
        RCLCPP_WARN(poses_log(), "arm_pose.%s has %zu values (expected %d), skipped",
                    pose_name.c_str(), vals.size(), arm_dof_);
        continue;
      }
      ArmPose pose(static_cast<std::size_t>(arm_dof_), 0.0);
      for (int i = 0; i < arm_dof_; ++i) {
        pose[i] = vals[i] * kDeg2Rad;
      }
      arm_poses_[pose_name] = pose;
      ++arm_count;
    } catch (const std::exception& e) {
      RCLCPP_WARN(poses_log(), "failed to load arm_pose.%s: %s", pose_name.c_str(), e.what());
    }
  }

  RCLCPP_INFO(poses_log(), "loaded %d hand poses, %d arm poses (total: %zu hand, %zu arm)",
              hand_count, arm_count, hand_poses_.size(), arm_poses_.size());

  // Non-default variants seed no compile-time defaults (see constructor), so an
  // empty map means the operator forgot the poses YAML — fail fast rather than
  // let every pose lookup throw an opaque "unknown pose" at tick time.
  if (hand_poses_.empty()) {
    throw std::runtime_error(
        "no hand poses loaded: non-default hand_group requires hand_pose.* in a poses YAML");
  }
  if (arm_poses_.empty()) {
    throw std::runtime_error(
        "no arm poses loaded: non-default arm_group requires arm_pose.* in a poses YAML");
  }
}

void BtRosBridge::LoadFingerMap(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
  // Discover finger_map.<finger> integer-array parameters. Their presence is
  // the signal to switch from prefix-matching to an explicit finger→index map
  // (needed for hands whose joint names carry no finger prefix, e.g. LEAP).
  auto result = node->list_parameters({"finger_map"}, 1);
  std::map<std::string, std::vector<int>> finger_map;
  const std::string prefix = "finger_map.";
  for (const auto& param_name : result.names) {
    if (param_name.size() <= prefix.size())
      continue;
    std::string finger = param_name.substr(prefix.size());
    try {
      auto vals = node->get_parameter(param_name).as_integer_array();
      std::vector<int> indices;
      indices.reserve(vals.size());
      for (auto v : vals) {
        indices.push_back(static_cast<int>(v));
      }
      finger_map[finger] = std::move(indices);
    } catch (const std::exception& e) {
      RCLCPP_WARN(poses_log(), "failed to load finger_map.%s: %s", finger.c_str(), e.what());
    }
  }

  if (!finger_map.empty()) {
    // Validate indices against the hand DoF now, at configure time: an out-of-
    // range index would otherwise become an OOB vector subscript the first time
    // that finger is commanded, far from the config error. Throwing here surfaces
    // as a clean on_configure FAILURE (the transition wraps this call).
    for (const auto& [finger, indices] : finger_map) {
      for (int idx : indices) {
        if (idx < 0 || idx >= hand_dof_) {
          throw std::runtime_error("finger_map." + finger + " index " + std::to_string(idx) +
                                   " out of range [0," + std::to_string(hand_dof_) + ")");
        }
      }
    }
    RCLCPP_INFO(poses_log(), "finger resolver: explicit (%zu finger maps)", finger_map.size());
    finger_resolver_ = std::make_unique<ExplicitFingerResolver>(std::move(finger_map));
  } else {
    RCLCPP_INFO(poses_log(), "finger resolver: prefix (joint-name matching)");
  }
}

const HandPose& BtRosBridge::GetHandPose(const std::string& name) const {
  auto it = hand_poses_.find(name);
  if (it == hand_poses_.end()) {
    throw BT::RuntimeError("PoseLibrary: unknown hand pose: " + name);
  }
  return it->second;
}

const ArmPose& BtRosBridge::GetArmPose(const std::string& name) const {
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

  auto make_health = [&](const std::string& name, bool received, TimePoint last) {
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
      make_health(topic_namer_.ArmJointStates(), arm_gui_received_, arm_gui_last_),
      make_health(topic_namer_.HandJointStates(), hand_gui_received_, hand_gui_last_),
      make_health(grasp_state_topic_, grasp_state_received_, grasp_state_last_),
      make_health(wbc_state_topic_, wbc_state_received_, wbc_state_last_),
      make_health("/world_target_info", world_target_received_, world_target_last_),
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
  return is_ok(arm_gui_received_, arm_gui_last_) && is_ok(hand_gui_received_, hand_gui_last_);
}

// ── Phase 4: dynamic rewiring for controller-owned topics ────────────────
// Invoked from the active_ctrl_sub_ callback. Idempotent — skips when the
// controller name has not changed since the last call.
//
// THREADING: the sub `.reset()` + `create_subscription` sequence below is NOT
// externally locked against the callbacks it rewires (transforms_sub_,
// grasp_state_sub_, …) or against GetTcpPose (tick timer). Safe ONLY because
// the node runs under a SingleThreadedExecutor (main.cpp `rclcpp::spin`) with
// the default MutuallyExclusive callback group — so rewire, those callbacks,
// and GetTcpPose are mutually exclusive by construction. If a future change
// adopts a MultiThreadedExecutor or a Reentrant group for any of these,
// revisit: `.reset()`-while-callback-in-flight and interleaved tf_buffer_
// writes would become racy and this must gain an explicit shared group / lock.
void BtRosBridge::RewireControllerTopics(const std::string& ctrl_name) {
  if (ctrl_name.empty())
    return;
  {
    std::lock_guard lock(controller_topics_mutex_);
    if (ctrl_name == rewired_controller_)
      return;
  }
  // rewired_controller_ is committed at the END, together with the clients it
  // describes. Committing it here instead would make a throw from any create_*
  // below unrecoverable: the name is already recorded, so a re-delivery of the
  // same name early-returns above, and the pubs reset below stay null forever —
  // IsControllerWired() never latches and the tree never ticks again.

  const std::string ns = "/" + ctrl_name;

  // Rebind the controller-owned health labels to the live namespaced paths so
  // the watchdog reports the topic actually subscribed below.
  {
    std::lock_guard lock(health_mutex_);
    grasp_state_topic_ = topic_namer_.GraspState(ns);
    wbc_state_topic_ = topic_namer_.WbcState(ns);
  }

  // Drop previous sub/pub handles before recreating to avoid two live
  // subscribers holding references to the same state maps.
  // Phase 4: arm/hand joint state는 controller-agnostic /rtc_cm/<group>/
  // joint_states 로 이동 — rewire 대상 아님 (constructor에서 1회 설정).
  // TCP pose 의 `base → tool0_actual` 는 active controller 의 `<ns>/transforms`
  // 로만 발행되므로 transforms_sub_ 를 여기서 rewire 해 tf_buffer_ 에 직접 feed.
  grasp_state_sub_.reset();
  wbc_state_sub_.reset();
  tof_snapshot_sub_.reset();
  transforms_sub_.reset();
  arm_target_pub_.reset();
  hand_target_pub_.reset();

  // #292: the task frame and the pose cache are controller-sourced state and
  // have to die with the subscription that fed them. Nothing the previous
  // controller broadcast is evidence about what the next one controls, and its
  // cached pose must stop reading as live — otherwise MoveToPose's convergence
  // test can find the stale pose within tolerance of its target and report
  // SUCCESS for a motion that never happened. This reset is the load-bearing
  // half, and test_task_frame's two rewire cases fail without it.
  //
  // The tf buffer is replaced for the same reason, though it is defence in
  // depth rather than the primary guard here: the frame selection is driven by
  // observed TFMessages (OnTransforms), not by what happens to resolve through
  // lookupTransform, so a stale buffer cannot by itself mis-select a frame.
  // What it would leave behind is a rewired-away controller's transforms still
  // resolvable for 10 s (tf2's default cache) — which the reset below already
  // makes unreachable, but only for as long as selection stays message-driven.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  {
    std::lock_guard lock(state_mutex_);
    tf_child_frame_.clear();
    tf_fallback_seen_ = 0;
    tcp_pose_valid_ = false;
  }

  // Feed the active controller's broadcast TF into tf_buffer_ so GetTcpPose's
  // `base → tool0_actual` lookup resolves. The controller publishes these on
  // `<ns>/transforms`, not /tf, so this manual feed (mirroring demo_gui
  // _transforms_cb) is what makes the bare TransformListener's buffer usable —
  // no external /tf re-broadcaster required. No restamp: GetTcpPose looks up
  // the latest transform (TimePointZero), so the source stamp is fine.
  transforms_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      topic_namer_.Transforms(ns), rclcpp::QoS{1},
      [this](tf2_msgs::msg::TFMessage::SharedPtr msg) { OnTransforms(std::move(msg)); });

  grasp_state_sub_ = node_->create_subscription<rtc_msgs::msg::GraspState>(
      topic_namer_.GraspState(ns), rclcpp::QoS{1},
      [this](rtc_msgs::msg::GraspState::SharedPtr msg) { OnGraspState(std::move(msg)); });

  {
    // ARCH-6 exception: the callback push_backs every snapshot into a
    // collection buffer (GetCollectedToFData iterates all) — depth 1 would
    // drop intermediate samples during a live scan. Deep sensor queue by design.
    auto tof_qos = rclcpp::SensorDataQoS();
    tof_qos.keep_last(100);  // ARCH-6-exempt
    tof_snapshot_sub_ = node_->create_subscription<rtc_msgs::msg::ToFSnapshot>(
        ns + "/tof/snapshot", tof_qos,
        [this](rtc_msgs::msg::ToFSnapshot::SharedPtr msg) { OnToFSnapshot(std::move(msg)); });
  }

  // Subscribe to wbc_state alongside grasp_state — only the active controller
  // publishes one of them, the other stays empty/stale. Caller picks via
  // GetGraspState() vs GetWbcState() based on the active controller.
  wbc_state_sub_ = node_->create_subscription<rtc_msgs::msg::WbcState>(
      topic_namer_.WbcState(ns), rclcpp::QoS{1},
      [this](rtc_msgs::msg::WbcState::SharedPtr msg) { OnWbcState(std::move(msg)); });

  arm_target_pub_ = node_->create_publisher<rtc_msgs::msg::RobotTarget>(
      topic_namer_.ArmJointGoal(ns), rclcpp::QoS{1});
  hand_target_pub_ = node_->create_publisher<rtc_msgs::msg::RobotTarget>(
      topic_namer_.HandJointGoal(ns), rclcpp::QoS{1});
  // LifecycleNode::on_activate sweeps only the managed entities that exist when
  // it runs. Unlike the constructor's publishers, these are born in the rewire,
  // which fires whenever the latched name arrives — usually well after the node
  // went ACTIVE. They would miss that sweep and stay disabled for good, so
  // enable them here. A rewire that lands while still INACTIVE needs nothing:
  // the on_activate to come sweeps them like any other entity.
  if (node_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    arm_target_pub_->on_activate();
    hand_target_pub_->on_activate();
  }

  // Phase C: bind parameter + grasp_command clients to the active controller.
  //   Param services live on the LifecycleNode FQN /<ctrl>/<ctrl>; relative
  //   srv 'grasp_command' resolves under namespace /<ctrl>/grasp_command.
  // Assign under controller_topics_mutex_ so the reads in
  // SetActiveControllerGainsAsync / SendGraspCommandAsync (which take the same
  // lock) see a consistent client pointer — keeps the locking symmetric rather
  // than relying only on the single-thread executor invariant.
  auto new_param_client =
      std::make_shared<rclcpp::AsyncParametersClient>(node_, ns + "/" + ctrl_name);
  auto new_grasp_client = node_->create_client<rtc_msgs::srv::GraspCommand>(ns + "/grasp_command");
  {
    std::lock_guard lock(controller_topics_mutex_);
    active_param_client_ = std::move(new_param_client);
    grasp_command_client_ = std::move(new_grasp_client);
    // Commit last: everything this name promises now exists. Until this line a
    // throw above leaves the previous name in place, so the next delivery of
    // this one retries the rewire instead of being skipped as a duplicate.
    rewired_controller_ = ctrl_name;
  }

  RCLCPP_INFO(bridge_log(), "rewired controller-owned topics to '%s'", ctrl_name.c_str());
}

// ── Phase C: parameter + grasp_command async senders ───────────────────────

BtRosBridge::SetParametersFuture BtRosBridge::SetActiveControllerGainsAsync(
    const std::vector<rclcpp::Parameter>& params, std::string& message) {
  rclcpp::AsyncParametersClient::SharedPtr client;
  std::string ctrl;
  {
    std::lock_guard lock(controller_topics_mutex_);
    client = active_param_client_;
    ctrl = rewired_controller_;
  }
  if (!client) {
    message = "no active controller (RewireControllerTopics not invoked)";
    return {};  // invalid future
  }
  // Non-blocking readiness probe (0ms). Not-ready is retriable by the caller.
  if (!client->service_is_ready()) {
    message = "parameter service for /" + ctrl + " not ready";
    return {};
  }
  message.clear();
  return client->set_parameters_atomically(params);
}

BtRosBridge::GraspCommandFuture BtRosBridge::SendGraspCommandAsync(uint8_t command,
                                                                   double target_force,
                                                                   std::string& message) {
  rclcpp::Client<rtc_msgs::srv::GraspCommand>::SharedPtr client;
  std::string ctrl;
  {
    std::lock_guard lock(controller_topics_mutex_);
    client = grasp_command_client_;
    ctrl = rewired_controller_;
  }
  if (!client) {
    message = "no active controller (RewireControllerTopics not invoked)";
    return {};  // invalid future
  }
  // Non-blocking readiness probe (0ms). Not-ready is retriable by the caller.
  if (!client->service_is_ready()) {
    message = "/" + ctrl + "/grasp_command service not ready";
    return {};
  }
  auto req = std::make_shared<rtc_msgs::srv::GraspCommand::Request>();
  req->command = command;
  req->target_force = target_force;
  message.clear();
  return client->async_send_request(req).future.share();
}

}  // namespace rtc_bt
