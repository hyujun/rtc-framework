// ── Subscription and publisher creation, topic parameter exposure
// ─────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <lifecycle_msgs/msg/state.hpp>

#include <set>

namespace urtc = rtc;

void RtControllerNode::CreateSubscriptions() {
  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = cb_group_sensor_;

  // ── Helper: find DeviceTimeoutEntry index for a given group name ─────────
  auto find_timeout_idx_by_group = [this](const std::string &group) -> int {
    for (std::size_t i = 0; i < device_timeouts_.size(); ++i) {
      if (device_timeouts_[i].group_name == group)
        return static_cast<int>(i);
    }
    return -1;
  };

  // ── Create subscriptions for all active device groups ────────────────────
  //
  // QoS strategy per role:
  //   kState / kSensorState → BEST_EFFORT, depth 2
  //   kTarget → RELIABLE, depth 10
  rclcpp::QoS sensor_sub_qos{2};
  sensor_sub_qos.best_effort();

  std::set<std::string> created_topics;

  for (const auto &tc : controller_topic_configs_) {
    for (const auto &[group_name, group] : tc.groups) {
      if (!active_groups_.contains(group_name))
        continue;

      const int slot = group_slot_map_[group_name];
      const int dt_idx = find_timeout_idx_by_group(group_name);

      for (const auto &entry : group.subscribe) {
        // Controller-owned subscriptions are created by the controller's own
        // on_configure — CM does not bind them here.
        if (entry.ownership == urtc::TopicOwnership::kController)
          continue;
        if (!created_topics.insert(entry.topic_name).second)
          continue;

        switch (entry.role) {
        case urtc::SubscribeRole::kState: {
          auto sub = create_subscription<sensor_msgs::msg::JointState>(
              entry.topic_name, sensor_sub_qos,
              [this, slot,
               dt_idx](sensor_msgs::msg::JointState::SharedPtr msg) {
                DeviceJointStateCallback(slot, std::move(msg));
                if (dt_idx >= 0) {
                  const auto dti = static_cast<std::size_t>(dt_idx);
                  device_timeouts_[dti].last_update =
                      std::chrono::steady_clock::now();
                  device_timeouts_[dti].received.store(
                      true, std::memory_order_relaxed);
                }
              },
              sub_options);
          topic_subscriptions_.push_back(sub);
          RCLCPP_INFO(get_logger(),
                      "  Subscribe [%s/state]: %s (slot %d, BEST_EFFORT/2)",
                      group_name.c_str(), entry.topic_name.c_str(), slot);
          break;
        }
        case urtc::SubscribeRole::kMotorState: {
          auto sub = create_subscription<sensor_msgs::msg::JointState>(
              entry.topic_name, sensor_sub_qos,
              [this, slot,
               dt_idx](sensor_msgs::msg::JointState::SharedPtr msg) {
                DeviceMotorStateCallback(slot, std::move(msg));
                if (dt_idx >= 0) {
                  const auto dti = static_cast<std::size_t>(dt_idx);
                  device_timeouts_[dti].last_update =
                      std::chrono::steady_clock::now();
                  device_timeouts_[dti].received.store(
                      true, std::memory_order_relaxed);
                }
              },
              sub_options);
          topic_subscriptions_.push_back(sub);
          RCLCPP_INFO(
              get_logger(),
              "  Subscribe [%s/motor_state]: %s (slot %d, BEST_EFFORT/2)",
              group_name.c_str(), entry.topic_name.c_str(), slot);
          break;
        }
        case urtc::SubscribeRole::kSensorState: {
          auto sub = create_subscription<rtc_msgs::msg::HandSensorState>(
              entry.topic_name, sensor_sub_qos,
              [this, slot,
               dt_idx](rtc_msgs::msg::HandSensorState::SharedPtr msg) {
                HandSensorStateCallback(slot, std::move(msg));
                if (dt_idx >= 0) {
                  const auto dti = static_cast<std::size_t>(dt_idx);
                  device_timeouts_[dti].last_update =
                      std::chrono::steady_clock::now();
                  device_timeouts_[dti].received.store(
                      true, std::memory_order_relaxed);
                }
              },
              sub_options);
          topic_subscriptions_.push_back(sub);
          RCLCPP_INFO(
              get_logger(),
              "  Subscribe [%s/sensor_state]: %s (slot %d, BEST_EFFORT/2)",
              group_name.c_str(), entry.topic_name.c_str(), slot);
          break;
        }
        case urtc::SubscribeRole::kTarget: {
          auto sub = create_subscription<rtc_msgs::msg::RobotTarget>(
              entry.topic_name, 10,
              [this, slot](rtc_msgs::msg::RobotTarget::SharedPtr msg) {
                DeviceTargetCallback(slot, std::move(msg));
              },
              sub_options);
          topic_subscriptions_.push_back(sub);
          RCLCPP_INFO(get_logger(), "  Subscribe [%s/target]: %s (slot %d)",
                      group_name.c_str(), entry.topic_name.c_str(), slot);
          break;
        }
        }
      }
    }
  }

  // ── GUI control subscriptions (use aux callback group to avoid blocking
  //    sensor callbacks, which would stall device_timeouts_ updates and
  //    trigger E-STOP during controller switch / gains update) ──────────────
  auto aux_sub_options = rclcpp::SubscriptionOptions();
  aux_sub_options.callback_group = cb_group_aux_;

  controller_gains_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/" + robot_ns_ + "/controller_gains", 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        const int idx = active_controller_idx_.load(std::memory_order_acquire);
        controllers_[static_cast<std::size_t>(idx)]->UpdateGainsFromMsg(
            msg->data);
        RCLCPP_INFO(get_logger(), "Gains updated for %s",
                    controllers_[static_cast<std::size_t>(idx)]->Name().data());
      },
      aux_sub_options);

  request_gains_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/" + robot_ns_ + "/request_gains", 10,
      [this](std_msgs::msg::Bool::SharedPtr /*msg*/) {
        const int idx = active_controller_idx_.load(std::memory_order_acquire);
        const auto gains =
            controllers_[static_cast<std::size_t>(idx)]->GetCurrentGains();
        std_msgs::msg::Float64MultiArray gains_msg;
        gains_msg.data = gains;
        current_gains_pub_->publish(gains_msg);
        RCLCPP_INFO(get_logger(), "Published current gains for %s (%zu values)",
                    controllers_[static_cast<std::size_t>(idx)]->Name().data(),
                    gains.size());
      },
      aux_sub_options);
}

void RtControllerNode::CreatePublishers() {
  // BEST_EFFORT + depth 1: minimises DDS overhead on the 500 Hz RT path.
  rclcpp::QoS cmd_qos{1};
  cmd_qos.best_effort();

  // Helper: create a publisher for a publish entry if not already created.
  auto create_pub = [&](const urtc::PublishTopicEntry &entry,
                        const std::string &group_name) {
    switch (entry.role) {
    case urtc::PublishRole::kJointCommand: {
      if (joint_command_publishers_.count(entry.topic_name) > 0) {
        return;
      }
      JointCommandPublisherEntry jce;
      jce.publisher = create_publisher<rtc_msgs::msg::JointCommand>(
          entry.topic_name, cmd_qos);
      {
        auto cfg_it = device_name_configs_.find(group_name);
        if (cfg_it != device_name_configs_.end()) {
          const auto &state_names = cfg_it->second.joint_state_names;
          const auto &cmd_names = cfg_it->second.joint_command_names;
          jce.msg.joint_names = cmd_names;
          jce.msg.values.resize(cmd_names.size(), 0.0);
          if (!state_names.empty() && !cmd_names.empty() &&
              state_names != cmd_names) {
            jce.reorder_map.resize(cmd_names.size(), -1);
            for (std::size_t ci = 0; ci < cmd_names.size(); ++ci) {
              for (std::size_t si = 0; si < state_names.size(); ++si) {
                if (cmd_names[ci] == state_names[si]) {
                  jce.reorder_map[ci] = static_cast<int>(si);
                  break;
                }
              }
            }
            RCLCPP_INFO(get_logger(),
                        "  [%s] kJointCommand reorder map built (%zu → %zu)",
                        group_name.c_str(), state_names.size(),
                        cmd_names.size());
          }
        }
      }
      jce.msg.command_type = "position";
      joint_command_publishers_[entry.topic_name] = std::move(jce);
      RCLCPP_INFO(get_logger(),
                  "  Publish [%s/joint_command]: %s (JointCommand)",
                  group_name.c_str(), entry.topic_name.c_str());
      return;
    }
    default:
      break;
    }

    // ── Typed message publishers (GuiPosition, RobotTarget, etc.) ───────────
    auto cfg_it = device_name_configs_.find(group_name);
    const auto &joint_names = (cfg_it != device_name_configs_.end())
                                  ? cfg_it->second.joint_state_names
                                  : std::vector<std::string>{};
    const auto &sensor_names = (cfg_it != device_name_configs_.end())
                                   ? cfg_it->second.sensor_names
                                   : std::vector<std::string>{};
    const auto n = joint_names.size();

    auto log_pub = [&](const char *role_str) {
      RCLCPP_INFO(get_logger(), "  Publish [%s/%s]: %s", group_name.c_str(),
                  role_str, entry.topic_name.c_str());
    };

    switch (entry.role) {
    case urtc::PublishRole::kGuiPosition: {
      if (gui_position_publishers_.count(entry.topic_name) > 0) {
        return;
      }
      TypedPublisherEntry<rtc_msgs::msg::GuiPosition> pe;
      pe.publisher =
          create_publisher<rtc_msgs::msg::GuiPosition>(entry.topic_name, 10);
      pe.msg.joint_names.assign(joint_names.begin(), joint_names.end());
      pe.msg.joint_positions.resize(n, 0.0);
      gui_position_publishers_[entry.topic_name] = std::move(pe);
      log_pub("gui_position");
      return;
    }
    case urtc::PublishRole::kRobotTarget: {
      if (robot_target_publishers_.count(entry.topic_name) > 0) {
        return;
      }
      TypedPublisherEntry<rtc_msgs::msg::RobotTarget> pe;
      pe.publisher = create_publisher<rtc_msgs::msg::RobotTarget>(
          entry.topic_name, cmd_qos);
      pe.msg.joint_names.assign(joint_names.begin(), joint_names.end());
      pe.msg.joint_target.resize(n, 0.0);
      pe.msg.goal_type = "joint";
      robot_target_publishers_[entry.topic_name] = std::move(pe);
      log_pub("robot_target");
      return;
    }
    case urtc::PublishRole::kDeviceStateLog: {
      if (device_state_log_publishers_.count(entry.topic_name) > 0) {
        return;
      }
      TypedPublisherEntry<rtc_msgs::msg::DeviceStateLog> pe;
      pe.publisher = create_publisher<rtc_msgs::msg::DeviceStateLog>(
          entry.topic_name, cmd_qos);
      pe.msg.joint_names.assign(joint_names.begin(), joint_names.end());
      pe.msg.actual_positions.resize(n, 0.0);
      pe.msg.actual_velocities.resize(n, 0.0);
      pe.msg.efforts.resize(n, 0.0);
      pe.msg.commands.resize(n, 0.0);
      pe.msg.command_type = "position";
      pe.msg.goal_type = "joint";
      pe.msg.joint_goal.resize(n, 0.0);
      pe.msg.trajectory_positions.resize(n, 0.0);
      pe.msg.trajectory_velocities.resize(n, 0.0);
      if (cfg_it != device_name_configs_.end() &&
          !cfg_it->second.motor_state_names.empty()) {
        const auto &mnames = cfg_it->second.motor_state_names;
        const auto nm = mnames.size();
        pe.msg.motor_names.assign(mnames.begin(), mnames.end());
        pe.msg.motor_positions.resize(nm, 0.0);
        pe.msg.motor_velocities.resize(nm, 0.0);
        pe.msg.motor_efforts.resize(nm, 0.0);
      }
      device_state_log_publishers_[entry.topic_name] = std::move(pe);
      log_pub("device_state_log");
      return;
    }
    case urtc::PublishRole::kDeviceSensorLog: {
      if (device_sensor_log_publishers_.count(entry.topic_name) > 0) {
        return;
      }
      TypedPublisherEntry<rtc_msgs::msg::DeviceSensorLog> pe;
      pe.publisher = create_publisher<rtc_msgs::msg::DeviceSensorLog>(
          entry.topic_name, cmd_qos);
      pe.msg.sensor_names.assign(sensor_names.begin(), sensor_names.end());
      // Pre-allocate to max sensor channels to avoid resize() in publish loop
      const auto max_sc = static_cast<std::size_t>(urtc::kMaxSensorChannels);
      pe.msg.sensor_data_raw.resize(max_sc, 0);
      pe.msg.sensor_data.resize(max_sc, 0);
      const auto max_inf = static_cast<std::size_t>(urtc::kMaxInferenceValues);
      pe.msg.inference_output.resize(max_inf, 0.0f);
      device_sensor_log_publishers_[entry.topic_name] = std::move(pe);
      log_pub("device_sensor_log");
      return;
    }
    case urtc::PublishRole::kGraspState: {
      if (grasp_state_publishers_.count(entry.topic_name) > 0) {
        return;
      }
      TypedPublisherEntry<rtc_msgs::msg::GraspState> pe;
      pe.publisher = create_publisher<rtc_msgs::msg::GraspState>(
          entry.topic_name, rclcpp::QoS{10});
      pe.msg.fingertip_names.assign(sensor_names.begin(), sensor_names.end());
      // Pre-allocate to max fingertips to avoid resize() in publish loop
      const auto max_ft = static_cast<std::size_t>(urtc::kMaxFingertips);
      pe.msg.force_magnitude.resize(max_ft, 0.0f);
      pe.msg.contact_flag.resize(max_ft, 0.0f);
      pe.msg.inference_valid.resize(max_ft, false);
      pe.msg.finger_s.resize(max_ft, 0.0f);
      pe.msg.finger_filtered_force.resize(max_ft, 0.0f);
      pe.msg.finger_force_error.resize(max_ft, 0.0f);
      grasp_state_publishers_[entry.topic_name] = std::move(pe);
      log_pub("grasp_state");
      return;
    }
    case urtc::PublishRole::kToFSnapshot: {
      if (tof_snapshot_publishers_.count(entry.topic_name) > 0) {
        return;
      }
      TypedPublisherEntry<rtc_msgs::msg::ToFSnapshot> pe;
      rclcpp::QoS qos{5};
      qos.best_effort();
      pe.publisher =
          create_publisher<rtc_msgs::msg::ToFSnapshot>(entry.topic_name, qos);
      tof_snapshot_publishers_[entry.topic_name] = std::move(pe);
      log_pub("tof_snapshot");
      return;
    }
    default:
      break;
    }

    // Float64MultiArray publishers (kRos2Command)
    if (topic_publishers_.count(entry.topic_name) > 0) {
      return;
    }

    int data_size = entry.data_size;
    if (data_size <= 0) {
      auto cfg_it2 = device_name_configs_.find(group_name);
      const int device_joints =
          (cfg_it2 != device_name_configs_.end())
              ? static_cast<int>(cfg_it2->second.joint_command_names.size())
              : rtc::kMaxRobotDOF;
      data_size = device_joints;
    }

    rclcpp::QoS ros2_cmd_qos{1};
    if (entry.role == urtc::PublishRole::kRos2Command) {
      ros2_cmd_qos.reliable();
    } else {
      ros2_cmd_qos.best_effort();
    }

    PublisherEntry pe;
    pe.publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
        entry.topic_name, ros2_cmd_qos);
    pe.msg.data.resize(static_cast<std::size_t>(data_size), 0.0);

    if (entry.role == urtc::PublishRole::kRos2Command) {
      auto cfg_it3 = device_name_configs_.find(group_name);
      if (cfg_it3 != device_name_configs_.end()) {
        const auto &state_names = cfg_it3->second.joint_state_names;
        const auto &cmd_names = cfg_it3->second.joint_command_names;
        if (!state_names.empty() && !cmd_names.empty() &&
            state_names != cmd_names) {
          pe.reorder_map.resize(cmd_names.size(), -1);
          for (std::size_t ci = 0; ci < cmd_names.size(); ++ci) {
            for (std::size_t si = 0; si < state_names.size(); ++si) {
              if (cmd_names[ci] == state_names[si]) {
                pe.reorder_map[ci] = static_cast<int>(si);
                break;
              }
            }
          }
          RCLCPP_INFO(get_logger(),
                      "  [%s] kRos2Command reorder map built (%zu → %zu)",
                      group_name.c_str(), state_names.size(), cmd_names.size());
        }
      }
    }

    topic_publishers_[entry.topic_name] = std::move(pe);

    const char *role_str = urtc::PublishRoleToString(entry.role);
    RCLCPP_INFO(get_logger(), "  Publish [%s/%s]: %s (size=%d)",
                group_name.c_str(), role_str, entry.topic_name.c_str(),
                data_size);
  };

  // ── Create publishers for all active device groups ────────────────────────
  for (const auto &tc : controller_topic_configs_) {
    for (const auto &[group_name, group] : tc.groups) {
      if (!active_groups_.contains(group_name))
        continue;
      for (const auto &entry : group.publish) {
        // Controller-owned publishers are created by the controller's own
        // on_configure — CM does not bind them here.
        if (entry.ownership == urtc::TopicOwnership::kController)
          continue;
        create_pub(entry, group_name);
      }
    }
  }

  // ── Digital Twin republishers (RELIABLE, depth 10) ──────────────────────
  {
    rclcpp::QoS dt_qos{10};
    dt_qos.reliable();
    for (const auto &[group_name, slot] : group_slot_map_) {
      std::string dt_topic = "/" + group_name + "/digital_twin/joint_states";
      DigitalTwinEntry dte;
      dte.publisher =
          create_publisher<sensor_msgs::msg::JointState>(dt_topic, dt_qos);
      auto cfg_it = device_name_configs_.find(group_name);
      if (cfg_it != device_name_configs_.end()) {
        const auto &names = cfg_it->second.joint_state_names;
        dte.msg.name.assign(names.begin(), names.end());
        dte.msg.position.resize(names.size(), 0.0);
        dte.msg.velocity.resize(names.size(), 0.0);
        dte.msg.effort.resize(names.size(), 0.0);
      }
      digital_twin_publishers_[dt_topic] = std::move(dte);
      slot_to_dt_topic_[slot] = dt_topic;
      RCLCPP_INFO(get_logger(), "  Digital Twin publish: %s (RELIABLE/10)",
                  dt_topic.c_str());
    }
  }

  // ── Fixed safety publishers (always present, non-lifecycle) ────────────────
  // These use the standalone rclcpp::create_publisher to remain as regular
  // rclcpp::Publisher (not LifecyclePublisher). This ensures E-STOP status
  // and controller name can be published regardless of lifecycle state.
  estop_pub_ = rclcpp::create_publisher<std_msgs::msg::Bool>(
      this->get_node_topics_interface(), "/system/estop_status",
      rclcpp::QoS(10));

  rclcpp::QoS latch_qos{1};
  latch_qos.transient_local();
  active_ctrl_name_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(
      this->get_node_topics_interface(),
      "/" + robot_ns_ + "/active_controller_name", latch_qos);

  current_gains_pub_ =
      rclcpp::create_publisher<std_msgs::msg::Float64MultiArray>(
          this->get_node_topics_interface(), "/" + robot_ns_ + "/current_gains",
          rclcpp::QoS(10));
}

// ── Expose topic configuration as read-only ROS2 parameters ─────────────────
void RtControllerNode::ExposeTopicParameters() {
  for (std::size_t i = 0; i < controllers_.size(); ++i) {
    const auto &tc = controller_topic_configs_[i];
    const std::string prefix =
        "controllers." + std::string(controllers_[i]->Name());

    for (const auto &[group_name, group] : tc.groups) {
      for (const auto &entry : group.subscribe) {
        const std::string param_name = prefix + "." + group_name +
                                       ".subscribe." +
                                       urtc::SubscribeRoleToString(entry.role);
        if (!has_parameter(param_name)) {
          declare_parameter(param_name, entry.topic_name);
        }
      }
      for (const auto &entry : group.publish) {
        const std::string param_name = prefix + "." + group_name + ".publish." +
                                       urtc::PublishRoleToString(entry.role);
        if (!has_parameter(param_name)) {
          declare_parameter(param_name, entry.topic_name);
        }
      }
    }
  }

  // ── Read-only guard: reject any runtime mutation of topic parameters ────
  param_callback_handle_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &params)
          -> rcl_interfaces::msg::SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &p : params) {
          if (p.get_name().rfind("controllers.", 0) == 0) {
            result.successful = false;
            result.reason =
                "Topic parameters are read-only after initialisation";
            return result;
          }
        }
        result.successful = true;
        return result;
      });

  RCLCPP_INFO(get_logger(), "Topic parameters exposed (read-only) — use 'ros2 "
                            "param list' to inspect");
}

// ── Switch helper (aux thread) ──────────────────────────────────────────
//
// Sync sequence (Phase 2 / D-A1):
//   1. precondition: name resolves, target != current, E-STOP idle
//   2. snapshot = BuildDeviceSnapshot(target)
//   3. target.on_activate(prev_state, snapshot)        (hold-init inside)
//   4. atomic store active_controller_idx_ = target    (release)
//   5. wait one RT tick (sleep_for(1.5 * dt))          (OQ-2 = sleep_for)
//   6. previous.on_deactivate(prev_state)              (sets state=Inactive)
//   7. publish /<robot_ns>/active_controller_name      (latched)
//
// The race between step 4 and step 6 is benign by F-3: demo controllers'
// on_deactivate only toggles LifecyclePublishers (which drop internally
// when inactive) and the WBC MPC thread (Pause is idempotent). If a future
// controller introduces shared state cleanup that Compute() depends on,
// step 5 must be upgraded to a cv-based RT-tick acknowledgement.
bool RtControllerNode::SwitchActiveController(const std::string &name,
                                              std::string &message) {
  if (IsGlobalEstopped()) {
    message = "E-STOP active";
    return false;
  }
  const auto it = controller_name_to_idx_.find(name);
  if (it == controller_name_to_idx_.end()) {
    message = "Unknown controller name: " + name;
    return false;
  }
  const int target_idx = it->second;
  if (target_idx < 0 || target_idx >= static_cast<int>(controllers_.size())) {
    message = "Invalid controller index";
    return false;
  }
  const int prev_idx = active_controller_idx_.load(std::memory_order_acquire);
  if (prev_idx == target_idx) {
    message = "Already active: " + name;
    return true; // no-op success
  }

  const auto target_uidx = static_cast<std::size_t>(target_idx);
  const rclcpp_lifecycle::State active_state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

  // Build snapshot only if auto_hold is requested AND state has been
  // received; otherwise pass empty snapshot (controller's on_activate base
  // skips hold init when num_devices == 0).
  rtc::ControllerState snapshot{};
  if (auto_hold_position_) {
    snapshot = BuildDeviceSnapshot(target_uidx);
  }

  // Step 3: activate target.
  const auto rc = ActivateController(target_uidx, active_state, snapshot);
  if (rc != CallbackReturn::SUCCESS) {
    message = "target on_activate failed";
    return false;
  }

  // Step 4: publish new active idx to RT loop.
  active_controller_idx_.store(target_idx, std::memory_order_release);

  // Step 5: let one RT tick observe the new idx before deactivating prev.
  // 1.5 × dt at default 500 Hz ≈ 3 ms. PREEMPT_RT missed deadlines surface
  // as overrun_count_ regression, not as a wrong-controller dispatch.
  const auto dt_us = static_cast<long>(
      1'500'000.0 / (control_rate_ > 0.0 ? control_rate_ : 500.0));
  std::this_thread::sleep_for(std::chrono::microseconds(dt_us));

  // Step 6: deactivate prev (only if it was Active — defensive against
  // CM startup state where prev_idx may be the initial controller that
  // was already activated in CM on_activate).
  if (prev_idx >= 0 && prev_idx < static_cast<int>(controllers_.size())) {
    const auto prev_uidx = static_cast<std::size_t>(prev_idx);
    if (controller_states_[prev_uidx].load(std::memory_order_acquire) == 1) {
      (void)DeactivateController(prev_uidx, active_state);
    }
  }

  // Step 7: publish latched confirm.
  std_msgs::msg::String ctrl_name_msg;
  ctrl_name_msg.data = std::string(controllers_[target_uidx]->Name());
  active_ctrl_name_pub_->publish(ctrl_name_msg);

  RCLCPP_INFO(
      get_logger(), "Switched controller: %s → %s",
      prev_idx >= 0 && prev_idx < static_cast<int>(controllers_.size())
          ? controllers_[static_cast<std::size_t>(prev_idx)]->Name().data()
          : "(none)",
      controllers_[target_uidx]->Name().data());
  message = "ok";
  return true;
}

// ── /rtc_cm/* services (Phase 3) ────────────────────────────────────────
//
// list_controllers — read-only snapshot of controllers_ + controller_states_.
//   Aux thread; no blocking on RT path. Response order matches controllers_.
//
// switch_controller — single-active D-A1 wrapper around
//   SwitchActiveController(name, message). STRICT validates inputs and
//   refuses violations; BEST_EFFORT trims to the first activate target and
//   warns on extras. Timeout field is accepted but currently unused — the
//   underlying switch helper is sync and bounded by sleep_for(1.5×dt) +
//   controller hooks (~ms). M-1 will measure actual latency in Phase 4.
void RtControllerNode::CreateServices() {
  const rmw_qos_profile_t srv_qos = rmw_qos_profile_services_default;

  list_controllers_srv_ = create_service<rtc_msgs::srv::ListControllers>(
      "/rtc_cm/list_controllers",
      [this](const std::shared_ptr<
                 rtc_msgs::srv::ListControllers::Request> /*req*/,
             std::shared_ptr<rtc_msgs::srv::ListControllers::Response> resp) {
        const std::size_t n = controllers_.size();
        resp->controllers.clear();
        resp->controllers.reserve(n);
        const int active_idx =
            active_controller_idx_.load(std::memory_order_acquire);
        for (std::size_t i = 0; i < n; ++i) {
          rtc_msgs::msg::ControllerState cs;
          cs.name = std::string(controllers_[i]->Name());
          const int s = controller_states_[i].load(std::memory_order_acquire);
          cs.state = (s == 1) ? "active" : "inactive";
          cs.is_active = (static_cast<int>(i) == active_idx) && (s == 1);
          cs.type = (i < controller_types_.size()) ? controller_types_[i] : "";
          if (i < controller_topic_configs_.size()) {
            cs.claimed_groups.reserve(
                controller_topic_configs_[i].groups.size());
            for (const auto &[group_name, _g] :
                 controller_topic_configs_[i].groups) {
              cs.claimed_groups.push_back(group_name);
            }
          }
          resp->controllers.push_back(std::move(cs));
        }
      },
      srv_qos, cb_group_aux_);

  switch_controller_srv_ = create_service<rtc_msgs::srv::SwitchController>(
      "/rtc_cm/switch_controller",
      [this](
          const std::shared_ptr<rtc_msgs::srv::SwitchController::Request> req,
          std::shared_ptr<rtc_msgs::srv::SwitchController::Response> resp) {
        const auto strict = req->strictness;
        const bool best_effort =
            strict == rtc_msgs::srv::SwitchController::Request::BEST_EFFORT;

        // Single-active validation (D-A1). STRICT: any multi-target → reject.
        // BEST_EFFORT: trim to first entry, warn on extras.
        if (req->activate_controllers.size() > 1) {
          if (!best_effort) {
            resp->ok = false;
            resp->message = "Single-active: activate_controllers must have ≤ 1 "
                            "entry under STRICT";
            return;
          }
          RCLCPP_WARN(get_logger(),
                      "switch_controller BEST_EFFORT: trimming "
                      "activate_controllers (%zu → 1)",
                      req->activate_controllers.size());
        }
        if (req->deactivate_controllers.size() > 1) {
          if (!best_effort) {
            resp->ok = false;
            resp->message = "Single-active: deactivate_controllers must have ≤ "
                            "1 entry under STRICT";
            return;
          }
          RCLCPP_WARN(get_logger(),
                      "switch_controller BEST_EFFORT: trimming "
                      "deactivate_controllers (%zu → 1)",
                      req->deactivate_controllers.size());
        }

        // No activate target → either pure deactivate (unsupported in
        // single-active) or empty no-op. Empty is a no-op success; pure
        // deactivate is rejected because RT loop always needs an active idx.
        if (req->activate_controllers.empty()) {
          if (!req->deactivate_controllers.empty()) {
            resp->ok = false;
            resp->message =
                "Pure deactivate not supported under single-active (D-A1) — "
                "specify the replacement in activate_controllers";
            return;
          }
          resp->ok = true;
          resp->message = "no-op (empty request)";
          return;
        }

        const std::string &target = req->activate_controllers.front();
        std::string sub_msg;
        const bool ok = SwitchActiveController(target, sub_msg);
        resp->ok = ok;
        if (ok) {
          resp->message = "switched -> " + target +
                          (sub_msg.empty() ? "" : " (" + sub_msg + ")");
        } else {
          resp->message = sub_msg;
        }
      },
      srv_qos, cb_group_aux_);

  RCLCPP_INFO(get_logger(), "Services ready: /rtc_cm/list_controllers, "
                            "/rtc_cm/switch_controller");
}
