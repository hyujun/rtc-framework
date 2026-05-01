// ── Publish offload thread (SPSC drain + ROS2 publish) ───────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include <rtc_base/threading/thread_utils.hpp>

#include <poll.h>        // poll
#include <sched.h>       // sched_yield (fallback if eventfd unavailable)
#include <sys/eventfd.h> // eventfd_read

#include <algorithm>

namespace urtc = rtc;

// ── Publish offload thread
// ────────────────────────────────────────────────────
//
// Drains the SPSC publish buffer and performs all ROS2 publish() calls.
// Runs on a non-RT core (Core 5/6, SCHED_OTHER nice -3).
// All DDS serialization, string allocation, and sendto() syscalls happen here,
// keeping the RT path free of unbounded-latency operations.

void RtControllerNode::PublishLoopEntry(const urtc::ThreadConfig &cfg) {
  static_cast<void>(urtc::ApplyThreadConfig(cfg));
  publish_running_.store(true, std::memory_order_release);

  urtc::PublishSnapshot snap{};

  while (publish_running_.load(std::memory_order_acquire) && rclcpp::ok()) {
    if (!publish_buffer_.Pop(snap)) {
      // Wait for RT thread signal via eventfd (or 1ms polling fallback)
      if (publish_eventfd_ >= 0) {
        struct pollfd pfd {};
        pfd.fd = publish_eventfd_;
        pfd.events = POLLIN;
        poll(&pfd, 1, 1); // 1ms timeout
        if (pfd.revents & POLLIN) {
          eventfd_t val{};
          static_cast<void>(eventfd_read(publish_eventfd_, &val));
        }
      } else {
        sched_yield();
      }
      continue;
    }

    const auto &active_tc = controller_topic_configs_[static_cast<std::size_t>(
        snap.active_controller_idx)];

    // Shared timestamp for JointCommand messages
    const auto sec = static_cast<int32_t>(snap.stamp_ns / 1'000'000'000L);
    const auto nsec = static_cast<uint32_t>(snap.stamp_ns % 1'000'000'000L);
    const char *cmd_type_str = (snap.command_type == urtc::CommandType::kTorque)
                                   ? "torque"
                                   : "position";

    // Helper: publish a single topic entry from snapshot data
    auto publish_entry = [&](const urtc::PublishTopicEntry &pt,
                             std::size_t group_idx) {
      const auto &gc = snap.group_commands[group_idx];
      const int nc = gc.num_channels;

      switch (pt.role) {
      case urtc::PublishRole::kJointCommand: {
        // Skip publishing when the controller has no output for this
        // device (nc=0). This prevents sending zero-filled commands
        // before the device state is known (e.g. hand not yet valid).
        if (nc <= 0) {
          return;
        }
        auto jc_it = joint_command_publishers_.find(pt.topic_name);
        if (jc_it == joint_command_publishers_.end()) {
          return;
        }
        auto &jce = jc_it->second;
        jce.msg.header.stamp.sec = sec;
        jce.msg.header.stamp.nanosec = nsec;
        jce.msg.command_type = cmd_type_str;
        const auto n =
            std::min(static_cast<std::size_t>(nc), jce.msg.values.size());
        if (!jce.reorder_map.empty()) {
          // Reorder from joint_state_names order → joint_command_names order
          for (std::size_t i = 0; i < n; ++i) {
            const int src =
                (i < jce.reorder_map.size()) ? jce.reorder_map[i] : -1;
            jce.msg.values[i] = (src >= 0 && src < nc)
                                    ? gc.commands[static_cast<std::size_t>(src)]
                                    : 0.0;
          }
        } else {
          for (std::size_t i = 0; i < n; ++i) {
            jce.msg.values[i] = gc.commands[i];
          }
        }
        jce.publisher->publish(jce.msg);
        return;
      }
      case urtc::PublishRole::kRos2Command: {
        auto it = topic_publishers_.find(pt.topic_name);
        if (it == topic_publishers_.end()) {
          return;
        }
        auto &pe = it->second;
        const auto n =
            std::min(static_cast<std::size_t>(nc), pe.msg.data.size());
        if (!pe.reorder_map.empty()) {
          // Reorder from joint_state_names order → joint_command_names order
          for (std::size_t i = 0; i < n; ++i) {
            const int src =
                (i < pe.reorder_map.size()) ? pe.reorder_map[i] : -1;
            pe.msg.data[i] = (src >= 0 && src < nc)
                                 ? gc.commands[static_cast<std::size_t>(src)]
                                 : 0.0;
          }
        } else {
          for (std::size_t i = 0; i < n; ++i) {
            pe.msg.data[i] = gc.commands[i];
          }
        }
        pe.publisher->publish(pe.msg);
        return;
      }
      case urtc::PublishRole::kGuiPosition: {
        auto it = gui_position_publishers_.find(pt.topic_name);
        if (it == gui_position_publishers_.end()) {
          return;
        }
        auto &m = it->second.msg;
        m.header.stamp.sec = sec;
        m.header.stamp.nanosec = nsec;
        // Use actual_num_channels (from device state) instead of nc
        // (from controller output) so that GUI always reflects the
        // latest device state even when the controller skips the
        // device (e.g., E-Stop, hand not yet valid in controller).
        const auto n_actual =
            std::min(static_cast<std::size_t>(gc.actual_num_channels),
                     m.joint_positions.size());
        for (std::size_t i = 0; i < n_actual; ++i) {
          m.joint_positions[i] = gc.actual_positions[i];
        }
        // actual_task_positions is a snapshot-level field containing only the
        // robot arm FK result. Copy it only for the robot group (index 0);
        // other devices (e.g. hand) have no FK and should report zeros.
        if (group_idx == 0) {
          std::copy(snap.actual_task_positions.begin(),
                    snap.actual_task_positions.end(), m.task_positions.begin());
        } else {
          m.task_positions.fill(0.0);
        }
        it->second.publisher->publish(m);
        return;
      }
      case urtc::PublishRole::kRobotTarget: {
        auto it = robot_target_publishers_.find(pt.topic_name);
        if (it == robot_target_publishers_.end()) {
          return;
        }
        auto &m = it->second.msg;
        m.header.stamp.sec = sec;
        m.header.stamp.nanosec = nsec;
        const auto n =
            std::min(static_cast<std::size_t>(nc), m.joint_target.size());
        for (std::size_t i = 0; i < n; ++i) {
          m.joint_target[i] = gc.goal_positions[i];
        }
        std::copy(snap.task_goals[group_idx].begin(),
                  snap.task_goals[group_idx].end(), m.task_target.begin());
        it->second.publisher->publish(m);
        return;
      }
      // (Phase C: kDeviceStateLog / kDeviceSensorLog publish branches
      // removed — controller-owned ControllerLogSet writes the same data
      // directly to <session>/controllers/<key>/*.csv.)
      case urtc::PublishRole::kGraspState: {
        auto it = grasp_state_publishers_.find(pt.topic_name);
        if (it == grasp_state_publishers_.end()) {
          return;
        }
        auto &m = it->second.msg;
        m.header.stamp.sec = sec;
        m.header.stamp.nanosec = nsec;
        const auto &gs = gc.grasp_state;
        const auto nf = static_cast<std::size_t>(std::min(
            gs.num_fingertips, static_cast<int>(urtc::kMaxFingertips)));
        for (std::size_t i = 0; i < nf; ++i) {
          m.force_magnitude[i] = gs.force_magnitude[i];
          m.contact_flag[i] = gs.contact_flag[i];
          m.inference_valid[i] = gs.inference_valid[i];
        }
        m.num_active_contacts = gs.num_active_contacts;
        m.max_force = gs.max_force;
        m.grasp_detected = gs.grasp_detected;
        m.force_threshold = gs.force_threshold;
        m.min_fingertips = gs.min_fingertips_for_grasp;
        // Force-PI grasp controller fields
        m.grasp_phase = gs.grasp_phase;
        m.grasp_target_force = gs.grasp_target_force;
        for (std::size_t i = 0; i < nf; ++i) {
          m.finger_s[i] = gs.finger_s[i];
          m.finger_filtered_force[i] = gs.finger_filtered_force[i];
          m.finger_force_error[i] = gs.finger_force_error[i];
        }
        it->second.publisher->publish(m);
        return;
      }
      case urtc::PublishRole::kToFSnapshot: {
        if (!gc.tof_snapshot.populated) {
          return;
        }
        auto it = tof_snapshot_publishers_.find(pt.topic_name);
        if (it == tof_snapshot_publishers_.end()) {
          return;
        }
        auto &m = it->second.msg;
        m.stamp.sec = sec;
        m.stamp.nanosec = nsec;
        const auto &ts = gc.tof_snapshot;
        const int total_sensors =
            std::min(ts.num_fingers * ts.sensors_per_finger,
                     static_cast<int>(m.distances.size()));
        const int num_poses =
            std::min(ts.num_fingers, static_cast<int>(m.tip_poses.size()));
        for (int i = 0; i < total_sensors; ++i) {
          const auto ui = static_cast<std::size_t>(i);
          m.distances[ui] = ts.distances[ui];
          m.valid[ui] = ts.valid[ui];
        }
        for (int f = 0; f < num_poses; ++f) {
          const auto fi = static_cast<std::size_t>(f);
          const auto &src = ts.tip_poses[fi];
          auto &dst = m.tip_poses[fi];
          dst.position.x = src.position[0];
          dst.position.y = src.position[1];
          dst.position.z = src.position[2];
          dst.orientation.w = src.quaternion[0];
          dst.orientation.x = src.quaternion[1];
          dst.orientation.y = src.quaternion[2];
          dst.orientation.z = src.quaternion[3];
        }
        it->second.publisher->publish(m);
        return;
      }
      default:
        break;
      }
    };

    // Publish all device groups uniformly (manager-owned entries only —
    // controller-owned entries are skipped; the controller forwards them
    // via PublishNonRtSnapshot below).
    std::size_t group_idx = 0;
    for ([[maybe_unused]] const auto &[group_name, group] : active_tc.groups) {
      for (const auto &pt : group.publish) {
        if (pt.ownership == urtc::TopicOwnership::kController)
          continue;
        publish_entry(pt, group_idx);
      }
      ++group_idx;
    }

    // Forward the snapshot to the active controller so it can publish its
    // own (controller-owned) topics via LifecyclePublishers it created in
    // on_configure / activated in on_activate.  Index is read from the
    // snapshot (fixed by the RT loop this tick) to stay consistent with
    // active_tc above.
    const auto aidx = static_cast<std::size_t>(snap.active_controller_idx);
    if (aidx < controllers_.size() && controllers_[aidx]) {
      controllers_[aidx]->PublishNonRtSnapshot(snap);
    }
  }
}

void RtControllerNode::StartPublishLoop(const urtc::ThreadConfig &pub_cfg) {
  publish_thread_ =
      std::jthread([this, pub_cfg]() { PublishLoopEntry(pub_cfg); });
}

void RtControllerNode::StopPublishLoop() {
  publish_running_.store(false, std::memory_order_release);
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

void RtControllerNode::PublishEstopStatus(bool estopped) {
  std_msgs::msg::Bool msg;
  msg.data = estopped;
  estop_pub_->publish(msg);
}
