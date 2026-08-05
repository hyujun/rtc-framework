#include "integrated_bringup/backends/mujoco_native_backend.hpp"

#include "rtc_controller_manager/device_backend_registry.hpp"
#include <rtc_base/tracing/trace_scope.hpp>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rtc {

void MujocoNativeBackend::Configure(rclcpp_lifecycle::LifecycleNode* node,
                                    const DeviceBackendConfig& config,
                                    rclcpp::CallbackGroup::SharedPtr state_cb_group) {
  config_ = config;

  if (config_.command_topic.empty() && config_.state_topic.empty()) {
    return;  // Nothing to bind; degenerate config — caller logs.
  }

  // joint_command_names → command publisher layout (when provided). Pre-size
  // values/feedforward once here so WriteCommand (RT thread) only fills them
  // in place — no allocation on the actuator-publish path.
  if (!config_.joint_command_names.empty()) {
    cmd_msg_.joint_names = config_.joint_command_names;
    cmd_msg_.values.assign(config_.joint_command_names.size(), 0.0);
    cmd_msg_.feedforward.assign(config_.joint_command_names.size(), 0.0);
  }
  cmd_msg_.command_type = "position";

  rclcpp::QoS qos{1};
  qos.best_effort();

  // RT inbound boundary: route the state-lane sub onto the rt_callback
  // callback group (FIFO 70) so RT loop reads see fresh data without
  // bouncing through the non-RT default executor.
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = state_cb_group;

  if (!config_.state_topic.empty()) {
    state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        config_.state_topic, qos,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnJointState(std::move(msg)); },
        sub_opts);
  }
  if (!config_.command_topic.empty()) {
    cmd_pub_ = node->create_publisher<rtc_msgs::msg::JointCommand>(config_.command_topic, qos);
  }

  // Fingertip wrench lane (Y2c: read via ros 2 param nested key so we avoid
  // touching rtc_base/DeviceBackendBinding). sim.yaml lists topics at
  // `devices.<group>.backend.fingertip_wrench_topics`; absent → no subs.
  const std::string param_prefix = "devices." + config_.group_name + ".backend";
  const std::string topics_key = param_prefix + ".fingertip_wrench_topics";
  const std::string miss_key = param_prefix + ".max_consecutive_missed_ticks";

  std::vector<std::string> wrench_topics;
  if (node->has_parameter(topics_key)) {
    wrench_topics = node->get_parameter(topics_key).as_string_array();
  } else {
    wrench_topics =
        node->declare_parameter<std::vector<std::string>>(topics_key, std::vector<std::string>{});
  }

  int max_miss = 5;
  if (node->has_parameter(miss_key)) {
    max_miss = static_cast<int>(node->get_parameter(miss_key).as_int());
  } else {
    max_miss = static_cast<int>(node->declare_parameter<int>(miss_key, max_miss));
  }
  max_missed_ticks_ = std::max(1, max_miss);

  if (wrench_topics.empty()) {
    // Documented fallback: HasSensorState() still reports true, but
    // ReadSensorState publishes num_inference_groups=0. Surface once so
    // mis-configuration is visible in launch logs.
    RCLCPP_INFO(logger_, "[mujoco_native:%s] fingertip wrench lane disabled — no topics configured",
                config_.group_name.c_str());
  } else {
    rclcpp::SubscriptionOptions wrench_opts;
    // Reuse the RT callback group: callbacks run alongside JointState on the
    // same FIFO 70 thread the executor wires up. Mismatched-QoS warnings are
    // logged by rclcpp; we keep BEST_EFFORT to mirror the simulator publisher.
    // SPSC guard: OnWrench's load-modify-store on sensor_mirror_ is safe only
    // when this cb_group is MutuallyExclusive. WARN (do not abort) so dev
    // builds can opt into Reentrant for diagnosis with the trade-off visible.
    if (state_cb_group && state_cb_group->type() != rclcpp::CallbackGroupType::MutuallyExclusive) {
      RCLCPP_WARN(logger_,
                  "[mujoco_native:%s] wrench cb_group is not MutuallyExclusive — "
                  "OnWrench SPSC guarantee is broken; concurrent callbacks may "
                  "lose updates via SeqLock load-modify-store race",
                  config_.group_name.c_str());
    }
    wrench_opts.callback_group = state_cb_group;
    const std::size_t n_topics =
        std::min(wrench_topics.size(), static_cast<std::size_t>(kMaxSensorGroups));
    wrench_subs_.reserve(n_topics);
    for (std::size_t i = 0; i < n_topics; ++i) {
      const int idx = static_cast<int>(i);
      wrench_subs_.push_back(node->create_subscription<geometry_msgs::msg::WrenchStamped>(
          wrench_topics[i], rclcpp::SensorDataQoS().keep_last(1),
          [this, idx](const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
            OnWrench(idx, *msg);
          },
          wrench_opts));
    }
  }
}

void MujocoNativeBackend::OnWrench(int finger_idx,
                                   const geometry_msgs::msg::WrenchStamped& msg) noexcept {
  // L2 under the rt_callback executor's ros2:callback_* (L1) — the fingertip
  // wrench SeqLock write on the rt_callback lane.
  RTC_TRACE_SCOPE("MujocoNativeBackend::OnWrench");
  // This is a sensor lane on the same FIFO 70 thread as the joint lane, so it
  // belongs in the same CSV: #349 reads the sum of t_total_us as slot 2's
  // load, and a group with fingertip_wrench_topics configured runs one of
  // these per fingertip per sample. Leaving it out understated that load by
  // however many fingertip lanes were enabled.
  StateLaneTimingScope timing_scope(*this);
  if (finger_idx < 0 || finger_idx >= static_cast<int>(kMaxSensorGroups)) {
    return;
  }
  const double fx_d = msg.wrench.force.x;
  const double fy_d = msg.wrench.force.y;
  const double fz_d = msg.wrench.force.z;
  if (!std::isfinite(fx_d) || !std::isfinite(fy_d) || !std::isfinite(fz_d)) {
    // Drop NaN/Inf — RT reader keeps last valid value. Throttled WARN so a
    // flooding bad publisher does not spam logs but the failure mode stays
    // visible (silent drop made debug "why did force freeze" too hard).
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
                         "[mujoco_native:%s] non-finite wrench on fingertip %d, dropping",
                         config_.group_name.c_str(), finger_idx);
    return;
  }

  auto mirror = sensor_mirror_.Load();
  const auto fidx = static_cast<std::size_t>(finger_idx);
  auto& tip = mirror.tips[fidx];
  tip.fx = static_cast<float>(fx_d);
  tip.fy = static_cast<float>(fy_d);
  tip.fz = static_cast<float>(fz_d);
  tip.received_at_least_once = true;
  if (finger_idx + 1 > mirror.num_tips) {
    mirror.num_tips = finger_idx + 1;
  }
  sensor_mirror_.Store(mirror);
  wrench_seq_[fidx].fetch_add(1, std::memory_order_release);
}

void MujocoNativeBackend::ReadSensorState(DeviceStateCache& cache) noexcept {
  // L3 under CM::ReadDeviceState — RT-tick fingertip-wrench mirror load (not the
  // OnWrench callback lane, which is the non-RT write side).
  RTC_TRACE_SCOPE("MujocoNativeBackend::ReadSensorState");
  const auto mirror = sensor_mirror_.Load();
  cache.num_inference_groups = mirror.num_tips;

  for (int f = 0; f < mirror.num_tips; ++f) {
    const auto fu = static_cast<std::size_t>(f);
    const uint64_t cur_seq = wrench_seq_[fu].load(std::memory_order_acquire);

    if (cur_seq != rt_last_seen_seq_[fu]) {
      rt_last_seen_seq_[fu] = cur_seq;
      rt_miss_count_[fu] = 0;
    } else if (rt_miss_count_[fu] < max_missed_ticks_) {
      ++rt_miss_count_[fu];  // saturates at max_missed_ticks_ so no overflow.
    }

    const auto& tip = mirror.tips[fu];
    const bool fresh = tip.received_at_least_once && (rt_miss_count_[fu] < max_missed_ticks_);
    cache.inference_enable[fu] = fresh;

    // Stride 7 mirror — slot 0 contact_flag / 4..6 displacement intentionally
    // 0-filled (controller does not consume them; udp_hand backend remains
    // the source for those lanes). Force values are preserved across stale
    // ticks so the controller can keep using the last known fx/fy/fz.
    const std::size_t base =
        static_cast<std::size_t>(f) * static_cast<std::size_t>(kInferenceStride);
    cache.inference_data[base + 0] = 0.0F;
    cache.inference_data[base + 1] = tip.fx;
    cache.inference_data[base + 2] = tip.fy;
    cache.inference_data[base + 3] = tip.fz;
    cache.inference_data[base + 4] = 0.0F;
    cache.inference_data[base + 5] = 0.0F;
    cache.inference_data[base + 6] = 0.0F;
  }
}

void MujocoNativeBackend::Activate() {
  if (cmd_pub_) {
    cmd_pub_->on_activate();
  }
}

void MujocoNativeBackend::Deactivate() {
  if (cmd_pub_) {
    cmd_pub_->on_deactivate();
  }
}

void MujocoNativeBackend::OnJointState(sensor_msgs::msg::JointState::SharedPtr msg) {
  // L2 under the rt_callback executor's ros2:callback_* (L1) — the joint-state
  // decode + SeqLock write on the rt_callback lane.
  RTC_TRACE_SCOPE("MujocoNativeBackend::OnJointState");
  StateLaneTimingScope timing_scope(*this);
  if (msg->position.empty())
    return;

  // One-shot reorder build from the first message that carries names —
  // fixed-capacity, allocation-free (RT callback rule, issue #156). Reference
  // order = joint_command_names (the device-canonical order).
  if (!state_reorder_built_.load(std::memory_order_acquire) && !msg->name.empty()) {
    BuildJointStateReorder(msg->name, config_.joint_command_names, state_reorder_);
    state_reorder_built_.store(true, std::memory_order_release);
  }

  auto ds = state_cache_.Load();
  WriteJointStateToCache(*msg, state_reorder_, ds);
  state_cache_.Store(ds);

  last_state_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count(),
                       std::memory_order_release);

  NotifyStateReady();
}

bool MujocoNativeBackend::ReadState(DeviceStateCache& cache) noexcept {
  // L3 under CM::ReadDeviceState — RT-tick SeqLock load.
  RTC_TRACE_SCOPE("MujocoNativeBackend::ReadState");
  cache = state_cache_.Load();
  return cache.valid;
}

void MujocoNativeBackend::WriteCommand(const PublishSnapshot::GroupCommandSlot& slot,
                                       CommandType command_type) noexcept {
  // L3 under CM::WriteCommand — RT-tick actuator publish.
  RTC_TRACE_SCOPE("MujocoNativeBackend::WriteCommand");
  if (!cmd_pub_)
    return;
  const int nc = slot.num_channels;
  if (nc <= 0)
    return;  // Skip until the controller has output.

  const bool pd_ff = (command_type == CommandType::kPdFeedforward);
  cmd_msg_.command_type = CommandTypeToString(command_type);

  // Stamp the JointCommand header from the RT-loop wall-clock capture (integer
  // div/mod only — RT-safe). slot.stamp_ns is CLOCK_REALTIME ns so the topic
  // shares the ROS wall time axis; 0 (unset) yields a zero stamp harmlessly.
  cmd_msg_.header.stamp.sec = static_cast<int32_t>(slot.stamp_ns / 1'000'000'000LL);
  cmd_msg_.header.stamp.nanosec = static_cast<uint32_t>(slot.stamp_ns % 1'000'000'000LL);

  // Direct copy: slot.commands is always in joint_command_names order (same
  // order the message labels carry) — wire-order differences are the
  // receiver's job (mujoco_simulator_node remaps by joint_names).
  const std::size_t n = std::min(static_cast<std::size_t>(nc), cmd_msg_.values.size());
  const std::size_t nff = std::min(n, cmd_msg_.feedforward.size());
  for (std::size_t i = 0; i < n; ++i) {
    cmd_msg_.values[i] = slot.commands[i];
    if (i < nff)
      cmd_msg_.feedforward[i] = pd_ff ? slot.feedforward[i] : 0.0;
  }
  cmd_pub_->publish(cmd_msg_);
  cmd_published_ = true;
}

void MujocoNativeBackend::WriteSafeCommand() noexcept {
  // L3 under CM::WriteCommand — same RT tick, same publisher.
  RTC_TRACE_SCOPE("MujocoNativeBackend::WriteSafeCommand");
  // Nothing published yet means there is no known-good command to fall back
  // on, and this backend has no disable to reach for instead. Inventing a
  // value here would be worse than the silence: 0.0 on a position lane is
  // "go to the origin", which is the failure #196 spent a whole phase
  // keeping off the wire.
  if (!cmd_pub_ || !cmd_published_) {
    return;
  }
  // Re-publish the last command known to have gone out. Physically this
  // leaves the device where the silent path left it — the difference is that
  // the intent is now explicit on the wire and the lane stays live.
  cmd_pub_->publish(cmd_msg_);
}

}  // namespace rtc

RTC_REGISTER_DEVICE_BACKEND(mujoco_native, std::make_unique<rtc::MujocoNativeBackend>())
