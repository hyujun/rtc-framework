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
    // GuardStateLane: CM drains this lane before destroying the backend (#941).
    state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        config_.state_topic, qos,
        GuardStateLane(
            [this](sensor_msgs::msg::JointState::SharedPtr msg) { OnJointState(std::move(msg)); }),
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

  // SPSC guard shared by the two optional SeqLock-writing lanes below (wrench:
  // load-modify-store; base: whole-struct Store that still needs a single
  // writer). Each lane WARNs (does not abort) only when it is actually
  // configured, so dev builds can opt into Reentrant for diagnosis with the
  // trade-off visible and a joint-only group is not told about lanes it lacks.
  const bool single_writer_group =
      !state_cb_group || state_cb_group->type() == rclcpp::CallbackGroupType::MutuallyExclusive;

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
    // when this cb_group is MutuallyExclusive.
    if (!single_writer_group) {
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
          GuardStateLane([this, idx](const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
            OnWrench(idx, *msg);
          }),
          wrench_opts));
    }
  }

  // Floating-base and IMU lanes: `devices.<group>.backend.base_state.*` and
  // `.imu_state.*`. Same parameter channel as the wrench topics; a lane whose
  // `sensor_topic` is absent is not claimed.
  const auto read_string = [node](const std::string& full) {
    if (node->has_parameter(full)) {
      return node->get_parameter(full).as_string();
    }
    return node->declare_parameter<std::string>(full, std::string{});
  };
  const auto read_string_array = [node](const std::string& full) {
    if (node->has_parameter(full)) {
      return node->get_parameter(full).as_string_array();
    }
    return node->declare_parameter<std::vector<std::string>>(full, std::vector<std::string>{});
  };
  // Each lane has its own tick budget: its topic need not run at the wrench
  // lane's rate. Defaults to the shared value so a bringup that sets only
  // `max_consecutive_missed_ticks` keeps one number for all.
  const auto read_budget = [node, this](const std::string& full) {
    int miss = max_missed_ticks_;
    if (node->has_parameter(full)) {
      miss = static_cast<int>(node->get_parameter(full).as_int());
    } else {
      miss = static_cast<int>(node->declare_parameter<int>(full, miss));
    }
    return std::max(1, miss);
  };
  // Given in the node's parameters at all, declared or not — for keys this
  // backend no longer reads but must not let pass in silence.
  const auto given = [node](const std::string& full) {
    if (node->has_parameter(full)) {
      return true;
    }
    const auto& overrides = node->get_node_parameters_interface()->get_parameter_overrides();
    return overrides.find(full) != overrides.end();
  };

  const std::string base_prefix = param_prefix + ".base_state.";
  const std::string base_topic = read_string(base_prefix + "sensor_topic");
  base_max_missed_ticks_ = read_budget(base_prefix + "max_consecutive_missed_ticks");
  base_names_.pos = read_string(base_prefix + "pos");
  base_names_.quat = read_string(base_prefix + "quat");
  base_names_.linvel = read_string(base_prefix + "linvel");
  base_names_.angvel = read_string(base_prefix + "angvel");

  const std::string imu_prefix = param_prefix + ".imu_state.";
  const std::string imu_topic = read_string(imu_prefix + "sensor_topic");
  imu_max_missed_ticks_ = read_budget(imu_prefix + "max_consecutive_missed_ticks");
  const std::vector<std::string> site_names = read_string_array(imu_prefix + "site_names");
  const std::vector<std::string> gyro_names = read_string_array(imu_prefix + "gyro");
  const std::vector<std::string> accel_names = read_string_array(imu_prefix + "accel");

  if (given(base_prefix + "imu_gyro") || given(base_prefix + "imu_accel")) {
    // The IMU left the base lane. A profile that still lists it there meant
    // the IMU to be read; claiming the base alone would lose it without a
    // word, so neither lane is claimed until the lists move to imu_state.
    RCLCPP_ERROR(logger_,
                 "[mujoco_native:%s] base_state.imu_gyro / imu_accel are no longer read — move "
                 "the IMU lists to imu_state.{site_names, gyro, accel}; floating-base and IMU "
                 "lanes NOT claimed",
                 config_.group_name.c_str());
    return;
  }

  bool base_ok = !base_topic.empty();
  if (base_ok && (base_names_.pos.empty() || base_names_.quat.empty() ||
                  base_names_.linvel.empty() || base_names_.angvel.empty())) {
    // Refused at configure rather than discovered per message: a lane that
    // can never become valid would otherwise look like a quiet publisher.
    RCLCPP_ERROR(logger_,
                 "[mujoco_native:%s] base_state.sensor_topic is set but pos/quat/linvel/angvel "
                 "sensor names are not all given — floating-base lane NOT claimed",
                 config_.group_name.c_str());
    base_ok = false;
  }

  bool imu_ok = !imu_topic.empty();
  if (imu_ok) {
    // Every way the lists can fail to describe a site refuses the lane (and
    // only this lane): a truncated or ambiguous list would hand a consumer a
    // site index that does not mean what its configuration says.
    const std::size_t n = site_names.size();
    const char* why = nullptr;
    if (n == 0) {
      why = "site_names is empty";
    } else if (n > static_cast<std::size_t>(kMaxImuSites)) {
      why = "more sites than DeviceState carries";
    } else if (gyro_names.size() != n || accel_names.size() != n) {
      why = "gyro / accel are not one name per site";
    } else {
      for (std::size_t i = 0; i < n && why == nullptr; ++i) {
        if (site_names[i].empty() || gyro_names[i].empty() || accel_names[i].empty()) {
          why = "a site, gyro or accel name is empty";
        }
        for (std::size_t j = 0; j < i && why == nullptr; ++j) {
          if (site_names[j] == site_names[i]) {
            why = "a site name is listed twice";
          }
        }
      }
    }
    if (why != nullptr) {
      RCLCPP_ERROR(logger_,
                   "[mujoco_native:%s] imu_state: %s (site_names %zu, gyro %zu, accel %zu, at "
                   "most %d sites) — IMU lane NOT claimed",
                   config_.group_name.c_str(), why, n, gyro_names.size(), accel_names.size(),
                   kMaxImuSites);
      imu_ok = false;
    } else {
      imu_names_.num_sites = static_cast<int>(n);
      for (std::size_t i = 0; i < n; ++i) {
        imu_names_.site[i] = site_names[i];
        imu_names_.gyro[i] = gyro_names[i];
        imu_names_.accel[i] = accel_names[i];
      }
    }
  }

  if (!base_ok && !imu_ok) {
    return;  // Fixed-base group without an IMU (the default): nothing more to bind.
  }
  if (!single_writer_group) {
    RCLCPP_WARN(logger_,
                "[mujoco_native:%s] base_state / imu_state cb_group is not MutuallyExclusive — "
                "the mirrors' single-writer SeqLock guarantee is broken; concurrent callbacks "
                "may tear them",
                config_.group_name.c_str());
  }
  base_sensor_idx_.fill(-1);
  imu_sensor_idx_.fill(-1);
  rclcpp::SubscriptionOptions lane_opts;
  lane_opts.callback_group = state_cb_group;  // rt_callback lane, like the subs above.
  const auto subscribe = [&](const std::string& topic, bool decode_base, bool decode_imu) {
    return node->create_subscription<rtc_msgs::msg::SimSensorState>(
        topic, qos,
        GuardStateLane(
            [this, decode_base, decode_imu](rtc_msgs::msg::SimSensorState::ConstSharedPtr msg) {
              OnSimSensorState(*msg, decode_base, decode_imu);
            }),
        lane_opts);
  };
  const bool shared = base_ok && imu_ok && base_topic == imu_topic;
  if (shared) {
    base_sub_ = subscribe(base_topic, true, true);
  } else {
    if (base_ok) {
      base_sub_ = subscribe(base_topic, true, false);
    }
    if (imu_ok) {
      imu_sub_ = subscribe(imu_topic, false, true);
    }
  }
  base_lane_enabled_ = base_ok;
  imu_lane_enabled_ = imu_ok;
  if (base_ok) {
    RCLCPP_INFO(logger_, "[mujoco_native:%s] floating-base lane on '%s'",
                config_.group_name.c_str(), base_topic.c_str());
  }
  if (imu_ok) {
    RCLCPP_INFO(logger_, "[mujoco_native:%s] IMU lane on '%s' (%d site%s%s)",
                config_.group_name.c_str(), imu_topic.c_str(), imu_names_.num_sites,
                imu_names_.num_sites == 1 ? "" : "s", shared ? ", shared subscription" : "");
  }
}

namespace {

// Index of `name` in `msg.sensors`, trying `hint` first. The simulator
// publishes its sensors in a fixed order, so after the first message the hint
// is right and the lookup is one string compare per name; a layout change
// (or a first message) falls back to the linear scan. -1 when absent. No
// allocation either way, so it can run on the rt_callback lane.
[[nodiscard]] int FindSensor(const rtc_msgs::msg::SimSensorState& msg, const std::string& name,
                             int hint) noexcept {
  const auto n = static_cast<int>(msg.sensors.size());
  if (hint >= 0 && hint < n && msg.sensors[static_cast<std::size_t>(hint)].name == name) {
    return hint;
  }
  for (int i = 0; i < n; ++i) {
    if (msg.sensors[static_cast<std::size_t>(i)].name == name) {
      return i;
    }
  }
  return -1;
}

// Copies the `Dim` values of sensor `idx` out. False when absent (idx < 0),
// of another dimension, or non-finite — the three ways a configured sensor can
// fail to be the quantity the lane claims it is.
template <std::size_t Dim>
[[nodiscard]] bool CopySensorAt(const rtc_msgs::msg::SimSensorState& msg, int idx,
                                std::array<double, Dim>& out, bool& non_finite) noexcept {
  if (idx < 0) {
    return false;
  }
  const auto& s = msg.sensors[static_cast<std::size_t>(idx)];
  if (s.values.size() != Dim) {
    return false;
  }
  for (std::size_t i = 0; i < Dim; ++i) {
    if (!std::isfinite(s.values[i])) {
      non_finite = true;
      return false;
    }
    out[i] = s.values[i];
  }
  return true;
}

}  // namespace

void MujocoNativeBackend::OnSimSensorState(const rtc_msgs::msg::SimSensorState& msg,
                                           bool decode_base, bool decode_imu) noexcept {
  // L2 under the rt_callback executor's ros2:callback_* (L1) — the base / IMU
  // decode + SeqLock writes on the rt_callback lane. Same CSV as the joint and
  // wrench lanes (#349): it is load on the same FIFO thread.
  RTC_TRACE_SCOPE("MujocoNativeBackend::OnSimSensorState");
  StateLaneTimingScope timing_scope(*this);
  // One receipt time per message, for both lanes: a consumer comparing the
  // base and IMU stamps of one message must see them equal.
  const int64_t recv_steady_ns = rtc::SteadyNowNs();
  // Both decoders run whatever the first one decided — no early return.
  if (decode_base) {
    static_cast<void>(DecodeBase(msg, recv_steady_ns));
  }
  if (decode_imu) {
    static_cast<void>(DecodeImu(msg, recv_steady_ns));
  }
}

bool MujocoNativeBackend::DecodeBase(const rtc_msgs::msg::SimSensorState& msg,
                                     int64_t recv_steady_ns) noexcept {
  // Slot k of base_sensor_idx_ caches where name k was last found.
  auto lookup = [&](std::size_t slot, const std::string& name) {
    base_sensor_idx_[slot] = FindSensor(msg, name, base_sensor_idx_[slot]);
    return base_sensor_idx_[slot];
  };

  std::array<double, 3> pos{};
  std::array<double, 4> quat{};
  std::array<double, 3> linvel{};
  std::array<double, 3> angvel{};
  bool non_finite = false;
  bool found = CopySensorAt(msg, lookup(kBaseSlotPos, base_names_.pos), pos, non_finite) &&
               CopySensorAt(msg, lookup(kBaseSlotQuat, base_names_.quat), quat, non_finite) &&
               CopySensorAt(msg, lookup(kBaseSlotLinvel, base_names_.linvel), linvel, non_finite) &&
               CopySensorAt(msg, lookup(kBaseSlotAngvel, base_names_.angvel), angvel, non_finite);
  if (found && !IsUnitQuaternionWxyz(quat)) {
    non_finite = true;  // A zero quaternion is not a rotation (NUM-2) — same verdict.
    found = false;
  }

  if (!found) {
    if (non_finite) {
      // Drop — mirror, stamp and counter untouched, so the RT side keeps the
      // last accepted sample and its honest age. Throttled WARN so a flooding
      // bad publisher stays visible without spamming (mirrors OnWrench).
      RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
                           "[mujoco_native:%s] non-finite or non-unit base sample, dropping",
                           config_.group_name.c_str());
    } else {
      // A configured sensor name the message does not carry (or carries at
      // another dimension) is a configuration error, not a transient: no
      // sample is ever accepted, so base_valid stays false, and this line
      // says why — repeated at a slow throttle so a log opened later still
      // shows it.
      RCLCPP_WARN_THROTTLE(logger_, clock_, 5000,
                           "[mujoco_native:%s] base_state sensor names not all found in "
                           "SimSensorState (pos='%s' quat='%s' linvel='%s' angvel='%s') — lane "
                           "stays invalid",
                           config_.group_name.c_str(), base_names_.pos.c_str(),
                           base_names_.quat.c_str(), base_names_.linvel.c_str(),
                           base_names_.angvel.c_str());
    }
    return false;
  }

  // Load → update → Store: this decoder is the mirror's only writer, so the
  // Load sees its own last Store and the counter continues from it.
  BaseMirror mirror = base_mirror_.Load();
  mirror.sample = MujocoFrameToPinocchio(pos, quat, linvel, angvel);
  mirror.received_at_least_once = true;
  // Stamped for an ACCEPTED sample only (see BaseMirror::recv_steady_ns).
  mirror.recv_steady_ns = recv_steady_ns;
  mirror.sequence += 1;
  base_mirror_.Store(mirror);
  return true;
}

bool MujocoNativeBackend::DecodeImu(const rtc_msgs::msg::SimSensorState& msg,
                                    int64_t recv_steady_ns) noexcept {
  ImuMirror mirror = imu_mirror_.Load();
  bool accepted_any = false;
  bool non_finite = false;
  bool missing = false;
  for (int s = 0; s < imu_names_.num_sites; ++s) {
    const auto su = static_cast<std::size_t>(s);
    int& gyro_idx = imu_sensor_idx_[2 * su];
    int& accel_idx = imu_sensor_idx_[(2 * su) + 1];
    gyro_idx = FindSensor(msg, imu_names_.gyro[su], gyro_idx);
    accel_idx = FindSensor(msg, imu_names_.accel[su], accel_idx);
    std::array<double, 3> gyro{};
    std::array<double, 3> accel{};
    bool site_non_finite = false;
    const bool found = CopySensorAt(msg, gyro_idx, gyro, site_non_finite) &&
                       CopySensorAt(msg, accel_idx, accel, site_non_finite);
    if (!found) {
      // This site only: its values, stamp and counter stay those of its last
      // accepted sample while the other site and the base lane move on.
      non_finite = non_finite || site_non_finite;
      missing = missing || !site_non_finite;
      continue;
    }
    // Raw, in the site's own frame — no conversion (DeviceState::imu).
    mirror.imu[su] = {gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]};
    mirror.recv_steady_ns[su] = recv_steady_ns;
    mirror.sequence[su] += 1;
    mirror.received[su] = true;
    accepted_any = true;
  }
  if (accepted_any) {
    mirror.num_sites = imu_names_.num_sites;
    imu_mirror_.Store(mirror);
  }
  if (non_finite) {
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
                         "[mujoco_native:%s] non-finite IMU sample, dropping that site",
                         config_.group_name.c_str());
  }
  if (missing) {
    // Same reading as the base lane's missing-name line: configuration, not
    // a transient — that site never becomes valid.
    RCLCPP_WARN_THROTTLE(logger_, clock_, 5000,
                         "[mujoco_native:%s] imu_state gyro / accel names not all found in "
                         "SimSensorState (%d site(s)) — the missing site stays invalid",
                         config_.group_name.c_str(), imu_names_.num_sites);
  }
  return accepted_any;
}

void MujocoNativeBackend::ReadBaseState(DeviceStateCache& cache) noexcept {
  // L3 under CM::ReadDeviceState — RT-tick base-mirror load.
  RTC_TRACE_SCOPE("MujocoNativeBackend::ReadBaseState");
  // One Load: the counter is part of the payload, so it is always the count
  // of the sample it is read with.
  const BaseMirror mirror = base_mirror_.Load();

  if (mirror.sequence != rt_base_last_seen_seq_) {
    rt_base_last_seen_seq_ = mirror.sequence;
    rt_base_miss_count_ = 0;
  } else if (rt_base_miss_count_ < base_max_missed_ticks_) {
    ++rt_base_miss_count_;  // saturates — no overflow on a long silence.
  }

  // Values are copied whether or not the verdict is fresh: `base_valid` is
  // the flag, not a gate on the payload (DeviceState contract — a consumer
  // deciding what to do about a stale base needs the last pose and HOW old
  // it is, and `base_recv_steady_ns` / `base_sequence` carry that).
  cache.base_pose = mirror.sample.pose;
  cache.base_twist = mirror.sample.twist;
  cache.base_recv_steady_ns = mirror.recv_steady_ns;
  cache.base_sequence = mirror.sequence;
  cache.base_valid =
      mirror.received_at_least_once && (rt_base_miss_count_ < base_max_missed_ticks_);
}

void MujocoNativeBackend::ReadImuState(DeviceStateCache& cache) noexcept {
  // L3 under CM::ReadDeviceState — RT-tick IMU-mirror load.
  RTC_TRACE_SCOPE("MujocoNativeBackend::ReadImuState");
  const ImuMirror mirror = imu_mirror_.Load();

  cache.num_imu_sites = mirror.num_sites;
  for (std::size_t s = 0; s < static_cast<std::size_t>(kMaxImuSites); ++s) {
    // Per-site miss counter: a site that stopped being accepted goes stale on
    // its own while the other one stays fresh.
    if (mirror.sequence[s] != rt_imu_last_seen_seq_[s]) {
      rt_imu_last_seen_seq_[s] = mirror.sequence[s];
      rt_imu_miss_count_[s] = 0;
    } else if (rt_imu_miss_count_[s] < imu_max_missed_ticks_) {
      ++rt_imu_miss_count_[s];  // saturates — no overflow on a long silence.
    }
    // Values held under a false flag, as on the base lane.
    cache.imu[s] = mirror.imu[s];
    cache.imu_recv_steady_ns[s] = mirror.recv_steady_ns[s];
    cache.imu_sequence[s] = mirror.sequence[s];
    cache.imu_valid[s] = mirror.received[s] && (rt_imu_miss_count_[s] < imu_max_missed_ticks_);
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
  // Stamped HERE rather than at the RT read, and only for an ACCEPTED sample:
  // the question this answers is "how old is the force the controller is
  // about to use", so a dropped NaN message (above) must not refresh it — the
  // early return leaves the previous stamp with the previous force, which is
  // the pair that is actually still in the mirror.
  tip.recv_steady_ns = rtc::SteadyNowNs();
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
  // The counters are read BEFORE the mirror, and the order is load-bearing.
  // OnWrench stores the mirror and THEN increments its counter, so a counter
  // read afterwards can be one ahead of the payload it is about to be paired
  // with — the same counter/payload hazard the catching ingress avoids by
  // judging newness from the payload. Reading it first can only pair a counter with a payload at
  // least as new, which under-reports newness for one tick instead of claiming a sample that is not
  // there.
  std::array<uint64_t, kMaxSensorGroups> seq_before{};
  for (std::size_t i = 0; i < seq_before.size(); ++i) {
    seq_before[i] = wrench_seq_[i].load(std::memory_order_acquire);
  }
  const auto mirror = sensor_mirror_.Load();
  cache.num_inference_groups = mirror.num_tips;

  for (int f = 0; f < mirror.num_tips; ++f) {
    const auto fu = static_cast<std::size_t>(f);
    const uint64_t cur_seq = seq_before[fu];

    if (cur_seq != rt_last_seen_seq_[fu]) {
      rt_last_seen_seq_[fu] = cur_seq;
      rt_miss_count_[fu] = 0;
    } else if (rt_miss_count_[fu] < max_missed_ticks_) {
      ++rt_miss_count_[fu];  // saturates at max_missed_ticks_ so no overflow.
    }

    const auto& tip = mirror.tips[fu];
    const bool fresh = tip.received_at_least_once && (rt_miss_count_[fu] < max_missed_ticks_);
    cache.inference_enable[fu] = fresh;
    // The receipt time and the counter travel BESIDE the verdict, not
    // instead of it. `cur_seq` is the accepted-sample count this backend
    // already maintains for its own miss accounting, so it is the honest
    // sequence rather than a second counter that could disagree with it.
    // Both are published even when `fresh` is false — a consumer deciding
    // TIP_STALE needs to know HOW stale, and a consumer that only trusts the
    // flag is unaffected.
    cache.inference_recv_steady_ns[fu] = tip.recv_steady_ns;
    cache.inference_sequence[fu] = cur_seq;

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
