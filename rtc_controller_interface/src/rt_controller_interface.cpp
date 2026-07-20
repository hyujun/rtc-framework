#include "rtc_controller_interface/rt_controller_interface.hpp"

#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/types.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace rtc {

namespace {

// Throttle window for the ingress joint-limit warning. Mirrors
// integrated_bringup's kThrottleSlowMs ("generic recurring warning"), which is
// not reachable from here — that header is integrated_bringup-private.
// A stuck publisher (GUI slider held at a bound, an exploration node streaming
// goals) would otherwise reprint per message.
constexpr int kTargetLimitWarnThrottleMs = 2000;

// conventions.md §Logger naming: logs emitted from the RTControllerInterface
// base ride the library-level logger (`<full_package_name>`) and name the
// concrete controller via a message prefix instead. node_->get_logger() would
// render as "<config_key>.<config_key>" — CM builds each controller node with
// name == namespace == config_key — which matches none of the three tiers.
const rclcpp::Logger& TargetWarnLogger() {
  static const rclcpp::Logger logger = rclcpp::get_logger("rtc_controller_interface");
  return logger;
}

// Phase 4: device-wire roles (state / motor_state / sensor_state / joint_command
// / ros2_command) live in `devices.<group>.backend:` (sim.yaml/robot.yaml) and
// are owned by DeviceBackend impls. Controller YAML retains only target /
// robot_target / robot_transforms / digital_twin_state — grasp_state /
// wbc_state / tof_snapshot are now controller-owned via per-controller
// SeqLock and are not declared in YAML.
//
// Phase 4 trailing cleanup: SubscribeRole enum dropped — the only remaining
// value (kTarget) carries no discrimination. The parser still validates the
// `role:` string as documentation + drift detection.
const std::unordered_set<std::string> kSubscribeRoleStrings = {
    "target",
    "goal",  // backward compat
};

const std::unordered_map<std::string, PublishRole> kPublishRoleMap = {
    // Topic-based State/Command/Goal
    {"robot_target", PublishRole::kRobotTarget},
    // backward compat
    {"joint_goal", PublishRole::kRobotTarget},
    {"robot_transforms", PublishRole::kRobotTransforms},
    // Digital twin
    {"digital_twin_state", PublishRole::kDigitalTwinState},
};

// Parse subscribe/publish arrays from a YAML device group node (ur5e or hand).
void ParseDeviceTopicGroup(const YAML::Node& group_node, DeviceTopicGroup& out) {
  if (group_node["subscribe"] && group_node["subscribe"].IsSequence()) {
    for (const auto& entry : group_node["subscribe"]) {
      const auto topic = entry["topic"].as<std::string>();
      const auto role_str = entry["role"].as<std::string>();
      if (kSubscribeRoleStrings.find(role_str) == kSubscribeRoleStrings.end()) {
        throw std::runtime_error("Unknown subscribe role: " + role_str);
      }
      out.subscribe.push_back({topic});
    }
  }

  if (group_node["publish"] && group_node["publish"].IsSequence()) {
    for (const auto& entry : group_node["publish"]) {
      const auto topic = entry["topic"].as<std::string>();
      const auto role_str = entry["role"].as<std::string>();
      auto it = kPublishRoleMap.find(role_str);
      if (it == kPublishRoleMap.end()) {
        throw std::runtime_error("Unknown publish role: " + role_str);
      }
      int data_size = 0;
      if (entry["data_size"]) {
        data_size = entry["data_size"].as<int>();
      }
      out.publish.push_back({topic, it->second, data_size});
    }
  }
}

}  // namespace

RTControllerInterface::RTControllerInterface() : topic_config_{} {}

RTControllerInterface::~RTControllerInterface() = default;

// ── Lifecycle hooks: default implementations ─────────────────────────────────
//
// noexcept contract: transitions that touch YAML/allocators must catch all
// exceptions and report FAILURE via CallbackReturn.

// Roll back the base-owned state a failed configure attempt may have left
// behind, then latch kFailed.
//
// Only base-owned state can be undone here: a derived LoadConfig() that threw
// halfway through its own members is not unwound, and there is no general way
// to do so. The safety property does not depend on it — kFailed is terminal
// until on_cleanup(), so a controller carrying half-parsed derived state can
// never be configured, activated, or reach RT Compute().
void RTControllerInterface::FailConfigure() noexcept {
  node_.reset();
  topic_config_ = TopicConfig{};
  config_state_ = ConfigState::kFailed;
}

RTControllerInterface::CallbackReturn RTControllerInterface::PreConfigure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, const YAML::Node& yaml_cfg) noexcept {
  if (!node) {
    FailConfigure();
    return CallbackReturn::FAILURE;
  }
  node_ = std::move(node);
  try {
    LoadConfig(yaml_cfg);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("rtc_controller_interface"), "[%s] LoadConfig failed: %s",
                 std::string(Name()).c_str(), e.what());
    FailConfigure();
    return CallbackReturn::FAILURE;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("rtc_controller_interface"),
                 "[%s] LoadConfig failed: unknown exception", std::string(Name()).c_str());
    FailConfigure();
    return CallbackReturn::FAILURE;
  }
  config_state_ = ConfigState::kPreConfigured;
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, const YAML::Node& yaml_cfg) noexcept {
  // Fail-closed re-entry (issue #196 §1): a controller whose PreConfigure
  // failed must not be resurrected here. The old guard keyed off `node_ !=
  // nullptr` alone, which is exactly the state a failed PreConfigure used to
  // leave behind — so the failure silently turned into SUCCESS. FailConfigure
  // now clears node_, but the explicit state check is what makes the
  // guarantee: no configure path flips kFailed back without on_cleanup().
  if (config_state_ == ConfigState::kFailed) {
    RCLCPP_ERROR(rclcpp::get_logger("rtc_controller_interface"),
                 "[%s] on_configure refused: controller is in the failed "
                 "configure state (cleanup required before retry)",
                 std::string(Name()).c_str());
    return CallbackReturn::FAILURE;
  }

  // Idempotent re-entry: when CM drives the 3-pass bring-up, PreConfigure has
  // already stored node_ and parsed yaml_cfg. Direct callers (unit tests,
  // legacy paths) skip PreConfigure, so we still accept the node + reparse
  // here.
  if (!node_) {
    if (!node) {
      FailConfigure();
      return CallbackReturn::FAILURE;
    }
    node_ = std::move(node);
    try {
      LoadConfig(yaml_cfg);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("rtc_controller_interface"), "[%s] LoadConfig failed: %s",
                   std::string(Name()).c_str(), e.what());
      FailConfigure();
      return CallbackReturn::FAILURE;
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("rtc_controller_interface"),
                   "[%s] LoadConfig failed: unknown exception", std::string(Name()).c_str());
      FailConfigure();
      return CallbackReturn::FAILURE;
    }
  }
  config_state_ = ConfigState::kConfigured;
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) noexcept {
  // Controller-internal init policy: each derived controller seeds its
  // target slot from the current device state on the first Compute() tick
  // after activation. The base has no hold-init responsibility — see the
  // header comment for the rationale (single-writer SeqLock invariant).
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) noexcept {
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) noexcept {
  node_.reset();
  // The one path out of kFailed. Cleanup is the explicit lifecycle transition
  // back to Unconfigured, so a caller that has torn the controller down may
  // configure it again from scratch; nothing else clears the failed latch.
  config_state_ = ConfigState::kUnconfigured;
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_shutdown(
    const rclcpp_lifecycle::State& previous_state) noexcept {
  return on_cleanup(previous_state);
}

RTControllerInterface::CallbackReturn RTControllerInterface::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) noexcept {
  return CallbackReturn::SUCCESS;
}

void RTControllerInterface::SetSystemModelConfig(const rtc_urdf_bridge::ModelConfig& config) {
  system_model_config_ = std::make_unique<rtc_urdf_bridge::ModelConfig>(config);
  OnSystemModelConfigSet();
}

const rtc_urdf_bridge::ModelConfig* RTControllerInterface::GetSystemModelConfig() const noexcept {
  return system_model_config_.get();
}

void RTControllerInterface::SetSharedModelBuilder(
    std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> builder) noexcept {
  shared_model_builder_ = std::move(builder);
}

std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder>
RTControllerInterface::GetSharedModelBuilder() const noexcept {
  return shared_model_builder_;
}

TopicConfig RTControllerInterface::ParseTopicConfig(const YAML::Node& topics_node) {
  // ── Detect deprecated flat format (topics.subscribe exists directly) ──
  if (topics_node["subscribe"] && topics_node["subscribe"].IsSequence()) {
    throw std::runtime_error(
        "Flat topics: format is deprecated. "
        "Migrate to device-group keyed format (e.g. topics.ur5e / "
        "topics.hand). "
        "See config/controllers/ examples for the new format.");
  }

  TopicConfig cfg;

  // ── Dynamic device group parsing: iterate all keys ──
  for (auto it = topics_node.begin(); it != topics_node.end(); ++it) {
    const std::string group_name = it->first.as<std::string>();
    if (!it->second.IsMap()) {
      continue;
    }
    ParseDeviceTopicGroup(it->second, cfg[group_name]);
  }

  return cfg;
}

TargetLimitViolation CheckTargetLimits(std::span<const double> ordered,
                                       const DeviceJointLimits& lim) noexcept {
  TargetLimitViolation out;
  // Ragged/short config must never drive an out-of-bounds read: a device may
  // declare max_velocity but omit position bounds, in which case there is
  // nothing to screen against and every joint is reported clean.
  const std::size_t n =
      std::min({ordered.size(), lim.position_lower.size(), lim.position_upper.size()});

  for (std::size_t i = 0; i < n; ++i) {
    const double v = ordered[i];
    const bool bad_finite = !std::isfinite(v);
    // Ordering matters: a NaN fails both comparisons below, so the finite test
    // must come first or NaN would be classified clean.
    const bool out_of_range =
        !bad_finite && (v < lim.position_lower[i] || v > lim.position_upper[i]);
    if (!bad_finite && !out_of_range) {
      continue;
    }
    if (out.count == 0) {
      out.first_idx = static_cast<int>(i);
      out.value = v;
      out.lower = lim.position_lower[i];
      out.upper = lim.position_upper[i];
      out.non_finite = bad_finite;
    }
    ++out.count;
  }
  return out;
}

void RTControllerInterface::WarnIfTargetOutOfLimits(
    const std::string& group_name, std::span<const double> ordered) const noexcept {
  if (!node_) {
    return;
  }
  auto cfg_it = device_name_configs_.find(group_name);
  if (cfg_it == device_name_configs_.end() || !cfg_it->second.joint_limits) {
    return;
  }
  const auto& cfg = cfg_it->second;
  const TargetLimitViolation v = CheckTargetLimits(ordered, *cfg.joint_limits);
  if (v.count == 0) {
    return;
  }

  const auto idx = static_cast<std::size_t>(v.first_idx);
  // joint_limits arrays are parser-validated to match joint_state_names in
  // length (rt_controller_node_device_config.cpp clears the block otherwise),
  // so this index is sound; the guard covers a hand-built config in tests.
  const char* jname =
      (idx < cfg.joint_state_names.size()) ? cfg.joint_state_names[idx].c_str() : "<unnamed>";

  // `%.*s` — Name() is a string_view with no null-termination guarantee.
  // Scalars otherwise: no std::string assembly, because the caller
  // (DeliverTargetMessage) is noexcept and a throwing allocation would
  // terminate the process.
  const auto name_len = static_cast<int>(Name().size());
  const char* name_ptr = Name().data();
  if (v.non_finite) {
    RCLCPP_WARN_THROTTLE(TargetWarnLogger(), *node_->get_clock(), kTargetLimitWarnThrottleMs,
                         "[%.*s] device '%s': joint_goal '%s' is non-finite (%f) — %d joint(s) "
                         "outside limits; note the RT clamp does NOT stop a non-finite command",
                         name_len, name_ptr, group_name.c_str(), jname, v.value, v.count);
    return;
  }
  RCLCPP_WARN_THROTTLE(TargetWarnLogger(), *node_->get_clock(), kTargetLimitWarnThrottleMs,
                       "[%.*s] device '%s': %d joint goal(s) outside limits — first: '%s' = "
                       "%.4f rad, limit [%.4f, %.4f] — command will be clamped",
                       name_len, name_ptr, group_name.c_str(), v.count, jname, v.value, v.lower,
                       v.upper);
}

void RTControllerInterface::DeliverTargetMessage(const std::string& group_name, int device_idx,
                                                 const rtc_msgs::msg::RobotTarget& msg) noexcept {
  const double* data_ptr = nullptr;
  int data_size = 0;

  if (msg.goal_type == "task") {
    data_ptr = msg.task_target.data();
    data_size = static_cast<int>(msg.task_target.size());
  } else {
    if (msg.joint_target.empty()) {
      return;
    }
    data_ptr = msg.joint_target.data();
    data_size = static_cast<int>(msg.joint_target.size());
  }

  std::array<double, kMaxDeviceChannels> reordered{};
  const double* ordered_ptr = data_ptr;
  int ordered_size = data_size;

  if (msg.goal_type != "task" && !msg.joint_names.empty()) {
    auto cfg_it = device_name_configs_.find(group_name);
    if (cfg_it != device_name_configs_.end()) {
      const auto& ref_names = cfg_it->second.joint_state_names;
      if (!ref_names.empty()) {
        for (std::size_t mi = 0; mi < msg.joint_names.size() && mi < msg.joint_target.size();
             ++mi) {
          for (std::size_t ri = 0; ri < ref_names.size(); ++ri) {
            if (msg.joint_names[mi] == ref_names[ri]) {
              reordered[ri] = msg.joint_target[mi];
              break;
            }
          }
        }
        ordered_ptr = reordered.data();
        ordered_size = std::min(static_cast<int>(ref_names.size()), kMaxDeviceChannels);
      }
    }
  }

  const int n = std::min(ordered_size, kMaxDeviceChannels);
  const std::span<const double> ordered_span(ordered_ptr, static_cast<std::size_t>(n));
  // A task goal routes to the SE3 slot (SetDeviceTaskTarget); the default
  // override forwards to SetDeviceTarget, so task controllers are unchanged.
  // Joint goals keep the legacy SetDeviceTarget path verbatim.
  if (msg.goal_type == "task") {
    SetDeviceTaskTarget(device_idx, ordered_span);
  } else {
    // Warn-only, and only here: `ordered_span` is already in device-channel
    // order, so it aligns index-for-index with joint_limits/joint_state_names.
    // A task goal carries Cartesian [x,y,z,r,p,y] — joint limits do not apply.
    WarnIfTargetOutOfLimits(group_name, ordered_span);
    SetDeviceTarget(device_idx, ordered_span);
  }
}

void RTControllerInterface::LoadDeviceLimitsFromConfig(
    std::array<std::vector<double>, ControllerState::kMaxDevices>& position_lower,
    std::array<std::vector<double>, ControllerState::kMaxDevices>& position_upper,
    std::array<std::vector<double>, ControllerState::kMaxDevices>& max_velocity,
    double default_lower, double default_upper, double default_velocity) const {
  // Walk topic_config_.groups in declaration order so the controller-local
  // device index matches the rest of the controller's data layout
  // (state.devices[i], device_max_velocity_[i], …).
  const std::size_t n_groups =
      std::min<std::size_t>(topic_config_.groups.size(), ControllerState::kMaxDevices);

  for (std::size_t i = 0; i < n_groups; ++i) {
    const auto& [device_name, _group] = topic_config_.groups[i];
    auto cfg_it = device_name_configs_.find(device_name);
    if (cfg_it != device_name_configs_.end() && cfg_it->second.joint_limits) {
      const auto& limits = *cfg_it->second.joint_limits;
      if (!limits.position_lower.empty()) {
        position_lower[i] = limits.position_lower;
      }
      if (!limits.position_upper.empty()) {
        position_upper[i] = limits.position_upper;
      }
      if (!limits.max_velocity.empty()) {
        max_velocity[i] = limits.max_velocity;
      }
    }
  }

  // Fill remaining empty slots (slots without matching DeviceNameConfig, or
  // with empty joint_limits) with caller-supplied fallbacks so RT-path
  // clamping always has valid bounds of length kMaxDeviceChannels.
  for (auto& slot : position_lower) {
    if (slot.empty()) {
      slot.assign(kMaxDeviceChannels, default_lower);
    }
  }
  for (auto& slot : position_upper) {
    if (slot.empty()) {
      slot.assign(kMaxDeviceChannels, default_upper);
    }
  }
  for (auto& slot : max_velocity) {
    if (slot.empty()) {
      slot.assign(kMaxDeviceChannels, default_velocity);
    }
  }
}

std::vector<double> RTControllerInterface::ParseArmSafePosition(
    const YAML::Node& cfg, std::size_t expected_size, const std::string& controller_name) {
  if (!cfg["estop"] || !cfg["estop"].IsMap()) {
    throw std::runtime_error(controller_name + ": required 'estop' section is missing");
  }
  const auto estop_node = cfg["estop"];
  if (!estop_node["arm_safe_position"] || !estop_node["arm_safe_position"].IsSequence()) {
    throw std::runtime_error(controller_name +
                             ": required 'estop.arm_safe_position' must be a sequence");
  }
  const auto safe_seq = estop_node["arm_safe_position"];
  if (safe_seq.size() != expected_size) {
    throw std::runtime_error(controller_name + ": 'estop.arm_safe_position' length " +
                             std::to_string(safe_seq.size()) + " != expected " +
                             std::to_string(expected_size));
  }
  std::vector<double> out;
  out.reserve(expected_size);
  for (std::size_t i = 0; i < expected_size; ++i) {
    out.push_back(safe_seq[i].as<double>());
  }
  return out;
}

void RTControllerInterface::LoadConfig(const YAML::Node& cfg) {
  if (!cfg) {
    return;
  }

  // ── Deprecation warning for removed device enable flags ──
  if (cfg["enable_ur5e"] || cfg["enable_hand"]) {
    std::cerr << "[WARN] enable_ur5e/enable_hand in controller YAML is "
              << "deprecated and ignored. Device activation is now determined "
              << "by the topics section presence.\n";
  }

  // Parse topics section if present; otherwise leave topic_config_ empty.
  // Controllers without YAML topics keep an empty topic_config_; the CM /
  // owned-topics helpers tolerate that (rtc_* stays robot-agnostic — no
  // hardcoded fallback device name lives in the framework).
  if (cfg["topics"]) {
    topic_config_ = ParseTopicConfig(cfg["topics"]);
  }
}

}  // namespace rtc
