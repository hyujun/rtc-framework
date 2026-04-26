// ── Parameter declaration and loading
// ──────────────────────────────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include "rtc_controller_interface/controller_registry.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rtc_base/logging/session_dir.hpp>
#include <rtc_urdf_bridge/pinocchio_model_builder.hpp>
#include <rtc_urdf_bridge/urdf_analyzer.hpp>
#include <rtc_urdf_bridge/xacro_processor.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

namespace urtc = rtc;

namespace {

// Split `str` on `.` into non-empty components.
std::vector<std::string> SplitDotPath(const std::string &str) {
  std::vector<std::string> out;
  std::string::size_type start = 0;
  for (auto pos = str.find('.'); pos != std::string::npos;
       pos = str.find('.', start)) {
    if (pos > start) {
      out.emplace_back(str.substr(start, pos - start));
    }
    start = pos + 1;
  }
  if (start < str.size()) {
    out.emplace_back(str.substr(start));
  }
  return out;
}

// Write @p param's typed value into @p node under @p key.
// Supports the scalar + string-array parameter kinds that controller YAMLs
// exercise; silently skips types this helper hasn't been extended to carry.
void SetYamlScalarFromParam(YAML::Node node, const std::string &key,
                            const rclcpp::Parameter &param) {
  switch (param.get_type()) {
  case rclcpp::ParameterType::PARAMETER_BOOL:
    node[key] = param.as_bool();
    break;
  case rclcpp::ParameterType::PARAMETER_INTEGER:
    node[key] = param.as_int();
    break;
  case rclcpp::ParameterType::PARAMETER_DOUBLE:
    node[key] = param.as_double();
    break;
  case rclcpp::ParameterType::PARAMETER_STRING:
    node[key] = param.as_string();
    break;
  case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: {
    YAML::Node arr(YAML::NodeType::Sequence);
    for (bool v : param.as_bool_array()) {
      arr.push_back(v);
    }
    node[key] = arr;
    break;
  }
  case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
    YAML::Node arr(YAML::NodeType::Sequence);
    for (int64_t v : param.as_integer_array()) {
      arr.push_back(v);
    }
    node[key] = arr;
    break;
  }
  case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
    YAML::Node arr(YAML::NodeType::Sequence);
    for (double v : param.as_double_array()) {
      arr.push_back(v);
    }
    node[key] = arr;
    break;
  }
  case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
    YAML::Node arr(YAML::NodeType::Sequence);
    for (const auto &v : param.as_string_array()) {
      arr.push_back(v);
    }
    node[key] = arr;
    break;
  }
  default:
    // Unsupported: byte_array, not_set. Silently skip.
    break;
  }
}

// Walk ROS parameter names under `<config_key>.` and apply each as a nested
// write into @p ctrl_node, treating `.` as the YAML path separator.
//
// Example: param `demo_wbc_controller.mpc.enabled = true` applied against
// the node loaded from `demo_wbc_controller.yaml` pokes `mpc.enabled` to
// `true`, overriding whatever the YAML file contained.
//
// Rationale — the controller config is read via `YAML::LoadFile(...)` and
// does NOT participate in the ROS parameter tree, so launch-arg overrides
// (e.g. sim.launch.py's `enable_mpc` / `mpc_engine` flags) would otherwise
// be silently discarded. This helper is the single bridge between the two.
void ApplyControllerParamOverrides(rclcpp_lifecycle::LifecycleNode &node,
                                   YAML::Node ctrl_node,
                                   const std::string &config_key) {
  // list_parameters depth counts `.`-separated segments from the root; we
  // set a generous cap to cover any reasonable nested YAML.
  constexpr uint64_t kMaxDepth = 20;
  const auto params = node.list_parameters({config_key}, kMaxDepth);
  const std::string prefix = config_key + ".";

  for (const auto &pname : params.names) {
    if (pname.rfind(prefix, 0) != 0) {
      continue;
    }
    const auto parts = SplitDotPath(pname.substr(prefix.size()));
    if (parts.empty()) {
      continue;
    }

    // Walk to the parent map. CRITICAL: `Node::operator=` in yaml-cpp is a
    // VALUE COPY that overwrites the left-hand node's data — using it here
    // (e.g. `parent = parent[x]`) would splice the child subtree over the
    // ctrl_node root and lose sibling keys (observed as the grand
    // `integration` section vanishing after applying a single
    // `demo_wbc_controller.mpc.*` override). `Node::reset(other)` is the
    // reference-rebind that leaves the underlying tree untouched.
    YAML::Node parent(ctrl_node);
    for (std::size_t i = 0; i + 1 < parts.size(); ++i) {
      parent.reset(parent[parts[i]]);
    }

    try {
      SetYamlScalarFromParam(parent, parts.back(), node.get_parameter(pname));
    } catch (const std::exception &e) {
      RCLCPP_WARN(node.get_logger(),
                  "[param-override] failed to apply '%s': %s", pname.c_str(),
                  e.what());
    }
  }
}

} // namespace

// ── Initialisation helpers
// ────────────────────────────────────────────────────
void RtControllerNode::DeclareAndLoadParameters() {
  // Helper: declare only if not already auto-declared from YAML overrides.
  // (NodeOptions::automatically_declare_parameters_from_overrides is enabled.)
  auto safe_declare = [this](const std::string &name,
                             const rclcpp::ParameterValue &val) {
    if (!has_parameter(name)) {
      declare_parameter(name, val);
    }
  };

  safe_declare("control_rate", rclcpp::ParameterValue(500.0));
  safe_declare("kp", rclcpp::ParameterValue(5.0));
  safe_declare("kd", rclcpp::ParameterValue(0.5));
  safe_declare("enable_logging", rclcpp::ParameterValue(true));
  safe_declare("log_dir", rclcpp::ParameterValue(std::string("")));
  safe_declare("max_log_sessions", rclcpp::ParameterValue(10));
  safe_declare("enable_timing_log", rclcpp::ParameterValue(true));
  safe_declare("enable_device_log", rclcpp::ParameterValue(true));
  safe_declare("enable_estop", rclcpp::ParameterValue(true));
  safe_declare("init_timeout_sec", rclcpp::ParameterValue(5.0));
  safe_declare("auto_hold_position", rclcpp::ParameterValue(true));
  safe_declare("initial_controller",
               rclcpp::ParameterValue(std::string("joint_pd_controller")));
  safe_declare("use_sim_time_sync", rclcpp::ParameterValue(false));
  safe_declare("sim_sync_timeout_sec", rclcpp::ParameterValue(5.0));
  safe_declare("robot_namespace", rclcpp::ParameterValue(std::string("ur5e")));

  // ── Device timeouts (replaces robot_timeout_ms / enable_ur5e / enable_hand)
  // ─
  safe_declare("device_timeout_names",
               rclcpp::ParameterValue(std::vector<std::string>{}));
  safe_declare("device_timeout_values",
               rclcpp::ParameterValue(std::vector<double>{}));

  control_rate_ = get_parameter("control_rate").as_double();
  budget_us_ = 1.0e6 / control_rate_;

  const double init_timeout_sec = get_parameter("init_timeout_sec").as_double();
  init_timeout_ticks_ = static_cast<uint64_t>(init_timeout_sec * control_rate_);
  auto_hold_position_ = get_parameter("auto_hold_position").as_bool();
  enable_logging_ = get_parameter("enable_logging").as_bool();
  enable_estop_ = get_parameter("enable_estop").as_bool();
  use_sim_time_sync_ = get_parameter("use_sim_time_sync").as_bool();
  sim_sync_timeout_sec_ = get_parameter("sim_sync_timeout_sec").as_double();
  robot_ns_ = get_parameter("robot_namespace").as_string();

  // NOTE: Logging setup and device name configs are deferred until after
  // controller loading, because active_groups_ must be known first.

  // ── Resolve system URDF and model topology ─────────────────────────────────
  std::string urdf_path;
  std::shared_ptr<rtc_urdf_bridge::PinocchioModelBuilder> shared_builder;
  {
    // 1) Top-level urdf section (preferred)
    if (has_parameter("urdf.package") && has_parameter("urdf.path")) {
      try {
        const auto pkg = get_parameter("urdf.package").as_string();
        const auto rel = get_parameter("urdf.path").as_string();
        urdf_path =
            ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
        system_model_config_.urdf_path = urdf_path;

        if (has_parameter("urdf.root_joint_type")) {
          system_model_config_.root_joint_type =
              get_parameter("urdf.root_joint_type").as_string();
        }

        ParseSubModels(system_model_config_);
        ParseTreeModels(system_model_config_);

        if (has_parameter("urdf.passive_joints")) {
          system_model_config_.passive_joints =
              get_parameter("urdf.passive_joints").as_string_array();
        }

        // Build the system PinocchioModelBuilder once and share it with every
        // registered controller. Each controller would otherwise re-run the
        // same xacro → tinyxml2 → Pinocchio pipeline against an identical
        // ModelConfig (4× URDF parses on a 3-controller bring-up). Failure
        // is non-fatal: controllers fall back to constructing their own
        // builder from GetSystemModelConfig().
        try {
          shared_builder =
              std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(
                  system_model_config_);
        } catch (const std::exception &e) {
          RCLCPP_WARN(get_logger(),
                      "Shared PinocchioModelBuilder build failed (%s) — "
                      "controllers will build their own",
                      e.what());
        }

        if (shared_builder) {
          const auto &analyzer = shared_builder->GetAnalyzer();
          RCLCPP_INFO(
              get_logger(),
              "System URDF: %s (%zu sub_models, %zu tree_models, "
              "%zu yaml-passive_joints; %zu <mimic> tags auto-locked by "
              "PinocchioModelBuilder, %zu transmission-less passive)",
              urdf_path.c_str(), system_model_config_.sub_models.size(),
              system_model_config_.tree_models.size(),
              system_model_config_.passive_joints.size(),
              analyzer.GetMimicJoints().size(),
              analyzer.GetPassiveJoints().size());
        } else {
          RCLCPP_INFO(get_logger(),
                      "System URDF: %s (%zu sub_models, %zu tree_models, %zu "
                      "yaml-passive_joints)",
                      urdf_path.c_str(), system_model_config_.sub_models.size(),
                      system_model_config_.tree_models.size(),
                      system_model_config_.passive_joints.size());
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Failed to resolve system URDF config: %s",
                    e.what());
      }
    }

    // 2) Fallback: scan devices for first URDF config (backward compatibility)
    if (urdf_path.empty()) {
      const auto params = list_parameters({"devices"}, 10);
      for (const auto &prefix : params.prefixes) {
        const std::string pkg_key = prefix + ".urdf.package";
        const std::string path_key = prefix + ".urdf.path";
        if (has_parameter(pkg_key) && has_parameter(path_key)) {
          try {
            const auto pkg = get_parameter(pkg_key).as_string();
            const auto rel = get_parameter(path_key).as_string();
            urdf_path =
                ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
            system_model_config_.urdf_path = urdf_path;
            RCLCPP_INFO(get_logger(),
                        "URDF path from devices config (fallback): %s",
                        urdf_path.c_str());
          } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(),
                        "Failed to resolve URDF from devices config: %s",
                        e.what());
          }
          break;
        }
      }
    }

    if (urdf_path.empty()) {
      RCLCPP_WARN(get_logger(),
                  "No URDF configured — controllers may lack kinematics");
    }
  }

  // ── Instantiate and configure all registered controllers ─────────────────
  std::unordered_map<std::string, int> name_to_idx;
  const auto &entries = urtc::ControllerRegistry::Instance().GetEntries();

  for (std::size_t i = 0; i < entries.size(); ++i) {
    const auto &entry = entries[i];
    auto ctrl = entry.factory(urdf_path);

    if (!system_model_config_.urdf_path.empty()) {
      ctrl->SetSystemModelConfig(system_model_config_);
    }
    if (shared_builder) {
      ctrl->SetSharedModelBuilder(shared_builder);
    }

    // Load YAML for this controller; empty node on failure so on_configure
    // falls through to built-in defaults (matches previous behavior).
    YAML::Node ctrl_node;
    try {
      const std::string pkg_dir =
          ament_index_cpp::get_package_share_directory(entry.config_package);
      const std::string yaml_path = pkg_dir + "/config/controllers/" +
                                    entry.config_subdir + entry.config_key +
                                    ".yaml";
      YAML::Node file_node = YAML::LoadFile(yaml_path);
      ctrl_node = file_node[entry.config_key];
      ApplyControllerParamOverrides(*this, ctrl_node, entry.config_key);
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(),
                  "Config load failed for '%s' (pkg=%s, %s) — using defaults",
                  ctrl->Name().data(), entry.config_package.c_str(), e.what());
    }

    // Create a dedicated LifecycleNode per controller.  Namespace is
    // "/<config_key>" so controller-owned topics declared with relative
    // paths in YAML resolve to /<config_key>/<topic> automatically.
    // main() attaches these to aux_executor.
    // Per-controller LifecycleNodes must NOT inherit the parent's CLI
    // remappings (e.g. `--ros-args -r __node:=ur5e_rt_controller`).  Without
    // use_global_arguments(false), the global remap rules rename every child
    // to the parent's node name and namespace, which clobbers the intended
    // /<config_key> namespacing and produces one
    // `rcl.logging_rosout: Publisher already registered` warning per child.
    const std::string ctrl_ns = "/" + entry.config_key;
    const auto ctrl_node_options =
        rclcpp::NodeOptions().use_global_arguments(false);
    auto ctrl_lc_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        entry.config_key, ctrl_ns, ctrl_node_options);

    // Inject the target-received notifier so controllers that own their own
    // target subscription flip CM's target_received_ gate in the callback.
    // Matches the prior behavior of CM's DeviceTargetCallback.
    ctrl->SetTargetReceivedNotifier(
        [this]() { target_received_.store(true, std::memory_order_release); });

    // Drive the controller's lifecycle on_configure.  Default implementation
    // stores the node and invokes LoadConfig(ctrl_node) internally; a FAILURE
    // return is non-fatal (matches previous "use defaults" semantics).
    const rclcpp_lifecycle::State unconfigured_state(
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    const auto cfg_ret =
        ctrl->on_configure(unconfigured_state, ctrl_lc_node, ctrl_node);
    if (cfg_ret != urtc::RTControllerInterface::CallbackReturn::SUCCESS) {
      RCLCPP_WARN(get_logger(),
                  "Controller '%s' on_configure returned non-SUCCESS — "
                  "continuing with defaults",
                  ctrl->Name().data());
    }

    name_to_idx[std::string(ctrl->Name())] = static_cast<int>(i);
    name_to_idx[entry.config_key] = static_cast<int>(i);
    controller_name_to_idx_[std::string(ctrl->Name())] = static_cast<int>(i);
    controller_name_to_idx_[entry.config_key] = static_cast<int>(i);

    controllers_.push_back(std::move(ctrl));
    controller_nodes_.push_back(std::move(ctrl_lc_node));
    controller_types_.push_back(entry.config_key);
  }

  // Cache per-controller topic configs and build active_groups_ +
  // group_slot_map_
  controller_topic_configs_.reserve(controllers_.size());
  for (const auto &ctrl : controllers_) {
    controller_topic_configs_.push_back(ctrl->GetTopicConfig());

    const auto &tc = controller_topic_configs_.back();
    std::string groups_info;
    for (const auto &[name, group] : tc.groups) {
      if (!group.subscribe.empty() || !group.publish.empty()) {
        groups_info += name + "(" + std::to_string(group.subscribe.size()) +
                       "sub+" + std::to_string(group.publish.size()) + "pub) ";
      }
    }
    RCLCPP_INFO(get_logger(), "Controller '%s': %s", ctrl->Name().data(),
                groups_info.c_str());
  }

  // ── Build active_groups_ (union of all controllers' groups) ──────────────
  {
    int slot_idx = 0;
    for (const auto &tc : controller_topic_configs_) {
      for (const auto &[name, group] : tc.groups) {
        if (!group.subscribe.empty() || !group.publish.empty()) {
          if (active_groups_.insert(name).second) {
            group_slot_map_[name] = slot_idx++;
          }
        }
      }
    }
    if (slot_idx > urtc::PublishSnapshot::kMaxGroups) {
      RCLCPP_FATAL(get_logger(), "Too many device groups (%d > %d)", slot_idx,
                   urtc::PublishSnapshot::kMaxGroups);
      rclcpp::shutdown();
      return;
    }
  }

  // ── Load device name configs (needs active_groups_) ─────────────────────
  LoadDeviceNameConfigs();

  // ── Build per-controller flat slot mappings (RT-safe, no map lookup) ────
  controller_slot_mappings_.resize(controllers_.size());
  for (std::size_t ci = 0; ci < controllers_.size(); ++ci) {
    auto &mapping = controller_slot_mappings_[ci];
    int gi = 0;
    for (const auto &[gname, ggroup] : controller_topic_configs_[ci].groups) {
      if (gi < ControllerSlotMapping::kMaxSlots) {
        const auto gidx = static_cast<std::size_t>(gi);
        mapping.slots[gidx] = group_slot_map_[gname];
        mapping.capabilities[gidx] = ggroup.capability;
      }
      ++gi;
    }
    mapping.num_groups = gi;
  }

  // Pass control rate and device configs to all controllers
  for (auto &ctrl : controllers_) {
    ctrl->SetControlRate(control_rate_);
    ctrl->SetDeviceNameConfigs(device_name_configs_);
  }

  // ── Deferred logging setup (needs active_groups_ + device_name_configs_) ─
  if (enable_logging_) {
    const std::string log_dir_param = get_parameter("log_dir").as_string();
    const int max_sessions =
        static_cast<int>(get_parameter("max_log_sessions").as_int());

    std::filesystem::path session_dir;
    if (!log_dir_param.empty()) {
      session_dir = std::filesystem::path(log_dir_param);
      std::filesystem::create_directories(session_dir);
      urtc::EnsureSessionSubdirs(session_dir);
    } else {
      session_dir = ResolveAndSetupSessionDir();
    }
    const auto logging_root = session_dir.parent_path();
    urtc::CleanupOldSessions(logging_root, max_sessions);

    const bool enable_timing = get_parameter("enable_timing_log").as_bool();
    const bool enable_device = get_parameter("enable_device_log").as_bool();
    const auto ctrl_dir = session_dir / "controller";
    std::filesystem::create_directories(ctrl_dir);

    const std::string timing_path =
        enable_timing ? (ctrl_dir / "timing_log.csv").string() : "";

    std::vector<urtc::DeviceLogConfig> log_configs;
    if (enable_device && !controller_topic_configs_.empty()) {
      const int init_idx =
          active_controller_idx_.load(std::memory_order_relaxed);
      const auto &init_tc =
          controller_topic_configs_[static_cast<std::size_t>(init_idx)];

      int gi = 0;
      for (const auto &[gname, group] : init_tc.groups) {
        for (const auto &pt : group.publish) {
          if (pt.role != urtc::PublishRole::kDeviceStateLog &&
              pt.role != urtc::PublishRole::kDeviceSensorLog)
            continue;

          urtc::DeviceLogConfig dlc;
          dlc.device_name = gname;
          dlc.role = pt.role;
          dlc.device_index = gi;

          std::string fname = pt.topic_name;
          std::replace(fname.begin(), fname.end(), '/', '_');
          if (!fname.empty() && fname.front() == '_')
            fname.erase(0, 1);
          dlc.path = ctrl_dir / (fname + ".csv");

          auto it = device_name_configs_.find(gname);
          if (it != device_name_configs_.end()) {
            dlc.joint_names = it->second.joint_state_names;
            dlc.motor_names = it->second.motor_state_names;
            dlc.sensor_names = it->second.sensor_names;
            dlc.num_channels =
                static_cast<int>(it->second.joint_state_names.size());
            dlc.num_motor_channels =
                static_cast<int>(it->second.motor_state_names.size());
            dlc.num_sensor_channels =
                static_cast<int>(it->second.sensor_names.size() *
                                 urtc::kSensorValuesPerFingertip);
          }
          log_configs.push_back(std::move(dlc));
        }
        ++gi;
      }
    }

    int max_inference = 0;
    for (const auto &lc : log_configs) {
      if (lc.role == urtc::PublishRole::kDeviceSensorLog &&
          !lc.sensor_names.empty()) {
        const int niv = static_cast<int>(lc.sensor_names.size()) *
                        urtc::kFTValuesPerFingertip;
        if (niv > max_inference)
          max_inference = niv;
      }
    }

    logger_ = std::make_unique<urtc::DataLogger>(
        timing_path, std::move(log_configs), max_inference);
    RCLCPP_INFO(get_logger(), "Logging to: %s/controller/ (max_sessions=%d)",
                session_dir.string().c_str(), max_sessions);
  }

  // ── Parse device_timeouts & match to active topic groups ─────────────────
  {
    const auto timeout_names =
        get_parameter("device_timeout_names").as_string_array();
    const auto timeout_values =
        get_parameter("device_timeout_values").as_double_array();
    for (std::size_t i = 0;
         i < timeout_names.size() && i < timeout_values.size(); ++i) {
      const auto &name = timeout_names[i];
      if (!active_groups_.contains(name)) {
        RCLCPP_WARN(get_logger(),
                    "Device timeout '%s' has no matching topic group — ignored",
                    name.c_str());
        continue;
      }
      std::string state_topic;
      for (const auto &tc : controller_topic_configs_) {
        state_topic =
            tc.GetSubscribeTopicName(name, urtc::SubscribeRole::kState);
        if (!state_topic.empty())
          break;
      }
      if (state_topic.empty()) {
        RCLCPP_WARN(get_logger(),
                    "Device timeout '%s' has no state subscription — ignored",
                    name.c_str());
        continue;
      }
      DeviceTimeoutEntry entry;
      entry.group_name = name;
      entry.state_topic = state_topic;
      entry.timeout =
          std::chrono::milliseconds(static_cast<int>(timeout_values[i]));
      device_timeouts_.push_back(std::move(entry));
      RCLCPP_INFO(get_logger(), "Device timeout: '%s' → watching '%s' (%dms)",
                  name.c_str(), state_topic.c_str(),
                  static_cast<int>(timeout_values[i]));
    }
  }

  // Skip init wait if no device timeouts configured
  if (device_timeouts_.empty()) {
    state_received_.store(true, std::memory_order_release);
    target_received_.store(true, std::memory_order_release);
    RCLCPP_INFO(get_logger(),
                "No device timeouts configured — skipping init wait");
  }

  // Resolve initial_controller parameter → controller index
  const std::string initial_ctrl =
      get_parameter("initial_controller").as_string();
  const auto it = name_to_idx.find(initial_ctrl);
  if (it != name_to_idx.end()) {
    active_controller_idx_.store(it->second);
  } else {
    RCLCPP_WARN(
        get_logger(),
        "Unknown initial_controller '%s', defaulting to joint_pd_controller",
        initial_ctrl.c_str());
    const auto pd_it = name_to_idx.find("joint_pd_controller");
    const int fallback = (pd_it != name_to_idx.end() &&
                          pd_it->second < static_cast<int>(controllers_.size()))
                             ? pd_it->second
                             : 0;
    active_controller_idx_.store(fallback);
  }
}
