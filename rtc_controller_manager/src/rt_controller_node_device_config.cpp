// ── Device name configuration, URDF validation, backend wiring ──────────────
#include "rtc_controller_manager/device_backend_registry.hpp"
#include "rtc_controller_manager/rt_controller_node.hpp"
#include "rtc_urdf_bridge/xacro_processor.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>

namespace urtc = rtc;

// ── System model configuration parsing ──────────────────────────────────────

void RtControllerNode::ParseSubModels(rtc_urdf_bridge::ModelConfig& config) {
  // YAML map format (Jazzy-compatible):
  //   urdf.sub_models.<name>.root_link, urdf.sub_models.<name>.tip_link
  //
  // The map key IS the sub-model name, so no separate "name" field is needed.
  // list_parameters() returns prefixes like "urdf.sub_models.<name>".

  const auto params = list_parameters({"urdf.sub_models"}, 10);

  for (const auto& prefix : params.prefixes) {
    // Skip the "urdf.sub_models" prefix itself — we want child prefixes
    if (prefix == "urdf.sub_models")
      continue;

    // Extract model name from prefix: "urdf.sub_models.<name>" → "<name>"
    const std::string model_name = prefix.substr(std::string("urdf.sub_models.").size());
    if (model_name.empty())
      continue;

    const std::string root_key = prefix + ".root_link";
    const std::string tip_key = prefix + ".tip_link";
    if (!has_parameter(root_key) || !has_parameter(tip_key))
      continue;

    rtc_urdf_bridge::SubModelConfig sm;
    sm.name = model_name;
    sm.root_link = get_parameter(root_key).as_string();
    sm.tip_link = get_parameter(tip_key).as_string();
    config.sub_models.push_back(std::move(sm));
  }
}

void RtControllerNode::ParseTreeModels(rtc_urdf_bridge::ModelConfig& config) {
  // YAML map format (Jazzy-compatible):
  //   urdf.tree_models.<name>.root_link, urdf.tree_models.<name>.tip_links

  const auto params = list_parameters({"urdf.tree_models"}, 10);

  for (const auto& prefix : params.prefixes) {
    if (prefix == "urdf.tree_models")
      continue;

    const std::string model_name = prefix.substr(std::string("urdf.tree_models.").size());
    if (model_name.empty())
      continue;

    const std::string root_key = prefix + ".root_link";
    if (!has_parameter(root_key))
      continue;

    rtc_urdf_bridge::TreeModelConfig tm;
    tm.name = model_name;
    tm.root_link = get_parameter(root_key).as_string();

    const std::string tips_key = prefix + ".tip_links";
    if (has_parameter(tips_key)) {
      try {
        tm.tip_links = get_parameter(tips_key).as_string_array();
      } catch (const rclcpp::ParameterTypeException&) {
        // Might be flattened as individual params — skip
      }
    }

    config.tree_models.push_back(std::move(tm));
  }
}

// ── Device name configuration loading ────────────────────────────────────────

void RtControllerNode::LoadDeviceNameConfigs() {
  // Build reverse lookup: slot index → group name
  slot_to_group_name_.resize(static_cast<std::size_t>(group_slot_map_.size()));
  slot_to_sensor_layout_.assign(static_cast<std::size_t>(group_slot_map_.size()), std::nullopt);
  for (const auto& [name, slot] : group_slot_map_) {
    slot_to_group_name_[static_cast<std::size_t>(slot)] = name;
  }

  // For each active device group, read its config from parameters
  for (const auto& group_name : active_groups_) {
    urtc::DeviceNameConfig cfg;
    cfg.device_name = group_name;

    const std::string prefix = "devices." + group_name;

    // joint_state_names (required for meaningful name mapping)
    const std::string jsn_key = prefix + ".joint_state_names";
    if (has_parameter(jsn_key)) {
      cfg.joint_state_names = get_parameter(jsn_key).as_string_array();
    }

    // joint_command_names (optional — defaults to joint_state_names)
    const std::string jcn_key = prefix + ".joint_command_names";
    if (has_parameter(jcn_key)) {
      cfg.joint_command_names = get_parameter(jcn_key).as_string_array();
    }
    if (cfg.joint_command_names.empty()) {
      cfg.joint_command_names = cfg.joint_state_names;
    }

    // motor_state_names (optional — motor-space names for /hand/motor_states)
    const std::string msn_key = prefix + ".motor_state_names";
    if (has_parameter(msn_key)) {
      try {
        cfg.motor_state_names = get_parameter(msn_key).as_string_array();
      } catch (const rclcpp::ParameterTypeException&) {
        cfg.motor_state_names.clear();
      }
    }

    // sensor_names (optional)
    const std::string sn_key = prefix + ".sensor_names";
    if (has_parameter(sn_key)) {
      try {
        cfg.sensor_names = get_parameter(sn_key).as_string_array();
      } catch (const rclcpp::ParameterTypeException&) {
        // Empty YAML array [] has no type — treat as empty
        cfg.sensor_names.clear();
      }
    }

    // joint_limits (optional block — per-joint arrays)
    {
      const auto nj = cfg.joint_state_names.size();
      const std::string lp = prefix + ".joint_limits";

      auto read_double_array = [&](const std::string& key) -> std::vector<double> {
        if (!has_parameter(key))
          return {};
        return get_parameter(key).as_double_array();
      };

      auto vel = read_double_array(lp + ".max_velocity");
      auto acc = read_double_array(lp + ".max_acceleration");
      auto trq = read_double_array(lp + ".max_torque");
      auto plo = read_double_array(lp + ".position_lower");
      auto pup = read_double_array(lp + ".position_upper");

      // Only create limits if at least one array was provided
      if (!vel.empty() || !acc.empty() || !trq.empty() || !plo.empty() || !pup.empty()) {
        rtc::DeviceJointLimits lim;
        auto validate_size = [&](const std::string& name, std::vector<double>& v) {
          if (!v.empty() && v.size() != nj) {
            RCLCPP_ERROR(get_logger(),
                         "[%s] joint_limits.%s size (%zu) != joint_state_names "
                         "size (%zu)",
                         group_name.c_str(), name.c_str(), v.size(), nj);
            v.clear();
          }
        };
        validate_size("max_velocity", vel);
        validate_size("max_acceleration", acc);
        validate_size("max_torque", trq);
        validate_size("position_lower", plo);
        validate_size("position_upper", pup);

        lim.max_velocity = std::move(vel);
        lim.max_acceleration = std::move(acc);
        lim.max_torque = std::move(trq);
        lim.position_lower = std::move(plo);
        lim.position_upper = std::move(pup);
        cfg.joint_limits = std::move(lim);

        RCLCPP_INFO(get_logger(), "[%s] Joint limits loaded from YAML", group_name.c_str());
      }
    }

    // safe_position (optional — E-STOP target, per-joint)
    {
      const std::string sp_key = prefix + ".safe_position";
      if (has_parameter(sp_key)) {
        cfg.safe_position = get_parameter(sp_key).as_double_array();
        if (!cfg.safe_position.empty() &&
            cfg.safe_position.size() != cfg.joint_state_names.size()) {
          RCLCPP_ERROR(get_logger(),
                       "[%s] safe_position size (%zu) != joint_state_names size (%zu)",
                       group_name.c_str(), cfg.safe_position.size(), cfg.joint_state_names.size());
          cfg.safe_position.clear();
        }
        if (!cfg.safe_position.empty()) {
          RCLCPP_INFO(get_logger(), "[%s] Safe position loaded from YAML", group_name.c_str());
        }
      }
    }

    // sensor_layout (optional — per-group sensor packing for HandSensorState
    // and analogous packed-sensor topics). CM uses these counts only for
    // stride/offset arithmetic in its sensor callback; the field semantics
    // (barometer/ToF/etc.) live in the device-driver package.
    {
      const std::string sl_prefix = prefix + ".sensor_layout";
      const std::string pri_key = sl_prefix + ".primary_count_per_group";
      const std::string sec_key = sl_prefix + ".secondary_count_per_group";
      const std::string vpg_key = sl_prefix + ".values_per_group";
      const std::string ipg_key = sl_prefix + ".inference_values_per_group";
      const bool any = has_parameter(pri_key) || has_parameter(sec_key) || has_parameter(vpg_key) ||
                       has_parameter(ipg_key);
      if (any) {
        rtc::DeviceSensorLayout sl;
        if (has_parameter(pri_key))
          sl.primary_count_per_group = static_cast<int>(get_parameter(pri_key).as_int());
        if (has_parameter(sec_key))
          sl.secondary_count_per_group = static_cast<int>(get_parameter(sec_key).as_int());
        if (has_parameter(vpg_key)) {
          sl.values_per_group = static_cast<int>(get_parameter(vpg_key).as_int());
        } else {
          sl.values_per_group = sl.primary_count_per_group + sl.secondary_count_per_group;
        }
        if (has_parameter(ipg_key))
          sl.inference_values_per_group = static_cast<int>(get_parameter(ipg_key).as_int());
        cfg.sensor_layout = sl;
        RCLCPP_INFO(get_logger(),
                    "[%s] Sensor layout: primary=%d secondary=%d values_per_group=%d "
                    "inference_values_per_group=%d",
                    group_name.c_str(), sl.primary_count_per_group, sl.secondary_count_per_group,
                    sl.values_per_group, sl.inference_values_per_group);
      }
    }

    // URDF config (optional block)
    // Priority: per-device urdf.package/path → system-level urdf
    // root_link/tip_link: per-device → auto-resolve from sub_models/tree_models
    const std::string urdf_pkg_key = prefix + ".urdf.package";
    const std::string urdf_path_key = prefix + ".urdf.path";

    bool has_per_device_urdf = has_parameter(urdf_pkg_key) && has_parameter(urdf_path_key);
    bool has_system_urdf = !system_model_config_.urdf_path.empty();
    bool has_per_device_links =
        has_parameter(prefix + ".urdf.root_link") || has_parameter(prefix + ".urdf.tip_link");

    if (has_per_device_urdf || has_system_urdf || has_per_device_links) {
      urtc::DeviceUrdfConfig urdf_cfg;

      // Resolve URDF package/path
      if (has_per_device_urdf) {
        urdf_cfg.package = get_parameter(urdf_pkg_key).as_string();
        urdf_cfg.path = get_parameter(urdf_path_key).as_string();
      }

      // Resolve root_link / tip_link: per-device override first
      const std::string root_key = prefix + ".urdf.root_link";
      const std::string tip_key = prefix + ".urdf.tip_link";
      if (has_parameter(root_key)) {
        urdf_cfg.root_link = get_parameter(root_key).as_string();
      }
      if (has_parameter(tip_key)) {
        urdf_cfg.tip_link = get_parameter(tip_key).as_string();
      }

      // Auto-resolve from system sub_models/tree_models by device name
      if (urdf_cfg.root_link.empty() || urdf_cfg.tip_link.empty()) {
        for (const auto& sm : system_model_config_.sub_models) {
          if (sm.name == group_name) {
            if (urdf_cfg.root_link.empty())
              urdf_cfg.root_link = sm.root_link;
            if (urdf_cfg.tip_link.empty())
              urdf_cfg.tip_link = sm.tip_link;
            break;
          }
        }
      }
      if (urdf_cfg.root_link.empty()) {
        for (const auto& tm : system_model_config_.tree_models) {
          if (tm.name == group_name) {
            urdf_cfg.root_link = tm.root_link;
            // tree_model has multiple tip_links — leave tip_link empty
            break;
          }
        }
      }

      // Resolve full URDF path for validation
      std::string full_urdf_path;
      if (!urdf_cfg.package.empty()) {
        full_urdf_path =
            ament_index_cpp::get_package_share_directory(urdf_cfg.package) + "/" + urdf_cfg.path;
      } else if (has_system_urdf) {
        full_urdf_path = system_model_config_.urdf_path;
      }

      // Validate joint names against URDF
      if (!full_urdf_path.empty()) {
        try {
          pinocchio::Model model;
          if (rtc_urdf_bridge::IsXacroFile(full_urdf_path)) {
            const auto urdf_xml = rtc_urdf_bridge::ProcessXacro(full_urdf_path);
            pinocchio::urdf::buildModelFromXML(urdf_xml, model);
          } else {
            pinocchio::urdf::buildModel(full_urdf_path, model);
          }

          // Extract URDF joint names (skip universe at index 0)
          std::vector<std::string> urdf_joint_names;
          for (int j = 1; j < model.njoints; ++j) {
            urdf_joint_names.push_back(model.names[static_cast<std::size_t>(j)]);
          }

          // Check all joint_state_names exist in URDF.  The order check
          // compares YAML order against the *relative* URDF order of the same
          // subset — the YAML may legitimately list a device-local subset (e.g.
          // hand joints starting at URDF index 6 because arm joints precede
          // them in the system URDF), so absolute-index comparison would
          // misfire.  We require the URDF indices of YAML joints to be
          // strictly increasing.
          bool all_found = true;
          bool order_match = true;
          std::size_t prev_urdf_idx = 0;
          bool prev_set = false;
          for (std::size_t i = 0; i < cfg.joint_state_names.size(); ++i) {
            auto it = std::find(urdf_joint_names.begin(), urdf_joint_names.end(),
                                cfg.joint_state_names[i]);
            if (it == urdf_joint_names.end()) {
              RCLCPP_ERROR(get_logger(), "[%s] YAML joint '%s' NOT FOUND in URDF",
                           group_name.c_str(), cfg.joint_state_names[i].c_str());
              all_found = false;
            } else {
              const auto urdf_idx =
                  static_cast<std::size_t>(std::distance(urdf_joint_names.begin(), it));
              if (prev_set && urdf_idx <= prev_urdf_idx) {
                order_match = false;
              }
              prev_urdf_idx = urdf_idx;
              prev_set = true;
            }
          }

          if (!all_found) {
            std::string avail;
            for (const auto& n : urdf_joint_names) {
              avail += "  " + n + "\n";
            }
            RCLCPP_ERROR(get_logger(),
                         "[%s] Joint name mismatch with URDF.\nAvailable URDF "
                         "joints:\n%s",
                         group_name.c_str(), avail.c_str());
          } else if (!order_match) {
            std::string yaml_order;
            for (const auto& n : cfg.joint_state_names) {
              yaml_order += " " + n;
            }
            std::string urdf_order;
            for (const auto& n : urdf_joint_names) {
              auto it = std::find(cfg.joint_state_names.begin(), cfg.joint_state_names.end(), n);
              if (it != cfg.joint_state_names.end()) {
                urdf_order += " " + n;
              }
            }
            RCLCPP_WARN(get_logger(),
                        "[%s] Joint name order differs from URDF — verify "
                        "gains/limits match YAML order",
                        group_name.c_str());
            RCLCPP_WARN(get_logger(), "[%s]   YAML order:%s", group_name.c_str(),
                        yaml_order.c_str());
            RCLCPP_WARN(get_logger(), "[%s]   URDF order:%s", group_name.c_str(),
                        urdf_order.c_str());
          } else {
            RCLCPP_INFO(get_logger(), "[%s] Joint names validated against URDF (%zu joints)",
                        group_name.c_str(), cfg.joint_state_names.size());
          }

          // Validate root_link and tip_link exist in URDF frames
          if (!urdf_cfg.root_link.empty()) {
            if (!model.existFrame(urdf_cfg.root_link)) {
              RCLCPP_WARN(get_logger(), "[%s] root_link '%s' not found in URDF frames",
                          group_name.c_str(), urdf_cfg.root_link.c_str());
            }
          }
          if (!urdf_cfg.tip_link.empty()) {
            if (!model.existFrame(urdf_cfg.tip_link)) {
              RCLCPP_WARN(get_logger(), "[%s] tip_link '%s' not found in URDF frames",
                          group_name.c_str(), urdf_cfg.tip_link.c_str());
            } else {
              RCLCPP_INFO(get_logger(), "[%s] tip_link: %s", group_name.c_str(),
                          urdf_cfg.tip_link.c_str());
            }
          }

          // ── Merge joint limits with URDF ──────────────────────────────────
          // Build YAML→URDF index mapping
          const int nj = static_cast<int>(cfg.joint_state_names.size());
          std::vector<int> yaml_to_urdf(static_cast<std::size_t>(nj), -1);
          for (int i = 0; i < nj; ++i) {
            auto it = std::find(urdf_joint_names.begin(), urdf_joint_names.end(),
                                cfg.joint_state_names[static_cast<std::size_t>(i)]);
            if (it != urdf_joint_names.end()) {
              yaml_to_urdf[static_cast<std::size_t>(i)] =
                  static_cast<int>(std::distance(urdf_joint_names.begin(), it));
            }
          }

          if (cfg.joint_limits) {
            auto& lim = *cfg.joint_limits;
            for (int i = 0; i < nj; ++i) {
              const auto ui = static_cast<std::size_t>(i);
              if (yaml_to_urdf[ui] < 0)
                continue;
              // urdf_joint_names already skips universe (built from j=1),
              // so yaml_to_urdf[ui] is the direct index into q/v limit vectors.
              const auto uidx = yaml_to_urdf[ui];

              if (!lim.position_lower.empty())
                lim.position_lower[ui] =
                    std::max(lim.position_lower[ui], model.lowerPositionLimit[uidx]);
              if (!lim.position_upper.empty())
                lim.position_upper[ui] =
                    std::min(lim.position_upper[ui], model.upperPositionLimit[uidx]);
              if (!lim.max_velocity.empty())
                lim.max_velocity[ui] = std::min(lim.max_velocity[ui], model.velocityLimit[uidx]);
              if (!lim.max_torque.empty())
                lim.max_torque[ui] = std::min(lim.max_torque[ui], model.effortLimit[uidx]);
            }
            RCLCPP_INFO(get_logger(), "[%s] Joint limits merged with URDF (tighter bounds applied)",
                        group_name.c_str());
          } else {
            // No YAML limits → create from URDF only
            rtc::DeviceJointLimits lim;
            lim.position_lower.resize(static_cast<std::size_t>(nj));
            lim.position_upper.resize(static_cast<std::size_t>(nj));
            lim.max_velocity.resize(static_cast<std::size_t>(nj));
            lim.max_torque.resize(static_cast<std::size_t>(nj));
            for (int i = 0; i < nj; ++i) {
              const auto ui = static_cast<std::size_t>(i);
              if (yaml_to_urdf[ui] < 0)
                continue;
              const auto uidx = yaml_to_urdf[ui];
              lim.position_lower[ui] = model.lowerPositionLimit[uidx];
              lim.position_upper[ui] = model.upperPositionLimit[uidx];
              lim.max_velocity[ui] = model.velocityLimit[uidx];
              lim.max_torque[ui] = model.effortLimit[uidx];
            }
            cfg.joint_limits = std::move(lim);
            RCLCPP_INFO(get_logger(), "[%s] Joint limits loaded from URDF (no YAML overrides)",
                        group_name.c_str());
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(), "[%s] URDF validation failed: %s", group_name.c_str(),
                      e.what());
        }
      }  // if (!full_urdf_path.empty())

      cfg.urdf = std::move(urdf_cfg);
    }

    // Log device config summary
    {
      auto join = [](const std::vector<std::string>& v) {
        std::string s;
        for (std::size_t i = 0; i < v.size(); ++i) {
          if (i > 0)
            s += ", ";
          s += v[i];
        }
        return s;
      };
      RCLCPP_INFO(get_logger(), "Device '%s': joints(%zu)=[%s], sensors(%zu)=[%s]%s",
                  group_name.c_str(), cfg.joint_state_names.size(),
                  join(cfg.joint_state_names).c_str(), cfg.sensor_names.size(),
                  join(cfg.sensor_names).c_str(), cfg.urdf ? " [URDF]" : "");
    }

    // Populate slot-indexed sensor layout cache (used by hot-path callbacks
    // to avoid map lookup).
    if (auto slot_it = group_slot_map_.find(group_name); slot_it != group_slot_map_.end()) {
      const auto uslot = static_cast<std::size_t>(slot_it->second);
      if (uslot < slot_to_sensor_layout_.size()) {
        slot_to_sensor_layout_[uslot] = cfg.sensor_layout;
      }
    }

    device_name_configs_[group_name] = std::move(cfg);
  }
}

// ── Device backend wiring ───────────────────────────────────────────────────
//
// One DeviceBackend per active device group. CM resolves topics from the
// active controller's TopicConfig (state/motor/sensor/command), infers the
// backend type from the same YAML, creates the backend via registry, forwards
// the sensor layout (when present), wires a callback that refreshes
// E-STOP/state/digital-twin/sim-sync state, and finally Configures the
// backend so it can create its own subs/pubs on this LifecycleNode.
//
// The callback is the single bridge between backend sub callbacks (sensor
// executor) and CM's accounting state — it consolidates the timestamp
// refresh, state_received_ flag, digital-twin republish, and sim-sync
// condvar notify that used to live in three separate sub callbacks.

std::string RtControllerNode::InferBackendType(const std::string& group_name) const {
  // Inspect the first controller's topic config that has this group. All
  // active controllers share the same HW/sim adapter (single CM, one device
  // per group), so the first match is authoritative.
  for (const auto& tc : controller_topic_configs_) {
    for (const auto& [name, group] : tc.groups) {
      if (name != group_name)
        continue;
      bool has_sensor = false;
      bool has_ros2_command = false;
      for (const auto& sub : group.subscribe) {
        if (sub.role == urtc::SubscribeRole::kSensorState)
          has_sensor = true;
      }
      for (const auto& pub : group.publish) {
        if (pub.role == urtc::PublishRole::kRos2Command)
          has_ros2_command = true;
      }
      if (has_sensor)
        return "udp_hand_native";
      if (has_ros2_command)
        return "ur_driver_native";
      return "mujoco_native";
    }
  }
  return "mujoco_native";
}

namespace {

struct BackendTopicSelection {
  std::string state_topic;
  std::string motor_topic;
  std::string sensor_topic;
  std::string command_topic;
};

BackendTopicSelection SelectBackendTopics(const urtc::TopicConfig& tc,
                                          const std::string& group_name) {
  BackendTopicSelection sel;
  for (const auto& [name, group] : tc.groups) {
    if (name != group_name)
      continue;
    for (const auto& sub : group.subscribe) {
      if (sub.ownership == urtc::TopicOwnership::kController)
        continue;
      switch (sub.role) {
        case urtc::SubscribeRole::kState:
          if (sel.state_topic.empty())
            sel.state_topic = sub.topic_name;
          break;
        case urtc::SubscribeRole::kMotorState:
          if (sel.motor_topic.empty())
            sel.motor_topic = sub.topic_name;
          break;
        case urtc::SubscribeRole::kSensorState:
          if (sel.sensor_topic.empty())
            sel.sensor_topic = sub.topic_name;
          break;
        default:
          break;
      }
    }
    for (const auto& pub : group.publish) {
      if (pub.ownership == urtc::TopicOwnership::kController)
        continue;
      if ((pub.role == urtc::PublishRole::kJointCommand ||
           pub.role == urtc::PublishRole::kRos2Command) &&
          sel.command_topic.empty()) {
        sel.command_topic = pub.topic_name;
      }
    }
  }
  return sel;
}

}  // namespace

void RtControllerNode::CreateDeviceBackends() {
  auto& registry = urtc::DeviceBackendRegistry::Instance();

  for (const auto& [group_name, slot] : group_slot_map_) {
    const std::size_t uslot = static_cast<std::size_t>(slot);
    if (uslot >= kMaxDevices) {
      RCLCPP_ERROR(get_logger(), "Group '%s' slot %d exceeds kMaxDevices (%d) — skip backend",
                   group_name.c_str(), slot, kMaxDevices);
      continue;
    }

    BackendTopicSelection topics;
    // Walk every controller's topic config and merge — different controllers
    // may declare the same group with different command roles (joint vs
    // ros2). We accept the first non-empty value per lane.
    for (const auto& tc : controller_topic_configs_) {
      const auto sel = SelectBackendTopics(tc, group_name);
      if (topics.state_topic.empty())
        topics.state_topic = sel.state_topic;
      if (topics.motor_topic.empty())
        topics.motor_topic = sel.motor_topic;
      if (topics.sensor_topic.empty())
        topics.sensor_topic = sel.sensor_topic;
      if (topics.command_topic.empty())
        topics.command_topic = sel.command_topic;
    }

    urtc::DeviceBackendConfig cfg;
    cfg.group_name = group_name;
    cfg.type = InferBackendType(group_name);
    cfg.state_topic = std::move(topics.state_topic);
    cfg.motor_topic = std::move(topics.motor_topic);
    cfg.sensor_topic = std::move(topics.sensor_topic);
    cfg.command_topic = std::move(topics.command_topic);
    if (auto name_it = device_name_configs_.find(group_name);
        name_it != device_name_configs_.end()) {
      cfg.joint_command_names = name_it->second.joint_command_names;
    }

    auto backend = registry.Create(cfg.type);
    if (!backend) {
      RCLCPP_ERROR(get_logger(),
                   "DeviceBackendRegistry: no backend registered for type '%s' (group '%s') — "
                   "check that integrated_bringup (or your bringup package) is linked with "
                   "--whole-archive",
                   cfg.type.c_str(), group_name.c_str());
      continue;
    }

    // Forward sensor packing layout before Configure() so udp_hand_native
    // can accept sensor messages from the first packet. Layout is silently
    // ignored by other backend types.
    if (uslot < slot_to_sensor_layout_.size() && slot_to_sensor_layout_[uslot].has_value()) {
      backend->SetSensorLayout(slot_to_sensor_layout_[uslot].value());
    }

    // Find DeviceTimeoutEntry for this group (built earlier in
    // DeclareAndLoadParameters/CreateTimers; index stable for node
    // lifetime). Used by the state-ready callback below to refresh
    // last_update for the E-STOP watchdog.
    int dt_idx = -1;
    for (std::size_t i = 0; i < device_timeouts_.size(); ++i) {
      if (device_timeouts_[i].group_name == group_name) {
        dt_idx = static_cast<int>(i);
        break;
      }
    }

    backend->SetStateReadyCallback([this, slot, dt_idx]() {
      // Watchdog timestamp refresh (replaces sub-lambda accounting).
      if (dt_idx >= 0) {
        const auto dti = static_cast<std::size_t>(dt_idx);
        device_timeouts_[dti].last_update = std::chrono::steady_clock::now();
        device_timeouts_[dti].received.store(true, std::memory_order_relaxed);
      }
      state_received_.store(true, std::memory_order_release);

      // Digital-twin republish (RELIABLE/10) — single hop into the joint
      // cache via ReadState; trivially copyable POD.
      auto dt_it = slot_to_dt_topic_.find(slot);
      if (dt_it != slot_to_dt_topic_.end()) {
        auto pub_it = digital_twin_publishers_.find(dt_it->second);
        if (pub_it != digital_twin_publishers_.end()) {
          auto& dte = pub_it->second;
          urtc::DeviceStateCache cache{};
          (void)backends_[static_cast<std::size_t>(slot)]->ReadState(cache);
          const auto n = dte.msg.position.size();
          for (std::size_t i = 0; i < n; ++i) {
            dte.msg.position[i] = cache.positions[i];
            dte.msg.velocity[i] = cache.velocities[i];
            dte.msg.effort[i] = cache.efforts[i];
          }
          dte.publisher->publish(dte.msg);
        }
      }

      // Sim-sync wake-up
      if (use_sim_time_sync_) {
        state_fresh_.store(true, std::memory_order_release);
        state_cv_.notify_one();
      }
    });

    backend->Configure(this, cfg);
    backends_[uslot] = std::move(backend);

    RCLCPP_INFO(get_logger(),
                "  Backend [%s]: type=%s slot=%d state='%s' motor='%s' sensor='%s' cmd='%s'",
                group_name.c_str(), cfg.type.c_str(), slot, cfg.state_topic.c_str(),
                cfg.motor_topic.c_str(), cfg.sensor_topic.c_str(), cfg.command_topic.c_str());
  }
}
