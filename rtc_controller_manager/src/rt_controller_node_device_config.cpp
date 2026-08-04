// ── Device name configuration, URDF validation, backend wiring ──────────────
#include "rtc_controller_manager/device_backend_registry.hpp"
#include "rtc_controller_manager/rt_controller_node.hpp"
#include "rtc_urdf_bridge/xacro_processor.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <sys/eventfd.h>  // eventfd_write (state-lane mailbox signal)

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

bool RtControllerNode::LoadDeviceNameConfigs() {
  bool config_invalid = false;
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
    // Every fixed-size per-channel array in the RT path (DeviceState,
    // DeviceOutput, the ingress reorder buffer) is kMaxDeviceChannels wide, and
    // the ingress reorder indexes them by position in this list. A device that
    // declares more joints than that cannot be represented, so refuse the
    // config instead of writing past the buffers at runtime (issue #196 §2).
    if (cfg.joint_state_names.size() > static_cast<std::size_t>(urtc::kMaxDeviceChannels)) {
      RCLCPP_ERROR(get_logger(),
                   "[%s] joint_state_names declares %zu joints, exceeding the "
                   "kMaxDeviceChannels capacity of %d",
                   group_name.c_str(), cfg.joint_state_names.size(), urtc::kMaxDeviceChannels);
      config_invalid = true;
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

    // backend (Phase 4 SSoT — HW/sim adapter type + wire-format topics).
    // Required keys: type, state_topic, command_topic.
    // Optional keys: motor_topic, sensor_topic.
    // Without this block CreateDeviceBackends() will refuse to create a
    // backend for the group.
    {
      const std::string b_prefix = prefix + ".backend";
      const std::string type_key = b_prefix + ".type";
      if (has_parameter(type_key)) {
        rtc::DeviceBackendBinding b;
        b.type = get_parameter(type_key).as_string();
        const std::string st_key = b_prefix + ".state_topic";
        if (has_parameter(st_key))
          b.state_topic = get_parameter(st_key).as_string();
        const std::string ct_key = b_prefix + ".command_topic";
        if (has_parameter(ct_key))
          b.command_topic = get_parameter(ct_key).as_string();
        const std::string mt_key = b_prefix + ".motor_topic";
        if (has_parameter(mt_key))
          b.motor_topic = get_parameter(mt_key).as_string();
        const std::string sn_key2 = b_prefix + ".sensor_topic";
        if (has_parameter(sn_key2))
          b.sensor_topic = get_parameter(sn_key2).as_string();
        if (b.state_topic.empty() || b.command_topic.empty()) {
          RCLCPP_ERROR(get_logger(),
                       "[%s] devices.%s.backend missing required state_topic / command_topic — "
                       "ignoring block",
                       group_name.c_str(), group_name.c_str());
        } else {
          RCLCPP_INFO(get_logger(),
                      "[%s] Backend binding: type=%s state='%s' cmd='%s' motor='%s' sensor='%s'",
                      group_name.c_str(), b.type.c_str(), b.state_topic.c_str(),
                      b.command_topic.c_str(), b.motor_topic.c_str(), b.sensor_topic.c_str());
          cfg.backend = std::move(b);
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
      const std::string hnc_key = sl_prefix + ".has_native_contact";
      const std::string hnd_key = sl_prefix + ".has_native_displacement";
      const bool any = has_parameter(pri_key) || has_parameter(sec_key) || has_parameter(vpg_key) ||
                       has_parameter(ipg_key) || has_parameter(hnc_key) || has_parameter(hnd_key);
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
        if (has_parameter(hnc_key))
          sl.has_native_contact = get_parameter(hnc_key).as_bool();
        if (has_parameter(hnd_key))
          sl.has_native_displacement = get_parameter(hnd_key).as_bool();
        cfg.sensor_layout = sl;
        RCLCPP_INFO(get_logger(),
                    "[%s] Sensor layout: primary=%d secondary=%d values_per_group=%d "
                    "inference_values_per_group=%d has_native_contact=%d "
                    "has_native_displacement=%d",
                    group_name.c_str(), sl.primary_count_per_group, sl.secondary_count_per_group,
                    sl.values_per_group, sl.inference_values_per_group,
                    static_cast<int>(sl.has_native_contact),
                    static_cast<int>(sl.has_native_displacement));
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
                lim.max_velocity[ui] =
                    std::min(lim.max_velocity[ui], model.upperVelocityLimit[uidx]);
              if (!lim.max_torque.empty())
                lim.max_torque[ui] = std::min(lim.max_torque[ui], model.upperEffortLimit[uidx]);
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
              lim.max_velocity[ui] = model.upperVelocityLimit[uidx];
              lim.max_torque[ui] = model.upperEffortLimit[uidx];
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

  return !config_invalid;
}

// ── Device backend wiring ───────────────────────────────────────────────────
//
// One DeviceBackend per active device group. Phase 4: backend type +
// wire-format topics come directly from devices.<group>.backend in
// sim.yaml/robot.yaml (parsed by LoadDeviceNameConfigs). CM creates the
// backend via registry, forwards the sensor layout (when present), wires a
// callback that refreshes E-STOP/state/digital-twin/sim-sync state, and
// finally Configures the backend so it can create its own subs/pubs on this
// LifecycleNode.
//
// The callback is the single bridge between backend sub callbacks (sensor
// executor) and CM's accounting state — it consolidates the timestamp
// refresh, state_received_ flag, digital-twin republish, and sim-sync
// condvar notify that used to live in three separate sub callbacks.

void RtControllerNode::CreateDeviceBackends() {
  auto& registry = urtc::DeviceBackendRegistry::Instance();

  // Per-slot DeviceCapability bitmask, derived from the backend binding +
  // backend impl (HasMotorState/HasSensorState) and a sensor layout if any.
  // Propagated into ControllerSlotMapping by RebindControllerCapabilities()
  // after every backend is created.
  slot_to_capability_.assign(static_cast<std::size_t>(group_slot_map_.size()), 0);

  for (const auto& [group_name, slot] : group_slot_map_) {
    const std::size_t uslot = static_cast<std::size_t>(slot);
    if (uslot >= kMaxDevices) {
      RCLCPP_ERROR(get_logger(), "Group '%s' slot %d exceeds kMaxDevices (%d) — skip backend",
                   group_name.c_str(), slot, kMaxDevices);
      continue;
    }

    auto name_it = device_name_configs_.find(group_name);
    if (name_it == device_name_configs_.end() || !name_it->second.backend.has_value()) {
      RCLCPP_ERROR(get_logger(),
                   "Group '%s' has no devices.%s.backend block in YAML — skip backend. "
                   "Phase 4 requires devices.<group>.backend: {type, state_topic, command_topic} "
                   "in sim.yaml/robot.yaml.",
                   group_name.c_str(), group_name.c_str());
      continue;
    }
    const auto& binding = *name_it->second.backend;

    urtc::DeviceBackendConfig cfg;
    cfg.group_name = group_name;
    cfg.type = binding.type;
    cfg.state_topic = binding.state_topic;
    cfg.motor_topic = binding.motor_topic;
    cfg.sensor_topic = binding.sensor_topic;
    cfg.command_topic = binding.command_topic;
    cfg.joint_command_names = name_it->second.joint_command_names;

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

    backend->SetStateReadyCallback([this, slot]() {
      // No watchdog bookkeeping here any more: liveness is read straight from
      // the backend's own LastStateStamp(), which it publishes before invoking
      // this callback. The copy CM used to keep was both redundant and a data
      // race against the RT thread (issue #198 §1/§2).

      // Mailbox only (issue #198 Phase 2). This runs on cb_group_rt_callback_
      // (SCHED_FIFO 70) — the controller↔hardware RT boundary — so it may do
      // nothing but publish a bit and poke a file descriptor. The state
      // payload itself needs no hand-off: it already sits in the backend's
      // own SeqLock, written just before this callback fires.
      dt_dirty_[static_cast<std::size_t>(slot)].store(true, std::memory_order_release);
      // Load once: the fd is cleared to -1 by the lifecycle teardown paths
      // (issue #224), so re-reading it between the guard and the write would
      // widen the check-then-use gap rather than narrow it.
      const int nrt_fd = nrt_publish_eventfd_.load(std::memory_order_acquire);
      if (nrt_fd >= 0) {
        static_cast<void>(eventfd_write(nrt_fd, 1));
      }

      // Sim-sync wake-up. Counter accumulates, so a message that lands while
      // the RT thread is mid-tick still wakes the following wait.
      const int sim_fd = sim_wake_eventfd_.load(std::memory_order_acquire);
      if (use_sim_time_sync_ && sim_fd >= 0) {
        static_cast<void>(eventfd_write(sim_fd, 1));
      }
    });

    // Inject cb_group_rt_callback_ so backend state-lane subs (joint /
    // motor / sensor) are dispatched on the rt_callback executor (FIFO 70,
    // Core 2), matching the controller↔hardware RT boundary (layout v4.1).
    backend->Configure(this, cfg, cb_group_rt_callback_);

    // Derive DeviceCapability bitmask from backend feature set + sensor layout.
    // RT loop uses this to skip whole memcpy blocks per slot.
    uint16_t cap = static_cast<uint16_t>(urtc::DeviceCapability::kJointState);
    if (backend->HasMotorState())
      cap |= static_cast<uint16_t>(urtc::DeviceCapability::kMotorState);
    if (backend->HasSensorState()) {
      cap |= static_cast<uint16_t>(urtc::DeviceCapability::kSensorData);
      if (uslot < slot_to_sensor_layout_.size() && slot_to_sensor_layout_[uslot].has_value() &&
          slot_to_sensor_layout_[uslot]->inference_values_per_group > 0) {
        cap |= static_cast<uint16_t>(urtc::DeviceCapability::kInference);
      }
    }
    slot_to_capability_[uslot] = cap;

    backends_[uslot] = std::move(backend);

    RCLCPP_INFO(
        get_logger(),
        "  Backend [%s]: type=%s slot=%d state='%s' motor='%s' sensor='%s' cmd='%s' cap=0x%x",
        group_name.c_str(), cfg.type.c_str(), slot, cfg.state_topic.c_str(),
        cfg.motor_topic.c_str(), cfg.sensor_topic.c_str(), cfg.command_topic.c_str(),
        static_cast<unsigned>(cap));
  }
}

// ── Per-slot command-type capability cache (issue #342) ─────────────────────
//
// The pairing walk below judges the controller-global GetCommandType(). The RT
// loop needs the same answer for a value that does not exist yet at configure
// — the per-device override the controller resolves each tick — so it cannot
// ask the backend when it needs to know. AcceptsCommandType is "Not RT —
// called once during configure" by its own contract, and calling it per tick
// would break that contract rather than merely cost a virtual call.
//
// So the answer is taken here, exhaustively: CommandType has three values, so
// a slot's full capability is three bits and the tick-path check becomes a bit
// test. Deliberately a separate walk from ValidateCommandTypePairing's — that
// one is indexed by (controller, slot) because its messages name a controller,
// while this cache is a property of the slot alone. It is nonetheless the ONLY
// place AcceptsCommandType is called: the pairing walk reads this mask, so the
// two gates cannot answer differently about the same slot.
void RtControllerNode::CacheSlotCommandTypeMasks() noexcept {
  slot_command_type_mask_.fill(0U);
  for (std::size_t slot = 0; slot < backends_.size(); ++slot) {
    if (!backends_[slot]) {
      continue;  // no backend on this slot — the RT loop skips it too
    }
    uint8_t mask = 0U;
    for (const auto ct : kAllCommandTypes) {
      if (backends_[slot]->AcceptsCommandType(ct)) {
        mask = static_cast<uint8_t>(mask | CommandTypeBit(ct));
      }
    }
    slot_command_type_mask_[slot] = mask;
  }
}

// ── Controller command type vs backend capability ───────────────────────────
//
// Two different questions are asked of the same (controller, backend) pairing,
// and they have independent answers:
//
//   transport — can this wire carry the mode at all?  (AcceptsCommandType)
//   safety    — if it can, is CM's E-STOP substitution still a *stop*?
//
// The first is a refusal, the second an advisory. A backend answering "yes" to
// transport says nothing about safety, so the advisory cannot be folded into
// the capability declaration.
//
// Transport: WriteCommand's `command_type` argument was advisory: a backend is
// free to ignore it, and ur_driver_native does, publishing every value it is
// handed into a forward_position_controller Float64MultiArray. So a torque-mode
// controller bound to it would have put newton-metres on the wire as joint
// angles — and CM's own hold command, which emits 0.0 for kTorque because it
// has no dynamic model, would have commanded the arm to its zero
// configuration rather than releasing it. That backend's source asserted the
// pairing was "validated at YAML time"; no such validation existed anywhere.
// DeviceBackend::AcceptsCommandType therefore defaults to position-only, which
// makes torque fail-closed for every backend that has not opted in — in-tree
// that is mujoco_native alone.
//
// Both checks share one walk of the pairings on purpose: a second nested loop
// over the same (controller, slot) set is a drift candidate, and the names the
// messages need are already in hand here.
//
// The transport judgment is CommandTypeIsHonoured against the cached mask, not
// a second AcceptsCommandType call (issue #342 review). Two reasons, and the
// first is a defect this fixed:
//
//   * The two gates disagreed. CommandTypeIsHonoured passes kPdFeedforward on
//     a position-only backend — the graceful fallback types.hpp documents and
//     the shipped WBC→hand lane rides on — while raw AcceptsCommandType does
//     not. So the identical wire behaviour was REFUSED when it came from a
//     controller-global type and PERMITTED when it came from a per-device
//     override on the same pairing. The tolerance is a property of the enum
//     (kPosition and kPdFeedforward both carry a position target in
//     commands[]; only kTorque changes the physical quantity), so it cannot
//     depend on which axis the value arrived through.
//   * CacheSlotCommandTypeMasks has already asked every backend about every
//     mode by the time this runs (on_configure orders it first). Re-asking the
//     virtual makes AcceptsCommandType's purity load-bearing: a backend whose
//     answer depended on a member set during Configure could reply differently
//     to the two callers, leaving the configure refusal and the tick gate
//     permanently inconsistent with nothing to observe it by.
//
// What did NOT change is the fail-closed direction that matters: kTorque on a
// backend that has not opted in is still refused here, and still rejected on
// the tick path.
bool RtControllerNode::ValidateCommandTypePairing() {
  bool ok = true;
  for (std::size_t ci = 0; ci < controllers_.size(); ++ci) {
    const auto ct = controllers_[ci]->GetCommandType();
    const auto& mapping = controller_slot_mappings_[ci];
    for (int gi = 0; gi < mapping.num_groups && gi < ControllerSlotMapping::kMaxSlots; ++gi) {
      const int slot = mapping.slots[static_cast<std::size_t>(gi)];
      if (slot < 0 || slot >= kMaxDevices || !backends_[static_cast<std::size_t>(slot)]) {
        continue;  // backend bring-up already failed or was refused elsewhere
      }
      const char* group = slot < static_cast<int>(slot_to_group_name_.size())
                              ? slot_to_group_name_[static_cast<std::size_t>(slot)].c_str()
                              : "?";
      if (!CommandTypeIsHonoured(slot_command_type_mask_[static_cast<std::size_t>(slot)], ct)) {
        RCLCPP_ERROR(get_logger(),
                     "Controller '%s' emits command_type '%s', which the backend on slot %d "
                     "(group '%s') cannot honour — it would be reinterpreted on the wire. "
                     "Refusing to configure.",
                     controllers_[ci]->Name().data(), urtc::CommandTypeToString(ct), slot, group);
        ok = false;
        continue;  // refused already; the sag advisory below would be noise
      }

      // Safety advisory (issue #339). The pairing is legal, so this does not
      // refuse — refusing would also block torque development in sim, which is
      // the only in-tree configuration that reaches here.
      //
      // What the operator is not otherwise told: BuildHoldOutput() servos
      // kPosition / kPdFeedforward to the measured position, which is a true
      // stop, but has no equivalent for kTorque. CM carries no dynamic model,
      // so 0 N·m is the only value it can honestly emit and the arm sags under
      // gravity until the escalation lands.
      //
      // The exposure window is reported as measured rather than described,
      // because it is rate-derived: DeclareAndLoadParameters() has already run
      // (on_configure calls it before this), so control_rate_ and
      // invalid_output_estop_ticks_ are both final here.
      //
      // It deliberately does NOT tell the operator to shorten that window.
      // kOutputRejectEstopSeconds is a compile-time constant with no YAML knob,
      // and above the tick floor a higher control_rate leaves the wall-clock
      // window unchanged — advice to "tune it down" would point at nothing.
      if (ct == urtc::CommandType::kTorque) {
        const double window_ms =
            control_rate_ > 0.0
                ? 1000.0 * static_cast<double>(invalid_output_estop_ticks_) / control_rate_
                : 0.0;
        RCLCPP_WARN(get_logger(),
                    "Controller '%s' emits command_type 'torque' on slot %d (group '%s'), and the "
                    "backend honours it — but CM's E-STOP substitution for torque is 0 N.m, not a "
                    "stop: it has no dynamic model to synthesise gravity compensation, so this "
                    "group sags for up to %.0f ms (%lu ticks at %.0f Hz) before the escalation "
                    "lands. That window is a compile-time constant and cannot be shortened from "
                    "config. Ensure this group's actuators are safe to release.",
                    controllers_[ci]->Name().data(), slot, group, window_ms,
                    static_cast<unsigned long>(invalid_output_estop_ticks_), control_rate_);
      }
    }
  }
  return ok;
}

void RtControllerNode::PropagateCapabilitiesIntoMappings() {
  // Patch each controller's slot mapping with the capability bitmask freshly
  // derived from the backend impls. Must run after CreateDeviceBackends.
  for (auto& mapping : controller_slot_mappings_) {
    for (int gi = 0; gi < mapping.num_groups && gi < ControllerSlotMapping::kMaxSlots; ++gi) {
      const auto gidx = static_cast<std::size_t>(gi);
      const auto slot = static_cast<std::size_t>(mapping.slots[gidx]);
      if (slot < slot_to_capability_.size()) {
        mapping.capabilities[gidx] = slot_to_capability_[slot];
      }
    }
  }
}
