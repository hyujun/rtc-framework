// ── Device name configuration, URDF validation, reorder maps ─────────────────
#include "rtc_controller_manager/rt_controller_node.hpp"

#include "rtc_urdf_bridge/xacro_processor.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>

namespace urtc = rtc;

// ── System model configuration parsing ──────────────────────────────────────

void RtControllerNode::ParseSubModels(
    rtc_urdf_bridge::ModelConfig & config)
{
  // YAML map format (Jazzy-compatible):
  //   urdf.sub_models.<name>.root_link, urdf.sub_models.<name>.tip_link
  //
  // The map key IS the sub-model name, so no separate "name" field is needed.
  // list_parameters() returns prefixes like "urdf.sub_models.<name>".

  const auto params = list_parameters({"urdf.sub_models"}, 10);

  for (const auto & prefix : params.prefixes) {
    // Skip the "urdf.sub_models" prefix itself — we want child prefixes
    if (prefix == "urdf.sub_models") continue;

    // Extract model name from prefix: "urdf.sub_models.<name>" → "<name>"
    const std::string model_name = prefix.substr(std::string("urdf.sub_models.").size());
    if (model_name.empty()) continue;

    const std::string root_key = prefix + ".root_link";
    const std::string tip_key  = prefix + ".tip_link";
    if (!has_parameter(root_key) || !has_parameter(tip_key)) continue;

    rtc_urdf_bridge::SubModelConfig sm;
    sm.name = model_name;
    sm.root_link = get_parameter(root_key).as_string();
    sm.tip_link = get_parameter(tip_key).as_string();
    config.sub_models.push_back(std::move(sm));
  }
}

void RtControllerNode::ParseTreeModels(
    rtc_urdf_bridge::ModelConfig & config)
{
  // YAML map format (Jazzy-compatible):
  //   urdf.tree_models.<name>.root_link, urdf.tree_models.<name>.tip_links

  const auto params = list_parameters({"urdf.tree_models"}, 10);

  for (const auto & prefix : params.prefixes) {
    if (prefix == "urdf.tree_models") continue;

    const std::string model_name = prefix.substr(std::string("urdf.tree_models.").size());
    if (model_name.empty()) continue;

    const std::string root_key = prefix + ".root_link";
    if (!has_parameter(root_key)) continue;

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

void RtControllerNode::LoadDeviceNameConfigs()
{
  // Build reverse lookup: slot index → group name
  slot_to_group_name_.resize(static_cast<std::size_t>(group_slot_map_.size()));
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
        if (!has_parameter(key)) return {};
        return get_parameter(key).as_double_array();
      };

      auto vel  = read_double_array(lp + ".max_velocity");
      auto acc  = read_double_array(lp + ".max_acceleration");
      auto trq  = read_double_array(lp + ".max_torque");
      auto plo  = read_double_array(lp + ".position_lower");
      auto pup  = read_double_array(lp + ".position_upper");

      // Only create limits if at least one array was provided
      if (!vel.empty() || !acc.empty() || !trq.empty() || !plo.empty() || !pup.empty()) {
        rtc::DeviceJointLimits lim;
        auto validate_size = [&](const std::string& name, std::vector<double>& v) {
          if (!v.empty() && v.size() != nj) {
            RCLCPP_ERROR(get_logger(),
                         "[%s] joint_limits.%s size (%zu) != joint_state_names size (%zu)",
                         group_name.c_str(), name.c_str(), v.size(), nj);
            v.clear();
          }
        };
        validate_size("max_velocity", vel);
        validate_size("max_acceleration", acc);
        validate_size("max_torque", trq);
        validate_size("position_lower", plo);
        validate_size("position_upper", pup);

        lim.max_velocity     = std::move(vel);
        lim.max_acceleration = std::move(acc);
        lim.max_torque       = std::move(trq);
        lim.position_lower   = std::move(plo);
        lim.position_upper   = std::move(pup);
        cfg.joint_limits     = std::move(lim);

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
                       group_name.c_str(), cfg.safe_position.size(),
                       cfg.joint_state_names.size());
          cfg.safe_position.clear();
        }
        if (!cfg.safe_position.empty()) {
          RCLCPP_INFO(get_logger(), "[%s] Safe position loaded from YAML",
                      group_name.c_str());
        }
      }
    }

    // URDF config (optional block)
    // Priority: per-device urdf.package/path → system-level urdf
    // root_link/tip_link: per-device → auto-resolve from sub_models/tree_models
    const std::string urdf_pkg_key = prefix + ".urdf.package";
    const std::string urdf_path_key = prefix + ".urdf.path";

    bool has_per_device_urdf = has_parameter(urdf_pkg_key) && has_parameter(urdf_path_key);
    bool has_system_urdf = !system_model_config_.urdf_path.empty();
    bool has_per_device_links = has_parameter(prefix + ".urdf.root_link") ||
                                has_parameter(prefix + ".urdf.tip_link");

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
            if (urdf_cfg.root_link.empty()) urdf_cfg.root_link = sm.root_link;
            if (urdf_cfg.tip_link.empty()) urdf_cfg.tip_link = sm.tip_link;
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
        full_urdf_path = ament_index_cpp::get_package_share_directory(urdf_cfg.package) +
            "/" + urdf_cfg.path;
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

        // Check all joint_state_names exist in URDF
        bool all_found = true;
        bool order_match = true;
        for (std::size_t i = 0; i < cfg.joint_state_names.size(); ++i) {
          auto it = std::find(urdf_joint_names.begin(), urdf_joint_names.end(),
                              cfg.joint_state_names[i]);
          if (it == urdf_joint_names.end()) {
            RCLCPP_ERROR(get_logger(), "[%s] YAML joint '%s' NOT FOUND in URDF",
                         group_name.c_str(), cfg.joint_state_names[i].c_str());
            all_found = false;
          } else {
            const auto urdf_idx = static_cast<std::size_t>(
                std::distance(urdf_joint_names.begin(), it));
            if (urdf_idx != i) { order_match = false; }
          }
        }

        if (!all_found) {
          std::string avail;
          for (const auto& n : urdf_joint_names) { avail += "  " + n + "\n"; }
          RCLCPP_ERROR(get_logger(),
                       "[%s] Joint name mismatch with URDF.\nAvailable URDF joints:\n%s",
                       group_name.c_str(), avail.c_str());
        } else if (!order_match) {
          RCLCPP_WARN(get_logger(),
                      "[%s] Joint name order differs from URDF — verify gains/limits match YAML order",
                      group_name.c_str());
        } else {
          RCLCPP_INFO(get_logger(),
                      "[%s] Joint names validated against URDF (%zu joints)",
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
            RCLCPP_INFO(get_logger(), "[%s] tip_link: %s",
                        group_name.c_str(), urdf_cfg.tip_link.c_str());
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
            if (yaml_to_urdf[ui] < 0) continue;
            // urdf_joint_names already skips universe (built from j=1),
            // so yaml_to_urdf[ui] is the direct index into q/v limit vectors.
            const auto uidx = yaml_to_urdf[ui];

            if (!lim.position_lower.empty())
              lim.position_lower[ui] = std::max(lim.position_lower[ui],
                                                model.lowerPositionLimit[uidx]);
            if (!lim.position_upper.empty())
              lim.position_upper[ui] = std::min(lim.position_upper[ui],
                                                model.upperPositionLimit[uidx]);
            if (!lim.max_velocity.empty())
              lim.max_velocity[ui] = std::min(lim.max_velocity[ui],
                                              model.velocityLimit[uidx]);
            if (!lim.max_torque.empty())
              lim.max_torque[ui] = std::min(lim.max_torque[ui],
                                            model.effortLimit[uidx]);
          }
          RCLCPP_INFO(get_logger(),
                      "[%s] Joint limits merged with URDF (tighter bounds applied)",
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
            if (yaml_to_urdf[ui] < 0) continue;
            const auto uidx = yaml_to_urdf[ui];
            lim.position_lower[ui] = model.lowerPositionLimit[uidx];
            lim.position_upper[ui] = model.upperPositionLimit[uidx];
            lim.max_velocity[ui]   = model.velocityLimit[uidx];
            lim.max_torque[ui]     = model.effortLimit[uidx];
          }
          cfg.joint_limits = std::move(lim);
          RCLCPP_INFO(get_logger(),
                      "[%s] Joint limits loaded from URDF (no YAML overrides)",
                      group_name.c_str());
        }
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(), "[%s] URDF validation failed: %s",
                      group_name.c_str(), e.what());
        }
      }  // if (!full_urdf_path.empty())

      cfg.urdf = std::move(urdf_cfg);
    }

    // Log device config summary
    {
      auto join = [](const std::vector<std::string>& v) {
        std::string s;
        for (std::size_t i = 0; i < v.size(); ++i) {
          if (i > 0) s += ", ";
          s += v[i];
        }
        return s;
      };
      RCLCPP_INFO(get_logger(), "Device '%s': joints(%zu)=[%s], sensors(%zu)=[%s]%s",
                  group_name.c_str(),
                  cfg.joint_state_names.size(), join(cfg.joint_state_names).c_str(),
                  cfg.sensor_names.size(), join(cfg.sensor_names).c_str(),
                  cfg.urdf ? " [URDF]" : "");
    }

    device_name_configs_[group_name] = std::move(cfg);
  }
}

void RtControllerNode::BuildDeviceReorderMap(
    int device_slot, const std::vector<std::string>& msg_names)
{
  // Look up reference names from device config via slot → group name
  const auto slot_idx = static_cast<std::size_t>(device_slot);
  if (slot_idx >= slot_to_group_name_.size()) return;
  const auto& group_name = slot_to_group_name_[slot_idx];
  auto it = device_name_configs_.find(group_name);
  if (it == device_name_configs_.end()) return;
  const auto& ref_names = it->second.joint_state_names;
  if (ref_names.empty()) return;

  auto& map = device_reorder_maps_[static_cast<std::size_t>(device_slot)];
  map.reorder.assign(msg_names.size(), -1);  // clear + resize (was resize, kept stale data)

  for (std::size_t msg_i = 0; msg_i < msg_names.size(); ++msg_i) {
    for (std::size_t ref_i = 0; ref_i < ref_names.size(); ++ref_i) {
      if (msg_names[msg_i] == ref_names[ref_i]) {
        map.reorder[msg_i] = static_cast<int>(ref_i);
        break;
      }
    }
  }
  map.built = true;
  RCLCPP_INFO(get_logger(), "Built device reorder map for slot %d (%zu msg names → %zu ref names)",
              device_slot, msg_names.size(), ref_names.size());
}
