#include "integrated_bringup/support/demo_shared_config.hpp"

#include "integrated_bringup/support/bringup_logging.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace integrated_bringup {

GraspHandMode ParseGraspHandMode(const std::string& type) {
  if (type == "force_pi")
    return GraspHandMode::kForcePi;
  if (type == "contact_stop")
    return GraspHandMode::kContactStop;
  if (type == "none")
    return GraspHandMode::kNone;
  throw std::runtime_error(
      "demo_shared: 'grasp_controller_type' must be one of "
      "{force_pi, contact_stop, none}, got '" +
      type + "'");
}

const char* GraspHandModeName(GraspHandMode mode) noexcept {
  switch (mode) {
    case GraspHandMode::kForcePi:
      return "force_pi";
    case GraspHandMode::kContactStop:
      return "contact_stop";
    case GraspHandMode::kNone:
      return "none";
  }
  return "contact_stop";
}

PullPlaneNormalSource ParsePullPlaneNormalSource(const std::string& source) {
  if (source == "fixed")
    return PullPlaneNormalSource::kFixed;
  if (source == "pinch_geometry")
    return PullPlaneNormalSource::kPinchGeometry;
  throw std::runtime_error(
      "demo_shared: 'pull_estimator.plane_normal_source' must be one of "
      "{fixed, pinch_geometry} ('vision' is not implemented yet), got '" +
      source + "'");
}

const char* PullPlaneNormalSourceName(PullPlaneNormalSource source) noexcept {
  switch (source) {
    case PullPlaneNormalSource::kFixed:
      return "fixed";
    case PullPlaneNormalSource::kPinchGeometry:
      return "pinch_geometry";
  }
  return "fixed";
}

namespace {

constexpr const char* kSharedYamlFile = "/controllers/demo_shared.yaml";
constexpr const char* kSharedYamlRootKey = "demo_shared";

void ApplyVirtualTcp(const YAML::Node& node, VirtualTcpConfig& vtcp) {
  if (node["virtual_tcp_mode"]) {
    const auto mode_str = node["virtual_tcp_mode"].as<std::string>();
    if (mode_str == "centroid") {
      vtcp.mode = VirtualTcpMode::kCentroid;
    } else if (mode_str == "weighted") {
      vtcp.mode = VirtualTcpMode::kWeighted;
    } else if (mode_str == "constant") {
      vtcp.mode = VirtualTcpMode::kConstant;
    } else {
      vtcp.mode = VirtualTcpMode::kDisabled;
    }
  }
  if (node["virtual_tcp_offset"] && node["virtual_tcp_offset"].IsSequence()) {
    const auto seq = node["virtual_tcp_offset"];
    for (std::size_t i = 0; i < 3 && i < seq.size(); ++i) {
      vtcp.offset[i] = seq[i].as<double>();
    }
  }
  if (node["virtual_tcp_orientation"] && node["virtual_tcp_orientation"].IsSequence()) {
    const auto seq = node["virtual_tcp_orientation"];
    for (std::size_t i = 0; i < 3 && i < seq.size(); ++i) {
      vtcp.orientation[i] = seq[i].as<double>();
    }
  }
}

void ApplyForcePiBlock(const YAML::Node& fp, DemoSharedConfig& cfg) {
  cfg.has_force_pi_block = true;
  auto& gp = cfg.force_pi_params;

  if (fp["Kp_base"])
    gp.Kp_base = fp["Kp_base"].as<double>();
  if (fp["Ki_base"])
    gp.Ki_base = fp["Ki_base"].as<double>();
  if (fp["alpha_ema"])
    gp.alpha_ema = fp["alpha_ema"].as<double>();
  if (fp["beta"])
    gp.beta = fp["beta"].as<double>();
  if (fp["f_contact_threshold"])
    gp.f_contact_threshold = fp["f_contact_threshold"].as<double>();
  if (fp["f_target"])
    gp.f_target = fp["f_target"].as<double>();
  if (fp["f_ramp_rate"])
    gp.f_ramp_rate = fp["f_ramp_rate"].as<double>();
  if (fp["ds_max"])
    gp.ds_max = fp["ds_max"].as<double>();
  if (fp["delta_s_max"])
    gp.delta_s_max = fp["delta_s_max"].as<double>();
  if (fp["integral_clamp"])
    gp.integral_clamp = fp["integral_clamp"].as<double>();
  if (fp["approach_speed"])
    gp.approach_speed = fp["approach_speed"].as<double>();
  if (fp["release_speed"])
    gp.release_speed = fp["release_speed"].as<double>();
  if (fp["settle_epsilon"])
    gp.settle_epsilon = fp["settle_epsilon"].as<double>();
  if (fp["settle_time"])
    gp.settle_time = fp["settle_time"].as<double>();
  if (fp["contact_settle_time"])
    gp.contact_settle_time = fp["contact_settle_time"].as<double>();
  if (fp["df_slip_threshold"])
    gp.df_slip_threshold = fp["df_slip_threshold"].as<double>();
  if (fp["grip_tightening_ratio"])
    gp.grip_tightening_ratio = fp["grip_tightening_ratio"].as<double>();
  if (fp["f_max_multiplier"])
    gp.f_max_multiplier = fp["f_max_multiplier"].as<double>();
  if (fp["lpf_cutoff_hz"])
    gp.lpf_cutoff_hz = fp["lpf_cutoff_hz"].as<double>();

  if (fp["fingers"]) {
    const auto fingers_node = fp["fingers"];

    // Angle unit: "rad" (default) or "deg". Internally always stored in
    // radians.
    double angle_scale = 1.0;
    if (fingers_node["units"]) {
      const auto units_str = fingers_node["units"].as<std::string>();
      if (units_str == "deg" || units_str == "degrees") {
        angle_scale = std::numbers::pi_v<double> / 180.0;
      } else if (units_str != "rad" && units_str != "radians") {
        RCLCPP_WARN(::integrated_bringup::logging::SharedConfigLogger(),
                    "force_pi_grasp.fingers.units: unknown value '%s'; "
                    "defaulting to radians",
                    units_str.c_str());
      }
    }

    // Finger name order. Defaults to the assm_v1 3-finger set; override via
    // force_pi_grasp.fingers.finger_names to support other hands (e.g. proto_1b
    // which adds "ring"). Each finger's DoF is inferred from its posture length,
    // so heterogeneous per-finger DoF (thumb:4/index:3/middle:2) are supported.
    std::vector<std::string> names = {"thumb", "index", "middle"};
    if (fingers_node["finger_names"] && fingers_node["finger_names"].IsSequence()) {
      names.clear();
      for (const auto& n : fingers_node["finger_names"]) {
        names.push_back(n.as<std::string>());
      }
    }

    const int n_fingers =
        std::min<int>(static_cast<int>(names.size()), rtc::grasp::kMaxGraspFingers);
    for (int i = 0; i < n_fingers; ++i) {
      const auto fn = fingers_node[names[static_cast<std::size_t>(i)]];
      if (!fn) {
        continue;
      }
      auto& fc = cfg.force_pi_fingers[static_cast<std::size_t>(i)];
      int dof = 0;
      if (fn["q_open"]) {
        const auto seq = fn["q_open"];
        for (int j = 0; j < rtc::grasp::kMaxDoFPerFinger && j < static_cast<int>(seq.size()); ++j) {
          fc.q_open[static_cast<std::size_t>(j)] = seq[j].as<double>() * angle_scale;
        }
        dof = std::max(dof, std::min(static_cast<int>(seq.size()), rtc::grasp::kMaxDoFPerFinger));
      }
      if (fn["q_close"]) {
        const auto seq = fn["q_close"];
        for (int j = 0; j < rtc::grasp::kMaxDoFPerFinger && j < static_cast<int>(seq.size()); ++j) {
          fc.q_close[static_cast<std::size_t>(j)] = seq[j].as<double>() * angle_scale;
        }
        dof = std::max(dof, std::min(static_cast<int>(seq.size()), rtc::grasp::kMaxDoFPerFinger));
      }
      fc.dof = dof;
    }
  }
}

// Overlay a [x, y, z] sequence onto `out`; shorter sequences leave the
// remaining components untouched (virtual_tcp_offset precedent).
void ApplyVector3(const YAML::Node& n, Eigen::Vector3d& out) {
  if (!n || !n.IsSequence()) {
    return;
  }
  for (std::size_t i = 0; i < 3 && i < n.size(); ++i) {
    out[static_cast<Eigen::Index>(i)] = n[i].as<double>();
  }
}

void ApplyPullEstimatorBlock(const YAML::Node& pe, DemoSharedConfig& cfg) {
  cfg.has_pull_estimator_block = true;
  if (pe["enabled"]) {
    cfg.pull_estimator_enabled = pe["enabled"].as<bool>();
  }

  // Estimator parameters — keys mirror rtc::grasp::PullEstimatorParams field
  // names (force_pi_grasp precedent). sample_rate_hz is intentionally not a
  // YAML key: BuildPullForceEstimator overwrites it with the control rate.
  auto& pp = cfg.pull_estimator_params;
  if (pe["filter_cutoff_hz"])
    pp.filter_cutoff_hz = pe["filter_cutoff_hz"].as<double>();
  if (pe["min_valid_contacts"])
    pp.min_valid_contacts = pe["min_valid_contacts"].as<int>();
  if (pe["decay_time_constant_s"])
    pp.decay_time_constant_s = pe["decay_time_constant_s"].as<double>();
  if (pe["slip_risk_threshold"])
    pp.slip_risk_threshold = pe["slip_risk_threshold"].as<double>();
  if (pe["alignment_error_rad"])
    pp.alignment_error_rad = pe["alignment_error_rad"].as<double>();
  ApplyVector3(pe["gravity_force"], pp.gravity_force);

  if (pe["plane_normal_source"]) {
    cfg.pull_plane_normal_source =
        ParsePullPlaneNormalSource(pe["plane_normal_source"].as<std::string>());
  }
  ApplyVector3(pe["plane_normal"], cfg.pull_plane_normal);
  if (pe["use_baseline_subtraction"]) {
    cfg.pull_use_baseline_subtraction = pe["use_baseline_subtraction"].as<bool>();
  }
  if (pe["pull_direction"]) {
    ApplyVector3(pe["pull_direction"], cfg.pull_direction);
    cfg.pull_has_direction = true;
  }

  if (pe["tips"]) {
    const auto tips_node = pe["tips"];

    // Tip role order — defines the PullContactInput order P3 wiring must
    // follow. Defaults to the assm_v1 sensored set; override via
    // pull_estimator.tips.tip_names for other hands.
    std::vector<std::string> names = {"thumb", "index", "middle"};
    if (tips_node["tip_names"] && tips_node["tip_names"].IsSequence()) {
      names.clear();
      for (const auto& n : tips_node["tip_names"]) {
        names.push_back(n.as<std::string>());
      }
    }

    const int n_tips = std::min<int>(static_cast<int>(names.size()), rtc::grasp::kMaxPullContacts);
    cfg.num_pull_contacts = n_tips;
    for (int i = 0; i < n_tips; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      cfg.pull_tip_roles[idx] = names[idx];
      const auto tn = tips_node[names[idx]];
      if (!tn) {
        continue;
      }
      auto& pc = cfg.pull_contacts[idx];
      if (tn["link"]) {
        cfg.pull_tip_links[idx] = tn["link"].as<std::string>();
      }
      if (tn["friction_coeff"])
        pc.friction_coeff = tn["friction_coeff"].as<double>();
      if (tn["contact_on_threshold"])
        pc.contact_on_threshold = tn["contact_on_threshold"].as<double>();
      if (tn["contact_off_threshold"])
        pc.contact_off_threshold = tn["contact_off_threshold"].as<double>();
      if (tn["force_saturation"])
        pc.force_saturation = tn["force_saturation"].as<double>();
      if (tn["force_sign"])
        pc.force_sign = tn["force_sign"].as<double>();
      ApplyVector3(tn["force_bias"], pc.force_bias);
      if (tn["force_calibration"] && tn["force_calibration"].IsSequence() &&
          tn["force_calibration"].size() == 9) {
        const auto seq = tn["force_calibration"];
        for (Eigen::Index r = 0; r < 3; ++r) {
          for (Eigen::Index c = 0; c < 3; ++c) {
            pc.force_calibration(r, c) = seq[static_cast<std::size_t>(r * 3 + c)].as<double>();
          }
        }
      }
    }
  }

  // Required-role mask (after tips so roles are populated). An unknown role is
  // a hard config error: a typo like "thmub" would otherwise silently drop the
  // thumb-mandatory gate and let index+middle count as a valid pinch.
  if (pe["required_roles"] && pe["required_roles"].IsSequence()) {
    for (const auto& rn : pe["required_roles"]) {
      const auto role = rn.as<std::string>();
      bool found = false;
      for (int i = 0; i < cfg.num_pull_contacts; ++i) {
        if (cfg.pull_tip_roles[static_cast<std::size_t>(i)] == role) {
          cfg.pull_contacts[static_cast<std::size_t>(i)].required = true;
          found = true;
          break;
        }
      }
      if (!found) {
        throw std::runtime_error("demo_shared: pull_estimator.required_roles entry '" + role +
                                 "' does not match any tips.tip_names role");
      }
    }
  }
}

}  // namespace

void ApplyDemoSharedConfig(const YAML::Node& node, DemoSharedConfig& cfg) {
  if (!node) {
    return;
  }

  ApplyVirtualTcp(node, cfg.vtcp);

  if (node["grasp_contact_threshold"]) {
    cfg.grasp_contact_threshold = node["grasp_contact_threshold"].as<float>();
  }
  if (node["grasp_force_threshold"]) {
    cfg.grasp_force_threshold = node["grasp_force_threshold"].as<float>();
  }
  if (node["grasp_min_fingertips"]) {
    cfg.grasp_min_fingertips = node["grasp_min_fingertips"].as<int>();
  }
  if (node["grasp_controller_type"]) {
    cfg.grasp_controller_type = node["grasp_controller_type"].as<std::string>();
    // Validate against the whitelist here (on_configure, non-RT). ParseGraspHandMode
    // throws std::runtime_error on any value outside {force_pi, contact_stop,
    // none} so an unrecognized string cannot fall through to a controller branch;
    // the throw propagates to CallbackReturn::FAILURE.
    (void)ParseGraspHandMode(cfg.grasp_controller_type);
  }

  if (node["hand_finger_joint_map"] && node["hand_finger_joint_map"].IsSequence()) {
    const auto seq = node["hand_finger_joint_map"];
    const std::size_t n_fingers =
        std::min<std::size_t>(seq.size(), cfg.hand_finger_joint_map.size());
    cfg.num_grasp_fingers = static_cast<int>(n_fingers);
    for (std::size_t f = 0; f < n_fingers; ++f) {
      if (!seq[f].IsSequence()) {
        cfg.finger_dof[f] = 0;
        continue;
      }
      const std::size_t n_joints =
          std::min<std::size_t>(seq[f].size(), cfg.hand_finger_joint_map[f].size());
      cfg.finger_dof[f] = static_cast<int>(n_joints);
      for (std::size_t j = 0; j < n_joints; ++j) {
        cfg.hand_finger_joint_map[f][j] = seq[f][j].as<int>();
      }
    }
  }

  // Release-gate: preferred `hand_release_gate` list of {joint_index, loosen_sign}.
  if (node["hand_release_gate"] && node["hand_release_gate"].IsSequence()) {
    const auto seq = node["hand_release_gate"];
    const std::size_t n = std::min<std::size_t>(seq.size(), cfg.hand_release_gate.size());
    cfg.num_release_gates = static_cast<int>(n);
    for (std::size_t f = 0; f < n; ++f) {
      const auto& g = seq[f];
      if (g["joint_index"]) {
        cfg.hand_release_gate[f].joint_index = g["joint_index"].as<int>();
      }
      if (g["loosen_sign"]) {
        cfg.hand_release_gate[f].loosen_sign = (g["loosen_sign"].as<int>() >= 0) ? +1 : -1;
      }
    }
  } else {
    // Legacy fallback: the three named MCP_FE scalars (assm_v1 3-finger hand).
    if (node["hand_idx_thumb_cmc_fe"]) {
      cfg.hand_release_gate[0] = {node["hand_idx_thumb_cmc_fe"].as<int>(), +1};
    }
    if (node["hand_idx_index_mcp_fe"]) {
      cfg.hand_release_gate[1] = {node["hand_idx_index_mcp_fe"].as<int>(), -1};
    }
    if (node["hand_idx_middle_mcp_fe"]) {
      cfg.hand_release_gate[2] = {node["hand_idx_middle_mcp_fe"].as<int>(), -1};
    }
  }

  if (node["force_pi_grasp"]) {
    ApplyForcePiBlock(node["force_pi_grasp"], cfg);
  }

  if (node["pull_estimator"]) {
    ApplyPullEstimatorBlock(node["pull_estimator"], cfg);
  }
}

void LoadDemoSharedYamlFile(DemoSharedConfig& cfg, const std::string& config_variant) {
  std::string yaml_path;
  try {
    yaml_path.append(ament_index_cpp::get_package_share_directory("integrated_bringup"));
    yaml_path.append("/config/");
    if (!config_variant.empty()) {
      yaml_path.append(config_variant).append("/");
    }
    // Trim the leading slash from kSharedYamlFile since we already appended one.
    yaml_path.append(kSharedYamlFile + 1);
  } catch (const std::exception& e) {
    RCLCPP_WARN(::integrated_bringup::logging::SharedConfigLogger(),
                "demo_shared.yaml: package share dir lookup failed (%s); "
                "skipping shared defaults",
                e.what());
    return;
  }

  try {
    const YAML::Node file_node = YAML::LoadFile(yaml_path);
    ApplyDemoSharedConfig(file_node[kSharedYamlRootKey], cfg);
  } catch (const YAML::Exception& e) {
    RCLCPP_WARN(::integrated_bringup::logging::SharedConfigLogger(),
                "demo_shared.yaml: load failed at %s (%s); using built-in defaults",
                yaml_path.c_str(), e.what());
    return;
  }

  // Success path: record which knobs actually took effect. This is the
  // single line to grep when the operator suspects a controller is picking
  // up stale or unexpected shared defaults.
  const char* vtcp_mode_str = "disabled";
  switch (cfg.vtcp.mode) {
    case VirtualTcpMode::kCentroid:
      vtcp_mode_str = "centroid";
      break;
    case VirtualTcpMode::kWeighted:
      vtcp_mode_str = "weighted";
      break;
    case VirtualTcpMode::kConstant:
      vtcp_mode_str = "constant";
      break;
    case VirtualTcpMode::kDisabled:
      vtcp_mode_str = "disabled";
      break;
  }
  RCLCPP_INFO(::integrated_bringup::logging::SharedConfigLogger(),
              "demo_shared.yaml loaded: vtcp=%s grasp_type=%s force_pi_block=%s "
              "contact_thresh=%.2fN force_thresh=%.2fN min_fingers=%d",
              vtcp_mode_str, cfg.grasp_controller_type.c_str(),
              cfg.has_force_pi_block ? "yes" : "no",
              static_cast<double>(cfg.grasp_contact_threshold),
              static_cast<double>(cfg.grasp_force_threshold), cfg.grasp_min_fingertips);
}

void BuildGraspController(const DemoSharedConfig& cfg, double control_rate_hz,
                          std::unique_ptr<rtc::grasp::GraspController>& grasp_controller) {
  if (cfg.grasp_controller_type != "force_pi" || !cfg.has_force_pi_block) {
    grasp_controller.reset();
    return;
  }

  rtc::grasp::GraspParams gp = cfg.force_pi_params;
  gp.control_rate_hz = control_rate_hz;

  grasp_controller = std::make_unique<rtc::grasp::GraspController>();
  const auto n =
      static_cast<std::size_t>(std::clamp(cfg.num_grasp_fingers, 0, rtc::grasp::kMaxGraspFingers));
  grasp_controller->Init(std::span<const rtc::grasp::FingerConfig>(cfg.force_pi_fingers.data(), n),
                         gp);
}

void BuildPullForceEstimator(const DemoSharedConfig& cfg, double control_rate_hz,
                             std::unique_ptr<rtc::grasp::PullForceEstimator>& estimator) {
  if (!cfg.has_pull_estimator_block || !cfg.pull_estimator_enabled) {
    estimator.reset();
    return;
  }

  rtc::grasp::PullEstimatorParams params = cfg.pull_estimator_params;
  params.sample_rate_hz = control_rate_hz;

  estimator = std::make_unique<rtc::grasp::PullForceEstimator>();
  const auto n =
      static_cast<std::size_t>(std::clamp(cfg.num_pull_contacts, 0, rtc::grasp::kMaxPullContacts));
  estimator->Init(std::span<const rtc::grasp::PullContactConfig>(cfg.pull_contacts.data(), n),
                  params);
  if (cfg.pull_has_direction) {
    estimator->SetPullDirection(cfg.pull_direction);
  }
}

}  // namespace integrated_bringup
