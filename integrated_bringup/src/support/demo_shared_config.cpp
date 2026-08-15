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
  if (fp["K_est_max"])
    gp.K_est_max = fp["K_est_max"].as<double>();
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
  if (fp["f_slip_fraction"])
    gp.f_slip_fraction = fp["f_slip_fraction"].as<double>();
  // 제거된 키를 조용히 넘기지 않는다. 이 로더는 모르는 키를 무시하므로, 의미가
  // ratio(per tick) -> rate(N/s) 로 바뀐 상황에서 침묵은 곧 "YAML 이 지정한
  // 값이 사라지고 default 가 대신 도는" 것이다. 값이 아니라 단위가 바뀌었으니
  // 자동 변환도 불가능하다 — 실패시키는 것이 유일하게 옳다 (비-RT, LoadConfig).
  if (fp["grip_tightening_ratio"]) {
    throw std::runtime_error(
        "force_pi_grasp.grip_tightening_ratio was removed: it was applied once per tick, so the "
        "grip force a slip produced depended on control_rate. Use grip_tightening_rate [N/s].");
  }
  if (fp["grip_tightening_rate"])
    gp.grip_tightening_rate = fp["grip_tightening_rate"].as<double>();
  // grip_decay_rate 는 이 블록에 없어서 **YAML 로 설정할 수 없었다** — 키를 적어도
  // 조용히 무시되고 default 0.1 N/s 가 돌았다. tightening 이 rate 가 된 지금
  // 둘은 같은 축의 짝이므로 함께 노출한다.
  if (fp["grip_decay_rate"])
    gp.grip_decay_rate = fp["grip_decay_rate"].as<double>();
  if (fp["f_max_multiplier"])
    gp.f_max_multiplier = fp["f_max_multiplier"].as<double>();
  // Same key, same meaning, same tuning advice — the filter it configures lives
  // in the controller (per axis, upstream of the |F| collapse) rather than
  // inside GraspController, which no longer holds one.
  if (fp["lpf_cutoff_hz"])
    cfg.force_pi_lpf_cutoff_hz = fp["lpf_cutoff_hz"].as<double>();

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

    // Which slot is the thumb (#432). The Approaching -> Contact transition
    // needs the thumb plus any one other finger, and rtc::grasp::FingerConfig
    // carries only geometry, so this is the one finger role the config has to
    // hand over. Resolved from the same name list that keys the blocks below —
    // no second knob to keep in sync.
    //
    // Absent "thumb" keeps index 0 (GraspParams' default and the convention
    // every config here follows) and warns rather than failing: a hand without
    // a named thumb still configures, and the transition still fires — it just
    // treats slot 0 as the opposing digit. Silence would hide that.
    const auto thumb_it = std::find(names.begin(), names.end(), "thumb");
    if (thumb_it != names.end() && std::distance(names.begin(), thumb_it) < n_fingers) {
      gp.thumb_finger_index = static_cast<int>(std::distance(names.begin(), thumb_it));
    } else {
      RCLCPP_WARN(::integrated_bringup::logging::SharedConfigLogger(),
                  "force_pi_grasp.fingers.finger_names has no 'thumb' entry; the Contact "
                  "transition will treat finger 0 ('%s') as the opposing digit",
                  names.empty() ? "<none>" : names.front().c_str());
    }
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
  ApplyVector3(pe["inplane_x_reference"], pp.inplane_x_reference);

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
      // Optional independent touch hysteresis (#234 P-16). Omitting both keys
      // keeps the historical behaviour of reusing the contact pair; Init
      // rejects a half-set pair.
      if (tn["touch_on_threshold"])
        pc.touch_on_threshold = tn["touch_on_threshold"].as<double>();
      if (tn["touch_off_threshold"])
        pc.touch_off_threshold = tn["touch_off_threshold"].as<double>();
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

// #135 Layer 1b — `momentum_observer` block. Presence of the block is what
// enables the channel; `enabled: false` keeps the block (and its gains) around
// while switching it off.
void ApplyMomentumObserverBlock(const YAML::Node& mo, DemoSharedConfig& cfg) {
  if (!mo || !mo.IsMap()) {
    return;
  }
  auto& mp = cfg.momentum_observer;
  mp.has_block = true;
  if (mo["enabled"]) {
    mp.enabled = mo["enabled"].as<bool>();
  }
  if (mo["gains"]) {
    // A scalar is accepted as the one-entry broadcast form so a config does not
    // have to spell the same number dof times.
    if (mo["gains"].IsSequence()) {
      mp.gains.clear();
      for (const auto& g : mo["gains"]) {
        mp.gains.push_back(g.as<double>());
      }
    } else {
      mp.gains.assign(1, mo["gains"].as<double>());
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

  if (node["momentum_observer"]) {
    ApplyMomentumObserverBlock(node["momentum_observer"], cfg);
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

void BuildGraspController(const DemoSharedConfig& cfg,
                          std::unique_ptr<rtc::grasp::GraspController>& grasp_controller) {
  // The block, not the mode, gates construction — rationale in the header. A
  // config that ships the block gets the PI capability whatever mode it starts
  // in, so `grasp_controller_type` can be moved at runtime without a re-configure
  // (which controller switching never performs anyway).
  if (!cfg.has_force_pi_block) {
    grasp_controller.reset();
    return;
  }

  const rtc::grasp::GraspParams& gp = cfg.force_pi_params;

  grasp_controller = std::make_unique<rtc::grasp::GraspController>();
  const auto n =
      static_cast<std::size_t>(std::clamp(cfg.num_grasp_fingers, 0, rtc::grasp::kMaxGraspFingers));
  grasp_controller->Init(std::span<const rtc::grasp::FingerConfig>(cfg.force_pi_fingers.data(), n),
                         gp);
}

const char* GraspCommandRejectReason(GraspHandMode mode, bool has_controller) noexcept {
  // Capability first, then policy: "no block in your YAML" and "the block is
  // there but this mode does not run it" are different operator problems and the
  // fix for each is a different edit.
  if (!has_controller) {
    return "grasp_controller unavailable (no 'force_pi_grasp' block in demo_shared.yaml)";
  }
  if (mode != GraspHandMode::kForcePi) {
    return "grasp mode is not 'force_pi' (switch grasp_controller_type first)";
  }
  return nullptr;
}

const char* GraspModeChangeRejectReason(GraspHandMode current, GraspHandMode requested,
                                        const GraspModeSwitchState& state) noexcept {
  // No-op first, before any gate: the requested mode is already in force, so
  // accepting cannot change what the hand is doing. Refusing here would make an
  // idempotent set fail whenever an object happens to be held — including the
  // re-send a GUI does to confirm what it is already displaying.
  if (requested == current) {
    return nullptr;
  }
  // Capability before policy, same order as GraspCommandRejectReason: without a
  // force_pi_grasp block there is no PI law to switch to, and that is a YAML
  // problem rather than something waiting will fix.
  if (requested == GraspHandMode::kForcePi && !state.has_controller) {
    return "force_pi unavailable (no 'force_pi_grasp' block in demo_shared.yaml)";
  }
  // The two quiet refusals stay distinct: one is fixed by opening the hand, the
  // other by finishing or releasing the PI grasp, and this string is all the
  // operator gets.
  if (state.contact_latched) {
    return "hand is holding (contact_stop latch engaged) — release it first";
  }
  if (!state.grasp_phase_idle) {
    return "force_pi grasp is in progress — release it first";
  }
  return nullptr;
}

void BuildPullForceEstimator(const DemoSharedConfig& cfg, double control_rate_hz,
                             std::unique_ptr<rtc::grasp::PullForceEstimator>& estimator) {
  if (!cfg.has_pull_estimator_block || !cfg.pull_estimator_enabled) {
    estimator.reset();
    return;
  }

  rtc::grasp::PullEstimatorParams params = cfg.pull_estimator_params;
  params.sample_rate_hz = control_rate_hz;

  // Init() before publication: ConfigurePullEstimatorWiring's contract is that
  // every throw path leaves the wiring disabled, and `estimator != nullptr`
  // with num_contacts == 0 would read as enabled(). Build into a local, hand
  // it over only once Init (which throws on bad config) has returned.
  auto built = std::make_unique<rtc::grasp::PullForceEstimator>();
  const auto n =
      static_cast<std::size_t>(std::clamp(cfg.num_pull_contacts, 0, rtc::grasp::kMaxPullContacts));
  built->Init(std::span<const rtc::grasp::PullContactConfig>(cfg.pull_contacts.data(), n), params);
  if (cfg.pull_has_direction) {
    built->SetPullDirection(cfg.pull_direction);
  }
  estimator = std::move(built);
}

}  // namespace integrated_bringup
