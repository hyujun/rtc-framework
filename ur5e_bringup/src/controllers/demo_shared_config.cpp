#include "ur5e_bringup/controllers/demo_shared_config.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace ur5e_bringup {

namespace {

constexpr const char * kSharedYamlRelPath = "/config/controllers/demo_shared.yaml";
constexpr const char * kSharedYamlRootKey = "demo_shared";

void ApplyVirtualTcp(const YAML::Node & node, VirtualTcpConfig & vtcp)
{
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

void ApplyForcePiBlock(const YAML::Node & fp, DemoSharedConfig & cfg)
{
  cfg.has_force_pi_block = true;
  auto & gp = cfg.force_pi_params;

  if (fp["Kp_base"])               gp.Kp_base = fp["Kp_base"].as<double>();
  if (fp["Ki_base"])               gp.Ki_base = fp["Ki_base"].as<double>();
  if (fp["alpha_ema"])             gp.alpha_ema = fp["alpha_ema"].as<double>();
  if (fp["beta"])                  gp.beta = fp["beta"].as<double>();
  if (fp["f_contact_threshold"])   gp.f_contact_threshold = fp["f_contact_threshold"].as<double>();
  if (fp["f_target"])              gp.f_target = fp["f_target"].as<double>();
  if (fp["f_ramp_rate"])           gp.f_ramp_rate = fp["f_ramp_rate"].as<double>();
  if (fp["ds_max"])                gp.ds_max = fp["ds_max"].as<double>();
  if (fp["delta_s_max"])           gp.delta_s_max = fp["delta_s_max"].as<double>();
  if (fp["integral_clamp"])        gp.integral_clamp = fp["integral_clamp"].as<double>();
  if (fp["approach_speed"])        gp.approach_speed = fp["approach_speed"].as<double>();
  if (fp["release_speed"])         gp.release_speed = fp["release_speed"].as<double>();
  if (fp["settle_epsilon"])        gp.settle_epsilon = fp["settle_epsilon"].as<double>();
  if (fp["settle_time"])           gp.settle_time = fp["settle_time"].as<double>();
  if (fp["contact_settle_time"])   gp.contact_settle_time = fp["contact_settle_time"].as<double>();
  if (fp["df_slip_threshold"])     gp.df_slip_threshold = fp["df_slip_threshold"].as<double>();
  if (fp["grip_tightening_ratio"]) gp.grip_tightening_ratio = fp["grip_tightening_ratio"].as<double>();
  if (fp["f_max_multiplier"])      gp.f_max_multiplier = fp["f_max_multiplier"].as<double>();
  if (fp["lpf_cutoff_hz"])         gp.lpf_cutoff_hz = fp["lpf_cutoff_hz"].as<double>();

  if (fp["fingers"]) {
    const std::array<std::string, 3> names = {"thumb", "index", "middle"};
    for (int i = 0; i < rtc::grasp::kNumGraspFingers; ++i) {
      const auto fn = fp["fingers"][names[static_cast<std::size_t>(i)]];
      if (!fn) { continue; }
      if (fn["q_open"]) {
        const auto seq = fn["q_open"];
        for (int j = 0; j < rtc::grasp::kDoFPerFinger && j < static_cast<int>(seq.size()); ++j) {
          cfg.force_pi_fingers[static_cast<std::size_t>(i)]
            .q_open[static_cast<std::size_t>(j)] = seq[j].as<double>();
        }
      }
      if (fn["q_close"]) {
        const auto seq = fn["q_close"];
        for (int j = 0; j < rtc::grasp::kDoFPerFinger && j < static_cast<int>(seq.size()); ++j) {
          cfg.force_pi_fingers[static_cast<std::size_t>(i)]
            .q_close[static_cast<std::size_t>(j)] = seq[j].as<double>();
        }
      }
    }
  }
}

}  // namespace

void ApplyDemoSharedConfig(const YAML::Node & node, DemoSharedConfig & cfg)
{
  if (!node) { return; }

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
  }

  if (node["force_pi_grasp"]) {
    ApplyForcePiBlock(node["force_pi_grasp"], cfg);
  }
}

void LoadDemoSharedYamlFile(DemoSharedConfig & cfg)
{
  std::string yaml_path;
  try {
    yaml_path = ament_index_cpp::get_package_share_directory("ur5e_bringup")
              + kSharedYamlRelPath;
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("ur5e_bringup"),
      "demo_shared.yaml: package share dir lookup failed (%s); skipping shared defaults",
      e.what());
    return;
  }

  try {
    const YAML::Node file_node = YAML::LoadFile(yaml_path);
    ApplyDemoSharedConfig(file_node[kSharedYamlRootKey], cfg);
  } catch (const YAML::Exception & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("ur5e_bringup"),
      "demo_shared.yaml: load failed at %s (%s); using built-in defaults",
      yaml_path.c_str(), e.what());
  }
}

void BuildGraspController(
    const DemoSharedConfig & cfg,
    double control_rate_hz,
    std::unique_ptr<rtc::grasp::GraspController> & grasp_controller)
{
  if (cfg.grasp_controller_type != "force_pi" || !cfg.has_force_pi_block) {
    grasp_controller.reset();
    return;
  }

  rtc::grasp::GraspParams gp = cfg.force_pi_params;
  gp.control_rate_hz = control_rate_hz;

  grasp_controller = std::make_unique<rtc::grasp::GraspController>();
  grasp_controller->Init(cfg.force_pi_fingers, gp);
}

}  // namespace ur5e_bringup
