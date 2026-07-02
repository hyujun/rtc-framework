// ── closure_yaml_loader 구현 ─────────────────────────────────────────────────
#include "rtc_urdf_bridge/closure_yaml_loader.hpp"

#include "rtc_urdf_bridge/urdf_logging.hpp"
#include "rtc_urdf_bridge/xacro_processor.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace rtc_urdf_bridge {

namespace {
auto logger() {
  return ::rtc::urdf::logging::BuilderLogger();
}

// { xyz: [...], rpy: [...] } → 두 Vector3d. 없는 축은 0 유지.
void ParseOrigin(const YAML::Node& node, Eigen::Vector3d& xyz, Eigen::Vector3d& rpy) {
  if (!node) {
    return;
  }
  if (node["xyz"]) {
    const auto v = node["xyz"];
    xyz << v[0].as<double>(), v[1].as<double>(), v[2].as<double>();
  }
  if (node["rpy"]) {
    const auto v = node["rpy"];
    rpy << v[0].as<double>(), v[1].as<double>(), v[2].as<double>();
  }
}

// "contact_3d" | "3D" | "contact_6d" | "6D" → is_6d. type 은 §2 상 필수 필드.
bool ParseContactType(const std::string& s, const std::string& closure_name) {
  if (s == "contact_3d" || s == "3D" || s == "3d") {
    return false;
  }
  if (s == "contact_6d" || s == "6D" || s == "6d") {
    return true;
  }
  throw std::runtime_error("closure_yaml_loader: closure '" + closure_name +
                           "' 의 type 이 유효하지 않습니다: '" + s + "' (contact_3d | contact_6d)");
}

LoopReferenceFrame ParseReferenceFrame(const std::string& s, const std::string& closure_name) {
  if (s == "LOCAL") {
    return LoopReferenceFrame::kLocal;
  }
  if (s == "LOCAL_WORLD_ALIGNED") {
    return LoopReferenceFrame::kLocalWorldAligned;
  }
  if (s == "WORLD") {
    return LoopReferenceFrame::kWorld;
  }
  throw std::runtime_error("closure_yaml_loader: closure '" + closure_name +
                           "' 의 reference_frame 이 유효하지 않습니다: '" + s + "'");
}

ClosedChainInfo ParseClosure(const YAML::Node& node) {
  ClosedChainInfo cc;

  if (!node["name"]) {
    throw std::runtime_error("closure_yaml_loader: closure 항목에 name 필드가 없습니다");
  }
  cc.name = node["name"].as<std::string>();

  // type 필수 (§2)
  if (!node["type"]) {
    throw std::runtime_error("closure_yaml_loader: closure '" + cc.name +
                             "' 에 필수 필드 type 이 없습니다 (contact_3d | contact_6d)");
  }
  cc.is_6d = ParseContactType(node["type"].as<std::string>(), cc.name);

  // endpoint: frame 참조(1순위) 또는 link+origin(fallback)
  if (node["frame_1"] && node["frame_2"]) {
    cc.frame_1 = node["frame_1"].as<std::string>();
    cc.frame_2 = node["frame_2"].as<std::string>();
  } else if (node["parent_link"] && node["child_link"]) {
    cc.link_a = node["parent_link"].as<std::string>();
    cc.link_b = node["child_link"].as<std::string>();
    ParseOrigin(node["parent_origin"], cc.offset_a_xyz, cc.offset_a_rpy);
    ParseOrigin(node["child_origin"], cc.offset_b_xyz, cc.offset_b_rpy);
  } else {
    throw std::runtime_error(
        "closure_yaml_loader: closure '" + cc.name +
        "' 에 endpoint 미지정 (frame_1+frame_2 또는 parent_link+child_link 필요)");
  }

  if (node["reference_frame"]) {
    cc.reference_frame = ParseReferenceFrame(node["reference_frame"].as<std::string>(), cc.name);
  }

  // baumgarte: { Kp, Kd } — 미지정 시 파서가 안전 기본값을 채운다 (§2, §4c).
  if (node["baumgarte"]) {
    const auto b = node["baumgarte"];
    if (b["Kp"]) {
      cc.baumgarte_kp = b["Kp"].as<double>();
    }
    if (b["Kd"]) {
      cc.baumgarte_kd = b["Kd"].as<double>();
    }
    // Kp 지정 & Kd 미지정 → 임계감쇠 근처 (Kd ≈ 2√Kp) 를 로드 타임에 확정.
    if (cc.baumgarte_kp > 0.0 && cc.baumgarte_kd <= 0.0) {
      cc.baumgarte_kd = 2.0 * std::sqrt(cc.baumgarte_kp);
    }
  }

  return cc;
}
}  // namespace

ClosureSpec ParseClosureYaml(const std::string& yaml_text, std::string_view source_label) {
  ClosureSpec spec;
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("closure_yaml_loader: YAML 파싱 실패 (" + std::string(source_label) +
                             "): " + e.what());
  }

  if (root["name"]) {
    spec.name = root["name"].as<std::string>();
  }

  if (root["closures"]) {
    for (const auto& node : root["closures"]) {
      spec.closures.push_back(ParseClosure(node));
    }
  }

  if (root["actuation"] && root["actuation"]["actuated_joints"]) {
    for (const auto& j : root["actuation"]["actuated_joints"]) {
      spec.actuated_joints.push_back(j.as<std::string>());
    }
  }

  if (root["joint_substitutions"]) {
    for (const auto& node : root["joint_substitutions"]) {
      JointSubstitution sub;
      if (node["joint"]) {
        sub.joint = node["joint"].as<std::string>();
      }
      if (node["replacement"]) {
        sub.replacement = node["replacement"].as<std::string>();
      }
      spec.joint_substitutions.push_back(std::move(sub));
    }
  }

  RCLCPP_DEBUG(logger(), "closure sidecar '%s' 로드: closures=%zu, actuated=%zu", spec.name.c_str(),
               spec.closures.size(), spec.actuated_joints.size());
  return spec;
}

std::string DeriveClosureSidecarPath(std::string_view urdf_path) {
  std::filesystem::path p{std::string(urdf_path)};
  if (IsXacroFile(p.string())) {
    p.replace_extension();  // .xacro 제거 → foo.urdf
  }
  p.replace_extension();  // .urdf (또는 마지막 확장자) 제거 → foo
  return p.string() + ".closure.yaml";
}

ClosureSpec LoadClosureYaml(std::string_view yaml_path) {
  std::ifstream ifs{std::string(yaml_path)};
  if (!ifs) {
    throw std::runtime_error("closure_yaml_loader: sidecar 파일을 열 수 없습니다: " +
                             std::string(yaml_path));
  }
  std::stringstream ss;
  ss << ifs.rdbuf();
  return ParseClosureYaml(ss.str(), yaml_path);
}

}  // namespace rtc_urdf_bridge
