// ── urdf_pinocchio_bridge: 범용 URDF→Pinocchio 모델 라이브러리 ──────────────
#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace urdf_pinocchio_bridge
{

// ── URDF 관절 타입 분류 ──────────────────────────────────────────────────────
enum class UrdfJointType : std::uint8_t
{
  kFixed,
  kRevolute,
  kContinuous,
  kPrismatic,
  kFloating,
  kPlanar,
};

/// 관절 타입 문자열 변환 (디버깅/로깅용)
[[nodiscard]] inline const char * JointTypeToString(UrdfJointType t) noexcept
{
  switch (t) {
    case UrdfJointType::kFixed:      return "fixed";
    case UrdfJointType::kRevolute:   return "revolute";
    case UrdfJointType::kContinuous: return "continuous";
    case UrdfJointType::kPrismatic:  return "prismatic";
    case UrdfJointType::kFloating:   return "floating";
    case UrdfJointType::kPlanar:     return "planar";
  }
  return "unknown";
}

/// URDF 문자열 → UrdfJointType 변환
[[nodiscard]] inline UrdfJointType StringToJointType(const std::string & s) noexcept
{
  if (s == "revolute")   return UrdfJointType::kRevolute;
  if (s == "continuous") return UrdfJointType::kContinuous;
  if (s == "prismatic")  return UrdfJointType::kPrismatic;
  if (s == "fixed")      return UrdfJointType::kFixed;
  if (s == "floating")   return UrdfJointType::kFloating;
  if (s == "planar")     return UrdfJointType::kPlanar;
  return UrdfJointType::kFixed;
}

// ── 패시브 관절 정보 ─────────────────────────────────────────────────────────
struct PassiveJointInfo
{
  std::string joint_name;
  UrdfJointType type{UrdfJointType::kRevolute};
  double damping{0.0};
  double friction{0.0};
};

// ── 마이믹 관절 정보 ─────────────────────────────────────────────────────────
struct MimicJointInfo
{
  std::string joint_name;       // 이 관절 (mimic 수행하는 측)
  std::string mimicked_joint;   // 추종 대상 관절
  double multiplier{1.0};
  double offset{0.0};
};

// ── 인접 그래프 노드 (kinematic tree) ────────────────────────────────────────
struct LinkNode
{
  std::string link_name;
  std::string parent_joint_name;
  UrdfJointType parent_joint_type{UrdfJointType::kFixed};
  int parent_index{-1};
  std::vector<int> child_indices;
};

// ── 관절 메타데이터 ──────────────────────────────────────────────────────────
struct JointMeta
{
  std::string joint_name;
  std::string parent_link;
  std::string child_link;
  UrdfJointType type{UrdfJointType::kFixed};
  Eigen::Vector3d axis{0.0, 0.0, 1.0};
  double lower{0.0};
  double upper{0.0};
  double effort{0.0};
  double velocity{0.0};
  double damping{0.0};
  double friction{0.0};
};

// ── 서브모델 정의 (단일 체인: root → tip) ────────────────────────────────────
struct SubModelDefinition
{
  std::string name;
  std::string root_link;
  std::string tip_link;
  std::vector<std::string> joint_names;       // 경로 상 actuated 관절
  std::vector<std::string> all_joint_names;   // 경로 상 모든 관절 (fixed 포함)
  std::vector<std::string> link_names;        // 경로 상 모든 링크
};

// ── 트리 모델 정의 (하나의 root, 여러 tip) ──────────────────────────────────
struct TreeModelDefinition
{
  std::string name;
  std::string root_link;
  std::vector<std::string> tip_links;
  std::vector<std::string> joint_names;       // 트리 내 모든 actuated 관절
  std::vector<std::string> all_joint_names;   // 트리 내 모든 관절
  std::vector<std::string> link_names;        // 트리 내 모든 링크
  std::vector<std::string> branching_points;  // 분기점 링크 이름
};

// ── 폐쇄 체인 정보 ──────────────────────────────────────────────────────────
struct ClosedChainInfo
{
  std::string name;
  std::string link_a;
  std::string link_b;
  Eigen::Vector3d offset_a_xyz{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offset_a_rpy{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offset_b_xyz{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offset_b_rpy{Eigen::Vector3d::Zero()};
  bool is_6d{true};             // true → CONTACT_6D, false → CONTACT_3D
  double baumgarte_kp{0.0};
  double baumgarte_kd{0.0};
};

// ── YAML 모델 설정 ──────────────────────────────────────────────────────────
struct SubModelConfig
{
  std::string name;
  std::string root_link;
  std::string tip_link;
};

struct TreeModelConfig
{
  std::string name;
  std::string root_link;
  std::vector<std::string> tip_links;
};

struct ModelConfig
{
  // URDF 소스
  std::string urdf_path;          // 파일 경로
  std::string urdf_xml_string;    // 또는 XML 문자열 직접 제공

  // xacro 인자 (urdf_path가 .xacro 파일일 때 key:=value로 전달)
  std::unordered_map<std::string, std::string> xacro_args;

  // root joint 타입 ("fixed" | "floating")
  std::string root_joint_type{"fixed"};

  // 서브모델 / 트리모델 정의
  std::vector<SubModelConfig> sub_models;
  std::vector<TreeModelConfig> tree_models;

  // 폐쇄 체인 정의
  std::vector<ClosedChainInfo> closed_chains;

  // 패시브 관절 override 목록
  std::vector<std::string> passive_joints;

  // 잠금 관절의 기준 설정값 (joint_name → radian)
  std::unordered_map<std::string, double> lock_reference_config;
};

}  // namespace urdf_pinocchio_bridge
