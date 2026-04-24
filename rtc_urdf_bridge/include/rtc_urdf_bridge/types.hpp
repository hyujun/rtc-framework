// ── rtc_urdf_bridge: 범용 URDF→Pinocchio 모델 라이브러리 ──────────────
#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace rtc_urdf_bridge {

// ── URDF 관절 타입 분류 (topology 축) ────────────────────────────────────────
enum class UrdfJointType : std::uint8_t {
  kFixed,
  kRevolute,
  kContinuous,
  kPrismatic,
  kFloating,
  kPlanar,
};

// ── 관절 역할 (의미 축) ──────────────────────────────────────────────────────
// topology(UrdfJointType)와 직교. 컨트롤러/모델 빌더가 참조하는 "구동 가능성"
// 축.
enum class JointRole : std::uint8_t {
  kFixed,  // URDF type == "fixed"
  kActive, // 구동 대상 — 기본값(명시적 passive 분류 아니면 전부 여기로)
  kPassive, // 수동 — kinematic 구속 또는 under-actuated
};

// ── 패시브 관절 subsubtype ───────────────────────────────────────────────────
// role == kPassive 일 때 추가 분류. 중복 후보(mimic∧closed-chain)는 우선순위
// kMimic > kClosedChain > kFree 로 단일 선택하고 warning.
enum class PassiveSubtype : std::uint8_t {
  kNone,        // role != kPassive 일 때
  kMimic,       // <mimic joint="..."> 태그 존재
  kClosedChain, // 폐쇄 체인 루프 경로 위의 관절
  kFree, // 명시적 passive이지만 mimic/closed-chain 어디에도 속하지 않음
};

[[nodiscard]] inline const char *JointRoleToString(JointRole role) noexcept {
  switch (role) {
  case JointRole::kFixed:
    return "fixed";
  case JointRole::kActive:
    return "active";
  case JointRole::kPassive:
    return "passive";
  }
  return "unknown";
}

[[nodiscard]] inline const char *
PassiveSubtypeToString(PassiveSubtype subtype) noexcept {
  switch (subtype) {
  case PassiveSubtype::kNone:
    return "none";
  case PassiveSubtype::kMimic:
    return "mimic";
  case PassiveSubtype::kClosedChain:
    return "closed_chain";
  case PassiveSubtype::kFree:
    return "free";
  }
  return "unknown";
}

/// 관절 타입 문자열 변환 (디버깅/로깅용)
[[nodiscard]] inline const char *JointTypeToString(UrdfJointType t) noexcept {
  switch (t) {
  case UrdfJointType::kFixed:
    return "fixed";
  case UrdfJointType::kRevolute:
    return "revolute";
  case UrdfJointType::kContinuous:
    return "continuous";
  case UrdfJointType::kPrismatic:
    return "prismatic";
  case UrdfJointType::kFloating:
    return "floating";
  case UrdfJointType::kPlanar:
    return "planar";
  }
  return "unknown";
}

/// URDF 문자열 → UrdfJointType 변환
[[nodiscard]] inline UrdfJointType
StringToJointType(const std::string &s) noexcept {
  if (s == "revolute")
    return UrdfJointType::kRevolute;
  if (s == "continuous")
    return UrdfJointType::kContinuous;
  if (s == "prismatic")
    return UrdfJointType::kPrismatic;
  if (s == "fixed")
    return UrdfJointType::kFixed;
  if (s == "floating")
    return UrdfJointType::kFloating;
  if (s == "planar")
    return UrdfJointType::kPlanar;
  return UrdfJointType::kFixed;
}

// ── 패시브 관절 정보 ─────────────────────────────────────────────────────────
struct PassiveJointInfo {
  std::string joint_name;
  UrdfJointType type{UrdfJointType::kRevolute};
  PassiveSubtype subtype{PassiveSubtype::kFree};
  double damping{0.0};
  double friction{0.0};
};

// ── 마이믹 관절 정보 ─────────────────────────────────────────────────────────
struct MimicJointInfo {
  std::string joint_name;     // 이 관절 (mimic 수행하는 측)
  std::string mimicked_joint; // 추종 대상 관절
  double multiplier{1.0};
  double offset{0.0};
};

// ── 인접 그래프 노드 (kinematic tree) ────────────────────────────────────────
struct LinkNode {
  std::string link_name;
  std::string parent_joint_name;
  UrdfJointType parent_joint_type{UrdfJointType::kFixed};
  int parent_index{-1};
  std::vector<int> child_indices;
};

// ── 관절 메타데이터 ──────────────────────────────────────────────────────────
struct JointMeta {
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
  // 의미 축 분류 (ClassifyJointRoles 결과로 채워짐)
  JointRole role{JointRole::kFixed};
  PassiveSubtype passive_subtype{PassiveSubtype::kNone};
  // <limit> 태그 존재 AND effort > 0 AND (continuous 아니면) velocity > 0
  bool has_physics{false};
  // URDF 파싱 단계에서 각 신호 존재 여부 (has_physics 판정 근거 및 warning용)
  bool has_limit_tag{false};
};

// ── 서브모델 정의 (단일 체인: root → tip) ────────────────────────────────────
struct SubModelDefinition {
  std::string name;
  std::string root_link;
  std::string tip_link;
  std::vector<std::string> joint_names; // 경로 상 actuated 관절
  std::vector<std::string> all_joint_names; // 경로 상 모든 관절 (fixed 포함)
  std::vector<std::string> link_names; // 경로 상 모든 링크
};

// ── 트리 모델 정의 (하나의 root, 여러 tip) ──────────────────────────────────
struct TreeModelDefinition {
  std::string name;
  std::string root_link;
  std::vector<std::string> tip_links;
  std::vector<std::string> joint_names;     // 트리 내 모든 actuated 관절
  std::vector<std::string> all_joint_names; // 트리 내 모든 관절
  std::vector<std::string> link_names;      // 트리 내 모든 링크
  std::vector<std::string> branching_points; // 분기점 링크 이름
};

// ── 폐쇄 체인 정보 ──────────────────────────────────────────────────────────
struct ClosedChainInfo {
  std::string name;
  std::string link_a;
  std::string link_b;
  Eigen::Vector3d offset_a_xyz{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offset_a_rpy{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offset_b_xyz{Eigen::Vector3d::Zero()};
  Eigen::Vector3d offset_b_rpy{Eigen::Vector3d::Zero()};
  bool is_6d{true}; // true → CONTACT_6D, false → CONTACT_3D
  double baumgarte_kp{0.0};
  double baumgarte_kd{0.0};
};

// ── YAML 모델 설정 ──────────────────────────────────────────────────────────
struct SubModelConfig {
  std::string name;
  std::string root_link;
  std::string tip_link;
};

struct TreeModelConfig {
  std::string name;
  std::string root_link;
  std::vector<std::string> tip_links;
};

struct ModelConfig {
  // URDF 소스
  std::string urdf_path;       // 파일 경로
  std::string urdf_xml_string; // 또는 XML 문자열 직접 제공

  // xacro 인자 (urdf_path가 .xacro 파일일 때 key:=value로 전달)
  std::unordered_map<std::string, std::string> xacro_args;

  // root joint 타입 ("fixed" | "floating")
  std::string root_joint_type{"fixed"};

  // 서브모델 / 트리모델 정의
  std::vector<SubModelConfig> sub_models;
  std::vector<TreeModelConfig> tree_models;

  // 폐쇄 체인 정의
  std::vector<ClosedChainInfo> closed_chains;

  // 패시브 관절 hint/override 목록
  // - yaml_passive_override == false (기본): 자동 분류 보조 hint로만 사용
  //   subtype은 closed-chain 참여 여부로 자동 판정 (closed_chain/free).
  // - yaml_passive_override == true: 이 목록이 자동 분류를 덮어쓴다.
  std::vector<std::string> passive_joints;

  // YAML passive_joints override 모드 (Q4 옵션 A/B 스위치)
  // false = 옵션 B (hint), true = 옵션 A (override). 기본은 hint.
  bool yaml_passive_override{false};

  // 잠금 관절의 기준 설정값 (joint_name → radian)
  std::unordered_map<std::string, double> lock_reference_config;
};

} // namespace rtc_urdf_bridge
