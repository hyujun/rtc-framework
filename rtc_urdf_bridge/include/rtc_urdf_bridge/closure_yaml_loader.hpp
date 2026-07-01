// ── closure_yaml_loader: Extended-URDF sidecar (<name>.closure.yaml) 파서 ─────
#pragma once

#include "rtc_urdf_bridge/types.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace rtc_urdf_bridge {

/// @brief ball / U-joint 치환 정보 (joint_substitutions). 이번 구현 범위 밖이나
/// 스키마 완전성을 위해 raw 로 보존 (추후 소비).
struct JointSubstitution {
  std::string joint;        // 치환 대상 joint 이름
  std::string replacement;  // 치환 타입 (예: "spherical")
};

/// @brief `<name>.closure.yaml` sidecar 파싱 결과 (Extended-URDF 중간 표현).
/// URDF 는 순수 spanning-tree 로 유지되고, loop closure / actuation / 치환 정보만 여기에.
struct ClosureSpec {
  std::string name;
  std::vector<ClosedChainInfo> closures;
  std::vector<std::string> actuated_joints;  // 나머지 non-fixed 는 passive 로 간주
  std::vector<JointSubstitution> joint_substitutions;
};

/// @brief sidecar YAML 파일 경로 → ClosureSpec.
/// @throws std::runtime_error 파일 없음 / 파싱 실패 / 필수 필드 누락 / endpoint 미지정.
[[nodiscard]] ClosureSpec LoadClosureYaml(std::string_view yaml_path);

/// @brief YAML 텍스트 → ClosureSpec (테스트 및 in-memory 용).
/// @param source_label 에러 메시지에 표시할 출처 라벨.
/// @throws std::runtime_error 파싱 실패 / 필수 필드 누락 / endpoint 미지정.
[[nodiscard]] ClosureSpec ParseClosureYaml(const std::string& yaml_text,
                                           std::string_view source_label = "<string>");

}  // namespace rtc_urdf_bridge
