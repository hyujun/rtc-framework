// ── Xacro 전처리 유틸리티 ────────────────────────────────────────────────────
#pragma once

#include <string>
#include <string_view>
#include <unordered_map>

namespace urdf_pinocchio_bridge
{

/// 파일 경로가 .xacro 확장자인지 확인
[[nodiscard]] bool IsXacroFile(std::string_view file_path) noexcept;

/// xacro CLI를 호출하여 .xacro 파일을 URDF XML 문자열로 변환
///
/// @param file_path  .xacro 파일 경로 (절대/상대)
/// @param xacro_args xacro에 전달할 key:=value 인자 (선택)
/// @return 처리된 URDF XML 문자열
/// @throws std::runtime_error xacro 명령 실패 또는 미설치 시
[[nodiscard]] std::string ProcessXacro(
  const std::string & file_path,
  const std::unordered_map<std::string, std::string> & xacro_args = {});

}  // namespace urdf_pinocchio_bridge
