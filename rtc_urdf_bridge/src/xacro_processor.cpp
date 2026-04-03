// ── Xacro 전처리 유틸리티 구현 ───────────────────────────────────────────────
#include "rtc_urdf_bridge/xacro_processor.hpp"

#include <array>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace rtc_urdf_bridge
{

bool IsXacroFile(std::string_view file_path) noexcept
{
  return file_path.ends_with(".xacro");
}

std::string ProcessXacro(
  const std::string & file_path,
  const std::unordered_map<std::string, std::string> & xacro_args)
{
  // 파일 존재 확인
  if (!std::filesystem::exists(file_path)) {
    throw std::runtime_error(
      "ProcessXacro: 파일이 존재하지 않습니다: " + file_path);
  }

  // 명령어 구성: xacro "<file_path>" key:=value ... 2>&1
  std::string cmd = "xacro \"" + file_path + "\"";
  for (const auto & [key, value] : xacro_args) {
    cmd += " " + key + ":=" + value;
  }
  cmd += " 2>&1";

  // popen으로 실행
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-attributes"
  std::unique_ptr<FILE, decltype(& pclose)> pipe(
    popen(cmd.c_str(), "r"), pclose);  // NOLINT(cert-env33-c)
#pragma GCC diagnostic pop

  if (!pipe) {
    throw std::runtime_error(
      "ProcessXacro: popen 실행 실패 — 명령: " + cmd);
  }

  // stdout 읽기
  std::string output;
  std::array<char, 4096> buffer{};
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
    output += buffer.data();
  }

  // 종료 코드 확인 (pclose는 unique_ptr 소멸자에서 호출되므로 직접 호출)
  FILE * raw_pipe = pipe.release();
  int status = pclose(raw_pipe);
  int exit_code = WEXITSTATUS(status);

  if (exit_code != 0) {
    // exit code 127: 명령 미발견
    if (exit_code == 127) {
      throw std::runtime_error(
        "ProcessXacro: 'xacro' 명령을 찾을 수 없습니다. "
        "ros-${ROS_DISTRO}-xacro 패키지가 설치되어 있는지 확인하세요.");
    }
    throw std::runtime_error(
      "ProcessXacro: xacro 실행 실패 (exit code " +
      std::to_string(exit_code) + ")\n" + output);
  }

  if (output.empty()) {
    throw std::runtime_error(
      "ProcessXacro: xacro 출력이 비어 있습니다: " + file_path);
  }

  return output;
}

}  // namespace rtc_urdf_bridge
