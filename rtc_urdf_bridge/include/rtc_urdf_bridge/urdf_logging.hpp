// ── rtc_urdf_bridge 계층적 sub-logger 정의 ─────────────────────────────────────
//
// 모든 init-time 진단은 `urdf.*` 네임스페이스의 sub-logger를 통해 발사한다.
// 런타임에 `ros2 service call /<node>/set_logger_levels` 로 개별 카테고리만
// DEBUG로 켤 수 있다. 하나의 콘솔 출력 포맷에 logger 이름이 보이도록
// `RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"`로
// 환경 변수를 설정할 것을 권장한다.
//
// 주의: RtModelHandle의 RT-safe(noexcept) 메서드에서는 절대 사용하지 말 것.
//       이 헤더의 모든 logger는 init/teardown 경로 전용이다.

#ifndef RTC_URDF_BRIDGE__URDF_LOGGING_HPP_
#define RTC_URDF_BRIDGE__URDF_LOGGING_HPP_

#include <string>
#include <string_view>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace rtc::urdf::logging
{

// urdf.analyzer  — UrdfAnalyzer (URDF/xacro 파싱, 그래프 구축, 관절 분류)
inline rclcpp::Logger AnalyzerLogger() {return rclcpp::get_logger("urdf.analyzer");}

// urdf.builder   — PinocchioModelBuilder (full/sub/tree 모델 빌드, YAML 로드)
inline rclcpp::Logger BuilderLogger() {return rclcpp::get_logger("urdf.builder");}

// urdf.chain     — KinematicChainExtractor (체인/트리 추출, 잠금 관절 계산)
inline rclcpp::Logger ChainLogger() {return rclcpp::get_logger("urdf.chain");}

// urdf.xacro     — ProcessXacro (popen 기반 xacro 전처리)
inline rclcpp::Logger XacroLogger() {return rclcpp::get_logger("urdf.xacro");}

// urdf.handle    — RtModelHandle 생성/해제 단계 (RT 경로 외)
inline rclcpp::Logger HandleLogger() {return rclcpp::get_logger("urdf.handle");}

}  // namespace rtc::urdf::logging

#endif  // RTC_URDF_BRIDGE__URDF_LOGGING_HPP_
