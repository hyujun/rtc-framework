// ── 관성 파라미터 물리 실현가능성 검증 (V5 / V6 로드 게이트) ──────────────────
//
// URDF 가 선언한 관성이 **강체를 기술할 수 있는 값인가**를 모델 로드 시점에
// 판정한다. 판정 대상은 Pinocchio 가 URDF 를 읽어 조립한 `Model::inertias[]`
// — 즉 fixed joint 를 흡수한 **composite body** 이고, 이것이 M(q) 가 실제로
// 조립되는 값이다. 게이트의 의미는 정확히 "이 모델로 동역학을 돌려도 되는가"다.
//
// **오프라인 저작 게이트(`rtc_tools` 의 `compare_mjcf_urdf`)와 축이 다르다.**
// 그쪽 `_check_inertia_plausibility` 는 선언값과 collision 기하 추정값의 *비율*
// 을 보는 informational WARN 이라 기하가 필요하고 (mesh-only·zero-mass 는 조용히
// 면제), "어느 쪽이 진실인지 알 수 없다"고 스스로 못박는다 — **외부 정합**이다.
// 여기는 기하 없이 텐서 단독으로 finite·PSD·삼각부등식을 보는 **내부 실현
// 가능성** 판정이며, 위반이면 어떤 해석으로도 강체가 아니다. 둘은 겹치지
// 않으므로 한쪽이 다른 쪽을 대체하지 못한다 (#316 D-2).
//
// 주의: init 경로 전용이다. RT tick 에서 호출하지 말 것 (할당·고유값 분해).
#ifndef RTC_URDF_BRIDGE__INERTIAL_VALIDATION_HPP_
#define RTC_URDF_BRIDGE__INERTIAL_VALIDATION_HPP_

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace rtc_urdf_bridge {

// ── 상대 허용오차 ────────────────────────────────────────────────────────────
//
// **scale-aware 는 취향이 아니라 필수다.** 저장소 실모델 body 의 최대 주모멘트는
// 1.03e-7 ~ 7.07e-1 로 6.8 decade 에 걸쳐 있어, 작은 쪽에 맞춘 절대 tol 은 큰 쪽
// 에서 7e6 배 과하게 조여진다. 따라서 모든 판정은 주모멘트 크기(scale)로 정규화한다.
//
// 값 근거 (2026-08-17 실측): 정당한 최소 삼각 여유는 +5.53e-5 (`panda_link2`),
// 실제 위반은 -3.33e-2 (`schunk_svh_hand_*` 의 `*_hand_p`) — 약 600 배 간격
// 사이에 임계가 여유롭게 들어간다.
//
// **이 문단이 임계 근거의 SSoT 다.** README·`agent_docs/testing-debug.md` 는 이
// 숫자를 복제하지 말고 여기를 가리킨다 (자산이 바뀌면 재측정 지점이 하나여야 한다).
inline constexpr double kInertialRelTol = 1e-9;

// ── 결함 분류 ────────────────────────────────────────────────────────────────
enum class InertialDefect : std::uint8_t {
  // V6 — 물리적으로 불가능 (로드 실패)
  kNonFinite,                // mass · CoM 기준 관성 · CoM 위치(lever) 중 하나라도
                             // NaN/Inf, 혹은 고유값 분해 실패 (= 판정 불가.
                             // 판정 불가는 통과가 아니라 거부다)
  kNegativeMass,             // mass < 0
  kNotPositiveSemidefinite,  // 관성 텐서에 음의 고유값
  kTriangleInequality,       // I1 + I2 < I3 — 어떤 질량분포로도 실현 불가
  // mass = 0 인데 관성 텐서 ≠ 0. 질량 0 강체는 회전관성도 0 이어야 하므로 어떤
  // 해석으로도 강체가 아니다 — pinocchio 자신의 유효성 기준과 같은 축이다
  // ((m>0 && I valid) || (m==0 && I 전부 0)).
  kMasslessWithInertia,
  // V5 — 물리적으로 불완전 (경고)
  kMasslessMovableBody,  // movable body 의 composite mass 와 관성이 **함께** 0
                         // → 강체로서 모순은 없으나 M(q) 가 그 DoF 에서 특이
};

/// 결함 1건. `principal_moments` 는 **오름차순**, 관성은 CoM 기준이다.
struct InertialViolation {
  std::string joint_name;  ///< 결함 body 를 다는 관절 (Model::names)
  std::string body_name;   ///< 그 관절에 직결된 link (BODY frame)
  InertialDefect defect{InertialDefect::kNonFinite};
  double mass{0.0};
  std::array<double, 3> principal_moments{{0.0, 0.0, 0.0}};
  /// scale = max(|I1|,|I3|) 로 정규화된 여유 (kTriangleInequality → (I1+I2-I3)/scale,
  /// kNotPositiveSemidefinite → I1/scale). 음수면 위반이고 그 크기가 상대 위반량이나,
  /// **-1 에서 포화한다** — 음의 고유값이 절댓값 최대면 I1/scale 이 정확히 -1 이라
  /// (-5, 0.1, 1) 과 (-50, 0.1, 1) 이 같은 값을 낸다. 위반의 절대 크기가 필요하면
  /// principal_moments 를 보라. 크기 판정이 아닌 결함(kNonFinite 등)에서는 0.
  double margin{0.0};
};

/// 판정 결과. 두 레인은 **심각도가 아니라 종류가 다르다** — `fatal` 은 그 값이
/// 강체가 아니라는 뜻이고, `degenerate` 는 강체이긴 하나 동역학을 지탱하지
/// 못한다는 뜻이다. 순서는 Pinocchio 관절 인덱스 오름차순 = 결정적.
struct InertialValidationReport {
  std::vector<InertialViolation> fatal;       ///< V6 — 로드 실패시킬 것
  std::vector<InertialViolation> degenerate;  ///< V5 — 경고 + 술어 노출
};

/// 모든 movable body 의 composite 관성을 판정한다 (universe 관절 제외).
/// @param model  Pinocchio 모델 (buildModel 직후)
/// @param rel_tol 주모멘트 크기로 정규화된 허용오차. **등호는 허용된다** —
///   얇은 판·막대는 I1+I2=I3 이 정확히 성립하므로 `>` 로 판정하면 정당한
///   강체를 거부한다.
[[nodiscard]] InertialValidationReport ValidateInertias(const pinocchio::Model& model,
                                                        double rel_tol = kInertialRelTol);

/// 판정 + **처분**을 한 번에 수행한다 — V5 는 WARN 로그, V6 는 ERROR 로그 +
/// `std::runtime_error`. URDF → `pinocchio::Model` 로드 진입점은 **모두** 이것을
/// 부른다 (`PinocchioModelBuilder` · `BuildClosedChainModelFromExtendedUrdf`).
/// 처분을 진입점마다 손으로 적으면 문이 늘어날 때 조용히 갈라지므로 (실제로
/// 폐쇄 체인 진입점이 게이트 없이 열려 있었다 — #316 D-5) 여기 한 곳에 둔다.
/// @param context 예외·로그에 찍을 호출 진입점 이름.
/// @throws std::runtime_error V6 위반이 하나라도 있을 때.
/// @note init 경로 전용.
[[nodiscard]] InertialValidationReport EnforceInertialGate(const pinocchio::Model& model,
                                                           std::string_view context);

/// 결함 종류의 사람이 읽는 이름 (`V6:triangle-inequality` 등).
[[nodiscard]] const char* ToString(InertialDefect defect) noexcept;

/// 위반 목록을 결정적 순서의 다행 문자열로. 로그·예외 메시지 공용.
[[nodiscard]] std::string DescribeInertialViolations(
    const std::vector<InertialViolation>& violations);

}  // namespace rtc_urdf_bridge

#endif  // RTC_URDF_BRIDGE__INERTIAL_VALIDATION_HPP_
