// ── loop_projection: q 를 loop-consistent configuration 으로 사영 ─────────────
#pragma once

#include "rtc_urdf_bridge/loop_verification.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/constraints.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <vector>

namespace rtc_urdf_bridge {

/// 기본 actuated seed 증분 상한 (rad 또는 m — 관절 종류에 따른 tangent 단위).
///
/// 점 구속(CONTACT_3D) loop 은 조립 분기(assembly mode)가 둘 이상이고 **모든 분기가 φ=0 을
/// 정확히 만족**하므로, residual 만으로는 물리적으로 옳은 분기를 판정할 수 없다. 한 번의
/// Newton 사영에서 actuated seed 가 크게 움직이면 Gauss-Newton 스텝이 반대편 분기로
/// 착지하고, warm-start 구조상 그 분기에 영구히 고정된다 (#248). 분기는 residual 이 아니라
/// **homotopy 경로**가 결정하므로 seed 증분을 이 값으로 제한한다.
///
/// 0.05 rad 는 proto_1b(5-loop hand) 실측 이탈 임계 0.087~0.12 rad 대비 약 2배 마진이다.
inline constexpr double kDefaultActuatedIncrement = 0.05;

/// continuation sub-step 수 상한 (비정상적으로 큰 점프에서 무한정 반복하지 않도록).
///
/// ⚠ **cap 에 도달하면 sub-step 증분이 `max_actuated_increment` 를 초과한다** — 즉 분기 유지
/// 보장이 조용히 사라진다. 256 은 기본 증분 0.05 rad 기준 Δ≈12.8 rad 까지 보장을 유지하므로
/// ±2π 범위 관절의 cold start(최악 ≈4π)도 덮는다. 그보다 큰 점프는 측정 이상에 가까우며,
/// 그 경우 결과는 여전히 φ=0 을 만족하되 **어느 분기인지는 보장되지 않는다**.
inline constexpr int kMaxContinuationSubsteps = 256;

/// 사영 1회에서 passive q 가 seed 대비 움직일 수 있는 상한 (분기 이탈 2차 가드).
///
/// seed 증분을 @ref kDefaultActuatedIncrement 로 제한해도, near-singular 조립형상에서는 DLS
/// 스텝이 폭주해 passive 가 크게 튈 수 있다. 정상 tick 의 passive 이동량(≈ actuated 증분 규모)
/// 보다 한 자릿수 크고, 분기 이탈(≳π)보다는 확실히 작은 값으로 잡는다.
inline constexpr double kDefaultMaxPassiveDeviation = 0.5;

/// @brief position-level projection 옵션 (damped least-squares Newton).
struct ProjectionOptions {
  double tolerance{1e-10};  // 수렴 판정 ‖φ‖ 임계
  int max_iterations{100};
  double damping{1e-8};  // λ (DLS regularization coefficient; λ² 로 사용)
};

/// @brief projection 결과.
struct ProjectionResult {
  Eigen::VectorXd q;        // 사영된 configuration
  bool converged{false};    // ‖φ‖ < tolerance 도달 여부
  int iterations{0};        // 소요 반복
  double final_error{0.0};  // 최종 ‖φ‖
};

/// @brief φ(q)=0 을 damped least-squares Newton 으로 풀어 q 를 loop-consistent 로 사영.
///   dq = −Jcᵀ (Jc Jcᵀ + λ²I)⁻¹ φ(q),  q ← integrate(model, q, dq) 반복.
/// 수렴 실패 시 converged=false 반환(마지막 q 유지). **RT 밖에서만 호출.**
[[nodiscard]] ProjectionResult ProjectToConstraint(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q_init,
    const ProjectionOptions& opts = {});

/// @brief actuated joint 을 **고정**하고 passive DoF 만 사영해 φ(q)=0 을 만족시킨다.
///   측정된 actuated q (q_init 의 해당 슬롯) 를 그대로 두고, passive 열만 갱신한다.
///     dv_pass = −Jc_passᵀ (Jc_pass Jc_passᵀ + λ²I)⁻¹ φ(q); actuated tangent = 0;
///     q ← integrate(model, q, dv). → integrate(0) 이므로 actuated q 는 정확히 불변.
///   @param actuated_joint_ids 고정할 joint (velocity 열은 idx_vs..+nvs 로 마스킹).
///   @param q_init actuated 슬롯 = 측정값, passive 슬롯 = warm-start seed (직전 프레임 해).
///   내부 loop 구속의 Jc 는 root/base 열이 0 이므로 floating base 도 안전(base 미이동).
///   Extended-URDF 폐쇄 체인 시각화(actuated 스트림 → loop-consistent full q)용.
///   수렴 실패 시 converged=false (마지막 q 유지 — 소비자는 직전 해 hold 권장). **RT 밖에서만.**
[[nodiscard]] ProjectionResult ProjectPassiveToConstraint(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q_init,
    const std::vector<pinocchio::JointIndex>& actuated_joint_ids,
    const ProjectionOptions& opts = {});

/// @brief actuated seed 를 **여러 sub-step 으로 나눠** @ref ProjectPassiveToConstraint 를 연쇄
///   호출한다 (continuation). 큰 actuated 점프에서 조립 분기 이탈을 막는 표준 진입점.
///
///   `‖difference(q_prev, q_target)‖∞` (actuated 열만) 이 @p max_actuated_increment 를 넘으면
///   sub-step 수 `n = ⌈Δ/증분⌉` (상한 @ref kMaxContinuationSubsteps) 로 나누고, 각 sub-step 에서
///   actuated 슬롯만 `interpolate(q_prev, q_target, s/n)` 값으로 갱신한 뒤 직전 sub-step 의
///   passive 해를 warm-start 로 사영한다. 넘지 않으면 단일 호출과 동일하다.
///
///   @param q_prev 직전 loop-consistent 해. **passive warm-start 는 항상 여기서 온다.**
///   @param q_target actuated 슬롯만 유효 (새 측정값). **passive 슬롯은 무시된다.**
///   @param max_actuated_increment sub-step 당 actuated 증분 상한. ≤0 이면 continuation 비활성
///     (단일 사영 — 기존 동작). 기본 @ref kDefaultActuatedIncrement.
///   @return **마지막으로 실행된 sub-step** 의 결과 (`q` 는 전체 loop-consistent configuration).
///     `converged`/`final_error`/`iterations` 도 그 sub-step 기준이다.
///     ⚠ 중간 sub-step 이 미수렴하면 **그 지점에서 중단하고** 그 결과를 돌린다
///     (`converged=false`, `q` 는 도중까지만 진행된 형상). 미수렴 해를 warm-start 로 이어가면
///     homotopy 보장이 깨진 채 진행되고, 마지막 sub-step 만 수렴해도 `converged=true` 가 돼
///     소비자가 그 형상을 커밋하기 때문이다. 소비자는 기존대로 `converged` 로 hold 를 판정하면
///     된다 — 실패 시 actuated 슬롯이 `q_target` 에 도달하지 않았음에 유의.
///   크기가 model.nq 와 다른 입력은 continuation 없이 @ref ProjectPassiveToConstraint 로 위임한다.
///   **RT 밖에서만 호출** (sub-step 수가 입력 의존 → 비결정적. RT 경로는
///   @ref RtClosedChainHandle 의 tick 당 seed clamp 가 같은 역할을 결정적으로 수행한다).
[[nodiscard]] ProjectionResult ProjectPassiveWithContinuation(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q_prev,
    const Eigen::VectorXd& q_target, const std::vector<pinocchio::JointIndex>& actuated_joint_ids,
    const ProjectionOptions& opts = {}, double max_actuated_increment = kDefaultActuatedIncrement);

/// @brief velocity-level projection: v 를 Jc v = 0 을 만족하는 가장 가까운 벡터로 사영.
///   v_proj = v − Jcᵀ (Jc Jcᵀ + λ²I)⁻¹ Jc v. **RT 밖에서만 호출.**
[[nodiscard]] Eigen::VectorXd ProjectVelocity(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q,
    const Eigen::VectorXd& v, double damping = 1e-8);

}  // namespace rtc_urdf_bridge
