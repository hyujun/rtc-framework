// ── closed_chain_model: Extended-URDF 통합 loader (§4d) ──────────────────────
#pragma once

#include "rtc_urdf_bridge/closure_yaml_loader.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/constraints.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <string>
#include <string_view>
#include <vector>

namespace rtc_urdf_bridge {

/// @brief Extended-URDF loader 결과 값 타입.
/// actuated_joint_ids 로 actuator selection matrix S 를 구성한다 (S: n_a × nv, 각 row 는
/// 대응 actuated joint 의 velocity 인덱스에 1). torque 관계: tau = Sᵀ · tau_a.
/// q_ref 는 projection 을 통과한 loop-consistent configuration.
struct ClosedChainModel {
  pinocchio::Model model;
  std::vector<pinocchio::RigidConstraintModel> constraints;
  std::vector<pinocchio::JointIndex>
      actuated_joint_ids;       // actuation metadata (RigidConstraint 와 분리)
  Eigen::VectorXd q_ref;        // loop-consistent (projection 통과)
  bool q_ref_converged{false};  // projection 수렴 여부
  // q_ref 가 rank 결손(특이 조립형상) 인지. true 면 constraintDynamics 의 KKT 가 특이해져
  // NaN 을 낼 수 있으므로 q_ref 를 operating configuration 으로 바로 쓰면 안 된다.
  // (converged=true 여도 대칭 조립형상은 특이할 수 있다 — neutral 근방 4-bar 등.)
  bool q_ref_singular{false};
};

/// @brief 순수 spanning-tree URDF + sidecar closure YAML → ClosedChainModel.
///
/// 파이프라인 (모두 로드 타임, 예외 허용):
///   1. pinocchio::urdf::buildModel(urdf) — 표준 tree 빌드
///   2. LoadClosureYaml(closure_yaml) — sidecar 파싱
///   3. BuildRigidConstraints — RigidConstraintModel 생성 (§4b placement)
///   4. actuated_joints → JointIndex 해석
///   5. q_ref = ProjectToConstraint(neutral(model)) — loop-consistent 사영
///   6. q_ref 특이성 검사 (q_ref_singular 세팅 + 경고)
///
/// @param root_joint_floating true 면 FreeFlyer root (기본 fixed base).
/// @throws std::runtime_error URDF/YAML 로드 실패, frame/joint 미존재 시.
/// @note RT 경로에서 호출 금지.
[[nodiscard]] ClosedChainModel BuildClosedChainModelFromExtendedUrdf(
    std::string_view urdf_path, std::string_view closure_yaml_path,
    bool root_joint_floating = false);

}  // namespace rtc_urdf_bridge
