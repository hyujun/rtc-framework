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
  bool q_ref_converged{false};  // projection strict 수렴 (‖φ‖ < strict tolerance) 여부
  // q_ref 를 소비해도 되는가 (‖φ‖ ≤ acceptance tolerance, 유한). converged=true 면 항상
  // true. converged=false && acceptable=true 는 URDF 좌표 불일치 등의 residual floor —
  // q_ref 는 사용 가능하다 (#250). 둘 다 false 면 q_ref 를 정상 결과로 소비 금지.
  bool q_ref_acceptable{false};
  // q_ref 가 rank 결손(특이 조립형상) 인지. true 면 constraintDynamics 의 KKT 가 특이해져
  // NaN 을 낼 수 있으므로 q_ref 를 operating configuration 으로 바로 쓰면 안 된다.
  // (converged=true 여도 대칭 조립형상은 특이할 수 있다 — neutral 근방 4-bar 등.)
  bool q_ref_singular{false};
};

/// @brief ClosedChainModel 에서 소유 model 을 뺀 결과 (이미 빌드된 model 위에서 계산).
/// 이미 존재하는 pinocchio::Model (예: xacro 전처리를 거친 PinocchioModelBuilder 의
/// full_model_) 에 closure 파이프라인을 적용할 때 사용한다.
struct ClosedChainData {
  std::vector<pinocchio::RigidConstraintModel> constraints;
  std::vector<pinocchio::JointIndex> actuated_joint_ids;
  Eigen::VectorXd q_ref;        // loop-consistent (projection 통과)
  bool q_ref_converged{false};  // projection strict 수렴 여부
  bool q_ref_acceptable{false};  // q_ref 소비 가능 (residual floor 포함 — ClosedChainModel 참조)
  bool q_ref_singular{false};  // rank 결손 조립형상 (KKT 특이 → NaN 위험)
};

/// @brief 이미 빌드된 model + ClosureSpec → closure 파이프라인 (§4b 이후 단계).
///
/// 파이프라인 (모두 로드 타임, 예외 허용):
///   1. BuildRigidConstraints — RigidConstraintModel 생성 (§4b placement)
///   2. actuated_joints → JointIndex 해석
///   3. q_ref = ProjectToConstraint(neutral(model)) — loop-consistent 사영
///   4. q_ref 특이성 검사 (q_ref_singular 세팅 + 경고)
///
/// URDF/xacro 파싱과 model 빌드는 호출자 책임 (이 함수는 model 을 소비만 한다).
/// @throws std::runtime_error frame/joint 미존재 시.
/// @note RT 경로에서 호출 금지.
[[nodiscard]] ClosedChainData BuildClosedChainData(const pinocchio::Model& model,
                                                   const ClosureSpec& spec);

/// @brief 순수 spanning-tree URDF + sidecar closure YAML → ClosedChainModel.
///
/// 파이프라인 (모두 로드 타임, 예외 허용):
///   1. pinocchio::urdf::buildModel(urdf) — 표준 tree 빌드
///   2. LoadClosureYaml(closure_yaml) — sidecar 파싱
///   3. BuildClosedChainData — constraints / actuated / q_ref / 특이성 (위 단계 3-6)
///
/// @param root_joint_floating true 면 FreeFlyer root (기본 fixed base).
/// @throws std::runtime_error URDF/YAML 로드 실패, frame/joint 미존재 시.
/// @note RT 경로에서 호출 금지.
[[nodiscard]] ClosedChainModel BuildClosedChainModelFromExtendedUrdf(
    std::string_view urdf_path, std::string_view closure_yaml_path,
    bool root_joint_floating = false);

}  // namespace rtc_urdf_bridge
