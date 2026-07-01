// ── constraint_builder: ClosedChainInfo(DTO) → Pinocchio RigidConstraintModel ─
#pragma once

#include "rtc_urdf_bridge/types.hpp"

// Pinocchio 헤더 (경고 억제 — 기존 model builder 와 동일 패턴)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/constraints.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

#include <vector>

namespace rtc_urdf_bridge {

/// @brief endpoint 해석 결과 — joint frame 기준 placement.
/// X_joint_closure = "constraint frame placement expressed in the parent joint frame".
struct ResolvedEndpoint {
  pinocchio::JointIndex joint_id{0};
  pinocchio::SE3 joint_M_closure{pinocchio::SE3::Identity()};
};

/// @brief 하나의 endpoint(frame 참조 또는 link+origin)를 joint frame 기준 placement 로 해석.
///
/// - frame 참조 경로 (frame_name 비어있지 않음): getFrameId → frames[fid].parentJoint,
///   frames[fid].placement(= X_parentJoint_frame) 를 그대로 사용. **표준·최안전 경로.**
/// - link+origin fallback: joint_M_closure = frames[link].placement * link_M_closure,
///   여기서 link_M_closure = SE3(rpy→R, xyz), rpy 는 URDF extrinsic XYZ (Rz*Ry*Rx).
///
/// @throws std::runtime_error frame/link 가 model 에 없을 때 (명확한 메시지).
/// @note 로드 타임 전용 (예외 허용). RT 경로에서 호출 금지.
[[nodiscard]] ResolvedEndpoint ResolveEndpoint(const pinocchio::Model& model,
                                               const std::string& frame_name,
                                               const std::string& link_name,
                                               const Eigen::Vector3d& xyz,
                                               const Eigen::Vector3d& rpy);

/// @brief 단일 ClosedChainInfo → RigidConstraintModel.
///
/// legacy RigidConstraintModel 경로 (Pinocchio 4.0 에서 deprecated 이나 검증 안정).
/// 추후 PointAnchor/FrameAnchorConstraintModel 로의 이관은 이 함수 하나만 교체 (§7).
///
/// @throws std::runtime_error frame/link 미존재, 또는 reference_frame == WORLD (미지원).
/// @note 로드 타임 전용.
[[nodiscard]] pinocchio::RigidConstraintModel BuildRigidConstraint(const pinocchio::Model& model,
                                                                   const ClosedChainInfo& spec);

/// @brief 여러 ClosedChainInfo → RigidConstraintModel 목록.
/// @throws std::runtime_error 개별 spec 실패 시 (해당 이름 포함).
/// @note 로드 타임 전용.
[[nodiscard]] std::vector<pinocchio::RigidConstraintModel> BuildRigidConstraints(
    const pinocchio::Model& model, const std::vector<ClosedChainInfo>& specs);

}  // namespace rtc_urdf_bridge
