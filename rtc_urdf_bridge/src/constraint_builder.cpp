// ── constraint_builder 구현 ──────────────────────────────────────────────────
#include "rtc_urdf_bridge/constraint_builder.hpp"

#include "rtc_urdf_bridge/urdf_logging.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math/rpy.hpp>
#pragma GCC diagnostic pop

#include <cmath>
#include <stdexcept>
#include <string>

namespace rtc_urdf_bridge {

namespace {
auto logger() {
  return ::rtc::urdf::logging::BuilderLogger();
}

pinocchio::SE3 MakeSe3(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
  // URDF rpy 규약: extrinsic fixed-axis XYZ → R = Rz(yaw)*Ry(pitch)*Rx(roll).
  // pinocchio::rpy::rpyToMatrix(roll, pitch, yaw) 가 정확히 이 순서를 구현.
  return pinocchio::SE3(pinocchio::rpy::rpyToMatrix(rpy[0], rpy[1], rpy[2]), xyz);
}

pinocchio::ReferenceFrame ToPinocchioRF(LoopReferenceFrame rf) {
  switch (rf) {
    case LoopReferenceFrame::kLocal:
      return pinocchio::LOCAL;
    case LoopReferenceFrame::kLocalWorldAligned:
      return pinocchio::LOCAL_WORLD_ALIGNED;
    case LoopReferenceFrame::kWorld:
      // RigidConstraintModel::init() 은 LOCAL | LOCAL_WORLD_ALIGNED 만 허용.
      throw std::runtime_error(
          "constraint_builder: reference_frame=WORLD 는 RigidConstraintModel 에서 미지원 "
          "(LOCAL 또는 LOCAL_WORLD_ALIGNED 사용)");
  }
  return pinocchio::LOCAL;
}
}  // namespace

ResolvedEndpoint ResolveEndpoint(const pinocchio::Model& model, const std::string& frame_name,
                                 const std::string& link_name, const Eigen::Vector3d& xyz,
                                 const Eigen::Vector3d& rpy) {
  ResolvedEndpoint out;

  // ── 경로 1: frame 참조 (표준) ──────────────────────────────────────────────
  if (!frame_name.empty()) {
    if (!model.existFrame(frame_name)) {
      RCLCPP_ERROR(logger(), "폐쇄 체인 frame 을 찾을 수 없습니다: %s", frame_name.c_str());
      throw std::runtime_error("constraint_builder: frame 미존재: " + frame_name);
    }
    const auto fid = model.getFrameId(frame_name);
    out.joint_id = model.frames[fid].parentJoint;
    // frames[fid].placement = X_parentJoint_frame — 그대로 joint frame 기준 placement.
    out.joint_M_closure = model.frames[fid].placement;
    return out;
  }

  // ── 경로 2: link + link-frame 기준 origin (fallback) ───────────────────────
  if (link_name.empty()) {
    throw std::runtime_error(
        "constraint_builder: endpoint 에 frame 참조도 link 도 지정되지 않았습니다");
  }
  if (!model.existFrame(link_name)) {
    RCLCPP_ERROR(logger(), "폐쇄 체인 link 를 찾을 수 없습니다: %s", link_name.c_str());
    throw std::runtime_error("constraint_builder: link 미존재: " + link_name);
  }
  const auto fid = model.getFrameId(link_name);
  out.joint_id = model.frames[fid].parentJoint;
  // link frame 기준 origin → joint frame 기준으로 변환:
  //   joint_M_closure = joint_M_link * link_M_closure
  //   joint_M_link = frames[link].placement (BODY frame 이면 보통 Identity, 그러나
  //   fixed-joint 로 붙은 frame 은 non-identity → 반드시 합성해야 drift 없음).
  out.joint_M_closure = model.frames[fid].placement * MakeSe3(xyz, rpy);
  return out;
}

pinocchio::RigidConstraintModel BuildRigidConstraint(const pinocchio::Model& model,
                                                     const ClosedChainInfo& spec) {
  const ResolvedEndpoint ep1 =
      ResolveEndpoint(model, spec.frame_1, spec.link_a, spec.offset_a_xyz, spec.offset_a_rpy);
  const ResolvedEndpoint ep2 =
      ResolveEndpoint(model, spec.frame_2, spec.link_b, spec.offset_b_xyz, spec.offset_b_rpy);

  const auto contact_type = spec.is_6d ? pinocchio::CONTACT_6D : pinocchio::CONTACT_3D;
  const auto rf = ToPinocchioRF(spec.reference_frame);

  pinocchio::RigidConstraintModel constraint(contact_type, model, ep1.joint_id, ep1.joint_M_closure,
                                             ep2.joint_id, ep2.joint_M_closure, rf);
  constraint.name = spec.name;

  // ── Baumgarte 안정화 ───────────────────────────────────────────────────────
  // loop constraint 는 acceleration-level 만으로는 적분 drift 로 발산하므로 사실상 필수.
  // Kp > 0 일 때만 대입 (Pinocchio 4.0: 스칼라 double, baumgarte_corrector_parameters()).
  if (spec.baumgarte_kp > 0.0) {
    const double kd = spec.baumgarte_kd > 0.0
                          ? spec.baumgarte_kd
                          : 2.0 * std::sqrt(spec.baumgarte_kp);  // 임계감쇠 근처
    constraint.baumgarte_corrector_parameters().Kp = spec.baumgarte_kp;
    constraint.baumgarte_corrector_parameters().Kd = kd;
  }

  RCLCPP_INFO(
      logger(), "폐쇄 체인 등록: '%s' (%s, RF=%s, joint1=%lu, joint2=%lu, Kp=%.3f, Kd=%.3f)",
      spec.name.c_str(), LoopConstraintTypeToString(spec.Type()),
      LoopReferenceFrameToString(spec.reference_frame), static_cast<unsigned long>(ep1.joint_id),
      static_cast<unsigned long>(ep2.joint_id), constraint.baumgarte_corrector_parameters().Kp,
      constraint.baumgarte_corrector_parameters().Kd);

  return constraint;
}

std::vector<pinocchio::RigidConstraintModel> BuildRigidConstraints(
    const pinocchio::Model& model, const std::vector<ClosedChainInfo>& specs) {
  std::vector<pinocchio::RigidConstraintModel> out;
  out.reserve(specs.size());
  for (const auto& spec : specs) {
    try {
      out.push_back(BuildRigidConstraint(model, spec));
    } catch (const std::exception& e) {
      throw std::runtime_error("constraint_builder: 폐쇄 체인 '" + spec.name +
                               "' 구축 실패: " + e.what());
    }
  }
  return out;
}

}  // namespace rtc_urdf_bridge
