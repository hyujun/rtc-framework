// ── loop_verification 구현 ───────────────────────────────────────────────────
#include "rtc_urdf_bridge/loop_verification.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <span>

namespace rtc_urdf_bridge {

// CONTACT_6D → 6, 그 외(3D) → 3. RigidConstraintData/residualSize 의미 모호성 회피 위해
// type 에서 직접 유도한다 (안정적·명시적).
int ConstraintDim(const pinocchio::RigidConstraintModel& cm) noexcept {
  return cm.type == pinocchio::CONTACT_6D ? 6 : 3;
}

int TotalConstraintDim(const std::vector<pinocchio::RigidConstraintModel>& constraints) noexcept {
  int total = 0;
  for (const auto& cm : constraints) {
    total += ConstraintDim(cm);
  }
  return total;
}

bool IsFrameDownstreamOfLoop(const pinocchio::Model& model,
                             std::span<const pinocchio::JointIndex> actuated_joint_ids,
                             pinocchio::FrameIndex frame_id) noexcept {
  if (frame_id >= model.frames.size()) {
    return false;
  }
  // frame → 부모 joint → universe 까지의 support(조상) 경로.
  const auto parent_joint = model.frames[frame_id].parentJoint;
  const auto& support = model.supports[parent_joint];
  // 조상 중 movable & non-actuated(=loop-passive) 가 하나라도 있으면 하류다.
  for (const auto jid : support) {
    const auto jidx = static_cast<std::size_t>(jid);
    if (jid == 0 || model.nvs[jidx] == 0) {
      continue;  // universe / fixed
    }
    const bool is_actuated = std::find(actuated_joint_ids.begin(), actuated_joint_ids.end(), jid) !=
                             actuated_joint_ids.end();
    if (!is_actuated) {
      return true;
    }
  }
  return false;
}

void ConstraintKinScratch::Resize(int nv) {
  J1.setZero(6, nv);
  J2.setZero(6, nv);
  Jc1.setZero(6, nv);
  Jc2.setZero(6, nv);
  Jrel.setZero(6, nv);
  tmp3.setZero(3, nv);
}

void FillConstraintKinematicsRow(const pinocchio::Model& model, pinocchio::Data& data,
                                 const pinocchio::RigidConstraintModel& cm, int row, int dim,
                                 ConstraintKinScratch& scratch, Eigen::Ref<Eigen::VectorXd> phi,
                                 Eigen::Ref<Eigen::MatrixXd> Jc) noexcept {
  const pinocchio::JointIndex j1 = cm.joint1_id;
  const pinocchio::JointIndex j2 = cm.joint2_id;

  // 두 constraint frame 의 world placement 와 상대 변위.
  const pinocchio::SE3 oMc1 = data.oMi[j1] * cm.joint1_placement;
  const pinocchio::SE3 oMc2 = data.oMi[j2] * cm.joint2_placement;
  const pinocchio::SE3 c1Mc2 = oMc1.actInv(oMc2);  // = oMc1.inverse() * oMc2

  // joint LOCAL Jacobian → constraint frame(c) 로 이동: J_c = Ad(cMj) · J_joint.
  scratch.J1.setZero();
  scratch.J2.setZero();
  pinocchio::getJointJacobian(model, data, j1, pinocchio::LOCAL, scratch.J1);
  pinocchio::getJointJacobian(model, data, j2, pinocchio::LOCAL, scratch.J2);

  const Eigen::Matrix<double, 6, 6> Ad_c1j1 = cm.joint1_placement.inverse().toActionMatrix();
  const Eigen::Matrix<double, 6, 6> Ad_c2j2 = cm.joint2_placement.inverse().toActionMatrix();
  const Eigen::Matrix<double, 6, 6> Ad_c1c2 = c1Mc2.toActionMatrix();

  scratch.Jc1.noalias() = Ad_c1j1 * scratch.J1;  // 6 × nv, c1 frame
  scratch.Jc2.noalias() = Ad_c2j2 * scratch.J2;  // 6 × nv, c2 frame
  // c2 의 c1 기준 상대 spatial velocity: Ad(c1Mc2)·Jc2 − Jc1.
  scratch.Jrel.noalias() = Ad_c1c2 * scratch.Jc2;
  scratch.Jrel -= scratch.Jc1;  // 6 × nv

  if (dim == 6) {
    phi.segment<6>(row) = pinocchio::log6(c1Mc2).toVector();
    Jc.middleRows(row, 6) = scratch.Jrel;
  } else {
    const Eigen::Vector3d p = c1Mc2.translation();
    phi.segment<3>(row) = p;
    // φ = translation(c1Mc2) 의 정확한 시간미분: dp/dt = v_s − [p]×·ω_s,
    // 여기서 [v_s; ω_s] = Jrel (c1 frame 상대 spatial velocity). p→0(구속 만족) 에서만
    // v_s 와 일치하므로 angular 커플링 항 −[p]×·ω_s 를 포함해야 non-consistent q 에서도
    // Jc = ∂φ/∂v 가 정확하다 (projection/velocity/rank self-consistency).
    Eigen::Matrix3d p_hat;
    p_hat << 0.0, -p.z(), p.y(),  //
        p.z(), 0.0, -p.x(),       //
        -p.y(), p.x(), 0.0;
    scratch.tmp3.noalias() = p_hat * scratch.Jrel.bottomRows<3>();
    Jc.middleRows(row, 3) = scratch.Jrel.topRows<3>() - scratch.tmp3;
  }
}

ConstraintKinematics ComputeConstraintKinematics(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q) {
  // computeJointJacobians 는 data.oMi 와 data.J(모든 joint LOCAL Jacobian) 를 채운다.
  pinocchio::computeJointJacobians(model, data, q);

  const int total = TotalConstraintDim(constraints);
  ConstraintKinematics out;
  out.phi = Eigen::VectorXd::Zero(total);
  out.Jc = Eigen::MatrixXd::Zero(total, model.nv);
  out.row_offsets.reserve(constraints.size());
  out.row_sizes.reserve(constraints.size());

  ConstraintKinScratch scratch;
  scratch.Resize(model.nv);  // loop 밖 1회 할당 → constraint 간 재사용

  int row = 0;
  for (const auto& cm : constraints) {
    const int dim = ConstraintDim(cm);
    out.row_offsets.push_back(row);
    out.row_sizes.push_back(dim);
    FillConstraintKinematicsRow(model, data, cm, row, dim, scratch, out.phi, out.Jc);
    row += dim;
  }

  return out;
}

std::vector<ClosureError> ComputeClosureErrors(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q) {
  pinocchio::forwardKinematics(model, data, q);

  std::vector<ClosureError> errors;
  errors.reserve(constraints.size());
  for (const auto& cm : constraints) {
    const int dim = ConstraintDim(cm);
    const pinocchio::SE3 oMc1 = data.oMi[cm.joint1_id] * cm.joint1_placement;
    const pinocchio::SE3 oMc2 = data.oMi[cm.joint2_id] * cm.joint2_placement;
    const pinocchio::SE3 c1Mc2 = oMc1.actInv(oMc2);

    ClosureError e;
    e.name = cm.name;
    e.dim = dim;
    e.norm = dim == 6 ? pinocchio::log6(c1Mc2).toVector().norm() : c1Mc2.translation().norm();
    errors.push_back(std::move(e));
  }
  return errors;
}

JacobianReport AnalyzeConstraintJacobian(
    const pinocchio::Model& model, pinocchio::Data& data,
    const std::vector<pinocchio::RigidConstraintModel>& constraints, const Eigen::VectorXd& q) {
  const ConstraintKinematics kin = ComputeConstraintKinematics(model, data, constraints, q);

  JacobianReport report;
  report.rows = static_cast<int>(kin.Jc.rows());

  if (report.rows == 0) {
    report.full_rank = true;
    return report;
  }

  // rank + 최소 특이값
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(kin.Jc);
  const Eigen::VectorXd sv = svd.singularValues();
  report.smallest_singular_value = sv.size() > 0 ? sv(sv.size() - 1) : 0.0;
  Eigen::FullPivLU<Eigen::MatrixXd> lu(kin.Jc);
  report.rank = static_cast<int>(lu.rank());
  report.full_rank = report.rank == report.rows;

  // Delassus D = Jc M⁻¹ Jcᵀ 의 조건수. crba 는 upper 만 채우므로 대칭화.
  pinocchio::crba(model, data, q);
  Eigen::MatrixXd M = data.M;
  M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();

  const Eigen::MatrixXd MinvJt = M.ldlt().solve(kin.Jc.transpose());  // nv × rows
  const Eigen::MatrixXd delassus = kin.Jc * MinvJt;                   // rows × rows

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_d(delassus);
  const Eigen::VectorXd sd = svd_d.singularValues();
  if (sd.size() > 0 && sd(sd.size() - 1) > 0.0) {
    report.delassus_condition = sd(0) / sd(sd.size() - 1);
  } else {
    report.delassus_condition = std::numeric_limits<double>::infinity();
  }

  return report;
}

}  // namespace rtc_urdf_bridge
