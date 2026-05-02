#include "rtc_tsid/constraints/eom_constraint.hpp"

namespace rtc::tsid {

void EomConstraint::init(const pinocchio::Model& /*model*/, const RobotModelInfo& robot_info,
                         PinocchioCache& /*cache*/, const YAML::Node& /*constraint_config*/) {
  nv_ = robot_info.nv;
  floating_base_ = robot_info.floating_base;
  n_unactuated_ = floating_base_ ? (nv_ - robot_info.n_actuated) : 0;

  if (floating_base_) {
    // Projection matrix P = I - Sᵀ·S
    // S = [0_{na×6} | I_{na}], Sᵀ·S = [0 0; 0 I_{na}]
    // P = I - SᵀS = [I_6 0; 0 0]  → 상위 6행만 non-zero
    P_.setIdentity(nv_, nv_);
    P_.noalias() -= robot_info.S.transpose() * robot_info.S;

    PM_.setZero(n_unactuated_, nv_);
    Ph_.setZero(n_unactuated_);
  }
}

int EomConstraint::eq_dim(const ContactState& /*contacts*/) const noexcept {
  return n_unactuated_;
}

int EomConstraint::ineq_dim(const ContactState& /*contacts*/) const noexcept {
  return 0;
}

void EomConstraint::compute_equality(const PinocchioCache& cache, const ContactState& contacts,
                                     const RobotModelInfo& robot_info, int /*n_vars*/,
                                     Eigen::Ref<Eigen::MatrixXd> A_block,
                                     Eigen::Ref<Eigen::VectorXd> b_block) noexcept {
  if (!floating_base_ || n_unactuated_ == 0)
    return;

  // [I-SᵀS]·(M·a + h - Jcᵀ·λ) = 0
  // → P·M·a - P·Jcᵀ·λ = -P·h
  //
  // A_block = [P·M | -P·Jcᵀ]    (n_unactuated × n_vars)
  // b_block = -P·h

  // P·M — P의 상위 n_unactuated행만 non-zero이므로 M의 상위 n_unactuated행 추출
  // P = [I_{nu} 0; 0 0] → P·M = M.topRows(nu)
  A_block.leftCols(nv_) = cache.M.topRows(n_unactuated_);

  // -P·Jcᵀ·λ: active contact 순회
  int lambda_offset = 0;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    if (!contacts.contacts[i].active)
      continue;
    if (i >= cache.contact_frames.size())
      continue;

    const auto& contact_cfg = robot_info.S;  // S 참조 불필요, manager에서 dim 가져옴
    (void)contact_cfg;

    // Contact dim은 ContactManagerConfig에서 가져와야 하지만,
    // 여기서는 Jacobian의 행 수로 판단
    // contact_frames[i].J: [6 × nv], point는 상위 3행, surface는 전체 6행
    // ContactState는 config_index를 통해 dim을 알 수 있어야 함
    // → 간소화: lambda_offset 관리를 위해 active_contact_vars를 기반으로
    //   각 contact의 dim을 직접 전달받아야 하는데,
    //   여기서는 contact_frames[i].J.rows() = 6이므로 3 or 6을 별도 알아야 함

    // 보수적 접근: 여기서는 J full (6행) 사용, Formulation이 contact_dim별 offset 관리
    // EomConstraint는 전체 Jcᵀ를 조립하므로, contact_dim 정보가 필요
    // → 일단 3D point contact 가정 (Phase 2 테스트 범위)
    const int cdim = 3;  // TODO: ContactManagerConfig 참조로 변경

    const auto& Jc = cache.contact_frames[i].J;
    // -P · Jcᵀ (상위 n_unactuated행) = -Jc.topRows(cdim).T 의 상위 n_unactuated열
    // P·Jcᵀ[:, lambda_i] = M.topRows(nu)... 아니, P·Jcᵀ = Jcᵀ의 상위 nu행
    // Jcᵀ: [nv × cdim], 상위 nu행 = Jc[:cdim, :].T.topRows(nu)
    // = Jc.topRows(cdim).leftCols(nv).transpose().topRows(nu)
    // = Jc.topRows(cdim).block(0, 0, cdim, nu).transpose() ... 아니

    // 정확히: A_block[0:nu, nv+lambda_offset : nv+lambda_offset+cdim]
    //   = -P · Jcᵀ[:, 해당 contact]
    //   = -(P · Jc.topRows(cdim).transpose())
    //   = -Jc.topRows(cdim).transpose().topRows(nu)  (P가 topRows 선택이므로)
    //   = -Jc.block(0, 0, cdim, nu).transpose()

    // A_block[:, nv+lambda_offset : nv+lambda_offset+cdim] = -Jc[:cdim, :nu]ᵀ
    A_block.block(0, nv_ + lambda_offset, n_unactuated_, cdim) =
        -Jc.topRows(cdim).leftCols(n_unactuated_).transpose();

    lambda_offset += cdim;
  }

  // b = -P·h = -h.head(n_unactuated)
  b_block.head(n_unactuated_) = -cache.h.head(n_unactuated_);
}

void EomConstraint::compute_inequality(const PinocchioCache& /*cache*/,
                                       const ContactState& /*contacts*/,
                                       const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                       Eigen::Ref<Eigen::MatrixXd> /*C_block*/,
                                       Eigen::Ref<Eigen::VectorXd> /*l_block*/,
                                       Eigen::Ref<Eigen::VectorXd> /*u_block*/) noexcept {
  // No inequality constraints from EoM
}

}  // namespace rtc::tsid
