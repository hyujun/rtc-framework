// ── RtModelHandle 구현 ──────────────────────────────────────────────────────
#include "urdf_pinocchio_bridge/rt_model_handle.hpp"

// Pinocchio 알고리즘 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/constrained-dynamics.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <cstring>

namespace urdf_pinocchio_bridge
{

// ═══════════════════════════════════════════════════════════════════════════════
// 생성자 (non-RT) — 모든 버퍼 사전 할당
// ═══════════════════════════════════════════════════════════════════════════════

RtModelHandle::RtModelHandle(
  std::shared_ptr<const pinocchio::Model> model,
  std::vector<pinocchio::RigidConstraintModel> constraint_models)
: model_(std::move(model)),
  data_(*model_),
  q_(pinocchio::neutral(*model_)),
  v_(Eigen::VectorXd::Zero(model_->nv)),
  a_(Eigen::VectorXd::Zero(model_->nv)),
  tau_(Eigen::VectorXd::Zero(model_->nv)),
  J_(Eigen::MatrixXd::Zero(6, model_->nv)),
  constraint_models_(std::move(constraint_models))
{
  // 폐쇄 체인 구속 데이터 사전 생성
  constraint_datas_.reserve(constraint_models_.size());
  for (const auto & cm : constraint_models_) {
    constraint_datas_.emplace_back(cm);
  }

  // 구속 동역학 초기화
  if (!constraint_models_.empty()) {
    pinocchio::initConstraintDynamics(*model_, data_, constraint_models_);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// span → Eigen 복사 헬퍼
// ═══════════════════════════════════════════════════════════════════════════════

void RtModelHandle::CopyToEigen(
  std::span<const double> src, Eigen::VectorXd & dst) noexcept
{
  auto n = std::min(static_cast<Eigen::Index>(src.size()), dst.size());
  // memcpy가 가장 빠르고 Eigen VectorXd는 연속 메모리
  std::memcpy(dst.data(), src.data(), static_cast<std::size_t>(n) * sizeof(double));
}

// ═══════════════════════════════════════════════════════════════════════════════
// RT-safe compute 함수
// ═══════════════════════════════════════════════════════════════════════════════

void RtModelHandle::ComputeForwardKinematics(std::span<const double> q) noexcept
{
  CopyToEigen(q, q_);
  pinocchio::forwardKinematics(*model_, data_, q_);
  pinocchio::updateFramePlacements(*model_, data_);
}

void RtModelHandle::ComputeForwardKinematics(
  std::span<const double> q, std::span<const double> v) noexcept
{
  CopyToEigen(q, q_);
  CopyToEigen(v, v_);
  pinocchio::forwardKinematics(*model_, data_, q_, v_);
  pinocchio::updateFramePlacements(*model_, data_);
}

void RtModelHandle::ComputeJacobians(std::span<const double> q) noexcept
{
  CopyToEigen(q, q_);
  pinocchio::computeJointJacobians(*model_, data_, q_);
  pinocchio::updateFramePlacements(*model_, data_);
}

void RtModelHandle::GetFrameJacobian(
  pinocchio::FrameIndex frame_id,
  pinocchio::ReferenceFrame ref_frame,
  Eigen::Ref<Eigen::MatrixXd> J_out) noexcept
{
  J_out.setZero();
  pinocchio::getFrameJacobian(*model_, data_, frame_id, ref_frame, J_out);
}

void RtModelHandle::ComputeInverseDynamics(
  std::span<const double> q,
  std::span<const double> v,
  std::span<const double> a) noexcept
{
  CopyToEigen(q, q_);
  CopyToEigen(v, v_);
  CopyToEigen(a, a_);
  pinocchio::rnea(*model_, data_, q_, v_, a_);
}

void RtModelHandle::ComputeForwardDynamics(
  std::span<const double> q,
  std::span<const double> v,
  std::span<const double> tau) noexcept
{
  CopyToEigen(q, q_);
  CopyToEigen(v, v_);
  CopyToEigen(tau, tau_);
  pinocchio::aba(*model_, data_, q_, v_, tau_);
}

void RtModelHandle::ComputeNonLinearEffects(
  std::span<const double> q,
  std::span<const double> v) noexcept
{
  CopyToEigen(q, q_);
  CopyToEigen(v, v_);
  pinocchio::nonLinearEffects(*model_, data_, q_, v_);
}

void RtModelHandle::ComputeGeneralizedGravity(std::span<const double> q) noexcept
{
  CopyToEigen(q, q_);
  pinocchio::computeGeneralizedGravity(*model_, data_, q_);
}

void RtModelHandle::ComputeCoriolisMatrix(
  std::span<const double> q,
  std::span<const double> v) noexcept
{
  CopyToEigen(q, q_);
  CopyToEigen(v, v_);
  pinocchio::computeCoriolisMatrix(*model_, data_, q_, v_);
}

void RtModelHandle::ComputeMassMatrix(std::span<const double> q) noexcept
{
  CopyToEigen(q, q_);
  pinocchio::crba(*model_, data_, q_);
  // crba는 upper triangular만 채움 → 대칭화
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();
}

void RtModelHandle::ComputeConstraintDynamics(
  std::span<const double> q,
  std::span<const double> v,
  std::span<const double> tau) noexcept
{
  if (constraint_models_.empty()) return;

  CopyToEigen(q, q_);
  CopyToEigen(v, v_);
  CopyToEigen(tau, tau_);
  pinocchio::constraintDynamics(
    *model_, data_, q_, v_, tau_,
    constraint_models_, constraint_datas_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 결과 접근
// ═══════════════════════════════════════════════════════════════════════════════

const pinocchio::SE3 & RtModelHandle::GetFramePlacement(
  pinocchio::FrameIndex frame_id) const noexcept
{
  return data_.oMf[frame_id];
}

Eigen::Vector3d RtModelHandle::GetFramePosition(
  pinocchio::FrameIndex frame_id) const noexcept
{
  return data_.oMf[frame_id].translation();
}

Eigen::Matrix3d RtModelHandle::GetFrameRotation(
  pinocchio::FrameIndex frame_id) const noexcept
{
  return data_.oMf[frame_id].rotation();
}

Eigen::Ref<const Eigen::VectorXd> RtModelHandle::GetTau() const noexcept
{
  return data_.tau;
}

Eigen::Ref<const Eigen::VectorXd> RtModelHandle::GetDdq() const noexcept
{
  return data_.ddq;
}

Eigen::Ref<const Eigen::VectorXd> RtModelHandle::GetNonLinearEffects() const noexcept
{
  return data_.nle;
}

Eigen::Ref<const Eigen::VectorXd> RtModelHandle::GetGeneralizedGravity() const noexcept
{
  return data_.g;
}

Eigen::Ref<const Eigen::MatrixXd> RtModelHandle::GetCoriolisMatrix() const noexcept
{
  return data_.C;
}

Eigen::Ref<const Eigen::MatrixXd> RtModelHandle::GetMassMatrix() const noexcept
{
  return data_.M;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 메타데이터
// ═══════════════════════════════════════════════════════════════════════════════

int RtModelHandle::nq() const noexcept
{
  return model_->nq;
}

int RtModelHandle::nv() const noexcept
{
  return model_->nv;
}

pinocchio::FrameIndex RtModelHandle::GetFrameId(
  std::string_view frame_name) const noexcept
{
  if (!model_->existFrame(std::string(frame_name))) {
    return 0;  // universe frame
  }
  return model_->getFrameId(std::string(frame_name));
}

const pinocchio::Model & RtModelHandle::GetModel() const noexcept
{
  return *model_;
}

const pinocchio::Data & RtModelHandle::GetData() const noexcept
{
  return data_;
}

double RtModelHandle::ComputeMimicPosition(
  double mimicked_q, double multiplier, double offset) noexcept
{
  return multiplier * mimicked_q + offset;
}

}  // namespace urdf_pinocchio_bridge
