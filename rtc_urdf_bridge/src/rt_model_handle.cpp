// ── RtModelHandle 구현 ──────────────────────────────────────────────────────
#include "rtc_urdf_bridge/rt_model_handle.hpp"

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
#include <string>

namespace rtc_urdf_bridge
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    pinocchio::initConstraintDynamics(*model_, data_, constraint_models_);
#pragma GCC diagnostic pop
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

void RtModelHandle::CopyToEigenReordered(
  std::span<const double> src,
  Eigen::VectorXd & dst,
  const std::vector<int> & reorder_map) noexcept
{
  if (reorder_map.empty()) {
    CopyToEigen(src, dst);
    return;
  }
  auto n = std::min(src.size(), reorder_map.size());
  for (std::size_t i = 0; i < n; ++i) {
    dst[reorder_map[i]] = src[i];
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// RT-safe compute 함수
// ═══════════════════════════════════════════════════════════════════════════════

void RtModelHandle::ComputeForwardKinematics(std::span<const double> q) noexcept
{
  CopyToEigenReordered(q, q_, q_reorder_map_);
  pinocchio::forwardKinematics(*model_, data_, q_);
  pinocchio::updateFramePlacements(*model_, data_);
}

void RtModelHandle::ComputeForwardKinematics(
  std::span<const double> q, std::span<const double> v) noexcept
{
  CopyToEigenReordered(q, q_, q_reorder_map_);
  CopyToEigenReordered(v, v_, v_reorder_map_);
  pinocchio::forwardKinematics(*model_, data_, q_, v_);
  pinocchio::updateFramePlacements(*model_, data_);
}

void RtModelHandle::ComputeJacobians(std::span<const double> q) noexcept
{
  CopyToEigenReordered(q, q_, q_reorder_map_);
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
  CopyToEigenReordered(q, q_, q_reorder_map_);
  CopyToEigenReordered(v, v_, v_reorder_map_);
  CopyToEigenReordered(a, a_, v_reorder_map_);
  pinocchio::rnea(*model_, data_, q_, v_, a_);
}

void RtModelHandle::ComputeForwardDynamics(
  std::span<const double> q,
  std::span<const double> v,
  std::span<const double> tau) noexcept
{
  CopyToEigenReordered(q, q_, q_reorder_map_);
  CopyToEigenReordered(v, v_, v_reorder_map_);
  CopyToEigenReordered(tau, tau_, v_reorder_map_);
  pinocchio::aba(*model_, data_, q_, v_, tau_);
}

void RtModelHandle::ComputeNonLinearEffects(
  std::span<const double> q,
  std::span<const double> v) noexcept
{
  CopyToEigenReordered(q, q_, q_reorder_map_);
  CopyToEigenReordered(v, v_, v_reorder_map_);
  pinocchio::nonLinearEffects(*model_, data_, q_, v_);
}

void RtModelHandle::ComputeGeneralizedGravity(std::span<const double> q) noexcept
{
  CopyToEigenReordered(q, q_, q_reorder_map_);
  pinocchio::computeGeneralizedGravity(*model_, data_, q_);
}

void RtModelHandle::ComputeCoriolisMatrix(
  std::span<const double> q,
  std::span<const double> v) noexcept
{
  CopyToEigenReordered(q, q_, q_reorder_map_);
  CopyToEigenReordered(v, v_, v_reorder_map_);
  pinocchio::computeCoriolisMatrix(*model_, data_, q_, v_);
}

void RtModelHandle::ComputeMassMatrix(std::span<const double> q) noexcept
{
  CopyToEigenReordered(q, q_, q_reorder_map_);
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

  CopyToEigenReordered(q, q_, q_reorder_map_);
  CopyToEigenReordered(v, v_, v_reorder_map_);
  CopyToEigenReordered(tau, tau_, v_reorder_map_);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  pinocchio::constraintDynamics(
    *model_, data_, q_, v_, tau_,
    constraint_models_, constraint_datas_);
#pragma GCC diagnostic pop
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

// ═══════════════════════════════════════════════════════════════════════════════
// 관절 순서 재배열
// ═══════════════════════════════════════════════════════════════════════════════

bool RtModelHandle::SetJointOrder(
  std::span<const std::string> external_joint_names)
{
  std::vector<int> q_map;
  std::vector<int> v_map;

  for (const auto & name : external_joint_names) {
    if (!model_->existJointName(name)) {
      return false;
    }
    auto jid = model_->getJointId(name);
    auto pin_q_start = model_->idx_qs[jid];
    auto pin_v_start = model_->idx_vs[jid];
    auto nq_j = model_->nqs[jid];
    auto nv_j = model_->nvs[jid];

    for (int k = 0; k < nq_j; ++k) {
      q_map.push_back(static_cast<int>(pin_q_start + k));
    }
    for (int k = 0; k < nv_j; ++k) {
      v_map.push_back(static_cast<int>(pin_v_start + k));
    }
  }

  // identity 순서이면 매핑 생략 (zero-overhead)
  bool is_identity = (static_cast<int>(q_map.size()) == model_->nq);
  if (is_identity) {
    for (std::size_t i = 0; i < q_map.size(); ++i) {
      if (q_map[i] != static_cast<int>(i)) { is_identity = false; break; }
    }
  }

  if (is_identity) {
    q_reorder_map_.clear();
    v_reorder_map_.clear();
  } else {
    q_reorder_map_ = std::move(q_map);
    v_reorder_map_ = std::move(v_map);
  }
  return true;
}

bool RtModelHandle::HasJointReorder() const noexcept
{
  return !q_reorder_map_.empty();
}

std::vector<std::string> RtModelHandle::GetPinocchioJointNames() const
{
  std::vector<std::string> names;
  // names[0] = "universe" (skip)
  names.reserve(static_cast<std::size_t>(model_->njoints - 1));
  for (int i = 1; i < model_->njoints; ++i) {
    names.push_back(model_->names[static_cast<std::size_t>(i)]);
  }
  return names;
}

void RtModelHandle::ReorderOutput(
  Eigen::Ref<const Eigen::VectorXd> pinocchio_vec,
  std::span<double> external_out) const noexcept
{
  if (v_reorder_map_.empty()) {
    auto n = std::min(static_cast<Eigen::Index>(external_out.size()),
                      pinocchio_vec.size());
    std::memcpy(external_out.data(), pinocchio_vec.data(),
                static_cast<std::size_t>(n) * sizeof(double));
    return;
  }
  auto n = std::min(external_out.size(), v_reorder_map_.size());
  for (std::size_t i = 0; i < n; ++i) {
    external_out[i] = pinocchio_vec[v_reorder_map_[i]];
  }
}

}  // namespace rtc_urdf_bridge
