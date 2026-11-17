#include "ocp/internal/structured_quadratic_costs.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/vector-space.hpp>
#include <aligator/modelling/state-error.hpp>
#pragma GCC diagnostic pop

#include "rtc_urdf_bridge/free_flyer_root.hpp"

#include <boost/variant/get.hpp>

#include <cassert>
#include <cstddef>

namespace rtc::mpc::internal {

namespace {

using StageFunctionData = aligator::StageFunctionDataTpl<double>;
using RootBlock =
    Eigen::Matrix<double, StructuredQuadStateCost::kRootDim, StructuredQuadStateCost::kRootDim>;
using RootVector = Eigen::Matrix<double, StructuredQuadStateCost::kRootDim, 1>;

static_assert(StructuredQuadStateCost::kRootDim == pinocchio::JointModelFreeFlyer::NV,
              "the root block is the free flyer's tangent");

/// @return true when @p joint is one of @p Types (a variant type check).
template <typename... Types>
[[nodiscard]] bool IsOneOf(const pinocchio::JointModel& joint) noexcept {
  return ((boost::get<Types>(&joint.toVariant()) != nullptr) || ...);
}

/// Joint types whose configuration is a vector space: their difference is
/// `q₁ − q₀` (so `nq == nv`) and its Jacobian the identity.
[[nodiscard]] bool IsEuclideanJoint(const pinocchio::JointModel& joint) noexcept {
  return IsOneOf<pinocchio::JointModelRX, pinocchio::JointModelRY, pinocchio::JointModelRZ,
                 pinocchio::JointModelRevoluteUnaligned, pinocchio::JointModelPX,
                 pinocchio::JointModelPY, pinocchio::JointModelPZ,
                 pinocchio::JointModelPrismaticUnaligned, pinocchio::JointModelTranslation>(joint);
}

/// @return true when @p w is `n × n` and every off-diagonal entry is exactly 0.
[[nodiscard]] bool IsExactlyDiagonal(const Eigen::MatrixXd& w, Eigen::Index n) {
  if (w.rows() != n || w.cols() != n) {
    return false;
  }
  for (Eigen::Index j = 0; j < n; ++j) {
    for (Eigen::Index i = 0; i < n; ++i) {
      if (i != j && w(i, j) != 0.0) {
        return false;
      }
    }
  }
  return true;
}

/// The data of a structured cost: only its own createData's reaches it
/// (CostStack builds each component's data from it).
[[nodiscard]] StructuredQuadCostData& AsStructuredData(
    aligator::CostDataAbstractTpl<double>& data) noexcept {
  assert(dynamic_cast<StructuredQuadCostData*>(&data) != nullptr);
  return static_cast<StructuredQuadCostData&>(data);
}

/// Zeroes @p buf once after a fallback wrote all of it; the structured writes
/// then only touch the structural entries.
template <typename Buffer>
void RestoreZeroPattern(bool& clean, Buffer& buf) noexcept {
  if (!clean) {
    buf.setZero();
    clean = true;
  }
}

}  // namespace

bool HasFreeFlyerRootAndEuclideanJoints(const pinocchio::Model& model) {
  // Joint 1 is the root, at idx_q = idx_v = 0 (pinocchio orders the joints'
  // coordinates by index).
  if (!rtc_urdf_bridge::IsFreeFlyerRoot(model)) {
    return false;
  }
  for (std::size_t j = 2; j < model.joints.size(); ++j) {
    if (!IsEuclideanJoint(model.joints[j])) {
      return false;
    }
  }
  return true;
}

// ── State cost ───────────────────────────────────────────────────────────────

void StructuredQuadStateCost::DetectStructure() {
  const auto* phase =
      dynamic_cast<const aligator::MultibodyPhaseSpace<double>*>(&*residual().space_);
  if (phase == nullptr) {
    return;
  }
  const pinocchio::Model& model = phase->getModel();
  structured_ = HasFreeFlyerRootAndEuclideanJoints(model) && this->gauss_newton &&
                this->ndx() == phase->ndx() && residual_->nr == this->ndx() &&
                IsExactlyDiagonal(weights_, this->ndx());
}

std::shared_ptr<StructuredQuadStateCost::CostData> StructuredQuadStateCost::createData() const {
  // CostDataAbstractTpl zeroes grad_ and hess_: the pattern holds.
  return std::make_shared<StructuredQuadCostData>(this->ndx(), this->nu, residual_->createData());
}

void StructuredQuadStateCost::computeGradients(const ConstVectorRef& x, const ConstVectorRef& u,
                                               CostData& data_) const {
  StructuredQuadCostData& data = AsStructuredData(data_);
  if (!UseStructure()) {
    data.grad_pattern_clean = false;
    Base::computeGradients(x, u, data);
    return;
  }
  StageFunctionData& rd = *data.residual_data;
  residual_->computeJacobians(x, u, rd);  // J_b; the rest of J is the identity
  RestoreZeroPattern(data.grad_pattern_clean, data.grad_);
  const Eigen::Index n_rest = this->ndx() - kRootDim;
  // W r, from the value of the last evaluate (as Aligator's path reads it).
  data.Wv_buf.noalias() = weights_.diagonal().cwiseProduct(rd.value_);
  const RootBlock jb = rd.jac_buffer_.topLeftCorner<kRootDim, kRootDim>();
  data.grad_.head<kRootDim>().noalias() = jb.transpose() * data.Wv_buf.head<kRootDim>();
  data.grad_.segment(kRootDim, n_rest) = data.Wv_buf.tail(n_rest);
  // grad_u stays 0.
}

void StructuredQuadStateCost::computeHessians(const ConstVectorRef& x, const ConstVectorRef& u,
                                              CostData& data_) const {
  StructuredQuadCostData& data = AsStructuredData(data_);
  if (!UseStructure()) {
    data.hess_pattern_clean = false;
    Base::computeHessians(x, u, data);
    return;
  }
  RestoreZeroPattern(data.hess_pattern_clean, data.hess_);
  const Eigen::Index n_rest = this->ndx() - kRootDim;
  // J_b as computeGradients left it, as Aligator's path reads J.
  const RootBlock jb = data.residual_data->jac_buffer_.topLeftCorner<kRootDim, kRootDim>();
  const RootVector w_b = weights_.diagonal().head<kRootDim>();
  // (J_bᵀ W_b) J_b: Aligator's association, on the one non-identity block.
  const RootBlock jtw = jb.transpose() * w_b.asDiagonal();
  data.hess_.topLeftCorner<kRootDim, kRootDim>().noalias() = jtw * jb;
  data.hess_.diagonal().segment(kRootDim, n_rest) = weights_.diagonal().segment(kRootDim, n_rest);
  // Every other entry stays 0.
}

// ── Control cost ─────────────────────────────────────────────────────────────

void StructuredQuadControlCost::DetectStructure() {
  const auto* error = dynamic_cast<const aligator::ControlErrorResidualTpl<double>*>(&*residual_);
  if (error == nullptr) {
    return;
  }
  const auto* euclidean =
      dynamic_cast<const aligator::VectorSpaceTpl<double, Eigen::Dynamic>*>(&*error->space_);
  structured_ = euclidean != nullptr && this->gauss_newton && euclidean->nx() == this->nu &&
                euclidean->ndx() == this->nu && residual_->nr == this->nu &&
                IsExactlyDiagonal(weights_, this->nu);
}

std::shared_ptr<StructuredQuadControlCost::CostData> StructuredQuadControlCost::createData() const {
  // CostDataAbstractTpl zeroes grad_ and hess_: the pattern holds.
  return std::make_shared<StructuredQuadCostData>(this->ndx(), this->nu, residual_->createData());
}

void StructuredQuadControlCost::computeGradients(const ConstVectorRef& x, const ConstVectorRef& u,
                                                 CostData& data_) const {
  StructuredQuadCostData& data = AsStructuredData(data_);
  if (!UseStructure()) {
    data.grad_pattern_clean = false;
    Base::computeGradients(x, u, data);
    return;
  }
  RestoreZeroPattern(data.grad_pattern_clean, data.grad_);
  // J = [0, I]: no Jacobian to evaluate. W r from the last evaluate's r.
  data.Wv_buf.noalias() = weights_.diagonal().cwiseProduct(data.residual_data->value_);
  data.grad_.tail(this->nu) = data.Wv_buf;
  // grad_x stays 0.
}

void StructuredQuadControlCost::computeHessians(const ConstVectorRef& x, const ConstVectorRef& u,
                                                CostData& data_) const {
  StructuredQuadCostData& data = AsStructuredData(data_);
  if (!UseStructure()) {
    data.hess_pattern_clean = false;
    Base::computeHessians(x, u, data);
    return;
  }
  RestoreZeroPattern(data.hess_pattern_clean, data.hess_);
  data.hess_.bottomRightCorner(this->nu, this->nu).diagonal() = weights_.diagonal();
  // Every other entry stays 0.
}

}  // namespace rtc::mpc::internal
