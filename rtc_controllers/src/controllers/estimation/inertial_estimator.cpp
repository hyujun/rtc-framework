#include "rtc_controllers/estimation/inertial_estimator.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace rtc::estimation {
namespace {

/// Reject a threshold that cannot do its job. Non-finite is checked FIRST and
/// separately from the sign: `0.0 < NaN` is false, so a lone `<= 0.0` test lets
/// a NaN threshold through and every later comparison against it is false too —
/// the gate would be permanently open while reading as configured.
void RequirePositive(double v, const char* name) {
  if (!std::isfinite(v) || v <= 0.0) {
    throw std::invalid_argument(std::string("InertialEstimator::Init: ") + name +
                                " must be finite and > 0, got " + std::to_string(v));
  }
}

}  // namespace

void InertialEstimator::Init(int nv, const Config& config) {
  if (nv < 1 || nv > kMaxObserverDof) {
    throw std::invalid_argument("InertialEstimator::Init: nv out of range [1, " +
                                std::to_string(kMaxObserverDof) + "], got " + std::to_string(nv));
  }
  // The forgetting factor has an upper bound the others do not: above 1 the
  // accumulator grows without limit and σ_min crosses any gate on age alone,
  // which would turn the pose-diversity test into a timer.
  if (!std::isfinite(config.forgetting_factor) || config.forgetting_factor <= 0.0 ||
      config.forgetting_factor > 1.0) {
    throw std::invalid_argument(
        "InertialEstimator::Init: forgetting_factor must be finite and in (0, 1], got " +
        std::to_string(config.forgetting_factor));
  }
  RequirePositive(config.min_param_sigma, "min_param_sigma");
  RequirePositive(config.min_mass, "min_mass");
  RequirePositive(config.max_com_offset, "max_com_offset");
  RequirePositive(config.max_inertia_column, "max_inertia_column");

  nv_ = nv;
  config_ = config;
  r_buf_.setZero(nv);
  fit_buf_.setZero(nv);
  initialized_ = true;
  ResetRtState();
}

void InertialEstimator::Reject(InertialInvalidReason reason) noexcept {
  reason_ = reason;
  estimate_.reason = reason;
  estimate_.valid = false;
}

void InertialEstimator::Hold(InertialInvalidReason reason) noexcept {
  if (!initialized_) {
    Reject(InertialInvalidReason::kNotInitialized);
    return;
  }
  // kNone would assert "valid" through a path that never looked at data.
  Reject(reason == InertialInvalidReason::kNone ? InertialInvalidReason::kHeld : reason);
}

void InertialEstimator::ResetRtState() noexcept {
  A_.setZero();
  b_.setZero();
  phi_.setZero();
  estimate_ = InertialEstimate{};
  reason_ = initialized_ ? InertialInvalidReason::kInsufficientPoseDiversity
                         : InertialInvalidReason::kNotInitialized;
  estimate_.reason = reason_;
}

void InertialEstimator::Update(const Eigen::Ref<const Eigen::MatrixXd>& Y,
                               std::span<const double> r) noexcept {
  if (!initialized_) {
    Reject(InertialInvalidReason::kNotInitialized);
    return;
  }
  if (Y.rows() != nv_ || Y.cols() != 10 || static_cast<int>(r.size()) < nv_) {
    Reject(InertialInvalidReason::kShortInput);
    return;
  }
  if (!Y.allFinite()) {
    Reject(InertialInvalidReason::kNonFiniteInput);
    return;
  }
  for (int i = 0; i < nv_; ++i) {
    if (!std::isfinite(r[static_cast<std::size_t>(i)])) {
      Reject(InertialInvalidReason::kNonFiniteInput);
      return;
    }
    r_buf_[i] = r[static_cast<std::size_t>(i)];
  }

  // AC 10 enforced structurally rather than by trusting the caller's velocity
  // gate: if the inertia columns carry anything, this tick's residual contains
  // torque our four parameters cannot represent, and fitting it would push that
  // torque into m̂ and ĉ. Refusing is the only honest option in this lane.
  if (Y.rightCols<6>().cwiseAbs().maxCoeff() > config_.max_inertia_column) {
    Reject(InertialInvalidReason::kDynamicExcitation);
    return;
  }

  const auto Y4 = Y.leftCols<kParams>();

  // RLS accumulation. Decay first, then fold this tick in.
  A_ = config_.forgetting_factor * A_ + Y4.transpose() * Y4;
  b_ = config_.forgetting_factor * b_ + Y4.transpose() * r_buf_;

  // σ_min of the accumulated normal matrix is the pose-diversity meter. A_ is
  // symmetric PSD by construction, so its eigenvalues ARE its singular values;
  // a 4×4 self-adjoint solve is fixed-size and allocation-free.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, kParams, kParams>> eig(A_);
  if (eig.info() != Eigen::Success) {
    Reject(InertialInvalidReason::kSolverFailed);
    return;
  }
  const auto& evals = eig.eigenvalues();
  const double sigma_min = std::max(0.0, evals.minCoeff());
  const double sigma_max = std::max(0.0, evals.maxCoeff());
  estimate_.param_sigma_min = sigma_min;

  // Rank relative to the largest eigenvalue, not an absolute floor: A_ grows
  // with accumulated data, so an absolute cut would report rank on age.
  const double rank_tol = sigma_max * 1e-12;
  std::uint8_t rank = 0;
  for (int i = 0; i < kParams; ++i) {
    if (evals[i] > rank_tol) {
      ++rank;
    }
  }
  estimate_.param_rank = rank;

  // THE structural gate. One pose gives rank 3 with σ₄ exactly 0; until the arm
  // has visited poses spanning the missing direction there is no fourth
  // equation, and any solve fills it with an arbitrary number.
  if (rank < kParams || sigma_min < config_.min_param_sigma) {
    Reject(InertialInvalidReason::kInsufficientPoseDiversity);
    return;
  }

  const Eigen::LDLT<Eigen::Matrix<double, kParams, kParams>> ldlt(A_);
  if (ldlt.info() != Eigen::Success) {
    Reject(InertialInvalidReason::kSolverFailed);
    return;
  }
  phi_ = ldlt.solve(b_);
  if (!phi_.allFinite()) {
    Reject(InertialInvalidReason::kSolverFailed);
    return;
  }

  // [C3] physical consistency, reduced to what four parameters can assert.
  const double mass = phi_[0];
  if (!(mass > config_.min_mass)) {
    Reject(InertialInvalidReason::kNonPositiveMass);
    return;
  }
  const Eigen::Vector3d first_moment = phi_.template tail<3>();
  const Eigen::Vector3d com = first_moment / mass;
  if (com.norm() > config_.max_com_offset) {
    Reject(InertialInvalidReason::kComOutOfBounds);
    return;
  }

  fit_buf_.head(nv_).noalias() = Y4 * phi_;
  estimate_.fit_error = (fit_buf_.head(nv_) - r_buf_.head(nv_)).cwiseAbs().maxCoeff();

  estimate_.mass = mass;
  for (int i = 0; i < 3; ++i) {
    estimate_.first_moment[static_cast<std::size_t>(i)] = first_moment[i];
    estimate_.com[static_cast<std::size_t>(i)] = com[i];
  }
  reason_ = InertialInvalidReason::kNone;
  estimate_.reason = reason_;
  estimate_.valid = true;
}

}  // namespace rtc::estimation
