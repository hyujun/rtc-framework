// ── InertialEstimator RT-1 zero-allocation gate (#455) ──────────────────────
//
// TWO GATES WITH DIFFERENT REACH, and the difference is the whole reason this
// file is structured the way it is.
//
//   ScopedAllocGate counts global `operator new`. It sees across TU boundaries,
//   so it covers the REAL InertialEstimator::Update() — but it cannot see
//   Eigen, whose aligned_malloc calls std::malloc directly.
//
//   ScopedNoMalloc catches Eigen's allocator — but only for Eigen code
//   compiled in THIS translation unit (no_malloc_scope.hpp contract item 3).
//   InertialEstimator::Update() is compiled into the rtc_controllers library,
//   so pointing that gate at it would be FAIL-OPEN: a guaranteed green that
//   proves nothing.
//
// So the Eigen half is tested where it can actually be observed — by
// replicating the exact expressions Update() runs, in this TU, under the gate.
// That is a real measurement of the thing at risk (Y₄ᵀY₄ formed from a block of
// a dynamically-sized Eigen::Ref, then a 4×4 self-adjoint eigendecomposition and
// an LDLT), and the positive control below proves the gate is armed rather than
// asleep. What it does NOT prove is that the library's copy was compiled
// identically; that gap is inherent to the harness and is named here rather
// than papered over.
//
// no_malloc_scope.hpp MUST precede every Eigen header — it installs the Eigen
// allocation hook at include time.
#include "rtc_base/testing/no_malloc_scope.hpp"
#include "rtc_controllers/estimation/inertial_estimator.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace {

using rtc::estimation::InertialEstimator;

constexpr int kNv = 7;

Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d s;
  s << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return s;
}

Eigen::MatrixXd RegressorFor(const Eigen::Vector3d& g) {
  Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(kNv, 10);
  Y.block<3, 1>(0, 0) = g;
  Y.block<3, 3>(3, 1) = -Skew(g);
  return Y;
}

}  // namespace

// The gate is armed, not asleep. Without this, every assertion below could be
// green because ScopedNoMalloc silently did nothing.
TEST(InertialEstimatorRtSafety, EigenGatePositiveControl) {
  bool tripped = false;
  {
    const rtc::testing::ScopedNoMalloc eigen_gate;
    Eigen::MatrixXd a(8, 8);  // a runtime-sized Eigen allocation, in THIS TU
    a.setZero();
    tripped = eigen_gate.violations() > 0;
  }
  EXPECT_TRUE(tripped) << "the Eigen allocation gate never fired — it is inert";
}

// The Eigen half, measured where the gate can reach it.
TEST(InertialEstimatorRtSafety, UpdateExpressionsAreEigenAllocationFree) {
  const Eigen::MatrixXd Y = RegressorFor({0.0, 0.0, -9.81});
  const Eigen::Ref<const Eigen::MatrixXd> Yref(Y);
  Eigen::Matrix<double, 4, 4> A = Eigen::Matrix<double, 4, 4>::Zero();
  Eigen::Matrix<double, 4, 1> b = Eigen::Matrix<double, 4, 1>::Zero();
  Eigen::Matrix<double, 4, 1> phi = Eigen::Matrix<double, 4, 1>::Zero();
  Eigen::VectorXd r = Eigen::VectorXd::Constant(kNv, 0.3);
  Eigen::VectorXd fit = Eigen::VectorXd::Zero(kNv);

  // Warm-up outside the gate, exactly as the real path warms up.
  {
    const auto Y4 = Yref.leftCols<4>();
    A = 0.999 * A + Y4.transpose() * Y4;
    b = 0.999 * b + Y4.transpose() * r;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 4, 4>> warm(A);
    (void)warm.eigenvalues();
  }

  bool violated = false;
  {
    const rtc::testing::ScopedNoMalloc eigen_gate;
    for (int i = 0; i < 200; ++i) {
      const auto Y4 = Yref.leftCols<4>();
      A = 0.999 * A + Y4.transpose() * Y4;
      b = 0.999 * b + Y4.transpose() * r;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 4, 4>> eig(A);
      const double s_min = eig.eigenvalues().minCoeff();
      (void)s_min;
      const Eigen::LDLT<Eigen::Matrix<double, 4, 4>> ldlt(A);
      phi = ldlt.solve(b);
      fit.head(kNv).noalias() = Y4 * phi;
    }
    violated = eigen_gate.violations() > 0;
  }
  EXPECT_FALSE(violated) << "an Eigen expression on the RT path allocated";
}

TEST(InertialEstimatorRtSafety, UpdateIsAllocationFree) {
  InertialEstimator est;
  InertialEstimator::Config c;
  c.forgetting_factor = 0.999;
  c.min_param_sigma = 1e-9;
  c.min_mass = 1e-3;
  c.max_com_offset = 1.0;
  c.max_inertia_column = 1e-6;
  est.Init(kNv, c);  // non-RT, allowed to allocate

  const Eigen::MatrixXd Y1 = RegressorFor({0.0, 0.0, -9.81});
  const Eigen::MatrixXd Y2 = RegressorFor({0.0, -9.81, 0.0});
  const std::vector<double> r(static_cast<std::size_t>(kNv), 0.3);

  // Warm up outside the gates: the first pass through a decomposition may size
  // internals, and that one-time cost is not what RT-1 is about.
  est.Update(Y1, r);
  est.Update(Y2, r);

  std::size_t heap_allocs = 0;
  {
    const rtc::testing::ScopedAllocGate new_gate;
    const rtc::testing::ScopedNoMalloc eigen_gate;

    for (int i = 0; i < 200; ++i) {
      est.Update(Y1, r);
      est.Update(Y2, r);
    }
    // The reject paths run under the gate too — an early return that allocates
    // on its way out is still an RT violation.
    est.Hold(rtc::estimation::InertialInvalidReason::kUpstreamInvalid);
    est.ResetRtState();

    heap_allocs = new_gate.count();
  }

  EXPECT_EQ(heap_allocs, 0U) << "InertialEstimator::Update allocated on the RT path";
}
