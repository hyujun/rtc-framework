// Layer 1 of #135 — the DYNAMIC half of the momentum observer, driven by a real
// model in motion.
//
// WHY THIS FILE EXISTS. test_momentum_observer.cpp states it in its own header:
// every case there runs at q̇ == 0, so `p = M(q) q̇` and `Cᵀ(q,q̇) q̇` are
// identically zero and the arithmetic never touches M or C. The 2026-08-29 sim
// null test had the same blind spot for the same reason (max|q̇| ~ 1e-11 rad/s).
// So the two terms that #135 comment#3 §4.1 called the most likely
// implementation bug — and predicted would pass exactly this kind of
// quasi-static acceptance while being "조용히 틀린" under motion — had no sensor
// at all.
//
// THE ORACLE IS MODEL SELF-CONSISTENCY, NOT A SIMULATOR. Ground truth here is
// generated from the same Pinocchio model the observer's inputs are assembled
// from: tau_m := RNEA(q, q̇, q̈) is by construction the torque an arm with NO
// external force reports. [MO-1] then forces tau_ext == 0, so the residual must
// vanish. Nothing outside the model can shift the answer — which is the point:
// driving this from MuJoCo instead would make the verdict depend on MJCF↔URDF
// agreement, a fixture property that has nothing to do with whether the
// observer is correct.
//
// WHY dt-REFINEMENT RATHER THAN ONE TOLERANCE. [MO-3b] is backward Euler over a
// continuous identity, so under motion the residual carries an O(dt)
// discretization term that no correct implementation can remove. A single
// threshold would therefore have to be loose enough to hide a real bias. The
// discriminating signature is the SLOPE: a correct assembly's residual → 0 as
// dt → 0, while a mis-assembled one (C q̇ for Cᵀ q̇, or h − g) converges to a
// nonzero function of q̇ and stays there. MisassembledCoriolisDoesNotVanish is
// the power check that this file can actually tell those two apart.
#include "rtc_controllers/estimation/momentum_observer.hpp"
#include "test_urdf_path.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

using rtc::estimation::MomentumObserver;

// serial_7dof: alternating Z/Y axes, so g(q) is not identically zero and the
// Coriolis coupling between links is genuinely populated. serial_6dof is all-Z
// (g ≡ 0) and would make half of [MO-3a] inert.
pinocchio::Model MakeModel() {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(rtc::test::TestUrdfPath("serial_7dof.urdf"), model);
  return model;
}

// A smooth, genuinely moving trajectory. Distinct frequency and phase per joint
// so no two joints stay in lockstep — a Coriolis error that happened to be
// symmetric across joints would otherwise cancel in the norm.
struct Traj {
  Eigen::VectorXd q, v, a;
};

Traj Sample(const pinocchio::Model& model, double t) {
  const int n = model.nv;
  Traj s{Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n)};
  for (int i = 0; i < n; ++i) {
    const double amp = 0.45;
    const double w = 3.0 + 0.7 * static_cast<double>(i);  // rad/s, all distinct
    const double ph = 0.4 * static_cast<double>(i);
    s.q[i] = amp * std::sin(w * t + ph);
    s.v[i] = amp * w * std::cos(w * t + ph);
    s.a[i] = -amp * w * w * std::sin(w * t + ph);
  }
  return s;
}

// crba fills the UPPER triangle only (#135 comment#3 §4.6). Reading data.M
// without this mirror silently yields a triangular "mass matrix" and a momentum
// that is wrong in a way no assertion here would attribute correctly.
Eigen::MatrixXd MassMatrix(const pinocchio::Model& model, pinocchio::Data& data,
                           const Eigen::VectorXd& q) {
  pinocchio::crba(model, data, q);
  Eigen::MatrixXd M = data.M;
  M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
  return M;
}

enum class Assembly { kCorrect, kCoriolisTransposed };

// Run the observer along the trajectory and return max|r| over the final
// quarter of the run (the filter has settled by then: K_I * T >> 1).
//
// `tau_ext` is the constant external torque the virtual arm is under. The
// measured torque a real arm would report is RNEA (which excludes external
// forces) minus that torque, matching [MO-1]'s sign convention the same way
// test_momentum_observer.cpp's MeasuredTorque() does at rest.
double PeakSettledResidual(double dt, double duration, const Eigen::VectorXd& tau_ext,
                           Assembly mode = Assembly::kCorrect) {
  const pinocchio::Model model = MakeModel();
  pinocchio::Data data(model);
  const int n = model.nv;

  MomentumObserver obs;
  obs.Init(n, std::vector<double>(static_cast<std::size_t>(n), 20.0));

  const int steps = static_cast<int>(duration / dt);
  const int settle_from = (steps * 3) / 4;
  double peak = 0.0;

  std::vector<double> p(static_cast<std::size_t>(n));
  std::vector<double> ctv(static_cast<std::size_t>(n));
  std::vector<double> g(static_cast<std::size_t>(n));
  std::vector<double> tau(static_cast<std::size_t>(n));

  for (int k = 0; k < steps; ++k) {
    const Traj s = Sample(model, static_cast<double>(k) * dt);

    const Eigen::MatrixXd M = MassMatrix(model, data, s.q);
    pinocchio::computeCoriolisMatrix(model, data, s.q, s.v);
    const Eigen::MatrixXd C = data.C;
    pinocchio::computeGeneralizedGravity(model, data, s.q);
    const Eigen::VectorXd gv = data.g;
    // RNEA carries no external wrench, so this is exactly the torque of an
    // unloaded arm; subtracting tau_ext states the load we want observed.
    const Eigen::VectorXd tau_m = pinocchio::rnea(model, data, s.q, s.v, s.a) - tau_ext;

    const Eigen::VectorXd pv = M * s.v;
    const Eigen::VectorXd cv =
        (mode == Assembly::kCorrect) ? (C.transpose() * s.v).eval() : (C * s.v).eval();

    for (int i = 0; i < n; ++i) {
      p[static_cast<std::size_t>(i)] = pv[i];
      ctv[static_cast<std::size_t>(i)] = cv[i];
      g[static_cast<std::size_t>(i)] = gv[i];
      tau[static_cast<std::size_t>(i)] = tau_m[i];
    }
    obs.Update(p, ctv, g, tau, dt);

    if (k >= settle_from) {
      const auto r = obs.residual();
      for (int i = 0; i < n; ++i) {
        peak = std::max(peak, std::fabs(r[static_cast<std::size_t>(i)] - tau_ext[i]));
      }
    }
  }
  return peak;
}

}  // namespace

// ── The null case, but IN MOTION — what q̇ == 0 could never test ─────────────

// An unloaded arm swinging through a populated Coriolis field must still report
// nothing. Every term of [MO-3a] is live here; at q̇ == 0 four of them are not.
TEST(MomentumObserverModelOracle, UnloadedArmInMotionKeepsResidualNearZero) {
  const pinocchio::Model model = MakeModel();
  const double peak = PeakSettledResidual(2.5e-4, 2.0, Eigen::VectorXd::Zero(model.nv));

  // Scale reference: peak |tau_m| along this trajectory is O(10) N·m, so this
  // bounds the residual at well under a percent of the signal it rides on.
  EXPECT_LT(peak, 0.05) << "unloaded arm in motion reported a phantom external torque";
}

// The discriminating test. O(dt) is what a correct backward-Euler assembly
// leaves behind; a mis-assembled one leaves a dt-independent floor.
TEST(MomentumObserverModelOracle, ResidualIsFirstOrderInDtAndVanishes) {
  const pinocchio::Model model = MakeModel();
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(model.nv);

  const double coarse = PeakSettledResidual(1.0e-3, 2.0, zero);
  const double mid = PeakSettledResidual(5.0e-4, 2.0, zero);
  const double fine = PeakSettledResidual(2.5e-4, 2.0, zero);

  // Printed rather than only asserted: the slope IS the finding, and a future
  // reader deciding whether this suite still has power needs the numbers, not a
  // green tick.
  std::cout << "  [dt refinement] max|r| : dt=1.0e-3 -> " << coarse << " ,  dt=5.0e-4 -> " << mid
            << " ,  dt=2.5e-4 -> " << fine << "  (ratios " << coarse / mid << ", " << mid / fine
            << ")\n";

  EXPECT_LT(mid, coarse) << "refining dt did not reduce the residual";
  EXPECT_LT(fine, mid) << "refining dt did not reduce the residual";

  // Halving dt should roughly halve the error. Bracketed rather than pinned:
  // below ~1.6 would mean the convergence is slower than first order (a real
  // defect), above ~2.4 would mean it is being driven by something other than
  // the discretization this models.
  const double r1 = coarse / mid;
  const double r2 = mid / fine;
  EXPECT_GT(r1, 1.6) << "convergence slower than first order: ratio " << r1;
  EXPECT_LT(r1, 2.4) << "ratio " << r1;
  EXPECT_GT(r2, 1.6) << "convergence slower than first order: ratio " << r2;
  EXPECT_LT(r2, 2.4) << "ratio " << r2;
}

// ── Positive control ─────────────────────────────────────────────────────────

// The null case above passes for a do-nothing observer too. This one does not:
// the residual must REPRODUCE a known constant external torque while the arm is
// moving, which is the property Layer 2A actually consumes.
TEST(MomentumObserverModelOracle, ConstantExternalTorqueIsRecoveredUnderMotion) {
  const pinocchio::Model model = MakeModel();
  Eigen::VectorXd tau_ext(model.nv);
  tau_ext << 3.0, -5.0, 1.5, 0.25, -0.75, 2.0, -1.25;

  const double peak = PeakSettledResidual(2.5e-4, 2.0, tau_ext);
  EXPECT_LT(peak, 0.05) << "external torque not recovered under motion";
}

// ── Power check ──────────────────────────────────────────────────────────────

// Proves the suite above can actually see the #135 comment#3 §4.1 bug. Feeding
// C q̇ where Cᵀ q̇ belongs is the exact mutation that section named, and it is
// invisible at q̇ == 0 — both forms are zero there. If this test ever goes
// green, the cases above have stopped measuring the Coriolis lane.
TEST(MomentumObserverModelOracle, MisassembledCoriolisDoesNotVanish) {
  const pinocchio::Model model = MakeModel();
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(model.nv);

  const double mutated_coarse =
      PeakSettledResidual(1.0e-3, 2.0, zero, Assembly::kCoriolisTransposed);
  const double mutated_fine = PeakSettledResidual(2.5e-4, 2.0, zero, Assembly::kCoriolisTransposed);

  std::cout << "  [power check] C q̇ mutation max|r| : dt=1.0e-3 -> " << mutated_coarse
            << " ,  dt=2.5e-4 -> " << mutated_fine << "  (correct assembly is ~0.01 here)\n";

  EXPECT_GT(mutated_fine, 0.5) << "the C q̇ mutation produced no residual — the correct-assembly "
                                  "cases are not measuring the Coriolis term";
  // The signature that separates a bug from discretization: refining dt does
  // NOT drive this away, because the error is in the integrand, not the scheme.
  EXPECT_GT(mutated_fine, 0.5 * mutated_coarse)
      << "mutated residual shrank with dt like a discretization term would";
}

// ── The identity [MO-3a] is derived from (#135 comment#3 §4.5) ───────────────

// Ṁ = C + Cᵀ is load-bearing for the whole Layer 1 derivation, and the installed
// Pinocchio header documents only c = C(q,q̇)q̇ — the skew-symmetry property is
// nowhere in it. If Pinocchio ever returned a different valid factorization of
// the Coriolis forces, [MO-3a] would silently stop holding. Central difference
// along the trajectory direction, since Ṁ = Σ ∂M/∂q_i q̇_i.
TEST(MomentumObserverModelOracle, MassMatrixDerivativeEqualsCPlusCTranspose) {
  const pinocchio::Model model = MakeModel();
  pinocchio::Data data(model);
  const Traj s = Sample(model, 0.37);  // arbitrary non-degenerate point on the path

  const double h = 1e-6;
  const Eigen::MatrixXd M_plus = MassMatrix(model, data, (s.q + h * s.v).eval());
  const Eigen::MatrixXd M_minus = MassMatrix(model, data, (s.q - h * s.v).eval());
  const Eigen::MatrixXd M_dot_fd = (M_plus - M_minus) / (2.0 * h);

  pinocchio::computeCoriolisMatrix(model, data, s.q, s.v);
  const Eigen::MatrixXd C = data.C;

  EXPECT_LT((M_dot_fd - (C + C.transpose())).cwiseAbs().maxCoeff(), 1e-5)
      << "Pinocchio's Coriolis factorization does not satisfy Ṁ = C + Cᵀ — [MO-3a] does not hold";
}
