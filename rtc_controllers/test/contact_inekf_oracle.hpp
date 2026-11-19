// ── Independent long-double oracles for the contact InEKF core (test-only) ───
//
// Nothing here calls the core. Each oracle reaches the same quantity by a
// different road:
//
//   MeanOracle   — integrates R(t) = R·exp([ω̄]× t) by 32-point Gauss–Legendre
//                  (nodes from Newton on P₃₂ in long double), not Γ functions.
//   GammaOracle  — 40-term matrix power series Σ W^k/(k+m)!, not the scalar
//                  coefficient series the core evaluates.
//   VanLoan      — exp of the 2D×2D Van Loan block matrix of the LEFT-invariant
//                  linear system, mapped to the right-invariant error by the
//                  adjoints: Φ = Ad(X̂⁺) Φ_L Ad(X̂)⁻¹, Q_d = Ad(X̂⁺) Q_L Ad(X̂⁺)ᵀ.
//                  Uses Eigen's general matrix exponential in long double.
//
// Layout of every matrix here is the PACKED active layout
// [φ, v, p, d_0 … d_{k−1}, b_g, b_a] (k = active slot count).
#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wpedantic"
#include <unsupported/Eigen/MatrixFunctions>
#pragma GCC diagnostic pop

#include <array>
#include <cmath>
#include <vector>

namespace rtc::estimation::oracle {

using LD = long double;
using MatL = Eigen::Matrix<LD, Eigen::Dynamic, Eigen::Dynamic>;
using Mat3L = Eigen::Matrix<LD, 3, 3>;
using Vec3L = Eigen::Matrix<LD, 3, 1>;

inline Mat3L HatL(const Vec3L& w) {
  Mat3L W;
  W << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
  return W;
}

/// Σ_{k<terms} W^k/(k+m)!  as a matrix series.
inline Mat3L GammaOracle(int m, const Vec3L& phi, int terms = 40) {
  const Mat3L W = HatL(phi);
  Mat3L sum = Mat3L::Zero();
  Mat3L Wk = Mat3L::Identity();
  LD fact = 1;
  for (int i = 2; i <= m; ++i) {
    fact *= static_cast<LD>(i);
  }
  for (int k = 0; k < terms; ++k) {
    sum += Wk / fact;
    Wk = Wk * W;
    fact *= static_cast<LD>(k + 1 + m);
  }
  return sum;
}

struct GaussLegendre32 {
  std::array<LD, 32> node{};    // on [0, 1]
  std::array<LD, 32> weight{};  // sums to 1
};

inline const GaussLegendre32& Gl32() {
  static const GaussLegendre32 rule = [] {
    GaussLegendre32 r;
    constexpr int n = 32;
    const LD pi = 3.14159265358979323846264338327950288L;
    // P₃₂(x) and its derivative by the three-term recurrence.
    auto legendre = [n](LD x) {
      LD p0 = 1;
      LD p1 = x;
      for (int m = 2; m <= n; ++m) {
        const LD p2 = ((2 * m - 1) * x * p1 - static_cast<LD>(m - 1) * p0) / static_cast<LD>(m);
        p0 = p1;
        p1 = p2;
      }
      return std::array<LD, 2>{p1, static_cast<LD>(n) * (x * p1 - p0) / (x * x - 1)};
    };
    for (int k = 1; k <= n; ++k) {
      LD x = std::cos(pi * (static_cast<LD>(k) - 0.25L) / (static_cast<LD>(n) + 0.5L));
      for (int it = 0; it < 100; ++it) {
        const std::array<LD, 2> pd = legendre(x);
        const LD dx = pd[0] / pd[1];
        x -= dx;
        if (std::abs(dx) < 1e-21L) {
          break;
        }
      }
      const LD dp = legendre(x)[1];
      const auto i = static_cast<std::size_t>(k - 1);
      r.node[i] = (x + 1) / 2;
      r.weight[i] = 1 / ((1 - x * x) * dp * dp);  // (2/((1−x²)P′²)) / 2
    }
    return r;
  }();
  return rule;
}

struct MeanL {
  Mat3L R;
  Vec3L v;
  Vec3L p;
};

/// Exact ZOH mean by quadrature of the rotating specific force.
inline MeanL MeanOracle(const Mat3L& R, const Vec3L& v, const Vec3L& p, const Vec3L& w,
                        const Vec3L& a, LD dt, LD g0) {
  const GaussLegendre32& gl = Gl32();
  Vec3L iv = Vec3L::Zero();
  Vec3L ip = Vec3L::Zero();
  for (std::size_t k = 0; k < gl.node.size(); ++k) {
    const LD t = gl.node[k] * dt;
    const Vec3L f = R * (GammaOracle(0, w * t) * a);
    iv += (gl.weight[k] * dt) * f;
    ip += (gl.weight[k] * dt * (dt - t)) * f;
  }
  const Vec3L g(0, 0, -g0);
  MeanL out;
  out.R = R * GammaOracle(0, w * dt);
  out.v = v + iv + g * dt;
  out.p = p + v * dt + ip + g * (dt * dt / 2);
  return out;
}

/// Left-invariant continuous error matrix (packed layout, k slots).
inline MatL LeftErrorMatrix(const Vec3L& w, const Vec3L& a, int k) {
  const int D = 15 + 3 * k;
  const int bg = 9 + 3 * k;
  MatL A = MatL::Zero(D, D);
  const Mat3L W = HatL(w);
  A.block<3, 3>(0, 0) = -W;
  A.block<3, 3>(0, bg) = -Mat3L::Identity();
  A.block<3, 3>(3, 0) = -HatL(a);
  A.block<3, 3>(3, 3) = -W;
  A.block<3, 3>(3, bg + 3) = -Mat3L::Identity();
  A.block<3, 3>(6, 3) = Mat3L::Identity();
  A.block<3, 3>(6, 6) = -W;
  for (int i = 0; i < k; ++i) {
    A.block<3, 3>(9 + 3 * i, 9 + 3 * i) = -W;
  }
  return A;
}

struct Discrete {
  MatL Phi;
  MatL Qd;
};

/// Van Loan on the left-invariant system; qc is the diagonal of Q_c.
inline Discrete VanLoanLeft(const Vec3L& w, const Vec3L& a, LD dt, const std::vector<LD>& qc,
                            int k) {
  const MatL A = LeftErrorMatrix(w, a, k);
  const int D = static_cast<int>(A.rows());
  MatL M = MatL::Zero(2 * D, 2 * D);
  M.topLeftCorner(D, D) = -A * dt;
  for (int i = 0; i < D; ++i) {
    M(i, D + i) = qc[static_cast<std::size_t>(i)] * dt;
  }
  M.bottomRightCorner(D, D) = A.transpose() * dt;
  const MatL E = M.exp();
  Discrete out;
  out.Phi = E.bottomRightCorner(D, D).transpose();
  const MatL Qd = out.Phi * E.topRightCorner(D, D);
  out.Qd = (Qd + Qd.transpose()) / 2;
  return out;
}

/// Ad(X) on (R, cols = [v, p, d…]) extended by I on the bias.
inline MatL AdjointL(const Mat3L& R, const std::vector<Vec3L>& cols, bool inverse) {
  const int k = static_cast<int>(cols.size());
  const int D = 3 + 3 * k + 6;
  MatL A = MatL::Zero(D, D);
  if (!inverse) {
    A.block<3, 3>(0, 0) = R;
    for (int j = 0; j < k; ++j) {
      const int o = 3 + 3 * j;
      A.block<3, 3>(o, 0) = HatL(cols[static_cast<std::size_t>(j)]) * R;
      A.block<3, 3>(o, o) = R;
    }
  } else {
    A.block<3, 3>(0, 0) = R.transpose();
    for (int j = 0; j < k; ++j) {
      const int o = 3 + 3 * j;
      A.block<3, 3>(o, 0) = -R.transpose() * HatL(cols[static_cast<std::size_t>(j)]);
      A.block<3, 3>(o, o) = R.transpose();
    }
  }
  A.block<6, 6>(D - 6, D - 6).setIdentity();
  return A;
}

/// Right-invariant Φ, Q_d from the left-invariant pair and the two means.
inline Discrete ToRightInvariant(const Discrete& left, const Mat3L& R0,
                                 const std::vector<Vec3L>& cols0, const Mat3L& R1,
                                 const std::vector<Vec3L>& cols1) {
  const MatL Ad1 = AdjointL(R1, cols1, false);
  const MatL Ad0i = AdjointL(R0, cols0, true);
  Discrete out;
  out.Phi = Ad1 * left.Phi * Ad0i;
  const MatL Qd = Ad1 * left.Qd * Ad1.transpose();
  out.Qd = (Qd + Qd.transpose()) / 2;
  return out;
}

/// max |eig(L⁻¹ (Q̂ − Q) L⁻ᵀ)|, L = chol(Q), in long double.
inline double Whitened(const MatL& q_hat, const MatL& q_ref) {
  const Eigen::LLT<MatL> llt(q_ref);
  if (llt.info() != Eigen::Success) {
    return std::nan("");
  }
  const MatL dq = q_hat - q_ref;
  const MatL Y = llt.matrixL().solve(dq);
  MatL Z = llt.matrixL().solve(Y.transpose()).transpose();
  Z = (Z + Z.transpose()) / 2;
  const Eigen::SelfAdjointEigenSolver<MatL> es(Z, Eigen::EigenvaluesOnly);
  return static_cast<double>(es.eigenvalues().cwiseAbs().maxCoeff());
}

}  // namespace rtc::estimation::oracle
