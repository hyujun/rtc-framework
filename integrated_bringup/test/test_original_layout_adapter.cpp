// Original (wb_humanoid_mpc / OCS2) ↔ pinocchio free-flyer layout adapter
// (W1-F03, #1026).
//
// Every mistake this adapter can make is silent — a reversed Euler triple, a
// world twist left unrotated, Euler rates passed off as an angular velocity,
// a swapped yaw / roll weight all produce finite, plausible numbers. So each
// direction is held against an oracle that does not share the adapter's code:
//   - weights: an explicit literal index table (not the permutation rule
//     re-derived), one-hot over all 58 entries plus a non-symmetric input;
//   - rotation: pinocchio::rpy::rpyToMatrix, not the rtc_math helper the
//     adapter builds R with;
//   - twist: a pinocchio free-flyer model's own forward kinematics for the
//     linear part, and a central finite difference of R(t) along the Euler
//     trajectory for the angular part — not computeRpyJacobian;
//   - the weight approximation: the exact frame-transformed diagonal
//     Jᵀ W_o J, which must agree upright AND visibly disagree at roll 0.3 rad
//     (the pair proves the comparison can see the approximation at all).
// Tolerances are pre-registered: 1e-12 where only double round-off separates
// the two sides (~1e-15), 1e-6 against the h = 1e-6 central difference
// (truncation ~h² · |R⃛| ≈ 1e-12, round-off ~eps/h ≈ 1e-10).

#include "integrated_bringup/controllers/locomotion/original_layout_adapter.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/math.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <random>

namespace {

using integrated_bringup::locomotion::FreeFlyerToOriginal;
using integrated_bringup::locomotion::kGimbalMargin;
using integrated_bringup::locomotion::OriginalToFreeFlyer;
using integrated_bringup::locomotion::OriginalWeightsToTangent;
using integrated_bringup::locomotion::WorldFromBase;

constexpr double kTol = 1e-12;
constexpr double kFdTol = 1e-6;
constexpr double kFdStep = 1e-6;
constexpr double kPi = std::numbers::pi;

// The joint count of the weight table below; the state tests use it too.
constexpr int kNj = 23;
constexpr int kHalf = 6 + kNj;  // = nv
constexpr int kNx = 2 * kHalf;  // 58

// Original-layout indices of the Euler triple (ψ yaw, θ pitch, φ roll).
constexpr int kPsi = 3;
constexpr int kTheta = 4;
constexpr int kPhi = 5;

struct OriginalState {
  Eigen::VectorXd q_o;
  Eigen::VectorXd v_o;
};

OriginalState MakeOriginal(double psi, double theta, double phi, double psi_dot, double theta_dot,
                           double phi_dot, int nj) {
  OriginalState s{Eigen::VectorXd::Zero(6 + nj), Eigen::VectorXd::Zero(6 + nj)};
  s.q_o.head<3>() << 0.3, -1.2, 0.78;
  s.q_o[kPsi] = psi;
  s.q_o[kTheta] = theta;
  s.q_o[kPhi] = phi;
  s.v_o.head<3>() << 0.4, -0.25, 0.15;
  s.v_o[kPsi] = psi_dot;
  s.v_o[kTheta] = theta_dot;
  s.v_o[kPhi] = phi_dot;
  for (int j = 0; j < nj; ++j) {
    s.q_o[6 + j] = 0.01 * (j + 1);
    s.v_o[6 + j] = -0.02 * (j + 1);
  }
  return s;
}

OriginalState RandomOriginal(std::mt19937& rng, int nj, double max_pitch) {
  std::uniform_real_distribution<double> angle(-kPi, kPi);
  std::uniform_real_distribution<double> pitch(-max_pitch, max_pitch);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  OriginalState s{Eigen::VectorXd(6 + nj), Eigen::VectorXd(6 + nj)};
  for (int i = 0; i < 3; ++i) {
    s.q_o[i] = 2.0 * unit(rng);
    s.v_o[i] = 2.0 * unit(rng);
  }
  s.q_o[kPsi] = angle(rng);
  s.q_o[kTheta] = pitch(rng);
  s.q_o[kPhi] = angle(rng);
  for (int i = 3; i < 6; ++i) {
    s.v_o[i] = 3.0 * unit(rng);
  }
  for (int j = 0; j < nj; ++j) {
    s.q_o[6 + j] = 2.5 * unit(rng);
    s.v_o[6 + j] = 4.0 * unit(rng);
  }
  return s;
}

// Independent oracle: pinocchio's rpy → R with the original triple reversed.
Eigen::Matrix3d OracleRotation(double psi, double theta, double phi) {
  return pinocchio::rpy::rpyToMatrix(phi, theta, psi);
}

Eigen::Matrix3d RotationOfQ(const Eigen::VectorXd& q) {
  const Eigen::Quaterniond quat(q[6], q[3], q[4], q[5]);  // (w, x, y, z)
  return quat.toRotationMatrix();
}

double WrappedDiff(double a, double b) {
  const double d = a - b;
  return std::abs(std::atan2(std::sin(d), std::cos(d)));
}

Eigen::Vector3d Vee(const Eigen::Matrix3d& S) {
  return 0.5 * Eigen::Vector3d(S(2, 1) - S(1, 2), S(0, 2) - S(2, 0), S(1, 0) - S(0, 1));
}

// ── Weights: one-hot against an explicit table ───────────────────────────────

// Tangent index of a one-hot at original index i, nj = 23, written out by
// hand. Not computed: a rule shared with the adapter would cancel its own bug.
// Row 1: p_base_{x,y,z}, theta_base_{z,y,x} → δθ_{z,y,x} = 5, 4, 3, joints.
// Row 2: v_base_{x,y,z}, omega_base_{z,y,x} → δω_{z,y,x} = 34, 33, 32, joints.
constexpr std::array<int, kNx> kExpectedTangentIndex = {
    0,  1,  2,  5,  4,  3,  6,  7,  8,  9,  10, 11, 12, 13, 14,
    15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,  //
    29, 30, 31, 34, 33, 32, 35, 36, 37, 38, 39, 40, 41, 42, 43,
    44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
};

TEST(OriginalLayoutAdapter, ExpectedTableIsAPermutation) {
  std::array<int, kNx> hits{};
  for (const int k : kExpectedTangentIndex) {
    ASSERT_GE(k, 0);
    ASSERT_LT(k, kNx);
    ++hits[static_cast<std::size_t>(k)];
  }
  for (const int h : hits) {
    EXPECT_EQ(h, 1);
  }
}

TEST(OriginalLayoutAdapter, WeightOneHotLandsOnTheTableIndex) {
  for (int i = 0; i < kNx; ++i) {
    Eigen::VectorXd w_o = Eigen::VectorXd::Zero(kNx);
    w_o[i] = 1.0;
    Eigen::VectorXd w = Eigen::VectorXd::Constant(kNx, -1.0);
    ASSERT_TRUE(OriginalWeightsToTangent(w_o, w)) << "i = " << i;
    const int expected = kExpectedTangentIndex[static_cast<std::size_t>(i)];
    for (int k = 0; k < kNx; ++k) {
      EXPECT_EQ(w[k], k == expected ? 1.0 : 0.0) << "one-hot at " << i << ", tangent " << k;
    }
  }
}

TEST(OriginalLayoutAdapter, WeightNonSymmetricInputFollowsTheTable) {
  Eigen::VectorXd w_o(kNx);
  for (int i = 0; i < kNx; ++i) {
    w_o[i] = i + 1.0;
  }
  Eigen::VectorXd w(kNx);
  ASSERT_TRUE(OriginalWeightsToTangent(w_o, w));
  for (int i = 0; i < kNx; ++i) {
    EXPECT_EQ(w[kExpectedTangentIndex[static_cast<std::size_t>(i)]], i + 1.0) << "i = " << i;
  }
  // The four moved slots, by value: δθ_x ← theta_base_x (6), δθ_z ← theta_base_z (4).
  EXPECT_EQ(w[3], 6.0);
  EXPECT_EQ(w[4], 5.0);
  EXPECT_EQ(w[5], 4.0);
  EXPECT_EQ(w[32], 35.0);
  EXPECT_EQ(w[34], 33.0);

  // In place (one buffer) gives the same answer.
  Eigen::VectorXd in_place = w_o;
  ASSERT_TRUE(OriginalWeightsToTangent(in_place, in_place));
  EXPECT_TRUE(in_place == w);
}

// ── Forward: rotation and twist against independent oracles ──────────────────

TEST(OriginalLayoutAdapter, ForwardQuaternionMatchesPinocchioRpyToMatrix) {
  std::mt19937 rng(20261112u);
  for (int n = 0; n < 200; ++n) {
    const OriginalState s = RandomOriginal(rng, kNj, kPi / 2.0 - 0.05);
    Eigen::VectorXd q(7 + kNj);
    Eigen::VectorXd v(6 + kNj);
    ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));
    const Eigen::Matrix3d R_oracle = OracleRotation(s.q_o[kPsi], s.q_o[kTheta], s.q_o[kPhi]);
    EXPECT_LE((RotationOfQ(q) - R_oracle).cwiseAbs().maxCoeff(), kTol) << "sample " << n;
    EXPECT_LE(std::abs(q.segment<4>(3).norm() - 1.0), kTol) << "sample " << n;
    EXPECT_TRUE(q.head<3>() == s.q_o.head<3>());
    EXPECT_TRUE(q.tail(kNj) == s.q_o.tail(kNj));
    const Eigen::Vector3d v_b_oracle = R_oracle.transpose() * s.v_o.head<3>();
    EXPECT_LE((v.head<3>() - v_b_oracle).cwiseAbs().maxCoeff(), kTol) << "sample " << n;
    EXPECT_TRUE(v.tail(kNj) == s.v_o.tail(kNj));
  }
}

TEST(OriginalLayoutAdapter, RoundTripOriginalFreeFlyerOriginal) {
  std::mt19937 rng(1026u);
  const double max_pitch = 80.0 * kPi / 180.0;
  for (int n = 0; n < 200; ++n) {
    const OriginalState s = RandomOriginal(rng, kNj, max_pitch);
    Eigen::VectorXd q(7 + kNj);
    Eigen::VectorXd v(6 + kNj);
    ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v)) << "sample " << n;
    Eigen::VectorXd q_o(6 + kNj);
    Eigen::VectorXd v_o(6 + kNj);
    ASSERT_TRUE(FreeFlyerToOriginal(q, v, q_o, v_o)) << "sample " << n;

    EXPECT_LE((q_o.head<3>() - s.q_o.head<3>()).cwiseAbs().maxCoeff(), kTol) << "sample " << n;
    // Roll and yaw are drawn over (−π, π): compare on the circle.
    EXPECT_LE(WrappedDiff(q_o[kPsi], s.q_o[kPsi]), kTol) << "sample " << n;
    EXPECT_LE(std::abs(q_o[kTheta] - s.q_o[kTheta]), kTol) << "sample " << n;
    EXPECT_LE(WrappedDiff(q_o[kPhi], s.q_o[kPhi]), kTol) << "sample " << n;
    EXPECT_LE((q_o.tail(kNj) - s.q_o.tail(kNj)).cwiseAbs().maxCoeff(), kTol);
    EXPECT_LE((v_o - s.v_o).cwiseAbs().maxCoeff(), kTol) << "sample " << n;
  }
}

// ── Rejections, each beside a passing control on the same rig ────────────────

TEST(OriginalLayoutAdapter, SizeMismatchIsRejectedWithoutWriting) {
  const OriginalState s = MakeOriginal(0.4, 0.2, -0.3, 0.1, 0.2, 0.3, kNj);
  Eigen::VectorXd q = Eigen::VectorXd::Constant(7 + kNj, 9.0);
  Eigen::VectorXd v = Eigen::VectorXd::Constant(6 + kNj, 9.0);
  const Eigen::VectorXd v_o_long = Eigen::VectorXd::Zero(7 + kNj);
  EXPECT_FALSE(OriginalToFreeFlyer(s.q_o, v_o_long, q, v));
  EXPECT_TRUE((q.array() == 9.0).all());
  EXPECT_TRUE((v.array() == 9.0).all());
  Eigen::VectorXd q_short(6 + kNj);
  EXPECT_FALSE(OriginalToFreeFlyer(s.q_o, s.v_o, q_short, v));
  EXPECT_FALSE(OriginalToFreeFlyer(s.q_o.head(5), s.v_o.head(5), q.head(6), v.head(5)));
  ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));  // control

  Eigen::VectorXd q_o = Eigen::VectorXd::Constant(6 + kNj, 9.0);
  Eigen::VectorXd v_o = Eigen::VectorXd::Constant(6 + kNj, 9.0);
  EXPECT_FALSE(FreeFlyerToOriginal(q, v.head(5 + kNj), q_o, v_o));
  EXPECT_FALSE(FreeFlyerToOriginal(q, v, q_o.head(5 + kNj), v_o));
  EXPECT_TRUE((q_o.array() == 9.0).all());
  EXPECT_TRUE((v_o.array() == 9.0).all());
  EXPECT_TRUE(FreeFlyerToOriginal(q, v, q_o, v_o));  // control

  const Eigen::VectorXd w_o = Eigen::VectorXd::Ones(kNx);
  Eigen::VectorXd w = Eigen::VectorXd::Constant(kNx, 9.0);
  Eigen::VectorXd w_short = Eigen::VectorXd::Constant(kNx - 2, 9.0);
  EXPECT_FALSE(OriginalWeightsToTangent(w_o, w_short));
  EXPECT_FALSE(OriginalWeightsToTangent(w_o.head(kNx - 1), w.head(kNx - 1)));  // odd
  EXPECT_FALSE(OriginalWeightsToTangent(w_o.head(10), w.head(10)));            // < 2 · 6
  EXPECT_TRUE((w.array() == 9.0).all());
  EXPECT_TRUE(OriginalWeightsToTangent(w_o.head(12), w.head(12)));  // control, nj = 0
  EXPECT_TRUE(OriginalWeightsToTangent(w_o, w));                    // control
}

TEST(OriginalLayoutAdapter, NonFiniteOriginalStateIsRejected) {
  OriginalState s = MakeOriginal(0.4, 0.2, -0.3, 0.1, 0.2, 0.3, kNj);
  Eigen::VectorXd q(7 + kNj);
  Eigen::VectorXd v(6 + kNj);
  ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));  // control

  s.q_o[kTheta] = std::numeric_limits<double>::quiet_NaN();
  q.setConstant(9.0);
  EXPECT_FALSE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));
  EXPECT_TRUE((q.array() == 9.0).all());

  s.q_o[kTheta] = 0.2;
  s.v_o[6 + kNj - 1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));
}

TEST(OriginalLayoutAdapter, NonUnitQuaternionIsRejected) {
  const OriginalState s = MakeOriginal(0.4, 0.2, -0.3, 0.1, 0.2, 0.3, kNj);
  Eigen::VectorXd q(7 + kNj);
  Eigen::VectorXd v(6 + kNj);
  ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));
  Eigen::VectorXd q_o(6 + kNj);
  Eigen::VectorXd v_o(6 + kNj);
  ASSERT_TRUE(FreeFlyerToOriginal(q, v, q_o, v_o));  // control

  Eigen::VectorXd q_long = q;
  q_long.segment<4>(3) *= 1.01;  // ‖q‖² − 1 ≈ 0.02
  q_o.setConstant(9.0);
  EXPECT_FALSE(FreeFlyerToOriginal(q_long, v, q_o, v_o));
  EXPECT_TRUE((q_o.array() == 9.0).all());

  Eigen::VectorXd q_zero = q;
  q_zero.segment<4>(3).setZero();
  EXPECT_FALSE(FreeFlyerToOriginal(q_zero, v, q_o, v_o));

  Eigen::VectorXd q_nan = q;
  q_nan[5] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(FreeFlyerToOriginal(q_nan, v, q_o, v_o));
}

TEST(OriginalLayoutAdapter, GimbalPitchIsRejected) {
  const auto inverse_accepts = [](double pitch) {
    const OriginalState s = MakeOriginal(0.4, pitch, -0.3, 0.1, 0.2, 0.3, kNj);
    Eigen::VectorXd q(7 + kNj);
    Eigen::VectorXd v(6 + kNj);
    EXPECT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));  // forward is total
    Eigen::VectorXd q_o = Eigen::VectorXd::Constant(6 + kNj, 9.0);
    Eigen::VectorXd v_o = Eigen::VectorXd::Constant(6 + kNj, 9.0);
    const bool ok = FreeFlyerToOriginal(q, v, q_o, v_o);
    if (!ok) {
      EXPECT_TRUE((q_o.array() == 9.0).all());
      EXPECT_TRUE((v_o.array() == 9.0).all());
    }
    return ok;
  };
  const double edge = kPi / 2.0 - kGimbalMargin;
  EXPECT_TRUE(inverse_accepts(edge - kGimbalMargin));   // control, inside the margin
  EXPECT_TRUE(inverse_accepts(-edge + kGimbalMargin));  // control, negative side
  EXPECT_FALSE(inverse_accepts(edge + 0.5 * kGimbalMargin));
  EXPECT_FALSE(inverse_accepts(-edge - 0.5 * kGimbalMargin));
  EXPECT_FALSE(inverse_accepts(kPi / 2.0));
}

TEST(OriginalLayoutAdapter, NegativeOrNonFiniteWeightIsRejected) {
  Eigen::VectorXd w_o = Eigen::VectorXd::Ones(kNx);
  Eigen::VectorXd w = Eigen::VectorXd::Constant(kNx, 9.0);
  w_o[7] = 0.0;
  ASSERT_TRUE(OriginalWeightsToTangent(w_o, w));  // control: zero is a valid weight

  w.setConstant(9.0);
  w_o[7] = -1e-9;
  EXPECT_FALSE(OriginalWeightsToTangent(w_o, w));
  w_o[7] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(OriginalWeightsToTangent(w_o, w));
  w_o[7] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(OriginalWeightsToTangent(w_o, w));
  EXPECT_TRUE((w.array() == 9.0).all());
}

// ── Non-upright twist against a pinocchio free-flyer and a finite difference ─

TEST(OriginalLayoutAdapter, NonUprightTwistMatchesFreeFlyerKinematicsAndFiniteDifference) {
  pinocchio::Model model;
  const pinocchio::JointIndex root =
      model.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), "root_joint");
  model.appendBodyToJoint(root, pinocchio::Inertia::Identity(), pinocchio::SE3::Identity());
  pinocchio::Data data(model);
  ASSERT_EQ(model.nq, 7);
  ASSERT_EQ(model.nv, 6);

  // Roll, pitch, yaw all non-zero and mutually different; so are the rates.
  const double psi = 0.7;
  const double theta = -0.4;
  const double phi = 0.25;
  const double psi_dot = 0.3;
  const double theta_dot = -0.8;
  const double phi_dot = 1.1;
  const OriginalState s = MakeOriginal(psi, theta, phi, psi_dot, theta_dot, phi_dot, 0);

  Eigen::VectorXd q(7);
  Eigen::VectorXd v(6);
  ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));
  pinocchio::forwardKinematics(model, data, q, v);

  // pinocchio reads q as [p; xyzw]: its pose must be the original one.
  const Eigen::Matrix3d R0 = OracleRotation(psi, theta, phi);
  EXPECT_LE((data.oMi[root].rotation() - R0).cwiseAbs().maxCoeff(), kTol);
  EXPECT_LE((data.oMi[root].translation() - s.q_o.head<3>()).cwiseAbs().maxCoeff(), kTol);

  const pinocchio::Motion vel_lwa =
      pinocchio::getVelocity(model, data, root, pinocchio::LOCAL_WORLD_ALIGNED);
  EXPECT_LE((vel_lwa.linear() - s.v_o.head<3>()).cwiseAbs().maxCoeff(), kTol);

  // ω from Ṙ along the Euler-rate trajectory, central difference.
  const auto R_at = [&](double t) -> Eigen::Matrix3d {
    return OracleRotation(psi + t * psi_dot, theta + t * theta_dot, phi + t * phi_dot);
  };
  const Eigen::Matrix3d R_dot = (R_at(kFdStep) - R_at(-kFdStep)) / (2.0 * kFdStep);
  const Eigen::Vector3d w_world = Vee(R_dot * R0.transpose());
  const Eigen::Vector3d w_body = Vee(R0.transpose() * R_dot);
  EXPECT_LE((vel_lwa.angular() - w_world).cwiseAbs().maxCoeff(), kFdTol);
  EXPECT_LE((v.segment<3>(3) - w_body).cwiseAbs().maxCoeff(), kFdTol);
  // The rates are not the angular velocity here — a pass-through would fail.
  EXPECT_GT((v.segment<3>(3) - Eigen::Vector3d(phi_dot, theta_dot, psi_dot)).norm(), 1e-2);
}

// ── Weight approximation: exact upright, visibly not at roll 0.3 rad ─────────

// Jᵀ diag(w_o) J with J = ∂x_o/∂dx at zero base velocity:
// blkdiag(R, P·E_B⁻¹, I, R, P·E_B⁻¹, I), P the (r, p, y) → (y, p, r) reversal.
Eigen::MatrixXd ExactTangentWeight(double psi, double theta, double phi,
                                   const Eigen::VectorXd& w_o) {
  const Eigen::Matrix3d R = OracleRotation(psi, theta, phi);
  const Eigen::Vector3d rpy(phi, theta, psi);
  const Eigen::Matrix3d E_b_inv =
      pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::LOCAL).inverse();
  Eigen::Matrix3d P;
  P << 0, 0, 1, 0, 1, 0, 1, 0, 0;
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(kNx, kNx);
  for (const int off : {0, kHalf}) {
    J.block<3, 3>(off, off) = R;
    J.block<3, 3>(off + 3, off + 3) = P * E_b_inv;
    J.block(off + 6, off + 6, kNj, kNj).setIdentity();
  }
  return J.transpose() * w_o.asDiagonal() * J;
}

Eigen::VectorXd ApproximationProbeWeights() {
  // x / y isotropic (as the original task), every angular weight distinct and
  // non-zero so the angular blocks are tested for ANY weights.
  Eigen::VectorXd w_o(kNx);
  w_o.head<6>() << 10.0, 10.0, 20.0, 3.0, 5.0, 7.0;
  w_o.segment<6>(kHalf) << 1.0, 1.0, 4.0, 0.5, 1.5, 2.5;
  for (int j = 0; j < kNj; ++j) {
    w_o[6 + j] = 0.1 * (j + 1);
    w_o[kHalf + 6 + j] = 0.05 * (j + 1);
  }
  return w_o;
}

TEST(OriginalLayoutAdapter, WeightPermutationIsExactUprightAndVisiblyNotAtRoll) {
  const Eigen::VectorXd w_o = ApproximationProbeWeights();
  Eigen::VectorXd w(kNx);
  ASSERT_TRUE(OriginalWeightsToTangent(w_o, w));
  const Eigen::MatrixXd permuted = w.asDiagonal();

  // Upright with a non-zero yaw: exact.
  const Eigen::MatrixXd upright = ExactTangentWeight(0.9, 0.0, 0.0, w_o);
  EXPECT_LE((upright - permuted).cwiseAbs().maxCoeff(), kTol);

  // Same yaw, roll 0.3 rad: the approximation is visible.
  const Eigen::MatrixXd rolled = ExactTangentWeight(0.9, 0.0, 0.3, w_o);
  EXPECT_GT((rolled - permuted).cwiseAbs().maxCoeff(), 1e-3);
}

// The anisotropic case the header accepts as a deviation: x ≠ y at yaw 0.9, upright.
TEST(OriginalLayoutAdapter, AnisotropicXyWeightIsAnAcceptedDeviationAtYaw) {
  Eigen::VectorXd w_o = ApproximationProbeWeights();
  w_o[1] = 40.0;  // p_base_y ≠ p_base_x
  Eigen::VectorXd w(kNx);
  ASSERT_TRUE(OriginalWeightsToTangent(w_o, w));  // accepted, not refused
  const Eigen::MatrixXd permuted = w.asDiagonal();
  EXPECT_GT((ExactTangentWeight(0.9, 0.0, 0.0, w_o) - permuted).cwiseAbs().maxCoeff(), 1e-3);
  // At yaw 0 the same weights are exact again (sin ψ = 0).
  EXPECT_LE((ExactTangentWeight(0.0, 0.0, 0.0, w_o) - permuted).cwiseAbs().maxCoeff(), kTol);
}

// ── Literals no library computes: the layout itself ─────────────────────────

// R = Rz(ψ) Ry(θ) Rx(φ) by hand: a quarter turn about each axis moves one basis
// vector onto another. Ties the reversal AND the product order to the original's
// definition, which a library oracle of the same product could share a misreading of.
TEST(OriginalLayoutAdapter, QuarterTurnsMoveTheBasisByHand) {
  const double h = kPi / 2.0;

  struct Case {
    double psi, theta, phi;
    Eigen::Vector3d from, to;
  };

  const Case cases[] = {
      {h, 0.0, 0.0, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()},  // yaw: x → y
      {0.0, h, 0.0, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()},  // pitch: z → x
      {0.0, 0.0, h, Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()},  // roll: y → z
      // Roll then yaw (Rz·Rx): y → z (roll) → z (yaw); z → −y (roll) → x (yaw: −y → x).
      {h, 0.0, h, Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()},
      {h, 0.0, h, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX()},
  };
  for (const Case& c : cases) {
    const OriginalState s = MakeOriginal(c.psi, c.theta, c.phi, 0.0, 0.0, 0.0, 0);
    Eigen::VectorXd q(7);
    Eigen::VectorXd v(6);
    ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));
    const Eigen::Vector3d moved = RotationOfQ(q) * c.from;
    EXPECT_LE((moved - c.to).norm(), kTol) << "(ψ, θ, φ) = (" << c.psi << ", " << c.theta << ", "
                                           << c.phi << "): " << moved.transpose();
  }
}

// ω_B = E_B (φ̇, θ̇, ψ̇) with E_B written out from the header, column by column.
TEST(OriginalLayoutAdapter, EulerRatesMapThroughTheWrittenOutMatrix) {
  const double phi = 0.3;
  const double theta = 0.5;
  Eigen::Matrix3d E;
  E << 1.0, 0.0, -std::sin(theta),                          //
      0.0, std::cos(phi), std::sin(phi) * std::cos(theta),  //
      0.0, -std::sin(phi), std::cos(phi) * std::cos(theta);
  // One rate at a time, in the original order (ψ̇, θ̇, φ̇) → E columns z, y, x.
  const double rates[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  const int column[3] = {2, 1, 0};
  for (int k = 0; k < 3; ++k) {
    OriginalState s = MakeOriginal(-0.7, theta, phi, rates[k][0], rates[k][1], rates[k][2], 0);
    s.v_o.head<3>().setZero();
    Eigen::VectorXd q(7);
    Eigen::VectorXd v(6);
    ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));
    EXPECT_LE((v.segment<3>(3) - E.col(column[k])).norm(), kTol) << k;
  }
}

// ── Contract details ─────────────────────────────────────────────────────────

TEST(OriginalLayoutAdapter, VelocityBuffersMayBeTheSameVector) {
  const OriginalState s = MakeOriginal(0.8, -0.4, 0.6, 0.3, -0.2, 0.5, kNj);
  Eigen::VectorXd q(7 + kNj);
  Eigen::VectorXd v_separate(6 + kNj);
  ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v_separate));
  Eigen::VectorXd v_shared = s.v_o;
  ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, v_shared, q, v_shared));  // in place
  EXPECT_LE((v_shared - v_separate).norm(), kTol);

  Eigen::VectorXd q_o(6 + kNj);
  Eigen::VectorXd v_back = v_separate;
  ASSERT_TRUE(FreeFlyerToOriginal(q, v_back, q_o, v_back));  // in place
  EXPECT_LE((v_back - s.v_o).norm(), kTol);
}

TEST(OriginalLayoutAdapter, ForwardQuaternionHasNonNegativeW) {
  // A turn of about −3 rad about one axis has trace < 0; Eigen's matrix → quaternion
  // then makes the axis component positive, which here leaves w = cos(−1.5) · −1 < 0.
  struct Angles {
    double psi, theta, phi;
  };

  for (const Angles a : {Angles{0.0, 0.0, -3.0}, Angles{-3.0, 0.0, 0.0}, Angles{0.2, -0.3, -3.0},
                         Angles{3.1, 0.2, 2.9}}) {
    const OriginalState s = MakeOriginal(a.psi, a.theta, a.phi, 0.0, 0.0, 0.0, 0);
    Eigen::VectorXd q(7);
    Eigen::VectorXd v(6);
    ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));
    EXPECT_GE(q[6], 0.0) << a.psi << " " << a.theta << " " << a.phi;
    EXPECT_LE((RotationOfQ(q) - OracleRotation(a.psi, a.theta, a.phi)).norm(), kTol);
  }
}

TEST(OriginalLayoutAdapter, InverseAcceptsANearUnitAndANegatedQuaternion) {
  const OriginalState s = MakeOriginal(0.4, 0.2, -0.3, 0.1, 0.2, 0.3, kNj);
  Eigen::VectorXd q(7 + kNj);
  Eigen::VectorXd v(6 + kNj);
  ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));
  Eigen::VectorXd ref_q(6 + kNj);
  Eigen::VectorXd ref_v(6 + kNj);
  ASSERT_TRUE(FreeFlyerToOriginal(q, v, ref_q, ref_v));

  Eigen::VectorXd q_o(6 + kNj);
  Eigen::VectorXd v_o(6 + kNj);
  Eigen::VectorXd near = q;
  near.segment<4>(3) *= 1.0004;  // ‖q‖² − 1 ≈ 8e-4, inside the 1e-3 tolerance
  ASSERT_TRUE(FreeFlyerToOriginal(near, v, q_o, v_o));
  EXPECT_LE((q_o - ref_q).norm(), kTol);  // renormalised: the same angles
  EXPECT_LE((v_o - ref_v).norm(), kTol);

  Eigen::VectorXd negated = q;
  negated.segment<4>(3) = -negated.segment<4>(3);  // −q is the same rotation
  ASSERT_TRUE(FreeFlyerToOriginal(negated, v, q_o, v_o));
  EXPECT_LE((q_o - ref_q).norm(), kTol);
  EXPECT_LE((v_o - ref_v).norm(), kTol);
}

TEST(OriginalLayoutAdapter, InverseRejectsANonFiniteVelocityPositionOrJoint) {
  const OriginalState s = MakeOriginal(0.4, 0.2, -0.3, 0.1, 0.2, 0.3, kNj);
  Eigen::VectorXd q(7 + kNj);
  Eigen::VectorXd v(6 + kNj);
  ASSERT_TRUE(OriginalToFreeFlyer(s.q_o, s.v_o, q, v));
  Eigen::VectorXd q_o(6 + kNj);
  Eigen::VectorXd v_o(6 + kNj);
  ASSERT_TRUE(FreeFlyerToOriginal(q, v, q_o, v_o));  // control
  for (const int i : {0, 7 + 5}) {                   // a position, a joint
    Eigen::VectorXd bad = q;
    bad[i] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(FreeFlyerToOriginal(bad, v, q_o, v_o)) << "q[" << i << "]";
  }
  for (const int i : {1, 4, 6 + 3}) {  // linear, angular, a joint rate
    Eigen::VectorXd bad = v;
    bad[i] = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(FreeFlyerToOriginal(q, bad, q_o, v_o)) << "v[" << i << "]";
  }
}

// ── WorldFromBase ────────────────────────────────────────────────────────────

TEST(OriginalLayoutAdapter, WorldFromBaseIsRotateThenTranslate) {
  const pinocchio::SE3 base(OracleRotation(0.8, -0.3, 0.45), Eigen::Vector3d(1.0, -0.5, 0.9));
  const pinocchio::SE3 target(OracleRotation(-1.1, 0.2, 0.6), Eigen::Vector3d(0.3, 0.12, -0.7));
  const pinocchio::SE3 world = WorldFromBase(base, target);
  const Eigen::Vector3d p_manual = base.rotation() * target.translation() + base.translation();
  const Eigen::Matrix3d R_manual = base.rotation() * target.rotation();
  EXPECT_LE((world.translation() - p_manual).cwiseAbs().maxCoeff(), kTol);
  EXPECT_LE((world.rotation() - R_manual).cwiseAbs().maxCoeff(), kTol);
}

}  // namespace
