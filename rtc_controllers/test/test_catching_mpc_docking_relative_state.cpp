// mpc_docking relative-state seams (E1-F13, #739): every derivative the docking
// core assembles into a QP row, against a central difference of the nonlinear
// output it claims to differentiate; the crossing-plane covariance against a
// Monte Carlo; the reference's sanity cases; and the allocation gate.
//
// Why the fixture looks the way it does. A hand at rest makes the transport
// term ω × r vanish, and a ball on the capture axis makes the lateral terms
// vanish — a derivative that drops either would still pass. So every
// finite-difference case runs at a state where the hand ROTATES (‖ω_h‖ well
// away from zero), the ball is OFF the axis, and its velocity is not along it;
// MovingState() asserts those three, and the transport case measures that the
// formula WITHOUT the term is visibly wrong on this fixture.
//
// The C-level malloc gate (rtc_base's malloc_gate.hpp) is defined in this TU
// (one per binary).
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_controllers/catching/mpc_docking_relative_state.hpp"
#include "rtc_controllers/testing/alloc_gate.hpp"
#include "rtc_controllers/testing/mpc_segment_core_fixture.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <numbers>
#include <random>
#include <string>
#include <vector>

namespace {

using rtc::catching::ComputeDockingCrossing;
using rtc::catching::ComputeDockingFrameKinematics;
using rtc::catching::ComputeDockingImpact;
using rtc::catching::ComputeDockingManipulability;
using rtc::catching::ComputeDockingRelativeState;
using rtc::catching::DockingAxialSpeedRow;
using rtc::catching::DockingBallGradient;
using rtc::catching::DockingBallGradientOf;
using rtc::catching::DockingCorridorMinSlack;
using rtc::catching::DockingCorridorRow;
using rtc::catching::DockingCovariance6;
using rtc::catching::DockingCrossing;
using rtc::catching::DockingEnvelopeMinSlack;
using rtc::catching::DockingEnvelopeRow;
using rtc::catching::DockingFrameKinematics;
using rtc::catching::DockingImpact;
using rtc::catching::DockingImpactWork;
using rtc::catching::DockingLateralChanceRow;
using rtc::catching::DockingLateralSpeedRow;
using rtc::catching::DockingManipulabilityWork;
using rtc::catching::DockingRelativeAcceleration;
using rtc::catching::DockingRelativeState;
using rtc::catching::DockingScalar;
using rtc::catching::DockingTimingRow;
using rtc::catching::DockingTimingSigmaMax;
using rtc::catching::DockingVelocitySigma;
using rtc::catching::NormalCdf;
using rtc::catching::NormalQuantile;
namespace fx = rtc::testing::mpc_segment_core;

constexpr double kFdStep = 1e-6;
// Central differences at step 1e-6 on O(1) functions: truncation ~1e-12·f''',
// rounding ~1e-10. 5e-7 leaves two orders of margin and is three orders below
// any term a wrong derivative would drop on this fixture.
constexpr double kFdTol = 5e-7;

// RecordProperty(double) goes through to_string (6 fixed decimals); small
// values need their own formatting to mean anything in the result XML.
void RecordSci(const std::string& key, double value) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.6e", value);
  ::testing::Test::RecordProperty(key, buf);
}

struct State {
  fx::ArmModel arm;
  pinocchio::Model model;  // with armature
  Eigen::VectorXd q, v;
  Eigen::Vector3d p_b, v_b;
};

// A state with a rotating hand and an off-axis, obliquely moving ball.
State MovingState(const fx::ArmModel& arm, unsigned seed) {
  State s;
  s.arm = arm;
  s.model = *arm.model;
  s.model.armature = Eigen::VectorXd::Constant(s.model.nv, 0.05);
  std::mt19937 gen(seed);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  const Eigen::Index n = s.model.nv;
  s.q = arm.q_nominal;
  s.v.resize(n);
  for (Eigen::Index j = 0; j < n; ++j) {
    s.q[j] += 0.2 * uni(gen);
    s.v[j] = 0.8 * uni(gen) + (j % 2 == 0 ? 0.4 : -0.4);
  }
  pinocchio::Data data(s.model);
  DockingFrameKinematics kin;
  kin.Resize(n);
  EXPECT_TRUE(ComputeDockingFrameKinematics(s.model, data, arm.frame, s.q, s.v, kin));
  // 15 cm up the capture axis, 3 cm and −2 cm off it.
  s.p_b = kin.p + kin.R * Eigen::Vector3d(0.03, -0.02, 0.15);
  // Mostly toward the hand (−e₃), with a lateral part.
  s.v_b = kin.v + kin.R * Eigen::Vector3d(0.35, -0.2, -1.6);
  EXPECT_GT(kin.w.norm(), 0.3) << "the hand must rotate for the transport term to matter";
  return s;
}

struct Evaluated {
  DockingFrameKinematics kin;
  DockingRelativeState rel;
};

Evaluated Evaluate(const State& s, const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  Evaluated e;
  const Eigen::Index n = s.model.nv;
  e.kin.Resize(n);
  e.rel.Resize(n);
  pinocchio::Data data(s.model);
  EXPECT_TRUE(ComputeDockingFrameKinematics(s.model, data, s.arm.frame, q, v, e.kin));
  EXPECT_TRUE(ComputeDockingRelativeState(e.kin, s.p_b, s.v_b, e.rel));
  return e;
}

using VecFn = std::function<Eigen::VectorXd(const Eigen::VectorXd& q, const Eigen::VectorXd& v)>;

// Central difference of f with respect to q (wrt_v = false) or q̇.
Eigen::MatrixXd Fd(const State& s, const VecFn& f, bool wrt_v) {
  const Eigen::Index n = s.model.nv;
  const Eigen::VectorXd f0 = f(s.q, s.v);
  Eigen::MatrixXd jac(f0.size(), n);
  for (Eigen::Index j = 0; j < n; ++j) {
    Eigen::VectorXd qp = s.q;
    Eigen::VectorXd qm = s.q;
    Eigen::VectorXd vp = s.v;
    Eigen::VectorXd vm = s.v;
    (wrt_v ? vp : qp)[j] += kFdStep;
    (wrt_v ? vm : qm)[j] -= kFdStep;
    jac.col(j) = (f(qp, vp) - f(qm, vm)) / (2.0 * kFdStep);
  }
  return jac;
}

double MaxAbs(const Eigen::MatrixXd& m) {
  return m.size() == 0 ? 0.0 : m.cwiseAbs().maxCoeff();
}

// Check a scalar row's analytic gradient against central differences of its
// value, in q and q̇, and that the gradient is not trivially zero where the
// caller says it must not be.
using ScalarFn = std::function<void(const Evaluated&, DockingScalar&)>;

void ExpectScalarGradient(const State& s, const ScalarFn& row, const char* what,
                          bool expect_dv_nonzero) {
  const Eigen::Index n = s.model.nv;
  DockingScalar at;
  at.Resize(n);
  row(Evaluate(s, s.q, s.v), at);
  const VecFn value = [&](const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    DockingScalar out;
    out.Resize(n);
    row(Evaluate(s, q, v), out);
    return Eigen::VectorXd::Constant(1, out.value);
  };
  const Eigen::MatrixXd fd_q = Fd(s, value, false);
  const Eigen::MatrixXd fd_v = Fd(s, value, true);
  EXPECT_LT(MaxAbs(fd_q.row(0).transpose() - at.dq), kFdTol * std::max(1.0, MaxAbs(fd_q)))
      << what << " ∂/∂q on " << s.arm.name;
  EXPECT_LT(MaxAbs(fd_v.row(0).transpose() - at.dv), kFdTol * std::max(1.0, MaxAbs(fd_v)))
      << what << " ∂/∂q̇ on " << s.arm.name;
  EXPECT_GT(MaxAbs(fd_q), 1e-4) << what << ": the fixture does not exercise ∂/∂q";
  if (expect_dv_nonzero) {
    EXPECT_GT(MaxAbs(fd_v), 1e-4) << what << ": the fixture does not exercise ∂/∂q̇";
  }
}

std::vector<State> States() {
  return {MovingState(fx::RealArm6(), 11), MovingState(fx::RealArm7(), 23)};
}

Eigen::Matrix3d TestSigmaP() {
  Eigen::Matrix3d a;
  a << 4.0, 1.0, -0.5, 0.3, 6.0, 1.2, -0.8, 0.4, 15.0;
  const Eigen::Matrix3d s = 1e-6 * (a * a.transpose());  // mm-scale, anisotropic, correlated
  return 0.5 * (s + s.transpose());
}

DockingCovariance6 TestSigmaB() {
  Eigen::Matrix<double, 6, 6> a;
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      a(r, c) = std::sin(0.9 * r + 1.7 * c + 0.3) * (r < 3 ? 5e-3 : 6e-2);
    }
  }
  const DockingCovariance6 s = a * a.transpose();
  return 0.5 * (s + s.transpose());
}

// ── Kinematics ───────────────────────────────────────────────────────────────

TEST(DockingRelativeState, FrameKinematicsMatchFiniteDifferences) {
  for (const State& s : States()) {
    const Evaluated at = Evaluate(s, s.q, s.v);
    // J_p, J_ω from the pose: δp = J_p δq, δR Rᵀ = [J_ω δq]×.
    const Eigen::Index n = s.model.nv;
    for (Eigen::Index j = 0; j < n; ++j) {
      Eigen::VectorXd qp = s.q;
      Eigen::VectorXd qm = s.q;
      qp[j] += kFdStep;
      qm[j] -= kFdStep;
      const Evaluated ep = Evaluate(s, qp, s.v);
      const Evaluated em = Evaluate(s, qm, s.v);
      const Eigen::Vector3d dp = (ep.kin.p - em.kin.p) / (2.0 * kFdStep);
      const Eigen::Matrix3d dr = (ep.kin.R - em.kin.R) / (2.0 * kFdStep) * at.kin.R.transpose();
      const Eigen::Vector3d dtheta(dr(2, 1), dr(0, 2), dr(1, 0));
      EXPECT_LT((dp - at.kin.j_p.col(j)).cwiseAbs().maxCoeff(), kFdTol) << s.arm.name;
      EXPECT_LT((dtheta - at.kin.j_w.col(j)).cwiseAbs().maxCoeff(), kFdTol) << s.arm.name;
    }
    const VecFn v_h = [&](const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
      return Eigen::VectorXd(Evaluate(s, q, v).kin.v);
    };
    const VecFn w_h = [&](const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
      return Eigen::VectorXd(Evaluate(s, q, v).kin.w);
    };
    EXPECT_LT(MaxAbs(Fd(s, v_h, false) - at.kin.dv_dq), kFdTol) << s.arm.name << " ∂v_h/∂q";
    EXPECT_LT(MaxAbs(Fd(s, w_h, false) - at.kin.dw_dq), kFdTol) << s.arm.name << " ∂ω_h/∂q";
    EXPECT_LT(MaxAbs(Fd(s, v_h, true) - at.kin.j_p), kFdTol) << s.arm.name;
    EXPECT_LT(MaxAbs(Fd(s, w_h, true) - at.kin.j_w), kFdTol) << s.arm.name;
    EXPECT_GT(MaxAbs(at.kin.dw_dq), 1e-2) << "∂ω_h/∂q is not exercised";
  }
}

TEST(DockingRelativeState, RelativeStateMatchesFiniteDifferences) {
  for (const State& s : States()) {
    const Evaluated at = Evaluate(s, s.q, s.v);
    const VecFn r_h = [&](const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
      return Eigen::VectorXd(Evaluate(s, q, v).rel.r_h);
    };
    const VecFn nu_h = [&](const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
      return Eigen::VectorXd(Evaluate(s, q, v).rel.nu_h);
    };
    const Eigen::MatrixXd fd_nu_q = Fd(s, nu_h, false);
    EXPECT_LT(MaxAbs(Fd(s, r_h, false) - at.rel.dr_dq), kFdTol) << s.arm.name << " ∂r^H/∂q";
    EXPECT_LT(MaxAbs(Fd(s, r_h, true)), 1e-9) << s.arm.name << " r^H does not depend on q̇";
    EXPECT_LT(MaxAbs(fd_nu_q - at.rel.dnu_dq), kFdTol) << s.arm.name << " ∂ν^H/∂q";
    EXPECT_LT(MaxAbs(Fd(s, nu_h, true) - at.rel.dr_dq), kFdTol) << s.arm.name << " ∂ν^H/∂q̇";

    // The derived scalars.
    EXPECT_DOUBLE_EQ(at.rel.s, at.rel.r_h.z());
    EXPECT_DOUBLE_EQ(at.rel.c, -at.rel.nu_h.z());
    EXPECT_GT(at.rel.c, 0.5) << "the fixture's ball must approach";
    EXPECT_GT(at.rel.Rho().norm(), 0.02) << "the fixture's ball must be off the axis";

    // Positive control: the derivative of a HAND-FIXED point (no [ω]× J_p term,
    // which is what "the ball does not move with the hand" adds) is visibly
    // wrong here. If this stops failing, the fixture stopped testing it.
    Eigen::MatrixXd without = at.rel.dnu_dq;
    const Eigen::Matrix3d rt = at.kin.R.transpose();
    for (Eigen::Index j = 0; j < s.model.nv; ++j) {
      const Eigen::Vector3d jp = at.kin.j_p.col(j);
      without.col(j) -= rt * at.kin.w.cross(jp);
    }
    EXPECT_GT(MaxAbs(fd_nu_q - without), 1e-2) << s.arm.name;
  }
}

// Reference §5.2 / §15 item 2: the LOCAL route gives the same ν^H.
TEST(DockingRelativeState, LocalJacobianRouteGivesTheSameRelativeVelocity) {
  for (const State& s : States()) {
    const Evaluated at = Evaluate(s, s.q, s.v);
    pinocchio::Data data(s.model);
    Eigen::MatrixXd j_local = Eigen::MatrixXd::Zero(6, s.model.nv);
    pinocchio::computeFrameJacobian(s.model, data, s.q, s.arm.frame, pinocchio::LOCAL, j_local);
    // Not data.oMf: the LOCAL Jacobian pass never forms the world placement.
    const Eigen::Matrix3d r = at.kin.R;
    const Eigen::Vector3d w_local = j_local.bottomRows(3) * s.v;
    const Eigen::Vector3d nu_local =
        r.transpose() * s.v_b - j_local.topRows(3) * s.v - w_local.cross(at.rel.r_h);
    EXPECT_LT((nu_local - at.rel.nu_h).cwiseAbs().maxCoeff(), 1e-12) << s.arm.name;
    // Mixing the conventions (LOCAL Jacobian in the world-aligned formula) must
    // NOT agree — the fixture's frame is rotated away from the world.
    const Eigen::Vector3d mixed =
        r.transpose() * (s.v_b - j_local.topRows(3) * s.v) - w_local.cross(at.rel.r_h);
    EXPECT_GT((mixed - at.rel.nu_h).cwiseAbs().maxCoeff(), 1e-2) << s.arm.name;
  }
}

TEST(DockingRelativeState, RelativeAccelerationIsTheTimeDerivativeOfRelativeVelocity) {
  for (const State& s : States()) {
    const Eigen::Index n = s.model.nv;
    Eigen::VectorXd a(n);
    for (Eigen::Index j = 0; j < n; ++j) {
      a[j] = 1.5 * std::sin(1.0 + 0.7 * static_cast<double>(j));
    }
    const Eigen::Vector3d a_b(0.0, 0.0, -9.81);
    const auto nu_at = [&](double t) {
      const Eigen::VectorXd q = s.q + t * s.v + 0.5 * t * t * a;
      const Eigen::VectorXd v = s.v + t * a;
      State moved = s;
      moved.p_b = s.p_b + t * s.v_b + 0.5 * t * t * a_b;
      moved.v_b = s.v_b + t * a_b;
      return Evaluate(moved, q, v).rel.nu_h;
    };
    const double h = 1e-5;
    const Eigen::Vector3d fd = (nu_at(h) - nu_at(-h)) / (2.0 * h);
    pinocchio::Data data(s.model);
    Eigen::Vector3d a_rel;
    ASSERT_TRUE(DockingRelativeAcceleration(s.model, data, s.arm.frame, s.q, s.v, a, s.p_b, s.v_b,
                                            a_b, a_rel));
    EXPECT_LT((fd - a_rel).cwiseAbs().maxCoeff(), 1e-5) << s.arm.name;
    EXPECT_GT(a_rel.norm(), 1.0);
  }
}

// ── Approach rows ────────────────────────────────────────────────────────────

TEST(DockingRelativeState, CorridorAndEnvelopeRowsMatchFiniteDifferences) {
  const double r_ent = 0.04;
  const double tan_theta = 0.35;
  const double s_c = 0.006;
  for (const State& s : States()) {
    const double s_now = Evaluate(s, s.q, s.v).rel.s;
    // Both branches of ℓ⁺: the entrance below the ball (ℓ > 0) and above it.
    for (const double s_ent : {s_now - 0.08, s_now + 0.05}) {
      double dg_ds = 0.0;
      const ScalarFn corridor = [&](const Evaluated& e, DockingScalar& out) {
        DockingCorridorRow(e.rel, s_ent, r_ent, tan_theta, s_c, out, dg_ds);
      };
      ExpectScalarGradient(s, corridor, "corridor", false);
      // ∂g/∂s_c by a difference in s_c.
      const Evaluated at = Evaluate(s, s.q, s.v);
      DockingScalar gp;
      DockingScalar gm;
      gp.Resize(s.model.nv);
      gm.Resize(s.model.nv);
      double unused = 0.0;
      DockingCorridorRow(at.rel, s_ent, r_ent, tan_theta, s_c + 1e-6, gp, unused);
      DockingCorridorRow(at.rel, s_ent, r_ent, tan_theta, s_c - 1e-6, gm, unused);
      DockingScalar g0;
      g0.Resize(s.model.nv);
      DockingCorridorRow(at.rel, s_ent, r_ent, tan_theta, s_c, g0, dg_ds);
      EXPECT_NEAR((gp.value - gm.value) / 2e-6, dg_ds, 1e-8);
      EXPECT_LT(dg_ds, 0.0) << "more slack must loosen the row";

      // The minimal slack makes the row exactly active (or is zero inside).
      const double s_min = DockingCorridorMinSlack(at.rel, s_ent, r_ent, tan_theta);
      DockingScalar g_min;
      g_min.Resize(s.model.nv);
      DockingCorridorRow(at.rel, s_ent, r_ent, tan_theta, s_min, g_min, unused);
      if (s_min > 0.0) {
        EXPECT_NEAR(g_min.value, 0.0, 1e-15);
      } else {
        EXPECT_LE(g_min.value, 0.0);
      }

      const ScalarFn envelope = [&](const Evaluated& e, DockingScalar& out) {
        DockingEnvelopeRow(e.rel, s_ent, 0.6, 3.0, 0.2, out);
      };
      ExpectScalarGradient(s, envelope, "envelope", true);
      const double sv_min = DockingEnvelopeMinSlack(at.rel, s_ent, 0.6, 3.0);
      DockingScalar gv;
      gv.Resize(s.model.nv);
      DockingEnvelopeRow(at.rel, s_ent, 0.6, 3.0, sv_min, gv);
      EXPECT_LE(gv.value, 1e-15);
    }
    // With ℓ < 0 the guarded width does not shrink with the gap: the row has
    // no ∂/∂s part, so it equals the row of a ball exactly at the entrance.
    const Evaluated at = Evaluate(s, s.q, s.v);
    DockingScalar below;
    DockingScalar at_entrance;
    below.Resize(s.model.nv);
    at_entrance.Resize(s.model.nv);
    double d0 = 0.0;
    double d1 = 0.0;
    DockingCorridorRow(at.rel, at.rel.s + 0.05, r_ent, tan_theta, s_c, below, d0);
    DockingCorridorRow(at.rel, at.rel.s, r_ent, tan_theta, s_c, at_entrance, d1);
    EXPECT_DOUBLE_EQ(below.value, at_entrance.value);
    EXPECT_DOUBLE_EQ(d0, -2.0 * (r_ent + s_c));
  }
}

// ── Chance rows ──────────────────────────────────────────────────────────────

TEST(DockingRelativeState, ChanceRowsMatchFiniteDifferences) {
  const Eigen::Matrix3d sigma_p = TestSigmaP();
  const DockingCovariance6 sigma_b = TestSigmaB();
  const double eps = 1e-4;
  for (const State& s : States()) {
    const double c_now = Evaluate(s, s.q, s.v).rel.c;
    // Guard off (c > c_min) and guard on (c < c_min): both branches of c̃.
    for (const double c_min : {0.3 * c_now, 1.5 * c_now}) {
      for (const Eigen::Vector2d& a : {Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(-0.6, 0.8)}) {
        const ScalarFn lateral = [&](const Evaluated& e, DockingScalar& out) {
          DockingLateralChanceRow(e.kin, e.rel, sigma_p, a, 2.3, c_min, eps, out);
        };
        ExpectScalarGradient(s, lateral, c_min < c_now ? "lateral chance" : "lateral chance (c̃)",
                             true);
      }
    }
    const ScalarFn timing = [&](const Evaluated& e, DockingScalar& out) {
      DockingTimingRow(e.kin, e.rel, sigma_p, 0.012, eps, out);
    };
    ExpectScalarGradient(s, timing, "timing", true);

    for (const Eigen::Vector3d& m :
         {Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.8, -0.6, 0.0)}) {
      const ScalarFn sigma = [&](const Evaluated& e, DockingScalar& out) {
        DockingVelocitySigma(e.kin, sigma_b, m, eps, out);
      };
      ExpectScalarGradient(s, sigma, "velocity sigma", true);
    }
    for (const double kappa : {-2.1, 2.1}) {
      const ScalarFn axial = [&](const Evaluated& e, DockingScalar& out) {
        DockingAxialSpeedRow(e.kin, e.rel, sigma_b, kappa, eps, out);
      };
      ExpectScalarGradient(s, axial, "axial speed", true);
    }
    const ScalarFn face = [&](const Evaluated& e, DockingScalar& out) {
      DockingLateralSpeedRow(e.kin, e.rel, sigma_b, Eigen::Vector2d(0.5, std::sqrt(0.75)), 1.9, eps,
                             out);
    };
    ExpectScalarGradient(s, face, "lateral speed", true);
  }
}

// The gradient of √(variance + ε²) must stay finite at zero variance — the
// reason for ε_σ on every square root.
TEST(DockingRelativeState, ZeroCovarianceGivesFiniteGradients) {
  const State s = MovingState(fx::RealArm7(), 5);
  const Evaluated at = Evaluate(s, s.q, s.v);
  DockingScalar row;
  row.Resize(s.model.nv);
  const double eps = 1e-4;
  DockingLateralChanceRow(at.kin, at.rel, Eigen::Matrix3d::Zero(), Eigen::Vector2d(1.0, 0.0), 2.0,
                          0.1, eps, row);
  EXPECT_TRUE(row.dq.allFinite() && row.dv.allFinite());
  DockingTimingRow(at.kin, at.rel, Eigen::Matrix3d::Zero(), 0.01, eps, row);
  EXPECT_TRUE(row.dq.allFinite() && row.dv.allFinite());
  DockingAxialSpeedRow(at.kin, at.rel, DockingCovariance6::Zero(), 2.0, eps, row);
  EXPECT_TRUE(row.dq.allFinite() && row.dv.allFinite());
}

// ── Sanity cases of the reference ────────────────────────────────────────────

// Σ = 0: each chance row is its deterministic row shifted by exactly κ ε_σ.
TEST(DockingRelativeState, SanityZeroCovarianceIsTheDeterministicRowPlusKappaEps) {
  const State s = MovingState(fx::RealArm6(), 3);
  const Evaluated at = Evaluate(s, s.q, s.v);
  const double eps = 1e-4;
  const double kappa = 2.4;
  DockingScalar row;
  row.Resize(s.model.nv);
  const Eigen::Vector2d a(0.6, -0.8);
  DockingLateralChanceRow(at.kin, at.rel, Eigen::Matrix3d::Zero(), a, kappa, 0.1, eps, row);
  EXPECT_NEAR(row.value - a.dot(at.rel.Rho()), kappa * eps, 1e-16);
  EXPECT_LT(
      MaxAbs(row.dq - (a.x() * at.rel.dr_dq.row(0) + a.y() * at.rel.dr_dq.row(1)).transpose()),
      1e-15);
  EXPECT_LT(MaxAbs(row.dv), 1e-15);

  DockingAxialSpeedRow(at.kin, at.rel, DockingCovariance6::Zero(), -kappa, eps, row);
  EXPECT_NEAR(row.value - at.rel.c, -kappa * eps, 1e-16);
  DockingAxialSpeedRow(at.kin, at.rel, DockingCovariance6::Zero(), kappa, eps, row);
  EXPECT_NEAR(row.value - at.rel.c, kappa * eps, 1e-16);
  const Eigen::Vector2d u(0.0, 1.0);
  DockingLateralSpeedRow(at.kin, at.rel, DockingCovariance6::Zero(), u, kappa, eps, row);
  EXPECT_NEAR(row.value - at.rel.nu_h.y(), kappa * eps, 1e-16);
  const double k_t = 0.02;
  DockingTimingRow(at.kin, at.rel, Eigen::Matrix3d::Zero(), k_t, eps, row);
  EXPECT_NEAR(row.value - at.rel.c * k_t, -eps, 1e-16);

  DockingCrossing cross;
  ComputeDockingCrossing(at.kin, at.rel, Eigen::Matrix3d::Zero(), 0.1, eps, cross);
  EXPECT_EQ(cross.sigma_rho, Eigen::Matrix2d::Zero());
  EXPECT_DOUBLE_EQ(cross.sigma_s, eps);
}

// ν ∥ e₃: the crossing-plane covariance is the fixed-time lateral covariance.
TEST(DockingRelativeState, SanityAxialApproachHasTheFixedTimeLateralCovariance) {
  DockingFrameKinematics kin;
  kin.Resize(0);
  kin.R = Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.3, -0.5, 0.8).normalized()).toRotationMatrix();
  DockingRelativeState rel;
  rel.Resize(0);
  rel.nu_h = Eigen::Vector3d(0.0, 0.0, -1.7);
  rel.c = 1.7;
  const Eigen::Matrix3d sigma_p = TestSigmaP();
  DockingCrossing cross;
  ComputeDockingCrossing(kin, rel, sigma_p, 0.1, 1e-5, cross);
  const Eigen::Matrix3d sigma_h = kin.R.transpose() * sigma_p * kin.R;
  EXPECT_LT((cross.sigma_rho - sigma_h.topLeftCorner<2, 2>()).cwiseAbs().maxCoeff(), 1e-18);
  EXPECT_FALSE(cross.c_guarded);
  EXPECT_NEAR(cross.var_s, sigma_h(2, 2), 1e-18);
  EXPECT_NEAR(cross.sigma_t, cross.sigma_s / 1.7, 1e-18);
}

// A hand at rest and a ball coming straight down the axis: c = ‖v_b‖ > 0.
TEST(DockingRelativeState, SanityStationaryHandAxialBallClosesAtItsSpeed) {
  for (const fx::ArmModel& arm : {fx::RealArm6(), fx::RealArm7()}) {
    State s;
    s.arm = arm;
    s.model = *arm.model;
    s.q = arm.q_nominal;
    s.v = Eigen::VectorXd::Zero(s.model.nv);
    pinocchio::Data data(s.model);
    DockingFrameKinematics kin;
    kin.Resize(s.model.nv);
    ASSERT_TRUE(ComputeDockingFrameKinematics(s.model, data, arm.frame, s.q, s.v, kin));
    const double speed = 3.2;
    s.p_b = kin.p + 0.4 * kin.R.col(2);
    s.v_b = -speed * kin.R.col(2);
    const Evaluated at = Evaluate(s, s.q, s.v);
    EXPECT_NEAR(at.rel.c, speed, 1e-12) << arm.name;
    EXPECT_NEAR(at.rel.s, 0.4, 1e-12) << arm.name;
    EXPECT_LT(at.rel.Rho().norm(), 1e-12) << arm.name;
  }
}

// ── Crossing-plane covariance: Monte Carlo ───────────────────────────────────

// Reference §15 item 5: linear relative motion, position noise. In that model
// the crossing map is exactly Π δr, so this checks the ALGEBRA of Σ_ρ (signs,
// indices, frames) — not the size of the first-order approximation, which the
// next test records.
TEST(DockingRelativeState, CrossingCovarianceMatchesMonteCarlo) {
  DockingFrameKinematics kin;
  kin.Resize(0);
  kin.R = Eigen::AngleAxisd(1.1, Eigen::Vector3d(-0.4, 0.7, 0.2).normalized()).toRotationMatrix();
  DockingRelativeState rel;
  rel.Resize(0);
  rel.nu_h = Eigen::Vector3d(0.3, -0.1, -2.0);  // the reference's example
  rel.c = 2.0;
  const Eigen::Vector3d sigma_h(4e-3, 4e-3, 20e-3);
  const Eigen::Matrix3d sigma_r_h = sigma_h.cwiseAbs2().asDiagonal();
  const Eigen::Matrix3d sigma_p = kin.R * sigma_r_h * kin.R.transpose();
  DockingCrossing cross;
  ComputeDockingCrossing(kin, rel, sigma_p, 0.1, 0.0, cross);

  const int samples = 200000;
  std::mt19937_64 gen(20261006);
  std::normal_distribution<double> gauss(0.0, 1.0);
  Eigen::Matrix2d sum = Eigen::Matrix2d::Zero();
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (int i = 0; i < samples; ++i) {
    const Eigen::Vector3d dr(sigma_h.x() * gauss(gen), sigma_h.y() * gauss(gen),
                             sigma_h.z() * gauss(gen));
    // The ball crosses the plane e₃ᵀ r = s_ent when e₃ᵀ(δr + ν δt) = 0.
    const double dt = dr.z() / rel.c;
    const Eigen::Vector3d crossing = dr + rel.nu_h * dt;
    EXPECT_NEAR(crossing.z(), 0.0, 1e-15);
    const Eigen::Vector2d rho = crossing.head<2>();
    mean += rho;
    sum += rho * rho.transpose();
  }
  mean /= samples;
  const Eigen::Matrix2d sample = sum / samples - mean * mean.transpose();
  const double tol = 0.03 * cross.sigma_rho.norm();
  EXPECT_LT((sample - cross.sigma_rho).cwiseAbs().maxCoeff(), tol);

  // Positive control: the fixed-time covariance (Π = I) misses by more than
  // the tolerance on this example (x variance 16 → 25 mm²).
  const Eigen::Matrix2d fixed_time = sigma_r_h.topLeftCorner<2, 2>();
  EXPECT_GT((sample - fixed_time).cwiseAbs().maxCoeff(), tol);
  EXPECT_NEAR(cross.sigma_rho(0, 0), 25e-6, 1e-12);
  EXPECT_NEAR(fixed_time(0, 0), 16e-6, 1e-18);
  RecordSci("mc_max_abs_diff_m2", (sample - cross.sigma_rho).cwiseAbs().maxCoeff());
  RecordSci("mc_tolerance_m2", tol);
}

// RECORDED, not judged: how far the first-order Σ_ρ is from the covariance of
// the ACTUAL crossing point — 6×6 noise on [p; v], a ball under gravity, a
// hand that translates, accelerates and rotates, the crossing instant found by
// root-finding on e₃ᵀ r^H(t) = s_ent.
TEST(DockingRelativeState, RecordsFirstOrderErrorAgainstTheNonlinearCrossing) {
  DockingFrameKinematics kin;
  kin.Resize(0);
  kin.R = Eigen::AngleAxisd(0.9, Eigen::Vector3d(0.2, 0.9, -0.3).normalized()).toRotationMatrix();
  kin.p = Eigen::Vector3d(0.5, 0.1, 0.6);
  kin.v = Eigen::Vector3d(-0.6, 0.2, 0.3);
  kin.w = Eigen::Vector3d(0.8, -0.5, 0.4);
  const Eigen::Vector3d a_h(1.5, -1.0, 2.0);
  const Eigen::Vector3d g(0.0, 0.0, -9.81);
  const double s_ent = 0.08;
  // Nominal ball: ON the entrance plane at t = 0, off the axis, closing obliquely.
  const Eigen::Vector3d p_b = kin.p + kin.R * Eigen::Vector3d(0.01, -0.015, s_ent);
  const Eigen::Vector3d v_b =
      kin.v + kin.w.cross(p_b - kin.p) + kin.R * Eigen::Vector3d(0.4, -0.2, -2.5);
  DockingRelativeState rel;
  rel.Resize(0);
  ASSERT_TRUE(ComputeDockingRelativeState(kin, p_b, v_b, rel));
  ASSERT_NEAR(rel.s, s_ent, 1e-12);

  DockingCovariance6 sigma_b = DockingCovariance6::Zero();
  sigma_b.topLeftCorner<3, 3>() = TestSigmaP() * 4.0;  // ~10–30 mm
  sigma_b.bottomRightCorner<3, 3>() = Eigen::Vector3d(0.05, 0.05, 0.12).cwiseAbs2().asDiagonal();
  sigma_b(0, 3) = sigma_b(3, 0) = 2e-4;
  Eigen::LLT<DockingCovariance6> chol(sigma_b);
  ASSERT_EQ(chol.info(), Eigen::Success);
  const DockingCovariance6 lower = chol.matrixL();

  DockingCrossing cross;
  ComputeDockingCrossing(kin, rel, sigma_b.topLeftCorner<3, 3>(), 0.1, 0.0, cross);

  const auto gap = [&](const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, double t) {
    const Eigen::Matrix3d r_t =
        Eigen::AngleAxisd(kin.w.norm() * t, kin.w.normalized()).toRotationMatrix() * kin.R;
    const Eigen::Vector3d p_h = kin.p + kin.v * t + 0.5 * t * t * a_h;
    const Eigen::Vector3d ball = p0 + v0 * t + 0.5 * t * t * g;
    return Eigen::Vector3d(r_t.transpose() * (ball - p_h));
  };
  const int samples = 200000;
  std::mt19937_64 gen(739);
  std::normal_distribution<double> gauss(0.0, 1.0);
  Eigen::Matrix2d sum = Eigen::Matrix2d::Zero();
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  int used = 0;
  for (int i = 0; i < samples; ++i) {
    Eigen::Matrix<double, 6, 1> z;
    for (int k = 0; k < 6; ++k) {
      z[k] = gauss(gen);
    }
    const Eigen::Matrix<double, 6, 1> d = lower * z;
    const Eigen::Vector3d p0 = p_b + d.head<3>();
    const Eigen::Vector3d v0 = v_b + d.tail<3>();
    // Bisection on [−0.1, 0.1] s: the gap falls monotonically at ~2.5 m/s.
    double lo = -0.1;
    double hi = 0.1;
    if (!(gap(p0, v0, lo).z() > s_ent && gap(p0, v0, hi).z() < s_ent)) {
      continue;
    }
    for (int it = 0; it < 60; ++it) {
      const double mid = 0.5 * (lo + hi);
      (gap(p0, v0, mid).z() > s_ent ? lo : hi) = mid;
    }
    const Eigen::Vector2d rho = gap(p0, v0, 0.5 * (lo + hi)).head<2>();
    mean += rho;
    sum += rho * rho.transpose();
    ++used;
  }
  ASSERT_GT(used, samples * 99 / 100);
  mean /= used;
  const Eigen::Matrix2d sample = sum / used - mean * mean.transpose();
  RecordSci("nonlinear_rel_frobenius_diff",
            (sample - cross.sigma_rho).norm() / cross.sigma_rho.norm());
  RecordSci("nonlinear_sigma_rho_xx_m2", sample(0, 0));
  RecordSci("first_order_sigma_rho_xx_m2", cross.sigma_rho(0, 0));
  RecordSci("nonlinear_mean_shift_m", (mean - rel.Rho()).norm());
  ::testing::Test::RecordProperty("nonlinear_samples_used", used);
  SUCCEED();
}

// The lateral row's variance IS aᵀ Σ_ρ a (the row evaluates it as a quadratic
// form of Σ_p; ComputeDockingCrossing builds Σ_ρ through Π).
TEST(DockingRelativeState, LateralRowVarianceIsTheCrossingCovarianceQuadraticForm) {
  const State s = MovingState(fx::RealArm7(), 41);
  const Evaluated at = Evaluate(s, s.q, s.v);
  const Eigen::Matrix3d sigma_p = TestSigmaP();
  DockingCrossing cross;
  ComputeDockingCrossing(at.kin, at.rel, sigma_p, 0.1, 0.0, cross);
  DockingScalar row;
  row.Resize(s.model.nv);
  const Eigen::Vector2d a(-0.28, 0.96);
  const double kappa = 1.7;
  DockingLateralChanceRow(at.kin, at.rel, sigma_p, a, kappa, 0.1, 1e-9, row);
  const double sigma = (row.value - a.dot(at.rel.Rho())) / kappa;
  EXPECT_NEAR(sigma * sigma, a.dot(cross.sigma_rho * a), 1e-14);
}

// ── Timing ───────────────────────────────────────────────────────────────────

TEST(DockingRelativeState, NormalQuantileInvertsTheCdf) {
  for (const double p : {1e-9, 1e-4, 0.01, 0.2, 0.5, 0.8, 0.975, 0.999999}) {
    double z = 0.0;
    ASSERT_TRUE(NormalQuantile(p, z));
    EXPECT_NEAR(NormalCdf(z), p, 1e-14 * std::max(1.0, p / 1e-9)) << p;
  }
  double z = 0.0;
  ASSERT_TRUE(NormalQuantile(0.975, z));
  EXPECT_NEAR(z, 1.959963984540054, 1e-12);
  EXPECT_FALSE(NormalQuantile(0.0, z));
  EXPECT_FALSE(NormalQuantile(1.0, z));
  EXPECT_FALSE(NormalQuantile(std::numeric_limits<double>::quiet_NaN(), z));
}

TEST(DockingRelativeState, TimingSigmaMaxCentredIsTheReferenceFormula) {
  const double lo = 0.004;
  const double hi = 0.030;
  const double eps_t = 0.05;
  double sigma_max = 0.0;
  ASSERT_TRUE(DockingTimingSigmaMax(lo, hi, 0.5 * (lo + hi), eps_t, sigma_max));
  double kappa_t = 0.0;
  ASSERT_TRUE(NormalQuantile(1.0 - 0.5 * eps_t, kappa_t));
  EXPECT_NEAR(sigma_max, (hi - lo) / (2.0 * kappa_t), 1e-12);
}

TEST(DockingRelativeState, TimingSigmaMaxOffCentreMeetsTheProbabilityExactly) {
  const double lo = 0.004;
  const double hi = 0.030;
  const double eps_t = 0.05;
  double centred = 0.0;
  ASSERT_TRUE(DockingTimingSigmaMax(lo, hi, 0.5 * (lo + hi), eps_t, centred));
  for (const double d0 : {0.008, 0.012, 0.025}) {
    double sigma_max = 0.0;
    ASSERT_TRUE(DockingTimingSigmaMax(lo, hi, d0, eps_t, sigma_max));
    const double prob = NormalCdf((hi - d0) / sigma_max) - NormalCdf((lo - d0) / sigma_max);
    EXPECT_NEAR(prob, 1.0 - eps_t, 1e-12) << d0;
    EXPECT_LT(sigma_max, centred) << "an off-centre closure tolerates less timing spread";
  }
  double out = 0.0;
  EXPECT_FALSE(DockingTimingSigmaMax(lo, hi, lo, eps_t, out)) << "δ₀ on the edge";
  EXPECT_FALSE(DockingTimingSigmaMax(lo, hi, hi + 0.001, eps_t, out));
  EXPECT_FALSE(DockingTimingSigmaMax(hi, lo, 0.01, eps_t, out));
  EXPECT_FALSE(DockingTimingSigmaMax(lo, hi, 0.01, 0.0, out));
  EXPECT_FALSE(DockingTimingSigmaMax(lo, hi, 0.01, 1.0, out));
}

// ── Impact ───────────────────────────────────────────────────────────────────

struct ImpactEval {
  DockingImpact impact;
  bool ok{false};
};

ImpactEval EvaluateImpact(const State& s, const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                          const Eigen::Vector3d& p_c_hand, double m_ball, double restitution) {
  ImpactEval out;
  const Evaluated e = Evaluate(s, q, v);
  DockingImpactWork work;
  work.Init(s.model);
  out.impact.Resize(s.model.nv);
  out.ok = ComputeDockingImpact(s.model, s.arm.frame, q, v, e.kin, s.v_b, p_c_hand, m_ball,
                                restitution, work, out.impact);
  return out;
}

TEST(DockingRelativeState, ImpactMatchesFiniteDifferencesAndAnIndependentInertia) {
  const Eigen::Vector3d p_c_hand(0.012, -0.02, 0.035);
  const double m_ball = 0.058;
  const double restitution = 0.4;
  for (const State& s : States()) {
    const Eigen::Index n = s.model.nv;
    const ImpactEval at = EvaluateImpact(s, s.q, s.v, p_c_hand, m_ball, restitution);
    ASSERT_TRUE(at.ok);

    // β_h from an inertia matrix built WITHOUT crba (RNEA columns, so the
    // armature is whatever the model's own dynamics say) and a contact
    // Jacobian built by differencing the contact point.
    pinocchio::Data data(s.model);
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n);
    const Eigen::VectorXd gravity = pinocchio::rnea(s.model, data, s.q, zero, zero);
    Eigen::MatrixXd mass(n, n);
    for (Eigen::Index j = 0; j < n; ++j) {
      mass.col(j) =
          pinocchio::rnea(s.model, data, s.q, zero, Eigen::VectorXd::Unit(n, j)) - gravity;
    }
    const auto contact_point = [&](const Eigen::VectorXd& q) {
      pinocchio::Data d(s.model);
      pinocchio::framesForwardKinematics(s.model, d, q);
      return Eigen::Vector3d(d.oMf[s.arm.frame].act(p_c_hand));
    };
    Eigen::MatrixXd j_c(3, n);
    for (Eigen::Index j = 0; j < n; ++j) {
      Eigen::VectorXd qp = s.q;
      Eigen::VectorXd qm = s.q;
      qp[j] += kFdStep;
      qm[j] -= kFdStep;
      j_c.col(j) = (contact_point(qp) - contact_point(qm)) / (2.0 * kFdStep);
    }
    const Evaluated e = Evaluate(s, s.q, s.v);
    const Eigen::Vector3d normal = e.kin.R.col(2);
    const Eigen::VectorXd f = j_c.transpose() * normal;
    const double beta_ref = f.dot(mass.ldlt().solve(f));
    EXPECT_NEAR(at.impact.beta_h, beta_ref, 1e-6 * beta_ref) << s.arm.name;
    EXPECT_NEAR(at.impact.g_n.value, normal.dot(s.v_b - j_c * s.v), 1e-7) << s.arm.name;
    EXPECT_NEAR(at.impact.m_red, 1.0 / (1.0 / m_ball + beta_ref), 1e-8);
    EXPECT_LT(at.impact.m_red, m_ball) << "m_red ≤ m_b for any model";
    const double c_n = -at.impact.g_n.value;
    EXPECT_GT(c_n, 0.5) << "the fixture's contact must be approaching";
    EXPECT_NEAR(at.impact.energy.value, 0.5 * at.impact.m_red * c_n * c_n, 1e-15);
    EXPECT_NEAR(at.impact.impulse.value, (1.0 + restitution) * at.impact.m_red * c_n, 1e-15);
    EXPECT_NEAR(at.impact.root_energy.value * at.impact.root_energy.value, at.impact.energy.value,
                1e-15);

    const auto fd_of = [&](const std::function<double(const DockingImpact&)>& pick, bool wrt_v) {
      const VecFn f_val = [&](const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
        const ImpactEval ev = EvaluateImpact(s, q, v, p_c_hand, m_ball, restitution);
        EXPECT_TRUE(ev.ok);
        return Eigen::VectorXd::Constant(1, pick(ev.impact));
      };
      return Eigen::VectorXd(Fd(s, f_val, wrt_v).row(0).transpose());
    };
    const auto check = [&](const DockingScalar& row,
                           const std::function<double(const DockingImpact&)>& pick,
                           const char* what) {
      const Eigen::VectorXd fd_q = fd_of(pick, false);
      const Eigen::VectorXd fd_v = fd_of(pick, true);
      EXPECT_LT(MaxAbs(fd_q - row.dq), kFdTol * std::max(1.0, MaxAbs(fd_q)))
          << what << " ∂/∂q on " << s.arm.name;
      EXPECT_LT(MaxAbs(fd_v - row.dv), kFdTol * std::max(1.0, MaxAbs(fd_v)))
          << what << " ∂/∂q̇ on " << s.arm.name;
    };
    check(at.impact.beta, [](const DockingImpact& i) { return i.beta.value; }, "β_h");
    check(at.impact.g_n, [](const DockingImpact& i) { return i.g_n.value; }, "g_n");
    check(at.impact.energy, [](const DockingImpact& i) { return i.energy.value; }, "E");
    check(at.impact.impulse, [](const DockingImpact& i) { return i.impulse.value; }, "P");
    check(
        at.impact.root_energy, [](const DockingImpact& i) { return i.root_energy.value; },
        "√E residual");
    EXPECT_GT(MaxAbs(at.impact.beta.dq), 1e-3) << "∂β_h/∂q is not exercised";

    // Reference §7.2: adding inertia (armature) lowers β_h, so raises m_red.
    State heavier = s;
    heavier.model.armature = Eigen::VectorXd::Constant(n, 0.5);
    const ImpactEval heavy = EvaluateImpact(heavier, s.q, s.v, p_c_hand, m_ball, restitution);
    ASSERT_TRUE(heavy.ok);
    EXPECT_LT(heavy.impact.beta_h, at.impact.beta_h);
    EXPECT_GT(heavy.impact.m_red, at.impact.m_red);
  }
}

// ── Manipulability ───────────────────────────────────────────────────────────

TEST(DockingRelativeState, ManipulabilityGradientMatchesFiniteDifferences) {
  for (const State& s : States()) {
    const Eigen::Index n = s.model.nv;
    Eigen::VectorXd d_q(n);
    for (Eigen::Index j = 0; j < n; ++j) {
      d_q[j] = 0.6 + 0.2 * static_cast<double>(j);
    }
    const double d_lin = 0.4;
    const double d_ang = 1.3;
    const double delta = 1e-3;
    const auto psi_at = [&](const Eigen::VectorXd& q, Eigen::VectorXd& grad) {
      DockingManipulabilityWork work;
      work.Init(s.model);
      double psi = 0.0;
      EXPECT_TRUE(ComputeDockingManipulability(s.model, s.arm.frame, q, d_lin, d_ang, d_q, delta,
                                               work, psi, grad));
      return psi;
    };
    Eigen::VectorXd grad(n);
    const double psi = psi_at(s.q, grad);
    Eigen::VectorXd scratch(n);
    for (Eigen::Index j = 0; j < n; ++j) {
      Eigen::VectorXd qp = s.q;
      Eigen::VectorXd qm = s.q;
      qp[j] += kFdStep;
      qm[j] -= kFdStep;
      const double fd = (psi_at(qp, scratch) - psi_at(qm, scratch)) / (2.0 * kFdStep);
      EXPECT_NEAR(grad[j], fd, kFdTol * std::max(1.0, std::abs(fd)))
          << s.arm.name << " joint " << j;
    }
    EXPECT_GT(MaxAbs(grad), 1e-2);

    // The same ψ_m from the LOCAL Jacobian, assembled independently.
    pinocchio::Data data(s.model);
    Eigen::MatrixXd j_local = Eigen::MatrixXd::Zero(6, n);
    pinocchio::computeFrameJacobian(s.model, data, s.q, s.arm.frame, pinocchio::LOCAL, j_local);
    Eigen::Matrix<double, 6, 1> dx_inv;
    dx_inv << 1 / d_lin, 1 / d_lin, 1 / d_lin, 1 / d_ang, 1 / d_ang, 1 / d_ang;
    const Eigen::MatrixXd j_bar = dx_inv.asDiagonal() * j_local * d_q.asDiagonal();
    const Eigen::Matrix<double, 6, 6> a =
        j_bar * j_bar.transpose() + delta * Eigen::Matrix<double, 6, 6>::Identity();
    EXPECT_NEAR(psi, -std::log(a.determinant()), 1e-9) << s.arm.name;
  }
}

// ── Allocation ───────────────────────────────────────────────────────────────

TEST(DockingRelativeState, SeamsAllocateNothing) {
  const State s = MovingState(fx::RealArm7(), 77);
  const Eigen::Index n = s.model.nv;
  pinocchio::Data data(s.model);
  DockingFrameKinematics kin;
  kin.Resize(n);
  DockingRelativeState rel;
  rel.Resize(n);
  DockingScalar row;
  row.Resize(n);
  DockingImpactWork impact_work;
  impact_work.Init(s.model);
  DockingImpact impact;
  impact.Resize(n);
  DockingManipulabilityWork manip_work;
  manip_work.Init(s.model);
  Eigen::VectorXd grad(n);
  const Eigen::VectorXd d_q = Eigen::VectorXd::Ones(n);
  const Eigen::VectorXd acc = Eigen::VectorXd::Constant(n, 0.3);
  const Eigen::Matrix3d sigma_p = TestSigmaP();
  const DockingCovariance6 sigma_b = TestSigmaB();
  DockingCrossing cross;
  double psi = 0.0;
  double dg = 0.0;
  Eigen::Vector3d a_rel;
  bool ok = true;

  // Positive control first: an allocation INSIDE a library is counted.
  {
    const rtc::testing::ScopedMallocGate gate;
    const pinocchio::Data probe(s.model);
    EXPECT_GT(gate.count(), 0U) << "the malloc gate does not see library allocations";
  }
  std::size_t mallocs = 0;
  std::size_t news = 0;
  {
    const rtc::testing::ScopedAllocGate new_gate;
    const rtc::testing::ScopedMallocGate gate;
    for (int rep = 0; rep < 3; ++rep) {
      ok = ok && ComputeDockingFrameKinematics(s.model, data, s.arm.frame, s.q, s.v, kin);
      ok = ok && ComputeDockingRelativeState(kin, s.p_b, s.v_b, rel);
      DockingCorridorRow(rel, 0.05, 0.04, 0.3, 0.001, row, dg);
      DockingEnvelopeRow(rel, 0.05, 0.6, 3.0, 0.0, row);
      ComputeDockingCrossing(kin, rel, sigma_p, 0.1, 1e-4, cross);
      DockingLateralChanceRow(kin, rel, sigma_p, Eigen::Vector2d(1.0, 0.0), 2.0, 0.1, 1e-4, row);
      DockingTimingRow(kin, rel, sigma_p, 0.01, 1e-4, row);
      DockingAxialSpeedRow(kin, rel, sigma_b, -2.0, 1e-4, row);
      DockingLateralSpeedRow(kin, rel, sigma_b, Eigen::Vector2d(0.0, 1.0), 2.0, 1e-4, row);
      ok = ok &&
           ComputeDockingImpact(s.model, s.arm.frame, s.q, s.v, kin, s.v_b,
                                Eigen::Vector3d(0.0, 0.0, 0.02), 0.058, 0.3, impact_work, impact);
      ok = ok && ComputeDockingManipulability(s.model, s.arm.frame, s.q, 0.5, 1.0, d_q, 1e-3,
                                              manip_work, psi, grad);
      ok = ok && DockingRelativeAcceleration(s.model, data, s.arm.frame, s.q, s.v, acc, s.p_b,
                                             s.v_b, Eigen::Vector3d(0.0, 0.0, -9.81), a_rel);
    }
    mallocs = gate.count();
    news = new_gate.count();
  }
  EXPECT_TRUE(ok);
  EXPECT_EQ(mallocs, 0U);
  EXPECT_EQ(news, 0U);
}

TEST(DockingRelativeState, SizeMismatchIsRejected) {
  const State s = MovingState(fx::RealArm6(), 1);
  pinocchio::Data data(s.model);
  DockingFrameKinematics kin;
  kin.Resize(s.model.nv + 1);
  EXPECT_FALSE(ComputeDockingFrameKinematics(s.model, data, s.arm.frame, s.q, s.v, kin));
  kin.Resize(s.model.nv);
  EXPECT_FALSE(ComputeDockingFrameKinematics(s.model, data, s.model.frames.size(), s.q, s.v, kin));
  DockingRelativeState rel;
  rel.Resize(s.model.nv - 1);
  EXPECT_FALSE(ComputeDockingRelativeState(kin, s.p_b, s.v_b, rel));
}

// ── Derivatives with respect to the ball (E1-F14 PR 2, #740) ─────────────────
// A catch instant that is a variable moves the ball the catch node's rows
// read. Each gradient in (p_b, v_b) is checked against central differences of
// the row's VALUE with the robot state held fixed — and its rate along a
// prediction against the value's difference along that prediction.

using BallFn =
    std::function<Eigen::VectorXd(const Eigen::Vector3d& p_b, const Eigen::Vector3d& v_b)>;

// Central difference of f in p_b (wrt_v = false) or v_b: rows × 3.
Eigen::MatrixXd FdBall(const State& s, const BallFn& f, bool wrt_v) {
  const Eigen::VectorXd f0 = f(s.p_b, s.v_b);
  Eigen::MatrixXd jac(f0.size(), 3);
  for (Eigen::Index i = 0; i < 3; ++i) {
    Eigen::Vector3d pp = s.p_b;
    Eigen::Vector3d pm = s.p_b;
    Eigen::Vector3d vp = s.v_b;
    Eigen::Vector3d vm = s.v_b;
    (wrt_v ? vp : pp)[i] += kFdStep;
    (wrt_v ? vm : pm)[i] -= kFdStep;
    jac.col(i) = (f(pp, vp) - f(pm, vm)) / (2.0 * kFdStep);
  }
  return jac;
}

Evaluated EvaluateWithBall(const State& s, const Eigen::Vector3d& p_b, const Eigen::Vector3d& v_b) {
  State moved = s;
  moved.p_b = p_b;
  moved.v_b = v_b;
  return Evaluate(moved, s.q, s.v);
}

TEST(DockingRelativeState, RelativeStateMatchesFiniteDifferencesInTheBall) {
  for (const State& s : States()) {
    const Evaluated at = Evaluate(s, s.q, s.v);
    const BallFn r_h = [&](const Eigen::Vector3d& p, const Eigen::Vector3d& v) {
      return Eigen::VectorXd(EvaluateWithBall(s, p, v).rel.r_h);
    };
    const BallFn nu_h = [&](const Eigen::Vector3d& p, const Eigen::Vector3d& v) {
      return Eigen::VectorXd(EvaluateWithBall(s, p, v).rel.nu_h);
    };
    const Eigen::MatrixXd fd_nu_p = FdBall(s, nu_h, false);
    EXPECT_LT(MaxAbs(FdBall(s, r_h, false) - at.rel.dr_dpb), kFdTol) << s.arm.name;
    EXPECT_LT(MaxAbs(FdBall(s, r_h, true)), 1e-9) << s.arm.name << " r^H does not read v_b";
    EXPECT_LT(MaxAbs(fd_nu_p - at.rel.dnu_dpb), kFdTol) << s.arm.name;
    EXPECT_LT(MaxAbs(FdBall(s, nu_h, true) - at.rel.dnu_dvb), kFdTol) << s.arm.name;
    // The transport term is what a rotating hand adds: without it the
    // derivative in p_b would be zero, and here it is not.
    EXPECT_GT(MaxAbs(fd_nu_p), 0.2) << s.arm.name << ": the fixture's hand must rotate";
  }
}

// A catch-node row and, when asked, its ball gradient.
using BallRowFn = std::function<void(const Evaluated&, DockingScalar&, DockingBallGradient*)>;

void ExpectBallGradient(const State& s, const BallRowFn& row, const char* what, bool reads_p,
                        bool reads_v) {
  const Eigen::Index n = s.model.nv;
  const Evaluated at = Evaluate(s, s.q, s.v);
  DockingScalar with;
  DockingScalar without;
  with.Resize(n);
  without.Resize(n);
  DockingBallGradient g;
  row(at, with, &g);
  row(at, without, nullptr);
  // Asking for the ball gradient changes nothing else, to the bit.
  EXPECT_EQ(with.value, without.value) << what;
  EXPECT_EQ(with.dq, without.dq) << what;
  EXPECT_EQ(with.dv, without.dv) << what;
  const BallFn value = [&](const Eigen::Vector3d& p, const Eigen::Vector3d& v) {
    DockingScalar out;
    out.Resize(n);
    row(EvaluateWithBall(s, p, v), out, nullptr);
    return Eigen::VectorXd::Constant(1, out.value);
  };
  const Eigen::MatrixXd fd_p = FdBall(s, value, false);
  const Eigen::MatrixXd fd_v = FdBall(s, value, true);
  EXPECT_LT(MaxAbs(fd_p.row(0).transpose() - g.dp), kFdTol * std::max(1.0, MaxAbs(fd_p)))
      << what << " ∂/∂p_b on " << s.arm.name;
  EXPECT_LT(MaxAbs(fd_v.row(0).transpose() - g.dv), kFdTol * std::max(1.0, MaxAbs(fd_v)))
      << what << " ∂/∂v_b on " << s.arm.name;
  // Two orders above the tolerance the gradients are held to: a dropped term
  // of this size cannot pass.
  if (reads_p) {
    EXPECT_GT(MaxAbs(fd_p), 1e-4) << what << ": the fixture does not exercise ∂/∂p_b";
  }
  if (reads_v) {
    EXPECT_GT(MaxAbs(fd_v), 1e-4) << what << ": the fixture does not exercise ∂/∂v_b";
  }
  // Along a prediction — the ball accelerating, and not only by gravity — the
  // row moves at Rate(v_b, a_b).
  const Eigen::Vector3d a_b(0.7, -1.1, -9.81);
  const auto along = [&](double dt) {
    return value(s.p_b + s.v_b * dt + 0.5 * dt * dt * a_b, s.v_b + a_b * dt)[0];
  };
  const double fd_rate = (along(kFdStep) - along(-kFdStep)) / (2.0 * kFdStep);
  EXPECT_NEAR(g.Rate(s.v_b, a_b), fd_rate, kFdTol * std::max(1.0, std::abs(fd_rate)))
      << what << " rate on " << s.arm.name;
  EXPECT_GT(std::abs(fd_rate), 1e-3) << what << ": the fixture's row does not move in time";
  // The acceleration's part is there: with a_b = 0 the rate is another number.
  if (reads_v) {
    EXPECT_GT(std::abs(g.Rate(s.v_b, a_b) - g.Rate(s.v_b, Eigen::Vector3d::Zero())), 1e-4) << what;
  }
}

TEST(DockingRelativeState, CatchNodeRowsMatchFiniteDifferencesInTheBall) {
  const Eigen::Matrix3d sigma_p = TestSigmaP();
  const DockingCovariance6 sigma_b = TestSigmaB();
  const double eps = 1e-4;
  for (const State& s : States()) {
    const double c_now = Evaluate(s, s.q, s.v).rel.c;
    for (const double c_min : {0.3 * c_now, 1.5 * c_now}) {
      for (const Eigen::Vector2d& a : {Eigen::Vector2d(1.0, 0.0), Eigen::Vector2d(-0.6, 0.8)}) {
        const BallRowFn lateral = [&](const Evaluated& e, DockingScalar& out,
                                      DockingBallGradient* ball) {
          DockingLateralChanceRow(e.kin, e.rel, sigma_p, a, 2.3, c_min, eps, out, ball);
        };
        ExpectBallGradient(s, lateral, c_min < c_now ? "lateral chance" : "lateral chance (c̃)",
                           true, true);
      }
    }
    const BallRowFn timing = [&](const Evaluated& e, DockingScalar& out,
                                 DockingBallGradient* ball) {
      DockingTimingRow(e.kin, e.rel, sigma_p, 0.012, eps, out, ball);
    };
    // k_t is a time: the row's gradients are of its size.
    {
      const Eigen::Index n = s.model.nv;
      DockingScalar out;
      out.Resize(n);
      DockingBallGradient g;
      timing(Evaluate(s, s.q, s.v), out, &g);
      const BallFn value = [&](const Eigen::Vector3d& p, const Eigen::Vector3d& v) {
        DockingScalar o;
        o.Resize(n);
        timing(EvaluateWithBall(s, p, v), o, nullptr);
        return Eigen::VectorXd::Constant(1, o.value);
      };
      EXPECT_LT(MaxAbs(FdBall(s, value, false).row(0).transpose() - g.dp), kFdTol) << s.arm.name;
      EXPECT_LT(MaxAbs(FdBall(s, value, true).row(0).transpose() - g.dv), kFdTol) << s.arm.name;
      EXPECT_NEAR(g.dv.norm(), 0.012, 1e-12) << "∂/∂v_b of c·k_t is −k_t R e₃";
    }
    for (const double kappa : {-2.1, 2.1}) {
      const BallRowFn axial = [&](const Evaluated& e, DockingScalar& out,
                                  DockingBallGradient* ball) {
        DockingAxialSpeedRow(e.kin, e.rel, sigma_b, kappa, eps, out, ball);
      };
      ExpectBallGradient(s, axial, "axial speed", true, true);
    }
    const BallRowFn face = [&](const Evaluated& e, DockingScalar& out, DockingBallGradient* ball) {
      DockingLateralSpeedRow(e.kin, e.rel, sigma_b, Eigen::Vector2d(0.5, std::sqrt(0.75)), 1.9, eps,
                             out, ball);
    };
    ExpectBallGradient(s, face, "lateral speed", true, true);
    // The entrance plane and the terminal cost read r^H and ν^H themselves.
    const BallRowFn gap = [&](const Evaluated& e, DockingScalar& out, DockingBallGradient* ball) {
      out.value = e.rel.s;
      if (ball != nullptr) {
        *ball = DockingBallGradientOf(e.rel, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero());
      }
    };
    ExpectBallGradient(s, gap, "entrance gap", true, false);
  }
}

TEST(DockingRelativeState, ImpactMatchesFiniteDifferencesInTheBallsVelocity) {
  const Eigen::Vector3d p_c_hand(0.012, -0.02, 0.035);
  const double m_ball = 0.058;
  const double restitution = 0.4;
  for (const State& s : States()) {
    const ImpactEval at = EvaluateImpact(s, s.q, s.v, p_c_hand, m_ball, restitution);
    ASSERT_TRUE(at.ok);
    const BallFn values = [&](const Eigen::Vector3d&, const Eigen::Vector3d& v) {
      State moved = s;
      moved.v_b = v;
      const ImpactEval e = EvaluateImpact(moved, s.q, s.v, p_c_hand, m_ball, restitution);
      EXPECT_TRUE(e.ok);
      Eigen::VectorXd out(4);
      out << e.impact.g_n.value, e.impact.energy.value, e.impact.impulse.value,
          e.impact.root_energy.value;
      return out;
    };
    const Eigen::MatrixXd fd = FdBall(s, values, true);
    EXPECT_LT(MaxAbs(fd.row(0).transpose() - at.impact.g_n_dvb), kFdTol) << s.arm.name;
    EXPECT_LT(MaxAbs(fd.row(1).transpose() - at.impact.energy_dvb), kFdTol) << s.arm.name;
    EXPECT_LT(MaxAbs(fd.row(2).transpose() - at.impact.impulse_dvb), kFdTol) << s.arm.name;
    EXPECT_LT(MaxAbs(fd.row(3).transpose() - at.impact.root_energy_dvb), kFdTol) << s.arm.name;
    EXPECT_NEAR(at.impact.g_n_dvb.norm(), 1.0, 1e-12);
    EXPECT_GT(MaxAbs(fd.row(1)), 1e-3) << "the fixture's ball must hit";
    EXPECT_GT(MaxAbs(fd.row(3)), 1e-3);
  }
}

}  // namespace
