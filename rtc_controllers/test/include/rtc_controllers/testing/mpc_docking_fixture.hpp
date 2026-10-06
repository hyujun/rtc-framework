// ── mpc_docking core test fixture (test-only, E1-F13 #739) ─────────────────────
// Two things the docking core's suite needs and must not take from the core:
//
//  1. Constructed-feasible throws. A trajectory inside every limit is built
//     first (per joint, the minimum-norm block jerk that reaches a chosen
//     catch-node state and stops), the ball is then placed BACKWARDS from it —
//     on the entrance plane at the catch node, with a relative velocity inside
//     the tightened capture set — and the case is accepted only when the
//     checker below finds every hard row satisfied on that trajectory. So a
//     feasible point of the NLP is known to exist; the solver is never asked
//     to find one on faith.
//
//  2. An independent re-evaluation of the hard rows. HardRowViolations takes
//     node values and recomputes each row with pinocchio directly, through the
//     LOCAL Jacobian route and explicit Π / L matrices — none of the seams in
//     mpc_docking_relative_state.hpp, which are what the core calls.
//
// Test-only, NEVER installed (see mpc_segment_core_fixture.hpp).
#pragma once

#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"
#include "rtc_controllers/testing/mpc_segment_core_fixture.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <numbers>
#include <random>
#include <string>
#include <vector>

namespace rtc::testing::mpc_docking {

namespace mc = rtc::testing::mpc_segment_core;
using rtc::catching::BallNodeSample;
using rtc::catching::DockingRowGroup;
using rtc::catching::kNumDockingRowGroups;
using rtc::catching::MpcDockingSegmentCore;
using rtc::catching::MpcDockingSegmentCoreInput;
using rtc::catching::MpcDockingSegmentCoreLimits;
using rtc::catching::MpcDockingSegmentCoreParams;

inline const Eigen::Vector3d kGravity(0.0, 0.0, -9.81);

struct Rig {
  mc::ArmModel arm;
  pinocchio::Model model;  // arm + armature: what the core computes on
  MpcDockingSegmentCoreLimits limits;
  MpcDockingSegmentCoreParams params;
};

inline MpcDockingSegmentCoreLimits LimitsFromModel(const pinocchio::Model& m, double armature) {
  MpcDockingSegmentCoreLimits lim;
  lim.q_min = m.lowerPositionLimit;
  lim.q_max = m.upperPositionLimit;
  lim.qd_max = m.velocityLimit;
  lim.tau_max = m.effortLimit;
  lim.armature = Eigen::VectorXd::Constant(m.nv, armature);
  return lim;
}

// A grid with a 0.3 s approach (6 × 0.05 s) and a 0.4 s stop; every cost term
// of the reference on, with weights of comparable size; a capture set a
// tennis-ball-sized object would have.
inline MpcDockingSegmentCoreParams BaseParams(int n) {
  MpcDockingSegmentCoreParams p;
  p.n_pre = 6;
  p.dt_pre = 0.05;
  p.n_stop = 8;
  p.dt_stop = 0.05;
  p.n_blocks = 11;
  p.block_sizes = {1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2};
  p.u_scale = 1e3;
  p.r_tau = Eigen::VectorXd::Constant(n, 1e-4);
  p.r_acc = Eigen::VectorXd::Constant(n, 1e-3);
  p.r_jerk = Eigen::VectorXd::Constant(n, 1.0);
  p.w_q_nom = Eigen::VectorXd::Constant(n, 0.1);
  p.q_p = Eigen::Vector3d(20.0, 20.0, 20.0);
  p.q_v = Eigen::Vector3d(1.0, 1.0, 1.0);
  p.sigma_T = 0.1;
  p.nu_ref = Eigen::Vector3d(0.0, 0.0, -0.8);
  p.q_rho_f = Eigen::Vector2d(1e3, 1e3);
  p.q_nu_f = Eigen::Vector3d(5.0, 5.0, 5.0);
  p.approach_window = 0.12;
  p.s_ent = 0.05;
  p.r_ent = 0.04;
  p.tan_theta = 0.4;
  p.c_ent_max = 1.5;
  p.a_brake = 5.0;
  p.c_min = 0.2;
  p.c_cap_max = 1.5;
  p.v_perp_max = 0.5;
  p.speed_faces = 8;
  p.eps_nu = 0.01;
  p.eps_sigma = 1e-4;
  p.delta_lo = 0.0;
  p.delta_hi = 0.03;
  p.delta_0 = 0.015;
  p.eps_t = 0.05;
  p.sigma_tau = 0.002;
  p.max_iterations = 50;
  p.tol_violation = 1e-6;
  return p;
}

inline Rig MakeRig(const mc::ArmModel& arm) {
  Rig r;
  r.arm = arm;
  const double armature = 0.05;
  r.model = *arm.model;
  r.model.armature = arm.model->armature + Eigen::VectorXd::Constant(arm.model->nv, armature);
  r.limits = LimitsFromModel(*arm.model, armature);
  r.params = BaseParams(arm.model->nv);
  r.params.q_nom = arm.q_nominal;
  return r;
}

struct Nodes {
  Eigen::MatrixXd q, qd, qdd;  // n × (N+1)
};

// The trajectory of scaled block jerks z from x0, through the core's own
// stage-gain accessor (which a separate test pins against MpcSegmentCore).
inline Nodes NodesFromJerk(const MpcDockingSegmentCore& core, const Eigen::VectorXd& q0,
                           const Eigen::VectorXd& qd0, const Eigen::VectorXd& qdd0,
                           const Eigen::VectorXd& z) {
  const Eigen::Index n = core.Nv();
  const Eigen::Index N1 = core.NumNodes() + 1;
  Nodes out;
  out.q.resize(n, N1);
  out.qd.resize(n, N1);
  out.qdd.resize(n, N1);
  for (Eigen::Index k = 0; k < N1; ++k) {
    const double t = core.NodeTime(static_cast<int>(k));
    for (Eigen::Index j = 0; j < n; ++j) {
      double q = q0[j] + t * qd0[j] + 0.5 * t * t * qdd0[j];
      double v = qd0[j] + t * qdd0[j];
      double a = qdd0[j];
      for (int b = 0; b < core.NumBlocks(); ++b) {
        const double zb = z[b * n + j];
        q += core.StageGain(0, static_cast<int>(k), b) * zb;
        v += core.StageGain(1, static_cast<int>(k), b) * zb;
        a += core.StageGain(2, static_cast<int>(k), b) * zb;
      }
      out.q(j, k) = q;
      out.qd(j, k) = v;
      out.qdd(j, k) = a;
    }
  }
  return out;
}

// Minimum-norm block jerk from rest at q0 to (q_c, v_c) at the catch node and
// to rest at node N — four equations per joint.
inline Eigen::VectorXd JerkThrough(const MpcDockingSegmentCore& core, const Eigen::VectorXd& q0,
                                   const Eigen::VectorXd& q_c, const Eigen::VectorXd& v_c) {
  const Eigen::Index n = core.Nv();
  const int nb = core.NumBlocks();
  const int kc = core.CatchNode();
  const int N = core.NumNodes();
  Eigen::MatrixXd m(4, nb);
  for (int b = 0; b < nb; ++b) {
    m(0, b) = core.StageGain(0, kc, b);
    m(1, b) = core.StageGain(1, kc, b);
    m(2, b) = core.StageGain(1, N, b);
    m(3, b) = core.StageGain(2, N, b);
  }
  const Eigen::MatrixXd pinv = m.transpose() * (m * m.transpose()).inverse();
  Eigen::VectorXd z(n * nb);
  for (Eigen::Index j = 0; j < n; ++j) {
    const Eigen::Vector4d rhs(q_c[j] - q0[j], v_c[j], 0.0, 0.0);
    const Eigen::VectorXd zj = pinv * rhs;
    for (int b = 0; b < nb; ++b) {
      z[b * n + j] = zj[b];
    }
  }
  return z;
}

struct HandState {
  Eigen::Vector3d p, v_local, w_local;
  Eigen::Matrix3d R;
};

// Pose and LOCAL velocity of the capture frame — the reference's §5.2 route,
// independent of the LOCAL_WORLD_ALIGNED one the core uses.
inline HandState HandAt(const Rig& rig, const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  pinocchio::Data data(rig.model);
  pinocchio::forwardKinematics(rig.model, data, q, v);
  pinocchio::updateFramePlacement(rig.model, data, rig.arm.frame);
  const pinocchio::Motion vel =
      pinocchio::getFrameVelocity(rig.model, data, rig.arm.frame, pinocchio::LOCAL);
  HandState h;
  h.p = data.oMf[rig.arm.frame].translation();
  h.R = data.oMf[rig.arm.frame].rotation();
  h.v_local = vel.linear();
  h.w_local = vel.angular();
  return h;
}

struct Relative {
  Eigen::Vector3d r_h, nu_h;
  double s, c;
};

inline Relative RelativeAt(const HandState& h, const Eigen::Vector3d& p_b,
                           const Eigen::Vector3d& v_b) {
  Relative r;
  r.r_h = h.R.transpose() * (p_b - h.p);
  r.nu_h = h.R.transpose() * v_b - h.v_local - h.w_local.cross(r.r_h);
  r.s = r.r_h.z();
  r.c = -r.nu_h.z();
  return r;
}

inline Eigen::Matrix3d Hat(const Eigen::Vector3d& a) {
  Eigen::Matrix3d m;
  m << 0.0, -a.z(), a.y(), a.z(), 0.0, -a.x(), -a.y(), a.x(), 0.0;
  return m;
}

struct Throw {
  Eigen::VectorXd q0;
  Eigen::VectorXd q_c, v_c;  // the known trajectory's catch-node state
  Eigen::Vector3d p_b, v_b;  // the ball at the catch instant
  rtc::catching::BallCovariance cov;
  Nodes known;  // the known feasible trajectory
};

// Fill the core's ball input from a ballistic ball that is at (p_b, v_b) at
// the catch instant; the covariance goes to the catch node only.
inline void FillBall(const MpcDockingSegmentCore& core, const Eigen::Vector3d& p_b,
                     const Eigen::Vector3d& v_b, const rtc::catching::BallCovariance& cov,
                     MpcDockingSegmentCoreInput& in) {
  const int kc = core.CatchNode();
  const double t_c = core.NodeTime(kc);
  for (int k = 0; k <= kc; ++k) {
    const double tau = core.NodeTime(k) - t_c;
    BallNodeSample& b = in.ball[static_cast<std::size_t>(k)];
    b.p = p_b + v_b * tau + 0.5 * tau * tau * kGravity;
    b.v = v_b + kGravity * tau;
    b.a = kGravity;
    b.valid = true;
    b.cov_valid = false;
  }
  in.ball[static_cast<std::size_t>(kc)].cov = cov;
  in.ball[static_cast<std::size_t>(kc)].cov_valid = true;
}

// The worst row of each hard-row group on `nodes`, recomputed from scratch
// (file header), SIGNED: positive = violated by that much, negative = the
// smallest margin, −inf = the group has no rows. Units as the core's: torque
// as a fraction of τ_max, lengths in m, speeds in m/s; box and terminal in raw
// units. The two equality groups (entrance, terminal) are |residual| ≥ 0.
inline std::array<double, kNumDockingRowGroups> HardRowViolations(
    const Rig& rig, const MpcDockingSegmentCore& core, const Nodes& nodes,
    const MpcDockingSegmentCoreInput& in) {
  const MpcDockingSegmentCoreParams& p = rig.params;
  std::array<double, kNumDockingRowGroups> viol{};
  viol.fill(-std::numeric_limits<double>::infinity());
  const auto note = [&viol](DockingRowGroup g, double v) {
    const auto i = static_cast<std::size_t>(g);
    viol[i] = std::max(viol[i], v);
  };
  const Eigen::Index n = rig.model.nv;
  const int N = core.NumNodes();
  const int kc = core.CatchNode();
  pinocchio::Data data(rig.model);
  const Eigen::VectorXd tau_lo =
      rig.limits.tau_lo.size() == 0 ? Eigen::VectorXd(-rig.limits.tau_max) : rig.limits.tau_lo;
  const Eigen::VectorXd tau_hi =
      rig.limits.tau_hi.size() == 0 ? rig.limits.tau_max : rig.limits.tau_hi;
  for (int k = 1; k <= N; ++k) {
    const Eigen::VectorXd tau =
        pinocchio::rnea(rig.model, data, nodes.q.col(k), nodes.qd.col(k), nodes.qdd.col(k));
    for (Eigen::Index j = 0; j < n; ++j) {
      note(DockingRowGroup::kTorque, (tau[j] - tau_hi[j]) / rig.limits.tau_max[j]);
      note(DockingRowGroup::kTorque, (tau_lo[j] - tau[j]) / rig.limits.tau_max[j]);
      note(DockingRowGroup::kBox, rig.limits.q_min[j] - nodes.q(j, k));
      note(DockingRowGroup::kBox, nodes.q(j, k) - rig.limits.q_max[j]);
      note(DockingRowGroup::kBox, std::abs(nodes.qd(j, k)) - rig.limits.qd_max[j]);
      if (p.accel_box) {
        note(DockingRowGroup::kBox, std::abs(nodes.qdd(j, k)) - rig.limits.qdd_max[j]);
      }
    }
  }
  for (Eigen::Index j = 0; j < n; ++j) {
    note(DockingRowGroup::kTerminal, std::abs(nodes.qd(j, N)));
    note(DockingRowGroup::kTerminal, std::abs(nodes.qdd(j, N)));
  }
  for (int i = 0; i < core.NumApproachNodes(); ++i) {
    const int k = core.ApproachNode(i);
    const BallNodeSample& b = in.ball[static_cast<std::size_t>(k)];
    const Relative rel = RelativeAt(HandAt(rig, nodes.q.col(k), nodes.qd.col(k)), b.p, b.v);
    note(DockingRowGroup::kGap, -(rel.s - p.s_ent));
  }
  // ── Catch node ──
  const BallNodeSample& ball = in.ball[static_cast<std::size_t>(kc)];
  const HandState h = HandAt(rig, nodes.q.col(kc), nodes.qd.col(kc));
  const Relative rel = RelativeAt(h, ball.p, ball.v);
  note(DockingRowGroup::kEntrance, std::abs(rel.s - p.s_ent));
  const rtc::catching::BallCovariance cov =
      p.chance ? ball.cov : rtc::catching::BallCovariance(rtc::catching::BallCovariance::Zero());
  const double eps2 = p.eps_sigma * p.eps_sigma;
  // Σ_ρ = E⊥ᵀ Π Σ^H_r Πᵀ E⊥ with the explicit oblique projector.
  const Eigen::Matrix3d sigma_r_h = h.R.transpose() * cov.topLeftCorner<3, 3>() * h.R;
  Eigen::Matrix3d pi = Eigen::Matrix3d::Identity();
  pi += rel.nu_h * Eigen::Vector3d::UnitZ().transpose() / std::max(rel.c, p.c_min);
  const Eigen::Matrix2d sigma_rho = (pi * sigma_r_h * pi.transpose()).topLeftCorner<2, 2>();
  for (int i = 0; i < p.n_faces; ++i) {
    const auto fi = static_cast<std::size_t>(i);
    const Eigen::Vector2d& a = p.face_a[fi];
    const double row =
        a.dot(rel.r_h.head<2>()) + core.FaceKappa(i) * std::sqrt(a.dot(sigma_rho * a) + eps2);
    note(DockingRowGroup::kLateral, row - p.face_b[fi]);
  }
  if (p.chance && p.timing_row) {
    const double sigma_s = std::sqrt(sigma_r_h(2, 2) + eps2);
    const double k_t =
        std::sqrt(core.TimingSigmaMax() * core.TimingSigmaMax() - p.sigma_tau * p.sigma_tau);
    note(DockingRowGroup::kTiming, -(rel.c * k_t - sigma_s));
  }
  // Σ^H_ν = L Σ_b Lᵀ, L = [−[ω^H]× Rᵀ, Rᵀ] (reference §8.1).
  Eigen::Matrix<double, 3, 6> l;
  l.leftCols<3>() = -Hat(h.w_local) * h.R.transpose();
  l.rightCols<3>() = h.R.transpose();
  const Eigen::Matrix3d sigma_nu = l * cov * l.transpose();
  const double kappa_nu = core.VelocityKappa();
  const double sigma_c = std::sqrt(sigma_nu(2, 2) + eps2);
  note(DockingRowGroup::kVelocitySet, p.c_min - (rel.c - kappa_nu * sigma_c));
  note(DockingRowGroup::kVelocitySet, (rel.c + kappa_nu * sigma_c) - p.c_cap_max);
  for (int j = 0; j < p.speed_faces; ++j) {
    const double angle = 2.0 * std::numbers::pi * j / p.speed_faces;
    const Eigen::Vector3d u(std::cos(angle), std::sin(angle), 0.0);
    const double row = u.dot(rel.nu_h) + kappa_nu * std::sqrt(u.dot(sigma_nu * u) + eps2);
    note(DockingRowGroup::kVelocitySet,
         row - p.v_perp_max * std::cos(std::numbers::pi / p.speed_faces));
  }
  return viol;
}

inline double MaxViolation(const std::array<double, kNumDockingRowGroups>& v) {
  double m = -std::numeric_limits<double>::infinity();
  for (const double x : v) {
    m = std::max(m, x);
  }
  return m;
}

// The worst INEQUALITY row (every group but the two equalities).
inline double WorstInequality(const std::array<double, kNumDockingRowGroups>& v) {
  double m = -std::numeric_limits<double>::infinity();
  for (std::size_t g = 0; g < v.size(); ++g) {
    if (g != static_cast<std::size_t>(DockingRowGroup::kEntrance) &&
        g != static_cast<std::size_t>(DockingRowGroup::kTerminal)) {
      m = std::max(m, v[g]);
    }
  }
  return m;
}

// A covariance small enough for the base capture set: ~2 mm position,
// ~3 cm/s velocity, correlated and rotated (never axis-aligned).
inline rtc::catching::BallCovariance SmallCovariance(std::mt19937& gen) {
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  Eigen::Matrix<double, 6, 6> a;
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      a(r, c) = uni(gen) * (r < 3 ? 8e-4 : 1.2e-2);
    }
  }
  const rtc::catching::BallCovariance s = a * a.transpose();
  return 0.5 * (s + s.transpose());
}

// One candidate throw from `seed`; `ok` says whether the known trajectory
// satisfies every hard row with `margin` to spare.
inline bool MakeThrow(const Rig& rig, const MpcDockingSegmentCore& core, unsigned seed,
                      double margin, Throw& out, MpcDockingSegmentCoreInput& in) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  const Eigen::Index n = rig.model.nv;
  const MpcDockingSegmentCoreParams& p = rig.params;
  out.q0 = rig.arm.q_nominal;
  out.q_c.resize(n);
  out.v_c.resize(n);
  for (Eigen::Index j = 0; j < n; ++j) {
    out.q0[j] += 0.1 * uni(gen);
    out.q_c[j] = out.q0[j] + 0.25 * uni(gen);
    out.v_c[j] = 0.5 * uni(gen);
  }
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n);
  const Eigen::VectorXd z = JerkThrough(core, out.q0, out.q_c, out.v_c);
  out.known = NodesFromJerk(core, out.q0, zero, zero, z);

  const int kc = core.CatchNode();
  const HandState h = HandAt(rig, out.known.q.col(kc), out.known.qd.col(kc));
  // On the entrance plane, a little off the axis; closing at 0.6–1.0 m/s with
  // a small lateral part.
  const Eigen::Vector3d r_h(0.008 * uni(gen), 0.008 * uni(gen), p.s_ent);
  const Eigen::Vector3d nu_h(0.08 * uni(gen), 0.08 * uni(gen), -(0.8 + 0.2 * uni(gen)));
  out.p_b = h.p + h.R * r_h;
  out.v_b = h.R * (nu_h + h.v_local + h.w_local.cross(r_h));
  out.cov = SmallCovariance(gen);

  core.ResizeInput(in);
  in.q0 = out.q0;
  in.initial_valid = false;
  in.catch_target_valid = true;
  in.q_catch_target = out.q_c;
  FillBall(core, out.p_b, out.v_b, out.cov, in);
  const std::array<double, kNumDockingRowGroups> viol = HardRowViolations(rig, core, out.known, in);
  return WorstInequality(viol) <= -margin &&
         viol[static_cast<std::size_t>(DockingRowGroup::kEntrance)] <= 1e-9 &&
         viol[static_cast<std::size_t>(DockingRowGroup::kTerminal)] <= 1e-9;
}

// The search's IK solution is near, not at, the pose the docking problem ends
// up at: perturb the catch-node target by up to ±0.15 rad per joint.
inline void PerturbTarget(MpcDockingSegmentCoreInput& in, unsigned seed) {
  std::mt19937 gen(seed * 7919U + 13U);
  std::uniform_real_distribution<double> uni(-0.15, 0.15);
  for (Eigen::Index j = 0; j < in.q_catch_target.size(); ++j) {
    in.q_catch_target[j] += uni(gen);
  }
}

// ── Recorded conditions (no verdict) ─────────────────────────────────────────
// How a batch of solves ended. Used where the suite RECORDS a condition
// instead of judging it — a short lead, a real throw — on the plain arms here
// and on the shipped hand-locked sub-models in integrated_bringup.

struct SolveTally {
  int cases{0};
  int rejected{0};              // Solve() returned false: no iterate, nothing else tallied
  int constructed_feasible{0};  // the fixture's trajectory satisfied every hard row
  int converged{0};
  int feasible{0};
  int infeasible{0};
  int iteration_limit{0};
  int line_search_failed{0};
  int qp_failed{0};
  int iterations{0};
  int backtracks{0};
  int mu_updates{0};
  double worst_violation{0.0};  // over the solves that are NOT feasible: how far off
  std::array<int, rtc::catching::kNumDockingElasticGroups> residual_group{};
  std::vector<double> total_us;
};

inline void Tally(const rtc::catching::MpcDockingSegmentCoreResult& out, bool solved,
                  SolveTally& t) {
  using rtc::catching::MpcDockingReason;
  ++t.cases;
  if (!solved) {
    // A rejected call has a reason and nothing else: it is not a solve that
    // ended infeasible in some row group.
    ++t.rejected;
    return;
  }
  t.converged += out.converged ? 1 : 0;
  t.feasible += out.feasible ? 1 : 0;
  t.infeasible += out.reason == MpcDockingReason::kInfeasible ? 1 : 0;
  t.iteration_limit += out.reason == MpcDockingReason::kIterationLimit ? 1 : 0;
  t.line_search_failed += out.reason == MpcDockingReason::kLineSearchFailed ? 1 : 0;
  t.qp_failed += (out.reason == MpcDockingReason::kQpFailed ||
                  out.reason == MpcDockingReason::kSolutionNonFinite)
                     ? 1
                     : 0;
  t.iterations += out.iterations;
  t.backtracks += out.backtracks;
  t.mu_updates += out.mu_updates;
  if (!out.feasible) {
    ++t.residual_group[static_cast<std::size_t>(out.infeasible_group)];
    for (const double v : out.violation) {
      t.worst_violation = std::max(t.worst_violation, v);
    }
  }
  t.total_us.push_back(out.total_us);
}

inline void RecordTally(const std::string& tag, const SolveTally& t) {
  const auto rec = [&tag](const char* key, int value) {
    ::testing::Test::RecordProperty(tag + "_" + key, value);
  };
  rec("cases", t.cases);
  rec("rejected", t.rejected);
  rec("constructed_feasible", t.constructed_feasible);
  rec("converged", t.converged);
  rec("feasible", t.feasible);
  rec("reason_infeasible", t.infeasible);
  rec("reason_iteration_limit", t.iteration_limit);
  rec("reason_line_search_failed", t.line_search_failed);
  rec("reason_qp_failed", t.qp_failed);
  rec("iterations_total", t.iterations);
  rec("backtracks_total", t.backtracks);
  rec("mu_updates_total", t.mu_updates);
  for (int g = 0; g < rtc::catching::kNumDockingElasticGroups; ++g) {
    const int hits = t.residual_group[static_cast<std::size_t>(g)];
    if (hits > 0) {
      ::testing::Test::RecordProperty(
          tag + "_residual_" + rtc::catching::DockingRowGroupName(static_cast<DockingRowGroup>(g)),
          hits);
    }
  }
  // RecordProperty(double) goes through to_string (6 fixed decimals).
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.4e", t.worst_violation);
  ::testing::Test::RecordProperty(tag + "_worst_violation_of_infeasible", buf);
  mc::RecordMicros(tag + "_total_us_p50", mc::Percentile(t.total_us, 0.5));
  mc::RecordMicros(tag + "_total_us_max", mc::Percentile(t.total_us, 1.0));
}

// `cases` throws from the fixture's generator on this rig, each solved from a
// perturbed catch-pose target. The generator's own trajectory need not be
// feasible here (that is counted, not required).
inline SolveTally SolveGeneratedThrows(const Rig& rig, int cases) {
  SolveTally tally;
  MpcDockingSegmentCore core;
  const auto why = core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, nullptr);
  EXPECT_EQ(why, rtc::catching::MpcDockingReason::kNone)
      << rtc::catching::MpcDockingReasonName(why);
  if (why != rtc::catching::MpcDockingReason::kNone) {
    return tally;
  }
  rtc::catching::MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  for (unsigned seed = 1; tally.cases < cases; ++seed) {
    Throw th;
    MpcDockingSegmentCoreInput in;
    tally.constructed_feasible += MakeThrow(rig, core, seed, 0.0, th, in) ? 1 : 0;
    PerturbTarget(in, seed);
    const bool solved = core.Solve(in, out);
    EXPECT_TRUE(solved) << rtc::catching::MpcDockingReasonName(out.reason);
    Tally(out, solved, tally);
  }
  return tally;
}

// A real throw: the ball comes down the capture axis of the rig's nominal pose
// at `speed` [m/s] in the WORLD, the hand starts at rest there.
inline SolveTally SolveRealThrows(const Rig& rig, double speed_lo, double speed_hi, int cases) {
  SolveTally tally;
  MpcDockingSegmentCore core;
  const auto why = core.Init(rig.model, rig.arm.frame, rig.params, rig.limits, nullptr);
  EXPECT_EQ(why, rtc::catching::MpcDockingReason::kNone)
      << rtc::catching::MpcDockingReasonName(why);
  if (why != rtc::catching::MpcDockingReason::kNone) {
    return tally;
  }
  rtc::catching::MpcDockingSegmentCoreResult out;
  core.ResizeResult(out);
  const Eigen::Index n = rig.model.nv;
  const HandState h = HandAt(rig, rig.arm.q_nominal, Eigen::VectorXd::Zero(n));
  for (int i = 0; i < cases; ++i) {
    const double speed =
        speed_lo + (speed_hi - speed_lo) * static_cast<double>(i) / std::max(cases - 1, 1);
    std::mt19937 gen(100U + static_cast<unsigned>(i));
    MpcDockingSegmentCoreInput in;
    core.ResizeInput(in);
    in.q0 = rig.arm.q_nominal;
    in.catch_target_valid = true;
    in.q_catch_target = rig.arm.q_nominal;
    FillBall(core, h.p + h.R * Eigen::Vector3d(0.0, 0.0, rig.params.s_ent), -speed * h.R.col(2),
             SmallCovariance(gen), in);
    const bool solved = core.Solve(in, out);
    EXPECT_TRUE(solved) << rtc::catching::MpcDockingReasonName(out.reason);
    Tally(out, solved, tally);
  }
  return tally;
}

// The shipped stop segment (7 × 0.05 s, blocks 1, 1, 2, 3) behind `n_pre`
// pre-catch intervals of `dt_pre`, one block each.
inline void SetShortLeadGrid(MpcDockingSegmentCoreParams& p, int n_pre, double dt_pre) {
  p.n_pre = n_pre;
  p.dt_pre = dt_pre;
  p.n_stop = 7;
  p.dt_stop = 0.05;
  p.n_blocks = n_pre + 4;
  p.block_sizes.fill(1);
  p.block_sizes[static_cast<std::size_t>(n_pre) + 2] = 2;
  p.block_sizes[static_cast<std::size_t>(n_pre) + 3] = 3;
}

}  // namespace rtc::testing::mpc_docking
