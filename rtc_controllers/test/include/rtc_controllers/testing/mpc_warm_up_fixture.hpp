// ── The MPC segment planner's configure warm-ups, solved again (test-only) ─────
// #880: ProxQP's approximate primal-infeasibility test (eps_primal_inf) calls
// some feasible QPs infeasible. The warm-ups are where it showed first (#857
// D12): a stop from rest at a pose where no joint is loaded by gravity (a
// vertical 7-joint arm at q = 0) is degenerate — z = 0 solves it, every torque
// row sits at 0 — and ProxQP reported it infeasible after 3 iterations.
//
// WarmUpSolvesAt() re-solves every core's warm-up of a configured planner —
// the planner's own core parameters and inputs — at a pose of the caller's
// choice, on fresh cores whose infeasibility threshold is the caller's. The
// planner builds that input in MpcSegmentPlanner::WarmUp; the fields set here
// are the ones it sets, and WarmUpReplica (at the planner's own pose and
// threshold) checks that the result is the planner's, bit for bit — the
// copy cannot drift from WarmUp unnoticed.
//
// The suites that use it (approach planner, segment capacity, G1) record the
// verdict per threshold; RTC_MPC_EPS_GRID (comma-separated) widens the
// thresholds to the selection grid of #880 D8.
//
// Test-only, NEVER installed (mpc_segment_core_fixture.hpp: why).
#pragma once

#include "rtc_controllers/catching/mpc_segment_core.hpp"
#include "rtc_controllers/catching/mpc_segment_planner.hpp"

#include <Eigen/Core>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

namespace rtc::testing::mpc_warm_up {

/// One core's warm-up solve.
struct WarmUpSolve {
  std::string core;  ///< "stop k" / "catch n_pre"
  bool ok{false};
  int qp_status{-1};  ///< ProxQP's status (0 solved, 1 max_iter, 2 primal infeasible)
  int iterations{0};
  rtc::catching::MpcSegmentCoreReason reason{rtc::catching::MpcSegmentCoreReason::kNone};
  bool same_as_planner{false};  ///< the solution equals the planner core's, bit for bit
};

/// The middle of the cores' box (every core has the same one, MD-64).
[[nodiscard]] inline Eigen::VectorXd BoxMiddle(const rtc::catching::MpcSegmentPlanner& planner) {
  const Eigen::VectorXd& lo = planner.Core(0).PositionLow();
  const Eigen::VectorXd& hi = planner.Core(0).PositionHigh();
  return lo + 0.5 * (hi - lo);
}

/// The pose the planner warmed up at (the stop cores' input keeps it).
[[nodiscard]] inline Eigen::VectorXd PlannerWarmUpPose(
    const rtc::catching::MpcSegmentPlanner& planner) {
  return planner.StopCoreInput(0).q0;
}

/// Every core's warm-up of `planner` (configured on `model`) at pose `q`
/// (model order), each on a fresh core whose solver.eps_primal_inf is `eps`.
[[nodiscard]] inline std::vector<WarmUpSolve> WarmUpSolvesAt(
    const rtc::catching::MpcSegmentPlanner& planner,
    const rtc::catching::MpcSegmentPlannerModel& model, const Eigen::VectorXd& q, double eps) {
  using rtc::catching::MpcSegmentCore;
  using rtc::catching::MpcSegmentCoreInput;
  using rtc::catching::MpcSegmentCoreLimits;
  using rtc::catching::MpcSegmentCoreParams;
  using rtc::catching::MpcSegmentCoreReason;
  using rtc::catching::MpcSegmentCoreResult;
  const int n = planner.Nv();
  const auto& pp = planner.Params();
  MpcSegmentCoreLimits limits;  // as Configure builds them
  limits.q_min.resize(n);
  limits.q_max.resize(n);
  limits.qd_max.resize(n);
  limits.tau_max.resize(n);
  limits.armature = Eigen::VectorXd::Zero(n);
  for (int m = 0; m < n; ++m) {
    const auto u = static_cast<std::size_t>(m);
    limits.q_min[m] = model.q_min[u];
    limits.q_max[m] = model.q_max[u];
    limits.qd_max[m] = model.qdot_max[u];
    limits.tau_max[m] = model.tau_max[u];
  }
  pinocchio::Data data(*model.arm);
  pinocchio::forwardKinematics(*model.arm, data, q);
  pinocchio::updateFramePlacement(*model.arm, data, model.catch_frame);
  const Eigen::Vector3d p = data.oMf[model.catch_frame].translation();
  const Eigen::Vector3d z = data.oMf[model.catch_frame].rotation().col(2);
  const Eigen::Vector3d v_ball = -5.0 * z;
  const Eigen::Vector3d d_ball = v_ball.normalized();
  const bool perp_on = pp.w_perp > 0.0;

  std::vector<WarmUpSolve> out;
  const auto solve = [&](const MpcSegmentCoreParams& planner_params, const MpcSegmentCore& theirs,
                         const MpcSegmentCoreInput& in, const std::string& name) {
    WarmUpSolve s;
    s.core = name;
    MpcSegmentCoreParams mp = planner_params;
    mp.solver.eps_primal_inf = eps;
    MpcSegmentCore core;
    s.reason = core.Init(*model.arm, model.catch_frame, mp, limits);
    if (s.reason == MpcSegmentCoreReason::kNone) {
      MpcSegmentCoreResult res;
      core.ResizeResult(res);
      s.ok = core.Solve(in, res);
      s.reason = res.reason;
      s.qp_status = res.qp_status;
      s.iterations = res.iterations;
      s.same_as_planner = s.ok && core.LastSolution().size() == theirs.LastSolution().size() &&
                          core.LastSolution() == theirs.LastSolution();
    }
    out.push_back(s);
  };
  for (int k = 0; k <= pp.k_max; ++k) {
    MpcSegmentCoreInput in = planner.StopCoreInput(k);
    in.q0 = q;
    in.qd0.setZero();
    in.qdd0.setZero();
    in.reference_valid = false;
    in.cold_start = true;
    if (perp_on) {
      in.p_c = p;
      in.d_hat = d_ball;
    }
    solve(planner.StopCoreParams(k), planner.Core(k), in, "stop " + std::to_string(k));
  }
  for (int j = 1; j <= pp.n_pre_max; ++j) {
    const MpcSegmentCoreParams& cp = planner.ApproachCoreParams(j);
    MpcSegmentCoreInput in = planner.ApproachCoreInput(j);
    in.q0 = q;
    in.qd0.setZero();
    in.qdd0.setZero();
    for (Eigen::Index c = 0; c < in.q_ref.cols(); ++c) {
      in.q_ref.col(c) = q;
    }
    in.qd_ref.setZero();
    in.qdd_ref.setZero();
    in.reference_valid = true;
    in.cold_start = true;
    in.w_delta_scale = 0.0;
    in.p_b = p;
    in.a_d = z;
    in.v_b = v_ball;
    in.w_p = pp.w_const * Eigen::Matrix3d::Identity();
    in.gamma_ref = pp.gamma_ref;
    if (perp_on) {
      in.p_c = in.p_b;
      in.d_hat = d_ball;
    }
    in.path_on = cp.path_nodes;
    for (Eigen::Index k = 0; k < in.path_p.cols(); ++k) {
      in.path_p.col(k) = p;
      in.path_d.col(k) = d_ball;
    }
    solve(cp, planner.ApproachCore(j), in, "catch " + std::to_string(j));
  }
  return out;
}

/// The thresholds a suite records: RTC_MPC_EPS_GRID (comma-separated) when
/// set, else ProxQP's default 1e-4 and the core's own (MpcSegmentCoreParams{}).
[[nodiscard]] inline std::vector<double> EpsGrid() {
  std::vector<double> grid;
  if (const char* e = std::getenv("RTC_MPC_EPS_GRID"); e != nullptr && e[0] != '\0') {
    std::stringstream ss(e);
    std::string item;
    while (std::getline(ss, item, ',')) {
      grid.push_back(std::stod(item));
    }
    return grid;
  }
  grid.push_back(1e-4);
  const double own = rtc::catching::MpcSegmentCoreParams{}.solver.eps_primal_inf;
  if (own != 1e-4) {
    grid.push_back(own);
  }
  return grid;
}

/// A threshold as a property key ("1e-04", "0e+00").
[[nodiscard]] inline std::string EpsKey(double eps) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%.0e", eps);
  return buf;
}

/// One line per core: "[warm-up] <label> eps <e> <core>: ok|FAIL status s it i".
inline void PrintWarmUpSolves(const std::string& label, double eps,
                              const std::vector<WarmUpSolve>& solves) {
  for (const WarmUpSolve& s : solves) {
    std::printf("[warm-up] %-28s eps %-7.0e %-8s: %-4s status %d it %3d (%s)\n", label.c_str(), eps,
                s.core.c_str(), s.ok ? "ok" : "FAIL", s.qp_status, s.iterations,
                rtc::catching::MpcSegmentCoreReasonName(s.reason));
  }
}

}  // namespace rtc::testing::mpc_warm_up
