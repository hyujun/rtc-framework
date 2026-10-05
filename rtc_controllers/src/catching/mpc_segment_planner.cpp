// Decel planner (MPC E1-F03, E1-F08). See mpc_segment_planner.hpp.
#include "rtc_controllers/catching/mpc_segment_planner.hpp"

#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/traj_sampler.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>

namespace rtc::catching {

namespace {

[[nodiscard]] std::int64_t SecondsToNs(double s) noexcept {
  return static_cast<std::int64_t>(std::llround(s * 1e9));
}

// ⌈a / b⌉ for a ≥ 0, b > 0 (integer grid arithmetic, MD-27).
[[nodiscard]] std::int64_t CeilDiv(std::int64_t a, std::int64_t b) noexcept {
  return (a + b - 1) / b;
}

[[nodiscard]] std::size_t U(int i) noexcept {
  return static_cast<std::size_t>(i);
}

// The core's design values, all YAML keys, from `planner.decel_mpc.*`: the cost
// scalars (the stop-path weight among them — both core kinds carry that term,
// on their nodes from the catch on), the axis and trust-region limits, the
// rest tolerance and the solver's tolerances. `jerk_weight` is already in
// model order (empty = the core's all-ones); the preconditioner and the KKT
// backend are not design values and keep the core's own setting.
void ApplyCoreDesign(DecelMpcParams& mp, const DecelPlannerParams& p,
                     const Eigen::VectorXd& jerk_weight_model) {
  mp.jerk_weight = jerk_weight_model;
  mp.u_scale = p.u_scale;
  mp.w_delta = p.w_delta;
  mp.rho_tau = p.rho_tau;
  mp.w_perp = p.w_perp;
  mp.axis_theta_max = p.axis_theta_max;
  mp.delta_tr = p.delta_tr;
  mp.reference_rest_tol = p.reference_rest_tol;
  mp.solver.max_iter = p.solver_max_iter;
  mp.solver.max_iter_in = p.solver_max_iter_in;
  mp.solver.eps_abs = p.solver_eps_abs;
  mp.solver.eps_rel = p.solver_eps_rel;
}

}  // namespace

const char* DecelOutcomeName(DecelOutcome o) noexcept {
  switch (o) {
    case DecelOutcome::kOff:
      return "off";
    case DecelOutcome::kNoState:
      return "no_state";
    case DecelOutcome::kStaleState:
      return "stale_state";
    case DecelOutcome::kUpToDate:
      return "up_to_date";
    case DecelOutcome::kPastReplanWindow:
      return "past_replan_window";
    case DecelOutcome::kInputNonFinite:
      return "input_non_finite";
    case DecelOutcome::kSolveFailed:
      return "solve_failed";
    case DecelOutcome::kBudget:
      return "budget";
    case DecelOutcome::kLate:
      return "late";
    case DecelOutcome::kSlack:
      return "slack";
    case DecelOutcome::kReady:
      return "ready";
    case DecelOutcome::kPublished:
      return "published";
    case DecelOutcome::kSuperseded:
      return "superseded";
    case DecelOutcome::kNotAtRest:
      return "not_at_rest";
    case DecelOutcome::kTooLate:
      return "too_late";
    case DecelOutcome::kNotFollowed:
      return "not_followed";
    case DecelOutcome::kNoBall:
      return "no_ball";
    case DecelOutcome::kCatchError:
      return "catch_error";
    case DecelOutcome::kSpeed:
      return "speed";
  }
  return "unknown";
}

const char* DecelKindName(DecelKind k) noexcept {
  switch (k) {
    case DecelKind::kNone:
      return "none";
    case DecelKind::kFirst:
      return "first";
    case DecelKind::kSame:
      return "same";
    case DecelKind::kAdvance:
      return "advance";
    case DecelKind::kStop:
      return "stop";
  }
  return "unknown";
}

DecelBallTarget MakeDecelBallTarget(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                    bool cov_matched, std::int64_t t_c_ns, double v_eps) noexcept {
  DecelBallTarget b;
  const SampleEval e = SampleAt(traj, NowLead{t_c_ns});
  const double speed = e.v.norm();
  // Written as "usable" so a NaN speed or v_eps is not.
  if (e.valid && !e.extrapolated && std::isfinite(speed) && speed > v_eps) {
    b.valid = true;
    b.p_b = e.p;
    b.v_b = e.v;
    b.a_d = -e.v / speed;
  }
  // Σ_p: SampleAt's bracket, on integer ns. `traj.n` is bounded before any
  // index (wire data), and the covariance must hold the same samples.
  const int n = traj.n;
  if (!cov_matched || !cov.valid || !traj.valid || n < 1 || n > kCap || cov.n != n) {
    return b;
  }
  const auto t_at = [&traj](int i) noexcept { return traj.s[static_cast<std::size_t>(i)].t_ns; };
  if (t_c_ns < t_at(0) || t_c_ns > t_at(n - 1)) {
    return b;
  }
  int i = 0;
  while (i + 1 < n && t_at(i + 1) <= t_c_ns) {
    ++i;
  }
  const auto block = [&cov](int k, int r, int c) noexcept {
    return cov.c[static_cast<std::size_t>(k)][static_cast<std::size_t>(r * 6 + c)];
  };
  Eigen::Matrix3d s = Eigen::Matrix3d::Zero();
  if (t_c_ns == t_at(i)) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        s(r, c) = block(i, r, c);
      }
    }
  } else {
    if (i + 1 >= n) {
      return b;
    }
    const std::int64_t span = t_at(i + 1) - t_at(i);
    if (span < kMinInterpIntervalNs) {
      return b;
    }
    const double alpha = static_cast<double>(t_c_ns - t_at(i)) / static_cast<double>(span);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        s(r, c) = (1.0 - alpha) * block(i, r, c) + alpha * block(i + 1, r, c);
      }
    }
  }
  if (!s.allFinite()) {
    return b;
  }
  b.sigma_p = s;
  b.sigma_valid = true;
  return b;
}

bool DecelPlanner::Configure(const DecelPlannerModel& model, const DecelPlannerConstants& consts,
                             const DecelPlannerParams& params, ClockFn clock, std::string* error) {
  configured_ = false;
  perp_on_ = false;
  cores_.clear();
  inputs_.clear();
  results_.clear();
  warmup_max_ns_ = 0;
  warmup_total_ns_ = 0;
  catch_cores_.clear();
  catch_inputs_.clear();
  catch_results_.clear();
  catch_params_.clear();
  stop_params_.clear();
  ResetTrial();
  const auto fail = [error](const std::string& why) {
    if (error != nullptr) {
      *error = why;
    }
    return false;
  };
  if (clock == nullptr) {
    return fail("no clock");
  }
  if (!model.arm || model.nv < 1 || model.nv > kMaxDecelNv || model.arm->nv != model.nv) {
    return fail("the arm model is missing or its joint count is outside 1.." +
                std::to_string(kMaxDecelNv) + " (kMaxDecelNv)");
  }
  // Upper bounds keep every seconds → ns conversion far from int64 overflow.
  if (!std::isfinite(consts.eta_v) || !(consts.eta_v > 0.0) || consts.eta_v > 1.0 ||
      !std::isfinite(consts.t_arm_s) || consts.t_arm_s < 0.0 || consts.t_arm_s > 1.0 ||
      !std::isfinite(consts.control_dt) || !(consts.control_dt > 0.0) || consts.control_dt > 1.0) {
    return fail("eta_v, T_arm or control_dt is outside its range");
  }
  // A plan is published only with a first segment that starts before t_c
  // (MD-56), so a planner without a pre-catch grid has nothing to publish.
  if (params.n_pre_max < 1) {
    return fail("approach.n_pre_max is below 1 (a segment starts before t_c, MD-56)");
  }
  if (params.k_max < 0 || params.k_max > kMaxDecelReplans || params.DtNs() <= 0) {
    return fail("k_max or dt_s is outside its range");
  }
  // Device order must be a permutation (the payload is device order).
  std::array<bool, kMaxPlanNv> seen{};
  for (int m = 0; m < model.nv; ++m) {
    const int d = model.device_of_model[U(m)];
    if (d < 0 || d >= model.nv || seen[U(d)]) {
      return fail("device_of_model is not a permutation of the arm joints");
    }
    seen[U(d)] = true;
  }
  const int n = model.nv;
  // `cost.jerk_weight` is one entry per ARM joint in device order; the core
  // wants model order. Empty keeps the core's all-ones.
  jerk_weight_model_.resize(0);
  if (!params.jerk_weight.empty()) {
    if (params.jerk_weight.size() != static_cast<std::size_t>(n)) {
      return fail("decel_mpc.cost.jerk_weight has " + std::to_string(params.jerk_weight.size()) +
                  " entries, the arm has " + std::to_string(n) + " joints");
    }
    jerk_weight_model_.resize(n);
    for (int m = 0; m < n; ++m) {
      jerk_weight_model_[m] = params.jerk_weight[U(model.device_of_model[U(m)])];
    }
  }
  DecelMpcLimits limits;
  limits.q_min.resize(n);
  limits.q_max.resize(n);
  limits.qd_max.resize(n);
  limits.tau_max.resize(n);
  // MD-25: armature is not used in control — the core receives an explicit
  // zero, never an empty vector it would have to interpret.
  limits.armature = Eigen::VectorXd::Zero(n);
  for (int m = 0; m < n; ++m) {
    limits.q_min[m] = model.q_min[U(m)];
    limits.q_max[m] = model.q_max[U(m)];
    limits.qd_max[m] = model.qdot_max[U(m)];
    limits.tau_max[m] = model.tau_max[U(m)];
    v_box_[U(m)] = consts.eta_v * model.qdot_max[U(m)];
  }
  cores_.reserve(U(params.k_max + 1));
  inputs_.resize(U(params.k_max + 1));
  results_.resize(U(params.k_max + 1));
  for (int k = 0; k <= params.k_max; ++k) {
    DecelMpcParams mp;
    ApplyCoreDesign(mp, params, jerk_weight_model_);
    mp.n_nodes = params.n_nodes - k;
    mp.dt = static_cast<double>(params.DtNs()) * 1e-9;
    if (!DecelBlocksFor(params, k, mp.block_sizes, mp.n_blocks)) {
      return fail("replan instance " + std::to_string(k) + " has fewer than 3 blocks");
    }
    mp.eta_v = consts.eta_v;
    mp.eta_tau = params.eta_tau;
    mp.m_q = params.m_q;
    auto core = std::make_unique<DecelMpc>();
    const DecelMpcReason r = core->Init(*model.arm, model.catch_frame, mp, limits);
    if (r != DecelMpcReason::kNone) {
      return fail("DecelMpc::Init for replan instance " + std::to_string(k) +
                  " (N = " + std::to_string(mp.n_nodes) + "): " + DecelMpcReasonName(r));
    }
    core->ResizeResult(results_[U(k)]);
    DecelMpcInput& in = inputs_[U(k)];
    in.q0 = Eigen::VectorXd::Zero(n);
    in.qd0 = Eigen::VectorXd::Zero(n);
    in.qdd0 = Eigen::VectorXd::Zero(n);
    in.q_ref = Eigen::MatrixXd::Zero(n, mp.n_nodes + 1);
    in.qd_ref = Eigen::MatrixXd::Zero(n, mp.n_nodes + 1);
    in.qdd_ref = Eigen::MatrixXd::Zero(n, mp.n_nodes + 1);
    in.reference_valid = false;
    cores_.push_back(std::move(core));
    stop_params_.push_back(mp);
  }
  params_ = params;
  perp_on_ = params.w_perp > 0.0;
  clock_ = clock;
  nv_ = n;
  device_of_model_ = model.device_of_model;
  dt_ns_ = params.DtNs();
  t_arm_ns_ = SecondsToNs(consts.t_arm_s);
  consts_ = consts;
  h_ns_ = SecondsToNs(consts.control_dt);
  for (int m = 0; m < n; ++m) {
    qd_max_[U(m)] = model.qdot_max[U(m)];
    // Every core is built from the same limits and m_q, so one box serves
    // all. A locked joint's two bounds can cross by a rounding (the core
    // allows for it); std::clamp needs them ordered.
    q_lo_[U(m)] = cores_.front()->PositionLow()[m];
    q_hi_[U(m)] = std::max(q_lo_[U(m)], cores_.front()->PositionHigh()[m]);
  }
  std::string why;
  if (!ConfigureApproach(model, why)) {
    return fail(why);
  }
  configured_ = true;
  return true;
}

bool DecelPlanner::ConfigureApproach(const DecelPlannerModel& model, std::string& why) {
  const DecelPlannerParams& params = params_;
  const int n = nv_;
  dt_pre_ns_ = params.DtPreNs();
  first_ns_ = SecondsToNs(params.budget_first_s);
  replan_ns_ = SecondsToNs(params.budget_replan_s);
  std::array<int, kMaxDecelNodes> stop_blocks{};
  int stop_n_blocks = 0;
  if (!std::isfinite(consts_.v_eps) || !(consts_.v_eps > 0.0) || dt_pre_ns_ <= 0 ||
      dt_pre_ns_ > kMaxDecelDtPreNs || first_ns_ <= 0 || replan_ns_ <= 0 ||
      params.n_pre_max > kMaxDecelNodes - params.n_nodes ||
      !DecelBlocksFor(params, 0, stop_blocks, stop_n_blocks) ||
      params.n_pre_max > kMaxDecelNodes - stop_n_blocks) {
    why = "the pre-catch grid (n_pre_max, dt_pre_s, the budgets or v_eps) is outside its range";
    return false;
  }
  // The trust region must hold the position margin: with m_q ≥ δ a target at
  // a limit would put the trust row's lower bound above its upper one. δ is
  // the profile's `linearization.delta_tr`, the value the cores are built with.
  if (!(params.m_q < params.delta_tr)) {
    why = "decel_mpc.m_q must be below the trust region, decel_mpc.linearization.delta_tr (" +
          std::to_string(params.delta_tr) + " rad) with a pre-catch grid";
    return false;
  }
  for (int m = 0; m < n; ++m) {
    const double cap = params.ref_speed_fraction * consts_.eta_v * qd_max_[U(m)];
    if (!std::isfinite(cap) || !(cap > 0.0)) {
      why = "ref_speed_fraction·eta_v·qdot_max is not a positive finite number for model joint " +
            std::to_string(m);
      return false;
    }
  }
  rest_tol_ref_ = params.reference_rest_tol;
  DecelMpcLimits limits;
  limits.q_min.resize(n);
  limits.q_max.resize(n);
  limits.qd_max.resize(n);
  limits.tau_max.resize(n);
  limits.armature = Eigen::VectorXd::Zero(n);  // MD-25
  for (int m = 0; m < n; ++m) {
    limits.q_min[m] = model.q_min[U(m)];
    limits.q_max[m] = model.q_max[U(m)];
    limits.qd_max[m] = model.qdot_max[U(m)];
    limits.tau_max[m] = model.tau_max[U(m)];
  }
  const auto count = U(params.n_pre_max);
  catch_cores_.reserve(count);
  catch_inputs_.resize(count);
  catch_results_.resize(count);
  catch_params_.reserve(count);
  for (int j = 1; j <= params.n_pre_max; ++j) {
    DecelMpcParams mp;
    ApplyCoreDesign(mp, params, jerk_weight_model_);
    mp.n_pre = j;
    mp.dt_pre = static_cast<double>(dt_pre_ns_) * 1e-9;
    mp.n_nodes = params.n_nodes;
    mp.dt = static_cast<double>(dt_ns_) * 1e-9;
    mp.block_sizes = {};
    for (int b = 0; b < j; ++b) {
      mp.block_sizes[U(b)] = 1;
    }
    for (int b = 0; b < stop_n_blocks; ++b) {
      mp.block_sizes[U(j + b)] = stop_blocks[U(b)];
    }
    mp.n_blocks = j + stop_n_blocks;
    mp.catch_terms = true;
    mp.w_axis = params.w_axis;
    mp.w_v_par = params.w_v_par;
    mp.w_v_perp = params.w_v_perp;
    // The relative-velocity slack row lives at the catch node, so only the
    // catch cores carry it (0 = off: the QP keeps its dimensions).
    mp.rho_v = params.rho_v;
    mp.v_rel_allow = params.v_rel_allow;
    // The stop cores' box (MD-64): the stop core that takes over at t_c
    // refuses an initial state outside its own.
    mp.eta_v = consts_.eta_v;
    mp.eta_tau = params.eta_tau;
    mp.m_q = params.m_q;
    auto core = std::make_unique<DecelMpc>();
    const DecelMpcReason r = core->Init(*model.arm, model.catch_frame, mp, limits);
    if (r != DecelMpcReason::kNone) {
      why = "DecelMpc::Init for the catch core n_pre = " + std::to_string(j) + ": " +
            DecelMpcReasonName(r);
      return false;
    }
    const int cols = core->NumNodes() + 1;
    core->ResizeResult(catch_results_[U(j - 1)]);
    DecelMpcInput& in = catch_inputs_[U(j - 1)];
    in.q0 = Eigen::VectorXd::Zero(n);
    in.qd0 = Eigen::VectorXd::Zero(n);
    in.qdd0 = Eigen::VectorXd::Zero(n);
    in.q_ref = Eigen::MatrixXd::Zero(n, cols);
    in.qd_ref = Eigen::MatrixXd::Zero(n, cols);
    in.qdd_ref = Eigen::MatrixXd::Zero(n, cols);
    in.reference_valid = false;
    catch_cores_.push_back(std::move(core));
    catch_params_.push_back(mp);
  }
  return WarmUp(model, why);
}

bool DecelPlanner::WarmUp(const DecelPlannerModel& model, std::string& why) {
  // One solve per core at the middle of the joint range (MD-64): the first
  // ProxQP solve of a core is its slowest. The stop cores solve a stop from
  // rest (no reference: pre-solve + solve); the catch cores a catch where
  // the ball already is — at the catch frame, along its +z — so every term
  // runs. With the stop-path term on (w_⊥ > 0) both kinds solve on that
  // synthetic ball's line — through the catch frame, along its travel — so no
  // warm-up runs on the input's default line.
  const int n = nv_;
  warmup_max_ns_ = 0;
  warmup_total_ns_ = 0;
  // Timed on the real steady clock (the injected one may be a test's).
  const auto timed = [this](DecelMpc& core, const DecelMpcInput& in, DecelMpcResult& res) {
    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = core.Solve(in, res);
    const std::int64_t ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0)
            .count();
    warmup_max_ns_ = std::max(warmup_max_ns_, ns);
    warmup_total_ns_ += ns;
    return ok;
  };
  Eigen::VectorXd q_mid(n);
  for (int m = 0; m < n; ++m) {
    q_mid[m] = 0.5 * (model.q_min[U(m)] + model.q_max[U(m)]);
  }
  pinocchio::Data data(*model.arm);
  pinocchio::forwardKinematics(*model.arm, data, q_mid);
  pinocchio::updateFramePlacement(*model.arm, data, model.catch_frame);
  const Eigen::Vector3d p = data.oMf[model.catch_frame].translation();
  const Eigen::Vector3d z = data.oMf[model.catch_frame].rotation().col(2);
  const Eigen::Vector3d v_ball = -5.0 * z;
  const Eigen::Vector3d d_ball = v_ball.normalized();
  for (std::size_t k = 0; k < cores_.size(); ++k) {
    DecelMpcInput& in = inputs_[k];
    in.q0 = q_mid;
    in.qd0.setZero();
    in.qdd0.setZero();
    in.reference_valid = false;
    in.cold_start = true;
    if (perp_on_) {
      in.p_c = p;
      in.d_hat = d_ball;
    }
    if (!timed(*cores_[k], in, results_[k])) {
      why = "warm-up solve of stop core k = " + std::to_string(k) + ": " +
            DecelMpcReasonName(results_[k].reason);
      return false;
    }
    in.cold_start = false;
  }
  for (std::size_t j = 0; j < catch_cores_.size(); ++j) {
    DecelMpcInput& in = catch_inputs_[j];
    in.q0 = q_mid;
    in.qd0.setZero();
    in.qdd0.setZero();
    for (Eigen::Index c = 0; c < in.q_ref.cols(); ++c) {
      in.q_ref.col(c) = q_mid;
    }
    in.qd_ref.setZero();
    in.qdd_ref.setZero();
    in.reference_valid = true;
    in.cold_start = true;
    in.w_delta_scale = 0.0;
    in.p_b = p;
    in.a_d = z;
    in.v_b = v_ball;
    in.w_p = params_.w_const * Eigen::Matrix3d::Identity();
    in.gamma_ref = params_.gamma_ref;
    if (perp_on_) {
      in.p_c = in.p_b;
      in.d_hat = d_ball;
    }
    if (!timed(*catch_cores_[j], in, catch_results_[j])) {
      why = "warm-up solve of the catch core n_pre = " + std::to_string(j + 1) + ": " +
            DecelMpcReasonName(catch_results_[j].reason);
      return false;
    }
    in.reference_valid = false;
    in.cold_start = false;
  }
  return true;
}

void DecelPlanner::ResetTrial() noexcept {
  ring_n_ = 0;
  ring_plan_id_ = 0;
  ring_t_c_ns_ = 0;
  reported_pending_seq_ = 0;
  reported_active_seq_ = 0;
  last_solve_valid_ = false;
  ready_line_.valid = false;
}

bool DecelPlanner::CheckState(const PlannerRtState& rt, std::int64_t start, bool need_command,
                              DecelRecord& rec) const noexcept {
  if (!rt.valid || (need_command && !rt.cmd_seeded) || rt.nv != nv_) {
    rec.outcome = DecelOutcome::kNoState;
    return false;
  }
  const std::int64_t age = start - rt.rt_state_ns;
  if (age < 0 || age > kDecelMaxRtStateAgeNs) {
    rec.outcome = DecelOutcome::kStaleState;
    return false;
  }
  return true;
}

bool DecelPlanner::SetCatchInputs(const Eigen::Vector3d& p_b, const Eigen::Vector3d& v_b,
                                  const Eigen::Vector3d& a_d, const DecelBallTarget& ball,
                                  bool first, DecelMpcInput& in, DecelRecord& rec) const noexcept {
  if (!p_b.allFinite() || !v_b.allFinite() || !a_d.allFinite()) {
    return false;
  }
  in.p_b = p_b;
  in.v_b = v_b;
  in.a_d = a_d;
  in.gamma_ref = params_.gamma_ref;
  // MD-63: W_p from Σ_p, and w_Δ scheduled by tr Σ_p — or, without a usable
  // Σ_p, the constant weight and the full pull. The ratio is checked finite
  // before it is clamped: std::clamp hands a NaN straight through.
  bool fallback = !ball.sigma_valid;
  double scale = 1.0;
  if (!fallback) {
    Eigen::Matrix3d w;
    const double r = ball.sigma_p.trace() / (params_.sigma_ref * params_.sigma_ref);
    if (std::isfinite(r) &&
        CatchPositionWeight(ball.sigma_p, params_.kappa, params_.sigma_floor, params_.w_max, w)) {
      in.w_p = w;
      scale = std::clamp(r, 0.0, 1.0);
    } else {
      fallback = true;
    }
  }
  if (fallback) {
    in.w_p = params_.w_const * Eigen::Matrix3d::Identity();
    scale = 1.0;
  }
  // The first solve's reference is a curve, not a previous solution worth
  // staying near (formulation §1.3).
  in.w_delta_scale = first ? 0.0 : scale;
  rec.w_p_fallback = fallback;
  rec.w_delta_scale = in.w_delta_scale;
  return true;
}

bool DecelPlanner::SetStopLine(DecelMpcInput& in, DecelRecord& rec) const noexcept {
  // The ball as this solve takes it (SetCatchInputs wrote both, finite): the
  // line through p̂_b along v̂_b. Finite components can still have a norm that
  // overflows, and a ball at rest has no direction — neither is solved on a
  // line that was not built.
  const double speed = in.v_b.norm();
  if (!in.p_b.allFinite() || !std::isfinite(speed)) {
    rec.outcome = DecelOutcome::kInputNonFinite;
    return false;
  }
  // Written as "usable", so a NaN v_eps is not.
  if (!(speed > consts_.v_eps)) {
    rec.outcome = DecelOutcome::kNoBall;
    return false;
  }
  in.p_c = in.p_b;
  in.d_hat = in.v_b / speed;
  return true;
}

bool DecelBetweenNodeSpeedOk(const Eigen::Ref<const Eigen::MatrixXd>& qd,
                             const Eigen::Ref<const Eigen::MatrixXd>& qdd, int n_pre, double dt_pre,
                             double dt, std::span<const double> qd_max,
                             double& ratio_max) noexcept {
  const Eigen::Index n = qd.rows();
  const Eigen::Index cols = qd.cols();
  ratio_max = std::numeric_limits<double>::infinity();
  if (n < 1 || cols < 1 || qdd.rows() != n || qdd.cols() != cols ||
      qd_max.size() < static_cast<std::size_t>(n) || n_pre < 0) {
    return false;
  }
  double worst = 0.0;
  bool ok = true;
  const auto check = [&](double v, double lim) noexcept {
    const bool inside = std::isfinite(v) && std::isfinite(lim) && lim > 0.0 && std::fabs(v) <= lim;
    ok = ok && inside;
    const double r = std::fabs(v) / lim;
    // std::max drops a NaN, so a ratio that is not a number is recorded as +inf.
    worst = std::isfinite(r) ? std::max(worst, r) : std::numeric_limits<double>::infinity();
  };
  for (Eigen::Index k = 0; k < cols; ++k) {
    const double step = k < n_pre ? dt_pre : dt;
    for (Eigen::Index m = 0; m < n; ++m) {
      const double lim = qd_max[static_cast<std::size_t>(m)];
      const double v0 = qd(m, k);
      check(v0, lim);
      if (!std::isfinite(qdd(m, k))) {
        // No extremum can be located on a NaN acceleration: refuse it.
        ok = false;
        worst = std::numeric_limits<double>::infinity();
      }
      if (k + 1 < cols) {
        const double a0 = qdd(m, k);
        const double a1 = qdd(m, k + 1);
        if (a0 * a1 < 0.0) {
          const double tau = step * a0 / (a0 - a1);
          check(v0 + 0.5 * a0 * tau, lim);
        }
      }
    }
  }
  ratio_max = worst;
  return ok;
}

DecelOutcome DecelPlanner::Judge(const DecelMpcResult& r, bool ok, int n_pre, int n_total,
                                 bool catch_core, std::int64_t start, std::int64_t end,
                                 std::int64_t budget_ns, std::int64_t t_eff,
                                 DecelRecord& rec) const noexcept {
  rec.solve_ns = end - start;
  rec.core_reason = r.reason;
  rec.presolved = r.presolved;
  rec.iterations = r.iterations + r.presolve_iterations;
  rec.qp_status = r.qp_status;
  rec.solver_retried = r.cold_retried;
  if (!ok) {
    return DecelOutcome::kSolveFailed;
  }
  rec.slack_max = r.slack_max;
  rec.slack_terminal_max = r.slack_terminal_max;
  rec.tau_ratio_max =
      r.torque_evaluated ? r.tau_ratio_max : std::numeric_limits<double>::quiet_NaN();
  if (catch_core && r.catch_evaluated) {
    rec.catch_pos_err = r.catch_pos_err.norm();
    rec.catch_axis_err = r.catch_axis_err;
    rec.catch_gamma = r.catch_gamma;
    rec.catch_v_rel = r.catch_v_rel.norm();
    rec.slack_v = r.slack_v;
  }
  // Every check passes on a positive comparison, so a NaN fails it.
  if (!(rec.solve_ns <= budget_ns)) {
    return DecelOutcome::kBudget;
  }
  if (!StartsInTime(end, t_eff)) {
    return DecelOutcome::kLate;
  }
  const bool slack_ok = std::isfinite(r.slack_max) && r.slack_max <= params_.slack_max &&
                        std::isfinite(r.slack_terminal_max) &&
                        r.slack_terminal_max <= params_.slack_terminal_max;
  if (!slack_ok) {
    return DecelOutcome::kSlack;
  }
  if (catch_core && !(r.catch_evaluated && std::isfinite(rec.catch_pos_err) &&
                      rec.catch_pos_err <= params_.catch_pos_err_max)) {
    return DecelOutcome::kCatchError;
  }
  double ratio = 0.0;
  const bool speed_ok = DecelBetweenNodeSpeedOk(
      r.qd.leftCols(n_total + 1), r.qdd.leftCols(n_total + 1), n_pre,
      static_cast<double>(dt_pre_ns_) * 1e-9, static_cast<double>(dt_ns_) * 1e-9,
      std::span<const double>(qd_max_.data(), static_cast<std::size_t>(nv_)), ratio);
  rec.speed_ratio_max = ratio;
  if (!speed_ok) {
    return DecelOutcome::kSpeed;
  }
  // A published segment can be the next solve's reference, and the core
  // takes a reference only at rest to its own tolerance (tighter than the
  // payload's kDecelRestTol).
  for (int m = 0; m < nv_; ++m) {
    if (!(std::fabs(r.qd(m, n_total)) <= rest_tol_ref_ &&
          std::fabs(r.qdd(m, n_total)) <= rest_tol_ref_)) {
      rec.core_reason = DecelMpcReason::kNone;
      return DecelOutcome::kSolveFailed;
    }
  }
  return DecelOutcome::kReady;
}

void DecelPlanner::PackSegment(const PlannerRtState& rt, std::uint64_t track_generation,
                               std::uint32_t plan_id, std::int64_t t_c, std::int64_t t_eff,
                               int n_pre, int k0, int n_total, const DecelMpcResult& r,
                               DecelPlanSnapshot& out) const noexcept {
  out = DecelPlanSnapshot{};
  out.token.activation_generation = rt.activation_generation;
  out.token.generation = track_generation;
  out.rt_iteration = rt.rt_iteration;
  out.rt_state_ns = rt.rt_state_ns;
  out.plan_id = plan_id;
  out.t_c_ns = t_c;
  out.t0_ns = t_eff;
  out.dt_ns = dt_ns_;
  out.dt_pre_ns = n_pre > 0 ? dt_pre_ns_ : 0;
  out.k0 = k0;
  out.n_nodes = n_total;
  out.nv = nv_;
  out.n_pre = n_pre;
  for (int i = 0; i <= n_total; ++i) {
    for (int m = 0; m < nv_; ++m) {
      const auto e = static_cast<std::size_t>(i * kMaxDecelNv + device_of_model_[U(m)]);
      out.q[e] = r.q(m, i);
      out.qd[e] = r.qd(m, i);
      out.qdd[e] = r.qdd(m, i);
    }
  }
  out.slack_max = r.slack_max;
  out.slack_terminal_max = r.slack_terminal_max;
  // NaN = not evaluated (torque rows off): the core leaves its field stale then.
  out.tau_ratio_max =
      r.torque_evaluated ? r.tau_ratio_max : std::numeric_limits<double>::quiet_NaN();
  out.valid = true;
}

const DecelPlanSnapshot* DecelPlanner::FindInRing(std::uint32_t seq) const noexcept {
  for (int i = 0; i < ring_n_; ++i) {
    if (ring_[U(i)].decel_seq == seq) {
      return &ring_[U(i)];
    }
  }
  return nullptr;
}

std::uint32_t DecelPlanner::SourceSeq(const PlannerRtState& rt,
                                      std::int64_t t_eff_ns) const noexcept {
  if (ring_n_ == 0 || !rt.plan_active || rt.plan_id != ring_plan_id_ ||
      rt.plan_t_c_ns != ring_t_c_ns_) {
    return 0;
  }
  // The pending one is what the RT follows from its node 0 on.
  if (rt.decel_pending) {
    const DecelPlanSnapshot* p = FindInRing(rt.decel_pending_seq);
    if (p != nullptr && p->t0_ns <= t_eff_ns) {
      return p->decel_seq;
    }
  }
  if (rt.decel_active) {
    const DecelPlanSnapshot* p = FindInRing(rt.decel_seq);
    if (p != nullptr) {
      return p->decel_seq;
    }
  }
  return 0;
}

bool DecelPlanner::FollowedTrack(const PlannerRtState& rt,
                                 std::uint64_t& generation) const noexcept {
  if (ring_n_ == 0 || !rt.plan_active || rt.plan_id != ring_plan_id_ ||
      rt.plan_t_c_ns != ring_t_c_ns_) {
    return false;
  }
  // Every segment of a plan carries the plan's track (PlanFirst, Replan).
  generation = ring_[U(ring_n_ - 1)].token.generation;
  return true;
}

void DecelPlanner::NotePublished(const DecelPlanSnapshot& p) noexcept {
  if (ring_n_ > 0 && (p.plan_id != ring_plan_id_ || p.t_c_ns != ring_t_c_ns_)) {
    ring_n_ = 0;
  }
  ring_plan_id_ = p.plan_id;
  ring_t_c_ns_ = p.t_c_ns;
  if (ring_n_ == kRingSize) {
    // Evict the oldest segment the RT did not last report; at most two are
    // reported, so one of eight always qualifies.
    int victim = 0;
    for (int i = 0; i < ring_n_; ++i) {
      const std::uint32_t s = ring_[U(i)].decel_seq;
      if (s != reported_pending_seq_ && s != reported_active_seq_) {
        victim = i;
        break;
      }
    }
    for (int i = victim; i + 1 < ring_n_; ++i) {
      ring_[U(i)] = ring_[U(i + 1)];
      if (perp_on_) {
        ring_line_[U(i)] = ring_line_[U(i + 1)];
      }
    }
    --ring_n_;
  }
  ring_[U(ring_n_)] = p;
  if (perp_on_) {
    // The segment's stop-path line: the one the solve that produced it ran
    // on, handed over once. A segment published without such a solve right
    // before it has none, and a stop core is not solved from it.
    ring_line_[U(ring_n_)] = ready_line_;
    ready_line_.valid = false;
  }
  ++ring_n_;
}

bool DecelPlanner::ColdStartFor(bool catch_core, int index, std::int64_t t_eff,
                                std::uint32_t plan_id, std::int64_t t_c) const noexcept {
  return !(last_solve_valid_ && last_solve_catch_ == catch_core && last_solve_index_ == index &&
           last_solve_t_eff_ == t_eff && last_solve_plan_id_ == plan_id && last_solve_t_c_ == t_c);
}

void DecelPlanner::NoteSolve(bool ok, bool catch_core, int index, std::int64_t t_eff,
                             std::uint32_t plan_id, std::int64_t t_c) noexcept {
  last_solve_valid_ = ok;
  last_solve_catch_ = catch_core;
  last_solve_index_ = index;
  last_solve_t_eff_ = t_eff;
  last_solve_plan_id_ = plan_id;
  last_solve_t_c_ = t_c;
}

DecelBallTarget DecelPlanner::TargetAt(const BallPrediction& ball,
                                       std::int64_t t_c_ns) const noexcept {
  if (ball.Empty()) {
    return DecelBallTarget{};
  }
  return MakeDecelBallTarget(*ball.traj, *ball.cov, ball.cov_matched, t_c_ns, consts_.v_eps);
}

// The target is built BEFORE the solve's own entry, which is where its clock
// starts: the budget measures what it measured when the cycle built it.
bool DecelPlanner::PlanFirst(const PlannerRtState& rt, const PlanSnapshot& plan,
                             const BallPrediction& ball, DecelPlanSnapshot& out,
                             DecelRecord& rec) noexcept {
  return PlanFirst(rt, plan, TargetAt(ball, plan.t_c_ns), out, rec);
}

bool DecelPlanner::Replan(const PlannerRtState& rt, const BallPrediction& ball,
                          DecelPlanSnapshot& out, DecelRecord& rec) noexcept {
  return Replan(rt, TargetAt(ball, rt.plan_t_c_ns), out, rec);
}

bool DecelPlanner::PlanFirst(const PlannerRtState& rt, const PlanSnapshot& plan,
                             const DecelBallTarget& ball, DecelPlanSnapshot& out,
                             DecelRecord& rec) noexcept {
  rec = DecelRecord{};
  ready_line_.valid = false;  // a line belongs to the solve that built it
  if (!configured_) {
    return false;
  }
  rec.kind = DecelKind::kFirst;
  const std::int64_t start = clock_();
  // Before a plan the RT has no command yet (cmd_seeded false): it reports
  // the MEASURED pose with zero velocity, which is where it seeds the command
  // when it takes the plan — the same x₀ either way.
  if (!CheckState(rt, start, /*need_command=*/false, rec)) {
    return false;
  }
  if (!plan.valid || plan.t_c_ns <= 0 || plan.nv != nv_) {
    rec.outcome = DecelOutcome::kNoState;
    return false;
  }
  // The arm rests at its wait pose: x₀ = (q_cmd, 0, 0).
  double speed = 0.0;
  bool speed_finite = true;
  for (int d = 0; d < nv_; ++d) {
    const double v = rt.qd_cmd[U(d)];
    speed_finite = speed_finite && std::isfinite(v);
    speed = std::max(speed, std::fabs(v));
  }
  rec.x0_speed = speed_finite ? speed : std::numeric_limits<double>::quiet_NaN();
  if (!(speed_finite && speed <= params_.rest_tol)) {
    rec.outcome = DecelOutcome::kNotAtRest;
    return false;
  }
  // The largest pre-catch count whose node 0 the solve can still meet.
  const std::int64_t now_lead = start + t_arm_ns_;
  const std::int64_t numer = plan.t_c_ns - now_lead - first_ns_ - 2 * h_ns_;
  if (numer < dt_pre_ns_) {
    rec.outcome = DecelOutcome::kTooLate;
    return false;
  }
  const int n_pre = static_cast<int>(std::min<std::int64_t>(params_.n_pre_max, numer / dt_pre_ns_));
  const std::int64_t t_eff = plan.t_c_ns - static_cast<std::int64_t>(n_pre) * dt_pre_ns_;
  const int n_total = n_pre + params_.n_nodes;
  rec.k = -n_pre;
  rec.n_nodes = n_total;

  DecelMpc& core = *catch_cores_[U(n_pre - 1)];
  DecelMpcInput& in = catch_inputs_[U(n_pre - 1)];
  DecelMpcResult& res = catch_results_[U(n_pre - 1)];
  // Reference: per joint a minimum-jerk reach from q_cmd to q_star over the
  // pre-catch part, held from the catch node on. The target is clamped into
  // the core's position box (the IK clamps to the limits themselves) and the
  // reach shortened where its peak speed 1.875·|d|/T would pass
  // 0.9·η_v·q̇_max — a reference the box cannot follow would only be refused
  // by the trust region.
  const double t_span = static_cast<double>(n_pre) * static_cast<double>(dt_pre_ns_) * 1e-9;
  std::array<double, kMaxPlanNv> dq{};
  bool finite = true;
  double scale_min = 1.0;
  double shortfall = 0.0;
  for (int m = 0; m < nv_; ++m) {
    const auto d = U(device_of_model_[U(m)]);
    const double reported = rt.q_cmd[d];
    const double target = plan.q_star[d];
    if (!std::isfinite(reported) || !std::isfinite(target)) {
      finite = false;
      break;
    }
    // Into the core's box, as Replan does: a wait pose inside the m_q margin
    // of a limit would otherwise be refused on every wake, and with it the
    // plan (MD-62).
    const double q0 = std::clamp(reported, q_lo_[U(m)], q_hi_[U(m)]);
    rec.x0_clamped = rec.x0_clamped || q0 != reported;
    in.q0[m] = q0;
    in.qd0[m] = 0.0;
    in.qdd0[m] = 0.0;
    const double clamped = std::clamp(target, q_lo_[U(m)], q_hi_[U(m)]);
    rec.ref_clamped = rec.ref_clamped || clamped != target;
    double step = clamped - q0;
    const double allowed = params_.ref_speed_fraction * v_box_[U(m)] * t_span / 1.875;
    if (!(std::fabs(step) <= allowed)) {
      rec.ref_scaled = true;
      scale_min = std::min(scale_min, allowed / std::fabs(step));
      shortfall = std::max(shortfall, std::fabs(step) - allowed);
      step = std::copysign(allowed, step);
    }
    dq[U(m)] = step;
  }
  if (!finite) {
    rec.outcome = DecelOutcome::kInputNonFinite;
    return false;
  }
  rec.ref_scale = scale_min;
  rec.ref_shortfall = shortfall;
  for (int k = 0; k <= n_total; ++k) {
    // Integer ratio: s is exactly 1 from the catch node on, where the
    // velocity and acceleration polynomials vanish exactly.
    const double s = static_cast<double>(std::min(k, n_pre)) / static_cast<double>(n_pre);
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double pos = 10.0 * s3 - 15.0 * s3 * s + 6.0 * s3 * s2;
    const double vel = (30.0 * s2 - 60.0 * s3 + 30.0 * s2 * s2) / t_span;
    const double acc = (60.0 * s - 180.0 * s2 + 120.0 * s3) / (t_span * t_span);
    for (int m = 0; m < nv_; ++m) {
      const double step = dq[U(m)];
      in.q_ref(m, k) = in.q0[m] + step * pos;
      in.qd_ref(m, k) = step * vel;
      in.qdd_ref(m, k) = step * acc;
    }
  }
  in.reference_valid = true;
  in.cold_start = true;
  rec.cold_start = true;
  const Eigen::Map<const Eigen::Vector3d> p_c(plan.p_c.data());
  const Eigen::Map<const Eigen::Vector3d> v_c(plan.v_c.data());
  const Eigen::Map<const Eigen::Vector3d> a_d(plan.a_d.data());
  if (!SetCatchInputs(p_c, v_c, a_d, ball, /*first=*/true, in, rec)) {
    rec.outcome = DecelOutcome::kInputNonFinite;
    return false;
  }
  // The stop-path line: the plan's catch point along the ball's travel.
  if (perp_on_ && !SetStopLine(in, rec)) {
    return false;
  }
  // No retry: a catch core cannot be solved without its reference.
  const bool ok = core.Solve(in, res);
  const std::int64_t end = clock_();
  NoteSolve(ok, true, n_pre, t_eff, plan.plan_id, plan.t_c_ns);
  rec.outcome = Judge(res, ok, n_pre, n_total, true, start, end, first_ns_, t_eff, rec);
  if (rec.outcome != DecelOutcome::kReady) {
    return false;
  }
  PackSegment(rt, plan.token.generation, plan.plan_id, plan.t_c_ns, t_eff, n_pre, 0, n_total, res,
              out);
  out.x0_clamped = rec.x0_clamped;
  if (!ValidateDecelNodes(out)) {
    rec.outcome = DecelOutcome::kSolveFailed;
    rec.core_reason = DecelMpcReason::kNone;
    return false;
  }
  // A new plan: the RT reports nothing of it yet.
  reported_pending_seq_ = 0;
  reported_active_seq_ = 0;
  if (perp_on_) {
    ready_line_ = StopLine{true, in.p_c, in.d_hat};
  }
  return true;
}

bool DecelPlanner::Replan(const PlannerRtState& rt, const DecelBallTarget& ball,
                          DecelPlanSnapshot& out, DecelRecord& rec) noexcept {
  rec = DecelRecord{};
  ready_line_.valid = false;  // a line belongs to the solve that built it
  if (!configured_) {
    return false;
  }
  const std::int64_t start = clock_();
  if (!CheckState(rt, start, /*need_command=*/true, rec)) {
    return false;
  }
  if (!rt.plan_active || rt.plan_t_c_ns <= 0) {
    rec.outcome = DecelOutcome::kNoState;
    return false;
  }
  reported_pending_seq_ = rt.decel_pending ? rt.decel_pending_seq : 0;
  reported_active_seq_ = rt.decel_active ? rt.decel_seq : 0;

  // The grid point the replan budget reaches (MD-54, MD-58). Recorded as
  // soon as it is chosen, so a withheld solve still says where it was.
  const std::int64_t t_c = rt.plan_t_c_ns;
  const std::int64_t earliest = start + t_arm_ns_ + replan_ns_ + 2 * h_ns_;
  int n_pre = 0;
  int k = 0;
  std::int64_t t_eff = 0;
  if (t_c - earliest >= dt_pre_ns_) {
    n_pre =
        static_cast<int>(std::min<std::int64_t>(params_.n_pre_max, (t_c - earliest) / dt_pre_ns_));
    t_eff = t_c - static_cast<std::int64_t>(n_pre) * dt_pre_ns_;
    rec.k = -n_pre;
    rec.n_nodes = n_pre + params_.n_nodes;
  } else {
    const std::int64_t k64 = earliest <= t_c ? 0 : CeilDiv(earliest - t_c, dt_ns_);
    if (k64 > params_.k_max) {
      rec.k = static_cast<std::int32_t>(std::min<std::int64_t>(k64, 1'000'000));
      rec.outcome = DecelOutcome::kPastReplanWindow;
      return false;
    }
    k = static_cast<int>(k64);
    t_eff = t_c + k64 * dt_ns_;
    rec.k = k;
    rec.n_nodes = params_.n_nodes - k;
  }
  const bool pre = n_pre > 0;
  const int n_total = rec.n_nodes;
  rec.kind = pre ? DecelKind::kAdvance : DecelKind::kStop;

  // The source is what the RT reports, never an inference (MD-58).
  const std::uint32_t src_seq = SourceSeq(rt, t_eff);
  const DecelPlanSnapshot* src = src_seq != 0 ? FindInRing(src_seq) : nullptr;
  if (src == nullptr) {
    rec.outcome = DecelOutcome::kNotFollowed;
    return false;
  }
  rec.source_seq = src_seq;
  if (t_eff < src->t0_ns) {
    rec.outcome = DecelOutcome::kUpToDate;
    return false;
  }
  if (t_eff == src->t0_ns) {
    if (!(pre && params_.replan_same_point)) {
      rec.outcome = DecelOutcome::kUpToDate;
      return false;
    }
    rec.kind = DecelKind::kSame;
  }
  if (pre && !ball.valid) {
    rec.outcome = DecelOutcome::kNoBall;
    return false;
  }
  // A stop core's stop-path line is its SOURCE's — the line of the segment
  // the RT follows, which x₀ and the reference come from too — whatever
  // `ball` holds on this wake and whatever was published since: the hand
  // stops on the line it is on. A source without a line is not solved from —
  // never on a default line.
  const auto src_slot = static_cast<std::size_t>(src - ring_.data());
  if (perp_on_ && !pre && !ring_line_[src_slot].valid) {
    rec.outcome = DecelOutcome::kNoBall;
    return false;
  }

  const std::size_t slot = pre ? U(n_pre - 1) : U(k);
  DecelMpc& core = pre ? *catch_cores_[slot] : *cores_[slot];
  DecelMpcInput& in = pre ? catch_inputs_[slot] : inputs_[slot];
  DecelMpcResult& res = pre ? catch_results_[slot] : results_[slot];

  // x₀: the source at t_eff (MD-58), projected into the core's box —
  // a published node keeps the box only to the solver's tolerance.
  std::array<double, kMaxDecelNv> q{};
  std::array<double, kMaxDecelNv> qd{};
  std::array<double, kMaxDecelNv> qdd{};
  if (!NodeTrajectoryFollower::SampleJoints(*src, t_eff, q, qd, qdd)) {
    rec.outcome = DecelOutcome::kInputNonFinite;
    return false;
  }
  for (int m = 0; m < nv_; ++m) {
    const auto d = U(device_of_model_[U(m)]);
    in.q0[m] = q[d];
    in.qd0[m] = qd[d];
    in.qdd0[m] = qdd[d];
  }
  rec.from_segment = true;
  if (!in.q0.allFinite() || !in.qd0.allFinite() || !in.qdd0.allFinite()) {
    rec.outcome = DecelOutcome::kInputNonFinite;
    return false;
  }
  for (int m = 0; m < nv_; ++m) {
    const double qc = std::clamp(in.q0[m], q_lo_[U(m)], q_hi_[U(m)]);
    const double v = v_box_[U(m)];
    const double vc = std::clamp(in.qd0[m], -v, v);
    if (qc != in.q0[m] || vc != in.qd0[m]) {
      rec.x0_clamped = true;
      in.q0[m] = qc;
      in.qd0[m] = vc;
    }
  }
  // Reference: the source at the new grid's node instants — a column subset
  // of it, since both grids are anchored at t_c and share the end.
  for (int i = 0; i <= n_total; ++i) {
    const std::int64_t t_i = DecelGridNodeTimeNs(t_eff, t_c, dt_pre_ns_, dt_ns_, n_pre, i);
    if (!NodeTrajectoryFollower::SampleJoints(*src, t_i, q, qd, qdd)) {
      rec.outcome = DecelOutcome::kInputNonFinite;
      return false;
    }
    for (int m = 0; m < nv_; ++m) {
      const auto d = U(device_of_model_[U(m)]);
      in.q_ref(m, i) = q[d];
      in.qd_ref(m, i) = qd[d];
      in.qdd_ref(m, i) = qdd[d];
    }
  }
  in.reference_valid = true;
  const int index = pre ? n_pre : k;
  in.cold_start = ColdStartFor(pre, index, t_eff, rt.plan_id, t_c);
  rec.cold_start = in.cold_start;
  if (pre && !SetCatchInputs(ball.p_b, ball.v_b, ball.a_d, ball, /*first=*/false, in, rec)) {
    rec.outcome = DecelOutcome::kNoBall;
    return false;
  }
  if (perp_on_) {
    if (pre) {
      // This replan's ball: the newer prediction moves the line with it.
      if (!SetStopLine(in, rec)) {
        return false;
      }
    } else {
      in.p_c = ring_line_[src_slot].p_c;
      in.d_hat = ring_line_[src_slot].d_hat;
    }
  }
  bool ok = core.Solve(in, res);
  if (!ok && !pre &&
      (res.reason == DecelMpcReason::kTrustRegionConflict ||
       res.reason == DecelMpcReason::kReferenceNotAtRest)) {
    // The state left the source segment by more than the trust region, or
    // the source does not end at rest: its nodes cannot linearise this stop.
    // Re-solve from nothing. Not cold — the pre-solve's iterates are the
    // warm start the main QP wants.
    // `in` keeps the stop-path line set above.
    in.reference_valid = false;
    in.cold_start = false;
    rec.cold_retry = true;
    ok = core.Solve(in, res);
  }
  const std::int64_t end = clock_();
  NoteSolve(ok, pre, index, t_eff, rt.plan_id, t_c);
  rec.outcome = Judge(res, ok, n_pre, n_total, pre, start, end, replan_ns_, t_eff, rec);
  if (rec.outcome != DecelOutcome::kReady) {
    return false;
  }
  // The plan's track, not the RT's latest: after the freeze they can differ.
  PackSegment(rt, src->token.generation, rt.plan_id, t_c, t_eff, n_pre, pre ? 0 : k, n_total, res,
              out);
  out.x0_clamped = rec.x0_clamped;
  if (!ValidateDecelNodes(out)) {
    rec.outcome = DecelOutcome::kSolveFailed;
    rec.core_reason = DecelMpcReason::kNone;
    return false;
  }
  if (perp_on_) {
    // A catch-core segment carries the line it was solved on; a stop segment
    // inherits its source's (the same one — `in` holds it either way).
    ready_line_ = StopLine{true, in.p_c, in.d_hat};
  }
  return true;
}

}  // namespace rtc::catching
