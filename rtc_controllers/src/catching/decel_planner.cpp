// Decel planner (MPC E1-F03). See decel_planner.hpp.
#include "rtc_controllers/catching/decel_planner.hpp"

#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/traj_sampler.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace rtc::catching {

namespace {

// Wake-to-wake spacing inside which a q̇ difference is read as q̈: shorter is
// one RT tick's noise divided by almost nothing, longer spans a mode change.
constexpr std::int64_t kQddMinSpanNs = 5'000'000;    // 5 ms
constexpr std::int64_t kQddMaxSpanNs = 200'000'000;  // 0.2 s

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

// The first solve's reference reaches its target at this fraction of the
// velocity box η_v·q̇_max (MD-62): the minimum-jerk peak 1.875·|d|/T is held
// below it, so the QP has room to bend the reach toward the catch terms.
constexpr double kRefSpeedFraction = 0.9;

}  // namespace

const char* DecelOutcomeName(DecelOutcome o) noexcept {
  switch (o) {
    case DecelOutcome::kOff:
      return "off";
    case DecelOutcome::kNoState:
      return "no_state";
    case DecelOutcome::kStaleState:
      return "stale_state";
    case DecelOutcome::kNotDue:
      return "not_due";
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
  cores_.clear();
  inputs_.clear();
  results_.clear();
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
      !std::isfinite(consts.control_dt) || !(consts.control_dt > 0.0) || consts.control_dt > 1.0 ||
      !std::isfinite(consts.budget_s) || !(consts.budget_s > 0.0) || consts.budget_s > 1.0 ||
      !std::isfinite(consts.report_lead_s) || consts.report_lead_s < 0.0 ||
      consts.report_lead_s > 1.0) {
    return fail("eta_v, T_arm, control_dt, budget_s or report_lead_s is outside its range");
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
    qdd_cap_[U(m)] = model.qddot_cap[U(m)];
  }
  qdd_cap_valid_ = model.qddot_cap_valid;
  if (qdd_cap_valid_) {
    for (int m = 0; m < n; ++m) {
      if (!std::isfinite(qdd_cap_[U(m)]) || !(qdd_cap_[U(m)] > 0.0)) {
        qdd_cap_valid_ = false;  // a partial box is no box
      }
    }
  }
  cores_.reserve(U(params.k_max + 1));
  inputs_.resize(U(params.k_max + 1));
  results_.resize(U(params.k_max + 1));
  for (int k = 0; k <= params.k_max; ++k) {
    DecelMpcParams mp;
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
  clock_ = clock;
  nv_ = n;
  device_of_model_ = model.device_of_model;
  dt_ns_ = params.DtNs();
  t_pre_ns_ = SecondsToNs(params.t_pre_s);
  t_arm_ns_ = SecondsToNs(consts.t_arm_s);
  report_lead_ns_ = SecondsToNs(consts.report_lead_s);
  budget_ns_ = SecondsToNs(consts.budget_s);
  lead_margin_ns_ = budget_ns_ + 2 * SecondsToNs(consts.control_dt);
  consts_ = consts;
  h_ns_ = SecondsToNs(consts.control_dt);
  for (int m = 0; m < n; ++m) {
    qd_max_[U(m)] = model.qdot_max[U(m)];
    q_lo_[U(m)] = model.q_min[U(m)] + params.m_q;
    q_hi_[U(m)] = model.q_max[U(m)] - params.m_q;
  }
  if (params.n_pre_max > 0) {
    std::string why;
    if (!ConfigureApproach(model, why)) {
      return fail(why);
    }
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
  const DecelMpcParams core_defaults{};
  if (!std::isfinite(consts_.v_eps) || !(consts_.v_eps > 0.0) || dt_pre_ns_ <= 0 ||
      dt_pre_ns_ > kMaxDecelDtPreNs || first_ns_ <= 0 || replan_ns_ <= 0 ||
      params.n_pre_max > kMaxDecelNodes - params.n_nodes ||
      !DecelBlocksFor(params, 0, stop_blocks, stop_n_blocks) ||
      params.n_pre_max > kMaxDecelNodes - stop_n_blocks) {
    why = "the pre-catch grid (n_pre_max, dt_pre_s, the budgets or v_eps) is outside its range";
    return false;
  }
  // The trust region must hold the position margin: with m_q ≥ δ a target at
  // a limit would put the trust row's lower bound above its upper one.
  if (!(params.m_q < core_defaults.delta_tr)) {
    why = "decel_mpc.m_q must be below the core's trust region (" +
          std::to_string(core_defaults.delta_tr) + " rad) with a pre-catch grid";
    return false;
  }
  for (int m = 0; m < n; ++m) {
    const double cap = kRefSpeedFraction * consts_.eta_v * qd_max_[U(m)];
    if (!std::isfinite(cap) || !(cap > 0.0)) {
      why =
          "0.9·eta_v·qdot_max is not a positive finite number for model joint " + std::to_string(m);
      return false;
    }
  }
  rest_tol_ref_ = core_defaults.reference_rest_tol;
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
  // runs.
  const int n = nv_;
  Eigen::VectorXd q_mid(n);
  for (int m = 0; m < n; ++m) {
    q_mid[m] = 0.5 * (model.q_min[U(m)] + model.q_max[U(m)]);
  }
  for (std::size_t k = 0; k < cores_.size(); ++k) {
    DecelMpcInput& in = inputs_[k];
    in.q0 = q_mid;
    in.qd0.setZero();
    in.qdd0.setZero();
    in.reference_valid = false;
    in.cold_start = true;
    if (!cores_[k]->Solve(in, results_[k])) {
      why = "warm-up solve of stop core k = " + std::to_string(k) + ": " +
            DecelMpcReasonName(results_[k].reason);
      return false;
    }
    in.cold_start = false;
  }
  pinocchio::Data data(*model.arm);
  pinocchio::forwardKinematics(*model.arm, data, q_mid);
  pinocchio::updateFramePlacement(*model.arm, data, model.catch_frame);
  const Eigen::Vector3d p = data.oMf[model.catch_frame].translation();
  const Eigen::Vector3d z = data.oMf[model.catch_frame].rotation().col(2);
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
    in.v_b = -5.0 * z;
    in.w_p = params_.w_const * Eigen::Matrix3d::Identity();
    in.gamma_ref = params_.gamma_ref;
    if (!catch_cores_[j]->Solve(in, catch_results_[j])) {
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
  trial_open_ = false;
  trial_plan_id_ = 0;
  trial_t_c_ns_ = 0;
  have_published_ = false;
  prev_valid_ = false;
  qdd_est_valid_ = false;
  ring_n_ = 0;
  ring_plan_id_ = 0;
  ring_t_c_ns_ = 0;
  reported_pending_seq_ = 0;
  reported_active_seq_ = 0;
  last_solve_valid_ = false;
}

void DecelPlanner::NotePublished(const DecelPlanSnapshot& p) noexcept {
  last_ = p;
  have_published_ = true;
}

void DecelPlanner::UpdateAccelEstimate(const PlannerRtState& rt) noexcept {
  // q̈ from the planner's own wake-to-wake difference of the REPORTED q̇
  // (MD-28): a 2 ms difference on the RT side would amplify the CLIK
  // reference's noise and its reseed steps.
  const std::int64_t span = rt.rt_state_ns - prev_rt_ns_;
  qdd_est_valid_ = false;
  // Too close to the previous sample: keep that sample and wait — replacing
  // it would keep every span under the floor when wakes come faster than it
  // (a burst of trajectory signals), and the estimate would never run.
  if (prev_valid_ && span >= 0 && span < kQddMinSpanNs) {
    return;
  }
  if (prev_valid_ && span >= kQddMinSpanNs && span <= kQddMaxSpanNs && qdd_cap_valid_) {
    const double inv = 1e9 / static_cast<double>(span);
    bool finite = true;
    for (int m = 0; m < nv_; ++m) {
      const auto d = U(device_of_model_[U(m)]);
      const double a = (rt.qd_cmd[d] - prev_qd_[d]) * inv;
      finite = finite && std::isfinite(a);
      const double cap = qdd_cap_[U(m)];
      qdd_est_[d] = std::clamp(a, -cap, cap);
    }
    qdd_est_valid_ = finite;
  }
  prev_valid_ = true;
  prev_rt_ns_ = rt.rt_state_ns;
  prev_qd_ = rt.qd_cmd;
}

bool DecelPlanner::PredictX0(const PlannerRtState& rt, std::int64_t t_eff_ns,
                             DecelRecord& rec) noexcept {
  DecelMpcInput& in = inputs_[U(rec.k)];
  // The instant the reported command belongs to on the segment's axis:
  // real → lead (T_arm), then the command's own lead over the tick (MD-40).
  // Path (ii) reads a v1 command the same way although that command sits only
  // δ = 0.2 – 0.8 h past its tick: what has to line up is the RT's switch,
  // which compares the command the tick starts from (the previous tick's,
  // at t − h + δ) with the segment at t + h. Read at + L, the segment is the
  // command's trajectory delayed by L − δ, and the two meet when L = 2h —
  // whatever δ is. L = δ would put 2h between them.
  const std::int64_t t_rep = rt.rt_state_ns + t_arm_ns_ + report_lead_ns_;
  rec.h_s = static_cast<double>(t_eff_ns - t_rep) * 1e-9;
  // Path (i): the RT follows the planner's latest segment — evaluate it at
  // t_eff (exact on the segment). The RT reports it under mode mpc (E1-F04).
  if (have_published_ && rt.decel_active && rt.decel_seq == last_.decel_seq) {
    std::array<double, kMaxDecelNv> q{};
    std::array<double, kMaxDecelNv> qd{};
    std::array<double, kMaxDecelNv> qdd{};
    if (NodeTrajectoryFollower::SampleJoints(last_, t_eff_ns, q, qd, qdd)) {
      for (int m = 0; m < nv_; ++m) {
        const auto d = U(device_of_model_[U(m)]);
        in.q0[m] = q[d];
        in.qd0[m] = qd[d];
        in.qdd0[m] = qdd[d];
      }
      rec.from_segment = true;
    }
  }
  if (!rec.from_segment) {
    // Path (ii): extrapolate the reported command over h with constant q̈.
    const double h = rec.h_s;
    rec.qdd_trusted = qdd_est_valid_;
    for (int m = 0; m < nv_; ++m) {
      const auto d = U(device_of_model_[U(m)]);
      const double a = qdd_est_valid_ ? qdd_est_[d] : 0.0;
      in.q0[m] = rt.q_cmd[d] + rt.qd_cmd[d] * h + 0.5 * a * h * h;
      in.qd0[m] = rt.qd_cmd[d] + a * h;
      in.qdd0[m] = a;
    }
  }
  // Finiteness BEFORE the projection: a NaN passes std::clamp unchanged
  // (every comparison is false), so the projection is no filter for it.
  if (!in.q0.allFinite() || !in.qd0.allFinite() || !in.qdd0.allFinite() ||
      !std::isfinite(rec.h_s)) {
    return false;
  }
  for (int m = 0; m < nv_; ++m) {
    const double v = v_box_[U(m)];
    const double c = std::clamp(in.qd0[m], -v, v);
    if (c != in.qd0[m]) {
      rec.x0_clamped = true;
      in.qd0[m] = c;
    }
  }
  return true;
}

void DecelPlanner::ShiftReference(int k, DecelMpcInput& in) noexcept {
  // The latest segment covers t_c + k0·Δ .. t_c + N_s·Δ; instance k covers
  // t_c + k·Δ .. the same end. Column i of the new reference IS node
  // i + (k − k0) of the old one — an exact copy, no evaluation, no padding.
  const int s = k - last_.k0;
  const int cols = params_.n_nodes - k + 1;
  for (int i = 0; i < cols; ++i) {
    const int src = i + s;
    for (int m = 0; m < nv_; ++m) {
      const auto e = static_cast<std::size_t>(src * kMaxDecelNv + device_of_model_[U(m)]);
      in.q_ref(m, i) = last_.q[e];
      in.qd_ref(m, i) = last_.qd[e];
      in.qdd_ref(m, i) = last_.qdd[e];
    }
  }
  in.reference_valid = true;
}

void DecelPlanner::Pack(const PlannerRtState& rt, int k, std::int64_t t_eff_ns,
                        const DecelMpcResult& r, DecelPlanSnapshot& out) const noexcept {
  out = DecelPlanSnapshot{};
  out.token.activation_generation = rt.activation_generation;
  out.token.generation = rt.track_generation;
  out.rt_iteration = rt.rt_iteration;
  out.rt_state_ns = rt.rt_state_ns;
  out.plan_id = rt.plan_id;
  out.t_c_ns = rt.plan_t_c_ns;
  out.t0_ns = t_eff_ns;
  out.dt_ns = dt_ns_;
  out.k0 = k;
  out.n_nodes = params_.n_nodes - k;
  out.nv = nv_;
  for (int i = 0; i <= out.n_nodes; ++i) {
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

bool DecelPlanner::Plan(const PlannerRtState& rt, DecelPlanSnapshot& out,
                        DecelRecord& rec) noexcept {
  rec = DecelRecord{};
  if (!configured_) {
    return false;
  }
  if (!rt.valid || !rt.plan_active || rt.plan_t_c_ns <= 0 || !rt.cmd_seeded || rt.nv != nv_) {
    rec.outcome = DecelOutcome::kNoState;
    return false;
  }
  // A stop belongs to one catch plan: a new followed plan (or t_c) is a new
  // stop, and nothing of the previous one carries over.
  if (!trial_open_ || rt.plan_id != trial_plan_id_ || rt.plan_t_c_ns != trial_t_c_ns_) {
    ResetTrial();
    trial_open_ = true;
    trial_plan_id_ = rt.plan_id;
    trial_t_c_ns_ = rt.plan_t_c_ns;
  }
  UpdateAccelEstimate(rt);

  // The budget is measured from HERE (MD-26), not from the wake: a search
  // wake's tail must not count against the decel solve.
  const std::int64_t start = clock_();
  const std::int64_t now_lead = start + t_arm_ns_;
  const std::int64_t state_age = start - rt.rt_state_ns;
  if (state_age < 0 || state_age > kDecelMaxRtStateAgeNs) {
    rec.outcome = DecelOutcome::kStaleState;
    return false;
  }
  const std::int64_t t_c = rt.plan_t_c_ns;
  if (!have_published_ && t_c - now_lead > t_pre_ns_) {
    rec.outcome = DecelOutcome::kNotDue;
    return false;
  }
  const std::int64_t earliest = now_lead + lead_margin_ns_;
  const std::int64_t k64 = earliest <= t_c ? 0 : CeilDiv(earliest - t_c, dt_ns_);
  if (k64 > params_.k_max) {
    rec.k = static_cast<std::int32_t>(std::min<std::int64_t>(k64, 1'000'000));
    rec.outcome = DecelOutcome::kPastReplanWindow;
    return false;
  }
  const int k = static_cast<int>(k64);
  rec.k = k;
  rec.n_nodes = params_.n_nodes - k;
  if (have_published_ && k <= last_.k0) {
    rec.outcome = DecelOutcome::kUpToDate;
    return false;
  }
  const std::int64_t t_eff = t_c + k64 * dt_ns_;
  if (!PredictX0(rt, t_eff, rec)) {
    rec.outcome = DecelOutcome::kInputNonFinite;
    return false;
  }

  DecelMpc& core = *cores_[U(k)];
  DecelMpcInput& in = inputs_[U(k)];
  DecelMpcResult& res = results_[U(k)];
  in.reference_valid = false;
  if (have_published_) {
    ShiftReference(k, in);
  }
  bool ok = core.Solve(in, res);
  if (!ok && in.reference_valid &&
      (res.reason == DecelMpcReason::kTrustRegionConflict ||
       res.reason == DecelMpcReason::kReferenceNotAtRest)) {
    // The RT's state left the published segment by more than the trust
    // region (it is not following it yet, or the prediction moved): the
    // shifted reference cannot linearise this stop. Re-solve from nothing.
    in.reference_valid = false;
    rec.cold_retry = true;
    ok = core.Solve(in, res);
  }
  const std::int64_t end = clock_();
  rec.solve_ns = end - start;
  rec.core_reason = res.reason;
  rec.presolved = res.presolved;
  rec.iterations = res.iterations + res.presolve_iterations;
  rec.qp_status = res.qp_status;
  if (!ok) {
    rec.outcome = DecelOutcome::kSolveFailed;
    return false;
  }
  rec.slack_max = res.slack_max;
  rec.slack_terminal_max = res.slack_terminal_max;
  rec.tau_ratio_max =
      res.torque_evaluated ? res.tau_ratio_max : std::numeric_limits<double>::quiet_NaN();
  if (rec.solve_ns > budget_ns_) {
    rec.outcome = DecelOutcome::kBudget;
    return false;
  }
  // Unreachable while the budget check above holds (t_eff ≥ start_lead +
  // budget + 2·control_dt); kept as the guard for that arithmetic.
  if (end + t_arm_ns_ >= t_eff) {
    rec.outcome = DecelOutcome::kLate;
    return false;
  }
  // MD-33: positive comparisons only — `!(x > thr)` would let a NaN through.
  const bool slack_ok = std::isfinite(res.slack_max) && res.slack_max <= params_.slack_max &&
                        std::isfinite(res.slack_terminal_max) &&
                        res.slack_terminal_max <= params_.slack_terminal_max;
  if (!slack_ok) {
    rec.outcome = DecelOutcome::kSlack;
    return false;
  }
  Pack(rt, k, t_eff, res, out);
  out.x0_clamped = rec.x0_clamped;
  rec.outcome = DecelOutcome::kReady;
  return true;
}

// ── APPROACH–stop (E1-F08) ────────────────────────────────────────────────────

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
  }
  // Every check passes on a positive comparison, so a NaN fails it.
  if (!(rec.solve_ns <= budget_ns)) {
    return DecelOutcome::kBudget;
  }
  if (!(end + t_arm_ns_ + 2 * h_ns_ < t_eff)) {
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
  if (params_.shadow) {
    return ring_[U(ring_n_ - 1)].decel_seq;
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

void DecelPlanner::NoteApproachPublished(const DecelPlanSnapshot& p) noexcept {
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
    }
    --ring_n_;
  }
  ring_[U(ring_n_)] = p;
  ++ring_n_;
}

bool DecelPlanner::ColdStartFor(bool catch_core, int index, std::int64_t t_eff,
                                std::uint32_t plan_id, std::int64_t t_c) const noexcept {
  return !(last_solve_valid_ && last_solve_catch_ == catch_core && last_solve_index_ == index &&
           last_solve_t_eff_ == t_eff && last_solve_plan_id_ == plan_id && last_solve_t_c_ == t_c);
}

void DecelPlanner::NoteSolve(bool catch_core, int index, std::int64_t t_eff, std::uint32_t plan_id,
                             std::int64_t t_c) noexcept {
  last_solve_valid_ = true;
  last_solve_catch_ = catch_core;
  last_solve_index_ = index;
  last_solve_t_eff_ = t_eff;
  last_solve_plan_id_ = plan_id;
  last_solve_t_c_ = t_c;
}

bool DecelPlanner::PlanFirst(const PlannerRtState& rt, const PlanSnapshot& plan,
                             const DecelBallTarget& ball, DecelPlanSnapshot& out,
                             DecelRecord& rec) noexcept {
  rec = DecelRecord{};
  if (!ApproachConfigured()) {
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
    const double q0 = rt.q_cmd[d];
    const double target = plan.q_star[d];
    if (!std::isfinite(q0) || !std::isfinite(target)) {
      finite = false;
      break;
    }
    in.q0[m] = q0;
    in.qd0[m] = 0.0;
    in.qdd0[m] = 0.0;
    const double clamped = std::clamp(target, q_lo_[U(m)], q_hi_[U(m)]);
    rec.ref_clamped = rec.ref_clamped || clamped != target;
    double step = clamped - q0;
    const double allowed = kRefSpeedFraction * v_box_[U(m)] * t_span / 1.875;
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
  // No retry: a catch core cannot be solved without its reference.
  const bool ok = core.Solve(in, res);
  const std::int64_t end = clock_();
  NoteSolve(true, n_pre, t_eff, plan.plan_id, plan.t_c_ns);
  rec.outcome = Judge(res, ok, n_pre, n_total, true, start, end, first_ns_, t_eff, rec);
  if (rec.outcome != DecelOutcome::kReady) {
    return false;
  }
  PackSegment(rt, plan.token.generation, plan.plan_id, plan.t_c_ns, t_eff, n_pre, 0, n_total, res,
              out);
  if (!ValidateDecelNodes(out)) {
    rec.outcome = DecelOutcome::kSolveFailed;
    rec.core_reason = DecelMpcReason::kNone;
    return false;
  }
  // A new plan: the RT reports nothing of it yet.
  reported_pending_seq_ = 0;
  reported_active_seq_ = 0;
  return true;
}

bool DecelPlanner::Replan(const PlannerRtState& rt, const DecelBallTarget& ball,
                          DecelPlanSnapshot& out, DecelRecord& rec) noexcept {
  rec = DecelRecord{};
  if (!ApproachConfigured()) {
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

  const std::size_t slot = pre ? U(n_pre - 1) : U(k);
  DecelMpc& core = pre ? *catch_cores_[slot] : *cores_[slot];
  DecelMpcInput& in = pre ? catch_inputs_[slot] : inputs_[slot];
  DecelMpcResult& res = pre ? catch_results_[slot] : results_[slot];

  // x₀: the source at t_eff (MD-28 path (i)), projected into the core's box —
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
    const std::int64_t t_i =
        pre ? (i <= n_pre ? t_eff + static_cast<std::int64_t>(i) * dt_pre_ns_
                          : t_c + static_cast<std::int64_t>(i - n_pre) * dt_ns_)
            : t_eff + static_cast<std::int64_t>(i) * dt_ns_;
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
  bool ok = core.Solve(in, res);
  if (!ok && !pre &&
      (res.reason == DecelMpcReason::kTrustRegionConflict ||
       res.reason == DecelMpcReason::kReferenceNotAtRest)) {
    // As the stop-segment planner: re-solve from nothing. Not cold — the
    // pre-solve's iterates are the warm start the main QP wants.
    in.reference_valid = false;
    in.cold_start = false;
    rec.cold_retry = true;
    ok = core.Solve(in, res);
  }
  const std::int64_t end = clock_();
  NoteSolve(pre, index, t_eff, rt.plan_id, t_c);
  rec.outcome = Judge(res, ok, n_pre, n_total, pre, start, end, replan_ns_, t_eff, rec);
  if (rec.outcome != DecelOutcome::kReady) {
    return false;
  }
  PackSegment(rt, rt.track_generation, rt.plan_id, t_c, t_eff, n_pre, pre ? 0 : k, n_total, res,
              out);
  out.x0_clamped = rec.x0_clamped;
  if (!ValidateDecelNodes(out)) {
    rec.outcome = DecelOutcome::kSolveFailed;
    rec.core_reason = DecelMpcReason::kNone;
    return false;
  }
  return true;
}

}  // namespace rtc::catching
