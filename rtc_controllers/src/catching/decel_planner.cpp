// Decel planner (MPC E1-F03). See decel_planner.hpp.
#include "rtc_controllers/catching/decel_planner.hpp"

#include "rtc_controllers/catching/node_follower.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

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

}  // namespace

const char* DecelOutcomeName(DecelOutcome o) noexcept {
  switch (o) {
    case DecelOutcome::kOff:
      return "off";
    case DecelOutcome::kNoState:
      return "no_state";
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
  }
  return "unknown";
}

bool DecelPlanner::Configure(const DecelPlannerModel& model, const DecelPlannerConstants& consts,
                             const DecelPlannerParams& params, ClockFn clock, std::string* error) {
  configured_ = false;
  cores_.clear();
  inputs_.clear();
  results_.clear();
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
  if (!std::isfinite(consts.eta_v) || !(consts.eta_v > 0.0) || consts.eta_v > 1.0 ||
      !std::isfinite(consts.t_arm_s) || consts.t_arm_s < 0.0 || !std::isfinite(consts.control_dt) ||
      !(consts.control_dt > 0.0) || !std::isfinite(consts.budget_s) || !(consts.budget_s > 0.0)) {
    return fail("eta_v, T_arm, control_dt or budget_s is outside its range");
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
  }
  params_ = params;
  clock_ = clock;
  nv_ = n;
  device_of_model_ = model.device_of_model;
  dt_ns_ = params.DtNs();
  t_pre_ns_ = SecondsToNs(params.t_pre_s);
  t_arm_ns_ = SecondsToNs(consts.t_arm_s);
  budget_ns_ = SecondsToNs(consts.budget_s);
  lead_margin_ns_ = budget_ns_ + 2 * SecondsToNs(consts.control_dt);
  configured_ = true;
  return true;
}

void DecelPlanner::ResetTrial() noexcept {
  trial_open_ = false;
  trial_plan_id_ = 0;
  trial_t_c_ns_ = 0;
  have_published_ = false;
  prev_valid_ = false;
  qdd_est_valid_ = false;
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
  const std::int64_t t_rep = rt.rt_state_ns + t_arm_ns_;
  rec.h_s = static_cast<double>(t_eff_ns - t_rep) * 1e-9;
  // Path (i): the RT follows the planner's latest segment — evaluate it at
  // t_eff (exact on the segment). Only reachable once E1-F04 reports it.
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
  // Finiteness BEFORE the projection: std::clamp would launder a NaN into a
  // bound.
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
  out.tau_ratio_max = r.tau_ratio_max;
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

}  // namespace rtc::catching
