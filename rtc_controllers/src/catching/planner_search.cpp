// The planner's search (S6-B). See planner_search.hpp.
#include "rtc_controllers/catching/planner_search.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>

namespace rtc::catching {

namespace {

constexpr double kSecondsToNs = 1e9;

[[nodiscard]] std::int64_t SecondsToNs(double s) noexcept {
  return static_cast<std::int64_t>(std::llround(s * kSecondsToNs));
}

[[nodiscard]] PlanReason ReasonFor(JudgeReject r) noexcept {
  switch (r) {
    case JudgeReject::kInput:
      return PlanReason::kInputNonFinite;
    case JudgeReject::kIk:
      return PlanReason::kIkFailed;
    case JudgeReject::kManipulability:
      return PlanReason::kManipulability;
    case JudgeReject::kWorkspace:
      // The W_catch gate (L3 §4.9 — p_c AND p_stop inside catch_box). The
      // frozen message (D-20) has no separate workspace reason; which of the
      // two left the box is in planner_events.csv (rej_workspace).
      return PlanReason::kStoppingDistance;
    case JudgeReject::kNotEvaluated:
      return PlanReason::kBudgetExceeded;
    case JudgeReject::kNone:
      return PlanReason::kNone;
  }
  return PlanReason::kNone;
}

}  // namespace

double PlannerSearch::SwitchStep(const TrajectorySnapshot& traj, const PlannerRtState& rt,
                                 NowLead now_lead, double dp) const noexcept {
  const Eigen::Vector3d p_c(current_.p_c[0], current_.p_c[1], current_.p_c[2]);
  // The ramp terms need the ball target at the adoption instant; a target the
  // sampler cannot give fails the rule closed (NaN).
  const auto bound_at = [&](std::int64_t t_ns, double g, double gd, double gdd) noexcept {
    double xo_norm = 0.0;
    double v_o_norm = 0.0;
    if (gd != 0.0 || gdd != 0.0) {
      const SampleEval o = SampleAt(traj, NowLead{t_ns});
      xo_norm = o.valid ? (o.p - p_c).norm() : std::numeric_limits<double>::quiet_NaN();
      v_o_norm = o.valid ? o.v.norm() : 0.0;
    }
    return SwitchAccelStepBound(constants_.ref_omega, constants_.ref_zeta, g, gd, gdd, dp, xo_norm,
                                v_o_norm);
  };
  if (!rt.ref_valid) {
    return bound_at(now_lead.ns, 0.0, 0.0, 0.0);
  }
  if (!rt.ramp_valid || !(rt.ramp_t1_ns > rt.ramp_t0_ns)) {
    return bound_at(now_lead.ns, rt.gamma, rt.gamma_d, rt.gamma_dd);
  }
  // The RT adopts a publish of THIS cycle on its first tick after it: within
  // the search budget of the cycle's start, plus two ticks of slack. Over
  // that window the followed ramp can start — γ̇ = γ̈ = 0 on the snapshot's
  // tick is not the adoption's (2026-09-23 /code-review) — so the bound is
  // the worst over kSwitchSamples instants of the ramp the RT is running.
  const GammaProfile ramp{rt.ramp_g0, rt.ramp_gf, 0.0,
                          static_cast<double>(rt.ramp_t1_ns - rt.ramp_t0_ns) * kNsToS};
  const std::int64_t span = SecondsToNs(params_.budget_s + 2.0 * constants_.control_dt);
  double worst = 0.0;
  for (int i = 0; i < kSwitchSamples; ++i) {
    const std::int64_t t_ns = now_lead.ns + span * i / (kSwitchSamples - 1);
    double g = 0.0;
    double gd = 0.0;
    double gdd = 0.0;
    ramp.Eval(static_cast<double>(t_ns - rt.ramp_t0_ns) * kNsToS, g, gd, gdd);
    const double b = bound_at(t_ns, g, gd, gdd);
    if (std::isnan(b)) {
      return b;  // unjudgeable at one instant = unjudgeable
    }
    worst = std::max(worst, b);
  }
  return worst;
}

bool PlannerSearch::Configure(const PlannerModel& model, const PlannerConstants& constants,
                              const PlannerParams& params, const CatchPoseIkOptions& ik,
                              ClockFn clock) {
  configured_ = false;
  if (model.handle == nullptr || model.nv <= 0 || model.nv > static_cast<int>(kMaxPlanNv) ||
      clock == nullptr) {
    return false;
  }
  if (params.wait_pose_n != model.nv) {
    return false;
  }
  model_ = model;
  constants_ = constants;
  params_ = params;
  ik_options_ = ik;
  clock_ = clock;
  ik_.Resize(model.nv);
  unit_speed_.Resize(model.nv);
  // The rollout's reference: the profile's ω and ζ, NO saturation (§4.8 —
  // the peaks are what is judged against η_a a_max / η_v v_max).
  rollout_ds_.emplace(
      SoftCatchTranslation::Params{constants.ref_omega, constants.ref_zeta, 1e9, 1e9});
  rollout_ = RolloutSettings{};
  rollout_.a_max = constants.ref_a_max;
  rollout_.v_max = constants.v_max;
  rollout_.eta_a = params.eta_a;
  rollout_.eta_v = constants.eta_v;
  rollout_.eps_term = params.eps_term;
  rollout_.dt_coarse = params.rollout_dt_coarse;
  rollout_.dt_fine = constants.control_dt;
  // Spans into params_ (this object's copy), never into the argument.
  rollout_.gamma_grid = std::span<const double>(params_.gamma_grid.data(), params_.gamma_grid_n);
  rollout_.window_grid = std::span<const double>(params_.window_grid.data(), params_.window_grid_n);
  ik_cost_ns_ = 0;
  rollout_cost_ns_ = 0;
  // The wait pose arrives in DEVICE order (the YAML is written against the
  // arm's joint list); the IK seed is in model order.
  seed_.setZero(model.nv);
  for (int j = 0; j < model.nv; ++j) {
    seed_[j] = params.wait_pose[static_cast<std::size_t>(
        model.device_of_model[static_cast<std::size_t>(j)])];
  }
  ResetTrial();
  configured_ = true;
  return true;
}

void PlannerSearch::ResetTrial() noexcept {
  current_ = Current{};
  published_.fill(Current{});
  published_next_ = 0;
  track_known_ = false;
  track_generation_ = 0;
  sequence_seen_ = false;
  last_sequence_ = 0;
  settle_seen_ = 0;
}

void PlannerSearch::NotePublished(const PlanSnapshot& plan) noexcept {
  if (!plan.valid) {
    return;  // "no plan" leaves the RT's current plan (if any) where it was
  }
  Current& slot = published_[published_next_];
  published_next_ = (published_next_ + 1) % kPublishedRing;
  slot.valid = true;
  slot.plan_id = plan.plan_id;
  slot.t_c_ns = plan.t_c_ns;
  slot.p_c = plan.p_c;
}

PlannerSearch::Current PlannerSearch::Followed(const PlannerRtState& rt) const noexcept {
  if (!rt.plan_active) {
    return Current{};
  }
  // Newest first: a re-publish of the same id (tests do this) wins.
  for (std::size_t i = 0; i < kPublishedRing; ++i) {
    const std::size_t idx = (published_next_ + kPublishedRing - 1 - i) % kPublishedRing;
    if (published_[idx].valid && published_[idx].plan_id == rt.plan_id) {
      return published_[idx];
    }
  }
  return Current{};  // the RT follows a plan we did not publish (the oracle)
}

double PlannerSearch::SigmaMax(const CovarianceSnapshot& cov, int k) noexcept {
  if (!cov.valid || k < 0 || k >= cov.n || k >= static_cast<int>(kCap)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto& c = cov.c[static_cast<std::size_t>(k)];
  Eigen::Matrix3d s;
  // Row-major 6×6 (p_x..v_z); the position block is the top-left 3×3.
  for (int r = 0; r < 3; ++r) {
    for (int q = 0; q < 3; ++q) {
      s(r, q) = c[static_cast<std::size_t>(r * 6 + q)];
    }
  }
  if (!s.allFinite()) {
    return std::numeric_limits<double>::quiet_NaN();  // NaN is "not known" (L1)
  }
  const Eigen::Matrix3d sym = 0.5 * (s + s.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
  es.computeDirect(sym, Eigen::EigenvaluesOnly);
  const double lmax = es.eigenvalues().maxCoeff();
  return std::isfinite(lmax) ? std::sqrt(std::max(lmax, 0.0))
                             : std::numeric_limits<double>::quiet_NaN();
}

void PlannerSearch::Monitor(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                            bool cov_matched, const PlannerRtState& rt,
                            SearchStats& stats) const noexcept {
  stats = SearchStats{};
  stats.publish = false;
  const Current followed = Followed(rt);
  if (!followed.valid || !cov_matched || !traj.valid) {
    return;
  }
  // The sample nearest the committed t_c — the covariance is on the vision
  // grid and is not interpolated (A-S5-5; the ν̄/interpolation rule is S7).
  const int n = std::clamp(traj.n, 0, static_cast<int>(kCap));
  int best = -1;
  std::int64_t best_d = std::numeric_limits<std::int64_t>::max();
  for (int k = 0; k < n; ++k) {
    const std::int64_t d = std::llabs(traj.s[static_cast<std::size_t>(k)].t_ns - followed.t_c_ns);
    if (d < best_d) {
      best_d = d;
      best = k;
    }
  }
  if (best >= 0) {
    stats.sigma_l = SigmaMax(cov, best);
  }
}

PlanSnapshot PlannerSearch::Plan(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                 bool cov_matched, const PlannerRtState& rt, NowReal now,
                                 SearchStats& stats) noexcept {
  stats = SearchStats{};
  const std::int64_t t_start = clock_ != nullptr ? clock_() : 0;
  PlanSnapshot plan{};
  plan.token = traj.token;
  plan.token.activation_generation = rt.activation_generation;
  plan.rt_iteration = rt.rt_iteration;
  plan.rt_state_ns = rt.rt_state_ns;
  plan.valid = false;
  plan.reason = PlanReason::kNone;
  current_ = Followed(rt);
  // While the RT follows one of our plans, a cycle that ends without a
  // candidate HOLDS rather than publishing "no plan": that publish would
  // overwrite a replacement the RT has not loaded yet and log a no_current
  // decision against a plan being followed (2026-09-23 /code-review).
  const auto hold_if_following = [this, &stats] {
    if (current_.valid) {
      stats.publish = false;
      stats.decision = SwitchDecision::kHeldNoCandidate;
    }
  };
  if (!configured_ || !traj.valid) {
    hold_if_following();
    return plan;
  }

  // ── Settle after a track change (§4.4) ────────────────────────────────────
  if (!track_known_ || traj.token.generation != track_generation_) {
    track_known_ = true;
    track_generation_ = traj.token.generation;
    settle_seen_ = 0;
    sequence_seen_ = false;
  }
  if (!sequence_seen_ || traj.token.snapshot_sequence != last_sequence_) {
    sequence_seen_ = true;
    last_sequence_ = traj.token.snapshot_sequence;
    ++settle_seen_;
  }
  if (settle_seen_ <= params_.n_settle) {
    stats.settling = true;
    plan.reason = PlanReason::kUncertainty;
    hold_if_following();
    stats.search_ns = clock_() - t_start;
    return plan;
  }

  const int n = std::clamp(traj.n, 0, static_cast<int>(kCap));
  // Thin the vision grid to slice.dt (never interpolate at S6-B).
  int stride = 1;
  if (n >= 2) {
    const double spacing = static_cast<double>(traj.s[1].t_ns - traj.s[0].t_ns) * kNsToS;
    if (spacing > 0.0 && std::isfinite(spacing)) {
      stride = std::max(1, static_cast<int>(std::lround(params_.slice_dt / spacing)));
    }
  }
  const double lead_min = params_.LeadMin();
  const double lead_max = params_.slice_t_max;
  const double r_cap = params_.r_cap;
  const double kappa_rcap = params_.kappa_sigma * r_cap;
  const auto& w = params_.score;

  // ── Candidates and the cheap terms ────────────────────────────────────────
  int m = 0;
  for (int k = 0; k < n && m < static_cast<int>(kCap); k += stride) {
    const TrajSample& s = traj.s[static_cast<std::size_t>(k)];
    const double lead = static_cast<double>(s.t_ns - now.ns) * kNsToS;
    if (!(lead >= lead_min) || !(lead <= lead_max)) {
      continue;  // NaN lead_min (T_freeze unset) keeps every candidate out
    }
    Candidate& c = cands_[static_cast<std::size_t>(m)];
    c = Candidate{};
    c.k = k;
    c.lead_s = lead;
    ++stats.n_in_window;
    const Eigen::Vector3d p(s.p[0], s.p[1], s.p[2]);
    const Eigen::Vector3d v(s.v[0], s.v[1], s.v[2]);
    const double speed = v.norm();
    if (!p.allFinite() || !v.allFinite() || !std::isfinite(speed) || speed < ik_options_.v_eps) {
      c.reject = JudgeReject::kInput;
    } else if (!params_.catch_box.Contains(p.x(), p.y(), p.z())) {
      c.reject = JudgeReject::kWorkspace;
    } else {
      c.reject = JudgeReject::kNotEvaluated;  // until IK runs on it
    }
    c.sigma = cov_matched ? SigmaMax(cov, k) : std::numeric_limits<double>::quiet_NaN();
    c.sigma_known = std::isfinite(c.sigma);
    const bool unc_fail = !c.sigma_known || !(c.sigma <= kappa_rcap);
    c.pre_score = (c.sigma_known && r_cap > 0.0 ? w.w_sigma * c.sigma / r_cap : 0.0) +
                  w.w_late * (lead_max - lead) + (unc_fail ? w.penalty : 0.0);
    ++m;
  }
  if (m == 0) {
    plan.reason = PlanReason::kHorizonShort;
    hold_if_following();
    stats.search_ns = clock_() - t_start;
    return plan;
  }

  // ── Pre-filter: IK runs on the best `max_ik` by pre-score (R-2) ───────────
  int n_order = 0;
  for (int i = 0; i < m; ++i) {
    if (cands_[static_cast<std::size_t>(i)].reject == JudgeReject::kNotEvaluated) {
      order_[static_cast<std::size_t>(n_order++)] = i;
    }
  }
  std::sort(order_.begin(), order_.begin() + n_order, [this](int a, int b) {
    return cands_[static_cast<std::size_t>(a)].pre_score <
           cands_[static_cast<std::size_t>(b)].pre_score;
  });
  // Half a slice: how close a candidate's instant must be to count as "the
  // same" catch instant (the grid moves with every snapshot).
  const double half_slice_ns =
      0.5 * static_cast<double>(stride) *
      (n >= 2 ? static_cast<double>(traj.s[1].t_ns - traj.s[0].t_ns) : 0.0);
  // The followed plan's candidate goes FIRST: the switching rule needs its
  // verdict, and left to the pre-score it can fall outside max_ik or the
  // budget and read as "infeasible" (2026-09-23 /code-review).
  if (current_.valid) {
    for (int oi = 0; oi < n_order; ++oi) {
      const auto& c = cands_[static_cast<std::size_t>(order_[static_cast<std::size_t>(oi)])];
      const auto t_k = traj.s[static_cast<std::size_t>(c.k)].t_ns;
      if (std::fabs(static_cast<double>(t_k - current_.t_c_ns)) <= half_slice_ns) {
        std::rotate(order_.begin(), order_.begin() + oi, order_.begin() + oi + 1);
        break;
      }
    }
  }
  const int n_ik_max = std::min(n_order, params_.max_ik);

  // RT state into model order.
  const int nv = model_.nv;
  for (int j = 0; j < nv; ++j) {
    const auto d = static_cast<std::size_t>(model_.device_of_model[static_cast<std::size_t>(j)]);
    q0_[static_cast<std::size_t>(j)] = rt.q_cmd[d];
    w0_[static_cast<std::size_t>(j)] = rt.cmd_seeded ? rt.qd_cmd[d] : 0.0;
    qdot_plan_[static_cast<std::size_t>(j)] =
        constants_.eta_v * model_.qdot_max[static_cast<std::size_t>(j)];
  }
  const auto nvs = static_cast<std::size_t>(nv);
  const std::span<const double> q0(q0_.data(), nvs);
  const std::span<const double> w0(w0_.data(), nvs);
  const std::span<const double> qdot_plan(qdot_plan_.data(), nvs);
  const std::span<const double> qddot(model_.qddot_max.data(), model_.accel_box ? nvs : 0U);
  const NowLead now_lead = MakeNowLead(now, SecondsToNs(constants_.t_arm_s));
  const double v_tcp_plan = PlanningTcpSpeed(constants_.eta_v, constants_.v_max);
  const double commit_lead = constants_.t_close_total + constants_.t_arm_s + params_.time_margin;
  const std::int64_t budget_ns = SecondsToNs(params_.budget_s);

  // Where the reference starts (§4.8): its own state when it is running (a
  // replacement continues from it), else the catch frame at the current
  // command, at rest — what the RT seeds the reference with on adoption.
  Eigen::Vector3d x0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d xd0 = Eigen::Vector3d::Zero();
  if (rt.ref_valid) {
    x0 = Eigen::Vector3d(rt.ref_x[0], rt.ref_x[1], rt.ref_x[2]);
    xd0 = Eigen::Vector3d(rt.ref_xd[0], rt.ref_xd[1], rt.ref_xd[2]);
  } else {
    model_.handle->ComputeForwardKinematics(q0);
    x0 = model_.handle->GetFramePosition(model_.catch_frame);
  }
  const double g0 = (current_.valid && rt.ref_valid) ? rt.gamma : 0.0;

  // Best so far.
  int best = -1;
  double best_score = 0.0;
  double best_gamma = 0.0;
  double best_tw = 0.0;
  bool best_window_only = false;
  double best_gmin = 0.0;
  std::uint16_t best_mask = 0;
  std::array<double, kMaxPlanNv> best_q{};
  double best_w5 = 0.0;
  double best_w6 = 0.0;

  for (int oi = 0; oi < n_ik_max; ++oi) {
    // Does the NEXT candidate still fit? Its cost is estimated from recent
    // ones (IK + rollout), so the cycle overruns by at most the estimate's
    // error, not by a whole candidate (G3-C: p99 < budget_s).
    // The first candidate always runs: the estimates only decay when an IK
    // runs, so a single slow solve (> budget) would otherwise stop every later
    // cycle at the first check and never relax (2026-09-23 /code-review).
    const std::int64_t elapsed = clock_() - t_start;
    if (oi > 0 && elapsed + ik_cost_ns_ + rollout_cost_ns_ > budget_ns) {
      stats.budget_hit = true;
      break;
    }
    Candidate& c = cands_[static_cast<std::size_t>(order_[static_cast<std::size_t>(oi)])];
    const TrajSample& s = traj.s[static_cast<std::size_t>(c.k)];
    const Eigen::Vector3d p(s.p[0], s.p[1], s.p[2]);
    const Eigen::Vector3d v(s.v[0], s.v[1], s.v[2]);

    const std::int64_t ik_t0 = clock_();
    const CatchPoseIkResult ik =
        ik_.Solve(*model_.handle, model_.catch_frame, p, v, seed_, ik_options_);
    const std::int64_t ik_ns = clock_() - ik_t0;
    stats.ik_ns_max = std::max(stats.ik_ns_max, ik_ns);
    // A decaying maximum: one slow solve raises the estimate at once, and it
    // relaxes back over a few cycles rather than pinning the budget forever.
    ik_cost_ns_ = std::max(ik_ns, ik_cost_ns_ - ik_cost_ns_ / 8);
    ++stats.n_ik;
    if (!ik.accepted) {
      c.reject = ik.reason == CatchPoseReason::kBelowManipMin ? JudgeReject::kManipulability
                                                              : JudgeReject::kIk;
      continue;
    }
    for (int j = 0; j < nv; ++j) {
      q_star_[static_cast<std::size_t>(j)] = ik.q[j];
    }

    // q̇ᵘ at q* and the shared rank-gate core (G3-I: the map's function).
    const double speed = v.norm();
    const Eigen::Vector3d v_hat = v / speed;
    const UnitSpeedResult us = unit_speed_.Compute(*model_.handle, model_.catch_frame,
                                                   std::span<const double>(q_star_.data(), nvs),
                                                   v_hat, std::span<double>(qdot_u_.data(), nvs));
    if (!us.valid) {
      std::fill(qdot_u_.begin(), qdot_u_.begin() + nv, 0.0);  // → `undetermined`, a rank fail
    }
    RankGateInputs in;
    in.q0 = q0;
    in.w0 = w0;
    in.q_star = std::span<const double>(q_star_.data(), nvs);
    in.qdot_plan = qdot_plan;
    in.qddot_max = qddot;
    in.qdot_u = std::span<const double>(qdot_u_.data(), nvs);
    in.p_c = p;
    in.v_ball = v;
    in.jp_qdot_u = us.valid ? us.jp_qdot_u : Eigen::Vector3d::Zero();
    in.t_k = BallTime{s.t_ns};
    in.now_lead = now_lead;
    in.t_margin_ns = SecondsToNs(params_.time_margin);
    in.v_tcp_plan = v_tcp_plan;
    in.d_eff = params_.d_eff;
    in.t_close_total = constants_.t_close_total;
    in.gamma_margin = params_.gamma_margin;
    in.a_dec = constants_.a_dec;
    const RankGateResult r = JudgeRankGates(in);

    // γ_f and T_w from the rollout (§4.8, S6-C). The grid is restricted to
    // the γ window; an EMPTY window (g_min > g_max) is searched at the arm's
    // limit g_max — the most the arm can give, already a rank failure.
    double gamma_f = 0.0;
    double t_w = LongestWindow(rollout_.window_grid);
    bool rollout_ok = false;
    bool window_only = false;
    if (r.gamma_usable && rollout_ds_.has_value()) {
      const double g_lo = std::min(r.window.g_min, r.window.g_max);
      const std::int64_t ro_t0 = clock_();
      const RolloutChoice rc = ChooseGamma(*rollout_ds_, traj, x0, xd0, p, now_lead,
                                           BallTime{s.t_ns}, g0, g_lo, r.window.g_max, rollout_);
      const std::int64_t ro_ns = clock_() - ro_t0;
      stats.rollout_ns_max = std::max(stats.rollout_ns_max, ro_ns);
      stats.n_rollouts = static_cast<std::uint16_t>(stats.n_rollouts + rc.rollouts);
      rollout_cost_ns_ = std::max(ro_ns, rollout_cost_ns_ - rollout_cost_ns_ / 8);
      gamma_f = std::clamp(rc.gamma_f, 0.0, 1.0);
      t_w = rc.t_w;
      rollout_ok = rc.accepted;
      window_only = rc.window_only;
    }
    // Judgement: where the arm stops after the catch must be in the box.
    const StoppingReservation stop = StoppingPoint(p, v, gamma_f, constants_.a_dec);
    if (!stop.valid ||
        !params_.catch_box.Contains(stop.p_stop.x(), stop.p_stop.y(), stop.p_stop.z())) {
      c.reject = JudgeReject::kWorkspace;
      continue;
    }

    // ── Rank gates (decision D) ─────────────────────────────────────────────
    std::uint16_t mask = 0;
    if (!c.sigma_known || !(c.sigma <= kappa_rcap)) {
      mask |= kRankUncertainty;
    }
    if (!r.reach_ok) {
      mask |= kRankReach;
    }
    if (!r.gamma_ok) {
      mask |= kRankGamma;
    }
    if (!(c.lead_s >= commit_lead)) {
      mask |= kRankCommitLead;
    }
    const double sigma_gap = c.sigma_known
                                 ? CatchErrorSigma(gamma_f, c.sigma, c.sigma, params_.sigma_trk,
                                                   speed, params_.clock_err)
                                 : std::numeric_limits<double>::quiet_NaN();
    if (!(params_.n_sigma * sigma_gap <= r_cap)) {
      mask |= kRankErrorBudget;
    }
    if (!rollout_ok) {
      mask |= kRankRollout;
    }

    // ── Score (§4.10 + penalties) ───────────────────────────────────────────
    double score = w.w_late * (lead_max - c.lead_s) - w.w_gamma * gamma_f;
    if (c.sigma_known && r_cap > 0.0) {
      score += w.w_sigma * c.sigma / r_cap;
    }
    const double avail = c.lead_s - constants_.t_arm_s;
    if (r.reach.Usable() && std::isfinite(r.reach.t) && avail > 0.0) {
      score += w.w_t * r.reach.t / avail;
    }
    double dq2 = 0.0;
    for (int j = 0; j < nv; ++j) {
      const double d = q_star_[static_cast<std::size_t>(j)] - seed_[j];
      dq2 += d * d;
    }
    score += w.w_q * dq2 + w.penalty * static_cast<double>(std::popcount(mask));

    c.reject = JudgeReject::kNone;
    c.passed = true;
    c.score = score;
    ++stats.n_pass;
    if (best < 0 || score < best_score) {
      best = static_cast<int>(&c - cands_.data());
      best_score = score;
      best_gamma = gamma_f;
      best_tw = t_w;
      best_window_only = window_only;
      best_gmin = r.gamma_usable ? r.window.g_min : 0.0;
      best_mask = mask;
      for (int j = 0; j < nv; ++j) {
        best_q[static_cast<std::size_t>(j)] = q_star_[static_cast<std::size_t>(j)];
      }
      best_w5 = ik.w5;
      best_w6 = ik.w6;
    }
  }

  // Histogram of judgement rejects (candidates the IK never reached stay
  // kNotEvaluated — they were not judged, and saying so is the point).
  JudgeReject most = JudgeReject::kNone;
  std::uint16_t most_n = 0;
  for (int i = 0; i < m; ++i) {
    const auto r = cands_[static_cast<std::size_t>(i)].reject;
    const auto idx = static_cast<std::size_t>(r);
    ++stats.judge_rejects[idx];
    // "Not evaluated" is a bottleneck only when the budget cut the search;
    // cut by max_ik alone, it is not why nothing passed.
    const bool counts =
        r != JudgeReject::kNone && (r != JudgeReject::kNotEvaluated || stats.budget_hit);
    if (counts && stats.judge_rejects[idx] > most_n) {
      most_n = stats.judge_rejects[idx];
      most = r;
    }
  }

  // ── Switching (§4.7) and freeze (decision G) ──────────────────────────────
  const bool following = current_.valid;
  if (following) {
    stats.publish = false;
    const double to_tc = static_cast<double>(current_.t_c_ns - now.ns) * kNsToS;
    if (!(to_tc > params_.t_freeze)) {
      stats.decision = SwitchDecision::kHeldFreeze;
    } else if (best < 0) {
      stats.decision = SwitchDecision::kHeldNoCandidate;
    } else {
      // The current plan's score this cycle: the candidate at its t_c, if one
      // passed (the grid moves with every snapshot, so "at" means within half
      // a slice).
      const double half = half_slice_ns;
      bool cur_feasible = false;
      double cur_score = 0.0;
      for (int i = 0; i < m; ++i) {
        const Candidate& c = cands_[static_cast<std::size_t>(i)];
        const auto t_k = traj.s[static_cast<std::size_t>(c.k)].t_ns;
        if (c.passed && std::fabs(static_cast<double>(t_k - current_.t_c_ns)) <= half) {
          cur_feasible = true;
          cur_score = c.score;
          break;
        }
      }
      const bool better = !cur_feasible || (cur_score - best_score > params_.switch_delta_j);
      const TrajSample& bs =
          traj.s[static_cast<std::size_t>(cands_[static_cast<std::size_t>(best)].k)];
      const double dp = std::sqrt((bs.p[0] - current_.p_c[0]) * (bs.p[0] - current_.p_c[0]) +
                                  (bs.p[1] - current_.p_c[1]) * (bs.p[1] - current_.p_c[1]) +
                                  (bs.p[2] - current_.p_c[2]) * (bs.p[2] - current_.p_c[2]));
      // The step the switch puts into u_des, held to η_jump·a_max (§4.7,
      // decision ⑥), at its worst over the instants the RT may adopt it.
      const double step = SwitchStep(traj, rt, now_lead, dp);
      const bool jump_ok = step <= params_.switch_eta_jump * constants_.ref_a_max;
      const bool best_is_current =
          std::fabs(static_cast<double>(bs.t_ns - current_.t_c_ns)) <= half;
      if (!better) {
        // Same candidate, moved prediction: refresh it so the RT does not
        // keep aiming at where the ball was predicted to be.
        if (best_is_current && dp > params_.eps_term) {
          stats.decision = jump_ok ? SwitchDecision::kRefreshed : SwitchDecision::kHeldJump;
          stats.publish = jump_ok;
        } else {
          stats.decision = SwitchDecision::kHeldHysteresis;
        }
      } else if (!jump_ok) {
        stats.decision = SwitchDecision::kHeldJump;
      } else {
        stats.decision = SwitchDecision::kReplaced;
        stats.publish = true;
      }
    }
  }

  if (best < 0) {
    plan.reason = m > 0 && stats.n_ik == 0 && stats.budget_hit ? PlanReason::kBudgetExceeded
                                                               : ReasonFor(most);
    stats.search_ns = clock_() - t_start;
    return plan;
  }

  // ── The plan ───────────────────────────────────────────────────────────────
  const Candidate& bc = cands_[static_cast<std::size_t>(best)];
  const TrajSample& bs = traj.s[static_cast<std::size_t>(bc.k)];
  const Eigen::Vector3d v(bs.v[0], bs.v[1], bs.v[2]);
  const double speed = v.norm();
  plan.t_c_ns = bs.t_ns;
  plan.t_cmd_ns = std::isfinite(constants_.t_close_e2e)
                      ? bs.t_ns - SecondsToNs(constants_.t_close_e2e)
                      : bs.t_ns;
  plan.p_c = bs.p;
  plan.a_d = {-v.x() / speed, -v.y() / speed, -v.z() / speed};
  plan.v_c = bs.v;
  // γ continues from where the reference is when replacing (no step in γ)
  // and ramps over the window the rollout chose, [t_c − T_w, t_c] clipped to
  // now_lead — the exact profile the rollout judged. Evaluated against
  // now_lead (plan §3).
  plan.gamma_g0 = g0;
  plan.gamma_gf = best_gamma;
  plan.gamma_t0_ns = std::max(now_lead.ns, bs.t_ns - SecondsToNs(best_tw));
  plan.gamma_t1_ns = bs.t_ns;
  plan.gamma_min = best_gmin;
  plan.nv = nv;
  for (int j = 0; j < nv; ++j) {
    plan.q_star[static_cast<std::size_t>(model_.device_of_model[static_cast<std::size_t>(j)])] =
        best_q[static_cast<std::size_t>(j)];
  }
  plan.w5 = best_w5;
  plan.w6 = best_w6;
  plan.score = best_score;
  plan.sigma_c = bc.sigma_known ? bc.sigma : 0.0;
  plan.sigma_l = plan.sigma_c;
  plan.dp_impact = constants_.ball_mass * (1.0 - best_gamma) * speed;
  plan.valid = true;
  plan.reason = PlanReason::kNone;

  stats.chosen_rank_mask = best_mask;
  stats.chosen_score = best_score;
  stats.chosen_lead_s = bc.lead_s;
  stats.chosen_gamma_f = best_gamma;
  stats.chosen_t_w = best_tw;
  stats.chosen_rollout_window_only = best_window_only;
  stats.search_ns = clock_() - t_start;
  return plan;
}

}  // namespace rtc::catching
