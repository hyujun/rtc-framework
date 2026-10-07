// The docking segment planner (E1-F16 #742). See mpc_docking_segment_planner.hpp.
#include "rtc_controllers/catching/mpc_docking_segment_planner.hpp"

#include "rtc_controllers/catching/ball_node_samples.hpp"
#include "rtc_controllers/catching/catch_search.hpp"  // CatchSolution
#include "rtc_controllers/catching/mpc_docking_relative_state.hpp"
#include "rtc_controllers/catching/mpc_segment_planner.hpp"  // MpcSegmentBetweenNodeSpeedOk
#include "rtc_controllers/catching/nlp_catch_screening.hpp"  // ProjectStartIntoBox
#include "rtc_controllers/catching/node_follower.hpp"
#include "rtc_controllers/catching/time_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <utility>

namespace rtc::catching {

namespace {

// ns → s by DIVISION: the quotient is correctly rounded, so 50'000'000 ns is
// exactly the literal 0.05 a profile wrote — and the same number the search's
// cores were given for the same key, which is what lets one planner evaluate
// the other's solution on an identical grid.
constexpr double kNsPerSec = 1e9;
// A warm-up solve may take this many first-solve budgets: enough for its first
// iterations, bounded whatever the problem turns out to be.
constexpr std::int64_t kWarmUpBudgets = 4;

[[nodiscard]] constexpr std::size_t U(int i) noexcept {
  return static_cast<std::size_t>(i);
}

void SetCoreReason(SegmentRecord& rec, MpcDockingReason reason) noexcept {
  rec.core_reason = static_cast<std::uint8_t>(reason);
  rec.core_reason_name = MpcDockingReasonName(reason);
}

// The record's group arrays are sized apart from the core (segment_planner.hpp
// does not include it): the two have to agree.
static_assert(kSegmentDockingRowGroups == static_cast<std::size_t>(kNumDockingRowGroups));
static_assert(kSegmentDockingElasticGroups == static_cast<std::size_t>(kNumDockingElasticGroups));

// The core's account of a solve that reached an iterate, as the record carries it.
void RecordSolve(const MpcDockingSegmentCoreResult& r, DockingSolveStats& s) noexcept {
  s.ran = true;
  s.qp_solves = r.qp_solves;
  s.qp_iterations = r.qp_iterations;
  s.backtracks = r.backtracks;
  s.mu_updates = r.mu_updates;
  s.start_us = r.start_us;
  s.linearize_us = r.linearize_us;
  s.assemble_us = r.assemble_us;
  s.qp_us = r.qp_us;
  s.merit_us = r.merit_us;
  s.kkt_residual = r.kkt_residual;
  s.grad_norm = r.grad_norm;
  s.complementarity = r.complementarity;
  s.infeasible_group_name =
      r.reason == MpcDockingReason::kInfeasible ? DockingRowGroupName(r.infeasible_group) : "none";
  s.violation = r.violation;
  s.elastic = r.elastic;
  s.c_catch = r.c_catch;
  s.c_guarded = r.c_guarded;
  s.sigma_s = r.sigma_s;
  s.sigma_t = r.sigma_t;
  s.lateral_margin = r.lateral_margin;
  s.timing_margin = r.timing_margin;
  s.cost_reference = r.cost.reference;
  s.cost_stop = r.cost.stop;
}

// max |v_i|; NaN when a component is not finite (std::max would drop it).
[[nodiscard]] double MaxAbsOrNan(const Eigen::Ref<const Eigen::VectorXd>& v) noexcept {
  double worst = 0.0;
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    if (!std::isfinite(v[i])) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    worst = std::max(worst, std::fabs(v[i]));
  }
  return worst;
}

// Whether a warm-up solve got as far as its main QP.
struct WarmUpProbe {
  bool reached_qp{false};
};

void WarmUpStageHook(MpcDockingStage stage, bool begin, void* user) noexcept {
  if (begin && stage == MpcDockingStage::kAssemble) {
    static_cast<WarmUpProbe*>(user)->reached_qp = true;
  }
}

}  // namespace

bool MpcDockingSegmentPlanner::Configure(const MpcDockingSegmentPlannerModel& model,
                                         const MpcDockingSegmentPlannerConstants& consts,
                                         const MpcDockingSegmentPlannerParams& params,
                                         ClockFn clock, std::string* error) {
  configured_ = false;
  cores_.clear();
  inputs_.clear();
  results_.clear();
  ring_.Clear();
  const auto fail = [error](std::string why) {
    if (error != nullptr) {
      *error = std::move(why);
    }
    return false;
  };
  if (clock == nullptr) {
    return fail("mpc_docking planner: no clock");
  }
  if (!model.arm || model.nv < 1 || model.nv > kMaxSegmentNv || model.arm->nv != model.nv ||
      model.arm->nq != model.nv) {
    return fail(
        "mpc_docking planner: the arm model is absent, or its joint count is not within "
        "the segment payload's capacity");
  }
  for (int m = 0; m < model.nv; ++m) {
    const int d = model.device_of_model[U(m)];
    if (d < 0 || d >= model.nv) {
      return fail("mpc_docking planner: device_of_model is not a permutation of the arm's joints");
    }
    const double rating = model.qd_rating[U(m)];
    if (!std::isfinite(rating) || !(rating > 0.0)) {
      return fail("mpc_docking planner: a joint has no positive velocity rating");
    }
  }
  const MpcDockingSegmentPlannerParams& p = params;
  if (p.n_pre_max < 1 || p.n_stop < 3 || p.n_pre_max + p.n_stop > kMaxSegmentNodes ||
      p.n_stop_blocks < 3 || p.n_stop_blocks > p.n_stop ||
      p.n_pre_max + p.n_stop_blocks > kMaxSegmentNodes) {
    return fail(
        "mpc_docking planner: approach.n_pre_max, stop.n_nodes and stop.blocks do not "
        "fit the segment payload (n_pre_max >= 1, at least three stop blocks, "
        "n_pre_max + n_nodes within the payload's node capacity)");
  }
  const std::int64_t dt_pre_ns = p.DtPreNs();
  const std::int64_t dt_stop_ns = p.DtStopNs();
  if (dt_pre_ns <= 0 || dt_pre_ns > kMaxSegmentDtPreNs || dt_stop_ns <= 0) {
    return fail("mpc_docking planner: approach.dt_pre_s or stop.dt_s is not a usable spacing");
  }
  const std::int64_t first_ns = SecondsToNs(p.budget_first_s);
  const std::int64_t replan_ns = SecondsToNs(p.budget_replan_s);
  const std::int64_t t_arm_ns = SecondsToNs(consts.t_arm_s);
  const std::int64_t h_ns = SecondsToNs(consts.control_dt);
  if (first_ns <= 0 || replan_ns <= 0 || t_arm_ns < 0 || h_ns <= 0) {
    return fail("mpc_docking planner: a budget, T_arm or the control period is not usable");
  }
  if (!std::isfinite(p.rest_tol) || !(p.rest_tol > 0.0) || !std::isfinite(p.slack_c_max) ||
      p.slack_c_max < 0.0 || !std::isfinite(p.slack_v_max) || p.slack_v_max < 0.0) {
    return fail("mpc_docking planner: approach.rest_tol or a publish threshold is out of range");
  }

  params_ = p;
  clock_ = clock;
  nv_ = model.nv;
  device_of_model_ = model.device_of_model;
  qd_rating_ = model.qd_rating;
  dt_pre_ns_ = dt_pre_ns;
  dt_stop_ns_ = dt_stop_ns;
  first_ns_ = first_ns;
  replan_ns_ = replan_ns;
  t_arm_ns_ = t_arm_ns;
  h_ns_ = h_ns;

  cores_.reserve(U(p.n_pre_max));
  inputs_.resize(U(p.n_pre_max));
  results_.resize(U(p.n_pre_max));
  for (int n_pre = 1; n_pre <= p.n_pre_max; ++n_pre) {
    MpcDockingSegmentCoreParams cp = p.core;
    cp.n_pre = n_pre;
    cp.dt_pre = static_cast<double>(dt_pre_ns_) / kNsPerSec;
    cp.n_stop = p.n_stop;
    cp.dt_stop = static_cast<double>(dt_stop_ns_) / kNsPerSec;
    cp.n_blocks = n_pre + p.n_stop_blocks;
    cp.block_sizes.fill(1);  // one block per pre-catch interval
    for (int b = 0; b < p.n_stop_blocks; ++b) {
      cp.block_sizes[U(n_pre + b)] = p.stop_block_sizes[U(b)];
    }
    // The catch instant is the plan's: this planner does not move it.
    cp.catch_time_variable = false;
    auto core = std::make_unique<MpcDockingSegmentCore>();
    const MpcDockingReason why = core->Init(*model.arm, model.catch_frame, cp, model.limits, clock);
    if (why != MpcDockingReason::kNone) {
      cores_.clear();
      return fail(std::string("mpc_docking planner: the core for n_pre = ") +
                  std::to_string(n_pre) +
                  " refused its grid or parameters: " + MpcDockingReasonName(why));
    }
    core->ResizeInput(inputs_[U(n_pre - 1)]);
    core->ResizeResult(results_[U(n_pre - 1)]);
    cores_.push_back(std::move(core));
  }
  std::string why;
  if (!WarmUp(model, why)) {
    cores_.clear();
    return fail(std::move(why));
  }
  configured_ = true;
  return true;
}

bool MpcDockingSegmentPlanner::WarmUp(const MpcDockingSegmentPlannerModel& model,
                                      std::string& why) {
  // One solve per core on a ball that comes down the hand's own approach axis
  // at the reference closing speed and crosses the entrance plane at the catch
  // node: every stage runs once, on a problem that has a solution.
  const MpcDockingSegmentCore& first = *cores_.front();
  Eigen::VectorXd q(nv_);
  for (int m = 0; m < nv_; ++m) {
    q[m] = std::clamp(model.warm_pose[U(device_of_model_[U(m)])], first.PositionLower()[m],
                      first.PositionUpper()[m]);
  }
  pinocchio::Data data(*model.arm);
  DockingFrameKinematics kin;
  kin.Resize(nv_);
  const Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(nv_);
  if (!q.allFinite() ||
      !ComputeDockingFrameKinematics(*model.arm, data, model.catch_frame, q, v_zero, kin)) {
    why =
        "mpc_docking planner: the catch frame's kinematics cannot be evaluated at the warm-up "
        "pose";
    return false;
  }
  const Eigen::Vector3d e3 = kin.R.col(2);
  const double speed = std::fabs(params_.core.nu_ref.z());
  const Eigen::Vector3d v_b = -(speed > 0.0 ? speed : 1.0) * e3;
  const Eigen::Vector3d p_b = kin.p + params_.core.s_ent * e3;
  BallCovariance cov = BallCovariance::Zero();
  cov.diagonal() << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
  warmup_max_ns_ = 0;
  warmup_total_ns_ = 0;
  for (std::size_t slot = 0; slot < cores_.size(); ++slot) {
    MpcDockingSegmentCore& core = *cores_[slot];
    MpcDockingSegmentCoreInput& in = inputs_[slot];
    const int kc = core.CatchNode();
    const double t_c = core.NodeTime(kc);
    in.q0 = q;
    in.qd0.setZero();
    in.qdd0.setZero();
    for (int k = 0; k <= kc; ++k) {
      BallNodeSample& b = in.ball[U(k)];
      b = BallNodeSample{};
      b.p = p_b + v_b * (core.NodeTime(k) - t_c);
      b.v = v_b;
      b.valid = true;
    }
    in.ball[U(kc)].cov = cov;
    in.ball[U(kc)].cov_valid = true;
    in.initial_valid = false;
    in.catch_target_valid = true;
    in.q_catch_target = q;
    in.p_line = p_b;
    in.d_line = -e3;
    const std::int64_t start = clock_();
    in.deadline_ns = start + kWarmUpBudgets * first_ns_;
    WarmUpProbe probe;
    core.SetStageHook(&WarmUpStageHook, &probe);
    static_cast<void>(core.Solve(in, results_[slot]));
    core.SetStageHook(nullptr, nullptr);
    const std::int64_t took = clock_() - start;
    warmup_max_ns_ = std::max(warmup_max_ns_, took);
    warmup_total_ns_ += took;
    in.deadline_ns = 0;
    if (!probe.reached_qp) {
      why = std::string("mpc_docking planner: the warm-up solve of the core for n_pre = ") +
            std::to_string(kc) +
            " ended before its QP: " + MpcDockingReasonName(results_[slot].reason);
      return false;
    }
  }
  return true;
}

void MpcDockingSegmentPlanner::SetClock(ClockFn clock) noexcept {
  if (clock == nullptr) {
    return;
  }
  clock_ = clock;
  for (const auto& core : cores_) {
    core->SetClock(clock);
  }
}

void MpcDockingSegmentPlanner::SetCoreStageHookForTesting(MpcDockingSegmentCore::StageHook hook,
                                                          void* user) noexcept {
  for (const auto& core : cores_) {
    core->SetStageHook(hook, user);
  }
}

const MpcDockingSegmentCore* MpcDockingSegmentPlanner::Core(int n_pre) const noexcept {
  return n_pre >= 1 && n_pre <= static_cast<int>(cores_.size()) ? cores_[U(n_pre - 1)].get()
                                                                : nullptr;
}

const MpcDockingSegmentCoreResult* MpcDockingSegmentPlanner::LastResult(int n_pre) const noexcept {
  return n_pre >= 1 && n_pre <= static_cast<int>(results_.size()) ? &results_[U(n_pre - 1)]
                                                                  : nullptr;
}

const MpcDockingSegmentCoreInput* MpcDockingSegmentPlanner::LastInputForTesting(
    int n_pre) const noexcept {
  return n_pre >= 1 && n_pre <= static_cast<int>(inputs_.size()) ? &inputs_[U(n_pre - 1)] : nullptr;
}

void MpcDockingSegmentPlanner::ResetTrial() noexcept {
  ring_.Clear();
}

void MpcDockingSegmentPlanner::NotePublished(const SegmentSnapshot& p) noexcept {
  ring_.Push(p, NoSegmentPayload{});
}

bool MpcDockingSegmentPlanner::FollowedTrack(const PlannerRtState& rt,
                                             std::uint64_t& generation) const noexcept {
  return ring_.FollowedTrack(rt, generation);
}

void MpcDockingSegmentPlanner::Reported(const PlannerRtState& rt,
                                        ReportedSegments& out) const noexcept {
  ring_.Reported(rt, out);
}

std::uint32_t MpcDockingSegmentPlanner::SourceSeq(const PlannerRtState& rt,
                                                  std::int64_t t_eff_ns) const noexcept {
  return ring_.SourceSeq(rt, t_eff_ns);
}

bool MpcDockingSegmentPlanner::CheckState(const PlannerRtState& rt, std::int64_t start,
                                          bool need_command, SegmentRecord& rec) const noexcept {
  if (!rt.valid || (need_command && !rt.cmd_seeded) || rt.nv != nv_) {
    rec.outcome = SegmentOutcome::kNoState;
    return false;
  }
  const std::int64_t age = start - rt.rt_state_ns;
  if (age < 0 || age > kMpcDockingMaxRtStateAgeNs) {
    rec.outcome = SegmentOutcome::kStaleState;
    return false;
  }
  return true;
}

std::int64_t MpcDockingSegmentPlanner::NodeNs(std::int64_t t0_ns, int n_pre, int k) const noexcept {
  return k <= n_pre ? t0_ns + static_cast<std::int64_t>(k) * dt_pre_ns_
                    : t0_ns + static_cast<std::int64_t>(n_pre) * dt_pre_ns_ +
                          static_cast<std::int64_t>(k - n_pre) * dt_stop_ns_;
}

void MpcDockingSegmentPlanner::SetBall(const BallPrediction& ball, std::int64_t t0_ns, int n_pre,
                                       MpcDockingSegmentCoreInput& in) const noexcept {
  // The one rule every planner reads the prediction by (ball_node_samples.hpp).
  // Only the catch node's covariance is read; a node past the prediction is
  // the core's to refuse (kBallInvalid).
  int hint = 0;
  for (int k = 0; k <= n_pre; ++k) {
    const BallTime t{NodeNs(t0_ns, n_pre, k)};
    in.ball[U(k)] =
        SampleBallNode(*ball.traj, k == n_pre ? ball.cov : nullptr, ball.cov_matched, t, hint);
  }
  // The stop line: through the catch point, along the ball's travel.
  const BallNodeSample& at_catch = in.ball[U(n_pre)];
  in.p_line = at_catch.p;
  in.d_line = at_catch.v.normalized();
}

bool MpcDockingSegmentPlanner::OnGrid(const SegmentSnapshot& seg) const noexcept {
  return seg.valid && seg.nv == nv_ && seg.n_pre >= 1 &&
         seg.n_pre <= static_cast<int>(cores_.size()) && seg.k0 == 0 &&
         seg.n_nodes == seg.n_pre + params_.n_stop && seg.dt_ns == dt_stop_ns_ &&
         seg.dt_pre_ns == dt_pre_ns_ &&
         // A catch interval of its own length is another grid: a core whose
         // catch instant is fixed cannot evaluate it.
         seg.dt_catch_ns == 0 &&
         seg.t0_ns == seg.t_c_ns - static_cast<std::int64_t>(seg.n_pre) * dt_pre_ns_;
}

bool MpcDockingSegmentPlanner::RunCore(bool evaluate, MpcDockingSegmentCore& core,
                                       const MpcDockingSegmentCoreInput& in,
                                       MpcDockingSegmentCoreResult& res) noexcept {
  if (solver_hook_ != nullptr) {
    solver_hook_(true, solver_user_);
  }
  const bool ok = evaluate ? core.Evaluate(in, res) : core.Solve(in, res);
  if (solver_hook_ != nullptr) {
    solver_hook_(false, solver_user_);
  }
  return ok;
}

SegmentOutcome MpcDockingSegmentPlanner::Judge(const MpcDockingSegmentCoreResult& r, bool ok,
                                               bool converged, int n_pre, std::int64_t start,
                                               std::int64_t end, std::int64_t budget_ns,
                                               std::int64_t t_eff,
                                               SegmentRecord& rec) const noexcept {
  rec.solve_ns = end - start;
  SetCoreReason(rec, r.reason);
  if (!ok) {
    // Refused before any iterate: the result's nodes are another solve's.
    return end - start > budget_ns ? SegmentOutcome::kBudget : SegmentOutcome::kSolveFailed;
  }
  rec.iterations = r.iterations;
  rec.qp_status = r.qp_status;
  rec.tau_ratio_max = r.tau_ratio_max;
  // An iterate exists from here on, whatever the solve ended as: its account
  // and its slacks are recorded for every ending, so that a solve that failed
  // or was cut can be read for WHY. The corridor and the speed envelope are
  // soft in the problem: a solution every hard row accepts may still lean on
  // their slacks.
  RecordSolve(r, rec.docking);
  double slack_c = 0.0;
  double slack_v = 0.0;
  for (int i = 0; i < r.approach_nodes && i < kMaxMpcNodes; ++i) {
    slack_c = std::max(slack_c, r.slack_c[U(i)]);
    slack_v = std::max(slack_v, r.slack_v[U(i)]);
  }
  rec.slack_max = slack_c;
  rec.slack_v = slack_v;
  if (r.reason == MpcDockingReason::kDeadline || end - start > budget_ns) {
    return SegmentOutcome::kBudget;
  }
  if (!StartsInTime(end, t_eff)) {
    return SegmentOutcome::kLate;
  }
  if (!r.feasible || !converged) {
    return SegmentOutcome::kSolveFailed;
  }
  const double tol = params_.core.tol_violation;
  if (!std::isfinite(slack_c) || !std::isfinite(slack_v) || slack_c > params_.slack_c_max + tol ||
      slack_v > params_.slack_v_max + tol) {
    return SegmentOutcome::kSlack;
  }
  double ratio = std::numeric_limits<double>::infinity();
  const bool speed_ok =
      MpcSegmentBetweenNodeSpeedOk(r.qd, r.qdd, n_pre, static_cast<double>(dt_pre_ns_) / kNsPerSec,
                                   static_cast<double>(dt_stop_ns_) / kNsPerSec,
                                   std::span<const double>(qd_rating_.data(), U(nv_)), ratio);
  rec.speed_ratio_max = ratio;
  return speed_ok ? SegmentOutcome::kReady : SegmentOutcome::kSpeed;
}

void MpcDockingSegmentPlanner::Pack(const PlannerRtState& rt, std::uint64_t track_generation,
                                    std::uint32_t plan_id, std::int64_t t_c, std::int64_t t_eff,
                                    int n_pre, bool x0_clamped,
                                    const MpcDockingSegmentCoreResult& r,
                                    SegmentSnapshot& out) const noexcept {
  const int n_total = n_pre + params_.n_stop;
  // Whole, not field by field: the node arrays' unused entries would
  // otherwise keep an earlier, longer segment's.
  out = SegmentSnapshot{};
  out.token.activation_generation = rt.activation_generation;
  out.token.generation = track_generation;
  out.rt_iteration = rt.rt_iteration;
  out.rt_state_ns = rt.rt_state_ns;
  out.plan_id = plan_id;
  out.t_c_ns = t_c;
  out.t0_ns = t_eff;
  out.dt_ns = dt_stop_ns_;
  out.dt_pre_ns = dt_pre_ns_;
  out.n_nodes = n_total;
  out.nv = nv_;
  out.n_pre = n_pre;
  for (int k = 0; k <= n_total; ++k) {
    for (int m = 0; m < nv_; ++m) {
      const std::size_t e = U(k * kMaxSegmentNv + device_of_model_[U(m)]);
      out.q[e] = r.q(m, k);
      out.qd[e] = r.qd(m, k);
      out.qdd[e] = r.qdd(m, k);
    }
  }
  out.tau_ratio_max = r.tau_ratio_max;
  out.x0_clamped = x0_clamped;
  out.valid = true;
}

bool MpcDockingSegmentPlanner::PlanFirst(const PlannerRtState& rt, const PlanSnapshot& plan,
                                         const BallPrediction& ball, const CatchSolution* solution,
                                         SegmentSnapshot& out, SegmentRecord& rec) noexcept {
  rec = SegmentRecord{};
  if (!configured_) {
    return false;
  }
  rec.kind = SegmentKind::kFirst;
  const std::int64_t start = clock_();
  // The RT follows a plan: this is the first segment of its REPLACEMENT, and
  // it starts on the segment the arm will be on, not at rest.
  const bool following = rt.plan_active;
  // Before a plan the RT has no command yet (cmd_seeded false): it reports
  // the MEASURED pose with zero velocity, which is where it seeds the command
  // when it takes the plan — the same x₀ either way. Requiring a seeded
  // command there would withhold every first pair. While it follows a plan
  // the command is what the segments were started on: it has to be seeded.
  if (!CheckState(rt, start, /*need_command=*/following, rec)) {
    return false;
  }
  if (!plan.valid || plan.t_c_ns <= 0 || plan.nv != nv_) {
    rec.outcome = SegmentOutcome::kNoState;
    return false;
  }
  if (ball.Empty() || !ball.traj->valid) {
    rec.outcome = SegmentOutcome::kNoBall;
    return false;
  }
  if (!following) {
    // The arm rests on its command: a first solve starts from (q_cmd, 0, 0).
    // A NaN is not "at rest": std::max would drop it.
    double speed = 0.0;
    bool speed_finite = true;
    for (int d = 0; d < nv_; ++d) {
      const double v = rt.qd_cmd[U(d)];
      speed_finite = speed_finite && std::isfinite(v);
      speed = std::max(speed, std::fabs(v));
    }
    rec.x0_speed = speed_finite ? speed : std::numeric_limits<double>::quiet_NaN();
    if (!(speed_finite && speed <= params_.rest_tol)) {
      rec.outcome = SegmentOutcome::kNotAtRest;
      return false;
    }
  }
  const std::int64_t t_c = plan.t_c_ns;

  // ── The search's own solution, when it is this plan's ─────────────────────
  // ... and started where this planner would start it: on the segment the RT
  // reports for the solution's node 0 (none, for an arm that follows no
  // plan). A solution that started anywhere else is not published.
  if (solution != nullptr && solution->seg.t_c_ns == t_c &&
      solution->seg.rt_iteration == rt.rt_iteration && OnGrid(solution->seg) &&
      StartsOnTheReport(rt, *solution)) {
    const SegmentSnapshot& seg = solution->seg;
    const int n_pre = seg.n_pre;
    const int n_total = seg.n_nodes;
    const std::int64_t t_eff = seg.t0_ns;
    rec.k = -n_pre;
    rec.n_nodes = n_total;
    rec.from_search = true;
    rec.source_seq = solution->source_seq;
    MpcDockingSegmentCore& core = *cores_[U(n_pre - 1)];
    MpcDockingSegmentCoreInput& in = inputs_[U(n_pre - 1)];
    MpcDockingSegmentCoreResult& res = results_[U(n_pre - 1)];
    for (int k = 0; k <= n_total; ++k) {
      for (int m = 0; m < nv_; ++m) {
        const std::size_t e = U(k * kMaxSegmentNv + device_of_model_[U(m)]);
        in.q_init(m, k) = seg.q[e];
        in.qd_init(m, k) = seg.qd[e];
        in.qdd_init(m, k) = seg.qdd[e];
      }
    }
    for (int m = 0; m < nv_; ++m) {
      in.q0[m] = in.q_init(m, 0);
      in.qd0[m] = in.qd_init(m, 0);
      in.qdd0[m] = in.qdd_init(m, 0);
    }
    if (following) {
      rec.x0_from_segment = true;
      rec.x0_speed = MaxAbsOrNan(in.qd0);
    }
    in.initial_valid = true;
    in.catch_target_valid = false;
    in.deadline_ns = 0;
    SetBall(ball, t_eff, n_pre, in);
    const bool ok = RunCore(/*evaluate=*/true, core, in, res);
    const std::int64_t end = clock_();
    // An evaluation does not converge: that is the statement of the solve
    // that produced the nodes.
    rec.outcome = Judge(res, ok, solution->converged, n_pre, start, end, first_ns_, t_eff, rec);
    rec.x0_clamped = seg.x0_clamped;
    if (rec.outcome != SegmentOutcome::kReady) {
      return false;
    }
    // The search's nodes as they are — not the evaluation's projection of them.
    out = seg;
    out.plan_id = plan.plan_id;
    out.publish_ns = 0;
    out.segment_seq = 0;
    if (!ValidateSegmentNodes(out)) {
      rec.outcome = SegmentOutcome::kSolveFailed;
      return false;
    }
    NoteFirst(rt);
    return true;
  }

  // ── A solve: from rest, or from the segment the arm will be on ────────────
  const std::int64_t earliest = start + t_arm_ns_ + first_ns_ + 2 * h_ns_;
  if (t_c - earliest < dt_pre_ns_) {
    rec.outcome = SegmentOutcome::kTooLate;
    return false;
  }
  const int n_pre =
      static_cast<int>(std::min<std::int64_t>(params_.n_pre_max, (t_c - earliest) / dt_pre_ns_));
  const int n_total = n_pre + params_.n_stop;
  const std::int64_t t_eff = t_c - static_cast<std::int64_t>(n_pre) * dt_pre_ns_;
  rec.k = -n_pre;
  rec.n_nodes = n_total;
  MpcDockingSegmentCore& core = *cores_[U(n_pre - 1)];
  MpcDockingSegmentCoreInput& in = inputs_[U(n_pre - 1)];
  MpcDockingSegmentCoreResult& res = results_[U(n_pre - 1)];
  // The start state, model order. q̇₀ and q̈₀ stay zero for an arm at rest.
  std::array<double, kMaxSegmentNv> q0{};
  std::array<double, kMaxSegmentNv> qd0{};
  std::array<double, kMaxSegmentNv> qdd0{};
  bool finite = true;
  if (following) {
    // The source is what the RT reports, never an inference (MD-58): that
    // segment, read back by the RT's own evaluator at node 0's instant.
    const SegmentSnapshot* src = ring_.Source(rt, t_eff);
    if (src == nullptr) {
      rec.outcome = SegmentOutcome::kNotFollowed;
      return false;
    }
    rec.source_seq = src->segment_seq;
    std::array<double, kMaxSegmentNv> q{};
    std::array<double, kMaxSegmentNv> qd{};
    std::array<double, kMaxSegmentNv> qdd{};
    if (!NodeTrajectoryFollower::SampleJoints(*src, t_eff, q, qd, qdd)) {
      rec.outcome = SegmentOutcome::kInputNonFinite;
      return false;
    }
    rec.x0_from_segment = true;
    double speed = 0.0;
    for (int m = 0; m < nv_; ++m) {
      const std::size_t d = U(device_of_model_[U(m)]);
      q0[U(m)] = q[d];
      qd0[U(m)] = qd[d];
      qdd0[U(m)] = qdd[d];
      finite = finite && std::isfinite(q[d]) && std::isfinite(qd[d]) && std::isfinite(qdd[d]) &&
               std::isfinite(plan.q_star[d]);
      speed = std::max(speed, std::fabs(qd[d]));
      in.q_catch_target[m] = plan.q_star[d];
    }
    rec.x0_speed = finite ? speed : std::numeric_limits<double>::quiet_NaN();
  } else {
    for (int m = 0; m < nv_; ++m) {
      const std::size_t d = U(device_of_model_[U(m)]);
      q0[U(m)] = rt.q_cmd[d];
      finite = finite && std::isfinite(rt.q_cmd[d]) && std::isfinite(plan.q_star[d]);
      in.q_catch_target[m] = plan.q_star[d];
    }
  }
  if (!finite) {
    rec.outcome = SegmentOutcome::kInputNonFinite;
    return false;
  }
  // Into the core's box: the core refuses a start outside it, and a plan is
  // published only with its segment.
  const std::size_t n = U(nv_);
  rec.x0_clamped =
      ProjectStartIntoBox(std::span<double>(q0.data(), n), std::span<double>(qd0.data(), n),
                          std::span<const double>(core.PositionLower().data(), n),
                          std::span<const double>(core.PositionUpper().data(), n),
                          std::span<const double>(core.VelocityLimit().data(), n));
  for (int m = 0; m < nv_; ++m) {
    in.q0[m] = q0[U(m)];
    in.qd0[m] = following ? qd0[U(m)] : 0.0;
    in.qdd0[m] = following ? qdd0[U(m)] : 0.0;
  }
  in.initial_valid = false;
  in.catch_target_valid = true;
  SetBall(ball, t_eff, n_pre, in);
  in.deadline_ns = start + first_ns_;
  const bool ok = RunCore(/*evaluate=*/false, core, in, res);
  const std::int64_t end = clock_();
  rec.outcome = Judge(res, ok, ok && res.converged, n_pre, start, end, first_ns_, t_eff, rec);
  if (rec.outcome != SegmentOutcome::kReady) {
    return false;
  }
  Pack(rt, plan.token.generation, plan.plan_id, t_c, t_eff, n_pre, rec.x0_clamped, res, out);
  if (!ValidateSegmentNodes(out)) {
    rec.outcome = SegmentOutcome::kSolveFailed;
    return false;
  }
  NoteFirst(rt);
  return true;
}

bool MpcDockingSegmentPlanner::StartsOnTheReport(const PlannerRtState& rt,
                                                 const CatchSolution& solution) const noexcept {
  const std::uint32_t source = ring_.SourceSeq(rt, solution.seg.t0_ns);
  // An arm that follows a plan starts on a segment: a solution that started
  // on none is not that arm's.
  return solution.source_seq == source && (!rt.plan_active || source != 0);
}

void MpcDockingSegmentPlanner::NoteFirst(const PlannerRtState& rt) noexcept {
  // A new plan: the RT reports nothing of it yet. What it does report — the
  // plan it follows and that plan's segments, which the arm stays on until the
  // replacement's node 0 — the ring keeps beside the new segment.
  if (rt.plan_active) {
    ring_.NoteReported(rt);
  } else {
    ring_.ClearReported();
  }
}

bool MpcDockingSegmentPlanner::Replan(const PlannerRtState& rt, const BallPrediction& ball,
                                      SegmentSnapshot& out, SegmentRecord& rec) noexcept {
  rec = SegmentRecord{};
  if (!configured_) {
    return false;
  }
  const std::int64_t start = clock_();
  if (!CheckState(rt, start, /*need_command=*/true, rec)) {
    return false;
  }
  if (!rt.plan_active || rt.plan_t_c_ns <= 0) {
    rec.outcome = SegmentOutcome::kNoState;
    return false;
  }
  ring_.NoteReported(rt);

  // The first grid point the replan budget reaches. A solve needs at least
  // one pre-catch interval; past that the last published segment carries the
  // arm through the catch and to rest, and nothing more is published.
  const std::int64_t t_c = rt.plan_t_c_ns;
  const std::int64_t earliest = start + t_arm_ns_ + replan_ns_ + 2 * h_ns_;
  if (t_c - earliest < dt_pre_ns_) {
    rec.k = 0;
    rec.kind = SegmentKind::kStop;
    rec.outcome = SegmentOutcome::kPastReplanWindow;
    return false;
  }
  const int n_pre =
      static_cast<int>(std::min<std::int64_t>(params_.n_pre_max, (t_c - earliest) / dt_pre_ns_));
  const int n_total = n_pre + params_.n_stop;
  const std::int64_t t_eff = t_c - static_cast<std::int64_t>(n_pre) * dt_pre_ns_;
  rec.k = -n_pre;
  rec.n_nodes = n_total;
  rec.kind = SegmentKind::kAdvance;

  // The source is what the RT reports, never an inference (MD-58).
  const SegmentSnapshot* src = ring_.Source(rt, t_eff);
  if (src == nullptr) {
    rec.outcome = SegmentOutcome::kNotFollowed;
    return false;
  }
  rec.source_seq = src->segment_seq;
  if (t_eff < src->t0_ns) {
    rec.outcome = SegmentOutcome::kUpToDate;
    return false;
  }
  if (t_eff == src->t0_ns) {
    if (!params_.replan_same_point) {
      rec.outcome = SegmentOutcome::kUpToDate;
      return false;
    }
    rec.kind = SegmentKind::kSame;
  }
  if (ball.Empty() || !ball.traj->valid) {
    rec.outcome = SegmentOutcome::kNoBall;
    return false;
  }

  MpcDockingSegmentCore& core = *cores_[U(n_pre - 1)];
  MpcDockingSegmentCoreInput& in = inputs_[U(n_pre - 1)];
  MpcDockingSegmentCoreResult& res = results_[U(n_pre - 1)];
  // x₀ and the start trajectory: the source segment, read back by the RT's own
  // evaluator at this grid's instants (both end at the same stop instant).
  std::array<double, kMaxSegmentNv> q{};
  std::array<double, kMaxSegmentNv> qd{};
  std::array<double, kMaxSegmentNv> qdd{};
  bool sampled = true;
  for (int k = 0; sampled && k <= n_total; ++k) {
    sampled = NodeTrajectoryFollower::SampleJoints(*src, NodeNs(t_eff, n_pre, k), q, qd, qdd);
    for (int m = 0; sampled && m < nv_; ++m) {
      const std::size_t d = U(device_of_model_[U(m)]);
      in.q_init(m, k) = q[d];
      in.qd_init(m, k) = qd[d];
      in.qdd_init(m, k) = qdd[d];
      sampled = std::isfinite(q[d]) && std::isfinite(qd[d]) && std::isfinite(qdd[d]);
    }
  }
  if (!sampled) {
    rec.outcome = SegmentOutcome::kInputNonFinite;
    return false;
  }
  std::array<double, kMaxSegmentNv> q0{};
  std::array<double, kMaxSegmentNv> qd0{};
  for (int m = 0; m < nv_; ++m) {
    q0[U(m)] = in.q_init(m, 0);
    qd0[U(m)] = in.qd_init(m, 0);
  }
  const std::size_t n = U(nv_);
  rec.x0_from_segment = true;
  rec.x0_clamped =
      ProjectStartIntoBox(std::span<double>(q0.data(), n), std::span<double>(qd0.data(), n),
                          std::span<const double>(core.PositionLower().data(), n),
                          std::span<const double>(core.PositionUpper().data(), n),
                          std::span<const double>(core.VelocityLimit().data(), n));
  for (int m = 0; m < nv_; ++m) {
    in.q0[m] = q0[U(m)];
    in.qd0[m] = qd0[U(m)];
    in.qdd0[m] = in.qdd_init(m, 0);
    // Node 0 of the start trajectory is the start state itself.
    in.q_init(m, 0) = q0[U(m)];
    in.qd_init(m, 0) = qd0[U(m)];
    // Should the start break a linear row, the initialisation QP aims at the
    // source's own catch pose.
    in.q_catch_target[m] = in.q_init(m, n_pre);
  }
  in.initial_valid = true;
  in.catch_target_valid = true;
  SetBall(ball, t_eff, n_pre, in);
  in.deadline_ns = start + replan_ns_;
  const bool ok = RunCore(/*evaluate=*/false, core, in, res);
  const std::int64_t end = clock_();
  rec.outcome = Judge(res, ok, ok && res.converged, n_pre, start, end, replan_ns_, t_eff, rec);
  if (rec.outcome != SegmentOutcome::kReady) {
    return false;
  }
  // Every segment of a plan carries the plan's track: the source's.
  Pack(rt, src->token.generation, rt.plan_id, t_c, t_eff, n_pre, rec.x0_clamped, res, out);
  if (!ValidateSegmentNodes(out)) {
    rec.outcome = SegmentOutcome::kSolveFailed;
    return false;
  }
  return true;
}

std::unique_ptr<MpcDockingSegmentPlanner> MakeMpcDockingSegmentPlanner(
    const MpcDockingSegmentPlannerModel& model, const MpcDockingSegmentPlannerConstants& consts,
    const MpcDockingSegmentPlannerParams& params, MpcDockingSegmentPlanner::ClockFn clock,
    std::string* error) {
  auto planner = std::make_unique<MpcDockingSegmentPlanner>();
  if (!planner->Configure(model, consts, params, clock, error)) {
    return nullptr;
  }
  return planner;
}

}  // namespace rtc::catching
