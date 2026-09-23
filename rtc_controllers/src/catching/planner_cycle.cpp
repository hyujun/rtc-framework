// One planner wake (S6). See planner_cycle.hpp.
#include "rtc_controllers/catching/planner_cycle.hpp"

#include "rtc_base/types/types.hpp"  // rtc::SteadyNowNs

namespace rtc::catching {

PlannerCycle::PlannerCycle() noexcept : clock_(&rtc::SteadyNowNs) {}

bool PlannerCycle::Bind(const PlannerCycleIo& io) noexcept {
  bound_ = io.traj != nullptr && io.cov != nullptr && io.rt != nullptr && io.plan != nullptr;
  io_ = bound_ ? io : PlannerCycleIo{};
  return bound_;
}

PlanSnapshot PlannerCycle::PlanOnce(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                    bool cov_matched, const PlannerRtState& rt, NowReal now,
                                    SearchStats& stats) noexcept {
  if (search_.Configured()) {
    return search_.Plan(traj, cov, cov_matched, rt, now, stats);
  }
  stats = SearchStats{};
  static_cast<void>(cov);
  static_cast<void>(cov_matched);
  static_cast<void>(now);
  // S6-A stub: provenance filled, no candidate. `reason` stays kNone — "no
  // plan, and no gate is being named", because no gate ran. A stub that
  // reported a real gate's code would send the operator looking for a
  // bottleneck that does not exist.
  PlanSnapshot plan{};
  plan.token = traj.token;
  // The plan belongs to the activation the RT is in, which is the one the
  // caller has already checked the trajectory against.
  plan.token.activation_generation = rt.activation_generation;
  plan.rt_iteration = rt.rt_iteration;
  plan.rt_state_ns = rt.rt_state_ns;
  plan.valid = false;
  plan.reason = PlanReason::kNone;
  return plan;
}

PlannerCycleRecord PlannerCycle::Run(NowReal wake) noexcept {
  PlannerCycleRecord rec{};
  rec.wake_ns = wake.ns;
  if (!bound_) {
    return rec;
  }

  // ── 1. The RT state (L3 §5.3 step 2, read first: it says whether to plan) ─
  const PlannerRtState rt = io_.rt->Load();
  if (!rt.valid) {
    return rec;
  }
  rec.mode = rt.mode;
  // A trial reset since the last wake. The planner has no per-trial state at
  // S6-A, so noticing it is all there is to do; S6-B drops its per-trial
  // search state here. Recorded before the activity gate so a reset is seen
  // even when the new mode has nothing to plan for. (Signals raised for the
  // ended trial need no draining here: the wait already consumed them, and a
  // plan from that trial is refused by the RT's reset floor, JudgePlan (f).)
  if (rt.reset_epoch != seen_reset_epoch_) {
    seen_reset_epoch_ = rt.reset_epoch;
    rec.reset_seen = true;
    search_.ResetTrial();
  }
  const PlannerActivity activity = ActivityFor(static_cast<Mode>(rt.mode));
  if (activity == PlannerActivity::kMonitor) {
    // monitorOnly (§4.6): σ_ℓ at the committed t_c from the newest covariance.
    // Recorded, never published: the RT does not take a new plan once
    // committed, and σ_ℓ's consumer (L7 abort) is S7.2.
    io_.traj->LoadInto(traj_);
    io_.cov->LoadInto(cov_);
    search_.Monitor(traj_, cov_, cov_.valid && SameSnapshot(cov_.token, traj_.token), rt,
                    rec.search);
    return rec;
  }
  if (activity != PlannerActivity::kSearch) {
    return rec;
  }

  // ── 2. The trajectory, then its covariance (L3 §5.2 read order) ──────────
  io_.traj->LoadInto(traj_);
  if (!traj_.valid || traj_.token.activation_generation != rt.activation_generation) {
    rec.outcome = CycleOutcome::kNoInput;
    return rec;
  }
  rec.snapshot_sequence = traj_.token.snapshot_sequence;
  rec.track_generation = traj_.token.generation;
  rec.traj_recv_ns = traj_.token.traj_recv_ns;

  // The ingress stores covariance FIRST and trajectory second, so the window
  // in which the two disagree is one where the covariance is the newer one.
  // One re-read closes the common case (the writer landed between the two
  // loads); a second mismatch means the pair is not a pair, and it is not
  // used (L3 §5.2: "그 조합은 쓰지 않는다").
  io_.cov->LoadInto(cov_);
  rec.cov_matched = cov_.valid && SameSnapshot(cov_.token, traj_.token);
  if (!rec.cov_matched) {
    io_.cov->LoadInto(cov_);
    rec.cov_matched = cov_.valid && SameSnapshot(cov_.token, traj_.token);
  }

  // ── 3. Search (A-4 single entry) ─────────────────────────────────────────
  PlanSnapshot plan = PlanOnce(traj_, cov_, rec.cov_matched, rt, wake, rec.search);
  if (post_search_hook_ != nullptr) {
    post_search_hook_(post_search_context_);
  }

  // ── 4. Provenance re-check before publishing (L3 §5.2, planner side) ─────
  // A newer trajectory means the eventfd has already been signalled for it,
  // so the next wake plans against it; publishing this one would hand the RT
  // a plan for a prediction that is no longer the latest. A moved reset epoch
  // means the trial this was computed for is over.
  io_.traj->LoadInto(traj_recheck_);
  const PlannerRtState rt_now = io_.rt->Load();
  if (!SameSnapshot(traj_recheck_.token, traj_.token) || rt_now.reset_epoch != rt.reset_epoch ||
      rt_now.activation_generation != rt.activation_generation) {
    rec.outcome = CycleOutcome::kSuperseded;
    return rec;
  }

  // The switching rule decided to keep the plan the RT is following (§4.7,
  // freeze G): publishing nothing IS the decision.
  if (!rec.search.publish) {
    rec.outcome = CycleOutcome::kHeld;
    return rec;
  }

  // ── 5. Publish ───────────────────────────────────────────────────────────
  plan.plan_id = ++last_plan_id_;
  plan.publish_ns = clock_();
  io_.plan->Store(plan);
  search_.NotePublished(plan);
  rec.outcome = CycleOutcome::kPublished;
  rec.plan_id = plan.plan_id;
  rec.plan_valid = plan.valid;
  rec.reason = plan.reason;
  rec.publish_ns = plan.publish_ns;
  return rec;
}

}  // namespace rtc::catching
