#include "rtc_mpc/thread/handler_mpc_thread.hpp"

#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_mpc/handler/mpc_factory.hpp"
#include "rtc_mpc/phase/phase_context.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>

namespace rtc::mpc {

void HandlerMPCThread::Configure(const RobotModelHandler& model_handler,
                                 std::unique_ptr<MPCHandlerBase> handler,
                                 std::unique_ptr<PhaseManagerBase> phase_manager,
                                 YAML::Node factory_cfg_light,
                                 YAML::Node factory_cfg_rich) noexcept {
  model_ = &model_handler;
  handler_ = std::move(handler);
  phase_manager_ = std::move(phase_manager);
  factory_cfg_light_ = factory_cfg_light;
  factory_cfg_rich_ = factory_cfg_rich;

  // Pre-allocate FK data; reuse every tick. `make_unique` may throw
  // std::bad_alloc — Configure is off-RT and documented as noexcept-on-
  // success, so let std::terminate handle an OOM at init time rather than
  // propagate through a noexcept boundary. This mirrors Phase 5's
  // `mpc_handler_core` pattern.
  if (model_ != nullptr) {
    pdata_ = std::make_unique<pinocchio::Data>(model_->model());
    q_scratch_.resize(model_->nq());
    v_scratch_.resize(model_->nv());
    q_scratch_.setZero();
    v_scratch_.setZero();
  }
  // Phase 6 Open Decision #3: sensor input is zero-length; Phase 7
  // `GraspPhaseManager` will introduce a `SensorSource` seam.
  sensor_scratch_.resize(0);

  have_time_anchor_ = false;
  anchor_stamp_ns_ = 0;
  last_stamp_ns_ = 0;
  anchor_state_epoch_ = 0;
  time_base_s_ = 0.0;
  last_phase_t_ = 0.0;
  last_attempted_sequence_ = 0;
  require_fresh_state_ = handler_ != nullptr && handler_->solver_config().require_fresh_state;

  record_pending_ = false;  // the records restart with the counters
  record_seq_ = 0;
  record_counts_.fill(0);
  last_err_.store(0, std::memory_order_relaxed);
  last_phase_id_.store(-1, std::memory_order_relaxed);
  total_solves_.store(0, std::memory_order_relaxed);
  failed_solves_.store(0, std::memory_order_relaxed);
  deadline_misses_.store(0, std::memory_order_relaxed);
  unenforced_deadlines_.store(0, std::memory_order_relaxed);
  withheld_.store(0, std::memory_order_relaxed);
  cold_withheld_.store(0, std::memory_order_relaxed);
  repeated_skips_.store(0, std::memory_order_relaxed);
  null_handler_hit_.store(false, std::memory_order_relaxed);
  has_prev_out_ = false;
}

bool HandlerMPCThread::SolverThreadsMatch() const noexcept {
  return handler_ == nullptr ||
         handler_->solver_config().num_threads == LaunchConfig().solver_threads;
}

bool HandlerMPCThread::TryCrossModeSwap(const PhaseContext& ctx) {
  // Pick the matching pre-built YAML node. Baseline Phase 6 passes both as
  // Null, so this method is only reached from the cross-mode stretch test.
  YAML::Node cfg;
  if (ctx.ocp_type == "contact_light") {
    cfg = factory_cfg_light_;
  } else if (ctx.ocp_type == "contact_rich") {
    cfg = factory_cfg_rich_;
  } else {
    return false;
  }
  if (!cfg.IsDefined() || cfg.IsNull()) {
    return false;
  }

  std::unique_ptr<MPCHandlerBase> new_handler;
  // The new handler gets this thread's worker capacity, and must keep the
  // thread count the pool was built for: a different count would run its
  // parallel regions on a team the loop entry never named or pinned.
  const MPCFactoryStatus status =
      MPCFactory::Create(cfg, *model_, ctx, new_handler, LaunchConfig().workers.count);
  // Init restores the ICVs it touched; re-stating them is the cheap guarantee
  // that a rebuild on this thread leaves the pool's team size as it was.
  ApplySolverThreadIcvs();
  if (status.error != MPCFactoryError::kNoError || new_handler == nullptr ||
      new_handler->solver_config().num_threads != LaunchConfig().solver_threads) {
    return false;
  }

  if (has_prev_out_) {
    new_handler->SeedWarmStart(prev_out_);
  }
  handler_ = std::move(new_handler);
  return true;
}

double HandlerMPCThread::ConsumedPrimInf(const MPCSolution& sol, std::uint64_t period_ns,
                                         std::uint64_t offset_ns) noexcept {
  if (sol.horizon_length <= 0 || sol.horizon_length > kMaxHorizon || !(sol.dt_node > 0.0)) {
    return std::numeric_limits<double>::infinity();
  }
  // The first node read: the one at or before the delivery; the last: one
  // past the node the period after it ends on.
  const double first_nodes = static_cast<double>(offset_ns) * 1e-9 / sol.dt_node;
  const double nodes = static_cast<double>(offset_ns + period_ns) * 1e-9 / sol.dt_node;
  const double first_whole = std::floor(first_nodes + 1e-9);
  const double whole = std::ceil(nodes - 1e-9);
  const int first = !std::isfinite(first_whole) || first_whole <= 0.0 ? 0
                    : first_whole >= static_cast<double>(sol.horizon_length)
                        ? sol.horizon_length
                        : static_cast<int>(first_whole);
  const int last = std::isfinite(whole) && whole < static_cast<double>(sol.horizon_length)
                       ? static_cast<int>(whole) + 1
                       : sol.horizon_length;
  double r = 0.0;
  for (int k = first; k <= std::min(last, sol.horizon_length); ++k) {
    r = std::max(r, sol.node_prim_inf[static_cast<std::size_t>(k)]);
  }
  return r;
}

bool HandlerMPCThread::Solve(const MPCStateSnapshot& state, MPCSolution& out) {
  const auto entry = std::chrono::steady_clock::now();
  // ── Null-handler guard ────────────────────────────────────────────────
  if (handler_ == nullptr || phase_manager_ == nullptr || model_ == nullptr) {
    null_handler_hit_.store(true, std::memory_order_relaxed);
    failed_solves_.fetch_add(1, std::memory_order_relaxed);
    StashRecord(SolveOutcome::kFailed, -1, nullptr, state, entry);
    return false;
  }
  // One solve per observation: decided before any work, recorded at the
  // attempt so a withheld or failed solve is not retried on the same state.
  if (require_fresh_state_ && state.sequence == last_attempted_sequence_) {
    repeated_skips_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  last_attempted_sequence_ = state.sequence;

  // ── Copy snapshot → Eigen scratch via Map (zero-alloc view) ───────────
  const int nq = model_->nq();
  const int nv = model_->nv();
  // A non-finite state must not reach FK or the phase manager; the handler
  // refuses it with the same code.
  if (state.nq != nq || state.nv != nv || !StateFinite(state)) {
    last_err_.store(static_cast<int>(MPCSolveError::kStateDimMismatch), std::memory_order_relaxed);
    failed_solves_.fetch_add(1, std::memory_order_relaxed);
    total_solves_.fetch_add(1, std::memory_order_relaxed);
    StashRecord(SolveOutcome::kFailed, static_cast<int>(MPCSolveError::kStateDimMismatch), nullptr,
                state, entry);
    return false;
  }
  q_scratch_.head(nq) = Eigen::Map<const Eigen::VectorXd>(state.q.data(), nq);
  v_scratch_.head(nv) = Eigen::Map<const Eigen::VectorXd>(state.v.data(), nv);

  // ── FK for TCP pose (per-tick; reuses pdata_) ─────────────────────────
  // tcp is expressed in base_frame: tcp = oMb⁻¹ · oMf[ee].
  // base가 universe면 oMb = Identity → tcp = oMf[ee] (fast path).
  pinocchio::forwardKinematics(model_->model(), *pdata_, q_scratch_);
  pinocchio::updateFramePlacements(model_->model(), *pdata_);
  const pinocchio::SE3& ee_in_world =
      pdata_->oMf[static_cast<std::size_t>(model_->end_effector_frame_id())];
  const pinocchio::SE3 tcp =
      model_->base_frame_is_universe()
          ? ee_in_world
          : pdata_->oMf[static_cast<std::size_t>(model_->base_frame_id())].actInv(ee_in_world);

  // ── Phase manager tick ────────────────────────────────────────────────
  // Phase time on the snapshot's clock; a new state stream or a stamp that
  // went back continues from the last tick's time.
  if (!have_time_anchor_ || state.state_epoch != anchor_state_epoch_ ||
      state.timestamp_ns < last_stamp_ns_) {
    time_base_s_ = have_time_anchor_ ? last_phase_t_ : 0.0;
    anchor_stamp_ns_ = state.timestamp_ns;
    anchor_state_epoch_ = state.state_epoch;
    have_time_anchor_ = true;
  }
  const double t = time_base_s_ + static_cast<double>(state.timestamp_ns - anchor_stamp_ns_) * 1e-9;
  last_phase_t_ = t;
  last_stamp_ns_ = state.timestamp_ns;
  const PhaseContext ctx =
      phase_manager_->Update(state, q_scratch_, v_scratch_, sensor_scratch_, tcp, t);
  last_phase_id_.store(ctx.phase_id, std::memory_order_relaxed);

  // ── Cross-mode swap branch (try-wrapped; Solve stays noexcept) ────────
  if (ctx.phase_changed && ctx.ocp_type != handler_->ocp_type()) {
    RTC_TRACE_SCOPE("mpc_mode_swap");
    bool swapped = false;
    try {
      swapped = TryCrossModeSwap(ctx);
    } catch (...) {
      swapped = false;
    }
    if (!swapped) {
      last_err_.store(static_cast<int>(MPCSolveError::kRebuildRequired), std::memory_order_relaxed);
      failed_solves_.fetch_add(1, std::memory_order_relaxed);
      total_solves_.fetch_add(1, std::memory_order_relaxed);
      StashRecord(SolveOutcome::kFailed, static_cast<int>(MPCSolveError::kRebuildRequired), nullptr,
                  state, entry);
      return false;
    }
  }

  // ── Solve ─────────────────────────────────────────────────────────────
  MPCSolveError err = MPCSolveError::kNoError;
  {
    RTC_TRACE_SCOPE("mpc_handler_solve");
    err = handler_->Solve(ctx, state, out);
  }
  total_solves_.fetch_add(1, std::memory_order_relaxed);
  last_err_.store(static_cast<int>(err), std::memory_order_relaxed);

  if (err != MPCSolveError::kNoError) {
    failed_solves_.fetch_add(1, std::memory_order_relaxed);
    StashRecord(SolveOutcome::kFailed, static_cast<int>(err), nullptr, state, entry);
    return false;
  }

  const MPCSolverConfig& cfg = handler_->solver_config();
  if (cfg.publish_deadline_ms > 0.0) {
    const auto elapsed = std::chrono::steady_clock::now() - entry;
    if (std::chrono::duration<double, std::milli>(elapsed).count() > cfg.publish_deadline_ms) {
      if (ExternalTrigger()) {
        // The consumer waits for this solve: the deadline is a count here,
        // not a gate (the lock-step's "would have missed").
        unenforced_deadlines_.fetch_add(1, std::memory_order_relaxed);
      } else {
        deadline_misses_.fetch_add(1, std::memory_order_relaxed);
        StashRecord(SolveOutcome::kDeadline, 0, &out, state, entry);
        return false;
      }
    }
  }
  if (out.cold_start && !out.converged) {
    cold_withheld_.fetch_add(1, std::memory_order_relaxed);
    StashRecord(SolveOutcome::kColdWithheld, 0, &out, state, entry);
    return false;
  }
  if (!out.converged && cfg.publish_prim_tol > 0.0 &&
      !(ConsumedPrimInf(out, PeriodNs(), consumed_offset_ns_) <= cfg.publish_prim_tol)) {
    withheld_.fetch_add(1, std::memory_order_relaxed);
    StashRecord(SolveOutcome::kWithheld, 0, &out, state, entry);
    return false;
  }

  // Save for next tick's cross-mode seed (trivially-copyable memcpy).
  prev_out_ = out;
  has_prev_out_ = true;
  StashRecord(SolveOutcome::kPublished, 0, &out, state, entry);
  return true;
}

void HandlerMPCThread::StashRecord(SolveOutcome outcome, int err, const MPCSolution* out,
                                   const MPCStateSnapshot& state,
                                   std::chrono::steady_clock::time_point entry) noexcept {
  if (record_sink_ == nullptr) {
    return;
  }
  SolveRecord& r = pending_record_;
  r = SolveRecord{};
  r.outcome = outcome;
  r.err = err;
  r.state_sequence = state.sequence;
  r.state_epoch = state.state_epoch;
  r.compute_us =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - entry).count();
  if (out != nullptr) {
    r.iterations = out->iterations;
    r.converged = out->converged;
    r.cold_start = out->cold_start;
    r.prim_inf = out->residuals.prim_inf;
    r.consumed_prim = ConsumedPrimInf(*out, PeriodNs(), consumed_offset_ns_);
  } else {
    r.prim_inf = std::numeric_limits<double>::quiet_NaN();
    r.consumed_prim = std::numeric_limits<double>::quiet_NaN();
  }
  record_pending_ = true;
}

void HandlerMPCThread::OnTickEnd(bool published, std::uint64_t plan_id) noexcept {
  if (!record_pending_ || record_sink_ == nullptr) {
    return;
  }
  record_pending_ = false;
  const auto k = static_cast<std::size_t>(pending_record_.outcome);
  if (k < record_counts_.size()) {
    ++record_counts_[k];
  }
  SolveRecord& r = record_sink_->StartWrite();
  r = pending_record_;
  r.seq = ++record_seq_;
  r.plan_id = published && pending_record_.outcome == SolveOutcome::kPublished ? plan_id : 0;
  r.counts = record_counts_;
  record_sink_->FinishWrite();
}

}  // namespace rtc::mpc
