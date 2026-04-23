#include "rtc_mpc/thread/handler_mpc_thread.hpp"

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
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <cstddef>
#include <cstdio>
#include <memory>
#include <utility>

namespace rtc::mpc {

void HandlerMPCThread::Configure(
    const RobotModelHandler &model_handler,
    std::unique_ptr<MPCHandlerBase> handler,
    std::unique_ptr<PhaseManagerBase> phase_manager,
    YAML::Node factory_cfg_light, YAML::Node factory_cfg_rich) noexcept {
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

  start_time_ = std::chrono::steady_clock::now();

  last_err_.store(0, std::memory_order_relaxed);
  last_phase_id_.store(-1, std::memory_order_relaxed);
  total_solves_.store(0, std::memory_order_relaxed);
  failed_solves_.store(0, std::memory_order_relaxed);
  null_logged_.store(false, std::memory_order_relaxed);
  last_warn_ns_.store(0, std::memory_order_relaxed);
  has_prev_out_ = false;
}

void HandlerMPCThread::WarnThrottled(const char *what, int code) noexcept {
  constexpr std::int64_t kWarnThrottleNs = 5'000'000'000LL; // 5 s
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
  const auto last = last_warn_ns_.load(std::memory_order_relaxed);
  if (last != 0 && now_ns - last < kWarnThrottleNs) {
    return;
  }
  last_warn_ns_.store(now_ns, std::memory_order_relaxed);
  std::fprintf(
      stderr, "[HandlerMPCThread] %s code=%d total=%lu failed=%lu\n", what,
      code,
      static_cast<unsigned long>(total_solves_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(
          failed_solves_.load(std::memory_order_relaxed)));
}

bool HandlerMPCThread::TryCrossModeSwap(const PhaseContext &ctx) {
  // Pick the matching pre-built YAML node. Baseline Phase 6 passes both as
  // Null, so this method is only reached from the cross-mode stretch test.
  YAML::Node cfg;
  if (ctx.ocp_type == "light_contact") {
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
  const MPCFactoryStatus status =
      MPCFactory::Create(cfg, *model_, ctx, new_handler);
  if (status.error != MPCFactoryError::kNoError || new_handler == nullptr) {
    return false;
  }

  if (has_prev_out_) {
    new_handler->SeedWarmStart(prev_out_);
  }
  handler_ = std::move(new_handler);
  return true;
}

bool HandlerMPCThread::Solve(const MPCStateSnapshot &state, MPCSolution &out,
                             std::span<std::jthread> /*workers*/) {
  // ── Null-handler guard ────────────────────────────────────────────────
  if (handler_ == nullptr || phase_manager_ == nullptr || model_ == nullptr) {
    if (!null_logged_.exchange(true, std::memory_order_relaxed)) {
      std::fprintf(stderr,
                   "[HandlerMPCThread] handler_ / phase_manager_ / model_ is "
                   "null; skipping solve (one-shot log)\n");
    }
    failed_solves_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  // ── Copy snapshot → Eigen scratch via Map (zero-alloc view) ───────────
  const int nq = model_->nq();
  const int nv = model_->nv();
  if (state.nq != nq || state.nv != nv) {
    last_err_.store(static_cast<int>(MPCSolveError::kStateDimMismatch),
                    std::memory_order_relaxed);
    failed_solves_.fetch_add(1, std::memory_order_relaxed);
    total_solves_.fetch_add(1, std::memory_order_relaxed);
    WarnThrottled("state dim mismatch",
                  static_cast<int>(MPCSolveError::kStateDimMismatch));
    return false;
  }
  q_scratch_.head(nq) = Eigen::Map<const Eigen::VectorXd>(state.q.data(), nq);
  v_scratch_.head(nv) = Eigen::Map<const Eigen::VectorXd>(state.v.data(), nv);

  // ── FK for TCP pose (per-tick; reuses pdata_) ─────────────────────────
  pinocchio::forwardKinematics(model_->model(), *pdata_, q_scratch_);
  pinocchio::updateFramePlacements(model_->model(), *pdata_);
  const pinocchio::SE3 &tcp =
      pdata_->oMf[static_cast<std::size_t>(model_->end_effector_frame_id())];

  // ── Phase manager tick ────────────────────────────────────────────────
  const double t = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - start_time_)
                       .count();
  const PhaseContext ctx =
      phase_manager_->Update(q_scratch_, v_scratch_, sensor_scratch_, tcp, t);
  last_phase_id_.store(ctx.phase_id, std::memory_order_relaxed);

  // ── Cross-mode swap branch (try-wrapped; Solve stays noexcept) ────────
  if (ctx.phase_changed && ctx.ocp_type != handler_->ocp_type()) {
    bool swapped = false;
    try {
      swapped = TryCrossModeSwap(ctx);
    } catch (...) {
      swapped = false;
    }
    if (!swapped) {
      last_err_.store(static_cast<int>(MPCSolveError::kRebuildRequired),
                      std::memory_order_relaxed);
      failed_solves_.fetch_add(1, std::memory_order_relaxed);
      total_solves_.fetch_add(1, std::memory_order_relaxed);
      WarnThrottled("cross-mode swap failed",
                    static_cast<int>(MPCSolveError::kRebuildRequired));
      return false;
    }
  }

  // ── Solve ─────────────────────────────────────────────────────────────
  const MPCSolveError err = handler_->Solve(ctx, state, out);
  total_solves_.fetch_add(1, std::memory_order_relaxed);
  last_err_.store(static_cast<int>(err), std::memory_order_relaxed);

  if (err != MPCSolveError::kNoError) {
    failed_solves_.fetch_add(1, std::memory_order_relaxed);
    WarnThrottled("handler solve error", static_cast<int>(err));
    return false;
  }

  // Save for next tick's cross-mode seed (trivially-copyable memcpy).
  prev_out_ = out;
  has_prev_out_ = true;
  return true;
}

} // namespace rtc::mpc
