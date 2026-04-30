#include "rtc_mpc/thread/mpc_thread.hpp"

#include "rtc_base/threading/thread_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace rtc::mpc {

MPCThread::~MPCThread() { Join(); }

void MPCThread::Init(MPCSolutionManager &manager,
                     const MpcThreadLaunchConfig &launch_config) noexcept {
  manager_ = &manager;
  launch_config_ = launch_config;
  if (launch_config_.num_workers < 0) {
    launch_config_.num_workers = 0;
  }
  if (launch_config_.num_workers > kMaxMpcWorkers) {
    launch_config_.num_workers = kMaxMpcWorkers;
  }
  if (launch_config_.target_frequency_hz <= 0.0) {
    launch_config_.target_frequency_hz = 20.0;
  }
  initialised_ = (manager_ != nullptr);
}

void MPCThread::Start() {
  if (!initialised_ || running_.exchange(true)) {
    return;
  }
  // Workers are started first so that any solver which consumes the worker
  // span finds them live on first iteration.
  for (int i = 0; i < launch_config_.num_workers; ++i) {
    const rtc::ThreadConfig &wcfg =
        launch_config_.workers[static_cast<std::size_t>(i)];
    workers_[static_cast<std::size_t>(i)] =
        std::jthread([wcfg](std::stop_token /*stoken*/) {
          (void)rtc::ApplyThreadConfig(wcfg);
          // Workers are passive: they apply their thread config and then
          // sleep until destruction. Concrete solvers (e.g. Aligator)
          // schedule tasks onto them via their own worker pool.
        });
  }
  main_thread_ =
      std::jthread([this](std::stop_token stoken) { RunMain(stoken); });
}

void MPCThread::RequestStop() noexcept {
  main_thread_.request_stop();
  for (int i = 0; i < launch_config_.num_workers; ++i) {
    workers_[static_cast<std::size_t>(i)].request_stop();
  }
  // Wake any pause-blocked main loop so it observes stop_requested and
  // exits promptly rather than waiting indefinitely on the cv.
  { std::lock_guard<std::mutex> lock(pause_mutex_); }
  pause_cv_.notify_all();
}

void MPCThread::Pause() noexcept {
  paused_.store(true);
  // No notify needed — the loop polls paused_ at the top of each iteration
  // and enters the cv wait on its own. notify is only required to wake from
  // the wait (Resume / RequestStop).
}

void MPCThread::Resume() noexcept {
  {
    std::lock_guard<std::mutex> lock(pause_mutex_);
    paused_.store(false);
  }
  pause_cv_.notify_all();
}

void MPCThread::Join() noexcept {
  RequestStop();
  if (main_thread_.joinable()) {
    main_thread_.join();
  }
  for (int i = 0; i < launch_config_.num_workers; ++i) {
    auto &worker = workers_[static_cast<std::size_t>(i)];
    if (worker.joinable()) {
      worker.join();
    }
  }
  running_.store(false);
}

void MPCThread::RunMain(std::stop_token stoken) {
  (void)rtc::ApplyThreadConfig(launch_config_.main);

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / launch_config_.target_frequency_hz));
  const double budget_us = static_cast<double>(period.count()) / 1000.0;

  auto next_wake = std::chrono::steady_clock::now();
  MPCSolution scratch{};

  std::span<std::jthread> worker_span(
      workers_.data(), static_cast<std::size_t>(launch_config_.num_workers));

  // Period-vs-budget jitter is computed against the previous tick's t0.
  // First tick has no predecessor so jitter is reported as 0.
  bool have_prev_t0 = false;
  std::chrono::steady_clock::time_point prev_t0{};

  while (!stoken.stop_requested()) {
    // Pause gate: block here when Pause() has been called. Resume() /
    // RequestStop() both notify the cv. Re-anchor next_wake on resume so
    // we don't burst-catch-up cycles that elapsed during the pause. The
    // jitter chain is also broken across a pause.
    if (paused_.load()) {
      std::unique_lock<std::mutex> lock(pause_mutex_);
      pause_cv_.wait(lock, [this, &stoken]() {
        return !paused_.load() || stoken.stop_requested();
      });
      if (stoken.stop_requested()) {
        break;
      }
      next_wake = std::chrono::steady_clock::now();
      have_prev_t0 = false;
    }

    // ── Phase 0: tick start ──────────────────────────────────────────────
    const auto t0 = std::chrono::steady_clock::now();

    // ── Phase 1: state acquisition ───────────────────────────────────────
    const MPCStateSnapshot state = manager_->ReadState();
    const auto t1 = std::chrono::steady_clock::now();

    // ── Phase 2: solve ───────────────────────────────────────────────────
    const bool ok = Solve(state, scratch, worker_span);
    const auto t2 = std::chrono::steady_clock::now();

    // ── Phase 3: publish ─────────────────────────────────────────────────
    if (ok) {
      manager_->PublishSolution(scratch);
    }
    const auto t3 = std::chrono::steady_clock::now();

    // ── Phase 4: per-tick timing → SPSC producer ─────────────────────────
    rtc::RtTickTimingPayload payload{};
    payload.t_state_us =
        std::chrono::duration<double, std::micro>(t1 - t0).count();
    payload.t_compute_us =
        std::chrono::duration<double, std::micro>(t2 - t1).count();
    payload.t_publish_us =
        std::chrono::duration<double, std::micro>(t3 - t2).count();
    payload.t_total_us =
        std::chrono::duration<double, std::micro>(t3 - t0).count();
    if (have_prev_t0) {
      const double actual_period_us =
          std::chrono::duration<double, std::micro>(t0 - prev_t0).count();
      payload.jitter_us = std::abs(actual_period_us - budget_us);
    }
    prev_t0 = t0;
    have_prev_t0 = true;
    (void)timing_producer_.Push(payload);

    next_wake += period;
    // If we ran long, skip ahead to the next period rather than piling up
    // back-to-back solves. Break the jitter chain since prev_t0 no longer
    // reflects a regular cadence.
    const auto now = std::chrono::steady_clock::now();
    if (next_wake < now) {
      next_wake = now + period;
      have_prev_t0 = false;
    }
    std::this_thread::sleep_until(next_wake);
  }
}

} // namespace rtc::mpc
