#include "rtc_mpc/thread/mpc_thread.hpp"

#include "rtc_base/threading/thread_utils.hpp"

#include <algorithm>
#include <chrono>

namespace rtc::mpc {

MPCThread::~MPCThread() { Join(); }

void MPCThread::Init(MPCSolutionManager& manager,
                     const MpcThreadLaunchConfig& launch_config) noexcept {
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
    const rtc::ThreadConfig& wcfg = launch_config_.workers[
        static_cast<std::size_t>(i)];
    workers_[static_cast<std::size_t>(i)] =
        std::jthread([wcfg](std::stop_token /*stoken*/) {
          (void)rtc::ApplyThreadConfig(wcfg);
          // Workers are passive: they apply their thread config and then
          // sleep until destruction. Concrete solvers (e.g. Aligator)
          // schedule tasks onto them via their own worker pool.
        });
  }
  main_thread_ = std::jthread([this](std::stop_token stoken) {
    RunMain(stoken);
  });
}

void MPCThread::RequestStop() noexcept {
  main_thread_.request_stop();
  for (int i = 0; i < launch_config_.num_workers; ++i) {
    workers_[static_cast<std::size_t>(i)].request_stop();
  }
}

void MPCThread::Join() noexcept {
  RequestStop();
  if (main_thread_.joinable()) {
    main_thread_.join();
  }
  for (int i = 0; i < launch_config_.num_workers; ++i) {
    auto& worker = workers_[static_cast<std::size_t>(i)];
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

  auto next_wake = std::chrono::steady_clock::now();
  MPCSolution scratch{};

  std::span<std::jthread> worker_span(
      workers_.data(),
      static_cast<std::size_t>(launch_config_.num_workers));

  while (!stoken.stop_requested()) {
    const MPCStateSnapshot state = manager_->ReadState();
    const bool ok = Solve(state, scratch, worker_span);
    if (ok) {
      manager_->PublishSolution(scratch);
    }
    next_wake += period;
    // If we ran long, skip ahead to the next period rather than piling up
    // back-to-back solves.
    const auto now = std::chrono::steady_clock::now();
    if (next_wake < now) {
      next_wake = now + period;
    }
    std::this_thread::sleep_until(next_wake);
  }
}

}  // namespace rtc::mpc
