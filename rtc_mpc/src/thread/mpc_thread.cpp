#include "rtc_mpc/thread/mpc_thread.hpp"

#include "rtc_base/threading/thread_utils.hpp"

namespace rtc::mpc {

MPCThread::~MPCThread() { StopAndJoin(); }

void MPCThread::StopAndJoin() noexcept {
  // Base joins the main loop first; afterwards we tear down the worker
  // sleepers so any solver-pool shutdown ordering stays correct.
  PeriodicRtThread::Join();
  for (int i = 0; i < launch_config_.num_workers; ++i) {
    auto &worker = workers_[static_cast<std::size_t>(i)];
    if (worker.joinable()) {
      worker.request_stop();
      worker.join();
    }
  }
}

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
  if (!initialised_ || Running()) {
    return;
  }
  // Workers are started first so that any solver which consumes the worker
  // span finds them live on first iteration. Concrete solvers (e.g.
  // Aligator) schedule tasks onto these passive sleepers via their own
  // worker pool.
  for (int i = 0; i < launch_config_.num_workers; ++i) {
    const rtc::ThreadConfig &wcfg =
        launch_config_.workers[static_cast<std::size_t>(i)];
    workers_[static_cast<std::size_t>(i)] =
        std::jthread([wcfg](std::stop_token /*stoken*/) {
          (void)rtc::ApplyThreadConfig(wcfg);
        });
  }

  // Wire the SPSC ring before the base spawns its loop thread so the very
  // first tick already pushes a sample.
  SetTimingProducer<rtc::kMpcTimingBufferCapacity>(&timing_producer_);

  rtc::PeriodicRtThreadConfig pcfg{};
  pcfg.thread_config = launch_config_.main;
  pcfg.frequency_hz = launch_config_.target_frequency_hz;
  PeriodicRtThread::Start(pcfg);
}

void MPCThread::OnTick() noexcept {
  // Phase 1: state acquisition.
  const MPCStateSnapshot state = manager_->ReadState();
  MarkStateAcquired();

  // Phase 2: solve. Worker span lets parallel solvers schedule onto the
  // class-owned worker threads.
  std::span<std::jthread> worker_span(
      workers_.data(), static_cast<std::size_t>(launch_config_.num_workers));
  const bool ok = Solve(state, scratch_, worker_span);
  MarkComputeDone();

  // Phase 3: publish. PublishSolution is noexcept and runs off the 500 Hz
  // RT loop, so the SPSC handoff is RT-safe for the controller side.
  if (ok) {
    manager_->PublishSolution(scratch_);
  }
}

} // namespace rtc::mpc
