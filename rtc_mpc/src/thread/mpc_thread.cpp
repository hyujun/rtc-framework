#include "rtc_mpc/thread/mpc_thread.hpp"

#include "rtc_base/tracing/trace_scope.hpp"

namespace rtc::mpc {

MPCThread::~MPCThread() {
  StopAndJoin();
}

void MPCThread::StopAndJoin() noexcept {
  PeriodicRtThread::Join();
}

void MPCThread::Init(MPCSolutionManager& manager,
                     const MpcThreadLaunchConfig& launch_config) noexcept {
  manager_ = &manager;
  launch_config_ = launch_config;
  if (launch_config_.target_frequency_hz <= 0.0) {
    launch_config_.target_frequency_hz = 20.0;
  }
  initialised_ = (manager_ != nullptr);
}

void MPCThread::Start() {
  if (!initialised_ || Running()) {
    return;
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
  RTC_TRACE_SCOPE("mpc_tick");

  // Phase 1: state acquisition.
  MPCStateSnapshot state;
  {
    RTC_TRACE_SCOPE("mpc_read_state");
    state = manager_->ReadState();
    MarkStateAcquired();
  }

  // Phase 2: solve.
  bool ok = false;
  {
    RTC_TRACE_SCOPE("mpc_solve");
    ok = Solve(state, scratch_);
    MarkComputeDone();
  }

  // Phase 3: publish. PublishSolution is noexcept and runs off the RT
  // control loop, so the SPSC handoff is RT-safe for the controller side.
  if (ok) {
    RTC_TRACE_SCOPE("mpc_publish");
    manager_->PublishSolution(scratch_);
  }
}

}  // namespace rtc::mpc
