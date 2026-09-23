#include "integrated_bringup/controllers/catching/planner_thread.hpp"

#include "rtc_base/types/types.hpp"  // rtc::SteadyNowNs

#include <poll.h>
#include <sys/eventfd.h>

#include <algorithm>
#include <cmath>

namespace integrated_bringup {

CatchingPlannerThread::CatchingPlannerThread(rtc::catching::PlannerCycle& cycle, int wake_fd,
                                             double wake_timeout_s) noexcept
    : cycle_(cycle),
      wake_fd_(wake_fd),
      // Rounded UP to whole milliseconds (poll's unit), never below one: a
      // zero timeout would turn the wait into a busy loop on a FIFO thread.
      timeout_ms_(std::max(1, static_cast<int>(std::ceil(wake_timeout_s * 1000.0)))),
      frequency_hz_(wake_timeout_s > 0.0 ? 1.0 / wake_timeout_s : 0.0) {}

CatchingPlannerThread::~CatchingPlannerThread() {
  Join();
}

void CatchingPlannerThread::StartWith(const rtc::ThreadConfig& thread_config) {
  SetTimingProducer(&timing_);
  rtc::PeriodicRtThreadConfig cfg;
  cfg.thread_config = thread_config;
  cfg.frequency_hz = frequency_hz_;
  Start(cfg);
}

bool CatchingPlannerThread::Signal(int wake_fd) noexcept {
  if (wake_fd < 0) {
    return false;
  }
  return eventfd_write(wake_fd, 1) == 0;
}

bool CatchingPlannerThread::Drain() noexcept {
  // Non-semaphore eventfd: one read returns the whole counter and zeroes it,
  // so a burst of signals is consumed as one. EAGAIN (nothing pending) is the
  // non-blocking fd saying "empty".
  eventfd_t value = 0;
  return eventfd_read(wake_fd_, &value) == 0 && value > 0;
}

rtc::PeriodicRtThread::WaitResult CatchingPlannerThread::WaitForNextTick() noexcept {
  struct pollfd pfd {};

  pfd.fd = wake_fd_;
  pfd.events = POLLIN;
  const int rc = poll(&pfd, 1, timeout_ms_);
  if (rc > 0 && Drain()) {
    signal_wakes_.fetch_add(1, std::memory_order_relaxed);
  } else {
    // A timeout, EINTR, or a readable fd whose counter another reader emptied:
    // all of them are "no new trajectory", and the planner still runs so a
    // moving RT state is re-planned against at the timeout cadence.
    timeout_wakes_.fetch_add(1, std::memory_order_relaxed);
  }
  return WaitResult::kProceed;
}

void CatchingPlannerThread::OnRequestStop() noexcept {
  // Called on the stopping thread: nudge the poll so Join does not wait out
  // the timeout.
  static_cast<void>(Signal(wake_fd_));
}

void CatchingPlannerThread::OnTick() noexcept {
  const rtc::catching::NowReal wake{rtc::SteadyNowNs()};
  MarkStateAcquired();
  const rtc::catching::PlannerCycleRecord rec = cycle_.Run(wake);
  MarkComputeDone();
  if (rec.reset_seen) {
    // L7 §4.8: a signal raised for the trial the reset just ended must not
    // wake the planner into the next one. Whatever is pending now predates
    // this wake's view of the reset.
    static_cast<void>(Drain());
    resets_seen_.fetch_add(1, std::memory_order_relaxed);
  }
  if (rec.outcome == rtc::catching::CycleOutcome::kPublished) {
    published_.fetch_add(1, std::memory_order_relaxed);
  } else if (rec.outcome == rtc::catching::CycleOutcome::kSuperseded) {
    superseded_.fetch_add(1, std::memory_order_relaxed);
  }
  last_record_.Store(rec);
}

}  // namespace integrated_bringup
