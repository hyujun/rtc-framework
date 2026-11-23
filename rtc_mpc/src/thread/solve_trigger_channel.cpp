#include "rtc_mpc/thread/solve_trigger_channel.hpp"

#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>

namespace rtc::mpc {

bool SolveTriggerChannel::Open() noexcept {
  Close();
  trigger_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  done_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (!Opened()) {
    Close();
    return false;
  }
  return true;
}

void SolveTriggerChannel::Close() noexcept {
  armed_.store(false, std::memory_order_release);
  if (trigger_fd_ >= 0) {
    close(trigger_fd_);
    trigger_fd_ = -1;
  }
  if (done_fd_ >= 0) {
    close(done_fd_);
    done_fd_ = -1;
  }
}

void SolveTriggerChannel::Drain(int fd) noexcept {
  eventfd_t value{};
  static_cast<void>(eventfd_read(fd, &value));  // EAGAIN: nothing to drain
}

void SolveTriggerChannel::Write(int fd) noexcept {
  static_cast<void>(eventfd_write(fd, 1));  // the counter saturating is not a lost wake
}

void SolveTriggerChannel::Arm() noexcept {
  if (!Opened()) {
    return;
  }
  // Nothing outstanding: whatever an earlier run requested is answered.
  completed_id_.store(requested_id_.load(std::memory_order_acquire), std::memory_order_release);
  Drain(trigger_fd_);
  Drain(done_fd_);
  armed_.store(true, std::memory_order_release);
}

void SolveTriggerChannel::Disarm() noexcept {
  armed_.store(false, std::memory_order_release);
  Nudge();
}

void SolveTriggerChannel::Nudge() noexcept {
  if (trigger_fd_ >= 0) {
    Write(trigger_fd_);
  }
  if (done_fd_ >= 0) {
    Write(done_fd_);
  }
}

void SolveTriggerChannel::RequestSolve() noexcept {
  if (!armed_.load(std::memory_order_acquire)) {
    ignored_requests_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (!Idle()) {
    coalesced_requests_.fetch_add(1, std::memory_order_relaxed);
  }
  requested_id_.fetch_add(1, std::memory_order_release);
  Write(trigger_fd_);
}

SolveTriggerChannel::WaitStatus SolveTriggerChannel::WaitIdle(std::int64_t timeout_ns) noexcept {
  using Clock = std::chrono::steady_clock;
  const Clock::time_point deadline = Clock::now() + std::chrono::nanoseconds(timeout_ns);
  for (;;) {
    if (!armed_.load(std::memory_order_acquire)) {
      return WaitStatus::kCancelled;
    }
    if (Idle()) {
      return WaitStatus::kIdle;
    }
    const auto left = std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now()).count();
    if (left <= 0) {
      return WaitStatus::kTimeout;
    }

    struct pollfd pfd {};

    pfd.fd = done_fd_;
    pfd.events = POLLIN;
    const int rc = poll(&pfd, 1, static_cast<int>(left));
    if (rc < 0 && errno != EINTR) {
      return WaitStatus::kCancelled;  // a closed fd: nothing to wait on
    }
    if (rc > 0) {
      Drain(done_fd_);
    }
  }
}

bool SolveTriggerChannel::WaitTrigger(int timeout_ms) noexcept {
  if (!Opened()) {
    return false;
  }

  struct pollfd pfd {};

  pfd.fd = trigger_fd_;
  pfd.events = POLLIN;
  const int rc = poll(&pfd, 1, timeout_ms);
  if (rc > 0) {
    Drain(trigger_fd_);
  }
  return armed_.load(std::memory_order_acquire) && !Idle();
}

void SolveTriggerChannel::MarkCompleted(std::uint64_t id) noexcept {
  completed_id_.store(id, std::memory_order_release);
  if (done_fd_ >= 0) {
    Write(done_fd_);
  }
}

}  // namespace rtc::mpc
