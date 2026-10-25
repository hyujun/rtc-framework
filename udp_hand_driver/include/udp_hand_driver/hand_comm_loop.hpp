#ifndef UDP_HAND_DRIVER_HAND_COMM_LOOP_HPP_
#define UDP_HAND_DRIVER_HAND_COMM_LOOP_HPP_

// HandCommLoop — the one self-clocked comm thread (`hand_comm`, SCHED_FIFO 65)
// that drives every hand of the process.
//
// Each tick runs one comm cycle per hand, in Add() order, back to back on the
// same thread. That order IS the bus serialization of a two-hand process (epic
// #906 D69): the second hand's transaction can only start once the first one's
// receive window has closed, so no lock, no IPC and no phase grid is needed.
// With one hand the loop is exactly the per-controller loop it replaced.
//
// E-Stop (E-8, D70). Every hand checks the shared latch at the head of its own
// cycle (UdpHandController::RunCommCycle) and the loop checks it again between
// hands, so a latch raised during one hand's receive window stops the next hand
// before its transaction instead of a whole period later. Either way the abort
// is one sequence: every hand stops accepting work (MarkLoopStopping, before
// any zero-write — its subscribers gate on IsRunning()), the tick's timing row
// is dropped, the loop exits, and OnLoopAborted() zero-writes each hand once,
// in Add() order. Worst-case latency: one period plus one receive window.
//
// Timing (D72): one RtTickTimingPayload per tick, covering every hand. The
// state / compute breakpoints are the last hand's (MarkState / MarkCompute keep
// the latest stamp); a tick's row is dropped only when every hand that ran
// asked for it (a decimated skip on all of them), so one hand's skip cannot
// hide the other's transaction.
//
// Broadcast REQ (D75, not implemented): owning the tick is what a future
// "one REQ to 0xFF, every hand answers" cycle needs — drain all hands, send the
// broadcast once, then open each hand's receive stage. That change lands here
// and in the comm, not in the node.

#include "rtc_base/threading/periodic_rt_thread.hpp"

#include <array>
#include <atomic>
#include <cstddef>

namespace udp_hand_driver {

class HandCommLoop;

/// One hand as HandCommLoop sees it. UdpHandController implements it; tests
/// drive the loop with scripted members. Every method runs on the loop thread.
class HandCommLoopMember {
 public:
  virtual ~HandCommLoopMember() = default;

  HandCommLoopMember(const HandCommLoopMember&) = delete;
  HandCommLoopMember& operator=(const HandCommLoopMember&) = delete;
  HandCommLoopMember(HandCommLoopMember&&) = delete;
  HandCommLoopMember& operator=(HandCommLoopMember&&) = delete;

  /// One comm cycle of this hand (RT path: noexcept, allocation-free).
  virtual void RunCommCycle() noexcept = 0;

  /// The loop is about to exit, or has: stop accepting work. Called for every
  /// hand on the loop thread before any OnCommLoopAborted() (E-Stop), and by
  /// StopAll() on the stopping thread after the join. Must be a plain atomic
  /// store — it runs on either thread.
  virtual void MarkLoopStopping() noexcept = 0;

  /// Terminal E-Stop zero-write, once per abort, after the tick loop unwound.
  virtual void OnCommLoopAborted() noexcept = 0;

 protected:
  HandCommLoopMember() = default;
};

class HandCommLoop final : public rtc::PeriodicRtThread {
 public:
  /// Hands one bus can serve per tick (D69: W4 budget, `N × 2T < slot`).
  static constexpr std::size_t kMaxHands = 2;

  HandCommLoop() = default;

  // Joined here, not only in ~PeriodicRtThread: by the time the base destructor
  // runs this object's vtable is gone, and a tick still in flight would call a
  // pure virtual.
  ~HandCommLoop() override { Join(); }

  HandCommLoop(const HandCommLoop&) = delete;
  HandCommLoop& operator=(const HandCommLoop&) = delete;
  HandCommLoop(HandCommLoop&&) = delete;
  HandCommLoop& operator=(HandCommLoop&&) = delete;

  // ── Membership (non-RT, loop not running) ──────────────────────────────

  /// Append `member` to the tick order. Refused (false) for nullptr, a member
  /// already present, a full loop, or a running loop. The member must outlive
  /// every run of this loop.
  [[nodiscard]] bool Add(HandCommLoopMember* member) noexcept {
    if (member == nullptr || count_ >= kMaxHands || Running()) {
      return false;
    }
    for (std::size_t i = 0; i < count_; ++i) {
      if (members_[i] == member) {
        return false;
      }
    }
    members_[count_++] = member;
    return true;
  }

  /// Take `member` out of the tick order, keeping the others' order. Refused
  /// (false) while running or when it is not a member. A member's destructor
  /// calls it, so the loop never holds a pointer to a destroyed member.
  [[nodiscard]] bool Remove(const HandCommLoopMember* member) noexcept {
    if (Running()) {
      return false;
    }
    for (std::size_t i = 0; i < count_; ++i) {
      if (members_[i] == member) {
        for (std::size_t j = i + 1; j < count_; ++j) {
          members_[j - 1] = members_[j];
        }
        members_[--count_] = nullptr;
        return true;
      }
    }
    return false;
  }

  /// Forget every member. Refused (false) while running.
  [[nodiscard]] bool Clear() noexcept {
    if (Running()) {
      return false;
    }
    members_.fill(nullptr);
    count_ = 0;
    return true;
  }

  [[nodiscard]] std::size_t Count() const noexcept { return count_; }

  /// Join the loop and mark every hand on it stopped (non-RT). What stopping
  /// one hand of a shared loop means: the thread that ran it ran them all, so
  /// none may keep reporting IsRunning() over a loop that is gone (its
  /// detector would read the frozen cycle count as a dead link).
  void StopAll() noexcept {
    Join();
    for (std::size_t i = 0; i < count_; ++i) {
      members_[i]->MarkLoopStopping();
    }
  }

  /// The shared E-Stop latch the loop re-checks between hands (each hand
  /// checks its own at the head of its cycle, so this is in addition to the
  /// one given to every member, not instead of it). Owned by the caller,
  /// outlives the loop; nullptr disables only the between-hands check — the
  /// E-Stop then waits for the next hand's head check. Before Start().
  void SetEstopFlag(const std::atomic<bool>* flag) noexcept { estop_flag_ = flag; }

  // ── Called by members from inside RunCommCycle (loop thread) ───────────

  /// End of the read phase (base t1). The last hand's stamp wins.
  void MarkState() noexcept { MarkStateAcquired(); }

  /// End of the compute phase (base t2). The last hand's stamp wins.
  void MarkCompute() noexcept { MarkComputeDone(); }

  /// This hand did no representative work this tick (decimated skip). The
  /// row is dropped only when every hand that ran this tick asked.
  void SuppressTiming() noexcept { ++suppress_votes_; }

  /// Terminal E-Stop: stop every hand, drop this tick's row, exit the loop
  /// after this tick. Pure stores (RequestLoopExit is an atomic store) — safe
  /// on the RT path. Idempotent within one abort.
  void AbortForEstop() noexcept {
    if (aborting_) {
      return;
    }
    aborting_ = true;
    for (std::size_t i = 0; i < count_; ++i) {
      members_[i]->MarkLoopStopping();
    }
    SuppressTimingThisTick();
    RequestLoopExit();
  }

 protected:
  void OnTick() override {
    suppress_votes_ = 0;
    std::size_t ran = 0;
    for (std::size_t i = 0; i < count_; ++i) {
      // Between hands only: the first hand's own head check covers the tick
      // head, so with one hand this loop adds no load and no behaviour.
      if (i > 0 && estop_flag_ != nullptr && estop_flag_->load(std::memory_order_acquire)) {
        AbortForEstop();
        return;
      }
      members_[i]->RunCommCycle();
      ++ran;
      if (aborting_) {
        return;  // a hand hit the latch at its head; the rest must not transact
      }
    }
    if (ran > 0 && suppress_votes_ >= ran) {
      SuppressTimingThisTick();
    }
  }

  // Once, on the loop thread, after an AbortForEstop() tick (the base calls it
  // when the loop drops out on RequestLoopExit). Zero-writes each hand in tick
  // order, then re-arms for the next Start() — this is the last thing the loop
  // thread does, so the plain flag needs no synchronization with Start().
  void OnLoopAborted() noexcept override {
    for (std::size_t i = 0; i < count_; ++i) {
      members_[i]->OnCommLoopAborted();
    }
    aborting_ = false;
  }

 private:
  std::array<HandCommLoopMember*, kMaxHands> members_{};
  std::size_t count_{0};
  const std::atomic<bool>* estop_flag_{nullptr};

  // Loop thread only.
  std::size_t suppress_votes_{0};
  bool aborting_{false};
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_HAND_COMM_LOOP_HPP_
