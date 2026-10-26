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
// Shared request (#956, the node's `req_mode broadcast`, D82). One REQ that
// every hand answers replaces the per-hand REQs, so the tick is no longer one
// whole cycle per hand but four passes over the hands:
//   1. PrepareSharedReqCycle() per hand, in Add() order: the cycle head
//      (E-Stop · decimation · fake · drain · latch) and that hand's CMD pair
//      — every hand drains before the REQ draws any answer;
//   2. SendSharedReq() on the first hand that takes part: the one REQ — when
//      that send fails, the next taking-part hand's socket tries, so one
//      socket's tx fault does not fail every hand's set;
//   3. ReceiveSharedReq() per taking-part hand: its receive stage against the
//      bound anchored at that REQ — back to back, before any tail, so a later
//      hand starts receiving within microseconds of the earlier one's close
//      and a frame queued by then arrived by then (a tail between them would
//      let a frame that came after the bound pass as in time);
//   4. FinishSharedReqCycle() per taking-part hand: the cycle tail.
// A hand that does not take part (decimated skip, fake model — its whole
// cycle ran in step 1) gets no step 3 or 4; with none taking part no REQ goes
// out. E-Stop (E-8): the latch is re-checked before every Prepare after the
// first (each hand also checks it at its own head), right before the REQ, and
// before every later Receive and every Finish. An abort before the REQ leaves
// no burst on the bus; one after it skips the remaining steps (receive and
// bookkeeping only — the frames left queued are dropped by the next run's
// drain). Either way the abort is the same one sequence as above, with the
// same worst case: a period plus one window.

#include "rtc_base/threading/periodic_rt_thread.hpp"
#include "udp_hand_driver/hand_comm.hpp"

#include <array>
#include <atomic>
#include <cstddef>

namespace udp_hand_driver {

class HandCommLoop;

/// What PrepareSharedReqCycle() reports to the loop (#956).
struct SharedReqPrep {
  bool takes_part{false};  ///< this hand waits for the shared REQ's answer
  bool wrote{false};       ///< its command reached the link (the bound grows to 2T)
};

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

  // ── Shared-request tick (#956) — RT path, same rules as RunCommCycle ────

  /// Step 1: the cycle head and this hand's command, no request.
  [[nodiscard]] virtual SharedReqPrep PrepareSharedReqCycle() noexcept = 0;

  /// Step 2: the request every hand answers, on one hand's link (the next
  /// taking-part hand's after a failed send). `t_sent` is set either way;
  /// false when the send failed.
  [[nodiscard]] virtual bool SendSharedReq(HandReadOutcome::Clock::time_point& t_sent) noexcept = 0;

  /// Step 3: this hand's answer to `req` (receive only).
  virtual void ReceiveSharedReq(const SharedRequest& req) noexcept = 0;

  /// Step 4: the rest of this hand's cycle.
  virtual void FinishSharedReqCycle() noexcept = 0;

 protected:
  HandCommLoopMember() = default;
};

class HandCommLoop final : public rtc::PeriodicRtThread {
 public:
  /// Hands one bus can serve per tick (D69: W4 budget, `N × 2T < slot`; one
  /// shared-request window `2T < slot` under #956).
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

  /// Tick in shared-request mode (#956) or one whole cycle per hand (the
  /// default). Refused (false) while running.
  [[nodiscard]] bool SetSharedReq(bool on) noexcept {
    if (Running()) {
      return false;
    }
    shared_req_ = on;
    return true;
  }

  [[nodiscard]] bool shared_req() const noexcept { return shared_req_; }

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
    if (shared_req_) {
      RunSharedReqTick();
      return;
    }
    std::size_t ran = 0;
    for (std::size_t i = 0; i < count_; ++i) {
      // Between hands only: the first hand's own head check covers the tick
      // head, so with one hand this loop adds no load and no behaviour.
      if (i > 0 && EstopLatched()) {
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
  [[nodiscard]] bool EstopLatched() const noexcept {
    return estop_flag_ != nullptr && estop_flag_->load(std::memory_order_acquire);
  }

  // The shared-request tick (header comment): Prepare × N → one REQ →
  // Receive, then Finish, for every hand that takes part, with the latch
  // re-checked in between.
  void RunSharedReqTick() noexcept {
    std::size_t ran = 0;
    std::size_t first = kMaxHands;  // first hand that takes part — sends the REQ
    bool any_wrote = false;
    for (std::size_t i = 0; i < count_; ++i) {
      if (i > 0 && EstopLatched()) {
        AbortForEstop();
        return;
      }
      const SharedReqPrep prep = members_[i]->PrepareSharedReqCycle();
      ++ran;
      if (aborting_) {
        return;  // a hand hit the latch at its head: no REQ, no Finish
      }
      takes_part_[i] = prep.takes_part;
      if (prep.takes_part && first == kMaxHands) {
        first = i;
      }
      any_wrote = any_wrote || prep.wrote;
    }
    if (first < kMaxHands) {
      // The REQ draws every hand's burst: a latch raised while the commands
      // went out stops the tick before it.
      if (EstopLatched()) {
        AbortForEstop();
        return;
      }
      SharedRequest req{};
      for (std::size_t i = first; i < count_ && !req.sent; ++i) {
        if (takes_part_[i]) {
          req.sent = members_[i]->SendSharedReq(req.t_sent);
        }
      }
      req.after_write = any_wrote;
      bool received = false;
      for (std::size_t i = first; i < count_; ++i) {
        if (!takes_part_[i]) {
          continue;
        }
        if (received && EstopLatched()) {
          AbortForEstop();
          return;
        }
        members_[i]->ReceiveSharedReq(req);
        received = true;
      }
      for (std::size_t i = first; i < count_; ++i) {
        if (!takes_part_[i]) {
          continue;
        }
        if (EstopLatched()) {
          AbortForEstop();
          return;
        }
        members_[i]->FinishSharedReqCycle();
      }
    }
    if (ran > 0 && suppress_votes_ >= ran) {
      SuppressTimingThisTick();
    }
  }

  std::array<HandCommLoopMember*, kMaxHands> members_{};
  std::size_t count_{0};
  const std::atomic<bool>* estop_flag_{nullptr};
  bool shared_req_{false};  // set before Start(), read by the loop thread only while running

  // Loop thread only.
  std::size_t suppress_votes_{0};
  bool aborting_{false};
  std::array<bool, kMaxHands> takes_part_{};
};

}  // namespace udp_hand_driver

#endif  // UDP_HAND_DRIVER_HAND_COMM_LOOP_HPP_
