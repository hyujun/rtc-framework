// Unit tests for rtc::PeriodicRtThread:
//   - Start/Stop lifecycle (idempotent Join, no-spawn on bad config).
//   - Pause/Resume halt & resume the tick stream; RequestStop wakes a
//     paused loop without deadlocking.
//   - SetTimingProducer<N> wires per-tick t0..t3 capture into the
//     ThreadTimingProducer ring.
//   - WaitForNextTick override is honoured (kAbort exits the loop and
//     fires OnLoopAborted).
//   - OnOverrun is invoked when the default deadline wait detects lag.
//
// The tests use a low frequency (≤ 100 Hz) and short waits so the suite
// remains fast on a busy CI host. Scheduler determinism isn't required —
// only that the lifecycle / counters / hooks behave as documented.

#include "rtc_base/threading/periodic_rt_thread.hpp"
#include "rtc_base/threading/thread_config.hpp"
#include "rtc_base/timing/rt_tick_timing_sample.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

namespace {

using namespace std::chrono_literals;

// Polls `pred` every millisecond until it holds or `timeout` elapses, returning
// the final predicate value. Replaces blind sleep-as-sync: the suite reacts to
// observable progress (tick / overrun counters) instead of a fixed wall-clock
// guess, which removes timing flakiness on a busy CI host. The generous 2 s
// ceiling only bounds a genuine hang — the happy path returns in a few ms.
template <typename Pred>
[[nodiscard]] bool WaitUntil(Pred pred, std::chrono::milliseconds timeout = 2s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!pred()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(1ms);
  }
  return true;
}

rtc::ThreadConfig MakeNonRtConfig(const char* name) {
  // SCHED_OTHER + nice 0 keeps the test runnable without RT permissions.
  rtc::ThreadConfig cfg{};
  cfg.cpu_core = 0;
  cfg.sched_policy = SCHED_OTHER;
  cfg.sched_priority = 0;
  cfg.nice_value = 0;
  cfg.name = name;
  return cfg;
}

rtc::PeriodicRtThreadConfig MakeCfg(const char* name, double hz) {
  rtc::PeriodicRtThreadConfig cfg{};
  cfg.thread_config = MakeNonRtConfig(name);
  cfg.frequency_hz = hz;
  return cfg;
}

// Minimal subclass that just counts ticks. Sleeps a tiny bit per tick to
// keep timing payloads non-zero in the producer test.
class CountingThread : public rtc::PeriodicRtThread {
 public:
  std::atomic<std::uint64_t> ticks{0};

 protected:
  void OnTick() noexcept override {
    MarkStateAcquired();
    std::this_thread::sleep_for(50us);
    MarkComputeDone();
    ticks.fetch_add(1, std::memory_order_relaxed);
  }
};

}  // namespace

TEST(PeriodicRtThread, StartStopRunsTicks) {
  CountingThread t;
  t.Start(MakeCfg("rtc_test_a", 100.0));
  EXPECT_TRUE(t.Running());
  EXPECT_TRUE(WaitUntil([&] { return t.ticks.load() >= 5; }))
      << "thread did not reach 5 ticks within timeout";
  t.RequestStop();
  t.Join();
  EXPECT_FALSE(t.Running());
  EXPECT_GE(t.ticks.load(), 5u);
  EXPECT_GE(t.TickCount(), 5u);
}

TEST(PeriodicRtThread, StartIsNoopOnNonPositiveFrequency) {
  CountingThread t;
  t.Start(MakeCfg("rtc_test_b", 0.0));
  EXPECT_FALSE(t.Running());
  t.Start(MakeCfg("rtc_test_b", -10.0));
  EXPECT_FALSE(t.Running());
  EXPECT_EQ(t.ticks.load(), 0u);
}

TEST(PeriodicRtThread, JoinIsIdempotent) {
  CountingThread t;
  t.Start(MakeCfg("rtc_test_c", 100.0));
  EXPECT_TRUE(WaitUntil([&] { return t.ticks.load() >= 1; })) << "thread never ticked before Join";
  t.Join();
  // Second join must not block, throw, or restart anything.
  t.Join();
  EXPECT_FALSE(t.Running());
}

TEST(PeriodicRtThread, PauseHaltsTicksAndResumeRestarts) {
  CountingThread t;
  t.Start(MakeCfg("rtc_test_d", 100.0));
  EXPECT_TRUE(WaitUntil([&] { return t.ticks.load() >= 1; })) << "thread never ticked before Pause";

  t.Pause();
  EXPECT_TRUE(t.Paused());
  std::this_thread::sleep_for(40ms);  // let the loop enter the cv wait
  const auto frozen = t.ticks.load();
  std::this_thread::sleep_for(120ms);
  EXPECT_EQ(t.ticks.load(), frozen) << "Paused loop must not advance ticks";

  t.Resume();
  EXPECT_TRUE(WaitUntil([&] { return t.ticks.load() > frozen; }))
      << "Resume did not restart ticking within timeout";
  EXPECT_GT(t.ticks.load(), frozen) << "Resume must restart ticking";

  t.RequestStop();
  t.Join();
}

TEST(PeriodicRtThread, RequestStopWakesPausedLoop) {
  CountingThread t;
  t.Start(MakeCfg("rtc_test_e", 100.0));
  t.Pause();
  std::this_thread::sleep_for(40ms);

  // RequestStop must observe the cv and let Join return promptly.
  const auto t0 = std::chrono::steady_clock::now();
  t.RequestStop();
  t.Join();
  const auto elapsed = std::chrono::steady_clock::now() - t0;
  EXPECT_LT(elapsed, 500ms) << "RequestStop on a paused loop must not hang";
}

namespace {

// Subclass that binds a timing producer and lets the test inspect the
// drained samples.
class TimingThread : public rtc::PeriodicRtThread {
 public:
  rtc::CmTimingBuffer producer;

  TimingThread() { SetTimingProducer<rtc::kCmTimingBufferCapacity>(&producer); }

 protected:
  void OnTick() noexcept override {
    MarkStateAcquired();
    std::this_thread::sleep_for(100us);
    MarkComputeDone();
    std::this_thread::sleep_for(50us);
  }
};

}  // namespace

TEST(PeriodicRtThread, TimingProducerReceivesPerTickSamples) {
  TimingThread t;
  t.Start(MakeCfg("rtc_test_f", 100.0));
  EXPECT_TRUE(WaitUntil([&] { return t.TickCount() >= 6; }))
      << "fewer than 6 ticks produced within timeout";
  t.RequestStop();
  t.Join();

  std::vector<rtc::RtTickTimingSample> samples;
  t.producer.Drain([&](const rtc::RtTickTimingSample& s) { samples.push_back(s); });
  ASSERT_GE(samples.size(), 5u);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    EXPECT_EQ(samples[i].tick_count, i + 1);
    EXPECT_GT(samples[i].t_wall_ns, 0u);
    EXPECT_GE(samples[i].payload.t_total_us, 0.0);
    // Compute / state phases each waited > 0; total >= sum of phases.
    const double summed = samples[i].payload.t_state_us + samples[i].payload.t_compute_us +
                          samples[i].payload.t_publish_us;
    EXPECT_GE(samples[i].payload.t_total_us, summed - 1e-3);
  }
  // First sample has no predecessor → jitter must be exactly zero.
  EXPECT_EQ(samples.front().payload.jitter_us, 0.0);
}

namespace {

class AbortingThread : public rtc::PeriodicRtThread {
 public:
  std::atomic<int> wait_calls{0};
  std::atomic<bool> aborted{false};

 protected:
  WaitResult WaitForNextTick() noexcept override {
    const int n = wait_calls.fetch_add(1) + 1;
    if (n >= 3) {
      return WaitResult::kAbort;
    }
    std::this_thread::sleep_for(5ms);
    return WaitResult::kProceed;
  }

  void OnTick() noexcept override {
    // No-op — just verify base loop dispatching.
  }

  void OnLoopAborted() noexcept override { aborted.store(true); }
};

}  // namespace

TEST(PeriodicRtThread, WaitForNextTickAbortStopsLoop) {
  AbortingThread t;
  t.Start(MakeCfg("rtc_test_g", 200.0));
  // The override returns kAbort on the 3rd call; the loop should exit on
  // its own without RequestStop(), and OnLoopAborted must fire once.
  EXPECT_TRUE(WaitUntil([&] { return t.aborted.load(); }))
      << "loop did not self-abort within timeout";
  t.Join();
  EXPECT_TRUE(t.aborted.load());
  EXPECT_GE(t.wait_calls.load(), 3);
  // 2 ticks ran before the abort.
  EXPECT_EQ(t.TickCount(), 2u);
}

namespace {

class OverrunThread : public rtc::PeriodicRtThread {
 public:
  std::atomic<int> overruns{0};
  std::atomic<std::uint64_t> last_consecutive{0};

 protected:
  void OnTick() noexcept override {
    // Sleep > period to force the next deadline wait to detect lag.
    std::this_thread::sleep_for(15ms);
  }

  void OnOverrun(std::uint64_t consecutive) noexcept override {
    overruns.fetch_add(1);
    last_consecutive.store(consecutive);
  }
};

}  // namespace

namespace {

// Subclass that disables jitter measurement (mirrors CM sim mode where the
// CV wakeup makes |actual_period − budget| meaningless).
class NoJitterTimingThread : public rtc::PeriodicRtThread {
 public:
  rtc::CmTimingBuffer producer;

  NoJitterTimingThread() { SetTimingProducer<rtc::kCmTimingBufferCapacity>(&producer); }

 protected:
  void OnTick() noexcept override {
    MarkStateAcquired();
    std::this_thread::sleep_for(100us);
    MarkComputeDone();
    std::this_thread::sleep_for(50us);
  }

  [[nodiscard]] bool JitterMeaningful() const noexcept override { return false; }
};

}  // namespace

TEST(PeriodicRtThread, JitterZeroWhenSubclassDisablesIt) {
  NoJitterTimingThread thr;
  thr.Start(MakeCfg("rtc_test_nj", 100.0));
  EXPECT_TRUE(WaitUntil([&] { return thr.TickCount() >= 6; }))
      << "fewer than 6 ticks produced within timeout";
  thr.RequestStop();
  thr.Join();

  std::vector<rtc::RtTickTimingSample> samples;
  thr.producer.Drain([&](const rtc::RtTickTimingSample& smp) { samples.push_back(smp); });
  ASSERT_GE(samples.size(), 5U);
  for (const auto& smp : samples) {
    // Phase measurements remain real — only jitter is suppressed.
    EXPECT_GT(smp.payload.t_total_us, 0.0);
    EXPECT_GE(smp.payload.t_state_us, 0.0);
    EXPECT_GE(smp.payload.t_compute_us, 0.0);
    EXPECT_EQ(smp.payload.jitter_us, 0.0)
        << "JitterMeaningful()==false must zero jitter_us on every tick";
  }
}

TEST(PeriodicRtThread, OverrunHookFiresOnDeadlineMiss) {
  OverrunThread t;
  // 200 Hz → 5 ms period; OnTick sleeps 15 ms so every tick overruns.
  t.Start(MakeCfg("rtc_test_h", 200.0));
  EXPECT_TRUE(WaitUntil([&] { return t.overruns.load() >= 3; }))
      << "fewer than 3 overruns observed within timeout";
  t.RequestStop();
  t.Join();
  EXPECT_GE(t.overruns.load(), 3);
  EXPECT_GE(t.OverrunCount(), 3u);
  EXPECT_GT(t.SkipCount(), 0u);
  EXPECT_GE(t.last_consecutive.load(), 1u);
}

namespace {

// Subclass that self-exits from inside OnTick via RequestLoopExit() (the
// pure-atomic RT-safe exit) once `exit_after` ticks have run, and records how
// many times OnLoopAborted fired. Used to verify the terminal-exit path
// (mirrors the hand driver's E-Stop self-exit) and that the thread can be
// restarted afterwards (exit_from_loop_ cleared in Start()).
class ExitingThread : public rtc::PeriodicRtThread {
 public:
  std::atomic<int> ticks{0};
  std::atomic<int> aborts{0};
  std::atomic<int> exit_after{3};

 protected:
  void OnTick() noexcept override {
    const int n = ticks.fetch_add(1) + 1;
    if (n >= exit_after.load()) {
      SuppressTimingThisTick();  // terminal tick does no representative work
      RequestLoopExit();
    }
  }

  void OnLoopAborted() noexcept override { aborts.fetch_add(1); }
};

}  // namespace

TEST(PeriodicRtThread, RequestLoopExitStopsLoopAndFiresAbortOnce) {
  ExitingThread t;
  t.exit_after.store(3);
  t.Start(MakeCfg("rtc_test_le", 200.0));

  // The loop self-exits without RequestStop(); OnLoopAborted fires exactly once.
  EXPECT_TRUE(WaitUntil([&] { return t.aborts.load() >= 1; }))
      << "loop did not self-exit via RequestLoopExit within timeout";
  t.Join();
  EXPECT_EQ(t.aborts.load(), 1) << "OnLoopAborted must fire exactly once on a loop-exit";
  EXPECT_FALSE(t.Running());
  const int first_run_ticks = t.ticks.load();
  EXPECT_GE(first_run_ticks, 3);

  // Restart: Start() must clear exit_from_loop_ so the fresh loop runs again
  // rather than aborting immediately (regression for the hand driver's
  // deactivate→activate after an E-Stop self-exit).
  t.exit_after.store(first_run_ticks + 3);
  t.Start(MakeCfg("rtc_test_le2", 200.0));
  EXPECT_TRUE(t.Running());
  EXPECT_TRUE(WaitUntil([&] { return t.aborts.load() >= 2; }))
      << "restarted loop did not run and self-exit again";
  t.Join();
  EXPECT_EQ(t.aborts.load(), 2);
  EXPECT_GE(t.ticks.load(), first_run_ticks + 3);
}

TEST(PeriodicRtThread, SelfExitClearsRunningAndRestartsWithoutJoin) {
  // Regression (#8): a RequestLoopExit self-exit must clear running_ on the loop
  // thread — otherwise Running() reports a dead thread as alive and a Start()
  // BEFORE Join() silently no-ops on the exchange(true) guard. The prior test
  // Joins before checking, so it never exercised this precondition that the hand
  // driver's new self-exit feature imposes on all PeriodicRtThread users.
  ExitingThread t;
  t.exit_after.store(3);
  t.Start(MakeCfg("rtc_test_le_nojoin", 200.0));

  EXPECT_TRUE(WaitUntil([&] { return t.aborts.load() >= 1; }))
      << "loop did not self-exit via RequestLoopExit within timeout";
  // Running() must go false after the self-exit WITHOUT any Join().
  EXPECT_TRUE(WaitUntil([&] { return !t.Running(); }))
      << "Running() must be false after a self-exit, before any Join()";

  // Restart with NO intervening Join(): the guard must let it through and spawn a
  // fresh live loop (the new jthread move-assign joins the dead one first).
  const int ticks_before = t.ticks.load();
  t.exit_after.store(ticks_before + 1000);  // do not self-exit again immediately
  t.Start(MakeCfg("rtc_test_le_nojoin2", 200.0));
  EXPECT_TRUE(t.Running());
  EXPECT_TRUE(WaitUntil([&] { return t.ticks.load() > ticks_before + 2; }))
      << "Start() before Join() did not spawn a live loop";
  t.Join();
  EXPECT_FALSE(t.Running());
}

namespace {

// Subclass that suppresses the per-tick timing push on even ticks, so exactly
// the odd ticks emit a timing sample. Verifies SuppressTimingThisTick()
// (used by the hand driver for decimated skip / E-Stop cycles).
class SuppressTimingThread : public rtc::PeriodicRtThread {
 public:
  rtc::CmTimingBuffer producer;

  SuppressTimingThread() { SetTimingProducer<rtc::kCmTimingBufferCapacity>(&producer); }

 protected:
  void OnTick() noexcept override {
    MarkStateAcquired();
    const int n = ticks_.fetch_add(1) + 1;
    if (n % 2 == 0) {
      SuppressTimingThisTick();
    }
    MarkComputeDone();
  }

 private:
  std::atomic<int> ticks_{0};
};

}  // namespace

TEST(PeriodicRtThread, SuppressTimingThisTickSkipsPush) {
  SuppressTimingThread t;
  t.Start(MakeCfg("rtc_test_st", 100.0));
  EXPECT_TRUE(WaitUntil([&] { return t.TickCount() >= 8; }))
      << "fewer than 8 ticks produced within timeout";
  t.RequestStop();
  t.Join();

  const std::uint64_t final_ticks = t.TickCount();
  std::vector<rtc::RtTickTimingSample> samples;
  t.producer.Drain([&](const rtc::RtTickTimingSample& s) { samples.push_back(s); });

  // Only odd-numbered ticks push; count of odd n in [1, final_ticks] = ceil/2.
  EXPECT_EQ(samples.size(), (final_ticks + 1) / 2)
      << "suppressed (even) ticks must not emit a timing sample";
  EXPECT_LT(samples.size(), final_ticks) << "some ticks must have been suppressed";
}
