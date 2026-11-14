#include "rtc_mpc/thread/mpc_thread.hpp"

#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_base/tracing/trace_scope.hpp"

#include <omp.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>  // gettid

#include <cstddef>
#include <cstdio>
#include <cstdlib>

namespace rtc::mpc {

namespace {

/// @brief Touch a heap block and a stack page on the calling thread, so its
///        glibc arena and its first stack pages exist before the first solve.
void TouchThreadMemory() noexcept {
  void* block = std::malloc(256);
  // The barrier keeps the compiler from folding the malloc/free pair away.
  asm volatile("" : : "r"(block) : "memory");
  std::free(block);
  volatile char page[4096];
  page[0] = 0;
  page[sizeof(page) - 1] = 0;
}

/// @brief Whether the calling thread may run on exactly one CPU, `cpu`.
[[nodiscard]] bool AffinityIsExactly(int cpu) noexcept {
  cpu_set_t set;
  CPU_ZERO(&set);
  if (cpu < 0 || pthread_getaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
    return false;
  }
  return CPU_COUNT(&set) == 1 && CPU_ISSET(static_cast<std::size_t>(cpu), &set);
}

[[nodiscard]] int CurrentPolicy() noexcept {
  int policy = -1;
  sched_param param{};
  if (pthread_getschedparam(pthread_self(), &policy, &param) != 0) {
    return -1;
  }
  return policy;
}

}  // namespace

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
  pool_state_.store(SolverPoolState::kPending, std::memory_order_release);
  // Wire the SPSC ring before the base spawns its loop thread so the very
  // first tick already pushes a sample.
  SetTimingProducer<rtc::kMpcTimingBufferCapacity>(&timing_producer_);

  rtc::PeriodicRtThreadConfig pcfg{};
  pcfg.thread_config = launch_config_.main;
  pcfg.frequency_hz = launch_config_.target_frequency_hz;
  PeriodicRtThread::Start(pcfg);
}

void MPCThread::ApplySolverThreadIcvs() noexcept {
  // Default team size 1 whatever solver_threads is: the pool belongs to the
  // regions that name their team size (the loop entry's, the solver's). A
  // region sized by default — Eigen's own products — then stays serial, and
  // libgomp never retires a pinned worker for a smaller team.
  omp_set_dynamic(0);
  omp_set_num_threads(1);
}

void MPCThread::OnLoopEntry(bool thread_config_applied) noexcept {
  ApplySolverThreadIcvs();
  if (!SolverThreadsMatch()) {
    std::fprintf(stderr,
                 "[ERROR] MPC solver pool refused: the solver's thread count is not the "
                 "launch config's solver_threads (%d)\n",
                 launch_config_.solver_threads);
    pool_state_.store(SolverPoolState::kFailed, std::memory_order_release);
    RequestLoopExit();
    return;
  }
  if (launch_config_.solver_threads <= 1) {
    // One thread: nothing to place. A failed thread config is reported by
    // ApplyThreadConfigVerbose and tolerated here exactly as before.
    pool_state_.store(SolverPoolState::kReady, std::memory_order_release);
    return;
  }
  if (SetUpPool(thread_config_applied)) {
    pool_state_.store(SolverPoolState::kReady, std::memory_order_release);
    return;
  }
  pool_state_.store(SolverPoolState::kFailed, std::memory_order_release);
  RequestLoopExit();
}

bool MPCThread::SetUpPool(bool thread_config_applied) noexcept {
  const int n = launch_config_.solver_threads;
  const int workers = n - 1;
  const char* reason = nullptr;
  if (!thread_config_applied) {
    // The workers inherit this thread's placement and policy; a thread that
    // did not get its own cannot hand the pool the right ones.
    reason = "the MPC thread's own config did not apply";
  } else if (workers > launch_config_.workers.count || workers > rtc::kMaxMpcWorkers) {
    reason = "more workers than the layout profile reserves slots for";
  } else if (n > omp_get_thread_limit() || omp_in_parallel() != 0) {
    reason = "the OpenMP runtime cannot give this thread a team of that size";
  } else if (omp_get_proc_bind() != omp_proc_bind_false) {
    // OMP_PROC_BIND / OMP_PLACES would have libgomp place the team itself,
    // over the slots this thread pins it to.
    reason = "OpenMP thread binding is enabled (unset OMP_PROC_BIND / OMP_PLACES)";
  }

  int team = 0;
  if (reason == nullptr) {
    for (auto& r : worker_reports_) {
      r.apply_rc.store(-1, std::memory_order_relaxed);
      r.on_slot.store(0, std::memory_order_relaxed);
      r.policy.store(-1, std::memory_order_relaxed);
      r.tid.store(0, std::memory_order_relaxed);
    }
    const rtc::ThreadConfig main = launch_config_.main;
    const rtc::MpcWorkerSlots slots = launch_config_.workers;
    // The workers are created on this thread's CPU with its policy. Under
    // SCHED_FIFO a master that spins at the region's barrier (libgomp spins
    // before it sleeps; OMP_WAIT_POLICY / GOMP_SPINCOUNT set how long) would
    // keep same-priority workers on its CPU from ever running to pin
    // themselves away. The region therefore runs with this thread on
    // SCHED_OTHER — the workers start fair-scheduled, move to their own CPUs
    // and take main's policy there (ApplyThreadConfig) — and this thread
    // takes its own back afterwards.
    int own_policy = SCHED_OTHER;
    sched_param own_param{};
    const bool lowered = pthread_getschedparam(pthread_self(), &own_policy, &own_param) == 0 &&
                         own_policy != SCHED_OTHER && [] {
                           sched_param other{};
                           return pthread_setschedparam(pthread_self(), SCHED_OTHER, &other) == 0;
                         }();
#pragma omp parallel num_threads(n)
    {
      const int id = omp_get_thread_num();
      if (id == 0) {
        team = omp_get_num_threads();
      } else if (id - 1 < rtc::kMaxMpcWorkers) {
        WorkerReport& r = worker_reports_[static_cast<std::size_t>(id - 1)];
        const rtc::ThreadConfig cfg = rtc::MpcWorkerThreadConfig(main, slots, id - 1);
        // ApplyThreadConfig names the thread before it pins it, so a pin that
        // fails still leaves a worker the verifier finds by its own name, not
        // by the mpc_main it inherited. An index past the pool has no name and
        // is refused by the config check.
        const int rc = rtc::ApplyThreadConfig(cfg) ? 0 : 1;
        TouchThreadMemory();
        r.on_slot.store(AffinityIsExactly(rtc::SlotToLogicalCpu(cfg.cpu_core)) ? 1 : 0,
                        std::memory_order_relaxed);
        r.policy.store(CurrentPolicy(), std::memory_order_relaxed);
        r.tid.store(gettid(), std::memory_order_relaxed);
        r.apply_rc.store(rc, std::memory_order_relaxed);
      }
    }

    if (lowered && pthread_setschedparam(pthread_self(), own_policy, &own_param) != 0) {
      reason = "the MPC thread could not take its scheduling policy back after the pool set-up";
    }
    const pid_t self = gettid();
    const int main_policy = CurrentPolicy();
    if (reason != nullptr) {
      // already set: the policy restore failed
    } else if (team != n) {
      reason = "the parallel region ran with a different team size";
    } else if (omp_get_dynamic() != 0 || omp_get_max_threads() != 1) {
      reason = "this thread's OpenMP ICVs changed during the pool set-up";
    }
    for (int i = 0; reason == nullptr && i < workers; ++i) {
      const WorkerReport& r = worker_reports_[static_cast<std::size_t>(i)];
      const pid_t tid = r.tid.load(std::memory_order_relaxed);
      if (r.apply_rc.load(std::memory_order_relaxed) != 0) {
        reason = "a worker's thread config did not apply";
      } else if (r.on_slot.load(std::memory_order_relaxed) != 1) {
        reason = "a worker is not on exactly its slot's CPU";
      } else if (r.policy.load(std::memory_order_relaxed) != main_policy) {
        reason = "a worker's scheduling policy differs from the MPC thread's";
      } else if (tid == 0 || tid == self) {
        reason = "a worker ran on the MPC thread itself";
      }
      for (int j = 0; reason == nullptr && j < i; ++j) {
        if (worker_reports_[static_cast<std::size_t>(j)].tid.load(std::memory_order_relaxed) ==
            tid) {
          reason = "two workers share a thread";
        }
      }
    }
  }

  if (reason != nullptr) {
    // One-shot, before the first tick: the loop is about to end.
    std::fprintf(stderr,
                 "[ERROR] MPC solver pool refused (%d threads, %d worker slots, team %d): %s\n", n,
                 launch_config_.workers.count, team, reason);
    return false;
  }
  return true;
}

void MPCThread::OnOverrun(std::uint64_t /*consecutive*/) noexcept {
  (standing_tick_ ? standing_overruns_ : online_overruns_).fetch_add(1, std::memory_order_relaxed);
}

void MPCThread::OnTick() noexcept {
  RTC_TRACE_SCOPE("mpc_tick");
  standing_tick_ = false;

  // Phase 1: state acquisition.
  MPCStateSnapshot state;
  {
    RTC_TRACE_SCOPE("mpc_read_state");
    state = manager_->ReadState();
    MarkStateAcquired();
  }

  // Phase 2: solve — only once the RT side has stored a state at all.
  bool ok = false;
  {
    RTC_TRACE_SCOPE("mpc_solve");
    if (state.valid) {
      ok = Solve(state, scratch_);
    }
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
