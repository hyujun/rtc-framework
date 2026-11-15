#ifndef INTEGRATED_BRINGUP_SUPPORT_MPC_POOL_WAIT_HPP_
#define INTEGRATED_BRINGUP_SUPPORT_MPC_POOL_WAIT_HPP_

// ── The MPC thread's verdict on its solver pool, awaited at activation ───────
//
// Shared by every controller that owns an MPC thread (P5). Called from
// on_activate (aux thread, never the RT tick): the pool set-up is one parallel
// region before the thread's first tick, so it settles within milliseconds;
// the ceiling only bounds a thread that never reaches its loop entry, which is
// a refusal too.

#include "rtc_mpc/thread/mpc_thread.hpp"

#include <chrono>
#include <thread>

namespace integrated_bringup {

[[nodiscard]] inline bool AwaitMpcPool(const rtc::mpc::MPCThread& thread) noexcept {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    switch (thread.PoolState()) {
      case rtc::mpc::SolverPoolState::kReady:
        return true;
      case rtc::mpc::SolverPoolState::kFailed:
        return false;
      case rtc::mpc::SolverPoolState::kPending:
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_SUPPORT_MPC_POOL_WAIT_HPP_
