// Shared MPC-thread test helper: poll a predicate until it holds (or a deadline
// elapses) instead of sleeping a fixed wall-clock guess. Lets the lifecycle and
// integration tests react to observable progress (solve_count / phase id) so
// they stay deterministic on a loaded CI host. Function-template ⇒ implicitly
// inline ⇒ ODR-safe when both test binaries include it.
#ifndef RTC_MPC_TEST_WAIT_UNTIL_HPP_
#define RTC_MPC_TEST_WAIT_UNTIL_HPP_

#include <chrono>
#include <thread>

namespace rtc::test {

/// Polls `pred` every millisecond until it returns true or `timeout` elapses,
/// returning the final predicate value (false ⇒ timed out). The generous
/// default only bounds a genuine hang — the happy path returns within a few ms.
template <typename Pred>
[[nodiscard]] bool WaitUntil(Pred pred,
                             std::chrono::milliseconds timeout = std::chrono::seconds(2)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!pred()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

}  // namespace rtc::test

#endif  // RTC_MPC_TEST_WAIT_UNTIL_HPP_
