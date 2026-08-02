// Shared test-only helper: poll a predicate until it holds (or a deadline
// elapses) instead of sleeping a fixed wall-clock guess. Lets a suite react to
// observable progress (tick / solve / recv counters) so it stays deterministic
// on a loaded CI host. Function-template ⇒ implicitly inline ⇒ ODR-safe when
// several test binaries include it.
//
// Never installed: it lives under test/include, not include/, because ament's
// symlink install ignores install(PATTERN EXCLUDE) — a test header placed under
// include/ would ship in the runtime tree. Sibling packages consume it by
// source-tree include path (../rtc_base/test/include), the same layout
// testing/no_malloc_scope.hpp uses (see rtc_base/CMakeLists.txt).
//
// ⚠ This helper ONLY sleeps — see PROC-8 in agent_docs/invariants.md, which
// owns the rule, both failure modes it prevents, and the history (#356). It
// must never learn to drive an executor, and callers must not wrap it into one;
// a test that genuinely needs a pump keeps its own local SpinUntil, where the
// choice of what to spin stays visible next to the assertion. Enforced, not
// merely documented: verify-changes.sh Phase 0b greps this directory.
//
// The rule's rationale deliberately does NOT name the two leaf test files that
// motivated it. rtc_base sits at the bottom of the dependency graph and has no
// build- or test-time link to those leaves, so a rename there would rot this
// comment silently — and one such pointer had already gone stale, aiming at a
// symbol that no longer exists. PROC-8 and the issue stay reachable; paths do not.
#ifndef RTC_BASE_TESTING_WAIT_UNTIL_HPP_
#define RTC_BASE_TESTING_WAIT_UNTIL_HPP_

#include <chrono>
#include <thread>

namespace rtc::testing {

/// Polls `pred` every `poll` interval until it returns true or `timeout`
/// elapses, returning the final predicate value (false ⇒ timed out). The
/// predicate is evaluated before the deadline is consulted, so an
/// already-satisfied condition reports success even if the budget is spent.
/// The generous default timeout only bounds a genuine hang — the happy path
/// returns within a few poll intervals.
///
/// `pred` binds by forwarding reference and is NOT copied: the helper polls the
/// caller's own predicate, so mutations a stateful predicate makes (a call
/// counter, a captured latch, a `mutable` lambda) survive the wait and are
/// visible to the caller afterwards. Taking it by value would poll a copy and
/// silently discard those — a footgun to bake into a helper four packages
/// share. Not `const Pred&` either: that cannot invoke a `mutable` lambda.
/// `std::forward` is deliberately absent — the predicate is invoked repeatedly,
/// so it must stay an lvalue for the whole loop.
template <typename Pred>
[[nodiscard]] bool WaitUntil(Pred&& pred,
                             std::chrono::milliseconds timeout = std::chrono::seconds(2),
                             std::chrono::milliseconds poll = std::chrono::milliseconds(1)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!pred()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(poll);
  }
  return true;
}

}  // namespace rtc::testing

#endif  // RTC_BASE_TESTING_WAIT_UNTIL_HPP_
