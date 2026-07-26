// ── Bitwise comparison primitives for extraction golden-vector tests ────────
// A code-motion refactor that claims to be *inert* must be pinned bit-for-bit,
// not by tolerance: the classic silent regression is an algebraically identical
// reassociation (`α·(kp·e + kd·ė)` → `α·kp·e + α·kd·ė`) that sails through any
// EXPECT_NEAR while changing the torque a shipped robot emits. So the
// extraction suites compare `std::bit_cast<uint64_t>` and every such suite
// carries a NON-VACUITY partner test proving the comparison can actually reject
// a reassociated form — see ImpedanceLaw.BitwiseComparisonRejectsAReassociatedLaw
// (test_compliance_core.cpp) for the reference pair.
//
// Test-only, NEVER installed: this header lives under test/include and is
// consumed by source-tree path (`target_include_directories(... PRIVATE
// ${CMAKE_CURRENT_SOURCE_DIR}/test/include)`), because ament's symlink install
// ignores install(PATTERN EXCLUDE) and a header under include/ would ship.
#pragma once

#include <bit>
#include <cstdint>
#include <random>

namespace rtc::testing {

/// True iff `a` and `b` are the SAME double down to the bit pattern. Distinct
/// from `==` on NaN (bit-equal NaNs compare true here, as they must for a
/// "same computation" claim) and from -0.0 vs +0.0 (bit-distinct, and a sign
/// flip on a zero command IS an observable difference in a torque lane).
[[nodiscard]] inline bool BitsEqual(double a, double b) noexcept {
  return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

/// Fixed-seed generator — golden-vector suites must be reproducible, so every
/// draw sequence is deterministic across runs and machines.
[[nodiscard]] inline std::mt19937 MakeRng() {
  return std::mt19937(0xC0FFEEu);
}

}  // namespace rtc::testing
