// Unit tests for MPCSolution / MPCStateSnapshot.
//
// Goals:
//   * Confirm the RT-safety contracts (trivially copyable).
//   * Sanity-check `IsValid()` edge cases.
//   * Lock down sizeof bounds so future capacity bumps are deliberate.

#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <type_traits>

namespace rtc::mpc {
namespace {

TEST(MpcSolutionTypes, TriviallyCopyable) {
  static_assert(std::is_trivially_copyable_v<MPCSolution>);
  static_assert(std::is_trivially_copyable_v<MPCStateSnapshot>);
  SUCCEED();
}

TEST(MpcSolutionTypes, DefaultConstructedIsZeroFilled) {
  MPCSolution sol{};
  EXPECT_EQ(sol.timestamp_ns, 0u);
  EXPECT_EQ(sol.horizon_length, 0);
  EXPECT_EQ(sol.nv, 0);
  EXPECT_DOUBLE_EQ(sol.dt_node, 0.0);
  EXPECT_FALSE(sol.converged);

  // Spot-check a few trajectory entries: must be zero-initialized.
  EXPECT_DOUBLE_EQ(sol.q_traj[0][0], 0.0);
  EXPECT_DOUBLE_EQ(sol.v_traj[kMaxHorizon][kMaxNv - 1], 0.0);
  EXPECT_DOUBLE_EQ(sol.K_riccati[0][kMaxNu * kMaxNx - 1], 0.0);
}

TEST(MpcSolutionTypes, IsValidRejectsEmptyAndZeroDt) {
  MPCSolution sol{};
  EXPECT_FALSE(sol.IsValid()) << "Default-constructed must be invalid";

  sol.horizon_length = 10;
  sol.nv = 6;
  sol.dt_node = 0.0;
  EXPECT_FALSE(sol.IsValid()) << "dt_node == 0 must be rejected";

  sol.dt_node = 0.01;
  EXPECT_TRUE(sol.IsValid());

  sol.nv = 0;
  EXPECT_FALSE(sol.IsValid()) << "nv == 0 must be rejected";
}

TEST(MpcSolutionTypes, IsValidIgnoresConvergedFlag) {
  // A non-converged warm-start is still preferable to a stale solution;
  // IsValid() intentionally decouples from the convergence flag.
  MPCSolution sol{};
  sol.horizon_length = 5;
  sol.nv = 6;
  sol.dt_node = 0.01;
  sol.converged = false;
  EXPECT_TRUE(sol.IsValid());
}

TEST(MpcSolutionTypes, SizeofFitsExpectedBudget) {
  // Memory budget check: triple-buffer (×3) should stay under ~5 MB.
  // If this test fails after a capacity bump, make the change deliberate.
  constexpr std::size_t kMaxSinglePayloadBytes = 2 * 1024 * 1024;  // 2 MiB
  EXPECT_LE(sizeof(MPCSolution), kMaxSinglePayloadBytes)
      << "MPCSolution grew past 2 MiB — revisit capacity constants.";
  EXPECT_LE(sizeof(MPCStateSnapshot), std::size_t{4096});
}

TEST(MpcStateSnapshot, DefaultConstructedIsZeroFilled) {
  MPCStateSnapshot snap{};
  EXPECT_EQ(snap.timestamp_ns, 0u);
  EXPECT_EQ(snap.nq, 0);
  EXPECT_EQ(snap.nv, 0);
  EXPECT_DOUBLE_EQ(snap.q[0], 0.0);
  EXPECT_DOUBLE_EQ(snap.v[kMaxNv - 1], 0.0);
}

TEST(MpcStateSnapshot, MemcpyRoundTrip) {
  // SeqLock::Store/Load uses memcpy internally — exercise the same path.
  MPCStateSnapshot src{};
  src.timestamp_ns = 42;
  src.nq = 6;
  src.nv = 6;
  for (int i = 0; i < 6; ++i) {
    src.q[static_cast<std::size_t>(i)] = 0.1 * i;
    src.v[static_cast<std::size_t>(i)] = -0.2 * i;
  }

  MPCStateSnapshot dst{};
  std::memcpy(&dst, &src, sizeof(MPCStateSnapshot));

  EXPECT_EQ(dst.timestamp_ns, 42u);
  EXPECT_EQ(dst.nq, 6);
  EXPECT_EQ(dst.nv, 6);
  for (int i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(dst.q[static_cast<std::size_t>(i)], 0.1 * i);
    EXPECT_DOUBLE_EQ(dst.v[static_cast<std::size_t>(i)], -0.2 * i);
  }
}

}  // namespace
}  // namespace rtc::mpc
