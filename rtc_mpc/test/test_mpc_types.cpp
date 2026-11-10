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
  EXPECT_DOUBLE_EQ(sol.K_riccati[0][kMaxNu * kMaxNdx - 1], 0.0);
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

TEST(MpcSolutionTypes, CapacityConstantsAreDerivedFromEachOther) {
  // The relations the rest of the package relies on: a free-flyer adds one
  // configuration coordinate, the state tangent is twice the velocity dim,
  // the kinodynamic input fits, and the gain block is sized by the tangent.
  static_assert(kMaxNq == kMaxNv + 1);
  static_assert(kMaxNdx == 2 * kMaxNv);
  static_assert(kMaxNu >= kMaxNv - 6 + kMaxContactVars);
  static_assert(MPCSolution::kRiccatiGainFlatSize ==
                static_cast<std::size_t>(kMaxNu) * static_cast<std::size_t>(kMaxNdx));
  SUCCEED();
}

TEST(MpcSolutionTypes, MarkSolvedKeepsConvergedAndStatusTogether) {
  MPCSolution sol{};
  MarkSolved(sol, true);
  EXPECT_TRUE(sol.converged);
  EXPECT_EQ(sol.status, SolverStatus::kOk);
  MarkSolved(sol, false);
  EXPECT_FALSE(sol.converged);
  EXPECT_EQ(sol.status, SolverStatus::kMaxIters);
}

TEST(MpcSolutionTypes, IsValidRejectsFailureStatusesKeepsMaxIters) {
  MPCSolution sol{};
  sol.horizon_length = 5;
  sol.nv = 6;
  sol.dt_node = 0.01;

  sol.status = SolverStatus::kOk;
  EXPECT_TRUE(sol.IsValid());
  sol.status = SolverStatus::kMaxIters;
  EXPECT_TRUE(sol.IsValid()) << "budget-exhausted but finite is still usable";
  sol.status = SolverStatus::kDiverged;
  EXPECT_FALSE(sol.IsValid());
  sol.status = SolverStatus::kException;
  EXPECT_FALSE(sol.IsValid());
  sol.status = SolverStatus::kTruncated;
  EXPECT_FALSE(sol.IsValid());
  sol.status = SolverStatus::kNotSolved;
  EXPECT_FALSE(sol.IsValid());
}

TEST(MpcSolutionTypes, ProvenanceSurvivesMemcpy) {
  MPCSolution src{};
  src.observation_stamp_ns = 11;
  src.observation_sequence = 12;
  src.plan_id = 14;
  src.model_epoch = 15;
  src.status = SolverStatus::kMaxIters;
  src.residuals.prim_inf = 1e-3;
  src.residuals.dual_inf = 2e-3;
  src.residuals.traj_cost = 3.0;
  src.residuals.merit = 4.0;
  src.ndx = 12;
  src.gain_nodes = kMaxGainNodes;
  src.K_riccati[kMaxGainNodes - 1][0] = 0.5;

  MPCSolution dst{};
  std::memcpy(&dst, &src, sizeof(MPCSolution));
  EXPECT_EQ(dst.observation_stamp_ns, 11u);
  EXPECT_EQ(dst.observation_sequence, 12u);
  EXPECT_EQ(dst.plan_id, 14u);
  EXPECT_EQ(dst.model_epoch, 15u);
  EXPECT_EQ(dst.status, SolverStatus::kMaxIters);
  EXPECT_DOUBLE_EQ(dst.residuals.prim_inf, 1e-3);
  EXPECT_DOUBLE_EQ(dst.residuals.dual_inf, 2e-3);
  EXPECT_DOUBLE_EQ(dst.residuals.traj_cost, 3.0);
  EXPECT_DOUBLE_EQ(dst.residuals.merit, 4.0);
  EXPECT_EQ(dst.ndx, 12);
  EXPECT_EQ(dst.gain_nodes, kMaxGainNodes);
  EXPECT_DOUBLE_EQ(dst.K_riccati[kMaxGainNodes - 1][0], 0.5);
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

TEST(MpcStateSnapshot, ProvenanceFieldsDefaultInvalidAndSurviveMemcpy) {
  MPCStateSnapshot src{};
  EXPECT_FALSE(src.valid) << "a never-stored snapshot must read as invalid";
  EXPECT_EQ(src.sequence, 0u);
  src.sequence = 7;
  src.model_epoch = 3;
  src.contact_mask = 0b101u;
  src.valid = true;

  MPCStateSnapshot dst{};
  std::memcpy(&dst, &src, sizeof(MPCStateSnapshot));
  EXPECT_EQ(dst.sequence, 7u);
  EXPECT_EQ(dst.model_epoch, 3u);
  EXPECT_EQ(dst.contact_mask, 0b101u);
  EXPECT_TRUE(dst.valid);
}

TEST(MpcStateSnapshot, MemcpyRoundTrip) {
  // SeqLock::Store/Load copy the bytes of T — exercise a byte-copy round trip.
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
