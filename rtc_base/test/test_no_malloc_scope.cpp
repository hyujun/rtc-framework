// ── Self-test for the zero-allocation gate (rtc_base/testing) ────────────────
// Proves the ScopedNoMalloc guard actually detects Eigen heap allocation, and
// that it does so under this repo's Release/-DNDEBUG build (where the stock
// eigen_assert-based check would be a no-op). The gate MUST be included before
// any Eigen header — this file includes nothing Eigen-related beforehand.
#include "rtc_base/testing/no_malloc_scope.hpp"  // must precede <Eigen/*>

#include <gtest/gtest.h>

#include <Eigen/Dense>

namespace {

using rtc::testing::ScopedNoMalloc;

// A fixed-size Eigen computation performs no heap allocation → zero violations.
TEST(NoMallocScope, FixedSizeAllocatesNothing) {
  Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
  Eigen::Vector3d v(1.0, 2.0, 3.0);
  Eigen::Vector3d out;
  {
    ScopedNoMalloc guard;
    out.noalias() = m * v;  // fixed-size, stack only
    EXPECT_EQ(guard.violations(), 0u);
  }
  EXPECT_EQ(out, v);
}

// A dynamic-size allocation inside the scope is caught even in Release
// (NDEBUG), because detection does not ride the default eigen_assert.
TEST(NoMallocScope, DynamicAllocationIsCaught) {
  ScopedNoMalloc guard;
  Eigen::VectorXd dyn(128);  // heap allocation via Eigen's allocator
  dyn.setZero();
  EXPECT_GT(guard.violations(), 0u) << "dynamic Eigen allocation went undetected";
}

// The counter resets per scope, and leaving a scope re-allows allocation so a
// later dynamic allocation outside any scope does not register.
TEST(NoMallocScope, ScopeResetsAndRestores) {
  {
    ScopedNoMalloc guard;
    Eigen::VectorXd dyn(64);
    dyn.setOnes();
    EXPECT_GT(guard.violations(), 0u);
  }
  // Fresh scope starts clean (counter reset in the constructor).
  {
    ScopedNoMalloc guard;
    Eigen::Matrix4d m = Eigen::Matrix4d::Zero();
    m(0, 0) = 1.0;
    EXPECT_EQ(guard.violations(), 0u);
  }
  // Outside any scope, malloc is allowed again — this must not abort.
  Eigen::VectorXd after(32);
  after.setZero();
  SUCCEED();
}

}  // namespace
