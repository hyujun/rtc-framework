#pragma once

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace rtc::tsid::test {

// Every row of an assembled inequality box must be non-empty (l_i <= u_i): a
// single inverted row makes the whole QP primal-infeasible. The constraint
// tests before #551 asserted shapes and tightness and never looked at this —
// one of them was asserting an inverted box.
inline void ExpectBoxNonEmpty(const Eigen::Ref<const Eigen::VectorXd>& l,
                              const Eigen::Ref<const Eigen::VectorXd>& u) {
  ASSERT_EQ(l.size(), u.size());
  for (Eigen::Index i = 0; i < l.size(); ++i) {
    EXPECT_LE(l(i), u(i)) << "row " << i << " is an empty box: l=" << l(i) << " > u=" << u(i);
  }
}

}  // namespace rtc::tsid::test
