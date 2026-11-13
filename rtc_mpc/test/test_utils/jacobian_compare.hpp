#pragma once

/// @file jacobian_compare.hpp
/// @brief Element-wise comparison of an analytic Jacobian against a finite
///        difference one, for the derivative gates. Not installed — test-local
///        by design.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <Eigen/Core>
#pragma GCC diagnostic pop

#include <algorithm>

namespace rtc::mpc::test_utils {

/// Worst element-wise `|J − J_fd| / (abs + rel·|J_fd|)` over a column range
/// (≤ 1 passes), and the worst absolute error there.
struct ColumnErr {
  double ratio = 0.0;
  double abs = 0.0;

  void Fold(const ColumnErr& o) {
    ratio = std::max(ratio, o.ratio);
    abs = std::max(abs, o.abs);
  }
};

inline ColumnErr CompareColumns(const Eigen::MatrixXd& j, const Eigen::MatrixXd& j_fd, int col0,
                                int ncols, double abs_tol, double rel_tol) {
  ColumnErr e;
  if (ncols <= 0 || j.rows() == 0) {
    return e;
  }
  const Eigen::MatrixXd d = (j.middleCols(col0, ncols) - j_fd.middleCols(col0, ncols)).cwiseAbs();
  const Eigen::ArrayXXd bound = abs_tol + rel_tol * j_fd.middleCols(col0, ncols).cwiseAbs().array();
  e.ratio = (d.array() / bound).maxCoeff();
  e.abs = d.maxCoeff();
  return e;
}

}  // namespace rtc::mpc::test_utils
