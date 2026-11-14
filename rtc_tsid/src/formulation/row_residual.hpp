#pragma once

#include "rtc_tsid/types/qp_types.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rtc::tsid {

// max |A_r · x − b_r| over the equality rows [row, row + dim) of `qp`, at the
// iterate `x` (sized to A's columns). 0 for no rows; NaN when x (or the rows)
// are not finite. No allocation.
[[nodiscard]] inline double EqualityRowResidualInf(const QPData& qp, int row, int dim,
                                                   const Eigen::VectorXd& x) noexcept {
  if (dim <= 0) {
    return 0.0;
  }
  if (x.size() != qp.A.cols() || row < 0 || row + dim > qp.A.rows()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double out = 0.0;
  for (int r = row; r < row + dim; ++r) {
    const double e = std::abs(qp.A.row(r).dot(x) - qp.b(r));
    if (!std::isfinite(e)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    out = std::max(out, e);
  }
  return out;
}

}  // namespace rtc::tsid
