// Unit tests for TrajectoryInterpolator (Step 4: linear kernel).
//
// Validates:
//   * HasSolution() is false before SetSolution().
//   * Linear trajectories are reproduced to machine precision.
//   * Horizon boundary behaviour: beyond_horizon = true, a_ff = 0.
//   * Clamping of negative time deltas.
//   * RemainingHorizonSec() monotonicity.
//
// Step 5 will add the Hermite-specific accuracy tests.

#include "rtc_mpc/interpolation/trajectory_interpolator.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

namespace rtc::mpc {
namespace {

constexpr int kNq = 3;
constexpr int kNv = 3;
constexpr int kNc = 0;
constexpr int kHorizon = 10;
constexpr double kDtNode = 0.01;  // 10 ms per OCP node
constexpr uint64_t kReceiveNs = 1'000'000'000;  // arbitrary t=1 s origin
constexpr double kTol = 1e-12;

/// Build a trivially valid solution whose q(t) = slope * t at each node.
/// Velocity is set to `slope` (constant), lambda is zeroed.
MPCSolution MakeLinearSolution(double slope) {
  MPCSolution sol{};
  sol.horizon_length = kHorizon;
  sol.dt_node = kDtNode;
  sol.converged = true;
  sol.nq = kNq;
  sol.nv = kNv;
  sol.nu = kNv;
  sol.nx = kNq + kNv;
  sol.n_contact_vars = kNc;
  sol.timestamp_ns = kReceiveNs;

  for (int k = 0; k <= kHorizon; ++k) {
    const double t = static_cast<double>(k) * kDtNode;
    for (int i = 0; i < kNq; ++i) {
      sol.q_traj[static_cast<std::size_t>(k)]
                [static_cast<std::size_t>(i)] = slope * t + 0.1 * i;
    }
    for (int i = 0; i < kNv; ++i) {
      sol.v_traj[static_cast<std::size_t>(k)]
                [static_cast<std::size_t>(i)] = slope;
    }
  }
  return sol;
}

class TrajectoryInterpolatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    interp_.Init(kNq, kNv, kNc);
    q_ref_.setZero(kNq);
    v_ref_.setZero(kNv);
    a_ff_.setZero(kNv);
    lambda_ref_.setZero(std::max(1, kNc));  // Eigen rejects size-0 vectors
  }

  TrajectoryInterpolator interp_;
  Eigen::VectorXd q_ref_;
  Eigen::VectorXd v_ref_;
  Eigen::VectorXd a_ff_;
  Eigen::VectorXd lambda_ref_;
  InterpMeta meta_{};
};

TEST_F(TrajectoryInterpolatorTest, NoSolutionBeforeSet) {
  EXPECT_FALSE(interp_.HasSolution());
  interp_.Interpolate(kReceiveNs, q_ref_, v_ref_, a_ff_, lambda_ref_, meta_);
  EXPECT_FALSE(meta_.valid);
  EXPECT_FALSE(meta_.beyond_horizon);
}

TEST_F(TrajectoryInterpolatorTest, InvalidSolutionRejected) {
  MPCSolution sol{};  // horizon_length = 0 → IsValid() = false
  interp_.SetSolution(sol, kReceiveNs);
  EXPECT_FALSE(interp_.HasSolution());
}

TEST_F(TrajectoryInterpolatorTest, NodeZeroReproducesExactValues) {
  const double slope = 2.0;
  interp_.SetSolution(MakeLinearSolution(slope), kReceiveNs);

  interp_.Interpolate(kReceiveNs, q_ref_, v_ref_, a_ff_, lambda_ref_, meta_);
  ASSERT_TRUE(meta_.valid);
  EXPECT_FALSE(meta_.beyond_horizon);
  EXPECT_NEAR(meta_.progress, 0.0, kTol);

  for (int i = 0; i < kNq; ++i) {
    EXPECT_NEAR(q_ref_(i), 0.1 * i, kTol);
  }
  for (int i = 0; i < kNv; ++i) {
    EXPECT_NEAR(v_ref_(i), slope, kTol);
    // a_ff has small Hermite-basis roundoff even for linear input; a
    // nanometer-per-s² tolerance is plenty for RT control feedforward.
    EXPECT_NEAR(a_ff_(i), 0.0, 1e-9);
  }
}

TEST_F(TrajectoryInterpolatorTest, LinearTrajectoryReproducedExactly) {
  const double slope = 3.14;
  interp_.SetSolution(MakeLinearSolution(slope), kReceiveNs);

  // Sample at 3 non-node times; expect exact reproduction because the
  // underlying signal is linear.
  for (double t_offset : {0.003, 0.017, 0.076}) {
    const uint64_t now = kReceiveNs +
        static_cast<uint64_t>(t_offset * 1e9);
    interp_.Interpolate(now, q_ref_, v_ref_, a_ff_, lambda_ref_, meta_);
    ASSERT_TRUE(meta_.valid);
    EXPECT_FALSE(meta_.beyond_horizon);
    for (int i = 0; i < kNq; ++i) {
      const double expected = slope * t_offset + 0.1 * i;
      EXPECT_NEAR(q_ref_(i), expected, kTol)
          << "t_offset=" << t_offset << " i=" << i;
    }
    for (int i = 0; i < kNv; ++i) {
      EXPECT_NEAR(v_ref_(i), slope, kTol);
    }
  }
}

TEST_F(TrajectoryInterpolatorTest, BeyondHorizonFreezesLastNode) {
  const double slope = 1.0;
  interp_.SetSolution(MakeLinearSolution(slope), kReceiveNs);

  // 1 s past the horizon (horizon = 10 * 10ms = 100 ms).
  const uint64_t now = kReceiveNs + 1'000'000'000;
  interp_.Interpolate(now, q_ref_, v_ref_, a_ff_, lambda_ref_, meta_);
  ASSERT_TRUE(meta_.valid);
  EXPECT_TRUE(meta_.beyond_horizon);
  EXPECT_NEAR(meta_.progress, 1.0, kTol);

  const double horizon_sec = kHorizon * kDtNode;
  for (int i = 0; i < kNq; ++i) {
    EXPECT_NEAR(q_ref_(i), slope * horizon_sec + 0.1 * i, kTol);
  }
  for (int i = 0; i < kNv; ++i) {
    EXPECT_NEAR(a_ff_(i), 0.0, kTol)
        << "a_ff must be zero beyond horizon (safety default)";
  }
}

TEST_F(TrajectoryInterpolatorTest, NegativeTimeClampedToOrigin) {
  const double slope = 1.5;
  interp_.SetSolution(MakeLinearSolution(slope), kReceiveNs);

  // now_ns earlier than receive_ns — should clamp to t_local = 0.
  interp_.Interpolate(kReceiveNs - 500'000'000, q_ref_, v_ref_, a_ff_,
                      lambda_ref_, meta_);
  ASSERT_TRUE(meta_.valid);
  EXPECT_FALSE(meta_.beyond_horizon);
  for (int i = 0; i < kNq; ++i) {
    EXPECT_NEAR(q_ref_(i), 0.1 * i, kTol);
  }
}

TEST_F(TrajectoryInterpolatorTest, RemainingHorizonMonotoneDecrease) {
  interp_.SetSolution(MakeLinearSolution(1.0), kReceiveNs);

  const double r0 = interp_.RemainingHorizonSec(kReceiveNs);
  const double r_mid =
      interp_.RemainingHorizonSec(kReceiveNs + 50'000'000);  // +50 ms
  const double r_end =
      interp_.RemainingHorizonSec(kReceiveNs + 200'000'000);  // past horizon
  EXPECT_NEAR(r0, kHorizon * kDtNode, kTol);
  EXPECT_GT(r0, r_mid);
  EXPECT_GT(r_mid, 0.0);
  EXPECT_NEAR(r_end, 0.0, kTol);
}

/// Build a solution sampled from a cubic polynomial q(t) = c0 + c1 t + c2 t² + c3 t³
/// with matching analytical velocity. A cubic Hermite interpolator must
/// reproduce this exactly (up to fp roundoff) between any pair of nodes.
MPCSolution MakeCubicSolution(double c0, double c1, double c2, double c3) {
  MPCSolution sol{};
  sol.horizon_length = kHorizon;
  sol.dt_node = kDtNode;
  sol.converged = true;
  sol.nq = kNq;
  sol.nv = kNv;
  sol.nu = kNv;
  sol.nx = kNq + kNv;
  sol.n_contact_vars = kNc;

  for (int k = 0; k <= kHorizon; ++k) {
    const double t = static_cast<double>(k) * kDtNode;
    const double q = c0 + c1 * t + c2 * t * t + c3 * t * t * t;
    const double v = c1 + 2.0 * c2 * t + 3.0 * c3 * t * t;
    for (int i = 0; i < kNq; ++i) {
      sol.q_traj[static_cast<std::size_t>(k)]
                [static_cast<std::size_t>(i)] = q + 0.1 * i;
      sol.v_traj[static_cast<std::size_t>(k)]
                [static_cast<std::size_t>(i)] = v;
    }
  }
  return sol;
}

TEST_F(TrajectoryInterpolatorTest, HermiteReproducesCubicExactly) {
  const double c0 = 0.5, c1 = 1.2, c2 = -0.3, c3 = 0.7;
  interp_.SetSolution(MakeCubicSolution(c0, c1, c2, c3), kReceiveNs);

  // Sample at 7 non-node times spanning multiple segments.
  const std::array<double, 7> offsets{
      0.0015, 0.0049, 0.023, 0.038, 0.061, 0.077, 0.091};
  for (double t_offset : offsets) {
    const uint64_t now = kReceiveNs +
        static_cast<uint64_t>(t_offset * 1e9);
    interp_.Interpolate(now, q_ref_, v_ref_, a_ff_, lambda_ref_, meta_);
    ASSERT_TRUE(meta_.valid);

    const double t = t_offset;
    const double q = c0 + c1 * t + c2 * t * t + c3 * t * t * t;
    const double v = c1 + 2.0 * c2 * t + 3.0 * c3 * t * t;
    const double a = 2.0 * c2 + 6.0 * c3 * t;
    for (int i = 0; i < kNq; ++i) {
      EXPECT_NEAR(q_ref_(i), q + 0.1 * i, 1e-10)
          << "t=" << t_offset << " i=" << i;
    }
    for (int i = 0; i < kNv; ++i) {
      EXPECT_NEAR(v_ref_(i), v, 1e-10);
      EXPECT_NEAR(a_ff_(i), a, 1e-9);
    }
  }
}

TEST_F(TrajectoryInterpolatorTest, HermiteC1ContinuityAtNodeBoundary) {
  // A generic cubic — exactly reproduced — yields bit-exact C1 continuity
  // at node boundaries. We compare the limit from the left of node k+1
  // (t = (k+1)·Δt - ε) with the value at that node.
  const double c0 = 0.0, c1 = 2.0, c2 = 0.5, c3 = -0.8;
  interp_.SetSolution(MakeCubicSolution(c0, c1, c2, c3), kReceiveNs);

  const uint64_t dt_ns = static_cast<uint64_t>(kDtNode * 1e9);
  for (int k = 1; k < kHorizon; ++k) {
    const uint64_t node_ns = kReceiveNs + k * dt_ns;

    Eigen::VectorXd q_left = q_ref_;
    Eigen::VectorXd v_left = v_ref_;
    interp_.Interpolate(node_ns - 1, q_left, v_left, a_ff_, lambda_ref_,
                        meta_);
    ASSERT_TRUE(meta_.valid);

    Eigen::VectorXd q_right = q_ref_;
    Eigen::VectorXd v_right = v_ref_;
    interp_.Interpolate(node_ns + 1, q_right, v_right, a_ff_, lambda_ref_,
                        meta_);
    ASSERT_TRUE(meta_.valid);

    for (int i = 0; i < kNq; ++i) {
      EXPECT_NEAR(q_left(i), q_right(i), 1e-6) << "k=" << k << " i=" << i;
    }
    for (int i = 0; i < kNv; ++i) {
      EXPECT_NEAR(v_left(i), v_right(i), 1e-4) << "k=" << k << " i=" << i;
    }
  }
}

TEST_F(TrajectoryInterpolatorTest, HermiteReducesToLinearForLinearInput) {
  // With v_k = (q_{k+1} - q_k)/Δt (linear trajectory), Hermite must agree
  // with the linear interpolation test from Step 4. This pins down the
  // baseline we upgraded from.
  const double slope = 2.5;
  interp_.SetSolution(MakeLinearSolution(slope), kReceiveNs);

  for (double t_offset : {0.003, 0.017, 0.076}) {
    const uint64_t now = kReceiveNs +
        static_cast<uint64_t>(t_offset * 1e9);
    interp_.Interpolate(now, q_ref_, v_ref_, a_ff_, lambda_ref_, meta_);
    ASSERT_TRUE(meta_.valid);

    for (int i = 0; i < kNq; ++i) {
      EXPECT_NEAR(q_ref_(i), slope * t_offset + 0.1 * i, 1e-12);
    }
    for (int i = 0; i < kNv; ++i) {
      EXPECT_NEAR(v_ref_(i), slope, 1e-12);
      EXPECT_NEAR(a_ff_(i), 0.0, 1e-9)
          << "a_ff must vanish for a linear trajectory";
    }
  }
}

TEST_F(TrajectoryInterpolatorTest, InitWithOutOfRangeDimsClearsState) {
  interp_.Init(kMaxNq + 1, kNv, kNc);  // nq too large
  EXPECT_FALSE(interp_.HasSolution());

  interp_.SetSolution(MakeLinearSolution(1.0), kReceiveNs);
  interp_.Interpolate(kReceiveNs, q_ref_, v_ref_, a_ff_, lambda_ref_, meta_);
  // With nq_ == 0, q_ref_ entries are untouched (meta may still be valid).
  // Guard: the vector the caller provided must not be corrupted.
  for (int i = 0; i < q_ref_.size(); ++i) {
    EXPECT_NEAR(q_ref_(i), 0.0, kTol);
  }
}

}  // namespace
}  // namespace rtc::mpc
