#include "rtc_tsid/contact/grasp_cache.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace rtc::tsid {
namespace {

// Case 1: Identity-like mapping — G = [I_6, 0_{6×3}] is row-full-rank with
// rank 6. G_pinv·G should project orthogonally onto Range(Gᵀ); P_N should
// remove the row-rank-6 component. Specifically G·G_pinv ≈ I_6 (right
// inverse for full row-rank), and P_N annihilates rows 0..5 while leaving
// rows 6..8 intact.
TEST(GraspCacheTest, IdentityMappingFullRank) {
  const int n_lambda = 9;
  GraspCache cache;
  cache.Init(n_lambda);

  Eigen::MatrixXd G(6, n_lambda);
  G.setZero();
  G.leftCols<6>().setIdentity();  // identity in first 6 cols

  cache.Compute(G, n_lambda);

  EXPECT_EQ(cache.CurrentLambdaDim(), n_lambda);
  EXPECT_EQ(cache.Rank(), 6);

  // G · G_pinv ≈ I_6 (right inverse for full row-rank G).
  const Eigen::MatrixXd G_pinv = cache.GPinv();
  ASSERT_EQ(G_pinv.rows(), n_lambda);
  ASSERT_EQ(G_pinv.cols(), 6);
  const Eigen::MatrixXd G_Gpinv = G * G_pinv;
  EXPECT_LT((G_Gpinv - Eigen::MatrixXd::Identity(6, 6)).norm(), 1e-8);

  // P_N · G_pinv ≈ 0 (P_N projects onto Null(G); G_pinv columns live in
  // Range(Gᵀ) by construction, which is the orthogonal complement).
  const Eigen::MatrixXd P_N = cache.ProjN();
  EXPECT_LT((P_N * G_pinv).norm(), 1e-8);

  // P_N has 3-d range (the null space of [I_6 | 0_{6×3}] is the last 3
  // coordinates). Trace(P_N) ≈ rank(P_N) = n_lambda - rank_G = 3.
  EXPECT_NEAR(P_N.trace(), 3.0, 1e-8);
}

// Case 2: Two collinear point contacts at the same world position produce
// a rank-deficient G — both 6×3 blocks are identical, so rank(G) ≤ 3 and
// P_N has a non-trivial 3-d kernel direction (the antisymmetric
// combination λ_1 = +v, λ_2 = -v cancels out).
TEST(GraspCacheTest, RankDeficient_TwoCollinearPointContacts) {
  const int n_lambda = 6;  // 2 point contacts × 3 force coords
  GraspCache cache;
  cache.Init(n_lambda);

  const Eigen::Vector3d p_ci{0.2, -0.1, 0.4};  // both contacts at same point
  Eigen::MatrixXd G_block(6, 3);
  G_block.topLeftCorner<3, 3>().setIdentity();
  G_block.bottomLeftCorner<3, 3>() << 0.0, -p_ci.z(), p_ci.y(),  //
      p_ci.z(), 0.0, -p_ci.x(),                                  //
      -p_ci.y(), p_ci.x(), 0.0;                                  //

  Eigen::MatrixXd G(6, n_lambda);
  G.leftCols<3>() = G_block;
  G.rightCols<3>() = G_block;  // collinear duplicate

  cache.Compute(G, n_lambda);

  // Numerically rank(G) = 3 (both blocks are identical 6×3, so the
  // column-rank is 3).
  EXPECT_EQ(cache.Rank(), 3);

  // P_N · v_anti should preserve v_anti (it lies entirely in Null(G)).
  Eigen::VectorXd v_anti(n_lambda);
  v_anti.head<3>() = Eigen::Vector3d{1.0, -2.0, 0.5};
  v_anti.tail<3>() = -v_anti.head<3>();

  // G · v_anti ≈ 0 (numerical confirmation that v_anti ∈ Null(G)).
  EXPECT_LT((G * v_anti).norm(), 1e-12);

  const Eigen::MatrixXd P_N = cache.ProjN();
  const Eigen::VectorXd v_proj = P_N * v_anti;
  EXPECT_LT((v_proj - v_anti).norm(), 1e-6) << "v_anti should be fixed by P_N";

  // Conversely, a symmetric vector v_sym = (+v; +v) is in Range(Gᵀ), so
  // P_N · v_sym ≈ 0 (damped projector — tolerance reflects Tikhonov bias).
  Eigen::VectorXd v_sym(n_lambda);
  v_sym.head<3>() = Eigen::Vector3d{1.0, -2.0, 0.5};
  v_sym.tail<3>() = v_sym.head<3>();
  EXPECT_LT((P_N * v_sym).norm(), 1e-6);
}

// Case 3: Damped Penrose consistency — at the chosen damping tolerance
// (1e-9 default), G·G_pinv·G ≈ G and the related identities hold to a
// loose Tikhonov tolerance. The deeper Penrose-2 (G_pinv·G·G_pinv ≈ G_pinv)
// is intentionally not enforced here: damping perturbs G_pinv away from
// the exact Moore–Penrose inverse by O(ε / σ_min), so we instead verify
// the *useful* invariants that downstream Stage B-3/B-4 tasks rely on.
TEST(GraspCacheTest, PseudoInverseConsistency) {
  const int n_lambda = 9;
  GraspCache cache;
  cache.Init(n_lambda);

  // Build a deterministic well-conditioned G via a fixed seed.
  Eigen::MatrixXd G(6, n_lambda);
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < n_lambda; ++c) {
      G(r, c) = std::sin(0.5 * r + 0.3 * c + 1.0) + 0.1 * (r + c);
    }
  }

  cache.Compute(G, n_lambda);

  const Eigen::MatrixXd G_pinv = cache.GPinv();
  const Eigen::MatrixXd P_N = cache.ProjN();
  const Eigen::MatrixXd GT_pinv = cache.GTPinv();

  // Penrose-1: G·G_pinv·G ≈ G (damped, but still holds tight). This is
  // the invariant Stage B-3/B-4 actually depends on.
  EXPECT_LT((G * G_pinv * G - G).norm(), 1e-6) << "Penrose-1 violation";

  // GT_pinv coincides with G_pinv^T up to the LDLT inversion residual.
  // M is symmetric so M^{-1} is symmetric, hence M^{-1}·G == (G^T·M^{-1})^T.
  // Tolerance ~ trace(M)·eps_machine via two independent solveInPlace calls.
  EXPECT_LT((GT_pinv - G_pinv.transpose()).norm(), 1e-5);

  // Projector approximate idempotency: P_N·P_N ≈ P_N to the damping
  // tolerance. Tightness is bounded by the same Tikhonov bias.
  EXPECT_LT((P_N * P_N - P_N).norm(), 1e-3) << "P_N not idempotent (damped)";

  // Projector approximately annihilates Range(Gᵀ) (becomes exact only at
  // ε → 0).
  EXPECT_LT((P_N * G.transpose()).norm(), 1e-5);

  // Useful sanity: rank reported as 6 for this generic full-row-rank G.
  EXPECT_EQ(cache.Rank(), 6);
}

// Case 4: n_lambda < 6 path (under-determined direction-wise) — single
// 3-DOF point contact gives a 6×3 G with rank 3; the n_λ × n_λ normal
// equation path must produce the same Penrose-consistent pinv.
TEST(GraspCacheTest, SmallLambdaPath_PointContact) {
  const int n_lambda = 3;
  GraspCache cache;
  cache.Init(n_lambda);

  const Eigen::Vector3d p_ci{0.3, 0.0, 0.5};
  Eigen::MatrixXd G(6, n_lambda);
  G.topLeftCorner<3, 3>().setIdentity();
  G.bottomLeftCorner<3, 3>() << 0.0, -p_ci.z(), p_ci.y(),  //
      p_ci.z(), 0.0, -p_ci.x(),                            //
      -p_ci.y(), p_ci.x(), 0.0;                            //

  cache.Compute(G, n_lambda);

  EXPECT_EQ(cache.Rank(), 3);

  const Eigen::MatrixXd G_pinv = cache.GPinv();
  ASSERT_EQ(G_pinv.rows(), n_lambda);
  ASSERT_EQ(G_pinv.cols(), 6);

  // G_pinv · G ≈ I_3 (left inverse for full column-rank G).
  EXPECT_LT((G_pinv * G - Eigen::MatrixXd::Identity(n_lambda, n_lambda)).norm(), 1e-8);

  // P_N ≈ 0 (full column rank: nullity = 0).
  const Eigen::MatrixXd P_N = cache.ProjN();
  EXPECT_LT(P_N.norm(), 1e-8) << "expected near-zero P_N for full column rank";
}

}  // namespace
}  // namespace rtc::tsid
