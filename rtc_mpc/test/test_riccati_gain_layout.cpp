/// @file test_riccati_gain_layout.cpp
/// @brief Writer ↔ reader contract for the flattened Riccati gain
///        (`MPCSolution::K_riccati[k]`): `PackRiccatiGainRowMajor` must
///        produce exactly the layout `RiccatiFeedback::SetGain` consumes —
///        row-major, row stride = the block's actual column count, not a
///        capacity constant. The negative control packs with the capacity
///        stride and shows the reader no longer sees the matrix, which is
///        the drift this suite exists to catch. The solver-side packer is
///        covered end-to-end in test_contact_light_mpc.cpp.

#include "rtc_mpc/feedback/riccati_feedback.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <initializer_list>

namespace rtc::mpc {
namespace {

// nu ≠ nx ≠ kMaxNdx so a stride mistake in either direction is visible.
constexpr int kNq = 4;
constexpr int kNv = 3;
constexpr int kNx = kNq + kNv;
constexpr int kNu = 3;
static_assert(kNu != kNx && kNx != kMaxNdx && kNu != kMaxNdx);

/// Distinct value per element so a misplaced entry cannot alias another.
Eigen::MatrixXd DistinctGain() {
  Eigen::MatrixXd K(kNu, kNx);
  for (int i = 0; i < kNu; ++i) {
    for (int j = 0; j < kNx; ++j) {
      K(i, j) = 1.0 + 10.0 * i + j;
    }
  }
  return K;
}

/// Read column `j` of the installed gain back through Compute: with a
/// one-hot Δx (gain scale 1, full-nu output) `u_fb == K · e_j`, and the
/// product of exact small integers with 0 / 1 is exact.
Eigen::VectorXd ReadColumn(RiccatiFeedback& riccati, int j) {
  Eigen::VectorXd q_curr = Eigen::VectorXd::Zero(kNq);
  Eigen::VectorXd v_curr = Eigen::VectorXd::Zero(kNv);
  const Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(kNq);
  const Eigen::VectorXd v_ref = Eigen::VectorXd::Zero(kNv);
  if (j < kNq) {
    q_curr(j) = 1.0;
  } else {
    v_curr(j - kNq) = 1.0;
  }
  Eigen::VectorXd u_fb = Eigen::VectorXd::Zero(kNu);
  riccati.Compute(q_curr, v_curr, q_ref, v_ref, u_fb);
  return u_fb;
}

class RiccatiGainLayoutTest : public ::testing::Test {
 protected:
  void SetUp() override {
    riccati_.Init(kNv, kNu, kNx);
    riccati_.SetAccelOnly(false);
    riccati_.SetGainScale(1.0);
    flat_.fill(0.0);
  }

  RiccatiFeedback riccati_;
  std::array<double, MPCSolution::kRiccatiGainFlatSize> flat_{};
};

TEST_F(RiccatiGainLayoutTest, PackThenSetGainRecoversEveryElement) {
  const Eigen::MatrixXd K = DistinctGain();
  ASSERT_TRUE(PackRiccatiGainRowMajor(K, flat_.data(), flat_.size()));
  riccati_.SetGain(flat_.data(), kNu, kNx);
  ASSERT_TRUE(riccati_.HasGain());

  for (int j = 0; j < kNx; ++j) {
    const Eigen::VectorXd col = ReadColumn(riccati_, j);
    ASSERT_EQ(col.size(), kNu);
    for (int i = 0; i < kNu; ++i) {
      EXPECT_DOUBLE_EQ(col(i), K(i, j)) << "K(" << i << ", " << j << ")";
    }
  }
}

TEST_F(RiccatiGainLayoutTest, BlockRefBindsDirectAccessViewsWithoutCopy) {
  // The solver hands over a sub-block of a larger ColMajor matrix
  // (feedforward column + gain block). The parameter type must alias such a
  // view — and one with a non-unit inner stride — instead of copying it
  // into a heap temporary, which a plain Ref<const MatrixXd> would do for
  // the latter. Aliasing is observable: the Ref's data pointer is the
  // view's first element.
  Eigen::MatrixXd parent = Eigen::MatrixXd::Constant(kNu + 2, kNx + 1, -1.0);
  parent.topRightCorner(kNu, kNx) = DistinctGain();
  const RiccatiGainBlockRef corner_view(parent.topRightCorner(kNu, kNx));
  EXPECT_EQ(corner_view.data(), &parent(0, 1));

  // Every other row of a twice-as-tall ColMajor buffer: inner stride 2.
  Eigen::MatrixXd tall = Eigen::MatrixXd::Constant(2 * kNu, kNx, -1.0);
  for (int i = 0; i < kNu; ++i) {
    tall.row(2 * i) = DistinctGain().row(i);
  }
  using StridedMap = Eigen::Map<const Eigen::MatrixXd, 0, Eigen::Stride<Eigen::Dynamic, 2>>;
  const StridedMap strided(tall.data(), kNu, kNx, Eigen::Stride<Eigen::Dynamic, 2>(tall.rows(), 2));
  const RiccatiGainBlockRef strided_view(strided);
  EXPECT_EQ(strided_view.data(), tall.data());

  // Both views pack to the same reader-visible matrix.
  const Eigen::MatrixXd K = DistinctGain();
  for (const RiccatiGainBlockRef* view : {&corner_view, &strided_view}) {
    flat_.fill(0.0);
    ASSERT_TRUE(PackRiccatiGainRowMajor(*view, flat_.data(), flat_.size()));
    riccati_.SetGain(flat_.data(), kNu, kNx);
    for (int j = 0; j < kNx; ++j) {
      const Eigen::VectorXd col = ReadColumn(riccati_, j);
      for (int i = 0; i < kNu; ++i) {
        EXPECT_DOUBLE_EQ(col(i), K(i, j)) << "K(" << i << ", " << j << ")";
      }
    }
  }
}

TEST_F(RiccatiGainLayoutTest, CapacityStrideIsNotTheReaderLayout) {
  // Negative control: write with the capacity stride, as a writer that
  // confuses capacity with size would. Only the first row survives.
  const Eigen::MatrixXd K = DistinctGain();
  for (int i = 0; i < kNu; ++i) {
    for (int j = 0; j < kNx; ++j) {
      flat_[static_cast<std::size_t>(i) * static_cast<std::size_t>(kMaxNdx) +
            static_cast<std::size_t>(j)] = K(i, j);
    }
  }
  riccati_.SetGain(flat_.data(), kNu, kNx);

  const Eigen::VectorXd col0 = ReadColumn(riccati_, 0);
  EXPECT_DOUBLE_EQ(col0(0), K(0, 0)) << "row 0 is stride-independent";
  EXPECT_NE(col0(1), K(1, 0)) << "row 1 must be misread under the wrong stride";
}

TEST(RiccatiGainLayout, PackRejectsBlockLargerThanBuffer) {
  const Eigen::MatrixXd K = DistinctGain();
  constexpr std::size_t kNeeded = static_cast<std::size_t>(kNu) * static_cast<std::size_t>(kNx);

  std::array<double, kNeeded> exact{};
  exact.fill(-7.0);
  EXPECT_TRUE(PackRiccatiGainRowMajor(K, exact.data(), exact.size()));
  EXPECT_DOUBLE_EQ(exact.back(), K(kNu - 1, kNx - 1));

  std::array<double, kNeeded - 1> short_buf{};
  short_buf.fill(-7.0);
  EXPECT_FALSE(PackRiccatiGainRowMajor(K, short_buf.data(), short_buf.size()));
  for (const double v : short_buf) {
    EXPECT_DOUBLE_EQ(v, -7.0) << "a rejected pack must leave the buffer untouched";
  }

  EXPECT_FALSE(PackRiccatiGainRowMajor(K, nullptr, kNeeded));
}

}  // namespace
}  // namespace rtc::mpc
