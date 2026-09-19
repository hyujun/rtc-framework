/// @file test_clik_options.cpp
/// @brief ClikReferenceGenerator S2.2b options (dynamic_catching, L5 §5.1, gates
///        G5-A / G5-B / G5-B2 / G5-C / G5-C3 — CLIK side only).
///
/// Every option defaults off; "off ⇒ bit-identical" is the golden-vector
/// suite's job (test_clik_golden.cpp). This file checks what each option does
/// when it is ON.

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <random>
#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/kinematics/clik_reference.hpp"

// ── TU-local alloc counter (test_clik_reference.cpp pattern) ────────────────
namespace {

struct AllocCounter {
  inline static std::atomic<std::int64_t> alloc_count{0};
  inline static std::atomic<bool> armed{false};

  static void Arm() noexcept {
    alloc_count.store(0, std::memory_order_relaxed);
    armed.store(true, std::memory_order_release);
  }

  static void Disarm() noexcept { armed.store(false, std::memory_order_release); }

  static void Record() noexcept {
    if (armed.load(std::memory_order_acquire)) {
      alloc_count.fetch_add(1, std::memory_order_relaxed);
    }
  }
};

}  // namespace

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif

void* operator new(std::size_t sz) {
  void* p = std::malloc(sz);
  if (p == nullptr) {
    throw std::bad_alloc{};
  }
  AllocCounter::Record();
  return p;
}

void* operator new[](std::size_t sz) {
  void* p = std::malloc(sz);
  if (p == nullptr) {
    throw std::bad_alloc{};
  }
  AllocCounter::Record();
  return p;
}

void operator delete(void* p) noexcept {
  std::free(p);
}

void operator delete[](void* p) noexcept {
  std::free(p);
}

void operator delete(void* p, std::size_t) noexcept {
  std::free(p);
}

void operator delete[](void* p, std::size_t) noexcept {
  std::free(p);
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;
constexpr int kNv = 9;  // Panda: 7 arm + 2 finger, nq == nv
constexpr double kDt = 0.002;
// ProxQP eps_abs (QPSolverConfig default): the box holds only to this tolerance.
constexpr double kSolverEps = 1e-6;

using Vec6 = Eigen::Matrix<double, 6, 1>;

class ClikOptionsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;
    YAML::Node config;
    robot_info_.Build(*model_, config);
    ASSERT_EQ(robot_info_.nv, kNv);

    ContactManagerConfig contact_cfg;
    contact_cfg.max_contacts = 0;
    cache_.Init(model_, rtc::tsid::ContactFrameIds(contact_cfg));
    tcp_idx_ = cache_.RegisterFrame("panda_hand", model_->getFrameId("panda_hand"));
    base_idx_ = cache_.RegisterFrame("panda_link0", model_->getFrameId("panda_link0"));
    ASSERT_GE(tcp_idx_, 0);
    ASSERT_GE(base_idx_, 0);

    q_home_.resize(kNv);
    q_home_ << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.02, 0.02;
    v_zero_ = Eigen::VectorXd::Zero(kNv);
  }

  [[nodiscard]] static ClikReferenceGenerator::Config BaseConfig() {
    ClikReferenceGenerator::Config cfg;
    cfg.arm_v_idx = {0, 1, 2, 3, 4, 5, 6};
    cfg.hand_v_idx = {7, 8};
    cfg.damping_sq = 1e-4;
    cfg.v_limit = 1.5;
    return cfg;
  }

  [[nodiscard]] pinocchio::SE3 TipInBase() const {
    const auto& tip = cache_.registered_frames[static_cast<size_t>(tcp_idx_)];
    const auto& base = cache_.registered_frames[static_cast<size_t>(base_idx_)];
    return base.oMf.actInv(tip.oMf);
  }

  [[nodiscard]] pinocchio::SE3 OffsetTarget(double dz) {
    cache_.Update(q_home_, v_zero_);
    pinocchio::SE3 des = TipInBase();
    des.translation()(2) += dz;
    return des;
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  PinocchioCache cache_;
  int tcp_idx_{-1};
  int base_idx_{-1};
  Eigen::VectorXd q_home_;
  Eigen::VectorXd v_zero_;
};

// ── Diagnostics (status · iterations · solve time) ──────────────────────────

TEST_F(ClikOptionsTest, LastSolveReportsSolverOutcome) {
  ClikReferenceGenerator gen;
  gen.Init(kNv, BaseConfig());
  gen.SetTaskGain(Vec6::Constant(2.0));
  const pinocchio::SE3 des = OffsetTarget(0.05);

  EXPECT_FALSE(gen.LastSolve().reached_solve) << "nothing solved yet";

  cache_.Update(q_home_, v_zero_);
  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, kDt));
  const auto& ok = gen.LastSolve();
  EXPECT_TRUE(ok.reached_solve);
  EXPECT_TRUE(ok.converged);
  EXPECT_FALSE(ok.non_finite);
  EXPECT_EQ(ok.status, 0);  // PROXQP_SOLVED
  EXPECT_GT(ok.iterations, 0);
  EXPECT_GT(ok.solve_time_us, 0.0);

  // Non-finite target: the QP fails with non-finite iterates.
  pinocchio::SE3 bad = des;
  bad.translation()(0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(gen.Compute(cache_, tcp_idx_, base_idx_, bad, q_home_, kDt));
  EXPECT_TRUE(gen.LastSolve().reached_solve);
  EXPECT_FALSE(gen.LastSolve().converged);
  EXPECT_TRUE(gen.LastSolve().non_finite);

  // Precondition failure never reaches the solver and clears the record.
  EXPECT_FALSE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, 0.0));
  EXPECT_FALSE(gen.LastSolve().reached_solve);
  EXPECT_EQ(gen.LastSolve().status, -1);
}

// ── max_iter (G5-C3, CLIK side) ─────────────────────────────────────────────

TEST_F(ClikOptionsTest, MaxIterDefaultIsLegacyValue) {
  EXPECT_EQ(ClikReferenceGenerator::Config{}.max_iter, QPSolverConfig{}.max_iter);
}

TEST_F(ClikOptionsTest, MaxIterRejectsNonPositive) {
  for (const int bad : {0, -1}) {
    ClikReferenceGenerator gen;
    auto cfg = BaseConfig();
    cfg.max_iter = bad;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << bad;
  }
}

// A cap of 1 on a problem with active bounds cannot converge: the call fails,
// the status says why, and the iteration count respects the cap.
TEST_F(ClikOptionsTest, MaxIterCapIsHonouredAndReported) {
  auto cfg = BaseConfig();
  cfg.v_limit = 0.05;  // bind the velocity box so the QP needs several iterations
  cfg.max_iter = 1;
  ClikReferenceGenerator gen;
  gen.Init(kNv, cfg);
  gen.SetTaskGain(Vec6::Constant(10.0));
  const pinocchio::SE3 des = OffsetTarget(0.2);
  cache_.Update(q_home_, v_zero_);
  EXPECT_FALSE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, kDt));
  EXPECT_TRUE(gen.LastSolve().reached_solve);
  EXPECT_FALSE(gen.LastSolve().converged);
  EXPECT_EQ(gen.LastSolve().status, 1);  // PROXQP_MAX_ITER_REACHED
  EXPECT_LE(gen.LastSolve().iterations, 1);

  // The same problem converges under the default cap.
  ClikReferenceGenerator ref;
  cfg.max_iter = ClikReferenceGenerator::Config{}.max_iter;
  ref.Init(kNv, cfg);
  ref.SetTaskGain(Vec6::Constant(10.0));
  EXPECT_TRUE(ref.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, kDt));
}

}  // namespace
}  // namespace rtc::tsid
