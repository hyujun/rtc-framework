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

// ── Per-joint velocity limits ───────────────────────────────────────────────

TEST_F(ClikOptionsTest, PerJointVelocityLimitRejectsInvalid) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad : {0.0, -1.0, nan, inf}) {
    auto cfg = BaseConfig();
    cfg.v_limit_per_joint = Eigen::VectorXd::Constant(kNv, 1.0);
    cfg.v_limit_per_joint(4) = bad;
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << bad;
  }
  auto cfg = BaseConfig();
  cfg.v_limit_per_joint = Eigen::VectorXd::Constant(kNv - 1, 1.0);
  ClikReferenceGenerator gen;
  EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error);
}

// One tight joint among loose ones: the tight one is bounded by its own limit,
// and a loose one is allowed past the tight value (so the scalar is not used).
TEST_F(ClikOptionsTest, PerJointVelocityLimitBoundsEachJoint) {
  auto cfg = BaseConfig();
  cfg.v_limit = 0.01;  // would bind everything if it were still used
  cfg.v_limit_per_joint = Eigen::VectorXd::Constant(kNv, 2.0);
  cfg.v_limit_per_joint(1) = 0.1;
  ClikReferenceGenerator gen;
  gen.Init(kNv, cfg);
  gen.SetTaskGain(Vec6::Constant(20.0));
  const pinocchio::SE3 des = OffsetTarget(0.3);

  double max_tight = 0.0;
  double max_loose = 0.0;
  Eigen::VectorXd q = q_home_;
  for (int k = 0; k < 50; ++k) {
    cache_.Update(q, v_zero_);
    ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, kDt));
    for (int j = 0; j < kNv; ++j) {
      ASSERT_LE(std::abs(gen.VRef()(j)), cfg.v_limit_per_joint(j) + kSolverEps) << j;
    }
    max_tight = std::max(max_tight, std::abs(gen.VRef()(1)));
    for (const int j : {0, 2, 3, 4, 5, 6}) {
      max_loose = std::max(max_loose, std::abs(gen.VRef()(j)));
    }
    q = gen.QRef();
  }
  EXPECT_NEAR(max_tight, 0.1, kSolverEps) << "tight joint never reached its limit";
  EXPECT_GT(max_loose, 0.1 + 1e-3) << "loose joints were held to the tight or scalar value";
}

// A joint past q_max collapses its box; the recovery speed is re-clamped to
// that joint's own limit.
TEST_F(ClikOptionsTest, PerJointLimitReclampsCollapsedBox) {
  auto cfg = BaseConfig();
  cfg.q_min = model_->lowerPositionLimit;
  cfg.q_max = model_->upperPositionLimit;
  cfg.v_limit_per_joint = Eigen::VectorXd::Constant(kNv, 2.0);
  cfg.v_limit_per_joint(3) = 0.2;
  ClikReferenceGenerator gen;
  gen.Init(kNv, cfg);
  gen.SetTaskGain(Vec6::Constant(2.0));
  const pinocchio::SE3 des = OffsetTarget(0.05);

  Eigen::VectorXd q = q_home_;
  q(3) = model_->upperPositionLimit(3) + 0.05;  // raw collapse would be −25 rad/s at kDt
  cache_.Update(q, v_zero_);
  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, kDt));
  EXPECT_NEAR(gen.VRef()(3), -0.2, kSolverEps);
}

// ── Acceleration box + bound_conflict (G5-B, G5-B2 CLIK side) ──────────────

TEST_F(ClikOptionsTest, AccelBoxRejectsInvalid) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad : {0.0, -1.0, nan, inf}) {
    auto cfg = BaseConfig();
    cfg.a_max = Eigen::VectorXd::Constant(kNv, 10.0);
    cfg.a_max(2) = bad;
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << bad;
  }
  auto cfg = BaseConfig();
  cfg.a_max = Eigen::VectorXd::Constant(3, 10.0);
  ClikReferenceGenerator gen;
  EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error);
}

// G5-B: 1e4 ticks of random target jumps with the robot tracking q_ref
// exactly. Velocity and acceleration stay inside their boxes on every tick
// (to the solver tolerance), and the position limit is exceeded only through a
// reported conflict, by less than the margin the CLIK box was shrunk by.
TEST_F(ClikOptionsTest, RandomReferencesRespectVelocityAndAccelerationBoxes) {
  constexpr double kMargin = 0.1;
  constexpr double kAMax = 20.0;
  auto cfg = BaseConfig();
  cfg.v_limit_per_joint = Eigen::VectorXd::Constant(kNv, 1.5);
  cfg.v_limit_per_joint.tail(2).setConstant(0.1);
  cfg.a_max = Eigen::VectorXd::Constant(kNv, kAMax);
  const Eigen::VectorXd q_lo = model_->lowerPositionLimit;
  const Eigen::VectorXd q_hi = model_->upperPositionLimit;
  cfg.q_min = q_lo.array() + kMargin;
  cfg.q_max = q_hi.array() - kMargin;
  cfg.q_min.tail(2) = q_lo.tail(2);  // fingers: 4 cm range, no room for a margin
  cfg.q_max.tail(2) = q_hi.tail(2);
  ClikReferenceGenerator gen;
  gen.Init(kNv, cfg);
  gen.SetTaskGain(Vec6::Constant(20.0));
  gen.SetPostureGains(1.0, 1.0);

  cache_.Update(q_home_, v_zero_);
  const pinocchio::SE3 home = TipInBase();
  std::mt19937 rng(537);
  std::uniform_real_distribution<double> offset(-0.35, 0.35);

  Eigen::VectorXd q = q_home_;
  Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(kNv);
  pinocchio::SE3 des = home;
  int conflicts = 0;
  double worst_overshoot = 0.0;
  for (int k = 0; k < 10000; ++k) {
    if (k % 250 == 0) {
      des = home;
      des.translation() += Eigen::Vector3d(offset(rng), offset(rng), offset(rng));
    }
    cache_.Update(q, v_prev);
    ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, kDt)) << k;
    const Eigen::VectorXd& v = gen.VRef();
    const auto& diag = gen.LastSolve();
    conflicts += diag.bound_conflict ? 1 : 0;
    for (int j = 0; j < kNv; ++j) {
      ASSERT_LE(std::abs(v(j)), cfg.v_limit_per_joint(j) + kSolverEps)
          << "tick " << k << " j " << j;
      ASSERT_LE(std::abs(v(j) - v_prev(j)), kAMax * kDt + kSolverEps) << "tick " << k << " j " << j;
    }
    q = gen.QRef();
    v_prev = v;
    for (int j = 0; j < kNv; ++j) {
      const double over = std::max(q(j) - cfg.q_max(j), cfg.q_min(j) - q(j));
      worst_overshoot = std::max(worst_overshoot, over);
    }
  }
  RecordProperty("bound_conflict_ticks", std::to_string(conflicts));
  RecordProperty("worst_overshoot_rad", std::to_string(worst_overshoot));
  EXPECT_LT(worst_overshoot, kMargin) << "q left the true joint envelope";
  if (worst_overshoot > kSolverEps) {
    EXPECT_GT(conflicts, 0) << "overshoot without a reported conflict";
  }
}

// G5-B2 (CLIK side): a joint driven at speed toward its limit cannot stop
// within one tick; the acceleration bound wins, the conflict is reported with
// the joint's bit, and the step stays within a_max·dt.
TEST_F(ClikOptionsTest, AccelConflictKeepsAccelBoundAndReports) {
  constexpr double kAMax = 5.0;
  auto cfg = BaseConfig();
  cfg.q_min = model_->lowerPositionLimit;
  cfg.q_max = model_->upperPositionLimit;
  cfg.a_max = Eigen::VectorXd::Constant(kNv, kAMax);
  ClikReferenceGenerator gen;
  gen.Init(kNv, cfg);
  gen.SetTaskGain(Vec6::Zero());
  gen.SetPostureGains(50.0, 0.0);  // posture drives joint 0 hard toward q_max

  Eigen::VectorXd q_des = q_home_;
  q_des(0) = cfg.q_max(0) + 1.0;
  Eigen::VectorXd q = q_home_;
  q(0) = cfg.q_max(0) - 0.5;
  // Ramp up to speed, far from the limit.
  for (int k = 0; k < 200 && q(0) < cfg.q_max(0) - 0.02; ++k) {
    cache_.Update(q, v_zero_);
    ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, pinocchio::SE3::Identity(), q_des, kDt));
    q = gen.QRef();
  }
  ASSERT_GT(gen.VRef()(0), 0.5) << "joint 0 never got up to speed";

  bool saw_conflict = false;
  for (int k = 0; k < 400; ++k) {
    const double v_before = gen.VRef()(0);
    cache_.Update(q, v_zero_);
    ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, pinocchio::SE3::Identity(), q_des, kDt));
    ASSERT_LE(std::abs(gen.VRef()(0) - v_before), kAMax * kDt + kSolverEps) << k;
    if (gen.LastSolve().bound_conflict) {
      saw_conflict = true;
      EXPECT_TRUE(gen.LastSolve().conflict_mask & 1U) << "joint 0 bit";
    }
    q = gen.QRef();
  }
  EXPECT_TRUE(saw_conflict);
}

// ResetAnchor clears v_prev: after a fast run, the next step starts from rest.
TEST_F(ClikOptionsTest, ResetAnchorRestartsAccelerationFromRest) {
  constexpr double kAMax = 5.0;
  auto cfg = BaseConfig();
  cfg.a_max = Eigen::VectorXd::Constant(kNv, kAMax);
  ClikReferenceGenerator gen;
  gen.Init(kNv, cfg);
  gen.SetTaskGain(Vec6::Constant(20.0));
  const pinocchio::SE3 des = OffsetTarget(0.3);
  Eigen::VectorXd q = q_home_;
  for (int k = 0; k < 300; ++k) {
    cache_.Update(q, v_zero_);
    ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, kDt));
    q = gen.QRef();
  }
  ASSERT_GT(gen.VRef().cwiseAbs().maxCoeff(), 2.0 * kAMax * kDt);

  gen.ResetAnchor();
  cache_.Update(q_home_, v_zero_);
  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, kDt));
  EXPECT_LE(gen.VRef().cwiseAbs().maxCoeff(), kAMax * kDt + kSolverEps);
  EXPECT_FALSE(gen.LastSolve().bound_conflict);
  // Re-anchored to the measured state, not carried from the fast run.
  EXPECT_LE((gen.QRef() - q_home_).cwiseAbs().maxCoeff(), kAMax * kDt * kDt + 1e-12);
}

// ── Smoothing term ──────────────────────────────────────────────────────────

TEST_F(ClikOptionsTest, SmoothingRejectsInvalid) {
  for (const double bad :
       {-1e-3, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    auto cfg = BaseConfig();
    cfg.w_smooth = bad;
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << bad;
  }
}

// A target step: with smoothing on, the first-tick velocity jump shrinks, and
// the same stationary target is still reached (smoothing is a rate penalty,
// not a bias: at rest v_prev = v = 0).
TEST_F(ClikOptionsTest, SmoothingDampsTheStepAndKeepsTheFixedPoint) {
  const pinocchio::SE3 des = OffsetTarget(0.05);
  auto run = [&](double w_smooth, double* first_step) {
    auto cfg = BaseConfig();
    cfg.w_smooth = w_smooth;
    ClikReferenceGenerator gen;
    gen.Init(kNv, cfg);
    gen.SetTaskGain(Vec6::Constant(5.0));
    Eigen::VectorXd q = q_home_;
    for (int k = 0; k < 3000; ++k) {
      cache_.Update(q, v_zero_);
      EXPECT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, kDt));
      if (k == 0) {
        *first_step = gen.VRef().norm();
      }
      q = gen.QRef();
    }
    cache_.Update(q, v_zero_);
    return (TipInBase().translation() - des.translation()).norm();
  };
  double step_off = 0.0;
  double step_on = 0.0;
  const double err_off = run(0.0, &step_off);
  const double err_on = run(0.5, &step_on);
  EXPECT_LT(step_on, 0.5 * step_off);
  EXPECT_LT(err_off, 1e-3);
  EXPECT_LT(err_on, 1e-3);
}

}  // namespace
}  // namespace rtc::tsid
