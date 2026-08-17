/// @file test_clik_reference.cpp
/// @brief Stage C-1 ClikReferenceGenerator unit tests (sprint criteria ①-⑤):
///        ① TCP convergence, ② nullspace non-interference, ③ hand
///        decoupling, ④ singularity boundedness, ⑤ RT zero-alloc Compute.
///        Plus Init() config validation and Compute() precondition guards.

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/kinematics/clik_reference.hpp"
#include "rtc_tsid/kinematics/se3_error.hpp"

// ── TU-local alloc counter (rtc_mpc test_utils::AllocCounter pattern) ──────
// Counts global new/delete while armed; unarmed (fixture / gtest) traffic is
// ignored. Overrides live in this TU only.
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

// GCC 13+ -Wmismatched-new-delete fires on `new ... -> std::free` pairs it
// traces through these overrides; the pairing is correct (our operator new
// itself mallocs). Canonical suppression for global allocator overrides.
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

using Vec6 = Eigen::Matrix<double, 6, 1>;

// Panda fixture: 7 revolute arm + 2 prismatic finger joints → nq == nv == 9,
// which matches the generator's reduced-tree contract.
class ClikReferenceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.Build(*model_, config);
    ASSERT_EQ(robot_info_.nq, robot_info_.nv);
    ASSERT_EQ(robot_info_.nv, 9);

    ContactManagerConfig contact_cfg;
    contact_cfg.max_contacts = 0;
    cache_.Init(model_, rtc::tsid::ContactFrameIds(contact_cfg));
    contacts_.Init(0);

    tcp_idx_ = cache_.RegisterFrame("panda_hand", model_->getFrameId("panda_hand"));
    base_idx_ = cache_.RegisterFrame("panda_link0", model_->getFrameId("panda_link0"));
    ASSERT_GE(tcp_idx_, 0);
    ASSERT_GE(base_idx_, 0);

    q_home_.resize(9);
    q_home_ << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.02, 0.02;
    v_zero_ = Eigen::VectorXd::Zero(9);
  }

  [[nodiscard]] ClikReferenceGenerator MakeGenerator(double damping_sq, double v_limit) const {
    ClikReferenceGenerator gen;
    ClikReferenceGenerator::Config cfg;
    cfg.arm_v_idx = {0, 1, 2, 3, 4, 5, 6};
    cfg.hand_v_idx = {7, 8};
    cfg.damping_sq = damping_sq;
    cfg.v_limit = v_limit;
    gen.Init(robot_info_.nv, cfg);
    return gen;
  }

  [[nodiscard]] pinocchio::SE3 TipInBase() const {
    const auto& tip = cache_.registered_frames[static_cast<size_t>(tcp_idx_)];
    const auto& base = cache_.registered_frames[static_cast<size_t>(base_idx_)];
    return base.oMf.actInv(tip.oMf);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  PinocchioCache cache_;
  ContactState contacts_;
  int tcp_idx_{-1};
  int base_idx_{-1};
  Eigen::VectorXd q_home_;
  Eigen::VectorXd v_zero_;
};

// ── Init validation ────────────────────────────────────────────────────────

TEST_F(ClikReferenceTest, InitRejectsInvalidConfig) {
  ClikReferenceGenerator gen;
  ClikReferenceGenerator::Config cfg;
  cfg.arm_v_idx = {0, 1, 2};
  cfg.hand_v_idx = {3};
  cfg.damping_sq = 1e-6;

  EXPECT_THROW(gen.Init(0, cfg), std::runtime_error);  // nv ≤ 0

  ClikReferenceGenerator::Config empty_arm = cfg;
  empty_arm.arm_v_idx.clear();
  EXPECT_THROW(gen.Init(9, empty_arm), std::runtime_error);

  ClikReferenceGenerator::Config zero_damp = cfg;
  zero_damp.damping_sq = 0.0;
  EXPECT_THROW(gen.Init(9, zero_damp), std::runtime_error);

  ClikReferenceGenerator::Config out_of_range = cfg;
  out_of_range.hand_v_idx = {9};
  EXPECT_THROW(gen.Init(9, out_of_range), std::runtime_error);

  ClikReferenceGenerator::Config duplicated = cfg;
  duplicated.arm_v_idx = {0, 1, 1};
  EXPECT_THROW(gen.Init(9, duplicated), std::runtime_error);

  ClikReferenceGenerator::Config overlap = cfg;
  overlap.hand_v_idx = {2};  // also in arm_v_idx
  EXPECT_THROW(gen.Init(9, overlap), std::runtime_error);

  EXPECT_NO_THROW(gen.Init(9, cfg));
}

// ── Position box validation (NUM-7) ────────────────────────────────────────
//
// The size/symmetry checks that already existed are not a finiteness gate: the
// box assembly in Compute() is `lo = std::max(lo, (q_min-q)/dt)` /
// `hi = std::min(hi, (q_max-q)/dt)`, and std::max/std::min return their FIRST
// argument when the second is NaN. A non-finite bound therefore collapses to
// ±v_limit — the position bound silently disappears for that joint, the
// inverted-box guard (`lo > hi`) is blind because every NaN comparison is
// false, and Compute()'s own `allFinite()` output guard never fires because
// q_ref/v_ref stay finite and plausible. Measured with the gate reverted: a
// single NaN component leaves Compute() returning true with the box gone.
//
// Each case below is pinned separately so a reverted guard is attributable:
//   finiteness removed        → NaN + infinite cases red
//   finiteness → isnan only   → infinite case red alone
//   order check removed       → inverted case red alone
//   order check `>` → `>=`    → margin-clamped real-model case red alone
//
// Helper: a valid full-nv box around home, so each case mutates exactly one
// component away from a config that is otherwise known-good.
Eigen::VectorXd ValidBoxLow(const Eigen::VectorXd& q_home) {
  return q_home.array() - 0.1;
}

Eigen::VectorXd ValidBoxHigh(const Eigen::VectorXd& q_home) {
  return q_home.array() + 0.1;
}

TEST_F(ClikReferenceTest, InitRejectsNaNPositionBox) {
  ClikReferenceGenerator::Config cfg;
  cfg.arm_v_idx = {0, 1, 2, 3, 4, 5, 6};
  cfg.hand_v_idx = {7, 8};
  cfg.damping_sq = 1e-6;
  cfg.q_min = ValidBoxLow(q_home_);
  cfg.q_max = ValidBoxHigh(q_home_);

  // Sanity: the unmutated box is accepted, so each rejection below is caused by
  // the single NaN component and not by an unrelated defect in the fixture.
  {
    ClikReferenceGenerator gen;
    EXPECT_NO_THROW(gen.Init(robot_info_.nv, cfg));
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const int idx : {0, 8}) {  // an arm index and a hand index
    ClikReferenceGenerator::Config bad_min = cfg;
    bad_min.q_min(idx) = nan;
    ClikReferenceGenerator gen_min;
    EXPECT_THROW(gen_min.Init(robot_info_.nv, bad_min), std::runtime_error)
        << "NaN q_min at index " << idx << " must be rejected";

    ClikReferenceGenerator::Config bad_max = cfg;
    bad_max.q_max(idx) = nan;
    ClikReferenceGenerator gen_max;
    EXPECT_THROW(gen_max.Init(robot_info_.nv, bad_max), std::runtime_error)
        << "NaN q_max at index " << idx << " must be rejected";
  }
}

// ±inf in the *bound-disabling* direction (q_min = −inf, q_max = +inf) is the
// one a NaN-only guard would let through, and it is exactly the case that is
// only benign while the velocity box is on: with v_limit ≤ 0 the −inf reaches
// the QP as l = −inf. The sanctioned encoding for "no position bound" is an
// EMPTY box, which this test keeps distinguishable by asserting the empty box
// still passes.
TEST_F(ClikReferenceTest, InitRejectsInfinitePositionBox) {
  const double inf = std::numeric_limits<double>::infinity();
  ClikReferenceGenerator::Config cfg;
  cfg.arm_v_idx = {0, 1, 2, 3, 4, 5, 6};
  cfg.hand_v_idx = {7, 8};
  cfg.damping_sq = 1e-6;
  cfg.q_min = ValidBoxLow(q_home_);
  cfg.q_max = ValidBoxHigh(q_home_);

  ClikReferenceGenerator::Config unbounded_low = cfg;
  unbounded_low.q_min(3) = -inf;
  ClikReferenceGenerator gen_low;
  EXPECT_THROW(gen_low.Init(robot_info_.nv, unbounded_low), std::runtime_error);

  ClikReferenceGenerator::Config unbounded_high = cfg;
  unbounded_high.q_max(3) = inf;
  ClikReferenceGenerator gen_high;
  EXPECT_THROW(gen_high.Init(robot_info_.nv, unbounded_high), std::runtime_error);

  // The sanctioned way to say "no position bound" stays open.
  ClikReferenceGenerator::Config empty_box = cfg;
  empty_box.q_min = Eigen::VectorXd();
  empty_box.q_max = Eigen::VectorXd();
  ClikReferenceGenerator gen_empty;
  EXPECT_NO_THROW(gen_empty.Init(robot_info_.nv, empty_box));
}

// A strictly inverted box admits no velocity at all; the collapse guard in
// Compute() would then drive that joint at the full velocity limit every tick,
// forever, with no failure reported. Reject at Init instead.
TEST_F(ClikReferenceTest, InitRejectsInvertedPositionBox) {
  ClikReferenceGenerator::Config cfg;
  cfg.arm_v_idx = {0, 1, 2, 3, 4, 5, 6};
  cfg.hand_v_idx = {7, 8};
  cfg.damping_sq = 1e-6;
  cfg.q_min = ValidBoxLow(q_home_);
  cfg.q_max = ValidBoxHigh(q_home_);
  cfg.q_min(5) = cfg.q_max(5) + 1e-9;  // the smallest possible inversion

  ClikReferenceGenerator gen;
  EXPECT_THROW(gen.Init(robot_info_.nv, cfg), std::runtime_error);
}

// Equality must PASS, and the reason is a real asset rather than a synthetic
// vector: the production consumer (DemoWbcController::InitClik) builds the box
// as model.lowerPositionLimit + margin / model.upperPositionLimit − margin with
// a shipped margin of 0.02 rad, and the panda finger range [0, 0.04] lands on
// q_min == q_max exactly. Rejecting equality would take out that envelope, and
// the consumer only WARNs and silently falls back to its integrator path — so
// an over-strict gate degrades the position backbone instead of failing loudly.
TEST_F(ClikReferenceTest, InitAcceptsMarginClampedRealModelEnvelope) {
  constexpr double kPositionMargin = 0.02;  // integration.position_margin, shipped

  ClikReferenceGenerator::Config cfg;
  cfg.arm_v_idx = {0, 1, 2, 3, 4, 5, 6};
  cfg.hand_v_idx = {7, 8};
  cfg.damping_sq = 1e-6;
  cfg.q_min = model_->lowerPositionLimit.array() + kPositionMargin;
  cfg.q_max = model_->upperPositionLimit.array() - kPositionMargin;

  // The fixture must actually reach the equality case, else this test is
  // vacuous and would drift silently if the shipped URDF changed.
  ASSERT_TRUE((cfg.q_min.array() == cfg.q_max.array()).any())
      << "panda + margin " << kPositionMargin << " no longer produces a locked joint";
  ASSERT_TRUE((cfg.q_min.array() <= cfg.q_max.array()).all())
      << "fixture envelope is inverted — that is a separate case";

  ClikReferenceGenerator gen;
  EXPECT_NO_THROW(gen.Init(robot_info_.nv, cfg));
}

TEST_F(ClikReferenceTest, ComputePreconditionsReturnFalse) {
  auto gen = MakeGenerator(1e-6, 0.0);
  cache_.Update(q_home_, v_zero_);
  const pinocchio::SE3 des = TipInBase();

  // dt ≤ 0
  EXPECT_FALSE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, 0.0));
  // tcp frame index out of range
  EXPECT_FALSE(gen.Compute(cache_, 99, base_idx_, des, q_home_, 0.01));
  EXPECT_FALSE(gen.Compute(cache_, -1, base_idx_, des, q_home_, 0.01));
  // posture vector dimension mismatch
  const Eigen::VectorXd bad_posture = Eigen::VectorXd::Zero(10);
  EXPECT_FALSE(gen.Compute(cache_, tcp_idx_, base_idx_, des, bad_posture, 0.01));
  // uninitialized generator
  ClikReferenceGenerator raw;
  EXPECT_FALSE(raw.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, 0.01));
}

// ── ① TCP convergence ─────────────────────────────────────────────────────
// Closed loop: q ← QRef() each tick. Pose error must converge below
// 1e-3 m / 1e-3 rad against a reachable offset target. (Weighted-QP contract:
// the soft posture/damping weights inflate the effective L1 damping vs the
// legacy exact damped-inverse, so steady-state tracking lands at ~mm, not
// the original 1e-4 m.)
TEST_F(ClikReferenceTest, TcpConvergesToOffsetTarget) {
  auto gen = MakeGenerator(1e-6, 0.0);
  gen.SetTaskGain(Vec6::Constant(2.0));
  gen.SetPostureGains(0.1, 1.0);

  Eigen::VectorXd q = q_home_;
  cache_.Update(q, v_zero_);

  pinocchio::SE3 des = TipInBase();
  des.translation()(2) += 0.05;
  des.rotation() =
      des.rotation() * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();

  const double dt = 0.01;
  for (int k = 0; k < 2000; ++k) {
    cache_.Update(q, v_zero_);
    ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, dt));
    q = gen.QRef();
  }

  cache_.Update(q, v_zero_);
  const Vec6 err = ComputeTaskPoseError(TipInBase(), des);
  EXPECT_LT(err.head<3>().norm(), 1e-3);
  EXPECT_LT(err.tail<3>().norm(), 1e-3);
  EXPECT_LT(gen.TcpErrorNorm(), 2e-3);
}

// ── ② soft-priority posture suppression ───────────────────────────────────
// Weighted-QP contract (not exact nullspace projection): switching the arm
// posture objective on moves the joints, but because w_task ≫ w_arm the
// induced task-space disturbance J·Δv_ref stays well below the raw joint-space
// posture motion ‖Δv_ref‖ — the task is soft-prioritised over posture. (The
// legacy exact damped-inverse drove J·Δv to the O(μ²) damping leak ~1e-8; a
// single weighted QP leaks at O(w_arm/w_task) instead, by design.)
TEST_F(ClikReferenceTest, ArmPostureStaysInNullspace) {
  auto gen = MakeGenerator(1e-12, 0.0);
  gen.SetTaskGain(Vec6::Constant(1.0));

  cache_.Update(q_home_, v_zero_);
  pinocchio::SE3 des = TipInBase();
  des.translation()(0) += 0.1;

  gen.SetPostureGains(0.0, 0.0);
  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, 0.01));
  const Eigen::VectorXd v_no_posture = gen.VRef();

  Eigen::VectorXd q_posture_far = q_home_;
  q_posture_far.head(7).array() += 0.3;
  gen.SetPostureGains(5.0, 0.0);
  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_posture_far, 0.01));
  const Eigen::VectorXd v_with_posture = gen.VRef();

  // Posture must actually move the joints…
  const double posture_motion = (v_with_posture - v_no_posture).norm();
  EXPECT_GT(posture_motion, 1e-3);

  // …while the induced task-space disturbance stays soft-prioritised below the
  // raw posture motion (w_task ≫ w_arm). Not the exact-projection 1e-8.
  const auto& J = cache_.registered_frames[static_cast<size_t>(tcp_idx_)].J;
  const Vec6 task_vel_diff = J * (v_with_posture - v_no_posture);
  EXPECT_LT(task_vel_diff.norm(), posture_motion);
}

// ── ③ hand decoupling ─────────────────────────────────────────────────────
// TCP target changes barely touch the hand command and hand posture target
// changes barely touch the arm command. The QP H is block-diagonal between
// arm and hand (J_task has zero hand columns), so the optima are separable;
// the residual cross-talk (~1e-5) is the iterative solver's coupling, not the
// legacy exact decoupling — assert near-, not bitwise-, equality.
TEST_F(ClikReferenceTest, HandAndArmCommandsDecoupled) {
  auto gen = MakeGenerator(1e-6, 0.0);
  gen.SetTaskGain(Vec6::Constant(2.0));
  gen.SetPostureGains(0.5, 3.0);

  cache_.Update(q_home_, v_zero_);
  pinocchio::SE3 des_a = TipInBase();
  des_a.translation()(2) += 0.1;
  pinocchio::SE3 des_b = TipInBase();
  des_b.translation()(0) -= 0.1;

  Eigen::VectorXd q_posture = q_home_;
  q_posture.tail(2).array() += 0.01;  // non-zero hand posture error

  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des_a, q_posture, 0.01));
  const Eigen::VectorXd v_target_a = gen.VRef();
  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des_b, q_posture, 0.01));
  const Eigen::VectorXd v_target_b = gen.VRef();

  // Different TCP targets → (near-)identical hand command (and non-zero by Kh).
  EXPECT_GT((v_target_a.head(7) - v_target_b.head(7)).norm(), 1e-6);
  EXPECT_NEAR(v_target_a(7), v_target_b(7), 1e-4);
  EXPECT_NEAR(v_target_a(8), v_target_b(8), 1e-4);
  EXPECT_GT(v_target_a.tail(2).norm(), 0.0);

  // Different hand posture targets → identical arm command.
  Eigen::VectorXd q_posture_hand2 = q_posture;
  q_posture_hand2.tail(2).array() -= 0.02;
  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des_a, q_posture_hand2, 0.01));
  const Eigen::VectorXd v_hand2 = gen.VRef();
  EXPECT_LT((v_target_a.head(7) - v_hand2.head(7)).norm(), 1e-4);
  EXPECT_GT((v_target_a.tail(2) - v_hand2.tail(2)).norm(), 0.0);
}

// ── ④ singularity boundedness ─────────────────────────────────────────────
// Fully stretched panda (q = 0) is rank-deficient. The damped right-inverse
// keeps ‖v_arm‖ ≤ ‖Kx⊙e‖ / (2μ) and everything finite.
TEST_F(ClikReferenceTest, SingularConfigVelocityBounded) {
  const double damping_sq = 1e-4;  // μ = 1e-2
  auto gen = MakeGenerator(damping_sq, 0.0);
  gen.SetTaskGain(Vec6::Constant(2.0));
  gen.SetPostureGains(0.0, 0.0);

  const Eigen::VectorXd q_singular = Eigen::VectorXd::Zero(9);
  cache_.Update(q_singular, v_zero_);
  pinocchio::SE3 des = TipInBase();
  des.translation()(2) += 0.5;  // unreachable: already fully stretched

  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_singular, 0.01));
  const Eigen::VectorXd v_sing = gen.VRef();
  EXPECT_TRUE(v_sing.allFinite());
  EXPECT_TRUE(gen.QRef().allFinite());

  const Vec6 e = ComputeTaskPoseError(TipInBase(), des);
  const double bound = (2.0 * e).norm() / (2.0 * std::sqrt(damping_sq));
  EXPECT_LE(v_sing.norm(), bound * (1.0 + 1e-9));

  const double manip_singular = gen.Manipulability();
  EXPECT_GT(manip_singular, 0.0);
  EXPECT_TRUE(std::isfinite(manip_singular));

  // Manipulability must be lower at the singular config than at home.
  cache_.Update(q_home_, v_zero_);
  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, TipInBase(), q_home_, 0.01));
  EXPECT_LT(manip_singular, gen.Manipulability());
}

// Velocity box: with v_limit on, every joint obeys |v| ≤ v_limit to the QP's
// constraint-satisfaction tolerance (eps_abs ~1e-6). The legacy post-solve
// std::clamp was exact; the box is now enforced inside the QP, so allow an
// eps_abs-scale slack. The hard wire-level guarantee is the downstream
// ClampRange in WriteJointCommand.
TEST_F(ClikReferenceTest, VelocityLimitClampsPerJoint) {
  const double v_limit = 0.2;
  auto gen = MakeGenerator(1e-6, v_limit);
  gen.SetTaskGain(Vec6::Constant(50.0));  // deliberately aggressive
  gen.SetPostureGains(0.0, 50.0);

  cache_.Update(q_home_, v_zero_);
  pinocchio::SE3 des = TipInBase();
  des.translation()(2) += 0.3;
  Eigen::VectorXd q_posture = q_home_;
  q_posture.tail(2).array() += 0.02;

  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_posture, 0.01));
  EXPECT_LE(gen.VRef().cwiseAbs().maxCoeff(), v_limit + 1e-5);
}

// Position box, already-violating joint: the inverted-box collapse must stay
// velocity-bounded in BOTH directions and be symmetric between them.
//
// The consumer wires this box to the MARGIN-CLAMPED envelope (q_limit ∓
// position_margin, shipped 0.02 rad), so "already violating" is an ordinary
// pose parked inside the margin band, not a joint past its hard limit — and
// v_ref leaves as the device command velocity. The collapse lands on `hi` in
// both directions: that is +v_limit below q_min (bounded) but the raw
// (q_max − q)/dt above q_max, which grows without bound as dt shrinks. Only
// the upper case regresses when the re-clamp is removed; the lower case
// passes either way, and that asymmetry is the defect this pins.
TEST_F(ClikReferenceTest, PositionBoxRecoveryIsVelocityBoundedBothDirections) {
  constexpr double kVLimit = 0.2;
  constexpr double kDt = 0.01;
  constexpr double kHalfWidth = 0.1;   // envelope half-width around home
  constexpr double kViolation = 0.05;  // past the envelope → 5 rad/s if unclamped

  const Eigen::VectorXd q_min = q_home_.array() - kHalfWidth;
  const Eigen::VectorXd q_max = q_home_.array() + kHalfWidth;

  auto make_boxed = [&]() {
    ClikReferenceGenerator gen;
    ClikReferenceGenerator::Config cfg;
    cfg.arm_v_idx = {0, 1, 2, 3, 4, 5, 6};
    cfg.hand_v_idx = {7, 8};
    cfg.damping_sq = 1e-6;
    cfg.v_limit = kVLimit;
    cfg.q_min = q_min;
    cfg.q_max = q_max;
    gen.Init(robot_info_.nv, cfg);
    return gen;
  };

  // Drive joint 0 out of the envelope with every reference term zeroed (desired
  // pose = current, posture target = measured), so v_ref(0) is the box alone.
  auto solve_at = [&](const Eigen::VectorXd& q) {
    cache_.Update(q, v_zero_);
    auto gen = make_boxed();
    EXPECT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, TipInBase(), q, kDt));
    return gen.VRef()(0);
  };

  Eigen::VectorXd q_below = q_home_;
  q_below(0) -= kHalfWidth + kViolation;
  const double v_below = solve_at(q_below);

  Eigen::VectorXd q_above = q_home_;
  q_above(0) += kHalfWidth + kViolation;
  const double v_above = solve_at(q_above);

  // Recovery still runs at the full velocity limit — bounded, not disabled.
  EXPECT_LE(std::abs(v_below), kVLimit + 1e-5);
  EXPECT_LE(std::abs(v_above), kVLimit + 1e-5);
  EXPECT_NEAR(v_below, kVLimit, 1e-4);
  EXPECT_NEAR(v_above, -kVLimit, 1e-4);
  // Same violation on either side → same recovery speed, opposite sign.
  EXPECT_NEAR(v_below, -v_above, 1e-4);
}

// ── ⑤ RT zero-alloc Compute ───────────────────────────────────────────────
TEST_F(ClikReferenceTest, ComputeIsAllocationFree) {
  auto gen = MakeGenerator(1e-6, 1.5);
  gen.SetTaskGain(Vec6::Constant(2.0));
  gen.SetPostureGains(0.5, 1.0);

  cache_.Update(q_home_, v_zero_);
  pinocchio::SE3 des = TipInBase();
  des.translation()(2) += 0.05;

  // Warm-up: one untracked call materializes any lazy state.
  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, 0.002));

  int ok_count = 0;
  AllocCounter::Arm();
  for (int k = 0; k < 1000; ++k) {
    if (gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, 0.002)) {
      ++ok_count;
    }
  }
  AllocCounter::Disarm();

  EXPECT_EQ(ok_count, 1000);
  EXPECT_EQ(AllocCounter::alloc_count.load(), 0);
}

// ── carry-forward anchor (reseed_anchor=false) ─────────────────────────────
// With the measured state HELD FIXED (a robot that does not move, e.g. lagging
// or in pure self-hold), measured anchoring (reseed=true) re-derives the same
// one-step q_ref every tick → ‖q_ref − q_meas‖ stays at the single-step v·dt.
// Carry-forward (reseed=false) instead integrates from the previous desired, so
// q_ref accumulates away from the held measured state tick over tick.
TEST_F(ClikReferenceTest, CarryForwardAnchorAccumulatesFromPreviousDesired) {
  auto gen = MakeGenerator(1e-6, 0.0);  // anchor_drift_max defaults to 0 → clamp off
  gen.SetTaskGain(Vec6::Constant(1.0));
  gen.SetPostureGains(0.0, 0.0);

  cache_.Update(q_home_, v_zero_);  // measured held fixed throughout
  pinocchio::SE3 des = TipInBase();
  des.translation()(2) += 0.05;

  const double dt = 0.01;
  constexpr int kTicks = 50;

  // Measured anchoring: q_ref re-derived from the fixed measured state each tick.
  double reseed_drift = 0.0;
  for (int k = 0; k < kTicks; ++k) {
    ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, dt, /*reseed_anchor=*/true));
    reseed_drift = (gen.QRef() - q_home_).norm();
  }

  // Carry-forward: integrates from the previous desired (fresh generator so the
  // first call re-anchors to measured, then accumulates).
  auto gen_cf = MakeGenerator(1e-6, 0.0);
  gen_cf.SetTaskGain(Vec6::Constant(1.0));
  gen_cf.SetPostureGains(0.0, 0.0);
  double first_cf_drift = 0.0;
  double cf_drift = 0.0;
  for (int k = 0; k < kTicks; ++k) {
    ASSERT_TRUE(
        gen_cf.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, dt, /*reseed_anchor=*/false));
    if (k == 0) {
      first_cf_drift = (gen_cf.QRef() - q_home_).norm();
    }
    cf_drift = (gen_cf.QRef() - q_home_).norm();
  }

  // First carry-forward call still re-anchors to measured (anchor_initialized_
  // guard), so it matches the single-step measured drift.
  EXPECT_NEAR(first_cf_drift, reseed_drift, 1e-9);
  // After many ticks the carry-forward desired has accumulated far past the
  // fixed single-step measured re-anchor.
  EXPECT_GT(cf_drift, 5.0 * reseed_drift);
}

// anchor_drift_max bounds the carry-forward excursion from the measured state.
TEST_F(ClikReferenceTest, AnchorDriftClampBoundsCarryForward) {
  ClikReferenceGenerator gen;
  ClikReferenceGenerator::Config cfg;
  cfg.arm_v_idx = {0, 1, 2, 3, 4, 5, 6};
  cfg.hand_v_idx = {7, 8};
  cfg.damping_sq = 1e-6;
  cfg.v_limit = 0.0;
  constexpr double kDriftMax = 0.05;
  cfg.anchor_drift_max = kDriftMax;
  gen.Init(robot_info_.nv, cfg);
  gen.SetTaskGain(Vec6::Constant(1.0));
  gen.SetPostureGains(0.0, 0.0);

  cache_.Update(q_home_, v_zero_);  // measured held fixed
  pinocchio::SE3 des = TipInBase();
  des.translation()(2) += 0.2;  // large target → would drift far without the clamp

  const double dt = 0.01;
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(
        gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, dt, /*reseed_anchor=*/false));
    // Every per-joint |q_ref − q_meas| stays within the clamp (+ FP slack).
    const double max_excursion = (gen.QRef() - q_home_).cwiseAbs().maxCoeff();
    EXPECT_LE(max_excursion, kDriftMax + 1e-9);
  }
}

}  // namespace
}  // namespace rtc::tsid
