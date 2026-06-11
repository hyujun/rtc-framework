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

void operator delete(void* p) noexcept { std::free(p); }

void operator delete[](void* p) noexcept { std::free(p); }

void operator delete(void* p, std::size_t) noexcept { std::free(p); }

void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

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
    cache_.Init(model_, contact_cfg);
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

TEST_F(ClikReferenceTest, ComputePreconditionsReturnFalse) {
  auto gen = MakeGenerator(1e-6, 0.0);
  cache_.Update(q_home_, v_zero_, contacts_);
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
// 1e-4 m / 1e-3 rad against a reachable offset target.
TEST_F(ClikReferenceTest, TcpConvergesToOffsetTarget) {
  auto gen = MakeGenerator(1e-6, 0.0);
  gen.SetTaskGain(Vec6::Constant(2.0));
  gen.SetPostureGains(0.1, 1.0);

  Eigen::VectorXd q = q_home_;
  cache_.Update(q, v_zero_, contacts_);

  pinocchio::SE3 des = TipInBase();
  des.translation()(2) += 0.05;
  des.rotation() =
      des.rotation() * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();

  const double dt = 0.01;
  for (int k = 0; k < 2000; ++k) {
    cache_.Update(q, v_zero_, contacts_);
    ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, dt));
    q = gen.QRef();
  }

  cache_.Update(q, v_zero_, contacts_);
  const Vec6 err = ComputeSe3Error(TipInBase(), des);
  EXPECT_LT(err.head<3>().norm(), 1e-4);
  EXPECT_LT(err.tail<3>().norm(), 1e-3);
  EXPECT_LT(gen.TcpErrorNorm(), 2e-3);
}

// ── ② nullspace non-interference ──────────────────────────────────────────
// Switching the arm posture objective on must not change the task-space
// velocity J·v_ref beyond the damping leak (O(μ²/σ_min²); μ² = 1e-12 here).
TEST_F(ClikReferenceTest, ArmPostureStaysInNullspace) {
  auto gen = MakeGenerator(1e-12, 0.0);
  gen.SetTaskGain(Vec6::Constant(1.0));

  cache_.Update(q_home_, v_zero_, contacts_);
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
  EXPECT_GT((v_with_posture - v_no_posture).norm(), 1e-3);

  // …without disturbing the task-space velocity.
  const auto& J = cache_.registered_frames[static_cast<size_t>(tcp_idx_)].J;
  const Vec6 task_vel_diff = J * (v_with_posture - v_no_posture);
  EXPECT_LT(task_vel_diff.norm(), 1e-8);
}

// ── ③ hand decoupling ─────────────────────────────────────────────────────
// TCP target changes must not touch the hand command and hand posture
// target changes must not touch the arm command (bitwise identical).
TEST_F(ClikReferenceTest, HandAndArmCommandsDecoupled) {
  auto gen = MakeGenerator(1e-6, 0.0);
  gen.SetTaskGain(Vec6::Constant(2.0));
  gen.SetPostureGains(0.5, 3.0);

  cache_.Update(q_home_, v_zero_, contacts_);
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

  // Different TCP targets → identical hand command (and non-zero by Kh).
  EXPECT_GT((v_target_a.head(7) - v_target_b.head(7)).norm(), 1e-6);
  EXPECT_DOUBLE_EQ(v_target_a(7), v_target_b(7));
  EXPECT_DOUBLE_EQ(v_target_a(8), v_target_b(8));
  EXPECT_GT(v_target_a.tail(2).norm(), 0.0);

  // Different hand posture targets → identical arm command.
  Eigen::VectorXd q_posture_hand2 = q_posture;
  q_posture_hand2.tail(2).array() -= 0.02;
  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des_a, q_posture_hand2, 0.01));
  const Eigen::VectorXd v_hand2 = gen.VRef();
  EXPECT_EQ((v_target_a.head(7) - v_hand2.head(7)).norm(), 0.0);
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
  cache_.Update(q_singular, v_zero_, contacts_);
  pinocchio::SE3 des = TipInBase();
  des.translation()(2) += 0.5;  // unreachable: already fully stretched

  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_singular, 0.01));
  const Eigen::VectorXd v_sing = gen.VRef();
  EXPECT_TRUE(v_sing.allFinite());
  EXPECT_TRUE(gen.QRef().allFinite());

  const Vec6 e = ComputeSe3Error(TipInBase(), des);
  const double bound = (2.0 * e).norm() / (2.0 * std::sqrt(damping_sq));
  EXPECT_LE(v_sing.norm(), bound * (1.0 + 1e-9));

  const double manip_singular = gen.Manipulability();
  EXPECT_GT(manip_singular, 0.0);
  EXPECT_TRUE(std::isfinite(manip_singular));

  // Manipulability must be lower at the singular config than at home.
  cache_.Update(q_home_, v_zero_, contacts_);
  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, TipInBase(), q_home_, 0.01));
  EXPECT_LT(manip_singular, gen.Manipulability());
}

// Velocity clamp: with v_limit on, every joint obeys |v| ≤ v_limit.
TEST_F(ClikReferenceTest, VelocityLimitClampsPerJoint) {
  const double v_limit = 0.2;
  auto gen = MakeGenerator(1e-6, v_limit);
  gen.SetTaskGain(Vec6::Constant(50.0));  // deliberately aggressive
  gen.SetPostureGains(0.0, 50.0);

  cache_.Update(q_home_, v_zero_, contacts_);
  pinocchio::SE3 des = TipInBase();
  des.translation()(2) += 0.3;
  Eigen::VectorXd q_posture = q_home_;
  q_posture.tail(2).array() += 0.02;

  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_posture, 0.01));
  EXPECT_LE(gen.VRef().cwiseAbs().maxCoeff(), v_limit + 1e-12);
}

// ── ⑤ RT zero-alloc Compute ───────────────────────────────────────────────
TEST_F(ClikReferenceTest, ComputeIsAllocationFree) {
  auto gen = MakeGenerator(1e-6, 1.5);
  gen.SetTaskGain(Vec6::Constant(2.0));
  gen.SetPostureGains(0.5, 1.0);

  cache_.Update(q_home_, v_zero_, contacts_);
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

}  // namespace
}  // namespace rtc::tsid
