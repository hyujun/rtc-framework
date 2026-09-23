/// @file test_clik_accel_constraint.cpp
/// @brief ClikReferenceGenerator acceleration constraints kinematic / dynamic
///        (dynamic_catching decision K, S6-C2).
///
/// The oracle is independent of the rows the generator builds: pinocchio's own
/// RNEA (torque) and classical frame acceleration (task) on a FRESH Data, fed
/// the finite-difference acceleration (v_ref − v_prev)/dt the command implies.
/// Each bound is paired with a premise — the same scenario without the rows
/// breaks it — so a green run is not a scenario that never came near the
/// limit. "kBox ⇒ bit-identical" is test_clik_golden.cpp's job.

#include <gtest/gtest.h>

#include <algorithm>
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
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/kinematics/clik_reference.hpp"

// ── TU-local alloc counter (test_clik_options.cpp pattern) ──────────────────
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
constexpr int kArm = 7;
constexpr double kDt = 0.002;
constexpr double kEta = 0.8;
// The generator's own row tolerance is 1e-6 absolute + 1e-6 relative; the
// oracle recomputes the same quantity through a different path (RNEA /
// classical acceleration on a fresh Data), so allow a little more.
constexpr double kOracleTolRel = 1e-5;
// Rows are unit-norm, so the solver's eps_abs (1e-6) is a VELOCITY residual;
// on a task row J/dt that is ≈ 1e-6·‖J_r‖/dt ≈ 5e-4 m/s² against the 8 m/s²
// bound here, i.e. ≈ 6e-5 of it. The torque rows' M/dt carries the same
// residual to ≈ 1e-6 of their bound, which kOracleTolRel above covers.
constexpr double kKinematicTolRel = 2e-4;

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mode = ClikReferenceGenerator::AccelConstraint;

class ClikAccelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;
    ASSERT_EQ(model_->nv, kNv);
    oracle_data_ = std::make_unique<pinocchio::Data>(*model_);
    ContactManagerConfig contact_cfg;
    contact_cfg.max_contacts = 0;
    cache_.Init(model_, rtc::tsid::ContactFrameIds(contact_cfg));
    tcp_frame_ = model_->getFrameId("panda_hand");
    tcp_idx_ = cache_.RegisterFrame("panda_hand", tcp_frame_);
    ASSERT_GE(tcp_idx_, 0);
    q_home_.resize(kNv);
    q_home_ << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.02, 0.02;
    tau_max_ = model_->effortLimit;
    ASSERT_EQ(tau_max_.size(), kNv);
    for (int i = 0; i < kArm; ++i) {
      ASSERT_GT(tau_max_(i), 0.0) << "the Panda URDF carries effort limits";
    }
  }

  [[nodiscard]] static ClikReferenceGenerator::Config BaseConfig() {
    ClikReferenceGenerator::Config cfg;
    cfg.arm_v_idx = {0, 1, 2, 3, 4, 5, 6};
    cfg.hand_v_idx = {7, 8};
    cfg.damping_sq = 1e-4;
    cfg.v_limit = 1.5;
    // The fingers are locked, as the catching binding locks its hand: the
    // rows cover the arm, and the hand is commanded elsewhere.
    cfg.v_limit_per_joint = Eigen::VectorXd::Constant(kNv, 1.5);
    cfg.v_limit_per_joint(7) = 1e-9;
    cfg.v_limit_per_joint(8) = 1e-9;
    cfg.evaluate_at_command = true;
    return cfg;
  }

  [[nodiscard]] ClikReferenceGenerator::Config Dynamic() const {
    auto cfg = BaseConfig();
    cfg.accel_constraint = Mode::kDynamic;
    cfg.tau_max = tau_max_;
    cfg.eta_tau = kEta;
    return cfg;
  }

  [[nodiscard]] static ClikReferenceGenerator::Config Kinematic(double lin, double ang) {
    auto cfg = BaseConfig();
    cfg.accel_constraint = Mode::kKinematic;
    cfg.task_accel_max_linear = lin;
    cfg.task_accel_max_angular = ang;
    return cfg;
  }

  /// A far target with stiff gains: from rest the unconstrained solve jumps
  /// to the velocity limit in one tick.
  [[nodiscard]] pinocchio::SE3 FarTarget() {
    cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
    pinocchio::SE3 des = cache_.registered_frames[static_cast<size_t>(tcp_idx_)].oMf;
    des.translation() += Eigen::Vector3d(0.25, -0.2, 0.2);
    return des;
  }

  [[nodiscard]] ClikReferenceGenerator::PositionAxisTarget FarAxisTarget() {
    const pinocchio::SE3 des = FarTarget();
    ClikReferenceGenerator::PositionAxisTarget t;
    t.position = des.translation();
    t.axis = Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitX()) * des.rotation().col(2);
    return t;
  }

  [[nodiscard]] static ClikReferenceGenerator Make(const ClikReferenceGenerator::Config& cfg) {
    ClikReferenceGenerator gen;
    gen.Init(kNv, cfg);
    gen.SetTaskGain(Vec6::Constant(40.0));
    gen.SetAxisGain(40.0);
    gen.SetPostureGains(0.5, 0.0);
    return gen;
  }

  /// Oracle torque of the step v_prev → v at q: RNEA(q, v_prev, (v − v_prev)/dt).
  [[nodiscard]] Eigen::VectorXd OracleTorque(const Eigen::VectorXd& q,
                                             const Eigen::VectorXd& v_prev,
                                             const Eigen::VectorXd& v) const {
    const Eigen::VectorXd a = (v - v_prev) / kDt;
    return pinocchio::rnea(*model_, *oracle_data_, q, v_prev, a);
  }

  /// Oracle classical acceleration of the TCP frame (LOCAL_WORLD_ALIGNED).
  [[nodiscard]] Vec6 OracleTaskAccel(const Eigen::VectorXd& q, const Eigen::VectorXd& v_prev,
                                     const Eigen::VectorXd& v) const {
    const Eigen::VectorXd a = (v - v_prev) / kDt;
    pinocchio::forwardKinematics(*model_, *oracle_data_, q, v_prev, a);
    pinocchio::updateFramePlacements(*model_, *oracle_data_);
    return pinocchio::getFrameClassicalAcceleration(*model_, *oracle_data_, tcp_frame_,
                                                    pinocchio::LOCAL_WORLD_ALIGNED)
        .toVector();
  }

  /// Worst ratio |τ_i| / (η τ_max,i) over the arm, across `ticks` perfectly
  /// tracked steps toward the SE3 target (cache at the command state and
  /// velocity, D-6). Fails the test if a step fails.
  double WorstTorqueRatio(ClikReferenceGenerator& gen, const pinocchio::SE3& des, int ticks) {
    Eigen::VectorXd q = q_home_;
    Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(kNv);
    double worst = 0.0;
    for (int k = 0; k < ticks; ++k) {
      cache_.Update(q, v_prev);
      EXPECT_TRUE(gen.Compute(cache_, tcp_idx_, -1, des, q_home_, kDt)) << k;
      const Eigen::VectorXd v = gen.VRef();
      const Eigen::VectorXd tau = OracleTorque(q, v_prev, v);
      for (int i = 0; i < kArm; ++i) {
        worst = std::max(worst, std::abs(tau(i)) / (kEta * tau_max_(i)));
      }
      q = gen.QRef();
      v_prev = v;
    }
    return worst;
  }

  std::shared_ptr<const pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> oracle_data_;
  PinocchioCache cache_;
  pinocchio::FrameIndex tcp_frame_{0};
  int tcp_idx_{-1};
  Eigen::VectorXd q_home_;
  Eigen::VectorXd tau_max_;
};

// ── Init: exactly one form, and its own fields only ─────────────────────────

TEST_F(ClikAccelTest, InitRefusesFieldsOfAnotherForm) {
  {
    auto cfg = BaseConfig();  // kBox
    cfg.tau_max = tau_max_;
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << "box with tau_max";
  }
  {
    auto cfg = BaseConfig();
    cfg.task_accel_max_linear = 5.0;
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << "box with task_accel_max";
  }
  {
    auto cfg = BaseConfig();
    cfg.eta_tau = 0.5;  // a margin nobody would apply
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << "box with eta_tau";
  }
  {
    auto cfg = Kinematic(5.0, 10.0);
    cfg.eta_tau = 0.8;
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << "kinematic with eta_tau";
  }
  {
    auto cfg = Kinematic(5.0, 10.0);
    cfg.a_max = Eigen::VectorXd::Constant(kNv, 10.0);
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << "kinematic with a_max";
  }
  {
    auto cfg = Dynamic();
    cfg.a_max = Eigen::VectorXd::Constant(kNv, 10.0);
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << "dynamic with a_max";
  }
  {
    auto cfg = Dynamic();
    cfg.task_accel_max_angular = 1.0;
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << "dynamic with task_accel_max";
  }
}

TEST_F(ClikAccelTest, InitRefusesInvalidValues) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto& [lin, ang] :
       {std::pair{0.0, 10.0}, std::pair{5.0, -1.0}, std::pair{nan, 10.0}, std::pair{5.0, inf}}) {
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, Kinematic(lin, ang)), std::runtime_error) << lin << " " << ang;
  }
  for (const double eta : {0.0, -0.5, 1.5, nan}) {
    auto cfg = Dynamic();
    cfg.eta_tau = eta;
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << "eta " << eta;
  }
  {
    auto cfg = Dynamic();
    cfg.tau_max = Eigen::VectorXd::Constant(kNv - 1, 10.0);
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << "size";
  }
  {
    auto cfg = Dynamic();
    cfg.tau_max(3) = 0.0;  // an arm joint
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << "zero on the arm";
  }
  {
    auto cfg = Dynamic();
    cfg.tau_max(2) = nan;
    ClikReferenceGenerator gen;
    EXPECT_THROW(gen.Init(kNv, cfg), std::runtime_error) << "NaN";
  }
  {
    auto cfg = Dynamic();
    cfg.tau_max(7) = 0.0;  // a hand joint: never read
    cfg.tau_max(8) = 0.0;
    ClikReferenceGenerator gen;
    EXPECT_NO_THROW(gen.Init(kNv, cfg));
  }
}

// ── kDynamic ────────────────────────────────────────────────────────────────

TEST_F(ClikAccelTest, DynamicRowsKeepTheOracleTorqueInsideTheBound) {
  const pinocchio::SE3 des = FarTarget();
  // Premise: the same scenario without the rows demands more than η τ_max.
  auto free_gen = Make(BaseConfig());
  const double free_worst = WorstTorqueRatio(free_gen, des, 200);
  ASSERT_GT(free_worst, 1.5) << "premise: the unconstrained step exceeds the torque bound";

  auto gen = Make(Dynamic());
  const double worst = WorstTorqueRatio(gen, des, 200);
  EXPECT_LE(worst, 1.0 + kOracleTolRel) << "the torque rows did not hold";
  EXPECT_GT(worst, 0.9) << "the bound should be active in this scenario";
  EXPECT_EQ(gen.LastSolve().accel_rows, kArm);
  EXPECT_FALSE(gen.LastSolve().accel_rows_violated);
}

TEST_F(ClikAccelTest, DynamicRowsSlowTheApproachButKeepItsFixedPoint) {
  // The far target is not fully reachable under the posture term and the
  // velocity box — the unconstrained run settles a few cm short of it. The
  // rows must not change WHERE the loop settles, only how fast it gets there.
  const pinocchio::SE3 des = FarTarget();
  const auto settle = [&](const ClikReferenceGenerator::Config& cfg, int& binding_ticks) {
    auto gen = Make(cfg);
    Eigen::VectorXd q = q_home_;
    Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(kNv);
    binding_ticks = 0;
    for (int k = 0; k < 1500; ++k) {
      cache_.Update(q, v_prev);
      EXPECT_TRUE(gen.Compute(cache_, tcp_idx_, -1, des, q_home_, kDt)) << k;
      binding_ticks += gen.LastSolve().accel_rows_binding > 0 ? 1 : 0;
      q = gen.QRef();
      v_prev = gen.VRef();
    }
    cache_.Update(q, v_prev);
    return Eigen::Vector3d(
        cache_.registered_frames[static_cast<size_t>(tcp_idx_)].oMf.translation());
  };
  int free_binding = 0;
  int binding = 0;
  const Eigen::Vector3d p_free = settle(BaseConfig(), free_binding);
  const Eigen::Vector3d p_dyn = settle(Dynamic(), binding);
  EXPECT_LT((p_dyn - p_free).norm(), 1e-3) << "the rows moved the fixed point";
  EXPECT_EQ(free_binding, 0) << "no rows, no binding count";
  EXPECT_GT(binding, 0) << "the binding count never rose";
}

TEST_F(ClikAccelTest, RowsThatCannotHoldFailTheCallInsteadOfBreaking) {
  // Every arm joint velocity-locked (the box admits v ≈ 0 only) and a torque
  // bound below the gravity load: τ ≈ h, outside ±η τ_max. The call must fail
  // rather than return a command that breaks the rows.
  cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
  const Eigen::VectorXd h = cache_.h;
  int loaded = -1;
  for (int i = 0; i < kArm; ++i) {
    if (std::abs(h(i)) > 2.0) {
      loaded = i;
      break;
    }
  }
  ASSERT_GE(loaded, 0) << "premise: some arm joint carries a gravity load at home";

  auto cfg = Dynamic();
  cfg.v_limit_per_joint = Eigen::VectorXd::Constant(kNv, 1.5);
  for (int i = 0; i < kArm; ++i) {
    cfg.v_limit_per_joint(i) = 1e-9;
  }
  cfg.tau_max(loaded) = 0.5 * std::abs(h(loaded)) / kEta;  // η τ_max = |h|/2
  auto gen = Make(cfg);
  EXPECT_FALSE(gen.Compute(cache_, tcp_idx_, -1, FarTarget(), q_home_, kDt));
  const auto& s = gen.LastSolve();
  EXPECT_TRUE(s.accel_rows_violated || !s.converged)
      << "status " << s.status << " — a failed call must say why";
  EXPECT_TRUE(gen.VRef().isZero()) << "the failure branch commands v_ref = 0";
}

// ── kKinematic ──────────────────────────────────────────────────────────────

TEST_F(ClikAccelTest, KinematicRowsBoundTheTaskAccelerationSe3) {
  constexpr double kLin = 8.0;
  constexpr double kAng = 20.0;
  const pinocchio::SE3 des = FarTarget();
  const auto run = [&](const ClikReferenceGenerator::Config& cfg, double& lin_ratio,
                       double& ang_ratio) {
    auto gen = Make(cfg);
    Eigen::VectorXd q = q_home_;
    Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(kNv);
    lin_ratio = 0.0;
    ang_ratio = 0.0;
    for (int k = 0; k < 200; ++k) {
      cache_.Update(q, v_prev);
      ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, -1, des, q_home_, kDt)) << k;
      const Vec6 a = OracleTaskAccel(q, v_prev, gen.VRef());
      for (int r = 0; r < 3; ++r) {
        lin_ratio = std::max(lin_ratio, std::abs(a(r)) / kLin);
        ang_ratio = std::max(ang_ratio, std::abs(a(3 + r)) / kAng);
      }
      q = gen.QRef();
      v_prev = gen.VRef();
    }
  };
  double free_lin = 0.0;
  double free_ang = 0.0;
  run(BaseConfig(), free_lin, free_ang);
  ASSERT_GT(free_lin, 1.5) << "premise: the unconstrained step exceeds the task bound";

  double lin = 0.0;
  double ang = 0.0;
  run(Kinematic(kLin, kAng), lin, ang);
  EXPECT_LE(lin, 1.0 + kKinematicTolRel);
  EXPECT_LE(ang, 1.0 + kKinematicTolRel);
  EXPECT_GT(lin, 0.9) << "the linear bound should be active in this scenario";
}

TEST_F(ClikAccelTest, KinematicRowsBoundTheTaskAccelerationPositionAxis) {
  // The position + axis overload: 3 world-aligned linear rows and the two
  // LOCAL x, y angular rows (the approach-axis rows the cost tracks).
  constexpr double kLin = 8.0;
  constexpr double kAng = 20.0;
  const auto t = FarAxisTarget();
  auto gen = Make(Kinematic(kLin, kAng));
  Eigen::VectorXd q = q_home_;
  Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(kNv);
  double lin = 0.0;
  double ang = 0.0;
  for (int k = 0; k < 200; ++k) {
    cache_.Update(q, v_prev);
    ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, -1, t, q_home_, kDt)) << k;
    EXPECT_EQ(gen.LastSolve().accel_rows, 5);
    const Vec6 a = OracleTaskAccel(q, v_prev, gen.VRef());
    const Eigen::Matrix3d R =
        cache_.registered_frames[static_cast<size_t>(tcp_idx_)].oMf.rotation();
    const Eigen::Vector3d w_local = R.transpose() * a.tail<3>();
    for (int r = 0; r < 3; ++r) {
      lin = std::max(lin, std::abs(a(r)) / kLin);
    }
    for (int r = 0; r < 2; ++r) {
      ang = std::max(ang, std::abs(w_local(r)) / kAng);
    }
    q = gen.QRef();
    v_prev = gen.VRef();
  }
  EXPECT_LE(lin, 1.0 + kKinematicTolRel);
  EXPECT_LE(ang, 1.0 + kKinematicTolRel);
  EXPECT_GT(lin, 0.9);
}

// ── RT: both forms, both overloads — no heap allocation ────────────────────

TEST_F(ClikAccelTest, ComputeIsAllocationFreeInBothForms) {
  const pinocchio::SE3 des = FarTarget();
  const auto t = FarAxisTarget();
  for (const auto& cfg : {Dynamic(), Kinematic(8.0, 20.0)}) {
    auto gen = Make(cfg);
    Eigen::VectorXd q = q_home_;
    Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(kNv);
    cache_.Update(q, v_prev);
    ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, -1, des, q_home_, kDt));  // warm-up
    q = gen.QRef();
    v_prev = gen.VRef();
    // One block per overload (alternating two targets 0.6 rad apart every tick
    // is a torque demand no bound admits, and a failed call is not what this
    // case measures — the failure path is covered below).
    int ok = 0;
    AllocCounter::Arm();
    for (int k = 0; k < 500; ++k) {
      cache_.Update(q, v_prev);
      const bool r = (k < 250) ? gen.Compute(cache_, tcp_idx_, -1, des, q_home_, kDt)
                               : gen.Compute(cache_, tcp_idx_, -1, t, q_home_, kDt);
      ok += r ? 1 : 0;
      q = gen.QRef();
      v_prev = gen.VRef();
    }
    AllocCounter::Disarm();
    EXPECT_EQ(ok, 500) << "status " << gen.LastSolve().status;
    EXPECT_EQ(AllocCounter::alloc_count.load(), 0);
  }
}

TEST_F(ClikAccelTest, TheFailurePathIsAllocationFreeToo) {
  // The infeasible case of RowsThatCannotHoldFailTheCallInsteadOfBreaking,
  // repeated: failure branch + warm-start reset, no allocation.
  cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
  const Eigen::VectorXd h = cache_.h;
  int loaded = -1;
  for (int i = 0; i < kArm; ++i) {
    if (std::abs(h(i)) > 2.0) {
      loaded = i;
      break;
    }
  }
  ASSERT_GE(loaded, 0);
  auto cfg = Dynamic();
  cfg.v_limit_per_joint = Eigen::VectorXd::Constant(kNv, 1.5);
  for (int i = 0; i < kArm; ++i) {
    cfg.v_limit_per_joint(i) = 1e-9;
  }
  cfg.tau_max(loaded) = 0.5 * std::abs(h(loaded)) / kEta;
  auto gen = Make(cfg);
  const pinocchio::SE3 des = FarTarget();
  cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
  int failed = 0;
  AllocCounter::Arm();
  for (int k = 0; k < 50; ++k) {
    failed += gen.Compute(cache_, tcp_idx_, -1, des, q_home_, kDt) ? 0 : 1;
  }
  AllocCounter::Disarm();
  EXPECT_EQ(failed, 50);
  EXPECT_EQ(AllocCounter::alloc_count.load(), 0);
}

}  // namespace
}  // namespace rtc::tsid
