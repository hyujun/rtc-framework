/// @file test_clik_multiframe.cpp
/// @brief ClikReferenceGenerator's multi-frame Compute() overload (E2-F04, #636).
///
/// Three things are judged, each against something the overload did not
/// produce:
///
///   1. One task is the old problem. A single universe-base task must give the
///      single-task overloads' output BIT FOR BIT — on the golden table's
///      universe scenarios (so, transitively, the recorded table), and on
///      closed-loop runs with every option, on a 9-DoF and a 6-DoF model (the
///      cost products take a different Eigen kernel below 20 = rows + 2·nv).
///   2. The relative task is the motion of one frame seen from another. Its
///      Jacobian is checked against central differences of the relative pose on
///      a fresh pinocchio::Data, and the whole solve against a closed-form
///      weighted least-squares solution built from those differences.
///   3. The options hold what they say: the stacked acceleration rows against
///      pinocchio's RNEA / classical acceleration, the feedback cap against an
///      equivalent uncapped problem, the input checks against untouched
///      outputs.
///
/// The registered-base scenarios of the golden table are NOT replayed: the SE3
/// overload pairs a base-aligned error with the world-aligned Jacobian there,
/// and the multi-frame overload deliberately does not.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <numbers>
#include <random>
#include <span>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/explog.hpp>
#pragma GCC diagnostic pop

#include "alloc_counter.hpp"
#include "clik_golden_scenarios.hpp"
#include "rtc_tsid/kinematics/clik_reference.hpp"
#include "rtc_tsid/kinematics/relative_jacobian.hpp"
#include "rtc_tsid/kinematics/se3_error.hpp"
#include "test_urdf_path.hpp"
#include "tree_fixture.hpp"

namespace rtc::tsid {
namespace {

using Clik = ClikReferenceGenerator;
using Mode = Clik::AccelConstraint;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6X = Eigen::Matrix<double, 6, Eigen::Dynamic>;

[[nodiscard]] std::uint64_t Bits(double x) {
  std::uint64_t b = 0;
  std::memcpy(&b, &x, sizeof(b));
  return b;
}

/// A double as a test property: std::to_string keeps six decimals, which
/// prints every residual here as 0.000000.
[[nodiscard]] std::string Sci(double x) {
  std::array<char, 32> buf{};
  std::snprintf(buf.data(), buf.size(), "%.3e", x);
  return buf.data();
}

[[nodiscard]] ::testing::AssertionResult BitEqual(const Eigen::VectorXd& a,
                                                  const Eigen::VectorXd& b) {
  if (a.size() != b.size()) {
    return ::testing::AssertionFailure() << "sizes " << a.size() << " vs " << b.size();
  }
  for (Eigen::Index i = 0; i < a.size(); ++i) {
    if (Bits(a(i)) != Bits(b(i))) {
      return ::testing::AssertionFailure()
             << "index " << i << ": " << std::hexfloat << a(i) << " vs " << b(i);
    }
  }
  return ::testing::AssertionSuccess();
}

[[nodiscard]] PinocchioCache MakeCache(const std::shared_ptr<const pinocchio::Model>& model) {
  PinocchioCache cache;
  ContactManagerConfig contact_cfg;
  contact_cfg.max_contacts = 0;
  cache.Init(model, rtc::tsid::ContactFrameIds(contact_cfg));
  return cache;
}

[[nodiscard]] Clik::MultiFrameInput Input(std::span<const Clik::FrameTask> tasks,
                                          const Eigen::VectorXd& q_posture, double dt) {
  Clik::MultiFrameInput in;
  in.tasks = tasks;
  in.q_posture_des = &q_posture;
  in.dt = dt;
  return in;
}

// ── 1a. The golden table's universe scenarios ───────────────────────────────

class ClikMultiFrameGoldenTest : public test::clik_golden::ClikGoldenScenarioTest {};

TEST_F(ClikMultiFrameGoldenTest, OneUniverseSe3TaskReplaysTheGoldenScenariosBitForBit) {
  using test::clik_golden::kRecordWidth;
  using test::clik_golden::kScenarios;
  using test::clik_golden::Tick;
  int replayed = 0;
  for (int s = 0; s < static_cast<int>(std::size(kScenarios)); ++s) {
    if (kScenarios[s].registered_base) {
      continue;  // see the file header
    }
    const std::vector<double> single = RunScenario(s, [](Clik& gen, const Tick& t) {
      return gen.Compute(t.cache, t.tcp_idx, t.base_idx, t.des, t.q_posture, t.dt, t.reseed);
    });
    const std::vector<double> multi = RunScenario(s, [](Clik& gen, const Tick& t) {
      Clik::FrameTask task;
      task.kind = Clik::TaskKind::kSe3;
      task.frame_idx = t.tcp_idx;
      task.placement_des = t.des;
      task.gain = Vec6::Constant(test::clik_golden::kTaskGain);
      Clik::MultiFrameInput in =
          Input(std::span<const Clik::FrameTask>(&task, 1), t.q_posture, t.dt);
      in.reseed_anchor = t.reseed;
      return gen.Compute(t.cache, in);
    });
    ASSERT_EQ(single.size(), multi.size());
    size_t mismatches = 0;
    for (size_t i = 0; i < single.size(); ++i) {
      if (Bits(single[i]) != Bits(multi[i]) && ++mismatches <= 5) {
        ADD_FAILURE() << kScenarios[s].name << " value " << i << " (tick " << i / kRecordWidth
                      << ", field " << i % kRecordWidth << "): " << std::hexfloat << multi[i]
                      << " vs " << single[i];
      }
    }
    EXPECT_EQ(mismatches, 0U) << kScenarios[s].name;
    // Not a vacuous match: the scenario's failure tick is in the replay.
    const size_t fail_tick = (s == 1) ? 25U : 12U;
    EXPECT_EQ(multi[fail_tick * kRecordWidth], 0.0) << kScenarios[s].name;
    ++replayed;
  }
  EXPECT_EQ(replayed, 2) << "carry_forward_drift and no_position_box";
}

// ── 1b. One task against the single-task overloads, closed loop ─────────────

struct ArmModel {
  const char* name;
  std::shared_ptr<const pinocchio::Model> model;
  std::string tip;
  std::vector<int> arm;
  std::vector<int> hand;
  Eigen::VectorXd q_home;
};

[[nodiscard]] ArmModel Arm9() {
  ArmModel m;
  m.name = "9dof";
  m.model = test::SharedPandaModel();
  m.tip = "panda_hand";
  m.arm = {0, 1, 2, 3, 4, 5, 6};
  m.hand = {7, 8};
  m.q_home.resize(9);
  m.q_home << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.02, 0.02;
  return m;
}

[[nodiscard]] ArmModel Arm6() {
  ArmModel m;
  m.name = "6dof";
  auto model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(rtc::test::TestUrdfPath("serial_6dof.urdf"), *model);
  m.model = model;
  m.tip = "tool_link";
  m.arm = {0, 1, 2, 3, 4, 5};
  m.q_home.resize(6);
  m.q_home << 0.3, -0.8, 1.2, 0.4, 0.9, -0.2;
  return m;
}

constexpr int kVariants = 4;
constexpr const char* kVariantNames[kVariants] = {"plain", "boxes_smooth_accel_box", "kinematic",
                                                  "dynamic"};

[[nodiscard]] Clik::Config VariantConfig(const ArmModel& m, int variant) {
  Clik::Config cfg;
  cfg.arm_v_idx = m.arm;
  cfg.hand_v_idx = m.hand;
  cfg.damping_sq = 1e-4;
  cfg.v_limit = 1.5;
  cfg.w_task = 2.0;  // not the defaults: a weight wired to the wrong field shows
  cfg.w_axis = 0.7;
  const int nv = m.model->nv;
  switch (variant) {
    case 0:
      break;
    case 1:
      cfg.q_min = m.model->lowerPositionLimit;
      cfg.q_max = m.model->upperPositionLimit;
      cfg.v_limit_per_joint = m.model->velocityLimit;
      cfg.w_smooth = 1e-3;
      cfg.anchor_drift_max = 0.05;
      cfg.a_max = Eigen::VectorXd::Constant(nv, 20.0);
      break;
    case 2:
      cfg.q_min = m.model->lowerPositionLimit;
      cfg.q_max = m.model->upperPositionLimit;
      cfg.evaluate_at_command = true;
      cfg.accel_constraint = Mode::kKinematic;
      cfg.task_accel_max_linear = 8.0;
      cfg.task_accel_max_angular = 20.0;
      break;
    default:
      cfg.q_min = m.model->lowerPositionLimit;
      cfg.q_max = m.model->upperPositionLimit;
      cfg.evaluate_at_command = true;
      cfg.accel_constraint = Mode::kDynamic;
      cfg.tau_max = m.model->effortLimit;
      cfg.eta_tau = 0.8;
      break;
  }
  return cfg;
}

void ExpectSameCall(const Clik& single, const Clik& multi, bool axis_task, const std::string& at) {
  EXPECT_TRUE(BitEqual(single.QRef(), multi.QRef())) << "q_ref " << at;
  EXPECT_TRUE(BitEqual(single.VRef(), multi.VRef())) << "v_ref " << at;
  EXPECT_EQ(Bits(single.Manipulability()), Bits(multi.Manipulability())) << at;
  EXPECT_EQ(Bits(single.TcpErrorNorm()), Bits(multi.TcpErrorNorm())) << at;
  if (axis_task) {
    EXPECT_EQ(Bits(single.PositionErrorNorm()), Bits(multi.PositionErrorNorm())) << at;
    EXPECT_EQ(Bits(single.AxisErrorAngle()), Bits(multi.AxisErrorAngle())) << at;
    EXPECT_EQ(single.LastAxisRegion(), multi.LastAxisRegion()) << at;
  }
  const Clik::SolveDiagnostics& a = single.LastSolve();
  const Clik::SolveDiagnostics& b = multi.LastSolve();
  EXPECT_EQ(a.reached_solve, b.reached_solve) << at;
  EXPECT_EQ(a.converged, b.converged) << at;
  EXPECT_EQ(a.non_finite, b.non_finite) << at;
  EXPECT_EQ(a.status, b.status) << at;
  EXPECT_EQ(a.iterations, b.iterations) << at;
  EXPECT_EQ(a.command_mismatch, b.command_mismatch) << at;
  EXPECT_EQ(a.bound_conflict, b.bound_conflict) << at;
  EXPECT_EQ(a.conflict_mask, b.conflict_mask) << at;
  EXPECT_EQ(a.accel_rows, b.accel_rows) << at;
  EXPECT_EQ(a.accel_rows_binding, b.accel_rows_binding) << at;
  EXPECT_EQ(a.accel_rows_violated, b.accel_rows_violated) << at;
}

/// Runs the two generators side by side for `ticks` on a target that follows
/// the frame of a slowly moving joint posture, and returns how many ticks
/// succeeded. The loop closes on the single-task generator's output; the two
/// must agree bit for bit on every tick, so it does not matter which.
int RunParity(const ArmModel& m, int variant, bool axis_task, int ticks) {
  const int nv = m.model->nv;
  const double dt = 0.002;
  const Clik::Config cfg = VariantConfig(m, variant);
  PinocchioCache cache = MakeCache(m.model);
  const pinocchio::FrameIndex tip_frame = m.model->getFrameId(m.tip);
  const int tip = cache.RegisterFrame(m.tip, tip_frame);
  pinocchio::Data target_data(*m.model);

  Vec6 gain;
  gain << 12.0, 12.0, 12.0, 8.0, 8.0, 8.0;
  const double gain_axis = 9.0;
  Clik single;
  Clik multi;
  single.Init(nv, cfg);
  multi.Init(nv, cfg);
  for (Clik* gen : {&single, &multi}) {
    gen->SetTaskGain(gain);
    gen->SetAxisGain(gain_axis);
    gen->SetPostureGains(0.5, 2.0);
  }

  std::mt19937 rng(static_cast<std::uint32_t>(977 * (variant + 1) + nv + (axis_task ? 1 : 0)));
  std::uniform_real_distribution<double> uni(-1.0, 1.0);
  const std::string label =
      std::string(m.name) + "/" + kVariantNames[variant] + (axis_task ? "/axis" : "/se3");

  Eigen::VectorXd q = m.q_home;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd q_target = m.q_home;
  Eigen::VectorXd qd_ff(nv);
  int ok_ticks = 0;
  for (int k = 0; k < ticks; ++k) {
    for (const int j : m.arm) {
      q_target(j) = m.q_home(j) + 0.2 * std::sin(0.04 * k + 0.9 * j);
    }
    pinocchio::forwardKinematics(*m.model, target_data, q_target);
    pinocchio::updateFramePlacements(*m.model, target_data);
    pinocchio::SE3 des = target_data.oMf[tip_frame];
    Vec6 twist_ff;
    for (int i = 0; i < 6; ++i) {
      twist_ff(i) = 0.05 * uni(rng);
    }
    for (int i = 0; i < nv; ++i) {
      qd_ff(i) = 0.03 * uni(rng);
    }
    const bool with_ff = (k % 3) != 0;
    const bool with_qd_ff = (k % 2) != 0;

    cache.Update(q, v);
    bool ok_single = false;
    bool ok_multi = false;
    Clik::FrameTask task;
    task.frame_idx = tip;
    task.gain = gain;
    task.gain_axis = gain_axis;
    task.weight = cfg.w_task;
    task.weight_axis = cfg.w_axis;
    Clik::MultiFrameInput in = Input(std::span<const Clik::FrameTask>(&task, 1), m.q_home, dt);
    in.reseed_anchor = (k % 7) == 0;
    if (!axis_task) {
      if (k == 40) {
        des.translation()(1) = std::numeric_limits<double>::quiet_NaN();  // the failure branch
      }
      task.kind = Clik::TaskKind::kSe3;
      task.placement_des = des;
      task.twist_ff = with_ff ? &twist_ff : nullptr;
      ok_single = single.Compute(cache, tip, -1, des, m.q_home, dt, in.reseed_anchor,
                                 with_ff ? &twist_ff : nullptr);
    } else {
      Clik::PositionAxisTarget target;
      target.position = des.translation();
      target.axis = des.rotation().col(2);
      if (with_ff) {
        target.linear_velocity_ff = twist_ff.head<3>();
        target.angular_velocity_ff = twist_ff.tail<3>();
      }
      if (k == 41) {
        target.axis *= 2.0;  // not unit: refused before the solve
      }
      if (k == 43) {
        target.position(2) = std::numeric_limits<double>::quiet_NaN();  // refused too
      }
      task.kind = Clik::TaskKind::kPositionAxis;
      task.target = target;
      in.qd_posture_ff = with_qd_ff ? &qd_ff : nullptr;
      ok_single = single.Compute(cache, tip, -1, target, m.q_home, dt, in.reseed_anchor,
                                 with_qd_ff ? &qd_ff : nullptr);
    }
    ok_multi = multi.Compute(cache, in);

    const std::string at = label + " tick " + std::to_string(k);
    EXPECT_EQ(ok_single, ok_multi) << at;
    ExpectSameCall(single, multi, axis_task, at);
    if (ok_multi) {
      EXPECT_EQ(multi.LastSolve().tasks, 1) << at;
      ++ok_ticks;
    }
    if (::testing::Test::HasFailure()) {
      return ok_ticks;  // one tick's report is enough
    }

    // Close the loop. In command mode the next state IS the command; otherwise
    // a noisy measurement of it.
    q = single.QRef();
    v = single.VRef();
    if (!cfg.evaluate_at_command) {
      for (int i = 0; i < nv; ++i) {
        q(i) += 2e-4 * uni(rng);
        v(i) += 2e-3 * uni(rng);
      }
    }
  }
  return ok_ticks;
}

class ClikMultiFrameParityTest : public ::testing::TestWithParam<int> {};

TEST_P(ClikMultiFrameParityTest, OneUniverseTaskIsBitIdenticalToTheSingleTaskOverloads) {
  const int variant = GetParam();
  constexpr int kTicks = 100;
  for (const ArmModel& m : {Arm9(), Arm6()}) {
    for (const bool axis_task : {false, true}) {
      const int ok_ticks = RunParity(m, variant, axis_task, kTicks);
      ASSERT_FALSE(HasFailure()) << m.name << (axis_task ? " axis" : " se3");
      // The injected failures are 1 (SE3) or 2 (axis); everything else solves,
      // so the comparison above is of solved ticks, not of two failures.
      EXPECT_GE(ok_ticks, kTicks - 2) << m.name << (axis_task ? " axis" : " se3");
    }
  }
}

INSTANTIATE_TEST_SUITE_P(Variants, ClikMultiFrameParityTest, ::testing::Range(0, kVariants),
                         [](const ::testing::TestParamInfo<int>& info) {
                           return std::string(kVariantNames[info.param]);
                         });

// ── 2 and 3. The tree: relative tasks, stacked rows, options ────────────────

class ClikTreeTest : public ::testing::Test {
 protected:
  static constexpr int kNv = test::kTreeNv;
  static constexpr double kDt = 0.002;

  void SetUp() override {
    model_ = test::LoadTreeModel();
    ASSERT_EQ(model_->nv, kNv);
    ASSERT_EQ(model_->nq, kNv);
    oracle_data_ = std::make_unique<pinocchio::Data>(*model_);
    cache_ = MakeCache(model_);
    frame_a_ = model_->getFrameId("tip_a");
    frame_b_ = model_->getFrameId("tip_b");
    frame_torso_ = model_->getFrameId("torso");
    tip_a_ = cache_.RegisterFrame("tip_a", frame_a_);
    tip_b_ = cache_.RegisterFrame("tip_b", frame_b_);
    torso_ = cache_.RegisterFrame("torso", frame_torso_);
    ASSERT_GE(tip_a_, 0);
    ASSERT_GE(tip_b_, 0);
    ASSERT_GE(torso_, 0);
    trunk_ = test::VelocityIndices(*model_, "trunk_");
    arm_a_ = test::VelocityIndices(*model_, "arm_a_");
    arm_b_ = test::VelocityIndices(*model_, "arm_b_");
    ASSERT_EQ(trunk_.size(), 3U);
    ASSERT_EQ(arm_a_.size(), 7U);
    ASSERT_EQ(arm_b_.size(), 7U);
    for (int i = 0; i < kNv; ++i) {
      all_.push_back(i);
    }
    q_home_ = test::TreeHomePosture(*model_);
  }

  /// Every joint in the solve, three posture groups, no box: the unconstrained
  /// problem the closed-form oracle solves.
  [[nodiscard]] Clik::Config BaseConfig() const {
    Clik::Config cfg;
    cfg.arm_v_idx = all_;
    cfg.damping_sq = 1e-6;
    cfg.v_limit = 0.0;  // off
    cfg.max_frame_tasks = 2;
    cfg.relative_tasks = true;
    cfg.posture_groups = {{trunk_, 3e-2}, {arm_a_, 1e-2}, {arm_b_, 2e-2}};
    return cfg;
  }

  /// BaseConfig() with the model's boxes, in command mode — what the
  /// acceleration rows need.
  [[nodiscard]] Clik::Config BoxedConfig() const {
    Clik::Config cfg = BaseConfig();
    cfg.q_min = model_->lowerPositionLimit;
    cfg.q_max = model_->upperPositionLimit;
    cfg.v_limit_per_joint = model_->velocityLimit;
    cfg.evaluate_at_command = true;
    return cfg;
  }

  [[nodiscard]] pinocchio::SE3 OraclePose(const Eigen::VectorXd& q, pinocchio::FrameIndex frame,
                                          pinocchio::FrameIndex base, bool relative) const {
    pinocchio::forwardKinematics(*model_, *oracle_data_, q);
    pinocchio::updateFramePlacements(*model_, *oracle_data_);
    return relative ? oracle_data_->oMf[base].actInv(oracle_data_->oMf[frame])
                    : oracle_data_->oMf[frame];
  }

  /// d(pose of `frame` [in `base`]) / dq by central differences on a fresh
  /// Data: [origin velocity; angular velocity] in the base's axes (the world's
  /// when not relative). Independent of rf.J and of the relative-Jacobian
  /// formula.
  [[nodiscard]] Mat6X FdJacobian(const Eigen::VectorXd& q, pinocchio::FrameIndex frame,
                                 pinocchio::FrameIndex base, bool relative) const {
    constexpr double kEps = 1e-6;
    Mat6X j(6, kNv);
    Eigen::VectorXd qp = q;
    for (int i = 0; i < kNv; ++i) {
      qp(i) = q(i) + kEps;
      const pinocchio::SE3 plus = OraclePose(qp, frame, base, relative);
      qp(i) = q(i) - kEps;
      const pinocchio::SE3 minus = OraclePose(qp, frame, base, relative);
      qp(i) = q(i);
      j.col(i).head<3>() = (plus.translation() - minus.translation()) / (2.0 * kEps);
      j.col(i).tail<3>() =
          pinocchio::log3(plus.rotation() * minus.rotation().transpose()) / (2.0 * kEps);
    }
    return j;
  }

  /// A state away from home, deterministic in `seed`.
  [[nodiscard]] Eigen::VectorXd StateNear(int seed, double spread) const {
    Eigen::VectorXd q = q_home_;
    for (int i = 0; i < kNv; ++i) {
      q(i) += spread * std::sin(1.3 * seed + 0.7 * i + 0.2);
    }
    return q;
  }

  /// Task on tip_b in the world: the pose at `q` shifted and turned.
  [[nodiscard]] Clik::FrameTask WorldTask(const Eigen::VectorXd& q, const Eigen::Vector3d& shift,
                                          double turn) const {
    Clik::FrameTask task;
    task.kind = Clik::TaskKind::kSe3;
    task.frame_idx = tip_b_;
    task.placement_des = OraclePose(q, frame_b_, frame_torso_, false);
    task.placement_des.translation() += shift;
    task.placement_des.rotation() =
        task.placement_des.rotation() *
        Eigen::AngleAxisd(turn, Eigen::Vector3d(0.3, -0.5, 0.8).normalized()).toRotationMatrix();
    task.gain << 6.0, 6.0, 6.0, 4.0, 4.0, 4.0;
    task.weight = 1.0;
    return task;
  }

  /// Task on tip_a in the torso frame.
  [[nodiscard]] Clik::FrameTask TorsoTask(const Eigen::VectorXd& q, const Eigen::Vector3d& shift,
                                          double turn) const {
    Clik::FrameTask task;
    task.kind = Clik::TaskKind::kSe3;
    task.frame_idx = tip_a_;
    task.base_frame_idx = torso_;
    task.placement_des = OraclePose(q, frame_a_, frame_torso_, true);
    task.placement_des.translation() += shift;
    task.placement_des.rotation() =
        task.placement_des.rotation() *
        Eigen::AngleAxisd(turn, Eigen::Vector3d(-0.6, 0.2, 0.4).normalized()).toRotationMatrix();
    task.gain << 5.0, 5.0, 5.0, 3.0, 3.0, 3.0;
    task.weight = 0.4;
    return task;
  }

  /// Oracle torque of the step v_prev → v at q: RNEA(q, v_prev, (v − v_prev)/dt).
  [[nodiscard]] Eigen::VectorXd OracleTorque(const Eigen::VectorXd& q,
                                             const Eigen::VectorXd& v_prev,
                                             const Eigen::VectorXd& v) const {
    const Eigen::VectorXd a = (v - v_prev) / kDt;
    return pinocchio::rnea(*model_, *oracle_data_, q, v_prev, a);
  }

  /// Oracle classical acceleration of `frame` (LOCAL_WORLD_ALIGNED) for the
  /// step v_prev → v at q, and the frame's rotation there.
  [[nodiscard]] Vec6 OracleFrameAccel(const Eigen::VectorXd& q, const Eigen::VectorXd& v_prev,
                                      const Eigen::VectorXd& v, pinocchio::FrameIndex frame,
                                      Eigen::Matrix3d& rotation) const {
    const Eigen::VectorXd a = (v - v_prev) / kDt;
    pinocchio::forwardKinematics(*model_, *oracle_data_, q, v_prev, a);
    pinocchio::updateFramePlacements(*model_, *oracle_data_);
    rotation = oracle_data_->oMf[frame].rotation();
    return pinocchio::getFrameClassicalAcceleration(*model_, *oracle_data_, frame,
                                                    pinocchio::LOCAL_WORLD_ALIGNED)
        .toVector();
  }

  std::shared_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> oracle_data_;
  PinocchioCache cache_;
  pinocchio::FrameIndex frame_a_{0};
  pinocchio::FrameIndex frame_b_{0};
  pinocchio::FrameIndex frame_torso_{0};
  int tip_a_{-1};
  int tip_b_{-1};
  int torso_{-1};
  std::vector<int> trunk_;
  std::vector<int> arm_a_;
  std::vector<int> arm_b_;
  std::vector<int> all_;
  Eigen::VectorXd q_home_;
};

TEST_F(ClikTreeTest, RelativeJacobianMatchesFiniteDifferencesOfTheRelativePose) {
  struct Pair {
    int frame;
    int base;
    pinocchio::FrameIndex frame_id;
    pinocchio::FrameIndex base_id;
  };

  const std::array<Pair, 4> pairs = {{{tip_a_, torso_, frame_a_, frame_torso_},
                                      {tip_a_, tip_b_, frame_a_, frame_b_},
                                      {tip_b_, tip_a_, frame_b_, frame_a_},
                                      {torso_, tip_a_, frame_torso_, frame_a_}}};
  Eigen::MatrixXd j_rel(6, kNv);
  double worst = 0.0;
  double worst_if_world_aligned = 0.0;
  for (int seed = 0; seed < 5; ++seed) {
    const Eigen::VectorXd q = StateNear(seed, 0.3);
    cache_.Update(q, Eigen::VectorXd::Zero(kNv));
    for (const Pair& p : pairs) {
      const auto& rf = cache_.registered_frames[static_cast<size_t>(p.frame)];
      const auto& rb = cache_.registered_frames[static_cast<size_t>(p.base)];
      FillRelativeFrameJacobian(rf, rb, all_, j_rel);
      const Mat6X fd = FdJacobian(q, p.frame_id, p.base_id, true);
      worst = std::max(worst, (j_rel - fd).cwiseAbs().maxCoeff());
      // What pairing the relative error with the frame's own world-aligned
      // Jacobian would use instead.
      worst_if_world_aligned = std::max(worst_if_world_aligned, (rf.J - fd).cwiseAbs().maxCoeff());
    }
  }
  // Central differences at 1e-6: truncation ~1e-12, round-off ~1e-10.
  EXPECT_LT(worst, 1e-8);
  EXPECT_GT(worst_if_world_aligned, 0.1) << "premise: the two Jacobians differ on this tree";
}

TEST_F(ClikTreeTest, ATaskInTheTorsoFrameHasNoTrunkOrOtherArmColumns) {
  Eigen::MatrixXd j_rel(6, kNv);
  for (int seed = 0; seed < 5; ++seed) {
    const Eigen::VectorXd q = StateNear(seed, 0.4);
    cache_.Update(q, Eigen::VectorXd::Zero(kNv));
    const auto& rf = cache_.registered_frames[static_cast<size_t>(tip_a_)];
    const auto& rb = cache_.registered_frames[static_cast<size_t>(torso_)];
    FillRelativeFrameJacobian(rf, rb, all_, j_rel);
    for (const int i : trunk_) {
      EXPECT_LT(j_rel.col(i).cwiseAbs().maxCoeff(), 1e-12) << "trunk column " << i;
      // The premise: in the world the same frame does move with the trunk.
      EXPECT_GT(rf.J.col(i).norm(), 0.1) << "trunk column " << i;
    }
    for (const int i : arm_b_) {
      EXPECT_LT(j_rel.col(i).cwiseAbs().maxCoeff(), 1e-12) << "other-arm column " << i;
    }
    for (const int i : arm_a_) {
      EXPECT_GT(j_rel.col(i).norm(), 1e-3) << "own-arm column " << i;
    }
  }
}

TEST_F(ClikTreeTest, ATorsoFrameTaskLeavesTheTrunkStillWhereAWorldTaskMovesIt) {
  const Eigen::VectorXd q = StateNear(1, 0.2);
  Clik gen;
  gen.Init(kNv, BaseConfig());  // posture gains 0: nothing but the task moves a joint
  cache_.Update(q, Eigen::VectorXd::Zero(kNv));

  Clik::FrameTask torso_task = TorsoTask(q, Eigen::Vector3d(0.05, 0.04, -0.03), 0.2);
  ASSERT_TRUE(gen.Compute(cache_, Input(std::span<const Clik::FrameTask>(&torso_task, 1), q, kDt)));
  double trunk_speed = 0.0;
  for (const int i : trunk_) {
    trunk_speed = std::max(trunk_speed, std::abs(gen.VRef()(i)));
  }
  double arm_speed = 0.0;
  for (const int i : arm_a_) {
    arm_speed = std::max(arm_speed, std::abs(gen.VRef()(i)));
  }
  EXPECT_LT(trunk_speed, 1e-9);
  EXPECT_GT(arm_speed, 0.05) << "premise: the task does ask for motion";

  // The same frame as a world task: the trunk is the cheapest way there.
  Clik::FrameTask world_task = torso_task;
  world_task.base_frame_idx = -1;
  world_task.placement_des = OraclePose(q, frame_a_, frame_torso_, false);
  world_task.placement_des.translation() += Eigen::Vector3d(0.05, 0.04, -0.03);
  ASSERT_TRUE(gen.Compute(cache_, Input(std::span<const Clik::FrameTask>(&world_task, 1), q, kDt)));
  double world_trunk_speed = 0.0;
  for (const int i : trunk_) {
    world_trunk_speed = std::max(world_trunk_speed, std::abs(gen.VRef()(i)));
  }
  EXPECT_GT(world_trunk_speed, 1e-2);

  // Config::task_v_idx takes the trunk out of a world task's columns.
  Clik::Config cfg = BaseConfig();
  cfg.task_v_idx = arm_a_;
  Clik arm_only;
  arm_only.Init(kNv, cfg);
  ASSERT_TRUE(
      arm_only.Compute(cache_, Input(std::span<const Clik::FrameTask>(&world_task, 1), q, kDt)));
  for (const int i : trunk_) {
    EXPECT_LT(std::abs(arm_only.VRef()(i)), 1e-9) << i;
  }
}

TEST_F(ClikTreeTest, TwoTasksSolveTheClosedFormWeightedLeastSquares) {
  // No box, no rows: the QP is min Σ w_k‖J_k v − r_k‖² + Σ w_g‖v_g − v_post,g‖²
  // + μ²‖v‖², stationary where H·v = rhs. The oracle builds H and rhs from
  // finite differences of the poses and the shared SE(3) error of its own
  // poses, and judges the generator's v by the residual H·v − rhs — the
  // quantity the solver's stopping test (eps_abs = 1e-6) bounds. The distance
  // to the oracle's own solution is that residual through H⁻¹, i.e. up to
  // 1/w_g ≈ 100× larger along a pure posture direction, so it is recorded,
  // not asserted.
  Clik::Config cfg = BaseConfig();
  Clik gen;
  gen.Init(kNv, cfg);
  const std::array<double, 3> group_gain = {0.5, 1.5, 0.8};
  for (int g = 0; g < 3; ++g) {
    ASSERT_TRUE(gen.SetPostureGroupGain(g, group_gain[static_cast<size_t>(g)]));
  }
  EXPECT_FALSE(gen.SetPostureGroupGain(3, 1.0));
  EXPECT_FALSE(gen.SetPostureGroupGain(-1, 1.0));

  double worst = 0.0;
  double worst_if_world_aligned = 0.0;
  double worst_distance = 0.0;
  for (int seed = 0; seed < 4; ++seed) {
    const Eigen::VectorXd q = StateNear(seed, 0.25);
    const Eigen::VectorXd q_posture = StateNear(seed + 11, 0.1);
    Eigen::VectorXd qd_ff(kNv);
    for (int i = 0; i < kNv; ++i) {
      qd_ff(i) = 0.1 * std::cos(0.9 * i + seed);
    }
    Vec6 ff_world;
    ff_world << 0.05, -0.02, 0.03, 0.1, 0.0, -0.05;
    Vec6 ff_torso;
    ff_torso << -0.03, 0.04, 0.01, 0.0, 0.08, 0.02;
    std::array<Clik::FrameTask, 2> tasks = {WorldTask(q, Eigen::Vector3d(0.04, -0.03, 0.05), 0.15),
                                            TorsoTask(q, Eigen::Vector3d(-0.03, 0.02, 0.04), -0.2)};
    tasks[0].twist_ff = &ff_world;
    tasks[1].twist_ff = &ff_torso;

    cache_.Update(q, Eigen::VectorXd::Zero(kNv));
    Clik::MultiFrameInput in = Input(tasks, q_posture, kDt);
    in.qd_posture_ff = &qd_ff;
    ASSERT_TRUE(gen.Compute(cache_, in)) << seed;
    EXPECT_EQ(gen.LastSolve().tasks, 2);
    EXPECT_FALSE(gen.LastSolve().rejected_input);

    // Stationarity residual of the generator's v in the oracle's problem, and
    // its distance to the oracle's solution.
    const auto judge = [&](const Mat6X& j_world, const Mat6X& j_torso, double& distance) {
      const Vec6 r_world =
          tasks[0].gain.cwiseProduct(ComputeTaskPoseError(
              OraclePose(q, frame_b_, frame_torso_, false), tasks[0].placement_des)) +
          ff_world;
      const Vec6 r_torso =
          tasks[1].gain.cwiseProduct(ComputeTaskPoseError(
              OraclePose(q, frame_a_, frame_torso_, true), tasks[1].placement_des)) +
          ff_torso;
      Eigen::MatrixXd H = tasks[0].weight * j_world.transpose() * j_world +
                          tasks[1].weight * j_torso.transpose() * j_torso;
      Eigen::VectorXd rhs = tasks[0].weight * j_world.transpose() * r_world +
                            tasks[1].weight * j_torso.transpose() * r_torso;
      H.diagonal().array() += cfg.damping_sq;
      for (size_t g = 0; g < cfg.posture_groups.size(); ++g) {
        for (const int i : cfg.posture_groups[g].v_idx) {
          const double w = cfg.posture_groups[g].weight;
          H(i, i) += w;
          rhs(i) += w * (group_gain[g] * (q_posture(i) - q(i)) + qd_ff(i));
        }
      }
      const Eigen::VectorXd v_oracle = H.ldlt().solve(rhs);
      distance = (gen.VRef() - v_oracle).cwiseAbs().maxCoeff();
      return (H * gen.VRef() - rhs).cwiseAbs().maxCoeff();
    };
    const Mat6X j_world = FdJacobian(q, frame_b_, frame_torso_, false);
    const Mat6X j_torso = FdJacobian(q, frame_a_, frame_torso_, true);
    double distance = 0.0;
    worst = std::max(worst, judge(j_world, j_torso, distance));
    worst_distance = std::max(worst_distance, distance);
    // The same judgement with the torso task on the frame's world Jacobian.
    const Mat6X j_wrong = FdJacobian(q, frame_a_, frame_torso_, false);
    worst_if_world_aligned = std::max(worst_if_world_aligned, judge(j_world, j_wrong, distance));
  }
  RecordProperty("closed_form_worst_residual", Sci(worst));
  RecordProperty("closed_form_worst_distance", Sci(worst_distance));
  EXPECT_LT(worst, 5e-6);  // the solver's eps_abs is 1e-6
  EXPECT_GT(worst_if_world_aligned, 1e-2) << "premise: the oracle tells the two apart";
}

TEST_F(ClikTreeTest, ConsistentReferencesReturnTheReferenceVelocity) {
  // formulation §4 item 8: at q_c = q_ref, with each task's target the frame's
  // own pose at q_ref, its feed-forward the frame's velocity under q̇_ref, and
  // the posture feed-forward q̇_ref, every term is zero at v = q̇_ref. Only the
  // damping pulls away from it, by μ²/(w_g + μ²) of it at most. Posture weights
  // of 1 here: the solver stops on a residual of 1e-6, which is 1e-6/w_g in v
  // along a posture direction.
  Clik::Config cfg = BaseConfig();
  cfg.damping_sq = 1e-9;
  for (Clik::Config::PostureGroup& group : cfg.posture_groups) {
    group.weight = 1.0;
  }
  Clik gen;
  gen.Init(kNv, cfg);
  for (int g = 0; g < 3; ++g) {
    ASSERT_TRUE(gen.SetPostureGroupGain(g, 2.0));
  }

  const Eigen::VectorXd q_ref = StateNear(3, 0.25);
  Eigen::VectorXd qd_ref(kNv);
  for (int i = 0; i < kNv; ++i) {
    qd_ref(i) = 0.4 * std::sin(0.6 * i + 0.5);
  }

  // Feed-forwards from the oracle, not from the generator's Jacobians: the
  // world one is pinocchio's frame velocity on a fresh Data, the torso one a
  // central difference of the relative pose along q̇_ref.
  pinocchio::forwardKinematics(*model_, *oracle_data_, q_ref, qd_ref);
  pinocchio::updateFramePlacements(*model_, *oracle_data_);
  const Vec6 ff_world =
      pinocchio::getFrameVelocity(*model_, *oracle_data_, frame_b_, pinocchio::LOCAL_WORLD_ALIGNED)
          .toVector();
  constexpr double kEps = 1e-6;
  const pinocchio::SE3 plus = OraclePose(q_ref + kEps * qd_ref, frame_a_, frame_torso_, true);
  const pinocchio::SE3 minus = OraclePose(q_ref - kEps * qd_ref, frame_a_, frame_torso_, true);
  Vec6 ff_torso;
  ff_torso.head<3>() = (plus.translation() - minus.translation()) / (2.0 * kEps);
  ff_torso.tail<3>() =
      pinocchio::log3(plus.rotation() * minus.rotation().transpose()) / (2.0 * kEps);

  std::array<Clik::FrameTask, 2> tasks = {WorldTask(q_ref, Eigen::Vector3d::Zero(), 0.0),
                                          TorsoTask(q_ref, Eigen::Vector3d::Zero(), 0.0)};
  tasks[0].twist_ff = &ff_world;
  tasks[1].twist_ff = &ff_torso;

  cache_.Update(q_ref, Eigen::VectorXd::Zero(kNv));
  Clik::MultiFrameInput in = Input(tasks, q_ref, kDt);
  in.qd_posture_ff = &qd_ref;
  ASSERT_TRUE(gen.Compute(cache_, in));
  EXPECT_LT((gen.VRef() - qd_ref).cwiseAbs().maxCoeff(), 1e-5);
  EXPECT_LT(gen.TcpErrorNorm(), 1e-9);

  // Without the posture feed-forward the posture row asks for v = 0 and the
  // null space of the two tasks follows it — the lag this input removes.
  in.qd_posture_ff = nullptr;
  ASSERT_TRUE(gen.Compute(cache_, in));
  EXPECT_GT((gen.VRef() - qd_ref).cwiseAbs().maxCoeff(), 1e-2);
}

TEST_F(ClikTreeTest, DynamicRowsKeepEveryJointsOracleTorqueInsideTheBound) {
  constexpr double kEta = 0.8;
  const std::array<Clik::FrameTask, 2> tasks = [&] {
    std::array<Clik::FrameTask, 2> t = {
        WorldTask(q_home_, Eigen::Vector3d(0.15, -0.10, 0.15), 0.4),
        TorsoTask(q_home_, Eigen::Vector3d(0.10, 0.10, -0.10), 0.3)};
    t[0].gain = Vec6::Constant(40.0);
    t[1].gain = Vec6::Constant(40.0);
    return t;
  }();
  const Eigen::VectorXd bound = kEta * model_->effortLimit;

  const auto worst_ratio = [&](const Clik::Config& cfg, int ticks, int& failed, int& rows) {
    Clik gen;
    gen.Init(kNv, cfg);
    for (int g = 0; g < 3; ++g) {
      EXPECT_TRUE(gen.SetPostureGroupGain(g, 0.5));
    }
    Eigen::VectorXd q = q_home_;
    Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(kNv);
    double worst = 0.0;
    failed = 0;
    rows = 0;
    for (int k = 0; k < ticks; ++k) {
      cache_.Update(q, v_prev);
      if (!gen.Compute(cache_, Input(tasks, q_home_, kDt))) {
        ++failed;
        v_prev.setZero();
        continue;
      }
      rows = gen.LastSolve().accel_rows;
      const Eigen::VectorXd tau = OracleTorque(q, v_prev, gen.VRef());
      worst = std::max(worst, (tau.cwiseAbs().array() / bound.array()).maxCoeff());
      q = gen.QRef();
      v_prev = gen.VRef();
    }
    return worst;
  };

  int failed = 0;
  int rows = 0;
  // Premise: the same run without the rows asks for more than η τ_max.
  const double free_worst = worst_ratio(BoxedConfig(), 200, failed, rows);
  ASSERT_GT(free_worst, 1.5) << "premise: the unconstrained steps exceed the torque bound";
  EXPECT_EQ(rows, 0);

  Clik::Config cfg = BoxedConfig();
  cfg.accel_constraint = Mode::kDynamic;
  cfg.tau_max = model_->effortLimit;
  cfg.eta_tau = kEta;
  const double worst = worst_ratio(cfg, 200, failed, rows);
  EXPECT_EQ(failed, 0);
  EXPECT_EQ(rows, kNv) << "one torque row per joint of the tree";
  EXPECT_LE(worst, 1.0 + 1e-5) << "the torque rows did not hold";
  EXPECT_GT(worst, 0.9) << "the bound should be active in this run";
}

TEST_F(ClikTreeTest, KinematicRowsStackPerTaskAndBoundEachTasksAcceleration) {
  constexpr double kLin = 6.0;
  constexpr double kAng = 15.0;
  constexpr double kTolRel = 2e-4;  // test_clik_accel_constraint.cpp's, same reason
  // Two world tasks: an SE3 task on tip_b (6 rows) and a position + axis task
  // on tip_a (5 rows).
  Clik::FrameTask se3 = WorldTask(q_home_, Eigen::Vector3d(0.15, -0.10, 0.15), 0.4);
  se3.gain = Vec6::Constant(40.0);
  Clik::FrameTask axis;
  axis.kind = Clik::TaskKind::kPositionAxis;
  axis.frame_idx = tip_a_;
  const pinocchio::SE3 home_a = OraclePose(q_home_, frame_a_, frame_torso_, false);
  axis.target.position = home_a.translation() + Eigen::Vector3d(0.10, 0.10, -0.10);
  axis.target.axis = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX()) * home_a.rotation().col(2);
  axis.gain = Vec6::Constant(40.0);
  axis.gain_axis = 40.0;
  const std::array<Clik::FrameTask, 2> tasks = {se3, axis};

  const auto worst_ratio = [&](const Clik::Config& cfg, int ticks, int& rows) {
    Clik gen;
    gen.Init(kNv, cfg);
    Eigen::VectorXd q = q_home_;
    Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(kNv);
    double worst = 0.0;
    rows = 0;
    for (int k = 0; k < ticks; ++k) {
      cache_.Update(q, v_prev);
      EXPECT_TRUE(gen.Compute(cache_, Input(tasks, q_home_, kDt))) << k;
      rows = gen.LastSolve().accel_rows;
      Eigen::Matrix3d rotation;
      const Vec6 a_b = OracleFrameAccel(q, v_prev, gen.VRef(), frame_b_, rotation);
      for (int r = 0; r < 6; ++r) {
        worst = std::max(worst, std::abs(a_b(r)) / (r < 3 ? kLin : kAng));
      }
      const Vec6 a_a = OracleFrameAccel(q, v_prev, gen.VRef(), frame_a_, rotation);
      for (int r = 0; r < 3; ++r) {
        worst = std::max(worst, std::abs(a_a(r)) / kLin);
      }
      // The axis rows live in the frame's LOCAL x, y.
      const Eigen::Vector3d alpha_local = rotation.transpose() * a_a.tail<3>();
      worst = std::max(worst, std::abs(alpha_local(0)) / kAng);
      worst = std::max(worst, std::abs(alpha_local(1)) / kAng);
      q = gen.QRef();
      v_prev = gen.VRef();
    }
    return worst;
  };

  Clik::Config free_cfg = BoxedConfig();
  free_cfg.relative_tasks = false;
  int rows = 0;
  ASSERT_GT(worst_ratio(free_cfg, 150, rows), 1.5)
      << "premise: the unconstrained steps exceed the task acceleration bound";

  Clik::Config cfg = free_cfg;
  cfg.accel_constraint = Mode::kKinematic;
  cfg.task_accel_max_linear = kLin;
  cfg.task_accel_max_angular = kAng;
  const double worst = worst_ratio(cfg, 150, rows);
  EXPECT_EQ(rows, 11) << "6 rows of the SE3 task + 5 of the position + axis task";
  EXPECT_LE(worst, 1.0 + kTolRel);
  EXPECT_GT(worst, 0.9) << "the bound should be active in this run";
}

TEST_F(ClikTreeTest, FeedbackCapIsTheUncappedProblemAtScaledGains) {
  const Eigen::VectorXd q = StateNear(2, 0.2);
  cache_.Update(q, Eigen::VectorXd::Zero(kNv));
  Clik gen;
  Clik reference;
  gen.Init(kNv, BaseConfig());
  reference.Init(kNv, BaseConfig());

  Vec6 ff;
  ff << 0.02, 0.01, -0.03, 0.05, -0.04, 0.0;
  Clik::FrameTask capped = WorldTask(q, Eigen::Vector3d(0.3, -0.2, 0.25), 1.0);
  capped.twist_ff = &ff;
  capped.fb_lin_max = 0.2;
  capped.fb_ang_max = 0.3;

  // The uncapped feedback, from the oracle's pose.
  const Vec6 e =
      ComputeTaskPoseError(OraclePose(q, frame_b_, frame_torso_, false), capped.placement_des);
  const double fb_lin = capped.gain.head<3>().cwiseProduct(e.head<3>()).norm();
  const double fb_ang = capped.gain.tail<3>().cwiseProduct(e.tail<3>()).norm();
  ASSERT_GT(fb_lin, 2.0 * capped.fb_lin_max) << "premise: the linear block is over its cap";
  ASSERT_GT(fb_ang, 2.0 * capped.fb_ang_max) << "premise: the angular block is over its cap";

  ASSERT_TRUE(gen.Compute(cache_, Input(std::span<const Clik::FrameTask>(&capped, 1), q, kDt)));
  EXPECT_EQ(gen.LastSolve().fb_saturated, 0b11U);

  // Scaling a block onto its cap is a positive scalar on that block's gain: the
  // direction of each block is kept, and the feed-forward rides on top.
  Clik::FrameTask scaled = capped;
  scaled.fb_lin_max = 0.0;
  scaled.fb_ang_max = 0.0;
  scaled.gain.head<3>() *= capped.fb_lin_max / fb_lin;
  scaled.gain.tail<3>() *= capped.fb_ang_max / fb_ang;
  ASSERT_TRUE(
      reference.Compute(cache_, Input(std::span<const Clik::FrameTask>(&scaled, 1), q, kDt)));
  EXPECT_EQ(reference.LastSolve().fb_saturated, 0U);
  EXPECT_LT((gen.VRef() - reference.VRef()).cwiseAbs().maxCoeff(), 1e-9);

  // And it changed the answer: uncapped, the same task asks for far more.
  Clik::FrameTask uncapped = capped;
  uncapped.fb_lin_max = 0.0;
  uncapped.fb_ang_max = 0.0;
  ASSERT_TRUE(
      reference.Compute(cache_, Input(std::span<const Clik::FrameTask>(&uncapped, 1), q, kDt)));
  EXPECT_GT((gen.VRef() - reference.VRef()).cwiseAbs().maxCoeff(), 0.1);

  // A cap above the feedback is not there at all: bit-identical to cap off.
  Clik::FrameTask loose = uncapped;
  loose.fb_lin_max = 2.0 * fb_lin;
  loose.fb_ang_max = 2.0 * fb_ang;
  ASSERT_TRUE(gen.Compute(cache_, Input(std::span<const Clik::FrameTask>(&loose, 1), q, kDt)));
  EXPECT_EQ(gen.LastSolve().fb_saturated, 0U);
  EXPECT_TRUE(BitEqual(gen.VRef(), reference.VRef()));

  // Second task's bits, and the approach-axis block of a position + axis task.
  Clik::FrameTask axis;
  axis.kind = Clik::TaskKind::kPositionAxis;
  axis.frame_idx = tip_a_;
  const pinocchio::SE3 pose_a = OraclePose(q, frame_a_, frame_torso_, false);
  axis.target.position = pose_a.translation() + Eigen::Vector3d(0.0, 0.0, 0.001);
  axis.target.axis =
      Eigen::AngleAxisd(1.0, Eigen::Vector3d(pose_a.rotation().col(0))) * pose_a.rotation().col(2);
  axis.gain = Vec6::Constant(5.0);
  axis.gain_axis = 5.0;
  axis.fb_lin_max = 0.2;  // 5 · 1 mm: under
  axis.fb_ang_max = 0.3;  // 5 · 1 rad: over
  const std::array<Clik::FrameTask, 2> two = {uncapped, axis};
  ASSERT_TRUE(gen.Compute(cache_, Input(two, q, kDt)));
  EXPECT_EQ(gen.LastSolve().fb_saturated, 0b1000U) << "task 1, angular block";
}

TEST_F(ClikTreeTest, RotationErrorNearPiRaisesTheDiagnosticBit) {
  const Eigen::VectorXd q = StateNear(0, 0.2);
  cache_.Update(q, Eigen::VectorXd::Zero(kNv));
  Clik gen;
  gen.Init(kNv, BaseConfig());

  std::array<Clik::FrameTask, 2> tasks = {WorldTask(q, Eigen::Vector3d::Zero(), 2.0),
                                          TorsoTask(q, Eigen::Vector3d::Zero(), 0.3)};
  ASSERT_TRUE(gen.Compute(cache_, Input(tasks, q, kDt)));
  EXPECT_EQ(gen.LastSolve().rot_near_pi, 0U) << "2.0 rad and 0.3 rad are not near π";

  tasks[1] = TorsoTask(q, Eigen::Vector3d::Zero(), std::numbers::pi - 0.01);
  tasks[1].fb_ang_max = 0.5;  // what a caller near π sets
  ASSERT_TRUE(gen.Compute(cache_, Input(tasks, q, kDt)));
  EXPECT_EQ(gen.LastSolve().rot_near_pi, 0b10U) << "task 1";
  EXPECT_TRUE(gen.VRef().allFinite());

  // The approach axis of a position + axis task, antiparallel to its target.
  Clik::FrameTask axis;
  axis.kind = Clik::TaskKind::kPositionAxis;
  axis.frame_idx = tip_a_;
  const pinocchio::SE3 pose_a = OraclePose(q, frame_a_, frame_torso_, false);
  axis.target.position = pose_a.translation();
  // About the frame's own x, which is ⟂ its z: the angle between the two
  // axes is then the rotation angle.
  axis.target.axis =
      Eigen::AngleAxisd(std::numbers::pi - 0.02, Eigen::Vector3d(pose_a.rotation().col(0))) *
      pose_a.rotation().col(2);
  axis.gain = Vec6::Constant(5.0);
  axis.gain_axis = 5.0;
  ASSERT_TRUE(gen.Compute(cache_, Input(std::span<const Clik::FrameTask>(&axis, 1), q, kDt)));
  EXPECT_EQ(gen.LastSolve().rot_near_pi, 0b1U);
}

// ── Braking-distance bound ──────────────────────────────────────────────────

/// Drives five heavy joints back and forth between their position limits with
/// a stiff posture term, under the torque rows, for `ticks` perfectly tracked
/// steps. This is the situation the one-tick position box cannot survive: a
/// joint arrives at its limit at full speed and has one tick to stop.
struct LimitRunResult {
  int failed{0};
  int ticks_at_a_limit{0};       ///< some driven joint within 1 mrad of a limit
  int ticks_braking{0};          ///< LastSolve().brake_active != 0
  double worst_torque_ratio{0};  ///< oracle |τ_i| / (η τ_max,i) over solved ticks
  double worst_overshoot{0};     ///< q_ref past a position limit [rad]
  std::uint64_t static_infeasible{0};
  bool box_empty{false};
};

class ClikBrakeTest : public ClikTreeTest {
 protected:
  static constexpr double kEta = 0.8;

  [[nodiscard]] Clik::Config DynamicConfig(bool brake, double margin) const {
    Clik::Config cfg = BoxedConfig();
    for (Clik::Config::PostureGroup& group : cfg.posture_groups) {
      group.weight = 1.0;
    }
    cfg.relative_tasks = false;
    cfg.max_frame_tasks = 1;
    cfg.accel_constraint = Mode::kDynamic;
    cfg.tau_max = model_->effortLimit;
    cfg.eta_tau = kEta;
    cfg.brake_from_torque = brake;
    cfg.brake_margin = margin;
    return cfg;
  }

  [[nodiscard]] LimitRunResult RunBetweenLimits(const Clik::Config& cfg, int ticks) {
    Clik gen;
    gen.Init(kNv, cfg);
    for (int g = 0; g < 3; ++g) {
      EXPECT_TRUE(gen.SetPostureGroupGain(g, 20.0));
    }
    const std::array<int, 5> driven = {trunk_[0], arm_a_[0], arm_a_[1], arm_a_[3], arm_b_[0]};
    const Eigen::VectorXd& q_min = model_->lowerPositionLimit;
    const Eigen::VectorXd& q_max = model_->upperPositionLimit;
    const Eigen::VectorXd bound = kEta * model_->effortLimit;

    // A light task that asks for nothing: tip_b stays where it is.
    Clik::FrameTask hold;
    hold.kind = Clik::TaskKind::kSe3;
    hold.frame_idx = tip_b_;
    hold.weight = 1e-3;

    // Start 0.3 rad inside the upper limits, so the first approach is at speed.
    Eigen::VectorXd q = q_home_;
    for (const int i : driven) {
      q(i) = q_max(i) - 0.3;
    }
    Eigen::VectorXd v_prev = Eigen::VectorXd::Zero(kNv);
    Eigen::VectorXd q_posture = q_home_;
    LimitRunResult out;
    for (int k = 0; k < ticks; ++k) {
      // The posture target sits past a limit and changes side every 250 ticks.
      const bool up = ((k / 250) % 2) == 0;
      for (const int i : driven) {
        q_posture(i) = up ? q_max(i) + 0.5 : q_min(i) - 0.5;
      }
      cache_.Update(q, v_prev);
      hold.placement_des = cache_.registered_frames[static_cast<size_t>(tip_b_)].oMf;
      const bool ok =
          gen.Compute(cache_, Input(std::span<const Clik::FrameTask>(&hold, 1), q_posture, kDt));
      out.static_infeasible |= gen.LastSolve().brake_static_infeasible;
      out.box_empty = out.box_empty || gen.LastSolve().brake_box_empty;
      out.ticks_braking += gen.LastSolve().brake_active != 0 ? 1 : 0;
      if (!ok) {
        ++out.failed;
      } else {
        const Eigen::VectorXd tau = OracleTorque(q, v_prev, gen.VRef());
        out.worst_torque_ratio =
            std::max(out.worst_torque_ratio, (tau.cwiseAbs().array() / bound.array()).maxCoeff());
      }
      // Perfect tracking of whatever came out — after a failure that is
      // q_ref = q, v_ref = 0.
      q = gen.QRef();
      v_prev = gen.VRef();
      bool at_limit = false;
      for (const int i : driven) {
        at_limit = at_limit || q_max(i) - q(i) < 1e-3 || q(i) - q_min(i) < 1e-3;
        out.worst_overshoot = std::max({out.worst_overshoot, q(i) - q_max(i), q_min(i) - q(i)});
      }
      out.ticks_at_a_limit += at_limit ? 1 : 0;
    }
    return out;
  }
};

TEST_F(ClikBrakeTest, BrakingBoundKeepsTheTorqueRowsFeasibleAtThePositionLimits) {
  constexpr int kTicks = 1000;
  // Positive control: the same run on the one-tick position box alone fails.
  const LimitRunResult off = RunBetweenLimits(DynamicConfig(false, 1.0), kTicks);
  EXPECT_GT(off.failed, 0) << "premise: without the bound the run breaks at a limit";
  EXPECT_GT(off.ticks_at_a_limit, 50) << "premise: the run does reach the limits";
  EXPECT_EQ(off.ticks_braking, 0);

  const LimitRunResult on = RunBetweenLimits(DynamicConfig(true, 1.0), kTicks);
  RecordProperty("failed_off", off.failed);
  RecordProperty("failed_on", on.failed);
  RecordProperty("ticks_braking", on.ticks_braking);
  RecordProperty("ticks_at_a_limit_on", on.ticks_at_a_limit);
  RecordProperty("worst_torque_excess_on", Sci(on.worst_torque_ratio - 1.0));
  RecordProperty("worst_torque_excess_off", Sci(off.worst_torque_ratio - 1.0));
  EXPECT_EQ(on.failed, 0);
  EXPECT_GT(on.ticks_braking, 0) << "the bound never narrowed a box";
  EXPECT_GT(on.ticks_at_a_limit, 50) << "the bound must not keep the joints away from the limits";
  // The generator judges its unit-norm rows to 1e-6 absolute + 1e-6 of the
  // row's shift, and the shift carries M·v_c/dt: at the 3 rad/s these joints
  // reach that is ≈ 2e-5 of the bound (the SE3 runs above move slower and sit
  // inside 1e-5). The same run without the bound shows the same excess.
  EXPECT_LE(on.worst_torque_ratio, 1.0 + 1e-4) << "the torque rows still hold";
  EXPECT_LE(off.worst_torque_ratio, 1.0 + 1e-4);
  EXPECT_LT(on.worst_overshoot, 1e-6);
  EXPECT_EQ(on.static_infeasible, 0U) << "the tree's effort limits carry its gravity load";
  EXPECT_FALSE(on.box_empty);

  // A margin plans with less deceleration: it brakes earlier, and still holds.
  const LimitRunResult cautious = RunBetweenLimits(DynamicConfig(true, 0.5), kTicks);
  EXPECT_EQ(cautious.failed, 0);
  EXPECT_GE(cautious.ticks_braking, on.ticks_braking);
}

TEST_F(ClikBrakeTest, BrakingBoundLeavesTheSolveAloneAwayFromTheLimits) {
  // Mid-range and slow, every joint's braking speed is above its velocity
  // limit: the box is the one without the option, and so is the output.
  Clik on;
  Clik off;
  on.Init(kNv, DynamicConfig(true, 1.0));
  off.Init(kNv, DynamicConfig(false, 1.0));
  Clik::FrameTask task = WorldTask(q_home_, Eigen::Vector3d(0.03, -0.02, 0.02), 0.1);
  Eigen::VectorXd q = q_home_;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(kNv);
  for (int k = 0; k < 50; ++k) {
    cache_.Update(q, v);
    const Clik::MultiFrameInput in =
        Input(std::span<const Clik::FrameTask>(&task, 1), q_home_, kDt);
    ASSERT_TRUE(on.Compute(cache_, in)) << k;
    ASSERT_TRUE(off.Compute(cache_, in)) << k;
    EXPECT_EQ(on.LastSolve().brake_active, 0U) << k;
    ASSERT_TRUE(BitEqual(on.VRef(), off.VRef())) << k;
    q = on.QRef();
    v = on.VRef();
  }
}

TEST_F(ClikBrakeTest, ANonFiniteDynamicsTermFailsTheCallInsteadOfStoppingQuietly) {
  Clik gen;
  gen.Init(kNv, DynamicConfig(true, 1.0));
  Clik::FrameTask task = WorldTask(q_home_, Eigen::Vector3d(0.03, 0.0, 0.0), 0.0);
  const Clik::MultiFrameInput in = Input(std::span<const Clik::FrameTask>(&task, 1), q_home_, kDt);
  cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
  ASSERT_TRUE(gen.Compute(cache_, in));
  // Command mode: the next call's state is this call's command.
  const Eigen::VectorXd q_c = gen.QRef();
  const Eigen::VectorXd v_c = gen.VRef();
  cache_.Update(q_c, v_c);

  // max(0, NaN) is 0: unchecked, a NaN inertia would read as "no deceleration
  // available" and pin the joint at v = 0 with every output finite.
  const int joint = arm_a_[0];
  cache_.M(joint, joint) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(gen.Compute(cache_, in));
  EXPECT_TRUE(gen.LastSolve().non_finite);
  EXPECT_FALSE(gen.LastSolve().reached_solve);
  EXPECT_FALSE(gen.LastSolve().command_mismatch);
  EXPECT_TRUE(BitEqual(gen.QRef(), q_c)) << "the failure branch's outputs";
  EXPECT_EQ(gen.VRef().cwiseAbs().maxCoeff(), 0.0);

  cache_.Update(gen.QRef(), gen.VRef());  // M is finite again
  ASSERT_TRUE(gen.Compute(cache_, in)) << "recovers on the next finite tick";
}

TEST_F(ClikBrakeTest, InitRejectsABrakingBoundWithoutItsInputs) {
  const auto throws = [&](const Clik::Config& cfg) {
    Clik gen;
    try {
      gen.Init(kNv, cfg);
    } catch (const std::runtime_error&) {
      return true;
    }
    return false;
  };
  EXPECT_FALSE(throws(DynamicConfig(true, 1.0))) << "premise";
  EXPECT_FALSE(throws(DynamicConfig(true, 0.5)));

  Clik::Config cfg = BoxedConfig();  // kBox
  cfg.brake_from_torque = true;
  EXPECT_TRUE(throws(cfg)) << "no torque bound to brake with";

  cfg = DynamicConfig(true, 1.0);
  cfg.q_min.resize(0);
  cfg.q_max.resize(0);
  EXPECT_TRUE(throws(cfg)) << "no position box to brake for";

  for (const double bad : {0.0, -0.5, 1.5, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
    EXPECT_TRUE(throws(DynamicConfig(true, bad))) << "brake_margin " << bad;
  }
  EXPECT_TRUE(throws(DynamicConfig(false, 0.5))) << "a margin for a bound that is off";
}

TEST_F(ClikTreeTest, InitRejectsInconsistentMultiFrameConfig) {
  const auto throws = [&](const Clik::Config& cfg) {
    Clik gen;
    try {
      gen.Init(kNv, cfg);
    } catch (const std::runtime_error&) {
      return true;
    }
    return false;
  };
  EXPECT_FALSE(throws(BaseConfig())) << "premise: the base config is valid";

  Clik::Config cfg = BaseConfig();
  cfg.max_frame_tasks = 0;
  EXPECT_TRUE(throws(cfg));
  cfg.max_frame_tasks = 17;
  EXPECT_TRUE(throws(cfg));
  cfg.max_frame_tasks = 16;
  EXPECT_FALSE(throws(cfg));

  cfg = BaseConfig();
  cfg.accel_constraint = Mode::kKinematic;
  cfg.task_accel_max_linear = 5.0;
  cfg.task_accel_max_angular = 10.0;
  EXPECT_TRUE(throws(cfg)) << "relative tasks with the kinematic rows";
  cfg.relative_tasks = false;
  EXPECT_FALSE(throws(cfg));

  cfg = BaseConfig();
  cfg.task_v_idx = {0, 1, kNv};
  EXPECT_TRUE(throws(cfg)) << "task_v_idx out of range";
  cfg.task_v_idx = {0, 1, 1};
  EXPECT_TRUE(throws(cfg)) << "task_v_idx duplicate";

  cfg = BaseConfig();
  cfg.posture_groups[1].v_idx.push_back(trunk_[0]);
  EXPECT_TRUE(throws(cfg)) << "a joint in two posture groups";
  cfg = BaseConfig();
  cfg.posture_groups[0].v_idx.push_back(-1);
  EXPECT_TRUE(throws(cfg)) << "posture group index out of range";
  for (const double bad :
       {-1e-3, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    cfg = BaseConfig();
    cfg.posture_groups[2].weight = bad;
    EXPECT_TRUE(throws(cfg)) << "posture group weight " << bad;
  }
}

TEST_F(ClikTreeTest, RefusedInputLeavesTheOutputsUntouched) {
  const Eigen::VectorXd q = StateNear(0, 0.2);
  cache_.Update(q, Eigen::VectorXd::Zero(kNv));
  Clik gen;
  gen.Init(kNv, BaseConfig());
  const std::array<Clik::FrameTask, 2> good = {WorldTask(q, Eigen::Vector3d(0.05, 0.0, 0.0), 0.1),
                                               TorsoTask(q, Eigen::Vector3d(0.0, 0.05, 0.0), 0.1)};
  ASSERT_TRUE(gen.Compute(cache_, Input(good, q, kDt)));
  const Eigen::VectorXd q_ref = gen.QRef();
  const Eigen::VectorXd v_ref = gen.VRef();
  ASSERT_GT(v_ref.norm(), 1e-3);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  int cases = 0;
  const auto refused = [&](const char* what, std::span<const Clik::FrameTask> tasks,
                           const Eigen::VectorXd* q_posture, const Eigen::VectorXd* qd_ff,
                           double dt) {
    Clik::MultiFrameInput in;
    in.tasks = tasks;
    in.q_posture_des = q_posture;
    in.qd_posture_ff = qd_ff;
    in.dt = dt;
    EXPECT_FALSE(gen.Compute(cache_, in)) << what;
    EXPECT_TRUE(gen.LastSolve().rejected_input) << what;
    EXPECT_FALSE(gen.LastSolve().reached_solve) << what;
    EXPECT_EQ(gen.LastSolve().tasks, 0) << what;
    EXPECT_TRUE(BitEqual(gen.QRef(), q_ref)) << what;
    EXPECT_TRUE(BitEqual(gen.VRef(), v_ref)) << what;
    ++cases;
  };
  const auto with = [&](const char* what, int k, const auto& edit) {
    std::array<Clik::FrameTask, 2> tasks = good;
    edit(tasks[static_cast<size_t>(k)]);
    refused(what, tasks, &q, nullptr, kDt);
  };

  // The call's shape.
  refused("no task", std::span<const Clik::FrameTask>(), &q, nullptr, kDt);
  const std::array<Clik::FrameTask, 3> three = {good[0], good[1], good[0]};
  refused("more tasks than max_frame_tasks", three, &q, nullptr, kDt);
  refused("no posture", good, nullptr, nullptr, kDt);
  const Eigen::VectorXd short_vec = Eigen::VectorXd::Zero(kNv - 1);
  refused("posture size", good, &short_vec, nullptr, kDt);
  refused("posture feed-forward size", good, &q, &short_vec, kDt);
  Eigen::VectorXd bad_ff = Eigen::VectorXd::Zero(kNv);
  bad_ff(4) = nan;
  refused("posture feed-forward NaN", good, &q, &bad_ff, kDt);
  refused("dt 0", good, &q, nullptr, 0.0);
  refused("dt NaN", good, &q, nullptr, nan);
  refused("dt negative", good, &q, nullptr, -kDt);

  // Frames.
  with("frame index negative", 0, [](Clik::FrameTask& t) { t.frame_idx = -1; });
  with("frame index past the end", 1, [](Clik::FrameTask& t) { t.frame_idx = 3; });
  with("base index past the end", 1, [](Clik::FrameTask& t) { t.base_frame_idx = 3; });
  with("base equal to the frame", 1, [](Clik::FrameTask& t) { t.base_frame_idx = t.frame_idx; });

  // Gains, weights, caps.
  with("weight 0", 0, [](Clik::FrameTask& t) { t.weight = 0.0; });
  with("weight NaN", 0, [&](Clik::FrameTask& t) { t.weight = nan; });
  with("weight inf", 1, [&](Clik::FrameTask& t) { t.weight = inf; });
  with("gain NaN", 1, [&](Clik::FrameTask& t) { t.gain(4) = nan; });
  with("linear cap negative", 0, [](Clik::FrameTask& t) { t.fb_lin_max = -0.1; });
  with("linear cap NaN", 0, [&](Clik::FrameTask& t) { t.fb_lin_max = nan; });
  with("angular cap inf", 1, [&](Clik::FrameTask& t) { t.fb_ang_max = inf; });

  // Feed-forwards and the position + axis target.
  Vec6 bad_twist = Vec6::Zero();
  bad_twist(2) = inf;
  with("twist feed-forward inf", 0, [&](Clik::FrameTask& t) { t.twist_ff = &bad_twist; });
  const auto axis_task = [&](Clik::FrameTask& t) {
    const pinocchio::SE3 pose = OraclePose(q, frame_a_, frame_torso_, true);
    t.kind = Clik::TaskKind::kPositionAxis;
    t.target.position = pose.translation();
    t.target.axis = pose.rotation().col(2);
    t.gain_axis = 3.0;
  };
  {
    // Premise, on a generator of its own (a solve moves the warm start, and
    // `gen`'s outputs are the reference here): the axis task as built is valid.
    std::array<Clik::FrameTask, 2> tasks = good;
    axis_task(tasks[1]);
    Clik probe;
    probe.Init(kNv, BaseConfig());
    ASSERT_TRUE(probe.Compute(cache_, Input(tasks, q, kDt))) << "premise: the axis task is valid";
  }
  with("axis target position NaN", 1, [&](Clik::FrameTask& t) {
    axis_task(t);
    t.target.position(0) = nan;
  });
  with("axis target feed-forward NaN", 1, [&](Clik::FrameTask& t) {
    axis_task(t);
    t.target.angular_velocity_ff(1) = nan;
  });
  with("axis weight 0", 1, [&](Clik::FrameTask& t) {
    axis_task(t);
    t.weight_axis = 0.0;
  });
  with("axis gain NaN", 1, [&](Clik::FrameTask& t) {
    axis_task(t);
    t.gain_axis = nan;
  });
  with("axis not unit", 1, [&](Clik::FrameTask& t) {
    axis_task(t);
    t.target.axis *= 1.5;
  });
  EXPECT_EQ(cases, 26) << "every case above ran";

  // A relative task where the config does not allow one.
  Clik::Config cfg = BaseConfig();
  cfg.relative_tasks = false;
  Clik strict;
  strict.Init(kNv, cfg);
  EXPECT_FALSE(strict.Compute(cache_, Input(good, q, kDt)));
  EXPECT_TRUE(strict.LastSolve().rejected_input);
  EXPECT_TRUE(strict.Compute(cache_, Input(std::span<const Clik::FrameTask>(&good[0], 1), q, kDt)));

  // A non-finite SE3 target is NOT refused: it reaches the solve and takes the
  // failure branch, whose outputs are the safe ones.
  std::array<Clik::FrameTask, 2> tasks = good;
  tasks[0].placement_des.translation()(1) = nan;
  EXPECT_FALSE(gen.Compute(cache_, Input(tasks, q, kDt)));
  EXPECT_FALSE(gen.LastSolve().rejected_input);
  EXPECT_TRUE(gen.LastSolve().reached_solve);
  EXPECT_TRUE(BitEqual(gen.QRef(), q));
  EXPECT_EQ(gen.VRef().cwiseAbs().maxCoeff(), 0.0);
}

TEST_F(ClikTreeTest, ComputeCallsNoOperatorNew) {
  // The C++ side only. Eigen's own allocator does not go through operator new;
  // test_clik_multiframe_malloc.cpp counts those.
  Clik::Config cfg = BoxedConfig();
  cfg.accel_constraint = Mode::kDynamic;
  cfg.tau_max = model_->effortLimit;
  cfg.eta_tau = 0.8;
  cfg.w_smooth = 1e-3;
  Clik gen;
  gen.Init(kNv, cfg);
  for (int g = 0; g < 3; ++g) {
    ASSERT_TRUE(gen.SetPostureGroupGain(g, 0.5));
  }
  Vec6 ff = Vec6::Constant(0.01);
  std::array<Clik::FrameTask, 2> tasks = {
      WorldTask(q_home_, Eigen::Vector3d(0.05, -0.03, 0.04), 0.2),
      TorsoTask(q_home_, Eigen::Vector3d(0.03, 0.03, -0.03), 0.2)};
  tasks[0].twist_ff = &ff;
  tasks[0].fb_lin_max = 0.1;
  tasks[1].fb_ang_max = 0.1;
  const Eigen::VectorXd qd_ff = Eigen::VectorXd::Constant(kNv, 0.01);
  Clik::MultiFrameInput in = Input(tasks, q_home_, kDt);
  in.qd_posture_ff = &qd_ff;

  Eigen::VectorXd q = q_home_;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(kNv);
  for (int k = 0; k < 5; ++k) {  // warm-up
    cache_.Update(q, v);
    ASSERT_TRUE(gen.Compute(cache_, in)) << k;
    q = gen.QRef();
    v = gen.VRef();
  }
  cache_.Update(q, v);
  test::AllocCounter::Arm();
  const bool ok = gen.Compute(cache_, in);
  test::AllocCounter::Disarm();
  EXPECT_TRUE(ok);
  EXPECT_EQ(test::AllocCounter::alloc_count.load(), 0);
  EXPECT_NE(gen.LastSolve().fb_saturated, 0U) << "premise: the cap path ran";
}

}  // namespace
}  // namespace rtc::tsid
