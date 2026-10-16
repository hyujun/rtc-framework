/// @file test_rotated_base_frame.cpp
/// @brief SE3Task and the CLIK SE3 overload on a base frame that is turned
///        against the world (#779).
///
/// Both measure the pose error of the frame IN THE BASE, so in the base's
/// axes, while the registered-frame Jacobian is in the world's. The base here
/// is a frame fixed to the universe at an offset, turned about two axes
/// (panda_fixture.hpp, kRotatedBaseFrame), so ignoring the rotation or
/// applying it the wrong way round changes every result. Each case is judged
/// against something that does not go through the base:
///
///   - SE3Task: the same task stated in the world (universe base, target and
///     feed-forwards moved into the world). Its rows rotated into the base
///     must be the base task's rows — for the full mask and for partial masks.
///   - CLIK: the commanded TCP motion of the first step points along the error
///     in the base; a closed loop converges onto the target in the base; and
///     the SE3 overload gives the multi-frame overload's relative-task answer
///     (the relative Jacobian of a world-fixed base is R_bᵀ·J).

#include "clik_test_util.hpp"
#include "panda_fixture.hpp"
#include "rtc_tsid/kinematics/clik_reference.hpp"
#include "rtc_tsid/kinematics/se3_error.hpp"
#include "rtc_tsid/tasks/se3_task.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <span>
#include <vector>

namespace rtc::tsid {
namespace {

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

constexpr int kNv = 9;  // Panda: 7 arm + 2 finger

/// blockdiag(R, R).
[[nodiscard]] Mat6 BlockDiag(const Eigen::Matrix3d& R) {
  Mat6 out = Mat6::Zero();
  out.topLeftCorner<3, 3>() = R;
  out.bottomRightCorner<3, 3>() = R;
  return out;
}

class RotatedBaseTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = test::SharedPandaWithRotatedBase();
    ASSERT_EQ(model_->nv, kNv);
    robot_info_.Build(*model_, YAML::Node{});
    test::InitContactFreeCache(cache_, model_);
    contacts_.Init(0);
    ref_.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated, 0);

    q_home_.resize(kNv);
    q_home_ << 0.1, -0.6, 0.2, -2.1, 0.15, 1.7, 0.6, 0.02, 0.02;
    v_.resize(kNv);
    v_ << 0.3, -0.2, 0.25, 0.1, -0.35, 0.2, 0.15, 0.0, 0.0;
    R_wb_ = test::RotatedBaseRotation();
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  PinocchioCache cache_;
  ContactState contacts_;
  ControlReference ref_;
  Eigen::VectorXd q_home_;
  Eigen::VectorXd v_;
  Eigen::Matrix3d R_wb_;
};

// ── SE3Task ──────────────────────────────────────────────────────────────

class SE3TaskRotatedBaseTest : public RotatedBaseTest {
 protected:
  /// The rows of `task` on `cache` for the target `des` and feed-forwards
  /// in the task's base.
  void Rows(SE3Task& task, const PinocchioCache& cache, const pinocchio::SE3& des,
            const Vec6& v_des, const Vec6& a_ff, Eigen::MatrixXd& J, Eigen::VectorXd& r) {
    task.SetSe3Reference(des, v_des, a_ff);
    const int rows = task.ResidualDim();
    J = Eigen::MatrixXd::Zero(rows, kNv);
    r = Eigen::VectorXd::Zero(rows);
    task.ComputeResidual(cache, ref_, contacts_, kNv, J, r);
  }

  static YAML::Node Config(const char* base, const std::vector<int>& mask) {
    YAML::Node cfg;
    cfg["frame"] = "panda_hand";
    cfg["base_frame"] = base;
    cfg["mask"] = mask;
    // One gain per block: a per-axis gain acts along the BASE's axes, so a
    // task with different gains on one block is not the same task in the world.
    cfg["kp"] = std::vector<double>{100.0, 100.0, 100.0, 60.0, 60.0, 60.0};
    cfg["kd"] = std::vector<double>{20.0, 20.0, 20.0, 15.0, 15.0, 15.0};
    return cfg;
  }
};

TEST_F(SE3TaskRotatedBaseTest, RowsAreTheWorldTaskRotatedIntoTheBase) {
  const std::array<std::vector<int>, 4> masks = {
      std::vector<int>{1, 1, 1, 1, 1, 1}, std::vector<int>{1, 1, 1, 0, 0, 0},
      std::vector<int>{0, 0, 0, 1, 1, 1}, std::vector<int>{1, 0, 1, 0, 1, 0}};

  for (const auto& mask : masks) {
    PinocchioCache cache;
    test::InitContactFreeCache(cache, model_);
    SE3Task in_base;
    SE3Task in_world;
    in_base.Init(*model_, robot_info_, cache, Config(test::kRotatedBaseFrame, mask));
    in_world.Init(*model_, robot_info_, cache, Config("universe", {1, 1, 1, 1, 1, 1}));
    cache.Update(q_home_, v_);

    const auto& tip = cache.registered_frames[0];
    const auto& base = cache.registered_frames[1];
    ASSERT_EQ(base.name, test::kRotatedBaseFrame);
    ASSERT_TRUE(base.oMf.rotation().isApprox(R_wb_, 1e-15));
    ASSERT_GT(tip.dJv.norm(), 1e-3) << "the drift has to be non-zero to be judged";

    // Target: the TCP in the base, moved and turned; feed-forwards in the base.
    pinocchio::SE3 des_b = base.oMf.actInv(tip.oMf);
    des_b.translation() += Eigen::Vector3d(0.04, -0.03, 0.05);
    des_b.rotation() =
        des_b.rotation() *
        Eigen::AngleAxisd(0.25, Eigen::Vector3d(1.0, 2.0, -1.0).normalized()).toRotationMatrix();
    Vec6 v_des_b;
    v_des_b << 0.05, -0.02, 0.03, 0.1, -0.05, 0.08;
    Vec6 a_ff_b;
    a_ff_b << 0.4, 0.1, -0.3, 0.2, 0.3, -0.1;

    // The same task in the world.
    const pinocchio::SE3 des_w = base.oMf * des_b;
    const Mat6 Rb = BlockDiag(R_wb_);
    const Vec6 v_des_w = Rb * v_des_b;
    const Vec6 a_ff_w = Rb * a_ff_b;

    Eigen::MatrixXd J_b;
    Eigen::VectorXd r_b;
    Rows(in_base, cache, des_b, v_des_b, a_ff_b, J_b, r_b);
    Eigen::MatrixXd J_w;
    Eigen::VectorXd r_w;
    Rows(in_world, cache, des_w, v_des_w, a_ff_w, J_w, r_w);

    // The world rows in the base's axes, then the mask.
    const Eigen::MatrixXd J_wb = Rb.transpose() * J_w;
    const Eigen::VectorXd r_wb = Rb.transpose() * r_w;
    int row = 0;
    for (int i = 0; i < 6; ++i) {
      if (mask[static_cast<size_t>(i)] == 0) {
        continue;
      }
      const double j_scale = 1.0 + J_wb.row(i).norm();
      EXPECT_LT((J_b.row(row) - J_wb.row(i)).norm(), 1e-12 * j_scale) << "mask row " << i;
      EXPECT_NEAR(r_b(row), r_wb(i), 1e-12 * (1.0 + std::abs(r_wb(i)))) << "mask row " << i;
      ++row;
    }
    EXPECT_EQ(row, J_b.rows());
  }
}

TEST_F(SE3TaskRotatedBaseTest, TheDriftTermIsInTheBaseAxesToo) {
  // Zero gains and zero feed-forward leave −J̇·v alone in the residual: it
  // must be the world drift in the base's axes.
  SE3Task task;
  task.Init(*model_, robot_info_, cache_, Config(test::kRotatedBaseFrame, {1, 1, 1, 1, 1, 1}));
  cache_.Update(q_home_, v_);
  const auto& tip = cache_.registered_frames[0];
  const auto& base = cache_.registered_frames[1];
  task.SetGains(Vec6::Zero(), Vec6::Zero());
  Eigen::MatrixXd J;
  Eigen::VectorXd r;
  Rows(task, cache_, base.oMf.actInv(tip.oMf), Vec6::Zero(), Vec6::Zero(), J, r);

  const Vec6 drift_b = BlockDiag(R_wb_).transpose() * tip.dJv;
  ASSERT_GT(tip.dJv.norm(), 1e-3) << "the drift has to be non-zero to be judged";
  EXPECT_LT((r + drift_b).norm(), 1e-12 * (1.0 + drift_b.norm()));
}

// ── CLIK SE3 overload ────────────────────────────────────────────────────

class ClikRotatedBaseTest : public RotatedBaseTest {
 protected:
  void SetUp() override {
    RotatedBaseTest::SetUp();
    tcp_idx_ = cache_.RegisterFrame("panda_hand", model_->getFrameId("panda_hand"));
    base_idx_ =
        cache_.RegisterFrame(test::kRotatedBaseFrame, model_->getFrameId(test::kRotatedBaseFrame));
    ASSERT_GE(tcp_idx_, 0);
    ASSERT_GE(base_idx_, 0);
  }

  [[nodiscard]] static ClikReferenceGenerator::Config ClikConfig() {
    ClikReferenceGenerator::Config cfg;
    cfg.arm_v_idx = {0, 1, 2, 3, 4, 5, 6};
    cfg.hand_v_idx = {7, 8};
    cfg.damping_sq = 1e-6;
    cfg.v_limit = 0.0;  // no velocity box: the direction is the task's alone
    cfg.w_arm = 0.0;    // no posture pull either (it acts as extra damping)
    cfg.w_hand = 0.0;
    cfg.relative_tasks = true;
    return cfg;
  }

  [[nodiscard]] pinocchio::SE3 TcpInBase() const {
    const auto& tip = cache_.registered_frames[static_cast<size_t>(tcp_idx_)];
    const auto& base = cache_.registered_frames[static_cast<size_t>(base_idx_)];
    return base.oMf.actInv(tip.oMf);
  }

  int tcp_idx_{-1};
  int base_idx_{-1};
};

TEST_F(ClikRotatedBaseTest, TheFirstStepMovesTheTcpAlongTheErrorInTheBase) {
  ClikReferenceGenerator gen;
  gen.Init(kNv, ClikConfig());
  gen.SetTaskGain(Vec6::Constant(5.0));

  cache_.Update(q_home_, Eigen::VectorXd::Zero(kNv));
  pinocchio::SE3 des = TcpInBase();
  const Eigen::Vector3d offset_b(0.03, -0.02, 0.01);
  des.translation() += offset_b;

  ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, 0.01));

  // TCP linear velocity of the command, in the base's axes.
  const auto& tip = cache_.registered_frames[static_cast<size_t>(tcp_idx_)];
  const Eigen::Vector3d v_tcp_b = R_wb_.transpose() * (tip.J.topRows<3>() * gen.VRef());
  const double cos = v_tcp_b.dot(offset_b) / (v_tcp_b.norm() * offset_b.norm());
  RecordProperty("cos_to_error", test::Sci(cos));
  EXPECT_GT(cos, 0.999) << "v_tcp_b " << v_tcp_b.transpose();
}

TEST_F(ClikRotatedBaseTest, AClosedLoopConvergesOntoTheTargetInTheBase) {
  ClikReferenceGenerator gen;
  gen.Init(kNv, ClikConfig());
  gen.SetTaskGain(Vec6::Constant(5.0));

  Eigen::VectorXd q = q_home_;
  cache_.Update(q, Eigen::VectorXd::Zero(kNv));
  pinocchio::SE3 des = TcpInBase();
  des.translation() += Eigen::Vector3d(0.06, 0.04, -0.05);
  des.rotation() =
      des.rotation() * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();

  constexpr double kDt = 0.01;
  double first_error = 0.0;
  for (int k = 0; k < 400; ++k) {
    ASSERT_TRUE(gen.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, kDt)) << k;
    if (k == 0) {
      first_error = gen.TcpErrorNorm();
    }
    q = gen.QRef();
    cache_.Update(q, gen.VRef());
  }
  const Vec6 e = ComputeTaskPoseError(TcpInBase(), des);
  RecordProperty("final_error", test::Sci(e.norm()));
  EXPECT_GT(first_error, 0.05);
  EXPECT_LT(e.norm(), 1e-5);
}

TEST_F(ClikRotatedBaseTest, TheSe3OverloadIsTheRelativeTaskOnAFixedBase) {
  ClikReferenceGenerator single;
  ClikReferenceGenerator multi;
  single.Init(kNv, ClikConfig());
  multi.Init(kNv, ClikConfig());
  const Vec6 gain = (Vec6() << 5.0, 4.0, 6.0, 3.0, 2.5, 3.5).finished();
  single.SetTaskGain(gain);

  cache_.Update(q_home_, v_);
  pinocchio::SE3 des = TcpInBase();
  des.translation() += Eigen::Vector3d(0.05, -0.04, 0.03);
  des.rotation() =
      des.rotation() * Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Vec6 twist_ff;
  twist_ff << 0.02, 0.01, -0.03, 0.05, -0.02, 0.04;

  ASSERT_TRUE(single.Compute(cache_, tcp_idx_, base_idx_, des, q_home_, 0.01, true, &twist_ff));

  ClikReferenceGenerator::FrameTask task;
  task.kind = ClikReferenceGenerator::TaskKind::kSe3;
  task.frame_idx = tcp_idx_;
  task.base_frame_idx = base_idx_;
  task.placement_des = des;
  task.twist_ff = &twist_ff;
  task.gain = gain;
  ClikReferenceGenerator::MultiFrameInput in;
  in.tasks = std::span<const ClikReferenceGenerator::FrameTask>(&task, 1);
  in.q_posture_des = &q_home_;
  in.dt = 0.01;
  ASSERT_TRUE(multi.Compute(cache_, in));

  const double dv = (single.VRef() - multi.VRef()).norm();
  const double dq = (single.QRef() - multi.QRef()).norm();
  RecordProperty("v_ref_diff", test::Sci(dv));
  EXPECT_LT(dv, 1e-9);
  EXPECT_LT(dq, 1e-9);
  EXPECT_NEAR(single.TcpErrorNorm(), multi.TcpErrorNorm(), 1e-12);
  EXPECT_GT(single.VRef().norm(), 1e-2) << "a zero command would match trivially";
}

TEST_F(ClikRotatedBaseTest, TheUniverseBaseIsNotRotated) {
  // The same target stated in the world through the universe base: the
  // registered base and the universe give one command.
  ClikReferenceGenerator in_base;
  ClikReferenceGenerator in_world;
  in_base.Init(kNv, ClikConfig());
  in_world.Init(kNv, ClikConfig());
  in_base.SetTaskGain(Vec6::Constant(5.0));
  in_world.SetTaskGain(Vec6::Constant(5.0));

  cache_.Update(q_home_, v_);
  pinocchio::SE3 des_b = TcpInBase();
  des_b.translation() += Eigen::Vector3d(-0.03, 0.05, 0.02);
  const auto& base = cache_.registered_frames[static_cast<size_t>(base_idx_)];
  const pinocchio::SE3 des_w = base.oMf * des_b;
  Vec6 twist_b;
  twist_b << 0.01, 0.02, -0.01, 0.03, 0.0, -0.02;
  const Vec6 twist_w = BlockDiag(R_wb_) * twist_b;

  ASSERT_TRUE(in_base.Compute(cache_, tcp_idx_, base_idx_, des_b, q_home_, 0.01, true, &twist_b));
  ASSERT_TRUE(in_world.Compute(cache_, tcp_idx_, -1, des_w, q_home_, 0.01, true, &twist_w));
  EXPECT_LT((in_base.VRef() - in_world.VRef()).norm(), 1e-9);
}

}  // namespace
}  // namespace rtc::tsid
