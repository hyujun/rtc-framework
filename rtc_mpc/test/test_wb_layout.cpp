/// @file test_wb_layout.cpp
/// @brief `WbLayout` is the one place the whole-body vector conventions are
///        written down: free-flyer `[p, q_xyzw]` first in `q`, LOCAL twist
///        first in `v`, wrenches then joint accelerations in `u`, and the
///        state-tangent difference used by the Riccati feedback. Checked on
///        the Panda built with a free-flyer root (nq = nv + 1) and on the
///        fixed-base Panda, with no robot-specific numbers in the suite —
///        every dimension is derived from the model.

#include "rtc_mpc/model/wb_layout.hpp"

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/types/contact_plan_types.hpp"
#include "test_utils/panda_fixture.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <limits>

namespace rtc::mpc {
namespace {

using test_utils::kPandaUrdf;

constexpr double kTol = 1e-12;

/// Handler YAML for a floating model: the base must be universe.
constexpr const char* kFloatingYaml = R"(
end_effector_frame: panda_hand
base_frame: universe
contact_frames:
  - name: panda_leftfinger
    dim: 3
  - name: panda_rightfinger
    dim: 3
)";

class WbLayoutFloatingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
    }
    model_ = test_utils::BuildPandaFreeFlyer();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(kFloatingYaml)), RobotModelInitError::kNoError);
    layout_ = WbLayout::FromHandler(handler_);
    ASSERT_GT(layout_.nq, 0);
  }

  /// A configuration with a unit random base orientation and arbitrary joints.
  Eigen::VectorXd RandomConfiguration(unsigned seed) const {
    std::srand(seed);
    Eigen::VectorXd q = pinocchio::randomConfiguration(model_, -Eigen::VectorXd::Ones(model_.nq),
                                                       Eigen::VectorXd::Ones(model_.nq));
    const Eigen::Quaterniond rot = Eigen::Quaterniond::UnitRandom();
    q.segment<4>(3) = rot.coeffs();  // xyzw, pinocchio's free-flyer order
    return q;
  }

  pinocchio::Model model_;
  RobotModelHandler handler_;
  WbLayout layout_{};
};

TEST_F(WbLayoutFloatingTest, DimensionsFollowTheModelAndTheContactFrames) {
  EXPECT_TRUE(layout_.floating_base());
  EXPECT_EQ(layout_.nq, model_.nq);
  EXPECT_EQ(layout_.nv, model_.nv);
  EXPECT_EQ(layout_.ndx, 2 * model_.nv);
  EXPECT_EQ(layout_.n_base_q, model_.joints[1].nq());
  EXPECT_EQ(layout_.n_base_v, model_.joints[1].nv());
  EXPECT_EQ(layout_.n_contacts, handler_.n_contacts());
  int total = 0;
  for (int i = 0; i < handler_.n_contacts(); ++i) {
    EXPECT_EQ(layout_.contact_u_offset[static_cast<std::size_t>(i)], total);
    EXPECT_EQ(layout_.contact_dim[static_cast<std::size_t>(i)],
              handler_.contact_frames()[static_cast<std::size_t>(i)].dim);
    total += handler_.contact_frames()[static_cast<std::size_t>(i)].dim;
  }
  EXPECT_EQ(layout_.n_contact_vars, total);
  EXPECT_EQ(layout_.n_wrench_u, total);
  EXPECT_EQ(layout_.acc_u_offset, total);
  EXPECT_EQ(layout_.n_acc, model_.nv - layout_.n_base_v);
  EXPECT_EQ(layout_.nu, total + model_.nv - layout_.n_base_v);
  EXPECT_EQ(layout_.nu, handler_.nu()) << "handler and layout agree on the kinodynamic input";
  EXPECT_EQ(handler_.layout().nu, layout_.nu);
  EXPECT_TRUE(handler_.layout().floating_base());
}

TEST_F(WbLayoutFloatingTest, ComposeAndSplitQAreBitExact) {
  const Eigen::VectorXd q = RandomConfiguration(7);
  Eigen::VectorXd base_pose(layout_.n_base_q);
  Eigen::VectorXd q_j(layout_.nq - layout_.n_base_q);
  ASSERT_TRUE(layout_.SplitQ(q, base_pose, q_j));
  EXPECT_EQ(base_pose, q.head(layout_.n_base_q));
  EXPECT_EQ(q_j, q.tail(layout_.nq - layout_.n_base_q));

  Eigen::VectorXd back(layout_.nq);
  ASSERT_TRUE(layout_.ComposeQ(base_pose, q_j, back));
  EXPECT_EQ(back, q) << "compose(split(q)) must be bit-exact";
}

TEST_F(WbLayoutFloatingTest, ComposeAndSplitVAndUAreBitExact) {
  std::srand(11);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(layout_.nv);
  Eigen::VectorXd twist(layout_.n_base_v);
  Eigen::VectorXd v_j(layout_.nv - layout_.n_base_v);
  ASSERT_TRUE(layout_.SplitV(v, twist, v_j));
  Eigen::VectorXd v_back(layout_.nv);
  ASSERT_TRUE(layout_.ComposeV(twist, v_j, v_back));
  EXPECT_EQ(v_back, v);

  const Eigen::VectorXd u = Eigen::VectorXd::Random(layout_.nu);
  Eigen::VectorXd wrench(layout_.n_contact_vars);
  Eigen::VectorXd qdd(layout_.n_acc);
  ASSERT_TRUE(layout_.SplitU(u, wrench, qdd));
  EXPECT_EQ(wrench, u.head(layout_.n_contact_vars));
  EXPECT_EQ(qdd, u.tail(layout_.n_acc));
  Eigen::VectorXd u_back(layout_.nu);
  ASSERT_TRUE(layout_.ComposeU(wrench, qdd, u_back));
  EXPECT_EQ(u_back, u);
}

TEST_F(WbLayoutFloatingTest, StateDifferenceMatchesPinocchioAndIsFinite) {
  const Eigen::VectorXd q_ref = RandomConfiguration(21);
  const Eigen::VectorXd q_cur = RandomConfiguration(22);
  std::srand(23);
  const Eigen::VectorXd v_ref = Eigen::VectorXd::Random(layout_.nv);
  const Eigen::VectorXd v_cur = Eigen::VectorXd::Random(layout_.nv);

  Eigen::VectorXd dx(layout_.ndx);
  ASSERT_TRUE(layout_.StateDifference(model_, q_ref, v_ref, q_cur, v_cur, dx));
  EXPECT_TRUE(dx.allFinite());

  const Eigen::VectorXd dq_expected = pinocchio::difference(model_, q_ref, q_cur);
  EXPECT_LT((dx.head(layout_.nv) - dq_expected).norm(), kTol);
  EXPECT_LT((dx.tail(layout_.nv) - (v_cur - v_ref)).norm(), kTol);

  // Sign: integrating the reference by dx.head(nv) lands on the current one.
  const Eigen::VectorXd back =
      pinocchio::integrate(model_, q_ref, Eigen::VectorXd(dx.head(layout_.nv)));
  EXPECT_LT(pinocchio::difference(model_, back, q_cur).norm(), 1e-9);
}

TEST_F(WbLayoutFloatingTest, AntipodalQuaternionsAreTheSameRotation) {
  // q and −q encode one orientation: the tangent difference must vanish.
  Eigen::VectorXd q_ref = RandomConfiguration(51);
  Eigen::VectorXd q_cur = q_ref;
  q_cur.segment<4>(3) = -q_ref.segment<4>(3);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(layout_.nv);
  Eigen::VectorXd dx(layout_.ndx);
  ASSERT_TRUE(layout_.StateDifference(model_, q_ref, v, q_cur, v, dx));
  EXPECT_LT(dx.norm(), 1e-9);
}

TEST_F(WbLayoutFloatingTest, NonUnitQuaternionIsRefused) {
  // pinocchio would return a finite but wrong step for a non-unit quaternion;
  // the layout refuses before calling it, leaving the output untouched.
  const Eigen::VectorXd q_ref = RandomConfiguration(61);
  Eigen::VectorXd q_bad = q_ref;
  q_bad.segment<4>(3) *= 1.01;
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(layout_.nv);
  Eigen::VectorXd dx = Eigen::VectorXd::Constant(layout_.ndx, -3.0);
  EXPECT_FALSE(layout_.StateDifference(model_, q_ref, v, q_bad, v, dx));
  EXPECT_FALSE(layout_.StateDifference(model_, q_bad, v, q_ref, v, dx));
  EXPECT_TRUE((dx.array() == -3.0).all());

  Eigen::VectorXd q_nan = q_ref;
  q_nan(4) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(layout_.StateDifference(model_, q_ref, v, q_nan, v, dx));
}

TEST_F(WbLayoutFloatingTest, TranslationDifferenceIsInTheReferenceLocalFrame) {
  // Same orientation (yawed), different position: the free-flyer part of dx
  // is the displacement seen from the reference body, R_refᵀ (p_cur − p_ref).
  Eigen::VectorXd q_ref = pinocchio::neutral(model_);
  const Eigen::Quaterniond yaw(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  q_ref.segment<4>(3) = yaw.coeffs();
  Eigen::VectorXd q_cur = q_ref;
  const Eigen::Vector3d step(0.3, -0.1, 0.05);
  q_cur.head<3>() += step;
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(layout_.nv);
  Eigen::VectorXd dx(layout_.ndx);
  ASSERT_TRUE(layout_.StateDifference(model_, q_ref, v, q_cur, v, dx));
  const Eigen::Vector3d expected = yaw.toRotationMatrix().transpose() * step;
  EXPECT_LT((dx.head<3>() - expected).norm(), 1e-9) << "LOCAL (reference) frame, not world";
  EXPECT_GT((dx.head<3>() - step).norm(), 0.1) << "a world displacement would differ here";
  EXPECT_LT(dx.segment<3>(3).norm(), 1e-9) << "no rotation change";
}

TEST_F(WbLayoutFloatingTest, SizeMismatchReturnsFalseWithoutWriting) {
  const Eigen::VectorXd q = RandomConfiguration(31);
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(layout_.nv);
  Eigen::VectorXd dx = Eigen::VectorXd::Constant(layout_.ndx, -7.0);

  const Eigen::VectorXd q_short = q.head(layout_.nq - 1);
  EXPECT_FALSE(layout_.StateDifference(model_, q_short, v, q, v, dx));
  EXPECT_FALSE(layout_.StateDifference(model_, q, v, q, v, dx.head(layout_.ndx - 1)));
  EXPECT_TRUE((dx.array() == -7.0).all()) << "a refused call leaves the output untouched";

  Eigen::VectorXd base(layout_.n_base_q - 1);
  Eigen::VectorXd q_j(layout_.nq - layout_.n_base_q);
  EXPECT_FALSE(layout_.SplitQ(q, base, q_j));
  Eigen::VectorXd u(layout_.nu + 1);
  EXPECT_FALSE(layout_.ComposeU(Eigen::VectorXd::Zero(layout_.n_contact_vars),
                                Eigen::VectorXd::Zero(layout_.n_acc), u));
}

TEST_F(WbLayoutFloatingTest, BaseTwistIsTheRootJointVelocityInLocalFrame) {
  // Yaw the base: a LOCAL twist and a world-aligned twist then differ, so the
  // check pins the convention and not just the slot.
  Eigen::VectorXd q = pinocchio::neutral(model_);
  const Eigen::Quaterniond yaw(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  q.segment<4>(3) = yaw.coeffs();

  Eigen::VectorXd twist(layout_.n_base_v);
  twist << 1.0, 0.0, 0.0, 0.0, 0.0, 0.5;
  const Eigen::VectorXd v_j = Eigen::VectorXd::Zero(layout_.nv - layout_.n_base_v);
  Eigen::VectorXd v(layout_.nv);
  ASSERT_TRUE(layout_.ComposeV(twist, v_j, v));

  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, q, v);
  const pinocchio::FrameIndex root = model_.getFrameId("root_joint");
  ASSERT_LT(root, model_.frames.size());

  const pinocchio::Motion local = pinocchio::getFrameVelocity(model_, data, root, pinocchio::LOCAL);
  EXPECT_LT((local.toVector() - twist).norm(), kTol) << "v.head(6) is the LOCAL root twist";

  const pinocchio::Motion aligned =
      pinocchio::getFrameVelocity(model_, data, root, pinocchio::LOCAL_WORLD_ALIGNED);
  EXPECT_GT((aligned.linear() - twist.head<3>()).norm(), 0.5)
      << "with a yawed base the world-aligned linear velocity differs";
}

// ── Fixed base: the same type, no base block, Euclidean difference ──────

class WbLayoutFixedTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
    }
    pinocchio::urdf::buildModel(kPandaUrdf, model_);
    ASSERT_EQ(handler_.Init(model_, YAML::Load(test_utils::kPandaTwoFingerYaml)),
              RobotModelInitError::kNoError);
  }

  pinocchio::Model model_;
  RobotModelHandler handler_;
};

TEST_F(WbLayoutFixedTest, FixedLayoutHasNoBaseBlockAndTorqueInput) {
  const WbLayout from_handler = WbLayout::FromHandler(handler_);
  EXPECT_FALSE(from_handler.floating_base());
  EXPECT_EQ(from_handler.n_base_q, 0);
  EXPECT_EQ(from_handler.n_base_v, 0);
  EXPECT_EQ(from_handler.nu, model_.nv);
  EXPECT_EQ(from_handler.n_wrench_u, 0) << "a torque input has no wrench block";
  EXPECT_EQ(from_handler.acc_u_offset, 0);
  EXPECT_EQ(from_handler.n_acc, model_.nv);
  int summed = 0;
  for (const ContactFrameInfo& f : handler_.contact_frames()) {
    summed += f.dim;
  }
  EXPECT_EQ(from_handler.n_contact_vars, summed);

  const WbLayout fixed = WbLayout::FixedBase(model_.nq, model_.nv, from_handler.n_contact_vars);
  EXPECT_EQ(fixed.nq, from_handler.nq);
  EXPECT_EQ(fixed.nv, from_handler.nv);
  EXPECT_EQ(fixed.nu, from_handler.nu);
  EXPECT_EQ(fixed.n_contact_vars, from_handler.n_contact_vars);

  const WbLayout from_model = WbLayout::FromModel(model_, from_handler.n_contact_vars);
  EXPECT_EQ(from_model.nu, model_.nv);
  EXPECT_EQ(from_model.n_contacts, 0) << "total-only layout carries no split";
  EXPECT_EQ(from_model.n_contact_vars, from_handler.n_contact_vars);
}

TEST_F(WbLayoutFixedTest, StateDifferenceIsEuclideanOnAFixedBase) {
  const WbLayout layout = WbLayout::FromHandler(handler_);
  std::srand(41);
  const Eigen::VectorXd q_ref = Eigen::VectorXd::Random(layout.nq);
  const Eigen::VectorXd q_cur = Eigen::VectorXd::Random(layout.nq);
  const Eigen::VectorXd v_ref = Eigen::VectorXd::Random(layout.nv);
  const Eigen::VectorXd v_cur = Eigen::VectorXd::Random(layout.nv);
  Eigen::VectorXd dx(layout.ndx);
  ASSERT_TRUE(layout.StateDifference(model_, q_ref, v_ref, q_cur, v_cur, dx));
  EXPECT_LT((dx.head(layout.nv) - (q_cur - q_ref)).norm(), kTol)
      << "revolute / prismatic joints: plain subtraction";
  EXPECT_EQ(dx.tail(layout.nv), v_cur - v_ref);

  // Fixed-base U has no wrench block: the whole input is the joint part.
  const Eigen::VectorXd u = Eigen::VectorXd::Random(layout.nu);
  Eigen::VectorXd wrench(0);
  Eigen::VectorXd qdd(layout.n_acc);
  ASSERT_TRUE(layout.SplitU(u, wrench, qdd));
  EXPECT_EQ(qdd, u);
}

TEST(WbLayoutStandalone, InvalidInputsYieldAnEmptyLayout) {
  const WbLayout empty_layout = WbLayout::FixedBase(0, 3, 0);
  EXPECT_FALSE(empty_layout.valid());
  Eigen::VectorXd none(0);
  EXPECT_FALSE(empty_layout.ComposeQ(none, none, none)) << "an invalid layout refuses every call";
  EXPECT_FALSE(empty_layout.SplitU(none, none, none));
  EXPECT_EQ(WbLayout::FixedBase(0, 3, 0).nq, 0);
  EXPECT_EQ(WbLayout::FixedBase(3, 3, kMaxContactVars + 1).nq, 0);
  pinocchio::Model empty;
  EXPECT_FALSE(IsFloatingBaseModel(empty));
  EXPECT_EQ(WbLayout::FromModel(empty, 0).nq, 0) << "a model without joints has no layout";
}

}  // namespace
}  // namespace rtc::mpc
