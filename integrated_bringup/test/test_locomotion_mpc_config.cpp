// locomotion_controller.mpc.* (g1_loco): the installed fragment, composed by the CM's own loader
// (rtc::LoadControllerConfig — the `include:` merge included), loads through rtc_mpc's
// kinodynamic OCP config loader onto the MPC model — the shipped floating body with the hand
// locked — and creates the MPC through MPCFactory. Its weights are the reference whole-body
// task's: the state, terminal and input diagonals equal that task's Q, Q_final and R (the R
// scaling folded in), carried onto this repo's tangent by the layout adapter. The schema and every
// refusal are rtc_mpc's own tests; this one is the wiring and the transcription. No ROS node.
#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"
#include "integrated_bringup/controllers/locomotion/original_layout_adapter.hpp"
#include "rtc_controller_manager/controller_config_loader.hpp"
#include "rtc_mpc/handler/mpc_factory.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp_config.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "shipped_catching_bringup_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fx = integrated_bringup::testfx;
namespace loco = integrated_bringup::locomotion;

constexpr const char* kProfile = "g1_loco";

YAML::Node InstalledLocomotionConfig() {
  const std::string main = ament_index_cpp::get_package_share_directory("integrated_bringup") +
                           "/config/" + kProfile + "/controllers/locomotion_controller.yaml";
  return rtc::LoadControllerConfig(main, "locomotion_controller");
}

// The reference whole-body task's diagonals, in its own layout and order
// (base position x y z; Euler angles z, y, x; the 23 body joints; base linear
// velocity; Euler-angle rates z, y, x; joint rates).
constexpr std::array<double, 23> kQJoints = {
    0.05, 0.2, 0.2, 0.1, 0.1, 0.1,  // left leg: hip pitch, roll, yaw, knee, ankle pitch, roll
    0.05, 0.2, 0.2, 0.1, 0.1, 0.1,  // right leg
    2.0,  2.0, 2.0,                 // waist yaw, roll, pitch
    0.4,  2.0, 0.4, 0.4,            // left arm: shoulder pitch, roll, yaw, elbow
    0.4,  2.0, 0.4, 0.4};           // right arm
constexpr std::array<double, 23> kQfJoints = {
    0.05, 0.1, 0.1, 0.1, 0.1, 0.1,  // Q_final: hip roll and yaw 0.1
    0.05, 0.1, 0.1, 0.1, 0.1, 0.1,  //
    2.0,  2.0, 2.0,                 //
    0.4,  2.0, 0.4, 0.4,            //
    0.4,  2.0, 0.4, 0.4};
constexpr std::array<double, 23> kQJointRates = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001,  //
                                                 0.001, 0.001, 0.001, 0.001, 0.001, 0.001,  //
                                                 0.2,   0.2,   0.2,                         //
                                                 0.02,  0.02,  0.02,  0.02,                 //
                                                 0.02,  0.02,  0.02,  0.02};

Eigen::VectorXd ReferenceStateWeights(const std::array<double, 23>& joints) {
  Eigen::VectorXd w(58);
  w.head<6>() << 0.0, 0.0, 10.0, 0.0, 20.0, 20.0;  // p x y z; θ z y x
  for (int i = 0; i < 23; ++i) {
    w[6 + i] = joints[static_cast<std::size_t>(i)];
  }
  w.segment<6>(29) << 3.0, 3.0, 3.0, 3.0, 3.0, 3.0;  // v x y z; θ̇ z y x
  for (int i = 0; i < 23; ++i) {
    w[35 + i] = kQJointRates[static_cast<std::size_t>(i)];
  }
  return w;
}

// R: left wrench, right wrench (force x y z, moment x y z), then the 23 joint
// accelerations; the task's `scaling 1e-3` applied.
Eigen::VectorXd ReferenceInputWeights() {
  Eigen::VectorXd r(35);
  for (int c = 0; c < 2; ++c) {
    r.segment<6>(6 * c) << 0.003, 0.003, 0.001, 0.01, 0.01, 0.1;
  }
  r.tail<23>().setConstant(0.005);
  return r * 1e-3;
}

bool BitEqual(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  return a.size() == b.size() &&
         std::memcmp(a.data(), b.data(), sizeof(double) * static_cast<std::size_t>(a.size())) == 0;
}

class LocomotionMpcConfig : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = InstalledLocomotionConfig();
    ASSERT_TRUE(root_["mpc"].IsMap()) << "the include: merge did not bring mpc.* in";
    // The MPC model: the CM's floating model with every joint outside the body
    // roster (the hand) locked at its neutral value.
    const auto full = fx::ShippedModelBuilder(kProfile)->GetFullModel();
    ASSERT_NE(full, nullptr);
    const YAML::Node roster = fx::ShippedSimParams(kProfile)["devices"]["g1"]["joint_state_names"];
    ASSERT_TRUE(roster.IsSequence());
    for (const auto& j : roster) {
      body_.push_back(j.as<std::string>());
    }
    ASSERT_EQ(body_.size(), 23U);
    std::vector<pinocchio::JointIndex> lock;
    for (pinocchio::JointIndex j = 2; j < static_cast<pinocchio::JointIndex>(full->njoints); ++j) {
      if (std::find(body_.begin(), body_.end(), full->names[j]) == body_.end()) {
        lock.push_back(j);
      }
    }
    model_ = pinocchio::buildReducedModel(*full, lock, pinocchio::neutral(*full));
    const loco::HumanoidModelConfig cfg = loco::LoadHumanoidModelConfig(root_["model"], kProfile);
    ASSERT_EQ(handler_.Init(model_, loco::ToRobotModelHandlerYaml(cfg)),
              rtc::mpc::RobotModelInitError::kNoError);
  }

  YAML::Node root_;
  std::vector<std::string> body_;
  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
};

TEST_F(LocomotionMpcConfig, TheMpcModelIsTheFloatingBodyInRosterOrder) {
  EXPECT_EQ(model_.nv, 29);
  EXPECT_EQ(model_.nq, 30);
  EXPECT_TRUE(handler_.floating_base());
  EXPECT_EQ(handler_.nu(), 35);
  // The reference task's joint order is the roster's, which the weight
  // vectors below rely on.
  EXPECT_EQ(loco::JointOrder(model_), body_);
}

// The installed weights, laid onto the model, equal the reference task's Q,
// Q_final and R carried onto the tangent — exactly.
TEST_F(LocomotionMpcConfig, TheInstalledWeightsAreTheReferenceTasks) {
  rtc::mpc::KinodynamicOCPConfig cfg;
  ASSERT_EQ(rtc::mpc::LoadKinodynamicOCPConfig(root_["mpc"]["kinodynamic"], handler_, cfg),
            rtc::mpc::KinodynamicOCPConfigError::kNoError);
  Eigen::VectorXd q(58);
  Eigen::VectorXd qf(58);
  ASSERT_TRUE(loco::OriginalWeightsToTangent(ReferenceStateWeights(kQJoints), q));
  ASSERT_TRUE(loco::OriginalWeightsToTangent(ReferenceStateWeights(kQfJoints), qf));
  EXPECT_TRUE(BitEqual(cfg.state_weights, q));
  EXPECT_TRUE(BitEqual(cfg.terminal_weights, qf));
  EXPECT_TRUE(BitEqual(cfg.input_weights, ReferenceInputWeights()));
  EXPECT_EQ(cfg.terminal_scaling, 4.0);
  EXPECT_EQ(cfg.horizon_nodes, 32);
  EXPECT_EQ(cfg.dt, 0.035);
  EXPECT_EQ(cfg.integrator, rtc::mpc::KinodynamicIntegrator::kSemiImplicitEuler);
  // The foot weights the reference applies (its loader's, not its file's).
  Eigen::Matrix<double, 18, 1> foot;
  foot << 0.0, 0.0, 0.0, 1e4, 1e4, 0.0, 5.0, 5.0, 0.0, 2.0, 2.0, 2.0, 0.01, 0.01, 0.01, 0.01, 0.01,
      0.01;
  EXPECT_TRUE(BitEqual(cfg.contact_tracking_weights, foot));
}

// The installed contact constraints equal the reference task's: its
// foot_constraint gains, friction cone and CoP rectangle with their barriers, its joint-limit and
// collision barriers, and its 16 foot / knee sphere pairs rebuilt here from
// its definition (foot centre, centre ± 0.6·x_max along the sole, ankle; the
// knees) — with the sole frame of this model as the foot centre.
TEST_F(LocomotionMpcConfig, TheInstalledConstraintsAreTheReferenceTasks) {
  rtc::mpc::KinodynamicOCPConfig cfg;
  ASSERT_EQ(rtc::mpc::LoadKinodynamicOCPConfig(root_["mpc"]["kinodynamic"], handler_, cfg),
            rtc::mpc::KinodynamicOCPConfigError::kNoError);
  const rtc::mpc::KinodynamicConstraintsConfig& c = cfg.constraints;
  const rtc::mpc::FootConstraintGains& g = c.foot_constraint;
  EXPECT_EQ(g.position_z, 100.0);
  EXPECT_EQ(g.orientation, 80.0);
  EXPECT_EQ(g.linear_velocity_xy, 20.0);
  EXPECT_EQ(g.linear_velocity_z, 10.0);
  EXPECT_EQ(g.angular_velocity, 20.0);
  EXPECT_EQ(g.linear_acceleration_xy, 1.0);
  EXPECT_EQ(g.linear_acceleration_z, 1.0);
  EXPECT_EQ(g.angular_acceleration, 1.0);
  EXPECT_EQ(c.friction.mu, 0.4);
  EXPECT_EQ(c.friction.grip_force, 0.0);
  EXPECT_EQ(c.friction.regularization, 25.0);
  EXPECT_EQ(c.contact_rectangle.x_min, -0.09);
  EXPECT_EQ(c.contact_rectangle.x_max, 0.09);
  EXPECT_EQ(c.contact_rectangle.y_min, -0.03);
  EXPECT_EQ(c.contact_rectangle.y_max, 0.03);
  EXPECT_EQ(c.friction_barrier.mu, 0.2);
  EXPECT_EQ(c.friction_barrier.delta, 5.0);
  EXPECT_EQ(c.cop_barrier.mu, 0.6);
  EXPECT_EQ(c.cop_barrier.delta, 0.03);
  EXPECT_EQ(c.joint_limits.mu, 1200.0);
  EXPECT_EQ(c.joint_limits.delta, 0.1);
  EXPECT_FALSE(c.collision_enabled);
  EXPECT_EQ(c.collision.mu, 1500.0);
  EXPECT_EQ(c.collision.delta, 0.04);

  // The sole frames sit in their ankle-roll joint frames, unrotated.
  std::array<Eigen::Vector3d, 2> centre;
  const std::array<const char*, 2> side = {"left", "right"};
  for (std::size_t s = 0; s < 2; ++s) {
    const pinocchio::Frame& sole = model_.frames[model_.getFrameId(std::string(side[s]) + "_sole")];
    const pinocchio::Frame& ankle =
        model_.frames[model_.getFrameId(std::string(side[s]) + "_ankle_roll_joint")];
    ASSERT_EQ(sole.parentJoint, ankle.parentJoint);
    const pinocchio::SE3 rel = ankle.placement.actInv(sole.placement);
    EXPECT_TRUE(rel.rotation().isIdentity(1e-12));
    centre[s] = rel.translation();
  }

  struct Point {
    std::size_t side;
    char what;  // c centre, 1 / 2 the sole's front / back point, a ankle, k knee
  };

  const auto where = [&](const Point& p, std::string& frame, Eigen::Vector3d& offset, double& r) {
    const std::string s(side[p.side]);
    frame = s + (p.what == 'k' ? "_knee_joint" : "_ankle_roll_joint");
    r = p.what == 'k' ? 0.075 : 0.05;
    offset = Eigen::Vector3d::Zero();
    if (p.what == 'c' || p.what == '1' || p.what == '2') {
      offset = centre[p.side];
    }
    if (p.what == '1') {
      offset.x() += 0.6 * c.contact_rectangle.x_max;
    }
    if (p.what == '2') {
      offset.x() += 0.6 * c.contact_rectangle.x_min;
    }
  };
  const std::size_t L = 0;
  const std::size_t R = 1;
  const std::array<std::pair<Point, Point>, 16> expect = {{
      {{L, '1'}, {R, '1'}},
      {{L, '1'}, {R, '2'}},
      {{L, '2'}, {R, '1'}},
      {{L, '2'}, {R, '2'}},
      {{L, 'c'}, {R, '1'}},
      {{L, 'c'}, {R, '2'}},
      {{R, 'c'}, {L, '1'}},
      {{R, 'c'}, {L, '2'}},
      {{L, 'c'}, {R, 'c'}},
      {{L, 'k'}, {R, 'k'}},
      {{L, 'c'}, {R, 'a'}},
      {{L, '1'}, {R, 'a'}},
      {{L, '2'}, {R, 'a'}},
      {{R, 'c'}, {L, 'a'}},
      {{R, '1'}, {L, 'a'}},
      {{R, '2'}, {L, 'a'}},
  }};
  ASSERT_EQ(c.collision_pairs.size(), expect.size());
  for (std::size_t i = 0; i < expect.size(); ++i) {
    const rtc::mpc::SpherePair& got = c.collision_pairs[i];
    std::string fa;
    std::string fb;
    Eigen::Vector3d oa;
    Eigen::Vector3d ob;
    double ra = 0.0;
    double rb = 0.0;
    where(expect[i].first, fa, oa, ra);
    where(expect[i].second, fb, ob, rb);
    EXPECT_EQ(got.frame_a, model_.getFrameId(fa)) << "pair " << i;
    EXPECT_EQ(got.frame_b, model_.getFrameId(fb)) << "pair " << i;
    EXPECT_LE((got.offset_a - oa).cwiseAbs().maxCoeff(), 1e-12) << "pair " << i;
    EXPECT_LE((got.offset_b - ob).cwiseAbs().maxCoeff(), 1e-12) << "pair " << i;
    EXPECT_EQ(got.radius_a, ra) << "pair " << i;
    EXPECT_EQ(got.radius_b, rb) << "pair " << i;
  }
}

// The installed solver section: the tolerances the OCP's tests solve to, the
// standing budget and the (provisional) online one, the rest at its default.
TEST_F(LocomotionMpcConfig, TheInstalledSolverSectionIsTheRegisteredOne) {
  ASSERT_TRUE(root_["mpc"]["solver"].IsMap());
  rtc::mpc::MPCSolverConfig s;
  ASSERT_TRUE(rtc::mpc::MPCFactory::LoadSolverConfig(root_["mpc"]["solver"], s));
  EXPECT_EQ(s.prim_tol, 1e-4);
  EXPECT_EQ(s.dual_tol, 1e-4);
  EXPECT_EQ(s.mu_init, 1e-2);
  EXPECT_EQ(s.max_iters, 50);
  EXPECT_EQ(s.max_al_iters, 20);
  EXPECT_EQ(s.max_initial_iters, 300);
  EXPECT_EQ(s.max_online_al_iters, 20);
  EXPECT_EQ(s.rollout_type, rtc::mpc::MPCRolloutType::kLinear);
  EXPECT_EQ(s.linear_solver, rtc::mpc::MPCLinearSolver::kSerial);
  EXPECT_EQ(s.num_threads, 1);
  EXPECT_TRUE(s.force_initial_condition);
  EXPECT_FALSE(s.verbose);
}

TEST_F(LocomotionMpcConfig, TheFactoryCreatesTheMpcOnTheBodyModel) {
  rtc::mpc::PhaseContext ctx;
  ctx.ocp_type = "kinodynamic";
  ctx.cost_config.horizon_length = 32;
  ctx.cost_config.dt = 0.035;
  std::unique_ptr<rtc::mpc::MPCHandlerBase> mpc;
  const rtc::mpc::MPCFactoryStatus status = rtc::mpc::MPCFactory::Create(root_, handler_, ctx, mpc);
  ASSERT_EQ(status.error, rtc::mpc::MPCFactoryError::kNoError)
      << "init " << static_cast<int>(status.init_error) << " config "
      << static_cast<int>(status.ocp_config_error);
  ASSERT_NE(mpc, nullptr);
  EXPECT_EQ(mpc->ocp_type(), "kinodynamic");
  EXPECT_EQ(mpc->horizon_length(), 32);
  EXPECT_EQ(mpc->nu(), 35);
}

}  // namespace
