// locomotion_controller.mpc.* (g1_loco): the installed fragment, composed by the CM's own loader
// (rtc::LoadControllerConfig — the `include:` merge included), loads through rtc_mpc's
// kinodynamic OCP config loader onto the MPC model — the shipped floating body with the hand
// locked — and creates the MPC through MPCFactory. Weights, gains and solver settings are tuning:
// the tests hold the properties a run needs (finite, positive, sized to the model), not the values.
// The schema and every refusal are rtc_mpc's own tests; this one is the wiring. No ROS node.
#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"
#include "rtc_controller_manager/controller_config_loader.hpp"
#include "rtc_mpc/handler/mpc_factory.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp_config.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "shipped_catching_bringup_fixture.hpp"
#include "test_utils/shipped_walk_yaml.hpp"

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
#include <cmath>
#include <cstddef>
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

// Every entry finite and >= 0 (a weight diagonal; 0 is a valid "unweighted").
void ExpectNonNegativeFinite(const Eigen::VectorXd& w, const char* what) {
  for (Eigen::Index i = 0; i < w.size(); ++i) {
    EXPECT_TRUE(std::isfinite(w[i])) << what << "[" << i << "]";
    EXPECT_GE(w[i], 0.0) << what << "[" << i << "]";
  }
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

// The installed weights, laid onto the model, are what the OCP needs to run:
// one diagonal entry per tangent coordinate / input, finite and non-negative,
// at least one active, over a positive horizon. Their values are tuning, not
// pinned here.
TEST_F(LocomotionMpcConfig, TheInstalledWeightsFitTheModelAndAreUsable) {
  rtc::mpc::KinodynamicOCPConfig cfg;
  ASSERT_EQ(rtc::mpc::LoadKinodynamicOCPConfig(root_["mpc"]["kinodynamic"], handler_, cfg),
            rtc::mpc::KinodynamicOCPConfigError::kNoError);
  // State = tangent position (nv) + velocity (nv); input = 2 wrenches + joint accelerations.
  const Eigen::Index state_dim = 2 * model_.nv;
  EXPECT_EQ(cfg.state_weights.size(), state_dim);
  EXPECT_EQ(cfg.terminal_weights.size(), state_dim);
  EXPECT_EQ(cfg.input_weights.size(), static_cast<Eigen::Index>(handler_.nu()));
  ExpectNonNegativeFinite(cfg.state_weights, "state_weights");
  ExpectNonNegativeFinite(cfg.terminal_weights, "terminal_weights");
  ExpectNonNegativeFinite(cfg.input_weights, "input_weights");
  EXPECT_GT(cfg.state_weights.maxCoeff(), 0.0);
  EXPECT_GT(cfg.input_weights.minCoeff(), 0.0) << "an unweighted input leaves the Hessian singular";
  EXPECT_TRUE(std::isfinite(cfg.terminal_scaling));
  EXPECT_GT(cfg.terminal_scaling, 0.0);
  EXPECT_GT(cfg.horizon_nodes, 0);
  EXPECT_TRUE(std::isfinite(cfg.dt));
  EXPECT_GT(cfg.dt, 0.0);
  const double horizon_s = cfg.dt * cfg.horizon_nodes;
  EXPECT_GE(horizon_s, 0.1) << "a horizon shorter than a gait phase cannot plan a step";
  EXPECT_LE(horizon_s, 10.0);
  EXPECT_TRUE(cfg.integrator == rtc::mpc::KinodynamicIntegrator::kSemiImplicitEuler ||
              cfg.integrator == rtc::mpc::KinodynamicIntegrator::kRk2);
  ExpectNonNegativeFinite(cfg.contact_tracking_weights, "contact_tracking_weights");
}

// The installed contact constraints are usable and fit the model: gains and
// barrier parameters finite and positive, the friction cone and CoP rectangle
// non-degenerate, and the collision spheres rebuilt here from their definition
// (foot centre, centre ± 0.6·x_max along the sole, ankle; the knees) — with the
// sole frame of this model as the foot centre. Values are tuning, not pinned.
TEST_F(LocomotionMpcConfig, TheInstalledConstraintsAreUsableAndFitTheModel) {
  rtc::mpc::KinodynamicOCPConfig cfg;
  ASSERT_EQ(rtc::mpc::LoadKinodynamicOCPConfig(root_["mpc"]["kinodynamic"], handler_, cfg),
            rtc::mpc::KinodynamicOCPConfigError::kNoError);
  const rtc::mpc::KinodynamicConstraintsConfig& c = cfg.constraints;
  const rtc::mpc::FootConstraintGains& g = c.foot_constraint;
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  EXPECT_TRUE(positive(g.position_z));
  EXPECT_TRUE(positive(g.orientation));
  EXPECT_TRUE(positive(g.linear_velocity_xy));
  EXPECT_TRUE(positive(g.linear_velocity_z));
  EXPECT_TRUE(positive(g.angular_velocity));
  EXPECT_TRUE(positive(g.linear_acceleration_xy));
  EXPECT_TRUE(positive(g.linear_acceleration_z));
  EXPECT_TRUE(positive(g.angular_acceleration));
  EXPECT_TRUE(positive(c.friction.mu));
  EXPECT_LT(c.friction.mu, 2.0) << "a friction coefficient above 2 is not a floor contact";
  EXPECT_TRUE(std::isfinite(c.friction.grip_force));
  EXPECT_GE(c.friction.grip_force, 0.0);
  EXPECT_TRUE(positive(c.friction.regularization));
  EXPECT_TRUE(std::isfinite(c.contact_rectangle.x_min));
  EXPECT_TRUE(std::isfinite(c.contact_rectangle.x_max));
  EXPECT_TRUE(std::isfinite(c.contact_rectangle.y_min));
  EXPECT_TRUE(std::isfinite(c.contact_rectangle.y_max));
  EXPECT_LT(c.contact_rectangle.x_min, 0.0);
  EXPECT_GT(c.contact_rectangle.x_max, 0.0);
  EXPECT_LT(c.contact_rectangle.y_min, 0.0);
  EXPECT_GT(c.contact_rectangle.y_max, 0.0);
  EXPECT_TRUE(positive(c.friction_barrier.mu));
  EXPECT_TRUE(positive(c.friction_barrier.delta));
  EXPECT_TRUE(positive(c.cop_barrier.mu));
  EXPECT_TRUE(positive(c.cop_barrier.delta));
  EXPECT_TRUE(positive(c.joint_limits.mu));
  EXPECT_TRUE(positive(c.joint_limits.delta));
  if (c.collision_enabled) {
    EXPECT_TRUE(positive(c.collision.mu));
    EXPECT_TRUE(positive(c.collision.delta));
  }

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

  const double kSoleHalfLengthM = 0.09;
  EXPECT_LE(c.contact_rectangle.x_max, kSoleHalfLengthM) << "the CoP bound lies inside the sole";
  EXPECT_GE(c.contact_rectangle.x_min, -kSoleHalfLengthM);

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
    // The foot's geometry: 0.6 of the sole's half length either way — not of
    // the CoP rectangle, which is a bound inside the sole.
    if (p.what == '1') {
      offset.x() += 0.6 * kSoleHalfLengthM;
    }
    if (p.what == '2') {
      offset.x() -= 0.6 * kSoleHalfLengthM;
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

/// Whether two YAML trees carry the same values: numbers compared as doubles
/// (so `0.10` and `0.1` agree, a changed digit does not), other scalars as
/// text, sequences element-wise, maps key for key. Appends the first
/// difference to `where`.
bool SameValues(const YAML::Node& a, const YAML::Node& b, std::string& where) {
  if (a.Type() != b.Type()) {
    where += ": node kinds differ";
    return false;
  }
  if (a.IsScalar()) {
    double x = 0.0;
    double y = 0.0;
    if (YAML::convert<double>::decode(a, x) && YAML::convert<double>::decode(b, y)) {
      if (x == y) {
        return true;
      }
    } else if (a.Scalar() == b.Scalar()) {
      return true;
    }
    where += ": " + a.Scalar() + " vs " + b.Scalar();
    return false;
  }
  if (a.IsSequence()) {
    if (a.size() != b.size()) {
      where += ": sequence lengths differ";
      return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
      std::string at = where + "[" + std::to_string(i) + "]";
      if (!SameValues(a[i], b[i], at)) {
        where = at;
        return false;
      }
    }
    return true;
  }
  if (a.IsMap()) {
    if (a.size() != b.size()) {
      where += ": key counts differ";
      return false;
    }
    for (const auto& kv : a) {
      const std::string key = kv.first.as<std::string>();
      std::string at = where + "." + key;
      if (!b[key]) {
        where = at + ": missing";
        return false;
      }
      if (!SameValues(kv.second, b[key], at)) {
        where = at;
        return false;
      }
    }
    return true;
  }
  return true;  // null on both sides
}

// rtc_mpc's suites measure the walk on a copy of these four sections (it reads
// no profile file): the stage-parallel equivalence, the speed-up and the
// receding bounds are numbers about the shipped walk only while the copy is
// the installed profile. Same values, every key, both directions.
TEST_F(LocomotionMpcConfig, TheWalkRtcMpcMeasuresOnIsTheInstalledOne) {
  namespace tu = rtc::mpc::test_utils;
  const std::pair<const char*, const char*> sections[] = {
      {"gait", tu::kShippedGaitYaml},
      {"reference", tu::kShippedReferenceYaml},
      {"swing", tu::kShippedSwingYaml},
      {"terrain", tu::kShippedTerrainYaml},
  };
  for (const auto& [key, copy] : sections) {
    ASSERT_TRUE(root_[key].IsDefined()) << "the installed profile has no " << key << " section";
    YAML::Node installed = YAML::Clone(root_[key]);
    const YAML::Node copied = YAML::Load(copy);
    if (std::string(key) == "reference") {
      // q_default also names the joints the MPC model locks (the hand), which
      // the copy's model does not have: compare the body joints, and require
      // every other installed entry to be a joint outside the MPC model.
      const YAML::Node q_inst = installed["q_default"];
      const YAML::Node q_copy = copied["q_default"];
      ASSERT_TRUE(q_inst.IsMap() && q_copy.IsMap());
      EXPECT_EQ(q_copy.size(), body_.size()) << "the copy covers the body joints";
      for (const std::string& j : body_) {
        ASSERT_TRUE(q_inst[j] && q_copy[j]) << "q_default." << j;
        EXPECT_EQ(q_inst[j].as<double>(), q_copy[j].as<double>()) << "q_default." << j;
      }
      for (const auto& kv : q_inst) {
        const std::string j = kv.first.as<std::string>();
        if (std::find(body_.begin(), body_.end(), j) == body_.end()) {
          EXPECT_FALSE(model_.existJointName(j)) << "q_default." << j << " is in the MPC model";
        }
      }
      installed.remove("q_default");
      YAML::Node rest = YAML::Clone(copied);
      rest.remove("q_default");
      std::string where = key;
      EXPECT_TRUE(SameValues(installed, rest, where)) << where;
      continue;
    }
    std::string where = key;
    EXPECT_TRUE(SameValues(installed, copied, where)) << where;
  }
}

// The installed solver section loads and holds settings an online solve can
// run on: positive tolerances, a positive iteration budget, a thread count
// the loader accepted. The numbers are tuning and not pinned here; what is
// held is that rtc_mpc's receding tests measure on the installed budget
// (its copies, shipped_walk_yaml.hpp, like the gait sections above).
TEST_F(LocomotionMpcConfig, TheInstalledSolverSectionIsUsable) {
  ASSERT_TRUE(root_["mpc"]["solver"].IsMap());
  rtc::mpc::MPCSolverConfig s;
  ASSERT_TRUE(rtc::mpc::MPCFactory::LoadSolverConfig(root_["mpc"]["solver"], s));
  EXPECT_TRUE(std::isfinite(s.prim_tol));
  EXPECT_GT(s.prim_tol, 0.0);
  EXPECT_TRUE(std::isfinite(s.dual_tol));
  EXPECT_GT(s.dual_tol, 0.0);
  EXPECT_TRUE(std::isfinite(s.mu_init));
  EXPECT_GT(s.mu_init, 0.0);
  EXPECT_GE(s.max_iters, 1);
  EXPECT_GE(s.max_al_iters, 1);
  EXPECT_EQ(s.max_iters, rtc::mpc::test_utils::kShippedOnlineMaxIters)
      << "the copy rtc_mpc tests on";
  EXPECT_EQ(s.max_al_iters, rtc::mpc::test_utils::kShippedOnlineAlIters)
      << "the copy rtc_mpc tests on";
  EXPECT_GE(s.num_threads, 1);
  EXPECT_FALSE(s.verbose) << "a shipped solve does not print per iteration";
}

// The installed publish rules and period: the deadline is one period (the
// period's node intervals), the consumed-segment tolerance is a usable gate.
TEST_F(LocomotionMpcConfig, TheInstalledPublishRulesFollowThePeriod) {
  rtc::mpc::MPCSolverConfig s;
  ASSERT_TRUE(rtc::mpc::MPCFactory::LoadSolverConfig(root_["mpc"]["solver"], s));
  EXPECT_TRUE(std::isfinite(s.publish_prim_tol));
  EXPECT_GT(s.publish_prim_tol, 0.0);
  EXPECT_EQ(s.publish_prim_tol, rtc::mpc::test_utils::kShippedPublishPrimTol)
      << "the copy rtc_mpc tests on";
  const int period_nodes = root_["mpc"]["period_nodes"].as<int>();
  EXPECT_GE(period_nodes, 1);
  EXPECT_EQ(period_nodes, rtc::mpc::test_utils::kShippedPeriodNodes) << "the copy rtc_mpc tests on";
  EXPECT_NEAR(s.publish_deadline_ms,
              period_nodes * root_["mpc"]["kinodynamic"]["dt"].as<double>() * 1e3, 1e-9)
      << "the deadline is one period";
}

// The installed loop solves each state snapshot once: a stalled state
// stream does not keep a core busy re-solving the same observation.
TEST_F(LocomotionMpcConfig, TheInstalledLoopSolvesEachSnapshotOnce) {
  rtc::mpc::MPCSolverConfig s;
  EXPECT_FALSE(s.require_fresh_state) << "the default stays off";
  ASSERT_TRUE(rtc::mpc::MPCFactory::LoadSolverConfig(root_["mpc"]["solver"], s));
  EXPECT_TRUE(s.require_fresh_state);
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
