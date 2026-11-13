/// @file test_kinodynamic_ocp.cpp
/// @brief The kinodynamic OCP on the public G1 (free-flyer, both soles 6-D):
///        the `mpc.kinodynamic:` loader (every refusal), where each weight
///        lands, the per-tick mode and reference binding (a walk window from
///        the stage-reference builder), the stage and terminal cost against an
///        independent sum, and solves through the handler core (full stance;
///        one foot swinging), on both integrators. Reports (no gate) the heap
///        of the built problem plus the solver's workspace, one solver
///        iteration's time and one residual evaluation's time.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/handler/kinodynamic_mpc.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/ocp/barrier_costs.hpp"
#include "rtc_mpc/ocp/contact_constraints.hpp"
#include "rtc_mpc/ocp/contact_tracking_residual.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp_config.hpp"
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "rtc_mpc/reference/stage_reference_builder.hpp"
#include "rtc_mpc/reference/surface_frame.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_fixture.hpp"
#include "test_utils/kinodynamic_ocp_fixture.hpp"
#include "test_utils/loco_reference_fixture.hpp"
#include "test_utils/panda_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/costs/quad-residual-cost.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>
#include <malloc.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

namespace tu = rtc::mpc::test_utils;
using rtc::mpc::CollisionBarrierCost;
using rtc::mpc::ContactConstraintResidual;
using rtc::mpc::ContactTrackingResidual;
using rtc::mpc::ContactWrenchConeResidual;
using rtc::mpc::HorizonReference;
using rtc::mpc::JointLimitBarrierCost;
using rtc::mpc::KinodynamicDynamics;
using rtc::mpc::KinodynamicIntegrator;
using rtc::mpc::KinodynamicMPC;
using rtc::mpc::KinodynamicOCP;
using rtc::mpc::KinodynamicOCPConfig;
using rtc::mpc::KinodynamicOCPConfigError;
using rtc::mpc::LoadKinodynamicOCPConfig;
using rtc::mpc::OCPBuildError;
using rtc::mpc::PhaseContext;
using CostStack = aligator::CostStackTpl<double>;
using QuadResidualCost = aligator::QuadraticResidualCostTpl<double>;
using Err = KinodynamicOCPConfigError;

constexpr int kRows = ContactTrackingResidual::kRowsPerContact;

/// Bitwise equality of two vectors (sizes included).
bool BitEqual(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  return a.size() == b.size() &&
         std::memcmp(a.data(), b.data(), sizeof(double) * static_cast<std::size_t>(a.size())) == 0;
}

/// As `tu::SwingContact` (mode bits, wrench redistribution, impact
/// proximity), with a 1 − cos height reference instead of the half sine:
/// `z = z0 + (h/2)(1 − cos(2π t/T))` with ż, z̈, `T = (last − first + 2)·dt`,
/// `t = (k − first + 1)·dt` — zero vertical velocity at liftoff and touchdown.
void SwingContactCos(const rtc::mpc::RobotModelHandler& handler, HorizonReference& h, int swing,
                     int first, int last, double height, double impact) {
  tu::SwingContact(handler, h, swing, first, last, height, impact);
  const double T = (last - first + 2) * h.dt;
  const double w = 2.0 * std::acos(-1.0) / T;
  const auto i = static_cast<std::size_t>(swing);
  const double z0 = h.stages[0].contact_pose[7 * i + 2];
  for (int k = first; k <= last; ++k) {
    rtc::mpc::StageReference& s = h.stages[static_cast<std::size_t>(k)];
    const double t = (k - first + 1) * h.dt;
    s.contact_pose[7 * i + 2] = z0 + 0.5 * height * (1.0 - std::cos(w * t));
    s.contact_twist[6 * i + 2] = 0.5 * height * w * std::sin(w * t);
    s.contact_acceleration[6 * i + 2] = 0.5 * height * w * w * std::cos(w * t);
  }
}

class KinodynamicOCPTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(tu::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << tu::kG1Urdf;
    }
    model_ = tu::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(tu::kG1SolesYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    ASSERT_EQ(LoadKinodynamicOCPConfig(tu::G1KinodynamicNode(), handler_, cfg_), Err::kNoError);
    stand_ = tu::G1Stand(model_);
  }

  [[nodiscard]] Err Load(const YAML::Node& n) {
    KinodynamicOCPConfig out;
    return LoadKinodynamicOCPConfig(n, handler_, out);
  }

  [[nodiscard]] PhaseContext Context(const HorizonReference* h = nullptr) const {
    PhaseContext ctx;
    ctx.ocp_type = std::string(KinodynamicOCP::kType);
    ctx.cost_config.horizon_length = tu::kKinoNodes;
    ctx.cost_config.dt = tu::kKinoDt;
    ctx.horizon = h;
    return ctx;
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  KinodynamicOCPConfig cfg_;
  Eigen::VectorXd stand_;
};

// ── Loader ──────────────────────────────────────────────────────────────────

TEST_F(KinodynamicOCPTest, LoadsTheG1ConfigSizedByTheModel) {
  EXPECT_EQ(cfg_.horizon_nodes, 32);
  EXPECT_EQ(cfg_.dt, 0.035);
  EXPECT_EQ(cfg_.integrator, KinodynamicIntegrator::kSemiImplicitEuler);
  EXPECT_EQ(cfg_.terminal_scaling, 4.0);
  EXPECT_EQ(cfg_.state_weights.size(), 2 * model_.nv);
  EXPECT_EQ(cfg_.terminal_weights.size(), 2 * model_.nv);
  EXPECT_EQ(cfg_.input_weights.size(), handler_.nu());
  EXPECT_EQ(cfg_.state_weights.size(), 58);
  EXPECT_EQ(cfg_.input_weights.size(), 35);
}

TEST_F(KinodynamicOCPTest, EveryKeyIsRequired) {
  const char* top[] = {"horizon_nodes", "dt",
                       "integrator",    "terminal_scaling",
                       "state_weights", "terminal_weights",
                       "input_weights", "contact_tracking",
                       "constraints"};
  for (const char* k : top) {
    YAML::Node n = YAML::Clone(tu::G1KinodynamicNode());
    n.remove(k);
    EXPECT_EQ(Load(n), Err::kMissingKey) << k;
  }
  const char* state[] = {"base_position",        "base_orientation",      "joints",
                         "base_linear_velocity", "base_angular_velocity", "joint_velocities"};
  for (const char* map : {"state_weights", "terminal_weights"}) {
    for (const char* k : state) {
      YAML::Node n = YAML::Clone(tu::G1KinodynamicNode());
      n[map].remove(k);
      EXPECT_EQ(Load(n), Err::kMissingKey) << map << "." << k;
    }
  }
  for (const char* k : {"contact_force", "contact_moment", "joint_accelerations"}) {
    YAML::Node n = YAML::Clone(tu::G1KinodynamicNode());
    n["input_weights"].remove(k);
    EXPECT_EQ(Load(n), Err::kMissingKey) << k;
  }
  for (const char* k : {"position", "orientation", "linear_velocity", "angular_velocity",
                        "linear_acceleration", "angular_acceleration"}) {
    YAML::Node n = YAML::Clone(tu::G1KinodynamicNode());
    n["contact_tracking"].remove(k);
    EXPECT_EQ(Load(n), Err::kMissingKey) << k;
  }
  YAML::Node null_dt = YAML::Clone(tu::G1KinodynamicNode());
  null_dt["dt"] = YAML::Node(YAML::NodeType::Null);
  EXPECT_EQ(Load(null_dt), Err::kMissingKey);
  EXPECT_EQ(Load(YAML::Node()), Err::kMissingKey);
  EXPECT_EQ(Load(YAML::Load("[1, 2]")), Err::kNotAMap);
}

TEST_F(KinodynamicOCPTest, AnUnknownKeyIsRefusedAtEveryLevel) {
  for (const char* map :
       {"", "state_weights", "terminal_weights", "input_weights", "contact_tracking"}) {
    YAML::Node n = YAML::Clone(tu::G1KinodynamicNode());
    if (std::strlen(map) == 0) {
      n["horizon"] = 32;
    } else {
      n[map]["typo"] = 1.0;
    }
    EXPECT_EQ(Load(n), Err::kUnknownKey) << map;
  }
}

TEST_F(KinodynamicOCPTest, NegativeNonFiniteAndOutOfRangeValuesAreRefused) {
  const auto with = [](const char* path_a, const char* path_b, const YAML::Node& v) {
    YAML::Node n = YAML::Clone(tu::G1KinodynamicNode());
    if (path_b == nullptr) {
      n[path_a] = v;
    } else {
      n[path_a][path_b] = v;
    }
    return n;
  };
  EXPECT_EQ(Load(with("state_weights", "base_position", YAML::Load("[0, -1, 0]"))),
            Err::kNegativeWeight);
  EXPECT_EQ(Load(with("terminal_weights", "base_orientation", YAML::Load("[0, .nan, 0]"))),
            Err::kNonFinite);
  EXPECT_EQ(Load(with("input_weights", "contact_moment", YAML::Load("[0, .inf, 0]"))),
            Err::kNonFinite);
  EXPECT_EQ(Load(with("contact_tracking", "orientation", YAML::Load("[1, 1, -1e-9]"))),
            Err::kNegativeWeight);
  EXPECT_EQ(Load(with("contact_tracking", "position", YAML::Load("[1, 1]"))), Err::kWrongType);
  EXPECT_EQ(Load(with("terminal_scaling", nullptr, YAML::Node(-1.0))), Err::kNegativeWeight);
  EXPECT_EQ(Load(with("terminal_scaling", nullptr, YAML::Load(".nan"))), Err::kNonFinite);
  EXPECT_EQ(Load(with("dt", nullptr, YAML::Node(0.0))), Err::kInvalidDt);
  EXPECT_EQ(Load(with("dt", nullptr, YAML::Node(-0.01))), Err::kInvalidDt);
  EXPECT_EQ(Load(with("horizon_nodes", nullptr, YAML::Node(0))), Err::kInvalidHorizon);
  EXPECT_EQ(Load(with("horizon_nodes", nullptr, YAML::Node(rtc::mpc::kMaxHorizon + 1))),
            Err::kInvalidHorizon);
  EXPECT_EQ(Load(with("horizon_nodes", nullptr, YAML::Node(1.5))), Err::kWrongType);
  // A joint weight: negative, non-finite, or not a number.
  YAML::Node neg = YAML::Clone(tu::G1KinodynamicNode());
  neg["state_weights"]["joints"]["left_knee_joint"] = -0.1;
  EXPECT_EQ(Load(neg), Err::kNegativeWeight);
  YAML::Node nan = YAML::Clone(tu::G1KinodynamicNode());
  nan["input_weights"]["joint_accelerations"]["waist_yaw_joint"] = YAML::Load(".nan");
  EXPECT_EQ(Load(nan), Err::kNonFinite);
  YAML::Node map = YAML::Clone(tu::G1KinodynamicNode());
  map["state_weights"]["joint_velocities"] = YAML::Load("[1, 2]");
  EXPECT_EQ(Load(map), Err::kNotAMap);
}

TEST_F(KinodynamicOCPTest, AMissingModelJointIsRefusedAndAForeignNameIgnored) {
  for (const auto& [a, b] :
       std::vector<std::pair<const char*, const char*>>{{"state_weights", "joints"},
                                                        {"state_weights", "joint_velocities"},
                                                        {"terminal_weights", "joints"},
                                                        {"terminal_weights", "joint_velocities"},
                                                        {"input_weights", "joint_accelerations"}}) {
    YAML::Node n = YAML::Clone(tu::G1KinodynamicNode());
    n[a][b].remove("right_elbow_joint");
    EXPECT_EQ(Load(n), Err::kMissingJoint) << a << "." << b;
  }
  // A name the model lacks changes nothing — but its value is still checked.
  YAML::Node extra = YAML::Clone(tu::G1KinodynamicNode());
  extra["state_weights"]["joints"]["thumb_mcp_joint"] = 7.0;
  extra["input_weights"]["joint_accelerations"]["thumb_mcp_joint"] = 7.0;
  KinodynamicOCPConfig out;
  ASSERT_EQ(LoadKinodynamicOCPConfig(extra, handler_, out), Err::kNoError);
  EXPECT_TRUE(BitEqual(out.state_weights, cfg_.state_weights));
  EXPECT_TRUE(BitEqual(out.input_weights, cfg_.input_weights));
  extra["state_weights"]["joints"]["thumb_mcp_joint"] = -7.0;
  EXPECT_EQ(Load(extra), Err::kNegativeWeight);
}

TEST_F(KinodynamicOCPTest, TheIntegratorIsOneOfTwoNames) {
  KinodynamicOCPConfig out;
  ASSERT_EQ(LoadKinodynamicOCPConfig(tu::G1KinodynamicNode("rk2"), handler_, out), Err::kNoError);
  EXPECT_EQ(out.integrator, KinodynamicIntegrator::kRk2);
  EXPECT_EQ(Load(tu::G1KinodynamicNode("rk4")), Err::kUnknownIntegrator);
  YAML::Node seq = tu::G1KinodynamicNode();
  seq["integrator"] = YAML::Load("[rk2]");
  EXPECT_EQ(Load(seq), Err::kWrongType);
}

TEST_F(KinodynamicOCPTest, AFixedBaseOrUninitialisedModelIsRefused) {
  rtc::mpc::RobotModelHandler none;
  KinodynamicOCPConfig out;
  EXPECT_EQ(LoadKinodynamicOCPConfig(tu::G1KinodynamicNode(), none, out),
            Err::kModelNotInitialised);
  if (!std::filesystem::exists(tu::kPandaUrdf)) {
    GTEST_SKIP() << "Panda URDF not found at " << tu::kPandaUrdf;
  }
  pinocchio::Model panda;
  pinocchio::urdf::buildModel(tu::kPandaUrdf, panda);
  rtc::mpc::RobotModelHandler fixed;
  ASSERT_EQ(fixed.Init(panda, YAML::Load(tu::kPandaTwoFingerYaml)),
            rtc::mpc::RobotModelInitError::kNoError);
  EXPECT_EQ(LoadKinodynamicOCPConfig(tu::G1KinodynamicNode(), fixed, out), Err::kFixedBase);
}

// ── Loader: constraints ─────────────────────────────────────────────────────

/// The keys of each `constraints:` sub-map.
const std::vector<std::pair<const char*, std::vector<const char*>>>& ConstraintSections() {
  static const std::vector<std::pair<const char*, std::vector<const char*>>> kSections = {
      {"foot_constraint",
       {"position_z", "orientation", "linear_velocity_xy", "linear_velocity_z", "angular_velocity",
        "linear_acceleration_xy", "linear_acceleration_z", "angular_acceleration"}},
      {"friction", {"mu", "grip_force", "regularization"}},
      {"contact_rectangle", {"x_min", "x_max", "y_min", "y_max"}},
      {"joint_limits", {"mu", "delta"}},
      {"collision", {"enabled", "mu", "delta", "pairs"}},
  };
  return kSections;
}

/// The keys of one collision pair.
const std::vector<const char*>& PairKeys() {
  static const std::vector<const char*> kKeys = {"frame_a", "offset_a", "radius_a",
                                                 "frame_b", "offset_b", "radius_b"};
  return kKeys;
}

/// The fixture node with `constraints.<section>.<key>` set to @p v.
YAML::Node WithConstraint(const char* section, const char* key, const YAML::Node& v) {
  YAML::Node n = YAML::Clone(tu::G1KinodynamicNode());
  n["constraints"][section][key] = v;
  return n;
}

/// The fixture node with `constraints.collision.pairs[pair].<key>` set to @p v.
YAML::Node WithPairValue(std::size_t pair, const char* key, const YAML::Node& v) {
  YAML::Node n = YAML::Clone(tu::G1KinodynamicNode());
  n["constraints"]["collision"]["pairs"][pair][key] = v;
  return n;
}

TEST_F(KinodynamicOCPTest, EveryConstraintsKeyIsRequired) {
  for (const auto& [section, keys] : ConstraintSections()) {
    YAML::Node whole = YAML::Clone(tu::G1KinodynamicNode());
    whole["constraints"].remove(section);
    EXPECT_EQ(Load(whole), Err::kMissingKey) << section;
    for (const char* k : keys) {
      YAML::Node n = YAML::Clone(tu::G1KinodynamicNode());
      n["constraints"][section].remove(k);
      EXPECT_EQ(Load(n), Err::kMissingKey) << section << "." << k;
      EXPECT_EQ(Load(WithConstraint(section, k, YAML::Node(YAML::NodeType::Null))),
                Err::kMissingKey)
          << section << "." << k << ": null";
    }
  }
  const std::size_t n_pairs = tu::G1KinodynamicNode()["constraints"]["collision"]["pairs"].size();
  ASSERT_EQ(n_pairs, 16U);
  for (const std::size_t pair : {std::size_t{0}, n_pairs - 1}) {
    for (const char* k : PairKeys()) {
      YAML::Node n = YAML::Clone(tu::G1KinodynamicNode());
      n["constraints"]["collision"]["pairs"][pair].remove(k);
      EXPECT_EQ(Load(n), Err::kMissingKey) << "pair " << pair << "." << k;
    }
  }
}

TEST_F(KinodynamicOCPTest, AnUnknownConstraintsKeyIsRefusedAtEveryLevel) {
  YAML::Node top = YAML::Clone(tu::G1KinodynamicNode());
  top["constraints"]["typo"] = 1.0;
  EXPECT_EQ(Load(top), Err::kUnknownKey) << "constraints";
  for (const auto& section : ConstraintSections()) {
    EXPECT_EQ(Load(WithConstraint(section.first, "typo", YAML::Node(1.0))), Err::kUnknownKey)
        << section.first;
  }
  for (const std::size_t pair : {std::size_t{0}, std::size_t{7}, std::size_t{15}}) {
    EXPECT_EQ(Load(WithPairValue(pair, "typo", YAML::Node(1.0))), Err::kUnknownKey)
        << "pair " << pair;
  }
}

TEST_F(KinodynamicOCPTest, ANonFiniteConstraintsValueIsRefused) {
  for (const char* bad : {".nan", ".inf", "-.inf"}) {
    for (const auto& [section, keys] : ConstraintSections()) {
      for (const char* k : keys) {
        const std::string key(k);
        if (key == "enabled" || key == "pairs") {
          continue;
        }
        EXPECT_EQ(Load(WithConstraint(section, k, YAML::Load(bad))), Err::kNonFinite)
            << section << "." << k << " = " << bad;
      }
    }
    for (const char* k : {"radius_a", "radius_b"}) {
      EXPECT_EQ(Load(WithPairValue(3, k, YAML::Load(bad))), Err::kNonFinite) << k << " " << bad;
    }
    const std::string seq = std::string("[0.0, ") + bad + ", 0.0]";
    for (const char* k : {"offset_a", "offset_b"}) {
      EXPECT_EQ(Load(WithPairValue(3, k, YAML::Load(seq))), Err::kNonFinite) << k << " " << bad;
    }
  }
}

// The values marked > 0 refuse 0 (and negatives); the rest accept 0 and
// refuse negatives.
TEST_F(KinodynamicOCPTest, ConstraintsValuesHaveTheirSigns) {
  const std::vector<std::pair<const char*, const char*>> positive = {
      {"friction", "regularization"},
      {"friction", "mu"},
      {"joint_limits", "delta"},
      {"collision", "delta"},
      {"foot_constraint", "linear_acceleration_xy"},
      {"foot_constraint", "linear_acceleration_z"},
      {"foot_constraint", "angular_acceleration"},
  };
  for (const auto& [section, key] : positive) {
    EXPECT_EQ(Load(WithConstraint(section, key, YAML::Node(0.0))), Err::kNonPositive)
        << section << "." << key << " = 0";
    EXPECT_EQ(Load(WithConstraint(section, key, YAML::Node(-1e-9))), Err::kNonPositive)
        << section << "." << key << " < 0";
  }
  const std::vector<std::pair<const char*, const char*>> non_negative = {
      {"foot_constraint", "position_z"},
      {"foot_constraint", "orientation"},
      {"foot_constraint", "linear_velocity_xy"},
      {"foot_constraint", "linear_velocity_z"},
      {"foot_constraint", "angular_velocity"},
      {"friction", "grip_force"},
      {"joint_limits", "mu"},
      {"collision", "mu"},
  };
  for (const auto& [section, key] : non_negative) {
    EXPECT_EQ(Load(WithConstraint(section, key, YAML::Node(-1e-9))), Err::kNegativeWeight)
        << section << "." << key << " < 0";
    EXPECT_EQ(Load(WithConstraint(section, key, YAML::Node(0.0))), Err::kNoError)
        << section << "." << key << " = 0";
  }
  for (const char* k : {"radius_a", "radius_b"}) {
    EXPECT_EQ(Load(WithPairValue(5, k, YAML::Node(-0.01))), Err::kNegativeWeight) << k;
    EXPECT_EQ(Load(WithPairValue(5, k, YAML::Node(0.0))), Err::kNoError) << k;
  }
}

TEST_F(KinodynamicOCPTest, ARectangleNotContainingItsOriginIsRefused) {
  EXPECT_EQ(Load(WithConstraint("contact_rectangle", "x_min", YAML::Node(0.01))),
            Err::kInvalidRectangle);
  EXPECT_EQ(Load(WithConstraint("contact_rectangle", "y_max", YAML::Node(-0.01))),
            Err::kInvalidRectangle);
  // The origin on an edge is not inside.
  for (const char* k : {"x_min", "x_max", "y_min", "y_max"}) {
    EXPECT_EQ(Load(WithConstraint("contact_rectangle", k, YAML::Node(0.0))), Err::kInvalidRectangle)
        << k;
  }
}

TEST_F(KinodynamicOCPTest, AnUnknownCollisionFrameIsRefusedEvenWhenDisabled) {
  for (const bool enabled : {false, true}) {
    for (const char* k : {"frame_a", "frame_b"}) {
      YAML::Node n = WithPairValue(9, k, YAML::Node("no_such_frame"));
      n["constraints"]["collision"]["enabled"] = enabled;
      EXPECT_EQ(Load(n), Err::kUnknownFrame) << k << " enabled " << enabled;
    }
  }
  EXPECT_EQ(Load(WithPairValue(0, "frame_a", YAML::Load("[left_knee_joint]"))), Err::kWrongType);
  EXPECT_EQ(Load(WithPairValue(0, "offset_b", YAML::Load("[0.0, 0.0]"))), Err::kWrongType);
  EXPECT_EQ(Load(WithConstraint("collision", "enabled", YAML::Node("maybe"))), Err::kWrongType);
  EXPECT_EQ(Load(WithConstraint("collision", "pairs", YAML::Load("{a: 1}"))), Err::kWrongType);
}

// A model whose limited one-entry joint has lower ≥ upper.
TEST_F(KinodynamicOCPTest, AModelJointWithAnEmptyLimitRangeIsRefused) {
  const auto idx =
      static_cast<Eigen::Index>(model_.joints[model_.getJointId("left_knee_joint")].idx_q());
  for (const double gap : {0.0, -0.1}) {
    pinocchio::Model bad = model_;
    bad.upperPositionLimit[idx] = bad.lowerPositionLimit[idx] + gap;
    rtc::mpc::RobotModelHandler h;
    ASSERT_EQ(h.Init(bad, YAML::Load(tu::kG1SolesYaml)), rtc::mpc::RobotModelInitError::kNoError);
    KinodynamicOCPConfig out;
    EXPECT_EQ(LoadKinodynamicOCPConfig(tu::G1KinodynamicNode(), h, out), Err::kInvalidJointLimit)
        << "upper - lower = " << gap;
  }
}

// The loader reads its input without touching it: the tree dumps the same
// after a load, and a second load of it gives the same config.
TEST_F(KinodynamicOCPTest, TheLoaderLeavesItsInputUnchanged) {
  const YAML::Node n = YAML::Clone(tu::G1KinodynamicNode());
  const std::string before = YAML::Dump(n);
  KinodynamicOCPConfig a;
  KinodynamicOCPConfig b;
  ASSERT_EQ(LoadKinodynamicOCPConfig(n, handler_, a), Err::kNoError);
  EXPECT_EQ(YAML::Dump(n), before);
  ASSERT_EQ(LoadKinodynamicOCPConfig(n, handler_, b), Err::kNoError);
  EXPECT_EQ(a.constraints.foot_constraint.position_z, b.constraints.foot_constraint.position_z);
  EXPECT_EQ(a.constraints.friction.mu, b.constraints.friction.mu);
  EXPECT_EQ(a.constraints.collision_pairs.size(), b.constraints.collision_pairs.size());
}

TEST_F(KinodynamicOCPTest, LoadsTheConstraintsAsWritten) {
  const rtc::mpc::KinodynamicConstraintsConfig& c = cfg_.constraints;
  EXPECT_EQ(c.foot_constraint.position_z, 100.0);
  EXPECT_EQ(c.foot_constraint.orientation, 80.0);
  EXPECT_EQ(c.foot_constraint.linear_velocity_xy, 20.0);
  EXPECT_EQ(c.foot_constraint.linear_velocity_z, 10.0);
  EXPECT_EQ(c.foot_constraint.angular_velocity, 20.0);
  EXPECT_EQ(c.foot_constraint.linear_acceleration_xy, 1.0);
  EXPECT_EQ(c.foot_constraint.linear_acceleration_z, 1.0);
  EXPECT_EQ(c.foot_constraint.angular_acceleration, 1.0);
  EXPECT_EQ(c.friction.mu, 0.4);
  EXPECT_EQ(c.friction.grip_force, 0.0);
  EXPECT_EQ(c.friction.regularization, 25.0);
  EXPECT_EQ(c.contact_rectangle.x_min, -0.09);
  EXPECT_EQ(c.contact_rectangle.x_max, 0.09);
  EXPECT_EQ(c.contact_rectangle.y_min, -0.03);
  EXPECT_EQ(c.contact_rectangle.y_max, 0.03);
  EXPECT_EQ(c.joint_limits.mu, 1200.0);
  EXPECT_EQ(c.joint_limits.delta, 0.1);
  EXPECT_FALSE(c.collision_enabled);
  EXPECT_EQ(c.collision.mu, 1500.0);
  EXPECT_EQ(c.collision.delta, 0.04);
  const YAML::Node pairs = tu::G1KinodynamicNode()["constraints"]["collision"]["pairs"];
  ASSERT_EQ(c.collision_pairs.size(), 16U);
  ASSERT_EQ(pairs.size(), 16U);
  for (std::size_t i = 0; i < c.collision_pairs.size(); ++i) {
    const rtc::mpc::SpherePair& p = c.collision_pairs[i];
    const YAML::Node y = pairs[i];
    const std::string fa = y["frame_a"].as<std::string>();
    const std::string fb = y["frame_b"].as<std::string>();
    ASSERT_TRUE(model_.existFrame(fa)) << fa;
    ASSERT_TRUE(model_.existFrame(fb)) << fb;
    EXPECT_EQ(p.frame_a, model_.getFrameId(fa)) << "pair " << i;
    EXPECT_EQ(p.frame_b, model_.getFrameId(fb)) << "pair " << i;
    EXPECT_EQ(model_.frames[p.frame_a].name, fa) << "pair " << i;
    EXPECT_EQ(model_.frames[p.frame_b].name, fb) << "pair " << i;
    EXPECT_EQ(p.radius_a, y["radius_a"].as<double>()) << "pair " << i;
    EXPECT_EQ(p.radius_b, y["radius_b"].as<double>()) << "pair " << i;
    for (int j = 0; j < 3; ++j) {
      EXPECT_EQ(p.offset_a[j], y["offset_a"][j].as<double>()) << "pair " << i << " a" << j;
      EXPECT_EQ(p.offset_b[j], y["offset_b"][j].as<double>()) << "pair " << i << " b" << j;
    }
  }
  // Enabled loads the same pairs.
  YAML::Node on = YAML::Clone(tu::G1KinodynamicNode());
  on["constraints"]["collision"]["enabled"] = true;
  KinodynamicOCPConfig cfg_on;
  ASSERT_EQ(LoadKinodynamicOCPConfig(on, handler_, cfg_on), Err::kNoError);
  EXPECT_TRUE(cfg_on.constraints.collision_enabled);
  EXPECT_EQ(cfg_on.constraints.collision_pairs.size(), 16U);
}

// ── Where each weight lands ─────────────────────────────────────────────────

// Distinct sentinels, one per entry, land on exactly the tangent / input /
// residual row they name and nowhere else.
TEST_F(KinodynamicOCPTest, EveryWeightLandsOnItsOwnRow) {
  YAML::Node n = tu::G1KinodynamicNode();
  const auto set3 = [](YAML::Node node, double a) {
    node[0] = a;
    node[1] = a + 0.1;
    node[2] = a + 0.2;
  };
  set3(n["state_weights"]["base_position"], 1.0);
  set3(n["state_weights"]["base_orientation"], 2.0);
  set3(n["state_weights"]["base_linear_velocity"], 3.0);
  set3(n["state_weights"]["base_angular_velocity"], 4.0);
  set3(n["input_weights"]["contact_force"], 5.0);
  set3(n["input_weights"]["contact_moment"], 6.0);
  const char* blocks[] = {"position",         "orientation",         "linear_velocity",
                          "angular_velocity", "linear_acceleration", "angular_acceleration"};
  for (int b = 0; b < 6; ++b) {
    set3(n["contact_tracking"][blocks[b]], 10.0 * (b + 1));
  }
  for (pinocchio::JointIndex j = 2; j < static_cast<pinocchio::JointIndex>(model_.njoints); ++j) {
    const std::string& name = model_.names[j];
    n["state_weights"]["joints"][name] = 100.0 + static_cast<double>(j);
    n["state_weights"]["joint_velocities"][name] = 200.0 + static_cast<double>(j);
    n["input_weights"]["joint_accelerations"][name] = 300.0 + static_cast<double>(j);
  }
  KinodynamicOCPConfig cfg;
  ASSERT_EQ(LoadKinodynamicOCPConfig(n, handler_, cfg), Err::kNoError);

  const int nv = model_.nv;
  Eigen::VectorXd q_expect = Eigen::VectorXd::Zero(2 * nv);
  Eigen::VectorXd u_expect = Eigen::VectorXd::Zero(handler_.nu());
  for (int i = 0; i < 3; ++i) {
    q_expect[i] = 1.0 + 0.1 * i;
    q_expect[3 + i] = 2.0 + 0.1 * i;
    q_expect[nv + i] = 3.0 + 0.1 * i;
    q_expect[nv + 3 + i] = 4.0 + 0.1 * i;
    for (int c = 0; c < handler_.n_contacts(); ++c) {
      const int off = handler_.layout().contact_u_offset[static_cast<std::size_t>(c)];
      u_expect[off + i] = 5.0 + 0.1 * i;
      u_expect[off + 3 + i] = 6.0 + 0.1 * i;
    }
  }
  for (pinocchio::JointIndex j = 2; j < static_cast<pinocchio::JointIndex>(model_.njoints); ++j) {
    const int iv = model_.joints[j].idx_v();
    q_expect[iv] = 100.0 + static_cast<double>(j);
    q_expect[nv + iv] = 200.0 + static_cast<double>(j);
    u_expect[handler_.layout().acc_u_offset + iv - 6] = 300.0 + static_cast<double>(j);
  }
  EXPECT_TRUE(BitEqual(cfg.state_weights, q_expect));
  EXPECT_TRUE(BitEqual(cfg.input_weights, u_expect));

  // The residual's weight matrix in a built stage: diagonal, each contact's
  // six blocks in residual order.
  KinodynamicOCP ocp(cfg);
  ASSERT_EQ(ocp.Build(Context(), handler_, {}), OCPBuildError::kNoError);
  auto* stack = ocp.problem().stages_[0]->getCost<CostStack>();
  ASSERT_NE(stack, nullptr);
  const auto* track =
      stack->getComponent<QuadResidualCost>(std::string(KinodynamicOCP::kContactTrackingCostKey));
  ASSERT_NE(track, nullptr);
  const Eigen::MatrixXd& w = track->weights_;
  const int nr = kRows * handler_.n_contacts();
  ASSERT_EQ(w.rows(), nr);
  ASSERT_EQ(w.cols(), nr);
  for (int c = 0; c < handler_.n_contacts(); ++c) {
    for (int b = 0; b < 6; ++b) {
      for (int i = 0; i < 3; ++i) {
        const int row = kRows * c + 3 * b + i;
        EXPECT_EQ(w(row, row), 10.0 * (b + 1) + 0.1 * i) << "contact " << c << " block " << b;
      }
    }
  }
  EXPECT_EQ((w - Eigen::MatrixXd(w.diagonal().asDiagonal())).cwiseAbs().maxCoeff(), 0.0);
  // The state and input costs carry the loaded diagonals; the terminal the
  // scaled terminal weights.
  const auto* xc = stack->getComponent<KinodynamicOCP::QuadStateCost>(
      std::string(KinodynamicOCP::kStateCostKey));
  const auto* uc = stack->getComponent<KinodynamicOCP::QuadControlCost>(
      std::string(KinodynamicOCP::kInputCostKey));
  ASSERT_NE(xc, nullptr);
  ASSERT_NE(uc, nullptr);
  EXPECT_TRUE(BitEqual(xc->weights_.diagonal(), q_expect));
  EXPECT_TRUE(BitEqual(uc->weights_.diagonal(), u_expect));
  const auto* term = dynamic_cast<const KinodynamicOCP::QuadStateCost*>(&*ocp.problem().term_cost_);
  ASSERT_NE(term, nullptr);
  EXPECT_TRUE(BitEqual(term->weights_.diagonal(), Eigen::VectorXd(4.0 * cfg.terminal_weights)));
  EXPECT_EQ(stack->getWeight(std::string(KinodynamicOCP::kStateCostKey)), tu::kKinoDt);
}

// ── Build ───────────────────────────────────────────────────────────────────

TEST_F(KinodynamicOCPTest, BuildChecksTheCostConfigAndShapesTheProblem) {
  KinodynamicOCP ocp(cfg_);
  PhaseContext ctx = Context();
  ctx.cost_config.horizon_length = 31;
  EXPECT_EQ(ocp.Build(ctx, handler_, {}), OCPBuildError::kInvalidCostConfig);
  ctx = Context();
  ctx.cost_config.dt = 0.034;
  EXPECT_EQ(ocp.Build(ctx, handler_, {}), OCPBuildError::kInvalidCostConfig);
  ctx = Context();
  ctx.ocp_type = "contact_light";
  EXPECT_EQ(ocp.Build(ctx, handler_, {}), OCPBuildError::kInvalidPhaseContext);
  EXPECT_FALSE(ocp.Built());

  ASSERT_EQ(ocp.Build(Context(), handler_, {}), OCPBuildError::kNoError);
  ASSERT_TRUE(ocp.Built());
  auto& p = ocp.problem();
  ASSERT_EQ(p.stages_.size(), 32U);
  for (auto& stage : p.stages_) {
    EXPECT_EQ(stage->ndx1(), 58);
    EXPECT_EQ(stage->nu(), 35);
    // The contact equality and the wrench cone, every contact's rows of both
    // phases whatever the mode.
    EXPECT_EQ(stage->numConstraints(), 2U);
    EXPECT_EQ(stage->nc(), 2 * (rtc::mpc::ContactConstraintResidual::EqualityRows(6) +
                                rtc::mpc::ContactWrenchConeResidual::InequalityRows(6)));
    // Full stance until a horizon arrives.
    const auto* ode = KinodynamicDynamics::OdeOf(*stage);
    ASSERT_NE(ode, nullptr);
    EXPECT_EQ(ode->contact_states_, (std::vector<bool>{true, true}));
  }
  // A config of another model is refused.
  KinodynamicOCPConfig small = cfg_;
  small.input_weights.conservativeResize(34);
  KinodynamicOCP other(small);
  EXPECT_EQ(other.Build(Context(), handler_, {}), OCPBuildError::kInvalidCostConfig);
}

// ── Per-tick binding on a walk window ───────────────────────────────────────

/// The stage-reference builder on the G1 walking from the stand pose: the
/// gait fixture's walk (contacts = the soles, bit 0 left, bit 1 right).
struct WalkWindow {
  rtc::mpc::GaitScheduler scheduler;
  rtc::mpc::StageReferenceBuilder builder;

  WalkWindow(const pinocchio::Model& model, const rtc::mpc::RobotModelHandler& handler,
             const Eigen::VectorXd& stand, double t0) {
    const rtc::mpc::GaitTable table = tu::LocoGaitTable();
    EXPECT_EQ(scheduler.Init(table, tu::kKinoNodes * tu::kKinoDt),
              rtc::mpc::GaitSchedulerError::kNoError);
    const std::vector<double> q_joints(stand.data() + 7, stand.data() + model.nq);
    rtc::mpc::ReferenceConfig ref;
    EXPECT_EQ(rtc::mpc::LoadReferenceConfig(
                  tu::ReferenceYaml(model, q_joints, false, {{"left_elbow_joint", 1.0}}),
                  tu::ActuatedJointNames(model), ref),
              rtc::mpc::ReferenceError::kNoError);
    EXPECT_EQ(builder.Init(handler, ref, tu::ShippedSwing(), rtc::mpc::FlatSurface(0.0),
                           tu::kKinoNodes + 1, tu::kKinoDt),
              rtc::mpc::ReferenceError::kNoError);
    tu::WalkFrom(scheduler, t0);
    rtc::mpc::FilteredCommand cmd = tu::RegisteredCommand();
    cmd.base_height = 0.75;
    EXPECT_EQ(builder.Build(scheduler, tu::SnapshotAt(model, stand), cmd),
              rtc::mpc::ReferenceError::kNoError);
  }

  [[nodiscard]] const HorizonReference& horizon() const { return builder.active(); }
};

/// What UpdateReferences wrote, read back out of the problem.
struct Written {
  std::vector<std::vector<bool>> modes;
  std::vector<std::vector<double>> scales;
  std::vector<Eigen::VectorXd> x_ref;
  std::vector<Eigen::VectorXd> u_ref;
  Eigen::VectorXd x_term;

  bool operator==(const Written& o) const {
    if (modes != o.modes || scales.size() != o.scales.size() || x_ref.size() != o.x_ref.size() ||
        !BitEqual(x_term, o.x_term)) {
      return false;
    }
    for (std::size_t k = 0; k < x_ref.size(); ++k) {
      if (!BitEqual(x_ref[k], o.x_ref[k]) || !BitEqual(u_ref[k], o.u_ref[k]) ||
          std::memcmp(scales[k].data(), o.scales[k].data(), sizeof(double) * scales[k].size()) !=
              0) {
        return false;
      }
    }
    return true;
  }
};

Written ReadBack(KinodynamicOCP& ocp) {
  Written w;
  auto& p = ocp.problem();
  for (auto& stage : p.stages_) {
    w.modes.push_back(KinodynamicDynamics::OdeOf(*stage)->contact_states_);
    auto* stack = stage->getCost<CostStack>();
    auto* track =
        stack->getComponent<QuadResidualCost>(std::string(KinodynamicOCP::kContactTrackingCostKey))
            ->getResidual<ContactTrackingResidual>();
    std::vector<double> s;
    for (int c = 0; c < track->n_contacts(); ++c) {
      s.push_back(track->scale(c));
    }
    w.scales.push_back(s);
    w.x_ref.push_back(stack
                          ->getComponent<KinodynamicOCP::QuadStateCost>(
                              std::string(KinodynamicOCP::kStateCostKey))
                          ->getTarget());
    w.u_ref.push_back(stack
                          ->getComponent<KinodynamicOCP::QuadControlCost>(
                              std::string(KinodynamicOCP::kInputCostKey))
                          ->getTarget());
  }
  w.x_term = dynamic_cast<KinodynamicOCP::QuadStateCost&>(*p.term_cost_).getTarget();
  return w;
}

TEST_F(KinodynamicOCPTest, UpdateReferencesBindsModesAndTargetsWithoutRebuilding) {
  const WalkWindow walk(model_, handler_, stand_, 20.0);
  const HorizonReference& h = walk.horizon();
  ASSERT_EQ(h.n_stages, 33U);
  // The window is a walk: both single supports and double support appear.
  bool seen[4] = {false, false, false, false};
  for (int k = 0; k < tu::kKinoNodes; ++k) {
    seen[h.stages[static_cast<std::size_t>(k)].mode & 3U] = true;
  }
  ASSERT_TRUE(seen[1] && seen[2] && seen[3]) << "the window has no walk in it";

  KinodynamicOCP ocp(cfg_);
  ASSERT_EQ(ocp.Build(Context(), handler_, {}), OCPBuildError::kNoError);
  const auto* problem_before = &ocp.problem();
  const auto* stage0_before = &*ocp.problem().stages_[0];
  ASSERT_EQ(ocp.UpdateReferences(Context(&h)), OCPBuildError::kNoError);
  EXPECT_TRUE(ocp.Built());
  EXPECT_EQ(&ocp.problem(), problem_before);
  EXPECT_EQ(&*ocp.problem().stages_[0], stage0_before);

  const Written w = ReadBack(ocp);
  const int nq = model_.nq;
  const int nv = model_.nv;
  const int ncv = handler_.layout().n_contact_vars;
  for (int k = 0; k < tu::kKinoNodes; ++k) {
    const auto& s = h.stages[static_cast<std::size_t>(k)];
    for (int c = 0; c < 2; ++c) {
      const bool bit = ((s.mode >> c) & 1U) != 0U;
      EXPECT_EQ(w.modes[k][c], bit) << "stage " << k;
      const double expect = bit ? 0.0 : s.impact_proximity[static_cast<std::size_t>(c)];
      EXPECT_EQ(std::memcmp(&w.scales[k][c], &expect, sizeof(double)), 0) << "stage " << k;
    }
    Eigen::VectorXd x(nq + nv);
    x << Eigen::Map<const Eigen::VectorXd>(s.q.data(), nq),
        Eigen::Map<const Eigen::VectorXd>(s.v.data(), nv);
    EXPECT_TRUE(BitEqual(w.x_ref[k], x)) << "stage " << k;
    Eigen::VectorXd u = Eigen::VectorXd::Zero(handler_.nu());
    u.head(ncv) = Eigen::Map<const Eigen::VectorXd>(s.wrench.data(), ncv);
    EXPECT_TRUE(BitEqual(w.u_ref[k], u)) << "stage " << k;
  }
  const auto& last = h.stages[tu::kKinoNodes];
  Eigen::VectorXd x_term(nq + nv);
  x_term << Eigen::Map<const Eigen::VectorXd>(last.q.data(), nq),
      Eigen::Map<const Eigen::VectorXd>(last.v.data(), nv);
  EXPECT_TRUE(BitEqual(w.x_term, x_term));
}

TEST_F(KinodynamicOCPTest, AHorizonOfAnotherShapeOrWithABadValueChangesNothing) {
  const WalkWindow walk(model_, handler_, stand_, 20.0);
  KinodynamicOCP ocp(cfg_);
  ASSERT_EQ(ocp.Build(Context(), handler_, {}), OCPBuildError::kNoError);
  ASSERT_EQ(ocp.UpdateReferences(Context(&walk.horizon())), OCPBuildError::kNoError);
  const Written before = ReadBack(ocp);

  EXPECT_EQ(ocp.UpdateReferences(Context(nullptr)), OCPBuildError::kMissingHorizonReference);

  const auto bad = [&](auto&& edit) {
    auto h = std::make_unique<HorizonReference>(walk.horizon());
    edit(*h);
    return ocp.UpdateReferences(Context(h.get()));
  };
  EXPECT_EQ(bad([](HorizonReference& h) { h.n_stages = 32; }), OCPBuildError::kInvalidPhaseContext);
  EXPECT_EQ(bad([](HorizonReference& h) { h.n_contacts = 1; }),
            OCPBuildError::kInvalidPhaseContext);
  EXPECT_EQ(bad([](HorizonReference& h) { h.nq = 29; }), OCPBuildError::kInvalidPhaseContext);
  EXPECT_EQ(bad([](HorizonReference& h) { h.nv = 30; }), OCPBuildError::kInvalidPhaseContext);
  EXPECT_EQ(bad([](HorizonReference& h) { h.n_contact_vars = 6; }),
            OCPBuildError::kInvalidPhaseContext);
  EXPECT_EQ(bad([](HorizonReference& h) { h.dt = 0.0351; }), OCPBuildError::kInvalidPhaseContext);
  // A cost config of another horizon is a topology change too.
  PhaseContext ctx = Context(&walk.horizon());
  ctx.cost_config.dt = 0.03;
  EXPECT_EQ(ocp.UpdateReferences(ctx), OCPBuildError::kInvalidPhaseContext);

  // Bad values — at the last node, so a write-as-you-go would have changed
  // every node before it.
  const auto last = static_cast<std::size_t>(tu::kKinoNodes - 1);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(bad([&](HorizonReference& h) { h.stages[last].q[10] = nan; }),
            OCPBuildError::kInvalidHorizonReference);
  EXPECT_EQ(bad([&](HorizonReference& h) { h.stages[last + 1].v[3] = nan; }),
            OCPBuildError::kInvalidHorizonReference);
  EXPECT_EQ(bad([&](HorizonReference& h) { h.stages[last].q[6] = 0.9; }),
            OCPBuildError::kInvalidHorizonReference);
  EXPECT_EQ(bad([&](HorizonReference& h) { h.stages[last].mode = 4; }),
            OCPBuildError::kInvalidHorizonReference);
  EXPECT_EQ(bad([&](HorizonReference& h) { h.stages[last].wrench[2] = nan; }),
            OCPBuildError::kInvalidHorizonReference);
  EXPECT_EQ(bad([&](HorizonReference& h) {
              h.stages[last].mode = 0b01;
              h.stages[last].impact_proximity[1] = 1.5;
            }),
            OCPBuildError::kInvalidHorizonReference);
  EXPECT_EQ(bad([&](HorizonReference& h) { h.stages[last].contact_twist[4] = nan; }),
            OCPBuildError::kInvalidHorizonReference);
  EXPECT_EQ(bad([&](HorizonReference& h) { h.terrain.normal = {0.0, 0.0, 1.1}; }),
            OCPBuildError::kInvalidHorizonReference);
  // A stance contact's impact proximity is not read (its scale is 0).
  EXPECT_EQ(bad([&](HorizonReference& h) {
              h.stages[last].mode = 0b11;
              h.stages[last].impact_proximity[1] = 1.5;
            }),
            OCPBuildError::kNoError);
  ASSERT_EQ(ocp.UpdateReferences(Context(&walk.horizon())), OCPBuildError::kNoError);

  EXPECT_TRUE(ReadBack(ocp) == before);
}

// A refused Build changes nothing: the handler core rebuilds on
// kInvalidPhaseContext, and that rebuild must not take a working problem down.
TEST_F(KinodynamicOCPTest, BuildWithAHorizonAppliesItAndARefusedBuildChangesNothing) {
  const WalkWindow walk(model_, handler_, stand_, 20.0);
  KinodynamicOCP built_then_updated(cfg_);
  ASSERT_EQ(built_then_updated.Build(Context(), handler_, {}), OCPBuildError::kNoError);
  ASSERT_EQ(built_then_updated.UpdateReferences(Context(&walk.horizon())), OCPBuildError::kNoError);
  KinodynamicOCP built_with(cfg_);
  ASSERT_EQ(built_with.Build(Context(&walk.horizon()), handler_, {}), OCPBuildError::kNoError);
  EXPECT_TRUE(ReadBack(built_with) == ReadBack(built_then_updated));

  const Written before = ReadBack(built_with);
  const auto* problem = &built_with.problem();
  const auto* stage0 = &*built_with.problem().stages_[0];
  auto h = std::make_unique<HorizonReference>(walk.horizon());
  h->n_stages = 20;
  EXPECT_EQ(built_with.Build(Context(h.get()), handler_, {}), OCPBuildError::kInvalidPhaseContext);
  h = std::make_unique<HorizonReference>(walk.horizon());
  h->stages[3].q[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(built_with.Build(Context(h.get()), handler_, {}),
            OCPBuildError::kInvalidHorizonReference);
  PhaseContext other = Context(&walk.horizon());
  other.cost_config.horizon_length = 20;
  EXPECT_EQ(built_with.Build(other, handler_, {}), OCPBuildError::kInvalidCostConfig);
  // No input box in this formulation, so a control box is refused rather
  // than dropped.
  rtc::mpc::OCPLimits box;
  box.u_min = Eigen::VectorXd::Constant(handler_.nu(), -1.0);
  box.u_max = Eigen::VectorXd::Constant(handler_.nu(), 1.0);
  EXPECT_EQ(built_with.Build(Context(&walk.horizon()), handler_, box),
            OCPBuildError::kLimitsDimMismatch);
  ASSERT_TRUE(built_with.Built());
  EXPECT_EQ(&built_with.problem(), problem);
  EXPECT_EQ(&*built_with.problem().stages_[0], stage0);
  EXPECT_TRUE(ReadBack(built_with) == before);
}

// ── Constraint and soft-limit layout ────────────────────────────────────────

/// A full-stance horizon at @p q whose stage k is in mode (k + shift) mod 2^nc.
std::unique_ptr<HorizonReference> EveryModeHorizon(const rtc::mpc::RobotModelHandler& handler,
                                                   const Eigen::VectorXd& q, std::uint32_t shift) {
  auto h = tu::StanceHorizon(handler, q);
  const std::uint32_t n_modes = 1U << static_cast<unsigned>(handler.n_contacts());
  for (int k = 0; k < tu::kKinoNodes; ++k) {
    h->stages[static_cast<std::size_t>(k)].mode = (static_cast<std::uint32_t>(k) + shift) % n_modes;
  }
  return h;
}

// Every stage, in every mode: both constraints with the model's row count,
// bound to the stage's own ODE; the joint-limit barrier (and the collision
// barrier when enabled) in the cost stack at weight dt; nothing terminal.
TEST_F(KinodynamicOCPTest, EveryStageCarriesTheFixedConstraintRowsAndTheSoftLimits) {
  int expect_nc = 0;
  for (const auto& f : handler_.contact_frames()) {
    expect_nc += ContactConstraintResidual::EqualityRows(f.dim) +
                 ContactWrenchConeResidual::InequalityRows(f.dim);
  }
  EXPECT_EQ(expect_nc, 24);
  const int n_limited = JointLimitBarrierCost::CountLimitedJoints(model_);
  EXPECT_EQ(n_limited, 23);

  for (const bool collision : {false, true}) {
    YAML::Node node = YAML::Clone(tu::G1KinodynamicNode());
    node["constraints"]["collision"]["enabled"] = collision;
    KinodynamicOCPConfig cfg;
    ASSERT_EQ(LoadKinodynamicOCPConfig(node, handler_, cfg), Err::kNoError);
    KinodynamicOCP ocp(cfg);
    ASSERT_EQ(ocp.Build(Context(), handler_, {}), OCPBuildError::kNoError);
    auto& p = ocp.problem();
    EXPECT_EQ(p.term_cstrs_.size(), 0U) << "a terminal constraint";
    const std::uint32_t n_modes = 1U << static_cast<unsigned>(handler_.n_contacts());
    for (std::uint32_t shift = 0; shift < n_modes; ++shift) {
      const auto h = EveryModeHorizon(handler_, stand_, shift);
      ASSERT_EQ(ocp.UpdateReferences(Context(h.get())), OCPBuildError::kNoError);
      for (std::size_t k = 0; k < p.stages_.size(); ++k) {
        const std::string what = "collision " + std::to_string(collision) + " shift " +
                                 std::to_string(shift) + " stage " + std::to_string(k);
        auto& stage = *p.stages_[k];
        const auto* ode = KinodynamicDynamics::OdeOf(stage);
        ASSERT_NE(ode, nullptr) << what;
        const std::uint32_t mode = h->stages[k].mode;
        for (int c = 0; c < handler_.n_contacts(); ++c) {
          EXPECT_EQ(ode->contact_states_[static_cast<std::size_t>(c)], ((mode >> c) & 1U) != 0U)
              << what;
        }
        EXPECT_EQ(stage.numConstraints(), 2U) << what;
        EXPECT_EQ(stage.nc(), expect_nc) << what;
        const auto* eq = stage.constraints_.getConstraint<ContactConstraintResidual>(
            KinodynamicOCP::kEqualityConstraint);
        const auto* in = stage.constraints_.getConstraint<ContactWrenchConeResidual>(
            KinodynamicOCP::kInequalityConstraint);
        ASSERT_NE(eq, nullptr) << what;
        ASSERT_NE(in, nullptr) << what;
        EXPECT_EQ(eq->bound_ode(), ode) << what;
        EXPECT_EQ(in->bound_ode(), ode) << what;

        auto* stack = stage.getCost<CostStack>();
        ASSERT_NE(stack, nullptr) << what;
        const std::string jl_key(KinodynamicOCP::kJointLimitCostKey);
        const std::string col_key(KinodynamicOCP::kCollisionCostKey);
        const auto* jl = stack->getComponent<JointLimitBarrierCost>(jl_key);
        ASSERT_NE(jl, nullptr) << what;
        EXPECT_EQ(jl->n_joints(), n_limited) << what;
        EXPECT_EQ(stack->components_.at(jl_key).second, tu::kKinoDt) << what;
        const bool has_collision = stack->components_.find(col_key) != stack->components_.end();
        EXPECT_EQ(has_collision, collision) << what;
        EXPECT_EQ(stack->size(), collision ? 5U : 4U) << what;
        if (has_collision) {
          const auto* col = stack->getComponent<CollisionBarrierCost>(col_key);
          ASSERT_NE(col, nullptr) << what;
          EXPECT_EQ(col->n_pairs(), 16) << what;
          EXPECT_EQ(col->bound_ode(), ode) << what;
          EXPECT_EQ(stack->components_.at(col_key).second, tu::kKinoDt) << what;
        }
      }
    }
  }
}

/// Every stage's contact equality targets, read back out of the problem.
std::vector<std::vector<rtc::mpc::ContactHeightTarget>> ReadTargets(KinodynamicOCP& ocp) {
  std::vector<std::vector<rtc::mpc::ContactHeightTarget>> out;
  for (auto& stage : ocp.problem().stages_) {
    const auto* eq = stage->constraints_.getConstraint<ContactConstraintResidual>(
        KinodynamicOCP::kEqualityConstraint);
    std::vector<rtc::mpc::ContactHeightTarget> t;
    for (int c = 0; c < eq->n_contacts(); ++c) {
      t.push_back(eq->target(c));
    }
    out.push_back(t);
  }
  return out;
}

bool SameTargets(const std::vector<std::vector<rtc::mpc::ContactHeightTarget>>& a,
                 const std::vector<std::vector<rtc::mpc::ContactHeightTarget>>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t k = 0; k < a.size(); ++k) {
    if (a[k].size() != b[k].size()) {
      return false;
    }
    for (std::size_t c = 0; c < a[k].size(); ++c) {
      if (std::memcmp(&a[k][c].z, &b[k][c].z, sizeof(double)) != 0 ||
          std::memcmp(&a[k][c].vz, &b[k][c].vz, sizeof(double)) != 0 ||
          std::memcmp(&a[k][c].az, &b[k][c].az, sizeof(double)) != 0) {
        return false;
      }
    }
  }
  return true;
}

// A non-finite trajectory has no residual report: max and min would skip its
// NaNs and report it as clean.
TEST_F(KinodynamicOCPTest, ConstraintResidualsRefuseANonFiniteTrajectory) {
  const auto h = tu::StanceHorizon(handler_, stand_);
  KinodynamicOCP ocp(cfg_);
  ASSERT_EQ(ocp.Build(Context(h.get()), handler_, {}), OCPBuildError::kNoError);
  Eigen::VectorXd x0(model_.nq + model_.nv);
  x0 << stand_, Eigen::VectorXd::Zero(model_.nv);
  std::vector<Eigen::VectorXd> xs(static_cast<std::size_t>(tu::kKinoNodes + 1), x0);
  std::vector<Eigen::VectorXd> us(static_cast<std::size_t>(tu::kKinoNodes),
                                  Eigen::VectorXd::Zero(handler_.nu()));
  for (int k = 0; k < tu::kKinoNodes; ++k) {
    ASSERT_TRUE(ocp.ReferenceControl(k, us[static_cast<std::size_t>(k)]));
  }
  rtc::mpc::ConstraintResidualReport r;
  ASSERT_TRUE(ocp.ConstraintResiduals(xs, us, r));
  r.stance_equality_max = -1.0;
  us[7][2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ocp.ConstraintResiduals(xs, us, r));
  EXPECT_EQ(r.stance_equality_max, -1.0);  // written nothing
  us[7][2] = 0.0;
  xs[3][model_.nq + 2] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ocp.ConstraintResiduals(xs, us, r));
  xs[3][model_.nq + 2] = 0.0;
  xs.back()[0] = std::numeric_limits<double>::quiet_NaN();  // the terminal node too
  EXPECT_FALSE(ocp.ConstraintResiduals(xs, us, r));
  xs.back() = Eigen::VectorXd::Zero(3);
  EXPECT_FALSE(ocp.ConstraintResiduals(xs, us, r));
  xs.pop_back();
  EXPECT_FALSE(ocp.ConstraintResiduals(xs, us, r));
}

TEST_F(KinodynamicOCPTest, UpdateReferencesWritesTheContactEqualityTargets) {
  auto h = tu::StanceHorizon(handler_, stand_);
  tu::SwingContact(handler_, *h, 1, 8, 19, 0.05, 0.5);
  SwingContactCos(handler_, *h, 0, 22, 29, 0.04, 1.0);
  KinodynamicOCP ocp(cfg_);
  ASSERT_EQ(ocp.Build(Context(), handler_, {}), OCPBuildError::kNoError);
  ASSERT_EQ(ocp.UpdateReferences(Context(h.get())), OCPBuildError::kNoError);
  const auto written = ReadTargets(ocp);
  ASSERT_EQ(written.size(), static_cast<std::size_t>(tu::kKinoNodes));
  for (int k = 0; k < tu::kKinoNodes; ++k) {
    const auto& s = h->stages[static_cast<std::size_t>(k)];
    for (int c = 0; c < handler_.n_contacts(); ++c) {
      const auto i = static_cast<std::size_t>(c);
      const rtc::mpc::ContactHeightTarget& t = written[static_cast<std::size_t>(k)][i];
      EXPECT_EQ(std::memcmp(&t.z, &s.contact_pose[7 * i + 2], sizeof(double)), 0)
          << "stage " << k << " contact " << c;
      EXPECT_EQ(std::memcmp(&t.vz, &s.contact_twist[6 * i + 2], sizeof(double)), 0)
          << "stage " << k << " contact " << c;
      EXPECT_EQ(std::memcmp(&t.az, &s.contact_acceleration[6 * i + 2], sizeof(double)), 0)
          << "stage " << k << " contact " << c;
    }
    const auto* eq = ocp.problem()
                         .stages_[static_cast<std::size_t>(k)]
                         ->constraints_.getConstraint<ContactConstraintResidual>(
                             KinodynamicOCP::kEqualityConstraint);
    EXPECT_EQ(eq->plane_normal(),
              Eigen::Vector3d(h->terrain.normal[0], h->terrain.normal[1], h->terrain.normal[2]))
        << "stage " << k;
  }
  // The swing windows are in the targets (not all one stance height).
  EXPECT_GT(written[13][1].z, written[0][1].z + 0.04);
  EXPECT_GT(written[25][0].z, written[0][0].z + 0.02);
  EXPECT_NE(written[10][1].vz, 0.0);
  EXPECT_NE(written[10][1].az, 0.0);

  // A refused horizon — other targets everywhere, one bad value at the last
  // stage — leaves every target as it was.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto refused = [&](auto&& edit) {
    auto bad = tu::StanceHorizon(handler_, stand_);
    tu::SwingContact(handler_, *bad, 0, 2, 12, 0.08, 1.0);
    for (auto& s : bad->stages) {
      s.contact_pose[2] += 0.01;
      s.contact_pose[9] -= 0.01;
    }
    edit(*bad);
    return ocp.UpdateReferences(Context(bad.get()));
  };
  const auto last = static_cast<std::size_t>(tu::kKinoNodes - 1);
  EXPECT_EQ(refused([&](HorizonReference& b) { b.stages[last].contact_acceleration[8] = nan; }),
            OCPBuildError::kInvalidHorizonReference);
  EXPECT_EQ(refused([&](HorizonReference& b) { b.stages[last].contact_pose[9] = nan; }),
            OCPBuildError::kInvalidHorizonReference);
  EXPECT_EQ(refused([&](HorizonReference& b) { b.stages[last].contact_twist[2] = nan; }),
            OCPBuildError::kInvalidHorizonReference);
  EXPECT_EQ(refused([&](HorizonReference& b) { b.terrain.normal = {0.0, 0.0, 0.9}; }),
            OCPBuildError::kInvalidHorizonReference);
  EXPECT_TRUE(SameTargets(ReadTargets(ocp), written));
  // The same edits without the bad value are written.
  EXPECT_EQ(refused([](HorizonReference&) {}), OCPBuildError::kNoError);
  EXPECT_FALSE(SameTargets(ReadTargets(ocp), written));
}

// ── Cost values ─────────────────────────────────────────────────────────────

// Stage cost == dt · (½‖x ⊖ x_ref‖²_Q + ½‖u − u_ref‖²_R + ½ Σ w r² + J(q)),
// the difference from pinocchio, the residual from a separate instance on a
// separate ODE of the stage's mode, and J the joint-limit barrier written out
// here from the model's limits; terminal == ½‖x ⊖ x_ref‖² over 4·Q_f.
TEST_F(KinodynamicOCPTest, StageAndTerminalCostsAreTheIndependentSums) {
  const WalkWindow walk(model_, handler_, stand_, 20.0);
  const HorizonReference& h = walk.horizon();
  KinodynamicOCP ocp(cfg_);
  ASSERT_EQ(ocp.Build(Context(&h), handler_, {}), OCPBuildError::kNoError);
  KinodynamicDynamics dyn;
  ASSERT_EQ(dyn.Init(handler_), rtc::mpc::KinodynamicError::kNoError);
  const tu::KinoOracle oracle(model_, tu::G1SoleFrames(model_), 6);
  const std::vector<tu::KinoSample> samples = tu::G1GateSamples(oracle, 20261114U);
  const int nq = model_.nq;
  const int nv = model_.nv;
  const int ncv = handler_.layout().n_contact_vars;
  const Eigen::VectorXd w_track = cfg_.contact_tracking_weights.replicate(2, 1);
  // The piecewise-polynomial barrier on both sides of every limited
  // one-entry joint (the fixture's mu 1200, delta 0.1).
  const double bmu = cfg_.constraints.joint_limits.mu;
  const double bdelta = cfg_.constraints.joint_limits.delta;
  const auto barrier = [&](double m) {
    if (m >= bdelta) {
      return 0.0;
    }
    const double quad = 0.5 * m * m - 0.5 * bdelta * m + bdelta * bdelta / 6.0;
    return m <= 0.0 ? bmu * quad : bmu * (quad - m * m * m / (6.0 * bdelta));
  };
  const auto joint_barrier = [&](const Eigen::VectorXd& q) {
    double sum = 0.0;
    for (int j = 1; j < model_.njoints; ++j) {
      const auto& joint = model_.joints[static_cast<std::size_t>(j)];
      const double lo = model_.lowerPositionLimit[joint.idx_q()];
      const double hi = model_.upperPositionLimit[joint.idx_q()];
      if (joint.nq() != 1 || joint.nv() != 1 || !std::isfinite(lo) || !std::isfinite(hi)) {
        continue;
      }
      sum += barrier(q[joint.idx_q()] - lo) + barrier(hi - q[joint.idx_q()]);
    }
    return sum;
  };
  double worst = 0.0;
  double worst_term = 0.0;
  auto& p = ocp.problem();
  for (int i = 0; i < 50; ++i) {
    const auto& smp = samples[static_cast<std::size_t>(i)];
    const int k = (7 * i) % tu::kKinoNodes;
    const auto& s = h.stages[static_cast<std::size_t>(k)];
    const Eigen::Map<const Eigen::VectorXd> q_ref(s.q.data(), nq);
    const Eigen::Map<const Eigen::VectorXd> v_ref(s.v.data(), nv);
    Eigen::VectorXd dx(2 * nv);
    dx.head(nv) = pinocchio::difference(model_, q_ref, smp.x.head(nq));
    dx.tail(nv) = smp.x.tail(nv) - v_ref;
    Eigen::VectorXd du = smp.u;
    du.head(ncv) -= Eigen::Map<const Eigen::VectorXd>(s.wrench.data(), ncv);
    const KinodynamicDynamics::Ode ode = dyn.MakeOde(s.mode).value();
    ContactTrackingResidual r(ode);
    ASSERT_TRUE(r.Bind(&ode));
    ASSERT_TRUE(r.SetFromStage(s, h.terrain));
    auto rd = r.createData();
    r.evaluate(smp.x, smp.u, *rd);
    const double expect = tu::kKinoDt * (0.5 * dx.dot(cfg_.state_weights.cwiseProduct(dx)) +
                                         0.5 * du.dot(cfg_.input_weights.cwiseProduct(du)) +
                                         0.5 * rd->value_.dot(w_track.cwiseProduct(rd->value_)) +
                                         joint_barrier(smp.x.head(nq)));
    auto& stage = *p.stages_[static_cast<std::size_t>(k)];
    auto cd = stage.cost_->createData();
    stage.cost_->evaluate(smp.x, smp.u, *cd);
    ASSERT_TRUE(std::isfinite(cd->value_));
    worst = std::max(worst, std::abs(cd->value_ - expect) / std::max(1.0, std::abs(expect)));

    const auto& t = h.stages[tu::kKinoNodes];
    Eigen::VectorXd dxt(2 * nv);
    dxt.head(nv) = pinocchio::difference(model_, Eigen::Map<const Eigen::VectorXd>(t.q.data(), nq),
                                         smp.x.head(nq));
    dxt.tail(nv) = smp.x.tail(nv) - Eigen::Map<const Eigen::VectorXd>(t.v.data(), nv);
    const double expect_t = 4.0 * 0.5 * dxt.dot(cfg_.terminal_weights.cwiseProduct(dxt));
    auto td = p.term_cost_->createData();
    p.term_cost_->evaluate(smp.x, smp.u, *td);
    worst_term =
        std::max(worst_term, std::abs(td->value_ - expect_t) / std::max(1.0, std::abs(expect_t)));
  }
  std::printf("[report] stage cost worst rel %.3g, terminal %.3g (50 samples)\n", worst,
              worst_term);
  EXPECT_LE(worst, 1e-10);
  EXPECT_LE(worst_term, 1e-10);
}

// ── Reports (no gate) ───────────────────────────────────────────────────────

// Report only: heap of Build + solver setup, one iteration, one residual call.
TEST_F(KinodynamicOCPTest, ReportsHeapAndTimes) {
  const auto stance = tu::StanceHorizon(handler_, stand_);
  for (const char* integrator : {"semi_implicit_euler", "rk2"}) {
    rtc::mpc::KinodynamicOCPConfig cfg;
    ASSERT_EQ(LoadKinodynamicOCPConfig(tu::G1KinodynamicNode(integrator), handler_, cfg),
              Err::kNoError);
    rtc::mpc::MPCSolverConfig solver;
    solver.prim_tol = 1e-4;
    solver.dual_tol = 1e-4;
    solver.max_iters = 1;
    const std::size_t before = mallinfo2().uordblks;
    auto mpc = std::make_unique<rtc::mpc::KinodynamicMPC>(cfg);
    ASSERT_EQ(mpc->Init(solver, handler_, {}, Context(stance.get())),
              rtc::mpc::MPCInitError::kNoError);
    const std::size_t after = mallinfo2().uordblks;
    auto out = std::make_unique<rtc::mpc::MPCSolution>();
    const auto t0 = std::chrono::steady_clock::now();
    ASSERT_EQ(mpc->Solve(Context(stance.get()), tu::SnapshotAt(model_, stand_), *out),
              rtc::mpc::MPCSolveError::kNoError);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf(
        "[report] %s: problem + solver heap %.2f MB (%.2f MB per stage), "
        "1-iteration solve %.2f ms\n",
        integrator, static_cast<double>(after - before) / 1e6,
        static_cast<double>(after - before) / 1e6 / tu::kKinoNodes, ms);
  }

  rtc::mpc::KinodynamicDynamics dyn;
  ASSERT_EQ(dyn.Init(handler_), rtc::mpc::KinodynamicError::kNoError);
  const auto ode = dyn.MakeOde(0b00).value();
  rtc::mpc::ContactTrackingResidual r(ode);
  ASSERT_TRUE(r.Bind(&ode));
  ASSERT_TRUE(r.SetScale(0, 1.0));
  ASSERT_TRUE(r.SetScale(1, 1.0));
  auto data = r.createData();
  const tu::KinoOracle oracle(model_, tu::G1SoleFrames(model_), 6);
  const auto samples = tu::G1GateSamples(oracle, 20261114U);
  constexpr int kReps = 200;
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < kReps; ++i) {
    const auto& s = samples[static_cast<std::size_t>(i)];
    r.evaluate(s.x, s.u, *data);
    r.computeJacobians(s.x, s.u, *data);
  }
  const double us =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() /
      kReps;
  std::printf("[report] contact tracking residual evaluate + Jacobians %.1f us (both swinging)\n",
              us);
}

// ── Solves through the handler core ─────────────────────────────────────────

class KinodynamicSolve : public KinodynamicOCPTest,
                         public ::testing::WithParamInterface<const char*> {
 protected:
  [[nodiscard]] static rtc::mpc::MPCSolverConfig Solver() {
    rtc::mpc::MPCSolverConfig s;
    s.prim_tol = 1e-4;
    s.dual_tol = 1e-4;
    s.mu_init = 1e-2;
    s.max_iters = 50;
    s.max_al_iters = 20;
    return s;
  }

  /// Init on @p h, one Solve from the stand state (the OCP's cold seed: each
  /// node's reference control).
  rtc::mpc::MPCSolveError SolveOnce(const HorizonReference& h, rtc::mpc::MPCSolution& out,
                                    double* ms = nullptr) {
    return SolveOnceFrom(h, tu::SnapshotAt(model_, stand_), out, ms);
  }

  /// As `SolveOnce`, from @p state.
  rtc::mpc::MPCSolveError SolveOnceFrom(const HorizonReference& h,
                                        const rtc::mpc::MPCStateSnapshot& state,
                                        rtc::mpc::MPCSolution& out, double* ms = nullptr) {
    KinodynamicOCPConfig cfg;
    EXPECT_EQ(LoadKinodynamicOCPConfig(tu::G1KinodynamicNode(GetParam()), handler_, cfg),
              Err::kNoError);
    mpc_ = std::make_unique<KinodynamicMPC>(cfg);
    const PhaseContext ctx = Context(&h);
    EXPECT_EQ(mpc_->Init(Solver(), handler_, {}, ctx), rtc::mpc::MPCInitError::kNoError);
    const auto t0 = std::chrono::steady_clock::now();
    const rtc::mpc::MPCSolveError e = mpc_->Solve(ctx, state, out);
    if (ms != nullptr) {
      *ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }
    return e;
  }

  [[nodiscard]] bool Finite(const rtc::mpc::MPCSolution& out) const {
    for (int k = 0; k <= out.horizon_length; ++k) {
      const auto i = static_cast<std::size_t>(k);
      for (int j = 0; j < model_.nq; ++j) {
        if (!std::isfinite(out.q_traj[i][static_cast<std::size_t>(j)])) {
          return false;
        }
      }
      for (int j = 0; j < model_.nv; ++j) {
        if (!std::isfinite(out.v_traj[i][static_cast<std::size_t>(j)])) {
          return false;
        }
      }
      if (k == out.horizon_length) {
        break;
      }
      for (int j = 0; j < handler_.nu(); ++j) {
        if (!std::isfinite(out.u_traj[i][static_cast<std::size_t>(j)])) {
          return false;
        }
      }
    }
    return true;
  }

  /// The solution's trajectory as solver vectors.
  void Trajectory(const rtc::mpc::MPCSolution& out, std::vector<Eigen::VectorXd>& xs,
                  std::vector<Eigen::VectorXd>& us) const {
    const int n = out.horizon_length;
    xs.assign(static_cast<std::size_t>(n + 1), Eigen::VectorXd::Zero(model_.nq + model_.nv));
    us.assign(static_cast<std::size_t>(n), Eigen::VectorXd::Zero(handler_.nu()));
    for (int k = 0; k <= n; ++k) {
      const auto i = static_cast<std::size_t>(k);
      xs[i].head(model_.nq) = Eigen::Map<const Eigen::VectorXd>(out.q_traj[i].data(), model_.nq);
      xs[i].tail(model_.nv) = Eigen::Map<const Eigen::VectorXd>(out.v_traj[i].data(), model_.nv);
      if (k < n) {
        us[i] = Eigen::Map<const Eigen::VectorXd>(out.u_traj[i].data(), handler_.nu());
      }
    }
  }

  /// The pass criterion of a converged solve: every constraint residual, as
  /// the OCP evaluates it on the solution, is within the solver's primal
  /// infeasibility, which is within its tolerance. Reports the residuals,
  /// the physical margins and the smallest joint margin to its limits.
  void ExpectResidualsWithinTolerance(const rtc::mpc::MPCSolution& out, const char* name,
                                      rtc::mpc::ConstraintResidualReport* report = nullptr) {
    std::vector<Eigen::VectorXd> xs;
    std::vector<Eigen::VectorXd> us;
    Trajectory(out, xs, us);
    rtc::mpc::ConstraintResidualReport r;
    ASSERT_TRUE(mpc_->ocp().ConstraintResiduals(xs, us, r));
    const double prim = out.residuals.prim_inf;
    std::printf(
        "[report] %s %s: prim %.3g, stance eq %.3g, swing eq %.3g, ineq violation %.3g; "
        "friction margin [N] %.1f / %.1f, CoP margin [m] %.4f / %.4f\n",
        GetParam(), name, prim, r.stance_equality_max, r.swing_equality_max,
        r.inequality_violation_max, r.min_friction_margin[0], r.min_friction_margin[1],
        r.min_cop_margin[0], r.min_cop_margin[1]);
    // The same values the solver measured, recomputed in another order: equal
    // to within rounding (relative 1e-12), not bit for bit.
    const double bound = prim * (1.0 + 1e-12);
    EXPECT_LE(r.stance_equality_max, bound) << name;
    EXPECT_LE(r.swing_equality_max, bound) << name;
    EXPECT_LE(r.inequality_violation_max, bound) << name;
    EXPECT_LE(prim, Solver().prim_tol) << name;
    double joint_margin = std::numeric_limits<double>::infinity();
    for (const auto& x : xs) {
      for (int j = 1; j < model_.njoints; ++j) {
        const auto& joint = model_.joints[static_cast<std::size_t>(j)];
        if (joint.nq() != 1 || joint.nv() != 1) {
          continue;
        }
        const double q = x[joint.idx_q()];
        joint_margin = std::min({joint_margin, q - model_.lowerPositionLimit[joint.idx_q()],
                                 model_.upperPositionLimit[joint.idx_q()] - q});
      }
    }
    std::printf("[report] %s %s: smallest joint margin %.4f rad\n", GetParam(), name, joint_margin);
    if (report != nullptr) {
      *report = r;
    }
  }

  /// Report: contact @p c's height error over the swing nodes, its vertical
  /// velocity at touchdown (node @p last + 1) and how far it goes below its
  /// stance height after that.
  void ReportSwing(const rtc::mpc::MPCSolution& out, const HorizonReference& h, int c, int first,
                   int last, const char* name) const {
    pinocchio::Data data(model_);
    const auto foot = static_cast<pinocchio::FrameIndex>(
        handler_.contact_frames()[static_cast<std::size_t>(c)].frame_id);
    const double z0 = h.stages[0].contact_pose[7 * static_cast<std::size_t>(c) + 2];
    double worst_z = 0.0;
    double lowest = std::numeric_limits<double>::infinity();
    double vz_touch = 0.0;
    for (int k = first; k <= out.horizon_length; ++k) {
      const auto i = static_cast<std::size_t>(k);
      const Eigen::Map<const Eigen::VectorXd> q(out.q_traj[i].data(), model_.nq);
      const Eigen::Map<const Eigen::VectorXd> v(out.v_traj[i].data(), model_.nv);
      pinocchio::forwardKinematics(model_, data, q, v);
      pinocchio::updateFramePlacements(model_, data);
      const double z = data.oMf[foot].translation().z();
      if (k <= last) {
        worst_z = std::max(
            worst_z, std::abs(z - h.stages[i].contact_pose[7 * static_cast<std::size_t>(c) + 2]));
      } else {
        lowest = std::min(lowest, z);
      }
      if (k == last + 1) {
        vz_touch = pinocchio::getFrameVelocity(model_, data, foot, pinocchio::LOCAL_WORLD_ALIGNED)
                       .linear()
                       .z();
      }
    }
    std::printf(
        "[report] %s %s: swing z error max %.4f m, touchdown v_z %.3f m/s, lowest z after "
        "touchdown %.4f m below the stance height\n",
        GetParam(), name, worst_z, vz_touch, z0 - lowest);
  }

  std::unique_ptr<KinodynamicMPC> mpc_;
};

/// Contact @p swing lifted over nodes `[first, last]` like `tu::SwingContact`,
/// but on a `1 − cos` height profile: it leaves and lands at zero vertical
/// velocity (the reference planner's touchdown velocity).
void SwingContactCosine(const rtc::mpc::RobotModelHandler& handler, HorizonReference& h, int swing,
                        int first, int last, double height, double impact) {
  tu::SwingContact(handler, h, swing, first, last, height, impact);
  const double T = (last - first + 2) * h.dt;
  const double w = 2.0 * std::acos(-1.0) / T;
  const auto i = static_cast<std::size_t>(swing);
  const double z0 = h.stages[0].contact_pose[7 * i + 2];
  for (int k = first; k <= last; ++k) {
    rtc::mpc::StageReference& s = h.stages[static_cast<std::size_t>(k)];
    const double t = (k - first + 1) * h.dt;
    s.contact_pose[7 * i + 2] = z0 + 0.5 * height * (1.0 - std::cos(w * t));
    s.contact_twist[6 * i + 2] = 0.5 * height * w * std::sin(w * t);
    s.contact_acceleration[6 * i + 2] = 0.5 * height * w * w * std::cos(w * t);
  }
}

TEST_P(KinodynamicSolve, StandingConvergesFromTheReferenceSeed) {
  const auto h = tu::StanceHorizon(handler_, stand_);
  auto out = std::make_unique<rtc::mpc::MPCSolution>();
  double ms = 0.0;
  ASSERT_EQ(SolveOnce(*h, *out, &ms), rtc::mpc::MPCSolveError::kNoError);
  std::printf("[report] %s stance: %d iterations, %.1f ms (solve, excl. setup)\n", GetParam(),
              out->iterations, ms);
  EXPECT_TRUE(out->converged);
  EXPECT_LE(out->iterations, 10);
  EXPECT_TRUE(Finite(*out));
  ExpectResidualsWithinTolerance(*out, "stance");
}

TEST_P(KinodynamicSolve, OneFootSwingingConvergesAndItsWrenchStaysAtTheReference) {
  auto h = tu::StanceHorizon(handler_, stand_);
  tu::SwingContact(handler_, *h, 1, 8, 19, 0.05, 0.5);
  auto out = std::make_unique<rtc::mpc::MPCSolution>();
  ASSERT_EQ(SolveOnce(*h, *out), rtc::mpc::MPCSolveError::kNoError);
  std::printf("[report] %s swing: %d iterations\n", GetParam(), out->iterations);
  EXPECT_TRUE(out->converged);
  EXPECT_LE(out->iterations, 50);
  ASSERT_TRUE(Finite(*out));

  // The swinging contact's wrench slot is bound to its reference (0) by R alone.
  const int off = handler_.layout().contact_u_offset[1];
  double worst_slot = 0.0;
  for (int k = 8; k <= 19; ++k) {
    for (int j = 0; j < 6; ++j) {
      worst_slot = std::max(
          worst_slot,
          std::abs(out->u_traj[static_cast<std::size_t>(k)][static_cast<std::size_t>(off + j)]));
    }
  }
  std::printf("[report] %s swing: inactive wrench slot max |u - u_ref| %.3g\n", GetParam(),
              worst_slot);
  EXPECT_LE(worst_slot, 1e-9);

  // Report: the swing foot's height against its reference (the position
  // weight is 0, so nothing makes it follow yet).
  pinocchio::Data data(model_);
  const auto foot = static_cast<pinocchio::FrameIndex>(handler_.contact_frames()[1].frame_id);
  double worst_z = 0.0;
  for (int k = 8; k <= 19; ++k) {
    const Eigen::Map<const Eigen::VectorXd> q(out->q_traj[static_cast<std::size_t>(k)].data(),
                                              model_.nq);
    pinocchio::framesForwardKinematics(model_, data, q);
    worst_z = std::max(worst_z, std::abs(data.oMf[foot].translation().z() -
                                         h->stages[static_cast<std::size_t>(k)].contact_pose[9]));
  }
  std::printf("[report] %s swing: foot height max |z - z_ref| %.3g m (reference peak 0.05)\n",
              GetParam(), worst_z);
  ExpectResidualsWithinTolerance(*out, "swing");
}

TEST_P(KinodynamicSolve, OneFootSwingingWithZeroTouchdownVelocityConverges) {
  auto h = tu::StanceHorizon(handler_, stand_);
  SwingContactCosine(handler_, *h, 0, 8, 19, 0.05, 1.0);
  auto out = std::make_unique<rtc::mpc::MPCSolution>();
  double ms = 0.0;
  ASSERT_EQ(SolveOnce(*h, *out, &ms), rtc::mpc::MPCSolveError::kNoError);
  std::printf("[report] %s swing (1 - cos, left): %d iterations, %.1f ms per iteration\n",
              GetParam(), out->iterations, ms / std::max(1, out->iterations));
  EXPECT_TRUE(out->converged);
  EXPECT_LE(out->iterations, Solver().max_iters);
  ASSERT_TRUE(Finite(*out));
  ExpectResidualsWithinTolerance(*out, "swing (1 - cos)");
  ReportSwing(*out, *h, 0, 8, 19, "swing (1 - cos)");

  // Report: the next tick, warm-started — the schedule one node further on,
  // so liftoff and touchdown move by one node.
  auto next = tu::StanceHorizon(handler_, stand_);
  SwingContactCosine(handler_, *next, 0, 7, 18, 0.05, 1.0);
  rtc::mpc::MPCStateSnapshot state = tu::SnapshotAt(model_, stand_);
  state.sequence = 1;
  auto out2 = std::make_unique<rtc::mpc::MPCSolution>();
  const rtc::mpc::MPCSolveError e2 = mpc_->Solve(Context(next.get()), state, *out2);
  std::printf(
      "[report] %s two ticks: tick 1 %d iterations (prim %.3g), tick 2 %s %d iterations "
      "(prim %.3g, converged %d)\n",
      GetParam(), out->iterations, out->residuals.prim_inf,
      e2 == rtc::mpc::MPCSolveError::kNoError ? "ok" : "error", out2->iterations,
      out2->residuals.prim_inf, out2->converged ? 1 : 0);
}

// Report only: the half-sine reference lands at a non-zero vertical velocity.
TEST_P(KinodynamicSolve, HalfSineSwingReport) {
  auto h = tu::StanceHorizon(handler_, stand_);
  tu::SwingContact(handler_, *h, 0, 8, 19, 0.05, 1.0);
  auto out = std::make_unique<rtc::mpc::MPCSolution>();
  ASSERT_EQ(SolveOnce(*h, *out), rtc::mpc::MPCSolveError::kNoError);
  std::printf("[report] %s swing (half sine, left): %d iterations, converged %d\n", GetParam(),
              out->iterations, out->converged ? 1 : 0);
  if (out->converged && Finite(*out)) {
    ReportSwing(*out, *h, 0, 8, 19, "swing (half sine)");
  }
}

TEST_P(KinodynamicSolve, StandingConvergesFromAMovingBase) {
  const auto h = tu::StanceHorizon(handler_, stand_);
  rtc::mpc::MPCStateSnapshot state = tu::SnapshotAt(model_, stand_);
  // Base LOCAL linear velocity; the stand base is level, so these are world axes too.
  state.v[0] = 0.05;
  state.v[1] = 0.05;
  state.v[2] = 0.05;
  auto out = std::make_unique<rtc::mpc::MPCSolution>();
  ASSERT_EQ(SolveOnceFrom(*h, state, *out), rtc::mpc::MPCSolveError::kNoError);
  std::printf("[report] %s moving base: %d iterations\n", GetParam(), out->iterations);
  EXPECT_TRUE(out->converged);
  EXPECT_LE(out->iterations, 10);
  ASSERT_TRUE(Finite(*out));
  ExpectResidualsWithinTolerance(*out, "moving base");

  // The initial node's stance rows: the node the solver cannot move.
  std::vector<Eigen::VectorXd> xs;
  std::vector<Eigen::VectorXd> us;
  Trajectory(*out, xs, us);
  auto& stage0 = *mpc_->ocp().problem().stages_[0];
  const auto* eq = stage0.constraints_.getConstraint<rtc::mpc::ContactConstraintResidual>(
      KinodynamicOCP::kEqualityConstraint);
  ASSERT_NE(eq, nullptr);
  auto d = eq->createData();
  eq->evaluate(xs[0], us[0], *d);
  const double stage0_eq = d->value_.cwiseAbs().maxCoeff();
  std::printf("[report] %s moving base: stage-0 stance equality %.3g\n", GetParam(), stage0_eq);
  EXPECT_LE(stage0_eq, 1e-4);
}

// A tick whose horizon has another shape is refused through the core — whose
// rebuild attempt fails without touching the problem — and the next good tick
// solves on the same problem, warm.
TEST_P(KinodynamicSolve, AMismatchedTickIsRefusedAndTheNextSolvesWarm) {
  const auto h = tu::StanceHorizon(handler_, stand_);
  auto out = std::make_unique<rtc::mpc::MPCSolution>();
  ASSERT_EQ(SolveOnce(*h, *out), rtc::mpc::MPCSolveError::kNoError);
  const auto* problem = &mpc_->ocp().problem();
  auto bad = std::make_unique<HorizonReference>(*h);
  bad->dt = 0.0351;
  EXPECT_EQ(mpc_->Solve(Context(bad.get()), tu::SnapshotAt(model_, stand_), *out),
            rtc::mpc::MPCSolveError::kOCPRebuildFailed);
  EXPECT_EQ(mpc_->Solve(Context(nullptr), tu::SnapshotAt(model_, stand_), *out),
            rtc::mpc::MPCSolveError::kOCPRebuildFailed);
  ASSERT_TRUE(mpc_->ocp().Built());
  EXPECT_EQ(&mpc_->ocp().problem(), problem);
  ASSERT_EQ(mpc_->Solve(Context(h.get()), tu::SnapshotAt(model_, stand_), *out),
            rtc::mpc::MPCSolveError::kNoError);
  EXPECT_TRUE(out->converged);
  EXPECT_LE(out->iterations, 2) << "warm from the converged standing solve";
}

INSTANTIATE_TEST_SUITE_P(Integrators, KinodynamicSolve,
                         ::testing::Values("semi_implicit_euler", "rk2"));

}  // namespace
