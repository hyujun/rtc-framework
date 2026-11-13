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
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
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
using rtc::mpc::ContactTrackingResidual;
using rtc::mpc::HorizonReference;
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
                       "input_weights", "contact_tracking"};
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
    EXPECT_EQ(stage->numConstraints(), 0U);
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
  // No constraint rows, so a control box is refused rather than dropped.
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

// ── Cost values ─────────────────────────────────────────────────────────────

// Stage cost == dt · (½‖x ⊖ x_ref‖²_Q + ½‖u − u_ref‖²_R + ½ Σ w r²), the
// difference from pinocchio and the residual from a separate instance on a
// separate ODE of the stage's mode; terminal == ½‖x ⊖ x_ref‖² over 4·Q_f.
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
                                         0.5 * rd->value_.dot(w_track.cwiseProduct(rd->value_)));
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

  /// Init on @p h, one Solve from the stand state (zero-control cold seed).
  rtc::mpc::MPCSolveError SolveOnce(const HorizonReference& h, rtc::mpc::MPCSolution& out,
                                    double* ms = nullptr) {
    KinodynamicOCPConfig cfg;
    EXPECT_EQ(LoadKinodynamicOCPConfig(tu::G1KinodynamicNode(GetParam()), handler_, cfg),
              Err::kNoError);
    mpc_ = std::make_unique<KinodynamicMPC>(cfg);
    const PhaseContext ctx = Context(&h);
    EXPECT_EQ(mpc_->Init(Solver(), handler_, {}, ctx), rtc::mpc::MPCInitError::kNoError);
    const auto t0 = std::chrono::steady_clock::now();
    const rtc::mpc::MPCSolveError e = mpc_->Solve(ctx, tu::SnapshotAt(model_, stand_), out);
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

  std::unique_ptr<KinodynamicMPC> mpc_;
};

TEST_P(KinodynamicSolve, StandingConvergesFromTheZeroSeed) {
  const auto h = tu::StanceHorizon(handler_, stand_);
  auto out = std::make_unique<rtc::mpc::MPCSolution>();
  double ms = 0.0;
  ASSERT_EQ(SolveOnce(*h, *out, &ms), rtc::mpc::MPCSolveError::kNoError);
  std::printf("[report] %s stance: %d iterations, %.1f ms (solve, excl. setup)\n", GetParam(),
              out->iterations, ms);
  EXPECT_TRUE(out->converged);
  EXPECT_LE(out->iterations, 10);
  EXPECT_TRUE(Finite(*out));
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
