// The iteration caps of the installed g1_loco whole_body_qp.solver, read from
// the installed file (no value is asserted here): what a solve needs against
// what the caps allow, and what a tick that cannot be solved costs.
//
//   - standing, one foot released, both again, the other released, both again:
//     every tick solves with half the caps to spare (outer steps against
//     max_iter, a solve's inner iterations against max_iter_in) — caps set
//     near the need would pass the trace and miss on the first harder tick;
//   - one forced miss: the next tick starts from the equality-constrained
//     solution and must solve inside the same caps, or one miss becomes a run
//     of them and the solve health faults. Under caps no cold start fits in
//     (1 × 1) the same sequence never solves and faults — a cold start is the
//     first tick's start too, so no caps separate the two;
//   - a contradictory QP every tick: no solution is taken, it ends inside the
//     caps, and the health reaches Fault after health.max_invalid_ticks + 1 of
//     them. Whether ProxQP refuses it before the caps depends on the form of
//     the constraints (with friction_cone.zero_torsion_equality it runs to
//     them) — the time it costs is in the YAML's header;
//   - 300 states off the nominal trace: every tick that solves under caps of
//     200 × 500 solves under the installed ones.
//
// The QP is the controller's: LocomotionFormulationBuilder on the CM's model
// with the hand group's joints locked, the contact set and torso frame of
// model.*. The state is held (no integration): the references are consistent
// with it only while both feet stand, which is the harder case for the solver.
#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"
#include "rtc_controller_manager/controller_config_loader.hpp"
#include "rtc_tsid/controller/solve_health.hpp"
#include "rtc_tsid/core/constraint_base.hpp"
#include "rtc_tsid/formulation/wqp_formulation.hpp"
#include "rtc_tsid/locomotion/locomotion_formulation.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "shipped_catching_bringup_fixture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

namespace fx = integrated_bringup::testfx;
namespace loco = integrated_bringup::locomotion;
using rtc::tsid::CommandOutput;
using rtc::tsid::SolveHealthState;

constexpr const char* kProfile = "g1_loco";

// The profile's control period, from its own control_rate.
double ControlPeriod() {
  return 1.0 / fx::ShippedSimParams(kProfile)["control_rate"].as<double>();
}

// A row the dynamics cannot meet: the base accelerating upward at 1000 m/s²
// or more, which the torque limits and the friction cones leave no force for.
// The contradiction runs through the equations of motion, so ProxQP finds no
// two-row certificate, and has to work for the one it returns. Inactive it
// adds one unbounded row.
class Contradiction final : public rtc::tsid::ConstraintBase {
 public:
  [[nodiscard]] std::string_view Name() const noexcept override { return "contradiction"; }

  void Init(const pinocchio::Model&, const rtc::tsid::RobotModelInfo&, rtc::tsid::PinocchioCache&,
            const YAML::Node&) override {}

  [[nodiscard]] int EqDim(const rtc::tsid::ContactState&) const noexcept override { return 0; }

  [[nodiscard]] int IneqDim(const rtc::tsid::ContactState&) const noexcept override { return 1; }

  [[nodiscard]] int MaxEqDim(const rtc::tsid::ContactManagerConfig&) const noexcept override {
    return 0;
  }

  [[nodiscard]] int MaxIneqDim(const rtc::tsid::ContactManagerConfig&) const noexcept override {
    return 1;
  }

  void ComputeEquality(const rtc::tsid::PinocchioCache&, const rtc::tsid::ContactState&,
                       const rtc::tsid::RobotModelInfo&, int, Eigen::Ref<Eigen::MatrixXd>,
                       Eigen::Ref<Eigen::VectorXd>) noexcept override {}

  void ComputeInequality(const rtc::tsid::PinocchioCache&, const rtc::tsid::ContactState&,
                         const rtc::tsid::RobotModelInfo& info, int, Eigen::Ref<Eigen::MatrixXd> C,
                         Eigen::Ref<Eigen::VectorXd> l,
                         Eigen::Ref<Eigen::VectorXd> u) noexcept override {
    static_cast<void>(info);
    C(0, 2) = 1.0;
    l(0) = 1000.0;
    u(0) = std::numeric_limits<double>::infinity();
  }
};

YAML::Node InstalledController() {
  return rtc::LoadControllerConfig(
      ament_index_cpp::get_package_share_directory("integrated_bringup") +
          "/config/g1_loco/controllers/locomotion_controller.yaml",
      "locomotion_controller");
}

// The body model: the CM's actuated model with the hand groups' joints locked.
std::shared_ptr<const pinocchio::Model> BodyModel(const loco::HumanoidModelConfig& model_cfg) {
  const std::shared_ptr<const pinocchio::Model> actuated =
      fx::ShippedModelBuilder(kProfile)->GetActuatedModel();
  const YAML::Node devices = fx::ShippedSimParams(kProfile)["devices"];
  std::vector<pinocchio::JointIndex> lock;
  for (const std::string& group : model_cfg.hand_groups) {
    for (const auto& name : devices[group]["joint_state_names"].as<std::vector<std::string>>()) {
      lock.push_back(actuated->getJointId(name));
    }
  }
  return std::make_shared<const pinocchio::Model>(
      pinocchio::buildReducedModel(*actuated, lock, pinocchio::neutral(*actuated)));
}

struct Rig {
  YAML::Node root = InstalledController();
  loco::HumanoidModelConfig model_cfg =
      loco::LoadHumanoidModelConfig(root["model"], "locomotion_controller.model");
  std::shared_ptr<const pinocchio::Model> model = BodyModel(model_cfg);
  rtc::tsid::RobotModelInfo info;
  rtc::tsid::PinocchioCache cache;
  rtc::tsid::LocomotionFormulationBuilder builder;
  rtc::tsid::ContactState contacts;
  rtc::tsid::ControlState state;
  rtc::tsid::ControlReference ref;
  Contradiction* contradiction{nullptr};
  int max_iter{0};
  int max_iter_in{0};
  int max_invalid_ticks{0};

  // `caps`: {max_iter, max_iter_in} in place of the installed ones (the control).
  explicit Rig(const std::pair<int, int>* caps = nullptr) {
    YAML::Node wbq = YAML::Clone(root["whole_body_qp"]);
    if (caps != nullptr) {
      wbq["solver"]["max_iter"] = caps->first;
      wbq["solver"]["max_iter_in"] = caps->second;
    }
    rtc::tsid::LocomotionModelSpec spec;
    for (const auto& c : model_cfg.contacts) {
      spec.contacts.push_back({c.name, c.frame, c.dim});
    }
    spec.torso_frame = model_cfg.torso_frame;
    info.Build(*model, YAML::Node{});
    cache.Init(model, rtc::tsid::LocomotionFormulationBuilder::ContactFrameIds(*model, spec));
    builder.Init(*model, info, cache, spec, wbq, ControlPeriod());
    const auto& wqp = dynamic_cast<const rtc::tsid::WQPFormulation&>(Formulation());
    max_iter = wqp.SolverConfig().max_iter;
    max_iter_in = wqp.SolverConfig().max_iter_in;
    max_invalid_ticks = wbq["health"]["max_invalid_ticks"].as<int>();

    auto con = std::make_unique<Contradiction>();
    contradiction = con.get();
    contradiction->SetActive(false);
    Formulation().AddConstraint(std::move(con));

    // Knees bent a little, the base lowered so the lower sole touches z = 0.
    Eigen::VectorXd q = pinocchio::neutral(*model);
    for (const char* side : {"left", "right"}) {
      SetJoint(q, std::string(side) + "_hip_pitch_joint", -0.1);
      SetJoint(q, std::string(side) + "_knee_joint", 0.2);
      SetJoint(q, std::string(side) + "_ankle_pitch_joint", -0.1);
    }
    pinocchio::Data d(*model);
    pinocchio::framesForwardKinematics(*model, d, q);
    double z_min = std::numeric_limits<double>::infinity();
    for (const auto& c : model_cfg.contacts) {
      z_min = std::min(z_min, d.oMf[model->getFrameId(c.frame)].translation().z());
    }
    q(2) -= z_min;
    state.q = q;
    state.v = Eigen::VectorXd::Zero(model->nv);
    cache.Update(state.q, state.v);
    contacts = builder.MakeContactState();
    ref.Init(info.nq, info.nv, info.n_actuated, builder.Contacts().max_contact_vars);
    ref.q_des = q;
    Support({true, true});
  }

  rtc::tsid::FormulationBase& Formulation() { return builder.Controller().Formulation(); }

  void SetJoint(Eigen::VectorXd& q, const std::string& name, double value) const {
    ASSERT_TRUE(model->existJointName(name)) << name;
    q(model->joints[model->getJointId(name)].idx_q()) = value;
  }

  // The feet that stand, and the weight shared evenly among them as λ_des.
  void Support(const std::vector<bool>& standing) {
    const double weight = pinocchio::computeTotalMass(*model) * model->gravity.linear().norm();
    const auto n = static_cast<double>(std::count(standing.begin(), standing.end(), true));
    ref.lambda_des.setZero();
    Eigen::Index offset = 0;
    for (std::size_t c = 0; c < model_cfg.contacts.size(); ++c) {
      auto& slot = contacts.contacts[c];
      slot.active = standing[c];
      slot.activation = standing[c] ? 1.0 : 0.0;
      slot.activation_target = slot.activation;
      if (standing[c]) {
        ref.lambda_des(offset + 2) = weight / n;
      }
      offset += model_cfg.contacts[c].dim;
    }
    contacts.RecomputeActive(builder.Contacts());
  }

  const CommandOutput& Solve() { return builder.Compute(state, ref, cache, contacts); }
};

TEST(G1LocoQpIterationBudget, AStandReleaseReturnLeavesHalfTheCaps) {
  Rig rig;
  ASSERT_EQ(rig.model_cfg.contacts.size(), 2U);
  constexpr int kTicksPerPhase = 40;
  const std::vector<std::vector<bool>> phases = {
      {true, true}, {true, false}, {true, true}, {false, true}, {true, true}};
  int outer_max = 0;
  int inner_max = 0;
  for (std::size_t p = 0; p < phases.size(); ++p) {
    rig.Support(phases[p]);
    for (int k = 0; k < kTicksPerPhase; ++k) {
      const CommandOutput& out = rig.Solve();
      ASSERT_TRUE(out.tau_valid) << "phase " << p << " tick " << k << " status " << out.qp_status;
      outer_max = std::max(outer_max, out.outer_iterations);
      inner_max = std::max(inner_max, out.qp_iterations);
    }
  }
  RecordProperty("max_iter", rig.max_iter);
  RecordProperty("max_iter_in", rig.max_iter_in);
  RecordProperty("outer_iterations_max", outer_max);
  RecordProperty("inner_iterations_max", inner_max);
  EXPECT_GT(outer_max, 0) << "premise: the solver was asked to work";
  EXPECT_LE(2 * outer_max, rig.max_iter);
  // All the inner iterations of a solve against ONE outer step's cap: the sum
  // over the caps' product would follow from the line above.
  EXPECT_LE(2 * inner_max, rig.max_iter_in);
  EXPECT_EQ(rig.builder.Health().State(), SolveHealthState::kNominal);
}

// A forced miss on tick `miss_at` of single support, then `after` more ticks.
// Returns the longest run of invalid ticks; `fault` whether the health faulted.
struct MissRun {
  int longest_bad_run{0};
  bool fault{false};
  bool miss_was_a_miss{false};
  bool next_started_cold{false};
  bool next_valid{false};
};

MissRun RunWithOneMiss(Rig& rig) {
  MissRun run;
  rig.Support({true, false});
  int bad = 0;
  const auto tick = [&]() -> const CommandOutput& {
    const CommandOutput& out = rig.Solve();
    bad = out.tau_valid ? 0 : bad + 1;
    run.longest_bad_run = std::max(run.longest_bad_run, bad);
    run.fault = run.fault || rig.builder.Health().State() == SolveHealthState::kFault;
    return out;
  };
  for (int k = 0; k < 20; ++k) {
    static_cast<void>(tick());
  }
  rig.contradiction->SetActive(true);
  run.miss_was_a_miss = !tick().tau_valid;
  rig.contradiction->SetActive(false);
  const CommandOutput& next = tick();
  run.next_started_cold = !next.qp_warm_started;
  run.next_valid = next.tau_valid;
  for (int k = 0; k < 20; ++k) {
    static_cast<void>(tick());
  }
  return run;
}

TEST(G1LocoQpIterationBudget, TheTickAfterAMissSolvesInsideTheCaps) {
  Rig rig;
  const MissRun run = RunWithOneMiss(rig);
  EXPECT_TRUE(run.miss_was_a_miss) << "premise";
  EXPECT_TRUE(run.next_started_cold) << "premise: the equality-constrained start";
  EXPECT_TRUE(run.next_valid);
  EXPECT_EQ(run.longest_bad_run, 1);
  EXPECT_FALSE(run.fault);

  // The control: under caps no cold start fits in, the same sequence is one
  // run of misses and a fault.
  const std::pair<int, int> tight{1, 1};
  Rig control(&tight);
  const MissRun chained = RunWithOneMiss(control);
  EXPECT_GT(chained.longest_bad_run, rig.max_invalid_ticks);
  EXPECT_TRUE(chained.fault);
}

// What the contradictory QP costs: the most outer steps, inner iterations and
// time of a solve.
struct ContradictionCost {
  int outer{0};
  int inner{0};
  double solve_us{0.0};
};

TEST(G1LocoQpIterationBudget, AContradictoryQpEndsInsideTheCapsAndFaults) {
  Rig rig;
  ASSERT_TRUE(rig.Solve().tau_valid);
  rig.contradiction->SetActive(true);
  ContradictionCost cost;
  for (int k = 0; k < rig.max_invalid_ticks + 5; ++k) {
    const CommandOutput& out = rig.Solve();
    EXPECT_FALSE(out.tau_valid) << k;
    EXPECT_GT(out.qp_status, 0) << k << ": the solver ran and gave up";
    EXPECT_LE(out.outer_iterations, rig.max_iter) << k;
    EXPECT_LE(out.qp_iterations, rig.max_iter * rig.max_iter_in) << k;
    cost.outer = std::max(cost.outer, out.outer_iterations);
    cost.inner = std::max(cost.inner, out.qp_iterations);
    cost.solve_us = std::max(cost.solve_us, out.solve_time_us);
    EXPECT_EQ(rig.builder.Health().State(),
              k < rig.max_invalid_ticks ? SolveHealthState::kDegraded : SolveHealthState::kFault)
        << k;
  }
  // The QP comes back once the contradiction is gone (the latch is the controller's).
  rig.contradiction->SetActive(false);
  EXPECT_TRUE(rig.Solve().tau_valid);

  RecordProperty("contradictory_outer_iterations_max", cost.outer);
  RecordProperty("contradictory_inner_iterations_max", cost.inner);
  RecordProperty("contradictory_solve_us_max", std::to_string(cost.solve_us));
}

// Off the nominal trace: joints thrown about the stand pose (some into their
// limits' margins), the body moving, the force reference left at the standing
// one — each state solved from a cold start and again warm, on both feet and
// on one. A tick that solves at all (under caps of 200 × 500) must solve
// under the installed caps; the most outer steps any of them took is recorded.
TEST(G1LocoQpIterationBudget, OffNominalTicksThatSolveAtAllSolveInsideTheCaps) {
  Rig rig;
  const std::pair<int, int> loose{200, 500};
  Rig ref(&loose);
  const Eigen::VectorXd q0 = rig.state.q;
  const int nv = rig.model->nv;
  std::mt19937 gen(1198);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  int solved_loose = 0;
  int solved_both = 0;
  int outer_max_loose = 0;
  int inner_max_loose = 0;
  int outer_max = 0;
  constexpr int kStates = 300;
  for (int n = 0; n < kStates; ++n) {
    const double q_amp = 0.05 + 0.45 * (n % 10) / 9.0;      // [rad] up to 0.5
    const double v_amp = 0.2 + 3.8 * ((n / 10) % 5) / 4.0;  // [rad/s] up to 4
    Eigen::VectorXd q = q0;
    Eigen::VectorXd v = Eigen::VectorXd::Zero(nv);
    for (int j = 0; j < nv - 6; ++j) {
      q(7 + j) = std::clamp(q0(7 + j) + q_amp * unit(gen), rig.model->lowerPositionLimit(7 + j),
                            rig.model->upperPositionLimit(7 + j));
      v(6 + j) = v_amp * unit(gen);
    }
    for (int j = 0; j < 6; ++j) {
      v(j) = 0.25 * v_amp * unit(gen);
    }
    const std::vector<bool> support = n % 3 == 0   ? std::vector<bool>{true, true}
                                      : n % 3 == 1 ? std::vector<bool>{true, false}
                                                   : std::vector<bool>{false, true};
    for (Rig* r : {&rig, &ref}) {
      r->state.q = q;
      r->state.v = v;
      r->cache.Update(q, v);
      r->Support(support);
      r->builder.Reset();
    }
    for (int pass = 0; pass < 2; ++pass) {  // cold, then warm on the same QP
      const CommandOutput& loose_out = ref.Solve();
      const bool loose_ok = loose_out.tau_valid;
      const int loose_outer = loose_out.outer_iterations;
      const int loose_inner = loose_out.qp_iterations;
      const CommandOutput& out = rig.Solve();
      if (loose_ok) {
        ++solved_loose;
        outer_max_loose = std::max(outer_max_loose, loose_outer);
        inner_max_loose = std::max(inner_max_loose, loose_inner);
        solved_both += out.tau_valid ? 1 : 0;
        EXPECT_TRUE(out.tau_valid) << "state " << n << " pass " << pass << ": solved in "
                                   << loose_outer << " outer steps under 200 x 500, status "
                                   << out.qp_status << " under the installed caps";
      }
      if (out.tau_valid) {
        outer_max = std::max(outer_max, out.outer_iterations);
      }
    }
  }
  RecordProperty("off_nominal_solves", 2 * kStates);
  RecordProperty("off_nominal_solved_200x500", solved_loose);
  RecordProperty("off_nominal_solved_installed", solved_both);
  RecordProperty("off_nominal_outer_max_200x500", outer_max_loose);
  RecordProperty("off_nominal_inner_max_200x500", inner_max_loose);
  RecordProperty("off_nominal_outer_max_installed", outer_max);
  EXPECT_GT(solved_loose, kStates) << "premise: most of these ticks can be solved";
}

}  // namespace
