// ── HandJointController: the feedforward lane and the reference lane ────────
//
// The claim this file pins is the one the shipped p2 profile stands on: the
// torque on the wire is the hand's gravity term at the measured pose, with the
// loop closures accounted for, under the hold policy the header states. The
// oracle for the closed chain is the PHYSICS, not another call of the same
// code: the reduced gravity force of a constrained mechanism is the gradient
// of its potential energy along the loop-consistent configuration manifold,
//     g_a = ∂U(q_full(q_a)) / ∂q_a,
// and that gradient is taken here by central differences of pinocchio's
// potential energy on configurations the non-RT ClosedChainHandle converges
// to. It shares no projection step, no reduction matrix and no joint order
// with the RT handle the controller uses. The plain tree's oracle is
// pinocchio's own gravity vector on the tree model, addressed by joint NAME.
//
// The inertia term k_M · M(q) · q̈_ref has its own two oracles, again sharing
// nothing with the controller's path: the closed chain's reduced inertia is
// G_fdᵀ · M_full · G_fd, with the reduction map G_fd taken by central
// differences of the non-RT projection and M_full · x by pinocchio's RNEA at
// zero velocity (no CRBA, no RT handle); the plain tree's is RNEA on the tree
// model by joint NAME. q̈_ref itself is checked against the second difference
// of the reference the controller put on the wire.
//
// Everything else: the hold policy (zero before the first valid term, last
// value through a held / unreadable tick, zero under E-STOP and with the lane
// off), the clamp against the YAML bound, the goal lane, the three modes
// (tracking / damping / damping_feedforward: what the two lanes carry), the
// refusals at configure, the shipped p2 YAML, and on_configure on a real
// LifecycleNode.
#include "hand_joint_test_rig.hpp"
#include "rtc_urdf_bridge/closed_chain_handle.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/energy.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifndef RTC_DEMO_SHARED_CONFIG_DIR
#error "RTC_DEMO_SHARED_CONFIG_DIR must be defined by CMake"
#endif

namespace {

using namespace hand_joint_rig;  // NOLINT(google-build-using-namespace)
using integrated_bringup::HandJointMode;
using integrated_bringup::HandJointModeName;
using integrated_bringup::JointFeedforwardLogPod;
using integrated_bringup::ParseHandJointConfig;
using integrated_bringup::ParseHandJointMode;

void SetMode(HandJointController& ctrl, HandJointMode mode) {
  auto gains = ctrl.GetGains();
  gains.mode = mode;
  ctrl.SetGains(gains);
}

constexpr double kTauTol = 1e-4;  // N·m, the pre-registered oracle tolerance

std::size_t IndexOf(const std::vector<std::string>& names, const std::string& name) {
  const auto it = std::find(names.begin(), names.end(), name);
  EXPECT_NE(it, names.end()) << name;
  return static_cast<std::size_t>(it - names.begin());
}

// ── The closed-chain oracle: ∂U/∂q_a by central differences ─────────────────
// `q_a` in the ORACLE handle's independent order; the result is mapped to the
// controller's device order by name.
std::map<std::string, double> PotentialGradient(const Rig& rig,
                                                const std::map<std::string, double>& q_by_name) {
  const auto model = rig.builder->GetFullModel();
  rub::ClosedChainHandle oracle(model, rig.builder->GetConstraintModels(),
                                rig.builder->GetClosureActuatedJointIds(), {});
  const std::vector<std::string> indep = oracle.GetIndependentJointNames();
  std::vector<double> q_a(indep.size());
  for (std::size_t k = 0; k < indep.size(); ++k) {
    q_a[k] = q_by_name.at(indep[k]);
  }
  pinocchio::Data data(*model);
  auto energy = [&](const std::vector<double>& q) {
    const auto st = oracle.Update(q);
    EXPECT_TRUE(st.converged) << "oracle projection did not converge";
    return pinocchio::computePotentialEnergy(*model, data, oracle.GetFullConfiguration());
  };
  constexpr double h = 1e-5;
  std::map<std::string, double> grad;
  for (std::size_t k = 0; k < indep.size(); ++k) {
    std::vector<double> plus = q_a;
    std::vector<double> minus = q_a;
    plus[k] += h;
    minus[k] -= h;
    const double up = energy(plus);
    const double um = energy(minus);
    grad[indep[k]] = (up - um) / (2.0 * h);
  }
  return grad;
}

// ── The closed-chain inertia oracle: G_fdᵀ · M_full · G_fd · a ──────────────
// `a_by_name` is the reduced acceleration by independent-joint name; the
// result is the reduced inertia torque by the same names.
std::map<std::string, double> ReducedInertiaTimes(const Rig& rig,
                                                  const std::map<std::string, double>& q_by_name,
                                                  const std::map<std::string, double>& a_by_name) {
  const auto model = rig.builder->GetFullModel();
  rub::ClosedChainHandle oracle(model, rig.builder->GetConstraintModels(),
                                rig.builder->GetClosureActuatedJointIds(), {});
  const std::vector<std::string> indep = oracle.GetIndependentJointNames();
  const auto na = static_cast<Eigen::Index>(indep.size());
  std::vector<double> q_a(indep.size());
  Eigen::VectorXd a(na);
  for (std::size_t k = 0; k < indep.size(); ++k) {
    q_a[k] = q_by_name.at(indep[k]);
    a[static_cast<Eigen::Index>(k)] = a_by_name.at(indep[k]);
  }
  auto project = [&](const std::vector<double>& q) {
    const auto st = oracle.Update(q);
    EXPECT_TRUE(st.converged) << "oracle projection did not converge";
    return Eigen::VectorXd(oracle.GetFullConfiguration());
  };
  const Eigen::VectorXd q_full = project(q_a);
  // G_fd: column k = ∂q_full/∂q_a[k], central differences on the manifold.
  constexpr double h = 1e-5;
  Eigen::MatrixXd g_fd(model->nv, na);
  for (Eigen::Index k = 0; k < na; ++k) {
    std::vector<double> plus = q_a;
    std::vector<double> minus = q_a;
    plus[static_cast<std::size_t>(k)] += h;
    minus[static_cast<std::size_t>(k)] -= h;
    const Eigen::VectorXd qp = project(plus);
    const Eigen::VectorXd qm = project(minus);
    g_fd.col(k) = pinocchio::difference(*model, qm, qp) / (2.0 * h);
  }
  // M_full · x = RNEA(q, 0, x) − RNEA(q, 0, 0).
  pinocchio::Data data(*model);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(model->nv);
  const Eigen::VectorXd a_full = g_fd * a;
  const Eigen::VectorXd m_a_full = pinocchio::rnea(*model, data, q_full, zero, a_full);
  const Eigen::VectorXd g_full = pinocchio::rnea(*model, data, q_full, zero, zero);
  const Eigen::VectorXd tau_a = g_fd.transpose() * (m_a_full - g_full);
  std::map<std::string, double> out;
  for (std::size_t k = 0; k < indep.size(); ++k) {
    out[indep[k]] = tau_a[static_cast<Eigen::Index>(k)];
  }
  return out;
}

/// A goal off `q` by a different amount (and sign) on every joint, so a
/// swapped joint or a dropped off-diagonal term changes the inertia torque.
std::vector<double> StaggeredGoal(const std::vector<double>& q) {
  std::vector<double> goal = q;
  for (std::size_t i = 0; i < goal.size(); ++i) {
    goal[i] += 0.1 * static_cast<double>(1 + i % 3) * (i % 2 == 0 ? 1.0 : -1.0);
  }
  return goal;
}

/// Three consecutive ticks a few ticks into the goal: the middle one is where
/// the term is judged, the outer two give the reference's second difference.
struct MidMotion {
  std::vector<double> q_ref_prev, q_ref, q_ref_next;
  std::vector<double> qdd, tau_m, tau_ff, g;
  // The tick record of the judged tick.
  std::vector<double> log_qdd, log_tau_inertia;
  double log_inertia_gain{0.0};
};

MidMotion RunIntoAGoal(HandJointController& ctrl, const std::vector<double>& q, std::uint64_t t0,
                       int ticks_in = 15) {
  ctrl.SetDeviceTarget(0, StaggeredGoal(q));
  MidMotion m;
  const auto n = static_cast<std::size_t>(ctrl.Dof());
  auto refs = [&](const ControllerOutput& out) {
    std::vector<double> r(n);
    for (std::size_t i = 0; i < n; ++i) {
      r[i] = out.devices[0].commands[i];
    }
    return r;
  };
  for (int k = 0; k <= ticks_in + 1; ++k) {
    const ControllerOutput out =
        ctrl.Compute(MakeState(q, true, t0 + static_cast<std::uint64_t>(k)));
    EXPECT_TRUE(ctrl.LastTick().traj_active) << "the goal ended before tick " << k;
    if (k == ticks_in - 1) {
      m.q_ref_prev = refs(out);
    } else if (k == ticks_in) {
      m.q_ref = refs(out);
      const auto qdd = ctrl.LastReferenceAcceleration();
      const auto tm = ctrl.LastInertiaTerm();
      m.qdd.assign(qdd.begin(), qdd.end());
      m.tau_m.assign(tm.begin(), tm.end());
      const auto& tick = ctrl.LastTick();
      for (std::size_t i = 0; i < n; ++i) {
        m.tau_ff.push_back(tick.tau_ff[i]);
        m.g.push_back(tick.g[i]);
        m.log_qdd.push_back(tick.qdd_ref[i]);
        m.log_tau_inertia.push_back(tick.tau_inertia[i]);
      }
      m.log_inertia_gain = tick.inertia_gain;
    } else if (k == ticks_in + 1) {
      m.q_ref_next = refs(out);
    }
  }
  return m;
}

/// q̈_ref is the acceleration of the reference on the wire: its second
/// difference over the three ticks (O(dt²) truncation, ~1e-4 relative here).
void ExpectAccelerationIsTheReferences(const MidMotion& m) {
  double scale = 0.0;
  for (const double a : m.qdd) {
    scale = std::max(scale, std::abs(a));
  }
  ASSERT_GT(scale, 1.0) << "rad/s² — the judged tick is not inside the motion";
  for (std::size_t i = 0; i < m.qdd.size(); ++i) {
    const double fd = (m.q_ref_next[i] - 2.0 * m.q_ref[i] + m.q_ref_prev[i]) / (kDt * kDt);
    EXPECT_NEAR(m.qdd[i], fd, 1e-3 * scale) << i;
  }
}

// ── Configure ───────────────────────────────────────────────────────────────

TEST(HandJointConfigure, ClosedChainRigWires) {
  const Rig rig = ClosedRig();
  auto ctrl = BringUp(rig);
  EXPECT_EQ(ctrl->ConfigError(), "") << ctrl->ConfigError();
  EXPECT_TRUE(ctrl->ClosedChainActive());
  EXPECT_EQ(ctrl->Dof(), 10);
  EXPECT_EQ(ctrl->GetCommandType(), rtc::CommandType::kPdFeedforward);
  // The bound is min(YAML 10, device 3) on every joint.
  for (const double b : ctrl->TorqueBounds()) {
    EXPECT_DOUBLE_EQ(b, 3.0);
  }
}

TEST(HandJointConfigure, PlainTreeRigWires) {
  const Rig rig = PlainRig();
  auto ctrl = BringUp(rig);
  EXPECT_EQ(ctrl->ConfigError(), "") << ctrl->ConfigError();
  EXPECT_FALSE(ctrl->ClosedChainActive());
  EXPECT_EQ(ctrl->Dof(), 10);
}

TEST(HandJointConfigure, AMissingIndependentJointIsRefused) {
  Rig rig = ClosedRig();
  std::vector<std::string> short_roster = kClosedJoints;
  short_roster.pop_back();  // ring_mcp_fe_joint drives a loop: the projection cannot be fed
  rig.devices[kGroup] = Device(short_roster, 3.0);
  auto ctrl = BringUp(rig);
  EXPECT_NE(ctrl->ConfigError().find("ring_mcp_fe_joint"), std::string::npos)
      << ctrl->ConfigError();
}

TEST(HandJointConfigure, APlainDeviceThatDoesNotCoverTheTreeIsRefused) {
  Rig rig = PlainRig();
  std::vector<std::string> short_roster = kPlainJoints;
  short_roster.pop_back();
  rig.devices[kGroup] = Device(short_roster, 3.0);
  auto ctrl = BringUp(rig);
  EXPECT_FALSE(ctrl->ConfigError().empty());
}

TEST(HandJointConfigure, TheYamlRefusesWhatItCannotRun) {
  const Rig rig = ClosedRig();
  auto make = [&](const std::string& yaml) {
    auto ctrl = std::make_unique<HandJointController>("");
    ctrl->SetSystemModelConfig(rig.model);
    ctrl->SetSharedModelBuilder(rig.builder);
    ctrl->SetControlRate(1.0 / kDt);
    ctrl->LoadConfig(YAML::Load(yaml));
  };
  EXPECT_THROW(make(Yaml(10.0, true, 1.0, "position")), std::runtime_error);
  // The inertia gain is bounded like the gravity gain.
  EXPECT_THROW(make(Yaml(10.0, true, 1.0, "pd_feedforward", 2.5)), std::runtime_error);
  EXPECT_NO_THROW(make(Yaml(10.0, true, 1.0, "pd_feedforward", 2.0)));
  EXPECT_THROW(make(Yaml(10.0, true, 1.0, "torque")), std::runtime_error);
  // No max_abs: the bound is required.
  std::string no_bound = Yaml();
  no_bound.erase(no_bound.find("  max_abs:"), std::string("  max_abs: 10.000000\n").size());
  EXPECT_THROW(make(no_bound), std::runtime_error);
  // Two groups: this controller speaks for one.
  std::string two =
      Yaml() +
      "  other:\n    subscribe:\n      - topic: \"other/joint_goal\"\n        role: \"target\"\n";
  EXPECT_THROW(make(two), std::runtime_error);
  EXPECT_NO_THROW(make(Yaml()));
}

// The block and its gravity key were renamed (gravity_ff.gain →
// dynamics_feedforward.gravity_gain). A YAML left on either old spelling must
// stop at configure: the old block is a missing block, and the old key would
// otherwise leave k_g at its seed without a word.
TEST(HandJointConfigure, StaleFeedforwardSpellingsAreRefused) {
  const std::string good = Yaml();
  ASSERT_NO_THROW((void)ParseHandJointConfig(YAML::Load(good)));
  auto replaced = [&](const std::string& from, const std::string& to) {
    std::string y = good;
    const auto at = y.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    return at == std::string::npos ? y : y.replace(at, from.size(), to);
  };
  auto message = [](const std::string& yaml) {
    try {
      (void)ParseHandJointConfig(YAML::Load(yaml));
    } catch (const std::runtime_error& e) {
      return std::string(e.what());
    }
    return std::string("no throw");
  };
  const std::string old_block = message(replaced("dynamics_feedforward:", "gravity_ff:"));
  EXPECT_NE(old_block.find("dynamics_feedforward"), std::string::npos) << old_block;
  const std::string old_key = message(replaced("  gravity_gain:", "  gain:"));
  EXPECT_NE(old_key.find("dynamics_feedforward.gain is not a key"), std::string::npos) << old_key;
}

TEST(HandJointConfigure, TaskGoalsAreNotAccepted) {
  const Rig rig = ClosedRig();
  auto ctrl = BringUp(rig);
  EXPECT_TRUE(ctrl->AcceptedGoalKinds(0).joint);
  EXPECT_FALSE(ctrl->AcceptedGoalKinds(0).task);
}

// ── The feedforward against the physics ─────────────────────────────────────

TEST(HandJointFeedforward, ClosedChainTorqueIsThePotentialEnergyGradient) {
  const Rig rig = ClosedRig();
  auto ctrl = BringUp(rig);
  ASSERT_EQ(ctrl->ConfigError(), "");
  const ControllerOutput out = TickUntilFresh(*ctrl, rig.q);
  const auto& tick = ctrl->LastTick();
  ASSERT_TRUE(tick.model_fresh && !tick.held && !tick.singular)
      << "held_ticks=" << tick.held_ticks << " closure_error=" << tick.closure_error;
  ASSERT_TRUE(tick.ff_finite);
  EXPECT_EQ(out.command_type, rtc::CommandType::kPdFeedforward);

  std::map<std::string, double> q_by_name;
  for (std::size_t i = 0; i < rig.joints.size(); ++i) {
    q_by_name[rig.joints[i]] = rig.q[i];
  }
  const auto grad = PotentialGradient(rig, q_by_name);
  double largest = 0.0;
  for (std::size_t i = 0; i < rig.joints.size(); ++i) {
    const double want = grad.at(rig.joints[i]);
    EXPECT_NEAR(out.devices[0].feedforward[i], want, kTauTol) << rig.joints[i];
    EXPECT_NEAR(tick.tau_ff[i], want, kTauTol) << rig.joints[i];
    EXPECT_DOUBLE_EQ(tick.g[i], tick.tau_ff[i]) << "gain 1, no clamp: g == tau";
    largest = std::max(largest, std::abs(want));
  }
  // Non-vacuity: the pose actually loads the hand.
  EXPECT_GT(largest, 10.0 * kTauTol);
}

TEST(HandJointFeedforward, TheLoopsChangeTheTorque) {
  // The tree-only gravity (loop-passive joints at the projected pose, no
  // constraint forces) is NOT the reduced g_a on the joints that drive a
  // loop: if it were, the closed-chain path would be a detour. Measured on
  // the oracle side alone so the claim does not depend on the controller.
  const Rig rig = ClosedRig();
  const auto model = rig.builder->GetFullModel();
  rub::ClosedChainHandle oracle(model, rig.builder->GetConstraintModels(),
                                rig.builder->GetClosureActuatedJointIds(), {});
  const std::vector<std::string> indep = oracle.GetIndependentJointNames();
  std::vector<double> q_a(indep.size(), kClosedQ);
  ASSERT_TRUE(oracle.Update(q_a).converged);
  pinocchio::Data data(*model);
  const Eigen::VectorXd g_full =
      pinocchio::computeGeneralizedGravity(*model, data, oracle.GetFullConfiguration());
  std::map<std::string, double> q_by_name;
  for (std::size_t k = 0; k < indep.size(); ++k) {
    q_by_name[indep[k]] = q_a[k];
  }
  const auto grad = PotentialGradient(rig, q_by_name);
  double max_gap = 0.0;
  for (const auto& name : indep) {
    const auto jid = model->getJointId(name);
    const double tree_term = g_full[static_cast<Eigen::Index>(model->joints[jid].idx_v())];
    max_gap = std::max(max_gap, std::abs(tree_term - grad.at(name)));
  }
  EXPECT_GT(max_gap, 10.0 * kTauTol) << "the loops carry no weight on this fixture";
}

TEST(HandJointFeedforward, PlainTreeTorqueIsPinocchioGravityInDeviceOrder) {
  const Rig rig = PlainRig();
  auto ctrl = BringUp(rig);
  ASSERT_EQ(ctrl->ConfigError(), "");
  const ControllerOutput out = TickUntilFresh(*ctrl, rig.q, 5);
  ASSERT_TRUE(ctrl->LastTick().model_fresh);

  const auto model = rig.builder->GetTreeModel(kGroup);
  Eigen::VectorXd q = pinocchio::neutral(*model);
  for (std::size_t i = 0; i < rig.joints.size(); ++i) {
    q[static_cast<Eigen::Index>(model->joints[model->getJointId(rig.joints[i])].idx_q())] =
        rig.q[i];
  }
  pinocchio::Data data(*model);
  const Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(*model, data, q);
  for (std::size_t i = 0; i < rig.joints.size(); ++i) {
    const auto vi =
        static_cast<Eigen::Index>(model->joints[model->getJointId(rig.joints[i])].idx_v());
    EXPECT_NEAR(out.devices[0].feedforward[i], g[vi], 1e-9) << rig.joints[i];
  }
  // Fingertip poses: valid and root-relative on the plain path too.
  for (std::size_t f = 0; f < kPlainTips.size(); ++f) {
    EXPECT_TRUE(out.task_link_pose_valid[f]) << f;
  }
}

// ── Inertia term ────────────────────────────────────────────────────────────

TEST(HandJointInertia, ClosedChainTermIsTheReducedInertiaTimesTheReferenceAcceleration) {
  const Rig rig = ClosedRig(Yaml(10.0, true, 1.0, "pd_feedforward", 1.0));
  auto ctrl = BringUp(rig);
  ASSERT_EQ(ctrl->ConfigError(), "");
  ASSERT_DOUBLE_EQ(ctrl->GetGains().inertia_gain, 1.0) << "the YAML key reaches the tick";
  (void)TickUntilFresh(*ctrl, rig.q);
  // Let the projection settle (it refines by a Newton step per tick).
  for (std::uint64_t t = 3000; t < 3100; ++t) {
    (void)ctrl->Compute(MakeState(rig.q, true, t));
  }
  // At rest the term is 0 and the torque is gravity alone.
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(ctrl->LastInertiaTerm()[ui], 0.0) << ui;
    EXPECT_DOUBLE_EQ(ctrl->LastTick().tau_ff[ui], ctrl->LastTick().g[ui]) << ui;
  }
  // The measurement stays at rig.q, so M is the one at rig.q throughout.
  const MidMotion m = RunIntoAGoal(*ctrl, rig.q, 3100);
  ExpectAccelerationIsTheReferences(m);
  std::map<std::string, double> q_by_name;
  std::map<std::string, double> a_by_name;
  for (std::size_t i = 0; i < rig.joints.size(); ++i) {
    q_by_name[rig.joints[i]] = rig.q[i];
    a_by_name[rig.joints[i]] = m.qdd[i];
  }
  const auto want = ReducedInertiaTimes(rig, q_by_name, a_by_name);
  double scale = 0.0;
  for (const auto& [name, v] : want) {
    scale = std::max(scale, std::abs(v));
  }
  ASSERT_GT(scale, 1e-6) << "N·m — the fixture's inertia term is too small to judge";
  for (std::size_t i = 0; i < rig.joints.size(); ++i) {
    EXPECT_NEAR(m.tau_m[i], want.at(rig.joints[i]), 1e-3 * scale) << rig.joints[i];
    // Gains 1, no clamp: the wire is the sum of the two terms.
    EXPECT_DOUBLE_EQ(m.tau_ff[i], m.g[i] + m.tau_m[i]) << rig.joints[i];
    // And the tick record says so: tau_ff = k_g · g + tau_inertia off one row.
    EXPECT_DOUBLE_EQ(m.log_qdd[i], m.qdd[i]) << rig.joints[i];
    EXPECT_DOUBLE_EQ(m.log_tau_inertia[i], m.tau_m[i]) << rig.joints[i];
  }
  EXPECT_DOUBLE_EQ(m.log_inertia_gain, 1.0);
}

TEST(HandJointInertia, PlainTreeTermIsPinocchioInertiaInDeviceOrder) {
  const Rig rig = PlainRig(Yaml(10.0, true, 1.0, "pd_feedforward", 1.0));
  auto ctrl = BringUp(rig);
  ASSERT_EQ(ctrl->ConfigError(), "");
  (void)TickUntilFresh(*ctrl, rig.q, 5);
  const MidMotion m = RunIntoAGoal(*ctrl, rig.q, 100);
  ExpectAccelerationIsTheReferences(m);
  const auto model = rig.builder->GetTreeModel(kGroup);
  Eigen::VectorXd q = pinocchio::neutral(*model);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model->nv);
  for (std::size_t i = 0; i < rig.joints.size(); ++i) {
    const auto& joint = model->joints[model->getJointId(rig.joints[i])];
    q[static_cast<Eigen::Index>(joint.idx_q())] = rig.q[i];
    a[static_cast<Eigen::Index>(joint.idx_v())] = m.qdd[i];
  }
  pinocchio::Data data(*model);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(model->nv);
  const Eigen::VectorXd ma = pinocchio::rnea(*model, data, q, zero, a);
  const Eigen::VectorXd g = pinocchio::rnea(*model, data, q, zero, zero);
  double scale = 0.0;
  for (std::size_t i = 0; i < rig.joints.size(); ++i) {
    const auto vi =
        static_cast<Eigen::Index>(model->joints[model->getJointId(rig.joints[i])].idx_v());
    const double want = ma[vi] - g[vi];
    scale = std::max(scale, std::abs(want));
    EXPECT_NEAR(m.tau_m[i], want, 1e-9) << rig.joints[i];
    EXPECT_DOUBLE_EQ(m.tau_ff[i], m.g[i] + m.tau_m[i]) << rig.joints[i];
  }
  EXPECT_GT(scale, 1e-6) << "N·m — the fixture's inertia term is too small to judge";
}

TEST(HandJointInertia, GainZeroIsGravityAloneThroughAMotion) {
  // The shipped default: during a goal the wire is k_g · g, bit for bit, and
  // the term reports 0 — the lane is the gravity-only one it was before.
  for (const bool closed : {true, false}) {
    const Rig rig = closed ? ClosedRig() : PlainRig();
    auto ctrl = BringUp(rig);
    ASSERT_DOUBLE_EQ(ctrl->GetGains().inertia_gain, 0.0) << "absent key → 0";
    (void)TickUntilFresh(*ctrl, rig.q);
    const MidMotion m = RunIntoAGoal(*ctrl, rig.q, 3000);
    ASSERT_FALSE(m.tau_ff.empty());
    EXPECT_DOUBLE_EQ(m.log_inertia_gain, 0.0) << closed;
    for (std::size_t i = 0; i < m.tau_ff.size(); ++i) {
      EXPECT_DOUBLE_EQ(m.tau_m[i], 0.0) << closed << " " << i;
      EXPECT_DOUBLE_EQ(m.tau_ff[i], m.g[i]) << closed << " " << i;
      EXPECT_DOUBLE_EQ(m.log_tau_inertia[i], 0.0) << closed << " " << i;
    }
  }
}

TEST(HandJointInertia, AnUnreadableTickFreezesTheReferenceAtRest) {
  // Mid-motion, the device goes unreadable: the reference is frozen, and a
  // frozen reference is at rest — q̈_ref and the inertia term are 0 for the
  // whole outage, the held gravity term stays. When the device is back, the
  // trajectory resumes and so does the term.
  for (const bool closed : {true, false}) {
    const std::string yaml = Yaml(10.0, true, 1.0, "pd_feedforward", 1.0);
    const Rig rig = closed ? ClosedRig(yaml) : PlainRig(yaml);
    auto ctrl = BringUp(rig);
    (void)TickUntilFresh(*ctrl, rig.q);
    (void)RunIntoAGoal(*ctrl, rig.q, 3000);
    for (std::uint64_t t = 3100; t < 3110; ++t) {
      const ControllerOutput frozen = ctrl->Compute(MakeState(rig.q, false, t));
      ASSERT_FALSE(ctrl->LastTick().readable);
      ASSERT_TRUE(ctrl->LastTick().traj_active) << "the trajectory is paused, not dropped";
      for (int i = 0; i < ctrl->Dof(); ++i) {
        const auto ui = static_cast<std::size_t>(i);
        EXPECT_DOUBLE_EQ(ctrl->LastReferenceAcceleration()[ui], 0.0) << closed << " " << ui;
        EXPECT_DOUBLE_EQ(ctrl->LastInertiaTerm()[ui], 0.0) << closed << " " << ui;
        // Gain 1, no clamp, no inertia: the wire is the held gravity term.
        EXPECT_DOUBLE_EQ(frozen.devices[0].feedforward[ui], ctrl->LastTick().g[ui])
            << closed << " " << ui;
      }
    }
    (void)ctrl->Compute(MakeState(rig.q, true, 3110));
    ASSERT_TRUE(ctrl->LastTick().traj_active);
    double back = 0.0;
    for (int i = 0; i < ctrl->Dof(); ++i) {
      back = std::max(back, std::abs(ctrl->LastInertiaTerm()[static_cast<std::size_t>(i)]));
    }
    EXPECT_GT(back, 0.0) << closed;
  }
}

TEST(HandJointInertia, HoldPolicy) {
  const Rig rig = ClosedRig(Yaml(10.0, true, 1.0, "pd_feedforward", 1.0));
  auto ctrl = BringUp(rig);
  (void)TickUntilFresh(*ctrl, rig.q);
  (void)RunIntoAGoal(*ctrl, rig.q, 3000);
  // A non-finite measurement mid-motion: the projection holds, the model terms
  // keep their last valid values, and q̈_ref is still the running reference's.
  std::vector<double> bad = rig.q;
  bad[3] = std::numeric_limits<double>::quiet_NaN();
  (void)ctrl->Compute(MakeState(bad, true, 3100));
  ASSERT_TRUE(ctrl->LastTick().held);
  ASSERT_TRUE(ctrl->LastTick().traj_active);
  double through_hold = 0.0;
  for (int i = 0; i < ctrl->Dof(); ++i) {
    through_hold =
        std::max(through_hold, std::abs(ctrl->LastInertiaTerm()[static_cast<std::size_t>(i)]));
  }
  EXPECT_GT(through_hold, 0.0) << "the term follows the reference through a held projection";
  // damping_feedforward: no trajectory, so no inertia term — gravity alone.
  SetMode(*ctrl, HandJointMode::kDampingFeedforward);
  (void)TickUntilFresh(*ctrl, rig.q);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(ctrl->LastInertiaTerm()[ui], 0.0) << ui;
    EXPECT_DOUBLE_EQ(ctrl->LastReferenceAcceleration()[ui], 0.0) << ui;
    EXPECT_DOUBLE_EQ(ctrl->LastTick().tau_ff[ui], ctrl->LastTick().g[ui]) << ui;
  }
  // Back to tracking (re-seeded at rest), into a goal, then E-STOP: every term 0.
  SetMode(*ctrl, HandJointMode::kTracking);
  (void)ctrl->Compute(MakeState(rig.q, true, 3103));
  (void)RunIntoAGoal(*ctrl, rig.q, 3200);
  ctrl->TriggerEstop();
  const ControllerOutput stop = ctrl->Compute(MakeState(rig.q, true, 3300));
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(stop.devices[0].feedforward[ui], 0.0) << ui;
    EXPECT_DOUBLE_EQ(ctrl->LastInertiaTerm()[ui], 0.0) << ui;
  }
}

TEST(HandJointInertia, TheGainTakesEffectOnTheTickItIsSet) {
  // Mid-motion, the gain goes 0 → 1: the plain path computes M on that very
  // tick (EvaluateModel and the law read the same tick's gains), so the term
  // is there at once — and 1 → 0 removes it at once.
  for (const bool closed : {true, false}) {
    const Rig rig = closed ? ClosedRig() : PlainRig();
    auto ctrl = BringUp(rig);
    (void)TickUntilFresh(*ctrl, rig.q);
    (void)RunIntoAGoal(*ctrl, rig.q, 3000);
    auto gains = ctrl->GetGains();
    gains.inertia_gain = 1.0;
    ctrl->SetGains(gains);
    (void)ctrl->Compute(MakeState(rig.q, true, 3100));
    ASSERT_TRUE(ctrl->LastTick().traj_active);
    double on = 0.0;
    for (int i = 0; i < ctrl->Dof(); ++i) {
      on = std::max(on, std::abs(ctrl->LastInertiaTerm()[static_cast<std::size_t>(i)]));
    }
    EXPECT_GT(on, 0.0) << closed;
    gains.inertia_gain = 0.0;
    ctrl->SetGains(gains);
    (void)ctrl->Compute(MakeState(rig.q, true, 3101));
    for (int i = 0; i < ctrl->Dof(); ++i) {
      const auto ui = static_cast<std::size_t>(i);
      EXPECT_DOUBLE_EQ(ctrl->LastInertiaTerm()[ui], 0.0) << closed << " " << ui;
      EXPECT_DOUBLE_EQ(ctrl->LastTick().tau_ff[ui], ctrl->LastTick().g[ui]) << closed << " " << ui;
    }
  }
}

// ── Hold policy ─────────────────────────────────────────────────────────────

TEST(HandJointHold, ZeroBeforeTheFirstValidTermThenTheLastValueThroughAHeldTick) {
  const Rig rig = ClosedRig();
  auto ctrl = BringUp(rig);
  // Tick 1: the projection walks in from its seed (held), so the term is not
  // yet valid — the torque is 0 and says so.
  ControllerOutput out = ctrl->Compute(MakeState(rig.q, true, 1));
  EXPECT_TRUE(ctrl->LastTick().held) << "the fixture's first tick is expected to walk in";
  EXPECT_FALSE(ctrl->LastTick().ff_finite);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    EXPECT_DOUBLE_EQ(out.devices[0].feedforward[static_cast<std::size_t>(i)], 0.0);
  }
  // Converge.
  out = TickUntilFresh(*ctrl, rig.q);
  ASSERT_TRUE(ctrl->LastTick().model_fresh);
  std::vector<double> held_tau(static_cast<std::size_t>(ctrl->Dof()));
  for (int i = 0; i < ctrl->Dof(); ++i) {
    held_tau[static_cast<std::size_t>(i)] = out.devices[0].feedforward[static_cast<std::size_t>(i)];
  }
  // An unreadable tick: the last torque and the last reference stay.
  const ControllerOutput held = ctrl->Compute(MakeState(rig.q, /*valid=*/false, 900));
  EXPECT_FALSE(ctrl->LastTick().readable);
  EXPECT_FALSE(ctrl->LastTick().model_fresh);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(held.devices[0].feedforward[ui], held_tau[ui]);
    EXPECT_DOUBLE_EQ(held.devices[0].commands[ui], out.devices[0].commands[ui]);
  }
  // A non-finite measurement: the projection holds, the torque stays.
  std::vector<double> bad = rig.q;
  bad[3] = std::numeric_limits<double>::quiet_NaN();
  const ControllerOutput nan_tick = ctrl->Compute(MakeState(bad, true, 901));
  EXPECT_TRUE(ctrl->LastTick().held);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(nan_tick.devices[0].feedforward[ui], held_tau[ui]);
  }
}

TEST(HandJointHold, TheEnableSwitchZeroesTheTorqueAndKeepsTheLane) {
  const Rig rig = ClosedRig();
  auto ctrl = BringUp(rig);
  (void)TickUntilFresh(*ctrl, rig.q);
  auto gains = ctrl->GetGains();
  gains.ff_enabled = false;
  ctrl->SetGains(gains);
  const ControllerOutput off = ctrl->Compute(MakeState(rig.q, true, 1000));
  EXPECT_EQ(off.command_type, rtc::CommandType::kPdFeedforward);
  EXPECT_FALSE(ctrl->LastTick().ff_enabled);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    EXPECT_DOUBLE_EQ(off.devices[0].feedforward[static_cast<std::size_t>(i)], 0.0);
  }
  gains.ff_enabled = true;
  gains.gravity_gain = 0.5;
  ctrl->SetGains(gains);
  const ControllerOutput half = ctrl->Compute(MakeState(rig.q, true, 1001));
  const auto& tick = ctrl->LastTick();
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_NEAR(half.devices[0].feedforward[ui], 0.5 * tick.g[ui], 1e-12);
  }
}

TEST(HandJointHold, TheBoundClampsAndIsVisibleInTheRecord) {
  // A bound far below the gravity term: every loaded joint is clamped to it.
  const Rig rig = ClosedRig(Yaml(/*max_abs=*/1e-4));
  auto ctrl = BringUp(rig);
  const ControllerOutput out = TickUntilFresh(*ctrl, rig.q);
  const auto& tick = ctrl->LastTick();
  ASSERT_TRUE(tick.ff_finite);
  EXPECT_NE(tick.clamped_mask, 0u);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_LE(std::abs(out.devices[0].feedforward[ui]), 1e-4 + 1e-15);
    if (std::abs(tick.g[ui]) > 1e-4) {
      EXPECT_TRUE((tick.clamped_mask >> i) & 1u) << ui;
    }
  }
}

TEST(HandJointHold, EstopHoldsTheMeasurementZeroesTheTorqueAndReseedsOnRelease) {
  const Rig rig = ClosedRig();
  auto ctrl = BringUp(rig);
  (void)TickUntilFresh(*ctrl, rig.q);
  // A goal under way, then E-STOP.
  std::vector<double> goal = rig.q;
  for (double& v : goal) {
    v += 0.1;
  }
  ctrl->SetDeviceTarget(0, goal);
  (void)ctrl->Compute(MakeState(rig.q, true, 2000));
  ASSERT_TRUE(ctrl->LastTick().traj_active);
  ctrl->TriggerEstop();
  std::vector<double> moved = rig.q;
  moved[0] += 0.02;
  const ControllerOutput stop = ctrl->Compute(MakeState(moved, true, 2001));
  EXPECT_TRUE(ctrl->LastTick().estop);
  EXPECT_FALSE(ctrl->LastTick().traj_active);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(stop.devices[0].commands[ui], moved[ui]);
    EXPECT_DOUBLE_EQ(stop.devices[0].feedforward[ui], 0.0);
  }
  // A goal sent during the E-STOP is dropped, not applied on release.
  ctrl->SetDeviceTarget(0, goal);
  (void)ctrl->Compute(MakeState(moved, true, 2002));
  ctrl->ClearEstop();
  const ControllerOutput released = ctrl->Compute(MakeState(moved, true, 2003));
  EXPECT_FALSE(ctrl->LastTick().estop);
  EXPECT_FALSE(ctrl->LastTick().traj_active);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(released.devices[0].commands[ui], moved[ui]);
  }
  // The torque comes back on its own once the projection is fresh again.
  (void)TickUntilFresh(*ctrl, moved);
  EXPECT_TRUE(ctrl->LastTick().ff_finite);
}

// ── The reference lane ──────────────────────────────────────────────────────

TEST(HandJointGoals, AGoalIsFollowedToTheClampedTargetAndAShortOneIsRefused) {
  const Rig rig = PlainRig();
  auto ctrl = BringUp(rig);
  (void)TickUntilFresh(*ctrl, rig.q, 5);
  // A short goal: refused and counted.
  const std::vector<double> short_goal(5, 0.1);
  ctrl->SetDeviceTarget(0, short_goal);
  (void)ctrl->Compute(MakeState(rig.q, true, 10));
  EXPECT_EQ(ctrl->GoalRejectCount(), 1u);
  EXPECT_FALSE(ctrl->LastTick().traj_active);
  // A full goal, one entry beyond the device limit (+3): reached, clamped.
  std::vector<double> goal = rig.q;
  for (double& v : goal) {
    v += 0.5;
  }
  goal[2] = 5.0;
  ctrl->SetDeviceTarget(0, goal);
  ControllerOutput out{};
  for (std::uint64_t t = 11; t < 11 + 5000; ++t) {
    out = ctrl->Compute(MakeState(rig.q, true, t));
    if (!ctrl->LastTick().traj_active && t > 12) {
      break;
    }
  }
  EXPECT_FALSE(ctrl->LastTick().traj_active);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double want = std::min(goal[ui], 3.0);
    EXPECT_DOUBLE_EQ(out.devices[0].commands[ui], want) << ui;
    EXPECT_DOUBLE_EQ(out.devices[0].goal_positions[ui], want) << ui;
  }
  // The reference moved at most max_traj_velocity (4 rad/s) × dt per tick is
  // implied by the duration rule; here: the peak step over the run.
}

TEST(HandJointGoals, TheReferenceSpeedIsBoundedByTheCap) {
  const Rig rig = PlainRig();
  auto ctrl = BringUp(rig);
  (void)TickUntilFresh(*ctrl, rig.q, 5);
  // A runtime speed set above the cap: the tick clamps it to the cap (the
  // parameter callback refuses it too, but SetGains is the raw path).
  auto gains = ctrl->GetGains();
  gains.trajectory_speed = 50.0;
  ctrl->SetGains(gains);
  std::vector<double> goal = rig.q;
  goal[0] += 2.0;  // 2 rad; peak 15/8 × 2 / T must stay ≤ 4 (the rig's device rating)
  ctrl->SetDeviceTarget(0, goal);
  double prev = rig.q[0];
  double max_step = 0.0;
  for (std::uint64_t t = 1; t < 2000; ++t) {
    const ControllerOutput out = ctrl->Compute(MakeState(rig.q, true, t));
    max_step = std::max(max_step, std::abs(out.devices[0].commands[0] - prev));
    prev = out.devices[0].commands[0];
    if (!ctrl->LastTick().traj_active && t > 2) {
      break;
    }
  }
  EXPECT_LE(max_step / kDt, 4.0 + 1e-6);
  EXPECT_GT(max_step / kDt, 1.0);  // and it did move
}

TEST(HandJointGoals, APerJointDeviceRatingBelowTheYamlCapBoundsThatJoint) {
  Rig rig = PlainRig();
  // Joint 0 rated at 0.5 rad/s, the others at the rig's 4.
  rig.devices[kGroup].joint_limits->max_velocity[0] = 0.5;
  auto ctrl = BringUp(rig);
  (void)TickUntilFresh(*ctrl, rig.q, 5);
  std::vector<double> goal = rig.q;
  goal[0] += 1.0;
  goal[1] += 1.0;
  ctrl->SetDeviceTarget(0, goal);
  double prev0 = rig.q[0];
  double prev1 = rig.q[1];
  double step0 = 0.0;
  double step1 = 0.0;
  for (std::uint64_t t = 1; t < 5000; ++t) {
    const ControllerOutput out = ctrl->Compute(MakeState(rig.q, true, t));
    step0 = std::max(step0, std::abs(out.devices[0].commands[0] - prev0));
    step1 = std::max(step1, std::abs(out.devices[0].commands[1] - prev1));
    prev0 = out.devices[0].commands[0];
    prev1 = out.devices[0].commands[1];
    if (!ctrl->LastTick().traj_active && t > 2) {
      break;
    }
  }
  // One duration for every joint: the slow joint's cap stretches the whole
  // trajectory, so joint 1 (same distance) moves at the same bounded speed.
  EXPECT_LE(step0 / kDt, 0.5 + 1e-6);
  EXPECT_LE(step1 / kDt, 0.5 + 1e-6);
  EXPECT_GT(step0 / kDt, 0.1);
}

TEST(HandJointGoals, ARetargetMidMotionRestartsFromRestWithinTheCap) {
  const Rig rig = PlainRig();
  auto ctrl = BringUp(rig);
  (void)TickUntilFresh(*ctrl, rig.q, 5);
  std::vector<double> goal = rig.q;
  goal[0] += 2.0;
  ctrl->SetDeviceTarget(0, goal);
  // Run to roughly the peak of the first quintic, then retarget close by.
  double prev = rig.q[0];
  for (std::uint64_t t = 1; t < 260; ++t) {
    prev = ctrl->Compute(MakeState(rig.q, true, t)).devices[0].commands[0];
  }
  std::vector<double> near = rig.q;
  near[0] = prev + 0.01;
  ctrl->SetDeviceTarget(0, near);
  double max_step = 0.0;
  for (std::uint64_t t = 260; t < 3000; ++t) {
    const ControllerOutput out = ctrl->Compute(MakeState(rig.q, true, t));
    max_step = std::max(max_step, std::abs(out.devices[0].commands[0] - prev));
    prev = out.devices[0].commands[0];
    if (!ctrl->LastTick().traj_active && t > 262) {
      break;
    }
  }
  EXPECT_LE(max_step / kDt, 4.0 + 1e-6);
  EXPECT_NEAR(prev, near[0], 1e-9);
}

// ── Modes ───────────────────────────────────────────────────────────────────

TEST(HandJointModes, DampingCommandsTheMeasurementZeroesTheTorqueAndRefusesGoals) {
  const Rig rig = ClosedRig();
  auto ctrl = BringUp(rig);
  (void)TickUntilFresh(*ctrl, rig.q);
  ASSERT_TRUE(ctrl->LastTick().ff_finite);
  SetMode(*ctrl, HandJointMode::kDamping);
  // The hand is moved by hand: the command follows, nothing pulls it back.
  std::vector<double> moved = rig.q;
  moved[1] += 0.05;
  moved[7] -= 0.03;
  const ControllerOutput out = ctrl->Compute(MakeState(moved, true, 3000));
  EXPECT_TRUE(out.valid);
  EXPECT_EQ(out.command_type, rtc::CommandType::kPdFeedforward);
  EXPECT_EQ(ctrl->LastTick().mode, 1u);
  EXPECT_FALSE(ctrl->LastTick().estop);
  EXPECT_FALSE(ctrl->LastTick().traj_active);
  EXPECT_FALSE(ctrl->LastTick().ff_finite);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(out.devices[0].commands[ui], moved[ui]) << ui;
    EXPECT_DOUBLE_EQ(out.devices[0].trajectory_velocities[ui], 0.0) << ui;
    EXPECT_DOUBLE_EQ(out.devices[0].goal_positions[ui], moved[ui]) << ui;
    EXPECT_DOUBLE_EQ(out.devices[0].feedforward[ui], 0.0) << ui;
  }
  // A goal in this mode is drained and refused (counted) — not E-STOP's
  // silent discard — and the command stays the measurement.
  const auto rejects = ctrl->GoalRejectCount();
  ctrl->SetDeviceTarget(0, rig.q);
  const ControllerOutput next = ctrl->Compute(MakeState(moved, true, 3001));
  EXPECT_EQ(ctrl->GoalRejectCount(), rejects + 1);
  EXPECT_FALSE(ctrl->LastTick().traj_active);
  EXPECT_DOUBLE_EQ(next.devices[0].commands[1], moved[1]);
}

TEST(HandJointModes, DampingFeedforwardCommandsTheMeasurementAndKeepsTrackingsTorque) {
  const Rig rig = ClosedRig();
  auto ctrl = BringUp(rig);
  (void)TickUntilFresh(*ctrl, rig.q);
  // Let the projection settle past its first "fresh" tick (it keeps refining
  // by a Newton step per tick for a while) so the two outputs compared below
  // differ by the mode alone.
  ControllerOutput tracking{};
  for (std::uint64_t t = 3000; t < 3100; ++t) {
    tracking = ctrl->Compute(MakeState(rig.q, true, t));
  }
  ASSERT_TRUE(ctrl->LastTick().ff_finite);
  SetMode(*ctrl, HandJointMode::kDampingFeedforward);
  const ControllerOutput ff = ctrl->Compute(MakeState(rig.q, true, 3100));
  EXPECT_TRUE(ff.valid);
  EXPECT_EQ(ctrl->LastTick().mode, 2u);
  EXPECT_TRUE(ctrl->LastTick().ff_finite);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(ff.devices[0].commands[ui], rig.q[ui]) << ui;
    // The same law on the same tick's term (gain 1, no clamp): exact. Against
    // the settled tracking tick: the lane, not the mode, decides the torque.
    EXPECT_DOUBLE_EQ(ff.devices[0].feedforward[ui], ctrl->LastTick().g[ui]) << ui;
    EXPECT_NEAR(ff.devices[0].feedforward[ui], tracking.devices[0].feedforward[ui], 1e-9) << ui;
  }
  // The lane's own switch still rules it.
  auto gains = ctrl->GetGains();
  gains.ff_enabled = false;
  ctrl->SetGains(gains);
  const ControllerOutput off = ctrl->Compute(MakeState(rig.q, true, 3101));
  for (int i = 0; i < ctrl->Dof(); ++i) {
    EXPECT_DOUBLE_EQ(off.devices[0].feedforward[static_cast<std::size_t>(i)], 0.0);
  }
  gains.ff_enabled = true;
  ctrl->SetGains(gains);
  // E-STOP dominates the mode: every joint 0.
  ctrl->TriggerEstop();
  const ControllerOutput stop = ctrl->Compute(MakeState(rig.q, true, 3102));
  EXPECT_TRUE(ctrl->LastTick().estop);
  EXPECT_EQ(ctrl->LastTick().mode, 2u);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(stop.devices[0].feedforward[ui], 0.0) << ui;
    EXPECT_DOUBLE_EQ(stop.devices[0].commands[ui], rig.q[ui]) << ui;
  }
  ctrl->ClearEstop();
  (void)TickUntilFresh(*ctrl, rig.q);
  EXPECT_TRUE(ctrl->LastTick().ff_finite) << "the lane comes back after release";
}

TEST(HandJointModes, ActivatedInDampingTheOutputIsValidFromTheFirstReadableTick) {
  // The hold counts as a seed: an output invalid for 0.1 s would be a global
  // E-STOP by the controller manager.
  const Rig rig = PlainRig(Yaml() + "mode: damping\n");
  auto ctrl = BringUp(rig);
  ASSERT_EQ(ctrl->ConfigError(), "");
  EXPECT_EQ(ctrl->GetGains().mode, HandJointMode::kDamping);
  const ControllerOutput first = ctrl->Compute(MakeState(rig.q, true, 1));
  EXPECT_TRUE(first.valid);
  EXPECT_TRUE(ctrl->LastTick().seeded);
  EXPECT_EQ(ctrl->LastTick().mode, 1u);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(first.devices[0].commands[ui], rig.q[ui]) << ui;
    EXPECT_DOUBLE_EQ(first.devices[0].feedforward[ui], 0.0) << ui;
  }
  // An unreadable tick holds the last command, as in tracking.
  const ControllerOutput held = ctrl->Compute(MakeState(rig.q, false, 2));
  EXPECT_TRUE(held.valid);
  EXPECT_DOUBLE_EQ(held.devices[0].commands[0], rig.q[0]);
}

TEST(HandJointModes, ReturningToTrackingReseedsFromTheMeasurementAndAcceptsGoals) {
  const Rig rig = PlainRig();
  auto ctrl = BringUp(rig);
  (void)TickUntilFresh(*ctrl, rig.q, 5);
  // A goal under way, then the mode leaves tracking: the trajectory is dropped.
  std::vector<double> goal = rig.q;
  goal[0] += 1.0;
  ctrl->SetDeviceTarget(0, goal);
  (void)ctrl->Compute(MakeState(rig.q, true, 10));
  ASSERT_TRUE(ctrl->LastTick().traj_active);
  SetMode(*ctrl, HandJointMode::kDamping);
  std::vector<double> moved = rig.q;
  moved[0] += 0.2;
  (void)ctrl->Compute(MakeState(moved, true, 11));
  EXPECT_FALSE(ctrl->LastTick().traj_active);
  // Back to tracking: the first tick re-seeds at the measurement (goal =
  // "here"), nothing moves, and the next goal is accepted.
  SetMode(*ctrl, HandJointMode::kTracking);
  const ControllerOutput back = ctrl->Compute(MakeState(moved, true, 12));
  EXPECT_EQ(ctrl->LastTick().mode, 0u);
  EXPECT_FALSE(ctrl->LastTick().traj_active);
  for (int i = 0; i < ctrl->Dof(); ++i) {
    const auto ui = static_cast<std::size_t>(i);
    EXPECT_DOUBLE_EQ(back.devices[0].commands[ui], moved[ui]) << ui;
    EXPECT_DOUBLE_EQ(back.devices[0].goal_positions[ui], moved[ui]) << ui;
  }
  const auto rejects = ctrl->GoalRejectCount();
  ctrl->SetDeviceTarget(0, goal);
  (void)ctrl->Compute(MakeState(moved, true, 13));
  EXPECT_EQ(ctrl->GoalRejectCount(), rejects);
  EXPECT_TRUE(ctrl->LastTick().traj_active);
}

TEST(HandJointModes, TheModeParsesFromTheYamlAndATypoIsRefused) {
  EXPECT_EQ(ParseHandJointMode("tracking"), HandJointMode::kTracking);
  EXPECT_EQ(ParseHandJointMode("damping"), HandJointMode::kDamping);
  EXPECT_EQ(ParseHandJointMode("damping_feedforward"), HandJointMode::kDampingFeedforward);
  EXPECT_THROW((void)ParseHandJointMode("dampng"), std::runtime_error);
  EXPECT_THROW((void)ParseHandJointMode(""), std::runtime_error);
  for (const auto m :
       {HandJointMode::kTracking, HandJointMode::kDamping, HandJointMode::kDampingFeedforward}) {
    EXPECT_EQ(ParseHandJointMode(HandJointModeName(m)), m);
  }
  // Omitted → tracking; spelled → that mode; a typo throws at load.
  EXPECT_EQ(ParseHandJointConfig(YAML::Load(Yaml())).mode, HandJointMode::kTracking);
  EXPECT_EQ(ParseHandJointConfig(YAML::Load(Yaml() + "mode: damping_feedforward\n")).mode,
            HandJointMode::kDampingFeedforward);
  EXPECT_THROW((void)ParseHandJointConfig(YAML::Load(Yaml() + "mode: hold\n")), std::runtime_error);
}

// ── The shipped p2 / p2_left YAML ───────────────────────────────────────────

// One hand per profile: the right hand's p2 (group p2_right) and the left
// hand's p2_left (group p2_left) ship the same controller values.
class HandJointShipped : public ::testing::TestWithParam<std::pair<const char*, const char*>> {};

TEST_P(HandJointShipped, TheYamlParsesAndNamesOneGroup) {
  const auto [profile, group] = GetParam();
  const std::filesystem::path path = std::filesystem::path(RTC_DEMO_SHARED_CONFIG_DIR) / profile /
                                     "controllers" / "hand_joint_controller.yaml";
  ASSERT_TRUE(std::filesystem::exists(path)) << path;
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node cfg = root["hand_joint_controller"];
  ASSERT_TRUE(cfg);
  const auto parsed = ParseHandJointConfig(cfg);
  EXPECT_EQ(parsed.mode, HandJointMode::kTracking) << "the shipped mode is spelled out";
  EXPECT_TRUE(parsed.ff.enabled);
  EXPECT_DOUBLE_EQ(parsed.ff.gravity_gain, 1.0);
  EXPECT_DOUBLE_EQ(parsed.ff.inertia_gain, 1.0) << "the inertia term ships on, spelled out";
  EXPECT_TRUE(cfg["dynamics_feedforward"]["inertia_gain"]);
  // The bound covers the sum of the gravity and the inertia term.
  EXPECT_DOUBLE_EQ(parsed.ff.max_abs, 0.3);
  // The bound is an explicit choice well under the device's placeholder
  // rating (1.0 N·m in _base.yaml): the YAML, not the rating, bounds the wire.
  EXPECT_LT(parsed.ff.max_abs, 1.0);
  ASSERT_TRUE(cfg["topics"]);
  EXPECT_EQ(cfg["topics"].size(), 1u);
  EXPECT_TRUE(cfg["topics"][group]);
  // The log instances the judge and the plotter look for, and the generic
  // per-device state file the controller requires as <group>_state.
  bool has_joint_ff = false;
  bool has_state = false;
  for (const auto& e : cfg["logs"]) {
    if (e["instance"] && e["instance"].as<std::string>() == "joint_ff") {
      has_joint_ff = true;
    }
    if (e["instance"] && e["instance"].as<std::string>() == std::string(group) + "_state") {
      has_state = true;
    }
  }
  EXPECT_TRUE(has_joint_ff);
  EXPECT_TRUE(has_state);
}

INSTANTIATE_TEST_SUITE_P(Profiles, HandJointShipped,
                         ::testing::Values(std::make_pair("p2", "p2_right"),
                                           std::make_pair("p2_left", "p2_left")));

// ── Lifecycle on a real node ────────────────────────────────────────────────

TEST(HandJointLifecycle, ConfiguresDeclaresParametersAndCleansUp) {
  const Rig rig = ClosedRig();
  auto ctrl = BringUp(rig);
  ASSERT_EQ(ctrl->ConfigError(), "");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("hand_joint_controller");
  const rclcpp_lifecycle::State unconfigured;
  ASSERT_EQ(ctrl->on_configure(unconfigured, node, YAML::Load(rig.yaml)),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(node->has_parameter("dynamics_feedforward.enabled"));
  EXPECT_TRUE(node->has_parameter("dynamics_feedforward.gravity_gain"));
  EXPECT_TRUE(node->has_parameter("dynamics_feedforward.inertia_gain"));
  EXPECT_TRUE(node->has_parameter("trajectory_speed"));
  EXPECT_TRUE(node->has_parameter("mode"));
  EXPECT_EQ(node->get_parameter("mode").as_string(), "tracking");
  // The mode reaches the tick's gains; a typo is refused with the whitelist
  // in the reason and the mode kept.
  const auto damp = node->set_parameter(rclcpp::Parameter("mode", "damping_feedforward"));
  EXPECT_TRUE(damp.successful) << damp.reason;
  EXPECT_EQ(ctrl->GetGains().mode, HandJointMode::kDampingFeedforward);
  const auto typo = node->set_parameter(rclcpp::Parameter("mode", "dampng"));
  EXPECT_FALSE(typo.successful);
  EXPECT_NE(typo.reason.find("damping_feedforward"), std::string::npos) << typo.reason;
  EXPECT_EQ(ctrl->GetGains().mode, HandJointMode::kDampingFeedforward);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("mode", "tracking")).successful);
  EXPECT_EQ(ctrl->GetGains().mode, HandJointMode::kTracking);
  // The runtime switch reaches the tick's gains.
  const auto res = node->set_parameter(rclcpp::Parameter("dynamics_feedforward.enabled", false));
  EXPECT_TRUE(res.successful) << res.reason;
  EXPECT_FALSE(ctrl->GetGains().ff_enabled);
  // An out-of-range gain is refused and the old one kept.
  const auto bad = node->set_parameter(rclcpp::Parameter("dynamics_feedforward.gravity_gain", 5.0));
  EXPECT_FALSE(bad.successful);
  EXPECT_NE(bad.reason.find("gravity_gain"), std::string::npos) << bad.reason;
  EXPECT_DOUBLE_EQ(ctrl->GetGains().gravity_gain, 1.0);
  // The inertia gain: in range reaches the tick, out of range is refused.
  EXPECT_TRUE(
      node->set_parameter(rclcpp::Parameter("dynamics_feedforward.inertia_gain", 0.5)).successful);
  EXPECT_DOUBLE_EQ(ctrl->GetGains().inertia_gain, 0.5);
  const auto bad_m =
      node->set_parameter(rclcpp::Parameter("dynamics_feedforward.inertia_gain", 2.5));
  EXPECT_FALSE(bad_m.successful);
  EXPECT_NE(bad_m.reason.find("inertia_gain"), std::string::npos) << bad_m.reason;
  EXPECT_DOUBLE_EQ(ctrl->GetGains().inertia_gain, 0.5);
  // A speed above the read-only cap (4 rad/s in the rig) is refused too.
  const auto fast = node->set_parameter(rclcpp::Parameter("trajectory_speed", 5.0));
  EXPECT_FALSE(fast.successful);
  EXPECT_DOUBLE_EQ(ctrl->GetGains().trajectory_speed, 2.0);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("trajectory_speed", 3.0)).successful);
  EXPECT_DOUBLE_EQ(ctrl->GetGains().trajectory_speed, 3.0);
  ASSERT_EQ(ctrl->on_activate(unconfigured),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl->on_deactivate(unconfigured),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  ASSERT_EQ(ctrl->on_cleanup(unconfigured),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST(HandJointLifecycle, AWrongTypedInertiaGainOverrideCostsThatParameterAlone) {
  // An integer `1` where a double is declared throws inside declare_parameter.
  // It must not abort the declares after it: `mode` is still declared and
  // settable, and the YAML inertia gain stays in force.
  const Rig rig = ClosedRig();
  auto ctrl = BringUp(rig);
  ASSERT_EQ(ctrl->ConfigError(), "");
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({rclcpp::Parameter("dynamics_feedforward.inertia_gain", 1)});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("hand_joint_controller", opts);
  const rclcpp_lifecycle::State unconfigured;
  ASSERT_EQ(ctrl->on_configure(unconfigured, node, YAML::Load(rig.yaml)),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(node->has_parameter("dynamics_feedforward.inertia_gain"));
  EXPECT_DOUBLE_EQ(ctrl->GetGains().inertia_gain, 0.0);
  ASSERT_TRUE(node->has_parameter("mode"));
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("mode", "damping")).successful);
  EXPECT_EQ(ctrl->GetGains().mode, HandJointMode::kDamping);
  ASSERT_EQ(ctrl->on_cleanup(unconfigured),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST(HandJointLifecycle, AConfigErrorRefusesConfigure) {
  Rig rig = ClosedRig();
  std::vector<std::string> short_roster = kClosedJoints;
  short_roster.pop_back();
  rig.devices[kGroup] = Device(short_roster, 3.0);
  auto ctrl = BringUp(rig);
  ASSERT_FALSE(ctrl->ConfigError().empty());
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("hand_joint_controller_refused");
  const rclcpp_lifecycle::State unconfigured;
  EXPECT_EQ(ctrl->on_configure(unconfigured, node, YAML::Load(rig.yaml)),
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
