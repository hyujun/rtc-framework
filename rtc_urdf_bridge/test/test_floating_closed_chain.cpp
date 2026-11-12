// Floating CM model with a closed-chain hand (W1-F04, WD-37 / WD-38).
//
// The shipped g1_loco CM model (robot_descriptions g1 floating URDF + the p1b surrogate hand,
// five contact_3d loops = 15 rows, 33 actuated + 10 loop-passive joints) is built twice, with a
// fixed and with a free-flyer root, through the builder the controllers use. The fixed build is
// the reference: every closed-chain quantity of the floating build must equal it —
//   - the free-flyer is never a loop-passive lock, and sub / tree models lock it (WD-37),
//   - both closed-chain handles hold it: neither independent nor dependent, its 7 q slots stay
//     at the seed bit for bit, its G rows are 0 (WD-38),
//   - at the identity base the RT handle is bit-identical to the fixed one; at a rotated and
//     translated base the base-invariant quantities agree to roundoff.
// The tree definitions mirror the g1_loco profile's `urdf.tree_models` (integrated_bringup is not
// a dependency of this package). Model files come from the robot_descriptions share directory.
#include "closure_test_fixtures.hpp"
#include "rtc_urdf_bridge/closed_chain_handle.hpp"
#include "rtc_urdf_bridge/free_flyer_root.hpp"
#include "rtc_urdf_bridge/loop_projection.hpp"
#include "rtc_urdf_bridge/loop_verification.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#pragma GCC diagnostic pop

#include <Eigen/SVD>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;

namespace {

constexpr const char* kCmXacro = "robots/g1/urdf/g1_29dof_floating_with_p1b_surrogate.urdf.xacro";
constexpr const char* kCmClosure =
    "robots/g1/urdf/g1_29dof_floating_with_p1b_surrogate.closure.yaml";
constexpr const char* kPlainUrdf = "robots/g1/urdf/g1_29dof_floating.urdf";
constexpr const char* kHandRoot = "base_adapter";
const std::array<std::string, 4> kTips = {"l_thumb_tip_bracket", "l_index_tip_bracket",
                                          "l_middle_tip_bracket", "l_ring_tip_bracket"};

// Pre-registered (#1031 [SPRINT] v3, gate 3).
constexpr unsigned kSeed = 20261112U;
constexpr int kTargets = 200;
constexpr double kHandSpan = 0.3;   // rad, hand actuated joints
constexpr double kBodySpan = 0.05;  // rad, body joints
constexpr double kTol = 1e-12;

std::string Share(const std::string& rel) {
  return (std::filesystem::path(
              ament_index_cpp::get_package_share_directory("robot_descriptions")) /
          rel)
      .string();
}

rub::ModelConfig CmConfig(const std::string& root) {
  rub::ModelConfig cfg;
  cfg.urdf_path = Share(kCmXacro);
  cfg.closure_yaml_path = Share(kCmClosure);
  cfg.root_joint_type = root;
  cfg.tree_models.push_back(
      {"g1",
       "pelvis",
       {"left_ankle_roll_link", "right_ankle_roll_link", "left_wrist_yaw_link", kHandRoot}});
  cfg.tree_models.push_back({"p1b", kHandRoot, {kTips.begin(), kTips.end()}});
  return cfg;
}

const rub::PinocchioModelBuilder& Fixed() {
  static const rub::PinocchioModelBuilder b(CmConfig("fixed"));
  return b;
}

const rub::PinocchioModelBuilder& Floating() {
  static const rub::PinocchioModelBuilder b(CmConfig("floating"));
  return b;
}

std::vector<std::string> JointNames(const pinocchio::Model& m) {
  return {m.names.begin(), m.names.end()};
}

std::vector<std::string> ViolationNames(const std::vector<rub::InertialViolation>& v) {
  std::vector<std::string> out;
  for (const auto& x : v) {
    out.push_back(x.joint_name + "/" + x.body_name);
  }
  return out;
}

// Exact equality of two matrices (== per coefficient, so +0 == -0).
template <typename A, typename B>
bool BitEqual(const A& a, const B& b) {
  return a.rows() == b.rows() && a.cols() == b.cols() && (a.array() == b.array()).all();
}

double MaxAbsDiff(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
  return (a - b).cwiseAbs().maxCoeff();
}

// A fingertip's pose in the hand root's frame — base-invariant.
pinocchio::SE3 TipInHand(const rub::RtClosedChainHandle& h, const std::string& tip) {
  return h.GetFramePlacement(h.GetFrameId(kHandRoot))
      .actInv(h.GetFramePlacement(h.GetFrameId(tip)));
}

rub::RtClosedChainHandle MakeRt(const rub::PinocchioModelBuilder& b, const Eigen::VectorXd& seed,
                                double max_seed_increment = rub::kDefaultActuatedIncrement) {
  return rub::RtClosedChainHandle(b.GetFullModel(), b.GetConstraintModels(),
                                  b.GetClosureActuatedJointIds(), seed, 2, 1e-6, 1e-6,
                                  max_seed_increment);
}

// Independent q of a handle's current configuration, in its q_a order.
std::vector<double> IndependentQ(const rub::RtClosedChainHandle& h) {
  const pinocchio::Model& m = h.GetModel();
  std::vector<double> q;
  for (const auto& name : h.GetIndependentJointNames()) {
    q.push_back(h.GetFullConfiguration()[m.idx_qs[m.getJointId(name)]]);
  }
  return q;
}

// κ(Jc_D) of the fixed model at q (loop-passive columns).
double PassiveCondition(const rub::PinocchioModelBuilder& b, const Eigen::VectorXd& q) {
  const pinocchio::Model& m = *b.GetFullModel();
  pinocchio::Data d(m);
  const rub::ConstraintKinematics kin =
      rub::ComputeConstraintKinematics(m, d, b.GetConstraintModels(), q);
  const std::vector<int> cols =
      rub::LoopPassiveVelocityIndices(m, b.GetConstraintModels(), b.GetClosureActuatedJointIds());
  Eigen::MatrixXd Jd(kin.Jc.rows(), static_cast<Eigen::Index>(cols.size()));
  for (std::size_t c = 0; c < cols.size(); ++c) {
    Jd.col(static_cast<Eigen::Index>(c)) = kin.Jc.col(cols[c]);
  }
  const Eigen::VectorXd sv = Eigen::JacobiSVD<Eigen::MatrixXd>(Jd).singularValues();
  return sv(0) / sv(sv.size() - 1);
}

// A base configuration [p, q_xyzw]: the identity, or a random unit quaternion with |t| ≤ 2 m.
// The random quaternion is nudged (an ulp at a time — it stays unit) until pinocchio's
// first-order renormalisation, q·(3 − ‖q‖²)/2, moves it: SE3 integrate applies that even for a
// zero increment, so only such seeds can tell a held root from one the integrate rewrote
// (‖q‖² one or two ulp below 1 rounds the factor to exactly 1). The tests check it with
// `IntegrateMovesTheRoot`.
Eigen::Matrix<double, 7, 1> BaseSeed(int k) {
  Eigen::Matrix<double, 7, 1> base;
  base << 0, 0, 0, 0, 0, 0, 1;
  if (k == 0) {
    return base;
  }
  std::mt19937_64 rng(kSeed + static_cast<unsigned>(k));
  std::normal_distribution<double> n01(0.0, 1.0);
  std::uniform_real_distribution<double> u(-1.15, 1.15);  // |t| ≤ √3·1.15 < 2
  Eigen::Vector4d quat(n01(rng), n01(rng), n01(rng), n01(rng));
  quat.normalize();
  const auto renormalised = [](const Eigen::Vector4d& v) {
    return Eigen::Vector4d(v * ((3.0 - v.squaredNorm()) / 2.0));
  };
  // Nudge the largest-magnitude component (|·| ≥ 1/2), a bounded number of times — the callers
  // assert the seed moves (`IntegrateMovesTheRoot`), so an exhausted loop is red, not a hang.
  Eigen::Index big = 0;
  quat.cwiseAbs().maxCoeff(&big);
  for (int i = 0; i < 64 && (renormalised(quat).array() == quat.array()).all(); ++i) {
    quat[big] = std::nextafter(quat[big], quat[big] >= 0.0 ? 2.0 : -2.0);
  }
  base << u(rng), u(rng), u(rng), quat;
  return base;
}

// Whether a zero-increment integrate changes the root slots of q (see BaseSeed).
bool IntegrateMovesTheRoot(const pinocchio::Model& m, const Eigen::VectorXd& q) {
  Eigen::VectorXd out(m.nq);
  pinocchio::integrate(m, q, Eigen::VectorXd::Zero(m.nv), out);
  return !BitEqual(out.head<rub::kFreeFlyerNq>(), q.head<rub::kFreeFlyerNq>());
}

}  // namespace

// ── Gate 1: model counts (WD-37) ────────────────────────────────────────────

TEST(FloatingClosedChain, ShippedModelKeepsTheFreeFlyerOutOfTheLoopLocks) {
  const auto& fx = Fixed();
  const auto& fl = Floating();
  const auto& locks = fl.GetClosurePassiveLockNames();
  EXPECT_EQ(locks.size(), 10u);
  EXPECT_EQ(std::count(locks.begin(), locks.end(), "root_joint"), 0);
  EXPECT_EQ(locks, fx.GetClosurePassiveLockNames());

  EXPECT_EQ(fx.GetFullModel()->nq, 43);
  EXPECT_EQ(fl.GetFullModel()->nq, 50);
  EXPECT_EQ(fl.GetFullModel()->nv, 49);
  ASSERT_NE(fl.GetActuatedModel(), nullptr);
  EXPECT_TRUE(rub::IsFreeFlyerRoot(*fl.GetActuatedModel()));
  EXPECT_EQ(fl.GetActuatedModel()->nq, 40);
  EXPECT_EQ(fl.GetActuatedModel()->nv, 39);
  EXPECT_EQ(fx.GetActuatedModel()->nv, 33);
}

TEST(FloatingClosedChain, TreeModelsLockTheFreeFlyerAndEqualTheFixedBuild) {
  for (const char* name : {"g1", "p1b"}) {
    SCOPED_TRACE(name);
    const pinocchio::Model& fx = *Fixed().GetTreeModel(name);
    const pinocchio::Model& fl = *Floating().GetTreeModel(name);
    EXPECT_FALSE(rub::IsFreeFlyerRoot(fl));
    EXPECT_EQ(fl.nq, fx.nq);
    EXPECT_EQ(fl.nv, fx.nv);
    EXPECT_EQ(JointNames(fl), JointNames(fx));
    // Every fixed-build frame, by name, at the same neutral placement.
    pinocchio::Data dx(fx);
    pinocchio::Data dl(fl);
    pinocchio::framesForwardKinematics(fx, dx, pinocchio::neutral(fx));
    pinocchio::framesForwardKinematics(fl, dl, pinocchio::neutral(fl));
    for (std::size_t f = 0; f < fx.frames.size(); ++f) {
      const std::string& fname = fx.frames[f].name;
      ASSERT_TRUE(fl.existFrame(fname)) << fname;
      const pinocchio::SE3& a = dx.oMf[f];
      const pinocchio::SE3& b = dl.oMf[fl.getFrameId(fname)];
      EXPECT_LE((a.translation() - b.translation()).norm(), kTol) << fname;
      EXPECT_LE((a.rotation() - b.rotation()).norm(), kTol) << fname;
    }
  }
  EXPECT_EQ(Floating().GetTreeModel("g1")->nv, 23);
}

TEST(FloatingClosedChain, PlainFloatingUrdfLocksTheFreeFlyerInSubAndTreeModels) {
  const auto build = [](const std::string& root) {
    rub::ModelConfig cfg;
    cfg.urdf_path = Share(kPlainUrdf);
    cfg.root_joint_type = root;
    cfg.tree_models.push_back({"body",
                               "pelvis",
                               {"left_ankle_roll_link", "right_ankle_roll_link",
                                "left_wrist_yaw_link", "right_wrist_yaw_link"}});
    cfg.sub_models.push_back({"left_leg", "pelvis", "left_ankle_roll_link"});
    return std::make_unique<rub::PinocchioModelBuilder>(cfg);
  };
  const auto fx = build("fixed");
  const auto fl = build("floating");
  EXPECT_TRUE(rub::IsFreeFlyerRoot(*fl->GetFullModel()));
  EXPECT_EQ(fl->GetFullModel()->nq, fx->GetFullModel()->nq + rub::kFreeFlyerNq);
  for (const auto& m :
       {std::make_pair(fx->GetTreeModel("body"), fl->GetTreeModel("body")),
        std::make_pair(fx->GetReducedModel("left_leg"), fl->GetReducedModel("left_leg"))}) {
    EXPECT_FALSE(rub::IsFreeFlyerRoot(*m.second));
    EXPECT_EQ(m.second->nq, m.first->nq);
    EXPECT_EQ(JointNames(*m.second), JointNames(*m.first));
  }
}

TEST(FloatingClosedChain, InertialReportEqualsTheFixedBuild) {
  const auto& fx = Fixed().GetInertialReport();
  const auto& fl = Floating().GetInertialReport();
  EXPECT_EQ(ViolationNames(fl.fatal), ViolationNames(fx.fatal));
  EXPECT_EQ(ViolationNames(fl.degenerate), ViolationNames(fx.degenerate));
}

TEST(FloatingClosedChain, ClosureReferenceKeepsTheRootNeutralAndTheFixedJoints) {
  const Eigen::VectorXd& qx = Fixed().GetClosureReferenceConfig();
  const Eigen::VectorXd& ql = Floating().GetClosureReferenceConfig();
  ASSERT_EQ(ql.size(), qx.size() + rub::kFreeFlyerNq);
  const Eigen::VectorXd neutral = pinocchio::neutral(*Floating().GetFullModel());
  EXPECT_TRUE(BitEqual(ql.head<rub::kFreeFlyerNq>(), neutral.head<rub::kFreeFlyerNq>()));
  EXPECT_TRUE(BitEqual(ql.tail(qx.size()), qx));
  EXPECT_TRUE(Floating().IsClosureReferenceAcceptable());
  EXPECT_EQ(Floating().IsClosureReferenceSingular(), Fixed().IsClosureReferenceSingular());
}

// ── Gate 2: handle structure (WD-38) ───────────────────────────────────────

TEST(FloatingClosedChain, BothHandlesBuildOnTheShippedFloatingModel) {
  const auto& fl = Floating();
  const auto& fx = Fixed();
  std::unique_ptr<rub::RtClosedChainHandle> rt;
  ASSERT_NO_THROW(
      rt = std::make_unique<rub::RtClosedChainHandle>(MakeRt(fl, fl.GetClosureReferenceConfig())));
  std::unique_ptr<rub::ClosedChainHandle> nrt;
  ASSERT_NO_THROW(nrt = std::make_unique<rub::ClosedChainHandle>(fl));
  const rub::RtClosedChainHandle rt_fx = MakeRt(fx, fx.GetClosureReferenceConfig());
  EXPECT_EQ(rt->nv_independent(), 33);
  EXPECT_EQ(rt->GetIndependentJointNames(), rt_fx.GetIndependentJointNames());
  EXPECT_EQ(nrt->GetIndependentJointNames(), rt_fx.GetIndependentJointNames());

  // Dependent rows of G: 10 joint rows, every one coupled to the actuation; the root rows are 0.
  ASSERT_FALSE(rt->Update(IndependentQ(*rt)).held);
  const Eigen::MatrixXd G = rt->GetReductionMap();
  EXPECT_TRUE((G.topRows(rub::kFreeFlyerNv).array() == 0.0).all());
  const pinocchio::Model& m = *fl.GetFullModel();
  std::vector<bool> indep(static_cast<std::size_t>(m.nv), false);
  for (const auto& name : rt->GetIndependentJointNames()) {
    indep[static_cast<std::size_t>(m.idx_vs[m.getJointId(name)])] = true;
  }
  int coupled = 0;
  for (int r = rub::kFreeFlyerNv; r < m.nv; ++r) {
    if (!indep[static_cast<std::size_t>(r)] && G.row(r).norm() > 0.0) {
      ++coupled;
    }
  }
  EXPECT_EQ(coupled, 10);
}

TEST(FloatingClosedChain, RemovingSixActuatedJointsRefusesBothRoots) {
  for (const rub::PinocchioModelBuilder* b : {&Fixed(), &Floating()}) {
    SCOPED_TRACE(b->GetConfig().root_joint_type);
    std::vector<pinocchio::JointIndex> ids = b->GetClosureActuatedJointIds();
    ids.erase(ids.begin(), ids.begin() + 6);  // dependent 16 > 15 rows
    EXPECT_THROW(rub::RtClosedChainHandle(b->GetFullModel(), b->GetConstraintModels(), ids,
                                          b->GetClosureReferenceConfig()),
                 std::runtime_error);
    EXPECT_THROW(rub::ClosedChainHandle(b->GetFullModel(), b->GetConstraintModels(), ids,
                                        b->GetClosureReferenceConfig()),
                 std::runtime_error);
  }
}

TEST(FloatingClosedChain, AWorldAnchoredLoopIsRefusedOnlyOnTheFloatingModel) {
  for (const rub::PinocchioModelBuilder* b : {&Fixed(), &Floating()}) {
    const bool floating = rub::IsFreeFlyerRoot(*b->GetFullModel());
    SCOPED_TRACE(floating ? "floating" : "fixed");
    const pinocchio::Model& m = *b->GetFullModel();
    std::vector<pinocchio::RigidConstraintModel> cms = b->GetConstraintModels();
    const auto& c = cms.front();
    cms.emplace_back(pinocchio::CONTACT_3D, m, 0, pinocchio::SE3::Identity(), c.joint2_id,
                     c.joint2_placement, pinocchio::LOCAL);
    ASSERT_TRUE(rub::HasWorldAnchoredConstraint(cms));
    const auto make_rt = [&] {
      return rub::RtClosedChainHandle(b->GetFullModel(), cms, b->GetClosureActuatedJointIds(),
                                      b->GetClosureReferenceConfig());
    };
    const auto make_nrt = [&] {
      return rub::ClosedChainHandle(b->GetFullModel(), cms, b->GetClosureActuatedJointIds(),
                                    b->GetClosureReferenceConfig());
    };
    if (floating) {
      EXPECT_THROW(make_rt(), std::runtime_error);
      EXPECT_THROW(make_nrt(), std::runtime_error);
    } else {
      EXPECT_NO_THROW(make_rt());
      EXPECT_NO_THROW(make_nrt());
    }
    // The projections refuse it too — the held root cannot close it (the builder's q_ref with it).
    pinocchio::Data d(m);
    if (floating) {
      EXPECT_THROW(
          static_cast<void>(rub::ProjectToConstraint(m, d, cms, b->GetClosureReferenceConfig())),
          std::invalid_argument);
    } else {
      EXPECT_NO_THROW(
          static_cast<void>(rub::ProjectToConstraint(m, d, cms, b->GetClosureReferenceConfig())));
    }
  }
}

// The velocity projection holds the root as the position-level ones do: v's root part is kept,
// the joint part satisfies Jc v = 0 as on the fixed model.
TEST(FloatingClosedChain, ProjectVelocityKeepsTheRootVelocity) {
  const pinocchio::Model& ml = *Floating().GetFullModel();
  const pinocchio::Model& mx = *Fixed().GetFullModel();
  pinocchio::Data dl(ml);
  pinocchio::Data dx(mx);
  std::mt19937_64 rng(kSeed);
  Eigen::VectorXd vl(ml.nv);
  for (Eigen::Index i = 0; i < vl.size(); ++i) {
    vl[i] = std::uniform_real_distribution<double>(-1.0, 1.0)(rng);
  }
  const Eigen::VectorXd pl = rub::ProjectVelocity(ml, dl, Floating().GetConstraintModels(),
                                                  Floating().GetClosureReferenceConfig(), vl);
  const Eigen::VectorXd px = rub::ProjectVelocity(
      mx, dx, Fixed().GetConstraintModels(), Fixed().GetClosureReferenceConfig(), vl.tail(mx.nv));
  EXPECT_TRUE(BitEqual(pl.head<rub::kFreeFlyerNv>(), vl.head<rub::kFreeFlyerNv>()));
  EXPECT_LE((pl.tail(mx.nv) - px).cwiseAbs().maxCoeff(), kTol);
  EXPECT_GT((px - vl.tail(mx.nv)).norm(), 1e-3) << "the projection must actually correct v";
}

TEST(FloatingClosedChain, TheIdentityHandleStillRefusesAFreeFlyer) {
  const auto& fl = Floating();
  EXPECT_THROW(rub::RtClosedChainHandle(fl.GetFullModel(), {}, {}, {}), std::runtime_error);
  EXPECT_THROW(rub::ClosedChainHandle(fl.GetFullModel(), {}, {}, {}), std::runtime_error);
}

// ── Gate 3: closure equivalence of the RT handle (pre-registered) ───────────

TEST(FloatingClosedChain, RtHandleMatchesTheFixedHandleAtFourBases) {
  const auto& fx = Fixed();
  const auto& fl = Floating();
  const pinocchio::Model& mx = *fx.GetFullModel();
  const rub::RtClosedChainHandle probe = MakeRt(fx, fx.GetClosureReferenceConfig());
  const std::vector<std::string> names = probe.GetIndependentJointNames();
  const std::vector<double> q_a_ref = IndependentQ(probe);
  const auto& hand_names = Fixed().GetTreeModel("p1b")->names;

  // Targets: offsets from q_ref, clipped to the joint limits.
  std::mt19937_64 rng(kSeed);
  std::vector<std::vector<double>> targets(kTargets);
  for (auto& t : targets) {
    t = q_a_ref;
    for (std::size_t i = 0; i < names.size(); ++i) {
      const bool hand =
          std::find(hand_names.begin(), hand_names.end(), names[i]) != hand_names.end();
      const double span = hand ? kHandSpan : kBodySpan;
      const int qi = mx.idx_qs[mx.getJointId(names[i])];
      t[i] = std::clamp(t[i] + std::uniform_real_distribution<double>(-span, span)(rng),
                        mx.lowerPositionLimit[qi], mx.upperPositionLimit[qi]);
    }
  }

  // Fingertip poses at q_ref, for the motion floor.
  std::array<pinocchio::SE3, 4> tips_ref;
  for (std::size_t k = 0; k < kTips.size(); ++k) {
    tips_ref[k] = TipInHand(probe, kTips[k]);
  }

  for (int base_k = 0; base_k < 4; ++base_k) {
    SCOPED_TRACE("base seed " + std::to_string(base_k));
    const bool identity_base = base_k == 0;
    Eigen::VectorXd seed = fl.GetClosureReferenceConfig();
    seed.head<rub::kFreeFlyerNq>() = BaseSeed(base_k);
    const Eigen::Matrix<double, 7, 1> root = seed.head<rub::kFreeFlyerNq>();
    if (!identity_base) {
      ASSERT_TRUE(IntegrateMovesTheRoot(*fl.GetFullModel(), seed))
          << "the seed cannot see a rewritten root";
    }
    rub::RtClosedChainHandle hx = MakeRt(fx, fx.GetClosureReferenceConfig());
    rub::RtClosedChainHandle hl = MakeRt(fl, seed);
    ASSERT_EQ(hl.GetIndependentJointNames(), names);

    int reached = 0;
    int multi_clamp = 0;
    int moved = 0;
    int failures = 0;       // keep the log bounded
    double margin_g = 0.0;  // observed / tolerance, rotated bases (printed for the PR)
    double margin_tip = 0.0;
    double margin_closure = 0.0;
    for (const auto& target : targets) {
      const std::vector<double> now = IndependentQ(hx);
      double delta = 0.0;
      for (std::size_t i = 0; i < now.size(); ++i) {
        delta = std::max(delta, std::abs(target[i] - now[i]));
      }
      const int cap = static_cast<int>(std::ceil(delta / rub::kDefaultActuatedIncrement)) + 2;
      int held = 0;
      bool released = false;
      for (int tick = 0; tick < cap && !released; ++tick) {
        const auto sx = hx.Update(target);
        const auto sl = hl.Update(target);
        bool ok = sx.held == sl.held && sx.held_ticks == sl.held_ticks;
        const Eigen::MatrixXd Gx = hx.GetReductionMap();
        const Eigen::MatrixXd Gl = hl.GetReductionMap();
        ok = ok && BitEqual(hl.GetFullConfiguration().head<rub::kFreeFlyerNq>(), root);
        ok = ok && (Gl.topRows(rub::kFreeFlyerNv).array() == 0.0).all();
        const Eigen::MatrixXd Gl_joints = Gl.bottomRows(Gx.rows());
        if (identity_base) {
          ok = ok && sx.closure_error == sl.closure_error && BitEqual(Gl_joints, Gx);
        } else {
          const double kappa = PassiveCondition(fx, hx.GetFullConfiguration());
          const bool both_inf = std::isinf(sx.closure_error) && std::isinf(sl.closure_error);
          const double d_closure = both_inf ? 0.0 : std::abs(sx.closure_error - sl.closure_error);
          ok = ok && d_closure <= kTol;
          ok = ok && MaxAbsDiff(Gl_joints, Gx) <= kTol * kappa * kappa;
          margin_closure = std::max(margin_closure, d_closure / kTol);
          margin_g = std::max(margin_g, MaxAbsDiff(Gl_joints, Gx) / (kTol * kappa * kappa));
        }
        for (const auto& tip : kTips) {
          const pinocchio::SE3 a = TipInHand(hx, tip);
          const pinocchio::SE3 b = TipInHand(hl, tip);
          if (identity_base) {
            ok = ok && BitEqual(a.translation(), b.translation()) &&
                 BitEqual(a.rotation(), b.rotation());
          } else {
            const double d_tip = std::max((a.translation() - b.translation()).cwiseAbs().maxCoeff(),
                                          (a.rotation() - b.rotation()).cwiseAbs().maxCoeff());
            ok = ok && d_tip <= kTol;
            margin_tip = std::max(margin_tip, d_tip / kTol);
          }
        }
        if (!ok && failures++ < 5) {
          ADD_FAILURE() << "tick " << tick << ": fixed vs floating diverged (held " << sx.held
                        << "/" << sl.held << ", closure " << sx.closure_error << "/"
                        << sl.closure_error << ", G Δ " << MaxAbsDiff(Gl_joints, Gx) << ")";
        }
        if (sl.held) {
          ++held;
        } else {
          released = true;
        }
      }
      if (released) {
        ++reached;
      }
      if (held >= 2) {
        ++multi_clamp;
      }
      double motion = 0.0;
      for (std::size_t k = 0; k < kTips.size(); ++k) {
        motion = std::max(
            motion, (TipInHand(hx, kTips[k]).translation() - tips_ref[k].translation()).norm());
      }
      if (motion >= 1e-3) {
        ++moved;
      }
    }
    EXPECT_EQ(failures, 0);
    // Non-vacuity floors: the comparison ran on settled, clamped, moving targets.
    EXPECT_GE(reached, kTargets * 95 / 100) << "settled " << reached << "/" << kTargets;
    EXPECT_GE(multi_clamp, kTargets / 2) << "≥ 2 clamp ticks " << multi_clamp << "/" << kTargets;
    EXPECT_GE(moved, kTargets / 2) << "fingertip ≥ 1 mm " << moved << "/" << kTargets;
    std::printf("[floor] base %d: settled %d · ≥2 clamp ticks %d · fingertip ≥1 mm %d (of %d)\n",
                base_k, reached, multi_clamp, moved, kTargets);
    if (!identity_base) {
      std::printf("[margin] base %d: max observed/tolerance — closure %.3g · G %.3g · tip %.3g\n",
                  base_k, margin_closure, margin_g, margin_tip);
    }
  }
}

// The non-RT handle on the same model: joint slots == fixed, root held, base-fixed dynamics.
TEST(FloatingClosedChain, NonRtHandleMatchesTheFixedHandleAtTheIdentityBase) {
  rub::ClosedChainHandle hx(Fixed());
  rub::ClosedChainHandle hl(Floating());
  const pinocchio::Model& mx = *Fixed().GetFullModel();
  const auto names = hx.GetIndependentJointNames();
  ASSERT_EQ(hl.GetIndependentJointNames(), names);
  const Eigen::VectorXd root = hl.GetFullConfiguration().head<rub::kFreeFlyerNq>();
  std::mt19937_64 rng(kSeed);
  for (int k = 0; k < 20; ++k) {
    std::vector<double> q_a;
    for (const auto& name : names) {
      const int qi = mx.idx_qs[mx.getJointId(name)];
      q_a.push_back(std::clamp(Fixed().GetClosureReferenceConfig()[qi] +
                                   std::uniform_real_distribution<double>(-0.2, 0.2)(rng),
                               mx.lowerPositionLimit[qi], mx.upperPositionLimit[qi]));
    }
    const std::vector<double> v_a(names.size(), 0.1);
    const auto sx = hx.Update(q_a, v_a);
    const auto sl = hl.Update(q_a, v_a);
    ASSERT_EQ(sx.acceptable, sl.acceptable) << k;
    EXPECT_TRUE(BitEqual(hl.GetFullConfiguration().head<rub::kFreeFlyerNq>(), root)) << k;
    EXPECT_TRUE(BitEqual(hl.GetFullConfiguration().tail(mx.nq), hx.GetFullConfiguration())) << k;
    const Eigen::MatrixXd Gl = hl.GetReductionMap();
    EXPECT_TRUE((Gl.topRows(rub::kFreeFlyerNv).array() == 0.0).all()) << k;
    EXPECT_LE(MaxAbsDiff(Gl.bottomRows(mx.nv), hx.GetReductionMap()), kTol) << k;
    // Base-fixed reduced dynamics: the joint block of the floating model's.
    const Eigen::MatrixXd Mx = hx.GetMassMatrix();
    EXPECT_LE(MaxAbsDiff(hl.GetMassMatrix(), Mx), 1e-10 * Mx.cwiseAbs().maxCoeff()) << k;
    EXPECT_LE((hl.GetGeneralizedGravity() - hx.GetGeneralizedGravity()).cwiseAbs().maxCoeff(), 1e-9)
        << k;
    EXPECT_LE((hl.GetNonLinearEffects() - hx.GetNonLinearEffects()).cwiseAbs().maxCoeff(), 1e-8)
        << k;
  }
}

// At a rotated base the held root is a fixed base at that orientation: the floating handle's
// reduced dynamics equal a fixed-root model's whose gravity is the world gravity seen from the
// base (Rᵀ g). The translation does not enter.
TEST(FloatingClosedChain, NonRtHandleAtARotatedBaseIsTheFixedModelUnderRotatedGravity) {
  Eigen::VectorXd seed = Floating().GetClosureReferenceConfig();
  seed.head<rub::kFreeFlyerNq>() = BaseSeed(2);
  const Eigen::Matrix3d R = Eigen::Quaterniond(seed.segment<4>(3)).toRotationMatrix();
  auto tilted = std::make_shared<pinocchio::Model>(*Fixed().GetFullModel());
  tilted->gravity.linear(R.transpose() * tilted->gravity.linear());
  ASSERT_GT((tilted->gravity.linear() - Fixed().GetFullModel()->gravity.linear()).norm(), 1.0)
      << "the seed must actually tilt gravity";
  rub::ClosedChainHandle hx(tilted, Fixed().GetConstraintModels(),
                            Fixed().GetClosureActuatedJointIds(),
                            Fixed().GetClosureReferenceConfig());
  rub::ClosedChainHandle hl(Floating().GetFullModel(), Floating().GetConstraintModels(),
                            Floating().GetClosureActuatedJointIds(), seed);
  const pinocchio::Model& mx = *tilted;
  const auto names = hx.GetIndependentJointNames();
  std::mt19937_64 rng(kSeed + 7U);
  for (int k = 0; k < 10; ++k) {
    std::vector<double> q_a;
    for (const auto& name : names) {
      const int qi = mx.idx_qs[mx.getJointId(name)];
      q_a.push_back(std::clamp(Fixed().GetClosureReferenceConfig()[qi] +
                                   std::uniform_real_distribution<double>(-0.2, 0.2)(rng),
                               mx.lowerPositionLimit[qi], mx.upperPositionLimit[qi]));
    }
    const std::vector<double> v_a(names.size(), 0.1);
    ASSERT_TRUE(hx.Update(q_a, v_a).acceptable) << k;
    ASSERT_TRUE(hl.Update(q_a, v_a).acceptable) << k;
    const Eigen::MatrixXd Mx = hx.GetMassMatrix();
    EXPECT_LE(MaxAbsDiff(hl.GetMassMatrix(), Mx), 1e-10 * Mx.cwiseAbs().maxCoeff()) << k;
    EXPECT_LE((hl.GetGeneralizedGravity() - hx.GetGeneralizedGravity()).cwiseAbs().maxCoeff(), 1e-9)
        << k;
    EXPECT_LE((hl.GetNonLinearEffects() - hx.GetNonLinearEffects()).cwiseAbs().maxCoeff(), 1e-8)
        << k;
  }
}

// A held root is never renormalised, so a seed whose root quaternion pinocchio would not call
// normalised is refused at the door (it would abort a Debug build inside the RT Update); an
// ulp-off unit seed — what an estimator produces — is accepted.
TEST(FloatingClosedChain, ANonNormalizedRootSeedIsRefused) {
  const auto& fl = Floating();
  Eigen::VectorXd bad = fl.GetClosureReferenceConfig();
  bad.segment<4>(3) *= 1.0 + 1e-6;
  ASSERT_FALSE(rub::HasNormalizedFreeFlyerRoot(*fl.GetFullModel(), bad));
  EXPECT_THROW(MakeRt(fl, bad), std::invalid_argument);
  EXPECT_THROW(rub::ClosedChainHandle(fl.GetFullModel(), fl.GetConstraintModels(),
                                      fl.GetClosureActuatedJointIds(), bad),
               std::invalid_argument);
  pinocchio::Data d(*fl.GetFullModel());
  EXPECT_THROW(
      static_cast<void>(rub::ProjectPassiveToConstraint(
          *fl.GetFullModel(), d, fl.GetConstraintModels(), bad, fl.GetClosureActuatedJointIds())),
      std::invalid_argument);
  Eigen::VectorXd zero = bad;
  zero.segment<4>(3).setZero();
  EXPECT_FALSE(rub::HasNormalizedFreeFlyerRoot(*fl.GetFullModel(), zero));

  Eigen::VectorXd ulp_off = fl.GetClosureReferenceConfig();
  ulp_off.head<rub::kFreeFlyerNq>() = BaseSeed(1);
  EXPECT_TRUE(rub::HasNormalizedFreeFlyerRoot(*fl.GetFullModel(), ulp_off));
  EXPECT_NO_THROW(MakeRt(fl, ulp_off));
  // A fixed model has no root quaternion to check.
  EXPECT_TRUE(rub::HasNormalizedFreeFlyerRoot(*Fixed().GetFullModel(),
                                              Fixed().GetClosureReferenceConfig()));
}

// ── Gate 4: NUM-5 on the floating model ─────────────────────────────────────

// A floating copy of RtClosedChainHandle.SeedClampPreservesAssemblyBranchOnLargeJump, at a
// rotated and translated base: the clamp walks the jump in on the reference branch, the base
// stays bit-identical throughout, and without the clamp the same jump leaves the branch.
TEST(FloatingClosedChain, SeedClampPreservesTheBranchOnAFloatingCrankRocker) {
  const rub::ClosedChainModel ccm = rub::BuildClosedChainModelFromExtendedUrdf(
      rtc::test::TestUrdfPath("crank_rocker.urdf"),
      rtc::test::TestUrdfPath("crank_rocker.closure.yaml"), /*root_joint_floating=*/true);
  auto model = std::make_shared<pinocchio::Model>(ccm.model);
  ASSERT_TRUE(rub::IsFreeFlyerRoot(*model));
  ASSERT_FALSE(rub::HasWorldAnchoredConstraint(ccm.constraints));

  constexpr double kStart = 0.05;
  constexpr double kTarget = 1.2;
  Eigen::VectorXd q0 = ccm.q_ref;
  q0.head<rub::kFreeFlyerNq>() = BaseSeed(1);
  const Eigen::Matrix<double, 7, 1> root = q0.head<rub::kFreeFlyerNq>();
  ASSERT_TRUE(IntegrateMovesTheRoot(*model, q0)) << "the seed cannot see a rewritten root";

  rub::ClosedChainHandle ref(model, ccm.constraints, ccm.actuated_joint_ids, q0);
  ASSERT_EQ(ref.nv_independent(), 1);
  ASSERT_TRUE(ref.Update(std::vector<double>{kStart}).converged);
  const Eigen::VectorXd seed = ref.GetFullConfiguration();
  ASSERT_TRUE(BitEqual(seed.head<rub::kFreeFlyerNq>(), root));
  constexpr int kFineSteps = 115;
  for (int k = 1; k <= kFineSteps; ++k) {
    const double q = kStart + (kTarget - kStart) * static_cast<double>(k) / kFineSteps;
    ASSERT_TRUE(ref.Update(std::vector<double>{q}).converged) << k;
  }
  EXPECT_TRUE(BitEqual(ref.GetFullConfiguration().head<rub::kFreeFlyerNq>(), root));
  pinocchio::Data ref_data(*model);
  pinocchio::framesForwardKinematics(*model, ref_data, ref.GetFullConfiguration());

  rub::RtClosedChainHandle rt(model, ccm.constraints, ccm.actuated_joint_ids, seed, 2);
  const auto fid = rt.GetFrameId("c1");
  ASSERT_NE(fid, 0u);
  const Eigen::Vector3d p_truth = ref_data.oMf[fid].translation();
  auto st = rt.Update(std::vector<double>{kTarget});
  EXPECT_TRUE(st.held);
  int ticks = 1;
  for (; ticks < 200 && st.held; ++ticks) {
    st = rt.Update(std::vector<double>{kTarget});
    EXPECT_TRUE(BitEqual(rt.GetFullConfiguration().head<rub::kFreeFlyerNq>(), root)) << ticks;
  }
  EXPECT_FALSE(st.held);
  EXPECT_LE(ticks, 30);
  EXPECT_LT(st.closure_error, 1e-6);
  EXPECT_LT((rt.GetFramePlacement(fid).translation() - p_truth).norm(), 1e-6);

  rub::RtClosedChainHandle rt_off(model, ccm.constraints, ccm.actuated_joint_ids, seed, 2, 1e-6,
                                  1e-6, /*max_seed_increment=*/0.0,
                                  /*max_passive_deviation=*/0.0);
  for (int t = 0; t < 200; ++t) {
    static_cast<void>(rt_off.Update(std::vector<double>{kTarget}));
  }
  EXPECT_GT((rt_off.GetFramePlacement(fid).translation() - p_truth).norm(), 1e-3)
      << "the floating fixture no longer reproduces #248";
}

TEST(FloatingClosedChain, LoopDownstreamMatchesTheFixedModelOnEveryFrame) {
  const pinocchio::Model& mx = *Fixed().GetFullModel();
  const pinocchio::Model& ml = *Floating().GetFullModel();
  const auto& ax = Fixed().GetClosureActuatedJointIds();
  const auto& al = Floating().GetClosureActuatedJointIds();
  const rub::RtClosedChainHandle hl = MakeRt(Floating(), Floating().GetClosureReferenceConfig());
  int downstream = 0;
  for (std::size_t f = 0; f < mx.frames.size(); ++f) {
    const std::string& name = mx.frames[f].name;
    ASSERT_TRUE(ml.existFrame(name)) << name;
    const auto fl = ml.getFrameId(name);
    const bool want = rub::IsFrameDownstreamOfLoop(mx, ax, f);
    EXPECT_EQ(rub::IsFrameDownstreamOfLoop(ml, al, fl), want) << name;
    EXPECT_EQ(hl.IsFrameDownstreamOfLoop(fl), want) << name;
    downstream += want ? 1 : 0;
  }
  // Non-vacuity: the hand's loop links are downstream, the body is not.
  EXPECT_GT(downstream, 0);
  EXPECT_FALSE(rub::IsFrameDownstreamOfLoop(ml, al, ml.getFrameId("pelvis")));
  EXPECT_FALSE(rub::IsFrameDownstreamOfLoop(ml, al, ml.getFrameId(kHandRoot)));
}
