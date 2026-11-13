/// @file test_kinodynamic_dynamics.cpp
/// @brief `KinodynamicDynamics` on the public G1 (free-flyer, two 6-D sole
///        contacts): configure checks, the base-acceleration law against
///        three rigid-body oracles (RNEA, M_bb, ABA with SE3-transported
///        forces), oracle mutants, gravity from the model, static support,
///        contact masking and in-place mode switching inside a stage.
///
/// Gates are pre-registered: seed 20261114, 200 draws (50 per mode) plus a
/// fixed edge set of 40 (knees at their lower limit, high speed, near-π
/// attitude and ±q pairs), and 10 later additions with the knees at exactly
/// zero. Tolerances: RNEA `64·nv·eps·S` with `S = ‖Σ JᵀW‖∞ + ‖h_b‖∞` (at
/// least 1); M_bb `64·nv·eps·max(1, ‖v̇_b‖∞)`; ABA `1e-9·max(1, ‖v̇_b‖∞)`.

#include "rtc_mpc/dynamics/kinodynamic_dynamics.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_fixture.hpp"
#include "test_utils/loco_reference_fixture.hpp"
#include "test_utils/panda_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Geometry>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <aligator/modelling/dynamics/integrator-semi-euler.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <random>
#include <set>
#include <string>
#include <vector>

namespace {

using rtc::mpc::KinodynamicDynamics;
using rtc::mpc::KinodynamicError;
using rtc::mpc::RobotModelHandler;
using rtc::mpc::RobotModelInitError;
using rtc::mpc::test_utils::KinoOracle;
using rtc::mpc::test_utils::KinoRanges;
using rtc::mpc::test_utils::KinoSample;
using Ode = KinodynamicDynamics::Ode;
using OdeData = KinodynamicDynamics::OdeData;
using Vec6 = Eigen::Matrix<double, 6, 1>;

constexpr std::uint32_t kGateSeed = 20261114U;
constexpr std::uint32_t kControlSeed = 20261113U;
constexpr double kEps = std::numeric_limits<double>::epsilon();

[[nodiscard]] bool BitEqual(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
  return a.rows() == b.rows() && a.cols() == b.cols() &&
         std::memcmp(a.data(), b.data(), sizeof(double) * static_cast<std::size_t>(a.size())) == 0;
}

/// One ODE evaluation on fresh data.
struct Eval {
  Eigen::VectorXd xdot;
  Eigen::MatrixXd jx;
  Eigen::MatrixXd ju;
};

Eval Evaluate(const Ode& ode, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  auto data = ode.createData();
  ode.forward(x, u, *data);
  ode.dForward(x, u, *data);
  return {data->xdot_, data->Jx_, data->Ju_};
}

Eval Evaluate(const Ode& ode, OdeData& data, const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  ode.forward(x, u, data);
  ode.dForward(x, u, data);
  return {data.xdot_, data.Jx_, data.Ju_};
}

class KinodynamicDynamicsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(rtc::mpc::test_utils::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << rtc::mpc::test_utils::kG1Urdf;
    }
    model_ = rtc::mpc::test_utils::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(rtc::mpc::test_utils::kG1SolesYaml)),
              RobotModelInitError::kNoError);
    ASSERT_EQ(dyn_.Init(handler_), KinodynamicError::kNoError);
    std::vector<pinocchio::FrameIndex> ids;
    ids.push_back(model_.getFrameId("left_sole"));
    ids.push_back(model_.getFrameId("right_sole"));
    oracle_.emplace(model_, ids, 6);
  }

  Vec6 BaseAccel(const Eval& e) const { return e.xdot.segment<6>(model_.nv); }

  std::vector<KinoSample> GateSamples(std::uint32_t seed) const {
    return rtc::mpc::test_utils::G1GateSamples(*oracle_, seed);
  }

  pinocchio::Model model_;
  RobotModelHandler handler_;
  KinodynamicDynamics dyn_;
  std::optional<KinoOracle> oracle_;
};

// ── Configure ────────────────────────────────────────────────────────────

TEST_F(KinodynamicDynamicsTest, LayoutMatchesTheHandler) {
  ASSERT_EQ(model_.nv, 29);  // fixture sanity: 23 actuated joints on a free-flyer
  EXPECT_EQ(dyn_.n_contacts(), 2);
  EXPECT_EQ(dyn_.force_size(), 6);
  EXPECT_EQ(dyn_.nu(), handler_.nu());
  EXPECT_EQ(dyn_.n_modes(), 4U);
  const rtc::mpc::WbLayout& layout = handler_.layout();
  for (int k = 0; k < 2; ++k) {
    EXPECT_EQ(layout.contact_u_offset[static_cast<std::size_t>(k)], k * dyn_.force_size());
  }
  for (std::uint32_t mode = 0; mode < dyn_.n_modes(); ++mode) {
    const std::optional<Ode> ode = dyn_.MakeOde(mode);
    ASSERT_TRUE(ode.has_value()) << mode;
    EXPECT_EQ(ode->nu(), handler_.nu());
    ASSERT_EQ(ode->contact_ids_.size(), 2U);
    for (std::size_t k = 0; k < 2; ++k) {
      EXPECT_EQ(static_cast<int>(ode->contact_ids_[k]), handler_.contact_frames()[k].frame_id);
      EXPECT_EQ(ode->contact_states_[k], ((mode >> k) & 1U) != 0U) << mode;
    }
    EXPECT_EQ(ode->gravity_, model_.gravity.linear());
  }
}

TEST_F(KinodynamicDynamicsTest, RejectsWhatItCannotModelWithDistinctCodes) {
  std::set<KinodynamicError> seen;

  KinodynamicDynamics d;
  const RobotModelHandler uninit;
  const KinodynamicError e_uninit = d.Init(uninit);
  seen.insert(e_uninit);
  EXPECT_EQ(e_uninit, KinodynamicError::kModelNotInitialised);
  EXPECT_FALSE(d.Initialised());

  if (std::filesystem::exists(rtc::mpc::test_utils::kPandaUrdf)) {
    pinocchio::Model fixed;
    pinocchio::urdf::buildModel(rtc::mpc::test_utils::kPandaUrdf, fixed);
    RobotModelHandler h;
    ASSERT_EQ(h.Init(fixed, YAML::Load(rtc::mpc::test_utils::kPandaTwoFingerYaml)),
              RobotModelInitError::kNoError);
    const KinodynamicError e = d.Init(h);
    seen.insert(e);
    EXPECT_EQ(e, KinodynamicError::kFixedBase);

    // A floating model with one 6-D and one 3-D contact.
    const pinocchio::Model mixed = rtc::mpc::test_utils::BuildLocoPanda();
    RobotModelHandler hm;
    ASSERT_EQ(hm.Init(mixed, YAML::Load(rtc::mpc::test_utils::kLocoHandlerYaml)),
              RobotModelInitError::kNoError);
    const KinodynamicError em = d.Init(hm);
    seen.insert(em);
    EXPECT_EQ(em, KinodynamicError::kMixedContactDim);
  } else {
    ADD_FAILURE() << "Panda URDF missing: the fixed-base and mixed-dim cases did not run";
  }

  RobotModelHandler none;
  ASSERT_EQ(none.Init(model_, YAML::Load(R"(
end_effector_frame: pelvis
base_frame: universe
contact_frames: []
)")),
            RobotModelInitError::kNoError);
  const KinodynamicError e_none = d.Init(none);
  seen.insert(e_none);
  EXPECT_EQ(e_none, KinodynamicError::kNoContacts);
  EXPECT_FALSE(d.Initialised());
  EXPECT_FALSE(d.MakeOde(0).has_value());
  EXPECT_EQ(seen.size(), 4U) << "the four refusals must not share a code";

  // A mode naming a contact that does not exist is refused and changes nothing.
  std::optional<Ode> ode = dyn_.MakeOde(0b01);
  ASSERT_TRUE(ode.has_value());
  EXPECT_FALSE(dyn_.MakeOde(dyn_.n_modes()).has_value());
  for (const std::uint32_t bad : {4U, 5U, 0xFFFFFFFFU}) {
    EXPECT_FALSE(dyn_.SetMode(*ode, bad)) << bad;
    EXPECT_TRUE(ode->contact_states_[0]);
    EXPECT_FALSE(ode->contact_states_[1]);
  }
  // An ODE built for another contact configuration is refused too.
  RobotModelHandler one;
  ASSERT_EQ(one.Init(model_, YAML::Load(R"(
end_effector_frame: pelvis
base_frame: universe
contact_frames:
  - name: left_sole
    dim: 6
)")),
            RobotModelInitError::kNoError);
  KinodynamicDynamics d_one;
  ASSERT_EQ(d_one.Init(one), KinodynamicError::kNoError);
  EXPECT_FALSE(d_one.SetMode(*ode, 1U));
  // Same count and force size, other frames (the soles in the other order).
  RobotModelHandler swapped;
  ASSERT_EQ(swapped.Init(model_, YAML::Load(R"(
end_effector_frame: pelvis
base_frame: universe
contact_frames:
  - name: right_sole
    dim: 6
  - name: left_sole
    dim: 6
)")),
            RobotModelInitError::kNoError);
  KinodynamicDynamics d_swapped;
  ASSERT_EQ(d_swapped.Init(swapped), KinodynamicError::kNoError);
  EXPECT_FALSE(d_swapped.SetMode(*ode, 2U));
  EXPECT_TRUE(ode->contact_states_[0]);
  EXPECT_FALSE(ode->contact_states_[1]);
}

// ── The base-acceleration law against rigid-body oracles ────────────────

TEST_F(KinodynamicDynamicsTest, RneaBaseResidualIsAtRoundoff) {
  const int nv = model_.nv;
  int pass = 0;
  double worst = 0.0;
  double cond_max = 0.0;
  const std::vector<KinoSample> samples = GateSamples(kGateSeed);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const KinoSample& s = samples[i];
    const std::optional<Ode> ode = dyn_.MakeOde(s.mode);
    ASSERT_TRUE(ode.has_value());
    const Eval e = Evaluate(*ode, s.x, s.u);
    const Eigen::VectorXd q = s.x.head(model_.nq);
    const Eigen::VectorXd v = s.x.tail(nv);
    const Eigen::VectorXd jtw = oracle_->JtW(q, s.u, s.mode);
    const Vec6 hb = oracle_->HBase(q, v);
    const double scale =
        std::max(1.0, jtw.head<6>().cwiseAbs().maxCoeff() + hb.cwiseAbs().maxCoeff());
    const double tol = 64.0 * nv * kEps * scale;
    const Vec6 r = oracle_->RneaBase(q, v, oracle_->FullA(BaseAccel(e), s.u)) - jtw.head<6>();
    const double res = r.cwiseAbs().maxCoeff();
    worst = std::max(worst, res / tol);
    pass += (std::isfinite(res) && res <= tol) ? 1 : 0;
    EXPECT_LE(res, tol) << "sample " << i << " mode " << s.mode;
    cond_max = std::max(cond_max, oracle_->CondAb(q, v));
  }
  EXPECT_EQ(pass, static_cast<int>(samples.size()));
  RecordProperty("rnea_worst_over_tol", std::to_string(worst));
  RecordProperty("cond_Ab_max", std::to_string(cond_max));
  std::printf("RNEA: %d/%zu pass, worst residual/tol %.3e, cond(A_b) max %.1f\n", pass,
              samples.size(), worst, cond_max);
}

TEST_F(KinodynamicDynamicsTest, MassMatrixBaseSolveAgrees) {
  int pass = 0;
  double worst = 0.0;
  const std::vector<KinoSample> samples = GateSamples(kGateSeed);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const KinoSample& s = samples[i];
    const Eval e = Evaluate(dyn_.MakeOde(s.mode).value(), s.x, s.u);
    const Eigen::VectorXd q = s.x.head(model_.nq);
    const Eigen::VectorXd v = s.x.tail(model_.nv);
    const Vec6 ab = oracle_->MbbSolve(q, v, s.u, oracle_->JtW(q, s.u, s.mode));
    const Vec6 vb = BaseAccel(e);
    const double tol = 64.0 * model_.nv * kEps * std::max(1.0, vb.cwiseAbs().maxCoeff());
    const double res = (vb - ab).cwiseAbs().maxCoeff();
    worst = std::max(worst, res / tol);
    pass += (std::isfinite(res) && res <= tol) ? 1 : 0;
    EXPECT_LE(res, tol) << "sample " << i << " mode " << s.mode;
  }
  EXPECT_EQ(pass, static_cast<int>(samples.size()));
  RecordProperty("mbb_worst_over_tol", std::to_string(worst));
  std::printf("M_bb: %d/%zu pass, worst residual/tol %.3e\n", pass, samples.size(), worst);
}

// ABA with the wrenches carried to their joints by SE3 actions (no
// Jacobian; the force acts at the contact frame origin). The tolerance was
// set against this oracle's floor on the control seed (max 8.6e-14).
TEST_F(KinodynamicDynamicsTest, AbaOracleAgrees) {
  int pass = 0;
  double worst = 0.0;
  const std::vector<KinoSample> samples = GateSamples(kGateSeed);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const KinoSample& s = samples[i];
    const Eval e = Evaluate(dyn_.MakeOde(s.mode).value(), s.x, s.u);
    const Eigen::VectorXd q = s.x.head(model_.nq);
    const Eigen::VectorXd v = s.x.tail(model_.nv);
    const Vec6 vb = BaseAccel(e);
    const Eigen::VectorXd a_aba =
        oracle_->AbaAcceleration(q, v, oracle_->FullA(vb, s.u), s.u, s.mode);
    const double tol = 1e-9 * std::max(1.0, vb.cwiseAbs().maxCoeff());
    const double res = (a_aba.head<6>() - vb).cwiseAbs().maxCoeff();
    worst = std::max(worst, res / tol);
    pass += (std::isfinite(res) && res <= tol) ? 1 : 0;
    EXPECT_LE(res, tol) << "sample " << i << " mode " << s.mode;
  }
  EXPECT_EQ(pass, static_cast<int>(samples.size()));
  RecordProperty("aba_worst_over_tol", std::to_string(worst));
  std::printf("ABA: %d/%zu pass, worst residual/tol %.3e\n", pass, samples.size(), worst);
}

// Each oracle mutant — a plausible wrong convention — misses the ODE by at
// least 1e3 times the M_bb tolerance on every sample it applies to, so the
// gates above can tell them apart.
TEST_F(KinodynamicDynamicsTest, OracleMutantsAreFarOutsideTheTolerance) {
  pinocchio::Model g_mutant_model = model_;
  g_mutant_model.gravity.linear(Eigen::Vector3d(0.0, 0.0, -9.80665));
  KinoOracle g_mutant(g_mutant_model, oracle_->contacts(), 6);

  struct Margin {
    const char* name;
    double min_ratio = std::numeric_limits<double>::infinity();
    int n = 0;
  };

  std::array<Margin, 6> m{{{"3x3 split solve"},
                           {"LOCAL Jacobian"},
                           {"WORLD Jacobian"},
                           {"moment about the CoM"},
                           {"contacts swapped"},
                           {"g = 9.80665"}}};
  auto note = [](Margin& mg, const Vec6& mutant, const Vec6& vb, double tol) {
    mg.min_ratio = std::min(mg.min_ratio, (mutant - vb).cwiseAbs().maxCoeff() / tol);
    ++mg.n;
  };
  for (const KinoSample& s : GateSamples(kGateSeed)) {
    const Eval e = Evaluate(dyn_.MakeOde(s.mode).value(), s.x, s.u);
    const Eigen::VectorXd q = s.x.head(model_.nq);
    const Eigen::VectorXd v = s.x.tail(model_.nv);
    const Vec6 vb = BaseAccel(e);
    const double tol = 64.0 * model_.nv * kEps * std::max(1.0, vb.cwiseAbs().maxCoeff());
    const Eigen::VectorXd jtw = oracle_->JtW(q, s.u, s.mode);
    note(m[5], g_mutant.MbbSolve(q, v, s.u, jtw), vb, tol);
    if (s.mode == 0U) {
      continue;  // the contact mutants coincide with no contact
    }
    note(m[0], oracle_->MbbSolveSplit(q, v, s.u, jtw), vb, tol);
    note(m[1], oracle_->MbbSolve(q, v, s.u, oracle_->JtW(q, s.u, s.mode, pinocchio::LOCAL)), vb,
         tol);
    note(m[2], oracle_->MbbSolve(q, v, s.u, oracle_->JtW(q, s.u, s.mode, pinocchio::WORLD)), vb,
         tol);
    note(m[3], oracle_->MbbSolve(q, v, s.u, oracle_->FextGeneralized(q, s.u, s.mode, true)), vb,
         tol);
    note(m[4],
         oracle_->MbbSolve(q, v, s.u,
                           oracle_->JtW(q, s.u, s.mode, pinocchio::LOCAL_WORLD_ALIGNED, true)),
         vb, tol);
  }
  for (const Margin& mg : m) {
    EXPECT_GT(mg.n, 0) << mg.name;
    EXPECT_GE(mg.min_ratio, 1e3) << mg.name;
    std::printf("mutant %-22s n %3d  min |Δ|/tol %.3e\n", mg.name, mg.n, mg.min_ratio);
  }
  // The point-of-action oracle itself agrees with the Jacobian one when the
  // moment is taken at the contact (the control for the CoM mutant).
  for (const KinoSample& s : GateSamples(kGateSeed)) {
    const Eigen::VectorXd q = s.x.head(model_.nq);
    const Eigen::VectorXd a = oracle_->FextGeneralized(q, s.u, s.mode, false);
    const Eigen::VectorXd b = oracle_->JtW(q, s.u, s.mode);
    EXPECT_LE((a - b).cwiseAbs().maxCoeff(),
              64.0 * model_.nv * kEps * std::max(1.0, b.cwiseAbs().maxCoeff()));
  }
}

// Gravity comes from the model, not a constant.
TEST_F(KinodynamicDynamicsTest, GravityIsTheModels) {
  pinocchio::Model tilted = model_;
  tilted.gravity.linear(Eigen::Vector3d(1.0, -2.0, -9.5));
  RobotModelHandler h;
  ASSERT_EQ(h.Init(tilted, YAML::Load(rtc::mpc::test_utils::kG1SolesYaml)),
            RobotModelInitError::kNoError);
  KinodynamicDynamics d;
  ASSERT_EQ(d.Init(h), KinodynamicError::kNoError);
  KinoOracle oracle(tilted, oracle_->contacts(), 6);
  std::mt19937 rng(kGateSeed);
  for (int i = 0; i < 20; ++i) {
    const KinoSample s = oracle.Draw(rng, static_cast<std::uint32_t>(i % 4));
    const Eval e = Evaluate(d.MakeOde(s.mode).value(), s.x, s.u);
    const Eigen::VectorXd q = s.x.head(tilted.nq);
    const Eigen::VectorXd v = s.x.tail(tilted.nv);
    const Eigen::VectorXd jtw = oracle.JtW(q, s.u, s.mode);
    const Vec6 hb = oracle.HBase(q, v);
    const double tol =
        64.0 * tilted.nv * kEps *
        std::max(1.0, jtw.head<6>().cwiseAbs().maxCoeff() + hb.cwiseAbs().maxCoeff());
    const Vec6 r =
        oracle.RneaBase(q, v, oracle.FullA(e.xdot.segment<6>(tilted.nv), s.u)) - jtw.head<6>();
    EXPECT_LE(r.cwiseAbs().maxCoeff(), tol) << "sample " << i;
  }
}

// q and −q are one rotation: the dynamics must not see the sign.
TEST_F(KinodynamicDynamicsTest, QuaternionSignDoesNotMatter) {
  int pairs = 0;
  for (const KinoSample& s : GateSamples(kGateSeed)) {
    KinoSample flipped = s;
    flipped.x.segment<4>(3) = -s.x.segment<4>(3);
    const Ode ode = dyn_.MakeOde(s.mode).value();
    const Eval a = Evaluate(ode, s.x, s.u);
    const Eval b = Evaluate(ode, flipped.x, flipped.u);
    const double tol = 64.0 * model_.nv * kEps * std::max(1.0, a.xdot.cwiseAbs().maxCoeff());
    EXPECT_LE((a.xdot - b.xdot).cwiseAbs().maxCoeff(), tol) << "mode " << s.mode;
    ++pairs;
  }
  EXPECT_EQ(pairs, 250);
}

// 3-D point contacts follow the same law (no moment block).
TEST_F(KinodynamicDynamicsTest, PointContactsFollowTheSameLaw) {
  RobotModelHandler h;
  ASSERT_EQ(h.Init(model_, YAML::Load(R"(
end_effector_frame: pelvis
base_frame: universe
contact_frames:
  - name: left_sole
    dim: 3
  - name: right_sole
    dim: 3
)")),
            RobotModelInitError::kNoError);
  KinodynamicDynamics d;
  ASSERT_EQ(d.Init(h), KinodynamicError::kNoError);
  ASSERT_EQ(d.force_size(), 3);
  ASSERT_EQ(d.nu(), h.nu());
  KinoOracle oracle(model_, oracle_->contacts(), 3);
  std::mt19937 rng(kGateSeed);
  for (int i = 0; i < 40; ++i) {
    const KinoSample s = oracle.Draw(rng, static_cast<std::uint32_t>(i % 4));
    const Eval e = Evaluate(d.MakeOde(s.mode).value(), s.x, s.u);
    const Eigen::VectorXd q = s.x.head(model_.nq);
    const Eigen::VectorXd v = s.x.tail(model_.nv);
    const Eigen::VectorXd jtw = oracle.JtW(q, s.u, s.mode);
    const Vec6 hb = oracle.HBase(q, v);
    const double tol =
        64.0 * model_.nv * kEps *
        std::max(1.0, jtw.head<6>().cwiseAbs().maxCoeff() + hb.cwiseAbs().maxCoeff());
    const Vec6 r = oracle.RneaBase(q, v, oracle.FullA(BaseAccel(e), s.u)) - jtw.head<6>();
    EXPECT_LE(r.cwiseAbs().maxCoeff(), tol) << "sample " << i;
  }
}

// At rest in double support with weight-compensating wrenches and no joint
// acceleration, the base does not accelerate.
TEST_F(KinodynamicDynamicsTest, StaticDoubleSupportHoldsTheBase) {
  std::mt19937 rng(kGateSeed);
  const std::optional<Ode> ode = dyn_.MakeOde(0b11);
  ASSERT_TRUE(ode.has_value());
  double worst = 0.0;
  for (int i = 0; i < 20; ++i) {
    Eigen::VectorXd x(model_.nq + model_.nv);
    x << oracle_->DrawQ(rng), Eigen::VectorXd::Zero(model_.nv);
    const Eigen::VectorXd u = oracle_->WeightCompensation(x.head(model_.nq), 0b11);
    const Eval e = Evaluate(*ode, x, u);
    const double a = BaseAccel(e).cwiseAbs().maxCoeff();
    worst = std::max(worst, a);
    EXPECT_LE(a, 1e-12) << "sample " << i;
  }
  RecordProperty("static_worst", std::to_string(worst));
  std::printf("static support: worst |v̇_b|∞ %.3e (gate 1e-12)\n", worst);
}

// ── Contact masking and mode switching ──────────────────────────────────

TEST_F(KinodynamicDynamicsTest, InactiveWrenchSlotsReachNothing) {
  const int fs = dyn_.force_size();
  int inactive_checked = 0;
  int inactive_expected = 0;
  for (const KinoSample& s : GateSamples(kGateSeed)) {
    inactive_expected += 2 - static_cast<int>((s.mode & 1U) + ((s.mode >> 1) & 1U));
    const Ode ode = dyn_.MakeOde(s.mode).value();
    const Eval ref = Evaluate(ode, s.x, s.u);
    // Control: the same input twice is bit-identical.
    const Eval again = Evaluate(ode, s.x, s.u);
    ASSERT_TRUE(BitEqual(again.xdot, ref.xdot) && BitEqual(again.jx, ref.jx) &&
                BitEqual(again.ju, ref.ju));
    for (int c = 0; c < 2; ++c) {
      Eigen::VectorXd u2 = s.u;
      u2.segment(c * fs, fs).setConstant(1e3);
      u2[c * fs + 1] = -1e3;
      const Eval pert = Evaluate(ode, s.x, u2);
      const bool active = ((s.mode >> c) & 1U) != 0U;
      if (active) {
        // Control: an active slot does move the result.
        EXPECT_FALSE(BitEqual(pert.xdot, ref.xdot)) << "mode " << s.mode << " slot " << c;
        continue;
      }
      ++inactive_checked;
      EXPECT_TRUE(BitEqual(pert.xdot, ref.xdot)) << "mode " << s.mode << " slot " << c;
      EXPECT_TRUE(BitEqual(pert.jx, ref.jx)) << "mode " << s.mode << " slot " << c;
      EXPECT_TRUE(BitEqual(pert.ju, ref.ju)) << "mode " << s.mode << " slot " << c;
      for (int j = 0; j < fs; ++j) {
        EXPECT_TRUE((ref.ju.col(c * fs + j).array() == 0.0).all())
            << "mode " << s.mode << " slot " << c << " column " << j;
      }
    }
  }
  EXPECT_EQ(inactive_checked, inactive_expected);
  EXPECT_GT(inactive_checked, 200);
}

// The mode is written to the ODE copy the stage owns, in place, and one data
// object serves every mode in turn; an ODE outside the stage — even the one
// it was built from — does not reach it.
TEST_F(KinodynamicDynamicsTest, ModeSwitchesInPlaceInsideAStage) {
  using Integrator = aligator::dynamics::IntegratorSemiImplEulerTpl<double>;
  using Stage = aligator::StageModelTpl<double>;
  const int nu = dyn_.nu();
  Ode prototype = dyn_.MakeOde(0b00).value();
  const aligator::CostStackTpl<double> cost(*dyn_.space(), nu);
  std::vector<xyz::polymorphic<Stage>> stages;
  stages.emplace_back(Stage(cost, Integrator(prototype, 0.02)));
  stages.emplace_back(Stage(cost, Integrator(prototype, 0.02)));
  aligator::TrajOptProblemTpl<double> problem(dyn_.space()->neutral(), stages, cost);

  Ode* in_stage = KinodynamicDynamics::OdeOf(*problem.stages_[1]);
  ASSERT_NE(in_stage, nullptr);
  ASSERT_NE(in_stage, &prototype);
  ASSERT_NE(in_stage, KinodynamicDynamics::OdeOf(*problem.stages_[0]));
  auto reused = in_stage->createData();

  std::mt19937 rng(kGateSeed);
  for (const std::uint32_t mode : {0U, 3U, 1U, 2U, 0U}) {
    const KinoSample s = oracle_->Draw(rng, mode);
    ASSERT_TRUE(dyn_.SetMode(*in_stage, mode));
    const Eval got = Evaluate(*in_stage, static_cast<OdeData&>(*reused), s.x, s.u);
    const Eval fresh = Evaluate(dyn_.MakeOde(mode).value(), s.x, s.u);
    EXPECT_TRUE(BitEqual(got.xdot, fresh.xdot)) << "mode " << mode;
    EXPECT_TRUE(BitEqual(got.jx, fresh.jx)) << "mode " << mode;
    EXPECT_TRUE(BitEqual(got.ju, fresh.ju)) << "mode " << mode;
  }
  // Control: switching the prototype, or the other stage, leaves this one.
  ASSERT_TRUE(dyn_.SetMode(*in_stage, 0b10));
  ASSERT_TRUE(dyn_.SetMode(prototype, 0b01));
  ASSERT_TRUE(dyn_.SetMode(*KinodynamicDynamics::OdeOf(*problem.stages_[0]), 0b11));
  EXPECT_FALSE(in_stage->contact_states_[0]);
  EXPECT_TRUE(in_stage->contact_states_[1]);

  // A stage whose dynamics is not an integrator over this ODE has none.
  struct NotKino : aligator::ExplicitDynamicsModelTpl<double> {
    explicit NotKino(const KinodynamicDynamics::Space& sp, int n)
        : aligator::ExplicitDynamicsModelTpl<double>(sp, n) {}

    void forward(const ConstVectorRef& x, const ConstVectorRef&, Data& d) const override {
      d.xnext_ = x;
    }

    void dForward(const ConstVectorRef&, const ConstVectorRef&, Data&) const override {}
  };

  Stage other(cost, NotKino(*dyn_.space(), nu));
  EXPECT_EQ(KinodynamicDynamics::OdeOf(other), nullptr);
}

}  // namespace
