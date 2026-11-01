// The shipped g1_p1b capsule spec (config/g1_p1b/collision_capsules.yaml) on
// the profile's segment model (`g1`, 17 joints, the catch frame on the right
// hand): it binds (every link a BODY frame of the tree model, every pair
// across two joints, the ball capsules off the catching arm), the real
// model's ∇_q d passes the central-difference gate in the separated and the
// penetrating range (radii doubled for the latter), and the per-joint lever
// bound holds. The model is the profile's TEST model (robot_descriptions' G1
// body with the synthetic p1b_surrogate hand, g1_p1b_test_fixture.hpp): the
// body links are the real ones, the right-hand capsule is the surrogate's.
#include "capsule_fd_gate.hpp"
#include "g1_p1b_test_fixture.hpp"
#include "rtc_urdf_bridge/collision/capsule_workspace.hpp"

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <random>
#include <string>

namespace {

namespace fx = integrated_bringup::testfx;
namespace rub = rtc_urdf_bridge;

std::string SpecPath() {
  return std::string(RTC_DEMO_SHARED_CONFIG_DIR) + "/g1_p1b/collision_capsules.yaml";
}

struct Scene {
  fx::G1CatchModels models = fx::LoadG1CatchModels();
  std::shared_ptr<const rub::CapsuleModel> spec;
  std::shared_ptr<const pinocchio::Model> model;
  pinocchio::FrameIndex catch_frame{0};
  std::unique_ptr<pinocchio::Data> data;
  rub::CapsuleWorkspace ws;

  [[nodiscard]] bool Loaded() const { return models.segment.rig.arm.model != nullptr; }

  void Bind(double radius_scale) {
    model = models.segment.rig.arm.model;
    catch_frame = models.segment.rig.arm.frame;
    data = std::make_unique<pinocchio::Data>(*model);
    const auto loaded = rub::CapsuleModel::LoadYaml(SpecPath());
    if (radius_scale == 1.0) {
      spec = loaded;
    } else {
      rub::CapsuleModelSpec s = loaded->Spec();
      for (rub::CapsuleSpec& c : s.capsules) {
        c.radius *= radius_scale;
      }
      spec = std::make_shared<const rub::CapsuleModel>(std::move(s));
    }
    rub::CapsuleWorkspace::BindOptions opt;
    opt.with_ball = true;
    opt.catch_frame = catch_frame;
    ASSERT_EQ(ws.Bind(spec, *model, opt), rub::CapsuleBindStatus::kOk) << ws.BindError();
  }

  // The gate's sample box: the model's limits pulled in a little.
  rtc::test::CapsuleFdOptions Options(bool penetrating, unsigned seed) const {
    rtc::test::CapsuleFdOptions opt;
    opt.penetrating = penetrating;
    opt.seed = seed;
    opt.q_lo = model->lowerPositionLimit.array() + 0.05;
    opt.q_hi = model->upperPositionLimit.array() - 0.05;
    opt.with_ball = true;
    // A ball beside the left arm at the home posture, inside its reach.
    opt.ball_p0 = Eigen::Vector3d(0.25, 0.30, 0.25);
    opt.ball_p1 = Eigen::Vector3d(0.15, 0.35, 0.20);
    opt.ball_radius = 0.035;
    return opt;
  }
};

}  // namespace

TEST(G1CollisionCapsules, TheShippedSpecBindsToTheSegmentModel) {
  Scene sc;
  if (!sc.Loaded()) {
    GTEST_SKIP() << "the g1_p1b test model is not available";
  }
  sc.Bind(1.0);
  EXPECT_EQ(sc.ws.NumSelfPairs(), 42U);
  EXPECT_EQ(sc.ws.NumBallPairs(), 4U);
  EXPECT_EQ(sc.ws.Nv(), fx::kG1Dof);
  // Only the pelvis pairs read the waist: the trunk moves with the arms (a
  // common ancestor moves both sides), as do the arm–arm pairs.
  int waist_pairs = 0;
  for (std::size_t p = 0; p < sc.ws.NumSelfPairs(); ++p) {
    const rub::CapsulePairInfo& info = sc.ws.Pair(p);
    EXPECT_GT(info.lever_arm_bound, 0.0);
    if (info.lever_arm.head(3).cwiseAbs().maxCoeff() > 0.0) {
      ++waist_pairs;
    }
  }
  EXPECT_EQ(waist_pairs, 6) << "the six pelvis pairs read the waist, nothing else does";
  // At the home posture nothing is in collision and the ball rows are clear.
  Eigen::VectorXd q(fx::kG1Dof);
  for (int m = 0; m < fx::kG1Dof; ++m) {
    q[m] = fx::kG1Home[static_cast<std::size_t>(
        sc.models.segment.device_of_model[static_cast<std::size_t>(m)])];
  }
  rtc::test::CapsuleDistancesAt(*sc.model, *sc.data, sc.ws, sc.Options(false, 0), q);
  double min_self = std::numeric_limits<double>::infinity();
  for (std::size_t p = 0; p < sc.ws.NumSelfPairs(); ++p) {
    min_self = std::min(min_self, sc.ws.Distance(p));
  }
  std::cout << "[g1 capsules] home posture min self distance = " << min_self << " m\n";
  // Clear of d_safe + margin of the shipped collision keys' defaults (0.03 m):
  // the home posture must not sit on an active row.
  EXPECT_GT(min_self, 0.03) << "the home posture is inside the safe distance on the spec";
}

TEST(G1CollisionCapsules, GradientsMatchFiniteDifferencesSeparated) {
  Scene sc;
  if (!sc.Loaded()) {
    GTEST_SKIP() << "the g1_p1b test model is not available";
  }
  sc.Bind(1.0);
  const rtc::test::CapsuleFdStats st = rtc::test::CheckCapsuleGradientsAgainstFd(
      *sc.model, *sc.data, *sc.spec, sc.ws, sc.Options(false, 11));
  rtc::test::PrintCapsuleFdStats("g1 separated", st);
  EXPECT_GE(st.separated, 2000);
}

TEST(G1CollisionCapsules, GradientsMatchFiniteDifferencesPenetrating) {
  Scene sc;
  if (!sc.Loaded()) {
    GTEST_SKIP() << "the g1_p1b test model is not available";
  }
  sc.Bind(2.0);  // doubled radii: the random postures penetrate often
  const rtc::test::CapsuleFdStats st = rtc::test::CheckCapsuleGradientsAgainstFd(
      *sc.model, *sc.data, *sc.spec, sc.ws, sc.Options(true, 12));
  rtc::test::PrintCapsuleFdStats("g1 penetrating", st);
  EXPECT_GE(st.penetrating, 100);
}

TEST(G1CollisionCapsules, LeverArmBoundHoldsOnTheRealModel) {
  Scene sc;
  if (!sc.Loaded()) {
    GTEST_SKIP() << "the g1_p1b test model is not available";
  }
  sc.Bind(1.0);
  const rtc::test::CapsuleFdOptions opt = sc.Options(false, 13);
  std::mt19937 rng(13);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::uniform_real_distribution<double> step(-0.02, 0.02);
  double worst = 0.0;
  for (int s = 0; s < 200; ++s) {
    Eigen::VectorXd q(fx::kG1Dof);
    Eigen::VectorXd dq(fx::kG1Dof);
    for (int i = 0; i < fx::kG1Dof; ++i) {
      q[i] = opt.q_lo[i] + (opt.q_hi[i] - opt.q_lo[i]) * u(rng);
      dq[i] = step(rng);
    }
    rtc::test::CapsuleDistancesAt(*sc.model, *sc.data, sc.ws, opt, q);
    std::vector<double> d0(sc.ws.NumPairs());
    for (std::size_t p = 0; p < d0.size(); ++p) {
      d0[p] = sc.ws.Distance(p);
    }
    rtc::test::CapsuleDistancesAt(*sc.model, *sc.data, sc.ws, opt, q + dq);
    for (std::size_t p = 0; p < d0.size(); ++p) {
      const double change = std::abs(sc.ws.Distance(p) - d0[p]);
      const double bound = sc.ws.Pair(p).lever_arm.dot(dq.cwiseAbs());
      worst = std::max(worst, change / bound);
      EXPECT_LE(change, bound + 1e-12) << "pair " << p;
    }
  }
  std::cout << "[g1 lever arm] worst |Δd| / (Σ_i c_ij |Δq_i|) = " << worst << "\n";
}
