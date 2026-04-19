/// @file test_mpc_factory.cpp
/// @brief Phase 5 factory tests: YAML dispatch and cross-mode handler swap
///        with warm-start transfer.

#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <memory>

#include "rtc_mpc/handler/mpc_factory.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_context.hpp"
#include "rtc_mpc/phase/phase_cost_config.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

namespace {

constexpr const char *kPandaUrdf =
    "/usr/local/share/example-robot-data/robots/panda_description/urdf/"
    "panda.urdf";

constexpr const char *kLightCost = R"(
horizon_length: 15
dt: 0.01
w_frame_placement: 100.0
w_state_reg: 1.0
w_control_reg: 0.01
w_contact_force: 0.0
w_centroidal_momentum: 0.0
W_placement: [100.0, 100.0, 100.0, 10.0, 10.0, 10.0]
q_posture_ref: [0, 0, 0, -1.57, 0, 1.57, 0.785, 0.02, 0.02]
F_target: [0, 0, 0, 0, 0, 0]
custom_weights: {}
)";

constexpr const char *kContactCost = R"(
horizon_length: 10
dt: 0.01
w_frame_placement: 10.0
w_state_reg: 1.0
w_control_reg: 0.1
w_contact_force: 0.5
w_centroidal_momentum: 0.0
W_placement: [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
q_posture_ref: [0, 0, 0, -1.57, 0, 1.57, 0.785, 0.02, 0.02]
F_target: [0, 0, 0, 0, 0, 0]
custom_weights: {}
)";

class MPCFactoryTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!std::filesystem::exists(kPandaUrdf)) {
      GTEST_SKIP() << "Panda URDF not installed — run ./install.sh verify";
    }
    pinocchio::urdf::buildModel(kPandaUrdf, model_);

    auto robot_cfg = YAML::Load(R"(
end_effector_frame: panda_hand
contact_frames:
  - name: panda_leftfinger
    dim: 3
  - name: panda_rightfinger
    dim: 3
)");
    ASSERT_EQ(handler_.Init(model_, robot_cfg),
              rtc::mpc::RobotModelInitError::kNoError);
  }

  rtc::mpc::PhaseContext MakeLightContext() {
    rtc::mpc::PhaseCostConfig cfg{};
    auto yn = YAML::Load(kLightCost);
    EXPECT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(yn, handler_, cfg),
              rtc::mpc::PhaseCostConfigError::kNoError);

    rtc::mpc::PhaseContext ctx{};
    ctx.phase_id = 0;
    ctx.ocp_type = "light_contact";
    ctx.cost_config = cfg;
    ctx.contact_plan = {};
    pinocchio::Data pdata(model_);
    const Eigen::VectorXd q0 = pinocchio::neutral(model_);
    pinocchio::framesForwardKinematics(model_, pdata, q0);
    ctx.ee_target = pdata.oMf[handler_.end_effector_frame_id()];
    return ctx;
  }

  rtc::mpc::PhaseContext MakeContactContext() {
    rtc::mpc::PhaseCostConfig cfg{};
    auto yn = YAML::Load(kContactCost);
    EXPECT_EQ(rtc::mpc::PhaseCostConfig::LoadFromYaml(yn, handler_, cfg),
              rtc::mpc::PhaseCostConfigError::kNoError);

    rtc::mpc::PhaseContext ctx{};
    ctx.phase_id = 1;
    ctx.phase_changed = true;
    ctx.ocp_type = "contact_rich";
    ctx.cost_config = cfg;
    const int lf = handler_.contact_frames()[0].frame_id;
    const int rf = handler_.contact_frames()[1].frame_id;
    ctx.contact_plan.frames = handler_.contact_frames();
    ctx.contact_plan.phases.push_back(
        rtc::mpc::ContactPhase{{lf, rf}, 0.0, 100.0});

    pinocchio::Data pdata(model_);
    const Eigen::VectorXd q0 = pinocchio::neutral(model_);
    pinocchio::framesForwardKinematics(model_, pdata, q0);
    ctx.ee_target = pdata.oMf[handler_.end_effector_frame_id()];
    return ctx;
  }

  rtc::mpc::MPCStateSnapshot MakeStateSnapshot() const {
    rtc::mpc::MPCStateSnapshot s{};
    const Eigen::VectorXd q = pinocchio::neutral(model_);
    s.nq = handler_.nq();
    s.nv = handler_.nv();
    for (int i = 0; i < s.nq; ++i)
      s.q[static_cast<std::size_t>(i)] = q[i];
    for (int i = 0; i < s.nv; ++i)
      s.v[static_cast<std::size_t>(i)] = 0.0;
    return s;
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
};

TEST_F(MPCFactoryTest, DispatchesLightContact) {
  auto cfg = YAML::Load(R"(
mpc:
  ocp_type: light_contact
  solver:
    prim_tol: 1.0e-3
    dual_tol: 1.0e-2
    max_iters: 30
)");
  auto ctx = MakeLightContext();
  std::unique_ptr<rtc::mpc::MPCHandlerBase> h;
  const auto status = rtc::mpc::MPCFactory::Create(cfg, handler_, ctx, h);
  ASSERT_EQ(status.error, rtc::mpc::MPCFactoryError::kNoError);
  ASSERT_NE(h, nullptr);
  EXPECT_EQ(h->ocp_type(), std::string_view("light_contact"));
  EXPECT_TRUE(h->Initialised());
}

TEST_F(MPCFactoryTest, DispatchesContactRich) {
  auto cfg = YAML::Load(R"(
mpc:
  ocp_type: contact_rich
  solver:
    prim_tol: 1.0e-3
    dual_tol: 1.0e-2
    max_iters: 30
  limits:
    friction_mu: 0.7
    n_friction_facets: 4
)");
  auto ctx = MakeContactContext();
  std::unique_ptr<rtc::mpc::MPCHandlerBase> h;
  const auto status = rtc::mpc::MPCFactory::Create(cfg, handler_, ctx, h);
  ASSERT_EQ(status.error, rtc::mpc::MPCFactoryError::kNoError);
  ASSERT_NE(h, nullptr);
  EXPECT_EQ(h->ocp_type(), std::string_view("contact_rich"));
}

TEST_F(MPCFactoryTest, RejectsUnknownOcpType) {
  auto cfg = YAML::Load(R"(
mpc:
  ocp_type: not_a_real_type
)");
  auto ctx = MakeLightContext();
  ctx.ocp_type = "not_a_real_type";
  std::unique_ptr<rtc::mpc::MPCHandlerBase> h;
  const auto status = rtc::mpc::MPCFactory::Create(cfg, handler_, ctx, h);
  EXPECT_EQ(status.error, rtc::mpc::MPCFactoryError::kUnknownOcpType);
  EXPECT_EQ(h, nullptr);
}

TEST_F(MPCFactoryTest, RejectsYamlOcpTypeDriftVsContext) {
  auto cfg = YAML::Load(R"(
mpc:
  ocp_type: contact_rich
)");
  auto ctx = MakeLightContext(); // light_contact, mismatches YAML
  std::unique_ptr<rtc::mpc::MPCHandlerBase> h;
  const auto status = rtc::mpc::MPCFactory::Create(cfg, handler_, ctx, h);
  EXPECT_EQ(status.error, rtc::mpc::MPCFactoryError::kUnknownOcpType);
  EXPECT_EQ(h, nullptr);
}

TEST_F(MPCFactoryTest, RejectsMissingOcpTypeKey) {
  auto cfg = YAML::Load(R"(
mpc:
  solver:
    prim_tol: 1.0e-3
)");
  auto ctx = MakeLightContext();
  std::unique_ptr<rtc::mpc::MPCHandlerBase> h;
  const auto status = rtc::mpc::MPCFactory::Create(cfg, handler_, ctx, h);
  EXPECT_EQ(status.error, rtc::mpc::MPCFactoryError::kMissingOcpTypeKey);
}

TEST_F(MPCFactoryTest, RejectsULimitsDimMismatch) {
  // u_min with wrong size.
  auto cfg = YAML::Load(R"(
mpc:
  ocp_type: light_contact
  limits:
    u_min: [0, 0, 0]
)");
  auto ctx = MakeLightContext();
  std::unique_ptr<rtc::mpc::MPCHandlerBase> h;
  const auto status = rtc::mpc::MPCFactory::Create(cfg, handler_, ctx, h);
  EXPECT_EQ(status.error, rtc::mpc::MPCFactoryError::kInvalidLimits);
}

// ── Cross-mode switch ────────────────────────────────────────────────────

TEST_F(MPCFactoryTest, CrossModeSwapPreservesSolveability) {
  // Build LightContact first, solve, extract warm-start solution.
  auto cfg_light = YAML::Load(R"(
mpc:
  ocp_type: light_contact
  solver:
    prim_tol: 1.0e-3
    dual_tol: 1.0e-2
    max_iters: 40
)");
  auto ctx_light = MakeLightContext();
  std::unique_ptr<rtc::mpc::MPCHandlerBase> light;
  ASSERT_EQ(
      rtc::mpc::MPCFactory::Create(cfg_light, handler_, ctx_light, light).error,
      rtc::mpc::MPCFactoryError::kNoError);

  const auto state = MakeStateSnapshot();
  rtc::mpc::MPCSolution prev{};
  ASSERT_EQ(light->Solve(ctx_light, state, prev),
            rtc::mpc::MPCSolveError::kNoError);

  // Build ContactRich and seed its warm-start with the LightContact
  // solution, then verify the first solve converges (or at minimum does
  // not diverge / throw).
  auto cfg_rich = YAML::Load(R"(
mpc:
  ocp_type: contact_rich
  solver:
    prim_tol: 1.0e-3
    dual_tol: 1.0e-2
    max_iters: 40
  limits:
    friction_mu: 0.7
)");
  auto ctx_rich = MakeContactContext();
  std::unique_ptr<rtc::mpc::MPCHandlerBase> rich;
  ASSERT_EQ(
      rtc::mpc::MPCFactory::Create(cfg_rich, handler_, ctx_rich, rich).error,
      rtc::mpc::MPCFactoryError::kNoError);

  rich->SeedWarmStart(prev);

  rtc::mpc::MPCSolution after{};
  const auto err = rich->Solve(ctx_rich, state, after);
  std::cout << "[cross-mode swap] err=" << static_cast<int>(err)
            << " iters=" << after.iterations << "\n";
  EXPECT_NE(err, rtc::mpc::MPCSolveError::kSolverDiverged);
  EXPECT_NE(err, rtc::mpc::MPCSolveError::kSolverException);
}

} // namespace
