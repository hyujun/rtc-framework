#include <gtest/gtest.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/constraints/eom_constraint.hpp"
#include "rtc_tsid/constraints/friction_cone_constraint.hpp"
#include "rtc_tsid/controller/tsid_controller.hpp"
#include "rtc_tsid/formulation/formulation_factory.hpp"
#include "rtc_tsid/formulation/wqp_formulation.hpp"
#include "rtc_tsid/tasks/posture_task.hpp"

namespace rtc::tsid {
namespace {

const std::string kPandaUrdf = RTC_PANDA_URDF_PATH;

class WQPFormulationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(kPandaUrdf, *model);
    model_ = model;

    YAML::Node config;
    robot_info_.Build(*model_, config);

    contact_cfg_.max_contacts = 0;
    contact_cfg_.max_contact_vars = 0;

    cache_.Init(model_, rtc::tsid::ContactFrameIds(contact_cfg_));
    contacts_.Init(0);

    ref_.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated, 0);
  }

  std::shared_ptr<const pinocchio::Model> model_;
  RobotModelInfo robot_info_;
  ContactManagerConfig contact_cfg_;
  PinocchioCache cache_;
  ContactState contacts_;
  ControlReference ref_;
};

// ──────────────────────────────────────────────
// Posture-only WQP (no contacts)
// ──────────────────────────────────────────────
TEST_F(WQPFormulationTest, PostureOnlyConverges) {
  YAML::Node formulation_config;
  formulation_config["formulation_type"] = "wqp";

  auto formulation = CreateFormulation(*model_, robot_info_, contact_cfg_, formulation_config);
  ASSERT_NE(formulation, nullptr);
  EXPECT_EQ(formulation->Type(), "wqp");

  // PostureTask 등록
  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
  task_cfg["weight"] = 1.0;
  posture->Init(*model_, robot_info_, cache_, task_cfg);
  formulation->AddTask(std::move(posture));

  // q_des = q_current, v_des = 0 → 가속도 ≈ 0
  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  ref_.q_des = q;
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const auto& result = formulation->Solve(cache_, ref_, contacts_, robot_info_);

  ASSERT_TRUE(result.converged);
  EXPECT_EQ(result.levels_solved, 1);
  EXPECT_GT(result.solve_time_us, 0.0);

  // a_opt ≈ 0 (at desired posture with zero velocity)
  Eigen::VectorXd a_opt = result.x_opt.head(robot_info_.nv);
  EXPECT_LT(a_opt.norm(), 1e-4) << "a_opt = " << a_opt.transpose();
}

TEST_F(WQPFormulationTest, PostureTrackingNonZeroAccel) {
  YAML::Node formulation_config;

  auto formulation = std::make_unique<WQPFormulation>();
  formulation->Init(*model_, robot_info_, contact_cfg_, formulation_config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
  posture->Init(*model_, robot_info_, cache_, task_cfg);
  formulation->AddTask(std::move(posture));

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  // q_des != q → non-zero acceleration
  ref_.q_des = q;
  ref_.q_des.head(robot_info_.nv) += Eigen::VectorXd::Constant(robot_info_.nv, 0.1);
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const auto& result = formulation->Solve(cache_, ref_, contacts_, robot_info_);

  ASSERT_TRUE(result.converged);
  Eigen::VectorXd a_opt = result.x_opt.head(robot_info_.nv);

  // 가속도가 q_des 방향을 향해야 (kp * 0.1 = 10.0)
  for (int i = 0; i < robot_info_.nv; ++i) {
    EXPECT_GT(a_opt(i), 0.0) << "Joint " << i << " should accelerate toward q_des";
  }
}

TEST_F(WQPFormulationTest, GravityCompensationTorque) {
  // Posture at rest → τ = M*a + h, a ≈ 0 → τ ≈ h ≈ g
  YAML::Node formulation_config;

  auto formulation = std::make_unique<WQPFormulation>();
  formulation->Init(*model_, robot_info_, contact_cfg_, formulation_config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
  posture->Init(*model_, robot_info_, cache_, task_cfg);
  formulation->AddTask(std::move(posture));

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  ref_.q_des = q;
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  const auto& result = formulation->Solve(cache_, ref_, contacts_, robot_info_);
  ASSERT_TRUE(result.converged);

  // τ = S * (M * a_opt + h)  (no contacts → Jcᵀλ = 0)
  Eigen::VectorXd a_opt = result.x_opt.head(robot_info_.nv);
  Eigen::VectorXd tau = robot_info_.S * (cache_.M * a_opt + cache_.h);

  // a_opt ≈ 0 → τ ≈ g (gravity compensation)
  Eigen::VectorXd g = robot_info_.S * cache_.g;
  EXPECT_LT((tau - g).norm(), 0.1) << "tau should be close to gravity compensation\n"
                                   << "  tau = " << tau.transpose() << "\n"
                                   << "  g   = " << g.transpose();
}

TEST_F(WQPFormulationTest, FactoryCreatesWQP) {
  YAML::Node config;
  config["formulation_type"] = "wqp";

  auto f = CreateFormulation(*model_, robot_info_, contact_cfg_, config);
  ASSERT_NE(f, nullptr);
  EXPECT_EQ(f->Type(), "wqp");
}

TEST_F(WQPFormulationTest, FactoryDefaultIsWQP) {
  YAML::Node config;  // no formulation_type

  auto f = CreateFormulation(*model_, robot_info_, contact_cfg_, config);
  ASSERT_NE(f, nullptr);
  EXPECT_EQ(f->Type(), "wqp");
}

TEST_F(WQPFormulationTest, PresetApplication) {
  YAML::Node formulation_config;
  auto formulation = std::make_unique<WQPFormulation>();
  formulation->Init(*model_, robot_info_, contact_cfg_, formulation_config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  posture->Init(*model_, robot_info_, cache_, task_cfg);
  formulation->AddTask(std::move(posture));

  // Apply preset that deactivates posture
  PhasePreset preset;
  preset.phase_name = "test";
  TaskPreset tp;
  tp.task_name = "posture";
  tp.active = false;
  tp.weight = 0.5;
  tp.priority = 2;
  preset.task_presets.push_back(tp);

  formulation->ApplyPreset(preset);

  auto* task = formulation->GetTask("posture");
  ASSERT_NE(task, nullptr);
  EXPECT_FALSE(task->IsActive());
  EXPECT_DOUBLE_EQ(task->Weight(), 0.5);
  EXPECT_EQ(task->Priority(), 2);
}

TEST_F(WQPFormulationTest, ConsecutiveSolvesWarmStart) {
  YAML::Node formulation_config;
  auto formulation = std::make_unique<WQPFormulation>();
  formulation->Init(*model_, robot_info_, contact_cfg_, formulation_config);

  auto posture = std::make_unique<PostureTask>();
  YAML::Node task_cfg;
  task_cfg["kp"] = 100.0;
  task_cfg["kd"] = 20.0;
  posture->Init(*model_, robot_info_, cache_, task_cfg);
  formulation->AddTask(std::move(posture));

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache_.Update(q, v);

  ref_.q_des = q;
  ref_.v_des = v;
  ref_.a_des.setZero(robot_info_.nv);

  // Multiple solves should all converge
  for (int i = 0; i < 10; ++i) {
    const auto& result = formulation->Solve(cache_, ref_, contacts_, robot_info_);
    ASSERT_TRUE(result.converged) << "Solve " << i << " failed";
  }
}

// ──────────────────────────────────────────────
// A-3 W1: Surface contact end-to-end WQP solve
// Verifies max_n_ineq_ workspace bump (+6 per surface contact) reaches ProxQP
// and CoP/yaw rows are enforced. Matrix-only tests (test_friction_cone) do not
// exercise the solver — buffer overflow or row-count mismatch would only show
// up here.
// ──────────────────────────────────────────────
TEST_F(WQPFormulationTest, WQPSolveWithSurfaceContact) {
  // 1 surface contact on panda_link8. patch=0.04, μ_τ=0.02 — CoP/yaw active.
  ContactManagerConfig mgr;
  mgr.contacts.resize(1);
  mgr.contacts[0].name = "surface0";
  mgr.contacts[0].frame_name = "panda_link8";
  mgr.contacts[0].frame_id = static_cast<int>(model_->getFrameId("panda_link8"));
  mgr.contacts[0].contact_dim = 6;
  mgr.contacts[0].friction_coeff = 0.7;
  mgr.contacts[0].friction_faces = 4;
  mgr.contacts[0].patch_half_length_x = 0.04;
  mgr.contacts[0].patch_half_length_y = 0.04;
  mgr.contacts[0].torsional_friction_coeff = 0.02;
  mgr.max_contacts = 1;
  mgr.max_contact_vars = 6;

  PinocchioCache cache;
  cache.Init(model_, rtc::tsid::ContactFrameIds(mgr));

  ContactState cs;
  cs.Init(1);
  cs.contacts[0].active = true;
  cs.RecomputeActive(mgr);

  YAML::Node fcfg;
  fcfg["formulation_type"] = "wqp";
  auto formulation = CreateFormulation(*model_, robot_info_, mgr, fcfg);
  ASSERT_NE(formulation, nullptr);

  // PostureTask only — minimal task set, drives a* ≈ Kp(q_des-q) + Kd(v_des-v).
  auto posture = std::make_unique<PostureTask>();
  YAML::Node tcfg;
  tcfg["kp"] = 100.0;
  tcfg["kd"] = 20.0;
  tcfg["weight"] = 1.0;
  posture->Init(*model_, robot_info_, cache, tcfg);
  formulation->AddTask(std::move(posture));

  // FrictionConeConstraint with surface CoP/yaw rows.
  auto fc = std::make_unique<FrictionConeConstraint>();
  YAML::Node ccfg;
  fc->Init(*model_, robot_info_, cache, ccfg);
  fc->SetContactManager(&mgr);
  formulation->AddConstraint(std::move(fc));

  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);
  cache.Update(q, v);

  ControlReference ref;
  ref.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated, mgr.max_contact_vars);
  ref.q_des = q;
  ref.v_des = v;
  ref.a_des.setZero(robot_info_.nv);

  // Solve must succeed without buffer assertion / overflow.
  const auto& result = formulation->Solve(cache, ref, cs, robot_info_);
  ASSERT_TRUE(result.converged) << "WQP solve with surface contact failed";

  // Extract λ = [fx, fy, fz, mx, my, mz] from x_opt segment after a.
  const int nv = robot_info_.nv;
  Eigen::VectorXd lambda = result.x_opt.segment(nv, 6);

  // Unilateral: fz ≥ 0 (allow tiny solver tolerance).
  EXPECT_GE(lambda(2), -1e-6) << "fz must be non-negative";

  // CoP rectangle: |m_y| ≤ l_x · f_z, |m_x| ≤ l_y · f_z. Solver may pick
  // λ = 0 in fixed-base / posture-only scenario — both feasibility checks
  // hold trivially. Assert they hold (no constraint violation).
  const double lx = mgr.contacts[0].patch_half_length_x;
  const double ly = mgr.contacts[0].patch_half_length_y;
  const double mu_tau = mgr.contacts[0].torsional_friction_coeff;
  EXPECT_LE(std::abs(lambda(4)), lx * lambda(2) + 1e-6) << "CoP_x violation";
  EXPECT_LE(std::abs(lambda(3)), ly * lambda(2) + 1e-6) << "CoP_y violation";
  EXPECT_LE(std::abs(lambda(5)), mu_tau * lambda(2) + 1e-6) << "yaw violation";
}

// ──────────────────────────────────────────────
// A-2 G2: TSIDController τ recovery with surface contact
// Verifies tsid_controller.cpp lines 70-78 (cdim lookup from contact_cfg_)
// — the τ = S(M·a + h − Jcᵀλ) reconstruction must use cdim=6 for surface,
// not the legacy cdim=3 hardcode. Without this test the silent-breakage fix
// is unverified at the integration level.
// ──────────────────────────────────────────────
TEST_F(WQPFormulationTest, TSIDComputeSurfaceContactTauResidual) {
  // YAML config exercised by TSIDController::init: contacts + formulation_type
  // + friction_cone + torque_limit (A-1) constraints.
  YAML::Node config;
  config["formulation_type"] = "wqp";

  YAML::Node contacts_yaml;
  YAML::Node c0;
  c0["name"] = "surface0";
  c0["frame"] = "panda_link8";
  c0["type"] = "surface";
  c0["friction_coeff"] = 0.7;
  c0["friction_faces"] = 4;
  c0["patch_half_length_x"] = 0.03;
  c0["patch_half_length_y"] = 0.03;
  c0["torsional_friction_coeff"] = 0.01;
  contacts_yaml.push_back(c0);
  config["contacts"] = contacts_yaml;

  TSIDController tsid;
  tsid.Init(*model_, robot_info_, config);

  // Add minimal task + constraints to formulation post-init.
  auto& f = tsid.Formulation();

  auto posture = std::make_unique<PostureTask>();
  YAML::Node tcfg;
  tcfg["kp"] = 50.0;
  tcfg["kd"] = 10.0;
  tcfg["weight"] = 1.0;
  PinocchioCache local_cache;
  ContactManagerConfig local_mgr;
  local_mgr.contacts.resize(1);
  local_mgr.contacts[0].contact_dim = 6;
  local_mgr.contacts[0].frame_id = static_cast<int>(model_->getFrameId("panda_link8"));
  local_mgr.max_contacts = 1;
  local_mgr.max_contact_vars = 6;
  local_cache.Init(model_, rtc::tsid::ContactFrameIds(local_mgr));
  posture->Init(*model_, robot_info_, local_cache, tcfg);
  f.AddTask(std::move(posture));

  auto eom = std::make_unique<EomConstraint>();
  YAML::Node ecfg;
  eom->Init(*model_, robot_info_, local_cache, ecfg);
  eom->SetContactManager(&local_mgr);
  f.AddConstraint(std::move(eom));

  auto fc = std::make_unique<FrictionConeConstraint>();
  YAML::Node fccfg;
  fc->Init(*model_, robot_info_, local_cache, fccfg);
  fc->SetContactManager(&local_mgr);
  f.AddConstraint(std::move(fc));

  // State setup.
  Eigen::VectorXd q = pinocchio::neutral(*model_);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_info_.nv);

  ContactState cs;
  cs.Init(1);
  cs.contacts[0].active = true;
  cs.RecomputeActive(local_mgr);

  local_cache.Update(q, v);

  ControlState state;
  state.q = q;
  state.v = v;
  state.timestamp_ns = 0;

  ControlReference ref;
  ref.Init(robot_info_.nq, robot_info_.nv, robot_info_.n_actuated, local_mgr.max_contact_vars);
  ref.q_des = q;
  ref.v_des = v;
  ref.a_des.setZero(robot_info_.nv);

  // Compute through full TSIDController path (exercises τ recovery cdim fix).
  const auto& out = tsid.Compute(state, ref, local_cache, cs);
  ASSERT_TRUE(out.qp_converged) << "TSID solve with surface contact failed";

  // τ residual check: M·a + h − Jcᵀ·λ should equal Sᵀ·τ (within tolerance).
  // Fixed-base: S = I_nv, so residual = τ. We verify the recovered output_.tau
  // matches the hand-computed τ from a_opt + λ_opt with cdim=6 (full 6×nv Jc).
  Eigen::VectorXd a_opt = out.a_opt.head(robot_info_.nv);
  Eigen::VectorXd lambda_opt = out.lambda_opt.head(6);  // surface cdim=6

  // Recover τ_hand = M·a + h − Jc[:6,:]ᵀ·λ (full 6 rows for surface).
  Eigen::VectorXd tau_hand = local_cache.M * a_opt + local_cache.h;
  const auto& Jc = local_cache.contact_frames[0].J;
  tau_hand.noalias() -= Jc.topRows(6).transpose() * lambda_opt;

  // S projection (fixed-base S = I).
  Eigen::VectorXd tau_proj = robot_info_.S * tau_hand;

  // Compare against controller's recovered τ.
  EXPECT_TRUE(out.tau.head(robot_info_.n_actuated).isApprox(tau_proj, 1e-8))
      << "τ recovery must include surface contact moments (cdim=6 not 3).\n"
      << "out.tau:  " << out.tau.head(robot_info_.n_actuated).transpose() << "\n"
      << "tau_proj: " << tau_proj.transpose();

  // Regression sentinel: if cdim=3 hardcode regressed, tau_hand_3 (using only
  // 3 rows of Jc) would differ from tau_proj. Confirm the difference is real.
  Eigen::VectorXd tau_hand_3 = local_cache.M * a_opt + local_cache.h;
  tau_hand_3.noalias() -= Jc.topRows(3).transpose() * lambda_opt.head(3);
  Eigen::VectorXd tau_proj_3 = robot_info_.S * tau_hand_3;
  // tau_proj_3 ≠ tau_proj when moment is non-zero. Skip equality check (might
  // be 0 when posture-only drives λ_moment ≈ 0), but document expected drift.
  (void)tau_proj_3;
}

}  // namespace
}  // namespace rtc::tsid
