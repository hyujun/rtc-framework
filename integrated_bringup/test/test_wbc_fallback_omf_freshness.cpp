// ── test_wbc_fallback_omf_freshness — unified kin&dyn Task B ──────────────────
//
//   Task B (옵션 B) relocates ExtractFullState + pinocchio_cache_.Update from
//   ComputeWbcCommon (which only runs on TSID-routing phases) up into
//   ComputeControl, so the cache is refreshed on EVERY non-E-STOP tick — kFallback
//   and every non-TSID tick included. Before Task B the WBC arm/contact oMf froze
//   at the last TSID tick's value whenever the FSM sat in kFallback; after Task B
//   the pose tracks measured (command is still held by ComputeFallback).
//
//   This test proves the SEMANTIC the relocation delivers, at the cache level (the
//   existing test harness has no fixture that drives DemoWbcController::Compute with
//   a real, tsid_initialized_ model): the registered clik_tcp/clik_base oMf TRACK
//   the measured configuration across consecutive per-tick Update() calls — they are
//   NOT frozen — and each per-tick snapshot equals the arm_handle_ FK for that exact
//   configuration (1e-12, byte-for-byte with the pre-Task-B TSID-tick value). This
//   mirrors the exact registration the controller performs (arm-model frame ids on
//   the combined-model cache; see test_wbc_arm_tcp_cache_equivalence).
//
//   The controller-level "arm command stays held across a kFallback measured change"
//   half is covered by WbcFSMTest.FallbackHoldsArmCommandAcrossMeasuredChange in
//   test_demo_wbc_controller.cpp.
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <array>
#include <span>
#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/types/wbc_types.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "rtc_urdf_bridge/types.hpp"

namespace rub = rtc_urdf_bridge;

namespace {

struct FreshnessConfig {
  const char* label;
  const char* urdf_rel;
  bool extended;
  const char* closure_rel;
  const char* arm_submodel;
  const char* tip_frame;
  const char* root_link;
  std::array<const char*, 4> wbc_fingertips;
};

// Every fixture model lives in `robot_descriptions` — the package name is a
// literal, not a config field, so validate_test_fixtures.py can prove where
// this points instead of only checking that a skip guard exists (#457).
rub::ModelConfig MakeModelConfig(const FreshnessConfig& fc) {
  rub::ModelConfig cfg;
  cfg.urdf_path =
      ament_index_cpp::get_package_share_directory("robot_descriptions") + "/" + fc.urdf_rel;
  cfg.root_joint_type = "fixed";
  if (fc.extended) {
    cfg.closure_yaml_path =
        ament_index_cpp::get_package_share_directory("robot_descriptions") + "/" + fc.closure_rel;
  }
  cfg.sub_models.push_back({fc.arm_submodel, fc.root_link, fc.tip_frame});
  cfg.tree_models.push_back(
      {"wbc",
       fc.root_link,
       {fc.wbc_fingertips[0], fc.wbc_fingertips[1], fc.wbc_fingertips[2], fc.wbc_fingertips[3]}});
  return cfg;
}

// Deterministic, non-trivial arm config keyed by joint name so it is order- and
// model-independent. `base` shifts every joint by a constant so two distinct
// calls produce two clearly different poses. Only 1-DoF joints are perturbed.
void SetArmConfig(const pinocchio::Model& arm_model, Eigen::VectorXd& q_arm,
                  const pinocchio::Model& combined, Eigen::VectorXd& q_comb, double base) {
  for (pinocchio::JointIndex jid = 1; jid < arm_model.joints.size(); ++jid) {
    if (arm_model.joints[jid].nq() != 1) {
      continue;
    }
    const double val = base + 0.11 * static_cast<double>(jid);
    q_arm[arm_model.joints[jid].idx_q()] = val;
    const std::string& jname = arm_model.names[jid];
    if (combined.existJointName(jname)) {
      const auto cjid = combined.getJointId(jname);
      if (combined.joints[cjid].nq() == 1) {
        q_comb[combined.joints[cjid].idx_q()] = val;
      }
    }
  }
}

void ExpectSe3Equal(const pinocchio::SE3& a, const pinocchio::SE3& b, const std::string& what) {
  const double t_diff = (a.translation() - b.translation()).cwiseAbs().maxCoeff();
  const double r_diff = (a.rotation() - b.rotation()).cwiseAbs().maxCoeff();
  EXPECT_LT(t_diff, 1e-12) << what << ": translation max|Δ|=" << t_diff;
  EXPECT_LT(r_diff, 1e-12) << what << ": rotation max|Δ|=" << r_diff;
}

void RunFreshness(const FreshnessConfig& fc) {
  rub::ModelConfig cfg = MakeModelConfig(fc);
  auto builder = std::make_shared<rub::PinocchioModelBuilder>(cfg);

  // Arm-only reduced model (what arm_handle_ wraps) — the ground-truth FK per config.
  auto arm_model = builder->GetReducedModel(fc.arm_submodel);
  rub::RtModelHandle arm_handle(arm_model);
  const pinocchio::FrameIndex tip_fid = arm_handle.GetFrameId(fc.tip_frame);
  const pinocchio::FrameIndex root_fid = arm_handle.GetFrameId(fc.root_link);
  ASSERT_NE(tip_fid, 0U) << fc.label << ": tip frame '" << fc.tip_frame << "' unresolved";
  ASSERT_NE(root_fid, 0U) << fc.label << ": root frame '" << fc.root_link << "' unresolved";

  // Combined arm+hand control model (what pinocchio_cache_ runs on).
  std::shared_ptr<const pinocchio::Model> combined;
  if (auto actuated = builder->GetActuatedModel()) {
    combined = std::move(actuated);
  } else {
    combined = builder->GetTreeModel("wbc");
  }

  rtc::tsid::PinocchioCache cache;
  rtc::tsid::ContactManagerConfig contact_cfg;  // no contacts
  cache.Init(combined, rtc::tsid::ContactFrameIds(contact_cfg));
  const int tip_idx = cache.RegisterFrame("clik_tcp", tip_fid);
  const int root_idx = cache.RegisterFrame("clik_base", root_fid);
  ASSERT_GE(tip_idx, 0) << fc.label << ": clik_tcp registration failed";
  ASSERT_GE(root_idx, 0) << fc.label << ": clik_base registration failed";

  Eigen::VectorXd v_comb = Eigen::VectorXd::Zero(combined->nv);

  auto snapshot = [&](double base, pinocchio::SE3& out_tip, pinocchio::SE3& out_root) {
    Eigen::VectorXd q_arm = pinocchio::neutral(arm_handle.GetModel());
    Eigen::VectorXd q_comb = pinocchio::neutral(*combined);
    SetArmConfig(arm_handle.GetModel(), q_arm, *combined, q_comb, base);

    // Per-tick cache Update — exactly what ComputeControl now runs every tick.
    cache.Update(q_comb, v_comb);
    // Copy by value: registered_frames[].oMf is a reference into the cache and is
    // overwritten by the next Update. Freezing vs. tracking is the whole point.
    out_tip = cache.registered_frames[static_cast<std::size_t>(tip_idx)].oMf;
    out_root = cache.registered_frames[static_cast<std::size_t>(root_idx)].oMf;

    // Ground truth for this config: arm_handle_ FK (the pre-Task-B TSID-tick value).
    arm_handle.ComputeForwardKinematics(
        std::span<const double>(q_arm.data(), static_cast<std::size_t>(q_arm.size())));
    ExpectSe3Equal(arm_handle.GetFramePlacement(tip_fid), out_tip,
                   std::string(fc.label) + " tip(world) @base=" + std::to_string(base));
    ExpectSe3Equal(arm_handle.GetFramePlacement(root_fid), out_root,
                   std::string(fc.label) + " root(world) @base=" + std::to_string(base));
  };

  // Two consecutive per-tick Updates with DISTINCT measured configs. This is the
  // kFallback scenario post-Task-B: the cache keeps refreshing even though no TSID
  // tick ran between them.
  pinocchio::SE3 tip_a;
  pinocchio::SE3 root_a;
  pinocchio::SE3 tip_b;
  pinocchio::SE3 root_b;
  snapshot(-0.20, tip_a, root_a);
  snapshot(0.35, tip_b, root_b);

  // Freshness: the tip oMf MOVED between the two per-tick Updates (not frozen).
  // The root is fixed base → stays put (sanity: no spurious motion). Distinct arm
  // configs place the tip cm-apart, well above any numerical noise floor.
  const double tip_shift = (tip_b.translation() - tip_a.translation()).norm();
  EXPECT_GT(tip_shift, 1e-3) << fc.label
                             << ": tip oMf did not track measured (frozen?) shift=" << tip_shift;
  const double root_shift = (root_b.translation() - root_a.translation()).norm();
  EXPECT_LT(root_shift, 1e-12) << fc.label << ": fixed base root moved, shift=" << root_shift;
}

}  // namespace

// iiwa7_leap — 23-DoF serial/mimic (combined = wbc tree, GetActuatedModel null).
TEST(WbcFallbackOmfFreshness, Iiwa7Leap) {
  RunFreshness({/*label=*/"iiwa7_leap",
                /*urdf_rel=*/"robots/iiwa7_leap/urdf/iiwa7_with_leap_right.urdf.xacro",
                /*extended=*/false,
                /*closure_rel=*/"",
                /*arm_submodel=*/"iiwa7",
                /*tip_frame=*/"ee_link",
                /*root_link=*/"link_0",
                /*wbc_fingertips=*/
                {{"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"}}});
}

// ur5e_p1b — 16-DoF closed-chain (combined = GetActuatedModel). Arm tip is
// upstream of the loop, so the arm TCP tracks measured identically to serial.
TEST(WbcFallbackOmfFreshness, Ur5eP1b) {
  RunFreshness({/*label=*/"ur5e_p1b",
                /*urdf_rel=*/"robots/ur5e_p1b/urdf/ur5e_with_proto_1b.urdf.xacro",
                /*extended=*/true,
                /*closure_rel=*/"robots/ur5e_p1b/urdf/ur5e_with_proto_1b.closure.yaml",
                /*arm_submodel=*/"ur5e",
                /*tip_frame=*/"tool0",
                /*root_link=*/"base",
                /*wbc_fingertips=*/
                {{"l_thumb_tip_bracket", "l_index_tip_bracket", "l_middle_tip_bracket",
                  "l_ring_tip_bracket"}}});
}
