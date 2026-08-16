// ── test_wbc_arm_tcp_cache_equivalence — unified kin&dyn Phase 2 ──────────────
//   Phase 2 replaces the WBC controller's per-tick arm TCP FK recompute
//   (FillLogOutput/FillPublishOutput/ComputeTcpError via arm_handle_) with a
//   read of the shared PinocchioCache registered-frame oMf. The controller
//   registers clik_tcp/clik_base on the cache with the frame ids resolved from
//   the ARM-only sub-model (arm_handle_->GetFrameId), while the cache runs on
//   the COMBINED arm+hand control model. This test proves the substitution is
//   value-preserving:
//
//     arm_handle FK(tip)  ==  cache.registered_frames[tip].oMf   (world tip)
//     arm_handle FK(root) ==  cache.registered_frames[root].oMf  (world root)
//
//   for identical arm joint configs (hand at neutral). The arm tip/root lie
//   upstream of any hand loop closure, so equivalence is expected byte-for-byte
//   (tight tol) on BOTH the serial (iiwa7_leap) and closed-chain (ur5e_p1b)
//   configs — this also empirically settles arm-model vs combined-model
//   frame-id consistency, the precondition for the controller's existing
//   clik_tcp registration to be correct.
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <span>
#include <string>
#include <vector>

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

struct EquivConfig {
  const char* label;
  const char* urdf_rel;
  bool extended;
  const char* closure_rel;   // share-relative; only used when extended
  const char* arm_submodel;  // GetReducedModel(name) — the model the controller's arm_handle_ uses
  const char* tip_frame;     // urdf.tip_link — resolved via arm_handle_->GetFrameId (clik_tcp)
  const char* root_link;     // urdf.root_link — resolved via arm_handle_->GetFrameId (clik_base)
  std::array<const char*, 4> wbc_fingertips;  // wbc control-tree tip frames
};

// Every fixture model lives in `robot_descriptions` — the package name is a
// literal, not a config field, so validate_test_fixtures.py can prove where
// this points instead of only checking that a skip guard exists (#457).
rub::ModelConfig MakeModelConfig(const EquivConfig& ec) {
  rub::ModelConfig cfg;
  cfg.urdf_path =
      ament_index_cpp::get_package_share_directory("robot_descriptions") + "/" + ec.urdf_rel;
  cfg.root_joint_type = "fixed";
  if (ec.extended) {
    cfg.closure_yaml_path =
        ament_index_cpp::get_package_share_directory("robot_descriptions") + "/" + ec.closure_rel;
  }
  cfg.sub_models.push_back({ec.arm_submodel, ec.root_link, ec.tip_frame});
  cfg.tree_models.push_back(
      {"wbc",
       ec.root_link,
       {ec.wbc_fingertips[0], ec.wbc_fingertips[1], ec.wbc_fingertips[2], ec.wbc_fingertips[3]}});
  return cfg;
}

// Set a deterministic, non-trivial arm config in BOTH models, mapped by joint
// name so it is order-independent. Hand joints stay at neutral. Only 1-DoF
// (revolute/prismatic) joints are perturbed — every arm joint here qualifies.
void SetMatchedArmConfig(const pinocchio::Model& arm_model, Eigen::VectorXd& q_arm,
                         const pinocchio::Model& combined, Eigen::VectorXd& q_comb) {
  for (pinocchio::JointIndex jid = 1; jid < arm_model.joints.size(); ++jid) {
    if (arm_model.joints[jid].nq() != 1) {
      continue;
    }
    const double val = 0.13 * static_cast<double>(jid) - 0.2;
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
  // Tight tol: the arm chain is identical in both models, so pinocchio composes
  // the same joint transforms in the same tree order → effectively exact.
  const double t_diff = (a.translation() - b.translation()).cwiseAbs().maxCoeff();
  const double r_diff = (a.rotation() - b.rotation()).cwiseAbs().maxCoeff();
  EXPECT_LT(t_diff, 1e-12) << what << ": translation max|Δ|=" << t_diff;
  EXPECT_LT(r_diff, 1e-12) << what << ": rotation max|Δ|=" << r_diff;
}

void RunEquivalence(const EquivConfig& ec) {
  rub::ModelConfig cfg = MakeModelConfig(ec);
  auto builder = std::make_shared<rub::PinocchioModelBuilder>(cfg);

  // Arm-only reduced model — exactly what DemoWbcController::arm_handle_ wraps.
  auto arm_model = builder->GetReducedModel(ec.arm_submodel);
  rub::RtModelHandle arm_handle(arm_model);
  const pinocchio::FrameIndex tip_fid = arm_handle.GetFrameId(ec.tip_frame);
  const pinocchio::FrameIndex root_fid = arm_handle.GetFrameId(ec.root_link);
  ASSERT_NE(tip_fid, 0U) << ec.label << ": tip frame '" << ec.tip_frame << "' unresolved";
  ASSERT_NE(root_fid, 0U) << ec.label << ": root frame '" << ec.root_link << "' unresolved";

  // Combined arm+hand control model — exactly what pinocchio_cache_ runs on
  // (GetActuatedModel for closed-chain, GetTreeModel("wbc") for serial/mimic).
  std::shared_ptr<const pinocchio::Model> combined;
  if (auto actuated = builder->GetActuatedModel()) {
    combined = std::move(actuated);
  } else {
    combined = builder->GetTreeModel("wbc");
  }

  rtc::tsid::PinocchioCache cache;
  rtc::tsid::ContactManagerConfig contact_cfg;  // no contacts
  cache.Init(combined, rtc::tsid::ContactFrameIds(contact_cfg));

  // Replicate the controller's clik_tcp / clik_base registration EXACTLY: it
  // registers the ARM-model frame ids (tip_frame_id_ / root_frame_id_) on the
  // combined-model cache (controller.cpp InitClik). If the arm-model id did not
  // address the same physical frame in the combined model, the cache oMf would
  // diverge here — this is the frame-id-consistency check.
  const int tip_idx = cache.RegisterFrame("clik_tcp", tip_fid);
  const int root_idx = cache.RegisterFrame("clik_base", root_fid);
  ASSERT_GE(tip_idx, 0) << ec.label << ": clik_tcp registration failed";
  ASSERT_GE(root_idx, 0) << ec.label << ": clik_base registration failed";

  Eigen::VectorXd q_arm = pinocchio::neutral(arm_handle.GetModel());
  Eigen::VectorXd q_comb = pinocchio::neutral(*combined);
  Eigen::VectorXd v_comb = Eigen::VectorXd::Zero(combined->nv);
  SetMatchedArmConfig(arm_handle.GetModel(), q_arm, *combined, q_comb);

  // Old path: arm_handle_ FK (what FillTaskPosePods / ComputeTcpError read today).
  arm_handle.ComputeForwardKinematics(
      std::span<const double>(q_arm.data(), static_cast<std::size_t>(q_arm.size())));
  const pinocchio::SE3 fk_tip = arm_handle.GetFramePlacement(tip_fid);
  const pinocchio::SE3 fk_root = arm_handle.GetFramePlacement(root_fid);

  // New path: shared cache oMf (what Phase 2 makes them read).
  cache.Update(q_comb, v_comb);
  const pinocchio::SE3& cache_tip = cache.registered_frames[static_cast<std::size_t>(tip_idx)].oMf;
  const pinocchio::SE3& cache_root =
      cache.registered_frames[static_cast<std::size_t>(root_idx)].oMf;

  ExpectSe3Equal(fk_tip, cache_tip, std::string(ec.label) + " tip(world)");
  ExpectSe3Equal(fk_root, cache_root, std::string(ec.label) + " root(world)");

  // Base-relative TCP (the value FillTaskPosePods publishes when use_root_frame_):
  // both paths must agree after the identical actInv composition.
  const pinocchio::SE3 rel_old = fk_root.actInv(fk_tip);
  const pinocchio::SE3 rel_new = cache_root.actInv(cache_tip);
  ExpectSe3Equal(rel_old, rel_new, std::string(ec.label) + " tcp(base-relative)");
}

}  // namespace

// iiwa7_leap — 23-DoF serial/mimic (combined = wbc tree, GetActuatedModel null).
TEST(WbcArmTcpCacheEquivalence, Iiwa7Leap) {
  RunEquivalence({/*label=*/"iiwa7_leap",
                  /*urdf_rel=*/"robots/iiwa7_leap/urdf/iiwa7_with_leap_right.urdf.xacro",
                  /*extended=*/false,
                  /*closure_rel=*/"",
                  /*arm_submodel=*/"iiwa7",
                  /*tip_frame=*/"ee_link",
                  /*root_link=*/"link_0",
                  /*wbc_fingertips=*/
                  {{"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"}}});
}

// ur5e_p1b — 16-DoF closed-chain (combined = GetActuatedModel; arm tip is
// upstream of the loop, so arm TCP stays byte-for-byte even here).
TEST(WbcArmTcpCacheEquivalence, Ur5eP1b) {
  RunEquivalence({/*label=*/"ur5e_p1b",
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
