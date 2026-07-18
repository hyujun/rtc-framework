// ── test_joint_arm_cache_equivalence — unified kin&dyn Phase 5 ────────────────
//   Phase 5 moves DemoJointController's arm TCP FK off the arm-only arm_handle_
//   direct calls and onto the shared combined (arm+hand) PinocchioCache (updated
//   once per non-E-STOP tick), exactly like DemoWbcController / DemoTaskController.
//   The controller registers the arm TCP tip/base frames on the cache with the
//   frame ids resolved from the ARM-only sub-model (arm_handle_->GetFrameId),
//   while the cache runs on the COMBINED control model. Joint is FK-only — it has
//   no Jacobian consumer — so this proves only the FK substitution:
//
//     arm_handle FK(tip)  == cache.registered_frames[tip].oMf   (world tip)
//     arm_handle FK(root) == cache.registered_frames[root].oMf  (world root)
//
//   for identical arm joint configs (hand at neutral), on BOTH the serial
//   (iiwa7_leap) and closed-chain (ur5e_p1b) profiles. This is the joint-profile
//   counterpart of test_task_arm_cache_equivalence / test_wbc_arm_tcp_cache_
//   equivalence: the model selection + arm-model-frame-id registration mechanism
//   is shared, so the tight 1e-12 agreement is expected (the arm chain is
//   identical in both models; the two carry different Data but the same tree).
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

#include "rtc_urdf_bridge/pinocchio_cache.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "rtc_urdf_bridge/types.hpp"

namespace rub = rtc_urdf_bridge;

namespace {

struct EquivConfig {
  const char* label;
  const char* urdf_pkg;
  const char* urdf_rel;
  bool extended;
  const char* closure_rel;   // pkg-relative; only used when extended
  const char* arm_submodel;  // GetReducedModel(name) — the model arm_handle_ wraps
  const char* tip_frame;     // urdf.tip_link — resolved via arm_handle_->GetFrameId (arm_tcp)
  const char* root_link;     // urdf.root_link — resolved via arm_handle_->GetFrameId (arm_base)
  std::array<const char*, 4> wbc_fingertips;  // combined control-tree tip frames
};

rub::ModelConfig MakeModelConfig(const EquivConfig& ec) {
  rub::ModelConfig cfg;
  cfg.urdf_path = ament_index_cpp::get_package_share_directory(ec.urdf_pkg) + "/" + ec.urdf_rel;
  cfg.root_joint_type = "fixed";
  if (ec.extended) {
    cfg.closure_yaml_path =
        ament_index_cpp::get_package_share_directory(ec.urdf_pkg) + "/" + ec.closure_rel;
  }
  cfg.sub_models.push_back({ec.arm_submodel, ec.root_link, ec.tip_frame});
  cfg.tree_models.push_back(
      {"wbc",
       ec.root_link,
       {ec.wbc_fingertips[0], ec.wbc_fingertips[1], ec.wbc_fingertips[2], ec.wbc_fingertips[3]}});
  return cfg;
}

// Set a deterministic, non-trivial arm config in BOTH models, mapped by joint
// name so it is order-independent. Hand joints stay at neutral.
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
  const double t_diff = (a.translation() - b.translation()).cwiseAbs().maxCoeff();
  const double r_diff = (a.rotation() - b.rotation()).cwiseAbs().maxCoeff();
  EXPECT_LT(t_diff, 1e-12) << what << ": translation max|Δ|=" << t_diff;
  EXPECT_LT(r_diff, 1e-12) << what << ": rotation max|Δ|=" << r_diff;
}

void RunEquivalence(const EquivConfig& ec) {
  rub::ModelConfig cfg = MakeModelConfig(ec);
  auto builder = std::make_shared<rub::PinocchioModelBuilder>(cfg);

  // Arm-only reduced model — exactly what DemoJointController::arm_handle_ wraps.
  auto arm_model_ptr = builder->GetReducedModel(ec.arm_submodel);
  rub::RtModelHandle arm_handle(arm_model_ptr);
  const pinocchio::Model& arm_model = arm_handle.GetModel();
  const pinocchio::FrameIndex tip_fid = arm_handle.GetFrameId(ec.tip_frame);
  const pinocchio::FrameIndex root_fid = arm_handle.GetFrameId(ec.root_link);
  ASSERT_NE(tip_fid, 0U) << ec.label << ": tip frame '" << ec.tip_frame << "' unresolved";
  ASSERT_NE(root_fid, 0U) << ec.label << ": root frame '" << ec.root_link << "' unresolved";

  // Combined arm+hand control model — exactly what pinocchio_cache_ runs on
  // (mirrors DemoJointController::InitControlModelCache).
  std::shared_ptr<const pinocchio::Model> combined;
  if (auto actuated = builder->GetActuatedModel()) {
    combined = std::move(actuated);
  } else {
    combined = builder->GetTreeModel("wbc");
  }

  rub::PinocchioCache cache;
  cache.Init(combined, {});  // joint has no TSID contact frames

  // Replicate the controller's arm_tcp / arm_base registration: it registers the
  // ARM-model frame ids on the combined-model cache.
  const int tip_idx = cache.RegisterFrame("arm_tcp", tip_fid);
  const int root_idx = cache.RegisterFrame("arm_base", root_fid);
  ASSERT_GE(tip_idx, 0) << ec.label << ": arm_tcp registration failed";
  ASSERT_GE(root_idx, 0) << ec.label << ": arm_base registration failed";

  Eigen::VectorXd q_arm = pinocchio::neutral(arm_model);
  Eigen::VectorXd q_comb = pinocchio::neutral(*combined);
  Eigen::VectorXd v_comb = Eigen::VectorXd::Zero(combined->nv);
  SetMatchedArmConfig(arm_model, q_arm, *combined, q_comb);

  // Old path: arm_handle_ FK (what ComputeControl cached into arm_tcp_pose_).
  arm_handle.ComputeForwardKinematics(
      std::span<const double>(q_arm.data(), static_cast<std::size_t>(q_arm.size())));
  const pinocchio::SE3 fk_tip = arm_handle.GetFramePlacement(tip_fid);
  const pinocchio::SE3 fk_root = arm_handle.GetFramePlacement(root_fid);

  // New path: shared cache oMf.
  cache.Update(q_comb, v_comb);
  const pinocchio::SE3& cache_tip = cache.registered_frames[static_cast<std::size_t>(tip_idx)].oMf;
  const pinocchio::SE3& cache_root =
      cache.registered_frames[static_cast<std::size_t>(root_idx)].oMf;

  ExpectSe3Equal(fk_tip, cache_tip, std::string(ec.label) + " tip(world)");
  ExpectSe3Equal(fk_root, cache_root, std::string(ec.label) + " root(world)");
  // Base-relative TCP (arm_tcp_pose_ when use_root_frame_): both paths must agree
  // after the identical actInv composition.
  ExpectSe3Equal(fk_root.actInv(fk_tip), cache_root.actInv(cache_tip),
                 std::string(ec.label) + " tcp(base-relative)");
}

}  // namespace

// iiwa7_leap — 23-DoF serial/mimic (combined = wbc tree, GetActuatedModel null).
TEST(JointArmCacheEquivalence, Iiwa7Leap) {
  RunEquivalence({/*label=*/"iiwa7_leap",
                  /*urdf_pkg=*/"robot_descriptions",
                  /*urdf_rel=*/"robots/iiwa7_leap/urdf/iiwa7_with_leap_right.urdf.xacro",
                  /*extended=*/false,
                  /*closure_rel=*/"",
                  /*arm_submodel=*/"iiwa7",
                  /*tip_frame=*/"ee_link",
                  /*root_link=*/"link_0",
                  /*wbc_fingertips=*/
                  {{"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"}}});
}

// ur5e_p1b — 16-DoF closed-chain (combined = GetActuatedModel; arm tip is upstream
// of the loop, so the arm TCP stays exact even here).
TEST(JointArmCacheEquivalence, Ur5eP1b) {
  RunEquivalence({/*label=*/"ur5e_p1b",
                  /*urdf_pkg=*/"hand_description",
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
