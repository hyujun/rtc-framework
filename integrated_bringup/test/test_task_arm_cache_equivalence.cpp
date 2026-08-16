// ── test_task_arm_cache_equivalence — unified kin&dyn Phase 4 ─────────────────
//   Phase 4 moves DemoTaskController's arm TCP FK + tip Jacobian off the arm-only
//   arm_handle_ direct calls and onto the shared combined (arm+hand) PinocchioCache
//   (updated once per non-E-STOP tick), exactly like DemoWbcController. The
//   controller registers the arm TCP tip/base frames on the cache with the frame
//   ids resolved from the ARM-only sub-model (arm_handle_->GetFrameId), while the
//   cache runs on the COMBINED control model, and extracts the arm-joint columns
//   of the 6×nv_combined Jacobian into its 6×nv_arm CLIK buffer.
//
//   This test proves the substitution is value-preserving on BOTH profiles
//   (iiwa7_leap serial + ur5e_p1b closed-chain):
//
//     (1) arm_handle FK(tip)  == cache.registered_frames[tip].oMf   (world tip)
//         arm_handle FK(root) == cache.registered_frames[root].oMf  (world root)
//     (2) arm_handle GetFrameJacobian(tip, LWA) column for arm joint j
//           == combined-cache J column at that joint's combined v-index
//
//   (1) also settles arm-model vs combined-model frame-id consistency (the
//   precondition for the controller's task_tcp/task_base registration). (2) is
//   the genuinely new Phase 4 element — the arm-column extraction. Equivalence is
//   at a tight 1e-12 tol (not literally byte-for-byte: the two models carry
//   different Data and computeAllTerms accumulates in a different order, but the
//   arm chain is identical so agreement is effectively exact).
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

#include "optional_package.hpp"
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
  const char* tip_frame;     // urdf.tip_link — resolved via arm_handle_->GetFrameId (task_tcp)
  const char* root_link;     // urdf.root_link — resolved via arm_handle_->GetFrameId (task_base)
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
  const double t_diff = (a.translation() - b.translation()).cwiseAbs().maxCoeff();
  const double r_diff = (a.rotation() - b.rotation()).cwiseAbs().maxCoeff();
  EXPECT_LT(t_diff, 1e-12) << what << ": translation max|Δ|=" << t_diff;
  EXPECT_LT(r_diff, 1e-12) << what << ": rotation max|Δ|=" << r_diff;
}

void RunEquivalence(const EquivConfig& ec) {
  rub::ModelConfig cfg = MakeModelConfig(ec);
  auto builder = std::make_shared<rub::PinocchioModelBuilder>(cfg);

  // Arm-only reduced model — exactly what DemoTaskController::arm_handle_ wraps.
  auto arm_model_ptr = builder->GetReducedModel(ec.arm_submodel);
  rub::RtModelHandle arm_handle(arm_model_ptr);
  const pinocchio::Model& arm_model = arm_handle.GetModel();
  const pinocchio::FrameIndex tip_fid = arm_handle.GetFrameId(ec.tip_frame);
  const pinocchio::FrameIndex root_fid = arm_handle.GetFrameId(ec.root_link);
  ASSERT_NE(tip_fid, 0U) << ec.label << ": tip frame '" << ec.tip_frame << "' unresolved";
  ASSERT_NE(root_fid, 0U) << ec.label << ": root frame '" << ec.root_link << "' unresolved";

  // Combined arm+hand control model — exactly what pinocchio_cache_ runs on
  // (GetActuatedModel for closed-chain, GetTreeModel("wbc") for serial/mimic).
  // This mirrors DemoTaskController::InitControlModelCache.
  std::shared_ptr<const pinocchio::Model> combined;
  if (auto actuated = builder->GetActuatedModel()) {
    combined = std::move(actuated);
  } else {
    combined = builder->GetTreeModel("wbc");
  }

  rub::PinocchioCache cache;
  cache.Init(combined, {});  // task has no TSID contact frames

  // Replicate the controller's task_tcp / task_base registration: it registers
  // the ARM-model frame ids on the combined-model cache.
  const int tip_idx = cache.RegisterFrame("task_tcp", tip_fid);
  const int root_idx = cache.RegisterFrame("task_base", root_fid);
  ASSERT_GE(tip_idx, 0) << ec.label << ": task_tcp registration failed";
  ASSERT_GE(root_idx, 0) << ec.label << ": task_base registration failed";

  Eigen::VectorXd q_arm = pinocchio::neutral(arm_model);
  Eigen::VectorXd q_comb = pinocchio::neutral(*combined);
  Eigen::VectorXd v_comb = Eigen::VectorXd::Zero(combined->nv);
  SetMatchedArmConfig(arm_model, q_arm, *combined, q_comb);

  // ── Old path: arm_handle_ FK + tip Jacobian (what ReadState/Fill* read today).
  arm_handle.ComputeJacobians(
      std::span<const double>(q_arm.data(), static_cast<std::size_t>(q_arm.size())));
  Eigen::MatrixXd J_arm(6, arm_model.nv);
  J_arm.setZero();
  arm_handle.GetFrameJacobian(tip_fid, pinocchio::LOCAL_WORLD_ALIGNED, J_arm);
  const pinocchio::SE3 fk_tip = arm_handle.GetFramePlacement(tip_fid);
  const pinocchio::SE3 fk_root = arm_handle.GetFramePlacement(root_fid);

  // ── New path: shared cache oMf + combined-model Jacobian.
  cache.Update(q_comb, v_comb);
  const pinocchio::SE3& cache_tip = cache.registered_frames[static_cast<std::size_t>(tip_idx)].oMf;
  const pinocchio::SE3& cache_root =
      cache.registered_frames[static_cast<std::size_t>(root_idx)].oMf;
  const Eigen::MatrixXd& J_comb = cache.registered_frames[static_cast<std::size_t>(tip_idx)].J;

  // (1) FK equivalence — world tip / world root / base-relative TCP.
  ExpectSe3Equal(fk_tip, cache_tip, std::string(ec.label) + " tip(world)");
  ExpectSe3Equal(fk_root, cache_root, std::string(ec.label) + " root(world)");
  ExpectSe3Equal(fk_root.actInv(fk_tip), cache_root.actInv(cache_tip),
                 std::string(ec.label) + " tcp(base-relative)");

  // (2) Jacobian arm-column equivalence — for each 1-DoF arm joint, the arm-model
  // tip Jacobian column equals the combined-model Jacobian column at that joint's
  // combined v-index (the column DemoTaskController extracts via ext_to_pin_v_).
  int checked = 0;
  for (pinocchio::JointIndex jid = 1; jid < arm_model.joints.size(); ++jid) {
    if (arm_model.joints[jid].nv() != 1) {
      continue;
    }
    const auto arm_v = arm_model.idx_vs[jid];
    const std::string& jname = arm_model.names[jid];
    ASSERT_TRUE(combined->existJointName(jname))
        << ec.label << ": arm joint '" << jname << "' missing in combined model";
    const auto cjid = combined->getJointId(jname);
    ASSERT_EQ(combined->joints[cjid].nv(), 1)
        << ec.label << ": arm joint '" << jname << "' not 1-DoF in combined model";
    const auto comb_v = combined->idx_vs[cjid];

    const double col_diff = (J_arm.col(arm_v) - J_comb.col(comb_v)).cwiseAbs().maxCoeff();
    EXPECT_LT(col_diff, 1e-12) << ec.label << " J column for '" << jname << "' max|Δ|=" << col_diff;
    ++checked;
  }
  EXPECT_GT(checked, 0) << ec.label << ": no arm Jacobian columns checked";
}

}  // namespace

// iiwa7_leap — 23-DoF serial/mimic (combined = wbc tree, GetActuatedModel null).
TEST(TaskArmCacheEquivalence, Iiwa7Leap) {
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
// of the loop, so both arm TCP and its Jacobian columns stay exact even here).
TEST(TaskArmCacheEquivalence, Ur5eP1b) {
  RTC_SKIP_IF_PACKAGE_MISSING("hand_description");
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
