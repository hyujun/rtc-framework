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

#include "arm_cache_equivalence_fixture.hpp"
#include "rtc_tsid/types/wbc_types.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "rtc_urdf_bridge/types.hpp"

namespace rub = rtc_urdf_bridge;

namespace {

using integrated_bringup::testfx::CombinedControlModel;
using integrated_bringup::testfx::EquivConfig;
using integrated_bringup::testfx::ExpectSe3Equal;
using integrated_bringup::testfx::MakeModelConfig;
using integrated_bringup::testfx::SetMatchedArmConfig;

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
  const std::shared_ptr<const pinocchio::Model> combined = CombinedControlModel(*builder);

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
  RunEquivalence(integrated_bringup::testfx::kIiwa7LeapEquiv);
}

// ur5e_p1b — 16-DoF closed-chain (combined = GetActuatedModel; arm tip is
// upstream of the loop, so arm TCP stays byte-for-byte even here).
TEST(WbcArmTcpCacheEquivalence, Ur5eP1b) {
  RunEquivalence(integrated_bringup::testfx::kUr5eP1bEquiv);
}
