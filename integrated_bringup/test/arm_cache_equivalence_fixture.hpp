// Shared pieces of the arm-TCP cache-equivalence suites (unified kin&dyn
// Phases 2, 4, 5): test_wbc_arm_tcp_cache_equivalence,
// test_task_arm_cache_equivalence, test_joint_arm_cache_equivalence. Each
// proves that its controller's arm_handle_ reads and the combined-model
// PinocchioCache it moved onto agree; the profiles, the model config, the
// matched joint configs and the 1e-12 comparison are the same for all three.

#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/types.hpp"

namespace integrated_bringup::testfx {

struct EquivConfig {
  const char* label;
  const char* urdf_rel;
  bool extended;
  const char* closure_rel;   // share-relative; only used when extended
  const char* arm_submodel;  // GetReducedModel(name) — the model arm_handle_ wraps
  const char* tip_frame;     // urdf.tip_link — resolved via arm_handle_->GetFrameId
  const char* root_link;     // urdf.root_link — resolved via arm_handle_->GetFrameId
  std::array<const char*, 4> wbc_fingertips;  // combined control-tree tip frames
};

/// iiwa7_leap — 23-DoF serial/mimic (combined = wbc tree, GetActuatedModel null).
inline const EquivConfig kIiwa7LeapEquiv{
    /*label=*/"iiwa7_leap",
    /*urdf_rel=*/"robots/iiwa7_leap/urdf/iiwa7_with_leap_right.urdf.xacro",
    /*extended=*/false,
    /*closure_rel=*/"",
    /*arm_submodel=*/"iiwa7",
    /*tip_frame=*/"ee_link",
    /*root_link=*/"link_0",
    /*wbc_fingertips=*/{{"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"}}};

/// ur5e_p1b — 16-DoF closed-chain (combined = GetActuatedModel; the arm tip is
/// upstream of the loop).
inline const EquivConfig kUr5eP1bEquiv{
    /*label=*/"ur5e_p1b",
    /*urdf_rel=*/"robots/ur5e_p1b/urdf/ur5e_with_proto_1b.urdf.xacro",
    /*extended=*/true,
    /*closure_rel=*/"robots/ur5e_p1b/urdf/ur5e_with_proto_1b.closure.yaml",
    /*arm_submodel=*/"ur5e",
    /*tip_frame=*/"tool0",
    /*root_link=*/"base",
    /*wbc_fingertips=*/
    {{"l_thumb_tip_bracket", "l_index_tip_bracket", "l_middle_tip_bracket", "l_ring_tip_bracket"}}};

// Every fixture model lives in `robot_descriptions` — the package name is a
// literal, not a config field, so validate_test_fixtures.py can prove where
// this points instead of only checking that a skip guard exists (#457).
inline rtc_urdf_bridge::ModelConfig MakeModelConfig(const EquivConfig& ec) {
  rtc_urdf_bridge::ModelConfig cfg;
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

/// The combined arm+hand control model pinocchio_cache_ runs on:
/// GetActuatedModel for closed-chain, GetTreeModel("wbc") for serial/mimic —
/// the selection each controller's InitControlModelCache makes.
inline std::shared_ptr<const pinocchio::Model> CombinedControlModel(
    const rtc_urdf_bridge::PinocchioModelBuilder& builder) {
  if (auto actuated = builder.GetActuatedModel()) {
    return actuated;
  }
  return builder.GetTreeModel("wbc");
}

// Set a deterministic, non-trivial arm config in BOTH models, mapped by joint
// name so it is order-independent. Hand joints stay at neutral. Only 1-DoF
// (revolute/prismatic) joints are perturbed — every arm joint here qualifies.
inline void SetMatchedArmConfig(const pinocchio::Model& arm_model, Eigen::VectorXd& q_arm,
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

inline void ExpectSe3Equal(const pinocchio::SE3& a, const pinocchio::SE3& b,
                           const std::string& what) {
  // Tight tol: the arm chain is identical in both models, so pinocchio composes
  // the same joint transforms in the same tree order → effectively exact.
  const double t_diff = (a.translation() - b.translation()).cwiseAbs().maxCoeff();
  const double r_diff = (a.rotation() - b.rotation()).cwiseAbs().maxCoeff();
  EXPECT_LT(t_diff, 1e-12) << what << ": translation max|Δ|=" << t_diff;
  EXPECT_LT(r_diff, 1e-12) << what << ": rotation max|Δ|=" << r_diff;
}

}  // namespace integrated_bringup::testfx
