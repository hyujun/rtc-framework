// ── closed_chain_model 구현 ──────────────────────────────────────────────────
#include "rtc_urdf_bridge/closed_chain_model.hpp"

#include "rtc_urdf_bridge/constraint_builder.hpp"
#include "rtc_urdf_bridge/inertial_validation.hpp"
#include "rtc_urdf_bridge/loop_projection.hpp"
#include "rtc_urdf_bridge/loop_verification.hpp"
#include "rtc_urdf_bridge/urdf_logging.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joints.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <stdexcept>
#include <string>

namespace rtc_urdf_bridge {

namespace {
auto logger() {
  return ::rtc::urdf::logging::BuilderLogger();
}
}  // namespace

ClosedChainData BuildClosedChainData(const pinocchio::Model& model, const ClosureSpec& spec) {
  ClosedChainData out;

  // (1) RigidConstraintModel 생성 (§4b joint-frame placement)
  out.constraints = BuildRigidConstraints(model, spec.closures);

  // (2) actuated joint 이름 → JointIndex (actuation metadata, 구속과 분리)
  out.actuated_joint_ids.reserve(spec.actuated_joints.size());
  for (const auto& jname : spec.actuated_joints) {
    if (!model.existJointName(jname)) {
      RCLCPP_ERROR(logger(), "actuated joint 를 찾을 수 없습니다: %s", jname.c_str());
      throw std::runtime_error("closed_chain_model: actuated joint 미존재: " + jname);
    }
    out.actuated_joint_ids.push_back(model.getJointId(jname));
  }

  // (3) q_ref: neutral → loop-consistent 사영. strict 수렴 실패여도 residual 이 acceptance
  //     (기본 1e-6 m) 이내면 q_ref 는 사용 가능 (#250) — URDF 좌표 불일치의 residual floor
  //     (예: P1B ring closure ~20 nm → ‖φ‖ 2.8e-8) 는 strict tolerance 에 원리적으로 도달
  //     불가하므로 WARN 대상이 아니다.
  pinocchio::Data data(model);
  const Eigen::VectorXd q0 = pinocchio::neutral(model);
  const ProjectionOptions proj_opts;
  const ProjectionResult proj = ProjectToConstraint(model, data, out.constraints, q0, proj_opts);
  out.q_ref = proj.q;
  out.q_ref_converged = proj.converged;
  out.q_ref_acceptable = proj.acceptable;
  if (!proj.acceptable) {
    RCLCPP_WARN(logger(),
                "q_ref projection 실패 (iter=%d, ‖φ‖=%.3e > acceptance %.1e; strict %.1e) — "
                "q_ref 를 정상 결과로 사용하지 마세요. neutral 근방 조립 상태 확인 필요",
                proj.iterations, proj.final_error, proj_opts.acceptance_tolerance,
                proj_opts.tolerance);
  } else if (!proj.converged) {
    // 수용된 residual floor — 시작 로그를 시끄럽게 하지 않도록 DEBUG (builder 경유 로드는
    // pinocchio_model_builder 의 INFO 요약 라인에 acceptable 플래그가 함께 찍힌다).
    RCLCPP_DEBUG(logger(),
                 "q_ref projection 은 strict tolerance %.1e 미도달이나 수용됨 (iter=%d, "
                 "‖φ‖=%.3e ≤ acceptance %.1e) — residual floor 로 판단",
                 proj_opts.tolerance, proj.iterations, proj.final_error,
                 proj_opts.acceptance_tolerance);
  }

  // (4) q_ref 특이성 검사 — 두 결손을 구분한다 (loop_verification.hpp JacobianReport 참조).
  //     · 조립 특이 (passive 열 Jc_D 의 rank 결손): passive 가 actuated 로 결정되지 않아 축약
  //       핸들이 damped 값을 돌려준다 → q_ref_singular + WARN. ClosedChainHandle /
  //       RtClosedChainHandle 의 Status::singular 와 같은 임계를 공유한다.
  //     · 구속 중복 (행 종속; 평면 loop 의 contact_3d 축방향 행 등): 자세 무관·구조적이고 축약
  //       핸들에는 무해 — Delassus 기반 solver (constraintDynamics plain overload) 만 특이해지므로
  //       INFO 로만 남긴다. converged 여도 둘 중 하나가 성립할 수 있다 (별개 신호).
  if (!out.constraints.empty()) {
    const JacobianReport rep =
        AnalyzeConstraintJacobian(model, data, out.constraints, out.q_ref, out.actuated_joint_ids);
    out.q_ref_singular = rep.reduction_singular;
    if (rep.passive_cols > rep.rows) {
      RCLCPP_WARN(logger(),
                  "loop-passive DoF %d > 구속 rows %d — actuated_joints 가 loop 를 다 결정하지 "
                  "못합니다 (sidecar actuation 확인; 축약 핸들은 이 모델의 생성을 거부합니다)",
                  rep.passive_cols, rep.rows);
    } else if (out.q_ref_singular) {
      RCLCPP_WARN(logger(),
                  "q_ref 가 조립 특이입니다 (σ_min(Jc_D)=%.3e < %.1e) — passive 관절이 actuated "
                  "로 결정되지 않아 축약 동역학이 damped 값을 냅니다; operating configuration "
                  "으로 직접 사용하지 마세요",
                  rep.sigma_min_reduction, kClosedChainSingularSvThreshold);
    } else if (out.actuated_joint_ids.empty()) {
      RCLCPP_DEBUG(logger(),
                   "actuated joint 가 없어 q_ref 조립 특이 판정을 하지 않습니다 (passive 집합 "
                   "미정의, singular=false 유지)");
    }
    if (rep.redundant_rows > 0) {
      RCLCPP_INFO(logger(),
                  "구속 중복 %d 행 (rank %d < rows %d; 평면 loop 의 contact_3d 축방향 행 등) — "
                  "Delassus 기반 solver (constraintDynamics) 는 이 모델에서 자세 무관 특이, "
                  "축약 핸들은 영향 없음",
                  rep.redundant_rows, rep.rank, rep.rows);
    }
  }

  return out;
}

ClosedChainModel BuildClosedChainModelFromExtendedUrdf(std::string_view urdf_path,
                                                       std::string_view closure_yaml_path,
                                                       bool root_joint_floating) {
  ClosedChainModel out;

  // (1) 표준 URDF spanning-tree 빌드
  if (root_joint_floating) {
    pinocchio::urdf::buildModel(std::string(urdf_path), pinocchio::JointModelFreeFlyer(),
                                out.model);
  } else {
    pinocchio::urdf::buildModel(std::string(urdf_path), out.model);
  }

  // (1b) 관성 실현가능성 게이트 — builder 경로와 **같은 처분**을 받는다 (#316 D-5).
  // sidecar 파싱보다 먼저 발사한다: 모델 자체가 강체가 아니면 closure 를 읽을
  // 이유가 없고, 실패 사유도 그쪽 오류에 가려지지 않는다.
  static_cast<void>(EnforceInertialGate(out.model, "BuildClosedChainModelFromExtendedUrdf"));

  // (2) sidecar closure YAML 파싱
  const ClosureSpec spec = LoadClosureYaml(closure_yaml_path);

  // (3) 이미 빌드된 model 위에서 constraints / actuated / q_ref / 특이성 계산
  ClosedChainData data = BuildClosedChainData(out.model, spec);
  out.constraints = std::move(data.constraints);
  out.actuated_joint_ids = std::move(data.actuated_joint_ids);
  out.q_ref = std::move(data.q_ref);
  out.q_ref_converged = data.q_ref_converged;
  out.q_ref_acceptable = data.q_ref_acceptable;
  out.q_ref_singular = data.q_ref_singular;

  return out;
}

}  // namespace rtc_urdf_bridge
