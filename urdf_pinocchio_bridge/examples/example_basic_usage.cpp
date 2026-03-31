// ── example_basic_usage.cpp ──────────────────────────────────────────────────
// urdf_pinocchio_bridge 기본 사용법:
//   1. YAML 설정 → URDF 로드 → 분석
//   2. Pinocchio 모델 구축
//   3. RtModelHandle로 FK, Jacobian, nle 계산
//
// 사용법: ./example_basic_usage <yaml_config_path>
// ─────────────────────────────────────────────────────────────────────────────

#include "urdf_pinocchio_bridge/pinocchio_model_builder.hpp"
#include "urdf_pinocchio_bridge/rt_model_handle.hpp"

#include <Eigen/Core>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace upb = urdf_pinocchio_bridge;

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "사용법: " << argv[0] << " <yaml_config_path>\n";
    return EXIT_FAILURE;
  }

  // ── (1) YAML 설정 로드 + URDF 분석 + 모델 구축 ────────────────────────────
  std::cout << "=== urdf_pinocchio_bridge: 기본 사용법 ===\n\n";
  std::cout << "[1] YAML 설정 파일: " << argv[1] << "\n";

  upb::PinocchioModelBuilder builder(argv[1]);

  // ── (2) URDF 분석 결과 출력 ───────────────────────────────────────────────
  const auto & analyzer = builder.GetAnalyzer();
  std::cout << "\n[2] URDF 분석 결과\n";
  std::cout << "  루트 링크: " << analyzer.GetRootLinkName() << "\n";
  std::cout << "  링크 수: " << analyzer.GetNumLinks() << "\n";
  std::cout << "  관절 수: " << analyzer.GetNumJoints() << "\n";

  std::cout << "  Actuated 관절: ";
  for (const auto & j : analyzer.GetActuatedJointNames()) {
    std::cout << j << " ";
  }
  std::cout << "\n";

  if (!analyzer.GetPassiveJoints().empty()) {
    std::cout << "  Passive 관절: ";
    for (const auto & pj : analyzer.GetPassiveJoints()) {
      std::cout << pj.joint_name << " ";
    }
    std::cout << "\n";
  }

  if (!analyzer.GetMimicJoints().empty()) {
    std::cout << "  Mimic 관절: ";
    for (const auto & mi : analyzer.GetMimicJoints()) {
      std::cout << mi.joint_name << " (→ " << mi.mimicked_joint
                << ", x" << mi.multiplier << " +" << mi.offset << ") ";
    }
    std::cout << "\n";
  }

  // ── (3) Pinocchio 모델 정보 출력 ──────────────────────────────────────────
  auto full_model = builder.GetFullModel();
  std::cout << "\n[3] Pinocchio 모델 정보\n";
  std::cout << "  nq (설정 공간 차원): " << full_model->nq << "\n";
  std::cout << "  nv (속도 공간 차원): " << full_model->nv << "\n";
  std::cout << "  njoints: " << full_model->njoints << "\n";
  std::cout << "  nframes: " << full_model->nframes << "\n";

  // ── (4) RtModelHandle 생성 + FK ───────────────────────────────────────────
  upb::RtModelHandle handle(full_model);

  // neutral config에서 FK
  std::vector<double> q(static_cast<std::size_t>(handle.nq()), 0.0);
  handle.ComputeForwardKinematics(q);
  handle.ComputeJacobians(q);

  // 마지막 non-universe 프레임의 SE3 출력
  std::cout << "\n[4] FK 결과 (neutral config)\n";
  auto last_frame_id = static_cast<pinocchio::FrameIndex>(full_model->nframes - 1);
  const auto & last_frame_name = full_model->frames[last_frame_id].name;
  auto pos = handle.GetFramePosition(last_frame_id);
  auto rot = handle.GetFrameRotation(last_frame_id);

  std::cout << "  마지막 프레임: " << last_frame_name << " (id=" << last_frame_id << ")\n";
  std::cout << "  위치: [" << pos.x() << ", " << pos.y() << ", " << pos.z() << "]\n";
  std::cout << "  회전:\n" << rot << "\n";

  // ── (5) Jacobian 출력 ─────────────────────────────────────────────────────
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, handle.nv());
  handle.GetFrameJacobian(last_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
  std::cout << "\n[5] Jacobian (LOCAL_WORLD_ALIGNED, " << last_frame_name << ")\n";
  std::cout << J << "\n";

  // ── (6) 비선형 효과 (중력 벡터) ──────────────────────────────────────────
  std::vector<double> v(static_cast<std::size_t>(handle.nv()), 0.0);
  handle.ComputeNonLinearEffects(q, v);
  std::cout << "\n[6] 비선형 효과 nle = C(q,0)*0 + g(q):\n";
  std::cout << "  " << handle.GetNonLinearEffects().transpose() << "\n";

  std::cout << "\n완료.\n";
  return EXIT_SUCCESS;
}
