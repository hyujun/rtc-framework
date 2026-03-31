// ── example_tree_model.cpp ──────────────────────────────────────────────────
// 트리모델 사용법:
//   1. YAML에 정의된 트리모델 로드 (하나의 root, 여러 tip)
//   2. 트리모델 구축 + tool frame_id 매핑
//   3. 임의 config에서 모든 tip FK 동시 출력
//   4. branching point 출력
//
// 사용법: ./example_tree_model <yaml_config_path>
// ─────────────────────────────────────────────────────────────────────────────

#include "urdf_pinocchio_bridge/pinocchio_model_builder.hpp"
#include "urdf_pinocchio_bridge/rt_model_handle.hpp"

#include <cstdlib>
#include <iostream>
#include <vector>

namespace upb = urdf_pinocchio_bridge;

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "사용법: " << argv[0] << " <yaml_config_path>\n";
    return EXIT_FAILURE;
  }

  std::cout << "=== urdf_pinocchio_bridge: 트리모델 사용법 ===\n\n";

  // ── (1) Builder 생성 ──────────────────────────────────────────────────────
  upb::PinocchioModelBuilder builder(argv[1]);

  auto tree_names = builder.GetTreeModelNames();
  std::cout << "[1] 등록된 트리모델 수: " << tree_names.size() << "\n";

  for (const auto & name : tree_names) {
    auto model = builder.GetTreeModel(name);
    const auto & def = builder.GetTreeModelDefinition(name);

    std::cout << "\n[2] 트리모델 '" << name << "'\n";
    std::cout << "  root: " << def.root_link << "\n";
    std::cout << "  tips: ";
    for (const auto & tip : def.tip_links) std::cout << tip << " ";
    std::cout << "\n";
    std::cout << "  nq=" << model->nq << ", nv=" << model->nv << "\n";
    std::cout << "  actuated joints: ";
    for (const auto & j : def.joint_names) std::cout << j << " ";
    std::cout << "\n";

    // ── (3) tool frame_id 매핑 ──────────────────────────────────────────────
    upb::RtModelHandle handle(model);

    std::cout << "\n[3] Tip frame ID 매핑:\n";
    std::vector<pinocchio::FrameIndex> tip_fids;
    for (const auto & tip : def.tip_links) {
      auto fid = handle.GetFrameId(tip);
      tip_fids.push_back(fid);
      std::cout << "  " << tip << " → frame_id=" << fid << "\n";
    }

    // ── (4) 임의 config에서 모든 tip FK ─────────────────────────────────────
    std::vector<double> q(static_cast<std::size_t>(handle.nq()), 0.3);
    handle.ComputeForwardKinematics(q);

    std::cout << "\n[4] FK 결과 (q=0.3 rad):\n";
    for (std::size_t i = 0; i < def.tip_links.size(); ++i) {
      auto pos = handle.GetFramePosition(tip_fids[i]);
      std::cout << "  " << def.tip_links[i]
                << ": [" << pos.transpose() << "]\n";
    }

    // ── (5) Jacobian 추출 (첫 번째 tip) ─────────────────────────────────────
    if (!tip_fids.empty()) {
      handle.ComputeJacobians(q);
      Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, handle.nv());
      handle.GetFrameJacobian(tip_fids[0], pinocchio::LOCAL_WORLD_ALIGNED, J);
      std::cout << "\n[5] Jacobian (" << def.tip_links[0] << ", LOCAL_WORLD_ALIGNED):\n";
      std::cout << J << "\n";
    }

    // ── (6) Branching points ────────────────────────────────────────────────
    std::cout << "\n[6] 분기점 (branching points):\n";
    if (def.branching_points.empty()) {
      std::cout << "  (없음)\n";
    } else {
      for (const auto & bp : def.branching_points) {
        std::cout << "  " << bp << "\n";
      }
    }
  }

  std::cout << "\n완료.\n";
  return EXIT_SUCCESS;
}
