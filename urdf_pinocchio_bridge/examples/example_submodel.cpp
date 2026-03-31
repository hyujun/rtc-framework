// ── example_submodel.cpp ────────────────────────────────────────────────────
// 서브모델(reduced model) 사용법:
//   1. YAML에 정의된 서브모델 로드
//   2. 각 서브모델의 reduced model 정보 출력
//   3. Full model vs Sub-model FK 비교
//   4. 서브모델별 독립 RtModelHandle 생성
//
// 사용법: ./example_submodel <yaml_config_path>
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

  std::cout << "=== urdf_pinocchio_bridge: 서브모델 사용법 ===\n\n";

  // ── (1) Builder 생성 ──────────────────────────────────────────────────────
  upb::PinocchioModelBuilder builder(argv[1]);

  auto full_model = builder.GetFullModel();
  std::cout << "[1] Full model: nq=" << full_model->nq
            << ", nv=" << full_model->nv << "\n\n";

  // ── (2) 서브모델 정보 출력 ────────────────────────────────────────────────
  auto sub_names = builder.GetSubModelNames();
  std::cout << "[2] 등록된 서브모델 수: " << sub_names.size() << "\n";

  for (const auto & name : sub_names) {
    auto model = builder.GetReducedModel(name);
    const auto & def = builder.GetSubModelDefinition(name);

    std::cout << "  '" << name << "': "
              << def.root_link << " → " << def.tip_link
              << " | nq=" << model->nq << ", nv=" << model->nv
              << " | joints: ";
    for (const auto & j : def.joint_names) {
      std::cout << j << " ";
    }
    std::cout << "\n";
  }

  // ── (3) Full vs Sub-model FK 비교 ────────────────────────────────────────
  if (!sub_names.empty()) {
    std::cout << "\n[3] FK 비교: full model vs 서브모델\n";

    auto first_sub_name = sub_names[0];
    auto sub_model = builder.GetReducedModel(first_sub_name);
    const auto & def = builder.GetSubModelDefinition(first_sub_name);

    upb::RtModelHandle full_handle(full_model);
    upb::RtModelHandle sub_handle(sub_model);

    // 전체 모델용 q (모든 관절 = 0.2 rad)
    std::vector<double> q_full(static_cast<std::size_t>(full_handle.nq()), 0.2);
    // 서브모델용 q (서브모델 관절만)
    std::vector<double> q_sub(static_cast<std::size_t>(sub_handle.nq()), 0.2);

    full_handle.ComputeForwardKinematics(q_full);
    sub_handle.ComputeForwardKinematics(q_sub);

    auto tip_full = full_handle.GetFrameId(def.tip_link);
    auto tip_sub = sub_handle.GetFrameId(def.tip_link);

    if (tip_full > 0 && tip_sub > 0) {
      auto pos_full = full_handle.GetFramePosition(tip_full);
      auto pos_sub = sub_handle.GetFramePosition(tip_sub);

      std::cout << "  서브모델: '" << first_sub_name << "'\n";
      std::cout << "  Full  FK: [" << pos_full.transpose() << "]\n";
      std::cout << "  Sub   FK: [" << pos_sub.transpose() << "]\n";
      std::cout << "  차이 norm: " << (pos_full - pos_sub).norm() << "\n";
    }
  }

  // ── (4) 독립 handle로 다른 q에서 계산 ────────────────────────────────────
  if (sub_names.size() >= 2) {
    std::cout << "\n[4] 독립 handle 동시 계산\n";

    auto model_a = builder.GetReducedModel(sub_names[0]);
    auto model_b = builder.GetReducedModel(sub_names[1]);

    upb::RtModelHandle handle_a(model_a);
    upb::RtModelHandle handle_b(model_b);

    std::vector<double> q_a(static_cast<std::size_t>(handle_a.nq()), 0.5);
    std::vector<double> q_b(static_cast<std::size_t>(handle_b.nq()), -0.3);

    handle_a.ComputeForwardKinematics(q_a);
    handle_b.ComputeForwardKinematics(q_b);

    std::cout << "  Handle A ('" << sub_names[0] << "', q=0.5): nq=" << handle_a.nq() << "\n";
    std::cout << "  Handle B ('" << sub_names[1] << "', q=-0.3): nq=" << handle_b.nq() << "\n";
    std::cout << "  (두 handle은 서로 독립적으로 동작)\n";
  }

  std::cout << "\n완료.\n";
  return EXIT_SUCCESS;
}
