// ── example_rt_integration.cpp ──────────────────────────────────────────────
// 외부 패키지의 RT 제어 루프에서 rtc_urdf_bridge를 사용하는 전형적 패턴.
//
// [Phase 1] 초기화 (non-RT)
//   - YAML → URDF 분석 → 모델 구축 → Handle 생성
//
// [Phase 2] RT 루프 시뮬레이션
//   - 매 사이클: FK → Jacobian → nle → 제어 법칙 → 토크 계산
//   - Joint-space impedance + Task-space impedance 모두 시연
//
// [Phase 3] 정리 (RAII 자동)
//
// 사용법: ./example_rt_integration <yaml_config_path>
//
// ═══════════════════════════════════════════════════════════════════════════════
// [부록] 외부 패키지 CMake 사용법:
//
//   # robot_control/CMakeLists.txt:
//   find_package(rtc_urdf_bridge REQUIRED)
//   target_link_libraries(my_controller
//     PUBLIC rtc_urdf_bridge::rtc_urdf_bridge)
//
//   # robot_control/package.xml:
//   <depend>rtc_urdf_bridge</depend>
// ═══════════════════════════════════════════════════════════════════════════════

#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

// Pinocchio SE3 로그용
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/explog.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "사용법: " << argv[0] << " <yaml_config_path>\n";
    return EXIT_FAILURE;
  }

  // ================================================================
  // [Phase 1] 초기화 — non-RT (on_configure / main 시작부)
  // ================================================================
  std::cout << "=== Phase 1: 초기화 (non-RT) ===\n\n";

  // (1) YAML 설정 → 모델 구축
  rub::PinocchioModelBuilder builder(argv[1]);
  const auto & analyzer = builder.GetAnalyzer();

  std::cout << "URDF 루트 링크: " << analyzer.GetRootLinkName() << "\n";
  std::cout << "Active 관절 수: " << analyzer.GetActiveJointNames().size() << "\n";

  // (2) Full model handle (항상 생성)
  auto full_model = builder.GetFullModel();
  rub::RtModelHandle full_handle(full_model);
  std::cout << "Full model: nq=" << full_handle.nq() << ", nv=" << full_handle.nv() << "\n";

  // (3) YAML에 정의된 서브모델별 handle 동적 생성 (이름 하드코딩 없음)
  //     pin::Model 수명 > RtModelHandle 수명 (Builder scope > handle map scope)
  std::map<std::string, rub::RtModelHandle> sub_handles;
  for (const auto & name : builder.GetSubModelNames()) {
    auto model = builder.GetReducedModel(name);
    sub_handles.emplace(name, rub::RtModelHandle(model));
    std::cout << "서브모델 '" << name << "': nq=" << model->nq
              << ", nv=" << model->nv << "\n";
  }

  // (4) YAML에 정의된 트리모델별 handle
  std::map<std::string, rub::RtModelHandle> tree_handles;
  for (const auto & name : builder.GetTreeModelNames()) {
    auto model = builder.GetTreeModel(name);
    tree_handles.emplace(name, rub::RtModelHandle(model));
    std::cout << "트리모델 '" << name << "': nq=" << model->nq
              << ", nv=" << model->nv << "\n";
  }

  // (5) Tool frame ID 캐싱 (RT 루프에서 문자열 검색 방지)
  auto tool_frame_id = static_cast<pinocchio::FrameIndex>(full_model->nframes - 1);
  std::cout << "Tool frame: " << full_model->frames[tool_frame_id].name
            << " (id=" << tool_frame_id << ")\n";

  // (6) 제어 파라미터 설정 (예제용 하드코딩)
  const int nv = full_handle.nv();
  Eigen::VectorXd Kp_joint = Eigen::VectorXd::Constant(nv, 100.0);
  Eigen::VectorXd Kd_joint = Eigen::VectorXd::Constant(nv, 20.0);
  Eigen::Matrix<double, 6, 1> Kp_task;
  Kp_task << 200.0, 200.0, 200.0, 50.0, 50.0, 50.0;
  Eigen::Matrix<double, 6, 1> Kd_task;
  Kd_task << 40.0, 40.0, 40.0, 10.0, 10.0, 10.0;

  // 사전 할당 버퍼 (RT 경로에서 힙 할당 금지)
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, nv);
  Eigen::VectorXd tau_joint = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd tau_task = Eigen::VectorXd::Zero(nv);

  std::cout << "\n=== Phase 2: RT 루프 시뮬레이션 ===\n\n";

  // ================================================================
  // [Phase 2] RT 루프 시뮬레이션 — 매 사이클 호출 패턴
  // ================================================================
  constexpr int kNumCycles = 1000;
  constexpr double kDt = 0.002;  // 500 Hz

  for (int cycle = 0; cycle < kNumCycles; ++cycle) {
    double t = static_cast<double>(cycle) * kDt;

    // ── (1) 센서 데이터 생성 (시뮬레이션: sine wave trajectory) ────────────
    std::vector<double> q_sensor(static_cast<std::size_t>(full_handle.nq()));
    std::vector<double> v_sensor(static_cast<std::size_t>(full_handle.nv()), 0.0);

    for (int i = 0; i < full_handle.nq(); ++i) {
      double freq = 0.5 + 0.1 * static_cast<double>(i);
      q_sensor[static_cast<std::size_t>(i)] = 0.3 * std::sin(2.0 * M_PI * freq * t);
      v_sensor[static_cast<std::size_t>(i)] =
        0.3 * 2.0 * M_PI * freq * std::cos(2.0 * M_PI * freq * t);
    }

    // 목표 설정값
    std::vector<double> q_des(static_cast<std::size_t>(full_handle.nq()), 0.0);
    std::vector<double> v_des(static_cast<std::size_t>(full_handle.nv()), 0.0);

    // ── (2) FK + Jacobian + nle 계산 ────────────────────────────────────────
    // 모든 함수는 noexcept — RT 안전
    full_handle.ComputeForwardKinematics(q_sensor, v_sensor);  // noexcept
    full_handle.ComputeJacobians(q_sensor);                    // noexcept
    full_handle.GetFrameJacobian(                              // noexcept
      tool_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
    full_handle.ComputeNonLinearEffects(q_sensor, v_sensor);   // noexcept

    auto nle = full_handle.GetNonLinearEffects();

    // ── (3) 제어 법칙 A: Joint-space impedance ──────────────────────────────
    // τ = nle + Kp·(q_des - q) + Kd·(v_des - v)
    for (int i = 0; i < nv; ++i) {
      double eq = q_des[static_cast<std::size_t>(i)] -
                  q_sensor[static_cast<std::size_t>(i)];
      double ev = v_des[static_cast<std::size_t>(i)] -
                  v_sensor[static_cast<std::size_t>(i)];
      tau_joint[i] = nle[i] + Kp_joint[i] * eq + Kd_joint[i] * ev;
    }

    // ── (4) 제어 법칙 B: Task-space impedance ───────────────────────────────
    // e_x = log(x_des · x_cur⁻¹) — SE3 에러
    // τ = J^T · (Kp·e_x + Kd·e_dx) + nle
    auto x_cur = full_handle.GetFramePlacement(tool_frame_id);
    pinocchio::SE3 x_des = pinocchio::SE3::Identity();  // 원점 목표 (예제용)

    // SE3 에러 = log(x_des * x_cur⁻¹) → 6D twist
    pinocchio::Motion error_twist = pinocchio::log6(x_des * x_cur.inverse());
    Eigen::Matrix<double, 6, 1> e_x = error_twist.toVector();

    // 현재 task-space 속도: dx = J * v
    Eigen::Map<const Eigen::VectorXd> v_eigen(
      v_sensor.data(), static_cast<Eigen::Index>(v_sensor.size()));
    Eigen::Matrix<double, 6, 1> dx = J * v_eigen;
    Eigen::Matrix<double, 6, 1> e_dx = -dx;  // v_des = 0

    // τ = J^T · (Kp·e_x + Kd·e_dx) + nle
    Eigen::Matrix<double, 6, 1> f_task =
      Kp_task.cwiseProduct(e_x) + Kd_task.cwiseProduct(e_dx);
    tau_task = J.transpose() * f_task;
    for (int i = 0; i < nv; ++i) {
      tau_task[i] += nle[i];
    }

    // ── (5) 결과 출력 (매 100 사이클) ───────────────────────────────────────
    if (cycle % 100 == 0) {
      std::cout << "cycle=" << cycle << " t=" << t << "s\n";
      std::cout << "  tool pos: ["
                << x_cur.translation().transpose() << "]\n";
      std::cout << "  tau_joint: [" << tau_joint.head(3).transpose() << " ...]\n";
      std::cout << "  tau_task:  [" << tau_task.head(3).transpose() << " ...]\n";
    }
  }

  // ================================================================
  // [Phase 3] 정리 — RAII 자동 해제
  // ================================================================
  //
  // 소유권 순서:
  //   PinocchioModelBuilder (shared_ptr<Model> 소유)
  //   └── sub_handles / tree_handles (shared_ptr<const Model> 참조 + Data 소유)
  //
  // sub_handles가 먼저 소멸 (scope 종료) → handle이 보유한 shared_ptr 해제
  // builder가 그 다음 소멸 → 마지막 shared_ptr 해제 시 Model 삭제
  //
  // shared_ptr을 사용하므로 소멸 순서와 무관하게 안전.
  // 여러 handle이 같은 Model을 const 참조로 공유해도 안전 (Model은 immutable).
  // 각 handle의 Data는 독립이므로 서로 다른 q/v로 동시 계산 가능.

  std::cout << "\n=== Phase 3: RAII 정리 완료 ===\n";
  return EXIT_SUCCESS;
}
