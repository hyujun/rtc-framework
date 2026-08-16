// ── test_wbc_contact_permutation_real — Phase ③ contact override (real 16-DoF closure) ──
//   test_wbc_contact_kinematics_loop_consistent 의 CrankRocker 케이스는 control.nv==1 이라
//   ScatterFrameJacobian 의 열 scatter·override 경로를 **단일 열**로만 검증한다. 이 테스트는 실제
//   ur5e_p1b closed-chain(16-DoF, 독립 관절 다수)을 로드해 loop-하류 contact frame override 의 **전
//   경로**(Configure 좌표 permutation 구성 → FillReducedDynamics 사영 → FillReducedFrameKinematics
//   다열 scatter → oMf)를 태우고, 결과가 독립 참조 핸들 J·oMf 를 계약대로(핸들 독립좌표 k → cache
//   v-index) 재배열한 값과 **다열**에서 일치함을 검증한다 — 열-인덱스 버그(idx_qs 오용/방향 반전
//   등)를 잡는다. 통합 경로는 PinocchioCache::Update 로 재현한다.
//
//   **permutation 은 항등이 관측된다**: 제어(actuated) 모델은 full 모델에서 passive 관절만 잠근
//   buildReducedModel 산출이라 남은 독립 관절의 상대 순서가 보존되고, 핸들도 독립 관절을 full-model
//   v-index 오름차순으로 노출하므로 cache_v_idx_[k]==k 다. 비항등은 buildReducedModel 이 만들지
//   못하는 순서에서만 나오는 defensive 경로(Configure 주석 "buildReducedModel 순서를 가정하지
//   않는다")라 프로덕션 미도달 → 강제하지 않고, 대신 **다열 override 정합**을 실규모로 검증한다.
#include "integrated_bringup/support/wbc_reduced_dynamics_provider.hpp"
#include "rtc_urdf_bridge/pinocchio_cache.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"
#include "rtc_urdf_bridge/types.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;
namespace ib = integrated_bringup;

namespace {

rub::ModelConfig MakeUr5eP1bConfig() {
  const std::string share = ament_index_cpp::get_package_share_directory("robot_descriptions");
  rub::ModelConfig cfg;
  cfg.urdf_path = share + "/robots/ur5e_p1b/urdf/ur5e_with_proto_1b.urdf.xacro";
  cfg.root_joint_type = "fixed";
  cfg.closure_yaml_path = share + "/robots/ur5e_p1b/urdf/ur5e_with_proto_1b.closure.yaml";
  cfg.sub_models.push_back({"ur5e", "base", "tool0"});
  cfg.tree_models.push_back({"wbc",
                             "base",
                             {"l_thumb_tip_bracket", "l_index_tip_bracket", "l_middle_tip_bracket",
                              "l_ring_tip_bracket"}});
  return cfg;
}

}  // namespace

// ── 실제 closed-chain 에서 비항등 permutation 으로 override J·oMf 가 참조 핸들과 일치 ──────────
TEST(WbcContactPermutationReal, Ur5eP1bMultiColumnOverrideMatchesHandle) {
  auto builder = std::make_shared<rub::PinocchioModelBuilder>(MakeUr5eP1bConfig());

  const auto control = builder->GetActuatedModel();
  ASSERT_TRUE(control) << "ur5e_p1b 는 extended → actuated(closed-chain) control 모델이 있어야 함";
  const auto full = builder->GetFullModel();
  ASSERT_TRUE(full);

  // provider 활성 (M/h/g + 좌표 permutation 구성).
  ib::WbcReducedDynamicsProvider provider;
  ASSERT_TRUE(provider.Configure(full, builder->GetConstraintModels(),
                                 builder->GetClosureActuatedJointIds(),
                                 builder->GetClosureReferenceConfig(), *control));

  // 독립 참조 핸들 (provider 와 동일 입력·default iterations → 동일 사영).
  rub::RtClosedChainHandle ref(full, builder->GetConstraintModels(),
                               builder->GetClosureActuatedJointIds(),
                               builder->GetClosureReferenceConfig());
  const std::vector<std::string> indep = ref.GetIndependentJointNames();
  const int n_a = ref.nv_independent();
  ASSERT_EQ(n_a, control->nv);

  // loop-하류 contact frame 선정: 알려진 fingertip bracket. control 모델에 존재 + 하류여야 함.
  const std::string contact_name = "l_index_tip_bracket";
  ASSERT_TRUE(control->existFrame(contact_name)) << contact_name << " 이 control 모델에 있어야 함";
  const pinocchio::FrameIndex ctrl_fid = control->getFrameId(contact_name);
  const pinocchio::FrameIndex full_fid = ref.GetFrameId(contact_name);
  ASSERT_NE(full_fid, static_cast<pinocchio::FrameIndex>(0));
  ASSERT_TRUE(ref.IsFrameDownstreamOfLoop(full_fid)) << contact_name << " 은 loop-하류여야 함";

  provider.ConfigureContactFrames(*control, std::vector<pinocchio::FrameIndex>{ctrl_fid});
  ASSERT_TRUE(provider.unmapped_contact_frames().empty()) << "contact frame 매핑 성공해야 함";

  // 통합 경로 cache (control 모델, 해당 contact frame).
  rub::PinocchioCache cache;
  cache.Init(control, std::vector<pinocchio::FrameIndex>{ctrl_fid});
  cache.reduced_provider = &provider;

  // loop-consistent q(control): seed(full)의 독립 관절값을 control 좌표로 복사 → fresh 사영 보장.
  const Eigen::VectorXd& seed_full = builder->GetClosureReferenceConfig();
  Eigen::VectorXd q = pinocchio::neutral(*control);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(control->nv);
  Eigen::VectorXd q_a(n_a);
  std::vector<double> v_a(static_cast<std::size_t>(n_a));
  int non_identity = 0;
  for (int k = 0; k < n_a; ++k) {
    const std::string& name = indep[static_cast<std::size_t>(k)];
    ASSERT_TRUE(control->existJointName(name));
    const auto cj = control->getJointId(name);
    const auto fj = full->getJointId(name);
    const int cqs = static_cast<int>(control->idx_qs[cj]);
    const int fqs = static_cast<int>(full->idx_qs[fj]);
    const int cvs = static_cast<int>(control->idx_vs[cj]);
    for (int d = 0; d < control->nqs[cj]; ++d) {
      q[cqs + d] = seed_full[fqs + d];
    }
    // 참조 핸들 입력 q_a (스칼라 각; continuous 는 atan2 로 복원 — provider 와 동일 규약).
    q_a[k] = (control->nqs[cj] == 2) ? std::atan2(q[cqs + 1], q[cqs]) : q[cqs];
    v[cvs] = 0.05 * static_cast<double>(k + 1);  // 비영 속도 (dJv 검증이 비자명하도록)
    v_a[static_cast<std::size_t>(k)] = v[cvs];  // 참조 핸들 입력 (핸들 독립좌표 순서)
    if (cvs != k) {
      ++non_identity;
    }
  }
  // permutation 은 항등이 관측된다 (buildReducedModel 순서 보존 — 파일 헤더 주석). 정보용 기록.
  std::cout << "[ Phase3   ] ur5e_p1b n_a=" << n_a << ", non-identity cols=" << non_identity
            << " (0=identity, defensive 비항등 경로는 프로덕션 미도달)\n";

  // 통합 override 실행.
  cache.Update(q, v);

  // 참조: 동일 q_a 로 핸들 사영 후 loop-consistent J(핸들 순서)·oMf.
  const auto st = ref.Update(std::vector<double>(q_a.data(), q_a.data() + n_a));
  ASSERT_FALSE(st.held) << "loop-consistent seed → held 아님";
  ASSERT_FALSE(st.singular) << "loop-consistent seed → singular 아님";
  Eigen::MatrixXd J_ref(6, n_a);
  ref.GetFrameJacobian(full_fid, pinocchio::LOCAL_WORLD_ALIGNED, J_ref);
  const pinocchio::SE3 oMf_ref = ref.GetFramePlacement(full_fid);
  // 같은 tick 의 UpdateDynamics 로 loop-consistent drift (provider 경로와 동일 순서).
  ASSERT_FALSE(ref.UpdateDynamics(v_a).held);
  Eigen::Matrix<double, 6, 1> dJv_ref;
  ref.GetFrameClassicalAccelerationDrift(full_fid, pinocchio::LOCAL_WORLD_ALIGNED, dJv_ref);

  // 기대 cache J: 계약대로 핸들 열 k → cache v-index(control.idx_vs[name]).
  Eigen::MatrixXd J_expected = Eigen::MatrixXd::Zero(6, control->nv);
  for (int k = 0; k < n_a; ++k) {
    const auto cj = control->getJointId(indep[static_cast<std::size_t>(k)]);
    J_expected.col(static_cast<Eigen::Index>(control->idx_vs[cj])) = J_ref.col(k);
  }

  const Eigen::MatrixXd& J_cache = cache.contact_frames[0].J;
  EXPECT_LT((J_cache - J_expected).norm(), 1e-9)
      << "override J 가 참조 핸들 J 를 계약 열매핑으로 재배열한 값과 일치 (비항등 permutation)";
  EXPECT_LT((cache.contact_frames[0].oMf.translation() - oMf_ref.translation()).norm(), 1e-9)
      << "override oMf 위치 일치";
  EXPECT_LT((cache.contact_frames[0].oMf.rotation() - oMf_ref.rotation()).norm(), 1e-9)
      << "override oMf 회전 일치";
  // L2-exact (#173): dJv 는 frame-space 6×1 (열 순열 무관) — 참조 핸들 drift 와 직접 비교.
  //   (L2-zero 잠정안의 dJv=0 assertion 교체 — spec 변경, issue #173 본문.)
  EXPECT_GT(dJv_ref.norm(), 1e-10) << "비영 v → loop-consistent drift 비영 (공허 통과 방지)";
  EXPECT_LT((cache.contact_frames[0].dJv - dJv_ref).norm(), 1e-9)
      << "override dJv 가 참조 핸들 drift 와 일치 (L2-exact)";
}
