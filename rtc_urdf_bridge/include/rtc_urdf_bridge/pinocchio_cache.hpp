// ── PinocchioCache: 매 tick 1회 결합(arm+hand) actuated 모델 위에서 FK·J·M·h·g 를
//    계산해 task/constraint 소비자와 공유하는 RT 캐시 ──────────────────────────────
//
// Unified kin&dyn 솔버 Phase 1 (#unified-kindyn): 원래 rtc_tsid 에 있던 타입을
// rtc_urdf_bridge (pinocchio·RtModelHandle·RtClosedChainHandle 이 이미 상주) 로 이관.
// rtc_tsid 는 이제 소비자 (ARCH-2 순방향). rtc::tsid::PinocchioCache 심볼은
// rtc_tsid/types/wbc_types.hpp 의 alias 재export 로 유지된다.
//
// ContactState 결합 해소: 이전 Update(q,v,ContactState) 는 rtc_tsid 의 ContactState 를
// active 마스크로만 썼다 (비활성 contact frame Jacobian skip — RT 마이크로 최적화).
// Phase 0 실측(결합 모델 computeAllTerms ~1% 예산)으로 always-compute 가 viable 임이
// 확정되어, 이 타입은 등록된 contact frame 을 **무조건** 계산한다 → ContactState 무의존.
#pragma once

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

namespace rtc_urdf_bridge {

// closed-chain 축약 동역학/축약 frame kinematics 주입 인터페이스 (전방 선언 — 헤더 순환 방지).
class ReducedDynamicsProvider;

// ────────────────────────────────────────────────
// Pinocchio 계산 캐시 (매 tick 1회 갱신, task/constraint 공유)
// ────────────────────────────────────────────────
struct PinocchioCache {
  // Pinocchio model/data (model은 공유, data는 소유)
  std::shared_ptr<const pinocchio::Model> model_ptr;
  pinocchio::Data data;

  // #120: optional closed-chain 축약 동역학 provider (non-owning; nullptr=open-chain, byte-동일).
  // set 시 Update() 의 open-chain M/h/g 계산 직후 호출돼 M/h/g 를 constraint-consistent 축약값으로
  // 덮는다. 소유·수명은 주입 측(integrated_bringup WBC)이 보장. RT tick 매번 호출 → RT-safe 계약.
  ReducedDynamicsProvider* reduced_provider{nullptr};

  // 질량 행렬, 비선형 항
  Eigen::MatrixXd M;  // [nv × nv]
  Eigen::VectorXd h;  // [nv] nonlinear effects (Coriolis + gravity)
  Eigen::VectorXd g;  // [nv] gravity only

  // 현재 q, v 복사본
  Eigen::VectorXd q;  // [nq]
  Eigen::VectorXd v;  // [nv]

  // Contact frame Jacobian 캐시
  struct FrameCache {
    pinocchio::FrameIndex frame_id{0};
    // Identity-init: pinocchio::SE3's default ctor leaves the rotation
    // uninitialized (garbage), and a consumer may read oMf before the first
    // Update() (e.g. a registered frame on a tick where Update was skipped).
    // A garbage rotation trips pinocchio::rpy::matrixToRpy's NDEBUG assert.
    pinocchio::SE3 oMf{pinocchio::SE3::Identity()};  // world-frame placement
    Eigen::MatrixXd J;                               // [6 × nv] LOCAL_WORLD_ALIGNED
    Eigen::Matrix<double, 6, 1> dJv;                 // J̇·v (classical acceleration)
  };

  std::vector<FrameCache> contact_frames;  // [max_contacts]

  // Task용 등록 frame 캐시
  struct RegisteredFrame {
    std::string name;
    pinocchio::FrameIndex frame_id{0};
    // Identity-init (see FrameCache::oMf) — RegisterFrame leaves oMf unset until
    // the first Update(), so a pre-Update reader must see a valid rotation.
    pinocchio::SE3 oMf{pinocchio::SE3::Identity()};
    Eigen::MatrixXd J;
    Eigen::Matrix<double, 6, 1> dJv;
  };

  std::vector<RegisteredFrame> registered_frames;
  bool registration_locked{false};  // Update() 호출 후 true → 추가 등록 금지

  // CoM (optional — CoMTask 등록 시 활성화)
  bool compute_com{false};
  Eigen::Vector3d com_position;
  Eigen::MatrixXd Jcom;       // [3 × nv]
  Eigen::Vector3d com_drift;  // dJ_com·v (zero acceleration에서의 CoM 가속도) [3]

  // Centroidal momentum (optional — MomentumTask 등록 시 활성화)
  bool compute_centroidal{false};
  Eigen::Matrix<double, 6, 1> h_centroidal;
  Eigen::MatrixXd Ag;                    // [6 × nv]
  Eigen::Matrix<double, 6, 1> hg_drift;  // dAg·v (centroidal momentum rate drift) [6]

  // 초기화: buffer pre-allocate (init 시 1회). contact_frame_ids 는 매 tick 무조건 계산할
  // contact frame 의 Pinocchio FrameIndex 리스트 (소비자 인덱싱과 동일 순서). 빈 리스트 허용.
  void Init(std::shared_ptr<const pinocchio::Model> model,
            const std::vector<pinocchio::FrameIndex>& contact_frame_ids);

  // Task가 init 시 필요한 frame 등록 → Update()에서 자동 계산
  // 반환: registered_frames 내 인덱스
  int RegisterFrame(const std::string& name, pinocchio::FrameIndex frame_id);

  // 매 tick 갱신 (RT-safe: 사전 할당된 버퍼만 사용). contact frame 은 always-compute.
  void Update(const Eigen::VectorXd& q_in, const Eigen::VectorXd& v_in) noexcept;
};

}  // namespace rtc_urdf_bridge
