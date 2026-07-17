#include "rtc_urdf_bridge/pinocchio_cache.hpp"

#include "rtc_urdf_bridge/reduced_dynamics_provider.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#pragma GCC diagnostic pop

namespace rtc_urdf_bridge {

// ════════════════════════════════════════════════
// PinocchioCache
// ════════════════════════════════════════════════
void PinocchioCache::Init(std::shared_ptr<const pinocchio::Model> model,
                          const std::vector<pinocchio::FrameIndex>& contact_frame_ids) {
  model_ptr = std::move(model);
  const auto& mdl = *model_ptr;
  data = pinocchio::Data(mdl);

  const int nv = mdl.nv;
  const int nq = mdl.nq;

  M.setZero(nv, nv);
  h.setZero(nv);
  g.setZero(nv);
  q.setZero(nq);
  v.setZero(nv);

  contact_frames.resize(contact_frame_ids.size());
  for (size_t i = 0; i < contact_frame_ids.size(); ++i) {
    auto& fc = contact_frames[i];
    fc.frame_id = contact_frame_ids[i];
    fc.J.setZero(6, nv);
    fc.dJv.setZero();
  }

  com_position.setZero();
  Jcom.setZero(3, nv);
  com_drift.setZero();
  h_centroidal.setZero();
  Ag.setZero(6, nv);
  hg_drift.setZero();

  // #120: 축약 동역학 provider 는 이 model 좌표(nv)에 맞춰 배선되므로 model 재바인딩 시
  // 반드시 무효화한다. 현재는 호출처(WBC LoadConfig)가 Init 직후 ConfigureReducedDynamicsProvider
  // 로 재배선하지만, 그 순서에 의존하지 않도록 여기서 stale 포인터를 끊는다 (재-Init 시 옛 nv
  // 기준 permutation 으로 M/h/g 를 out-of-bounds 로 덮는 것을 방지).
  reduced_provider = nullptr;

  registration_locked = false;
}

int PinocchioCache::RegisterFrame(const std::string& name, pinocchio::FrameIndex frame_id) {
  if (registration_locked) {
    return -1;
  }

  // 중복 검사
  for (size_t i = 0; i < registered_frames.size(); ++i) {
    if (registered_frames[i].frame_id == frame_id) {
      return static_cast<int>(i);
    }
  }

  const int nv = model_ptr->nv;
  RegisteredFrame rf;
  rf.name = name;
  rf.frame_id = frame_id;
  rf.J.setZero(6, nv);
  rf.dJv.setZero();
  registered_frames.push_back(std::move(rf));

  return static_cast<int>(registered_frames.size()) - 1;
}

void PinocchioCache::Update(const Eigen::VectorXd& q_in, const Eigen::VectorXd& v_in) noexcept {
  registration_locked = true;

  const auto& mdl = *model_ptr;
  q = q_in;
  v = v_in;

  // FK + CRBA(M) + NLE(h) + Jacobians + frame placements
  pinocchio::computeAllTerms(mdl, data, q, v);

  // Mass matrix (symmetrize)
  M = data.M;
  M.triangularView<Eigen::StrictlyLower>() = M.triangularView<Eigen::StrictlyUpper>().transpose();

  // Nonlinear effects
  h = data.nle;

  // Gravity only
  pinocchio::computeGeneralizedGravity(mdl, data, q);
  g = data.g;

  // #120: closed-chain 축약 동역학 주입. provider 가 set 돼 있으면 open-chain M/h/g 를
  // constraint-consistent 축약값으로 덮는다 (nullptr=미변경, byte-동일).
  if (reduced_provider != nullptr) {
    (void)reduced_provider->FillReducedDynamics(q, v, M, h, g);
  }

  // Contact frame Jacobian + dJv. Phase 1(#unified-kindyn): always-compute —
  // 이전 active 마스크 게이트 제거 (Phase 0 실측으로 예산 확인). 비활성 contact 도 fresh 계산되나
  // 소비자가 active 플래그로 별도 게이팅하므로 제어 출력 불변.
  for (auto& fc : contact_frames) {
    fc.J.setZero();
    pinocchio::getFrameJacobian(mdl, data, fc.frame_id, pinocchio::LOCAL_WORLD_ALIGNED, fc.J);
    fc.oMf = data.oMf[fc.frame_id];

    fc.dJv = pinocchio::getFrameClassicalAcceleration(mdl, data, fc.frame_id,
                                                      pinocchio::LOCAL_WORLD_ALIGNED)
                 .toVector();
  }

  // Phase ③ 대비: closed-chain 활성 시 contact frame J·oMf 를 loop-consistent 값으로 격상.
  // Phase 1 은 default no-op(false) → open-chain(frozen-loop) 값 유지 (byte-for-byte).
  if (reduced_provider != nullptr) {
    (void)reduced_provider->FillReducedFrameKinematics(q, v, contact_frames);
  }

  // Registered frame Jacobian + dJv
  for (auto& rf : registered_frames) {
    rf.J.setZero();
    pinocchio::getFrameJacobian(mdl, data, rf.frame_id, pinocchio::LOCAL_WORLD_ALIGNED, rf.J);
    rf.oMf = data.oMf[rf.frame_id];
    rf.dJv = pinocchio::getFrameClassicalAcceleration(mdl, data, rf.frame_id,
                                                      pinocchio::LOCAL_WORLD_ALIGNED)
                 .toVector();
  }

  // CoM (optional)
  if (compute_com) {
    // CoM 위치, 속도, Jacobian 계산
    pinocchio::centerOfMass(mdl, data, q, v, false);
    com_position = data.com[0];
    pinocchio::jacobianCenterOfMass(mdl, data, q, false);
    Jcom = data.Jcom;

    // CoM drift: zero acceleration에서의 CoM 가속도 = dJ_com·v
    // centerOfMass(model, data, q, v, a=0) → data.acom[0]
    const auto zero_a = Eigen::VectorXd::Zero(mdl.nv);
    pinocchio::centerOfMass(mdl, data, q, v, zero_a);
    com_drift = data.acom[0];
  }

  // Centroidal momentum (optional)
  if (compute_centroidal) {
    pinocchio::computeCentroidalMomentum(mdl, data, q, v);
    h_centroidal = data.hg.toVector();
    pinocchio::computeCentroidalMap(mdl, data, q);
    Ag = data.Ag;

    // Centroidal momentum drift: dAg·v (momentum rate at zero acceleration)
    pinocchio::computeCentroidalMomentumTimeVariation(mdl, data, q, v,
                                                      Eigen::VectorXd::Zero(mdl.nv));
    hg_drift = data.dhg.toVector();
  }
}

}  // namespace rtc_urdf_bridge
