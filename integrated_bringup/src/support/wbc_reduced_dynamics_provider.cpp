// ── WbcReducedDynamicsProvider 구현 (#120) ───────────────────────────────────
#include "integrated_bringup/support/wbc_reduced_dynamics_provider.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

namespace integrated_bringup {

namespace rub = rtc_urdf_bridge;

bool WbcReducedDynamicsProvider::Configure(std::shared_ptr<const pinocchio::Model> full_model,
                                           std::vector<pinocchio::RigidConstraintModel> constraints,
                                           std::vector<pinocchio::JointIndex> actuated_joint_ids,
                                           Eigen::VectorXd q_seed,
                                           const pinocchio::Model& control_model) {
  active_ = false;
  have_last_ = false;
  missing_joint_.clear();

  if (!full_model || constraints.empty()) {
    return false;  // plain URDF (구속 없음) → open-chain
  }

  // ill-posed closure (dep>m, actuated 부족 등) → 핸들 생성 throw → graceful 비활성.
  try {
    handle_ = std::make_unique<rub::RtClosedChainHandle>(
        std::move(full_model), std::move(constraints), std::move(actuated_joint_ids),
        std::move(q_seed));
  } catch (const std::exception&) {
    handle_.reset();
    return false;
  }

  n_a_ = handle_->nv_independent();
  // bijection 전제: 축약 좌표 수 == cache(control) 모델 nv. 아니면 M/h/g 덮어쓰기 불가.
  if (n_a_ <= 0 || n_a_ != control_model.nv) {
    handle_.reset();
    return false;
  }

  // 이름 기반 permutation: 핸들 독립좌표 k ↔ cache 모델 (q_idx, v_idx). 모든 독립 관절이
  // cache 모델에 단일-DoF 로 존재해야 활성 (없으면 좌표 정렬 불가 → open-chain fallback).
  const std::vector<std::string> indep_names = handle_->GetIndependentJointNames();
  cache_q_idx_.assign(static_cast<std::size_t>(n_a_), 0);
  cache_v_idx_.assign(static_cast<std::size_t>(n_a_), 0);
  is_continuous_.assign(static_cast<std::size_t>(n_a_), 0);
  for (int k = 0; k < n_a_; ++k) {
    const std::string& name = indep_names[static_cast<std::size_t>(k)];
    if (!control_model.existJointName(name)) {
      missing_joint_ = name;
      handle_.reset();
      return false;
    }
    const auto jid = control_model.getJointId(name);
    const auto jidx = static_cast<std::size_t>(jid);
    if (control_model.nvs[jidx] != 1) {
      missing_joint_ = name;  // multi-DoF → 스칼라 q_a 매핑 불가
      handle_.reset();
      return false;
    }
    const auto ki = static_cast<std::size_t>(k);
    cache_q_idx_[ki] = static_cast<int>(control_model.idx_qs[jidx]);
    cache_v_idx_[ki] = static_cast<int>(control_model.idx_vs[jidx]);
    is_continuous_[ki] = (control_model.nqs[jidx] == 2) ? 1 : 0;
  }

  // RT 스크래치 사전 할당.
  q_a_ = Eigen::VectorXd::Zero(n_a_);
  v_a_ = Eigen::VectorXd::Zero(n_a_);
  M_last_ = Eigen::MatrixXd::Zero(n_a_, n_a_);
  h_last_ = Eigen::VectorXd::Zero(n_a_);
  g_last_ = Eigen::VectorXd::Zero(n_a_);

  active_ = true;
  return true;
}

void WbcReducedDynamicsProvider::ScatterInto(Eigen::MatrixXd& M, Eigen::VectorXd& h,
                                             Eigen::VectorXd& g, const Eigen::MatrixXd& M_src,
                                             const Eigen::VectorXd& h_src,
                                             const Eigen::VectorXd& g_src) const noexcept {
  // cache 좌표 (cache_v_idx_) 로 permute. cache_v_idx_ 는 0..nv-1 의 순열이므로 M(nv×nv) 전체를
  // 덮는다 (nv == n_a). RT-safe: 계수별 대입, 재할당 없음.
  for (int k = 0; k < n_a_; ++k) {
    const int vk = cache_v_idx_[static_cast<std::size_t>(k)];
    g[vk] = g_src[k];
    h[vk] = h_src[k];
    for (int l = 0; l < n_a_; ++l) {
      M(vk, cache_v_idx_[static_cast<std::size_t>(l)]) = M_src(k, l);
    }
  }
}

bool WbcReducedDynamicsProvider::FillReducedDynamics(const Eigen::VectorXd& q,
                                                     const Eigen::VectorXd& v, Eigen::MatrixXd& M,
                                                     Eigen::VectorXd& h,
                                                     Eigen::VectorXd& g) noexcept {
  if (!active_) {
    return false;
  }

  // cache 좌표 q/v → 핸들 독립좌표 q_a/v_a (스칼라). continuous 관절은 (cos,sin) → atan2 로 각
  // 복원.
  for (int k = 0; k < n_a_; ++k) {
    const auto ki = static_cast<std::size_t>(k);
    const int qi = cache_q_idx_[ki];
    q_a_[k] = is_continuous_[ki] ? std::atan2(q[qi + 1], q[qi]) : q[qi];
    v_a_[k] = v[cache_v_idx_[ki]];
  }

  const rub::RtClosedChainHandle::Status st =
      handle_->Update(std::span<const double>(q_a_.data(), static_cast<std::size_t>(n_a_)));
  // F3 handshake: 핸들 kinematics(data_)가 이번 tick q 로 갱신됨 → FillReducedFrameKinematics 가
  // 같은 Update() 안에서 이 사영을 재사용해도 안전. held tick 도 data_ 는 직전 유효 q_full_ 로
  // 복원돼 유효하므로 여기서 set (frame kin 은 자체 held 판정으로 last-good hold).
  dynamics_projected_ = true;
  bool ok = !st.held && !st.singular;
  if (ok) {
    const rub::RtClosedChainHandle::Status dst = handle_->UpdateDynamics(
        std::span<const double>(v_a_.data(), static_cast<std::size_t>(n_a_)));
    ok = !dst.held;
  }

  if (!ok) {
    // held/singular → degraded 동역학 주입 금지. 직전 유효 축약값 유지, 없으면 open-chain fallback.
    if (!have_last_) {
      return false;
    }
    ScatterInto(M, h, g, M_last_, h_last_, g_last_);
    return true;
  }

  // fresh 축약 동역학 → last-good 갱신 + cache 좌표로 흩뿌림.
  M_last_ = handle_->GetMassMatrix();
  h_last_ = handle_->GetNonLinearEffects();
  g_last_ = handle_->GetGeneralizedGravity();
  have_last_ = true;
  ScatterInto(M, h, g, M_last_, h_last_, g_last_);
  return true;
}

// ── Phase ③: loop-consistent contact frame kinematics ────────────────────────
void WbcReducedDynamicsProvider::ConfigureContactFrames(
    const pinocchio::Model& control_model,
    const std::vector<pinocchio::FrameIndex>& contact_frame_ids) {
  contact_frames_configured_ = false;
  n_contacts_ = 0;
  contact_full_fid_.clear();
  contact_downstream_.clear();
  contact_J_last_.clear();
  contact_oMf_last_.clear();
  contact_dJv_last_.clear();
  contact_have_last_.clear();
  unmapped_contact_frames_.clear();

  // provider 비활성(구속 없음/좌표 미매칭)이면 override 대상 없음 → open-chain 유지.
  if (!active_ || !handle_) {
    return;
  }

  const std::size_t n = contact_frame_ids.size();
  n_contacts_ = static_cast<int>(n);
  contact_full_fid_.assign(n, 0);
  contact_downstream_.assign(n, 0);
  contact_oMf_last_.assign(n, pinocchio::SE3::Identity());
  contact_dJv_last_.assign(n, Eigen::Matrix<double, 6, 1>::Zero());
  contact_have_last_.assign(n, 0);
  contact_J_last_.resize(n);

  for (std::size_t i = 0; i < n; ++i) {
    contact_J_last_[i] = Eigen::MatrixXd::Zero(6, n_a_);  // last-good 버퍼 사전 할당

    // cache(control) 모델 frame id → 이름 → full 모델 fid. 실패 시 미매핑(open-chain).
    // 매핑 실패 frame 은 loop-하류 여부를 판정할 수 없어 override 에서 빠지고 open-chain
    // (frozen-loop) 값이 조용히 유지되므로, 이름을 기록해 호출측이 WARN 하도록 한다 (non-RT).
    const pinocchio::FrameIndex cfid = contact_frame_ids[i];
    if (cfid >= control_model.frames.size()) {
      unmapped_contact_frames_.push_back("<oob frame id " + std::to_string(cfid) + ">");
      continue;  // OOB frame id
    }
    const std::string& name = control_model.frames[cfid].name;
    const pinocchio::FrameIndex full_fid = handle_->GetFrameId(name);
    if (full_fid == 0) {
      unmapped_contact_frames_.push_back(name);
      continue;  // full 모델에 없음(universe 반환) → 매핑 불가
    }
    contact_full_fid_[i] = full_fid;
    // 비하류(serial 등가) frame 은 open-chain 이 정확값 → override 안 함 (byte-for-byte).
    contact_downstream_[i] = handle_->IsFrameDownstreamOfLoop(full_fid) ? 1 : 0;
  }

  J_a_scratch_ = Eigen::MatrixXd::Zero(6, n_a_);
  contact_frames_configured_ = true;
}

void WbcReducedDynamicsProvider::ScatterFrameJacobian(Eigen::MatrixXd& J_dst,
                                                      const Eigen::MatrixXd& J_a) const noexcept {
  // 핸들 독립좌표 k(J_a 열) → cache v-idx(J_dst 열). cache_v_idx_ 는 0..nv-1 의 순열이므로
  // J_dst(6×nv) 전열이 덮인다 (nv == n_a). RT-safe: 열 대입, 재할당 없음.
  for (int k = 0; k < n_a_; ++k) {
    J_dst.col(cache_v_idx_[static_cast<std::size_t>(k)]) = J_a.col(k);
  }
}

bool WbcReducedDynamicsProvider::FillReducedFrameKinematics(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    std::vector<rub::PinocchioCache::FrameCache>& contact_frames) noexcept {
  (void)q;  // 핸들은 같은 tick 의 FillReducedDynamics 에서 이미 Update(q_a) 됨 — 재사영 없음.
  (void)v;
  if (!active_ || !contact_frames_configured_ || !handle_) {
    return false;
  }

  // F3: 이 메서드는 **같은 PinocchioCache::Update() 안에서 FillReducedDynamics 가 먼저 돌아**
  // 핸들 kinematics 를 이번 tick q 로 사영했다는 것을 전제로 q/v 를 무시하고 핸들 상태를 재사용한다
  // (pinocchio_cache.cpp 의 호출 순서가 유일한 보증). 그 순서가 깨지면(예: 호출 재배치, provider
  // 를 dynamics 주입 없이 배선) 여기서 직전 tick 의 stale kinematics 를 주입하게 된다. per-tick
  // 핸드셰이크 토큰으로 이 오배치를 debug 빌드에서 잡는다 (Release/NDEBUG 는 컴파일 아웃 → RT
  // no-op).
  assert(dynamics_projected_ &&
         "FillReducedFrameKinematics requires a same-Update() preceding FillReducedDynamics "
         "(handle kinematics stale otherwise)");
  dynamics_projected_ = false;  // 토큰 소비 (다음 tick 은 FillReducedDynamics 가 다시 set)

  // 같은 tick 의 핸들 사영 상태. held/singular 면 이번 tick 은 loop-untrustworthy → last-good hold
  // (L1: open-chain frozen-loop 미주입). fresh 면 override + last-good 갱신.
  const rub::RtClosedChainHandle::Status& st = handle_->GetStatus();
  const bool fresh = !st.held && !st.singular;

  bool any = false;
  const std::size_t n = std::min(contact_frames.size(), static_cast<std::size_t>(n_contacts_));
  for (std::size_t i = 0; i < n; ++i) {
    if (!contact_downstream_[i]) {
      continue;  // 비하류/미매핑 → open-chain 정확값 유지 (byte-for-byte)
    }
    if (fresh) {
      // fresh → J/oMf/dJv 를 **한 snapshot 단위**로 저장 (#173 F3). drift 는 같은 tick 의
      // FillReducedDynamics 가 돌린 UpdateDynamics 의 2차 FK 상태에서 읽는다 (L2-exact).
      handle_->GetFrameJacobian(contact_full_fid_[i], pinocchio::LOCAL_WORLD_ALIGNED, J_a_scratch_);
      contact_J_last_[i] = J_a_scratch_;
      contact_oMf_last_[i] = handle_->GetFramePlacement(contact_full_fid_[i]);
      handle_->GetFrameClassicalAccelerationDrift(
          contact_full_fid_[i], pinocchio::LOCAL_WORLD_ALIGNED, contact_dJv_last_[i]);
      contact_have_last_[i] = 1;
    } else if (!contact_have_last_[i]) {
      continue;  // 최초 유효 tick 이전 held/singular → open-chain (불가피)
    }
    // last-good(=이번 fresh 또는 직전 유효) loop-consistent J/oMf/dJv 3값 동시 주입
    // (#173 L2-exact). dJv 는 frame-space 6×1 이라 열 순열 불필요 — 직접 복사.
    ScatterFrameJacobian(contact_frames[i].J, contact_J_last_[i]);
    contact_frames[i].oMf = contact_oMf_last_[i];
    contact_frames[i].dJv = contact_dJv_last_[i];
    any = true;
  }
  return any;
}

}  // namespace integrated_bringup
