// ── WbcReducedDynamicsProvider 구현 (#120) ───────────────────────────────────
#include "integrated_bringup/support/wbc_reduced_dynamics_provider.hpp"

#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
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

}  // namespace integrated_bringup
