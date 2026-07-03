// ── RtClosedChainHandle 구현 ─────────────────────────────────────────────────
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"

#include "rtc_urdf_bridge/loop_verification.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace rtc_urdf_bridge {

namespace {

const pinocchio::Model& RequireModel(const std::shared_ptr<const pinocchio::Model>& m) {
  if (!m) {
    throw std::invalid_argument("RtClosedChainHandle: model 이 null 입니다.");
  }
  return *m;
}

// 범위 밖 frame_id 접근 시 반환할 항등 pose. namespace-scope const → static-init 시 1회 구성,
// RT 핫패스에서 local-static guard(락) 없이 참조만 한다.
const pinocchio::SE3 kIdentitySE3 = pinocchio::SE3::Identity();

}  // namespace

// ── 생성자 ──────────────────────────────────────────────────────────────────

RtClosedChainHandle::RtClosedChainHandle(std::shared_ptr<const pinocchio::Model> model,
                                         std::vector<pinocchio::RigidConstraintModel> constraints,
                                         std::vector<pinocchio::JointIndex> actuated_joint_ids,
                                         Eigen::VectorXd q_seed, int num_iterations,
                                         double projection_damping, double reduction_damping)
    : model_(std::move(model)),
      data_(RequireModel(model_)),
      constraints_(std::move(constraints)),
      actuated_joint_ids_(std::move(actuated_joint_ids)),
      num_iterations_(std::max(0, num_iterations)),
      proj_lambda2_(projection_damping * projection_damping),
      reduction_lambda_(reduction_damping) {
  nv_ = model_->nv;
  m_ = TotalConstraintDim(constraints_);
  identity_ = constraints_.empty();
  // q_seed: 빈 벡터(size 0)는 명시적 "neutral 사용" 신호. 그 외 크기가 nq 와 다르면 misconfig
  // 이므로 조용히 neutral 로 떨어뜨리지 않고 throw 한다 (Configure 가 catch → graceful serial
  // fallback + WARN). 대칭 링키지의 neutral 은 특이 조립일 수 있어, silent fallback 은 매 tick 신뢰
  // 불가 → fingertip 영구 저하로 이어진다.
  if (q_seed.size() != 0 && q_seed.size() != model_->nq) {
    throw std::invalid_argument("RtClosedChainHandle: q_seed 크기(" +
                                std::to_string(q_seed.size()) + ") 가 model.nq(" +
                                std::to_string(model_->nq) + ") 와 다릅니다.");
  }
  q_full_ = (q_seed.size() == model_->nq) ? std::move(q_seed) : pinocchio::neutral(*model_);
  Initialize();
}

// ── 초기화: 독립/종속 분할 + well-posedness 검증 + 버퍼 사전 할당 (non-RT) ─────

void RtClosedChainHandle::Initialize() {
  const pinocchio::Model& model = *model_;

  // 독립(구동) 관절 집합. serial 등가면 모든 movable = 독립, 아니면 sidecar 목록.
  std::vector<pinocchio::JointIndex> indep_joints;
  if (identity_) {
    for (int jid = 1; jid < model.njoints; ++jid) {
      if (model.nvs[static_cast<std::size_t>(jid)] > 0) {
        indep_joints.push_back(static_cast<pinocchio::JointIndex>(jid));
      }
    }
  } else {
    for (const auto jid : actuated_joint_ids_) {
      if (jid == 0 || jid >= static_cast<pinocchio::JointIndex>(model.njoints)) {
        continue;
      }
      indep_joints.push_back(jid);
    }
  }

  // 독립 슬롯 (단일-DoF 만 지원 — 스칼라 q_a).
  independent_.reserve(indep_joints.size());
  for (const auto jid : indep_joints) {
    const auto jidx = static_cast<std::size_t>(jid);
    if (model.nvs[jidx] != 1) {
      throw std::runtime_error("RtClosedChainHandle: 독립 관절 '" + model.names[jidx] +
                               "' 이 단일-DoF(nv==1) 가 아닙니다 — 스칼라 q_a 입력으로 표현 불가.");
    }
    IndependentSlot slot;
    slot.jid = jid;
    slot.q_idx = static_cast<int>(model.idx_qs[jidx]);
    slot.v_idx = static_cast<int>(model.idx_vs[jidx]);
    slot.is_continuous = (model.nqs[jidx] == 2);
    independent_.push_back(slot);
  }
  std::sort(independent_.begin(), independent_.end(),
            [](const IndependentSlot& a, const IndependentSlot& b) { return a.v_idx < b.v_idx; });
  n_a_ = static_cast<int>(independent_.size());

  // 종속(=passive) velocity 인덱스 = 전체 − 독립 (오름차순).
  {
    std::vector<char> is_indep(static_cast<std::size_t>(nv_), 0);
    for (const auto& s : independent_) {
      is_indep[static_cast<std::size_t>(s.v_idx)] = 1;
    }
    dep_v_idx_.reserve(static_cast<std::size_t>(nv_ - n_a_));
    for (int vi = 0; vi < nv_; ++vi) {
      if (!is_indep[static_cast<std::size_t>(vi)]) {
        dep_v_idx_.push_back(vi);
      }
    }
  }
  dep_ = static_cast<int>(dep_v_idx_.size());

  // well-posedness (ClosedChainHandle 과 동일 계약).
  if (!identity_ && n_a_ == 0) {
    throw std::runtime_error(
        "RtClosedChainHandle: 구속이 있으나 actuated(독립) 관절이 없습니다 — 축약 불가.");
  }
  if (!identity_ && dep_ > m_) {
    throw std::runtime_error("RtClosedChainHandle: 종속 DoF(" + std::to_string(dep_) +
                             ") > 구속 rows m(" + std::to_string(m_) +
                             ") — reduction underdetermined. actuated_joints 확인.");
  }

  // 사전 할당.
  q_work_ = q_full_;
  q_next_ = q_full_;
  dq_ = Eigen::VectorXd::Zero(nv_);
  phi_ = Eigen::VectorXd::Zero(m_);
  Jc_ = Eigen::MatrixXd::Zero(m_, nv_);
  scratch_.Resize(nv_);

  Jc_free_ = Eigen::MatrixXd::Zero(m_, std::max(dep_, 1));
  JJt_ = Eigen::MatrixXd::Zero(std::max(m_, 1), std::max(m_, 1));
  y_ = Eigen::VectorXd::Zero(std::max(m_, 1));
  dv_free_ = Eigen::VectorXd::Zero(std::max(dep_, 1));
  ldlt_proj_ = Eigen::LDLT<Eigen::MatrixXd>(std::max(m_, 1));

  G_ = Eigen::MatrixXd::Zero(nv_, std::max(n_a_, 1));
  Jc_I_ = Eigen::MatrixXd::Zero(m_, std::max(n_a_, 1));
  A_dep_ = Eigen::MatrixXd::Zero(std::max(dep_, 1), std::max(dep_, 1));
  B_dep_ = Eigen::MatrixXd::Zero(std::max(dep_, 1), std::max(n_a_, 1));
  M_dep_ = Eigen::MatrixXd::Zero(std::max(dep_, 1), std::max(n_a_, 1));
  ldlt_G_ = Eigen::LDLT<Eigen::MatrixXd>(std::max(dep_, 1));

  J_full_ = Eigen::MatrixXd::Zero(6, nv_);

  // 독립 블록 = I (정적). serial 등가면 G = I 로 고정.
  G_.setZero();
  for (int k = 0; k < n_a_; ++k) {
    G_(independent_[static_cast<std::size_t>(k)].v_idx, k) = 1.0;
  }

  // 최초 Update 전에도 FK getter 가 q_full_ 형상을 반환하도록 data_ 를 채운다.
  pinocchio::computeJointJacobians(model, data_, q_full_);
  pinocchio::updateFramePlacements(model, data_);
}

// ── 구속 kinematics (RT-safe: preallocated phi_/Jc_ + 공용 FillConstraintKinematicsRow) ──

void RtClosedChainHandle::ComputeConstraintKinematicsRt(const Eigen::VectorXd& q) noexcept {
  const pinocchio::Model& model = *model_;
  pinocchio::computeJointJacobians(model, data_, q);  // data_.oMi, data_.J 갱신

  int row = 0;
  for (const auto& cm : constraints_) {
    const int dim = ConstraintDim(cm);
    // non-RT ComputeConstraintKinematics 와 동일 규약(단일 출처) — preallocated scratch 로 RT-safe.
    FillConstraintKinematicsRow(model, data_, cm, row, dim, scratch_, phi_, Jc_);
    row += dim;
  }
}

// ── G reduction map 재계산 (damped 정규방정식 left-pinv, dep×dep LDLT) ──────────

void RtClosedChainHandle::RebuildReductionMap() noexcept {
  if (identity_) {
    status_.singular = false;
    return;  // G = I 고정 (Initialize 에서 설정, 변하지 않음)
  }
  // 현재 q_full_ 형상의 Jc → 독립/종속 열 분할. (Update 가 마지막에 q_full_ 로 채워둔 Jc_ 사용.)
  for (int k = 0; k < n_a_; ++k) {
    Jc_I_.col(k) = Jc_.col(independent_[static_cast<std::size_t>(k)].v_idx);
  }
  if (dep_ == 0) {
    status_.singular = false;
    return;  // 종속 없음 → G = I (독립 블록만)
  }
  for (int r = 0; r < dep_; ++r) {
    Jc_free_.col(r) = Jc_.col(dep_v_idx_[static_cast<std::size_t>(r)]);  // Jc_D
  }

  // damped 정규방정식 left-pinv: v_D = −(Jc_Dᵀ Jc_D + λ²I)⁻¹ Jc_Dᵀ Jc_I v_I.
  A_dep_.noalias() = Jc_free_.transpose() * Jc_free_;  // 무damping normal matrix
  B_dep_.noalias() = Jc_free_.transpose() * Jc_I_;

  // 특이 판정은 **무damping** A = Jc_Dᵀ Jc_D 의 최소 pivot ≈ σ_min(Jc_D)² 으로 한다. 아래
  // solve 는 λ² 를 더한 damped A 로 하므로 min pivot 이 λ² 로 바닥에 깔려(→ 항상 σ≈λ) 임계를
  // 못 넘는 문제가 있어, damping 전에 먼저 factor 해 honest σ_min 대리를 얻는다.
  //
  // ⚠ 정밀도 한계 (RT 제약상 SVD/QR 불가 — 반복·할당): normal matrix 를 명시 형성(Jc_Dᵀ Jc_D)
  // 하면 condition number 가 제곱되고, 계산된 최소 고유값에 ~u·σ_max(Jc_D)² (u≈2.2e-16) 크기의
  // 절대 바닥오차가 낀다. 즉 이 σ_min 대리는 σ_min(Jc_D) ≫ √u·σ_max(Jc_D) (≈1.5e-8·σ_max) 일
  // 때만 신뢰할 수 있다. 손·소형 링키지(σ_max≈O(1)) 는 바닥오차 ~2e-16 ≪ 임계(1e-6) 이라
  // 안전하나, σ_max≳30 인 대형 링키지(긴 lever-arm, SI m)에서는 σ_min≈1e-6 부근에서 대리가
  // noise-dominated 가 되어 flag 가 tick 마다 흔들리거나 false-negative(garbage J_a 를 신뢰) 가
  // 날 수 있다. 그런 스케일을 배선할 때는 상대(조건수) 판정으로 교체하거나 λ 재튜닝 필요.
  ldlt_G_.compute(A_dep_);
  const double min_pivot = ldlt_G_.vectorD().minCoeff();
  const double sigma_proxy = min_pivot > 0.0 ? std::sqrt(min_pivot) : 0.0;
  status_.singular = sigma_proxy < kClosedChainSingularSvThreshold;

  // 안정적 solve 를 위해 λ² 가산 후 재factor (특이형상에서 A0 는 semidefinite → 직접 solve 불안정).
  A_dep_.diagonal().array() += reduction_lambda_ * reduction_lambda_;
  ldlt_G_.compute(A_dep_);
  M_dep_ = ldlt_G_.solve(B_dep_);  // A⁻¹ B (dep × n_a), presized → 재할당 없음

  // G 의 종속 블록 = −A⁻¹B, 독립 블록 = I (Initialize 에서 설정, 불변).
  for (int k = 0; k < n_a_; ++k) {
    for (int r = 0; r < dep_; ++r) {
      G_(dep_v_idx_[static_cast<std::size_t>(r)], k) = -M_dep_(r, k);
    }
  }
}

// ── 갱신 (RT-safe) ──────────────────────────────────────────────────────────

RtClosedChainHandle::Status RtClosedChainHandle::Update(std::span<const double> q_a) noexcept {
  const pinocchio::Model& model = *model_;
  status_ = Status{};

  // 크기 불일치는 RT 에서 throw 대신 hold (직전 해 유지).
  if (static_cast<int>(q_a.size()) != n_a_) {
    status_.held = true;
    status_.closure_error =
        std::numeric_limits<double>::infinity();  // held → 임계 비교가 안전히 실패
    return status_;
  }

  // warm-start: 직전 loop-consistent 해를 seed, 독립 슬롯만 측정값으로 덮어쓴다.
  q_work_ = q_full_;
  for (std::size_t i = 0; i < independent_.size(); ++i) {
    const IndependentSlot& s = independent_[i];
    if (s.is_continuous) {
      q_work_[s.q_idx] = std::cos(q_a[i]);
      q_work_[s.q_idx + 1] = std::sin(q_a[i]);
    } else {
      q_work_[s.q_idx] = q_a[i];
    }
  }

  if (identity_) {
    // 구속 없음 → 사영 불필요, FK passthrough. 단 비유한 q(측정 NaN/Inf)는 직전 유효 해로 hold —
    // 비-identity 경로(아래 allFinite guard)와 대칭. NaN 이 held=false 로 누출되는 것을 막는다.
    if (!q_work_.allFinite()) {
      status_.held = true;
      status_.closure_error = std::numeric_limits<double>::infinity();
      pinocchio::computeJointJacobians(model, data_, q_full_);
      pinocchio::updateFramePlacements(model, data_);
      return status_;
    }
    // data_ 를 q_full_ 로 채우고 반환.
    q_full_ = q_work_;
    status_.closure_error = 0.0;
    pinocchio::computeJointJacobians(model, data_, q_full_);
    pinocchio::updateFramePlacements(model, data_);
    return status_;  // G = I 고정, singular=false
  }

  // 고정 K Newton 스텝 (passive=종속 열만 자유; actuated 열 증분 0 → q 불변).
  for (int iter = 0; iter < num_iterations_; ++iter) {
    ComputeConstraintKinematicsRt(q_work_);  // computeJointJacobians(q_work_) + phi_/Jc_
    for (int r = 0; r < dep_; ++r) {
      Jc_free_.col(r) = Jc_.col(dep_v_idx_[static_cast<std::size_t>(r)]);  // Jc_D
    }
    // dv_free = −Jc_freeᵀ (Jc_free Jc_freeᵀ + λ²I)⁻¹ φ.
    JJt_.noalias() = Jc_free_ * Jc_free_.transpose();
    JJt_.diagonal().array() += proj_lambda2_;
    ldlt_proj_.compute(JJt_);
    y_ = ldlt_proj_.solve(phi_);
    dv_free_.noalias() = -Jc_free_.transpose() * y_;
    // scatter → 전체 tangent (고정 열 = 0 → integrate 시 불변).
    dq_.setZero();
    for (int r = 0; r < dep_; ++r) {
      dq_[dep_v_idx_[static_cast<std::size_t>(r)]] = dv_free_[r];
    }
    pinocchio::integrate(model, q_work_, dq_, q_next_);
    q_work_ = q_next_;
  }
  // 최종 형상에서 closure residual + Jc (G 재계산에 재사용). data_ 도 q_work_ 로 채워짐.
  ComputeConstraintKinematicsRt(q_work_);
  status_.closure_error = phi_.norm();

  // NaN/비유한 guard → 직전 해 hold. data_ 가 비유한 q_work_ 로 오염됐으므로 직전 유효
  // q_full_ 로 FK 를 복원해 getter 가 직전 해를 돌려주도록 한다 (G_ 는 미갱신 → 직전 값 유지).
  if (!q_work_.allFinite() || !std::isfinite(status_.closure_error)) {
    status_.held = true;
    status_.closure_error =
        std::numeric_limits<double>::infinity();  // held → 임계 비교가 안전히 실패
    pinocchio::computeJointJacobians(model, data_, q_full_);
    pinocchio::updateFramePlacements(model, data_);
    return status_;
  }

  q_full_ = q_work_;
  // data_ 는 이미 q_full_(==q_work_) 형상 (마지막 ComputeConstraintKinematicsRt). oMf 만 갱신.
  pinocchio::updateFramePlacements(model, data_);
  RebuildReductionMap();  // Jc_ (q_full_ 형상) 재사용
  return status_;
}

// ── FK / Jacobian getter ────────────────────────────────────────────────────

const pinocchio::SE3& RtClosedChainHandle::GetFramePlacement(
    pinocchio::FrameIndex frame_id) const noexcept {
  if (frame_id >= data_.oMf.size()) {
    return kIdentitySE3;  // 범위 밖(stale/foreign id) → OOB 대신 항등 반환
  }
  return data_.oMf[frame_id];
}

Eigen::Vector3d RtClosedChainHandle::GetFramePosition(
    pinocchio::FrameIndex frame_id) const noexcept {
  if (frame_id >= data_.oMf.size()) {
    return Eigen::Vector3d::Zero();
  }
  return data_.oMf[frame_id].translation();
}

Eigen::Matrix3d RtClosedChainHandle::GetFrameRotation(
    pinocchio::FrameIndex frame_id) const noexcept {
  if (frame_id >= data_.oMf.size()) {
    return Eigen::Matrix3d::Identity();
  }
  return data_.oMf[frame_id].rotation();
}

const Eigen::VectorXd& RtClosedChainHandle::GetFullConfiguration() const noexcept {
  return q_full_;
}

void RtClosedChainHandle::GetFrameJacobian(pinocchio::FrameIndex frame_id,
                                           pinocchio::ReferenceFrame ref_frame,
                                           Eigen::Ref<Eigen::MatrixXd> J_out) noexcept {
  J_full_.setZero();
  pinocchio::getFrameJacobian(*model_, data_, frame_id, ref_frame, J_full_);
  J_out.noalias() = J_full_ * G_;  // 6 × n_a
}

Eigen::Ref<const Eigen::MatrixXd> RtClosedChainHandle::GetReductionMap() const noexcept {
  return G_;
}

// ── 메타데이터 ──────────────────────────────────────────────────────────────

int RtClosedChainHandle::nq_full() const noexcept {
  return model_->nq;
}

std::vector<std::string> RtClosedChainHandle::GetIndependentJointNames() const {
  std::vector<std::string> names;
  names.reserve(independent_.size());
  for (const auto& s : independent_) {
    names.push_back(model_->names[static_cast<std::size_t>(s.jid)]);
  }
  return names;
}

pinocchio::FrameIndex RtClosedChainHandle::GetFrameId(std::string_view frame_name) const noexcept {
  if (!model_->existFrame(std::string(frame_name))) {
    return 0;
  }
  return model_->getFrameId(std::string(frame_name));
}

bool RtClosedChainHandle::IsFrameDownstreamOfLoop(pinocchio::FrameIndex frame_id) const noexcept {
  const pinocchio::Model& model = *model_;
  if (identity_ || frame_id >= model.frames.size()) {
    return false;
  }
  const auto parent_joint = model.frames[frame_id].parentJoint;
  const auto& support = model.supports[parent_joint];
  for (const auto jid : support) {
    const auto jidx = static_cast<std::size_t>(jid);
    if (jid == 0 || model.nvs[jidx] == 0) {
      continue;
    }
    const bool is_actuated = std::find(actuated_joint_ids_.begin(), actuated_joint_ids_.end(),
                                       jid) != actuated_joint_ids_.end();
    if (!is_actuated) {
      return true;
    }
  }
  return false;
}

}  // namespace rtc_urdf_bridge
