// ── ClosedChainHandle 구현 ──────────────────────────────────────────────────────
#include "rtc_urdf_bridge/closed_chain_handle.hpp"

#include "rtc_urdf_bridge/loop_verification.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#pragma GCC diagnostic pop

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace rtc_urdf_bridge {

namespace {

constexpr double kDriftEps = 1e-6;  // γ = J̇c·v 중앙차분 스텝
// Jc_D 최소 특이값 특이 판정 임계. RT @ref RtClosedChainHandle 과 공유 (단일 출처:
// loop_verification.hpp) — 한쪽만 튜닝해 semantics 가 desync 되는 것을 방지한다.
constexpr double kSingularSvThreshold = kClosedChainSingularSvThreshold;
// G 축약 map 의 damped pinv 정규화 λ. σ⁺=σ/(σ²+λ²) 의 최대 증폭 1/(2λ) 를 유계화하기 위해
// 특이 임계와 동급으로 고정한다 → 미-flag(σ≥λ) 구간 증폭 ≤ 1/(2λ). Newton 위치 사영의
// damping(수렴 튜닝) 을 재사용하지 않는다 (출력 크기 안전이라는 다른 목적).
constexpr double kReductionDamping = kSingularSvThreshold;

/// damped(Tikhonov) pseudo-inverse via SVD — NUM-1 (특이형상에서 폭주 억제).
/// σ⁺ = σ / (σ² + λ²). rank / 최소 특이값도 함께 반환.
Eigen::MatrixXd DampedPinv(const Eigen::MatrixXd& A, double lambda, double& sigma_min) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd& sv = svd.singularValues();
  sigma_min = (sv.size() > 0) ? sv(sv.size() - 1) : 0.0;
  Eigen::VectorXd sinv(sv.size());
  const double l2 = lambda * lambda;
  for (Eigen::Index i = 0; i < sv.size(); ++i) {
    sinv(i) = sv(i) / (sv(i) * sv(i) + l2);
  }
  return svd.matrixV() * sinv.asDiagonal() * svd.matrixU().transpose();
}

/// null model 조기 차단 (init-list 의 *model_ deref 전에 검사).
const pinocchio::Model& RequireModel(const std::shared_ptr<const pinocchio::Model>& m) {
  if (!m) {
    throw std::invalid_argument("ClosedChainHandle: model 이 null 입니다.");
  }
  return *m;
}

}  // namespace

// ── 생성자 ──────────────────────────────────────────────────────────────────

ClosedChainHandle::ClosedChainHandle(const PinocchioModelBuilder& builder)
    : ClosedChainHandle(builder.GetFullModel(), builder.GetConstraintModels(),
                        builder.GetClosureActuatedJointIds(), builder.GetClosureReferenceConfig()) {
}

ClosedChainHandle::ClosedChainHandle(std::shared_ptr<const pinocchio::Model> model,
                                     std::vector<pinocchio::RigidConstraintModel> constraints,
                                     std::vector<pinocchio::JointIndex> actuated_joint_ids,
                                     Eigen::VectorXd q_ref)
    : model_(std::move(model)),
      data_(RequireModel(model_)),
      constraints_(std::move(constraints)),
      actuated_joint_ids_(std::move(actuated_joint_ids)) {
  nv_ = model_->nv;
  m_ = TotalConstraintDim(constraints_);
  identity_ = constraints_.empty();
  q_full_ = (q_ref.size() == model_->nq) ? std::move(q_ref) : pinocchio::neutral(*model_);
  Initialize();
}

// ── 초기화: 독립/종속 인덱스 분할 + well-posed 검증 + 버퍼 사전 할당 ──────────

void ClosedChainHandle::Initialize() {
  const pinocchio::Model& model = *model_;

  // 독립(구동) 관절 집합 결정.
  //   - 구속 없음(serial 등가): 모든 movable 관절이 독립 → 항등 축약.
  //   - 구속 있음: sidecar actuated_joint_ids 가 독립, 나머지 movable = 종속(loop-passive).
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
        continue;  // universe / invalid
      }
      indep_joints.push_back(jid);
    }
  }

  // 독립 관절 슬롯 구성 (단일-DoF 만 지원: q_a/v_a 를 스칼라로 해석).
  independent_.reserve(indep_joints.size());
  for (const auto jid : indep_joints) {
    const auto jidx = static_cast<std::size_t>(jid);
    if (model.nvs[jidx] != 1) {
      throw std::runtime_error("ClosedChainHandle: 독립 관절 '" + model.names[jidx] +
                               "' 이 단일-DoF(nv==1) 가 아닙니다 — 스칼라 q_a 입력으로 표현 불가.");
    }
    IndependentSlot slot;
    slot.jid = jid;
    slot.q_idx = static_cast<int>(model.idx_qs[jidx]);
    slot.v_idx = static_cast<int>(model.idx_vs[jidx]);
    slot.is_continuous = (model.nqs[jidx] == 2);  // continuous 관절 (cos,sin)
    independent_.push_back(slot);
  }

  // velocity 인덱스 오름차순 정렬 → q_a/v_a 입력 순서 확정.
  std::sort(independent_.begin(), independent_.end(),
            [](const IndependentSlot& a, const IndependentSlot& b) { return a.v_idx < b.v_idx; });

  indep_v_idx_.reserve(independent_.size());
  for (const auto& s : independent_) {
    indep_v_idx_.push_back(s.v_idx);
  }
  n_a_ = static_cast<int>(indep_v_idx_.size());

  // 종속 velocity 인덱스 = 전체 − 독립 (오름차순).
  {
    std::vector<char> is_indep(static_cast<std::size_t>(nv_), 0);
    for (const int vi : indep_v_idx_) {
      is_indep[static_cast<std::size_t>(vi)] = 1;
    }
    dep_v_idx_.reserve(static_cast<std::size_t>(nv_ - n_a_));
    for (int vi = 0; vi < nv_; ++vi) {
      if (!is_indep[static_cast<std::size_t>(vi)]) {
        dep_v_idx_.push_back(vi);
      }
    }
  }

  // 종속 DoF 수 = nv − n_a. Jc_D 는 (m × dep) 로 tall 일 수 있다 (planar contact_3d 처럼
  // 구속이 redundant → rank < m). 축약은 n_a = nv − rank(Jc) 일 때 정의되며, rank 는
  // 형상 의존이라 여기서 정방 가정을 강제하지 않고 RebuildReducedDynamics 에서 damped
  // pseudo-inverse + singular flag 로 처리한다. 구속이 있는데 독립 좌표가 없으면 축약 불가.
  if (!identity_ && n_a_ == 0) {
    throw std::runtime_error(
        "ClosedChainHandle: 구속이 있으나 actuated(독립) 관절이 없습니다 — 축약 불가. "
        "sidecar actuation.actuated_joints 를 확인.");
  }
  // 구조적 well-posedness: 종속 DoF 수는 구속 rows m 를 넘을 수 없다 (넘으면 Jc_D 가
  // wide → 종속을 유일하게 풀 수 없어 reduction underdetermined; damped pinv 는 finite
  // 하나 silently 틀린 min-norm 을 낸다). actuated_joints 가 너무 적을 때 조기 차단.
  if (!identity_ && static_cast<int>(dep_v_idx_.size()) > m_) {
    throw std::runtime_error(
        "ClosedChainHandle: 종속 DoF(" + std::to_string(dep_v_idx_.size()) + ") > 구속 rows m(" +
        std::to_string(m_) +
        ") — reduction underdetermined. sidecar actuation.actuated_joints 에 독립 관절이 "
        "충분히 열거됐는지 확인.");
  }

  // 버퍼 사전 할당.
  v_full_ = Eigen::VectorXd::Zero(nv_);
  G_ = Eigen::MatrixXd::Zero(nv_, n_a_);
  M_a_ = Eigen::MatrixXd::Zero(n_a_, n_a_);
  g_a_ = Eigen::VectorXd::Zero(n_a_);
  h_a_ = Eigen::VectorXd::Zero(n_a_);
  J_full_ = Eigen::MatrixXd::Zero(6, nv_);

  if (identity_) {
    G_.setIdentity();  // serial 등가: v_full = v_a, 축약 = full
  }

  // 최초 Update 전에도 FK getter 가 q_ref(=현재 q_full_) 형상을 반환하도록 data_ 를 채운다
  // (미채움 시 oMf 가 identity → GetFramePlacement 가 (0,0,0) 반환하는 함정).
  pinocchio::computeJointJacobians(*model_, data_, q_full_);
  pinocchio::updateFramePlacements(*model_, data_);
}

// ── 갱신 ────────────────────────────────────────────────────────────────────

ClosedChainHandle::Status ClosedChainHandle::Update(std::span<const double> q_a,
                                                    std::span<const double> v_a) {
  const pinocchio::Model& model = *model_;
  const bool have_velocity = !v_a.empty();

  // 크기 계약: q_a 는 정확히 n_a, v_a 는 비어 있거나 n_a. 불일치는 off-by-one 프로그래밍
  // 오류이므로 조용히 truncate(→ 잘못된 형상을 converged=true 로 반환) 하지 않고 차단한다.
  if (static_cast<int>(q_a.size()) != n_a_) {
    throw std::invalid_argument("ClosedChainHandle::Update: q_a 크기(" +
                                std::to_string(q_a.size()) + ") != 독립 좌표 수 n_a(" +
                                std::to_string(n_a_) + ").");
  }
  if (have_velocity && static_cast<int>(v_a.size()) != n_a_) {
    throw std::invalid_argument("ClosedChainHandle::Update: v_a 크기(" +
                                std::to_string(v_a.size()) + ") != 독립 좌표 수 n_a(" +
                                std::to_string(n_a_) + ").");
  }

  // 직전 loop-consistent 해를 seed 로 두고 독립 슬롯만 측정값으로 덮어쓴다.
  Eigen::VectorXd q_seed = q_full_;
  const auto n = std::min<std::size_t>(q_a.size(), independent_.size());
  for (std::size_t i = 0; i < n; ++i) {
    const IndependentSlot& s = independent_[i];
    if (s.is_continuous) {
      q_seed[s.q_idx] = std::cos(q_a[i]);
      q_seed[s.q_idx + 1] = std::sin(q_a[i]);
    } else {
      q_seed[s.q_idx] = q_a[i];
    }
  }

  // passive DoF 사영 (구속 있을 때만). 미수렴/비유한 시 직전 해 hold.
  status_ = Status{};
  if (identity_) {
    q_full_ = std::move(q_seed);
    status_.converged = true;
  } else {
    const ProjectionResult res = ProjectPassiveToConstraint(model, data_, constraints_, q_seed,
                                                            actuated_joint_ids_, projection_opts_);
    status_.iterations = res.iterations;
    status_.closure_error = res.final_error;
    if (res.converged && res.q.allFinite()) {
      q_full_ = res.q;
      status_.converged = true;
    } else {
      status_.converged = false;  // q_full_ (직전 해) 유지
    }
  }

  // 독립 속도 벡터 조립 (v_full 은 RebuildReducedDynamics 에서 G v_a 로 채운다).
  Eigen::VectorXd v_indep = Eigen::VectorXd::Zero(n_a_);
  if (have_velocity) {
    const auto nvsz = std::min<std::size_t>(v_a.size(), independent_.size());
    for (std::size_t i = 0; i < nvsz; ++i) {
      v_indep[static_cast<Eigen::Index>(i)] = v_a[i];
    }
  }

  RebuildReducedDynamics(have_velocity, v_indep);
  return status_;
}

// ── 축약 동역학 재계산 ──────────────────────────────────────────────────────

void ClosedChainHandle::RebuildReducedDynamics(bool have_velocity, const Eigen::VectorXd& v_indep) {
  const pinocchio::Model& model = *model_;

  if (identity_) {
    // serial 등가: G = I, 사영/드리프트 없음.
    v_full_ = v_indep;
    pinocchio::crba(model, data_, q_full_);
    data_.M.triangularView<Eigen::StrictlyLower>() =
        data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    M_a_ = data_.M;
    pinocchio::computeGeneralizedGravity(model, data_, q_full_);
    g_a_ = data_.g;
    if (have_velocity) {
      pinocchio::nonLinearEffects(model, data_, q_full_, v_full_);
      h_a_ = data_.nle;
    } else {
      h_a_ = g_a_;
    }
  } else {
    // (1) 현재 형상의 Jc → 독립/종속 열 분할 → 속도 축약 map G.
    const ConstraintKinematics ck =
        ComputeConstraintKinematics(model, data_, constraints_, q_full_);
    const int dep = static_cast<int>(dep_v_idx_.size());  // = nv − n_a
    Eigen::MatrixXd Jc_I(m_, n_a_);
    Eigen::MatrixXd Jc_D(m_, dep);  // tall (m ≥ dep) 가능 — redundant contact_3d 등
    for (int k = 0; k < n_a_; ++k) {
      Jc_I.col(k) = ck.Jc.col(indep_v_idx_[static_cast<std::size_t>(k)]);
    }
    for (int r = 0; r < dep; ++r) {
      Jc_D.col(r) = ck.Jc.col(dep_v_idx_[static_cast<std::size_t>(r)]);
    }
    // Jc_D⁺ (dep × m): tall Jc_D 의 damped left pseudo-inverse. 종속 블록이 rank 를
    // 잃으면(특이 조립형상) sigma_min→0 로 flag. dep==0(종속 없음)은 역행렬 불필요·비특이.
    double sigma_min = std::numeric_limits<double>::infinity();
    Eigen::MatrixXd Jc_D_inv;
    if (dep > 0) {
      Jc_D_inv = DampedPinv(Jc_D, kReductionDamping, sigma_min);
    }
    status_.singular = (sigma_min < kSingularSvThreshold);

    // v_D = −Jc_D⁺ Jc_I v_I → G 의 종속 블록 = −Jc_D⁺ Jc_I (dep × n_a), 독립 블록 = I.
    G_.setZero();
    for (int k = 0; k < n_a_; ++k) {
      G_(indep_v_idx_[static_cast<std::size_t>(k)], k) = 1.0;
    }
    if (dep > 0) {
      const Eigen::MatrixXd M_dep = -(Jc_D_inv * Jc_I);  // dep × n_a
      for (int k = 0; k < n_a_; ++k) {
        for (int r = 0; r < dep; ++r) {
          G_(dep_v_idx_[static_cast<std::size_t>(r)], k) = M_dep(r, k);
        }
      }
    }

    // (2) 구속-정합 full 속도.
    v_full_.noalias() = G_ * v_indep;

    // (3) 축약 관성 / 중력.
    pinocchio::crba(model, data_, q_full_);
    data_.M.triangularView<Eigen::StrictlyLower>() =
        data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    M_a_.noalias() = G_.transpose() * data_.M * G_;
    M_a_ = 0.5 * (M_a_ + M_a_.transpose()).eval();  // 부동소수점 비대칭 제거 (하류 LLT/LDLT)
    pinocchio::computeGeneralizedGravity(model, data_, q_full_);
    g_a_.noalias() = G_.transpose() * data_.g;

    // (4) 축약 비선형효과 h_a = Gᵀ·rnea(q, v_full, a_drift).
    //     a_drift: a_I=0 일 때 구속-정합 가속 (종속 = −Jc_D⁻¹·γ, γ = J̇c·v_full).
    //     γ 는 동일 Jc 함수의 중앙차분으로 산출 → Jc 규약과 정합 (round-trip 검증됨).
    if (have_velocity) {
      Eigen::VectorXd a_drift = Eigen::VectorXd::Zero(nv_);
      if (dep > 0) {
        const Eigen::VectorXd dq = kDriftEps * v_full_;
        const Eigen::VectorXd q_plus = pinocchio::integrate(model, q_full_, dq);
        const Eigen::VectorXd q_minus = pinocchio::integrate(model, q_full_, (-dq).eval());
        const ConstraintKinematics ck_p =
            ComputeConstraintKinematics(model, data_, constraints_, q_plus);
        const ConstraintKinematics ck_m =
            ComputeConstraintKinematics(model, data_, constraints_, q_minus);
        const Eigen::VectorXd gamma =
            (ck_p.Jc * v_full_ - ck_m.Jc * v_full_) / (2.0 * kDriftEps);  // m
        const Eigen::VectorXd a_dep = -(Jc_D_inv * gamma);                // dep
        for (int r = 0; r < dep; ++r) {
          a_drift[dep_v_idx_[static_cast<std::size_t>(r)]] = a_dep[r];
        }
      }
      pinocchio::rnea(model, data_, q_full_, v_full_, a_drift);
      h_a_.noalias() = G_.transpose() * data_.tau;
    } else {
      h_a_ = g_a_;
    }
  }

  // (5) FK/Jacobian getter 를 위해 data_ 를 q_full_ 상태로 복원
  //     (드리프트 유한차분이 data_.J/oMi 를 q_plus/minus 로 오염시켰을 수 있음).
  pinocchio::computeJointJacobians(model, data_, q_full_);
  pinocchio::updateFramePlacements(model, data_);
}

// ── 축약 결과 접근 ──────────────────────────────────────────────────────────

Eigen::Ref<const Eigen::MatrixXd> ClosedChainHandle::GetMassMatrix() const noexcept {
  return M_a_;
}

Eigen::Ref<const Eigen::VectorXd> ClosedChainHandle::GetGeneralizedGravity() const noexcept {
  return g_a_;
}

Eigen::Ref<const Eigen::VectorXd> ClosedChainHandle::GetNonLinearEffects() const noexcept {
  return h_a_;
}

void ClosedChainHandle::GetFrameJacobian(pinocchio::FrameIndex frame_id,
                                         pinocchio::ReferenceFrame ref_frame,
                                         Eigen::Ref<Eigen::MatrixXd> J_out) {
  J_full_.setZero();
  pinocchio::getFrameJacobian(*model_, data_, frame_id, ref_frame, J_full_);
  J_out.noalias() = J_full_ * G_;  // 6 × n_a
}

Eigen::Ref<const Eigen::MatrixXd> ClosedChainHandle::GetReductionMap() const noexcept {
  return G_;
}

const pinocchio::SE3& ClosedChainHandle::GetFramePlacement(
    pinocchio::FrameIndex frame_id) const noexcept {
  return data_.oMf[frame_id];
}

Eigen::Vector3d ClosedChainHandle::GetFramePosition(pinocchio::FrameIndex frame_id) const noexcept {
  return data_.oMf[frame_id].translation();
}

Eigen::Matrix3d ClosedChainHandle::GetFrameRotation(pinocchio::FrameIndex frame_id) const noexcept {
  return data_.oMf[frame_id].rotation();
}

const Eigen::VectorXd& ClosedChainHandle::GetFullConfiguration() const noexcept {
  return q_full_;
}

// ── 메타데이터 ──────────────────────────────────────────────────────────────

int ClosedChainHandle::nv_independent() const noexcept {
  return n_a_;
}

int ClosedChainHandle::nv_full() const noexcept {
  return nv_;
}

int ClosedChainHandle::nq_full() const noexcept {
  return model_->nq;
}

int ClosedChainHandle::constraint_dim() const noexcept {
  return m_;
}

std::vector<std::string> ClosedChainHandle::GetIndependentJointNames() const {
  std::vector<std::string> names;
  names.reserve(independent_.size());
  for (const auto& s : independent_) {
    names.push_back(model_->names[static_cast<std::size_t>(s.jid)]);
  }
  return names;
}

pinocchio::FrameIndex ClosedChainHandle::GetFrameId(std::string_view frame_name) const noexcept {
  if (!model_->existFrame(std::string(frame_name))) {
    return 0;
  }
  return model_->getFrameId(std::string(frame_name));
}

const pinocchio::Model& ClosedChainHandle::GetModel() const noexcept {
  return *model_;
}

bool ClosedChainHandle::IsFrameDownstreamOfLoop(pinocchio::FrameIndex frame_id) const noexcept {
  const pinocchio::Model& model = *model_;
  // 구속 없음(serial 등가) 이면 loop-passive 가 없어 frozen-loop 근사 자체가 없다.
  if (identity_ || frame_id >= model.frames.size()) {
    return false;
  }
  // frame → 부모 joint → universe 까지의 support(조상) 경로.
  const auto parent_joint = model.frames[frame_id].parentJoint;
  const auto& support = model.supports[parent_joint];
  // 조상 중 movable & non-actuated(=loop-passive) 가 하나라도 있으면 하류다.
  for (const auto jid : support) {
    const auto jidx = static_cast<std::size_t>(jid);
    if (jid == 0 || model.nvs[jidx] == 0) {
      continue;  // universe / fixed
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
