#pragma once

#include <Eigen/Core>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

// #unified-kindyn Phase 1: PinocchioCache / ReducedDynamicsProvider 는 rtc_urdf_bridge 로 이관됨.
// 아래 alias 재export 로 `rtc::tsid::PinocchioCache` / `rtc::tsid::ReducedDynamicsProvider` 심볼
// 을 유지한다 (rtc_tsid 는 이제 소비자 — ARCH-2 순방향).
#include "rtc_urdf_bridge/pinocchio_cache.hpp"
#include "rtc_urdf_bridge/reduced_dynamics_provider.hpp"

namespace rtc::tsid {

// ────────────────────────────────────────────────
// 로봇 모델 정보 (init 시 Pinocchio Model + YAML로부터 구성)
// ────────────────────────────────────────────────
struct RobotModelInfo {
  int nq{0};          // generalized coordinate dim
  int nv{0};          // generalized velocity dim
  int n_actuated{0};  // actuated DoF count
  bool floating_base{false};

  Eigen::MatrixXd S;        // [n_actuated × nv] selection matrix
  Eigen::VectorXd tau_max;  // [n_actuated]
  Eigen::VectorXd tau_min;  // [n_actuated]
  Eigen::VectorXd q_upper;  // [nq]
  Eigen::VectorXd q_lower;  // [nq]
  Eigen::VectorXd v_max;    // [nv]

  // Pinocchio model + YAML config로부터 구성
  void Build(const pinocchio::Model& model, const YAML::Node& config);
};

// ────────────────────────────────────────────────
// Contact 설정 (YAML로부터 로드, init 시 1회)
// ────────────────────────────────────────────────
struct ContactConfig {
  std::string name;
  std::string frame_name;
  int frame_id{0};     // Pinocchio frame ID (init 시 resolve)
  int contact_dim{3};  // 3 (point) or 6 (surface)
  double friction_coeff{0.7};
  int friction_faces{4};  // linearized cone 면 수

  // Surface contact (contact_dim == 6) 전용 — CoP rectangle 반치수 & yaw moment 한계.
  // Point contact 에서는 무시.
  // 좌표: contact frame +z = normal, patch 는 xy 평면. CoP_x ∈ [-l_x, l_x],
  // CoP_y ∈ [-l_y, l_y], |m_z| ≤ μ_τ · f_z.
  //
  // patch=0 / μ_τ=0 default 의 *결과*: f_z 열은 0 이 되지만 moment 열 ±1 은 살아
  // `m_x = m_y = m_z = 0` 의 stiff equality 가 됨 — 즉 "moment 강제 0 인 point-like
  // restraint". ineq_dim 일관성을 위해 고의로 도입 (vacuous 행 아님).
  double patch_half_length_x{0.0};
  double patch_half_length_y{0.0};
  double torsional_friction_coeff{0.0};

  // Stage A-4: world-frame contact normal seed. Default world +Z preserves
  // pre-A-4 fixed-normal behaviour. Runtime override via
  // `ContactState::UpdateNormal(idx, n_raw, alpha)`; YAML override via
  // optional `normal: [x, y, z]` per-contact key.
  Eigen::Vector3d default_normal{0.0, 0.0, 1.0};
};

struct ContactManagerConfig {
  std::vector<ContactConfig> contacts;
  int max_contacts{0};
  int max_contact_vars{0};  // Σ contact_dim_i

  // Stage A-4: global low-pass filter coefficient for runtime normal updates
  // via ContactState::UpdateNormal. Default 0.1 (~10-tick rise at 500 Hz).
  // YAML: `tsid.contacts_normal_filter_alpha: <α>` (sibling of `contacts:`).
  double normal_filter_alpha{0.1};

  // Stage C-0.1: set by Load() when any contact specifies both the canonical
  // key (friction_coeff / friction_faces) and its alias (mu / n_faces) with
  // disagreeing values. The loader has no logger; the consumer (controller)
  // reads this to WARN. The explicit canonical key always wins.
  bool friction_key_conflict{false};

  // YAML로부터 로드 + frame name → frame ID resolve
  void Load(const YAML::Node& config, const pinocchio::Model& model);
};

// ────────────────────────────────────────────────
// 런타임 Contact 상태 (매 tick 또는 phase 전환 시 갱신)
// ────────────────────────────────────────────────
// Stage A-5b: activation deadband — below this magnitude the contact is
// treated as inactive (zero-row in constraints, active flag flipped to
// false). Above it the contact contributes via sqrt(s) cost scaling.
inline constexpr double kActivationDeadband = 1e-3;

struct ContactState {
  struct Entry {
    int config_index{0};
    bool active{false};
    Eigen::Vector3d normal{0.0, 0.0, 1.0};  // world-frame 접촉 법선

    // Stage A-5b: continuous activation ∈ [0, 1]. 1 = fully active (cost +
    // constraints fully on), 0 = fully inactive (zero-row in everything,
    // λ_i drifts to 0 via Hessian regularization). Ramp progresses each
    // tick via UpdateActivation(dt) toward activation_target with rate
    // 1/t_ramp_sec.
    double activation{1.0};
    double activation_target{1.0};
    double t_ramp_sec{0.1};
  };

  std::vector<Entry> contacts;  // init 시 resize(max_contacts), 이후 크기 불변
  int active_count{0};
  int active_contact_vars{0};  // Σ active contact_dim

  // max_contacts 크기로 pre-allocate
  void Init(int max_contacts);

  // Stage A-4: seed per-entry normals from manager.contacts[i].default_normal.
  // Safe to call after Init(). RT-safe (no alloc).
  void SeedNormals(const ContactManagerConfig& manager) noexcept;

  // Stage A-4: RT-safe low-pass update of a single contact normal.
  //   n_filtered = normalize(α·n_raw + (1−α)·n_prev).
  // Out-of-range idx → no-op. Zero/NaN n_raw → no-op (keeps prior normal).
  // Caller threads typically a tactile-inferred or geometric estimate per tick.
  void UpdateNormal(int idx, const Eigen::Vector3d& n_raw, double alpha) noexcept;

  // Stage A-5b: phase-FSM-facing API to set the linear-ramp target.
  // RT-safe: clamps target to [0,1] and t_ramp to (0, ∞). Out-of-range idx
  // → no-op. Backward-compat: passing target=1.0 with active==true keeps
  // the legacy "instant on" via t_ramp=0+.
  void SetActivationTarget(int idx, double target, double t_ramp_sec) noexcept;

  // Stage A-5b: per-tick ramp progression. Linear toward activation_target
  // with rate 1/t_ramp_sec. dt non-positive → no-op. Also flips the legacy
  // `active : bool` field based on the kActivationDeadband threshold so
  // existing consumers (active_contact_vars, fast skip paths) stay in sync.
  void UpdateActivation(double dt) noexcept;

  // active contact 개수 및 active_contact_vars 재계산
  void RecomputeActive(const ContactManagerConfig& manager);

  // RT-safe active contact 순회 (no alloc)
  template <typename Func>
  void ForEachActive(const ContactManagerConfig& manager, Func&& fn) const {
    for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
      if (contacts[static_cast<size_t>(i)].active) {
        fn(i, manager.contacts[static_cast<size_t>(i)]);
      }
    }
  }
};

// ────────────────────────────────────────────────
// Pinocchio 계산 캐시 (매 tick 1회 갱신, task/constraint 공유)
// ────────────────────────────────────────────────
// #unified-kindyn Phase 1: PinocchioCache / ReducedDynamicsProvider 는 rtc_urdf_bridge 로 이관.
// 여기서는 심볼 호환을 위해 재export 한다 — 기존 `rtc::tsid::PinocchioCache` 사용처 무변경.
// 결합 해소: 이관된 PinocchioCache::Update(q,v) 는 ContactState 무의존(always-compute), Init 은
// contact frame id 리스트를 받는다. TSID 소비자는 ContactManagerConfig → id 리스트 변환에
// 아래 ContactFrameIds() 헬퍼를 쓴다.
using rtc_urdf_bridge::PinocchioCache;
using rtc_urdf_bridge::ReducedDynamicsProvider;

// ContactManagerConfig 의 contact frame id 를 PinocchioCache::Init 이 받는 리스트로 추출한다
// (소비자 인덱싱과 동일 순서 — cache.contact_frames[i] ↔ manager.contacts[i]).
[[nodiscard]] std::vector<pinocchio::FrameIndex> ContactFrameIds(
    const ContactManagerConfig& manager);

// ────────────────────────────────────────────────
// 제어 입출력
// ────────────────────────────────────────────────
struct ControlState {
  Eigen::VectorXd q;  // [nq]
  Eigen::VectorXd v;  // [nv]
  uint64_t timestamp_ns{0};
};

struct ControlReference {
  Eigen::VectorXd q_des;       // [nq]
  Eigen::VectorXd v_des;       // [nv]
  Eigen::VectorXd a_des;       // [nv] desired accel (feedforward)
  Eigen::VectorXd tau_ff;      // [n_actuated] feedforward torque
  Eigen::VectorXd lambda_des;  // [max_contact_vars] desired contact forces

  void Init(int nq, int nv, int n_actuated, int max_contact_vars);
};

struct CommandOutput {
  Eigen::VectorXd tau;         // [n_actuated]
  Eigen::VectorXd a_opt;       // [nv]
  Eigen::VectorXd lambda_opt;  // [max_contact_vars] (Stage A-5a: fixed-dim QP)
  bool qp_converged{false};
  double solve_time_us{0.0};
  int solve_levels{0};  // WQP: 1, HQP: 실제 level 수

  void Init(int nv, int n_actuated, int max_contact_vars);
};

}  // namespace rtc::tsid
