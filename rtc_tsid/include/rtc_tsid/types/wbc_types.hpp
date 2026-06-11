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
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

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
struct PinocchioCache {
  // Pinocchio model/data (model은 공유, data는 소유)
  std::shared_ptr<const pinocchio::Model> model_ptr;
  pinocchio::Data data;

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
    pinocchio::SE3 oMf;               // world-frame placement
    Eigen::MatrixXd J;                // [6 × nv] LOCAL_WORLD_ALIGNED
    Eigen::Matrix<double, 6, 1> dJv;  // J̇·v (classical acceleration)
  };

  std::vector<FrameCache> contact_frames;  // [max_contacts]

  // Task용 등록 frame 캐시
  struct RegisteredFrame {
    std::string name;
    pinocchio::FrameIndex frame_id{0};
    pinocchio::SE3 oMf;
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

  // 초기화: buffer pre-allocate (init 시 1회)
  void Init(std::shared_ptr<const pinocchio::Model> model, const ContactManagerConfig& contact_cfg);

  // Task가 init 시 필요한 frame 등록 → Update()에서 자동 계산
  // 반환: registered_frames 내 인덱스
  int RegisterFrame(const std::string& name, pinocchio::FrameIndex frame_id);

  // 매 tick 갱신 (RT-safe: 사전 할당된 버퍼만 사용)
  void Update(const Eigen::VectorXd& q_in, const Eigen::VectorXd& v_in,
              const ContactState& contacts) noexcept;
};

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
