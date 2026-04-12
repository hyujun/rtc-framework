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
  int nq{0};           // generalized coordinate dim
  int nv{0};           // generalized velocity dim
  int n_actuated{0};   // actuated DoF count
  bool floating_base{false};

  Eigen::MatrixXd S;       // [n_actuated × nv] selection matrix
  Eigen::VectorXd tau_max; // [n_actuated]
  Eigen::VectorXd tau_min; // [n_actuated]
  Eigen::VectorXd q_upper; // [nq]
  Eigen::VectorXd q_lower; // [nq]
  Eigen::VectorXd v_max;   // [nv]

  // Pinocchio model + YAML config로부터 구성
  void build(const pinocchio::Model& model, const YAML::Node& config);
};

// ────────────────────────────────────────────────
// Contact 설정 (YAML로부터 로드, init 시 1회)
// ────────────────────────────────────────────────
struct ContactConfig {
  std::string name;
  std::string frame_name;
  int frame_id{0};       // Pinocchio frame ID (init 시 resolve)
  int contact_dim{3};    // 3 (point) or 6 (surface)
  double friction_coeff{0.7};
  int friction_faces{4}; // linearized cone 면 수
};

struct ContactManagerConfig {
  std::vector<ContactConfig> contacts;
  int max_contacts{0};
  int max_contact_vars{0};  // Σ contact_dim_i

  // YAML로부터 로드 + frame name → frame ID resolve
  void load(const YAML::Node& config, const pinocchio::Model& model);
};

// ────────────────────────────────────────────────
// 런타임 Contact 상태 (매 tick 또는 phase 전환 시 갱신)
// ────────────────────────────────────────────────
struct ContactState {
  struct Entry {
    int config_index{0};
    bool active{false};
    Eigen::Vector3d normal{0.0, 0.0, 1.0};  // world-frame 접촉 법선
  };

  std::vector<Entry> contacts;  // init 시 resize(max_contacts), 이후 크기 불변
  int active_count{0};
  int active_contact_vars{0};   // Σ active contact_dim

  // max_contacts 크기로 pre-allocate
  void init(int max_contacts);

  // active contact 개수 및 active_contact_vars 재계산
  void recompute_active(const ContactManagerConfig& manager);

  // RT-safe active contact 순회 (no alloc)
  template <typename Func>
  void for_each_active(const ContactManagerConfig& manager, Func&& fn) const {
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
    pinocchio::SE3 oMf;                     // world-frame placement
    Eigen::MatrixXd J;                      // [6 × nv] LOCAL_WORLD_ALIGNED
    Eigen::Matrix<double, 6, 1> dJv;        // J̇·v (classical acceleration)
  };
  std::vector<FrameCache> contact_frames;   // [max_contacts]

  // Task용 등록 frame 캐시
  struct RegisteredFrame {
    std::string name;
    pinocchio::FrameIndex frame_id{0};
    pinocchio::SE3 oMf;
    Eigen::MatrixXd J;
    Eigen::Matrix<double, 6, 1> dJv;
  };
  std::vector<RegisteredFrame> registered_frames;
  bool registration_locked{false};  // update() 호출 후 true → 추가 등록 금지

  // CoM (optional — CoMTask 등록 시 활성화)
  bool compute_com{false};
  Eigen::Vector3d com_position;
  Eigen::MatrixXd Jcom;  // [3 × nv]

  // Centroidal momentum (optional — MomentumTask 등록 시 활성화)
  bool compute_centroidal{false};
  Eigen::Matrix<double, 6, 1> h_centroidal;
  Eigen::MatrixXd Ag;  // [6 × nv]

  // 초기화: buffer pre-allocate (init 시 1회)
  void init(std::shared_ptr<const pinocchio::Model> model,
            const ContactManagerConfig& contact_cfg);

  // Task가 init 시 필요한 frame 등록 → update()에서 자동 계산
  // 반환: registered_frames 내 인덱스
  int register_frame(const std::string& name, pinocchio::FrameIndex frame_id);

  // 매 tick 갱신 (RT-safe: 사전 할당된 버퍼만 사용)
  void update(const Eigen::VectorXd& q_in,
              const Eigen::VectorXd& v_in,
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
  Eigen::VectorXd q_des;      // [nq]
  Eigen::VectorXd v_des;      // [nv]
  Eigen::VectorXd a_des;      // [nv] desired accel (feedforward)
  Eigen::VectorXd tau_ff;     // [n_actuated] feedforward torque
  Eigen::VectorXd lambda_des; // [max_contact_vars] desired contact forces

  void init(int nq, int nv, int n_actuated, int max_contact_vars);
};

struct CommandOutput {
  Eigen::VectorXd tau;         // [n_actuated]
  Eigen::VectorXd a_opt;       // [nv]
  Eigen::VectorXd lambda_opt;  // [active_contact_vars]
  bool qp_converged{false};
  double solve_time_us{0.0};
  int solve_levels{0};         // WQP: 1, HQP: 실제 level 수

  void init(int nv, int n_actuated, int max_contact_vars);
};

}  // namespace rtc::tsid
