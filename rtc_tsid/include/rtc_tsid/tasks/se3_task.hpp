#pragma once

#include "rtc_tsid/core/task_base.hpp"

#include <array>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>
#pragma GCC diagnostic pop

namespace rtc::tsid {

// ────────────────────────────────────────────────
// SE3 pose tracking task (6D, mask 지원)
//
// cost = w * ‖J_frame(masked) · a - r_des‖²
//
// r_des = a_ff + Kp·e_pos + Kd·e_vel - dJ·v  (masked)
//
// Frame contract:
//   - tip_frame   (YAML "frame")        : 제어 대상 frame
//   - base_frame  (YAML "base_frame")   : reference 좌표계 (필수, F-4 strict).
//   placement_des는 base_frame 기준으로 해석되며, 내부에서
//   bMf = oMb⁻¹·oMf 로 변환 후 비교한다. base_frame이 universe(frame_id 0)이면
//   기존 world-기준 동작과 동일 (fast-path). YAML에서 키가 누락되면 init은
//   std::runtime_error를 throw 한다.
//
// 위치 오차: e_pos[0:3] = p_des - p_curr_b
// 자세 오차: e_pos[3:6] = log3(R_currᵀ · R_des) — base frame에서
// 속도 오차: e_vel = v_des - J_frame · v_curr  (base frame이 fixed인 경우만 유효)
//
// mask: 6D 중 제어할 축 선택 [vx,vy,vz,wx,wy,wz]
// residual_dim = mask에서 활성 축 수
// ────────────────────────────────────────────────
class SE3Task final : public TaskBase {
 public:
  [[nodiscard]] std::string_view name() const noexcept override { return name_; }

  void init(const pinocchio::Model& model, const RobotModelInfo& robot_info, PinocchioCache& cache,
            const YAML::Node& task_config) override;

  [[nodiscard]] int residual_dim() const noexcept override { return active_dim_; }

  void compute_residual(const PinocchioCache& cache, const ControlReference& ref,
                        const ContactState& contacts, int n_vars,
                        Eigen::Ref<Eigen::MatrixXd> J_block,
                        Eigen::Ref<Eigen::VectorXd> r_block) noexcept override;

  /// @brief SE3 목표 설정 (RT-safe)
  /// @param placement_des 목표 SE3 pose, **base_frame 기준**.
  ///        base_frame이 명시적 universe(frame_id 0)이면 world 기준.
  /// @param v_des 목표 spatial velocity [6] (default: zero)
  /// @param a_ff feedforward spatial acceleration [6] (default: zero)
  void set_se3_reference(
      const pinocchio::SE3& placement_des,
      const Eigen::Matrix<double, 6, 1>& v_des = Eigen::Matrix<double, 6, 1>::Zero(),
      const Eigen::Matrix<double, 6, 1>& a_ff = Eigen::Matrix<double, 6, 1>::Zero()) noexcept;

  /// @brief PD gains 설정 (RT-safe)
  void set_gains(const Eigen::Matrix<double, 6, 1>& kp,
                 const Eigen::Matrix<double, 6, 1>& kd) noexcept;

 private:
  std::string name_{"se3"};
  int nv_{0};
  int registered_frame_idx_{-1};  // PinocchioCache registered_frames 인덱스 (tip)
  // Base frame: reference 좌표계. base_is_universe_=true면 -1로 두고 fast-path를
  // 탄다. 명시적 base_frame이 universe(frame_id 0)일 때도 동일.
  int base_frame_idx_{-1};
  bool base_is_universe_{true};  // base_frame == universe → fast path

  // 6D mask: true = 해당 축 제어
  std::array<bool, 6> mask_{};
  int active_dim_{0};

  // PD gains (축별)
  Eigen::Matrix<double, 6, 1> kp_;
  Eigen::Matrix<double, 6, 1> kd_;

  // Reference
  pinocchio::SE3 placement_des_{pinocchio::SE3::Identity()};
  Eigen::Matrix<double, 6, 1> v_des_;
  Eigen::Matrix<double, 6, 1> a_ff_;

  // Pre-allocated workspace
  Eigen::Matrix<double, 6, 1> error_full_;    // 6D pose error
  Eigen::Matrix<double, 6, 1> v_error_full_;  // 6D velocity error
  Eigen::Matrix<double, 6, 1> a_des_full_;    // 6D desired acceleration
};

}  // namespace rtc::tsid
