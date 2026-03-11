#pragma once

#include "ur5e_rt_controller/rt_controller_interface.hpp"
#include "ur5e_rt_controller/trajectory/joint_space_trajectory.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <span>
#include <string_view>

namespace ur5e_rt_controller
{

/// 관절 공간 PD 컨트롤러 (중력/코리올리 보상 선택적 적용)
///
/// 제어 법칙:
/// @code
///   τ[i] = ff_vel[i] + Kp[i]*e[i] + Kd[i]*ė[i]  [+ g(q)[i]]  [+ C(q,v)·v [i]]
/// @endcode
///
///   - e[i]       : 위치 오차 (목표 − 현재)
///   - ė[i]       : 오차 미분 (Δe / dt)
///   - g(q)       : Pinocchio RNEA로 계산한 중력 토크 벡터 (enable_gravity_compensation=true 시)
///   - C(q,v)·v   : 코리올리/원심력 (enable_coriolis_compensation=true 시)
///
/// 기본값(enable_gravity_compensation=false)에서는 순수 PD 컨트롤러로 동작.
/// Pinocchio 모델은 생성자에서 1회 로드; RT 경로에서 힙 할당 없음.
///
/// UpdateGainsFromMsg 레이아웃: [kp×6, kd×6, gravity(0/1), coriolis(0/1), trajectory_speed]
class JointPDController final : public RTControllerInterface
{
public:
  struct Gains
  {
    std::array<double, 6> kp{{200.0, 200.0, 150.0, 120.0, 120.0, 120.0}};
    std::array<double, 6> kd{{30.0, 30.0, 25.0, 20.0, 20.0, 20.0}};
    bool   enable_gravity_compensation{false};  ///< g(q) 보상 활성화
    bool   enable_coriolis_compensation{false}; ///< C(q,v)·v 보상 활성화
    double trajectory_speed{1.0};              ///< 궤적 최대 속도 (rad/s)
  };

  /// URDF 파일로 모델을 로드하고 컨트롤러를 초기화한다.
  /// @param urdf_path  UR5e URDF 파일의 절대 경로
  /// @param gains      PD 게인 및 보상 플래그 (기본값 사용 가능)
  /// @throws std::runtime_error  URDF 파싱 실패 시
  explicit JointPDController(std::string_view urdf_path);
  JointPDController(std::string_view urdf_path, Gains gains);

  // ── RTControllerInterface ──────────────────────────────────────────────────
  [[nodiscard]] ControllerOutput Compute(
    const ControllerState & state) noexcept override;

  void SetRobotTarget(
    std::span<const double, kNumRobotJoints> target) noexcept override;

  void SetHandTarget(
    std::span<const double, kNumHandJoints> target) noexcept override;

  [[nodiscard]] std::string_view Name() const noexcept override
  {
    return "JointPDController";
  }

  // ── E-STOP ─────────────────────────────────────────────────────────────────
  void TriggerEstop() noexcept override;
  void ClearEstop()   noexcept override;
  [[nodiscard]] bool IsEstopped() const noexcept override;
  void SetHandEstop(bool active) noexcept override;

  // ── 컨트롤러 레지스트리 훅 ────────────────────────────────────────────────
  void LoadConfig(const YAML::Node & cfg) override;
  void UpdateGainsFromMsg(std::span<const double> gains) noexcept override;
  [[nodiscard]] std::vector<double> GetCurrentGains() const noexcept override;
  [[nodiscard]] CommandType GetCommandType() const noexcept override
  {
    return command_type_;
  }

  // ── 게인 접근자 ────────────────────────────────────────────────────────────
  void  set_gains(const Gains & g) noexcept {gains_ = g;}
  [[nodiscard]] Gains get_gains() const noexcept {return gains_;}

  // ── 진단용 접근자 (RT 경로 외부에서만 사용) ────────────────────────────────
  [[nodiscard]] std::array<double, kNumRobotJoints> gravity_torques() const noexcept;
  [[nodiscard]] std::array<double, 3>               tcp_position()    const noexcept;
  [[nodiscard]] Eigen::MatrixXd                     jacobian()        const noexcept
  {
    return jacobian_;
  }

private:
  static constexpr std::array<double, kNumRobotJoints> kSafePosition{
    0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  static constexpr double kMaxJointVelocity{2.0};  // rad/s

  // ── Pinocchio 모델 및 작업 버퍼 (생성자에서 1회 할당) ─────────────────────
  pinocchio::Model model_;
  pinocchio::Data  data_;

  Eigen::VectorXd q_;               ///< 관절 위치 벡터 (nq)
  Eigen::VectorXd v_;               ///< 관절 속도 벡터 (nv)
  Eigen::VectorXd coriolis_forces_; ///< C(q,v)·v 결과 버퍼 (nv)
  Eigen::MatrixXd jacobian_;        ///< 6×nv 자코비안 버퍼

  // ── 컨트롤러 상태 ──────────────────────────────────────────────────────────
  Gains  gains_;
  std::array<double, kNumRobotJoints> robot_target_{};
  std::array<double, kNumHandJoints>  hand_target_{};
  std::array<double, kNumRobotJoints> prev_error_{};

  std::atomic<bool> new_target_{false};
  trajectory::JointSpaceTrajectory<kNumRobotJoints> trajectory_;
  double trajectory_time_{0.0};

  // ── 캐시된 진단 출력 ───────────────────────────────────────────────────────
  std::array<double, kNumRobotJoints> gravity_torques_{};
  std::array<double, 3>               tcp_position_{};

  // ── E-STOP ─────────────────────────────────────────────────────────────────
  std::atomic<bool> estopped_{false};
  std::atomic<bool> hand_estopped_{false};

  CommandType command_type_{CommandType::kTorque};

  // ── 내부 헬퍼 ─────────────────────────────────────────────────────────────
  /// E-STOP 활성 시 kSafePosition으로 PD 제어 (derivative 항 포함)
  [[nodiscard]] ControllerOutput ComputeEstop(
    const ControllerState & state) noexcept;

  /// Pinocchio 알고리즘 실행 (중력, FK, 자코비안, 코리올리 — 힙 할당 없음)
  void UpdateDynamics(const RobotState & robot) noexcept;

  [[nodiscard]] static std::array<double, kNumRobotJoints> ClampCommands(
    std::array<double, kNumRobotJoints> cmds) noexcept;
};

}  // namespace ur5e_rt_controller
