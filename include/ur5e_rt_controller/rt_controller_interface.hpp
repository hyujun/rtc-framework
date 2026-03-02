#pragma once
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace ur5e_rt_controller {

// UR5e 로봇 상태
struct RobotState {
  Eigen::VectorXd q{6};        // 관절 위치
  Eigen::VectorXd qd{6};       // 관절 속도
  Eigen::Vector3d tcp_pos{3};  // TCP 위치
  double dt = 0.002;
  uint64_t iter = 0;
};

// 커스텀 핸드 상태
struct HandState {
  Eigen::VectorXd motor_pos{11};      // 11개 모터 위치
  Eigen::VectorXd motor_vel{11};      // 11개 모터 속도
  Eigen::VectorXd motor_current{11};  // 11개 모터 전류
  Eigen::VectorXd sensor_data{44};    // 4개 센서 × 11개 = 44개 데이터
  bool valid = false;
};

// 통합 제어 상태
struct ControllerState {
  RobotState robot;
  HandState hand;
  double dt = 0.002;
  uint64_t iter = 0;
};

// 제어 출력
struct ControllerOutput {
  Eigen::VectorXd robot_cmd{6};   // UR5e 명령
  Eigen::VectorXd hand_cmd{11};   // Hand 모터 위치 명령
  bool valid = true;
};

// 제어기 인터페이스
class RTControllerInterface {
public:
  virtual ~RTControllerInterface() = default;
  virtual ControllerOutput compute(const ControllerState& state) noexcept = 0;
  virtual void setRobotTarget(const Eigen::VectorXd& target) noexcept { robot_target_ = target; }
  virtual void setHandTarget(const Eigen::VectorXd& target) noexcept { hand_target_ = target; }
  virtual std::string name() const noexcept = 0;

protected:
  Eigen::VectorXd robot_target_{6};
  Eigen::VectorXd hand_target_{11};
};

}  // namespace ur5e_rt_controller
