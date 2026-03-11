#include "ur5e_rt_controller/controllers/direct/joint_pd_controller.hpp"

#include <algorithm>
#include <cstddef>
#include <pinocchio/math/rpy.hpp>

namespace ur5e_rt_controller
{

// ── 생성자 ────────────────────────────────────────────────────────────────────

JointPDController::JointPDController(std::string_view urdf_path)
: JointPDController(urdf_path, Gains{}) {}

JointPDController::JointPDController(
  std::string_view urdf_path,
  Gains gains)
: data_(pinocchio::Model{}), gains_(gains)
{
  // URDF 파싱은 시작 시 1회만 수행; RT 경로에서 실행되지 않음
  pinocchio::urdf::buildModel(std::string(urdf_path), model_);
  data_ = pinocchio::Data(model_);

  // Eigen 작업 버퍼 사전 할당 — RT 경로에서 힙 할당 없음
  q_ = pinocchio::neutral(model_);
  v_ = Eigen::VectorXd::Zero(model_.nv);
  coriolis_forces_ = Eigen::VectorXd::Zero(model_.nv);
  jacobian_ = Eigen::MatrixXd::Zero(6, model_.nv);

  trajectory_.initialize({}, {}, 0.0);
}

// ── RTControllerInterface 구현 ────────────────────────────────────────────────

ControllerOutput JointPDController::Compute(
  const ControllerState & state) noexcept
{
  if (estopped_.load(std::memory_order_acquire)) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    return out;
  }

  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);

  // Pinocchio 상태 갱신 (중력, FK, 자코비안, 코리올리)
  UpdateDynamics(state.robot);

  // 새 목표 수신 시 궤적 생성 — acquire-load guarantees robot_target_ writes
  // from SetRobotTarget() (sensor thread) are visible.
  if (new_target_.load(std::memory_order_acquire)) {
    trajectory::JointSpaceTrajectory<kNumRobotJoints>::State start_state;
    trajectory::JointSpaceTrajectory<kNumRobotJoints>::State goal_state;

    double max_dist = 0.0;
    for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
      start_state.positions[i]     = state.robot.positions[i];
      start_state.velocities[i]    = state.robot.velocities[i];
      start_state.accelerations[i] = 0.0;

      goal_state.positions[i]     = robot_target_[i];
      goal_state.velocities[i]    = 0.0;
      goal_state.accelerations[i] = 0.0;

      max_dist = std::max(max_dist,
        std::abs(robot_target_[i] - state.robot.positions[i]));
    }

    const double duration = std::max(0.01, max_dist / gains_.trajectory_speed);
    trajectory_.initialize(start_state, goal_state, duration);
    trajectory_time_ = 0.0;
    new_target_.store(false, std::memory_order_relaxed);
  }

  const auto traj_state = trajectory_.compute(trajectory_time_);
  trajectory_time_ += dt;

  // PD 제어 + 피드포워드 속도 + 선택적 중력/코리올리 보상
  ControllerOutput output;

  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    const double e  = traj_state.positions[i] - state.robot.positions[i];
    const double de = (e - prev_error_[i]) / dt;

    output.robot_commands[i] = gains_.kp[i] * e + gains_.kd[i] * de;
    if (command_type_ != CommandType::kTorque) {
      output.robot_commands[i] += traj_state.velocities[i];
    }

    if (gains_.enable_gravity_compensation) {
      output.robot_commands[i] += gravity_torques_[i];
    }
    if (gains_.enable_coriolis_compensation) {
      output.robot_commands[i] +=
        coriolis_forces_[static_cast<Eigen::Index>(i)];
    }

    prev_error_[i] = e;
  }

  output.actual_target_positions = traj_state.positions;
  output.robot_commands = ClampCommands(output.robot_commands, command_type_);

  // TCP 포즈 출력 (task_positions: [x, y, z, roll, pitch, yaw])
  const auto last_joint =
    static_cast<pinocchio::JointIndex>(model_.njoints - 1);
  const pinocchio::SE3 & tcp = data_.oMi[last_joint];
  const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(tcp.rotation());
  output.actual_task_positions[0] = tcp.translation().x();
  output.actual_task_positions[1] = tcp.translation().y();
  output.actual_task_positions[2] = tcp.translation().z();
  output.actual_task_positions[3] = rpy[0];
  output.actual_task_positions[4] = rpy[1];
  output.actual_task_positions[5] = rpy[2];

  output.command_type = command_type_;
  return output;
}

void JointPDController::SetRobotTarget(
  std::span<const double, kNumRobotJoints> target) noexcept
{
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    robot_target_[i] = target[i];
  }
  // release-store: guarantees robot_target_ writes are visible to the RT thread
  // when it acquire-loads new_target_ in Compute().
  new_target_.store(true, std::memory_order_release);
}

void JointPDController::SetHandTarget(
  std::span<const double, kNumHandJoints> target) noexcept
{
  for (std::size_t i = 0; i < kNumHandJoints; ++i) {
    hand_target_[i] = target[i];
  }
}

// ── E-STOP ────────────────────────────────────────────────────────────────────

void JointPDController::TriggerEstop() noexcept
{
  estopped_.store(true, std::memory_order_release);
}

void JointPDController::ClearEstop() noexcept
{
  estopped_.store(false, std::memory_order_release);
  prev_error_ = {};   // 미분 항 리셋
  new_target_.store(true, std::memory_order_relaxed); // 현재 위치부터 궤적 재생성
}

bool JointPDController::IsEstopped() const noexcept
{
  return estopped_.load(std::memory_order_acquire);
}

void JointPDController::SetHandEstop(bool active) noexcept
{
  hand_estopped_.store(active, std::memory_order_release);
}

// ── 진단용 접근자 ─────────────────────────────────────────────────────────────

std::array<double, kNumRobotJoints>
JointPDController::gravity_torques() const noexcept
{
  return gravity_torques_;
}

std::array<double, 3> JointPDController::tcp_position() const noexcept
{
  return tcp_position_;
}

// ── 컨트롤러 레지스트리 훅 ────────────────────────────────────────────────────

void JointPDController::LoadConfig(const YAML::Node & cfg)
{
  if (!cfg) {return;}

  if (cfg["kp"] && cfg["kp"].IsSequence() && cfg["kp"].size() == 6) {
    for (std::size_t i = 0; i < 6; ++i) {
      gains_.kp[i] = cfg["kp"][i].as<double>();
    }
  }
  if (cfg["kd"] && cfg["kd"].IsSequence() && cfg["kd"].size() == 6) {
    for (std::size_t i = 0; i < 6; ++i) {
      gains_.kd[i] = cfg["kd"][i].as<double>();
    }
  }
  if (cfg["enable_gravity_compensation"]) {
    gains_.enable_gravity_compensation =
      cfg["enable_gravity_compensation"].as<bool>();
  }
  if (cfg["enable_coriolis_compensation"]) {
    gains_.enable_coriolis_compensation =
      cfg["enable_coriolis_compensation"].as<bool>();
  }
  if (cfg["trajectory_speed"]) {
    gains_.trajectory_speed = cfg["trajectory_speed"].as<double>();
  }
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }
}

void JointPDController::UpdateGainsFromMsg(
  std::span<const double> gains) noexcept
{
  // 레이아웃: [kp×6, kd×6, enable_gravity(0/1), enable_coriolis(0/1), trajectory_speed]
  if (gains.size() < 14) {return;}

  for (std::size_t i = 0; i < 6; ++i) {gains_.kp[i] = gains[i];}
  for (std::size_t i = 0; i < 6; ++i) {gains_.kd[i] = gains[6 + i];}
  gains_.enable_gravity_compensation  = gains[12] > 0.5;
  gains_.enable_coriolis_compensation = gains[13] > 0.5;
  if (gains.size() >= 15) {gains_.trajectory_speed = gains[14];}
}

std::vector<double> JointPDController::GetCurrentGains() const noexcept
{
  // layout: [kp×6, kd×6, enable_gravity(0/1), enable_coriolis(0/1), trajectory_speed]
  std::vector<double> v;
  v.reserve(15);
  v.insert(v.end(), gains_.kp.begin(), gains_.kp.end());
  v.insert(v.end(), gains_.kd.begin(), gains_.kd.end());
  v.push_back(gains_.enable_gravity_compensation ? 1.0 : 0.0);
  v.push_back(gains_.enable_coriolis_compensation ? 1.0 : 0.0);
  v.push_back(gains_.trajectory_speed);
  return v;
}

// ── 내부 헬퍼 ─────────────────────────────────────────────────────────────────

ControllerOutput JointPDController::ComputeEstop(
  const ControllerState & state) noexcept
{
  // E-STOP: kSafePosition으로 PD 제어 (궤적 없이 직접 구동)
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);

  ControllerOutput output;
  for (std::size_t i = 0; i < kNumRobotJoints; ++i) {
    const double e  = kSafePosition[i] - state.robot.positions[i];
    const double de = (e - prev_error_[i]) / dt;
    output.robot_commands[i] = gains_.kp[i] * e + gains_.kd[i] * de;
    prev_error_[i] = e;
  }

  output.actual_target_positions = kSafePosition;
  output.robot_commands = ClampCommands(output.robot_commands, command_type_);
  new_target_.store(true, std::memory_order_relaxed);  // E-STOP 해제 후 궤적 재생성
  return output;
}

void JointPDController::UpdateDynamics(const RobotState & robot) noexcept
{
  const std::size_t nv = static_cast<std::size_t>(model_.nv);
  const std::size_t n  = std::min(static_cast<std::size_t>(kNumRobotJoints), nv);

  for (std::size_t i = 0; i < n; ++i) {
    q_[static_cast<Eigen::Index>(i)] = robot.positions[i];
    v_[static_cast<Eigen::Index>(i)] = robot.velocities[i];
  }

  // ── 중력 토크 g(q) ─────────────────────────────────────────────────────
  if (gains_.enable_gravity_compensation) {
    const Eigen::VectorXd & g =
      pinocchio::computeGeneralizedGravity(model_, data_, q_);
    for (std::size_t i = 0; i < n; ++i) {
      gravity_torques_[i] = g[static_cast<Eigen::Index>(i)];
    }
  }

  // ── 순기구학 (FK) ──────────────────────────────────────────────────────
  pinocchio::forwardKinematics(model_, data_, q_, v_);

  // ── TCP 위치 캐시 ──────────────────────────────────────────────────────
  const auto last_joint =
    static_cast<pinocchio::JointIndex>(model_.njoints - 1);
  const pinocchio::SE3 & tcp = data_.oMi[last_joint];
  tcp_position_[0] = tcp.translation()[0];
  tcp_position_[1] = tcp.translation()[1];
  tcp_position_[2] = tcp.translation()[2];

  // ── 자코비안 ──────────────────────────────────────────────────────────
  pinocchio::computeJointJacobian(model_, data_, q_, last_joint, jacobian_);

  // ── 코리올리/원심력 C(q,v)·v ───────────────────────────────────────────
  if (gains_.enable_coriolis_compensation) {
    const Eigen::MatrixXd & C =
      pinocchio::computeCoriolisMatrix(model_, data_, q_, v_);
    coriolis_forces_.noalias() = C * v_;
  }
}

std::array<double, kNumRobotJoints> JointPDController::ClampCommands(
  std::array<double, kNumRobotJoints> cmds, CommandType type) noexcept
{
  const double limit = (type == CommandType::kTorque)
                            ? kMaxJointTorque : kMaxJointVelocity;
  for (auto & c : cmds) {
    c = std::clamp(c, -limit, limit);
  }
  return cmds;
}

}  // namespace ur5e_rt_controller
