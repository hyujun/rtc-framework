#ifndef INTEGRATED_BRINGUP_TEST_DUALARM_TEST_RIG_HPP_
#define INTEGRATED_BRINGUP_TEST_DUALARM_TEST_RIG_HPP_

// Rig for DemoDualArmController on the synthetic fixture
// rtc_urdf_bridge/test/urdf/dual_arm_trunk_hand.urdf: a trunk (3 joints) with
// two 7-joint arms as ONE device group and a 4-joint serial hand on the right
// arm. Shared by the behaviour suite and the allocation gate (which is its own
// binary and can share no fixture class).
//
// The servo is IDEAL unless a test says otherwise: the measurement of a tick
// is the command of the tick before. A test that has to tell "re-seeded from
// the measurement" from "kept its command" adds an offset to the measurement,
// because under the ideal servo the two are the same number.

#include "integrated_bringup/controllers/demo_dualarm_controller.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "test_urdf_path.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <yaml-cpp/yaml.h>

#include <array>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace integrated_bringup {

/// The friend the controller declares: what a test may reach past the public
/// surface. Read access, plus the one tick step the allocation gate has to
/// measure apart from the solve it shares a tick with.
struct DualArmTestAccess {
  static auto& Tasks(DemoDualArmController& c) { return c.tasks_; }

  static const auto& FrameTasks(const DemoDualArmController& c) { return c.frame_tasks_; }

  static const rtc::tsid::ClikReferenceGenerator& Clik(const DemoDualArmController& c) {
    return c.clik_;
  }

  static const ControllerTopicHandles& Topics(const DemoDualArmController& c) {
    return c.owned_topics_;
  }

  static const Eigen::VectorXd& PostureTarget(const DemoDualArmController& c) {
    return c.q_posture_des_;
  }

  static const CombinedModelCache& Cache(const DemoDualArmController& c) {
    return *c.combined_cache_;
  }

  static bool ApplyTaskGoal(DemoDualArmController& c, std::size_t k, const TaskGoal& goal) {
    return c.ApplyTaskGoal(k, goal);
  }

  static void AdvanceReference(DemoDualArmController& c, std::size_t k, double dt) {
    c.AdvanceReference(k, dt);
  }
};

}  // namespace integrated_bringup

namespace dualarm_rig {

namespace rub = rtc_urdf_bridge;
using integrated_bringup::DemoDualArmController;
using rtc::ControllerOutput;
using rtc::ControllerState;

inline constexpr double kDt = 0.002;
inline constexpr int kBodyDof = 17;
inline constexpr int kHandDof = 4;

inline const std::vector<std::string> kTrunkJoints = {"trunk_yaw_joint", "trunk_roll_joint",
                                                      "trunk_pitch_joint"};
inline const std::vector<std::string> kLeftJoints = {
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint",          "left_wrist_roll_joint",    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint"};
inline const std::vector<std::string> kRightJoints = {
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint",          "right_wrist_roll_joint",    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint"};
inline const std::vector<std::string> kHandJoints = {
    "finger_a_proximal_joint", "finger_a_distal_joint", "finger_b_proximal_joint",
    "finger_b_distal_joint"};

inline std::vector<std::string> BodyJoints() {
  std::vector<std::string> all = kTrunkJoints;
  all.insert(all.end(), kLeftJoints.begin(), kLeftJoints.end());
  all.insert(all.end(), kRightJoints.begin(), kRightJoints.end());
  return all;
}

// Every joint off zero and off its limit band, the two arms at different
// angles, both elbows bent: no task Jacobian is singular here, and reading one
// arm's joints as the other's changes every pose.
inline const std::vector<double> kBodyQ = {
    0.10,  -0.05, 0.08,                               // trunk
    0.30,  0.40,  -0.20, 0.90, 0.20,  -0.30, 0.10,    // left
    -0.20, -0.50, 0.30,  1.10, -0.20, 0.40,  -0.10};  // right
inline const std::vector<double> kHandQ = {0.20, 0.30, 0.10, 0.40};

inline const char* const kRoot = "pelvis";
inline const char* const kHandMount = "hand_base";
inline const char* const kRightFrame = "catch_frame";
inline const char* const kLeftFrame = "left_tool";
inline const char* const kLeftBase = "torso_link";
inline const std::vector<std::string> kTips = {"finger_a_tip", "finger_b_tip"};

// Device limits = the fixture URDF's (trunk, shoulder ×3, elbow, wrist ×3).
inline void BodyLimits(std::vector<double>& lower, std::vector<double>& upper,
                       std::vector<double>& torque) {
  const std::vector<double> arm_lo = {-2.6, -2.6, -2.6, -0.3, -1.8, -1.8, -1.8};
  const std::vector<double> arm_hi = {2.6, 2.6, 2.6, 2.6, 1.8, 1.8, 1.8};
  const std::vector<double> arm_tau = {25.0, 25.0, 25.0, 25.0, 5.0, 5.0, 5.0};
  lower = {-1.2, -1.2, -1.2};
  upper = {1.2, 1.2, 1.2};
  torque = {40.0, 40.0, 40.0};
  for (int arm = 0; arm < 2; ++arm) {
    lower.insert(lower.end(), arm_lo.begin(), arm_lo.end());
    upper.insert(upper.end(), arm_hi.begin(), arm_hi.end());
    torque.insert(torque.end(), arm_tau.begin(), arm_tau.end());
  }
}

/// What a test may vary in the controller YAML. The defaults are the shipped
/// profile's values, on the fixture's names.
struct Knobs {
  std::string right_frame = kRightFrame;
  std::string right_base = kRoot;
  std::string left_frame = kLeftFrame;
  std::string left_base = kLeftBase;
  bool left_first = false;  ///< list the left-hand task before the right-hand one
  std::string kind = "se3";
  std::string accel_constraint = "dynamic";
  double gain_linear = 10.0;
  double gain_angular = 5.0;
  double weight_right = 1.0;
  double weight_left = 1.0;
  double fb_lin_max = 0.25;
  double fb_ang_max = 1.0;
  double posture_weight_trunk = 1.0e-3;
  double posture_weight_arm = 1.0e-4;
  double posture_gain = 1.0;
  std::string extra_posture_joint;  ///< appended to the trunk group when set
  double eta_tau = 0.8;
  double limit_margin = 0.05;
  double joint_velocity_max = 3.14;
  bool brake = true;
  double linear_speed = 0.1;
  double angular_speed = 0.5;
  int max_qp_fail_ticks = 5;
  double track_err_max = 0.0;
  std::vector<std::string> target_frames = {"world", "pelvis", "torso_link"};
  bool with_hand = true;
  bool with_logs = false;
};

inline std::string JoinQuoted(const std::vector<std::string>& names) {
  std::string out;
  for (const auto& n : names) {
    out += (out.empty() ? "\"" : ", \"") + n + "\"";
  }
  return out;
}

inline std::string Yaml(const Knobs& k = {}) {
  std::ostringstream os;
  os.precision(17);
  const auto task = [&](const char* name, const std::string& frame, const std::string& base,
                        double weight) {
    os << "    - name: " << name << "\n      frame: \"" << frame << "\"\n      base_frame: \""
       << base << "\"\n      kind: " << k.kind << "\n      gain_linear: " << k.gain_linear
       << "\n      gain_angular: " << k.gain_angular << "\n      weight: " << weight
       << "\n      fb_lin_max: " << k.fb_lin_max << "\n      fb_ang_max: " << k.fb_ang_max << "\n";
  };
  std::vector<std::string> trunk = kTrunkJoints;
  if (!k.extra_posture_joint.empty()) {
    trunk.push_back(k.extra_posture_joint);
  }
  os << "command_type: \"position\"\nclik:\n  damping_sq: 1.0e-4\n  w_smooth: 1.0e-3\n"
     << "  qp: {max_iter: 20}\n  accel_constraint: " << k.accel_constraint
     << "\n  eta_tau: " << k.eta_tau << "\n  limit_margin: " << k.limit_margin
     << "\n  joint_velocity_max: " << k.joint_velocity_max
     << "\n  brake: {enabled: " << (k.brake ? "true" : "false") << ", margin: 0.9}\n"
     << "  target_frames: [" << JoinQuoted(k.target_frames) << "]\n  tasks:\n";
  if (k.left_first) {
    task("left_hand", k.left_frame, k.left_base, k.weight_left);
    task("right_hand", k.right_frame, k.right_base, k.weight_right);
  } else {
    task("right_hand", k.right_frame, k.right_base, k.weight_right);
    task("left_hand", k.left_frame, k.left_base, k.weight_left);
  }
  os << "  posture_groups:\n    - {name: trunk, joints: [" << JoinQuoted(trunk)
     << "], weight: " << k.posture_weight_trunk << ", gain: " << k.posture_gain << "}\n"
     << "    - {name: left_arm, joints: [" << JoinQuoted(kLeftJoints)
     << "], weight: " << k.posture_weight_arm << ", gain: " << k.posture_gain << "}\n"
     << "    - {name: right_arm, joints: [" << JoinQuoted(kRightJoints)
     << "], weight: " << k.posture_weight_arm << ", gain: " << k.posture_gain << "}\n"
     << "trajectory:\n  linear_speed: " << k.linear_speed
     << "\n  angular_speed: " << k.angular_speed
     << "\n  linear_speed_max: 0.5\n  angular_speed_max: 1.0\n  hand_speed: 3.14\n"
     << "  hand_speed_max: 6.28\nfault:\n  max_qp_fail_ticks: " << k.max_qp_fail_ticks
     << "\n  track_err_max: " << k.track_err_max << "\ntopics:\n  body:\n    subscribe:\n"
     << "      - {topic: \"body/joint_goal\", role: \"target\"}\n    publish:\n"
     << "      - {topic: \"transforms\", role: \"robot_transforms\"}\n";
  if (k.with_hand) {
    os << "  hand:\n    subscribe:\n      - {topic: \"hand/joint_goal\", role: \"target\"}\n";
  }
  if (k.with_logs) {
    os << "logs:\n  - {msg_type: rtc_msgs/DeviceStateLog, instance: body_state}\n"
       << "  - {msg_type: rtc_msgs/DeviceStateLog, instance: hand_state}\n"
       << "  - {msg_type: integrated_bringup/DualArmDiagLog, instance: dualarm_diag}\n";
  }
  return os.str();
}

inline const rub::ModelConfig& ModelConfig() {
  static const rub::ModelConfig cfg = [] {
    rub::ModelConfig c;
    c.urdf_path = rtc::test::TestUrdfPath("dual_arm_trunk_hand.urdf");
    c.root_joint_type = "fixed";
    c.tree_models.push_back({"body", kRoot, {kLeftFrame, kHandMount}});
    c.tree_models.push_back({"hand", kHandMount, kTips});
    return c;
  }();
  return cfg;
}

inline const std::shared_ptr<rub::PinocchioModelBuilder>& Builder() {
  static const auto builder = std::make_shared<rub::PinocchioModelBuilder>(ModelConfig());
  return builder;
}

inline std::map<std::string, rtc::DeviceNameConfig> Devices(bool with_hand = true) {
  std::map<std::string, rtc::DeviceNameConfig> devices;
  rtc::DeviceNameConfig body;
  body.device_name = "body";
  body.joint_state_names = BodyJoints();
  rtc::DeviceUrdfConfig urdf;
  urdf.root_link = kRoot;
  body.urdf = urdf;
  rtc::DeviceJointLimits limits;
  BodyLimits(limits.position_lower, limits.position_upper, limits.max_torque);
  limits.max_velocity.assign(static_cast<std::size_t>(kBodyDof), 20.0);
  body.joint_limits = limits;
  devices["body"] = body;
  if (with_hand) {
    rtc::DeviceNameConfig hand;
    hand.device_name = "hand";
    hand.joint_state_names = kHandJoints;
    rtc::DeviceUrdfConfig hand_urdf;
    hand_urdf.root_link = kHandMount;
    hand.urdf = hand_urdf;
    rtc::DeviceJointLimits hand_limits;
    hand_limits.position_lower.assign(static_cast<std::size_t>(kHandDof), 0.0);
    hand_limits.position_upper.assign(static_cast<std::size_t>(kHandDof), 1.5);
    hand_limits.max_velocity.assign(static_cast<std::size_t>(kHandDof), 10.0);
    hand_limits.max_torque.assign(static_cast<std::size_t>(kHandDof), 1.0);
    hand.joint_limits = hand_limits;
    devices["hand"] = hand;
  }
  return devices;
}

/// A controller with what the controller manager injects before any config is
/// read: the system model, the shared builder, the control rate.
inline std::unique_ptr<DemoDualArmController> Make() {
  auto ctrl = std::make_unique<DemoDualArmController>("");
  ctrl->SetSystemModelConfig(ModelConfig());
  ctrl->SetSharedModelBuilder(Builder());
  ctrl->SetControlRate(1.0 / kDt);
  return ctrl;
}

/// Config, then device configs — the controller manager's order, with no node.
inline std::unique_ptr<DemoDualArmController> BringUp(
    const Knobs& knobs = {},
    const std::map<std::string, rtc::DeviceNameConfig>* devices = nullptr) {
  auto ctrl = Make();
  ctrl->LoadConfig(YAML::Load(Yaml(knobs)));
  ctrl->SetDeviceNameConfigs(devices != nullptr ? *devices : Devices(knobs.with_hand));
  return ctrl;
}

/// One controller and the plant it talks to.
struct Harness {
  std::unique_ptr<DemoDualArmController> ctrl;
  std::vector<double> body_q = kBodyQ;  ///< what the servo is at (the next measurement)
  std::vector<double> hand_q = kHandQ;
  /// Added to the body measurement (NOT to the plant): what makes "the
  /// measurement" and "the last command" different numbers.
  std::vector<double> body_offset = std::vector<double>(static_cast<std::size_t>(kBodyDof), 0.0);
  bool body_valid = true;
  bool hand_valid = true;
  bool servo = true;  ///< false: the plant ignores the command (frozen)
  std::uint64_t iteration = 0;
  ControllerState state{};
  ControllerOutput out{};

  explicit Harness(const Knobs& knobs = {}) : ctrl(BringUp(knobs)) {}

  explicit Harness(std::unique_ptr<DemoDualArmController> c) : ctrl(std::move(c)) {}

  /// The state the next Compute() will see.
  void FillState() {
    state = ControllerState{};
    state.num_devices = 2;
    state.dt = kDt;
    state.iteration = iteration;
    state.t_relative_s = static_cast<double>(iteration) * kDt;
    auto& dev0 = state.devices[0];
    dev0.valid = body_valid;
    dev0.num_channels = kBodyDof;
    for (std::size_t i = 0; i < body_q.size(); ++i) {
      dev0.positions[i] = body_q[i] + body_offset[i];
    }
    auto& dev1 = state.devices[1];
    dev1.valid = hand_valid;
    dev1.num_channels = kHandDof;
    for (std::size_t i = 0; i < hand_q.size(); ++i) {
      dev1.positions[i] = hand_q[i];
    }
  }

  /// One tick. The plant then moves to the command (ideal servo) wherever the
  /// controller issued one.
  const ControllerOutput& Tick() {
    FillState();
    out = ctrl->Compute(state);
    AfterCompute();
    return out;
  }

  /// What follows a Compute(): the tick count, and the plant moving to the
  /// command. Separate so a test can wrap the Compute() call alone.
  void AfterCompute() {
    ++iteration;
    if (servo) {
      if (out.devices[0].num_channels >= kBodyDof) {
        for (std::size_t i = 0; i < body_q.size(); ++i) {
          body_q[i] = out.devices[0].commands[i];
        }
      }
      if (out.devices[1].num_channels >= kHandDof) {
        for (std::size_t i = 0; i < hand_q.size(); ++i) {
          hand_q[i] = out.devices[1].commands[i];
        }
      }
    }
  }

  void Run(int ticks) {
    for (int i = 0; i < ticks; ++i) {
      (void)Tick();
    }
  }

  [[nodiscard]] std::vector<double> Command() const {
    const auto& pod = ctrl->LastTick();
    return {pod.q_cmd.begin(), pod.q_cmd.begin() + kBodyDof};
  }
};

inline rtc_msgs::msg::RobotTarget TaskGoalMsg(const pinocchio::SE3& pose,
                                              const std::string& frame_id = "") {
  rtc_msgs::msg::RobotTarget msg;
  msg.goal_type = "task";
  msg.header.frame_id = frame_id;
  // ZYX: R = Rz(yaw)·Ry(pitch)·Rx(roll).
  const Eigen::Vector3d ypr = pose.rotation().eulerAngles(2, 1, 0);
  msg.task_target = {pose.translation().x(),
                     pose.translation().y(),
                     pose.translation().z(),
                     ypr[2],
                     ypr[1],
                     ypr[0]};
  return msg;
}

/// Full-model configuration from the device vectors, by joint NAME.
inline Eigen::VectorXd FullQ(const std::vector<double>& body_q, const std::vector<double>& hand_q) {
  const pinocchio::Model& model = *Builder()->GetFullModel();
  Eigen::VectorXd q = pinocchio::neutral(model);
  const auto body = BodyJoints();
  for (std::size_t i = 0; i < body.size(); ++i) {
    q[model.joints[model.getJointId(body[i])].idx_q()] = body_q[i];
  }
  for (std::size_t i = 0; i < kHandJoints.size(); ++i) {
    q[model.joints[model.getJointId(kHandJoints[i])].idx_q()] = hand_q[i];
  }
  return q;
}

/// The oracle: pose of `frame` in `base`, by forward kinematics on the FULL
/// model addressed by name. It shares no cache, no reorder map and no frame id
/// with the code under test.
inline pinocchio::SE3 FramePose(const std::vector<double>& body_q,
                                const std::vector<double>& hand_q, const std::string& base,
                                const std::string& frame) {
  const pinocchio::Model& model = *Builder()->GetFullModel();
  pinocchio::Data data(model);
  pinocchio::framesForwardKinematics(model, data, FullQ(body_q, hand_q));
  return data.oMf[model.getFrameId(base)].actInv(data.oMf[model.getFrameId(frame)]);
}

inline pinocchio::SE3 ToSe3(const std::array<double, 3>& p, const std::array<double, 4>& q) {
  return {Eigen::Quaterniond(q[0], q[1], q[2], q[3]).toRotationMatrix(),
          Eigen::Vector3d(p[0], p[1], p[2])};
}

inline pinocchio::SE3 ToSe3(const rtc::Pose& pose) {
  return ToSe3(pose.position, pose.quaternion);
}

inline double PositionGap(const pinocchio::SE3& a, const pinocchio::SE3& b) {
  return (a.translation() - b.translation()).norm();
}

inline double RotationGap(const pinocchio::SE3& a, const pinocchio::SE3& b) {
  return Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle();
}

}  // namespace dualarm_rig

#endif  // INTEGRATED_BRINGUP_TEST_DUALARM_TEST_RIG_HPP_
