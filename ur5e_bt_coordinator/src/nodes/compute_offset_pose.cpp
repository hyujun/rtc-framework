#include "ur5e_bt_coordinator/action_nodes/compute_offset_pose.hpp"
#include "ur5e_bt_coordinator/bt_logging.hpp"

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace rtc_bt {

namespace {
auto logger() { return ::rtc_bt::logging::ActionLogger("compute_offset_pose"); }

constexpr double kGimbalEpsilon = 1.0e-3;       // [rad] from ±π/2 (warn band)
constexpr double kQuatToRpyDegenerate = 1.0e-9; // singularity threshold on m(2,0)

/// ZYX-intrinsic (= XYZ-extrinsic = ROS standard RPY) → quaternion.
Eigen::Quaterniond RpyToQuat(double r, double p, double y)
{
  return Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ())
       * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
       * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
}

/// Quaternion → ZYX-intrinsic RPY. Manual extraction (Eigen's eulerAngles
/// has branch issues). Falls back to a degenerate but stable choice at
/// |pitch| = π/2.
void QuatToRpy(const Eigen::Quaterniond& q, double& r, double& p, double& y)
{
  const auto m = q.toRotationMatrix();
  const double sy = std::clamp(-m(2, 0), -1.0, 1.0);
  p = std::asin(sy);
  if (std::abs(m(2, 0)) < 1.0 - kQuatToRpyDegenerate) {
    r = std::atan2(m(2, 1), m(2, 2));
    y = std::atan2(m(1, 0), m(0, 0));
  } else {
    // Gimbal lock: yaw and roll are not separable; pin roll = 0.
    r = 0.0;
    y = std::atan2(-m(0, 1), m(1, 1));
  }
}
}  // namespace

BT::PortsList ComputeOffsetPose::providedPorts()
{
  return {
    BT::InputPort<Pose6D>("input_pose"),
    BT::InputPort<double>("offset_x", 0.0, "X offset [m]"),
    BT::InputPort<double>("offset_y", 0.0, "Y offset [m]"),
    BT::InputPort<double>("offset_z", 0.0, "Z offset [m]"),
    BT::InputPort<double>("offset_roll",  0.0, "Roll offset [rad]"),
    BT::InputPort<double>("offset_pitch", 0.0, "Pitch offset [rad]"),
    BT::InputPort<double>("offset_yaw",   0.0, "Yaw offset [rad]"),
    BT::InputPort<std::string>("rotation_mode", "add",
        "Rotation composition: 'add' | 'quat_body' | 'quat_world'"),
    BT::OutputPort<Pose6D>("output_pose"),
  };
}

BT::NodeStatus ComputeOffsetPose::tick()
{
  auto pose = getInput<Pose6D>("input_pose");
  if (!pose) {
    RCLCPP_ERROR(logger(), "missing input_pose port");
    throw BT::RuntimeError("ComputeOffsetPose: missing input_pose");
  }

  const double dx = getInput<double>("offset_x").value_or(0.0);
  const double dy = getInput<double>("offset_y").value_or(0.0);
  const double dz = getInput<double>("offset_z").value_or(0.0);
  const double dr = getInput<double>("offset_roll").value_or(0.0);
  const double dp = getInput<double>("offset_pitch").value_or(0.0);
  const double dy_rot = getInput<double>("offset_yaw").value_or(0.0);
  const auto mode = getInput<std::string>("rotation_mode").value_or("add");

  Pose6D result = pose.value();
  result.x += dx;
  result.y += dy;
  result.z += dz;

  if (mode == "add") {
    result.roll  += dr;
    result.pitch += dp;
    result.yaw   += dy_rot;
  } else if (mode == "quat_body" || mode == "quat_world") {
    const Eigen::Quaterniond q_cur = RpyToQuat(pose->roll, pose->pitch, pose->yaw);
    const Eigen::Quaterniond q_off = RpyToQuat(dr, dp, dy_rot);
    const Eigen::Quaterniond q_res =
        (mode == "quat_body") ? (q_cur * q_off).normalized()
                              : (q_off * q_cur).normalized();
    QuatToRpy(q_res, result.roll, result.pitch, result.yaw);
    if (std::abs(std::abs(result.pitch) - M_PI_2) < kGimbalEpsilon) {
      RCLCPP_WARN(logger(),
                  "near gimbal lock: pitch=%.4f rad "
                  "(roll/yaw decomposition is degenerate)",
                  result.pitch);
    }
  } else {
    RCLCPP_ERROR(logger(),
                 "unknown rotation_mode='%s' "
                 "(expected 'add' | 'quat_body' | 'quat_world')",
                 mode.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG(logger(),
               "mode=%s offset_xyz=[%.3f, %.3f, %.3f] "
               "offset_rpy=[%.3f, %.3f, %.3f] -> "
               "result_xyz=[%.3f, %.3f, %.3f] result_rpy=[%.3f, %.3f, %.3f]",
               mode.c_str(), dx, dy, dz, dr, dp, dy_rot,
               result.x, result.y, result.z,
               result.roll, result.pitch, result.yaw);

  setOutput("output_pose", result);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtc_bt
