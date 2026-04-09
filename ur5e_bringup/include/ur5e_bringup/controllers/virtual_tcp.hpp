// ── Virtual TCP computation shared between Demo{Joint,Task}Controller ────────
#pragma once

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/math/rpy.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

namespace ur5e_bringup {

/// Virtual TCP computation mode for fingertip-based control point.
enum class VirtualTcpMode : uint8_t {
  kDisabled,   ///< Use tool0 as control point (default)
  kCentroid,   ///< Fingertip position centroid
  kWeighted,   ///< Contact-force weighted fingertip centroid
  kConstant    ///< Fixed offset from TCP frame (YAML configured)
};

/// YAML-configurable virtual TCP parameters.
struct VirtualTcpConfig {
  VirtualTcpMode mode{VirtualTcpMode::kDisabled};
  std::array<double, 3> offset{{0.0, 0.0, 0.0}};       ///< Constant mode: [x,y,z] in TCP frame [m]
  std::array<double, 3> orientation{{0.0, 0.0, 0.0}};   ///< RPY [rad] orientation of vtcp in TCP frame
};

/// Per-fingertip input for virtual TCP computation.
struct FingertipVtcpInput {
  Eigen::Vector3d position_in_tcp{Eigen::Vector3d::Zero()};  ///< Position in TCP/hand frame
  double force_magnitude{0.0};   ///< Force magnitude [N] (for weighted mode)
  bool active{false};            ///< true if this fingertip has valid FK data
};

/// Result of virtual TCP computation.
struct VirtualTcpResult {
  pinocchio::SE3 T_tcp_vtcp{pinocchio::SE3::Identity()};   ///< TCP -> virtual TCP transform
  pinocchio::SE3 world_pose{pinocchio::SE3::Identity()};    ///< World-frame virtual TCP pose
  bool valid{false};
};

/// Compute the virtual TCP pose from fingertip data.
///
/// @param config    Virtual TCP configuration (mode, offset, orientation)
/// @param T_base_tcp  Current base -> TCP (tool0) transform
/// @param fingertips  Per-fingertip positions in TCP frame + force data
/// @return  VirtualTcpResult with valid=true on success
[[nodiscard]] inline VirtualTcpResult ComputeVirtualTcp(
    const VirtualTcpConfig& config,
    const pinocchio::SE3& T_base_tcp,
    std::span<const FingertipVtcpInput> fingertips) noexcept
{
  VirtualTcpResult result;
  if (config.mode == VirtualTcpMode::kDisabled) return result;

  // Pre-compute vtcp orientation from RPY config (applied to all modes)
  const Eigen::Matrix3d R_tcp_vtcp = pinocchio::rpy::rpyToMatrix(
      config.orientation[0], config.orientation[1], config.orientation[2]);

  switch (config.mode) {
    case VirtualTcpMode::kCentroid: {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      int count = 0;
      for (const auto& ft : fingertips) {
        if (ft.active) {
          sum += ft.position_in_tcp;
          ++count;
        }
      }
      if (count == 0) return result;
      result.T_tcp_vtcp.translation() = sum / static_cast<double>(count);
      result.T_tcp_vtcp.rotation() = R_tcp_vtcp;
      break;
    }
    case VirtualTcpMode::kWeighted: {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      double total_weight = 0.0;
      for (const auto& ft : fingertips) {
        if (!ft.active) continue;
        const double w = 1.0 + ft.force_magnitude;  // base weight ensures non-zero
        sum += w * ft.position_in_tcp;
        total_weight += w;
      }
      if (total_weight <= 0.0) return result;
      result.T_tcp_vtcp.translation() = sum / total_weight;
      result.T_tcp_vtcp.rotation() = R_tcp_vtcp;
      break;
    }
    case VirtualTcpMode::kConstant: {
      result.T_tcp_vtcp.translation() = Eigen::Vector3d(
          config.offset[0], config.offset[1], config.offset[2]);
      result.T_tcp_vtcp.rotation() = R_tcp_vtcp;
      break;
    }
    default: return result;
  }

  result.world_pose = T_base_tcp.act(result.T_tcp_vtcp);
  result.valid = true;
  return result;
}

}  // namespace ur5e_bringup
