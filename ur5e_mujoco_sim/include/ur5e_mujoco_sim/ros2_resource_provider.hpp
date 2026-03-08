// ── ros2_resource_provider.hpp ────────────────────────────────────────────────
// Custom MuJoCo resource provider to natively support "package://" URIs.
// ──────────────────────────────────────────────────────────────────────────────
#ifndef UR5E_MUJOCO_SIM_ROS2_RESOURCE_PROVIDER_HPP_
#define UR5E_MUJOCO_SIM_ROS2_RESOURCE_PROVIDER_HPP_

namespace ur5e_rt_controller {

/// @brief Registers a custom MuJoCo resource provider for ROS 2 "package://" paths.
/// @details This function registers a global mjpResourceProvider with MuJoCo
///          (introduced in MuJoCo 3.x). This allows mj_loadXML to natively
///          understand "<mesh file=\"package://...\"/>" and paths passed into it.
void RegisterRos2ResourceProvider();

}  // namespace ur5e_rt_controller

#endif  // UR5E_MUJOCO_SIM_ROS2_RESOURCE_PROVIDER_HPP_
