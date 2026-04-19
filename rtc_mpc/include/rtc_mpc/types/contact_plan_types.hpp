#ifndef RTC_MPC_TYPES_CONTACT_PLAN_TYPES_HPP_
#define RTC_MPC_TYPES_CONTACT_PLAN_TYPES_HPP_

/// @file contact_plan_types.hpp
/// @brief Contact-plan description types for OCP construction (non-RT).
///
/// These types live on the **OCP build / reconfigure path** (phase change),
/// NOT the 500Hz RT loop. `std::vector` / `std::string` are therefore allowed
/// — RT-path interchange uses the fixed-capacity structs in
/// @ref rtc_mpc/types/mpc_solution_types.hpp instead.
///
/// Robot-agnostic: frame ids are resolved against `pinocchio::Model` at
/// runtime via @ref rtc_mpc/model/robot_model_handler.hpp. No robot names or
/// hardcoded frame strings appear in rtc_mpc.

#include <string>
#include <vector>

namespace rtc::mpc {

/// @brief Static description of a single contact frame.
///
/// Populated once per robot from YAML + `pinocchio::Model::getFrameId(name)`;
/// then consumed by OCP handlers to place contact-force variables / friction
/// cone constraints.
struct ContactFrameInfo {
  int frame_id{-1};   ///< Pinocchio frame index (`pinocchio::FrameIndex`)
  int dim{3};         ///< force/torque dim: 3 (point) or 6 (wrench)
  std::string name{}; ///< original URDF frame name (diagnostics only)
};

/// @brief Set of contact frames active during a time segment of the OCP.
///
/// `active_frame_ids` holds Pinocchio frame indices (not array positions in
/// `ContactPlan::frames`) so consumers can dispatch directly to Pinocchio
/// algorithms without an extra lookup.
struct ContactPhase {
  std::vector<int> active_frame_ids{}; ///< Pinocchio frame ids active here
  double t_start{0.0};                 ///< phase start time [s]
  double t_end{0.0};                   ///< phase end time   [s]
};

/// @brief Full contact plan across the MPC horizon.
///
/// `frames` enumerates every possible contact frame (union across phases);
/// `phases` lists which are active on each segment. Empty `phases` means
/// "no contacts anywhere on the horizon" (free-flight / approach).
struct ContactPlan {
  std::vector<ContactFrameInfo> frames{};
  std::vector<ContactPhase> phases{};
};

} // namespace rtc::mpc

#endif // RTC_MPC_TYPES_CONTACT_PLAN_TYPES_HPP_
