#ifndef RTC_MPC_REFERENCE_STAGE_REFERENCE_HPP_
#define RTC_MPC_REFERENCE_STAGE_REFERENCE_HPP_

/// @file stage_reference.hpp
/// @brief What the locomotion OCP tracks at one node, and the whole horizon
///        of them: the output of @ref StageReferenceBuilder.
///
/// Conventions (those of @ref WbLayout · @ref WbcReference):
/// - `q`: free-flyer `[p_xyz, quaternion_xyzw]` then the joints; `v`: base
///   twist in the base **LOCAL** frame (linear, angular) then the joints.
/// - contacts are in the model handler's contact order (bit k of `mode` is
///   contact k — ARCH-1: no side semantics). `contact_pose[7k …]` is
///   `[p_xyz, quaternion_xyzw]` in world; `contact_twist[6k …]` and
///   `contact_acceleration[6k …]` are on world axes at the contact frame
///   origin (LOCAL_WORLD_ALIGNED), the acceleration classical (not spatial).
/// - `wrench`: the kinodynamic input's wrench block, flattened the way
///   `WbLayout` lays `u` out (contact k at `contact_u_offset[k]`, its own
///   dimension, world axes at the frame origin) — the same convention as
///   `WbcReference::wrench_ref`.
/// - `impact_proximity[k] ∈ [0, 1]`: 1 at and near a contact's touch-down /
///   lift-off, small mid-swing (the foot cost's weight ramp).
/// - `time` is absolute on the scheduler owner's clock; a consumer exporting
///   stages off the MPC thread writes `time − t0` (the ContactSchedule rule).
///
/// Both PODs are trivially copyable and carry no padding (equal fields ⇔
/// equal bytes), so a horizon can be compared or copied whole.

#include "rtc_mpc/model/wb_layout.hpp"
#include "rtc_mpc/reference/surface_frame.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rtc::mpc {

struct StageReference {
  double time{0.0};  ///< [s] absolute
  std::array<double, kMaxNq> q{};
  std::array<double, kMaxNv> v{};
  std::array<double, static_cast<std::size_t>(kMaxContacts) * 7> contact_pose{};
  std::array<double, static_cast<std::size_t>(kMaxContacts) * 6> contact_twist{};
  std::array<double, static_cast<std::size_t>(kMaxContacts) * 6> contact_acceleration{};
  std::array<double, kMaxContactVars> wrench{};
  std::array<double, kMaxContacts> impact_proximity{};
  std::uint32_t mode{0};      ///< contact bitmask at `time`
  std::uint32_t n_stance{0};  ///< bits set in `mode`
};

static_assert(std::is_trivially_copyable_v<StageReference>);
static_assert(sizeof(StageReference) ==
                  sizeof(double) * (1 + kMaxNq + kMaxNv + 7 * kMaxContacts + 6 * kMaxContacts +
                                    6 * kMaxContacts + kMaxContactVars + kMaxContacts) +
                      2 * sizeof(std::uint32_t),
              "StageReference must carry no padding: equal fields ⇔ equal bytes");

/// @brief One horizon: `n_stages` nodes at `t0 + k·dt`, `k = 0 … n_stages − 1`
///        (the last is the terminal node). Provenance names the inputs it was
///        built from.
struct HorizonReference {
  std::array<StageReference, kMaxHorizon + 1> stages{};
  SurfaceFrame terrain{};
  double t0{0.0};                         ///< [s] == the schedule's `t0`
  double dt{0.0};                         ///< [s]
  std::uint64_t reference_id{0};          ///< +1 per successful Build (0 = never built)
  std::uint64_t schedule_id{0};           ///< the ContactSchedule it sampled
  std::uint64_t schedule_epoch{0};        ///< its gait commit count
  std::uint64_t observation_sequence{0};  ///< the MPCStateSnapshot it started from
  std::uint64_t command_sequence{0};      ///< the FilteredCommand it integrated
  std::uint32_t n_stages{0};
  std::uint32_t n_contacts{0};
  std::uint32_t model_epoch{0};
  std::uint32_t current_gait{0};
  std::uint32_t target_gait{0};
  std::uint32_t nq{0};
  std::uint32_t nv{0};
  std::uint32_t n_contact_vars{0};
};

static_assert(std::is_trivially_copyable_v<HorizonReference>);
static_assert(sizeof(HorizonReference) == sizeof(StageReference) * (kMaxHorizon + 1) +
                                              sizeof(SurfaceFrame) + 2 * sizeof(double) +
                                              5 * sizeof(std::uint64_t) + 8 * sizeof(std::uint32_t),
              "HorizonReference must carry no padding");
static_assert(sizeof(HorizonReference) <= 256 * 1024, "two slots must stay small (heap, Init)");

}  // namespace rtc::mpc

#endif  // RTC_MPC_REFERENCE_STAGE_REFERENCE_HPP_
