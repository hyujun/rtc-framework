// LocomotionTruthLog — the external ground truth a locomotion_controller tick
// saw, in a file of its own (`locomotion_truth.csv`) so that what a robot can
// log (locomotion_diag.csv, the R tier) and what only a simulator can (this,
// the T tier) never share a file. Written by the same tick as the diag row,
// with the same `tick` column for the join.
//
// A row is pushed only on a tick where at least one truth lane is fresh
// (`truth_mask != 0`), and the channel opens its file on the first row
// (rtc::LogOpen::kOnFirstRow), so a run without truth lanes — a robot — has no
// such file at all rather than one of NaN or zero rows.
//
// What counts as truth (see locomotion-controller.md "Log tiers"):
//   - The body group's base lane (DeviceState::base_*), fresh by
//     rtc::IsBaseFresh against the state source's body_max_age. Copied RAW:
//     the quaternion is not normalised (the R tier's base_q is the state
//     source's normalised copy) and the twist is base-LOCAL, [v; ω].
//   - The body group's contact wrench lane (inference group c = contact c),
//     fresh by rtc::IsSensorGroupFresh against the same age. Its producers on
//     the shipped profiles are the simulator's sole sensors (the G1 ones
//     count only contacts with the world body, body2="world": a foot resting
//     on the other foot or leg reads zero); the robots have no foot force
//     sensor, which is why the lane is filed as truth. A robot that gains
//     one has to be classified again.
//
// Frames and signs of the contact block (truth_grf_<c>_*, truth_cop_<c>_*):
//   - Axes and origin are the lane's: rtc_mujoco_sim's sole sensor site,
//     moment about the site origin.
//   - The sign is FLIPPED from the lane's (which is the force the foot applies
//     to the floor): these columns are the wrench the floor applies to the
//     foot, the sign of the controller's contact forces — a standing foot has
//     fz > 0. (The diag's meas_contact_valid bit c is the backend's verdict on the same
//     group alone; this file's bit also asks for its age.)
//   - The moment columns are NaN unless the group's sensor_layout declares
//     has_native_moment (otherwise the lane's slots 4..6 are not a moment).
//   - CoP, in the site's x / y axes relative to its origin, from the floor-on-
//     foot wrench with the site plane z = 0: x = −τ_y / f_z, y = τ_x / f_z.
//     NaN without a moment or while f_z <= cop_min_fz (a fraction of the
//     robot's weight — the ratio is undefined near lift-off).
//
// The external wrench block (truth_ext_*), present only when the controller is
// given a topic for it (`external_wrench.topic`): the wrench a simulator
// reports it is applying to one body — a push. World axes, about the point the
// request named, the environment's wrench ON the robot [N, N·m], as received.
// Its bit (kExternalBit) is set while the last message is younger than
// `external_wrench.max_age_ms`; otherwise the six values are NaN. The age
// (−1: never received) is written either way. A column is "what was applied",
// so a tick where the simulator applies nothing reads 0 with the bit set, and
// a tick where nothing is known reads NaN with the bit clear.
#ifndef INTEGRATED_BRINGUP_LOGGING_LOCOMOTION_TRUTH_LOG_POD_HPP_
#define INTEGRATED_BRINGUP_LOGGING_LOCOMOTION_TRUTH_LOG_POD_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace integrated_bringup {

inline constexpr std::string_view kLocomotionTruthLogMsgType =
    "integrated_bringup/LocomotionTruthLog";
inline constexpr std::string_view kLocomotionTruthLogInstance = "locomotion_truth";

struct LocomotionTruthLogPod {
  static constexpr std::size_t kMaxContacts = 4;
  static constexpr std::uint32_t kBaseBit = 1U;  ///< contact c is bit 1 + c
  /// The external wrench lane: the bit above every contact's, whatever the
  /// contact count, so a reader needs no contact count to find it.
  static constexpr std::uint32_t kExternalBit = 1U << (1 + kMaxContacts);

  double t_relative_s{0.0};
  std::uint64_t tick{0};
  std::uint32_t truth_mask{0};  ///< fresh lanes this tick: kBaseBit, then one bit per contact

  // Base lane — NaN unless kBaseBit. The sequence and the age are written
  // either way (age −1: never received).
  std::uint64_t base_seq{0};
  double base_age_us{-1.0};
  std::array<double, 3> base_p{};       ///< world [m]
  std::array<double, 4> base_q_xyzw{};  ///< world ← base, as received (not normalised)
  std::array<double, 3> base_v{};       ///< base-LOCAL linear velocity [m/s]
  std::array<double, 3> base_w{};       ///< base-LOCAL angular velocity [rad/s]

  // Contact lane, per contact — NaN unless its bit. Sequence and age as above.
  std::array<double, kMaxContacts * 6> grf{};  ///< floor-on-foot [f; τ], site axes [N, N·m]
  std::array<double, kMaxContacts * 2> cop{};  ///< site x, y [m]
  std::array<double, kMaxContacts> contact_age_us{};
  std::array<std::uint64_t, kMaxContacts> contact_seq{};

  // External wrench lane — NaN unless kExternalBit. The age is written either way.
  std::array<double, 6> ext_wrench{};  ///< on the robot [f; τ], world axes [N, N·m]
  double ext_age_us{-1.0};
};

static_assert(std::is_trivially_copyable_v<LocomotionTruthLogPod>,
              "LocomotionTruthLogPod rides an SPSC ring and must be trivially copyable");

/// Column geometry, derived once from the configured contact names and captured
/// by both writers.
struct LocomotionTruthLogColumns {
  std::size_t contacts{0};
  bool external_wrench{false};  ///< the truth_ext_* block is written
};

[[nodiscard]] inline LocomotionTruthLogColumns LocomotionTruthLogColumnsFor(
    const std::vector<std::string>& contact_names, bool external_wrench = false) {
  return {std::min(contact_names.size(), LocomotionTruthLogPod::kMaxContacts), external_wrench};
}

/// Emit the CSV header. The logger appends '\n'.
inline void WriteLocomotionTruthLogHeader(std::ostream& os,
                                          const std::vector<std::string>& contact_names,
                                          const LocomotionTruthLogColumns& cols) {
  os << "t_relative_s,tick,truth_mask,truth_base_seq,truth_base_age_us,truth_"
        "base_px,"
        "truth_base_py,truth_base_pz,truth_base_qx,truth_base_qy,truth_base_qz,"
        "truth_base_qw,"
        "truth_base_vx,truth_base_vy,truth_base_vz,truth_base_wx,truth_base_wy,"
        "truth_base_wz";
  for (std::size_t c = 0; c < cols.contacts; ++c) {
    const std::string& n = contact_names[c];
    for (const char* axis : {"fx", "fy", "fz", "tx", "ty", "tz"}) {
      os << ",truth_grf_" << n << '_' << axis;
    }
    os << ",truth_cop_" << n << "_x,truth_cop_" << n << "_y,truth_age_" << n << "_us,truth_seq_"
       << n;
  }
  if (cols.external_wrench) {
    os << ",truth_ext_fx,truth_ext_fy,truth_ext_fz,truth_ext_tx,truth_ext_ty,truth_ext_tz,"
          "truth_ext_age_us";
  }
}

inline void WriteLocomotionTruthLogRow(std::ostream& os, const LocomotionTruthLogPod& p,
                                       const LocomotionTruthLogColumns& cols) {
  os << p.t_relative_s << ',' << p.tick << ',' << p.truth_mask << ',' << p.base_seq << ','
     << p.base_age_us;
  for (double x : p.base_p) {
    os << ',' << x;
  }
  for (double x : p.base_q_xyzw) {
    os << ',' << x;
  }
  for (double x : p.base_v) {
    os << ',' << x;
  }
  for (double x : p.base_w) {
    os << ',' << x;
  }
  for (std::size_t c = 0; c < cols.contacts; ++c) {
    for (std::size_t k = 0; k < 6; ++k) {
      os << ',' << p.grf[c * 6 + k];
    }
    os << ',' << p.cop[c * 2] << ',' << p.cop[c * 2 + 1] << ',' << p.contact_age_us[c] << ','
       << p.contact_seq[c];
  }
  if (cols.external_wrench) {
    for (double x : p.ext_wrench) {
      os << ',' << x;
    }
    os << ',' << p.ext_age_us;
  }
}

}  // namespace integrated_bringup

#endif  // INTEGRATED_BRINGUP_LOGGING_LOCOMOTION_TRUTH_LOG_POD_HPP_
