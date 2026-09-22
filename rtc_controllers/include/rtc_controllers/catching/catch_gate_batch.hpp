// ── Offline batch judge for the S3.5b gate-catchable map ────────────────────
// dynamic_catching plan §11. `catch_pose_ik_batch` answers "is there a good
// catch posture"; this answers the rest of the planner's chain for a candidate
// that already has one — can the arm get there in time (L3 §4.3), is there a γ
// that both the hand and the arm can live with (§4.5), and where would the arm
// come to rest afterwards (§4.9). The verdicts are the runtime functions of
// `time_feasibility.hpp`, called here and nowhere re-derived, so the map
// describes the system that flies.
//
// What this does NOT judge, and the python side must not pretend it does:
//   - the γ rollout (§4.8) — no such function exists before S6.3;
//   - membership of p_stop in the catch workspace — `planner.workspace.catch_box`
//     is still TBD, so p_stop is reported and the caller applies its own bound;
//   - q̇ᵘ itself. The damped least-squares joint velocity behind v_dir,max has no
//     runtime producer yet (S6.2), so it arrives as an input column and the
//     equivalence claim (G3-I) covers the gate functions only.
//
// It knows no robot and no model: limits, timing and hand constants all come
// from the caller, and a candidate row carries its own postures.
#pragma once

#include "rtc_controllers/catching/time_feasibility.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <iosfwd>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace rtc::catching {

/// Run constants. Nothing here has a default: a gate fed a guessed constant
/// still produces a complete, plausible map.
struct GateSettings {
  std::vector<double> qdot_max;   ///< joint velocity limits [rad/s], model joint order
  std::vector<double> qddot_max;  ///< joint acceleration box [rad/s²] (plan §9, D-16)
  double eta_v{0.0};          ///< `planner.gamma.eta_v`; applied to `qdot_max` AND `v_max` (S4.4)
  double v_max{0.0};          ///< `reference.v_max` [m/s]
  double d_eff{0.0};          ///< `planner.hand.d_eff` [m]
  double t_close_total{0.0};  ///< T_close,e2e + h/2 [s] (L3 §4.5)
  double gamma_margin{0.0};   ///< `planner.gamma.margin` [m/s]
  double a_dec{0.0};          ///< `supervisor.decel.a_dec` [m/s²]
  double first_plan_s{0.0};   ///< when the first plan exists, from release: T_det + L [s]
  double t_arm_s{0.0};        ///< arm command delay T_arm [s]
  double t_margin_s{0.0};     ///< `planner.time.margin` [s]
};

/// Why `settings` cannot be used, or empty if it can. `nv` is the posture width
/// the candidates carry.
[[nodiscard]] std::string ValidateGateSettings(const GateSettings& settings, int nv);

/// One kinematically accepted candidate, with the posture that accepted it.
struct GateCandidate {
  std::int64_t id{0};
  int seed_id{0};                                      ///< which wait pose the arm starts from
  double t_c_s{0.0};                                   ///< catch instant, from release [s]
  Eigen::Vector3d p_c{Eigen::Vector3d::Zero()};        ///< model world [m]
  Eigen::Vector3d v_ball{Eigen::Vector3d::Zero()};     ///< model world [m/s]
  Eigen::VectorXd q_star;                              ///< catch posture [rad]
  Eigen::VectorXd qdot_u;                              ///< DLS unit-speed joint velocity
  Eigen::Vector3d jp_qdot_u{Eigen::Vector3d::Zero()};  ///< J_p q̇ᵘ, what q̇ᵘ achieves [m/s]
};

/// First gate, in planner order, that turned the candidate away.
enum class GateReason : std::uint8_t {
  kNone = 0,          ///< every gate judged here passed
  kReachInvalid,      ///< reach-time inputs or limits unusable (fail closed)
  kReachTime,         ///< the arm cannot be at q* by t_c (L3 §4.3)
  kGammaInvalid,      ///< v_dir,max or the window inputs unusable (fail closed)
  kGammaWindowEmpty,  ///< g_min > g_max, or the ball is inside the speed margin (§4.5)
  kStopInvalid,       ///< the stopping point could not be formed (§4.9)
};

[[nodiscard]] std::string_view GateReasonName(GateReason reason) noexcept;

/// Every gate's own result, not just the first failure: the map's histogram has
/// to be able to say how many candidates EACH gate would stop on its own.
struct GateRow {
  GateCandidate candidate{};
  double lead_s{0.0};  ///< t_c − now_lead, what the reach gate compares against [s]
  TMinResult reach{};  ///< max_i t_min,i from the wait pose at rest
  bool reach_ok{false};
  DirectionalSpeed direction{};
  double v_tcp_plan{0.0};  ///< η_v · v_max [m/s]
  GammaWindow window{};
  double max_catchable{0.0};  ///< [m/s]
  bool gamma_ok{false};
  StoppingReservation stop_gamma_min{};  ///< at g_min — the least the arm must retreat
  StoppingReservation stop_gamma_max{};  ///< at g_max — the most it may
  GateReason reason{GateReason::kReachInvalid};
};

/// Read candidates. Column ORDER IS TAKEN FROM THE HEADER. Required:
/// `id,t_c_s,p_c_{x,y,z},v_{x,y,z},jpu_{x,y,z}` plus `qs<i>` and `qu<i>` for
/// i = 0…nv−1 (q* and q̇ᵘ); `seed_id` is optional (0). Throws
/// `std::invalid_argument` naming the line on anything else, including a
/// non-finite cell — a broken generator must not land in the map as physics.
[[nodiscard]] std::vector<GateCandidate> ParseGateCandidateCsv(std::istream& in, int nv);

[[nodiscard]] std::string GateCsvHeader();

/// One result row, doubles at round-trip precision.
[[nodiscard]] std::string GateCsvRow(const GateRow& row);

/// Judge one candidate. Pure: no state is carried between calls, which is what
/// lets python shard and reorder the batch.
[[nodiscard]] GateRow JudgeGates(const GateCandidate& candidate, const Eigen::VectorXd& q_wait,
                                 const GateSettings& settings);

/// Judge every candidate, in the given order.
/// @throws std::invalid_argument on unusable settings, a missing seed, or a
///         posture whose width is not the seed's.
[[nodiscard]] std::vector<GateRow> RunGateBatch(const std::vector<GateCandidate>& candidates,
                                                const std::map<int, Eigen::VectorXd>& seeds,
                                                const GateSettings& settings);

}  // namespace rtc::catching
