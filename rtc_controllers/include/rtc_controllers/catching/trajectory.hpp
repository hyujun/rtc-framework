// ── Shared snapshot types of the catching core (dynamic_catching S1.2) ───────
// The POD payloads that cross threads through rtc::SeqLock:
//
//   TrajectorySnapshot — vision prediction, written by the nrt ingress
//                        (L1), read every RT tick (L2) and by the planner (L3)
//   PlanSnapshot       — planner (L3) → RT tick (L4/L7)
//
// SeqLock requires trivially copyable payloads, and Eigen::Vector3d is not one,
// so vectors are std::array<double, 3> and the computing side views them with
// Eigen::Map (L0 §5.2, plan §6). Every instant is an absolute steady-ns
// BallTime (plan §3); relative seconds exist only inside the numeric core.
//
// This header is the single owner of the trajectory capacity kCap (L0 §5.2). It
// is shared by L1/L2/L3 so that none of them depends on another for the type.
#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

namespace rtc::catching {

/// Compile-time capacity of one predicted trajectory [points].
///
/// PROVISIONAL (plan D-15 / S0.7, user decision 2026-09-19): 40 holds ≈1.95 s at
/// the 0.05 s sim-profile spacing, twice the largest point count the S0.7 sweep
/// needed. The run-time bound n_max ≤ kCap comes from S3.6; if S3.6 needs more,
/// raise this and re-run the S1 gates (plan §4.2 backfill). The snapshot is
/// copied whole every RT tick (D-21 forbids gating the copy on
/// SeqLock::sequence()), so this constant IS the per-tick copy cost.
inline constexpr int kCap = 40;

/// Capacity of a joint vector carried in PlanSnapshot (IK solution over the
/// planner's control model). Checked against the model nv at configure time.
/// Separate from kCap, which counts trajectory points, not joints.
inline constexpr int kMaxPlanNv = 32;

/// One predicted ball sample. The ball instant is absolute steady ns (BallTime
/// axis, converted once on receipt by ConvertRemoteStamp + SampleBallTime).
struct TrajSample {
  std::int64_t t_ns{0};
  std::array<double, 3> p{};  // [m]   world
  std::array<double, 3> v{};  // [m/s]
  std::array<double, 3> a{};  // [m/s²]
};

/// Identity a snapshot carries so that consumers can reject a stale or
/// mismatched one (D-22, D-23). The trajectory snapshot, the planner-side
/// covariance buffer and PlanSnapshot all carry the same four values.
struct ProvenanceToken {
  std::uint64_t activation_generation{0};  // base ActivationGeneration() at ingress
  std::uint64_t generation{0};             // vision track epoch (D-4)
  std::uint64_t snapshot_sequence{0};      // monotone; the ONLY "is new" signal (D-21)
  std::int64_t traj_recv_ns{0};            // steady receive instant — source age input
};

/// Vision prediction as the RT tick and the planner see it.
///
/// `n` is untrusted until Check() has accepted it against [n_min, n_max]; every
/// reader bounds-checks it before indexing `s` (the reference sampler indexed
/// first and flagged later — an out-of-range read at n > capacity).
struct TrajectorySnapshot {
  ProvenanceToken token{};
  std::int32_t n{0};
  bool valid{false};
  std::array<TrajSample, kCap> s{};
};

/// Why a plan is absent or a candidate was dropped (L3 §5.2, one code per gate).
enum class PlanReason : std::uint8_t {
  kNone = 0,
  kUncertainty,
  kIkFailed,
  kManipulability,
  kReachTime,
  kLimitsInvalid,
  kGammaWindow,
  kStoppingDistance,
  kRollout,
  kErrorBudget,
  kImpulse,
  kHorizonShort,  // received horizon below the requirement (D-15)
  kBudgetExceeded,
  kInputNonFinite,
};

/// Planner → RT payload (L3 §5.2). Field set frozen here; the planner fills it
/// in S6. Instants are absolute steady ns; t_c / t_cmd / γ-profile times are on
/// the BallTime axis and are compared against now_lead / now per plan §3.
struct PlanSnapshot {
  ProvenanceToken token{};
  std::uint64_t rt_iteration{0};  // RT state snapshot the plan was computed from (D-22)
  std::int64_t rt_state_ns{0};    // its steady instant — state age input
  std::int64_t publish_ns{0};     // when the planner published this

  std::uint32_t plan_id{0};
  std::int64_t t_c_ns{0};       // catch instant (BallTime)
  std::int64_t t_cmd_ns{0};     // hand close command instant (BallTime)
  std::array<double, 3> p_c{};  // catch point [m], world
  std::array<double, 3> a_d{};  // desired approach axis (unit), world
  std::array<double, 3> v_c{};  // predicted ball velocity at t_c [m/s]

  double gamma_g0{0.0};
  double gamma_gf{0.0};
  std::int64_t gamma_t0_ns{0};  // BallTime axis, evaluated against now_lead
  std::int64_t gamma_t1_ns{0};
  double gamma_min{0.0};  // γ-window lower bound (diagnostic, D-8)

  std::array<double, kMaxPlanNv> q_star{};
  std::int32_t nv{0};
  double w5{0.0};  // catchability manipulability, gate definition (D-18)
  double w6{0.0};  // 6-row variant, always recorded alongside (C-3)

  double score{0.0};
  double sigma_c{0.0};
  double sigma_l{0.0};
  double dp_impact{0.0};  // expected impulse [kg m/s]

  PlanReason reason{PlanReason::kNone};
  bool valid{false};
};

static_assert(std::is_trivially_copyable_v<TrajSample>);
static_assert(std::is_trivially_copyable_v<ProvenanceToken>);
static_assert(std::is_trivially_copyable_v<TrajectorySnapshot>);
static_assert(std::is_trivially_copyable_v<PlanSnapshot>);

}  // namespace rtc::catching
