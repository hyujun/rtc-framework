// ── Shared snapshot types of the catching core (dynamic_catching S1.2) ───────
// The POD payloads that cross threads through rtc::SeqLock:
//
//   TrajectorySnapshot — vision prediction, written by the nrt ingress
//                        (L1), read every RT tick (L2) and by the planner (L3)
//   PlanSnapshot       — planner (L3) → RT tick (L4/L7)
//   SegmentSnapshot  — planner (L3) → RT tick: the segment MPC's stop segment
//                        as joint nodes (MPC plan E1-F02, MD-27)
//
// SeqLock requires trivially copyable payloads, and Eigen::Vector3d is not one,
// so vectors are std::array<double, 3> and the computing side views them with
// Eigen::Map (L0 §5.2, L3 §5.3). Every instant is an absolute steady-ns
// BallTime (L0 §4.5); relative seconds exist only inside the numeric core.
//
// This header is the single owner of the trajectory capacity kCap (L0 §5.2). It
// is shared by L1/L2/L3 so that none of them depends on another for the type.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace rtc::catching {

/// Compile-time capacity of one predicted trajectory [points].
///
/// PROVISIONAL (D-15 / S0.7, user decision 2026-09-19): 40 holds ≈1.95 s at
/// the 0.05 s sim-profile spacing, twice the largest point count the S0.7 sweep
/// needed. The run-time bound n_max ≤ kCap comes from S3.6; if S3.6 needs more,
/// raise this and re-run the S1 gates (ID_INDEX §3). The snapshot is
/// copied whole every RT tick (D-21 forbids gating the copy on
/// SeqLock::sequence()), so this constant IS the per-tick copy cost.
inline constexpr int kCap = 40;

/// Capacity of a joint vector carried in PlanSnapshot (IK solution over the
/// planner's control model). Checked against the model nv at configure time.
/// Separate from kCap, which counts trajectory points, not joints.
inline constexpr int kMaxPlanNv = 32;

/// Node capacity of a published segment: N ≤ kMaxSegmentNodes, so a payload holds
/// N + 1 node columns. The MPC segment core (mpc_segment_core.hpp, which includes this
/// header) uses it for its stop segment and its block array; the core's own
/// horizon, pre-catch nodes included, is bounded by its kMaxMpcNodes.
inline constexpr int kMaxSegmentNodes = 24;

/// Joint capacity of SegmentSnapshot (E1 arms have 6 and 7). Deliberately
/// NOT kMaxPlanNv: at 32 the payload would be 19.2 KB and the RT's SeqLock
/// copy (and its retry window against the planner's store) four times longer
/// for joints no E1 robot has (MD-27). Checked against the arm at configure.
inline constexpr int kMaxSegmentNv = 8;

/// One predicted ball sample. The ball instant is absolute steady ns (BallTime
/// axis, converted once on receipt by ConvertRemoteStamp + SampleBallTime).
/// Position, velocity and acceleration are in the MODEL world (the Pinocchio
/// universe the planner and CLIK work in): the ingress converts the vision
/// frame once on receipt (L3 §4.2 — the two can differ by a rigid transform,
/// e.g. a model rooted at a frame rotated from the vision world).
struct TrajSample {
  std::int64_t t_ns{0};
  std::array<double, 3> p{};  // [m]   model world
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
/// the BallTime axis and are compared against now_lead / now per L0 §4.5.
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

/// Planner → RT: the segment MPC's stop segment (MPC plan E1-F02, MD-27). A
/// SIBLING of PlanSnapshot on its own SeqLock, not a field of it: the RT stops
/// taking PlanSnapshots once it has committed (JudgePlan too_late / repeat),
/// and the stop segment is re-planned after that, up to k_max grid points past
/// t_c (MD-31).
///
/// Nodes are joint states in the arm's DEVICE order — the order PlannerRtState
/// and PlanSnapshot::q_star speak — at t0_ns + k·dt_ns, k = 0..n_nodes. Between
/// nodes the jerk is constant (jerk_segment.hpp), so the RT reproduces exactly
/// the trajectory the QP optimised and derives the hand's pose and twist from
/// it by FK (MD-9). Storage is node-major with a fixed stride of kMaxSegmentNv:
/// entry (joint j, node k) is `[k * kMaxSegmentNv + j]`, which an Eigen::Map with
/// OuterStride<kMaxSegmentNv> views as the n × (N+1) node matrix the sampler
/// takes, without a copy.
///
/// Instants are absolute steady ns on the LEAD axis (t_c is BallTime, which
/// the RT compares against now_lead; L0 §4.5). t0_ns is the EFFECTIVE instant
/// t_eff — t_c for the pre-catch solve, a later grid point for a post-catch
/// replan (MD-10, MD-31) — never "t_c" by assumption. All grid arithmetic is
/// integer ns so that t0_ns lands exactly on t_c + k·dt_ns.
///
/// TWO SPACINGS (MD-54, MD-60). An APPROACH–stop segment (n_pre > 0) starts
/// n_pre pre-catch intervals of dt_pre_ns before t_c and continues with the
/// n_nodes − n_pre stop intervals of dt_ns after it: node n_pre is the catch
/// node at t_c, and node 0 is t_c − n_pre·dt_pre_ns (k0 is 0). n_pre = 0 is
/// a segment that starts at t_c or after it — the stop part alone, as above.
/// SegmentNodeTimeNs() is the one place a node's
/// instant is computed.
///
/// THE CATCH INTERVAL'S OWN LENGTH (E1-F14, #740). A search that lets the
/// catch instant move inside its candidate's cell stretches ONE interval — the
/// last one before the catch node — and leaves every pre-catch node where it
/// was. dt_catch_ns is that interval's length: node n_pre − 1 is at
/// t0 + (n_pre − 1)·dt_pre_ns and the catch node dt_catch_ns after it, at t_c;
/// node 0 is t_c − (n_pre − 1)·dt_pre_ns − dt_catch_ns. 0 means "the same as
/// dt_pre_ns" and is the ONLY way to say so: a writer whose catch interval
/// came out at dt_pre_ns writes 0, and the validator refuses dt_catch_ns equal
/// to dt_pre_ns — one grid, one encoding, so a payload without the field is
/// read by exactly the arithmetic it was read by before the field existed.
struct SegmentSnapshot {
  // activation_generation and track generation of the followed plan. A
  // segment carries the PLAN's track (its first segment is packed from the
  // plan's own token and every later one from its source), which the RT's
  // latest consumed track need not be after the freeze. snapshot_sequence /
  // traj_recv_ns stay 0.
  ProvenanceToken token{};
  std::uint64_t rt_iteration{0};  // RT state the stop was predicted from (D-22)
  std::int64_t rt_state_ns{0};
  std::int64_t publish_ns{0};

  std::uint32_t plan_id{0};      // the PlanSnapshot (catch plan) this stop ends
  std::uint32_t segment_seq{0};  // planner's own monotone counter — the "is new" signal
  std::int64_t t_c_ns{0};        // catch instant of that plan (grid anchor)
  std::int64_t t0_ns{0};         // node 0 instant, t_eff = t_c + k0·dt_ns
  std::int64_t dt_ns{0};         // node spacing Δ_s
  std::int64_t dt_pre_ns{0};     // pre-catch spacing Δ_pre; used only when n_pre > 0
  std::int32_t k0{0};            // grid index of node 0 (0 = pre-catch solve)
  std::int32_t n_nodes{0};       // N; the segment ends at t0 + N·dt = t_c + N_s·dt
  std::int32_t nv{0};
  std::int32_t n_pre{0};  // pre-catch intervals before t_c (0 = the stop part alone)
  // Length of the interval that ends at the catch node; 0 = dt_pre_ns (the
  // only encoding of that). Used only when n_pre > 0.
  std::int64_t dt_catch_ns{0};

  std::array<double, kMaxSegmentNv*(kMaxSegmentNodes + 1)> q{};    // [rad]
  std::array<double, kMaxSegmentNv*(kMaxSegmentNodes + 1)> qd{};   // [rad/s]
  std::array<double, kMaxSegmentNv*(kMaxSegmentNodes + 1)> qdd{};  // [rad/s²]

  // The QP's own account of the published solution (MD-33 publish gate).
  double slack_max{0.0};           // fraction of τ_max
  double slack_terminal_max{0.0};  // node N — static torque at the stop posture
  double tau_ratio_max{0.0};       // linearised max |τ/τ_max|, nodes 1..N
  bool x0_clamped{false};          // the start state was projected into the core's box

  bool valid{false};
};

/// |q̇|, |q̈| bound on node N of a published segment. The sampler HOLDS node N
/// past the end (jerk_segment.hpp), so a moving node N would be followed as a
/// frozen velocity; the MPC's terminal equality makes it zero to solver
/// tolerance, far below this.
inline constexpr double kSegmentRestTol = 1e-3;

/// Upper bound on the pre-catch spacing a payload may carry, so that
/// n_pre·dt_pre_ns cannot overflow (a shape bound, not a tuning limit).
inline constexpr std::int64_t kMaxSegmentDtPreNs = 1'000'000'000;

/// Upper bound on the catch interval's own length (the same kind of bound): a
/// catch instant moves by less than one pre-catch spacing inside its cell.
inline constexpr std::int64_t kMaxSegmentDtCatchNs = 2 * kMaxSegmentDtPreNs;

/// Instant of node k of a two-spacing grid (MD-54): pre-catch nodes at
/// t0 + k·dt_pre, the catch node at t_c, stop nodes at t_c + (k − n_pre)·dt;
/// with n_pre = 0, t0 + k·dt. The grid rule lives here — the payload's node
/// instants and the planner's own grid both read it.
///
/// `dt_catch_ns` ≠ 0 is a grid whose last pre-catch interval has its own
/// length (SegmentSnapshot's note): the nodes before the catch node stay at
/// t0 + k·dt_pre, and the catch node is at t_c — which is then NOT
/// t0 + n_pre·dt_pre.
[[nodiscard]] constexpr std::int64_t SegmentGridNodeTimeNs(std::int64_t t0_ns, std::int64_t t_c_ns,
                                                           std::int64_t dt_pre_ns,
                                                           std::int64_t dt_ns, int n_pre, int k,
                                                           std::int64_t dt_catch_ns = 0) noexcept {
  if (n_pre > 0 && dt_catch_ns != 0 && k >= n_pre) {
    return t_c_ns + static_cast<std::int64_t>(k - n_pre) * dt_ns;
  }
  if (n_pre > 0) {
    return k <= n_pre ? t0_ns + static_cast<std::int64_t>(k) * dt_pre_ns
                      : t_c_ns + static_cast<std::int64_t>(k - n_pre) * dt_ns;
  }
  return t0_ns + static_cast<std::int64_t>(k) * dt_ns;
}

/// Instant of node k (0 ≤ k ≤ n_nodes) of a payload whose shape the caller
/// has checked (SegmentGridNodeTimeNs on its fields).
[[nodiscard]] constexpr std::int64_t SegmentNodeTimeNs(const SegmentSnapshot& p, int k) noexcept {
  return SegmentGridNodeTimeNs(p.t0_ns, p.t_c_ns, p.dt_pre_ns, p.dt_ns, p.n_pre, k, p.dt_catch_ns);
}

/// Whether a SegmentSnapshot's shape and node values can be sampled: sizes
/// inside the capacities, a positive spacing, node 0 ON the grid t_c + k0·Δ
/// (k0 ≥ 0) — or, with n_pre > 0, at t_c − n_pre·Δ_pre with k0 = 0 and at
/// least one stop interval after t_c — node N at rest (kSegmentRestTol), and
/// every used node entry finite. A catch interval with its own length
/// (dt_catch_ns ≠ 0) needs n_pre > 0, a length in (0, kMaxSegmentDtCatchNs]
/// that is NOT dt_pre_ns (that grid is written with 0), and node 0 at
/// t_c − (n_pre − 1)·Δ_pre − Δ_catch. The RT runs this once per NEW payload (by
/// segment_seq), not per tick — the sampler itself does not check node values
/// (jerk_segment.hpp), so an unvalidated NaN node would reach the CLIK target.
[[nodiscard]] inline bool ValidateSegmentNodes(const SegmentSnapshot& p) noexcept {
  if (!p.valid || p.nv < 1 || p.nv > kMaxSegmentNv || p.n_nodes < 1 ||
      p.n_nodes > kMaxSegmentNodes || p.dt_ns <= 0 || p.k0 < 0 || p.k0 > kMaxSegmentNodes ||
      p.n_pre < 0 || p.n_pre >= p.n_nodes) {
    return false;
  }
  if (p.n_pre > 0) {
    if (p.dt_pre_ns <= 0 || p.dt_pre_ns > kMaxSegmentDtPreNs || p.k0 != 0) {
      return false;
    }
    if (p.dt_catch_ns == 0) {
      if (p.t0_ns != p.t_c_ns - static_cast<std::int64_t>(p.n_pre) * p.dt_pre_ns) {
        return false;
      }
    } else if (p.dt_catch_ns < 0 || p.dt_catch_ns > kMaxSegmentDtCatchNs ||
               p.dt_catch_ns == p.dt_pre_ns ||
               p.t0_ns != p.t_c_ns - static_cast<std::int64_t>(p.n_pre - 1) * p.dt_pre_ns -
                              p.dt_catch_ns) {
      return false;
    }
  } else if (p.dt_catch_ns != 0 ||
             p.t0_ns != p.t_c_ns + static_cast<std::int64_t>(p.k0) * p.dt_ns) {
    return false;
  }
  for (int k = 0; k <= p.n_nodes; ++k) {
    for (int j = 0; j < p.nv; ++j) {
      const auto i = static_cast<std::size_t>(k * kMaxSegmentNv + j);
      if (!std::isfinite(p.q[i]) || !std::isfinite(p.qd[i]) || !std::isfinite(p.qdd[i])) {
        return false;
      }
      if (k == p.n_nodes &&
          !(std::fabs(p.qd[i]) <= kSegmentRestTol && std::fabs(p.qdd[i]) <= kSegmentRestTol)) {
        return false;
      }
    }
  }
  return true;
}

static_assert(std::is_trivially_copyable_v<TrajSample>);
static_assert(std::is_trivially_copyable_v<ProvenanceToken>);
static_assert(std::is_trivially_copyable_v<TrajectorySnapshot>);
static_assert(std::is_trivially_copyable_v<PlanSnapshot>);
static_assert(std::is_trivially_copyable_v<SegmentSnapshot>);
// The size MD-27 budgets: three node blocks of kMaxSegmentNv × (kMaxSegmentNodes + 1).
static_assert(sizeof(SegmentSnapshot) < 5 * 1024);

}  // namespace rtc::catching
