#ifndef RTC_MPC_TYPES_MPC_SOLUTION_TYPES_HPP_
#define RTC_MPC_TYPES_MPC_SOLUTION_TYPES_HPP_

/// @file mpc_solution_types.hpp
/// @brief Trivially copyable MPC-RT interchange types.
///
/// These structs travel through a single-producer/single-consumer TripleBuffer
/// (@see rtc_mpc/comm/triple_buffer.hpp) and a SeqLock
/// (@see rtc_base/threading/seqlock.hpp), both of which require
/// `std::is_trivially_copyable_v<T>`. Dynamic members (std::vector, Eigen::*X*)
/// are therefore forbidden — the fixed-capacity `std::array` members below are
/// sized by the capacity constants, and the **actual** sizes of a solve travel
/// inside each struct.

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rtc::mpc {

/// Capacity constants. They describe "a free-flyer base plus a bounded number
/// of actuated joints, with a bounded number of contact-force variables" and
/// are **capacities, not per-robot values** — topology comes from the model
/// and YAML at runtime (ARCH-1). The derived ones are spelled as expressions
/// of the primary ones so they cannot drift apart; the asserts below state
/// the relations a reader may rely on.
inline constexpr int kMaxHorizon = 64;
inline constexpr int kMaxNv = 36;  ///< max tangent (velocity) dim: free-flyer 6 + joints
inline constexpr int kMaxNq =
    kMaxNv + 1;  ///< max configuration dim: the free-flyer quaternion adds one
inline constexpr int kMaxNdx =
    2 * kMaxNv;  ///< max state-tangent dim `[δq; δv]` — Riccati gain columns
inline constexpr int kMaxContactVars = 24;  ///< max Σ(contact_dim) over the contact frames
/// Max control dim. Kinodynamic input `[W_1 … W_nc, q̈_j]` is
/// `Σ contact_dim + nv − 6`; the fixed-base torque input `nu = nv` is smaller.
inline constexpr int kMaxNu = kMaxNv - 6 + kMaxContactVars;
/// Riccati gain nodes stored per solution (a capacity — the actual count is
/// `MPCSolution::gain_nodes`). The consumer applies the gain of the node
/// nearest in time, so only the leading nodes are ever read; shipping every
/// node's gain would be the bulk of the payload.
inline constexpr int kMaxGainNodes = 4;

static_assert(kMaxNu >= kMaxNv, "fixed-base torque input (nu == nv) must fit");
static_assert(kMaxGainNodes >= 1 && kMaxGainNodes <= kMaxHorizon,
              "at least the first node's gain is shipped, never more than the horizon");

/// @brief Outcome of the solve that produced an @ref MPCSolution.
///
/// `kMaxIters` is still a usable solution (the solver used its iteration
/// budget without meeting tolerance — a warm-start-quality trajectory);
/// the three remaining failure states make @ref MPCSolution::IsValid false.
enum class SolverStatus : std::uint8_t {
  kOk = 0,     ///< converged to tolerance
  kMaxIters,   ///< iteration budget exhausted, trajectory finite and usable
  kDiverged,   ///< non-finite state — do not consume
  kException,  ///< solver threw — do not consume
  kTruncated,  ///< a dimension exceeded a capacity constant — do not consume
  kNotSolved,  ///< no solve ran for this slot (not initialised, state dim mismatch, rebuild needed)
};

/// @brief Solver residuals at the end of the solve (diagnostics, not gates).
struct ResidualSummary {
  double prim_inf{0.0};   ///< primal infeasibility (dynamics + constraints)
  double dual_inf{0.0};   ///< dual infeasibility (stationarity)
  double traj_cost{0.0};  ///< trajectory cost of the returned iterate
  double merit{0.0};      ///< merit function value of the returned iterate
};

/// @brief Optimal trajectory produced by a single MPC solve.
///
/// Layout notes:
/// - State trajectory `(q, v)` has length `horizon_length + 1` (nodes 0..H).
/// - Control, contact-force, and feedforward-acceleration trajectories have
///   length `horizon_length` (nodes 0..H-1).
/// - `K_riccati[k]` is stored row-major as `nu × ndx` (`ndx = 2·nv`, the
///   state-tangent dim) with **row stride = the actual `ndx`**, never the
///   capacity constant. `rtc_mpc/feedback/riccati_feedback.hpp` owns the
///   layout (writer `PackRiccatiGainRowMajor`, reader
///   `RiccatiFeedback::SetGain`). `ndx` equals `nx` only when every joint
///   has `nq == nv`; the reader must take its column count from `ndx`, not
///   from `nx`. Only the first `gain_nodes` nodes carry a gain.
/// - `horizon_length` / `nq` / `nv` / `nu` / `nx` / `ndx` / `n_contact_vars`
///   / `gain_nodes` are the **actual** sizes used for this solve; consumers
///   must not read past them.
///
/// Provenance: `observation_*` identify the state snapshot the solve started
/// from (`observation_stamp_ns` is in the **consumer's clock**, the one
/// handed to `WriteState`), `plan_id` is stamped by the publisher (monotonic
/// per manager), `model_epoch` is copied from that snapshot so the consumer
/// can reject a plan computed against a model it no longer runs. The time
/// the consumer received the plan is recorded on the consumer side in the
/// same clock, so policy age never mixes clock domains; `timestamp_ns` is
/// the producer's own steady clock and is for profiling only.
struct MPCSolution {
  // ── Metadata ────────────────────────────────────────────────────────
  uint64_t timestamp_ns{0};       ///< wall-clock time solve completed
  uint64_t solve_duration_ns{0};  ///< profiling: time spent in solve()
  int horizon_length{0};          ///< valid node count (≤ kMaxHorizon)
  double dt_node{0.0};            ///< OCP node spacing [s]
  bool converged{false};  ///< solver convergence flag; producers keep it equal to `status == kOk`
  int iterations{0};      ///< solver iteration count

  // ── Provenance ──────────────────────────────────────────────────────
  uint64_t observation_stamp_ns{0};  ///< `MPCStateSnapshot::timestamp_ns` the solve used
  uint64_t observation_sequence{0};  ///< `MPCStateSnapshot::sequence` the solve used
  uint64_t plan_id{0};               ///< monotonic publish counter (0 == never published)
  uint32_t model_epoch{0};           ///< model generation the plan was computed against
  SolverStatus status{SolverStatus::kOk};
  ResidualSummary residuals{};

  // ── Dimensions of this solve ────────────────────────────────────────
  int nq{0};
  int nv{0};
  int nu{0};
  int nx{0};   ///< state vector length `nq + nv`
  int ndx{0};  ///< state-tangent dim `2·nv` — the `K_riccati[k]` column count and row stride
  int n_contact_vars{0};
  int gain_nodes{0};  ///< nodes with a Riccati gain (≤ kMaxGainNodes, ≤ horizon_length)

  // ── Trajectories (see header-level comment for index conventions) ──
  std::array<std::array<double, kMaxNq>, kMaxHorizon + 1> q_traj{};
  std::array<std::array<double, kMaxNv>, kMaxHorizon + 1> v_traj{};
  std::array<std::array<double, kMaxNu>, kMaxHorizon> u_traj{};
  std::array<std::array<double, kMaxContactVars>, kMaxHorizon> lambda_traj{};
  std::array<std::array<double, kMaxNv>, kMaxHorizon> a_ff_traj{};

  /// Riccati gains K[k] (nu × ndx, row-major, stride = actual ndx — see the
  /// struct comment). Flattened capacity computed in std::size_t to avoid
  /// implicit widening on capacity arithmetic.
  static constexpr std::size_t kRiccatiGainFlatSize =
      static_cast<std::size_t>(kMaxNu) * static_cast<std::size_t>(kMaxNdx);
  std::array<std::array<double, kRiccatiGainFlatSize>, kMaxGainNodes> K_riccati{};

  /// @return true if this solution is safe to consume.
  /// Does not check `converged`: a `kMaxIters` solution is still better than
  /// a stale one. Every other non-`kOk` status is a failed or absent solve.
  [[nodiscard]] constexpr bool IsValid() const noexcept {
    return horizon_length > 0 && nv > 0 && dt_node > 0.0 &&
           (status == SolverStatus::kOk || status == SolverStatus::kMaxIters);
  }
};

static_assert(std::is_trivially_copyable_v<MPCSolution>,
              "MPCSolution must be trivially copyable (TripleBuffer contract)");

/// @brief Record a completed solve's outcome. The single writer of the
///        `converged` ↔ `status` pair: a solve that returned finite iterates
///        is `kOk` when it met tolerance and `kMaxIters` otherwise.
constexpr void MarkSolved(MPCSolution& sol, bool converged) noexcept {
  sol.converged = converged;
  sol.status = converged ? SolverStatus::kOk : SolverStatus::kMaxIters;
}

/// @brief State snapshot shipped from RT → MPC every control tick.
///
/// Written by the RT thread via `rtc::SeqLock<MPCStateSnapshot>::Store()`,
/// read by the MPC thread at the top of each solve via `Load()`.
/// `sequence` increments per store, `model_epoch` names the model the RT
/// side runs, `contact_mask` is one bit per contact frame in the model
/// handler's frame order (no left/right semantics here — ARCH-1).
struct MPCStateSnapshot {
  std::array<double, kMaxNq> q{};
  std::array<double, kMaxNv> v{};
  uint64_t timestamp_ns{0};
  uint64_t sequence{0};
  uint32_t model_epoch{0};
  uint32_t contact_mask{0};
  int nq{0};
  int nv{0};
  bool valid{false};  ///< false until the RT side has stored a state
};

static_assert(std::is_trivially_copyable_v<MPCStateSnapshot>,
              "MPCStateSnapshot must be trivially copyable (SeqLock contract)");

}  // namespace rtc::mpc

#endif  // RTC_MPC_TYPES_MPC_SOLUTION_TYPES_HPP_
