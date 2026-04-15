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
/// sized for the worst-case manipulator we expect to support.

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rtc::mpc {

/// Upper-bound capacity constants. Values chosen for UR5e + 10-DoF hand
/// (16 DoF) with headroom for dual-arm setups; over-allocation is acceptable
/// because the buffer lives in the MPC thread's pre-allocated arena.
inline constexpr int kMaxHorizon = 64;
inline constexpr int kMaxNq = 32;           ///< max generalized-coordinate dim
inline constexpr int kMaxNv = 32;           ///< max generalized-velocity dim
inline constexpr int kMaxNu = 24;           ///< max control-input dim
inline constexpr int kMaxNx = 48;           ///< max state dim (nq + nv)
inline constexpr int kMaxContactVars = 24;  ///< max Σ(contact_dim)

/// @brief Optimal trajectory produced by a single MPC solve.
///
/// Layout notes:
/// - State trajectory `(q, v)` has length `horizon_length + 1` (nodes 0..H).
/// - Control, contact-force, and feedforward-acceleration trajectories have
///   length `horizon_length` (nodes 0..H-1).
/// - `K_riccati[k]` is stored row-major as `nu × nx`.
/// - `horizon_length` / `nq` / `nv` / `nu` / `nx` / `n_contact_vars` are the
///   **actual** sizes used for this solve; consumers must not read past them.
struct MPCSolution {
  // ── Metadata ────────────────────────────────────────────────────────
  uint64_t timestamp_ns{0};       ///< wall-clock time solve completed
  uint64_t solve_duration_ns{0};  ///< profiling: time spent in solve()
  int horizon_length{0};          ///< valid node count (≤ kMaxHorizon)
  double dt_node{0.0};            ///< OCP node spacing [s]
  bool converged{false};          ///< solver convergence flag
  int iterations{0};              ///< solver iteration count

  // ── Dimensions of this solve ────────────────────────────────────────
  int nq{0};
  int nv{0};
  int nu{0};
  int nx{0};
  int n_contact_vars{0};

  // ── Trajectories (see header-level comment for index conventions) ──
  std::array<std::array<double, kMaxNq>, kMaxHorizon + 1> q_traj{};
  std::array<std::array<double, kMaxNv>, kMaxHorizon + 1> v_traj{};
  std::array<std::array<double, kMaxNu>, kMaxHorizon> u_traj{};
  std::array<std::array<double, kMaxContactVars>, kMaxHorizon> lambda_traj{};
  std::array<std::array<double, kMaxNv>, kMaxHorizon> a_ff_traj{};

  /// Riccati gains K[k] (nu × nx, row-major). Flattened length computed in
  /// std::size_t to avoid implicit widening on capacity arithmetic.
  static constexpr std::size_t kRiccatiGainFlatSize =
      static_cast<std::size_t>(kMaxNu) * static_cast<std::size_t>(kMaxNx);
  std::array<std::array<double, kRiccatiGainFlatSize>, kMaxHorizon>
      K_riccati{};

  /// @return true if this solution is safe to consume.
  /// Does not check `converged` because a non-converged but usable warm-start
  /// solution is still better than a stale one.
  [[nodiscard]] constexpr bool IsValid() const noexcept {
    return horizon_length > 0 && nv > 0 && dt_node > 0.0;
  }
};

static_assert(std::is_trivially_copyable_v<MPCSolution>,
              "MPCSolution must be trivially copyable (TripleBuffer contract)");

/// @brief State snapshot shipped from RT → MPC every control tick.
///
/// Written by the RT thread via `rtc::SeqLock<MPCStateSnapshot>::Store()`,
/// read by the MPC thread at the top of each solve via `Load()`.
struct MPCStateSnapshot {
  std::array<double, kMaxNq> q{};
  std::array<double, kMaxNv> v{};
  uint64_t timestamp_ns{0};
  int nq{0};
  int nv{0};
};

static_assert(std::is_trivially_copyable_v<MPCStateSnapshot>,
              "MPCStateSnapshot must be trivially copyable (SeqLock contract)");

}  // namespace rtc::mpc

#endif  // RTC_MPC_TYPES_MPC_SOLUTION_TYPES_HPP_
