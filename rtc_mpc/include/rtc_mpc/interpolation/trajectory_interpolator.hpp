#ifndef RTC_MPC_INTERPOLATION_TRAJECTORY_INTERPOLATOR_HPP_
#define RTC_MPC_INTERPOLATION_TRAJECTORY_INTERPOLATOR_HPP_

/// @file trajectory_interpolator.hpp
/// @brief Node-to-node interpolation of an @ref MPCSolution for RT consumption.
///
/// The MPC thread produces a trajectory at a coarse node spacing `dt_node`
/// (typically 10 ms), while the RT control loop samples at `dt_rt` (typically
/// 2 ms). @ref TrajectoryInterpolator bridges the gap: given an absolute time
/// in the consumer's clock, it returns an interpolated `(q_ref, v_ref, a_ref,
/// wrench)` tuple plus metadata describing progress through the horizon.
///
/// Time: node `k` of the installed plan sits at `origin + k·dt_ns`, with
/// `dt_ns = round(dt_node · 1e9)` and integer nanoseconds throughout, so a
/// sample at a node time lands on that node with `α = 0` exactly. Before the
/// origin the plan is sampled at node 0; from `origin + H·dt_ns` on it is
/// held on node `H` (`beyond_horizon`, zero acceleration).
///
/// One kernel, a fixed or a floating base (@ref Sample takes the layout):
/// - Joint rows: cubic Hermite in `q` with the node velocities as slopes, so
///   the signal is `C^1` at node boundaries and tracks a cubic exactly; the
///   acceleration is the same polynomial's second derivative. Joint `i` reads
///   configuration entry `n_base_q + i` and tangent entry `n_base_v + i`, so
///   a fixed base (both offsets 0) runs the identical arithmetic.
/// - Free-flyer rows: the quaternion is the slerp of the two node
///   quaternions (the plan keeps them on one hemisphere), normalised; the
///   position is a cubic Hermite whose slopes are the node LOCAL linear
///   velocities rotated to world by each node's orientation; the LOCAL twist
///   is interpolated linearly; the acceleration is the plan's interval mean
///   (`a_ff_traj[k]`, held over the interval).
/// - Contact wrench and contact mask: held over the interval (the OCP's
///   input is constant there): `lambda_traj[k]` with the slots of contacts
///   inactive in `node_mode[k]` zeroed, mask `node_mode[k]`; past the
///   horizon, interval `H − 1`'s.
///
/// Cubic Hermite basis (with `Δt = dt_node`, `s ∈ [0, 1]`):
/// @f$ h_{00}(s) = 2 s^3 - 3 s^2 + 1 @f$,
/// @f$ h_{10}(s) = s^3 - 2 s^2 + s @f$,
/// @f$ h_{01}(s) = -2 s^3 + 3 s^2 @f$,
/// @f$ h_{11}(s) = s^3 - s^2 @f$.
/// Position:
/// @f$ q(s) = h_{00} q_k + h_{10} \Delta t\, v_k + h_{01} q_{k+1} + h_{11} \Delta t\, v_{k+1} @f$.
/// Velocity and acceleration are obtained by differentiating with the chain
/// rule (division by `Δt` and `Δt²` respectively).
/// Reference: Bartels, Beatty, Barsky, "An Introduction to Splines for Use
/// in Computer Graphics and Geometric Modeling" (1987), §3.6.
///
/// RT-safety: all working buffers are sized and allocated in @ref Init; the
/// hot-path methods never allocate and never throw.
///
/// Thread-safety: a single consumer (typically the RT control thread) owns
/// a TrajectoryInterpolator exclusively. @ref SetSolution and
/// @ref Interpolate must not be called concurrently.

#include "rtc_mpc/model/wb_layout.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <memory>

namespace rtc::mpc {

/// Metadata reported alongside an interpolated reference.
struct InterpMeta {
  /// Normalized progress through the horizon in `[0, 1]`. Undefined when
  /// @ref valid is false.
  double progress{0.0};
  /// True if @ref TrajectoryInterpolator holds a usable solution and the
  /// interpolated outputs are populated.
  bool valid{false};
  /// True when the current time has advanced past the last solution node.
  /// Consumers typically treat this as a signal to request a fresh MPC
  /// solve or fall back to a safe reference.
  bool beyond_horizon{false};
  /// Never set: a floating layout is sampled like a fixed one.
  bool floating_unsupported{false};
};

/// Where one sample landed in the plan (@ref TrajectoryInterpolator::Sample).
struct SampleSpot {
  bool valid{false};           ///< a plan was sampled and the outputs written
  bool beyond_horizon{false};  ///< held on the last node
  double progress{0.0};        ///< `τ / (H · dt_ns)` clamped to `[0, 1]`
  int node{0};                 ///< interval index `k` (`H` beyond the horizon)
  int64_t tau_ns{0};           ///< time since the origin, clamped at 0
  int64_t dt_ns{0};            ///< the node spacing in integer nanoseconds
};

/// @brief Interpolates between MPC solution nodes at RT rate.
///
/// Usage:
/// @code
/// TrajectoryInterpolator interp;
/// interp.Init(nq, nv, n_contact_vars);
/// // on every new MPC solution:
/// interp.SetSolution(sol, rt_clock_ns);
/// // every RT tick:
/// InterpMeta meta;
/// interp.Interpolate(now_ns, q_ref, v_ref, a_ff, lambda_ref, meta);
/// @endcode
class TrajectoryInterpolator {
 public:
  TrajectoryInterpolator() = default;
  ~TrajectoryInterpolator() = default;

  TrajectoryInterpolator(const TrajectoryInterpolator&) = delete;
  TrajectoryInterpolator& operator=(const TrajectoryInterpolator&) = delete;
  TrajectoryInterpolator(TrajectoryInterpolator&&) = delete;
  TrajectoryInterpolator& operator=(TrajectoryInterpolator&&) = delete;

  /// @brief Allocate working buffers (the heap slot that holds the active
  ///        solution copy). Call once during controller Init, before any
  ///        hot-path calls; the object itself stays small.
  /// @param nq              joint-position dimension
  /// @param nv              joint-velocity dimension
  /// @param n_contact_vars  total contact-force dimension (Σ contact_dim)
  void Init(int nq, int nv, int n_contact_vars);

  /// @brief Install @p sol with node 0 at @p origin_ns (consumer clock; may
  ///        be negative or ahead of now). Same acceptance as @ref SetSolution.
  [[nodiscard]] bool SetSolutionWithOrigin(const MPCSolution& sol, int64_t origin_ns) noexcept;

  /// @brief Sample the active plan at @p now_ns for the layout @p layout
  ///        (dimensions as the plan's; see the file comment for the rows).
  /// @param q, v, a   out: `nq`, `nv`, `nv` entries
  /// @param wrench    out: `n_contact_vars` entries
  /// @param mask      out: the interval's contact mode (0 without a plan)
  /// @return where the sample landed; `valid == false` (outputs untouched)
  ///         without a plan, with a node spacing that is not a positive,
  ///         finite number of nanoseconds, or on a floating @p layout whose
  ///         base block the plan cannot hold.
  SampleSpot Sample(int64_t now_ns, const WbLayout& layout, double* q, double* v, double* a,
                    double* wrench, uint32_t& mask) const noexcept;

  /// @brief Install a newly-acquired MPC solution as the active trajectory.
  ///
  /// @param sol           freshly acquired solution (copied into an internal
  ///                      slot so the caller may reuse @p sol immediately)
  /// @param receive_ns    time (ns) of node 0 of this solution; subsequent
  ///                      @ref Interpolate calls use this as the time origin
  /// @return false when @p sol is not consumable (`IsValid()` false) or
  ///         @ref Init has not allocated the slot; the solution installed
  ///         before, if any, stays active.
  [[nodiscard]] bool SetSolution(const MPCSolution& sol, uint64_t receive_ns) noexcept;

  /// @return the solution installed by the last accepted @ref SetSolution
  ///         (its provenance fields included), or nullptr when there is
  ///         none.
  [[nodiscard]] const MPCSolution* Solution() const noexcept {
    return has_solution_ ? sol_.get() : nullptr;
  }

  /// @brief Interpolate the active solution at absolute time @p now_ns as a
  ///        fixed-base plan (@ref Sample with both base offsets 0 and no
  ///        contact table: the wrench is held, unmasked).
  ///
  /// @param now_ns         current time (ns), the origin's clock
  /// @param q_ref          out: position reference (size nq)
  /// @param v_ref          out: velocity reference (size nv)
  /// @param a_ff           out: feedforward acceleration (size nv, zero in
  ///                       linear mode)
  /// @param lambda_ref     out: contact-force reference (size n_contact_vars),
  ///                       held over each node interval
  /// @param meta_out       out: progress / validity flags
  void Interpolate(uint64_t now_ns, Eigen::Ref<Eigen::VectorXd> q_ref,
                   Eigen::Ref<Eigen::VectorXd> v_ref, Eigen::Ref<Eigen::VectorXd> a_ff,
                   Eigen::Ref<Eigen::VectorXd> lambda_ref, InterpMeta& meta_out) noexcept;

  /// @return true once a valid solution has been installed via
  ///         @ref SetSolution (and not dropped by @ref Clear since).
  [[nodiscard]] bool HasSolution() const noexcept { return has_solution_; }

  /// @brief Drop the active solution: @ref HasSolution is false until the
  ///        next accepted @ref SetSolution.
  void Clear() noexcept { has_solution_ = false; }

  /// @return remaining horizon time in seconds, or 0 if no solution or
  ///         beyond horizon.
  [[nodiscard]] double RemainingHorizonSec(uint64_t now_ns) const noexcept;

 private:
  std::unique_ptr<MPCSolution> sol_;
  int64_t origin_ns_{0};
  bool has_solution_{false};
  WbLayout plain_layout_{};  ///< fixed-base layout of the Init dims (@ref Interpolate)

  int nq_{0};
  int nv_{0};
  int n_contact_vars_{0};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_INTERPOLATION_TRAJECTORY_INTERPOLATOR_HPP_
