#ifndef RTC_MPC_INTERPOLATION_TRAJECTORY_INTERPOLATOR_HPP_
#define RTC_MPC_INTERPOLATION_TRAJECTORY_INTERPOLATOR_HPP_

/// @file trajectory_interpolator.hpp
/// @brief Node-to-node interpolation of an @ref MPCSolution for RT consumption.
///
/// The MPC thread produces a trajectory at a coarse node spacing `dt_node`
/// (typically 10 ms), while the RT control loop samples at `dt_rt` (typically
/// 2 ms). @ref TrajectoryInterpolator bridges the gap: given an absolute
/// wall-clock time, it returns an interpolated `(q_ref, v_ref, a_ff,
/// lambda_ref)` tuple plus metadata describing progress through the horizon.
///
/// Interpolation kernel (Step 5 and later):
/// - `q` and `v` use cubic Hermite so the produced signal is `C^1` continuous
///   at node boundaries and tracks a cubic trajectory exactly.
/// - `a_ff` is the second time-derivative of the same Hermite polynomial,
///   providing a physically consistent feedforward acceleration.
/// - `lambda` uses linear interpolation because contact forces are allowed
///   to be non-smooth (impact / lift-off events).
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

#include <Eigen/Core>

#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <cstdint>

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

  /// @brief Allocate working buffers. Call once during controller Init,
  ///        before any hot-path calls.
  /// @param nq              joint-position dimension
  /// @param nv              joint-velocity dimension
  /// @param n_contact_vars  total contact-force dimension (Σ contact_dim)
  void Init(int nq, int nv, int n_contact_vars);

  /// @brief Install a newly-acquired MPC solution as the active trajectory.
  ///
  /// @param sol           freshly acquired solution (copied into an internal
  ///                      slot so the caller may reuse @p sol immediately)
  /// @param receive_ns    wall-clock time (ns) corresponding to node 0 of
  ///                      this solution; subsequent @ref Interpolate calls
  ///                      use this as the time origin
  void SetSolution(const MPCSolution& sol, uint64_t receive_ns) noexcept;

  /// @brief Interpolate the active solution at absolute time @p now_ns.
  ///
  /// Outputs are written into caller-supplied Eigen blocks so the
  /// interpolator can remain Eigen::Ref-free and allocation-free.
  ///
  /// @param now_ns         current wall-clock time (ns)
  /// @param q_ref          out: position reference (size nq)
  /// @param v_ref          out: velocity reference (size nv)
  /// @param a_ff           out: feedforward acceleration (size nv, zero in
  ///                       linear mode)
  /// @param lambda_ref     out: contact-force reference (size n_contact_vars)
  /// @param meta_out       out: progress / validity flags
  void Interpolate(uint64_t now_ns,
                   Eigen::Ref<Eigen::VectorXd> q_ref,
                   Eigen::Ref<Eigen::VectorXd> v_ref,
                   Eigen::Ref<Eigen::VectorXd> a_ff,
                   Eigen::Ref<Eigen::VectorXd> lambda_ref,
                   InterpMeta& meta_out) noexcept;

  /// @return true once a valid solution has been installed via
  ///         @ref SetSolution.
  [[nodiscard]] bool HasSolution() const noexcept { return has_solution_; }

  /// @return remaining horizon time in seconds, or 0 if no solution or
  ///         beyond horizon.
  [[nodiscard]] double RemainingHorizonSec(uint64_t now_ns) const noexcept;

 private:
  /// Cubic Hermite kernel for q / v / a_ff (+ linear for lambda).
  /// @param node_k  left node index (k ≥ 0, k + 1 ≤ horizon_length)
  /// @param alpha   normalized time within the segment, ∈ [0, 1]
  void InterpolateHermite(int node_k, double alpha,
                          Eigen::Ref<Eigen::VectorXd> q_ref,
                          Eigen::Ref<Eigen::VectorXd> v_ref,
                          Eigen::Ref<Eigen::VectorXd> a_ff,
                          Eigen::Ref<Eigen::VectorXd> lambda_ref) noexcept;

  MPCSolution sol_{};
  uint64_t receive_ns_{0};
  bool has_solution_{false};

  int nq_{0};
  int nv_{0};
  int n_contact_vars_{0};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_INTERPOLATION_TRAJECTORY_INTERPOLATOR_HPP_
