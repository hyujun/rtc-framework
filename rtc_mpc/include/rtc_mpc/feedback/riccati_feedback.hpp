#ifndef RTC_MPC_FEEDBACK_RICCATI_FEEDBACK_HPP_
#define RTC_MPC_FEEDBACK_RICCATI_FEEDBACK_HPP_

/// @file riccati_feedback.hpp
/// @brief Riccati-gain feedback for robustifying the MPC reference.
///
/// Computes @f$ u_{fb} = s \cdot K \cdot \Delta x @f$ where
/// @f$ \Delta x = \begin{bmatrix} q_{curr} - q_{ref} \\ v_{curr} - v_{ref}
/// \end{bmatrix} @f$,
/// `K` is the Riccati gain at the current node (nearest-neighbour lookup,
/// never interpolated — linear interpolation of gain matrices is not
/// guaranteed to preserve the stabilising property), and `s ∈ [0, 1]` is a
/// runtime-tunable scale so the feedback can be smoothly faded in.
///
/// Two consumption modes:
/// - `accel_only = true` (default): only the first `nv` rows of `u_fb` are
///   written. Intended for downstream TSID task feedforward where the
///   Riccati output is fused with the interpolator's `a_ff` term.
/// - `accel_only = false`: the full `nu`-dim feedback is written; caller
///   handles both acceleration and contact-force feedforward.
///
/// Numerical safety:
/// - Large state errors (e.g. during E-STOP recovery) are squashed by a
///   `Δx` norm cap; above the cap, `u_fb` is linearly scaled down so the
///   magnitude of the applied feedback is bounded.
///
/// RT-safety: all matrices and vectors are sized at @ref Init and never
/// resized on the hot path. Eigen noalias is used for the matrix-vector
/// product. No allocations, no exceptions.

#include <Eigen/Core>

#include <cstddef>

namespace rtc::mpc {

/// @brief Flatten a Riccati feedback block `K` into the layout
///        @ref RiccatiFeedback::SetGain reads: row-major with the row stride
///        equal to the block's **actual** column count (the state-tangent
///        dim), never a capacity constant. This is the single writer-side
///        definition of the `MPCSolution::K_riccati[k]` layout — every
///        producer (solver handler, mocks, test helpers) goes through it so
///        writer and reader cannot drift apart.
/// @param K              feedback block (`nu × ndx`). The fully dynamic
///                       stride lets any direct-access ColMajor view — a
///                       corner block as well as one with a non-unit inner
///                       stride — bind without a temporary, where a plain
///                       `Ref<const MatrixXd>` would silently heap-allocate.
///                       A RowMajor expression is still copied (storage order
///                       must match); the solver's gains are ColMajor
/// @param flat           destination buffer
/// @param flat_capacity  number of doubles available at `flat`
/// @return false (buffer untouched) when `K.rows() · K.cols()` exceeds
///         `flat_capacity` or `flat` is null. RT-safe: no allocation, no
///         exception.
using RiccatiGainBlockRef =
    Eigen::Ref<const Eigen::MatrixXd, 0, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>;
[[nodiscard]] bool PackRiccatiGainRowMajor(const RiccatiGainBlockRef& K, double* flat,
                                           std::size_t flat_capacity) noexcept;

class RiccatiFeedback {
 public:
  RiccatiFeedback() = default;
  ~RiccatiFeedback() = default;

  RiccatiFeedback(const RiccatiFeedback&) = delete;
  RiccatiFeedback& operator=(const RiccatiFeedback&) = delete;
  RiccatiFeedback(RiccatiFeedback&&) = delete;
  RiccatiFeedback& operator=(RiccatiFeedback&&) = delete;

  /// @brief Allocate workspace. Call once during controller Init.
  /// @param max_nv  velocity dim (upper bound; @ref Compute accepts any
  ///                `nv ≤ max_nv`)
  /// @param max_nu  control-input dim
  /// @param max_nx  state dim (typically `nq + nv`)
  void Init(int max_nv, int max_nu, int max_nx);

  /// @brief Install a new Riccati gain matrix.
  /// @param K_data  pointer to row-major `nu × nx` data, read as
  ///                `K_data[row * nx + col]` — the row stride is the actual
  ///                `nx`, the layout @ref PackRiccatiGainRowMajor writes
  /// @param nu      actual control dim (must satisfy `nu ≤ max_nu`)
  /// @param nx      actual state dim (must satisfy `nx ≤ max_nx`)
  /// @note No-op if bounds are violated.
  void SetGain(const double* K_data, int nu, int nx) noexcept;

  /// @brief Compute @f$ u_{fb} = s \cdot K \cdot \Delta x @f$.
  ///
  /// @param q_curr   current joint position (size nq)
  /// @param v_curr   current joint velocity (size nv)
  /// @param q_ref    reference joint position (size nq)
  /// @param v_ref    reference joint velocity (size nv)
  /// @param u_fb_out output buffer; at minimum `nv` entries are written,
  ///                 `nu` entries when @ref SetAccelOnly(false)
  ///
  /// On @f$ \|\Delta x\| > `max_delta_x_norm_` @f$, `Δx` is scaled so the
  /// effective @f$ u_{fb} @f$ magnitude stays bounded.
  void Compute(const Eigen::Ref<const Eigen::VectorXd>& q_curr,
               const Eigen::Ref<const Eigen::VectorXd>& v_curr,
               const Eigen::Ref<const Eigen::VectorXd>& q_ref,
               const Eigen::Ref<const Eigen::VectorXd>& v_ref,
               Eigen::Ref<Eigen::VectorXd> u_fb_out) noexcept;

  /// @brief Runtime gain scale, clamped to `[0, 1]`.
  void SetGainScale(double scale) noexcept;

  [[nodiscard]] double GainScale() const noexcept { return gain_scale_; }

  /// @brief Toggle "acceleration-only" mode (see class docs).
  void SetAccelOnly(bool accel_only) noexcept { accel_only_ = accel_only; }

  [[nodiscard]] bool AccelOnly() const noexcept { return accel_only_; }

  /// @brief Cap on @f$ \|\Delta x\| @f$ before feedback magnitude is
  ///        scaled down. Must be positive.
  void SetMaxDeltaXNorm(double value) noexcept;

  [[nodiscard]] double MaxDeltaXNorm() const noexcept { return max_delta_x_norm_; }

  [[nodiscard]] bool HasGain() const noexcept { return nu_ > 0 && nx_ > 0; }

 private:
  Eigen::MatrixXd K_;         // nu × nx (allocated max_nu × max_nx)
  Eigen::VectorXd dx_;        // nx     (allocated max_nx)
  Eigen::VectorXd u_fb_raw_;  // nu     (allocated max_nu)

  int max_nv_{0};
  int max_nu_{0};
  int max_nx_{0};
  int nu_{0};  ///< Actual gain rows after SetGain (0 == uninitialised)
  int nx_{0};  ///< Actual gain columns (state-tangent dim)

  double gain_scale_{1.0};
  bool accel_only_{true};
  double max_delta_x_norm_{100.0};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_FEEDBACK_RICCATI_FEEDBACK_HPP_
