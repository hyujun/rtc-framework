#ifndef RTC_MPC_FEEDBACK_RICCATI_FEEDBACK_HPP_
#define RTC_MPC_FEEDBACK_RICCATI_FEEDBACK_HPP_

/// @file riccati_feedback.hpp
/// @brief Riccati-gain feedback for robustifying the MPC reference.
///
/// Computes @f$ u_{fb} = s \cdot K \cdot \Delta x @f$ where `K` is the Riccati
/// gain at the current node (nearest-neighbour lookup, never interpolated —
/// linear interpolation of gain matrices is not guaranteed to preserve the
/// stabilising property) and `s ∈ [0, 1]` is a runtime-tunable scale so the
/// feedback can be smoothly faded in. `Δx` is the state-tangent difference
/// (`ndx = 2·nv` columns of `K`), obtained one of two ways:
/// - @ref Compute forms the Euclidean @f$ \Delta x = [q_{curr} - q_{ref};
///   v_{curr} - v_{ref}] @f$ itself — valid only when every joint has
///   `nq == nv` (then `nq + nv == ndx`); otherwise it writes zeros.
/// - @ref ComputeFromDelta takes a ready `Δx`, e.g. from
///   `WbLayout::StateDifference` (tangent difference on a free-flyer base).
///
/// Which rows of `u_fb` are written:
/// - `accel_only = true` (default): only the joint-acceleration rows of the
///   gain output, `[acc_row_offset, acc_row_offset + n_acc)`, copied to the
///   front of `u_fb_out`. @ref Init sets the fixed-base torque default (row
///   0, `max_nv` rows); a kinodynamic layout sets them with @ref SetAccelRows
///   so the wrench rows are never mistaken for accelerations.
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
  /// @param max_nv   velocity dim upper bound (@ref Compute accepts any
  ///                 `nv ≤ max_nv`; also the default accel row count)
  /// @param max_nu   control-input dim upper bound (gain rows)
  /// @param max_ndx  state-tangent dim upper bound (gain columns)
  void Init(int max_nv, int max_nu, int max_ndx);

  /// @brief Install a new Riccati gain matrix.
  /// @param K_data  pointer to row-major `nu × ndx` data, read as
  ///                `K_data[row * ndx + col]` — the row stride is the actual
  ///                `ndx`, the layout @ref PackRiccatiGainRowMajor writes
  /// @param nu      actual control dim (must satisfy `nu ≤ max_nu`)
  /// @param ndx     actual state-tangent dim (must satisfy `ndx ≤ max_ndx`)
  /// @note No-op (gain cleared) if bounds are violated or the data is not
  ///       finite — a NaN in `K` would poison every output even at zero error.
  void SetGain(const double* K_data, int nu, int ndx) noexcept;

  /// @brief Compute @f$ u_{fb} = s \cdot K \cdot \Delta x @f$ from the
  ///        Euclidean difference of configurations and velocities.
  ///
  /// @param q_curr   current joint position (size nq)
  /// @param v_curr   current joint velocity (size nv)
  /// @param q_ref    reference joint position (size nq)
  /// @param v_ref    reference joint velocity (size nv)
  /// @param u_fb_out output buffer; the accel rows are written in accel-only
  ///                 mode, `nu` entries otherwise; the rest is zeroed
  ///
  /// Writes zeros when `nq + nv != ndx` (a configuration with more
  /// coordinates than tangent entries has no Euclidean `Δq`; use
  /// @ref ComputeFromDelta). On @f$ \|\Delta x\| > @f$ `max_delta_x_norm_`,
  /// `Δx` is scaled so the effective @f$ u_{fb} @f$ magnitude stays bounded.
  void Compute(const Eigen::Ref<const Eigen::VectorXd>& q_curr,
               const Eigen::Ref<const Eigen::VectorXd>& v_curr,
               const Eigen::Ref<const Eigen::VectorXd>& q_ref,
               const Eigen::Ref<const Eigen::VectorXd>& v_ref,
               Eigen::Ref<Eigen::VectorXd> u_fb_out) noexcept;

  /// @brief Compute @f$ u_{fb} = s \cdot K \cdot \Delta x @f$ from a ready
  ///        state-tangent difference (size `ndx`, as installed by
  ///        @ref SetGain). Same scaling, cap and row selection as
  ///        @ref Compute.
  /// @return false (zeros written) when there is no gain, `dx` has the wrong
  ///         size, or `dx` is not finite.
  [[nodiscard]] bool ComputeFromDelta(const Eigen::Ref<const Eigen::VectorXd>& dx,
                                      Eigen::Ref<Eigen::VectorXd> u_fb_out) noexcept;

  /// @brief Which gain-output rows are the joint accelerations (accel-only
  ///        mode copies exactly these to the front of `u_fb_out`).
  /// @param offset  first acceleration row of `u`
  /// @param n       number of acceleration rows
  /// @return false (unchanged) when the block does not fit `max_nu`.
  [[nodiscard]] bool SetAccelRows(int offset, int n) noexcept;

  [[nodiscard]] int AccelRowOffset() const noexcept { return acc_row_offset_; }

  [[nodiscard]] int AccelRows() const noexcept { return acc_rows_; }

  /// @brief Runtime gain scale, clamped to `[0, 1]`; a non-finite value is ignored.
  void SetGainScale(double scale) noexcept;

  [[nodiscard]] double GainScale() const noexcept { return gain_scale_; }

  /// @brief Toggle "acceleration-only" mode (see class docs).
  void SetAccelOnly(bool accel_only) noexcept { accel_only_ = accel_only; }

  [[nodiscard]] bool AccelOnly() const noexcept { return accel_only_; }

  /// @brief Cap on @f$ \|\Delta x\| @f$ before feedback magnitude is
  ///        scaled down. Must be positive.
  void SetMaxDeltaXNorm(double value) noexcept;

  [[nodiscard]] double MaxDeltaXNorm() const noexcept { return max_delta_x_norm_; }

  [[nodiscard]] bool HasGain() const noexcept { return nu_ > 0 && ndx_ > 0; }

 private:
  /// Shared tail of both Compute paths: cap, scale, multiply, select rows.
  /// Zeros the output and returns false when `dx_` is not finite.
  bool Apply(Eigen::Ref<Eigen::VectorXd> u_fb_out) noexcept;

  Eigen::MatrixXd K_;         // nu × ndx (allocated max_nu × max_ndx)
  Eigen::VectorXd dx_;        // ndx    (allocated max_ndx)
  Eigen::VectorXd u_fb_raw_;  // nu     (allocated max_nu)

  int max_nv_{0};
  int max_nu_{0};
  int max_ndx_{0};
  int nu_{0};   ///< Actual gain rows after SetGain (0 == uninitialised)
  int ndx_{0};  ///< Actual gain columns (state-tangent dim)

  int acc_row_offset_{0};
  int acc_rows_{0};  ///< set by Init (fixed-base default) or SetAccelRows

  double gain_scale_{1.0};
  bool accel_only_{true};
  double max_delta_x_norm_{100.0};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_FEEDBACK_RICCATI_FEEDBACK_HPP_
