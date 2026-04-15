#ifndef RTC_MPC_MANAGER_MPC_SOLUTION_MANAGER_HPP_
#define RTC_MPC_MANAGER_MPC_SOLUTION_MANAGER_HPP_

/// @file mpc_solution_manager.hpp
/// @brief Facade combining TripleBuffer + Interpolator + RiccatiFeedback +
///        SeqLock into a single MPC-RT interface.
///
/// Ownership:
/// - Producer side (MPC thread): @ref PublishSolution, @ref ReadState.
/// - Consumer side (RT thread): @ref WriteState, @ref ComputeReference.
///
/// Staleness policy: every call to @ref ComputeReference increments
/// `stale_count_` if no new solution has arrived since the previous call.
/// Once `stale_count_` reaches @ref max_stale_solutions_, the manager
/// reports "not valid" via @ref InterpMeta and the downstream controller
/// should fall back to a safe reference (e.g. the Phase 4 fixed pose).
/// A fresh @ref PublishSolution resets the counter.

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_mpc/comm/triple_buffer.hpp"
#include "rtc_mpc/feedback/riccati_feedback.hpp"
#include "rtc_mpc/interpolation/trajectory_interpolator.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <cstdint>

namespace rtc::mpc {

class MPCSolutionManager {
 public:
  MPCSolutionManager() = default;
  ~MPCSolutionManager() = default;

  MPCSolutionManager(const MPCSolutionManager&) = delete;
  MPCSolutionManager& operator=(const MPCSolutionManager&) = delete;
  MPCSolutionManager(MPCSolutionManager&&) = delete;
  MPCSolutionManager& operator=(MPCSolutionManager&&) = delete;

  /// @brief Configure the manager and allocate all buffers.
  /// @param cfg             YAML node with `mpc:` keys:
  ///                        `enabled`, `max_stale_solutions`,
  ///                        `riccati.{enabled, gain_scale, accel_only,
  ///                         max_delta_x_norm}`.
  /// @param nq              joint-position dim
  /// @param nv              joint-velocity dim
  /// @param n_contact_vars  total contact-force dim
  void Init(const YAML::Node& cfg, int nq, int nv, int n_contact_vars);

  /// @brief Runtime toggle for MPC consumption. When false,
  ///        @ref ComputeReference reports invalid immediately.
  void SetEnabled(bool enabled) noexcept { enabled_ = enabled; }
  [[nodiscard]] bool Enabled() const noexcept { return enabled_; }

  /// @brief Runtime tuning of the Riccati scale (0..1).
  void SetRiccatiGainScale(double scale) noexcept {
    riccati_.SetGainScale(scale);
  }
  [[nodiscard]] double RiccatiGainScale() const noexcept {
    return riccati_.GainScale();
  }

  // ── Producer (MPC thread) ──────────────────────────────────────────

  /// @brief Publish a freshly computed solution.
  /// @note Called on the MPC thread. Non-blocking.
  void PublishSolution(const MPCSolution& sol) noexcept;

  /// @brief Read the latest RT-thread state snapshot.
  /// @note Called on the MPC thread at the top of a solve. Non-blocking.
  [[nodiscard]] MPCStateSnapshot ReadState() const noexcept;

  // ── Consumer (RT thread) ───────────────────────────────────────────

  /// @brief Publish the current RT-thread state for the MPC thread.
  /// @note Called on the RT thread. Wait-free.
  void WriteState(const Eigen::Ref<const Eigen::VectorXd>& q,
                  const Eigen::Ref<const Eigen::VectorXd>& v,
                  uint64_t timestamp_ns) noexcept;

  /// @brief Try to compute the MPC-derived reference for the current tick.
  ///
  /// On success writes q_ref / v_ref / a_ff / lambda_ref / u_fb and returns
  /// true. On fallback (MPC disabled, no solution yet, or too many stale
  /// cycles) returns false and sets @p meta_out.valid = false.
  ///
  /// @param q_curr     current position (size nq)
  /// @param v_curr     current velocity (size nv)
  /// @param now_ns     current wall-clock time
  /// @param q_ref      out: position reference (size nq)
  /// @param v_ref      out: velocity reference (size nv)
  /// @param a_ff       out: feedforward acceleration (size nv)
  /// @param lambda_ref out: contact-force reference (size n_contact_vars)
  /// @param u_fb       out: Riccati feedback (size nv in accel_only mode)
  /// @param meta_out   out: interpolation status
  [[nodiscard]] bool ComputeReference(
      const Eigen::Ref<const Eigen::VectorXd>& q_curr,
      const Eigen::Ref<const Eigen::VectorXd>& v_curr,
      uint64_t now_ns,
      Eigen::Ref<Eigen::VectorXd> q_ref,
      Eigen::Ref<Eigen::VectorXd> v_ref,
      Eigen::Ref<Eigen::VectorXd> a_ff,
      Eigen::Ref<Eigen::VectorXd> lambda_ref,
      Eigen::Ref<Eigen::VectorXd> u_fb,
      InterpMeta& meta_out) noexcept;

  /// @return number of consecutive RT ticks without a fresh solution.
  [[nodiscard]] int StaleCount() const noexcept { return stale_count_; }

  /// @return the max_stale_solutions threshold loaded from YAML.
  [[nodiscard]] int MaxStaleSolutions() const noexcept {
    return max_stale_solutions_;
  }

  /// @return true if a valid solution has ever been installed.
  [[nodiscard]] bool HasEverReceivedSolution() const noexcept {
    return has_ever_received_solution_;
  }

 private:
  bool enabled_{false};
  bool riccati_enabled_{true};
  int max_stale_solutions_{5};
  int stale_count_{0};
  bool has_ever_received_solution_{false};

  int nq_{0};
  int nv_{0};
  int n_contact_vars_{0};

  TripleBuffer<MPCSolution> solution_buffer_;
  TrajectoryInterpolator interpolator_;
  RiccatiFeedback riccati_;
  rtc::SeqLock<MPCStateSnapshot> state_lock_;
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_MANAGER_MPC_SOLUTION_MANAGER_HPP_
