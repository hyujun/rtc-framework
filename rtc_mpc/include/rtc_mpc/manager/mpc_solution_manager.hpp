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
///
/// Eviction: the plan being sampled is dropped — the consumer falls back
/// until the next accepted plan — at the first @ref ComputeReference that
/// finds (1) its `model_epoch` or `state_epoch` no longer the current one,
/// (2) its age past the horizon by more than one node
/// (`now − observation_stamp_ns > (horizon_length + 1) · dt_node`), or (3)
/// that @ref SetEnabled re-enabled the manager since the previous call.

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_mpc/comm/triple_buffer.hpp"
#include "rtc_mpc/feedback/riccati_feedback.hpp"
#include "rtc_mpc/interpolation/trajectory_interpolator.hpp"
#include "rtc_mpc/model/wb_layout.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <array>
#include <atomic>
#include <cstddef>
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
  /// @param cfg     YAML node with `mpc:` keys:
  ///                `enabled`, `max_stale_solutions`,
  ///                `riccati.{enabled, gain_scale, accel_only,
  ///                 max_delta_x_norm}`.
  /// @param layout  state / input layout of the model the consumer runs
  ///                (`WbLayout::FromModel` or `FromHandler`). A floating
  ///                layout is accepted but @ref ComputeReference refuses to
  ///                produce a reference for it (`InterpMeta::floating_unsupported`)
  ///                until a manifold-aware sampler replaces the Euclidean
  ///                interpolator; the fixed-base path is unchanged. A layout
  ///                that is not `valid()` (a failed factory) leaves the
  ///                manager disabled.
  void Init(const YAML::Node& cfg, const WbLayout& layout);

  /// @brief Fixed-base convenience: `Init(cfg, WbLayout::FixedBase(nq, nv, n_contact_vars))`.
  void Init(const YAML::Node& cfg, int nq, int nv, int n_contact_vars) {
    Init(cfg, WbLayout::FixedBase(nq, nv, n_contact_vars));
  }

  [[nodiscard]] const WbLayout& Layout() const noexcept { return layout_; }

  /// @brief Runtime toggle for MPC consumption. When false,
  ///        @ref ComputeReference reports invalid immediately; a switch from
  ///        false to true drops the plan held from before, so a re-enabled
  ///        consumer never samples it. Any single thread (parameter
  ///        callbacks); the RT tick reads it.
  void SetEnabled(bool enabled) noexcept {
    if (enabled && !enabled_.load(std::memory_order_relaxed)) {
      // Ordered before the flag: a consumer that sees `true` from this call
      // sees the new generation too.
      enable_generation_.fetch_add(1, std::memory_order_release);
    }
    enabled_.store(enabled, std::memory_order_release);
  }

  [[nodiscard]] bool Enabled() const noexcept { return enabled_.load(std::memory_order_acquire); }

  /// @brief Runtime tuning of the Riccati scale (0..1).
  void SetRiccatiGainScale(double scale) noexcept { riccati_.SetGainScale(scale); }

  [[nodiscard]] double RiccatiGainScale() const noexcept { return riccati_.GainScale(); }

  // ── Producer (MPC thread) ──────────────────────────────────────────

  /// @brief Publish a freshly computed solution. The published copy gets a
  ///        monotonic `plan_id`; @p sol itself is left untouched.
  /// @note Called on the MPC thread. Non-blocking.
  void PublishSolution(const MPCSolution& sol) noexcept;

  /// @brief Read the latest RT-thread state snapshot.
  /// @note Called on the MPC thread at the top of a solve. Non-blocking.
  [[nodiscard]] MPCStateSnapshot ReadState() const noexcept;

  // ── Consumer (RT thread) ───────────────────────────────────────────

  /// @brief Publish the current RT-thread state for the MPC thread. Each
  ///        store carries an incrementing `sequence`, the current model and
  ///        state epochs and contact mask, @p source_sequence (the caller's
  ///        own sequence of this observation, carried back in the plan as
  ///        `observation_source_sequence`; 0 when it has none) and
  ///        `valid = true`.
  /// @note Called on the RT thread. Wait-free.
  void WriteState(const Eigen::Ref<const Eigen::VectorXd>& q,
                  const Eigen::Ref<const Eigen::VectorXd>& v, uint64_t timestamp_ns,
                  uint64_t source_sequence = 0) noexcept;

  /// @brief Contact mask stamped into the next @ref WriteState (one bit per
  ///        contact frame, model-handler order). Same thread as WriteState.
  void SetContactMask(uint32_t mask) noexcept { contact_mask_ = mask; }

  /// @brief Model generation the RT side runs. Stamped into every state
  ///        snapshot, carried back in the plan, and **enforced** in
  ///        @ref ComputeReference: a plan whose epoch differs from the
  ///        current one is not installed (counted in @ref EpochRejections,
  ///        treated like a missing solution). Any thread; release / acquire
  ///        so whatever the caller changed before bumping the epoch is
  ///        visible to the RT side that observes the new value.
  void SetModelEpoch(uint32_t epoch) noexcept {
    model_epoch_.store(epoch, std::memory_order_release);
  }

  [[nodiscard]] uint32_t ModelEpoch() const noexcept {
    return model_epoch_.load(std::memory_order_acquire);
  }

  /// @brief State-stream generation: the consumer bumps it when the state it
  ///        writes stops continuing the earlier one (a reset, a change of
  ///        state source, a time base that went back). Stamped into every
  ///        state snapshot and carried back in the plan; a plan of another
  ///        state epoch is neither installed nor kept (@ref
  ///        StateEpochRejections), and the MPC side starts its next solve
  ///        cold. Any thread, release / acquire like @ref SetModelEpoch.
  void SetStateEpoch(uint32_t epoch) noexcept {
    state_epoch_.store(epoch, std::memory_order_release);
  }

  [[nodiscard]] uint32_t StateEpoch() const noexcept {
    return state_epoch_.load(std::memory_order_acquire);
  }

  /// @return plans refused or dropped by @ref ComputeReference because their
  ///         `model_epoch` was not the current one. Any thread.
  [[nodiscard]] uint64_t EpochRejections() const noexcept {
    return epoch_rejections_.load(std::memory_order_relaxed);
  }

  /// @return plans refused or dropped by @ref ComputeReference because their
  ///         `state_epoch` was not the current one. Any thread.
  [[nodiscard]] uint64_t StateEpochRejections() const noexcept {
    return state_epoch_rejections_.load(std::memory_order_relaxed);
  }

  /// @return plans dropped by @ref ComputeReference because their age passed
  ///         the horizon by more than one node. Any thread.
  [[nodiscard]] uint64_t PolicyAgeRejections() const noexcept {
    return policy_age_rejections_.load(std::memory_order_relaxed);
  }

  /// @return plans refused by @ref ComputeReference because their `nq` / `nv`
  ///         / `ndx` / `nu` disagree with the layout given to @ref Init.
  ///         Any thread.
  [[nodiscard]] uint64_t DimensionRejections() const noexcept {
    return dimension_rejections_.load(std::memory_order_relaxed);
  }

  /// @brief Try to compute the MPC-derived reference for the current tick.
  ///
  /// On success writes q_ref / v_ref / a_ff / lambda_ref / u_fb and returns
  /// true. On fallback (MPC disabled, floating-base layout, no solution yet,
  /// the plan evicted — see the file comment — or too many stale cycles)
  /// returns false and sets @p meta_out.valid = false.
  ///
  /// @param q_curr     current position (size nq)
  /// @param v_curr     current velocity (size nv)
  /// @param now_ns     current time, in the clock given to @ref WriteState
  /// @param q_ref      out: position reference (size nq)
  /// @param v_ref      out: velocity reference (size nv)
  /// @param a_ff       out: feedforward acceleration (size nv)
  /// @param lambda_ref out: contact-force reference (size n_contact_vars)
  /// @param u_fb       out: Riccati feedback (size nv in accel_only mode)
  /// @param meta_out   out: interpolation status
  [[nodiscard]] bool ComputeReference(const Eigen::Ref<const Eigen::VectorXd>& q_curr,
                                      const Eigen::Ref<const Eigen::VectorXd>& v_curr,
                                      uint64_t now_ns, Eigen::Ref<Eigen::VectorXd> q_ref,
                                      Eigen::Ref<Eigen::VectorXd> v_ref,
                                      Eigen::Ref<Eigen::VectorXd> a_ff,
                                      Eigen::Ref<Eigen::VectorXd> lambda_ref,
                                      Eigen::Ref<Eigen::VectorXd> u_fb,
                                      InterpMeta& meta_out) noexcept;

  /// @brief The latest published solution not yet taken, as published (no
  ///        epoch, dimension or validity check, no interpolation), or null
  ///        when none arrived since the last call. Valid until a later call
  ///        returns another.
  ///        For a consumer that samples the plan itself — on a floating-base
  ///        layout, which @ref ComputeReference refuses. Consumer thread; the
  ///        buffer has one consumer, so a caller of this does not also call
  ///        @ref ComputeReference. It keeps none of ComputeReference's state:
  ///        not @ref StaleCount, not the active plan's provenance — the
  ///        caller judges the plan's age itself (its `observation_stamp_ns`
  ///        against its own clock).
  [[nodiscard]] const MPCSolution* TryAcquireLatestSolution() noexcept {
    return solution_buffer_.TryAcquireLatest();
  }

  /// @return number of consecutive RT ticks without a fresh solution.
  [[nodiscard]] int StaleCount() const noexcept { return stale_count_; }

  /// @return the max_stale_solutions threshold loaded from YAML.
  [[nodiscard]] int MaxStaleSolutions() const noexcept { return max_stale_solutions_; }

  /// @return true if a valid solution has ever been installed.
  [[nodiscard]] bool HasEverReceivedSolution() const noexcept {
    return has_ever_received_solution_;
  }

  /// Provenance of the plan the consumer currently samples from (the last
  /// one accepted in @ref ComputeReference), read from the held solution;
  /// zero until the first acceptance, reset by @ref Init. The receive stamp
  /// is in the clock the consumer hands to @ref WriteState /
  /// @ref ComputeReference — the same clock as the plan's observation stamp,
  /// so their difference is the policy age. Consumer thread only.
  [[nodiscard]] uint64_t ActivePlanId() const noexcept {
    const MPCSolution* s = interpolator_.Solution();
    return s != nullptr ? s->plan_id : 0;
  }

  [[nodiscard]] uint64_t ActiveReceiveStampNs() const noexcept { return active_receive_stamp_ns_; }

  [[nodiscard]] uint64_t ActiveObservationStampNs() const noexcept {
    const MPCSolution* s = interpolator_.Solution();
    return s != nullptr ? s->observation_stamp_ns : 0;
  }

  [[nodiscard]] uint32_t ActiveModelEpoch() const noexcept {
    const MPCSolution* s = interpolator_.Solution();
    return s != nullptr ? s->model_epoch : 0;
  }

  [[nodiscard]] uint32_t ActiveStateEpoch() const noexcept {
    const MPCSolution* s = interpolator_.Solution();
    return s != nullptr ? s->state_epoch : 0;
  }

  // ── Solve-timing probe (consumer-side perf monitor) ────────────────────
  //
  // Each call to @ref PublishSolution appends the incoming solution's
  // `solve_duration_ns` to a bounded ring buffer (@ref kSolveStatsWindow
  // samples). Callers poll @ref GetSolveStats off-RT to get percentile
  // statistics over the current window — the ring is overwritten in
  // round-robin order once full. The MPC thread is an RT context
  // (architecture.md §Execution Contexts), so the producer side is
  // lock-free: it updates a thread-private working copy and publishes it
  // through a SeqLock; readers never block it.
  //
  // Scope: perf-monitoring aid for the Phase 7c exit metrics
  // (KinoDynamics p50<10ms, p99<20ms; FullDynamics p50<25ms, p99<45ms).
  // p50/p99 here is a sliding-window approximation; good enough for
  // dashboards but not a statistical guarantee.

  /// Fixed-size sample window. Picked at 256 so a 20 Hz solve loop keeps
  /// ~12 s of history — comfortably larger than any single-phase dwell
  /// time in the Phase 7 grasp FSM.
  static constexpr std::size_t kSolveStatsWindow = 256;

  struct SolveTimingStats {
    std::uint64_t count{0};    ///< total solves observed (monotonic)
    std::uint32_t window{0};   ///< samples actually averaged (≤ kSolveStatsWindow)
    std::uint64_t last_ns{0};  ///< most recent sample
    std::uint64_t min_ns{0};   ///< min over the window (0 if window==0)
    std::uint64_t max_ns{0};   ///< max over the window
    std::uint64_t p50_ns{0};   ///< window median
    std::uint64_t p99_ns{0};   ///< window 99-th percentile
    double mean_ns{0.0};       ///< window arithmetic mean
  };

  /// @brief Snapshot the current solve-timing window and compute stats.
  /// Non-RT (sorts a copy of the window); lock-free with respect to the
  /// producer — a SeqLock load that retries only while a publish is in flight.
  [[nodiscard]] SolveTimingStats GetSolveStats() const noexcept;

  /// @brief Reset the ring buffer + total-solve counter. Non-RT.
  /// @pre No @ref PublishSolution is in flight (MPC thread paused or not yet
  ///      started): the reset writes the producer's working copy, and the
  ///      SeqLock admits a single writer.
  void ResetSolveStats() noexcept;

  // Per-tick raw-sample stream lives on the MPCThread (producer thread)
  // — see rtc_mpc/thread/mpc_thread.hpp::TimingProducer(). Drained by a
  // non-RT consumer into <session>/timing/mpc_timing_log.csv
  // via rtc::ThreadTimingCsvLogger<rtc::RtTickTimingPayload>.

 private:
  std::atomic<bool> enabled_{false};
  std::atomic<uint64_t> enable_generation_{0};  // SetEnabled false → true transitions
  uint64_t seen_enable_generation_{0};          // consumer thread only
  bool riccati_enabled_{true};
  int max_stale_solutions_{5};
  int stale_count_{0};
  bool has_ever_received_solution_{false};

  WbLayout layout_{};

  uint64_t state_sequence_{0};  // RT thread only (WriteState)
  uint32_t contact_mask_{0};    // RT thread only (SetContactMask / WriteState)
  std::atomic<uint32_t> model_epoch_{0};
  std::atomic<uint32_t> state_epoch_{0};
  uint64_t plan_counter_{0};  // MPC thread only (PublishSolution)

  // Consumer-side provenance not carried by the plan itself (consumer thread only).
  uint64_t active_receive_stamp_ns_{0};
  // Written by the consumer thread only (load + 1, store); read from any.
  std::atomic<uint64_t> epoch_rejections_{0};
  std::atomic<uint64_t> state_epoch_rejections_{0};
  std::atomic<uint64_t> policy_age_rejections_{0};
  std::atomic<uint64_t> dimension_rejections_{0};

  /// Drop the active plan when it is no longer the current epochs' or has
  /// outlived its horizon (see the file comment). Consumer thread.
  void EvictIfExpired(uint64_t now_ns, uint32_t model_epoch, uint32_t state_epoch) noexcept;

  TripleBuffer<MPCSolution> solution_buffer_;
  TrajectoryInterpolator interpolator_;
  RiccatiFeedback riccati_;
  rtc::SeqLock<MPCStateSnapshot> state_lock_;

  // Solve-timing probe state. `PublishSolution` (single producer, MPC
  // thread) updates `solve_stats_work_` and stores it into
  // `solve_stats_lock_`; `GetSolveStats` (non-RT) loads a copy. No mutex:
  // a FIFO MPC thread blocking on a lock held by the SCHED_OTHER stats
  // reader was a priority inversion (RT-4).
  struct SolveStatsRing {
    std::array<std::uint64_t, kSolveStatsWindow> ring{};
    std::uint32_t next{0};    // next write slot, wraps at kSolveStatsWindow
    std::uint32_t filled{0};  // samples in ring (≤ kSolveStatsWindow)
    std::uint64_t total{0};   // lifetime solve count
    std::uint64_t last{0};    // most recent sample
  };

  SolveStatsRing solve_stats_work_{};  // producer-private working copy
  rtc::SeqLock<SolveStatsRing> solve_stats_lock_;
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_MANAGER_MPC_SOLUTION_MANAGER_HPP_
