#ifndef RTC_MPC_MANAGER_MPC_SOLUTION_MANAGER_HPP_
#define RTC_MPC_MANAGER_MPC_SOLUTION_MANAGER_HPP_

/// @file mpc_solution_manager.hpp
/// @brief Facade combining TripleBuffer + Interpolator + RiccatiFeedback +
///        SeqLock into a single MPC-RT interface.
///
/// Ownership:
/// - Producer side (MPC thread): @ref PublishSolution, @ref ReadState.
/// - Consumer side (RT thread): @ref WriteState, then @ref SampleReference
///   (or @ref ComputeReference) — both sample the one plan the manager
///   holds, through one kernel (`TrajectoryInterpolator::Sample`), on a fixed
///   or a floating base.
///
/// Time: a plan's node 0 sits at `observation_stamp_ns − horizon_offset_ns`
/// in the clock given to @ref WriteState (the state it started from, moved
/// back onto the node grid), not at the time the consumer received it.
/// Policy age is `receive − observation` (@ref ActivePolicyAgeNs).
///
/// Riccati feedback: the gain of the node nearest to the sample time,
/// `min(round(τ / dt_node), gain_nodes − 1)` (a clamp is counted,
/// @ref GainNodeClamps), read in place from the held plan. `Δx` is
/// `WbLayout::StateDifference` on a floating base (needs the model given to
/// @ref Init — without one the feedback is 0), the Euclidean difference on a
/// fixed one.
///
/// Staleness policy: every sample increments `stale_count_` if no new
/// solution has arrived since the previous one. Once it reaches
/// @ref max_stale_solutions_, @ref ComputeReference reports "not valid" and
/// the downstream controller should fall back to a safe reference (e.g. the
/// Phase 4 fixed pose); @ref SampleReference judges staleness by age instead
/// (`max_policy_age_s`, below). A fresh @ref PublishSolution resets it.
///
/// Eviction: the plan being sampled is dropped — the consumer falls back
/// until the next accepted plan — at the first @ref ComputeReference that
/// finds (1) its `model_epoch` or `state_epoch` no longer the current one,
/// (2) its age past `max_policy_age_s` (when positive) or past the horizon
/// by more than one node either way
/// (`|now − observation_stamp_ns| > (horizon_length + 1) · dt_node`; a
/// stamp that far ahead means the consumer's clock went back), or (3) that
/// @ref SetEnabled re-enabled the manager since the previous call. After a
/// re-enable, a plan whose observation predates the first @ref WriteState
/// after it is refused (counted with the state-epoch rejections: another
/// state stream) — whether it was published before or after the re-enable.
///
/// Threads: @ref Init runs before the producer and consumer start.

#include "rtc_base/threading/seqlock.hpp"
#include "rtc_mpc/comm/triple_buffer.hpp"
#include "rtc_mpc/feedback/riccati_feedback.hpp"
#include "rtc_mpc/interpolation/trajectory_interpolator.hpp"
#include "rtc_mpc/model/wb_layout.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"
#include "rtc_mpc/types/wbc_reference.hpp"

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
  ///                `enabled`, `max_stale_solutions`, `max_policy_age_s`
  ///                (≥ 0 s, 0 = off), `riccati.{enabled, gain_scale,
  ///                accel_only, max_delta_x_norm, max_delta_blocks}`.
  ///                `max_delta_blocks` maps a block name (`base_position`,
  ///                `base_orientation`, `joint_position`,
  ///                `base_linear_velocity`, `base_angular_velocity`,
  ///                `joint_velocity`) to its own cap; a floating base caps
  ///                every block (default `max_delta_x_norm` each), a fixed
  ///                base only when the map is given (joint blocks only).
  ///                A plan whose `gain_nodes`, `nu` or `ndx` exceed the
  ///                capacities is refused as a dimension mismatch.
  /// @param layout  state / input layout of the model the consumer runs
  ///                (`WbLayout::FromModel` or `FromHandler`). A layout that
  ///                is not `valid()` (a failed factory) leaves the manager
  ///                disabled.
  /// @param model   the model of @p layout, for the floating-base Riccati
  ///                `Δx` — non-owning, read on the RT tick, so it must stay
  ///                where it is and unchanged until the next Init; optional.
  /// @note A value out of range — a negative or non-finite age, a cap that
  ///       is not positive and finite, an unknown block name, a base block
  ///       on a fixed base — leaves the manager disabled (@ref ConfigValid).
  void Init(const YAML::Node& cfg, const WbLayout& layout, const pinocchio::Model* model = nullptr);

  /// @return false when the last @ref Init refused its YAML.
  [[nodiscard]] bool ConfigValid() const noexcept { return config_valid_; }

  /// @brief Fixed-base convenience: `Init(cfg, WbLayout::FixedBase(nq, nv, n_contact_vars))`.
  void Init(const YAML::Node& cfg, int nq, int nv, int n_contact_vars) {
    Init(cfg, WbLayout::FixedBase(nq, nv, n_contact_vars));
  }

  [[nodiscard]] const WbLayout& Layout() const noexcept { return layout_; }

  /// @brief Runtime toggle for MPC consumption. When false,
  ///        @ref ComputeReference reports invalid immediately; after a switch
  ///        back to true the consumer samples no plan computed from a state
  ///        written before it (see the file comment). `true` is ignored
  ///        after an @ref Init that refused its layout or YAML. One setter
  ///        thread at a time (parameter or lifecycle callbacks); the RT tick
  ///        reads it.
  ///        Two setters racing can merge a short disable into none — the
  ///        consumer then never stopped, and the age rules still apply.
  void SetEnabled(bool enabled) noexcept {
    if (enabled && (!config_valid_ || !layout_.valid())) {
      return;  // a refused configuration stays off until a valid Init
    }
    if (enabled && !enabled_.load(std::memory_order_acquire)) {
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
  ///        cold. Epochs only grow: a value used again would let a plan of
  ///        the earlier stream back in. Call it on the consumer thread,
  ///        between the last @ref WriteState of the old stream and the first
  ///        of the new one — from another thread, a state written around the
  ///        bump can carry either epoch. Release / acquire like
  ///        @ref SetModelEpoch.
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

  /// @brief Sample the held plan for this tick into @p out (see the file
  ///        comment), taking a newly published plan first.
  ///
  /// `u_fb_acc` holds the joint-acceleration rows of the Riccati feedback
  /// at their tangent entries (the base rows 0); `receive_stamp_ns` is the
  /// @p now_ns the plan was taken at. @p out must not alias @p q_curr or
  /// @p v_curr (it is cleared first).
  /// @return true with `out.valid`; false (`out.valid` false, the rest of
  ///         @p out zero) when disabled, without a plan, with the plan
  ///         evicted or with a sample that is not finite — not on a
  ///         stale-tick count.
  /// @note Consumer thread. Allocation-free.
  [[nodiscard]] bool SampleReference(const Eigen::Ref<const Eigen::VectorXd>& q_curr,
                                     const Eigen::Ref<const Eigen::VectorXd>& v_curr,
                                     uint64_t now_ns, WbcReference& out) noexcept;

  /// @brief Try to compute the MPC-derived reference for the current tick.
  ///
  /// On success writes q_ref / v_ref / a_ff / lambda_ref / u_fb and returns
  /// true. On fallback (MPC disabled, no solution yet, the plan evicted —
  /// see the file comment — or too many stale cycles) returns false and sets
  /// @p meta_out.valid = false. The same sample as @ref SampleReference;
  /// in accel-only mode `u_fb` gets the joint-acceleration rows at their
  /// tangent entries (the front on a fixed base), otherwise the input-sized
  /// feedback at its front.
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

  /// @return the held plan's policy age, receive − observation stamp [ns]
  ///         (0 without one).
  [[nodiscard]] int64_t ActivePolicyAgeNs() const noexcept {
    const MPCSolution* s = interpolator_.Solution();
    return s != nullptr ? static_cast<int64_t>(active_receive_stamp_ns_) -
                              static_cast<int64_t>(s->observation_stamp_ns)
                        : 0;
  }

  /// @return samples whose nearest gain node was past the plan's last
  ///         shipped gain node (the last one was used). Any thread.
  [[nodiscard]] uint64_t GainNodeClamps() const noexcept {
    return gain_node_clamps_.load(std::memory_order_relaxed);
  }

  /// @return samples with a gain but no feedback: a floating base without
  ///         the model given to @ref Init, a state `WbLayout::StateDifference`
  ///         refused (a non-unit quaternion on either side), or a non-finite
  ///         state. The reference is still valid. Any thread.
  [[nodiscard]] uint64_t FeedbackSkips() const noexcept {
    return feedback_skips_.load(std::memory_order_relaxed);
  }

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
  uint64_t seen_enable_generation_{0};          // consumer thread only (ComputeReference)
  uint64_t written_enable_generation_{0};       // consumer thread only (WriteState)
  uint64_t first_sequence_after_enable_{0};     // plans observed before it are refused
  bool riccati_enabled_{true};
  bool gain_installed_{false};  // the active plan's gain is in riccati_ (consumer thread)
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
  std::atomic<uint64_t> gain_node_clamps_{0};
  std::atomic<uint64_t> feedback_skips_{0};

  const pinocchio::Model* model_{nullptr};  // Init's; floating-base Riccati Δx
  int64_t max_policy_age_ns_{0};            // 0 = off
  bool config_valid_{true};
  Eigen::VectorXd dx_{};    // ndx, floating-base Riccati Δx
  Eigen::VectorXd u_fb_{};  // max(nu, nv), the feedback rows the tick wrote
  WbcReference sample_{};   // ComputeReference's sample (SampleReference writes the caller's)

  /// The tick both public paths share: take a fresh plan, evict, sample into
  /// @p out, compute the feedback into `u_fb_`. @p count_stale applies the
  /// stale-tick rule. Consumer thread.
  bool Tick(const Eigen::Ref<const Eigen::VectorXd>& q_curr,
            const Eigen::Ref<const Eigen::VectorXd>& v_curr, uint64_t now_ns, bool count_stale,
            WbcReference& out) noexcept;

  /// Drop the active plan when it is no longer the current epochs' or has
  /// outlived its horizon (see the file comment). Consumer thread.
  void EvictIfExpired(uint64_t now_ns, uint32_t model_epoch, uint32_t state_epoch) noexcept;

  /// Forget the active plan and its receive stamp (the next accepted plan
  /// sets `gain_installed_`). Consumer thread.
  void DropActivePlan() noexcept;

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
