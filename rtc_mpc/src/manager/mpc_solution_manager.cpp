#include "rtc_mpc/manager/mpc_solution_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

namespace rtc::mpc {

namespace {

// Single-writer counter: a plain load + store, visible to readers on any thread.
void Bump(std::atomic<uint64_t>& c) noexcept {
  c.store(c.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
}

/// Every gain node the plan ships is finite (checked once per plan).
bool GainsFinite(const MPCSolution& s) noexcept {
  const Eigen::Index n = static_cast<Eigen::Index>(s.nu) * s.ndx;
  for (int g = 0; g < s.gain_nodes; ++g) {
    if (!Eigen::Map<const Eigen::VectorXd>(s.K_riccati[static_cast<std::size_t>(g)].data(), n)
             .allFinite()) {
      return false;
    }
  }
  return true;
}

}  // namespace

void MPCSolutionManager::Init(const YAML::Node& cfg, const WbLayout& layout,
                              const pinocchio::Model* model) {
  layout_ = layout;
  model_ = model;
  config_valid_ = true;
  const int nq = layout.nq;
  const int nv = layout.nv;
  const int n_contact_vars = layout.n_contact_vars;

  // A layout the factory refused has no dimensions: nothing downstream can
  // be sized, so the manager stays off regardless of the YAML switch.
  enabled_.store(layout.valid() && cfg["enabled"] && cfg["enabled"].as<bool>(),
                 std::memory_order_relaxed);
  if (cfg["max_stale_solutions"]) {
    max_stale_solutions_ = cfg["max_stale_solutions"].as<int>();
    if (max_stale_solutions_ < 1) {
      max_stale_solutions_ = 1;
    }
  }

  interpolator_.Init(nq, nv, n_contact_vars);

  // Riccati block
  const YAML::Node r = cfg["riccati"];
  riccati_enabled_ = !r || !r["enabled"] || r["enabled"].as<bool>();

  // Workspace sized by the layout: the gain has `nu` rows (more than nv on a
  // kinodynamic input) and `ndx` columns. A plan whose dimensions disagree
  // with the layout is refused at acceptance rather than silently dropped.
  riccati_.Init(nv, std::max(layout.nu, 1), std::max(layout.ndx, 1));
  (void)riccati_.SetAccelRows(layout.acc_u_offset, layout.n_acc);

  if (r && r["gain_scale"]) {
    riccati_.SetGainScale(r["gain_scale"].as<double>());
  }
  const bool accel_only = !r || !r["accel_only"] || r["accel_only"].as<bool>();
  riccati_.SetAccelOnly(accel_only);

  if (r && r["max_delta_x_norm"]) {
    riccati_.SetMaxDeltaXNorm(r["max_delta_x_norm"].as<double>());
  }

  // Per-block Δx caps: every block on a floating base, the joint blocks on a
  // fixed one when asked for.
  // (A default-constructed YAML::Node is a defined Null node: test the key.)
  const bool have_blocks = r && r["max_delta_blocks"];
  const YAML::Node blocks = have_blocks ? r["max_delta_blocks"] : YAML::Node{};
  if (have_blocks && !blocks.IsMap()) {
    config_valid_ = false;
  } else if (layout.valid() && (layout.floating_base() || have_blocks)) {
    RiccatiBlockCaps caps{};
    caps.fill(riccati_.MaxDeltaXNorm());
    if (have_blocks) {
      for (const auto& kv : blocks) {
        const std::string name = kv.first.as<std::string>();
        const auto* it = std::find_if(kRiccatiBlockNames.begin(), kRiccatiBlockNames.end(),
                                      [&name](const char* n) { return name == n; });
        const double cap = kv.second.as<double>();
        const auto b = static_cast<std::size_t>(it - kRiccatiBlockNames.begin());
        if (it == kRiccatiBlockNames.end() || !(cap > 0.0) || !std::isfinite(cap) ||
            (IsBaseBlock(static_cast<RiccatiBlock>(b)) && !layout.floating_base())) {
          config_valid_ = false;
        } else {
          caps[b] = cap;
        }
      }
    }
    if (config_valid_ && !riccati_.SetBlockCaps(caps, nv, layout.floating_base())) {
      config_valid_ = false;
    }
  }

  max_policy_age_ns_ = 0;
  if (cfg["max_policy_age_s"]) {
    const double age = cfg["max_policy_age_s"].as<double>();
    if (!(age >= 0.0) || !std::isfinite(age)) {
      config_valid_ = false;
    } else {
      max_policy_age_ns_ = static_cast<int64_t>(std::llround(age * 1e9));
    }
  }
  if (!config_valid_) {
    enabled_.store(false, std::memory_order_relaxed);
  }

  dx_.setZero(std::max(layout.ndx, 1));
  u_fb_.setZero(std::max({layout.nu, nv, 1}));
  sample_ = WbcReference{};
  stale_count_ = 0;
  has_ever_received_solution_ = false;
  state_sequence_ = 0;
  plan_counter_ = 0;
  active_receive_stamp_ns_ = 0;
  gain_installed_ = false;
  epoch_rejections_.store(0, std::memory_order_relaxed);
  state_epoch_rejections_.store(0, std::memory_order_relaxed);
  policy_age_rejections_.store(0, std::memory_order_relaxed);
  dimension_rejections_.store(0, std::memory_order_relaxed);
  gain_node_clamps_.store(0, std::memory_order_relaxed);
  feedback_skips_.store(0, std::memory_order_relaxed);
  seen_enable_generation_ = enable_generation_.load(std::memory_order_acquire);
  written_enable_generation_ = seen_enable_generation_;
  first_sequence_after_enable_ = 0;
}

void MPCSolutionManager::PublishSolution(const MPCSolution& sol) noexcept {
  MPCSolution& slot = solution_buffer_.StartWrite();
  std::memcpy(&slot, &sol, sizeof(MPCSolution));
  slot.plan_id = ++plan_counter_;
  solution_buffer_.FinishWrite();

  // Timing probe — record the just-published solve_duration into the
  // bounded ring. Lock-free: only this (MPC) thread writes the working copy,
  // and the SeqLock store never waits on the non-RT GetSolveStats reader.
  SolveStatsRing& w = solve_stats_work_;
  w.ring[w.next] = sol.solve_duration_ns;
  w.next = static_cast<std::uint32_t>((w.next + 1) % kSolveStatsWindow);
  if (w.filled < kSolveStatsWindow) {
    ++w.filled;
  }
  ++w.total;
  w.last = sol.solve_duration_ns;
  solve_stats_lock_.Store(w);

  // Per-tick raw-sample stream is owned by MPCThread (producer thread).
  // It pushes one rtc::RtTickTimingPayload per main-loop iteration with
  // t_state_us / t_compute_us / t_publish_us / t_total_us / jitter_us
  // captured around this PublishSolution call.
}

MPCSolutionManager::SolveTimingStats MPCSolutionManager::GetSolveStats() const noexcept {
  SolveTimingStats s{};

  // Snapshot through the SeqLock, compute on the copy.
  const SolveStatsRing loaded = solve_stats_lock_.Load();
  s.count = loaded.total;
  s.last_ns = loaded.last;
  const std::uint32_t window = loaded.filled;
  std::array<std::uint64_t, kSolveStatsWindow> snap = loaded.ring;

  s.window = window;
  if (window == 0) {
    return s;
  }

  std::sort(snap.begin(), snap.begin() + window);
  s.min_ns = snap.front();
  s.max_ns = snap[window - 1];

  // Sum for mean. Overflow unlikely (256 samples × ~1e9 ns = ~2.56e11 fits
  // well inside uint64).
  std::uint64_t sum = 0;
  for (std::size_t i = 0; i < window; ++i) {
    sum += snap[i];
  }
  s.mean_ns = static_cast<double>(sum) / static_cast<double>(window);

  // Rank-based percentile. For small windows p99 may coincide with max;
  // callers should read `window` to judge sample adequacy.
  const auto rank_for = [window](double q) -> std::size_t {
    const double r = q * static_cast<double>(window - 1);
    const std::size_t idx = static_cast<std::size_t>(r + 0.5);
    return idx < window ? idx : window - 1;
  };
  s.p50_ns = snap[rank_for(0.50)];
  s.p99_ns = snap[rank_for(0.99)];
  return s;
}

void MPCSolutionManager::ResetSolveStats() noexcept {
  solve_stats_work_ = SolveStatsRing{};
  solve_stats_lock_.Store(solve_stats_work_);
}

MPCStateSnapshot MPCSolutionManager::ReadState() const noexcept {
  return state_lock_.Load();
}

void MPCSolutionManager::WriteState(const Eigen::Ref<const Eigen::VectorXd>& q,
                                    const Eigen::Ref<const Eigen::VectorXd>& v,
                                    uint64_t timestamp_ns, uint64_t source_sequence) noexcept {
  MPCStateSnapshot snap{};
  const int nq = std::min(static_cast<int>(q.size()), static_cast<int>(kMaxNq));
  const int nv = std::min(static_cast<int>(v.size()), static_cast<int>(kMaxNv));
  for (int i = 0; i < nq; ++i) {
    snap.q[static_cast<std::size_t>(i)] = q(i);
  }
  for (int i = 0; i < nv; ++i) {
    snap.v[static_cast<std::size_t>(i)] = v(i);
  }
  snap.nq = nq;
  snap.nv = nv;
  snap.timestamp_ns = timestamp_ns;
  snap.sequence = ++state_sequence_;
  // The first state written after a re-enable fences off the plans of the
  // states before it.
  const uint64_t generation = enable_generation_.load(std::memory_order_acquire);
  if (generation != written_enable_generation_) {
    written_enable_generation_ = generation;
    first_sequence_after_enable_ = snap.sequence;
  }
  snap.source_sequence = source_sequence;
  snap.model_epoch = model_epoch_.load(std::memory_order_acquire);
  snap.state_epoch = state_epoch_.load(std::memory_order_acquire);
  snap.contact_mask = contact_mask_;
  snap.valid = true;
  state_lock_.Store(snap);
}

bool MPCSolutionManager::Tick(const Eigen::Ref<const Eigen::VectorXd>& q_curr,
                              const Eigen::Ref<const Eigen::VectorXd>& v_curr, uint64_t now_ns,
                              bool count_stale, WbcReference& out) noexcept {
  out = WbcReference{};
  if (!enabled_.load(std::memory_order_acquire)) {
    return false;
  }
  // Re-enabled since the last call: the held plan predates it (the plans
  // still to come from older states are fenced at acceptance).
  const uint64_t generation = enable_generation_.load(std::memory_order_acquire);
  if (generation != seen_enable_generation_) {
    seen_enable_generation_ = generation;
    DropActivePlan();
  }

  // Try to pick up a newly published solution. One acceptance decision: the
  // plan must be consumable (IsValid), computed against the model and from
  // the state stream the RT side runs now, and laid out like this manager's
  // layout; only an accepted plan touches the interpolator, the gain, the
  // stale counter and the provenance.
  const uint32_t model_epoch = model_epoch_.load(std::memory_order_acquire);
  const uint32_t state_epoch = state_epoch_.load(std::memory_order_acquire);
  const MPCSolution* fresh = solution_buffer_.TryAcquireLatest();
  bool accepted = false;
  if (fresh != nullptr) {
    if (fresh->model_epoch != model_epoch) {
      Bump(epoch_rejections_);
    } else if (fresh->state_epoch != state_epoch ||
               fresh->observation_sequence < first_sequence_after_enable_) {
      Bump(state_epoch_rejections_);
    } else if (fresh->nq != layout_.nq || fresh->nv != layout_.nv || fresh->ndx != layout_.ndx ||
               fresh->nu != layout_.nu || fresh->gain_nodes < 0 ||
               fresh->gain_nodes > kMaxGainNodes || fresh->nu > kMaxNu || fresh->ndx > kMaxNdx) {
      Bump(dimension_rejections_);
    } else {
      // Node 0 sits at the observation, moved back onto the node grid.
      const int64_t origin = static_cast<int64_t>(fresh->observation_stamp_ns) -
                             static_cast<int64_t>(fresh->horizon_offset_ns);
      accepted = interpolator_.SetSolutionWithOrigin(*fresh, origin);
    }
  }
  if (accepted) {
    active_receive_stamp_ns_ = now_ns;
    // The gains are read in place from the held copy; checked once here.
    const MPCSolution& held = *interpolator_.Solution();
    gain_installed_ = riccati_enabled_ && held.gain_nodes > 0 && GainsFinite(held);
    stale_count_ = 0;
    has_ever_received_solution_ = true;
  } else if (has_ever_received_solution_ && stale_count_ < max_stale_solutions_) {
    ++stale_count_;  // saturates at the threshold
  }

  EvictIfExpired(now_ns, model_epoch, state_epoch);
  if (!interpolator_.HasSolution()) {
    return false;
  }
  if (count_stale && stale_count_ >= max_stale_solutions_) {
    return false;
  }

  const SampleSpot spot = interpolator_.Sample(static_cast<int64_t>(now_ns), layout_,
                                               out.q_ref.data(), out.v_ref.data(), out.a_ref.data(),
                                               out.wrench_ref.data(), out.contact_mask);
  // The producer ships finite plans; a non-finite sample still never leaves.
  using ConstMap = Eigen::Map<const Eigen::VectorXd>;
  if (!spot.valid || !ConstMap(out.q_ref.data(), layout_.nq).allFinite() ||
      !ConstMap(out.v_ref.data(), layout_.nv).allFinite() ||
      !ConstMap(out.a_ref.data(), layout_.nv).allFinite() ||
      !ConstMap(out.wrench_ref.data(), layout_.n_contact_vars).allFinite()) {
    out = WbcReference{};
    return false;
  }
  const MPCSolution& s = *interpolator_.Solution();
  out.nq = layout_.nq;
  out.nv = layout_.nv;
  out.n_contact_vars = std::min(layout_.n_contact_vars, s.n_contact_vars);
  out.observation_stamp_ns = s.observation_stamp_ns;
  out.observation_sequence = s.observation_sequence;
  out.observation_source_sequence = s.observation_source_sequence;
  out.receive_stamp_ns = active_receive_stamp_ns_;
  out.plan_id = s.plan_id;
  out.model_epoch = s.model_epoch;
  out.state_epoch = s.state_epoch;
  out.schedule_id = s.schedule_id;
  out.schedule_epoch = s.schedule_epoch;
  out.progress = spot.progress;
  out.beyond_horizon = spot.beyond_horizon;

  // Riccati feedback: the gain node nearest the sample time, read in place.
  u_fb_.setZero();
  if (gain_installed_) {
    // gain_nodes ≤ kMaxGainNodes was checked at acceptance.
    const int64_t nearest = (spot.tau_ns + spot.dt_ns / 2) / spot.dt_ns;
    const int64_t last = s.gain_nodes - 1;
    if (nearest > last) {
      Bump(gain_node_clamps_);
    }
    const int g = static_cast<int>(std::min(nearest, last));
    riccati_.UseGain(s.K_riccati[static_cast<std::size_t>(g)].data(), s.nu, s.ndx);
    const Eigen::Map<const Eigen::VectorXd> q_ref(out.q_ref.data(), layout_.nq);
    const Eigen::Map<const Eigen::VectorXd> v_ref(out.v_ref.data(), layout_.nv);
    bool fed_back = false;
    if (!layout_.floating_base()) {
      fed_back = q_curr.allFinite() && v_curr.allFinite();
      riccati_.Compute(q_curr, v_curr, q_ref, v_ref, u_fb_);
    } else if (model_ != nullptr &&
               layout_.StateDifference(*model_, q_ref, v_ref, q_curr, v_curr, dx_)) {
      fed_back = riccati_.ComputeFromDelta(dx_, u_fb_);
    }
    if (!fed_back) {
      Bump(feedback_skips_);
    }
  }
  out.valid = true;
  return true;
}

bool MPCSolutionManager::SampleReference(const Eigen::Ref<const Eigen::VectorXd>& q_curr,
                                         const Eigen::Ref<const Eigen::VectorXd>& v_curr,
                                         uint64_t now_ns, WbcReference& out) noexcept {
  if (!Tick(q_curr, v_curr, now_ns, false, out)) {
    return false;
  }
  // The joint-acceleration rows of the feedback, at their tangent entries.
  const int row0 = riccati_.AccelOnly() ? 0 : layout_.acc_u_offset;
  const int n = std::min(layout_.n_acc, layout_.nv - layout_.n_base_v);
  for (int i = 0; i < n; ++i) {
    out.u_fb_acc[static_cast<std::size_t>(layout_.n_base_v + i)] = u_fb_[row0 + i];
  }
  return true;
}

bool MPCSolutionManager::ComputeReference(const Eigen::Ref<const Eigen::VectorXd>& q_curr,
                                          const Eigen::Ref<const Eigen::VectorXd>& v_curr,
                                          uint64_t now_ns, Eigen::Ref<Eigen::VectorXd> q_ref,
                                          Eigen::Ref<Eigen::VectorXd> v_ref,
                                          Eigen::Ref<Eigen::VectorXd> a_ff,
                                          Eigen::Ref<Eigen::VectorXd> lambda_ref,
                                          Eigen::Ref<Eigen::VectorXd> u_fb,
                                          InterpMeta& meta_out) noexcept {
  meta_out.valid = false;
  meta_out.beyond_horizon = false;
  meta_out.progress = 0.0;
  u_fb.setZero();
  if (!Tick(q_curr, v_curr, now_ns, true, sample_)) {
    return false;
  }
  const auto copy = [](const double* src, Eigen::Ref<Eigen::VectorXd> dst, int n) {
    const int m = std::min(n, static_cast<int>(dst.size()));
    dst.head(m) = Eigen::Map<const Eigen::VectorXd>(src, m);
  };
  copy(sample_.q_ref.data(), q_ref, sample_.nq);
  copy(sample_.v_ref.data(), v_ref, sample_.nv);
  copy(sample_.a_ref.data(), a_ff, sample_.nv);
  copy(sample_.wrench_ref.data(), lambda_ref, sample_.n_contact_vars);
  // Accel-only feedback goes to the joints' tangent entries (the front on a
  // fixed base); the full input-sized feedback to the front.
  const int at = riccati_.AccelOnly() ? layout_.n_base_v : 0;
  const int n_fb =
      std::max(0, std::min(static_cast<int>(u_fb.size()) - at, static_cast<int>(u_fb_.size())));
  u_fb.segment(at, n_fb) = u_fb_.head(n_fb);
  meta_out.valid = true;
  meta_out.beyond_horizon = sample_.beyond_horizon;
  meta_out.progress = sample_.progress;
  return true;
}

void MPCSolutionManager::DropActivePlan() noexcept {
  interpolator_.Clear();
  active_receive_stamp_ns_ = 0;
}

void MPCSolutionManager::EvictIfExpired(uint64_t now_ns, uint32_t model_epoch,
                                        uint32_t state_epoch) noexcept {
  const MPCSolution* s = interpolator_.Solution();
  if (s == nullptr) {
    return;
  }
  if (s->model_epoch != model_epoch) {
    Bump(epoch_rejections_);
    DropActivePlan();
    return;
  }
  if (s->state_epoch != state_epoch) {
    Bump(state_epoch_rejections_);
    DropActivePlan();
    return;
  }
  // Integer age: the two stamps are in one clock, so a stamp ahead of `now`
  // by more than the cap means that clock went back.
  const int64_t age_ns =
      static_cast<int64_t>(now_ns) - static_cast<int64_t>(s->observation_stamp_ns);
  const double cap_ns = static_cast<double>(s->horizon_length + 1) * s->dt_node * 1e9;
  if (std::abs(static_cast<double>(age_ns)) > cap_ns ||
      (max_policy_age_ns_ > 0 && age_ns > max_policy_age_ns_)) {
    Bump(policy_age_rejections_);
    DropActivePlan();
  }
}

}  // namespace rtc::mpc
