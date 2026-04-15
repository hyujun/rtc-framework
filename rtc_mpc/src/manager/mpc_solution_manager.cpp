#include "rtc_mpc/manager/mpc_solution_manager.hpp"

#include <algorithm>
#include <cstring>

namespace rtc::mpc {

void MPCSolutionManager::Init(const YAML::Node& cfg,
                              int nq, int nv, int n_contact_vars) {
  nq_ = nq;
  nv_ = nv;
  n_contact_vars_ = n_contact_vars;

  enabled_ = cfg["enabled"] && cfg["enabled"].as<bool>();
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

  const int nx = nq + nv;
  riccati_.Init(nv, nv, nx);  // nu == nv in accel-only mode

  if (r && r["gain_scale"]) {
    riccati_.SetGainScale(r["gain_scale"].as<double>());
  }
  const bool accel_only = !r || !r["accel_only"] ||
                           r["accel_only"].as<bool>();
  riccati_.SetAccelOnly(accel_only);

  if (r && r["max_delta_x_norm"]) {
    riccati_.SetMaxDeltaXNorm(r["max_delta_x_norm"].as<double>());
  }

  stale_count_ = 0;
  has_ever_received_solution_ = false;
}

void MPCSolutionManager::PublishSolution(const MPCSolution& sol) noexcept {
  MPCSolution& slot = solution_buffer_.StartWrite();
  std::memcpy(&slot, &sol, sizeof(MPCSolution));
  solution_buffer_.FinishWrite();
}

MPCStateSnapshot MPCSolutionManager::ReadState() const noexcept {
  return state_lock_.Load();
}

void MPCSolutionManager::WriteState(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v,
    uint64_t timestamp_ns) noexcept {
  MPCStateSnapshot snap{};
  const int nq = std::min(static_cast<int>(q.size()),
                          static_cast<int>(kMaxNq));
  const int nv = std::min(static_cast<int>(v.size()),
                          static_cast<int>(kMaxNv));
  for (int i = 0; i < nq; ++i) {
    snap.q[static_cast<std::size_t>(i)] = q(i);
  }
  for (int i = 0; i < nv; ++i) {
    snap.v[static_cast<std::size_t>(i)] = v(i);
  }
  snap.nq = nq;
  snap.nv = nv;
  snap.timestamp_ns = timestamp_ns;
  state_lock_.Store(snap);
}

bool MPCSolutionManager::ComputeReference(
    const Eigen::Ref<const Eigen::VectorXd>& q_curr,
    const Eigen::Ref<const Eigen::VectorXd>& v_curr,
    uint64_t now_ns,
    Eigen::Ref<Eigen::VectorXd> q_ref,
    Eigen::Ref<Eigen::VectorXd> v_ref,
    Eigen::Ref<Eigen::VectorXd> a_ff,
    Eigen::Ref<Eigen::VectorXd> lambda_ref,
    Eigen::Ref<Eigen::VectorXd> u_fb,
    InterpMeta& meta_out) noexcept {
  meta_out.valid = false;
  meta_out.beyond_horizon = false;
  meta_out.progress = 0.0;
  u_fb.setZero();

  if (!enabled_) {
    return false;
  }

  // Try to pick up a newly published solution.
  const MPCSolution* fresh = solution_buffer_.TryAcquireLatest();
  if (fresh != nullptr) {
    interpolator_.SetSolution(*fresh, now_ns);
    // Install gains from the first Riccati node (nearest-node policy).
    if (riccati_enabled_ && fresh->horizon_length > 0) {
      const int nu = fresh->nu > 0 ? fresh->nu : fresh->nv;
      riccati_.SetGain(fresh->K_riccati[0].data(), nu, fresh->nx);
    }
    stale_count_ = 0;
    has_ever_received_solution_ = true;
  } else if (has_ever_received_solution_) {
    ++stale_count_;
  }

  if (!interpolator_.HasSolution()) {
    return false;
  }
  if (stale_count_ >= max_stale_solutions_) {
    return false;
  }

  interpolator_.Interpolate(now_ns, q_ref, v_ref, a_ff, lambda_ref, meta_out);
  if (!meta_out.valid) {
    return false;
  }

  if (riccati_enabled_ && riccati_.HasGain()) {
    riccati_.Compute(q_curr, v_curr, q_ref, v_ref, u_fb);
  }
  return true;
}

}  // namespace rtc::mpc
