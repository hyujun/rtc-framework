// The NLP catch search (E1-F14 #740). See nlp_catch_search.hpp.
#include "rtc_controllers/catching/nlp_catch_search.hpp"

#include "rtc_controllers/catching/node_follower.hpp"  // NodeTrajectoryFollower::SampleJoints

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace rtc::catching {

namespace {

// ns → s by DIVISION: the quotient is correctly rounded, so 50'000'000 ns is
// exactly the literal 0.05 a caller wrote. A product with 1e-9 is one ulp off.
constexpr double kNsPerSec = 1e9;
// A warm-up solve may take this many shares of the solve budget: enough for
// its first iterations, bounded whatever the problem turns out to be.
constexpr std::int64_t kWarmUpShares = 4;

[[nodiscard]] bool FiniteNonNegative(double x) noexcept {
  return std::isfinite(x) && x >= 0.0;
}

[[nodiscard]] bool FinitePositive(double x) noexcept {
  return std::isfinite(x) && x > 0.0;
}

// √λ_max of the position block, or NaN.
[[nodiscard]] double SigmaMaxOf(const BallNodeSample& b) noexcept {
  if (!b.cov_valid) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const Eigen::Matrix3d s = b.cov.topLeftCorner<3, 3>();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
  es.computeDirect(s, Eigen::EigenvaluesOnly);
  const double lmax = es.eigenvalues().maxCoeff();
  return std::isfinite(lmax) ? std::sqrt(std::max(lmax, 0.0))
                             : std::numeric_limits<double>::quiet_NaN();
}

[[nodiscard]] bool IsChanceGroup(int group) noexcept {
  return group == static_cast<int>(DockingRowGroup::kLateral) ||
         group == static_cast<int>(DockingRowGroup::kTiming) ||
         group == static_cast<int>(DockingRowGroup::kVelocitySet);
}

// Whether a warm-up solve got as far as its main QP.
struct WarmUpProbe {
  bool reached_qp{false};
};

void WarmUpStageHook(MpcDockingStage stage, bool begin, void* user) noexcept {
  if (begin && stage == MpcDockingStage::kAssemble) {
    static_cast<WarmUpProbe*>(user)->reached_qp = true;
  }
}

}  // namespace

NlpCatchSearch::NlpCatchSearch() = default;
NlpCatchSearch::~NlpCatchSearch() = default;

bool NlpCatchSearch::Configure(const NlpCatchSearchModel& model,
                               const NlpCatchSearchConstants& constants,
                               const NlpCatchSearchParams& params, const CatchPoseIkOptions& ik,
                               ClockFn clock, std::string* error) {
  configured_ = false;
  cores_.clear();
  inputs_.clear();
  results_.clear();
  const auto fail = [error](std::string why) {
    if (error != nullptr) {
      *error = std::move(why);
    }
    return false;
  };
  if (!model.arm || model.handle == nullptr || clock == nullptr) {
    return fail("nlp search: no arm model, no IK handle or no clock");
  }
  const int nv = model.nv;
  if (nv < 1 || nv > kMaxSegmentNv || model.arm->nv != nv || model.arm->nq != nv) {
    return fail("nlp search: nv must be the arm model's and at most " +
                std::to_string(kMaxSegmentNv));
  }
  if (model.handle->nv() != nv) {
    return fail("nlp search: the IK handle is on a model of another joint count than the arm");
  }
  std::array<bool, kMaxSegmentNv> seen{};
  for (int m = 0; m < nv; ++m) {
    const int d = model.device_of_model[U(m)];
    if (d < 0 || d >= nv || seen[U(d)]) {
      return fail("nlp search: device_of_model is not a permutation of the arm's joints");
    }
    seen[U(d)] = true;
  }
  const NlpCatchSearchParams& p = params;
  if (p.wait_pose_n != nv) {
    return fail("nlp search: wait_pose must have one entry per arm joint");
  }
  for (int d = 0; d < nv; ++d) {
    if (!std::isfinite(p.wait_pose[U(d)])) {
      return fail("nlp search: wait_pose is not finite");
    }
  }
  if (!p.catch_box.set) {
    return fail("nlp search: catch_box is not set");
  }
  if (!FinitePositive(p.cand_dt) || !FinitePositive(p.t_lead_min) || !FinitePositive(p.t_max) ||
      !FinitePositive(p.dt_pre) || !FinitePositive(p.dt_stop) || !FinitePositive(p.budget_s) ||
      !FinitePositive(p.solve_budget_s) || !FiniteNonNegative(p.start_lead_s) ||
      !FinitePositive(p.t_ref_s) || !FiniteNonNegative(p.w_time) ||
      !FiniteNonNegative(p.w_switch) || !FiniteNonNegative(p.rank_w_q) ||
      !FiniteNonNegative(p.rank_w_manip) || !FiniteNonNegative(p.rest_tol) ||
      !FinitePositive(p.rt_state_age_max_s) || !FiniteNonNegative(constants.t_arm_s) ||
      !FinitePositive(constants.control_dt)) {
    return fail("nlp search: a time, weight or tolerance is non-finite or out of range");
  }
  if (p.max_solves < 1 || p.max_solves > kNlpMaxSolves) {
    return fail("nlp search: max_solves must be in [1, " + std::to_string(kNlpMaxSolves) + "]");
  }
  if (p.n_pre_min < 1 || p.n_pre_max < p.n_pre_min || p.n_stop < 3 ||
      p.n_pre_max + p.n_stop > kMaxSegmentNodes) {
    return fail(
        "nlp search: the arm grid needs 1 <= n_pre_min <= n_pre_max, n_stop >= 3 and "
        "n_pre_max + n_stop <= " +
        std::to_string(kMaxSegmentNodes));
  }
  if (p.n_stop_blocks < 3 || p.n_stop_blocks > p.n_stop ||
      p.n_pre_max + p.n_stop_blocks > kMaxSegmentNodes) {
    return fail("nlp search: the stop part needs at least 3 move blocks and no more than n_stop");
  }
  int stop_sum = 0;
  for (int b = 0; b < p.n_stop_blocks; ++b) {
    if (p.stop_block_sizes[U(b)] < 1) {
      return fail("nlp search: a stop block is empty");
    }
    stop_sum += p.stop_block_sizes[U(b)];
  }
  if (stop_sum != p.n_stop) {
    return fail("nlp search: the stop blocks do not add up to n_stop");
  }

  // The keys as the integers the lattice and the grid are computed on.
  h_ns_ = SecondsToNs(p.cand_dt);
  dt_pre_ns_ = SecondsToNs(p.dt_pre);
  dt_stop_ns_ = SecondsToNs(p.dt_stop);
  t_lead_min_ns_ = SecondsToNs(p.t_lead_min);
  t_max_ns_ = SecondsToNs(p.t_max);
  t_arm_ns_ = SecondsToNs(constants.t_arm_s);
  control_dt_ns_ = SecondsToNs(constants.control_dt);
  budget_ns_ = SecondsToNs(p.budget_s);
  solve_budget_ns_ = SecondsToNs(p.solve_budget_s);
  start_lead_ns_ = SecondsToNs(p.start_lead_s);
  age_max_ns_ = SecondsToNs(p.rt_state_age_max_s);
  if (h_ns_ <= 0 || dt_pre_ns_ <= 0 || dt_pre_ns_ > kMaxSegmentDtPreNs || dt_stop_ns_ <= 0 ||
      solve_budget_ns_ <= 0 || budget_ns_ <= 0) {
    return fail("nlp search: a spacing or budget rounds to no nanoseconds, or dt_pre is over 1 s");
  }
  // The pre-catch grid must COVER the candidate window: a candidate farther
  // than n_pre_max·Δ_a would have the arm wait before it moves, and one
  // nearer than n_pre_min·Δ_a has no grid to solve on.
  if (static_cast<std::int64_t>(p.n_pre_max) * dt_pre_ns_ < t_max_ns_) {
    return fail(
        "nlp search: n_pre_max * dt_pre is shorter than t_max — the pre-catch grid does "
        "not cover the candidate window");
  }
  if (t_lead_min_ns_ < static_cast<std::int64_t>(p.n_pre_min) * dt_pre_ns_) {
    return fail("nlp search: t_lead_min is shorter than n_pre_min * dt_pre");
  }
  if (t_max_ns_ < t_lead_min_ns_) {
    return fail("nlp search: t_max is below t_lead_min");
  }
  if (p.cand_capacity < 1 || p.cand_capacity > kNlpMaxCandidates ||
      t_max_ns_ / h_ns_ + 1 > static_cast<std::int64_t>(p.cand_capacity)) {
    return fail(
        "nlp search: cand_capacity must hold floor(t_max / cand_dt) + 1 candidates and "
        "be at most " +
        std::to_string(kNlpMaxCandidates));
  }

  model_ = model;
  constants_ = constants;
  params_ = params;
  ik_options_ = ik;
  clock_ = clock;
  nv_ = nv;

  // ── One core per pre-catch count ──────────────────────────────────────────
  const int n_cores = p.n_pre_max - p.n_pre_min + 1;
  cores_.reserve(U(n_cores));
  inputs_.resize(U(n_cores));
  results_.resize(U(n_cores));
  for (int n_pre = p.n_pre_min; n_pre <= p.n_pre_max; ++n_pre) {
    MpcDockingSegmentCoreParams cp = p.core;
    cp.n_pre = n_pre;
    // The spacing the integer grid uses, exactly.
    cp.dt_pre = static_cast<double>(dt_pre_ns_) / kNsPerSec;
    cp.n_stop = p.n_stop;
    cp.dt_stop = static_cast<double>(dt_stop_ns_) / kNsPerSec;
    cp.n_blocks = n_pre + p.n_stop_blocks;
    cp.block_sizes.fill(1);  // one block per pre-catch interval
    for (int b = 0; b < p.n_stop_blocks; ++b) {
      cp.block_sizes[U(n_pre + b)] = p.stop_block_sizes[U(b)];
    }
    auto core = std::make_unique<MpcDockingSegmentCore>();
    const MpcDockingReason why = core->Init(*model.arm, model.catch_frame, cp, p.limits, clock);
    if (why != MpcDockingReason::kNone) {
      cores_.clear();
      return fail(std::string("nlp search: the core for n_pre = ") + std::to_string(n_pre) +
                  " refused its grid or parameters: " + MpcDockingReasonName(why));
    }
    const std::size_t slot = U(n_pre - p.n_pre_min);
    core->ResizeInput(inputs_[slot]);
    core->ResizeResult(results_[slot]);
    cores_.push_back(std::move(core));
  }
  timing_on_ = p.core.chance && p.core.timing_row;
  impact_on_ = std::isfinite(p.core.e_max) || std::isfinite(p.core.p_max);
  sigma_max_ = cores_.front()->TimingSigmaMax();

  // ── Screening's own kinematics, on the cores' model ───────────────────────
  arm_model_ = *model.arm;
  if (p.limits.armature.size() == nv) {
    arm_model_.armature = model.arm->armature + p.limits.armature;
  }
  arm_data_ = pinocchio::Data(arm_model_);
  kin_.Resize(nv);
  impact_work_.Init(arm_model_);
  impact_.Resize(nv);
  manip_work_.Init(arm_model_);
  manip_d_q_ = p.core.manip_d_q.size() == nv ? p.core.manip_d_q : Eigen::VectorXd::Ones(nv);
  manip_grad_ = Eigen::VectorXd::Zero(nv);
  q_model_ = Eigen::VectorXd::Zero(nv);
  v_zero_ = Eigen::VectorXd::Zero(nv);

  ik_.Resize(nv);
  seed_yaml_ = Eigen::VectorXd::Zero(nv);
  for (int m = 0; m < nv; ++m) {
    seed_yaml_[m] = p.wait_pose[U(model.device_of_model[U(m)])];
  }
  seed_ = seed_yaml_;

  cands_.assign(U(p.cand_capacity), CandidateRecord{});
  ranked_.assign(U(p.cand_capacity), 0);
  memory_.assign(U(p.cand_capacity), Memory{});
  fresh_.assign(U(p.max_solves), Memory{});
  n_cands_ = 0;
  eval_order_n_ = 0;
  ResetTrial();

  if (!WarmUp(error)) {
    cores_.clear();
    return false;
  }
  configured_ = true;
  return true;
}

// One solve per solver, outside any trial: the first solve of a ProxQP object
// is its slowest and allocates the most. Success is not asked for — the wait
// pose with a ball dropped down its capture axis is a problem, not a test —
// only that the solve got to its main QP, and it is bounded by a deadline.
bool NlpCatchSearch::WarmUp(std::string* error) {
  const auto fail = [error](std::string why) {
    if (error != nullptr) {
      *error = std::move(why);
    }
    return false;
  };
  const MpcDockingSegmentCore& first = *cores_.front();
  Eigen::VectorXd q = seed_yaml_;
  for (int m = 0; m < nv_; ++m) {
    q[m] = std::clamp(q[m], first.PositionLower()[m], first.PositionUpper()[m]);
  }
  if (!ComputeDockingFrameKinematics(arm_model_, arm_data_, model_.catch_frame, q, v_zero_, kin_)) {
    return fail("nlp search: the catch frame's kinematics cannot be evaluated at the wait pose");
  }
  const Eigen::Vector3d e3 = kin_.R.col(2);
  const double speed = std::fabs(params_.core.nu_ref.z());
  const Eigen::Vector3d v_b = -(speed > 0.0 ? speed : 1.0) * e3;
  const Eigen::Vector3d p_b = kin_.p + params_.core.s_ent * e3;
  // The IK's solver: the pose it is already at.
  static_cast<void>(ik_.Solve(*model_.handle, model_.catch_frame, kin_.p, v_b, q, ik_options_));
  BallCovariance cov = BallCovariance::Zero();
  cov.diagonal() << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
  for (std::size_t slot = 0; slot < cores_.size(); ++slot) {
    MpcDockingSegmentCore& core = *cores_[slot];
    MpcDockingSegmentCoreInput& in = inputs_[slot];
    const int kc = core.CatchNode();
    const double t_c = core.NodeTime(kc);
    in.q0 = q;
    in.qd0.setZero();
    in.qdd0.setZero();
    for (int k = 0; k <= kc; ++k) {
      BallNodeSample& b = in.ball[U(k)];
      b = BallNodeSample{};
      b.p = p_b + v_b * (core.NodeTime(k) - t_c);
      b.v = v_b;
      b.valid = true;
    }
    in.ball[U(kc)].cov = cov;
    in.ball[U(kc)].cov_valid = true;
    in.initial_valid = false;
    in.catch_target_valid = true;
    in.q_catch_target = q;
    in.p_line = p_b;
    in.d_line = -e3;
    in.deadline_ns = clock_() + kWarmUpShares * solve_budget_ns_;
    WarmUpProbe probe;
    core.SetStageHook(&WarmUpStageHook, &probe);
    static_cast<void>(core.Solve(in, results_[slot]));
    core.SetStageHook(nullptr, nullptr);
    if (!probe.reached_qp) {
      return fail(std::string("nlp search: the warm-up solve of the core for n_pre = ") +
                  std::to_string(core.CatchNode()) +
                  " ended before its QP: " + MpcDockingReasonName(results_[slot].reason));
    }
    in.deadline_ns = 0;
  }
  return true;
}

void NlpCatchSearch::ResetTrial() noexcept {
  anchor_set_ = false;
  t_ref_ns_ = 0;
  track_known_ = false;
  track_generation_ = 0;
  chose_before_ = false;
  last_chosen_t_c_ns_ = 0;
  solution_valid_ = false;
  for (Memory& m : memory_) {
    m.valid = false;
  }
}

void NlpCatchSearch::NotePublished(const PlanSnapshot& plan) noexcept {
  static_cast<void>(plan);
}

void NlpCatchSearch::SetEvaluationOrderForTesting(std::span<const int> order) noexcept {
  eval_order_n_ = 0;
  if (order.size() > eval_order_.size()) {
    return;
  }
  for (std::size_t i = 0; i < order.size(); ++i) {
    eval_order_[i] = order[i];
  }
  eval_order_n_ = static_cast<int>(order.size());
}

void NlpCatchSearch::SetCoreStageHookForTesting(MpcDockingSegmentCore::StageHook hook,
                                                void* user) noexcept {
  for (auto& core : cores_) {
    core->SetStageHook(hook, user);
  }
}

const MpcDockingSegmentCore& NlpCatchSearch::Core(int n_pre) const noexcept {
  const int slot = std::clamp(CoreSlot(n_pre), 0, static_cast<int>(cores_.size()) - 1);
  return *cores_[U(slot)];
}

std::size_t NlpCatchSearch::MemorySlot(std::int64_t index) const noexcept {
  const auto cap = static_cast<std::int64_t>(memory_.size());
  const std::int64_t r = index % cap;
  return static_cast<std::size_t>(r < 0 ? r + cap : r);
}

const NlpCatchSearch::Memory* NlpCatchSearch::Remembered(std::int64_t index) const noexcept {
  if (memory_.empty()) {
    return nullptr;
  }
  const Memory& m = memory_[MemorySlot(index)];
  return m.valid && m.index == index ? &m : nullptr;
}

const NlpCatchSearch::Memory* NlpCatchSearch::NearestRemembered(std::int64_t index) const noexcept {
  const Memory* best = nullptr;
  std::int64_t best_d = 0;
  for (const Memory& m : memory_) {
    if (!m.valid) {
      continue;
    }
    const std::int64_t d = m.index > index ? m.index - index : index - m.index;
    // The smaller index on a tie: the choice must not depend on the slot order.
    if (best == nullptr || d < best_d || (d == best_d && m.index < best->index)) {
      best = &m;
      best_d = d;
    }
  }
  return best;
}

const SegmentSnapshot* NlpCatchSearch::RememberedSolution(std::int64_t index) const noexcept {
  const Memory* m = Remembered(index);
  return m != nullptr ? &m->seg : nullptr;
}

void NlpCatchSearch::Monitor(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                             bool cov_matched, const PlannerRtState& rt,
                             SearchStats& stats) const noexcept {
  stats = SearchStats{};
  stats.publish = false;
  if (!configured_ || !rt.plan_active || !traj.valid) {
    return;
  }
  int hint = 0;
  const BallNodeSample b = SampleBallNode(traj, &cov, cov_matched, BallTime{rt.plan_t_c_ns}, hint);
  if (b.valid) {
    stats.sigma_l = SigmaMaxOf(b);
  }
}

bool NlpCatchSearch::StartState(const PlannerRtState& rt, const ReportedSegments& arm,
                                bool following, CandidateRecord& c) noexcept {
  const std::size_t n = U(nv_);
  if (!following) {
    // No plan followed: the arm rests on its command (the wake checked that).
    for (int m = 0; m < nv_; ++m) {
      c.q0[U(m)] = rt.q_cmd[U(model_.device_of_model[U(m)])];
      c.qd0[U(m)] = 0.0;
      c.qdd0[U(m)] = 0.0;
    }
    c.source_seq = 0;
  } else {
    // The segment the RT will be on at t_s, evaluated by the RT's own sampler.
    const SegmentSnapshot* src = SourceSegmentAt(arm, c.t_s_ns);
    if (src == nullptr || src->nv != nv_) {
      return false;
    }
    std::array<double, kMaxSegmentNv> q{};
    std::array<double, kMaxSegmentNv> qd{};
    std::array<double, kMaxSegmentNv> qdd{};
    if (!NodeTrajectoryFollower::SampleJoints(*src, c.t_s_ns, q, qd, qdd)) {
      return false;
    }
    for (int m = 0; m < nv_; ++m) {
      const std::size_t d = U(model_.device_of_model[U(m)]);
      c.q0[U(m)] = q[d];
      c.qd0[U(m)] = qd[d];
      c.qdd0[U(m)] = qdd[d];
    }
    c.source_seq = src->segment_seq;
  }
  for (std::size_t m = 0; m < n; ++m) {
    if (!std::isfinite(c.q0[m]) || !std::isfinite(c.qd0[m]) || !std::isfinite(c.qdd0[m])) {
      return false;
    }
  }
  const MpcDockingSegmentCore& core = *cores_.front();
  c.x0_clamped =
      ProjectStartIntoBox(std::span<double>(c.q0.data(), n), std::span<double>(c.qd0.data(), n),
                          std::span<const double>(core.PositionLower().data(), n),
                          std::span<const double>(core.PositionUpper().data(), n),
                          std::span<const double>(core.VelocityLimit().data(), n));
  return true;
}

void NlpCatchSearch::Screen(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                            bool cov_matched, const PlannerRtState& rt, const ReportedSegments& arm,
                            bool following, bool has_previous, std::int64_t t_c_prev_ns,
                            CandidateRecord& c, SearchStats& stats) noexcept {
  const std::size_t n = U(nv_);
  // (S1) the lead. A candidate nearer than the minimum has too few pre-catch
  // intervals — or none — to solve on.
  if (c.t_c_ns - t_0_ns_ < t_lead_min_ns_ || c.n_pre < params_.n_pre_min) {
    c.reject = NlpReject::kLeadShort;
    return;
  }
  ++stats.n_in_window;
  // The ball at every node of the grid, as the solve will read it.
  int hint = 0;
  BallNodeSample at_catch;
  for (int k = 0; k <= c.n_pre; ++k) {
    const BallTime t{c.t_s_ns + static_cast<std::int64_t>(k) * dt_pre_ns_};
    const BallNodeSample b = SampleBallNode(traj, &cov, cov_matched, t, hint);
    if (!b.valid || b.after_horizon || !b.p.allFinite() || !b.v.allFinite() || !b.a.allFinite()) {
      c.reject = NlpReject::kBallInvalid;
      return;
    }
    if (k == c.n_pre) {
      at_catch = b;
    }
  }
  const double speed = at_catch.v.norm();
  if (!(speed >= ik_options_.v_eps)) {
    c.reject = NlpReject::kBallInvalid;
    return;
  }
  if (!params_.catch_box.Contains(at_catch.p.x(), at_catch.p.y(), at_catch.p.z())) {
    c.reject = NlpReject::kWorkspace;
    return;
  }
  if (params_.core.chance && !at_catch.cov_valid) {
    c.reject = NlpReject::kCovariance;
    return;
  }
  if (!StartState(rt, arm, following, c)) {
    c.reject = NlpReject::kNoSource;
    return;
  }

  // (S4) the catch pose: the frame's origin on the entrance plane's far side
  // of the ball (the ball is s_ent along +e₃ = −v̂ of it), +z against the ball.
  const Eigen::Vector3d v_hat = at_catch.v / speed;
  const Eigen::Vector3d target = at_catch.p + params_.core.s_ent * v_hat;
  if (solver_hook_ != nullptr) {
    solver_hook_(true, solver_user_);
  }
  const std::int64_t ik_start = clock_();
  const CatchPoseIkResult ik =
      ik_.Solve(*model_.handle, model_.catch_frame, target, at_catch.v, seed_, ik_options_);
  const std::int64_t ik_ns = clock_() - ik_start;
  if (solver_hook_ != nullptr) {
    solver_hook_(false, solver_user_);
  }
  ++stats.n_ik;
  stats.ik_ns_max = std::max(stats.ik_ns_max, ik_ns);
  c.ik_run = true;
  c.ik_reason = ik.reason;
  if (!ik.accepted) {
    c.reject =
        ik.reason == CatchPoseReason::kBelowManipMin ? NlpReject::kManipulability : NlpReject::kIk;
    return;
  }
  for (int m = 0; m < nv_; ++m) {
    c.q_ik[U(m)] = ik.q[m];
    q_model_[m] = ik.q[m];
  }
  c.w5 = ik.w5;
  c.w6 = ik.w6;

  // (S4) reach, over the time the arm actually moves: its n_pre intervals.
  const MpcDockingSegmentCore& core = *cores_[U(CoreSlot(c.n_pre))];
  const double t_move =
      static_cast<double>(static_cast<std::int64_t>(c.n_pre) * dt_pre_ns_) / kNsPerSec;
  const NlpReachVerdict reach = NlpReachCheck(
      std::span<const double>(c.q_ik.data(), n), std::span<const double>(c.q0.data(), n),
      std::span<const double>(c.qd0.data(), n),
      std::span<const double>(core.VelocityLimit().data(), n),
      std::span<const double>(core.AccelerationLimit().data(), core.HasAccelerationBox() ? n : 0U),
      t_move);
  c.reach_limit = reach.limit;
  c.reach_joint = reach.joint;
  if (!reach.Reachable()) {
    c.reject = NlpReject::kReach;
    return;
  }

  // (S3) the closing-speed window, every term at the IK pose.
  NlpSpeedWindowInput w;
  w.c_min = params_.core.c_min;
  w.c_cap_max = params_.core.c_cap_max;
  bool evaluated = ComputeDockingFrameKinematics(arm_model_, arm_data_, model_.catch_frame,
                                                 q_model_, v_zero_, kin_);
  if (evaluated && timing_on_) {
    const Eigen::Vector3d d = kin_.R.col(2);
    const Eigen::Matrix3d sigma_p = at_catch.cov.topLeftCorner<3, 3>();
    const double var = d.dot(sigma_p * d);
    w.timing = true;
    // A negative variance is no variance: NaN empties the window.
    w.sigma_s = var >= 0.0 ? std::sqrt(var + params_.core.eps_sigma * params_.core.eps_sigma)
                           : std::numeric_limits<double>::quiet_NaN();
    w.sigma_max = sigma_max_;
    w.sigma_tau = params_.core.sigma_tau;
  }
  if (evaluated && impact_on_) {
    evaluated =
        ComputeDockingImpact(arm_model_, model_.catch_frame, q_model_, v_zero_, kin_, at_catch.v,
                             params_.core.contact_point_hand, params_.core.m_ball,
                             params_.core.restitution, impact_work_, impact_);
    w.impact = true;
    w.m_red = impact_.m_red;
    w.e_max = params_.core.e_max;
    w.p_max = params_.core.p_max;
    w.restitution = params_.core.restitution;
  }
  const NlpSpeedWindow window = NlpClosingSpeedWindow(w);
  c.speed_lo = window.lo;
  c.speed_hi = window.hi;
  if (!evaluated || window.empty) {
    c.reject = NlpReject::kSpeedWindow;
    return;
  }

  // The rank: the outer cost's two terms and a proxy for the motion's.
  c.j_time = NlpTimeCost(params_.w_time, c.lead_s, params_.t_ref_s);
  c.j_switch =
      NlpSwitchCost(params_.w_switch, c.t_c_ns, has_previous, t_c_prev_ns, params_.t_ref_s);
  double dq_sq = 0.0;
  for (std::size_t m = 0; m < n; ++m) {
    const double dq = c.q_ik[m] - c.q0[m];
    dq_sq += dq * dq;
  }
  double psi = 0.0;
  if (params_.rank_w_manip > 0.0 &&
      !ComputeDockingManipulability(arm_model_, model_.catch_frame, q_model_,
                                    params_.core.manip_d_lin, params_.core.manip_d_ang, manip_d_q_,
                                    params_.core.manip_delta, manip_work_, psi, manip_grad_)) {
    psi = 0.0;
  }
  c.rank_key = c.j_time + c.j_switch + params_.rank_w_q * dq_sq + params_.rank_w_manip * psi;
  if (!std::isfinite(c.rank_key)) {
    // Last in the rank, and still an ordering: a NaN key is none.
    c.rank_key = std::numeric_limits<double>::infinity();
  }
  c.reject = NlpReject::kNotRanked;  // until a solve says otherwise
}

void NlpCatchSearch::Pack(const PlannerRtState& rt, std::uint64_t track_generation,
                          const CandidateRecord& c, const MpcDockingSegmentCoreResult& r,
                          SegmentSnapshot& out) const noexcept {
  const int n_total = c.n_pre + params_.n_stop;
  // Whole, not field by field: the node arrays' unused entries (joints past
  // nv, nodes past N) would otherwise keep an earlier, longer solution's.
  out = SegmentSnapshot{};
  out.token.activation_generation = rt.activation_generation;
  out.token.generation = track_generation;
  out.rt_iteration = rt.rt_iteration;
  out.rt_state_ns = rt.rt_state_ns;
  out.t_c_ns = c.t_c_ns;
  out.t0_ns = c.t_s_ns;
  out.dt_ns = dt_stop_ns_;
  out.dt_pre_ns = dt_pre_ns_;
  out.n_nodes = n_total;
  out.nv = nv_;
  out.n_pre = c.n_pre;
  for (int k = 0; k <= n_total; ++k) {
    for (int m = 0; m < nv_; ++m) {
      const std::size_t e = U(k * kMaxSegmentNv + model_.device_of_model[U(m)]);
      out.q[e] = r.q(m, k);
      out.qd[e] = r.qd(m, k);
      out.qdd[e] = r.qdd(m, k);
    }
  }
  out.tau_ratio_max = r.tau_ratio_max;
  out.x0_clamped = c.x0_clamped;
  out.valid = true;
}

void NlpCatchSearch::Solve(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                           bool cov_matched, const PlannerRtState& rt, CandidateRecord& c,
                           Memory& out) noexcept {
  const std::size_t slot = U(CoreSlot(c.n_pre));
  MpcDockingSegmentCore& core = *cores_[slot];
  MpcDockingSegmentCoreInput& in = inputs_[slot];
  MpcDockingSegmentCoreResult& res = results_[slot];
  const int n_total = c.n_pre + params_.n_stop;

  // Every field the core reads is written here, for THIS candidate: the input
  // and the result are shared by every candidate with the same n_pre.
  for (int m = 0; m < nv_; ++m) {
    in.q0[m] = c.q0[U(m)];
    in.qd0[m] = c.qd0[U(m)];
    in.qdd0[m] = c.qdd0[U(m)];
    in.q_catch_target[m] = c.q_ik[U(m)];
  }
  int hint = 0;
  for (int k = 0; k <= c.n_pre; ++k) {
    const BallTime t{c.t_s_ns + static_cast<std::int64_t>(k) * dt_pre_ns_};
    in.ball[U(k)] = SampleBallNode(traj, &cov, cov_matched, t, hint);
  }
  const BallNodeSample& at_catch = in.ball[U(c.n_pre)];
  // The stop line: through the catch point, along the ball's travel.
  in.p_line = at_catch.p;
  in.d_line = at_catch.v.normalized();
  in.catch_target_valid = true;
  in.initial_valid = false;
  c.start = Start::kIkTarget;
  c.start_index = c.index;

  // The start point, from the previous wakes' memory only.
  const Memory* own = Remembered(c.index);
  if (own != nullptr && own->seg.t_c_ns == c.t_c_ns && own->seg.nv == nv_ &&
      own->seg.n_pre >= c.n_pre && own->seg.n_nodes - own->seg.n_pre == params_.n_stop) {
    // The same candidate: the nodes that have passed are dropped, the rest
    // are the same instants.
    const int shift = own->seg.n_pre - c.n_pre;
    for (int k = 0; k <= n_total; ++k) {
      for (int m = 0; m < nv_; ++m) {
        const std::size_t e = U((k + shift) * kMaxSegmentNv + model_.device_of_model[U(m)]);
        in.q_init(m, k) = own->seg.q[e];
        in.qd_init(m, k) = own->seg.qd[e];
        in.qdd_init(m, k) = own->seg.qdd[e];
      }
    }
    in.initial_valid = true;
    c.start = Start::kSameCandidate;
  } else if (const Memory* other = NearestRemembered(c.index);
             other != nullptr && other->seg.nv == nv_ && other->seg.t0_ns <= c.t_s_ns &&
             other->seg.t_c_ns > c.t_s_ns &&
             other->seg.n_nodes - other->seg.n_pre == params_.n_stop) {
    // A new candidate: the nearest remembered solution as a function of
    // absolute time, stretched about node 0's instant so that its catch
    // instant lands on this one's: σ(t) = t_s + α (t − t_s),
    // α = (t_c,prev − t_s)/(t_c − t_s); velocities scale by α, accelerations
    // by α². From the catch node on, its stop part is taken as it is.
    const SegmentSnapshot& prev = other->seg;
    const double alpha =
        static_cast<double>(prev.t_c_ns - c.t_s_ns) / static_cast<double>(c.t_c_ns - c.t_s_ns);
    std::array<double, kMaxSegmentNv> q{};
    std::array<double, kMaxSegmentNv> qd{};
    std::array<double, kMaxSegmentNv> qdd{};
    bool ok = std::isfinite(alpha) && alpha > 0.0;
    for (int k = 0; ok && k < c.n_pre; ++k) {
      const double since = alpha * static_cast<double>(static_cast<std::int64_t>(k) * dt_pre_ns_);
      const std::int64_t sigma_ns = c.t_s_ns + static_cast<std::int64_t>(std::llround(since));
      ok = NodeTrajectoryFollower::SampleJoints(prev, std::min(sigma_ns, prev.t_c_ns), q, qd, qdd);
      for (int m = 0; ok && m < nv_; ++m) {
        const std::size_t d = U(model_.device_of_model[U(m)]);
        in.q_init(m, k) = q[d];
        in.qd_init(m, k) = alpha * qd[d];
        in.qdd_init(m, k) = alpha * alpha * qdd[d];
      }
    }
    for (int k = c.n_pre; ok && k <= n_total; ++k) {
      const double scale_v = k == c.n_pre ? alpha : 1.0;
      for (int m = 0; m < nv_; ++m) {
        const std::size_t e =
            U((prev.n_pre + k - c.n_pre) * kMaxSegmentNv + model_.device_of_model[U(m)]);
        in.q_init(m, k) = prev.q[e];
        in.qd_init(m, k) = scale_v * prev.qd[e];
        in.qdd_init(m, k) = scale_v * scale_v * prev.qdd[e];
      }
    }
    if (ok) {
      in.initial_valid = true;
      c.start = Start::kNeighbour;
      c.start_index = other->index;
    }
  }
  if (in.initial_valid) {
    // Node 0 is the start state, whatever the remembered trajectory had there.
    for (int m = 0; m < nv_; ++m) {
      in.q_init(m, 0) = c.q0[U(m)];
      in.qd_init(m, 0) = c.qd0[U(m)];
      in.qdd_init(m, 0) = c.qdd0[U(m)];
    }
  }

  if (solver_hook_ != nullptr) {
    solver_hook_(true, solver_user_);
  }
  const std::int64_t start_ns = clock_();
  // Its own share of the budget, from its own start: what another candidate
  // spent or saved does not reach it.
  in.deadline_ns = start_ns + solve_budget_ns_;
  const bool ok = core.Solve(in, res);
  const std::int64_t end_ns = clock_();
  if (solver_hook_ != nullptr) {
    solver_hook_(false, solver_user_);
  }
  c.solve_ns = end_ns - start_ns;
  c.core_reason = res.reason;
  c.solved = ok;
  out.valid = false;
  if (ok) {
    // The trajectory is read only now: a refused solve leaves the result's
    // nodes as the previous candidate with this n_pre left them.
    c.feasible = res.feasible;
    c.converged = res.converged;
    c.iterations = res.iterations;
    c.start_us = res.start_us;
    c.qp_us = res.qp_us;
    c.qp_solves = res.qp_solves;
    c.j_reference = res.cost.reference;
    c.j_stop = res.cost.stop;
    double worst = 0.0;
    int worst_group = 0;
    bool chance_violated = false;
    bool other_violated = false;
    for (int g = 0; g < kNumDockingRowGroups; ++g) {
      const double v = res.violation[U(g)];
      if (!(v <= params_.core.tol_violation)) {
        (IsChanceGroup(g) ? chance_violated : other_violated) = true;
      }
      if (g == 0 || v > worst) {
        worst = v;
        worst_group = g;
      }
    }
    c.c_catch = res.c_catch;
    c.worst_violation = worst;
    c.worst_group = static_cast<DockingRowGroup>(worst_group);
    Pack(rt, traj.token.generation, c, res, out.seg);
    out.valid = true;
    out.index = c.index;
    out.source_seq = c.source_seq;
    out.cost_reference = res.cost.reference;
    out.cost_stop = res.cost.stop;
    out.feasible = res.feasible;
    out.converged = res.converged;
    if (res.reason == MpcDockingReason::kDeadline || end_ns > in.deadline_ns) {
      c.reject = NlpReject::kDeadline;
    } else if (!res.feasible) {
      // A solution only the chance rows refuse is refused for the ball's
      // uncertainty, not for the arm.
      c.reject = chance_violated && !other_violated ? NlpReject::kChance : NlpReject::kHardRow;
    } else if (!res.converged) {
      c.reject = NlpReject::kUnconverged;
    } else {
      c.reject = NlpReject::kNone;
      c.phi = c.j_reference + c.j_time + c.j_switch;
    }
  } else {
    c.reject = end_ns > in.deadline_ns ? NlpReject::kDeadline : NlpReject::kSolverRejected;
  }
}

PlanSnapshot NlpCatchSearch::Plan(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                  bool cov_matched, const PlannerRtState& rt,
                                  const ReportedSegments& arm, NowReal now,
                                  SearchStats& stats) noexcept {
  stats = SearchStats{};
  stats.nlp.ran = true;
  PlanSnapshot plan{};
  plan.token = traj.token;
  plan.token.activation_generation = rt.activation_generation;
  plan.rt_iteration = rt.rt_iteration;
  plan.rt_state_ns = rt.rt_state_ns;
  plan.valid = false;
  plan.reason = PlanReason::kNone;
  solution_valid_ = false;
  n_cands_ = 0;
  if (!configured_) {
    stats.nlp.reason = NlpReject::kRtInvalid;
    plan.reason = NlpPlanReason(stats.nlp.reason);
    return plan;
  }
  const std::int64_t t_start = clock_();
  const bool following = rt.plan_active;
  // A wake that ends without a candidate while the RT follows a plan HOLDS:
  // publishing "no plan" would take away what the arm is doing.
  const auto none = [&](NlpReject why) {
    stats.nlp.reason = why;
    plan.reason = NlpPlanReason(why);
    if (following) {
      stats.publish = false;
      stats.decision = SwitchDecision::kHeldNoCandidate;
    }
    stats.search_ns = clock_() - t_start;
    return plan;
  };

  // ── The RT's report ───────────────────────────────────────────────────────
  const std::int64_t age_ns = now.ns - rt.rt_state_ns;
  bool rt_ok = rt.valid && rt.nv == nv_ && age_ns >= 0 && age_ns <= age_max_ns_;
  double speed_cmd = 0.0;
  for (int d = 0; rt_ok && d < nv_; ++d) {
    rt_ok = std::isfinite(rt.q_cmd[U(d)]) && std::isfinite(rt.qd_cmd[U(d)]);
    speed_cmd = std::max(speed_cmd, std::fabs(rt.qd_cmd[U(d)]));
  }
  if (!rt_ok) {
    return none(NlpReject::kRtInvalid);
  }
  if (!traj.valid) {
    return none(NlpReject::kBallInvalid);
  }
  if (!following && !(speed_cmd <= params_.rest_tol)) {
    return none(NlpReject::kNotAtRest);
  }
  if (following && !arm.has_pending && !arm.has_following) {
    return none(NlpReject::kNoSource);
  }
  // What the command and the followed segment disagree by, where the RT read
  // the segment for that command (the lead instant plus one period).
  if (following && arm.has_following && arm.following.nv == nv_) {
    std::array<double, kMaxSegmentNv> q{};
    std::array<double, kMaxSegmentNv> qd{};
    std::array<double, kMaxSegmentNv> qdd{};
    if (NodeTrajectoryFollower::SampleJoints(
            arm.following, rt.rt_state_ns + t_arm_ns_ + control_dt_ns_, q, qd, qdd)) {
      double gap_q = 0.0;
      double gap_qd = 0.0;
      for (int d = 0; d < nv_; ++d) {
        gap_q = std::max(gap_q, std::fabs(rt.q_cmd[U(d)] - q[U(d)]));
        gap_qd = std::max(gap_qd, std::fabs(rt.qd_cmd[U(d)] - qd[U(d)]));
      }
      stats.nlp.cmd_gap_q = gap_q;
      stats.nlp.cmd_gap_qd = gap_qd;
    }
  }

  // ── The lattice ───────────────────────────────────────────────────────────
  // A new track is a new catch attempt: its lattice and its memory.
  if (!track_known_ || traj.token.generation != track_generation_) {
    track_known_ = true;
    track_generation_ = traj.token.generation;
    anchor_set_ = false;
    chose_before_ = false;
    for (Memory& m : memory_) {
      m.valid = false;
    }
  }
  if (!anchor_set_) {
    anchor_set_ = true;
    t_ref_ns_ = now.ns;
  }
  // The earliest instant a segment of this wake can be read by the RT.
  t_0_ns_ = now.ns + t_arm_ns_ + budget_ns_ + start_lead_ns_;
  // The IK seed: the wait pose — the RT's adopted one when it reports it.
  seed_ = seed_yaml_;
  if (rt.wait_pose_adopted) {
    for (int m = 0; m < nv_; ++m) {
      seed_[m] = rt.wait_pose[U(model_.device_of_model[U(m)])];
    }
  }
  // The switch term's anchor: the catch instant in force.
  const bool has_previous = following || chose_before_;
  const std::int64_t t_c_prev_ns = following ? rt.plan_t_c_ns : last_chosen_t_c_ns_;

  // Every lattice instant ahead of t_0 and inside the horizon; the ones nearer
  // than the minimum lead are kept as candidates so that they are REPORTED.
  const NlpCandidateRange range =
      NlpCandidatesInWindow(t_ref_ns_, h_ns_, t_0_ns_ + 1, t_0_ns_ + t_max_ns_);
  const std::int64_t count =
      std::min<std::int64_t>(range.Count(), static_cast<std::int64_t>(cands_.size()));
  // ── Screening, in lattice order ───────────────────────────────────────────
  for (std::int64_t i = 0; i < count; ++i) {
    CandidateRecord& c = cands_[static_cast<std::size_t>(i)];
    c = CandidateRecord{};
    c.index = range.first + i;
    c.t_c_ns = NlpCandidateInstant(t_ref_ns_, h_ns_, c.index);
    const NlpCandidateGrid grid = NlpCandidateGridAt(c.t_c_ns, t_0_ns_, dt_pre_ns_);
    c.n_pre = grid.n_pre;
    c.t_s_ns = grid.t_s_ns;
    c.wait_ns = grid.wait_ns;
    c.lead_s = static_cast<double>(c.t_c_ns - t_0_ns_) / kNsPerSec;
    Screen(traj, cov, cov_matched, rt, arm, following, has_previous, t_c_prev_ns, c, stats);
  }
  n_cands_ = static_cast<int>(count);
  stats.nlp.n_lattice = static_cast<std::uint16_t>(n_cands_);
  if (n_cands_ == 0) {
    return none(NlpReject::kNoCandidate);
  }

  // ── Rank, and how many solves the budget holds ────────────────────────────
  int n_ranked = 0;
  for (int i = 0; i < n_cands_; ++i) {
    if (cands_[U(i)].reject == NlpReject::kNotRanked) {
      ranked_[U(n_ranked++)] = i;
    }
  }
  stats.nlp.n_screened = static_cast<std::uint16_t>(n_ranked);
  std::sort(ranked_.begin(), ranked_.begin() + n_ranked, [this](int a, int b) {
    const CandidateRecord& ca = cands_[U(a)];
    const CandidateRecord& cb = cands_[U(b)];
    return ca.rank_key < cb.rank_key || (ca.rank_key == cb.rank_key && ca.index < cb.index);
  });
  for (int r = 0; r < n_ranked; ++r) {
    cands_[U(ranked_[U(r)])].rank = r;
  }
  stats.nlp.screen_ns = clock_() - t_start;
  const std::int64_t left_ns = budget_ns_ - stats.nlp.screen_ns;
  const std::int64_t by_budget = left_ns > 0 ? left_ns / solve_budget_ns_ : 0;
  const int n_solve =
      static_cast<int>(std::min<std::int64_t>({static_cast<std::int64_t>(params_.max_solves),
                                               static_cast<std::int64_t>(n_ranked), by_budget}));
  // The TIME budget cut the solves — not `max_solves`, which is a cap the
  // wake was configured with.
  stats.budget_hit =
      by_budget < std::min<std::int64_t>(params_.max_solves, static_cast<std::int64_t>(n_ranked));

  // The test seam's order, when it is a permutation of exactly these solves.
  bool permuted = eval_order_n_ == n_solve && n_solve > 0;
  for (int i = 0; permuted && i < n_solve; ++i) {
    permuted = eval_order_[U(i)] >= 0 && eval_order_[U(i)] < n_solve;
    for (int j = 0; permuted && j < i; ++j) {
      permuted = eval_order_[U(j)] != eval_order_[U(i)];
    }
  }

  // ── Solves ────────────────────────────────────────────────────────────────
  for (int i = 0; i < n_solve; ++i) {
    const int r = permuted ? eval_order_[U(i)] : i;
    CandidateRecord& c = cands_[U(ranked_[U(r)])];
    Solve(traj, cov, cov_matched, rt, c, fresh_[U(r)]);
    ++stats.nlp.n_solved;
    stats.nlp.solve_ns_max = std::max(stats.nlp.solve_ns_max, c.solve_ns);
  }

  // ── The choice ────────────────────────────────────────────────────────────
  int best_rank = -1;
  for (int r = 0; r < n_solve; ++r) {
    const CandidateRecord& c = cands_[U(ranked_[U(r)])];
    if (c.reject != NlpReject::kNone) {
      continue;
    }
    ++stats.nlp.n_valid;
    if (best_rank < 0) {
      best_rank = r;
      continue;
    }
    const CandidateRecord& b = cands_[U(ranked_[U(best_rank)])];
    if (c.phi < b.phi || (c.phi == b.phi && c.index < b.index)) {
      best_rank = r;
    }
  }
  stats.n_pass = stats.nlp.n_valid;
  NlpReject farthest = NlpReject::kNone;
  for (int i = 0; i < n_cands_; ++i) {
    const NlpReject why = cands_[U(i)].reject;
    ++stats.nlp.rejects[static_cast<std::size_t>(why)];
    if (why != NlpReject::kNone &&
        static_cast<std::uint8_t>(why) > static_cast<std::uint8_t>(farthest)) {
      farthest = why;
    }
  }
  if (best_rank >= 0) {
    solution_.seg = fresh_[U(best_rank)].seg;
    solution_.source_seq = fresh_[U(best_rank)].source_seq;
    solution_.cost_reference = fresh_[U(best_rank)].cost_reference;
    solution_.cost_stop = fresh_[U(best_rank)].cost_stop;
    solution_.feasible = fresh_[U(best_rank)].feasible;
    solution_.converged = fresh_[U(best_rank)].converged;
    solution_valid_ = true;
  }
  // This wake's solutions become the next wake's memory — only now, so that no
  // solve of this wake started from another's.
  for (int r = 0; r < n_solve; ++r) {
    if (fresh_[U(r)].valid) {
      memory_[MemorySlot(fresh_[U(r)].index)] = fresh_[U(r)];
    }
  }
  if (best_rank < 0) {
    return none(farthest);
  }

  // ── The plan ──────────────────────────────────────────────────────────────
  const CandidateRecord& c = cands_[U(ranked_[U(best_rank)])];
  // The ball at the catch instant, sampled again: the shared core input may
  // hold another candidate's by now.
  int hint = 0;
  const BallNodeSample ball = SampleBallNode(traj, &cov, cov_matched, BallTime{c.t_c_ns}, hint);
  const double speed = ball.v.norm();
  plan.t_c_ns = c.t_c_ns;
  plan.t_cmd_ns = std::isfinite(constants_.t_close_e2e)
                      ? c.t_c_ns - SecondsToNs(constants_.t_close_e2e)
                      : c.t_c_ns;
  plan.p_c = {ball.p.x(), ball.p.y(), ball.p.z()};
  plan.v_c = {ball.v.x(), ball.v.y(), ball.v.z()};
  plan.a_d = {-ball.v.x() / speed, -ball.v.y() / speed, -ball.v.z() / speed};
  plan.nv = nv_;
  const SegmentSnapshot& seg = solution_.seg;
  for (int d = 0; d < nv_; ++d) {
    plan.q_star[U(d)] = seg.q[U(c.n_pre * kMaxSegmentNv + d)];
  }
  plan.w5 = c.w5;
  plan.w6 = c.w6;
  plan.dp_impact = params_.core.m_ball * c.c_catch;
  plan.score = c.phi;
  const double sigma = SigmaMaxOf(ball);
  plan.sigma_c = std::isfinite(sigma) ? sigma : 0.0;
  plan.sigma_l = plan.sigma_c;
  plan.reason = PlanReason::kNone;
  plan.valid = true;

  stats.nlp.reason = NlpReject::kNone;
  stats.nlp.chosen_index = c.index;
  stats.nlp.chosen_n_pre = c.n_pre;
  stats.nlp.chosen_iterations = c.iterations;
  stats.nlp.chosen_source_seq = c.source_seq;
  stats.nlp.chosen_x0_clamped = c.x0_clamped;
  stats.nlp.chosen_lead_s = c.lead_s;
  stats.nlp.chosen_wait_s = static_cast<double>(c.wait_ns) / kNsPerSec;
  stats.nlp.chosen_phi = c.phi;
  stats.nlp.chosen_j_reference = c.j_reference;
  stats.nlp.chosen_j_stop = c.j_stop;
  stats.nlp.chosen_j_time = c.j_time;
  stats.nlp.chosen_j_switch = c.j_switch;
  stats.chosen_score = c.phi;
  stats.chosen_lead_s = static_cast<double>(c.t_c_ns - now.ns) / kNsPerSec;
  stats.publish = true;
  stats.decision = !following                   ? SwitchDecision::kNoCurrent
                   : c.t_c_ns == rt.plan_t_c_ns ? SwitchDecision::kRefreshed
                                                : SwitchDecision::kReplaced;
  chose_before_ = true;
  last_chosen_t_c_ns_ = c.t_c_ns;
  stats.search_ns = clock_() - t_start;
  return plan;
}

}  // namespace rtc::catching
