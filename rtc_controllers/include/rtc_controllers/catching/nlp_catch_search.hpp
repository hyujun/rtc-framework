// ── The NLP catch search: one arm problem per candidate catch instant ─────────
// (dynamic_catching E1-F14, #740; reference
//  docs/dynamic_catching/ref/ball_catching_inverse_dynamics_mpc.md §9.5, §11,
//  §12.7, §17.11, §17.12)
//
// The second CatchSearch (catch_search.hpp). Where GridCatchSearch scores a
// candidate catch point with closed-form gates, this one SOLVES the arm motion
// that catches there — MpcDockingSegmentCore, the same problem the
// mpc_docking segment planner solves — and chooses by that motion's cost. It
// knows no ROS, no controller and no robot. Nothing installs it yet: the
// planner cycle that would call it after the RT has adopted a plan is E1-F16.
//
// ── One wake (Plan) ───────────────────────────────────────────────────────────
//  1. WHERE THE RESULT CAN START. t_0 = now + T_arm + budget + start_lead: the
//     earliest instant a segment published by this wake can be read by the RT
//     — if the wake keeps to its budget (8). "budget" is `budget_s`, or the
//     cap the caller passes when that is smaller (CatchSearch::Plan): a wake
//     that has more to run behind the search is given less.
//  2. CANDIDATES are the instants of a lattice of ABSOLUTE times,
//     t_c(i) = t_ref + i·h, that lie in (t_0, t_0 + T_max]. t_ref is the first
//     searching wake's `now` of the trial, so a candidate keeps its index from
//     wake to wake (nlp_catch_screening.hpp).
//  3. THE ARM GRID of a candidate is the core's, anchored at t_c:
//     n_pre = ⌊(t_c − t_0)/Δ_a⌋ pre-catch intervals, node 0 at
//     t_s = t_c − n_pre·Δ_a ∈ [t_0, t_0 + Δ_a). One core per n_pre is built at
//     Configure; a wake solves each candidate on the core of its n_pre.
//  4. THE START STATE x_0 is where the arm will be at t_s:
//       – the RT follows no plan: its command, at rest (a command that moves
//         is no start for a first plan — the wake's reason is not_at_rest);
//       – the RT follows a plan: the segment it reports (ReportedSegments —
//         the pending one from its node 0 on, else the followed one,
//         SourceSegmentAt), evaluated at t_s by the RT's own sampler. Never
//         the measured state, never the command extrapolated.
//     A state outside the core's box is projected into it and marked.
//  5. SCREENING, every candidate, in lattice order — each a NECESSARY
//     condition, the first that fails is the candidate's reason:
//     lead (S1) → the ball at every node of its grid → workspace → covariance
//     → a source segment → catch-pose IK and its catchability gate → joint
//     reach (S4) → closing-speed window (S3).
//  6. RANK the survivors by J_time + J_switch + a proxy on the IK pose — the
//     candidate of the followed plan's cell first, whatever its key (8) — and
//     solve the best L, L = min(max_solves, ⌊(budget − screening)/solve_budget⌋).
//     Each solve has its own deadline (its share), not the wake's — except
//     under a caller's cap (below), where it is also no later than the wake's
//     end. L is fixed
//     before the first solve and no solve is skipped for what another took:
//     which candidates are solved must not depend on the order they run in.
//  7. A SOLVE'S START POINT comes from the PREVIOUS wake's memory only (so the
//     order solves run in cannot matter): the same candidate's solution with
//     the nodes that have passed dropped; else the nearest remembered
//     candidate's solution stretched in time about t_s; else the IK pose.
//     What is remembered is every iterate a solve ENDED ON — valid, cut by
//     its deadline, unconverged, or stationary with a row still violated (the
//     point the solve stays nearest to when the prediction sharpens). What is
//     not: the iterate of a solve the solver itself failed on (a QP that did
//     not converge, a non-finite evaluation). That candidate's memory is
//     dropped, so its next solve starts elsewhere.
//  8. VALID is: solved inside its share, every hard row within tolerance,
//     converged — and still readable from its node 0. The core reads its
//     deadline between iterations only, so solves overrun their shares and a
//     wake can end past its budget; a candidate whose node 0 lies less than
//     that overrun after t_0 is refused (`deadline`) like one that ran past
//     its own share. Φ = J⋆ + J_time + J_switch with J⋆ the solve's cost UP TO
//     THE CATCH NODE (the stop part's terms are recorded, not chosen on). The
//     smallest Φ wins, the smaller index on a tie.
//  9. THE PLAN is that candidate; Solution() is its arm trajectory in the form
//     a segment planner publishes. The plan's q_star is the SOLVED catch pose
//     and its w5 / w6 are evaluated there (the IK pose is where the solve was
//     aimed, not where it ended).
//
// ── The catch instant inside a cell (continuous_tc) ───────────────────────────
// The lattice is 20 – 40 ms coarse and the best catch instant is rarely on it.
// With `continuous_tc` each solved candidate is solved TWICE:
//   ① on its lattice instant, as above (the same core, the same start);
//   ② when ① ended on an iterate, again with the catch instant free inside
//     the candidate's CELL — δt_c ∈ [−⌊h/2⌋, h − ⌊h/2⌋), further cut to the
//     instants the lattice search itself would take as a candidate: not
//     nearer than the minimum lead, not past the window, and — each side of
//     the lattice instant, found on the prediction the plan reads — as far
//     as the ball there passes what the screening asks of the ball alone
//     (inside the prediction, moving, inside the catch box, a covariance
//     with `chance`). Only the interval that ends at the catch node
//     stretches: node 0, the start state and every pre-catch node are ①'s,
//     so the candidate keeps its index, its grid and its place in the next
//     wake's memory.
//   ② starts from the candidate's own continuous solution of the previous
//   wake when there is one, else from ①'s solution at δt_c = 0. It is the
//   candidate's solution when it is valid by ①'s own rule (in its share,
//   every hard row, converged) and the ball at the catch instant it ended at
//   passes the same checks (the cut above finds an END of the instants that
//   do; one that fails between two that pass is caught here).
//   Otherwise the candidate keeps ①'s solution and ①'s verdict, and the
//   record says that ② ran, why it was not taken and what it ended on. What
//   ② ended on is next wake's start for it either way — and when its search
//   of the catch instant did not finish that is the best point it had
//   solved, not the one it stopped at (`continuous_settled`).
// The core is given the outer cost's two terms as functions of δt_c, so ②
// minimises J⋆ + J_stop + J_time + J_switch over the trajectory AND the
// instant; Φ is then evaluated at the instant it ended at, by the same
// functions as ①'s. A valid ② is taken even if its Φ is the larger one (Φ
// leaves out J_stop, which the core does not): both are recorded.
// The covariance ② is solved with is the cell's lattice instant's — the one
// the screening passed; the plan's sigma_c is the catch instant's own.
//
// The cell of the plan the RT follows is the one exception: its catch instant
// is the PLAN's (not the lattice's, not free) — the candidate is screened
// there and solved once, by ②'s core with δt_c held. Choosing it is
// `refreshed`; a catch instant elsewhere in the same cell does not exist as a
// candidate, so a plan is never replaced by its own cell.
//
// ── After adoption ────────────────────────────────────────────────────────────
// "The plan the RT follows" is a plan of THIS track when the segment it
// reports carries the track generation of the prediction being searched
// (a segment carries its plan's track). For such a plan, and only then:
//  • its CELL — the half-open lattice cell its catch instant is in
//    (NlpCellOf) — names one candidate, and that candidate is ranked first
//    when it passes screening: a wake whose budget holds one solve re-solves
//    what the arm is doing, and can say `refreshed`;
//  • the cell of the FIRST such plan of the track is remembered (until
//    ResetTrial, a new track, or a wake on which the RT follows no such
//    plan), and with `follow_window` ≥ 0 the candidates farther than that
//    many cells from it are removed before anything else is checked
//    (`follow_window`) — the catch instant cannot wander off the one the
//    approach was started for by more than the window, however many times
//    the plan is replaced. The followed plan's own cell is never removed;
//  • SearchStats::nlp records that cell and where the chosen candidate is
//    from it, with or without a window.
//
// ── What a wake reports about the plan the RT follows ─────────────────────────
// SearchStats::decision: kNoCurrent (the RT follows none); kRefreshed (the
// chosen catch instant is the followed one, to the nanosecond); kReplaced (it
// is another); kHeldNoCandidate with `publish` false (nothing is valid — the
// RT keeps what it has). The switch term's anchor is the followed plan's catch
// instant while there is one, else the instant this search last chose.
//
// ── Contracts ─────────────────────────────────────────────────────────────────
//  • Configure is non-RT: it builds every core, input, result, the IK and the
//    per-candidate memory, and runs each solver once (the first solve of a
//    ProxQP object is its slowest and allocates the most).
//  • A solution with δt_c ≠ 0 is a segment whose catch interval has its own
//    length (SegmentSnapshot::dt_catch_ns) and whose t_c_ns is the plan's.
//  • Plan, Monitor, NotePublished, ResetTrial are noexcept and allocate
//    nothing of their own — pinned with a C-level malloc gate over a whole
//    Plan. The QP solvers inside the IK and the cores DO allocate in their
//    own update()/solve() (#654): on hardware this search must not run on a
//    SCHED_FIFO planner thread until that is closed.
//  • `now` is the axis of every instant here (the lattice, t_0, the RT
//    state's age). The clock handed to Configure measures DURATIONS only — the
//    budget and the solves' deadlines — so a test can step it.
//  • Joint vectors are in MODEL order inside (the cores' and the IK's), in the
//    arm's DEVICE order at the interface (PlannerRtState, PlanSnapshot,
//    SegmentSnapshot).
#pragma once

#include "rtc_controllers/catching/ball_node_samples.hpp"
#include "rtc_controllers/catching/catch_pose_ik.hpp"
#include "rtc_controllers/catching/catch_search.hpp"
#include "rtc_controllers/catching/mpc_docking_relative_state.hpp"
#include "rtc_controllers/catching/mpc_docking_segment_core.hpp"
#include "rtc_controllers/catching/nlp_catch_screening.hpp"
#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/planner_params.hpp"  // CatchBox
#include "rtc_controllers/catching/search_stats.hpp"
#include "rtc_controllers/catching/time_types.hpp"
#include "rtc_controllers/catching/traj_ingress.hpp"
#include "rtc_controllers/catching/trajectory.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace rtc::catching {

/// Most solves one wake may run (the bound on `max_solves`).
inline constexpr int kNlpMaxSolves = 32;
/// Most lattice instants one wake may look at (the bound on `cand_capacity`).
inline constexpr int kNlpMaxCandidates = 256;

/// The arm model the search plans in (configure time, owned by the binding).
struct NlpCatchSearchModel {
  /// The hand-locked arm in pinocchio velocity order — what every core is
  /// built on (MpcDockingSegmentCore::Init).
  std::shared_ptr<const pinocchio::Model> arm;
  /// The planner thread's own handle on the same sub-model, for the catch-pose
  /// IK. No joint reorder may be installed on it (CatchPoseIk refuses one).
  rtc_urdf_bridge::RtModelHandle* handle{nullptr};
  pinocchio::FrameIndex catch_frame{0};  ///< in both; its +z is the approach axis
  int nv{0};
  /// `device_of_model[j]` = the arm device's index of model joint j.
  std::array<int, kMaxPlanNv> device_of_model{};
};

/// Constants that live outside the search's own keys, resolved by the binding.
struct NlpCatchSearchConstants {
  double t_arm_s{0.0};       ///< `joint_cmd.lag.T_arm` [s]
  double control_dt{0.002};  ///< the RT period [s]
  /// `robot.hand.T_close_lead` [s] — what the hand sequencer subtracts from the
  /// catch instant; NaN = unknown, the plan's close instant is then the catch
  /// instant.
  double t_close_lead{std::numeric_limits<double>::quiet_NaN()};
};

/// Tuning. Every field is validated by Configure.
struct NlpCatchSearchParams {
  // ── Candidates (§11.1) ──
  double cand_dt{0.02};    ///< lattice spacing h [s], > 0
  double t_lead_min{0.2};  ///< T_min: smallest t_c − t_0 [s]; ≥ n_pre_min·dt_pre
  double t_max{0.6};       ///< T_max: largest t_c − t_0 [s]; ≤ n_pre_max·dt_pre
  int cand_capacity{64};   ///< candidates a wake can hold; ≥ ⌊t_max/cand_dt⌋ + 1
  CatchBox catch_box{};    ///< the catch point must lie inside (must be set)
  /// The IK seed, DEVICE order, `wait_pose_n` = nv entries — the wait pose,
  /// the same for every candidate and every wake (the RT's adopted wait pose
  /// replaces it when it reports one).
  std::array<double, kMaxPlanNv> wait_pose{};
  int wait_pose_n{0};

  // ── The arm grid: the keys the mpc_docking segment planner shares ──
  int n_pre_min{2};      ///< fewest pre-catch intervals a candidate may have (≥ 1)
  int n_pre_max{6};      ///< most; one core is built for each count in between
  double dt_pre{0.1};    ///< Δ_a [s]
  int n_stop{7};         ///< stop intervals after the catch node (≥ 3)
  double dt_stop{0.05};  ///< Δ_s [s]
  /// Move blocks of the stop part (Σ = n_stop, at least 3 blocks). Each
  /// pre-catch interval is a block of its own.
  int n_stop_blocks{4};
  std::array<int, kMaxSegmentNodes> stop_block_sizes{1, 1, 2, 3};

  // ── Budget (§11.2) ──
  double budget_s{0.025};       ///< one wake's whole budget [s]
  double solve_budget_s{0.01};  ///< one candidate's share [s], ≤ budget_s
  /// Margin between the budget's end and t_0 [s], ≥ 0. It has to hold what
  /// the search cannot measure: the time from `now` to Plan's entry, and from
  /// Plan's return to the RT reading the published segment.
  double start_lead_s{0.004};
  int max_solves{4};  ///< 1..kNlpMaxSolves

  // ── Outer cost (§9.5) and the rank proxy (§11.3) ──
  double w_time{0.0};        ///< w_T ≥ 0
  double w_switch{0.0};      ///< w_sw ≥ 0
  double t_ref_s{0.5};       ///< T_ref > 0 [s] — the cost's time scale
  double rank_w_q{1.0};      ///< weight of ‖q^c − q_0‖² in the rank [1/rad²], ≥ 0
  double rank_w_manip{0.0};  ///< weight of ψ_m(q^c) in the rank, ≥ 0

  // ── The catch instant inside a candidate's cell ──
  /// After a candidate's fixed-grid solve, solve it again with the catch
  /// instant free inside its lattice cell (MpcDockingSegmentCore's
  /// catch_time_variable; the step limit and the two penalties are `core`'s).
  /// A candidate then takes two shares of the budget.
  bool continuous_tc{false};

  // ── After the RT has adopted a plan ──
  /// While the RT follows a plan of the track being searched, a wake looks
  /// only at the candidates within this many lattice cells either side of the
  /// cell of the FIRST plan it followed on that track (the cell the followed
  /// plan is in now is never removed). Negative = no window.
  int follow_window{-1};

  // ── The RT's report ──
  double rest_tol{1e-3};            ///< max |q̇_cmd| that counts as "at rest" [rad/s]
  double rt_state_age_max_s{0.05};  ///< oldest report a wake plans from [s]

  /// The arm problem. `n_pre`, `dt_pre`, `n_stop`, `dt_stop`, `n_blocks` and
  /// `block_sizes` are overwritten per core from the keys above; everything
  /// else is every core's.
  MpcDockingSegmentCoreParams core;
  /// Joint limits, model order — the cores' box (the caller's margins already
  /// applied). The reach condition reads the same numbers back from a core.
  MpcDockingSegmentCoreLimits limits;
};

class NlpCatchSearch final : public CatchSearch {
 public:
  NlpCatchSearch();
  ~NlpCatchSearch() override;
  // Owns its cores and scratch; a copy would be a second set of solvers.
  NlpCatchSearch(const NlpCatchSearch&) = delete;
  NlpCatchSearch& operator=(const NlpCatchSearch&) = delete;

  /// @brief Build everything a wake uses (non-RT). A second call rebuilds from
  ///        scratch.
  /// @param clock steady clock for the budget and the solves' deadlines [ns]
  /// @param[out] error why it failed, when it did
  /// @return false, and unconfigured, on an unusable binding, a parameter out
  ///         of range, a grid a core refuses, or a warm-up solve that never
  ///         reached its QP.
  [[nodiscard]] bool Configure(const NlpCatchSearchModel& model,
                               const NlpCatchSearchConstants& constants,
                               const NlpCatchSearchParams& params, const CatchPoseIkOptions& ik,
                               ClockFn clock, std::string* error = nullptr);

  [[nodiscard]] bool Configured() const noexcept { return configured_; }

  /// Replace the steady clock the budget and the solves' deadlines are read
  /// on (non-RT; the planner thread is not running). A null `clock` is
  /// ignored. Every core it owns takes the same clock, so a deadline set from
  /// this search's clock is checked on that axis.
  void SetClock(ClockFn clock) noexcept override {
    if (clock == nullptr) {
      return;
    }
    clock_ = clock;
    for (auto& core : cores_) {
      core->SetClock(clock);
    }
    for (auto& core : cores_tc_) {
      core->SetClock(clock);
    }
  }

  /// One search (RT-safe apart from the QP solvers — header note).
  /// `budget_cap_ns` > 0 makes the wake's budget min(`budget_s`, the cap):
  /// t_0, the number of solves and the overrun are counted on it, and no
  /// solve's deadline is later than the wake's start plus it.
  [[nodiscard]] PlanSnapshot Plan(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                                  bool cov_matched, const PlannerRtState& rt,
                                  const ReportedSegments& arm, NowReal now,
                                  std::int64_t budget_cap_ns, SearchStats& stats) noexcept override;

  /// The chosen candidate's arm trajectory, or nullptr when the last Plan
  /// chose none. Good until the next Plan or ResetTrial.
  [[nodiscard]] const CatchSolution* Solution() const noexcept override {
    return solution_valid_ ? &solution_ : nullptr;
  }

  /// monitorOnly: σ_ℓ = √λ_max(Σ_p) of the newest prediction at the followed
  /// plan's catch instant; NaN when it cannot be read.
  void Monitor(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov, bool cov_matched,
               const PlannerRtState& rt, SearchStats& stats) const noexcept override;

  /// Nothing to remember: the switch term's anchor is what the RT reports
  /// following, or what this search last CHOSE — published or not.
  void NotePublished(const PlanSnapshot& plan) noexcept override;

  /// Forget the lattice anchor, every remembered solution and the last choice.
  void ResetTrial() noexcept override;

  // ── The last wake, candidate by candidate (tests, diagnostics, offline maps) ─

  /// How a candidate's solve was started.
  enum class Start : std::uint8_t {
    kNone = 0,       ///< not solved
    kIkTarget,       ///< no memory: the IK pose as the catch-node target
    kSameCandidate,  ///< its own previous solution, the passed nodes dropped
    kNeighbour,      ///< another candidate's previous solution, stretched in time
    kFixedSolution,  ///< (continuous solve) this wake's fixed-grid solution of the candidate
  };

  struct CandidateRecord {
    std::int64_t index{0};     ///< lattice index
    std::int64_t t_c_ns{0};    ///< catch instant: t_hat_ns + delta_ns
    std::int64_t t_hat_ns{0};  ///< the cell's lattice instant (the arm grid's anchor)
    std::int64_t delta_ns{0};  ///< 0 unless a continuous solution, or a pinned candidate
    /// The cell of the plan the RT follows, with `continuous_tc`: its catch
    /// instant is that plan's and does not move.
    bool pinned{false};
    std::int64_t t_s_ns{0};   ///< node 0's instant
    std::int64_t wait_ns{0};  ///< t_s − t_0
    double lead_s{0.0};       ///< T = t_c − t_0 [s]
    int n_pre{0};
    NlpReject reject{NlpReject::kNone};
    // ── Screening ──
    CatchPoseReason ik_reason{CatchPoseReason::kNone};
    bool ik_run{false};
    NlpReachLimit reach_limit{NlpReachLimit::kNone};
    int reach_joint{-1};
    double speed_lo{0.0}, speed_hi{0.0};  ///< the closing-speed window [m/s]
    double rank_key{0.0};
    int rank{-1};  ///< position in the rank (0 = best), −1 when not screened in
    /// The start state, MODEL order: where the arm is at t_s.
    std::array<double, kMaxSegmentNv> q0{}, qd0{}, qdd0{};
    std::array<double, kMaxSegmentNv> q_ik{};  ///< the IK pose, MODEL order (when it converged)
    double w5{0.0}, w6{0.0};                   ///< the IK's manipulability at THAT pose
    std::uint32_t source_seq{0};               ///< the segment x_0 was read from; 0 = rest
    bool x0_clamped{false};
    // ── The solve ──
    Start start{Start::kNone};
    std::int64_t start_index{0};  ///< the candidate whose memory started it
    MpcDockingReason core_reason{MpcDockingReason::kNone};
    DockingRowGroup worst_group{DockingRowGroup::kTorque};  ///< read on hard_row / chance
    double worst_violation{0.0};
    bool solved{false};  ///< the core returned an iterate
    bool feasible{false};
    bool converged{false};
    int iterations{0};
    std::int64_t solve_ns{0};
    /// Valid by its own solve, refused because the wake ended too far past its
    /// budget for the RT to read this candidate's node 0 (`reject` is kDeadline).
    bool late{false};
    /// Where the solve's time went, as the core measured it on its own clock
    /// [µs]: before the first iterate (start-point projection or the
    /// initialisation QP), in the QP solver, and how many QPs that was.
    double start_us{0.0}, qp_us{0.0};
    int qp_solves{0};
    double c_catch{0.0};  ///< closing speed at the catch node of the solution [m/s]
    double j_reference{0.0}, j_stop{0.0}, j_time{0.0}, j_switch{0.0};
    double phi{0.0};  ///< Φ, meaningful when `reject` is kNone
    // ── The continuous solve (`continuous_tc`). The fields above are of the
    // solve the candidate USES; these are of the other one, and of the pair. ──
    bool continuous_run{false};   ///< it ran
    bool continuous_used{false};  ///< the candidate's solution is the continuous one
    /// Why it is not, when it ran and is not: the verdict a fixed-grid solve
    /// would have had (deadline, hard_row, chance, unconverged, …).
    NlpReject continuous_reject{NlpReject::kNone};
    Start continuous_start{Start::kNone};
    MpcDockingReason continuous_reason{MpcDockingReason::kNone};
    int continuous_iterations{0};
    int continuous_moves{0};  ///< moves of the catch instant
    /// What ② returned solves the problem at the instant it is at (the core's
    /// catch_time_settled) — also when its search of the instant did not end.
    bool continuous_settled{false};
    std::int64_t continuous_solve_ns{0};
    std::int64_t delta_lo_ns{0}, delta_hi_ns{0};  ///< δt_c's box as given to the core
    /// The fixed-grid solve of a candidate that uses its continuous solution.
    NlpReject fixed_reject{NlpReject::kNone};
    int fixed_iterations{0};
    std::int64_t fixed_solve_ns{0};
    double fixed_phi{0.0};  ///< its Φ (meaningful when fixed_reject is kNone)
    /// What the core minimises — J_ref + J_stop + its two terms in the catch
    /// instant — at each solve's end (0 for a solve that did not run).
    double objective_fixed{0.0}, objective_continuous{0.0};
  };

  /// The last Plan's candidates, in lattice order.
  [[nodiscard]] std::span<const CandidateRecord> Candidates() const noexcept {
    return {cands_.data(), static_cast<std::size_t>(n_cands_)};
  }

  /// t_0 and the lattice anchor of the last Plan [ns].
  [[nodiscard]] std::int64_t LastStartInstantNs() const noexcept { return t_0_ns_; }

  /// The budget the last Plan ran on [ns]: `budget_s`, or the caller's cap
  /// when that was smaller.
  [[nodiscard]] std::int64_t LastWakeBudgetNs() const noexcept { return wake_budget_ns_; }

  /// The latest deadline a solve of the last Plan was given [ns, the search's
  /// clock]; 0 when it ran none (tests).
  [[nodiscard]] std::int64_t LastSolveDeadlineMaxNsForTesting() const noexcept {
    return solve_deadline_max_ns_;
  }

  [[nodiscard]] std::int64_t LatticeAnchorNs() const noexcept { return t_ref_ns_; }

  /// The solution remembered for lattice index `index` — what the NEXT wake
  /// starts that candidate from — or nullptr.
  [[nodiscard]] const SegmentSnapshot* RememberedSolution(std::int64_t index) const noexcept;

  /// The CONTINUOUS solution remembered for lattice index `index` (its
  /// t_c_ns is off the lattice by the δt_c it ended at), or nullptr.
  [[nodiscard]] const SegmentSnapshot* RememberedContinuous(std::int64_t index) const noexcept;

  /// The core for `n_pre` pre-catch intervals, or nullptr: outside
  /// n_pre_min..n_pre_max, or not configured.
  [[nodiscard]] const MpcDockingSegmentCore* Core(int n_pre) const noexcept;

  /// Its twin with the catch instant a variable; nullptr without
  /// `continuous_tc`.
  [[nodiscard]] const MpcDockingSegmentCore* ContinuousCore(int n_pre) const noexcept;

  // ── Test seams ──────────────────────────────────────────────────────────────

  /// Solve the ranked candidates in another order: `order[i]` is the rank
  /// position solved i-th. Applied only on a wake whose number of solves
  /// equals `order.size()`; it changes neither which candidates are solved
  /// nor the screening. Empty restores the rank order.
  void SetEvaluationOrderForTesting(std::span<const int> order) noexcept;

  /// Called right before and right after every QP-solving call — the IK's
  /// Solve and a core's Solve — so that an allocation gate over a whole Plan
  /// can be suspended exactly there (#654).
  using SolverHook = void (*)(bool begin, void* user) noexcept;

  void SetSolverHookForTesting(SolverHook hook, void* user) noexcept {
    solver_hook_ = hook;
    solver_user_ = user;
  }

  /// Forwarded to every core (MpcDockingSegmentCore::SetStageHook).
  void SetCoreStageHookForTesting(MpcDockingSegmentCore::StageHook hook, void* user) noexcept;

 private:
  /// One remembered solution: the candidate it is of and what its solve ended
  /// on (the trajectory in DEVICE order, as it would be published).
  struct Memory {
    bool valid{false};
    /// A wake's own entry only: the solver failed on this candidate, so what
    /// was remembered for it is dropped when the wake ends.
    bool forget{false};
    std::int64_t index{0};
    CatchSolution sol{};
  };

  [[nodiscard]] static std::size_t U(int i) noexcept { return static_cast<std::size_t>(i); }

  [[nodiscard]] int CoreSlot(int n_pre) const noexcept { return n_pre - params_.n_pre_min; }

  [[nodiscard]] std::size_t MemorySlot(std::int64_t index) const noexcept;
  [[nodiscard]] const Memory* Remembered(std::int64_t index) const noexcept;
  [[nodiscard]] const Memory* NearestRemembered(std::int64_t index) const noexcept;
  /// The nearest remembered solution a candidate without one of its own can
  /// start from (of this arm, begun by the candidate's node 0, catching
  /// after it, with this stop part), or nullptr.
  [[nodiscard]] const Memory* UsableNeighbour(const CandidateRecord& c) const noexcept;

  /// What the ball at a catch instant refuses the instant for — the checks of
  /// the screening that read the ball alone, in its order (kNone: none).
  [[nodiscard]] NlpReject CatchBallVerdict(const BallNodeSample& ball) const noexcept;
  /// One end of δt_c's box: `end_ns` when the ball at t̂ + end_ns passes
  /// CatchBallVerdict, else the farthest instant toward it that does, by
  /// halving from δt_c = 0 (which the screening passed).
  [[nodiscard]] std::int64_t CatchOffsetEnd(const TrajectorySnapshot& traj,
                                            const CovarianceSnapshot& cov, bool cov_matched,
                                            std::int64_t t_hat_ns,
                                            std::int64_t end_ns) const noexcept;
  /// x_0 of the candidate at node-0 instant t_s into `c` (model order).
  [[nodiscard]] bool StartState(const PlannerRtState& rt, const ReportedSegments& arm,
                                bool following, CandidateRecord& c) noexcept;
  /// Every necessary condition, in order; sets c.reject.
  void Screen(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov, bool cov_matched,
              const PlannerRtState& rt, const ReportedSegments& arm, bool following,
              bool has_previous, std::int64_t t_c_prev_ns, CandidateRecord& c,
              SearchStats& stats) noexcept;
  /// Fill the candidate's core input and solve it; sets c.reject and Φ.
  void Solve(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov, bool cov_matched,
             const PlannerRtState& rt, CandidateRecord& c, Memory& out) noexcept;
  /// The start trajectory of `c` from a remembered solution of the SAME
  /// candidate (the nodes that have passed dropped), or from another one's
  /// stretched in time about node 0; false when it cannot be read.
  void StartFromOwn(const SegmentSnapshot& mine, const CandidateRecord& c,
                    MpcDockingSegmentCoreInput& in) const noexcept;
  [[nodiscard]] bool StartFromNeighbour(const SegmentSnapshot& prev, const CandidateRecord& c,
                                        MpcDockingSegmentCoreInput& in) const noexcept;
  /// The verdict on an iterate a core returned, as a reason (kNone: valid).
  [[nodiscard]] NlpReject Verdict(const MpcDockingSegmentCoreResult& res, bool past_deadline,
                                  double& worst, int& worst_group) const noexcept;
  /// The numbers of the solve a candidate stands on, from the core's result.
  static void RecordSolve(const MpcDockingSegmentCoreResult& res, double worst, int worst_group,
                          CandidateRecord& c) noexcept;
  /// The solve with the catch instant free in the candidate's cell — or held
  /// at the followed plan's (`c.pinned`). `fixed` is this wake's fixed-grid
  /// solution of the candidate, or nullptr.
  void SolveContinuous(const TrajectorySnapshot& traj, const CovarianceSnapshot& cov,
                       bool cov_matched, const PlannerRtState& rt, bool has_previous,
                       std::int64_t t_c_prev_ns, const Memory* fixed, CandidateRecord& c,
                       Memory& out) noexcept;
  /// w5 / w6 at the catch pose `q_dev` (DEVICE order); 0 where the
  /// factorisation is not valid, as CatchPoseIk reports them.
  void CatchPoseManipulability(const std::array<double, kMaxPlanNv>& q_dev, double& w5,
                               double& w6) noexcept;
  /// The core's result as the segment a planner publishes (device order).
  void Pack(const PlannerRtState& rt, std::uint64_t track_generation, const CandidateRecord& c,
            const MpcDockingSegmentCoreResult& r, std::int64_t delta_ns,
            SegmentSnapshot& out) const noexcept;
  [[nodiscard]] bool WarmUp(std::string* error);

  bool configured_{false};
  NlpCatchSearchModel model_{};
  NlpCatchSearchConstants constants_{};
  NlpCatchSearchParams params_{};
  CatchPoseIkOptions ik_options_{};
  ClockFn clock_{nullptr};
  int nv_{0};

  // Integer forms of the keys (the arithmetic of the lattice and the grid).
  std::int64_t h_ns_{0}, dt_pre_ns_{0}, dt_stop_ns_{0};
  std::int64_t t_lead_min_ns_{0}, t_max_ns_{0};
  std::int64_t t_arm_ns_{0}, control_dt_ns_{0};
  std::int64_t budget_ns_{0}, solve_budget_ns_{0}, start_lead_ns_{0}, age_max_ns_{0};
  // This wake's budget — budget_ns_, or the caller's cap when that is smaller —
  // and, under a cap, the instant no solve's deadline may pass (0 = none): a
  // solve's deadline is its own share from its own start otherwise.
  std::int64_t wake_budget_ns_{0};
  std::int64_t solve_deadline_cap_ns_{0};
  std::int64_t solve_deadline_max_ns_{0};
  // The deadline of a solve that starts at `start_ns`.
  [[nodiscard]] std::int64_t SolveDeadlineNs(std::int64_t start_ns) noexcept;

  // The arm problem, one per n_pre (index n_pre − n_pre_min).
  std::vector<std::unique_ptr<MpcDockingSegmentCore>> cores_;
  std::vector<MpcDockingSegmentCoreInput> inputs_;
  std::vector<MpcDockingSegmentCoreResult> results_;
  // Their twins with the catch instant a variable (`continuous_tc`; else empty).
  std::vector<std::unique_ptr<MpcDockingSegmentCore>> cores_tc_;
  std::vector<MpcDockingSegmentCoreInput> inputs_tc_;
  std::vector<MpcDockingSegmentCoreResult> results_tc_;
  CatchPoseIk ik_;
  // The cores' model (armature added) for the screening's own kinematics.
  pinocchio::Model arm_model_;
  pinocchio::Data arm_data_;
  DockingFrameKinematics kin_;
  DockingImpactWork impact_work_;
  DockingImpact impact_;
  DockingManipulabilityWork manip_work_;
  Eigen::VectorXd manip_d_q_, manip_grad_;
  Eigen::VectorXd q_model_, v_zero_, seed_, seed_yaml_;
  bool timing_on_{false}, impact_on_{false};
  double sigma_max_{0.0};

  // Candidates of the last wake and the order they are solved in.
  std::vector<CandidateRecord> cands_;
  int n_cands_{0};
  std::vector<int> ranked_;
  std::array<int, kNlpMaxSolves> eval_order_{};
  int eval_order_n_{0};

  // Per-candidate memory. A wake READS `memory_` (the previous wakes') and
  // WRITES `fresh_`; the two are merged when the wake ends.
  std::vector<Memory> memory_;
  std::vector<Memory> fresh_;
  // The same pair for the continuous solutions, by lattice index too: a
  // continuous solution's catch instant is not a lattice instant, its cell is.
  std::vector<Memory> memory_tc_;
  std::vector<Memory> fresh_tc_;

  // The lattice and the last choice.
  bool anchor_set_{false};
  std::int64_t t_ref_ns_{0};
  std::int64_t t_0_ns_{0};
  bool track_known_{false};
  std::uint64_t track_generation_{0};
  bool chose_before_{false};
  std::int64_t last_chosen_t_c_ns_{0};
  // The first plan the RT followed on this track: its cell and catch instant.
  bool follow_anchor_set_{false};
  std::int64_t follow_anchor_index_{0};
  std::int64_t follow_first_t_c_ns_{0};

  CatchSolution solution_{};
  bool solution_valid_{false};

  SolverHook solver_hook_{nullptr};
  void* solver_user_{nullptr};
};

/// @brief The PlanReason a reason of the NLP search is published as.
///
/// PlanSnapshot's reason set is frozen, so several reasons share one code; the
/// exact one is in SearchStats::nlp and NlpCatchSearch::Candidates().
[[nodiscard]] constexpr PlanReason NlpPlanReason(NlpReject r) noexcept {
  switch (r) {
    case NlpReject::kNone:
      return PlanReason::kNone;
    case NlpReject::kFollowWindow:  // outside what this wake looks at, as too near or too far is
    case NlpReject::kLeadShort:
    case NlpReject::kNoCandidate:
      return PlanReason::kHorizonShort;
    case NlpReject::kBallInvalid:
      return PlanReason::kInputNonFinite;
    case NlpReject::kWorkspace:
      return PlanReason::kStoppingDistance;  // the workspace gate's code, as the grid search's
    case NlpReject::kCovariance:
    case NlpReject::kChance:
      return PlanReason::kUncertainty;
    case NlpReject::kIk:
      return PlanReason::kIkFailed;
    case NlpReject::kManipulability:
      return PlanReason::kManipulability;
    case NlpReject::kReach:
      return PlanReason::kReachTime;
    case NlpReject::kSpeedWindow:
      return PlanReason::kGammaWindow;
    case NlpReject::kNotRanked:
    case NlpReject::kDeadline:
      return PlanReason::kBudgetExceeded;
    case NlpReject::kSolverRejected:
    case NlpReject::kNotAtRest:
    case NlpReject::kNoSource:
    case NlpReject::kRtInvalid:
      return PlanReason::kLimitsInvalid;
    case NlpReject::kHardRow:
    case NlpReject::kUnconverged:
      return PlanReason::kRollout;
  }
  return PlanReason::kNone;
}

}  // namespace rtc::catching
