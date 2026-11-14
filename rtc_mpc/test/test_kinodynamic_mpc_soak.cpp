/// @file test_kinodynamic_mpc_soak.cpp
/// @brief Headless soak of the kinodynamic MPC: `HandlerMPCThread` driving the
///        locomotion phase manager and the kinodynamic handler on its own
///        thread, publishing into an `MPCSolutionManager`, with this test's
///        thread in the role of the RT side (it writes the state every 2 ms).
///
/// Model and configuration: the public G1 floating body with the shipped
/// walk (test_utils/shipped_walk_fixture.hpp — copies of the g1_loco
/// sections) and the OCP fixture (test_utils/kinodynamic_ocp_fixture.hpp).
/// Twelve combinations: three scenarios × MPC period {60 Hz, 1/dt} ×
/// integrator {semi-implicit Euler, RK2}, OMP_NUM_THREADS taken from the
/// environment (ctest leaves it unset: set it to 1 for a measurement).
///
///  - stance_static: the stance gait, the state the stand pose at rest;
///  - walk_rollout: stance until the first online tick, then the walk at
///    0.5 m/s; the state is the reference the manager's sampler
///    (`SampleReference`) gives at the current time — a lower bound: no model
///    mismatch, no delay but the plan's own;
///  - walk_disturbed: as walk_rollout, plus base velocity (σ 0.05 m/s) and
///    attitude (σ 0.01 rad) noise on every state written (seed 20261114),
///    and every 10th online tick held 1.5 periods after its solve (an
///    overrun: the next tick shifts by two or more nodes).
///
/// Metrics per combination (printed, and written as
/// `test_kinodynamic_mpc_soak.summary.{csv,json}` next to the gtest XML, or
/// into `RTC_MPC_SOAK_OUT_DIR`): solve time p50 / p99 / max of the online
/// ticks (the handler's `Solve`, wall clock), the online ticks whose solve
/// took longer than the period, the thread's overruns (online and after a
/// standing tick, apart), deadline misses, withheld solves, standing-solve
/// ticks, the node shifts, C-level allocations per solve, the heap (glibc
/// `mallinfo2` arena + hblkhd, sampled on the MPC thread at a tick start once
/// a second), the primal residual (whole horizon and consumed segment) and
/// the friction / CoP margins (every 10th online tick) of the window's first
/// and second half.
///
/// PASS conditions: no failed solve (exception, divergence, refusal); the
/// heap did not grow from the end of the warm-up to the end (the leak gate;
/// judged only when the window after the warm-up is at least 30 s — the
/// smoke reports it); the allocation counts do not grow late in the run —
/// for each (node shift, iterations) group of online warm ticks with at
/// least 30 ticks in an armed run, the median of its last third (time order)
/// is not above the maximum of its first third. The counts themselves vary
/// with the line search's trial count, which the group does not fix, so
/// their spread is reported, and the smoke and smaller groups report the
/// comparison only. Timing is reported, never a failure: it is judged on the
/// control PCs.
///
/// Default (colcon test): a 3 s window per combination, warm-up 1 s. Armed
/// with `RTC_RUN_MPC_SOAK=1`: `RTC_MPC_SOAK_SECONDS` (default 600) per
/// combination, warm-up `RTC_MPC_SOAK_WARMUP_SECONDS` (default 60). Other
/// knobs: `RTC_MPC_SOAK_SLOTS` (comma-separated layout slots — the MPC thread's,
/// then one per OpenMP worker; slot meaning, translated as the launch layout
/// does) with `RTC_MPC_SOAK_THREADS` (solver threads, default the slot count or
/// 1; a slot list of another length is refused), `RTC_MPC_SOAK_CPU` (the
/// MPC thread's slot alone; −1 none — the first entry of the slot list),
/// `RTC_MPC_SOAK_FIFO_PRIORITY` (SCHED_FIFO at that priority; 0 =
/// SCHED_OTHER — FIFO needs a pinned MPC core), `RTC_MPC_SOAK_WRITER_CPU`
/// (pin this test's state / command writer; by default, under FIFO, the first
/// core of its affinity other than the MPC core: the phase manager's seqlock
/// reader on a FIFO core would spin against a writer preempted on the same
/// core, so the two are refused on one core), `RTC_MPC_SOAK_MAX_ITERS` / `RTC_MPC_SOAK_AL_ITERS`
/// (the online budget, default 10 / 1), `RTC_MPC_SOAK_DEADLINE_MS` / `RTC_MPC_SOAK_PRIM_TOL` (the
/// publish rules, default the shipped g1_loco values). The summary records the scheduling the MPC
/// thread actually runs with (read back on it), not the requested one. An armed run exceeds the
/// verification hook's test bound: run it by hand through
/// `repo_scripts/scripts/with_verify_hold.sh` — the command and the control-PC recipe are in
/// docs/testing.md.
///
/// Not in the mimalloc LD_PRELOAD list: this binary defines the C allocator
/// (rtc_base/testing/malloc_gate.hpp) to count allocations — on the thread
/// that runs the solve only (the gate is thread-local), so a multi-threaded
/// run's allocation counts are the MPC thread's, not its workers'. The MPC
/// thread sets its own OpenMP team size, so OMP_NUM_THREADS no longer matters;
/// OMP_PROC_BIND / OMP_PLACES make a multi-threaded run refuse its pool.

#include "rtc_base/testing/malloc_gate.hpp"  // one TU per binary: defines malloc & co.
#include "rtc_mpc/handler/kinodynamic_mpc.hpp"
#include "rtc_mpc/manager/mpc_solution_manager.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/model/wb_layout.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp.hpp"
#include "rtc_mpc/ocp/kinodynamic_ocp_config.hpp"
#include "rtc_mpc/phase/locomotion_phase_manager.hpp"
#include "rtc_mpc/thread/handler_mpc_thread.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/kinodynamic_ocp_fixture.hpp"
#include "test_utils/loco_reference_fixture.hpp"
#include "test_utils/shipped_walk_fixture.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Geometry>
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include <gtest/gtest.h>
#include <pthread.h>
#include <sched.h>
#include <sys/utsname.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace {

namespace tu = rtc::mpc::test_utils;
using rtc::mpc::MPCSolution;
using rtc::mpc::MPCSolveError;
using rtc::mpc::MPCSolverConfig;
using rtc::mpc::MPCStateSnapshot;
using rtc::mpc::PhaseContext;
using Clock = std::chrono::steady_clock;

/// The publish rules of the g1_loco profile (its mpc.yaml `solver:`), the
/// soak's defaults: provisional values the control-PC soak decides.
constexpr double kShippedDeadlineMs = 35.0;
constexpr double kShippedPrimTol = 0.05;

constexpr double kWriterPeriodS = 0.002;  ///< the RT side's state writes
constexpr double kStandingTimeoutS = 180.0;

// ── Settings ────────────────────────────────────────────────────────────────

double EnvDouble(const char* name, double fallback) {
  const char* v = std::getenv(name);
  if (v == nullptr || *v == '\0') {
    return fallback;
  }
  char* end = nullptr;
  const double d = std::strtod(v, &end);
  return end != v && std::isfinite(d) ? d : fallback;
}

struct Settings {
  bool armed{false};
  double seconds{3.0};
  double warmup{1.0};
  int cpu{-1};             ///< the MPC thread's slot (the first of `slots`)
  std::vector<int> slots;  ///< MPC thread then workers; empty = only `cpu`
  int threads{1};          ///< solver threads
  bool slots_malformed{false};
  int fifo_priority{0};
  int writer_cpu{-1};  ///< −1: chosen under FIFO, else not pinned
  int max_iters{10};
  int al_iters{1};
  double deadline_ms{kShippedDeadlineMs};
  double prim_tol{kShippedPrimTol};
};

Settings ReadSettings() {
  Settings s;
  const char* armed = std::getenv("RTC_RUN_MPC_SOAK");
  s.armed = armed != nullptr && std::string(armed) == "1";
  s.seconds = EnvDouble("RTC_MPC_SOAK_SECONDS", s.armed ? 600.0 : 3.0);
  s.warmup = EnvDouble("RTC_MPC_SOAK_WARMUP_SECONDS", s.armed ? 60.0 : 1.0);
  s.cpu = static_cast<int>(EnvDouble("RTC_MPC_SOAK_CPU", -1.0));
  if (const char* list = std::getenv("RTC_MPC_SOAK_SLOTS"); list != nullptr && *list != '\0') {
    std::stringstream ss(list);
    for (std::string item; std::getline(ss, item, ',');) {
      char* end = nullptr;
      const long v = std::strtol(item.c_str(), &end, 10);
      if (end == item.c_str() || *end != '\0' || v < 0) {
        s.slots_malformed = true;
        break;
      }
      s.slots.push_back(static_cast<int>(v));
    }
    if (!s.slots.empty()) {
      s.cpu = s.slots.front();
    }
  }
  s.threads = static_cast<int>(EnvDouble(
      "RTC_MPC_SOAK_THREADS", s.slots.empty() ? 1.0 : static_cast<double>(s.slots.size())));
  s.fifo_priority = static_cast<int>(EnvDouble("RTC_MPC_SOAK_FIFO_PRIORITY", 0.0));
  s.writer_cpu = static_cast<int>(EnvDouble("RTC_MPC_SOAK_WRITER_CPU", -1.0));
  s.max_iters = static_cast<int>(EnvDouble("RTC_MPC_SOAK_MAX_ITERS", 10.0));
  s.al_iters = static_cast<int>(EnvDouble("RTC_MPC_SOAK_AL_ITERS", 1.0));
  s.deadline_ms = EnvDouble("RTC_MPC_SOAK_DEADLINE_MS", kShippedDeadlineMs);
  s.prim_tol = EnvDouble("RTC_MPC_SOAK_PRIM_TOL", kShippedPrimTol);
  return s;
}

/// The directory of the gtest XML (`--gtest_output=xml:<path>`), else the
/// working directory; `RTC_MPC_SOAK_OUT_DIR` wins.
std::filesystem::path OutDir() {
  if (const char* d = std::getenv("RTC_MPC_SOAK_OUT_DIR"); d != nullptr && *d != '\0') {
    return d;
  }
  const std::string flag = GTEST_FLAG_GET(output);
  const std::string prefix = "xml:";
  if (flag.rfind(prefix, 0) == 0) {
    const std::filesystem::path p(flag.substr(prefix.size()));
    if (p.has_parent_path()) {
      return p.parent_path();
    }
  }
  return std::filesystem::current_path();
}

/// glibc's heap footprint: arena (sbrk and thread heaps) + mmapped blocks.
double HeapBytes() {
  const struct mallinfo2 mi = mallinfo2();
  return static_cast<double>(mi.arena) + static_cast<double>(mi.hblkhd);
}

// ── Per tick, on the MPC thread ─────────────────────────────────────────────

struct TickRecord {
  double t{0.0};  ///< [s] since the run's start, at the handler's Solve entry
  int shift{-1};  ///< grid index moved since the last good tick; −1 a cold seed
  int iterations{0};
  bool ok{false};
  bool converged{false};
  bool initial{false};
  bool injected{false};
  double solve_ms{0.0};
  std::size_t mallocs{0};
  double prim{0.0};
  double consumed{0.0};
  double friction_margin{std::numeric_limits<double>::quiet_NaN()};
  double cop_margin{std::numeric_limits<double>::quiet_NaN()};
  double heap{-1.0};   ///< [B] sampled at this tick's start, else −1
  int error{0};        ///< the handler's MPCSolveError
  int phase_error{0};  ///< the phase manager's LocomotionPhaseError on a failed tick
};

/// The kinodynamic handler as the thread sees it, recording each tick into a
/// pre-sized array (read after the thread is joined).
class SoakHandler final : public rtc::mpc::MPCHandlerBase {
 public:
  SoakHandler(std::unique_ptr<rtc::mpc::KinodynamicMPC> inner, std::size_t capacity,
              Clock::time_point t0, std::uint64_t period_ns, bool inject_overruns)
      : inner_(std::move(inner)),
        records_(capacity),
        t0_(t0),
        period_ns_(period_ns),
        inject_(inject_overruns) {
    xs_.assign(static_cast<std::size_t>(tu::kKinoNodes + 1), Eigen::VectorXd());
    us_.assign(static_cast<std::size_t>(tu::kKinoNodes), Eigen::VectorXd());
  }

  rtc::mpc::MPCInitError Init(const MPCSolverConfig& cfg, const rtc::mpc::RobotModelHandler& model,
                              const rtc::mpc::OCPLimits& limits,
                              const PhaseContext& ctx) noexcept override {
    const auto e = inner_->Init(cfg, model, limits, ctx);
    for (auto& x : xs_) {
      x = Eigen::VectorXd::Zero(model.nq() + model.nv());
    }
    for (auto& u : us_) {
      u = Eigen::VectorXd::Zero(model.nu());
    }
    return e;
  }

  MPCSolveError Solve(const PhaseContext& ctx, const MPCStateSnapshot& state,
                      MPCSolution& out) noexcept override {
    const Clock::time_point entry = Clock::now();
    if (!sched_read_) {
      ReadScheduling();
    }
    TickRecord r;
    r.t = std::chrono::duration<double>(entry - t0_).count();
    if (entry >= next_heap_) {
      r.heap = HeapBytes();
      next_heap_ = entry + std::chrono::seconds(1);
    }
    MPCSolveError e = MPCSolveError::kNoError;
    {
      const rtc::testing::ScopedMallocGate gate;
      e = inner_->Solve(ctx, state, out);
      r.mallocs = gate.count();
    }
    r.solve_ms = std::chrono::duration<double, std::milli>(Clock::now() - entry).count();
    const std::int64_t d = ctx.node_index - last_index_;
    r.shift = have_last_ && d >= 0 && d < tu::kKinoNodes ? static_cast<int>(d) : -1;
    r.ok = e == MPCSolveError::kNoError;
    r.error = static_cast<int>(e);
    if (!r.ok && phase_ != nullptr) {
      r.phase_error = static_cast<int>(phase_->LastUpdateError());
    }
    have_last_ = r.ok;
    last_index_ = ctx.node_index;
    if (r.ok) {
      r.iterations = out.iterations;
      r.converged = out.converged;
      r.initial = out.initial_solve;
      r.prim = out.residuals.prim_inf;
      r.consumed = rtc::mpc::HandlerMPCThread::ConsumedPrimInf(out, period_ns_);
      if (!out.initial_solve) {
        ++online_;
        online_seen_.store(true, std::memory_order_release);
        if (online_ % 10 == 0) {
          Margins(out, r);
          if (inject_) {
            r.injected = true;
            std::this_thread::sleep_for(std::chrono::nanoseconds(3 * period_ns_ / 2));
          }
        }
      }
    }
    if (n_ < records_.size()) {
      records_[n_++] = r;
    }
    return e;
  }

  [[nodiscard]] bool Initialised() const noexcept override { return inner_->Initialised(); }

  [[nodiscard]] const MPCSolverConfig& solver_config() const noexcept override {
    return inner_->solver_config();
  }

  [[nodiscard]] bool InitialSolvePending() const noexcept override {
    return inner_->InitialSolvePending();
  }

  [[nodiscard]] std::string_view ocp_type() const noexcept override { return inner_->ocp_type(); }

  [[nodiscard]] int horizon_length() const noexcept override { return inner_->horizon_length(); }

  [[nodiscard]] int nq() const noexcept override { return inner_->nq(); }

  [[nodiscard]] int nv() const noexcept override { return inner_->nv(); }

  [[nodiscard]] int nu() const noexcept override { return inner_->nu(); }

  [[nodiscard]] int n_contact_vars() const noexcept override { return inner_->n_contact_vars(); }

  void SeedWarmStart(const MPCSolution& prev) noexcept override { inner_->SeedWarmStart(prev); }

  /// After the thread is joined: the scheduling it ran with, read on it.
  [[nodiscard]] const std::string& AppliedPolicy() const noexcept { return applied_policy_; }

  [[nodiscard]] int AppliedPriority() const noexcept { return applied_priority_; }

  [[nodiscard]] const std::string& AppliedCpus() const noexcept { return applied_cpus_; }

  /// Before the thread starts: the phase manager whose error a failed tick records.
  void WatchPhaseManager(const rtc::mpc::LocomotionPhaseManager* m) noexcept { phase_ = m; }

  /// Any thread: an online (not standing) tick solved.
  [[nodiscard]] bool OnlineSeen() const noexcept {
    return online_seen_.load(std::memory_order_acquire);
  }

  /// After the thread is joined.
  [[nodiscard]] std::vector<TickRecord> Records() const {
    return {records_.begin(), records_.begin() + static_cast<std::ptrdiff_t>(n_)};
  }

 private:
  /// On the MPC thread, once (the first tick; allocates — outside the gate).
  void ReadScheduling() noexcept {
    sched_read_ = true;
    int policy = 0;
    sched_param param{};
    if (pthread_getschedparam(pthread_self(), &policy, &param) == 0) {
      applied_policy_ = policy == SCHED_FIFO ? "SCHED_FIFO"
                        : policy == SCHED_RR ? "SCHED_RR"
                                             : "SCHED_OTHER";
      applied_priority_ = param.sched_priority;
    }
    cpu_set_t set;
    CPU_ZERO(&set);
    if (pthread_getaffinity_np(pthread_self(), sizeof(set), &set) == 0) {
      applied_cpus_.clear();
      int n = 0;
      for (int c = 0; c < CPU_SETSIZE; ++c) {
        if (CPU_ISSET(c, &set)) {
          applied_cpus_ += (n++ > 0 ? ";" : "") + std::to_string(c);
        }
      }
    }
  }

  /// The friction / CoP margins of the plan (the OCP's own residuals, its
  /// current modes and targets). Allocation-free.
  void Margins(const MPCSolution& out, TickRecord& r) noexcept {
    const int nq = out.nq;
    const int nv = out.nv;
    for (std::size_t k = 0; k < xs_.size(); ++k) {
      for (int i = 0; i < nq; ++i) {
        xs_[k][i] = out.q_traj[k][static_cast<std::size_t>(i)];
      }
      for (int i = 0; i < nv; ++i) {
        xs_[k][nq + i] = out.v_traj[k][static_cast<std::size_t>(i)];
      }
    }
    for (std::size_t k = 0; k < us_.size(); ++k) {
      for (int i = 0; i < out.nu; ++i) {
        us_[k][i] = out.u_traj[k][static_cast<std::size_t>(i)];
      }
    }
    rtc::mpc::ConstraintResidualReport report;
    if (!inner_->ocp().ConstraintResiduals(xs_, us_, report)) {
      return;
    }
    r.friction_margin = std::numeric_limits<double>::infinity();
    r.cop_margin = std::numeric_limits<double>::infinity();
    for (std::size_t c = 0; c < 2; ++c) {
      r.friction_margin = std::min(r.friction_margin, report.min_friction_margin[c]);
      r.cop_margin = std::min(r.cop_margin, report.min_cop_margin[c]);
    }
  }

  std::unique_ptr<rtc::mpc::KinodynamicMPC> inner_;
  const rtc::mpc::LocomotionPhaseManager* phase_{nullptr};
  std::vector<TickRecord> records_;
  std::size_t n_{0};
  Clock::time_point t0_;
  Clock::time_point next_heap_{};
  std::uint64_t period_ns_;
  bool inject_;
  bool have_last_{false};
  std::int64_t last_index_{0};
  std::uint64_t online_{0};
  std::atomic<bool> online_seen_{false};
  bool sched_read_{false};
  std::string applied_policy_{"unknown"};
  int applied_priority_{-1};
  std::string applied_cpus_{"unknown"};
  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> us_;
};

// ── Summary ─────────────────────────────────────────────────────────────────

struct Summary {
  std::string name;
  double window_s{0.0};
  std::size_t online_ticks{0};
  double p50_ms{0.0};
  double p99_ms{0.0};
  double max_ms{0.0};
  std::size_t over_period{0};
  std::uint64_t online_overruns{0};
  std::uint64_t standing_overruns{0};
  std::uint64_t deadline_misses{0};
  std::uint64_t withheld{0};
  std::uint64_t cold_withheld{0};
  std::uint64_t initial_ticks{0};
  std::uint64_t failed{0};
  std::uint64_t published{0};
  std::size_t shift0{0};
  std::size_t shift1{0};
  std::size_t shift2{0};
  std::size_t shift3plus{0};
  std::size_t cold{0};
  std::size_t injected{0};
  std::size_t malloc_groups{0};
  std::size_t malloc_mismatched_groups{0};
  std::size_t malloc_drifting_groups{0};
  std::size_t malloc_judged_groups{0};
  std::size_t malloc_unjudged_late_growth{0};  ///< groups too small / smoke, comparison > max
  std::string applied_policy;
  int applied_priority{-1};
  std::string applied_cpus;
  int writer_cpu{-1};
  std::string omp_threads;
  int solver_threads{1};
  std::string worker_slots{"-"};     ///< requested, ';'-joined
  std::string worker_cpus{"-"};      ///< read back per worker, ';'-joined
  std::string worker_policies{"-"};  ///< read back per worker, ';'-joined
  std::string pool_state{"-"};
  std::string host;  ///< HostFacts(), ','-free
  double mallocs_per_iteration{0.0};
  double heap_warm{0.0};
  double heap_end{0.0};
  double heap_growth{0.0};
  double heap_excursion{0.0};
  bool heap_judged{false};
  double prim_med_first{0.0};
  double prim_med_second{0.0};
  double consumed_med{0.0};
  double consumed_max{0.0};
  double friction_first{0.0};
  double friction_second{0.0};
  double cop_first{0.0};
  double cop_second{0.0};
};

double Percentile(std::vector<double> v, double q) {
  if (v.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(v.begin(), v.end());
  const auto i = static_cast<std::size_t>(std::lround(q * static_cast<double>(v.size() - 1)));
  return v[std::min(i, v.size() - 1)];
}

double Min(const std::vector<double>& v) {
  double m = std::numeric_limits<double>::quiet_NaN();
  for (const double x : v) {
    if (std::isfinite(x) && !(x >= m)) {
      m = x;
    }
  }
  return m;
}

std::vector<Summary>& Summaries() {
  static std::vector<Summary> s;
  return s;
}

/// Host facts recorded with every row (one value each, no ','; lists use ';').
constexpr const char* kHostFactKeys =
    "commit,build_type,kernel,physical_core_slots,sched_rt_runtime_us,fair_server_runtime_ns,"
    "omp_wait_policy,gomp_spincount,omp_proc_bind,shield_marker";

/// The CSV columns that hold text; everything else is a number in the JSON.
[[nodiscard]] bool IsTextColumn(const std::string& key) {
  static const char* const kText[] = {
      "applied_policy",     "applied_cpus",    "omp_num_threads",     "worker_slots",
      "worker_cpus",        "worker_policies", "pool_state",          "commit",
      "build_type",         "kernel",          "physical_core_slots", "fair_server_runtime_ns",
      "omp_wait_policy",    "gomp_spincount",  "omp_proc_bind",       "shield_marker",
      "sched_rt_runtime_us"};
  for (const char* t : kText) {
    if (key == t) {
      return true;
    }
  }
  return false;
}

/// `s` with ',' and newlines replaced, so a free-text fact stays one CSV cell.
[[nodiscard]] std::string Cell(std::string s) {
  for (char& c : s) {
    if (c == ',' || c == '\n' || c == '\r') {
      c = ';';
    }
  }
  while (!s.empty() && s.back() == ';') {
    s.pop_back();
  }
  return s.empty() ? "-" : s;
}

[[nodiscard]] std::string ReadFirstLine(const std::string& path) {
  std::ifstream in(path);
  std::string line;
  if (!in || !std::getline(in, line)) {
    return "na";
  }
  return line;
}

[[nodiscard]] std::string EnvOr(const char* key, const char* fallback) {
  const char* v = std::getenv(key);
  return v != nullptr && *v != '\0' ? v : fallback;
}

/// The run's host facts, in kHostFactKeys order. `mpc_cpu` is the MPC thread's
/// logical CPU (its fair-server runtime; root-only debugfs, else "na").
[[nodiscard]] std::string HostFacts(int mpc_cpu) {
  utsname u{};
  const std::string kernel = uname(&u) == 0 ? u.release : "na";
  std::string slots;
  for (const int c : rtc::GetCpuTopology().physical_core_slots) {
    slots += (slots.empty() ? "" : ";") + std::to_string(c);
  }
  const std::string fair = mpc_cpu >= 0 ? ReadFirstLine("/sys/kernel/debug/sched/fair_server/cpu" +
                                                        std::to_string(mpc_cpu) + "/runtime")
                                        : "na";
#ifdef RTC_SOAK_BUILD_TYPE
  const std::string build_type = RTC_SOAK_BUILD_TYPE;
#else
  const std::string build_type = "unknown";
#endif
  const std::vector<std::string> facts = {
      EnvOr("RTC_MPC_SOAK_COMMIT", "unknown"),
      build_type,
      kernel,
      slots,
      ReadFirstLine("/proc/sys/kernel/sched_rt_runtime_us"),
      fair,
      EnvOr("OMP_WAIT_POLICY", "unset"),
      EnvOr("GOMP_SPINCOUNT", "unset"),
      EnvOr("OMP_PROC_BIND", "unset"),
      ReadFirstLine(EnvOr("RTC_SHIELD_MARKER_FILE", "/tmp/cpu_shield_mode"))};
  std::string out;
  for (std::size_t i = 0; i < facts.size(); ++i) {
    out += (i > 0 ? "," : "") + Cell(facts[i]);
  }
  return out;
}

/// The process's `mpc_worker_<i>` threads, by name: their CPUs and policies,
/// ';'-joined in worker order. Read from this (writer) thread through /proc
/// while the pool is alive.
void ReadWorkers(std::string& cpus, std::string& policies) {
  std::map<std::string, std::pair<std::string, std::string>> found;
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator("/proc/self/task", ec)) {
    const std::string comm = ReadFirstLine(entry.path().string() + "/comm");
    if (comm.rfind("mpc_worker_", 0) != 0) {
      continue;
    }
    const auto tid = static_cast<pid_t>(std::stol(entry.path().filename().string()));
    cpu_set_t set;
    CPU_ZERO(&set);
    std::string mask;
    if (sched_getaffinity(tid, sizeof(set), &set) == 0) {
      for (int c = 0; c < CPU_SETSIZE; ++c) {
        if (CPU_ISSET(c, &set)) {
          mask += (mask.empty() ? "" : "+") + std::to_string(c);
        }
      }
    }
    const int policy = sched_getscheduler(tid);
    found[comm] = {mask.empty() ? "?" : mask, policy == SCHED_FIFO    ? "SCHED_FIFO"
                                              : policy == SCHED_RR    ? "SCHED_RR"
                                              : policy == SCHED_OTHER ? "SCHED_OTHER"
                                                                      : "?"};
  }
  cpus.clear();
  policies.clear();
  for (const auto& [name, v] : found) {
    cpus += (cpus.empty() ? "" : ";") + v.first;
    policies += (policies.empty() ? "" : ";") + v.second;
  }
  if (cpus.empty()) {
    cpus = "-";
    policies = "-";
  }
}

std::string CsvHeader() {
  return "combination,window_s,online_ticks,solve_p50_ms,solve_p99_ms,solve_max_ms,over_period,"
         "online_overruns,standing_overruns,deadline_misses,withheld,cold_withheld,initial_ticks,"
         "failed,"
         "published,shift0,shift1,shift2,shift3plus,cold,injected,malloc_groups,"
         "malloc_mismatched_groups,malloc_judged_groups,malloc_drifting_groups,"
         "malloc_unjudged_late_growth,mallocs_per_iteration,heap_warm_B,heap_"
         "end_B,heap_growth_B,"
         "heap_excursion_B,heap_judged,prim_med_first,prim_med_second,consumed_med,consumed_max,"
         "friction_margin_first,friction_margin_second,cop_margin_first,cop_margin_second,"
         "applied_policy,applied_priority,applied_cpus,writer_cpu,omp_num_threads,"
         "solver_threads,worker_slots,worker_cpus,worker_policies,pool_state," +
         std::string(kHostFactKeys);
}

std::string CsvRow(const Summary& s) {
  std::ostringstream o;
  o << s.name << ',' << s.window_s << ',' << s.online_ticks << ',' << s.p50_ms << ',' << s.p99_ms
    << ',' << s.max_ms << ',' << s.over_period << ',' << s.online_overruns << ','
    << s.standing_overruns << ',' << s.deadline_misses << ',' << s.withheld << ','
    << s.cold_withheld << ',' << s.initial_ticks << ',' << s.failed << ',' << s.published << ','
    << s.shift0 << ',' << s.shift1 << ',' << s.shift2 << ',' << s.shift3plus << ',' << s.cold << ','
    << s.injected << ',' << s.malloc_groups << ',' << s.malloc_mismatched_groups << ','
    << s.malloc_judged_groups << ',' << s.malloc_drifting_groups << ','
    << s.malloc_unjudged_late_growth << ',' << s.mallocs_per_iteration << ',' << s.heap_warm << ','
    << s.heap_end << ',' << s.heap_growth << ',' << s.heap_excursion << ','
    << (s.heap_judged ? 1 : 0) << ',' << s.prim_med_first << ',' << s.prim_med_second << ','
    << s.consumed_med << ',' << s.consumed_max << ',' << s.friction_first << ','
    << s.friction_second << ',' << s.cop_first << ',' << s.cop_second << ',' << s.applied_policy
    << ',' << s.applied_priority << ',' << s.applied_cpus << ',' << s.writer_cpu << ','
    << s.omp_threads << ',' << s.solver_threads << ',' << s.worker_slots << ',' << s.worker_cpus
    << ',' << s.worker_policies << ',' << s.pool_state << ',' << s.host;
  return o.str();
}

/// Writes the summaries once every combination ran.
class SummaryWriter : public ::testing::Environment {
 public:
  void TearDown() override {
    if (Summaries().empty()) {
      return;
    }
    const std::filesystem::path dir = OutDir();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    std::ofstream csv(dir / "test_kinodynamic_mpc_soak.summary.csv");
    csv << CsvHeader() << '\n';
    for (const Summary& s : Summaries()) {
      csv << CsvRow(s) << '\n';
    }
    std::ofstream json(dir / "test_kinodynamic_mpc_soak.summary.json");
    const Settings set = ReadSettings();
    json << "{\"armed\": " << (set.armed ? "true" : "false") << ", \"seconds\": " << set.seconds
         << ", \"warmup\": " << set.warmup << ", \"cpu\": " << set.cpu
         << ", \"fifo_priority\": " << set.fifo_priority << ", \"max_iters\": " << set.max_iters
         << ", \"al_iters\": " << set.al_iters << ", \"deadline_ms\": " << set.deadline_ms
         << ", \"prim_tol\": " << set.prim_tol << ", \"threads\": " << set.threads
         << ", \"slots\": \"" <<
        [&set] {
          std::string l;
          for (const int v : set.slots) {
            l += (l.empty() ? "" : ";") + std::to_string(v);
          }
          return l;
        }()
         << "\", \"combinations\": [";
    const std::string header = CsvHeader();
    for (std::size_t i = 0; i < Summaries().size(); ++i) {
      std::vector<std::string> keys;
      std::vector<std::string> values;
      std::stringstream hs(header);
      std::stringstream vs(CsvRow(Summaries()[i]));
      for (std::string k; std::getline(hs, k, ',');) {
        keys.push_back(k);
      }
      for (std::string v; std::getline(vs, v, ',');) {
        values.push_back(v);
      }
      json << (i > 0 ? ", " : "") << '{';
      for (std::size_t k = 0; k < keys.size() && k < values.size(); ++k) {
        const bool text = k == 0 || IsTextColumn(keys[k]) || values[k] == "nan" ||
                          values[k] == "inf" || values[k] == "-inf";
        json << (k > 0 ? ", " : "") << '"' << keys[k]
             << "\": " << (text ? "\"" + values[k] + "\"" : values[k]);
      }
      json << '}';
    }
    json << "]}\n";
    std::printf("[report] soak summaries: %s\n",
                (dir / "test_kinodynamic_mpc_soak.summary.*").c_str());
  }
};

[[maybe_unused]] ::testing::Environment* const kSummaryWriter =
    ::testing::AddGlobalTestEnvironment(new SummaryWriter);

// ── The soak ────────────────────────────────────────────────────────────────

enum class Scenario { kStanceStatic, kWalkRollout, kWalkDisturbed };

using Combo = std::tuple<Scenario, bool, const char*>;  // scenario, 60 Hz, integrator

std::string ScenarioName(Scenario s) {
  switch (s) {
    case Scenario::kStanceStatic:
      return "stance_static";
    case Scenario::kWalkRollout:
      return "walk_rollout";
    default:
      return "walk_disturbed";
  }
}

std::string ComboName(const Combo& c) {
  return ScenarioName(std::get<0>(c)) + (std::get<1>(c) ? "_60hz_" : "_1_over_dt_") +
         (std::string(std::get<2>(c)) == "rk2" ? "rk2" : "sie");
}

class KinodynamicMpcSoak : public ::testing::TestWithParam<Combo> {
 protected:
  void SetUp() override {
    if (!std::filesystem::exists(tu::kG1Urdf)) {
      GTEST_SKIP() << "G1 URDF not found at " << tu::kG1Urdf;
    }
    model_ = tu::BuildG1Floating();
    ASSERT_EQ(handler_.Init(model_, YAML::Load(tu::kG1SolesYaml)),
              rtc::mpc::RobotModelInitError::kNoError);
    stand_ = tu::G1Stand(model_);
  }

  pinocchio::Model model_;
  rtc::mpc::RobotModelHandler handler_;
  Eigen::VectorXd stand_;
};

TEST_P(KinodynamicMpcSoak, RunsWithoutFailureAllocationDriftOrHeapGrowth) {
  const Settings set = ReadSettings();
  const auto [scenario, sixty_hz, integrator] = GetParam();
  const std::string name = ComboName(GetParam());
  const double dt = tu::kKinoDt;
  const double hz = sixty_hz ? 60.0 : 1.0 / dt;
  const bool walk = scenario != Scenario::kStanceStatic;

  // Under SCHED_FIFO the MPC thread is pinned and this thread (the state and
  // command writer) runs on another core: the phase manager's seqlock read
  // would spin on the FIFO core against a writer preempted there.
  ASSERT_TRUE(set.fifo_priority <= 0 || set.cpu >= 0)
      << "RTC_MPC_SOAK_FIFO_PRIORITY needs RTC_MPC_SOAK_CPU (a pinned MPC core)";
  // The solver's OpenMP pool inherits the MPC thread's affinity: on one core
  // a team spins against itself (a 140 ms solve took 11 s). Every solver
  // thread therefore gets its own slot — the MPC thread sets its team size to
  // the thread count, whatever OMP_NUM_THREADS says — and a run with more
  // threads than listed slots is refused rather than left to share one.
  const char* omp = std::getenv("OMP_NUM_THREADS");
  const std::string omp_threads = omp != nullptr ? omp : "unset";
  ASSERT_FALSE(set.slots_malformed) << "RTC_MPC_SOAK_SLOTS must be comma-separated slots >= 0";
  ASSERT_GE(set.threads, 1) << "RTC_MPC_SOAK_THREADS must be at least 1";
  ASSERT_TRUE(set.slots.empty() || static_cast<int>(set.slots.size()) == set.threads)
      << "RTC_MPC_SOAK_SLOTS lists " << set.slots.size() << " slot(s) for " << set.threads
      << " solver thread(s); list one per thread, the MPC thread's first";
  ASSERT_TRUE(set.threads == 1 || static_cast<int>(set.slots.size()) == set.threads)
      << "a multi-threaded solve needs RTC_MPC_SOAK_SLOTS (one slot per solver thread)";
  ASSERT_LE(set.threads - 1, rtc::kMaxMpcWorkers) << "more workers than the layout can name";
  // The writer avoids every solver thread's CPU, not just the MPC thread's.
  std::vector<int> solver_cpus;
  for (const int slot : set.slots.empty() ? std::vector<int>{set.cpu} : set.slots) {
    if (slot >= 0) {
      solver_cpus.push_back(rtc::SlotToLogicalCpu(slot));
    }
  }
  const auto is_solver_cpu = [&solver_cpus](int c) {
    return std::find(solver_cpus.begin(), solver_cpus.end(), c) != solver_cpus.end();
  };
  cpu_set_t original_mask;
  CPU_ZERO(&original_mask);
  ASSERT_EQ(pthread_getaffinity_np(pthread_self(), sizeof(original_mask), &original_mask), 0);
  int writer_cpu = set.writer_cpu;
  if (writer_cpu < 0 && set.fifo_priority > 0) {
    for (int c = 0; c < CPU_SETSIZE && writer_cpu < 0; ++c) {
      if (!is_solver_cpu(c) && CPU_ISSET(c, &original_mask)) {
        writer_cpu = c;
      }
    }
    ASSERT_GE(writer_cpu, 0) << "no core for the writer besides the solver's";
  }
  ASSERT_TRUE(writer_cpu < 0 || !is_solver_cpu(writer_cpu))
      << "the writer must not share a solver thread's core (" << writer_cpu << ")";
  if (writer_cpu >= 0) {
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(writer_cpu, &mask);
    ASSERT_EQ(pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask), 0)
        << "cannot pin the writer to core " << writer_cpu;
  }
  const bool disturbed = scenario == Scenario::kWalkDisturbed;

  // The handler: the OCP fixture with the integrator, the online budget and
  // the publish rules of the settings, standing solve as shipped.
  rtc::mpc::KinodynamicOCPConfig ocp_cfg;
  ASSERT_EQ(
      rtc::mpc::LoadKinodynamicOCPConfig(tu::G1KinodynamicNode(integrator), handler_, ocp_cfg),
      rtc::mpc::KinodynamicOCPConfigError::kNoError);
  MPCSolverConfig solver;
  solver.prim_tol = 1e-4;
  solver.dual_tol = 1e-4;
  solver.mu_init = 1e-2;
  solver.max_iters = set.max_iters;
  solver.max_al_iters = 20;
  solver.max_initial_iters = std::max(300, set.max_iters);
  solver.max_online_al_iters = set.al_iters;
  solver.publish_deadline_ms = set.deadline_ms;
  solver.publish_prim_tol = set.prim_tol;
  solver.num_threads = set.threads;
  solver.worker_capacity = set.threads - 1;

  const Clock::time_point t0 = Clock::now();
  const auto period_ns = static_cast<std::uint64_t>(1e9 / hz + 0.5);
  const auto capacity =
      static_cast<std::size_t>((set.seconds + kStandingTimeoutS + 30.0) * hz) + 1000;
  auto soak = std::make_unique<SoakHandler>(std::make_unique<rtc::mpc::KinodynamicMPC>(ocp_cfg),
                                            capacity, t0, period_ns, disturbed);
  {
    const auto stance = tu::StanceHorizon(handler_, stand_);
    PhaseContext ctx;
    ctx.ocp_type = std::string(rtc::mpc::KinodynamicOCP::kType);
    ctx.cost_config.horizon_length = tu::kKinoNodes;
    ctx.cost_config.dt = dt;
    ctx.horizon = stance.get();
    ASSERT_EQ(soak->Init(solver, handler_, {}, ctx), rtc::mpc::MPCInitError::kNoError);
  }
  SoakHandler* soak_raw = soak.get();

  auto manager =
      std::make_unique<rtc::mpc::LocomotionPhaseManager>(handler_, tu::ShippedContactNames());
  ASSERT_EQ(manager->Load(tu::ShippedLocomotionYaml()), rtc::mpc::LocomotionPhaseError::kNoError);
  rtc::mpc::FilteredCommand cmd = tu::RegisteredCommand();
  cmd.v_x = 0.0;
  cmd.v_y = 0.0;
  cmd.omega_z = 0.0;
  cmd.base_height = 0.7925;
  cmd.sequence = 1;
  manager->SetCommand(cmd);
  manager->SetPolicyEnabled(false);
  rtc::mpc::LocomotionPhaseManager* manager_raw = manager.get();
  soak_raw->WatchPhaseManager(manager_raw);

  rtc::mpc::MPCSolutionManager mgr;
  YAML::Node mgr_cfg;
  mgr_cfg["enabled"] = true;
  mgr.Init(mgr_cfg, rtc::mpc::WbLayout::FromHandler(handler_));

  rtc::mpc::HandlerMPCThread thread;
  thread.Configure(handler_, std::move(soak), std::move(manager));
  rtc::mpc::MpcThreadLaunchConfig launch{};
  launch.main.cpu_core = set.cpu;
  launch.main.sched_policy = set.fifo_priority > 0 ? SCHED_FIFO : SCHED_OTHER;
  launch.main.sched_priority = set.fifo_priority;
  launch.main.name = "mpc_soak";
  launch.target_frequency_hz = hz;
  launch.solver_threads = set.threads;
  std::string worker_slots;
  for (int i = 1; i < set.threads; ++i) {
    launch.workers.slots[static_cast<std::size_t>(i - 1)] = set.slots[static_cast<std::size_t>(i)];
    worker_slots +=
        (worker_slots.empty() ? "" : ";") + std::to_string(set.slots[static_cast<std::size_t>(i)]);
  }
  launch.workers.count = set.threads - 1;
  thread.Init(mgr, launch);

  // The RT side: write the state every 2 ms.
  const int nq = model_.nq;
  const int nv = model_.nv;
  Eigen::VectorXd q = stand_;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd q_write(nq);
  Eigen::VectorXd v_write(nv);
  std::mt19937_64 rng(20261114);
  std::normal_distribution<double> noise(0.0, 1.0);
  rtc::mpc::WbcReference ref{};
  std::uint64_t last_plan_id = 0;
  std::uint64_t plans = 0;
  bool window_open = false;
  Clock::time_point window_start{};
  Clock::time_point next = Clock::now();
  thread.Start();
  for (;;) {
    const Clock::time_point now = Clock::now();
    const auto now_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
    // The RT side's sampler: the plan's node 0 at observation − offset.
    const bool sampled = mgr.SampleReference(q, v, now_ns, ref);
    if (sampled && ref.plan_id != last_plan_id) {
      last_plan_id = ref.plan_id;
      ++plans;
    }
    if (walk && sampled && ref.nq == nq && ref.nv == nv) {
      q = Eigen::Map<const Eigen::VectorXd>(ref.q_ref.data(), nq);
      v = Eigen::Map<const Eigen::VectorXd>(ref.v_ref.data(), nv);
    }
    q_write = q;
    v_write = v;
    if (disturbed) {
      for (int i = 0; i < 3; ++i) {
        v_write[i] += 0.05 * noise(rng);
      }
      const Eigen::Vector3d d(0.01 * noise(rng), 0.01 * noise(rng), 0.01 * noise(rng));
      const double angle = d.norm();
      const Eigen::Quaterniond tilt = angle > 0.0
                                          ? Eigen::Quaterniond(Eigen::AngleAxisd(angle, d / angle))
                                          : Eigen::Quaterniond::Identity();
      Eigen::Quaterniond quat(q[6], q[3], q[4], q[5]);
      quat = quat * tilt;
      quat.normalize();
      q_write[3] = quat.x();
      q_write[4] = quat.y();
      q_write[5] = quat.z();
      q_write[6] = quat.w();
    }
    mgr.WriteState(q_write, v_write, now_ns);

    if (!window_open) {
      if (soak_raw->OnlineSeen()) {
        if (walk) {
          manager_raw->RequestGait(2);
          cmd.v_x = 0.5;
          cmd.sequence = 2;
          manager_raw->SetCommand(cmd);
        }
        window_open = true;
        window_start = now;
      } else if (now - t0 > std::chrono::duration<double>(kStandingTimeoutS)) {
        break;
      }
    } else if (now - window_start >= std::chrono::duration<double>(set.seconds)) {
      break;
    }
    next +=
        std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(kWriterPeriodS));
    std::this_thread::sleep_until(next);
  }
  // The pool lives as long as the MPC thread: read it back before the join.
  std::string worker_cpus = "-";
  std::string worker_policies = "-";
  ReadWorkers(worker_cpus, worker_policies);
  const rtc::mpc::SolverPoolState pool_state = thread.PoolState();
  thread.StopAndJoin();
  pthread_setaffinity_np(pthread_self(), sizeof(original_mask), &original_mask);
  ASSERT_EQ(pool_state, rtc::mpc::SolverPoolState::kReady)
      << name << ": the MPC thread refused its solver pool (reason on stderr)";
  // The pool the loop entry pinned is the pool the run ended with: libgomp
  // retires and respawns pool threads when a region's team size changes, and
  // a respawned worker would sit unpinned on the MPC thread's core.
  std::string expected_cpus;
  for (int i = 1; i < set.threads; ++i) {
    expected_cpus += (expected_cpus.empty() ? "" : ";") +
                     std::to_string(rtc::SlotToLogicalCpu(set.slots[static_cast<std::size_t>(i)]));
  }
  EXPECT_EQ(worker_cpus, expected_cpus.empty() ? std::string("-") : expected_cpus)
      << name << ": the workers at the end of the run are not the pinned pool";
  ASSERT_TRUE(window_open) << name << ": no online tick within " << kStandingTimeoutS << " s";

  // ── Metrics over the window ──────────────────────────────────────────────
  const std::vector<TickRecord> records = soak_raw->Records();
  const double w0 = std::chrono::duration<double>(window_start - t0).count();
  Summary s;
  s.name = name;
  s.window_s = set.seconds;
  s.online_overruns = thread.OnlineOverrunCount();
  s.standing_overruns = thread.StandingOverrunCount();
  s.deadline_misses = thread.DeadlineMissCount();
  s.withheld = thread.WithheldCount();
  s.initial_ticks = thread.InitialSolveTicks();
  s.failed = thread.FailedSolves();
  s.cold_withheld = thread.ColdWithheldCount();
  s.applied_policy = soak_raw->AppliedPolicy();
  s.applied_priority = soak_raw->AppliedPriority();
  s.applied_cpus = soak_raw->AppliedCpus();
  s.writer_cpu = writer_cpu;
  s.omp_threads = omp_threads;
  s.solver_threads = set.threads;
  s.worker_slots = worker_slots.empty() ? "-" : worker_slots;
  s.worker_cpus = worker_cpus;
  s.worker_policies = worker_policies;
  s.pool_state = "ready";
  s.host = HostFacts(set.cpu >= 0 ? rtc::SlotToLogicalCpu(set.cpu) : -1);
  s.published = plans;
  std::vector<double> solve_ms;
  std::vector<double> prim_first;
  std::vector<double> prim_second;
  std::vector<double> consumed;
  std::vector<double> fr_first;
  std::vector<double> fr_second;
  std::vector<double> cop_first;
  std::vector<double> cop_second;
  // (shift, iterations) → the allocation counts of those ticks
  std::map<std::pair<int, int>, std::vector<std::size_t>> mallocs;
  // (solve error, phase manager error) → failed ticks, over the whole run
  std::map<std::pair<int, int>, std::size_t> failures;
  const double half = w0 + 0.5 * set.seconds;
  const double period_ms = 1e3 / hz;
  double heap_warm = -1.0;
  double heap_max = -1.0;
  double heap_end = -1.0;
  for (const TickRecord& r : records) {
    if (r.heap >= 0.0 && r.t >= w0 + set.warmup) {
      if (heap_warm < 0.0) {
        heap_warm = r.heap;
      }
      heap_max = std::max(heap_max, r.heap);
      heap_end = r.heap;
    }
    if (!r.ok) {
      ++failures[{r.error, r.phase_error}];
    }
    if (r.t < w0 || !r.ok || r.initial) {
      continue;
    }
    ++s.online_ticks;
    solve_ms.push_back(r.solve_ms);
    s.over_period += r.solve_ms > period_ms;
    s.injected += r.injected;
    if (r.shift < 0) {
      ++s.cold;
    } else if (r.shift == 0) {
      ++s.shift0;
    } else if (r.shift == 1) {
      ++s.shift1;
    } else if (r.shift == 2) {
      ++s.shift2;
    } else {
      ++s.shift3plus;
    }
    if (r.shift >= 0) {
      mallocs[{r.shift, r.iterations}].push_back(r.mallocs);
    }
    (r.t < half ? prim_first : prim_second).push_back(r.prim);
    consumed.push_back(r.consumed);
    if (std::isfinite(r.friction_margin) || std::isfinite(r.cop_margin)) {
      (r.t < half ? fr_first : fr_second).push_back(r.friction_margin);
      (r.t < half ? cop_first : cop_second).push_back(r.cop_margin);
    }
  }
  std::string failure_kinds;
  for (const auto& [key, n] : failures) {
    failure_kinds += " (solve error " + std::to_string(key.first) + ", phase error " +
                     std::to_string(key.second) + ": " + std::to_string(n) + ")";
  }
  s.p50_ms = Percentile(solve_ms, 0.5);
  s.p99_ms = Percentile(solve_ms, 0.99);
  s.max_ms = Percentile(solve_ms, 1.0);
  double per_iteration_sum = 0.0;
  std::size_t per_iteration_n = 0;
  std::string mismatches;
  std::string drifts;
  for (const auto& [key, counts] : mallocs) {
    ++s.malloc_groups;
    const auto [lo, hi] = std::minmax_element(counts.begin(), counts.end());
    if (*lo != *hi) {
      ++s.malloc_mismatched_groups;
      mismatches += " (shift " + std::to_string(key.first) + ", " + std::to_string(key.second) +
                    " it: " + std::to_string(*lo) + "-" + std::to_string(*hi) + " in " +
                    std::to_string(counts.size()) + ")";
    }
    // Late growth: the median of the group's last third (time order) above
    // the maximum of its first third. Judged on an armed run's groups of 30
    // ticks or more; the rest report it.
    if (counts.size() >= 3) {
      const std::size_t third = counts.size() / 3;
      const std::size_t first_max =
          *std::max_element(counts.begin(), counts.begin() + static_cast<std::ptrdiff_t>(third));
      std::vector<std::size_t> last(counts.end() - static_cast<std::ptrdiff_t>(third),
                                    counts.end());
      std::sort(last.begin(), last.end());
      const std::size_t last_median = last[last.size() / 2];
      const bool judged = set.armed && counts.size() >= 30;
      s.malloc_judged_groups += judged;
      if (last_median > first_max) {
        ++(judged ? s.malloc_drifting_groups : s.malloc_unjudged_late_growth);
        drifts += std::string(judged ? " (judged" : " (reported") + " shift " +
                  std::to_string(key.first) + ", " + std::to_string(key.second) +
                  " it: first-third max " + std::to_string(first_max) + ", last-third median " +
                  std::to_string(last_median) + " in " + std::to_string(counts.size()) + ")";
      }
    }
    if (key.second > 0) {
      per_iteration_sum += static_cast<double>(*lo) / key.second;
      ++per_iteration_n;
    }
  }
  s.mallocs_per_iteration =
      per_iteration_n > 0 ? per_iteration_sum / static_cast<double>(per_iteration_n) : 0.0;
  s.heap_warm = heap_warm;
  s.heap_end = heap_end;
  s.heap_growth = heap_warm >= 0.0 ? heap_end - heap_warm : 0.0;
  s.heap_excursion = heap_warm >= 0.0 ? heap_max - heap_warm : 0.0;
  s.heap_judged = set.seconds - set.warmup >= 30.0 && heap_warm >= 0.0;
  s.prim_med_first = Percentile(prim_first, 0.5);
  s.prim_med_second = Percentile(prim_second, 0.5);
  s.consumed_med = Percentile(consumed, 0.5);
  s.consumed_max = Percentile(consumed, 1.0);
  s.friction_first = Min(fr_first);
  s.friction_second = Min(fr_second);
  s.cop_first = Min(cop_first);
  s.cop_second = Min(cop_second);
  Summaries().push_back(s);

  std::printf(
      "[report] soak %s (%s, %.0f s, warm-up %.0f s, cpu %d, fifo %d, budget %d / %d, deadline "
      "%.1f ms, prim tol %.3g):\n"
      "[report]   online ticks %zu, solve p50 %.1f ms p99 %.1f ms max %.1f ms, over the %.1f ms "
      "period %zu\n"
      "[report]   MPC thread %s priority %d on cpus %s, writer cpu %d, OMP_NUM_THREADS %s; "
      "%d solver thread(s), workers on cpus %s (%s)\n"
      "[report]   overruns online %llu standing %llu; deadline misses %llu, withheld %llu (cold "
      "%llu), "
      "standing ticks %llu, failed %llu%s, published %llu\n"
      "[report]   shifts 0:%zu 1:%zu 2:%zu 3+:%zu cold:%zu (injected holds %zu)\n"
      "[report]   mallocs: %zu (shift, iterations) groups, %zu not constant%s; late growth judged "
      "on %zu, "
      "%zu growing, %zu reported only%s; "
      "%.0f per iteration\n"
      "[report]   heap warm %.0f B, end %.0f B, growth %.0f B, excursion %.0f B (%s)\n"
      "[report]   prim median %.3g -> %.3g; consumed median %.3g max %.3g; friction margin min "
      "%.3g -> %.3g N; CoP margin min %.3g -> %.3g m\n",
      name.c_str(), set.armed ? "armed" : "smoke", set.seconds, set.warmup, set.cpu,
      set.fifo_priority, set.max_iters, set.al_iters, set.deadline_ms, set.prim_tol, s.online_ticks,
      s.p50_ms, s.p99_ms, s.max_ms, period_ms, s.over_period, s.applied_policy.c_str(),
      s.applied_priority, s.applied_cpus.c_str(), s.writer_cpu, s.omp_threads.c_str(),
      s.solver_threads, s.worker_cpus.c_str(), s.worker_policies.c_str(),
      static_cast<unsigned long long>(s.online_overruns),
      static_cast<unsigned long long>(s.standing_overruns),
      static_cast<unsigned long long>(s.deadline_misses),
      static_cast<unsigned long long>(s.withheld), static_cast<unsigned long long>(s.cold_withheld),
      static_cast<unsigned long long>(s.initial_ticks), static_cast<unsigned long long>(s.failed),
      failure_kinds.c_str(), static_cast<unsigned long long>(s.published), s.shift0, s.shift1,
      s.shift2, s.shift3plus, s.cold, s.injected, s.malloc_groups, s.malloc_mismatched_groups,
      mismatches.c_str(), s.malloc_judged_groups, s.malloc_drifting_groups,
      s.malloc_unjudged_late_growth, drifts.c_str(), s.mallocs_per_iteration, s.heap_warm,
      s.heap_end, s.heap_growth, s.heap_excursion,
      s.heap_judged ? "judged" : "reported only: window after warm-up < 30 s", s.prim_med_first,
      s.prim_med_second, s.consumed_med, s.consumed_max, s.friction_first, s.friction_second,
      s.cop_first, s.cop_second);

  EXPECT_GT(s.online_ticks, 0U) << name;
  EXPECT_EQ(s.failed, 0U) << name << ":" << failure_kinds;
  EXPECT_EQ(s.malloc_drifting_groups, 0U)
      << name << ": allocations grew late in the run:" << drifts;
  if (s.heap_judged) {
    EXPECT_LE(s.heap_growth, 0.0) << name << ": the heap grew after the warm-up";
  }
}

INSTANTIATE_TEST_SUITE_P(
    Combinations, KinodynamicMpcSoak,
    ::testing::Combine(::testing::Values(Scenario::kStanceStatic, Scenario::kWalkRollout,
                                         Scenario::kWalkDisturbed),
                       ::testing::Values(true, false),
                       ::testing::Values("semi_implicit_euler", "rk2")),
    [](const ::testing::TestParamInfo<Combo>& info) { return ComboName(info.param); });

}  // namespace
