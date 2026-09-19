#ifndef RTC_MPC_THREAD_HANDLER_MPC_THREAD_HPP_
#define RTC_MPC_THREAD_HANDLER_MPC_THREAD_HPP_

/// @file handler_mpc_thread.hpp
/// @brief Concrete `MPCThread` subclass that wires a `PhaseManagerBase` FSM
///        into an `MPCHandlerBase` solver.
///
/// Role in the pipeline (Phase 6 contract; `git log --grep='rtc_mpc Phase 6'`):
///
///   RT thread                      MPC thread (this class)
///   ──────────                     ──────────────────────────
///   WriteState ──► SeqLock ──►     ReadState
///                                    │
///                                    ▼
///                                  phase_manager_->Update(q,v,sensor,tcp,t)
///                                    │                    (non-RT)
///                                    ▼ ctx
///                                  handler_->Solve(ctx, state, out) (noexcept)
///                                    │
///                                    ▼
///   ComputeReference ◄──TripleBuffer─ PublishSolution(out)
///
/// Ownership — `HandlerMPCThread` takes unique-ownership of both the handler
/// and the phase manager via `Configure`. The `RobotModelHandler` reference
/// is non-owning and must outlive this thread. `Solve` is single-threaded —
/// Aligator's shift-warm-start path runs on the calling thread.
///
/// Cross-mode swap (stretch scope, §Phase 6 Step 6) — on phase transition
/// where `ctx.ocp_type` differs from the current handler's dispatch key,
/// `Solve` calls `MPCFactory::Create` with the matching pre-built YAML node
/// and re-seeds the new handler via `SeedWarmStart(prev_out_)`. The factory
/// call is wrapped in `try/catch (...)` so this method stays `noexcept`;
/// allocations during the swap are accepted as a one-tick cost (phase-
/// transition ticks are infrequent). This is a KNOWN RT-1/RT-2 deviation on
/// an RT thread, not an exemption — production (DemoWbc) passes both factory
/// YAMLs, so it runs on real phase changes (invariants.md §RT Path). In baseline mode
/// (`factory_cfg_light_` / `factory_cfg_rich_` both null), mismatches return `kRebuildRequired`
/// without swapping and increment `failed_solves_`.
///
/// RT-safety on the steady-state path —
/// - `Solve` is `noexcept`; the cross-mode branch is the only try-wrapped
///   section and it is skipped on unchanged-`ocp_type` ticks.
/// - No `new`/`push_back`/`throw`; `pdata_`, `q_scratch_`, `v_scratch_`,
///   `sensor_scratch_`, and `prev_out_` are pre-allocated in `Configure`.
/// - No logging or stderr I/O: the MPC thread is an RT context
///   (architecture.md §Execution Contexts). Failures only bump the atomics
///   below; a non-RT reader (the owning controller's aux timer) reports them.
///
/// Observability — `Last*` / `Total*` / `Failed*` / `NullHandlerHit` atomics
/// expose solve-loop counters to tests and diagnostics. All reads/writes use `relaxed` memory
/// ordering since the counters are not used for synchronisation.

#include "rtc_mpc/handler/mpc_handler_base.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/phase/phase_manager_base.hpp"
#include "rtc_mpc/thread/mpc_thread.hpp"
#include "rtc_mpc/types/mpc_solution_types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Core>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>

namespace rtc::mpc {

/// @brief Concrete `MPCThread` that drives a `PhaseManagerBase` +
///        `MPCHandlerBase` pair at the target frequency.
class HandlerMPCThread final : public MPCThread {
 public:
  HandlerMPCThread() = default;
  ~HandlerMPCThread() override = default;

  HandlerMPCThread(const HandlerMPCThread&) = delete;
  HandlerMPCThread& operator=(const HandlerMPCThread&) = delete;
  HandlerMPCThread(HandlerMPCThread&&) = delete;
  HandlerMPCThread& operator=(HandlerMPCThread&&) = delete;

  /// @brief Install all runtime dependencies. Called once, off-RT, before
  ///        @ref MPCThread::Start. Takes ownership of @p handler and
  ///        @p phase_manager.
  ///
  /// @param model_handler       must outlive this thread; provides `nq` /
  ///                            `nv` / `ee_frame_id` for FK scratch sizing.
  /// @param handler             concrete MPC handler (`ContactLightMPC`
  ///                            or `ContactRichMPC`). Must be Init()'d
  ///                            already; this class does not re-Init.
  /// @param phase_manager       concrete FSM producing `PhaseContext` per
  ///                            tick. The mock or production FSM.
  /// @param factory_cfg_light   YAML node used if a cross-mode swap TO
  ///                            `"contact_light"` is requested. Pass
  ///                            `YAML::Node{}` to disable the swap branch.
  /// @param factory_cfg_rich    Likewise for `"contact_rich"`.
  ///
  /// @note Both factory YAMLs may be Null (default) — that disables cross-
  ///       mode swap and returns `kRebuildRequired` on mismatch ticks.
  void Configure(const RobotModelHandler& model_handler, std::unique_ptr<MPCHandlerBase> handler,
                 std::unique_ptr<PhaseManagerBase> phase_manager,
                 YAML::Node factory_cfg_light = YAML::Node{},
                 YAML::Node factory_cfg_rich = YAML::Node{}) noexcept;

  // ── Observability (lock-free atomics; safe from any thread) ─────────────

  /// @return the most recent `MPCSolveError` cast to int (0 on clean path).
  [[nodiscard]] int LastSolveErrorCode() const noexcept {
    return last_err_.load(std::memory_order_relaxed);
  }

  /// @return the phase id returned by the most recent `PhaseManager::Update`.
  [[nodiscard]] int LastPhaseId() const noexcept {
    return last_phase_id_.load(std::memory_order_relaxed);
  }

  /// @return solves attempted since @ref MPCThread::Start.
  [[nodiscard]] std::uint64_t TotalSolves() const noexcept {
    return total_solves_.load(std::memory_order_relaxed);
  }

  /// @return solves that returned non-`kNoError`.
  [[nodiscard]] std::uint64_t FailedSolves() const noexcept {
    return failed_solves_.load(std::memory_order_relaxed);
  }

  /// @return true once the null-handler guard has rejected a solve.
  [[nodiscard]] bool NullHandlerHit() const noexcept {
    return null_handler_hit_.load(std::memory_order_relaxed);
  }

 protected:
  bool Solve(const MPCStateSnapshot& state, MPCSolution& out) override;

 private:
  /// @brief Execute the cross-mode handler swap. Non-noexcept internally;
  ///        the caller wraps in try/catch. On success, returns true and
  ///        `handler_` points at the new instance (seeded via SeedWarmStart).
  bool TryCrossModeSwap(const PhaseContext& ctx);

  // Owned dependencies.
  std::unique_ptr<MPCHandlerBase> handler_{};
  std::unique_ptr<PhaseManagerBase> phase_manager_{};
  std::unique_ptr<pinocchio::Data> pdata_{};

  // Non-owning; must outlive this thread.
  const RobotModelHandler* model_{nullptr};

  // Pre-built YAML nodes for cross-mode swap; Null → swap disabled.
  YAML::Node factory_cfg_light_{};
  YAML::Node factory_cfg_rich_{};

  // Scratch buffers (sized in Configure, reused each tick).
  Eigen::VectorXd q_scratch_{};
  Eigen::VectorXd v_scratch_{};
  Eigen::VectorXd sensor_scratch_{};  // zero-length in Phase 6

  // Last-published solution, used to seed the next handler on cross-mode swap.
  // MPCSolution is trivially copyable (~17 KB std::array bundle) so `=` is an
  // alloc-free memcpy.
  MPCSolution prev_out_{};
  bool has_prev_out_{false};

  std::chrono::steady_clock::time_point start_time_{};

  // Observability counters (all `relaxed`).
  std::atomic<int> last_err_{0};
  std::atomic<int> last_phase_id_{-1};
  std::atomic<std::uint64_t> total_solves_{0};
  std::atomic<std::uint64_t> failed_solves_{0};
  std::atomic<bool> null_handler_hit_{false};
};

}  // namespace rtc::mpc

#endif  // RTC_MPC_THREAD_HANDLER_MPC_THREAD_HPP_
