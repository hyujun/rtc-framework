// locomotion_controller: the MPC — its build (Pass 1), its thread (activation
// to cleanup), and the tick's use of its plan (the reference sources, the
// plausibility check, the contact mask). See locomotion_controller.hpp.
#include "integrated_bringup/controllers/locomotion/pose_guard.hpp"
#include "integrated_bringup/controllers/locomotion_controller.hpp"
#include "integrated_bringup/support/mpc_pool_wait.hpp"
#include "rtc_base/threading/thread_utils.hpp"
#include "rtc_mpc/handler/mpc_factory.hpp"
#include "rtc_mpc/model/wb_layout.hpp"
#include "rtc_mpc/phase/phase_context.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace integrated_bringup {

namespace mpc = rtc::mpc;
using rtc::ControllerState;

namespace {

constexpr const char* kContext = "locomotion_controller";

// A contact's normal force may dip this far below zero (× the robot's weight).
// The plan's soft cone barrier keeps a stance foot's force positive; an
// unconverged plan may dip slightly below.
constexpr double kPullTolerance = 0.02;
// A unit quaternion to this tolerance on its squared norm.
constexpr double kQuaternionTolerance = 1e-6;
// The command filter's timer period stays at least a millisecond.
constexpr double kMaxCommandRateHz = 1000.0;

double Norm3(const double* v) noexcept {
  return Eigen::Map<const Eigen::Vector3d>(v).norm();
}

[[noreturn]] void Fail(const std::string& what) {
  throw std::invalid_argument(std::string(kContext) + ".mpc: " + what);
}

YAML::Node Section(const YAML::Node& cfg, const char* key) {
  const YAML::Node n = cfg[key];
  if (!n.IsDefined() || !n.IsMap()) {
    Fail(std::string("mpc.enabled needs the `") + key + ":` section (a map)");
  }
  return n;
}

}  // namespace

// ── Pass 1 ───────────────────────────────────────────────────────────────────

void LocomotionController::BuildMpc(const YAML::Node& cfg) {
  mpc_enabled_ = false;
  mpc_spawn_refused_ = false;
  rejected_plan_id_ = 0;  // the sampler's plan ids restart with its Init below
  phase_ = nullptr;
  phase_owned_.reset();
  mpc_model_.reset();
  if (!cfg_.mpc.enabled) {
    return;
  }
  tick_dt_ns_ = std::llround(GetDefaultDt() * 1e9);
  if (tick_dt_ns_ <= 0) {
    Fail("the control rate gives no tick period");
  }
  const YAML::Node& m = cfg_.mpc.node;
  const YAML::Node kino = Section(m, "kinodynamic");
  try {
    mpc_horizon_nodes_ = kino["horizon_nodes"].as<int>();
    mpc_dt_ = kino["dt"].as<double>();
  } catch (const YAML::Exception&) {
    Fail("kinodynamic.horizon_nodes / dt are not readable");
  }
  if (mpc_horizon_nodes_ < 1 || mpc_horizon_nodes_ > mpc::kMaxHorizon ||
      !(std::isfinite(mpc_dt_) && mpc_dt_ > 0.0)) {
    Fail("kinodynamic.horizon_nodes must be in [1, " + std::to_string(mpc::kMaxHorizon) +
         "] and dt > 0");
  }
  // The lock-step's tick counts: the MPC period in ticks must be whole, the
  // delivery within it (header). Whether this CM is a simulation's is the
  // node's to say (on_configure).
  mpc_lockstep_ = cfg_.mpc.lockstep;
  mpc_grid_ticks_ = 0;
  mpc_delivery_ticks_ = 0;
  mpc_wait_timeout_ns_ = 0;
  if (mpc_lockstep_) {
    const double period_ticks = static_cast<double>(cfg_.mpc.period_nodes) * mpc_dt_ * 1e9 /
                                static_cast<double>(tick_dt_ns_);
    const double whole = std::round(period_ticks);
    if (!(whole >= 1.0) || std::abs(period_ticks - whole) > 1e-6) {
      Fail("sync.mode lockstep needs period_nodes * dt to be a whole number of control ticks (" +
           std::to_string(period_ticks) + ")");
    }
    mpc_grid_ticks_ = static_cast<std::int64_t>(whole);
    mpc_delivery_ticks_ = cfg_.mpc.delivery_ticks == 0 ? mpc_grid_ticks_ : cfg_.mpc.delivery_ticks;
    if (mpc_delivery_ticks_ < 1 || mpc_delivery_ticks_ > mpc_grid_ticks_) {
      Fail("sync.delivery_ticks must be in [1, " + std::to_string(mpc_grid_ticks_) +
           "] (the period in ticks), got " + std::to_string(mpc_delivery_ticks_));
    }
    mpc_wait_timeout_ns_ = std::llround(cfg_.mpc.wait_timeout_s * 1e9);
  }

  // The MPC's model is the dynamics model (hands locked at neutral).
  mpc_model_ = std::make_unique<mpc::RobotModelHandler>();
  if (const auto err =
          mpc_model_->Init(*reduced_model_, locomotion::ToRobotModelHandlerYaml(cfg_.model));
      err != mpc::RobotModelInitError::kNoError) {
    Fail("the MPC model handler refused the model (error " + std::to_string(static_cast<int>(err)) +
         ")");
  }
  const mpc::WbLayout layout = mpc::WbLayout::FromHandler(*mpc_model_);
  if (!layout.valid() || layout.nq != info_.nq || layout.nv != info_.nv ||
      layout.n_contact_vars != qp_.Contacts().max_contact_vars) {
    Fail("the MPC's state / contact layout is not the whole-body QP's");
  }

  // The stage reference: q_default's body joints go to the MPC (its model has
  // no hand joint); the hand joints are the hands' hold under the MPC.
  const YAML::Node ref_in = Section(cfg, "reference");
  const YAML::Node qd = ref_in["q_default"];
  if (!qd.IsDefined() || !qd.IsMap()) {
    Fail("reference.q_default must be a map");
  }
  std::map<std::string, std::size_t> full_index;
  for (std::size_t a = 0; a < full_joint_order_.size(); ++a) {
    full_index[full_joint_order_[a]] = a;
  }
  const std::set<std::string> body(body_joint_names_.begin(), body_joint_names_.end());
  std::set<std::string> hands_named;
  const Eigen::VectorXd neutral_full = pinocchio::neutral(*full_model_);
  YAML::Node qd_out(YAML::NodeType::Map);
  q_default_full_.fill(0.0);
  for (const auto& kv : qd) {
    const std::string name = kv.first.as<std::string>();
    if (body.count(name) != 0) {
      qd_out[name] = YAML::Clone(kv.second);
      continue;
    }
    const auto it = full_index.find(name);
    if (it == full_index.end()) {
      Fail("reference.q_default names '" + name + "', a joint of neither the body nor a hand");
    }
    double v = std::numeric_limits<double>::quiet_NaN();
    try {
      v = kv.second.as<double>();
    } catch (const YAML::Exception&) {
    }
    if (!std::isfinite(v)) {
      Fail("reference.q_default." + name + " must be a finite number");
    }
    // The MPC and the QP model the hand locked at its neutral value; a hold
    // anywhere else would put mass where neither model has it.
    if (v != neutral_full(static_cast<Eigen::Index>(7 + it->second))) {
      Fail("reference.q_default." + name +
           " must be the joint's neutral value (the hands are locked there in the dynamics "
           "model)");
    }
    q_default_full_[it->second] = v;
    hands_named.insert(name);
  }
  for (const auto& name : full_joint_order_) {
    if (body.count(name) == 0 && hands_named.count(name) == 0) {
      Fail("reference.q_default does not name the hand joint '" + name +
           "' (its hold under the MPC)");
    }
  }
  YAML::Node ref_out = YAML::Clone(ref_in);
  ref_out["q_default"] = qd_out;

  YAML::Node pm(YAML::NodeType::Map);
  pm["horizon_nodes"] = mpc_horizon_nodes_;
  pm["dt"] = mpc_dt_;
  for (const char* key : {"gait", "swing", "terrain"}) {
    pm[key] = YAML::Clone(Section(cfg, key));
  }
  pm["reference"] = ref_out;
  phase_owned_ = std::make_unique<mpc::LocomotionPhaseManager>(*mpc_model_, contact_names_);
  if (const auto err = phase_owned_->Load(pm); err != mpc::LocomotionPhaseError::kNoError) {
    Fail("the gait / swing / terrain / reference sections were refused (LocomotionPhaseError " +
         std::to_string(static_cast<int>(err)) + ")");
  }
  phase_ = phase_owned_.get();

  // The command: at rest at the configured stance height until a command
  // source drives the filter. The builder reads the height of a command that
  // has seen no sample yet.
  mpc::CommandFilterConfig command{};
  if (const auto err = mpc::LoadCommandFilterConfig(Section(cfg, "command"), command);
      err != mpc::CommandFilterError::kNoError) {
    Fail("command: was refused (CommandFilterError " + std::to_string(static_cast<int>(err)) + ")");
  }
  if (command_filter_.Init(command) != mpc::CommandFilterError::kNoError ||
      command_filter_.Reset(command.base_height) != mpc::CommandFilterError::kNoError) {
    Fail("the command filter refused command:");
  }
  if (command.sample_rate_hz > kMaxCommandRateHz) {
    Fail("command.sample_rate_hz must be at most " + std::to_string(kMaxCommandRateHz) +
         " (the command filter's wall timer)");
  }
  phase_->SetCommand(command_filter_.output());

  mpc_manager_.Init(m, layout, reduced_model_.get());
  if (!mpc_manager_.ConfigValid()) {
    Fail("the plan sampler refused max_policy_age_s / riccati / max_stale_solutions");
  }
  mpc_manager_.SetModelEpoch(model_epoch_);
  mpc_manager_.SetEnabled(false);  // on at activation

  for (std::size_t j = 0; j < nb_; ++j) {
    joint_lo_[j] = reduced_model_->lowerPositionLimit(static_cast<Eigen::Index>(7 + j));
    joint_hi_[j] = reduced_model_->upperPositionLimit(static_cast<Eigen::Index>(7 + j));
  }
  mpc_enabled_ = true;
}

// ── The thread (aux thread) ──────────────────────────────────────────────────

bool LocomotionController::SpawnMpcThread(std::unique_ptr<mpc::MPCHandlerBase> handler) noexcept {
  if (mpc_thread_) {
    return true;
  }
  if (mpc_spawn_refused_ || !phase_owned_ || !mpc_model_) {
    RCLCPP_ERROR(logger_,
                 "[locomotion] the MPC thread was refused at an earlier activation (or was "
                 "stopped) — reconfigure the controller first");
    return false;
  }
  // Every refusal ends the same way: the handler and the phase manager may
  // have gone with a thread that never ran, and nothing retries.
  const auto refuse = [this]() noexcept {
    phase_ = nullptr;
    mpc_spawn_refused_ = true;
    return false;
  };
  try {
    const auto threads = rtc::SelectThreadConfigs();
    mpc::MpcThreadLaunchConfig launch{};
    launch.main = threads.mpc.main;
    launch.target_frequency_hz = 1.0 / (cfg_.mpc.period_nodes * mpc_dt_);
    if (!handler) {
      mpc::PhaseContext ctx;
      ctx.ocp_type = cfg_.mpc.node["ocp_type"] ? cfg_.mpc.node["ocp_type"].as<std::string>() : "";
      YAML::Node factory(YAML::NodeType::Map);
      factory["mpc"] = cfg_.mpc.node;
      ctx.cost_config.horizon_length = mpc_horizon_nodes_;
      ctx.cost_config.dt = mpc_dt_;
      const mpc::MPCFactoryStatus status =
          mpc::MPCFactory::Create(factory, *mpc_model_, ctx, handler, mpc_worker_slots_.count);
      if (status.error != mpc::MPCFactoryError::kNoError || !handler) {
        RCLCPP_ERROR(logger_,
                     "[locomotion] MPCFactory::Create refused mpc.* (err=%d, init=%d, config=%d); "
                     "this run's layout profile reserves %d MPC worker slot(s) — activation "
                     "refused",
                     static_cast<int>(status.error), static_cast<int>(status.init_error),
                     static_cast<int>(status.ocp_config_error), mpc_worker_slots_.count);
        return refuse();
      }
    }
    launch.solver_threads = handler->solver_config().num_threads;
    launch.workers = mpc_worker_slots_;
    if (mpc_lockstep_) {
      // One tick per request from the RT tick (the channel is this
      // controller's, opened at configure); the publish rule reads the nodes
      // from the delivery on.
      launch.trigger = mpc::SolveTrigger::kExternal;
      launch.channel = &mpc_channel_;
    }
    auto thread = std::make_unique<mpc::HandlerMPCThread>();
    thread->Configure(*mpc_model_, std::move(handler), std::move(phase_owned_));
    if (mpc_lockstep_) {
      thread->SetConsumedWindowOffsetNs(
          static_cast<std::uint64_t>(mpc_delivery_ticks_ * tick_dt_ns_));
    }
    thread->Init(mpc_manager_, launch);
    // A record of an earlier thread run is not this one's: the RT tick reads
    // "none" until this thread writes its first (thread start orders the two
    // writers).
    mpc_solve_record_.StartWrite() = mpc::SolveRecord{};
    mpc_solve_record_.FinishWrite();
    thread->SetSolveRecordSink(&mpc_solve_record_);
    thread->Start();
    // One thread too: its loop entry can refuse (thread count, layout).
    if (!AwaitMpcPool(*thread)) {
      RCLCPP_ERROR(logger_,
                   "[locomotion] the MPC thread refused its %d-thread solver set-up (reason "
                   "on stderr) — activation refused",
                   launch.solver_threads);
      thread->StopAndJoin();
      return refuse();
    }
    mpc_thread_ = std::move(thread);
    RCLCPP_INFO(logger_,
                "[locomotion] MPC thread started: core=%d prio=%d %.2f Hz, %d solver thread(s)",
                launch.main.cpu_core, launch.main.sched_priority, launch.target_frequency_hz,
                launch.solver_threads);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "[locomotion] the MPC thread could not start: %s", e.what());
    return refuse();
  }
  return true;
}

void LocomotionController::ReleaseLockstepTick() noexcept {
  // The channel's contract (rtc_mpc solve_trigger_channel.hpp): disarm before
  // the thread is paused or stopped. A tick waiting on the channel wakes with
  // "cancelled" at once; give it a few milliseconds to leave Compute before
  // the thread it was waiting on goes — never the wait timeout itself (this
  // runs on the nrt executor).
  mpc_channel_.Disarm();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5);
  while (in_compute_.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

void LocomotionController::PauseMpc() noexcept {
  StopCommandTimer();  // its writer is the paused thread's phase manager
  mpc_manager_.SetEnabled(false);
  if (mpc_thread_) {
    ReleaseLockstepTick();
    mpc_thread_->Pause();
  }
}

void LocomotionController::StopMpcThread() noexcept {
  StopCommandTimer();  // before the thread: the timer writes its phase manager
  mpc_manager_.SetEnabled(false);
  mpc_timing_timer_.reset();
  mpc_timing_cb_group_.reset();
  if (mpc_thread_) {
    ReleaseLockstepTick();
    mpc_thread_->StopAndJoin();
    mpc_thread_.reset();
    phase_ = nullptr;  // owned by the thread
  }
  StoreMpcCounters();        // zeros: no thread
  mpc_failed_reported_ = 0;  // a later thread counts from 0
  mpc_timing_ticks_ = 0;
}

void LocomotionController::LogMpcTick() noexcept {
  if (!mpc_thread_) {
    return;
  }
  StoreMpcCounters();
  mpc_thread_->TimingProducer().Drain([this](const rtc::RtTickTimingSample& s) {
    if (mpc_timing_logger_.IsOpen()) {
      mpc_timing_logger_.Log(s);
    }
  });
  const std::uint64_t failed = mpc_thread_->FailedSolves();
  if (failed > mpc_failed_reported_) {
    RCLCPP_WARN(logger_, "[locomotion] %lu MPC solve(s) failed (total %lu, last error %d)",
                static_cast<unsigned long>(failed - mpc_failed_reported_),
                static_cast<unsigned long>(mpc_thread_->TotalSolves()),
                mpc_thread_->LastSolveErrorCode());
    mpc_failed_reported_ = failed;
  }
  constexpr std::uint64_t kInfoEveryTicks = 10;
  if (mpc_manager_.Enabled() && ++mpc_timing_ticks_ % kInfoEveryTicks == 0) {
    const auto stats = mpc_manager_.GetSolveStats();
    RCLCPP_INFO(logger_,
                "[locomotion] mpc: solves %lu (failed %lu, deadline miss %lu, withheld %lu, "
                "overrun %lu), solve p50 %.1f ms p99 %.1f ms; refused: epoch %lu state %lu age %lu "
                "dim %lu implausible %lu (last check %u, held %lu)",
                static_cast<unsigned long>(mpc_thread_->TotalSolves()),
                static_cast<unsigned long>(failed),
                static_cast<unsigned long>(mpc_thread_->DeadlineMissCount()),
                static_cast<unsigned long>(mpc_thread_->WithheldCount()),
                static_cast<unsigned long>(mpc_thread_->OverrunCount()),
                static_cast<double>(stats.p50_ns) * 1e-6, static_cast<double>(stats.p99_ns) * 1e-6,
                static_cast<unsigned long>(mpc_manager_.EpochRejections()),
                static_cast<unsigned long>(mpc_manager_.StateEpochRejections()),
                static_cast<unsigned long>(mpc_manager_.PolicyAgeRejections()),
                static_cast<unsigned long>(mpc_manager_.DimensionRejections()),
                static_cast<unsigned long>(plausibility_rejects_.load(std::memory_order_relaxed)),
                static_cast<unsigned>(last_reject_check_.load(std::memory_order_relaxed)),
                static_cast<unsigned long>(plan_holds_.load(std::memory_order_relaxed)));
  }
}

// ── The tick (RT thread) ─────────────────────────────────────────────────────

void LocomotionController::WriteMpcState() noexcept {
  mpc_manager_.SetContactMask(contact_mask_);
  mpc_manager_.WriteState(qp_state_.q, qp_state_.v, mpc_t_ns_, measured_.sequence);
}

void LocomotionController::RestartMpcStream() noexcept {
  // The gait first, then the epoch: a solve of the new stream must not run on
  // the old schedule.
  if (phase_ != nullptr) {
    phase_->RequestReset();
  }
  mpc_manager_.SetStateEpoch(++mpc_state_epoch_);
  // Lock-step: the grid restarts with the stream's first latched tick; a solve
  // still running answers its request (the manager refuses its plan by epoch),
  // and the next grid tick waits for it before it triggers.
  mpc_grid_known_ = false;
  wbc_ref_ = mpc::WbcReference{};
  u_fb_norm_ = 0.0;
  rejected_plan_id_ = 0;
  ForgetFollowedPlans();
  ref_source_ = ReferenceSource::kIdle;
  ApplyContactMask(all_contacts_mask_);
}

void LocomotionController::LockstepTick(const ControllerState& state) noexcept {
  const std::uint64_t it = state.iteration;
  mpc_wait_us_ = 0.0;
  // 1. A solve requested L ticks ago is due: wait for it, whatever the state
  //    source says this tick (the plan's age must not depend on it).
  if (mpc_trigger_outstanding_ &&
      it >= mpc_trigger_iter_ + static_cast<std::uint64_t>(mpc_delivery_ticks_)) {
    WaitForSolve();
  }
  if (fault_latched_tick_ || !(source_ok_ && reference_latched_)) {
    return;  // no plan to follow, nothing to send; the grid keeps its origin
  }
  // 2. The due plan (the manager's delivery delay makes it due now) before a
  //    new request can overwrite the buffer behind it.
  FollowPlan(state);
  if (fault_latched_tick_) {
    return;
  }
  // 3. The grid: the stream's first latched tick is its origin.
  if (!mpc_grid_known_) {
    mpc_grid_known_ = true;
    mpc_grid_origin_ = it;
  }
  if ((it - mpc_grid_origin_) % static_cast<std::uint64_t>(mpc_grid_ticks_) != 0) {
    return;
  }
  if (mpc_trigger_outstanding_) {
    WaitForSolve();  // L < N never reaches here busy; a restart's old solve can
    if (fault_latched_tick_) {
      return;
    }
  }
  WriteMpcState();
  mpc_channel_.RequestSolve();
  mpc_trigger_outstanding_ = mpc_channel_.Armed();
  mpc_trigger_iter_ = it;
}

void LocomotionController::WaitForSolve() noexcept {
  const auto t0 = std::chrono::steady_clock::now();
  const auto status = mpc_channel_.WaitIdle(mpc_wait_timeout_ns_);
  mpc_wait_us_ =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
  ++mpc_wait_count_;
  mpc_trigger_outstanding_ = false;
  if (status == mpc::SolveTriggerChannel::WaitStatus::kTimeout) {
    Latch(FaultCause::kMpcWait);
  }
}

void LocomotionController::FollowPlan(const ControllerState& state) noexcept {
  u_fb_norm_ = 0.0;
  // A plan refused once stays refused for every later sample of it.
  const bool sampled = mpc_manager_.SampleReference(qp_state_.q, qp_state_.v, mpc_t_ns_, wbc_ref_);
  RecordSample(sampled);
  bool usable = sampled && wbc_ref_.plan_id != rejected_plan_id_;
  if (usable) {
    if (const std::uint8_t check = Plausible(wbc_ref_); check != 0) {
      sample_diag_.check = check;
      NoteRefusal(check);
      usable = HoldEarlierPlan();
    }
  }
  if (!usable) {
    wbc_ref_.valid = false;
  }
  const bool all_support = wbc_ref_.contact_mask == all_contacts_mask_;
  if (ref_source_ == ReferenceSource::kIdle) {
    // A published plan with every contact in support is a plan to stand on:
    // the producer's publish rules (a cold solve only once converged, else
    // the consumed nodes' residual) are the gate. (Under the E-STOP this is
    // not reached: nothing is latched, so nothing is followed.)
    if (!(usable && all_support)) {
      return;
    }
  } else if (!usable) {
    LosePlan(state);
    return;
  }
  ref_source_ = all_support ? ReferenceSource::kStand : ReferenceSource::kWalk;
  if (wbc_ref_.plan_id != followed_id_) {
    followed_before_id_ = followed_id_;
    followed_id_ = wbc_ref_.plan_id;
  }
  CopyPlan();
  ApplyContactMask(wbc_ref_.contact_mask);
}

void LocomotionController::ForgetFollowedPlans() noexcept {
  followed_id_ = 0;
  followed_before_id_ = 0;
}

void LocomotionController::NoteRefusal(std::uint8_t check) noexcept {
  last_reject_check_.store(check, std::memory_order_relaxed);
  rejected_plan_id_ = wbc_ref_.plan_id;
  plausibility_rejects_.fetch_add(1, std::memory_order_relaxed);
}

void LocomotionController::RecordSample(bool sampled) noexcept {
  auto& s = sample_diag_;  // cleared at the top of the tick
  if (!sampled) {
    return;
  }
  const auto& r = wbc_ref_;
  s.plan_id = r.plan_id;
  s.converged = r.converged;
  s.iterations = r.iterations;
  s.prim_inf = r.prim_inf;
  s.prim_head = r.prim_inf_head;
  s.prim_node = r.prim_inf_node;
  s.dual_inf = r.dual_inf;
  s.mask = r.contact_mask;
  s.progress = r.progress;
  for (std::size_t j = 0; j < nb_; ++j) {  // a sample is finite (the manager's rule)
    const double a = std::abs(r.a_ref[6 + j]);
    if (a > s.a_joint_max) {
      s.a_joint_max = a;
      s.a_joint_idx = static_cast<std::int32_t>(j);
    }
  }
}

bool LocomotionController::HoldEarlierPlan() noexcept {
  // Only the plan followed before the refused one (none in idle): the
  // manager's earlier plan may be one this controller never followed.
  const std::uint64_t before =
      rejected_plan_id_ == followed_id_ ? followed_before_id_ : followed_id_;
  if (mpc_manager_.EarlierPlanId() != before ||
      !mpc_manager_.RejectActivePlan(qp_state_.q, qp_state_.v, mpc_t_ns_, wbc_ref_)) {
    return false;
  }
  // Followed before, but it is checked again on this tick (a refusal like any other).
  if (const std::uint8_t check = Plausible(wbc_ref_); check != 0) {
    NoteRefusal(check);
    return false;
  }
  plan_holds_.fetch_add(1, std::memory_order_relaxed);
  return true;
}

std::uint8_t LocomotionController::Plausible(const mpc::WbcReference& r) const noexcept {
  // 0 = plausible; otherwise the number of the check that failed (logged).
  // Every comparison is written so that a NaN fails it. The plan's
  // accelerations are not judged: neither the original whole-body MPC's joint
  // controller nor simple-mpc's inverse dynamics bounds them (a sample is
  // finite by the manager's rule), and on the sim walk a bound refused the
  // swing knee's lift-off demand of every first step. Numbers 6 and 11 (the
  // two acceleration bounds) are retired so that a logged number keeps its
  // meaning.
  const auto& s = cfg_.mpc;
  if (r.nq != info_.nq || r.nv != info_.nv) {
    return 1;
  }
  if (r.contact_mask == 0 || (r.contact_mask & ~all_contacts_mask_) != 0) {
    return 2;  // no support, or a contact the model does not have
  }
  // The base orientation: a unit quaternion within joint_delta_rad of the measured one.
  const Eigen::Quaterniond q_ref(r.q_ref[6], r.q_ref[3], r.q_ref[4], r.q_ref[5]);
  if (!(std::abs(q_ref.squaredNorm() - 1.0) < kQuaternionTolerance)) {
    return 3;
  }
  const Eigen::Quaterniond q_meas(qp_state_.q(6), qp_state_.q(3), qp_state_.q(4), qp_state_.q(5));
  if (!(q_ref.angularDistance(q_meas) <= s.sanity_joint_delta_rad)) {
    return 4;
  }
  for (std::size_t j = 0; j < nb_; ++j) {
    const double q = r.q_ref[7 + j];
    const double q_now = qp_state_.q(static_cast<Eigen::Index>(7 + j));
    if (!(q >= joint_lo_[j] - s.sanity_joint_delta_rad &&
          q <= joint_hi_[j] + s.sanity_joint_delta_rad &&
          std::abs(q - q_now) <= s.sanity_joint_delta_rad)) {
      return 5;
    }
  }
  const auto& cmd = command_filter_.config();
  std::array<double, LocomotionDiagLogPod::kMaxContacts> z{};
  const std::size_t nc = ContactHeights(z);
  double z_lo = std::numeric_limits<double>::infinity();
  for (std::size_t c = 0; c < nc; ++c) {
    z_lo = std::min(z_lo, z[c]);
  }
  const double height = r.q_ref[2] - z_lo;
  if (!(height >= cmd.h_min && height <= cmd.h_max)) {
    return 7;
  }
  if (!(Norm3(r.v_ref.data()) <= s.sanity_base_speed)) {
    return 8;
  }
  const double weight = weight_;
  double fz_sum = 0.0;
  std::size_t offset = 0;
  for (const auto& c : cfg_.model.contacts) {
    const double fz = r.wrench_ref[offset + 2];
    if (!(fz >= -kPullTolerance * weight)) {
      return 9;
    }
    fz_sum += fz;
    offset += static_cast<std::size_t>(c.dim);
  }
  // The plan carries the robot: between 1 / wrench_scale and wrench_scale of its weight.
  if (!(fz_sum <= s.sanity_wrench_scale * weight && fz_sum >= weight / s.sanity_wrench_scale)) {
    return 10;
  }
  return 0;
}

void LocomotionController::CopyPlan() noexcept {
  using ConstMap = Eigen::Map<const Eigen::VectorXd>;
  const auto& r = wbc_ref_;
  ref_.q_des = ConstMap(r.q_ref.data(), ref_.q_des.size());
  ref_.v_des = ConstMap(r.v_ref.data(), ref_.v_des.size());
  ref_.a_des = ConstMap(r.a_ref.data(), ref_.a_des.size());
  if (cfg_.mpc.riccati_enabled) {
    const ConstMap u_fb(r.u_fb_acc.data(), ref_.a_des.size());
    ref_.a_des += u_fb;
    u_fb_norm_ = u_fb.norm();
  }
  ref_.tau_ff.setZero();
  ref_.lambda_des.setZero();
  const Eigen::Index nl = std::min<Eigen::Index>(ref_.lambda_des.size(), r.n_contact_vars);
  ref_.lambda_des.head(nl) = ConstMap(r.wrench_ref.data(), nl);
}

void LocomotionController::ApplyContactMask(std::uint32_t mask) noexcept {
  const int nc = static_cast<int>(contact_frames_full_.size());
  for (int i = 0; i < nc; ++i) {
    contacts_.SetActivationTarget(i, ((mask >> static_cast<unsigned>(i)) & 1U) != 0 ? 1.0 : 0.0,
                                  0.0);
  }
  contacts_.UpdateActivation(static_cast<double>(tick_dt_ns_) * 1e-9);
  contacts_.RecomputeActive(qp_.Contacts());
  contact_mask_ = mask;
}

std::size_t LocomotionController::ContactHeights(
    std::array<double, LocomotionDiagLogPod::kMaxContacts>& z) const noexcept {
  const std::size_t nc = std::min(contact_frames_full_.size(), z.size());
  for (std::size_t c = 0; c < nc; ++c) {
    z[c] = full_data_->oMf[contact_frames_full_[c]].translation().z();
  }
  return nc;
}

void LocomotionController::LosePlan(const ControllerState& state) noexcept {
  ForgetFollowedPlans();
  // The pose guard with the stance checks (header): a still double support
  // the guard passes is re-latched at once; anything else is the fault — a
  // swing foot cannot wait for the robot to settle.
  if (GuardPose(/*stance=*/true) == 0) {
    ref_source_ = ReferenceSource::kIdle;
    ApplyContactMask(all_contacts_mask_);
    reference_latched_ = false;
    LatchReference(state);
  } else {
    Latch(FaultCause::kPlanLoss);
  }
}

}  // namespace integrated_bringup
