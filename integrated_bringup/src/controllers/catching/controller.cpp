#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "integrated_bringup/support/bringup_logging.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_controller_interface/device_readability.hpp"

#include <unistd.h>  // close (planner wake eventfd)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

namespace integrated_bringup {

namespace {

// Fallback position bounds for a device whose YAML declares no `joint_limits`
// (LoadDeviceLimitsFromConfig fills the slot with these). ±2π is the same
// envelope the other demo bindings pass — stated here rather than hidden in a
// default argument so this controller's clamp is visible at its call site.
inline constexpr double kFallbackPositionLower = -6.2832;
inline constexpr double kFallbackPositionUpper = 6.2832;
inline constexpr double kFallbackMaxVelocity = 2.0;

/// The tick's own reading of the steady clock.
///
/// Read per tick rather than accumulated as `iteration * dt` (plan §3): the
/// two diverge by exactly the jitter and overrun this controller's deadlines
/// are about, and the accumulated version cannot see a missed tick at all.
/// The axis itself is `rtc::SteadyNowNs` — the same read the device backends
/// use for their receipt stamps, which is what makes an age computed here
/// comparable to one computed there.
[[nodiscard]] std::int64_t SteadyNowNs() noexcept {
  return rtc::SteadyNowNs();
}

}  // namespace

DemoCatchingController::DemoCatchingController(std::string_view urdf_path) : urdf_path_(urdf_path) {
  // Nothing to build: this skeleton runs no model. urdf_path_ is stored so the
  // signature matches the registry factory and S5.1 can build one here without
  // changing the registration.
}

DemoCatchingController::~DemoCatchingController() {
  // The drain timer first — its callback captures `this`. Then the thread: it
  // holds references to planner_cycle_, the boxes and the timing ring, and it
  // may be inside poll() on the eventfd. Join wakes it (OnRequestStop writes
  // the fd) and waits for the loop to leave; only then is the fd closed, so a
  // recycled fd number can never be polled by a thread that outlived it.
  planner_timing_timer_.reset();
  planner_timing_cb_group_.reset();
  planner_thread_.reset();
  const int fd = planner_wake_fd_.exchange(-1, std::memory_order_acq_rel);
  if (fd >= 0) {
    static_cast<void>(::close(fd));
  }
}

// ── Configuration ───────────────────────────────────────────────────────────

void DemoCatchingController::LoadConfig(const YAML::Node& cfg) {
  // Base first: `topics:` (the group list this controller's device indices are
  // defined against) and `command_type`.
  RTControllerInterface::LoadConfig(cfg);

  if (!cfg || !cfg.IsMap()) {
    throw std::runtime_error(
        "DemoCatchingController: config node is absent or not a map — this controller is "
        "registered as REQUIRING_CONFIG and has no defensible defaults (no hand profile)");
  }

  // ── diagnostic.hand_step ────────────────────────────────────────────────
  // Default false: from S7.1 the hand sequencer owns the hand, and an
  // operator-issued step would then race it. S4a's own YAML turns it on.
  hand_step_enabled_ = false;
  if (const YAML::Node diag = cfg["diagnostic"]; diag) {
    if (!diag.IsMap()) {
      throw std::runtime_error("DemoCatchingController: 'diagnostic' must be a map");
    }
    if (const YAML::Node step = diag["hand_step"]; step) {
      hand_step_enabled_ = step.as<bool>();
    }
  }

  // ── catching: (L0 §5.3 / §9 G0-C subset) ────────────────────────────────
  // Parsed here so a malformed tree refuses the configure through the same
  // try/catch every other controller's LoadConfig uses. The VERDICT is applied
  // in on_configure, which is the hook that can refuse.
  const YAML::Node catching = cfg["catching"];
  catching_section_present_ = static_cast<bool>(catching);
  if (catching_section_present_) {
    params_ = rtc::catching::ParseCatchingParams(catching);
  }

  // ── io: the two keys the numeric core does not carry ────────────────────
  // `CatchingParams` is deliberately numbers-only (it is the schema the
  // validator judges), so the topic and frame names are read here — the same
  // split `diagnostic.hand_step` already uses. They live under `catching.io`
  // with the rest of the lane so the operator reads one block, not two.
  traj_topic_.clear();
  expected_frame_ = "world";
  vision_base_frame_.clear();
  vision_yaw_deg_ = 0.0;
  vision_translation_ = {0.0, 0.0, 0.0};
  if (catching_section_present_) {
    if (const YAML::Node io = catching["io"]; io) {
      if (!io.IsMap()) {
        throw std::runtime_error("DemoCatchingController: 'catching.io' must be a map");
      }
      if (const YAML::Node topic = io["traj_topic"]; topic) {
        traj_topic_ = topic.as<std::string>();
      }
      if (const YAML::Node frame = io["expected_frame"]; frame) {
        expected_frame_ = frame.as<std::string>();
      }
      // Vision world → model world (plan §11). Two keys, the map tool's split:
      // the frame is a property of the URDF, the transform a measurement.
      if (const YAML::Node base = io["arm_base_frame"]; base) {
        vision_base_frame_ = base.as<std::string>();
      }
      if (const YAML::Node btw = io["base_T_world"]; btw) {
        if (!btw.IsMap() || vision_base_frame_.empty()) {
          throw std::runtime_error(
              "DemoCatchingController: 'catching.io.base_T_world' must be a map {yaw_deg, "
              "translation} and needs 'catching.io.arm_base_frame'");
        }
        vision_yaw_deg_ = btw["yaw_deg"] ? btw["yaw_deg"].as<double>() : 0.0;
        if (const YAML::Node tr = btw["translation"]; tr) {
          if (!tr.IsSequence() || tr.size() != 3) {
            throw std::runtime_error(
                "DemoCatchingController: 'catching.io.base_T_world.translation' must be [x, y, "
                "z]");
          }
          for (std::size_t k = 0; k < 3; ++k) {
            vision_translation_[k] = tr[k].as<double>();
          }
        }
        if (!std::isfinite(vision_yaw_deg_) || !std::isfinite(vision_translation_[0]) ||
            !std::isfinite(vision_translation_[1]) || !std::isfinite(vision_translation_[2])) {
          throw std::runtime_error(
              "DemoCatchingController: 'catching.io.base_T_world' must be finite");
        }
      }
    }
  }

  // ── robot.arm / catch_frame / oracle: the binding-level keys ────────────
  if (catching_section_present_) {
    if (const YAML::Node frame = catching["catch_frame"]; frame) {
      catch_frame_name_ = frame.as<std::string>();
    }
    if (const YAML::Node robot = catching["robot"]; robot && robot["arm"]) {
      const YAML::Node arm = robot["arm"];
      if (const YAML::Node pkg = arm["accel_limits_package"]; pkg) {
        accel_limits_package_ = pkg.as<std::string>();
      }
      if (const YAML::Node path = arm["accel_limits_path"]; path) {
        accel_limits_path_ = path.as<std::string>();
      }
      if (const YAML::Node group = arm["accel_limits_group"]; group) {
        accel_limits_group_ = group.as<std::string>();
      }
    }
  }

  // ── diagnostic.oracle_plan (A-S5-8) ─────────────────────────────────────
  // The stand-in for the S6 planner: one fixed catch point, supplied by the
  // operator from a ground-truth measurement. It exists because the tracking
  // law cannot be exercised — in a test or in the sim — without SOMETHING
  // naming a catch point, and inventing one inside the controller would make
  // the verification run measure the invention.
  oracle_enabled_ = false;
  if (const YAML::Node diag = cfg["diagnostic"]; diag) {
    if (const YAML::Node oracle = diag["oracle_plan"]; oracle) {
      if (!oracle.IsMap()) {
        throw std::runtime_error("DemoCatchingController: 'diagnostic.oracle_plan' must be a map");
      }
      oracle_enabled_ = oracle["enabled"] && oracle["enabled"].as<bool>();
      const auto read3 = [&oracle](const char* key, std::array<double, 3>& dst) {
        const YAML::Node node = oracle[key];
        if (!node) {
          return;
        }
        if (!node.IsSequence() || node.size() != 3) {
          throw std::runtime_error(std::string("DemoCatchingController: 'diagnostic.oracle_plan.") +
                                   key + "' must be three numbers");
        }
        for (std::size_t i = 0; i < 3; ++i) {
          dst[i] = node[i].as<double>();
        }
      };
      read3("p_c", oracle_p_c_);
      read3("a_d", oracle_a_d_);
      if (const YAML::Node t = oracle["t_c_offset_s"]; t) {
        oracle_t_c_offset_s_ = t.as<double>();
      }
      if (const YAML::Node g = oracle["gamma_f"]; g) {
        oracle_gamma_f_ = g.as<double>();
      }
    }
  }

  // ── planner.* thread keys (S6-A) ────────────────────────────────────────
  // Parsed here, like the rest of the tree, so a malformed key refuses the
  // configure through LoadConfig's own try/catch rather than defaulting.
  planner_params_ = catching_section_present_ ? rtc::catching::ParsePlannerParams(catching)
                                              : rtc::catching::PlannerParams{};
  catch_pose_ik_config_ = catching_section_present_
                              ? rtc::catching::ParseCatchPoseIkParams(catching)
                              : rtc::catching::CatchPoseIkConfig{};

  // ── logs: (Phase C) ─────────────────────────────────────────────────────
  parsed_log_entries_.clear();
  if (const YAML::Node logs = cfg["logs"]; logs) {
    if (!logs.IsSequence()) {
      throw std::runtime_error("DemoCatchingController: 'logs' must be a sequence");
    }
    for (const auto& entry : logs) {
      if (!entry.IsMap() || !entry["msg_type"]) {
        throw std::runtime_error("DemoCatchingController: each `logs` entry needs `msg_type`");
      }
      ParsedLogEntry e;
      e.msg_type = entry["msg_type"].as<std::string>();
      if (entry["instance"]) {
        e.instance = entry["instance"].as<std::string>();
      }
      // Closed set: two channel types — the per-device state logs and the
      // per-tick record (S5.4). A typo is a hard fail at parse time rather
      // than a CSV that never appears.
      if (e.msg_type != "rtc_msgs/DeviceStateLog" && e.msg_type != kCatchingDiagLogMsgType) {
        throw std::runtime_error("DemoCatchingController: unknown msg_type in `logs`: " +
                                 e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
}

void DemoCatchingController::OnDeviceConfigsSet() {
  ResolveDevices();
}

void DemoCatchingController::ResolveDevices() {
  const auto arm = GetPrimaryDeviceName();
  const auto hand = GetSecondaryDeviceName();

  arm_dof_ = 0;
  hand_dof_ = 0;
  arm_joint_names_.clear();
  hand_joint_names_.clear();
  hand_motor_names_.clear();
  hand_sensor_names_.clear();
  non_sim_groups_.clear();
  device_configs_seen_ = false;

  if (const auto* cfg = GetDeviceNameConfig(arm); cfg != nullptr) {
    arm_dof_ = static_cast<int>(cfg->joint_state_names.size());
    arm_joint_names_ = cfg->joint_state_names;
  }
  if (!hand.empty()) {
    if (const auto* cfg = GetDeviceNameConfig(hand); cfg != nullptr) {
      hand_dof_ = static_cast<int>(cfg->joint_state_names.size());
      hand_joint_names_ = cfg->joint_state_names;
      hand_motor_names_ = cfg->motor_state_names;
      hand_sensor_names_ = cfg->sensor_names;
    }
  }
  arm_dof_ = std::min(arm_dof_, kDemoCatchingMaxArmDof);
  hand_dof_ = std::min(hand_dof_, kDemoCatchingMaxHandDof);

  // ── Which validator axis this configure is judged on (A-S5-1) ───────────
  // Fail-closed on BOTH "wrong backend" and "no backend block": the question
  // is not "is this a real arm" but "has this device PROVEN it is the
  // simulator", and silence does not prove it. Since E-8 was approved this no
  // longer gates activation by itself — it selects `real_arm_config`, and the
  // provisional/TBD rule (L0 §5.3) is what can then park the instance.
  for (const auto& group : topic_config_.groups) {
    const auto* cfg = GetDeviceNameConfig(group.first);
    if (cfg == nullptr) {
      continue;  // no config for this group yet — nothing claimed, nothing to prove
    }
    device_configs_seen_ = true;
    if (!cfg->backend.has_value() || cfg->backend->type != kCatchingSimBackendType) {
      non_sim_groups_.push_back(group.first);
    }
  }
  // No device config resolved yet ⇒ nothing has proven anything, so the strict
  // axis applies. ResolveDevices runs twice (hook, then on_configure), and the
  // first run can legitimately see no groups.
  real_arm_config_ = !device_configs_seen_ || !non_sim_groups_.empty();

  LoadDeviceLimitsFromConfig(device_position_lower_, device_position_upper_, device_max_velocity_,
                             kFallbackPositionLower, kFallbackPositionUpper, kFallbackMaxVelocity);
}

// ── Targets ─────────────────────────────────────────────────────────────────

void DemoCatchingController::SetDeviceTarget(int device_idx,
                                             std::span<const double> target) noexcept {
  // Off-RT (the controller's non-RT callback group). Two refusals happen here
  // rather than on the RT side so the operator's goal is answered where it was
  // issued, and so a refused goal never occupies a mailbox slot.
  if (device_idx != kCatchingHandDeviceIdx) {
    // The arm slot has exactly one writer: the activation latch. S5.1 opens
    // this path together with the E-8 approval.
    arm_target_reject_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "target for device %d refused: this controller holds the arm and accepts "
                         "goals on the hand device (%d) only",
                         device_idx, kCatchingHandDeviceIdx);
    return;
  }
  if (!hand_step_enabled_) {
    hand_step_disabled_reject_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "hand target refused: 'diagnostic.hand_step' is false, so the hand holds "
                         "its activation pose");
    return;
  }
  // A partial goal is refused rather than half-applied. This controller has no
  // persistent target row — the overlay in WriteDeviceCommand covers
  // [0, hand_target_width_) and the REST comes from the activation latch — so a
  // short goal does not leave the untouched joints where the previous step put
  // them, it snaps them back to the pose held at activation. The base already
  // refuses a goal LONGER than the device and refuses partial NAMED goals; this
  // closes the unnamed-short case for the one binding whose goals are unshaped
  // steps. (It is also what stops a 6-value task goal, see SetDeviceTaskTarget.)
  if (hand_dof_ > 0 && target.size() != static_cast<std::size_t>(hand_dof_)) {
    hand_target_width_reject_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "hand target refused: %zu values for a %d-joint hand. A step commands "
                         "every joint or none — a partial one would snap the rest back to the "
                         "activation pose.",
                         target.size(), hand_dof_);
    return;
  }
  PushPendingTarget(device_idx, target, /*is_task=*/false);
}

void DemoCatchingController::SetDeviceTaskTarget(int device_idx,
                                                 std::span<const double> task6) noexcept {
  // There is no task lane here — no CLIK, no IK, nothing that turns a Cartesian
  // pose into joint values (see the header). The base's default forwards a task
  // goal to SetDeviceTarget, which on this controller would write the six pose
  // numbers into the first six HAND JOINTS as an unshaped step, clamped to the
  // joint limits and with the base's limit warning deliberately skipped because
  // "joint limits do not apply to a Cartesian goal". Refuse instead.
  static_cast<void>(task6);
  task_target_reject_count_.fetch_add(1, std::memory_order_relaxed);
  RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                       "task goal for device %d refused: this controller has no task lane. Send "
                       "joint values (goal_type 'joint') to step the hand.",
                       device_idx);
}

void DemoCatchingController::ApplyPendingTarget(int device_idx, std::span<const double> values,
                                                bool /*is_task*/) noexcept {
  // RT tick, once per surviving mailbox entry. Entries invalidated by a later
  // activation never reach here (the base's generation gate), which is what
  // keeps a goal queued while Inactive off the first tick after re-activation.
  if (device_idx != kCatchingHandDeviceIdx) {
    return;
  }
  const auto n = std::min(values.size(), hand_target_raw_.size());
  for (std::size_t i = 0; i < n; ++i) {
    hand_target_raw_[i] = values[i];
  }
  hand_target_width_ = static_cast<int>(n);
  hand_step_applied_count_.fetch_add(1, std::memory_order_relaxed);
}

void DemoCatchingController::ResetTargetInitialization() noexcept {
  // Lifecycle thread, from the base's on_activate. Under P-1 (a) this hook
  // may REQUEST a reset and must not perform one: the single writer of the
  // values is the RT tick.
  //
  // It needs no request of its own. The base bumped the activation generation
  // immediately before calling this, and ServiceResetRequests keys off exactly
  // that (D-23) — so the first tick of the new activation already reseeds the
  // hold latches from the pose the robot is actually in and drops a step the
  // operator issued before this activation.
  //
  // WHY NOT JUST CLEAR THEM HERE, as S4.0 did. It worked, because CM calls
  // on_activate with no tick running. It stops working the moment anything
  // else can request a reset — and E-STOP can, from a service callback, with
  // a tick mid-flight. Two writers of the same state under two different
  // synchronisation stories is the arrangement P-1 (a) exists to forbid, so
  // the one path that could have kept its shortcut gives it up too.
  reset_requested_.fetch_add(1, std::memory_order_release);
}

// ── Vision ingress (S5.2) ───────────────────────────────────────────────────

void DemoCatchingController::OnTrajectoryCloud(const sensor_msgs::msg::PointCloud2& msg) noexcept {
  // Non-RT: the controller LifecycleNode's default callback group, which CM
  // runs on `nrt_callback_executor` — shared with the lifecycle services, so
  // this must stay small. It does exactly three things: stamp, decode, store.
  //
  // The two instants are sampled TOGETHER and FIRST. They are the pair the
  // D-2 conversion stands on, and any work between them lands in the origin
  // delay as if the publisher had been slow.
  const std::int64_t recv_steady = SteadyNowNs();
  const std::int64_t recv_wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count();

  rtc::catching::TrajectorySnapshot snap{};
  rtc::catching::CovarianceSnapshot cov{};
  // The snapshot is stamped with the CURRENT activation generation (D-23).
  // The subscription outlives deactivation, so a message that arrives while
  // the controller is inactive is decoded and stored — and then refused by the
  // RT read, which is where the judgement belongs: the callback cannot know
  // whether an activation will happen before the next tick.
  const CloudReject reject =
      traj_input_.OnCloud(msg, recv_steady, recv_wall, ActivationGeneration(), snap, cov);
  // Published on EVERY message, accepted or refused, and BEFORE the early
  // return below — OnCloud has already filled the diagnostics either way. A
  // lane that is being refused and a lane that is silent look identical from
  // the RT side, so these counters are the only thing that tells them apart,
  // and they would be missing from exactly the case that needs them if this
  // sat after the return.
  ingress_diag_box_.Store(traj_input_.Snapshot());

  if (reject != CloudReject::kNone) {
    // Counted inside the ingress. Throttled here so a publisher that has
    // started producing garbage is visible without a per-message log on the
    // executor the lifecycle services share.
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "vision message refused (%s); check the reject counters",
                         CloudRejectName(reject));
    return;
  }

  // Covariance first. The two locks are written in the order the planner will
  // read them in, so the window in which they disagree is the one where the
  // covariance is NEWER than the trajectory — and the token check rejects
  // that pairing, while the reverse would let an old covariance pass as the
  // current one (D-22).
  cov_box_.Store(cov);
  traj_box_.Store(snap);
  // Wake the planner (D-7c). After BOTH stores, so the wake it causes reads a
  // matched pair. Non-blocking; the RT tick never does this.
  if (planner_params_.enabled) {
    static_cast<void>(
        CatchingPlannerThread::Signal(planner_wake_fd_.load(std::memory_order_acquire)));
  }

  const auto& diag = traj_input_.LastDiagnostics();
  if (diag.jump_m >= 0.0 && traj_jump_warn_m_ > 0.0 && diag.jump_m > traj_jump_warn_m_) {
    // L1 §4.4's J: how far two consecutive predictions of the SAME track
    // disagree at one instant. A warning, never a gate — the plan says so, and
    // the reason is that a jump is information about VISION, while the
    // supervisor's business is what to do about the ball. Without this the key
    // was parsed, validated and plumbed through to a comparison nobody made.
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "vision prediction jumped %.1f mm between consecutive snapshots "
                         "(warning threshold %.1f mm)",
                         diag.jump_m * 1000.0, traj_jump_warn_m_ * 1000.0);
  }

  if (diag.horizon_short) {
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "vision horizon %ld ms is shorter than the %ld ms this controller needs "
                         "(D-15): late catch points are not reaching the planner",
                         static_cast<long>(diag.horizon_ns / 1000000),
                         static_cast<long>(traj_horizon_min_ns_ / 1000000));
  }
}

// ── The tracking law (S5.3) ─────────────────────────────────────────────────

void DemoCatchingController::SeedArmCommand(const ControllerState& state) noexcept {
  // Start the command AT the measurement. Every later tick carries it forward
  // (D-6 evaluates along the commanded path), so this is the only moment the
  // two are tied together, and getting it wrong means the first commanded tick
  // is a step of whatever the initial gap happened to be.
  //
  // A command that is ALREADY carried is kept (#537 S7, C-32). From S7 the arm
  // is driven before the first plan — homed to the wait pose and held there
  // by the command, not the latch — so re-seeding at the measurement would
  // step the command by whatever servo error it had, which is the very jump
  // this function exists to prevent.
  const auto& dev = state.devices[kCatchingArmDeviceIdx];
  if (!arm_cmd_seeded_) {
    for (int i = 0; i < arm_dof_ && i < kDemoCatchingMaxArmDof; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      arm_q_cmd_[ui] = dev.positions[ui];
      arm_qd_cmd_[ui] = 0.0;
    }
    arm_cmd_seeded_ = true;
  }
  // NOT the QP failure streak. "Consecutive" (L7 §4.2) means "with no
  // successful solve in between", and the retry cycle an abort goes through —
  // ABORT_SAFE, RETREAT, ARMED, TRACKING, a fresh plan — contains no solve at
  // all. Clearing the streak here would make the fault latch unreachable: the
  // controller would retry a solve that cannot succeed, forever, one abort at
  // a time. It is cleared by a solve that works, and by a real reset.
  // The posture target is the command the trial STARTS from — the wait pose
  // once S7 has homed the arm there, else the pose the operator armed in. It
  // carries a small weight and only resolves the redundant degree of freedom,
  // so the choice matters for which elbow the arm keeps, not for where the
  // hand goes.
  if (combined_cache_.reorder_valid() && q_posture_.size() == combined_cache_.q().size()) {
    q_posture_ = combined_cache_.q();
    for (int i = 0; i < arm_dof_; ++i) {
      const int pq = combined_cache_.ext_to_pin_q(i);
      if (pq >= 0 && pq < q_posture_.size()) {
        q_posture_[pq] = arm_q_cmd_[static_cast<std::size_t>(i)];
      }
    }
  }
  track_err_ = 0.0;
  // The CLIK anchor and v_prev go with it: a carried-over v_prev would make
  // the first acceleration box a window around the PREVIOUS trial's velocity,
  // and the first solve would raise bound_conflict for no reason (L5 §4.2).
  clik_.ResetAnchor();
}

bool DemoCatchingController::PrepareLawTick(const ControllerState& state) noexcept {
  if (!clik_enabled_ || !reference_.has_value() || catch_frame_idx_ < 0 || !arm_readable_) {
    return false;  // nothing to run; the hold latch keeps the arm still
  }
  // ── The evaluation state (D-6) ───────────────────────────────────────────
  // CLIK is configured `evaluate_at_command`, so the cache it reads must carry
  // the COMMANDED arm configuration — its own previous QRef(). The hand
  // entries stay measured: the hand is commanded elsewhere (L6), and CLIK does
  // not check them.
  combined_cache_.ExtractFullState(state, arm_dof_, hand_dof_);
  q_eval_ = combined_cache_.q();
  v_eval_ = combined_cache_.v();
  for (int i = 0; i < arm_dof_; ++i) {
    const int pq = combined_cache_.ext_to_pin_q(i);
    const int pv = combined_cache_.ext_to_pin_v(i);
    if (pq >= 0 && pq < q_eval_.size()) {
      q_eval_[pq] = arm_q_cmd_[static_cast<std::size_t>(i)];
    }
    if (pv >= 0 && pv < v_eval_.size()) {
      // The commanded velocity, for the same reason as the position: the
      // Jacobian and the boxes must describe the path the command is on.
      v_eval_[pv] = arm_qd_cmd_[static_cast<std::size_t>(i)];
    }
  }
  combined_cache_.cache().Update(q_eval_, v_eval_);
  return true;
}

rtc::catching::Reason DemoCatchingController::RunTrackingTick(
    const ControllerState& state, const rtc::catching::TrajectorySnapshot& snapshot) noexcept {
  using rtc::catching::Mode;
  using rtc::catching::Reason;
  if (!PrepareLawTick(state)) {
    return Reason::kNone;
  }

  // The reference generator starts AT the current catch-frame pose, so the
  // first commanded step is the DS's own first step rather than a jump from
  // wherever the generator was left. It is seeded here, not in
  // SeedArmCommand, because the pose comes from the cache and the cache is
  // only current inside the tick.
  if (!reference_seeded_) {
    const pinocchio::SE3 pose = combined_cache_.ArmTcpPoseFromCache(catch_frame_idx_, -1);
    if (!reference_->Reset(pose.translation(), Eigen::Vector3d::Zero())) {
      return Reason::kRefSaturated;
    }
    const rtc::catching::BallTime t0{plan_.gamma_t0_ns};
    const rtc::catching::BallTime t1{plan_.gamma_t1_ns};
    if (!reference_->SetIntercept(
            Eigen::Vector3d(plan_.p_c[0], plan_.p_c[1], plan_.p_c[2]),
            rtc::catching::MakeGammaProfile(plan_.gamma_g0, plan_.gamma_gf, t0, t1, t0))) {
      return Reason::kPlanInvalid;
    }
    reference_seeded_ = true;
  }

  // After the freeze (COMMITTED, CLOSING) the law does not stop for the ball
  // lane (R-ORDER): the sampler reads the committed track's last snapshot
  // whatever its age, and past the horizon it extrapolates — recorded, not
  // obeyed. Before it, the same two facts are reasons to give the plan up.
  const bool committed = (mode_ == Mode::kCommitted || mode_ == Mode::kClosing);

  // ── The ball at the LEAD instant ────────────────────────────────────────
  // The snapshot is the one THIS tick already loaded (D-21: one Load per
  // tick). Loading again here would let a writer land between the two and
  // judge freshness on one snapshot while sampling another — with
  // `traj_hint_`, the sampler's cursor, carried across the pair.
  const rtc::catching::SampleEval sample =
      rtc::catching::SampleAt(snapshot, tick_now_lead_, traj_hint_);
  if (!sample.valid) {
    // Frozen, a snapshot that cannot be sampled at all leaves no trajectory to
    // finish the catch on: that is the long-stale case, not a pause.
    return committed ? Reason::kBallStaleLong : Reason::kBallStale;
  }
  if (sample.after_horizon) {
    if (!committed) {
      return Reason::kHorizonExtrap;
    }
    law_horizon_extrap_ = true;
  }

  rtc::catching::TargetState target;
  target.p = sample.p;
  target.v = sample.v;
  target.a = sample.a;

  // The γ profile and the sample share ONE origin so the two never drift: the
  // profile's own t0, expressed in the relative seconds the numeric core
  // takes. Mixing origins here is the bug plan §3 exists to prevent, and it
  // would look like a reference that is subtly early or late rather than wrong.
  const rtc::catching::BallTime origin{plan_.gamma_t0_ns};
  const double t_rel = rtc::catching::ProfileSeconds(tick_now_lead_, origin);
  return StepReferenceAndSolve(state, target, t_rel, /*count_saturation=*/true);
}

rtc::catching::Reason DemoCatchingController::StepReferenceAndSolve(
    const ControllerState& state, const rtc::catching::TargetState& target, double t_rel,
    bool count_saturation) noexcept {
  using rtc::catching::Reason;
  const double dt = state.dt;
  const rtc::catching::TranslationOutput ref = reference_->Step(target, t_rel, dt);
  RecordReference(ref);
  if (!ref.valid) {
    // The generator refuses non-finite input and keeps its state. Reporting
    // saturation as an abort would be wrong (it is a real, bounded reference),
    // but an invalid step means there is no reference at all this tick.
    return Reason::kRefSaturated;
  }
  // REF_SATURATED (#537 S7 Q6): a reference that stays on its bounds for
  // `supervisor.sat_ticks` ticks in a row is not following the ball any more.
  // One saturated tick is normal at a plan switch; a run of them is the
  // generator admitting the plan asks for more than it may give.
  bool saturated_too_long = false;
  if (count_saturation) {
    sat_streak_ = ref.saturated ? sat_streak_ + 1 : 0;
    saturated_too_long = sat_ticks_ > 0 && sat_streak_ >= sat_ticks_;
  } else {
    sat_streak_ = 0;
  }

  // ── CLIK ────────────────────────────────────────────────────────────────
  rtc::tsid::ClikReferenceGenerator::PositionAxisTarget clik_target;
  clik_target.position = ref.x;
  clik_target.linear_velocity_ff = ref.xd;
  clik_target.axis = Eigen::Vector3d(plan_.a_d[0], plan_.a_d[1], plan_.a_d[2]);
  // No angular feedforward: the approach axis of a fixed catch point does not
  // rotate, and a fabricated ω_ff would be the controller telling itself the
  // target is turning.
  const bool ok = clik_.Compute(combined_cache_.cache(), catch_frame_idx_, base_frame_idx_,
                                clik_target, q_posture_, dt, /*reseed_anchor=*/false);
  const auto& solve = clik_.LastSolve();
  RecordClikSolve(solve);
  if (!ok) {
    ++qp_fail_streak_;
    // `bound_conflict` is a DIFFERENT failure from a solver that did not
    // converge: the boxes disagreed, which the supervisor routes the same way
    // but which names a configuration problem rather than a numerical one.
    return solve.bound_conflict ? Reason::kJointConflict : Reason::kQpFailed;
  }
  qp_fail_streak_ = 0;

  const auto& q_ref = clik_.QRef();
  const auto& v_ref = clik_.VRef();
  for (int i = 0; i < arm_dof_ && i < kDemoCatchingMaxArmDof; ++i) {
    const int pq = combined_cache_.ext_to_pin_q(i);
    const int pv = combined_cache_.ext_to_pin_v(i);
    if (pq >= 0 && pq < q_ref.size()) {
      arm_q_cmd_[static_cast<std::size_t>(i)] = q_ref[pq];
    }
    if (pv >= 0 && pv < v_ref.size()) {
      arm_qd_cmd_[static_cast<std::size_t>(i)] = v_ref[pv];
    }
  }

  // ── TRACK_ERR (L7 §4.2) ─────────────────────────────────────────────────
  // The ONLY place the measured state enters the loop. D-6 keeps it out of
  // CLIK so the servo lag is not compensated twice; that makes this watchdog
  // the only thing that notices the arm is not where it was told to be.
  UpdateTrackError(state);
  if (track_err_abort_rad_ > 0.0 && track_err_ > track_err_abort_rad_) {
    return Reason::kTrackErr;
  }
  return saturated_too_long ? Reason::kRefSaturated : Reason::kNone;
}

void DemoCatchingController::UpdateTrackError(const ControllerState& state) noexcept {
  const auto& dev = state.devices[kCatchingArmDeviceIdx];
  double err_sq = 0.0;
  for (int i = 0; i < arm_dof_; ++i) {
    const auto ui = static_cast<std::size_t>(i);
    const double d = dev.positions[ui] - arm_q_cmd_[ui];
    err_sq += d * d;
  }
  track_err_ = std::sqrt(err_sq);
  // Recorded HERE and nowhere else: only the ticks that move the command
  // compute it, so a record filled outside would republish the last such
  // tick's number on every hold tick that followed (PROC-7).
  tick_record_.track_err_rad = track_err_;
}

void DemoCatchingController::RunJointSpaceAbort(const ControllerState& state) noexcept {
  // The QP-independent stop (A-S5-10). Reached when the abort was CAUSED by
  // the joint command layer, where routing the stop back through it is the one
  // thing that cannot be done. (L7 records the exemption: ABORT_SAFE takes
  // this stop whatever the cause, #537 S7 C-35.)
  abort_stopped_ = RampArmToStop(state);
}

bool DemoCatchingController::RampArmToStop(const ControllerState& state) noexcept {
  if (!arm_cmd_seeded_) {
    return true;
  }
  const auto n = static_cast<std::size_t>(std::min(arm_dof_, kDemoCatchingMaxArmDof));
  // The MARGINED box, not the device limits. `JointSpaceDecelStep` documents
  // its bounds as "the caller's box, already margined" and means it: ramping
  // against the raw limits ends the stop at a value CLIK was kept away from,
  // and the backend's clamp of it is invisible to everything upstream. Empty
  // when the box was incomplete, which the size guard below then catches as
  // "no honest ramp" — the same answer it already gave for a missing limit.
  const auto& lower = arm_q_min_margined_;
  const auto& upper = arm_q_max_margined_;
  if (arm_qdd_max_.size() < n || lower.size() < n || upper.size() < n) {
    // Without a limit there is no honest ramp. Freezing the command is the
    // conservative answer: the arm holds what it was last told, which is a
    // pose it was already at or moving toward.
    std::fill(arm_qd_cmd_.begin(), arm_qd_cmd_.end(), 0.0);
    return true;
  }
  const auto step = rtc::catching::JointSpaceDecelStep(
      std::span<double>(arm_q_cmd_.data(), n), std::span<double>(arm_qd_cmd_.data(), n),
      std::span<const double>(arm_qdd_max_.data(), n), std::span<const double>(lower.data(), n),
      std::span<const double>(upper.data(), n), n, state.dt);
  return step.valid && step.stopped;
}

void DemoCatchingController::StoreOraclePlan(rtc::catching::NowReal now) noexcept {
  // A-S5-8. One fixed catch point, from the operator's ground-truth
  // measurement, with the γ ramp spanning from now to the catch instant.
  //
  // Stored in the plan box rather than written to plan_ directly (S6-A), so
  // the oracle and the planner reach the tracking law through ONE path —
  // JudgePlan → AdoptPlan — and an oracle run exercises the code a planner run
  // depends on. The RT is this box's writer only because the profile enabled
  // the oracle; a profile enabling the planner as well is parked at configure.
  rtc::catching::PlanSnapshot plan{};
  plan.token.activation_generation = ActivationGeneration();
  // The track the RT is holding: the plan is FOR this ball, and JudgePlan (c)
  // refuses a plan for any other.
  plan.token.generation = last_track_generation_;
  plan.plan_id = ++oracle_plan_id_;
  plan.p_c = oracle_p_c_;
  plan.a_d = oracle_a_d_;
  plan.t_c_ns = now.ns + static_cast<std::int64_t>(oracle_t_c_offset_s_ * 1e9);
  plan.t_cmd_ns = plan.t_c_ns;
  plan.gamma_g0 = 0.0;
  plan.gamma_gf = oracle_gamma_f_;
  // The ramp starts NOW and ends at the catch instant: γ(t_c) = γ_f with
  // γ̇ = γ̈ = 0 there is what L4 §4.2's corollary needs, and starting at 0
  // makes the initial error independent of how far away the ball is.
  plan.gamma_t0_ns = now.ns;
  plan.gamma_t1_ns = plan.t_c_ns;
  // The instant this plan came into being — `plan_age_s` means one thing for
  // both writers, and JudgePlan's age bound applies to both.
  plan.publish_ns = now.ns;
  plan.valid = true;
  plan_box_.Store(plan);
}

void DemoCatchingController::AdoptPlan(const rtc::catching::PlanSnapshot& plan) noexcept {
  plan_ = plan;
  plan_active_ = true;
  admitted_plan_ = rtc::catching::AdmittedPlan{true, plan.plan_id};
}

void DemoCatchingController::StorePlannerRtState(const ControllerState& state,
                                                 rtc::catching::NowReal now) noexcept {
  // Built from scratch every tick (PROC-7 in spirit): a field this tick did
  // not compute is zero, never a previous tick's value.
  rtc::catching::PlannerRtState s{};
  s.valid = true;
  s.activation_generation = ActivationGeneration();
  s.rt_iteration = state.iteration;
  s.rt_state_ns = now.ns;
  s.reset_epoch = planner_reset_epoch_;
  s.mode = static_cast<std::uint8_t>(mode_);
  s.armed = arm_requested_.load(std::memory_order_relaxed);
  const int nv = std::min<int>(arm_dof_, static_cast<int>(rtc::catching::kMaxPlanNv));
  s.nv = nv;
  s.cmd_seeded = arm_cmd_seeded_;
  if (arm_cmd_seeded_) {
    for (int i = 0; i < nv && i < kDemoCatchingMaxArmDof; ++i) {
      const auto u = static_cast<std::size_t>(i);
      s.q_cmd[u] = arm_q_cmd_[u];
      s.qd_cmd[u] = arm_qd_cmd_[u];
    }
  } else if (arm_readable_ && state.num_devices > kCatchingArmDeviceIdx) {
    // Not yet tied to a command (TRACKING before a plan): the arm is where it
    // is measured, at rest as far as the planner's reach time is concerned.
    // `cmd_seeded` false tells the planner the velocity is not a command's.
    const auto& dev = state.devices[kCatchingArmDeviceIdx];
    for (int i = 0; i < nv && i < kDemoCatchingMaxArmDof; ++i) {
      s.q_cmd[static_cast<std::size_t>(i)] = dev.positions[static_cast<std::size_t>(i)];
    }
  }
  // The reference block of THIS tick's record: RecordReference fills it only on
  // a tick that ran the generator, and the record is fresh every tick.
  s.ref_valid = tick_record_.ref_valid;
  if (s.ref_valid) {
    s.ref_x = tick_record_.ref_x;
    s.ref_xd = tick_record_.ref_xd;
    s.gamma = tick_record_.ref_gamma;
    s.gamma_d = tick_record_.ref_gamma_d;
    s.gamma_dd = tick_record_.ref_gamma_dd;
  }
  s.plan_active = plan_active_;
  s.plan_id = plan_active_ ? plan_.plan_id : 0U;
  // The ramp SetIntercept was given (first adoption and replacements alike).
  s.ramp_valid = s.ref_valid && plan_active_;
  if (s.ramp_valid) {
    s.ramp_g0 = plan_.gamma_g0;
    s.ramp_gf = plan_.gamma_gf;
    s.ramp_t0_ns = plan_.gamma_t0_ns;
    s.ramp_t1_ns = plan_.gamma_t1_ns;
  }
  s.track_seen = track_seen_;
  s.track_generation = last_track_generation_;
  planner_rt_box_.Store(s);
}

// ── E-STOP and fault hooks (P-1 (a): request only) ──────────────────────────
//
// Every body here is a store. None of them touches the hold latches, the step
// target, the supervisor mode or any counter the tick owns — that is the whole
// of P-1 (a), and it is also what makes these safe to call from a service
// callback while a tick is mid-flight.
//
// The base's own comment warns that an override must not branch on the global
// latch, because CM propagates both directions while the latch still reads
// true. None of these reads it.

void DemoCatchingController::TriggerEstop() noexcept {
  estop_requested_.store(true, std::memory_order_release);
  // The epoch moves on the TRIGGER as well as on the clear. A trigger→clear
  // pair that lands entirely between two ticks would otherwise be invisible:
  // the flag is back to false and the tick would have nothing to tell it that
  // a stop happened at all, so q_c would carry on from before the stop.
  estop_epoch_.fetch_add(1, std::memory_order_release);
}

void DemoCatchingController::ClearEstop() noexcept {
  estop_requested_.store(false, std::memory_order_release);
  estop_epoch_.fetch_add(1, std::memory_order_release);
  // NOT cleared here: fault_latched_ (P-1 (d) — the fault path is separate and
  // a global clear must not launder it) and arm_requested_, which the tick
  // lowered on the stop. Leaving the arm latch down is what makes "no
  // automatic resume" (P-1 (c)) a property of the mechanism rather than of
  // whoever is holding the clear button.
}

bool DemoCatchingController::IsEstopped() const noexcept {
  return estop_requested_.load(std::memory_order_acquire);
}

void DemoCatchingController::ResetFault() noexcept {
  // A request, not a clear: the tick decides whether the cause is gone. The
  // base's contract says exactly this ("the clear happens on the RT thread,
  // one tick later, and can be refused there"), and HasLatchedFault is how the
  // caller finds out.
  fault_reset_epoch_.fetch_add(1, std::memory_order_release);
}

bool DemoCatchingController::HasLatchedFault() const noexcept {
  return fault_latched_.load(std::memory_order_acquire);
}

// ── Supervisor (S1.8 transition table, S7 driver) ───────────────────────────

void DemoCatchingController::AdvanceMode(rtc::catching::Reason reason) noexcept {
  rtc::catching::Mode next = mode_;
  // A reason with no row in this mode is INAPPLICABLE here, which the table
  // expresses by having no row — not an error and not a fallback. Leaving the
  // mode alone is the documented contract of LookupTransition, and it is why
  // this driver can be this small: every "may I go there from here" question
  // is already answered by the data, and S7.2 grows the driver rather than
  // the table.
  if (rtc::catching::LookupTransition(rtc::catching::kTransitionTable, mode_, reason, next)) {
    const rtc::catching::Mode prev = mode_;
    mode_ = next;
    if (next != prev) {
      OnModeEntered(prev);
    }
  }
  last_reason_ = reason;
}

void DemoCatchingController::OnModeEntered(rtc::catching::Mode prev) noexcept {
  using rtc::catching::Mode;
  using rtc::catching::Outcome;
  switch (mode_) {
    case Mode::kArmed:
      // RETREAT → ARMED is the re-arm boundary (L7 §4.8). IDLE → ARMED is not
      // a reset: nothing of a trial exists yet.
      if (prev == Mode::kRetreat) {
        ResetForRearm();
      }
      break;
    case Mode::kCommitted:
      // The freeze (L3 §4.11, R-TRACK). From here the catch instant, the hand's
      // close instant and the track are fixed: the planner may keep publishing,
      // and none of it is taken.
      trial_committed_ = true;
      committed_t_c_ns_ = plan_.t_c_ns;
      committed_t_cmd_ns_ = rtc::catching::detail::SatSub(plan_.t_c_ns, t_close_e2e_ns_);
      committed_generation_ = plan_.token.generation;
      if (hand_seq_enabled_) {
        static_cast<void>(hand_seq_.Commit(rtc::catching::BallTime{plan_.t_c_ns}));
      }
      break;
    case Mode::kHold:
      hold_entry_ns_ = tick_now_.ns;
      break;
    case Mode::kRetreat: {
      // The plan is over, whether it ended in a catch or an abort. Dropping it
      // here is what lets the next trial build its own — and what stops a plan
      // whose t_c is now in the past from being picked up again.
      plan_active_ = false;
      reference_seeded_ = false;
      retreat_stage_ = RetreatStage::kStop;
      // A HOLD that ends has already judged the attempt (EvaluateDecelOrHold).
      // Any other way in is an abort of an attempt, if one was under way.
      if (prev != Mode::kHold && trial_active_) {
        outcome_ = Outcome::kAborted;
      }
      // RETREAT never moves the hand (#537 S7, 2026-09-24; this replaces the
      // Q12/Q14 split by verdict). A hand that closed stays closed until the
      // arm is back at the wait pose, and RunRetreatMotion releases it there,
      // whatever the verdict. The verdict comes from the fingertips only, and
      // a ball lying on the links or the palm reads Missed (sim 260923_2336).
      // Opening at the catch point drops it. A close that is armed but not yet
      // issued is cancelled here. The hand is still at q_pre, so the Release
      // moves nothing: it only stops the close from firing during the return.
      if (hand_seq_enabled_ && !hand_seq_.CloseIssued()) {
        hand_seq_.Release();
      }
      break;
    }
    case Mode::kIdle:
      homing_ = false;
      homing_done_ = false;
      break;
    case Mode::kTracking:
    case Mode::kApproach:
    case Mode::kClosing:
    case Mode::kDecel:
    case Mode::kAbortSafe:
    case Mode::kFault:
      break;
  }
}

void DemoCatchingController::NoteLawVerdict(rtc::catching::Reason law) noexcept {
  using rtc::catching::Reason;
  // BOTH CLIK failure reasons count toward the streak, because both increment
  // it: `kJointConflict` is a solve that could not honour its own boxes, and a
  // box that keeps conflicting is exactly as unrecoverable as a solver that
  // keeps failing. Counting only one of them leaves a controller that loops
  // APPROACH → ABORT_SAFE → RETREAT → ARMED → TRACKING forever without ever
  // escalating. Checked in every mode the law runs in (#537 S7), not only in
  // APPROACH: a streak that completes after the freeze is the same streak.
  const bool clik_failed = (law == Reason::kQpFailed || law == Reason::kJointConflict);
  if (clik_failed && n_qp_fault_ > 0 && qp_fail_streak_ >= n_qp_fault_) {
    // L7 §4.2: a streak of QP failures is not a transient. The latch is what
    // stops the controller from retrying forever against a solve that is
    // never going to succeed.
    fault_latched_.store(true, std::memory_order_release);
  }
}

namespace {

/// The law verdicts that end a trial (R-PREC's "law failure / fatal" tier).
[[nodiscard]] bool IsFatalLawReason(rtc::catching::Reason r) noexcept {
  using rtc::catching::Reason;
  return r == Reason::kQpFailed || r == Reason::kJointConflict || r == Reason::kTrackErr ||
         r == Reason::kRefSaturated || r == Reason::kBallStaleLong || r == Reason::kPlanInvalid ||
         r == Reason::kParamsTbd;
}

}  // namespace

bool DemoCatchingController::ArmCommandStopped() const noexcept {
  for (int i = 0; i < arm_dof_ && i < kDemoCatchingMaxArmDof; ++i) {
    if (arm_qd_cmd_[static_cast<std::size_t>(i)] != 0.0) {
      return false;
    }
  }
  return true;
}

bool DemoCatchingController::ArmAtWaitPose(const ControllerState& state) const noexcept {
  if (!arm_readable_ || state.num_devices <= kCatchingArmDeviceIdx) {
    return false;
  }
  const auto& dev = state.devices[kCatchingArmDeviceIdx];
  for (int i = 0; i < arm_dof_ && i < kDemoCatchingMaxArmDof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    // Written as "within" so a NaN reading is not at the pose.
    if (!(std::abs(dev.positions[u] - wait_pose_[u]) <= pose_tol_) ||
        !(std::abs(dev.velocities[u]) <= homing_qd_tol_)) {
      return false;
    }
  }
  return true;
}

bool DemoCatchingController::HandSettledAtPre(const ControllerState& state) const noexcept {
  if (!hand_readable_ || state.num_devices <= kCatchingHandDeviceIdx) {
    return false;
  }
  const auto& dev = state.devices[kCatchingHandDeviceIdx];
  const auto& hand = params_.hand;
  for (int i = 0; i < hand_dof_ && i < hand.dof; ++i) {
    const auto u = static_cast<std::size_t>(i);
    if (!(std::abs(dev.positions[u] - hand.q_pre[u]) <= hand.q_tol) ||
        !(std::abs(dev.velocities[u]) <= hand.qd_tol)) {
      return false;
    }
  }
  return true;
}

DemoCatchingController::ReasonDecision DemoCatchingController::EvaluateIdle(
    const ControllerState& state) noexcept {
  using rtc::catching::Reason;
  if (!trials_enabled_) {
    // No law, no homing: nothing moves in this configuration, so readiness is
    // the operator's latch alone — the S5/S6 behaviour, kept for the
    // configurations that cannot run a trial (no model, a unit fixture).
    return {Reason::kNone, true};
  }
  // Q13: an arm that is already at the wait pose skips homing — only the hand
  // is told to wait at q_pre. Decided HERE, on the measured state, so an
  // aligned arm arms on its first armed tick rather than one tick later.
  if (!homing_ && !homing_done_ && ArmCommandStopped() && ArmAtWaitPose(state)) {
    homing_done_ = true;
    if (hand_seq_enabled_) {
      hand_seq_.Ready();
    }
  }
  const bool arm_ready = homing_done_ && !homing_ && ArmCommandStopped() && ArmAtWaitPose(state);
  // §4.5-6 (Q4): the hand is AT q_pre and at rest — the contact baseline is
  // learned from a hand that is not moving. A hand-step profile has no
  // sequencer and makes no claim about the hand.
  const bool hand_ready = !hand_seq_enabled_ || HandSettledAtPre(state);
  return {Reason::kNone, arm_ready && hand_ready};
}

DemoCatchingController::ReasonDecision DemoCatchingController::EvaluateRetreat() noexcept {
  using rtc::catching::Reason;
  // R-WATCHDOG: the return is a motion like any other, and TRACK_ERR has a row
  // out of RETREAT. (Measured by the motion stage's previous tick.)
  if (retreat_stage_ == RetreatStage::kReturn && track_err_abort_rad_ > 0.0 &&
      track_err_ > track_err_abort_rad_) {
    return {Reason::kTrackErr, true};
  }
  if (retreat_stage_ != RetreatStage::kRelease) {
    return {Reason::kNone, false};
  }
  // Back at the wait pose; re-arm once the hand is at q_pre and settled (the
  // sequencer's Release ends in Preshape exactly then). hand_out_ is last
  // tick's hand stage, which ran after the arrival tick's Release, so it can
  // never show the pre-release Hold as ready.
  const bool hand_ready =
      !hand_seq_enabled_ ||
      (hand_out_.active && hand_out_.phase == rtc::catching::HandPhase::kPreshape &&
       hand_out_.at_target);
  return {Reason::kNone, hand_ready};
}

rtc::catching::Reason DemoCatchingController::RunDecelLawTick(
    const ControllerState& state) noexcept {
  using rtc::catching::Reason;
  if (!PrepareLawTick(state)) {
    return Reason::kNone;
  }
  // τ on the lead axis from the entry instant (plan §3). Floored at 0: the
  // entry tick evaluates τ = 0 exactly, and a later tick whose clock read
  // jittered below the entry's must not ask the target for a negative τ.
  const double tau =
      std::max(0.0, static_cast<double>(tick_now_lead_.ns - decel_t_s_ns_) * rtc::catching::kNsToS);
  const rtc::catching::DecelTarget tgt =
      rtc::catching::EvaluateDecelTarget(decel_entry_, decel_a_dec_, tau);
  if (!tgt.valid) {
    // a_dec is refused at configure and the entry is the reference's own
    // (finite) state, so this is a value the law needs that is not there.
    return Reason::kParamsTbd;
  }
  decel_stopped_ = tgt.stopped;
  rtc::catching::TargetState target;
  target.p = tgt.p_v;
  target.v = tgt.v_v;
  target.a = tgt.a_v;
  return StepReferenceAndSolve(state, target, tau, /*count_saturation=*/false);
}

rtc::catching::Reason DemoCatchingController::EnterDecel(const ControllerState& state) noexcept {
  // L7 §4.3 / R-DECEL-ENTRY. The entry state is the reference's CURRENT state
  // — the previous tick's step output, i.e. the reference for THIS tick — and
  // t_s is this tick's lead instant, so the τ = 0 step taken below sees
  // e = x_s − p_v(0) = 0 and ė = ẋ_s − v_v(0) = 0 exactly (G7-B): the
  // generator is not reset, only its target changes.
  decel_entry_.x_s = reference_->Position();
  decel_entry_.xdot_s = reference_->Velocity();
  decel_t_s_ns_ = tick_now_lead_.ns;
  decel_stopped_ = false;
  // γ ≡ 1: the reference follows the virtual target itself, not a blend of it
  // with the catch point (L7 §4.3). The ramp is a formality of the profile's
  // shape — g0 = gf — so its length only has to be positive.
  constexpr std::int64_t kFlatRampNs = 1'000'000;
  const rtc::catching::BallTime t_s{decel_t_s_ns_};
  if (!reference_->SetIntercept(
          reference_->Intercept(),
          rtc::catching::MakeGammaProfile(1.0, 1.0, t_s,
                                          rtc::catching::BallTime{t_s.ns + kFlatRampNs}, t_s))) {
    return rtc::catching::Reason::kParamsTbd;
  }
  return RunDecelLawTick(state);
}

DemoCatchingController::ReasonDecision DemoCatchingController::EvaluateCommitted(
    const ControllerState& state) noexcept {
  using rtc::catching::Mode;
  using rtc::catching::Reason;
  const bool closing = (mode_ == Mode::kClosing);

  // CLOSING → DECEL at t_c on the lead axis. The first DECEL step is taken on
  // this tick, so the reference never sees a tick with no target.
  if (closing &&
      rtc::catching::DecelDue(tick_now_lead_, rtc::catching::BallTime{committed_t_c_ns_})) {
    const Reason law = EnterDecel(state);
    NoteLawVerdict(law);
    if (IsFatalLawReason(law)) {
      return {law, true};
    }
    return {Reason::kNone, true};
  }

  // R-TRACK (Q15): after the freeze the law samples the COMMITTED track. A box
  // holding another generation is stale for this catch — the frozen plan goes
  // on (recorded), and the committed snapshot ages until it is too old.
  const std::int64_t recv = law_snapshot_.token.traj_recv_ns;
  const std::int64_t age = recv > 0 ? rtc::catching::AgeNs(tick_now_, rtc::catching::NowReal{recv})
                                    : std::numeric_limits<std::int64_t>::max();
  const bool other_track = traj_view_.age_ns >= 0 && !traj_view_.stale &&
                           last_track_generation_ != committed_generation_;
  const bool stale = other_track || age > t_stale_ns_;
  const bool stale_long = age > t_stale_ns_ + stale_committed_max_ns_;

  // R-ORDER: the law runs whatever the ball lane says.
  const Reason law = RunTrackingTick(state, law_snapshot_);
  NoteLawVerdict(law);
  // R-PREC: a law failure outranks the long-stale abort, which outranks time.
  if (IsFatalLawReason(law)) {
    return {law, true};
  }
  if (stale_long) {
    return {Reason::kBallStaleLong, true};
  }
  // COMMITTED → CLOSING once the close command has gone out (R-CLOSE). The
  // sequencer owns that instant; a hand-step profile has no sequencer and
  // uses the same rule on the same t_cmd.
  if (!closing) {
    const bool close_out = hand_seq_enabled_
                               ? hand_seq_.CloseIssued()
                               : rtc::catching::HandCommandDueRounded(
                                     tick_now_, rtc::catching::BallTime{committed_t_cmd_ns_},
                                     static_cast<std::int64_t>(state.dt * 1e9));
    if (close_out) {
      return {Reason::kNone, true};
    }
  }
  // Record-only (R-PREC's last tier): only on a tick with no edge, and each
  // has a self-loop row in both modes.
  if (stale) {
    return {Reason::kBallStaleCommitted, false};
  }
  if (law_horizon_extrap_) {
    return {Reason::kHorizonExtrap, false};
  }
  if (closing && hand_out_.timeout) {
    return {Reason::kHandTimeout, false};
  }
  if (tip_stale_now_) {
    return {Reason::kTipStale, false};
  }
  return {Reason::kNone, false};
}

DemoCatchingController::ReasonDecision DemoCatchingController::EvaluateDecelOrHold(
    const ControllerState& state) noexcept {
  using rtc::catching::Mode;
  using rtc::catching::Outcome;
  using rtc::catching::Reason;
  const Reason law = RunDecelLawTick(state);
  NoteLawVerdict(law);
  // DECEL and HOLD have no REF_SATURATED row: a saturated reference while
  // stopping is the stop taking longer, not a failed catch.
  if (IsFatalLawReason(law) && law != Reason::kRefSaturated) {
    return {law, true};
  }
  if (mode_ == Mode::kDecel) {
    if (decel_stopped_) {
      return {Reason::kNone, true};
    }
    if (hand_out_.timeout) {
      return {Reason::kHandTimeout, false};
    }
    if (tip_stale_now_) {
      return {Reason::kTipStale, false};
    }
    return {Reason::kNone, false};
  }
  // HOLD for T_hold, then judge the attempt (S7.3) and retreat.
  if (tick_now_.ns - hold_entry_ns_ >= t_hold_ns_) {
    outcome_ = JudgeOutcome();
    return {Reason::kNone, true};
  }
  if (tip_stale_now_) {
    return {Reason::kTipStale, false};
  }
  return {Reason::kNone, false};
}

DemoCatchingController::ReasonDecision DemoCatchingController::EvaluateReason(
    const ControllerState& state, const rtc::catching::TrajectorySnapshot& snapshot) noexcept {
  using rtc::catching::Mode;
  using rtc::catching::Reason;

  // R-PREC (#537 S7): ESTOP > fault reset / escalation > readiness lost >
  // law failure > time advance > record-only. Ordered by authority, not by
  // likelihood — E-STOP is the one condition whose handling must not depend
  // on what else is true this tick.
  if (estop_active_) {
    return {Reason::kEstop, true};
  }
  if (fault_reset_serviced_) {
    // Consumed here rather than in ServiceResetRequests so the FSM edge and
    // the state reset happen on the same tick and in the table's order.
    fault_reset_serviced_ = false;
    return {Reason::kFaultReset, true};
  }
  if (fault_latched_.load(std::memory_order_relaxed)) {
    // The latch is raised while the controller is still in ABORT_SAFE (the
    // failure that raised it sent it there). THAT is the escalation L7 §4.2
    // names — an abort that cannot recover — and it is the only edge out of
    // ABORT_SAFE that does not lead back toward another attempt.
    if (mode_ == Mode::kAbortSafe) {
      return {Reason::kAbortEscalated, true};
    }
    // Anywhere else, hold. In Mode::kFault the table has rows only for
    // kFaultReset and kEstop, so holding is what makes the latch persistent —
    // a property of the driver, not of an invented "still faulted" reason.
    return {Reason::kNone, false};
  }
  if (!armable_ || !arm_requested_.load(std::memory_order_relaxed)) {
    // §4.5's readiness conditions are not met — either the profile is not
    // armable (G0-C) or the operator has not armed this controller.
    //
    // kParamsTbd is the DOCUMENTED reuse for "an ARMED precondition stopped
    // holding" (L7 §4.5 has no dedicated reason; transition_table.hpp's header
    // records the reuse). It self-loops in IDLE, sends ARMED and RETREAT to
    // IDLE, and sends every mode that can be CARRYING MOTION to ABORT_SAFE.
    // A RETREAT that was moving lands in IDLE with its command still carried:
    // IDLE owns the stop ramp (R-IDLE, RunIdleMotion).
    //
    // ABORT_SAFE is the one mode that must NOT answer it. Its exit is the
    // completion of the stop (`abort_stopped_` below), and returning a reason
    // here would pre-empt that check every tick — the ramp would keep running
    // and the machine would never reach RETREAT, so a disarm during an abort
    // would strand the controller in the state it was trying to leave. Falling
    // through is what closes the cycle: APPROACH → ABORT_SAFE → (stopped) →
    // RETREAT → IDLE, with the arm ramped down rather than frozen.
    if (mode_ != Mode::kAbortSafe) {
      return {Reason::kParamsTbd, true};
    }
  }
  // ── Modes that do not ask vision anything (R-ORDER) ──────────────────────
  // BEFORE the vision lane, because none of them is a question about the
  // ball. Asking first would strand the controller: after a catch or an abort
  // the ball has landed, the lane goes quiet within `io.t_stale`, and a stale
  // snapshot would then answer for them — a DECEL that never stops, a HOLD
  // that never ends, a RETREAT that never returns.
  switch (mode_) {
    case Mode::kAbortSafe:
      // Leave only when the arm has actually stopped (L7 §4.1). Advancing on
      // the tick the abort STARTS would make ABORT_SAFE a label rather than a
      // state, and the next trial would begin while the arm is still moving.
      return {Reason::kNone, abort_stopped_};
    case Mode::kIdle:
      return EvaluateIdle(state);
    case Mode::kRetreat:
      return EvaluateRetreat();
    case Mode::kDecel:
    case Mode::kHold:
      return EvaluateDecelOrHold(state);
    case Mode::kCommitted:
    case Mode::kClosing:
      // Frozen: the law runs on the committed track whatever the lane says.
      traj_new_track_ = false;
      return EvaluateCommitted(state);
    case Mode::kArmed:
    case Mode::kTracking:
    case Mode::kApproach:
    case Mode::kFault:
      break;
  }

  // ── The vision lane (S5.2) ───────────────────────────────────────────────
  // From here the controller is armed and the profile is clean, so what is
  // left to decide is what the prediction says. The ORDER matters: a track
  // change outranks staleness because it says the old plan was about a
  // different ball, and a stale snapshot of the right ball is a reason to
  // stop planning, not to plan against something else.
  const bool tracking = (mode_ == Mode::kTracking || mode_ == Mode::kApproach);

  if (tracking && traj_new_track_) {
    traj_new_track_ = false;
    return {Reason::kTrackChanged, true};
  }
  traj_new_track_ = false;

  // Whether there is a prediction worth acting on. `expired` is grouped with
  // `stale` for the ARMING question and kept apart for the ABORT question:
  // both mean "do not start tracking on this", but they call for different
  // reasons once tracking has started, and the fixes differ (a publisher that
  // stopped against a horizon that is too short, D-15).
  const bool usable = !traj_view_.stale && !traj_view_.expired;

  if (!usable) {
    if (tracking) {
      return traj_view_.stale ? ReasonDecision{Reason::kBallStale, true}
                              : ReasonDecision{Reason::kHorizonExtrap, true};
    }
    // Not tracking yet, and nothing to report: waiting for a ball IS the
    // ARMED state.
    return {Reason::kNone, false};
  }

  if (mode_ == Mode::kArmed) {
    // R-TRACK (Q15): after a re-arm the previous trial's ball is not a new
    // one. Wait for a track number the last trial did not use — a lane that
    // keeps publishing the ball that was just caught (or missed) must not
    // start another trial on it.
    if (last_trial_generation_valid_ && snapshot.token.generation == last_trial_generation_) {
      return {Reason::kNone, false};
    }
    return {Reason::kNone, true};
  }

  // A usable prediction. From TRACKING, `kNone` would advance to APPROACH,
  // and APPROACH means "a valid plan is being followed" — so the edge is taken
  // only on a plan this tick admitted, and otherwise the honest answer is
  // `kNoCatchablePlan`, which the table self-loops in TRACKING.
  if (mode_ == Mode::kTracking) {
    // `arm_readable_` gates the EDGE, not just the law. SeedArmCommand ties
    // the command to the measurement and that is the only moment the two are
    // tied, so seeding off an unreadable device latches whatever the mirror
    // happens to hold — zeros for a device that never reported, the previous
    // sample for a hole — and `arm_cmd_seeded_` then makes it permanent. The
    // arm is silenced while unreadable, so the damage only appears on the
    // first readable tick, as a step away from the pose the arm is actually
    // in. Waiting self-loops in TRACKING, which is where a controller that
    // can see the ball but not its own arm belongs.
    if (!plan_active_ && clik_enabled_ && arm_readable_ &&
        plan_refusal_ == rtc::catching::PlanRefusal::kNone) {
      // The plan in the box passed every admission check this tick (L3 §5.2
      // (a)-(g)) — the planner's, or the oracle's stand-in (A-S5-8), through
      // the same path. Taking it HERE, on the tick the supervisor is ready for
      // one, is what makes the TRACKING → APPROACH edge the real edge rather
      // than a special case.
      AdoptPlan(plan_in_);
      // An ATTEMPT (S8 counts attempts against successes): only this edge.
      // A replacement in APPROACH is the same attempt and counts separately.
      plan_admitted_count_.fetch_add(1, std::memory_order_relaxed);
      trial_active_ = true;
      law_snapshot_ = snapshot;
      SeedArmCommand(state);
      reference_seeded_ = false;
      return {Reason::kNone, true};
    }
    // The ball is seen and no admissible plan exists for it — none published,
    // or one this tick refused (stale activation, another track, too old,
    // already taken, from before the last reset, or already inside the freeze
    // window). All of those are "no plan" to the supervisor, which is exactly
    // what NO_CATCHABLE_PLAN says; a reason per refusal would be a change to
    // the frozen message (D-20), and the refusal itself is observable through
    // GetLastPlanRefusal().
    return {Reason::kNoCatchablePlan, true};
  }

  if (mode_ == Mode::kApproach) {
    // A replacement plan (§4.7, S6-B). The planner already applied the
    // switching rule; the RT adds its own freeze check (decision G) so a late
    // publish cannot move a catch point the supervisor is about to commit to.
    // Re-targeted in place — no Reset — so the reference state is continuous;
    // the new plan's γ ramp starts from the γ the reference is at.
    if (plan_active_ && plan_refusal_ == rtc::catching::PlanRefusal::kNone &&
        plan_in_.plan_id != plan_.plan_id) {
      const std::int64_t to_tc = plan_.t_c_ns - tick_now_.ns;
      if (plan_freeze_ns_ <= 0 || to_tc > plan_freeze_ns_) {
        // The new ramp starts from the γ the reference is at THIS tick, and not
        // before this tick: the planner's gamma_g0 is the γ it saw when it
        // planned (one search earlier), and a ramp whose start is already in
        // the past would be part-way up on its first step — either is a step
        // in γ, which reaches e and u_des through γ·(o − p_c) and γ̈·(o − p_c)
        // (2026-09-23 /code-review).
        double g_now = plan_in_.gamma_g0;
        if (reference_seeded_ && reference_.has_value()) {
          double g = 0.0;
          double gd = 0.0;
          double gdd = 0.0;
          reference_->Gamma().Eval(rtc::catching::ProfileSeconds(
                                       tick_now_lead_, rtc::catching::BallTime{plan_.gamma_t0_ns}),
                                   g, gd, gdd);
          if (std::isfinite(g)) {
            g_now = g;
          }
        }
        AdoptPlan(plan_in_);
        plan_.gamma_g0 = g_now;
        plan_.gamma_t0_ns = std::max(plan_.gamma_t0_ns, tick_now_lead_.ns);
        plan_replaced_count_.fetch_add(1, std::memory_order_relaxed);
        if (reference_seeded_ && reference_.has_value()) {
          const rtc::catching::BallTime t0{plan_.gamma_t0_ns};
          const rtc::catching::BallTime t1{plan_.gamma_t1_ns};
          if (!reference_->SetIntercept(
                  Eigen::Vector3d(plan_.p_c[0], plan_.p_c[1], plan_.p_c[2]),
                  rtc::catching::MakeGammaProfile(plan_.gamma_g0, plan_.gamma_gf, t0, t1, t0))) {
            return {Reason::kPlanInvalid, true};
          }
        }
      }
    }
    // Following a plan. The law runs here and its verdict IS this tick's
    // reason: a healthy tick reports nothing.
    const Reason law = RunTrackingTick(state, snapshot);
    NoteLawVerdict(law);
    if (law != Reason::kNone) {
      return {law, true};
    }
    // APPROACH → COMMITTED at t_c − now ≤ T_freeze (real axis). Only on a
    // healthy tick (R-PREC: a failure outranks time). `plan_freeze_ns_` is
    // positive whenever trials are enabled — SetupSupervisor parks otherwise.
    if (plan_freeze_ns_ > 0 &&
        rtc::catching::CommitDue(tick_now_, rtc::catching::BallTime{plan_.t_c_ns},
                                 plan_freeze_ns_)) {
      return {Reason::kNone, true};
    }
    return {Reason::kNone, false};
  }

  // Mode::kFault is handled above (latched) or by the fault reset; nothing
  // here moves it.
  return {Reason::kNone, false};
}

// ── Reset service (P-1 (a)/(b): the tick is the only writer) ────────────────

void DemoCatchingController::ResetForRearm() noexcept {
  // L7 §4.8's re-arm list (S7.4), RETREAT → ARMED. The table in the header is
  // the authority on which member is here and which is exempt, and why.
  //
  // R-TRACK (Q15): this trial's ball is refused until a new track arrives —
  // taken before the plan is forgotten, since the plan names the track.
  if (trial_committed_) {
    last_trial_generation_ = committed_generation_;
    last_trial_generation_valid_ = true;
  } else if (trial_active_) {
    last_trial_generation_ = plan_.token.generation;
    last_trial_generation_valid_ = true;
  }
  // The plan lane, as on every reset: forget which plan was taken, refuse
  // every plan published before this instant (JudgePlan (f)) and tell the
  // planner (reset_epoch) — in the SAME place, because admission is decided by
  // the floor and the planner by the epoch, and a re-arm that moved only one
  // of them lets a plan published just before the stop into the next trial.
  plan_ = rtc::catching::PlanSnapshot{};
  plan_active_ = false;
  admitted_plan_ = rtc::catching::AdmittedPlan{};
  reset_floor_ns_ = tick_now_.ns;
  ++planner_reset_epoch_;
  reference_seeded_ = false;
  traj_hint_ = 0;
  // The carried command stays (it IS the wait pose); only its velocity is
  // zeroed, which the return has already brought to rest.
  std::fill(arm_qd_cmd_.begin(), arm_qd_cmd_.end(), 0.0);
  track_err_ = 0.0;
  trial_active_ = false;
  trial_committed_ = false;
  committed_t_c_ns_ = 0;
  committed_t_cmd_ns_ = 0;
  committed_generation_ = 0;
  law_snapshot_ = rtc::catching::TrajectorySnapshot{};
  decel_entry_ = rtc::catching::DecelEntryState{};
  decel_t_s_ns_ = 0;
  decel_stopped_ = false;
  hold_entry_ns_ = 0;
  sat_streak_ = 0;
  law_horizon_extrap_ = false;
  retreat_stage_ = RetreatStage::kStop;
  contact_.ResetForRearm();
  tip_baseline_n_.fill(0);
  window_confirmed_seen_ = false;
  window_stale_seen_ = false;
  homing_ = false;
  homing_done_ = true;  // RETREAT ended at the wait pose
  if (hand_seq_enabled_) {
    hand_seq_.Ready();
  }
}

void DemoCatchingController::ResetTrialState(bool reset_mode) noexcept {
  // L7 §4.8's list for an activation, an E-STOP or an explicit reset. The
  // header's reset table is the authority; the rule that comes with it is that
  // a new stateful member is added THERE and HERE in the same change — a
  // member missing from this function is a trial that starts with the previous
  // trial's state, which is the failure mode §4.8 was written for.
  arm_hold_ = HoldLatch{};
  hand_hold_ = HoldLatch{};
  hand_target_raw_.fill(0.0);
  hand_target_width_ = 0;
  // Queued goals are dropped rather than carried over: a target issued before
  // this reset was issued against the state the reset just discarded. The
  // base's generation gate already stops goals from a previous ACTIVATION, but
  // an E-STOP does not bump that generation, so this is the half of the
  // problem the gate does not cover.
  DiscardPendingTargets();
  // The MODE is not always the reset's to set. An activation starts a fresh
  // controller and IDLE is where that starts; an E-STOP does not, and forcing
  // IDLE there would erase a latched FAULT that the transition table
  // deliberately keeps through a stop ({FAULT, ESTOP} → FAULT, P-1 (d)). The
  // reason code drives the mode in that case, which is the whole point of
  // having a table.
  if (reset_mode) {
    mode_ = rtc::catching::Mode::kIdle;
    last_reason_ = rtc::catching::Reason::kNone;
  }
  // The plan lane (S6-A). The box is NOT cleared here — the RT is not its
  // writer when a planner runs. Instead: forget which plan was taken, refuse
  // every plan published before this instant (JudgePlan (f) — an E-STOP does
  // not move the activation generation, so (b) alone would let a plan from
  // the stopped trial into the next one), and tell the planner a reset
  // happened (L7 §4.8) through PlannerRtState::reset_epoch.
  plan_ = rtc::catching::PlanSnapshot{};
  plan_active_ = false;
  admitted_plan_ = rtc::catching::AdmittedPlan{};
  reset_floor_ns_ = SteadyNowNs();
  ++planner_reset_epoch_;
  arm_cmd_seeded_ = false;
  reference_seeded_ = false;
  qp_fail_streak_ = 0;
  track_err_ = 0.0;
  traj_hint_ = 0;
  std::fill(arm_qd_cmd_.begin(), arm_qd_cmd_.end(), 0.0);
  // L7 §4.8's list, vision half: the consumed-sequence memory and the track
  // identity. Without the first, the snapshot in the box reads as "already
  // seen" after the reset and the trial starts by ignoring the only
  // prediction it has; without the second, the first snapshot of the new
  // trial looks like a track CHANGE and aborts it.
  consumed_ = rtc::catching::ConsumedToken{};
  last_track_generation_ = 0;
  track_seen_ = false;
  traj_new_track_ = false;
  traj_view_ = rtc::catching::TrajView{};
  // The S7 supervisor (header table). The hand goes back to the latch, which
  // the next readable tick re-seeds at the MEASURED pose (C-17): after a stop
  // the hand is not opened, and not squeezed either.
  hand_seq_.Deactivate();
  hand_out_ = rtc::catching::HandSequencerOutput{};
  homing_ = false;
  homing_done_ = false;
  retreat_stage_ = RetreatStage::kStop;
  trial_committed_ = false;
  committed_t_c_ns_ = 0;
  committed_t_cmd_ns_ = 0;
  committed_generation_ = 0;
  law_snapshot_ = rtc::catching::TrajectorySnapshot{};
  last_trial_generation_ = 0;
  last_trial_generation_valid_ = false;
  decel_entry_ = rtc::catching::DecelEntryState{};
  decel_t_s_ns_ = 0;
  decel_stopped_ = false;
  hold_entry_ns_ = 0;
  sat_streak_ = 0;
  law_horizon_extrap_ = false;
  // An E-STOP that lands on an attempt ends it (L7 §4.1): the verdict says so.
  // An activation starts with no attempt behind it.
  outcome_ = (!reset_mode && trial_active_) ? rtc::catching::Outcome::kAborted
                                            : rtc::catching::Outcome::kNone;
  trial_active_ = false;
  contact_.ResetForRearm();
  tip_baseline_n_.fill(0);
  tip_last_seq_.fill(0);
  window_confirmed_seen_ = false;
  window_stale_seen_ = false;
  // `tick_record_` is deliberately absent from this list: it is
  // default-constructed at the top of EVERY tick including this one (PROC-7),
  // which is a stronger guarantee than anything enumerated here could give.
}

// ── Contact lane (S7.3, L7 §4.4) ────────────────────────────────────────────

void DemoCatchingController::RunContactLane(const ControllerState& state) noexcept {
  using rtc::catching::Mode;
  tip_count_ = 0;
  tip_confirmed_now_ = 0;
  tip_stale_now_ = false;
  tip_force_now_.fill(0.0);
  tip_fresh_now_.fill(false);
  tip_contact_now_.fill(false);
  if (!contact_configured_ || state.num_devices <= kCatchingHandDeviceIdx) {
    return;
  }
  const auto& dev = state.devices[kCatchingHandDeviceIdx];
  tip_count_ = std::clamp(dev.num_inference_groups, 0, static_cast<int>(kTips));
  const bool contact_mode = mode_ == Mode::kCommitted || mode_ == Mode::kClosing ||
                            mode_ == Mode::kDecel || mode_ == Mode::kHold;
  // The baseline is learned from a hand that is AT q_pre and not moving
  // (§4.5-6, Q4): contact at rest on the preshape is the bias, anything on top
  // of it later is the ball.
  const bool learning = (mode_ == Mode::kArmed || mode_ == Mode::kTracking) && hand_out_.active &&
                        hand_out_.phase == rtc::catching::HandPhase::kPreshape &&
                        hand_out_.at_target;
  for (int g = 0; g < tip_count_; ++g) {
    const auto u = static_cast<std::size_t>(g);
    // BOTH the backend's flag and this controller's deadline (D-24):
    // IsSensorGroupFresh requires each.
    tip_fresh_now_[u] = rtc::IsSensorGroupFresh(dev, g, tick_now_.ns, contact_t_stale_ns_);
    const std::uint64_t seq = dev.inference_sequence[u];
    const bool is_new = seq != tip_last_seq_[u];
    tip_last_seq_[u] = seq;
    // Slots 1..3 are the force (hand_sensor_layout.hpp); only its MAGNITUDE
    // above the bias is judged, so the sensor's sign convention drops out.
    const auto base = u * static_cast<std::size_t>(tip_stride_);
    if (base + 3 >= dev.inference_data.size()) {
      continue;
    }
    const Eigen::Vector3d f(static_cast<double>(dev.inference_data[base + 1]),
                            static_cast<double>(dev.inference_data[base + 2]),
                            static_cast<double>(dev.inference_data[base + 3]));
    const auto& baseline = contact_.Baseline(u);
    tip_force_now_[u] = baseline.initialized ? (f - baseline.bias).norm() : f.norm();
    // Fed per SAMPLE: a 250 Hz lane read by a 500 Hz tick would otherwise
    // count every sample twice toward the debounce and the baseline.
    if (is_new && dev.inference_enable[u]) {
      if (learning) {
        if (contact_.UpdateBaseline(u, f)) {
          ++tip_baseline_n_[u];
        }
      } else if (contact_mode) {
        static_cast<void>(contact_.UpdateContact(u, f));
      }
    }
    tip_contact_now_[u] = contact_mode && contact_.IsConfirmed(u) && tip_fresh_now_[u];
    if (tip_contact_now_[u]) {
      ++tip_confirmed_now_;
    }
    if (contact_mode && !tip_fresh_now_[u]) {
      tip_stale_now_ = true;
    }
  }
  if (!contact_mode) {
    return;
  }
  if (trial_committed_ && rtc::catching::InContactWindow(
                              tick_now_, rtc::catching::BallTime{committed_t_cmd_ns_},
                              rtc::catching::BallTime{committed_t_c_ns_}, contact_t_confirm_ns_)) {
    window_confirmed_seen_ = window_confirmed_seen_ || tip_confirmed_now_ >= contact_m_min_;
    window_stale_seen_ = window_stale_seen_ || tip_stale_now_;
  }
}

rtc::catching::Outcome DemoCatchingController::JudgeOutcome() const noexcept {
  using rtc::catching::Outcome;
  // No verdict without the lane: no fingertips, fewer than m_min of them, or a
  // bias learned from too few samples (never Missed — that would release a
  // ball that may be in the hand, and count a success as a failure).
  if (!contact_configured_ || tip_count_ < contact_m_min_) {
    return Outcome::kUndetermined;
  }
  for (int g = 0; g < tip_count_; ++g) {
    if (tip_baseline_n_[static_cast<std::size_t>(g)] < contact_n_baseline_min_) {
      return Outcome::kUndetermined;
    }
  }
  if (window_stale_seen_) {
    return Outcome::kUndetermined;
  }
  // Caught: m_min fingertips agreed inside the window AND still agree now.
  // Agreeing in the window and not at the end is a ball that left the hand.
  if (window_confirmed_seen_ && tip_confirmed_now_ >= contact_m_min_) {
    return Outcome::kCaptured;
  }
  return Outcome::kMissed;
}

// ── Motion and hand stages (S7, after the decision) ─────────────────────────

void DemoCatchingController::RunArmMotion(const ControllerState& state) noexcept {
  using rtc::catching::Mode;
  if (estop_active_) {
    return;  // CM holds the robot; nothing here may move the command
  }
  switch (mode_) {
    case Mode::kAbortSafe:
    case Mode::kFault:
      // Only a stop caused by the joint command layer takes the QP-independent
      // route (L7 §4.1) — and, recorded as an exemption (C-35), every
      // ABORT_SAFE does.
      if (arm_cmd_seeded_) {
        RunJointSpaceAbort(state);
      }
      break;
    case Mode::kIdle:
      RunIdleMotion(state);
      break;
    case Mode::kRetreat:
      RunRetreatMotion(state);
      break;
    case Mode::kArmed:
    case Mode::kTracking:
    case Mode::kApproach:
    case Mode::kCommitted:
    case Mode::kClosing:
    case Mode::kDecel:
    case Mode::kHold:
      break;  // held by the carried command, or moved by the law already
  }
}

bool DemoCatchingController::StepTowardWaitPose(const ControllerState& state) noexcept {
  const auto n = static_cast<std::size_t>(std::min(arm_dof_, kDemoCatchingMaxArmDof));
  const auto step = rtc::catching::JointSpaceHomeStep(
      std::span<double>(arm_q_cmd_.data(), n), std::span<double>(arm_qd_cmd_.data(), n),
      std::span<const double>(wait_pose_.data(), n),
      std::span<const double>(arm_qdd_max_.data(), n), homing_eta_a_, homing_v_max_,
      std::span<const double>(arm_q_min_margined_.data(), n),
      std::span<const double>(arm_q_max_margined_.data(), n), n, state.dt);
  UpdateTrackError(state);
  return step.valid && step.arrived && ArmAtWaitPose(state);
}

void DemoCatchingController::RunIdleMotion(const ControllerState& state) noexcept {
  if (!trials_enabled_) {
    return;
  }
  const bool armed = armable_ && arm_requested_.load(std::memory_order_relaxed);
  if (!armed) {
    // R-IDLE: a command that was still moving when IDLE took over (a RETREAT
    // disarmed mid-return, a homing disarmed mid-way) is ramped to rest here,
    // never frozen in one tick.
    homing_ = false;
    homing_done_ = false;
    if (!ArmCommandStopped()) {
      static_cast<void>(RampArmToStop(state));
    }
    return;
  }
  // Armed. First bring any carried motion to rest; homing starts from rest.
  if (!homing_ && !ArmCommandStopped()) {
    static_cast<void>(RampArmToStop(state));
    return;
  }
  if (!homing_ && !homing_done_) {
    if (!arm_readable_) {
      return;
    }
    // Not at the wait pose (EvaluateIdle would have taken the Q13 skip): home.
    SeedArmCommand(state);
    homing_ = true;
    if (hand_seq_enabled_) {
      hand_seq_.Home();  // Q4: q_open while the arm moves
    }
  }
  if (!homing_) {
    return;
  }
  if (StepTowardWaitPose(state)) {
    homing_ = false;
    homing_done_ = true;
    if (hand_seq_enabled_) {
      hand_seq_.Ready();
    }
    return;
  }
  // R-WATCHDOG. IDLE has no TRACK_ERR row: the answer is to stop and disarm —
  // the operator re-arms deliberately (P-1 (c)) — and to say why.
  if (track_err_abort_rad_ > 0.0 && track_err_ > track_err_abort_rad_) {
    arm_requested_.store(false, std::memory_order_relaxed);
    homing_ = false;
    last_reason_ = rtc::catching::Reason::kTrackErr;
  }
}

void DemoCatchingController::RunRetreatMotion(const ControllerState& state) noexcept {
  switch (retreat_stage_) {
    case RetreatStage::kStop:
      // The stop first, at the full acceleration box (a no-op after
      // ABORT_SAFE, which already stopped the arm). The return starts from rest.
      if (RampArmToStop(state)) {
        retreat_stage_ = RetreatStage::kReturn;
      }
      // Measured on EVERY tick of the stop, not just the return's: the error
      // EvaluateRetreat judges must be this motion's. A TRACK_ERR abort leaves
      // its own (large) value behind, and judging the return on it sent the
      // controller back to ABORT_SAFE and round again, forever, with the arm
      // standing still (found by the S7 scenario suite).
      if (arm_cmd_seeded_ && arm_readable_) {
        UpdateTrackError(state);
      }
      break;
    case RetreatStage::kReturn:
      if (!arm_cmd_seeded_) {
        SeedArmCommand(state);
      }
      if (StepTowardWaitPose(state)) {
        retreat_stage_ = RetreatStage::kRelease;
        // The ball comes back with the arm and is let go here, whatever the
        // verdict (OnModeEntered, RETREAT). A hand that never closed is already
        // at q_pre, and its Release ends in Preshape on the next hand stage.
        if (hand_seq_enabled_) {
          hand_seq_.Release();
        }
      }
      break;
    case RetreatStage::kRelease:
      break;  // waiting for the hand (EvaluateRetreat)
  }
}

void DemoCatchingController::LatchHandFromSequencer() noexcept {
  if (!hand_out_.active || !hand_hold_.IsLatched()) {
    return;
  }
  const int n = std::min(hand_out_.dof, hand_hold_.width);
  for (int i = 0; i < n; ++i) {
    hand_hold_.commands[static_cast<std::size_t>(i)] =
        hand_out_.target[static_cast<std::size_t>(i)];
  }
  rtc::utils::ClampRange(hand_hold_.commands, n,
                         std::span<const double>(device_position_lower_[kCatchingHandDeviceIdx]),
                         std::span<const double>(device_position_upper_[kCatchingHandDeviceIdx]),
                         kFallbackPositionLower, kFallbackPositionUpper);
}

void DemoCatchingController::RunHandStage(const ControllerState& state) noexcept {
  using rtc::catching::Mode;
  if (!hand_seq_enabled_ || !trials_enabled_ || estop_active_) {
    hand_out_ = rtc::catching::HandSequencerOutput{};
    return;
  }
  const bool armed = armable_ && arm_requested_.load(std::memory_order_relaxed);
  if (mode_ == Mode::kIdle && !armed) {
    // A disarmed IDLE holds the hand where the sequencer last put it: the
    // latch takes over from the sequencer's last target, not from a pose the
    // hand left long ago (and a hand holding a ball keeps holding it).
    if (hand_seq_.Active()) {
      LatchHandFromSequencer();
      hand_seq_.Deactivate();
    }
    hand_out_ = rtc::catching::HandSequencerOutput{};
    return;
  }
  if (mode_ == Mode::kFault) {
    return;  // FAULT holds the last output (L7 hand policy)
  }
  std::span<const double> q{};
  std::span<const double> qd{};
  if (hand_readable_ && state.num_devices > kCatchingHandDeviceIdx) {
    const auto& dev = state.devices[kCatchingHandDeviceIdx];
    const auto n = static_cast<std::size_t>(hand_dof_);
    q = std::span<const double>(dev.positions.data(), n);
    qd = std::span<const double>(dev.velocities.data(), n);
  }
  hand_out_ = hand_seq_.Update(tick_now_, static_cast<std::int64_t>(state.dt * 1e9), q, qd);
}

bool DemoCatchingController::ServiceResetRequests(const ControllerState& state) noexcept {
  static_cast<void>(state);
  bool reset = false;

  // D-23: the activation boundary is decided from the generation, not from a
  // quiescence wait. PeriodicRtThread::Pause does not stop an iteration
  // already in flight, so "no tick is running" is not something on_activate
  // can establish — but "the generation changed" is something this tick can
  // observe, and it is true exactly once per activation.
  bool fresh_start = false;

  const std::uint32_t generation = ActivationGeneration();
  if (!activation_seen_ || generation != serviced_activation_generation_) {
    serviced_activation_generation_ = generation;
    activation_seen_ = true;
    reset = true;
    fresh_start = true;
  }

  const std::uint32_t reset_epoch = reset_requested_.load(std::memory_order_acquire);
  if (reset_epoch != serviced_reset_epoch_) {
    serviced_reset_epoch_ = reset_epoch;
    reset = true;
    fresh_start = true;
  }

  const std::uint32_t estop_epoch = estop_epoch_.load(std::memory_order_acquire);
  if (estop_epoch != serviced_estop_epoch_) {
    serviced_estop_epoch_ = estop_epoch;
    reset = true;
    // Both edges disarm. On the trigger it is the stop itself; on the clear it
    // is P-1 (c), and doing it on BOTH is what covers the trigger→clear pair
    // that lands between two ticks — the tick then sees one epoch move, and
    // that single reset must still leave the controller disarmed.
    arm_requested_.store(false, std::memory_order_relaxed);
  }

  const std::uint32_t fault_epoch = fault_reset_epoch_.load(std::memory_order_acquire);
  if (fault_epoch != serviced_fault_reset_epoch_) {
    serviced_fault_reset_epoch_ = fault_epoch;
    // The cause is gone as soon as it is asked about, at S5.1: the only thing
    // that can raise this latch is the supervisor reaching Mode::kFault, and
    // no S5.1 path reaches it (the QP failure streak that does is S5.3). When
    // that path lands, this is where "refuse the clear while the cause is
    // still present" belongs.
    if (fault_latched_.load(std::memory_order_relaxed)) {
      fault_latched_.store(false, std::memory_order_release);
      fault_reset_serviced_ = true;
      reset = true;
    }
  }

  // A latched fault disarms too, and it does so on EVERY tick the latch is up
  // rather than on the edge that raised it: the latch outlives the reset that
  // noticed it, so an operator setting `catching.enable` while faulted must not
  // find the controller armed the moment the fault is cleared. (Nothing raises
  // this latch before S5.3's QP-failure streak — this is the rule being in
  // place before the cause exists, not dead code for its own sake.)
  if (fault_latched_.load(std::memory_order_relaxed)) {
    arm_requested_.store(false, std::memory_order_relaxed);
  }

  if (reset) {
    ResetTrialState(fresh_start);
    rt_reset_count_.fetch_add(1, std::memory_order_relaxed);
  }
  return reset;
}

// ── RT tick ─────────────────────────────────────────────────────────────────

void DemoCatchingController::ReportGateClosure(const ControllerState& state, int device_idx,
                                               int dof, const char* axis) noexcept {
  if (device_idx >= state.num_devices) {
    return;
  }
  const auto& dev = state.devices[static_cast<std::size_t>(device_idx)];
  // RT-3's throttle exception: primitive-only format, no assembly. `axis` is a
  // string literal selected by the caller, which is on the allowed list.
  //
  // Not an else-if pair: IsGateClosedByHoles requires the width test to have
  // PASSED, so the two are mutually exclusive by construction and chaining them
  // would only hide that.
  if (rtc::IsGateClosedByWidth(dev, dof)) {
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "F5 gate closed: %s device reports num_channels=%d < dof=%d — that axis "
                         "is silenced every tick; fix the group's 'joint_state_names' or the "
                         "backend's reported channel count",
                         axis, dev.num_channels, dof);
  }
  if (rtc::IsGateClosedByHoles(dev, dof)) {
    RCLCPP_WARN_THROTTLE(logger_, log_clock_, ::integrated_bringup::logging::kThrottleSlowMs,
                         "F5 gate closed: %s device slot %d was never written although "
                         "num_channels=%d >= dof=%d — that axis is silenced every tick; the state "
                         "message carries no joint named at that position of the group's "
                         "'joint_command_names' (defaults to 'joint_state_names')",
                         axis, rtc::FirstHoleSlot(dev, dof), dev.num_channels, dof);
  }
}

void DemoCatchingController::WriteDeviceCommand(const ControllerState& state,
                                                ControllerOutput& output, int device_idx,
                                                bool readable) noexcept {
  const auto idx = static_cast<std::size_t>(device_idx);
  const auto& dev = state.devices[idx];
  auto& out = output.devices[idx];
  out.goal_type = rtc::GoalType::kJoint;

  const bool is_hand = (device_idx == kCatchingHandDeviceIdx);
  HoldLatch& latch = is_hand ? hand_hold_ : arm_hold_;

  // Latch on the first READABLE tick, and never again until the next
  // activation. Seeding from an unreadable device would freeze a hold target
  // whose unreported joints are 0 — i.e. "go to the origin" — and the latch
  // would make that permanent.
  if (!latch.IsLatched() && readable) {
    // Trimmed at the first HOLE, not at num_channels. `readable` only vouches
    // for [0, dof) — slots past it can be wire channels no state message ever
    // wrote, which sit at 0.0 in the persistent cache. Latching those would
    // freeze "go to the origin" on them for the whole activation, and unlike
    // the joint controller's per-tick tail this latch never re-seeds. This is
    // what SelfReportedChannelBound exists for, and it is a no-op on a
    // hole-free device, so a legitimately wide device still gets full width.
    const int width = rtc::SelfReportedChannelBound(dev, static_cast<int>(kMaxDeviceChannels));
    for (std::size_t i = 0; i < static_cast<std::size_t>(width); ++i) {
      latch.commands[i] = dev.positions[i];
    }
    rtc::utils::ClampRange(latch.commands, width,
                           std::span<const double>(device_position_lower_[idx]),
                           std::span<const double>(device_position_upper_[idx]),
                           kFallbackPositionLower, kFallbackPositionUpper);
    latch.width = width;
  }

  if (!latch.IsLatched()) {
    // Nothing honest to command yet: zero-length is "no update" and the drive
    // holds its own setpoint. nc zeros would be a real command to the origin.
    rtc::SilenceDeviceOutput(out);
    rtc::HoldTelemetryAtMeasured(out, dev.num_channels, std::span<const double>(dev.positions));
    return;
  }

  // Once the law owns the arm, the latch is no longer what is commanded: it
  // stays as the value the controller falls back to, and the law's carried
  // command takes over. The hand is the same shape from S7.1: the sequencer's
  // target while it is active, the latch otherwise — and on an E-STOP tick
  // the latch in both cases, like the arm.
  const bool arm_driven = !is_hand && arm_cmd_seeded_ && !estop_active_;
  const bool hand_driven = is_hand && hand_out_.active && !estop_active_;

  out.num_channels = latch.width;
  for (std::size_t i = 0; i < static_cast<std::size_t>(latch.width); ++i) {
    const auto ii = static_cast<int>(i);
    if (arm_driven && ii < arm_dof_) {
      out.commands[i] = arm_q_cmd_[i];
    } else if (hand_driven && ii < hand_out_.dof) {
      out.commands[i] = hand_out_.target[i];
    } else {
      out.commands[i] = latch.commands[i];
    }
    // The reference lanes carry the same value the command does: this
    // controller integrates one command state, so there is no second
    // trajectory to report. The hand's goal lane is the UNCLAMPED target
    // (below), so a clamp shows in the CSV instead of rewriting the target.
    out.target_positions[i] = out.commands[i];
    out.trajectory_positions[i] = out.commands[i];
    out.goal_positions[i] = out.commands[i];
  }
  if (hand_driven) {
    // Clamped against YAML ∩ URDF here, for the same reason as the step
    // below: the CM copies `commands` to the backend verbatim.
    const int width = std::min(hand_out_.dof, latch.width);
    rtc::utils::ClampRange(out.commands, width,
                           std::span<const double>(device_position_lower_[idx]),
                           std::span<const double>(device_position_upper_[idx]),
                           kFallbackPositionLower, kFallbackPositionUpper);
    for (std::size_t i = 0; i < static_cast<std::size_t>(width); ++i) {
      out.target_positions[i] = out.commands[i];
      out.trajectory_positions[i] = out.commands[i];
    }
    return;
  }

  if (!is_hand || hand_target_width_ == 0 || estop_active_) {
    // `estop_active_`: a stop holds the pose it was stopped in. Without this
    // the overlay below would keep re-applying the last accepted step on every
    // tick of the stop — the value is latched in `hand_target_raw_`, so it
    // does not need a NEW target to reach the output.
    return;
  }

  // ── The step ────────────────────────────────────────────────────────────
  // G6-B's controller half: what this writes into `commands` is what the CM
  // copies to the backend verbatim (it neither clamps nor filters), so the
  // clamp against YAML ∩ URDF limits has to happen HERE. `goal_positions`
  // keeps the raw target so a clamp shows up in the CSV as a divergence
  // instead of silently rewriting what was asked for.
  const int width = std::min(hand_target_width_, latch.width);
  for (std::size_t i = 0; i < static_cast<std::size_t>(width); ++i) {
    out.commands[i] = hand_target_raw_[i];
    out.goal_positions[i] = hand_target_raw_[i];
    out.target_positions[i] = hand_target_raw_[i];
    out.trajectory_positions[i] = hand_target_raw_[i];
  }
  rtc::utils::ClampRange(out.commands, width, std::span<const double>(device_position_lower_[idx]),
                         std::span<const double>(device_position_upper_[idx]),
                         kFallbackPositionLower, kFallbackPositionUpper);
}

// ── The tick record (S5.4, D-20 + L8 §5.2) ─────────────────────────────────
//
// Three fillers, each called at the point its values EXIST. That placement is
// the whole PROC-7 mechanism: a tick that never reached the reference
// generator never calls RecordReference, and the block stays zero because the
// record was default-constructed at the top of the tick.

void DemoCatchingController::RecordInputLane(
    const rtc::catching::TrajectorySnapshot& snapshot) noexcept {
  tick_record_.input_valid = snapshot.valid;
  tick_record_.input_stale = traj_view_.stale;
  tick_record_.input_expired = traj_view_.expired;
  tick_record_.input_new = traj_view_.is_new;
  tick_record_.input_n = snapshot.n;
  tick_record_.input_generation = snapshot.token.generation;
  tick_record_.input_snapshot_sequence = snapshot.token.snapshot_sequence;
  tick_record_.input_activation_generation = snapshot.token.activation_generation;
  tick_record_.input_age_s = static_cast<double>(traj_view_.age_ns) * 1e-9;
  // The horizon the snapshot actually carries — last sample minus first, not
  // the configured minimum. Reporting the requirement instead would make a
  // publisher that is one sample short look exactly like one that is not.
  if (snapshot.valid && snapshot.n > 1) {
    const auto last = static_cast<std::size_t>(snapshot.n - 1);
    tick_record_.input_horizon_s =
        static_cast<double>(snapshot.s[last].t_ns - snapshot.s[0].t_ns) * 1e-9;
  }
}

void DemoCatchingController::RecordReference(const rtc::catching::TranslationOutput& ref) noexcept {
  tick_record_.ref_valid = ref.valid;
  tick_record_.ref_saturated = ref.saturated;
  for (int i = 0; i < 3; ++i) {
    const auto u = static_cast<std::size_t>(i);
    tick_record_.ref_x[u] = ref.x[i];
    tick_record_.ref_xd[u] = ref.xd[i];
    tick_record_.ref_xdd[u] = ref.xdd[i];
    tick_record_.ref_u_des[u] = ref.u_des[i];
    tick_record_.ref_e[u] = ref.e[i];
    tick_record_.ref_ed[u] = ref.ed[i];
  }
  tick_record_.ref_gamma = ref.gamma;
  tick_record_.ref_gamma_d = ref.gamma_d;
  tick_record_.ref_gamma_dd = ref.gamma_dd;
}

void DemoCatchingController::RecordClikSolve(
    const rtc::tsid::ClikReferenceGenerator::SolveDiagnostics& solve) noexcept {
  tick_record_.clik_ran = solve.reached_solve;
  tick_record_.clik_converged = solve.converged;
  tick_record_.clik_bound_conflict = solve.bound_conflict;
  tick_record_.clik_command_mismatch = solve.command_mismatch;
  tick_record_.clik_status = solve.status;
  tick_record_.clik_iterations = solve.iterations;
  tick_record_.clik_solve_us = solve.solve_time_us;
  tick_record_.clik_conflict_mask = solve.conflict_mask;
}

void DemoCatchingController::PublishTickRecord(const ControllerState& state) noexcept {
  // The tick's own instant (R-DECEL-ENTRY): the record describes the tick the
  // decisions were taken on, not the moment it was written.
  const std::int64_t now_ns = tick_now_.ns;

  // ── Supervisor and controller-level state ───────────────────────────────
  // All of these are CURRENT on every tick — they are states, not per-tick
  // measurements, so filling them here rather than where they changed is what
  // the field means.
  tick_record_.mode = static_cast<std::uint8_t>(mode_);
  tick_record_.reason = static_cast<std::uint8_t>(last_reason_);
  tick_record_.armed = arm_requested_.load(std::memory_order_relaxed);
  tick_record_.estop_active = estop_active_;
  tick_record_.fault_latched = fault_latched_.load(std::memory_order_relaxed);
  tick_record_.armable = armable_;
  tick_record_.law_enabled = clik_enabled_;
  tick_record_.real_arm_config = real_arm_config_;
  tick_record_.qp_fail_streak = qp_fail_streak_;
  tick_record_.abort_stopped = abort_stopped_;
  tick_record_.outcome = static_cast<std::uint8_t>(outcome_);

  // ── The hand (L6, S7.1) ─────────────────────────────────────────────────
  // `hand_phase_valid` false = the latch commands the hand this tick (not
  // armed, E-STOP, a hand-step profile, or a configuration with no trials).
  if (hand_out_.active) {
    tick_record_.hand_phase_valid = true;
    tick_record_.hand_phase = static_cast<std::uint8_t>(hand_out_.phase);
    tick_record_.hand_rho = hand_out_.rho;
    tick_record_.hand_timeout = hand_out_.timeout;
  }

  // ── The plan ────────────────────────────────────────────────────────────
  if (plan_active_ && plan_.valid) {
    tick_record_.plan_valid = true;
    tick_record_.plan_id = plan_.plan_id;
    tick_record_.plan_t_c_s = static_cast<double>(plan_.t_c_ns - now_ns) * 1e-9;
    tick_record_.plan_age_s = static_cast<double>(now_ns - plan_.publish_ns) * 1e-9;
    tick_record_.plan_p_c = plan_.p_c;
    tick_record_.plan_a_d = plan_.a_d;
    tick_record_.plan_v_c = plan_.v_c;
    tick_record_.plan_gamma_f = plan_.gamma_gf;
    tick_record_.plan_w5 = plan_.w5;
    tick_record_.plan_w6 = plan_.w6;
    tick_record_.plan_sigma_c = plan_.sigma_c;
    tick_record_.plan_score = plan_.score;
    tick_record_.plan_reason = static_cast<std::uint8_t>(plan_.reason);
  } else if (plan_refusal_ == rtc::catching::PlanRefusal::kInvalid &&
             plan_in_.token.activation_generation == ActivationGeneration() &&
             plan_in_.publish_ns > 0) {
    // Following nothing, and the planner's latest word for THIS activation is
    // "no plan": carry its reason (the first bottleneck, decision E) and its
    // id and age, so the operator sees WHY there is no plan rather than just
    // that there is none (§13 S6). `plan_valid` stays false — there is no plan
    // to describe, and the other plan fields stay zero (PROC-7).
    tick_record_.plan_id = plan_in_.plan_id;
    tick_record_.plan_age_s = static_cast<double>(now_ns - plan_in_.publish_ns) * 1e-9;
    tick_record_.plan_reason = static_cast<std::uint8_t>(plan_in_.reason);
  }

  // ── The arm, commanded against measured ─────────────────────────────────
  // Filled on every tick including the holding ones: a hold is a command, and
  // a reader that saw nothing could not tell it from a tick that produced no
  // command at all.
  //
  // The selection MIRRORS WriteDeviceCommand above, because the only useful
  // reading of this column is "what went out on the wire this tick". Writing
  // 0.0 whenever the law has not taken over reported "commanded to the
  // origin" for every holding tick while the latch was really commanding the
  // activation pose, and ‖q_meas − q_cmd‖ computed offline from these columns
  // then showed a multi-radian error that does not exist (2026-09-23 review).
  // Before the latch is set the output is SILENCED — no command exists — and
  // that case is NaN rather than a number, so a reader cannot average it away.
  const auto n_arm = static_cast<std::size_t>(
      std::min<int>(arm_dof_, static_cast<int>(CatchingDiagLogPod::kMaxArmJoints)));
  tick_record_.num_arm_joints = static_cast<std::uint8_t>(n_arm);
  const bool arm_driven = arm_cmd_seeded_ && !estop_active_;
  const auto latched_width = static_cast<std::size_t>(std::max(arm_hold_.width, 0));
  for (std::size_t i = 0; i < n_arm; ++i) {
    if (!arm_hold_.IsLatched()) {
      tick_record_.q_cmd[i] = std::numeric_limits<double>::quiet_NaN();
    } else if (arm_driven) {
      tick_record_.q_cmd[i] = arm_q_cmd_[i];
    } else {
      tick_record_.q_cmd[i] =
          i < latched_width ? arm_hold_.commands[i] : std::numeric_limits<double>::quiet_NaN();
    }
  }
  if (arm_readable_ && state.num_devices > kCatchingArmDeviceIdx) {
    const auto& dev = state.devices[kCatchingArmDeviceIdx];
    for (std::size_t i = 0; i < n_arm; ++i) {
      tick_record_.q_meas[i] = dev.positions[i];
    }
  }

  // ── Fingertip ages (D-24) ───────────────────────────────────────────────
  // The age is a measurement; `tip_fresh` / `tip_contact` / `tip_force` wait
  // for S7's threshold and its force layout — see the POD header.
  // EVERY slot starts at "never received", including the ones past the
  // runtime count. The POD's zero-init would leave them at 0.0, which on this
  // wire means "arrived this instant" — and the message's arrays are sized
  // from the device's SENSOR NAMES while this loop is bounded by
  // `num_inference_groups`, so a backend that reports no groups (mujoco with
  // no `fingertip_wrench_topics`) would publish four fingertips as fresh.
  tick_record_.tip_age_s.fill(-1.0);
  if (state.num_devices > kCatchingHandDeviceIdx) {
    const auto& hand = state.devices[kCatchingHandDeviceIdx];
    const auto n_tip = static_cast<std::size_t>(
        std::min<int>(hand.num_inference_groups, static_cast<int>(CatchingDiagLogPod::kMaxTips)));
    tick_record_.num_tips = static_cast<std::uint8_t>(n_tip);
    for (std::size_t i = 0; i < n_tip; ++i) {
      const std::int64_t age = rtc::SensorGroupAgeNs(hand, static_cast<int>(i), now_ns);
      // Negative means never received, and that is what the wire says too —
      // a zero would read as "arrived this instant".
      tick_record_.tip_age_s[i] = age < 0 ? -1.0 : static_cast<double>(age) * 1e-9;
      // S7.3's verdicts, as the contact lane computed them this tick. `tip_force`
      // is |F − bias| once a bias exists (|F| before); `tip_contact` is the
      // debounced verdict, true only while the lane is judging contact.
      if (i < kTips && static_cast<int>(i) < tip_count_) {
        tick_record_.tip_force[i] = tip_force_now_[i];
        tick_record_.tip_fresh[i] = tip_fresh_now_[i];
        tick_record_.tip_contact[i] = tip_contact_now_[i];
      }
    }
  }

  catching_state_lock_.Store(tick_record_);
  if (catching_diag_log_handle_) {
    catching_diag_log_handle_.Push(tick_record_);
  }
}

void DemoCatchingController::PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept {
  // CM's publish jthread (SCHED_OTHER), not the RT tick. Both loads are
  // SeqLock reads of single-writer payloads.
  const auto tick = catching_state_lock_.Load();
  const auto ingress = ingress_diag_box_.Load();
  PublishCatchingStateFromSnapshot(snap, owned_topics_, tick, &ingress);
}

ControllerOutput DemoCatchingController::Compute(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoCatchingController::Compute");
  ControllerOutput output;
  output.num_devices = state.num_devices;
  output.command_type = CommandType::kPosition;
  output.valid = true;

  // PROC-7, by construction. A FRESH record every tick, so a block this tick
  // does not compute is zero rather than the previous tick's value — and no
  // helper has to remember to clear its own fields. The three stamps below are
  // the only things true of every tick regardless of what it decides.
  tick_record_ = CatchingDiagLogPod{};
  tick_record_.t_relative_s = state.t_relative_s;
  tick_record_.tick = state.iteration;
  tick_record_.t_arm_s = static_cast<double>(t_arm_ns_) * 1e-9;

  // Read ONCE per tick, into a tick-local bool. Re-reading the atomic further
  // down would let one tick act on two different answers — the command written
  // for a stop the supervisor did not see, or the reverse.
  estop_active_ = estop_requested_.load(std::memory_order_acquire);
  if (estop_active_) {
    estop_tick_count_.fetch_add(1, std::memory_order_relaxed);
  }

  arm_readable_ = state.num_devices > kCatchingArmDeviceIdx &&
                  rtc::IsDeviceReadable(state.devices[kCatchingArmDeviceIdx], arm_dof_);
  hand_readable_ = state.num_devices > kCatchingHandDeviceIdx &&
                   rtc::IsDeviceReadable(state.devices[kCatchingHandDeviceIdx], hand_dof_);
  ReportGateClosure(state, kCatchingArmDeviceIdx, arm_dof_, "arm");
  ReportGateClosure(state, kCatchingHandDeviceIdx, hand_dof_, "hand");

  // BEFORE the target drain and before the command write (P-1 (b)): a reset
  // discards queued goals and drops the hold latches, so anything that
  // consumed a goal first would apply state the reset is about to throw away,
  // and anything that wrote a command first would command the pose the reset
  // has just decided not to trust.
  (void)ServiceResetRequests(state);

  if (estop_active_) {
    // Drained AND DISCARDED while stopped. Not skipped: the mailbox is a
    // fixed-capacity SPSC, so leaving entries in it would hand the hand a step
    // issued mid-stop as soon as the stop cleared — later, and with no
    // operator expecting it. Discarding says what the stop means.
    //
    // The CM substitutes its own hold for this controller's entire output
    // while the global latch is up, so nothing here reaches an actuator either
    // way. That is exactly why this is worth doing rather than relying on it:
    // a second layer costs one branch, and the layer that is doing the work
    // today belongs to a different component.
    DiscardPendingTargets();
  } else {
    // Drained before the write, not after: WriteDeviceCommand seeds the hold
    // latch and then overlays the step, so a step arriving on the same tick as
    // the first readable state is honoured on that tick instead of a tick later.
    (void)DrainPendingTargets();
  }

  // D-21: Load() unconditionally, once per tick, BEFORE anything decides
  // anything. Not gated on SeqLock::sequence() — reading the counter and the
  // payload as two steps lets a writer land between them and pair a new
  // counter with an old payload, which hides the newest snapshot for as long
  // as the pattern repeats.
  const rtc::catching::TrajectorySnapshot snapshot = traj_box_.Load();
  // The tick's ONE clock read (R-DECEL-ENTRY), after the load so the
  // snapshot's receive instant cannot be later than `now`. Everything below —
  // the vision verdict, the law, the sequencer, every edge — judges this
  // instant; the plan admission alone takes its own (see there).
  tick_now_ = rtc::catching::NowReal{SteadyNowNs()};
  tick_now_lead_ = rtc::catching::MakeNowLead(tick_now_, t_arm_ns_);
  law_horizon_extrap_ = false;
  const rtc::catching::NowReal now = tick_now_;
  traj_view_ = rtc::catching::ReadTraj(snapshot, now, tick_now_lead_, t_stale_ns_,
                                       ActivationGeneration(), consumed_);
  RecordInputLane(snapshot);
  if (traj_view_.is_new) {
    // A track change is latched here rather than compared in EvaluateReason:
    // the comparison is only meaningful on the tick the snapshot arrives, and
    // the supervisor may need a tick or two to act on it.
    traj_new_track_ = track_seen_ && snapshot.token.generation != last_track_generation_;
    last_track_generation_ = snapshot.token.generation;
    track_seen_ = true;
    // The followed track's latest snapshot, which the law samples once the
    // plan is frozen (R-TRACK): the plan's track before the freeze, the
    // committed one after — never a different ball's.
    const bool frozen =
        mode_ == rtc::catching::Mode::kCommitted || mode_ == rtc::catching::Mode::kClosing;
    const std::uint64_t followed = frozen ? committed_generation_ : plan_.token.generation;
    if (plan_active_ && snapshot.valid && snapshot.token.generation == followed) {
      law_snapshot_ = snapshot;
    }
  }

  // The fingertips, before anything decides: the decision reads this tick's
  // contact verdict and freshness (S7.3).
  RunContactLane(state);

  // ── The plan lane (S6-A, L3 §5.2) ───────────────────────────────────────
  // The oracle stand-in writes first (it is this box's writer only when the
  // profile has no planner), then the box is loaded ONCE, unconditionally
  // (D-21), and judged against what this tick knows. The verdict is what
  // EvaluateReason's TRACKING branch consults; nothing else reads plan_in_.
  if (oracle_enabled_ && mode_ == rtc::catching::Mode::kTracking && !plan_active_ &&
      clik_enabled_ && arm_readable_ && track_seen_) {
    StoreOraclePlan(now);
  }
  plan_in_ = plan_box_.Load();
  {
    rtc::catching::PlanAdmissionContext ctx{};
    ctx.activation_generation = ActivationGeneration();
    ctx.track_seen = track_seen_;
    ctx.track_generation = last_track_generation_;
    // Sampled AFTER the load, not the tick's `now` (taken before the vision
    // read): a planner on the same steady clock may publish between the two,
    // and judging its plan against the earlier instant would call a brand-new
    // plan "from the future" (JudgePlan (e)) and slip its adoption a tick.
    ctx.now = rtc::catching::NowReal{SteadyNowNs()};
    // A plan older than the ingress staleness bound was computed against a
    // prediction this tick would itself refuse as stale.
    ctx.max_age_ns = t_stale_ns_;
    ctx.reset_floor_ns = reset_floor_ns_;
    // (g) R-ADMIT: a plan already inside the freeze window is too late.
    ctx.t_freeze_ns = plan_freeze_ns_;
    plan_refusal_ = rtc::catching::JudgePlan(plan_in_, ctx, admitted_plan_);
    plan_refusal_observed_.store(static_cast<std::uint8_t>(plan_refusal_),
                                 std::memory_order_relaxed);
  }

  const ReasonDecision decision = EvaluateReason(state, snapshot);
  if (decision.advance) {
    AdvanceMode(decision.reason);
  } else {
    last_reason_ = decision.reason;
  }
  // Published every tick, including the early ones and the estopped ones —
  // the discipline the state message inherits as PROC-7. These two atomics
  // are the in-process read path (tests, and anything holding the controller
  // directly); the record below is the one that leaves the process.
  mode_observed_.store(static_cast<std::uint8_t>(mode_), std::memory_order_relaxed);
  reason_observed_.store(static_cast<std::uint8_t>(last_reason_), std::memory_order_relaxed);

  // The joint-space motions (stop ramp, homing, return) run AFTER the
  // supervisor has decided, because it is the decision that selects them. The
  // hand follows, so an edge taken this tick (a Release on RETREAT, a Commit)
  // is on the wire this tick.
  RunArmMotion(state);
  RunHandStage(state);
  // A motion-stage verdict (IDLE's homing watchdog) is recorded too.
  reason_observed_.store(static_cast<std::uint8_t>(last_reason_), std::memory_order_relaxed);

  if (state.num_devices > kCatchingArmDeviceIdx) {
    WriteDeviceCommand(state, output, kCatchingArmDeviceIdx, arm_readable_);
  }
  if (state.num_devices > kCatchingHandDeviceIdx) {
    WriteDeviceCommand(state, output, kCatchingHandDeviceIdx, hand_readable_);
  }

  if (arm_state_log_handle_) {
    DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, output, kCatchingArmDeviceIdx, pod);
    arm_state_log_handle_.Push(pod);
  }
  if (hand_state_log_handle_) {
    DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, output, kCatchingHandDeviceIdx, pod);
    hand_state_log_handle_.Push(pod);
  }

  // Every tick, after the decision and the law, so the planner sees the mode
  // and the reference state this tick left behind.
  StorePlannerRtState(state, now);

  // LAST, and unconditional. Every branch above reaches here — Compute() has a
  // single exit on purpose, so "Store on every early-return branch" (PROC-7)
  // is a structural property rather than a list of places to remember.
  PublishTickRecord(state);

  return output;
}

}  // namespace integrated_bringup
