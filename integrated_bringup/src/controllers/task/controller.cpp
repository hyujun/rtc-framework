// ── Includes: project header first, then C++ stdlib
// ────────────────────────────
#include "integrated_bringup/controllers/demo_task_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "integrated_bringup/support/demo_shared_config.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_base/utils/clamp_commands.hpp"
#include "rtc_controller_interface/device_readability.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"  // FloorMaxDamping, kMinSigma0
#include "rtc_controllers/gain_floor.hpp"
#include "rtc_math/se3/so3.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <array>
#include <cmath>  // std::sqrt
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math.hpp>
#include <pinocchio/spatial.hpp>
#pragma GCC diagnostic pop

namespace integrated_bringup {

// ── Constructor ─────────────────────────────────────────────────────────────

DemoTaskController::DemoTaskController(std::string_view urdf_path, Gains gains)
    : gains_lock_(gains), urdf_path_(urdf_path) {
  // Model is built in LoadConfig() (bridge YAML driven) or InitArmModel().
  // Constructor only stores urdf_path for later use.
  //
  // Init the contact_stop hold LPF now so the RT path always has valid
  // coefficients even when the controller is exercised without LoadConfig
  // (unit tests). LoadConfig re-Inits at the runtime-configured rate/cutoff.
  hand_pos_filter_.Init(gains.contact_stop_lpf_cutoff_hz, 1.0 / GetDefaultDt());
  hand_pos_filter_.Reset();
}

void DemoTaskController::InitArmModel(const rtc_urdf_bridge::ModelConfig& config) {
  namespace rub = rtc_urdf_bridge;
  // Prefer the shared builder injected by RtControllerNode so the URDF is
  // parsed only once across every controller. Fall back to building our own
  // (e.g. for unit tests that bypass CM, or when the shared build failed).
  if (auto shared = GetSharedModelBuilder()) {
    builder_ = std::move(shared);
  } else {
    builder_ = std::make_shared<rub::PinocchioModelBuilder>(config);
  }

  // Resolve sub-model name: match primary device name, fallback to "arm"
  const auto primary = GetPrimaryDeviceName();
  std::string model_name = "arm";
  for (const auto& sm : config.sub_models) {
    if (sm.name == primary) {
      model_name = primary;
      break;
    }
  }
  arm_handle_ = std::make_unique<rub::RtModelHandle>(builder_->GetReducedModel(model_name));

  const int nv = arm_handle_->nv();

  // Pre-allocate all Eigen buffers to their final sizes.
  q_ = Eigen::VectorXd::Zero(nv);
  J_full_ = Eigen::MatrixXd::Zero(6, nv);
  J_pos_ = Eigen::MatrixXd::Zero(3, nv);
  // §6.5 DLS working buffers (#282). Resize() is the ONLY allocating entry
  // point on either object; ComputeControl may not call it (RT-1).
  ik_6d_.Resize(nv, 6);
  ik_3d_.Resize(nv, 3);
  dq_ = Eigen::VectorXd::Zero(nv);
  desired_q_ = Eigen::VectorXd::Zero(nv);
  traj_dq_ = Eigen::VectorXd::Zero(nv);
  qdot_null_ = Eigen::VectorXd::Zero(nv);
  pos_error_ = Eigen::Vector3d::Zero();

  pos_error_6d_ = Eigen::Matrix<double, 6, 1>::Zero();
}

// ── Hand tree-model initialization ──────────────────────────────────────────
void DemoTaskController::InitHandModel(const rtc_urdf_bridge::ModelConfig& /*config*/) {
  namespace rub = rtc_urdf_bridge;
  // Secondary device name == tree_model name == device_name_configs key
  // (robot-agnostic: e.g. "p1a" for ur5e_p1a, "leap" for iiwa7_leap).
  const auto secondary = GetSecondaryDeviceName();
  if (secondary.empty()) {
    return;
  }
  // Guard the tree-model lookup: a malformed config (secondary device declared
  // without a matching tree_model) would otherwise throw std::out_of_range,
  // caught by the base class as an on_configure FAILURE. Mirror the control-
  // model selection's fallback and graceful-degrade to no hand FK (#125 F2).
  try {
    hand_handle_ = std::make_unique<rub::RtModelHandle>(builder_->GetTreeModel(secondary));
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_,
                "DemoTaskController: secondary device '%s' has no matching tree_model (%s) — "
                "hand fingertip FK disabled",
                secondary.c_str(), e.what());
    return;
  }

  // Set joint reorder mapping: YAML joint_state_names → Pinocchio model order
  if (auto* hand_cfg = GetDeviceNameConfig(secondary); hand_cfg) {
    if (!hand_handle_->SetJointOrder(hand_cfg->joint_state_names)) {
      RCLCPP_WARN(logger_,
                  "DemoTaskController: secondary device '%s' SetJointOrder failed — "
                  "joint_state_names not all in Pinocchio model",
                  secondary.c_str());
    }
  }

  const auto* sys_cfg = GetSystemModelConfig();
  if (sys_cfg) {
    for (const auto& tm : sys_cfg->tree_models) {
      if (tm.name == secondary) {
        if (!tm.root_link.empty()) {
          hand_root_frame_id_ = hand_handle_->GetFrameId(tm.root_link);
          if (hand_root_frame_id_ != 0) {
            use_hand_root_frame_ = true;
          }
        }
        for (std::size_t i = 0; i < std::min(tm.tip_links.size(), kNumFingertips); ++i) {
          fingertip_frame_ids_[i] = hand_handle_->GetFrameId(tm.tip_links[i]);
        }
        break;
      }
    }
  }

  hand_q_ = Eigen::VectorXd::Zero(hand_handle_->nq());
  for (auto& p : fingertip_positions_)
    p = Eigen::Vector3d::Zero();
  for (auto& r : fingertip_rotations_)
    r = Eigen::Matrix3d::Identity();
}

// ── #121: closed-chain hand FK wiring (non-RT configure + RT dispatch) ────────

void DemoTaskController::ConfigureClosedChainHandFk() {
  if (!builder_) {
    return;
  }
  const auto primary = GetPrimaryDeviceName();
  const auto secondary = GetSecondaryDeviceName();

  // device_joint_names index order must match Compute()'s dev0/dev1: primary=0,
  // secondary=1. The closed handle's independent joints may span both devices.
  std::vector<std::vector<std::string>> dev_names;
  if (auto* c = GetDeviceNameConfig(primary); c) {
    dev_names.push_back(c->joint_state_names);
  } else {
    dev_names.emplace_back();
  }
  if (!secondary.empty()) {
    if (auto* c = GetDeviceNameConfig(secondary); c) {
      dev_names.push_back(c->joint_state_names);
    } else {
      dev_names.emplace_back();
    }
  }

  // fingertip links + hand-root frame from the secondary tree-model definition.
  std::vector<std::string> tips;
  std::string hand_root;
  if (const auto* sys = GetSystemModelConfig()) {
    for (const auto& tm : sys->tree_models) {
      if (tm.name == secondary) {
        tips = tm.tip_links;
        hand_root = tm.root_link;
        break;
      }
    }
  }

  const auto res =
      closed_hand_fk_.Configure(builder_->GetFullModel(), builder_->GetConstraintModels(),
                                builder_->GetClosureActuatedJointIds(),
                                builder_->GetClosureReferenceConfig(), dev_names, tips, hand_root);
  LogHandFkWiring(logger_, "[task]", res, closed_hand_fk_.missing_joint());
}

bool DemoTaskController::ComputeHandForwardKinematics(const ControllerState& state) noexcept {
  return RunHandForwardKinematics(closed_hand_fk_, hand_handle_.get(), hand_q_, state);
}

bool DemoTaskController::HandFingertipPose(std::size_t f, pinocchio::SE3& out) const noexcept {
  return HandFingertipPoseDispatch(closed_hand_fk_, hand_handle_.get(), fingertip_frame_ids_,
                                   use_hand_root_frame_, hand_root_frame_id_, f, out);
}

void DemoTaskController::OnDeviceConfigsSet() {
  const auto primary = GetPrimaryDeviceName();
  const auto secondary = GetSecondaryDeviceName();

  // ── Runtime DoF resolution ─────────────────────────────────────────────
  // arm_dof_ was set from YAML `estop.arm_safe_position` in LoadConfig.
  // Resolve hand_dof_ from secondary device joint_state_names and
  // cross-validate arm DoF against primary device joint_state_names.
  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
    const auto js_size = static_cast<int>(cfg->joint_state_names.size());
    if (js_size > 0 && arm_dof_ > 0 && js_size != arm_dof_) {
      RCLCPP_ERROR(logger_,
                   "[task] arm DoF mismatch: estop.arm_safe_position=%d but primary device "
                   "'%s' joint_state_names size=%d",
                   arm_dof_, primary.c_str(), js_size);
    }
  }
  hand_dof_ = 0;
  if (!secondary.empty()) {
    if (auto* cfg = GetDeviceNameConfig(secondary); cfg) {
      hand_dof_ = static_cast<int>(cfg->joint_state_names.size());
    }
  }
  if (hand_dof_ < 0 || hand_dof_ > kDemoTaskMaxHandDof) {
    RCLCPP_ERROR(logger_, "[task] hand DoF %d exceeds capacity kDemoTaskMaxHandDof=%d — clamping",
                 hand_dof_, kDemoTaskMaxHandDof);
    hand_dof_ = std::min(std::max(hand_dof_, 0), kDemoTaskMaxHandDof);
  }

  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
    if (cfg->urdf && !cfg->urdf->tip_link.empty()) {
      auto fid = arm_handle_->GetFrameId(cfg->urdf->tip_link);
      if (fid != 0) {  // 0 = universe (not found)
        tip_frame_id_ = fid;
      }
    }
    if (cfg->urdf && !cfg->urdf->root_link.empty()) {
      auto fid = arm_handle_->GetFrameId(cfg->urdf->root_link);
      if (fid != 0) {
        root_frame_id_ = fid;
        use_root_frame_ = true;
      }
    }
  }

  // ── Unified kin&dyn (Phase 4): reorder map + arm TCP frame registration ──
  // full_dof_ / reorder need the device configs resolved above; RegisterFrame
  // must run here (configure) — before the first RT cache.Update() locks
  // registration. The arm-model frame ids (tip_frame_id_/root_frame_id_) address
  // the same physical frames in the combined model (proven by
  // test_wbc_arm_tcp_cache_equivalence), so they register directly on the cache.
  full_dof_ = arm_dof_ + hand_dof_;
  {
    const auto* arm_cfg = GetDeviceNameConfig(GetPrimaryDeviceName());
    const auto* hand_cfg = GetDeviceNameConfig(GetSecondaryDeviceName());
    combined_cache_.BuildReorderMap(arm_cfg ? &arm_cfg->joint_state_names : nullptr,
                                    hand_cfg ? &hand_cfg->joint_state_names : nullptr, full_dof_,
                                    "[task]", logger_);
  }
  if (combined_cache_.model() && tip_frame_id_ != 0) {
    task_tcp_frame_idx_ = combined_cache_.cache().RegisterFrame("task_tcp", tip_frame_id_);
    if (task_tcp_frame_idx_ < 0) {
      RCLCPP_ERROR(logger_, "[task] arm TCP frame registration failed (cache locked)");
    } else if (use_root_frame_ && root_frame_id_ != 0) {
      task_base_frame_idx_ = combined_cache_.cache().RegisterFrame("task_base", root_frame_id_);
      if (task_base_frame_idx_ < 0) {
        RCLCPP_ERROR(logger_, "[task] arm base frame registration failed (cache locked)");
        task_tcp_frame_idx_ = -1;
      }
    } else {
      task_base_frame_idx_ = -1;  // world/base frame — ArmTcpPoseFromCache returns world tip
    }
  } else {
    RCLCPP_WARN(logger_, "[task] arm TCP cache disabled: tip frame unresolved");
  }
  // Lift L1: per-device joint limits (position + velocity) loaded from
  // device_name_configs_ in topic_config_ group order; missing slots get
  // ±2π / 2 rad/s fallbacks so RT clamping always has valid bounds.
  LoadDeviceLimitsFromConfig(device_position_lower_, device_position_upper_, device_max_velocity_,
                             -6.2832, 6.2832, 2.0);

  // Phase C: capture joint/sensor names for CSV header expansion.
  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
    primary_joint_names_ = cfg->joint_state_names;
  }
  if (!secondary.empty()) {
    if (auto* cfg = GetDeviceNameConfig(secondary); cfg) {
      secondary_joint_names_ = cfg->joint_state_names;
      secondary_motor_names_ = cfg->motor_state_names;
      secondary_sensor_names_ = cfg->sensor_names;
    }
  }

  // Cache fingertip sensor capability flags from the secondary (hand) device.
  // See DemoJointController::OnDeviceConfigsSet for the rationale.
  if (!secondary.empty()) {
    if (const auto layout = GetSensorLayout(secondary); layout.has_value()) {
      has_native_contact_ = layout->has_native_contact;
      has_native_displacement_ = layout->has_native_displacement;
    }
  }
  RCLCPP_INFO(logger_,
              "[task] fingertip sensor capability: has_native_contact=%d "
              "has_native_displacement=%d (secondary='%s')",
              static_cast<int>(has_native_contact_), static_cast<int>(has_native_displacement_),
              secondary.c_str());

  // #121: wire closed-chain-consistent hand FK if the model has loop closure and a
  // fingertip is downstream of a loop-passive joint. No-op (serial FK) otherwise.
  ConfigureClosedChainHandFk();
}

// ── Virtual TCP computation ─────────────────────────────────────────────────

// ── RTControllerInterface implementation ────────────────────────────────────

ControllerOutput DemoTaskController::Compute(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoTaskController::Compute");
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);

  // estop_active_ is loaded once here (before ReadState) so every downstream
  // reader — including ReadState's ExtractFullState gate and the Stage-1 cache
  // refresh below — sees one consistent value for this tick.
  estop_active_ = estopped_.load(std::memory_order_acquire);
  // F5 gate (#236 S7b), loaded beside estop_active_ for the same reason: every
  // phase below has to agree on whether device 0 is usable this tick, and both
  // output lanes — the CLIK path and ComputeEstop — honour it.
  arm_readable_ = rtc::IsDeviceReadable(state.devices[0], arm_dof_);
  // Same gate on the secondary axis (#291). Separate flag, not a widening of
  // arm_readable_: §3.7 keeps the hand commandable when the ARM goes missing,
  // and this answers the different question of whether the HAND reported the
  // hand_dof_ channels its own loops read. `num_devices > 1` folds in here so
  // the read sites carry one predicate instead of a two-term guard.
  hand_readable_ = state.num_devices > 1 && rtc::IsDeviceReadable(state.devices[1], hand_dof_);

  ReadState(state);

  // ── Stage 1 (compute model): refresh the combined-model cache from the state
  // ReadState scattered into q_curr_full_/v_curr_full_ this tick — AFTER the raw
  // read, BEFORE DrainTargetSlot and the control law. Kept at Compute scope (not
  // inside ComputeControl) on purpose: DrainTargetSlot's one-time arm self-init
  // reads ArmTcpPoseFromCache() and must see this tick's fresh FK. Same gate and
  // q_curr_full_ as the prior Compute-top Update — byte-for-byte. E-STOP ticks
  // skip it and keep arm_handle_ FK (ComputeEstop).
  // arm_readable_ joins the gate: refreshing the model from a state this tick
  // could not read would put a partially-ZERO configuration into the Jacobian
  // every consumer downstream of the cache reads.
  if (!estop_active_ && arm_readable_ && combined_cache_.reorder_valid() &&
      task_tcp_frame_idx_ >= 0) {
    combined_cache_.Update();
  }

  DrainTargetSlot(state);

  // Pose-validity flags default to invalid every tick, BEFORE the arm stage that
  // would set them. They cannot be reset inside that stage: it is skipped
  // whenever the arm is unreadable or the cache is not fresh, and the publish
  // path below still runs — so a stale `true` would republish a prior tick's
  // cached fingertip / virtual-TCP pose as though it were this tick's
  // measurement (#125 F1). Only ComputeControl sets them back to true.
  fingertip_pose_valid_.fill(false);
  vtcp_valid_ = false;
  // Same reason, one tick later in the story (#292): UpdateVirtualTcp is skipped
  // outright when the hand FK produces nothing, so without this reset the frame
  // identity ComputeControl compares against would carry a prior tick's
  // participating fingertip set into a tick that measured none.
  vtcp_member_mask_ = 0;

  // One gains snapshot for the whole tick (SeqLock: torn-read-free), shared by
  // the arm stage and the hand stage so the two halves cannot disagree.
  const auto gains = gains_lock_.Load();
  ComputeControl(state, dt, gains);
  // Secondary (hand) lane — outside the arm stage's F5 gate (§3.7 "secondary
  // passthrough 유지", see ComputeSecondary's header comment) but NOT outside
  // E-STOP. These blocks used to sit inside ComputeControl, whose early return
  // covered both conditions at once; splitting them apart means the E-STOP half
  // of that guard has to be restated here, or an E-STOP tick would keep
  // advancing the hand trajectory and stepping the grasp FSM while ComputeEstop
  // owns the wire.
  if (!estop_active_) {
    ComputeSecondary(state, dt, gains);
  }
  // Output composition split by consumer (wire / log / publish). See
  // demo_joint_controller.hpp for the bucket assignment rationale.
  auto output = WriteJointCommand(state, dt);
  FillLogOutput(state, output, dt);
  FillPublishOutput(state, output, dt);

  // ── Phase C: push log PODs (only from inside Compute()) ──────────────
  if (primary_state_log_handle_) {
    integrated_bringup::DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, output, /*device_idx=*/0, pod);
    primary_state_log_handle_.Push(pod);
  }
  if (secondary_state_log_handle_) {
    integrated_bringup::DeviceStateLogPod pod{};
    FillDeviceStateLogPod(state, output, /*device_idx=*/1, pod);
    secondary_state_log_handle_.Push(pod);
  }
  if (secondary_sensor_log_handle_) {
    integrated_bringup::DeviceSensorLogPod pod{};
    FillDeviceSensorLogPod(state, /*device_idx=*/1, num_active_fingertips_, pod);
    secondary_sensor_log_handle_.Push(pod);
  }
  PushPullEstimatorLog(pull_estimator_log_handle_, pull_wiring_, state.t_relative_s,
                       state.iteration);
  return output;
}

// ── Phase 1: Read joint states + sensor data ────────────────────────────────

// ── Phase 2: Compute control (IK/CLIK + trajectory + sensor-based logic) ────

// ── Phase 3: Write output ────────────────────────────────────────────────────

void DemoTaskController::SetDeviceTarget(int device_idx, std::span<const double> target) noexcept {
  // Off-RT marshal. The base stamps the activation generation, bounds the
  // index and the width, and queues; the RT thread drains inside Compute() and
  // is the SOLE writer of target_seqlock_.
  PushPendingTarget(device_idx, target, /*is_task=*/false);
}

// RT-thread-only. Refreshes current_target_slot_, drains off-RT pending
// entries, and runs first-tick self-init seeded from the current TCP pose.
void DemoTaskController::DrainTargetSlot(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoTaskController::DrainTargetSlot");
  current_target_slot_ = target_seqlock_.Load();
  bool slot_dirty = false;

  // ── Arm / TCP self-init — deferred until the combined-model cache is fresh ──
  // (a non-E-STOP tick with the TCP frame registered). The hold pose must seed
  // from a real cache Update, never a zero/stale frame — so if E-STOP is active
  // at activation this latches on the first clear tick instead, mirroring the
  // hand self-init's deferral until device 1 is valid.
  // arm_readable_ joins the deferral (#265 audit T1/T2): the seeds below read
  // desired_q_ and null_target from dev0.positions arm_dof_/nv deep without
  // consulting nc0, so on a narrow device the unreported joints would latch at 0
  // — "hold at the origin" — and arm_target_initialized_ makes that permanent.
  if (!arm_target_initialized_.load(std::memory_order_acquire) && !estop_active_ && arm_readable_ &&
      combined_cache_.reorder_valid() && task_tcp_frame_idx_ >= 0) {
    if (arm_dof_ == 0 && state.num_devices > 0) {
      arm_dof_ = std::min(state.devices[0].num_channels, kDemoTaskMaxArmDof);
    }

    const auto& dev0 = state.devices[0];
    // Arm TCP from the combined-model cache (refreshed by the Compute-scope
    // Stage-1 pinocchio_cache_.Update for this non-E-STOP tick, just after
    // ReadState). DrainTargetSlot runs after that Update — this self-init is the
    // reason the Update stays at Compute scope rather than inside ComputeControl.
    pinocchio::SE3 tcp_pose =
        combined_cache_.ArmTcpPoseFromCache(task_tcp_frame_idx_, task_base_frame_idx_);

    pinocchio::SE3 hold_pose = tcp_pose;
    if (ComputeHandForwardKinematics(state)) {
      UpdateVirtualTcp(tcp_pose, gains_lock_.Load());
      if (vtcp_valid_) {
        hold_pose = vtcp_pose_;
      }
    }

    // Tagged with the frame the seed was taken in (#292), reusing the same
    // vtcp_valid_ decision that picked hold_pose above. ComputeControl compares
    // that tag against the frame it controls in and re-seeds if they diverge,
    // which is what makes the end of the hand-FK walk-in free of arm motion.
    SeedHoldTarget(hold_pose, dev0, ControlFrameId{vtcp_valid_, vtcp_member_mask_, true});

    for (int i = 0; i < arm_dof_; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      // Prefer YAML seed (null_target_init_); fall back to current pose.
      current_target_slot_.null_target[idx] =
          (null_target_init_[idx] != 0.0) ? null_target_init_[idx] : dev0.positions[idx];
    }

    arm_target_initialized_.store(true, std::memory_order_release);
    slot_dirty = true;
  } else {
    // Restore RT-thread working SE3 from POD storage.
    std::memcpy(tcp_target_pose_.rotation().data(), current_target_slot_.tcp_target_rot.data(),
                sizeof(current_target_slot_.tcp_target_rot));
    std::memcpy(tcp_target_pose_.translation().data(), current_target_slot_.tcp_target_t.data(),
                sizeof(current_target_slot_.tcp_target_t));

    // ── Joint-space hold fallback (arm not yet TCP-initialized) ──────────────
    // The TCP self-init above is deferred until the cache is fresh. If the cache
    // is misconfigured (frame unregistered or reorder invalid) it never becomes
    // fresh, ComputeControl early-returns forever, and desired_q_ would stay at
    // its zero init — driving the arm to its zero configuration. Seed desired_q_
    // from the measured positions so WriteJointCommand holds the current pose
    // instead. Left un-latched (arm_target_initialized_ stays false) so the full
    // TCP self-init still runs the moment the cache does become fresh. Skipped
    // under E-STOP (ComputeEstop owns the wire command there).
    //
    // arm_readable_ replaces the `i < dev0.num_channels` term (#265 audit T3).
    // That term was the repository's only explicit nc0 bound in these four
    // controllers, and it is exactly the case #265 decision B warns about: it
    // stopped the read at nc0 but left desired_q_[nc0, nv) at its previous
    // value, which on the first tick is zero — the same partially-ZERO hold it
    // was meant to prevent. The bound stays for OOB; the gate is what refuses.
    if (!arm_target_initialized_.load(std::memory_order_acquire) && !estop_active_ &&
        arm_readable_ && arm_handle_) {
      const auto& dev0 = state.devices[0];
      const int nseed = rtc::ModelChannelBound(
          std::min(arm_handle_->nv(), static_cast<int>(desired_q_.size())), dev0.num_channels);
      for (int i = 0; i < nseed; ++i) {
        desired_q_[i] = dev0.positions[static_cast<std::size_t>(i)];
      }
    }
  }

  // ── Hand (device 1) self-init — DEFERRED until device 1 is first READABLE ───
  // Seeding while the hand is still coming up would lock targets[1] to its zero
  // init and drive every finger to 0. Retried each tick until readable; when no
  // hand device is configured the flag latches true immediately.
  //
  // hand_readable_, not `valid` (#291): the hold seed below runs hand_dof_ deep,
  // so a hand that reported only part of its channels passes `valid` and seeds
  // the unreported fingers from a phantom 0. This is the hand's T1/T2 (#265) —
  // and it is WORSE than the per-tick read sites, because the flag LATCHES on
  // success: an ordinary A site re-poisons every tick and therefore heals the
  // tick the gate goes true, while a poisoned seed here is never re-seeded.
  if (!hand_target_initialized_.load(std::memory_order_acquire)) {
    if (state.num_devices <= 1) {
      hand_target_initialized_.store(true, std::memory_order_release);
    } else if (hand_readable_) {
      if (hand_dof_ == 0) {
        hand_dof_ = std::min(state.devices[1].num_channels, kDemoTaskMaxHandDof);
      }
      for (std::size_t d = 1; d < static_cast<std::size_t>(state.num_devices); ++d) {
        const auto& dev = state.devices[d];
        if (!dev.valid) {
          continue;
        }
        for (std::size_t i = 0;
             i < static_cast<std::size_t>(dev.num_channels) && i < kMaxDeviceChannels; ++i) {
          current_target_slot_.targets[d][i] = dev.positions[i];
        }
        if (d == 1) {
          trajectory::JointSpaceTrajectory<kDemoTaskMaxHandDof>::State hold_state;
          for (int i = 0; i < hand_dof_; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            hold_state.positions[idx] = dev.positions[idx];
            hold_state.velocities[idx] = 0.0;
            hold_state.accelerations[idx] = 0.0;
          }
          hand_trajectory_.initialize(hold_state, hold_state, 0.01);
          hand_trajectory_time_ = 0.0;
          hand_new_target_pending_ = false;
        }
      }
      hand_target_initialized_.store(true, std::memory_order_release);
      slot_dirty = true;
    }
  }

  // Base drain: pops everything queued, drops what a later activation
  // invalidated (#196 §3), and calls ApplyPendingTarget for each survivor.
  if (DrainPendingTargets() > 0) {
    slot_dirty = true;
  }

  if (slot_dirty) {
    target_seqlock_.Store(current_target_slot_);
  }
}

// RT-thread-only (#292). Seeds the arm hold target at `pose` and tags it with
// the control frame that pose was taken in, so ComputeControl's transition check
// finds the goal and the control frame in agreement and CLIK sees zero task
// error. Shared by the first-tick self-init above and ComputeControl's re-seed.
//
// The self-init's null_target loop is deliberately NOT part of this: the
// nullspace posture goal is frame-independent, and re-running it on every frame
// transition would silently discard an operator's posture goal.
//
// The caller owns the target_seqlock_ Store — the self-init batches it into its
// existing slot_dirty publish, ComputeControl issues its own right after. Two
// Store sites are safe because the RT thread is the SOLE writer of that seqlock.
void DemoTaskController::SeedHoldTarget(const pinocchio::SE3& pose, const rtc::DeviceState& dev0,
                                        ControlFrameId id) noexcept {
  if (arm_handle_) {
    // rtc::ModelChannelBound where the self-init this was extracted from looped
    // to arm_handle_->nv() unbounded. Narrowing is not a weakening but alignment
    // with the S7b contract (#265 decision B): on a device reporting fewer
    // channels than the model has, the unbounded loop read dev0.positions past
    // the device's own width to seed joints it never reported.
    const int nseed = rtc::ModelChannelBound(
        std::min(arm_handle_->nv(), static_cast<int>(desired_q_.size())), dev0.num_channels);
    for (int i = 0; i < nseed; ++i) {
      desired_q_[i] = dev0.positions[static_cast<std::size_t>(i)];
    }
  }

  tcp_target_pose_ = pose;
  current_target_slot_.tcp_target = {pose.translation()[0], pose.translation()[1],
                                     pose.translation()[2]};
  std::memcpy(current_target_slot_.tcp_target_rot.data(), pose.rotation().data(),
              sizeof(current_target_slot_.tcp_target_rot));
  std::memcpy(current_target_slot_.tcp_target_t.data(), pose.translation().data(),
              sizeof(current_target_slot_.tcp_target_t));

  // Rest-to-rest at the seed pose: start == goal, so the trajectory holds and
  // computePoseError returns zero for as long as the frame does not move again.
  trajectory_.initialize(pose, pinocchio::Motion::Zero(), pose, pinocchio::Motion::Zero(), 0.01);
  trajectory_time_ = 0.0;
  has_pending_segment_ = false;
  new_target_pending_ = false;

  target_frame_id_ = id;
  target_is_hold_seed_.store(true, std::memory_order_release);
  frame_wait_ticks_.store(0, std::memory_order_release);
}

// RT tick, called once per surviving mailbox entry from DrainTargetSlot().
// Writes the RT working copy, which DrainTargetSlot publishes once at the end.
void DemoTaskController::ApplyPendingTarget(int device_idx, std::span<const double> values,
                                            bool /*is_task*/) noexcept {
  const auto didx = static_cast<std::size_t>(device_idx);
  if (didx != 0) {
    for (std::size_t i = 0; i < values.size() && i < kMaxDeviceChannels; ++i) {
      current_target_slot_.targets[didx][i] = values[i];
    }
    if (didx == 1) {
      hand_new_target_pending_ = true;
    }
    return;
  }

  // Device 0 carries the task goal. Under control_6dof it is a full SE3 pose
  // (x,y,z,r,p,y); a short goal is ignored rather than half-applied, since a
  // pose assembled from three values would command an arbitrary orientation.
  // ZYX (yaw·pitch·roll) at the wire edge — CLAUDE.md §10, shared with DemoWbc
  // through rtc::math::se3::RpyToRotationZyx so the convention has one owner.
  //
  // One snapshot for the whole function (#292): the frame tagging below needs
  // the vtcp mode, and re-Loading for it would let the goal be tagged with a
  // different gains generation than the one that decided control_6dof.
  const auto g = gains_lock_.Load();

  // #292: an external goal is authored in the controller's *intended* control
  // frame. It is NOT tagged with whatever frame happens to be live right now —
  // during the hand-FK walk-in that is tool0, and reading a vtcp-authored goal
  // as a tool0 goal is exactly the silent reinterpretation this fixes. The GUI
  // upholds the other half of the contract by disabling task-target entry until
  // the frame is settled, so authoring inside that window is not possible.
  // Left unbound: the participating fingertip set cannot be known off-RT, so
  // ComputeControl binds it on the first tick the frames agree.
  const auto tag_external_goal = [this, &g]() noexcept {
    target_is_hold_seed_.store(false, std::memory_order_release);
    frame_wait_ticks_.store(0, std::memory_order_release);
    target_frame_id_ = ControlFrameId{
        (g.vtcp.mode != VirtualTcpMode::kDisabled) && (hand_handle_ != nullptr), 0, false};
  };

  if (g.control_6dof) {
    if (values.size() >= 6) {
      current_target_slot_.tcp_target[0] = values[0];
      current_target_slot_.tcp_target[1] = values[1];
      current_target_slot_.tcp_target[2] = values[2];
      const Eigen::Matrix3d rotation =
          rtc::math::se3::RpyToRotationZyx(Eigen::Vector3d(values[3], values[4], values[5]));
      const Eigen::Vector3d translation(values[0], values[1], values[2]);
      std::memcpy(current_target_slot_.tcp_target_rot.data(), rotation.data(),
                  sizeof(current_target_slot_.tcp_target_rot));
      std::memcpy(current_target_slot_.tcp_target_t.data(), translation.data(),
                  sizeof(current_target_slot_.tcp_target_t));
      tcp_target_pose_.translation() = translation;
      tcp_target_pose_.rotation() = rotation;
      new_target_pending_ = true;
      tag_external_goal();
    }
    return;
  }

  // Position-only mode. The goal is POSITIONALLY ALIGNED with the arm joint
  // vector, not a [xyz | posture] concatenation: values[0..2] are the TCP
  // position and values[3..arm_dof) set null_target[3..arm_dof) — the nullspace
  // posture for joints 3 and up. Joints 0-2 keep whatever the first-tick
  // self-init seeded (null_target_init_ from YAML, else the measured pose),
  // because those three slots carry the TCP position on the wire instead.
  // A client that sends [x,y,z] followed by a FULL posture q[0..n) has its
  // posture applied shifted by three; the wire format has no room for one.
  const auto cap = (arm_dof_ > 0) ? static_cast<std::size_t>(arm_dof_)
                                  : static_cast<std::size_t>(kDemoTaskMaxArmDof);
  const std::size_t pn = std::min(values.size(), cap);
  for (std::size_t i = 0; i < std::min(pn, std::size_t{3}); ++i) {
    current_target_slot_.tcp_target[i] = values[i];
  }
  for (std::size_t i = 3; i < pn; ++i) {
    current_target_slot_.null_target[i] = values[i];
  }
  new_target_pending_ = true;
  tag_external_goal();
}

std::string_view DemoTaskController::Name() const noexcept {
  return "DemoTaskController";
}

void DemoTaskController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void DemoTaskController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
}

bool DemoTaskController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void DemoTaskController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

// ── Private helpers ──────────────────────────────────────────────────────────

// ── Controller registry hooks ────────────────────────────────────────────────

void DemoTaskController::LoadConfig(const YAML::Node& cfg) {
  RTControllerInterface::LoadConfig(cfg);
  if (!cfg) {
    return;
  }

  // ── Build arm model from system model config or bridge YAML ──────────────
  namespace rub = rtc_urdf_bridge;
  const auto* sys_cfg = GetSystemModelConfig();
  if (sys_cfg && !sys_cfg->urdf_path.empty() && !sys_cfg->sub_models.empty()) {
    // System-level ModelConfig (top-level "urdf:" YAML section)
    InitArmModel(*sys_cfg);
  } else if (cfg["model_config"]) {
    // Fallback: separate model config YAML file (backward compatibility)
    const auto yaml_name = cfg["model_config"].as<std::string>();
    const auto yaml_path =
        ament_index_cpp::get_package_share_directory("integrated_bringup") + "/config/" + yaml_name;
    auto model_cfg = rub::PinocchioModelBuilder::LoadModelConfig(yaml_path);
    model_cfg.urdf_path = urdf_path_;
    InitArmModel(model_cfg);
  } else if (!urdf_path_.empty()) {
    // Fallback: arm-only URDF, no sub-model extraction
    rub::ModelConfig model_cfg;
    model_cfg.urdf_path = urdf_path_;
    model_cfg.root_joint_type = "fixed";
    model_cfg.sub_models.push_back({"arm", "base_link", "tool0"});
    InitArmModel(model_cfg);
  }

  // ── Build hand tree-model if configured ─────────────────────────────
  if (sys_cfg && !sys_cfg->tree_models.empty()) {
    InitHandModel(*sys_cfg);
  }

  // ── Unified kin&dyn (#174): combined-model cache on the same builder ──
  // Arm TCP FK/Jacobian are sourced from this cache on the RT path (arm_handle_
  // retained only for the E-STOP TF path). Frames registered in OnDeviceConfigsSet.
  // Task has no TSID contact frames → empty contact-frame list.
  if (builder_) {
    (void)combined_cache_.InitModel(*builder_, /*contact_frame_ids=*/{}, "[task]", logger_);
  }

  // ── E-STOP arm safe position (required) ─────────────────────────────
  // Lift L2: estop arm safe position parsing.
  //
  // Runtime arm_dof_ is established from the YAML's `estop.arm_safe_position`
  // length (authoritative at LoadConfig time; device configs arrive later in
  // OnDeviceConfigsSet, which cross-checks joint_state_names size).
  if (!cfg["estop"] || !cfg["estop"]["arm_safe_position"] ||
      !cfg["estop"]["arm_safe_position"].IsSequence()) {
    throw std::runtime_error(
        "demo_task_controller: required 'estop.arm_safe_position' must be a sequence");
  }
  {
    const auto seq_size = cfg["estop"]["arm_safe_position"].size();
    if (seq_size == 0 || seq_size > static_cast<std::size_t>(kDemoTaskMaxArmDof)) {
      throw std::runtime_error("demo_task_controller: 'estop.arm_safe_position' size " +
                               std::to_string(seq_size) + " out of range [1, " +
                               std::to_string(kDemoTaskMaxArmDof) + "]");
    }
    arm_dof_ = static_cast<int>(seq_size);
    const auto sp = ParseArmSafePosition(cfg, seq_size, "demo_task_controller");
    safe_position_.fill(0.0);
    for (std::size_t i = 0; i < seq_size; ++i) {
      safe_position_[i] = sp[i];
    }
  }

  auto g = gains_lock_.Load();
  // CLIK gains — translation / rotation separated. Shape-checked rather than
  // fitted to whatever length arrives (#302).
  //
  // This binding already enforces the rule on its OTHER surface:
  // OnGainParametersSet rejects `v.size() != 3` with "kp_translation requires 3
  // values", so `ros2 param set` and the BT SetGains node cannot install a
  // mis-shaped gain. The YAML path used `min(size, 3)` and disagreed — the same
  // key, on the same controller, accepted from a file what it refuses from a
  // parameter. The two surfaces now say the same thing.
  //
  // `min(size, 3)` was also the worst of the three spellings this repo had,
  // because it PARTIALLY fills: `kp_translation: [5.0, 5.0]` runs [5, 5,
  // <default>], a gain no operator wrote and no diagnostic can flag, since the
  // entries that did come from YAML make the POD look deliberate. The schema
  // parsers' silent form at least left the whole triple at its default. An
  // over-long sequence dropped the tail in silence, which is the #172 defect
  // proper.
  auto load3 = [&cfg](const char* key, std::array<double, 3>& arr) {
    const YAML::Node& n = cfg[key];
    if (!n) {
      return;
    }
    if (!n.IsSequence() || n.size() != 3) {
      throw std::runtime_error(std::string("demo_task_controller: '") + key +
                               "' must be a 3-entry sequence [x,y,z]");
    }
    for (std::size_t i = 0; i < 3; ++i) {
      arr[i] = n[i].as<double>();
    }
  };
  load3("kp_translation", g.kp_translation);
  load3("kp_rotation", g.kp_rotation);
  // §6.5 σ_min-adaptive DLS (#282). Both floors are the shared symbols, not a
  // hand-written std::max — `max(1e-4, NaN) == 1e-4` would launder a non-finite
  // λ_max into a plausible one and hand the caller a damping shell that only
  // looks armed (NUM-1), and `max(1e-6, NaN) == 1e-6` does the same to σ₀ for a
  // shell so narrow no reachable pose enters it (NUM-2). ComputeControl floors
  // BOTH again at the point of use.
  if (cfg["singularity_threshold"]) {
    g.singularity_threshold =
        rtc::compliance::FloorSigma0(cfg["singularity_threshold"].as<double>());
  }
  if (cfg["max_damping"]) {
    g.max_damping = rtc::compliance::FloorMaxDamping(cfg["max_damping"].as<double>());
  }
  if (cfg["damping"]) {
    // Retired in #282. Reported rather than mapped onto max_damping, for the
    // same reason the five rtc_controllers schemas give (#236 S2b/S3b): a
    // constant λ and the ceiling of a σ_min-adaptive ramp are not the same
    // quantity, so any mapping would be a guess. WARN and not a throw because
    // this controller ships in three robot configs and a stale key must not
    // stop a bring-up — but the ramp then runs on ITS defaults, not on 0.01.
    RCLCPP_WARN(logger_,
                "demo_task_controller: 'damping' is retired (#282) and IGNORED — it is not "
                "mapped onto max_damping. Replace it with max_damping / "
                "singularity_threshold; the §6.5 ramp is running on %.3g / %.3g.",
                g.max_damping, g.singularity_threshold);
  }
  if (cfg["null_kp"]) {
    g.null_kp = cfg["null_kp"].as<double>();
  }
  // NUM-6 (#277), unconditionally rather than inside the parse above: when the
  // key is ABSENT the value still arrives from the constructor or a previous
  // `ros2 param set` / BT SetGains, which is where a negative gain actually
  // reaches this controller. q̇₀ = K_p·(q_null − q) with K_p < 0 drives the
  // posture away from its target and (I − J⁺J) hides that from the Cartesian
  // task, so it is a silent drift and not a fault. ComputeControl floors it
  // again at the point of use.
  g.null_kp = rtc::FloorNonNegativeGain(g.null_kp);
  if (cfg["enable_null_space"]) {
    g.enable_null_space = cfg["enable_null_space"].as<bool>();
  }
  if (cfg["control_6dof"]) {
    g.control_6dof = cfg["control_6dof"].as<bool>();
  }

  // Trajectory speed
  if (cfg["trajectory_speed"]) {
    g.trajectory_speed = std::max(1e-6, cfg["trajectory_speed"].as<double>());
  }
  if (cfg["trajectory_angular_speed"]) {
    g.trajectory_angular_speed = std::max(1e-6, cfg["trajectory_angular_speed"].as<double>());
  }
  if (cfg["hand_trajectory_speed"]) {
    g.hand_trajectory_speed = std::max(1e-6, cfg["hand_trajectory_speed"].as<double>());
  }
  if (cfg["max_traj_velocity"]) {
    g.max_traj_velocity = cfg["max_traj_velocity"].as<double>();
  }
  if (cfg["max_traj_angular_velocity"]) {
    g.max_traj_angular_velocity = cfg["max_traj_angular_velocity"].as<double>();
  }
  if (cfg["hand_max_traj_velocity"]) {
    g.hand_max_traj_velocity = cfg["hand_max_traj_velocity"].as<double>();
  }

  // ── FSM / trajectory tuning (required) ──────────────────────────────────
  if (!cfg["fsm"] || !cfg["fsm"].IsMap()) {
    throw std::runtime_error("demo_task_controller: required 'fsm' section is missing");
  }
  {
    const auto& fsm = cfg["fsm"];
    if (!fsm["pi_rotation_margin"]) {
      throw std::runtime_error(
          "demo_task_controller: required 'fsm.pi_rotation_margin' is missing");
    }
    const double pi_margin = fsm["pi_rotation_margin"].as<double>();
    if (!(pi_margin >= 0.0 && pi_margin <= M_PI_2)) {
      throw std::runtime_error(
          "demo_task_controller: 'fsm.pi_rotation_margin' out of range "
          "[0, pi/2]");
    }
    g.pi_rotation_margin = pi_margin;

    if (!fsm["contact_stop_release_eps"]) {
      throw std::runtime_error(
          "demo_task_controller: required 'fsm.contact_stop_release_eps' "
          "is missing");
    }
    const double eps = fsm["contact_stop_release_eps"].as<double>();
    if (!(eps >= 0.0 && eps <= 0.1)) {
      throw std::runtime_error(
          "demo_task_controller: 'fsm.contact_stop_release_eps' out of range "
          "[0, 0.1]");
    }
    g.contact_stop_release_eps = eps;

    // Optional: LPF cutoff for the latched contact_stop hold (default from Gains).
    // Must stay below Nyquist (control_rate / 2) or BesselFilter::Init throws.
    if (fsm["contact_stop_lpf_cutoff_hz"]) {
      const double cutoff = fsm["contact_stop_lpf_cutoff_hz"].as<double>();
      const double nyquist = 0.5 / GetDefaultDt();
      if (!(cutoff > 0.0 && cutoff < nyquist)) {
        throw std::runtime_error(
            "demo_task_controller: 'fsm.contact_stop_lpf_cutoff_hz' out of range "
            "(0, control_rate/2)");
      }
      g.contact_stop_lpf_cutoff_hz = cutoff;
    }
  }

  // ── Shared params: defaults from demo_shared.yaml, overridden by cfg ──
  // config_variant is declared on the per-controller LifecycleNode by the CM
  // (see rt_controller_node_params.cpp).  Empty → legacy flat layout.
  DemoSharedConfig shared;
  const std::string variant = node_ && node_->has_parameter("config_variant")
                                  ? node_->get_parameter("config_variant").as_string()
                                  : std::string{};
  LoadDemoSharedYamlFile(shared, variant);
  ApplyDemoSharedConfig(cfg, shared);

  g.vtcp = shared.vtcp;
  g.grasp_contact_threshold = shared.grasp_contact_threshold;
  g.grasp_force_threshold = shared.grasp_force_threshold;
  g.grasp_min_fingertips = shared.grasp_min_fingertips;
  gains_lock_.Store(g);
  // Already whitelist-validated in ApplyDemoSharedConfig; resolve to the enum
  // the RT hot path branches on.
  grasp_hand_mode_ = ParseGraspHandMode(shared.grasp_controller_type);
  num_grasp_fingers_ = shared.num_grasp_fingers;
  finger_dof_ = shared.finger_dof;
  finger_joint_map_ = shared.hand_finger_joint_map;
  num_release_gates_ = shared.num_release_gates;
  for (int f = 0; f < num_release_gates_ && f < rtc::grasp::kMaxGraspFingers; ++f) {
    const auto idx = static_cast<std::size_t>(f);
    release_gate_idx_[idx] = static_cast<std::size_t>(shared.hand_release_gate[idx].joint_index);
    release_gate_sign_[idx] = shared.hand_release_gate[idx].loosen_sign;
  }

  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    command_type_ = (s == "torque") ? CommandType::kTorque : CommandType::kPosition;
  }

  BuildGraspController(shared, 1.0 / GetDefaultDt(), grasp_controller_);

  // ── #167 P3: pull estimator + tip-link → FK slot resolve ────────────────
  // Slot order = tree-model tip_links order (the fingertip_data_ /
  // fingertip_rotations_ indexing every consumer in this controller uses),
  // capped at kNumFingertips (only those slots receive FK poses). Wiring
  // errors (unknown link / missing thumb role) throw → configure FAILURE.
  {
    const std::vector<std::string> pull_links =
        ResolvePullTipLinks(GetSystemModelConfig(), GetSecondaryDeviceName(), kNumFingertips);
    ConfigurePullEstimatorWiring(shared, 1.0 / GetDefaultDt(), pull_links, pull_wiring_);
    LogPullEstimatorWiring(logger_, pull_wiring_, shared);
  }

  // contact_stop hold LPF: Init at config time (may throw) so Apply() on the RT
  // path stays allocation- and throw-free. Reset the latch to a clean state.
  hand_pos_filter_.Init(g.contact_stop_lpf_cutoff_hz, 1.0 / GetDefaultDt());
  hand_pos_filter_.Reset();
  contact_latched_ = false;
  hand_hold_position_.fill(0.0);

  // ── Phase C: parse `logs:` section ──────────────────────────────────────
  parsed_log_entries_.clear();
  if (cfg["logs"]) {
    if (!cfg["logs"].IsSequence()) {
      throw std::runtime_error("DemoTaskController: 'logs' must be a sequence");
    }
    for (const auto& entry : cfg["logs"]) {
      if (!entry.IsMap() || !entry["msg_type"]) {
        throw std::runtime_error("DemoTaskController: each `logs` entry needs `msg_type`");
      }
      ParsedLogEntry e;
      e.msg_type = entry["msg_type"].as<std::string>();
      if (entry["instance"]) {
        e.instance = entry["instance"].as<std::string>();
      }
      if (e.msg_type != "rtc_msgs/DeviceStateLog" && e.msg_type != "rtc_msgs/DeviceSensorLog" &&
          e.msg_type != integrated_bringup::kPullEstimatorLogMsgType) {
        throw std::runtime_error("DemoTaskController: unknown msg_type in `logs`: " + e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
}

// ── Phase 4: controller-owned topic lifecycle ─────────────────────────────

void DemoTaskController::PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept {
  const auto grasp_loaded = grasp_state_lock_.Load();
  const auto tof_loaded = tof_snapshot_lock_.Load();
  PublishOwnedTopicsFromSnapshot(snap, owned_topics_, /*grasp=*/&grasp_loaded, /*wbc=*/nullptr,
                                 /*tof=*/&tof_loaded);
}

// ── Phase B: gain → ROS 2 parameter declaration & callback ────────────────
//
// Declare every tunable gain as a parameter on the controller's LifecycleNode,
// seeded from the current `gains_lock_` snapshot (which LoadConfig populated
// from YAML). After declare, read each parameter back so any startup overrides
// (--params-file) supersede the YAML defaults, and store the final Gains
// snapshot via gains_lock_.Store() — RT-safe by SeqLock.
//
// Read-only fields (D-2): max_traj_velocity, max_traj_angular_velocity,
// hand_max_traj_velocity. Declared with read_only=true; honour startup
// overrides but reject runtime `ros2 param set`.

}  // namespace integrated_bringup
