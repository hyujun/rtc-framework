#include "integrated_bringup/controllers/demo_joint_controller.hpp"
#include "integrated_bringup/logging/pod_fill.hpp"
#include "integrated_bringup/support/demo_shared_config.hpp"
#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_controller_interface/device_readability.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/math.hpp>
#pragma GCC diagnostic pop

namespace integrated_bringup {

DemoJointController::DemoJointController(std::string_view urdf_path)
    : DemoJointController(urdf_path, Gains{}) {}

DemoJointController::DemoJointController(std::string_view urdf_path, Gains gains)
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

void DemoJointController::InitArmModel(const rtc_urdf_bridge::ModelConfig& config) {
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
}

// ── Hand tree-model initialization ──────────────────────────────────────────
void DemoJointController::InitHandModel(const rtc_urdf_bridge::ModelConfig& /*config*/) {
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
                "DemoJointController: secondary device '%s' has no matching tree_model (%s) — "
                "hand fingertip FK disabled",
                secondary.c_str(), e.what());
    return;
  }

  // Set joint reorder mapping: YAML joint_state_names → Pinocchio model order
  if (auto* hand_cfg = GetDeviceNameConfig(secondary); hand_cfg) {
    if (!hand_handle_->SetJointOrder(hand_cfg->joint_state_names)) {
      RCLCPP_WARN(logger_,
                  "DemoJointController: secondary device '%s' SetJointOrder failed — "
                  "joint_state_names not all in Pinocchio model",
                  secondary.c_str());
    }
  }

  // Resolve fingertip frame IDs from tree_model tip_links
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

  // Pre-allocate hand joint vector
  hand_q_ = Eigen::VectorXd::Zero(hand_handle_->nq());

  // Initialize position/rotation buffers
  for (auto& p : fingertip_positions_)
    p = Eigen::Vector3d::Zero();
  for (auto& r : fingertip_rotations_)
    r = Eigen::Matrix3d::Identity();
}

// ── #121: closed-chain hand FK wiring (non-RT configure + RT dispatch) ────────

void DemoJointController::ConfigureClosedChainHandFk() {
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
  LogHandFkWiring(logger_, "[joint]", res, closed_hand_fk_.missing_joint());
}

bool DemoJointController::ComputeHandForwardKinematics(const ControllerState& state) noexcept {
  return RunHandForwardKinematics(closed_hand_fk_, hand_handle_.get(), hand_q_, state);
}

bool DemoJointController::HandFingertipPose(std::size_t f, pinocchio::SE3& out) const noexcept {
  return HandFingertipPoseDispatch(closed_hand_fk_, hand_handle_.get(), fingertip_frame_ids_,
                                   use_hand_root_frame_, hand_root_frame_id_, f, out);
}

void DemoJointController::OnDeviceConfigsSet() {
  const auto primary = GetPrimaryDeviceName();
  const auto secondary = GetSecondaryDeviceName();

  // ── Runtime DoF resolution ─────────────────────────────────────────────
  // arm_dof_ was set from YAML `estop.arm_safe_position` in LoadConfig.
  // Resolve hand_dof_ from secondary device joint_state_names size and
  // cross-validate arm DoF against the primary device's joint_state_names.
  if (auto* cfg = GetDeviceNameConfig(primary); cfg) {
    const auto js_size = static_cast<int>(cfg->joint_state_names.size());
    if (js_size > 0 && arm_dof_ > 0 && js_size != arm_dof_) {
      RCLCPP_ERROR(logger_,
                   "[joint] arm DoF mismatch: estop.arm_safe_position=%d but primary device "
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
  if (hand_dof_ < 0 || hand_dof_ > kDemoJointMaxHandDof) {
    RCLCPP_ERROR(logger_, "[joint] hand DoF %d exceeds capacity kDemoJointMaxHandDof=%d — clamping",
                 hand_dof_, kDemoJointMaxHandDof);
    hand_dof_ = std::min(std::max(hand_dof_, 0), kDemoJointMaxHandDof);
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

  // ── Unified kin&dyn (Phase 5): reorder map + arm TCP frame registration ──
  // full_dof_ / reorder need the device configs resolved above; RegisterFrame
  // must run here (configure) — before the first RT cache.Update() locks
  // registration. The arm-model frame ids address the same physical frames in
  // the combined model (proven by test_wbc_arm_tcp_cache_equivalence).
  full_dof_ = arm_dof_ + hand_dof_;
  {
    const auto* arm_cfg = GetDeviceNameConfig(GetPrimaryDeviceName());
    const auto* hand_cfg = GetDeviceNameConfig(GetSecondaryDeviceName());
    combined_cache_.BuildReorderMap(arm_cfg ? &arm_cfg->joint_state_names : nullptr,
                                    hand_cfg ? &hand_cfg->joint_state_names : nullptr, full_dof_,
                                    "[joint]", logger_);
  }
  if (combined_cache_.model() && tip_frame_id_ != 0) {
    arm_tcp_frame_idx_ = combined_cache_.cache().RegisterFrame("arm_tcp", tip_frame_id_);
    if (arm_tcp_frame_idx_ < 0) {
      RCLCPP_ERROR(logger_, "[joint] arm TCP frame registration failed (cache locked)");
    } else if (use_root_frame_ && root_frame_id_ != 0) {
      arm_base_frame_idx_ = combined_cache_.cache().RegisterFrame("arm_base", root_frame_id_);
      if (arm_base_frame_idx_ < 0) {
        RCLCPP_ERROR(logger_, "[joint] arm base frame registration failed (cache locked)");
        arm_tcp_frame_idx_ = -1;
      }
    } else {
      arm_base_frame_idx_ = -1;  // world/base frame — ArmTcpPoseFromCache returns world tip
    }
  } else {
    RCLCPP_WARN(logger_, "[joint] arm TCP cache disabled: tip frame unresolved");
  }
  // Lift L1: per-device joint limits (position + velocity) loaded from
  // device_name_configs_ in topic_config_ group order; missing slots get
  // ±2π / 2 rad/s fallbacks so RT clamping always has valid bounds.
  LoadDeviceLimitsFromConfig(device_position_lower_, device_position_upper_, device_max_velocity_,
                             -6.2832, 6.2832, 2.0);

  // Capture joint/sensor names for CSV header expansion (Phase C).
  // Header writers run once at file open in on_configure → these vectors
  // are read non-RT only.
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
  // Both default false — primary-only configs or hand devices without a
  // sensor_layout block stay on the force-only derive path.
  if (!secondary.empty()) {
    if (const auto layout = GetSensorLayout(secondary); layout.has_value()) {
      has_native_contact_ = layout->has_native_contact;
      has_native_displacement_ = layout->has_native_displacement;
    }
  }
  RCLCPP_INFO(logger_,
              "[joint] fingertip sensor capability: has_native_contact=%d "
              "has_native_displacement=%d (secondary='%s')",
              static_cast<int>(has_native_contact_), static_cast<int>(has_native_displacement_),
              secondary.c_str());

  // #121: wire closed-chain-consistent hand FK if the model has loop closure and a
  // fingertip is downstream of a loop-passive joint. No-op (serial FK) otherwise.
  ConfigureClosedChainHandFk();
}

ControllerOutput DemoJointController::Compute(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoJointController::Compute");
  const double dt = (state.dt > 0.0) ? state.dt : (1.0 / 500.0);

  // estop_active_ is loaded once here (before ReadState) so all downstream
  // readers — including ReadState's ExtractFullState gate — see one consistent
  // value for this tick. The combined-model cache refresh is split across the RT
  // function roles: ExtractFullState (raw read + reindex) lives in ReadState;
  // pinocchio_cache_.Update (compute model) is ComputeControl Stage 1, before the
  // trajectory control law and the arm-TCP FK it consumes. E-STOP ticks skip both
  // and keep arm_handle_ FK (ComputeEstop).
  estop_active_ = estopped_.load(std::memory_order_acquire);
  // F5 gate, loaded next to estop_active_ and for the same reason: every phase
  // below has to agree on whether device 0 is usable this tick. Both output
  // lanes honour it — the E-STOP branch just below and the normal path — so
  // there is one answer to "unreadable arm" rather than one per lane (#236 S7b,
  // §3.7). arm_dof_ may still be 0 on the very first tick of a fixture that
  // bypasses YAML; IsDeviceReadable then degrades to a plain validity check,
  // which is the honest answer while nothing is known to be missing.
  arm_readable_ = rtc::IsDeviceReadable(state.devices[0], arm_dof_);
  // Same gate on the secondary axis (#291). Separate flag, not a widening of
  // arm_readable_: §3.7 keeps the hand commandable when the ARM goes missing,
  // and this answers the different question of whether the HAND reported the
  // hand_dof_ channels its own loops read. `num_devices > 1` folds in here so
  // the read sites carry one predicate instead of a two-term guard. hand_dof_
  // may still be 0 in a fixture that bypasses YAML, and IsDeviceReadable then
  // degrades to the plain validity check those sites used before.
  hand_readable_ = state.num_devices > 1 && rtc::IsDeviceReadable(state.devices[1], hand_dof_);

  ReadState(state);
  // RT-thread-only: refresh current_target_slot_ + run self-init if needed.
  // After this call ComputeControl / WriteJointCommand / FillLogOutput /
  // FillPublishOutput must read from
  // current_target_slot_, never from any old device_targets_ member.
  DrainTargetSlot(state);
  if (estop_active_) {
    auto out = ComputeEstop(state);
    out.command_type = command_type_;
    // Push minimal logs even in E-STOP so the file shows the gap clearly.
    if (primary_state_log_handle_) {
      integrated_bringup::DeviceStateLogPod pod{};
      FillDeviceStateLogPod(state, out, /*device_idx=*/0, pod);
      primary_state_log_handle_.Push(pod);
    }
    if (secondary_state_log_handle_) {
      integrated_bringup::DeviceStateLogPod pod{};
      FillDeviceStateLogPod(state, out, /*device_idx=*/1, pod);
      secondary_state_log_handle_.Push(pod);
    }
    if (secondary_sensor_log_handle_) {
      integrated_bringup::DeviceSensorLogPod pod{};
      FillDeviceSensorLogPod(state, /*device_idx=*/1, num_active_fingertips_, pod);
      secondary_sensor_log_handle_.Push(pod);
    }
    // Controller-owned publish state + pull CSV row for THIS tick (#234 P-1):
    // the CM publishes every tick regardless, so a stored-nothing tick would
    // ship the pre-E-STOP body under the current stamp. The CSV keeps a row
    // (valid=0) rather than a gap so it stays aligned with the sibling
    // *_state.csv channels above.
    FillEstopPublishState(dt);
    PushPullEstimatorLog(pull_estimator_log_handle_, pull_wiring_, state.t_relative_s,
                         state.iteration);
    return out;
  }
  ComputeControl(state, dt);
  // Output composition is split by consumer (wire / log / publish) so each
  // method's body lists exactly the fields the consumer reads. See
  // demo_joint_controller.hpp for the bucket assignment rationale.
  auto output = WriteJointCommand(state, dt);
  FillLogOutput(state, output, dt);
  FillPublishOutput(state, output, dt);

  // ── Phase C: push log PODs to controller-owned channels ────────────────
  // Push site is INSIDE Compute() per Q-ACTIVITY-GATING — inactive
  // controllers' Compute() is never called, so their CSVs do not grow.
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

// ReadState / UpdateVirtualTcp / ComputeControl /
// WriteJointCommand / FillLogOutput / FillPublishOutput / ComputeEstop
// live in demo_joint_controller_compute.cpp.

void DemoJointController::SetDeviceTarget(int device_idx, std::span<const double> target) noexcept {
  // Off-RT marshal. The base stamps the activation generation, bounds the
  // index and the width, and queues; the RT thread drains inside Compute() and
  // is the SOLE writer of target_seqlock_.
  PushPendingTarget(device_idx, target, /*is_task=*/false);
}

// RT tick, called once per surviving mailbox entry from DrainTargetSlot().
// Writes the RT working copy, which DrainTargetSlot publishes once at the end;
// storing per entry would put a burst of goals through the SeqLock for no
// reader benefit.
void DemoJointController::ApplyPendingTarget(int device_idx, std::span<const double> values,
                                             bool /*is_task*/) noexcept {
  auto& row = current_target_slot_.targets[static_cast<std::size_t>(device_idx)];
  for (std::size_t i = 0; i < values.size() && i < row.size(); ++i) {
    row[i] = values[i];
  }
  // Device 0 is the arm, device 1 the hand: each has its own trajectory that
  // must be re-planned from the new goal on this tick.
  if (device_idx == 0) {
    robot_new_target_pending_ = true;
  } else if (device_idx == 1) {
    hand_new_target_pending_ = true;
  }
}

// RT-thread-only. Refreshes current_target_slot_ from the SeqLock, drains any
// off-RT pending entries, and runs first-tick self-init (seeding the slot
// from the current device state + trajectories). The RT thread is the sole
// writer of target_seqlock_; SetDeviceTarget is marshal-only.
void DemoJointController::DrainTargetSlot(const ControllerState& state) noexcept {
  RTC_TRACE_SCOPE("DemoJointController::DrainTargetSlot");
  current_target_slot_ = target_seqlock_.Load();
  bool slot_dirty = false;

  // ── Arm (device 0) self-init — DEFERRED until device 0 is readable ─────────
  // It used to run on the first tick unconditionally, seeding arm_dof_ slots
  // from dev0.positions without ever consulting num_channels (#265 audit J1) —
  // so a device narrower than the configured arm would latch a hold target
  // whose unreported joints are 0, i.e. "go to the origin", and the latch made
  // it permanent. Retried each tick, exactly like the hand's deferred self-init
  // below; the two answers in this one function are now the same answer.
  if (!arm_target_initialized_.load(std::memory_order_acquire) && arm_readable_) {
    // Fallback DoF when LoadConfig/OnDeviceConfigsSet hasn't populated runtime
    // dimensions (e.g. unit tests that bypass YAML). Bounded by nc0, so the
    // resolved arm_dof_ can never re-open the gate this branch just passed.
    if (arm_dof_ == 0 && state.num_devices > 0) {
      arm_dof_ = std::min(state.devices[0].num_channels, kDemoJointMaxArmDof);
    }

    const auto& dev0 = state.devices[0];
    trajectory::JointSpaceTrajectory<kDemoJointMaxArmDof>::State hold_state;
    for (int i = 0; i < arm_dof_; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      current_target_slot_.targets[0][idx] = dev0.positions[idx];
      hold_state.positions[idx] = dev0.positions[idx];
      hold_state.velocities[idx] = 0.0;
      hold_state.accelerations[idx] = 0.0;
    }
    robot_trajectory_.initialize(hold_state, hold_state, 0.01);
    robot_trajectory_time_ = 0.0;
    robot_new_target_pending_ = false;

    arm_target_initialized_.store(true, std::memory_order_release);
    slot_dirty = true;
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
        hand_dof_ = std::min(state.devices[1].num_channels, kDemoJointMaxHandDof);
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
          trajectory::JointSpaceTrajectory<kDemoJointMaxHandDof>::State hold_state;
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

// ── Controller registry hooks ────────────────────────────────────────────────

void DemoJointController::LoadConfig(const YAML::Node& cfg) {
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
  // Arm TCP FK is sourced from this cache on the RT path (arm_handle_ retained
  // only for the E-STOP TF path). Frames registered in OnDeviceConfigsSet. Joint
  // has no TSID contact frames → empty contact-frame list.
  if (builder_) {
    // Return ignored: downstream reorder / RegisterFrame gate on combined_cache_.
    // model() being non-null, matching the prior void InitControlModelCache path.
    (void)combined_cache_.InitModel(*builder_, /*contact_frame_ids=*/{}, "[joint]", logger_);
  }

  // ── L2: E-STOP arm safe position (required) ──────────────────────────
  //
  // Runtime arm_dof_ is established from the YAML's `estop.arm_safe_position`
  // length (authoritative source at LoadConfig time; device configs arrive
  // later in OnDeviceConfigsSet, which cross-checks joint_state_names size).
  if (!cfg["estop"] || !cfg["estop"]["arm_safe_position"] ||
      !cfg["estop"]["arm_safe_position"].IsSequence()) {
    throw std::runtime_error(
        "demo_joint_controller: required 'estop.arm_safe_position' must be a sequence");
  }
  {
    const auto seq_size = cfg["estop"]["arm_safe_position"].size();
    if (seq_size == 0 || seq_size > static_cast<std::size_t>(kDemoJointMaxArmDof)) {
      throw std::runtime_error("demo_joint_controller: 'estop.arm_safe_position' size " +
                               std::to_string(seq_size) + " out of range [1, " +
                               std::to_string(kDemoJointMaxArmDof) + "]");
    }
    arm_dof_ = static_cast<int>(seq_size);
    const auto sp = ParseArmSafePosition(cfg, seq_size, "demo_joint_controller");
    safe_position_.fill(0.0);
    for (std::size_t i = 0; i < seq_size; ++i) {
      safe_position_[i] = sp[i];
    }
  }

  auto g = gains_lock_.Load();
  if (cfg["robot_trajectory_speed"]) {
    g.robot_trajectory_speed = std::max(1e-6, cfg["robot_trajectory_speed"].as<double>());
  }
  if (cfg["hand_trajectory_speed"]) {
    g.hand_trajectory_speed = std::max(1e-6, cfg["hand_trajectory_speed"].as<double>());
  }
  if (cfg["robot_max_traj_velocity"]) {
    g.robot_max_traj_velocity = cfg["robot_max_traj_velocity"].as<double>();
  }
  if (cfg["hand_max_traj_velocity"]) {
    g.hand_max_traj_velocity = cfg["hand_max_traj_velocity"].as<double>();
  }

  // ── FSM tuning (required) ───────────────────────────────────────────
  if (!cfg["fsm"] || !cfg["fsm"].IsMap()) {
    throw std::runtime_error("demo_joint_controller: required 'fsm' section is missing");
  }
  {
    const auto& fsm = cfg["fsm"];
    if (!fsm["contact_stop_release_eps"]) {
      throw std::runtime_error(
          "demo_joint_controller: required 'fsm.contact_stop_release_eps' "
          "is missing");
    }
    const double eps = fsm["contact_stop_release_eps"].as<double>();
    if (!(eps >= 0.0 && eps <= 0.1)) {
      throw std::runtime_error(
          "demo_joint_controller: 'fsm.contact_stop_release_eps' out of range "
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
            "demo_joint_controller: 'fsm.contact_stop_lpf_cutoff_hz' out of range "
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
  // Validation only; RegisterLog calls happen in on_configure once
  // device_name_configs_ is populated and joint_names are available.
  parsed_log_entries_.clear();
  if (cfg["logs"]) {
    if (!cfg["logs"].IsSequence()) {
      throw std::runtime_error("DemoJointController: 'logs' must be a sequence");
    }
    for (const auto& entry : cfg["logs"]) {
      if (!entry.IsMap() || !entry["msg_type"]) {
        throw std::runtime_error("DemoJointController: each `logs` entry needs `msg_type`");
      }
      ParsedLogEntry e;
      e.msg_type = entry["msg_type"].as<std::string>();
      if (entry["instance"]) {
        e.instance = entry["instance"].as<std::string>();
      }
      // Closed-set msg_type validation (typo = hard fail at parse time).
      if (e.msg_type != "rtc_msgs/DeviceStateLog" && e.msg_type != "rtc_msgs/DeviceSensorLog" &&
          e.msg_type != integrated_bringup::kPullEstimatorLogMsgType) {
        throw std::runtime_error("DemoJointController: unknown msg_type in `logs`: " + e.msg_type);
      }
      parsed_log_entries_.push_back(std::move(e));
    }
  }
}

// ── E-STOP ──────────────────────────────────────────────────────────────────

void DemoJointController::TriggerEstop() noexcept {
  estopped_.store(true, std::memory_order_release);
}

void DemoJointController::ClearEstop() noexcept {
  estopped_.store(false, std::memory_order_release);
}

bool DemoJointController::IsEstopped() const noexcept {
  return estopped_.load(std::memory_order_acquire);
}

void DemoJointController::SetHandEstop(bool active) noexcept {
  hand_estopped_.store(active, std::memory_order_release);
}

void DemoJointController::PublishNonRtSnapshot(const rtc::PublishSnapshot& snap) noexcept {
  const auto grasp_loaded = grasp_state_lock_.Load();
  const auto tof_loaded = tof_snapshot_lock_.Load();
  PublishOwnedTopicsFromSnapshot(snap, owned_topics_, /*grasp=*/&grasp_loaded, /*wbc=*/nullptr,
                                 /*tof=*/&tof_loaded);
}

// on_configure / on_activate / on_deactivate / on_cleanup
//   live in demo_joint_controller_lifecycle.cpp.
// DeclareGainParameters / OnGainParametersSet
//   live in demo_joint_controller_parameters.cpp.

}  // namespace integrated_bringup
