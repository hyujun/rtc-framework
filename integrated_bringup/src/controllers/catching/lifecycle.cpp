#include "integrated_bringup/controllers/demo_catching_controller.hpp"
#include "integrated_bringup/support/controller_log_registration.hpp"
#include "integrated_bringup/support/owned_topics.hpp"
#include "rtc_controllers/catching/catch_pose_ik_batch.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter.hpp>

#include <chrono>
#include <cstddef>
#include <exception>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace integrated_bringup {

namespace {

/// Human-readable tag for one validation entry. Static strings only — this
/// runs in a lifecycle callback, but the report itself is allocation-free by
/// contract and there is no reason to make its rendering the exception.
[[nodiscard]] const char* ReasonText(rtc::catching::CatchingValidationReason reason) noexcept {
  using R = rtc::catching::CatchingValidationReason;
  switch (reason) {
    case R::kActiveConfigTbd:
      return "still TBD in the active configuration";
    case R::kControlRateOutOfRange:
      return "control_rate out of range";
    case R::kRangeViolation:
      return "value outside its documented range";
    case R::kZetaNotCriticallyDamped:
      return "reference.zeta must be exactly 1 in v1";
    case R::kEtaVOutOfRange:
      return "eta_v must be in (0, 1]";
    case R::kDecelExceedsAMax:
      return "a_dec exceeds reference.a_max";
    case R::kUnstableDiscretization:
      return "omega*h is at or above the discrete stability limit";
    case R::kDiscretizationAccuracy:
      return "omega*h exceeds the accuracy recommendation";
    case R::kHandCagingGapTooSmall:
      return "caging joint moves by less than rho_eps between q_pre and q_close";
    case R::kProvisionalOnRealArm:
      return "provisional value blocks a real-arm configuration";
    case R::kProvisionalWarning:
      return "provisional value (sim only)";
    case R::kWeightOrdering:
      return "CLIK weight ordering broken (w_task/w_a must dominate w_arm, w_arm must dominate "
             "damping_sq)";
  }
  return "unknown";
}

/// Whether a validation entry names a key this controller actually consumes.
///
/// G0-C (L0 §5.3) blocks ARMING on a TBD active key. This controller does arm
/// now, so the gate matters — but the key set it consumes at S5.1 is still
/// small, and refusing to configure over a value that a LATER step decides
/// would invert the dependency the plan's own DAG records (S4.2 → S4.4 →
/// S3.5b: the measurement chain this controller feeds is what decides
/// `reference.v_max`, `supervisor.decel.a_dec` and the planner's values).
///
/// So the gate is the consumed subset and everything else is reported as a
/// warning naming the step that owns it. The subset grows with the
/// controller: S5.2 adds `io.*` / `prediction.*`, S5.3 adds `joint_cmd.*`,
/// `reference.*` and `robot.arm.*`, S6 adds `planner.*`, S7 the rest of
/// `supervisor.*`.
///
/// `robot.hand.T_close_e2e` is excluded BY NAME and stays excluded until S7.1:
/// it is the number S4.2 produces with this controller, so gating on it would
/// make the measurement its own precondition. The hand sequencer that derives
/// `T_close_timeout` from it is its first consumer.
[[nodiscard]] bool ConsumedByCatchingSkeleton(const char* key) noexcept {
  const std::string_view k{key};
  if (k == "control_rate") {
    return true;
  }
  // S5.2's vision ingress and S5.3's tracking law. Each prefix joined this set
  // in the step that started reading it — the gate and the code that consumes
  // the value move together, or the gate stops meaning anything.
  if (k.starts_with("io.") || k.starts_with("prediction.") || k.starts_with("joint_cmd.") ||
      k.starts_with("robot.arm.") || k.starts_with("reference.") ||
      k == "supervisor.track_err_abort" || k == "supervisor.n_qp") {
    return true;
  }
  if (k == "sim.io.future_tol") {
    return true;
  }
  // `robot.hand` WITHOUT a trailing dot is the provisional flag's own key
  // (catching_params.cpp reports the profile as a whole, not a field of it).
  // Matching only the dotted prefix silently let a provisional hand profile
  // through the real-arm gate — the one thing L0 §5.3 asks this gate to stop.
  if (k == "robot.hand") {
    return true;
  }
  return k.starts_with("robot.hand.") && k != "robot.hand.T_close_e2e";
}

/// Read-only descriptor for the mirrored profile parameters. They exist so an
/// off-process reader (the S4.2 runner, the GUI panel) uses what the CONTROLLER
/// loaded instead of re-reading the YAML — a run whose controller read
/// something else must not be analysed against the file on disk.
[[nodiscard]] rcl_interfaces::msg::ParameterDescriptor ReadOnlyDescriptor(std::string description) {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = std::move(description);
  d.read_only = true;
  return d;
}

}  // namespace

void DemoCatchingController::DeclareProfileParameters() {
  if (!node_) {
    return;
  }
  const auto& hand = params_.hand;
  const auto dof = static_cast<std::size_t>(hand.dof);

  std::vector<double> q_open(hand.q_open.begin(), hand.q_open.begin() + dof);
  std::vector<double> q_pre(hand.q_pre.begin(), hand.q_pre.begin() + dof);
  std::vector<double> q_close(hand.q_close.begin(), hand.q_close.begin() + dof);
  std::vector<bool> caging(hand.caging_mask.begin(), hand.caging_mask.begin() + dof);

  // Guarded, like every sibling binding's parameters.cpp. These are read_only,
  // so on_cleanup CANNOT undeclare them — a configure → cleanup → configure on
  // the same LifecycleNode would otherwise throw ParameterAlreadyDeclaredException
  // and turn a legal re-configure into a configure failure.
  const auto declare = [this](const std::string& name, const auto& value, std::string description) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, value, ReadOnlyDescriptor(std::move(description)));
    }
  };

  declare("hand.q_open", q_open, "L6 §5.1 open pose [rad]");
  declare("hand.q_pre", q_pre, "L6 §5.1 preshape pose [rad]");
  declare("hand.q_close", q_close, "L6 §5.1 closed pose [rad]");
  declare("hand.caging_mask", caging, "L6 §4.2 caging joint set C");
  declare("hand.eta_close", hand.eta_close.value, "L6 §4.2 closure threshold eta");
  declare("hand.rho_eps", hand.rho_eps, "L6 §4.2 caging gap floor [rad]");
  declare("hand.T_close_e2e", hand.T_close_e2e.value, "L6 §4.2 end-to-end closure time [s]");
  // The RT tick period the analyser needs for its tick axis and its dropped-row
  // test. It is mirrored here for the same reason as the poses: the off-process
  // reader must use what the CONTROLLER resolved, not a constant. The runner
  // cannot read `control_rate` itself — that parameter lives on the CM's node,
  // not on this per-controller one — and a hard-coded 0.002 silently scales the
  // tick axis and widens the drop gate on any bring-up that is not 500 Hz.
  declare("control.dt", GetDefaultDt(), "RT tick period [s] = 1/control_rate");
  declare("diagnostic.hand_step", hand_step_enabled_, "accept unshaped hand step targets (S4a)");
}

void DemoCatchingController::DeclareArmParameter() {
  if (!node_) {
    return;
  }
  // Read-WRITE, unlike the mirrored profile parameters: this one is an input.
  // Declared with `false` so a bring-up never starts armed.
  if (!node_->has_parameter(kCatchingEnableParam)) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description =
        "Arm the catching supervisor (A-S5-3). The controller lowers this itself on E-STOP and on "
        "a latched fault — P-1 (c), no automatic resume.";
    node_->declare_parameter(kCatchingEnableParam, false, d);
  }
  // Mirror whatever the declaration left behind (a parameter override on the
  // command line can make that `true`), so the tick and the parameter agree
  // from the first tick rather than from the first CHANGE.
  arm_requested_.store(node_->get_parameter(kCatchingEnableParam).as_bool(),
                       std::memory_order_release);

  if (arm_param_cb_handle_) {
    return;  // a re-configure keeps the one callback; registering twice would double-handle
  }
  arm_param_cb_handle_ =
      node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
          if (p.get_name() != kCatchingEnableParam) {
            continue;
          }
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
            result.successful = false;
            result.reason = "catching.enable must be a bool";
            continue;
          }
          // The ONLY thing this callback does. Not a reset, not a mode change,
          // not a command — the tick owns all three (P-1 (a)). It runs on the
          // controller's non-RT callback group.
          arm_requested_.store(p.as_bool(), std::memory_order_release);
        }
        return result;
      });
}

bool DemoCatchingController::LoadDerivedAccelLimits() {
  arm_qdd_max_.clear();
  if (accel_limits_path_.empty()) {
    RCLCPP_WARN(logger_,
                "no `robot.arm.accel_limits_path`: the CLIK acceleration box is OFF, so a command "
                "may step by more than the joint can follow (D-16)");
    return false;
  }
  // Package-relative, like every other file this repo's YAML points at
  // (`DeviceUrdfConfig`): the controller has no way to learn which config
  // variant it was loaded from, and a path relative to the process's cwd would
  // depend on where the operator started the bring-up.
  std::string path;
  try {
    path = ament_index_cpp::get_package_share_directory(accel_limits_package_) + "/" +
           accel_limits_path_;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "package '%s' not found for the derived acceleration limits: %s",
                 accel_limits_package_.c_str(), e.what());
    return false;
  }
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "could not read the derived acceleration limits at '%s': %s",
                 path.c_str(), e.what());
    return false;
  }
  const YAML::Node root = doc["derived_accel_limits"];
  const YAML::Node group = root ? root[accel_limits_group_] : YAML::Node();
  if (!group || !group.IsMap()) {
    RCLCPP_ERROR(logger_, "'%s' has no derived_accel_limits.%s", path.c_str(),
                 accel_limits_group_.c_str());
    return false;
  }
  // `adopted: false` marks a derivation the tool ran but nobody accepted (D-16
  // records the review, not just the number). Falling back from it would put
  // an unreviewed limit on the arm, which is the one thing a DERIVED box must
  // not allow — so it is refused rather than defaulted.
  if (!group["adopted"] || !group["adopted"].as<bool>()) {
    RCLCPP_ERROR(logger_, "derived_accel_limits.%s is not `adopted` — refusing to use it",
                 accel_limits_group_.c_str());
    return false;
  }
  const YAML::Node box = group["qdd_max"];
  if (!box || !box.IsSequence() || static_cast<int>(box.size()) != arm_dof_) {
    RCLCPP_ERROR(logger_, "derived_accel_limits.%s.qdd_max must have %d entries (the arm's)",
                 accel_limits_group_.c_str(), arm_dof_);
    return false;
  }
  arm_qdd_max_.assign(box.size(), 0.0);
  for (std::size_t i = 0; i < box.size(); ++i) {
    arm_qdd_max_[i] = box[i].as<double>();
    if (!std::isfinite(arm_qdd_max_[i]) || arm_qdd_max_[i] <= 0.0) {
      RCLCPP_ERROR(logger_, "derived_accel_limits.%s.qdd_max[%zu] is not a positive number",
                   accel_limits_group_.c_str(), i);
      arm_qdd_max_.clear();
      return false;
    }
  }
  RCLCPP_INFO(logger_, "derived acceleration box: %s (group '%s', %zu joints)", path.c_str(),
              accel_limits_group_.c_str(), arm_qdd_max_.size());
  return true;
}

void DemoCatchingController::BuildClikBoxes(int nv,
                                            rtc::tsid::ClikReferenceGenerator::Config& cfg) {
  qdd_max_pin_.resize(0);
  const auto& map = combined_cache_.ext_to_pin_v_map();
  const auto place = [&map, nv](int ext_idx, Eigen::VectorXd& dst, double value) {
    const int pv = map[static_cast<std::size_t>(ext_idx)];
    if (pv >= 0 && pv < nv) {
      dst[pv] = value;
    }
  };

  // ── Position box ────────────────────────────────────────────────────────
  // The device's own limits, pulled `limit_margin` inwards (L5 §4.3): the
  // backend clamps to the device limits, so a CLIK box that reached them would
  // let the solver return a value the backend then changes — and the
  // difference would show up as a tracking error with no attributable cause.
  //
  // The box is all-or-nothing by CLIK's contract, so ONE joint without a
  // finite limit drops it for every joint. That is the honest outcome: a
  // partial box would silently leave that joint unbounded while the operator
  // reads "position limits on".
  Eigen::VectorXd q_min = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd q_max = Eigen::VectorXd::Zero(nv);
  bool box_complete = true;
  for (int dev = 0; dev < 2 && box_complete; ++dev) {
    const int dof = (dev == kCatchingArmDeviceIdx) ? arm_dof_ : hand_dof_;
    const int base = (dev == kCatchingArmDeviceIdx) ? 0 : arm_dof_;
    const auto& lower = device_position_lower_[static_cast<std::size_t>(dev)];
    const auto& upper = device_position_upper_[static_cast<std::size_t>(dev)];
    if (static_cast<int>(lower.size()) < dof || static_cast<int>(upper.size()) < dof) {
      box_complete = false;
      break;
    }
    for (int i = 0; i < dof; ++i) {
      const auto ui = static_cast<std::size_t>(i);
      if (!std::isfinite(lower[ui]) || !std::isfinite(upper[ui]) || !(lower[ui] <= upper[ui])) {
        box_complete = false;
        break;
      }
      // The margin narrows from BOTH sides but never inverts the box: a joint
      // whose whole range is narrower than twice the margin keeps its midpoint
      // rather than becoming an empty interval that Init would throw on.
      const double mid = 0.5 * (lower[ui] + upper[ui]);
      const double lo = std::min(lower[ui] + limit_margin_, mid);
      const double hi = std::max(upper[ui] - limit_margin_, mid);
      place(base + i, q_min, lo);
      place(base + i, q_max, hi);
    }
  }
  if (box_complete) {
    cfg.q_min = q_min;
    cfg.q_max = q_max;
  } else {
    RCLCPP_WARN(logger_,
                "device position limits incomplete — the CLIK position box is OFF (the backend "
                "clamp is then the only bound, and its clamp is invisible to the solver)");
  }

  // ── Per-joint velocity box ──────────────────────────────────────────────
  Eigen::VectorXd v_limit = Eigen::VectorXd::Zero(nv);
  bool v_complete = true;
  for (int dev = 0; dev < 2 && v_complete; ++dev) {
    const int dof = (dev == kCatchingArmDeviceIdx) ? arm_dof_ : hand_dof_;
    const int base = (dev == kCatchingArmDeviceIdx) ? 0 : arm_dof_;
    const auto& vmax = device_max_velocity_[static_cast<std::size_t>(dev)];
    if (static_cast<int>(vmax.size()) < dof) {
      v_complete = false;
      break;
    }
    for (int i = 0; i < dof; ++i) {
      const double value = vmax[static_cast<std::size_t>(i)];
      if (!std::isfinite(value) || value <= 0.0) {
        v_complete = false;
        break;
      }
      // THE HAND IS LOCKED IN THIS SOLVE. Its joints are decision variables —
      // the catch frame hangs off the palm, so they appear in the frame's
      // Jacobian — but this controller does not command them: the sequencer
      // does (L6, D-11). A solver allowed to use them would satisfy part of
      // the task with motion that never happens, and the arm would then
      // under-deliver by exactly that part. It shows up as a steady-state
      // position error that no gain fixes (measured 2.7 mm before this lock,
      // against the 1 mm G5-A asks for).
      //
      // Locked through the velocity box rather than by dropping the columns:
      // the box is the one place a joint can be told "you do not move" without
      // changing the problem's shape, and the hand's REAL motion still reaches
      // the solve every tick through the measured configuration.
      const bool is_hand = (dev != kCatchingArmDeviceIdx);
      place(base + i, v_limit, is_hand ? kLockedJointVelocity : value);
    }
  }
  if (v_complete) {
    cfg.v_limit_per_joint = v_limit;
  }

  // ── Acceleration box (D-16) ─────────────────────────────────────────────
  // Only the ARM has a derived box — it is what the torque limits produced.
  // CLIK needs the whole [nv] or nothing, so the hand entries are derived
  // rather than invented: a joint allowed to reach its own velocity limit
  // within one tick is unconstrained WITHIN the velocity box, which is exactly
  // the statement "this box says nothing about the hand". The hand's command
  // comes from the sequencer (L6), not from this solve.
  if (!arm_qdd_max_.empty() && static_cast<int>(arm_qdd_max_.size()) == arm_dof_ && v_complete) {
    Eigen::VectorXd a_max = Eigen::VectorXd::Zero(nv);
    for (int i = 0; i < arm_dof_; ++i) {
      place(i, a_max, arm_qdd_max_[static_cast<std::size_t>(i)]);
    }
    const double dt = GetDefaultDt();
    for (int i = 0; i < hand_dof_; ++i) {
      const int pv = map[static_cast<std::size_t>(arm_dof_ + i)];
      if (pv >= 0 && pv < nv && dt > 0.0) {
        // The hand is velocity-locked above, so its acceleration bound only
        // has to be wide enough not to conflict with that lock. Derived from
        // the lock rather than invented: reaching the locked velocity within
        // one tick is what "unconstrained inside the box" means.
        a_max[pv] = v_limit[pv] / dt;
      }
    }
    bool all_positive = true;
    for (int i = 0; i < nv; ++i) {
      if (!(a_max[i] > 0.0) || !std::isfinite(a_max[i])) {
        all_positive = false;
        break;
      }
    }
    if (all_positive) {
      cfg.a_max = a_max;
      qdd_max_pin_ = a_max;
    }
  }
}

void DemoCatchingController::SetupArmCommand() {
  // Resolved HERE, not in the ingress setup: `limit_margin_` is read by the
  // box builder a few lines down, and a value set in another function is a
  // value whose ordering has to be remembered rather than seen.
  track_err_abort_rad_ =
      params_.supervisor_track_err_abort.tbd ? 0.0 : params_.supervisor_track_err_abort.value;
  n_qp_fault_ = params_.supervisor_n_qp;
  limit_margin_ = params_.robot_arm_limit_margin.tbd ? 0.05 : params_.robot_arm_limit_margin.value;

  clik_enabled_ = false;
  catch_frame_idx_ = -1;
  base_frame_idx_ = -1;

  const auto* sys_cfg = GetSystemModelConfig();
  if (sys_cfg == nullptr || sys_cfg->urdf_path.empty()) {
    RCLCPP_WARN(logger_, "no system model config: the arm will be held, not driven");
    return;
  }
  // Prefer the builder CM injected so the URDF is parsed once for the whole
  // bring-up; build our own only when running outside CM (fixtures).
  if (auto shared = GetSharedModelBuilder()) {
    builder_ = std::move(shared);
  } else {
    try {
      builder_ = std::make_shared<rtc_urdf_bridge::PinocchioModelBuilder>(*sys_cfg);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "model build failed: %s", e.what());
      return;
    }
  }
  if (!combined_cache_.InitModel(*builder_, /*contact_frame_ids=*/{}, "[catching]", logger_)) {
    RCLCPP_ERROR(logger_, "combined model/cache init failed");
    return;
  }
  full_dof_ = arm_dof_ + hand_dof_;
  const auto* arm_cfg = GetDeviceNameConfig(GetPrimaryDeviceName());
  const auto* hand_cfg = GetDeviceNameConfig(GetSecondaryDeviceName());
  combined_cache_.BuildReorderMap(arm_cfg != nullptr ? &arm_cfg->joint_state_names : nullptr,
                                  hand_cfg != nullptr ? &hand_cfg->joint_state_names : nullptr,
                                  full_dof_, "[catching]", logger_);
  if (!combined_cache_.reorder_valid() || !combined_cache_.model()) {
    RCLCPP_ERROR(logger_, "joint reorder map is not valid — the arm will be held");
    return;
  }
  const auto& model = *combined_cache_.model();
  // CLIK integrates q with v, so the two must have the same dimension. A model
  // with a floating or continuous joint does not, and the failure would show
  // up as a command that drifts rather than as an error.
  if (model.nq != model.nv) {
    RCLCPP_ERROR(logger_, "model nq (%d) != nv (%d) — CLIK needs a reduced model", model.nq,
                 model.nv);
    return;
  }

  // The catch frame is an `urdf.extra_frames` entry (D-10/D-17) and is the
  // whole target of this controller: without it there is nothing to align.
  try {
    const auto frame_id = rtc::catching::ResolveCatchFrame(model, catch_frame_name_);
    catch_frame_idx_ = combined_cache_.cache().RegisterFrame(catch_frame_name_, frame_id);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "catch frame '%s' not in the model: %s — the arm will be held",
                 catch_frame_name_.c_str(), e.what());
    return;
  }
  if (catch_frame_idx_ < 0) {
    RCLCPP_ERROR(logger_, "catch frame registration refused (cache already locked?)");
    return;
  }

  rtc::tsid::ClikReferenceGenerator::Config cfg;
  BuildArmHandVelocityIndexSets(arm_dof_, full_dof_, model.nv, combined_cache_.ext_to_pin_v_map(),
                                cfg.arm_v_idx, cfg.hand_v_idx);
  if (cfg.arm_v_idx.empty()) {
    RCLCPP_ERROR(logger_, "no arm velocity indices resolved — the arm will be held");
    return;
  }
  cfg.damping_sq = params_.joint_cmd_damping_sq.value;
  cfg.w_task = params_.joint_cmd_w_task.value;
  cfg.w_axis = params_.joint_cmd_w_axis.value;
  cfg.w_arm = params_.joint_cmd_w_arm.value;
  cfg.w_hand = params_.joint_cmd_w_arm.value;
  cfg.w_smooth = params_.joint_cmd_w_smooth.value;
  cfg.max_iter = params_.joint_cmd_max_iter;
  // D-6: evaluate along the COMMANDED path. The measured state would fold the
  // servo lag into the loop and double-count it against the lead compensation
  // (L5 §4.2); `anchor_drift_max` must then stay off, and Init throws if it
  // does not — the tracking watchdog (TRACK_ERR) is what supervises the gap.
  cfg.evaluate_at_command = true;
  cfg.anchor_drift_max = 0.0;

  const int nv = model.nv;
  static_cast<void>(LoadDerivedAccelLimits());
  BuildClikBoxes(nv, cfg);
  try {
    clik_.Init(nv, cfg);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "CLIK init failed: %s — the arm will be held", e.what());
    return;
  }
  Eigen::Matrix<double, 6, 1> kx;
  const double k_p = params_.joint_cmd_k_p.value;
  kx << k_p, k_p, k_p, 0.0, 0.0, 0.0;  // rotation rows are the axis task's, not this one's
  clik_.SetTaskGain(kx);
  clik_.SetAxisGain(params_.joint_cmd_k_axis.value);
  clik_.SetPostureGains(params_.joint_cmd_k_posture.value, params_.joint_cmd_k_posture.value);

  q_posture_ = Eigen::VectorXd::Zero(model.nq);
  q_eval_ = Eigen::VectorXd::Zero(model.nq);
  v_eval_ = Eigen::VectorXd::Zero(nv);

  rtc::catching::SoftCatchTranslation::Params ref_params;
  ref_params.omega = params_.reference_omega.value;
  ref_params.zeta = params_.reference_zeta.value;
  ref_params.a_max = params_.reference_a_max.tbd ? ref_params.a_max : params_.reference_a_max.value;
  ref_params.v_max = params_.reference_v_max.tbd ? ref_params.v_max : params_.reference_v_max.value;
  reference_.emplace(ref_params);
  if (!reference_->ParamsValid()) {
    RCLCPP_ERROR(logger_, "reference parameters rejected (omega/zeta/a_max/v_max)");
    reference_.reset();
    return;
  }

  clik_enabled_ = true;
  RCLCPP_INFO(logger_, "arm command path ready: nv=%d, catch frame '%s' (idx %d), accel box %s", nv,
              catch_frame_name_.c_str(), catch_frame_idx_,
              qdd_max_pin_.size() > 0 ? "from the derived file" : "OFF");
}

void DemoCatchingController::SetupTrajInput() {
  // Resolve the ingress configuration from the parsed params. Every number
  // comes from the same `catching:` tree the validator judged, so a value that
  // reaches the wire decode is one the report has already had an opinion on.
  const auto to_ns = [](double seconds) { return static_cast<std::int64_t>(seconds * 1e9); };
  TrajInputConfig cfg;
  cfg.n_min = params_.io_n_min > 0 ? params_.io_n_min : 2;
  // n_max is the RUNTIME bound S3.6 set (20 for the shipped profile), not the
  // snapshot capacity: a message longer than the profile is a profile change
  // nobody asked for, and accepting it silently would hide that.
  cfg.n_max = rtc::catching::kCap;
  if (!params_.prediction_dt_expected.tbd && params_.prediction_dt_expected.value > 0.0) {
    // The spacing floor is derived, not configured: a fraction of the expected
    // interval. Anything at or below this is not a denser prediction, it is a
    // malformed one, and the sampler's Hermite basis divides by h².
    cfg.dt_min_ns = std::max<std::int64_t>(
        1, to_ns(params_.prediction_dt_expected.value * kTrajSpacingFloorFraction));
  }
  const auto future_tol = rtc::catching::EffectiveFutureTol(params_, real_arm_config_);
  cfg.future_tol_ns = future_tol.tbd ? 0 : to_ns(future_tol.value);
  cfg.horizon_min_ns = params_.io_horizon_min.tbd ? 0 : to_ns(params_.io_horizon_min.value);
  cfg.track_eval_offset_ns =
      params_.io_track_eval_offset.tbd ? 0 : to_ns(params_.io_track_eval_offset.value);
  cfg.j_warn_m = params_.io_track_j_warn.tbd ? -1.0 : params_.io_track_j_warn.value;
  cfg.expected_frame = expected_frame_;
  traj_input_.Configure(cfg);

  traj_horizon_min_ns_ = cfg.horizon_min_ns;
  t_stale_ns_ = params_.io_t_stale.tbd ? 0 : to_ns(params_.io_t_stale.value);
  // The lead axis (L5 §4.5). OFF unless the profile says otherwise, because
  // the sim has no actuation lag to lead (2026-09-20) and leading a delay that
  // does not exist moves the command EARLY by exactly T_arm. `lead_enable` is
  // the switch the identification (S10) turns on once T_arm is measured; the
  // fixture that exercises the compensation supplies its own delay.
  t_arm_ns_ = 0;
  if (params_.joint_cmd_lag_lead_enable && !params_.joint_cmd_lag_t_arm.tbd) {
    t_arm_ns_ = static_cast<std::int64_t>(params_.joint_cmd_lag_t_arm.value * 1e9);
  }

  if (!node_ || traj_topic_.empty()) {
    return;
  }
  // best_effort KEEP_LAST(1), measured rather than assumed: S3.4 compared a
  // best_effort subscription against a reliable one over 856 messages on two
  // robots and found them identical, including under 50 ms delay and 30 %
  // drop injection. Depth 1 is not a choice — ARCH-6 fixes it, and it is also
  // what G1-I asserts: under a backlog the controller must take the NEWEST
  // prediction, not work through a queue of old ones.
  rclcpp::QoS qos{rclcpp::KeepLast(1)};
  qos.best_effort();
  traj_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      traj_topic_, qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { OnTrajectoryCloud(*msg); });
  RCLCPP_INFO(logger_, "vision prediction lane: '%s' (best_effort, depth 1)", traj_topic_.c_str());
}

RTControllerInterface::CallbackReturn DemoCatchingController::on_configure(
    const rclcpp_lifecycle::State& prev, rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const YAML::Node& yaml) noexcept {
  const auto ret = RTControllerInterface::on_configure(prev, node, yaml);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  try {
    // CM calls SetDeviceNameConfigs between PreConfigure and here, so the hook
    // has already run with the groups resolved. A fixture that calls
    // on_configure directly sets the configs FIRST, so the hook then saw no
    // groups — re-run rather than trust the ordering (idempotent).
    ResolveDevices();
    // Owned by this function alone, and re-decided on every configure: a
    // reconfigure after the backends changed must not inherit the old verdict.
    sim_only_disabled_ = false;

    if (topic_config_.groups.size() < 2) {
      RCLCPP_ERROR(logger_,
                   "refusing to configure: this controller claims an arm group and a hand group, "
                   "but `topics:` declares %zu",
                   topic_config_.groups.size());
      return CallbackReturn::FAILURE;
    }

    // ── Which axis this configure is judged on (A-S5-1) ──────────────────
    // S4.0 refused to ACTIVATE here unless every claimed device was the
    // simulator, standing in for the then-pending E-8 decision. That decision
    // was made on 2026-09-22, so the stand-in is gone and what is left is the
    // rule it was standing in for: a provisional or still-TBD value blocks a
    // REAL-ARM configuration and only warns in sim (L0 §5.3, L8 §5.4).
    //
    // ResolveDevices has already set `real_arm_config_` from the backends —
    // fail-closed, so a group whose config has not resolved yet counts as
    // unproven and gets the strict rule.
    if (real_arm_config_) {
      for (const auto& group : non_sim_groups_) {
        const auto* cfg = GetDeviceNameConfig(group);
        RCLCPP_INFO(logger_,
                    "device group '%s' is bound to backend '%s' (not '%s') — this configure is "
                    "judged as a REAL-ARM configuration: provisional and TBD values block it.",
                    group.c_str(),
                    (cfg != nullptr && cfg->backend.has_value()) ? cfg->backend->type.c_str()
                                                                 : "<none declared>",
                    kCatchingSimBackendType);
      }
      if (!device_configs_seen_) {
        RCLCPP_INFO(logger_,
                    "no device config resolved for any declared group — judged as a REAL-ARM "
                    "configuration (silence does not prove the simulator).");
      }
    }

    // ── Hand profile (G0-C, L0 §5.3) ─────────────────────────────────────
    if (!catching_section_present_) {
      RCLCPP_ERROR(logger_,
                   "refusing to configure: no `catching:` section. The hand profile has no "
                   "defensible default — see L6 §6 and plan §4.4 S4.1.");
      return CallbackReturn::FAILURE;
    }
    report_ =
        rtc::catching::ValidateCatchingParams(params_, 1.0 / GetDefaultDt(), real_arm_config_);
    for (std::size_t i = 0; i < report_.warning_count; ++i) {
      const auto& w = report_.warnings[i];
      RCLCPP_WARN(logger_, "catching config warning: %s — %s", w.key, ReasonText(w.reason));
    }
    bool consumed_failure = false;
    for (std::size_t i = 0; i < report_.failure_count; ++i) {
      const auto& f = report_.failures[i];
      if (!ConsumedByCatchingSkeleton(f.key)) {
        RCLCPP_WARN(logger_,
                    "catching config: %s — %s. Not consumed at this step; the step that owns "
                    "the value decides it.",
                    f.key, ReasonText(f.reason));
        continue;
      }
      consumed_failure = true;
      if (f.index >= 0) {
        RCLCPP_ERROR(logger_, "catching config error: %s[%d] — %s", f.key, f.index,
                     ReasonText(f.reason));
      } else {
        RCLCPP_ERROR(logger_, "catching config error: %s — %s", f.key, ReasonText(f.reason));
      }
    }
    if (consumed_failure) {
      // A real-arm configuration is PARKED, not refused (A-S5-1). The failure
      // there is "this profile is not cleared for hardware", and refusing the
      // configure would take the whole robot down with it — sim and real share
      // `config/<variant>/controllers/`, and CM latches `bring_up_failed` on
      // ANY controller's configure failure and then refuses to configure EVERY
      // controller. Nothing is commanded until a controller is active, so an
      // instance that can never activate can never command.
      //
      // In sim the same failures ARE a refusal: there is no other bring-up
      // riding on this instance, and a profile that cannot arm in sim is a
      // configuration mistake to be fixed now rather than discovered as a
      // controller that silently will not start.
      if (real_arm_config_) {
        sim_only_disabled_ = true;
        RCLCPP_ERROR(logger_,
                     "DISABLED: real-arm configuration with values this controller consumes that "
                     "are provisional or still TBD (L0 §5.3). It will refuse to activate; nothing "
                     "was commanded.");
        return CallbackReturn::SUCCESS;
      }
      return CallbackReturn::FAILURE;
    }
    // One bool for the tick's arming question (§4.5-1).
    //
    // NOT `report_.armable`: that verdict covers every key the schema knows,
    // including the ones a LATER step decides (`reference.v_max`,
    // `planner.*`, `supervisor.decel.*`). Arming on it would mean this
    // controller can never arm until S7 is finished, which inverts the plan's
    // own DAG — the measurements that decide those values are taken WITH this
    // controller. The gate is the same consumed subset the configure refusal
    // uses, so the two cannot drift apart: what refuses a sim configure is
    // exactly what refuses to arm.
    armable_ = !consumed_failure;

    if (params_.hand.dof != hand_dof_) {
      RCLCPP_ERROR(logger_,
                   "refusing to configure: hand profile declares %d joints but the hand device "
                   "reports %d (L6 §5.1: n must equal the hand device's channel count)",
                   params_.hand.dof, hand_dof_);
      return CallbackReturn::FAILURE;
    }

    // ── Controller-owned CSVs ────────────────────────────────────────────
    // Instance keys are derived from the device names so a YAML `instance:`
    // tracks the active config_variant, exactly as the other bindings do.
    const auto arm_key = GetPrimaryDeviceName() + "_state";
    const auto hand_key = GetSecondaryDeviceName() + "_state";
    LogRegistrationContext ctx{
        .logger = logger_,
        .log_set = log_set_,
        .state_logs =
            {
                {arm_key, {arm_joint_names_, std::vector<std::string>{}}},
                {hand_key, {hand_joint_names_, hand_motor_names_}},
            },
    };
    auto reg = RegisterControllerLogs(parsed_log_entries_, ctx);
    if (reg.status == LogRegistrationStatus::kMissingInstance) {
      ResetLogState();
      return CallbackReturn::FAILURE;
    }
    if (auto it = reg.handles.state.find(arm_key); it != reg.handles.state.end()) {
      arm_state_log_handle_ = it->second;
    }
    if (auto it = reg.handles.state.find(hand_key); it != reg.handles.state.end()) {
      hand_state_log_handle_ = it->second;
    }

    // Drain timer on a non-RT callback group (10 Hz), same as every other
    // binding: the RT tick only pushes into the SPSC ring.
    if (!log_set_.empty() && node_) {
      log_drain_cb_group_ =
          node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      log_drain_timer_ = node_->create_wall_timer(
          std::chrono::milliseconds(100),
          [this]() { DrainControllerLogs(log_set_, logger_, log_drops_reported_); },
          log_drain_cb_group_);
    }

    // ── Controller-owned topics ──────────────────────────────────────────
    // Creates the hand group's `joint_goal` subscription declared in the
    // `topics:` block. Last, and only past every refusal above, so a
    // controller that is going to fail configure never exposes a step lane.
    // Without this the YAML entry is inert: the topic name parses, the group
    // shows up in topic_config_, and no endpoint is ever created — the step
    // then vanishes with no error anywhere (the step path is what S4.2
    // measures through, so it fails as silence, not as a failure).
    CreateOwnedTopics(*this, owned_topics_);
    SetupTrajInput();
    // After the topics and before the parameters: the arm path needs the
    // device configs (resolved above) and it must not be half-built if a
    // later step throws — the catch block below tears everything down.
    SetupArmCommand();

    DeclareProfileParameters();
    DeclareArmParameter();

    RCLCPP_INFO(logger_,
                "configured: arm=%s(%d) hand=%s(%d), hand_step=%s, config=%s, armable=%s "
                "(arm it with the '%s' parameter)",
                GetPrimaryDeviceName().c_str(), arm_dof_, GetSecondaryDeviceName().c_str(),
                hand_dof_, hand_step_enabled_ ? "enabled" : "disabled",
                real_arm_config_ ? "real-arm" : "sim", armable_ ? "yes" : "no",
                kCatchingEnableParam);
  } catch (const std::exception& e) {
    // Undo everything this function may already have built. Without this a
    // throw past the log registration leaves a half-configured instance —
    // live log channels, a live 100 ms timer capturing `this`, and a live step
    // subscription — behind a FAILURE that the operator reads as "nothing
    // happened". CM then refuses the bring-up and destroys the node anyway,
    // but a fixture that retries configure would inherit the wreckage.
    RCLCPP_ERROR(logger_, "on_configure failed: %s", e.what());
    TearDownConfiguredResources();
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

RTControllerInterface::CallbackReturn DemoCatchingController::on_activate(
    const rclcpp_lifecycle::State& prev) noexcept {
  // The park check lives here, not in on_configure: refusing to configure took
  // the whole robot down with it (see the header). Nothing is commanded until
  // a controller is active, so refusing here is what the check actually needs
  // to do.
  if (sim_only_disabled_) {
    RCLCPP_ERROR(logger_,
                 "refusing to activate: this instance was parked at configure because it is a "
                 "real-arm configuration carrying provisional or TBD values this controller "
                 "consumes (L0 §5.3). Nothing was commanded.");
    return CallbackReturn::FAILURE;
  }
  // Base first, always: it bumps the activation generation and calls
  // ResetTargetInitialization, and the generation bump is what the first tick
  // reads to know an activation boundary was crossed (D-23, #196 §3).
  const auto ret = RTControllerInterface::on_activate(prev);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }
  // An activation does NOT arm the controller (P-1 (c) in spirit, A-S5-3): the
  // operator arms it through `catching.enable`, and a controller that armed
  // itself on activation would resume catching after any deactivate/activate
  // cycle — including the one an E-STOP recovery goes through.
  arm_requested_.store(false, std::memory_order_release);
  // The accepted-sequence memory and the previous prediction belong to the
  // previous activation. Keeping them would refuse the first message of this
  // one (its sequence is below the old high-water mark after a vision
  // restart) and would compare this activation's first prediction against a
  // trajectory the operator has no reason to think is still relevant.
  traj_input_.Reset();
  if (node_ && node_->has_parameter(kCatchingEnableParam)) {
    // Bring the parameter back in line with the latch, so the value an
    // operator reads is the value the tick will act on. Without this the
    // parameter would still read `true` from a previous run while the tick is
    // disarmed — and the next thing anyone does is set it to true, which is a
    // no-op change that fires no callback.
    //
    // Guarded because this override is `noexcept` and `set_parameter` throws:
    // any OTHER on-set callback registered on this node may reject the change,
    // and an immutable-parameter or not-declared exception here would be
    // std::terminate rather than a failed activation. The latch above is what
    // actually governs the tick, so a parameter that could not be written back
    // is a display inconsistency to report, not a reason to take the process
    // down. (Lifecycle callback — non-RT, so RT-2's try/catch ban
    // does not apply here.)
    try {
      node_->set_parameter(rclcpp::Parameter(kCatchingEnableParam, false));
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_,
                  "could not write '%s' back to false on activation (%s) — the controller is "
                  "DISARMED regardless; the parameter's displayed value may disagree until it is "
                  "set again",
                  kCatchingEnableParam, e.what());
    }
  }
  return CallbackReturn::SUCCESS;
}

void DemoCatchingController::ResetLogState() noexcept {
  log_set_.Reset();
  // Reset() destroys every channel's drop counter, so the high-water mark has
  // to go with it or the next session's first drop burst is swallowed (#238).
  log_drops_reported_ = 0;
  arm_state_log_handle_ = {};
  hand_state_log_handle_ = {};
}

void DemoCatchingController::TearDownConfiguredResources() noexcept {
  log_set_.DrainAll();
  // Tear the timer down BEFORE Reset() so no drain callback runs against
  // channels that are being destroyed.
  log_drain_timer_.reset();
  log_drain_cb_group_.reset();
  ResetLogState();
  ResetOwnedTopics(owned_topics_);
  // The subscription captures `this`; dropping it here is what keeps a
  // callback from arriving against a half-torn-down controller.
  traj_sub_.reset();
  traj_input_.Reset();
}

RTControllerInterface::CallbackReturn DemoCatchingController::on_cleanup(
    const rclcpp_lifecycle::State& prev) noexcept {
  TearDownConfiguredResources();
  // The next configure re-decides this from the backends it is given.
  sim_only_disabled_ = false;
  return RTControllerInterface::on_cleanup(prev);
}

}  // namespace integrated_bringup
