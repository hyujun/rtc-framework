#include "rtc_controllers/params/task_impedance_params.hpp"

#include "rtc_controllers/compliance/external_wrench.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/gain_floor.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace rtc::params {

void ParseTaskImpedanceParams(const YAML::Node& cfg, TaskImpedanceParams& out,
                              TaskSelection selection, TaskImpedanceConfig& config) {
  if (!cfg) {
    // Even without YAML this is a CONFIGURE, so it runs the same steps the YAML
    // path does — floors, §5.3's reject, §6.4, §6.1 — on the gains it has.
    // NUM-6's loader half is "regardless of whether the key is present", and an
    // absent NODE is the widest case of that: these gains come from the
    // constructor or a previous set_gains(), i.e. exactly the two paths the
    // floor exists for. The floor is written back into `out` and not just
    // evaluated for the guard — otherwise get_gains() would keep reporting a
    // gain Compute() refuses to run, on the one path where nothing else rewrites
    // the POD. The caller commits `out` even when a guard below throws; see
    // LoadConfig.
    out.nullspace_kp = FloorNonNegativeGain(out.nullspace_kp);
    out.nullspace_kd = FloorNonNegativeGain(out.nullspace_kd);
    out.joint_limit_kp = FloorNonNegativeGain(out.joint_limit_kp);
    out.joint_limit_kd = FloorNonNegativeGain(out.joint_limit_kd);
    if (!IsFiniteNonNegative(out.joint_limit_margin)) {
      throw std::runtime_error("task_impedance: joint_limit_margin must be >= 0 and finite");
    }
    if (out.nullspace_kp > 0.0) {
      out.nullspace_kd = std::max(out.nullspace_kd, 2.0 * std::sqrt(out.nullspace_kp));
    }
    if (selection == TaskSelection::kTranslationOnly && !(out.nullspace_kp > 0.0)) {
      throw std::runtime_error(
          "task_impedance: TRANSLATION_ONLY requires nullspace_stiffness > 0 (§6.1) — "
          "orientation is otherwise uncontrolled");
    }
    return;
  }

  // Shape-checked, never silently ignored (#302). `kp_pos` is the sharpest case
  // in the repo: `inner.kp_pos: [1, 2, 3, 4]` throws in cascaded_compliance —
  // pinned by test_params_schema — while the SAME key spelled at this schema's
  // top level parsed clean and kept the default. Reading the five schemas side
  // by side in one directory is what made that visible (#298 S7c-2); the silent
  // form was transcribed from the adapter under a functional-equivalence brief,
  // so converging it is a separate change. No scalar broadcast (D5).
  auto load3 = [](const YAML::Node& n, std::array<double, 3>& arr, const char* what) {
    if (!n) {
      return;
    }
    if (!n.IsSequence() || n.size() != 3) {
      throw std::runtime_error(std::string("task_impedance: ") + what +
                               " must be a 3-entry sequence [x,y,z]");
    }
    for (std::size_t i = 0; i < 3; ++i) {
      arr[i] = n[i].as<double>();
    }
  };
  load3(cfg["kp_pos"], out.impedance.kp_pos, "kp_pos");
  load3(cfg["kd_pos"], out.impedance.kd_pos, "kd_pos");
  load3(cfg["kp_rot"], out.impedance.kp_rot, "kp_rot");
  load3(cfg["kd_rot"], out.impedance.kd_rot, "kd_rot");
  if (cfg["nullspace_stiffness"]) {
    out.nullspace_kp = cfg["nullspace_stiffness"].as<double>();
  }
  if (cfg["nullspace_damping"]) {
    out.nullspace_kd = cfg["nullspace_damping"].as<double>();
  }
  if (cfg["singularity_threshold"]) {
    out.singularity_threshold = compliance::FloorSigma0(cfg["singularity_threshold"].as<double>());
  }
  if (cfg["singularity_critical"]) {
    out.singularity_critical = std::max(0.0, cfg["singularity_critical"].as<double>());
  }
  if (cfg["max_damping"]) {
    out.max_damping = compliance::FloorMaxDamping(cfg["max_damping"].as<double>());
  }
  // §5.3's three. Read raw here and bounded unconditionally below, next to the
  // posture floors — a bound written inside these `if`s would miss the absent
  // key, which is the case that most needs it (#280).
  if (cfg["joint_limit_margin"]) {
    out.joint_limit_margin = cfg["joint_limit_margin"].as<double>();
  }
  if (cfg["joint_limit_stiffness"]) {
    out.joint_limit_kp = cfg["joint_limit_stiffness"].as<double>();
  }
  if (cfg["joint_limit_damping"]) {
    out.joint_limit_kd = cfg["joint_limit_damping"].as<double>();
  }
  // F8 — the two thresholds a first tick compares against. Both were transcribed
  // unguarded from the adapter, and the sibling schemas (cascade for both,
  // admittance for pose_error_limit) already reject exactly these values; the
  // three now sit in one directory, which is what made the asymmetry visible
  // (#298 S7c-2). `!(v > 0.0)` and not `v <= 0.0` so NaN is rejected too — it
  // makes every comparison false, which is the quietest failure of the three.
  //
  // max_torque_rate is a rate BOUND: compliance::RateLimit early-returns on
  // `max_rate <= 0.0`, so a 0 does not freeze the command — it turns §10.5's
  // slew stage OFF for the life of the deployment, with `SafetyStatus::
  // rate_limited` never going true and nothing in the diagnostics to say the
  // arm has no slew protection. Rejected rather than clamped: there is no
  // defensible value to clamp TO, and a silently disabled limiter is worse than
  // a configure failure.
  if (const YAML::Node& n = cfg["max_torque_rate"]; n) {
    const double v = n.as<double>();
    if (!(v > 0.0) || !std::isfinite(v)) {
      throw std::runtime_error("task_impedance: max_torque_rate must be > 0 and finite");
    }
    out.max_torque_rate = v;
  }
  // pose_error_limit is compared every tick against a CRITICAL fault, so a 0 or
  // negative bound makes `e.norm() > limit` true forever: SAFE_STOP latches on
  // the first tick and `pose_error_exceeded` carries no cause field to point at
  // the config. An infinite one is the mirror image — the CRITICAL bound never
  // fires at all. The `<= 0 disables` idiom is deliberately NOT offered (D6):
  // this IS the guard, and a way to switch it off makes a mis-configuration look
  // like a legitimate setting.
  if (const YAML::Node& n = cfg["pose_error_limit"]; n) {
    const double v = n.as<double>();
    if (!(v > 0.0) || !std::isfinite(v)) {
      throw std::runtime_error("task_impedance: pose_error_limit must be > 0 and finite");
    }
    out.pose_error_limit = v;
  }
  if (cfg["activation_ramp_time"]) {
    out.activation_ramp_time = cfg["activation_ramp_time"].as<double>();
  }
  if (cfg["estop_damping"]) {
    out.estop_damping = std::max(0.0, cfg["estop_damping"].as<double>());
  }

  // Posture gains floored at 0 (#277), HERE — after the parse and before the two
  // rules below, because the order is what makes those two mean what they read
  // as, and unconditionally rather than inside the `if (cfg[…])` above, because
  // when the key is ABSENT the value still arrives from the constructor or a
  // previous set_gains() and both rules must judge the gain Compute() will run:
  //   • §6.4's correction is `if (kp > 0.0)`, so a negative K_pⁿ skipped the
  //     damping floor entirely — the configuration that most needs damping was
  //     the one that never got it.
  //   • §6.1's guard tests `kp == 0.0` while its message claims `> 0`, so a
  //     negative K_pⁿ walked straight through the check that exists precisely
  //     because TRANSLATION_ONLY leaves orientation to the null space.
  // A binding must floor again at the point of use because set_gains() bypasses
  // configure entirely (NUM-1) — the same requirement λ_max carries through
  // compliance::FloorMaxDamping. Neither tick-side half has a caller today: the
  // adapter that held them went with #298 S7c-2 and no shipped binding has
  // replaced it. The two rules below do NOT get that second half regardless;
  // see §6.4's note.
  out.nullspace_kp = FloorNonNegativeGain(out.nullspace_kp);
  out.nullspace_kd = FloorNonNegativeGain(out.nullspace_kd);

  // §5.3's two safety-layer gains take the SAME bound (#280), and for the same
  // reason the posture gains do — with the sign flipped, which is why this
  // schema being the one without it was the sharpest gap in the directory:
  // inside the margin band `q − lo` is negative, so a negative k_lim turns the
  // term that pushes a joint AWAY from its limit into one that pushes it IN, and
  // a negative d_lim injects energy exactly where the arm is closest to a hard
  // stop. The shipped UR5e config runs `k_lim=0, d_lim=2` (pure damping), which
  // makes d_lim the only defence there. Unconditional, and NOT `std::max`: the
  // §10.5 finite check sits immediately after AddJointLimitRepulsive and BEFORE
  // saturation, so laundering a NaN gain to 0.0 here is what retires the
  // `nan_inf` SAFE_STOP that a NaN torque would otherwise raise.
  out.joint_limit_kp = FloorNonNegativeGain(out.joint_limit_kp);
  out.joint_limit_kd = FloorNonNegativeGain(out.joint_limit_kd);
  // The margin is the one member of the trio that CANNOT be floored — see
  // IsFiniteNonNegative. Unconditional like the floors above, so a δ that
  // arrived from the constructor or a set_gains() POD is judged too.
  if (!IsFiniteNonNegative(out.joint_limit_margin)) {
    throw std::runtime_error("task_impedance: joint_limit_margin must be >= 0 and finite");
  }

  // §6.4 nullspace damping floor K_dⁿ ≥ 2√K_pⁿ (also allows K_pⁿ=0 pure damping).
  //
  // CONFIGURE-TIME ONLY, deliberately — σ₀'s side of the NUM-1 split, not
  // λ_max's. This is not a bound on the gain being read but a REWRITE of a
  // different gain, so putting it on the tick would (a) make get_gains() and
  // Compute() disagree about K_dⁿ for every positive K_pⁿ and (b) oblige every
  // literal oracle of this posture block to mirror a configure-time rule. It
  // also buys less than the floor does: `set_gains({kp, kd < 2√kp})` leaves an
  // under-damped null space — oscillatory, bounded, and visible — where an
  // unfloored negative gain leaves a DIVERGENT one that Nᵀ hides. If a caller
  // ever needs the correction at runtime, the honest fix is to apply it where
  // the POD is installed (set_gains), not to re-derive it every tick.
  if (out.nullspace_kp > 0.0) {
    out.nullspace_kd = std::max(out.nullspace_kd, 2.0 * std::sqrt(out.nullspace_kp));
  }

  // §6.1: TRANSLATION_ONLY leaves orientation to the nullspace, so zero nullspace
  // stiffness would let it drift uncontrolled — a configure error, not a warning.
  // `!(kp > 0.0)` and not `kp == 0.0`: the message claims `> 0`, and after the
  // floor above the only remaining way to fail it while comparing equal to
  // nothing is a NaN gain, which is exactly the value that must not configure.
  if (selection == TaskSelection::kTranslationOnly && !(out.nullspace_kp > 0.0)) {
    throw std::runtime_error(
        "task_impedance: TRANSLATION_ONLY requires nullspace_stiffness > 0 (§6.1)");
  }

  // ── Axis A: formulation (§6.2 vs §6.3) ─────────────────────────────────────
  TaskImpedanceFormulation formulation = TaskImpedanceFormulation::kJacobianTranspose;
  if (cfg["formulation"]) {
    const auto s = cfg["formulation"].as<std::string>();
    if (s == "jacobian_transpose") {
      formulation = TaskImpedanceFormulation::kJacobianTranspose;
    } else if (s == "inertia_shaping") {
      formulation = TaskImpedanceFormulation::kInertiaShaping;
    } else {
      throw std::runtime_error(
          "task_impedance: formulation must be 'jacobian_transpose' or 'inertia_shaping' "
          "(got '" +
          s + "')");
    }
  }

  // §5.2 MUST — inertia shaping stays OFF unless asked for, and Λ_d defaults to
  // Λ_S (neutral) unless a desired inertia is actually supplied.
  if (const YAML::Node& di = cfg["desired_inertia"]; di && di.IsSequence()) {
    if (di.size() != 6) {
      throw std::runtime_error("task_impedance: desired_inertia must have 6 entries");
    }
    for (std::size_t i = 0; i < 6; ++i) {
      const double v = di[i].as<double>();
      // NUM-2: Λ_d⁻¹ is formed every tick — a zero or negative desired inertia
      // is not a soft-clamped tuning mistake, it is a non-SPD Λ_d.
      if (!(v > 0.0)) {
        throw std::runtime_error(
            "task_impedance: desired_inertia entries must be > 0 (Λ_d must be SPD)");
      }
      out.inertia.desired_inertia[i] = v;
    }
    out.inertia.desired_inertia_natural = false;
  }
  if (cfg["max_inertia_ratio"]) {
    out.inertia.max_inertia_ratio = std::max(1.0, cfg["max_inertia_ratio"].as<double>());
  }

  // ── External wrench source (§3.2.1). Disabled ⇒ A=NONE, slice-1 behaviour ──
  bool wrench_enabled = false;
  std::string sensor_frame;
  compliance::WrenchConditioningConfig wc;
  if (const YAML::Node& ew = cfg["external_wrench"]; ew && ew.IsMap()) {
    if (ew["enabled"]) {
      wrench_enabled = ew["enabled"].as<bool>();
    }
    if (ew["sensor_frame"]) {
      sensor_frame = ew["sensor_frame"].as<std::string>();
    }
    auto load6 = [](const YAML::Node& n, compliance::Wrench6& arr, const char* what) {
      if (!n) {
        return;
      }
      if (!n.IsSequence() || n.size() != 6) {
        throw std::runtime_error(std::string("task_impedance: ") + what +
                                 " must be a 6-entry sequence [fx,fy,fz,tx,ty,tz]");
      }
      for (std::size_t i = 0; i < 6; ++i) {
        arr[i] = n[i].as<double>();
      }
    };
    load6(ew["deadband"], wc.deadband, "external_wrench.deadband");
    load6(ew["max"], wc.max_abs, "external_wrench.max");
    if (ew["payload_mass"]) {
      wc.payload_mass = ew["payload_mass"].as<double>();
    }
    if (const YAML::Node& com = ew["payload_com"]; com && com.IsSequence()) {
      if (com.size() != 3) {
        throw std::runtime_error("task_impedance: external_wrench.payload_com must have 3 entries");
      }
      for (std::size_t i = 0; i < 3; ++i) {
        wc.payload_com(static_cast<Eigen::Index>(i)) = com[i].as<double>();
      }
    }
    if (ew["filter_enabled"]) {
      wc.filter_enabled = ew["filter_enabled"].as<bool>();
    }
    if (ew["filter_cutoff_force"]) {
      wc.filter_cutoff_force_hz = ew["filter_cutoff_force"].as<double>();
    }
    if (ew["filter_cutoff_torque"]) {
      wc.filter_cutoff_torque_hz = ew["filter_cutoff_torque"].as<double>();
    }
    if (ew["bias_calibration_samples"]) {
      wc.bias_samples = ew["bias_calibration_samples"].as<int>();
    }
    if (ew["timeout"]) {
      out.wrench_timeout = std::max(0.0, ew["timeout"].as<double>());
    }
    if (ew["fadeout_time"]) {
      out.wrench_fadeout_time = std::max(0.0, ew["fadeout_time"].as<double>());
    }
    if (ew["contact_threshold"]) {
      out.contact_threshold = std::max(0.0, ew["contact_threshold"].as<double>());
    }
    if (ew["contact_release_ratio"]) {
      // Clamped strictly below 1 so the ⇄ transition always has hysteresis
      // (§10.6 MUST): equal thresholds chatter at the boundary.
      out.contact_release_ratio = std::clamp(ew["contact_release_ratio"].as<double>(), 0.0, 0.99);
    }
  }

  // Torque-only (MuJoCo backend scope). Reject other command types fail-fast,
  // BEFORE committing anything, so a rejected reconfigure never mutates the
  // caller's live wiring.
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    if (s != "torque") {
      throw std::runtime_error("task_impedance: command_type must be 'torque' (got '" + s + "')");
    }
  }

  // Commit the non-gain half. Nothing above this line has written `config`, so
  // every throw so far left the caller's live state exactly as it was — the
  // frame lookup and the wrench reconfigure the caller still has to run are the
  // last two things that can fail (see the header).
  config.sensor_frame = sensor_frame;
  config.wrench_enabled = wrench_enabled;
  config.wrench = wc;
  config.formulation = formulation;
  config.command_type = CommandType::kTorque;
}

}  // namespace rtc::params
