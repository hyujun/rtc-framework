#include "rtc_controllers/params/task_admittance_params.hpp"

#include "rtc_controllers/compliance/external_wrench.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/joint/posture_law.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace rtc::params {

void ParseTaskAdmittanceParams(const YAML::Node& cfg, TaskAdmittanceParams& out,
                               TaskAdmittanceConfig& config) {
  if (!cfg) {
    // NUM-6's loader half is "regardless of whether the key is present", and an
    // absent NODE is the widest case of that: the gain here came from the
    // constructor or a previous set_gains(), the two paths the floor exists for.
    out.nullspace_kp = joint::FloorPostureGain(out.nullspace_kp);
    return;
  }

  // Shape-checked exactly like load6 below. Silently ignoring a mis-shaped node
  // is worse here than anywhere else in this function: the header documents
  // `ik_kp_*: 0` as the way to reproduce §7.3's pure-feedforward law literally,
  // so a scalar `ik_kp_pos: 0.0` — the obvious way to write that — was dropped
  // and the default 2.0 kept, which means the one experiment the knob exists for
  // silently ran as a CLIK variant, with nothing in the diagnostics to say so.
  // No scalar broadcast (D5): a convenience shorthand would re-introduce the
  // same "quietly a different value" failure in a new shape.
  auto load3 = [](const YAML::Node& n, std::array<double, 3>& arr, const char* what) {
    if (!n) {
      return;
    }
    if (!n.IsSequence() || n.size() != 3) {
      throw std::runtime_error(std::string("task_admittance: ") + what +
                               " must be a 3-entry sequence [x,y,z]");
    }
    for (std::size_t i = 0; i < 3; ++i) {
      arr[i] = n[i].as<double>();
    }
  };
  auto load6 = [](const YAML::Node& n, std::array<double, 6>& arr, const char* what,
                  bool require_positive) {
    if (!n) {
      return;
    }
    if (!n.IsSequence() || n.size() != 6) {
      throw std::runtime_error(std::string("task_admittance: ") + what +
                               " must be a 6-entry sequence [x,y,z,rx,ry,rz]");
    }
    for (std::size_t i = 0; i < 6; ++i) {
      const double v = n[i].as<double>();
      // NUM-2: Λ_d is inverted every tick. A zero or negative entry is not a
      // soft-clamped tuning mistake, it is a divide.
      if (require_positive && !(v > 0.0)) {
        throw std::runtime_error(std::string("task_admittance: ") + what + " entries must be > 0");
      }
      if (!require_positive && v < 0.0) {
        throw std::runtime_error(std::string("task_admittance: ") + what + " entries must be >= 0");
      }
      arr[i] = v;
    }
  };

  // ── §7.2 virtual dynamics ────────────────────────────────────────────────
  load6(cfg["desired_inertia"], out.admittance.inertia, "desired_inertia", true);
  load6(cfg["damping"], out.admittance.damping, "damping", false);
  load6(cfg["stiffness"], out.admittance.stiffness, "stiffness", false);

  // ── §7.4 contact-stability floor: [translation kg, rotation kg·m²] ───────
  if (const YAML::Node& n = cfg["min_desired_inertia"]; n) {
    if (!n.IsSequence() || n.size() != 2) {
      throw std::runtime_error(
          "task_admittance: min_desired_inertia must be [translation, rotation]");
    }
    out.admittance.min_inertia_lin = std::max(0.0, n[0].as<double>());
    out.admittance.min_inertia_ang = std::max(0.0, n[1].as<double>());
  }

  // ── §7.5 workspace / velocity bounds ─────────────────────────────────────
  if (const YAML::Node& n = cfg["max_compliant_displacement"]; n) {
    if (!n.IsSequence() || n.size() != 2) {
      throw std::runtime_error(
          "task_admittance: max_compliant_displacement must be [metres, radians]");
    }
    out.admittance.max_displacement_lin = n[0].as<double>();
    out.admittance.max_displacement_ang = n[1].as<double>();
  }
  if (cfg["max_compliant_linear_velocity"]) {
    out.admittance.max_velocity_lin = cfg["max_compliant_linear_velocity"].as<double>();
  }
  if (cfg["max_compliant_angular_velocity"]) {
    out.admittance.max_velocity_ang = cfg["max_compliant_angular_velocity"].as<double>();
  }
  // Separate from the two above on purpose — see AdmittanceParams. Not floored
  // here: the floor lives at the point of use so set_gains() cannot bypass it
  // (NUM-1), and clamping here as well would only hide a bad YAML value from
  // whoever reads the gains back.
  if (cfg["max_return_linear_velocity"]) {
    out.admittance.max_return_velocity_lin = cfg["max_return_linear_velocity"].as<double>();
  }
  if (cfg["max_return_angular_velocity"]) {
    out.admittance.max_return_velocity_ang = cfg["max_return_angular_velocity"].as<double>();
  }
  if (const YAML::Node& n = cfg["barrier_stiffness"]; n) {
    if (!n.IsSequence() || n.size() != 2) {
      throw std::runtime_error("task_admittance: barrier_stiffness must be [linear, angular]");
    }
    out.admittance.barrier_stiffness_lin = std::max(0.0, n[0].as<double>());
    out.admittance.barrier_stiffness_ang = std::max(0.0, n[1].as<double>());
  }

  // ── §7.3 IK ──────────────────────────────────────────────────────────────
  load3(cfg["ik_kp_pos"], out.ik_kp_pos, "ik_kp_pos");
  load3(cfg["ik_kp_rot"], out.ik_kp_rot, "ik_kp_rot");
  if (cfg["nullspace_kp"]) {
    out.nullspace_kp = cfg["nullspace_kp"].as<double>();
  }
  // Floored at 0 (#277): a negative posture gain drives q̇₀ away from the
  // null-space target, and N hides that from the task so it shows up only as a
  // slow silent drift. Unconditional rather than folded into the parse above —
  // when the key is ABSENT the value still arrives from the constructor or a
  // previous set_gains(). A binding must floor it again at the point of use
  // because set_gains() bypasses configure (NUM-1) — the same requirement λ_max
  // carries through compliance::FloorMaxDamping. Neither tick-side half has a
  // caller today; the adapter that held them went with #298 S7c-2.
  out.nullspace_kp = joint::FloorPostureGain(out.nullspace_kp);
  if (cfg["integrate_from_measured"]) {
    out.integrate_from_measured = cfg["integrate_from_measured"].as<bool>();
  }
  if (cfg["singularity_threshold"]) {
    out.singularity_threshold =
        std::max(compliance::kMinSigma0, cfg["singularity_threshold"].as<double>());
  }
  if (cfg["singularity_critical"]) {
    out.singularity_critical = std::max(0.0, cfg["singularity_critical"].as<double>());
  }
  if (cfg["max_damping"]) {
    out.max_damping = compliance::FloorMaxDamping(cfg["max_damping"].as<double>());
  }

  // ── Safety / activation ──────────────────────────────────────────────────
  // Strictly positive, and rejected rather than clamped (D6). `e.norm() > limit`
  // is evaluated every tick against a CRITICAL fault, so a 0 or negative bound
  // makes that comparison true forever: SAFE_STOP latches on the first tick, and
  // `ComplianceFaults::pose_error_exceeded` carries no cause field to point at
  // the config. The `<= 0 disables` idiom the §7.5 bounds use is deliberately
  // NOT offered — this is the guard, and a way to switch it off would make a
  // mis-configuration look like a legitimate setting.
  if (const YAML::Node& n = cfg["pose_error_limit"]; n) {
    const double v = n.as<double>();
    if (!(v > 0.0)) {
      throw std::runtime_error("task_admittance: pose_error_limit must be > 0");
    }
    out.pose_error_limit = v;
  }
  if (cfg["command_divergence_limit"]) {
    out.command_divergence_limit = std::max(0.0, cfg["command_divergence_limit"].as<double>());
  }
  if (cfg["joint_limit_margin"]) {
    out.joint_limit_margin = std::max(0.0, cfg["joint_limit_margin"].as<double>());
  }
  if (cfg["activation_ramp_time"]) {
    out.activation_ramp_time = cfg["activation_ramp_time"].as<double>();
  }
  if (cfg["saturation_persist_time"]) {
    out.saturation_persist_time = std::max(0.0, cfg["saturation_persist_time"].as<double>());
  }

  // ── External wrench source (§3.2.1) — REQUIRED here (§7.1, axis A ≠ NONE) ─
  bool wrench_enabled = true;
  std::string sensor_frame;
  compliance::WrenchConditioningConfig wc;
  if (const YAML::Node& ew = cfg["external_wrench"]; ew && ew.IsMap()) {
    if (ew["enabled"]) {
      wrench_enabled = ew["enabled"].as<bool>();
    }
    if (ew["sensor_frame"]) {
      sensor_frame = ew["sensor_frame"].as<std::string>();
    }
    auto load_w6 = [](const YAML::Node& n, compliance::Wrench6& arr, const char* what) {
      if (!n) {
        return;
      }
      if (!n.IsSequence() || n.size() != 6) {
        throw std::runtime_error(std::string("task_admittance: ") + what +
                                 " must be a 6-entry sequence [fx,fy,fz,tx,ty,tz]");
      }
      for (std::size_t i = 0; i < 6; ++i) {
        arr[i] = n[i].as<double>();
      }
    };
    load_w6(ew["deadband"], wc.deadband, "external_wrench.deadband");
    load_w6(ew["max"], wc.max_abs, "external_wrench.max");
    if (ew["payload_mass"]) {
      wc.payload_mass = ew["payload_mass"].as<double>();
    }
    if (const YAML::Node& com = ew["payload_com"]; com && com.IsSequence()) {
      if (com.size() != 3) {
        throw std::runtime_error(
            "task_admittance: external_wrench.payload_com must have 3 entries");
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
      out.wrench.timeout = std::max(0.0, ew["timeout"].as<double>());
    }
    if (ew["fadeout_time"]) {
      out.wrench.fadeout_time = std::max(0.0, ew["fadeout_time"].as<double>());
    }
    if (ew["contact_threshold"]) {
      out.wrench.contact_threshold = std::max(0.0, ew["contact_threshold"].as<double>());
    }
    if (ew["contact_release_ratio"]) {
      // Clamped strictly below 1 so the ⇄ transition always keeps a hysteresis
      // band (§10.6 MUST): equal thresholds chatter at the boundary.
      out.wrench.contact_release_ratio =
          std::clamp(ew["contact_release_ratio"].as<double>(), 0.0, 0.99);
    }
  }

  // §7.1: admittance takes force as its INPUT. Without a wrench source the
  // compliant frame never leaves X_d and the controller degenerates into an
  // expensive position hold — a configure error, not a fallback (unlike the
  // impedance controller, where A=NONE is a first-class law).
  if (!wrench_enabled) {
    throw std::runtime_error(
        "task_admittance: external_wrench.enabled must be true (§7.1 — admittance "
        "requires axis A != NONE)");
  }

  // Position-only output (D14 — no CommandType::kVelocity). Rejected fail-fast
  // BEFORE committing anything so a bad reconfigure never mutates live state.
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    if (s != "position") {
      throw std::runtime_error("task_admittance: command_type must be 'position' (got '" + s +
                               "')");
    }
  }

  // Commit the non-gain half. Nothing above this line has written `config`, so
  // every throw so far left the caller's live wiring exactly as it was — the
  // frame lookup and the wrench reconfigure the caller still has to run are the
  // last two things that can fail (see the header).
  config.sensor_frame = sensor_frame;
  config.wrench_enabled = wrench_enabled;
  config.wrench = wc;
  config.command_type = CommandType::kPosition;
}

}  // namespace rtc::params
