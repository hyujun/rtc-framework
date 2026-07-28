#include "rtc_controllers/params/cascaded_compliance_params.hpp"

#include "rtc_controllers/compliance/external_wrench.hpp"
#include "rtc_controllers/compliance/task_dynamics.hpp"
#include "rtc_controllers/joint/posture_law.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace rtc::params {

void ParseCascadedComplianceParams(const YAML::Node& cfg, CascadedComplianceParams& out,
                                   CascadedComplianceConfig& config) {
  if (!cfg) {
    // NUM-6's loader half is "regardless of whether the key is present", and an
    // absent NODE is the widest case of that: the gains here came from the
    // constructor or a previous set_gains(), the two paths the floor exists for.
    out.nullspace_kp = joint::FloorPostureGain(out.nullspace_kp);
    out.nullspace_kd = joint::FloorPostureGain(out.nullspace_kd);
    return;
  }

  // Shape-checked, never silently ignored (F7, PR #256): a mis-shaped node that
  // is dropped leaves the DEFAULT gain running under a config file that says
  // otherwise, and nothing downstream can tell the two apart. No scalar
  // broadcast (D5) — a shorthand would re-introduce "quietly a different value"
  // in a new shape.
  // Every scalar read goes through this. NaN and ±inf are NOT tuning mistakes
  // that a clamp can absorb: `std::max(0.0, NaN)` returns 0.0 — silently a
  // different value, the exact failure mode the shape check above exists to
  // prevent — and an infinite gain reaches the admittance state on the first
  // tick, so the controller latches SAFE_STOP with `nan_inf` and nothing points
  // at the config line that caused it.
  auto num = [](const YAML::Node& n, const char* what) {
    const double v = n.as<double>();
    if (!std::isfinite(v)) {
      throw std::runtime_error(std::string("cascaded_compliance: ") + what +
                               " must be a finite number");
    }
    return v;
  };
  // A section that is ABSENT is legal (the defaults apply, which is the
  // documented behaviour); a section that is PRESENT but not a map is a config
  // error. Silently skipping the latter leaves the defaults running under a file
  // that says otherwise — the F7 failure class, one level up from the leaves.
  auto require_map = [](const YAML::Node& n, const char* what) {
    if (n && !n.IsMap()) {
      throw std::runtime_error(std::string("cascaded_compliance: ") + what +
                               " must be a map of keys");
    }
  };
  auto load3 = [&num](const YAML::Node& n, std::array<double, 3>& arr, const char* what,
                      bool require_non_negative) {
    if (!n) {
      return;
    }
    if (!n.IsSequence() || n.size() != 3) {
      throw std::runtime_error(std::string("cascaded_compliance: ") + what +
                               " must be a 3-entry sequence [x,y,z]");
    }
    for (std::size_t i = 0; i < 3; ++i) {
      const double v = num(n[i], what);
      if (require_non_negative && !(v >= 0.0)) {
        throw std::runtime_error(std::string("cascaded_compliance: ") + what +
                                 " entries must be >= 0");
      }
      arr[i] = v;
    }
  };
  auto load6 = [&num](const YAML::Node& n, std::array<double, 6>& arr, const char* what,
                      bool require_positive) {
    if (!n) {
      return;
    }
    if (!n.IsSequence() || n.size() != 6) {
      throw std::runtime_error(std::string("cascaded_compliance: ") + what +
                               " must be a 6-entry sequence [x,y,z,rx,ry,rz]");
    }
    for (std::size_t i = 0; i < 6; ++i) {
      // NaN is rejected by `num` above, not here: `v < 0.0` is FALSE for NaN, so
      // the non-negative branch would pass it straight into the admittance state.
      const double v = num(n[i], what);
      // NUM-2: Λ_d is inverted every tick. A zero or negative entry is not a
      // soft-clamped tuning mistake, it is a divide.
      if (require_positive && !(v > 0.0)) {
        throw std::runtime_error(std::string("cascaded_compliance: ") + what +
                                 " entries must be > 0");
      }
      if (!require_positive && v < 0.0) {
        throw std::runtime_error(std::string("cascaded_compliance: ") + what +
                                 " entries must be >= 0");
      }
      arr[i] = v;
    }
  };

  // ── OUTER loop (§7.2 / §7.4 / §7.5) ──────────────────────────────────────
  // Nested under `outer:` / `inner:` rather than flattened: both loops have a
  // stiffness and a damping, and a flat `stiffness:` key would be the single
  // most consequential ambiguity in this file — the outer one defines how the
  // robot yields to force, the inner one only how tightly it tracks.
  require_map(cfg["outer"], "outer");
  if (const YAML::Node& o = cfg["outer"]; o) {
    load6(o["desired_inertia"], out.admittance.inertia, "outer.desired_inertia", true);
    load6(o["damping"], out.admittance.damping, "outer.damping", false);
    load6(o["stiffness"], out.admittance.stiffness, "outer.stiffness", false);
    if (const YAML::Node& n = o["min_desired_inertia"]; n) {
      if (!n.IsSequence() || n.size() != 2) {
        throw std::runtime_error(
            "cascaded_compliance: outer.min_desired_inertia must be [translation, rotation]");
      }
      out.admittance.min_inertia_lin = std::max(0.0, num(n[0], "outer.min_desired_inertia"));
      out.admittance.min_inertia_ang = std::max(0.0, num(n[1], "outer.min_desired_inertia"));
    }
    if (const YAML::Node& n = o["max_compliant_displacement"]; n) {
      if (!n.IsSequence() || n.size() != 2) {
        throw std::runtime_error(
            "cascaded_compliance: outer.max_compliant_displacement must be [metres, radians]");
      }
      out.admittance.max_displacement_lin = num(n[0], "outer.max_compliant_displacement");
      out.admittance.max_displacement_ang = num(n[1], "outer.max_compliant_displacement");
    }
    if (o["max_compliant_linear_velocity"]) {
      out.admittance.max_velocity_lin =
          num(o["max_compliant_linear_velocity"], "outer.max_compliant_linear_velocity");
    }
    if (o["max_compliant_angular_velocity"]) {
      out.admittance.max_velocity_ang =
          num(o["max_compliant_angular_velocity"], "outer.max_compliant_angular_velocity");
    }
    // Separate keys from the two above on purpose (PR #256 F2) — see
    // AdmittanceParams. Not floored here: the floor lives at the point of use so
    // set_gains() cannot bypass it (NUM-1).
    if (o["max_return_linear_velocity"]) {
      out.admittance.max_return_velocity_lin =
          num(o["max_return_linear_velocity"], "outer.max_return_linear_velocity");
    }
    if (o["max_return_angular_velocity"]) {
      out.admittance.max_return_velocity_ang =
          num(o["max_return_angular_velocity"], "outer.max_return_angular_velocity");
    }
    if (const YAML::Node& n = o["barrier_stiffness"]; n) {
      if (!n.IsSequence() || n.size() != 2) {
        throw std::runtime_error(
            "cascaded_compliance: outer.barrier_stiffness must be [linear, angular]");
      }
      out.admittance.barrier_stiffness_lin = std::max(0.0, num(n[0], "outer.barrier_stiffness"));
      out.admittance.barrier_stiffness_ang = std::max(0.0, num(n[1], "outer.barrier_stiffness"));
    }
  }

  // ── INNER loop (§6.2) ────────────────────────────────────────────────────
  require_map(cfg["inner"], "inner");
  if (const YAML::Node& in = cfg["inner"]; in) {
    load3(in["kp_pos"], out.impedance.kp_pos, "inner.kp_pos", true);
    load3(in["kd_pos"], out.impedance.kd_pos, "inner.kd_pos", true);
    load3(in["kp_rot"], out.impedance.kp_rot, "inner.kp_rot", true);
    load3(in["kd_rot"], out.impedance.kd_rot, "inner.kd_rot", true);
  }

  // ── Nullspace / DLS / safety / activation ────────────────────────────────
  // The posture floors this controller has carried since it shipped — #277
  // converged the other four onto them. Applied unconditionally AFTER the parse
  // so an ABSENT key cannot let a constructor / set_gains() value through, and
  // floored once more at the point of use because set_gains() bypasses configure
  // (NUM-1). The parse itself no longer clamps: `max(0, max(0, x)) == max(0, x)`
  // for every input, so a clamp inside the `if` could never change the stored
  // value — two spellings of one bound, only one of which covers the absent key.
  if (cfg["nullspace_stiffness"]) {
    out.nullspace_kp = num(cfg["nullspace_stiffness"], "nullspace_stiffness");
  }
  if (cfg["nullspace_damping"]) {
    out.nullspace_kd = num(cfg["nullspace_damping"], "nullspace_damping");
  }
  out.nullspace_kp = joint::FloorPostureGain(out.nullspace_kp);
  out.nullspace_kd = joint::FloorPostureGain(out.nullspace_kd);
  if (cfg["singularity_threshold"]) {
    out.singularity_threshold = std::max(
        compliance::kMinSigma0, num(cfg["singularity_threshold"], "singularity_threshold"));
  }
  if (cfg["singularity_critical"]) {
    out.singularity_critical =
        std::max(0.0, num(cfg["singularity_critical"], "singularity_critical"));
  }
  if (cfg["max_damping"]) {
    out.max_damping = std::max(compliance::kMinMaxDamping, num(cfg["max_damping"], "max_damping"));
  }
  if (cfg["joint_limit_margin"]) {
    out.joint_limit_margin = std::max(0.0, num(cfg["joint_limit_margin"], "joint_limit_margin"));
  }
  if (cfg["joint_limit_stiffness"]) {
    out.joint_limit_kp = std::max(0.0, num(cfg["joint_limit_stiffness"], "joint_limit_stiffness"));
  }
  if (cfg["joint_limit_damping"]) {
    out.joint_limit_kd = std::max(0.0, num(cfg["joint_limit_damping"], "joint_limit_damping"));
  }
  // F8 — every threshold that a first tick compares against must be incapable of
  // latching SAFE_STOP by being 0 or negative.
  //
  // max_torque_rate is a rate BOUND: 0 would freeze the command at the
  // rate-limit history forever (the arm stops responding while every fault stays
  // clear), so it is rejected rather than clamped.
  if (const YAML::Node& n = cfg["max_torque_rate"]; n) {
    const double v = num(n, "max_torque_rate");
    if (!(v > 0.0)) {
      throw std::runtime_error("cascaded_compliance: max_torque_rate must be > 0");
    }
    out.max_torque_rate = v;
  }
  // pose_error_limit is compared every tick against a CRITICAL fault, so a 0 or
  // negative bound makes `e.norm() > limit` true forever: SAFE_STOP latches on
  // the first tick and `pose_error_exceeded` carries no cause field to point at
  // the config. The `<= 0 disables` idiom is deliberately NOT offered — this is
  // the guard (D6).
  if (const YAML::Node& n = cfg["pose_error_limit"]; n) {
    const double v = num(n, "pose_error_limit");
    if (!(v > 0.0)) {
      throw std::runtime_error("cascaded_compliance: pose_error_limit must be > 0");
    }
    out.pose_error_limit = v;
  }
  if (cfg["activation_ramp_time"]) {
    out.activation_ramp_time = num(cfg["activation_ramp_time"], "activation_ramp_time");
  }
  if (cfg["estop_damping"]) {
    out.estop_damping = std::max(0.0, num(cfg["estop_damping"], "estop_damping"));
  }
  // saturation_persist_time == 0 would degrade on the FIRST saturated tick.
  // That is a legitimate (if twitchy) setting — DEGRADED is recoverable and
  // carries no latch — so it is clamped at 0 rather than rejected.
  if (cfg["saturation_persist_time"]) {
    out.saturation_persist_time =
        std::max(0.0, num(cfg["saturation_persist_time"], "saturation_persist_time"));
  }
  // 0 silences the §7.6 MUST-1 flag; negative is meaningless, not a disable.
  if (cfg["min_bandwidth_ratio"]) {
    out.min_bandwidth_ratio = std::max(0.0, num(cfg["min_bandwidth_ratio"], "min_bandwidth_ratio"));
  }

  // ── External wrench source (§3.2.1) — REQUIRED (the outer loop's input) ───
  bool wrench_enabled = true;
  std::string sensor_frame;
  compliance::WrenchConditioningConfig wc;
  require_map(cfg["external_wrench"], "external_wrench");
  if (const YAML::Node& ew = cfg["external_wrench"]; ew) {
    if (ew["enabled"]) {
      wrench_enabled = ew["enabled"].as<bool>();
    }
    if (ew["sensor_frame"]) {
      sensor_frame = ew["sensor_frame"].as<std::string>();
    }
    auto load_w6 = [&num](const YAML::Node& n, compliance::Wrench6& arr, const char* what) {
      if (!n) {
        return;
      }
      if (!n.IsSequence() || n.size() != 6) {
        throw std::runtime_error(std::string("cascaded_compliance: ") + what +
                                 " must be a 6-entry sequence [fx,fy,fz,tx,ty,tz]");
      }
      for (std::size_t i = 0; i < 6; ++i) {
        arr[i] = num(n[i], what);
      }
    };
    load_w6(ew["deadband"], wc.deadband, "external_wrench.deadband");
    load_w6(ew["max"], wc.max_abs, "external_wrench.max");
    if (ew["payload_mass"]) {
      wc.payload_mass = num(ew["payload_mass"], "external_wrench.payload_mass");
    }
    if (const YAML::Node& com = ew["payload_com"]; com && com.IsSequence()) {
      if (com.size() != 3) {
        throw std::runtime_error(
            "cascaded_compliance: external_wrench.payload_com must have 3 entries");
      }
      for (std::size_t i = 0; i < 3; ++i) {
        wc.payload_com(static_cast<Eigen::Index>(i)) = num(com[i], "external_wrench.payload_com");
      }
    }
    if (ew["filter_enabled"]) {
      wc.filter_enabled = ew["filter_enabled"].as<bool>();
    }
    if (ew["filter_cutoff_force"]) {
      wc.filter_cutoff_force_hz =
          num(ew["filter_cutoff_force"], "external_wrench.filter_cutoff_force");
    }
    if (ew["filter_cutoff_torque"]) {
      wc.filter_cutoff_torque_hz =
          num(ew["filter_cutoff_torque"], "external_wrench.filter_cutoff_torque");
    }
    if (ew["bias_calibration_samples"]) {
      wc.bias_samples = ew["bias_calibration_samples"].as<int>();
    }
    if (ew["timeout"]) {
      out.wrench.timeout = std::max(0.0, num(ew["timeout"], "external_wrench.timeout"));
    }
    if (ew["fadeout_time"]) {
      out.wrench.fadeout_time =
          std::max(0.0, num(ew["fadeout_time"], "external_wrench.fadeout_time"));
    }
    if (ew["contact_threshold"]) {
      out.wrench.contact_threshold =
          std::max(0.0, num(ew["contact_threshold"], "external_wrench.contact_threshold"));
    }
    if (ew["contact_release_ratio"]) {
      // Clamped strictly below 1 so the ⇄ transition always keeps a hysteresis
      // band (§10.6 MUST): equal thresholds chatter at the boundary.
      out.wrench.contact_release_ratio = std::clamp(
          num(ew["contact_release_ratio"], "external_wrench.contact_release_ratio"), 0.0, 0.99);
    }
  }

  // §7.6: the cascade exists to turn a MEASURED force into motion. With no
  // source the compliant frame never leaves X_d and the controller is a §6.2
  // impedance controller with extra steps — one that already exists and is
  // better tested. A configure error, not a fallback.
  if (!wrench_enabled) {
    throw std::runtime_error(
        "cascaded_compliance: external_wrench.enabled must be true (§7.6 — the outer "
        "admittance loop has no input otherwise; use the task-impedance law for A=NONE)");
  }

  // Torque output only. Rejected fail-fast BEFORE committing anything so a bad
  // reconfigure never mutates live state.
  if (cfg["command_type"]) {
    const auto s = cfg["command_type"].as<std::string>();
    if (s != "torque") {
      throw std::runtime_error("cascaded_compliance: command_type must be 'torque' (got '" + s +
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
  config.command_type = CommandType::kTorque;
}

}  // namespace rtc::params
