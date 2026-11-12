#include "rtc_mpc/reference/stage_reference_builder.hpp"

#include "reference/internal/yaml_fields.hpp"
#include "rtc_mpc/reference/contact_schedule.hpp"

#include <Eigen/Geometry>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <bit>
#include <cmath>
#include <numbers>

namespace rtc::mpc {
namespace {

using internal::FieldRead;

constexpr double kQuaternionTolerance = 1e-6;  ///< `|‖q‖² − 1|` the snapshot may carry

[[nodiscard]] bool Listed(std::span<const std::string> names, const std::string& name) noexcept {
  return std::find(names.begin(), names.end(), name) != names.end();
}

/// Joints after the root (id ≥ 2) must be 1-DoF; id 1 is the free-flyer.
[[nodiscard]] bool OneDof(const pinocchio::Model& model, pinocchio::JointIndex j) noexcept {
  return model.joints[j].nq() == 1 && model.joints[j].nv() == 1;
}

/// The yaw-only quaternion `(0, 0, sin ψ/2, cos ψ/2)` into `out[0 … 3]` (xyzw).
void YawQuaternion(double psi, double* out) noexcept {
  out[0] = 0.0;
  out[1] = 0.0;
  out[2] = std::sin(0.5 * psi);
  out[3] = std::cos(0.5 * psi);
}

/// `a + (b − a)·α` — the one interpolation form (an endpoint is reproduced exactly at α ∈ {0, 1}).
[[nodiscard]] constexpr double Lerp(double a, double b, double alpha) noexcept {
  return a + (b - a) * alpha;
}

/// The yaw of a rotation: `atan2(R10, R00)`.
[[nodiscard]] double Yaw(const Eigen::Matrix3d& R) noexcept {
  return std::atan2(R(1, 0), R(0, 0));
}

}  // namespace

// ── YAML ──────────────────────────────────────────────────────────────────────

ReferenceError LoadReferenceConfig(const YAML::Node& node, std::span<const std::string> joint_names,
                                   ReferenceConfig& out) noexcept {
  try {
    if (!node.IsDefined() || !node.IsMap()) {
      return ReferenceError::kNotAMap;
    }
    if (!internal::HasOnlyKeys(node, {"intermediate_fraction", "q_default", "arm_swing"})) {
      return ReferenceError::kUnknownKey;
    }
    ReferenceConfig c;
    const FieldRead f = internal::ReadScalar(internal::Child(node, "intermediate_fraction"),
                                             c.intermediate_fraction);
    if (f != FieldRead::kOk) {
      return internal::ToError<ReferenceError>(f);
    }
    if (!(c.intermediate_fraction > 0.0 && c.intermediate_fraction < 1.0)) {
      return ReferenceError::kInvalidFraction;
    }

    const YAML::Node qd = internal::Child(node, "q_default");
    if (!internal::Present(qd)) {
      return ReferenceError::kMissingKey;
    }
    if (!qd.IsMap()) {
      return ReferenceError::kNotAMap;
    }
    for (const auto& kv : qd) {
      if (!kv.first.IsScalar()) {
        return ReferenceError::kWrongType;
      }
      const std::string name = kv.first.Scalar();
      if (!Listed(joint_names, name)) {
        return ReferenceError::kUnknownJoint;
      }
      double value = 0.0;
      const FieldRead r = internal::ReadScalar(kv.second, value);
      if (r != FieldRead::kOk) {
        return r == FieldRead::kMissing ? ReferenceError::kWrongType
                                        : internal::ToError<ReferenceError>(r);
      }
      c.q_default.emplace_back(name, value);
    }
    for (const std::string& name : joint_names) {
      const bool covered = std::any_of(c.q_default.begin(), c.q_default.end(),
                                       [&](const auto& e) { return e.first == name; });
      if (!covered) {
        return ReferenceError::kMissingJoint;
      }
    }

    const YAML::Node arm = internal::Child(node, "arm_swing");
    if (!internal::Present(arm)) {
      return ReferenceError::kMissingKey;
    }
    if (!arm.IsMap()) {
      return ReferenceError::kNotAMap;
    }
    if (!internal::HasOnlyKeys(arm, {"enabled", "gain", "phase_offset", "joints"})) {
      return ReferenceError::kUnknownKey;
    }
    const FieldRead reads[] = {
        internal::ReadBool(internal::Child(arm, "enabled"), c.arm_swing_enabled),
        internal::ReadScalar(internal::Child(arm, "gain"), c.arm_swing_gain),
        internal::ReadScalar(internal::Child(arm, "phase_offset"), c.arm_swing_phase_offset),
    };
    for (const FieldRead r : reads) {
      if (r != FieldRead::kOk) {
        return internal::ToError<ReferenceError>(r);
      }
    }
    const YAML::Node joints = internal::Child(arm, "joints");
    if (!joints.IsDefined()) {
      return ReferenceError::kMissingKey;
    }
    if (!joints.IsNull() && !joints.IsSequence()) {
      return ReferenceError::kWrongType;
    }
    if (joints.IsSequence()) {
      for (const YAML::Node& item : joints) {
        if (!item.IsMap()) {
          return ReferenceError::kNotAMap;
        }
        if (!internal::HasOnlyKeys(item, {"name", "sign"})) {
          return ReferenceError::kUnknownKey;
        }
        const YAML::Node name = internal::Child(item, "name");
        if (!internal::Present(name)) {
          return ReferenceError::kMissingKey;
        }
        if (!name.IsScalar()) {
          return ReferenceError::kWrongType;
        }
        ArmSwingJoint j;
        j.name = name.Scalar();
        if (!Listed(joint_names, j.name)) {
          return ReferenceError::kUnknownJoint;
        }
        const FieldRead r = internal::ReadScalar(internal::Child(item, "sign"), j.sign);
        if (r != FieldRead::kOk) {
          return internal::ToError<ReferenceError>(r);
        }
        const bool twice = std::any_of(c.arm_swing_joints.begin(), c.arm_swing_joints.end(),
                                       [&](const ArmSwingJoint& a) { return a.name == j.name; });
        if (twice) {
          return ReferenceError::kDuplicateJoint;
        }
        c.arm_swing_joints.push_back(std::move(j));
      }
    }
    const ReferenceError e = ValidateReferenceConfig(c);
    if (e == ReferenceError::kNoError) {
      out = std::move(c);
    }
    return e;
  } catch (...) {
    return ReferenceError::kWrongType;
  }
}

ReferenceError ValidateReferenceConfig(const ReferenceConfig& c) noexcept {
  if (!std::isfinite(c.intermediate_fraction) || !std::isfinite(c.arm_swing_gain) ||
      !std::isfinite(c.arm_swing_phase_offset)) {
    return ReferenceError::kNonFinite;
  }
  if (!(c.intermediate_fraction > 0.0 && c.intermediate_fraction < 1.0)) {
    return ReferenceError::kInvalidFraction;
  }
  for (const ArmSwingJoint& j : c.arm_swing_joints) {
    if (j.sign != 1.0 && j.sign != -1.0) {
      return ReferenceError::kInvalidSign;
    }
  }
  if (c.arm_swing_enabled && c.arm_swing_joints.empty()) {
    return ReferenceError::kEmptyArmSwingJoints;
  }
  return ReferenceError::kNoError;
}

// ── Init ──────────────────────────────────────────────────────────────────────

ReferenceError StageReferenceBuilder::Init(const RobotModelHandler& handler,
                                           const ReferenceConfig& cfg, const SwingConfig& swing,
                                           const SurfaceFrame& terrain, int n_nodes, double dt) {
  if (!handler.Initialised()) {
    return ReferenceError::kHandlerNotInitialised;
  }
  const pinocchio::Model& model = handler.model();
  // Capacity first: a handler over kMaxContacts carries an empty layout.
  if (n_nodes < 2 || n_nodes > kMaxHorizon + 1 || handler.nq() > kMaxNq || handler.nv() > kMaxNv ||
      handler.n_contacts() > kMaxContacts || handler.layout().n_contact_vars > kMaxContactVars) {
    return ReferenceError::kCapacity;
  }
  if (!handler.floating_base()) {
    return ReferenceError::kFixedBase;
  }
  if (!std::isfinite(dt) || dt < kMinReferenceTimeStep) {
    return ReferenceError::kInvalidTimeStep;
  }
  if (ValidateSwingConfig(swing) != SwingError::kNoError) {
    return ReferenceError::kInvalidSwingConfig;
  }
  if (ValidateSurfaceFrame(terrain) != SurfaceFrameError::kNoError) {
    return ReferenceError::kInvalidTerrain;
  }
  if (const ReferenceError e = ValidateReferenceConfig(cfg); e != ReferenceError::kNoError) {
    return e;
  }

  // q_default: every joint after the root is 1-DoF and has an entry inside its limits.
  std::vector<double> q_default(static_cast<std::size_t>(model.nq), 0.0);
  for (pinocchio::JointIndex j = 2; j < static_cast<pinocchio::JointIndex>(model.njoints); ++j) {
    if (!OneDof(model, j)) {
      return ReferenceError::kUnsupportedJoint;
    }
    const auto it = std::find_if(cfg.q_default.begin(), cfg.q_default.end(),
                                 [&](const auto& e) { return e.first == model.names[j]; });
    if (it == cfg.q_default.end()) {
      return ReferenceError::kMissingJoint;
    }
    const auto idx = static_cast<std::size_t>(model.joints[j].idx_q());
    if (!std::isfinite(it->second)) {
      return ReferenceError::kNonFinite;
    }
    if (it->second < model.lowerPositionLimit[static_cast<Eigen::Index>(idx)] ||
        it->second > model.upperPositionLimit[static_cast<Eigen::Index>(idx)]) {
      return ReferenceError::kOutOfLimits;
    }
    q_default[idx] = it->second;
  }
  std::vector<ArmJoint> arm;  // stays empty when arm swing is off
  for (std::size_t i = 0; cfg.arm_swing_enabled && i < cfg.arm_swing_joints.size(); ++i) {
    const ArmSwingJoint& a = cfg.arm_swing_joints[i];
    if (!model.existJointName(a.name)) {
      continue;  // a joint of a bigger model (the loader checked its name), like q_default's
    }
    const pinocchio::JointIndex j = model.getJointId(a.name);
    if (j < 2 || !OneDof(model, j)) {
      return ReferenceError::kUnsupportedJoint;
    }
    const Eigen::Index idx = model.joints[j].idx_q();
    arm.push_back(ArmJoint{static_cast<int>(idx), a.sign, model.lowerPositionLimit[idx],
                           model.upperPositionLimit[idx]});
  }

  auto data = std::make_unique<pinocchio::Data>(model);
  std::array<std::unique_ptr<HorizonReference>, 2> slots = {std::make_unique<HorizonReference>(),
                                                            std::make_unique<HorizonReference>()};
  handler_ = &handler;
  data_ = std::move(data);
  slots_ = std::move(slots);
  active_ = 0;
  q_default_ = std::move(q_default);
  arm_joints_ = std::move(arm);
  swing_ = swing;
  terrain_ = terrain;
  n_nodes_ = n_nodes;
  dt_ = dt;
  weight_ = pinocchio::computeTotalMass(model) * model.gravity.linear().norm();
  fraction_ = cfg.intermediate_fraction;
  gain_ = cfg.arm_swing_gain;
  phase_offset_ = cfg.arm_swing_phase_offset;
  initialized_ = true;
  return ReferenceError::kNoError;
}

// ── Build ─────────────────────────────────────────────────────────────────────

ReferenceError StageReferenceBuilder::Build(const GaitScheduler& scheduler,
                                            const MPCStateSnapshot& snapshot,
                                            const FilteredCommand& cmd) noexcept {
  if (!initialized_) {
    return ReferenceError::kNotInitialised;
  }
  const ContactSchedule& schedule = scheduler.schedule();
  const pinocchio::Model& model = handler_->model();
  const int nq = handler_->nq();
  const int nv = handler_->nv();
  const auto n_contacts = static_cast<std::size_t>(handler_->n_contacts());
  if (schedule.schedule_id == 0) {
    return ReferenceError::kNoSchedule;
  }
  if (schedule.n_contacts != n_contacts) {
    return ReferenceError::kScheduleMismatch;
  }
  for (std::uint32_t i = 0; i <= schedule.n_events && i <= kMaxScheduleEvents; ++i) {
    if ((schedule.mode[i] >> n_contacts) != 0) {
      return ReferenceError::kScheduleMismatch;  // a bit beyond the model's contacts
    }
  }
  if (snapshot.nq != nq || snapshot.nv != nv) {
    return ReferenceError::kSnapshotMismatch;
  }
  for (int i = 0; i < nq; ++i) {
    if (!std::isfinite(snapshot.q[static_cast<std::size_t>(i)])) {
      return ReferenceError::kNonFiniteInput;
    }
  }
  for (int i = 0; i < nv; ++i) {
    if (!std::isfinite(snapshot.v[static_cast<std::size_t>(i)])) {
      return ReferenceError::kNonFiniteInput;
    }
  }
  if (!std::isfinite(cmd.v_x) || !std::isfinite(cmd.v_y) || !std::isfinite(cmd.omega_z) ||
      !std::isfinite(cmd.base_height)) {
    return ReferenceError::kNonFiniteInput;
  }
  const Eigen::Quaterniond quat(snapshot.q[6], snapshot.q[3], snapshot.q[4], snapshot.q[5]);
  if (std::fabs(quat.squaredNorm() - 1.0) > kQuaternionTolerance) {
    return ReferenceError::kSnapshotMismatch;
  }
  const auto horizon_nodes = static_cast<std::size_t>(n_nodes_ - 1);
  if (!SampleNodes(schedule, schedule.t0, dt_, horizon_nodes,
                   std::span<std::uint32_t>(modes_.data(), static_cast<std::size_t>(n_nodes_)))) {
    return ReferenceError::kWindowTooShort;
  }
  const double p0x = snapshot.q[0];
  const double p0y = snapshot.q[1];
  if (!(cmd.base_height > HeightAt(terrain_, p0x, p0y))) {
    return ReferenceError::kBaseBelowTerrain;
  }

  // Forward kinematics of the measured configuration: where each contact is.
  const Eigen::Map<const Eigen::VectorXd> q(snapshot.q.data(), nq);
  pinocchio::forwardKinematics(model, *data_, q);

  struct Foot {
    double x, y, psi, terrain;
  };

  std::array<Foot, kMaxContacts> foot{};
  for (std::size_t c = 0; c < n_contacts; ++c) {
    const auto fid = static_cast<pinocchio::FrameIndex>(handler_->contact_frames()[c].frame_id);
    const pinocchio::SE3& oMf = pinocchio::updateFramePlacement(model, *data_, fid);
    foot[c].x = oMf.translation()[0];
    foot[c].y = oMf.translation()[1];
    foot[c].psi = Yaw(oMf.rotation());
    foot[c].terrain = HeightAt(terrain_, foot[c].x, foot[c].y);
  }

  // The swing plan of every stretch in the window. A stretch under way at t0
  // is closed at the lift-off the scheduler reports; one clamped at the
  // gait's take-over (a reset or a switch into a swing phase) keeps the
  // open-start plan — the foot is in the air with no lift-off to plan from.
  std::array<std::array<SwingInterval, kMaxSwingsPerContact>, kMaxContacts> intervals{};
  std::array<std::size_t, kMaxContacts> n_swings{};
  for (std::size_t c = 0; c < n_contacts; ++c) {
    const std::size_t count = SwingIntervals(schedule, c, intervals[c]);
    if (count > kMaxSwingsPerContact) {
      return ReferenceError::kSwingCapacity;
    }
    n_swings[c] = count;
    for (std::size_t i = 0; i < count; ++i) {
      SwingInterval iv = intervals[c][i];
      if (iv.open_start) {
        double t_lo = 0.0;
        bool clamped = false;
        if (scheduler.SwingStart(c, schedule.t0, t_lo, clamped) && !clamped) {
          iv.lift_off = t_lo;
          iv.open_start = false;
        }
      }
      if (BuildSwing(iv, swing_, foot[c].terrain, swings_[c][i]) != SwingError::kNoError) {
        return ReferenceError::kSwingFailed;
      }
    }
  }

  // Base knots (file comment).
  const Eigen::Matrix3d R = quat.toRotationMatrix();
  const double psi0 = Yaw(R);
  const Eigen::Vector3d v_lin(snapshot.v[0], snapshot.v[1], snapshot.v[2]);
  const Eigen::Vector3d omega(snapshot.v[3], snapshot.v[4], snapshot.v[5]);
  const Eigen::Vector3d v_m = R * v_lin;
  const double w_m = (R * omega)[2];
  const double vx = cmd.valid ? cmd.v_x : 0.0;
  const double vy = cmd.valid ? cmd.v_y : 0.0;
  const double wz = cmd.valid ? cmd.omega_z : 0.0;
  const double c0 = std::cos(psi0);
  const double s0 = std::sin(psi0);
  const double vcx = c0 * vx - s0 * vy;
  const double vcy = s0 * vx + c0 * vy;
  const double f = fraction_;
  const double H = static_cast<double>(n_nodes_ - 1) * dt_;
  const double p1x = p0x + f * H * (v_m[0] + vcx) / 2.0;
  const double p1y = p0y + f * H * (v_m[1] + vcy) / 2.0;
  const double psi1 = psi0 + f * H * (w_m + wz) / 2.0;
  const double p2x = p1x + (1.0 - f) * H * vcx;
  const double p2y = p1y + (1.0 - f) * H * vcy;
  const double psi2 = psi1 + (1.0 - f) * H * wz;
  if (!std::isfinite(p1x) || !std::isfinite(p1y) || !std::isfinite(psi1) || !std::isfinite(p2x) ||
      !std::isfinite(p2y) || !std::isfinite(psi2)) {
    return ReferenceError::kNonFiniteInput;  // finite but huge velocities overflowed a knot
  }

  GaitPhaseInfo phase{};
  const bool phased = !arm_joints_.empty() && scheduler.PhaseInfo(phase);
  const WbLayout& layout = handler_->layout();

  HorizonReference& out = *slots_[1 - active_];
  for (int k = 0; k < n_nodes_; ++k) {
    StageReference& st = out.stages[static_cast<std::size_t>(k)];
    const double tk = static_cast<double>(k) * dt_;
    st.time = schedule.t0 + tk;
    st.mode = modes_[static_cast<std::size_t>(k)];
    st.n_stance = static_cast<std::uint32_t>(std::popcount(st.mode));

    // base
    double px = 0.0, py = 0.0, psi = 0.0;
    if (tk <= f * H) {
      const double alpha = std::min(tk / (f * H), 1.0);
      px = Lerp(p0x, p1x, alpha);
      py = Lerp(p0y, p1y, alpha);
      psi = Lerp(psi0, psi1, alpha);
    } else {
      const double alpha = std::min((tk - f * H) / ((1.0 - f) * H), 1.0);
      px = Lerp(p1x, p2x, alpha);
      py = Lerp(p1y, p2y, alpha);
      psi = Lerp(psi1, psi2, alpha);
    }
    st.q[0] = px;
    st.q[1] = py;
    st.q[2] = cmd.base_height;
    YawQuaternion(psi, &st.q[3]);
    const double cr = std::cos(psi0 - psi);
    const double sr = std::sin(psi0 - psi);
    st.v[0] = cr * vx - sr * vy;
    st.v[1] = sr * vx + cr * vy;
    st.v[2] = 0.0;
    st.v[3] = 0.0;
    st.v[4] = 0.0;
    st.v[5] = wz;

    // joints
    for (int i = 7; i < nq; ++i) {
      st.q[static_cast<std::size_t>(i)] = q_default_[static_cast<std::size_t>(i)];
    }
    for (int i = 6; i < nv; ++i) {
      st.v[static_cast<std::size_t>(i)] = 0.0;
    }
    if (phased && k < n_nodes_ - 1) {
      if (!SingleModeAt(phase, st.time)) {
        const double phi = GaitPhase(phase, st.time);
        const double swing = gain_ * std::sin(2.0 * std::numbers::pi * (phi - phase_offset_)) * vx;
        for (const ArmJoint& a : arm_joints_) {
          double& qj = st.q[static_cast<std::size_t>(a.idx_q)];
          qj = std::clamp(qj + a.sign * swing, a.lower, a.upper);
        }
      }
    }

    // contacts
    st.wrench.fill(0.0);
    for (std::size_t c = 0; c < n_contacts; ++c) {
      const bool stance = (st.mode & (1U << c)) != 0;
      SwingSample sample;
      sample.z = foot[c].terrain;
      if (!stance && n_swings[c] > 0) {
        // The stretch this node is in: the last one starting at or before t_k.
        std::size_t pick = 0;
        for (std::size_t i = 1; i < n_swings[c]; ++i) {
          if (intervals[c][i].lift_off <= st.time) {
            pick = i;
          }
        }
        sample = swings_[c][pick].Evaluate(st.time);
      }
      double* pose = &st.contact_pose[7 * c];
      pose[0] = foot[c].x;
      pose[1] = foot[c].y;
      pose[2] = sample.z;
      YawQuaternion(foot[c].psi, &pose[3]);
      double* twist = &st.contact_twist[6 * c];
      double* acc = &st.contact_acceleration[6 * c];
      for (int i = 0; i < 6; ++i) {
        twist[i] = 0.0;
        acc[i] = 0.0;
      }
      twist[2] = sample.z_dot;
      acc[2] = sample.z_ddot;
      st.impact_proximity[c] = sample.impact;
      if (stance) {
        const auto off = static_cast<std::size_t>(layout.contact_u_offset[c]);
        st.wrench[off + 2] = weight_ / static_cast<double>(st.n_stance);
      }
    }
  }

  out.terrain = terrain_;
  out.t0 = schedule.t0;
  out.dt = dt_;
  out.reference_id = slots_[active_]->reference_id + 1;
  out.schedule_id = schedule.schedule_id;
  out.schedule_epoch = schedule.epoch;
  out.observation_sequence = snapshot.sequence;
  out.command_sequence = cmd.sequence;
  out.n_stages = static_cast<std::uint32_t>(n_nodes_);
  out.n_contacts = static_cast<std::uint32_t>(n_contacts);
  out.model_epoch = snapshot.model_epoch;
  out.current_gait = scheduler.current_gait();
  out.target_gait = scheduler.target_gait();
  out.nq = static_cast<std::uint32_t>(nq);
  out.nv = static_cast<std::uint32_t>(nv);
  out.n_contact_vars = static_cast<std::uint32_t>(layout.n_contact_vars);
  active_ = 1 - active_;
  return ReferenceError::kNoError;
}

}  // namespace rtc::mpc
