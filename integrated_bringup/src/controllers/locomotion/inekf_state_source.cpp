// The inekf measured-state source and its configuration. See
// inekf_state_source.hpp and inekf_source_config.hpp.
#include "integrated_bringup/controllers/locomotion/inekf_state_source.hpp"

#include "rtc_controllers/params/yaml_read.hpp"
#include "rtc_urdf_bridge/free_flyer_root.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace integrated_bringup::locomotion {

namespace {

namespace est = rtc::estimation;
using rtc::params::FindMissingKey;
using rtc::params::FindUnknownKey;
using rtc::params::Spelling;

[[noreturn]] void Refuse(const std::string& where, const std::string& what) {
  throw std::invalid_argument(where + ": " + what);
}

YAML::Node CheckSection(const YAML::Node& sec, std::initializer_list<const char*> keys,
                        const std::string& where) {
  if (!sec || !sec.IsMap()) {
    Refuse(where, "expected a map, got " + Spelling(sec));
  }
  if (const auto unknown = FindUnknownKey(sec, keys)) {
    Refuse(where, "unknown key '" + *unknown + "'");
  }
  if (const auto missing = FindMissingKey(sec, keys)) {
    Refuse(where, "required key '" + *missing + "' is missing");
  }
  return sec;
}

double ReadNumber(const YAML::Node& map, const char* key, const std::string& where) {
  const YAML::Node v = map[key];
  double x = std::numeric_limits<double>::quiet_NaN();
  try {
    if (v.IsScalar()) {
      x = v.as<double>();
    }
  } catch (const YAML::Exception&) {
  }
  if (!std::isfinite(x)) {
    Refuse(where + "." + key, "expected a finite number, got " + Spelling(v));
  }
  return x;
}

double ReadPositive(const YAML::Node& map, const char* key, const std::string& where) {
  const double x = ReadNumber(map, key, where);
  if (!(x > 0.0)) {
    Refuse(where + "." + key, "must be > 0, got " + Spelling(map[key]));
  }
  return x;
}

int ReadInt(const YAML::Node& map, const char* key, int lo, const std::string& where) {
  constexpr int kHi = 1'000'000;
  const YAML::Node v = map[key];
  long long n = 0;
  bool ok = false;
  try {
    if (v.IsScalar()) {
      n = v.as<long long>();
      ok = true;
    }
  } catch (const YAML::Exception&) {
  }
  if (!ok || n < lo || n > kHi) {
    Refuse(where + "." + key, "expected an integer in [" + std::to_string(lo) + ", " +
                                  std::to_string(kHi) + "], got " + Spelling(v));
  }
  return static_cast<int>(n);
}

// ⌈d / T⌉ ≥ 1, with the FSM's 1e-12 relative allowance so d = mT gives m.
int TicksOf(double d, double period) {
  const double n = std::ceil(d / period * (1.0 - 1e-12));
  return std::max(1, static_cast<int>(std::min(n, 1e9)));
}

bool Finite3(const double* x) noexcept {
  return std::isfinite(x[0]) && std::isfinite(x[1]) && std::isfinite(x[2]);
}

}  // namespace

// ── estimator.inekf ─────────────────────────────────────────────────────────

InekfSourceConfig ParseInekfSourceConfig(const YAML::Node& inekf, const std::string& context) {
  CheckSection(inekf, {"imu_frame", "imu", "core", "fsm", "init", "policy"}, context);
  InekfSourceConfig cfg;
  const YAML::Node frame = inekf["imu_frame"];
  if (!frame.IsScalar() || frame.Scalar().empty()) {
    Refuse(context + ".imu_frame", "expected a frame name, got " + Spelling(frame));
  }
  cfg.imu_frame = frame.Scalar();

  const std::string imu_at = context + ".imu";
  const YAML::Node imu = CheckSection(
      inekf["imu"],
      {"sample_period_s", "gyro_sigma", "accel_sigma", "gyro_bias_walk", "accel_bias_walk"},
      imu_at);
  cfg.imu.sample_period_s = ReadPositive(imu, "sample_period_s", imu_at);
  cfg.imu.gyro_sigma = ReadPositive(imu, "gyro_sigma", imu_at);
  cfg.imu.accel_sigma = ReadPositive(imu, "accel_sigma", imu_at);
  cfg.imu.gyro_bias_walk = ReadPositive(imu, "gyro_bias_walk", imu_at);
  cfg.imu.accel_bias_walk = ReadPositive(imu, "accel_bias_walk", imu_at);

  const std::string core_at = context + ".core";
  const YAML::Node core =
      CheckSection(inekf["core"],
                   {"gravity", "contact_noise_density", "joint_position_sigma", "kinematic_sigma",
                    "contact_gate_chi2", "prior_velocity_sigma", "prior_position_sigma",
                    "prior_yaw_sigma", "prior_gyro_bias_sigma", "prior_accel_bias_sigma",
                    "stationary_accel_tolerance", "stationary_gyro_tolerance"},
                   core_at);
  auto& c = cfg.core;
  c.gravity = ReadPositive(core, "gravity", core_at);
  // A per-sample σ of a sensor sampled every T is the continuous density σ·√T.
  const double root_t = std::sqrt(cfg.imu.sample_period_s);
  c.gyro_noise_density = cfg.imu.gyro_sigma * root_t;
  c.accel_noise_density = cfg.imu.accel_sigma * root_t;
  c.gyro_bias_walk = cfg.imu.gyro_bias_walk;
  c.accel_bias_walk = cfg.imu.accel_bias_walk;
  c.contact_noise_density = ReadPositive(core, "contact_noise_density", core_at);
  c.joint_position_sigma = ReadPositive(core, "joint_position_sigma", core_at);
  c.kinematic_sigma = ReadPositive(core, "kinematic_sigma", core_at);
  c.contact_gate_chi2 = ReadPositive(core, "contact_gate_chi2", core_at);
  c.prior_velocity_sigma = ReadPositive(core, "prior_velocity_sigma", core_at);
  c.prior_position_sigma = ReadPositive(core, "prior_position_sigma", core_at);
  c.prior_yaw_sigma = ReadPositive(core, "prior_yaw_sigma", core_at);
  c.prior_gyro_bias_sigma = ReadPositive(core, "prior_gyro_bias_sigma", core_at);
  c.prior_accel_bias_sigma = ReadPositive(core, "prior_accel_bias_sigma", core_at);
  c.stationary_accel_tolerance = ReadPositive(core, "stationary_accel_tolerance", core_at);
  c.stationary_gyro_tolerance = ReadPositive(core, "stationary_gyro_tolerance", core_at);

  const std::string fsm_at = context + ".fsm";
  const YAML::Node fsm =
      CheckSection(inekf["fsm"],
                   {"enter_speed", "slip_speed", "nis_low", "nis_high", "enter_dwell",
                    "confirm_dwell", "release_dwell", "slip_dwell", "nis_dwell", "recover_dwell",
                    "evidence_time_constant", "unknown_noise_scale"},
                   fsm_at);
  auto& f = cfg.fsm;
  f.period = cfg.imu.sample_period_s;
  f.enter_speed = ReadPositive(fsm, "enter_speed", fsm_at);
  f.slip_speed = ReadPositive(fsm, "slip_speed", fsm_at);
  f.nis_low = ReadPositive(fsm, "nis_low", fsm_at);
  f.nis_high = ReadPositive(fsm, "nis_high", fsm_at);
  f.enter_dwell = ReadPositive(fsm, "enter_dwell", fsm_at);
  f.confirm_dwell = ReadPositive(fsm, "confirm_dwell", fsm_at);
  f.release_dwell = ReadPositive(fsm, "release_dwell", fsm_at);
  f.slip_dwell = ReadPositive(fsm, "slip_dwell", fsm_at);
  f.nis_dwell = ReadPositive(fsm, "nis_dwell", fsm_at);
  f.recover_dwell = ReadPositive(fsm, "recover_dwell", fsm_at);
  f.evidence_time_constant = ReadPositive(fsm, "evidence_time_constant", fsm_at);
  f.unknown_noise_scale = ReadPositive(fsm, "unknown_noise_scale", fsm_at);
  if (!(f.enter_speed < f.slip_speed)) {
    Refuse(fsm_at, "enter_speed must be below slip_speed");
  }
  if (!(f.nis_low < f.nis_high)) {
    Refuse(fsm_at, "nis_low must be below nis_high");
  }
  if (!(f.unknown_noise_scale >= 1.0)) {
    Refuse(fsm_at + ".unknown_noise_scale", "must be >= 1");
  }

  const std::string init_at = context + ".init";
  const YAML::Node init = CheckSection(
      inekf["init"], {"window_ticks", "timeout_s", "yaw", "joint_speed_tolerance"}, init_at);
  cfg.init.window_ticks = ReadInt(init, "window_ticks", 1, init_at);
  cfg.init.timeout_s = ReadPositive(init, "timeout_s", init_at);
  cfg.init.yaw = ReadNumber(init, "yaw", init_at);
  cfg.init.joint_speed_tolerance = ReadPositive(init, "joint_speed_tolerance", init_at);

  const std::string policy_at = context + ".policy";
  const YAML::Node policy =
      CheckSection(inekf["policy"],
                   {"max_hold_ticks", "max_joint_imu_skew_ticks", "max_skew_invalid_ticks",
                    "max_no_anchor_s", "max_gated_s", "max_refused_ticks"},
                   policy_at);
  auto& p = cfg.policy;
  p.max_hold_ticks = ReadInt(policy, "max_hold_ticks", 0, policy_at);
  p.max_joint_imu_skew_ticks = ReadInt(policy, "max_joint_imu_skew_ticks", 0, policy_at);
  p.max_skew_invalid_ticks = ReadInt(policy, "max_skew_invalid_ticks", 0, policy_at);
  p.max_no_anchor_s = ReadPositive(policy, "max_no_anchor_s", policy_at);
  p.max_gated_s = ReadPositive(policy, "max_gated_s", policy_at);
  p.max_refused_ticks = ReadInt(policy, "max_refused_ticks", 1, policy_at);
  if (p.max_skew_invalid_ticks < p.max_joint_imu_skew_ticks) {
    Refuse(policy_at, "max_skew_invalid_ticks must be >= max_joint_imu_skew_ticks");
  }

  // The core and FSM keep the rest of their contracts (step range, dwell bounds).
  try {
    est::ContactInekfSequencer probe;
    probe.Init(cfg.core, cfg.fsm, 1);
  } catch (const std::invalid_argument& e) {
    Refuse(context, e.what());
  }
  return cfg;
}

// ── ImuFrameKinematics ──────────────────────────────────────────────────────

ImuFrameKinematics::ImuFrameKinematics(std::shared_ptr<const pinocchio::Model> model,
                                       const std::vector<std::string>& contact_frames,
                                       const std::string& imu_frame)
    : handle_((model && rtc_urdf_bridge::IsFreeFlyerRoot(*model))
                  ? model
                  : throw std::invalid_argument(
                        "ImuFrameKinematics: the model has no free-flyer root")) {
  const pinocchio::Model& m = handle_.GetModel();
  nj_ = m.nv - 6;
  if (nj_ > est::kInekfMaxJacobianCols) {
    throw std::invalid_argument("ImuFrameKinematics: " + std::to_string(nj_) +
                                " joints, more than a contact Jacobian holds (" +
                                std::to_string(est::kInekfMaxJacobianCols) + ")");
  }
  if (contact_frames.empty() ||
      contact_frames.size() > static_cast<std::size_t>(est::kInekfMaxContacts)) {
    throw std::invalid_argument("ImuFrameKinematics: " + std::to_string(contact_frames.size()) +
                                " contacts; the estimator takes 1 to " +
                                std::to_string(est::kInekfMaxContacts));
  }
  for (const auto& name : contact_frames) {
    if (!m.existFrame(name)) {
      throw std::invalid_argument("ImuFrameKinematics: no frame '" + name + "' in the model");
    }
    frames_.push_back(m.getFrameId(name));
  }
  if (!m.existFrame(imu_frame)) {
    throw std::invalid_argument("ImuFrameKinematics: no IMU frame '" + imu_frame +
                                "' in the model");
  }
  const pinocchio::FrameIndex imu = m.getFrameId(imu_frame);
  // Joint 1 is the free-flyer root: a frame on it is rigid on the base.
  if (m.frames[imu].parentJoint != 1) {
    throw std::invalid_argument("ImuFrameKinematics: the IMU frame '" + imu_frame +
                                "' is on joint '" + m.names[m.frames[imu].parentJoint] +
                                "', not on the floating-base root — its pose would move "
                                "with the joints");
  }
  q_ = pinocchio::neutral(m);
  J_ = Eigen::MatrixXd::Zero(6, m.nv);
  handle_.ComputeForwardKinematics(
      std::span<const double>(q_.data(), static_cast<std::size_t>(q_.size())));
  R_BI_ = handle_.GetFrameRotation(imu);
  p_BI_ = handle_.GetFramePosition(imu);
  R_IB_ = R_BI_.transpose();
}

void ImuFrameKinematics::Compute(std::span<const double> q, std::span<const double> qd,
                                 std::span<est::ContactKinematics> kin,
                                 std::span<Eigen::Vector3d> rdot) noexcept {
  // The base stays at its neutral pose: B axes are the world axes of the model.
  for (int j = 0; j < nj_; ++j) {
    q_(7 + j) = q[static_cast<std::size_t>(j)];
  }
  handle_.ComputeJacobians(std::span<const double>(q_.data(), static_cast<std::size_t>(q_.size())));
  const Eigen::Map<const Eigen::VectorXd> qd_map(qd.data(), nj_);
  for (std::size_t c = 0; c < frames_.size(); ++c) {
    handle_.GetFrameJacobian(frames_[c], pinocchio::LOCAL_WORLD_ALIGNED, J_);
    const Eigen::Vector3d p_Bc = handle_.GetFramePosition(frames_[c]);
    kin[c].position.noalias() = R_IB_ * (p_Bc - p_BI_);
    kin[c].jacobian.resize(3, nj_);
    kin[c].jacobian.noalias() = R_IB_.lazyProduct(J_.block(0, 6, 3, nj_));
    rdot[c].noalias() = kin[c].jacobian.lazyProduct(qd_map);
  }
}

// ── InekfStateSource ────────────────────────────────────────────────────────

InekfStateSource::InekfStateSource(const Params& params,
                                   std::unique_ptr<ImuFrameKinematics> kinematics,
                                   std::vector<int> body_to_full)
    : params_(params), kin_model_(std::move(kinematics)), body_to_full_(std::move(body_to_full)) {
  if (params_.body_max_age_ns <= 0 || params_.hand_max_age_ns <= 0) {
    throw std::invalid_argument("InekfStateSource: the age limits must be positive");
  }
  if (params_.imu_site < 0 || params_.imu_site >= rtc::kMaxImuSites) {
    throw std::invalid_argument("InekfStateSource: IMU site " + std::to_string(params_.imu_site) +
                                " is out of range");
  }
  if (!kin_model_) {
    throw std::invalid_argument("InekfStateSource: no kinematics");
  }
  if (static_cast<int>(body_to_full_.size()) != kin_model_->num_joints() ||
      body_to_full_.size() > kMaxStateJoints) {
    throw std::invalid_argument("InekfStateSource: " + std::to_string(body_to_full_.size()) +
                                " body joint slots for a model of " +
                                std::to_string(kin_model_->num_joints()));
  }
  for (const int a : body_to_full_) {
    if (a < 0 || a >= static_cast<int>(kMaxStateJoints)) {
      throw std::invalid_argument("InekfStateSource: a body joint slot is out of range");
    }
  }
  const InekfSourceConfig& cfg = params_.config;
  num_slots_ = kin_model_->num_slots();
  seq_.Init(cfg.core, cfg.fsm, num_slots_);
  slot_mask_ = (1U << static_cast<uint32_t>(num_slots_)) - 1U;
  period_s_ = cfg.imu.sample_period_s;
  const double t_ns = period_s_ * 1e9;
  skew_ns_ = std::llround(static_cast<double>(cfg.policy.max_joint_imu_skew_ticks) * t_ns);
  skew_invalid_ns_ = std::llround(static_cast<double>(cfg.policy.max_skew_invalid_ticks) * t_ns);
  gated_limit_ = TicksOf(cfg.policy.max_gated_s, period_s_);
  no_anchor_limit_ = TicksOf(cfg.policy.max_no_anchor_s, period_s_);
  for (auto& k : kin_) {
    k.jacobian.resize(3, kin_model_->num_joints());
    k.jacobian.setZero();
  }
  telemetry_.num_slots = static_cast<uint8_t>(num_slots_);
}

void InekfStateSource::RestartFilter(EstimatorResetReason reason) noexcept {
  seq_.Reset();
  window_.Reset();
  started_ = false;
  hold_ticks_ = 0;
  refused_ticks_ = 0;
  ++state_epoch_;
  telemetry_.reset_reason = static_cast<uint8_t>(reason);
  ++telemetry_.reset_count;
}

void InekfStateSource::Reset() noexcept {
  RestartFilter(EstimatorResetReason::kRequested);
  have_sequence_ = false;
  last_sequence_ = 0;
  held_ = HeldHands{};
}

void InekfStateSource::TryStart(bool kinematics_ok, const Eigen::Vector3d& gyro,
                                const Eigen::Vector3d& accel, uint32_t planned,
                                const MeasuredRobotState& out) noexcept {
  const auto& cfg = params_.config;
  bool still = kinematics_ok && planned != 0 && gyro.norm() <= cfg.core.stationary_gyro_tolerance &&
               std::abs(accel.norm() - cfg.core.gravity) <= cfg.core.stationary_accel_tolerance;
  for (std::size_t j = 0; still && j < body_to_full_.size(); ++j) {
    still = std::abs(out.v[6 + static_cast<std::size_t>(body_to_full_[j])]) <=
            cfg.init.joint_speed_tolerance;
  }
  if (!still || !window_.Add(gyro, accel)) {
    window_.Reset();
    return;
  }
  if (window_.count() < cfg.init.window_ticks) {
    return;
  }
  int n = 0;
  for (int c = 0; c < num_slots_; ++c) {
    if ((planned >> static_cast<uint32_t>(c) & 1U) != 0) {
      stance_[static_cast<std::size_t>(n++)] = kin_[static_cast<std::size_t>(c)].position;
    }
  }
  const est::InekfStatus st = seq_.Initialize(
      window_.MeanGyro(), window_.MeanAccel(), static_cast<double>(window_.count()) * period_s_,
      cfg.init.yaw, std::span<const Eigen::Vector3d>(stance_.data(), static_cast<std::size_t>(n)));
  window_.Reset();
  if (st == est::InekfStatus::kOk) {
    started_ = true;
  } else {
    telemetry_.core_status = static_cast<uint8_t>(st);
  }
}

EstimatorResetReason InekfStateSource::StepPolicy(est::ContactFsmStatus st) noexcept {
  if (st == est::ContactFsmStatus::kOk) {
    refused_ticks_ = 0;
  } else if (st != est::ContactFsmStatus::kStateMismatch) {
    // kPropagateRefused; kUpdateRefused (the FSM filters every input the update
    // would refuse — a defensive count); any other refusal alike.
    ++refused_ticks_;
  }
  if (st == est::ContactFsmStatus::kStateMismatch) {
    return EstimatorResetReason::kStateMismatch;
  }
  if (refused_ticks_ >= params_.config.policy.max_refused_ticks) {
    return EstimatorResetReason::kRefused;
  }
  if (seq_.consecutive_gated_ticks() >= gated_limit_) {
    return EstimatorResetReason::kGated;
  }
  if (seq_.ticks_without_confirmed_anchor() >= no_anchor_limit_) {
    return EstimatorResetReason::kNoAnchor;
  }
  return EstimatorResetReason::kNone;
}

bool InekfStateSource::Update(const rtc::ControllerState& state, const DeviceIndices& indices,
                              const SourceTickInput& tick, int64_t now_steady_ns,
                              MeasuredRobotState& out) noexcept {
  const auto n = static_cast<std::size_t>(
      std::clamp(indices.num_joints, 0, static_cast<int>(kMaxStateJoints)));
  out.nq = 7 + static_cast<int>(n);
  out.nv = 6 + static_cast<int>(n);
  out.kind = StateSourceKind::kEstimated;
  out.model_epoch = params_.model_epoch;
  out.ee_wrench = {};
  out.fresh_mask = 0;
  out.valid_mask = 0;
  out.stamp_steady_ns = 0;
  out.sequence = 0;
  out.q.fill(0.0);
  out.q[6] = 1.0;
  out.v.fill(0.0);

  const JointLaneVerdict joints = ReadJointLanes(
      state, indices, now_steady_ns, params_.body_max_age_ns, params_.hand_max_age_ns, held_, out);
  const bool joints_ok = !joints.body_unreadable && !joints.non_finite;
  EstimatorTelemetry& tel = telemetry_;
  tel.periods = -1;
  tel.step_status = 0;
  tel.skew_ns = 0;
  tel.imu = {};
  const uint32_t planned = tick.planned_mask & slot_mask_;
  tel.planned_mask = planned;

  // ── the primary IMU site ──────────────────────────────────────────────────
  bool imu_missing = true;
  bool regressed = false;
  uint64_t sequence = 0;
  int64_t imu_recv = 0;
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
  if (joints.body_present) {
    const auto& dev = state.devices[static_cast<std::size_t>(indices.body.device)];
    const auto s = static_cast<std::size_t>(params_.imu_site);
    if (params_.imu_site < dev.num_imu_sites) {
      const auto& sample = dev.imu[s];
      for (std::size_t i = 0; i < 6; ++i) {
        tel.imu[i] = sample[i];
      }
      imu_recv = dev.imu_recv_steady_ns[s];
      sequence = dev.imu_sequence[s];
      imu_missing =
          !dev.imu_valid[s] || imu_recv <= 0 || !Finite3(&sample[0]) || !Finite3(&sample[3]);
      gyro = Eigen::Vector3d(sample[0], sample[1], sample[2]);
      accel = Eigen::Vector3d(sample[3], sample[4], sample[5]);
    }
  }
  tel.imu_sequence = sequence;
  if (!imu_missing && have_sequence_ && sequence < last_sequence_) {
    regressed = true;
    imu_missing = true;
  }

  // ── kinematics and the joint / IMU coherence ──────────────────────────────
  bool skew_invalid = false;
  bool kinematics_ok = joints_ok;
  if (!imu_missing && joints.body_present) {
    tel.skew_ns = out.stamp_steady_ns - imu_recv;
    const int64_t skew = tel.skew_ns < 0 ? -tel.skew_ns : tel.skew_ns;
    kinematics_ok = kinematics_ok && skew <= skew_ns_;
    skew_invalid = skew > skew_invalid_ns_;
  }
  const auto ns = static_cast<std::size_t>(num_slots_);
  if (kinematics_ok) {
    for (std::size_t j = 0; j < body_to_full_.size(); ++j) {
      const auto a = static_cast<std::size_t>(body_to_full_[j]);
      q_body_[j] = out.q[7 + a];
      qd_body_[j] = out.v[6 + a];
    }
    kin_model_->Compute(std::span<const double>(q_body_.data(), body_to_full_.size()),
                        std::span<const double>(qd_body_.data(), body_to_full_.size()),
                        std::span<est::ContactKinematics>(kin_.data(), ns),
                        std::span<Eigen::Vector3d>(rdot_.data(), ns));
  } else {
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t c = 0; c < ns; ++c) {
      kin_[c].position.setConstant(kNaN);
      rdot_[c].setConstant(kNaN);
    }
  }

  // ── cadence, start, step, resets ──────────────────────────────────────────
  bool reset_now = false;
  bool advanced = false;
  if (regressed) {
    RestartFilter(EstimatorResetReason::kGap);
    reset_now = true;
    last_sequence_ = sequence;
  } else if (!imu_missing) {
    const uint64_t delta = have_sequence_ ? sequence - last_sequence_ : 1U;
    have_sequence_ = true;
    last_sequence_ = sequence;
    // The core's own bound on the nominal step, in its own expression.
    const bool gap =
        delta > 1'000'000U || static_cast<double>(delta) * period_s_ > est::kMaxPropagationStep;
    if (delta == 0) {
      tel.periods = 0;
      if (started_) {
        ++hold_ticks_;
      }
    } else if (!started_) {
      if (gap) {
        window_.Reset();
      } else {
        TryStart(kinematics_ok, gyro, accel, planned, out);
        advanced = started_;
      }
    } else if (gap) {
      RestartFilter(EstimatorResetReason::kGap);
      reset_now = true;
    } else {
      hold_ticks_ = 0;
      const auto k = static_cast<int>(delta);
      tel.periods = k;
      est::ContactStepInput in;
      in.gyro = gyro;
      in.accel = accel;
      in.kinematics = std::span<const est::ContactKinematics>(kin_.data(), ns);
      in.joint_velocity = std::span<const Eigen::Vector3d>(rdot_.data(), ns);
      in.planned_mask = planned;
      in.periods = k;
      const est::ContactFsmStatus st = seq_.Step(in);
      tel.step_status = static_cast<uint8_t>(st);
      if (seq_.core_status() != est::InekfStatus::kOk) {
        tel.core_status = static_cast<uint8_t>(seq_.core_status());
      }
      advanced = st != est::ContactFsmStatus::kPropagateRefused;
      const EstimatorResetReason why = StepPolicy(st);
      if (why != EstimatorResetReason::kNone) {
        RestartFilter(why);
        reset_now = true;
        advanced = false;
      }
    }
  }

  // ── the base ──────────────────────────────────────────────────────────────
  bool imu_stale = false;
  bool non_finite = joints.non_finite;
  if (started_ && !imu_missing && !reset_now) {
    imu_stale = hold_ticks_ > static_cast<uint32_t>(params_.config.policy.max_hold_ticks);
    if (!imu_stale) {
      const est::ContactInekf& f = seq_.core();
      const est::BaseState b = est::ContactInekf::BaseFromImu(
          f.rotation(), f.position(), f.velocity(), f.angular_velocity(), kin_model_->R_BI(),
          kin_model_->p_BI());
      Eigen::Quaterniond quat(b.R_WB);
      quat.normalize();
      const Eigen::Vector3d v_local = b.R_WB.transpose() * b.v_WB;
      const Eigen::Vector3d w_local = kin_model_->R_BI() * f.angular_velocity();
      const std::array<double, 7> qb = {b.p_WB.x(), b.p_WB.y(), b.p_WB.z(), quat.x(),
                                        quat.y(),   quat.z(),   quat.w()};
      const std::array<double, 6> vb = {v_local.x(), v_local.y(), v_local.z(),
                                        w_local.x(), w_local.y(), w_local.z()};
      bool finite = true;
      for (double x : qb) {
        finite = finite && std::isfinite(x);
      }
      for (double x : vb) {
        finite = finite && std::isfinite(x);
      }
      if (!finite) {
        non_finite = true;
      } else {
        std::copy(qb.begin(), qb.end(), out.q.begin());
        std::copy(vb.begin(), vb.end(), out.v.begin());
        out.valid_mask |= kBaseBit;
        if (advanced) {
          out.fresh_mask |= kBaseBit;
        }
      }
    }
  }
  if (!imu_missing) {
    out.stamp_steady_ns = joints.body_present ? std::min(out.stamp_steady_ns, imu_recv) : imu_recv;
  }
  out.state_epoch = state_epoch_;
  FillTelemetry(out);
  out.contact_mask = tel.estimated_mask;

  StateSourceStatus status = StateSourceStatus::kOk;
  if (joints.body_unreadable) {
    status = StateSourceStatus::kBodyUnreadable;
  } else if (joints.body_stale) {
    status = StateSourceStatus::kBodyStale;
  } else if (imu_missing) {
    status = StateSourceStatus::kImuMissing;
  } else if (imu_stale) {
    status = StateSourceStatus::kImuStale;
  } else if (skew_invalid) {
    status = StateSourceStatus::kImuSkew;
  } else if (!started_ || reset_now) {
    status = StateSourceStatus::kNotInitialized;
  } else if (non_finite) {
    status = StateSourceStatus::kNonFinite;
  }
  out.status = status;
  out.valid = status == StateSourceStatus::kOk;
  return out.valid;
}

void InekfStateSource::FillTelemetry(MeasuredRobotState& out) noexcept {
  EstimatorTelemetry& tel = telemetry_;
  tel.initialized = started_;
  tel.hold_ticks = hold_ticks_;
  tel.refused_ticks = refused_ticks_;
  tel.report_valid = started_ && seq_.report_valid();
  tel.no_anchor_ticks = seq_.ticks_without_confirmed_anchor();
  tel.gated_ticks = seq_.consecutive_gated_ticks();
  tel.estimated_mask = 0;
  const est::ContactFsm& fsm = seq_.fsm();
  for (int c = 0; c < num_slots_; ++c) {
    const auto uc = static_cast<std::size_t>(c);
    const est::ContactMode m = fsm.mode(c);
    tel.mode[uc] = static_cast<uint8_t>(m);
    tel.reason[uc] = static_cast<uint8_t>(fsm.reason(c));
    tel.evidence[uc] = fsm.evidence(c);
    tel.speed[uc] = fsm.tested_speed(c);
    const est::ContactUpdateReport& r = seq_.report(c);
    tel.update[uc] = tel.report_valid ? static_cast<uint8_t>(r.result) : 0;
    tel.m2[uc] = tel.report_valid ? r.mahalanobis2 : 0.0;
    if (est::ContactFsm::Confirmed(m)) {
      tel.estimated_mask |= 1U << static_cast<uint32_t>(c);
    }
  }
  tel.p_diag = {};
  tel.unobservable = 0;
  if (started_) {
    const auto& P = seq_.core().covariance();
    const auto dim = static_cast<std::size_t>(seq_.core().dim());
    for (std::size_t i = 0; i < std::min(dim, kMaxEstimatorCovDiag); ++i) {
      tel.p_diag[i] = P(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i));
    }
    tel.unobservable = kYawUnobservable | kPositionUnobservable;
  }
  out.estimator = tel;
}

}  // namespace integrated_bringup::locomotion
