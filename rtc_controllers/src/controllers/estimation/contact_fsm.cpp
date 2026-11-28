#include "rtc_controllers/estimation/contact_fsm.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace rtc::estimation {

namespace {

// Streak / counter ceiling: far above any dwell (≤ 10⁶ periods), far below
// INT_MAX, so a saturating add never overflows.
constexpr int kCounterCap = 1 << 30;
constexpr double kMaxDwellPeriods = 1e6;

int SaturatingAdd(int a, int b) noexcept {
  return static_cast<int>(
      std::min<long long>(kCounterCap, static_cast<long long>(a) + static_cast<long long>(b)));
}

static_assert(kInekfMaxContacts <= 32, "planned_mask is a uint32_t bit per slot");

void RequirePositive(double v, const char* name) {
  if (!std::isfinite(v) || !(v > 0.0)) {
    throw std::invalid_argument(std::string("ContactFsm::Config::") + name +
                                " must be finite and > 0");
  }
}

int DwellTicks(double dwell, double period, const char* name) {
  RequirePositive(dwell, name);
  const double ratio = dwell / period;
  if (!(ratio <= kMaxDwellPeriods)) {
    throw std::invalid_argument(std::string("ContactFsm::Config::") + name +
                                " exceeds 1e6 periods");
  }
  // d = mT must give m: the quotient of two decimal fractions lands a few
  // ulp either side of the integer.
  const double n = std::ceil(ratio * (1.0 - 1e-12));
  return std::max(1, static_cast<int>(n));
}

double Clamp01(double x) noexcept {
  return std::clamp(x, 0.0, 1.0);
}

}  // namespace

// ── ContactFsm ────────────────────────────────────────────────────────────────

void ContactFsm::Init(const Config& config, int num_slots) {
  if (num_slots < 1 || num_slots > kInekfMaxContacts) {
    throw std::invalid_argument("ContactFsm::Init: num_slots outside [1, kInekfMaxContacts]");
  }
  RequirePositive(config.period, "period");
  if (config.period < kMinPropagationStep || config.period > kMaxPropagationStep) {
    throw std::invalid_argument("ContactFsm::Config::period outside the core's step range");
  }
  RequirePositive(config.enter_speed, "enter_speed");
  RequirePositive(config.slip_speed, "slip_speed");
  RequirePositive(config.nis_low, "nis_low");
  RequirePositive(config.nis_high, "nis_high");
  RequirePositive(config.evidence_time_constant, "evidence_time_constant");
  RequirePositive(config.unknown_noise_scale, "unknown_noise_scale");
  if (!(config.enter_speed < config.slip_speed)) {
    throw std::invalid_argument("ContactFsm::Config: enter_speed must be < slip_speed");
  }
  if (!(config.nis_low < config.nis_high)) {
    throw std::invalid_argument("ContactFsm::Config: nis_low must be < nis_high");
  }
  if (!(config.unknown_noise_scale >= 1.0)) {
    throw std::invalid_argument("ContactFsm::Config::unknown_noise_scale must be >= 1");
  }
  Dwells n;
  n.enter = DwellTicks(config.enter_dwell, config.period, "enter_dwell");
  n.confirm = DwellTicks(config.confirm_dwell, config.period, "confirm_dwell");
  n.release = DwellTicks(config.release_dwell, config.period, "release_dwell");
  n.slip = DwellTicks(config.slip_dwell, config.period, "slip_dwell");
  n.nis = DwellTicks(config.nis_dwell, config.period, "nis_dwell");
  n.recover = DwellTicks(config.recover_dwell, config.period, "recover_dwell");

  config_ = config;
  num_slots_ = num_slots;
  n_ = n;
  Reset();
}

void ContactFsm::Reset() noexcept {
  for (Slot& s : slots_) {
    s = Slot{};
  }
}

bool ContactFsm::Meets(const Streak& s, int n) noexcept {
  return s.ticks >= n && s.samples >= std::min(n, 2);
}

void ContactFsm::ClearSignalStreaks(Slot& s) noexcept {
  s.low_speed = {};
  s.high_speed = {};
  s.nis_high = {};
  s.nis_low = {};
}

void ContactFsm::Enter(Slot& s, ContactMode to, ContactReason why) noexcept {
  s.mode = to;
  s.reason = why;
  s.release = {};
  ClearSignalStreaks(s);
  s.age = 0;
  s.last = Stored::kNone;
}

void ContactFsm::Decide(Slot& s, bool planned, bool valid, int k, SlotAction& a) noexcept {
  a = SlotAction{};
  auto extend = [k](Streak& st, bool on) noexcept {
    if (on) {
      st.ticks = SaturatingAdd(st.ticks, k);
      st.samples = SaturatingAdd(st.samples, 1);
    } else {
      st = {};
    }
  };
  // Release outranks every other row of every mode, valid input or not.
  auto release = [&]() noexcept {
    a.remove = HoldsAnchor(s.mode);
    Enter(s, ContactMode::kSwing, ContactReason::kReleased);
  };
  extend(s.release, !planned);
  const bool released = Meets(s.release, n_.release) && s.mode != ContactMode::kSwing;
  if (HoldsAnchor(s.mode)) {
    s.age = SaturatingAdd(s.age, k);  // time, so it runs on invalid samples too
  }

  if (!valid) {
    ClearSignalStreaks(s);
    if (released) {
      release();
    }
    return;  // use stays false; evidence held
  }

  extend(s.low_speed, s.speed <= config_.enter_speed);
  extend(s.high_speed, s.speed > config_.slip_speed);
  if (s.last != Stored::kNone) {  // no report: the NIS streaks hold
    extend(s.nis_high, s.last == Stored::kHigh);
    extend(s.nis_low, s.last == Stored::kLow);
  }

  // Evidence, from the inputs this tick decides on.
  const double g =
      Clamp01((config_.slip_speed - s.speed) / (config_.slip_speed - config_.enter_speed));
  double h = 1.0;
  if (s.last == Stored::kHigh) {
    h = 0.0;
  } else if (s.last != Stored::kNone) {
    h = Clamp01((config_.nis_high - s.last_m2) / (config_.nis_high - config_.nis_low));
  }
  const double q = (planned ? 1.0 : 0.0) * g * h;
  const double alpha =
      -std::expm1(-static_cast<double>(k) * config_.period / config_.evidence_time_constant);
  s.evidence = Clamp01(s.evidence + alpha * (q - s.evidence));

  if (released) {
    release();
    return;
  }
  const bool slow_enter = planned && Meets(s.low_speed, n_.enter);
  const bool fast = Meets(s.high_speed, n_.slip);
  switch (s.mode) {
    case ContactMode::kSwing:
    case ContactMode::kSlip:
      if (slow_enter) {
        a.insert = true;
        Enter(s, ContactMode::kCandidate, ContactReason::kTouchdown);
      }
      break;
    case ContactMode::kCandidate:
      if (fast) {
        a.remove = true;
        Enter(s, ContactMode::kSwing, ContactReason::kRejectedSpeed);
      } else if (s.last == Stored::kHigh) {
        a.remove = true;
        Enter(s, ContactMode::kSwing, ContactReason::kRejectedNis);
      } else if (s.age >= std::max(n_.confirm, 2) && Meets(s.low_speed, n_.confirm) &&
                 s.last == Stored::kLow) {
        Enter(s, ContactMode::kStance, ContactReason::kConfirmed);
      }
      break;
    case ContactMode::kStance:
    case ContactMode::kUnknown:
      if (fast) {
        a.remove = true;
        Enter(s, ContactMode::kSlip, ContactReason::kSlip);
      } else if (s.mode == ContactMode::kStance && Meets(s.nis_high, n_.nis)) {
        a.set_scale = true;
        a.noise_scale = config_.unknown_noise_scale;
        Enter(s, ContactMode::kUnknown, ContactReason::kNisHigh);
      } else if (s.mode == ContactMode::kUnknown && Meets(s.nis_low, n_.recover) &&
                 Meets(s.low_speed, n_.recover)) {
        a.set_scale = true;
        a.noise_scale = 1.0;
        Enter(s, ContactMode::kStance, ContactReason::kRecovered);
      }
      break;
  }
  a.use = HoldsAnchor(s.mode);
}

ContactFsmStatus ContactFsm::PreUpdate(int periods, std::uint32_t planned_mask,
                                       std::span<const Eigen::Vector3d> contact_velocity,
                                       std::span<SlotAction> actions) noexcept {
  if (!configured()) {
    return ContactFsmStatus::kNotConfigured;
  }
  if (periods < 1) {
    return ContactFsmStatus::kBadPeriods;
  }
  const auto n = static_cast<std::size_t>(num_slots_);
  if (contact_velocity.size() != n || actions.size() != n) {
    return ContactFsmStatus::kSizeMismatch;
  }
  bool any_confirmed = false;
  std::array<bool, kInekfMaxContacts> valid{};
  std::array<bool, kInekfMaxContacts> planned{};
  for (std::size_t i = 0; i < n; ++i) {
    any_confirmed = any_confirmed || Confirmed(slots_[i].mode);
    valid[i] = contact_velocity[i].allFinite();
    planned[i] = ((planned_mask >> i) & 1U) != 0U;
  }
  for (std::size_t i = 0; i < n; ++i) {
    Slot& s = slots_[i];
    if (!valid[i]) {
      s.speed = std::numeric_limits<double>::quiet_NaN();
    } else if (any_confirmed) {
      s.speed = contact_velocity[i].norm();
    } else {
      // No confirmed anchor: v̂ is dead reckoning, so test the spread between
      // planned slots, in which its error cancels.
      double spread = 0.0;
      for (std::size_t j = 0; j < n; ++j) {
        if (j != i && planned[j] && valid[j]) {
          spread = std::max(spread, (contact_velocity[i] - contact_velocity[j]).norm());
        }
      }
      s.speed = spread;
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    // A finite u_i gives a finite or +inf speed (inf reads as fast); never NaN.
    Decide(slots_[i], planned[i], valid[i] && !std::isnan(slots_[i].speed), periods, actions[i]);
  }
  return ContactFsmStatus::kOk;
}

void ContactFsm::RevertInsert(int slot) noexcept {
  if (slot < 0 || slot >= num_slots_) {
    return;
  }
  Slot& s = slots_[static_cast<std::size_t>(slot)];
  if (s.mode == ContactMode::kCandidate) {
    Enter(s, ContactMode::kSwing, ContactReason::kInsertRefused);
  }
}

void ContactFsm::PostUpdate(std::span<const ContactUpdateReport> report, bool valid) noexcept {
  const auto n = static_cast<std::size_t>(num_slots_);
  const bool usable = valid && report.size() == n;
  for (std::size_t i = 0; i < n; ++i) {
    Slot& s = slots_[i];
    s.last = Stored::kNone;
    if (!usable) {
      continue;
    }
    const ContactUpdateReport& r = report[i];
    const double m2 = r.mahalanobis2;
    const bool gated = r.result == ContactUpdateResult::kGated;
    const bool used = r.result == ContactUpdateResult::kUsed;
    if (gated || (used && !std::isfinite(m2))) {  // a used NIS that is not finite is out of family
      s.last = Stored::kHigh;
      s.last_m2 = std::isfinite(m2) ? m2 : 0.0;
    } else if (used) {
      s.last = m2 <= config_.nis_low   ? Stored::kLow
               : m2 > config_.nis_high ? Stored::kHigh
                                       : Stored::kNeutral;
      s.last_m2 = m2;
    }
  }
}

// ── ContactInekfSequencer ─────────────────────────────────────────────────────

void ContactInekfSequencer::Init(const ContactInekf::Config& core, const ContactFsm::Config& fsm,
                                 int num_slots) {
  ContactFsm checked;
  checked.Init(fsm, num_slots);
  const double sd = fsm.unknown_noise_scale * core.contact_noise_density;
  if (!std::isfinite(sd * sd)) {
    throw std::invalid_argument(
        "ContactInekfSequencer::Init: (unknown_noise_scale * contact_noise_density)^2 not finite");
  }
  core_.Init(core, num_slots);
  fsm_ = checked;
  ground_ = GroundConfig{};
  settle_ticks_ = 1;
  Reset();
}

void ContactInekfSequencer::Init(const ContactInekf::Config& core, const ContactFsm::Config& fsm,
                                 int num_slots, const GroundConfig& ground) {
  if (ground.enabled) {
    ContactFsm checked;  // the FSM first: settle_s is counted in its periods
    checked.Init(fsm, num_slots);
    const double var = ground.sigma * ground.sigma;
    const double periods = ground.settle_s / fsm.period;
    if (!std::isfinite(ground.height) || !std::isfinite(ground.sigma) || !(ground.sigma > 0.0) ||
        !std::isfinite(var) || !std::isnormal(var) || !(ground.gate_chi2 > 0.0) ||
        ground.max_gated < 1 || !(ground.settle_s > 0.0) || !(periods <= 1e6)) {
      throw std::invalid_argument(
          "ContactInekfSequencer::Init: ground needs a finite height, sigma > 0 with a finite "
          "normal square, gate_chi2 > 0, max_gated >= 1 and 0 < settle_s <= 1e6 periods");
    }
  }
  Init(core, fsm, num_slots);
  ground_ = ground;
  if (ground.enabled) {
    // The dwells' rounding: ⌈d / T⌉ with a 1e-12 relative allowance, at least 1.
    settle_ticks_ =
        std::max(1, static_cast<int>(std::ceil(ground.settle_s / fsm.period * (1.0 - 1e-12))));
  }
}

void ContactInekfSequencer::Reset() noexcept {
  core_.Reset();
  fsm_.Reset();
  core_status_ = InekfStatus::kOk;
  report_valid_ = false;
  no_anchor_ticks_ = 0;
  gated_ticks_ = 0;
  gated_pins_ = 0;
  for (std::size_t i = 0; i < kInekfMaxContacts; ++i) {
    report_[i] = ContactUpdateReport{};
    ground_report_[i] = GroundUpdateReport{};
    anchor_age_[i] = 0;
    pinned_[i] = false;
    actions_[i] = ContactFsm::SlotAction{};
    velocity_[i].setZero();
    kin_[i].use = false;
  }
}

InekfStatus ContactInekfSequencer::Initialize(
    const Eigen::Vector3d& mean_gyro, const Eigen::Vector3d& mean_accel, double window_duration,
    double yaw, std::span<const Eigen::Vector3d> stance_points) noexcept {
  if (!fsm_.configured()) {
    return InekfStatus::kNotConfigured;
  }
  return core_.Initialize(mean_gyro, mean_accel, window_duration, yaw, stance_points);
}

const ContactUpdateReport& ContactInekfSequencer::report(int slot) const noexcept {
  return report_[Index(slot)];
}

const GroundUpdateReport& ContactInekfSequencer::ground_report(int slot) const noexcept {
  return ground_report_[Index(slot)];
}

const ContactFsm::SlotAction& ContactInekfSequencer::action(int slot) const noexcept {
  return actions_[Index(slot)];
}

ContactFsmStatus ContactInekfSequencer::Step(const ContactStepInput& in) noexcept {
  // This tick's outputs start empty: a refused Step reports no action and no update.
  report_valid_ = false;
  for (ContactFsm::SlotAction& a : actions_) {
    a = ContactFsm::SlotAction{};
  }
  for (GroundUpdateReport& g : ground_report_) {
    g = GroundUpdateReport{};
  }
  if (!fsm_.configured()) {
    return ContactFsmStatus::kNotConfigured;
  }
  if (!core_.initialized()) {
    return ContactFsmStatus::kNotInitialized;
  }
  if (in.periods < 1) {
    return ContactFsmStatus::kBadPeriods;
  }
  const auto n = static_cast<std::size_t>(num_slots());
  if (in.kinematics.size() != n || in.joint_velocity.size() != n) {
    return ContactFsmStatus::kSizeMismatch;
  }
  const InekfStatus ps =
      core_.Propagate(in.gyro, in.accel, static_cast<double>(in.periods) * fsm_.period());
  if (ps != InekfStatus::kOk) {
    core_status_ = ps;
    return ContactFsmStatus::kPropagateRefused;
  }
  core_status_ = InekfStatus::kOk;

  const Eigen::Matrix3d& R = core_.rotation();
  const Eigen::Vector3d& v = core_.velocity();
  const Eigen::Vector3d& w = core_.angular_velocity();
  for (std::size_t i = 0; i < n; ++i) {
    const ContactKinematics& k = in.kinematics[i];
    const Eigen::Vector3d& rdot = in.joint_velocity[i];
    if (k.position.allFinite() && k.jacobian.allFinite() && rdot.allFinite()) {
      const Eigen::Vector3d body = w.cross(k.position) + rdot;
      velocity_[i] = v + R * body;
    } else {
      velocity_[i].setConstant(std::numeric_limits<double>::quiet_NaN());
    }
  }
  const std::span<const Eigen::Vector3d> velocity(velocity_.data(), n);
  const std::span<ContactFsm::SlotAction> actions(actions_.data(), n);
  if (fsm_.PreUpdate(in.periods, in.planned_mask, velocity, actions) != ContactFsmStatus::kOk) {
    return ContactFsmStatus::kSizeMismatch;  // unreachable: sizes checked above
  }

  bool consistent = true;
  std::array<bool, kInekfMaxContacts> pin_now{};
  for (std::size_t i = 0; i < n; ++i) {
    const int slot = static_cast<int>(i);
    ContactFsm::SlotAction& a = actions_[i];
    if (a.use) {  // the core reads no other slot's (an inserting slot is requested too)
      kin_[i].position = in.kinematics[i].position;
      kin_[i].jacobian = in.kinematics[i].jacobian;
    }
    if (a.remove) {
      const InekfStatus st = core_.RemoveContact(slot);
      if (st != InekfStatus::kOk) {
        core_status_ = st;
        consistent = false;
      }
    }
    if (a.insert) {
      const InekfStatus st = core_.InsertContact(slot, kin_[i]);
      if (st != InekfStatus::kOk) {
        core_status_ = st;
        fsm_.RevertInsert(slot);
        a.insert = false;
        a.use = false;
      }
    }
    if (a.set_scale) {
      const InekfStatus st = core_.SetContactNoiseScale(slot, a.noise_scale);
      if (st != InekfStatus::kOk) {
        core_status_ = st;
        consistent = false;
      }
    }
    if (a.insert) {
      anchor_age_[i] = 0;
      pinned_[i] = false;
    } else if (core_.contact_active(slot)) {
      anchor_age_[i] = SaturatingAdd(anchor_age_[i], in.periods);
      pin_now[i] = ground_.enabled && !pinned_[i] && anchor_age_[i] >= settle_ticks_ &&
                   fsm_.mode(slot) == ContactMode::kStance;
    }
    if (ContactFsm::HoldsAnchor(fsm_.mode(slot)) != core_.contact_active(slot)) {
      consistent = false;
    }
    kin_[i].use = a.use;
  }

  // The pins, once every slot's core action is done and the anchors agree: a
  // mismatched Step pins nothing, and no slot's action sees another's pin.
  for (std::size_t i = 0; consistent && i < n; ++i) {
    if (pin_now[i]) {
      Pin(static_cast<int>(i));
      pinned_[i] = true;
    }
  }

  const std::span<ContactUpdateReport> report(report_.data(), n);
  ContactFsmStatus out = ContactFsmStatus::kOk;
  if (!consistent) {
    out = ContactFsmStatus::kStateMismatch;
  } else {
    const InekfStatus us =
        core_.UpdateContacts(std::span<const ContactKinematics>(kin_.data(), n), report);
    if (us != InekfStatus::kOk) {
      core_status_ = us;
      out = ContactFsmStatus::kUpdateRefused;
    }
  }
  report_valid_ = out == ContactFsmStatus::kOk;
  fsm_.PostUpdate(report, report_valid_);
  FinishTick(in.periods);
  return out;
}

void ContactInekfSequencer::Pin(int slot) noexcept {
  const bool inflate = gated_pins_ >= ground_.max_gated;
  GroundUpdateReport g;
  const InekfStatus st =
      core_.UpdateGroundHeight(slot, ground_.height, ground_.sigma, ground_.gate_chi2, inflate, g);
  if (st != InekfStatus::kOk) {
    core_status_ = st;  // no pin on this release; the gated count is left as it was
    return;
  }
  ground_report_[Index(slot)] = g;
  gated_pins_ = g.result == GroundUpdateResult::kGated ? SaturatingAdd(gated_pins_, 1) : 0;
}

void ContactInekfSequencer::FinishTick(int periods) noexcept {
  const int n = num_slots();
  bool confirmed = false;
  bool used = false;
  bool gated = false;
  for (int i = 0; i < n; ++i) {
    confirmed = confirmed || ContactFsm::Confirmed(fsm_.mode(i));
    if (report_valid_) {
      const ContactUpdateResult r = report_[static_cast<std::size_t>(i)].result;
      used = used || r == ContactUpdateResult::kUsed;
      gated = gated || r == ContactUpdateResult::kGated;
    }
  }
  no_anchor_ticks_ = confirmed ? 0 : SaturatingAdd(no_anchor_ticks_, periods);
  if (used) {
    gated_ticks_ = 0;
  } else if (gated) {
    gated_ticks_ = SaturatingAdd(gated_ticks_, periods);
  }
}

}  // namespace rtc::estimation
