// ── Shared fixture for the contact FSM / sequencer tests (test-only) ─────────
//
// The test seam (FSM slot state, the sequencer's core), the pre-registered
// thresholds, an analytic synthetic world that never calls the FSM or the
// core, the per-tick contract checker, and the metric definitions.
//
// Synthetic world: an IMU body and n contact points in W. In stance a point is
// fixed in W; in swing it moves by a min-jerk stride and a cubic lift whose
// approach speed is 0.3 m/s. The IMU samples are the analytic rate and
// specific force at the middle of each sample interval plus white noise
// (density/√T) and a random-walk bias; r_i = R_WIᵀ(f_i − p_I) + σ_kin n +
// σ_q J n₆ with a fixed 3×6 J; ṙ_i is the analytic derivative of the true r_i
// plus N(0, (0.01 m/s)²). Ground truth = the generator's stance intervals.
#pragma once

#include "contact_inekf_testing.hpp"
#include "rtc_controllers/estimation/contact_fsm.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

namespace rtc::estimation {

struct ContactFsmTestAccess {
  static ContactInekf& Core(ContactInekfSequencer& s) { return s.core_; }

  /// Every field of every slot, as doubles (bit-comparable).
  static std::vector<double> SlotState(const ContactFsm& f) {
    std::vector<double> out;
    for (const auto& s : f.slots_) {
      for (double x :
           {static_cast<double>(s.mode), static_cast<double>(s.reason),
            static_cast<double>(s.release.ticks), static_cast<double>(s.release.samples),
            static_cast<double>(s.low_speed.ticks), static_cast<double>(s.low_speed.samples),
            static_cast<double>(s.high_speed.ticks), static_cast<double>(s.high_speed.samples),
            static_cast<double>(s.nis_high.ticks), static_cast<double>(s.nis_high.samples),
            static_cast<double>(s.nis_low.ticks), static_cast<double>(s.nis_low.samples),
            static_cast<double>(s.age), static_cast<double>(s.last), s.last_m2, s.evidence,
            s.speed}) {
        out.push_back(x);
      }
    }
    return out;
  }

  /// FSM slots + the sequencer counters (not this tick's outputs).
  static std::vector<double> SequencerState(const ContactInekfSequencer& s) {
    std::vector<double> out = SlotState(s.fsm_);
    out.push_back(static_cast<double>(s.no_anchor_ticks_));
    out.push_back(static_cast<double>(s.gated_ticks_));
    out.push_back(static_cast<double>(s.gated_pins_));
    for (std::size_t i = 0; i < kInekfMaxContacts; ++i) {
      out.push_back(static_cast<double>(s.anchor_age_[i]));
      out.push_back(static_cast<double>(s.pinned_[i]));
    }
    return out;  // core_status_ / report_valid_ / actions are per-tick outputs, checked apart
  }
};

}  // namespace rtc::estimation

namespace rtc::estimation::testing {

inline constexpr double kPeriod = 2e-3;

/// Pre-registered FSM thresholds.
inline ContactFsm::Config FsmFixture() {
  ContactFsm::Config c;
  c.period = kPeriod;
  c.enter_speed = 0.05;
  c.slip_speed = 0.15;
  c.nis_low = 6.25;
  c.nis_high = 11.34;
  c.enter_dwell = 10e-3;
  c.confirm_dwell = 10e-3;
  c.release_dwell = 4e-3;
  c.slip_dwell = 6e-3;
  c.nis_dwell = 6e-3;
  c.recover_dwell = 20e-3;
  c.evidence_time_constant = 20e-3;
  c.unknown_noise_scale = 100.0;
  return c;
}

/// Dwell tick counts of FsmFixture() (hand-computed: d / 2 ms).
inline constexpr int kNEnter = 5;
inline constexpr int kNConfirm = 5;
inline constexpr int kNRelease = 2;
inline constexpr int kNSlip = 3;
inline constexpr int kNNis = 3;
inline constexpr int kNRecover = 10;

/// The core fixture with the sequencer's contact gate.
inline ContactInekf::Config CoreFixture() {
  ContactInekf::Config c = FixtureConfig();
  c.contact_gate_chi2 = 16.27;
  return c;
}

/// Bit-equality of two double vectors.
inline bool SameBits(const std::vector<double>& a, const std::vector<double>& b) {
  return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

// ── Synthetic world ───────────────────────────────────────────────────────────

enum class World : std::uint8_t {
  kStatic,      // every point in stance, body still
  kGait,        // two points alternating (stance 0.5 s of 0.8 s), body at 0.3 m/s
  kQuad,        // four points, one swinging at a time (stance 0.6 s of 0.8 s)
  kSlip,        // static; point 0 slides along y: 10 ms ramp, hold, 10 ms ramp
  kFkStep,      // static; r_0 gets +2 cm in y for 100 ms
  kAccelSpike,  // static; accel x gets a step of `magnitude` for 4 ms
  kAllOff,      // static; every planned bit off for `duration`
  kBand,        // static; point 0 slides at 0.1 + 0.03 sin(2π t / 0.2) m/s for 2 s
};

/// Planned ↔ truth offset of the gait worlds (D = 40 ms).
enum class Shift : std::uint8_t {
  kNone,
  kPlannedOnEarly,   // planned = GT(t) ∨ GT(t + D)
  kPlannedOnLate,    // planned = GT(t) ∧ GT(t − D)
  kPlannedOffEarly,  // planned = GT(t) ∧ GT(t + D)
  kPlannedOffLate,   // planned = GT(t) ∨ GT(t − D)
};

struct Scenario {
  World world{World::kStatic};
  Shift shift{Shift::kNone};
  double seconds{10.0};
  double magnitude{0.3};   // slip speed [m/s] or accel step [m/s²]
  double duration{0.3};    // all-off duration [s]
  double tilt{0.0};        // contact plane about world x [rad]
  bool foot_speed{false};  // record the true contact-point speeds (RunLog::foot_speed)
  // Gait worlds: a stance point lands `stance_rise` [m] below the ground and
  // rises back to it linearly over the first kRiseTime of stance (an impact
  // sink that rebounds) — slower than v_enter, so the FSM anchors it mid-rise.
  double stance_rise{0.0};
};

inline constexpr double kShiftD = 0.04;
inline constexpr double kCycle = 0.8;
inline constexpr double kStride = 0.24;
inline constexpr double kApproach = 0.3;   // [m/s] vertical approach speed at touchdown
inline constexpr double kRiseTime = 0.06;  // [s] Scenario::stance_rise's rebound
inline constexpr double kEventStart = 2.0;
inline constexpr double kEventEvery = 1.5;
inline constexpr double kEventWindow = 1.4;
inline constexpr int kInitTicks = 125;

inline int NumSlots(const Scenario& sc) {
  return sc.world == World::kQuad ? 4 : 2;
}

inline bool Moving(const Scenario& sc) {
  return sc.world == World::kGait || sc.world == World::kQuad;
}

/// Gait time: still until 0.5 s, a smooth ramp to full speed by 1.0 s.
inline double Warp(double t, const Scenario& sc) {
  if (!Moving(sc) || t < 0.5) {
    return 0.0;
  }
  if (t < 1.0) {
    const double x = (t - 0.5) / 0.5;
    return 0.5 * (x * x * x - 0.5 * x * x * x * x);
  }
  return 0.25 + (t - 1.0);
}

inline double StanceFraction(const Scenario& sc) {
  return sc.world == World::kQuad ? 0.75 : 0.625;
}

inline double Phase(int i, double t, const Scenario& sc) {
  const double off = sc.world == World::kQuad ? 0.25 * i : 0.5 * i;
  return Warp(t, sc) / kCycle + off;
}

inline bool GtStance(int i, double t, const Scenario& sc) {
  if (!Moving(sc)) {
    return true;
  }
  const double ph = Phase(i, t, sc);
  return (ph - std::floor(ph)) < StanceFraction(sc);
}

/// Event index and time into it, or −1 before the first.
inline double EventTime(double t) {
  if (t < kEventStart) {
    return -1.0;
  }
  return std::fmod(t - kEventStart, kEventEvery);
}

inline int NumEvents(const Scenario& sc) {
  int e = 0;
  while (kEventStart + kEventEvery * e + kEventWindow < sc.seconds) {
    ++e;
  }
  return e;
}

/// ∫ speed of the trapezoid slip profile, summed over every event so far.
inline double SlipOffset(double t, double speed) {
  if (t < kEventStart) {
    return 0.0;
  }
  const double n = std::floor((t - kEventStart) / kEventEvery);
  const double e = (t - kEventStart) - kEventEvery * n;
  constexpr double ramp = 0.01;
  constexpr double hold = 0.1;
  auto one = [&](double x) {
    if (x <= 0.0) {
      return 0.0;
    }
    if (x < ramp) {
      return 0.5 * speed * x * x / ramp;
    }
    if (x < ramp + hold) {
      return 0.5 * speed * ramp + speed * (x - ramp);
    }
    if (x < 2.0 * ramp + hold) {
      const double y = x - ramp - hold;
      return 0.5 * speed * ramp + speed * hold + speed * y - 0.5 * speed * y * y / ramp;
    }
    return speed * (ramp + hold);
  };
  return n * speed * (ramp + hold) + one(e);
}

inline constexpr double kBandStart = 2.0;
inline constexpr double kBandEnd = 4.0;

inline double BandOffset(double t) {
  const double x = std::clamp(t, kBandStart, kBandEnd) - kBandStart;
  const double w = 2.0 * M_PI / 0.2;
  return 0.1 * x - 0.03 / w * (std::cos(w * x) - 1.0);
}

inline Mat3 Tilt(const Scenario& sc) {
  return Eigen::AngleAxisd(sc.tilt, Vec3::UnitX()).toRotationMatrix();
}

/// Contact point i in W.
inline Vec3 Foot(int i, double t, const Scenario& sc) {
  Vec3 f;
  if (sc.world == World::kQuad) {
    static constexpr std::array<double, 4> kx{0.15, -0.15, 0.15, -0.15};
    static constexpr std::array<double, 4> ky{0.1, -0.1, -0.1, 0.1};
    const auto si = static_cast<std::size_t>(i);
    f = Vec3(kx[si] - 0.1, ky[si], 0.0);
  } else {
    f = Vec3(i == 0 ? 0.0 : 0.12, i == 0 ? 0.1 : -0.1, 0.0);
  }
  if (Moving(sc)) {
    const double ph = Phase(i, t, sc);
    const double n = std::floor(ph);
    const double fr = ph - n;
    const double sf = StanceFraction(sc);
    f.x() += kStride * n;
    if (fr >= sf) {
      const double swing_t = (1.0 - sf) * kCycle;
      const double s = (fr - sf) / (1.0 - sf);
      const double u = s * swing_t;
      f.x() += kStride * (10 * s * s * s - 15 * s * s * s * s + 6 * s * s * s * s * s);
      f.z() = (kApproach / swing_t) * u * u * (1.0 - u / swing_t);
    } else if (sc.stance_rise != 0.0) {
      f.z() -= sc.stance_rise * std::max(0.0, 1.0 - fr * kCycle / kRiseTime);
    }
  }
  if (sc.world == World::kSlip && i == 0) {
    f.y() += SlipOffset(t, sc.magnitude);
  }
  if (sc.world == World::kBand && i == 0) {
    f.y() += BandOffset(t);
  }
  return Tilt(sc) * f;
}

inline Vec3 ImuPos(double t, const Scenario& sc) {
  const double tau = Warp(t, sc);
  const Vec3 p(0.3 * tau - 0.1, 0.02 * std::sin(2 * M_PI * 1.25 * tau),
               0.8 + 0.01 * std::sin(2 * M_PI * 2.5 * tau));
  return Tilt(sc) * p;
}

inline Mat3 ImuRot(double t, const Scenario& sc) {
  const double tau = Warp(t, sc);
  const double roll = 0.05 * std::sin(2 * M_PI * 1.25 * tau);
  const double pitch = 0.05 * std::sin(2 * M_PI * 2.5 * tau + 0.3);
  return Tilt(sc) *
         (Eigen::AngleAxisd(pitch, Vec3::UnitY()) * Eigen::AngleAxisd(roll, Vec3::UnitX()))
             .toRotationMatrix();
}

inline Vec3 Vee(const Mat3& S) {
  return Vec3(S(2, 1), S(0, 2), S(1, 0));
}

inline Vec3 TrueRelative(int i, double t, const Scenario& sc) {
  return ImuRot(t, sc).transpose() * (Foot(i, t, sc) - ImuPos(t, sc));
}

inline Vec3 TrueImuVelocity(double t, const Scenario& sc) {
  constexpr double h = 1e-5;
  return (ImuPos(t + h, sc) - ImuPos(t - h, sc)) / (2 * h);
}

inline double TrueFootSpeed(int i, double t, const Scenario& sc) {
  constexpr double h = 1e-5;
  return ((Foot(i, t + h, sc) - Foot(i, t - h, sc)) / (2 * h)).norm();
}

inline bool Planned(int i, double t, const Scenario& sc) {
  auto gt = [&](double x) { return GtStance(i, std::max(0.0, x), sc); };
  bool p = gt(t);
  switch (sc.shift) {
    case Shift::kNone:
      break;
    case Shift::kPlannedOnEarly:
      p = gt(t) || gt(t + kShiftD);
      break;
    case Shift::kPlannedOnLate:
      p = gt(t) && gt(t - kShiftD);
      break;
    case Shift::kPlannedOffEarly:
      p = gt(t) && gt(t + kShiftD);
      break;
    case Shift::kPlannedOffLate:
      p = gt(t) || gt(t - kShiftD);
      break;
  }
  if (sc.world == World::kAllOff) {
    const double e = EventTime(t);
    if (e >= 0.0 && e < sc.duration) {
      p = false;
    }
  }
  return p;
}

inline Eigen::Matrix<double, 3, 6> FixedJacobian() {
  Eigen::Matrix<double, 3, 6> J;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 6; ++c) {
      J(r, c) = 0.3 * std::sin(1.0 + 1.7 * r + 0.9 * c);
    }
  }
  return J;
}

/// One sample of the world, as the sequencer consumes it.
struct Sample {
  Vec3 gyro;
  Vec3 accel;
  std::array<ContactKinematics, kInekfMaxContacts> kin;
  std::array<Vec3, kInekfMaxContacts> rdot;
  std::uint32_t planned{0};
  std::uint32_t gt{0};
};

/// The noisy sample stream of one seed.
class SyntheticWorld {
 public:
  SyntheticWorld(const Scenario& sc, std::uint64_t seed)
      : sc_(sc), rng_(seed), cfg_(CoreFixture()), J_(FixedJacobian()) {
    bg_ = Vec3(Draw(), Draw(), Draw()) * 1e-3;
    ba_ = Vec3(Draw(), Draw(), Draw()) * 5e-2;
  }

  /// Sample at tick k (t = kT), covering (t − T, t].
  Sample At(int k) {
    const double t = k * kPeriod;
    Sample s;
    const double tm = t - 0.5 * kPeriod;
    constexpr double h = 1e-5;
    constexpr double h2 = 1e-4;
    const Mat3 R = ImuRot(tm, sc_);
    const Vec3 w = Vee(R.transpose() * (ImuRot(tm + h, sc_) - ImuRot(tm - h, sc_)) / (2 * h));
    const Vec3 a = (ImuPos(tm + h2, sc_) - 2 * ImuPos(tm, sc_) + ImuPos(tm - h2, sc_)) / (h2 * h2);
    for (int j = 0; j < 3; ++j) {
      bg_[j] += cfg_.gyro_bias_walk * std::sqrt(kPeriod) * Draw();
      ba_[j] += cfg_.accel_bias_walk * std::sqrt(kPeriod) * Draw();
    }
    const Vec3 g(0.0, 0.0, -kG0);
    s.gyro = w + bg_ + Noise3() * cfg_.gyro_noise_density / std::sqrt(kPeriod);
    s.accel =
        R.transpose() * (a - g) + ba_ + Noise3() * cfg_.accel_noise_density / std::sqrt(kPeriod);
    const double e = EventTime(t);
    if (sc_.world == World::kAccelSpike && e >= 0.0 && e < 0.004) {
      s.accel.x() += sc_.magnitude;
    }
    for (int i = 0; i < NumSlots(sc_); ++i) {
      const auto si = static_cast<std::size_t>(i);
      Eigen::Matrix<double, 6, 1> nq;
      for (int j = 0; j < 6; ++j) {
        nq[j] = Draw();
      }
      Vec3 r = TrueRelative(i, t, sc_) + cfg_.kinematic_sigma * Noise3() +
               cfg_.joint_position_sigma * J_ * nq;
      if (sc_.world == World::kFkStep && i == 0 && e >= 0.0 && e < 0.1) {
        r.y() += 0.02;
      }
      s.kin[si].position = r;
      s.kin[si].jacobian = J_;
      s.rdot[si] =
          (TrueRelative(i, t + h, sc_) - TrueRelative(i, t - h, sc_)) / (2 * h) + 0.01 * Noise3();
      if (Planned(i, t, sc_)) {
        s.planned |= 1U << i;
      }
      if (GtStance(i, t, sc_)) {
        s.gt |= 1U << i;
      }
    }
    return s;
  }

 private:
  double Draw() { return nd_(rng_); }

  Vec3 Noise3() { return Vec3(Draw(), Draw(), Draw()); }

  Scenario sc_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> nd_{0.0, 1.0};
  ContactInekf::Config cfg_;
  Eigen::Matrix<double, 3, 6> J_;
  Vec3 bg_;
  Vec3 ba_;
};

// ── Run + per-tick contract checker ───────────────────────────────────────────

struct TickLog {
  std::array<ContactMode, kInekfMaxContacts> mode{};
  std::uint32_t planned{0};
  std::uint32_t gt{0};
};

struct RunLog {
  Scenario sc;
  int slots{2};
  int first{kInitTicks + 1};                                      // first stepped tick
  std::vector<TickLog> tick;                                      // index = k
  std::vector<std::array<double, kInekfMaxContacts>> foot_speed;  // true, in W
  double vhat_err_after_1s{0.0};
  int bad_status{0};  // Step() not kOk
  int contract{0};    // core-contract violations
  int insertions{0};
  int promotions{0};
  int max_no_anchor_ticks{0};
};

/// Per-tick checks of how the sequencer drives the core. Counted, not asserted,
/// so a run reports every violation.
class ContractChecker {
 public:
  explicit ContractChecker(int slots) : slots_(slots) {}

  void Check(const ContactInekfSequencer& s, int k, const std::array<ContactMode, 4>& before,
             RunLog& log) {
    const ContactInekf& core = s.core();
    for (int i = 0; i < slots_; ++i) {
      const auto si = static_cast<std::size_t>(i);
      const ContactMode m = s.fsm().mode(i);
      const ContactFsm::SlotAction& a = s.action(i);
      // (a) insertion only on candidate entry, reported kInsertedThisStep.
      const bool entered_candidate = m == ContactMode::kCandidate && before[si] != m;
      if (a.insert != entered_candidate) {
        ++log.contract;
      }
      if (a.insert) {
        ++log.insertions;
        insert_tick_[si] = k;
        used_since_insert_[si] = false;
        if (!s.report_valid() || s.report(i).result != ContactUpdateResult::kInsertedThisStep) {
          ++log.contract;
        }
      }
      // (b) promotion: ≥ max(n_confirm, 2) after insertion and after a kUsed.
      if (before[si] == ContactMode::kCandidate && m == ContactMode::kStance) {
        ++log.promotions;
        if (k - insert_tick_[si] < std::max(kNConfirm, 2) || !used_since_insert_[si]) {
          ++log.contract;
        }
      }
      if (s.report_valid() && s.report(i).result == ContactUpdateResult::kUsed &&
          k > insert_tick_[si]) {
        used_since_insert_[si] = true;
      }
      // (c) slip entry drops the anchor and the update; scales by mode.
      if (m == ContactMode::kSlip && before[si] != m && (!a.remove || a.use)) {
        ++log.contract;
      }
      if (m == ContactMode::kUnknown &&
          core.contact_noise_scale(i) != config_.unknown_noise_scale) {
        ++log.contract;
      }
      if ((m == ContactMode::kStance || m == ContactMode::kCandidate) &&
          core.contact_noise_scale(i) != 1.0) {
        ++log.contract;
      }
      if (a.set_scale && !core.contact_active(i)) {
        ++log.contract;
      }
      // An inactive slot is back at scale 1 (RemoveContact resets it).
      if (!core.contact_active(i) && core.contact_noise_scale(i) != 1.0) {
        ++log.contract;
      }
      // (d) anchor-holding mode ⇔ core slot active.
      if (ContactFsm::HoldsAnchor(m) != core.contact_active(i)) {
        ++log.contract;
      }
    }
  }

 private:
  int slots_;
  ContactFsm::Config config_{FsmFixture()};
  std::array<int, kInekfMaxContacts> insert_tick_{-1000000, -1000000, -1000000, -1000000};
  std::array<bool, kInekfMaxContacts> used_since_insert_{};
};

/// Run one scenario for one seed through a fresh sequencer.
inline RunLog Run(const Scenario& sc, std::uint64_t seed,
                  const ContactFsm::Config& fsm = FsmFixture()) {
  RunLog log;
  log.sc = sc;
  log.slots = NumSlots(sc);
  const int n = static_cast<int>(sc.seconds / kPeriod);
  log.tick.resize(static_cast<std::size_t>(n));
  log.foot_speed.resize(static_cast<std::size_t>(n));
  SyntheticWorld world(sc, seed);
  ContactInekfSequencer seq;
  seq.Init(CoreFixture(), fsm, log.slots);
  StationaryImuAccumulator acc;
  ContractChecker checker(log.slots);
  const auto ns = static_cast<std::size_t>(log.slots);
  for (int k = 1; k < n; ++k) {
    const Sample s = world.At(k);
    TickLog& tl = log.tick[static_cast<std::size_t>(k)];
    tl.planned = s.planned;
    tl.gt = s.gt;
    const double t = k * kPeriod;
    for (int i = 0; sc.foot_speed && i < log.slots; ++i) {
      log.foot_speed[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] =
          TrueFootSpeed(i, t, sc);
    }
    if (k <= kInitTicks) {
      (void)acc.Add(s.gyro, s.accel);
      if (k == kInitTicks) {
        std::array<Vec3, kInekfMaxContacts> pts;
        for (std::size_t i = 0; i < ns; ++i) {
          pts[i] = s.kin[i].position;
        }
        if (seq.Initialize(acc.MeanGyro(), acc.MeanAccel(), kInitTicks * kPeriod, 0.0,
                           std::span<const Vec3>(pts.data(), ns)) != InekfStatus::kOk) {
          ++log.bad_status;
          return log;
        }
      }
      continue;
    }
    std::array<ContactMode, 4> before{};
    for (int i = 0; i < log.slots; ++i) {
      before[static_cast<std::size_t>(i)] = seq.fsm().mode(i);
    }
    ContactStepInput in;
    in.gyro = s.gyro;
    in.accel = s.accel;
    in.kinematics = std::span<const ContactKinematics>(s.kin.data(), ns);
    in.joint_velocity = std::span<const Vec3>(s.rdot.data(), ns);
    in.planned_mask = s.planned;
    if (seq.Step(in) != ContactFsmStatus::kOk) {
      ++log.bad_status;
    }
    checker.Check(seq, k, before, log);
    for (int i = 0; i < log.slots; ++i) {
      tl.mode[static_cast<std::size_t>(i)] = seq.fsm().mode(i);
    }
    log.max_no_anchor_ticks =
        std::max(log.max_no_anchor_ticks, t > 1.0 ? seq.ticks_without_confirmed_anchor() : 0);
    if (t > 1.0) {
      log.vhat_err_after_1s =
          std::max(log.vhat_err_after_1s, (seq.core().velocity() - TrueImuVelocity(t, sc)).norm());
    }
  }
  return log;
}

// ── Metrics ───────────────────────────────────────────────────────────────────

inline bool Bit(std::uint32_t m, int i) {
  return ((m >> i) & 1U) != 0U;
}

inline bool InStance(const RunLog& log, int k, int i) {
  return log.tick[static_cast<std::size_t>(k)].mode[static_cast<std::size_t>(i)] ==
         ContactMode::kStance;
}

inline ContactMode ModeAt(const RunLog& log, int k, int i) {
  return log.tick[static_cast<std::size_t>(k)].mode[static_cast<std::size_t>(i)];
}

/// Nearest-rank quantile (q ∈ [0, 1]); NaN when empty.
inline double Quantile(std::vector<double> v, double q) {
  if (v.empty()) {
    return std::nan("");
  }
  std::sort(v.begin(), v.end());
  const auto idx = static_cast<std::size_t>(q * static_cast<double>(v.size() - 1) + 0.5);
  return v[std::min(v.size() - 1, idx)];
}

struct Detection {
  long tp{0};
  long fp{0};
  long fn{0};
  std::vector<double> touchdown;  // [s]
  std::vector<double> liftoff;    // [s]
  int slip_entries{0};
  int unknown_entries{0};

  [[nodiscard]] double precision() const {
    return tp + fp > 0 ? static_cast<double>(tp) / static_cast<double>(tp + fp) : std::nan("");
  }

  [[nodiscard]] double recall() const {
    return tp + fn > 0 ? static_cast<double>(tp) / static_cast<double>(tp + fn) : std::nan("");
  }

  void Merge(const Detection& o) {
    tp += o.tp;
    fp += o.fp;
    fn += o.fn;
    touchdown.insert(touchdown.end(), o.touchdown.begin(), o.touchdown.end());
    liftoff.insert(liftoff.end(), o.liftoff.begin(), o.liftoff.end());
    slip_entries += o.slip_entries;
    unknown_entries += o.unknown_entries;
  }
};

inline constexpr double kEvalStart = 1.0;   // [s] precision / recall from here
inline constexpr double kEdgeGuard = 0.06;  // [s] ± around a GT transition

/// Precision / recall outside ±(60 ms + |offset|) of GT transitions (t ≥ 1 s),
/// touchdown delay from max(GT, planned) on edge to the first stance, liftoff
/// delay from min(GT, planned) off edge to the first non-stance.
inline Detection Detect(const RunLog& log, double offset) {
  Detection d;
  const int n = static_cast<int>(log.tick.size());
  const int guard = static_cast<int>(std::lround((kEdgeGuard + std::abs(offset)) / kPeriod));
  const int start = std::max(log.first + 1, static_cast<int>(std::lround(kEvalStart / kPeriod)));
  for (int i = 0; i < log.slots; ++i) {
    std::vector<int> edges;
    for (int k = log.first + 1; k < n; ++k) {
      if (Bit(log.tick[static_cast<std::size_t>(k)].gt, i) !=
          Bit(log.tick[static_cast<std::size_t>(k - 1)].gt, i)) {
        edges.push_back(k);
      }
    }
    std::size_t e_idx = 0;
    for (int k = start; k < n; ++k) {
      while (e_idx < edges.size() && edges[e_idx] < k - guard) {
        ++e_idx;
      }
      const bool near = e_idx < edges.size() && std::abs(edges[e_idx] - k) <= guard;
      if (!near) {
        const bool est = InStance(log, k, i);
        const bool gt = Bit(log.tick[static_cast<std::size_t>(k)].gt, i);
        d.tp += est && gt;
        d.fp += est && !gt;
        d.fn += !est && gt;
      }
    }
    for (int k = log.first + 1; k < n; ++k) {
      const ContactMode m = ModeAt(log, k, i);
      if (m != ModeAt(log, k - 1, i)) {
        d.slip_entries += m == ContactMode::kSlip;
        d.unknown_entries += m == ContactMode::kUnknown;
      }
    }
    auto planned = [&](int k) { return Bit(log.tick[static_cast<std::size_t>(k)].planned, i); };
    auto gt = [&](int k) { return Bit(log.tick[static_cast<std::size_t>(k)].gt, i); };
    for (int e : edges) {
      if (e < start) {
        continue;
      }
      if (gt(e)) {
        int ref = e;  // max(GT, planned) on edge
        while (ref < n && !planned(ref)) {
          ++ref;
        }
        for (int k = ref; k < n; ++k) {
          if (InStance(log, k, i)) {
            d.touchdown.push_back((k - ref) * kPeriod);
            break;
          }
          if (!gt(k)) {  // never acquired before the next liftoff: a miss
            d.touchdown.push_back(std::numeric_limits<double>::infinity());
            break;
          }
        }
      } else {
        int ref = e;  // min(GT, planned) off edge: back over GT-on, planned-off ticks
        while (ref - 1 > log.first && gt(ref - 1) && !planned(ref - 1)) {
          --ref;
        }
        for (int k = ref; k < n; ++k) {
          if (!InStance(log, k, i)) {
            d.liftoff.push_back((k - ref) * kPeriod);
            break;
          }
        }
      }
    }
  }
  return d;
}

}  // namespace rtc::estimation::testing
