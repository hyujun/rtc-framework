// StageReferenceBuilder with `reference.foothold.enabled` (W8-F08 PR 1): the
// swing contacts' x · y · yaw planned to a landing target. Oracles are written
// here from the header's formula, not from the implementation: the base
// reference from the snapshot and command (the knot arithmetic), the nominal
// offsets from a separate pinocchio Data FK at q_default, the stance after a
// touch-down from the window's next lift-off or the YAML gait numbers, the
// measured contact velocity from a separate getFrameVelocity. Off, the
// horizon is byte for byte the builder's without the foothold (the main-vs-
// branch byte comparison of the shipped walk is a probe outside the repo).
//
// malloc_gate.hpp may be included in exactly one translation unit per binary.
#include "rtc_base/testing/malloc_gate.hpp"
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/reference/contact_schedule.hpp"
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/gait_table.hpp"
#include "rtc_mpc/reference/stage_reference.hpp"
#include "rtc_mpc/reference/stage_reference_builder.hpp"
#include "rtc_mpc/reference/swing_trajectory.hpp"
#include "test_utils/alloc_counter.hpp"
#include "test_utils/alloc_counter_overrides.hpp"
#include "test_utils/g1_fixture.hpp"
#include "test_utils/loco_reference_fixture.hpp"
#include "test_utils/shipped_walk_yaml.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#pragma GCC diagnostic pop

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

namespace rtc::mpc {
namespace {

namespace tu = test_utils;
using tu::AllocCounter;

constexpr double kTol = 1e-12;
constexpr double kShippedDt = 0.035;
constexpr int kShippedNodes = 33;
constexpr double kF = 0.7;  // the shipped intermediate_fraction
const char* const kSoles[2] = {"left_sole", "right_sole"};
const std::vector<std::string> kGaitContacts = {"left_foot", "right_foot"};

bool UrdfMissing() {
  return !std::filesystem::exists(tu::kG1Urdf);
}

double AngleDiff(double a, double b) {
  return std::remainder(a - b, 2.0 * std::numbers::pi);
}

/// The yaw of a yaw-only pose quaternion (x y z w at p[3 … 6]).
double PoseYaw(const double* pose) {
  return 2.0 * std::atan2(pose[5], pose[6]);
}

struct G1 {
  pinocchio::Model model;
  RobotModelHandler handler;
  std::vector<std::string> joints;

  G1() : model(tu::BuildG1Floating()) {
    EXPECT_EQ(handler.Init(model, YAML::Load(tu::kG1SolesYaml)), RobotModelInitError::kNoError);
    joints = tu::ActuatedJointNames(model);
  }

  [[nodiscard]] ReferenceConfig Config(bool on) const {
    ReferenceConfig c;
    YAML::Node y = YAML::Load(tu::kShippedReferenceYaml);
    y["foothold"]["enabled"] = on;
    // hip yaws apart, so the soles' nominal yaws ψ_c differ from 0 and each other
    y["q_default"]["left_hip_yaw_joint"] = 0.12;
    y["q_default"]["right_hip_yaw_joint"] = -0.06;
    EXPECT_EQ(LoadReferenceConfig(y, joints, c), ReferenceError::kNoError);
    EXPECT_EQ(c.foothold_enabled, on);
    return c;
  }

  /// The stand pose at (0.3, −0.2), yawed, slightly tilted, the joints moved
  /// off the stand; `moving` gives the base and every joint a velocity.
  [[nodiscard]] MPCStateSnapshot Measured(double yaw, bool moving) const {
    Eigen::VectorXd q = pinocchio::neutral(model);
    const ReferenceConfig cfg = Config(false);
    for (const auto& [name, value] : cfg.q_default) {
      if (model.existJointName(name)) {
        q[model.joints[model.getJointId(name)].idx_q()] = value;
      }
    }
    MPCStateSnapshot s{};
    s.nq = model.nq;
    s.nv = model.nv;
    s.q[0] = 0.3;
    s.q[1] = -0.2;
    s.q[2] = 0.76;
    const Eigen::Quaterniond quat(tu::Rzyx(yaw, moving ? 0.02 : 0.0, moving ? -0.01 : 0.0));
    s.q[3] = quat.x();
    s.q[4] = quat.y();
    s.q[5] = quat.z();
    s.q[6] = quat.w();
    for (int i = 7; i < model.nq; ++i) {
      s.q[static_cast<std::size_t>(i)] = q[i] + (moving ? 0.03 * std::sin(1.7 * i) : 0.0);
    }
    if (moving) {
      const double v[6] = {0.3, 0.05, 0.01, 0.02, -0.03, 0.15};
      for (int i = 0; i < 6; ++i) {
        s.v[static_cast<std::size_t>(i)] = v[i];
      }
      for (int i = 6; i < model.nv; ++i) {
        s.v[static_cast<std::size_t>(i)] = 0.4 * std::sin(0.9 * i + 0.3);
      }
    }
    s.sequence = 5;
    s.valid = true;
    return s;
  }

  [[nodiscard]] Eigen::VectorXd Q(const MPCStateSnapshot& s) const {
    Eigen::VectorXd q(model.nq);
    for (int i = 0; i < model.nq; ++i) {
      q[i] = s.q[static_cast<std::size_t>(i)];
    }
    return q;
  }
};

/// Planar pose of the two soles: at q_default (base at the origin, level) or
/// at a snapshot — a separate Data from the builder's.
struct Planar {
  double x, y, psi;
};

std::array<Planar, 2> SolesAt(const G1& g, const Eigen::VectorXd& q) {
  pinocchio::Data d(g.model);
  pinocchio::framesForwardKinematics(g.model, d, q);
  std::array<Planar, 2> out{};
  for (std::size_t c = 0; c < 2; ++c) {
    const pinocchio::SE3& oMf = d.oMf[g.model.getFrameId(kSoles[c])];
    out[c] = Planar{oMf.translation()[0], oMf.translation()[1],
                    std::atan2(oMf.rotation()(1, 0), oMf.rotation()(0, 0))};
  }
  return out;
}

std::array<Planar, 2> NominalOracle(const G1& g) {
  Eigen::VectorXd q = pinocchio::neutral(g.model);  // base at the origin, identity
  for (const auto& [name, value] : g.Config(false).q_default) {
    if (g.model.existJointName(name)) {
      q[g.model.joints[g.model.getJointId(name)].idx_q()] = value;
    }
  }
  return SolesAt(g, q);
}

/// The base reference (header: three knots, linear, continued past H at the
/// commanded velocity), from the snapshot and the command alone.
struct BaseOracle {
  double p0x, p0y, psi0, p1x, p1y, psi1, p2x, p2y, psi2, vcx, vcy, wc, H;

  BaseOracle(const MPCStateSnapshot& s, const FilteredCommand& cmd, double horizon) : H(horizon) {
    const Eigen::Quaterniond q(s.q[6], s.q[3], s.q[4], s.q[5]);
    const Eigen::Matrix3d R = q.toRotationMatrix();
    psi0 = std::atan2(R(1, 0), R(0, 0));
    const Eigen::Vector3d vm = R * Eigen::Vector3d(s.v[0], s.v[1], s.v[2]);
    const double wm = (R * Eigen::Vector3d(s.v[3], s.v[4], s.v[5]))[2];
    const double vx = cmd.valid ? cmd.v_x : 0.0;
    const double vy = cmd.valid ? cmd.v_y : 0.0;
    wc = cmd.valid ? cmd.omega_z : 0.0;
    vcx = std::cos(psi0) * vx - std::sin(psi0) * vy;
    vcy = std::sin(psi0) * vx + std::cos(psi0) * vy;
    p0x = s.q[0];
    p0y = s.q[1];
    p1x = p0x + kF * H * (vm[0] + vcx) / 2.0;
    p1y = p0y + kF * H * (vm[1] + vcy) / 2.0;
    psi1 = psi0 + kF * H * (wm + wc) / 2.0;
    p2x = p1x + (1.0 - kF) * H * vcx;
    p2y = p1y + (1.0 - kF) * H * vcy;
    psi2 = psi1 + (1.0 - kF) * H * wc;
  }

  [[nodiscard]] Planar At(double t) const {
    if (t <= kF * H) {
      const double a = t / (kF * H);
      return {p0x + (p1x - p0x) * a, p0y + (p1y - p0y) * a, psi0 + (psi1 - psi0) * a};
    }
    if (t <= H) {
      const double a = (t - kF * H) / ((1.0 - kF) * H);
      return {p1x + (p2x - p1x) * a, p1y + (p2y - p1y) * a, psi1 + (psi2 - psi1) * a};
    }
    return {p2x + (t - H) * vcx, p2y + (t - H) * vcy, psi2 + (t - H) * wc};
  }

  /// The landing target of a contact with nominal `o` touching down at `t_td`
  /// (relative to node 0) with `t_st` of stance after it.
  [[nodiscard]] Planar Landing(const Planar& o, double t_td, double t_st) const {
    const Planar b = At(t_td);
    return {b.x + std::cos(b.psi) * o.x - std::sin(b.psi) * o.y + 0.5 * t_st * vcx,
            b.y + std::sin(b.psi) * o.x + std::cos(b.psi) * o.y + 0.5 * t_st * vcy,
            b.psi + o.psi + 0.5 * t_st * wc};
  }
};

/// One gait `walk` over the two soles: [left], [both], [right], [both] at
/// `times` (the right foot swings in phase 0, the left in phase 2).
GaitTable OneWalk(const std::string& times, double switch_lead) {
  const std::string y =
      "sequence: [walk]\nmin_dwell: 0.0\nswitch_lead: " + std::to_string(switch_lead) +
      "\ngaits:\n  walk:\n"
      "    modes: [[left_foot], [left_foot, right_foot], [right_foot], "
      "[left_foot, right_foot]]\n"
      "    switching_times: " +
      times +
      "\n    faster_above: [0.5, 0.5]\n    slower_below: [0.05, 0.05]\n"
      "    measured_tolerance: [0.05, 0.05]\n";
  GaitTable t;
  EXPECT_EQ(LoadGaitTable(YAML::Load(y), kGaitContacts, t), GaitTableError::kNoError);
  return t;
}

GaitTable ShippedTable() {
  GaitTable t;
  EXPECT_EQ(LoadGaitTable(YAML::Load(tu::kShippedGaitYaml), kGaitContacts, t),
            GaitTableError::kNoError);
  return t;
}

FilteredCommand Command(double vx, double vy, double wz) {
  FilteredCommand c;
  c.v_x = vx;
  c.v_y = vy;
  c.omega_z = wz;
  c.base_height = 0.7644;
  c.valid = true;
  c.sequence = 3;
  return c;
}

/// The shipped walk running at `t0` (stance from t0 − 5, walk requested at
/// once, policy off), the window rebuilt at `t0`.
void ShippedWalkAt(GaitScheduler& s, double t0, double lead = 0.0) {
  ASSERT_EQ(s.Init(ShippedTable(), (kShippedNodes - 1) * kShippedDt), GaitSchedulerError::kNoError);
  s.SetPolicyEnabled(false);
  const FilteredCommand walk = Command(0.5, 0.0, 0.0);
  ASSERT_EQ(s.Advance(t0 - 5.0 - lead, walk, PlanarVelocity{}), GaitSchedulerError::kNoError);
  s.RequestGait(2);
  for (const double t : {t0 - 4.9 - lead, t0 - 3.0 - lead, t0 - 1.0 - lead, t0}) {
    ASSERT_EQ(s.Advance(t, walk, PlanarVelocity{}), GaitSchedulerError::kNoError);
  }
  ASSERT_EQ(s.current_gait(), 2U);
  ASSERT_FALSE(s.switch_pending());
}

/// One contact's landings in the horizon: each closed stretch touching down at
/// or before the last node, with the stance after it.
struct Landing {
  std::size_t contact;
  double t_td;       // absolute
  double next_lift;  // absolute, +inf when the window shows none
};

std::vector<Landing> Landings(const ContactSchedule& w, double t_last) {
  std::vector<Landing> out;
  for (std::size_t c = 0; c < 2; ++c) {
    std::array<SwingInterval, kMaxSwingsPerContact> iv{};
    const std::size_t n = SwingIntervals(w, c, iv);
    for (std::size_t i = 0; i < n; ++i) {
      if (!iv[i].open_end && iv[i].touch_down <= t_last) {
        out.push_back({c, iv[i].touch_down,
                       i + 1 < n ? iv[i + 1].lift_off : std::numeric_limits<double>::infinity()});
      }
    }
  }
  return out;
}

/// Every stance node from `l.t_td` up to the next lift-off holds `target`
/// (planar twist and acceleration 0); returns how many it checked.
int ExpectLandingHeld(const HorizonReference& h, const Landing& l, const Planar& target) {
  int checked = 0;
  for (std::uint32_t k = 0; k < h.n_stages; ++k) {
    const StageReference& st = h.stages[k];
    if (st.time < l.t_td || st.time >= l.next_lift) {
      continue;
    }
    SCOPED_TRACE("contact " + std::to_string(l.contact) + " node " + std::to_string(k));
    EXPECT_NE(st.mode & (1U << l.contact), 0U);
    const double* pose = &st.contact_pose[7 * l.contact];
    EXPECT_NEAR(pose[0], target.x, kTol);
    EXPECT_NEAR(pose[1], target.y, kTol);
    EXPECT_NEAR(AngleDiff(PoseYaw(pose), target.psi), 0.0, kTol);
    for (const int i : {0, 1, 5}) {
      EXPECT_EQ(st.contact_twist[6 * l.contact + i], 0.0);
      EXPECT_EQ(st.contact_acceleration[6 * l.contact + i], 0.0);
    }
    ++checked;
  }
  return checked;
}

// ── Schema ────────────────────────────────────────────────────────────────────

TEST(FootholdConfig, TheKeyIsARequiredBoolMapAndBothValuesLoad) {
  const YAML::Node base = YAML::Load(tu::kShippedReferenceYaml);
  std::vector<std::string> joints;
  for (const auto& kv : base["q_default"]) {
    joints.push_back(kv.first.as<std::string>());
  }
  for (const bool on : {false, true}) {
    YAML::Node n = YAML::Clone(base);
    n["foothold"]["enabled"] = on;
    ReferenceConfig c;
    c.foothold_enabled = !on;
    ASSERT_EQ(LoadReferenceConfig(n, joints, c), ReferenceError::kNoError);
    EXPECT_EQ(c.foothold_enabled, on);
  }

  struct Case {
    const char* what;
    void (*edit)(YAML::Node&);
    ReferenceError expected;
  };

  const Case cases[] = {
      {"missing", [](YAML::Node& n) { n.remove("foothold"); }, ReferenceError::kMissingKey},
      {"a scalar", [](YAML::Node& n) { n["foothold"] = 1; }, ReferenceError::kNotAMap},
      {"no enabled", [](YAML::Node& n) { n["foothold"] = YAML::Load("{}"); },
       ReferenceError::kMissingKey},
      {"unknown key", [](YAML::Node& n) { n["foothold"]["gain"] = 0.5; },
       ReferenceError::kUnknownKey},
      {"not a bool", [](YAML::Node& n) { n["foothold"]["enabled"] = "maybe"; },
       ReferenceError::kWrongType},
  };
  for (const Case& k : cases) {
    SCOPED_TRACE(k.what);
    YAML::Node n = YAML::Clone(base);
    k.edit(n);
    ReferenceConfig c;
    c.intermediate_fraction = -1.0;
    EXPECT_EQ(LoadReferenceConfig(n, joints, c), k.expected);
    EXPECT_EQ(c.intermediate_fraction, -1.0);  // untouched
  }
}

// ── Off / on ──────────────────────────────────────────────────────────────────

// Off: contacts x · y · yaw are the measured FK, planar twist 0. On: the same
// horizon in every byte but the planar contact fields, which do move.
TEST(Foothold, OnlyThePlanarContactFieldsDifferAndOffIsTheMeasuredFoot) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "G1 URDF not installed";
  }
  const G1 g;
  const double t0 = 300.0 + 0.0123;
  GaitScheduler s;
  ShippedWalkAt(s, t0);
  const MPCStateSnapshot snap = g.Measured(0.4, true);
  const FilteredCommand cmd = Command(0.5, 0.1, 0.3);
  StageReferenceBuilder off, on;
  ASSERT_EQ(off.Init(g.handler, g.Config(false), tu::ShippedSwing(), FlatSurface(0.0),
                     kShippedNodes, kShippedDt),
            ReferenceError::kNoError);
  ASSERT_EQ(on.Init(g.handler, g.Config(true), tu::ShippedSwing(), FlatSurface(0.0), kShippedNodes,
                    kShippedDt),
            ReferenceError::kNoError);
  ASSERT_EQ(off.Build(s, snap, cmd), ReferenceError::kNoError);
  ASSERT_EQ(on.Build(s, snap, cmd), ReferenceError::kNoError);
  const auto measured = SolesAt(g, g.Q(snap));

  HorizonReference masked = on.active();
  int moved = 0;
  for (std::uint32_t k = 0; k < masked.n_stages; ++k) {
    StageReference& m = masked.stages[k];
    const StageReference& o = off.active().stages[k];
    for (std::size_t c = 0; c < 2; ++c) {
      for (const int i : {0, 1, 3, 4, 5, 6}) {
        moved += m.contact_pose[7 * c + i] != o.contact_pose[7 * c + i];
        m.contact_pose[7 * c + i] = o.contact_pose[7 * c + i];
      }
      for (const int i : {0, 1, 5}) {
        m.contact_twist[6 * c + i] = o.contact_twist[6 * c + i];
        m.contact_acceleration[6 * c + i] = o.contact_acceleration[6 * c + i];
      }
      // off: the measured foot, at rest in the plane
      ASSERT_NEAR(o.contact_pose[7 * c], measured[c].x, kTol);
      ASSERT_NEAR(o.contact_pose[7 * c + 1], measured[c].y, kTol);
      ASSERT_NEAR(AngleDiff(PoseYaw(&o.contact_pose[7 * c]), measured[c].psi), 0.0, kTol);
      for (const int i : {0, 1, 5}) {
        ASSERT_EQ(o.contact_twist[6 * c + i], 0.0);
        ASSERT_EQ(o.contact_acceleration[6 * c + i], 0.0);
      }
    }
  }
  EXPECT_EQ(std::memcmp(&masked, &off.active(), sizeof(HorizonReference)), 0);
  EXPECT_GT(moved, 0);
}

// ── Landing target ────────────────────────────────────────────────────────────

// The shipped walk turning and side-stepping: every landing in the horizon
// holds p_ref(t_td) + R_z(ψ_ref)·o_c + (T_st/2)·v_c, T_st up to the window's
// next lift-off — on both overloads of Build.
TEST(Foothold, EveryLandingIsTheHeadersFormula) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "G1 URDF not installed";
  }
  const G1 g;
  const auto nominal = NominalOracle(g);
  ASSERT_GT(std::abs(nominal[0].y - nominal[1].y), 0.05);  // the offsets are apart
  ASSERT_GT(std::abs(nominal[0].psi), 0.05);               // and yawed
  ASSERT_GT(std::abs(nominal[0].psi - nominal[1].psi), 0.1);
  for (const double lead : {0.0, 0.2, 0.47}) {
    SCOPED_TRACE("lead " + std::to_string(lead));
    const double t0 = 400.0 + 0.0123;
    GaitScheduler s;
    ShippedWalkAt(s, t0, lead);
    const MPCStateSnapshot snap = g.Measured(2.9, true);  // near π: the yaw unwraps
    const FilteredCommand cmd = Command(0.5, 0.1, 0.3);
    StageReferenceBuilder b;
    ASSERT_EQ(b.Init(g.handler, g.Config(true), tu::ShippedSwing(), FlatSurface(0.0), kShippedNodes,
                     kShippedDt),
              ReferenceError::kNoError);
    ASSERT_EQ(b.Build(s, snap, cmd), ReferenceError::kNoError);
    const double H = (kShippedNodes - 1) * kShippedDt;
    const BaseOracle base(snap, cmd, H);
    const double t_last = b.active().stages[kShippedNodes - 1].time;
    const std::vector<Landing> landings = Landings(s.schedule(), t_last);
    ASSERT_GE(landings.size(), 2U);
    int checked = 0;
    for (const Landing& l : landings) {
      ASSERT_TRUE(std::isfinite(l.next_lift));  // the shipped window shows the next lift-off
      const Planar target = base.Landing(nominal[l.contact], l.t_td - t0, l.next_lift - l.t_td);
      checked += ExpectLandingHeld(b.active(), l, target);
    }
    EXPECT_GT(checked, 0);
  }
}

// Commanded 0 (measured at rest): the two landing targets are the q_default
// feet around the measured base — their spacing and relative yaw are the FK's.
TEST(Foothold, WithoutACommandTheLandingsKeepTheDefaultStance) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "G1 URDF not installed";
  }
  const G1 g;
  const auto nominal = NominalOracle(g);
  const double t0 = 500.0 + 0.0123;
  GaitScheduler s;
  ShippedWalkAt(s, t0);
  const MPCStateSnapshot snap = g.Measured(0.7, false);
  const FilteredCommand cmd = Command(0.0, 0.0, 0.0);
  StageReferenceBuilder b;
  ASSERT_EQ(b.Init(g.handler, g.Config(true), tu::ShippedSwing(), FlatSurface(0.0), kShippedNodes,
                   kShippedDt),
            ReferenceError::kNoError);
  ASSERT_EQ(b.Build(s, snap, cmd), ReferenceError::kNoError);
  const HorizonReference& h = b.active();
  std::array<bool, 2> found{};
  std::array<Planar, 2> land{};
  for (const Landing& l : Landings(s.schedule(), h.stages[kShippedNodes - 1].time)) {
    for (std::uint32_t k = 0; k < h.n_stages && !found[l.contact]; ++k) {
      if (h.stages[k].time >= l.t_td) {
        const double* pose = &h.stages[k].contact_pose[7 * l.contact];
        land[l.contact] = Planar{pose[0], pose[1], PoseYaw(pose)};
        found[l.contact] = true;
      }
    }
  }
  ASSERT_TRUE(found[0] && found[1]);
  const double dx = nominal[0].x - nominal[1].x;
  const double dy = nominal[0].y - nominal[1].y;
  const double c = std::cos(0.7), sn = std::sin(0.7);
  EXPECT_NEAR(land[0].x - land[1].x, c * dx - sn * dy, kTol);
  EXPECT_NEAR(land[0].y - land[1].y, sn * dx + c * dy, kTol);
  EXPECT_NEAR(std::hypot(land[0].x - land[1].x, land[0].y - land[1].y), std::hypot(dx, dy), kTol);
  EXPECT_NEAR(AngleDiff(land[0].psi - land[1].psi, nominal[0].psi - nominal[1].psi), 0.0, kTol);
  // and around the measured base: base + R_z(ψ0)·o_c
  for (std::size_t k = 0; k < 2; ++k) {
    EXPECT_NEAR(land[k].x, 0.3 + c * nominal[k].x - sn * nominal[k].y, kTol);
    EXPECT_NEAR(land[k].y, -0.2 + sn * nominal[k].x + c * nominal[k].y, kTol);
  }
}

// ── Curves on a fine grid ─────────────────────────────────────────────────────

// dt 2 ms: the right foot is in the air at t0 (lands 0.05 s on), the left
// lifts off at 0.07 s. The stretch under way starts from the measured pose
// and contact velocity; pose · twist · acceleration agree with each other's
// central differences; lift-off and touch-down are continuous; the left foot
// stands at its measured pose until it lifts.
TEST(Foothold, TheCurvesAreConsistentContinuousAndStartFromTheMeasuredFoot) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "G1 URDF not installed";
  }
  const G1 g;
  constexpr int kN = 65;
  constexpr double kH = 0.002;
  const double horizon = (kN - 1) * kH;
  GaitScheduler s;
  ASSERT_EQ(s.Init(OneWalk("[0.0, 0.06, 0.08, 0.14, 0.16]", 0.04), horizon),
            GaitSchedulerError::kNoError);
  s.SetPolicyEnabled(false);
  const double ta = 600.0;
  const double t0 = ta + 0.16 + 0.01;  // one stride later: the lift-off is a real one
  const FilteredCommand cmd = Command(0.5, 0.1, 0.3);
  ASSERT_EQ(s.Advance(ta, cmd, PlanarVelocity{}), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(t0, cmd, PlanarVelocity{}), GaitSchedulerError::kNoError);
  const MPCStateSnapshot snap = g.Measured(3.12, true);
  StageReferenceBuilder b;
  ASSERT_EQ(b.Init(g.handler, g.Config(true), tu::ShippedSwing(), FlatSurface(0.0), kN, kH),
            ReferenceError::kNoError);
  ASSERT_EQ(b.Build(s, snap, cmd), ReferenceError::kNoError);
  const HorizonReference& h = b.active();
  const auto measured = SolesAt(g, g.Q(snap));

  // The measured contact velocity (world axes at the frame origin).
  pinocchio::Data d(g.model);
  Eigen::VectorXd v(g.model.nv);
  for (int i = 0; i < g.model.nv; ++i) {
    v[i] = snap.v[static_cast<std::size_t>(i)];
  }
  pinocchio::forwardKinematics(g.model, d, g.Q(snap), v);
  const pinocchio::Motion vr = pinocchio::getFrameVelocity(
      g.model, d, g.model.getFrameId(kSoles[1]), pinocchio::LOCAL_WORLD_ALIGNED);
  ASSERT_GT(std::hypot(vr.linear()[0], vr.linear()[1]), 0.05);  // a real start velocity

  // Right foot (1): under way at t0.
  ASSERT_EQ(h.stages[0].mode & 2U, 0U);
  EXPECT_NEAR(h.stages[0].contact_pose[7], measured[1].x, kTol);
  EXPECT_NEAR(h.stages[0].contact_pose[8], measured[1].y, kTol);
  EXPECT_NEAR(AngleDiff(PoseYaw(&h.stages[0].contact_pose[7]), measured[1].psi), 0.0, kTol);
  EXPECT_NEAR(h.stages[0].contact_twist[6], vr.linear()[0], 1e-12);
  EXPECT_NEAR(h.stages[0].contact_twist[7], vr.linear()[1], 1e-12);
  EXPECT_NEAR(h.stages[0].contact_twist[11], vr.angular()[2], 1e-12);
  // Its landing: T_st up to the window's next lift-off (0.10 s).
  const BaseOracle base(snap, cmd, horizon);
  const auto nominal = NominalOracle(g);
  const std::vector<Landing> landings = Landings(s.schedule(), h.stages[kN - 1].time);
  ASSERT_EQ(landings.size(), 1U);
  ASSERT_EQ(landings[0].contact, 1U);
  EXPECT_NEAR(landings[0].next_lift - landings[0].t_td, 0.10, 1e-9);
  EXPECT_GT(ExpectLandingHeld(h, landings[0],
                              base.Landing(nominal[1], landings[0].t_td - t0,
                                           landings[0].next_lift - landings[0].t_td)),
            0);

  // Both curves against a Hermite written here: (start, at its velocity) →
  // (landing target, at rest), the yaw target within π of the start. The left
  // foot lands past H (the base continued at the command) after a stance the
  // table gives: [both] 0.02 + [left] 0.06 + [both] 0.02 s.
  {
    std::array<SwingInterval, kMaxSwingsPerContact> ivl{}, ivr{};
    ASSERT_EQ(SwingIntervals(s.schedule(), 0, ivl), 1U);
    ASSERT_GE(SwingIntervals(s.schedule(), 1, ivr), 1U);
    ASSERT_TRUE(ivr[0].open_start);
    ASSERT_FALSE(ivl[0].open_start || ivl[0].open_end);
    ASSERT_GT(ivl[0].touch_down - t0, horizon);
    ASSERT_TRUE(measured[0].psi < 0.0 || measured[1].psi < 0.0);  // a measured yaw wrapped past π

    struct Curve {
      double ts, te;
      Planar p0, v0, p1;
    };

    const Curve curves[2] = {{ivl[0].lift_off,
                              ivl[0].touch_down,
                              measured[0],
                              {0.0, 0.0, 0.0},
                              base.Landing(nominal[0], ivl[0].touch_down - t0, 0.10)},
                             {t0,
                              ivr[0].touch_down,
                              measured[1],
                              {vr.linear()[0], vr.linear()[1], vr.angular()[2]},
                              base.Landing(nominal[1], ivr[0].touch_down - t0, 0.10)}};
    for (std::size_t c = 0; c < 2; ++c) {
      const Curve& cv = curves[c];
      const double T = cv.te - cv.ts;
      const double p1psi =
          cv.p0.psi + std::remainder(cv.p1.psi - cv.p0.psi, 2.0 * std::numbers::pi);
      const auto hermite = [&](double p0, double v0, double p1, double t, double& d) {
        const double u = (t - cv.ts) / T;
        d = (6.0 * u * u - 6.0 * u) * p0 / T + (3.0 * u * u - 4.0 * u + 1.0) * v0 +
            (-6.0 * u * u + 6.0 * u) * p1 / T;
        return (2.0 * u * u * u - 3.0 * u * u + 1.0) * p0 + (u * u * u - 2.0 * u * u + u) * T * v0 +
               (-2.0 * u * u * u + 3.0 * u * u) * p1;
      };
      int n = 0;
      for (int k = 0; k < kN; ++k) {
        const StageReference& st = h.stages[k];
        if ((st.mode & (1U << c)) != 0) {
          continue;
        }
        SCOPED_TRACE("hermite contact " + std::to_string(c) + " node " + std::to_string(k));
        double dx = 0.0, dy = 0.0, dpsi = 0.0;
        const double x = hermite(cv.p0.x, cv.v0.x, cv.p1.x, st.time, dx);
        const double y = hermite(cv.p0.y, cv.v0.y, cv.p1.y, st.time, dy);
        const double psi = hermite(cv.p0.psi, cv.v0.psi, p1psi, st.time, dpsi);
        EXPECT_NEAR(st.contact_pose[7 * c], x, 1e-11);
        EXPECT_NEAR(st.contact_pose[7 * c + 1], y, 1e-11);
        EXPECT_NEAR(AngleDiff(PoseYaw(&st.contact_pose[7 * c]), psi), 0.0, 1e-11);
        EXPECT_NEAR(st.contact_twist[6 * c], dx, 1e-9);
        EXPECT_NEAR(st.contact_twist[6 * c + 1], dy, 1e-9);
        EXPECT_NEAR(st.contact_twist[6 * c + 5], dpsi, 1e-9);
        ++n;
      }
      EXPECT_GT(n, 10);
    }
  }

  for (std::size_t c = 0; c < 2; ++c) {
    SCOPED_TRACE("contact " + std::to_string(c));
    std::vector<int> swing;
    for (int k = 0; k < kN; ++k) {
      if ((h.stages[k].mode & (1U << c)) == 0) {
        swing.push_back(k);
      }
    }
    ASSERT_GE(swing.size(), 10U);
    // one stretch each in this window: the swing nodes are consecutive
    ASSERT_EQ(swing.back() - swing.front() + 1, static_cast<int>(swing.size()));
    const auto P = [&](int k, int i) { return h.stages[k].contact_pose[7 * c + i]; };
    const auto V = [&](int k, int i) { return h.stages[k].contact_twist[6 * c + i]; };
    const auto A = [&](int k, int i) { return h.stages[k].contact_acceleration[6 * c + i]; };
    const auto Yw = [&](int k) { return PoseYaw(&h.stages[k].contact_pose[7 * c]); };
    double a_max = 0.0;
    for (const int k : swing) {
      for (const int i : {0, 1, 5}) {
        a_max = std::max(a_max, std::abs(A(k, i)));
      }
    }
    ASSERT_GT(a_max, 0.1);
    // Central differences on the interior (a cubic: exact for twist → acc up
    // to rounding, the pose → twist off by h²/6·jerk).
    for (std::size_t j = 1; j + 1 < swing.size(); ++j) {
      const int k = swing[j];
      for (const int i : {0, 1, 5}) {
        const double pm = i == 5 ? 0.0 : P(k - 1, i);
        const double pp = i == 5 ? AngleDiff(Yw(k + 1), Yw(k - 1)) : P(k + 1, i);
        const double fd_pose = (pp - pm) / (2.0 * kH);
        const double fd_twist = (V(k + 1, i) - V(k - 1, i)) / (2.0 * kH);
        const double jerk = (A(k + 1, i) - A(k - 1, i)) / (2.0 * kH);
        EXPECT_NEAR(fd_pose, V(k, i), kH * kH / 6.0 * std::abs(jerk) + 1e-8) << "node " << k;
        EXPECT_NEAR(fd_twist, A(k, i), 1e-6 * (1.0 + std::abs(A(k, i)))) << "node " << k;
      }
    }
    // Lift-off (left) and touch-down (right): the jump to the neighbouring
    // stance node is what the curve moves at rest over less than one node.
    const int k_in = c == 0 ? swing.front() : swing.back();
    const int k_out = c == 0 ? swing.front() - 1 : swing.back() + 1;
    ASSERT_GE(k_out, 0);
    ASSERT_LT(k_out, kN);
    const double jump = std::hypot(P(k_in, 0) - P(k_out, 0), P(k_in, 1) - P(k_out, 1));
    EXPECT_LE(jump, 0.75 * a_max * kH * kH + 1e-12);
    EXPECT_LE(std::abs(AngleDiff(Yw(k_in), Yw(k_out))), 0.75 * a_max * kH * kH + 1e-12);
  }
  // Left foot (0): at its measured pose until it lifts.
  int stood = 0;
  for (int k = 0; k < kN && (h.stages[k].mode & 1U) != 0; ++k) {
    EXPECT_NEAR(h.stages[k].contact_pose[0], measured[0].x, kTol);
    EXPECT_NEAR(h.stages[k].contact_pose[1], measured[0].y, kTol);
    for (const int i : {0, 1, 5}) {
      EXPECT_EQ(h.stages[k].contact_twist[i], 0.0);
    }
    ++stood;
  }
  EXPECT_GT(stood, 10);
}

// ── Fallbacks ─────────────────────────────────────────────────────────────────

// A window too short for the next lift-off: T_st from the gait table (the
// right foot's stance [both] [right] [both] = 0.1 + 0.3 + 0.1 s); a stretch
// landing beyond the window holds its lift-off pose.
TEST(Foothold, PastTheWindowTheGaitTableAndTheLiftOffPoseStandIn) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "G1 URDF not installed";
  }
  const G1 g;
  constexpr int kN = 31;
  constexpr double kH = 0.02;
  const double horizon = (kN - 1) * kH;
  GaitScheduler s;
  ASSERT_EQ(s.Init(OneWalk("[0.0, 0.3, 0.4, 0.7, 0.8]", 0.0), horizon),
            GaitSchedulerError::kNoError);
  s.SetPolicyEnabled(false);
  const double ta = 700.0;
  const double t0 = ta + 0.8 + 0.05;
  const FilteredCommand cmd = Command(0.5, 0.1, 0.3);
  ASSERT_EQ(s.Advance(ta, cmd, PlanarVelocity{}), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(t0, cmd, PlanarVelocity{}), GaitSchedulerError::kNoError);
  const MPCStateSnapshot snap = g.Measured(0.4, true);
  StageReferenceBuilder b;
  ASSERT_EQ(b.Init(g.handler, g.Config(true), tu::ShippedSwing(), FlatSurface(0.0), kN, kH),
            ReferenceError::kNoError);
  ASSERT_EQ(b.Build(s, snap, cmd), ReferenceError::kNoError);
  const HorizonReference& h = b.active();
  const BaseOracle base(snap, cmd, horizon);
  const auto nominal = NominalOracle(g);
  const auto measured = SolesAt(g, g.Q(snap));

  const std::vector<Landing> landings = Landings(s.schedule(), h.stages[kN - 1].time);
  ASSERT_EQ(landings.size(), 1U);
  ASSERT_EQ(landings[0].contact, 1U);
  ASSERT_FALSE(std::isfinite(landings[0].next_lift));  // the window ends first
  const Planar table = base.Landing(nominal[1], landings[0].t_td - t0, 0.5);
  const Planar shown =
      base.Landing(nominal[1], landings[0].t_td - t0, t0 + horizon - landings[0].t_td);
  ASSERT_GT(std::hypot(table.x - shown.x, table.y - shown.y), 0.01);  // the two readings differ
  EXPECT_GT(ExpectLandingHeld(h, landings[0], table), 0);

  // The left foot lifts at 0.35 s and lands at 0.65 s, past the window's 0.6 s.
  std::array<SwingInterval, kMaxSwingsPerContact> iv{};
  ASSERT_EQ(SwingIntervals(s.schedule(), 0, iv), 1U);
  ASSERT_TRUE(iv[0].open_end);
  int held = 0;
  for (int k = 0; k < kN; ++k) {
    if ((h.stages[k].mode & 1U) == 0) {
      EXPECT_NEAR(h.stages[k].contact_pose[0], measured[0].x, kTol);
      EXPECT_NEAR(h.stages[k].contact_pose[1], measured[0].y, kTol);
      EXPECT_NEAR(AngleDiff(PoseYaw(&h.stages[k].contact_pose[0]), measured[0].psi), 0.0, kTol);
      for (const int i : {0, 1, 5}) {
        EXPECT_EQ(h.stages[k].contact_twist[i], 0.0);
        EXPECT_EQ(h.stages[k].contact_acceleration[i], 0.0);
      }
      ++held;
    }
  }
  EXPECT_GT(held, 5);
}

// A switch to stance (a gait whose contacts never lift): the foot landing at
// the switch takes one stance period (0.5 s) as its T_st.
TEST(Foothold, LandingIntoAGaitThatNeverLiftsTakesOnePeriod) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "G1 URDF not installed";
  }
  const G1 g;
  const auto nominal = NominalOracle(g);
  const double H = (kShippedNodes - 1) * kShippedDt;
  int found = 0;
  for (int j = 0; j < 23 && found == 0; ++j) {
    const double t0 = 800.0 + 0.0123;
    GaitScheduler s;
    ShippedWalkAt(s, t0, 0.05 * j);
    s.RequestGait(0);
    const FilteredCommand cmd = Command(0.5, 0.1, 0.3);
    ASSERT_EQ(s.Advance(t0, cmd, PlanarVelocity{}), GaitSchedulerError::kNoError);
    ASSERT_TRUE(s.switch_pending());
    const double t_s = s.switch_time();
    if (t_s > t0 + H) {
      continue;  // the switch lands past the last node: try another phase
    }
    const MPCStateSnapshot snap = g.Measured(0.4, true);
    StageReferenceBuilder b;
    ASSERT_EQ(b.Init(g.handler, g.Config(true), tu::ShippedSwing(), FlatSurface(0.0), kShippedNodes,
                     kShippedDt),
              ReferenceError::kNoError);
    ASSERT_EQ(b.Build(s, snap, cmd), ReferenceError::kNoError);
    const BaseOracle base(snap, cmd, H);
    for (const Landing& l : Landings(s.schedule(), b.active().stages[kShippedNodes - 1].time)) {
      if (std::abs(l.t_td - t_s) < 1e-9) {
        ASSERT_FALSE(std::isfinite(l.next_lift));
        EXPECT_GT(
            ExpectLandingHeld(b.active(), l, base.Landing(nominal[l.contact], l.t_td - t0, 0.5)),
            0);
        ++found;
      }
    }
  }
  EXPECT_GT(found, 0);
}

// ── Refusals ──────────────────────────────────────────────────────────────────

// A contact velocity that overflows (finite joint velocities) under a stretch
// under way is refused with the active horizon kept; off, the same input
// builds (the foothold alone reads it). A NaN is refused either way.
TEST(Foothold, ANonFiniteContactVelocityIsRefusedAndTheHorizonStays) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "G1 URDF not installed";
  }
  const G1 g;
  constexpr int kN = 31;
  constexpr double kH = 0.02;
  GaitScheduler s;
  ASSERT_EQ(s.Init(OneWalk("[0.0, 0.3, 0.4, 0.7, 0.8]", 0.0), (kN - 1) * kH),
            GaitSchedulerError::kNoError);
  s.SetPolicyEnabled(false);
  const FilteredCommand cmd = Command(0.5, 0.1, 0.3);
  ASSERT_EQ(s.Advance(900.0, cmd, PlanarVelocity{}), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(900.85, cmd, PlanarVelocity{}), GaitSchedulerError::kNoError);
  const MPCStateSnapshot good = g.Measured(0.4, true);
  MPCStateSnapshot huge = good;
  for (int i = 6; i < g.model.nv; ++i) {
    huge.v[static_cast<std::size_t>(i)] = 1e308;
  }
  MPCStateSnapshot nan = good;
  nan.v[10] = std::numeric_limits<double>::quiet_NaN();
  StageReferenceBuilder off, on;
  ASSERT_EQ(off.Init(g.handler, g.Config(false), tu::ShippedSwing(), FlatSurface(0.0), kN, kH),
            ReferenceError::kNoError);
  ASSERT_EQ(on.Init(g.handler, g.Config(true), tu::ShippedSwing(), FlatSurface(0.0), kN, kH),
            ReferenceError::kNoError);
  ASSERT_EQ(on.Build(s, good, cmd), ReferenceError::kNoError);
  const HorizonReference before = on.active();
  EXPECT_EQ(on.Build(s, huge, cmd), ReferenceError::kNonFiniteInput);
  EXPECT_EQ(on.Build(s, nan, cmd), ReferenceError::kNonFiniteInput);
  EXPECT_EQ(std::memcmp(&before, &on.active(), sizeof(HorizonReference)), 0);
  EXPECT_EQ(off.Build(s, huge, cmd), ReferenceError::kNoError);
  EXPECT_EQ(off.Build(s, nan, cmd), ReferenceError::kNonFiniteInput);
}

// ── Allocation gate (MPC thread, SCHED_FIFO) ──────────────────────────────────

TEST(FootholdAllocGate, AThousandTicksOfTheShippedWalkAllocateNothing) {
  if (UrdfMissing()) {
    GTEST_SKIP() << "G1 URDF not installed";
  }
  const G1 g;
  StageReferenceBuilder b;
  ASSERT_EQ(b.Init(g.handler, g.Config(true), tu::ShippedSwing(), FlatSurface(0.0), kShippedNodes,
                   kShippedDt),
            ReferenceError::kNoError);
  GaitScheduler s;
  ASSERT_EQ(s.Init(ShippedTable(), (kShippedNodes - 1) * kShippedDt), GaitSchedulerError::kNoError);
  const MPCStateSnapshot snap = g.Measured(0.4, true);
  MPCStateSnapshot bad = snap;
  bad.v[12] = std::numeric_limits<double>::infinity();
  // positive control: the gate counts in this binary
  std::size_t control = 0;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    const YAML::Node n = YAML::Load(tu::kShippedGaitYaml);
    control = gate.count();
  }
  const std::int64_t control_new = AllocCounter::AllocCount();
  AllocCounter::Disarm();
  ASSERT_GT(control, 0U);
  ASSERT_GT(control_new, 0);

  int errors = 0;
  int swing_nodes = 0;
  int walked = 0;
  double sink = 0.0;
  std::size_t malloc_count = 0;
  AllocCounter::Arm();
  {
    const rtc::testing::ScopedMallocGate gate;
    for (int k = 0; k < 1000; ++k) {
      const double t = 1000.0 + kShippedDt * k;
      // stand → walk → side-step and turn → stand again (the policy picks the gait)
      const double vx = k < 100 ? 0.0 : (k < 700 ? 0.6 : 0.0);
      const double vy = (k > 300 && k < 450) ? 0.15 : 0.0;
      const double wz = (k > 450 && k < 650) ? 0.4 : 0.0;
      const FilteredCommand cmd = Command(vx, vy, wz);
      errors += s.Advance(t, cmd, PlanarVelocity{vx, vy, wz}) != GaitSchedulerError::kNoError;
      errors += b.Build(s, snap, cmd) != ReferenceError::kNoError;
      walked += s.current_gait() != 0U;
      const HorizonReference& h = b.active();
      for (std::uint32_t i = 0; i < h.n_stages; ++i) {
        swing_nodes += std::popcount(h.stages[i].mode) < 2;
        sink += h.stages[i].contact_pose[0] + h.stages[i].contact_twist[6];
      }
      if (k % 100 == 0) {
        errors += b.Build(s, bad, cmd) != ReferenceError::kNonFiniteInput;
      }
    }
    malloc_count = gate.count();
  }
  AllocCounter::Disarm();
  EXPECT_EQ(malloc_count, 0U);
  EXPECT_EQ(AllocCounter::AllocCount(), 0);
  EXPECT_EQ(errors, 0);
  EXPECT_GT(walked, 300);
  EXPECT_GT(swing_nodes, 1000);
  EXPECT_TRUE(std::isfinite(sink));
}

}  // namespace
}  // namespace rtc::mpc
