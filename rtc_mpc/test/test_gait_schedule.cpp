// Gait table · GaitScheduler · ContactSchedule (reference/): the W2-F01
// [SPRINT] PR B gates.
//
// The tiling oracle is an explicit (time, mode) list built by looping tiles
// from the anchor — every phase start, then merged on equal modes — not a
// modulo of t. Fixture contacts [A, B, C] = bits 0, 1, 2:
//   walk-like  [0, 0.13, 0.37, 0.52, 0.90]  {0b101, 0b111, 0b010, 0b111}
//              (the first event after a 0.63 s lead is not full support)
//   wrap       [0, 0.3, 0.6, 0.9]           {0b011, 0b110, 0b011} (last == first)
//   hop        [0, 0.25, 0.5]               {0b001, 0b010} (no full support)
//   stance     [0, 0.5]                     {0b111}
// Every refusal sits next to an accepted twin.
#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_mpc/reference/contact_schedule.hpp"
#include "rtc_mpc/reference/gait_scheduler.hpp"
#include "rtc_mpc/reference/gait_table.hpp"
#include "test_utils/panda_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {

using rtc::mpc::ContactSchedule;
using rtc::mpc::FilteredCommand;
using rtc::mpc::GaitScheduler;
using rtc::mpc::GaitSchedulerError;
using rtc::mpc::GaitTable;
using rtc::mpc::GaitTableError;
using rtc::mpc::GaitTemplate;
using rtc::mpc::PlanarVelocity;

const std::vector<std::string> kAbc = {"contact_a", "contact_b", "contact_c"};
constexpr double kHorizon = 1.0;
constexpr double kDt = 0.01;

struct GaitDef {
  const char* modes;
  const char* times;
};

const std::map<std::string, GaitDef>& Library() {
  static const std::map<std::string, GaitDef> lib = {
      {"walk",
       {"[[contact_a, contact_c], [contact_a, contact_b, contact_c], [contact_b], "
        "[contact_a, contact_b, contact_c]]",
        "[0.0, 0.13, 0.37, 0.52, 0.90]"}},
      {"wrap",
       {"[[contact_a, contact_b], [contact_b, contact_c], [contact_a, contact_b]]",
        "[0.0, 0.3, 0.6, 0.9]"}},
      {"hop", {"[[contact_a], [contact_b]]", "[0.0, 0.25, 0.5]"}},
      {"stance", {"[[contact_a, contact_b, contact_c]]", "[0.0, 0.5]"}},
  };
  return lib;
}

// Gait p of a sequence: faster_above 0.1 + 0.4 p, slower_below (p > 0) 0.05 + 0.4 (p − 1),
// tolerance 0.05 (10 on the slowest — it leaves only by command).
YAML::Node GaitYaml(const std::string& name, std::size_t p) {
  YAML::Node g;
  g["modes"] = YAML::Load(Library().at(name).modes);
  g["switching_times"] = YAML::Load(Library().at(name).times);
  const double f = 0.1 + 0.4 * static_cast<double>(p);
  const double s = p == 0 ? 0.0 : 0.05 + 0.4 * static_cast<double>(p - 1);
  const double tol = p == 0 ? 10.0 : 0.05;
  g["faster_above"] = YAML::Load("[" + std::to_string(f) + ", " + std::to_string(f) + "]");
  g["slower_below"] = YAML::Load("[" + std::to_string(s) + ", " + std::to_string(s) + "]");
  g["measured_tolerance"] =
      YAML::Load("[" + std::to_string(tol) + ", " + std::to_string(tol) + "]");
  return g;
}

YAML::Node TableYaml(const std::vector<std::string>& seq, double lead, double dwell = 0.2) {
  YAML::Node n;
  for (const auto& s : seq) {
    n["sequence"].push_back(s);
  }
  n["min_dwell"] = dwell;
  n["switch_lead"] = lead;
  for (std::size_t p = 0; p < seq.size(); ++p) {
    n["gaits"][seq[p]] = GaitYaml(seq[p], p);
  }
  return n;
}

GaitTable Table(const std::vector<std::string>& seq, double lead, double dwell = 0.2) {
  GaitTable t;
  EXPECT_EQ(rtc::mpc::LoadGaitTable(TableYaml(seq, lead, dwell), kAbc, t),
            GaitTableError::kNoError);
  return t;
}

FilteredCommand Cmd(double vx, double vy = 0.0, double wz = 0.0) {
  FilteredCommand c;
  c.v_x = vx;
  c.v_y = vy;
  c.omega_z = wz;
  c.valid = true;
  return c;
}

PlanarVelocity Meas(double vx, double vy = 0.0, double wz = 0.0) {
  return PlanarVelocity{vx, vy, wz};
}

// ── The oracle ───────────────────────────────────────────────────────────────

struct Edge {
  double t;
  std::uint32_t mode;
};

// Every phase start of `g` anchored at `anchor`, up to `until`.
std::vector<Edge> PhaseStarts(const GaitTemplate& g, double anchor, double until) {
  std::vector<Edge> out;
  for (int k = 0;; ++k) {
    for (std::uint32_t j = 0; j < g.n_phases; ++j) {
      const double t = anchor + k * g.period + g.switching_time[j];
      if (t > until) {
        return out;
      }
      out.push_back({t, g.mode[j]});
    }
  }
}

// Mode at x from an edge list (last start ≤ x).
std::uint32_t OracleMode(const std::vector<Edge>& starts, double x) {
  std::uint32_t m = starts.front().mode;
  for (const Edge& e : starts) {
    if (e.t > x) {
      break;
    }
    m = e.mode;
  }
  return m;
}

// The window [t0, t_end) the list implies: mode at t0 and the merged changes.
struct Window {
  std::uint32_t first;
  std::vector<Edge> events;
};

Window OracleWindow(const std::vector<Edge>& starts, double t0, double t_end) {
  Window w{OracleMode(starts, t0), {}};
  std::uint32_t last = w.first;
  for (const Edge& e : starts) {
    if (e.t > t0 && e.t < t_end && e.mode != last) {
      w.events.push_back(e);
      last = e.mode;
    }
  }
  return w;
}

void ExpectWindow(const ContactSchedule& s, const Window& w) {
  ASSERT_EQ(s.n_events, w.events.size());
  EXPECT_EQ(s.mode[0], w.first);
  for (std::size_t i = 0; i < w.events.size(); ++i) {
    EXPECT_NEAR(s.event_time[i], w.events[i].t, 1e-9) << "event " << i;
    EXPECT_EQ(s.mode[i + 1], w.events[i].mode) << "event " << i;
  }
}

// First event of `starts` at or after `earliest` whose new mode is `full`.
double OracleSwitch(const std::vector<Edge>& starts, double earliest, double limit,
                    std::uint32_t full) {
  std::uint32_t prev = starts.front().mode;
  for (std::size_t i = 0; i < starts.size(); ++i) {
    const Edge& e = starts[i];
    if (e.t >= limit) {
      break;
    }
    if (i > 0 && e.t >= earliest && e.mode == full && prev != full) {
      return e.t;
    }
    prev = e.mode;
  }
  return limit;
}

// ── YAML · bits ──────────────────────────────────────────────────────────────

constexpr const char* kOriginalYaml = R"(
sequence: [stance, slow_walk, walk]
min_dwell: 0.2
switch_lead: 1.0
gaits:
  stance:
    modes: [[contact_a, contact_b]]
    switching_times: [0.0, 0.5]
    faster_above: [0.1, 0.1]
    slower_below: [0.0, 0.0]
    measured_tolerance: [10.0, 10.0]
  slow_walk:
    modes: [[contact_a], [contact_a, contact_b], [contact_b], [contact_a, contact_b]]
    switching_times: [0.0, 0.65, 0.85, 1.5, 1.7]
    faster_above: [0.3, 0.2]
    slower_below: [0.05, 0.05]
    measured_tolerance: [0.05, 0.05]
  walk:
    modes: [[contact_a], [contact_a, contact_b], [contact_b], [contact_a, contact_b]]
    switching_times: [0.0, 0.6, 0.7, 1.3, 1.4]
    faster_above: [0.5, 0.35]
    slower_below: [0.25, 0.15]
    measured_tolerance: [0.05, 0.05]
)";

TEST(GaitTable, LoadsTheStanceAndWalkGaitsAsBitmasks) {
  const std::vector<std::string> ab = {"contact_a", "contact_b"};
  GaitTable t;
  std::vector<std::string> names;
  ASSERT_EQ(rtc::mpc::LoadGaitTable(YAML::Load(kOriginalYaml), ab, t, &names),
            GaitTableError::kNoError);
  EXPECT_EQ(names, (std::vector<std::string>{"stance", "slow_walk", "walk"}));
  ASSERT_EQ(t.n_gaits, 3U);
  EXPECT_EQ(t.n_contacts, 2U);
  const GaitTemplate& walk = t.gaits[2];
  ASSERT_EQ(walk.n_phases, 4U);
  EXPECT_EQ(walk.mode[0], 0b01U);
  EXPECT_EQ(walk.mode[1], 0b11U);
  EXPECT_EQ(walk.mode[2], 0b10U);
  EXPECT_EQ(walk.mode[3], 0b11U);
  EXPECT_EQ(rtc::mpc::ModeUnion(walk), 0b11U);
  EXPECT_EQ(walk.period, 1.4);
  EXPECT_EQ(t.gaits[0].mode[0], 0b11U);
  EXPECT_EQ(t.gaits[0].n_phases, 1U);
  EXPECT_EQ(walk.faster_above[rtc::mpc::kAngularAxis], 0.35);
  EXPECT_EQ(t.min_dwell, 0.2);
  EXPECT_EQ(t.switch_lead, 1.0);
}

TEST(GaitTable, TheContactListSetsTheBitOrder) {
  const std::vector<std::string> cab = {"contact_c", "contact_a", "contact_b"};
  GaitTable t;
  ASSERT_EQ(rtc::mpc::LoadGaitTable(TableYaml({"walk"}, 0.63), cab, t), GaitTableError::kNoError);
  EXPECT_EQ(t.gaits[0].mode[0], 0b011U);  // {a, c} → bits 1, 0
  EXPECT_EQ(t.gaits[0].mode[1], 0b111U);
  EXPECT_EQ(t.gaits[0].mode[2], 0b100U);  // {b} → bit 2
  const GaitTable abc = Table({"walk"}, 0.63);
  EXPECT_EQ(abc.gaits[0].mode[0], 0b101U);
  EXPECT_EQ(abc.gaits[0].mode[2], 0b010U);
}

GaitTableError LoadEdited(const std::function<void(YAML::Node&)>& edit) {
  YAML::Node n = TableYaml({"stance", "walk", "wrap"}, 0.63);
  edit(n);
  GaitTable t;
  return rtc::mpc::LoadGaitTable(n, kAbc, t);
}

TEST(GaitTable, RefusesEachBadValueNextToAnAcceptedTwin) {
  using E = GaitTableError;

  struct Case {
    const char* what;
    std::function<void(YAML::Node&)> bad;
    E error;
    std::function<void(YAML::Node&)> good;
  };

  const auto none = [](YAML::Node&) {};
  const std::vector<Case> cases = {
      {"unknown top key", [](YAML::Node& n) { n["min_dwel"] = 0.2; }, E::kUnknownKey, none},
      {"unknown gait key", [](YAML::Node& n) { n["gaits"]["walk"]["period"] = 0.9; },
       E::kUnknownKey, none},
      {"missing key", [](YAML::Node& n) { n.remove("switch_lead"); }, E::kMissingKey, none},
      {"length mismatch",
       [](YAML::Node& n) {
         n["gaits"]["walk"]["switching_times"] = YAML::Load("[0.0, 0.13, 0.37, 0.52]");
       },
       E::kLengthMismatch, none},
      {"first time not 0",
       [](YAML::Node& n) {
         n["gaits"]["walk"]["switching_times"] = YAML::Load("[0.01, 0.13, 0.37, 0.52, 0.90]");
       },
       E::kBadSwitchingTimes,
       [](YAML::Node& n) {
         n["gaits"]["walk"]["switching_times"] = YAML::Load("[0.0, 0.13, 0.37, 0.52, 0.91]");
       }},
      {"not increasing",
       [](YAML::Node& n) {
         n["gaits"]["walk"]["switching_times"] = YAML::Load("[0.0, 0.37, 0.37, 0.52, 0.90]");
       },
       E::kBadSwitchingTimes,
       [](YAML::Node& n) {
         n["gaits"]["walk"]["switching_times"] = YAML::Load("[0.0, 0.36, 0.37, 0.52, 0.90]");
       }},
      {"non-finite",
       [](YAML::Node& n) {
         n["gaits"]["walk"]["switching_times"] = YAML::Load("[0.0, 0.13, .nan, 0.52, 0.90]");
       },
       E::kNonFinite, none},
      {"non-finite threshold",
       [](YAML::Node& n) { n["gaits"]["walk"]["faster_above"] = YAML::Load("[.inf, 0.5]"); },
       E::kNonFinite, none},
      {"unknown contact",
       [](YAML::Node& n) {
         n["gaits"]["stance"]["modes"] = YAML::Load("[[contact_a, contact_d]]");
       },
       E::kUnknownContact,
       [](YAML::Node& n) {
         n["gaits"]["stance"]["modes"] = YAML::Load("[[contact_a, contact_c]]");
       }},
      {"a contact twice in one mode",
       [](YAML::Node& n) {
         n["gaits"]["stance"]["modes"] = YAML::Load("[[contact_a, contact_a]]");
       },
       E::kRepeatedContact,
       [](YAML::Node& n) {
         n["gaits"]["stance"]["modes"] = YAML::Load("[[contact_a, contact_b]]");
       }},
      {"gait without a mode",
       [](YAML::Node& n) {
         n["gaits"]["stance"]["modes"] = YAML::Load("[]");
         n["gaits"]["stance"]["switching_times"] = YAML::Load("[0.0]");
       },
       E::kNoPhase,
       [](YAML::Node& n) { n["gaits"]["stance"]["modes"] = YAML::Load("[[]]"); }},  // flight passes
      {"gait outside the sequence", [](YAML::Node& n) { n["gaits"]["hop"] = GaitYaml("hop", 3); },
       E::kBadSequence,
       [](YAML::Node& n) {
         n["gaits"]["hop"] = GaitYaml("hop", 3);
         n["sequence"].push_back("hop");
       }},
      {"sequence names an undefined gait", [](YAML::Node& n) { n["sequence"].push_back("hop"); },
       E::kBadSequence, none},
      {"a gait listed twice",
       [](YAML::Node& n) { n["sequence"] = YAML::Load("[stance, walk, wrap, walk]"); },
       E::kBadSequence, none},
      {"too many phases",
       [](YAML::Node& n) {
         n["gaits"]["walk"]["modes"] = YAML::Load(
             "[[contact_a],[contact_b],[contact_a],[contact_b],[contact_a],[contact_b],[contact_a],"
             "[contact_b],[contact_a]]");
         n["gaits"]["walk"]["switching_times"] =
             YAML::Load("[0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9]");
       },
       E::kTooManyPhases,
       [](YAML::Node& n) {
         n["gaits"]["walk"]["modes"] = YAML::Load(
             "[[contact_a],[contact_b],[contact_a],[contact_b],[contact_a],[contact_b],[contact_a],"
             "[contact_b]]");
         n["gaits"]["walk"]["switching_times"] = YAML::Load("[0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8]");
       }},
      {"hysteresis inversion",
       [](YAML::Node& n) { n["gaits"]["walk"]["slower_below"] = YAML::Load("[0.1, 0.05]"); },
       E::kHysteresisInversion,
       [](YAML::Node& n) { n["gaits"]["walk"]["slower_below"] = YAML::Load("[0.0999, 0.05]"); }},
      {"negative min_dwell", [](YAML::Node& n) { n["min_dwell"] = -0.01; }, E::kNegativeValue,
       [](YAML::Node& n) { n["min_dwell"] = 0.0; }},
      {"negative switch_lead", [](YAML::Node& n) { n["switch_lead"] = -0.01; }, E::kNegativeValue,
       [](YAML::Node& n) { n["switch_lead"] = 0.0; }},
      {"negative tolerance",
       [](YAML::Node& n) {
         n["gaits"]["walk"]["measured_tolerance"] = YAML::Load("[-0.01, 0.05]");
       },
       E::kNegativeValue,
       [](YAML::Node& n) { n["gaits"]["walk"]["measured_tolerance"] = YAML::Load("[0.0, 0.05]"); }},
      {"wrong kind", [](YAML::Node& n) { n["gaits"]["walk"]["faster_above"] = 0.5; }, E::kWrongType,
       none},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.what);
    EXPECT_EQ(LoadEdited(c.bad), c.error);
    EXPECT_EQ(LoadEdited(c.good), E::kNoError);
  }
}

TEST(GaitTable, RefusesTooManyGaitsAndABadContactList) {
  YAML::Node n = TableYaml({"stance"}, 0.5);
  n["sequence"] = YAML::Node();
  for (int i = 0; i < 9; ++i) {
    const std::string name = "g" + std::to_string(i);
    n["sequence"].push_back(name);
    n["gaits"][name] = GaitYaml("stance", static_cast<std::size_t>(i));
  }
  n["gaits"].remove("stance");
  GaitTable t;
  EXPECT_EQ(rtc::mpc::LoadGaitTable(n, kAbc, t), GaitTableError::kTooManyGaits);
  n["sequence"].remove(8);
  n["gaits"].remove("g8");
  EXPECT_EQ(rtc::mpc::LoadGaitTable(n, kAbc, t), GaitTableError::kNoError);

  const YAML::Node ok = TableYaml({"stance"}, 0.5);
  const std::vector<std::string> dup = {"contact_a", "contact_a", "contact_c"};
  EXPECT_EQ(rtc::mpc::LoadGaitTable(ok, dup, t), GaitTableError::kBadContactList);
  EXPECT_EQ(rtc::mpc::LoadGaitTable(ok, {}, t), GaitTableError::kBadContactList);
  EXPECT_EQ(rtc::mpc::LoadGaitTable(ok, kAbc, t), GaitTableError::kNoError);
}

TEST(GaitTable, ValidateRefusesAPeriodOffTheLastTimeAndABitOutOfRange) {
  GaitTable t = Table({"walk"}, 0.63);
  ASSERT_EQ(rtc::mpc::ValidateGaitTable(t), GaitTableError::kNoError);
  GaitTable bad = t;
  bad.gaits[0].period = 0.91;
  EXPECT_EQ(rtc::mpc::ValidateGaitTable(bad), GaitTableError::kBadSwitchingTimes);
  bad = t;
  bad.gaits[0].mode[2] = 0b1000;  // bit 3 with 3 contacts
  EXPECT_EQ(rtc::mpc::ValidateGaitTable(bad), GaitTableError::kModeOutOfRange);
}

TEST(GaitTable, TheShippedTemplateLoads) {
  const YAML::Node root =
      YAML::LoadFile(std::string(RTC_MPC_CONFIG_DIR) + "/reference_default.yaml");
  const std::vector<std::string> ab = {"contact_a", "contact_b"};
  GaitTable t;
  ASSERT_EQ(rtc::mpc::LoadGaitTable(root["gait"], ab, t), GaitTableError::kNoError);
  GaitScheduler s;
  EXPECT_EQ(s.Init(t, kHorizon), GaitSchedulerError::kNoError);
}

// ── Tiling ───────────────────────────────────────────────────────────────────

GaitTable ScaledWalk(double period, double lead) {
  GaitTable t = Table({"walk"}, lead);
  GaitTemplate& g = t.gaits[0];
  const double scale = period / g.period;
  for (std::size_t j = 0; j <= g.n_phases; ++j) {
    g.switching_time[j] *= scale;
  }
  g.period = g.switching_time[g.n_phases];
  return t;
}

TEST(GaitScheduler, TilesTheGaitExactlyAsTheOracleList) {
  std::mt19937_64 rng(11);
  std::uniform_real_distribution<double> t0_dist(0.0, 100.0);
  std::uniform_real_distribution<double> period_dist(0.5, 2.2);
  int mode_checks = 0;
  for (int n = 0; n < 1000; ++n) {
    const double period = period_dist(rng);
    const double lead = 0.7 * period;
    const GaitTable table = ScaledWalk(period, lead);
    GaitScheduler s;
    ASSERT_EQ(s.Init(table, kHorizon), GaitSchedulerError::kNoError);
    ASSERT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
    const double t0 = t0_dist(rng);
    ASSERT_EQ(s.Advance(t0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
    const ContactSchedule& w = s.schedule();
    const GaitTemplate& g = table.gaits[0];
    ASSERT_GE(w.t_end, t0 + kHorizon + lead);
    const auto starts = PhaseStarts(g, 0.0, w.t_end + period);
    ExpectWindow(w, OracleWindow(starts, w.t0, w.t_end));
    const double min_phase = rtc::mpc::MinPhase(table);
    for (std::uint32_t i = 0; i + 1 < w.n_events; ++i) {
      ASSERT_GE(w.event_time[i + 1] - w.event_time[i], min_phase - 1e-9);
    }
    for (std::uint32_t i = 0; i < w.n_events; ++i) {
      ASSERT_NE(w.mode[i + 1], w.mode[i]);
      ASSERT_EQ(rtc::mpc::ModeAt(w, w.event_time[i]), OracleMode(starts, w.event_time[i]));
      ASSERT_EQ(rtc::mpc::ModeAt(w, std::nextafter(w.event_time[i], -1e300)), w.mode[i]);
    }
    std::uniform_real_distribution<double> in(w.t0, w.t_end);
    for (int k = 0; k < 10; ++k, ++mode_checks) {
      const double x = in(rng);
      ASSERT_EQ(rtc::mpc::ModeAt(w, x), OracleMode(starts, x)) << "t = " << x;
    }
  }
  EXPECT_EQ(mode_checks, 10000);
}

TEST(GaitScheduler, StanceHasNoEventAndALastEqualsFirstGaitNoPeriodEvent) {
  GaitScheduler s;
  ASSERT_EQ(s.Init(Table({"stance"}, 0.35), kHorizon), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(3.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  EXPECT_EQ(s.schedule().n_events, 0U);
  EXPECT_EQ(s.schedule().mode[0], 0b111U);

  const GaitTable wrap = Table({"wrap"}, 0.63);
  ASSERT_EQ(s.Init(wrap, kHorizon), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  for (double t = 0.0; t < 5.0; t += 0.05) {
    ASSERT_EQ(s.Advance(t, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
    const ContactSchedule& w = s.schedule();
    for (std::uint32_t i = 0; i < w.n_events; ++i) {
      const double cycles = w.event_time[i] / 0.9;
      ASSERT_GT(std::abs(cycles - std::round(cycles)), 1e-9) << "an event on a period boundary";
    }
    ExpectWindow(w, OracleWindow(PhaseStarts(wrap.gaits[0], 0.0, w.t_end + 1.0), w.t0, w.t_end));
  }
}

TEST(GaitScheduler, InitRefusesAWindowBeyondTheEventCapacityAndChangesNothing) {
  // window = 1.0 + 0.16 = 1.16 s; ceil(1.16 / 0.04) + 2 = 31 fits, a 0.0399 s phase → 32.
  auto table_with_phase = [](double phase) {
    GaitTable t = Table({"hop"}, 0.16);
    t.gaits[0].switching_time[1] = phase;
    t.gaits[0].switching_time[2] = 2.0 * phase;
    t.gaits[0].period = 2.0 * phase;
    return t;
  };
  GaitScheduler s;
  ASSERT_EQ(s.Init(table_with_phase(0.04), kHorizon), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.5, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  const ContactSchedule before = s.schedule();
  EXPECT_LE(before.n_events, rtc::mpc::kMaxScheduleEvents);
  EXPECT_EQ(s.Init(table_with_phase(0.0399), kHorizon), GaitSchedulerError::kCapacity);
  EXPECT_EQ(std::memcmp(&before, &s.schedule(), sizeof(ContactSchedule)), 0);
  EXPECT_EQ(s.table().gaits[0].switching_time[1], 0.04);
  EXPECT_EQ(s.Init(Table({"hop"}, 0.16), 0.0), GaitSchedulerError::kInvalidHorizon);
  GaitTable invalid = Table({"hop"}, 0.16);
  invalid.n_contacts = 0;
  EXPECT_EQ(s.Init(invalid, kHorizon), GaitSchedulerError::kInvalidTable);
}

// ── Switches ─────────────────────────────────────────────────────────────────

TEST(GaitScheduler, ARequestedSwitchLandsOnTheFirstFullSupportEventAfterTheLead) {
  const double lead = 0.63;
  const GaitTable table = Table({"walk", "wrap"}, lead);
  GaitScheduler s;
  ASSERT_EQ(s.Init(table, kHorizon), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.01, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  const ContactSchedule prev = s.schedule();
  const std::uint64_t epoch = prev.epoch;
  s.RequestGait(1);
  ASSERT_EQ(s.Advance(0.02, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  const ContactSchedule& w = s.schedule();
  EXPECT_EQ(w.epoch, epoch + 1);

  const auto walk_starts = PhaseStarts(table.gaits[0], 0.0, 10.0);
  const double t_s = OracleSwitch(walk_starts, 0.02 + lead, w.t_end, 0b111);
  EXPECT_NEAR(t_s, 1.03, 1e-12);  // 0.90 (→ 0b101) comes first and is not full support
  ASSERT_TRUE(s.switch_pending());
  EXPECT_DOUBLE_EQ(s.switch_time(), t_s);

  // Before t_s: the previous window's events; from t_s: wrap anchored at t_s.
  std::vector<Edge> expected;
  for (std::uint32_t i = 0; i < prev.n_events; ++i) {
    if (prev.event_time[i] > 0.02 && prev.event_time[i] < t_s) {
      expected.push_back({prev.event_time[i], prev.mode[i + 1]});
    }
  }
  std::uint32_t last = expected.empty() ? rtc::mpc::ModeAt(prev, 0.02) : expected.back().mode;
  for (const Edge& e : PhaseStarts(table.gaits[1], t_s, w.t_end)) {
    if (e.t < w.t_end && e.mode != last) {
      expected.push_back(e);
      last = e.mode;
    }
  }
  ExpectWindow(w, Window{rtc::mpc::ModeAt(prev, 0.02), expected});

  // t_s is latched: it does not move as t passes t_s − lead, and the switch lands on it.
  bool switched = false;
  for (int k = 3; k <= 200; ++k) {
    const double t = k * kDt;
    ASSERT_EQ(s.Advance(t, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
    if (!switched && s.current_gait() == 1U) {
      switched = true;
      EXPECT_GE(t, t_s);
      EXPECT_LT(t - kDt, t_s);
    }
    if (!switched) {
      ASSERT_DOUBLE_EQ(s.switch_time(), t_s) << "t = " << t;
    }
  }
  ASSERT_TRUE(switched);
  EXPECT_EQ(s.schedule().epoch, epoch + 1);
  const ContactSchedule& after = s.schedule();
  ExpectWindow(after, OracleWindow(PhaseStarts(table.gaits[1], t_s, after.t_end + 1.0), after.t0,
                                   after.t_end));
}

TEST(GaitScheduler, WithoutFullSupportTheSwitchFallsBackToTheWindowEnd) {
  GaitScheduler s;
  ASSERT_EQ(s.Init(Table({"hop", "stance"}, 0.35), kHorizon), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  s.RequestGait(1);
  ASSERT_EQ(s.Advance(0.2, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  EXPECT_DOUBLE_EQ(s.switch_time(), s.schedule().t_end);
  EXPECT_DOUBLE_EQ(s.switch_time(), 0.2 + kHorizon + 0.35);
}

TEST(GaitScheduler, ASingleModeGaitSwitchesAtTheLead) {
  GaitScheduler s;
  ASSERT_EQ(s.Init(Table({"stance", "walk"}, 0.63), kHorizon), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  s.RequestGait(1);
  ASSERT_EQ(s.Advance(0.4, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  const double t_s = 0.4 + 0.63;
  EXPECT_DOUBLE_EQ(s.switch_time(), t_s);
  // Walk enters at its full-support phase 1 ([0.13, 0.37) of its period), so the
  // stance support carries on through t_s and the first change is walk's 0b010.
  const ContactSchedule& w = s.schedule();
  ASSERT_GE(w.n_events, 1U);
  EXPECT_EQ(w.mode[0], 0b111U);
  EXPECT_NEAR(w.event_time[0], t_s + (0.37 - 0.13), 1e-12);
  EXPECT_EQ(w.mode[1], 0b010U);
}

// Switching between two strides (single support → double support → the other
// single support) must not merge two swings into one or cut out the double
// support: the executed mode never goes straight from one single support to the
// other, and no swing outlasts the longest single support of either gait.
TEST(GaitScheduler, SwitchingBetweenStridesKeepsTheDoubleSupport) {
  const std::vector<std::string> ab = {"contact_a", "contact_b"};
  GaitTable t;
  ASSERT_EQ(rtc::mpc::LoadGaitTable(YAML::Load(kOriginalYaml), ab, t), GaitTableError::kNoError);
  GaitScheduler s;
  ASSERT_EQ(s.Init(t, kHorizon), GaitSchedulerError::kNoError);
  std::mt19937_64 rng(9);
  std::uniform_real_distribution<double> gap(1.0, 3.0);
  double next_request = 0.0;
  std::uint32_t want = 1;
  std::uint32_t prev_mode = 0b11;
  std::array<double, 2> swing_start = {-1.0, -1.0};
  double longest = 0.0;
  int switches = 0;
  std::uint32_t last_gait = 0;
  for (int k = 0; k < 20000; ++k) {
    const double t = k * kDt;
    if (t >= next_request) {
      s.RequestGait(want);
      want = want == 1 ? 2 : 1;
      next_request = t + gap(rng);
    }
    ASSERT_EQ(s.Advance(t, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
    if (s.current_gait() != last_gait) {
      ++switches;
      last_gait = s.current_gait();
    }
    const std::uint32_t m = rtc::mpc::ModeAt(s.schedule(), t);
    ASSERT_FALSE((prev_mode == 0b01U && m == 0b10U) || (prev_mode == 0b10U && m == 0b01U))
        << "single support to single support at t = " << t;
    for (std::size_t c = 0; c < 2; ++c) {
      const bool down = (m >> c) & 1U;
      if (!down && swing_start[c] < 0.0) {
        swing_start[c] = t;
      } else if (down && swing_start[c] >= 0.0) {
        longest = std::max(longest, t - swing_start[c]);
        swing_start[c] = -1.0;
      }
    }
    prev_mode = m;
  }
  EXPECT_GT(switches, 20);
  EXPECT_LE(longest, 0.65 + kDt + 1e-9);  // slow_walk's single support, at tick resolution
}

TEST(GaitScheduler, AnotherInitKeepsCountingIds) {
  GaitScheduler s;
  ASSERT_EQ(s.Init(Table({"walk"}, 0.63), kHorizon), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.5, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  const ContactSchedule before = s.schedule();
  ASSERT_EQ(s.Init(Table({"walk"}, 0.63), kHorizon), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  EXPECT_GT(s.schedule().schedule_id, before.schedule_id);
  EXPECT_GT(s.schedule().epoch, before.epoch);
}

std::string Content(const ContactSchedule& s) {
  ContactSchedule c = s;
  c.schedule_id = 0;
  return std::string(reinterpret_cast<const char*>(&c), sizeof(c));
}

TEST(GaitScheduler, TheIdChangesExactlyWithTheContent) {
  GaitScheduler s;
  ASSERT_EQ(s.Init(Table({"stance", "walk", "wrap"}, 0.63), kHorizon),
            GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.5, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  const ContactSchedule first = s.schedule();
  EXPECT_EQ(first.schedule_id, 1U);
  ASSERT_EQ(s.Advance(0.5, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  EXPECT_EQ(std::memcmp(&first, &s.schedule(), sizeof(ContactSchedule)), 0);  // unused slots too
  ASSERT_EQ(s.Advance(0.5 + kDt, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  EXPECT_EQ(s.schedule().schedule_id, 2U);

  std::map<std::uint64_t, std::string> by_id;
  std::set<std::string> contents;
  std::mt19937_64 rng(5);
  std::uniform_real_distribution<double> v(-1.2, 1.2);
  for (int k = 0; k < 1000; ++k) {
    const double t = 0.52 + kDt * (k / 2);  // every t twice
    const double c = (k / 100) % 2 == 0 ? 1.2 : 0.0;
    ASSERT_EQ(s.Advance(t, Cmd(c), Meas(c + 0.1 * v(rng))), GaitSchedulerError::kNoError);
    const auto [it, fresh] = by_id.emplace(s.schedule().schedule_id, Content(s.schedule()));
    if (fresh) {
      contents.insert(it->second);
    } else {
      ASSERT_EQ(it->second, Content(s.schedule())) << "one id, two contents";
    }
  }
  EXPECT_EQ(contents.size(), by_id.size()) << "two ids, one content";
  EXPECT_GE(by_id.size(), 500U);
}

TEST(GaitScheduler, AfterWalkToStanceTheTailSlotsAreZero) {
  GaitScheduler s;
  ASSERT_EQ(s.Init(Table({"stance", "walk"}, 0.63), kHorizon), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  s.RequestGait(1);
  double t = 0.0;
  for (; t < 3.0; t += kDt) {
    ASSERT_EQ(s.Advance(t, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  }
  ASSERT_EQ(s.current_gait(), 1U);
  ASSERT_GT(s.schedule().n_events, 2U);
  s.RequestGait(0);
  for (; t < 8.0; t += kDt) {
    ASSERT_EQ(s.Advance(t, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  }
  const ContactSchedule& w = s.schedule();
  ASSERT_EQ(s.current_gait(), 0U);
  ASSERT_EQ(w.n_events, 0U);
  EXPECT_EQ(w.mode[0], 0b111U);
  for (std::size_t i = 0; i < rtc::mpc::kMaxScheduleEvents; ++i) {
    EXPECT_EQ(w.event_time[i], 0.0) << i;
    EXPECT_EQ(w.mode[i + 1], 0U) << i;
  }
}

TEST(GaitScheduler, ARequestedGaitLatchesUntilThePolicyIsEnabled) {
  GaitScheduler s;
  ASSERT_EQ(s.Init(Table({"stance", "walk", "wrap"}, 0.63), kHorizon),
            GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  s.RequestGait(1);
  EXPECT_FALSE(s.policy_enabled());
  double t = 0.0;
  for (; t < 10.0; t += kDt) {  // far above walk's faster_above, measured too
    ASSERT_EQ(s.Advance(t, Cmd(1.5), Meas(1.5)), GaitSchedulerError::kNoError);
    ASSERT_EQ(s.target_gait(), 1U) << "t = " << t;
  }
  EXPECT_EQ(s.current_gait(), 1U);
  s.SetPolicyEnabled(true);
  ASSERT_EQ(s.Advance(t, Cmd(1.5), Meas(1.5)), GaitSchedulerError::kNoError);
  EXPECT_EQ(s.target_gait(), 2U);
}

TEST(GaitScheduler, AResetReanchorsTheFirstGait) {
  GaitScheduler s;
  ASSERT_EQ(s.Init(Table({"stance", "walk"}, 0.63), kHorizon), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  s.RequestGait(1);
  ASSERT_EQ(s.Advance(2.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(4.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.current_gait(), 1U);
  const ContactSchedule before = s.schedule();
  s.RequestReset();
  ASSERT_EQ(s.Advance(4.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  EXPECT_EQ(s.current_gait(), 0U);
  EXPECT_FALSE(s.switch_pending());
  EXPECT_EQ(s.schedule().epoch, before.epoch + 1);
  EXPECT_EQ(s.schedule().schedule_id, before.schedule_id + 1);
}

TEST(GaitScheduler, RefusesBadTimesAndLeavesTheScheduleAlone) {
  GaitScheduler s;
  EXPECT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNotInitialised);
  ASSERT_EQ(s.Init(Table({"walk"}, 0.63), kHorizon), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(1.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  const ContactSchedule before = s.schedule();
  EXPECT_EQ(s.Advance(0.99, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kTimeWentBack);
  EXPECT_EQ(s.Advance(std::numeric_limits<double>::quiet_NaN(), Cmd(0.0), Meas(0.0)),
            GaitSchedulerError::kNonFiniteTime);
  EXPECT_EQ(std::memcmp(&before, &s.schedule(), sizeof(ContactSchedule)), 0);
  ASSERT_EQ(s.Advance(1.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
}

// ── Transition policy ────────────────────────────────────────────────────────

TEST(GaitScheduler, ThePolicyFollowsTheScenarioTable) {
  // [stance, walk, wrap]: faster_above 0.1 / 0.5 / 0.9, slower_below 0 / 0.05 / 0.45,
  // tolerance 10 / 0.05 / 0.05 (same on both axes); dwell 0.2. Stance gets a
  // slower_below of 0.05 so that "slower" is true at the bottom and the clamp is what holds it.
  GaitTable table = Table({"stance", "walk", "wrap"}, 0.63);
  table.gaits[0].slower_below = {0.05, 0.05};
  GaitScheduler s;
  ASSERT_EQ(s.Init(table, kHorizon), GaitSchedulerError::kNoError);
  const GaitTable& tb = s.table();
  const double s1 = tb.gaits[1].slower_below[0] + tb.gaits[1].measured_tolerance[0];

  struct Step {
    const char* what;
    double t;
    FilteredCommand c;
    PlanarVelocity m;
    std::uint32_t expected;
  };

  const double dwell_edge = 0.5 + tb.min_dwell;  // the commit below lands at 0.5
  const std::vector<Step> steps = {
      {"start", 0.0, Cmd(0.0), Meas(0.0), 0},
      {"A1 stance band: |c| = faster_above is not above", 0.25, Cmd(0.1), Meas(0.0), 0},
      {"A1 up from stance by command (tolerance 10)", 0.5, Cmd(0.11), Meas(0.0), 1},
      {"D dwell: just before last + min_dwell", std::nextafter(dwell_edge, 0.0), Cmd(0.6),
       Meas(0.46), 1},
      {"D dwell: at last + min_dwell", dwell_edge, Cmd(0.6), Meas(0.46), 2},
      {"E top clamp", 1.0, Cmd(2.0, 1.2, 1.0), Meas(2.0, 1.2, 1.0), 2},
      {"down from wrap", 1.25, Cmd(0.3), Meas(0.3), 1},
      {"A1 walk band", 1.5, Cmd(0.3), Meas(0.3), 1},
      {"A2 measured below faster_above − tol", 1.75, Cmd(0.6), Meas(0.44), 1},
      {"A2 any measured axis", 2.0, Cmd(0.6), Meas(0.44, 0.0, 0.46), 2},
      {"down to walk", 2.25, Cmd(0.0), Meas(0.0), 1},
      {"B measured yaw rate holds walk", 2.5, Cmd(0.0), Meas(0.0, 0.0, 0.2), 1},
      {"C measured == slower_below + tol is not below", 2.75, Cmd(0.0), Meas(s1), 1},
      {"C just below", 3.0, Cmd(0.0), Meas(std::nextafter(s1, 0.0)), 0},
      {"E bottom clamp", 3.25, Cmd(0.0), Meas(0.0), 0},
      {"invalid command reads as zero", 3.5,
       [] {
         FilteredCommand c = Cmd(1.0);
         c.valid = false;
         return c;
       }(),
       Meas(1.0), 0},
  };
  for (const Step& st : steps) {
    SCOPED_TRACE(st.what);
    ASSERT_EQ(s.Advance(st.t, st.c, st.m), GaitSchedulerError::kNoError);
    EXPECT_EQ(s.target_gait(), st.expected);
  }
}

TEST(GaitScheduler, ARandomRunStaysInRangeAndKeepsTheDwell) {
  GaitScheduler s;
  ASSERT_EQ(s.Init(Table({"stance", "walk", "wrap"}, 0.63), kHorizon),
            GaitSchedulerError::kNoError);
  std::mt19937_64 rng(3);
  std::uniform_real_distribution<double> v(-2.0, 2.0);
  std::uint32_t last = 0;
  double last_change = 0.0;
  int changes = 0;
  for (int k = 0; k < 100000; ++k) {
    const double t = k * kDt;
    const double a = v(rng);
    ASSERT_EQ(s.Advance(t, Cmd(a, 0.1 * v(rng), 0.1 * v(rng)), Meas(a + 0.1 * v(rng))),
              GaitSchedulerError::kNoError);
    const std::uint32_t g = s.target_gait();
    ASSERT_LT(g, 3U);
    if (g != last) {
      if (changes > 0) {
        ASSERT_GE(t - last_change, 0.2 - 1e-9) << "t = " << t;
      }
      ++changes;
      last = g;
      last_change = t;
    }
  }
  EXPECT_GT(changes, 100);
}

// ── Helpers ──────────────────────────────────────────────────────────────────

TEST(ContactSchedule, SwingIntervalsAlternateWithTheTableLengths) {
  struct Expect {
    const char* gait;
    std::array<double, 3> swing;  // per contact, 0 = never lifts
  };

  for (const Expect& e : {Expect{"walk", {0.15, 0.13, 0.15}}, Expect{"hop", {0.25, 0.25, 0.5}},
                          Expect{"stance", {0.0, 0.0, 0.0}}}) {
    SCOPED_TRACE(e.gait);
    GaitScheduler s;
    ASSERT_EQ(s.Init(Table({e.gait}, 0.63), kHorizon), GaitSchedulerError::kNoError);
    ASSERT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
    ASSERT_EQ(s.Advance(5.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
    const ContactSchedule& w = s.schedule();
    for (std::size_t c = 0; c < 3; ++c) {
      std::array<rtc::mpc::SwingInterval, 16> out{};
      const std::size_t n = rtc::mpc::SwingIntervals(w, c, out);
      if (e.swing[c] == 0.0) {
        EXPECT_EQ(n, 0U) << "contact " << c;
        continue;
      }
      if (e.swing[c] == 0.5) {  // hop's contact C never touches: one open stretch
        ASSERT_EQ(n, 1U);
        EXPECT_TRUE(out[0].open_start && out[0].open_end);
        continue;
      }
      ASSERT_GE(n, 1U) << "contact " << c;
      double prev_touch = -1.0;
      for (std::size_t i = 0; i < n; ++i) {
        const auto& sw = out[i];
        ASSERT_LT(sw.lift_off, sw.touch_down);
        ASSERT_GT(sw.lift_off, prev_touch - 1e-15);
        prev_touch = sw.touch_down;
        if (!sw.open_start && !sw.open_end) {
          EXPECT_NEAR(sw.touch_down - sw.lift_off, e.swing[c], 1e-12) << "contact " << c;
          EXPECT_EQ(rtc::mpc::ModeAt(w, sw.lift_off) & (1U << c), 0U);
          EXPECT_NE(rtc::mpc::ModeAt(w, sw.touch_down) & (1U << c), 0U);
        }
      }
    }
    std::array<rtc::mpc::SwingInterval, 2> tiny{};
    EXPECT_EQ(rtc::mpc::SwingIntervals(w, 3, tiny), 0U);  // out of range
  }
}

TEST(ContactSchedule, NextEventAndSampleNodesFollowModeAt) {
  GaitScheduler s;
  ASSERT_EQ(s.Init(Table({"walk"}, 0.63), kHorizon), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(0.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  const ContactSchedule& w = s.schedule();
  double te = 0.0;
  std::uint32_t after = 0;
  ASSERT_TRUE(rtc::mpc::NextEvent(w, 0.0, te, after));
  EXPECT_NEAR(te, 0.13, 1e-12);
  EXPECT_EQ(after, 0b111U);
  ASSERT_TRUE(rtc::mpc::NextEvent(w, te, te, after));  // strictly after
  EXPECT_NEAR(te, 0.37, 1e-12);
  double untouched = -1.0;
  EXPECT_FALSE(rtc::mpc::NextEvent(w, w.t_end, untouched, after));
  EXPECT_EQ(untouched, -1.0);

  std::array<std::uint32_t, 21> nodes{};
  ASSERT_TRUE(rtc::mpc::SampleNodes(w, 0.0, 0.05, 20, nodes));
  for (std::size_t k = 0; k <= 20; ++k) {
    EXPECT_EQ(nodes[k], rtc::mpc::ModeAt(w, 0.05 * static_cast<double>(k)));
  }
  std::array<std::uint32_t, 21> keep{};
  keep.fill(7);
  EXPECT_FALSE(rtc::mpc::SampleNodes(w, 0.0, 0.1, 20, keep));     // 2.0 s > t_end
  EXPECT_FALSE(rtc::mpc::SampleNodes(w, -0.01, 0.05, 20, keep));  // before t0
  EXPECT_FALSE(rtc::mpc::SampleNodes(w, 0.0, 0.0, 20, keep));
  EXPECT_FALSE(rtc::mpc::SampleNodes(w, 0.0, 0.05, 21, keep));  // out too short
  EXPECT_EQ(keep[3], 7U);
}

TEST(ContactSchedule, ToContactPlanMapsBitsToTheModelContactFrames) {
  if (!std::filesystem::exists(rtc::mpc::test_utils::kPandaUrdf)) {
    GTEST_SKIP() << "Panda URDF not installed";
  }
  pinocchio::Model model;
  pinocchio::urdf::buildModel(rtc::mpc::test_utils::kPandaUrdf, model);
  rtc::mpc::RobotModelHandler handler;
  ASSERT_EQ(handler.Init(model, YAML::Load(rtc::mpc::test_utils::kPandaTwoFingerYaml)),
            rtc::mpc::RobotModelInitError::kNoError);
  const std::vector<std::string> two = {"contact_a", "contact_b"};
  GaitTable t;
  YAML::Node n = TableYaml({"hop"}, 0.2);
  ASSERT_EQ(rtc::mpc::LoadGaitTable(n, two, t), GaitTableError::kNoError);
  GaitScheduler s;
  ASSERT_EQ(s.Init(t, 0.6), GaitSchedulerError::kNoError);
  ASSERT_EQ(s.Advance(1.0, Cmd(0.0), Meas(0.0)), GaitSchedulerError::kNoError);
  rtc::mpc::ContactPlan plan;
  ASSERT_EQ(rtc::mpc::ToContactPlan(s.schedule(), handler, plan),
            rtc::mpc::ContactPlanError::kNoError);
  const ContactSchedule& w = s.schedule();
  ASSERT_EQ(plan.frames.size(), 2U);
  ASSERT_EQ(plan.phases.size(), w.n_events + 1);
  EXPECT_EQ(plan.phases.front().t_start, 0.0);
  EXPECT_NEAR(plan.phases.back().t_end, w.t_end - w.t0, 1e-12);
  for (std::size_t i = 0; i < plan.phases.size(); ++i) {
    ASSERT_EQ(plan.phases[i].active_frame_ids.size(), 1U);  // hop: one foot at a time
    const int expected = plan.frames[w.mode[i] == 0b01U ? 0 : 1].frame_id;
    EXPECT_EQ(plan.phases[i].active_frame_ids[0], expected);
  }
  ContactSchedule three = w;
  three.n_contacts = 3;
  EXPECT_EQ(rtc::mpc::ToContactPlan(three, handler, plan),
            rtc::mpc::ContactPlanError::kContactCountMismatch);
}

}  // namespace
