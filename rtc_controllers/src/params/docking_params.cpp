// Parser for the docking keys (E1-F16). See docking_params.hpp.
#include "rtc_controllers/catching/docking_params.hpp"

#include "catching_yaml_read.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace rtc::catching {

namespace {

using params_detail::ReadSectionNode;
using params_detail::SectionKind;
using params_detail::Spelling;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// The tolerance on a face normal's length: a YAML carries a few decimals.
constexpr double kUnitNormTol = 1e-3;

/// An interval of finite numbers, each end open or closed.
struct Range {
  double lo;
  bool lo_open;
  double hi;
  bool hi_open;
};

constexpr Range Closed(double lo, double hi) {
  return {lo, false, hi, false};
}

/// (0, inf)
constexpr Range Positive() {
  return {0.0, true, kInf, false};
}

/// [0, inf)
constexpr Range NonNegative() {
  return {0.0, false, kInf, false};
}

/// Any finite number.
constexpr Range AnyFinite() {
  return {-kInf, false, kInf, false};
}

std::string Num(double v) {
  std::ostringstream os;
  os << v;
  return os.str();
}

/// The range as a message spells it. An infinite end is always written open:
/// no finite number reaches it.
std::string RangeText(const Range& r) {
  const bool lo_paren = r.lo_open || std::isinf(r.lo);
  const bool hi_paren = r.hi_open || std::isinf(r.hi);
  return std::string(lo_paren ? "(" : "[") + (std::isinf(r.lo) ? std::string("-inf") : Num(r.lo)) +
         ", " + (std::isinf(r.hi) ? std::string("inf") : Num(r.hi)) + (hi_paren ? ")" : "]");
}

std::string Quote(const std::string& path) {
  return "'" + path + "'";
}

std::string Join(const std::string& base, const std::string& key) {
  return base.empty() ? key : base + "." + key;
}

bool IsTbd(const YAML::Node& v) {
  return v.IsScalar() && v.Scalar() == "TBD";
}

/// What a node is, for a message about a list's length.
std::string Shape(const YAML::Node& v) {
  if (v.IsSequence()) {
    return "a sequence of " + std::to_string(v.size()) + " entries";
  }
  return Spelling(v);
}

/// A key that is a measured value of the hand or derived from it: the way it is
/// refused under a function's map says where it belongs, because writing it
/// there is an easy mistake and an ignored one would run on a default.
std::string KeyHint(const std::string& key) {
  static constexpr const char* kHandKeys[] = {
      "s_ent",      "r_ent",   "tan_theta",         "c_min",    "c_cap_max", "c_ent_max",
      "v_perp_max", "a_brake", "delta_lo",          "delta_hi", "sigma_tau", "restitution",
      "e_max",      "p_max",   "n_faces",           "face_a",   "face_b",    "faces_a",
      "faces_b",    "rho_ref", "contact_point_hand"};
  for (const char* k : kHandKeys) {
    if (key == k) {
      return "; it is a measured value of the hand and lives under 'robot.hand.docking', not "
             "under a function's map";
    }
  }
  if (key == "delta_0") {
    return "; it is derived (robot.hand T_close_e2e minus T_close_lead) and is not a key";
  }
  if (key == "m_ball") {
    return "; it is derived (the ball's mass) and is not a key";
  }
  return {};
}

/// Reads nodes under one parser's name: every message is
/// `<who>: '<full.key.path>' …`. The full path of a key is `base + "." + key`;
/// `base` is the full path of the section the key sits in.
class Reader {
 public:
  Reader(std::string who, bool key_hints) : who_(std::move(who)), key_hints_(key_hints) {}

  [[noreturn]] void Reject(const std::string& what) const {
    throw std::invalid_argument(who_ + ": " + what);
  }

  /// The section under `parent`, empty when absent; refuses a non-map.
  YAML::Node Section(const YAML::Node& parent, const char* key, const std::string& base) const {
    const auto s = ReadSectionNode(parent, key);
    if (s.kind == SectionKind::kNotAMap) {
      Reject(Quote(Join(base, key)) + " must be a map, got " + Spelling(parent[key]));
    }
    return s.node;
  }

  /// Refuse a key of `sec` that is not in `allowed`, by name.
  void CheckKeys(const YAML::Node& sec, const std::string& base,
                 std::initializer_list<const char*> allowed) const {
    if (!sec.IsMap()) {
      return;
    }
    for (auto it = sec.begin(); it != sec.end(); ++it) {
      const YAML::Node k = it->first;
      const std::string name = k.IsScalar() ? k.Scalar() : std::string("<non-scalar key>");
      bool known = false;
      for (const char* a : allowed) {
        known = known || name == a;
      }
      if (!known) {
        Reject(Quote(Join(base, name)) + " is not a key of " + Quote(base) +
               (key_hints_ ? KeyHint(name) : std::string()));
      }
    }
  }

  /// A finite number of `v` inside `r`; `path` is its full key.
  double Check(const YAML::Node& v, const std::string& path, const Range& r) const {
    double d = 0.0;
    try {
      d = v.as<double>();
    } catch (const YAML::Exception&) {
      Reject(Quote(path) + " must be a number, got " + Spelling(v));
    }
    const bool lo_ok = r.lo_open ? d > r.lo : d >= r.lo;
    const bool hi_ok = r.hi_open ? d < r.hi : d <= r.hi;
    if (!std::isfinite(d) || !lo_ok || !hi_ok) {
      Reject(Quote(path) + " = " + Spelling(v) + " must be a finite number in " + RangeText(r));
    }
    return d;
  }

  /// A finite number inside `r`, or the fallback when absent. `TBD` is refused.
  double Number(const YAML::Node& sec, const char* key, const std::string& base, double fallback,
                const Range& r) const {
    const YAML::Node v = sec[key];
    if (!v) {
      return fallback;
    }
    return Check(v, Join(base, key), r);
  }

  /// A DECISION value: absent or `TBD` → NaN (unset); otherwise a finite number
  /// inside `r`.
  double Decision(const YAML::Node& sec, const char* key, const std::string& base,
                  const Range& r) const {
    const YAML::Node v = sec[key];
    if (!v || IsTbd(v)) {
      return kNaN;
    }
    return Check(v, Join(base, key), r);
  }

  /// A number that is a whole number of nanoseconds (the grids are integer ns).
  double DtSeconds(const YAML::Node& sec, const char* key, const std::string& base, double fallback,
                   const Range& r) const {
    const double d = Number(sec, key, base, fallback, r);
    if (std::fabs(d * 1e9 - static_cast<double>(std::llround(d * 1e9))) > 1e-3) {
      Reject(Quote(Join(base, key)) + " = " + Num(d) +
             " must be a whole number of nanoseconds (the grid is integer ns)");
    }
    return d;
  }

  /// An integer of `v` inside [lo, hi]; `path` is its full key.
  int CheckInt(const YAML::Node& v, const std::string& path, int lo, int hi) const {
    int i = 0;
    try {
      i = v.as<int>();
    } catch (const YAML::Exception&) {
      Reject(Quote(path) + " must be an integer, got " + Spelling(v));
    }
    if (i < lo || i > hi) {
      Reject(Quote(path) + " = " + Spelling(v) + " is outside [" + std::to_string(lo) + ", " +
             std::to_string(hi) + "]");
    }
    return i;
  }

  int Int(const YAML::Node& sec, const char* key, const std::string& base, int fallback, int lo,
          int hi) const {
    const YAML::Node v = sec[key];
    if (!v) {
      return fallback;
    }
    return CheckInt(v, Join(base, key), lo, hi);
  }

  /// A DECISION integer: absent or `TBD` → 0 (unset).
  int DecisionInt(const YAML::Node& sec, const char* key, const std::string& base, int lo,
                  int hi) const {
    const YAML::Node v = sec[key];
    if (!v || IsTbd(v)) {
      return 0;
    }
    return CheckInt(v, Join(base, key), lo, hi);
  }

  bool Bool(const YAML::Node& sec, const char* key, const std::string& base, bool fallback) const {
    const YAML::Node v = sec[key];
    if (!v) {
      return fallback;
    }
    try {
      return v.as<bool>();
    } catch (const YAML::Exception&) {
      Reject(Quote(Join(base, key)) + " must be a bool, got " + Spelling(v));
    }
  }

  /// N numbers, each inside `r`, from the node `v` at full key `path`.
  template <std::size_t N>
  void Elements(const YAML::Node& v, const std::string& path, const Range& r,
                std::array<double, N>& out) const {
    if (!v.IsSequence() || v.size() != N) {
      Reject(Quote(path) + " must be " + std::to_string(N) + " numbers, got " + Shape(v));
    }
    std::array<double, N> tmp{};
    for (std::size_t i = 0; i < N; ++i) {
      tmp[i] = Check(v[i], path + "[" + std::to_string(i) + "]", r);
    }
    out = tmp;
  }

  /// `Elements` for a key that may be absent; true when it was read.
  template <std::size_t N>
  bool Array(const YAML::Node& sec, const char* key, const std::string& base, const Range& r,
             std::array<double, N>& out) const {
    const YAML::Node v = sec[key];
    if (!v) {
      return false;
    }
    Elements(v, Join(base, key), r, out);
    return true;
  }

  /// A list of 3..kMaxSegmentNodes positive block sizes; true when present.
  bool Blocks(const YAML::Node& sec, const char* key, const std::string& base, int& n_blocks,
              std::array<int, kMaxSegmentNodes>& sizes) const {
    const YAML::Node b = sec[key];
    if (!b) {
      return false;
    }
    const std::string path = Join(base, key);
    if (!b.IsSequence() || b.size() < 3 || b.size() > static_cast<std::size_t>(kMaxSegmentNodes)) {
      Reject(Quote(path) + " must be a sequence of 3.." + std::to_string(kMaxSegmentNodes) +
             " positive integers, got " + Shape(b));
    }
    std::array<int, kMaxSegmentNodes> tmp{};
    for (std::size_t i = 0; i < b.size(); ++i) {
      tmp[i] = CheckInt(b[i], path + "[" + std::to_string(i) + "]", 1, kMaxSegmentNodes);
    }
    sizes = tmp;
    n_blocks = static_cast<int>(b.size());
    return true;
  }

 private:
  std::string who_;
  bool key_hints_;
};

void RequireCatching(const Reader& rd, const YAML::Node& catching) {
  if (!catching || !catching.IsMap()) {
    rd.Reject("must be given the `catching:` map");
  }
}

// ── The stop grid, shared by the nlp search and the mpc_docking planner ─────────

/// `stop.{n_nodes, dt_s, blocks}` under `base`. The blocks must sum to the
/// node count whichever of the two was written: a profile that changes one and
/// not the other is told so here, not by the core's Init.
void ReadStopGrid(const Reader& rd, const YAML::Node& stop, const std::string& base, int& n_stop,
                  double& dt_stop, int& n_blocks, std::array<int, kMaxSegmentNodes>& sizes) {
  rd.CheckKeys(stop, base, {"n_nodes", "dt_s", "blocks"});
  n_stop = rd.Int(stop, "n_nodes", base, n_stop, 3, kMaxSegmentNodes);
  dt_stop = rd.DtSeconds(stop, "dt_s", base, dt_stop, Closed(0.005, 0.2));
  static_cast<void>(rd.Blocks(stop, "blocks", base, n_blocks, sizes));
  int sum = 0;
  for (int i = 0; i < n_blocks; ++i) {
    sum += sizes[static_cast<std::size_t>(i)];
  }
  if (sum != n_stop) {
    rd.Reject(Quote(Join(base, "blocks")) + " sums to " + std::to_string(sum) + " but " +
              Quote(Join(base, "n_nodes")) + " is " + std::to_string(n_stop) +
              " (Σ blocks = n_nodes)");
  }
}

}  // namespace

// ═══ The hand ═════════════════════════════════════════════════════════════════

const char* HandDockingParams::FirstUnset() const noexcept {
  if (!std::isfinite(s_ent)) {
    return "robot.hand.docking.s_ent";
  }
  if (!std::isfinite(r_ent)) {
    return "robot.hand.docking.corridor.r_ent";
  }
  if (!std::isfinite(tan_theta)) {
    return "robot.hand.docking.corridor.tan_theta";
  }
  if (n_faces < 1) {
    return "robot.hand.docking.lateral.n_faces";
  }
  const int n = std::min(n_faces, kMaxDockingFaces);
  for (int i = 0; i < n; ++i) {
    if (!face_a[static_cast<std::size_t>(i)].allFinite()) {
      return "robot.hand.docking.lateral.faces_a";
    }
  }
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(face_b[static_cast<std::size_t>(i)])) {
      return "robot.hand.docking.lateral.faces_b";
    }
  }
  if (!rho_ref.allFinite()) {
    return "robot.hand.docking.lateral.rho_ref";
  }
  if (!std::isfinite(c_min)) {
    return "robot.hand.docking.speed.c_min";
  }
  if (!std::isfinite(c_cap_max)) {
    return "robot.hand.docking.speed.c_cap_max";
  }
  if (!std::isfinite(c_ent_max)) {
    return "robot.hand.docking.speed.c_ent_max";
  }
  if (!std::isfinite(v_perp_max)) {
    return "robot.hand.docking.speed.v_perp_max";
  }
  if (!std::isfinite(a_brake)) {
    return "robot.hand.docking.speed.a_brake";
  }
  if (!std::isfinite(delta_lo)) {
    return "robot.hand.docking.closure.delta_lo";
  }
  if (!std::isfinite(delta_hi)) {
    return "robot.hand.docking.closure.delta_hi";
  }
  if (!std::isfinite(sigma_tau)) {
    return "robot.hand.docking.closure.sigma_tau";
  }
  if (!contact_point_hand.allFinite()) {
    return "robot.hand.docking.impact.contact_point_hand";
  }
  if (!std::isfinite(restitution)) {
    return "robot.hand.docking.impact.restitution";
  }
  return nullptr;
}

HandDockingParams ParseHandDockingParams(const YAML::Node& catching) {
  const Reader rd("ParseHandDockingParams", false);
  RequireCatching(rd, catching);
  HandDockingParams out;
  // Eigen leaves a default-constructed vector uninitialised.
  out.face_a.fill(Eigen::Vector2d::Zero());
  const YAML::Node robot = rd.Section(catching, "robot", "");
  const YAML::Node hand = rd.Section(robot, "hand", "robot");
  const YAML::Node dock = rd.Section(hand, "docking", "robot.hand");
  const std::string d = "robot.hand.docking";
  rd.CheckKeys(dock, d,
               {"provisional", "s_ent", "corridor", "lateral", "speed", "closure", "impact"});

  out.provisional = rd.Bool(dock, "provisional", d, out.provisional);
  out.s_ent = rd.Decision(dock, "s_ent", d, Closed(0.0, 0.5));

  const std::string dc = d + ".corridor";
  const YAML::Node corridor = rd.Section(dock, "corridor", d);
  rd.CheckKeys(corridor, dc, {"r_ent", "tan_theta"});
  out.r_ent = rd.Decision(corridor, "r_ent", dc, {0.0, true, 0.5, false});
  out.tan_theta = rd.Decision(corridor, "tan_theta", dc, Closed(0.0, 10.0));

  // The lateral polygon. The two lists are flat on purpose (the launch overlay
  // bridge cannot carry a list of maps), and they exist only as long as
  // n_faces does: with n_faces unset they are not read at all.
  const std::string dl = d + ".lateral";
  const YAML::Node lateral = rd.Section(dock, "lateral", d);
  rd.CheckKeys(lateral, dl, {"n_faces", "faces_a", "faces_b", "rho_ref"});
  out.n_faces = rd.DecisionInt(lateral, "n_faces", dl, 3, kMaxDockingFaces);
  if (out.n_faces > 0) {
    const std::size_t n = static_cast<std::size_t>(out.n_faces);

    const YAML::Node va = lateral["faces_a"];
    if (!va || IsTbd(va)) {
      for (std::size_t i = 0; i < n; ++i) {
        out.face_a[i] = Eigen::Vector2d(kNaN, kNaN);
      }
    } else {
      const std::string path = Join(dl, "faces_a");
      if (!va.IsSequence() || va.size() != 2 * n) {
        rd.Reject(Quote(path) + " must be a flat list of 2*n_faces = " + std::to_string(2 * n) +
                  " numbers [ax0, ay0, ax1, ay1, ...] (n_faces = " + std::to_string(n) + "), got " +
                  Shape(va));
      }
      for (std::size_t i = 0; i < n; ++i) {
        const double ax =
            rd.Check(va[2 * i], path + "[" + std::to_string(2 * i) + "]", AnyFinite());
        const double ay =
            rd.Check(va[2 * i + 1], path + "[" + std::to_string(2 * i + 1) + "]", AnyFinite());
        const double norm = std::hypot(ax, ay);
        if (!(std::fabs(norm - 1.0) <= kUnitNormTol)) {
          rd.Reject(Quote(path) + " normal " + std::to_string(i) + " = [" + Num(ax) + ", " +
                    Num(ay) + "] has norm " + Num(norm) + ", must be a unit vector to within 1e-3");
        }
        // Exactly unit: the core refuses a normal that is not, and a YAML carries
        // only a few decimals.
        out.face_a[i] = Eigen::Vector2d(ax / norm, ay / norm);
      }
    }
    const YAML::Node vb = lateral["faces_b"];
    if (!vb || IsTbd(vb)) {
      for (std::size_t i = 0; i < n; ++i) {
        out.face_b[i] = kNaN;
      }
    } else {
      const std::string path = Join(dl, "faces_b");
      if (!vb.IsSequence() || vb.size() != n) {
        rd.Reject(Quote(path) + " must be a list of n_faces = " + std::to_string(n) +
                  " numbers (one offset per face), got " + Shape(vb));
      }
      for (std::size_t i = 0; i < n; ++i) {
        out.face_b[i] = rd.Check(vb[i], path + "[" + std::to_string(i) + "]", Closed(-0.5, 0.5));
      }
    }
  }
  if (const YAML::Node v = lateral["rho_ref"]; v && !IsTbd(v)) {
    std::array<double, 2> a{};
    rd.Elements(v, Join(dl, "rho_ref"), Closed(-0.5, 0.5), a);
    out.rho_ref = Eigen::Vector2d(a[0], a[1]);
  }

  const std::string ds = d + ".speed";
  const YAML::Node speed = rd.Section(dock, "speed", d);
  rd.CheckKeys(speed, ds, {"c_min", "c_cap_max", "c_ent_max", "v_perp_max", "a_brake"});
  out.c_min = rd.Decision(speed, "c_min", ds, {0.0, true, 20.0, false});
  out.c_cap_max = rd.Decision(speed, "c_cap_max", ds, {0.0, true, 20.0, false});
  out.c_ent_max = rd.Decision(speed, "c_ent_max", ds, {0.0, true, 20.0, false});
  out.v_perp_max = rd.Decision(speed, "v_perp_max", ds, {0.0, true, 20.0, false});
  out.a_brake = rd.Decision(speed, "a_brake", ds, Closed(0.0, 1000.0));
  if (std::isfinite(out.c_min) && std::isfinite(out.c_cap_max) && !(out.c_cap_max > out.c_min)) {
    rd.Reject(Quote(Join(ds, "c_cap_max")) + " = " + Num(out.c_cap_max) + " must exceed " +
              Quote(Join(ds, "c_min")) + " = " + Num(out.c_min));
  }

  const std::string dw = d + ".closure";
  const YAML::Node closure = rd.Section(dock, "closure", d);
  rd.CheckKeys(closure, dw, {"delta_lo", "delta_hi", "sigma_tau"});
  out.delta_lo = rd.Decision(closure, "delta_lo", dw, Closed(-2.0, 2.0));
  out.delta_hi = rd.Decision(closure, "delta_hi", dw, Closed(-2.0, 2.0));
  out.sigma_tau = rd.Decision(closure, "sigma_tau", dw, Closed(0.0, 1.0));
  if (std::isfinite(out.delta_lo) && std::isfinite(out.delta_hi) &&
      !(out.delta_hi > out.delta_lo)) {
    rd.Reject(Quote(Join(dw, "delta_hi")) + " = " + Num(out.delta_hi) + " must exceed " +
              Quote(Join(dw, "delta_lo")) + " = " + Num(out.delta_lo));
  }

  const std::string di = d + ".impact";
  const YAML::Node impact = rd.Section(dock, "impact", d);
  rd.CheckKeys(impact, di, {"contact_point_hand", "restitution", "e_max", "p_max"});
  if (const YAML::Node v = impact["contact_point_hand"]; v && !IsTbd(v)) {
    std::array<double, 3> a{};
    rd.Elements(v, Join(di, "contact_point_hand"), Closed(-0.5, 0.5), a);
    out.contact_point_hand = Eigen::Vector3d(a[0], a[1], a[2]);
  }
  out.restitution = rd.Decision(impact, "restitution", di, Closed(0.0, 1.0));
  // Not decisions: absent leaves the row off, and `TBD` is refused.
  out.e_max = rd.Number(impact, "e_max", di, out.e_max, Positive());
  out.p_max = rd.Number(impact, "p_max", di, out.p_max, Positive());
  return out;
}

void ApplyHandDocking(const HandDockingParams& hand, double t_close_e2e_s, double t_close_lead_s,
                      double ball_mass_kg, MpcDockingSegmentCoreParams& core) noexcept {
  core.s_ent = hand.s_ent;
  core.r_ent = hand.r_ent;
  core.tan_theta = hand.tan_theta;
  core.n_faces = hand.n_faces;
  const int n = std::clamp(hand.n_faces, 0, kMaxDockingFaces);
  for (int i = 0; i < n; ++i) {
    const auto k = static_cast<std::size_t>(i);
    core.face_a[k] = hand.face_a[k];
    core.face_b[k] = hand.face_b[k];
  }
  core.rho_ref = hand.rho_ref;
  core.c_min = hand.c_min;
  core.c_cap_max = hand.c_cap_max;
  core.c_ent_max = hand.c_ent_max;
  core.v_perp_max = hand.v_perp_max;
  core.a_brake = hand.a_brake;
  core.delta_lo = hand.delta_lo;
  core.delta_hi = hand.delta_hi;
  core.sigma_tau = hand.sigma_tau;
  core.delta_0 = t_close_e2e_s - t_close_lead_s;
  core.contact_point_hand = hand.contact_point_hand;
  core.m_ball = ball_mass_kg;
  core.restitution = hand.restitution;
  core.e_max = hand.e_max;
  core.p_max = hand.p_max;
}

// ═══ The docking core's tuning ═══════════════════════════════════════════════

namespace {

void ReadCore(const Reader& rd, const YAML::Node& m, const std::string& path, int nv,
              MpcDockingSegmentCoreParams& core) {
  rd.CheckKeys(m, path,
               {"u_scale", "cost", "catch", "approach", "capture", "timing", "stop", "rows",
                "catch_time", "sqp", "init", "solver"});
  const auto sub = [&](const char* name, std::initializer_list<const char*> keys) {
    const YAML::Node n = rd.Section(m, name, path);
    rd.CheckKeys(n, Join(path, name), keys);
    return n;
  };
  // One scalar for every joint; `zero_is_off` leaves the vector empty (the
  // core's "off") for an exact 0.
  const auto uniform = [&](const YAML::Node& sec, const char* key, const std::string& base,
                           const Range& r, bool zero_is_off, Eigen::VectorXd& dst) {
    const YAML::Node v = sec[key];
    if (!v) {
      return;
    }
    const double x = rd.Check(v, Join(base, key), r);
    if (zero_is_off && x == 0.0) {
      dst = Eigen::VectorXd();
    } else {
      dst = Eigen::VectorXd::Constant(nv, x);
    }
  };

  core.u_scale = rd.Number(m, "u_scale", path, core.u_scale, Positive());

  const std::string pc = Join(path, "cost");
  const YAML::Node cost = sub("cost", {"r_tau", "r_acc", "r_jerk", "w_q_nom", "w_manip",
                                       "manip_d_lin", "manip_d_ang", "manip_delta"});
  uniform(cost, "r_tau", pc, NonNegative(), true, core.r_tau);
  uniform(cost, "r_acc", pc, NonNegative(), true, core.r_acc);
  uniform(cost, "r_jerk", pc, Positive(), false, core.r_jerk);
  uniform(cost, "w_q_nom", pc, NonNegative(), true, core.w_q_nom);
  core.w_manip = rd.Number(cost, "w_manip", pc, core.w_manip, NonNegative());
  core.manip_d_lin = rd.Number(cost, "manip_d_lin", pc, core.manip_d_lin, Positive());
  core.manip_d_ang = rd.Number(cost, "manip_d_ang", pc, core.manip_d_ang, Positive());
  core.manip_delta = rd.Number(cost, "manip_delta", pc, core.manip_delta, Positive());

  const std::string pt = Join(path, "catch");
  const YAML::Node catch_map =
      sub("catch", {"q_p", "q_v", "sigma_T", "nu_ref", "q_rho_f", "q_nu_f", "w_impact", "e_ref"});
  {
    std::array<double, 3> a3{};
    std::array<double, 2> a2{};
    if (rd.Array(catch_map, "q_p", pt, NonNegative(), a3)) {
      core.q_p = Eigen::Vector3d(a3[0], a3[1], a3[2]);
    }
    if (rd.Array(catch_map, "q_v", pt, NonNegative(), a3)) {
      core.q_v = Eigen::Vector3d(a3[0], a3[1], a3[2]);
    }
    core.sigma_T = rd.Number(catch_map, "sigma_T", pt, core.sigma_T, Positive());
    if (rd.Array(catch_map, "nu_ref", pt, AnyFinite(), a3)) {
      if (!(-a3[2] > 0.0)) {
        rd.Reject(Quote(Join(pt, "nu_ref")) + " has -nu_ref[2] = " + Num(-a3[2]) +
                  ", which must be > 0 (the ball approaches along the axis)");
      }
      core.nu_ref = Eigen::Vector3d(a3[0], a3[1], a3[2]);
    }
    if (rd.Array(catch_map, "q_rho_f", pt, NonNegative(), a2)) {
      core.q_rho_f = Eigen::Vector2d(a2[0], a2[1]);
    }
    if (rd.Array(catch_map, "q_nu_f", pt, NonNegative(), a3)) {
      core.q_nu_f = Eigen::Vector3d(a3[0], a3[1], a3[2]);
    }
  }
  core.w_impact = rd.Number(catch_map, "w_impact", pt, core.w_impact, NonNegative());
  core.e_ref = rd.Number(catch_map, "e_ref", pt, core.e_ref, Positive());

  const std::string pa = Join(path, "approach");
  const YAML::Node approach =
      sub("approach", {"window", "lambda1_c", "lambda2_c", "lambda1_v", "lambda2_v"});
  core.approach_window =
      rd.Number(approach, "window", pa, core.approach_window, {0.0, true, 5.0, false});
  core.lambda1_c = rd.Number(approach, "lambda1_c", pa, core.lambda1_c, NonNegative());
  core.lambda2_c = rd.Number(approach, "lambda2_c", pa, core.lambda2_c, NonNegative());
  core.lambda1_v = rd.Number(approach, "lambda1_v", pa, core.lambda1_v, NonNegative());
  core.lambda2_v = rd.Number(approach, "lambda2_v", pa, core.lambda2_v, NonNegative());
  // A slack with no penalty is free, which switches its row off.
  if (!(core.lambda1_c + core.lambda2_c > 0.0)) {
    rd.Reject(Quote(Join(pa, "lambda1_c")) + " and " + Quote(Join(pa, "lambda2_c")) +
              " must sum to more than 0 (a free slack would switch its row off)");
  }
  if (!(core.lambda1_v + core.lambda2_v > 0.0)) {
    rd.Reject(Quote(Join(pa, "lambda1_v")) + " and " + Quote(Join(pa, "lambda2_v")) +
              " must sum to more than 0 (a free slack would switch its row off)");
  }

  const std::string pp = Join(path, "capture");
  const YAML::Node capture =
      sub("capture", {"chance", "face_eps", "speed_faces", "eps_nu", "eps_sigma"});
  core.chance = rd.Bool(capture, "chance", pp, core.chance);
  if (const YAML::Node v = capture["face_eps"]; v) {
    // One risk for every face, written to every entry the core can hold.
    core.face_eps.fill(rd.Check(v, Join(pp, "face_eps"), {0.0, true, 0.5, true}));
  }
  core.speed_faces = rd.Int(capture, "speed_faces", pp, core.speed_faces, 3, kMaxDockingSpeedFaces);
  core.eps_nu = rd.Number(capture, "eps_nu", pp, core.eps_nu, {0.0, true, 0.5, true});
  core.eps_sigma = rd.Number(capture, "eps_sigma", pp, core.eps_sigma, Positive());

  const std::string pm = Join(path, "timing");
  const YAML::Node timing = sub("timing", {"row", "eps_t"});
  core.timing_row = rd.Bool(timing, "row", pm, core.timing_row);
  core.eps_t = rd.Number(timing, "eps_t", pm, core.eps_t, {0.0, true, 0.5, true});

  const std::string ps = Join(path, "stop");
  const YAML::Node stop = sub("stop", {"r_jerk", "w_perp"});
  uniform(stop, "r_jerk", ps, Positive(), false, core.r_jerk_stop);
  core.w_perp = rd.Number(stop, "w_perp", ps, core.w_perp, NonNegative());

  const std::string pr = Join(path, "rows");
  const YAML::Node rows = sub("rows", {"accel_box", "jerk_box"});
  core.accel_box = rd.Bool(rows, "accel_box", pr, core.accel_box);
  core.jerk_box = rd.Bool(rows, "jerk_box", pr, core.jerk_box);

  const std::string pk = Join(path, "catch_time");
  const YAML::Node catch_time =
      sub("catch_time", {"delta_t_step", "mu_init_post_box", "mu_init_terminal"});
  core.delta_t_step = rd.Number(catch_time, "delta_t_step", pk, core.delta_t_step, Positive());
  core.mu_init_post_box =
      rd.Number(catch_time, "mu_init_post_box", pk, core.mu_init_post_box, Positive());
  core.mu_init_terminal =
      rd.Number(catch_time, "mu_init_terminal", pk, core.mu_init_terminal, Positive());

  const std::string pq = Join(path, "sqp");
  const YAML::Node sqp = sub(
      "sqp", {"max_iterations", "armijo_eta", "backtrack_beta", "max_backtracks", "delta_tr",
              "mu_init", "mu_growth", "mu_max", "mu_min_gain", "stall_window", "stall_reduction",
              "tol_violation", "tol_kkt", "tol_complementarity", "tol_linear"});
  core.max_iterations = rd.Int(sqp, "max_iterations", pq, core.max_iterations, 1, 10000);
  core.armijo_eta = rd.Number(sqp, "armijo_eta", pq, core.armijo_eta, {0.0, true, 1.0, true});
  core.backtrack_beta =
      rd.Number(sqp, "backtrack_beta", pq, core.backtrack_beta, {0.0, true, 1.0, true});
  core.max_backtracks = rd.Int(sqp, "max_backtracks", pq, core.max_backtracks, 0, 100);
  core.delta_tr = rd.Number(sqp, "delta_tr", pq, core.delta_tr, Positive());
  if (const YAML::Node v = sqp["mu_init"]; v) {
    // One penalty for every elastic group.
    core.mu_init.fill(rd.Check(v, Join(pq, "mu_init"), Positive()));
  }
  core.mu_growth = rd.Number(sqp, "mu_growth", pq, core.mu_growth, {1.0, true, kInf, false});
  core.mu_max = rd.Number(sqp, "mu_max", pq, core.mu_max, Positive());
  core.mu_min_gain = rd.Number(sqp, "mu_min_gain", pq, core.mu_min_gain, {0.0, true, 1.0, true});
  core.stall_window = rd.Int(sqp, "stall_window", pq, core.stall_window, 1, kDockingStallHistory);
  core.stall_reduction =
      rd.Number(sqp, "stall_reduction", pq, core.stall_reduction, {0.0, true, 1.0, true});
  core.tol_violation = rd.Number(sqp, "tol_violation", pq, core.tol_violation, Positive());
  core.tol_kkt = rd.Number(sqp, "tol_kkt", pq, core.tol_kkt, Positive());
  core.tol_complementarity =
      rd.Number(sqp, "tol_complementarity", pq, core.tol_complementarity, Positive());
  core.tol_linear = rd.Number(sqp, "tol_linear", pq, core.tol_linear, Positive());

  const std::string pi = Join(path, "init");
  const YAML::Node init = sub("init", {"w_q", "w_v", "pinv_damping"});
  core.init_w_q = rd.Number(init, "w_q", pi, core.init_w_q, Positive());
  core.init_w_v = rd.Number(init, "w_v", pi, core.init_w_v, NonNegative());
  core.init_pinv_damping =
      rd.Number(init, "pinv_damping", pi, core.init_pinv_damping, NonNegative());

  const std::string pv = Join(path, "solver");
  const YAML::Node solver = sub("solver", {"eps_abs", "eps_rel", "max_iter"});
  core.solver.eps_abs = rd.Number(solver, "eps_abs", pv, core.solver.eps_abs, Positive());
  core.solver.eps_rel = rd.Number(solver, "eps_rel", pv, core.solver.eps_rel, NonNegative());
  core.solver.max_iter = rd.Int(solver, "max_iter", pv, core.solver.max_iter, 1, 1000000);
}

}  // namespace

void ReadDockingCoreParams(const YAML::Node& core_map, const std::string& path, int nv,
                           MpcDockingSegmentCoreParams& core) {
  const Reader rd("ReadDockingCoreParams", true);
  if (!core_map || core_map.IsNull()) {
    return;
  }
  if (!core_map.IsMap()) {
    rd.Reject(Quote(path) + " must be a map, got " + Spelling(core_map));
  }
  if (nv < 1 || nv > kMaxPlanNv) {
    rd.Reject("nv = " + std::to_string(nv) +
              " (the joint count the per-joint weights are sized "
              "by) is outside [1, " +
              std::to_string(kMaxPlanNv) + "]");
  }
  ReadCore(rd, core_map, path, nv, core);
}

// ═══ The nlp search ══════════════════════════════════════════════════════════

NlpCatchSearchParams ParseNlpSearchParams(const YAML::Node& catching, int nv) {
  const Reader rd("ParseNlpSearchParams", true);
  RequireCatching(rd, catching);
  NlpCatchSearchParams out;
  const YAML::Node planner = rd.Section(catching, "planner", "");
  const YAML::Node search = rd.Section(planner, "search", "planner");
  const YAML::Node nlp = rd.Section(search, "nlp", "planner.search");
  const std::string b = "planner.search.nlp";
  // `mode` is the sibling `planner.search.mode`'s and `ik` is the sub-map the
  // catch-pose parser reads: neither is this parser's, neither is an error.
  // `catch_box` is the removed catch box (MD-94): not read, and not this
  // parser's to refuse — the binding parks on it (kRemovedCatchingKeys), which
  // keeps the robot up where a throw here would fail the whole configure.
  rd.CheckKeys(nlp, b,
               {"cand_dt", "t_lead_min", "t_max", "cand_capacity", "catch_box", "n_pre", "dt_pre_s",
                "stop", "budget", "cost", "continuous_tc", "follow_window", "rest_tol",
                "rt_state_age_max_s", "core", "mode", "ik", "catchability"});

  // ── Candidates ──
  out.cand_dt = rd.Number(nlp, "cand_dt", b, out.cand_dt, Closed(0.001, 0.2));
  out.t_lead_min = rd.Number(nlp, "t_lead_min", b, out.t_lead_min, Closed(0.01, 2.0));
  out.t_max = rd.Number(nlp, "t_max", b, out.t_max, Closed(0.05, 3.0));
  if (!(out.t_max > out.t_lead_min)) {
    rd.Reject(Quote(Join(b, "t_max")) + " = " + Num(out.t_max) + " must exceed " +
              Quote(Join(b, "t_lead_min")) + " = " + Num(out.t_lead_min));
  }
  out.cand_capacity = rd.Int(nlp, "cand_capacity", b, out.cand_capacity, 1, kNlpMaxCandidates);

  // ── The arm grid ──
  const std::string pn = Join(b, "n_pre");
  const YAML::Node n_pre = rd.Section(nlp, "n_pre", b);
  rd.CheckKeys(n_pre, pn, {"min", "max"});
  out.n_pre_min = rd.Int(n_pre, "min", pn, out.n_pre_min, 1, kMaxSegmentNodes);
  out.n_pre_max = rd.Int(n_pre, "max", pn, out.n_pre_max, 1, kMaxSegmentNodes);
  if (out.n_pre_max < out.n_pre_min) {
    rd.Reject(Quote(Join(pn, "max")) + " = " + std::to_string(out.n_pre_max) +
              " must be at least " + Quote(Join(pn, "min")) + " = " +
              std::to_string(out.n_pre_min));
  }
  out.dt_pre = rd.DtSeconds(nlp, "dt_pre_s", b, out.dt_pre, Closed(0.005, 0.5));
  const std::string ps = Join(b, "stop");
  ReadStopGrid(rd, rd.Section(nlp, "stop", b), ps, out.n_stop, out.dt_stop, out.n_stop_blocks,
               out.stop_block_sizes);
  // `Blocks` leaves the tail of the array as it was; the core's grid is the
  // first n_stop_blocks entries, and the rest are zero.
  for (std::size_t i = static_cast<std::size_t>(out.n_stop_blocks); i < out.stop_block_sizes.size();
       ++i) {
    out.stop_block_sizes[i] = 0;
  }
  if (out.n_pre_max + out.n_stop > kMaxSegmentNodes) {
    rd.Reject(Quote(Join(pn, "max")) + " = " + std::to_string(out.n_pre_max) + " plus " +
              Quote(Join(ps, "n_nodes")) + " = " + std::to_string(out.n_stop) +
              " is above the node capacity kMaxSegmentNodes = " + std::to_string(kMaxSegmentNodes));
  }

  // ── Budget ──
  const std::string pg = Join(b, "budget");
  const YAML::Node budget = rd.Section(nlp, "budget", b);
  rd.CheckKeys(budget, pg, {"budget_s", "solve_s", "start_lead_s", "max_solves"});
  out.budget_s = rd.Number(budget, "budget_s", pg, out.budget_s, Closed(0.001, 5.0));
  out.solve_budget_s = rd.Number(budget, "solve_s", pg, out.solve_budget_s, Closed(0.0005, 5.0));
  if (out.solve_budget_s > out.budget_s) {
    rd.Reject(Quote(Join(pg, "solve_s")) + " = " + Num(out.solve_budget_s) + " must not exceed " +
              Quote(Join(pg, "budget_s")) + " = " + Num(out.budget_s));
  }
  out.start_lead_s = rd.Number(budget, "start_lead_s", pg, out.start_lead_s, Closed(0.0, 0.1));
  out.max_solves = rd.Int(budget, "max_solves", pg, out.max_solves, 1, kNlpMaxSolves);

  // ── Outer cost and rank ──
  const std::string pw = Join(b, "cost");
  const YAML::Node cost = rd.Section(nlp, "cost", b);
  rd.CheckKeys(cost, pw, {"w_time", "w_switch", "rank_w_q", "rank_w_manip", "t_ref_s"});
  out.w_time = rd.Number(cost, "w_time", pw, out.w_time, NonNegative());
  out.w_switch = rd.Number(cost, "w_switch", pw, out.w_switch, NonNegative());
  out.rank_w_q = rd.Number(cost, "rank_w_q", pw, out.rank_w_q, NonNegative());
  out.rank_w_manip = rd.Number(cost, "rank_w_manip", pw, out.rank_w_manip, NonNegative());
  out.t_ref_s = rd.Number(cost, "t_ref_s", pw, out.t_ref_s, Positive());

  out.continuous_tc = rd.Bool(nlp, "continuous_tc", b, out.continuous_tc);
  out.follow_window =
      rd.Int(nlp, "follow_window", b, out.follow_window, -1, std::numeric_limits<int>::max());
  out.rest_tol = rd.Number(nlp, "rest_tol", b, out.rest_tol, {0.0, true, 1.0, false});
  out.rt_state_age_max_s =
      rd.Number(nlp, "rt_state_age_max_s", b, out.rt_state_age_max_s, {0.0, true, 1.0, false});

  ReadDockingCoreParams(nlp.IsMap() ? nlp["core"] : YAML::Node(), Join(b, "core"), nv, out.core);
  return out;
}

// ═══ The mpc_docking segment planner ═════════════════════════════════════════

MpcDockingSegmentPlannerParams ParseMpcDockingSegmentParams(const YAML::Node& catching, int nv) {
  const Reader rd("ParseMpcDockingSegmentParams", true);
  RequireCatching(rd, catching);
  MpcDockingSegmentPlannerParams out;
  const YAML::Node planner = rd.Section(catching, "planner", "");
  const YAML::Node segment = rd.Section(planner, "segment", "planner");
  const YAML::Node mpc = rd.Section(segment, "mpc_docking", "planner.segment");
  const std::string b = "planner.segment.mpc_docking";
  rd.CheckKeys(
      mpc, b,
      {"switch_margin", "eta_v", "approach", "stop", "budget", "replan", "publish", "core"});

  out.switch_margin = rd.Number(mpc, "switch_margin", b, out.switch_margin, Positive());
  out.eta_v = rd.Number(mpc, "eta_v", b, out.eta_v, {0.0, true, 1.0, true});

  const std::string pa = Join(b, "approach");
  const YAML::Node approach = rd.Section(mpc, "approach", b);
  rd.CheckKeys(approach, pa, {"n_pre_max", "dt_pre_s", "rest_tol"});
  out.n_pre_max = rd.Int(approach, "n_pre_max", pa, out.n_pre_max, 1, kMaxSegmentNodes);
  out.dt_pre_s = rd.DtSeconds(approach, "dt_pre_s", pa, out.dt_pre_s, Closed(0.005, 0.5));
  out.rest_tol = rd.Number(approach, "rest_tol", pa, out.rest_tol, {0.0, true, 1.0, false});

  const std::string ps = Join(b, "stop");
  ReadStopGrid(rd, rd.Section(mpc, "stop", b), ps, out.n_stop, out.dt_stop_s, out.n_stop_blocks,
               out.stop_block_sizes);
  for (std::size_t i = static_cast<std::size_t>(out.n_stop_blocks); i < out.stop_block_sizes.size();
       ++i) {
    out.stop_block_sizes[i] = 0;
  }
  if (out.n_pre_max + out.n_stop > kMaxSegmentNodes) {
    rd.Reject(Quote(Join(pa, "n_pre_max")) + " = " + std::to_string(out.n_pre_max) + " plus " +
              Quote(Join(ps, "n_nodes")) + " = " + std::to_string(out.n_stop) +
              " is above the node capacity kMaxSegmentNodes = " + std::to_string(kMaxSegmentNodes));
  }

  const std::string pg = Join(b, "budget");
  const YAML::Node budget = rd.Section(mpc, "budget", b);
  rd.CheckKeys(budget, pg, {"first_s", "replan_s"});
  out.budget_first_s = rd.Number(budget, "first_s", pg, out.budget_first_s, Closed(0.001, 5.0));
  out.budget_replan_s = rd.Number(budget, "replan_s", pg, out.budget_replan_s, Closed(0.001, 5.0));

  const std::string pr = Join(b, "replan");
  const YAML::Node replan = rd.Section(mpc, "replan", b);
  rd.CheckKeys(replan, pr, {"same_point"});
  out.replan_same_point = rd.Bool(replan, "same_point", pr, out.replan_same_point);

  const std::string pu = Join(b, "publish");
  const YAML::Node publish = rd.Section(mpc, "publish", b);
  rd.CheckKeys(publish, pu, {"slack_c_max", "slack_v_max"});
  out.slack_c_max = rd.Number(publish, "slack_c_max", pu, out.slack_c_max, NonNegative());
  out.slack_v_max = rd.Number(publish, "slack_v_max", pu, out.slack_v_max, NonNegative());

  ReadDockingCoreParams(mpc.IsMap() ? mpc["core"] : YAML::Node(), Join(b, "core"), nv, out.core);
  return out;
}

// ═══ The two grids agree ═════════════════════════════════════════════════════

const char* DockingGridMismatch(const NlpCatchSearchParams& search,
                                const MpcDockingSegmentPlannerParams& planner) noexcept {
  const auto ns = [](double s) { return static_cast<std::int64_t>(std::llround(s * 1e9)); };
  if (ns(search.dt_pre) != planner.DtPreNs()) {
    return "planner.search.nlp.dt_pre_s differs from planner.segment.mpc_docking.approach.dt_pre_s";
  }
  if (search.n_stop != planner.n_stop) {
    return "planner.search.nlp.stop.n_nodes differs from "
           "planner.segment.mpc_docking.stop.n_nodes";
  }
  if (ns(search.dt_stop) != planner.DtStopNs()) {
    return "planner.search.nlp.stop.dt_s differs from planner.segment.mpc_docking.stop.dt_s";
  }
  bool blocks_equal = search.n_stop_blocks == planner.n_stop_blocks;
  if (blocks_equal) {
    const int n = std::clamp(search.n_stop_blocks, 0, kMaxSegmentNodes);
    for (int i = 0; i < n; ++i) {
      const auto k = static_cast<std::size_t>(i);
      blocks_equal = blocks_equal && search.stop_block_sizes[k] == planner.stop_block_sizes[k];
    }
  }
  if (!blocks_equal) {
    return "planner.search.nlp.stop.blocks differs from planner.segment.mpc_docking.stop.blocks";
  }
  if (search.n_pre_max > planner.n_pre_max) {
    return "planner.search.nlp.n_pre.max exceeds planner.segment.mpc_docking.approach.n_pre_max";
  }
  return nullptr;
}

}  // namespace rtc::catching
