#include "rtc_controllers/catching/catch_gate_batch.hpp"

#include "batch_csv.hpp"
#include "rtc_controllers/catching/time_types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <istream>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace rtc::catching {
namespace {

using batch_csv::IsSkippable;
using batch_csv::Num;
using batch_csv::ParseFinite;
using batch_csv::ParseIntCell;
using batch_csv::SplitCsv;

constexpr double kSToNs = 1e9;

[[nodiscard]] std::int64_t ToNs(double seconds) noexcept {
  return static_cast<std::int64_t>(std::llround(seconds * kSToNs));
}

[[nodiscard]] bool PositiveFinite(double v) noexcept {
  return v > 0.0 && std::isfinite(v);
}

[[nodiscard]] bool NonNegativeFinite(double v) noexcept {
  return v >= 0.0 && std::isfinite(v);
}

[[nodiscard]] std::string Flag(bool v) {
  return v ? "1" : "0";
}

void AppendPoint(std::string& out, const StoppingReservation& stop) {
  out += ',' + Num(stop.distance);
  for (int i = 0; i < 3; ++i) {
    out += ',' + Num(stop.p_stop(i));
  }
  out += ',' + Flag(stop.valid);
}

}  // namespace

std::string_view GateReasonName(GateReason reason) noexcept {
  switch (reason) {
    case GateReason::kNone:
      return "none";
    case GateReason::kReachInvalid:
      return "reach_invalid";
    case GateReason::kReachTime:
      return "reach_time";
    case GateReason::kGammaInvalid:
      return "gamma_invalid";
    case GateReason::kGammaWindowEmpty:
      return "gamma_window_empty";
    case GateReason::kStopInvalid:
      return "stop_invalid";
  }
  return "unknown";
}

std::string ValidateGateSettings(const GateSettings& s, int nv) {
  if (nv <= 0) {
    return "posture width must be positive";
  }
  const auto n = static_cast<std::size_t>(nv);
  if (s.qdot_max.size() != n || s.qddot_max.size() != n) {
    return "qdot_max and qddot_max must each have " + std::to_string(nv) + " entries (got " +
           std::to_string(s.qdot_max.size()) + " and " + std::to_string(s.qddot_max.size()) + ")";
  }
  if (!std::all_of(s.qdot_max.begin(), s.qdot_max.end(), PositiveFinite) ||
      !std::all_of(s.qddot_max.begin(), s.qddot_max.end(), PositiveFinite)) {
    return "every joint velocity and acceleration limit must be positive and finite";
  }
  if (!(s.eta_v > 0.0) || !(s.eta_v <= 1.0)) {
    return "eta_v must be in (0, 1]";
  }
  if (!PositiveFinite(s.v_max) || !PositiveFinite(s.t_close_total) || !PositiveFinite(s.a_dec)) {
    return "v_max, t_close_total and a_dec must be positive and finite";
  }
  if (!NonNegativeFinite(s.d_eff) || !NonNegativeFinite(s.gamma_margin) ||
      !NonNegativeFinite(s.first_plan_s) || !NonNegativeFinite(s.t_arm_s) ||
      !NonNegativeFinite(s.t_margin_s)) {
    return "d_eff, gamma_margin, first_plan_s, t_arm_s and t_margin_s must be ≥ 0 and finite";
  }
  return {};
}

std::vector<GateCandidate> ParseGateCandidateCsv(std::istream& in, int nv) {
  if (nv <= 0) {
    throw std::invalid_argument("gate candidate CSV: posture width must be positive");
  }
  std::vector<std::string> required = {"id",  "t_c_s", "p_c_x", "p_c_y", "p_c_z", "v_x",
                                       "v_y", "v_z",   "jpu_x", "jpu_y", "jpu_z"};
  for (int i = 0; i < nv; ++i) {
    required.push_back("qs" + std::to_string(i));
    required.push_back("qu" + std::to_string(i));
  }
  std::vector<GateCandidate> out;
  std::string line;
  int line_no = 0;
  std::vector<std::string> header;
  while (std::getline(in, line)) {
    ++line_no;
    if (IsSkippable(line)) {
      continue;
    }
    if (header.empty()) {
      header = SplitCsv(line);
      for (const std::string& name : required) {
        if (std::find(header.begin(), header.end(), name) == header.end()) {
          throw std::invalid_argument("gate candidate CSV header lacks column '" + name + "'");
        }
      }
      // A posture one column wider than the seed is a different arm, not a
      // column to ignore.
      if (std::find(header.begin(), header.end(), "qs" + std::to_string(nv)) != header.end()) {
        throw std::invalid_argument("gate candidate CSV carries more than " + std::to_string(nv) +
                                    " posture columns");
      }
      continue;
    }
    const std::vector<std::string> cells = SplitCsv(line);
    if (cells.size() != header.size()) {
      throw std::invalid_argument("line " + std::to_string(line_no) + ": expected " +
                                  std::to_string(header.size()) + " columns, got " +
                                  std::to_string(cells.size()));
    }
    const auto cell = [&](const std::string& name) -> const std::string& {
      const auto it = std::find(header.begin(), header.end(), name);
      return cells.at(static_cast<std::size_t>(std::distance(header.begin(), it)));
    };
    const auto number = [&](const std::string& name) {
      return ParseFinite(cell(name), name, line_no);
    };
    GateCandidate c;
    c.id = ParseIntCell<std::int64_t>(cell("id"), "id", line_no);
    if (std::find(header.begin(), header.end(), "seed_id") != header.end()) {
      c.seed_id = ParseIntCell<int>(cell("seed_id"), "seed_id", line_no);
    }
    c.t_c_s = number("t_c_s");
    static constexpr std::array<const char*, 3> kAxis = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
      const std::string axis = kAxis.at(static_cast<std::size_t>(i));
      c.p_c(i) = number("p_c_" + axis);
      c.v_ball(i) = number("v_" + axis);
      c.jp_qdot_u(i) = number("jpu_" + axis);
    }
    c.q_star.resize(nv);
    c.qdot_u.resize(nv);
    for (int i = 0; i < nv; ++i) {
      c.q_star(i) = number("qs" + std::to_string(i));
      c.qdot_u(i) = number("qu" + std::to_string(i));
    }
    out.push_back(std::move(c));
  }
  if (header.empty()) {
    throw std::invalid_argument("gate candidate CSV is empty (a header line is required)");
  }
  return out;
}

std::string GateCsvHeader() {
  return "id,seed_id,accepted,reason_name,lead_s,t_min_s,reach_ok,reach_w0_clamped,"
         "reach_limits_invalid,reach_input_invalid,v_dir_max,dir_limits_invalid,"
         "dir_input_invalid,dir_undetermined,v_tcp_plan,g_min,g_max,window_input_invalid,"
         "max_catchable_m_s,gamma_ok,stop_gmin_distance,stop_gmin_x,stop_gmin_y,stop_gmin_z,"
         "stop_gmin_valid,stop_gmax_distance,stop_gmax_x,stop_gmax_y,stop_gmax_z,stop_gmax_valid";
}

std::string GateCsvRow(const GateRow& row) {
  std::string out = std::to_string(row.candidate.id) + ',' + std::to_string(row.candidate.seed_id);
  out += ',' + Flag(row.reason == GateReason::kNone);
  out += ',' + std::string(GateReasonName(row.reason));
  out += ',' + Num(row.lead_s) + ',' + Num(row.reach.t) + ',' + Flag(row.reach_ok);
  out += ',' + Flag(row.reach.w0_clamped) + ',' + Flag(row.reach.limits_invalid) + ',' +
         Flag(row.reach.input_invalid);
  out += ',' + Num(row.direction.v_dir_max) + ',' + Flag(row.direction.limits_invalid) + ',' +
         Flag(row.direction.input_invalid) + ',' + Flag(row.direction.undetermined);
  out += ',' + Num(row.v_tcp_plan) + ',' + Num(row.window.g_min) + ',' + Num(row.window.g_max) +
         ',' + Flag(row.window.input_invalid);
  out += ',' + Num(row.max_catchable) + ',' + Flag(row.gamma_ok);
  AppendPoint(out, row.stop_gamma_min);
  AppendPoint(out, row.stop_gamma_max);
  return out;
}

GateRow JudgeGates(const GateCandidate& candidate, const Eigen::VectorXd& q_wait,
                   const GateSettings& settings) {
  GateRow row;
  row.candidate = candidate;

  // Planning limits carry the η_v margin on the joints too (S4.4): with
  // v_dir,max binding instead of the TCP term, a margin on v_max alone would
  // leave the plan no buffer at all (D-9).
  std::vector<double> w_plan(settings.qdot_max);
  for (double& w : w_plan) {
    w *= settings.eta_v;
  }

  // ── reach time (L3 §4.3): wait pose at rest → (q*, 0) ──────────────────────
  const std::vector<double> w0(static_cast<std::size_t>(q_wait.size()), 0.0);
  row.reach = MaxJointTMin(
      std::span<const double>(q_wait.data(), static_cast<std::size_t>(q_wait.size())), w0,
      std::span<const double>(candidate.q_star.data(),
                              static_cast<std::size_t>(candidate.q_star.size())),
      w_plan, settings.qddot_max);
  const BallTime t_k{ToNs(candidate.t_c_s)};
  const NowLead now_lead =
      MakeNowLead(NowReal{ToNs(settings.first_plan_s)}, ToNs(settings.t_arm_s));
  row.lead_s = LeadSecondsUntil(now_lead, t_k);
  row.reach_ok = ReachTimeFeasible(t_k, now_lead, ToNs(settings.t_margin_s), row.reach);

  // ── γ window (L3 §4.5) ─────────────────────────────────────────────────────
  const double speed = candidate.v_ball.norm();
  row.v_tcp_plan = PlanningTcpSpeed(settings.eta_v, settings.v_max);
  bool gamma_usable = false;
  if (speed > 0.0 && std::isfinite(speed)) {
    const Eigen::Vector3d v_hat = candidate.v_ball / speed;
    row.direction = DirectionalSpeedMax(
        v_hat, candidate.jp_qdot_u,
        std::span<const double>(candidate.qdot_u.data(),
                                static_cast<std::size_t>(candidate.qdot_u.size())),
        w_plan);
    if (!row.direction.limits_invalid && !row.direction.input_invalid) {
      row.window = ComputeGammaWindow(speed, row.direction.v_dir_max, row.v_tcp_plan,
                                      settings.d_eff, settings.t_close_total);
      row.max_catchable = MaxCatchableSpeed(row.direction.v_dir_max, row.v_tcp_plan, settings.d_eff,
                                            settings.t_close_total);
      gamma_usable = !row.window.input_invalid;
    }
  } else {
    row.direction.input_invalid = true;
  }
  row.gamma_ok =
      gamma_usable && row.window.Feasible() && speed + settings.gamma_margin <= row.max_catchable;

  // ── stopping point (L3 §4.9), at both ends of the window ───────────────────
  if (gamma_usable) {
    row.stop_gamma_min =
        StoppingPoint(candidate.p_c, candidate.v_ball, row.window.g_min, settings.a_dec);
    row.stop_gamma_max =
        StoppingPoint(candidate.p_c, candidate.v_ball, row.window.g_max, settings.a_dec);
  }

  if (!row.reach.Usable() || !std::isfinite(row.reach.t)) {
    row.reason = GateReason::kReachInvalid;
  } else if (!row.reach_ok) {
    row.reason = GateReason::kReachTime;
  } else if (!gamma_usable) {
    row.reason = GateReason::kGammaInvalid;
  } else if (!row.gamma_ok) {
    row.reason = GateReason::kGammaWindowEmpty;
  } else if (!row.stop_gamma_min.valid) {
    row.reason = GateReason::kStopInvalid;
  } else {
    row.reason = GateReason::kNone;
  }
  return row;
}

std::vector<GateRow> RunGateBatch(const std::vector<GateCandidate>& candidates,
                                  const std::map<int, Eigen::VectorXd>& seeds,
                                  const GateSettings& settings) {
  if (seeds.empty()) {
    throw std::invalid_argument("no wait pose given");
  }
  const int nv = static_cast<int>(seeds.begin()->second.size());
  if (const std::string why = ValidateGateSettings(settings, nv); !why.empty()) {
    throw std::invalid_argument("gate settings: " + why);
  }
  std::vector<GateRow> out;
  out.reserve(candidates.size());
  for (const GateCandidate& c : candidates) {
    const auto seed = seeds.find(c.seed_id);
    if (seed == seeds.end()) {
      throw std::invalid_argument("candidate " + std::to_string(c.id) + " names seed " +
                                  std::to_string(c.seed_id) + ", which is absent");
    }
    if (seed->second.size() != nv || c.q_star.size() != nv || c.qdot_u.size() != nv) {
      throw std::invalid_argument("candidate " + std::to_string(c.id) +
                                  ": posture width differs from the wait pose's");
    }
    out.push_back(JudgeGates(c, seed->second, settings));
  }
  return out;
}

}  // namespace rtc::catching
