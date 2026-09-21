#include "rtc_controllers/catching/catch_pose_ik_batch.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <istream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rtc::catching {
namespace {

// %.17g round-trips every finite double exactly, which is the point: the map's
// numbers must be the solver's numbers. Shortest-round-trip would be prettier
// but std::to_chars for doubles is not uniformly available in this toolchain.
[[nodiscard]] std::string Num(double v) {
  std::array<char, 32> buf{};
  const int n = std::snprintf(buf.data(), buf.size(), "%.17g", v);
  if (n <= 0) {
    return "nan";
  }
  return std::string(buf.data(), static_cast<std::size_t>(n));
}

/// Split on ',' KEEPING every field, including empty ones at the end.
///
/// `std::getline(ss, cell, ',')` silently drops a trailing empty field, so a
/// row whose last columns are blank — which is exactly what a poseless result
/// writes — parses one column short and the width check then rejects a row
/// that is in fact well formed.
[[nodiscard]] std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = line.find(',', start);
    const std::string cell =
        line.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
    const auto b = cell.find_first_not_of(" \t\r\n");
    const auto e = cell.find_last_not_of(" \t\r\n");
    out.push_back(b == std::string::npos ? std::string() : cell.substr(b, e - b + 1));
    if (comma == std::string::npos) {
      return out;
    }
    start = comma + 1;
  }
}

/// True for a line that carries no data (blank, or a `#` comment).
[[nodiscard]] bool IsSkippable(const std::string& line) {
  const auto b = line.find_first_not_of(" \t\r\n");
  return b == std::string::npos || line[b] == '#';
}

[[nodiscard]] double ParseFinite(const std::string& cell, std::string_view what, int line_no) {
  double v = 0.0;
  try {
    std::size_t used = 0;
    v = std::stod(cell, &used);
    if (used != cell.size()) {
      throw std::invalid_argument("trailing characters");
    }
  } catch (const std::exception&) {
    throw std::invalid_argument("line " + std::to_string(line_no) + ": '" + std::string(what) +
                                "' is not a number: '" + cell + "'");
  }
  if (!std::isfinite(v)) {
    // A non-finite candidate would be judged kTargetNonFinite and land in the
    // map as a legitimate rejection, hiding a broken generator as physics.
    throw std::invalid_argument("line " + std::to_string(line_no) + ": '" + std::string(what) +
                                "' is not finite: '" + cell + "'");
  }
  return v;
}

[[nodiscard]] int ParseIntCell(const std::string& cell, std::string_view what, int line_no) {
  int v = 0;
  const char* first = cell.data();
  const char* last = cell.data() + cell.size();
  const auto res = std::from_chars(first, last, v);
  if (res.ec != std::errc() || res.ptr != last) {
    throw std::invalid_argument("line " + std::to_string(line_no) + ": '" + std::string(what) +
                                "' is not an integer: '" + cell + "'");
  }
  return v;
}

}  // namespace

std::string_view CatchPoseReasonName(CatchPoseReason reason) noexcept {
  switch (reason) {
    case CatchPoseReason::kNone:
      return "none";
    case CatchPoseReason::kOptionsInvalid:
      return "options_invalid";
    case CatchPoseReason::kModelInvalid:
      return "model_invalid";
    case CatchPoseReason::kJointOrderMismatch:
      return "joint_order_mismatch";
    case CatchPoseReason::kSeedNonFinite:
      return "seed_non_finite";
    case CatchPoseReason::kTargetNonFinite:
      return "target_non_finite";
    case CatchPoseReason::kVelocityNonFinite:
      return "velocity_non_finite";
    case CatchPoseReason::kSpeedTooLow:
      return "speed_too_low";
    case CatchPoseReason::kAxisAlignInvalid:
      return "axis_align_invalid";
    case CatchPoseReason::kJacobianNonFinite:
      return "jacobian_non_finite";
    case CatchPoseReason::kQpFailed:
      return "qp_failed";
    case CatchPoseReason::kNotConverged:
      return "not_converged";
    case CatchPoseReason::kRankDeficient:
      return "rank_deficient";
    case CatchPoseReason::kBelowManipMin:
      return "below_manip_min";
  }
  return "unknown";
}

std::vector<BatchCandidate> ParseCandidateCsv(std::istream& in) {
  static constexpr std::array<const char*, 6> kRequired = {"p_c_x", "p_c_y", "p_c_z",
                                                           "v_x",   "v_y",   "v_z"};
  std::vector<BatchCandidate> out;
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
      for (const char* name : kRequired) {
        if (std::find(header.begin(), header.end(), std::string(name)) == header.end()) {
          throw std::invalid_argument("candidate CSV header lacks column '" + std::string(name) +
                                      "'");
        }
      }
      if (std::find(header.begin(), header.end(), "id") == header.end()) {
        throw std::invalid_argument("candidate CSV header lacks column 'id'");
      }
      continue;
    }
    const std::vector<std::string> cells = SplitCsv(line);
    if (cells.size() != header.size()) {
      throw std::invalid_argument("line " + std::to_string(line_no) + ": expected " +
                                  std::to_string(header.size()) + " columns, got " +
                                  std::to_string(cells.size()));
    }
    const auto cell = [&](std::string_view name) -> const std::string& {
      const auto it = std::find(header.begin(), header.end(), std::string(name));
      return cells.at(static_cast<std::size_t>(std::distance(header.begin(), it)));
    };
    BatchCandidate c;
    c.id = static_cast<std::int64_t>(ParseIntCell(cell("id"), "id", line_no));
    if (std::find(header.begin(), header.end(), "seed_id") != header.end()) {
      c.seed_id = ParseIntCell(cell("seed_id"), "seed_id", line_no);
    }
    for (int i = 0; i < 3; ++i) {
      c.p_c(i) = ParseFinite(cell(kRequired.at(static_cast<std::size_t>(i))),
                             kRequired.at(static_cast<std::size_t>(i)), line_no);
      c.v_ball(i) = ParseFinite(cell(kRequired.at(static_cast<std::size_t>(i) + 3)),
                                kRequired.at(static_cast<std::size_t>(i) + 3), line_no);
    }
    out.push_back(c);
  }
  if (header.empty()) {
    throw std::invalid_argument("candidate CSV is empty (a header line is required)");
  }
  return out;
}

std::map<int, Eigen::VectorXd> ParseSeedCsv(std::istream& in) {
  std::map<int, Eigen::VectorXd> out;
  std::string line;
  int line_no = 0;
  std::size_t width = 0;
  while (std::getline(in, line)) {
    ++line_no;
    if (IsSkippable(line)) {
      continue;
    }
    const std::vector<std::string> cells = SplitCsv(line);
    if (cells.size() < 2) {
      throw std::invalid_argument("line " + std::to_string(line_no) +
                                  ": a seed row is 'seed_id,q0,...'");
    }
    if (line_no == 1 && !cells.empty() && cells.front() == "seed_id") {
      continue;  // optional header
    }
    if (width == 0) {
      width = cells.size();
    } else if (cells.size() != width) {
      throw std::invalid_argument("line " + std::to_string(line_no) + ": expected " +
                                  std::to_string(width) + " columns, got " +
                                  std::to_string(cells.size()));
    }
    const int id = ParseIntCell(cells.front(), "seed_id", line_no);
    Eigen::VectorXd q(static_cast<Eigen::Index>(cells.size() - 1));
    for (std::size_t i = 1; i < cells.size(); ++i) {
      q(static_cast<Eigen::Index>(i - 1)) =
          ParseFinite(cells[i], "q" + std::to_string(i - 1), line_no);
    }
    if (!out.emplace(id, std::move(q)).second) {
      throw std::invalid_argument("line " + std::to_string(line_no) + ": duplicate seed_id " +
                                  std::to_string(id));
    }
  }
  if (out.empty()) {
    throw std::invalid_argument("seed CSV is empty");
  }
  return out;
}

std::string BatchCsvHeader(int nv) {
  std::string h =
      "id,seed_id,accepted,reason,reason_name,iterations,pos_error,theta,w5,w6,w5_valid,w6_valid,"
      "manip_converged,manip_grad_norm,manip_grad_failures,sigma_min,lambda_sq,qp_status,"
      "qp_iterations,qp_failures,nv";
  for (int i = 0; i < nv; ++i) {
    h += ",q" + std::to_string(i);
  }
  return h;
}

std::string BatchCsvRow(const BatchRow& row) {
  const CatchPoseIkResult& r = row.result;
  std::string s;
  s += std::to_string(row.candidate.id);
  s += ',' + std::to_string(row.candidate.seed_id);
  s += ',' + std::to_string(static_cast<int>(r.accepted));
  s += ',' + std::to_string(static_cast<int>(r.reason));
  s += ',' + std::string(CatchPoseReasonName(r.reason));
  s += ',' + std::to_string(r.iterations);
  s += ',' + Num(r.pos_error);
  s += ',' + Num(r.theta);
  s += ',' + Num(r.w5);
  s += ',' + Num(r.w6);
  s += ',' + std::to_string(static_cast<int>(r.w5_valid));
  s += ',' + std::to_string(static_cast<int>(r.w6_valid));
  s += ',' + std::to_string(static_cast<int>(r.manip_converged));
  s += ',' + Num(r.manip_grad_norm);
  s += ',' + std::to_string(r.manip_grad_failures);
  s += ',' + Num(r.sigma_min);
  s += ',' + Num(r.lambda_sq);
  s += ',' + std::to_string(r.qp_status);
  s += ',' + std::to_string(r.qp_iterations);
  s += ',' + std::to_string(r.qp_failures);
  s += ',' + std::to_string(r.nv);
  // q* exists only for the reasons that converged on a pose; for the others the
  // buffer was never written and printing it would publish uninitialised zeros
  // as a posture.
  const bool has_pose = r.reason == CatchPoseReason::kNone ||
                        r.reason == CatchPoseReason::kBelowManipMin ||
                        r.reason == CatchPoseReason::kRankDeficient;
  for (int i = 0; i < r.nv; ++i) {
    s += ',';
    if (has_pose) {
      s += Num(r.q[static_cast<std::size_t>(i)]);
    }
  }
  return s;
}

std::vector<BatchRow> RunBatch(rtc_urdf_bridge::RtModelHandle& model,
                               pinocchio::FrameIndex catch_frame,
                               const std::vector<BatchCandidate>& candidates,
                               const std::map<int, Eigen::VectorXd>& seeds,
                               const CatchPoseIkOptions& opt) {
  CatchPoseIk ik;
  ik.Resize(model.nv());
  std::vector<BatchRow> out;
  out.reserve(candidates.size());
  for (const BatchCandidate& c : candidates) {
    const auto it = seeds.find(c.seed_id);
    if (it == seeds.end()) {
      throw std::invalid_argument("candidate id " + std::to_string(c.id) + " names seed_id " +
                                  std::to_string(c.seed_id) + ", which is not in the seed file");
    }
    if (it->second.size() != model.nv()) {
      throw std::invalid_argument("seed_id " + std::to_string(c.seed_id) + " has " +
                                  std::to_string(it->second.size()) + " entries, model nv is " +
                                  std::to_string(model.nv()));
    }
    BatchRow row;
    row.candidate = c;
    row.result = ik.Solve(model, catch_frame, c.p_c, c.v_ball, it->second, opt);
    out.push_back(std::move(row));
  }
  return out;
}

}  // namespace rtc::catching
