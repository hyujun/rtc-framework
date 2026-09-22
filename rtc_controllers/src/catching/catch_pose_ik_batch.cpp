#include "rtc_controllers/catching/catch_pose_ik_batch.hpp"

#include "batch_csv.hpp"

#include <algorithm>
#include <array>
#include <istream>
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
  static const std::vector<std::string> kAxes = {"x", "y", "z"};
  std::vector<std::string> required = {"id"};
  for (const std::string& a : kAxes) {
    required.push_back("p_c_" + a);
  }
  for (const std::string& a : kAxes) {
    required.push_back("v_" + a);
  }
  std::vector<BatchCandidate> out;
  batch_csv::ReadHeadered(in, "candidate CSV", required, [&](const batch_csv::Row& row) {
    BatchCandidate c;
    c.id = row.Integer<std::int64_t>("id");
    if (row.Has("seed_id")) {
      c.seed_id = row.Integer<int>("seed_id");
    }
    for (int i = 0; i < 3; ++i) {
      const std::string& a = kAxes.at(static_cast<std::size_t>(i));
      c.p_c(i) = row.Finite("p_c_" + a);
      c.v_ball(i) = row.Finite("v_" + a);
    }
    out.push_back(c);
  });
  return out;
}

std::map<int, Eigen::VectorXd> ParseSeedCsv(std::istream& in) {
  std::map<int, Eigen::VectorXd> out;
  std::string line;
  int line_no = 0;
  std::size_t width = 0;
  // The optional header is the first line that CARRIES something, not physical
  // line 1: blank and `#` lines are skipped before it, so a file that opens
  // with a comment has its header on line 2 or later.
  bool first_content_line = true;
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
    const bool is_header = first_content_line && cells.front() == "seed_id";
    first_content_line = false;
    if (is_header) {
      continue;  // optional header
    }
    if (width == 0) {
      width = cells.size();
    } else if (cells.size() != width) {
      throw std::invalid_argument("line " + std::to_string(line_no) + ": expected " +
                                  std::to_string(width) + " columns, got " +
                                  std::to_string(cells.size()));
    }
    const int id = ParseIntCell<int>(cells.front(), "seed_id", line_no);
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

std::string BatchCsvRow(const BatchRow& row, int nv) {
  const CatchPoseIkResult& r = row.result;
  // q* exists only for the reasons that converged on a pose; for the others the
  // buffer was never written and printing it would publish uninitialised zeros
  // as a posture.
  const bool has_pose = r.reason == CatchPoseReason::kNone ||
                        r.reason == CatchPoseReason::kBelowManipMin ||
                        r.reason == CatchPoseReason::kRankDeficient;
  if (has_pose && r.nv != nv) {
    throw std::invalid_argument("candidate id " + std::to_string(row.candidate.id) +
                                " carries a pose of " + std::to_string(r.nv) +
                                " joints, but the CSV has " + std::to_string(nv) + " q columns");
  }
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
  // Exactly `nv` cells — the header's width. `r.nv` is NOT the bound: the
  // reasons `Solve` returns before it records an nv (kOptionsInvalid,
  // kModelInvalid, kJointOrderMismatch) leave it 0, and those rows must still
  // be as wide as every other or the reader rejects the whole file.
  for (int i = 0; i < nv; ++i) {
    s += ',';
    if (has_pose) {
      s += Num(r.q[static_cast<std::size_t>(i)]);
    }
  }
  return s;
}

pinocchio::FrameIndex ResolveCatchFrame(const pinocchio::Model& model,
                                        std::string_view frame_name) {
  const std::string name(frame_name);
  if (!model.existFrame(name)) {
    throw std::invalid_argument("model '" + model.name + "' has no frame '" + name +
                                "' (misspelt, not declared under extra_frames, or locked away "
                                "by the sub-model)");
  }
  const pinocchio::FrameIndex id = model.getFrameId(name);
  if (id == 0) {
    throw std::invalid_argument("frame '" + name + "' is the universe frame of model '" +
                                model.name + "', which cannot be a catch frame");
  }
  return id;
}

CatchingTree ResolveCatchingTree(const YAML::Node& root, const std::string& source) {
  const auto has_map = [](const YAML::Node& n, const char* key) {
    if (!n.IsMap()) {
      return false;
    }
    const YAML::Node child = n[key];
    return child && child.IsMap();
  };
  const bool root_has_catching = has_map(root, "catching");
  const bool root_has_planner = has_map(root, "planner");
  if (root_has_catching && root_has_planner) {
    throw std::invalid_argument("params file '" + source +
                                "' has both a top-level `catching:` and a top-level `planner:` "
                                "map — which one is the catching tree is ambiguous");
  }
  if (root_has_catching) {
    return {root["catching"], "catching"};
  }
  if (root_has_planner) {
    return {root, "<root>"};
  }
  if (root.IsMap() && root.size() == 1) {
    const auto only = root.begin();
    if (only->first.IsScalar() && has_map(only->second, "catching")) {
      return {only->second["catching"], only->first.Scalar() + ".catching"};
    }
  }
  throw std::invalid_argument(
      "params file '" + source +
      "' holds no catching tree: looked for a top-level `catching:` map, a single top-level "
      "`<controller>:` key whose value has a `catching:` map, or a top-level `planner:` map. "
      "Refusing to fall back to in-code defaults");
}

std::string FormatCatchPoseIkOptions(const CatchPoseIkOptions& opt) {
  std::string s;
  const auto num = [&s](const char* key, double v) { s += std::string(key) + ' ' + Num(v) + '\n'; };
  const auto integer = [&s](const char* key, int v) {
    s += std::string(key) + ' ' + std::to_string(v) + '\n';
  };
  integer("planner.ik.max_iter", opt.max_iter);
  num("planner.ik.eps_pos", opt.eps_pos);
  num("planner.ik.alpha_max", opt.alpha_max);
  num("planner.ik.rho", opt.rho);
  num("planner.ik.sigma0", opt.sigma0);
  num("planner.ik.lambda_max", opt.lambda_max);
  num("planner.ik.dq_step_max", opt.dq_step_max);
  num("planner.ik.mu", opt.mu);
  num("planner.ik.qp_eps_abs", opt.qp_eps_abs);
  integer("planner.ik.qp_max_iter", opt.qp_max_iter);
  num("planner.ik.k_null", opt.k_null);
  num("planner.ik.k_manip", opt.k_manip);
  num("planner.ik.manip_grad_tol", opt.manip_grad_tol);
  num("planner.ik.v_eps", opt.v_eps);
  num("fd_step", opt.fd_step);
  s += std::string("planner.catchability.definition ") +
       (opt.definition == ManipDefinition::kArm6Row ? "arm_6row" : "arm_5row") + '\n';
  num("manipulability_min", opt.manipulability_min);
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
