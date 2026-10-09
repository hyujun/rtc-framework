// The reach bound of a frame. See reach_bound.hpp.
#include "rtc_controllers/catching/reach_bound.hpp"

#include <Eigen/Dense>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rtc::catching {
namespace {

// One joint of the chain, as the bound reads it.
struct Link {
  Eigen::Vector3d o;  ///< the joint's origin in the frame of the joint before it
  Eigen::Vector3d d;  ///< its axis in that same frame (R_i a_i)
  Eigen::Vector3d a;  ///< its axis in its own frame
  bool revolute{false};
  double stroke{0.0};  ///< a prismatic joint's largest |q|
};

constexpr double kAxisTol = 1e-9;
constexpr double kWeightFloor = 1e-9;
constexpr double kImprovementFloor = 1e-13;

// The joint's motion subspace at the neutral configuration: a revolute joint
// is one angular column, a prismatic one linear column. Returns false for
// anything else.
bool Classify(const pinocchio::Model& model, pinocchio::JointIndex j, const Eigen::VectorXd& q,
              Link& link) {
  const auto& jm = model.joints[j];
  if (jm.nv() != 1) {
    return false;
  }
  pinocchio::JointData jd = jm.createData();
  jm.calc(jd, q);
  const Eigen::Matrix<double, 6, Eigen::Dynamic> s = jd.S().matrix();
  if (s.cols() != 1) {
    return false;
  }
  const Eigen::Vector3d lin = s.col(0).head<3>();
  const Eigen::Vector3d ang = s.col(0).tail<3>();
  if (lin.norm() < kAxisTol && std::fabs(ang.norm() - 1.0) < kAxisTol) {
    link.revolute = true;
    link.a = ang;
    return true;
  }
  if (ang.norm() < kAxisTol && std::fabs(lin.norm() - 1.0) < kAxisTol && jm.nq() == 1) {
    const auto iq = static_cast<Eigen::Index>(jm.idx_q());
    const double lo = model.lowerPositionLimit[iq];
    const double hi = model.upperPositionLimit[iq];
    // A joint built without limits carries the largest double as its limit:
    // that is no stroke to add.
    const auto limit = [](double v) noexcept {
      return std::isfinite(v) && std::fabs(v) < std::numeric_limits<double>::max();
    };
    if (!limit(lo) || !limit(hi)) {
      return false;
    }
    link.revolute = false;
    link.a = lin;
    link.stroke = std::max(std::fabs(lo), std::fabs(hi));
    return true;
  }
  return false;
}

// Σ_i ‖term_i(s)‖ + strokes, where term_i = c_{i+1} − s_i a_i in joint i's
// frame and the last term is the frame's origin.
double Sum(const std::vector<Link>& chain, const Eigen::Vector3d& f, const Eigen::VectorXd& s) {
  double total = 0.0;
  const std::size_t n = chain.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector3d from = s[static_cast<Eigen::Index>(i)] * chain[i].a;
    const Eigen::Vector3d to =
        i + 1 < n
            ? Eigen::Vector3d(chain[i + 1].o + s[static_cast<Eigen::Index>(i + 1)] * chain[i + 1].d)
            : f;
    total += (to - from).norm() + chain[i].stroke;
  }
  return total;
}

}  // namespace

ReachBound ComputeReachBound(const pinocchio::Model& model, pinocchio::FrameIndex frame,
                             int iterations) {
  if (frame >= model.frames.size()) {
    throw std::invalid_argument("reach bound: frame index " + std::to_string(frame) +
                                " is not in the model");
  }
  ReachBound out;
  const pinocchio::JointIndex tip = model.frames[frame].parentJoint;
  const Eigen::Vector3d f = model.frames[frame].placement.translation();
  const Eigen::VectorXd q = pinocchio::neutral(model);

  std::vector<Link> chain;
  for (const pinocchio::JointIndex j : model.supports[tip]) {
    if (j == 0) {
      continue;  // the universe
    }
    Link link;
    link.o = model.jointPlacements[j].translation();
    if (!Classify(model, j, q, link)) {
      out.joints = static_cast<int>(chain.size()) + 1;
      out.unbounded_by = model.names[j];
      return out;  // radius stays infinite
    }
    link.d = model.jointPlacements[j].rotation() * link.a;
    chain.push_back(link);
  }
  out.joints = static_cast<int>(chain.size());
  const auto n = static_cast<Eigen::Index>(chain.size());
  if (n == 0) {
    out.centre = f;
    out.radius = 0.0;
    return out;
  }

  // The points' positions along their axes; a prismatic joint's stays 0.
  Eigen::VectorXd s = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd best_s = s;
  double best = Sum(chain, f, s);
  // Reweighted least squares on Σ ‖b_k + Σ_j s_j c_kj‖: term i depends on s_i
  // (coefficient −a_i) and s_{i+1} (coefficient d_{i+1}).
  for (int it = 0; it < iterations; ++it) {
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(n);
    for (Eigen::Index i = 0; i < n; ++i) {
      const auto u = static_cast<std::size_t>(i);
      const Eigen::Vector3d b = i + 1 < n ? chain[u + 1].o : f;
      Eigen::Vector3d r = b - s[i] * chain[u].a;
      if (i + 1 < n) {
        r += s[i + 1] * chain[u + 1].d;
      }
      const double w = 1.0 / std::max(r.norm(), kWeightFloor);
      const Eigen::Vector3d ci = -chain[u].a;
      g[i] += w * ci.dot(b);
      h(i, i) += w * ci.dot(ci);
      if (i + 1 < n) {
        const Eigen::Vector3d cj = chain[u + 1].d;
        g[i + 1] += w * cj.dot(b);
        h(i + 1, i + 1) += w * cj.dot(cj);
        h(i, i + 1) += w * ci.dot(cj);
        h(i + 1, i) += w * cj.dot(ci);
      }
    }
    // A prismatic joint's point does not move: its row and column are its own.
    for (Eigen::Index i = 0; i < n; ++i) {
      if (!chain[static_cast<std::size_t>(i)].revolute) {
        h.row(i).setZero();
        h.col(i).setZero();
        h(i, i) = 1.0;
        g[i] = 0.0;
      }
    }
    h.diagonal().array() += 1e-12;
    const Eigen::VectorXd next = h.ldlt().solve(-g);
    if (!next.allFinite()) {
      break;
    }
    s = next;
    const double total = Sum(chain, f, s);
    if (total < best - kImprovementFloor) {
      best = total;
      best_s = s;
    } else if (total >= best) {
      break;  // converged (or wandering): the best so far is kept
    }
  }
  out.centre = chain[0].o + best_s[0] * chain[0].d;  // joint 1's parent is the universe
  out.radius = best;
  return out;
}

}  // namespace rtc::catching
