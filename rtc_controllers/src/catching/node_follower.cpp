// RT sampler of the decel MPC's joint nodes (E1-F02). See node_follower.hpp.
#include "rtc_controllers/catching/node_follower.hpp"

#include "rtc_controllers/catching/jerk_segment.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <cstddef>

namespace rtc::catching {

namespace {

using NodeMap = Eigen::Map<const Eigen::MatrixXd, 0, Eigen::OuterStride<>>;
using OutMap = Eigen::Map<Eigen::VectorXd>;

// Shape a sampler can index without going out of bounds.
[[nodiscard]] bool ShapeOk(const DecelPlanSnapshot& p) noexcept {
  return p.nv >= 1 && p.nv <= kMaxDecelNv && p.n_nodes >= 1 && p.n_nodes <= kMaxDecelNodes &&
         p.dt_ns > 0;
}

}  // namespace

bool NodeTrajectoryFollower::Init(std::shared_ptr<const pinocchio::Model> arm,
                                  pinocchio::FrameIndex frame,
                                  std::span<const int> device_of_model) {
  model_.reset();
  nv_ = 0;
  if (!arm || arm->nq != arm->nv || arm->nv < 1 || arm->nv > kMaxDecelNv ||
      frame >= static_cast<pinocchio::FrameIndex>(arm->nframes) ||
      device_of_model.size() != static_cast<std::size_t>(arm->nv)) {
    return false;
  }
  std::array<bool, kMaxDecelNv> seen{};
  for (int m = 0; m < arm->nv; ++m) {
    const int d = device_of_model[static_cast<std::size_t>(m)];
    if (d < 0 || d >= arm->nv || seen[static_cast<std::size_t>(d)]) {
      return false;
    }
    seen[static_cast<std::size_t>(d)] = true;
    device_of_model_[static_cast<std::size_t>(m)] = d;
  }
  data_ = pinocchio::Data(*arm);
  frame_ = frame;
  nv_ = arm->nv;
  q_model_ = Eigen::VectorXd::Zero(nv_);
  v_model_ = Eigen::VectorXd::Zero(nv_);
  model_ = std::move(arm);
  return true;
}

bool NodeTrajectoryFollower::SampleJoints(const DecelPlanSnapshot& plan, std::int64_t t_lead_ns,
                                          std::span<double> q, std::span<double> qd,
                                          std::span<double> qdd, bool* held) noexcept {
  if (!ShapeOk(plan)) {
    return false;
  }
  const auto n = static_cast<std::size_t>(plan.nv);
  if (q.size() < n || qd.size() < n || qdd.size() < n) {
    return false;
  }
  // Integer difference first: both instants are absolute ns (~1e18), whose
  // double conversion alone would lose ~100 ns before the subtraction.
  const std::int64_t since_ns = t_lead_ns - plan.t0_ns;
  if (since_ns < 0) {
    return false;
  }
  const Eigen::Index rows = plan.nv;
  const Eigen::Index cols = plan.n_nodes + 1;
  const Eigen::OuterStride<> stride(kMaxDecelNv);
  const NodeMap Q(plan.q.data(), rows, cols, stride);
  const NodeMap Qd(plan.qd.data(), rows, cols, stride);
  const NodeMap Qdd(plan.qdd.data(), rows, cols, stride);
  OutMap q_out(q.data(), rows);
  OutMap qd_out(qd.data(), rows);
  OutMap qdd_out(qdd.data(), rows);
  const double dt = static_cast<double>(plan.dt_ns) * 1e-9;
  const double t = static_cast<double>(since_ns) * 1e-9;
  if (!SampleJerkTrajectory(Q, Qd, Qdd, dt, t, q_out, qd_out, qdd_out)) {
    return false;
  }
  if (held != nullptr) {
    *held = since_ns >= static_cast<std::int64_t>(plan.n_nodes) * plan.dt_ns;
  }
  return true;
}

bool NodeTrajectoryFollower::Sample(const DecelPlanSnapshot& plan, std::int64_t t_lead_ns,
                                    DecelNodeSample& out) noexcept {
  if (model_ == nullptr || plan.nv != nv_) {
    return false;
  }
  // Evaluate into locals first: `out` must stay untouched on any failure.
  std::array<double, kMaxDecelNv> q{};
  std::array<double, kMaxDecelNv> qd{};
  std::array<double, kMaxDecelNv> qdd{};
  bool held = false;
  if (!SampleJoints(plan, t_lead_ns, q, qd, qdd, &held)) {
    return false;
  }
  for (int m = 0; m < nv_; ++m) {
    const auto d = static_cast<std::size_t>(device_of_model_[static_cast<std::size_t>(m)]);
    q_model_[m] = q[d];
    v_model_[m] = qd[d];
  }
  pinocchio::forwardKinematics(*model_, data_, q_model_, v_model_);
  pinocchio::updateFramePlacement(*model_, data_, frame_);
  const pinocchio::Motion v =
      pinocchio::getFrameVelocity(*model_, data_, frame_, pinocchio::LOCAL_WORLD_ALIGNED);
  out.q = q;
  out.qd = qd;
  out.qdd = qdd;
  out.placement = data_.oMf[frame_];
  out.twist.head<3>() = v.linear();
  out.twist.tail<3>() = v.angular();
  out.t_s = static_cast<double>(t_lead_ns - plan.t0_ns) * 1e-9;
  out.held = held;
  return true;
}

bool NodeTrajectoryFollower::NodesInsideBox(const DecelPlanSnapshot& plan,
                                            const std::array<double, 3>& lo,
                                            const std::array<double, 3>& hi,
                                            const std::array<double, 3>* anchor,
                                            int* first_outside) noexcept {
  if (first_outside != nullptr) {
    *first_outside = -1;
  }
  if (model_ == nullptr || plan.nv != nv_ || !ShapeOk(plan)) {
    return false;
  }
  // Node 0's position, the origin the anchored check measures from.
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  for (int k = 0; k <= plan.n_nodes; ++k) {
    for (int m = 0; m < nv_; ++m) {
      const auto d = static_cast<std::size_t>(device_of_model_[static_cast<std::size_t>(m)]);
      q_model_[m] = plan.q[static_cast<std::size_t>(k) * kMaxDecelNv + d];
    }
    pinocchio::forwardKinematics(*model_, data_, q_model_);
    pinocchio::updateFramePlacement(*model_, data_, frame_);
    const Eigen::Vector3d& fk = data_.oMf[frame_].translation();
    if (k == 0) {
      origin = fk;
    }
    for (int a = 0; a < 3; ++a) {
      const auto u = static_cast<std::size_t>(a);
      const double p = anchor != nullptr ? (*anchor)[u] + (fk[a] - origin[a]) : fk[a];
      // Written as "inside" so a NaN coordinate is outside.
      if (!(p >= lo[u] && p <= hi[u])) {
        if (first_outside != nullptr) {
          *first_outside = k;
        }
        return false;
      }
    }
  }
  return true;
}

bool NodeTrajectoryFollower::NodePosition(const DecelPlanSnapshot& plan, int k,
                                          std::array<double, 3>& p) noexcept {
  if (model_ == nullptr || plan.nv != nv_ || !ShapeOk(plan) || k < 0 || k > plan.n_nodes) {
    return false;
  }
  for (int m = 0; m < nv_; ++m) {
    const auto d = static_cast<std::size_t>(device_of_model_[static_cast<std::size_t>(m)]);
    q_model_[m] = plan.q[static_cast<std::size_t>(k) * kMaxDecelNv + d];
  }
  pinocchio::forwardKinematics(*model_, data_, q_model_);
  pinocchio::updateFramePlacement(*model_, data_, frame_);
  const Eigen::Vector3d& fk = data_.oMf[frame_].translation();
  for (int a = 0; a < 3; ++a) {
    p[static_cast<std::size_t>(a)] = fk[a];
  }
  return true;
}

}  // namespace rtc::catching
