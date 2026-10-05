// RT sampler of the segment MPC's joint nodes (E1-F02). See node_follower.hpp.
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

// Shape a sampler can index without going out of bounds. The sampler does not
// run ValidateSegmentNodes, so this is its only guard on n_pre: a broken one
// would give the node maps a column count of zero or less.
[[nodiscard]] bool ShapeOk(const SegmentSnapshot& p) noexcept {
  return p.nv >= 1 && p.nv <= kMaxSegmentNv && p.n_nodes >= 1 && p.n_nodes <= kMaxSegmentNodes &&
         p.dt_ns > 0 && p.n_pre >= 0 && p.n_pre < p.n_nodes &&
         (p.n_pre == 0 || (p.dt_pre_ns > 0 && p.dt_pre_ns <= kMaxSegmentDtPreNs));
}

}  // namespace

bool NodeTrajectoryFollower::Init(std::shared_ptr<const pinocchio::Model> arm,
                                  pinocchio::FrameIndex frame,
                                  std::span<const int> device_of_model) {
  model_.reset();
  nv_ = 0;
  if (!arm || arm->nq != arm->nv || arm->nv < 1 || arm->nv > kMaxSegmentNv ||
      frame >= static_cast<pinocchio::FrameIndex>(arm->nframes) ||
      device_of_model.size() != static_cast<std::size_t>(arm->nv)) {
    return false;
  }
  std::array<bool, kMaxSegmentNv> seen{};
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

bool NodeTrajectoryFollower::SampleJoints(const SegmentSnapshot& plan, std::int64_t t_lead_ns,
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
  // Two spacings (MD-60): before the catch node the pre-catch columns
  // 0..n_pre at dt_pre, from it on the stop columns n_pre..N at dt. The split
  // is on integer ns; the catch node's instant belongs to the stop part (τ 0).
  const std::int64_t pre_ns = static_cast<std::int64_t>(plan.n_pre) * plan.dt_pre_ns;
  const bool in_pre = since_ns < pre_ns;
  const int first_col = in_pre ? 0 : plan.n_pre;
  const Eigen::Index rows = plan.nv;
  const Eigen::Index cols = in_pre ? plan.n_pre + 1 : plan.n_nodes - plan.n_pre + 1;
  const auto offset = static_cast<std::size_t>(first_col) * kMaxSegmentNv;
  const Eigen::OuterStride<> stride(kMaxSegmentNv);
  const NodeMap Q(plan.q.data() + offset, rows, cols, stride);
  const NodeMap Qd(plan.qd.data() + offset, rows, cols, stride);
  const NodeMap Qdd(plan.qdd.data() + offset, rows, cols, stride);
  OutMap q_out(q.data(), rows);
  OutMap qd_out(qd.data(), rows);
  OutMap qdd_out(qdd.data(), rows);
  const double dt = static_cast<double>(in_pre ? plan.dt_pre_ns : plan.dt_ns) * 1e-9;
  const double t = static_cast<double>(in_pre ? since_ns : since_ns - pre_ns) * 1e-9;
  if (!SampleJerkTrajectory(Q, Qd, Qdd, dt, t, q_out, qd_out, qdd_out)) {
    return false;
  }
  if (held != nullptr) {
    *held = since_ns >= SegmentNodeTimeNs(plan, plan.n_nodes) - plan.t0_ns;
  }
  return true;
}

bool NodeTrajectoryFollower::Sample(const SegmentSnapshot& plan, std::int64_t t_lead_ns,
                                    SegmentNodeSample& out) noexcept {
  if (model_ == nullptr || plan.nv != nv_) {
    return false;
  }
  // Evaluate into locals first: `out` must stay untouched on any failure.
  std::array<double, kMaxSegmentNv> q{};
  std::array<double, kMaxSegmentNv> qd{};
  std::array<double, kMaxSegmentNv> qdd{};
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

}  // namespace rtc::catching
