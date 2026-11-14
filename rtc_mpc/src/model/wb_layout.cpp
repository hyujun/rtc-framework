#include "rtc_mpc/model/wb_layout.hpp"

#include "rtc_mpc/model/robot_model_handler.hpp"
#include "rtc_urdf_bridge/free_flyer_root.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <cmath>

namespace rtc::mpc {

namespace {

/// Unit-norm tolerance for a free-flyer quaternion fed to pinocchio: a
/// non-unit quaternion yields a finite but wrong tangent step, never a NaN,
/// so it has to be refused here.
constexpr double kQuatUnitTol = 1e-6;

bool FreeFlyerQuaternionIsUnit(const Eigen::Ref<const Eigen::VectorXd>& q) noexcept {
  const double n2 = q.segment(kFreeFlyerQuatOffset, kFreeFlyerQuatSize).squaredNorm();
  return std::isfinite(n2) && std::abs(n2 - 1.0) < kQuatUnitTol;
}

/// The one place the layout arithmetic lives. With a per-contact table
/// (`n_contacts > 0`) the total is its sum; without one, @p total is used as
/// is (the per-contact split is then unknown).
WbLayout Build(int nq, int nv, bool floating, const int* contact_dims, int n_contacts,
               int total) noexcept {
  WbLayout l{};
  if (nq <= 0 || nv <= 0 || n_contacts < 0 || n_contacts > kMaxContacts ||
      (n_contacts > 0 && contact_dims == nullptr)) {
    return l;
  }
  if (n_contacts > 0) {
    total = 0;
    for (int i = 0; i < n_contacts; ++i) {
      const int d = contact_dims[i];
      if (d <= 0 || total + d > kMaxContactVars) {
        return WbLayout{};
      }
      l.contact_dim[static_cast<std::size_t>(i)] = d;
      l.contact_u_offset[static_cast<std::size_t>(i)] = total;
      total += d;
    }
  }
  if (total < 0 || total > kMaxContactVars) {
    return WbLayout{};
  }
  l.nq = nq;
  l.nv = nv;
  l.ndx = 2 * nv;
  l.n_base_q = floating ? kFreeFlyerNq : 0;
  l.n_base_v = floating ? kFreeFlyerNv : 0;
  l.n_contacts = n_contacts;
  l.n_contact_vars = total;
  // Kinodynamic input on a floating base: wrenches first, then the actuated
  // accelerations. Identity torque input on a fixed base: no wrench block.
  l.n_wrench_u = floating ? total : 0;
  l.acc_u_offset = l.n_wrench_u;
  l.n_acc = nv - l.n_base_v;
  l.nu = l.n_wrench_u + l.n_acc;
  return l;
}

/// `out = [head; tail]` with exact sizes; the shape shared by q, v and u.
bool ComposeHeadTail(int n_head, int n_total, const Eigen::Ref<const Eigen::VectorXd>& head,
                     const Eigen::Ref<const Eigen::VectorXd>& tail,
                     Eigen::Ref<Eigen::VectorXd> out) noexcept {
  if (head.size() != n_head || tail.size() != n_total - n_head || out.size() != n_total) {
    return false;
  }
  out.head(n_head) = head;
  out.tail(n_total - n_head) = tail;
  return true;
}

bool SplitHeadTail(int n_head, int n_total, const Eigen::Ref<const Eigen::VectorXd>& in,
                   Eigen::Ref<Eigen::VectorXd> head, Eigen::Ref<Eigen::VectorXd> tail) noexcept {
  if (in.size() != n_total || head.size() != n_head || tail.size() != n_total - n_head) {
    return false;
  }
  head = in.head(n_head);
  tail = in.tail(n_total - n_head);
  return true;
}

}  // namespace

bool IsFloatingBaseModel(const pinocchio::Model& model) noexcept {
  return rtc_urdf_bridge::IsFreeFlyerRoot(model);
}

WbLayout WbLayout::FixedBase(int nq, int nv, int n_contact_vars) noexcept {
  return Build(nq, nv, false, nullptr, 0, n_contact_vars);
}

WbLayout WbLayout::FromModel(const pinocchio::Model& model, int n_contact_vars) noexcept {
  return Build(static_cast<int>(model.nq), static_cast<int>(model.nv), IsFloatingBaseModel(model),
               nullptr, 0, n_contact_vars);
}

WbLayout WbLayout::FromHandler(const RobotModelHandler& handler) noexcept {
  if (!handler.Initialised()) {
    return WbLayout{};
  }
  const int n = handler.n_contacts();
  if (n > kMaxContacts) {
    return WbLayout{};
  }
  std::array<int, kMaxContacts> dims{};
  for (int i = 0; i < n; ++i) {
    dims[static_cast<std::size_t>(i)] = handler.contact_frames()[static_cast<std::size_t>(i)].dim;
  }
  return Build(handler.nq(), handler.nv(), IsFloatingBaseModel(handler.model()), dims.data(), n, 0);
}

bool WbLayout::ComposeQ(const Eigen::Ref<const Eigen::VectorXd>& base_pose,
                        const Eigen::Ref<const Eigen::VectorXd>& q_j,
                        Eigen::Ref<Eigen::VectorXd> q) const noexcept {
  return valid() && ComposeHeadTail(n_base_q, nq, base_pose, q_j, q);
}

bool WbLayout::SplitQ(const Eigen::Ref<const Eigen::VectorXd>& q,
                      Eigen::Ref<Eigen::VectorXd> base_pose,
                      Eigen::Ref<Eigen::VectorXd> q_j) const noexcept {
  return valid() && SplitHeadTail(n_base_q, nq, q, base_pose, q_j);
}

bool WbLayout::ComposeV(const Eigen::Ref<const Eigen::VectorXd>& base_twist,
                        const Eigen::Ref<const Eigen::VectorXd>& v_j,
                        Eigen::Ref<Eigen::VectorXd> v) const noexcept {
  return valid() && ComposeHeadTail(n_base_v, nv, base_twist, v_j, v);
}

bool WbLayout::SplitV(const Eigen::Ref<const Eigen::VectorXd>& v,
                      Eigen::Ref<Eigen::VectorXd> base_twist,
                      Eigen::Ref<Eigen::VectorXd> v_j) const noexcept {
  return valid() && SplitHeadTail(n_base_v, nv, v, base_twist, v_j);
}

bool WbLayout::ComposeU(const Eigen::Ref<const Eigen::VectorXd>& wrench,
                        const Eigen::Ref<const Eigen::VectorXd>& qdd_j,
                        Eigen::Ref<Eigen::VectorXd> u) const noexcept {
  return valid() && ComposeHeadTail(n_wrench_u, nu, wrench, qdd_j, u);
}

bool WbLayout::SplitU(const Eigen::Ref<const Eigen::VectorXd>& u,
                      Eigen::Ref<Eigen::VectorXd> wrench,
                      Eigen::Ref<Eigen::VectorXd> qdd_j) const noexcept {
  return valid() && SplitHeadTail(n_wrench_u, nu, u, wrench, qdd_j);
}

bool WbLayout::BaseQuaternionIsUnit(const Eigen::Ref<const Eigen::VectorXd>& q) const noexcept {
  if (!floating_base()) {
    return true;
  }
  return q.size() == nq && FreeFlyerQuaternionIsUnit(q);
}

bool WbLayout::StateDifference(const pinocchio::Model& model,
                               const Eigen::Ref<const Eigen::VectorXd>& q_ref,
                               const Eigen::Ref<const Eigen::VectorXd>& v_ref,
                               const Eigen::Ref<const Eigen::VectorXd>& q_cur,
                               const Eigen::Ref<const Eigen::VectorXd>& v_cur,
                               Eigen::Ref<Eigen::VectorXd> dx) const noexcept {
  // pinocchio checks argument sizes by throwing; every size is checked here
  // first so the call below cannot throw through this noexcept boundary.
  if (!valid() || model.nq != nq || model.nv != nv || q_ref.size() != nq || q_cur.size() != nq ||
      v_ref.size() != nv || v_cur.size() != nv || dx.size() != ndx) {
    return false;
  }
  if (!BaseQuaternionIsUnit(q_ref) || !BaseQuaternionIsUnit(q_cur)) {
    return false;
  }
  pinocchio::difference(model, q_ref, q_cur, dx.head(nv));
  dx.tail(nv) = v_cur - v_ref;
  if (!dx.allFinite()) {
    dx.setZero();
    return false;
  }
  return true;
}

}  // namespace rtc::mpc
