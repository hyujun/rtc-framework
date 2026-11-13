/// @file contact_kinematics.cpp
/// @brief Frame values and analytic Jacobians through the kinodynamic ODE (see
///        the header for the quantities and the order of the calls).

#include "rtc_mpc/ocp/contact_kinematics.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/skew.hpp>
#pragma GCC diagnostic pop

#include <cmath>

namespace rtc::mpc {

ContactKinematics::ContactKinematics(const Ode& ode)
    : pin_data(ode.pin_model_),
      ode_data(ode.createData()),
      vdot(Eigen::VectorXd::Zero(ode.pin_model_.nv)),
      v_dq(Matrix6x::Zero(6, ode.pin_model_.nv)),
      v_dv(Matrix6x::Zero(6, ode.pin_model_.nv)),
      a_dq(Matrix6x::Zero(6, ode.pin_model_.nv)),
      a_dv(Matrix6x::Zero(6, ode.pin_model_.nv)),
      a_da(Matrix6x::Zero(6, ode.pin_model_.nv)),
      jw(Matrix3x::Zero(3, ode.pin_model_.nv)),
      lin(Matrix3x::Zero(3, ode.pin_model_.nv)),
      acc_jac(Matrix6x::Zero(6, ode.pin_model_.nv)) {}

void ContactKinematics::Forward(const Ode& ode, const ConstVectorRef& x, const ConstVectorRef& u) {
  const pinocchio::Model& model = ode.pin_model_;
  ode.forward(x, u, *ode_data);
  vdot = ode_data->xdot_.segment(model.nv, model.nv);
  pinocchio::forwardKinematics(model, pin_data, x.head(model.nq), x.segment(model.nq, model.nv),
                               vdot);
}

void ContactKinematics::FrameMotionOf(const pinocchio::Model& model, pinocchio::FrameIndex fid,
                                      FrameMotion& out) noexcept {
  const pinocchio::SE3& oMf = pinocchio::updateFramePlacement(model, pin_data, fid);
  const pinocchio::Motion vel =
      pinocchio::getFrameVelocity(model, pin_data, fid, pinocchio::LOCAL_WORLD_ALIGNED);
  const pinocchio::Motion acc = pinocchio::getFrameClassicalAcceleration(
      model, pin_data, fid, pinocchio::LOCAL_WORLD_ALIGNED);
  out.position = oMf.translation();
  out.rotation = oMf.rotation();
  out.linear_velocity = vel.linear();
  out.angular_velocity = vel.angular();
  out.linear_acceleration = acc.linear();
  out.angular_acceleration = acc.angular();
}

void ContactKinematics::Derivatives(const Ode& ode, const ConstVectorRef& x,
                                    const ConstVectorRef& u) {
  const pinocchio::Model& model = ode.pin_model_;
  ode.dForward(x, u, *ode_data);
  pinocchio::computeForwardKinematicsDerivatives(model, pin_data, x.head(model.nq),
                                                 x.segment(model.nq, model.nv), vdot);
}

void ContactKinematics::FrameJacobian(const pinocchio::Model& model, pinocchio::FrameIndex fid,
                                      MatrixRef jx, MatrixRef ju) noexcept {
  const int nv = model.nv;
  // ∂v̇/∂x and ∂v̇/∂u: the velocity-derivative rows of the ODE Jacobians.
  const Eigen::Ref<const Eigen::MatrixXd> dvdot_dx = ode_data->Jx_.bottomRows(nv);
  const Eigen::Ref<const Eigen::MatrixXd> dvdot_du = ode_data->Ju_.bottomRows(nv);

  // The derivative routines write only the columns of the frame's support.
  v_dq.setZero();
  v_dv.setZero();
  a_dq.setZero();
  a_dv.setZero();
  a_da.setZero();
  // In LOCAL, then rotated to world axes: x_w = R·x_L, so
  // δx_w = R·δx_L − [x_w]× δθ with δθ = J_ω δq the world rotation.
  // (pinocchio's LOCAL_WORLD_ALIGNED derivatives disagree with central
  // differences of the LOCAL_WORLD_ALIGNED quantities in the q columns —
  // the base-translation columns of the linear rows come out non-zero.)
  pinocchio::getFrameAccelerationDerivatives(model, pin_data, fid, pinocchio::LOCAL, v_dq, v_dv,
                                             a_dq, a_dv, a_da);
  const Eigen::Matrix3d& rot = pin_data.oMf[fid].rotation();
  const pinocchio::Motion vel_l =
      pinocchio::getFrameVelocity(model, pin_data, fid, pinocchio::LOCAL);
  const pinocchio::Motion acc_l =
      pinocchio::getFrameAcceleration(model, pin_data, fid, pinocchio::LOCAL);
  const Eigen::Vector3d v_l = vel_l.linear();
  const Eigen::Vector3d w_l = vel_l.angular();
  const Eigen::Vector3d acc_cl = acc_l.linear() + w_l.cross(v_l);  // classical, LOCAL
  const Eigen::Matrix3d skew_v_w = pinocchio::skew(Eigen::Vector3d(rot * v_l));
  const Eigen::Matrix3d skew_w_w = pinocchio::skew(Eigen::Vector3d(rot * w_l));
  const Eigen::Matrix3d skew_a_w = pinocchio::skew(Eigen::Vector3d(rot * acc_cl));
  const Eigen::Matrix3d skew_alpha_w = pinocchio::skew(Eigen::Vector3d(rot * acc_l.angular()));
  const Eigen::Matrix3d skew_vl = pinocchio::skew(v_l);
  const Eigen::Matrix3d skew_wl = pinocchio::skew(w_l);
  jw.noalias() = rot.lazyProduct(v_dv.bottomRows<3>());

  Eigen::Ref<Eigen::MatrixXd> jq = jx.leftCols(nv);
  Eigen::Ref<Eigen::MatrixXd> jv = jx.rightCols(nv);

  // Position: the linear part of the frame Jacobian, on world axes.
  jq.middleRows<3>(0).noalias() = rot.lazyProduct(v_dv.topRows<3>());
  jv.middleRows<3>(0).setZero();
  // Rotation increment.
  jq.middleRows<3>(3) = jw;
  jv.middleRows<3>(3).setZero();

  // Velocity.
  jq.middleRows<3>(6).noalias() = rot.lazyProduct(v_dq.topRows<3>());
  jq.middleRows<3>(6).noalias() -= skew_v_w.lazyProduct(jw);
  jv.middleRows<3>(6).noalias() = rot.lazyProduct(v_dv.topRows<3>());
  jq.middleRows<3>(9).noalias() = rot.lazyProduct(v_dq.bottomRows<3>());
  jq.middleRows<3>(9).noalias() -= skew_w_w.lazyProduct(jw);
  jv.middleRows<3>(9).noalias() = rot.lazyProduct(v_dv.bottomRows<3>());

  // Linear acceleration at fixed v̇: the LOCAL classical acceleration
  // a_L + ω_L × v_L (δ(ω × v) = [ω]× δv − [v]× δω), rotated.
  lin.noalias() = a_dq.topRows<3>();
  lin.noalias() += skew_wl.lazyProduct(v_dq.topRows<3>());
  lin.noalias() -= skew_vl.lazyProduct(v_dq.bottomRows<3>());
  jq.middleRows<3>(12).noalias() = rot.lazyProduct(lin);
  jq.middleRows<3>(12).noalias() -= skew_a_w.lazyProduct(jw);
  lin.noalias() = a_dv.topRows<3>();
  lin.noalias() += skew_wl.lazyProduct(v_dv.topRows<3>());
  lin.noalias() -= skew_vl.lazyProduct(v_dv.bottomRows<3>());
  jv.middleRows<3>(12).noalias() = rot.lazyProduct(lin);

  // Angular acceleration at fixed v̇.
  jq.middleRows<3>(15).noalias() = rot.lazyProduct(a_dq.bottomRows<3>());
  jq.middleRows<3>(15).noalias() -= skew_alpha_w.lazyProduct(jw);
  jv.middleRows<3>(15).noalias() = rot.lazyProduct(a_dv.bottomRows<3>());

  // Through v̇(x, u): ∂a/∂v̇ is the frame Jacobian on world axes (the
  // classical term has no v̇). Position, rotation and velocity do not
  // depend on u.
  acc_jac.topRows<3>().noalias() = rot.lazyProduct(a_da.topRows<3>());
  acc_jac.bottomRows<3>().noalias() = rot.lazyProduct(a_da.bottomRows<3>());
  jx.middleRows<6>(12).noalias() += acc_jac * dvdot_dx;
  ju.topRows<12>().setZero();
  ju.middleRows<6>(12).noalias() = acc_jac * dvdot_du;
}

void FrameTilt(const Eigen::Vector3d& a, const Eigen::Vector3d& n, double min_norm,
               Eigen::Vector3d& e, Eigen::Matrix3d* de_da) noexcept {
  const Eigen::Vector3d c = a.cross(n);
  const double w = 1.0 + a.dot(n);
  const double raw = std::sqrt(c.squaredNorm() + w * w);
  const bool floored = !(raw >= min_norm);
  const double norm = floored ? min_norm : raw;
  e = -c / norm;
  if (de_da == nullptr) {
    return;
  }
  // ∂c/∂a = −[n]×, ∂w/∂a = nᵀ.
  const Eigen::Matrix3d dc = -pinocchio::skew(n);
  Eigen::RowVector3d dnorm = Eigen::RowVector3d::Zero();
  if (!floored) {
    dnorm = (c.transpose() * dc + w * n.transpose()) / norm;
  }
  *de_da = -dc / norm + (c / (norm * norm)) * dnorm;
}

}  // namespace rtc::mpc
