#pragma once

/// @file kinodynamic_fixture.hpp
/// @brief Samples and rigid-body oracles for the kinodynamic dynamics suites:
///        a seeded draw of (x, u, mode) on a two-contact floating-base model,
///        a fixed edge set, and the oracle side of the base-acceleration law
///        (Σ JᵀW, RNEA, M_bb, ABA with SE3-transported external forces). Not
///        installed — test-local by design.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <Eigen/Dense>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace rtc::mpc::test_utils {

/// One draw: state, input and contact mode (bit k = contact k).
struct KinoSample {
  Eigen::VectorXd x;
  Eigen::VectorXd u;
  std::uint32_t mode{0};
};

/// The draw's ranges.
struct KinoRanges {
  double v_lin{0.5};    ///< |base linear velocity| per entry (m/s)
  double v_ang{0.5};    ///< |base angular velocity| per entry (rad/s)
  double v_joint{0.5};  ///< |joint rate| per entry (rad/s)
  double f_xy{0.3};     ///< tangential force bound, × m·g
  double f_z_max{1.0};  ///< normal force U[0, f_z_max·m·g]
  double moment{20.0};  ///< |contact moment| per entry (N·m)
  double qdd{10.0};     ///< |joint acceleration| per entry (rad/s²)
};

/// Rigid-body side of the dynamics for one floating-base model with `nc`
/// contacts of `force_size` entries each (`u = [W_0 … W_{nc−1}; q̈_j]`).
class KinoOracle {
 public:
  KinoOracle(const pinocchio::Model& model, std::vector<pinocchio::FrameIndex> contacts,
             int force_size)
      : model_(model),
        data_(model),
        contacts_(std::move(contacts)),
        force_size_(force_size),
        mass_(pinocchio::computeTotalMass(model)) {}

  [[nodiscard]] int nq() const { return model_.nq; }

  [[nodiscard]] int nv() const { return model_.nv; }

  [[nodiscard]] int na() const { return model_.nv - 6; }

  [[nodiscard]] int nc() const { return static_cast<int>(contacts_.size()); }

  [[nodiscard]] int nu() const { return nc() * force_size_ + na(); }

  [[nodiscard]] double mass() const { return mass_; }

  [[nodiscard]] const std::vector<pinocchio::FrameIndex>& contacts() const { return contacts_; }

  /// Configuration: base position U[−1, 1]³ m, Shoemake-uniform rotation,
  /// joints uniform inside their limits less @p margin.
  Eigen::VectorXd DrawQ(std::mt19937& rng, double margin = 0.05) const {
    std::uniform_real_distribution<double> sym(-1.0, 1.0);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    Eigen::VectorXd q = pinocchio::neutral(model_);
    for (int i = 0; i < 3; ++i) {
      q[i] = sym(rng);
    }
    const double u1 = unit(rng);
    const double u2 = 2.0 * M_PI * unit(rng);
    const double u3 = 2.0 * M_PI * unit(rng);
    q[3] = std::sqrt(1.0 - u1) * std::sin(u2);
    q[4] = std::sqrt(1.0 - u1) * std::cos(u2);
    q[5] = std::sqrt(u1) * std::sin(u3);
    q[6] = std::sqrt(u1) * std::cos(u3);
    for (int i = 7; i < model_.nq; ++i) {
      const double lo = model_.lowerPositionLimit[i] + margin;
      const double hi = model_.upperPositionLimit[i] - margin;
      q[i] = lo + (hi - lo) * unit(rng);
    }
    return q;
  }

  Eigen::VectorXd DrawV(std::mt19937& rng, const KinoRanges& r) const {
    std::uniform_real_distribution<double> sym(-1.0, 1.0);
    Eigen::VectorXd v(model_.nv);
    for (int i = 0; i < model_.nv; ++i) {
      const double bound = i < 3 ? r.v_lin : (i < 6 ? r.v_ang : r.v_joint);
      v[i] = bound * sym(rng);
    }
    return v;
  }

  Eigen::VectorXd DrawU(std::mt19937& rng, const KinoRanges& r) const {
    std::uniform_real_distribution<double> sym(-1.0, 1.0);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double mg = mass_ * std::abs(model_.gravity.linear().z());
    Eigen::VectorXd u(nu());
    for (int c = 0; c < nc(); ++c) {
      const int o = c * force_size_;
      u[o + 0] = r.f_xy * mg * sym(rng);
      u[o + 1] = r.f_xy * mg * sym(rng);
      u[o + 2] = r.f_z_max * mg * unit(rng);
      for (int i = 3; i < force_size_; ++i) {
        u[o + i] = r.moment * sym(rng);
      }
    }
    for (int i = 0; i < na(); ++i) {
      u[nc() * force_size_ + i] = r.qdd * sym(rng);
    }
    return u;
  }

  KinoSample Draw(std::mt19937& rng, std::uint32_t mode, const KinoRanges& r = {}) const {
    KinoSample s;
    const Eigen::VectorXd q = DrawQ(rng);
    const Eigen::VectorXd v = DrawV(rng, r);
    s.x.resize(model_.nq + model_.nv);
    s.x << q, v;
    s.u = DrawU(rng, r);
    s.mode = mode;
    return s;
  }

  [[nodiscard]] bool Active(std::uint32_t mode, int c) const { return ((mode >> c) & 1U) != 0U; }

  /// Contact @p c's 6×nv frame Jacobian in @p rf at @p q.
  Eigen::MatrixXd Jacobian(const Eigen::VectorXd& q, int c,
                           pinocchio::ReferenceFrame rf = pinocchio::LOCAL_WORLD_ALIGNED) {
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::getFrameJacobian(model_, data_, contacts_[static_cast<std::size_t>(c)], rf, J);
    return J;
  }

  /// The wrench block of contact @p c as a 6-vector (zero moment for 3-D).
  [[nodiscard]] Eigen::Matrix<double, 6, 1> Wrench(const Eigen::VectorXd& u, int c) const {
    Eigen::Matrix<double, 6, 1> w = Eigen::Matrix<double, 6, 1>::Zero();
    w.head(force_size_) = u.segment(c * force_size_, force_size_);
    return w;
  }

  /// Σ_{active} J_kᵀ W_k (generalized force), Jacobians in @p rf. With
  /// @p swap, contact k's wrench is applied at contact nc − 1 − k.
  Eigen::VectorXd JtW(const Eigen::VectorXd& q, const Eigen::VectorXd& u, std::uint32_t mode,
                      pinocchio::ReferenceFrame rf = pinocchio::LOCAL_WORLD_ALIGNED,
                      bool swap = false) {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(model_.nv);
    for (int c = 0; c < nc(); ++c) {
      if (!Active(mode, c)) {
        continue;
      }
      const int at = swap ? nc() - 1 - c : c;
      out += Jacobian(q, at, rf).transpose() * Wrench(u, c);
    }
    return out;
  }

  /// The joint-acceleration block of @p u.
  [[nodiscard]] Eigen::VectorXd Qdd(const Eigen::VectorXd& u) const { return u.tail(na()); }

  /// Full acceleration `[v̇_b; q̈_j]`.
  [[nodiscard]] Eigen::VectorXd FullA(const Eigen::Matrix<double, 6, 1>& vdot_b,
                                      const Eigen::VectorXd& u) const {
    Eigen::VectorXd a(model_.nv);
    a << vdot_b, Qdd(u);
    return a;
  }

  /// Base rows of rnea(q, v, a) — the generalized force the motion needs.
  Eigen::Matrix<double, 6, 1> RneaBase(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                       const Eigen::VectorXd& a) {
    return pinocchio::rnea(model_, data_, q, v, a).head<6>();
  }

  /// Base rows of the nonlinear effects h(q, v) (with gravity).
  Eigen::Matrix<double, 6, 1> HBase(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    return pinocchio::nonLinearEffects(model_, data_, q, v).head<6>();
  }

  /// The symmetric mass matrix.
  Eigen::MatrixXd M(const Eigen::VectorXd& q) {
    pinocchio::crba(model_, data_, q);
    Eigen::MatrixXd m = data_.M;
    m.triangularView<Eigen::StrictlyLower>() = m.transpose().triangularView<Eigen::StrictlyLower>();
    return m;
  }

  /// v̇_b from `M_bb v̇_b = −h_b − M_bj q̈_j + (Σ JᵀW)_b` (6×6 LDLT), given
  /// the generalized contact force @p jtw.
  Eigen::Matrix<double, 6, 1> MbbSolve(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                       const Eigen::VectorXd& u, const Eigen::VectorXd& jtw) {
    const Eigen::MatrixXd m = M(q);
    const Eigen::Matrix<double, 6, 1> rhs =
        -HBase(q, v) - m.topRightCorner(6, na()) * Qdd(u) + jtw.head<6>();
    return m.topLeftCorner<6, 6>().ldlt().solve(rhs);
  }

  /// The same right-hand side solved as two decoupled 3×3 blocks — a wrong
  /// solve, for the mutant check.
  Eigen::Matrix<double, 6, 1> MbbSolveSplit(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                            const Eigen::VectorXd& u, const Eigen::VectorXd& jtw) {
    const Eigen::MatrixXd m = M(q);
    const Eigen::Matrix<double, 6, 1> rhs =
        -HBase(q, v) - m.topRightCorner(6, na()) * Qdd(u) + jtw.head<6>();
    Eigen::Matrix<double, 6, 1> out;
    out.head<3>() = m.topLeftCorner<3, 3>().ldlt().solve(rhs.head<3>());
    out.tail<3>() = m.block<3, 3>(3, 3).ldlt().solve(rhs.tail<3>());
    return out;
  }

  /// Joint-local external forces for ABA / RNEA: each active wrench, taken
  /// from world axes about its contact-frame origin to the world origin, then
  /// into its parent joint's frame with SE3 actions (no Jacobian). With
  /// @p at_com the moment is taken about the CoM instead (a wrong point, for
  /// the mutant check).
  pinocchio::container::aligned_vector<pinocchio::Force> Fext(const Eigen::VectorXd& q,
                                                              const Eigen::VectorXd& u,
                                                              std::uint32_t mode,
                                                              bool at_com = false) {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    const Eigen::Vector3d com = pinocchio::centerOfMass(model_, data_, q);
    pinocchio::container::aligned_vector<pinocchio::Force> fext(model_.njoints,
                                                                pinocchio::Force::Zero());
    for (int c = 0; c < nc(); ++c) {
      if (!Active(mode, c)) {
        continue;
      }
      const pinocchio::FrameIndex fid = contacts_[static_cast<std::size_t>(c)];
      const Eigen::Vector3d p = at_com ? com : data_.oMf[fid].translation();
      const Eigen::Matrix<double, 6, 1> w = Wrench(u, c);
      // About the world origin: n_0 = τ + p × f.
      const pinocchio::Force f_world(w.head<3>(), w.tail<3>() + p.cross(w.head<3>()));
      const pinocchio::JointIndex j = model_.frames[fid].parentJoint;
      fext[j] += data_.oMi[j].actInv(f_world);
    }
    return fext;
  }

  /// The generalized force of `Fext(q, u, mode, at_com)`:
  /// `rnea(q, 0, 0) − rnea(q, 0, 0, fext)` (gravity cancels).
  Eigen::VectorXd FextGeneralized(const Eigen::VectorXd& q, const Eigen::VectorXd& u,
                                  std::uint32_t mode, bool at_com) {
    const auto fext = Fext(q, u, mode, at_com);
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(model_.nv);
    const Eigen::VectorXd free = pinocchio::rnea(model_, data_, q, zero, zero);
    return free - pinocchio::rnea(model_, data_, q, zero, zero, fext);
  }

  /// ABA oracle: the joint torques the motion `[v̇_b; q̈_j]` needs under the
  /// contact forces, then ABA with the base torque set to zero. Returns the
  /// full acceleration ABA finds.
  Eigen::VectorXd AbaAcceleration(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                  const Eigen::VectorXd& a, const Eigen::VectorXd& u,
                                  std::uint32_t mode) {
    const auto fext = Fext(q, u, mode);
    Eigen::VectorXd tau = pinocchio::rnea(model_, data_, q, v, a, fext);
    tau.head<6>().setZero();
    return pinocchio::aba(model_, data_, q, v, tau, fext);
  }

  /// Condition number (2-norm) of the centroidal base block A_b.
  double CondAb(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
    pinocchio::ccrba(model_, data_, q, v);
    const Eigen::Matrix<double, 6, 6> ab = data_.Ag.leftCols<6>();
    const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(ab);
    return svd.singularValues()[0] / svd.singularValues()[5];
  }

  /// Contact wrenches that hold the robot in static double support: every
  /// active contact pushes `−m·g/n_active`, with the moment that cancels its
  /// force's moment about the CoM. Joint accelerations zero.
  Eigen::VectorXd WeightCompensation(const Eigen::VectorXd& q, std::uint32_t mode) {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    const Eigen::Vector3d com = pinocchio::centerOfMass(model_, data_, q);
    int n_active = 0;
    for (int c = 0; c < nc(); ++c) {
      n_active += Active(mode, c) ? 1 : 0;
    }
    Eigen::VectorXd u = Eigen::VectorXd::Zero(nu());
    const Eigen::Vector3d f = -mass_ * model_.gravity.linear() / n_active;
    for (int c = 0; c < nc(); ++c) {
      if (!Active(mode, c)) {
        continue;
      }
      const Eigen::Vector3d p = data_.oMf[contacts_[static_cast<std::size_t>(c)]].translation();
      u.segment<3>(c * force_size_) = f;
      u.segment<3>(c * force_size_ + 3) = -(p - com).cross(f);
    }
    return u;
  }

  [[nodiscard]] const pinocchio::Model& model() const { return model_; }

 private:
  const pinocchio::Model& model_;
  pinocchio::Data data_;
  std::vector<pinocchio::FrameIndex> contacts_;
  int force_size_;
  double mass_;
};

/// Index of the joint named @p name in q (its first configuration entry).
inline int JointQIndex(const pinocchio::Model& model, const std::string& name) {
  return model.idx_qs[model.getJointId(name)];
}

}  // namespace rtc::mpc::test_utils
