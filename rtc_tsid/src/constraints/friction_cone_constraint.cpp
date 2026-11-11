#include "rtc_tsid/constraints/friction_cone_constraint.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace rtc::tsid {

namespace {

// Stage A-4: contact-local → world rotation. Returns true if normal is
// effectively +Z (within numerical tolerance) so the caller can skip the
// 3×3 matrix multiply and use the byte-identical pre-A-4 cone layout.
//
// R_c = [t1 | t2 | n] with t1, t2 forming an orthonormal tangent basis.
// Gram-Schmidt against a fixed seed axis (chosen to avoid degeneracy
// when n is close to the seed). RT-safe: stack-allocated, no heap, no
// branching beyond the seed pick.
inline bool BuildContactBasis(const Eigen::Vector3d& n_in, Eigen::Matrix3d& R_c) noexcept {
  constexpr double kPlusZTol = 1.0 - 1e-12;
  const double nn = n_in.norm();
  if (!(nn > 1e-9)) {
    // Degenerate normal → fall back to world +Z.
    R_c.setIdentity();
    return true;
  }
  const Eigen::Vector3d n = n_in / nn;
  if (n.z() > kPlusZTol) {
    // +Z short-circuit: R_c = I, regression-safe path. Cone columns end
    // up byte-identical to the pre-A-4 implementation.
    R_c.setIdentity();
    return true;
  }
  // Seed axis pick: world X unless n is too close to X (in which case
  // world Y), so Gram-Schmidt stays well-conditioned.
  const Eigen::Vector3d seed =
      (std::abs(n.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
  Eigen::Vector3d t1 = seed - n * (seed.dot(n));
  const double t1n = t1.norm();
  if (t1n < 1e-9) {
    // Should not happen given the seed switch, but guard anyway.
    R_c.setIdentity();
    return true;
  }
  t1 /= t1n;
  const Eigen::Vector3d t2 = n.cross(t1);
  R_c.col(0) = t1;
  R_c.col(1) = t2;
  R_c.col(2) = n;
  return false;
}

constexpr std::int8_t kBranchNone = 0;
constexpr std::int8_t kBranchX = 1;
constexpr std::int8_t kBranchY = 2;

// Surface contacts: the tangent axes follow the contact FRAME, so the CoP
// rectangle (l_x along the frame's x, l_y along its y) turns with the foot.
// t1 = frame x projected on the plane ⊥ n; when the frame's x is too close to
// n, its y is projected instead and t1 = t2 × n. R_c = [t1 | t2 | n] is
// right-handed (t2 = n × t1). Never the +Z short-circuit: on flat ground that
// would put the rectangle on the world axes whatever the foot's yaw.
// The two rules' t1 can differ by any angle up to 180° about n (it depends
// on which way the frame's x leans off n), so which one is used has hysteresis in `branch` (the
// contact's last rule; see the class comment): the x rule holds down to a
// projection of kSurfaceBasisXToY, the y rule up to kSurfaceBasisYToX, and a
// contact without a branch splits at kSurfaceBasisFresh. On the y branch the
// x projection is ≤ 0.12, so y's projection is ≥ √(1 − 0.12²) ≈ 0.993.
inline void BuildSurfaceBasis(const Eigen::Vector3d& n_in, const Eigen::Matrix3d& oRf,
                              std::int8_t& branch, Eigen::Matrix3d& R_c) noexcept {
  const double nn = n_in.norm();
  const Eigen::Vector3d n = (nn > 1e-9) ? Eigen::Vector3d(n_in / nn) : Eigen::Vector3d::UnitZ();
  Eigen::Vector3d t1 = oRf.col(0) - n * oRf.col(0).dot(n);
  const double len = t1.norm();
  if (!std::isfinite(len)) {
    // A non-finite frame or normal: the rows go non-finite so the solve fails
    // (never laundered), and the contact's branch is kept as it was.
    R_c.setConstant(std::numeric_limits<double>::quiet_NaN());
    return;
  }
  bool use_x = false;
  if (branch == kBranchX) {
    use_x = len >= FrictionConeConstraint::kSurfaceBasisXToY;
  } else if (branch == kBranchY) {
    use_x = len > FrictionConeConstraint::kSurfaceBasisYToX;
  } else {
    use_x = len >= FrictionConeConstraint::kSurfaceBasisFresh;
  }
  branch = use_x ? kBranchX : kBranchY;
  if (use_x) {
    t1 /= len;
  } else {
    Eigen::Vector3d t2 = oRf.col(1) - n * oRf.col(1).dot(n);
    t2.normalize();
    t1 = t2.cross(n);
  }
  R_c.col(0) = t1;
  R_c.col(1) = n.cross(t1);
  R_c.col(2) = n;
}

// Point contact (cdim=3): n_faces + 1 (cone faces + unilateral -fz ≤ 0)
// Surface contact (cdim=6): + 4 (CoP rectangle) + 2 (yaw moment) = n_faces + 7.
// patch=0 / μ_τ=0 인 surface 라도 trivial 행 (0 ≤ 0) 으로 유지 — ineq_dim 일관.
int RowsPerContact(const ContactConfig& cfg) noexcept {
  int per_contact = cfg.friction_faces + 1;
  if (cfg.contact_dim == 6) {
    per_contact += 6;
  }
  return per_contact;
}

}  // namespace

void FrictionConeConstraint::SetContactManager(const ContactManagerConfig* manager) {
  manager_ = manager;
  surface_branch_.assign(manager != nullptr ? manager->contacts.size() : 0, kBranchNone);
}

void FrictionConeConstraint::Init(const pinocchio::Model& /*model*/,
                                  const RobotModelInfo& robot_info, PinocchioCache& /*cache*/,
                                  const YAML::Node& /*constraint_config*/) {
  nv_ = robot_info.nv;
}

int FrictionConeConstraint::EqDim(const ContactState& /*contacts*/) const noexcept {
  return 0;
}

int FrictionConeConstraint::IneqDim(const ContactState& contacts) const noexcept {
  if (!manager_)
    return 0;

  int total = 0;
  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    if (!contacts.contacts[i].active)
      continue;
    total += RowsPerContact(manager_->contacts[i]);
  }
  return total;
}

int FrictionConeConstraint::MaxEqDim(const ContactManagerConfig& /*contacts*/) const noexcept {
  return 0;
}

int FrictionConeConstraint::MaxIneqDim(const ContactManagerConfig& contacts) const noexcept {
  int total = 0;
  for (const auto& cfg : contacts.contacts) {
    total += RowsPerContact(cfg);
  }
  return total;
}

void FrictionConeConstraint::ComputeEquality(const PinocchioCache& /*cache*/,
                                             const ContactState& /*contacts*/,
                                             const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                             Eigen::Ref<Eigen::MatrixXd> /*A_block*/,
                                             Eigen::Ref<Eigen::VectorXd> /*b_block*/) noexcept {}

void FrictionConeConstraint::ComputeInequality(const PinocchioCache& cache,
                                               const ContactState& contacts,
                                               const RobotModelInfo& /*robot_info*/, int /*n_vars*/,
                                               Eigen::Ref<Eigen::MatrixXd> C_block,
                                               Eigen::Ref<Eigen::VectorXd> l_block,
                                               Eigen::Ref<Eigen::VectorXd> u_block) noexcept {
  if (!manager_)
    return;

  int row = 0;
  int lambda_offset = 0;

  // Stage A-4: per-contact local→world rotation. Stack-allocated 3×3,
  // zero heap. R_c = [t1 | t2 | n] s.t. contact-local +Z aligns with the
  // ContactState::Entry::normal in world frame. is_plus_z=true means
  // R_c is the identity and the cone matrix is built byte-identically
  // to the pre-A-4 implementation (no matmul, regression-safe).
  Eigen::Matrix3d R_c;

  for (size_t i = 0; i < contacts.contacts.size(); ++i) {
    const auto& cfg = manager_->contacts[i];
    const int cdim = cfg.contact_dim;
    const int n_faces = cfg.friction_faces;
    const double mu = cfg.friction_coeff;

    // Stage A-5a: fixed-dim QP — advance the contact's λ column slot even
    // when inactive, so active contacts land on stable QP indices. Inactive
    // contacts contribute zero rows (skipped below).
    if (!contacts.contacts[i].active) {
      // A contact that comes back picks its surface basis afresh.
      if (i < surface_branch_.size()) {
        surface_branch_[i] = kBranchNone;
      }
      lambda_offset += cdim;
      continue;
    }

    const int lambda_col = nv_ + lambda_offset;

    // Build contact basis from the runtime-filtered normal (surface: and the
    // contact frame's tangent axes).
    bool is_plus_z = false;
    if (cdim == 6 && i < cache.contact_frames.size()) {
      // No stored branch (not reachable through SetContactManager): fresh rule.
      std::int8_t no_branch = kBranchNone;
      std::int8_t& branch = i < surface_branch_.size() ? surface_branch_[i] : no_branch;
      BuildSurfaceBasis(contacts.contacts[i].normal, cache.contact_frames[i].oMf.rotation(), branch,
                        R_c);
    } else {
      is_plus_z = BuildContactBasis(contacts.contacts[i].normal, R_c);
    }

    // Normal-force bounds on the unilateral row −f_n ∈ [−f_max, −a·f_min]
    // (a = activation, so the floor ramps with the contact). Defaults 0 / +∞
    // keep the pre-existing row: l = −1e10, u = 0.
    const double f_min = cfg.normal_force_min * contacts.contacts[i].activation;
    const double uni_l = std::isinf(cfg.normal_force_max) ? -1e10 : -cfg.normal_force_max;
    const double uni_u = (f_min > 0.0) ? -f_min : 0.0;

    // Linearized friction cone (force components, common to point & surface):
    //   Contact-local:  [cos θk, sin θk, -μ] · [f_t1, f_t2, f_n]ᵀ ≤ 0
    //   World (λ_w):    ([cos θk, sin θk, -μ] · R_cᵀ) · λ_w ≤ 0
    // +Z short-circuit: R_c = I → identical to pre-A-4 (cos·fx + sin·fy - μ·fz).
    if (is_plus_z) {
      for (int k = 0; k < n_faces; ++k) {
        const double theta = 2.0 * M_PI * static_cast<double>(k) / static_cast<double>(n_faces);
        C_block(row, lambda_col + 0) = std::cos(theta);
        C_block(row, lambda_col + 1) = std::sin(theta);
        C_block(row, lambda_col + 2) = -mu;
        l_block(row) = -1e10;
        u_block(row) = 0.0;
        ++row;
      }
      // Unilateral: -f_n ≤ 0 (local) → -fz ≤ 0 (world, since n = +Z).
      C_block(row, lambda_col + 2) = -1.0;
      l_block(row) = uni_l;
      u_block(row) = uni_u;
      ++row;
    } else {
      // Rotated cone rows: world coeffs = [cos θ, sin θ, -μ] · R_cᵀ.
      for (int k = 0; k < n_faces; ++k) {
        const double theta = 2.0 * M_PI * static_cast<double>(k) / static_cast<double>(n_faces);
        const Eigen::Vector3d row_local(std::cos(theta), std::sin(theta), -mu);
        const Eigen::Vector3d row_world = R_c * row_local;  // (R_cᵀ)ᵀ · row_local
        C_block(row, lambda_col + 0) = row_world.x();
        C_block(row, lambda_col + 1) = row_world.y();
        C_block(row, lambda_col + 2) = row_world.z();
        l_block(row) = -1e10;
        u_block(row) = 0.0;
        ++row;
      }
      // Unilateral: -f_n ≤ 0 (local) → -nᵀ · λ_w ≤ 0 (world).
      C_block(row, lambda_col + 0) = -R_c(0, 2);
      C_block(row, lambda_col + 1) = -R_c(1, 2);
      C_block(row, lambda_col + 2) = -R_c(2, 2);
      l_block(row) = uni_l;
      u_block(row) = uni_u;
      ++row;
    }

    // Surface contact (cdim=6): wrench cone — CoP rectangle + yaw moment.
    // λ_local = [fx, fy, fz, mx, my, mz] (contact frame, +z normal).
    // In world frame, λ_w_force = R_c · λ_local_force, same for moments,
    // so each row's local coefficients must be transformed by R_c on both
    // force and moment blocks: row_world_block = row_local_block · R_cᵀ.
    if (cdim == 6) {
      const double lx = cfg.patch_half_length_x;
      const double ly = cfg.patch_half_length_y;
      const double mu_tau = cfg.torsional_friction_coeff;

      // Helper lambda: write one row of [a_force | a_moment] · R_cᵀ into
      // C_block[row, lambda_col..lambda_col+5]. is_plus_z short-circuits.
      auto write_surface_row = [&](const Eigen::Vector3d& a_force,
                                   const Eigen::Vector3d& a_moment) {
        if (is_plus_z) {
          C_block(row, lambda_col + 0) = a_force.x();
          C_block(row, lambda_col + 1) = a_force.y();
          C_block(row, lambda_col + 2) = a_force.z();
          C_block(row, lambda_col + 3) = a_moment.x();
          C_block(row, lambda_col + 4) = a_moment.y();
          C_block(row, lambda_col + 5) = a_moment.z();
        } else {
          const Eigen::Vector3d f_w = R_c * a_force;
          const Eigen::Vector3d m_w = R_c * a_moment;
          C_block(row, lambda_col + 0) = f_w.x();
          C_block(row, lambda_col + 1) = f_w.y();
          C_block(row, lambda_col + 2) = f_w.z();
          C_block(row, lambda_col + 3) = m_w.x();
          C_block(row, lambda_col + 4) = m_w.y();
          C_block(row, lambda_col + 5) = m_w.z();
        }
        l_block(row) = -1e10;
        u_block(row) = 0.0;
        ++row;
      };

      // Local-frame coefficients identical to pre-A-4 rows:
      //   m_y - l_x·f_z ≤ 0          → a_force=(0,0,-lx),    a_moment=(0,+1,0)
      //   -m_y - l_x·f_z ≤ 0         → a_force=(0,0,-lx),    a_moment=(0,-1,0)
      //   m_x - l_y·f_z ≤ 0          → a_force=(0,0,-ly),    a_moment=(+1,0,0)
      //   -m_x - l_y·f_z ≤ 0         → a_force=(0,0,-ly),    a_moment=(-1,0,0)
      //   m_z - μ_τ·f_z ≤ 0          → a_force=(0,0,-mu_tau),a_moment=(0,0,+1)
      //   -m_z - μ_τ·f_z ≤ 0         → a_force=(0,0,-mu_tau),a_moment=(0,0,-1)
      write_surface_row({0.0, 0.0, -lx}, {0.0, 1.0, 0.0});
      write_surface_row({0.0, 0.0, -lx}, {0.0, -1.0, 0.0});
      write_surface_row({0.0, 0.0, -ly}, {1.0, 0.0, 0.0});
      write_surface_row({0.0, 0.0, -ly}, {-1.0, 0.0, 0.0});
      write_surface_row({0.0, 0.0, -mu_tau}, {0.0, 0.0, 1.0});
      write_surface_row({0.0, 0.0, -mu_tau}, {0.0, 0.0, -1.0});
    }

    lambda_offset += cdim;
  }
}

}  // namespace rtc::tsid
