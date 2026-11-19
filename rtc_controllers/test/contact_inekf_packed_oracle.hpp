// ── Packed dynamic-size reference filter (test-only) ─────────────────────────
//
// The same filter in the packed layout of the original contact-aided InEKF
// implementation: Eigen::MatrixXd covariance over the ACTIVE contacts only
// (ordered by slot id), grown by row/column insertion through the augmentation
// Jacobians F, G (P ← F P Fᵀ + G Σ Gᵀ) and shrunk by deleting rows/columns.
// It shares no code with the fixed-slot core: the comparison is what checks
// the fixed-capacity bookkeeping (inactive rows, slot offsets, masks).
//
// Propagation takes Φ and Q_d as inputs (gathered from the core): their
// correctness has its own oracles; here only the layout is under test.
#pragma once

#include "contact_inekf_testing.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <iterator>
#include <map>
#include <utility>
#include <vector>

namespace rtc::estimation::testing {

struct PackedMeasurement {
  int slot;
  Vec3 r;      // IMU frame
  Mat3 sigma;  // body-frame covariance of r
};

class PackedFilter {
 public:
  Mat3 R;
  Vec3 v, p, bg, ba;
  std::map<int, Vec3> d;  // active slot id → anchor (ordered by id)
  Eigen::MatrixXd P;      // 15 + 3·|d|

  static PackedFilter FromCore(const ContactInekf& f) {
    PackedFilter o;
    o.R = f.rotation();
    o.v = f.velocity();
    o.p = f.position();
    o.bg = f.gyro_bias();
    o.ba = f.accel_bias();
    for (int i = 0; i < f.num_slots(); ++i) {
      if (f.contact_active(i)) {
        o.d[i] = f.contact_position(i);
      }
    }
    o.P = Gather(f.covariance(), ActiveIndex(f));
    return o;
  }

  int Dim() const { return 15 + 3 * static_cast<int>(d.size()); }

  int Pos(int slot) const {  // packed row of a slot (if inserted now)
    return 9 + 3 * static_cast<int>(std::distance(d.begin(), d.lower_bound(slot)));
  }

  void Propagate(const Eigen::MatrixXd& Phi, const Eigen::MatrixXd& Qd, const Mat3& R1,
                 const Vec3& v1, const Vec3& p1) {
    P = Phi * P * Phi.transpose() + Qd;
    R = R1;
    v = v1;
    p = p1;
  }

  void Insert(int slot, const Vec3& r, const Mat3& sigma) {
    const int D = Dim();
    const int pos = Pos(slot);
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(D + 3, D);
    F.topLeftCorner(pos, pos).setIdentity();
    F.block(pos, 6, 3, 3).setIdentity();  // new row = p's row
    F.bottomRightCorner(D - pos, D - pos).setIdentity();
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(D + 3, 3);
    G.block(pos, 0, 3, 3) = R;
    P = F * P * F.transpose() + G * sigma * G.transpose();
    d[slot] = p + R * r;
  }

  void Remove(int slot) {
    const int D = Dim();
    const int pos = Pos(slot);
    Eigen::MatrixXd Q(D - 3, D - 3);
    std::vector<int> keep;
    for (int i = 0; i < D; ++i) {
      if (i < pos || i >= pos + 3) {
        keep.push_back(i);
      }
    }
    for (std::size_t c = 0; c < keep.size(); ++c) {
      for (std::size_t r = 0; r < keep.size(); ++r) {
        Q(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = P(keep[r], keep[c]);
      }
    }
    P = Q;
    d.erase(slot);
  }

  void Update(const std::vector<PackedMeasurement>& m) {
    const int D = Dim();
    const auto rows = static_cast<Eigen::Index>(3 * m.size());
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, D);
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(rows, rows);
    Eigen::VectorXd z(rows);
    for (std::size_t q = 0; q < m.size(); ++q) {
      const auto o = static_cast<Eigen::Index>(3 * q);
      H.block(o, 6, 3, 3) = -Mat3::Identity();
      H.block(o, Pos(m[q].slot), 3, 3) = Mat3::Identity();
      N.block(o, o, 3, 3) = R * m[q].sigma * R.transpose();
      z.segment(o, 3) = R * m[q].r + p - d.at(m[q].slot);
    }
    const Eigen::MatrixXd S = H * P * H.transpose() + N;
    const Eigen::MatrixXd K =
        P * H.transpose() * S.llt().solve(Eigen::MatrixXd::Identity(rows, rows));
    const Eigen::VectorXd delta = K * z;
    const Vec3 dphi = delta.head<3>();
    const Mat3 dR = rtc::math::se3::exp3(dphi);
    const Mat3 J = rtc::math::se3::leftJacobian(dphi);
    R = dR * R;
    v = dR * v + J * delta.segment<3>(3);
    p = dR * p + J * delta.segment<3>(6);
    int j = 0;
    for (auto& [slot, x] : d) {
      x = dR * x + J * delta.segment<3>(9 + 3 * j);
      ++j;
    }
    bg += delta.segment<3>(D - 6);
    ba += delta.segment<3>(D - 3);
    const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(D, D) - K * H;
    P = IKH * P * IKH.transpose() + K * N * K.transpose();
  }

  /// Relative Frobenius distance of the state vector and of P to a core.
  std::pair<double, double> Distance(const ContactInekf& f) const {
    const PackedFilter c = FromCore(f);
    Eigen::VectorXd a(21 + 3 * static_cast<Eigen::Index>(d.size()));
    Eigen::VectorXd b(a.size());
    auto fill = [](Eigen::VectorXd& out, const PackedFilter& x) {
      out.head<9>() = Eigen::Map<const Eigen::Matrix<double, 9, 1>>(x.R.data());
      out.segment<3>(9) = x.v;
      out.segment<3>(12) = x.p;
      out.segment<3>(15) = x.bg;
      out.segment<3>(18) = x.ba;
      Eigen::Index o = 21;
      for (const auto& [slot, y] : x.d) {
        out.segment<3>(o) = y;
        o += 3;
      }
    };
    if (c.d.size() != d.size() || c.P.rows() != P.rows()) {
      return {1.0, 1.0};
    }
    for (const auto& [slot, y] : d) {
      if (!c.d.count(slot)) {
        return {1.0, 1.0};
      }
    }
    fill(a, *this);
    fill(b, c);
    return {(a - b).norm() / a.norm(), (P - c.P).norm() / P.norm()};
  }
};

}  // namespace rtc::estimation::testing
