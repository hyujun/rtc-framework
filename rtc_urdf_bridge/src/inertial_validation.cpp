// ── 관성 파라미터 물리 실현가능성 검증 구현 ──────────────────────────────────
#include "rtc_urdf_bridge/inertial_validation.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace rtc_urdf_bridge {

namespace {

/// 관절 `jid` 에 직결된 link 이름 (BODY frame). Pinocchio 는 fixed joint 를
/// 흡수하므로 한 관절에 여러 BODY frame 이 달릴 수 있다 — 그중 **처음**,
/// 즉 관절이 직접 다는 link 를 귀속 대상으로 삼는다.
std::string PrimaryBodyName(const pinocchio::Model& model, pinocchio::JointIndex jid) {
  for (const auto& frame : model.frames) {
    if (frame.type == pinocchio::BODY && frame.parentJoint == jid) {
      return frame.name;
    }
  }
  return {};
}

}  // namespace

const char* ToString(InertialDefect defect) noexcept {
  switch (defect) {
    case InertialDefect::kNonFinite:
      return "V6:non-finite";
    case InertialDefect::kNegativeMass:
      return "V6:negative-mass";
    case InertialDefect::kNotPositiveSemidefinite:
      return "V6:not-positive-semidefinite";
    case InertialDefect::kTriangleInequality:
      return "V6:triangle-inequality";
    case InertialDefect::kMasslessMovableBody:
      return "V5:massless-movable-body";
  }
  return "V?:unknown";
}

InertialValidationReport ValidateInertias(const pinocchio::Model& model, double rel_tol) {
  InertialValidationReport report;

  // index 0 은 universe — movable body 가 아니므로 건너뛴다.
  for (pinocchio::JointIndex jid = 1; jid < static_cast<pinocchio::JointIndex>(model.njoints);
       ++jid) {
    const pinocchio::Inertia& body = model.inertias[jid];
    const double mass = body.mass();
    // Inertia::inertia() 는 **CoM 기준** 회전관성이다 (Model::inertias 의 lever 로
    // 평행축 이동이 분리돼 있다). 삼각부등식은 CoM 기준 주모멘트의 성질이므로
    // 여기서 바로 판정할 수 있다.
    const Eigen::Matrix3d inertia = body.inertia().matrix();

    InertialViolation v;
    v.joint_name = model.names[jid];
    v.body_name = PrimaryBodyName(model, jid);
    v.mass = mass;

    if (!std::isfinite(mass) || !inertia.allFinite()) {
      v.defect = InertialDefect::kNonFinite;
      report.fatal.push_back(std::move(v));
      continue;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    solver.compute(inertia, Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success) {
      // 유한한 입력에서도 성분이 배정도 한계 근처면 분해가 실패할 수 있다. 이때
      // eigenvalues() 는 쓰레기이므로 "통과" 로 흘리면 안 된다 — 판정 불가는
      // 통과가 아니라 거부다.
      v.defect = InertialDefect::kNonFinite;
      report.fatal.push_back(std::move(v));
      continue;
    }
    const Eigen::Vector3d eigs = solver.eigenvalues();  // 오름차순
    const double i1 = eigs(0);
    const double i2 = eigs(1);
    const double i3 = eigs(2);
    v.principal_moments = {i1, i2, i3};

    // 크기 정규화 기준. 고유값이 음수일 수 있으므로 절댓값 최대를 쓴다
    // (최소·최대 중 하나가 반드시 절댓값 최대다). 영텐서면 0 이 되어
    // 아래 판정이 그대로 엄격 부등호로 축퇴한다 — 의도된 동작이다.
    const double scale = std::max(std::abs(i1), std::abs(i3));
    const double tol = rel_tol * scale;

    if (mass < 0.0) {
      v.defect = InertialDefect::kNegativeMass;
      report.fatal.push_back(std::move(v));
      continue;
    }
    if (i1 < -tol) {
      v.defect = InertialDefect::kNotPositiveSemidefinite;
      v.margin = (scale > 0.0) ? i1 / scale : i1;
      report.fatal.push_back(std::move(v));
      continue;
    }
    if ((i1 + i2 - i3) < -tol) {
      v.defect = InertialDefect::kTriangleInequality;
      v.margin = (scale > 0.0) ? (i1 + i2 - i3) / scale : (i1 + i2 - i3);
      report.fatal.push_back(std::move(v));
      continue;
    }

    // V5 — 텐서 자체는 강체로서 모순이 없으나 movable body 가 질량을 안 진다.
    // M(q) 가 이 DoF 위에서 특이해지므로 동역학 소비자에게는 치명적이지만,
    // FK / Jacobian / IK 만 쓰는 소비자에게는 무해하다. 그래서 throw 가 아니다.
    if (mass == 0.0) {
      v.defect = InertialDefect::kMasslessMovableBody;
      report.degenerate.push_back(std::move(v));
    }
  }

  return report;
}

std::string DescribeInertialViolations(const std::vector<InertialViolation>& violations) {
  std::ostringstream oss;
  for (const auto& v : violations) {
    oss << "  [" << ToString(v.defect) << "] joint='" << v.joint_name << "' link='" << v.body_name
        << "' mass=" << v.mass << " principal_moments=(" << v.principal_moments[0] << ", "
        << v.principal_moments[1] << ", " << v.principal_moments[2] << ")";
    if (v.margin != 0.0) {
      oss << " relative_margin=" << v.margin;
    }
    oss << '\n';
  }
  return oss.str();
}

}  // namespace rtc_urdf_bridge
