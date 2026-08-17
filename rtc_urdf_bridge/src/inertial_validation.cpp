// ── 관성 파라미터 물리 실현가능성 검증 구현 ──────────────────────────────────
#include "rtc_urdf_bridge/inertial_validation.hpp"

#include "rtc_urdf_bridge/urdf_logging.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

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
    case InertialDefect::kMasslessWithInertia:
      return "V6:massless-with-inertia";
    case InertialDefect::kMasslessMovableBody:
      return "V5:massless-movable-body";
  }
  return "V?:unknown";
}

InertialValidationReport ValidateInertias(const pinocchio::Model& model, double rel_tol) {
  InertialValidationReport report;

  // 고유값 분해기는 루프 밖에서 한 번만 만든다 (내부 버퍼 재사용).
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;

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
    v.mass = mass;

    // body_name 해석은 frames 전체 스캔이라 위반이 확정된 분기에서만 치른다 —
    // 결함 없는 관절(대다수)에서까지 부르면 O(njoints × nframes) 가 된다.
    const auto record = [&](std::vector<InertialViolation>& lane, InertialDefect defect,
                            double margin) {
      v.defect = defect;
      v.margin = margin;
      v.body_name = PrimaryBodyName(model, jid);
      lane.push_back(std::move(v));
    };

    // 비유한 검사는 **세 성분 모두**를 봐야 한다. mass·CoM 기준 관성이 멀쩡해도
    // lever(CoM 위치)가 NaN 이면 평행축 이동에서 M(q) 가 통째로 NaN 이 되는데,
    // Inertia 는 그 lever 를 회전관성과 분리해 들고 있어 inertia().allFinite() 로는
    // 잡히지 않는다.
    if (!std::isfinite(mass) || !inertia.allFinite() || !body.lever().allFinite()) {
      record(report.fatal, InertialDefect::kNonFinite, 0.0);
      continue;
    }

    solver.compute(inertia, Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success) {
      // 유한한 입력에서도 성분이 배정도 한계 근처면 분해가 실패할 수 있다. 이때
      // eigenvalues() 는 쓰레기이므로 "통과" 로 흘리면 안 된다 — 판정 불가는
      // 통과가 아니라 거부다.
      record(report.fatal, InertialDefect::kNonFinite, 0.0);
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
      record(report.fatal, InertialDefect::kNegativeMass, 0.0);
      continue;
    }
    // 아래 두 분기 안에서는 scale > 0 이 보장된다: 어느 쪽이든 부등식이 성립하려면
    // 좌변이 음수여야 하는데, scale == 0 이면 I1 = I2 = I3 = 0 이라 좌변이 0 이다.
    if (i1 < -tol) {
      record(report.fatal, InertialDefect::kNotPositiveSemidefinite, i1 / scale);
      continue;
    }
    if ((i1 + i2 - i3) < -tol) {
      record(report.fatal, InertialDefect::kTriangleInequality, (i1 + i2 - i3) / scale);
      continue;
    }

    if (mass == 0.0) {
      // 질량 0 강체는 회전관성도 0 이어야 한다. 둘이 어긋나면(관성 ≠ 0) 어떤
      // 질량분포로도 실현할 수 없으므로 V6 다 — 여기서 scale > 0 이 곧 "텐서가
      // 0 이 아니다" 이다 (PSD 분기를 지나 왔으므로 고유값은 모두 ≥ -tol).
      if (scale > 0.0) {
        record(report.fatal, InertialDefect::kMasslessWithInertia, 0.0);
        continue;
      }
      // V5 — 질량도 관성도 0. 강체로서 모순은 없으나 movable body 가 질량을 안
      // 진다. M(q) 가 이 DoF 위에서 특이해지므로 동역학 소비자에게는 치명적이지만,
      // FK / Jacobian / IK 만 쓰는 소비자에게는 무해하다. 그래서 throw 가 아니다.
      record(report.degenerate, InertialDefect::kMasslessMovableBody, 0.0);
    }
  }

  return report;
}

InertialValidationReport EnforceInertialGate(const pinocchio::Model& model,
                                             std::string_view context) {
  InertialValidationReport report = ValidateInertias(model);

  if (!report.degenerate.empty()) {
    RCLCPP_WARN(rtc::urdf::logging::BuilderLogger(),
                "관성 게이트: movable body %zu 개가 질량을 지지 않는다 — 이 모델의 M(q) 는 해당 "
                "DoF 에서 특이하다. FK/Jacobian/IK 소비는 안전하나 동역학 소비자는 "
                "IsFullModelDynamicsCapable() 로 자기 검사할 것.\n%s",
                report.degenerate.size(), DescribeInertialViolations(report.degenerate).c_str());
  }

  if (!report.fatal.empty()) {
    const std::string detail = DescribeInertialViolations(report.fatal);
    RCLCPP_ERROR(rtc::urdf::logging::BuilderLogger(),
                 "관성 게이트: body %zu 개의 관성이 물리적으로 실현 불가능하다.\n%s",
                 report.fatal.size(), detail.c_str());
    throw std::runtime_error(std::string(context) + ": 물리적으로 실현 불가능한 관성 " +
                             std::to_string(report.fatal.size()) + "건\n" + detail);
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
