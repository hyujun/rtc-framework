// ── CapsuleWorkspace 구현 ─────────────────────────────────────────────────────
#include "rtc_urdf_bridge/collision/capsule_workspace.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/collision/coal-pinocchio-conversions.hpp>
#include <pinocchio/geometry.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace rtc_urdf_bridge {

struct CapsuleWorkspace::Shapes {
  std::vector<std::shared_ptr<coal::Capsule>> robot;
  std::shared_ptr<coal::Capsule> ball;
};

namespace {

constexpr double kDegenerateLength = 1e-12;

// z 축이 axis 를 향하는 회전 (x · y 는 임의의 직교 보완). axis 가 0 이면 항등. Eigen 의
// FromTwoVectors (SVD 분기) 를 피한 이유는 그 경로가 GCC 의 -Wmaybe-uninitialized 를 깨우기
// 때문이다 — 결과는 어느 쪽이든 z 열만 뜻이 있다.
Eigen::Matrix3d RotationWithZ(const Eigen::Vector3d& axis) {
  const double n = axis.norm();
  if (n < kDegenerateLength) {
    return Eigen::Matrix3d::Identity();
  }
  const Eigen::Vector3d z = axis / n;
  const Eigen::Vector3d helper =
      std::abs(z.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d x = helper.cross(z).normalized();
  const Eigen::Vector3d y = z.cross(x);
  Eigen::Matrix3d r;
  r.col(0) = x;
  r.col(1) = y;
  r.col(2) = z;
  return r;
}

bool IsPrismatic(const pinocchio::Model& model, pinocchio::JointIndex j) {
  const std::string name = model.joints[j].shortname();
  return name.find("JointModelP") == 0;  // PX · PY · PZ · PrismaticUnaligned
}

bool Contains(const pinocchio::Model::IndexVector& v, pinocchio::JointIndex j) {
  return std::find(v.begin(), v.end(), j) != v.end();
}

}  // namespace

CapsuleWorkspace::CapsuleWorkspace() : shapes_(std::make_unique<Shapes>()) {}

CapsuleWorkspace::~CapsuleWorkspace() = default;
CapsuleWorkspace::CapsuleWorkspace(CapsuleWorkspace&&) noexcept = default;
CapsuleWorkspace& CapsuleWorkspace::operator=(CapsuleWorkspace&&) noexcept = default;

void CapsuleWorkspace::LeverArmBound(const pinocchio::Model& model, pinocchio::JointIndex joint_a,
                                     pinocchio::JointIndex joint_b, double reach_a, double reach_b,
                                     Eigen::VectorXd& per_v, double& sum) {
  // 한쪽 capsule 의 관절 사슬 (supports: 0 … joint) 에서 다른 쪽 사슬에 없는 관절마다, 그 관절
  // 원점에서 capsule 의 어느 점까지의 거리 상한 = (그 관절 아래 사슬의 placement 평행이동 노름
  // 합) + reach. 공통 조상은 두 capsule 을 같이 움직이므로 거리에 기여하지 않는다. 관절의 v
  // 좌표 전부에 같은 값을 쓴다 (1 자유도 관절이 보통이다).
  per_v.setZero(model.nv);
  sum = 0.0;
  const auto side = [&](pinocchio::JointIndex mine, pinocchio::JointIndex other, double reach) {
    const pinocchio::Model::IndexVector& chain = model.supports[mine];
    const pinocchio::Model::IndexVector& other_chain = model.supports[other];
    for (std::size_t k = 0; k < chain.size(); ++k) {
      const pinocchio::JointIndex j = chain[k];
      if (j == 0 || Contains(other_chain, j)) {
        continue;
      }
      double lever = 1.0;
      if (!IsPrismatic(model, j)) {
        lever = reach;
        for (std::size_t m = k + 1; m < chain.size(); ++m) {
          lever += model.jointPlacements[chain[m]].translation().norm();
        }
      }
      const Eigen::Index v0 = model.joints[j].idx_v();
      const Eigen::Index nv = model.joints[j].nv();
      for (Eigen::Index v = v0; v < v0 + nv; ++v) {
        per_v[v] += lever;
      }
      sum += lever;
    }
  };
  side(joint_a, joint_b, reach_a);
  side(joint_b, joint_a, reach_b);
}

CapsuleBindStatus CapsuleWorkspace::Bind(std::shared_ptr<const CapsuleModel> spec,
                                         const pinocchio::Model& model,
                                         const BindOptions& options) {
  const auto fail = [&](CapsuleBindStatus st, const std::string& msg) {
    bind_error_ = "CapsuleWorkspace::Bind: " + msg;
    geom_model_.reset();
    geom_data_.reset();
    shapes_ = std::make_unique<Shapes>();
    pairs_.clear();
    geom_of_capsule_.clear();
    spec_.reset();
    return st;
  };
  if (bound_) {
    return fail(CapsuleBindStatus::kAlreadyBound, "이미 Bind 된 workspace 입니다");
  }
  if (!spec) {
    return fail(CapsuleBindStatus::kNoSpec, "spec 이 null 입니다");
  }
  spec_ = std::move(spec);
  const CapsuleModelSpec& s = spec_->Spec();

  geom_model_ = std::make_unique<pinocchio::GeometryModel>();
  geom_of_capsule_.assign(s.capsules.size(), 0);
  std::vector<pinocchio::JointIndex> joint_of_capsule(s.capsules.size(), 0);
  std::vector<double> reach_of_capsule(s.capsules.size(), 0.0);

  for (std::size_t i = 0; i < s.capsules.size(); ++i) {
    const CapsuleSpec& c = s.capsules[i];
    if (!model.existFrame(c.link, pinocchio::BODY)) {
      return fail(CapsuleBindStatus::kLinkMissing,
                  "capsule '" + c.name + "' 의 link '" + c.link + "' 가 모델에 없습니다");
    }
    const pinocchio::FrameIndex fid = model.getFrameId(c.link, pinocchio::BODY);
    const pinocchio::Frame& frame = model.frames[fid];
    const pinocchio::JointIndex joint = frame.parentJoint;
    // 끝점을 link 좌표계 → 부모 관절 좌표계로 (fixed 로 묶인 링크는 placement 가 항등이 아니다).
    const Eigen::Vector3d a = frame.placement.act(c.p0);
    const Eigen::Vector3d b = frame.placement.act(c.p1);
    const Eigen::Vector3d axis = b - a;
    const double length = axis.norm();
    const pinocchio::SE3 placement(RotationWithZ(axis), 0.5 * (a + b));
    auto shape = std::make_shared<coal::Capsule>(c.radius, length);  // lz = 전체 길이
    shapes_->robot.push_back(shape);
    pinocchio::GeometryObject obj(c.name, joint, fid, placement, shape);
    geom_of_capsule_[i] = geom_model_->addGeometryObject(obj);
    joint_of_capsule[i] = joint;
    reach_of_capsule[i] = std::max(a.norm(), b.norm()) + c.radius;
  }
  n_robot_geoms_ = geom_model_->ngeoms;

  pairs_.clear();
  for (std::size_t i = 0; i < s.pairs.size(); ++i) {
    const auto [ia, ib] = spec_->PairIndices(i);
    const std::string where = "쌍 (" + s.capsules[ia].name + ", " + s.capsules[ib].name + ")";
    const pinocchio::JointIndex ja = joint_of_capsule[ia];
    const pinocchio::JointIndex jb = joint_of_capsule[ib];
    if (ja == 0 && jb == 0) {
      return fail(CapsuleBindStatus::kBothOnUniverse,
                  where + " 의 두 capsule 이 모두 universe 에 붙어 있습니다");
    }
    if (ja == jb) {
      return fail(CapsuleBindStatus::kSameParentJoint,
                  where + " 의 두 capsule 이 같은 관절 '" + model.names[ja] +
                      "' 에 붙어 있습니다 (fixed 로 묶인 링크 — 기울기 0)");
    }
    geom_model_->addCollisionPair(
        pinocchio::CollisionPair(geom_of_capsule_[ia], geom_of_capsule_[ib]));
    CapsulePairInfo info;
    info.capsule_a = ia;
    info.capsule_b = ib;
    info.is_ball = false;
    info.joint_a = ja;
    info.joint_b = jb;
    LeverArmBound(model, ja, jb, reach_of_capsule[ia], reach_of_capsule[ib], info.lever_arm,
                  info.lever_arm_bound);
    pairs_.push_back(info);
  }
  n_self_pairs_ = pairs_.size();

  with_ball_ = options.with_ball;
  if (with_ball_) {
    if (s.ball_capsules.empty()) {
      return fail(CapsuleBindStatus::kBallWithoutSpec,
                  "with_ball 인데 spec 의 ball_capsules 가 비어 있습니다");
    }
    shapes_->ball = std::make_shared<coal::Capsule>(0.0, 0.0);
    pinocchio::GeometryObject ball("__ball", 0, pinocchio::SE3::Identity(), shapes_->ball);
    ball_geom_ = geom_model_->addGeometryObject(ball);
    for (std::size_t i = 0; i < s.ball_capsules.size(); ++i) {
      const std::size_t ib = spec_->BallCapsuleIndex(i);
      const pinocchio::JointIndex jb = joint_of_capsule[ib];
      if (options.catch_frame != 0) {
        const pinocchio::JointIndex jc = model.frames[options.catch_frame].parentJoint;
        const bool in_support = jb != 0 && Contains(model.supports[jc], jb);
        const bool in_subtree = Contains(model.subtrees[jc], jb);
        if (in_support || in_subtree) {
          return fail(CapsuleBindStatus::kBallInSubtree,
                      "ball capsule '" + s.capsules[ib].name + "' 가 catch frame '" +
                          model.frames[options.catch_frame].name +
                          "' 의 support · subtree 에 붙어 있습니다 (포구 손이 공을 피하게 된다)");
        }
      }
      geom_model_->addCollisionPair(pinocchio::CollisionPair(ball_geom_, geom_of_capsule_[ib]));
      CapsulePairInfo info;
      info.capsule_a = ib;  // 쓰지 않는다 — is_ball
      info.capsule_b = ib;
      info.is_ball = true;
      info.joint_a = 0;
      info.joint_b = jb;
      LeverArmBound(model, 0, jb, 0.0, reach_of_capsule[ib], info.lever_arm, info.lever_arm_bound);
      pairs_.push_back(info);
    }
  }

  // GeometryData — distance functor (coal 닫힌식 조회) 가 여기서 만들어진다. 지원하지 않는 쌍이면
  // coal 이 throw 하지만 capsule–capsule 뿐이라 일어나지 않는다 (non-RT).
  geom_data_ = std::make_unique<pinocchio::GeometryData>(*geom_model_);
  // pinocchio's SE3 default constructor leaves the rotation uninitialised: a
  // geom read before its first placement (the ball before SetBall) would feed
  // garbage to coal. Every placement starts at the identity.
  for (pinocchio::SE3& m : geom_data_->oMg) {
    m.setIdentity();
  }
  for (coal::DistanceRequest& req : geom_data_->distanceRequests) {
    req.enable_signed_distance = true;  // 침범이면 d < 0, normal · 최근접점은 그대로 유효
  }

  nv_ = model.nv;
  njoints_ = model.njoints;
  nframes_ = model.nframes;

  const std::size_t np = pairs_.size();
  distance_.assign(np, std::numeric_limits<double>::quiet_NaN());
  valid_.assign(np, 0);
  nearest_.assign(2 * np, Eigen::Vector3d::Zero());
  normal_.assign(np, Eigen::Vector3d::Zero());
  gradient_.setZero(nv_, static_cast<Eigen::Index>(np));
  J_a_.setZero(6, nv_);
  J_b_.setZero(6, nv_);
  Jp_.setZero(3, nv_);

  bind_error_.clear();
  bound_ = true;
  return CapsuleBindStatus::kOk;
}

bool CapsuleWorkspace::FingerprintMatches(const pinocchio::Model& model) const noexcept {
  return bound_ && model.nv == nv_ && model.njoints == njoints_ && model.nframes == nframes_;
}

bool CapsuleWorkspace::UpdatePlacements(const pinocchio::Model& model,
                                        const pinocchio::Data& data) noexcept {
  if (!FingerprintMatches(model)) {
    return false;
  }
  for (std::size_t i = 0; i < n_robot_geoms_; ++i) {
    const pinocchio::GeometryObject& obj = geom_model_->geometryObjects[i];
    if (obj.parentJoint > 0) {
      geom_data_->oMg[i] = data.oMi[obj.parentJoint] * obj.placement;
    } else {
      geom_data_->oMg[i] = obj.placement;
    }
  }
  return true;
}

void CapsuleWorkspace::SetBall(const Eigen::Vector3d& p0_world, const Eigen::Vector3d& p1_world,
                               double radius) noexcept {
  if (!bound_ || !with_ball_) {
    return;
  }
  const Eigen::Vector3d axis = p1_world - p0_world;
  shapes_->ball->halfLength = 0.5 * axis.norm();
  shapes_->ball->radius = radius;
  geom_data_->oMg[ball_geom_].rotation() = RotationWithZ(axis);
  geom_data_->oMg[ball_geom_].translation() = 0.5 * (p0_world + p1_world);
}

std::size_t CapsuleWorkspace::EvaluateDistances() noexcept {
  if (!bound_) {
    return 0;
  }
  std::size_t invalid = 0;
  for (std::size_t p = 0; p < pairs_.size(); ++p) {
    const pinocchio::CollisionPair& cp = geom_model_->collisionPairs[p];
    coal::DistanceRequest& req = geom_data_->distanceRequests[p];
    coal::DistanceResult& res = geom_data_->distanceResults[p];
    res.clear();
    // distance_functors 직접 호출 — pinocchio::computeDistance 의 throw 경로를 지난다.
    const double d = geom_data_->distance_functors[p](
        pinocchio::toCoalTransform3s(geom_data_->oMg[cp.first]),
        pinocchio::toCoalTransform3s(geom_data_->oMg[cp.second]), req, res);
    const bool ok = std::isfinite(d) && res.normal.allFinite() &&
                    res.nearest_points[0].allFinite() && res.nearest_points[1].allFinite();
    valid_[p] = ok ? 1 : 0;
    if (!ok) {
      ++invalid;
      distance_[p] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    distance_[p] = d;
    nearest_[2 * p] = res.nearest_points[0];
    nearest_[2 * p + 1] = res.nearest_points[1];
    normal_[p] = res.normal;
  }
  return invalid;
}

bool CapsuleWorkspace::EvaluateGradients(const pinocchio::Model& model,
                                         const pinocchio::Data& data) noexcept {
  if (!FingerprintMatches(model)) {
    return false;
  }
  // 점 Jacobian 의 기여를 g 에 더한다: g += sign · nᵀ (J_v − [r]× J_ω). 열 단위 cross 로
  // Eigen 의 GEMM 경로 (임시 버퍼) 를 피한다.
  const auto accumulate =
      [&](pinocchio::JointIndex joint, const Eigen::Vector3d& point, const Eigen::Vector3d& n,
          double sign, Eigen::Matrix<double, 6, Eigen::Dynamic>& J, Eigen::Ref<Eigen::VectorXd> g) {
        if (joint == 0) {
          return;
        }
        J.setZero();
        pinocchio::getJointJacobian(model, data, joint, pinocchio::LOCAL_WORLD_ALIGNED, J);
        const Eigen::Vector3d r = point - data.oMi[joint].translation();
        for (Eigen::Index c = 0; c < J.cols(); ++c) {
          const Eigen::Vector3d v = J.col(c).head<3>();
          const Eigen::Vector3d w = J.col(c).tail<3>();
          g[c] += sign * n.dot(v - r.cross(w));
        }
      };
  for (std::size_t p = 0; p < pairs_.size(); ++p) {
    Eigen::Ref<Eigen::VectorXd> g = gradient_.col(static_cast<Eigen::Index>(p));
    g.setZero();
    if (valid_[p] == 0) {
      continue;
    }
    const CapsulePairInfo& info = pairs_[p];
    const Eigen::Vector3d& n = normal_[p];
    accumulate(info.joint_b, nearest_[2 * p + 1], n, +1.0, J_b_, g);
    if (!info.is_ball) {
      accumulate(info.joint_a, nearest_[2 * p], n, -1.0, J_a_, g);
    }
  }
  return true;
}

double CapsuleWorkspace::MinDistance() const noexcept {
  double m = std::numeric_limits<double>::infinity();
  for (std::size_t p = 0; p < pairs_.size(); ++p) {
    if (valid_[p] != 0) {
      m = std::min(m, distance_[p]);
    }
  }
  return m;
}

}  // namespace rtc_urdf_bridge
