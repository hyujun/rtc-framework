// ── test_capsule_workspace — Bind 거부 · ∇_q d 의 FD 대조 · lever arm 상한 · 배치 경로 ──
//
// 기울기는 유한하고 매끄럽게 틀릴 수 있는 양이라 (부호 뒤집힘, J_v 와 J_ω 의 행 교환, 점
// Jacobian 의 [r]× 누락) 자코비안을 전혀 쓰지 않는 oracle — 거리의 중앙차분 — 로 대조한다.
// 분리 구간과 **침범 구간** 둘 다 본다: slack 행은 침범 상태에서 선형화되는데 coal 의 normal
// 은 침범에서 부호를 바꾸므로 (#642 검토 §1) 그 구간의 기울기가 따로 검사돼야 한다.
//
// 제외 (기록): |d| < 1e-3 (최근접점이 반지름 경계를 지나는 점 — 미분 불연속) 과 두 축이
// 서로 교차하는 자세 (축 사이 거리 < 1e-3 — 최근접점 비유일). 제외 수는 출력에 남긴다.
#include "rtc_urdf_bridge/collision/capsule_workspace.hpp"
#include "test_urdf_path.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;
using rtc::test::TestUrdfPath;

namespace {

constexpr double kFdStep = 1e-6;
constexpr double kFdAbs = 1e-7;
constexpr double kFdRel = 1e-5;
constexpr double kExclude = 1e-3;
constexpr int kSamples = 200;
constexpr double kQRange = 1.2;

struct Capsule {
  const char* name;
  const char* link;
  double p1x, p1y, p1z;
  double radius;
};

// 몸통 하나, 팔마다 셋 (상완 · 전완 · 손). 손은 fixed 로 묶인 링크 (hand_base) 에 둬 그 placement
// 경로도 지나간다.
const std::vector<Capsule> kCapsules = {
    {"torso", "torso_link", 0.0, 0.0, 0.25, 0.08},
    {"l_upper", "left_shoulder_yaw_link", 0.012, 0.0, -0.12, 0.04},
    {"l_fore", "left_elbow_link", 0.1, 0.0, 0.0, 0.035},
    {"l_hand", "left_wrist_yaw_link", 0.05, 0.0, 0.0, 0.03},
    {"r_upper", "right_shoulder_yaw_link", 0.012, 0.0, -0.13, 0.04},
    {"r_fore", "right_elbow_link", 0.11, 0.0, 0.0, 0.035},
    {"r_hand", "hand_base", 0.06, 0.0, 0.0, 0.03},
};

const std::vector<std::pair<const char*, const char*>> kPairs = {
    {"l_upper", "r_upper"}, {"l_fore", "r_fore"},  {"l_hand", "r_hand"},  {"l_fore", "torso"},
    {"r_fore", "torso"},    {"l_hand", "torso"},   {"r_hand", "torso"},   {"l_hand", "r_fore"},
    {"r_hand", "l_fore"},   {"l_upper", "r_fore"}, {"l_fore", "r_upper"},
};

std::string SpecYaml(double radius_scale, bool sphere_left_hand,
                     const std::vector<std::pair<const char*, const char*>>& pairs,
                     const std::vector<const char*>& balls) {
  std::string y = "collision_capsules:\n  capsules:\n";
  for (const Capsule& c : kCapsules) {
    const bool sphere = sphere_left_hand && std::string(c.name) == "l_hand";
    y += "    - {name: " + std::string(c.name) + ", link: " + c.link + ", p0: [0, 0, 0], p1: [" +
         std::to_string(sphere ? 0.0 : c.p1x) + ", " + std::to_string(sphere ? 0.0 : c.p1y) + ", " +
         std::to_string(sphere ? 0.0 : c.p1z) +
         "], radius: " + std::to_string(c.radius * radius_scale) + "}\n";
  }
  y += "  pairs:\n";
  for (const auto& [a, b] : pairs) {
    y += std::string("    - [") + a + ", " + b + "]\n";
  }
  if (!balls.empty()) {
    y += "  ball_capsules: [";
    for (std::size_t i = 0; i < balls.size(); ++i) {
      y += std::string(i ? ", " : "") + balls[i];
    }
    y += "]\n";
  }
  return y;
}

pinocchio::Model LoadModel() {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(TestUrdfPath("dual_arm_trunk_hand.urdf"), model);
  return model;
}

// 고정 공 (world): 왼팔 쪽, 팔 길이 안.
const Eigen::Vector3d kBallP0(0.25, 0.25, 0.95);
const Eigen::Vector3d kBallP1(0.15, 0.35, 0.85);
constexpr double kBallRadius = 0.035;

struct Fixture {
  pinocchio::Model model = LoadModel();
  pinocchio::Data data{model};
  std::shared_ptr<const rub::CapsuleModel> spec;
  rub::CapsuleWorkspace ws;

  void Bind(const std::string& yaml, bool with_ball) {
    spec = rub::CapsuleModel::ParseYaml(yaml, "fixture");
    rub::CapsuleWorkspace::BindOptions opt;
    opt.with_ball = with_ball;
    opt.catch_frame = model.getFrameId("catch_frame", pinocchio::BODY);
    const rub::CapsuleBindStatus st = ws.Bind(spec, model, opt);
    ASSERT_EQ(st, rub::CapsuleBindStatus::kOk) << ws.BindError();
  }

  // FK (oMi · oMf) → 배치 → 공 → 거리. 기울기는 호출자가 computeJointJacobians 뒤에 따로.
  void Distances(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    ASSERT_TRUE(ws.UpdatePlacements(model, data));
    ws.SetBall(kBallP0, kBallP1, kBallRadius);
    EXPECT_EQ(ws.EvaluateDistances(), 0U);
  }

  void DistancesAndGradients(const Eigen::VectorXd& q) {
    pinocchio::computeJointJacobians(model, data, q);  // FK + data.J
    pinocchio::updateFramePlacements(model, data);
    ASSERT_TRUE(ws.UpdatePlacements(model, data));
    ws.SetBall(kBallP0, kBallP1, kBallRadius);
    EXPECT_EQ(ws.EvaluateDistances(), 0U);
    ASSERT_TRUE(ws.EvaluateGradients(model, data));
  }

  // spec capsule i 의 world 끝점 (BODY frame 경유 — workspace 와 독립인 경로).
  void WorldEndpoints(std::size_t i, Eigen::Vector3d& p0, Eigen::Vector3d& p1) const {
    const rub::CapsuleSpec& c = spec->Spec().capsules[i];
    const pinocchio::SE3& oMf = data.oMf[model.getFrameId(c.link, pinocchio::BODY)];
    p0 = oMf.act(c.p0);
    p1 = oMf.act(c.p1);
  }

  void PairEndpoints(std::size_t p, Eigen::Vector3d& a0, Eigen::Vector3d& a1, Eigen::Vector3d& b0,
                     Eigen::Vector3d& b1, double& ra, double& rb) const {
    const rub::CapsulePairInfo& info = ws.Pair(p);
    if (info.is_ball) {
      a0 = kBallP0;
      a1 = kBallP1;
      ra = kBallRadius;
    } else {
      WorldEndpoints(info.capsule_a, a0, a1);
      ra = spec->Spec().capsules[info.capsule_a].radius;
    }
    WorldEndpoints(info.capsule_b, b0, b1);
    rb = spec->Spec().capsules[info.capsule_b].radius;
  }
};

Eigen::VectorXd RandomQ(const pinocchio::Model& model, std::mt19937& rng) {
  std::uniform_real_distribution<double> u(-kQRange, kQRange);
  Eigen::VectorXd q(model.nq);
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    q[i] = u(rng);
  }
  return q;
}

// 선분–선분 최소 거리 (Ericson, Real-Time Collision Detection 5.1.9) — coal 과 독립인 구현.
double SegmentDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& q1,
                       const Eigen::Vector3d& p2, const Eigen::Vector3d& q2) {
  const Eigen::Vector3d d1 = q1 - p1;
  const Eigen::Vector3d d2 = q2 - p2;
  const Eigen::Vector3d r = p1 - p2;
  const double a = d1.dot(d1);
  const double e = d2.dot(d2);
  const double f = d2.dot(r);
  constexpr double eps = 1e-30;
  double s = 0.0;
  double t = 0.0;
  if (a <= eps && e <= eps) {
    return (p1 - p2).norm();
  }
  if (a <= eps) {
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= eps) {
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;
      s = denom != 0.0 ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  return ((p1 + d1 * s) - (p2 + d2 * t)).norm();
}

struct FdStats {
  int separated{0};
  int penetrating{0};
  int excluded_near_zero{0};
  int excluded_axis_cross{0};
  int excluded_out_of_range{0};
  double worst_ratio{0.0};
};

// 모든 쌍 · 모든 sample 의 FD 대조. want_penetrating 이면 침범 구간만, 아니면 분리 구간만 센다.
FdStats CheckGradientsAgainstFd(Fixture& fx, bool want_penetrating, std::mt19937& rng) {
  FdStats st;
  const Eigen::Index nv = fx.model.nv;
  EXPECT_EQ(fx.model.nq, nv) << "이 fixture 는 revolute 만 — q 와 v 가 같은 좌표";
  const std::size_t np = fx.ws.NumPairs();
  std::vector<Eigen::VectorXd> fd(np, Eigen::VectorXd::Zero(nv));
  std::vector<double> d_plus(np);
  std::vector<double> d_minus(np);
  for (int s = 0; s < kSamples; ++s) {
    const Eigen::VectorXd q = RandomQ(fx.model, rng);
    fx.DistancesAndGradients(q);
    std::vector<double> d(np);
    std::vector<Eigen::VectorXd> g(np);
    std::vector<bool> use(np, true);
    for (std::size_t p = 0; p < np; ++p) {
      d[p] = fx.ws.Distance(p);
      g[p] = fx.ws.Gradient(p);
      Eigen::Vector3d a0, a1, b0, b1;
      double ra = 0.0;
      double rb = 0.0;
      fx.PairEndpoints(p, a0, a1, b0, b1, ra, rb);
      const double axis = SegmentDistance(a0, a1, b0, b1);
      if (std::abs(d[p]) < kExclude) {
        use[p] = false;
        ++st.excluded_near_zero;
      } else if (axis < kExclude) {
        use[p] = false;
        ++st.excluded_axis_cross;
      } else if (want_penetrating) {
        if (d[p] > 0.0 || d[p] < -(ra + rb) + kExclude) {
          use[p] = false;
          ++st.excluded_out_of_range;
        }
      } else if (d[p] < 0.0) {
        use[p] = false;
        ++st.excluded_out_of_range;
      }
    }
    for (Eigen::Index j = 0; j < nv; ++j) {
      Eigen::VectorXd qp = q;
      qp[j] += kFdStep;
      fx.Distances(qp);
      for (std::size_t p = 0; p < np; ++p) {
        d_plus[p] = fx.ws.Distance(p);
      }
      qp[j] = q[j] - kFdStep;
      fx.Distances(qp);
      for (std::size_t p = 0; p < np; ++p) {
        d_minus[p] = fx.ws.Distance(p);
      }
      for (std::size_t p = 0; p < np; ++p) {
        fd[p][j] = (d_plus[p] - d_minus[p]) / (2.0 * kFdStep);
      }
    }
    for (std::size_t p = 0; p < np; ++p) {
      if (!use[p]) {
        continue;
      }
      (d[p] > 0.0 ? st.separated : st.penetrating) += 1;
      const double err = (fd[p] - g[p]).cwiseAbs().maxCoeff();
      const double tol = kFdAbs + kFdRel * g[p].cwiseAbs().maxCoeff();
      st.worst_ratio = std::max(st.worst_ratio, err / tol);
      EXPECT_LE(err, tol) << "sample " << s << " pair " << p << " d=" << d[p]
                          << "\n  fd=" << fd[p].transpose() << "\n  an=" << g[p].transpose();
    }
  }
  return st;
}

void PrintStats(const char* what, const FdStats& st) {
  std::cout << "[FD " << what << "] separated=" << st.separated << " penetrating=" << st.penetrating
            << " excluded(|d|<1e-3)=" << st.excluded_near_zero
            << " excluded(axis cross)=" << st.excluded_axis_cross
            << " excluded(out of range)=" << st.excluded_out_of_range
            << " worst err/tol=" << st.worst_ratio << "\n";
}

}  // namespace

// ── Bind ────────────────────────────────────────────────────────────────────

TEST(CapsuleWorkspace, BindsAndReportsPairs) {
  Fixture fx;
  fx.Bind(SpecYaml(1.0, false, kPairs, {"l_upper", "l_fore", "l_hand"}), true);
  EXPECT_EQ(fx.ws.NumSelfPairs(), kPairs.size());
  EXPECT_EQ(fx.ws.NumBallPairs(), 3U);
  EXPECT_EQ(fx.ws.NumPairs(), kPairs.size() + 3);
  EXPECT_TRUE(fx.ws.HasBall());
  EXPECT_EQ(fx.ws.Nv(), fx.model.nv);
  for (std::size_t p = 0; p < fx.ws.NumPairs(); ++p) {
    const rub::CapsulePairInfo& info = fx.ws.Pair(p);
    EXPECT_GT(info.lever_arm_bound, 0.0);
    EXPECT_NE(info.joint_b, 0U);
    if (info.is_ball) {
      EXPECT_EQ(info.joint_a, 0U);
    } else {
      EXPECT_NE(info.joint_a, info.joint_b);
    }
  }
  // 손 capsule 은 fixed 로 묶인 hand_base 에 있어도 부모 관절이 right_wrist_yaw_joint 다.
  const rub::CapsulePairInfo& hands = fx.ws.Pair(2);
  EXPECT_EQ(fx.model.names[hands.joint_b], "right_wrist_yaw_joint");
}

TEST(CapsuleWorkspace, BindRejectsByName) {
  const pinocchio::Model model = LoadModel();
  const auto bind = [&](const std::string& yaml, bool with_ball, bool with_catch) {
    rub::CapsuleWorkspace ws;
    rub::CapsuleWorkspace::BindOptions opt;
    opt.with_ball = with_ball;
    opt.catch_frame = with_catch ? model.getFrameId("catch_frame", pinocchio::BODY) : 0;
    const rub::CapsuleBindStatus st =
        ws.Bind(rub::CapsuleModel::ParseYaml(yaml, "reject"), model, opt);
    EXPECT_EQ(ws.IsBound(), st == rub::CapsuleBindStatus::kOk);
    return std::make_pair(st, ws.BindError());
  };

  {
    const auto [st, msg] = bind(R"(
collision_capsules:
  capsules:
    - {name: ghost_cap, link: no_such_link, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1}
    - {name: torso, link: torso_link, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1}
  pairs: [[ghost_cap, torso]])",
                                false, false);
    EXPECT_EQ(st, rub::CapsuleBindStatus::kLinkMissing);
    EXPECT_NE(msg.find("no_such_link"), std::string::npos) << msg;
    EXPECT_NE(msg.find("ghost_cap"), std::string::npos) << msg;
  }
  {
    // hand_base 는 right_wrist_yaw_link 에 fixed — 같은 관절, 기울기 0.
    const auto [st, msg] = bind(R"(
collision_capsules:
  capsules:
    - {name: r_hand, link: hand_base, p0: [0, 0, 0], p1: [0.06, 0, 0], radius: 0.03}
    - {name: r_wrist, link: right_wrist_yaw_link, p0: [0, 0, 0], p1: [0.03, 0, 0], radius: 0.02}
  pairs: [[r_hand, r_wrist]])",
                                false, false);
    EXPECT_EQ(st, rub::CapsuleBindStatus::kSameParentJoint);
    EXPECT_NE(msg.find("right_wrist_yaw_joint"), std::string::npos) << msg;
    EXPECT_NE(msg.find("r_hand"), std::string::npos) << msg;
  }
  {
    // 포구 손 (catch_frame 의 support) 에 ball capsule — 손이 공을 피하게 된다.
    const auto [st, msg] = bind(R"(
collision_capsules:
  capsules:
    - {name: r_hand, link: hand_base, p0: [0, 0, 0], p1: [0.06, 0, 0], radius: 0.03}
  ball_capsules: [r_hand])",
                                true, true);
    EXPECT_EQ(st, rub::CapsuleBindStatus::kBallInSubtree);
    EXPECT_NE(msg.find("r_hand"), std::string::npos) << msg;
    EXPECT_NE(msg.find("catch_frame"), std::string::npos) << msg;
  }
  {
    // 포구 손의 조상 (전완) 도 support 다.
    const auto [st, msg] = bind(R"(
collision_capsules:
  capsules:
    - {name: r_fore, link: right_elbow_link, p0: [0, 0, 0], p1: [0.11, 0, 0], radius: 0.035}
  ball_capsules: [r_fore])",
                                true, true);
    EXPECT_EQ(st, rub::CapsuleBindStatus::kBallInSubtree);
    EXPECT_NE(msg.find("r_fore"), std::string::npos) << msg;
  }
  {
    // catch frame 의 subtree (손가락).
    const auto [st, msg] = bind(R"(
collision_capsules:
  capsules:
    - {name: finger, link: finger_a_distal_link, p0: [0, 0, 0], p1: [0.01, 0, 0], radius: 0.005}
  ball_capsules: [finger])",
                                true, true);
    EXPECT_EQ(st, rub::CapsuleBindStatus::kBallInSubtree);
    EXPECT_NE(msg.find("finger"), std::string::npos) << msg;
  }
  {
    // catch frame 을 주지 않으면 같은 spec 이 통과한다 — 검사는 옵션이다.
    const auto [st, msg] = bind(R"(
collision_capsules:
  capsules:
    - {name: r_hand, link: hand_base, p0: [0, 0, 0], p1: [0.06, 0, 0], radius: 0.03}
  ball_capsules: [r_hand])",
                                true, false);
    EXPECT_EQ(st, rub::CapsuleBindStatus::kOk) << msg;
  }
  {
    const auto [st, msg] = bind(R"(
collision_capsules:
  capsules:
    - {name: torso, link: torso_link, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1})",
                                true, false);
    EXPECT_EQ(st, rub::CapsuleBindStatus::kBallWithoutSpec);
    EXPECT_NE(msg.find("ball_capsules"), std::string::npos) << msg;
  }
}

TEST(CapsuleWorkspace, BindRejectsBothOnUniverseAndRebind) {
  // 손으로 지은 모델: 관절 하나, universe 에 BODY frame 둘.
  pinocchio::Model model;
  const pinocchio::JointIndex j1 =
      model.addJoint(0, pinocchio::JointModelRX(), pinocchio::SE3::Identity(), "j1");
  model.addJointFrame(j1);
  model.appendBodyToJoint(j1, pinocchio::Inertia::Identity(), pinocchio::SE3::Identity());
  model.addBodyFrame("link1", j1);
  model.addBodyFrame("base_a", 0,
                     pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.0)));
  model.addBodyFrame("base_b", 0,
                     pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.5, 0.0, 0.0)));
  const char* yaml = R"(
collision_capsules:
  capsules:
    - {name: ca, link: base_a, p0: [0, 0, 0], p1: [0, 0, 0.1], radius: 0.05}
    - {name: cb, link: base_b, p0: [0, 0, 0], p1: [0, 0, 0.1], radius: 0.05}
    - {name: c1, link: link1, p0: [0, 0, 0], p1: [0.2, 0, 0], radius: 0.05}
  pairs: [[ca, cb]])";
  rub::CapsuleWorkspace ws;
  const rub::CapsuleBindStatus st =
      ws.Bind(rub::CapsuleModel::ParseYaml(yaml, "universe"), model, {});
  EXPECT_EQ(st, rub::CapsuleBindStatus::kBothOnUniverse);
  EXPECT_NE(ws.BindError().find("ca"), std::string::npos) << ws.BindError();
  EXPECT_FALSE(ws.IsBound());

  // 실패한 workspace 는 다시 Bind 할 수 있다; universe–관절 쌍은 허용 (한쪽 기울기만 0).
  const char* ok = R"(
collision_capsules:
  capsules:
    - {name: ca, link: base_a, p0: [0, 0, 0], p1: [0, 0, 0.1], radius: 0.05}
    - {name: c1, link: link1, p0: [0, 0, 0], p1: [0.2, 0, 0], radius: 0.05}
  pairs: [[ca, c1]])";
  ASSERT_EQ(ws.Bind(rub::CapsuleModel::ParseYaml(ok, "ok"), model, {}), rub::CapsuleBindStatus::kOk)
      << ws.BindError();
  EXPECT_TRUE(ws.IsBound());
  EXPECT_EQ(ws.Bind(rub::CapsuleModel::ParseYaml(ok, "ok"), model, {}),
            rub::CapsuleBindStatus::kAlreadyBound);
  rub::CapsuleWorkspace fresh;
  EXPECT_EQ(fresh.Bind(nullptr, model, {}), rub::CapsuleBindStatus::kNoSpec);
}

// ── 배치 경로 · 지문 ────────────────────────────────────────────────────────

TEST(CapsuleWorkspace, PlacementPathMatchesIndependentSegmentDistance) {
  Fixture fx;
  fx.Bind(SpecYaml(1.0, false, kPairs, {"l_upper", "l_fore", "l_hand"}), true);
  std::mt19937 rng(642);
  for (int s = 0; s < 50; ++s) {
    fx.Distances(RandomQ(fx.model, rng));
    for (std::size_t p = 0; p < fx.ws.NumPairs(); ++p) {
      Eigen::Vector3d a0, a1, b0, b1;
      double ra = 0.0;
      double rb = 0.0;
      fx.PairEndpoints(p, a0, a1, b0, b1, ra, rb);
      const double expected = SegmentDistance(a0, a1, b0, b1) - ra - rb;
      ASSERT_TRUE(fx.ws.PairValid(p));
      EXPECT_NEAR(fx.ws.Distance(p), expected, 1e-12) << "pair " << p;
      // 최근접점은 각 형상 위에 있고 normal 은 o1 → o2, d = n · (p2 − p1).
      const Eigen::Vector3d diff = fx.ws.NearestPoint(p, 1) - fx.ws.NearestPoint(p, 0);
      EXPECT_NEAR(fx.ws.Normal(p).norm(), 1.0, 1e-12);
      EXPECT_NEAR(fx.ws.Normal(p).dot(diff), fx.ws.Distance(p), 1e-9) << "pair " << p;
    }
    EXPECT_LE(fx.ws.MinDistance(), fx.ws.Distance(0));
  }
}

TEST(CapsuleWorkspace, FingerprintMismatchRejected) {
  Fixture fx;
  fx.Bind(SpecYaml(1.0, false, kPairs, {}), false);
  pinocchio::Model other;
  pinocchio::urdf::buildModel(TestUrdfPath("serial_6dof.urdf"), other);
  pinocchio::Data other_data(other);
  pinocchio::computeJointJacobians(other, other_data, pinocchio::neutral(other));
  EXPECT_FALSE(fx.ws.UpdatePlacements(other, other_data));
  EXPECT_FALSE(fx.ws.EvaluateGradients(other, other_data));
  // 공 없는 workspace 의 SetBall 은 무시된다 (쌍 수 그대로).
  fx.ws.SetBall(kBallP0, kBallP1, kBallRadius);
  EXPECT_FALSE(fx.ws.HasBall());
  EXPECT_EQ(fx.ws.NumBallPairs(), 0U);
  rub::CapsuleWorkspace unbound;
  EXPECT_FALSE(unbound.UpdatePlacements(fx.model, fx.data));
  EXPECT_EQ(unbound.EvaluateDistances(), 0U);
  EXPECT_EQ(unbound.NumPairs(), 0U);
}

// ── FD 게이트 ───────────────────────────────────────────────────────────────

TEST(CapsuleWorkspace, GradientMatchesFiniteDifferenceSeparated) {
  Fixture fx;
  fx.Bind(SpecYaml(1.0, false, kPairs, {"l_upper", "l_fore", "l_hand"}), true);
  std::mt19937 rng(1);
  const FdStats st = CheckGradientsAgainstFd(fx, false, rng);
  PrintStats("separated", st);
  EXPECT_GE(st.separated, 1500) << "분리 구간 표본이 너무 적다 — fixture 를 확인";
}

TEST(CapsuleWorkspace, GradientMatchesFiniteDifferencePenetrating) {
  // 반지름 ×3 — 이웃 쌍이 침범 구간 [−(r1+r2)+1e-3, −1e-3] 에 자주 든다.
  Fixture fx;
  fx.Bind(SpecYaml(3.0, false, kPairs, {"l_upper", "l_fore", "l_hand"}), true);
  std::mt19937 rng(2);
  const FdStats st = CheckGradientsAgainstFd(fx, true, rng);
  PrintStats("penetrating", st);
  EXPECT_GE(st.penetrating, 100) << "침범 구간 표본이 너무 적다 — fixture 를 확인";
}

TEST(CapsuleWorkspace, DegenerateCapsuleBothOrders) {
  // 왼손을 길이 0 capsule (구) 로 — (구, capsule) 과 (capsule, 구) 두 순서에서 거리 · 기울기가
  // 같고 FD 를 통과한다.
  Fixture fwd;
  fwd.Bind(SpecYaml(1.0, true, {{"l_hand", "r_fore"}, {"l_hand", "torso"}}, {"l_hand"}), true);
  Fixture rev;
  rev.Bind(SpecYaml(1.0, true, {{"r_fore", "l_hand"}, {"torso", "l_hand"}}, {"l_hand"}), true);
  std::mt19937 rng(3);
  for (int s = 0; s < 50; ++s) {
    const Eigen::VectorXd q = RandomQ(fwd.model, rng);
    fwd.DistancesAndGradients(q);
    rev.DistancesAndGradients(q);
    for (std::size_t p = 0; p < 3; ++p) {
      EXPECT_NEAR(fwd.ws.Distance(p), rev.ws.Distance(p), 1e-12) << "pair " << p;
      EXPECT_LE((fwd.ws.Gradient(p) - rev.ws.Gradient(p)).cwiseAbs().maxCoeff(), 1e-9)
          << "pair " << p;
    }
  }
  std::mt19937 rng_fd(4);
  const FdStats st = CheckGradientsAgainstFd(rev, false, rng_fd);
  PrintStats("degenerate", st);
  EXPECT_GE(st.separated, 300);
}

// ── lever arm 상한 ──────────────────────────────────────────────────────────

TEST(CapsuleWorkspace, LeverArmBoundBoundsDistanceChange) {
  // |d(q + Δ) − d(q)| ≤ c_j ‖Δ‖∞ — c_j 가 정말 상한인지 (formulation §1.3 노드 사이 여유의 전제).
  Fixture fx;
  fx.Bind(SpecYaml(1.0, false, kPairs, {"l_upper", "l_fore", "l_hand"}), true);
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  constexpr double kStep = 0.02;
  double worst = 0.0;
  for (int s = 0; s < kSamples; ++s) {
    const Eigen::VectorXd q = RandomQ(fx.model, rng);
    Eigen::VectorXd dq(fx.model.nv);
    for (Eigen::Index i = 0; i < dq.size(); ++i) {
      dq[i] = kStep * u(rng);
    }
    fx.Distances(q);
    std::vector<double> d0(fx.ws.NumPairs());
    for (std::size_t p = 0; p < d0.size(); ++p) {
      d0[p] = fx.ws.Distance(p);
    }
    fx.Distances(q + dq);
    const double inf = dq.cwiseAbs().maxCoeff();
    for (std::size_t p = 0; p < d0.size(); ++p) {
      const double change = std::abs(fx.ws.Distance(p) - d0[p]);
      const double bound = fx.ws.Pair(p).lever_arm_bound * inf;
      worst = std::max(worst, change / bound);
      EXPECT_LE(change, bound + 1e-12) << "pair " << p;
    }
  }
  std::cout << "[lever arm] worst |Δd| / (c_j ‖Δq‖∞) = " << worst << "\n";
  EXPECT_GT(worst, 0.05) << "상한이 너무 느슨하면 여유가 과보수가 된다 — 기록";
}
