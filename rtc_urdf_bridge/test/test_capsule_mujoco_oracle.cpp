// ── test_capsule_mujoco_oracle — 거리를 MuJoCo mj_geomDistance 와 대조 ──────────
//
// fixture 는 rtc_tools/rtc_tools/validation/gen_capsule_oracle.py 가 기록했다 (seed 642).
// 두 질문을 나눠 묻는다:
//   (1) 형상 거리 — MuJoCo 가 컴파일한 geom 의 world 자세 · size 로 같은 capsule 을 지어
//       |Δd| ≤ 1e-9. MJCF size 는 (반지름, **반**길이) 이고 coal Capsule(r, lz) 는 전체 길이다
//       — 이 변환 (spec 의 끝점 → coal) 이 여기서 고정된다. 길이 0 행은 MuJoCo 쪽이 sphere.
//   (2) 모델 배치 경로 — 실제 로봇 MJCF 를 pinocchio 로 읽고 같은 geom 을 BODY frame 의
//       spec capsule 로 붙여, 무작위 q 에서 FK → oMg → 거리가 MuJoCo 와 |Δd| ≤ 1e-8.
//       모델 파일은 robot_descriptions 의 share 에서 런타임에 찾는다 (ARCH-5) — 없으면 skip
//       하고 그 사실을 출력에 남긴다.
#include "rtc_urdf_bridge/collision/capsule_workspace.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#pragma GCC diagnostic pop

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace rub = rtc_urdf_bridge;

namespace {

std::filesystem::path DataPath(const std::string& name) {
  return std::filesystem::path(__FILE__).parent_path() / "data" / name;
}

std::vector<std::vector<std::string>> ReadCsv(const std::filesystem::path& path) {
  std::ifstream in(path);
  EXPECT_TRUE(in) << path;
  std::vector<std::vector<std::string>> rows;
  std::string line;
  bool header = true;
  while (std::getline(in, line)) {
    if (header) {
      header = false;
      continue;
    }
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      cells.push_back(cell);
    }
    rows.push_back(cells);
  }
  return rows;
}

std::string Num(double v) {
  std::ostringstream ss;
  ss.precision(17);
  ss << v;
  return ss.str();
}

// MuJoCo geom (pos, quat wxyz, r, 반길이) → spec capsule 한 줄 (링크 좌표계).
std::string CapsuleLine(const std::string& name, const std::string& link,
                        const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat, double r,
                        double half) {
  const Eigen::Vector3d axis = quat.toRotationMatrix().col(2) * half;
  const Eigen::Vector3d p0 = pos - axis;
  const Eigen::Vector3d p1 = pos + axis;
  return "    - {name: " + name + ", link: " + link + ", p0: [" + Num(p0.x()) + ", " + Num(p0.y()) +
         ", " + Num(p0.z()) + "], p1: [" + Num(p1.x()) + ", " + Num(p1.y()) + ", " + Num(p1.z()) +
         "], radius: " + Num(r) + "}\n";
}

}  // namespace

TEST(CapsuleMujocoOracle, ShapeDistanceMatchesMujoco) {
  const auto rows = ReadCsv(DataPath("capsule_oracle_shapes.csv"));
  ASSERT_GE(rows.size(), 100U);

  // 관절 둘 (각자 universe 의 자식, 항등 배치, q = 0) — 두 링크 좌표계가 world 와 같다.
  pinocchio::Model model;
  for (int i = 1; i <= 2; ++i) {
    const pinocchio::JointIndex j = model.addJoint(
        0, pinocchio::JointModelRX(), pinocchio::SE3::Identity(), "j" + std::to_string(i));
    model.addJointFrame(j);
    model.appendBodyToJoint(j, pinocchio::Inertia::Identity(), pinocchio::SE3::Identity());
    model.addBodyFrame("b" + std::to_string(i), j);
  }
  pinocchio::Data data(model);
  pinocchio::forwardKinematics(model, data, pinocchio::neutral(model));

  int spheres = 0;
  int penetrating = 0;
  double worst = 0.0;
  for (std::size_t r = 0; r < rows.size(); ++r) {
    const auto& c = rows[r];
    ASSERT_EQ(c.size(), 19U);
    std::string yaml = "collision_capsules:\n  capsules:\n";
    for (int i = 0; i < 2; ++i) {
      const std::size_t o = static_cast<std::size_t>(9 * i);
      const double radius = std::stod(c[o]);
      const double half = std::stod(c[o + 1]);
      const Eigen::Vector3d pos(std::stod(c[o + 2]), std::stod(c[o + 3]), std::stod(c[o + 4]));
      const Eigen::Quaterniond quat(std::stod(c[o + 5]), std::stod(c[o + 6]), std::stod(c[o + 7]),
                                    std::stod(c[o + 8]));
      if (half == 0.0) {
        ++spheres;
      }
      yaml += CapsuleLine("c" + std::to_string(i + 1), "b" + std::to_string(i + 1), pos, quat,
                          radius, half);
    }
    yaml += "  pairs: [[c1, c2]]\n";
    const double expected = std::stod(c[18]);
    if (expected < 0.0) {
      ++penetrating;
    }
    rub::CapsuleWorkspace ws;
    ASSERT_EQ(ws.Bind(rub::CapsuleModel::ParseYaml(yaml, "shape"), model, {}),
              rub::CapsuleBindStatus::kOk)
        << ws.BindError();
    ASSERT_TRUE(ws.UpdatePlacements(model, data));
    ASSERT_EQ(ws.EvaluateDistances(), 0U);
    const double err = std::abs(ws.Distance(0) - expected);
    worst = std::max(worst, err);
    EXPECT_LE(err, 1e-9) << "row " << r << " ours=" << ws.Distance(0) << " mujoco=" << expected;
  }
  std::cout << "[shape oracle] rows=" << rows.size() << " spheres=" << spheres
            << " penetrating=" << penetrating << " worst |Δd|=" << worst << "\n";
  EXPECT_GT(spheres, 0);
  EXPECT_GT(penetrating, 0);
}

TEST(CapsuleMujocoOracle, ModelPlacementPathMatchesMujoco) {
  std::filesystem::path mjcf;
  try {
    mjcf =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_descriptions")) /
        "robots/ur5e/mjcf/ur5e.xml";
  } catch (const std::exception& e) {
    GTEST_SKIP() << "robot_descriptions share 없음 — 배치 경로 오라클 skip: " << e.what();
  }
  if (!std::filesystem::exists(mjcf)) {
    GTEST_SKIP() << "MJCF 없음 — 배치 경로 오라클 skip: " << mjcf;
  }
  pinocchio::Model model;
  pinocchio::mjcf::buildModel(mjcf.string(), model);
  pinocchio::Data data(model);

  const auto geoms = ReadCsv(DataPath("capsule_oracle_ur5e_geoms.csv"));
  const auto dists = ReadCsv(DataPath("capsule_oracle_ur5e_dist.csv"));
  ASSERT_FALSE(geoms.empty());
  ASSERT_FALSE(dists.empty());
  ASSERT_EQ(static_cast<int>(dists[0].size()) - 4, model.nq) << "fixture 의 q 차원 ≠ 모델 nq";

  std::string yaml = "collision_capsules:\n  capsules:\n";
  for (const auto& g : geoms) {
    ASSERT_EQ(g.size(), 11U);
    const std::string body = g[1];
    ASSERT_TRUE(model.existFrame(body, pinocchio::BODY)) << body;
    const Eigen::Vector3d pos(std::stod(g[2]), std::stod(g[3]), std::stod(g[4]));
    const Eigen::Quaterniond quat(std::stod(g[5]), std::stod(g[6]), std::stod(g[7]),
                                  std::stod(g[8]));
    yaml += CapsuleLine("g" + g[0], body, pos, quat, std::stod(g[9]), std::stod(g[10]));
  }
  std::set<std::pair<std::string, std::string>> pairs;
  for (const auto& d : dists) {
    pairs.emplace(d[1], d[2]);
  }
  yaml += "  pairs:\n";
  std::map<std::pair<std::string, std::string>, std::size_t> pair_index;
  for (const auto& p : pairs) {
    pair_index[p] = pair_index.size();
    yaml += "    - [g" + p.first + ", g" + p.second + "]\n";
  }
  rub::CapsuleWorkspace ws;
  ASSERT_EQ(ws.Bind(rub::CapsuleModel::ParseYaml(yaml, "ur5e"), model, {}),
            rub::CapsuleBindStatus::kOk)
      << ws.BindError();

  int current_sample = -1;
  double worst = 0.0;
  for (const auto& d : dists) {
    const int sample = std::stoi(d[0]);
    if (sample != current_sample) {
      Eigen::VectorXd q(model.nq);
      for (Eigen::Index i = 0; i < q.size(); ++i) {
        q[i] = std::stod(d[static_cast<std::size_t>(4 + i)]);
      }
      pinocchio::forwardKinematics(model, data, q);
      ASSERT_TRUE(ws.UpdatePlacements(model, data));
      ASSERT_EQ(ws.EvaluateDistances(), 0U);
      current_sample = sample;
    }
    const std::size_t p = pair_index.at({d[1], d[2]});
    const double expected = std::stod(d[3]);
    const double err = std::abs(ws.Distance(p) - expected);
    worst = std::max(worst, err);
    EXPECT_LE(err, 1e-8) << "sample " << sample << " pair (" << d[1] << ", " << d[2]
                         << ") ours=" << ws.Distance(p) << " mujoco=" << expected;
  }
  std::cout << "[placement oracle] samples=" << current_sample + 1 << " pairs=" << pairs.size()
            << " rows=" << dists.size() << " worst |Δd|=" << worst << "\n";
}
