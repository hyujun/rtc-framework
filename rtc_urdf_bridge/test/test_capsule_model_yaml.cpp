// ── test_capsule_model_yaml — CapsuleModel 파서의 수용과 거부 (이름으로) ─────────
//
// 거부 규칙 하나마다 "거부된다" 가 아니라 "그 이름 · 키를 말하며 거부된다" 를 고정한다 —
// 철자 오류가 안전 행을 조용히 끄지 않게 (#642 D12) 하는 것이 이 파서의 역할이라, 메시지가
// 어느 항목인지 말하지 않으면 configure 로그에서 고칠 수 없다.
#include "rtc_urdf_bridge/collision/capsule_model.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace rub = rtc_urdf_bridge;

namespace {

const char* const kValid = R"(
collision_capsules:
  capsules:
    - {name: torso, link: torso_link, p0: [0, 0, 0], p1: [0, 0, 0.25], radius: 0.08}
    - {name: l_fore, link: left_elbow_link, p0: [0, 0, 0], p1: [0.1, 0, 0], radius: 0.035}
    - {name: r_fore, link: right_elbow_link, p0: [0, 0, 0], p1: [0.11, 0, 0], radius: 0.035, shape: capsule}
    - {name: l_tip, link: left_tool, p0: [0, 0, 0], p1: [0, 0, 0], radius: 0.02}
  pairs:
    - [l_fore, r_fore]
    - [l_fore, torso]
    - [torso, r_fore]
  ball_capsules: [l_fore, l_tip]
)";

// 실패 메시지가 needle 을 담는지까지 본다.
void ExpectRejects(const std::string& yaml, const std::string& needle) {
  try {
    static_cast<void>(rub::CapsuleModel::ParseYaml(yaml, "test"));
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find(needle), std::string::npos)
        << "메시지에 '" << needle << "' 가 없다: " << e.what();
    return;
  }
  FAIL() << "거부되지 않았다 (기대 메시지 조각: '" << needle << "')";
}

}  // namespace

TEST(CapsuleModelYaml, AcceptsValidSpec) {
  const auto m = rub::CapsuleModel::ParseYaml(kValid, "valid");
  ASSERT_EQ(m->NumCapsules(), 4U);
  ASSERT_EQ(m->NumPairs(), 3U);
  ASSERT_EQ(m->NumBallCapsules(), 2U);
  EXPECT_EQ(m->FindCapsule("torso"), 0U);
  EXPECT_EQ(m->FindCapsule("l_tip"), 3U);
  EXPECT_FALSE(m->FindCapsule("nope").has_value());
  EXPECT_EQ(m->PairIndices(2), std::make_pair(std::size_t{0}, std::size_t{2}));
  EXPECT_EQ(m->BallCapsuleIndex(1), 3U);
  EXPECT_DOUBLE_EQ(m->Spec().capsules[0].p1.z(), 0.25);
  EXPECT_DOUBLE_EQ(m->Spec().capsules[2].radius, 0.035);
}

TEST(CapsuleModelYaml, PairsAndBallsAreOptional) {
  const auto m = rub::CapsuleModel::ParseYaml(R"(
collision_capsules:
  capsules:
    - {name: a, link: la, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1}
)");
  EXPECT_EQ(m->NumPairs(), 0U);
  EXPECT_EQ(m->NumBallCapsules(), 0U);
}

TEST(CapsuleModelYaml, RejectsUnknownKeys) {
  ExpectRejects("collision_capsule: {}", "collision_capsule");  // 최상위 철자 오류
  ExpectRejects(R"(
collision_capsules:
  capsules: []
  pair: [])",
                "pair");
  ExpectRejects(R"(
collision_capsules:
  capsules:
    - {name: a, link: la, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1, radiuss: 0.2})",
                "radiuss");
}

TEST(CapsuleModelYaml, RejectsMissingFields) {
  ExpectRejects(R"(
collision_capsules:
  capsules:
    - {link: la, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1})",
                "name");
  ExpectRejects(R"(
collision_capsules:
  capsules:
    - {name: a, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1})",
                "link");
  ExpectRejects(R"(
collision_capsules:
  capsules:
    - {name: a, link: la, p1: [0, 0, 1], radius: 0.1})",
                "p0");
  ExpectRejects(R"(
collision_capsules:
  capsules:
    - {name: a, link: la, p0: [0, 0, 0], p1: [0, 0, 1]})",
                "radius");
  ExpectRejects(R"(
collision_capsules:
  capsules:
    - {name: a, link: la, p0: [0, 0], p1: [0, 0, 1], radius: 0.1})",
                "p0");
  ExpectRejects("collision_capsules: {capsules: []}", "capsules");
}

TEST(CapsuleModelYaml, RejectsBadNumbers) {
  ExpectRejects(R"(
collision_capsules:
  capsules:
    - {name: neg, link: la, p0: [0, 0, 0], p1: [0, 0, 1], radius: -0.1})",
                "neg");
  ExpectRejects(R"(
collision_capsules:
  capsules:
    - {name: nan_r, link: la, p0: [0, 0, 0], p1: [0, 0, 1], radius: .nan})",
                "nan_r");
  ExpectRejects(R"(
collision_capsules:
  capsules:
    - {name: inf_p, link: la, p0: [0, 0, .inf], p1: [0, 0, 1], radius: 0.1})",
                "inf_p");
  ExpectRejects(R"(
collision_capsules:
  capsules:
    - {name: str_r, link: la, p0: [0, 0, 0], p1: [0, 0, 1], radius: big})",
                "str_r");
}

TEST(CapsuleModelYaml, RejectsUnsupportedShape) {
  ExpectRejects(R"(
collision_capsules:
  capsules:
    - {name: boxy, link: la, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1, shape: box})",
                "boxy");
}

TEST(CapsuleModelYaml, RejectsDuplicateCapsuleName) {
  ExpectRejects(R"(
collision_capsules:
  capsules:
    - {name: dup, link: la, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1}
    - {name: dup, link: lb, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1})",
                "dup");
}

TEST(CapsuleModelYaml, RejectsBadPairs) {
  const std::string two = R"(
collision_capsules:
  capsules:
    - {name: a, link: la, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1}
    - {name: b, link: lb, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1}
)";
  ExpectRejects(two + "  pairs: [[a, ghost]]", "ghost");
  ExpectRejects(two + "  pairs: [[a, a]]", "같은 capsule");
  ExpectRejects(two + "  pairs: [[a, b], [b, a]]", "중복 쌍");
  ExpectRejects(two + "  pairs: [[a, b, a]]", "pairs[0]");
  ExpectRejects(two + "  pairs: [a]", "pairs[0]");
}

TEST(CapsuleModelYaml, RejectsBadBallCapsules) {
  const std::string two = R"(
collision_capsules:
  capsules:
    - {name: a, link: la, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1}
    - {name: b, link: lb, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1}
)";
  ExpectRejects(two + "  ball_capsules: [phantom]", "phantom");
  ExpectRejects(two + "  ball_capsules: [a, a]", "중복");
  ExpectRejects(two + "  ball_capsules: a", "ball_capsules");
}

TEST(CapsuleModelYaml, RejectsOverCapacity) {
  std::string yaml = "collision_capsules:\n  capsules:\n";
  for (std::size_t i = 0; i <= rub::kMaxCapsules; ++i) {
    yaml += "    - {name: c" + std::to_string(i) +
            ", link: l, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1}\n";
  }
  ExpectRejects(yaml, "상한");

  std::string pairs = "collision_capsules:\n  capsules:\n";
  const std::size_t n = 20;  // 20C2 = 190 > kMaxCapsulePairs
  for (std::size_t i = 0; i < n; ++i) {
    pairs += "    - {name: c" + std::to_string(i) +
             ", link: l, p0: [0, 0, 0], p1: [0, 0, 1], radius: 0.1}\n";
  }
  pairs += "  pairs:\n";
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = i + 1; j < n; ++j) {
      pairs += "    - [c" + std::to_string(i) + ", c" + std::to_string(j) + "]\n";
    }
  }
  ExpectRejects(pairs, "상한");
}

TEST(CapsuleModelYaml, RejectsMalformedYaml) {
  ExpectRejects("collision_capsules: [", "파싱 실패");
  ExpectRejects("- not a map", "최상위");
}

TEST(CapsuleModelYaml, LoadYamlReportsMissingFile) {
  EXPECT_THROW(static_cast<void>(rub::CapsuleModel::LoadYaml("/nonexistent/collision.yaml")),
               std::runtime_error);
}
