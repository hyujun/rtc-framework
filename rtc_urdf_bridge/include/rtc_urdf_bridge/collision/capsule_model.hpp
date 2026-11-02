// ── CapsuleModel: 충돌 capsule 기하의 불변 spec (YAML → 검증된 목록) ──────────
//
// 로봇의 링크에 붙은 capsule 들과 거리를 잴 쌍의 목록이다. 이 타입은 pinocchio 모델을
// 모른다 — 링크 이름의 존재, 같은 관절에 붙은 쌍 같은 모델 의존 검사는
// CapsuleWorkspace::Bind 가 한다. 소비자 (MPC 코어, CLIK damper) 는 같은 spec 을
// `shared_ptr<const CapsuleModel>` 로 공유하고 각자 CapsuleWorkspace 를 짓는다 — pinocchio
// GeometryData 는 복사하면 GeometryModel 의 원소를 가리키는 raw pointer 가 따라와
// dangling 이 되므로 (geometry-object.hxx ComputeDistance), 공유 대상은 spec 뿐이다.
//
// 형상은 capsule 하나다. 구는 길이 0 capsule (p0 == p1) 로 적는다 — coal 의
// capsule–capsule 닫힌식이 퇴화 선분을 그대로 다룬다 (test_capsule_workspace 가 고정).
#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rtc_urdf_bridge {

/// @brief 링크 하나에 붙은 capsule. 끝점은 그 링크 (URDF link = pinocchio BODY frame) 좌표계.
struct CapsuleSpec {
  std::string name;
  std::string link;
  Eigen::Vector3d p0{Eigen::Vector3d::Zero()};  ///< 축 끝점 [m]
  Eigen::Vector3d p1{Eigen::Vector3d::Zero()};  ///< 축 끝점 [m] (p0 == p1 이면 구)
  double radius{0.0};                           ///< [m], ≥ 0
};

/// @brief 거리를 잴 capsule 쌍 (이름). 순서는 뜻이 없다 — (a, b) 와 (b, a) 는 같은 쌍이다.
struct CapsulePairSpec {
  std::string a;
  std::string b;
};

/// @brief YAML `collision_capsules:` 의 내용.
struct CapsuleModelSpec {
  std::vector<CapsuleSpec> capsules;
  std::vector<CapsulePairSpec> pairs;
  /// 공 (움직이는 외부 형상) 과 거리를 잴 capsule 들. 공 형상은 MPC 쪽 workspace 만 갖는다
  /// (CapsuleWorkspace::BindOptions::with_ball) — damper 는 이 목록을 읽지 않는다.
  std::vector<std::string> ball_capsules;
};

/// 파서의 용량 상한 — 소비자의 고정 버퍼 (MPC 행 · damper 행) 가 이 수로 잡힌다.
inline constexpr std::size_t kMaxCapsules = 64;
inline constexpr std::size_t kMaxCapsulePairs = 128;
inline constexpr std::size_t kMaxBallCapsules = 16;

/// @brief 검증된 불변 capsule spec. 생성 뒤에는 바뀌지 않는다.
///
/// 생성자와 로더는 모델 없이 판정할 수 있는 규칙을 전부 **이름을 넣은** `std::runtime_error`
/// 로 거부한다: 미지 키 · 필수 필드 누락 · 음수 · 비유한 값 · 중복 capsule 이름 · 미등록
/// 이름의 쌍 · 같은 capsule 의 쌍 · 중복 쌍 (순서 무관) · 미등록 · 중복 `ball_capsules` ·
/// 용량 초과 · 지원하지 않는 `shape`. 모델이 필요한 규칙 (링크 존재 · 같은 parentJoint ·
/// 둘 다 universe · catch frame subtree 의 ball capsule) 은 CapsuleWorkspace::Bind.
class CapsuleModel {
 public:
  /// @throws std::runtime_error 위의 규칙 위반 (메시지에 capsule · 쌍 이름).
  explicit CapsuleModel(CapsuleModelSpec spec);

  /// @brief YAML 파일 → CapsuleModel. 최상위 키는 `collision_capsules` 하나여야 한다.
  /// @throws std::runtime_error 파일 없음 · 파싱 실패 · 위의 규칙 위반.
  [[nodiscard]] static std::shared_ptr<const CapsuleModel> LoadYaml(std::string_view yaml_path);

  /// @brief YAML 텍스트 → CapsuleModel (테스트 · in-memory 용).
  /// @param source_label 에러 메시지에 표시할 출처.
  /// @throws std::runtime_error 파싱 실패 · 위의 규칙 위반.
  [[nodiscard]] static std::shared_ptr<const CapsuleModel> ParseYaml(
      const std::string& yaml_text, std::string_view source_label = "<string>");

  [[nodiscard]] const CapsuleModelSpec& Spec() const noexcept { return spec_; }

  [[nodiscard]] std::size_t NumCapsules() const noexcept { return spec_.capsules.size(); }

  [[nodiscard]] std::size_t NumPairs() const noexcept { return spec_.pairs.size(); }

  [[nodiscard]] std::size_t NumBallCapsules() const noexcept { return spec_.ball_capsules.size(); }

  /// @brief 이름 → capsules 의 인덱스. 없으면 nullopt.
  [[nodiscard]] std::optional<std::size_t> FindCapsule(std::string_view name) const noexcept;

  /// @brief 쌍 i 의 두 capsule 인덱스 (spec().pairs[i] 의 a, b 순서).
  [[nodiscard]] std::pair<std::size_t, std::size_t> PairIndices(std::size_t i) const noexcept {
    return pair_indices_[i];
  }

  /// @brief ball_capsules[i] 의 capsule 인덱스.
  [[nodiscard]] std::size_t BallCapsuleIndex(std::size_t i) const noexcept {
    return ball_indices_[i];
  }

 private:
  CapsuleModelSpec spec_;
  std::vector<std::pair<std::size_t, std::size_t>> pair_indices_;
  std::vector<std::size_t> ball_indices_;
};

}  // namespace rtc_urdf_bridge
