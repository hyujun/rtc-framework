// ── CapsuleWorkspace: 소유자별 capsule 거리 · 기울기 평가기 (pinocchio geometry + coal) ──
//
// 하나의 pinocchio Model 에 대해 CapsuleModel (spec) 로부터 자기 GeometryModel ·
// GeometryData · Jacobian 스크래치를 짓는다. 소비자 (MPC 코어 · CLIK damper) 마다 하나다 —
// GeometryData 의 distance functor 는 GeometryModel::geometryObjects 원소를 raw pointer 로
// 잡고 복사 생성자가 그 포인터를 그대로 복사하므로 (pinocchio 4.1 geometry.hxx) 이 타입은
// 복사하지 않는다. 이동은 된다 (GeometryModel · GeometryData 를 unique_ptr 로 쥔다).
//
// RT 계약: Bind 뒤의 UpdatePlacements · SetBall · EvaluateDistances · EvaluateGradients 는
// noexcept · 힙 할당 0 (test_capsule_workspace_alloc 이 C malloc 수준에서 고정). 거리는
// pinocchio::computeDistance 가 아니라 GeometryData::distance_functors 를 직접 부른다 —
// 전자는 PINOCCHIO_CHECK_INPUT_ARGUMENT 와 coal 예외의 재throw 경로를 갖는다.
//
// 기울기: coal 의 normal (o1 → o2, 침범이면 부호 포함) 과 최근접점의 점 Jacobian 으로
//   ∇_q d = normalᵀ (J_p2 − J_p1),  J_p = J_v − [r]× J_ω  (관절 Jacobian, LOCAL_WORLD_ALIGNED)
// — ocs2 SelfCollision · pink SelfCollisionBarrier 와 같은 식. 호출자가 그 Data 에
// computeJointJacobians (FK + data.J) 를 돌려 둔다. 두 축이 평행한 capsule 쌍에서는
// 최근접점이 유일하지 않아 기울기가 연속이 아니다 — formulation §1.3 의 알려진 한계.
#pragma once

#include "rtc_urdf_bridge/collision/capsule_model.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/multibody.hpp>
#pragma GCC diagnostic pop

#ifndef PINOCCHIO_WITH_COLLISION
// GeometryData 의 레이아웃이 이 매크로에 달려 있다 — 매크로 없는 TU 하나가 ODR 로 조용히
// 깨뜨린다. pinocchio::pinocchio 타깃이 INTERFACE 정의로 준다.
#error "capsule_workspace.hpp requires PINOCCHIO_WITH_COLLISION (link pinocchio::pinocchio)"
#endif

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace pinocchio {
struct GeometryModel;
struct GeometryData;
}  // namespace pinocchio

namespace rtc_urdf_bridge {

/// @brief Bind 의 결과. kOk 외에는 `BindError()` 가 이름을 넣은 메시지를 갖는다.
enum class CapsuleBindStatus : std::uint8_t {
  kOk = 0,
  kNoSpec,        ///< spec 이 null
  kAlreadyBound,  ///< 한 workspace 는 한 번만 Bind 한다
  kLinkMissing,   ///< spec 의 link 가 모델에 없다 (BODY frame)
  kSameParentJoint,  ///< 쌍의 두 capsule 이 같은 관절에 붙어 있다 (fixed 로 묶인 링크 — 기울기 0)
  kBothOnUniverse,   ///< 쌍의 두 capsule 이 모두 universe 에 붙어 있다
  kBallInSubtree,    ///< ball capsule 이 catch frame 의 support · subtree 에 든다
  kBallWithoutSpec,  ///< with_ball 인데 spec 의 ball_capsules 가 비어 있다
};

/// @brief 쌍 하나의 고정 정보 (Bind 에서 확정).
struct CapsulePairInfo {
  std::size_t capsule_a{0};  ///< spec capsules 인덱스. 공 쌍이면 쓰지 않는다 (is_ball)
  std::size_t capsule_b{0};  ///< spec capsules 인덱스 (팔 쪽)
  bool is_ball{false};       ///< 공 쌍: o1 = 공, o2 = capsule_b
  pinocchio::JointIndex joint_a{0};
  pinocchio::JointIndex joint_b{0};
  /// 노드 사이 여유의 lever arm 상한 [m/rad] — formulation §1.3 (#642 D5). 두 capsule 사이
  /// 경로 위의 관절 i 마다 (그 관절에서 capsule 끝점까지의 placement 평행이동 노름 합 + capsule
  /// 끝점 노름 + 반지름) 이고 (prismatic 은 1), 경로 밖의 관절은 0. 관절 i 가 Δq_i 돌 때 쌍의
  /// 거리가 변하는 양의 상한이 lever_arm[i]·|Δq_i| 다 — 보수적이다 (lever arm 이 전부 최대로
  /// 정렬된 경우). `lever_arm` 은 v 좌표 순서 (nv), `lever_arm_bound` 는 그 합 c_j (‖Δq‖∞ 에
  /// 곱하는 스칼라 — 관절 수가 많으면 매우 보수적이라 소비자는 벡터를 쓴다).
  Eigen::VectorXd lever_arm;
  double lever_arm_bound{0.0};
};

/// @brief capsule 거리 · 기울기 평가기. 사용 순서 (RT tick · 계획기 노드):
/// @code
///   // [non-RT]
///   CapsuleWorkspace ws;  ws.Bind(spec, model, {.with_ball = true, .catch_frame = fid});
///   // [RT · 노드마다]
///   pinocchio::computeJointJacobians(model, data, q);   // FK + data.J
///   ws.UpdatePlacements(model, data);                   // oMg
///   ws.SetBall(p0_world, p1_world, r_b);                // with_ball 일 때 — 구간 swept capsule
///   ws.EvaluateDistances();                             // d_j, 최근접점, normal
///   ws.EvaluateGradients(model, data);                  // ∇_q d_j  (data.J 필요)
/// @endcode
class CapsuleWorkspace {
 public:
  struct BindOptions {
    /// 공 형상 (joint 0 에 붙은 capsule) 과 spec.ball_capsules 의 쌍을 더한다. 자세 · 길이는
    /// SetBall 이 노드마다 쓴다. damper 는 false — 원점의 가짜 공을 피하지 않게.
    bool with_ball{false};
    /// 0 이 아니면 ball capsule 이 이 frame 의 support (조상 관절) 또는 subtree 에 붙어 있을 때
    /// 거부한다 — 포구 손이 공을 피하게 된다.
    pinocchio::FrameIndex catch_frame{0};
  };

  CapsuleWorkspace();
  ~CapsuleWorkspace();
  CapsuleWorkspace(const CapsuleWorkspace&) = delete;
  CapsuleWorkspace& operator=(const CapsuleWorkspace&) = delete;
  CapsuleWorkspace(CapsuleWorkspace&&) noexcept;
  CapsuleWorkspace& operator=(CapsuleWorkspace&&) noexcept;

  /// @brief [non-RT] spec 을 이 모델에 대해 짓는다. 모든 버퍼 사전 할당.
  /// 쌍 순서: spec.pairs 순 (자기충돌) 다음에 spec.ball_capsules 순 (공 쌍, with_ball 일 때).
  [[nodiscard]] CapsuleBindStatus Bind(std::shared_ptr<const CapsuleModel> spec,
                                       const pinocchio::Model& model, const BindOptions& options);

  /// @brief 마지막 Bind 실패의 메시지 (이름 포함). kOk 면 빈 문자열.
  [[nodiscard]] const std::string& BindError() const noexcept { return bind_error_; }

  [[nodiscard]] bool IsBound() const noexcept { return bound_; }

  // ── RT ──────────────────────────────────────────────────────────────────

  /// @brief data.oMi (FK 완료) 로 모든 로봇 capsule 의 world 자세를 갱신한다.
  /// @return false — model 이 Bind 한 모델과 다르다 (nv · njoints · nframes 지문). 그때는
  ///         아무것도 쓰지 않는다.
  [[nodiscard]] bool UpdatePlacements(const pinocchio::Model& model,
                                      const pinocchio::Data& data) noexcept;

  /// @brief 공 capsule 의 축 끝점 (world) 과 반지름. with_ball 이 아니면 무시. 길이 0 허용.
  void SetBall(const Eigen::Vector3d& p0_world, const Eigen::Vector3d& p1_world,
               double radius) noexcept;

  /// @brief 모든 쌍의 signed 거리 · 최근접점 · normal. UpdatePlacements (· SetBall) 뒤.
  /// @return 거리가 유한하지 않은 쌍의 수 (coal 이 실패한 경우 — 그 쌍은 `PairValid` 가 false).
  std::size_t EvaluateDistances() noexcept;

  /// @brief 모든 유효 쌍의 ∇_q d (nv). EvaluateDistances 뒤, data.J 는 같은 q 의 것.
  /// @return false — 지문 불일치 (아무것도 쓰지 않는다).
  [[nodiscard]] bool EvaluateGradients(const pinocchio::Model& model,
                                       const pinocchio::Data& data) noexcept;

  // ── 결과 ────────────────────────────────────────────────────────────────

  [[nodiscard]] std::size_t NumPairs() const noexcept { return pairs_.size(); }

  [[nodiscard]] std::size_t NumSelfPairs() const noexcept { return n_self_pairs_; }

  [[nodiscard]] std::size_t NumBallPairs() const noexcept { return pairs_.size() - n_self_pairs_; }

  [[nodiscard]] const CapsulePairInfo& Pair(std::size_t i) const noexcept { return pairs_[i]; }

  [[nodiscard]] bool HasBall() const noexcept { return with_ball_; }

  [[nodiscard]] int Nv() const noexcept { return nv_; }

  /// signed 거리 [m] (침범이면 음수). PairValid 가 false 면 NaN.
  [[nodiscard]] double Distance(std::size_t i) const noexcept { return distance_[i]; }

  [[nodiscard]] bool PairValid(std::size_t i) const noexcept { return valid_[i] != 0; }

  /// 최근접점 (world) — [0] 은 o1 (capsule_a 또는 공), [1] 은 o2 (capsule_b).
  [[nodiscard]] const Eigen::Vector3d& NearestPoint(std::size_t i, int side) const noexcept {
    return nearest_[2 * i + static_cast<std::size_t>(side)];
  }

  /// o1 → o2 단위 normal. d = normal · (p2 − p1).
  [[nodiscard]] const Eigen::Vector3d& Normal(std::size_t i) const noexcept { return normal_[i]; }

  /// ∇_q d_i (nv). EvaluateGradients 가 채운다. 무효 쌍은 0.
  [[nodiscard]] Eigen::Ref<const Eigen::VectorXd> Gradient(std::size_t i) const noexcept {
    return gradient_.col(static_cast<Eigen::Index>(i));
  }

  /// 유효 쌍 가운데 최소 거리. 유효 쌍이 없으면 +inf.
  [[nodiscard]] double MinDistance() const noexcept;

  [[nodiscard]] const std::shared_ptr<const CapsuleModel>& Spec() const noexcept { return spec_; }

 private:
  void Swap(CapsuleWorkspace& o) noexcept;

  struct Shapes;  // coal 형상 (헤더에 coal 을 노출하지 않는다)

  [[nodiscard]] bool FingerprintMatches(const pinocchio::Model& model) const noexcept;
  static void LeverArmBound(const pinocchio::Model& model, pinocchio::JointIndex joint_a,
                            pinocchio::JointIndex joint_b, double reach_a, double reach_b,
                            Eigen::VectorXd& per_v, double& sum);

  std::shared_ptr<const CapsuleModel> spec_;
  std::unique_ptr<pinocchio::GeometryModel> geom_model_;
  std::unique_ptr<pinocchio::GeometryData> geom_data_;
  std::unique_ptr<Shapes> shapes_;
  std::vector<CapsulePairInfo> pairs_;
  std::vector<pinocchio::GeomIndex> geom_of_capsule_;  ///< spec capsule → GeomIndex
  std::size_t n_robot_geoms_{0};
  pinocchio::GeomIndex ball_geom_{0};
  std::size_t n_self_pairs_{0};
  bool with_ball_{false};
  bool bound_{false};
  std::string bind_error_;

  // 지문
  int nv_{0};
  int njoints_{0};
  int nframes_{0};
  std::size_t tree_hash_{0};  ///< parents · idx_v · nv of every joint

  // 결과 버퍼 (Bind 에서 크기 고정)
  std::vector<double> distance_;
  std::vector<std::uint8_t> valid_;
  std::vector<Eigen::Vector3d> nearest_;  ///< 2 per pair
  std::vector<Eigen::Vector3d> normal_;
  Eigen::MatrixXd gradient_;  ///< nv × n_pairs

  // 스크래치
  Eigen::Matrix<double, 6, Eigen::Dynamic> J_a_;  ///< 6 × nv
  Eigen::Matrix<double, 6, Eigen::Dynamic> J_b_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> Jp_;  ///< 3 × nv
};

}  // namespace rtc_urdf_bridge
