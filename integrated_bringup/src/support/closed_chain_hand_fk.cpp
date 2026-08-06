// ── ClosedChainHandFk 구현 ───────────────────────────────────────────────────
#include "integrated_bringup/support/closed_chain_hand_fk.hpp"

#include "rtc_controller_interface/device_readability.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <utility>

namespace integrated_bringup {

namespace rub = rtc_urdf_bridge;

void ClosedChainHandFk::ResetWiring() noexcept {
  active_ = false;
  use_hand_root_ = false;
  hand_root_fid_ = 0;
  fingertip_fid_ = {};
  fingertip_active_ = {};
  fingertip_downstream_ = {};
  last_pose_valid_ = {};
  bridge_.clear();
  missing_joint_.clear();
  handle_.reset();
  borrowed_model_ = nullptr;
  // 계측 카운터도 배선과 함께 되돌린다 (PR #374 리뷰). owning 으로 M tick 돌린 래퍼를 borrowed 로
  // 다시 배선하면 "borrowed 는 절대 증가하지 않는다" 는 계약이 이전 배선의 잔량 때문에 깨져
  // 보인다 — 소비자(2→1 계측)는 현재 배선의 사영 횟수를 묻는 것이다.
  projection_count_ = 0;
}

HandFkWiringResult ClosedChainHandFk::ResolveFrames(const rub::RtClosedChainHandle& src,
                                                    std::span<const std::string> fingertip_links,
                                                    std::string_view hand_root_link) {
  // (a) hand-root 프레임 해석 (full model 기준). closed 핸들은 arm-base world 이므로 fingertip 을
  //     hand-root 상대로 표현해야 serial 합성(tcp_pose.act)과 정합한다. 해결 안 되면 비활성(#3).
  if (!hand_root_link.empty()) {
    hand_root_fid_ = src.GetFrameId(hand_root_link);
    use_hand_root_ = (hand_root_fid_ != 0);
  }
  if (!use_hand_root_) {
    return HandFkWiringResult::kInactiveNoHandRoot;  // serial 경로가 정합 (byte-for-byte)
  }

  // (b) fingertip 프레임 해석. 활성화되면 **모든 valid-frame fingertip** 을 서비스한다 —
  //     loop 하류는 loop-consistent, 비하류는 full-model FK(serial 등가). topology 판정은
  //     "적어도 하나의 fingertip 이 하류인가"(=보정이 필요한가)로 전체 활성화만 게이트한다 (#1).
  bool any_downstream = false;
  const std::size_t n_ft = std::min(fingertip_links.size(), kMaxFingertips);
  for (std::size_t f = 0; f < n_ft; ++f) {
    if (fingertip_links[f].empty()) {
      continue;
    }
    const pinocchio::FrameIndex fid = src.GetFrameId(fingertip_links[f]);
    if (fid == 0) {
      continue;  // full model 에 없는 프레임 — 서비스 불가 (serial 도 id==0 이면 inactive)
    }
    fingertip_fid_[f] = fid;
    fingertip_active_[f] = true;
    if (src.IsFrameDownstreamOfLoop(fid)) {
      fingertip_downstream_[f] = true;
      any_downstream = true;
    }
  }
  if (!any_downstream) {
    return HandFkWiringResult::kInactiveNoDownstream;  // serial 경로가 이미 정확 (byte-for-byte)
  }
  return HandFkWiringResult::kActive;
}

HandFkWiringResult ClosedChainHandFk::Configure(
    std::shared_ptr<const pinocchio::Model> model,
    std::vector<pinocchio::RigidConstraintModel> constraints,
    std::vector<pinocchio::JointIndex> actuated_joint_ids, Eigen::VectorXd q_seed,
    const std::vector<std::vector<std::string>>& device_joint_names,
    std::span<const std::string> fingertip_links, std::string_view hand_root_link,
    double closure_error_threshold) {
  ResetWiring();
  closure_error_threshold_ = closure_error_threshold;

  // closure 없음(plain URDF) → serial 경로 유지 (byte-for-byte).
  if (!model || constraints.empty()) {
    return HandFkWiringResult::kInactiveNoClosure;
  }

  // ill-posed closure(비단일-DoF 독립관절 / dep>m 등)면 RtClosedChainHandle 생성자가 throw 한다.
  // 컨트롤러 config abort 대신 graceful 하게 serial 로 떨어진다 (#2).
  try {
    handle_ = std::make_unique<rub::RtClosedChainHandle>(
        std::move(model), std::move(constraints), std::move(actuated_joint_ids), std::move(q_seed));
  } catch (const std::exception&) {
    handle_.reset();
    return HandFkWiringResult::kInactiveConstructionFailed;
  }
  if (const auto frames = ResolveFrames(*handle_, fingertip_links, hand_root_link);
      frames != HandFkWiringResult::kActive) {
    ResetWiring();
    return frames;
  }

  // (c) 독립 관절 이름 → (device, channel) 브릿지. 하나라도 못 찾으면 비활성.
  const std::vector<std::string> indep_names = handle_->GetIndependentJointNames();
  bridge_.reserve(indep_names.size());
  for (const auto& name : indep_names) {
    QSource src;
    bool found = false;
    for (std::size_t d = 0; d < device_joint_names.size() && !found; ++d) {
      const auto& names = device_joint_names[d];
      for (std::size_t c = 0; c < names.size(); ++c) {
        if (names[c] == name) {
          src.device = static_cast<int>(d);
          src.channel = static_cast<int>(c);
          found = true;
          break;
        }
      }
    }
    if (!found) {
      ResetWiring();  // 사유는 리셋 뒤에 다시 적는다 (ResetWiring 이 missing_joint_ 도 지운다).
      missing_joint_ = name;
      return HandFkWiringResult::kInactiveBridgeIncomplete;
    }
    bridge_.push_back(src);
  }

  q_a_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(bridge_.size()));
  active_ = true;
  return HandFkWiringResult::kActive;
}

HandFkWiringResult ClosedChainHandFk::ConfigureBorrowed(
    const rub::RtClosedChainHandle& projection, std::span<const std::string> fingertip_links,
    std::string_view hand_root_link, double closure_error_threshold) {
  ResetWiring();
  closure_error_threshold_ = closure_error_threshold;
  // 핸들을 만들지 않는다 — 사영은 남이 소유하고 이 래퍼는 읽기만 한다. q_a 브릿지도 필요 없다
  // (사영 입력은 빌려준 쪽이 채우므로, 이 래퍼가 device 에서 q 를 다시 모으면 그 사영과 다른
  // 입력을 보는 셈이 된다). 그래서 owning 이 (c) 에서 하는 일이 여기엔 없다.
  //
  // 참조도 남기지 않는다: 프레임 해석만 지금 하고, 매 tick 쓸 사영은 UpdateFromProjection 이
  // 받는다. 소유자가 재구성하면 저장해 둔 포인터는 dangling 이 되고 — 크래시가 아니라 placement
  // 가 조용히 쓰레기값이 되므로 pose 가 그럴듯하게 틀린다.
  borrowed_model_ = &projection.GetModel();

  if (const auto frames = ResolveFrames(projection, fingertip_links, hand_root_link);
      frames != HandFkWiringResult::kActive) {
    ResetWiring();
    return frames;
  }
  active_ = true;
  return HandFkWiringResult::kActive;
}

void ClosedChainHandFk::Update(const rtc::ControllerState& state) noexcept {
  if (!active_) {
    return;
  }
  // borrowed 모드가 이 진입점으로 새면 남의 사영을 이 래퍼의 q_a 로 덮어쓰려 시도하게 된다
  // (핸들이 없으므로 실제로는 null 역참조). Release 는 컴파일 아웃 — 아래 guard 가 남는다.
  assert(handle_ != nullptr && "borrowed 모드는 UpdateFromProjection 을 쓴다 (#175)");
  if (handle_ == nullptr) {
    return;
  }
  // 독립 관절 순서로 device 전반에서 측정 q 를 모은다. 소스 device 가 invalid 이거나 channel 이
  // 범위를 벗어나면 이 tick 은 신뢰 불가로 표시(#5).
  //
  // 판정은 base 의 `IsSlotFresh` 로 한다 — 이 세 조건(valid · 범위 · 이번 메시지에 써짐)을 여기서
  // 손으로 재구현한 것이 #284 를 놓친 경로였다. 그때 게이트에 접힌 세 번째 항(per-slot freshness)이
  // 이 루프에는 도달하지 못했고, 구멍 난 슬롯이 sources_ok 를 통과해 직전 값이 측정값으로 사영에
  // 들어갔다. 이 reader 가 필요로 하는 것은 prefix `[0, model_dim)` 가 아니라 특정 (device,
  // channel) 쌍이므로 `IsDeviceReadable` 이 아니라 per-slot 술어가 맞는 짝이다.
  bool sources_ok = true;
  for (std::size_t i = 0; i < bridge_.size(); ++i) {
    const QSource& s = bridge_[i];
    const auto d = static_cast<std::size_t>(s.device);
    const auto c = static_cast<std::size_t>(s.channel);
    double v = 0.0;
    if (d < static_cast<std::size_t>(state.num_devices) &&
        rtc::IsSlotFresh(state.devices[d], s.channel)) {
      v = state.devices[d].positions[c];
    } else {
      sources_ok = false;
    }
    q_a_[static_cast<Eigen::Index>(i)] = v;
  }
  // 소스가 하나라도 invalid 이면 handle_->Update 를 호출하지 않는다: 0-fill 된 조작 q_a 를 사영하면
  // handle 내부 warm-start seed(q_full_)가 그 관절=0 형상으로 commit
  // 되어(rt_closed_chain_handle.cpp:299), 소스 복구 tick 에서 K=2 사영이 큰 불연속을 못 닫아 여러
  // tick 동안 held 로 남는다. Update 를 건너뛰면 직전 loop-consistent seed 가 보존되어 복구 tick 이
  // 즉시 수렴한다.
  if (sources_ok) {
    // 반환 status 는 아래에서 GetStatus() 로 다시 읽는다 (nodiscard 무시 명시).
    static_cast<void>(handle_->Update(
        std::span<const double>(q_a_.data(), static_cast<std::size_t>(q_a_.size()))));
    ++projection_count_;  // wrapper 축 계측: 이 래퍼가 직접 돌린 사영 (#175)
  }

  // status 소비를 래퍼가 내부화(#4): 신뢰 가능한 tick 에서만 fingertip pose 캐시를 갱신하고,
  // 아니면(미수렴/특이/held/소스 이상) 직전 유효 pose 를 유지한다. sources_ok 가 false 면 아래
  // trustworthy 가 반드시 false 이므로, Update 를 건너뛴 stale status 를 읽어도 안전하다.
  ApplyProjection(*handle_, sources_ok, handle_->GetStatus());
}

void ClosedChainHandFk::UpdateFromProjection(
    const rub::RtClosedChainHandle& projection, bool projection_fresh,
    const rub::RtClosedChainHandle::Status& kin_status) noexcept {
  if (!active_) {
    return;
  }
  // 배선 때와 다른 모델의 사영이 오면 프레임 id 가 다른 것을 가리킨다 (id 는 모델 종속).
  assert(borrowed_model_ == &projection.GetModel() &&
         "빌린 사영이 배선 시점과 다른 모델 위에 있다 (#175)");
  if (borrowed_model_ != &projection.GetModel()) {
    return;  // 캐시 유지 — 엉뚱한 프레임을 읽느니 직전 pose 를 지킨다
  }
  // owning 모드가 이 진입점으로 새면 사영이 아무도 안 돌린 채 정책만 돌아 pose 가 조용히 stale
  // 이 된다 (조회는 성공하므로 증상이 "값이 안 변한다" 뿐이다). Release 는 컴파일 아웃.
  assert(handle_ == nullptr && "owning 모드는 Update(state) 를 쓴다 (#175)");
  // 사영은 이미 남이 돌렸다 — 이 래퍼는 정책만 적용한다. `projection_fresh` 의 두 축(사영이
  // 이번 tick 에 돌았는가 · 그 입력이 이번 tick 측정값인가)은 호출측이 AND 해서 넘긴다.
  ApplyProjection(projection, projection_fresh, kin_status);
}

void ClosedChainHandFk::ApplyProjection(const rub::RtClosedChainHandle& src, bool inputs_ok,
                                        const rub::RtClosedChainHandle::Status& st) noexcept {
  // 결과가 유한(입력 유효 && !held && finite closure)하면 사영 내부 data_ 는 현재 actuated q 를
  // 반영한다(사영은 passive 열만 이동, actuated 열은 측정값 고정). 비하류(serial 등가) fingertip 의
  // pose 는 actuated q 만의 함수라 이 조건만으로 유효하다 (#3).
  const bool finite_result = inputs_ok && !st.held && std::isfinite(st.closure_error);
  if (!finite_result) {
    return;  // 캐시(last_pose_) 유지 — 입력 이상/비유한
  }
  // loop 하류 fingertip 은 loop-consistency 까지 신뢰돼야 갱신 (미수렴/특이 tick 은 hold).
  const bool loop_trustworthy = !st.singular && st.closure_error < closure_error_threshold_;
  for (std::size_t f = 0; f < kMaxFingertips; ++f) {
    if (!fingertip_active_[f]) {
      continue;
    }
    if (fingertip_downstream_[f] && !loop_trustworthy) {
      continue;  // 하류 tip 은 loop-untrustworthy tick 에서 직전 유효 pose 유지
    }
    const pinocchio::SE3& tip = src.GetFramePlacement(fingertip_fid_[f]);
    last_pose_[f] = src.GetFramePlacement(hand_root_fid_).actInv(tip);  // hand-root 상대
    last_pose_valid_[f] = true;
  }
}

bool ClosedChainHandFk::GetFingertipHandRootPose(std::size_t f,
                                                 pinocchio::SE3& out) const noexcept {
  if (!active_ || f >= kMaxFingertips || !fingertip_active_[f] || !last_pose_valid_[f]) {
    return false;
  }
  out = last_pose_[f];  // 직전 신뢰 tick 값 (untrustworthy tick 은 hold)
  return true;
}

rub::RtClosedChainHandle::Status ClosedChainHandFk::status() const noexcept {
  // borrowed 모드는 사영을 들고 있지 않으므로 여기서 답할 수 없다 (소유자에게 물어야 한다).
  // owning 모드의 소비자만 이 접근자를 쓴다.
  if (!active_ || handle_ == nullptr) {
    return {};
  }
  return handle_->GetStatus();
}

// ── 컨트롤러 wiring 공용 dispatch (task/joint 공유) ───────────────────────────

bool RunHandForwardKinematics(ClosedChainHandFk& fk, rub::RtModelHandle* hand_handle,
                              Eigen::VectorXd& hand_q, const rtc::ControllerState& state) noexcept {
  if (hand_handle == nullptr) {
    return false;
  }
  // closed 경로: 독립 관절 소스가 device 1(hand)에 한정되지 않는다(arm+hand 스팬 가능). 따라서
  // device 1 valid 를 진입 게이트로 강제하지 않고, 각 소스 유효성은 ClosedChainHandFk::Update 가
  // tick 내에서 확인해 unfit tick 을 hold 로 내부 처리한다 (#8).
  if (fk.active()) {
    // borrowed 래퍼는 이 dispatch 로 구동할 수 없다 — 사영을 소유하지 않으므로 `Update(state)` 가
    // 즉시 반환하고(Release 는 assert 컴파일 아웃), 그런데도 true 를 돌리면 호출측은 FK 가 돈
    // 것으로 알고 **얼어붙은 pose 캐시를 유효한 TF 로 발행**한다. 증상이 "TF 가 안 움직인다" 뿐인
    // 조용한 stale 이므로, 여기서 withhold 로 떨어뜨려 배선 실수를 관측 가능하게 만든다.
    // (WBC 의 borrowed 경로는 이 dispatch 를 우회하고 UpdateFromProjection 을 직접 부른다 — 그쪽은
    // provenance 소스가 컨트롤러 전용이라 helper 로 접을 수 없다, #175.)
    if (!fk.owns_projection()) {
      return false;
    }
    fk.Update(state);  // measured actuated q → loop-consistent full FK (per-source hold 내부화)
    return true;
  }
  // serial 경로: device 1(hand) 측정값 필요. `valid` + channel 범위 검사(#4) 두 조건은
  // 합치면 정확히 F5 술어이므로 base 로 수렴시킨다 (#291) — 이 파일의 손수 구현이
  // device-1 축에서 저장소가 이미 갖고 있던 올바른 답이었고, 이제 팔 축과 같은 이름을
  // 쓴다. 동작은 그대로: num_channels < nq 이면 positions[] out-of-bounds/stale read
  // 대신 `return false` 로 직전 FK 를 유지한다 (withhold 계약, 침묵과는 다른 lane).
  const auto hand_nq = static_cast<std::size_t>(hand_handle->nq());
  if (state.num_devices <= 1 ||
      !rtc::IsDeviceReadable(state.devices[1], static_cast<int>(hand_nq))) {
    return false;
  }
  const auto& dev1 = state.devices[1];
  for (std::size_t i = 0; i < hand_nq; ++i) {
    hand_q[static_cast<Eigen::Index>(i)] = dev1.positions[i];
  }
  hand_handle->ComputeForwardKinematics(std::span<const double>(hand_q.data(), hand_nq));
  return true;
}

bool HandFingertipPoseDispatch(
    const ClosedChainHandFk& fk, const rub::RtModelHandle* hand_handle,
    const std::array<pinocchio::FrameIndex, ClosedChainHandFk::kMaxFingertips>& fingertip_ids,
    bool use_hand_root, pinocchio::FrameIndex hand_root_id, std::size_t f,
    pinocchio::SE3& out) noexcept {
  if (fk.active()) {
    return fk.GetFingertipHandRootPose(f, out);
  }
  if (hand_handle == nullptr || f >= fingertip_ids.size() || fingertip_ids[f] == 0) {
    return false;
  }
  out = hand_handle->GetFramePlacement(fingertip_ids[f]);
  if (use_hand_root) {
    out = hand_handle->GetFramePlacement(hand_root_id).actInv(out);
  }
  return true;
}

void LogHandFkWiring(const rclcpp::Logger& logger, const char* tag, HandFkWiringResult result,
                     const std::string& missing_joint) {
  switch (result) {
    case HandFkWiringResult::kActive:
      RCLCPP_INFO(logger, "%s closed-chain hand FK active (loop-consistent fingertip FK).", tag);
      break;
    case HandFkWiringResult::kInactiveBridgeIncomplete:
      RCLCPP_WARN(logger,
                  "%s loop closure present but actuated joint '%s' is not in any device "
                  "joint_state_names — closed-chain hand FK disabled (serial FK).",
                  tag, missing_joint.c_str());
      break;
    case HandFkWiringResult::kInactiveConstructionFailed:
      RCLCPP_WARN(logger,
                  "%s loop closure present but RtClosedChainHandle construction failed (ill-posed "
                  "closure) — closed-chain hand FK disabled (serial FK).",
                  tag);
      break;
    case HandFkWiringResult::kInactiveNoHandRoot:
      RCLCPP_WARN(logger,
                  "%s loop closure present but hand-root frame did not resolve on the full model — "
                  "closed-chain hand FK disabled (serial FK).",
                  tag);
      break;
    case HandFkWiringResult::kInactiveNoDownstream:
      RCLCPP_INFO(logger,
                  "%s loop closure present but no fingertip is downstream of a loop-passive "
                  "joint — serial hand FK (byte-for-byte).",
                  tag);
      break;
    case HandFkWiringResult::kInactiveNoClosure:
      break;  // plain URDF (no closure) — serial path, no log
  }
}

}  // namespace integrated_bringup
