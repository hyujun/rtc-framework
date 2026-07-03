// ── ClosedChainHandFk 구현 ───────────────────────────────────────────────────
#include "integrated_bringup/support/closed_chain_hand_fk.hpp"

#include <algorithm>
#include <utility>

namespace integrated_bringup {

namespace rub = rtc_urdf_bridge;

HandFkWiringResult ClosedChainHandFk::Configure(
    std::shared_ptr<const pinocchio::Model> model,
    std::vector<pinocchio::RigidConstraintModel> constraints,
    std::vector<pinocchio::JointIndex> actuated_joint_ids, Eigen::VectorXd q_seed,
    const std::vector<std::vector<std::string>>& device_joint_names,
    std::span<const std::string> fingertip_links, std::string_view hand_root_link) {
  active_ = false;
  use_hand_root_ = false;
  hand_root_fid_ = 0;
  fingertip_fid_ = {};
  fingertip_active_ = {};
  bridge_.clear();
  missing_joint_.clear();
  handle_.reset();

  // closure 없음(plain URDF) → serial 경로 유지 (byte-for-byte).
  if (!model || constraints.empty()) {
    return HandFkWiringResult::kInactiveNoClosure;
  }

  handle_ = std::make_unique<rub::RtClosedChainHandle>(
      std::move(model), std::move(constraints), std::move(actuated_joint_ids), std::move(q_seed));

  // (a) hand-root + fingertip 프레임 해석 (full model 기준).
  if (!hand_root_link.empty()) {
    hand_root_fid_ = handle_->GetFrameId(hand_root_link);
    use_hand_root_ = (hand_root_fid_ != 0);
  }
  bool any_downstream = false;
  const std::size_t n_ft = std::min(fingertip_links.size(), kMaxFingertips);
  for (std::size_t f = 0; f < n_ft; ++f) {
    if (fingertip_links[f].empty()) {
      continue;
    }
    const pinocchio::FrameIndex fid = handle_->GetFrameId(fingertip_links[f]);
    // topology-driven: loop-passive 하류 fingertip 만 closed-chain 보정이 의미 있다.
    if (fid != 0 && handle_->IsFrameDownstreamOfLoop(fid)) {
      fingertip_fid_[f] = fid;
      fingertip_active_[f] = true;
      any_downstream = true;
    }
  }
  if (!any_downstream) {
    handle_.reset();
    return HandFkWiringResult::kInactiveNoDownstream;  // serial 경로가 이미 정확 (byte-for-byte)
  }

  // (b) 독립 관절 이름 → (device, channel) 브릿지. 하나라도 못 찾으면 비활성.
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
      missing_joint_ = name;
      handle_.reset();
      bridge_.clear();
      return HandFkWiringResult::kInactiveBridgeIncomplete;
    }
    bridge_.push_back(src);
  }

  q_a_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(bridge_.size()));
  active_ = true;
  return HandFkWiringResult::kActive;
}

void ClosedChainHandFk::Update(const rtc::ControllerState& state) noexcept {
  if (!active_) {
    return;
  }
  // 독립 관절 순서로 device 전반에서 측정 q 를 모은다.
  for (std::size_t i = 0; i < bridge_.size(); ++i) {
    const QSource& s = bridge_[i];
    const auto d = static_cast<std::size_t>(s.device);
    const auto c = static_cast<std::size_t>(s.channel);
    double v = 0.0;
    if (d < static_cast<std::size_t>(state.num_devices) && c < state.devices[d].positions.size()) {
      v = state.devices[d].positions[c];
    }
    q_a_[static_cast<Eigen::Index>(i)] = v;
  }
  handle_->Update(std::span<const double>(q_a_.data(), static_cast<std::size_t>(q_a_.size())));
}

bool ClosedChainHandFk::GetFingertipHandRootPose(std::size_t f,
                                                 pinocchio::SE3& out) const noexcept {
  if (!active_ || f >= kMaxFingertips || !fingertip_active_[f]) {
    return false;
  }
  const pinocchio::SE3& tip = handle_->GetFramePlacement(fingertip_fid_[f]);
  if (use_hand_root_) {
    out = handle_->GetFramePlacement(hand_root_fid_).actInv(tip);
  } else {
    out = tip;
  }
  return true;
}

rub::RtClosedChainHandle::Status ClosedChainHandFk::status() const noexcept {
  if (!active_) {
    return {};
  }
  return handle_->GetStatus();
}

}  // namespace integrated_bringup
