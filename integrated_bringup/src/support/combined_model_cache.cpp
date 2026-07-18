// ── CombinedModelCache 구현 (#174) ──
#include "integrated_bringup/support/combined_model_cache.hpp"

#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_base/types/types.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <exception>
#include <utility>

namespace integrated_bringup {

bool CombinedModelCache::InitModel(rtc_urdf_bridge::PinocchioModelBuilder& builder,
                                   const std::vector<pinocchio::FrameIndex>& contact_frame_ids,
                                   std::string_view log_prefix, const rclcpp::Logger& logger) {
  // Model selection mirrors the former per-controller InitControlModelCache:
  // actuated closed-chain → reduced tree "wbc" → raw full URDF model.
  if (auto actuated = builder.GetActuatedModel()) {
    full_model_ptr_ = std::move(actuated);
    RCLCPP_INFO(logger, "%.*s control model: actuated closed-chain (nq=%d nv=%d)",
                static_cast<int>(log_prefix.size()), log_prefix.data(), full_model_ptr_->nq,
                full_model_ptr_->nv);
  } else {
    try {
      full_model_ptr_ = builder.GetTreeModel("wbc");
      RCLCPP_INFO(logger, "%.*s control model: reduced tree 'wbc' (nq=%d nv=%d)",
                  static_cast<int>(log_prefix.size()), log_prefix.data(), full_model_ptr_->nq,
                  full_model_ptr_->nv);
    } catch (const std::exception& e) {
      full_model_ptr_ = builder.GetFullModel();
      RCLCPP_INFO(logger,
                  "%.*s control model: URDF full model (nq=%d nv=%d) — tree 'wbc' missing (%s)",
                  static_cast<int>(log_prefix.size()), log_prefix.data(), full_model_ptr_->nq,
                  full_model_ptr_->nv, e.what());
    }
  }
  if (!full_model_ptr_) {
    return false;
  }
  // contact_frame_ids: empty for joint/task; WBC passes its TSID contact frames.
  cache_.Init(full_model_ptr_, contact_frame_ids);
  q_curr_full_ = Eigen::VectorXd::Zero(full_model_ptr_->nq);
  v_curr_full_ = Eigen::VectorXd::Zero(full_model_ptr_->nv);
  return true;
}

void CombinedModelCache::BuildReorderMap(const std::vector<std::string>* arm_joint_names,
                                         const std::vector<std::string>* hand_joint_names,
                                         int full_dof, std::string_view log_prefix,
                                         const rclcpp::Logger& logger) {
  full_dof_ = full_dof;
  if (!full_model_ptr_) {
    joint_reorder_valid_ = false;
    return;
  }
  const auto& model = *full_model_ptr_;

  if (arm_joint_names == nullptr) {
    RCLCPP_WARN(logger, "%.*s primary device config unavailable — identity reorder map",
                static_cast<int>(log_prefix.size()), log_prefix.data());
    for (int i = 0; i < full_dof_; ++i) {
      ext_to_pin_q_[static_cast<std::size_t>(i)] = i;
      ext_to_pin_v_[static_cast<std::size_t>(i)] = i;
    }
    joint_reorder_valid_ = (full_dof_ > 0);
    return;
  }

  int ext_idx = 0;
  const auto map_joint = [&](const std::string& jname) {
    if (!model.existJointName(jname)) {
      RCLCPP_ERROR(logger, "%.*s joint '%s' not found in control model",
                   static_cast<int>(log_prefix.size()), log_prefix.data(), jname.c_str());
      return;
    }
    const auto jid = model.getJointId(jname);
    const auto eidx = static_cast<std::size_t>(ext_idx);
    ext_to_pin_q_[eidx] = model.idx_qs[jid];
    ext_to_pin_v_[eidx] = model.idx_vs[jid];
    ++ext_idx;
  };

  for (const auto& jname : *arm_joint_names) {
    map_joint(jname);
  }
  if (hand_joint_names != nullptr) {
    for (const auto& jname : *hand_joint_names) {
      map_joint(jname);
    }
  }

  joint_reorder_valid_ = (ext_idx == full_dof_);
  if (!joint_reorder_valid_) {
    RCLCPP_ERROR(logger, "%.*s joint reorder incomplete: mapped %d/%d joints",
                 static_cast<int>(log_prefix.size()), log_prefix.data(), ext_idx, full_dof_);
  }
}

void CombinedModelCache::ExtractFullState(const rtc::ControllerState& state, int arm_dof,
                                          int hand_dof) noexcept {
  RTC_TRACE_SCOPE("CombinedModelCache::ExtractFullState");
  if (!joint_reorder_valid_) {
    return;
  }
  const auto& dev0 = state.devices[0];
  // Clamp to the device channel count: arm_dof == num_channels by config
  // construction, but a mismatched config must not scatter stale slots past the
  // valid measured range into q/v.
  const int narm = std::min(arm_dof, dev0.num_channels);
  for (int i = 0; i < narm; ++i) {
    const auto eidx = static_cast<std::size_t>(i);
    const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
    const auto pv = static_cast<Eigen::Index>(ext_to_pin_v_[eidx]);
    q_curr_full_[pq] = dev0.positions[eidx];
    v_curr_full_[pv] = dev0.velocities[eidx];
  }
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    const int nhand = std::min(hand_dof, dev1.num_channels);
    for (int i = 0; i < nhand; ++i) {
      const auto eidx = static_cast<std::size_t>(arm_dof + i);
      const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
      const auto pv = static_cast<Eigen::Index>(ext_to_pin_v_[eidx]);
      q_curr_full_[pq] = dev1.positions[static_cast<std::size_t>(i)];
      v_curr_full_[pv] = dev1.velocities[static_cast<std::size_t>(i)];
    }
  }
}

void CombinedModelCache::Update() noexcept {
  cache_.Update(q_curr_full_, v_curr_full_);
}

pinocchio::SE3 CombinedModelCache::ArmTcpPoseFromCache(int tcp_idx, int base_idx) const noexcept {
  // Identity unless the cache is both configured (frame registered) AND fresh
  // (reorder valid → Update() ran this run). Reading a registered-but-never-
  // Updated frame would return a stale/Identity oMf; gating here keeps every
  // consumer (FK log, vTCP, CLIK) consistent with the Update guard.
  if (tcp_idx < 0 || !joint_reorder_valid_) {
    return pinocchio::SE3::Identity();
  }
  const auto& frames = cache_.registered_frames;
  const pinocchio::SE3& tip = frames[static_cast<std::size_t>(tcp_idx)].oMf;
  if (base_idx >= 0) {
    return frames[static_cast<std::size_t>(base_idx)].oMf.actInv(tip);
  }
  return tip;
}

}  // namespace integrated_bringup
