// ── CombinedModelCache 구현 (#174) ──
#include "integrated_bringup/support/combined_model_cache.hpp"

#include "rtc_base/tracing/trace_scope.hpp"
#include "rtc_controller_interface/device_readability.hpp"
#include "rtc_base/types/types.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <exception>
#include <utility>

namespace integrated_bringup {

bool CombinedModelCache::SelectModel(rtc_urdf_bridge::PinocchioModelBuilder& builder,
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
  q_curr_full_ = Eigen::VectorXd::Zero(full_model_ptr_->nq);
  v_curr_full_ = Eigen::VectorXd::Zero(full_model_ptr_->nv);
  return true;
}

bool CombinedModelCache::InitModel(rtc_urdf_bridge::PinocchioModelBuilder& builder,
                                   const std::vector<pinocchio::FrameIndex>& contact_frame_ids,
                                   std::string_view log_prefix, const rclcpp::Logger& logger) {
  if (!SelectModel(builder, log_prefix, logger)) {
    return false;
  }
  // contact_frame_ids: empty for joint/task; WBC passes its TSID contact frames.
  // (WBC defers this cache Init to InitCacheDeferred once its contact frame ids
  // are parsed — see DemoWbcController::LoadConfig.)
  cache_.Init(full_model_ptr_, contact_frame_ids);
  cache_initialized_ = true;
  return true;
}

void CombinedModelCache::InitCacheDeferred(
    const std::vector<pinocchio::FrameIndex>& contact_frame_ids) {
  // Deferred counterpart to SelectModel: SelectModel must have run (full_model_ptr_
  // set). Sets cache_initialized_ so Update()/ArmTcpPoseFromCache() gate correctly
  // without the caller reaching into cache().Init() raw.
  cache_.Init(full_model_ptr_, contact_frame_ids);
  cache_initialized_ = true;
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
  // F5 gate, INSIDE the helper (#236 S7b). Six call sites across three
  // controllers scatter through here, and the bound below cannot make a narrow
  // device safe — q_curr_full_ PERSISTS across ticks, so the slots the bound
  // skips keep their previous value (zero before the first readable tick) and
  // the model runs at a configuration numerically identical to reading the
  // unreported channels outright (#265 comment 2 §2). Every caller gates as
  // well; this one is what makes the seventh caller safe by construction.
  if (!rtc::IsDeviceReadable(dev0, arm_dof)) {
    return;
  }
  // Still bounded: over-reporting (nc0 > arm_dof) is a normal input, and
  // ext_to_pin_q_ only has full_dof_ entries. This is the OOB half — see
  // rtc_controller_interface/device_readability.hpp for why it has its own name.
  const int narm = rtc::ModelChannelBound(arm_dof, dev0.num_channels);
  for (int i = 0; i < narm; ++i) {
    const auto eidx = static_cast<std::size_t>(i);
    const auto pq = static_cast<Eigen::Index>(ext_to_pin_q_[eidx]);
    const auto pv = static_cast<Eigen::Index>(ext_to_pin_v_[eidx]);
    q_curr_full_[pq] = dev0.positions[eidx];
    v_curr_full_[pv] = dev0.velocities[eidx];
  }
  if (state.num_devices > 1 && state.devices[1].valid) {
    const auto& dev1 = state.devices[1];
    // Secondary lane keeps its own (weaker) `valid` guard: the F5 gate above is
    // the primary device's policy, and tightening the hand to
    // IsDeviceReadable(dev1, hand_dof) would change which ticks refresh the
    // hand model — a separate decision, outside #236 S7b's scope.
    const int nhand = rtc::ModelChannelBound(hand_dof, dev1.num_channels);
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
  if (!cache_initialized_) {
    return;
  }
  cache_.Update(q_curr_full_, v_curr_full_);
}

pinocchio::SE3 CombinedModelCache::ArmTcpPoseFromCache(int tcp_idx, int base_idx) const noexcept {
  // Identity unless the cache is both configured (frame registered) AND fresh
  // (reorder valid → Update() ran this run). Reading a registered-but-never-
  // Updated frame would return a stale/Identity oMf; gating here keeps every
  // consumer (FK log, vTCP, CLIK) consistent with the Update guard.
  if (tcp_idx < 0 || !joint_reorder_valid_ || !cache_initialized_) {
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
