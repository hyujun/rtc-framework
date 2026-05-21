#pragma once

#include "rtc_tsid/types/object_frame.hpp"

#include <Eigen/Core>

namespace rtc::tsid {

// ────────────────────────────────────────────────
// Stage B-4: ObjectStateProvider — abstract interface that decouples
// ObjectSE3Task from the *source* of object state (vision / external pose
// topic / kinematic estimator / ground-truth in sim). The interface is
// introduced now (before any non-stub implementation exists) so future
// concrete providers can plug in without touching task code — proactive
// satisfaction of CLAUDE.md ARCH-3 ("second concrete implementation only
// after an abstract interface").
//
// Contract:
//   - GetPose()  → current object pose (world frame). Caller treats the
//                  returned ObjectFrame as a snapshot for this tick.
//   - GetTwist() → current object spatial velocity [6] (linear; angular),
//                  world-aligned at p_o. May be zero for static priors.
//   - IsValid()  → freshness flag. Returns false when the underlying source
//                  has not produced a sample within `pose_timeout_sec`
//                  (or whenever the provider deems the state stale). When
//                  IsValid() is false, ObjectSE3Task skips (ResidualDim=0).
//
// Watchdog semantics live inside concrete providers — the interface only
// surfaces the validity bit. IdentityObjectStateProvider (below) ignores
// timeout entirely (always valid, identity pose, zero twist).
//
// RT-safe: implementations must guarantee O(1), no heap, no logging in the
// three accessors. IdentityObjectStateProvider trivially satisfies this.
// ────────────────────────────────────────────────
class ObjectStateProvider {
 public:
  virtual ~ObjectStateProvider() = default;

  // Polymorphic accessors — see contract above. All three must be noexcept
  // and RT-safe in concrete implementations.
  [[nodiscard]] virtual const ObjectFrame& GetPose() const noexcept = 0;
  [[nodiscard]] virtual const Eigen::Matrix<double, 6, 1>& GetTwist() const noexcept = 0;
  [[nodiscard]] virtual bool IsValid() const noexcept = 0;
};

// ────────────────────────────────────────────────
// IdentityObjectStateProvider — default stub. Holds a single ObjectFrame
// (set once at configure) and a zero twist; always reports valid. Used by
// DemoWbcController in Stage B-5 default wiring before a real vision /
// estimator source is plumbed in.
//
// Stage B-4 scope: interface + stub only. No watchdog logic here — that
// arrives with the first non-trivial provider (e.g., a pose-topic adapter
// in B-5+ that owns its own last_update_time_ + pose_timeout_sec gate).
// ────────────────────────────────────────────────
class IdentityObjectStateProvider final : public ObjectStateProvider {
 public:
  void SetPose(const ObjectFrame& pose) noexcept { pose_ = pose; }

  void SetTwist(const Eigen::Matrix<double, 6, 1>& twist) noexcept { twist_ = twist; }

  void SetValid(bool valid) noexcept { valid_ = valid; }

  [[nodiscard]] const ObjectFrame& GetPose() const noexcept override { return pose_; }

  [[nodiscard]] const Eigen::Matrix<double, 6, 1>& GetTwist() const noexcept override {
    return twist_;
  }

  [[nodiscard]] bool IsValid() const noexcept override { return valid_; }

 private:
  ObjectFrame pose_{};
  Eigen::Matrix<double, 6, 1> twist_{Eigen::Matrix<double, 6, 1>::Zero()};
  bool valid_{true};
};

}  // namespace rtc::tsid
