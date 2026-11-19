// ── The estimated measured state: IMU + leg kinematics through the contact InEKF ─
//
// InekfStateSource is the second MeasuredStateSource (measured_state_source.hpp):
// the joints come from the encoders as with truth, the floating base from
// rtc::estimation::ContactInekfSequencer (contact_fsm.hpp) driven by the body
// group's primary IMU site and the contact points' forward kinematics. The
// simulator's base lane is never read.
//
// ### Frames and symbols
//
//   W   the estimator's world: z up, origin and heading fixed at the start (gauge)
//   B   the model's floating-base root frame (pinocchio free-flyer)
//   I   the primary IMU site's frame = the model frame `imu_frame`, rigid on B
//   R_BI, p_BI   the IMU's pose in B: imu_frame's placement with the base at
//                its neutral pose (a frame that is not on the root joint is
//                refused at configure — its pose would move with the joints)
//   T   the sampling period = the control period (sample_period_s)
//
// ### Inputs of one tick (ImuFrameKinematics, then the sequencer's Step)
//
// The reduced (body) model at the base's neutral pose and the measured body
// joints q, q̇ gives, for contact c's frame origin, its position p_Bc and the
// linear rows J_Bc of its LOCAL_WORLD_ALIGNED Jacobian (= B axes at the
// neutral base), joint columns only. Into I:
//
//   r_c = R_BIᵀ (p_Bc − p_BI)        J_c = R_BIᵀ J_Bc        ṙ_c = J_c q̇
//
// (each contact is a point anchor at its frame origin.) The IMU sample is the
// raw gyro / accelerometer of the primary site, site frame = I.
//
// ### Cadence: k from the IMU sequence
//
//   k = imu_sequence − the last one read
//   k = 0                 the sample repeated: no step, the base is the last
//                         output's (bit for bit), its fresh bit clear, for up to
//                         max_hold_ticks ticks; one more → kImuStale (invalid).
//                         A tick with the IMU missing counts the same way.
//   the next sample after more than max_hold_ticks quiet ticks → reset (kGap):
//                         the samples stopped longer than the state was held
//   1 ≤ k, k·T ≤ 20 ms    Step(periods = k): the nominal step k·T (never a clock
//                         difference); the expression is the core's own bound
//   k·T > 20 ms           reset (kGap) — longer than one step may span
//   sequence went back    reset (kGap): a gap of unknown length; status kImuMissing
// The first sample after Reset() counts as k = 1. Missing (site absent, flagged
// invalid, never received, non-finite): no step, kImuMissing, the sequence is not
// consumed. A refused step does not consume its sample either: the next step
// spans both. Before the start, a missed sample or a sequence that goes back
// restarts the window (nothing to reset). The IMU is sampled once per control
// tick (sample_period_s = the control period): k counts its samples, and k·T is
// the time they span.
//
// ### Joint / IMU coherence (receipt clocks, never interpolated)
//
//   s = |joint_recv − imu_recv|
//   s > max_joint_imu_skew_ticks·T   every slot's kinematics is NaN this tick:
//                                    the FSM's invalid rule (no update, modes
//                                    kept), the IMU still propagates
//   s > max_skew_invalid_ticks·T     the same, and the state is invalid (kImuSkew)
// Unreadable, stale or non-finite body joints make the kinematics NaN in the same way.
//
// ### Start (stationary window)
//
// Until the filter runs, every new sample is tested on its own: ‖ω‖ ≤
// core.stationary_gyro_tolerance, |‖a‖ − g₀| ≤ core.stationary_accel_tolerance,
// the kinematics valid, every body joint speed ≤ init.joint_speed_tolerance, and
// at least one planned contact. init.window_ticks consecutive passing samples
// start it: Initialize(window means, window = count·T, yaw = init.yaw, stance
// points = r_c of the planned contacts of the last sample). A failing sample, a
// gap or the core's refusal restarts the window. With window_ticks = 1 the
// first still sample starts the filter and that tick's state is valid. Before
// the start the state is kNotInitialized (the joints are filled, the base is
// neutral). Giving up is the caller's: init.timeout_s is read by the
// controller, which faults when the source has not started in time (whatever
// the status of those ticks — a missing IMU never starts it either).
//
// The gauge: the world origin is the mean of the stance points at the start,
// the heading init.yaw — neither is observable afterwards (telemetry
// `unobservable`).
//
// ### Resets (each bumps state_epoch, records one reason)
//
//   priority  cause                                             reason
//   1         k·T > 20 ms, or the sequence went back             kGap
//   2         the step returned kStateMismatch                   kStateMismatch
//   3         ≥ max_refused_ticks steps refused in a row         kRefused
//   4         consecutive_gated_ticks ≥ ⌈max_gated_s / T⌉         kGated
//   5         ticks_without_confirmed_anchor ≥ ⌈max_no_anchor_s / T⌉   kNoAnchor
//   —         Reset() (the controller's activation)              kRequested
// A reset tick's state is invalid (kNotInitialized); the start runs again from
// the next sample. kPropagateRefused leaves the base where it was, fresh bit
// clear; the defensive kUpdateRefused has propagated (the base advanced, fresh)
// without a contact update. Both count toward max_refused_ticks.
//
// ### Output
//
//   R_WB, p_WB, v_WB = ContactInekf::BaseFromImu(R̂_WI, p̂, v̂, ω̂_I, R_BI, p_BI)
//   q_base = [p_WB, quat(R_WB) xyzw, unit, in the last output's hemisphere]
//   v_base = [R_WBᵀ v_WB ; R_BI ω̂_I]
//   joints = the encoders (as truth); contact_mask = the slots in stance or unknown
//   stamp = the older of the joint and IMU receipts; sequence = joint_sequence
//
// Verdict: the first of body unreadable, body stale, kImuMissing, kImuStale,
// kImuSkew, kNotInitialized, non-finite. The base bit of valid_mask means "the
// estimate holds a state"; fresh_mask's means "it advanced on this tick's sample".
//
// ### RT contract
//
// Construction is non-RT and throws std::invalid_argument; Reset() and Update()
// are noexcept, allocation-free and fixed-capacity.
#pragma once

#include "integrated_bringup/controllers/locomotion/inekf_source_config.hpp"
#include "integrated_bringup/controllers/locomotion/measured_state_source.hpp"
#include "rtc_controllers/estimation/contact_fsm.hpp"
#include "rtc_controllers/estimation/contact_inekf.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#pragma GCC diagnostic pop

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace integrated_bringup::locomotion {

/// @brief The contact points' IMU-frame kinematics from a floating-base model
///        (header §Inputs). One per source; not shared.
class ImuFrameKinematics {
 public:
  /// @param model           a free-flyer-root model whose joints are the body's
  /// @param contact_frames  one frame per slot, slot order
  /// @param imu_frame       the IMU's frame; must be on the root joint
  /// @throws std::invalid_argument on a model without a free-flyer root or with a
  ///         joint of more than one DoF after it (nq ≠ nv + 1), a frame
  ///         it does not have, an IMU frame on another joint, no contact or more
  ///         than rtc::estimation::kInekfMaxContacts, or more joints than a
  ///         contact Jacobian holds.
  ImuFrameKinematics(std::shared_ptr<const pinocchio::Model> model,
                     const std::vector<std::string>& contact_frames, const std::string& imu_frame);

  [[nodiscard]] int num_slots() const noexcept { return static_cast<int>(frames_.size()); }

  [[nodiscard]] int num_joints() const noexcept { return nj_; }

  [[nodiscard]] const Eigen::Matrix3d& R_BI() const noexcept { return R_BI_; }

  [[nodiscard]] const Eigen::Vector3d& p_BI() const noexcept { return p_BI_; }

  /// r_c, J_c into `kin[c]` and ṙ_c into `rdot[c]` for every slot.
  /// @param q, qd  body joints in the model's joint order (num_joints() each)
  /// @param kin    num_slots() entries whose jacobian is already 3 × num_joints()
  void Compute(std::span<const double> q, std::span<const double> qd,
               std::span<rtc::estimation::ContactKinematics> kin,
               std::span<Eigen::Vector3d> rdot) noexcept;

 private:
  rtc_urdf_bridge::RtModelHandle handle_;
  std::vector<pinocchio::FrameIndex> frames_;
  int nj_{0};
  Eigen::Matrix3d R_BI_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d R_IB_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d p_BI_{Eigen::Vector3d::Zero()};
  Eigen::VectorXd q_;  ///< the model's q: neutral base, then the joints
  Eigen::MatrixXd J_;  ///< 6 × nv
};

/// @brief IMU + leg kinematics → the measured state (header).
class InekfStateSource final : public MeasuredStateSource {
 public:
  struct Params {
    int64_t body_max_age_ns{0};  ///< > 0
    int64_t hand_max_age_ns{0};  ///< > 0
    uint32_t model_epoch{0};
    int imu_site{0};  ///< the primary IMU site's index in DeviceState::imu
    InekfSourceConfig config;
  };

  /// @param kinematics   the model binding; its slots are the model's contacts
  /// @param body_to_full kinematics joint j's slot a in the full joint order (q[7 + a])
  /// @throws std::invalid_argument on a non-positive age, a site out of range, a
  ///         `body_to_full` of another length than the kinematics' joints, or a
  ///         configuration the sequencer refuses.
  InekfStateSource(const Params& params, std::unique_ptr<ImuFrameKinematics> kinematics,
                   std::vector<int> body_to_full);

  void Reset() noexcept override;
  [[nodiscard]] bool Update(const rtc::ControllerState& state, const DeviceIndices& indices,
                            const SourceTickInput& tick, int64_t now_steady_ns,
                            MeasuredRobotState& out) noexcept override;

  [[nodiscard]] const rtc::estimation::ContactInekfSequencer& Sequencer() const noexcept {
    return seq_;
  }

  [[nodiscard]] const ImuFrameKinematics& Kinematics() const noexcept { return *kin_model_; }

 private:
  friend struct InekfStateSourceTestAccess;

  void RestartFilter(EstimatorResetReason reason) noexcept;
  /// One new sample before the start (header §Start).
  void TryStart(bool kinematics_ok, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel,
                uint32_t planned, const MeasuredRobotState& out) noexcept;
  /// The reset the step's outputs call for (kNone = none), by the priority table.
  [[nodiscard]] EstimatorResetReason StepPolicy(rtc::estimation::ContactFsmStatus st) noexcept;
  /// `stepped`: a step ran on this tick's sample and propagated.
  void FillTelemetry(MeasuredRobotState& out, bool stepped) noexcept;

  /// The quiet-tick count saturates here (a frozen IMU must not wrap it to valid).
  static constexpr uint32_t kMaxQuietTicks = 1U << 30U;

  Params params_;
  std::unique_ptr<ImuFrameKinematics> kin_model_;
  std::vector<int> body_to_full_;
  rtc::estimation::ContactInekfSequencer seq_;
  rtc::estimation::StationaryImuAccumulator window_;
  int num_slots_{0};
  uint32_t slot_mask_{0};
  double period_s_{0.0};
  int64_t skew_ns_{0};          ///< max_joint_imu_skew_ticks · T [ns]
  int64_t skew_invalid_ns_{0};  ///< max_skew_invalid_ticks · T [ns]
  int gated_limit_{1};          ///< ⌈max_gated_s / T⌉
  int no_anchor_limit_{1};      ///< ⌈max_no_anchor_s / T⌉

  // Held across ticks.
  uint32_t state_epoch_{0};
  HeldHands held_;
  bool started_{false};
  bool have_sequence_{false};
  uint64_t last_sequence_{0};
  uint32_t hold_ticks_{0};  ///< ticks since the last new sample (repeated or missing)
  int refused_ticks_{0};
  Eigen::Vector4d last_quat_{Eigen::Vector4d::Zero()};  ///< the last output's quaternion (xyzw)
  bool have_quat_{false};
  EstimatorTelemetry telemetry_{};

  // Per-tick scratch (fixed capacity).
  std::array<rtc::estimation::ContactKinematics, rtc::estimation::kInekfMaxContacts> kin_{};
  std::array<Eigen::Vector3d, rtc::estimation::kInekfMaxContacts> rdot_{};
  std::array<Eigen::Vector3d, rtc::estimation::kInekfMaxContacts> stance_{};
  std::array<double, kMaxStateJoints> q_body_{};
  std::array<double, kMaxStateJoints> qd_body_{};
};

}  // namespace integrated_bringup::locomotion
