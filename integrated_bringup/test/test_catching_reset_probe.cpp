// ── The catching controller's reset table, checked at run time (G8-A2) ──────
//
// test_catching_reset_table.py proves every RT-owned member HAS a row in the
// header's reset table. This proves the rows are TRUE: every member is set to
// a value no reset would produce ("poisoned"), a reset runs, and each member
// must then be exactly what its row says — put back by that reset, or left
// alone because it is exempt from it. A member the table claims is reset but
// the function forgot is red here; so is one the table calls exempt that the
// function quietly clears.
//
// It reads and writes private state, which is what the probe is for: it is
// the header's one test-only friend.

#include "integrated_bringup/controllers/demo_catching_controller.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

namespace integrated_bringup {

/// Test-only friend of DemoCatchingController (declared in its header).
class DemoCatchingControllerResetProbe {
 public:
  explicit DemoCatchingControllerResetProbe(DemoCatchingController& c) : c_(c) {}

  static constexpr std::uint64_t kGen = 777;
  static constexpr std::int64_t kNs = 123'456'789;

  /// Every RT-owned member in the table, set to a value no reset produces.
  void Poison() {
    c_.arm_hold_.width = 3;
    c_.arm_hold_.commands[0] = 9.0;
    c_.hand_hold_.width = 2;
    c_.hand_target_raw_[0] = 9.0;
    c_.hand_target_width_ = 2;
    c_.mode_ = rtc::catching::Mode::kHold;
    c_.last_reason_ = rtc::catching::Reason::kTrackErr;
    c_.plan_.plan_id = 41;
    c_.plan_.token.generation = kGen;
    c_.plan_active_ = true;
    c_.admitted_plan_ = rtc::catching::AdmittedPlan{true, 41};
    c_.reset_floor_ns_ = 5;
    c_.planner_reset_epoch_ = 10;
    c_.arm_cmd_seeded_ = true;
    c_.arm_q_cmd_[0] = 1.5;
    c_.arm_qd_cmd_[0] = 0.7;
    c_.reference_seeded_ = true;
    c_.traj_hint_ = 6;
    c_.qp_fail_streak_ = 2;
    c_.track_err_ = 0.4;
    c_.abort_stopped_ = true;
    c_.consumed_.seen = true;
    c_.last_track_generation_ = kGen;
    c_.track_seen_ = true;
    c_.traj_new_track_ = true;
    c_.traj_view_.stale = false;
    c_.hand_out_.active = true;
    c_.homing_ = true;
    c_.homing_done_ = false;
    c_.retreat_stage_ = DemoCatchingController::RetreatStage::kRelease;
    c_.release_deferred_ = true;
    c_.trial_active_ = true;
    c_.trial_committed_ = true;
    c_.committed_t_c_ns_ = kNs;
    c_.committed_t_cmd_ns_ = kNs;
    c_.committed_generation_ = kGen;
    c_.law_snapshot_.valid = true;
    c_.last_trial_generation_ = 3;
    c_.last_trial_generation_valid_ = false;
    c_.decel_entry_.x_s.x() = 1.0;
    c_.decel_t_s_ns_ = kNs;
    c_.decel_stopped_ = true;
    c_.hold_entry_ns_ = kNs;
    c_.sat_streak_ = 4;
    c_.law_horizon_extrap_ = true;
    c_.outcome_ = rtc::catching::Outcome::kCaptured;
    c_.contact_confirmed_seen_ = true;
    rtc::catching::ContactDebounceConfig cfg;
    cfg.n_debounce = 1;
    static_cast<void>(c_.contact_.Configure(cfg));
    static_cast<void>(c_.contact_.UpdateBaseline(0, Eigen::Vector3d::Zero()));
    static_cast<void>(c_.contact_.UpdateContact(0, Eigen::Vector3d(5.0, 0.0, 0.0)));
    c_.tip_baseline_n_[0] = 30;
    c_.tip_last_seq_[0] = 99;
    c_.window_confirmed_seen_ = true;
    c_.window_stale_seen_ = true;
    c_.tick_now_ = rtc::catching::NowReal{kNs * 2};
  }

  void Rearm() { c_.ResetForRearm(); }

  void TrialReset(bool reset_mode) { c_.ResetTrialState(reset_mode); }

  void CheckRearm() {
    Poison();
    Rearm();
    auto& c = c_;

    // ── R rows: back to their start-of-trial value ──
    EXPECT_FALSE(c.plan_active_);
    EXPECT_EQ(c.plan_.plan_id, 0U);
    EXPECT_FALSE(c.admitted_plan_.seen);
    EXPECT_EQ(c.reset_floor_ns_, DemoCatchingControllerResetProbe::kNs * 2)
        << "the floor is the re-arm tick's instant";
    EXPECT_EQ(c.planner_reset_epoch_, 11U) << "the floor and the epoch move together";
    EXPECT_EQ(c.arm_qd_cmd_[0], 0.0);
    EXPECT_FALSE(c.reference_seeded_);
    EXPECT_EQ(c.traj_hint_, 0);
    EXPECT_EQ(c.track_err_, 0.0);
    EXPECT_FALSE(c.homing_);
    EXPECT_TRUE(c.homing_done_) << "RETREAT ends at the wait pose";
    EXPECT_EQ(c.retreat_stage_, DemoCatchingController::RetreatStage::kStop);
    EXPECT_FALSE(c.release_deferred_);
    EXPECT_FALSE(c.trial_active_);
    EXPECT_FALSE(c.trial_committed_);
    EXPECT_EQ(c.committed_t_c_ns_, 0);
    EXPECT_EQ(c.committed_t_cmd_ns_, 0);
    EXPECT_EQ(c.committed_generation_, 0U);
    EXPECT_FALSE(c.law_snapshot_.valid);
    EXPECT_EQ(c.decel_entry_.x_s.x(), 0.0);
    EXPECT_EQ(c.decel_t_s_ns_, 0);
    EXPECT_FALSE(c.decel_stopped_);
    EXPECT_EQ(c.hold_entry_ns_, 0);
    EXPECT_EQ(c.sat_streak_, 0);
    EXPECT_FALSE(c.law_horizon_extrap_);
    EXPECT_FALSE(c.contact_confirmed_seen_);
    EXPECT_FALSE(c.contact_.Baseline(0).initialized);
    EXPECT_FALSE(c.contact_.IsConfirmed(0));
    EXPECT_EQ(c.tip_baseline_n_[0], 0);
    EXPECT_FALSE(c.window_confirmed_seen_);
    EXPECT_FALSE(c.window_stale_seen_);
    // R writes this one: the trial that just ended was on the committed track.
    EXPECT_TRUE(c.last_trial_generation_valid_);
    EXPECT_EQ(c.last_trial_generation_, DemoCatchingControllerResetProbe::kGen);

    // ── Exempt from R: untouched ──
    EXPECT_EQ(c.arm_hold_.width, 3) << "the latch is the activation's, not the trial's";
    EXPECT_EQ(c.hand_hold_.width, 2);
    EXPECT_EQ(c.hand_target_width_, 2);
    EXPECT_EQ(c.mode_, rtc::catching::Mode::kHold) << "the mode is the table's, not the reset's";
    EXPECT_TRUE(c.arm_cmd_seeded_) << "the carried command IS the wait pose";
    EXPECT_EQ(c.arm_q_cmd_[0], 1.5);
    EXPECT_EQ(c.qp_fail_streak_, 2) << "C-29: a retry cycle contains no solve";
    EXPECT_TRUE(c.abort_stopped_);
    EXPECT_TRUE(c.consumed_.seen) << "the next ball is judged against the vision memory";
    EXPECT_EQ(c.last_track_generation_, DemoCatchingControllerResetProbe::kGen);
    EXPECT_TRUE(c.track_seen_);
    EXPECT_EQ(c.outcome_, rtc::catching::Outcome::kCaptured) << "it reports the LAST attempt";
    EXPECT_EQ(c.tip_last_seq_[0], 99U) << "a sample fed before the re-arm is not new";
  }

  void CheckTrialReset() {
    Poison();
    TrialReset(/*reset_mode=*/true);
    auto& c = c_;

    EXPECT_EQ(c.arm_hold_.width, 0);
    EXPECT_EQ(c.hand_hold_.width, 0);
    EXPECT_EQ(c.hand_target_raw_[0], 0.0);
    EXPECT_EQ(c.hand_target_width_, 0);
    EXPECT_EQ(c.mode_, rtc::catching::Mode::kIdle);
    EXPECT_EQ(c.last_reason_, rtc::catching::Reason::kNone);
    EXPECT_FALSE(c.plan_active_);
    EXPECT_FALSE(c.admitted_plan_.seen);
    EXPECT_GT(c.reset_floor_ns_, 5);
    EXPECT_EQ(c.planner_reset_epoch_, 11U);
    EXPECT_FALSE(c.arm_cmd_seeded_);
    EXPECT_EQ(c.arm_qd_cmd_[0], 0.0);
    EXPECT_FALSE(c.reference_seeded_);
    EXPECT_EQ(c.traj_hint_, 0);
    EXPECT_EQ(c.qp_fail_streak_, 0);
    EXPECT_EQ(c.track_err_, 0.0);
    EXPECT_FALSE(c.consumed_.seen);
    EXPECT_EQ(c.last_track_generation_, 0U);
    EXPECT_FALSE(c.track_seen_);
    EXPECT_FALSE(c.traj_new_track_);
    EXPECT_TRUE(c.traj_view_.stale);
    EXPECT_FALSE(c.hand_out_.active);
    EXPECT_FALSE(c.hand_seq_.Active());
    EXPECT_FALSE(c.homing_);
    EXPECT_FALSE(c.homing_done_);
    EXPECT_EQ(c.retreat_stage_, DemoCatchingController::RetreatStage::kStop);
    EXPECT_FALSE(c.release_deferred_);
    EXPECT_FALSE(c.trial_active_);
    EXPECT_FALSE(c.trial_committed_);
    EXPECT_EQ(c.committed_generation_, 0U);
    EXPECT_FALSE(c.law_snapshot_.valid);
    EXPECT_FALSE(c.last_trial_generation_valid_);
    EXPECT_EQ(c.last_trial_generation_, 0U);
    EXPECT_EQ(c.decel_t_s_ns_, 0);
    EXPECT_EQ(c.hold_entry_ns_, 0);
    EXPECT_EQ(c.sat_streak_, 0);
    EXPECT_FALSE(c.law_horizon_extrap_);
    EXPECT_EQ(c.outcome_, rtc::catching::Outcome::kNone);
    EXPECT_FALSE(c.contact_confirmed_seen_);
    EXPECT_FALSE(c.contact_.Baseline(0).initialized);
    EXPECT_EQ(c.tip_baseline_n_[0], 0);
    EXPECT_EQ(c.tip_last_seq_[0], 0U);
    EXPECT_FALSE(c.window_confirmed_seen_);
    EXPECT_FALSE(c.window_stale_seen_);
    // Exempt from every reset: written on ABORT_SAFE entry.
    EXPECT_TRUE(c.abort_stopped_);
  }

  void CheckEstopKeepsTheMode() {
    // P-1 (d): a FAULT survives a stop — the reset must not force IDLE.
    Poison();
    c_.mode_ = rtc::catching::Mode::kFault;
    TrialReset(/*reset_mode=*/false);
    EXPECT_EQ(c_.mode_, rtc::catching::Mode::kFault);
    // An E-STOP that lands on an attempt ends it as Aborted (L7 §4.1); the
    // poison left one under way (trial_active_).
    EXPECT_EQ(c_.outcome_, rtc::catching::Outcome::kAborted);
    EXPECT_FALSE(c_.trial_active_);
  }

 private:
  DemoCatchingController& c_;
};

}  // namespace integrated_bringup

namespace {

using integrated_bringup::DemoCatchingController;
using integrated_bringup::DemoCatchingControllerResetProbe;
using Probe = DemoCatchingControllerResetProbe;

// ── ResetForRearm (RETREAT → ARMED) ─────────────────────────────────────────

TEST(CatchingResetProbe, ResetForRearmPutsBackItsRowsAndLeavesItsExemptions) {
  DemoCatchingController ctrl{""};
  Probe p(ctrl);
  p.CheckRearm();
}

// ── ResetTrialState (activation / E-STOP / explicit) ────────────────────────

TEST(CatchingResetProbe, ResetTrialStatePutsBackEveryTrialRow) {
  DemoCatchingController ctrl{""};
  Probe p(ctrl);
  p.CheckTrialReset();
}

TEST(CatchingResetProbe, AnEstopResetLeavesTheModeToTheTable) {
  DemoCatchingController ctrl{""};
  Probe p(ctrl);
  p.CheckEstopKeepsTheMode();
}

}  // namespace
