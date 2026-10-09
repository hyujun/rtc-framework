// ── A digest of what the planner published and recorded (test-only) ─────────
// A refactor of the planner thread's path that claims to change nothing is
// pinned bit for bit, not by tolerance (the rule of bit_compare.hpp): the
// suites digest every VALUE a wake leaves — its record, the plan and the
// segment — and compare the digest with one taken before the change.
//
// FNV-1a (64-bit, the standard offset basis and prime), field by field. Not
// the bytes of a struct: its padding is not part of what was published and
// those bytes are not defined.
//
// A field added to one of the digested structs is NOT in the digest until it
// is added to the matching function below — and adding it changes every
// constant taken with these functions.
//
// Test-only, NEVER installed (see bit_compare.hpp for why it lives under
// test/include).
#pragma once

#include "rtc_controllers/catching/grid_catch_search.hpp"
#include "rtc_controllers/catching/mpc_segment_planner.hpp"
#include "rtc_controllers/catching/planner_cycle.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace rtc::testing {

/// FNV-1a over values: arithmetic and enum values by their object bytes (a
/// double by its bit pattern, so a NaN and a signed zero count), arrays
/// element by element.
class ValueDigest {
 public:
  template <typename T>
  void Add(const T& v) noexcept {
    static_assert(std::is_arithmetic_v<T> || std::is_enum_v<T>);
    std::array<unsigned char, sizeof(T)> bytes{};
    std::memcpy(bytes.data(), &v, sizeof(T));
    for (const unsigned char b : bytes) {
      h_ ^= b;
      h_ *= kFnvPrime;
    }
  }

  template <typename T, std::size_t N>
  void Add(const std::array<T, N>& a) noexcept {
    for (const T& v : a) {
      Add(v);
    }
  }

  void Add(const catching::ProvenanceToken& t) noexcept {
    Add(t.activation_generation);
    Add(t.generation);
    Add(t.snapshot_sequence);
    Add(t.traj_recv_ns);
  }

  [[nodiscard]] std::uint64_t Value() const noexcept { return h_; }

 private:
  static constexpr std::uint64_t kFnvOffsetBasis = 0xcbf29ce484222325ULL;
  static constexpr std::uint64_t kFnvPrime = 0x100000001b3ULL;
  std::uint64_t h_{kFnvOffsetBasis};
};

// Every field of each struct, in declaration order — except the blocks and
// fields that were added for one implementation or for the log after the
// constants were recorded: SearchStats::nlp (the NLP search's block, E1-F14),
// SegmentRecord::docking (the mpc_docking planner's block, E1-F18), and
// PlannerCycleRecord's `replacement` and `replace_step`. A digest is a pin on
// the behaviour of the searches and planners it was recorded from, and a
// block those leave at its default would move every recorded constant without
// any of them behaving differently. Each of the excepted ones is asserted
// directly by the tests of what fills it.

inline void AddSearchStats(ValueDigest& h, const catching::SearchStats& s) noexcept {
  h.Add(s.settling);
  h.Add(s.n_in_window);
  h.Add(s.n_ik);
  h.Add(s.n_pass);
  // In the layout the constants were recorded with: six counts, the fifth the
  // catch box's (JudgeReject::kWorkspace, removed with that gate — L3 §4.9). No recorded catch
  // had a candidate that gate removed, so its count is the zero it always was
  // — and the constants stay a pin on what the search does.
  // The count added after them (JudgeReject::kTooFar, the reach bound) is not
  // part of the digest either: the tests of the searches assert it directly.
  static_assert(catching::kJudgeRejectCount == 6);
  for (std::size_t i = 0; i < 4; ++i) {
    h.Add(s.judge_rejects[i]);
  }
  h.Add(std::uint16_t{0});
  h.Add(s.judge_rejects[4]);
  h.Add(s.budget_hit);
  h.Add(s.search_ns);
  h.Add(s.ik_ns_max);
  h.Add(s.rollout_ns_max);
  h.Add(s.n_rollouts);
  h.Add(s.chosen_rank_mask);
  h.Add(s.chosen_score);
  h.Add(s.chosen_lead_s);
  h.Add(s.chosen_gamma_f);
  h.Add(s.chosen_t_w);
  h.Add(s.chosen_rollout_window_only);
  h.Add(s.chosen_g_min);
  h.Add(s.chosen_g_max);
  h.Add(s.chosen_v_dir_max);
  h.Add(s.chosen_max_catchable);
  h.Add(s.decision);
  h.Add(s.publish);
  h.Add(s.sigma_l);
}

inline void AddSegmentRecord(ValueDigest& h, const catching::SegmentRecord& d) noexcept {
  h.Add(d.outcome);
  h.Add(d.core_reason);
  h.Add(d.k);
  h.Add(d.n_nodes);
  h.Add(d.segment_seq);
  h.Add(d.x0_clamped);
  h.Add(d.x0_from_segment);
  h.Add(d.presolved);
  h.Add(d.cold_retry);
  h.Add(d.iterations);
  h.Add(d.qp_status);
  h.Add(d.solve_ns);
  h.Add(d.publish_ns);
  h.Add(d.slack_max);
  h.Add(d.slack_terminal_max);
  h.Add(d.tau_ratio_max);
  h.Add(d.kind);
  h.Add(d.cold_start);
  h.Add(d.solver_retried);
  h.Add(d.ref_clamped);
  h.Add(d.ref_scaled);
  h.Add(d.ref_scale);
  h.Add(d.ref_shortfall);
  h.Add(d.x0_speed);
  h.Add(d.catch_pos_err);
  h.Add(d.catch_axis_err);
  h.Add(d.catch_gamma);
  h.Add(d.catch_v_rel);
  h.Add(d.slack_v);
  h.Add(d.speed_ratio_max);
  h.Add(d.w_p_fallback);
  h.Add(d.w_delta_scale);
  h.Add(d.source_seq);
}

inline void AddCycleRecord(ValueDigest& h, const catching::PlannerCycleRecord& r) noexcept {
  h.Add(r.outcome);
  h.Add(r.reset_seen);
  h.Add(r.cov_matched);
  h.Add(r.mode);
  h.Add(r.plan_id);
  h.Add(r.plan_valid);
  h.Add(r.search_valid);
  h.Add(r.reason);
  h.Add(r.snapshot_sequence);
  h.Add(r.track_generation);
  h.Add(r.traj_recv_ns);
  h.Add(r.wake_ns);
  h.Add(r.publish_ns);
  AddSearchStats(h, r.search);
  AddSegmentRecord(h, r.segment);
}

inline void AddPlan(ValueDigest& h, const catching::PlanSnapshot& p) noexcept {
  h.Add(p.token);
  h.Add(p.rt_iteration);
  h.Add(p.rt_state_ns);
  h.Add(p.publish_ns);
  h.Add(p.plan_id);
  h.Add(p.t_c_ns);
  h.Add(p.t_cmd_ns);
  h.Add(p.p_c);
  h.Add(p.a_d);
  h.Add(p.v_c);
  h.Add(p.gamma_g0);
  h.Add(p.gamma_gf);
  h.Add(p.gamma_t0_ns);
  h.Add(p.gamma_t1_ns);
  h.Add(p.gamma_min);
  h.Add(p.q_star);
  h.Add(p.nv);
  h.Add(p.w5);
  h.Add(p.w6);
  h.Add(p.score);
  h.Add(p.sigma_c);
  h.Add(p.sigma_l);
  h.Add(p.dp_impact);
  h.Add(p.reason);
  h.Add(p.valid);
}

inline void AddSegment(ValueDigest& h, const catching::SegmentSnapshot& p) noexcept {
  h.Add(p.token);
  h.Add(p.rt_iteration);
  h.Add(p.rt_state_ns);
  h.Add(p.publish_ns);
  h.Add(p.plan_id);
  h.Add(p.segment_seq);
  h.Add(p.t_c_ns);
  h.Add(p.t0_ns);
  h.Add(p.dt_ns);
  h.Add(p.dt_pre_ns);
  h.Add(p.k0);
  h.Add(p.n_nodes);
  h.Add(p.nv);
  h.Add(p.n_pre);
  h.Add(p.q);
  h.Add(p.qd);
  h.Add(p.qdd);
  h.Add(p.slack_max);
  h.Add(p.slack_terminal_max);
  h.Add(p.tau_ratio_max);
  h.Add(p.x0_clamped);
  h.Add(p.valid);
}

}  // namespace rtc::testing
