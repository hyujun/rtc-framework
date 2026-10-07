// ── The published segments of one plan (E1-F16 #742) ─────────────────────────
//
// A segment planner solves a later segment from an earlier one: the initial
// state of a replan is evaluated on the segment the RT reports pending or
// following (MD-58), and the search is handed the same two (SegmentPlanner::
// Reported). So a planner keeps what it published — and every planner keeps it
// the same way, which is why it is here and not in each of them:
//
//  • ONE plan. The ring holds the segments of the plan the last stored segment
//    belongs to (its `plan_id` and `t_c_ns`); a segment of another plan empties
//    it first. A report of a plan the ring is not of finds nothing in it.
//  • EIGHT segments, oldest first. When it is full the oldest one the RT did
//    not last report is dropped (MD-58): the RT reports at most two, so one of
//    eight always qualifies, and a burst of same-point re-solves cannot push
//    the followed segment out.
//  • A PAYLOAD per segment, kept beside it and moved with it — what the planner
//    has to remember of the solve that produced the segment and that the
//    segment itself does not carry (a stop-path line). A planner with nothing
//    to remember uses NoSegmentPayload.
//
// RT-safe: fixed-size, no allocation, no lock, no log, no throw. One caller
// (the planner thread).
#pragma once

#include "rtc_controllers/catching/planner_io.hpp"
#include "rtc_controllers/catching/trajectory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace rtc::catching {

/// The payload of a ring whose planner remembers nothing beside a segment.
struct NoSegmentPayload {};

/// @brief The segments published for one plan, with a payload each (header).
template <typename Payload>
class SegmentRing {
 public:
  static constexpr int kSize = 8;

  /// The two segments `rt` reports, as entries of the ring.
  struct Report {
    const SegmentSnapshot* pending{nullptr};
    const SegmentSnapshot* following{nullptr};
  };

  /// @brief Forget everything: a trial reset (RT-safe).
  void Clear() noexcept {
    n_ = 0;
    plan_id_ = 0;
    t_c_ns_ = 0;
    ClearReported();
  }

  /// @brief Take what `rt` reports pending and following as the two segments
  ///        the next Push must not drop (RT-safe). A replan calls it with the
  ///        report it solves from.
  void NoteReported(const PlannerRtState& rt) noexcept {
    reported_pending_seq_ = rt.segment_pending ? rt.segment_pending_seq : 0;
    reported_active_seq_ = rt.segment_active ? rt.segment_seq : 0;
  }

  /// @brief Nothing is reported: a new plan, of which the RT has said nothing
  ///        yet (RT-safe).
  void ClearReported() noexcept {
    reported_pending_seq_ = 0;
    reported_active_seq_ = 0;
  }

  /// @brief Store a published segment and its payload (RT-safe). A segment of
  ///        another plan than the ring's empties the ring first.
  void Push(const SegmentSnapshot& p, const Payload& payload) noexcept {
    if (n_ > 0 && (p.plan_id != plan_id_ || p.t_c_ns != t_c_ns_)) {
      n_ = 0;
    }
    plan_id_ = p.plan_id;
    t_c_ns_ = p.t_c_ns;
    if (n_ == kSize) {
      // Drop the oldest segment the RT did not last report; at most two are
      // reported, so one of eight always qualifies.
      int victim = 0;
      for (int i = 0; i < n_; ++i) {
        const std::uint32_t s = seg_[U(i)].segment_seq;
        if (s != reported_pending_seq_ && s != reported_active_seq_) {
          victim = i;
          break;
        }
      }
      for (int i = victim; i + 1 < n_; ++i) {
        seg_[U(i)] = seg_[U(i + 1)];
        payload_[U(i)] = payload_[U(i + 1)];
      }
      --n_;
    }
    seg_[U(n_)] = p;
    payload_[U(n_)] = payload;
    ++n_;
  }

  [[nodiscard]] bool Empty() const noexcept { return n_ == 0; }

  /// Whether the ring holds segments of the plan `rt` follows.
  [[nodiscard]] bool IsOf(const PlannerRtState& rt) const noexcept {
    return n_ > 0 && rt.plan_active && rt.plan_id == plan_id_ && rt.plan_t_c_ns == t_c_ns_;
  }

  /// The entry with `seq`, or nullptr. Good until the next Push or Clear.
  [[nodiscard]] const SegmentSnapshot* Find(std::uint32_t seq) const noexcept {
    for (int i = 0; i < n_; ++i) {
      if (seg_[U(i)].segment_seq == seq) {
        return &seg_[U(i)];
      }
    }
    return nullptr;
  }

  /// The payload stored with `entry` — a pointer Find or ReportedIn returned.
  [[nodiscard]] const Payload& PayloadOf(const SegmentSnapshot* entry) const noexcept {
    return payload_[static_cast<std::size_t>(entry - seg_.data())];
  }

  /// @brief The ring's entries for the two segments `rt` reports — null where
  ///        it reports none, the ring is another plan's, or the seq is not in
  ///        it (RT-safe). Pointers INTO the ring: good until the next Push.
  [[nodiscard]] Report ReportedIn(const PlannerRtState& rt) const noexcept {
    // The ring is the last PUBLISHED plan's; it says nothing of another one.
    if (!IsOf(rt)) {
      return {};
    }
    return {rt.segment_pending ? Find(rt.segment_pending_seq) : nullptr,
            rt.segment_active ? Find(rt.segment_seq) : nullptr};
  }

  /// @brief SegmentPlanner::Reported on this ring: the two reported segments
  ///        as copies (RT-safe).
  void Reported(const PlannerRtState& rt, ReportedSegments& out) const noexcept {
    const Report r = ReportedIn(rt);
    out.has_pending = r.pending != nullptr;
    out.has_following = r.following != nullptr;
    if (out.has_pending) {
      out.pending = *r.pending;
    }
    if (out.has_following) {
      out.following = *r.following;
    }
  }

  /// @brief SegmentPlanner::SourceSeq on this ring: the `segment_seq` of the
  ///        segment a solve at `t_eff_ns` starts from (SourceSegmentAt on what
  ///        the RT reports — never inferred), 0 when there is none (RT-safe).
  [[nodiscard]] std::uint32_t SourceSeq(const PlannerRtState& rt,
                                        std::int64_t t_eff_ns) const noexcept {
    const Report r = ReportedIn(rt);
    const SegmentSnapshot* src = SourceSegmentAt(r.pending, r.following, t_eff_ns);
    return src != nullptr ? src->segment_seq : 0;
  }

  /// @brief SegmentPlanner::FollowedTrack on this ring: the track generation
  ///        of the plan `rt` follows, false when nothing was published for
  ///        that plan (RT-safe).
  [[nodiscard]] bool FollowedTrack(const PlannerRtState& rt,
                                   std::uint64_t& generation) const noexcept {
    if (!IsOf(rt)) {
      return false;
    }
    // Every segment of a plan carries the plan's track.
    generation = seg_[U(n_ - 1)].token.generation;
    return true;
  }

 private:
  [[nodiscard]] static constexpr std::size_t U(int i) noexcept {
    return static_cast<std::size_t>(i);
  }

  std::array<SegmentSnapshot, kSize> seg_{};
  std::array<Payload, kSize> payload_{};
  int n_{0};
  std::uint32_t plan_id_{0};
  std::int64_t t_c_ns_{0};
  std::uint32_t reported_pending_seq_{0};
  std::uint32_t reported_active_seq_{0};
};

}  // namespace rtc::catching
