// ── The published segments of the followed plan and of its replacement ───────
// (E1-F16 #742, E1-F17 #743)
//
// A segment planner solves a later segment from an earlier one: the initial
// state of a replan is evaluated on the segment the RT reports pending or
// following (MD-58), the first segment of a REPLACEMENT plan starts on one too,
// and the search is handed the same two (SegmentPlanner::Reported). So a
// planner keeps what it published — and every planner keeps it the same way,
// which is why it is here and not in each of them:
//
//  • WHOSE segments. Every entry is of the plan it carries (`plan_id` and
//    `t_c_ns`). Storing a segment keeps the entries of three plans and drops
//    every other one's: the plan the RT was last noted to FOLLOW
//    (NoteReported), the replacement it was last noted to HOLD
//    (PlannerRtState::plan_pending), and the plan of the segment being stored.
//    So the first segment of a replacement is stored beside the followed
//    plan's segments — the arm is still on them — and stays there while the RT
//    holds the pair, whatever is stored meanwhile: at the switch it is the one
//    segment the planner has of the plan the RT then follows. With nothing
//    noted as followed a segment of another plan leaves the ring with that
//    plan alone. In a wake's normal run the three are two: the stored segment
//    is the followed plan's or the replacement's.
//  • A REPORT is read plan by plan. The followed segment is looked up among
//    the entries of the plan the RT follows; the pending one among those, or —
//    while the RT holds a replacement (PlannerRtState::plan_pending) — among
//    the replacement's. A seq that is in the ring under another plan is not
//    what the RT reports. A report of a plan the ring holds nothing of finds
//    nothing in it.
//  • EIGHT segments, oldest first, however many plans they are of. When it
//    is full the oldest one the RT did not last report is dropped (MD-58):
//    the RT reports at most two — the followed segment and the pending one,
//    which while it holds a replacement is that replacement's first segment —
//    and each is found under its own plan, so one of eight always qualifies,
//    a burst of same-point re-solves cannot push the followed segment out,
//    and a held replacement's first segment cannot be pushed out either.
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

/// @brief The segments published for the followed plan and its replacement,
///        with a payload each (header).
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
    ClearReported();
  }

  /// @brief Take what `rt` reports as what the next Push must keep (RT-safe):
  ///        the plan it follows and the replacement it holds — whose segments
  ///        stay when a segment of another plan is stored — and the two
  ///        segments it reports pending and following, which a full ring does
  ///        not drop. A solve calls it with the report it starts from.
  void NoteReported(const PlannerRtState& rt) noexcept { noted_ = Named::Of(rt); }

  /// @brief Nothing is reported: the RT follows no plan (RT-safe). The next
  ///        Push of another plan's segment leaves the ring with that plan
  ///        alone.
  void ClearReported() noexcept { noted_ = Named{}; }

  /// @brief Store a published segment and its payload (RT-safe). Entries that
  ///        are of none of three plans are dropped first: its own, the plan
  ///        last noted as followed, and the replacement last noted as held.
  void Push(const SegmentSnapshot& p, const Payload& payload) noexcept {
    int kept = 0;
    for (int i = 0; i < n_; ++i) {
      const SegmentSnapshot& s = seg_[U(i)];
      if (IsOfPlan(s, p.plan_id, p.t_c_ns) || IsOfANotedPlan(s)) {
        if (kept != i) {
          seg_[U(kept)] = s;
          payload_[U(kept)] = payload_[U(i)];
        }
        ++kept;
      }
    }
    n_ = kept;
    if (n_ == kSize) {
      // Drop the oldest segment the RT did not last report; at most two are
      // reported, so one of eight always qualifies.
      const Report keep = Lookup(noted_);
      int victim = 0;
      for (int i = 0; i < n_; ++i) {
        const SegmentSnapshot* s = &seg_[U(i)];
        if (s != keep.pending && s != keep.following) {
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
    return rt.plan_active && NewestOf(rt.plan_id, rt.plan_t_c_ns) != nullptr;
  }

  /// The entry with `seq`, or nullptr — the oldest one, should the two plans'
  /// entries share a seq. Good until the next Push or Clear.
  [[nodiscard]] const SegmentSnapshot* Find(std::uint32_t seq) const noexcept {
    for (int i = 0; i < n_; ++i) {
      if (seg_[U(i)].segment_seq == seq) {
        return &seg_[U(i)];
      }
    }
    return nullptr;
  }

  /// The payload stored with `entry` — a pointer Find, ReportedIn or Source
  /// returned.
  [[nodiscard]] const Payload& PayloadOf(const SegmentSnapshot* entry) const noexcept {
    return payload_[static_cast<std::size_t>(entry - seg_.data())];
  }

  /// @brief The ring's entries for the two segments `rt` reports (RT-safe):
  ///        the followed one among the entries of the plan `rt` follows, the
  ///        pending one among the entries of the replacement `rt` holds
  ///        (`rt.plan_pending`) or of the followed plan. Null where `rt`
  ///        reports none, where the seq is not in the ring under that plan,
  ///        and both when the ring holds nothing of the followed plan.
  ///        Pointers INTO the ring: good until the next Push.
  [[nodiscard]] Report ReportedIn(const PlannerRtState& rt) const noexcept {
    return Lookup(Named::Of(rt));
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

  /// @brief The entry a solve at `t_eff_ns` starts from: SourceSegmentAt on
  ///        what the RT reports — never inferred — or nullptr when there is
  ///        none (RT-safe). A pointer INTO the ring: good until the next Push.
  [[nodiscard]] const SegmentSnapshot* Source(const PlannerRtState& rt,
                                              std::int64_t t_eff_ns) const noexcept {
    const Report r = ReportedIn(rt);
    return SourceSegmentAt(r.pending, r.following, t_eff_ns);
  }

  /// @brief SegmentPlanner::SourceSeq on this ring: the `segment_seq` of
  ///        Source(rt, t_eff_ns), 0 when there is none (RT-safe).
  [[nodiscard]] std::uint32_t SourceSeq(const PlannerRtState& rt,
                                        std::int64_t t_eff_ns) const noexcept {
    const SegmentSnapshot* src = Source(rt, t_eff_ns);
    return src != nullptr ? src->segment_seq : 0;
  }

  /// @brief SegmentPlanner::FollowedTrack on this ring: the track generation
  ///        of the plan `rt` follows, false when nothing was published for
  ///        that plan (RT-safe).
  [[nodiscard]] bool FollowedTrack(const PlannerRtState& rt,
                                   std::uint64_t& generation) const noexcept {
    const SegmentSnapshot* s = rt.plan_active ? NewestOf(rt.plan_id, rt.plan_t_c_ns) : nullptr;
    if (s == nullptr) {
      return false;
    }
    // Every segment of a plan carries the plan's track.
    generation = s->token.generation;
    return true;
  }

 private:
  // What a report names: the followed plan, the replacement it holds, and the
  // two segments — the part of PlannerRtState the ring reads.
  struct Named {
    bool plan_active{false};
    std::uint32_t plan_id{0};
    std::int64_t plan_t_c_ns{0};
    bool plan_pending{false};
    std::uint32_t plan_pending_id{0};
    std::int64_t plan_pending_t_c_ns{0};
    bool segment_active{false};
    std::uint32_t segment_seq{0};
    bool segment_pending{false};
    std::uint32_t segment_pending_seq{0};

    [[nodiscard]] static constexpr Named Of(const PlannerRtState& rt) noexcept {
      Named n;
      n.plan_active = rt.plan_active;
      n.plan_id = rt.plan_id;
      n.plan_t_c_ns = rt.plan_t_c_ns;
      n.plan_pending = rt.plan_pending;
      n.plan_pending_id = rt.plan_pending_id;
      n.plan_pending_t_c_ns = rt.plan_pending_t_c_ns;
      n.segment_active = rt.segment_active;
      n.segment_seq = rt.segment_seq;
      n.segment_pending = rt.segment_pending;
      n.segment_pending_seq = rt.segment_pending_seq;
      return n;
    }
  };

  [[nodiscard]] static constexpr std::size_t U(int i) noexcept {
    return static_cast<std::size_t>(i);
  }

  [[nodiscard]] static constexpr bool IsOfPlan(const SegmentSnapshot& s, std::uint32_t plan_id,
                                               std::int64_t t_c_ns) noexcept {
    return s.plan_id == plan_id && s.t_c_ns == t_c_ns;
  }

  // Whether `s` is of the plan last noted as followed, or of the replacement
  // last noted as held (a replacement is held by an RT that follows a plan).
  [[nodiscard]] constexpr bool IsOfANotedPlan(const SegmentSnapshot& s) const noexcept {
    if (!noted_.plan_active) {
      return false;
    }
    return IsOfPlan(s, noted_.plan_id, noted_.plan_t_c_ns) ||
           (noted_.plan_pending && IsOfPlan(s, noted_.plan_pending_id, noted_.plan_pending_t_c_ns));
  }

  // The entry of plan (plan_id, t_c_ns) with `seq`, or nullptr.
  [[nodiscard]] const SegmentSnapshot* FindIn(std::uint32_t plan_id, std::int64_t t_c_ns,
                                              std::uint32_t seq) const noexcept {
    for (int i = 0; i < n_; ++i) {
      if (seg_[U(i)].segment_seq == seq && IsOfPlan(seg_[U(i)], plan_id, t_c_ns)) {
        return &seg_[U(i)];
      }
    }
    return nullptr;
  }

  // The last stored entry of plan (plan_id, t_c_ns), or nullptr.
  [[nodiscard]] const SegmentSnapshot* NewestOf(std::uint32_t plan_id,
                                                std::int64_t t_c_ns) const noexcept {
    for (int i = n_ - 1; i >= 0; --i) {
      if (IsOfPlan(seg_[U(i)], plan_id, t_c_ns)) {
        return &seg_[U(i)];
      }
    }
    return nullptr;
  }

  [[nodiscard]] Report Lookup(const Named& named) const noexcept {
    // The ring says nothing of a plan it holds no segment of.
    if (!named.plan_active || NewestOf(named.plan_id, named.plan_t_c_ns) == nullptr) {
      return {};
    }
    Report r;
    if (named.segment_active) {
      r.following = FindIn(named.plan_id, named.plan_t_c_ns, named.segment_seq);
    }
    if (named.segment_pending) {
      // A held replacement's first segment is the pending one; without a
      // replacement it is the followed plan's.
      if (named.plan_pending) {
        r.pending =
            FindIn(named.plan_pending_id, named.plan_pending_t_c_ns, named.segment_pending_seq);
      }
      if (r.pending == nullptr) {
        r.pending = FindIn(named.plan_id, named.plan_t_c_ns, named.segment_pending_seq);
      }
    }
    return r;
  }

  std::array<SegmentSnapshot, kSize> seg_{};
  std::array<Payload, kSize> payload_{};
  int n_{0};
  // The report the next Push keeps by: the entries of the followed plan and
  // of the held replacement, and the two reported segments.
  Named noted_{};
};

}  // namespace rtc::catching
