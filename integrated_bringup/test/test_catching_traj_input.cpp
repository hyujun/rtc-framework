// ── Vision ingress: PointCloud2 decode and refusal (S5.2, G1-A / G1-B) ──────
//
// The layout under test is ball_perception's, reproduced here from its own
// `trajectory_layout` constants (384 B per point). That duplication is the
// point: this suite is the thing that fails when the publisher's layout moves
// under us, and a fixture that imported the publisher's header would move with
// it silently. The parser finds fields BY NAME, so an offset-only change is
// expected to be ACCEPTED — there is a case for that below, and it is what
// makes the duplication safe rather than brittle.
//
// Every refusal has its own case. They are not interchangeable: a lane that is
// being refused is indistinguishable from a silent lane on the RT side, so the
// counter that says WHICH refusal is the only diagnostic there is.

#include "catching_cloud_fixture.hpp"
#include "integrated_bringup/controllers/catching/traj_input.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace {

using integrated_bringup::CatchingTrajInput;
using integrated_bringup::CloudReject;
using integrated_bringup::TrajInputConfig;
using rtc::catching::CovarianceSnapshot;
using rtc::catching::TrajectorySnapshot;

using integrated_bringup::testing::CloudSpec;
using integrated_bringup::testing::kActivation;
using integrated_bringup::testing::kMs;
using integrated_bringup::testing::kPointStep;
using integrated_bringup::testing::kRecvSteady;
using integrated_bringup::testing::kRecvWall;
using integrated_bringup::testing::MakeCloud;
using PointField = sensor_msgs::msg::PointField;

TrajInputConfig MakeConfig() {
  TrajInputConfig cfg;
  cfg.n_min = 4;
  cfg.n_max = 20;
  cfg.dt_min_ns = 1 * kMs;
  cfg.future_tol_ns = 1 * kMs;
  cfg.horizon_min_ns = 300 * kMs;
  cfg.track_eval_offset_ns = 50 * kMs;
  cfg.expected_frame = "world";
  return cfg;
}

/// Feed one message and return the verdict; `snap`/`cov` are the caller's.
CloudReject Feed(CatchingTrajInput& in, const sensor_msgs::msg::PointCloud2& msg,
                 TrajectorySnapshot& snap, CovarianceSnapshot& cov,
                 std::int64_t recv_steady = kRecvSteady) {
  return in.OnCloud(msg, recv_steady, kRecvWall, kActivation, snap, cov);
}

class TrajInputTest : public ::testing::Test {
 protected:
  void SetUp() override { input_.Configure(MakeConfig()); }

  CatchingTrajInput input_;
  TrajectorySnapshot snap_{};
  CovarianceSnapshot cov_{};
};

// ── Acceptance ──────────────────────────────────────────────────────────────

TEST_F(TrajInputTest, DecodesAWellFormedMessage) {
  const auto msg = MakeCloud({});
  ASSERT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kNone);

  EXPECT_EQ(snap_.n, 8);
  EXPECT_TRUE(snap_.valid);
  EXPECT_EQ(snap_.token.generation, 42U);
  EXPECT_EQ(snap_.token.snapshot_sequence, 7U);
  EXPECT_EQ(snap_.token.activation_generation, kActivation);
  EXPECT_EQ(snap_.token.traj_recv_ns, kRecvSteady);

  // D-2: the first sample's ball instant is (recv_steady − origin_delay) plus
  // that point's horizon offset. Asserted as an equality rather than a range
  // because every deadline downstream is derived from it.
  const std::int64_t t_ref = kRecvSteady - 30 * kMs;
  EXPECT_EQ(snap_.s[0].t_ns, t_ref + 50 * kMs);
  EXPECT_EQ(snap_.s[7].t_ns, t_ref + 400 * kMs);

  EXPECT_DOUBLE_EQ(snap_.s[0].p[0], 0.05);
  EXPECT_DOUBLE_EQ(snap_.s[0].p[1], 0.10);
  EXPECT_DOUBLE_EQ(snap_.s[0].v[2], 3.0);
  EXPECT_DOUBLE_EQ(snap_.s[0].a[2], -9.81);

  // The covariance rides its own snapshot with the SAME token (D-22), and an
  // unknown element stays unknown rather than becoming a zero that reads as
  // certainty.
  EXPECT_EQ(cov_.token.snapshot_sequence, snap_.token.snapshot_sequence);
  EXPECT_DOUBLE_EQ(cov_.c[0][0], 2.5e-5);
  EXPECT_TRUE(std::isnan(cov_.c[0][CovarianceSnapshot::kElems - 1]));

  EXPECT_EQ(input_.LastDiagnostics().origin_delay_ns, 30 * kMs);
  EXPECT_EQ(input_.LastDiagnostics().horizon_ns, 350 * kMs);
  EXPECT_FALSE(input_.LastDiagnostics().horizon_short);
  EXPECT_EQ(input_.AcceptCount(), 1U);
}

TEST_F(TrajInputTest, AnOffsetOnlyLayoutChangeIsFollowed) {
  // The whole reason fields are found by name. The publisher is a debug topic
  // with no stable ABI; a parser that assumed offsets would keep decoding and
  // every number would stay plausible.
  const auto original = MakeCloud({});
  ASSERT_EQ(Feed(input_, original, snap_, cov_), CloudReject::kNone);
  const double x_before = snap_.s[0].p[0];

  CloudSpec moved;
  moved.sequence = 8;
  moved.shift_layout = true;
  const auto shifted = MakeCloud(moved);
  ASSERT_EQ(Feed(input_, shifted, snap_, cov_), CloudReject::kNone);
  EXPECT_DOUBLE_EQ(snap_.s[0].p[0], x_before);
  EXPECT_GE(input_.LastDiagnostics().layout_rebuilds, 2U);
}

TEST_F(TrajInputTest, AShortHorizonIsDiagnosedNotRefused) {
  // D-15: the message is well-formed and the ball is real; the prediction
  // simply does not reach far enough to commit on. Refusing it would hide a
  // vision-configuration problem behind a rejection counter.
  CloudSpec spec;
  spec.n = 4;  // 4 x 50 ms spans 150 ms, below the 300 ms requirement
  const auto msg = MakeCloud(spec);
  EXPECT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kNone);
  EXPECT_TRUE(input_.LastDiagnostics().horizon_short);
}

// ── Shape and layout refusals ───────────────────────────────────────────────

TEST_F(TrajInputTest, RefusesShapeAndSizeDefects) {
  struct Case {
    const char* what;
    CloudSpec spec;
    CloudReject expect;
  };

  std::vector<Case> cases;
  {
    CloudSpec s;
    s.big_endian = true;
    cases.push_back({"big endian", s, CloudReject::kBigEndian});
  }
  {
    CloudSpec s;
    s.height = 2;
    cases.push_back({"height != 1", s, CloudReject::kShape});
  }
  {
    CloudSpec s;
    s.n = 2;  // below n_min = 4
    cases.push_back({"too few points", s, CloudReject::kShape});
  }
  {
    CloudSpec s;
    s.n = 21;  // above n_max = 20
    cases.push_back({"above n_max", s, CloudReject::kShape});
  }
  {
    CloudSpec s;
    // Above the snapshot capacity as well. The refusal must happen BEFORE any
    // point is indexed — the reference implementation indexed first and
    // range-checked afterwards, which read past the end (S1.2, found by ASan).
    s.n = 100;
    cases.push_back({"above kCap", s, CloudReject::kShape});
  }
  {
    CloudSpec s;
    s.size_delta = -8;
    cases.push_back({"truncated data", s, CloudReject::kSize});
  }
  {
    CloudSpec s;
    s.frame_id = "camera_optical";
    cases.push_back({"wrong frame", s, CloudReject::kFrameId});
  }

  for (const auto& c : cases) {
    CatchingTrajInput in;
    in.Configure(MakeConfig());
    TrajectorySnapshot snap{};
    CovarianceSnapshot cov{};
    const auto msg = MakeCloud(c.spec);
    EXPECT_EQ(Feed(in, msg, snap, cov), c.expect) << c.what;
    EXPECT_EQ(in.RejectCount(c.expect), 1U) << c.what << ": not counted";
    EXPECT_EQ(in.AcceptCount(), 0U) << c.what;
  }
}

TEST_F(TrajInputTest, RefusesFieldDefects) {
  {
    CloudSpec s;
    s.drop_validity = true;
    const auto msg = MakeCloud(s);
    EXPECT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kMissingField);
  }
  {
    // Right name, wrong datatype: the bytes would decode to something, which
    // is exactly why the type is checked rather than trusted.
    CloudSpec s;
    s.validity_type = PointField::FLOAT64;
    const auto msg = MakeCloud(s);
    EXPECT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kFieldType);
  }
  {
    // Right name and type, wrong element count — a 6x6 covariance that is not
    // 36 elements is a different quantity.
    CloudSpec s;
    s.covariance_count = 9;
    const auto msg = MakeCloud(s);
    EXPECT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kFieldType);
  }
  {
    // A field that would read past the end of its own point. Caught while
    // building the map, so the per-point loop never has to ask.
    auto msg = MakeCloud({});
    for (auto& f : msg.fields) {
      if (f.name == "validity") {
        f.offset = kPointStep;  // one byte past the last valid offset
      }
    }
    EXPECT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kFieldBounds);
  }
}

// ── Identity, validity and order ────────────────────────────────────────────

TEST_F(TrajInputTest, RefusesAMessageWithAnyPointNotEvaluated) {
  // C-1. One unevaluated point refuses the whole message: the points are one
  // prediction, and half of one is not a shorter prediction.
  CloudSpec spec;
  spec.invalid_point = 5;
  const auto msg = MakeCloud(spec);
  EXPECT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kNotEvaluated);
  EXPECT_EQ(input_.RejectCount(CloudReject::kNotEvaluated), 1U);
}

TEST_F(TrajInputTest, RefusesAMessageWhosePointsDisagreeAboutIdentity) {
  // The identity fields are per-point on the wire but describe the MESSAGE.
  // Points that disagree leave it unattributable, and accepting it would file
  // one track's points under another's.
  CloudSpec spec;
  spec.differing_id_point = 3;
  const auto msg = MakeCloud(spec);
  EXPECT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kInconsistentId);
}

TEST_F(TrajInputTest, RefusesADuplicateOrOvertakenSnapshot) {
  const auto first = MakeCloud({});
  ASSERT_EQ(Feed(input_, first, snap_, cov_), CloudReject::kNone);

  EXPECT_EQ(Feed(input_, first, snap_, cov_), CloudReject::kStaleSequence);

  CloudSpec older;
  older.sequence = 6;
  const auto overtaken = MakeCloud(older);
  EXPECT_EQ(Feed(input_, overtaken, snap_, cov_), CloudReject::kStaleSequence);
  EXPECT_EQ(input_.RejectCount(CloudReject::kStaleSequence), 2U);

  // The accepted snapshot is untouched by the refusals — a caller that
  // published it to its SeqLock still holds the one it published.
  EXPECT_EQ(snap_.token.snapshot_sequence, 7U);
}

TEST_F(TrajInputTest, ANewTrackMayRestartItsNumbering) {
  // A-S5-4: the silent failure this rule removes is blindness after a vision
  // restart, where every message of the new track is below the old track's
  // high-water mark.
  const auto first = MakeCloud({});
  ASSERT_EQ(Feed(input_, first, snap_, cov_), CloudReject::kNone);

  CloudSpec restarted;
  restarted.generation = 43;
  restarted.sequence = 1;
  const auto msg = MakeCloud(restarted);
  EXPECT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kNone);
  EXPECT_EQ(snap_.token.generation, 43U);
}

// ── Stamp and content ───────────────────────────────────────────────────────

TEST_F(TrajInputTest, RefusesAStampFromTheFuture) {
  // The D-2 conversion subtracts the origin delay; a stamp ahead of receipt by
  // more than the tolerance means the two clocks disagree, and every deadline
  // derived from the conversion would inherit that disagreement.
  CloudSpec spec;
  spec.origin_delay_ns = -5 * kMs;  // tolerance is 1 ms
  const auto msg = MakeCloud(spec);
  EXPECT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kFutureStamp);
  // The delay is recorded even on the refusal path: its distribution is the
  // diagnostic that shows a clock drifting before it crosses the threshold.
  EXPECT_EQ(input_.LastDiagnostics().origin_delay_ns, -5 * kMs);
}

TEST_F(TrajInputTest, RefusesNonFiniteAndNonMonotonicContent) {
  // Distinct sequences, because the order check runs BEFORE the content check
  // and records what it accepted: two content defects sharing one sequence
  // would have the second refused as a duplicate and this test would pass
  // while proving only half of what it says.
  {
    CloudSpec s;
    s.sequence = 10;
    s.nan_point = 4;
    const auto msg = MakeCloud(s);
    EXPECT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kMalformed);
  }
  {
    CloudSpec s;
    s.sequence = 11;
    s.non_monotonic = true;
    const auto msg = MakeCloud(s);
    EXPECT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kMalformed);
  }
}

TEST_F(TrajInputTest, ResetForgetsTheSequenceMemory) {
  // Called on re-activation: the previous activation's accepted sequence is
  // not a claim about this one, and holding it would refuse the first message
  // of the new activation.
  const auto msg = MakeCloud({});
  ASSERT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kNone);
  ASSERT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kStaleSequence);

  input_.Reset();
  EXPECT_EQ(Feed(input_, msg, snap_, cov_), CloudReject::kNone);
}

}  // namespace
