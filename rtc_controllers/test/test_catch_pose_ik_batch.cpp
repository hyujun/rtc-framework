// ── Batch plumbing for the S3.5a map: CSV contract + batch purity ───────────
//
// The map's credibility rests on two claims that `CatchPoseIk`'s own suite
// cannot make, because they are about the batch layer rather than the solver:
//
//   1. the CSV carries the solver's numbers, not a rendering of them — a map
//      whose w₅ column is rounded cannot be compared against a runtime that
//      is not (G3-I);
//   2. a candidate's verdict does not depend on what ran before it, so python
//      may shard, reorder and resume the grid freely.
//
// Both are pinned here against the real 6R arm, plus the parser's refusals —
// a candidate file with swapped columns or a non-finite entry would otherwise
// produce a plausible map rather than an error.
#include "rtc_controllers/catching/catch_pose_ik_batch.hpp"
#include "rtc_controllers/catching/catch_pose_ik_params.hpp"
#include "rtc_controllers/testing/catch_arm_fixture.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using rtc::catching::BatchCandidate;
using rtc::catching::BatchCsvHeader;
using rtc::catching::BatchCsvRow;
using rtc::catching::BatchRow;
using rtc::catching::CatchPoseIkOptions;
using rtc::catching::CatchPoseReason;
using rtc::catching::CatchPoseReasonName;
using rtc::catching::FormatCatchPoseIkOptions;
using rtc::catching::ParseCandidateCsv;
using rtc::catching::ParseCatchPoseIkParams;
using rtc::catching::ParseSeedCsv;
using rtc::catching::ResolveCatchFrame;
using rtc::catching::ResolveCatchingTree;
using rtc::catching::RunBatch;

[[nodiscard]] CatchPoseIkOptions MapOptions() {
  CatchPoseIkOptions o;
  o.max_iter = 200;
  o.eps_pos = 1e-4;
  o.alpha_max = 1e-3;
  o.dq_step_max = 0.2;
  o.manipulability_min = 0.0;
  return o;
}

/// Split keeping EVERY field, trailing empties included — `std::getline` with a
/// ',' delimiter drops the last one, which is precisely the case a poseless
/// result produces, so using it here would hide the bug this file looks for.
[[nodiscard]] std::vector<std::string> Cells(const std::string& row) {
  std::vector<std::string> out;
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = row.find(',', start);
    out.push_back(
        row.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
    if (comma == std::string::npos) {
      return out;
    }
    start = comma + 1;
  }
}

/// Field count read off the separators alone, independent of the splitter.
[[nodiscard]] std::size_t CommaCount(const std::string& s) {
  return static_cast<std::size_t>(std::count(s.begin(), s.end(), ','));
}

[[nodiscard]] int ColumnOf(const std::string& header, const std::string& name) {
  const std::vector<std::string> h = Cells(header);
  const auto it = std::find(h.begin(), h.end(), name);
  return it == h.end() ? -1 : static_cast<int>(std::distance(h.begin(), it));
}

/// Bit-identical, not almost-equal: a column that round-trips to 15 digits
/// would still break the S6.2 equivalence oracle it is meant to serve.
[[nodiscard]] bool SameBits(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

/// nv of the 6R fixture arm. Every `for (j < result.nv)` loop below is preceded
/// by an ASSERT against this: a result whose nv is 0 (which is what `Solve`'s
/// earliest returns leave) would otherwise run the loop zero times and pass
/// having checked nothing.
constexpr int kFixtureNv = 6;

struct Fixture {
  rtc::testing::Arm arm{rtc::testing::Arm6R()};
  Eigen::VectorXd seed{Eigen::VectorXd::Zero(arm.nv)};
  std::map<int, Eigen::VectorXd> seeds;

  Fixture() {
    seed << 0.15, -0.35, 0.55, 0.25, -0.45, 0.65;
    seeds[0] = seed;
  }

  /// Candidates that the seed's own neighbourhood can actually satisfy, so a
  /// rejection here would be the solver's and not an unreachable goal.
  [[nodiscard]] std::vector<BatchCandidate> Candidates(int n) {
    std::vector<BatchCandidate> out;
    for (int i = 0; i < n; ++i) {
      Eigen::VectorXd q = seed;
      q(0) += 0.08 * static_cast<double>(i);
      q(2) -= 0.05 * static_cast<double>(i);
      const rtc::testing::Target t = rtc::testing::TargetAt(arm, q, 6.0 + 0.3 * i);
      BatchCandidate c;
      c.id = 100 + i;
      c.p_c = t.p_c;
      c.v_ball = t.v_ball;
      out.push_back(c);
    }
    return out;
  }
};

// ── 1. the CSV is the solver's numbers ──────────────────────────────────────

TEST(CatchPoseIkBatchCsv, ColumnsRoundTripBitExactly) {
  Fixture f;
  const std::vector<BatchCandidate> cands = f.Candidates(4);
  const std::vector<BatchRow> rows =
      RunBatch(*f.arm.handle, f.arm.frame, cands, f.seeds, MapOptions());
  ASSERT_EQ(rows.size(), cands.size());
  ASSERT_EQ(f.arm.nv, kFixtureNv);
  const std::string header = BatchCsvHeader(f.arm.nv);

  // The columns whose exactness the map depends on.
  const struct {
    const char* name;
    double (*get)(const BatchRow&);
  } kDoubleCols[] = {
      {"pos_error", [](const BatchRow& r) { return r.result.pos_error; }},
      {"theta", [](const BatchRow& r) { return r.result.theta; }},
      {"w5", [](const BatchRow& r) { return r.result.w5; }},
      {"w6", [](const BatchRow& r) { return r.result.w6; }},
      {"manip_grad_norm", [](const BatchRow& r) { return r.result.manip_grad_norm; }},
      {"sigma_min", [](const BatchRow& r) { return r.result.sigma_min; }},
      {"lambda_sq", [](const BatchRow& r) { return r.result.lambda_sq; }},
  };

  bool saw_nontrivial = false;
  int q_cells_checked = 0;
  for (const BatchRow& row : rows) {
    const std::vector<std::string> cells = Cells(BatchCsvRow(row, f.arm.nv));
    ASSERT_EQ(cells.size(), Cells(header).size());
    for (const auto& col : kDoubleCols) {
      const int i = ColumnOf(header, col.name);
      ASSERT_GE(i, 0) << col.name;
      const double want = col.get(row);
      EXPECT_TRUE(SameBits(std::stod(cells.at(static_cast<std::size_t>(i))), want))
          << col.name << " printed as " << cells.at(static_cast<std::size_t>(i)) << ", want "
          << want;
      // Guard against a vacuous pass on a column that is all zeros.
      if (std::abs(want) > 1e-12 && want != std::floor(want)) {
        saw_nontrivial = true;
      }
    }
    // q* likewise, for the reasons that leave a pose.
    if (row.result.reason == CatchPoseReason::kNone) {
      ASSERT_EQ(row.result.nv, kFixtureNv) << "the q loop below would check nothing";
      for (int j = 0; j < row.result.nv; ++j) {
        const int i = ColumnOf(header, "q" + std::to_string(j));
        ASSERT_GE(i, 0);
        EXPECT_TRUE(SameBits(std::stod(cells.at(static_cast<std::size_t>(i))), row.result.q[j]))
            << "q" << j;
        ++q_cells_checked;
      }
    }
  }
  EXPECT_GT(q_cells_checked, 0) << "no candidate converged, so no q column was ever compared";
  EXPECT_TRUE(saw_nontrivial) << "every checked column was 0 or integral — the round-trip "
                                 "assertion proved nothing";
}

TEST(CatchPoseIkBatchCsv, PoselessReasonsLeaveTheQColumnsEmpty) {
  // kSpeedTooLow never produces an iterate, so q must not be published: the
  // buffer is untouched and printing its zeros would read as a posture.
  Fixture f;
  BatchCandidate c;
  c.id = 7;
  c.p_c = Eigen::Vector3d(0.2, 0.1, 0.3);
  c.v_ball = Eigen::Vector3d::Zero();
  const std::vector<BatchRow> rows =
      RunBatch(*f.arm.handle, f.arm.frame, {c}, f.seeds, MapOptions());
  ASSERT_EQ(rows.size(), 1U);
  ASSERT_EQ(rows[0].result.reason, CatchPoseReason::kSpeedTooLow);
  const std::string header = BatchCsvHeader(f.arm.nv);
  const std::vector<std::string> cells = Cells(BatchCsvRow(rows[0], f.arm.nv));
  EXPECT_EQ(cells.at(static_cast<std::size_t>(ColumnOf(header, "reason_name"))), "speed_too_low");
  ASSERT_EQ(f.arm.nv, kFixtureNv);
  ASSERT_EQ(rows[0].result.nv, kFixtureNv) << "the q loop below would check nothing";
  for (int j = 0; j < rows[0].result.nv; ++j) {
    const int i = ColumnOf(header, "q" + std::to_string(j));
    ASSERT_GE(i, 0);
    EXPECT_TRUE(cells.at(static_cast<std::size_t>(i)).empty()) << "q" << j;
  }
}

TEST(CatchPoseIkBatchCsv, HeaderWidthMatchesRowWidth) {
  Fixture f;
  const std::vector<BatchRow> rows =
      RunBatch(*f.arm.handle, f.arm.frame, f.Candidates(1), f.seeds, MapOptions());
  const std::string header = BatchCsvHeader(f.arm.nv);
  EXPECT_EQ(Cells(header).size(), Cells(BatchCsvRow(rows[0], f.arm.nv)).size());
  EXPECT_EQ(CommaCount(header), CommaCount(BatchCsvRow(rows[0], f.arm.nv)));

  // The poseless row ends in nv empty fields, which is where a splitter that
  // drops trailing empties reports the wrong width.
  BatchCandidate stalled;
  stalled.id = 9;
  stalled.p_c = Eigen::Vector3d(0.2, 0.1, 0.3);
  stalled.v_ball = Eigen::Vector3d::Zero();
  const std::string poseless = BatchCsvRow(
      RunBatch(*f.arm.handle, f.arm.frame, {stalled}, f.seeds, MapOptions()).at(0), f.arm.nv);
  EXPECT_EQ(CommaCount(header), CommaCount(poseless));
  EXPECT_EQ(Cells(header).size(), Cells(poseless).size());
  EXPECT_GE(ColumnOf(BatchCsvHeader(f.arm.nv), "q" + std::to_string(f.arm.nv - 1)), 0);
  EXPECT_LT(ColumnOf(BatchCsvHeader(f.arm.nv), "q" + std::to_string(f.arm.nv)), 0);
}

TEST(CatchPoseIkBatchCsv, EarlyExitReasonsAreAsWideAsTheHeader) {
  // `Solve` returns kOptionsInvalid (and kModelInvalid, kJointOrderMismatch)
  // BEFORE it records an nv, so result.nv is 0 for them. A row sized off the
  // result is then nv cells short, the reader rejects the file, and the
  // fail-closed verdict never reaches the map. Driven through RunBatch with
  // the very options a TBD threshold produces (non-finite manipulability_min).
  Fixture f;
  ASSERT_EQ(f.arm.nv, kFixtureNv);
  CatchPoseIkOptions bad = MapOptions();
  bad.manipulability_min = std::numeric_limits<double>::quiet_NaN();
  const std::vector<BatchRow> rows =
      RunBatch(*f.arm.handle, f.arm.frame, f.Candidates(1), f.seeds, bad);
  ASSERT_EQ(rows.size(), 1U);
  ASSERT_EQ(rows[0].result.reason, CatchPoseReason::kOptionsInvalid);
  // The premise of the bug, pinned so that this case cannot go vacuous if
  // Solve ever starts recording nv earlier.
  ASSERT_EQ(rows[0].result.nv, 0);

  const std::string header = BatchCsvHeader(f.arm.nv);
  const std::string line = BatchCsvRow(rows[0], f.arm.nv);
  const std::vector<std::string> cells = Cells(line);
  ASSERT_EQ(cells.size(), Cells(header).size());
  EXPECT_EQ(CommaCount(line), CommaCount(header));
  EXPECT_EQ(cells.at(static_cast<std::size_t>(ColumnOf(header, "reason_name"))), "options_invalid");
  EXPECT_EQ(cells.at(static_cast<std::size_t>(ColumnOf(header, "nv"))), "0")
      << "the nv column stays the solver's number; only the row WIDTH is the header's";
  for (int j = 0; j < kFixtureNv; ++j) {
    const int i = ColumnOf(header, "q" + std::to_string(j));
    ASSERT_GE(i, 0);
    EXPECT_TRUE(cells.at(static_cast<std::size_t>(i)).empty()) << "q" << j;
  }
}

TEST(CatchPoseIkBatchCsv, ModelInvalidRowIsAsWideAsTheHeader) {
  // The second early exit, reached the way a misspelt frame reaches it: frame
  // id 0 is what RtModelHandle::GetFrameId answers for an unknown name.
  Fixture f;
  ASSERT_EQ(f.arm.nv, kFixtureNv);
  const std::vector<BatchRow> rows =
      RunBatch(*f.arm.handle, /*catch_frame=*/0, f.Candidates(1), f.seeds, MapOptions());
  ASSERT_EQ(rows.size(), 1U);
  ASSERT_EQ(rows[0].result.reason, CatchPoseReason::kModelInvalid);
  const std::string header = BatchCsvHeader(f.arm.nv);
  const std::string line = BatchCsvRow(rows[0], f.arm.nv);
  EXPECT_EQ(Cells(line).size(), Cells(header).size());
  EXPECT_EQ(CommaCount(line), CommaCount(header));
}

TEST(CatchPoseIkBatchCsv, APoseIsNeverTruncatedOrPaddedIntoTheColumns) {
  Fixture f;
  const std::vector<BatchRow> rows =
      RunBatch(*f.arm.handle, f.arm.frame, f.Candidates(1), f.seeds, MapOptions());
  ASSERT_EQ(rows.size(), 1U);
  ASSERT_EQ(rows[0].result.reason, CatchPoseReason::kNone);
  ASSERT_EQ(rows[0].result.nv, kFixtureNv);
  EXPECT_THROW((void)BatchCsvRow(rows[0], kFixtureNv - 1), std::invalid_argument);
  EXPECT_THROW((void)BatchCsvRow(rows[0], kFixtureNv + 1), std::invalid_argument);
  EXPECT_NO_THROW((void)BatchCsvRow(rows[0], kFixtureNv));
}

// ── 2. the batch is a pure function of the candidate set ────────────────────

TEST(CatchPoseIkBatchPurity, ReorderingCandidatesDoesNotChangeAnyVerdict) {
  Fixture f;
  std::vector<BatchCandidate> forward = f.Candidates(6);
  std::vector<BatchCandidate> reversed(forward.rbegin(), forward.rend());
  const std::vector<BatchRow> a =
      RunBatch(*f.arm.handle, f.arm.frame, forward, f.seeds, MapOptions());
  const std::vector<BatchRow> b =
      RunBatch(*f.arm.handle, f.arm.frame, reversed, f.seeds, MapOptions());
  ASSERT_EQ(a.size(), b.size());
  std::map<std::int64_t, std::string> by_id;
  for (const BatchRow& r : b) {
    by_id[r.candidate.id] = BatchCsvRow(r, f.arm.nv);
  }
  for (const BatchRow& r : a) {
    ASSERT_TRUE(by_id.count(r.candidate.id)) << r.candidate.id;
    EXPECT_EQ(BatchCsvRow(r, f.arm.nv), by_id.at(r.candidate.id)) << "id " << r.candidate.id;
  }
}

TEST(CatchPoseIkBatchPurity, MatchesAnInProcessSolveBitForBit) {
  Fixture f;
  const std::vector<BatchCandidate> cands = f.Candidates(3);
  const std::vector<BatchRow> rows =
      RunBatch(*f.arm.handle, f.arm.frame, cands, f.seeds, MapOptions());
  rtc::catching::CatchPoseIk ik;
  ik.Resize(f.arm.nv);
  for (std::size_t i = 0; i < cands.size(); ++i) {
    const auto direct =
        ik.Solve(*f.arm.handle, f.arm.frame, cands[i].p_c, cands[i].v_ball, f.seed, MapOptions());
    EXPECT_EQ(direct.reason, rows[i].result.reason);
    EXPECT_TRUE(SameBits(direct.w5, rows[i].result.w5));
    EXPECT_TRUE(SameBits(direct.w6, rows[i].result.w6));
    EXPECT_TRUE(SameBits(direct.pos_error, rows[i].result.pos_error));
    EXPECT_EQ(direct.iterations, rows[i].result.iterations);
    ASSERT_EQ(direct.nv, kFixtureNv) << "the q loop below would check nothing";
    ASSERT_EQ(rows[i].result.nv, kFixtureNv);
    for (int j = 0; j < direct.nv; ++j) {
      EXPECT_TRUE(SameBits(direct.q[j], rows[i].result.q[j])) << "candidate " << i << " q" << j;
    }
  }
}

TEST(CatchPoseIkBatchPurity, SeedSelectionIsPerCandidate) {
  Fixture f;
  Eigen::VectorXd other = f.seed;
  other(1) += 0.9;
  f.seeds[1] = other;
  std::vector<BatchCandidate> cands = f.Candidates(2);
  cands[1].seed_id = 1;
  const std::vector<BatchRow> rows =
      RunBatch(*f.arm.handle, f.arm.frame, cands, f.seeds, MapOptions());
  // Same candidate geometry, different seed ⇒ the recorded seed_id must follow
  // the candidate, and the solver must actually have been given that seed.
  EXPECT_EQ(rows[0].candidate.seed_id, 0);
  EXPECT_EQ(rows[1].candidate.seed_id, 1);
  rtc::catching::CatchPoseIk ik;
  ik.Resize(f.arm.nv);
  const auto with_other =
      ik.Solve(*f.arm.handle, f.arm.frame, cands[1].p_c, cands[1].v_ball, other, MapOptions());
  ASSERT_EQ(with_other.nv, kFixtureNv) << "the q loop below would check nothing";
  ASSERT_EQ(rows[1].result.nv, kFixtureNv);
  for (int j = 0; j < with_other.nv; ++j) {
    EXPECT_TRUE(SameBits(with_other.q[j], rows[1].result.q[j])) << "q" << j;
  }
}

TEST(CatchPoseIkBatchPurity, AbsentOrWrongSizedSeedIsRefused) {
  Fixture f;
  std::vector<BatchCandidate> cands = f.Candidates(1);
  cands[0].seed_id = 42;
  EXPECT_THROW((void)RunBatch(*f.arm.handle, f.arm.frame, cands, f.seeds, MapOptions()),
               std::invalid_argument);

  std::map<int, Eigen::VectorXd> short_seeds;
  short_seeds[0] = Eigen::VectorXd::Zero(f.arm.nv - 1);
  EXPECT_THROW(
      (void)RunBatch(*f.arm.handle, f.arm.frame, f.Candidates(1), short_seeds, MapOptions()),
      std::invalid_argument);
}

// ── 3. the candidate file cannot be misread quietly ─────────────────────────

TEST(CatchPoseIkBatchParse, ColumnOrderComesFromTheHeader) {
  // Position and velocity swapped in the header. A parser that assumed the
  // order would read the velocity as the target and still produce a map.
  std::istringstream in(
      "id,v_x,v_y,v_z,p_c_x,p_c_y,p_c_z\n"
      "5,-1,-2,-3,0.4,0.5,0.6\n");
  const std::vector<BatchCandidate> got = ParseCandidateCsv(in);
  ASSERT_EQ(got.size(), 1U);
  EXPECT_EQ(got[0].id, 5);
  EXPECT_EQ(got[0].p_c, Eigen::Vector3d(0.4, 0.5, 0.6));
  EXPECT_EQ(got[0].v_ball, Eigen::Vector3d(-1.0, -2.0, -3.0));
}

TEST(CatchPoseIkBatchParse, SeedIdIsOptionalAndDefaultsToZero) {
  std::istringstream in(
      "# a comment\n"
      "\n"
      "id,p_c_x,p_c_y,p_c_z,v_x,v_y,v_z\n"
      "1,0.1,0.2,0.3,1,2,3\n");
  const std::vector<BatchCandidate> got = ParseCandidateCsv(in);
  ASSERT_EQ(got.size(), 1U);
  EXPECT_EQ(got[0].seed_id, 0);

  std::istringstream with(
      "id,seed_id,p_c_x,p_c_y,p_c_z,v_x,v_y,v_z\n"
      "1,3,0.1,0.2,0.3,1,2,3\n");
  EXPECT_EQ(ParseCandidateCsv(with).at(0).seed_id, 3);
}

TEST(CatchPoseIkBatchParse, RejectsMissingColumnRaggedRowAndNonFinite) {
  std::istringstream missing("id,p_c_x,p_c_y,v_x,v_y,v_z\n1,0,0,1,2,3\n");
  EXPECT_THROW((void)ParseCandidateCsv(missing), std::invalid_argument);

  std::istringstream no_id("p_c_x,p_c_y,p_c_z,v_x,v_y,v_z\n0,0,0,1,2,3\n");
  EXPECT_THROW((void)ParseCandidateCsv(no_id), std::invalid_argument);

  std::istringstream ragged("id,p_c_x,p_c_y,p_c_z,v_x,v_y,v_z\n1,0,0,0,1,2\n");
  EXPECT_THROW((void)ParseCandidateCsv(ragged), std::invalid_argument);

  // A NaN would be judged kTargetNonFinite and land in the reason histogram as
  // physics rather than as a broken generator.
  std::istringstream nan_row("id,p_c_x,p_c_y,p_c_z,v_x,v_y,v_z\n1,nan,0,0,1,2,3\n");
  EXPECT_THROW((void)ParseCandidateCsv(nan_row), std::invalid_argument);

  std::istringstream inf_row("id,p_c_x,p_c_y,p_c_z,v_x,v_y,v_z\n1,0,0,0,inf,2,3\n");
  EXPECT_THROW((void)ParseCandidateCsv(inf_row), std::invalid_argument);

  std::istringstream words("id,p_c_x,p_c_y,p_c_z,v_x,v_y,v_z\n1,left,0,0,1,2,3\n");
  EXPECT_THROW((void)ParseCandidateCsv(words), std::invalid_argument);

  std::istringstream empty("");
  EXPECT_THROW((void)ParseCandidateCsv(empty), std::invalid_argument);
}

TEST(CatchPoseIkBatchParse, SeedCsvRejectsDuplicatesAndRaggedRows) {
  std::istringstream ok("seed_id,q0,q1\n0,0.1,0.2\n1,0.3,0.4\n");
  const std::map<int, Eigen::VectorXd> seeds = ParseSeedCsv(ok);
  ASSERT_EQ(seeds.size(), 2U);
  EXPECT_EQ(seeds.at(1), Eigen::Vector2d(0.3, 0.4));

  std::istringstream dup("0,0.1,0.2\n0,0.3,0.4\n");
  EXPECT_THROW((void)ParseSeedCsv(dup), std::invalid_argument);

  std::istringstream ragged("0,0.1,0.2\n1,0.3\n");
  EXPECT_THROW((void)ParseSeedCsv(ragged), std::invalid_argument);

  std::istringstream empty("\n# nothing\n");
  EXPECT_THROW((void)ParseSeedCsv(empty), std::invalid_argument);
}

TEST(CatchPoseIkBatchParse, SeedHeaderIsRecognisedAfterCommentAndBlankLines) {
  // The header is the first line that carries anything, not physical line 1.
  std::istringstream in(
      "# seeds for the iiwa wait-pose sweep\n"
      "\n"
      "seed_id,q0,q1\n"
      "0,0.1,0.2\n"
      "1,0.3,0.4\n");
  const std::map<int, Eigen::VectorXd> seeds = ParseSeedCsv(in);
  ASSERT_EQ(seeds.size(), 2U);
  EXPECT_EQ(seeds.at(0), Eigen::Vector2d(0.1, 0.2));
  EXPECT_EQ(seeds.at(1), Eigen::Vector2d(0.3, 0.4));

  // ...and ONLY the first such line: a second `seed_id,...` row is data that
  // does not parse, not another header to be skipped.
  std::istringstream twice("seed_id,q0,q1\n0,0.1,0.2\nseed_id,q0,q1\n");
  EXPECT_THROW((void)ParseSeedCsv(twice), std::invalid_argument);
}

TEST(CatchPoseIkBatchParse, CandidateIdIsSixtyFourBit) {
  // BatchCandidate::id is int64; ids past 2^31 come from a flattened grid index.
  constexpr std::int64_t kBig = 3000000000LL;
  ASSERT_GT(kBig, static_cast<std::int64_t>(std::numeric_limits<int>::max()));
  Fixture f;
  const rtc::testing::Target t = rtc::testing::TargetAt(f.arm, f.seed, 6.0);
  std::ostringstream csv;
  csv.precision(17);
  csv << "id,p_c_x,p_c_y,p_c_z,v_x,v_y,v_z\n"
      << "3000000000," << t.p_c.x() << ',' << t.p_c.y() << ',' << t.p_c.z() << ',' << t.v_ball.x()
      << ',' << t.v_ball.y() << ',' << t.v_ball.z() << '\n';
  std::istringstream in(csv.str());
  const std::vector<BatchCandidate> got = ParseCandidateCsv(in);
  ASSERT_EQ(got.size(), 1U);
  EXPECT_EQ(got[0].id, kBig);

  // Round trip into the result row python joins on.
  const std::vector<BatchRow> rows =
      RunBatch(*f.arm.handle, f.arm.frame, got, f.seeds, MapOptions());
  ASSERT_EQ(rows.size(), 1U);
  const std::string header = BatchCsvHeader(f.arm.nv);
  const std::vector<std::string> cells = Cells(BatchCsvRow(rows[0], f.arm.nv));
  EXPECT_EQ(cells.at(static_cast<std::size_t>(ColumnOf(header, "id"))), "3000000000");

  // seed_id stays an int, and an id that overflows even 64 bits is refused.
  std::istringstream too_big(
      "id,p_c_x,p_c_y,p_c_z,v_x,v_y,v_z\n99999999999999999999,0,0,0,1,2,3\n");
  EXPECT_THROW((void)ParseCandidateCsv(too_big), std::invalid_argument);
}

// ── 3b. the catch frame cannot be misnamed quietly ──────────────────────────

TEST(CatchPoseIkBatchFrame, KnownFrameResolvesToTheHandlesId) {
  Fixture f;
  ASSERT_NE(f.arm.frame, 0U);
  EXPECT_EQ(ResolveCatchFrame(f.arm.handle->GetModel(), "catch_frame"), f.arm.frame);
}

TEST(CatchPoseIkBatchFrame, UnknownFrameIsRefusedNamingFrameAndModel) {
  Fixture f;
  const pinocchio::Model& model = f.arm.handle->GetModel();
  // The premise: the handle answers a typo with the universe frame, silently.
  ASSERT_EQ(f.arm.handle->GetFrameId("catch_frmae"), 0U);
  ASSERT_FALSE(model.name.empty());
  try {
    (void)ResolveCatchFrame(model, "catch_frmae");
    FAIL() << "a misspelt frame resolved";
  } catch (const std::invalid_argument& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("catch_frmae"), std::string::npos) << what;
    EXPECT_NE(what.find(model.name), std::string::npos) << what;
  }
}

TEST(CatchPoseIkBatchFrame, UniverseFrameIsRefused) {
  // It EXISTS, so an existFrame check alone would pass it, and Solve would then
  // reject every candidate as kModelInvalid (catch_frame == 0).
  Fixture f;
  const pinocchio::Model& model = f.arm.handle->GetModel();
  ASSERT_TRUE(model.existFrame("universe"));
  EXPECT_THROW((void)ResolveCatchFrame(model, "universe"), std::invalid_argument);
}

// ── 3c. the params file cannot be misread as "all defaults" ─────────────────

constexpr const char* kPlannerBody =
    "planner:\n"
    "  ik:\n"
    "    k_manip: 0.5\n"
    "  catchability:\n"
    "    manipulability_min:\n"
    "      arm_5row: 0.174\n";

[[nodiscard]] std::string Indented(const std::string& body, int spaces) {
  std::istringstream in(body);
  std::string line;
  std::string out;
  while (std::getline(in, line)) {
    out += std::string(static_cast<std::size_t>(spaces), ' ') + line + '\n';
  }
  return out;
}

/// The tree must not merely be FOUND — it must be the one that parses to the
/// file's numbers, which differ from the in-code defaults on purpose.
void ExpectResolvesToTheFilesValues(const std::string& yaml, const std::string& want_path) {
  const CatchPoseIkOptions defaults{};
  ASSERT_NE(defaults.k_manip, 0.5);
  ASSERT_NE(defaults.manipulability_min, 0.174);
  const rtc::catching::CatchingTree tree = ResolveCatchingTree(YAML::Load(yaml), "test.yaml");
  EXPECT_EQ(tree.path, want_path);
  const CatchPoseIkOptions opt = ParseCatchPoseIkParams(tree.node).options;
  EXPECT_DOUBLE_EQ(opt.k_manip, 0.5);
  EXPECT_DOUBLE_EQ(opt.manipulability_min, 0.174);
}

TEST(CatchPoseIkBatchParamsTree, AcceptsTopLevelCatching) {
  ExpectResolvesToTheFilesValues("catching:\n" + Indented(kPlannerBody, 2), "catching");
}

TEST(CatchPoseIkBatchParamsTree, AcceptsTheShippedControllerConfigShape) {
  // `<controller>: {catching: {...}}` with sibling keys, as the shipped files
  // have. The old resolver returned this file's ROOT, which has no `planner`,
  // and the parser then defaulted everything without a word.
  const std::string yaml =
      "demo_catching_controller:\n"
      "  command_type: \"position\"\n"
      "  diagnostic:\n"
      "    hand_step: true\n"
      "  catching:\n" +
      Indented(kPlannerBody, 4) +
      "  topics:\n"
      "    arm: {}\n";
  ExpectResolvesToTheFilesValues(yaml, "demo_catching_controller.catching");
}

TEST(CatchPoseIkBatchParamsTree, AcceptsTheBareTree) {
  ExpectResolvesToTheFilesValues(kPlannerBody, "<root>");
}

TEST(CatchPoseIkBatchParamsTree, RejectsEverythingElseNamingTheFile) {
  const char* const kRejected[] = {
      // two controllers: which one is a guess
      "a:\n  catching:\n    planner: {}\nb:\n  catching:\n    planner: {}\n",
      // ros__parameters-style nesting, one level too deep
      "node:\n  ros__parameters:\n    catching:\n      planner: {}\n",
      // a tree with neither marker
      "reference:\n  omega: 10.0\n",
      // markers of the wrong type
      "catching: 3\n",
      "planner: [1, 2]\n",
      // both markers at once
      "catching:\n  planner: {}\nplanner:\n  ik: {}\n",
      // not a map at all
      "[1, 2, 3]\n",
      "",
  };
  for (const char* yaml : kRejected) {
    try {
      (void)ResolveCatchingTree(YAML::Load(yaml), "some/params.yaml");
      ADD_FAILURE() << "resolved a tree out of:\n" << yaml;
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string(e.what()).find("some/params.yaml"), std::string::npos) << e.what();
    }
  }
}

TEST(CatchPoseIkBatchParamsTree, FormatShowsTheResolvedNumbers) {
  CatchPoseIkOptions opt;
  opt.k_manip = 0.5;
  opt.manipulability_min = 0.174;
  opt.definition = rtc::catching::ManipDefinition::kArm6Row;
  const std::string text = FormatCatchPoseIkOptions(opt);
  EXPECT_NE(text.find("planner.ik.k_manip 0.5\n"), std::string::npos) << text;
  EXPECT_NE(text.find("planner.catchability.definition arm_6row\n"), std::string::npos) << text;
  // Round-trip precision, like the CSV: the line carries the double itself.
  const std::string key = "manipulability_min ";
  const std::size_t at = text.rfind(key);
  ASSERT_NE(at, std::string::npos) << text;
  EXPECT_TRUE(SameBits(std::stod(text.substr(at + key.size())), 0.174)) << text;
}

// ── 4. reason names are a usable histogram key ──────────────────────────────

TEST(CatchPoseIkBatchCsv, EveryReasonHasADistinctName) {
  const CatchPoseReason kAll[] = {
      CatchPoseReason::kNone,
      CatchPoseReason::kOptionsInvalid,
      CatchPoseReason::kModelInvalid,
      CatchPoseReason::kJointOrderMismatch,
      CatchPoseReason::kSeedNonFinite,
      CatchPoseReason::kTargetNonFinite,
      CatchPoseReason::kVelocityNonFinite,
      CatchPoseReason::kSpeedTooLow,
      CatchPoseReason::kAxisAlignInvalid,
      CatchPoseReason::kJacobianNonFinite,
      CatchPoseReason::kQpFailed,
      CatchPoseReason::kNotConverged,
      CatchPoseReason::kRankDeficient,
      CatchPoseReason::kBelowManipMin,
  };
  std::vector<std::string> names;
  for (const CatchPoseReason r : kAll) {
    const std::string n(CatchPoseReasonName(r));
    EXPECT_NE(n, "unknown") << static_cast<int>(r);
    EXPECT_FALSE(n.empty());
    EXPECT_EQ(n.find(','), std::string::npos) << n << " would split the CSV";
    names.push_back(n);
  }
  std::sort(names.begin(), names.end());
  EXPECT_EQ(std::unique(names.begin(), names.end()), names.end()) << "two reasons share a name";
}

}  // namespace
