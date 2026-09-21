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
#include "rtc_controllers/testing/catch_arm_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <sstream>
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
using rtc::catching::ParseCandidateCsv;
using rtc::catching::ParseSeedCsv;
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
  for (const BatchRow& row : rows) {
    const std::vector<std::string> cells = Cells(BatchCsvRow(row));
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
      for (int j = 0; j < row.result.nv; ++j) {
        const int i = ColumnOf(header, "q" + std::to_string(j));
        ASSERT_GE(i, 0);
        EXPECT_TRUE(SameBits(std::stod(cells.at(static_cast<std::size_t>(i))), row.result.q[j]))
            << "q" << j;
      }
    }
  }
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
  const std::vector<std::string> cells = Cells(BatchCsvRow(rows[0]));
  EXPECT_EQ(cells.at(static_cast<std::size_t>(ColumnOf(header, "reason_name"))), "speed_too_low");
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
  EXPECT_EQ(Cells(header).size(), Cells(BatchCsvRow(rows[0])).size());
  EXPECT_EQ(CommaCount(header), CommaCount(BatchCsvRow(rows[0])));

  // The poseless row ends in nv empty fields, which is where a splitter that
  // drops trailing empties reports the wrong width.
  BatchCandidate stalled;
  stalled.id = 9;
  stalled.p_c = Eigen::Vector3d(0.2, 0.1, 0.3);
  stalled.v_ball = Eigen::Vector3d::Zero();
  const std::string poseless =
      BatchCsvRow(RunBatch(*f.arm.handle, f.arm.frame, {stalled}, f.seeds, MapOptions()).at(0));
  EXPECT_EQ(CommaCount(header), CommaCount(poseless));
  EXPECT_EQ(Cells(header).size(), Cells(poseless).size());
  EXPECT_GE(ColumnOf(BatchCsvHeader(f.arm.nv), "q" + std::to_string(f.arm.nv - 1)), 0);
  EXPECT_LT(ColumnOf(BatchCsvHeader(f.arm.nv), "q" + std::to_string(f.arm.nv)), 0);
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
    by_id[r.candidate.id] = BatchCsvRow(r);
  }
  for (const BatchRow& r : a) {
    ASSERT_TRUE(by_id.count(r.candidate.id)) << r.candidate.id;
    EXPECT_EQ(BatchCsvRow(r), by_id.at(r.candidate.id)) << "id " << r.candidate.id;
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
