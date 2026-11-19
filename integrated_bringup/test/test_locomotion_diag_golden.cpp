// locomotion_diag.csv against a golden recorded on the code before the R-tier
// columns were appended: a fixed run on the installed g1_loco profile (nominal,
// degraded, a source fault and its damping ticks, the body contact lane live)
// read back from the file, every golden column compared BY NAME. The joints
// move a little and have a velocity, so the servo columns are not 0. New columns
// appended after it are not in the golden and are ignored, so a column that
// moved, changed its meaning or lost precision fails here while an appended
// one does not.
//
// Excluded: the columns that are wall time (solve / tick durations, lane and
// plan ages) — nothing deterministic to compare.
//
// Re-record (only when an existing column's contract really changes, as its
// own commit with the reason): RTC_RECORD_DIAG_GOLDEN=1 ctest -R
// test_locomotion_diag_golden (writes the file next to this test's data).
#include "csv_log_fixture.hpp"
#include "integrated_bringup/logging/locomotion_diag_log_pod.hpp"
#include "locomotion_controller_rig.hpp"
#include "locomotion_mpc_stub.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

namespace {

using namespace loco_rig;  // NOLINT(google-build-using-namespace) — the shared rig
using integrated_bringup::LocomotionControllerTestAccess;
namespace fs = std::filesystem;

const std::set<std::string> kWallTime = {"solve_time_us", "wbc_tick_us", "source_age_us",
                                         "policy_age_us", "mpc_solve_us"};

class LocomotionDiagGolden : public LocomotionControllerTest {
 protected:
  void StampContacts(int64_t now) {
    auto& body = state_.devices[kBody];
    body.num_inference_groups = 2;
    for (std::size_t c = 0; c < 2; ++c) {
      body.inference_enable[c] = true;
      body.inference_recv_steady_ns[c] = now;
      body.inference_data[c * 7 + 3] = -170.0F - 3.0F * static_cast<float>(c);
    }
  }

  rtc::testing::ScopedSessionDir session_;
};

bool SameCell(const std::string& want, const std::string& got) {
  if (want == got) {
    return true;
  }
  char* end_w = nullptr;
  char* end_g = nullptr;
  const double w = std::strtod(want.c_str(), &end_w);
  const double g = std::strtod(got.c_str(), &end_g);
  if (*end_w != '\0' || *end_g != '\0') {
    return false;
  }
  if (std::isnan(w) || std::isnan(g)) {
    return std::isnan(w) && std::isnan(g);
  }
  // 9 significant digits in the file: a relative bound (1e-6, with a 1e-12
  // floor for the solver residuals near 0) that survives another
  // architecture's rounding (FMA) and still catches a changed value.
  return std::abs(w - g) <= 1e-6 * std::abs(w) + 1e-12;
}

TEST_F(LocomotionDiagGolden, ExistingColumnsMatchTheGolden) {
  const YAML::Node cfg = ShippedConfig();
  ConfigureWithDevices(cfg, ShippedLaneDevices(cfg));
  Activate();
  MakeStanding();
  // The joints move off the latched pose and have a velocity (a fixed
  // function of the tick), so the servo / feedforward columns are not 0.
  const std::vector<double> stand = StandPose();
  int tick = 0;
  const auto step = [this, &stand, &tick](bool stale) {
    auto& body = state_.devices[kBody];
    for (std::size_t c = 0; c < stand.size(); ++c) {
      const double phase = 0.1 * static_cast<double>(tick) + 0.3 * static_cast<double>(c);
      body.positions[c] = stand[c] + 0.002 * std::sin(phase);
      body.velocities[c] = 0.02 * std::cos(phase);
    }
    ++tick;
    StampContacts(NowNs());
    last_out_ = stale ? StaleTick() : Tick();
  };
  for (int k = 0; k < 30; ++k) {
    step(false);  // nominal
  }
  for (int k = 0; k < 2; ++k) {
    step(true);  // degraded: a short source drop
  }
  for (int k = 0; k < 10; ++k) {
    step(false);
  }
  for (int k = 0; k < 8; ++k) {
    step(true);  // past max_source_bad_ticks: the source fault, then damping
  }
  for (int k = 0; k < 10; ++k) {
    step(false);  // the fault stays latched
  }
  ASSERT_EQ(LastMode(), Mode::kFault);
  LocomotionControllerTestAccess::DrainLogs(*ctrl_);
  ASSERT_EQ(LocomotionControllerTestAccess::LogDrops(*ctrl_), 0U);
  const fs::path diag = LocomotionControllerTestAccess::LogPath(
      *ctrl_, integrated_bringup::kLocomotionDiagLogInstance);
  const auto got = integrated_bringup::testfx::ReadCsv(diag);
  ASSERT_EQ(got.rows.size(), 60U);

  const fs::path golden = LOCO_DIAG_GOLDEN;
  if (std::getenv("RTC_RECORD_DIAG_GOLDEN") != nullptr) {
    fs::copy_file(diag, golden, fs::copy_options::overwrite_existing);
    GTEST_SKIP() << "recorded " << golden;
  }
  const auto want = integrated_bringup::testfx::ReadCsv(golden);
  ASSERT_EQ(want.rows.size(), got.rows.size()) << golden;
  std::size_t compared = 0;
  for (std::size_t c = 0; c < want.header.size(); ++c) {
    const std::string& name = want.header[c];
    if (kWallTime.count(name) != 0) {
      continue;
    }
    const std::size_t at = got.Column(name);
    ASSERT_LT(at, got.header.size()) << "golden column " << name << " is gone";
    EXPECT_EQ(at, c) << "golden column " << name << " moved";
    for (std::size_t r = 0; r < want.rows.size(); ++r) {
      ASSERT_EQ(got.rows[r].size(), got.header.size()) << "row " << r;
      EXPECT_TRUE(SameCell(want.rows[r][c], got.rows[r][at]))
          << name << " row " << r << ": golden " << want.rows[r][c] << ", now " << got.rows[r][at];
    }
    ++compared;
  }
  EXPECT_GT(compared, 100U);
}

}  // namespace
