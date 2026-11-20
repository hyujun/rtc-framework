// The inekf state source replayed from a run's logs (estimator_replay.hpp).
#include "integrated_bringup/controllers/locomotion/estimator_replay.hpp"

#include "integrated_bringup/controllers/locomotion/humanoid_model_config.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace integrated_bringup::locomotion {

namespace est = rtc::estimation;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// A logged age / skew [µs] → ns; a non-finite or out-of-range value (strtod
// reads "nan" / "inf") is no value.
std::optional<int64_t> UsToNs(double us) {
  if (!(std::isfinite(us) && std::abs(us) < 9e12)) {
    return std::nullopt;
  }
  return std::llround(us * 1e3);
}

}  // namespace

// ── one tick rebuilt ────────────────────────────────────────────────────────

bool RebuildState(const ReplayInput& in, int imu_site, int64_t now_ns,
                  rtc::ControllerState& state) noexcept {
  state = rtc::ControllerState{};
  state.num_devices = 1;
  state.iteration = in.tick;
  state.t_relative_s = in.t_relative_s;
  auto& dev = state.devices[0];
  dev.valid = in.source_status != static_cast<uint8_t>(StateSourceStatus::kBodyUnreadable);
  const int n = std::clamp(in.num_joints, 0, static_cast<int>(kMaxStateJoints));
  dev.num_channels = n;
  for (std::size_t c = 0; c < static_cast<std::size_t>(n); ++c) {
    dev.positions[c] = in.q[c];
    dev.velocities[c] = in.qd[c];
  }
  dev.joint_sequence = in.joint_sequence;
  bool exact = false;
  if (imu_site >= 0 && imu_site < rtc::kMaxImuSites) {
    const auto s = static_cast<std::size_t>(imu_site);
    dev.num_imu_sites = imu_site + 1;
    dev.imu[s] = in.imu;
    dev.imu_valid[s] = in.imu_valid;
    dev.imu_sequence[s] = in.imu_sequence;
    const auto age = UsToNs(in.imu_age_us);
    const auto skew = UsToNs(in.skew_us);
    const int64_t imu_recv = age && *age >= 0 ? now_ns - *age : 0;
    dev.imu_recv_steady_ns[s] = imu_recv;
    exact = in.imu_valid && imu_recv > 0 && skew.has_value();
    if (exact) {
      dev.joint_recv_steady_ns = imu_recv + *skew;
    }
  }
  if (!exact) {
    // −1 is "no receipt"; any other value (a negative one too: a receipt
    // stamped after the clock read) is an age.
    const auto age = UsToNs(in.source_age_us);
    dev.joint_recv_steady_ns = in.source_age_us != -1.0 && age ? now_ns - *age : 0;
  }
  return exact;
}

// ── the driver ──────────────────────────────────────────────────────────────

EstimatorReplay::EstimatorReplay(const ReplaySetup& setup) : setup_(setup) {
  if (!setup_.model) {
    throw std::invalid_argument("estimator replay: no model");
  }
  const pinocchio::Model& model = *setup_.model;
  const std::vector<std::string> order = JointOrder(model);
  // The truth FK writes q = [p, q_xyzw, joints], v = [v, ω, joints].
  if (model.nq != 7 + static_cast<int>(order.size()) ||
      model.nv != 6 + static_cast<int>(order.size())) {
    throw std::invalid_argument(
        "estimator replay: the model is not a free-flyer root plus one-DoF joints");
  }
  indices_ = BuildDeviceIndices(order, 0, setup_.roster, {}, "estimator replay");
  std::vector<int> body_to_full(order.size());
  std::iota(body_to_full.begin(), body_to_full.end(), 0);
  auto kinematics = std::make_unique<ImuFrameKinematics>(setup_.model, setup_.contact_frames,
                                                         setup_.params.config.imu_frame);
  R_BI_ = kinematics->R_BI();
  p_BI_ = kinematics->p_BI();
  if (setup_.contact_frames.size() > kMaxEstimatorSlots) {
    throw std::invalid_argument("estimator replay: more contacts than the telemetry holds");
  }
  source_ = std::make_unique<InekfStateSource>(setup_.params, std::move(kinematics),
                                               std::move(body_to_full));
  period_ns_ = std::llround(setup_.params.config.imu.sample_period_s * 1e9);
  data_ = std::make_unique<pinocchio::Data>(model);
  for (const auto& f : setup_.contact_frames) {
    frames_.push_back(model.getFrameId(f));
  }
  roster_to_model_.resize(setup_.roster.size());
  for (std::size_t c = 0; c < setup_.roster.size(); ++c) {
    roster_to_model_[c] = indices_.body.joint[c];
  }
  for (auto& v : truth_vel_) {
    v.fill(kNaN);
  }
}

EstimatorReplay::~EstimatorReplay() = default;

const MeasuredRobotState& EstimatorReplay::Step(const ReplayInput& in, const ReplayTruth* truth) {
  // The activation's Reset(); the first tick of a log stands for one too (the
  // source the controller built was reset when it was activated).
  if (in.tick_count == 1 || !started_) {
    source_->Reset();
    started_ = true;
  }
  const int64_t now = kReplayEpochNs + static_cast<int64_t>(in.tick) * period_ns_;
  joint_exact_ = RebuildState(in, setup_.params.imu_site, now, state_);
  static_cast<void>(source_->Update(state_, indices_, SourceTickInput{in.planned_mask}, now, out_));
  TruthVelocities(in, truth);
  return out_;
}

void EstimatorReplay::TruthVelocities(const ReplayInput& in, const ReplayTruth* truth) {
  for (auto& v : truth_vel_) {
    v.fill(kNaN);
  }
  if (truth == nullptr || !truth->base_valid) {
    return;
  }
  const pinocchio::Model& model = *setup_.model;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::Quaterniond quat(truth->base_q[6], truth->base_q[3], truth->base_q[4], truth->base_q[5]);
  if (!(quat.norm() > 0.5)) {
    return;  // not a rotation (a zero or NaN quaternion): the velocities stay NaN
  }
  quat.normalize();
  q.head<3>() = Eigen::Vector3d(truth->base_q[0], truth->base_q[1], truth->base_q[2]);
  q.segment<4>(3) = quat.coeffs();
  for (std::size_t i = 0; i < 6; ++i) {
    v(static_cast<Eigen::Index>(i)) = truth->base_v[i];
  }
  for (std::size_t c = 0; c < roster_to_model_.size(); ++c) {
    const auto j = static_cast<Eigen::Index>(roster_to_model_[c]);
    q(7 + j) = in.q[c];
    v(6 + j) = in.qd[c];
  }
  if (!(q.allFinite() && v.allFinite())) {
    return;
  }
  pinocchio::forwardKinematics(model, *data_, q, v);
  for (std::size_t c = 0; c < frames_.size(); ++c) {
    const Eigen::Vector3d lin =
        pinocchio::getFrameVelocity(model, *data_, frames_[c], pinocchio::LOCAL_WORLD_ALIGNED)
            .linear();
    truth_vel_[c] = {lin.x(), lin.y(), lin.z()};
  }
}

// ── reading the logs ────────────────────────────────────────────────────────

namespace {

// One CSV read field by field, only the columns asked for parsed.
class CsvReader {
 public:
  explicit CsvReader(const std::string& path) : path_(path), in_(path) {
    if (!in_) {
      throw std::runtime_error(path + ": cannot open");
    }
    std::string line;
    if (!std::getline(in_, line)) {
      throw std::runtime_error(path + ": empty");
    }
    header_ = Split(line);
  }

  [[nodiscard]] const std::vector<std::string>& header() const noexcept { return header_; }

  [[nodiscard]] std::size_t Column(const std::string& name) const {
    const auto it = std::find(header_.begin(), header_.end(), name);
    if (it == header_.end()) {
      throw std::runtime_error(path_ + ": no column '" + name + "'");
    }
    return static_cast<std::size_t>(it - header_.begin());
  }

  /// The next row's fields; false at the end. A row of another width is refused.
  bool Next(std::vector<std::string>& fields) {
    std::string line;
    while (std::getline(in_, line)) {
      ++row_;
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (line.empty()) {
        continue;
      }
      fields = Split(line);
      if (fields.size() != header_.size()) {
        throw std::runtime_error(path_ + ": row " + std::to_string(row_) + " has " +
                                 std::to_string(fields.size()) + " fields, the header " +
                                 std::to_string(header_.size()));
      }
      return true;
    }
    return false;
  }

  [[nodiscard]] double Double(const std::vector<std::string>& f, std::size_t c) const {
    char* end = nullptr;
    const double x = std::strtod(f[c].c_str(), &end);
    if (end == f[c].c_str() || *end != '\0') {
      throw std::runtime_error(path_ + ": row " + std::to_string(row_) + " column '" + header_[c] +
                               "' is not a number: '" + f[c] + "'");
    }
    return x;
  }

  [[nodiscard]] uint64_t Unsigned(const std::vector<std::string>& f, std::size_t c) const {
    char* end = nullptr;
    const unsigned long long x = std::strtoull(f[c].c_str(), &end, 10);  // NOLINT(runtime/int)
    if (end == f[c].c_str() || *end != '\0' || f[c][0] == '-') {
      throw std::runtime_error(path_ + ": row " + std::to_string(row_) + " column '" + header_[c] +
                               "' is not an unsigned integer: '" + f[c] + "'");
    }
    return static_cast<uint64_t>(x);
  }

 private:
  static std::vector<std::string> Split(const std::string& line) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (true) {
      const std::size_t comma = line.find(',', start);
      out.push_back(line.substr(start, comma - start));
      if (comma == std::string::npos) {
        break;
      }
      start = comma + 1;
    }
    if (!out.empty() && !out.back().empty() && out.back().back() == '\r') {
      out.back().pop_back();
    }
    return out;
  }

  std::string path_;
  std::ifstream in_;
  std::vector<std::string> header_;
  std::size_t row_{0};
};

// The run a row belongs to: a new one wherever `tick` goes back (a new sim
// appended to the same file).
struct RunCounter {
  std::size_t run{0};
  bool any{false};
  uint64_t last{0};

  std::size_t Of(uint64_t tick) {
    if (any && tick < last) {
      ++run;
    }
    any = true;
    last = tick;
    return run;
  }
};

constexpr const char* kXyz[] = {"x", "y", "z"};

}  // namespace

ReplayLog LoadReplayLog(const std::string& diag_path, const std::string& joint_ff_path,
                        const std::string& truth_path,
                        const std::vector<std::string>& contact_names, int run) {
  ReplayLog log;

  // ── joint_ff: q per (run, tick) ───────────────────────────────────────────
  CsvReader diag(diag_path);
  for (const auto& name : diag.header()) {
    if (name.rfind("v_", 0) == 0) {
      log.roster.push_back(name.substr(2));
    }
  }
  if (log.roster.empty() || log.roster.size() > kMaxStateJoints) {
    throw std::runtime_error(diag_path + ": expected 1 to " + std::to_string(kMaxStateJoints) +
                             " v_<joint> columns, got " + std::to_string(log.roster.size()));
  }
  const std::size_t nj = log.roster.size();
  std::vector<std::unordered_map<uint64_t, std::array<double, kMaxStateJoints>>> q_of;
  {
    CsvReader ff(joint_ff_path);
    const std::size_t tick = ff.Column("tick");
    std::vector<std::size_t> qc;
    for (const auto& j : log.roster) {
      qc.push_back(ff.Column("q_" + j));
    }
    RunCounter runs;
    std::vector<std::string> f;
    while (ff.Next(f)) {
      const uint64_t t = ff.Unsigned(f, tick);
      const std::size_t r = runs.Of(t);
      if (q_of.size() <= r) {
        q_of.resize(r + 1);
      }
      std::array<double, kMaxStateJoints> q{};
      for (std::size_t j = 0; j < nj; ++j) {
        q[j] = ff.Double(f, qc[j]);
      }
      q_of[r][t] = q;
    }
  }

  // ── truth: per (run, tick) ────────────────────────────────────────────────
  std::vector<std::unordered_map<uint64_t, ReplayTruth>> truth_of;
  if (!truth_path.empty()) {
    CsvReader tr(truth_path);
    const std::size_t tick = tr.Column("tick");
    const std::size_t mask = tr.Column("truth_mask");
    std::array<std::size_t, 13> bc{};
    const char* base[] = {"px", "py", "pz", "qx", "qy", "qz", "qw",
                          "vx", "vy", "vz", "wx", "wy", "wz"};
    for (std::size_t i = 0; i < 13; ++i) {
      bc[i] = tr.Column(std::string("truth_base_") + base[i]);
    }
    // By name: the model's contact order need not be the recording's.
    std::vector<std::size_t> fzc;
    for (const auto& c : contact_names) {
      fzc.push_back(tr.Column("truth_grf_" + c + "_fz"));
    }
    RunCounter runs;
    std::vector<std::string> f;
    while (tr.Next(f)) {
      const uint64_t t = tr.Unsigned(f, tick);
      const std::size_t r = runs.Of(t);
      if (truth_of.size() <= r) {
        truth_of.resize(r + 1);
      }
      ReplayTruth row;
      row.base_valid = (tr.Unsigned(f, mask) & 1U) != 0;
      for (std::size_t i = 0; i < 7; ++i) {
        row.base_q[i] = tr.Double(f, bc[i]);
      }
      for (std::size_t i = 0; i < 6; ++i) {
        row.base_v[i] = tr.Double(f, bc[7 + i]);
      }
      row.fz.fill(kNaN);
      for (std::size_t c = 0; c < fzc.size() && c < kMaxEstimatorSlots; ++c) {
        row.fz[c] = tr.Double(f, fzc[c]);
      }
      truth_of[r][t] = row;
    }
  }

  // ── diag: the ticks, joined ───────────────────────────────────────────────
  const auto col = [&diag](const std::string& n) { return diag.Column(n); };
  const std::size_t c_tick = col("tick");
  const std::size_t c_count = col("tick_count");
  const std::size_t c_t = col("t_relative_s");
  const std::size_t c_status = col("source_status");
  const std::size_t c_seq = col("source_sequence");
  const std::size_t c_age = col("source_age_us");
  std::vector<std::size_t> c_v;
  for (const auto& j : log.roster) {
    c_v.push_back(col("v_" + j));
  }
  std::array<std::size_t, 6> c_imu{};
  const char* imu[] = {"imu_gx", "imu_gy", "imu_gz", "imu_ax", "imu_ay", "imu_az"};
  for (std::size_t i = 0; i < 6; ++i) {
    c_imu[i] = col(imu[i]);
  }
  const std::size_t c_ivalid = col("est_imu_valid");
  const std::size_t c_iseq = col("est_imu_sequence");
  const std::size_t c_iage = col("est_imu_age_us");
  const std::size_t c_skew = col("est_skew_us");
  const std::size_t c_planned = col("est_planned_mask");
  std::array<std::size_t, 13> c_base{};
  const char* base[] = {"base_px", "base_py", "base_pz", "base_qx", "base_qy", "base_qz", "base_qw",
                        "base_vx", "base_vy", "base_vz", "base_wx", "base_wy", "base_wz"};
  for (std::size_t i = 0; i < 13; ++i) {
    c_base[i] = col(base[i]);
  }
  const std::size_t c_init = col("est_initialized");
  const std::size_t c_reason = col("est_reset_reason");
  const std::size_t c_resets = col("est_reset_count");
  const std::size_t c_periods = col("est_periods");
  const std::size_t c_hold = col("est_hold_ticks");
  const std::size_t c_step = col("est_step_status");
  const std::size_t c_report = col("est_report_valid");
  const std::size_t c_mask = col("est_mask");
  const std::size_t c_mode_ctrl = col("mode");
  const std::size_t c_fault = col("fault_latched");
  std::vector<std::size_t> c_mode;
  std::vector<std::size_t> c_update;
  for (const auto& name : diag.header()) {
    if (name.rfind("est_mode_", 0) == 0) {
      c_mode.push_back(col(name));
      c_update.push_back(col("est_update_" + name.substr(9)));
    }
  }
  const bool use_truth = !truth_of.empty();
  std::vector<std::size_t> run_of;
  RunCounter runs;
  bool have_prev = false;
  std::size_t prev_run = 0;
  uint64_t prev_tick = 0;
  std::vector<std::string> f;
  while (diag.Next(f)) {
    ++log.diag_rows;
    ReplayInput in;
    in.tick = diag.Unsigned(f, c_tick);
    const std::size_t r = runs.Of(in.tick);
    const bool joined = r < q_of.size() && q_of[r].count(in.tick) != 0;
    if (!joined) {
      ++log.unmatched;
      continue;
    }
    if (have_prev && r == prev_run && in.tick > prev_tick + 1) {
      in.gap = in.tick - prev_tick - 1;
      ++log.gaps;
      log.missing_ticks += in.gap;
    }
    have_prev = true;
    prev_run = r;
    prev_tick = in.tick;
    in.tick_count = diag.Unsigned(f, c_count);
    in.t_relative_s = diag.Double(f, c_t);
    in.source_status = static_cast<uint8_t>(diag.Unsigned(f, c_status));
    in.joint_sequence = diag.Unsigned(f, c_seq);
    in.source_age_us = diag.Double(f, c_age);
    in.num_joints = static_cast<int>(nj);
    in.q = q_of[r].at(in.tick);
    for (std::size_t j = 0; j < nj; ++j) {
      in.qd[j] = diag.Double(f, c_v[j]);
    }
    for (std::size_t i = 0; i < 6; ++i) {
      in.imu[i] = diag.Double(f, c_imu[i]);
    }
    in.imu_valid = diag.Unsigned(f, c_ivalid) != 0;
    in.imu_sequence = diag.Unsigned(f, c_iseq);
    in.imu_age_us = diag.Double(f, c_iage);
    in.skew_us = diag.Double(f, c_skew);
    in.planned_mask = static_cast<uint32_t>(diag.Unsigned(f, c_planned));
    auto& o = in.online;
    for (std::size_t i = 0; i < 7; ++i) {
      o.base_q[i] = diag.Double(f, c_base[i]);
    }
    for (std::size_t i = 0; i < 6; ++i) {
      o.base_v[i] = diag.Double(f, c_base[7 + i]);
    }
    o.ctrl_mode = static_cast<uint8_t>(diag.Unsigned(f, c_mode_ctrl));
    o.fault_latched = diag.Unsigned(f, c_fault) != 0;
    o.initialized = diag.Unsigned(f, c_init) != 0;
    o.reset_reason = static_cast<uint8_t>(diag.Unsigned(f, c_reason));
    o.reset_count = static_cast<uint32_t>(diag.Unsigned(f, c_resets));
    o.periods = static_cast<int32_t>(diag.Double(f, c_periods));
    o.hold_ticks = static_cast<uint32_t>(diag.Unsigned(f, c_hold));
    o.step_status = static_cast<uint8_t>(diag.Unsigned(f, c_step));
    o.report_valid = diag.Unsigned(f, c_report) != 0;
    o.estimated_mask = static_cast<uint32_t>(diag.Unsigned(f, c_mask));
    for (std::size_t c = 0; c < c_mode.size() && c < kMaxEstimatorSlots; ++c) {
      o.mode[c] = static_cast<uint8_t>(diag.Unsigned(f, c_mode[c]));
      o.update[c] = static_cast<uint8_t>(diag.Unsigned(f, c_update[c]));
    }
    if (use_truth) {
      const bool has = r < truth_of.size() && truth_of[r].count(in.tick) != 0;
      log.truth.push_back(has ? std::optional<ReplayTruth>(truth_of[r].at(in.tick)) : std::nullopt);
    }
    log.ticks.push_back(in);
    run_of.push_back(r);
  }
  log.runs = runs.any ? runs.run + 1 : 0;
  if (q_of.size() != log.runs) {
    throw std::runtime_error(joint_ff_path + ": " + std::to_string(q_of.size()) +
                             " runs against the diagnostic's " + std::to_string(log.runs) +
                             " — the joint rows cannot be paired");
  }
  if (use_truth && truth_of.size() != log.runs) {
    throw std::runtime_error(truth_path + ": " + std::to_string(truth_of.size()) +
                             " runs against the diagnostic's " + std::to_string(log.runs) +
                             " — the truth rows cannot be paired (--no-truth replays without)");
  }
  // Keep one piece: its gap counts are taken again over it (diag_rows and
  // unmatched stay the file's).
  log.run = run == kLastRun ? static_cast<int>(log.runs) - 1 : run;
  if (run != kAllRuns && log.runs > 0) {
    if (log.run < 0 || static_cast<std::size_t>(log.run) >= log.runs) {
      throw std::runtime_error(diag_path + ": no run " + std::to_string(run) + " (it has " +
                               std::to_string(log.runs) + ")");
    }
    const auto keep = static_cast<std::size_t>(log.run);
    std::vector<ReplayInput> ticks;
    std::vector<std::optional<ReplayTruth>> truth;
    log.gaps = 0;
    log.missing_ticks = 0;
    for (std::size_t i = 0; i < log.ticks.size(); ++i) {
      if (run_of[i] != keep) {
        continue;
      }
      ReplayInput in = log.ticks[i];
      if (ticks.empty()) {
        in.gap = 0;
      }
      log.gaps += in.gap > 0 ? 1 : 0;
      log.missing_ticks += in.gap;
      ticks.push_back(in);
      if (use_truth) {
        truth.push_back(log.truth[i]);
      }
    }
    log.ticks = std::move(ticks);
    log.truth = std::move(truth);
  }
  return log;
}

// ── the replay CSV ──────────────────────────────────────────────────────────

void WriteReplayHeader(std::ostream& os, const std::vector<std::string>& contact_names) {
  os << "tick,tick_count,t_relative_s,gap,joint_exact,valid,status,state_epoch,fresh_mask,"
        "valid_mask,contact_mask,initialized,reset_reason,reset_count,periods,hold_ticks,"
        "step_status,core_status,report_valid,planned_mask,est_mask,no_anchor_ticks,gated_ticks,"
        "refused_ticks,base_px,base_py,base_pz,base_qx,base_qy,base_qz,base_qw,base_vx,base_vy,"
        "base_vz,base_wx,base_wy,base_wz,imu_qx,imu_qy,imu_qz,imu_qw,imu_px,imu_py,imu_pz,"
        "imu_vx,imu_vy,imu_vz,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z,in_imu_valid,in_imu_gx,in_imu_gy,"
        "in_imu_gz,in_imu_ax,in_imu_ay,in_imu_az";
  for (const char* field : {"mode", "reason", "update", "evidence", "speed", "m2"}) {
    for (const auto& c : contact_names) {
      os << ',' << field << '_' << c;
    }
  }
  for (int i = 0; i < 6; ++i) {
    for (int j = i; j < 6; ++j) {
      os << ",P_" << i << '_' << j;
    }
  }
  for (std::size_t i = 0; i < kEstimatorCovDiagBase + 3 * contact_names.size(); ++i) {
    os << ",p_diag_" << i;
  }
  os << ",online_mode,online_fault_latched,online_base_px,online_base_py,online_base_pz,online_"
        "base_qx,online_base_qy,"
        "online_base_qz,online_base_qw,online_base_vx,online_base_vy,online_base_vz,"
        "online_base_wx,online_base_wy,online_base_wz,online_initialized,online_reset_reason,"
        "online_reset_count,online_periods,online_hold_ticks,online_step_status,"
        "online_report_valid,online_est_mask";
  for (const char* field : {"online_mode", "online_update"}) {
    for (const auto& c : contact_names) {
      os << ',' << field << '_' << c;
    }
  }
  os << ",truth_valid,truth_px,truth_py,truth_pz,truth_qx,truth_qy,truth_qz,truth_qw,"
        "truth_vx,truth_vy,truth_vz,truth_wx,truth_wy,truth_wz";
  for (const auto& c : contact_names) {
    os << ",truth_fz_" << c;
  }
  for (const auto& c : contact_names) {
    for (const char* a : kXyz) {
      os << ",truth_v_" << c << '_' << a;
    }
  }
}

void WriteReplayRow(std::ostream& os, const ReplayInput& in, const ReplayTruth* truth,
                    const EstimatorReplay& replay) {
  const MeasuredRobotState& out = replay.last();
  const EstimatorTelemetry& e = out.estimator;
  const auto nc = static_cast<std::size_t>(replay.num_contacts());
  os << in.tick << ',' << in.tick_count << ',' << in.t_relative_s << ',' << in.gap << ','
     << (replay.last_joint_exact() ? 1 : 0) << ',' << (out.valid ? 1 : 0) << ','
     << static_cast<unsigned>(out.status) << ',' << out.state_epoch << ',' << out.fresh_mask << ','
     << out.valid_mask << ',' << out.contact_mask << ',' << (e.initialized ? 1 : 0) << ','
     << static_cast<unsigned>(e.reset_reason) << ',' << e.reset_count << ',' << e.periods << ','
     << e.hold_ticks << ',' << static_cast<unsigned>(e.step_status) << ','
     << static_cast<unsigned>(e.core_status) << ',' << (e.report_valid ? 1 : 0) << ','
     << e.planned_mask << ',' << e.estimated_mask << ',' << e.no_anchor_ticks << ','
     << e.gated_ticks << ',' << e.refused_ticks;
  for (std::size_t i = 0; i < 7; ++i) {
    os << ',' << out.q[i];
  }
  for (std::size_t i = 0; i < 6; ++i) {
    os << ',' << out.v[i];
  }
  // The filter's own state (IMU point, world): NaN until it has started.
  const est::ContactInekf& f = replay.source().Sequencer().core();
  const bool on = e.initialized;
  const Eigen::Quaterniond quat = on ? f.orientation() : Eigen::Quaterniond(kNaN, kNaN, kNaN, kNaN);
  os << ',' << quat.x() << ',' << quat.y() << ',' << quat.z() << ',' << quat.w();
  for (const Eigen::Vector3d* x : {&f.position(), &f.velocity(), &f.gyro_bias(), &f.accel_bias()}) {
    for (int i = 0; i < 3; ++i) {
      os << ',' << (on ? (*x)(i) : kNaN);
    }
  }
  os << ',' << (in.imu_valid ? 1 : 0);
  for (double x : in.imu) {
    os << ',' << x;
  }
  for (const auto* field : {&e.mode, &e.reason, &e.update}) {
    for (std::size_t c = 0; c < nc; ++c) {
      os << ',' << static_cast<unsigned>((*field)[c]);
    }
  }
  for (const auto* field : {&e.evidence, &e.speed, &e.m2}) {
    for (std::size_t c = 0; c < nc; ++c) {
      os << ',' << (*field)[c];
    }
  }
  const auto& P = f.covariance();
  for (int i = 0; i < 6; ++i) {
    for (int j = i; j < 6; ++j) {
      os << ',' << (on ? P(i, j) : kNaN);
    }
  }
  for (std::size_t i = 0; i < kEstimatorCovDiagBase + 3 * nc; ++i) {
    os << ',' << e.p_diag[i];
  }
  const auto& o = in.online;
  os << ',' << static_cast<unsigned>(o.ctrl_mode) << ',' << (o.fault_latched ? 1 : 0);
  for (double x : o.base_q) {
    os << ',' << x;
  }
  for (double x : o.base_v) {
    os << ',' << x;
  }
  os << ',' << (o.initialized ? 1 : 0) << ',' << static_cast<unsigned>(o.reset_reason) << ','
     << o.reset_count << ',' << o.periods << ',' << o.hold_ticks << ','
     << static_cast<unsigned>(o.step_status) << ',' << (o.report_valid ? 1 : 0) << ','
     << o.estimated_mask;
  for (const auto* field : {&o.mode, &o.update}) {
    for (std::size_t c = 0; c < nc; ++c) {
      os << ',' << static_cast<unsigned>((*field)[c]);
    }
  }
  const bool tv = truth != nullptr && truth->base_valid;
  os << ',' << (tv ? 1 : 0);
  for (std::size_t i = 0; i < 7; ++i) {
    os << ',' << (tv ? truth->base_q[i] : kNaN);
  }
  for (std::size_t i = 0; i < 6; ++i) {
    os << ',' << (tv ? truth->base_v[i] : kNaN);
  }
  for (std::size_t c = 0; c < nc; ++c) {
    os << ',' << (truth != nullptr ? truth->fz[c] : kNaN);
  }
  const auto& vel = replay.truth_contact_velocity();
  for (std::size_t c = 0; c < nc; ++c) {
    for (std::size_t a = 0; a < 3; ++a) {
      os << ',' << vel[c][a];
    }
  }
}

}  // namespace integrated_bringup::locomotion
