// ── `catch_gate_batch` — offline gate judge for the S3.5b map ───────────────
//
// Reads kinematically accepted catch candidates (posture included) and runs the
// runtime gate functions of `time_feasibility.hpp` on each — reach time, γ
// window, stopping point — writing one CSV row per candidate. The python side
// (`rtc_tools.analysis.catch_gate_map`) owns everything else; the verdict is
// this binary's, so the map and the runtime planner share one judgement.
//
// ARCH-7-exempt: an offline inspection tool in the sense of
// design-principles.md §"ARCH-7 의 범위" — it knows no robot and no model (every
// limit and constant comes from argv), owns no RT loop and no ROS node, and
// appears in no launch file or bringup chain.
#include "rtc_controllers/catching/catch_gate_batch.hpp"
#include "rtc_controllers/catching/catch_pose_ik_batch.hpp"

#include <algorithm>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr std::string_view kUsage =
    R"(catch_gate_batch — offline gate judge (dynamic_catching S3.5b, plan §11)

Every option is required; none has a default, because a gate fed a guessed
constant still writes a complete map.

  --candidates PATH     CSV: id[,seed_id],t_c_s,p_c_{x,y,z},v_{x,y,z},jpu_{x,y,z},
                        qs0..,qu0..  (q*, the DLS unit-speed joint velocity and
                        what it achieves; MODEL WORLD coordinates). '-' = stdin
  --seeds PATH          wait poses: seed_id,q0,...
  --out PATH            output CSV ('-' = stdout)
  --qdot-max "w0 w1 …"  joint velocity limits [rad/s], model joint order
  --qddot-max "a0 a1 …" joint acceleration box [rad/s^2] (plan §9)
  --eta-v X             planner.gamma.eta_v, applied to the joint limits and v_max
  --v-max X             reference.v_max [m/s]
  --d-eff X             planner.hand.d_eff [m]
  --t-close-total X     T_close,e2e + h/2 [s]
  --gamma-margin X      planner.gamma.margin [m/s]
  --a-dec X             supervisor.decel.a_dec [m/s^2]
  --first-plan-s X      when the first plan exists, from release: T_det + L [s]
  --t-arm-s X           arm command delay T_arm [s]
  --t-margin-s X        planner.time.margin [s]
  -h, --help            this text
)";

[[noreturn]] void Die(const std::string& msg) {
  std::cerr << "catch_gate_batch: " << msg << '\n';
  std::exit(2);
}

[[nodiscard]] double Number(const std::string& text, std::string_view flag) {
  try {
    std::size_t used = 0;
    const double v = std::stod(text, &used);
    if (used != text.size()) {
      throw std::invalid_argument("trailing characters");
    }
    return v;
  } catch (const std::exception&) {
    Die(std::string(flag) + ": '" + text + "' is not a number");
  }
}

[[nodiscard]] std::vector<double> NumberList(const std::string& text, std::string_view flag) {
  std::vector<double> out;
  std::istringstream ss(text);
  std::string cell;
  while (ss >> cell) {
    out.push_back(Number(cell, flag));
  }
  return out;
}

struct Args {
  std::string candidates;
  std::string seeds;
  std::string out;
  rtc::catching::GateSettings settings;
};

[[nodiscard]] Args ParseArgs(int argc, char** argv) {
  Args a;
  std::map<std::string, std::string> raw;
  const std::vector<std::string> flags = {
      "--candidates", "--seeds",        "--out",     "--qdot-max",      "--qddot-max",
      "--eta-v",      "--v-max",        "--d-eff",   "--t-close-total", "--gamma-margin",
      "--a-dec",      "--first-plan-s", "--t-arm-s", "--t-margin-s"};
  for (int i = 1; i < argc; ++i) {
    const std::string f = argv[i];
    if (f == "-h" || f == "--help") {
      std::cout << kUsage;
      std::exit(0);
    }
    if (std::find(flags.begin(), flags.end(), f) == flags.end()) {
      Die("unknown argument '" + f + "' (try --help)");
    }
    if (i + 1 >= argc) {
      Die(f + " needs a value");
    }
    raw[f] = argv[++i];
  }
  for (const std::string& f : flags) {
    if (raw.find(f) == raw.end()) {
      Die(f + " is required (try --help)");
    }
  }
  a.candidates = raw["--candidates"];
  a.seeds = raw["--seeds"];
  a.out = raw["--out"];
  auto& s = a.settings;
  s.qdot_max = NumberList(raw["--qdot-max"], "--qdot-max");
  s.qddot_max = NumberList(raw["--qddot-max"], "--qddot-max");
  s.eta_v = Number(raw["--eta-v"], "--eta-v");
  s.v_max = Number(raw["--v-max"], "--v-max");
  s.d_eff = Number(raw["--d-eff"], "--d-eff");
  s.t_close_total = Number(raw["--t-close-total"], "--t-close-total");
  s.gamma_margin = Number(raw["--gamma-margin"], "--gamma-margin");
  s.a_dec = Number(raw["--a-dec"], "--a-dec");
  s.first_plan_s = Number(raw["--first-plan-s"], "--first-plan-s");
  s.t_arm_s = Number(raw["--t-arm-s"], "--t-arm-s");
  s.t_margin_s = Number(raw["--t-margin-s"], "--t-margin-s");
  return a;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args args = ParseArgs(argc, argv);

    std::ifstream seed_file(args.seeds);
    if (!seed_file) {
      Die("cannot open seeds '" + args.seeds + "'");
    }
    const auto seeds = rtc::catching::ParseSeedCsv(seed_file);
    const int nv = static_cast<int>(seeds.begin()->second.size());

    std::vector<rtc::catching::GateCandidate> candidates;
    if (args.candidates == "-") {
      candidates = rtc::catching::ParseGateCandidateCsv(std::cin, nv);
    } else {
      std::ifstream in(args.candidates);
      if (!in) {
        Die("cannot open candidates '" + args.candidates + "'");
      }
      candidates = rtc::catching::ParseGateCandidateCsv(in, nv);
    }

    const auto rows = rtc::catching::RunGateBatch(candidates, seeds, args.settings);

    std::ofstream out_file;
    if (args.out != "-") {
      out_file.open(args.out);
      if (!out_file) {
        Die("cannot write '" + args.out + "'");
      }
    }
    std::ostream& out = args.out == "-" ? std::cout : out_file;
    out << rtc::catching::GateCsvHeader() << '\n';
    for (const auto& row : rows) {
      out << rtc::catching::GateCsvRow(row) << '\n';
    }
    out.flush();
    if (!out) {
      Die("write failed");
    }
    return 0;
  } catch (const std::exception& e) {
    Die(e.what());
  }
}
