// ── test_unified_kindyn_bench — Phase 0 RT-cost measurement ──────────────────
//   Controller-unification go/no-go input. The plan replaces the three
//   per-controller kin/dyn paths (joint = arm-only FK, task = arm-only Jacobian,
//   wbc = full PinocchioCache) with ONE call:
//     rtc::tsid::PinocchioCache::Update(q, v, contacts)  on the COMBINED
//     arm+hand control model (computeAllTerms + gravity + per-frame Jacobian).
//   "Always compute full dynamics" is only viable if the extra cost fits the RT
//   tick budget. This harness measures, on two real robot models, the per-call
//   latency delta between what joint/task pay today (baseline, arm-only reduced
//   model) and the unified full-compute (combined model).
//
//   ⚠ Absolute numbers are machine-dependent (build flags, load, affinity) —
//   REPORT ONLY, no hard timing asserts (mirrors
//   rtc_urdf_bridge/test/test_closed_chain_fk_measurement.cpp::UpdateComputeTime,
//   which ends in SUCCEED()). Only model nq/nv is asserted (confirms model path).
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numeric>
#include <span>
#include <string>
#include <vector>

// Pinocchio 헤더 (경고 억제)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop

#include "rtc_tsid/types/wbc_types.hpp"
#include "rtc_urdf_bridge/pinocchio_model_builder.hpp"
#include "rtc_urdf_bridge/rt_model_handle.hpp"
#include "rtc_urdf_bridge/types.hpp"

namespace rub = rtc_urdf_bridge;

namespace {

constexpr int kWarm = 50;              // ≥20 warm-up
constexpr int kN = 5000;               // ≥2000 measured iterations
constexpr double kBudget500 = 2000.0;  // µs @ 500 Hz
constexpr double kBudget1k = 1000.0;   // µs @ 1 kHz

struct Stats {
  std::size_t n{0};
  double min{0}, mean{0}, median{0}, p95{0}, p99{0}, max{0};
};

// 정렬된 μs 샘플에서 percentile(nearest-rank, 선형 보간 없음).
double PercentileUs(const std::vector<double>& sorted, double pct) {
  if (sorted.empty()) {
    return 0.0;
  }
  const auto n = static_cast<double>(sorted.size());
  auto idx = static_cast<std::size_t>(std::ceil(pct / 100.0 * n)) - 1;
  idx = std::min(idx, sorted.size() - 1);
  return sorted[idx];
}

Stats Summarize(std::vector<double>& us) {
  Stats s;
  s.n = us.size();
  if (us.empty()) {
    return s;
  }
  std::sort(us.begin(), us.end());
  s.min = us.front();
  s.max = us.back();
  s.mean = std::accumulate(us.begin(), us.end(), 0.0) / static_cast<double>(us.size());
  s.median = PercentileUs(us, 50);
  s.p95 = PercentileUs(us, 95);
  s.p99 = PercentileUs(us, 99);
  return s;
}

// pre(i): 타이밍 밖에서 q 를 살짝 변화(캐싱 아티팩트 방지). meas(): 타이밍 대상 호출.
template <typename PreFn, typename MeasFn>
Stats Bench(PreFn&& pre, MeasFn&& meas) {
  for (int i = 0; i < kWarm; ++i) {
    pre(i);
    meas();
  }
  std::vector<double> us;
  us.reserve(kN);
  for (int i = 0; i < kN; ++i) {
    pre(i);
    const auto t0 = std::chrono::steady_clock::now();
    meas();
    const auto t1 = std::chrono::steady_clock::now();
    us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
  }
  return Summarize(us);
}

void PrintRow(const char* label, const Stats& s) {
  std::printf(
      "[timing] %-16s n=%zu  min=%.3f  mean=%.3f  median=%.3f  p95=%.3f  p99=%.3f  max=%.3f (us)\n",
      label, s.n, s.min, s.mean, s.median, s.p95, s.p99, s.max);
}

struct BenchConfig {
  const char* label;
  const char* urdf_pkg;
  const char* urdf_rel;
  bool extended;
  const char* closure_rel;    // pkg-relative; only used when extended
  const char* arm_submodel;   // GetReducedModel(name) — baseline arm-only model
  const char* arm_tip_frame;  // ee / flange frame for the frame Jacobian
  int expected_nq;
  int expected_nv;
};

rub::ModelConfig MakeModelConfig(const BenchConfig& bc) {
  rub::ModelConfig cfg;
  cfg.urdf_path = ament_index_cpp::get_package_share_directory(bc.urdf_pkg) + "/" + bc.urdf_rel;
  cfg.root_joint_type = "fixed";
  if (bc.extended) {
    cfg.closure_yaml_path =
        ament_index_cpp::get_package_share_directory(bc.urdf_pkg) + "/" + bc.closure_rel;
  }
  // Mirror integrated_bringup config: arm sub_model + wbc control tree.
  // (Only the names/roots the benchmark actually consumes are populated;
  //  PinocchioModelBuilder derives passive/mimic/loop topology from the URDF +
  //  closure sidecar exactly as at bring-up.)
  return cfg;
}

void RunBench(const BenchConfig& bc) {
  std::printf("\n════════════════════════════════════════════════════════════════\n");
  std::printf("[config] %s  (urdf=%s, extended=%d)\n", bc.label, bc.urdf_rel,
              static_cast<int>(bc.extended));

  rub::ModelConfig cfg = MakeModelConfig(bc);
  // sub_models / tree_models the builder needs for GetReducedModel / GetTreeModel.
  cfg.sub_models.push_back({bc.arm_submodel, "", ""});  // root/tip filled below per-robot
  // Note: the reduced arm model and wbc tree require their root/tip links. We set
  // them from the shipped config values passed via BenchConfig-adjacent literals.

  // Root/tip links come straight from the shipped integrated_bringup config.
  // iiwa7: link_0 → ee_link ; ur5e: base → tool0.
  cfg.sub_models.back().root_link = bc.expected_nq == 23 ? "link_0" : "base";
  cfg.sub_models.back().tip_link = bc.arm_tip_frame;

  // wbc control tree (arm root → 4 fingertips), mirrors the shipped config.
  if (bc.expected_nq == 23) {
    cfg.tree_models.push_back(
        {"wbc",
         "link_0",
         {"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"}});
  } else {
    cfg.tree_models.push_back({"wbc",
                               "base",
                               {"l_thumb_tip_bracket", "l_index_tip_bracket",
                                "l_middle_tip_bracket", "l_ring_tip_bracket"}});
  }

  auto builder = std::make_shared<rub::PinocchioModelBuilder>(cfg);

  // ── Baseline arm-only reduced model (what joint/task controllers use today) ──
  auto arm_model = builder->GetReducedModel(bc.arm_submodel);
  rub::RtModelHandle arm_handle(arm_model);
  const int arm_nq = arm_handle.nq();
  const int arm_nv = arm_handle.nv();
  const pinocchio::FrameIndex arm_tip_fid = arm_handle.GetFrameId(bc.arm_tip_frame);
  ASSERT_NE(arm_tip_fid, 0u) << bc.label << ": arm tip frame '" << bc.arm_tip_frame
                             << "' not found in reduced arm model";

  // ── Combined arm+hand control model (unified full-compute target) ────────────
  std::shared_ptr<const pinocchio::Model> combined;
  bool actuated_null = false;
  if (auto actuated = builder->GetActuatedModel()) {
    combined = std::move(actuated);  // closed-chain robots (ur5e_p1b)
  } else {
    combined = builder->GetTreeModel("wbc");  // serial/mimic robots (iiwa7_leap)
    actuated_null = true;
  }

  std::printf(
      "[model]  arm-only(%s): nq=%d nv=%d  |  combined: nq=%d nv=%d  GetActuatedModel()=%s\n",
      bc.arm_submodel, arm_nq, arm_nv, combined->nq, combined->nv,
      actuated_null ? "null → GetTreeModel(\"wbc\")" : "non-null (closed-chain)");

  // Model-path confirmation (the only hard assert — nq/nv is deterministic).
  EXPECT_EQ(combined->nq, bc.expected_nq) << bc.label << ": combined model nq";
  EXPECT_EQ(combined->nv, bc.expected_nv) << bc.label << ": combined model nv";

  // ── PinocchioCache on the combined model (empty contacts, 1 arm-tip frame) ──
  rtc::tsid::PinocchioCache cache;
  rtc::tsid::ContactManagerConfig contact_cfg;  // max_contacts = 0 (default)
  cache.Init(combined, rtc::tsid::ContactFrameIds(contact_cfg));
  rtc::tsid::ContactState contacts;
  contacts.Init(0);

  // Register the arm tip frame on the COMBINED model so a frame Jacobian is part
  // of the unified Update (spec: ~1-2 registered frames). Fall back to the first
  // wbc fingertip if the arm-tip frame name is absent from the combined model.
  pinocchio::FrameIndex combined_tip_fid = 0;
  std::string registered_name = bc.arm_tip_frame;
  if (combined->existFrame(bc.arm_tip_frame)) {
    combined_tip_fid = combined->getFrameId(bc.arm_tip_frame);
  } else {
    registered_name = cfg.tree_models.back().tip_links.front();
    combined_tip_fid = combined->getFrameId(registered_name);
  }
  cache.RegisterFrame(registered_name, combined_tip_fid);
  std::printf("[frame]  registered '%s' (fid=%zu) on combined model\n", registered_name.c_str(),
              combined_tip_fid);

  // ── Working buffers ──────────────────────────────────────────────────────────
  Eigen::VectorXd q_arm = pinocchio::neutral(arm_handle.GetModel());
  Eigen::VectorXd q_comb = pinocchio::neutral(*combined);
  Eigen::VectorXd v_comb = Eigen::VectorXd::Zero(combined->nv);
  Eigen::MatrixXd J_arm(6, arm_nv);
  volatile double sink = 0.0;

  const std::span<const double> q_arm_span(q_arm.data(), static_cast<std::size_t>(q_arm.size()));

  // ── Baseline-joint: forwardKinematics + updateFramePlacements (arm-only) ─────
  Stats s_joint = Bench([&](int i) { q_arm(0) += 1e-6 * static_cast<double>(1 + (i % 7)); },
                        [&]() { arm_handle.ComputeForwardKinematics(q_arm_span); });
  sink += arm_handle.GetFramePosition(arm_tip_fid).x();

  // ── Baseline-task: computeJointJacobians + updateFramePlacements +
  //                  getFrameJacobian(tip, LOCAL_WORLD_ALIGNED)  (arm-only) ─────
  Stats s_task =
      Bench([&](int i) { q_arm(0) += 1e-6 * static_cast<double>(1 + (i % 7)); },
            [&]() {
              arm_handle.ComputeJacobians(q_arm_span);
              arm_handle.GetFrameJacobian(arm_tip_fid, pinocchio::LOCAL_WORLD_ALIGNED, J_arm);
            });
  sink += J_arm(0, 0);

  // ── Unified: PinocchioCache::Update on the combined model ────────────────────
  Stats s_unified = Bench([&](int i) { q_comb(0) += 1e-6 * static_cast<double>(1 + (i % 7)); },
                          [&]() { cache.Update(q_comb, v_comb); });
  sink += cache.M(0, 0);
  (void)sink;

  // ── Report ───────────────────────────────────────────────────────────────────
  std::printf("[timing] --- %s (median-based) ---\n", bc.label);
  PrintRow("baseline-joint", s_joint);
  PrintRow("baseline-task", s_task);
  PrintRow("unified", s_unified);

  const double delta = s_unified.median - s_task.median;
  std::printf("[delta]  unified - baseline-task (median) = %.3f us\n", delta);
  std::printf(
      "[budget] unified median = %.3f us  →  %.2f%% of 500Hz(2000us) | %.2f%% of 1kHz(1000us)\n",
      s_unified.median, 100.0 * s_unified.median / kBudget500,
      100.0 * s_unified.median / kBudget1k);
  std::printf(
      "[budget] delta          = %.3f us  →  %.2f%% of 500Hz(2000us) | %.2f%% of 1kHz(1000us)\n",
      delta, 100.0 * delta / kBudget500, 100.0 * delta / kBudget1k);
  std::printf("[budget] baseline-task  = %.3f us  →  %.2f%% of 1kHz(1000us)\n", s_task.median,
              100.0 * s_task.median / kBudget1k);
}

}  // namespace

// iiwa7_leap — 23-DoF (7 iiwa7 + 16 LEAP), serial/mimic (NOT loop-closed).
//   Combined control model = wbc tree (GetActuatedModel() is null here).
TEST(UnifiedKinDynBench, Iiwa7Leap) {
  RunBench({/*label=*/"iiwa7_leap (23-DoF serial/mimic)",
            /*urdf_pkg=*/"robot_descriptions",
            /*urdf_rel=*/"robots/iiwa7_leap/urdf/iiwa7_with_leap_right.urdf.xacro",
            /*extended=*/false,
            /*closure_rel=*/"",
            /*arm_submodel=*/"iiwa7",
            /*arm_tip_frame=*/"ee_link",
            /*expected_nq=*/23,
            /*expected_nv=*/23});
  SUCCEED();
}

// ur5e_p1b — 16-DoF (6 UR5e + 10 proto_1b), closed-chain (5-loop).
//   Combined control model = GetActuatedModel() (non-null; loop-passive locked).
TEST(UnifiedKinDynBench, Ur5eP1b) {
  RunBench({/*label=*/"ur5e_p1b (16-DoF closed-chain)",
            /*urdf_pkg=*/"hand_description",
            /*urdf_rel=*/"robots/ur5e_p1b/urdf/ur5e_with_proto_1b.urdf.xacro",
            /*extended=*/true,
            /*closure_rel=*/"robots/ur5e_p1b/urdf/ur5e_with_proto_1b.closure.yaml",
            /*arm_submodel=*/"ur5e",
            /*arm_tip_frame=*/"tool0",
            /*expected_nq=*/16,
            /*expected_nv=*/16});
  SUCCEED();
}
