// ── test_unified_kindyn_bench — Phase 0 RT-cost measurement ──────────────────
//   Controller-unification go/no-go input. The plan replaces the three
//   per-controller kin/dyn paths (joint = arm-only FK, task = arm-only Jacobian,
//   wbc = full PinocchioCache) with ONE call:
//     rtc::tsid::PinocchioCache::Update(q, v)  on the COMBINED
//     arm+hand control model (computeAllTerms + gravity + per-frame Jacobian).
//   "Always compute full dynamics" is only viable if the extra cost fits the RT
//   tick budget. This harness measures, on two real robot models, the per-call
//   latency delta between what joint/task pay today (baseline, arm-only reduced
//   model) and the unified full-compute (combined model).
//
//   #175 Phase 0 (closed-chain robots only) adds a second, independent question
//   on the same harness: what does ONE RtClosedChainHandle K-step projection
//   cost? WBC runs that projection twice per tick today (see the block guarded by
//   bc.extended below), so the "closed-proj-kin" median IS the saving a 2→1
//   unification would buy — the go/no-go input for that issue.
//
//   ⚠ Absolute numbers are machine-dependent (build flags, load, affinity) —
//   REPORT ONLY, no hard timing asserts (mirrors
//   rtc_urdf_bridge/test/test_closed_chain_fk_measurement.cpp::UpdateComputeTime,
//   which ends in SUCCEED()). The only hard asserts are path guards, not
//   thresholds: model nq/nv (confirms the model path) and held==0 on the closed
//   chain (confirms the timed ticks really ran the projection, not a hold
//   short-circuit).
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
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
#include "rtc_urdf_bridge/rt_closed_chain_handle.hpp"
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
  const char* urdf_rel;
  bool extended;
  const char* closure_rel;                    // share-relative; only used when extended
  const char* arm_submodel;                   // GetReducedModel(name) — baseline arm-only model
  const char* arm_tip_frame;                  // ee / flange frame for the frame Jacobian
  const char* root_link;                      // arm sub_model + wbc tree root link (shipped config)
  std::array<const char*, 4> wbc_fingertips;  // wbc control-tree tip frames
  int expected_nq;
  int expected_nv;
};

// Every fixture model lives in `robot_descriptions` — the package name is a
// literal, not a config field, so validate_test_fixtures.py can prove where
// this points instead of only checking that a skip guard exists (#457).
rub::ModelConfig MakeModelConfig(const BenchConfig& bc) {
  rub::ModelConfig cfg;
  cfg.urdf_path =
      ament_index_cpp::get_package_share_directory("robot_descriptions") + "/" + bc.urdf_rel;
  cfg.root_joint_type = "fixed";
  if (bc.extended) {
    cfg.closure_yaml_path =
        ament_index_cpp::get_package_share_directory("robot_descriptions") + "/" + bc.closure_rel;
  }
  // Only URDF/closure paths + the fixed root joint are set here; the arm
  // sub_model and wbc control tree (the names/roots the benchmark consumes) are
  // wired by the caller in RunBench. PinocchioModelBuilder derives passive/mimic/
  // loop topology from the URDF + closure sidecar exactly as at bring-up.
  return cfg;
}

void RunBench(const BenchConfig& bc) {
  std::printf("\n════════════════════════════════════════════════════════════════\n");
  std::printf("[config] %s  (urdf=%s, extended=%d)\n", bc.label, bc.urdf_rel,
              static_cast<int>(bc.extended));

  rub::ModelConfig cfg = MakeModelConfig(bc);
  // sub_models / tree_models the builder needs for GetReducedModel / GetTreeModel.
  // Root/tip links come straight from the shipped integrated_bringup config,
  // carried explicitly on BenchConfig (root_link / arm_tip_frame / wbc_fingertips).
  cfg.sub_models.push_back({bc.arm_submodel, bc.root_link, bc.arm_tip_frame});

  // wbc control tree (arm root → 4 fingertips), mirrors the shipped config.
  cfg.tree_models.push_back(
      {"wbc",
       bc.root_link,
       {bc.wbc_fingertips[0], bc.wbc_fingertips[1], bc.wbc_fingertips[2], bc.wbc_fingertips[3]}});

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

  // Model-path guard: closed-chain (extended) robots must expose a non-null
  // actuated model; serial/mimic robots must fall through to the wbc tree.
  // Without this the benchmark could silently time the wrong combined model.
  ASSERT_EQ(actuated_null, !bc.extended)
      << bc.label << ": GetActuatedModel() path mismatch (extended=" << bc.extended << ")";

  // Model-path confirmation (the other hard assert — nq/nv is deterministic).
  EXPECT_EQ(combined->nq, bc.expected_nq) << bc.label << ": combined model nq";
  EXPECT_EQ(combined->nv, bc.expected_nv) << bc.label << ": combined model nv";

  // ── PinocchioCache on the combined model (empty contacts, 1 arm-tip frame) ──
  rtc::tsid::PinocchioCache cache;
  rtc::tsid::ContactManagerConfig contact_cfg;  // max_contacts = 0 (default)
  cache.Init(combined, rtc::tsid::ContactFrameIds(contact_cfg));

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

  // ── #175 Phase 0: one RtClosedChainHandle K-step projection (closed chain) ───
  //   WBC pays this projection TWICE per tick on a loop-closed robot. Both owners
  //   build their handle from the SAME four closure inputs with the same default
  //   K/λ/seed-clamp (wbc/controller.cpp:214-217 vs :238-241), so one handle here
  //   is representative of either:
  //     - WbcReducedDynamicsProvider — Update(q_a) + UpdateDynamics(v_a)
  //       (wbc_reduced_dynamics_provider.cpp:116-127) → M_a/h_a/g_a + contact J/oMf
  //     - ClosedChainHandFk          — Update(q_a) only
  //       (closed_chain_hand_fk.cpp:156)              → fingertip pose
  //   Unification removes exactly one Update(); UpdateDynamics() is already called
  //   once. Hence "closed-proj-kin" = the 2→1 saving, "closed-proj+dyn" = what the
  //   surviving unified handle still costs per tick.
  bool proj_measured = false;
  Stats s_proj_kin;
  Stats s_proj_dyn;
  if (bc.extended) {
    rub::RtClosedChainHandle handle(builder->GetFullModel(), builder->GetConstraintModels(),
                                    builder->GetClosureActuatedJointIds(),
                                    builder->GetClosureReferenceConfig());
    const pinocchio::Model& full = handle.GetModel();
    const int n_a = handle.nv_independent();
    ASSERT_GT(n_a, 0) << bc.label << ": closure exposes no independent joints";
    std::printf("[closure] full: nq=%d nv=%d  n_a=%d  m=%d (constraint rows)\n", full.nq, full.nv,
                n_a, handle.constraint_dim());

    // q_a starts at the closure reference configuration (the handle's warm-start
    // seed), restricted to the independent joints — so the first projection starts
    // loop-consistent instead of walking in from neutral.
    const std::vector<std::string> indep_names = handle.GetIndependentJointNames();
    const Eigen::VectorXd& q_ref = builder->GetClosureReferenceConfig();
    std::vector<double> q_a(static_cast<std::size_t>(n_a), 0.0);
    if (q_ref.size() == full.nq) {
      for (std::size_t k = 0; k < indep_names.size(); ++k) {
        const auto jid = full.getJointId(indep_names[k]);
        q_a[k] = q_ref[full.joints[jid].idx_q()];
      }
    }
    // Non-zero v_a: UpdateDynamics takes the have_velocity branch on size alone,
    // but a moving robot is what the provider actually feeds (rnea + Jc central
    // difference drift), so keep the samples representative.
    const std::vector<double> v_a(static_cast<std::size_t>(n_a), 0.1);
    const std::span<const double> q_a_span(q_a.data(), q_a.size());
    const std::span<const double> v_a_span(v_a.data(), v_a.size());

    // Path guard, not a threshold: 1e-6 rad steps can never trip the seed clamp
    // (kDefaultActuatedIncrement = 0.05) or the passive-deviation guard (0.5), so
    // every timed tick must run the full K=2 projection. A held tick returns early
    // and would silently deflate the median we are about to report.
    int held_ticks = 0;
    s_proj_kin = Bench([&](int i) { q_a[0] += 1e-6 * static_cast<double>(1 + (i % 7)); },
                       [&]() {
                         if (handle.Update(q_a_span).held) {
                           ++held_ticks;
                         }
                       });
    sink += handle.GetFullConfiguration()(0);

    s_proj_dyn = Bench([&](int i) { q_a[0] += 1e-6 * static_cast<double>(1 + (i % 7)); },
                       [&]() {
                         const bool kin_held = handle.Update(q_a_span).held;
                         if (kin_held || handle.UpdateDynamics(v_a_span).held) {
                           ++held_ticks;
                         }
                       });
    sink += handle.GetMassMatrix()(0, 0);

    ASSERT_EQ(held_ticks, 0) << bc.label
                             << ": projection held on some ticks — timings below would not be "
                                "measuring the K-step projection";
    EXPECT_FALSE(handle.GetStatus().singular)
        << bc.label << ": projection reported singular — damped path, timings not representative";
    std::printf("[closure] closure_error after %d+%d projections = %.3e (held=%d, singular=%d)\n",
                kWarm + kN, kWarm + kN, handle.GetStatus().closure_error, held_ticks,
                static_cast<int>(handle.GetStatus().singular));
    proj_measured = true;
  }
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

  if (proj_measured) {
    PrintRow("closed-proj-kin", s_proj_kin);
    PrintRow("closed-proj+dyn", s_proj_dyn);
    std::printf(
        "[budget] closed-proj-kin = %.3f us  →  %.2f%% of 500Hz(2000us) | %.2f%% of 1kHz(1000us)"
        "  [#175: per-tick saving of 2→1]\n",
        s_proj_kin.median, 100.0 * s_proj_kin.median / kBudget500,
        100.0 * s_proj_kin.median / kBudget1k);
    std::printf(
        "[budget] closed-proj+dyn = %.3f us  →  %.2f%% of 500Hz(2000us) | %.2f%% of 1kHz(1000us)"
        "  [#175: cost the unified handle still pays]\n",
        s_proj_dyn.median, 100.0 * s_proj_dyn.median / kBudget500,
        100.0 * s_proj_dyn.median / kBudget1k);
    std::printf("[delta]  closed-proj+dyn - closed-proj-kin (median) = %.3f us  [dynamics-only]\n",
                s_proj_dyn.median - s_proj_kin.median);
  }
}

}  // namespace

// iiwa7_leap — 23-DoF (7 iiwa7 + 16 LEAP), serial/mimic (NOT loop-closed).
//   Combined control model = wbc tree (GetActuatedModel() is null here).
TEST(UnifiedKinDynBench, Iiwa7Leap) {
  RunBench({/*label=*/"iiwa7_leap (23-DoF serial/mimic)",
            /*urdf_rel=*/"robots/iiwa7_leap/urdf/iiwa7_with_leap_right.urdf.xacro",
            /*extended=*/false,
            /*closure_rel=*/"",
            /*arm_submodel=*/"iiwa7",
            /*arm_tip_frame=*/"ee_link",
            /*root_link=*/"link_0",
            /*wbc_fingertips=*/
            {{"thumb_tip_head", "index_tip_head", "middle_tip_head", "ring_tip_head"}},
            /*expected_nq=*/23,
            /*expected_nv=*/23});
  SUCCEED();
}

// ur5e_p1b — 16-DoF (6 UR5e + 10 proto_1b), closed-chain (5-loop).
//   Combined control model = GetActuatedModel() (non-null; loop-passive locked).
TEST(UnifiedKinDynBench, Ur5eP1b) {
  RunBench({/*label=*/"ur5e_p1b (16-DoF closed-chain)",
            /*urdf_rel=*/"robots/ur5e_p1b/urdf/ur5e_with_proto_1b.urdf.xacro",
            /*extended=*/true,
            /*closure_rel=*/"robots/ur5e_p1b/urdf/ur5e_with_proto_1b.closure.yaml",
            /*arm_submodel=*/"ur5e",
            /*arm_tip_frame=*/"tool0",
            /*root_link=*/"base",
            /*wbc_fingertips=*/
            {{"l_thumb_tip_bracket", "l_index_tip_bracket", "l_middle_tip_bracket",
              "l_ring_tip_bracket"}},
            /*expected_nq=*/16,
            /*expected_nv=*/16});
  SUCCEED();
}
