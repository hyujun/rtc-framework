#ifndef RTC_BASE_THREAD_CONFIG_HPP_
#define RTC_BASE_THREAD_CONFIG_HPP_

#include <sched.h>  // SCHED_FIFO, SCHED_OTHER, SCHED_RR

#include <array>

namespace rtc {

// Thread configuration for RT control and scheduling
struct ThreadConfig {
  int cpu_core;        // CPU affinity (0-based core index, -1 = no pinning)
  int sched_policy;    // SCHED_FIFO, SCHED_RR, or SCHED_OTHER
  int sched_priority;  // 1-99 for SCHED_FIFO/RR, ignored for OTHER
  int nice_value;      // -20 to 19 for SCHED_OTHER, ignored for FIFO/RR
  const char* name;    // Thread name for debugging (max 15 chars)
};

// ── MPC thread configuration ────────────────────────────────────────────────
// Holds the main MPC solve thread plus optional worker threads used by
// parallel solvers (e.g. Aligator's parallel rollout). Only the first
// `num_workers` entries of `workers` are valid.
//
// Invariants (checked by ValidateSystemThreadConfigs):
//   * `main.sched_priority` < rt_inbound thread priority (rt_inbound preempts MPC).
//   * Each worker `sched_priority` ≤ `main.sched_priority` (workers never
//     preempt the main solve).
//   * `0 ≤ num_workers ≤ 2` (matches 12-/16-core capacity).

inline constexpr int kMpcMaxWorkers = 2;

struct MpcThreadConfig {
  ThreadConfig main{};
  int num_workers{0};
  std::array<ThreadConfig, kMpcMaxWorkers> workers{};
};

// ── RT priority hierarchy (layout v3) ───────────────────────────────────────
//   90 (rt_control)  > 70 (rt_inbound) > 65 (rt_outbound) >
//   60 (mpc_main)    > 55 (mpc_workers)
//
// rt_inbound and rt_outbound share a single core (Core 3 on every tier
// except 4-core fallback). Priority diff (70 vs 65) guarantees rt_inbound
// preempts rt_outbound — no starvation. Hand-private UDP receive thread
// (priority 65) lives inside the hand_driver process and inherits affinity
// from the launch-level taskset, so it is NOT represented in SystemThreadConfigs.
//
// process-level threads (arm_driver, hand_driver, sim_thread, viewer) are
// SCHED_OTHER prio 0; only their cpu_core is consumed (taskset pin), all
// other fields are passed through ApplyThreadConfig as no-op.

// ── 6-core configuration (degraded mode — no deterministic RT guarantee) ────
// Core 0:  OS / DDS / IRQ + nrt_logging + nrt_callback
// Core 1:  arm_driver + hand_driver (shared, degraded)
// Core 2:  rt_control                       FIFO 90
// Core 3:  rt_inbound + rt_outbound         FIFO 70 / 65   ← v3 same-core
// Core 4:  mpc_main                         FIFO 60
// Core 5:  spare
//
// Trade-off: mpc_worker absent, arm/hand share Core 1. "degraded mode" label
// required — no deterministic RT guarantee.

inline const ThreadConfig kRtControlConfig{.cpu_core = 2,
                                           .sched_policy = SCHED_FIFO,
                                           .sched_priority = 90,
                                           .nice_value = 0,
                                           .name = "rt_control"};

inline const ThreadConfig kRtInboundConfig{.cpu_core = 3,
                                           .sched_policy = SCHED_FIFO,
                                           .sched_priority = 70,
                                           .nice_value = 0,
                                           .name = "rt_inbound"};

inline const ThreadConfig kRtOutboundConfig{.cpu_core = 3,
                                            .sched_policy = SCHED_FIFO,
                                            .sched_priority = 65,
                                            .nice_value = 0,
                                            .name = "rt_outbound"};

inline const ThreadConfig kNrtLoggingConfig{.cpu_core = 0,
                                            .sched_policy = SCHED_OTHER,
                                            .sched_priority = 0,
                                            .nice_value = -5,
                                            .name = "nrt_logging"};

inline const ThreadConfig kNrtCallbackConfig{.cpu_core = 0,
                                             .sched_policy = SCHED_OTHER,
                                             .sched_priority = 0,
                                             .nice_value = 0,
                                             .name = "nrt_callback"};

inline const ThreadConfig kArmDriverConfig{.cpu_core = 1,
                                           .sched_policy = SCHED_OTHER,
                                           .sched_priority = 0,
                                           .nice_value = 0,
                                           .name = "arm_driver"};

inline const ThreadConfig kHandDriverConfig{.cpu_core = 1,
                                            .sched_policy = SCHED_OTHER,
                                            .sched_priority = 0,
                                            .nice_value = 0,
                                            .name = "hand_driver"};

// sim_thread.cpu_core = -1 means caller-controlled (cpu_shield --sim releases
// the shield, MuJoCo physics may roam over freed cores under CFS).
inline const ThreadConfig kSimThreadConfig{.cpu_core = -1,
                                           .sched_policy = SCHED_OTHER,
                                           .sched_priority = 0,
                                           .nice_value = 0,
                                           .name = "sim_thread"};

inline const ThreadConfig kViewerConfig{.cpu_core = -1,
                                        .sched_policy = SCHED_OTHER,
                                        .sched_priority = 0,
                                        .nice_value = 0,
                                        .name = "viewer"};

inline const MpcThreadConfig kMpcConfig6Core{
    .main =
        ThreadConfig{
            .cpu_core = 4,
            .sched_policy = SCHED_FIFO,
            .sched_priority = 60,
            .nice_value = 0,
            .name = "mpc_main",
        },
    .num_workers = 0,
    .workers = {},
};

// ── 4-core fallback (degraded — no deterministic RT guarantee) ──────────────
// Core 0:   OS / DDS / IRQ + nrt_logging + nrt_callback + arm/hand_driver
// Core 1:   rt_control                     FIFO 90
// Core 2:   rt_inbound + rt_outbound       FIFO 70 / 65
// Core 3:   mpc_main (SCHED_OTHER nice -5 — degraded)

inline const ThreadConfig kRtControlConfig4Core{.cpu_core = 1,
                                                .sched_policy = SCHED_FIFO,
                                                .sched_priority = 90,
                                                .nice_value = 0,
                                                .name = "rt_control"};

inline const ThreadConfig kRtInboundConfig4Core{.cpu_core = 2,
                                                .sched_policy = SCHED_FIFO,
                                                .sched_priority = 70,
                                                .nice_value = 0,
                                                .name = "rt_inbound"};

// Phase 5 4-core: keep rt_outbound on SCHED_OTHER. The "rt_outbound.priority
// < rt_inbound.priority" invariant in ValidateSystemThreadConfigs is gated on
// both being RT, so CFS rt_outbound here is intentional and skipped by the
// gate. mpc_main is also CFS for the same reason.
inline const ThreadConfig kRtOutboundConfig4Core{.cpu_core = 2,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "rt_outbound"};

inline const ThreadConfig kNrtLoggingConfig4Core{.cpu_core = 0,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = -5,
                                                 .name = "nrt_logging"};

inline const ThreadConfig kNrtCallbackConfig4Core{.cpu_core = 0,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = 0,
                                                  .name = "nrt_callback"};

inline const ThreadConfig kArmDriverConfig4Core{.cpu_core = 0,
                                                .sched_policy = SCHED_OTHER,
                                                .sched_priority = 0,
                                                .nice_value = 0,
                                                .name = "arm_driver"};

inline const ThreadConfig kHandDriverConfig4Core{.cpu_core = 0,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "hand_driver"};

inline const ThreadConfig kSimThreadConfig4Core{.cpu_core = -1,
                                                .sched_policy = SCHED_OTHER,
                                                .sched_priority = 0,
                                                .nice_value = 0,
                                                .name = "sim_thread"};

inline const ThreadConfig kViewerConfig4Core{.cpu_core = -1,
                                             .sched_policy = SCHED_OTHER,
                                             .sched_priority = 0,
                                             .nice_value = 0,
                                             .name = "viewer"};

inline const MpcThreadConfig kMpcConfig4Core{
    .main =
        ThreadConfig{
            .cpu_core = 3,
            .sched_policy = SCHED_OTHER,
            .sched_priority = 0,
            .nice_value = -5,
            .name = "mpc_main",
        },
    .num_workers = 0,
    .workers = {},
};

// ── 8-core configuration (layout v3) ────────────────────────────────────────
// Core 0:  OS / DDS / IRQ + nrt_logging
// Core 1:  nrt_callback
// Core 2:  rt_control                       FIFO 90
// Core 3:  rt_inbound + rt_outbound         FIFO 70 / 65   ← v3 same-core
// Core 4:  mpc_main                         FIFO 60
// Core 5:  hand_driver (dedicated)
// Core 6:  arm_driver  (dedicated)
// Core 7:  spare / sim_thread

inline const ThreadConfig kRtControlConfig8Core{.cpu_core = 2,
                                                .sched_policy = SCHED_FIFO,
                                                .sched_priority = 90,
                                                .nice_value = 0,
                                                .name = "rt_control"};

inline const ThreadConfig kRtInboundConfig8Core{.cpu_core = 3,
                                                .sched_policy = SCHED_FIFO,
                                                .sched_priority = 70,
                                                .nice_value = 0,
                                                .name = "rt_inbound"};

inline const ThreadConfig kRtOutboundConfig8Core{.cpu_core = 3,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 65,
                                                 .nice_value = 0,
                                                 .name = "rt_outbound"};

inline const ThreadConfig kNrtLoggingConfig8Core{.cpu_core = 0,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = -5,
                                                 .name = "nrt_logging"};

inline const ThreadConfig kNrtCallbackConfig8Core{.cpu_core = 1,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = 0,
                                                  .name = "nrt_callback"};

inline const ThreadConfig kHandDriverConfig8Core{.cpu_core = 5,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "hand_driver"};

inline const ThreadConfig kArmDriverConfig8Core{.cpu_core = 6,
                                                .sched_policy = SCHED_OTHER,
                                                .sched_priority = 0,
                                                .nice_value = 0,
                                                .name = "arm_driver"};

inline const ThreadConfig kSimThreadConfig8Core{.cpu_core = 7,
                                                .sched_policy = SCHED_OTHER,
                                                .sched_priority = 0,
                                                .nice_value = 0,
                                                .name = "sim_thread"};

inline const ThreadConfig kViewerConfig8Core{.cpu_core = -1,
                                             .sched_policy = SCHED_OTHER,
                                             .sched_priority = 0,
                                             .nice_value = 0,
                                             .name = "viewer"};

inline const MpcThreadConfig kMpcConfig8Core{
    .main =
        ThreadConfig{
            .cpu_core = 4,
            .sched_policy = SCHED_FIFO,
            .sched_priority = 60,
            .nice_value = 0,
            .name = "mpc_main",
        },
    .num_workers = 0,
    .workers = {},
};

// ── 10-core configuration (layout v3) ───────────────────────────────────────
// Core 0:  OS / DDS / IRQ + nrt_logging
// Core 1:  nrt_callback
// Core 2:  rt_control                       FIFO 90
// Core 3:  rt_inbound + rt_outbound         FIFO 70 / 65   ← v3 same-core
// Core 4:  mpc_main                         FIFO 60
// Core 5:  mpc_worker_0                     FIFO 55
// Core 6:  hand_driver
// Core 7:  arm_driver
// Core 8:  spare
// Core 9:  spare / sim_thread

inline const ThreadConfig kRtControlConfig10Core{.cpu_core = 2,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 90,
                                                 .nice_value = 0,
                                                 .name = "rt_control"};

inline const ThreadConfig kRtInboundConfig10Core{.cpu_core = 3,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 70,
                                                 .nice_value = 0,
                                                 .name = "rt_inbound"};

inline const ThreadConfig kRtOutboundConfig10Core{.cpu_core = 3,
                                                  .sched_policy = SCHED_FIFO,
                                                  .sched_priority = 65,
                                                  .nice_value = 0,
                                                  .name = "rt_outbound"};

inline const ThreadConfig kNrtLoggingConfig10Core{.cpu_core = 0,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = -5,
                                                  .name = "nrt_logging"};

inline const ThreadConfig kNrtCallbackConfig10Core{.cpu_core = 1,
                                                   .sched_policy = SCHED_OTHER,
                                                   .sched_priority = 0,
                                                   .nice_value = 0,
                                                   .name = "nrt_callback"};

inline const ThreadConfig kHandDriverConfig10Core{.cpu_core = 6,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = 0,
                                                  .name = "hand_driver"};

inline const ThreadConfig kArmDriverConfig10Core{.cpu_core = 7,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "arm_driver"};

inline const ThreadConfig kSimThreadConfig10Core{.cpu_core = 9,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "sim_thread"};

inline const ThreadConfig kViewerConfig10Core{.cpu_core = -1,
                                              .sched_policy = SCHED_OTHER,
                                              .sched_priority = 0,
                                              .nice_value = 0,
                                              .name = "viewer"};

inline const MpcThreadConfig kMpcConfig10Core{
    .main =
        ThreadConfig{
            .cpu_core = 4,
            .sched_policy = SCHED_FIFO,
            .sched_priority = 60,
            .nice_value = 0,
            .name = "mpc_main",
        },
    .num_workers = 1,
    .workers =
        {
            ThreadConfig{
                .cpu_core = 5,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_0",
            },
            ThreadConfig{},
        },
};

// ── 12-core configuration (primary target, layout v3) ───────────────────────
// Core 0:  OS / DDS / IRQ + nrt_logging
// Core 1:  nrt_callback
// Core 2:  rt_control                       FIFO 90
// Core 3:  rt_inbound + rt_outbound         FIFO 70 / 65   ← v3 same-core
// Core 4:  mpc_main                         FIFO 60
// Core 5:  mpc_worker_0                     FIFO 55
// Core 6:  mpc_worker_1                     FIFO 55
// Core 7:  hand_driver
// Core 8:  arm_driver
// Core 9:  spare
// Core 10: spare / sim_thread (sim mode)
// Core 11: spare / user shield

inline const ThreadConfig kRtControlConfig12Core{.cpu_core = 2,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 90,
                                                 .nice_value = 0,
                                                 .name = "rt_control"};

inline const ThreadConfig kRtInboundConfig12Core{.cpu_core = 3,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 70,
                                                 .nice_value = 0,
                                                 .name = "rt_inbound"};

inline const ThreadConfig kRtOutboundConfig12Core{.cpu_core = 3,
                                                  .sched_policy = SCHED_FIFO,
                                                  .sched_priority = 65,
                                                  .nice_value = 0,
                                                  .name = "rt_outbound"};

inline const ThreadConfig kNrtLoggingConfig12Core{.cpu_core = 0,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = -5,
                                                  .name = "nrt_logging"};

inline const ThreadConfig kNrtCallbackConfig12Core{.cpu_core = 1,
                                                   .sched_policy = SCHED_OTHER,
                                                   .sched_priority = 0,
                                                   .nice_value = 0,
                                                   .name = "nrt_callback"};

inline const ThreadConfig kHandDriverConfig12Core{.cpu_core = 7,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = 0,
                                                  .name = "hand_driver"};

inline const ThreadConfig kArmDriverConfig12Core{.cpu_core = 8,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "arm_driver"};

inline const ThreadConfig kSimThreadConfig12Core{.cpu_core = 10,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "sim_thread"};

inline const ThreadConfig kViewerConfig12Core{.cpu_core = -1,
                                              .sched_policy = SCHED_OTHER,
                                              .sched_priority = 0,
                                              .nice_value = 0,
                                              .name = "viewer"};

inline const MpcThreadConfig kMpcConfig12Core{
    .main =
        ThreadConfig{
            .cpu_core = 4,
            .sched_policy = SCHED_FIFO,
            .sched_priority = 60,
            .nice_value = 0,
            .name = "mpc_main",
        },
    .num_workers = 2,
    .workers =
        {
            ThreadConfig{
                .cpu_core = 5,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_0",
            },
            ThreadConfig{
                .cpu_core = 6,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_1",
            },
        },
};

// ── 14-core configuration (layout v3) ───────────────────────────────────────
// Core 0:  OS / DDS / IRQ + nrt_logging
// Core 1:  nrt_callback
// Core 2:  rt_control                       FIFO 90
// Core 3:  rt_inbound + rt_outbound         FIFO 70 / 65   ← v3 same-core
// Core 4:  mpc_main                         FIFO 60
// Core 5:  mpc_worker_0                     FIFO 55
// Core 6:  mpc_worker_1                     FIFO 55
// Core 7:  hand_driver
// Core 8:  arm_driver
// Core 9:  spare
// Core 10: sim_thread (sim mode)
// Core 11-13: spare / user shield / viewer

inline const ThreadConfig kRtControlConfig14Core{.cpu_core = 2,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 90,
                                                 .nice_value = 0,
                                                 .name = "rt_control"};

inline const ThreadConfig kRtInboundConfig14Core{.cpu_core = 3,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 70,
                                                 .nice_value = 0,
                                                 .name = "rt_inbound"};

inline const ThreadConfig kRtOutboundConfig14Core{.cpu_core = 3,
                                                  .sched_policy = SCHED_FIFO,
                                                  .sched_priority = 65,
                                                  .nice_value = 0,
                                                  .name = "rt_outbound"};

inline const ThreadConfig kNrtLoggingConfig14Core{.cpu_core = 0,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = -5,
                                                  .name = "nrt_logging"};

inline const ThreadConfig kNrtCallbackConfig14Core{.cpu_core = 1,
                                                   .sched_policy = SCHED_OTHER,
                                                   .sched_priority = 0,
                                                   .nice_value = 0,
                                                   .name = "nrt_callback"};

inline const ThreadConfig kHandDriverConfig14Core{.cpu_core = 7,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = 0,
                                                  .name = "hand_driver"};

inline const ThreadConfig kArmDriverConfig14Core{.cpu_core = 8,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "arm_driver"};

inline const ThreadConfig kSimThreadConfig14Core{.cpu_core = 10,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "sim_thread"};

inline const ThreadConfig kViewerConfig14Core{.cpu_core = -1,
                                              .sched_policy = SCHED_OTHER,
                                              .sched_priority = 0,
                                              .nice_value = 0,
                                              .name = "viewer"};

inline const MpcThreadConfig kMpcConfig14Core{
    .main =
        ThreadConfig{
            .cpu_core = 4,
            .sched_policy = SCHED_FIFO,
            .sched_priority = 60,
            .nice_value = 0,
            .name = "mpc_main",
        },
    .num_workers = 2,
    .workers =
        {
            ThreadConfig{
                .cpu_core = 5,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_0",
            },
            ThreadConfig{
                .cpu_core = 6,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_1",
            },
        },
};

// ── 16-core configuration (cset shield 4-8 retained, layout v3) ─────────────
// Core 0:   OS / DDS / IRQ + nrt_logging
// Core 1:   nrt_callback
// Core 2:   rt_control                      FIFO 90
// Core 3:   rt_inbound + rt_outbound        FIFO 70 / 65   ← v3 same-core
// Core 4-8: cset shield "user" (retained from prior layout)
// Core 9:   mpc_main                        FIFO 60
// Core 10:  mpc_worker_0                    FIFO 55
// Core 11:  mpc_worker_1                    FIFO 55
// Core 12:  hand_driver
// Core 13:  arm_driver
// Core 14:  spare
// Core 15:  spare / sim_thread

inline const ThreadConfig kRtControlConfig16Core{.cpu_core = 2,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 90,
                                                 .nice_value = 0,
                                                 .name = "rt_control"};

inline const ThreadConfig kRtInboundConfig16Core{.cpu_core = 3,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 70,
                                                 .nice_value = 0,
                                                 .name = "rt_inbound"};

inline const ThreadConfig kRtOutboundConfig16Core{.cpu_core = 3,
                                                  .sched_policy = SCHED_FIFO,
                                                  .sched_priority = 65,
                                                  .nice_value = 0,
                                                  .name = "rt_outbound"};

inline const ThreadConfig kNrtLoggingConfig16Core{.cpu_core = 0,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = -5,
                                                  .name = "nrt_logging"};

inline const ThreadConfig kNrtCallbackConfig16Core{.cpu_core = 1,
                                                   .sched_policy = SCHED_OTHER,
                                                   .sched_priority = 0,
                                                   .nice_value = 0,
                                                   .name = "nrt_callback"};

inline const ThreadConfig kHandDriverConfig16Core{.cpu_core = 12,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = 0,
                                                  .name = "hand_driver"};

inline const ThreadConfig kArmDriverConfig16Core{.cpu_core = 13,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "arm_driver"};

inline const ThreadConfig kSimThreadConfig16Core{.cpu_core = 15,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "sim_thread"};

inline const ThreadConfig kViewerConfig16Core{.cpu_core = -1,
                                              .sched_policy = SCHED_OTHER,
                                              .sched_priority = 0,
                                              .nice_value = 0,
                                              .name = "viewer"};

inline const MpcThreadConfig kMpcConfig16Core{
    .main =
        ThreadConfig{
            .cpu_core = 9,
            .sched_policy = SCHED_FIFO,
            .sched_priority = 60,
            .nice_value = 0,
            .name = "mpc_main",
        },
    .num_workers = 2,
    .workers =
        {
            ThreadConfig{
                .cpu_core = 10,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_0",
            },
            ThreadConfig{
                .cpu_core = 11,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_1",
            },
        },
};

}  // namespace rtc

#endif  // RTC_BASE_THREAD_CONFIG_HPP_
