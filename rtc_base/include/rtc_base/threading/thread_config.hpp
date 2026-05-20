#ifndef RTC_BASE_THREAD_CONFIG_HPP_
#define RTC_BASE_THREAD_CONFIG_HPP_

#include <sched.h>  // SCHED_FIFO, SCHED_OTHER, SCHED_RR

#include <array>

namespace rtc {

// Thread configuration for RT control and scheduling
struct ThreadConfig {
  // CPU affinity *slot index* (0-based; -1 = no pinning).
  // A slot identifies one unique physical core, not a kernel logical CPU id:
  // ApplyThreadConfig translates the slot through CpuTopology::physical_core_slots
  // so RT threads always land on a P-core's *primary* logical (never the SMT
  // sibling) on hybrid Intel and AMD SMT alike. Slot ordering:
  //   hybrid     → P-physical → E-core → LP-E (P-core first, spill to E only
  //                  when slot index exceeds num_p_physical)
  //   non-hybrid → physical core first-logicals, ascending
  //   SMT off    → identity (slot N → logical N), unchanged behaviour
  // See SlotToLogicalCpu() in thread_utils.hpp.
  int cpu_core;
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
//   * `main.sched_priority` < rt_callback thread priority (rt_callback preempts MPC).
//   * Each worker `sched_priority` ≤ `main.sched_priority` (workers never
//     preempt the main solve).
//   * `0 ≤ num_workers ≤ 2` (matches 12-/16-core capacity).

inline constexpr int kMpcMaxWorkers = 2;

struct MpcThreadConfig {
  ThreadConfig main{};
  int num_workers{0};
  std::array<ThreadConfig, kMpcMaxWorkers> workers{};
};

// ── RT priority hierarchy (layout v4.1) ─────────────────────────────────────
//   90 (rt_control)  > 70 (rt_callback) > 60 (mpc_main) > 55 (mpc_workers)
//
// IMPORTANT — "Core N" below is a *slot index*, not a kernel logical CPU id.
// ApplyThreadConfig translates the slot through CpuTopology::physical_core_slots
// so RT threads land on a physical P-core's *primary* logical (never the SMT
// sibling) on hybrid Intel (Raptor Lake-S i9-13900K, NUC13 Pro, Meteor Lake)
// and AMD SMT alike. The "Core 1" slot maps to logical cpu 2 on a SMT-on
// 4P+8E hybrid, to logical cpu 1 on a SMT-off 4-core, to logical cpu 2 on
// AMD Ryzen 8C/16T — a single uniform meaning across topologies.
//
// Layout v4 unified former rt_inbound (FIFO 70) + rt_outbound (FIFO 65) into
// a single rt_callback thread. v4.1 shifts the RT cluster down to start at
// Core 1 (Core 0 is reserved for OS / DDS / IRQ only) — rt_control = Core 1,
// rt_callback + DDS recv co-pin = Core 2, mpc_main = Core 3, workers follow.
// The rt_outbound jthread + publish_buffer_ SPSC + eventfd wakeup are removed;
// rt_control performs DeviceBackend.WriteCommand inline in the rt_loop tick
// (RT-safe contract on backends). DDS receive thread is co-pinned to the same
// core as rt_callback (Core 2 on every tier except 4-core fallback) via
// launch-time taskset for cache locality. Hand-private UDP receive thread
// (priority 65) lives inside the hand_driver process and inherits affinity
// from the launch-level taskset, so it is NOT represented in
// SystemThreadConfigs.
//
// process-level threads (arm_driver, hand_driver, sim_thread, viewer) are
// SCHED_OTHER prio 0; only their cpu_core is consumed (taskset pin), all
// other fields are passed through ApplyThreadConfig as no-op.
//
// ── Tier block ordering ─────────────────────────────────────────────────────
// Tier blocks are listed in ascending core count: 4 → 6 → 8 → 10 → 12 → 14 → 16.
// Within each tier, entries are sorted by sched_priority descending, with
// alphabetical tie-break on the symbol name (Arm < Hand < NrtCallback <
// NrtLogging < SimThread < Viewer for the priority-0 group). MpcThreadConfig
// occupies its main.sched_priority slot (60 on RT tiers, 0 on 4-core).

// ── 4-core fallback (degraded — no deterministic RT guarantee) ──────────────
// Core 0:   OS / DDS / IRQ + nrt_logging + nrt_callback + arm/hand_driver
// Core 1:   rt_control                     FIFO 90
// Core 2:   rt_callback                    FIFO 70
// Core 3:   mpc_main (SCHED_OTHER nice -5 — degraded)

inline const ThreadConfig kRtControlConfig4Core{.cpu_core = 1,
                                                .sched_policy = SCHED_FIFO,
                                                .sched_priority = 90,
                                                .nice_value = 0,
                                                .name = "rt_control"};

// Layout v4 promotes rt_callback to SCHED_FIFO 70 on the 4-core fallback
// (v3 kept rt_outbound on CFS). On a 4-core box DDS receive co-pins to
// Core 2 (no spare core), so DDS dispatch shares the same FIFO 70 queue
// as the rt_callback executor. Because both serve the controller↔hardware
// state-input boundary, this co-residency is acceptable; the 4-core tier
// is explicitly "degraded — no deterministic RT guarantee" (header above).
inline const ThreadConfig kRtCallbackConfig4Core{.cpu_core = 2,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 70,
                                                 .nice_value = 0,
                                                 .name = "rt_callback"};

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

inline const ThreadConfig kNrtCallbackConfig4Core{.cpu_core = 0,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = 0,
                                                  .name = "nrt_callback"};

inline const ThreadConfig kNrtLoggingConfig4Core{.cpu_core = 0,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = -5,
                                                 .name = "nrt_logging"};

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

// ── 6-core configuration (degraded mode — no deterministic RT guarantee) ────
// Core 0:  OS / DDS / IRQ
// Core 1:  rt_control                       FIFO 90
// Core 2:  rt_callback + DDS recv           FIFO 70 (RT) / CFS (DDS)
// Core 3:  mpc_main                         FIFO 60
// Core 4:  arm_driver + hand_driver (shared, degraded)
// Core 5:  nrt_logging + nrt_callback (shared, degraded)
//
// Trade-off: mpc_worker absent, arm/hand share Core 4, nrt_* share Core 5.
// "degraded mode" label required — no deterministic RT guarantee. Core 0 is
// reserved for OS / DDS / IRQ; user-space threads start at Core 1.

inline const ThreadConfig kRtControlConfig{.cpu_core = 1,
                                           .sched_policy = SCHED_FIFO,
                                           .sched_priority = 90,
                                           .nice_value = 0,
                                           .name = "rt_control"};

inline const ThreadConfig kRtCallbackConfig{.cpu_core = 2,
                                            .sched_policy = SCHED_FIFO,
                                            .sched_priority = 70,
                                            .nice_value = 0,
                                            .name = "rt_callback"};

inline const MpcThreadConfig kMpcConfig6Core{
    .main =
        ThreadConfig{
            .cpu_core = 3,
            .sched_policy = SCHED_FIFO,
            .sched_priority = 60,
            .nice_value = 0,
            .name = "mpc_main",
        },
    .num_workers = 0,
    .workers = {},
};

inline const ThreadConfig kArmDriverConfig{.cpu_core = 4,
                                           .sched_policy = SCHED_OTHER,
                                           .sched_priority = 0,
                                           .nice_value = 0,
                                           .name = "arm_driver"};

inline const ThreadConfig kHandDriverConfig{.cpu_core = 4,
                                            .sched_policy = SCHED_OTHER,
                                            .sched_priority = 0,
                                            .nice_value = 0,
                                            .name = "hand_driver"};

inline const ThreadConfig kNrtCallbackConfig{.cpu_core = 5,
                                             .sched_policy = SCHED_OTHER,
                                             .sched_priority = 0,
                                             .nice_value = 0,
                                             .name = "nrt_callback"};

inline const ThreadConfig kNrtLoggingConfig{.cpu_core = 5,
                                            .sched_policy = SCHED_OTHER,
                                            .sched_priority = 0,
                                            .nice_value = -5,
                                            .name = "nrt_logging"};

// sim_thread / viewer cpu_core = -1: caller-controlled, no pinning on any
// tier. cpu_shield --sim releases the shield so MuJoCo physics + GLFW viewer
// may roam over freed cores under CFS.
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

// ── 8-core configuration (layout v4.1) ──────────────────────────────────────
// Core 0:  OS / DDS / IRQ
// Core 1:  rt_control                       FIFO 90
// Core 2:  rt_callback + DDS recv           FIFO 70 (RT) / CFS (DDS)
// Core 3:  mpc_main                         FIFO 60
// Core 4:  arm_driver (dedicated)
// Core 5:  hand_driver (dedicated)
// Core 6:  nrt_logging
// Core 7:  nrt_callback

inline const ThreadConfig kRtControlConfig8Core{.cpu_core = 1,
                                                .sched_policy = SCHED_FIFO,
                                                .sched_priority = 90,
                                                .nice_value = 0,
                                                .name = "rt_control"};

inline const ThreadConfig kRtCallbackConfig8Core{.cpu_core = 2,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 70,
                                                 .nice_value = 0,
                                                 .name = "rt_callback"};

inline const MpcThreadConfig kMpcConfig8Core{
    .main =
        ThreadConfig{
            .cpu_core = 3,
            .sched_policy = SCHED_FIFO,
            .sched_priority = 60,
            .nice_value = 0,
            .name = "mpc_main",
        },
    .num_workers = 0,
    .workers = {},
};

inline const ThreadConfig kArmDriverConfig8Core{.cpu_core = 4,
                                                .sched_policy = SCHED_OTHER,
                                                .sched_priority = 0,
                                                .nice_value = 0,
                                                .name = "arm_driver"};

inline const ThreadConfig kHandDriverConfig8Core{.cpu_core = 5,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "hand_driver"};

inline const ThreadConfig kNrtCallbackConfig8Core{.cpu_core = 7,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = 0,
                                                  .name = "nrt_callback"};

inline const ThreadConfig kNrtLoggingConfig8Core{.cpu_core = 6,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = -5,
                                                 .name = "nrt_logging"};

inline const ThreadConfig kSimThreadConfig8Core{.cpu_core = -1,
                                                .sched_policy = SCHED_OTHER,
                                                .sched_priority = 0,
                                                .nice_value = 0,
                                                .name = "sim_thread"};

inline const ThreadConfig kViewerConfig8Core{.cpu_core = -1,
                                             .sched_policy = SCHED_OTHER,
                                             .sched_priority = 0,
                                             .nice_value = 0,
                                             .name = "viewer"};

// ── 10-core configuration (layout v4.1) ─────────────────────────────────────
// Core 0:  OS / DDS / IRQ
// Core 1:  rt_control                       FIFO 90
// Core 2:  rt_callback + DDS recv           FIFO 70 (RT) / CFS (DDS)
// Core 3:  mpc_main                         FIFO 60
// Core 4:  mpc_worker_0                     FIFO 55
// Core 5:  arm_driver
// Core 6:  hand_driver
// Core 7:  nrt_logging
// Core 8:  nrt_callback
// Core 9:  spare

inline const ThreadConfig kRtControlConfig10Core{.cpu_core = 1,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 90,
                                                 .nice_value = 0,
                                                 .name = "rt_control"};

inline const ThreadConfig kRtCallbackConfig10Core{.cpu_core = 2,
                                                  .sched_policy = SCHED_FIFO,
                                                  .sched_priority = 70,
                                                  .nice_value = 0,
                                                  .name = "rt_callback"};

inline const MpcThreadConfig kMpcConfig10Core{
    .main =
        ThreadConfig{
            .cpu_core = 3,
            .sched_policy = SCHED_FIFO,
            .sched_priority = 60,
            .nice_value = 0,
            .name = "mpc_main",
        },
    .num_workers = 1,
    .workers =
        {
            ThreadConfig{
                .cpu_core = 4,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_0",
            },
            ThreadConfig{},
        },
};

inline const ThreadConfig kArmDriverConfig10Core{.cpu_core = 5,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "arm_driver"};

inline const ThreadConfig kHandDriverConfig10Core{.cpu_core = 6,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = 0,
                                                  .name = "hand_driver"};

inline const ThreadConfig kNrtCallbackConfig10Core{.cpu_core = 8,
                                                   .sched_policy = SCHED_OTHER,
                                                   .sched_priority = 0,
                                                   .nice_value = 0,
                                                   .name = "nrt_callback"};

inline const ThreadConfig kNrtLoggingConfig10Core{.cpu_core = 7,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = -5,
                                                  .name = "nrt_logging"};

inline const ThreadConfig kSimThreadConfig10Core{.cpu_core = -1,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "sim_thread"};

inline const ThreadConfig kViewerConfig10Core{.cpu_core = -1,
                                              .sched_policy = SCHED_OTHER,
                                              .sched_priority = 0,
                                              .nice_value = 0,
                                              .name = "viewer"};

// ── 12-core configuration (primary target, layout v4.1) ─────────────────────
// Core 0:  OS / DDS / IRQ
// Core 1:  rt_control                       FIFO 90
// Core 2:  rt_callback + DDS recv           FIFO 70 (RT) / CFS (DDS)
// Core 3:  mpc_main                         FIFO 60
// Core 4:  mpc_worker_0                     FIFO 55
// Core 5:  mpc_worker_1                     FIFO 55
// Core 6:  arm_driver
// Core 7:  hand_driver
// Core 8:  nrt_logging
// Core 9:  nrt_callback
// Core 10: spare
// Core 11: spare / user shield

inline const ThreadConfig kRtControlConfig12Core{.cpu_core = 1,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 90,
                                                 .nice_value = 0,
                                                 .name = "rt_control"};

inline const ThreadConfig kRtCallbackConfig12Core{.cpu_core = 2,
                                                  .sched_policy = SCHED_FIFO,
                                                  .sched_priority = 70,
                                                  .nice_value = 0,
                                                  .name = "rt_callback"};

inline const MpcThreadConfig kMpcConfig12Core{
    .main =
        ThreadConfig{
            .cpu_core = 3,
            .sched_policy = SCHED_FIFO,
            .sched_priority = 60,
            .nice_value = 0,
            .name = "mpc_main",
        },
    .num_workers = 2,
    .workers =
        {
            ThreadConfig{
                .cpu_core = 4,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_0",
            },
            ThreadConfig{
                .cpu_core = 5,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_1",
            },
        },
};

inline const ThreadConfig kArmDriverConfig12Core{.cpu_core = 6,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "arm_driver"};

inline const ThreadConfig kHandDriverConfig12Core{.cpu_core = 7,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = 0,
                                                  .name = "hand_driver"};

inline const ThreadConfig kNrtCallbackConfig12Core{.cpu_core = 9,
                                                   .sched_policy = SCHED_OTHER,
                                                   .sched_priority = 0,
                                                   .nice_value = 0,
                                                   .name = "nrt_callback"};

inline const ThreadConfig kNrtLoggingConfig12Core{.cpu_core = 8,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = -5,
                                                  .name = "nrt_logging"};

inline const ThreadConfig kSimThreadConfig12Core{.cpu_core = -1,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "sim_thread"};

inline const ThreadConfig kViewerConfig12Core{.cpu_core = -1,
                                              .sched_policy = SCHED_OTHER,
                                              .sched_priority = 0,
                                              .nice_value = 0,
                                              .name = "viewer"};

// ── 14-core configuration (layout v4.1) ─────────────────────────────────────
// Core 0:  OS / DDS / IRQ
// Core 1:  rt_control                       FIFO 90
// Core 2:  rt_callback + DDS recv           FIFO 70 (RT) / CFS (DDS)
// Core 3:  mpc_main                         FIFO 60
// Core 4:  mpc_worker_0                     FIFO 55
// Core 5:  mpc_worker_1                     FIFO 55
// Core 6:  arm_driver
// Core 7:  hand_driver
// Core 8:  nrt_logging
// Core 9:  nrt_callback
// Core 10-13: spare / user shield

inline const ThreadConfig kRtControlConfig14Core{.cpu_core = 1,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 90,
                                                 .nice_value = 0,
                                                 .name = "rt_control"};

inline const ThreadConfig kRtCallbackConfig14Core{.cpu_core = 2,
                                                  .sched_policy = SCHED_FIFO,
                                                  .sched_priority = 70,
                                                  .nice_value = 0,
                                                  .name = "rt_callback"};

inline const MpcThreadConfig kMpcConfig14Core{
    .main =
        ThreadConfig{
            .cpu_core = 3,
            .sched_policy = SCHED_FIFO,
            .sched_priority = 60,
            .nice_value = 0,
            .name = "mpc_main",
        },
    .num_workers = 2,
    .workers =
        {
            ThreadConfig{
                .cpu_core = 4,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_0",
            },
            ThreadConfig{
                .cpu_core = 5,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_1",
            },
        },
};

inline const ThreadConfig kArmDriverConfig14Core{.cpu_core = 6,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "arm_driver"};

inline const ThreadConfig kHandDriverConfig14Core{.cpu_core = 7,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = 0,
                                                  .name = "hand_driver"};

inline const ThreadConfig kNrtCallbackConfig14Core{.cpu_core = 9,
                                                   .sched_policy = SCHED_OTHER,
                                                   .sched_priority = 0,
                                                   .nice_value = 0,
                                                   .name = "nrt_callback"};

inline const ThreadConfig kNrtLoggingConfig14Core{.cpu_core = 8,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = -5,
                                                  .name = "nrt_logging"};

inline const ThreadConfig kSimThreadConfig14Core{.cpu_core = -1,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "sim_thread"};

inline const ThreadConfig kViewerConfig14Core{.cpu_core = -1,
                                              .sched_policy = SCHED_OTHER,
                                              .sched_priority = 0,
                                              .nice_value = 0,
                                              .name = "viewer"};

// ── 16-core configuration (layout v4.1) ─────────────────────────────────────
// Core 0:   OS / DDS / IRQ
// Core 1:   rt_control                      FIFO 90
// Core 2:   rt_callback + DDS recv          FIFO 70 (RT) / CFS (DDS)
// Core 3:   mpc_main                        FIFO 60
// Core 4:   mpc_worker_0                    FIFO 55
// Core 5:   mpc_worker_1                    FIFO 55
// Core 6:   arm_driver
// Core 7:   hand_driver
// Core 8:   nrt_logging
// Core 9:   nrt_callback
// Core 10-15: spare / user shield
//
// v4.1: prior cset shield "user" on Core 4-8 removed for RT cluster cache
// locality. Cores 10-15 remain free for user-level workloads.

inline const ThreadConfig kRtControlConfig16Core{.cpu_core = 1,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 90,
                                                 .nice_value = 0,
                                                 .name = "rt_control"};

inline const ThreadConfig kRtCallbackConfig16Core{.cpu_core = 2,
                                                  .sched_policy = SCHED_FIFO,
                                                  .sched_priority = 70,
                                                  .nice_value = 0,
                                                  .name = "rt_callback"};

inline const MpcThreadConfig kMpcConfig16Core{
    .main =
        ThreadConfig{
            .cpu_core = 3,
            .sched_policy = SCHED_FIFO,
            .sched_priority = 60,
            .nice_value = 0,
            .name = "mpc_main",
        },
    .num_workers = 2,
    .workers =
        {
            ThreadConfig{
                .cpu_core = 4,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_0",
            },
            ThreadConfig{
                .cpu_core = 5,
                .sched_policy = SCHED_FIFO,
                .sched_priority = 55,
                .nice_value = 0,
                .name = "mpc_worker_1",
            },
        },
};

inline const ThreadConfig kArmDriverConfig16Core{.cpu_core = 6,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "arm_driver"};

inline const ThreadConfig kHandDriverConfig16Core{.cpu_core = 7,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = 0,
                                                  .name = "hand_driver"};

inline const ThreadConfig kNrtCallbackConfig16Core{.cpu_core = 9,
                                                   .sched_policy = SCHED_OTHER,
                                                   .sched_priority = 0,
                                                   .nice_value = 0,
                                                   .name = "nrt_callback"};

inline const ThreadConfig kNrtLoggingConfig16Core{.cpu_core = 8,
                                                  .sched_policy = SCHED_OTHER,
                                                  .sched_priority = 0,
                                                  .nice_value = -5,
                                                  .name = "nrt_logging"};

inline const ThreadConfig kSimThreadConfig16Core{.cpu_core = -1,
                                                 .sched_policy = SCHED_OTHER,
                                                 .sched_priority = 0,
                                                 .nice_value = 0,
                                                 .name = "sim_thread"};

inline const ThreadConfig kViewerConfig16Core{.cpu_core = -1,
                                              .sched_policy = SCHED_OTHER,
                                              .sched_priority = 0,
                                              .nice_value = 0,
                                              .name = "viewer"};

}  // namespace rtc

#endif  // RTC_BASE_THREAD_CONFIG_HPP_
