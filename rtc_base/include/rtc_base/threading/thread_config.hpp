#ifndef RTC_BASE_THREAD_CONFIG_HPP_
#define RTC_BASE_THREAD_CONFIG_HPP_

#include <sched.h> // SCHED_FIFO, SCHED_OTHER, SCHED_RR

#include <array>

namespace rtc {

// Thread configuration for RT control and scheduling
struct ThreadConfig {
  int cpu_core;       // CPU affinity (0-based core index)
  int sched_policy;   // SCHED_FIFO, SCHED_RR, or SCHED_OTHER
  int sched_priority; // 1-99 for SCHED_FIFO/RR, ignored for OTHER
  int nice_value;     // -20 to 19 for SCHED_OTHER, ignored for FIFO/RR
  const char *name;   // Thread name for debugging (max 15 chars)
};

// ── MPC thread configuration (Phase 5) ──────────────────────────────────────
// Holds the main MPC solve thread plus optional worker threads used by
// parallel solvers (e.g. Aligator's parallel rollout). Only the first
// `num_workers` entries of `workers` are valid.
//
// Invariants (checked by ValidateSystemThreadConfigs):
//   * `main.sched_priority` < sensor thread priority (sensor preempts MPC).
//   * Each worker `sched_priority` ≤ `main.sched_priority` (workers never
//     preempt the main solve).
//   * `0 ≤ num_workers ≤ 2` (matches 12-/16-core capacity).

inline constexpr int kMpcMaxWorkers = 2;

struct MpcThreadConfig {
  ThreadConfig main{};
  int num_workers{0};
  std::array<ThreadConfig, kMpcMaxWorkers> workers{};
};

// ── 6-core configuration ────────────────────────────────────────────────────
// Core 0-1: OS / DDS / NIC IRQ  (isolated by isolcpus=2-5)
// Core 2:   RT Control           (500 Hz ControlLoop + 50 Hz E-STOP watchdog)
// Core 3:   Sensor I/O           (joint_state, target, hand callbacks —
// dedicated) Core 4:   Logging              (100 Hz CSV drain) Core 5:   UDP
// recv + Aux       (udp_recv FIFO 65, aux SCHED_OTHER 0)
//
// Note: udp_recv is on Core 5 (not Core 3) to avoid contention with sensor_io.
//       Even under UDP burst, JointStateCallback latency is unaffected.

inline const ThreadConfig kRtControlConfig{.cpu_core = 2,
                                           .sched_policy = SCHED_FIFO,
                                           .sched_priority = 90,
                                           .nice_value = 0,
                                           .name = "rt_control"};

inline const ThreadConfig kSensorConfig{.cpu_core = 3,
                                        .sched_policy = SCHED_FIFO,
                                        .sched_priority = 70,
                                        .nice_value = 0,
                                        .name = "sensor_io"};

inline const ThreadConfig kUdpRecvConfig{
    .cpu_core =
        5, // Moved from Core 3 → Core 5 (dedicated, no sensor_io contention)
    .sched_policy = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value = 0,
    .name = "udp_recv"};

inline const ThreadConfig kLoggingConfig{.cpu_core = 4,
                                         .sched_policy = SCHED_OTHER,
                                         .sched_priority = 0,
                                         .nice_value = -5,
                                         .name = "logger"};

inline const ThreadConfig kAuxConfig{
    .cpu_core =
        5, // Shares Core 5 with udp_recv (aux is event-driven, very light)
    .sched_policy = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value = 0,
    .name = "aux"};

// ── Publish offload thread (6-core) ──────────────────────────────────────────
// Drains SPSC publish buffer and calls all ROS2 publish() on a non-RT core.
// Shares Core 5 with aux and udp_recv — publish is SCHED_OTHER, preempted
// by udp_recv (SCHED_FIFO 65).

inline const ThreadConfig kPublishConfig{.cpu_core = 5,
                                         .sched_policy = SCHED_OTHER,
                                         .sched_priority = 0,
                                         .nice_value = -3,
                                         .name = "rt_publish"};

// MPC on 6-core: piggybacks on Core 4 with SCHED_FIFO 60. Logging also
// lives on Core 4 (SCHED_OTHER); MPC FIFO preempts logging whenever a
// solve is active. This is acceptable because logging is drain-only.
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

// ── 8-core configuration (Phase 5: MPC dedicated core) ─────────────────────
// Core 0-1: OS / DDS / NIC IRQ  (isolated by cset shield)
// Core 2:   RT Control           (500 Hz ControlLoop + 50 Hz E-STOP watchdog)
// Core 3:   Sensor I/O           (joint_state, target, hand callbacks)
// Core 4:   MPC main             (Phase 5, FIFO 60 — below sensor's 70)
// Core 5:   UDP recv             (dedicated — previously Core 4)
// Core 6:   Logging              (100 Hz CSV drain — previously Core 5)
// Core 7:   Aux + Publish        (E-STOP publisher — previously Core 6)
//
// Rationale: MPC is a first-class RT citizen (soft-RT 20 Hz solve with
// 10-30 ms compute bursts). A shared core would cause unpredictable
// latency for udp_recv and sensor callbacks. The 4→5→6→7 shift preserves
// every other thread's role; only core numbers change.

inline const ThreadConfig kRtControlConfig8Core{.cpu_core = 2,
                                                .sched_policy = SCHED_FIFO,
                                                .sched_priority = 90,
                                                .nice_value = 0,
                                                .name = "rt_control"};

inline const ThreadConfig kSensorConfig8Core{.cpu_core = 3,
                                             .sched_policy = SCHED_FIFO,
                                             .sched_priority = 70,
                                             .nice_value = 0,
                                             .name = "sensor_io"};

inline const ThreadConfig kUdpRecvConfig8Core{
    .cpu_core = 5, // Shifted from Core 4 (now MPC main).
    .sched_policy = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value = 0,
    .name = "udp_recv"};

inline const ThreadConfig kLoggingConfig8Core{.cpu_core =
                                                  6, // Shifted from Core 5.
                                              .sched_policy = SCHED_OTHER,
                                              .sched_priority = 0,
                                              .nice_value = -5,
                                              .name = "logger"};

inline const ThreadConfig kAuxConfig8Core{.cpu_core = 7, // Shifted from Core 6.
                                          .sched_policy = SCHED_OTHER,
                                          .sched_priority = 0,
                                          .nice_value = 0,
                                          .name = "aux"};

inline const ThreadConfig kPublishConfig8Core{.cpu_core =
                                                  7, // Shifted from Core 6.
                                              .sched_policy = SCHED_OTHER,
                                              .sched_priority = 0,
                                              .nice_value = -3,
                                              .name = "rt_publish"};

// MPC main on Core 4 (dedicated). No workers on 8-core — too few spare
// cores to host them without contention. Priority 60 < sensor(70) so a
// long solve can be preempted by sensor callbacks.
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

// ── 4-core fallback ─────────────────────────────────────────────────────────
// Core 0:   OS / DDS / IRQ
// Core 1:   RT Control
// Core 2:   Sensor I/O + UDP recv (shared — best effort)
// Core 3:   Logging + Aux

inline const ThreadConfig kRtControlConfig4Core{.cpu_core = 1,
                                                .sched_policy = SCHED_FIFO,
                                                .sched_priority = 90,
                                                .nice_value = 0,
                                                .name = "rt_control"};

inline const ThreadConfig kSensorConfig4Core{.cpu_core = 2,
                                             .sched_policy = SCHED_FIFO,
                                             .sched_priority = 70,
                                             .nice_value = 0,
                                             .name = "sensor_io"};

inline const ThreadConfig kUdpRecvConfig4Core{
    .cpu_core = 2, // Shares Core 2 with sensor_io (4-core: unavoidable)
    .sched_policy = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value = 0,
    .name = "udp_recv"};

inline const ThreadConfig kLoggingConfig4Core{.cpu_core = 3,
                                              .sched_policy = SCHED_OTHER,
                                              .sched_priority = 0,
                                              .nice_value = -5,
                                              .name = "logger"};

inline const ThreadConfig kAuxConfig4Core{
    .cpu_core = 3, // Shares Core 3 with logging (4-core: unavoidable)
    .sched_policy = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value = 0,
    .name = "aux"};

inline const ThreadConfig kPublishConfig4Core{.cpu_core = 3,
                                              .sched_policy = SCHED_OTHER,
                                              .sched_priority = 0,
                                              .nice_value = -3,
                                              .name = "rt_publish"};

// MPC on 4-core: piggybacks on Core 3 with SCHED_OTHER (degraded mode).
// With only 3 non-OS cores a FIFO MPC would starve logging/aux; CFS is
// the safer choice. Consumers are expected to reduce MPC frequency to
// 10 Hz on this tier.
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

// ── 10-core configuration (unified layout) ─────────────────────────────────
// Extends the 8-core layout with MPC worker 0; every RT thread keeps the
// same core number as in 8-core. Core 9 is free for MuJoCo sim / monitoring.
//
// Core 0-1:  OS / DDS / NIC IRQ
// Core 2:    RT Control           (SCHED_FIFO 90)
// Core 3:    Sensor I/O           (SCHED_FIFO 70)
// Core 4:    MPC main             (SCHED_FIFO 60)
// Core 5:    MPC worker 0         (SCHED_FIFO 55) — new vs 8-core
// Core 6:    UDP recv             (SCHED_FIFO 65)
// Core 7:    Logging              (100 Hz CSV drain)
// Core 8:    Aux + Publish        (E-STOP service + DDS drain)
// Core 9:    MuJoCo sim / spare   (GetSimCoreLayout(10) = {9,-1})
//
// Monotonicity vs 8-core: +1 MPC worker, +1 spare core. No RT thread loses
// dedication. Shield range (cpu_shield.sh) covers Core 2-8 to protect all
// RT cores.

inline const ThreadConfig kRtControlConfig10Core{.cpu_core = 2,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 90,
                                                 .nice_value = 0,
                                                 .name = "rt_control"};

inline const ThreadConfig kSensorConfig10Core{.cpu_core = 3,
                                              .sched_policy = SCHED_FIFO,
                                              .sched_priority = 70,
                                              .nice_value = 0,
                                              .name = "sensor_io"};

inline const ThreadConfig kUdpRecvConfig10Core{
    .cpu_core = 6, // Dedicated (was Core 9 shared prior to unified layout)
    .sched_policy = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value = 0,
    .name = "udp_recv"};

inline const ThreadConfig kLoggingConfig10Core{.cpu_core = 7, // Dedicated
                                               .sched_policy = SCHED_OTHER,
                                               .sched_priority = 0,
                                               .nice_value = -5,
                                               .name = "logger"};

inline const ThreadConfig kAuxConfig10Core{
    .cpu_core = 8, // Dedicated (shared only with rt_publish, both CFS)
    .sched_policy = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value = 0,
    .name = "aux"};

inline const ThreadConfig kPublishConfig10Core{
    .cpu_core = 8, // Shares Core 8 with aux (both SCHED_OTHER)
    .sched_policy = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value = -3,
    .name = "rt_publish"};

// MPC on 10-core: dedicated Core 4 main + Core 5 worker. Priority
// 60/55 keeps sensor (70) preemptive and UDP (65) ahead of MPC, matching
// the tiered priority contract documented in rt-safety.md.
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

// ── 12-core configuration (unified layout, MPC main + 2 workers) ───────────
// Extends the 10-core layout with MPC worker 1; every RT thread keeps the
// same core number up through sensor/mpc_main/mpc_worker_0. Cores 10-11
// are spare (user shield / MuJoCo viewer / ROS launch dispatcher).
//
// Core 0-1:  OS / DDS / NIC IRQ
// Core 2:    RT Control           (SCHED_FIFO 90)
// Core 3:    Sensor I/O           (SCHED_FIFO 70)
// Core 4:    MPC main             (SCHED_FIFO 60)
// Core 5:    MPC worker 0         (SCHED_FIFO 55)
// Core 6:    MPC worker 1         (SCHED_FIFO 55) — new vs 10-core
// Core 7:    UDP recv             (SCHED_FIFO 65)
// Core 8:    Logging              (100 Hz CSV drain)
// Core 9:    Aux + Publish        (E-STOP service + DDS drain)
// Core 10:   MuJoCo sim / spare   (GetSimCoreLayout(12) = {10,-1})
// Core 11:   Spare (user shield / monitoring)
//
// Monotonicity vs 10-core: +1 MPC worker, +2 spare cores. No RT thread
// loses dedication. Shield range covers Core 2-9.

inline const ThreadConfig kRtControlConfig12Core{.cpu_core = 2,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 90,
                                                 .nice_value = 0,
                                                 .name = "rt_control"};

inline const ThreadConfig kSensorConfig12Core{.cpu_core = 3,
                                              .sched_policy = SCHED_FIFO,
                                              .sched_priority = 70,
                                              .nice_value = 0,
                                              .name = "sensor_io"};

inline const ThreadConfig kUdpRecvConfig12Core{.cpu_core = 7, // Dedicated
                                               .sched_policy = SCHED_FIFO,
                                               .sched_priority = 65,
                                               .nice_value = 0,
                                               .name = "udp_recv"};

inline const ThreadConfig kLoggingConfig12Core{.cpu_core = 8, // Dedicated
                                               .sched_policy = SCHED_OTHER,
                                               .sched_priority = 0,
                                               .nice_value = -5,
                                               .name = "logger"};

inline const ThreadConfig kAuxConfig12Core{
    .cpu_core = 9, // Dedicated (shared with rt_publish, both CFS)
    .sched_policy = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value = 0,
    .name = "aux"};

inline const ThreadConfig kPublishConfig12Core{.cpu_core = 9,
                                               .sched_policy = SCHED_OTHER,
                                               .sched_priority = 0,
                                               .nice_value = -3,
                                               .name = "rt_publish"};

// MPC on 12-core: main + 2 workers enable full Aligator parallel-rollout
// capacity. Both workers at FIFO 55 — identical priority is safe because
// they run on distinct cores (5, 6); ValidateSystemThreadConfigs flags
// only same-core equal-FIFO pairs.
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

// ── 14-core configuration (unified layout, dedicated sim core) ─────────────
// Same RT/MPC layout as 12-core; the two extra cores (12, 13) extend the
// spare/user-shield range and Core 10 becomes the MuJoCo sim_thread home.
//
// Core 0-1:  OS / DDS / NIC IRQ
// Core 2:    RT Control           (SCHED_FIFO 90)
// Core 3:    Sensor I/O           (SCHED_FIFO 70)
// Core 4:    MPC main             (SCHED_FIFO 60)
// Core 5:    MPC worker 0         (SCHED_FIFO 55)
// Core 6:    MPC worker 1         (SCHED_FIFO 55)
// Core 7:    UDP recv             (SCHED_FIFO 65)
// Core 8:    Logging              (100 Hz CSV drain)
// Core 9:    Aux + Publish        (E-STOP service + DDS drain)
// Core 10:   MuJoCo sim_thread    (GetSimCoreLayout(14) = {10,-1})
// Core 11-13: Spare (user shield / MuJoCo viewer / monitoring)
//
// Prior to this block, SelectThreadConfigs() picked the 12-core tier for
// 14-core machines, silently wasting Cores 12-13. Splitting the tier fixes
// that and gives MuJoCo a dedicated physics core.

inline const ThreadConfig kRtControlConfig14Core{.cpu_core = 2,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 90,
                                                 .nice_value = 0,
                                                 .name = "rt_control"};

inline const ThreadConfig kSensorConfig14Core{.cpu_core = 3,
                                              .sched_policy = SCHED_FIFO,
                                              .sched_priority = 70,
                                              .nice_value = 0,
                                              .name = "sensor_io"};

inline const ThreadConfig kUdpRecvConfig14Core{.cpu_core = 7,
                                               .sched_policy = SCHED_FIFO,
                                               .sched_priority = 65,
                                               .nice_value = 0,
                                               .name = "udp_recv"};

inline const ThreadConfig kLoggingConfig14Core{.cpu_core = 8,
                                               .sched_policy = SCHED_OTHER,
                                               .sched_priority = 0,
                                               .nice_value = -5,
                                               .name = "logger"};

inline const ThreadConfig kAuxConfig14Core{.cpu_core = 9,
                                           .sched_policy = SCHED_OTHER,
                                           .sched_priority = 0,
                                           .nice_value = 0,
                                           .name = "aux"};

inline const ThreadConfig kPublishConfig14Core{.cpu_core = 9,
                                               .sched_policy = SCHED_OTHER,
                                               .sched_priority = 0,
                                               .nice_value = -3,
                                               .name = "rt_publish"};

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

// ── 16-core configuration (Phase 5: MPC main + 2 workers) ──────────────────
// cset shield isolates Core 4-8 → "user" cpuset (0 tasks).
// rtc_controller_manager lives in "system" cpuset (Core 0-3, 9-15).
// All threads MUST use system-cpuset cores to avoid pthread_setaffinity_np
// failures caused by cpuset boundary violations.
//
// Core 0-1:  OS / DDS / NIC IRQ
// Core 2:    RT Control           (SCHED_FIFO 90)
// Core 3:    Sensor I/O           (SCHED_FIFO 70)
// Core 4-8:  cset shield "user"   (reserved — reduces OS noise system-wide)
// Core 9:    MPC main             (Phase 5, SCHED_FIFO 60)
// Core 10:   MPC worker 0         (Phase 5, SCHED_FIFO 55)
// Core 11:   MPC worker 1         (Phase 5, SCHED_FIFO 55)
// Core 12:   UDP recv             (shifted from Core 9, SCHED_FIFO 65)
// Core 13:   Logging              (shifted from Core 10, 100 Hz CSV drain)
// Core 14:   Aux + Publish        (shifted from Core 11, E-STOP publisher)
// Core 15:   Spare / MuJoCo sim   (shifted from Core 12-13 pair)

inline const ThreadConfig kRtControlConfig16Core{.cpu_core = 2,
                                                 .sched_policy = SCHED_FIFO,
                                                 .sched_priority = 90,
                                                 .nice_value = 0,
                                                 .name = "rt_control"};

inline const ThreadConfig kSensorConfig16Core{.cpu_core = 3,
                                              .sched_policy = SCHED_FIFO,
                                              .sched_priority = 70,
                                              .nice_value = 0,
                                              .name = "sensor_io"};

inline const ThreadConfig kUdpRecvConfig16Core{
    .cpu_core = 12, // Shifted from Core 9 (now MPC main).
    .sched_policy = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value = 0,
    .name = "udp_recv"};

inline const ThreadConfig kLoggingConfig16Core{
    .cpu_core = 13, // Shifted from Core 10 (now MPC worker 0).
    .sched_policy = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value = -5,
    .name = "logger"};

inline const ThreadConfig kAuxConfig16Core{
    .cpu_core = 14, // Shifted from Core 11 (now MPC worker 1).
    .sched_policy = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value = 0,
    .name = "aux"};

inline const ThreadConfig kPublishConfig16Core{
    .cpu_core = 14, // Shares Core 14 with aux (both CFS).
    .sched_policy = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value = -3,
    .name = "rt_publish"};

// MPC on 16-core: full parallel solve capacity — main + 2 workers on three
// dedicated cores. Workers run FIFO 55 (< main's 60) so a long Aligator
// rollout on the main thread preempts waiting workers if they ever share
// a core due to thermal migration.
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

// ── Simulation core assignment (Tier 3, no RT scheduling) ────────────────────
// Used by mujoco_sim.launch.py for taskset pinning of MuJoCo threads.
// MuJoCo sim_thread runs physics computation (CPU-intensive, no RT
// constraints). MuJoCo viewer_thread handles GLFW rendering (GPU-bound, OS core
// OK).
struct SimCoreLayout {
  int sim_thread_core; // MuJoCo physics thread (-1 = no pinning)
  int viewer_core;     // GLFW viewer thread (-1 = OS cores, no pinning)
};

/// Returns MuJoCo simulation core layout based on physical core count.
/// Only pins sim_thread on 10+ core systems where a dedicated spare core
/// exists above the RT/MPC range. Avoids cores used by the unified RT
/// layout (rt_control/sensor/mpc_*/udp/log/aux) and cset shield ranges.
inline constexpr SimCoreLayout GetSimCoreLayout(int physical_cores) {
  // 16+: current layout — RT 2-3, shield 4-8, MPC 9-11, UDP 12, logging 13,
  // aux 14 → Sim 15. (Unchanged from pre-unified layout.)
  if (physical_cores >= 16)
    return {15, -1};
  // 14-15: unified layout — RT/MPC/IO on Cores 2-9 → first spare core is 10.
  if (physical_cores >= 14)
    return {10, -1};
  // 12-13: unified layout — RT/MPC/IO on Cores 2-9; Cores 10-11 are spare.
  // Core 10 hosts sim_thread; Core 11 remains for user shield / viewer.
  if (physical_cores >= 12)
    return {10, -1};
  // 10-11: unified layout — RT/MPC/IO on Cores 2-8; Core 9 is the spare.
  if (physical_cores >= 10)
    return {9, -1};
  // 8-9: RT uses Cores 2-7; no dedicated spare for sim. cset shield is
  // released in --sim mode by the launch script, letting MuJoCo roam
  // over the freed shield range under CFS.
  if (physical_cores >= 8)
    return {-1, -1};
  return {-1, -1}; // <8: no dedicated sim core
}

} // namespace rtc

#endif // RTC_BASE_THREAD_CONFIG_HPP_
