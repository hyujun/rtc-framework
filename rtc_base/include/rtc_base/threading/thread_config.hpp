#ifndef RTC_BASE_THREAD_CONFIG_HPP_
#define RTC_BASE_THREAD_CONFIG_HPP_

#include <sched.h>  // SCHED_FIFO, SCHED_OTHER, SCHED_RR

#include <array>

namespace rtc {

// Thread configuration for RT control and scheduling
struct ThreadConfig {
  int         cpu_core;         // CPU affinity (0-based core index)
  int         sched_policy;     // SCHED_FIFO, SCHED_RR, or SCHED_OTHER
  int         sched_priority;   // 1-99 for SCHED_FIFO/RR, ignored for OTHER
  int         nice_value;       // -20 to 19 for SCHED_OTHER, ignored for FIFO/RR
  const char* name;             // Thread name for debugging (max 15 chars)
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
  int          num_workers{0};
  std::array<ThreadConfig, kMpcMaxWorkers> workers{};
};

// ── 6-core configuration ────────────────────────────────────────────────────
// Core 0-1: OS / DDS / NIC IRQ  (isolated by isolcpus=2-5)
// Core 2:   RT Control           (500 Hz ControlLoop + 50 Hz E-STOP watchdog)
// Core 3:   Sensor I/O           (joint_state, target, hand callbacks — dedicated)
// Core 4:   Logging              (100 Hz CSV drain)
// Core 5:   UDP recv + Aux       (udp_recv FIFO 65, aux SCHED_OTHER 0)
//
// Note: udp_recv is on Core 5 (not Core 3) to avoid contention with sensor_io.
//       Even under UDP burst, JointStateCallback latency is unaffected.

inline const ThreadConfig kRtControlConfig{
    .cpu_core       = 2,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 90,
    .nice_value     = 0,
    .name           = "rt_control"
};

inline const ThreadConfig kSensorConfig{
    .cpu_core       = 3,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 70,
    .nice_value     = 0,
    .name           = "sensor_io"
};

inline const ThreadConfig kUdpRecvConfig{
    .cpu_core       = 5,       // Moved from Core 3 → Core 5 (dedicated, no sensor_io contention)
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value     = 0,
    .name           = "udp_recv"
};

inline const ThreadConfig kLoggingConfig{
    .cpu_core       = 4,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -5,
    .name           = "logger"
};

inline const ThreadConfig kAuxConfig{
    .cpu_core       = 5,       // Shares Core 5 with udp_recv (aux is event-driven, very light)
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = 0,
    .name           = "aux"
};

// ── Publish offload thread (6-core) ──────────────────────────────────────────
// Drains SPSC publish buffer and calls all ROS2 publish() on a non-RT core.
// Shares Core 5 with aux and udp_recv — publish is SCHED_OTHER, preempted
// by udp_recv (SCHED_FIFO 65).

inline const ThreadConfig kPublishConfig{
    .cpu_core       = 5,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -3,
    .name           = "rt_publish"
};

// MPC on 6-core: piggybacks on Core 4 with SCHED_FIFO 60. Logging also
// lives on Core 4 (SCHED_OTHER); MPC FIFO preempts logging whenever a
// solve is active. This is acceptable because logging is drain-only.
inline const MpcThreadConfig kMpcConfig6Core{
    .main = ThreadConfig{
        .cpu_core       = 4,
        .sched_policy   = SCHED_FIFO,
        .sched_priority = 60,
        .nice_value     = 0,
        .name           = "mpc_main",
    },
    .num_workers = 0,
    .workers     = {},
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

inline const ThreadConfig kRtControlConfig8Core{
    .cpu_core       = 2,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 90,
    .nice_value     = 0,
    .name           = "rt_control"
};

inline const ThreadConfig kSensorConfig8Core{
    .cpu_core       = 3,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 70,
    .nice_value     = 0,
    .name           = "sensor_io"
};

inline const ThreadConfig kUdpRecvConfig8Core{
    .cpu_core       = 5,       // Shifted from Core 4 (now MPC main).
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value     = 0,
    .name           = "udp_recv"
};

inline const ThreadConfig kLoggingConfig8Core{
    .cpu_core       = 6,       // Shifted from Core 5.
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -5,
    .name           = "logger"
};

inline const ThreadConfig kAuxConfig8Core{
    .cpu_core       = 7,       // Shifted from Core 6.
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = 0,
    .name           = "aux"
};

inline const ThreadConfig kPublishConfig8Core{
    .cpu_core       = 7,       // Shifted from Core 6.
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -3,
    .name           = "rt_publish"
};

// MPC main on Core 4 (dedicated). No workers on 8-core — too few spare
// cores to host them without contention. Priority 60 < sensor(70) so a
// long solve can be preempted by sensor callbacks.
inline const MpcThreadConfig kMpcConfig8Core{
    .main = ThreadConfig{
        .cpu_core       = 4,
        .sched_policy   = SCHED_FIFO,
        .sched_priority = 60,
        .nice_value     = 0,
        .name           = "mpc_main",
    },
    .num_workers = 0,
    .workers     = {},
};

// ── 4-core fallback ─────────────────────────────────────────────────────────
// Core 0:   OS / DDS / IRQ
// Core 1:   RT Control
// Core 2:   Sensor I/O + UDP recv (shared — best effort)
// Core 3:   Logging + Aux

inline const ThreadConfig kRtControlConfig4Core{
    .cpu_core       = 1,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 90,
    .nice_value     = 0,
    .name           = "rt_control"
};

inline const ThreadConfig kSensorConfig4Core{
    .cpu_core       = 2,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 70,
    .nice_value     = 0,
    .name           = "sensor_io"
};

inline const ThreadConfig kUdpRecvConfig4Core{
    .cpu_core       = 2,       // Shares Core 2 with sensor_io (4-core: unavoidable)
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value     = 0,
    .name           = "udp_recv"
};

inline const ThreadConfig kLoggingConfig4Core{
    .cpu_core       = 3,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -5,
    .name           = "logger"
};

inline const ThreadConfig kAuxConfig4Core{
    .cpu_core       = 3,       // Shares Core 3 with logging (4-core: unavoidable)
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = 0,
    .name           = "aux"
};

inline const ThreadConfig kPublishConfig4Core{
    .cpu_core       = 3,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -3,
    .name           = "rt_publish"
};

// MPC on 4-core: piggybacks on Core 3 with SCHED_OTHER (degraded mode).
// With only 3 non-OS cores a FIFO MPC would starve logging/aux; CFS is
// the safer choice. Consumers are expected to reduce MPC frequency to
// 10 Hz on this tier.
inline const MpcThreadConfig kMpcConfig4Core{
    .main = ThreadConfig{
        .cpu_core       = 3,
        .sched_policy   = SCHED_OTHER,
        .sched_priority = 0,
        .nice_value     = -5,
        .name           = "mpc_main",
    },
    .num_workers = 0,
    .workers     = {},
};

// ── 10-core configuration ───────────────────────────────────────────────────
// cset shield isolates Core 2-6 → "user" cpuset.
// rt_controller lives in "system" cpuset (Core 0-1, 7-9).
// Only 3 non-OS system cores available → udp_recv shares with logging/aux.
//
// Core 0-1:  OS / DDS / NIC IRQ
// Core 2-6:  cset shield "user"   (reserved — reduces OS noise system-wide)
// Core 7:    RT Control           (SCHED_FIFO 90)
// Core 8:    Sensor I/O           (SCHED_FIFO 70)
// Core 9:    UDP recv + Logging + Aux  (shared — udp_recv preempts via FIFO)

inline const ThreadConfig kRtControlConfig10Core{
    .cpu_core       = 7,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 90,
    .nice_value     = 0,
    .name           = "rt_control"
};

inline const ThreadConfig kSensorConfig10Core{
    .cpu_core       = 8,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 70,
    .nice_value     = 0,
    .name           = "sensor_io"
};

inline const ThreadConfig kUdpRecvConfig10Core{
    .cpu_core       = 9,       // Shared — SCHED_FIFO preempts logging/aux
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value     = 0,
    .name           = "udp_recv"
};

inline const ThreadConfig kLoggingConfig10Core{
    .cpu_core       = 9,       // Shared with udp_recv — preempted by FIFO 65
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -5,
    .name           = "logger"
};

inline const ThreadConfig kAuxConfig10Core{
    .cpu_core       = 9,       // Shared with udp_recv + logging
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = 0,
    .name           = "aux"
};

inline const ThreadConfig kPublishConfig10Core{
    .cpu_core       = 9,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -3,
    .name           = "rt_publish"
};

// MPC on 10-core: Core 9 shared with udp_recv/logging/aux/publish.
// SCHED_FIFO 60 < udp_recv 65 so UDP traffic still preempts MPC solves.
// This tier is a degraded configuration — 12+ core is recommended when
// MPC is enabled.
inline const MpcThreadConfig kMpcConfig10Core{
    .main = ThreadConfig{
        .cpu_core       = 9,
        .sched_policy   = SCHED_FIFO,
        .sched_priority = 60,
        .nice_value     = 0,
        .name           = "mpc_main",
    },
    .num_workers = 0,
    .workers     = {},
};

// ── 12-core configuration (Phase 5: MPC main + 1 worker) ───────────────────
// cset shield isolates Core 2-6 → "user" cpuset.
// rt_controller lives in "system" cpuset (Core 0-1, 7-11).
// 5 non-OS system cores → MPC gets Cores 9-10, UDP/logging/aux share Core 11.
//
// Core 0-1:  OS / DDS / NIC IRQ
// Core 2-6:  cset shield "user"   (reserved — reduces OS noise system-wide)
// Core 7:    RT Control           (SCHED_FIFO 90)
// Core 8:    Sensor I/O           (SCHED_FIFO 70)
// Core 9:    MPC main             (Phase 5, SCHED_FIFO 60)
// Core 10:   MPC worker 0         (Phase 5, SCHED_FIFO 55)
// Core 11:   UDP recv + logging + aux + publish (shared, udp_recv preempts)

inline const ThreadConfig kRtControlConfig12Core{
    .cpu_core       = 7,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 90,
    .nice_value     = 0,
    .name           = "rt_control"
};

inline const ThreadConfig kSensorConfig12Core{
    .cpu_core       = 8,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 70,
    .nice_value     = 0,
    .name           = "sensor_io"
};

inline const ThreadConfig kUdpRecvConfig12Core{
    .cpu_core       = 11,      // Shifted from Core 9 (now MPC main).
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 65,       // Still preempts logging/aux on Core 11
    .nice_value     = 0,
    .name           = "udp_recv"
};

inline const ThreadConfig kLoggingConfig12Core{
    .cpu_core       = 11,      // Shifted from Core 10 (now MPC worker 0).
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -5,
    .name           = "logger"
};

inline const ThreadConfig kAuxConfig12Core{
    .cpu_core       = 11,      // System cpuset — outside shield (2-6)
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = 0,
    .name           = "aux"
};

inline const ThreadConfig kPublishConfig12Core{
    .cpu_core       = 11,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -3,
    .name           = "rt_publish"
};

// MPC on 12-core: main + 1 worker enable parallel linear-solve strategies
// (e.g. Aligator's parallel rollout). Worker priority < main so the
// worker never preempts the main solve on its own core; kept as FIFO to
// stay ahead of any CFS task drifting in from the shield boundary.
inline const MpcThreadConfig kMpcConfig12Core{
    .main = ThreadConfig{
        .cpu_core       = 9,
        .sched_policy   = SCHED_FIFO,
        .sched_priority = 60,
        .nice_value     = 0,
        .name           = "mpc_main",
    },
    .num_workers = 1,
    .workers = {
        ThreadConfig{
            .cpu_core       = 10,
            .sched_policy   = SCHED_FIFO,
            .sched_priority = 55,
            .nice_value     = 0,
            .name           = "mpc_worker_0",
        },
        ThreadConfig{},
    },
};

// ── 16-core configuration (Phase 5: MPC main + 2 workers) ──────────────────
// cset shield isolates Core 4-8 → "user" cpuset (0 tasks).
// rt_controller lives in "system" cpuset (Core 0-3, 9-15).
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

inline const ThreadConfig kRtControlConfig16Core{
    .cpu_core       = 2,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 90,
    .nice_value     = 0,
    .name           = "rt_control"
};

inline const ThreadConfig kSensorConfig16Core{
    .cpu_core       = 3,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 70,
    .nice_value     = 0,
    .name           = "sensor_io"
};

inline const ThreadConfig kUdpRecvConfig16Core{
    .cpu_core       = 12,      // Shifted from Core 9 (now MPC main).
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value     = 0,
    .name           = "udp_recv"
};

inline const ThreadConfig kLoggingConfig16Core{
    .cpu_core       = 13,      // Shifted from Core 10 (now MPC worker 0).
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -5,
    .name           = "logger"
};

inline const ThreadConfig kAuxConfig16Core{
    .cpu_core       = 14,      // Shifted from Core 11 (now MPC worker 1).
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = 0,
    .name           = "aux"
};

inline const ThreadConfig kPublishConfig16Core{
    .cpu_core       = 14,      // Shares Core 14 with aux (both CFS).
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -3,
    .name           = "rt_publish"
};

// MPC on 16-core: full parallel solve capacity — main + 2 workers on three
// dedicated cores. Workers run FIFO 55 (< main's 60) so a long Aligator
// rollout on the main thread preempts waiting workers if they ever share
// a core due to thermal migration.
inline const MpcThreadConfig kMpcConfig16Core{
    .main = ThreadConfig{
        .cpu_core       = 9,
        .sched_policy   = SCHED_FIFO,
        .sched_priority = 60,
        .nice_value     = 0,
        .name           = "mpc_main",
    },
    .num_workers = 2,
    .workers = {
        ThreadConfig{
            .cpu_core       = 10,
            .sched_policy   = SCHED_FIFO,
            .sched_priority = 55,
            .nice_value     = 0,
            .name           = "mpc_worker_0",
        },
        ThreadConfig{
            .cpu_core       = 11,
            .sched_policy   = SCHED_FIFO,
            .sched_priority = 55,
            .nice_value     = 0,
            .name           = "mpc_worker_1",
        },
    },
};

// ── Simulation core assignment (Tier 3, no RT scheduling) ────────────────────
// Used by mujoco_sim.launch.py for taskset pinning of MuJoCo threads.
// MuJoCo sim_thread runs physics computation (CPU-intensive, no RT constraints).
// MuJoCo viewer_thread handles GLFW rendering (GPU-bound, OS core OK).
struct SimCoreLayout {
  int sim_thread_core;    // MuJoCo physics thread (-1 = no pinning)
  int viewer_core;        // GLFW viewer thread (-1 = OS cores, no pinning)
};

/// Returns MuJoCo simulation core layout based on physical core count.
/// Only pins sim_thread on 8+ core systems where dedicated Tier 3 cores exist.
/// Avoids cores used by RT thread layouts (including Phase 5 MPC cores)
/// and cset shield ranges.
inline constexpr SimCoreLayout GetSimCoreLayout(int physical_cores) {
  // 16+: RT 2-3, shield 4-8, MPC 9-11, UDP 12, logging 13, aux 14 → Sim 15.
  if (physical_cores >= 16) return {15, -1};
  // 12+: RT 7-8, MPC 9-10, shared IO on 11 → no dedicated sim core.
  if (physical_cores >= 12) return {-1, -1};
  // 10+: RT 7-8, MPC shares Core 9 with IO → no dedicated sim core.
  if (physical_cores >= 10) return {-1, -1};
  // 8: RT 2-3, MPC 4, UDP 5, logging 6, aux 7 → sim on shield-adjacent core
  // is unavailable in sim mode (cset frees 2-6 for "user" when --sim). The
  // launch script handles this by releasing shield for sim, so we return
  // -1 and rely on pure CFS scheduling.
  if (physical_cores >= 8)  return {-1, -1};
  return {-1, -1};                            // <8:  no dedicated sim core
}

}  // namespace rtc

#endif  // RTC_BASE_THREAD_CONFIG_HPP_
