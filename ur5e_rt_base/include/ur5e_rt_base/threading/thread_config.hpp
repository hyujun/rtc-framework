#ifndef UR5E_RT_BASE_THREAD_CONFIG_HPP_
#define UR5E_RT_BASE_THREAD_CONFIG_HPP_

#include <sched.h>  // SCHED_FIFO, SCHED_OTHER, SCHED_RR

namespace ur5e_rt_controller {

// Thread configuration for RT control and scheduling
struct ThreadConfig {
  int         cpu_core;         // CPU affinity (0-based core index)
  int         sched_policy;     // SCHED_FIFO, SCHED_RR, or SCHED_OTHER
  int         sched_priority;   // 1-99 for SCHED_FIFO/RR, ignored for OTHER
  int         nice_value;       // -20 to 19 for SCHED_OTHER, ignored for FIFO/RR
  const char* name;             // Thread name for debugging (max 15 chars)
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

// ── Non-RT monitoring threads (6-core) ──────────────────────────────────────
// Status monitor (10 Hz) and hand failure detector (50 Hz) are both non-RT.
// They share Core 4 with logging — all are SCHED_OTHER and I/O-light.

inline const ThreadConfig kStatusMonitorConfig{
    .cpu_core       = 4,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -2,
    .name           = "status_mon"
};

inline const ThreadConfig kHandFailureConfig{
    .cpu_core       = 4,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -2,
    .name           = "hand_detect"
};

// ── 8-core configuration ────────────────────────────────────────────────────
// Core 0-1: OS / DDS / NIC IRQ  (isolated by isolcpus=2-6)
// Core 2:   RT Control           (500 Hz ControlLoop + 50 Hz E-STOP watchdog)
// Core 3:   Sensor I/O           (joint_state, target, hand callbacks)
// Core 4:   UDP recv             (dedicated — fully isolated from sensor_io)
// Core 5:   Logging              (100 Hz CSV drain)
// Core 6:   Aux                  (E-STOP publisher)
// Core 7:   Spare                (monitoring, cyclictest measurement)

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
    .cpu_core       = 4,       // Dedicated core — fully isolated from sensor_io
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value     = 0,
    .name           = "udp_recv"
};

inline const ThreadConfig kLoggingConfig8Core{
    .cpu_core       = 5,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -5,
    .name           = "logger"
};

inline const ThreadConfig kAuxConfig8Core{
    .cpu_core       = 6,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = 0,
    .name           = "aux"
};

inline const ThreadConfig kPublishConfig8Core{
    .cpu_core       = 6,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -3,
    .name           = "rt_publish"
};

// ── Non-RT monitoring threads (8-core) ──────────────────────────────────────
// With 8 cores, monitoring threads get Core 6 (shared with aux).

inline const ThreadConfig kStatusMonitorConfig8Core{
    .cpu_core       = 6,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -2,
    .name           = "status_mon"
};

inline const ThreadConfig kHandFailureConfig8Core{
    .cpu_core       = 6,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -2,
    .name           = "hand_detect"
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

// ── Non-RT monitoring threads (4-core) ──────────────────────────────────────
// With 4 cores, monitoring threads share Core 3 with logging + aux.

inline const ThreadConfig kStatusMonitorConfig4Core{
    .cpu_core       = 3,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = 0,
    .name           = "status_mon"
};

inline const ThreadConfig kHandFailureConfig4Core{
    .cpu_core       = 3,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = 0,
    .name           = "hand_detect"
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

// ── Non-RT monitoring threads (10-core) ────────────────────────────────────

inline const ThreadConfig kStatusMonitorConfig10Core{
    .cpu_core       = 9,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -2,
    .name           = "status_mon"
};

inline const ThreadConfig kHandFailureConfig10Core{
    .cpu_core       = 9,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -2,
    .name           = "hand_detect"
};

// ── 12-core configuration ───────────────────────────────────────────────────
// cset shield isolates Core 2-6 → "user" cpuset.
// rt_controller lives in "system" cpuset (Core 0-1, 7-11).
// 5 non-OS system cores → dedicated core for each thread.
//
// Core 0-1:  OS / DDS / NIC IRQ
// Core 2-6:  cset shield "user"   (reserved — reduces OS noise system-wide)
// Core 7:    RT Control           (SCHED_FIFO 90)
// Core 8:    Sensor I/O           (SCHED_FIFO 70)
// Core 9:    UDP recv             (SCHED_FIFO 65)
// Core 10:   Logging + monitors   (100 Hz CSV drain)
// Core 11:   Aux                  (E-STOP publisher)

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
    .cpu_core       = 9,       // Dedicated — fully isolated from sensor_io
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value     = 0,
    .name           = "udp_recv"
};

inline const ThreadConfig kLoggingConfig12Core{
    .cpu_core       = 10,      // System cpuset — outside shield (2-6)
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

// ── Non-RT monitoring threads (12-core) ────────────────────────────────────
// Share Core 10 with logging — all SCHED_OTHER and I/O-light.

inline const ThreadConfig kStatusMonitorConfig12Core{
    .cpu_core       = 10,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -2,
    .name           = "status_mon"
};

inline const ThreadConfig kHandFailureConfig12Core{
    .cpu_core       = 10,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -2,
    .name           = "hand_detect"
};

// ── 16-core configuration ───────────────────────────────────────────────────
// cset shield isolates Core 4-8 → "user" cpuset (0 tasks).
// rt_controller lives in "system" cpuset (Core 0-3, 9-21).
// All threads MUST use system-cpuset cores to avoid pthread_setaffinity_np
// failures caused by cpuset boundary violations.
//
// Core 0-1:  OS / DDS / NIC IRQ
// Core 2:    RT Control           (SCHED_FIFO 90, protected by priority)
// Core 3:    Sensor I/O           (SCHED_FIFO 70, protected by priority)
// Core 4-8:  cset shield "user"   (reserved — reduces OS noise system-wide)
// Core 9:    UDP recv             (SCHED_FIFO 65)
// Core 10:   Logging              (100 Hz CSV drain)
// Core 11:   Aux                  (E-STOP publisher)

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
    .cpu_core       = 9,       // System cpuset — outside shield (4-8)
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value     = 0,
    .name           = "udp_recv"
};

inline const ThreadConfig kLoggingConfig16Core{
    .cpu_core       = 10,      // System cpuset — outside shield (4-8)
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -5,
    .name           = "logger"
};

inline const ThreadConfig kAuxConfig16Core{
    .cpu_core       = 11,      // System cpuset — outside shield (4-8)
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = 0,
    .name           = "aux"
};

inline const ThreadConfig kPublishConfig16Core{
    .cpu_core       = 11,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -3,
    .name           = "rt_publish"
};

// ── Non-RT monitoring threads (16-core) ────────────────────────────────────
// Share Core 10 with logging — all SCHED_OTHER and I/O-light.

inline const ThreadConfig kStatusMonitorConfig16Core{
    .cpu_core       = 10,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -2,
    .name           = "status_mon"
};

inline const ThreadConfig kHandFailureConfig16Core{
    .cpu_core       = 10,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -2,
    .name           = "hand_detect"
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
/// Avoids cores used by RT thread layouts and cset shield ranges.
inline constexpr SimCoreLayout GetSimCoreLayout(int physical_cores) {
  if (physical_cores >= 16) return {12, 13};  // 16+: RT 2-3,9-11, shield 4-8, Sim 12-13
  if (physical_cores >= 12) return {11, -1};  // 12+: RT 7-11, shield 2-6, Sim shares 11
  if (physical_cores >= 10) return {9, -1};   // 10+: RT 7-9, shield 2-6, Sim shares 9
  if (physical_cores >= 8)  return {7, -1};   // 8:   RT 2-6, Sim 7 (viewer on OS)
  return {-1, -1};                            // <8:  no dedicated sim core
}

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_BASE_THREAD_CONFIG_HPP_
