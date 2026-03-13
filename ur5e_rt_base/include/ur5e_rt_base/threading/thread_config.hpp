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

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_BASE_THREAD_CONFIG_HPP_
