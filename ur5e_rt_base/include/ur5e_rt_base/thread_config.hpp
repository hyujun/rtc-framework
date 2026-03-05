#ifndef UR5E_RT_BASE_THREAD_CONFIG_HPP_
#define UR5E_RT_BASE_THREAD_CONFIG_HPP_

#include <sched.h>  // SCHED_FIFO, SCHED_OTHER, SCHED_RR
#include <string>

namespace ur5e_rt_controller {

// Thread configuration for RT control and scheduling
struct ThreadConfig {
  int         cpu_core;         // CPU affinity (0-based core index)
  int         sched_policy;     // SCHED_FIFO, SCHED_RR, or SCHED_OTHER
  int         sched_priority;   // 1-99 for SCHED_FIFO/RR, ignored for OTHER
  int         nice_value;       // -20 to 19 for SCHED_OTHER, ignored for FIFO/RR
  std::string name;             // Thread name for debugging (max 15 chars)
};

// Predefined thread configurations for 6-core systems
// Core 0-1: OS / DDS (isolated by isolcpus=2-5)
// Core 2: RT Control (500Hz + 50Hz E-STOP)
// Core 3: Sensor I/O (joint_state, target, hand, UDP recv)
// Core 4: Logging (100Hz file I/O)
// Core 5: Aux (estop_pub, health monitor)

inline constexpr ThreadConfig kRtControlConfig{
    .cpu_core       = 2,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 90,
    .nice_value     = 0,
    .name           = "rt_control"
};

inline constexpr ThreadConfig kSensorConfig{
    .cpu_core       = 3,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 70,
    .nice_value     = 0,
    .name           = "sensor_io"
};

inline constexpr ThreadConfig kUdpRecvConfig{
    .cpu_core       = 3,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 65,
    .nice_value     = 0,
    .name           = "udp_recv"
};

inline constexpr ThreadConfig kLoggingConfig{
    .cpu_core       = 4,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -5,
    .name           = "logger"
};

inline constexpr ThreadConfig kAuxConfig{
    .cpu_core       = 5,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = 0,
    .name           = "aux"
};

// 4-core fallback configuration
inline constexpr ThreadConfig kRtControlConfig4Core{
    .cpu_core       = 1,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 90,
    .nice_value     = 0,
    .name           = "rt_control"
};

inline constexpr ThreadConfig kSensorConfig4Core{
    .cpu_core       = 2,
    .sched_policy   = SCHED_FIFO,
    .sched_priority = 70,
    .nice_value     = 0,
    .name           = "sensor_io"
};

inline constexpr ThreadConfig kLoggingConfig4Core{
    .cpu_core       = 3,
    .sched_policy   = SCHED_OTHER,
    .sched_priority = 0,
    .nice_value     = -5,
    .name           = "logger"
};

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_BASE_THREAD_CONFIG_HPP_
