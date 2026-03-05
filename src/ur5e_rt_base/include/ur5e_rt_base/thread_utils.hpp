#ifndef UR5E_RT_BASE_THREAD_UTILS_HPP_
#define UR5E_RT_BASE_THREAD_UTILS_HPP_

#include "ur5e_rt_base/thread_config.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>   // std::min_element, std::max_element
#include <cstring>
#include <numeric>     // std::accumulate
#include <string>
#include <tuple>       // std::tuple
#include <vector>      // std::vector

namespace ur5e_rt_controller {

// Apply thread configuration (CPU affinity, scheduler policy, priority)
// Returns true on success, false on failure (e.g., insufficient permissions)
//
// Requirements:
// - CAP_SYS_NICE capability or membership in 'realtime' group
// - /etc/security/limits.conf: @realtime - rtprio 99
inline bool ApplyThreadConfig(const ThreadConfig& cfg) noexcept {
  // 1. Set CPU affinity
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cfg.cpu_core, &cpuset);

  if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
    return false;
  }

  // 2. Set scheduler policy and priority
  sched_param param{};

  if (cfg.sched_policy == SCHED_FIFO || cfg.sched_policy == SCHED_RR) {
    param.sched_priority = cfg.sched_priority;

    if (pthread_setschedparam(pthread_self(), cfg.sched_policy, &param) != 0) {
      // Failed to set RT scheduling - likely permission issue
      return false;
    }
  } else if (cfg.sched_policy == SCHED_OTHER) {
    // Set nice value for SCHED_OTHER
    if (setpriority(PRIO_PROCESS, 0, cfg.nice_value) != 0) {
      // Non-critical: nice() can fail but thread still works
    }

    // Set SCHED_OTHER explicitly
    param.sched_priority = 0;
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
  }

  // 3. Set thread name for debugging (max 15 chars + null terminator)
  char name_buf[16];
  std::strncpy(name_buf, cfg.name.c_str(), sizeof(name_buf) - 1);
  name_buf[sizeof(name_buf) - 1] = '\0';
  pthread_setname_np(pthread_self(), name_buf);

  return true;
}

// Verify current thread configuration and return as string
// Useful for logging and debugging
inline std::string VerifyThreadConfig() noexcept {
  std::string result;

  // Get CPU affinity
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  if (pthread_getaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) == 0) {
    result += "CPU affinity: ";
    for (int i = 0; i < CPU_SETSIZE; ++i) {
      if (CPU_ISSET(i, &cpuset)) {
        result += std::to_string(i) + " ";
      }
    }
    result += "\n";
  }

  // Get scheduler policy and priority
  int policy;
  sched_param param;
  if (pthread_getschedparam(pthread_self(), &policy, &param) == 0) {
    result += "Scheduler: ";
    switch (policy) {
      case SCHED_FIFO:  result += "SCHED_FIFO"; break;
      case SCHED_RR:    result += "SCHED_RR"; break;
      case SCHED_OTHER: result += "SCHED_OTHER"; break;
      default:          result += "UNKNOWN"; break;
    }
    result += ", Priority: " + std::to_string(param.sched_priority) + "\n";
  }

  // Get nice value (for SCHED_OTHER)
  errno = 0;
  int nice_val = getpriority(PRIO_PROCESS, 0);
  if (errno == 0) {
    result += "Nice value: " + std::to_string(nice_val) + "\n";
  }

  // Get thread name
  char name[16];
  if (pthread_getname_np(pthread_self(), name, sizeof(name)) == 0) {
    result += "Thread name: " + std::string(name) + "\n";
  }

  return result;
}

// Get thread statistics (for jitter measurement)
// Returns {min_latency_us, max_latency_us, avg_latency_us}
inline std::tuple<double, double, double> GetThreadStats(
    const std::vector<double>& latencies_us) noexcept {
  if (latencies_us.empty()) {
    return {0.0, 0.0, 0.0};
  }

  double min_val = *std::min_element(latencies_us.begin(), latencies_us.end());
  double max_val = *std::max_element(latencies_us.begin(), latencies_us.end());
  double sum = std::accumulate(latencies_us.begin(), latencies_us.end(), 0.0);
  double avg = sum / latencies_us.size();

  return {min_val, max_val, avg};
}

// Returns the number of online logical CPUs on the current system.
inline int GetOnlineCpuCount() noexcept {
  const int n = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
  return (n > 0) ? n : 1;
}

// Aggregated thread configs selected at runtime for all four executor threads.
struct SystemThreadConfigs {
  ThreadConfig rt_control;
  ThreadConfig sensor;
  ThreadConfig logging;
  ThreadConfig aux;
};

// Selects the appropriate ThreadConfig set based on the number of online CPUs.
// >=6 cores: uses the standard 6-core configs (Cores 2-5).
// <6  cores: falls back to 4-core configs (Cores 1-3); aux shares Core 3.
inline SystemThreadConfigs SelectThreadConfigs() noexcept {
  if (GetOnlineCpuCount() >= 6) {
    return {kRtControlConfig, kSensorConfig, kLoggingConfig, kAuxConfig};
  }
  return {kRtControlConfig4Core, kSensorConfig4Core,
          kLoggingConfig4Core,   kLoggingConfig4Core};
}

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_BASE_THREAD_UTILS_HPP_
