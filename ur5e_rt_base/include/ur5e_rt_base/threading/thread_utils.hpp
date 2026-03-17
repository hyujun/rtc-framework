#ifndef UR5E_RT_BASE_THREAD_UTILS_HPP_
#define UR5E_RT_BASE_THREAD_UTILS_HPP_

#include "ur5e_rt_base/threading/thread_config.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>

#include <algorithm> // std::min_element, std::max_element
#include <cerrno>   // errno
#include <cmath>    // std::sqrt
#include <cstdio>   // fopen, fclose, fscanf
#include <cstring>
#include <map>     // std::map
#include <numeric> // std::accumulate
#include <set>     // std::set
#include <string>
#include <tuple>  // std::tuple
#include <utility> // std::pair
#include <vector> // std::vector

namespace ur5e_rt_controller
{

  // Forward declarations
  inline int GetOnlineCpuCount() noexcept;

  // Thread-safe alternative to std::strerror().
  // strerror() uses a static buffer and is not thread-safe; strerror_r()
  // writes to a caller-provided buffer, avoiding data races when multiple
  // threads call it concurrently (e.g. during startup).
  inline std::string SafeStrerror(int errnum) noexcept
  {
    char buf[128];
#if (_POSIX_C_SOURCE >= 200112L) && !defined(_GNU_SOURCE)
    // XSI-compliant strerror_r: returns int
    if (strerror_r(errnum, buf, sizeof(buf)) == 0)
    {
      return std::string(buf);
    }
    return "Unknown error " + std::to_string(errnum);
#else
    // GNU strerror_r: returns char* (may or may not use buf)
    const char *result = strerror_r(errnum, buf, sizeof(buf));
    return std::string(result);
#endif
  }

  // Validate ThreadConfig before applying
  // Returns empty string if valid, error message if invalid
  inline std::string ValidateThreadConfig(const ThreadConfig &cfg) noexcept
  {
    std::string errors;
    const int max_cores = GetOnlineCpuCount();

    // Validate CPU core
    if (cfg.cpu_core < 0 || cfg.cpu_core >= max_cores)
    {
      errors += "Invalid CPU core " + std::to_string(cfg.cpu_core) +
                " (valid range: 0-" + std::to_string(max_cores - 1) + "); ";
    }

    // Validate scheduler policy
    if (cfg.sched_policy != SCHED_FIFO && cfg.sched_policy != SCHED_RR &&
        cfg.sched_policy != SCHED_OTHER)
    {
      errors += "Invalid scheduler policy " + std::to_string(cfg.sched_policy) + "; ";
    }

    // Validate priorities for RT scheduling
    if ((cfg.sched_policy == SCHED_FIFO || cfg.sched_policy == SCHED_RR) &&
        (cfg.sched_priority < 1 || cfg.sched_priority > 99))
    {
      errors += "RT priority must be 1-99 for SCHED_FIFO/RR; ";
    }

    // Validate nice value for SCHED_OTHER
    if (cfg.sched_policy == SCHED_OTHER &&
        (cfg.nice_value < -20 || cfg.nice_value > 19))
    {
      errors += "Nice value must be -20 to 19 for SCHED_OTHER; ";
    }

    // Validate thread name
    if (!cfg.name || std::strlen(cfg.name) > 15)
    {
      errors += "Thread name must be non-null and <= 15 characters; ";
    }

    return errors;
  }

  // Apply thread configuration (CPU affinity, scheduler policy, priority)
  // Returns true on success, false on failure (e.g., insufficient permissions)
  //
  // Requirements:
  // - CAP_SYS_NICE capability or membership in 'realtime' group
  // - /etc/security/limits.conf: @realtime - rtprio 99
  inline bool ApplyThreadConfig(const ThreadConfig &cfg) noexcept
  {
    // Validate configuration first
    std::string validation_errors = ValidateThreadConfig(cfg);
    if (!validation_errors.empty())
    {
      // In a real system, you'd log these errors
      return false;
    }

    // 1. Set CPU affinity
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cfg.cpu_core, &cpuset);

    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0)
    {
      return false;
    }

    // 2. Set scheduler policy and priority
    sched_param param{};

    if (cfg.sched_policy == SCHED_FIFO || cfg.sched_policy == SCHED_RR)
    {
      param.sched_priority = cfg.sched_priority;

      if (pthread_setschedparam(pthread_self(), cfg.sched_policy, &param) != 0)
      {
        // Failed to set RT scheduling - likely permission issue
        return false;
      }
    } else if (cfg.sched_policy == SCHED_OTHER) {
      // Set nice value for SCHED_OTHER
      if (setpriority(PRIO_PROCESS, 0, cfg.nice_value) != 0)
      {
        // Non-critical: nice() can fail but thread still works
      }

      // Set SCHED_OTHER explicitly
      param.sched_priority = 0;
      pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    }

    // 3. Set thread name for debugging (max 15 chars + null terminator)
    char name_buf[16];
    std::strncpy(name_buf, cfg.name, sizeof(name_buf) - 1);
    name_buf[sizeof(name_buf) - 1] = '\0';
#ifdef __APPLE__
    pthread_setname_np(name_buf);
#else
    pthread_setname_np(pthread_self(), name_buf);
#endif

    return true;
  }

  // Apply thread configuration with graceful fallback
  // Attempts to apply as much configuration as possible, even if RT scheduling fails
  // Returns {full_success, warnings} - full_success is true only if everything succeeded
  inline std::pair<bool, std::string> ApplyThreadConfigWithFallback(const ThreadConfig &cfg) noexcept
  {
    std::string validation_errors = ValidateThreadConfig(cfg);
    if (!validation_errors.empty())
    {
      return {false, "Validation failed: " + validation_errors};
    }

    bool full_success = true;
    std::string warnings;

    // 1. Always try CPU affinity first (critical for isolation)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cfg.cpu_core, &cpuset);

    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0)
    {
      full_success = false;
      warnings += "CPU affinity failed: " + SafeStrerror(errno) + "; ";
    }

    // 2. Try RT scheduling, fallback to SCHED_OTHER if it fails
    sched_param param{};
    bool rt_success = false;

    if (cfg.sched_policy == SCHED_FIFO || cfg.sched_policy == SCHED_RR)
    {
      param.sched_priority = cfg.sched_priority;
      if (pthread_setschedparam(pthread_self(), cfg.sched_policy, &param) == 0)
      {
        rt_success = true;
      } else {
        full_success = false;
        warnings += "RT scheduling failed, falling back to SCHED_OTHER: " +
                    SafeStrerror(errno) + "; ";
      }
    }

    if (!rt_success)
    {
      // Apply SCHED_OTHER configuration
      if (cfg.sched_policy == SCHED_OTHER)
      {
        param.sched_priority = 0;
        if (setpriority(PRIO_PROCESS, 0, cfg.nice_value) != 0)
        {
          warnings += "Nice value setting failed: " + SafeStrerror(errno) + "; ";
        }
      } else {
        // Fallback: set nice value for RT policies that failed
        param.sched_priority = 0;
        if (setpriority(PRIO_PROCESS, 0, cfg.nice_value) != 0)
        {
          warnings += "Fallback nice value setting failed: " + SafeStrerror(errno) + "; ";
        }
      }
      pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    }

    // 3. Set thread name (usually succeeds, but log if it doesn't)
    char name_buf[16];
    std::strncpy(name_buf, cfg.name, sizeof(name_buf) - 1);
    name_buf[sizeof(name_buf) - 1] = '\0';
#ifdef __APPLE__
    if (pthread_setname_np(name_buf) != 0)
    {
      warnings += "Thread name setting failed: " + SafeStrerror(errno) + "; ";
    }
#else
    if (pthread_setname_np(pthread_self(), name_buf) != 0)
    {
      warnings += "Thread name setting failed: " + SafeStrerror(errno) + "; ";
    }
#endif

    return {full_success, warnings};
  }

  // Verify current thread configuration and return as string
  // Useful for logging and debugging
  inline std::string VerifyThreadConfig() noexcept
  {
    std::string result;

    // Get CPU affinity
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    if (pthread_getaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) == 0)
    {
      result += "CPU affinity: ";
      for (int i = 0; i < CPU_SETSIZE; ++i)
      {
        if (CPU_ISSET(i, &cpuset))
        {
          result += std::to_string(i) + " ";
        }
      }
      result += "\n";
    }

    // Get scheduler policy and priority
    int policy;
    sched_param param;
    if (pthread_getschedparam(pthread_self(), &policy, &param) == 0)
    {
      result += "Scheduler: ";
      switch (policy)
      {
      case SCHED_FIFO:
        result += "SCHED_FIFO";
        break;
      case SCHED_RR:
        result += "SCHED_RR";
        break;
      case SCHED_OTHER:
        result += "SCHED_OTHER";
        break;
      default:
        result += "UNKNOWN";
        break;
      }
      result += ", Priority: " + std::to_string(param.sched_priority) + "\n";
    }

    // Get nice value (for SCHED_OTHER)
    errno = 0;
    int nice_val = getpriority(PRIO_PROCESS, 0);
    if (errno == 0)
    {
      result += "Nice value: " + std::to_string(nice_val) + "\n";
    }

    // Get thread name
    char name[16];
    if (pthread_getname_np(pthread_self(), name, sizeof(name)) == 0)
    {
      result += "Thread name: " + std::string(name) + "\n";
    }

    return result;
  }

  // Get thread statistics (for jitter measurement).
  // Returns {min_latency_us, max_latency_us, avg_latency_us}.
  //
  // WARNING: NOT RT-safe — accepts std::vector (heap-allocated).
  // Call only from non-RT threads (e.g. logging, monitoring).
  inline std::tuple<double, double, double> GetThreadStats(
      const std::vector<double> &latencies_us) noexcept
  {
    if (latencies_us.empty())
    {
      return {0.0, 0.0, 0.0};
    }

    double min_val = *std::min_element(latencies_us.begin(), latencies_us.end());
    double max_val = *std::max_element(latencies_us.begin(), latencies_us.end());
    double sum = std::accumulate(latencies_us.begin(), latencies_us.end(), 0.0);
    double avg = sum / latencies_us.size();

    return {min_val, max_val, avg};
  }

// Enhanced thread metrics for comprehensive monitoring
struct ThreadMetrics
{
  double min_latency_us;
  double max_latency_us;
  double avg_latency_us;
  double jitter_us;        // Standard deviation of latencies
  double percentile_95_us; // 95th percentile latency
  double percentile_99_us; // 99th percentile latency
};

  // Get comprehensive thread statistics with percentiles.
  // Returns ThreadMetrics with latency analysis.
  //
  // WARNING: NOT RT-safe — copies and sorts the input vector (heap allocation).
  // Call only from non-RT threads (e.g. logging, monitoring).
  inline ThreadMetrics GetThreadMetrics(const std::vector<double> &latencies_us) noexcept
  {
    ThreadMetrics metrics{};

    if (latencies_us.empty())
    {
      return metrics;
    }

    // Sort for percentile calculations
    std::vector<double> sorted_latencies = latencies_us;
    std::sort(sorted_latencies.begin(), sorted_latencies.end());

    // Basic stats
    metrics.min_latency_us = sorted_latencies.front();
    metrics.max_latency_us = sorted_latencies.back();

    double sum = 0.0;
    double sum_sq = 0.0;
    for (double lat : latencies_us)
    {
      sum += lat;
      sum_sq += lat * lat;
    }
    metrics.avg_latency_us = sum / latencies_us.size();

    // Jitter (standard deviation)
    double variance = (sum_sq / latencies_us.size()) - (metrics.avg_latency_us * metrics.avg_latency_us);
    metrics.jitter_us = std::sqrt(std::max(0.0, variance));

    // Percentiles
    size_t n = sorted_latencies.size();
    size_t idx_95 = static_cast<size_t>(0.95 * (n - 1));
    size_t idx_99 = static_cast<size_t>(0.99 * (n - 1));
    metrics.percentile_95_us = sorted_latencies[idx_95];
    metrics.percentile_99_us = sorted_latencies[idx_99];

    return metrics;
  }

  // RT-safe thread health check using bitfield flags instead of strings.
  // Zero heap allocation — safe to call periodically from the 500 Hz RT path.
  //
  // Usage:
  //   auto flags = CheckThreadHealthFast(&kRtControlConfig);
  //   if (flags != ThreadHealthFlag::kOk) { /* log or handle */ }
  enum class ThreadHealthFlag : uint8_t
  {
    kOk = 0,
    kWrongCore = 1 << 0,
    kPolicyChanged = 1 << 1,
    kPriorityChanged = 1 << 2,
    kNiceChanged = 1 << 3,
  };

  inline ThreadHealthFlag operator|(ThreadHealthFlag a, ThreadHealthFlag b) noexcept
  {
    return static_cast<ThreadHealthFlag>(
        static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
  }
  inline ThreadHealthFlag operator&(ThreadHealthFlag a, ThreadHealthFlag b) noexcept
  {
    return static_cast<ThreadHealthFlag>(
        static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
  }
  inline ThreadHealthFlag &operator|=(ThreadHealthFlag &a, ThreadHealthFlag b) noexcept
  {
    a = a | b;
    return a;
  }

  inline ThreadHealthFlag CheckThreadHealthFast(const ThreadConfig &expected) noexcept
  {
    ThreadHealthFlag flags = ThreadHealthFlag::kOk;

    // Check CPU affinity
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    if (pthread_getaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) == 0)
    {
      if (!CPU_ISSET(expected.cpu_core, &cpuset))
      {
        flags |= ThreadHealthFlag::kWrongCore;
      }
    }

    // Check scheduler policy and priority
    int policy;
    sched_param param;
    if (pthread_getschedparam(pthread_self(), &policy, &param) == 0)
    {
      if (policy != expected.sched_policy)
      {
        flags |= ThreadHealthFlag::kPolicyChanged;
      }
      if ((policy == SCHED_FIFO || policy == SCHED_RR) &&
          param.sched_priority != expected.sched_priority)
      {
        flags |= ThreadHealthFlag::kPriorityChanged;
      }
    }

    // Check nice value for SCHED_OTHER
    if (expected.sched_policy == SCHED_OTHER)
    {
      errno = 0;
      int nice_val = getpriority(PRIO_PROCESS, 0);
      if (errno == 0 && nice_val != expected.nice_value)
      {
        flags |= ThreadHealthFlag::kNiceChanged;
      }
    }

    return flags;
  }

  // Check thread health and configuration consistency.
  // Returns empty string if healthy, warnings/issues if problems detected.
  //
  // WARNING: NOT RT-safe — uses std::string (heap allocation).
  // For RT-safe health checks, use CheckThreadHealthFast() instead.
  inline std::string CheckThreadHealth(const ThreadConfig *expected_config = nullptr) noexcept
  {
    std::string issues;

    // Check if still on expected CPU core
    if (expected_config)
    {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      if (pthread_getaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) == 0)
      {
        bool on_expected_core = false;
        for (int i = 0; i < CPU_SETSIZE; ++i)
        {
          if (CPU_ISSET(i, &cpuset) && i == expected_config->cpu_core)
          {
            on_expected_core = true;
            break;
          }
        }
        if (!on_expected_core)
        {
          issues += "Thread not on expected CPU core " +
                    std::to_string(expected_config->cpu_core) + "; ";
        }
      }
    }

    // Check if RT priority is maintained
    int policy;
    sched_param param;
    if (pthread_getschedparam(pthread_self(), &policy, &param) == 0)
    {
      if (expected_config)
      {
        if (policy != expected_config->sched_policy)
        {
          issues += "Scheduler policy changed from expected " +
                    std::to_string(expected_config->sched_policy) + " to " +
                    std::to_string(policy) + "; ";
        }
        if ((policy == SCHED_FIFO || policy == SCHED_RR) &&
            param.sched_priority != expected_config->sched_priority)
        {
          issues += "RT priority changed from expected " +
                    std::to_string(expected_config->sched_priority) + " to " +
                    std::to_string(param.sched_priority) + "; ";
        }
      } else {
        // General health check without expected config
        if (policy == SCHED_OTHER)
        {
          issues += "Thread using SCHED_OTHER instead of RT policy; ";
        }
      }
    }

    // Check nice value for SCHED_OTHER
    if (expected_config && expected_config->sched_policy == SCHED_OTHER)
    {
      errno = 0;
      int nice_val = getpriority(PRIO_PROCESS, 0);
      if (errno == 0 && nice_val != expected_config->nice_value)
      {
        issues += "Nice value " + std::to_string(nice_val) +
                  " differs from expected " + std::to_string(expected_config->nice_value) + "; ";
      }
    }

    return issues;
  }

  // Returns the number of online logical CPUs on the current system.
  // Includes SMT/Hyper-Threading siblings (e.g., 6C/12T returns 12).
  // Use for CPU affinity validation (logical CPU IDs are valid up to this count).
  inline int GetOnlineCpuCount() noexcept
  {
    const int n = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    return (n > 0) ? n : 1;
  }

  // Returns the number of physical CPU cores (excluding SMT/HT siblings).
  // Parses sysfs topology to count unique (socket_id, core_id) pairs.
  // Falls back to GetOnlineCpuCount() if sysfs topology is unavailable.
  //
  // This is the correct metric for selecting thread layouts (4/6/8-core)
  // because thread_config.hpp assigns threads to physical cores.
  // Example: i7-8700 (6C/12T) → returns 6, not 12.
  inline int GetPhysicalCpuCount() noexcept
  {
    std::set<std::pair<int, int>> unique_cores;

    for (int cpu = 0; cpu < 1024; ++cpu)
    {
      char pkg_path[128];
      char core_path[128];

      std::snprintf(pkg_path, sizeof(pkg_path),
                    "/sys/devices/system/cpu/cpu%d/topology/physical_package_id", cpu);
      std::snprintf(core_path, sizeof(core_path),
                    "/sys/devices/system/cpu/cpu%d/topology/core_id", cpu);

      FILE *pkg_file = std::fopen(pkg_path, "r");
      if (!pkg_file)
      {
        break; // No more CPUs
      }
      int socket_id = 0;
      if (std::fscanf(pkg_file, "%d", &socket_id) != 1)
      {
        std::fclose(pkg_file);
        break;
      }
      std::fclose(pkg_file);

      FILE *core_file = std::fopen(core_path, "r");
      if (!core_file)
      {
        break;
      }
      int core_id = 0;
      if (std::fscanf(core_file, "%d", &core_id) != 1)
      {
        std::fclose(core_file);
        break;
      }
      std::fclose(core_file);

      unique_cores.insert({socket_id, core_id});
    }

    if (unique_cores.empty())
    {
      // Fallback: sysfs topology unavailable (VM, container, etc.)
      return GetOnlineCpuCount();
    }

    return static_cast<int>(unique_cores.size());
  }

// Aggregated thread configs selected at runtime for all threads.
struct SystemThreadConfigs
{
  ThreadConfig rt_control;
  ThreadConfig sensor;
  ThreadConfig udp_recv;        // Hand UDP receiver (separate from sensor_io)
  ThreadConfig logging;
  ThreadConfig aux;
  ThreadConfig publish;         // Non-RT publish offload thread
  ThreadConfig status_monitor;  // Non-RT status monitor (10 Hz)
  ThreadConfig hand_failure;    // Non-RT hand failure detector (50 Hz)
};

  // Validate SystemThreadConfigs for conflicts and invalid configurations.
  // Returns empty string if valid, error messages if invalid.
  //
  // Core sharing rules:
  //   - Two RT threads (SCHED_FIFO/RR) on the same core: allowed only if
  //     they have DIFFERENT priorities (higher prio preempts lower).
  //   - RT + non-RT (SCHED_OTHER) on the same core: always allowed
  //     (RT preempts SCHED_OTHER unconditionally).
  //   - Two non-RT threads on the same core: always allowed (CFS handles it).
  //   - Two RT threads with the SAME priority on the SAME core: ERROR
  //     (SCHED_FIFO with equal priority causes starvation of one thread).
  inline std::string ValidateSystemThreadConfigs(const SystemThreadConfigs &configs) noexcept
  {
    std::string errors;

    // Validate each individual config
    errors += ValidateThreadConfig(configs.rt_control);
    errors += ValidateThreadConfig(configs.sensor);
    errors += ValidateThreadConfig(configs.udp_recv);
    errors += ValidateThreadConfig(configs.logging);
    errors += ValidateThreadConfig(configs.aux);
    errors += ValidateThreadConfig(configs.publish);
    errors += ValidateThreadConfig(configs.status_monitor);
    errors += ValidateThreadConfig(configs.hand_failure);

    // Collect all configs with names for conflict analysis
    struct NamedConfig
    {
      const char *name;
      const ThreadConfig *config;
    };
    const std::array<NamedConfig, 8> all_configs = {{
        {"rt_control", &configs.rt_control},
        {"sensor", &configs.sensor},
        {"udp_recv", &configs.udp_recv},
        {"logging", &configs.logging},
        {"aux", &configs.aux},
        {"publish", &configs.publish},
        {"status_monitor", &configs.status_monitor},
        {"hand_failure", &configs.hand_failure},
    }};

    auto is_rt = [](const ThreadConfig *c)
    {
      return c->sched_policy == SCHED_FIFO || c->sched_policy == SCHED_RR;
    };

    // Check for problematic core sharing: only flag RT+RT same-priority conflicts
    for (std::size_t i = 0; i < all_configs.size(); ++i)
    {
      for (std::size_t j = i + 1; j < all_configs.size(); ++j)
      {
        const auto &a = all_configs[i];
        const auto &b = all_configs[j];

        if (a.config->cpu_core != b.config->cpu_core)
        {
          continue; // different cores — no conflict possible
        }

        // Same core: only a problem if both are RT with identical priority
        if (is_rt(a.config) && is_rt(b.config) &&
            a.config->sched_priority == b.config->sched_priority)
        {
          errors += "RT priority " + std::to_string(a.config->sched_priority) +
                    " conflict on core " + std::to_string(a.config->cpu_core) +
                    " between '" + a.name + "' and '" + b.name + "'; ";
        }
      }
    }

    return errors;
  }

  // Selects the appropriate ThreadConfig set based on the number of physical CPU cores.
  // Uses GetPhysicalCpuCount() (not GetOnlineCpuCount()) to avoid SMT/HT over-counting.
  // Example: i7-8700 (6C/12T) correctly selects 6-core layout, not 8-core.
  //
  // cset shield core allocation (robot mode):
  //   ≤4: 1-3 | 5-7: 2-5 | 8-15: 2-6 | 16+: 4-8
  // Thread layouts place threads OUTSIDE the shield range to avoid cpuset conflicts.
  //
  // >=16 cores: 16-core layout — threads on 0-3,9+ (shield 4-8).
  // >=12 cores: 12-core layout — threads on 0-1,7-11 (shield 2-6), dedicated cores.
  // >=10 cores: 10-core layout — threads on 0-1,7-9 (shield 2-6), shared Core 9.
  // >=8 cores:  8-core layout — threads on 2-6 (no shield-aware mapping).
  // >=6 cores:  6-core layout — threads on 2-5, udp_recv shares Core 5 with aux.
  // < 6 cores:  4-core fallback — threads on 1-3, udp_recv shares Core 2.
  //
  // Note: 8-9 core layout still uses cores 2-6 which overlap with shield 2-6.
  // On these systems, SCHED_FIFO provides RT protection without cpuset isolation.
  inline SystemThreadConfigs SelectThreadConfigs() noexcept
  {
    const int ncpu = GetPhysicalCpuCount();
    if (ncpu >= 16)
    {
      return {kRtControlConfig16Core, kSensorConfig16Core, kUdpRecvConfig16Core,
              kLoggingConfig16Core, kAuxConfig16Core, kPublishConfig16Core,
              kStatusMonitorConfig16Core, kHandFailureConfig16Core};
    }
    if (ncpu >= 12)
    {
      return {kRtControlConfig12Core, kSensorConfig12Core, kUdpRecvConfig12Core,
              kLoggingConfig12Core, kAuxConfig12Core, kPublishConfig12Core,
              kStatusMonitorConfig12Core, kHandFailureConfig12Core};
    }
    if (ncpu >= 10)
    {
      return {kRtControlConfig10Core, kSensorConfig10Core, kUdpRecvConfig10Core,
              kLoggingConfig10Core, kAuxConfig10Core, kPublishConfig10Core,
              kStatusMonitorConfig10Core, kHandFailureConfig10Core};
    }
    if (ncpu >= 8)
    {
      return {kRtControlConfig8Core, kSensorConfig8Core, kUdpRecvConfig8Core,
              kLoggingConfig8Core, kAuxConfig8Core, kPublishConfig8Core,
              kStatusMonitorConfig8Core, kHandFailureConfig8Core};
    }
    if (ncpu >= 6)
    {
      return {kRtControlConfig, kSensorConfig, kUdpRecvConfig,
              kLoggingConfig, kAuxConfig, kPublishConfig,
              kStatusMonitorConfig, kHandFailureConfig};
    }
    return {kRtControlConfig4Core, kSensorConfig4Core, kUdpRecvConfig4Core,
            kLoggingConfig4Core, kAuxConfig4Core, kPublishConfig4Core,
            kStatusMonitorConfig4Core, kHandFailureConfig4Core};
  }

} // namespace ur5e_rt_controller

#endif // UR5E_RT_BASE_THREAD_UTILS_HPP_
