#ifndef RTC_BASE_THREAD_UTILS_HPP_
#define RTC_BASE_THREAD_UTILS_HPP_

#include "rtc_base/threading/cpu_topology.hpp"
#include "rtc_base/threading/thread_config.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>  // std::min_element, std::max_element
#include <array>      // std::array (NamedConfig table in ValidateSystemThreadConfigs)
#include <cerrno>     // errno
#include <cmath>      // std::sqrt
#include <cstdint>    // std::uint8_t (ThreadHealthFlag underlying type)
#include <cstdio>     // fopen, fclose, fscanf
#include <cstring>
#include <map>      // std::map
#include <numeric>  // std::accumulate
#include <set>      // std::set
#include <string>
#include <tuple>    // std::tuple
#include <utility>  // std::pair
#include <vector>   // std::vector

namespace rtc {

// Forward declarations
inline int GetOnlineCpuCount() noexcept;

// Thread-safe alternative to std::strerror().
// strerror() uses a static buffer and is not thread-safe; strerror_r()
// writes to a caller-provided buffer, avoiding data races when multiple
// threads call it concurrently (e.g. during startup).
inline std::string SafeStrerror(int errnum) noexcept {
  char buf[128];
#if (_POSIX_C_SOURCE >= 200112L) && !defined(_GNU_SOURCE)
  // XSI-compliant strerror_r: returns int
  if (strerror_r(errnum, buf, sizeof(buf)) == 0) {
    return std::string(buf);
  }
  return "Unknown error " + std::to_string(errnum);
#else
  // GNU strerror_r: returns char* (may or may not use buf)
  const char* result = strerror_r(errnum, buf, sizeof(buf));
  return std::string(result);
#endif
}

// Validate ThreadConfig before applying
// Returns empty string if valid, error message if invalid
inline std::string ValidateThreadConfig(const ThreadConfig& cfg) noexcept {
  std::string errors;

  // Validate CPU core. cpu_core is a *slot index*, not a logical CPU id, so
  // the upper bound is the number of unique physical cores (physical_core_slots
  // size) — falling back to logical count when topology detection is
  // unavailable (container without sysfs, where SlotToLogicalCpu degrades to
  // identity). cpu_core == -1 is a Phase 5 sentinel meaning "skip affinity,
  // inherit the calling process's taskset" — used by process-level pins
  // (sim_thread / viewer) and by RT receive threads that piggy-back on a
  // launch-level driver process taskset (Transceiver default kRtUdpRecvConfig,
  // udp_hand_driver kHandUdpRecvConfig). Apply the upper-bound check only
  // when cpu_core is non-sentinel.
  const auto& slots = GetCpuTopology().physical_core_slots;
  const int max_slots = slots.empty() ? GetOnlineCpuCount() : static_cast<int>(slots.size());
  if (cfg.cpu_core < -1 || cfg.cpu_core >= max_slots) {
    errors += "Invalid CPU core slot " + std::to_string(cfg.cpu_core) + " (valid range: -1, 0-" +
              std::to_string(max_slots - 1) + "); ";
  }

  // Validate scheduler policy
  if (cfg.sched_policy != SCHED_FIFO && cfg.sched_policy != SCHED_RR &&
      cfg.sched_policy != SCHED_OTHER) {
    errors += "Invalid scheduler policy " + std::to_string(cfg.sched_policy) + "; ";
  }

  // Validate priorities for RT scheduling
  if ((cfg.sched_policy == SCHED_FIFO || cfg.sched_policy == SCHED_RR) &&
      (cfg.sched_priority < 1 || cfg.sched_priority > 99)) {
    errors += "RT priority must be 1-99 for SCHED_FIFO/RR; ";
  }

  // Validate nice value for SCHED_OTHER
  if (cfg.sched_policy == SCHED_OTHER && (cfg.nice_value < -20 || cfg.nice_value > 19)) {
    errors += "Nice value must be -20 to 19 for SCHED_OTHER; ";
  }

  // Validate thread name
  if (!cfg.name || std::strlen(cfg.name) > 15) {
    errors += "Thread name must be non-null and <= 15 characters; ";
  }

  return errors;
}

// Translate a ThreadConfig::cpu_core *slot index* into the kernel logical CPU
// id passed to CPU_SET. Slot semantics:
//   slot == -1            → -1 (sentinel: caller-controlled pin, no CPU_SET)
//   slot in [0, slots-1)  → physical_core_slots[slot] (P-physical → E → LP-E
//                           on hybrid; identity / "even logicals" elsewhere)
//   slot >= slots OR
//   physical_core_slots empty (container without sysfs) → fall back to
//                           identity (slot == logical), preserving legacy
//                           behaviour for environments where topology
//                           detection cannot run.
//
// This is the only place ThreadConfig::cpu_core is interpreted as logical
// id, so RT threads never accidentally land on a P-core's SMT sibling.
//
// Two overloads: the no-arg form reads the process-wide cached topology
// (production hot path). The CpuTopology& form lets unit tests inject a
// mocked topology without touching the cached singleton.
[[nodiscard]] inline int SlotToLogicalCpu(int slot, const CpuTopology& topology) noexcept {
  if (slot < 0)
    return slot;
  if (topology.physical_core_slots.empty() ||
      slot >= static_cast<int>(topology.physical_core_slots.size()))
    return slot;
  return topology.physical_core_slots[static_cast<size_t>(slot)];
}

[[nodiscard]] inline int SlotToLogicalCpu(int slot) noexcept {
  return SlotToLogicalCpu(slot, GetCpuTopology());
}

// Apply thread configuration (CPU affinity, scheduler policy, priority)
// Returns true on success, false on failure (e.g., insufficient permissions)
//
// Requirements:
// - CAP_SYS_NICE capability or membership in 'realtime' group
// - /etc/security/limits.conf: @realtime - rtprio 99
[[nodiscard]] inline bool ApplyThreadConfig(const ThreadConfig& cfg) noexcept {
  // Validate configuration first
  std::string validation_errors = ValidateThreadConfig(cfg);
  if (!validation_errors.empty()) {
    std::fprintf(stderr, "[ApplyThreadConfig] '%s' validation failed: %s\n",
                 cfg.name ? cfg.name : "<null>", validation_errors.c_str());
    return false;
  }

  // 1. Set CPU affinity (skip when cpu_core == -1: the calling process's
  //    taskset already constrains this thread's affinity — typical for RT
  //    receive threads that piggy-back on a launch-level driver taskset).
  //    cpu_core is a *slot index*; SlotToLogicalCpu translates to the
  //    actual logical CPU id (P-core physical, not SMT sibling, on hybrid).
  if (cfg.cpu_core >= 0) {
    const int logical_cpu = SlotToLogicalCpu(cfg.cpu_core);
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(static_cast<std::size_t>(logical_cpu), &cpuset);

    // pthread_setaffinity_np returns the error code directly and does NOT
    // set errno (POSIX pthread API contract). Capture rc explicitly — reading
    // errno here yields stale state from earlier libc calls and routinely
    // prints "(success)" while the call actually failed.
    if (const int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset); rc != 0) {
      std::fprintf(
          stderr,
          "[ApplyThreadConfig] '%s' setaffinity failed: slot=%d -> logical_cpu=%d rc=%d (%s)\n",
          cfg.name, cfg.cpu_core, logical_cpu, rc, SafeStrerror(rc).c_str());
      return false;
    }
  }

  // 2. Set scheduler policy and priority
  sched_param param{};

  if (cfg.sched_policy == SCHED_FIFO || cfg.sched_policy == SCHED_RR) {
    param.sched_priority = cfg.sched_priority;

    // pthread_setschedparam also returns rc directly without touching errno.
    if (const int rc = pthread_setschedparam(pthread_self(), cfg.sched_policy, &param); rc != 0) {
      std::fprintf(stderr,
                   "[ApplyThreadConfig] '%s' setschedparam failed: policy=%d prio=%d rc=%d "
                   "(%s) — check ulimit -r and CAP_SYS_NICE\n",
                   cfg.name, cfg.sched_policy, cfg.sched_priority, rc, SafeStrerror(rc).c_str());
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
// Attempts to apply as much configuration as possible, even if RT scheduling
// fails Returns {full_success, warnings} - full_success is true only if
// everything succeeded
[[nodiscard]] inline std::pair<bool, std::string> ApplyThreadConfigWithFallback(
    const ThreadConfig& cfg) noexcept {
  std::string validation_errors = ValidateThreadConfig(cfg);
  if (!validation_errors.empty()) {
    return {false, "Validation failed: " + validation_errors};
  }

  bool full_success = true;
  std::string warnings;

  // 1. Try CPU affinity (skip when cpu_core == -1: launch-level taskset is
  //    the source of truth for this thread's affinity, see Phase 5 sentinel
  //    in kRtUdpRecvConfig / kHandUdpRecvConfig). cpu_core is a *slot index*;
  //    SlotToLogicalCpu translates to the actual logical CPU id (P-core
  //    physical, not SMT sibling, on hybrid).
  if (cfg.cpu_core >= 0) {
    const int logical_cpu = SlotToLogicalCpu(cfg.cpu_core);
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(static_cast<std::size_t>(logical_cpu), &cpuset);

    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
      full_success = false;
      warnings += "CPU affinity failed: " + SafeStrerror(errno) + "; ";
    }
  }

  // 2. Try RT scheduling, fallback to SCHED_OTHER if it fails
  sched_param param{};
  bool rt_success = false;

  if (cfg.sched_policy == SCHED_FIFO || cfg.sched_policy == SCHED_RR) {
    param.sched_priority = cfg.sched_priority;
    if (pthread_setschedparam(pthread_self(), cfg.sched_policy, &param) == 0) {
      rt_success = true;
    } else {
      full_success = false;
      warnings +=
          "RT scheduling failed, falling back to SCHED_OTHER: " + SafeStrerror(errno) + "; ";
    }
  }

  if (!rt_success) {
    // Apply SCHED_OTHER configuration
    if (cfg.sched_policy == SCHED_OTHER) {
      param.sched_priority = 0;
      if (setpriority(PRIO_PROCESS, 0, cfg.nice_value) != 0) {
        warnings += "Nice value setting failed: " + SafeStrerror(errno) + "; ";
      }
    } else {
      // Fallback: set nice value for RT policies that failed
      param.sched_priority = 0;
      if (setpriority(PRIO_PROCESS, 0, cfg.nice_value) != 0) {
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
  if (pthread_setname_np(name_buf) != 0) {
    warnings += "Thread name setting failed: " + SafeStrerror(errno) + "; ";
  }
#else
  if (pthread_setname_np(pthread_self(), name_buf) != 0) {
    warnings += "Thread name setting failed: " + SafeStrerror(errno) + "; ";
  }
#endif

  return {full_success, warnings};
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
    for (std::size_t i = 0; i < static_cast<std::size_t>(CPU_SETSIZE); ++i) {
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

// Apply thread configuration and emit the standard success/failure log lines.
//
// Single source of truth for the "[INFO] Thread '<name>' configured:" /
// "[WARN] Thread config failed for '<name>'" format historically emitted only
// by the RT controller's executor threads. Wrap every ApplyThreadConfig call at
// a thread's entry point with this so each runtime thread (rt_control, the
// executor dispatchers, nrt_publish, UDP receive, MuJoCo sim/viewer, MPC
// main/workers, hand detector) announces its resolved CPU affinity / scheduler
// / priority / nice / name on startup.
//
// Runs once at thread entry (one-shot init, before any periodic loop), so the
// fprintf + VerifyThreadConfig() string build stay off the RT hot path. On
// failure ApplyThreadConfig itself has already printed the specific
// "[ApplyThreadConfig] '<name>' setschedparam/setaffinity failed …" line to
// stderr; this adds the higher-level one-line summary. Returns ApplyThreadConfig's
// result; callers that only want the side-effect may discard it.
inline bool ApplyThreadConfigVerbose(const ThreadConfig& cfg) noexcept {
  const char* name = cfg.name ? cfg.name : "<null>";
  if (!ApplyThreadConfig(cfg)) {
    std::fprintf(stderr, "[WARN] Thread config failed for '%s' (need realtime permissions)\n",
                 name);
    return false;
  }
  std::fprintf(stdout, "[INFO] Thread '%s' configured:\n%s", name, VerifyThreadConfig().c_str());
  return true;
}

// Get thread statistics (for jitter measurement).
// Returns {min_latency_us, max_latency_us, avg_latency_us}.
//
// WARNING: NOT RT-safe — accepts std::vector (heap-allocated).
// Call only from non-RT threads (e.g. logging, monitoring).
inline std::tuple<double, double, double> GetThreadStats(
    const std::vector<double>& latencies_us) noexcept {
  if (latencies_us.empty()) {
    return {0.0, 0.0, 0.0};
  }

  double min_val = *std::min_element(latencies_us.begin(), latencies_us.end());
  double max_val = *std::max_element(latencies_us.begin(), latencies_us.end());
  double sum = std::accumulate(latencies_us.begin(), latencies_us.end(), 0.0);
  double avg = sum / static_cast<double>(latencies_us.size());

  return {min_val, max_val, avg};
}

// Enhanced thread metrics for comprehensive monitoring
struct ThreadMetrics {
  double min_latency_us;
  double max_latency_us;
  double avg_latency_us;
  double jitter_us;         // Standard deviation of latencies
  double percentile_95_us;  // 95th percentile latency
  double percentile_99_us;  // 99th percentile latency
};

// Get comprehensive thread statistics with percentiles.
// Returns ThreadMetrics with latency analysis.
//
// WARNING: NOT RT-safe — copies and sorts the input vector (heap allocation).
// Call only from non-RT threads (e.g. logging, monitoring).
inline ThreadMetrics GetThreadMetrics(const std::vector<double>& latencies_us) noexcept {
  ThreadMetrics metrics{};

  if (latencies_us.empty()) {
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
  for (double lat : latencies_us) {
    sum += lat;
    sum_sq += lat * lat;
  }
  metrics.avg_latency_us = sum / static_cast<double>(latencies_us.size());

  // Jitter (standard deviation)
  double variance = (sum_sq / static_cast<double>(latencies_us.size())) -
                    (metrics.avg_latency_us * metrics.avg_latency_us);
  metrics.jitter_us = std::sqrt(std::max(0.0, variance));

  // Percentiles
  size_t n = sorted_latencies.size();
  size_t idx_95 = static_cast<size_t>(0.95 * static_cast<double>(n - 1));
  size_t idx_99 = static_cast<size_t>(0.99 * static_cast<double>(n - 1));
  metrics.percentile_95_us = sorted_latencies[idx_95];
  metrics.percentile_99_us = sorted_latencies[idx_99];

  return metrics;
}

// RT-safe thread health check using bitfield flags instead of strings.
// Zero heap allocation — safe to call periodically from the RT path.
//
// Usage:
//   auto flags = CheckThreadHealthFast(&kRtControlConfig);
//   if (flags != ThreadHealthFlag::kOk) { /* log or handle */ }
enum class ThreadHealthFlag : uint8_t {
  kOk = 0,
  kWrongCore = 1 << 0,
  kPolicyChanged = 1 << 1,
  kPriorityChanged = 1 << 2,
  kNiceChanged = 1 << 3,
};

inline ThreadHealthFlag operator|(ThreadHealthFlag a, ThreadHealthFlag b) noexcept {
  return static_cast<ThreadHealthFlag>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

inline ThreadHealthFlag operator&(ThreadHealthFlag a, ThreadHealthFlag b) noexcept {
  return static_cast<ThreadHealthFlag>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}

inline ThreadHealthFlag& operator|=(ThreadHealthFlag& a, ThreadHealthFlag b) noexcept {
  a = a | b;
  return a;
}

inline ThreadHealthFlag CheckThreadHealthFast(const ThreadConfig& expected) noexcept {
  ThreadHealthFlag flags = ThreadHealthFlag::kOk;

  // Check CPU affinity — translate slot index through SlotToLogicalCpu so
  // we compare against the actual logical CPU that ApplyThreadConfig pinned.
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  if (pthread_getaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) == 0) {
    const int expected_logical = SlotToLogicalCpu(expected.cpu_core);
    if (expected_logical >= 0 && !CPU_ISSET(static_cast<std::size_t>(expected_logical), &cpuset)) {
      flags |= ThreadHealthFlag::kWrongCore;
    }
  }

  // Check scheduler policy and priority
  int policy;
  sched_param param;
  if (pthread_getschedparam(pthread_self(), &policy, &param) == 0) {
    if (policy != expected.sched_policy) {
      flags |= ThreadHealthFlag::kPolicyChanged;
    }
    if ((policy == SCHED_FIFO || policy == SCHED_RR) &&
        param.sched_priority != expected.sched_priority) {
      flags |= ThreadHealthFlag::kPriorityChanged;
    }
  }

  // Check nice value for SCHED_OTHER
  if (expected.sched_policy == SCHED_OTHER) {
    errno = 0;
    int nice_val = getpriority(PRIO_PROCESS, 0);
    if (errno == 0 && nice_val != expected.nice_value) {
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
inline std::string CheckThreadHealth(const ThreadConfig* expected_config = nullptr) noexcept {
  std::string issues;

  // Check if still on expected CPU core — compare against the slot-translated
  // logical CPU id, not the raw cpu_core (which is a slot index, not a
  // kernel logical id).
  if (expected_config) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    if (pthread_getaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) == 0) {
      const int expected_logical = SlotToLogicalCpu(expected_config->cpu_core);
      bool on_expected_core = false;
      for (std::size_t i = 0; i < static_cast<std::size_t>(CPU_SETSIZE); ++i) {
        if (CPU_ISSET(i, &cpuset) && static_cast<int>(i) == expected_logical) {
          on_expected_core = true;
          break;
        }
      }
      if (!on_expected_core) {
        issues += "Thread not on expected CPU core slot " +
                  std::to_string(expected_config->cpu_core) + " (logical " +
                  std::to_string(expected_logical) + "); ";
      }
    }
  }

  // Check if RT priority is maintained
  int policy;
  sched_param param;
  if (pthread_getschedparam(pthread_self(), &policy, &param) == 0) {
    if (expected_config) {
      if (policy != expected_config->sched_policy) {
        issues += "Scheduler policy changed from expected " +
                  std::to_string(expected_config->sched_policy) + " to " + std::to_string(policy) +
                  "; ";
      }
      if ((policy == SCHED_FIFO || policy == SCHED_RR) &&
          param.sched_priority != expected_config->sched_priority) {
        issues += "RT priority changed from expected " +
                  std::to_string(expected_config->sched_priority) + " to " +
                  std::to_string(param.sched_priority) + "; ";
      }
    } else {
      // General health check without expected config
      if (policy == SCHED_OTHER) {
        issues += "Thread using SCHED_OTHER instead of RT policy; ";
      }
    }
  }

  // Check nice value for SCHED_OTHER
  if (expected_config && expected_config->sched_policy == SCHED_OTHER) {
    errno = 0;
    int nice_val = getpriority(PRIO_PROCESS, 0);
    if (errno == 0 && nice_val != expected_config->nice_value) {
      issues += "Nice value " + std::to_string(nice_val) + " differs from expected " +
                std::to_string(expected_config->nice_value) + "; ";
    }
  }

  return issues;
}

// Returns the number of online logical CPUs on the current system.
// Includes SMT/Hyper-Threading siblings (e.g., 6C/12T returns 12).
// Use for CPU affinity validation (logical CPU IDs are valid up to this count).
inline int GetOnlineCpuCount() noexcept {
  const int n = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
  return (n > 0) ? n : 1;
}

// Returns the number of physical CPU cores (excluding SMT/HT siblings).
// Parses sysfs topology to count unique (socket_id, core_id) pairs.
// Falls back to cgroup CPU quota or GetOnlineCpuCount() if sysfs topology
// is unavailable (containers, VMs, restricted environments).
//
// This is the correct metric for selecting thread layouts (4/6/8-core)
// because thread_config.hpp assigns threads to physical cores.
// Example: i7-8700 (6C/12T) → returns 6, not 12.
//
// Container/VM detection order:
//   1. sysfs topology (bare-metal / full VM with topology exposed)
//   2. cgroup v2 cpu.max (Docker, Kubernetes with CPU limits)
//   3. cgroup v1 cpu.cfs_quota_us / cpu.cfs_period_us
//   4. sysconf(_SC_NPROCESSORS_ONLN) final fallback
inline int GetPhysicalCpuCount() noexcept {
  // ── Stage 1: sysfs topology (preferred) ──────────────────────────────
  std::set<std::pair<int, int>> unique_cores;

  for (int cpu = 0; cpu < 1024; ++cpu) {
    char pkg_path[128];
    char core_path[128];

    std::snprintf(pkg_path, sizeof(pkg_path),
                  "/sys/devices/system/cpu/cpu%d/topology/physical_package_id", cpu);
    std::snprintf(core_path, sizeof(core_path), "/sys/devices/system/cpu/cpu%d/topology/core_id",
                  cpu);

    FILE* pkg_file = std::fopen(pkg_path, "r");
    if (!pkg_file) {
      break;  // No more CPUs
    }
    int socket_id = 0;
    if (std::fscanf(pkg_file, "%d", &socket_id) != 1) {
      std::fclose(pkg_file);
      break;
    }
    std::fclose(pkg_file);

    FILE* core_file = std::fopen(core_path, "r");
    if (!core_file) {
      break;
    }
    int core_id = 0;
    if (std::fscanf(core_file, "%d", &core_id) != 1) {
      std::fclose(core_file);
      break;
    }
    std::fclose(core_file);

    unique_cores.insert({socket_id, core_id});
  }

  if (!unique_cores.empty()) {
    return static_cast<int>(unique_cores.size());
  }

  // ── Stage 2: cgroup v2 cpu.max ───────────────────────────────────────
  {
    FILE* f = std::fopen("/sys/fs/cgroup/cpu.max", "r");
    if (f) {
      char max_str[32]{};
      int period = 0;
      if (std::fscanf(f, "%31s %d", max_str, &period) == 2 && std::string_view(max_str) != "max" &&
          period > 0) {
        const int quota = std::atoi(max_str);
        if (quota > 0) {
          std::fclose(f);
          return std::max(1, quota / period);
        }
      }
      std::fclose(f);
    }
  }

  // ── Stage 3: cgroup v1 cpu.cfs_quota_us / period ─────────────────────
  {
    int quota = -1;
    int period = 0;
    FILE* fq = std::fopen("/sys/fs/cgroup/cpu/cpu.cfs_quota_us", "r");
    if (fq) {
      if (std::fscanf(fq, "%d", &quota) != 1) {
        quota = -1;
      }
      std::fclose(fq);
    }
    FILE* fp = std::fopen("/sys/fs/cgroup/cpu/cpu.cfs_period_us", "r");
    if (fp) {
      if (std::fscanf(fp, "%d", &period) != 1) {
        period = 0;
      }
      std::fclose(fp);
    }
    if (quota > 0 && period > 0) {
      return std::max(1, quota / period);
    }
  }

  // ── Stage 4: final fallback ─���────────────────────────────────────────
  return GetOnlineCpuCount();
}

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
inline std::string ValidateSystemThreadConfigs(const SystemThreadConfigs& configs) noexcept {
  std::string errors;

  // Validate each individual config
  errors += ValidateThreadConfig(configs.rt_control);
  errors += ValidateThreadConfig(configs.rt_callback);
  errors += ValidateThreadConfig(configs.nrt_logging);
  errors += ValidateThreadConfig(configs.nrt_callback);
  // Process-level configs (arm_driver / hand_driver / sim_thread / viewer)
  // are applied as taskset pins by the launch script (not via
  // ApplyThreadConfig), and sim_thread / viewer may carry cpu_core = -1 to
  // signal "no pinning, let MuJoCo roam across the released cpu_shield". Skip
  // ValidateThreadConfig for them — only the core-disjointness rules below
  // apply.
  // MPC: validate main + active workers. workers beyond num_workers are
  // zero-initialised and ignored.
  errors += ValidateThreadConfig(configs.mpc.main);
  if (configs.mpc.num_workers < 0 || configs.mpc.num_workers > kMpcMaxWorkers) {
    errors += "mpc.num_workers out of range [0, " + std::to_string(kMpcMaxWorkers) + "]; ";
  }
  for (int i = 0; i < configs.mpc.num_workers && i < kMpcMaxWorkers; ++i) {
    const ThreadConfig& w = configs.mpc.workers[static_cast<std::size_t>(i)];
    errors += ValidateThreadConfig(w);
    // Worker priority must not exceed main — prevents worker preempting
    // the solve loop it's supposed to assist.
    if ((w.sched_policy == SCHED_FIFO || w.sched_policy == SCHED_RR) &&
        (configs.mpc.main.sched_policy == SCHED_FIFO ||
         configs.mpc.main.sched_policy == SCHED_RR) &&
        w.sched_priority > configs.mpc.main.sched_priority) {
      errors += "mpc.worker[" + std::to_string(i) + "] priority exceeds mpc.main; ";
    }
  }
  // MPC main must not exceed rt_callback priority — rt_callback callbacks are hard
  // real-time and must always preempt long MPC solves.
  if ((configs.mpc.main.sched_policy == SCHED_FIFO || configs.mpc.main.sched_policy == SCHED_RR) &&
      (configs.rt_callback.sched_policy == SCHED_FIFO ||
       configs.rt_callback.sched_policy == SCHED_RR) &&
      configs.mpc.main.sched_priority >= configs.rt_callback.sched_priority) {
    errors += "mpc.main priority (" + std::to_string(configs.mpc.main.sched_priority) +
              ") must be below rt_callback priority (" +
              std::to_string(configs.rt_callback.sched_priority) + "); ";
  }

  // Collect all configs with names for conflict analysis. MPC main + up
  // to kMpcMaxWorkers workers are always included; inactive worker slots
  // have cpu_core == 0 but also sched_policy == 0 (SCHED_OTHER priority 0),
  // which cannot trigger an RT/RT same-priority conflict.
  struct NamedConfig {
    const char* name;
    const ThreadConfig* config;
  };

  // Layout v4: 4 fixed thread roles + 4 process-level pins (arm/hand,
  // sim/viewer) + mpc main + up to kMpcMaxWorkers. Process-level configs are
  // included so their cpu_core participates in the disjointness sweep below,
  // but their SCHED_OTHER policy means they cannot trigger an RT/RT
  // same-priority conflict (the only error condition).
  const std::array<NamedConfig, 8 + 1 + kMpcMaxWorkers> all_configs = {{
      {"rt_control", &configs.rt_control},
      {"rt_callback", &configs.rt_callback},
      {"nrt_logging", &configs.nrt_logging},
      {"nrt_callback", &configs.nrt_callback},
      {"arm_driver", &configs.arm_driver},
      {"hand_driver", &configs.hand_driver},
      {"sim_thread", &configs.sim_thread},
      {"viewer", &configs.viewer},
      {"mpc_main", &configs.mpc.main},
      {"mpc_worker_0", &configs.mpc.workers[0]},
      {"mpc_worker_1", &configs.mpc.workers[1]},
  }};

  auto is_rt = [](const ThreadConfig* c) {
    return c->sched_policy == SCHED_FIFO || c->sched_policy == SCHED_RR;
  };

  // Check for problematic core sharing: only flag RT+RT same-priority conflicts.
  // cpu_core == -1 is a sentinel meaning "no pinning" — those entries are
  // excluded from same-core matching since they don't claim any specific core.
  for (std::size_t i = 0; i < all_configs.size(); ++i) {
    for (std::size_t j = i + 1; j < all_configs.size(); ++j) {
      const auto& a = all_configs[i];
      const auto& b = all_configs[j];

      if (a.config->cpu_core < 0 || b.config->cpu_core < 0) {
        continue;  // unpinned thread — disjointness rule N/A
      }
      if (a.config->cpu_core != b.config->cpu_core) {
        continue;  // different cores — no conflict possible
      }

      // Same core: only a problem if both are RT with identical priority
      if (is_rt(a.config) && is_rt(b.config) &&
          a.config->sched_priority == b.config->sched_priority) {
        errors += "RT priority " + std::to_string(a.config->sched_priority) + " conflict on core " +
                  std::to_string(a.config->cpu_core) + " between '" + a.name + "' and '" + b.name +
                  "'; ";
      }
    }
  }

  // Disjointness: arm_driver / hand_driver must not collide with any RT
  // controller thread (rt_control / rt_callback / mpc_*).
  // Same-core sharing between arm and hand is tolerated (6-core degraded mode
  // intentionally puts both on Core 1). sim_thread / viewer are excluded
  // because in sim mode the launch script releases the cpu_shield and lets
  // MuJoCo roam freely over the freed cores.
  auto is_rt_controller = [](const char* name) noexcept {
    // Lightweight: RT controller roles share the same compile-time names.
    return std::string(name) == "rt_control" || std::string(name) == "rt_callback" ||
           std::string(name) == "mpc_main" || std::string(name) == "mpc_worker_0" ||
           std::string(name) == "mpc_worker_1";
  };
  for (const auto& driver_name : {std::string("arm_driver"), std::string("hand_driver")}) {
    const ThreadConfig* driver = nullptr;
    for (const auto& nc : all_configs) {
      if (driver_name == nc.name) {
        driver = nc.config;
        break;
      }
    }
    if (driver == nullptr || driver->cpu_core < 0) {
      continue;
    }
    for (const auto& nc : all_configs) {
      if (driver_name == nc.name) {
        continue;
      }
      if (!is_rt_controller(nc.name)) {
        continue;
      }
      if (nc.config->cpu_core == driver->cpu_core) {
        errors += driver_name + " core " + std::to_string(driver->cpu_core) +
                  " collides with RT controller '" + nc.name + "'; ";
      }
    }
  }

  return errors;
}

// Selects the appropriate ThreadConfig set for this machine. Uses
// GetPhysicalCpuCount() (not GetOnlineCpuCount()) to avoid SMT/HT over-counting:
// an i7-8700 (6C/12T) correctly selects the 6-core layout, not the 12-core one.
//
// The tier table and the breakpoints themselves live in the generated
// thread_config_generated.hpp (source of truth:
// repo_scripts/config/thread_layout.yaml). This wrapper only supplies the
// runtime core count -- call SelectThreadConfigsForCoreCount() directly to
// evaluate a tier this machine does not have (that is what the tests do).
inline SystemThreadConfigs SelectThreadConfigs() noexcept {
  return SelectThreadConfigsForCoreCount(GetPhysicalCpuCount());
}

}  // namespace rtc

#endif  // RTC_BASE_THREAD_UTILS_HPP_
