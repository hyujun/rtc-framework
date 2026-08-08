#ifndef RTC_BASE_THREAD_CONFIG_HPP_
#define RTC_BASE_THREAD_CONFIG_HPP_

#include <sched.h>  // SCHED_FIFO, SCHED_OTHER, SCHED_RR

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
// Holds the MPC solve thread. Worker slots lived here until #380: their
// threads applied a config and exited immediately, and no solver could have
// used them anyway — Aligator parallelises through OpenMP, which spawns and
// owns its own threads and accepts no external handles. Reintroducing
// parallel MPC means pinning that pool, not adding thread configs here.
//
// Invariant (checked by ValidateSystemThreadConfigs):
//   * `main.sched_priority` < rt_callback thread priority (rt_callback preempts MPC).
//
// Why this stays a one-field struct (#382 — collapse evaluated, kept):
// It carries nothing a bare `ThreadConfig mpc_main` would not, and the entire
// cost of the special case is on the reading side. The collapse was costed so
// the next reader need not re-derive it: 6 non-test `.mpc.main` consumers in 2
// files (5 in thread_utils.hpp — ValidateSystemThreadConfigs and the
// name→config table — plus the launch-config copy in integrated_bringup's wbc
// controller), 8 `kind: mpc` branches in gen_thread_layout.py, the manifest's
// `cpp_suffix_mpc` override, and test_mpc_thread_config.cpp, a suite for this
// type alone. Issue #382 is the decision record: reopen it rather than
// collapsing the aggregate ad hoc during unrelated work.
//
// One argument does NOT support keeping it — "parallel MPC will re-add slots
// here". Per the paragraph above that path pins an OpenMP pool and adds no
// ThreadConfig, so it would not restore this aggregate.

struct MpcThreadConfig {
  ThreadConfig main{};
};

// ── RT priority hierarchy (layout v5) ───────────────────────────────────────
//   90 (rt_control)  > 70 (rt_callback) > 60 (mpc_main)
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
// rt_callback + DDS recv co-pin = Core 2, mpc_main = Core 3.
//
// v5 (issue #349) makes Core 2 the *aux slot*: the three CFS lanes
// (nrt_logging, nrt_callback, nrt_publish) fold onto it on every tier >= 6
// instead of owning slots of their own. On-robot NUC13 telemetry put Core 2 at
// ~1.5% duty and the three nrt lanes together at ~5.6% (/proc/<tid>/stat
// differencing — the CSV timing scopes read ~6x low because they cover only the
// callback body, not rmw_wait/take/deserialize), so the fold lands Core 2 near
// ~7% and hands the vacated slots back to the system cpuset (the cset shield
// narrowed from "2-9,12-13" to "2-9" on the 12-core target). SCHED_FIFO
// preempts SCHED_OTHER unconditionally, so rt_callback keeps its latency
// contract; the residual risk is cache pollution and DDS-side (CFS) delay,
// watched on-robot through the drop counters rather than through t_total.
// The 4-core fallback is deliberately NOT folded — its nrt lanes already share
// the OS slot, so moving them would return no core while loading the most
// constrained tier's RT slot.
// The rt_outbound jthread + publish_buffer_ SPSC + eventfd wakeup are removed;
// rt_control performs DeviceBackend.WriteCommand inline in the rt_loop tick
// (RT-safe contract on backends). DDS receive thread is co-pinned to the same
// core as rt_callback (Core 2 on every tier except 4-core fallback) via
// launch-time taskset for cache locality. Hand-private UDP receive thread
// (priority 65) lives inside the hand_driver process and inherits that
// process's affinity, so it is NOT represented in SystemThreadConfigs. Since
// issue #345 that affinity comes from the process pinning its own main thread
// to hand_driver.cpu_core before it spins — not from a launch-level taskset
// sweep. The sweep was removed because `-a` also dragged the same process's
// blocking-file-I/O lane (hand_aux_io, a package-local ThreadConfig placed on
// the OS slot) back onto the hand core, undoing the split it needs.
//
// process-level threads (arm_driver, hand_driver, sim_thread, viewer) are
// SCHED_OTHER prio 0; only their cpu_core is consumed, all other fields are
// passed through ApplyThreadConfig as no-op. The launch files apply that pin
// for every entry except hand_driver, whose process reads this cpu_core itself
// in main() (issue #345); launch then only co-pins its DDS threads, which
// rclcpp::init() creates before the node exists.
//
// These entries model the *process*, not whatever RT thread runs inside it, and
// must stay SCHED_OTHER for that reason. The UR arm driver is the sharp case:
// its controller_manager control loop is SCHED_FIFO 50, but that thread belongs
// to upstream and is configured by CM parameters the launch file generates, not
// by ApplyThreadConfig (issue #343). Raising kArmDriverConfig* to FIFO would
// claim the process's main/executor thread is RT, which it is not.
//
// ── Tier blocks ─────────────────────────────────────────────────────────────
// The per-tier constants (kRtControlConfig8Core, kMpcConfig12Core, ...) and the
// SelectThreadConfigsForCoreCount() dispatch are GENERATED from
// repo_scripts/config/thread_layout.yaml into thread_config_generated.hpp,
// included at the bottom of this header. Do not add tier constants here by
// hand: edit the manifest and run
//   python3 repo_scripts/scripts/gen_thread_layout.py --write
// The manifest is the single source of truth shared with the shell helpers
// (repo_scripts/scripts/lib/thread_layout_generated.sh) and the launch mirror
// (rtc_tools/rtc_tools/launch/thread_layout_generated.py); `--check` is the
// drift gate (issue #153 M1).
//
// Emission order: tiers ascending by core count (4 → 6 → 8 → 10 → 12 → 14 →
// 16); within a tier, entries follow the manifest's role declaration order,
// which is sched_priority descending with an alphabetical tie-break on the
// symbol name (Arm < Hand < NrtCallback < NrtLogging < SimThread < Viewer for
// the priority-0 group). MpcThreadConfig occupies its main.sched_priority slot.

// Aggregated thread configs selected at runtime for all threads.
//
// Field naming (layout v5):
//   * rt_callback  = single RT callback dispatcher thread (Core 2 FIFO 70 on
//                    every tier; Core 0 = OS/DDS/IRQ only, RT cluster starts
//                    at Core 1). Hosts the executor that dispatches state
//                    subscriptions bound to the rt_callback callback group.
//                    Replaces the former rt_inbound (FIFO 70) + rt_outbound
//                    (FIFO 65) pair: actuator command publish is performed
//                    inline in the rt_loop tick on rt_control (Core 1 FIFO
//                    90), so no separate output thread is required. Since v5
//                    this slot is also the aux slot shared by the three CFS
//                    lanes below (tiers >= 6; the 4-core fallback keeps them
//                    on the OS slot).
//   * nrt_callback = non-RT callback dispatcher for services / lifecycle /
//                    non-RT-boundary subs
//   * nrt_publish  = the controller-owned publish jthread (PublishNonRtSnapshot
//                    drain). Shares nrt_callback's slot and policy by design —
//                    it is a distinct ENTRY, not a distinct core. It exists
//                    because the jthread used to apply nrt_callback's config
//                    verbatim, so two threads reported the same
//                    pthread_setname_np name and verify_rt_runtime.sh, which
//                    stores one TID per name, checked whichever it saw last
//                    and never noticed the other (issue #349 D15)
//   * nrt_logging  = non-RT CSV drain
//
// The three nrt_* lanes are separate ENTRIES that happen to resolve to the same
// slot under v5. They keep distinct nice values (nrt_logging -5, the other two
// 0) and distinct thread names, which is what lets the runtime verifier demand
// three TIDs on the aux slot rather than one.
//
// Process-level pins (applied at launch time, no ApplyThreadConfig call;
// SCHED_OTHER prio 0):
//   * arm_driver / hand_driver = external driver processes. hand_driver is a
//     taskset -a sweep (its RT thread inherits the process mask, issue #245).
//     arm_driver is NOT: ros2_control_node's main thread is the executor and
//     its 500 Hz loop is a separate thread that taskset cannot reach, so the
//     cpu_core here is handed to controller_manager's own cpu_affinity
//     parameter, which the loop applies to itself (issue #343). The entry still
//     models the process — the loop's FIFO 50 is upstream's, not ours.
//   * sim_thread / viewer      = MuJoCo physics + GLFW rendering threads in
//                                sim mode. cpu_core may be -1 (no pinning;
//                                launch script releases the cpu_shield for
//                                MuJoCo).
//
// DDS receive thread (CycloneDDS / Fast-RTPS) is co-pinned to the rt_callback
// core (Core 2, the v5 aux slot) via launch-time taskset for cache locality. Its CFS policy is
// preserved; the launch script pins only non-RT threads of the controller
// process. The hand UDP receive thread (RT priority 65) lives privately
// inside udp_hand_controller and inherits affinity from the launch-level
// taskset on the hand_driver core. Generic UDP receivers using
// rtc_communication::Transceiver pick up the kRtUdpRecvConfig default
// (cpu_core = -1, caller pins explicitly).
struct SystemThreadConfigs {
  ThreadConfig rt_control;
  ThreadConfig rt_callback;
  ThreadConfig nrt_logging;
  ThreadConfig nrt_callback;
  ThreadConfig nrt_publish;  // controller-owned publish jthread; shares nrt_callback's slot
  ThreadConfig arm_driver;   // external arm driver process pin (SCHED_OTHER)
  ThreadConfig hand_driver;  // external hand driver process pin (SCHED_OTHER)
  ThreadConfig sim_thread;   // MuJoCo physics thread (SCHED_OTHER, cpu_core may be -1)
  ThreadConfig viewer;       // GLFW viewer thread     (SCHED_OTHER, cpu_core may be -1)
  MpcThreadConfig mpc;       // MPC solve thread
};

}  // namespace rtc

// Generated tier constants + SelectThreadConfigsForCoreCount(). Included last:
// it needs ThreadConfig / MpcThreadConfig / SystemThreadConfigs above.
#include "rtc_base/threading/thread_config_generated.hpp"

#endif  // RTC_BASE_THREAD_CONFIG_HPP_
