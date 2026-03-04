#ifndef UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_
#define UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_

// ── Includes: project, then MuJoCo, then C++ stdlib ───────────────────────────
#include <mujoco/mujoco.h>

#ifdef MUJOCO_HAVE_GLFW
#include <GLFW/glfw3.h>
#endif

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace ur5e_rt_controller {

// ── MuJoCoSimulator ────────────────────────────────────────────────────────────
//
// Thread-safe wrapper around a MuJoCo physics model.
//
// Simulation modes (planning_sim.md §8.2):
//   kFreeRun  — advances mj_step() as fast as possible with no timing
//               constraints.  Best for algorithm validation, trajectory gen.
//   kSyncStep — publishes state, waits for one controller command, then takes
//               one physics step.  Step latency ≈ controller Compute() time,
//               enabling direct timing measurement.
//
// Threading model:
//   SimLoop thread  — runs SimLoopFreeRun or SimLoopSyncStep.
//   ViewerLoop thread — renders scene at ~60 Hz via GLFW (optional).
//   Caller thread   — calls SetCommand(), GetPositions(), GetVelocities().
//
// Synchronisation:
//   cmd_mutex_   — protects pending_cmd_ (caller → sim write)
//   cmd_pending_ — atomic flag for lock-free fast-path in FreeRun mode
//   sync_cv_     — condition variable to wake SimLoopSyncStep on command
//   state_mutex_ — protects latest_positions_ / latest_velocities_
//   viz_mutex_   — protects viz_qpos_ (try_lock avoids blocking SimLoop)
//
class MuJoCoSimulator {
 public:
  // Simulation execution mode.
  enum class SimMode {
    kFreeRun,   // Maximum speed — no timing constraints
    kSyncStep,  // 1:1 synchronised with controller commands
  };

  struct Config {
    std::string model_path;
    SimMode     mode{SimMode::kFreeRun};
    bool        enable_viewer{true};
    // kFreeRun: publish /joint_states every N physics steps.
    // Increase to reduce DDS traffic when the sim runs much faster than 500 Hz.
    int         publish_decimation{1};
    // kSyncStep: max time to wait for a command before using the previous one.
    double      sync_timeout_ms{50.0};
    // Maximum Real-Time Factor (0.0 = unlimited).
    // Both sim loops sleep after each mj_step() to keep RTF ≤ max_rtf.
    // Example: 10.0 → sim runs at most 10× real time.
    double      max_rtf{0.0};
    // Initial joint positions (radians) — UR5e safe upright pose.
    std::array<double, 6> initial_qpos{
        0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};
  };

  // Invoked from SimLoop after each publish step.
  // positions[0..5] = qpos mapped by joint name, velocities similarly.
  using StateCallback = std::function<void(
      const std::array<double, 6>& positions,
      const std::array<double, 6>& velocities)>;

  explicit MuJoCoSimulator(Config cfg) noexcept;
  ~MuJoCoSimulator();

  MuJoCoSimulator(const MuJoCoSimulator&)            = delete;
  MuJoCoSimulator& operator=(const MuJoCoSimulator&) = delete;
  MuJoCoSimulator(MuJoCoSimulator&&)                 = delete;
  MuJoCoSimulator& operator=(MuJoCoSimulator&&)      = delete;

  // Load MJCF model and resolve joint indices.  Must be called before Start().
  [[nodiscard]] bool Initialize() noexcept;

  // Start simulation and viewer threads.
  void Start() noexcept;

  // Signal stop and join all threads.
  void Stop() noexcept;

  // Write a position command into the pending buffer.
  // Thread-safe.  In kSyncStep mode also wakes the simulation loop.
  void SetCommand(const std::array<double, 6>& cmd) noexcept;

  // Register the callback invoked after each publish step.
  void SetStateCallback(StateCallback cb) noexcept;

  [[nodiscard]] std::array<double, 6> GetPositions()  const noexcept;
  [[nodiscard]] std::array<double, 6> GetVelocities() const noexcept;

  [[nodiscard]] bool     IsRunning()  const noexcept { return running_.load(); }
  [[nodiscard]] uint64_t StepCount()  const noexcept { return step_count_.load(); }
  [[nodiscard]] double   SimTimeSec() const noexcept { return sim_time_sec_.load(); }
  [[nodiscard]] int      NumJoints()  const noexcept { return model_ ? model_->nq : 0; }
  // Real-Time Factor: sim_time / wall_time (updated every 200 steps).
  // Returns 0.0 before the first measurement window completes.
  [[nodiscard]] double   GetRtf()     const noexcept { return rtf_.load(std::memory_order_relaxed); }

 private:
  Config   cfg_;
  mjModel* model_{nullptr};
  mjData*  data_{nullptr};

  std::atomic<bool>     running_{false};
  std::atomic<uint64_t> step_count_{0};
  std::atomic<double>   sim_time_sec_{0.0};

  // Command buffer.
  // cmd_pending_ is the lock-free fast-path flag (FreeRun: checked every step
  // without locking).  cmd_mutex_ guards pending_cmd_ during the actual copy.
  mutable std::mutex    cmd_mutex_;
  std::atomic<bool>     cmd_pending_{false};
  std::array<double, 6> pending_cmd_{};

  // SyncStep synchronisation — woken by SetCommand(); also woken by Stop().
  std::mutex              sync_mutex_;
  std::condition_variable sync_cv_;

  // State buffer — SimLoop writes, callers read.
  mutable std::mutex    state_mutex_;
  std::array<double, 6> latest_positions_{};
  std::array<double, 6> latest_velocities_{};

  // Viewer double-buffer — SimLoop writes with try_lock, ViewerLoop reads.
  mutable std::mutex  viz_mutex_;
  std::vector<double> viz_qpos_{};
  bool                viz_dirty_{false};

  StateCallback state_cb_{nullptr};

  std::jthread sim_thread_;
  std::jthread viewer_thread_;

  // Joint index maps resolved from MJCF joint names at initialise time.
  // joint_qpos_indices_[i] = data_->qpos offset for the i-th UR5e joint.
  // joint_qvel_indices_[i] = data_->qvel offset for the i-th UR5e joint.
  std::array<int, 6> joint_qpos_indices_{0, 1, 2, 3, 4, 5};
  std::array<int, 6> joint_qvel_indices_{0, 1, 2, 3, 4, 5};

  // Real-Time Factor (RTF = Δsim_time / Δwall_time).
  // rtf_wall_start_ and rtf_sim_start_ are only accessed from the sim thread
  // (no mutex needed).  rtf_ is written by the sim thread and read by the
  // viewer thread using relaxed ordering (stale reads are acceptable).
  std::chrono::steady_clock::time_point rtf_wall_start_{};
  double                                rtf_sim_start_{0.0};
  std::atomic<double>                   rtf_{0.0};

  // Max-RTF throttle reference (sim thread only, set at loop start).
  // ThrottleIfNeeded() sleeps to keep RTF ≤ cfg_.max_rtf.
  std::chrono::steady_clock::time_point throttle_wall_start_{};
  double                                throttle_sim_start_{0.0};

  // ── Internal helpers ───────────────────────────────────────────────────────
  // Resolve joint names → qpos/qvel indices using mj_name2id().
  // Falls back to static 0–5 mapping if a joint is not found.
  void ResolveJointIndices() noexcept;

  // Apply pending_cmd_ to data_->ctrl (holds cmd_mutex_ internally).
  void ApplyCommand() noexcept;

  // Copy data_->qpos / qvel → latest_positions_ / latest_velocities_
  // using resolved joint indices (holds state_mutex_ internally).
  void ReadState() noexcept;

  // Snapshot state under mutex then invoke state_cb_ without holding it.
  void InvokeStateCallback() noexcept;

  // Update viz_qpos_ using try_lock (never blocks SimLoop).
  void UpdateVizBuffer() noexcept;

  // Update rtf_ every 200 steps (sim thread only).
  void UpdateRtf(uint64_t step) noexcept;

  // Sleep after each mj_step() to enforce cfg_.max_rtf (no-op when max_rtf==0).
  void ThrottleIfNeeded() noexcept;

  void SimLoopFreeRun(std::stop_token stop) noexcept;
  void SimLoopSyncStep(std::stop_token stop) noexcept;
  void ViewerLoop(std::stop_token stop) noexcept;
};

// ── Joint name table (must match UR driver / custom_controller) ───────────────
static constexpr std::array<const char*, 6> kMjJointNames = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
};

// ── Inline implementation ──────────────────────────────────────────────────────

inline MuJoCoSimulator::MuJoCoSimulator(Config cfg) noexcept
    : cfg_(std::move(cfg)) {}

inline MuJoCoSimulator::~MuJoCoSimulator() {
  Stop();
  if (data_)  { mj_deleteData(data_);   data_  = nullptr; }
  if (model_) { mj_deleteModel(model_); model_ = nullptr; }
}

inline void MuJoCoSimulator::ResolveJointIndices() noexcept {
  for (std::size_t i = 0; i < 6; ++i) {
    const int jnt_id = mj_name2id(model_, mjOBJ_JOINT, kMjJointNames[i]);
    if (jnt_id < 0) {
      fprintf(stderr,
              "[MuJoCoSimulator] Joint '%s' not found in model "
              "— falling back to static index %zu\n",
              kMjJointNames[i], i);
      joint_qpos_indices_[i] = static_cast<int>(i);
      joint_qvel_indices_[i] = static_cast<int>(i);
    } else {
      joint_qpos_indices_[i] = model_->jnt_qposadr[jnt_id];
      joint_qvel_indices_[i] = model_->jnt_dofadr[jnt_id];
      fprintf(stdout,
              "[MuJoCoSimulator] '%s' → qpos[%d]  qvel[%d]\n",
              kMjJointNames[i],
              joint_qpos_indices_[i],
              joint_qvel_indices_[i]);
    }
  }
}

inline bool MuJoCoSimulator::Initialize() noexcept {
  char error[1000] = {};
  model_ = mj_loadXML(cfg_.model_path.c_str(), nullptr, error, sizeof(error));
  if (!model_) {
    fprintf(stderr, "[MuJoCoSimulator] Failed to load '%s': %s\n",
            cfg_.model_path.c_str(), error);
    return false;
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    fprintf(stderr, "[MuJoCoSimulator] mj_makeData failed\n");
    mj_deleteModel(model_);
    model_ = nullptr;
    return false;
  }

  viz_qpos_.assign(static_cast<std::size_t>(model_->nq), 0.0);

  // Resolve joint indices using model joint names (dynamic, not static mapping).
  ResolveJointIndices();

  // Set initial joint positions and matching ctrl targets.
  const int njoints = std::min(6, model_->nq);
  for (int i = 0; i < njoints; ++i) {
    const double q0 = cfg_.initial_qpos[static_cast<std::size_t>(i)];
    const int    qi = joint_qpos_indices_[static_cast<std::size_t>(i)];
    data_->qpos[qi] = q0;
    // ctrl uses actuator order (matches command order 0–5)
    data_->ctrl[i] = q0;
  }

  mj_forward(model_, data_);
  ReadState();

  fprintf(stdout,
          "[MuJoCoSimulator] Loaded '%s'  nq=%d  nv=%d  nu=%d  dt=%.4f s"
          "  mode=%s\n",
          cfg_.model_path.c_str(),
          model_->nq, model_->nv, model_->nu,
          static_cast<double>(model_->opt.timestep),
          cfg_.mode == SimMode::kFreeRun ? "free_run" : "sync_step");
  return true;
}

inline void MuJoCoSimulator::Start() noexcept {
  if (running_.exchange(true)) { return; }

  if (cfg_.mode == SimMode::kFreeRun) {
    sim_thread_ = std::jthread([this](std::stop_token st) { SimLoopFreeRun(st); });
  } else {
    sim_thread_ = std::jthread([this](std::stop_token st) { SimLoopSyncStep(st); });
  }

  if (cfg_.enable_viewer) {
    viewer_thread_ = std::jthread([this](std::stop_token st) { ViewerLoop(st); });
  }
}

inline void MuJoCoSimulator::Stop() noexcept {
  running_.store(false);
  sync_cv_.notify_all();  // wake SyncStep loop if it is waiting
  if (sim_thread_.joinable()) {
    sim_thread_.request_stop();
    sim_thread_.join();
  }
  if (viewer_thread_.joinable()) {
    viewer_thread_.request_stop();
    viewer_thread_.join();
  }
}

inline void MuJoCoSimulator::SetCommand(const std::array<double, 6>& cmd) noexcept {
  {
    std::lock_guard lock(cmd_mutex_);
    pending_cmd_ = cmd;
  }
  cmd_pending_.store(true, std::memory_order_release);
  if (cfg_.mode == SimMode::kSyncStep) {
    sync_cv_.notify_one();
  }
}

inline void MuJoCoSimulator::SetStateCallback(StateCallback cb) noexcept {
  state_cb_ = std::move(cb);
}

inline std::array<double, 6> MuJoCoSimulator::GetPositions() const noexcept {
  std::lock_guard lock(state_mutex_);
  return latest_positions_;
}

inline std::array<double, 6> MuJoCoSimulator::GetVelocities() const noexcept {
  std::lock_guard lock(state_mutex_);
  return latest_velocities_;
}

// ── Private helpers ────────────────────────────────────────────────────────────

inline void MuJoCoSimulator::ApplyCommand() noexcept {
  std::lock_guard lock(cmd_mutex_);
  if (!model_) { return; }
  const int nact = std::min(6, model_->nu);
  for (int i = 0; i < nact; ++i) {
    data_->ctrl[i] = pending_cmd_[static_cast<std::size_t>(i)];
  }
}

inline void MuJoCoSimulator::ReadState() noexcept {
  if (!model_ || !data_) { return; }
  std::lock_guard lock(state_mutex_);
  for (std::size_t i = 0; i < 6; ++i) {
    latest_positions_[i]  = data_->qpos[joint_qpos_indices_[i]];
    latest_velocities_[i] = data_->qvel[joint_qvel_indices_[i]];
  }
}

inline void MuJoCoSimulator::InvokeStateCallback() noexcept {
  if (!state_cb_) { return; }
  std::array<double, 6> pos{}, vel{};
  {
    std::lock_guard lock(state_mutex_);
    pos = latest_positions_;
    vel = latest_velocities_;
  }
  state_cb_(pos, vel);
}

inline void MuJoCoSimulator::UpdateVizBuffer() noexcept {
  if (viz_mutex_.try_lock()) {
    std::memcpy(viz_qpos_.data(), data_->qpos,
                static_cast<std::size_t>(model_->nq) * sizeof(double));
    viz_dirty_ = true;
    viz_mutex_.unlock();
  }
}

// ── UpdateRtf ──────────────────────────────────────────────────────────────────
//
// Called after every physics step from the sim thread.
// Refreshes rtf_ every 200 steps (rolling window) when at least 10 ms of
// wall-clock time has elapsed — avoids division by near-zero at startup.
//
inline void MuJoCoSimulator::UpdateRtf(uint64_t step) noexcept {
  if (step % 200 != 0) { return; }

  const auto   wall_now = std::chrono::steady_clock::now();
  const double wall_dt  =
      std::chrono::duration<double>(wall_now - rtf_wall_start_).count();
  const double sim_dt   = data_->time - rtf_sim_start_;

  if (wall_dt > 0.01) {  // require ≥10 ms to avoid near-zero division
    rtf_.store(sim_dt / wall_dt, std::memory_order_relaxed);
    rtf_wall_start_ = wall_now;
    rtf_sim_start_  = data_->time;
  }
}

// ── ThrottleIfNeeded ───────────────────────────────────────────────────────────
//
// Enforces cfg_.max_rtf by sleeping after each physics step.
// The reference point (throttle_wall_start_, throttle_sim_start_) is set once
// at the start of each SimLoop and never reset, so the constraint applies to
// the cumulative sim_time / wall_time ratio from loop start.
//
// If wall time falls behind (e.g., due to OS jitter), the loop simply runs at
// full speed until the ratio is restored — no "catch-up" in reverse.
// No-op when cfg_.max_rtf == 0.0.
//
inline void MuJoCoSimulator::ThrottleIfNeeded() noexcept {
  if (cfg_.max_rtf <= 0.0) { return; }

  const double sim_elapsed  = data_->time - throttle_sim_start_;
  const double target_wall  = sim_elapsed / cfg_.max_rtf;
  const double actual_wall  = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - throttle_wall_start_).count();

  if (actual_wall < target_wall) {
    std::this_thread::sleep_for(
        std::chrono::duration<double>(target_wall - actual_wall));
  }
}

// ── SimLoopFreeRun ─────────────────────────────────────────────────────────────
//
// Advances physics as fast as possible (no nanosleep / wall-clock sync).
// Publishes /joint_states every cfg_.publish_decimation steps.
// Use lock-free cmd_pending_ atomic to avoid mutex overhead on the hot path.
//
inline void MuJoCoSimulator::SimLoopFreeRun(std::stop_token stop) noexcept {
  if (!model_ || !data_) { return; }

  const auto decim = static_cast<uint64_t>(
      cfg_.publish_decimation > 0 ? cfg_.publish_decimation : 1);

  uint64_t step = 0;

  // Initialise RTF tracking and max-RTF throttle windows.
  const auto loop_start = std::chrono::steady_clock::now();
  rtf_wall_start_      = loop_start;
  rtf_sim_start_       = data_->time;
  throttle_wall_start_ = loop_start;
  throttle_sim_start_  = data_->time;

  while (!stop.stop_requested() && running_.load()) {
    // Lock-free fast path: acquire mutex only when a command arrived.
    if (cmd_pending_.load(std::memory_order_acquire)) {
      ApplyCommand();
      cmd_pending_.store(false, std::memory_order_release);
    }

    mj_step(model_, data_);
    ++step;
    step_count_.store(step, std::memory_order_relaxed);
    sim_time_sec_.store(data_->time, std::memory_order_relaxed);

    // Publish state every N steps.
    if (step % decim == 0) {
      ReadState();
      InvokeStateCallback();
    }

    // Update RTF every 200 steps.
    UpdateRtf(step);

    // Enforce max_rtf speed cap (no-op when max_rtf == 0).
    ThrottleIfNeeded();

    // Update viewer buffer at ~1/8 rate (avoid starving viewer with mutex contention).
    if ((step % 8 == 0) && cfg_.enable_viewer) {
      UpdateVizBuffer();
    }
  }

  fprintf(stdout,
          "[MuJoCoSimulator] FreeRun exited — steps=%lu  sim_time=%.3f s\n",
          static_cast<unsigned long>(step_count_.load()),
          sim_time_sec_.load());
}

// ── SimLoopSyncStep ────────────────────────────────────────────────────────────
//
// 1. Publish current state.
// 2. Wait for one command from the controller (or sync_timeout_ms).
// 3. Apply command and take one physics step.
//
// Step latency ≈ controller Compute() time → direct timing measurement.
//
inline void MuJoCoSimulator::SimLoopSyncStep(std::stop_token stop) noexcept {
  if (!model_ || !data_) { return; }

  const auto timeout = std::chrono::milliseconds(
      static_cast<int64_t>(cfg_.sync_timeout_ms > 0.0 ? cfg_.sync_timeout_ms : 50.0));

  uint64_t step = 0;

  // Initialise RTF tracking and max-RTF throttle windows.
  const auto loop_start_ss = std::chrono::steady_clock::now();
  rtf_wall_start_          = loop_start_ss;
  rtf_sim_start_           = data_->time;
  throttle_wall_start_     = loop_start_ss;
  throttle_sim_start_      = data_->time;

  while (!stop.stop_requested() && running_.load()) {
    // 1. Publish current state to controller.
    ReadState();
    InvokeStateCallback();

    // 2. Wait for controller command (or timeout → reuse previous command).
    {
      std::unique_lock lock(sync_mutex_);
      sync_cv_.wait_for(lock, timeout, [this, &stop] {
        return cmd_pending_.load(std::memory_order_relaxed) ||
               stop.stop_requested() || !running_.load();
      });
    }

    if (stop.stop_requested() || !running_.load()) { break; }

    // 3. Apply command and advance physics.
    if (cmd_pending_.load(std::memory_order_acquire)) {
      ApplyCommand();
      cmd_pending_.store(false, std::memory_order_release);
    }
    mj_step(model_, data_);
    ++step;
    step_count_.store(step, std::memory_order_relaxed);
    sim_time_sec_.store(data_->time, std::memory_order_relaxed);

    // Update RTF every 200 steps.
    UpdateRtf(step);

    // Enforce max_rtf speed cap (no-op when max_rtf == 0).
    ThrottleIfNeeded();

    if ((step % 8 == 0) && cfg_.enable_viewer) {
      UpdateVizBuffer();
    }
  }

  fprintf(stdout,
          "[MuJoCoSimulator] SyncStep exited — steps=%lu  sim_time=%.3f s\n",
          static_cast<unsigned long>(step_count_.load()),
          sim_time_sec_.load());
}

// ── ViewerLoop ─────────────────────────────────────────────────────────────────
inline void MuJoCoSimulator::ViewerLoop(std::stop_token stop) noexcept {
#ifdef MUJOCO_HAVE_GLFW
  if (!glfwInit()) {
    fprintf(stderr, "[MuJoCoSimulator] glfwInit failed — viewer disabled\n");
    return;
  }

  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

  GLFWwindow* window = glfwCreateWindow(
      1200, 900, "UR5e MuJoCo Simulator", nullptr, nullptr);
  if (!window) {
    fprintf(stderr,
            "[MuJoCoSimulator] glfwCreateWindow failed — viewer disabled\n");
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  mjvCamera  cam;
  mjvOption  opt;
  mjvScene   scn;
  mjrContext con;

  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  mjv_makeScene(model_, &scn, 2000);
  mjr_makeContext(model_, &con, mjFONTSCALE_150);

  cam.type      = mjCAMERA_FREE;
  cam.distance  = 2.5;
  cam.azimuth   = 90.0;
  cam.elevation = -20.0;

  // Visualization-only mjData — never written by the sim thread.
  mjData* vis_data = mj_makeData(model_);
  {
    std::lock_guard lock(state_mutex_);
    for (std::size_t i = 0; i < 6; ++i) {
      vis_data->qpos[joint_qpos_indices_[i]] =
          latest_positions_[i];
    }
  }
  mj_forward(model_, vis_data);

  fprintf(stdout, "[MuJoCoSimulator] Viewer running (close window to stop)\n");

  while (!stop.stop_requested() && running_.load() &&
         !glfwWindowShouldClose(window)) {
    {
      std::lock_guard lock(viz_mutex_);
      if (viz_dirty_) {
        std::memcpy(vis_data->qpos, viz_qpos_.data(),
                    static_cast<std::size_t>(model_->nq) * sizeof(double));
        viz_dirty_ = false;
      }
    }
    mj_forward(model_, vis_data);

    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    mjrRect viewport{0, 0, width, height};

    mjv_updateScene(model_, vis_data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // ── RTF overlay (top-right corner) ──────────────────────────────────────
    {
      char labels[64], values[64];
      const double rtf_val = rtf_.load(std::memory_order_relaxed);
      std::snprintf(labels, sizeof(labels),
                    "Real-Time Factor\nSim Time\nSteps\nMode");
      std::snprintf(values, sizeof(values),
                    "%.1fx\n%.2f s\n%lu\n%s",
                    rtf_val,
                    sim_time_sec_.load(std::memory_order_relaxed),
                    static_cast<unsigned long>(step_count_.load(std::memory_order_relaxed)),
                    cfg_.mode == SimMode::kFreeRun ? "free_run" : "sync_step");
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport,
                  labels, values, &con);
    }

    glfwSwapBuffers(window);
    glfwPollEvents();

    std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 Hz
  }

  mj_deleteData(vis_data);
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  glfwDestroyWindow(window);
  glfwTerminate();
  fprintf(stdout, "[MuJoCoSimulator] Viewer closed\n");

#else
  fprintf(stdout,
          "[MuJoCoSimulator] Viewer not available "
          "(build without -DMUJOCO_HAVE_GLFW)\n");
  (void)stop;
#endif
}

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_
