#ifndef UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_
#define UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_

// ── Includes: project, then MuJoCo, then C++ stdlib ───────────────────────────
#include <mujoco/mujoco.h>

// Optional GLFW viewer — define MUJOCO_HAVE_GLFW in CMakeLists.txt when found
#ifdef MUJOCO_HAVE_GLFW
#include <GLFW/glfw3.h>
#endif

#include <array>
#include <atomic>
#include <chrono>
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
// Threading model:
//   SimLoop  thread  — advances mj_step() at cfg.control_freq Hz; writes
//                      latest_positions_ / latest_velocities_ under state_mutex_;
//                      invokes state_cb_ (used to publish /joint_states).
//   ViewerLoop thread — reads viz_qpos_ (updated by SimLoop via try_lock) and
//                       renders the scene using GLFW + MuJoCo rendering API.
//   Caller thread     — calls SetCommand() (protected by cmd_mutex_) and
//                       GetPositions() / GetVelocities() (protected by state_mutex_).
//
// Synchronisation:
//   cmd_mutex_   — protects pending_cmd_ (caller → sim)
//   state_mutex_ — protects latest_positions_ / latest_velocities_ (sim → caller)
//   viz_mutex_   — protects viz_qpos_ (sim → viewer); SimLoop uses try_lock to
//                  avoid blocking the timing-critical step.
//
class MuJoCoSimulator {
 public:
  struct Config {
    std::string model_path;   // absolute path to scene.xml
    double      control_freq{500.0};  // simulation step frequency [Hz]
    bool        enable_viewer{true};  // launch GLFW viewer thread
    bool        realtime{true};       // sleep to match wall-clock
    double      sim_speed{1.0};       // realtime multiplier (2.0 = 2x faster)
    // Initial joint positions (radians) — UR5e safe pose
    std::array<double, 6> initial_positions{
        0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};
  };

  // Called from SimLoop after each step.
  // positions[0..5] = qpos, velocities[0..5] = qvel.
  using StateCallback = std::function<void(
      const std::array<double, 6>& positions,
      const std::array<double, 6>& velocities)>;

  explicit MuJoCoSimulator(Config cfg) noexcept;
  ~MuJoCoSimulator();

  MuJoCoSimulator(const MuJoCoSimulator&)            = delete;
  MuJoCoSimulator& operator=(const MuJoCoSimulator&) = delete;
  MuJoCoSimulator(MuJoCoSimulator&&)                 = delete;
  MuJoCoSimulator& operator=(MuJoCoSimulator&&)      = delete;

  // Load MJCF model and allocate mjData.  Must be called before Start().
  // Returns false if the XML fails to parse.
  [[nodiscard]] bool Initialize() noexcept;

  // Start simulation loop (and viewer loop if cfg.enable_viewer).
  void Start() noexcept;

  // Request stop and join threads.
  void Stop() noexcept;

  // Write a position target into the pending command buffer.
  // Thread-safe; called from ROS2 subscription callback.
  void SetCommand(const std::array<double, 6>& cmd) noexcept;

  // Register the callback invoked after every simulation step.
  void SetStateCallback(StateCallback cb) noexcept;

  // Read the latest joint state (polling alternative to callback).
  [[nodiscard]] std::array<double, 6> GetPositions()  const noexcept;
  [[nodiscard]] std::array<double, 6> GetVelocities() const noexcept;

  [[nodiscard]] bool     IsRunning()  const noexcept { return running_.load(); }
  [[nodiscard]] uint64_t StepCount()  const noexcept { return step_count_.load(); }
  [[nodiscard]] int      NumJoints()  const noexcept { return model_ ? model_->nq : 0; }

 private:
  Config   cfg_;
  mjModel* model_{nullptr};
  mjData*  data_{nullptr};

  std::atomic<bool>     running_{false};
  std::atomic<uint64_t> step_count_{0};

  // Command buffer — caller writes, SimLoop reads
  mutable std::mutex    cmd_mutex_;
  std::array<double, 6> pending_cmd_{};
  bool                  cmd_dirty_{false};

  // State buffer — SimLoop writes, caller reads
  mutable std::mutex    state_mutex_;
  std::array<double, 6> latest_positions_{};
  std::array<double, 6> latest_velocities_{};

  // Visualization buffer — SimLoop writes with try_lock, ViewerLoop reads
  mutable std::mutex  viz_mutex_;
  std::vector<double> viz_qpos_{};
  bool                viz_dirty_{false};

  StateCallback state_cb_{nullptr};

  std::jthread sim_thread_;
  std::jthread viewer_thread_;

  // ── Internal helpers ───────────────────────────────────────────────────────
  // Apply pending_cmd_ to data_->ctrl (called from SimLoop, holds cmd_mutex_).
  void ApplyCommand() noexcept;

  // Copy data_->qpos / qvel → latest_positions_ / latest_velocities_
  // (called from SimLoop, holds state_mutex_).
  void ReadState() noexcept;

  void SimLoop(std::stop_token stop) noexcept;
  void ViewerLoop(std::stop_token stop) noexcept;
};

// ── Inline implementation ──────────────────────────────────────────────────────

inline MuJoCoSimulator::MuJoCoSimulator(Config cfg) noexcept
    : cfg_(std::move(cfg)) {}

inline MuJoCoSimulator::~MuJoCoSimulator() {
  Stop();
  if (data_)  { mj_deleteData(data_);   data_  = nullptr; }
  if (model_) { mj_deleteModel(model_); model_ = nullptr; }
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

  // Allocate visualization buffer sized to nq
  viz_qpos_.assign(static_cast<std::size_t>(model_->nq), 0.0);

  // Set initial joint positions and matching ctrl targets
  const int njoints = std::min(6, model_->nq);
  for (int i = 0; i < njoints; ++i) {
    const double q0 = cfg_.initial_positions[static_cast<std::size_t>(i)];
    data_->qpos[i] = q0;
    data_->ctrl[i] = q0;
  }

  // Forward kinematics to initialise internal state
  mj_forward(model_, data_);
  ReadState();

  fprintf(stdout,
          "[MuJoCoSimulator] Loaded '%s'  nq=%d  nv=%d  nu=%d  dt=%.4f s\n",
          cfg_.model_path.c_str(),
          model_->nq, model_->nv, model_->nu,
          static_cast<double>(model_->opt.timestep));
  return true;
}

inline void MuJoCoSimulator::Start() noexcept {
  if (running_.exchange(true)) {
    return;  // already running
  }
  sim_thread_ = std::jthread([this](std::stop_token st) { SimLoop(st); });
  if (cfg_.enable_viewer) {
    viewer_thread_ = std::jthread([this](std::stop_token st) { ViewerLoop(st); });
  }
}

inline void MuJoCoSimulator::Stop() noexcept {
  running_.store(false);
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
  std::lock_guard lock(cmd_mutex_);
  pending_cmd_ = cmd;
  cmd_dirty_   = true;
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
  if (!cmd_dirty_ || !model_) { return; }
  const int nact = std::min(6, model_->nu);
  for (int i = 0; i < nact; ++i) {
    data_->ctrl[i] = pending_cmd_[static_cast<std::size_t>(i)];
  }
  cmd_dirty_ = false;
}

inline void MuJoCoSimulator::ReadState() noexcept {
  if (!model_ || !data_) { return; }
  std::lock_guard lock(state_mutex_);
  const int nq = std::min(6, model_->nq);
  const int nv = std::min(6, model_->nv);
  for (int i = 0; i < nq; ++i) {
    latest_positions_[static_cast<std::size_t>(i)]  = data_->qpos[i];
  }
  for (int i = 0; i < nv; ++i) {
    latest_velocities_[static_cast<std::size_t>(i)] = data_->qvel[i];
  }
}

// ── SimLoop ────────────────────────────────────────────────────────────────────
inline void MuJoCoSimulator::SimLoop(std::stop_token stop) noexcept {
  if (!model_ || !data_) { return; }

  // Compute step period in nanoseconds (adjusted by sim_speed multiplier)
  const long period_ns = static_cast<long>(
      1.0e9 / (cfg_.control_freq * cfg_.sim_speed));

  auto next_wake = std::chrono::steady_clock::now();

  while (!stop.stop_requested() && running_.load()) {
    // 1. Push pending position command into ctrl
    ApplyCommand();

    // 2. Advance physics by one timestep
    mj_step(model_, data_);
    const uint64_t count =
        step_count_.fetch_add(1, std::memory_order_relaxed) + 1;

    // 3. Copy new state into shared buffers
    ReadState();

    // 4. Update visualization buffer at ~62 Hz (every 8th step at 500 Hz).
    //    try_lock avoids blocking the RT sim loop if the viewer thread holds the mutex.
    if ((count % 8 == 0) && cfg_.enable_viewer) {
      if (viz_mutex_.try_lock()) {
        std::memcpy(viz_qpos_.data(), data_->qpos,
                    static_cast<std::size_t>(model_->nq) * sizeof(double));
        viz_dirty_ = true;
        viz_mutex_.unlock();
      }
    }

    // 5. Notify the ROS2 node (publishes /joint_states).
    //    Snapshot the state under the mutex, then call without holding it.
    if (state_cb_) {
      std::array<double, 6> pos{}, vel{};
      {
        std::lock_guard lock(state_mutex_);
        pos = latest_positions_;
        vel = latest_velocities_;
      }
      state_cb_(pos, vel);
    }

    // 6. Real-time sleep — advance deadline by one step period
    if (cfg_.realtime) {
      next_wake += std::chrono::nanoseconds(period_ns);
      std::this_thread::sleep_until(next_wake);
    }
  }

  fprintf(stdout, "[MuJoCoSimulator] SimLoop exited after %lu steps\n",
          static_cast<unsigned long>(step_count_.load()));
}

// ── ViewerLoop ─────────────────────────────────────────────────────────────────
inline void MuJoCoSimulator::ViewerLoop(std::stop_token stop) noexcept {
#ifdef MUJOCO_HAVE_GLFW
  // ── GLFW + OpenGL viewer ───────────────────────────────────────────────────
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
  glfwSwapInterval(1);  // vsync

  // ── MuJoCo visualization structures ───────────────────────────────────────
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

  // Initial camera position — looking from the front-right above
  cam.type      = mjCAMERA_FREE;
  cam.distance  = 2.5;
  cam.azimuth   = 90.0;
  cam.elevation = -20.0;

  // ── Visualization-only mjData copy ────────────────────────────────────────
  // This mjData is only used to compute forward kinematics for rendering;
  // it is NEVER written to by the sim thread.
  mjData* vis_data = mj_makeData(model_);
  {
    // Initialise from the latest known state
    std::lock_guard lock(state_mutex_);
    const int nq = std::min(6, model_->nq);
    for (int i = 0; i < nq; ++i) {
      vis_data->qpos[i] = latest_positions_[static_cast<std::size_t>(i)];
    }
  }
  mj_forward(model_, vis_data);

  fprintf(stdout, "[MuJoCoSimulator] Viewer running (close window to stop)\n");

  while (!stop.stop_requested() && running_.load() &&
         !glfwWindowShouldClose(window)) {
    // ── Sync vis_data with latest simulation state ─────────────────────────
    {
      std::lock_guard lock(viz_mutex_);
      if (viz_dirty_) {
        std::memcpy(vis_data->qpos, viz_qpos_.data(),
                    static_cast<std::size_t>(model_->nq) * sizeof(double));
        viz_dirty_ = false;
      }
    }
    // Recompute FK so geom positions are up to date
    mj_forward(model_, vis_data);

    // ── Render ────────────────────────────────────────────────────────────
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    mjrRect viewport{0, 0, width, height};

    mjv_updateScene(model_, vis_data, &opt, nullptr, &cam,
                    mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    glfwSwapBuffers(window);
    glfwPollEvents();

    // ~60 Hz viewer update
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }

  // ── Cleanup ────────────────────────────────────────────────────────────────
  mj_deleteData(vis_data);
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  glfwDestroyWindow(window);
  glfwTerminate();
  fprintf(stdout, "[MuJoCoSimulator] Viewer closed\n");

#else
  // ── Headless stub ──────────────────────────────────────────────────────────
  fprintf(stdout,
          "[MuJoCoSimulator] Viewer not available "
          "(build without -DMUJOCO_HAVE_GLFW)\n");
  (void)stop;
#endif
}

}  // namespace ur5e_rt_controller

#endif  // UR5E_RT_CONTROLLER_MUJOCO_SIMULATOR_HPP_
