// ── viewer_loop.cpp ────────────────────────────────────────────────────────────
// MuJoCoSimulator::ViewerLoop — GLFW 3D viewer rendered at ~60 Hz.
//
// This file implements the ViewerLoop member function, which keeps access to
// private members (viz_mutex_, viz_qpos_, model_, rtf_, etc.).
// All input handling is delegated to viewer_callbacks.cpp.
// All overlay rendering is delegated to viewer_overlays.cpp.
// ──────────────────────────────────────────────────────────────────────────────
#include "viewer/viewer_state.hpp"
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#ifdef MUJOCO_HAVE_GLFW
#include <GLFW/glfw3.h>
#endif

#include <chrono>
#include <cstdio>
#include <cstdlib>    // getenv
#include <cstring>
#include <ctime>
#include <thread>
#include <vector>

#ifdef MUJOCO_HAVE_GLFW
#include <sys/stat.h>  // mkdir
#endif

namespace rtc {

void MuJoCoSimulator::ViewerLoop(std::stop_token stop) noexcept {
#ifdef MUJOCO_HAVE_GLFW
  if (!glfwInit()) {
    fprintf(stderr, "[MuJoCoSimulator] glfwInit failed — viewer disabled\n");
    return;
  }

  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

  constexpr int kDefaultWindowWidth  = 1280;
  constexpr int kDefaultWindowHeight = 960;
  const char* window_title = cfg_.window_title.empty()
      ? "MuJoCo Simulator" : cfg_.window_title.c_str();
  GLFWwindow* window =
      glfwCreateWindow(kDefaultWindowWidth, kDefaultWindowHeight, window_title, nullptr, nullptr);
  if (!window) {
    fprintf(stderr, "[MuJoCoSimulator] glfwCreateWindow failed\n");
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // ── MuJoCo rendering structs ───────────────────────────────────────────────
  mjvCamera cam;   mjv_defaultCamera(&cam);
  mjvOption opt;   mjv_defaultOption(&opt);
  mjvScene  scn;   mjv_defaultScene(&scn);
  mjrContext con;  mjr_defaultContext(&con);

  mjv_makeScene(model_, &scn, 2000);
  mjr_makeContext(model_, &con, mjFONTSCALE_100);

  constexpr double kDefaultCameraDistance  = 2.5;
  constexpr double kDefaultCameraAzimuth   = 90.0;
  constexpr double kDefaultCameraElevation = -20.0;
  cam.type      = mjCAMERA_FREE;
  cam.distance  = kDefaultCameraDistance;
  cam.azimuth   = kDefaultCameraAzimuth;
  cam.elevation = kDefaultCameraElevation;

  // ── Visualisation-only mjData (only qpos is synced from physics thread) ────
  mjData* vis_data = mj_makeData(model_);
  for (auto& g : groups_) {
    if (!g->is_robot) continue;
    std::lock_guard lock(g->state_mutex);
    for (std::size_t i = 0; i < static_cast<std::size_t>(g->num_state_joints); ++i) {
      vis_data->qpos[g->state_qpos_indices[i]] = g->positions[i];
    }
  }
  mj_forward(model_, vis_data);

  // ── RTF profiler figure ────────────────────────────────────────────────────
  mjvFigure fig_profiler;
  mjv_defaultFigure(&fig_profiler);
  std::strncpy(fig_profiler.title,       "RTF History",   sizeof(fig_profiler.title)   - 1);
  std::strncpy(fig_profiler.xlabel,      "Frames (~60Hz)",sizeof(fig_profiler.xlabel)  - 1);
  std::strncpy(fig_profiler.linename[0], "RTF",           sizeof(fig_profiler.linename[0]) - 1);
  fig_profiler.figurergba[0] = 0.08f;
  fig_profiler.figurergba[1] = 0.08f;
  fig_profiler.figurergba[2] = 0.08f;
  fig_profiler.figurergba[3] = 0.88f;
  fig_profiler.linergb[0][0] = 0.2f;
  fig_profiler.linergb[0][1] = 1.0f;
  fig_profiler.linergb[0][2] = 0.4f;
  fig_profiler.linewidth      = 1.5f;
  fig_profiler.range[0][0]   = 0;
  fig_profiler.range[0][1]   = 200;
  fig_profiler.range[1][0]   = 0;
  fig_profiler.range[1][1]   = 30;
  fig_profiler.flg_extend    = 1;

  // ── ViewerState: accessed by all callbacks via glfwGetWindowUserPointer ────
  ViewerState vs;
  vs.cam          = &cam;
  vs.opt          = &opt;
  vs.scn          = &scn;
  vs.con          = &con;
  vs.model        = model_;
  vs.vis_data     = vis_data;
  vs.fig_profiler = &fig_profiler;
  vs.sim          = this;
  // Use body 1 (first non-world body) as default tracking target
  vs.track_body_id = (model_->nbody > 1) ? 1 : 0;
  mjv_defaultPerturb(&vs.pert);

  glfwSetWindowUserPointer(window, &vs);

  // ── Register callbacks (defined in viewer_callbacks.cpp) ──────────────────
  glfwSetKeyCallback        (window, OnKey);
  glfwSetMouseButtonCallback(window, OnMouseButton);
  glfwSetCursorPosCallback  (window, OnCursorPos);
  glfwSetScrollCallback     (window, OnScroll);

  fprintf(stdout,
      "[MuJoCoSimulator] Viewer ready — press F1 in the window for help\n"
      "  Simulation : Space=pause  +/-=speed  Right=step  R=reset\n"
      "  Camera     : TAB=cycle  Left=orbit  Right=pan  Scroll=zoom  Esc=reset\n"
      "  Physics    : G=gravity  N=contacts\n"
      "  Solver     : I=integrator  S=solver  ]/[=iterations  F4=stats\n"
      "  Visualise  : C=cpoints  F=cforces  0-5=geomgroups  J=joints\n"
      "               U=actuators  E=inertia  W=sites  L=lights  A=tendons  X=hulls\n"
      "               T=transp  F5=wireframe  F6=shadow  F7=skybox  F8=reflect\n"
      "  Overlays   : F3=profiler  F9=sensors  F10=modelinfo\n"
      "  Perturb    : Dbl-click=select  Ctrl+Left=torque  Ctrl+Right=force\n"
      "  Other      : P=screenshot\n");

  // ── Render loop ────────────────────────────────────────────────────────────
  while (!stop.stop_requested() && running_.load() &&
         !glfwWindowShouldClose(window)) {

    // Sync latest physics qpos into vis_data (try_lock — never blocks SimLoop)
    {
      std::lock_guard lock(viz_mutex_);
      if (viz_dirty_) {
        std::memcpy(vis_data->qpos, viz_qpos_.data(),
                    static_cast<std::size_t>(model_->nq) * sizeof(double));
        viz_dirty_ = false;
      }
    }
    mj_forward(model_, vis_data);  // update kinematics + sensor data

    // Sample RTF into rolling buffer
    const float cur_rtf = static_cast<float>(rtf_.load(std::memory_order_relaxed));
    vs.push_rtf(cur_rtf);
    vs.update_figure();

    // ── Render scene ──────────────────────────────────────────────────────
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    const mjrRect viewport{0, 0, width, height};

    mjv_updateScene(model_, vis_data, &opt, &vs.pert, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // ── Overlays ──────────────────────────────────────────────────────────
    // Status (top-right, always visible)
    RenderStatusOverlay(vs, viewport, cur_rtf);

    // Bottom-left hint or solver stats
    if (vs.show_solver) {
      RenderSolverOverlay(vs, viewport);
    } else {
      mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                  "F1: help", nullptr, &con);
    }

    // Top-left: help pages (F1) take priority over sensor (F9)
    if (vs.help_page > 0) {
      RenderHelpOverlay(vs, viewport, vs.help_page);
    } else if (vs.show_sensor) {
      RenderSensorOverlay(vs, viewport);
    }

    // Bottom-right: model info (F10)
    if (vs.show_model_info) {
      RenderModelInfoOverlay(vs, viewport);
    }

    // Bottom-right corner graph: RTF profiler (F3)
    RenderRtfProfiler(vs, viewport);

    // ── Screenshot (P key, written after all overlays are drawn) ─────────
    if (vs.take_screenshot) {
      vs.take_screenshot = false;
      const auto npix = static_cast<std::size_t>(width * height);
      std::vector<unsigned char> rgb(npix * 3);
      mjr_readPixels(rgb.data(), nullptr, viewport, &con);

      // 세션 디렉토리 기반 출력 경로 결정
      char dir[256];
      const char* session_env = std::getenv("UR5E_SESSION_DIR");
      if (session_env && session_env[0] != '\0') {
        std::snprintf(dir, sizeof(dir), "%s/sim", session_env);
      } else {
        const char* home = std::getenv("HOME");
        if (home) {
          std::snprintf(dir, sizeof(dir),
                        "%s/ros2_ws/ur5e_ws/logging_data", home);
        } else {
          std::strncpy(dir, "/tmp", sizeof(dir) - 1);
          dir[sizeof(dir) - 1] = '\0';
        }
      }
      mkdir(dir, 0755);  // no-op if already exists

      char fname[512];
      const auto now  = std::chrono::system_clock::now();
      const auto t    = std::chrono::system_clock::to_time_t(now);
      char       ts[32];
      std::strftime(ts, sizeof(ts), "%H%M%S", std::localtime(&t));
      std::snprintf(fname, sizeof(fname), "%s/screenshot_%s.ppm", dir, ts);

      // Write binary PPM (flip Y: OpenGL stores bottom-up)
      FILE* f = std::fopen(fname, "wb");
      if (f) {
        std::fprintf(f, "P6\n%d %d\n255\n", width, height);
        for (int y = height - 1; y >= 0; --y) {
          std::fwrite(rgb.data() + static_cast<std::size_t>(y * width) * 3,
                      1, static_cast<std::size_t>(width * 3), f);
        }
        std::fclose(f);
        fprintf(stdout, "[Viewer] Screenshot saved: %s\n", fname);
      } else {
        fprintf(stderr, "[Viewer] Failed to write screenshot: %s\n", fname);
      }
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(viewer_sleep_ms_));
  }

  // ── Cleanup ────────────────────────────────────────────────────────────────
  mj_deleteData(vis_data);
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  glfwDestroyWindow(window);
  glfwTerminate();
  fprintf(stdout, "[MuJoCoSimulator] Viewer closed\n");

#else
  fprintf(stdout,
          "[MuJoCoSimulator] Viewer not available (MUJOCO_HAVE_GLFW not set)\n");
  (void)stop;
#endif
}

}  // namespace rtc
