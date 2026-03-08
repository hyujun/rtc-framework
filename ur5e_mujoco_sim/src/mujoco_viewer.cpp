// ── mujoco_viewer.cpp
// ────────────────────────────────────────────────────────── ViewerLoop: GLFW
// 3D viewer with keyboard/mouse controls, overlays, RTF profiler graph, and
// solver stats display.
// ──────────────────────────────────────────────────────────────────────────────
#include "ur5e_mujoco_sim/mujoco_simulator.hpp"

#ifdef MUJOCO_HAVE_GLFW
#include <GLFW/glfw3.h>
#endif

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

namespace ur5e_rt_controller {

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

  GLFWwindow *window =
      glfwCreateWindow(1280, 960, "UR5e MuJoCo Simulator", nullptr, nullptr);
  if (!window) {
    fprintf(stderr, "[MuJoCoSimulator] glfwCreateWindow failed\n");
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // ── MuJoCo rendering ──────────────────────────────────────────────────────
  mjvCamera cam;
  mjv_defaultCamera(&cam);
  mjvOption opt;
  mjv_defaultOption(&opt);
  mjvScene scn;
  mjv_defaultScene(&scn);
  mjrContext con;
  mjr_defaultContext(&con);

  mjv_makeScene(model_, &scn, 2000);
  mjr_makeContext(model_, &con, mjFONTSCALE_150);

  cam.type = mjCAMERA_FREE;
  cam.distance = 2.5;
  cam.azimuth = 90.0;
  cam.elevation = -20.0;

  // ── Visualization-only mjData ─────────────────────────────────────────────
  mjData *vis_data = mj_makeData(model_);
  {
    std::lock_guard lock(state_mutex_);
    for (std::size_t i = 0; i < 6; ++i) {
      vis_data->qpos[joint_qpos_indices_[i]] = latest_positions_[i];
    }
  }
  mj_forward(model_, vis_data);

  // ── RTF profiler figure ───────────────────────────────────────────────────
  mjvFigure fig_profiler;
  mjv_defaultFigure(&fig_profiler);
  std::strncpy(fig_profiler.title, "RTF History",
               sizeof(fig_profiler.title) - 1);
  std::strncpy(fig_profiler.xlabel, "Frames (~60 Hz)",
               sizeof(fig_profiler.xlabel) - 1);
  std::strncpy(fig_profiler.linename[0], "RTF",
               sizeof(fig_profiler.linename[0]) - 1);
  fig_profiler.figurergba[0] = 0.08f;
  fig_profiler.figurergba[1] = 0.08f;
  fig_profiler.figurergba[2] = 0.08f;
  fig_profiler.figurergba[3] = 0.88f;
  fig_profiler.linergb[0][0] = 0.2f;
  fig_profiler.linergb[0][1] = 1.0f;
  fig_profiler.linergb[0][2] = 0.4f;
  fig_profiler.linewidth = 1.5f;
  fig_profiler.range[0][0] = 0;
  fig_profiler.range[0][1] = 200;
  fig_profiler.range[1][0] = 0;
  fig_profiler.range[1][1] = 30;
  fig_profiler.flg_extend = 1;

  // ── Viewer interaction state (shared with GLFW callbacks via user ptr) ────
  struct ViewerState {
    // MuJoCo handles (non-owning pointers, all local to ViewerLoop)
    mjvCamera *cam;
    mjvOption *opt;
    mjvScene *scn;
    const mjModel *model;
    mjData *vis_data;
    MuJoCoSimulator *sim;

    // Mouse tracking
    bool btn_left{false};
    bool btn_right{false};
    bool ctrl_held{false};
    double lastx{0.0};
    double lasty{0.0};

    // Perturbation state (local to viewer, transferred to sim via
    // UpdatePerturb)
    mjvPerturb pert{};

    // UI toggles
    bool show_profiler{false};
    bool show_help{false};
    bool show_solver{false}; // F4: solver stats overlay

    // RTF rolling buffer (200 samples at ~60 Hz ≈ 3.3 s window)
    const int kProfLen{200};
    float rtf_history[200]{};
    int rtf_head{0};
    int rtf_count{0};

    void push_rtf(float v) noexcept {
      rtf_history[rtf_head] = v;
      rtf_head = (rtf_head + 1) % kProfLen;
      if (rtf_count < kProfLen) {
        ++rtf_count;
      }
    }

    void update_figure(mjvFigure &fig) const noexcept {
      const int n = rtf_count;
      if (n == 0) {
        return;
      }
      const int oldest = (rtf_count < kProfLen) ? 0 : rtf_head;
      for (int i = 0; i < n; ++i) {
        const int idx = (oldest + i) % kProfLen;
        fig.linedata[0][i * 2] = static_cast<float>(i);
        fig.linedata[0][i * 2 + 1] = rtf_history[idx];
      }
      fig.linepnt[0] = n;
    }
  } vs{&cam, &opt, &scn, model_, vis_data, this};
  mjv_defaultPerturb(&vs.pert);

  glfwSetWindowUserPointer(window, &vs);

  // ── GLFW callbacks (captureless lambdas → implicit function pointers) ─────

  // Keyboard
  glfwSetKeyCallback(window, [](GLFWwindow *w, int key, int /*scan*/,
                                int action, int /*mods*/) noexcept {
    auto *s = static_cast<ViewerState *>(glfwGetWindowUserPointer(w));
    if (!s) {
      return;
    }

    // Track Ctrl key — needs both PRESS and RELEASE.
    if (key == GLFW_KEY_LEFT_CONTROL || key == GLFW_KEY_RIGHT_CONTROL) {
      s->ctrl_held = (action != GLFW_RELEASE);
      if (!s->ctrl_held && s->pert.active) {
        s->pert.active = 0;
        s->sim->ClearPerturb();
      }
      return;
    }

    if (action == GLFW_RELEASE) {
      return;
    }

    switch (key) {
    // ── Help overlay ──────────────────────────────────────────────────
    case GLFW_KEY_F1:
      s->show_help = !s->show_help;
      break;

    // ── Simulation controls ──────────────────────────────────────────
    case GLFW_KEY_SPACE:
      if (s->sim->IsPaused()) {
        s->sim->Resume();
      } else {
        s->sim->Pause();
      }
      break;

    case GLFW_KEY_EQUAL:
    case GLFW_KEY_KP_ADD: {
      const double cur = s->sim->GetMaxRtf();
      s->sim->SetMaxRtf(cur <= 0.0 ? 2.0 : cur * 2.0);
      break;
    }
    case GLFW_KEY_MINUS:
    case GLFW_KEY_KP_SUBTRACT: {
      const double cur = s->sim->GetMaxRtf();
      if (cur > 0.0 && cur <= 0.5) {
        s->sim->SetMaxRtf(0.0);
      } else if (cur > 0.5) {
        s->sim->SetMaxRtf(cur / 2.0);
      }
      break;
    }

    case GLFW_KEY_R:
      s->sim->RequestReset();
      break;

    // ── Physics toggles ───────────────────────────────────────────────
    case GLFW_KEY_G:
      s->sim->EnableGravity(!s->sim->IsGravityEnabled());
      fprintf(stdout, "[Viewer] Gravity %s\n",
              s->sim->IsGravityEnabled() ? "ON" : "OFF");
      break;

    // ── Visualisation toggles ─────────────────────────────────────────
    case GLFW_KEY_C:
      // Contact point markers
      s->opt->flags[mjVIS_CONTACTPOINT] ^= 1;
      break;

    case GLFW_KEY_F:
      // Contact force arrows
      s->opt->flags[mjVIS_CONTACTFORCE] ^= 1;
      break;

    case GLFW_KEY_V:
      // Toggle collision geometry group 0
      s->opt->geomgroup[0] ^= 1;
      break;

    case GLFW_KEY_T:
      // Transparency for all bodies
      s->opt->flags[mjVIS_TRANSPARENT] ^= 1;
      break;

    // ── Physics solver controls ───────────────────────────────────────
    case GLFW_KEY_I: {
      // Cycle integrator: Euler → RK4 → Implicit → ImplicitFast → Euler
      static constexpr int kIntegrators[] = {
          mjINT_EULER, mjINT_RK4, mjINT_IMPLICIT, mjINT_IMPLICITFAST};
      static constexpr const char *kIntNames[] = {"Euler", "RK4", "Implicit",
                                                  "ImplicitFast"};
      const int cur = s->sim->GetIntegrator();
      int next = 0;
      for (int k = 0; k < 4; ++k) {
        if (kIntegrators[k] == cur) {
          next = (k + 1) % 4;
          break;
        }
      }
      s->sim->SetIntegrator(kIntegrators[next]);
      fprintf(stdout, "[Viewer] Integrator → %s\n", kIntNames[next]);
      break;
    }

    case GLFW_KEY_S: {
      // Cycle solver: PGS → CG → Newton → PGS
      static constexpr int kSolvers[] = {mjSOL_PGS, mjSOL_CG, mjSOL_NEWTON};
      static constexpr const char *kSolNames[] = {"PGS", "CG", "Newton"};
      const int cur = s->sim->GetSolverType();
      int next = 0;
      for (int k = 0; k < 3; ++k) {
        if (kSolvers[k] == cur) {
          next = (k + 1) % 3;
          break;
        }
      }
      s->sim->SetSolverType(kSolvers[next]);
      fprintf(stdout, "[Viewer] Solver → %s\n", kSolNames[next]);
      break;
    }

    case GLFW_KEY_RIGHT_BRACKET:
      // ] → double solver iterations (max 1000)
      s->sim->SetSolverIterations(s->sim->GetSolverIterations() * 2);
      fprintf(stdout, "[Viewer] Solver iterations → %d\n",
              s->sim->GetSolverIterations());
      break;

    case GLFW_KEY_LEFT_BRACKET:
      // [ → halve solver iterations (min 1)
      s->sim->SetSolverIterations(s->sim->GetSolverIterations() / 2);
      fprintf(stdout, "[Viewer] Solver iterations → %d\n",
              s->sim->GetSolverIterations());
      break;

    case GLFW_KEY_N:
      // Toggle contact constraints
      s->sim->SetContactEnabled(!s->sim->IsContactEnabled());
      fprintf(stdout, "[Viewer] Contacts %s\n",
              s->sim->IsContactEnabled() ? "ENABLED" : "DISABLED");
      break;

    // ── Viewer controls ───────────────────────────────────────────────
    case GLFW_KEY_F3:
      s->show_profiler = !s->show_profiler;
      break;

    case GLFW_KEY_F4:
      s->show_solver = !s->show_solver;
      break;

    case GLFW_KEY_BACKSPACE:
      mjv_defaultOption(s->opt);
      break;

    case GLFW_KEY_ESCAPE:
      mjv_defaultCamera(s->cam);
      s->cam->distance = 2.5;
      s->cam->azimuth = 90.0;
      s->cam->elevation = -20.0;
      break;

    default:
      break;
    }
  });

  // Mouse buttons — orbit/pan or perturbation selection (Ctrl held)
  glfwSetMouseButtonCallback(window, [](GLFWwindow *w, int button, int action,
                                        int /*mods*/) noexcept {
    auto *s = static_cast<ViewerState *>(glfwGetWindowUserPointer(w));
    if (!s) {
      return;
    }
    const bool pressed = (action == GLFW_PRESS);

    if (s->ctrl_held && pressed && button == GLFW_MOUSE_BUTTON_LEFT) {
      // ── Ctrl+Left: body selection for perturbation ───────────────────
      double xp = 0.0, yp = 0.0;
      glfwGetCursorPos(w, &xp, &yp);
      int width = 0, height = 0;
      glfwGetWindowSize(w, &width, &height);
      if (width > 0 && height > 0) {
        mjtNum selpnt[3] = {};
        int selgeom = -1, selflexid = -1, selskin = -1;
        const int selobj = mjv_select(
            s->model, s->vis_data, s->opt,
            static_cast<mjtNum>(width) / static_cast<mjtNum>(height),
            static_cast<mjtNum>(xp) / static_cast<mjtNum>(width),
            static_cast<mjtNum>(height - yp) / static_cast<mjtNum>(height),
            s->scn, selpnt, &selgeom, &selflexid, &selskin);

        if (selobj > 0) { // > 0: a UR5e link (0 = world body)
          s->pert.select = selobj;
          mju_copy(s->pert.localpos, selpnt, 3);
          mjv_initPerturb(s->model, s->vis_data, s->scn, &s->pert);
          s->pert.active = mjPERT_TRANSLATE;
          s->sim->UpdatePerturb(s->pert);
          fprintf(stdout, "[Viewer] Perturbing body %d\n", selobj);
        }
      }
    } else if (!pressed && button == GLFW_MOUSE_BUTTON_LEFT && s->pert.active) {
      // ── Release left button: clear perturbation ───────────────────────
      s->pert.active = 0;
      s->sim->ClearPerturb();
    }

    if (!s->ctrl_held) {
      if (button == GLFW_MOUSE_BUTTON_LEFT) {
        s->btn_left = pressed;
      }
      if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        s->btn_right = pressed;
      }
    }
    if (pressed) {
      glfwGetCursorPos(w, &s->lastx, &s->lasty);
    }
  });

  // Cursor move — camera orbit/pan, or perturbation drag (Ctrl)
  glfwSetCursorPosCallback(
      window, [](GLFWwindow *w, double xpos, double ypos) noexcept {
        auto *s = static_cast<ViewerState *>(glfwGetWindowUserPointer(w));
        if (!s) {
          return;
        }

        int width = 0, height = 0;
        glfwGetWindowSize(w, &width, &height);
        if (width == 0 || height == 0) {
          s->lastx = xpos;
          s->lasty = ypos;
          return;
        }

        const double dx = xpos - s->lastx;
        const double dy = ypos - s->lasty;
        s->lastx = xpos;
        s->lasty = ypos;
        const double nx = dx / static_cast<double>(height);
        const double ny = dy / static_cast<double>(height);

        if (s->ctrl_held && s->pert.active) {
          // ── Ctrl+drag: move perturbation reference point ──────────────────
          mjv_movePerturb(s->model, s->vis_data, mjMOUSE_MOVE_V, nx, -ny,
                          s->scn, &s->pert);
          s->sim->UpdatePerturb(s->pert);
        } else if (s->btn_left) {
          // ── Left drag: orbit ───────────────────────────────────────────────
          mjv_moveCamera(s->model, mjMOUSE_ROTATE_V, nx, -ny, s->scn, s->cam);
        } else if (s->btn_right) {
          // ── Right drag: pan ────────────────────────────────────────────────
          mjv_moveCamera(s->model, mjMOUSE_MOVE_V, nx, -ny, s->scn, s->cam);
        }
      });

  // Scroll — zoom
  glfwSetScrollCallback(
      window, [](GLFWwindow *w, double /*xoffset*/, double yoffset) noexcept {
        auto *s = static_cast<ViewerState *>(glfwGetWindowUserPointer(w));
        if (!s) {
          return;
        }
        mjv_moveCamera(s->model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, s->scn,
                       s->cam);
      });

  fprintf(
      stdout,
      "[MuJoCoSimulator] Viewer ready — press F1 in the window for help\n"
      "  Simulation: Space=pause  +/-=speed  R=reset\n"
      "  Physics:    G=gravity  N=contacts\n"
      "  Solver:     I=integrator  S=solver  ]/[=iterations  F4=stats\n"
      "  Visualise:  C=cpoints  F=cforces  V=geoms  T=transp  F3=profiler\n"
      "  Camera:     Left-drag=orbit  Right-drag=pan  Scroll=zoom  Esc=reset\n"
      "  Perturb:    Ctrl+Left-drag=apply force to body\n");

  // ── Render loop ────────────────────────────────────────────────────────────
  while (!stop.stop_requested() && running_.load() &&
         !glfwWindowShouldClose(window)) {

    // Sync physics state to vis_data
    int ncon_snap = 0;
    {
      std::lock_guard lock(viz_mutex_);
      if (viz_dirty_) {
        std::memcpy(vis_data->qpos, viz_qpos_.data(),
                    static_cast<std::size_t>(model_->nq) * sizeof(double));
        viz_dirty_ = false;
      }
      ncon_snap = viz_ncon_;
    }
    mj_forward(model_, vis_data);

    // Sample RTF into rolling buffer
    const float cur_rtf =
        static_cast<float>(rtf_.load(std::memory_order_relaxed));
    vs.push_rtf(cur_rtf);
    vs.update_figure(fig_profiler);

    // ── Render ───────────────────────────────────────────────────────────────
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    mjrRect viewport{0, 0, width, height};

    // Pass &vs.pert so MuJoCo renders the perturbation grab indicator
    mjv_updateScene(model_, vis_data, &opt, &vs.pert, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // ── Status overlay (top-right)
    // ────────────────────────────────────────────
    {
      const double max_rtf_val =
          current_max_rtf_.load(std::memory_order_relaxed);
      const bool is_paused = paused_.load(std::memory_order_relaxed);
      const bool grav_on = gravity_enabled_.load(std::memory_order_relaxed);
      const bool perturbing = (vs.pert.active != 0);

      char limit_str[32];
      if (max_rtf_val > 0.0) {
        std::snprintf(limit_str, sizeof(limit_str), "%.1fx", max_rtf_val);
      } else {
        std::strncpy(limit_str, "unlimited", sizeof(limit_str) - 1);
        limit_str[sizeof(limit_str) - 1] = '\0';
      }

      // Solver stats snapshot
      const SolverStats ss = GetSolverStats();
      static constexpr const char *kIntNames[] = {"Euler", "RK4", "Implicit",
                                                  "ImplFast"};
      static constexpr const char *kSolNames[] = {"PGS", "CG", "Newton"};
      const int int_idx = std::max(0, std::min(GetIntegrator(), 3));
      const int sol_idx = std::max(0, std::min(GetSolverType(), 2));

      char labels[512], values[512];
      std::snprintf(
          labels, sizeof(labels),
          "Mode\nRTF\nLimit\nSim Time\nSteps\nContacts\nGravity\nStatus\n"
          "Integrator\nSolver\nIterations\nResidual");
      std::snprintf(values, sizeof(values),
                    "%s\n%.1fx\n%s\n%.2f s\n%lu\n%d/%s\n%s\n%s\n"
                    "%s\n%s\n%d/%d\n%.2e",
                    cfg_.mode == SimMode::kFreeRun ? "free_run" : "sync_step",
                    static_cast<double>(cur_rtf), limit_str,
                    sim_time_sec_.load(std::memory_order_relaxed),
                    static_cast<unsigned long>(step_count_.load()), ss.ncon,
                    contacts_enabled_.load(std::memory_order_relaxed) ? "on"
                                                                      : "OFF",
                    grav_on ? "ON" : "OFF",
                    is_paused ? "PAUSED" : (perturbing ? "perturb" : "running"),
                    kIntNames[int_idx], kSolNames[sol_idx], ss.iter,
                    GetSolverIterations(), ss.improvement);
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, labels, values,
                  &con);
    }

    // ── Help hint (bottom-left, always visible)
    // ───────────────────────────────
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, "F1: toggle help",
                nullptr, &con);

    // ── Detailed help overlay (top-left, F1 toggle)
    // ───────────────────────────
    if (vs.show_help) {
      const double max_rtf_now =
          current_max_rtf_.load(std::memory_order_relaxed);
      char rtf_str[32];
      if (max_rtf_now > 0.0) {
        std::snprintf(rtf_str, sizeof(rtf_str), "%.1fx", max_rtf_now);
      } else {
        std::strncpy(rtf_str, "unlimited", sizeof(rtf_str) - 1);
        rtf_str[sizeof(rtf_str) - 1] = '\0';
      }

      // Current solver labels for help overlay
      static constexpr const char *kHelpIntNames[] = {"Euler", "RK4",
                                                      "Implicit", "ImplFast"};
      static constexpr const char *kHelpSolNames[] = {"PGS", "CG", "Newton"};
      const int h_int = std::max(0, std::min(GetIntegrator(), 3));
      const int h_sol = std::max(0, std::min(GetSolverType(), 2));

      char help_keys[768], help_vals[768];
      std::snprintf(help_keys, sizeof(help_keys),
                    "── Simulation ──\n"
                    "Space\n"
                    "+  /  KP_ADD\n"
                    "-  /  KP_SUB\n"
                    "R\n"
                    "── Physics ──\n"
                    "G\n"
                    "N\n"
                    "── Solver ──\n"
                    "I\n"
                    "S\n"
                    "]  /  [\n"
                    "F4\n"
                    "── Visualisation ──\n"
                    "C\n"
                    "F\n"
                    "V\n"
                    "T\n"
                    "F3\n"
                    "Backspace\n"
                    "Escape\n"
                    "── Camera ──\n"
                    "Left drag\n"
                    "Right drag\n"
                    "Scroll\n"
                    "── Perturb ──\n"
                    "Ctrl + Left drag\n"
                    "── Help ──\n"
                    "F1");
      std::snprintf(
          help_vals, sizeof(help_vals),
          "\n"
          "Pause / Resume\n"
          "2x speed  [now %s]\n"
          "0.5x speed\n"
          "Reset to initial pose\n"
          "\n"
          "Toggle gravity  [%s]\n"
          "Toggle contacts  [%s]\n"
          "\n"
          "Cycle integrator  [%s]\n"
          "Cycle solver type  [%s]\n"
          "Increase / decrease iterations  [%d]\n"
          "Toggle solver stats overlay  [%s]\n"
          "\n"
          "Toggle contact points  [%s]\n"
          "Toggle contact forces  [%s]\n"
          "Toggle collision geoms  [%s]\n"
          "Toggle transparency  [%s]\n"
          "Toggle RTF profiler  [%s]\n"
          "Reset visualisation options\n"
          "Reset camera to default\n"
          "\n"
          "Orbit camera\n"
          "Pan camera\n"
          "Zoom in / out\n"
          "\n"
          "Apply spring force to body\n"
          "\n"
          "Hide help",
          rtf_str,
          gravity_enabled_.load(std::memory_order_relaxed) ? "ON" : "OFF",
          contacts_enabled_.load(std::memory_order_relaxed) ? "ON" : "OFF",
          kHelpIntNames[h_int], kHelpSolNames[h_sol], GetSolverIterations(),
          vs.show_solver ? "ON" : "OFF",
          (opt.flags[mjVIS_CONTACTPOINT]) ? "ON" : "OFF",
          (opt.flags[mjVIS_CONTACTFORCE]) ? "ON" : "OFF",
          (opt.geomgroup[0]) ? "ON" : "OFF",
          (opt.flags[mjVIS_TRANSPARENT]) ? "ON" : "OFF",
          vs.show_profiler ? "ON" : "OFF");

      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, help_keys, help_vals,
                  &con);
    }

    // ── Solver stats overlay (bottom-left extended, F4) ─────────────────────
    if (vs.show_solver) {
      const SolverStats ss2 = GetSolverStats();
      static constexpr const char *kOvIntNames[] = {"Euler", "RK4", "Implicit",
                                                    "ImplFast"};
      static constexpr const char *kOvSolNames[] = {"PGS", "CG", "Newton"};
      const int oi = std::max(0, std::min(GetIntegrator(), 3));
      const int os = std::max(0, std::min(GetSolverType(), 2));

      char sk[256], sv[256];
      std::snprintf(sk, sizeof(sk),
                    "Integrator\nSolver\nMax iter\nUsed iter\n"
                    "Improvement\nGradient\nContacts\nTimestep");
      std::snprintf(sv, sizeof(sv),
                    "%s\n%s\n%d\n%d\n"
                    "%.3e\n%.3e\n%d\n%.4f ms",
                    kOvIntNames[oi], kOvSolNames[os], GetSolverIterations(),
                    ss2.iter, ss2.improvement, ss2.gradient, ss2.ncon,
                    model_ ? static_cast<double>(model_->opt.timestep) * 1e3
                           : 0.0);
      mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, sk, sv, &con);
    }

    // ── Profiler (bottom-right, F3)
    // ───────────────────────────────────────────
    if (vs.show_profiler && vs.rtf_count > 0) {
      const int fw = width / 3;
      const int fh = height / 3;
      mjr_figure(mjrRect{width - fw, 0, fw, fh}, &fig_profiler, &con);
    }

    // Suppress unused variable warning for ncon_snap
    (void)ncon_snap;

    glfwSwapBuffers(window);
    glfwPollEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 Hz
  }

  mj_deleteData(vis_data);
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  glfwDestroyWindow(window);
  glfwTerminate();
  fprintf(stdout, "[MuJoCoSimulator] Viewer closed\n");

#else
  fprintf(
      stdout,
      "[MuJoCoSimulator] Viewer not available (MUJOCO_HAVE_GLFW not set)\n");
  (void)stop;
#endif
}

} // namespace ur5e_rt_controller
