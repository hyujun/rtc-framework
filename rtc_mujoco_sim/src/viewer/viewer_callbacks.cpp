// ── viewer_callbacks.cpp ───────────────────────────────────────────────────────
// GLFW keyboard / mouse callbacks for the MuJoCo viewer.
//
// All four callbacks are declared in viewer_state.hpp and registered by
// ViewerLoop() with glfwSet*Callback().  They are captureless — window state
// is retrieved via glfwGetWindowUserPointer() which returns a ViewerState*.
// ──────────────────────────────────────────────────────────────────────────────
#include "viewer/viewer_state.hpp"
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#ifdef MUJOCO_HAVE_GLFW

#include <algorithm>
#include <cstdio>

namespace rtc {

// ── Helper: ray-cast to select a body under the cursor ────────────────────────
static void SelectBody(GLFWwindow* w, ViewerState* s) noexcept {
  double xp = 0.0, yp = 0.0;
  glfwGetCursorPos(w, &xp, &yp);
  int width = 0, height = 0;
  glfwGetWindowSize(w, &width, &height);
  if (width <= 0 || height <= 0) { return; }

  mjtNum selpnt[3]  = {};
  int    selgeom    = -1;
  int    selflexid  = -1;
  int    selskin    = -1;
  const int selobj = mjv_select(
      s->model, s->vis_data, s->opt,
      static_cast<mjtNum>(width) / static_cast<mjtNum>(height),
      static_cast<mjtNum>(xp)    / static_cast<mjtNum>(width),
      static_cast<mjtNum>(height - yp) / static_cast<mjtNum>(height),
      s->scn, selpnt, &selgeom, &selflexid, &selskin);

  if (selobj > 0) {
    s->pert.select = selobj;
    
    // Convert global selection point to body local coordinates
    mjtNum pos[3];
    mju_sub3(pos, selpnt, s->vis_data->xpos + 3 * selobj);
    mju_mulMatTVec(s->pert.localpos, s->vis_data->xmat + 9 * selobj, pos, 3, 3);
    
    mjv_initPerturb(s->model, s->vis_data, s->scn, &s->pert);
    fprintf(stdout, "[Viewer] Selected body %d\n", selobj);
  }
}

// ── Keyboard callback ──────────────────────────────────────────────────────────
void OnKey(GLFWwindow* w, int key, int /*scan*/, int action, int /*mods*/) noexcept {
  auto* s = static_cast<ViewerState*>(glfwGetWindowUserPointer(w));
  if (!s) { return; }

  // ── Track modifier keys (PRESS and RELEASE both matter) ─────────────────────
  if (key == GLFW_KEY_LEFT_CONTROL || key == GLFW_KEY_RIGHT_CONTROL) {
    s->ctrl_held = (action != GLFW_RELEASE);
    if (!s->ctrl_held && s->pert.active) {
      s->pert.active = 0;
      s->sim->ClearPerturb();
    }
    return;
  }
  if (key == GLFW_KEY_LEFT_SHIFT || key == GLFW_KEY_RIGHT_SHIFT) {
    s->shift_held = (action != GLFW_RELEASE);
    return;
  }

  if (action == GLFW_RELEASE) { return; }

  switch (key) {

  // ── Help overlay (F1 cycles: off → page1 → page2 → off) ─────────────────
  case GLFW_KEY_F1:
    s->help_page = (s->help_page + 1) % 3;
    break;

  // ── Simulation controls ──────────────────────────────────────────────────
  case GLFW_KEY_SPACE:
    if (s->sim->IsPaused()) { s->sim->Resume(); } else { s->sim->Pause(); }
    break;

  case GLFW_KEY_RIGHT:
    // Single-step: advance exactly one physics step while paused.
    s->sim->StepOnce();
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
    if (cur > 0.0 && cur <= 0.5) { s->sim->SetMaxRtf(0.0); }
    else if (cur > 0.5)           { s->sim->SetMaxRtf(cur / 2.0); }
    break;
  }

  case GLFW_KEY_R:
    s->sim->RequestReset();
    break;

  // ── Camera mode (TAB cycles: Free → Tracking → Fixed[0..N-1] → Free) ────
  case GLFW_KEY_TAB: {
    const int ncam  = s->model->ncam;
    const int total = 2 + ncam;  // Free + Tracking + ncam Fixed cameras
    int cur = 0;
    if      (s->cam_mode == CameraMode::kFree)     { cur = 0; }
    else if (s->cam_mode == CameraMode::kTracking)  { cur = 1; }
    else                                             { cur = 2 + s->fixed_cam_idx; }
    const int next = (cur + 1) % total;

    if (next == 0) {
      s->cam_mode  = CameraMode::kFree;
      s->cam->type = mjCAMERA_FREE;
      fprintf(stdout, "[Viewer] Camera → Free\n");
    } else if (next == 1) {
      s->cam_mode          = CameraMode::kTracking;
      s->cam->type         = mjCAMERA_TRACKING;
      s->cam->trackbodyid  = s->track_body_id;
      fprintf(stdout, "[Viewer] Camera → Tracking body %d\n", s->track_body_id);
    } else {
      const int cam_id     = next - 2;
      s->cam_mode          = CameraMode::kFixed;
      s->fixed_cam_idx     = cam_id;
      s->cam->type         = mjCAMERA_FIXED;
      s->cam->fixedcamid   = cam_id;
      const char* name = mj_id2name(s->model, mjOBJ_CAMERA, cam_id);
      fprintf(stdout, "[Viewer] Camera → Fixed '%s'\n", name ? name : "(unnamed)");
    }
    break;
  }

  // ── Physics toggles ──────────────────────────────────────────────────────
  case GLFW_KEY_G:
    if (s->sim->IsGravityLockedByServo()) {
      fprintf(stdout, "[Viewer] Gravity locked by position servo (OFF)\n");
    } else {
      s->sim->EnableGravity(!s->sim->IsGravityEnabled());
      fprintf(stdout, "[Viewer] Gravity %s\n",
              s->sim->IsGravityEnabled() ? "ON" : "OFF");
    }
    break;

  case GLFW_KEY_N:
    s->sim->SetContactEnabled(!s->sim->IsContactEnabled());
    fprintf(stdout, "[Viewer] Contacts %s\n",
            s->sim->IsContactEnabled() ? "ENABLED" : "DISABLED");
    break;

  // ── Solver controls ──────────────────────────────────────────────────────
  case GLFW_KEY_I: {
    static constexpr int kInts[] = {
        mjINT_EULER, mjINT_RK4, mjINT_IMPLICIT, mjINT_IMPLICITFAST};
    static constexpr const char* kNames[] = {
        "Euler", "RK4", "Implicit", "ImplicitFast"};
    const int cur = s->sim->GetIntegrator();
    int next = 0;
    for (int k = 0; k < 4; ++k) {
      if (kInts[k] == cur) { next = (k + 1) % 4; break; }
    }
    s->sim->SetIntegrator(kInts[next]);
    fprintf(stdout, "[Viewer] Integrator → %s\n", kNames[next]);
    break;
  }
  case GLFW_KEY_S: {
    static constexpr int kSols[] = {mjSOL_PGS, mjSOL_CG, mjSOL_NEWTON};
    static constexpr const char* kNames[] = {"PGS", "CG", "Newton"};
    const int cur = s->sim->GetSolverType();
    int next = 0;
    for (int k = 0; k < 3; ++k) {
      if (kSols[k] == cur) { next = (k + 1) % 3; break; }
    }
    s->sim->SetSolverType(kSols[next]);
    fprintf(stdout, "[Viewer] Solver → %s\n", kNames[next]);
    break;
  }
  case GLFW_KEY_RIGHT_BRACKET:
    s->sim->SetSolverIterations(s->sim->GetSolverIterations() * 2);
    fprintf(stdout, "[Viewer] Solver iterations → %d\n",
            s->sim->GetSolverIterations());
    break;
  case GLFW_KEY_LEFT_BRACKET:
    s->sim->SetSolverIterations(s->sim->GetSolverIterations() / 2);
    fprintf(stdout, "[Viewer] Solver iterations → %d\n",
            s->sim->GetSolverIterations());
    break;

  // ── Visualisation flags ───────────────────────────────────────────────────
  case GLFW_KEY_C:  s->opt->flags[mjVIS_CONTACTPOINT] ^= 1;  break;
  case GLFW_KEY_F:  s->opt->flags[mjVIS_CONTACTFORCE]  ^= 1;  break;
  case GLFW_KEY_T:  s->opt->flags[mjVIS_TRANSPARENT]   ^= 1;  break;
  case GLFW_KEY_J:
    if (s->shift_held) {
      s->show_joint_frames = !s->show_joint_frames;
      fprintf(stdout, "[Viewer] Joint frames %s\n",
              s->show_joint_frames ? "ON" : "OFF");
    } else {
      s->opt->flags[mjVIS_JOINT] ^= 1;
    }
    break;
  case GLFW_KEY_B:
    s->show_link_frames = !s->show_link_frames;
    s->opt->frame = s->show_link_frames ? mjFRAME_BODY : mjFRAME_NONE;
    fprintf(stdout, "[Viewer] Link frames %s\n",
            s->show_link_frames ? "ON" : "OFF");
    break;
  case GLFW_KEY_U:  s->opt->flags[mjVIS_ACTUATOR]       ^= 1;  break;
  case GLFW_KEY_E:  s->opt->flags[mjVIS_INERTIA]        ^= 1;  break;
  case GLFW_KEY_W:  s->opt->flags[mjVIS_COM]             ^= 1;  break;
  case GLFW_KEY_L:  s->opt->flags[mjVIS_LIGHT]          ^= 1;  break;
  case GLFW_KEY_A:  s->opt->flags[mjVIS_TENDON]         ^= 1;  break;
  case GLFW_KEY_X:  s->opt->flags[mjVIS_CONVEXHULL]     ^= 1;  break;

  // Number keys 0-5: toggle geom groups
  case GLFW_KEY_0:  s->opt->geomgroup[0] ^= 1;  break;
  case GLFW_KEY_V:  s->opt->geomgroup[0] ^= 1;  break;  // legacy alias
  case GLFW_KEY_1:  s->opt->geomgroup[1] ^= 1;  break;
  case GLFW_KEY_2:  s->opt->geomgroup[2] ^= 1;  break;
  case GLFW_KEY_3:  s->opt->geomgroup[3] ^= 1;  break;
  case GLFW_KEY_4:  s->opt->geomgroup[4] ^= 1;  break;
  case GLFW_KEY_5:  s->opt->geomgroup[5] ^= 1;  break;

  // ── Rendering flags (scene-level) ─────────────────────────────────────────
  case GLFW_KEY_F5:  s->scn->flags[mjRND_WIREFRAME]  ^= 1;  break;
  case GLFW_KEY_F6:  s->scn->flags[mjRND_SHADOW]     ^= 1;  break;
  case GLFW_KEY_F7:  s->scn->flags[mjRND_SKYBOX]     ^= 1;  break;
  case GLFW_KEY_F8:  s->scn->flags[mjRND_REFLECTION] ^= 1;  break;

  // ── Overlay toggles ────────────────────────────────────────────────────────
  case GLFW_KEY_F3:   s->show_profiler   = !s->show_profiler;    break;
  case GLFW_KEY_F4:   s->show_solver     = !s->show_solver;      break;
  case GLFW_KEY_F9:   s->show_sensor     = !s->show_sensor;      break;
  case GLFW_KEY_F10:  s->show_model_info = !s->show_model_info;  break;

  // ── Screenshot ─────────────────────────────────────────────────────────────
  case GLFW_KEY_P:
    s->take_screenshot = true;
    break;

  // ── Reset options / camera ─────────────────────────────────────────────────
  case GLFW_KEY_BACKSPACE:
    mjv_defaultOption(s->opt);
    s->show_link_frames = false;
    s->show_joint_frames = false;
    break;

  case GLFW_KEY_ESCAPE:
    s->cam_mode  = CameraMode::kFree;
    s->cam->type = mjCAMERA_FREE;
    mjv_defaultCamera(s->cam);
    s->cam->distance  = 2.5;
    s->cam->azimuth   = 90.0;
    s->cam->elevation = -20.0;
    break;

  default:
    break;
  }
}

// ── Mouse button callback ──────────────────────────────────────────────────────
void OnMouseButton(GLFWwindow* w, int button, int action, int /*mods*/) noexcept {
  auto* s = static_cast<ViewerState*>(glfwGetWindowUserPointer(w));
  if (!s) { return; }

  const bool pressed = (action == GLFW_PRESS);
  if (pressed) { glfwGetCursorPos(w, &s->lastx, &s->lasty); }

  // ── Double-click (left): select body for perturbation ───────────────────
  if (button == GLFW_MOUSE_BUTTON_LEFT && pressed) {
    const double now = glfwGetTime();
    if (s->last_click_time >= 0.0 && (now - s->last_click_time) < 0.3) {
      SelectBody(w, s);
    }
    s->last_click_time = now;
  }

  // ── Ctrl+Left press: select body + begin torque perturbation ────────────
  if (s->ctrl_held && button == GLFW_MOUSE_BUTTON_LEFT && pressed) {
    SelectBody(w, s);
    if (s->pert.select > 0) {
      s->pert.active = mjPERT_ROTATE;
      s->sim->UpdatePerturb(s->pert);
    }
  }

  // ── Ctrl+Right press: begin force perturbation ──────────────────────────
  if (s->ctrl_held && button == GLFW_MOUSE_BUTTON_RIGHT && pressed) {
    if (s->pert.select <= 0) { SelectBody(w, s); }  // auto-select if none
    if (s->pert.select > 0) {
      s->pert.active = mjPERT_TRANSLATE;
      s->sim->UpdatePerturb(s->pert);
    }
  }

  // ── Release any button: deactivate perturbation ──────────────────────────
  if (!pressed && s->pert.active) {
    s->pert.active = 0;
    s->sim->ClearPerturb();
  }

  // ── Regular (non-Ctrl) button state tracking ─────────────────────────────
  if (!s->ctrl_held) {
    if (button == GLFW_MOUSE_BUTTON_LEFT)   { s->btn_left   = pressed; }
    if (button == GLFW_MOUSE_BUTTON_RIGHT)  { s->btn_right  = pressed; }
    if (button == GLFW_MOUSE_BUTTON_MIDDLE) { s->btn_middle = pressed; }
  }
}

// ── Cursor move callback ───────────────────────────────────────────────────────
void OnCursorPos(GLFWwindow* w, double xpos, double ypos) noexcept {
  auto* s = static_cast<ViewerState*>(glfwGetWindowUserPointer(w));
  if (!s) { return; }

  int width = 0, height = 0;
  glfwGetWindowSize(w, &width, &height);
  if (width == 0 || height == 0) { s->lastx = xpos; s->lasty = ypos; return; }

  const double dx = xpos - s->lastx;
  const double dy = ypos - s->lasty;
  s->lastx = xpos;
  s->lasty = ypos;
  const double nx = dx / static_cast<double>(height);
  const double ny = dy / static_cast<double>(height);

  if (s->ctrl_held && s->pert.active && s->pert.select > 0) {
    // ── Ctrl+drag: perturbation (torque or force depending on active flag) ──
    const int perturb_action =
        (s->pert.active == mjPERT_ROTATE)
            ? mjMOUSE_ROTATE_V                                // Ctrl+Left: torque
            : (s->shift_held ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V); // Ctrl+Right: force
    mjv_movePerturb(s->model, s->vis_data, perturb_action, nx, -ny, s->scn, &s->pert);
    s->sim->UpdatePerturb(s->pert);

  } else if (s->btn_left && !s->ctrl_held) {
    // ── Left drag: orbit (Shift → horizontal axis) ─────────────────────────
    const int cam_action = s->shift_held ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    mjv_moveCamera(s->model, cam_action, nx, -ny, s->scn, s->cam);

  } else if (s->btn_right && !s->ctrl_held) {
    // ── Right drag: pan (Shift → horizontal plane) ─────────────────────────
    const int cam_action = s->shift_held ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    mjv_moveCamera(s->model, cam_action, nx, -ny, s->scn, s->cam);

  } else if (s->btn_middle) {
    // ── Middle drag: zoom ───────────────────────────────────────────────────
    mjv_moveCamera(s->model, mjMOUSE_ZOOM, 0.0, -0.5 * ny, s->scn, s->cam);
  }
}

// ── Scroll callback ────────────────────────────────────────────────────────────
void OnScroll(GLFWwindow* w, double /*xoffset*/, double yoffset) noexcept {
  auto* s = static_cast<ViewerState*>(glfwGetWindowUserPointer(w));
  if (!s) { return; }
  mjv_moveCamera(s->model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, s->scn, s->cam);
}

}  // namespace rtc

#endif  // MUJOCO_HAVE_GLFW
