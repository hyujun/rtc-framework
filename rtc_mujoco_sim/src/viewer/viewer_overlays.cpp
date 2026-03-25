// ── viewer_overlays.cpp ────────────────────────────────────────────────────────
// All mjr_overlay / mjr_figure rendering functions for the MuJoCo viewer.
//
//   RenderStatusOverlay    — top-right: always visible status panel
//   RenderHelpOverlay      — top-left:  F1 detailed keyboard reference
//   RenderSolverOverlay    — bottom-left: F4 solver statistics
//   RenderSensorOverlay    — top-left:  F9 sensor values (exclusive with help)
//   RenderModelInfoOverlay — bottom-right: F10 model statistics
//   RenderRtfProfiler      — bottom-right corner graph: F3 RTF history
// ──────────────────────────────────────────────────────────────────────────────
#include "viewer/viewer_state.hpp"
#include "rtc_mujoco_sim/mujoco_simulator.hpp"

#ifdef MUJOCO_HAVE_GLFW

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace rtc {

// ── Shared name tables ─────────────────────────────────────────────────────────
static constexpr const char* kIntNames[] = {"Euler", "RK4", "Implicit", "ImplFast"};
static constexpr const char* kSolNames[] = {"PGS",   "CG",  "Newton"};
static constexpr const char* kCamNames[] = {"Free",  "Tracking", "Fixed"};

static int IntIdx(const ViewerState& vs) noexcept {
  return std::max(0, std::min(vs.sim->GetIntegrator(), 3));
}
static int SolIdx(const ViewerState& vs) noexcept {
  return std::max(0, std::min(vs.sim->GetSolverType(), 2));
}
static int CamIdx(const ViewerState& vs) noexcept {
  return static_cast<int>(vs.cam_mode);
}

// ── RenderStatusOverlay ────────────────────────────────────────────────────────
// Top-right: always-visible simulation status.
void RenderStatusOverlay(const ViewerState& vs, const mjrRect& vp,
                         float cur_rtf) noexcept {
  const double max_rtf_val = vs.sim->GetMaxRtf();
  const bool   is_paused   = vs.sim->IsPaused();
  const bool   grav_on     = vs.sim->IsGravityEnabled();
  const bool   grav_locked = vs.sim->IsGravityLockedByServo();
  const bool   perturbing  = (vs.pert.active != 0);

  char limit_str[32];
  if (max_rtf_val > 0.0) {
    std::snprintf(limit_str, sizeof(limit_str), "%.1fx", max_rtf_val);
  } else {
    std::strncpy(limit_str, "unlimited", sizeof(limit_str) - 1);
    limit_str[sizeof(limit_str) - 1] = '\0';
  }

  const auto ss  = vs.sim->GetSolverStats();
  const int  ii  = IntIdx(vs);
  const int  si  = SolIdx(vs);
  const int  ci  = CamIdx(vs);

  // Named fixed camera label (e.g. "Fixed:ee_cam")
  char cam_str[64];
  if (vs.cam_mode == CameraMode::kFixed) {
    const char* name = mj_id2name(vs.model, mjOBJ_CAMERA, vs.fixed_cam_idx);
    std::snprintf(cam_str, sizeof(cam_str), "Fixed:%s",
                  name ? name : "(unnamed)");
  } else {
    std::strncpy(cam_str, kCamNames[ci], sizeof(cam_str) - 1);
    cam_str[sizeof(cam_str) - 1] = '\0';
  }

  char labels[512], values[512];
  std::snprintf(labels, sizeof(labels),
      "Mode\nCamera\nRTF\nLimit\nSim Time\nSteps\nContacts\nGravity\nStatus\n"
      "Integrator\nSolver\nIterations\nResidual");
  std::snprintf(values, sizeof(values),
      "%s\n%s\n%.1fx\n%s\n%.2f s\n%lu\n%d/%s\n%s\n%s\n%s\n%s\n%d/%d\n%.2e",
      "sync",
      cam_str,
      static_cast<double>(cur_rtf), limit_str,
      vs.sim->SimTimeSec(),
      static_cast<unsigned long>(vs.sim->StepCount()),
      ss.ncon,
      vs.sim->IsContactEnabled() ? "on" : "OFF",
      grav_locked ? "OFF(lock)" : (grav_on ? "ON" : "OFF"),
      is_paused ? "PAUSED" : (perturbing ? "perturb" : "running"),
      kIntNames[ii], kSolNames[si],
      ss.iter, vs.sim->GetSolverIterations(),
      ss.improvement);

  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, vp, labels, values, vs.con);
}

// ── RenderHelpOverlay ──────────────────────────────────────────────────────────
// Page 1 (~20 lines): Simulation / Camera / Physics / Solver
// Page 2 (~22 lines): Visualisation / Rendering / Perturb / Other
// [ON/OFF] states omitted — already visible in the status overlay (top-right).
void RenderHelpOverlay(const ViewerState& vs, const mjrRect& vp, int page) noexcept {
  if (page == 1) {
    // ── Page 1: Simulation / Camera / Physics / Solver ──────────────────────
    const double max_rtf_now = vs.sim->GetMaxRtf();
    char rtf_str[24];
    if (max_rtf_now > 0.0) {
      std::snprintf(rtf_str, sizeof(rtf_str), "%.1fx", max_rtf_now);
    } else {
      std::strncpy(rtf_str, "unlimited", sizeof(rtf_str) - 1);
      rtf_str[sizeof(rtf_str) - 1] = '\0';
    }

    const int ii = IntIdx(vs);
    const int si = SolIdx(vs);

    char cam_lbl[48];
    if (vs.cam_mode == CameraMode::kFixed) {
      const char* name = mj_id2name(vs.model, mjOBJ_CAMERA, vs.fixed_cam_idx);
      std::snprintf(cam_lbl, sizeof(cam_lbl), "Fixed:%s",
                    name ? name : "(unnamed)");
    } else {
      std::strncpy(cam_lbl, kCamNames[static_cast<int>(vs.cam_mode)],
                   sizeof(cam_lbl) - 1);
      cam_lbl[sizeof(cam_lbl) - 1] = '\0';
    }

    char keys[800], vals[800];
    std::snprintf(keys, sizeof(keys),
        "Help 1/2  (F1=next)\n"
        "Space\n+/KP_ADD\n-/KP_SUB\nRight\nR\n"
        "TAB\nLeft drag\nShift+Left\nRight drag\nShift+Right\nScroll\nMiddle\nEsc\n"
        "G\nN\n"
        "I\nS\n]  /  [\nF4");
    std::snprintf(vals, sizeof(vals),
        "\n"
        "Pause / Resume\n"
        "2x speed [%s]\n"
        "0.5x speed\n"
        "Step once (paused)\n"
        "Reset pose\n"
        "Cycle camera [%s]\n"
        "Orbit\n"
        "Orbit horizontal\n"
        "Pan\n"
        "Pan horizontal\n"
        "Zoom\n"
        "Zoom (drag)\n"
        "Reset camera\n"
        "Gravity [%s]\n"
        "Contacts [%s]\n"
        "Integrator [%s]\n"
        "Solver [%s]\n"
        "Solver iter [%d]\n"
        "Solver stats",
        rtf_str, cam_lbl,
        vs.sim->IsGravityLockedByServo() ? "OFF(lock)"
            : (vs.sim->IsGravityEnabled() ? "ON" : "OFF"),
        vs.sim->IsContactEnabled() ? "ON" : "OFF",
        kIntNames[ii], kSolNames[si],
        vs.sim->GetSolverIterations());
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, vp, keys, vals, vs.con);

  } else {
    // ── Page 2: Visualisation / Rendering / Perturb / Other ─────────────────
    char keys[800], vals[800];
    std::snprintf(keys, sizeof(keys),
        "Help 2/2  (F1=close)\n"
        "C\nF\n0/V\n1..5\nJ\nU\nE\nW\nL\nA\nX\nT\nBackspace\n"
        "F5\nF6\nF7\nF8\n"
        "F3\nF9\nF10\n"
        "Dbl-click\nCtrl+L drag\nCtrl+R drag\nCtrl+Sh+R\n"
        "P");
    std::snprintf(vals, sizeof(vals),
        "\n"
        "Contact points [%s]\n"
        "Contact forces [%s]\n"
        "Geom group 0 [%s]\n"
        "Geom groups 1-5\n"
        "Joints [%s]\n"
        "Actuators [%s]\n"
        "Inertia [%s]\n"
        "CoM [%s]\n"
        "Lights [%s]\n"
        "Tendons [%s]\n"
        "Convex hulls [%s]\n"
        "Transparent [%s]\n"
        "Reset vis\n"
        "Wireframe [%s]\n"
        "Shadows [%s]\n"
        "Skybox [%s]\n"
        "Reflections [%s]\n"
        "RTF profiler\n"
        "Sensors\n"
        "Model info\n"
        "Select body\n"
        "Torque\n"
        "Force XZ\n"
        "Force XY\n"
        "Screenshot",
        vs.opt->flags[mjVIS_CONTACTPOINT] ? "ON" : "OFF",
        vs.opt->flags[mjVIS_CONTACTFORCE] ? "ON" : "OFF",
        vs.opt->geomgroup[0]              ? "ON" : "OFF",
        vs.opt->flags[mjVIS_JOINT]        ? "ON" : "OFF",
        vs.opt->flags[mjVIS_ACTUATOR]     ? "ON" : "OFF",
        vs.opt->flags[mjVIS_INERTIA]      ? "ON" : "OFF",
        vs.opt->flags[mjVIS_COM]          ? "ON" : "OFF",
        vs.opt->flags[mjVIS_LIGHT]        ? "ON" : "OFF",
        vs.opt->flags[mjVIS_TENDON]       ? "ON" : "OFF",
        vs.opt->flags[mjVIS_CONVEXHULL]   ? "ON" : "OFF",
        vs.opt->flags[mjVIS_TRANSPARENT]  ? "ON" : "OFF",
        vs.scn->flags[mjRND_WIREFRAME]    ? "ON" : "OFF",
        vs.scn->flags[mjRND_SHADOW]       ? "ON" : "OFF",
        vs.scn->flags[mjRND_SKYBOX]       ? "ON" : "OFF",
        vs.scn->flags[mjRND_REFLECTION]   ? "ON" : "OFF");
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, vp, keys, vals, vs.con);
  }
}

// ── RenderSolverOverlay ────────────────────────────────────────────────────────
// Bottom-left: F4 solver statistics panel.
void RenderSolverOverlay(const ViewerState& vs, const mjrRect& vp) noexcept {
  const auto ss = vs.sim->GetSolverStats();
  const int  ii = IntIdx(vs);
  const int  si = SolIdx(vs);

  char sk[256], sv[256];
  std::snprintf(sk, sizeof(sk),
      "Integrator\nSolver\nMax iter\nUsed iter\nImprovement\nGradient\nContacts\nTimestep");
  std::snprintf(sv, sizeof(sv),
      "%s\n%s\n%d\n%d\n%.3e\n%.3e\n%d\n%.4f ms",
      kIntNames[ii], kSolNames[si],
      vs.sim->GetSolverIterations(), ss.iter,
      ss.improvement, ss.gradient, ss.ncon,
      vs.model ? static_cast<double>(vs.model->opt.timestep) * 1e3 : 0.0);

  mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, vp, sk, sv, vs.con);
}

// ── RenderSensorOverlay ────────────────────────────────────────────────────────
// Top-left (exclusive with help): F9 sensor values from vis_data->sensordata.
// mj_forward() is called before rendering, so sensor data is up to date.
void RenderSensorOverlay(const ViewerState& vs, const mjrRect& vp) noexcept {
  if (!vs.model || !vs.vis_data) { return; }
  const int nsensor = vs.model->nsensor;
  if (nsensor <= 0) {
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, vp,
                "Sensors", "none in model", vs.con);
    return;
  }

  // Show up to 24 sensors to avoid overlay overflow
  const int show_n = std::min(nsensor, 24);
  char labels[1024] = {};
  char values[1024] = {};
  int  lpos = 0;
  int  vpos = 0;

  lpos += std::snprintf(labels + lpos, sizeof(labels) - static_cast<size_t>(lpos),
                        "── Sensors (%d) ──", nsensor);
  vpos += std::snprintf(values + vpos, sizeof(values) - static_cast<size_t>(vpos), " ");

  for (int i = 0; i < show_n && lpos < 1000 && vpos < 1000; ++i) {
    const char* name = mj_id2name(vs.model, mjOBJ_SENSOR, i);
    const int   adr  = vs.model->sensor_adr[i];
    const int   dim  = vs.model->sensor_dim[i];

    lpos += std::snprintf(labels + lpos,
                          sizeof(labels) - static_cast<size_t>(lpos),
                          "\n%s", name ? name : "(?)");

    if (dim == 1) {
      vpos += std::snprintf(values + vpos,
                            sizeof(values) - static_cast<size_t>(vpos),
                            "\n%.4g", vs.vis_data->sensordata[adr]);
    } else {
      // Multi-dim: show first value + dimension
      vpos += std::snprintf(values + vpos,
                            sizeof(values) - static_cast<size_t>(vpos),
                            "\n%.4g  [dim %d]",
                            vs.vis_data->sensordata[adr], dim);
    }
  }
  if (nsensor > show_n) {
    lpos += std::snprintf(labels + lpos,
                          sizeof(labels) - static_cast<size_t>(lpos),
                          "\n... +%d more", nsensor - show_n);
    std::snprintf(values + vpos,
                  sizeof(values) - static_cast<size_t>(vpos), "\n");
  }

  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, vp, labels, values, vs.con);
}

// ── RenderModelInfoOverlay ────────────────────────────────────────────────────
// Bottom-right: F10 static model statistics.
void RenderModelInfoOverlay(const ViewerState& vs, const mjrRect& vp) noexcept {
  if (!vs.model) { return; }
  char labels[256], values[256];
  std::snprintf(labels, sizeof(labels),
      "── Model Info ──\nnBody\nnGeom\nnJoint\nnActuator\nnSensor\nnSite\nnCamera\nnTimestep");
  std::snprintf(values, sizeof(values),
      "\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%.4f ms",
      vs.model->nbody,
      vs.model->ngeom,
      vs.model->njnt,
      vs.model->nu,
      vs.model->nsensor,
      vs.model->nsite,
      vs.model->ncam,
      static_cast<double>(vs.model->opt.timestep) * 1e3);
  mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMRIGHT, vp, labels, values, vs.con);
}

// ── RenderRtfProfiler ─────────────────────────────────────────────────────────
// Bottom-right corner (F3): RTF time-series graph using mjr_figure.
void RenderRtfProfiler(const ViewerState& vs, const mjrRect& vp) noexcept {
  if (!vs.show_profiler || vs.rtf_count == 0 || !vs.fig_profiler) { return; }
  const int fw = vp.width  / 3;
  const int fh = vp.height / 3;
  mjr_figure(mjrRect{vp.width - fw, 0, fw, fh}, vs.fig_profiler, vs.con);
}

}  // namespace rtc

#endif  // MUJOCO_HAVE_GLFW
