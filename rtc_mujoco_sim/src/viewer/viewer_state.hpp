// ── viewer_state.hpp ───────────────────────────────────────────────────────────
// ViewerState: shared state for all viewer subsystems (loop, callbacks, overlays).
// Internal header — not part of the public MuJoCoSimulator API.
// Also declares the free functions defined in viewer_callbacks.cpp /
// viewer_overlays.cpp so that viewer_loop.cpp can call them.
// ──────────────────────────────────────────────────────────────────────────────
#ifndef RTC_MUJOCO_SIM_VIEWER_VIEWER_STATE_HPP_
#define RTC_MUJOCO_SIM_VIEWER_VIEWER_STATE_HPP_

#ifdef MUJOCO_HAVE_GLFW
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

namespace rtc {

class MuJoCoSimulator;

// Camera mode for TAB cycling
enum class CameraMode { kFree, kTracking, kFixed };

// ── ViewerState ───────────────────────────────────────────────────────────────
// All GLFW callbacks access this struct via glfwGetWindowUserPointer().
struct ViewerState {
  // ── MuJoCo handles (non-owning pointers, all set before first use) ─────────
  mjvCamera     *cam{nullptr};
  mjvOption     *opt{nullptr};
  mjvScene      *scn{nullptr};
  mjrContext    *con{nullptr};      // needed for overlays + screenshot
  const mjModel *model{nullptr};
  mjData        *vis_data{nullptr};
  mjvFigure     *fig_profiler{nullptr};
  MuJoCoSimulator *sim{nullptr};

  // ── Mouse tracking ─────────────────────────────────────────────────────────
  bool   btn_left{false};
  bool   btn_right{false};
  bool   btn_middle{false};
  bool   ctrl_held{false};
  bool   shift_held{false};
  double lastx{0.0};
  double lasty{0.0};
  double last_click_time{-1.0};   // glfwGetTime() of last left press (double-click)

  // ── Perturbation state (local copy; synced to sim via UpdatePerturb) ───────
  mjvPerturb pert{};

  // ── UI toggles ─────────────────────────────────────────────────────────────
  // help_page: 0=off  1=page1(Sim/Camera/Physics/Solver)  2=page2(Vis/Render/Perturb)
  // F1 cycles 0→1→2→0.
  int  help_page{0};
  bool show_profiler{false};
  bool show_solver{false};
  bool show_sensor{false};        // F9: sensor values overlay
  bool show_model_info{false};    // F10: model statistics overlay
  bool take_screenshot{false};    // P:  write PPM to logging_data/

  // ── Frame visualization toggles ───────────────────────────────────────────
  bool show_link_frames{false};   // B: body/link coordinate frames (opt->frame)
  bool show_joint_frames{false};  // Shift+J: custom joint coordinate frames

  // ── Camera mode ────────────────────────────────────────────────────────────
  CameraMode cam_mode{CameraMode::kFree};
  int        fixed_cam_idx{0};  // index into model cameras when kFixed
  int        track_body_id{1};  // body to follow when kTracking

  // ── RTF rolling buffer (~200 frames at 60 Hz ≈ 3.3 s window) ─────────────
  static constexpr int kProfLen = 200;
  float rtf_history[200]{};
  int   rtf_head{0};
  int   rtf_count{0};

  void push_rtf(float v) noexcept {
    rtf_history[rtf_head] = v;
    rtf_head = (rtf_head + 1) % kProfLen;
    if (rtf_count < kProfLen) { ++rtf_count; }
  }

  void update_figure() noexcept {
    if (!fig_profiler || rtf_count == 0) { return; }
    const int oldest = (rtf_count < kProfLen) ? 0 : rtf_head;
    for (int i = 0; i < rtf_count; ++i) {
      const int idx = (oldest + i) % kProfLen;
      fig_profiler->linedata[0][i * 2]     = static_cast<float>(i);
      fig_profiler->linedata[0][i * 2 + 1] = rtf_history[idx];
    }
    fig_profiler->linepnt[0] = rtf_count;
  }
};

// ── Overlay functions (defined in viewer_overlays.cpp) ────────────────────────
void RenderStatusOverlay   (const ViewerState& vs, const mjrRect& vp, float cur_rtf) noexcept;
void RenderHelpOverlay     (const ViewerState& vs, const mjrRect& vp, int page) noexcept;
void RenderSolverOverlay   (const ViewerState& vs, const mjrRect& vp) noexcept;
void RenderSensorOverlay   (const ViewerState& vs, const mjrRect& vp) noexcept;
void RenderModelInfoOverlay(const ViewerState& vs, const mjrRect& vp) noexcept;
void RenderRtfProfiler     (const ViewerState& vs, const mjrRect& vp) noexcept;

// ── Frame visualization (defined in viewer_overlays.cpp) ─────────────────────
void AddJointFrameGeoms(ViewerState& vs) noexcept;

// ── Callback functions (defined in viewer_callbacks.cpp) ─────────────────────
void OnKey        (GLFWwindow* w, int key, int scan, int action, int mods) noexcept;
void OnMouseButton(GLFWwindow* w, int button, int action, int mods) noexcept;
void OnCursorPos  (GLFWwindow* w, double xpos, double ypos) noexcept;
void OnScroll     (GLFWwindow* w, double xoffset, double yoffset) noexcept;

}  // namespace rtc

#endif  // MUJOCO_HAVE_GLFW
#endif  // RTC_MUJOCO_SIM_VIEWER_VIEWER_STATE_HPP_
