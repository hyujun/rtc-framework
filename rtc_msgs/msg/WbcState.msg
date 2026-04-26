# WBC State — published by TSID-based whole-body controllers (e.g.
# DemoWbcController). Per-controller topic: /<config_key>/hand/wbc_state
# (500 Hz controller, ~50 Hz publish thread).
#
# Mirrors the role of GraspState for Force-PI grasp controllers, but the
# field set reflects WBC's TSID-based grasp algorithm: phase enum is the
# WBC FSM, force/contact comes from raw fingertip sensor parsing, and
# Force-PI per-finger fields (finger_s / finger_filtered_force /
# finger_force_error) are intentionally absent. BT coordinator subscribes
# to either grasp_state or wbc_state based on the active controller name.

std_msgs/Header header

# WbcPhase enum (8-state FSM, must match WbcPhase in
# ur5e_bringup/include/ur5e_bringup/controllers/demo_wbc_controller.hpp)
uint8 PHASE_IDLE      = 0
uint8 PHASE_APPROACH  = 1
uint8 PHASE_PRE_GRASP = 2
uint8 PHASE_CLOSURE   = 3
uint8 PHASE_HOLD      = 4
uint8 PHASE_RETREAT   = 5
uint8 PHASE_RELEASE   = 6
uint8 PHASE_FALLBACK  = 7
uint8 phase

# Per-fingertip raw sensor data (size = num_fingertips, typically 4)
string[]  fingertip_names    # e.g. ["thumb", "index", "middle", "ring"]
float32[] force_magnitude    # |F| per fingertip [N]
float32[] contact_flag       # contact probability (0.0–1.0)
float32[] displacement       # raw fingertip displacement [m]

# Aggregate
int32   num_active_contacts
float32 max_force            # max force across fingertips [N]
float32 grasp_target_force   # active target force [N] (from gains)
bool    grasp_detected       # num_active_contacts >= min_fingertips
int32   min_fingertips       # threshold for grasp_detected

# TSID solver diagnostics (populated by controller; informational only)
float32 tsid_solve_us        # last solver wall time per tick [us]
bool    tsid_solver_ok       # true if last QP converged
int32   qp_fail_count        # cumulative QP failures since activation
