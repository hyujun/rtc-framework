# Catching State — published by the dynamic-catching controller
# (DemoCatchingController). Per-controller topic: /<config_key>/catching_state
# (500 Hz controller — `control_rate` in both shipped profiles — and a ~50 Hz
# publish thread).
#
# Same ownership shape as WbcState / GraspState: the controller owns a
# SeqLock<CatchingDiagLogPod>, the CM publish thread loads it and fills this
# message. No PublishRole was added (dynamic_catching E-11).
#
# THE FIELD SET IS FROZEN HERE FOR S5 THROUGH S9 (dynamic_catching D-20). The
# stages after S5 fill values into fields that already exist rather than
# widening the message, because a consumer (GUI, rosbag replay, the offline
# evaluation scripts) that gains a column every stage cannot read a recording
# made one stage earlier. Fields no stage has implemented yet are published
# with their `*_valid` companion false — which is the same thing a tick that
# did not compute them publishes, and that is deliberate: "not yet built" and
# "not computed this tick" are both "do not read this number".
#
# EVERY TICK PUBLISHES A BODY (PROC-7). The controller Stores on every tick,
# including E-STOP, stale input, generation mismatch, short horizon, no plan
# and abort ticks. A tick that did not compute a block clears it rather than
# leaving the previous tick's values in place, so a frozen number in a
# recording means the controller really did recompute the same value — never
# that the publisher skipped a tick.

std_msgs/Header header

# ── Supervisor (L7 §4, S1.8 transition table) ───────────────────────────────
# Mode values mirror rtc::catching::Mode; reasons mirror rtc::catching::Reason.
# Both are dense, so a value outside the listed range is a version skew rather
# than a state.
uint8 MODE_IDLE       = 0
uint8 MODE_ARMED      = 1
uint8 MODE_TRACKING   = 2
uint8 MODE_APPROACH   = 3
uint8 MODE_COMMITTED  = 4
uint8 MODE_CLOSING    = 5
uint8 MODE_DECEL      = 6
uint8 MODE_HOLD       = 7
uint8 MODE_RETREAT    = 8
uint8 MODE_ABORT_SAFE = 9
uint8 MODE_FAULT      = 10
uint8 mode

uint8 REASON_NONE                 = 0
uint8 REASON_BALL_STALE           = 1
uint8 REASON_BALL_STALE_COMMITTED = 2
uint8 REASON_BALL_STALE_LONG      = 3
uint8 REASON_TRACK_CHANGED        = 4
uint8 REASON_HORIZON_EXTRAP       = 5
uint8 REASON_PRED_INCONSISTENT    = 6
uint8 REASON_NO_CATCHABLE_PLAN    = 7
uint8 REASON_PLAN_INVALID         = 8
uint8 REASON_QP_FAILED            = 9
uint8 REASON_REF_SATURATED        = 10
uint8 REASON_JOINT_CONFLICT       = 11
uint8 REASON_TRACK_ERR            = 12
uint8 REASON_ABORT_ESCALATED      = 13
uint8 REASON_ESTOP                = 14
uint8 REASON_FAULT_RESET          = 15
uint8 REASON_SPEED_SCALING        = 16
uint8 REASON_CLOCK_UNHEALTHY      = 17
uint8 REASON_PARAMS_TBD           = 18
uint8 REASON_HAND_TIMEOUT         = 19
uint8 REASON_TIP_STALE            = 20
uint8 reason

# Per-trial verdict (L7 §4.7). Filled from S8; MODE/REASON say what the
# controller is doing, this says how the last attempt ended.
uint8 OUTCOME_NONE         = 0
uint8 OUTCOME_CAPTURED     = 1
uint8 OUTCOME_MISSED       = 2
uint8 OUTCOME_UNDETERMINED = 3
uint8 OUTCOME_ABORTED      = 4
uint8 outcome

# ── Controller-level state ──────────────────────────────────────────────────
# `armed` is the latch AS THE RT TICK SEES IT, not the `catching.enable`
# parameter: the tick lowers it on E-STOP and on a fault latch (P-1 (c)), so
# the two disagree exactly when something disarmed the controller out from
# under the operator.
bool armed
bool estop_active
bool fault_latched
# The profile passed the consumed-subset gate at configure (G0-C). False means
# no amount of arming will start a trial.
bool armable
# The tracking law is wired (model + catch frame + CLIK + L4 reference). False
# means the controller holds the arm by design, not by failure.
bool law_enabled
# A real-arm configuration (no proof every claimed device is the simulator) —
# the axis the L0 validator used. Provisional values block only on this axis.
bool real_arm_config

uint64 tick             # ControllerState::iteration of the tick this body is from
float64 t_relative_s    # session-relative seconds, the CSV's join key
float64 t_arm_s         # T_arm: the lead the reference is evaluated at [s]

# ── Vision input lane (L1) ──────────────────────────────────────────────────
# What the RT tick concluded about the snapshot it was holding. These are the
# TICK's view, so they are meaningful on every tick including the ones that
# refused to use the snapshot.
bool input_valid        # a snapshot exists for this activation
bool input_stale        # older than io.t_stale on the STEADY RECEIVE axis (D-2)
bool input_expired      # prediction ends before the instant this command lands
bool input_new          # a snapshot the previous tick had not consumed
int32 input_n           # samples in the snapshot
uint64 input_generation            # vision track epoch (L1 §4.4, D-4)
uint64 input_snapshot_sequence     # provenance token (D-22)
uint64 input_activation_generation # provenance token (D-22)
float64 input_age_s     # age on the steady receive axis — the only staleness axis
float64 input_horizon_s # prediction horizon carried by the snapshot

# Ingress counters, updated on MESSAGE ARRIVAL rather than per tick. A lane
# being refused and a lane being silent look identical from the RT side, so
# these are the first thing to read when everything is stale.
uint64 input_accept_count
# CloudReject histogram, in the enum's own order. `input_reject_names` is
# filled once at configure with the matching names so a stored bag decodes
# without that build's header.
uint64[] input_reject_counts
string[] input_reject_names
uint64 input_layout_rebuilds  # PointCloud2 field layout changes followed by name
float64 input_origin_delay_s  # recv_wall − stamp; published, NEVER judged on (D-2)
float64 input_jump_m          # L1 §4.4 J vs the previous prediction; < 0 = not compared

# ── Plan (L3, filled by the planner from S6; by the S5 oracle before that) ──
bool plan_valid
uint32 plan_id
float64 plan_t_c_s       # seconds from now to the catch instant (negative = past)
float64 plan_age_s       # age of the plan on the steady axis
float64[3] plan_p_c      # catch point [m], world
float64[3] plan_a_d      # approach axis (unit), world
float64[3] plan_v_c      # predicted ball velocity at t_c [m/s]
float64 plan_gamma_f     # terminal γ
float64 plan_w5          # catchability manipulability, gate definition (D-18)
float64 plan_w6          # 6-row variant, always recorded alongside (C-3)
float64 plan_sigma_c     # catch-point uncertainty [m]
float64 plan_score
# Why the planner rejected candidates (rtc::catching::PlanReason). Dense.
uint8 PLAN_REASON_NONE              = 0
uint8 PLAN_REASON_UNCERTAINTY       = 1
uint8 PLAN_REASON_IK_FAILED         = 2
uint8 PLAN_REASON_MANIPULABILITY    = 3
uint8 PLAN_REASON_REACH_TIME        = 4
uint8 PLAN_REASON_LIMITS_INVALID    = 5
uint8 PLAN_REASON_GAMMA_WINDOW      = 6
uint8 PLAN_REASON_STOPPING_DISTANCE = 7
uint8 PLAN_REASON_ROLLOUT           = 8
uint8 PLAN_REASON_ERROR_BUDGET      = 9
uint8 PLAN_REASON_IMPULSE           = 10
uint8 PLAN_REASON_HORIZON_SHORT     = 11
uint8 PLAN_REASON_BUDGET_EXCEEDED   = 12
uint8 PLAN_REASON_INPUT_NON_FINITE  = 13
uint8 plan_reason

# ── Reference (L4 soft-catch translational DS) ──────────────────────────────
# `ref_xdd` is the REALISED acceleration and `ref_u_des` the pre-saturation
# demand. Both are carried because a saturated interval is uninterpretable
# from either one alone.
bool ref_valid
bool ref_saturated
float64[3] ref_x
float64[3] ref_xd
float64[3] ref_xdd
float64[3] ref_u_des
float64[3] ref_e         # tracking error of the DS, one tick old
float64[3] ref_ed
float64 ref_gamma
float64 ref_gamma_d
float64 ref_gamma_dd

# ── Joint command (L5, CLIK) ────────────────────────────────────────────────
bool clik_ran            # the law reached the solve this tick
bool clik_converged
bool clik_bound_conflict   # the acceleration box overrode velocity ∩ position
bool clik_command_mismatch # evaluate_at_command: cache.q (arm) ≠ previous q_ref
int32 clik_status          # ProxQP QPSolverOutput (0 = SOLVED), −1 = not solved
int32 clik_iterations
float64 clik_solve_us      # wall time of the solve [µs] — the G5-C budget's input
uint64 clik_conflict_mask  # bit i = velocity index i conflicted
int32 qp_fail_streak       # consecutive failures; supervisor.n_qp of them latch a fault

# ── Tracking ────────────────────────────────────────────────────────────────
# ‖q_meas − q_cmd‖ is the ONLY place the measured arm state enters the loop
# (D-6 keeps it out of CLIK), which makes it the only watchdog that notices
# the arm is not where it was told to be.
float64 track_err_rad
float64[] q_cmd          # commanded arm configuration, DEVICE order
float64[] q_meas         # measured arm configuration, same order
string[] arm_joint_names # names the two arrays are in, stamped once at configure
bool abort_stopped       # the QP-independent joint-space stop has brought the arm to rest

# ── Hand (L6, filled from S7) ───────────────────────────────────────────────
bool hand_phase_valid    # false until the sequencer owns the hand (S7.1)
uint8 HAND_PHASE_OPEN     = 0
uint8 HAND_PHASE_PRESHAPE = 1
uint8 HAND_PHASE_CLOSE    = 2
uint8 HAND_PHASE_HOLD     = 3
uint8 HAND_PHASE_RELEASE  = 4
uint8 hand_phase
float64 hand_rho         # closure fraction ρ(t) ∈ [0, 1]
bool hand_timeout        # t_cmd was missed (L6 §4)

# ── Fingertip sensors (D-24) ────────────────────────────────────────────────
# `tip_fresh` requires BOTH the backend's own validity flag AND this
# controller's deadline — a value being valid is not the same as a consumer's
# time limit being met, and the catching deadline is not the backend's.
string[] tip_names
float64[] tip_force      # |F| per fingertip [N], finger-on-object sign (L7 §4.4)
bool[] tip_contact
bool[] tip_fresh
float64[] tip_age_s      # age on the steady receive axis; < 0 = never received
