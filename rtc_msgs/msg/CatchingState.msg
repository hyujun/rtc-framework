# Catching State — published by the dynamic-catching controller
# (DemoCatchingController). Per-controller topic: /<config_key>/catching_state
# (500 Hz controller — `control_rate` in both shipped profiles — and a ~50 Hz
# publish thread).
#
# Same ownership shape as WbcState / GraspState: the controller owns a
# SeqLock<CatchingDiagLogPod>, the CM publish thread loads it and fills this
# message. No PublishRole was added.
#
# THE FIELD SET IS FROZEN. New work fills values into fields that already
# exist rather than widening the message, because a consumer (GUI, rosbag
# replay, the offline evaluation scripts) that gains a column with every
# change cannot read an older recording. A field a configuration does not
# compute is published with its `*_valid` companion false — the same thing a
# tick that did not compute it publishes, and that is deliberate: "not built"
# and "not computed this tick" are both "do not read this number".
#
# EVERY TICK PUBLISHES A BODY (PROC-7). The controller Stores on every tick,
# including E-STOP, stale input, generation mismatch, short horizon, no plan
# and abort ticks. A tick that did not compute a block clears it rather than
# leaving the previous tick's values in place, so a frozen number in a
# recording means the controller really did recompute the same value — never
# that the publisher skipped a tick. The publish thread samples the latest
# tick, so consecutive messages are not consecutive ticks.
#
# CONVENTIONS. SI units. Every vector is in the MODEL WORLD frame: the root
# frame of the controller's robot model, into which each vision prediction is
# mapped once on receipt. `now` is the tick's steady-clock instant;
# now_lead = now + T_arm (`t_arm_s`). Ball instants (t_c, prediction samples)
# are on the steady axis, converted once from the prediction's stamp on
# receipt. Ages are measured on the STEADY RECEIVE axis (now − receive
# instant); a negative age means "never received".

# header.stamp: wall-clock (CLOCK_REALTIME) instant the CM took the publish
# snapshot. Not the tick's instant — `tick` / `t_relative_s` identify the tick.
std_msgs/Header header

# ── Supervisor ──────────────────────────────────────────────────────────────
# Mode values mirror rtc::catching::Mode; reasons mirror rtc::catching::Reason.
# Both are dense, so a value outside the listed range is a version skew rather
# than a state.
#
# mode — the supervisor state after this tick's transition:
#   IDLE       not armed (holds the activation pose), or armed and homing to
#              the wait pose
#   ARMED      at the wait pose with the hand at q_pre, waiting for a usable
#              prediction
#   TRACKING   a usable prediction; waiting for a plan
#   APPROACH   following a plan; a replacement plan may still be taken
#   COMMITTED  plan frozen (t_c − now ≤ T_freeze); no plan change; waiting for
#              the hand close instant t_cmd
#   CLOSING    hand close commanded
#   DECEL      now_lead ≥ t_c: decelerating after the catch, contact judged
#   HOLD       arm stopped; held for T_hold, then the attempt is judged
#   RETREAT    stop, joint-space return to the wait pose, hand released there
#   ABORT_SAFE joint-space stop to zero commanded velocity at the per-joint
#              acceleration limit, whatever the cause
#   FAULT      fault latched: stopped and disarmed; left only by a fault reset
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

# reason — the reason this tick evaluated, whether or not it moved the mode
# (NONE = no failure reason; the nominal advance of each mode is keyed on it):
#   BALL_STALE            before the freeze: the prediction is stale (receive
#                         age > io.t_stale, or from another activation) or
#                         cannot be sampled at now_lead
#   BALL_STALE_COMMITTED  stale after the freeze; the frozen plan continues
#                         (recorded only)
#   BALL_STALE_LONG       stale after the freeze for longer than
#                         io.t_stale + supervisor.stale_committed_max_s
#   TRACK_CHANGED         the vision track epoch (`input_generation`) changed
#   HORIZON_EXTRAP        now_lead is past the prediction's last sample
#                         (recorded only after the freeze)
#   PRED_INCONSISTENT     prediction consistency over threshold; never emitted
#                         (no producer of the metric)
#   NO_CATCHABLE_PLAN     no plan to adopt this tick (TRACKING stays)
#   PLAN_INVALID          the followed plan cannot be applied
#   QP_FAILED             the CLIK QP failed
#   REF_SATURATED         the soft-catch reference saturated for
#                         supervisor.sat_ticks ticks in a row, or produced no
#                         valid reference
#   TRACK_ERR             track_err_rad > supervisor.track_err_abort
#   ABORT_ESCALATED       the fault latch went up (supervisor.n_qp trials in a
#                         row ended by a CLIK failure, or a stop / return
#                         motion passed its deadline)
#   ESTOP                 E-STOP raised or released
#   FAULT_RESET           a fault reset was accepted
#   SPEED_SCALING         arm speed scaling ≠ 1; never emitted (no signal)
#   CLOCK_UNHEALTHY       clock synchronisation over threshold; never emitted
#                         (no signal)
#   PARAMS_TBD            parameter validation failed, readiness was lost, or
#                         a value the law needs is missing
#   HAND_TIMEOUT          the hand close timed out (recorded), or in RETREAT
#                         the released hand did not settle at q_pre in time
#   TIP_STALE             a fingertip sensor went stale from COMMITTED on
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
# 11 is unassigned: its reason was removed (#755) and the numbers after it stay.
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

# outcome — how the last attempt ended (MODE/REASON say what the controller is
# doing now). Set when HOLD ends or an attempt is cut short, and kept until
# the next one; mirrors rtc::catching::Outcome:
#   NONE          no attempt judged since activation or the last E-STOP release
#   CAPTURED      at the end of HOLD, ≥ supervisor.contact.m_min fingertips
#                 confirmed contact inside the contact window
#                 [t_cmd, t_c + T_confirm] and still do, or the hand joints
#                 stalled on an object for the persistence time
#   MISSED        judgeable, and neither witness holds
#   UNDETERMINED  the fingertip lane could not judge (fewer than m_min tips,
#                 a bias learned from too few samples, or a stale tip inside
#                 the window)
#   ABORTED       the attempt ended before HOLD's verdict (abort, E-STOP)
uint8 OUTCOME_NONE         = 0
uint8 OUTCOME_CAPTURED     = 1
uint8 OUTCOME_MISSED       = 2
uint8 OUTCOME_UNDETERMINED = 3
uint8 OUTCOME_ABORTED      = 4
uint8 outcome

# ── Controller-level state ──────────────────────────────────────────────────
# `armed` is the latch AS THE RT TICK SEES IT, not the `catching.enable`
# parameter: the tick lowers it on E-STOP and on a fault latch (no automatic
# resume), so the two disagree exactly when something disarmed the controller
# out from under the operator.
bool armed
bool estop_active        # the CM's E-STOP latch is up this tick
bool fault_latched       # the controller's fault latch is up (cleared only by a fault reset)
# At configure, the parameters this configuration consumes passed validation
# (no TBD; provisional only in simulation). False means no amount of arming
# will start a trial.
bool armable
# The tracking law is wired (model + catch frame + CLIK + soft-catch
# reference). False means the controller holds the arm by design, not by
# failure.
bool law_enabled
# A real-arm configuration: true unless every device this controller claims
# is proven to be the simulator backend. Provisional parameter values block
# configure only when this is true.
bool real_arm_config

uint64 tick             # ControllerState::iteration of the tick this body is from
float64 t_relative_s    # session-relative seconds, the CSV's join key
float64 t_arm_s         # T_arm: the lead the reference is evaluated at [s]

# ── Vision input lane ───────────────────────────────────────────────────────
# What the RT tick concluded about the snapshot it was holding. These are the
# TICK's view, so they are meaningful on every tick including the ones that
# refused to use the snapshot.
bool input_valid        # the tick holds an accepted prediction (usability: input_stale)
# Unusable this tick: none held, from another activation, or older than
# io.t_stale on the steady receive axis (never judged on header.stamp).
bool input_stale
bool input_expired      # prediction ends before the instant this command lands
bool input_new          # a snapshot the previous tick had not consumed
int32 input_n           # samples in the snapshot
# Vision track epoch (the prediction's `generation` field): a new value is a
# new track.
uint64 input_generation
# The prediction's number within its track (`snapshot_sequence` field);
# strictly increasing within one generation.
uint64 input_snapshot_sequence
# The controller activation (incremented on every on_activate) the snapshot
# was received under; a snapshot from another activation is stale.
uint64 input_activation_generation
float64 input_age_s     # age on the steady receive axis — the only staleness axis.
                        # NEGATIVE means nothing has ever been received, the same
                        # 'never' tip_age_s below uses. Without the sentinel this
                        # field is now-minus-zero, i.e. the publisher's steady-clock
                        # uptime, on every tick before the first prediction.
float64 input_horizon_s # prediction horizon carried by the snapshot: last sample − first [s]

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
# recv_wall − header.stamp of the last message, accepted or not [s]
# (CLOCK_REALTIME on both sides; negative = stamp in the future). Published,
# NEVER judged on.
float64 input_origin_delay_s
# Jump between the last two accepted predictions of one track:
# J = ‖p_new(t*) − p_old(t*)‖ [m], t* = max(t₀,old, t₀,new) + io.track_eval_offset
# with t₀ each prediction's first sample instant. < 0 = not compared.
float64 input_jump_m

# ── Plan (filled by the planner) ────────────────────────────────────────────
# plan_valid false: no plan is followed. Then only plan_id, plan_age_s and
# plan_reason may be set — from the planner's latest "no plan" for this
# activation — and the other plan fields are zero.
bool plan_valid
uint32 plan_id           # the planner's monotone plan counter
float64 plan_t_c_s       # seconds from now to the catch instant (negative = past)
float64 plan_age_s       # now − the planner's publish instant [s], steady axis
float64[3] plan_p_c      # catch point [m], world
float64[3] plan_a_d      # approach axis (unit), world: −v_c/‖v_c‖
float64[3] plan_v_c      # predicted ball velocity at t_c [m/s]
# Terminal γ ∈ [0, 1], the velocity-matching ratio: at t_c the catch frame
# moves along v_c at γ‖v_c‖, so the ball enters the hand at (1 − γ)‖v_c‖.
float64 plan_gamma_f
# Catchability manipulability at the catch posture q*: w5 = √det(J₅J₅ᵀ), J₅
# the catch frame's 3 translation rows (world axes) and 2 approach-axis
# rotation rows (frame x, y), unweighted, mixed units. The default gate.
float64 plan_w5
# w6 = √det(J₆J₆ᵀ), the 6-row variant (roll about the approach axis included),
# recorded alongside w5 whichever one gates; the two are not comparable.
float64 plan_w6
# σ_max = √λ_max(Σ_pp) [m], Σ_pp the predicted ball position covariance at
# the catch sample; 0 when unknown.
float64 plan_sigma_c
# Ranking cost of the chosen candidate, lower is better (the search's own
# weighted cost; not comparable across search modes).
float64 plan_score
# Why the planner rejected candidates (rtc::catching::PlanReason). Dense. With
# no plan, the gate that refused the most candidates; the searches fold
# several rejections into one code (the planner CSV has the exact one):
#   NONE              no rejection
#   UNCERTAINTY       position uncertainty too large or unknown, or the
#                     search is still settling on a new track
#   IK_FAILED         no catch posture reaches the point (IK, or outside the
#                     reach bound)
#   MANIPULABILITY    w5 (or w6) below its minimum
#   REACH_TIME        the joints cannot reach q* before the catch instant
#   LIMITS_INVALID    joint limits unusable, or the solver / arm state refused
#   GAMMA_WINDOW      no γ meets both the hand-close and the arm-speed bound
#   STOPPING_DISTANCE no longer emitted
#   ROLLOUT           the law's rollout to t_c failed
#   ERROR_BUDGET      n_σ·σ_gap exceeds the capture radius
#   IMPULSE           never emitted (no impulse gate)
#   HORIZON_SHORT     the prediction offers no candidate (horizon below
#                     io.horizon_min, or none in the window)
#   BUDGET_EXCEEDED   the cycle's compute budget ran out first
#   INPUT_NON_FINITE  non-finite planner input
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

# ── Reference (soft-catch translational DS, SoftCatchTranslation) ───────────
# Filled on ticks the soft-catch reference stepped (the closed_form law);
# zero with ref_valid false on every other tick. The reference is the catch
# frame origin x [m], world, driven toward p_c + γ(o − p_c), o the ball (or
# the virtual decelerating target in DECEL), with the error dynamics
# ë = −ω²e − 2ζωė. `ref_xdd` is the REALISED acceleration and `ref_u_des` the
# pre-saturation demand. Both are carried because a saturated interval is
# uninterpretable from either one alone.
bool ref_valid           # false: non-finite input or command; the state was kept
bool ref_saturated       # |u| clipped to a_max or |ẋ| to v_max, or the step was rejected
float64[3] ref_x         # reference position for the next tick [m]
float64[3] ref_xd        # reference velocity for the next tick [m/s]
float64[3] ref_xdd       # acceleration realised over this tick [m/s²]
float64[3] ref_u_des     # pre-saturation demand [m/s²]
float64[3] ref_e         # e = (x − p_c) − γ(o − p_c) [m], at this tick (before the step)
float64[3] ref_ed        # ė [m/s], same instant
float64 ref_gamma        # γ(t) [-] of the quintic ramp toward plan_gamma_f
float64 ref_gamma_d      # γ̇ [1/s]
float64 ref_gamma_dd     # γ̈ [1/s²]

# ── Joint command (CLIK) ────────────────────────────────────────────────────
bool clik_ran            # the law reached the solve this tick
bool clik_converged      # the QP converged with finite iterates
bool clik_command_mismatch # evaluate_at_command: cache.q (arm) ≠ previous q_ref
# ProxQP QPSolverOutput: 0 SOLVED, 1 MAX_ITER_REACHED, 2 PRIMAL_INFEASIBLE,
# 3 SOLVED_CLOSEST_PRIMAL_FEASIBLE, 4 DUAL_INFEASIBLE, 5 NOT_RUN; −1 = not solved.
int32 clik_status
int32 clik_iterations    # ProxQP inner iterations
float64 clik_solve_us      # wall time of the solve [µs]
int32 qp_fail_streak       # consecutive TRIALS ended by a CLIK failure (not solves; a trial
                           # reaching HOLD's verdict or a fault reset resets it); supervisor.n_qp
                           # of them latch a fault

# ── Tracking ────────────────────────────────────────────────────────────────
# ‖q_meas − q_cmd‖₂ over the arm joints [rad], on the ticks that move the
# command (law, homing, RETREAT); 0 on other ticks. It is the ONLY place the
# measured arm state enters the loop (CLIK is evaluated at the command), which
# makes it the only watchdog that notices the arm is not where it was told to
# be.
float64 track_err_rad
float64[] q_cmd          # commanded arm configuration, DEVICE order — what actually
                         # went out on the wire this tick, including the hold latch
                         # while the law is not driving. NaN on a tick that commanded
                         # nothing (the output is silenced until the latch is set),
                         # so a reader cannot average a placeholder into the error.
float64[] q_meas         # measured arm configuration [rad], same order; 0 when unreadable
string[] arm_joint_names # names the two arrays are in, stamped once at configure
bool abort_stopped       # the QP-independent joint-space stop has brought the arm to rest

# ── Hand ────────────────────────────────────────────────────────────────────
# False while the hold latch commands the hand instead of the sequencer (not
# armed, E-STOP, or a configuration that runs no trials); hand_phase,
# hand_rho and hand_timeout are then zero.
bool hand_phase_valid
#   OPEN      q_open, only while the arm homes
#   PRESHAPE  q_pre, from the wait pose through COMMITTED
#   CLOSE     q_close, commanded at t_cmd = t_c − T_close_lead until ρ ≥ η_close
#             or the close timeout
#   HOLD      holding the closed posture
#   RELEASE   opening back to q_pre
uint8 HAND_PHASE_OPEN     = 0
uint8 HAND_PHASE_PRESHAPE = 1
uint8 HAND_PHASE_CLOSE    = 2
uint8 HAND_PHASE_HOLD     = 3
uint8 HAND_PHASE_RELEASE  = 4
uint8 hand_phase
# Closure fraction ρ = min over the caging joints i of
# (q_i − q_pre,i)·sign(q_close,i − q_pre,i) / |q_close,i − q_pre,i|, measured
# q, clamped to [0, 1]; 0 on a non-finite reading.
float64 hand_rho
bool hand_timeout        # CLOSE ended on its timeout, not on ρ ≥ η_close (sticky for the trial)

# ── Fingertip sensors ───────────────────────────────────────────────────────
# One entry per fingertip sensor group, in `tip_names` order. `tip_fresh`
# requires BOTH the backend's own validity flag AND this controller's
# deadline — a value being valid is not the same as a consumer's time limit
# being met, and the catching deadline is not the backend's.
string[] tip_names
# |F − b| [N]: F the fingertip force vector, b the bias learned while the
# hand rests at q_pre (ARMED, TRACKING); |F| before a bias exists. A magnitude,
# so the sensor's sign convention drops out. 0 without a contact lane.
float64[] tip_force
bool[] tip_contact       # debounced contact verdict; true only COMMITTED–HOLD and while fresh
bool[] tip_fresh         # backend-valid and receive age ≤ supervisor.contact.t_stale
float64[] tip_age_s      # age on the steady receive axis; < 0 = never received
