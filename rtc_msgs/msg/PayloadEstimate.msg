# Quasi-static payload estimate (#135 Layer 2A) — wire mirror of the
# integrated_bringup `MomentumObserverLogPod` row that `momentum_observer.csv`
# carries. Published per controller by every demo controller that wires a
# MomentumObserverWiring (joint / task / wbc).
#
# THE RESIDUAL AND THE PAYLOAD RIDE ONE MESSAGE ON PURPOSE (#135 D12). They are
# produced in a single tick from a single set of inputs, and a reader diagnosing
# a suspicious mass needs the `r` that produced it. Two topics would push that
# time alignment onto every consumer. The CSV channel made the same call for the
# same reason, and this message is deliberately field-for-field parallel to it
# so a stored file and a live topic are read the same way.
#
# STALENESS IS `tick`, NOT `header.stamp`. The stamp is the publish thread's
# wall clock at send time (rtc_base/threading/publish_buffer.hpp), so it keeps
# advancing after the RT loop stops writing. `tick` is the RT iteration the row
# was computed on and is the field a reader compares.

std_msgs/Header header

# CM RT-loop tick index and session-relative time of the tick this estimate was
# computed on. `tick` joins this message to the momentum_observer.csv row of the
# same number.
uint64  tick
float64 t_relative_s

# ── Residual block — Layer 1 (generalized-momentum observer) ──────────────────
#
# JOINT ORDER IS CARRIED, NOT ASSUMED. `residual` is in the arm DEVICE order,
# which in general is NOT the URDF/pinocchio order a model Jacobian's columns
# use. Pairing a residual with a quantity in the other order produces a finite,
# smooth, wrong answer — a real trap this repo has a mutation test for — so this
# message names its own order instead of leaving a consumer to guess. Same
# reason momentum_observer.csv writes `r_<joint>` column headers. `joint_names`
# and `residual` always have equal length.
string[]  joint_names
float64[] residual              # r [N.m], device order

# max|r| over `residual`. Derivable from the array, carried anyway: it is the
# quantity the acceptance criterion is stated in, and a single trace beats a
# per-tick max() over a variable-width block.
float64 residual_inf_norm

# Valid ticks since the momentum reference was last captured. While this is
# small the residual is still converging from zero after a gap, so a small |r|
# does not yet mean "no external torque" — a reader that must not confuse the
# two discards roughly the first 3/K_I worth of ticks.
uint32 ticks_since_seed

# rtc::estimation::MomentumInvalidReason — the first gate that failed;
# RESIDUAL_NONE iff residual_valid. A closed lane gate (RESIDUAL_HELD, which
# includes E-STOP) reads differently from a non-finite input or a stalled clock.
#
# On any tick with residual_valid=false the residual is FROZEN at its last
# accepted value rather than zeroed — a zero would assert "no external torque"
# on a tick whose inputs were refused.
uint8 RESIDUAL_NONE=0
uint8 RESIDUAL_NOT_INITIALIZED=1
uint8 RESIDUAL_HELD=2
uint8 RESIDUAL_SHORT_INPUT=3
uint8 RESIDUAL_NON_FINITE_INPUT=4
uint8 RESIDUAL_NON_POSITIVE_DT=5
uint8 residual_reason
bool  residual_valid

# ── Payload block — Layer 2A (quasi-static wrench inversion) ──────────────────
#
# All-zero with payload_reason = PAYLOAD_NOT_INITIALIZED on a controller whose
# config carries no `payload_estimator` block, or carries it disabled — which is
# distinguishable from a gate that closed, since a gate names itself. The
# residual block above stays live in that case: the observer and the estimator
# are configured independently, and the observer is the one this topic exists
# for when the estimator is off.

# The frame the wrench acts AT (config `payload_estimator.frame`, resolved
# against the arm sub-model). Empty iff the estimator was never configured.
#
# The wrench axes are NOT this frame's. They are LOCAL_WORLD_ALIGNED: aligned
# with `header.frame_id` (the arm root), with the moment taken about
# `payload_frame`'s origin. Both names are load-bearing — a wrench is a screw,
# so the reference point changes the torque even when the axes do not.
string payload_frame

# w_ext — the wrench the payload applies TO the robot, so a hanging mass gives a
# DOWNWARD force. The sign is inherited from the observer's [MO-3a] residual
# convention rather than chosen here; the derivation and the two independent
# confirmations are in rtc_controllers/estimation/payload_estimator.hpp.
geometry_msgs/Wrench wrench

# m = (f_ext . Wg)/|Wg|^2 [kg]. Positive for a real payload. Taken against the
# MODEL's gravity vector rather than an up-axis constant, so a model with
# gravity along Y — or a lunar one — needs no separate convention kept in sync.
float64 mass

# Section 6.5 damping diagnostics. lambda_sq is 0 away from singularities (the
# estimate is then undamped least squares, carrying no shrinkage bias);
# sigma_min says how observable the wrench was at this pose. Carried because the
# decision to keep the damping ADAPTIVE rather than flooring it is only
# reviewable if the damping actually applied is visible.
float64 sigma_min
float64 lambda_sq

# max|J^T w - r| [N.m] — how well the fitted wrench explains the residual. This
# is the one number that separates a payload from an unmodelled joint-level
# torque (friction, an MJCF armature term with no URDF equivalent): a residual
# outside J^T's range reconstructs badly no matter which wrench is fitted.
float64 fit_error

# rtc::estimation::PayloadInvalidReason — WHICH gate closed, in evaluation
# order. The gates fire at very different rates (the peripheral-motion one
# dominates in sim), so collapsing them into payload_valid would hide exactly
# what a tuner needs. PAYLOAD_ARM_MOVING / PAYLOAD_PERIPHERAL_MOVING /
# PAYLOAD_NOT_CONVERGED are TOLD to the estimator by its caller, since joint
# velocities and observer seed age belong to the caller's device view; the rest
# the estimator decides from inputs it can see.
#
# PAYLOAD_PERIPHERAL_MOVING mirrors PayloadInvalidReason::kHandMoving. It is
# named for what the gate does — the quasi-static assumption is violated by ANY
# device the observed sub-model does not include, not by a hand specifically —
# and matches the `max_peripheral_velocity` config key.
uint8 PAYLOAD_NONE=0
uint8 PAYLOAD_NOT_INITIALIZED=1
uint8 PAYLOAD_HELD=2
uint8 PAYLOAD_SHORT_INPUT=3
uint8 PAYLOAD_NON_FINITE_INPUT=4
uint8 PAYLOAD_OBSERVER_INVALID=5
uint8 PAYLOAD_NOT_CONVERGED=6
uint8 PAYLOAD_ARM_MOVING=7
uint8 PAYLOAD_PERIPHERAL_MOVING=8
uint8 PAYLOAD_DEGENERATE_GRAVITY=9
uint8 PAYLOAD_SOLVER_FAILED=10
uint8 PAYLOAD_RANK_DEFICIENT=11
uint8 PAYLOAD_POOR_FIT=12
uint8 payload_reason
bool  payload_valid
