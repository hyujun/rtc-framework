# In-plane pull-force estimate (#167) — rtc::grasp::PullForceEstimator output
# (wire mirror of the rtc::grasp::PullEstimateData POD). Embedded as
# `PullEstimate pull` in both GraspState.msg (Force-PI grasp) and WbcState.msg
# (TSID whole-body — measured R_i·f_i over the contact geometry, not λ_opt).
# Field names match the POD members so the publish-path copy and consumer
# (GUI) storage code are shared across both parent messages.
#
# Reference frame: the parent message's `header.frame_id` — every vector below
# (force, plane_normal, basis_x) is expressed in it. The publish path fills it
# with the controller's FK reference (robot base) link (#234 P-5).
float32[3] force                 # filtered F̂, reference frame [N]
float32[2] force_inplane         # Bᵀ·F̂ plane coordinates [N]

# ── Self-description of the plane and its basis (#234 P-5) ───────────────────
# The plane normal is re-derived from the *observed* pinch axis every tick and
# the in-plane basis is rebuilt from it, so force_inplane is uninterpretable —
# and incomparable across ticks — without the two vectors below. e_y is not
# sent because it is determined: e_y = plane_normal × basis_x.
#
# plane_normal is zero only when the tick had no usable normal at all
# (invalid_reason == INVALID_DEGENERATE_NORMAL). basis_x is additionally zero
# on any tick that was not valid, since the basis is only built for the ticks
# that produce an in-plane vector — so a tick can carry a normal but no basis
# (e.g. the plane was fine but the thumb dropped out).
float32[3] plane_normal          # n̂, reference frame (unit; zero ⇒ none)
float32[3] basis_x               # ê_x, reference frame (unit; zero ⇒ none)

# Which rule produced basis_x. force_inplane[0] is the component along the
# configured in-plane reference direction (anti-gravity by default) only under
# BASIS_REFERENCE; the fallbacks are path-dependent and carry no such naming.
uint8 BASIS_NONE=0
uint8 BASIS_REFERENCE=1          # configured reference projected into the plane
uint8 BASIS_CARRY=2              # previous ê_x re-projected (reference collapsed)
uint8 BASIS_SEED=3               # world-axis seed (no usable carry either)
uint8      basis_source

# First validity gate that failed; INVALID_NONE iff `valid`. Distinguishes the
# causes that otherwise collapse into a single valid=false — in particular it
# is what makes a WBC trace comparable with a joint/task one, since WBC also
# requires a TSID-routing tick and drops all contacts on release/fallback
# ticks (#234 P-11).
uint8 INVALID_NONE=0
uint8 INVALID_NOT_INITIALIZED=1
uint8 INVALID_DEGENERATE_NORMAL=2   # no usable plane normal this tick
uint8 INVALID_REQUIRED_CONTACT=3    # a required role (e.g. thumb) was not active
uint8 INVALID_INSUFFICIENT_CONTACTS=4
uint8      invalid_reason

# Per-contact sets, bit k = contact k in the estimator's configured tip order
# (#234 P-12/P-14). contact_mask is the post-hysteresis set that entered the
# force sum — the set valid_contact_count counts. touch_mask is the
# axis-independent |f_obj| set used to observe the grasp shape; it includes the
# thumb, so touch_mask == 0 ("nothing is touching") is distinguishable from a
# thumb-only or geometry-invalid grasp.
uint8      contact_mask
uint8      touch_mask

float32    magnitude             # |F̂| [N]
float32    directional           # dᵀ·F̂ (0 unless direction set) [N]
# Slip diagnostics. These are per-contact quantities and are evaluated
# independently of `valid` (#234 P-7): a required-role dropout must not report
# "no slip risk" for the tips still holding. friction_utilization is relative
# to the *assumed* per-contact μ from config, not a measured one — read it as a
# trend indicator, not an absolute safety margin (#234 P-18).
float32    friction_utilization  # max_i |f_t,i| / (μ_i·f_n,i)
float32    leakage_bound         # grip→in-plane leakage bound [N]
int32      valid_contact_count
# Pull-vector validity only: gates force / force_inplane / magnitude /
# directional. Does NOT gate friction_utilization / slip_risk (see above).
bool       valid
bool       slip_risk             # slip ratio over threshold
bool       any_saturated         # a contact saturated this tick
bool       baseline_applied      # baseline subtraction active
