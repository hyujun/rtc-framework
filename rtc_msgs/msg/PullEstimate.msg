# In-plane pull-force estimate (#167) — rtc::grasp::PullForceEstimator output
# (wire mirror of the rtc::grasp::PullEstimateData POD). Embedded as
# `PullEstimate pull` in both GraspState.msg (Force-PI grasp) and WbcState.msg
# (TSID whole-body — measured R_i·f_i over the contact geometry, not λ_opt).
# Field names match the POD members so the publish-path copy and consumer
# (GUI) storage code are shared across both parent messages.
float32[3] force                 # filtered F̂, reference frame [N]
float32[2] force_inplane         # Bᵀ·F̂ plane coordinates [N]
float32    magnitude             # |F̂| [N]
float32    directional           # dᵀ·F̂ (0 unless direction set) [N]
float32    friction_utilization  # max_i |f_t,i| / (μ_i·f_n,i)
float32    leakage_bound         # grip→in-plane leakage bound [N]
int32      valid_contact_count
bool       valid                 # estimate valid (required-role gate passed)
bool       slip_risk             # slip ratio over threshold
bool       any_saturated         # a contact saturated this tick
bool       baseline_applied      # baseline subtraction active
