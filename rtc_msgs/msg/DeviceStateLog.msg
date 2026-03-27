# Device별 통합 상태 로그 (state + command + goal + trajectory)
#
# 모든 배열은 joint_names와 1:1 대응 (N = len(joint_names)).
# task_goal, actual_task_positions는 항상 6개 고정.

std_msgs/Header header

string[]  joint_names            # [N]

# ── State ──────────────────────────────────────────────────────────────────
float64[] actual_positions       # [N]
float64[] actual_velocities      # [N]
float64[] efforts                # [N]

# ── Command ────────────────────────────────────────────────────────────────
float64[] commands               # [N]
string    command_type           # "position" | "torque"

# ── Goal ───────────────────────────────────────────────────────────────────
string    goal_type              # "joint" | "task"
float64[] joint_goal             # [N]
float64[6] task_goal             # [x, y, z, roll, pitch, yaw]

# ── Trajectory ─────────────────────────────────────────────────────────────
float64[] trajectory_positions   # [N]
float64[] trajectory_velocities  # [N]

# ── Task-space FK ──────────────────────────────────────────────────────────
float64[6] actual_task_positions # [x, y, z, roll, pitch, yaw]

# ── Motor State (optional, empty if no motor_state_names configured) ──────
string[]  motor_names            # [M]
float64[] motor_positions        # [M]
float64[] motor_velocities       # [M]
float64[] motor_efforts          # [M] (motor currents)
