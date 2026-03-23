# Device별 목표 위치 (joint space 또는 task space)
#
# goal_type으로 joint_goal과 task_goal 중 사용할 것을 결정.
# joint_names/joint_goal은 1:1 대응, joint_state_names 기반 동적 할당.
# task_goal은 항상 6개 고정.

std_msgs/Header header

string     goal_type        # "joint" | "task"

string[]   joint_names      # [N] YAML joint_state_names 기반
float64[]  joint_goal       # [N] 관절 목표 위치 (rad)

float64[6] task_goal        # [x, y, z, roll, pitch, yaw] 고정 6개
