# Device별 관절 상태
#
# joint_names/positions/velocities/efforts는 1:1 대응.
# joint_names가 비어있으면 기본 순서로 fallback.

std_msgs/Header header

string[]  joint_names   # YAML joint_state_names 기반 동적 할당
float64[] positions     # [N] 관절 위치 (rad)
float64[] velocities    # [N] 관절 속도 (rad/s)
float64[] efforts       # [N] 관절 토크 (Nm)
