# GUI 표시용 현재 위치 (joint + task space)
#
# joint_names와 joint_positions는 1:1 대응.
# task_positions는 FK 결과 [x, y, z, roll, pitch, yaw] 고정 6개.

std_msgs/Header header

string[]   joint_names       # YAML joint_state_names 기반 동적 할당
float64[]  joint_positions   # [N] 현재 관절 위치 (rad)
float64[6] task_positions    # [x, y, z, roll, pitch, yaw] FK 결과
