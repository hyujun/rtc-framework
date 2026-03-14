# 로봇 암 조인트 커맨드 (position 또는 torque)
#
# joint_names와 values는 1:1 대응.
# joint_names가 비어있으면 기본 조인트 순서로 fallback.

std_msgs/Header header

string[]  joint_names     # YAML에서 정의 (e.g. ["shoulder_pan_joint", "elbow_joint"])
float64[] values          # position(rad) 또는 torque(Nm)

string command_type       # "position" | "torque"
