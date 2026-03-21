# 핸드 모터 피드백
#
# motor_names/positions/velocities는 1:1 대응.
# motor_names가 비어있으면 기본 모터 순서로 fallback.

std_msgs/Header header

string[]  motor_names     # YAML에서 정의
float32[] positions       # 모터 위치 (rad)
float32[] velocities      # 모터 속도 (rad/s)
