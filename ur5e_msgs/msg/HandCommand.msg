# 핸드 모터 커맨드
#
# motor_names와 values는 1:1 대응.
# motor_names가 비어있으면 기본 모터 순서로 fallback.

std_msgs/Header header

string[]  motor_names     # YAML에서 정의 (e.g. ["thumb_cmc", "index_mcp"])
float32[] values          # motor position (rad)
