# 컨트롤러에서 500Hz로 계산된 grasp 상태
# BT coordinator가 /hand/sensor_states/monitor 대신 구독

std_msgs/Header header

# Per-fingertip (배열 크기 = num_fingertips, 보통 4)
string[]  fingertip_names       # ["thumb", "index", "middle", "ring"]
float32[] force_magnitude       # |F| per fingertip [N]
float32[] contact_flag          # 접촉 확률 (0.0~1.0)
bool[]    inference_valid       # per-fingertip inference enable

# Aggregate (컨트롤러에서 계산)
int32     num_active_contacts   # active contact 핑거팁 수
float32   max_force             # 전체 핑거팁 중 최대 force magnitude
bool      grasp_detected        # num_active_contacts >= min_fingertips
float32   force_threshold       # 감지에 사용된 threshold [N]
int32     min_fingertips        # grasp 판정 최소 핑거팁 수
