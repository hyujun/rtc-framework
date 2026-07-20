# 컨트롤러에서 500Hz로 계산된 grasp 상태
# BT coordinator가 /hand/sensor_states/monitor 대신 구독

std_msgs/Header header

# Per-fingertip (배열 크기 = num_fingertips, 보통 4)
string[]  fingertip_names       # ["thumb", "index", "middle", "ring"]
float32[] force_magnitude       # |F| per fingertip [N]
# contact_flag — backend capability 분기:
#   sensor A (udp_hand_native + ft_inferencer.enabled) → native sigmoid prob [0..1]
#   sensor B (force-only, e.g. mujoco wrench) → controller-derived binary
#     1.0F / 0.0F from |F| > gains.grasp_force_threshold
# BT/GUI 는 두 경우 모두 `> 0.5f` in-contact 판정.
float32[] contact_flag
bool[]    inference_valid       # per-fingertip inference enable

# Aggregate (컨트롤러에서 계산)
int32     num_active_contacts   # active contact 핑거팁 수
float32   max_force             # 전체 핑거팁 중 최대 force magnitude
bool      grasp_detected        # num_active_contacts >= min_fingertips
float32   force_threshold       # 감지에 사용된 threshold [N]
int32     min_fingertips        # grasp 판정 최소 핑거팁 수

# Force-PI grasp controller state (grasp_controller_type == "force_pi" 전용)
uint8     grasp_phase           # GraspPhase enum (0=Idle..5=Releasing)
float32[] finger_s              # grasp parameter per finger [0,1]
float32[] finger_filtered_force # filtered force per finger [N]
float32[] finger_force_error    # force error per finger [N]
float32   grasp_target_force    # active target force [N]

# In-plane pull-force estimate (#167) — rtc::grasp::PullForceEstimator 출력.
# GraspState/WbcState 가 동일한 PullEstimate 하위 메시지를 embed 하므로 GUI 등
# 소비자는 두 msg 의 저장 코드를 그대로 공유한다.
PullEstimate pull
