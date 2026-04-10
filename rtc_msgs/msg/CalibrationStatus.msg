# 센서 캘리브레이션 진행/완료 상태 (센서 타입당 1개 메시지)
#
# Publisher: ur5e_hand_driver가 주기적으로 (또는 상태 변화 시) publish
# Subscriber: demo_controller_gui 등에서 진행률/완료 표시
#
# QoS: reliable + transient_local 권장 (late-join GUI가 최신 상태 즉시 수신)

# ── State enum ───────────────────────────────────────────────────
uint8 STATE_IDLE     = 0   # 캘리브레이션 요청 없음 또는 완료 대기
uint8 STATE_RUNNING  = 1   # 샘플 누적 중
uint8 STATE_COMPLETE = 2   # 정상 완료 (baseline_offset 갱신됨)
uint8 STATE_FAILED   = 3   # 실패 (센서 미초기화, 추론기 없음, 등)

std_msgs/Header header

uint8  sensor_type      # CalibrationCommand.SENSOR_* 와 동일 enum
uint8  state            # STATE_* 중 하나
uint16 progress_count   # 현재까지 누적된 샘플 수
uint16 target_count     # 목표 샘플 수 (progress_count / target_count = 진행률)
string message          # 사람-읽기용 (실패 원인 등, 선택)
