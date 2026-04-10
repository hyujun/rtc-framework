# 센서 캘리브레이션 명령 (확장 가능: 여러 센서 타입을 단일 토픽으로 처리)
#
# 사용 예:
#   ros2 topic pub --once /hand/calibration/command rtc_msgs/msg/CalibrationCommand '{sensor_type: 0, action: 0, sample_count: 0}'
#
# 새 센서 타입 추가 절차:
#   1. 아래 SENSOR_* 상수에 enum 값 추가
#   2. HandController::DispatchCalibrationRequest() switch에 case 추가
#   3. HandController::GetCalibrationStatus()에 분기 추가
#   4. demo_controller_gui.py의 SENSOR_CALIBRATIONS 리스트에 한 줄 추가

# ── Sensor type enum ─────────────────────────────────────────────
uint8 SENSOR_BAROMETER = 0   # Fingertip barometer baseline offset
uint8 SENSOR_FT_ZERO   = 1   # (reserved) F/T sensor zero
uint8 SENSOR_TOF_DARK  = 2   # (reserved) ToF dark-level

# ── Action enum ──────────────────────────────────────────────────
uint8 ACTION_START = 0   # 누적 버퍼 리셋 후 재샘플링 시작
uint8 ACTION_ABORT = 1   # 진행 중인 캘리브레이션 취소

std_msgs/Header header

uint8  sensor_type      # SENSOR_* 중 하나
uint8  action           # ACTION_* 중 하나
uint16 sample_count     # 0 = YAML 기본값 사용, >0 = 이번 요청에 한해 override
