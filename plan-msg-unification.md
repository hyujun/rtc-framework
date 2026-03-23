# FingertipSensor + FingertipForceTorque 통합 계획

## 현재 상태 요약

| 메시지 | 용도 | 실제 사용 |
|--------|------|-----------|
| `FingertipSensor.msg` | 센서 raw + 추론 | **미사용 (dead code)** |
| `HandSensorState.msg` | FingertipSensor[] 래퍼 | **미사용 (dead code)** |
| `FingertipForceTorque.msg` | 추론 결과만 | hand_udp_node.cpp에서 publish |
| `HandForceTorqueState.msg` | FingertipForceTorque[] 래퍼 | hand_udp_node.cpp에서 publish |
| `Float64MultiArray` | 센서 raw 데이터 | rt_controller_node, digital_twin_node에서 subscribe |

## 통합 설계

### 새 메시지: `FingertipState.msg` (FingertipSensor + FingertipForceTorque 통합)

```msg
# 단일 fingertip의 센서 + 추론 통합 데이터
string     name                  # YAML에서 정의 (e.g. "index", "thumb")

# Raw sensor data
float32[8] barometer              # 압력 센서 8ch (고정)
float32[3] tof                    # ToF 거리 센서 3ch (고정)

# Inference output (ONNX 3-head model)
bool       contact                # 접촉 여부 (contact_probability > 0.5)
float32    contact_probability    # sigmoid(contact_logit) → [0, 1]
float32[3] force                  # 힘 벡터 [Fx, Fy, Fz] in N
float32[3] direction              # 단위 방향 벡터 [ux, uy, uz]
```

### 새 메시지: `HandState.msg` (HandSensorState + HandForceTorqueState 통합)

```msg
# 전체 핸드 상태 (센서 + 추론)
std_msgs/Header header
FingertipState[] fingertips
```

## 변경 단계

### Step 1: 새 메시지 파일 생성
- `rtc_msgs/msg/FingertipState.msg` 생성
- `rtc_msgs/msg/HandState.msg` 생성
- `rtc_msgs/CMakeLists.txt`에 새 메시지 등록

### Step 2: hand_udp_node.cpp 퍼블리셔 통합
- `Float64MultiArray` 센서 publisher + `HandForceTorqueState` FT publisher → 단일 `HandState` publisher로 통합
- 기존 두 publish 블록(L284-294, L296-321)을 하나로 병합
- `sensor_state_pub_`(Float64MultiArray) + `ft_pub_`(HandForceTorqueState) → `hand_state_pub_`(HandState) 하나로
- sensor_data(int32)를 float32 barometer[8] + tof[3]로 분리 매핑
  - `kSensorValuesPerFingertip=11` → barometer[0..7] + tof[8..10]
- FT inference 미활성 시: contact/force/direction 필드는 기본값(0) 유지

### Step 3: Subscriber 업데이트
- **rt_controller_node.cpp** (`DeviceSensorCallback`): `Float64MultiArray` → `HandState` 구독으로 변경
  - `DeviceStateCache::sensor_data[]` 매핑 로직 수정
- **digital_twin_node.py** (`_sensor_cb`): `Float64MultiArray` → `HandState` 구독으로 변경
- **controller YAML들** (`demo_task_controller.yaml`, `demo_joint_controller.yaml`): 토픽/타입 설정 업데이트

### Step 4: 구 메시지 삭제
- `FingertipSensor.msg` 삭제
- `HandSensorState.msg` 삭제
- `FingertipForceTorque.msg` 삭제
- `HandForceTorqueState.msg` 삭제
- `CMakeLists.txt`에서 제거

### Step 5: 문서/설정 업데이트
- `rtc_msgs/README.md` 업데이트
- `rtc_msgs/CHANGELOG.md` 업데이트
- `README.md` (루트) 업데이트
- `docs/CLAUDE.md` 업데이트

## 영향 범위 정리

| 파일 | 변경 유형 |
|------|-----------|
| `rtc_msgs/msg/FingertipState.msg` | **신규** |
| `rtc_msgs/msg/HandState.msg` | **신규** |
| `rtc_msgs/msg/FingertipSensor.msg` | **삭제** |
| `rtc_msgs/msg/HandSensorState.msg` | **삭제** |
| `rtc_msgs/msg/FingertipForceTorque.msg` | **삭제** |
| `rtc_msgs/msg/HandForceTorqueState.msg` | **삭제** |
| `rtc_msgs/CMakeLists.txt` | 수정 |
| `ur5e_hand_driver/src/hand_udp_node.cpp` | 수정 (publisher 통합) |
| `rtc_controller_manager/src/rt_controller_node.cpp` | 수정 (subscriber 변경) |
| `rtc_digital_twin/rtc_digital_twin/digital_twin_node.py` | 수정 (subscriber 변경) |
| `ur5e_bringup/config/controllers/*.yaml` | 수정 (토픽/타입) |
| `rtc_msgs/README.md` | 수정 |
| `rtc_msgs/CHANGELOG.md` | 수정 |
| `README.md` | 수정 |
| `docs/CLAUDE.md` | 수정 |

## 주의사항
- `sensor_data`는 현재 `int32_t[]`로 저장됨 → `float32` barometer/tof로 캐스팅 필요
- FT inference 비활성 시에도 `HandState`는 발행하되, 추론 필드는 기본값
- 토픽 이름은 `/hand/state`로 통합 (기존 `/hand/sensor_states` + `/hand/force_torque_state` 대체)
