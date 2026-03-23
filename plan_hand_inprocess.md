# 방식 A: HandController In-Process 통합 계획

## 목표

hand_udp_node(별도 프로세스)를 제거하고, rt_controller_node가 HandController를 직접
소유하여 DDS 토픽 경유 없이 SeqLock으로 hand state를 읽는다.

## Before → After

```
[Before] DDS 2회 경유 (~1-3ms 지연, 100Hz publish)

  rt_controller_node                 hand_udp_node (별도 프로세스)
    Phase 3 → SPSC → publish_thread    EventLoop (Core5, FIFO 65)
      → /hand/joint_command ─DDS─→       sub → SendCmd → UDP → SeqLock
      ← /hand/joint_states ──DDS─←       wall_timer(100Hz) → pub
      ← /hand/sensor_states ─DDS─←

[After] SeqLock 직접 접근 (~50ns, 500Hz 완전 동기)

  rt_controller_node (단일 프로세스)
    Phase 1:   hand_controller_->GetLatestState()  ← SeqLock (~50ns)
    Phase 2:   Compute()
    Phase 3:   SPSC push (모니터링용 publish 유지)
    Phase 3.5: hand_controller_->SendCommandAndRequestStates(cmd)
                                    ↓ condvar notify
    EventLoop (Core 5, FIFO 65):  UDP write+read → SeqLock store
```

## 변경 대상 파일 (7개)

| # | 파일 | 변경 내용 |
|---|------|----------|
| 1 | `rtc_controller_manager/CMakeLists.txt` | `find_package(ur5e_hand_driver)` 추가 |
| 2 | `rtc_controller_manager/package.xml` | `<depend>ur5e_hand_driver</depend>` 추가 |
| 3 | `rtc_controller_manager/include/.../rt_controller_node.hpp` | HandController 멤버 + 메서드 선언 |
| 4 | `rtc_controller_manager/src/rt_controller_node.cpp` | 핵심 로직 (Phase 1/3.5, subscription skip) |
| 5 | `ur5e_bringup/config/ur5e_robot.yaml` | hand_inprocess 파라미터 병합 |
| 6 | `ur5e_bringup/launch/robot.launch.py` | hand_udp_node 조건부 제거 |
| 7 | `ur5e_hand_driver/` | **변경 없음** (standalone 유지) |

## 구현 상세

### Step 1: 패키지 의존성 추가

**rtc_controller_manager/CMakeLists.txt:**
```cmake
find_package(ur5e_hand_driver REQUIRED)
# ament_target_dependencies에 ur5e_hand_driver 추가
```

**rtc_controller_manager/package.xml:**
```xml
<depend>ur5e_hand_driver</depend>
```

### Step 2: rt_controller_node.hpp 수정

```cpp
// 신규 include
#include <ur5e_hand_driver/hand_controller.hpp>
#include <ur5e_hand_driver/hand_failure_detector.hpp>

// 신규 멤버
std::unique_ptr<rtc::HandController>      hand_controller_;
std::unique_ptr<rtc::HandFailureDetector> hand_failure_detector_;
int  hand_device_slot_{-1};       // group_slot_map_["hand"], -1 = 비활성
bool hand_inprocess_enabled_{false};

// 신규 메서드
void InitHandController();
void UpdateHandStateFromController();
void SendHandCommand(const rtc::ControllerOutput& output);
```

### Step 3: rt_controller_node.cpp — InitHandController()

생성자의 LoadDeviceNameConfigs() 직후 호출:
- `hand_inprocess_enabled` 파라미터 확인
- group_slot_map_에서 "hand" 슬롯 탐색
- 파라미터 로드: target_ip, target_port, recv_timeout_ms, communication_mode,
  sensor_decimation, baro/tof_lpf, ft_inferencer 등
  (hand_udp_node.yaml과 fingertip_ft_inferencer.yaml의 파라미터 동일)
- `SelectThreadConfigs()`로 EventLoop/FailureDetector ThreadConfig 획득
- HandController 생성 + Start()
- HandFailureDetector 생성 + FailureCallback에서 TriggerGlobalEstop() 호출
- hand_controller_->SetEstopFlag(&global_estop_)

### Step 4: ControlLoop() 수정 — Phase 1 + Phase 3.5

**Phase 1 (상태 읽기) — try_lock 블록 직전에 추가:**
```cpp
if (hand_inprocess_enabled_ && hand_controller_ && hand_device_slot_ >= 0) {
    UpdateHandStateFromController();
}
```

UpdateHandStateFromController():
- `hand_controller_->GetLatestState()` (SeqLock Load, ~50ns)
- `device_states_[hand_device_slot_]`에 position/velocity/sensor 복사
- DeviceTimeout 갱신 (last_update + received)
- state_received_ 설정

**Phase 3.5 (명령 전송) — Phase 3 SPSC push 직후에 추가:**
```cpp
if (hand_inprocess_enabled_ && hand_controller_ && hand_device_slot_ >= 0) {
    SendHandCommand(output);
}
```

SendHandCommand():
- active_tc.groups에서 "hand"의 device index 찾기
- output.devices[hand_gi].commands를 float array로 변환
- `hand_controller_->SendCommandAndRequestStates(cmd)` (condvar notify, ~1µs)

### Step 5: CreateSubscriptions() 수정

hand_inprocess_enabled_ 시 "hand" 그룹의 kState, kSensorState role subscription skip:
```cpp
if (hand_inprocess_enabled_ && group_name == "hand" &&
    (entry.role == kState || entry.role == kSensorState)) {
    RCLCPP_INFO(get_logger(),
        "  [Hand in-process] Skip DDS sub: %s", entry.topic_name.c_str());
    continue;  // DDS subscription 생성 안 함
}
```
- kTarget role은 유지 (외부 목표값은 여전히 토픽 수신)
- publish 토픽 (/hand/joint_command 등)도 유지 (모니터링용)

### Step 6: ur5e_robot.yaml 파라미터 추가

```yaml
# ── Hand in-process integration ───────────────────────────
hand_inprocess_enabled: true

# ── Hand UDP (기존 hand_udp_node.yaml에서 이동) ──────────
hand_target_ip: "192.168.1.2"
hand_target_port: 55151
hand_recv_timeout_ms: 0.4
hand_communication_mode: "bulk"
hand_sensor_decimation: 3
hand_use_fake_hand: false
hand_baro_lpf_enabled: false
hand_baro_lpf_cutoff_hz: 30.0
hand_tof_lpf_enabled: false
hand_tof_lpf_cutoff_hz: 15.0
```
FT inferencer 파라미터는 기존 fingertip_ft_inferencer.yaml 로드 유지.

### Step 7: robot.launch.py 수정

```python
hand_inprocess_arg = DeclareLaunchArgument(
    'hand_inprocess', default_value='true',
    description='Hand in-process mode (true=SeqLock, false=separate node)')

# hand_udp_node는 hand_inprocess==false일 때만 launch
hand_udp_node = Node(
    ...
    condition=UnlessCondition(LaunchConfiguration('hand_inprocess'))
)
```

## 소멸자

```cpp
~RtControllerNode() {
    // 기존 cleanup ...
    if (hand_failure_detector_) hand_failure_detector_->Stop();
    if (hand_controller_) hand_controller_->Stop();
}
```

## 호환성

- `hand_inprocess_enabled: false` → 기존 hand_udp_node 별도 프로세스 방식 유지
- hand_udp_node 패키지 삭제 안 함 (standalone 디버깅/테스트용)
- Publish thread의 /hand/joint_command publish 유지 (외부 모니터링 도구 호환)

## Core 할당 변경 없음

In-process 통합 후 모든 스레드가 rt_controller_node 내부에서 동작:
- EventLoop: Core 5, FIFO 65 (기존과 동일)
- HandFailureDetector: Core 4, OTHER -2 (기존과 동일)
- DDS recv/send thread 불필요 → Core 0-1 부하 감소

## 구현 순서

1. Step 1: 패키지 의존성 (CMakeLists.txt, package.xml)
2. Step 2: hpp 수정 (멤버 + 메서드 선언)
3. Step 3: InitHandController() 구현
4. Step 4: ControlLoop() Phase 1 + Phase 3.5 수정
5. Step 5: CreateSubscriptions() hand subscription skip
6. Step 6: ur5e_robot.yaml 파라미터 병합
7. Step 7: robot.launch.py 조건부 launch
8. 빌드 + 테스트 (colcon build, mock hardware)
