# rtc_msgs

RTC 노드들이 주고받는 커스텀 ROS 2 메시지 · 서비스 정의. **유형**: 메시지 패키지

## 개요

로봇 비의존적인 인터페이스 정의만 담는다 (코드 없음, rosidl 이 C++ · Python 바인딩을 생성). 메시지 17개, 서비스 7개.

이 README 는 **색인**이다. 필드 의미 · 단위 · 판정 기준 · 거부 조건은 각 `.msg` / `.srv` 파일의 주석이 SSoT 이므로 여기에 옮겨 적지 않는다. 소비자: `rtc_controller_manager`, `rtc_controller_interface`, `integrated_bringup`, `udp_hand_driver`, `rtc_mujoco_sim`, `shape_estimation`, `ur5e_bt_coordinator`.

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다. `rtc_msgs` 를 바꾸면 전체 downstream 이 다시 빌드된다.

```bash
ros2 interface list | grep rtc_msgs
ros2 interface show rtc_msgs/msg/JointCommand
ros2 interface show rtc_msgs/srv/SwitchController
ros2 service call /rtc_cm/list_controllers rtc_msgs/srv/ListControllers
```

Python 에서는 `from rtc_msgs.msg import JointCommand`, `from rtc_msgs.srv import SwitchController` 로 쓴다. C++ 은 `rtc_msgs/msg/joint_command.hpp` 를 include 하고 `package.xml` 에 `<depend>rtc_msgs</depend>` 를 둔다.

## 인터페이스

토픽 이름의 `<config_key>` 는 컨트롤러 이름, `<group>` 은 디바이스 그룹 이름이다. 컨트롤러가 소유하는 토픽 (GraspState · WbcState · ToFSnapshot · CatchingState · PayloadEstimate) 은 컨트롤러 YAML 의 `topics:` role 로 선언하지 않는다 — 컨트롤러 YAML 이 받는 role 은 `target` (subscribe) 과 `robot_transforms` (publish) 뿐이다.

### 메시지

| 타입 | 용도 | 토픽 | 발행 노드 |
|---|---|---|---|
| [`JointCommand`](msg/JointCommand.msg) | 관절 커맨드 (position · torque · pd_feedforward) | `/<group>/joint_command` | `rtc_controller_manager` 의 device backend (구독: `rtc_mujoco_sim` · `udp_hand_driver`) |
| [`RobotTarget`](msg/RobotTarget.msg) | 관절 · 태스크 공간 목표 | 컨트롤러 YAML `role: "target"` 토픽 (예: `ur5e/joint_goal`) | 외부 · GUI · `shape_estimation` (구독: `rtc_controller_manager`) |
| [`HandSensorState`](msg/HandSensorState.msg) | 핸드 센서 상태 (핑거팁 집계) | `/<group>/sensor_states` · `…/monitor` (RELIABLE) | `udp_hand_driver` |
| [`FingertipSensor`](msg/FingertipSensor.msg) | 핑거팁 1개의 센서 값 + 추론 결과 | `HandSensorState` 의 원소 | `udp_hand_driver` |
| [`SimSensorState`](msg/SimSensorState.msg) | MuJoCo 센서 집계 | sim YAML 의 `sensor_topic` (기본 미설정) | `rtc_mujoco_sim` |
| [`SimSensor`](msg/SimSensor.msg) | MuJoCo 센서 1개 출력 | `SimSensorState` 의 원소 | `rtc_mujoco_sim` |
| [`GraspState`](msg/GraspState.msg) | Force-PI 파지 상태 판정 | `/<config_key>/<hand_group>/grasp_state` | joint · task · compliance 데모 컨트롤러 |
| [`WbcState`](msg/WbcState.msg) | TSID 기반 WBC 파지 상태 | `/<config_key>/<hand_group>/wbc_state` | WBC 데모 컨트롤러 |
| [`PullEstimate`](msg/PullEstimate.msg) | 당김 벡터 추정 | `GraspState` · `WbcState` 에 내장 | 위와 같음 |
| [`CatchingState`](msg/CatchingState.msg) | 동적 포구 컨트롤러 상태 | `/<config_key>/catching_state` | `DemoCatchingController` |
| [`PayloadEstimate`](msg/PayloadEstimate.msg) | payload wrench · 질량 추정 + 관절공간 잔차 | `/<config_key>/payload_estimate` | joint · task · compliance · WBC 데모 컨트롤러 |
| [`ToFSnapshot`](msg/ToFSnapshot.msg) | ToF 거리 + 핑거팁 SE3 자세 스냅샷 | `/<config_key>/tof/snapshot` | task 데모 컨트롤러 (구독: `shape_estimation`) |
| [`ControllerState`](msg/ControllerState.msg) | 컨트롤러 lifecycle 상태 + 진단 필드 | `ListControllers` 응답의 원소 | `rtc_controller_manager` |
| [`CalibrationCommand`](msg/CalibrationCommand.msg) | 센서 캘리브레이션 명령 | `/hand/calibration/command` (YAML 로 변경) | 외부 · GUI (구독: `udp_hand_driver`) |
| [`CalibrationStatus`](msg/CalibrationStatus.msg) | 캘리브레이션 진행 · 완료 상태 | `/hand/calibration/status` (YAML 로 변경) | `udp_hand_driver` |
| [`DeviceStateLog`](msg/DeviceStateLog.msg) | 디바이스 상태 로그 | 현재 발행하는 노드 없음 | — |
| [`DeviceSensorLog`](msg/DeviceSensorLog.msg) | 디바이스 센서 로그 | 현재 발행하는 노드 없음 | — |

`ControllerState` 의 진단 필드를 읽는 법은 [rtc_controller_manager README](../rtc_controller_manager/README.md#rtc_cmlist_controllers-진단-필드) 가 갖는다.

### 서비스

| 타입 | 용도 | 서비스 이름 | 서버 노드 |
|---|---|---|---|
| [`ListControllers`](srv/ListControllers.srv) | 등록된 컨트롤러의 lifecycle 상태 조회 | `/rtc_cm/list_controllers` | `rtc_controller_manager` |
| [`SwitchController`](srv/SwitchController.srv) | 컨트롤러 activate / deactivate (single-active) | `/rtc_cm/switch_controller` | `rtc_controller_manager` |
| [`ResetFault`](srv/ResetFault.srv) | latched controller-local fault 해제 | `/rtc_cm/reset_fault` | `rtc_controller_manager` |
| [`ClearEstop`](srv/ClearEstop.srv) | global E-STOP 래치 해제 (`ResetFault` 와 서로를 풀지 않는다) | `/rtc_cm/clear_estop` | `rtc_controller_manager` |
| [`GraspCommand`](srv/GraspCommand.srv) | Force-PI 파지 one-shot (grasp / release) | `/<active_controller>/grasp_command` | 활성 컨트롤러 (joint · task · compliance · WBC 데모) |
| [`SetExternalWrench`](srv/SetExternalWrench.srv) | sim body 에 알려진 외력 부착 · 해제 | `/sim/set_external_wrench` | `rtc_mujoco_sim` |
| [`LaunchBall`](srv/LaunchBall.srv) | sim 공을 지정한 상태로 발사 | `/sim/launch_ball_at` | `rtc_mujoco_sim` |

## 더 읽기

- 메시지 · 서비스 계약의 문서화 규칙: [conventions.md](../agent_docs/conventions.md)
- 서비스 계약을 한 곳에만 두는 근거: [conventions-rationale.md](../docs/reference/conventions-rationale.md)

## 의존성

`std_msgs` · `geometry_msgs` · `builtin_interfaces` · `rosidl_default_generators` — [package.xml](package.xml) · MIT
