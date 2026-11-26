# rtc_teleop_input

운영자 입력을 표준 게임패드 배치의 `sensor_msgs/Joy` 로 내는 도구. 키보드 · 마우스로 조작하는 virtual 패드 GUI 와 실물 패드 (joy 의 `game_controller_node`) 중 하나를 쓰고, 패드 버튼으로 controller manager 의 서비스 (E-STOP · `reset_fault` · 전환 · E-STOP 해제) 와 `std_srvs/Trigger` 를 부른다. **유형**: 노드 패키지 (Python, Qt)

## 개요

입력 노드는 어떤 입력이든 같은 모양의 Joy 를 고정 주기 (기본 50 Hz) 로 낸다. 배열 순서는 [`config/standard_gamepad.yaml`](config/standard_gamepad.yaml) 이 정하며 SDL GameController 순서 (axes 6 · buttons 21) 다. 그래서 실물 패드의 Joy 는 그대로 중계된다. `header.frame_id` 는 입력 출처 (`virtual` · `pad`) 다.

```
키보드 · 마우스 ─> virtual pad ─┐
                                ├─> teleop_input ─> Joy (joy_topic, 50 Hz)
실물 패드 ─> game_controller_node ┘        └─> /rtc_cm/* 서비스 · Trigger 서비스
```

Joy 를 어떻게 쓰는지 (축 → 명령, deadman) 는 소비하는 컨트롤러가 정한다. 이 패키지는 로봇 · 토픽 · 컨트롤러 이름을 기본값으로 갖지 않는다.

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다.

```bash
# virtual 패드 (기본 skin F710)
ros2 launch rtc_teleop_input teleop_input.launch.py joy_topic:=<topic>

# 실물 패드 + reset 버튼 + 상태 패널
ros2 launch rtc_teleop_input teleop_input.launch.py joy_topic:=<topic> input:=pad skin:=xbox \
    controller_name:=<controller> status_topics:=<status topic>
```

창을 닫으면 발행이 멈춘다. 소비자는 입력이 끊긴 것 (stale) 으로 처리한다.

## 인터페이스

| 이름 | 종류 | 설명 |
|---|---|---|
| `joy_topic` (`output_topic`) | pub `sensor_msgs/Joy` | 출력. depth 1, `rate_hz` 마다 한 메시지 |
| `pad_topic` (기본 `joy`) | sub `sensor_msgs/Joy` | `input:=pad` 의 상류. launch 가 `game_controller_node` 출력을 여기로 remap |
| `/rtc_cm/trigger_estop` | client `std_srvs/Trigger` | E-STOP 버튼을 누를 때마다 한 번 |
| `/rtc_cm/reset_fault` | client `rtc_msgs/ResetFault` | reset 버튼을 `reset_hold_s` 동안 hold 하면 한 번 |
| `switch_service` · `clear_estop_service` · `trigger_buttons` | client `SwitchController` · `ClearEstop` · `std_srvs/Trigger` | 그 hold 버튼마다 hold 한 번에 한 번 |
| `active_controller_topic` · `estop_status_topic` | sub `String` · `Bool` (latched) | 창에 활성 컨트롤러 · E-STOP |
| `status_topics` | sub `diagnostic_msgs/DiagnosticStatus` | 상태 패널에 키/값 · level 표시 |

기본 버튼과 키:

| 동작 | 패드 | 키보드 · 마우스 (virtual) |
|---|---|---|
| 왼쪽 스틱 (`leftx` · `lefty`) | 스틱 | `W` `S` `A` `D` (기울임 `deflection` = 0.25) · 스틱 끌기 |
| 오른쪽 스틱 X (`rightx`) | 스틱 | `←` `→` · 스틱 끌기 |
| deadman (`leftshoulder`) | LB 누르는 동안 | `Space` 누르는 동안 · LB 를 마우스로 누르고 있기 |
| E-STOP (`b`) | B 누름 | `Esc` · 창의 E-STOP 버튼 (`pad` 에서도 동작) |
| reset_fault (`back`) | BACK 1 s | `Backspace` 1 s |
| help | `help_combo` (기본 없음) | `F1` (`help_key`) |

전환 · E-STOP 해제 · Trigger 버튼은 기본으로 없고 소비자가 `params_file` 로 준다. hold 버튼은 deadman · 다른 hold 버튼이 눌려 있으면 처음부터 다시 세고, E-STOP 을 누르면 다시 누를 때까지 보내지 않는다 (규칙 전문: [`model.py`](rtc_teleop_input/model.py) 서두). help 는 패드 그림에 번호를, 그 아래 표에 기능 · 키를 보인다. `help_combo` 는 `input:=pad` 에서만 동작한다.

- 키의 자동 반복은 무시한다. 창이 포커스를 잃으면 모든 키 · 스틱 · 버튼을 놓고, 다시 누를 때까지 놓인 채다.
- `pad` 에서 상류가 `pad_timeout_s` (0.25 s) 넘게 조용하거나 크기가 다른 Joy 가 오면 놓은 상태를 낸다. 창에는 "pad lost" 가 뜬다.
- 서비스가 떠 있지 않으면 호출하지 않고 창에 표시한다. 응답은 창의 "last reply" 에 나온다.
- 창의 E-STOP 은 마우스를 누르는 순간 보낸다. 상태 패널의 토픽이 2 s 넘게 조용하면 그 행은 STALE 로 바뀐다.

## 설정

노드 파라미터 전체와 기본값은 [`model.py`](rtc_teleop_input/model.py) 의 `TeleopConfig` 다. launch 인자에 없는 것 (키 배치 · 기울임 · 버튼 역할 · 시간) 은 `params_file:=<yaml>` 로 준다. `input` · `output_topic` · `pad_topic` 은 launch 인자로만 준다 (파일에 있으면 launch 가 거부한다).

```yaml
teleop_input:
  ros__parameters:
    deflection: 0.4
    key_axes: ["lefty+:Up", "lefty-:Down", "leftx+:A", "leftx-:D", "rightx+:Q", "rightx-:E"]
    key_buttons: ["leftshoulder:Space", "b:Escape", "back:Backspace", "y:1"]
    switch_buttons: ["y:<controller>"]
    help_labels: ["lefty:forward / back"]
```

키 이름은 Qt 의 `Qt::Key_<이름>` 이다. 스틱 누름 (`leftstick` · `rightstick`) 은 마우스로 누를 수 없으니 필요하면 `key_buttons` 에 키를 단다. `rate_hz` 는 (0, 1000] 이고 주기가 정수 ms 여야 한다 (50 · 100 · 250 …). `controller_name` 이 비면 reset 은 꺼진다. 잘못된 값이면 노드가 시작하지 않는다.

## 문제 해결

| 증상 | 확인 |
|---|---|
| `input:=pad` 인데 계속 "pad lost" | `ros2 topic echo <pad_topic>` 가 조용하면 장치 권한: `getfacl /dev/input/event<N>` 에 현재 사용자가 있는지 (SSH 세션에는 붙지 않는다). `ros2 run joy joy_enumerate_devices` 로 열거 |
| F710 이 열거되지 않음 | 뒷면 스위치를 X (XInput) 로 |
| 키가 듣지 않음 | 창이 포커스를 가졌는지. 다른 창으로 갔다 오면 키를 다시 누른다 |
| `package 'joy' not found` | `rosdep install --from-paths src --ignore-src -y` (install.sh 가 돌린다) |

## 의존성

`rclpy` · `sensor_msgs` · `std_msgs` · `std_srvs` · `rtc_msgs` · `builtin_interfaces` · `diagnostic_msgs` · `python_qt_binding` (ROS apt) · `joy` — [package.xml](package.xml).
