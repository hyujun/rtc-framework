# rtc_digital_twin

여러 JointState 토픽을 병합해 어떤 로봇이든 RViz2 에 실시간으로 그리는 디지털 트윈 시각화 노드. **유형**: 노드 패키지 (Python)

## 개요

YAML 로 정의한 다중 JointState 소스를 구독해 하나의 `JointState` 로 병합하고, `robot_state_publisher` 가 이를 TF 로 전개해 RViz2 에 띄운다. URDF 조인트를 자동 분류 (active / passive_mimic / passive_closed_chain / fixed) 하고 mimic 조인트 위치를 계산하며, 필수 조인트 커버리지를 주기적으로 검증한다. 선택 기능으로 핑거팁 센서 마커 (`sensor_viz`), TCP 마커 (`tcp_viz`), 수동 조작 GUI (`joint_gui_node`), loop-closed 핸드 시각화가 있다.

이 패키지는 robot-agnostic (ARCH-1) 이라 기본 YAML 은 로봇 값을 갖지 않는다. URDF · 소스 토픽 · 센서 토픽 · 핑거팁 이름 · TCP 소스는 `integrated_bringup` 의 로봇별 `digital_twin.yaml` 이 `config_file:=` 로 얹는다.

```
rtc_controller_manager                digital_twin_node             robot_state_publisher
 /rtc_cm/{group}/joint_states ──────> merge (+ mimic, URDF 검증)
 /rtc_cm/{group}/joint_states ──────>        │
                                             └─ /digital_twin/joint_states ─> TF ─> RViz2
 <active>/transforms ─> /tf 재발행           └─ (선택) sensor_viz · tcp_viz MarkerArray
```

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다. 로봇별 YAML 을 쓰는 실사용 예:

```bash
ros2 launch rtc_digital_twin digital_twin.launch.py \
    config_file:=$(ros2 pkg prefix --share integrated_bringup)/config/ur5e_p1a/digital_twin.yaml
```

YAML 없이 URDF 만 확인하려면 (소스가 없으면 모든 조인트가 0 으로 퍼블리시된다), 패키지 상대 경로나 절대 경로를 준다. 사용 가능한 경로는 [robot_descriptions](../robot_descriptions/README.md) 의 `robots/<name>/urdf/` 아래에 있다.

```bash
ros2 launch rtc_digital_twin digital_twin.launch.py \
    robot_description_package:=robot_descriptions \
    robot_description_path:=robots/ur5e_assm_v1/urdf/ur5e_with_hand.urdf.xacro \
    use_joint_gui:=true

ros2 launch rtc_digital_twin digital_twin.launch.py \
    robot_description_file:=/path/to/robot.urdf use_rviz:=false display_rate:=30.0
```

## 인터페이스

### 실행파일 · launch 인자

실행파일은 `digital_twin_node`, `joint_gui_node` 이고 launch 는 [digital_twin.launch.py](launch/digital_twin.launch.py) 하나다. launch 는 `robot_description` 문자열을 `robot_state_publisher` (namespace `digital_twin`) 와 `digital_twin_node` 에 함께 넘긴다. YAML 에 `robot_description_*` 를 두면 인자를 생략할 수 있고, 둘 다 없으면 launch 가 오류를 낸다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `robot_description_package` | `''` | URDF 가 든 패키지 |
| `robot_description_path` | `''` | 패키지 안 상대 경로 (`.xacro` 면 자동 처리) |
| `robot_description_file` | `''` | 절대 경로 (패키지 방식의 대안) |
| `config_file` | `''` (내장 기본값) | digital_twin YAML |
| `use_rviz` | `true` | RViz2 실행 |
| `rviz_config` | `''` (내장 기본값) | RViz 설정 파일 |
| `display_rate` | `''` (YAML 값) | YAML 의 `display_rate` 덮어쓰기 |
| `use_joint_gui` | `false` | Joint GUI 실행 |
| `closure_path` | `''` (비활성) | Extended-URDF `<stem>.closure.yaml` 경로 (YAML 키로도 설정) |

실행되는 노드는 `robot_state_publisher` · `digital_twin_node` · `rviz2` (`use_rviz`) · `joint_gui_node` (`use_joint_gui`) · `closure_state_publisher` (`closure_path`, [rtc_urdf_bridge](../rtc_urdf_bridge/README.md#nodes) 제공) 다.

### 토픽

모든 구독의 QoS 는 RELIABLE, depth 1 이다 (`controller_tf.active_controller_topic` 은 TRANSIENT_LOCAL).

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| sub | `source_N.topic` | `sensor_msgs/JointState` | `rtc_controller_manager` 가 그룹별로 발행한 조인트 상태 ([SSoT](../rtc_controller_manager/README.md#per-group-jointstate-자동-퍼블리셔)) |
| sub | `sensor_viz.sensor_topic` (선택) | `rtc_msgs/HandSensorState` | 핑거팁 센서 |
| sub | `controller_tf.active_controller_topic` | `std_msgs/String` | 능동 컨트롤러 이름. 이를 따라 `<active>/transforms` 구독을 rewire |
| sub | `/<active>/transforms` | `tf2_msgs/TFMessage` | 능동 컨트롤러의 FK `_actual` 프레임 |
| pub | `output_topic` (`/digital_twin/joint_states`) | `sensor_msgs/JointState` | 병합 결과 (`display_rate` Hz). `closure_path` 설정 시 `/digital_twin/actuated_joint_states` 로 나가고 `closure_state_publisher` 가 이 이름으로 loop-consistent q 를 낸다 |
| pub | `/tf` | `tf2_msgs/TFMessage` | `<active>/transforms` 를 node clock 으로 restamp 해 재발행 |
| pub | `sensor_viz.marker_topic` | `visualization_msgs/MarkerArray` | 센서 마커 |
| pub | `tcp_viz.marker_topic` | `visualization_msgs/MarkerArray` | TCP 마커 |

컨트롤러는 전용 `/tf` publisher 없이 `<config_key>/transforms` 로만 `_actual` 프레임을 내므로, RViz 와 `tcp_viz` 의 tf2 lookup 이 그 프레임을 보려면 이 재발행이 필요하다 (`controller_tf.rebroadcast_enable: false` 면 순수 URDF twin). 컨트롤러 키는 토픽에서 읽으며 하드코딩하지 않는다.

### 소스 모드와 조인트 검증

- **Static 모드**: `source_N.joint_names` 에 이름을 적으면 그 조인트만 받는다. **Dynamic 모드**: 빈 배열이면 수신 메시지의 모든 조인트를 받는다.
- 데이터가 오기 전에도 URDF 의 모든 조인트를 position 0 으로 퍼블리시해 TF 트리를 유지한다.
- 시작 3초 뒤 첫 검증, 이후 10초 주기로 필수(active) 조인트 누락을 WARN 으로 알린다. 모두 갖춰지면 INFO 1회 후 누락 때만 경고한다. `closure_path` 모드에서는 loop-passive 조인트를 `closure_state_publisher` 가 채우므로 누락 WARN 을 억제한다.

### SensorVisualizer (`sensor_viz`)

`sensor_viz.sensor_topic` 이 비어 있지 않을 때만 켜진다. 마커 프레임은 `fingertip_names` 의 이름이고 수명은 100 ms 라, 이름이 URDF fingertip link 와 일치해야 RViz 가 그린다.

| 센서 | 마커 | 비고 |
|---|---|---|
| Barometer | Arrow(+Z) 8개 (2x4 격자) | 길이 = 압력 비례, 히트맵 색 |
| ToF | Arrow(+Z) 3개 | `tof_enabled: true` 일 때. 길이 = 거리 비례 |
| Force (F) | Arrow(3D) | 접촉 시에만 표시, 빨강 |
| Displacement (u) | Arrow(3D) | 접촉 시에만 표시, 시안 |
| Contact | Sphere | 빨강(접촉) / 초록(비접촉) / 회색(미보정) |

접촉 판정은 `inference_enable` 이고 `contact_flag >= 0.1` 이다. contact classifier 가 없는 로봇은 `sensor_viz.force_display_threshold > 0` (로봇 bringup YAML 에서만 설정) 이면 `|F| >= threshold` 에서 F Arrow 를 그린다.

### TcpVisualizer (`tcp_viz`)

`tcp_viz.source_topic` 이 비어 있지 않을 때 켜지며, 매 표시 주기에 tf2 로 `tcp_viz.frame_id` → `source_topic` transform 을 lookup 해 마커를 만든다. **`source_topic` 은 토픽이 아니라 TF child frame 이름**이다 (예: `tool0_actual`). `/` 가 들어간 값은 활성화 플래그로만 취급되고 child frame 은 `tool0_actual` 로 폴백한다.

`tcp_viz.frame_id` 의 기본 `""` 는 활성 컨트롤러가 `source_topic` 프레임을 발행하는 부모 (컨트롤러가 pose 를 계산한 root link) 를 쓰고, 그 transform 이 오기 전에는 마커를 내지 않는다. 값을 적으면 그 frame 이 우선한다. 마커는 TCP Sphere 와 RGB 자세 축 (R=X, G=Y, B=Z) 이고, `broadcast_tf: true` 면 조회한 pose 를 `tf_child_frame` (기본 `virtual_tcp`) 으로 재발행한다.

### JointGuiNode (`joint_gui_node`)

`use_joint_gui:=true` 또는 `joint_gui.enabled: true` 로 켜는 Qt GUI 다. `joint_state_publisher_gui` 와 달리 **degree** 단위이고, 값을 직접 입력할 수 있으며 (Enter 로 적용, limit 으로 clamp), slider 옆에 limit 범위를 보여 주고, **active 조인트만** 다룬다 (mimic · closed-chain 제외). Zero All / Center All 버튼이 있으며 `joint_gui.output_topic` (기본 `/joint_gui/joint_states`) 에 `sensor_msgs/JointState` (rad) 를 `joint_gui.publish_rate` Hz 로 낸다.

## 설정

[config/digital_twin.yaml](config/digital_twin.yaml) 이 기본값을 갖고 키마다 주석이 있다 (값은 파일을 직접 읽는다). 로봇 값은 비어 있어 로봇 bringup YAML 이 채운다. 예: `integrated_bringup/config/ur5e_p1a/digital_twin.yaml`.

| 키 | 기본값 | 의미 |
|---|---|---|
| `robot_description_package` / `_path` / `_file` | `""` | URDF 위치 (둘 중 한 방식) |
| `display_rate` | `60.0` | 병합 JointState 퍼블리시 [Hz] |
| `output_topic` | `/digital_twin/joint_states` | 병합 출력 |
| `auto_compute_mimic` | `true` | mimic 조인트를 `multiplier * master + offset` 으로 계산 |
| `closure_path` | `""` | loop-closed 핸드의 closure sidecar. 경로가 절대가 아니면 `robot_description_package` share 기준. Mimic 핸드는 비워 둔다 |
| `num_sources` | `0` | 소스 수. `source_N.topic` · `source_N.joint_names` 와 짝 |
| `controller_tf.rebroadcast_enable` | `true` | `<active>/transforms` → `/tf` 재발행 |
| `joint_gui.*` | `enabled: false` · `publish_rate: 10.0` | Joint GUI |
| `sensor_viz.*` | 센서 topic 비어 있음 | 마커 scale 과 `fingertip_names` |
| `tcp_viz.*` | `source_topic: ""` | TCP 마커 scale 과 frame |

`num_sources` 의 기본이 0 이라 단독 실행에서는 소스를 직접 지정해야 한다. 예:

```yaml
/**:
  ros__parameters:
    num_sources: 1
    source_0.topic: "/rtc_cm/<group>/joint_states"
    source_0.joint_names: [""]      # 빈 값 = 전부 수용
```

### Loop-closed 핸드 시각화 (opt-in)

linkage 핑거 같은 loop-closed 핸드는 spanning-tree URDF 와 `<stem>.closure.yaml` 짝으로 표현한다. `closure_path` 를 주면 `digital_twin_node` 는 actuated 조인트만 `/digital_twin/actuated_joint_states` 로 내보내고, `closure_state_publisher` 가 closure 구속을 풀어 loop-consistent full q 를 `/digital_twin/joint_states` 로 낸다. 결합은 토픽뿐이며 build-time 의존이 없다 (ARCH-2).

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| RViz 에서 로봇이 전부 0 자세 | 소스가 없다 (`num_sources: 0`) 또는 `source_N.topic` 에 데이터가 없다 | `config_file` 로 bringup YAML 을 준다. `ros2 topic echo` 로 소스 토픽을 확인한다 |
| 센서 마커가 안 보인다 | `fingertip_names` 가 URDF fingertip link 와 다르다 | 이름을 URDF link 접두사와 맞춘다 |
| TCP 마커가 안 나온다 | `_actual` 프레임이 `/tf` 에 없다 | `controller_tf.rebroadcast_enable: true` 와 능동 컨트롤러의 `<config_key>/transforms` 를 확인한다 |
| "missing joints" WARN 반복 | 소스 `joint_names` 가 URDF active 조인트를 다 덮지 못한다 | 소스를 추가하거나 `joint_names` 를 고친다 |

## 더 읽기

- [rtc_controller_manager README](../rtc_controller_manager/README.md#per-group-jointstate-자동-퍼블리셔) — `/rtc_cm/{group}/joint_states` 발행 쪽
- [rtc_urdf_bridge README](../rtc_urdf_bridge/README.md#nodes) — `closure_state_publisher`
- [architecture.md](../agent_docs/architecture.md) — controller transforms 와 TF 흐름
- 클래스 API (`UrdfParser`, `JointStateCache`) 는 `rtc_digital_twin/*.py` docstring

## 의존성

[package.xml](package.xml) — `rclpy` · `sensor_msgs` · `visualization_msgs` · `tf2_ros` · `rtc_msgs` (`HandSensorState`) 가 핵심이고, 실행 시 `robot_state_publisher` · `rviz2` · `xacro` · `python_qt_binding` 이 필요하다. MIT.
