# ur5e_bringup

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md) | RT 최적화: [RT_OPTIMIZATION.md](../docs/RT_OPTIMIZATION.md)

## 개요

UR5e 로봇을 위한 **launch, 설정, 데모 컨트롤러** 통합 패키지입니다. 실제 로봇과 MuJoCo 시뮬레이션 모드를 지원하며, CPU 격리, DDS 스레드 핀닝, 세션 기반 로깅을 자동으로 설정합니다.

**핵심 기능:**
- 2개 데모 컨트롤러 (DemoJointController, DemoTaskController)
- 3개 launch 파일 (robot, sim, hand)
- 2개 GUI 도구 (컨트롤러 튜닝, 모션 에디터)
- 자동 CPU 격리 + DDS 스레드 핀닝
- 세션 디렉토리 자동 생성 및 정리

---

## 패키지 구조

```
ur5e_bringup/
├── CMakeLists.txt
├── package.xml
├── include/ur5e_bringup/controllers/
│   ├── demo_joint_controller.hpp       ← 관절 공간 P 제어 (로봇+핸드)
│   └── demo_task_controller.hpp        ← 태스크 공간 CLIK 제어 (로봇+핸드)
├── src/
│   ├── ur5e_rt_controller_main.cpp     ← UR5e용 진입점
│   └── controllers/
│       ├── controller_registration.cpp ← 데모 컨트롤러 등록
│       ├── demo_joint_controller.cpp
│       └── demo_task_controller.cpp
├── config/
│   ├── ur5e_robot.yaml                 ← 실제 로봇 RTC 프레임워크 설정
│   ├── ur5e_sim.yaml                   ← 시뮬레이션 전용 설정
│   └── controllers/
│       ├── demo_joint_controller.yaml  ← DemoJoint 게인/토픽
│       └── demo_task_controller.yaml   ← DemoTask 게인/토픽
├── launch/
│   ├── robot.launch.py                 ← 실제 UR5e 로봇 launch
│   ├── sim.launch.py                   ← MuJoCo 시뮬레이션 launch
│   └── hand.launch.py                  ← 핸드 UDP 드라이버 launch
└── scripts/
    ├── demo_controller_gui.py          ← 컨트롤러 튜닝 GUI (tkinter)
    └── motion_editor_gui.py            ← 모션 에디터 GUI (PyQt5)
```

---

## 데모 컨트롤러

### DemoJointController (Index 4)

관절 공간 Quintic 궤적 생성기 — UR5e 6-DOF 로봇 암 + 10-DOF 핸드 통합 제어기입니다. Rest-to-rest quintic 다항식으로 부드러운 궤적을 생성합니다.

**궤적 계산:**

```
duration = max(min_duration, max(T_speed, T_velocity))
  T_speed    = max_joint_dist / trajectory_speed
  T_velocity = max_joint_dist × (15/8) / max_traj_velocity
position_output = quintic(t, q_start, q_goal, duration)
velocity_clamped = clamp(velocity, ±max_velocity)
```

**파라미터:**

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `robot_trajectory_speed` | `0.5` rad/s | 로봇 궤적 속도 |
| `hand_trajectory_speed` | `0.5` rad/s | 핸드 궤적 속도 |
| `robot_max_traj_velocity` | `3.14` rad/s | 로봇 최대 관절 속도 |
| `hand_max_traj_velocity` | `2.0` rad/s | 핸드 최대 관절 속도 |
| `command_type` | `"position"` | 출력 타입 |

**게인 업데이트 레이아웃 (4개 요소):**
`[robot_trajectory_speed, hand_trajectory_speed, robot_max_traj_velocity, hand_max_traj_velocity]`

---

### DemoTaskController (Index 5)

태스크 공간 CLIK + 핸드 Quintic 궤적 — 감쇠 의사역행렬과 영공간 보조 태스크를 사용합니다.

**로봇 암 제어 법칙 (3-DOF 모드):**

```
pos_error = x_des - FK(q)
J^# = J^T (J J^T + λ²I)^{-1}
N   = I - J^# J
dq  = kp × J^# × pos_error + null_kp × N × (q_null - q)
q_cmd = q + clamp(dq, ±v_max) × dt
```

**타겟 입력 형식:**

| 모드 | target[0:3] | target[3:6] |
|------|------------|-------------|
| `control_6dof=false` | TCP 위치 (x,y,z) | 영공간 관절 레퍼런스 |
| `control_6dof=true` | TCP 위치 (x,y,z) | 자세 (roll, pitch, yaw) |

**게인:**

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `kp_translation` | `[15.0, 15.0, 15.0]` | 위치 비례 게인 (x, y, z) [1/s] |
| `kp_rotation` | `[5.0, 5.0, 5.0]` | 자세 비례 게인 (rx, ry, rz) [1/s] |
| `damping` | `0.01` | 의사역행렬 감쇠 계수 |
| `null_kp` | `0.5` | 영공간 관절 센터링 게인 |
| `enable_null_space` | `false` | 영공간 활성화 |
| `control_6dof` | `true` | 6-DOF 제어 활성화 |
| `trajectory_speed` | `0.1` m/s | TCP 병진 궤적 속도 |
| `trajectory_angular_speed` | `0.5` rad/s | TCP 회전 궤적 속도 (6-DOF) |
| `hand_trajectory_speed` | `1.0` rad/s | 핸드 궤적 속도 |
| `max_traj_velocity` | `0.5` m/s | 최대 TCP 병진 속도 |
| `max_traj_angular_velocity` | `1.0` rad/s | 최대 TCP 각속도 |
| `hand_max_traj_velocity` | `2.0` rad/s | 핸드 최대 속도 |

**게인 업데이트 레이아웃 (16개 요소):**
`[kp_trans×3, kp_rot×3, damping, null_kp, enable_null(0/1), control_6dof(0/1), traj_speed, traj_angular_speed, hand_traj_speed, max_vel, max_angular_vel, hand_max_vel]`

**E-STOP:** 안전 위치 `[0, -1.57, 1.57, -1.57, -1.57, 0]` rad로 이동

---

## Launch 파일

### robot.launch.py — 실제 로봇

```bash
ros2 launch ur5e_bringup robot.launch.py robot_ip:=192.168.1.10
ros2 launch ur5e_bringup robot.launch.py use_mock_hardware:=true  # 모의 테스트
```

**Launch 인자:**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `robot_ip` | `192.168.1.10` | UR 로봇 IP |
| `use_mock_hardware` | `false` | 모의 하드웨어 사용 |
| `use_cpu_affinity` | `true` | CPU 격리 + DDS 핀닝 활성화 |

**Launch 순서:**

1. 세션 디렉토리 생성 (`~/rtc_framework/logging_data/YYMMDD_HHMM/`, 최근 10개 유지)
2. 환경 변수 설정 (`CYCLONEDDS_URI` → 성능 최적화 XML, `RMW_IMPLEMENTATION`, `UR5E_SESSION_DIR`)
3. `cpu_shield.sh on --robot` — CPU 격리 활성화
4. UR 드라이버 launch (`ur_robot_driver`)
5. Hand UDP 노드 launch (`ur5e_hand_driver/hand_udp_node`, 3초 지연)
6. RT 컨트롤러 노드 launch (`ur5e_rt_controller` + `ur5e_hand_status_monitor` 설정)
7. UR 드라이버 CPU 핀닝 → Core 0-1 (3초 지연)
8. RT 컨트롤러 DDS 스레드 핀닝 → Core 0-1 (5초 지연, 비-SCHED_FIFO 스레드만)

**CycloneDDS 최적화** (`cyclone_dds.xml`):
멀티캐스트 비활성화, 소켓 버퍼 확대(recv 8MB/send 2MB), write batching(8μs), NACK 지연 최소화(10ms), 동기 전달 활성화. 상세: [`rtc_controller_manager` README](../rtc_controller_manager/README.md#설정-파일-상세)

---

### sim.launch.py — MuJoCo 시뮬레이션

```bash
ros2 launch ur5e_bringup sim.launch.py
ros2 launch ur5e_bringup sim.launch.py sim_mode:=sync_step enable_viewer:=true
ros2 launch ur5e_bringup sim.launch.py sim_mode:=sync_step enable_viewer:=true
```

**Launch 인자:**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `model_path` | (YAML) | MuJoCo scene.xml 경로 |
| `sim_mode` | (YAML) | `free_run` 또는 `sync_step` |
| `enable_viewer` | (YAML) | MuJoCo 뷰어 활성화 |
| `sync_timeout_ms` | (YAML) | sync 커맨드 타임아웃 (ms) |
| `max_rtf` | (YAML) | 최대 실시간 배율 |
| `use_yaml_servo_gains` | (YAML) | YAML vs XML 서보 게인 사용 |
| `kp`, `kd` | (YAML) | PD 게인 오버라이드 |
| `use_cpu_affinity` | `true` | Tier 1 CPU 격리 + MuJoCo 핀닝 |
| `max_log_sessions` | `10` | 세션 폴더 최대 보관 수 |

**Launch 순서:**

1. 세션 디렉토리 생성
2. `cpu_shield.sh on --sim` — 경량 CPU 격리
3. MuJoCo 시뮬레이터 노드 launch
4. RT 컨트롤러 노드 launch
5. MuJoCo 시뮬레이터 코어 핀닝 (2초 지연)

---

### hand.launch.py — 핸드 드라이버

```bash
ros2 launch ur5e_bringup hand.launch.py target_ip:=192.168.1.2
```

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `target_ip` | `192.168.1.2` | 핸드 컨트롤러 IP |
| `target_port` | `55151` | UDP 포트 |
| `publish_rate` | `100.0` | 퍼블리시 주파수 (Hz) |

---

## 로봇 vs 시뮬레이션 모드 비교

| 항목 | Robot 모드 | Sim 모드 |
|------|-----------|---------|
| 관절 상태 소스 | UR 드라이버 | MuJoCo 시뮬레이터 |
| 핸드 소스 | UDP (실제 하드웨어) | MuJoCo 에코백 (선택) |
| 커맨드 대상 | UR5e 로봇 | MuJoCo 물리 엔진 |
| CPU 격리 | Tier 1+2 (`--robot`) | Tier 1 (`--sim`) |
| DDS 핀닝 | UR 드라이버 + RT 컨트롤러 | MuJoCo 시뮬레이터 |
| 설정 | `ur5e_robot.yaml` | `ur5e_sim.yaml` + `mujoco_simulator.yaml` |

---

## GUI 도구

### demo_controller_gui

실시간 컨트롤러 선택, 게인 튜닝, 상태 모니터링 GUI입니다 (tkinter).

```bash
ros2 run ur5e_bringup demo_controller_gui
```

- 컨트롤러 전환 (DemoJoint / DemoTask)
- 관절/태스크 공간 타겟 설정
- 로봇 + 핸드 게인 편집기
- 핸드 모터 슬라이더 (Thumb, Index, Middle, Ring)
- 실시간 TCP 위치, 관절 위치, E-STOP 상태 표시

### motion_editor_gui

모션 시퀀스 편집기입니다 (PyQt5, Catppuccin Mocha 테마).

```bash
ros2 run ur5e_bringup motion_editor_gui
```

- 다중 탭 인터페이스 (모션 파일 동시 편집)
- 포즈 테이블 (이름, UR5e/핸드 프리뷰, 궤적 시간, 대기 시간)
- JSON 모션 파일 로드/저장
- 포즈 삽입/삭제/복사/붙여넣기
- 선택 모션 재생

---

## 전역 설정 (`ur5e_robot.yaml`)

```yaml
/**:
  ros__parameters:
    control_rate: 500.0
    initial_controller: "demo_joint_controller"
    init_timeout_sec: 30.0          # 하드웨어 초기화 타임아웃
    auto_hold_position: true
    enable_estop: true
    device_timeout_names: ["ur5e", "hand"]
    device_timeout_values: [1000.0, 1000.0]  # ms
    enable_logging: true
    enable_timing_log: true
    enable_device_log: true
    max_log_sessions: 10
    enable_status_monitor: false

    devices:
      ur5e:
        joint_state_names: [shoulder_pan_joint, ..., wrist_3_joint]  # 6
        joint_command_names: [shoulder_pan_joint, ..., wrist_3_joint]
        urdf:
          package: "ur5e_description"
          path: "robots/ur5e/urdf/ur5e.urdf"
          root_link: "base_link"
          tip_link: "flange"
        joint_limits:
          max_velocity: [2.0, 2.0, 3.0, 3.0, 3.0, 3.0]
          max_acceleration: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
          max_torque: [150, 150, 150, 28, 28, 28]
      hand:
        joint_state_names: [thumb_cmc_aa, ..., ring_mcp_fe]  # 10
        motor_state_names: [motor_1, ..., motor_10]          # 10
        sensor_names: [thumb, index, middle, ring]            # 4
        joint_limits:
          max_velocity: [1.0, ..., 1.0]
          position_lower: [0.0, ..., 0.0]
          position_upper: [1.57, ..., 1.57]
```

**시뮬레이션 설정 (`ur5e_sim.yaml`) 차이점:**
- `init_timeout_sec: 0.0` (비활성화), `enable_estop: false`
- `use_sim_time_sync: true`, `sim_sync_timeout_sec: 5.0`
- `device_timeout_values: [10000.0, 10000.0]` (시작 시 여유)
- 핸드에 `motor_state_names` 없음 (MuJoCo가 직접 position 제공)

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `ament_cmake` | 빌드 시스템 |
| `rclcpp` | ROS2 클라이언트 |
| `rtc_controller_interface` | 컨트롤러 추상 인터페이스 |
| `rtc_controllers` | 내장 컨트롤러 |
| `rtc_controller_manager` | RT 제어 루프 |
| `rtc_base` | 타입, 스레딩 |
| `rtc_msgs` | 커스텀 메시지 |
| `pinocchio` | 기구학 (FK, Jacobian) |
| `yaml-cpp` | YAML 파싱 |
| `sensor_msgs` | JointState |
| `std_msgs` | 표준 메시지 |
| `rclpy` | Python GUI (exec) |
| `rtc_scripts` | CPU 격리 스크립트 (exec) |
| `ur5e_description` | URDF/MJCF 모델 (exec) |
| `ur5e_hand_driver` | 핸드 드라이버 (exec) |
| `ur5e_hand_status_monitor` | 핸드 상태 모니터 (exec) |
| `PyQt5` | 모션 에디터 GUI (exec) |

---

## 빌드

```bash
cd ~/ur_ws
colcon build --packages-select ur5e_bringup
source install/setup.bash
```

**빌드 산출물:**
- 정적 라이브러리: `libur5e_bringup_demo_controllers.a` (`--whole-archive` 링크)
- 실행 파일: `ur5e_rt_controller`
- Python 스크립트: `demo_controller_gui`, `motion_editor_gui`

---

## 의존성 그래프 내 위치

```
rtc_controller_manager + rtc_controllers + rtc_scripts + ur5e_description
    ↓
ur5e_bringup  ← UR5e 로봇별 통합 패키지
    │
    ├── robot.launch.py  → UR 드라이버 + RT 컨트롤러 + CPU 격리
    ├── sim.launch.py    → MuJoCo + RT 컨트롤러 + CPU 격리
    ├── DemoJointController (index 4)
    ├── DemoTaskController (index 5)
    ├── demo_controller_gui
    └── motion_editor_gui
```

---

## 변경 내역

### v5.17.0

| 영역 | 변경 내용 |
|------|----------|
| **DemoJointController** | P 제어 → Quintic rest-to-rest 궤적 생성기로 변경, trajectory_speed/max_traj_velocity 파라미터 |
| **DemoTaskController** | `kp_translation`/`kp_rotation` 분리 게인, 궤적 속도/각속도 파라미터 추가 |
| **토픽 설정** | YAML 기반 동적 토픽 라우팅 (`topics:` 블록), motor_state/sensor_state 구독 추가 |
| **디바이스 설정** | `devices:` 계층 구조 (ur5e/hand), `motor_state_names`/`sensor_names`/`joint_command_names` 추가 |
| **세션 관리** | `UR5E_SESSION_DIR` 환경변수, `max_log_sessions` 파라미터, 자동 세션 정리 |
| **robot.launch.py** | Hand UDP 노드 자동 launch, `ur5e_hand_status_monitor` 설정 로드 |
| **sim.launch.py** | `sync_timeout_ms`, `use_yaml_servo_gains`, `max_log_sessions` 인자 추가 |

### v5.16.1

| 영역 | 변경 내용 |
|------|----------|
| **코드 검증** | C++20, 컴파일러 경고 플래그, RT-safety 확인 완료 — 이미 적용됨 |

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
