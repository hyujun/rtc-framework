# RTC (Real-Time Control) Framework

![CI](https://github.com/hyujun/rtc-framework/actions/workflows/ros2-advanced-ci.yml/badge.svg)
[![codecov](https://codecov.io/gh/hyujun/rtc-framework/branch/main/graph/badge.svg?token=4Synh4Gk1v)](https://codecov.io/gh/hyujun/rtc-framework)
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-green)

**Ubuntu 22.04 (ROS 2 Humble) / Ubuntu 24.04 (ROS 2 Jazzy) | 모듈형 rtc_* 프레임워크 기반 UR5e 실시간 제어 시스템**

로봇 비의존적(robot-agnostic) RTC 프레임워크 위에 구축된 실시간 제어 솔루션입니다. 가변 DOF, 설정 가능한 RT 루프 주기 (`control_rate` YAML; 설계 범위 100Hz–5kHz, default 500Hz), 전략 패턴 기반 다중 제어기(P/JointPD/CLIK/OSC/TSID-WBC), TSID QP 전신 제어 (`rtc_tsid` + ProxSuite), **MPC↔RT 인터페이스 계층 (`rtc_mpc`: zero-copy TripleBuffer + cubic-Hermite 보간 + Riccati 피드백, dedicated-core MPC thread)**, 전송 계층 추상화(UDP/CAN-FD/EtherCAT/RS485), RT-안전 ONNX 추론 엔진, MuJoCo 3.x 물리 시뮬레이터, E-STOP 안전 시스템, CSV 데이터 로깅, GUI 도구를 포함합니다.

---

## 패키지 구성

20개 ROS 2 패키지로 구성되어 있으며, 로봇 비의존적 프레임워크(`rtc_*`), 형상 추정(`shape_estimation_*`), 멀티-로봇 데이터/통합 패키지로 분리됩니다. 각 패키지는 자체 `README.md`를 포함하며, 패키지 버전 핀은 README 에서 의도적으로 생략합니다 (drift 방지 — 정확한 버전은 각 `package.xml` 참조).

### 로봇 비의존적 프레임워크 (rtc_*)

| 패키지 | 설명 | 빌드 |
|--------|------|------|
| [`rtc_msgs`](rtc_msgs/) | 커스텀 ROS 2 메시지 14종 (JointCommand, FingertipSensor, HandSensorState, RobotTarget, DeviceStateLog, DeviceSensorLog, GraspState, WbcState, ToFSnapshot, ControllerState, CalibrationCommand, CalibrationStatus, SimSensor, SimSensorState) | ament_cmake |
| [`rtc_base`](rtc_base/) | 헤더-전용 RT 인프라: 타입, SeqLock, SPSC 버퍼, 스레딩(4/6/8/10/12/16코어 + MPC tier `MpcThreadConfig`), Bessel/Kalman 필터, CSV 로깅 | ament_cmake |
| [`rtc_communication`](rtc_communication/) | 헤더-전용 전송 계층 추상화: TransportInterface, UdpSocket RAII, PacketCodec concept, Transceiver 템플릿 | ament_cmake |
| [`rtc_controller_interface`](rtc_controller_interface/) | 추상 컨트롤러 인터페이스 (Strategy 패턴) + Singleton 레지스트리 (가변 DOF) | ament_cmake |
| [`rtc_controllers`](rtc_controllers/) | 범용 제어기 4종 (P, JointPD, CLIK, OSC) + 퀸틱 궤적 생성기 | ament_cmake |
| [`rtc_tsid`](rtc_tsid/) | TSID QP 프레임워크: WQP/HQP formulation, PostureTask/SE3Task/CoMTask/ForceTask, EOM/Contact/FrictionCone/TorqueLimit/JointLimit 제약, ProxSuite 백엔드 | ament_cmake |
| [`rtc_mpc`](rtc_mpc/) | MPC↔RT 인터페이스: zero-copy `TripleBuffer<T>` (single-atomic publish/acquire), cubic-Hermite `TrajectoryInterpolator`, `RiccatiFeedback`, `MPCSolutionManager` facade, `MPCThread`+`MockMPCThread` jthread skeleton (solver-agnostic; Aligator는 후속 패키지) | ament_cmake |
| [`rtc_controller_manager`](rtc_controller_manager/) | 설정 가능한 RT 루프 (`control_rate`, default 500Hz, clock_nanosleep) + 컨트롤러 라이프사이클 + SPSC publish offload + E-STOP + `DeviceBackend` 추상 | ament_cmake |
| [`rtc_inference`](rtc_inference/) | 헤더-전용 RT-안전 추론 엔진: ONNX Runtime IoBinding, 사전 할당 버퍼, 배치/다중 모델 | ament_cmake |
| [`rtc_mujoco_sim`](rtc_mujoco_sim/) | MuJoCo 3.x 물리 시뮬레이터: 멀티 그룹 물리, GLFW 뷰어, fake_hand 1차 필터, `max_rtf` 속도 제어, `n_substeps` 서브스텝 | ament_cmake |
| [`rtc_digital_twin`](rtc_digital_twin/) | RViz2 디지털 트윈 시각화: 다중 소스 관절 상태 통합, mimic 자동 계산, 핑거팁 센서 Arrow/Sphere 마커 | ament_python |
| [`rtc_tools`](rtc_tools/) | Python 유틸리티 7종: controller_gui, plot_rtc_log, compare_mjcf_urdf, urdf_to_mjcf, hand_udp_sender, hand_data_plot, session_dir | ament_python |
| [`repo_scripts`](repo_scripts/) | RT 시스템 설정 스크립트 (PREEMPT_RT 커널, CPU 격리, IRQ 어피니티, 네트워크 최적화, `setup_env.sh`, MPC 코어 헬퍼 `get_mpc_cores`/`get_rt_cores`/`get_os_cores`) — shell-only (CMake 빌드 없음) |

### 브릿지 패키지

| 패키지 | 설명 | 빌드 |
|--------|------|------|
| [`rtc_urdf_bridge`](rtc_urdf_bridge/) | 로봇 비의존적 URDF 파서 + Pinocchio 모델 빌더, YAML 기반 체인 추출 설정 | ament_cmake |

### 형상 추정 패키지 (shape_estimation_*)

| 패키지 | 설명 | 빌드 |
|--------|------|------|
| [`shape_estimation_msgs`](shape_estimation_msgs/) | ToF 기반 형상 추정용 커스텀 ROS 2 메시지 3종 (ToFReadings, TipPoses, ShapeEstimate) + ExploreShape 액션 | ament_cmake |
| [`shape_estimation`](shape_estimation/) | ToF 센서 기반 형상 추정: 복셀 포인트 누적, 최소제곱 프리미티브 피팅 (구/실린더/평면/박스) | ament_cmake |

### 멀티-로봇 데이터 + 통합 패키지

| 패키지 | 설명 | 빌드 |
|--------|------|------|
| [`robot_descriptions`](robot_descriptions/) | Robot-agnostic data hub — `robots/<name>/` 당 URDF/MJCF/mesh (현재: ur5e, ur5e_assm_v1, assm_v1, iiwa7, iiwa7_leap, leap_hand, schunk_hand). data-only 패키지 — 소비자는 `<exec_depend>` + ament_index 런타임 lookup만 사용 (ARCH-5) | ament_cmake |
| [`udp_hand_driver`](udp_hand_driver/) | 가변-DOF 핸드 UDP 드라이버: SeqLock 상태, ppoll sub-ms 타임아웃, 촉각 센서, ONNX F/T 추론 | ament_cmake |
| [`ur5e_bt_coordinator`](ur5e_bt_coordinator/) | BehaviorTree.CPP v4 기반 비-RT 태스크 코디네이터 (20 Hz, UR5e + 핸드 통합 모션) | ament_cmake |
| [`integrated_bringup`](integrated_bringup/) | per-robot launch/config (`config/<robot>/`: ur5e_hand, iiwa7_leap, ...) + 데모 컨트롤러 (DemoJoint, DemoTask, DemoWbc — TSID QP 기반 8-phase WBC + MPC 통합 경로, `enable_mpc` launch arg) + CPU 격리/DDS 핀닝 | ament_cmake |

### 의존성 그래프

```
rtc_msgs, rtc_base (독립)
  ├── rtc_communication ← rtc_base
  ├── rtc_inference ← rtc_base
  ├── rtc_controller_interface ← rtc_base, rtc_msgs, rtc_urdf_bridge
  │   ├── rtc_controllers ← rtc_controller_interface
  │   └── rtc_controller_manager ← rtc_controller_interface, rtc_communication
  ├── rtc_tsid ← Pinocchio, ProxSuite, Eigen3, yaml-cpp
  ├── rtc_mpc  ← rtc_base, Eigen3, yaml-cpp
  ├── rtc_mujoco_sim ← MuJoCo 3.x (optional)
  ├── rtc_digital_twin (독립, Python)
  └── rtc_tools (독립, Python)

repo_scripts (shell-only, no CMake)
rtc_urdf_bridge ← Pinocchio, tinyxml2, yaml-cpp

shape_estimation_msgs (독립)
  └── shape_estimation ← shape_estimation_msgs, Eigen3

robot_descriptions (독립, data-only — <exec_depend> only, ARCH-5)
udp_hand_driver ← rtc_communication, rtc_inference, rtc_base
ur5e_bt_coordinator ← rtc_msgs, BehaviorTree.CPP v4

integrated_bringup ← rtc_controller_manager, rtc_tsid, rtc_mpc,
                     udp_hand_driver, robot_descriptions (runtime lookup)
```

---

## 주요 기능

### RT 제어 코어
- **가변 DOF 실시간 제어**: `clock_nanosleep(TIMER_ABSTIME)` 기반 RT 루프 (`control_rate` YAML로 100Hz–5kHz 설정, default 500Hz), CPU 코어 자동 할당 (4/6/8/10/12/16코어)
- **Lock-Free SPSC 아키텍처**: RT 스레드 → SPSC 버퍼 → 비-RT 퍼블리시/로깅 (wait-free push, cache-line 정렬)
- **SeqLock 동기화**: 단일 Writer / 다중 Reader lock-free 상태 공유 (trivially copyable 타입 전용)
- **컨트롤러 계층 분리**: `rtc_controller_interface` (추상) → `rtc_controllers` (범용 4종) → `integrated_bringup` (데모 2종)
- **Lifecycle 관리**: 모든 C++ 노드가 `rclcpp_lifecycle::LifecycleNode` 기반 — `ros2 lifecycle` CLI로 런타임 상태 제어 (deactivate/activate), Launch event handler 기반 자동 configure→activate 체이닝

### 제어 알고리즘
- **PController**: Joint-space 비례 제어 (증분 스텝 `q + kp*error*dt`)
- **JointPDController**: PD + Pinocchio RNEA 중력/코리올리 보상, JointSpaceTrajectory 퀸틱 보간
- **ClikController**: Damped Jacobian 역운동학 (3/6-DOF), 영공간 제어, TaskSpaceTrajectory SE3 퀸틱
- **OperationalSpaceController**: 6-DOF Cartesian PD + SO(3) 회전 제어, Pinocchio log3 오차
- **DemoWbcController**: TSID QP 기반 16-DoF (arm + hand) 전신 제어, 8-phase FSM (Idle→Approach→PreGrasp→Closure→Hold→Retreat→Release→Fallback), ProxSuite Dense QP, semi-implicit Euler 적분, **Phase 5에서 MPC reference 주입 경로 지원 — `rtc_mpc`의 MockMPCThread(20 Hz) → TripleBuffer → cubic-Hermite 보간 → TSID task `q_des/v_des/a_des + u_fb` 주입, MPC 비활성 시 Phase 4 고정-reference 동작 bit-identical 유지**

### 안전 시스템
- **글로벌 E-STOP**: `atomic<bool>` + `compare_exchange_strong` 기반 통합 비상 정지 — 동적 디바이스 그룹 기반 트리거:
  - `init_timeout`: 초기화 시간 내 state 미수신 → 노드 종료
  - `{group}_timeout`: 디바이스 그룹별 state 토픽 갱신 타임아웃 (CheckTimeouts 50Hz, YAML 설정)
  - `sim_sync_timeout`: 시뮬레이션 동기화 타임아웃 (`use_sim_time_sync` 모드)
  - `consecutive_overrun`: ≥10회 연속 RT 루프 오버런
- **자동 복구**: protective_stop, 프로그램 연결 끊김에 대해 선택적 자동 복구 지원

### 시뮬레이션 & 추론
- **MuJoCo 3.x 시뮬레이터**: 동기식 루프, GLFW 인터랙티브 뷰어 (40+ 키보드 단축키), fake_hand 시뮬레이션, `max_rtf` 속도 제어, `n_substeps` 서브스텝으로 물리 해상도 조절
- **RT-안전 ONNX 추론**: IoBinding + 사전 할당 버퍼로 RT 경로 힙 할당 제거, 배치/다중 모델 지원

### 통신 & 로깅
- **전송 계층 추상화**: UDP 구현 완료, CAN-FD/EtherCAT/RS485 확장 가능
- **세션 기반 CSV 로깅**: `logging_data/YYMMDD_HHMM/` — timing, robot, device 3파일 분리
- **ROS2 파라미터 인트로스펙션**: 컨트롤러별 토픽 매핑을 읽기 전용 파라미터로 노출

### 도구
- **controller_gui**: tkinter 기반 실시간 게인 튜닝 GUI
- **plot_rtc_log**: Matplotlib CSV 궤적 시각화 (robot/device 자동 감지)
- **compare_mjcf_urdf**: MJCF/URDF 물리 파라미터 교차 검증
- **urdf_to_mjcf**: URDF → MJCF 자동 변환

---

## 빠른 시작

### 설치

```bash
chmod +x install.sh
./install.sh sim          # 시뮬레이션 전용
./install.sh robot        # 실제 로봇 전용
./install.sh full         # 전체 설치
```

#### 표준 ROS 2 toolchain 흐름 (CI · 외부 통합 환경)

`install.sh`가 만능 wrapper지만 표준 `rosdep install` → `colcon build` → `colcon test` 흐름도 직접 지원합니다.

```bash
cd ~/ros2_ws/rtc_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro=jazzy -y
colcon build --symlink-install
colcon test
```

다만 아래 의존성은 apt/rosdep에 없어 manual install path가 필요합니다 (`install.sh`가 자동 처리):

- **ONNX Runtime** (`rtc_inference` 빌드 요구) — apt에 없을 시 `/opt/onnxruntime` tarball fallback ([repo_scripts/scripts/lib/install_deps.sh](repo_scripts/scripts/lib/install_deps.sh) `install_onnxruntime`)
- **MuJoCo 3.x** (`rtc_mujoco_sim` 빌드 요구) — `/opt/mujoco-3.2.4` tarball ([repo_scripts/scripts/lib/install_deps.sh](repo_scripts/scripts/lib/install_deps.sh) `install_mujoco`)
- **MPC source-build deps** (`fmt` 11.1.4 + `mimalloc` 2.1.7 + `aligator` 0.19.0 — `rtc_mpc` 요구) — `<rtc_ws>/deps/install/`에 소스 빌드 ([repo_scripts/scripts/build_deps.sh](repo_scripts/scripts/build_deps.sh))
- **mujoco Python bindings** (`rtc_tools` urdf_to_mjcf / compare_mjcf_urdf 런타임) — `requirements.lock`에 박혀 있고 `install.sh`가 `uv pip sync`로 venv에 설치 (수동: `uv pip sync requirements.lock`)

### 빌드

```bash
chmod +x build.sh
./build.sh sim            # 시뮬레이션 패키지
./build.sh robot          # 로봇 패키지 (MuJoCo 제외)
./build.sh full           # 전체 패키지
./build.sh sim -c -j 4    # 클린 빌드 + 4 워커
./build.sh -p rtc_base    # 특정 패키지만 빌드

# 수동 빌드 (워크스페이스에 외부 패키지가 섞여 있을 때 권장)
source ~/ros2_ws/rtc_ws/src/rtc-framework/repo_scripts/scripts/setup_env.sh
deactivate 2>/dev/null   # venv 활성 시 CMake FindPython 충돌 방지
cd ~/ros2_ws/rtc_ws && colcon build --symlink-install
source install/setup.bash
```

> `setup_env.sh` 가 `RTC_DEPS_PREFIX` · ONNX Runtime · `mujoco_ROOT` · `COLCON_DEFAULTS_FILE` (`--symlink-install` / Release / `compile_commands` 자동 적용) 를 모두 export 하므로, 이후 plain `colcon build` 만으로도 의존성이 전부 발견됩니다. 단 venv 활성 상태면 `deactivate` 또는 `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` 가 필요합니다 (eigenpy/pinocchio configure 보호). 모드별 패키지 셀렉션 · `compile_commands.json` 머지 · RT 환경 점검은 `build.sh` 만 수행합니다 — 두 워크플로는 같은 `build/`·`install/` 트리를 공유하며 incremental 로 안전하게 병행할 수 있습니다 (단, `build.sh -c` 는 트리 전체를 삭제하므로 외부 패키지가 있으면 사용 금지).

### Python 의존성 sync (dev PC ↔ runtime PC 재현성)

`install.sh` 가 `uv` 를 자동 부트스트랩하여 `.venv` 를 `requirements.lock` 과 비트단위로 일치시킵니다. 정책:

- **apt 책임**: `numpy` / `scipy` / `matplotlib` / `pandas` / `PyQt5` / `rclpy` / `ament_*` — `--system-site-packages` 로 venv 가 상속 (lock 에는 박지 않음)
- **venv 책임 (lock)**: `mujoco` + transitive + `Cython` + `ruff` + `setuptools` / `wheel` — `requirements.lock` 에 sha256 hash 와 함께 박힘

```bash
# Lock 재생성 (의존성 추가/버전 변경 시)
uv pip compile requirements.in --generate-hashes \
    --no-emit-package numpy --no-emit-package scipy \
    --no-emit-package matplotlib --no-emit-package pandas \
    -o requirements.lock

# 새 머신에서 sync (install.sh 가 자동 수행, 수동:)
uv venv --system-site-packages .venv
source .venv/bin/activate
uv pip sync requirements.lock        # lock 과 정확히 일치 (extra 제거)
```

`uv pip sync` 는 `pip install -r` 과 달리 lock 에 없는 패키지를 venv 에서 제거하므로 dev PC ↔ runtime PC 간 의존성 drift 가 발생하지 않습니다 (system-site-packages 는 건드리지 않음). hash 검증으로 wheel 변조도 차단합니다.

### 실행

```bash
# MuJoCo 시뮬레이션
ros2 launch rtc_mujoco_sim mujoco_sim.launch.py

# 실제 로봇 (CycloneDDS 성능 최적화 설정 자동 로드)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch integrated_bringup robot.launch.py robot_ip:=192.168.1.10

# 가상 하드웨어 (로봇 불필요)
ros2 launch integrated_bringup robot.launch.py use_fake_hardware:=true

# 핸드 드라이버 노드
ros2 launch udp_hand_driver udp_hand.launch.py target_ip:=192.168.1.2
```

### 모니터링

```bash
ros2 topic hz /forward_position_controller/commands   # RT 루프 주기 (= 설정된 control_rate, default ~500Hz)
ros2 topic echo /system/estop_status                  # E-STOP 상태 (true = 활성)
ros2 topic echo /sim/status                           # MuJoCo: [step, time, rtf, paused]
ros2 param list /integrated_rt_controller | grep controllers  # 토픽 파라미터 확인

# RT 스레드 상태 확인 (exec 이름 = 노드 이름 = integrated_rt_controller)
PID=$(pgrep -f integrated_rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
```

---

## 아키텍처 개요

**ROS2 Humble / Jazzy** 멀티 패키지 리포지토리 (`ament_cmake`, C++20). `rtc_*` 패키지는 URDF 매니퓰레이터에 범용 적용 가능합니다.

```
[로봇 하드웨어 / MuJoCo 시뮬레이터]
    │  /joint_states (sensor_msgs/JointState)
    ▼
[integrated_rt_controller]  ←  /target_joint_positions (goal)
    │  RT 루프 (clock_nanosleep @ control_rate)
    │  제어기: rtc_controllers (P / JointPD / CLIK / OSC)
    │  전송: rtc_communication (UDP)
    ├──→ SPSC ──→ [publish_thread] ──→ /forward_position_controller/commands
    ├──→ SPSC ──→ [log_thread] ──→ CSV 3-파일 (timing, robot, device)
    └──→ E-STOP ──→ /system/estop_status

[핸드 HW] ←UDP 직접 소유→ [udp_hand_driver] ← SeqLock ← [ControlLoop]

[rtc_inference]   RT-안전 ONNX 추론 (IoBinding, 사전 할당)
```

### 스레딩 모델 (6코어 기준)

| 스레드 | 타입 | 코어 | 스케줄러 | 우선순위 | 역할 |
|--------|------|------|----------|----------|------|
| `rt_loop` | jthread (clock_nanosleep) | 2 | SCHED_FIFO | 90 | ControlLoop @ `control_rate` (default 500Hz, design 100Hz–5kHz) + CheckTimeouts 50Hz |
| `sensor_executor` | ROS2 Executor | 3 | SCHED_FIFO | 70 | /joint_states, /target_joint_positions 구독 |
| `log_executor` | ROS2 Executor | 4 | SCHED_OTHER | nice -5 | `cm_timing_log.csv` 드레인 + deferred E-STOP 로그 (Phase C 이후 controller-owned CSV 는 각 controller LifecycleNode 소유) |
| `mpc_main` (Phase 5) | jthread | 4 | SCHED_FIFO | 60 | 20 Hz MPC solve, TripleBuffer publish (6코어는 logging과 공유; 8+코어는 dedicated) |
| `publish_thread` | jthread (SPSC drain) | 5 | SCHED_OTHER | nice -3 | ROS2 publish offload (ControlPublishBuffer) |
| `aux_executor` | ROS2 Executor | 5 | SCHED_OTHER | 0 | E-STOP 상태 퍼블리시 |
| `udp_recv` | jthread | 5 | SCHED_FIFO | 65 | 핸드 UDP 수신 (udp_hand_driver) |

> Core 0–1: OS, DDS, NIC IRQ (isolcpus 대신 런타임 `cset shield` 사용). DDS 스레드는 `taskset`으로 Core 0-1에 자동 핀닝.
> CycloneDDS 성능 최적화: 멀티캐스트 비활성화, 소켓 버퍼 확대, write batching, NACK 지연 최소화.
> **MPC 스레드는 sensor_io보다 낮은 우선순위(60 < 70)를 가지므로 sensor callback이 항상 preempt — 긴 solve가 RT 루프에 영향을 주지 않음.** 12/16코어 tier는 MPC main + 1–2 worker(SCHED_FIFO 55)로 병렬 solve 지원. 8/10/12/16코어 레이아웃 및 `kMpcConfig{4,6,8,10,12,16}Core`는 `rtc_base` README 참조.

---

## 세션 기반 로깅

로그는 `logging_data/YYMMDD_HHMM/` 세션 디렉토리에 자동 저장됩니다 (`max_log_sessions: 10`):

| 서브디렉토리 | 내용 |
|---|---|
| `controller/` | `<device>_log.csv` (디바이스별: 관절/센서/추론; Phase C에서 controller-owned 경로로 이전 예정) |
| `controllers/<config_key>/` | per-controller 데이터 CSV |
| `timing/` | per-tick 스레드 타이밍 CSV (cm_timing_log, mpc_timing_log, hand_udp_timing_log — 동일 7열 RtTickTimingPayload 스키마) |
| `monitor/` | ur5e_failure_*.log, controller_stats.json |
| `hand/` | hand_udp_stats.json |
| `sim/` | screenshot_*.ppm (MuJoCo 전용) |
| `plots/`, `motions/` | rtc_tools 출력 |

환경변수 `RTC_SESSION_DIR` 로 모든 노드에 세션 경로 자동 전파.

---

## RT 권한 요구사항

```bash
sudo groupadd realtime
sudo usermod -aG realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
# 로그아웃 후 재로그인 필요
```

최대 RT 성능을 위한 CPU 격리:
```bash
# /etc/default/grub의 GRUB_CMDLINE_LINUX_DEFAULT에 추가 (6코어 기준)
# isolcpus=2-5 nohz_full=2-5 rcu_nocbs=2-5
sudo update-grub && sudo reboot
```

---

## 문서

| 문서 | 설명 |
|------|------|
| [docs/RT_OPTIMIZATION.md](docs/RT_OPTIMIZATION.md) | 실시간 최적화 가이드 (CPU 코어 할당, 커널 설정) |
| [docs/VSCODE_DEBUGGING.md](docs/VSCODE_DEBUGGING.md) | VS Code + GDB 디버깅 가이드 |
| [docs/NUC_HYBRID_SUPPORT.md](docs/NUC_HYBRID_SUPPORT.md) | NUC 13 Pro hybrid CPU (P/E core) 지원 게이팅 (Stage A done, Stage B pending) |
| [CLAUDE.md](CLAUDE.md) | AI 에이전트 컨텍스트 (harness, invariants, escalation) |
| [repo_scripts/README.md](repo_scripts/README.md) | RT 설정 / 빌드 / 환경 셋업 쉘 스크립트 가이드 (PREEMPT_RT, CPU shield, IRQ affinity, `setup_env.sh`) |

각 패키지의 상세 API, 설정, 아키텍처는 해당 패키지의 `README.md`를 참조하세요.

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.
