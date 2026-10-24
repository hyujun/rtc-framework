# RTC (Real-Time Control) Framework

![Docs Validate](https://github.com/hyujun/rtc-framework/actions/workflows/docs-validate.yml/badge.svg)
![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-green)

URDF 기반 매니퓰레이터를 위한 robot-agnostic 실시간 제어 프레임워크와, 그 위에 UR5e · iiwa7 · G1 등을 묶은 통합 패키지. **유형**: ROS 2 colcon workspace (Ubuntu 24.04 / Jazzy 에서 검증, 22.04 / Humble 은 지원하지만 검증하지 않는다)

## 개요

- **RT 제어 루프** — `clock_nanosleep` 기반, 주기는 `control_rate` YAML (기본 500 Hz). 가변 DOF, lock-free SPSC · SeqLock, 전역 E-STOP. 스레드 · 코어 배치는 [rtc_base/README.md](rtc_base/README.md#스레딩-threading)
- **컨트롤러** — joint PD · CLIK · OSC · compliance (impedance / admittance) · CLIK-QP whole-body · MPC 연동. 목록과 전환 방법은 [docs/controllers.md](docs/controllers.md)
- **장치** — UR 드라이버, UDP 핸드 드라이버, MuJoCo 3.x 시뮬레이터가 같은 `DeviceBackend` 경계 뒤에 있다
- **도구** — 세션 단위 CSV 로깅, 실시간 게인 튜닝 GUI, 로그 플롯 · URDF/MJCF 변환 스크립트

```
[로봇 HW / MuJoCo] ── /joint_states ──▶ [integrated_rt_controller]
                                          RT 루프 @ control_rate
                                          └ 컨트롤러 (rtc_controllers 법칙)
   ◀── actuator 명령 (inline, RT) ────────┤
   [udp_hand_driver] ◀── SeqLock ─────────┤
   세션 CSV · 비-RT 토픽 ◀── SPSC ────────┘
```

CI ([docs-validate](.github/workflows/docs-validate.yml)) 는 문서 · 생성물 · 셸 검사만 한다. 빌드와 테스트는 로컬에서 돌린다.

## 빠른 시작

sim 하나를 띄우기까지 — 클론 → 설치 → 빌드 → 실행.

```bash
git clone --recursive https://github.com/hyujun/rtc-framework.git ~/ros2_ws/rtc_ws/src/rtc-framework
cd ~/ros2_ws/rtc_ws/src/rtc-framework
./install.sh sim
./build.sh sim
source ~/ros2_ws/rtc_ws/install/setup.bash
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py
```

### 클론 (submodule 포함)

`robot_descriptions/object_sim` 은 git submodule ([vikashplus/object_sim](https://github.com/vikashplus/object_sim)) 이다. `--recursive` 없이 클론하면 이 디렉토리가 비어 물체 mesh 를 쓰는 MuJoCo scene 이 로드에 실패한다. 이미 클론했다면:

```bash
git submodule update --init --recursive
```

### 설치

```bash
./install.sh sim          # 시뮬레이션 (MuJoCo 포함, RT 권한 없음)
./install.sh robot        # 실제 로봇 (UR 드라이버, MuJoCo 제외)
./install.sh full         # 전부
./install.sh robot --rt   # 실기 PC 의 RT 설정까지 (GRUB · CPU shield · IRQ · 권한)
```

apt · rosdep 에 없는 의존성 (ONNX Runtime · MuJoCo · `fmt` / `mimalloc` / `aligator`) 은 `install.sh` 가 핀 버전으로 설치한다 — 버전과 경로는 [install_deps.sh](repo_scripts/scripts/lib/install_deps.sh) · [build_deps.sh](repo_scripts/scripts/build_deps.sh) 가 갖는다.

표준 `rosdep` → `colcon` 흐름도 된다. 이 경우 `setup_env.sh` 의 병렬도 제한이 걸리지 않으므로 job 수를 직접 준다:

```bash
cd ~/ros2_ws/rtc_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro=jazzy -y
MAKEFLAGS=-j6 colcon build --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON
```

### 빌드

```bash
./build.sh sim                   # sim 패키지 (robot / full 도 있다)
./build.sh -p rtc_base           # 특정 패키지만
./build.sh --tests -p rtc_base   # 테스트까지 (기본은 테스트를 빌드하지 않는다)
./build.sh sim -c -j 4           # 클린 빌드 + make job 4개
```

직접 `colcon` 을 칠 때는 workspace root 에서, `setup_env.sh` 를 절대경로로 source 한 서브셸로 돌린다 (`setup_env.sh` 가 의존성 경로 · 병렬도 · colcon 기본값을 export 한다):

```bash
( cd ~/ros2_ws/rtc_ws && source ~/ros2_ws/rtc_ws/src/rtc-framework/repo_scripts/scripts/setup_env.sh >/dev/null 2>&1 && colcon build --packages-select rtc_base )
```

- `--cmake-args` 를 줄 때의 형태와 venv 와의 관계: [repo_scripts/README.md](repo_scripts/README.md) "Plain `colcon build` 호환성"
- job 수 · 메모리 상한 · 빌드 시간: [repo_scripts/README.md](repo_scripts/README.md) "빌드 병렬도와 메모리"
- 다른 저장소와 한 workspace 를 쓸 때: [repo_scripts/README.md](repo_scripts/README.md) "workspace 에 다른 저장소가 함께 있을 때"
- 테스트는 `--tests` 로 빌드한 뒤 돌린다. 그러지 않으면 실패가 아니라 테스트 0개가 보고된다 — 명령은 [docs/testing.md](docs/testing.md)

### Python 의존성 sync

`install.sh` 가 `uv` 로 `.venv` 를 만들고 `requirements.lock` 과 정확히 맞춘다. ROS 2 Python (`rclpy` 등) 은 `--system-site-packages` 로 시스템 것을 쓴다.

```bash
uv pip compile requirements.in --generate-hashes -o requirements.lock   # 의존성을 바꿨을 때
uv venv --python /usr/bin/python3.12 --system-site-packages .venv       # 새 머신 (install.sh 가 한다)
source .venv/bin/activate && uv pip sync requirements.lock
```

venv 의 base 는 배포판 python **경로**로 고정한다 — `--python 3.12` 로 만들면 uv-managed python 이 잡혀 apt 의 `catkin_pkg` 를 못 본다 (아래 문제 해결).

### 실행

```bash
# MuJoCo 시뮬레이션 (로봇별 launch: sim_ur5e_p1a · sim_ur5e_p1b · sim_iiwa7_leap · sim_g1_p1b)
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py

# 실제 로봇
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch integrated_bringup robot_ur5e_p1a.launch.py robot_ip:=192.168.1.10

# 가상 하드웨어 (로봇 없이)
ros2 launch integrated_bringup robot_ur5e_p1a.launch.py use_fake_hardware:=true

# 핸드 드라이버
ros2 launch udp_hand_driver udp_hand.launch.py target_ip:=192.168.1.2
```

launch 인자와 컨트롤러 전환은 [integrated_bringup/README.md](integrated_bringup/README.md) 와 [docs/controllers.md](docs/controllers.md) 에 있다.

### 상태 확인

```bash
ros2 topic echo /system/estop_status                     # E-STOP (true = 정지)
ros2 topic echo /sim/status                              # MuJoCo: [step, time, rtf, paused]
ros2 param list /integrated_rt_controller | grep controllers
PID=$(pidof -s integrated_rt_controller) && ps -eLo pid,tid,cls,rtprio,psr,comm | grep $PID
```

### 로그

각 실행은 `logging_data/YYMMDD_HHMM/` 세션 디렉토리에 기록된다. 보관 개수는 로봇 config 의 `max_log_sessions` (기본 10) 이고 같은 이름의 launch 인자로 바꿀 수 있다.

| 서브디렉토리 | 내용 |
|---|---|
| `controllers/<config_key>/` | 컨트롤러별 데이터 CSV |
| `timing/` | 스레드별 tick 타이밍 CSV |
| `device/` | 장치 통계 JSON |
| `tracing/` | LTTng trace ([docs/tracing.md](docs/tracing.md)) |
| `sim/` · `plots/` · `motions/` | MuJoCo 스크린샷 · `rtc_tools` 출력 |

플롯: `ros2 run rtc_tools plot_rtc_log --help`.

### 실기 PC 의 RT 권한

`./install.sh robot --rt` 가 RT 설정을 한 번에 한다. 단계별 수동 설정과 확인: [docs/rt-optimization.md](docs/rt-optimization.md). 확인만 할 때:

```bash
~/ros2_ws/rtc_ws/src/rtc-framework/repo_scripts/scripts/check_rt_setup.sh
```

## 패키지 구성

각 패키지 디렉토리의 `README.md` 가 사용법을, `package.xml` 이 의존성을 갖는다. 층위와 의존 방향: [architecture.md](agent_docs/architecture.md#dependency-graph).

### 로봇 비의존 프레임워크 (rtc_*)

| 패키지 | 역할 |
|---|---|
| [`rtc_msgs`](rtc_msgs/) | 메시지 · 서비스 정의 |
| [`rtc_base`](rtc_base/) | header-only RT 인프라 — 타입 · SeqLock · SPSC · 스레드 배치 · 필터 · 로깅 |
| [`rtc_math`](rtc_math/) | header-only SE(3) Lie-group 연산과 task-space 오차 |
| [`rtc_communication`](rtc_communication/) | header-only transport 추상화 — UDP · CAN / CAN FD · RS485 |
| [`rtc_urdf_bridge`](rtc_urdf_bridge/) | URDF 파서 + Pinocchio 모델 빌더 |
| [`rtc_controller_interface`](rtc_controller_interface/) | 컨트롤러 추상 인터페이스와 레지스트리 |
| [`rtc_controllers`](rtc_controllers/) | 제어 법칙 라이브러리 (노드 없음) |
| [`rtc_tsid`](rtc_tsid/) | TSID / CLIK QP 프레임워크 (ProxSuite) |
| [`rtc_mpc`](rtc_mpc/) | MPC ↔ RT 인터페이스 — TripleBuffer · 보간 · Riccati feedback · MPC 스레드 |
| [`rtc_controller_manager`](rtc_controller_manager/) | RT 루프 · 컨트롤러 lifecycle · E-STOP · `DeviceBackend` |
| [`rtc_inference`](rtc_inference/) | header-only RT-safe ONNX 추론 |
| [`rtc_mujoco_sim`](rtc_mujoco_sim/) | MuJoCo 3.x 시뮬레이터 노드와 뷰어 |
| [`rtc_digital_twin`](rtc_digital_twin/) | RViz2 디지털 트윈 (Python) |
| [`rtc_tools`](rtc_tools/) | 로그 플롯 · URDF/MJCF 변환 · 분석 스크립트 (Python) |
| [`repo_scripts`](repo_scripts/) | 설치 · 빌드 · RT 호스트 설정 스크립트 |

### 형상 추정

| 패키지 | 역할 |
|---|---|
| [`shape_estimation_msgs`](shape_estimation_msgs/) | ToF 형상 추정 메시지 · 액션 |
| [`shape_estimation`](shape_estimation/) | ToF 포인트 누적과 primitive 피팅 |

### 로봇 데이터 · 통합

| 패키지 | 역할 |
|---|---|
| [`robot_descriptions`](robot_descriptions/) | 로봇별 URDF · MJCF · mesh (data-only) |
| [`udp_hand_driver`](udp_hand_driver/) | 가변 DOF 핸드 드라이버 (UDP · CAN FD) |
| [`ur5e_bt_coordinator`](ur5e_bt_coordinator/) | BehaviorTree.CPP v4 태스크 코디네이터 |
| [`integrated_bringup`](integrated_bringup/) | 로봇별 launch · config, 데모 컨트롤러, 튜닝 GUI |

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| `robot_descriptions` mesh 로드 실패 | submodule 이 비어 있다 | `git submodule update --init --recursive` |
| `…so.<옛 버전>: cannot open shared object` · `No rule to make target '…so.<옛 버전>'` | `install.sh` 재실행이 apt 패키지를 올렸다 (되돌릴 수 없다, 같은 PC 의 다른 프로젝트도 영향) | `colcon build --cmake-force-configure --cmake-args -DBUILD_TESTING=ON` 로 전체를 다시 구성 · 빌드 |
| configure 가 `No module named 'catkin_pkg'` | CMake 가 venv 나 uv-managed python 을 잡았다 | `./build.sh` 를 쓰거나 [repo_scripts/README.md](repo_scripts/README.md) 의 plain colcon 형태를 따른다 |
| `colcon test` 가 테스트 0개 | 테스트를 빌드하지 않았다 | `./build.sh --tests -p <pkg>` 후 다시 |
| repo 안에 `build/` · `install/` · `log/` 가 생겼다 | workspace root 가 아닌 곳에서 `colcon` 을 쳤다 | 지우고 workspace root 에서 다시 |

빌드 · 환경 문제 전반: [repo_scripts/README.md](repo_scripts/README.md) "흔한 실패와 감별".

## 더 읽기

| 문서 | 내용 |
|---|---|
| [docs/controllers.md](docs/controllers.md) | 컨트롤러 카탈로그 — 전환 · 게인 · 토픽 · 설정 |
| [docs/testing.md](docs/testing.md) | 테스트 명령, 바꾼 곳별 테스트, 런타임 디버그 토픽 |
| [docs/rt-optimization.md](docs/rt-optimization.md) | 실기 PC 의 RT 설정 — 커널 · GRUB · CPU shield · IRQ · NVIDIA |
| [docs/tracing.md](docs/tracing.md) | LTTng trace 와 Perfetto 로 스레드 × 코어 보기 |
| [docs/vscode-debugging.md](docs/vscode-debugging.md) | VS Code + GDB 디버깅 |
| [docs/modification-procedures.md](docs/modification-procedures.md) | 컨트롤러 · 메시지 · 장치 · 스레드 · 패키지를 추가하는 절차 |
| [repo_scripts/README.md](repo_scripts/README.md) | 설치 · 빌드 · RT 스크립트 |
| [AGENTS.md](AGENTS.md) · [CLAUDE.md](CLAUDE.md) · [agent_docs/](agent_docs/) · [docs/reference/](docs/reference/README.md) | 기여자 · AI coding assistant 용 규약 · 규범 · 근거 |

## 의존성

ROS 2 Jazzy (또는 Humble), Eigen3, Pinocchio, ProxSuite, MuJoCo 3.x, ONNX Runtime, BehaviorTree.CPP v4. 패키지별 의존은 각 `package.xml`, 설치는 `install.sh`. 라이선스: MIT ([LICENSE](LICENSE)).
