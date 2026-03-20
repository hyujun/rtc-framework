# UR5e RT Controller Repository Restructuring Plan

## 현재 구조의 문제점

1. **ur5e_rt_controller가 모놀리식** — Controller interface, 6개 controller 구현, RT loop, controller registry, launch files, RT setup scripts가 모두 하나의 패키지에 존재
2. **관심사 분리 부족** — controller 정의(interface)와 구현(implementations)과 관리(manager)가 혼재
3. **통신 계층 분산** — UDP 추상화는 ur5e_rt_base에, hand UDP는 별도 패키지, DDS config는 ur5e_rt_controller에 분산
4. **Bringup 분산** — launch 파일이 각 패키지에 흩어져 있고, 통합 launch 로직이 없음
5. **RT 인프라가 여러 곳에 혼재** — RT 스크립트, CPU 핀닝, DDS 설정 등이 ur5e_rt_controller에 묶여 있음

## 참고 아키텍처 분석

| 패턴 | ros2_control | franka_ros2 | lbr_fri_ros2_stack |
|------|-------------|-------------|-------------------|
| Interface/Base | `controller_interface` | (ros2_control 사용) | (ros2_control 사용) |
| Controller 구현 | 별도 repo (ros2_controllers) | `franka_example_controllers` | `lbr_ros2_control` 내 |
| Manager | `controller_manager` | (ros2_control 사용) | (ros2_control 사용) |
| Hardware 통신 | `hardware_interface` | `franka_hardware` | `lbr_fri_ros2` |
| Description | (별도 repo) | (URDF 포함) | `lbr_description` |
| Bringup | (별도 repo) | `franka_bringup` | `lbr_bringup` |
| Simulation | (별도 repo) | `franka_gazebo_bringup` | Gazebo 통합 |
| Messages | `controller_manager_msgs` | `franka_msgs` | (표준 사용) |

## 제안하는 새 구조

```
ur5e-rt-controller/
├── ur5e_msgs/                    # [유지] 커스텀 메시지 정의
├── ur5e_description/             # [유지] URDF/MJCF/meshes
├── ur5e_rt_base/                 # [축소] RT 인프라 공통 라이브러리
├── ur5e_controller_interface/    # [신규] Controller 추상 인터페이스
├── ur5e_controllers/             # [신규] Controller 구현체들
├── ur5e_controller_manager/      # [신규] RT loop + controller lifecycle
├── ur5e_communication/           # [신규] 통신 계층 (UDP, hardware)
├── ur5e_status_monitor/          # [유지] 안전 모니터링
├── ur5e_mujoco_sim/              # [유지] MuJoCo 시뮬레이션
├── ur5e_digital_twin/            # [유지] RViz2 시각화
├── ur5e_bringup/                 # [신규] 통합 launch/config/RT scripts
├── ur5e_tools/                   # [유지] 개발 유틸리티
├── docs/                         # [유지] 문서
├── build.sh
├── install.sh
└── README.md
```

---

## 패키지별 상세 계획

### 1. `ur5e_msgs` — 유지 (변경 없음)
현재 7개 메시지 타입 그대로 유지.

### 2. `ur5e_description` — 유지 (변경 없음)
URDF, MJCF, meshes 그대로 유지.

### 3. `ur5e_rt_base` — 축소 (통신 레이어 분리)

**유지할 내용:**
```
ur5e_rt_base/include/ur5e_rt_base/
├── types/              # Core types (types.hpp)
├── threading/          # Thread utilities (thread_utils, thread_config, publish_buffer, seqlock)
├── filters/            # RT-safe signal processing (bessel_filter, kalman_filter)
└── logging/            # Lock-free data logging (data_logger, log_buffer, session_dir)
```

**ur5e_communication으로 이동할 내용:**
```
udp/                    # → ur5e_communication으로 이동
├── udp_socket.hpp
├── udp_transceiver.hpp
└── udp_codec.hpp
```

### 4. `ur5e_controller_interface` — 신규 (ur5e_rt_controller에서 분리)

Controller의 추상 인터페이스를 정의하는 패키지. ros2_control의 `controller_interface` 패턴 참고.

**포함 내용:**
```
ur5e_controller_interface/
├── include/ur5e_controller_interface/
│   ├── rt_controller_interface.hpp     # ← 기존 rt_controller_interface.hpp 이동
│   ├── controller_types.hpp            # Controller 관련 type 정의
│   └── controller_state.hpp            # Controller state 정보 구조체
├── src/
│   └── rt_controller_interface.cpp     # ← 기존 rt_controller_interface.cpp 이동
├── package.xml
└── CMakeLists.txt
```

**의존성:** `ur5e_rt_base`, `ur5e_msgs`, `sensor_msgs`, `pinocchio`

**역할:**
- Controller가 구현해야 할 순수 가상 함수 정의 (`Init()`, `Compute()`, `Reset()`, `Cleanup()`)
- 공통 state/command 접근 인터페이스
- Controller 등록 매크로/유틸리티

### 5. `ur5e_controllers` — 신규 (ur5e_rt_controller에서 분리)

6개 controller 구현체를 모은 패키지. franka_ros2의 `franka_example_controllers`, ros2_controllers 패턴 참고.

**포함 내용:**
```
ur5e_controllers/
├── include/ur5e_controllers/
│   ├── direct/                         # Torque command controllers
│   │   ├── joint_pd_controller.hpp
│   │   └── operational_space_controller.hpp
│   └── indirect/                       # Position command controllers
│       ├── p_controller.hpp
│       ├── clik_controller.hpp
│       ├── demo_joint_controller.hpp
│       ├── demo_task_controller.hpp
│       └── ur5e_hand_controller.hpp
├── src/
│   └── controllers/                    # 각 controller 구현 파일
├── config/
│   └── controllers/
│       ├── direct/
│       │   ├── joint_pd_controller.yaml
│       │   └── operational_space_controller.yaml
│       └── indirect/
│           ├── p_controller.yaml
│           ├── clik_controller.yaml
│           ├── demo_joint_controller.yaml
│           ├── demo_task_controller.yaml
│           └── ur5e_hand_controller.yaml
├── package.xml
└── CMakeLists.txt
```

**의존성:** `ur5e_controller_interface`, `ur5e_rt_base`, `ur5e_msgs`, `pinocchio`

**포함할 보조 코드:**
```
├── include/ur5e_controllers/
│   └── trajectory/                     # ← 기존 trajectory 유틸 이동
│       ├── trajectory_utils.hpp
│       ├── joint_space_trajectory.hpp
│       └── task_space_trajectory.hpp
```

### 6. `ur5e_controller_manager` — 신규 (ur5e_rt_controller 핵심 분리)

RT loop 실행, controller lifecycle 관리, runtime switching을 담당. ros2_control의 `controller_manager` 패턴 참고.

**포함 내용:**
```
ur5e_controller_manager/
├── include/ur5e_controller_manager/
│   ├── rt_controller_node.hpp          # ← 기존 rt_controller_node.hpp 이동
│   ├── controller_registry.hpp         # Controller 등록/검색 (기존 node에서 분리)
│   └── controller_timing_profiler.hpp  # ← 기존 timing profiler 이동
├── src/
│   ├── rt_controller_node.cpp          # ← 기존 rt_controller_node.cpp 이동
│   └── rt_controller_main.cpp          # ← 기존 main 이동
├── config/
│   ├── ur5e_rt_controller.yaml         # ← 기존 메인 config 이동
│   └── cyclone_dds.xml                 # ← 기존 DDS config 이동
├── test/
│   └── ...
├── package.xml
└── CMakeLists.txt
```

**의존성:** `ur5e_controller_interface`, `ur5e_controllers`, `ur5e_rt_base`, `ur5e_communication`, `ur5e_status_monitor`, `ur5e_msgs`

**역할:**
- 500Hz RT loop 실행 (`clock_nanosleep`)
- Controller 등록, 선택, runtime switching
- Lock-free publish offload 아키텍처
- E-STOP 관리
- CPU core 핀닝 및 스레드 스케줄링

### 7. `ur5e_communication` — 신규 (ur5e_rt_base + ur5e_hand_udp 통합)

모든 하드웨어 통신을 담당하는 패키지. lbr_fri_ros2_stack의 `lbr_fri_ros2`, franka_ros2의 `franka_hardware` 패턴 참고.

**포함 내용:**
```
ur5e_communication/
├── include/ur5e_communication/
│   ├── udp/                            # ← ur5e_rt_base/udp/ 이동
│   │   ├── udp_socket.hpp
│   │   ├── udp_transceiver.hpp
│   │   └── udp_codec.hpp
│   └── hand/                           # ← ur5e_hand_udp/ 이동
│       ├── hand_packets.hpp
│       ├── hand_udp_codec.hpp
│       ├── hand_controller.hpp
│       └── hand_failure_detector.hpp
├── src/
│   └── hand/
│       └── hand_udp_node.cpp
├── config/
│   └── hand_udp_node.yaml              # ← 기존 hand config 이동
├── launch/
│   └── hand_udp.launch.py              # ← 기존 hand launch 이동
├── package.xml
└── CMakeLists.txt
```

**의존성:** `ur5e_rt_base`, `ur5e_msgs`

**역할:**
- UDP 통신 추상화 (socket, transceiver, codec)
- Hand UDP 프로토콜 (request-response)
- Hand failure detection
- 향후 다른 하드웨어 통신도 이곳에 추가 가능

### 8. `ur5e_status_monitor` — 유지 (변경 없음)
기존 안전 모니터링 로직 그대로 유지.

### 9. `ur5e_mujoco_sim` — 유지 (변경 없음)
기존 MuJoCo 시뮬레이션 패키지 그대로 유지.

### 10. `ur5e_digital_twin` — 유지 (변경 없음)
기존 RViz2 시각화 패키지 그대로 유지.

### 11. `ur5e_bringup` — 신규 (launch/config/scripts 통합)

시스템 전체의 launch, 설정, RT 환경 스크립트를 통합. franka_ros2의 `franka_bringup`, lbr_fri_ros2_stack의 `lbr_bringup` 패턴 참고.

**포함 내용:**
```
ur5e_bringup/
├── launch/
│   ├── ur5e_robot.launch.py            # ← 기존 ur_control.launch.py 리팩터링
│   ├── ur5e_sim.launch.py              # ← 기존 mujoco_sim.launch.py 래핑
│   ├── ur5e_full.launch.py             # 로봇 + 디지털 트윈 + 모니터링 통합
│   └── ur5e_hand.launch.py             # ← 기존 hand_udp.launch.py 래핑
├── config/
│   ├── robot_params.yaml               # 로봇 기본 파라미터 (IP, joint names, limits)
│   └── launch_profiles/                # 다양한 실행 프로파일
│       ├── default.yaml
│       ├── sim_only.yaml
│       └── full_system.yaml
├── scripts/
│   ├── build_rt_kernel.sh              # ← 기존 scripts 이동
│   ├── check_rt_setup.sh
│   ├── setup_irq_affinity.sh
│   ├── setup_udp_optimization.sh
│   ├── setup_nvidia_rt.sh
│   ├── cpu_shield.sh
│   ├── verify_rt_runtime.sh
│   └── lib/rt_common.sh
├── package.xml
└── CMakeLists.txt
```

**의존성:** `ur5e_controller_manager`, `ur5e_mujoco_sim`, `ur5e_digital_twin`, `ur5e_communication`

**역할:**
- 통합 시스템 launch (로봇/시뮬레이션/full)
- RT 환경 설정 스크립트 중앙 관리
- Launch profile 기반 설정

### 12. `ur5e_tools` — 유지 (변경 없음)
기존 Python 개발 유틸리티 그대로 유지.

---

## 패키지 의존성 그래프

```
ur5e_msgs (독립)
ur5e_description (독립)
ur5e_rt_base (독립, header-only)
    │
    ├── ur5e_communication ← ur5e_rt_base, ur5e_msgs
    │
    ├── ur5e_controller_interface ← ur5e_rt_base, ur5e_msgs
    │       │
    │       └── ur5e_controllers ← ur5e_controller_interface, ur5e_rt_base
    │               │
    │               └── ur5e_controller_manager ← ur5e_controllers, ur5e_controller_interface,
    │                                              ur5e_communication, ur5e_status_monitor
    │
    ├── ur5e_status_monitor ← ur5e_rt_base
    │
    ├── ur5e_mujoco_sim ← ur5e_description
    │
    ├── ur5e_digital_twin (독립, Python)
    │
    └── ur5e_bringup ← ur5e_controller_manager, ur5e_mujoco_sim,
                        ur5e_digital_twin, ur5e_communication

ur5e_tools (독립, Python)
```

---

## 마이그레이션 순서 (단계별)

### Phase 1: 기반 패키지 분리 (의존성 없는 것부터)
1. `ur5e_communication` 생성 — ur5e_rt_base에서 udp/ 이동 + ur5e_hand_udp 흡수
2. `ur5e_controller_interface` 생성 — ur5e_rt_controller에서 interface 분리

### Phase 2: 구현 패키지 분리
3. `ur5e_controllers` 생성 — 6개 controller 구현 + trajectory 유틸 이동
4. `ur5e_controller_manager` 생성 — RT loop, node, registry, timing profiler 이동

### Phase 3: 통합 패키지 생성
5. `ur5e_bringup` 생성 — launch files 통합 + RT scripts 이동
6. 기존 `ur5e_rt_controller`, `ur5e_hand_udp` 패키지 제거

### Phase 4: 정리
7. build.sh, install.sh 업데이트
8. README.md 업데이트
9. CI/CD 설정 업데이트

---

## 기존 → 새 구조 파일 매핑

| 기존 위치 | 새 위치 |
|-----------|---------|
| `ur5e_rt_controller/include/.../rt_controller_interface.hpp` | `ur5e_controller_interface/include/...` |
| `ur5e_rt_controller/src/rt_controller_interface.cpp` | `ur5e_controller_interface/src/...` |
| `ur5e_rt_controller/include/.../controllers/direct/` | `ur5e_controllers/include/.../direct/` |
| `ur5e_rt_controller/include/.../controllers/indirect/` | `ur5e_controllers/include/.../indirect/` |
| `ur5e_rt_controller/src/controllers/` | `ur5e_controllers/src/controllers/` |
| `ur5e_rt_controller/include/.../trajectory/` | `ur5e_controllers/include/.../trajectory/` |
| `ur5e_rt_controller/config/controllers/` | `ur5e_controllers/config/controllers/` |
| `ur5e_rt_controller/include/.../rt_controller_node.hpp` | `ur5e_controller_manager/include/...` |
| `ur5e_rt_controller/src/rt_controller_node.cpp` | `ur5e_controller_manager/src/...` |
| `ur5e_rt_controller/src/rt_controller_main.cpp` | `ur5e_controller_manager/src/...` |
| `ur5e_rt_controller/include/.../controller_timing_profiler.hpp` | `ur5e_controller_manager/include/...` |
| `ur5e_rt_controller/config/ur5e_rt_controller.yaml` | `ur5e_controller_manager/config/...` |
| `ur5e_rt_controller/config/cyclone_dds.xml` | `ur5e_controller_manager/config/...` |
| `ur5e_rt_controller/launch/ur_control.launch.py` | `ur5e_bringup/launch/ur5e_robot.launch.py` |
| `ur5e_rt_controller/scripts/*.sh` | `ur5e_bringup/scripts/` |
| `ur5e_rt_base/include/.../udp/` | `ur5e_communication/include/.../udp/` |
| `ur5e_hand_udp/` (전체) | `ur5e_communication/` (hand/ 서브디렉토리) |
| `ur5e_mujoco_sim/launch/mujoco_sim.launch.py` | 유지 + `ur5e_bringup/launch/ur5e_sim.launch.py`에서 래핑 |

---

## 장점

1. **관심사 분리** — Interface, Implementation, Manager가 명확히 분리
2. **독립 빌드/테스트** — 각 패키지를 독립적으로 빌드/테스트 가능
3. **재사용성** — controller_interface를 기반으로 새 controller 추가 용이
4. **표준 패턴** — ros2_control, franka_ros2, lbr_fri_ros2_stack과 동일한 구조
5. **통합 Bringup** — 시스템 전체를 하나의 진입점으로 launch 가능
6. **통신 추상화** — 새 하드웨어 추가 시 communication 패키지만 확장
