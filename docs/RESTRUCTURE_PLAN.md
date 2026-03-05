# 디렉토리 재구성 계획

**작성일:** 2026-03-04
**대상 버전:** v4.5.0 → v5.0.0

---

## 1. 현황 분석

### 현재 구조 (단일 패키지)

```
ur5e-rt-controller/          ← git repo = ROS2 패키지 루트
├── CMakeLists.txt           ← 4개 실행파일을 하나의 패키지에서 빌드
├── package.xml
├── include/ur5e_rt_controller/
│   ├── rt_controller_interface.hpp   (핵심 타입 + 추상 기반)
│   ├── log_buffer.hpp
│   ├── data_logger.hpp
│   ├── controller_timing_profiler.hpp
│   ├── thread_config.hpp
│   ├── thread_utils.hpp
│   ├── hand_udp_receiver.hpp         ← thread_config/utils 의존
│   ├── hand_udp_sender.hpp
│   ├── mujoco_simulator.hpp          ← 1451줄, mujoco+GLFW 의존
│   └── controllers/
│       ├── pd_controller.hpp
│       ├── p_controller.hpp
│       ├── pinocchio_controller.hpp
│       ├── clik_controller.hpp
│       └── operational_space_controller.hpp
├── src/                              ← 4개 노드 소스
├── config/                           ← 3개 설정 파일 혼재
├── launch/                           ← 3개 런치파일 혼재
├── models/ur5e/                      ← MuJoCo 모델
└── scripts/                          ← Python 유틸리티
```

### 문제점

| 문제 | 설명 |
|------|------|
| **빌드 결합도 과다** | MuJoCo 없으면 시뮬 제외, Pinocchio 없으면 제어기 제외 — 조건부 빌드가 복잡해짐 |
| **배포 단위 불명확** | Hand-only 배포, Sim-only 배포가 패키지 단위로 불가 |
| **헤더 혼재** | RT 핵심 / UDP 통신 / 시뮬레이터가 같은 include/ 에 섞여 있음 |
| **설정/런치 혼재** | config/, launch/ 에 서로 다른 기능의 파일이 혼재 |
| **의존성 불명확** | `mujoco_simulator.hpp` (1451줄)가 핵심 패키지에 포함 |

---

## 2. 재구성 목표

1. **기능별 패키지 분리** — 실행 단위를 기준으로 4개 패키지로 분리
2. **독립 빌드/배포** — 각 패키지가 독립적으로 빌드·설치 가능
3. **의존성 명확화** — 의존 방향이 단방향 DAG를 형성
4. **RT 핵심 보호** — RT 경로 코드와 옵션 기능(MuJoCo, Python 도구)을 분리

---

## 3. 의존성 현황 분석

### 주요 의존 관계

```
hand_udp_receiver.hpp
  ├── rt_controller_interface.hpp  (kNumHandJoints, HandState)
  ├── thread_config.hpp
  └── thread_utils.hpp

mujoco_simulator.hpp
  ├── mujoco.h (mujoco 3.x)
  └── GLFW/glfw3.h (선택적)

controllers/*.hpp
  ├── rt_controller_interface.hpp
  └── pinocchio (pinocchio_controller, clik, osc만)
```

### 분리 시 핵심 이슈

`hand_udp_receiver.hpp`가 `rt_controller_interface.hpp`와 `thread_config/utils`를 포함하므로
`ur5e_hand_udp` 패키지가 `ur5e_rt_controller`에 의존하거나,
공유 기반 패키지를 별도로 추출해야 합니다.

---

## 4. 제안 구조

### 방안 A — 실용적 분리 (권장, 4 패키지)

의존 방향: `ur5e_hand_udp` → `ur5e_rt_controller`

```
ur5e-rt-controller/                   ← git repo 루트 (비패키지)
│
├── ur5e_rt_controller/               ← Package 1: 핵심 RT 컨트롤러
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/ur5e_rt_controller/
│   │   ├── rt_controller_interface.hpp
│   │   ├── log_buffer.hpp
│   │   ├── data_logger.hpp
│   │   ├── controller_timing_profiler.hpp
│   │   ├── thread_config.hpp
│   │   ├── thread_utils.hpp
│   │   └── controllers/
│   │       ├── pd_controller.hpp
│   │       ├── p_controller.hpp
│   │       ├── pinocchio_controller.hpp
│   │       ├── clik_controller.hpp
│   │       └── operational_space_controller.hpp
│   ├── src/
│   │   └── custom_controller.cpp
│   ├── config/
│   │   └── ur5e_rt_controller.yaml
│   └── launch/
│       └── ur_control.launch.py
│
├── ur5e_hand_udp/                    ← Package 2: Hand UDP 통신
│   ├── CMakeLists.txt
│   ├── package.xml                   (depend: ur5e_rt_controller)
│   ├── include/ur5e_hand_udp/
│   │   ├── hand_udp_receiver.hpp
│   │   └── hand_udp_sender.hpp
│   ├── src/
│   │   ├── hand_udp_receiver_node.cpp
│   │   └── hand_udp_sender_node.cpp
│   ├── config/
│   │   └── hand_udp_receiver.yaml
│   └── launch/
│       └── hand_udp.launch.py
│
├── ur5e_mujoco_sim/                  ← Package 3: MuJoCo 시뮬레이션
│   ├── CMakeLists.txt
│   ├── package.xml                   (depend: mujoco, glfw3만)
│   ├── include/ur5e_mujoco_sim/
│   │   └── mujoco_simulator.hpp
│   ├── src/
│   │   └── mujoco_simulator_node.cpp
│   ├── config/
│   │   └── mujoco_simulator.yaml
│   ├── models/
│   │   └── ur5e/
│   │       ├── scene.xml
│   │       └── ur5e.xml
│   └── launch/
│       └── mujoco_sim.launch.py
│
├── ur5e_tools/                       ← Package 4: Python 유틸리티
│   ├── CMakeLists.txt
│   ├── package.xml                   (ament_python, depend: rclpy)
│   ├── requirements.txt
│   ├── resource/
│   │   └── ur5e_tools
│   ├── setup.py
│   ├── setup.cfg
│   └── scripts/
│       ├── motion_editor_gui.py
│       ├── monitor_data_health.py
│       ├── plot_ur_trajectory.py
│       └── hand_udp_sender_example.py
│
├── docs/
│   ├── CHANGELOG.md
│   ├── RT_OPTIMIZATION.md
│   └── RESTRUCTURE_PLAN.md          ← 이 파일
│
├── install.sh                        (워크스페이스 전체 빌드)
├── CLAUDE.md
└── README.md
```

**의존 그래프 (방향: → 의존)**
```
ur5e_rt_controller  ← (독립)
ur5e_hand_udp       ← ur5e_rt_controller
ur5e_mujoco_sim     ← (독립, mujoco/glfw만)
ur5e_tools          ← (독립, rclpy만)
```

**장단점**
| 장점 | 단점 |
|------|------|
| 패키지 수 최소화 | hand_udp가 rt_controller에 의존 (타입/스레드 공유 때문) |
| 빠른 마이그레이션 | hand_udp 단독 배포 시 rt_controller도 필요 |
| 코드 변경 최소화 | |

---

### 방안 B — 클린 분리 (5 패키지, 장기 권장)

공유 기반 패키지 `ur5e_rt_base`를 추출하여 hand_udp가 독립적으로 분리 가능:

```
ur5e_rt_base/           ← 공유 타입 + RT 유틸리티 (헤더 전용)
  include/ur5e_rt_base/
    ├── types.hpp                    (RobotState, HandState, kNum* 상수)
    ├── thread_config.hpp
    └── thread_utils.hpp

ur5e_rt_controller/     ← 제어기 구현 + 메인 노드
  (depend: ur5e_rt_base, pinocchio)

ur5e_hand_udp/          ← Hand UDP 통신
  (depend: ur5e_rt_base만)  ← rt_controller 불필요!

ur5e_mujoco_sim/        ← MuJoCo 시뮬레이션
  (depend: ur5e_rt_base)

ur5e_tools/             ← Python 도구
  (depend: rclpy)
```

**의존 그래프**
```
ur5e_rt_base      ← (독립)
ur5e_rt_controller ← ur5e_rt_base
ur5e_hand_udp      ← ur5e_rt_base
ur5e_mujoco_sim    ← ur5e_rt_base
ur5e_tools         ← (독립)
```

**필요 코드 변경:** `rt_controller_interface.hpp`에서 `types.hpp`로 데이터 구조 분리

---

## 5. 권장 실행 순서

### 1단계 — 방안 A 먼저 적용 (빠른 분리)

```bash
# git repo 루트 수준으로 패키지 이동
git mv CMakeLists.txt ur5e_rt_controller/
git mv package.xml ur5e_rt_controller/
git mv include/ ur5e_rt_controller/
git mv src/ ur5e_rt_controller/
git mv config/ur5e_rt_controller.yaml ur5e_rt_controller/config/
git mv launch/ur_control.launch.py ur5e_rt_controller/launch/
# ... (hand_udp, mujoco_sim, tools 패키지 생성)
```

#### 각 패키지별 작업

**ur5e_rt_controller** (기존 핵심 코드 이동)
- [ ] 패키지 디렉토리 생성 및 파일 이동
- [ ] CMakeLists.txt — mujoco_simulator 관련 코드 제거, `ament_export_include_directories` 추가
- [ ] package.xml — 버전 5.0.0, `buildtool_export_depend` 추가로 헤더 export

**ur5e_hand_udp** (신규 패키지)
- [ ] CMakeLists.txt 작성 (`find_package(ur5e_rt_controller REQUIRED)`)
- [ ] package.xml 작성 (`<depend>ur5e_rt_controller</depend>`)
- [ ] include 경로 변경: `ur5e_rt_controller/hand_udp_*.hpp` → `ur5e_hand_udp/hand_udp_*.hpp`
- [ ] hand_udp_receiver.hpp include 경로 수정 (rt_controller_interface.hpp는 유지)

**ur5e_mujoco_sim** (신규 패키지)
- [ ] CMakeLists.txt 작성 (mujoco, glfw3 find_package, OPTIONAL 처리)
- [ ] package.xml 작성
- [ ] mujoco_simulator.hpp include 경로: `ur5e_mujoco_sim/mujoco_simulator.hpp`
- [ ] mujoco_sim.launch.py 이동
- [ ] models/ 이동

**ur5e_tools** (신규 패키지, ament_python)
- [ ] setup.py, setup.cfg 작성
- [ ] CMakeLists.txt (ament_python_install_package)
- [ ] scripts/ 이동

### 2단계 — install.sh 업데이트

```bash
# 워크스페이스에서 모든 패키지 빌드
colcon build --packages-select \
  ur5e_rt_controller ur5e_hand_udp ur5e_mujoco_sim ur5e_tools \
  --symlink-install
```

### 3단계 — 방안 B 검토 (선택적)

`ur5e_rt_base` 추출 여부는 실제 패키지 분리 후 필요성 판단:
- hand_udp를 완전 독립 배포해야 한다면 진행
- 현재 시스템 규모에서는 방안 A로 충분할 가능성 높음

---

## 6. 각 패키지 CMakeLists.txt 스케치

### ur5e_rt_controller/CMakeLists.txt (주요 변경점)

```cmake
cmake_minimum_required(VERSION 3.22)
project(ur5e_rt_controller VERSION 5.0.0)

# ... find_package들 ...

# 헤더 export (ur5e_hand_udp가 사용)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs sensor_msgs pinocchio)

# custom_controller 실행파일만 (hand_udp, mujoco 제거)
add_executable(custom_controller src/custom_controller.cpp)
# ...

install(DIRECTORY include/ DESTINATION include/)  # 헤더 설치
```

### ur5e_hand_udp/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.22)
project(ur5e_hand_udp VERSION 1.0.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ur5e_rt_controller REQUIRED)  # thread_config, types 공유

add_executable(hand_udp_receiver_node src/hand_udp_receiver_node.cpp)
target_include_directories(hand_udp_receiver_node PUBLIC include)
ament_target_dependencies(hand_udp_receiver_node rclcpp std_msgs sensor_msgs ur5e_rt_controller)

add_executable(hand_udp_sender_node src/hand_udp_sender_node.cpp)
ament_target_dependencies(hand_udp_sender_node rclcpp std_msgs ur5e_rt_controller)
```

### ur5e_mujoco_sim/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.22)
project(ur5e_mujoco_sim VERSION 1.0.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(mujoco QUIET)  # 선택적

if(mujoco_FOUND)
  add_executable(mujoco_simulator_node src/mujoco_simulator_node.cpp)
  target_include_directories(mujoco_simulator_node PUBLIC include)
  ament_target_dependencies(mujoco_simulator_node rclcpp std_msgs sensor_msgs ament_index_cpp)
  target_link_libraries(mujoco_simulator_node mujoco::mujoco)
  # ...
else()
  message(WARNING "mujoco not found — mujoco_simulator_node will not be built")
endif()
```

---

## 7. 영향 받는 파일 전체 목록

| 파일 | 작업 | 비고 |
|------|------|------|
| `CMakeLists.txt` | 분할 → 4개 패키지 CMakeLists | |
| `package.xml` | 분할 → 4개 package.xml | 버전 5.0.0 |
| `include/ur5e_rt_controller/*.hpp` | 이동 + include 경로 수정 | |
| `include/ur5e_rt_controller/hand_udp_*.hpp` | `ur5e_hand_udp/` 로 이동 | |
| `include/ur5e_rt_controller/mujoco_simulator.hpp` | `ur5e_mujoco_sim/` 으로 이동 | |
| `src/custom_controller.cpp` | 이동 (내용 무변경) | |
| `src/hand_udp_receiver_node.cpp` | 이동 + include 경로 수정 | |
| `src/hand_udp_sender_node.cpp` | 이동 + include 경로 수정 | |
| `src/mujoco_simulator_node.cpp` | 이동 + include 경로 수정 | |
| `config/ur5e_rt_controller.yaml` | `ur5e_rt_controller/config/` 로 이동 | |
| `config/hand_udp_receiver.yaml` | `ur5e_hand_udp/config/` 로 이동 | |
| `config/mujoco_simulator.yaml` | `ur5e_mujoco_sim/config/` 로 이동 | |
| `launch/ur_control.launch.py` | `ur5e_rt_controller/launch/` 로 이동 | |
| `launch/hand_udp.launch.py` | `ur5e_hand_udp/launch/` 로 이동 | |
| `launch/mujoco_sim.launch.py` | `ur5e_mujoco_sim/launch/` 로 이동 | |
| `models/` | `ur5e_mujoco_sim/models/` 로 이동 | |
| `scripts/` | `ur5e_tools/scripts/` 로 이동 | |
| `requirements.txt` | `ur5e_tools/requirements.txt` 로 이동 | |
| `install.sh` | 워크스페이스 루트 유지, 빌드 커맨드 업데이트 | |
| `CLAUDE.md` | 레포 루트 유지, 경로 업데이트 | |

---

## 8. 버전 계획

| 패키지 | 버전 | 비고 |
|--------|------|------|
| `ur5e_rt_controller` | 5.0.0 | 기존 4.5.0에서 major bump (구조 변경) |
| `ur5e_hand_udp` | 1.0.0 | 신규 분리 |
| `ur5e_mujoco_sim` | 1.0.0 | 신규 분리 |
| `ur5e_tools` | 1.0.0 | 신규 분리 |

---

## 9. 결정 필요 사항

마이그레이션 전에 다음을 확인하세요:

1. **방안 A vs B**: `ur5e_hand_udp`가 `ur5e_rt_controller`에 의존해도 괜찮은가?
   - A (권장): hand_udp가 rt_controller에 의존 — 코드 변경 최소
   - B: `ur5e_rt_base` 추출 — 완전 독립, 추가 리팩토링 필요

2. **레포 구조**: 패키지를 레포 루트에 바로 둘 것인가, `src/` 서브디렉토리에 둘 것인가?
   - 루트 직하 (권장): `colcon build --symlink-install` 사용 용이
   - `src/` 아래: 표준 ROS2 workspace 규약, 레포 루트에 빌드 결과물 없음

3. **hand_udp_receiver.hpp 리팩토링**: 방안 A에서도 include 경로만 변경할지, 아니면 `HandState` 타입을 hand_udp 자체 타입으로 대체할지?

---

*이 계획은 승인 후 별도 PR로 단계적 구현 예정*
