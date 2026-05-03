# robot_descriptions

![version](https://img.shields.io/badge/version-v5.17.0-blue)

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md)

---

## 1. 패키지 이름 및 설명

| 항목 | 내용 |
|------|------|
| **패키지 이름** | `robot_descriptions` |
| **버전** | 5.17.0 |
| **빌드 시스템** | ament_cmake |
| **라이선스** | MIT |
| **관리자** | Junho Park (jeryblueput@gmail.com) |

> **이 패키지는 robot-specific 패키지가 아니다.** URDF / MJCF / mesh 자산을 담는 **robot-agnostic data hub**이며, 한 robot/hand 당 하나의 `robots/<name>/` 서브디렉토리를 두는 단일 규칙으로 동작한다. C++ 헤더, robot-specific 상수, 프레임워크 의존 코드는 일체 들어 있지 않다 (모두 각 driver/bringup 패키지가 보유). UR5e + assm_v1 hand는 첫 입주자일 뿐, 추후 allegro / leap / schunk / iiwa7 등이 같은 레이아웃으로 추가될 예정이다.

다운스트림은 ament share 경로 또는 `package://robot_descriptions/robots/<name>/...` URL로 자산을 참조하며, 새 robot 추가 시 본 패키지의 빌드/설치 규칙은 변경되지 않는다 (`install(DIRECTORY robots/)` 한 줄로 충분).

런타임 외부 의존: `xacro` (xacro 파일을 로드하는 robot에 한해).  코드 의존: 없음.

---

## 2. 개요

이 패키지는 자산을 세 가지 카테고리로 정리한다.

| 모델 유형 | 용도 | 위치 (per robot) |
|-----------|------|------|
| **MJCF** (MuJoCo XML) | MuJoCo 물리 시뮬레이션 | `robots/<name>/mjcf/` |
| **URDF** (`.urdf`, `.urdf.xacro`) | Pinocchio, RViz, ros2_control | `robots/<name>/urdf/` |
| **Mesh** (DAE / STL / OBJ) | 시각화 및 충돌 감지 | `robots/<name>/meshes/` |

현재 입주한 robots:

| 디렉토리 | 내용 |
|---------|------|
| `robots/ur5e/` | UR5e 6-DoF arm — URDF (xacro 사전 생성), MJCF, full mesh set |
| `robots/assm_v1/` | 10-DoF custom hand "assm v1" — URDF xacro, MJCF (cylinder geom) |
| `robots/ur5e_assm_v1/` | UR5e arm + assm_v1 hand 결합 — URDF xacro, MJCF scene (wrist3 말단 부착) |

향후 robot 추가는 `robots/<new_name>/` 서브디렉토리 추가만으로 끝난다 — 본 패키지의 `CMakeLists.txt` / `package.xml`은 손대지 않는다.

---

## 3. 디렉토리 구조

```
robot_descriptions/
├── CMakeLists.txt
├── package.xml
├── README.md
└── robots/
    ├── ur5e/                              # UR5e 로봇 암 (단독)
    │   ├── mjcf/
    │   │   ├── ur5e.xml                   # UR5e 로봇 단독 모델
    │   │   └── scene.xml                  # 씬 (지면 + 조명 + ur5e.xml)
    │   ├── urdf/
    │   │   └── ur5e.urdf                  # UR5e 로봇 URDF (xacro 사전 생성)
    │   └── meshes/                        # 3D 메시 파일
    │       ├── visual/                    # DAE 시각화용 (7개)
    │       │   ├── base.dae ... wrist3.dae
    │       ├── collision/                 # STL 충돌 감지용 (7개)
    │       │   ├── base.stl ... wrist3.stl
    │       └── assets/                    # OBJ MJCF 시각화용 (20개)
    │           ├── base_0.obj ~ wrist3.obj
    │
    ├── assm_v1/                           # 10-DOF 커스텀 핸드 "assm v1" (단독)
    │   ├── mjcf/
    │   │   └── hand.xml                   # 핸드 단독 MJCF 모델
    │   └── urdf/
    │       └── hand.urdf.xacro            # 핸드 xacro 매크로
    │
    └── ur5e_assm_v1/                      # UR5e + assm_v1 통합
        ├── mjcf/
        │   ├── ur5e_with_hand.xml         # 로봇 + 핸드 통합 MJCF
        │   └── scene_with_hand.xml        # 씬 (지면 + 조명 + 통합 모델)
        └── urdf/
            └── ur5e_with_hand.urdf.xacro  # 로봇 + 핸드 결합 xacro
```

---

## 4. MJCF 모델 (MuJoCo)

3개 디렉토리에 총 5개의 MJCF XML 파일이 있습니다.

| 파일 | 경로 | 설명 |
|------|------|------|
| `ur5e.xml` | `robots/ur5e/mjcf/` | UR5e 로봇 단독 모델. capsule collision geometry, position actuator (PD 제어), OBJ 메시 시각화. `implicitfast` 적분기 사용. |
| `scene.xml` | `robots/ur5e/mjcf/` | 시뮬레이션 진입점. `ur5e.xml`을 include하고 지면(checker 평면), 조명, 스카이박스를 추가한 씬 파일. |
| `hand.xml` | `robots/assm_v1/mjcf/` | 10-DOF 커스텀 핸드 단독 모델. cylinder geometry 기반 간소화 형상. 4개 손가락(thumb, index, middle, ring) 포함. |
| `ur5e_with_hand.xml` | `robots/ur5e_assm_v1/mjcf/` | UR5e 로봇과 커스텀 핸드를 하나의 모델로 통합. wrist3 말단에 핸드가 부착됨. |
| `scene_with_hand.xml` | `robots/ur5e_assm_v1/mjcf/` | 시뮬레이션 진입점. `ur5e_with_hand.xml`을 include하고 지면, 조명, 스카이박스를 추가한 씬 파일. |

### MJCF 액추에이터 설정

- **UR5e 관절 (shoulder/elbow)**: `gainprm=2000`, `biasprm="0 -2000 -400"`, `forcerange=[-150, 150]`
- **UR5e 관절 (wrist)**: `gainprm=500`, `biasprm="0 -500 -100"`, `forcerange=[-28, 28]`
- **핸드 관절**: `gainprm=500`, `biasprm="0 -500 -100"`, `forcerange=[-5, 5]`

---

## 5. URDF 모델

3개 디렉토리에 총 3개의 URDF 파일이 있습니다.

| 파일 | 경로 | 설명 |
|------|------|------|
| `ur5e.urdf` | `robots/ur5e/urdf/` | UR5e 로봇 사전 생성 URDF. UR 공식 `ur_description` 패키지의 `ur.urdf.xacro`를 `ur_type:=ur5e`로 변환하여 생성. DAE/STL 메시 참조. Pinocchio 모델 빌드의 진입점. |
| `hand.urdf.xacro` | `robots/assm_v1/urdf/` | 10-DOF 커스텀 핸드 xacro. 기하학적 프리미티브(box, cylinder, sphere)만 사용하며 메시 파일 불필요. 4개 핑거팁 프레임 정의. |
| `ur5e_with_hand.urdf.xacro` | `robots/ur5e_assm_v1/urdf/` | `ur5e.urdf`와 `hand.urdf.xacro`를 결합. tool0 링크에 fixed joint로 핸드를 부착. `xacro` 처리 필요. |

---

## 6. 관절 사양

### UR5e 로봇 관절 (6 revolute)

| 관절 | 타입 | 위치 한계 (rad) | 최대 속도 (rad/s) | 최대 토크 (Nm) |
|------|------|----------------|-------------------|----------------|
| shoulder_pan_joint | revolute | +-2pi | 3.1416 | 150.0 |
| shoulder_lift_joint | revolute | +-2pi | 3.1416 | 150.0 |
| elbow_joint | revolute | +-pi | 3.1416 | 150.0 |
| wrist_1_joint | revolute | +-2pi | 3.1416 | 28.0 |
| wrist_2_joint | revolute | +-2pi | 3.1416 | 28.0 |
| wrist_3_joint | revolute | +-2pi | 3.1416 | 28.0 |

### 커스텀 핸드 관절 (10 revolute)

| 핑거 | 관절 | 범위 (rad) | 최대 토크 (Nm) |
|-------|------|-----------|----------------|
| Thumb | thumb_cmc_aa, thumb_cmc_fe, thumb_mcp_fe | +-pi/2 | 5.0 |
| Index | index_mcp_aa, index_mcp_fe, index_dip_fe | +-pi/2 | 5.0 |
| Middle | middle_mcp_aa, middle_mcp_fe, middle_dip_fe | +-pi/2 | 5.0 |
| Ring | ring_mcp_fe | +-pi/2 | 5.0 |

**핑거팁 프레임 (fixed link):** `thumb_tip_link`, `index_tip_link`, `middle_tip_link`, `ring_tip_link`

### 링크 질량 비교

| 링크 | URDF (kg) | MJCF (kg) | 비고 |
|------|-----------|-----------|------|
| base_link_inertia | 4.0 | 4.0 | 고정 베이스 |
| shoulder_link | 3.761 | 3.7 | -- |
| upper_arm_link | 8.058 | 8.393 | MJCF +0.335 kg 차이 |
| forearm_link | 2.846 | 2.275 | MJCF -0.571 kg 차이 |
| wrist_1_link | 1.37 | 1.219 | -- |
| wrist_2_link | 1.3 | 1.219 | -- |
| wrist_3_link | 0.365 | 0.1889 | MJCF 51% 가벼움 |

> URDF는 UR 공식 xacro 기반, MJCF는 MuJoCo Menagerie 기반이므로 mass/inertia 값에 차이가 있습니다.
> `ros2 run rtc_tools compare_mjcf_urdf` 명령으로 상세 비교가 가능합니다.

---

## 7. 메시 파일

### 시각화용 (visual/)

| 파일 | 형식 | 용도 |
|------|------|------|
| `base.dae` ~ `wrist3.dae` (7개) | Collada (DAE) | URDF 시각화. CAD 기반 고해상도 렌더링. |

### 충돌 감지용 (collision/)

| 파일 | 형식 | 용도 |
|------|------|------|
| `base.stl` ~ `wrist3.stl` (7개) | STL | URDF 충돌 감지. 단순화된 저폴리곤 형상. |

### MJCF 시각화용 (assets/)

| 파일 | 형식 | 용도 |
|------|------|------|
| `base_0.obj` ~ `wrist3.obj` (20개) | Wavefront OBJ | MJCF 시각화. 링크별 분할 메시 (base 2개, shoulder 3개, upperarm 4개, forearm 4개, wrist1 3개, wrist2 3개, wrist3 1개). |

**출처**: UR 공식 ROS2 Description 레포지토리
- GitHub: [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) (`humble` 브랜치)
- 라이선스: BSD-3-Clause (UR 공식)

---

## 빌드

### 사전 요구사항

```bash
# ROS2 Humble (Ubuntu 22.04)
sudo apt install -y ros-humble-ur-description ros-humble-xacro

# ROS2 Jazzy (Ubuntu 24.04)
sudo apt install -y ros-jazzy-ur-description ros-jazzy-xacro
```

### 빌드 명령

```bash
cd ~/ur_ws
colcon build --packages-select robot_descriptions --symlink-install
```

빌드 후 `install/robot_descriptions/share/robot_descriptions/robots/` 아래에 모든 모델 파일이 설치됩니다.

---

## 설치 경로 (빌드 후)

```bash
# ament_index로 share 경로 획득
$(ros2 pkg prefix robot_descriptions)/share/robot_descriptions/

# MJCF
robots/ur5e/mjcf/ur5e.xml                       # 로봇 모델
robots/ur5e/mjcf/scene.xml                      # MuJoCo 시뮬레이션 진입점 (로봇 단독)
robots/assm_v1/mjcf/hand.xml                   # 핸드 단독 모델
robots/ur5e_assm_v1/mjcf/ur5e_with_hand.xml    # 로봇 + 핸드 모델
robots/ur5e_assm_v1/mjcf/scene_with_hand.xml   # MuJoCo 시뮬레이션 진입점 (로봇 + 핸드)

# URDF
robots/ur5e/urdf/ur5e.urdf                      # Pinocchio 진입점 (로봇 팔만)
robots/assm_v1/urdf/hand.urdf.xacro            # 10-DOF 커스텀 핸드
robots/ur5e_assm_v1/urdf/ur5e_with_hand.urdf.xacro  # 로봇 + 핸드 조합

# Mesh
robots/ur5e/meshes/visual/*.dae                 # 시각화용 (7 files)
robots/ur5e/meshes/collision/*.stl              # 충돌용 (7 files)
robots/ur5e/meshes/assets/*.obj                 # MJCF 시각화용 (20 files)
```

---

## 사용하는 패키지

| 패키지 | 참조 파일 |
|--------|----------|
| `rtc_mujoco_sim` | `mjcf/scene.xml` (기본 model_path) |
| `rtc_controller_manager` + `rtc_controllers` | `urdf/ur5e.urdf` (Pinocchio 모델 빌드, FK/IK/Dynamics) |
| `ur5e_bringup` | launch 파일에서 URDF 경로 설정 |

---

## MJCF vs URDF 파라미터 비교

```bash
# 빌드 후
ros2 run rtc_tools compare_mjcf_urdf

# 직접 경로 지정
ros2 run rtc_tools compare_mjcf_urdf --mjcf robots/ur5e/mjcf/ur5e.xml --urdf robots/ur5e/urdf/ur5e.urdf

# tolerance 조정
ros2 run rtc_tools compare_mjcf_urdf --tolerance 0.01
```

---

## 의존성 그래프 내 위치

**독립 패키지** -- 외부 ROS2 패키지 런타임 의존성 없음 (URDF 생성에 `xacro`, `ur_description` 필요).

```
robot_descriptions  <-- 독립 (MJCF/URDF/메시 제공)
    ^
    |-- rtc_mujoco_sim         (MJCF scene.xml 참조, ament_index + package:// URI)
    |-- rtc_controller_manager (URDF ur5e.urdf 참조, Pinocchio 모델 빌드)
    |-- rtc_controllers        (URDF 경유 FK/IK/Dynamics 계산)
    +-- ur5e_bringup           (launch 파일에서 URDF 경로 설정)
```

---

## 변경 내역

### Unreleased

| 영역 | 변경 내용 |
|------|----------|
| **패키지 rename** | `ur5e_description` → `robot_descriptions`. 더 이상 robot-specific 패키지가 아니라 multi-robot data hub임을 이름이 반영. UR5e는 첫 입주자일 뿐. |
| **서브디렉토리 rename** | `robots/hand_tmp/` → `robots/assm_v1/`, `robots/ur5e_hand_tmp/` → `robots/ur5e_assm_v1/`. `_tmp` 접미사 제거 — 이제 정식 이름. |
| **README 톤 변경** | 1절을 robot-agnostic hub 선언으로 다시 작성. 추가 robot/hand 입주 절차(서브디렉토리 추가만으로 충분, 본 패키지 빌드 변경 없음) 명시. |

### v5.17.0

| 영역 | 변경 내용 |
|------|----------|
| **순수 데이터화** | `include/ur5e_description/ur5e_constants.hpp` 제거. `kNumHandMotors`/`kDefaultHandMotorNames`/`kDefaultFingertipNames` → `udp_hand_driver`, `HandState` → `udp_hand_driver/hand_state.hpp`. `RobotState`/`kNumHandJoints`/`kDefaultRobotJointNames`는 사용처가 없어 삭제. `rtc_base` 의존도 제거 — 이 패키지는 URDF/MJCF/mesh만 담는다. |
| **URDF 구조** | `ur5e.urdf` 사전 생성 포함, `hand.urdf.xacro`/`ur5e_with_hand.urdf.xacro` 추가 |
| **관절 사양** | shoulder_pan/lift 최대 속도 수정: 2.0944 -> 3.1416 rad/s (URDF 실제 값 반영) |
| **메시 에셋** | assets 디렉토리 파일 수 명확화: 20개 OBJ 파일 |

### v5.16.1

| 영역 | 변경 내용 |
|------|----------|
| **코드 검증** | 데이터 패키지 (URDF/MJCF/메시) -- 리소스 구조 확인 완료 |

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
