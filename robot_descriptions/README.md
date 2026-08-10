# robot_descriptions


> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md)

---

## 1. 패키지 이름 및 설명

| 항목 | 내용 |
|------|------|
| **패키지 이름** | `robot_descriptions` |
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
| `robots/iiwa7/` | KUKA iiwa7 7-DoF arm — URDF (obj/glb 2종) + meshes |
| `robots/leap_hand/` | LEAP Hand 16-DoF — URDF (left/right) + meshes (collision-box composite inertia rebake 2026-05-15) |
| `robots/iiwa7_leap/` | iiwa7 arm + LEAP Hand 결합 — URDF xacro + MJCF (left/right 각 2종), scene 각 1종. `ee_link` 말단 부착. `meshes/{visual,collision}/`은 iiwa7 + leap_hand mesh hardlink |
| `robots/schunk_hand/` | Schunk SVH 5-finger hand — URDF (left/right × obj/glb 4종) + meshes |
| `robots/panda/` | Franka Emika Panda 7-DoF — **kinematics-only 테스트 fixture** (`urdf/panda.urdf`만, meshes 없음). rtc_tsid / rtc_mpc / integrated_bringup gtest 가 generic 7-DoF 모델로 `pinocchio::buildModel` 에 사용. 상세·출처: `robots/panda/README.md` |

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
    ├── ur5e_assm_v1/                      # UR5e + assm_v1 통합
    │   ├── mjcf/
    │   │   ├── ur5e_with_hand.xml         # 로봇 + 핸드 통합 MJCF
    │   │   └── scene_with_hand.xml        # 씬 (지면 + 조명 + 통합 모델)
    │   └── urdf/
    │       └── ur5e_with_hand.urdf.xacro  # 로봇 + 핸드 결합 xacro
    │
    ├── iiwa7/                             # KUKA iiwa7 7-DoF arm
    │   ├── mjcf/                          # iiwa7.xml + scene.xml
    │   ├── urdf/
    │   │   └── iiwa7.urdf                 # OBJ mesh 참조
    │   └── meshes/{visual,collision}/
    │
    ├── leap_hand/                         # LEAP Hand 16-DoF
    │   ├── mjcf/                          # leap_hand_{left,right}.xml + scene_{left,right}.xml
    │   ├── urdf/
    │   │   └── leap_hand_{left,right}.urdf
    │   └── meshes/{visual,collision}/
    │
    ├── iiwa7_leap/                        # iiwa7 + LEAP Hand 통합
    │   ├── urdf/
    │   │   ├── iiwa7_with_leap_left.urdf.xacro    # iiwa7 + leap_hand_left
    │   │   └── iiwa7_with_leap_right.urdf.xacro   # iiwa7 + leap_hand_right
    │   ├── mjcf/                                  # urdf_to_mjcf 변환 산출
    │   │   ├── iiwa7_with_leap_{left,right}.xml   # 로봇 본체 MJCF
    │   │   └── scene_{left,right}.xml             # 씬 (floor + light + skybox, right 는 scene_right(_with_object).xml 2종)
    │   └── meshes/{visual,collision}/             # iiwa7/leap_hand mesh hardlink
    │
    ├── schunk_hand/                       # Schunk SVH 5-finger hand (mjcf 없음)
    │   ├── urdf/
    │   │   └── schunk_svh_hand_{left,right}.urdf
    │   └── meshes/{visual,collision}/
    │
    └── panda/                             # Franka Panda 7-DoF (kinematics-only test fixture)
        └── urdf/
            └── panda.urdf                 # meshes 없음 — pinocchio::buildModel 전용
```

**Mesh 참조 규약**: 새로 입주하는 robot의 URDF는 mesh를 `package://robot_descriptions/robots/<name>/meshes/...` 절대 URL로 참조한다 (URDF가 어느 디렉토리로 옮겨져도 깨지지 않음). `ur5e/`는 ROS 표준 dependency 호환을 위해 ament share 경로 기반의 기존 패턴을 유지한다.

---

## 4. 자산 레이아웃 패턴

각 `robots/<name>/` 서브디렉토리는 그 로봇에 실제로 필요한 자산만 보유합니다 — 모든 로봇이 URDF+MJCF+mesh 3종을 다 갖추는 것은 아닙니다 (예: `panda`는 kinematics-only URDF만, `assm_v1`/`ur5e_assm_v1`은 primitive geometry만이라 mesh 없음).

| 로봇 | `urdf/` | `mjcf/` | `meshes/` |
|------|:---:|:---:|:---:|
| `ur5e` | O | O | O |
| `assm_v1` | O (xacro) | O | - |
| `ur5e_assm_v1` | O (xacro) | O | - |
| `iiwa7` | O | O | O |
| `leap_hand` | O | O | O |
| `iiwa7_leap` | O (xacro) | O | O (iiwa7 + leap_hand mesh hardlink) |
| `schunk_hand` | O | - | O |
| `panda` | O (kinematics-only) | - | - |

새 robot/파일 추가·삭제 시 이 표가 바로 stale 해지므로, 정확한 목록은 항상 직접 조회합니다:

```bash
find robot_descriptions/robots \( -iname "*.urdf" -o -iname "*.urdf.xacro" \)   # URDF
find robot_descriptions/robots -iname "*.xml"                                    # MJCF
```

**Mesh 참조 규약**: 새로 입주하는 robot의 URDF는 mesh를 `package://robot_descriptions/robots/<name>/meshes/...` 절대 URL로 참조합니다 (URDF가 어느 디렉토리로 옮겨져도 깨지지 않음). `ur5e/`는 ROS 표준 dependency 호환을 위해 ament share 경로 기반의 기존 패턴을 유지합니다.

---

## 5. 예시: UR5e / assm_v1 상세 사양

아래는 첫 입주자인 UR5e + assm_v1 hand 의 실측 사양입니다. **다른 로봇(iiwa7, leap_hand, iiwa7_leap, schunk_hand, panda)의 관절 사양·메시 구성·액추에이터 게인은 이 절의 수치와 다르므로, 각 URDF/MJCF 소스 또는 `ros2 run rtc_tools compare_mjcf_urdf`로 직접 확인하세요.**

### MJCF 액추에이터 설정 (`robots/ur5e/mjcf/ur5e.xml`, `robots/assm_v1/mjcf/hand.xml`)

- **UR5e 관절 (shoulder/elbow)**: `gainprm=2000`, `biasprm="0 -2000 -400"`, `forcerange=[-150, 150]`
- **UR5e 관절 (wrist)**: `gainprm=500`, `biasprm="0 -500 -100"`, `forcerange=[-28, 28]`
- **핸드 관절**: `gainprm=500`, `biasprm="0 -500 -100"`, `forcerange=[-5, 5]`

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

### 메시 파일 (UR5e)

#### 시각화용 (visual/)

| 파일 | 형식 | 용도 |
|------|------|------|
| `base.dae` ~ `wrist3.dae` (7개) | Collada (DAE) | URDF 시각화. CAD 기반 고해상도 렌더링. |

#### 충돌 감지용 (collision/)

| 파일 | 형식 | 용도 |
|------|------|------|
| `base.stl` ~ `wrist3.stl` (7개) | STL | URDF 충돌 감지. 단순화된 저폴리곤 형상. |

#### MJCF 시각화용 (assets/)

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

이 저장소는 `robot_descriptions/object_sim` (mesh 데이터, [vikashplus/object_sim](https://github.com/vikashplus/object_sim)) 을 git submodule 로 포함합니다. `--recursive` 없이 클론했다면 빌드 전에 `git submodule update --init --recursive` 로 채워야 일부 MuJoCo scene (예: `robots/iiwa7_leap/mjcf/scene_right_with_object.xml`) 의 mesh 로드가 성공합니다 — 상세 절차는 [루트 README "클론 (submodule 포함)"](../README.md#클론-submodule-포함) 참조.

### 빌드 명령

```bash
cd ~/ros2_ws/rtc_ws
colcon build --packages-select robot_descriptions --symlink-install
```

빌드 후 `install/robot_descriptions/share/robot_descriptions/robots/<name>/` 아래에 각 robot 의 모델 파일이 설치됩니다 (ament_index share 경로 획득: `$(ros2 pkg prefix robot_descriptions)/share/robot_descriptions/`). 정확한 설치 파일 목록은 §4 의 `find` 명령을 `install/robot_descriptions/share/robot_descriptions/robots` 에 대해 실행해 확인하세요.

---

## 사용하는 패키지

| 패키지 | 참조 파일 |
|--------|----------|
| `rtc_mujoco_sim` | `mjcf/scene.xml` (기본 model_path) |
| `rtc_controller_manager` + `rtc_controllers` | `urdf/ur5e.urdf` (Pinocchio 모델 빌드, FK/IK/Dynamics) |
| `integrated_bringup` | launch 파일에서 URDF 경로 설정 |

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

**`ur5e` 는 `--align-frames world base` 가 필요합니다** — 이 쌍은 두 파일의 world frame 이 다릅니다 (MJCF world = UR "Base"(DH) 프레임, URDF world = REP-103 `base_link`). 선언 없이 돌리면 관절 6개 전부 x 부호가 뒤집힌 것처럼 보이는데, 그건 모델 발산이 아니라 mounting 규약입니다 (#392). 옵션 의미·이름이 엇갈리는 함정은 [rtc_tools/README.md](../rtc_tools/README.md#compare_mjcf_urdfpy--mjcf-vs-urdf-파라미터-비교-검증) 참조.

```bash
ros2 run rtc_tools compare_mjcf_urdf --align-frames world base \
    --mjcf robots/ur5e/mjcf/ur5e.xml --urdf robots/ur5e/urdf/ur5e.urdf
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
    +-- integrated_bringup           (launch 파일에서 URDF 경로 설정)
```

---

## 라이선스

MIT License -- 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
