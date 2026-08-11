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

### 링크 질량 (URDF = MJCF, 합계 21.7 kg)

| 링크 | 질량 (kg) |
|------|-----------|
| base_link_inertia (MJCF: `base`) | 4.0 |
| shoulder_link | 3.761 |
| upper_arm_link | 8.058 |
| forearm_link | 2.846 |
| wrist_1_link | 1.37 |
| wrist_2_link | 1.3 |
| wrist_3_link | 0.365 |

> **URDF 가 SSoT 다** (#392 결정 #1). URDF 는 UR 공식 `ur_description` 의 `ur.urdf.xacro` 에서 기계 생성된 것이고, mass·inertia·kinematics 가 `config/ur5e/{physical_parameters,default_kinematics}.yaml` 과 일치한다. MJCF (MuJoCo Menagerie 유래) 는 한때 **UR5e 가 아니라 레거시 UR5 (CB3) 의 관성 세트**를 싣고 있었고 (shoulder 3.7 / upper_arm 8.393 / forearm 2.275 / wrist 1.219·1.219·0.1889), DH 길이도 소수 3자리로 반올림돼 있었다. 그 상태의 실측 비용은 중력 토크 상대오차 mean 14.5% · p95 24.9%, tool 위치 오차 최대 1.49 mm 였다 — sim(MJCF)과 컨트롤러 모델(URDF)이 어긋나므로 sim 에서 튜닝한 게인이 존재하지 않는 모델 오차를 보상하게 된다. 현재는 두 MJCF (`ur5e/mjcf/ur5e.xml`, `ur5e_assm_v1/mjcf/ur5e_with_hand.xml`) 가 URDF 에 정렬돼 있다.
>
> **MJCF 에만 있는 것**: 6관절 전부의 `armature=0.1` (로터 관성, URDF 무대응) — 의도적이며 수정 대상이 아니다. **프레임 배치 규약은 다르게 유지**한다 — MJCF 는 body frame 을 UR 이 visual mesh 를 놓는 자리(`shoulder_offset=0.138`, `elbow_offset=0.007`)에, URDF 는 DH frame 에 놓는다. 축 직선이 같으면 물리가 같으므로 정상이며, 이 차이 때문에 `compare_mjcf_urdf` 는 관절 원점을 점이 아니라 **축 직선**으로 비교한다.
>
> `ros2 run rtc_tools compare_mjcf_urdf --align-frames world base` 로 상세 비교가 가능합니다 (ur5e 는 `--align-frames` 필수 — 위 §"MJCF vs URDF 파라미터 비교" 참조).

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

**`ur5e` 는 선언 두 개가 필요합니다** (둘 다 이름·프레임이 엇갈려 자동 탐지로 풀리지 않습니다, #392):

- **`--align-frames world base`** — 두 파일의 world frame 이 다릅니다 (MJCF world = UR "Base"(DH) 프레임, URDF world = REP-103 `base_link`). 없으면 관절 6개 전부 x 부호가 뒤집힌 것처럼 보이는데, 모델 발산이 아니라 mounting 규약입니다.
- **`--link-map robots/ur5e/ur5e.link_map.yaml`** — MJCF body `base` 는 URDF `base_link_inertia`(4 kg) 인데 URDF 에도 **massless** `base` 프레임이 따로 있습니다. 자동 탐지는 이름이 같은 쌍만 맺으므로 그 둘을 잘못 짝지어 `base_link_inertia` 를 "mass=4 kg lost" 로 보고했습니다. 선언하면 base 링크 관성 검증이 켜지고(양쪽 값은 이미 동일) 허위 손실 보고가 사라집니다. 이 파일은 **예외 목록**이라 여기 없는 동명 쌍도 그대로 비교됩니다 (#411) — 일부만 적어도 비교 범위가 좁아지지 않습니다.

```bash
ros2 run rtc_tools compare_mjcf_urdf \
    --mjcf robots/ur5e/mjcf/ur5e.xml --urdf robots/ur5e/urdf/ur5e.urdf \
    --align-frames world base --link-map robots/ur5e/ur5e.link_map.yaml \
    --tip-frames attachment_site tool0 --fail-on-unverified
# -> Mismatches: 0  (Warnings: 2 — 아래 참조)
```

**이 비교는 자동으로 돕니다** — [`robots/model_pairs.yaml`](robots/model_pairs.yaml) 이 **8쌍**(단독 5 + 팔·손 조합 3)과 각 쌍의 선언을 갖고, [`rtc_tools/test/test_real_model_pairs.py`](../rtc_tools/test/test_real_model_pairs.py) 가 그걸 읽어 발사합니다. **여기의 MJCF/URDF 를 고쳤다면 그 테스트가 해당 sensor 입니다.** 로봇을 새로 들이면 `model_pairs.yaml` 에 한 줄 추가하세요 — 어떤 MJCF 가 어떤 URDF 와 짝인지는 자동 탐지로 추측할 수 없습니다 (`ur5e` 는 `ur5e.xml`·`scene.xml` 둘을 갖습니다). URDF 가 `*.urdf.xacro` 면 게이트가 테스트 시점에 확장하므로 그대로 적으면 됩니다 (#414) — 확장 실패는 skip 이 아니라 **테스트 실패**로 드러납니다. **팔+손 조합 모델이 특히 중요합니다**: 단독 모델만 검사하면 두 모델이 각각 맞으면서 결합부만 틀린 상태가 통과합니다.

> ⚠️ **로컬 `colcon test` 에서는 이 게이트가 skip 됩니다.** colcon 이 pytest 를 `/usr/bin/python3` 로 돌리는데 (colcon 자체의 shebang) 이 저장소는 mujoco 를 `.venv` 에 둡니다. CI 는 `pip install ... mujoco` 를 하므로 실제로 실행됩니다. 로컬에서 직접 돌리려면:
> ```bash
> PYTHONPATH=rtc_tools:$PYTHONPATH .venv/bin/python -m pytest rtc_tools/test/test_real_model_pairs.py
> ```
> `PYTHONPATH` 는 **덮어쓰지 말고 이어붙이세요** — xacro 쌍이 `import xacro` 를 하는데 그 모듈은 ROS 가 `PYTHONPATH` 로만 노출하므로, 대입하면 `ModuleNotFoundError: xacro` 가 납니다 (#414).

남는 warning 2건은 정당한 차이라 유지합니다: MJCF 가 링크당 visual mesh 를 쪼개고(20 vs 14) collision 을 capsule 로 따로 두기 때문입니다(29 vs 14).

### assm_v1 hand 의 선언 (`assm_v1.link_map.yaml` · `ur5e_assm_v1.link_map.yaml`)

hand 쪽은 두 가지가 더 필요합니다 (#413):

- **`hand_base: hand_base_link`** — palm 도 이름이 엇갈립니다.
- **`fuse:`** — 손끝 4개(`*_tip_link`, 각 0.01 kg)는 MJCF 에 별도 body 가 없고 부모 distal 에 접혀 있습니다(0.02 → 0.03 kg). 정당한 병합이며, 평행축 합성이 MJCF 값과 정확히 일치함을 확인했습니다 (#412: `m=0.03`, `com_z=0.0208333`, `Ixx=5.26667e-6`, `Izz=2e-7`). 선언하면 합성값으로 비교되고, 선언 없이는 "질량 손실 + 부모 과중" 2중 오탐이 납니다.

> **palm 질량이 MJCF 에 없었습니다** (#413). URDF `hand_base_link` 는 0.3 kg 인데 조합 MJCF 의 `hand_base` 에는 `<inertial>` 이 없어 geom density 로 0.2 kg 이 추론됐고, 단독 `hand.xml` 은 palm 이 worldbody geom 이라 아예 0 kg (손 전체 0.54 kg 중 56% 소실) 이었습니다. box 치수·위치는 양쪽이 정확히 일치했으므로 형상은 맞고 관성 선언만 빠진 상태였습니다. 실측 비용: **arm 중력 토크 최대 0.93 Nm (6.42%) · mean 0.40 Nm (1.41%)** — digital twin 이 로드하는 모델이라 sim 에서 튜닝한 게인이 이 오차를 보상하게 됩니다. 현재는 양쪽 MJCF 모두 URDF 를 미러합니다. 다만 URDF 의 `diag(1e-4, 1e-4, 1e-4)` 자체가 등방 placeholder 이며(균질 박스 계산값은 `diag(2.66e-4, 1.76e-4, 4.10e-4)`), 그 사실은 plausibility warning 이 계속 알립니다 — 텐서를 물리값으로 고치는 것은 URDF(SSoT)를 바꾸는 일이라 별도 축입니다.

**massless 프레임은 mismatch 로 세지 않습니다** — `base_link`·`flange`·`ft_frame`·`tool0`·`world`·`base` 는 질량 0 의 순수 좌표 프레임이고 MuJoCo 가 body 를 안 만드는 것이 정상 동작입니다. 다만 **MuJoCo 가 실제로 만들지 않은 것만** 면제하며(iiwa7 은 1개, leap_hand 는 5개의 massless 프레임을 실제 body 로 갖고 있습니다), **질량을 가진 링크가 사라지면 여전히 mismatch** 입니다 — fusestatic 이 질량을 부모로 흡수한 경우가 그것이고, 그 신호는 그대로 남습니다 (#385).

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
