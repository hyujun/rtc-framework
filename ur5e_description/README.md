# ur5e_description

![version](https://img.shields.io/badge/version-v5.16.0-blue)

> 이 패키지는 [UR5e RT Controller](../README.md) 워크스페이스의 일부입니다.
> 설치/빌드: [Root README](../README.md)

UR5e 로봇의 모델 description 파일 패키지입니다.

`rtc_mujoco_sim`, `rtc_controller_manager` + `rtc_controllers` 등 모든 패키지가 이 단일 소스에서 모델을 참조합니다.

---

## 구조

```
robots/ur5e/
├── mjcf/           # MuJoCo 물리 시뮬레이션 모델 (MJCF)
│   ├── ur5e.xml              # UR5e 로봇 MJCF (capsule geometry + position actuators)
│   ├── hand.xml              # 10-DOF 커스텀 핸드 MJCF (cylinder geometry)
│   ├── scene.xml             # 씬 파일 (지면 + 조명 + ur5e.xml include)
│   ├── scene_with_hand.xml   # 로봇+핸드 통합 씬
│   └── ur5e_with_hand.xml    # 로봇+핸드 통합 모델
│
├── urdf/           # Pinocchio / RViz / ros2_control용 URDF
│   └── ur5e.urdf   # ← 빌드 시 xacro로 자동 생성 (빌드 전 없음)
│
└── meshes/         # UR 공식 메시 파일
    ├── visual/     # DAE (Collada, CAD 기반 고해상도 시각화)
    │   ├── base.dae
    │   ├── shoulder.dae
    │   ├── upperarm.dae
    │   ├── forearm.dae
    │   ├── wrist1.dae
    │   ├── wrist2.dae
    │   └── wrist3.dae
    └── collision/  # STL (단순화, 충돌 감지용)
        ├── base.stl, shoulder.stl, upperarm.stl
        ├── forearm.stl, wrist1.stl, wrist2.stl, wrist3.stl
```

### 관절 사양

| 관절 | 타입 | 위치 한계 (rad) | 최대 속도 (rad/s) | 최대 토크 (Nm) |
|------|------|----------------|-------------------|----------------|
| shoulder_pan_joint | revolute | ±2π | 2.0944 | 150.0 |
| shoulder_lift_joint | revolute | ±2π | 2.0944 | 150.0 |
| elbow_joint | revolute | ±π | 3.1416 | 150.0 |
| wrist_1_joint | revolute | ±2π | 3.1416 | 28.0 |
| wrist_2_joint | revolute | ±2π | 3.1416 | 28.0 |
| wrist_3_joint | revolute | ±2π | 3.1416 | 28.0 |

### 링크 질량

| 링크 | URDF (kg) | MJCF (kg) | 비고 |
|------|-----------|-----------|------|
| base_link_inertia | 4.0 | 4.0 | 고정 베이스 |
| shoulder_link | 3.761 | 3.7 | — |
| upper_arm_link | 8.058 | 8.393 | MJCF +0.335 kg 차이 |
| forearm_link | 2.846 | 2.275 | MJCF -0.571 kg 차이 |
| wrist_1_link | 1.37 | 1.219 | — |
| wrist_2_link | 1.3 | 1.219 | — |
| wrist_3_link | 0.365 | 0.1889 | MJCF 51% 가벼움 |

> URDF와 MJCF 간 질량 차이가 있습니다. `ros2 run rtc_tools compare_mjcf_urdf`로 상세 비교 가능합니다.

---

### 커스텀 핸드 모델

10-DOF 커스텀 핸드 URDF가 포함되어 있습니다 (`hand.urdf.xacro`, `ur5e_with_hand.urdf.xacro`).

**관절 (10 revolute):**

| 핑거 | 관절 | 범위 | Effort |
|-------|------|------|--------|
| Thumb | thumb_cmc_aa, thumb_cmc_fe, thumb_mcp_fe | ±90° | 5.0 Nm |
| Index | index_mcp_aa, index_mcp_fe, index_dip_fe | ±90° | 5.0 Nm |
| Middle | middle_mcp_aa, middle_mcp_fe, middle_dip_fe | ±90° | 5.0 Nm |
| Ring | ring_mcp_fe | ±90° | 5.0 Nm |

**핑거팁 프레임:** `thumb_tip_link`, `index_tip_link`, `middle_tip_link`, `ring_tip_link` (fixed links)

---

## 빌드

### 사전 요구사항

```bash
# ROS2 Humble (Ubuntu 22.04)
sudo apt install -y ros-humble-ur-description ros-humble-xacro

# ROS2 Jazzy (Ubuntu 24.04)
sudo apt install -y ros-jazzy-ur-description ros-jazzy-xacro
```

### 빌드

```bash
cd ~/ur_ws
colcon build --packages-select ur5e_description --symlink-install
```

빌드 후 `install/ur5e_description/share/ur5e_description/robots/ur5e/urdf/ur5e.urdf` 생성됨.

---

## 설치 경로 (빌드 후)

```bash
# ament_index로 share 경로 획득
$(ros2 pkg prefix ur5e_description)/share/ur5e_description/

# MJCF
robots/ur5e/mjcf/scene.xml          # MuJoCo 진입점
robots/ur5e/mjcf/ur5e.xml           # 로봇 모델

# URDF (빌드 시 생성)
robots/ur5e/urdf/ur5e.urdf          # Pinocchio 진입점 (로봇 암만)
robots/ur5e/urdf/hand.urdf.xacro    # 10-DOF 커스텀 핸드
robots/ur5e/urdf/ur5e_with_hand.urdf.xacro  # 로봇 + 핸드 조합

# Mesh
robots/ur5e/meshes/visual/*.dae     # 시각화용 (7 files)
robots/ur5e/meshes/collision/*.stl  # 충돌용 (7 files)
robots/ur5e/meshes/assets/*.obj     # MJCF 시각화용 (24 files)
```

---

## 사용하는 패키지

| 패키지 | 파일 |
|--------|------|
| `rtc_mujoco_sim` | `mjcf/scene.xml` (기본 model_path) |
| `rtc_controller_manager` + `rtc_controllers` (Pinocchio/CLIK/OSC 컨트롤러) | `urdf/ur5e.urdf` |

---

## MJCF vs URDF 파라미터 비교

두 모델 파일(MJCF, URDF)의 물리 파라미터(mass, inertia, joint limit, effort 등)를 비교하려면:

```bash
# 빌드 후
ros2 run rtc_tools compare_mjcf_urdf

# 직접 경로 지정
ros2 run rtc_tools compare_mjcf_urdf --mjcf robots/ur5e/mjcf/ur5e.xml --urdf robots/ur5e/urdf/ur5e.urdf

# tolerance 조정
ros2 run rtc_tools compare_mjcf_urdf --tolerance 0.01
```

> **참고**: MJCF는 MuJoCo Menagerie 기반, URDF는 UR 공식 xacro 생성이므로 mass/inertia 값이 다를 수 있습니다. joint limit과 effort limit은 일치합니다.

---

## 메시 출처

`meshes/` 파일은 UR 공식 ROS2 Description 레포지토리에서 다운로드:
- **GitHub**: [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) (`humble` 브랜치)
- **라이선스**: BSD-3-Clause (UR 공식)

## URDF 출처

UR 공식 `ur_description` 패키지의 `ur.urdf.xacro`를 `ur_type:=ur5e`로 xacro 변환.

---

## 의존성 그래프 내 위치

**독립 패키지** — 외부 ROS2 패키지 의존성 없음 (URDF 생성에 `xacro`, `ur_description` 필요).

```
ur5e_description  ← 독립 (MJCF/URDF/메시 제공)
    ↑
    ├── rtc_mujoco_sim         (MJCF scene.xml 참조, ament_index + package:// URI)
    ├── rtc_controller_manager (URDF ur5e.urdf 참조, Pinocchio 모델 빌드)
    ├── rtc_controllers        (URDF 경유 FK/IK/Dynamics 계산)
    └── ur5e_bringup           (launch 파일에서 URDF 경로 설정)
```

---

## 라이선스

MIT License — 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.
