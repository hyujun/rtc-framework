# ur5e_description

UR5e 로봇의 모델 description 파일 패키지 (v5.2.2).

`ur5e_mujoco_sim`, `ur5e_rt_controller` 등 모든 패키지가 이 단일 소스에서 모델을 참조합니다.

---

## 구조

```
robots/ur5e/
├── mjcf/           # MuJoCo 물리 시뮬레이션 모델 (MJCF)
│   ├── ur5e.xml    # UR5e 로봇 MJCF (capsule geometry + position actuators)
│   └── scene.xml   # 씬 파일 (지면 + 조명 + ur5e.xml include)
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

---

## 빌드

### 사전 요구사항

```bash
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
robots/ur5e/urdf/ur5e.urdf          # Pinocchio 진입점

# Mesh
robots/ur5e/meshes/visual/*.dae
robots/ur5e/meshes/collision/*.stl
```

---

## 사용하는 패키지

| 패키지 | 파일 |
|--------|------|
| `ur5e_mujoco_sim` | `mjcf/scene.xml` (기본 model_path) |
| `ur5e_rt_controller` (Pinocchio/CLIK/OSC 컨트롤러) | `urdf/ur5e.urdf` |

---

## 메시 출처

`meshes/` 파일은 UR 공식 ROS2 Description 레포지토리에서 다운로드:
- **GitHub**: [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) (`humble` 브랜치)
- **라이선스**: BSD-3-Clause (UR 공식)

## URDF 출처

UR 공식 `ur_description` 패키지의 `ur.urdf.xacro`를 `ur_type:=ur5e`로 xacro 변환.
