# robot_descriptions

로봇·핸드의 URDF · MJCF · mesh 를 모아 두는 robot-agnostic data hub. **유형**: 데이터 패키지

## 개요

C++ 헤더·CMake target·robot-specific 코드는 없고 `robots/<name>/` 하위 디렉토리에 자산 파일만 설치한다. 새 로봇은 디렉토리를 추가하는 것으로 끝나며 `CMakeLists.txt` · `package.xml` 은 손대지 않는다.

소비자는 `rtc_mujoco_sim` (MJCF scene), `rtc_controller_manager` · `rtc_controllers` (URDF 로 Pinocchio 모델 빌드), `integrated_bringup` (launch 의 URDF 경로), `rtc_digital_twin` (RViz) 이다.

### 현재 입주한 robots

목록의 SSoT 는 `robots/` 디렉토리다 (`ls robot_descriptions/robots`). 아래는 용도 요약이다.

| 디렉토리 | 내용 |
|---|---|
| `ur5e/` | UR5e 6-DoF arm — URDF, MJCF, mesh |
| `assm_v1/` | 10-DoF custom hand — URDF xacro, MJCF |
| `ur5e_assm_v1/` | UR5e + assm_v1 결합 — URDF xacro, MJCF scene |
| `iiwa7/` | KUKA iiwa7 7-DoF arm — URDF, MJCF, mesh |
| `leap_hand/` | LEAP Hand 16-DoF (left/right) — URDF, MJCF, mesh |
| `iiwa7_leap/` | iiwa7 + LEAP Hand 결합 (left/right) — URDF xacro, MJCF scene |
| `schunk_hand/` | Schunk SVH 5-finger hand (left/right) — URDF, mesh (MJCF 없음) |
| `panda/` | kinematics-only 테스트 fixture — [README](robots/panda/README.md) |
| `p1b_surrogate/` | 합성 폐쇄 체인 손 + UR5e — 테스트 fixture, [README](robots/p1b_surrogate/README.md) |
| `g1/` | G1 상체 + 합성 손 — 테스트 fixture, [README](robots/g1/README.md) |

로봇마다 필요한 자산만 갖는다 (hand 계열은 primitive geometry 라 mesh 가 없고, fixture 는 URDF 만 갖는다). 파일 목록은 직접 조회한다.

```bash
find robot_descriptions/robots \( -iname "*.urdf" -o -iname "*.urdf.xacro" \)   # URDF
find robot_descriptions/robots -iname "*.xml"                                    # MJCF
```

## 빠른 시작

빌드는 [루트 README](../README.md#빠른-시작) 를 따른다. 이 저장소는 `object_sim` (mesh 데이터, upstream submodule) 을 포함하므로 submodule 없이 클론했다면 먼저 채운다 — 절차는 [루트 README](../README.md#클론-submodule-포함) 에 있다. 일부 MuJoCo scene (예: `robots/iiwa7_leap/mjcf/scene_right_with_object.xml`) 이 이 mesh 를 쓴다.

설치 위치를 확인하고 URDF 를 읽어 본다.

```bash
SHARE=$(ros2 pkg prefix robot_descriptions)/share/robot_descriptions
ls $SHARE/robots
xacro $SHARE/robots/ur5e_assm_v1/urdf/ur5e_with_hand.urdf.xacro > /tmp/robot.urdf && check_urdf /tmp/robot.urdf
```

## 인터페이스

이 패키지는 노드·토픽을 갖지 않고 **파일 경로**가 인터페이스다. 소비자는 `package.xml` 에 `<exec_depend>robot_descriptions</exec_depend>` 만 선언하고 (`<depend>` · `find_package` · `ament_target_dependencies` 금지 — ARCH-5, [invariants.md](../agent_docs/invariants.md)), 실행 시점에 ament index 로 경로를 푼다.

| 소비 방식 | 형태 |
|---|---|
| C++ | `ament_index_cpp::get_package_share_directory("robot_descriptions")` |
| Python | `ament_index_python.packages.get_package_share_directory("robot_descriptions")` |
| URDF · MJCF · launch · YAML | `package://robot_descriptions/robots/<name>/...` |
| launch 인자 (`rtc_digital_twin`) | `robot_description_package:=robot_descriptions robot_description_path:=robots/<name>/urdf/...` |

mesh 는 URDF 안에서 `package://robot_descriptions/robots/<name>/meshes/...` 절대 URL 로 참조한다 (URDF 를 옮겨도 깨지지 않는다).

## 디렉토리

```
robot_descriptions/
├── robots/<name>/   robot · hand 별 자산
│   ├── urdf/        URDF · xacro (closure sidecar)
│   ├── mjcf/        MuJoCo 모델 · scene
│   └── meshes/      visual · collision mesh
├── objects/         이 저장소가 소유한 sim 물체
└── object_sim/      upstream submodule (수정 금지)
```

- `robots/model_pairs.yaml` — MJCF 와 URDF 의 짝 선언. `*.link_map.yaml` 은 두 파일이 같은 link 를 다르게 부를 때만 둔다.
- `objects/` 는 `object_sim` 과 같은 `<name>/object.xml` 레이아웃이라 `rtc_mujoco_sim` object_pool 의 `directory` 를 `package://robot_descriptions/objects` 로 지정하면 그대로 쓴다. upstream submodule 에는 추가할 수 없어 이 저장소 소유 물체는 여기에 둔다. 물체의 기하가 정책·overlay 와 맞물린 계약이므로, 옮기거나 고치기 전에 각 `object.xml` 의 주석을 읽는다.
- `ur5e/urdf/ur5e.urdf` 는 `ur_description` xacro 에서 미리 생성한 파일이다. 다시 만들려면 `ros-jazzy-ur-description` · `ros-jazzy-xacro` 가 필요하다.

### MJCF 와 URDF 일관성

sim(MJCF) 과 컨트롤러 모델(URDF) 이 어긋나면 튜닝한 게인이 모델 오차를 보상하게 되므로 URDF 를 SSoT 로 맞춘다. 비교 도구는 `ros2 run rtc_tools compare_mjcf_urdf` 이고 사용법은 [rtc_tools README](../rtc_tools/README.md) 에 있다. `robots/model_pairs.yaml` 의 모든 쌍을 [test_real_model_pairs.py](../rtc_tools/test/test_real_model_pairs.py) 가 자동 검사하므로, 이 패키지의 MJCF/URDF 를 고쳤다면 그 테스트가 sensor 다. 로봇을 새로 들이면 `model_pairs.yaml` 에 한 줄을 추가한다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| 로컬 `colcon test` 에서 `test_real_model_pairs` 가 skip | colcon 이 `/usr/bin/python3` 로 pytest 를 돌리는데 mujoco 는 `.venv` 에 있다 | [docs/testing.md](../docs/testing.md#test-commands) 의 venv 직접 실행 (`PYTHONPATH` 는 덮어쓰지 말고 이어붙인다 — xacro 쌍이 `import xacro` 를 한다) |
| MuJoCo scene 의 mesh 로드 실패 | `object_sim` submodule 이 비어 있다 | `git submodule update --init --recursive` |
| 새 소비자 패키지에서 colcon 이 불필요한 build 의존 edge 를 만든다 | `<depend>` 또는 `find_package(robot_descriptions)` 를 썼다 | `<exec_depend>` 로 바꾼다 (ARCH-5) |

## 더 읽기

- [invariants.md](../agent_docs/invariants.md) — ARCH-5 (data-only 패키지 소비 규칙)
- 로봇을 새로 들이는 절차: `adding-component` skill → [modification-guide.md](../agent_docs/modification-guide.md)
- 설치 규칙과 소비자 규칙 원문: [CMakeLists.txt](CMakeLists.txt) 머리 주석

## 의존성

[package.xml](package.xml) — 런타임 `xacro` 만 (xacro 파일을 여는 로봇에 한함). MIT. 로봇별 mesh 출처·라이선스는 각 디렉토리의 LICENSE 와 URDF 헤더를 따른다.
