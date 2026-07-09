# VS Code Debugging Guide

이 문서는 `rtc-framework` 프로젝트에서 VS Code + GDB를 사용하여 C++ 노드를 디버깅하는 방법을 설명합니다.

> [!IMPORTANT]
> **`.vscode/` 는 리포에 체크인되지 않습니다.** `.gitignore` 에 `.vscode/` 가 등록되어 있어 (에디터 설정은 개발자 로컬 영역), 이 디렉토리는 **각자 생성**해야 합니다. 본 문서의 [§VS Code 설정 파일 구조](#vs-code-설정-파일-구조) 가 네 파일(`settings.json` / `tasks.json` / `launch.json` / `extensions.json`)의 복붙용 템플릿을 제공합니다 — VS Code 워크스페이스로 **repo 루트(`src/rtc-framework`)** 를 연다는 전제이며, colcon `build/` · `install/` 은 `${workspaceFolder}/../../` (ws 루트 `~/ros2_ws/rtc_ws`) 에 있습니다 (CLAUDE.md §9.1). `.clangd` 와 `merge_compile_commands.py` 는 repo 에 체크인되어 있습니다.

---

## 목차

1. [사전 요구사항](#1-사전-요구사항)
2. [디버그 빌드](#2-디버그-빌드)
3. [Launch 디버거 — 노드 직접 실행](#3-launch-디버거--노드-직접-실행)
4. [Attach 디버거 — 실행 중인 노드에 연결](#4-attach-디버거--실행-중인-노드에-연결)
5. [Breakpoint 사용법](#5-breakpoint-사용법)
6. [변수 및 메모리 검사](#6-변수-및-메모리-검사)
7. [GDB 콘솔 직접 사용](#7-gdb-콘솔-직접-사용)
8. [자주 발생하는 문제](#8-자주-발생하는-문제)
9. [VS Code 설정 파일 구조](#vs-code-설정-파일-구조)

---

## 1. 사전 요구사항

### 필수 패키지 설치

```bash
sudo apt install gdb
```

### VS Code 확장 설치

아래 [§VS Code 설정 파일 구조](#vs-code-설정-파일-구조)의 `extensions.json` 템플릿을 만들면 VS Code에서 `Ctrl+Shift+P` → `Show Recommended Extensions` → 일괄 설치할 수 있습니다.

| 확장 | 역할 |
|------|------|
| `llvm-vs-code-extensions.vscode-clangd` | **기본 IntelliSense**(cpptools 대신 사용) |
| `ms-vscode.cpptools` | **GDB 디버거** (IntelliSense는 비활성) |
| `ms-python.python` / `ms-python.debugpy` | Python 디버거 (launch 파일용) |
| `ms-vscode.cmake-tools` / `twxs.cmake` | CMake 지원 |
| `redhat.vscode-yaml` / `redhat.vscode-xml` | YAML/URDF/MJCF |
| `ms-iot.vscode-ros` | ROS 2 통합 |
| `smilerobotics.urdf` | URDF 뷰어 |

> [!NOTE]
> 본 프로젝트는 clangd를 기본 IntelliSense로 사용합니다(`C_Cpp.intelliSenseEngine: "disabled"`).
> cpptools는 **디버거 목적**으로만 활성화됩니다.

### GDB 실행 권한 확인

일부 시스템에서는 ptrace 보안 정책으로 인해 attach가 거부됩니다. 아래 명령으로 확인·해제합니다:

```bash
# 현재 설정 확인 (0=허용, 1=제한, 2=관리자만, 3=금지)
cat /proc/sys/kernel/yama/ptrace_scope

# 세션 내 임시 허용 (재부팅 후 원상복구)
echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope
```

> [!WARNING]
> `ptrace_scope=0`은 보안 수준을 낮춥니다. 개발 환경에서만 사용하고, 운영 환경에서는 복구하세요.

---

## 2. 디버그 빌드

디버거가 소스 코드와 실행 파일을 매핑하려면 반드시 **Debug 모드**로 빌드해야 합니다.

### 방법 A — VS Code Task 사용 (권장)

`Ctrl+Shift+B` → **`colcon: Build All (Debug)`** 선택

또는 `Ctrl+Shift+P` → `Run Task` → `colcon: Build All (Debug)`

### 방법 B — 터미널 직접 실행

```bash
cd /home/junho/ros2_ws/rtc_ws
source /opt/ros/${ROS_DISTRO}/setup.bash   # Humble 또는 Jazzy
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug \
               -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# clangd가 읽을 수 있도록 compile_commands.json을 워크스페이스에 병합
python3 src/rtc-framework/merge_compile_commands.py
```

> [!IMPORTANT]
> Release 빌드된 바이너리는 최적화(`-O2`/`-O3`)로 인해 **변수가 사라지거나 순서가 바뀌어** 디버깅이 부정확합니다. 반드시 Debug 빌드(`-g -O0`)로 사용하세요.

> [!TIP]
> `.vscode/tasks.json`의 **모든 빌드 태스크는 빌드 후 자동으로** `merge_compile_commands.py`를 실행합니다.
> 수동 빌드 시에만 위 파이썬 커맨드를 직접 실행하면 됩니다.

---

## 3. Launch 디버거 — 노드 직접 실행

VS Code가 프로세스를 직접 실행하면서 디버깅을 시작합니다. 아래 구성을 [§VS Code 설정 파일 구조](#vs-code-설정-파일-구조)의 `launch.json` 템플릿에 정의합니다:

| 구성 | 대상 바이너리 | 기본 `--params-file` |
|------|---------------|----------------------|
| `C++: Launch integrated_rt_controller (Debug)` | `install/integrated_bringup/lib/integrated_bringup/integrated_rt_controller` | `integrated_bringup/config/ur5e_hand/sim.yaml` |
| `C++: Launch mujoco_simulator_node (Debug)` | `install/rtc_mujoco_sim/lib/rtc_mujoco_sim/mujoco_simulator_node` | `integrated_bringup/config/ur5e_hand/mujoco_simulator.yaml` |
| `C++: Launch udp_hand_node (Debug)` | `install/udp_hand_driver/lib/udp_hand_driver/udp_hand_node` | `udp_hand_driver/config/udp_hand_node.yaml` |
| `C++: Launch bt_coordinator_node (Debug)` | `install/ur5e_bt_coordinator/lib/ur5e_bt_coordinator/bt_coordinator_node` | — |
| `C++: Launch shape_estimation_node (Debug)` | `install/shape_estimation/lib/shape_estimation/shape_estimation_node` | — |
| `C++: Attach to Node (Pick Process)` | (실행 중인 프로세스 선택) | — |
| `C++: Run GTest (Selected Package)` | 프롬프트로 GTest 바이너리 경로 입력 | — |
| `Python: Launch File (sim.launch.py)` | `ros2 launch integrated_bringup sim.launch.py` | — |
| `Python: Current File` | 현재 편집 중인 `.py` | — |

> [!NOTE]
> 위 표의 `--params-file` · launch 파일은 **기본 로봇 `ur5e_hand`** 기준입니다. 다른 로봇은 `sim_<robot>.launch.py` / `robot_<robot>.launch.py` (예: `sim_iiwa7_leap.launch.py`, `sim_ur5e_p1b.launch.py`) 를 쓰고 각자의 `config/<robot>/` 파라미터를 전달합니다 — 사용 가능한 조합은 [`integrated_bringup/launch/`](../integrated_bringup/launch/) 를 확인하세요.

### 3-1. `integrated_rt_controller` 노드 디버깅

1. **`F5`** 키 또는 사이드바 `Run and Debug` (▷ 아이콘) 클릭
2. 드롭다운에서 **`C++: Launch integrated_rt_controller (Debug)`** 선택
3. **`F5`** 눌러 시작

```
실행 흐름:
  preLaunchTask 실행 (colcon: Build All (Debug))
    → 빌드 완료
      → GDB가 integrated_rt_controller 프로세스 시작
        → Breakpoint에서 일시 정지
```

> [!NOTE]
> 이 설정은 `ros2 launch`를 우회하고 바이너리를 **직접** 실행합니다.
> 기본값으로 `integrated_bringup/config/ur5e_hand/sim.yaml`이 `--params-file`로 전달됩니다.
> 실로봇 설정(`ur5e_hand/robot.yaml`)으로 디버깅하려면 `launch.json`의 `args` 항목을 편집하세요.

### 3-2. `mujoco_simulator_node` 디버깅

동일한 방법으로 **`C++: Launch mujoco_simulator_node (Debug)`** 선택 후 `F5`.

### 3-3. GTest 단일 실행

`C++: Run GTest (Selected Package)`를 선택하면 GTest 바이너리 경로를 입력받아 그 테스트만 GDB로 실행합니다. 경로 예시:

```
${workspaceFolder}/../../build/rtc_base/test_seqlock
${workspaceFolder}/../../build/rtc_controllers/test_grasp_controller
${workspaceFolder}/../../build/rtc_tsid/test_tsid_wqp
```

전체 테스트 실행은 `Ctrl+Shift+P` → `Run Task` → **`colcon: Test All`** 또는 **`colcon: Test Selected Package`**.

---

## 4. Attach 디버거 — 실행 중인 노드에 연결

이미 `ros2 launch`로 실행 중인 노드에 디버거를 붙이는 방법입니다.
**실시간 로봇 시스템**에서 프로세스를 재시작하지 않고 디버깅할 때 특히 유용합니다.

### 4-1. 노드 실행

먼저 터미널에서 노드를 정상적으로 실행합니다:

```bash
cd /home/junho/ros2_ws/rtc_ws
source /opt/ros/${ROS_DISTRO}/setup.bash && source install/setup.bash  # Humble 또는 Jazzy
ros2 launch integrated_bringup robot.launch.py
```

### 4-2. VS Code에서 Attach

1. 사이드바 `Run and Debug` → **`C++: Attach to Node (Pick Process)`** 선택
2. **`F5`** 또는 ▷ 버튼 클릭
3. 프로세스 선택 팝업에서 **`integrated_rt_controller`** 검색 후 선택

```
Tip: 팝업에서 프로세스 이름을 타이핑하면 필터링됩니다.
예) "ur5e" 입력 → integrated_rt_controller 프로세스만 표시
```

### 4-3. 프로세스 ID로 직접 Attach (CLI)

프로세스 ID를 알고 있다면 터미널에서도 가능합니다:

```bash
# PID 확인
ros2 node list
ps aux | grep integrated_rt_controller

# GDB attach (참고용)
sudo gdb -p <PID>
```

> [!NOTE]
> Attach 모드는 프로세스가 이미 실행 중이므로, `preLaunchTask` (빌드)가 실행되지 않습니다.
> Attach 전에 노드가 **Debug 빌드**로 실행 중인지 확인하세요.

---

## 5. Breakpoint 사용법

Breakpoint 토글·Step/Continue 등 기본 조작(`F9` / `F5` / `F10` / `F11` / `Shift+F11`)과 우클릭 → `Edit Breakpoint` 로 여는 **조건부 Breakpoint**(`loop_count == 100`, `joint_pos[0] > 1.5` 같은 표현식)·**Hit Count**(N회 실행 후 정지)는 VS Code 표준 기능이므로 [공식 문서](https://code.visualstudio.com/docs/editor/debugging)를 참고하세요. 여기서는 **RT 제어에 특화된 주의점**만 다룹니다.

> [!WARNING]
> **실시간 루프(`control_rate`, 기본 500 Hz~최대 5 kHz)에서 Breakpoint 로 멈추는 것은 위험합니다.** 정지 동안 ROS 타이머가 쌓이거나 WatchDog / E-STOP 이 트리거되고, 재개 후 타이밍이 완전히 깨집니다. 특정 순간만 포착해야 할 때도 Hit Count/조건부 Breakpoint 로 멈추기보다 아래 **Logpoint** 를 우선하세요.

### Logpoint — 멈추지 않고 값 관찰 (RT 루프 권장)

Breakpoint 처럼 멈추지 않고 메시지만 출력하므로 루프 타이밍을 유지합니다.

1. 줄 번호 여백 우클릭 → `Add Logpoint`
2. 메시지 입력: `"joint_pos[0] = {joint_pos[0]}, time = {t}"` (`{}` 안에 변수)

> [!TIP]
> Logpoint 로도 부족하면 코드에 `RCLCPP_DEBUG` 추가 후 `rqt_console` 관찰, 또는 `data_logger` 로 파일 기록 후 오프라인 분석을 사용하세요 (§8 참고).

---

## 6. 변수 및 메모리 검사

Breakpoint 에서 멈췄을 때 `Run and Debug` 사이드바의 **Variables**(Locals/Globals/Registers)·**Watch**(표현식 등록)·**Call Stack** 패널과 변수 Hover 는 VS Code 표준 기능입니다. Watch 에는 `joint_pos[2]`, `*controller_ptr`, `state.q[0]` 같은 인덱싱·역참조·멤버 접근은 물론 `state.q[0] * (180.0 / 3.14159)`(rad→degree) 같은 수식도 넣을 수 있습니다.

---

## 7. GDB 콘솔 직접 사용

VS Code 하단 `DEBUG CONSOLE` 은 GDB MI 위에서 동작하므로, 순수 GDB 명령은 앞에 **`-exec`** 를 붙입니다 (`-exec print joint_pos[0]`, `-exec ptype state`, `-exec bt`, `-exec info threads` / `thread 2` 등 표준 GDB 명령 전체 사용 가능).

### Eigen / STL pretty-printing

모든 C++ launch 구성의 `setupCommands` 에 `-enable-pretty-printing` + `set print pretty on` 이 포함되어 있어(§launch.json) `std::vector` / `std::array` 및 Eigen 타입이 가독성 있게 표시됩니다. pretty-printer 가 안 붙는 raw 버퍼는 `-exec print q.data()@7`(포인터 + 개수)로 펼쳐 볼 수 있습니다.

---

## 8. 자주 발생하는 문제

### ❌ `Unable to open 'xxx.cpp': File not found`

소스 파일 경로를 찾지 못하는 경우입니다.

**원인**: 빌드 디렉토리와 소스 디렉토리 경로가 다르거나, Debug 빌드가 아닌 경우

**해결**:
```bash
# Debug 빌드 재실행
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

# GDB에 소스 경로 수동 등록 (DEBUG CONSOLE)
-exec set substitute-path /old/path /new/path
```

---

### ❌ `Operation not permitted` (Attach 실패)

ptrace 권한 문제입니다.

```bash
echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope
```

---

### ❌ 변수 값이 `optimized out`으로 표시

Release 빌드 또는 최적화 레벨 문제입니다.

**해결**: Debug 빌드로 재빌드 (`-DCMAKE_BUILD_TYPE=Debug`)

---

### ❌ Breakpoint에 빨간 원에 경고(⚠️) 표시

바이너리와 소스 파일이 불일치합니다. 소스 수정 후 빌드를 안 했을 때 발생합니다.

**해결**: `Ctrl+Shift+B` → `colcon: Build All (Debug)` 재실행

---

### ❌ `integrated_rt_controller` 노드가 팝업 목록에 없음 (Attach 시)

노드가 실행 중이지 않거나 이름이 다릅니다.

```bash
# 실행 중인 프로세스 확인
ps aux | grep -E "integrated_rt_controller|mujoco_simulator"

# ROS 2 노드 목록
ros2 node list
```

---

### ❌ Breakpoint에서 멈추면 RT 정기 tick 타이밍 깨짐

**실시간 루프** 디버깅 특이사항입니다. Breakpoint에서 멈추는 동안 ROS 타이머가 쌓이거나 WatchDog가 트리거될 수 있습니다.

**대안**:
1. **Logpoint** 사용 (멈추지 않고 로그 출력)
2. 코드에 `RCLCPP_DEBUG` 로그 추가 후 `ros2 run rqt_console rqt_console`로 관찰
3. `data_logger`를 활용해 파일로 기록 후 오프라인 분석

---

## VS Code 설정 파일 구조

`.vscode/` 는 `.gitignore` 로 추적되지 않는 **개발자 로컬 디렉토리**입니다 (에디터 설정을 팀에 강제하지 않기 위함). 아래 네 파일을 **직접 생성**하세요 — 각 subsection 에 복붙용 템플릿이 있습니다.

> [!NOTE]
> 템플릿의 전제: VS Code 워크스페이스로 **repo 루트** (`src/rtc-framework`) 를 엽니다. 따라서 `${workspaceFolder}` = repo 루트이고, colcon `build/` · `install/` 은 `${workspaceFolder}/../../` (ws 루트 `~/ros2_ws/rtc_ws`) 에 위치합니다.

```
.vscode/
├── settings.json       # clangd, 에디터, Python, ROS 설정
├── tasks.json          # colcon 빌드/테스트, compile_commands 병합, ros2 launch
├── launch.json         # GDB launch/attach 구성 (노드 5종 + GTest + Python)
└── extensions.json     # 권장 확장 목록
```

### `settings.json` 핵심 항목

| 키 | 값 | 의도 |
|----|-----|------|
| `C_Cpp.intelliSenseEngine` | `"disabled"` | cpptools IntelliSense 비활성 (clangd와 충돌 방지) |
| `clangd.arguments[--compile-commands-dir]` | `${workspaceFolder}` | `merge_compile_commands.py` 는 merged `compile_commands.json` 을 **repo 루트** (`.clangd` 옆) 에 출력 — `build/` 아님 |
| `editor.defaultFormatter` (C++) | `llvm-vs-code-extensions.vscode-clangd` | 저장 시 clang-format 적용 |
| `python.analysis.extraPaths` | `/opt/ros/jazzy/lib/python3.12/dist-packages` 외 | Pylance에서 `rclpy`/`rtc_tools` 인식 |
| `ros.distro` | `"jazzy"` | ROS 확장 기본 디스트로 |
| `files.exclude` / `files.watcherExclude` | `build`, `install`, `log`, `logging_data`, `.cache` | 탐색·워처 성능 |

```jsonc
// .vscode/settings.json
{
  "C_Cpp.intelliSenseEngine": "disabled",            // clangd 가 IntelliSense, cpptools 는 디버거 전용
  "clangd.arguments": ["--compile-commands-dir=${workspaceFolder}"],
  "[cpp]": { "editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd" },
  "python.analysis.extraPaths": ["/opt/ros/jazzy/lib/python3.12/dist-packages"],
  "ros.distro": "jazzy",
  "files.exclude":        { "**/build": true, "**/install": true, "**/log": true, "**/logging_data": true, "**/.cache": true },
  "files.watcherExclude": { "**/build/**": true, "**/install/**": true, "**/log/**": true, "**/logging_data/**": true, "**/.cache/**": true }
}
```

> [!NOTE]
> `c_cpp_properties.json`은 **의도적으로 생성하지 않습니다**.
> clangd가 `.clangd` (`CompilationDatabase: .`) + repo 루트 `compile_commands.json`을 직접 사용하므로 cpptools IntelliSense 설정은 불필요합니다.

### `tasks.json` 태스크 목록

| Label | 역할 |
|-------|------|
| `colcon: Build All (Debug)` | 전체 Debug 빌드 + `merge_compile_commands.py` 자동 실행 (**기본 빌드**, `Ctrl+Shift+B`) |
| `colcon: Build All (Release)` | 전체 Release 빌드 |
| `colcon: Build Selected Package` | `build.sh -p <pkg>`로 단일 패키지 빌드 (드롭다운 선택) |
| `colcon: Build Sim` / `Build Robot` | `build.sh sim` / `robot` |
| `colcon: Test All` / `Test Selected Package` | 전체 또는 단일 패키지 테스트 (`console_direct+`) |
| `rtc: Merge compile_commands.json` | 병합만 수동 실행 |
| `rtc: Clean Build Artifacts` | `rm -rf build install log` |
| `rtc: Check RT Setup` | `repo_scripts/scripts/check_rt_setup.sh --summary` |
| `ros2 launch: sim` / `robot` | MuJoCo 시뮬 / 실로봇 런치 (`robot_ip` 프롬프트) |

```jsonc
// .vscode/tasks.json (대표 태스크 — 위 표의 나머지는 동일 패턴으로 추가)
{
  "version": "2.0.0",
  "tasks": [
    { "label": "colcon: Build All (Debug)", "type": "shell",
      "command": "./build.sh -d",                  // full debug build; merge_compile_commands.py 자동 실행
      "options": { "cwd": "${workspaceFolder}" },
      "group": { "kind": "build", "isDefault": true }, "problemMatcher": ["$gcc"] },
    { "label": "colcon: Build All (Release)", "type": "shell", "command": "./build.sh -r",
      "options": { "cwd": "${workspaceFolder}" }, "problemMatcher": ["$gcc"] },
    { "label": "colcon: Build Selected Package", "type": "shell", "command": "./build.sh -d -p ${input:pkg}",
      "options": { "cwd": "${workspaceFolder}" }, "problemMatcher": ["$gcc"] },
    { "label": "rtc: Merge compile_commands.json", "type": "shell", "command": "python3 merge_compile_commands.py",
      "options": { "cwd": "${workspaceFolder}" } },
    { "label": "rtc: Check RT Setup", "type": "shell",
      "command": "./repo_scripts/scripts/check_rt_setup.sh --summary",
      "options": { "cwd": "${workspaceFolder}" } },
    { "label": "colcon: Test Selected Package", "type": "shell",
      "command": "colcon test --packages-select ${input:pkg} --event-handlers console_direct+",
      "options": { "cwd": "${workspaceFolder}/../.." } }   // colcon 은 반드시 ws 루트에서 (CLAUDE.md §9.1)
  ],
  "inputs": [ { "id": "pkg", "type": "promptString", "description": "Package name (e.g. rtc_base)" } ]
}
```

### `launch.json` 디버그 구성

[3. Launch 디버거](#3-launch-디버거--노드-직접-실행) 표 참고. 모든 C++ 구성은 `preLaunchTask: "colcon: Build All (Debug)"`로 빌드 후 실행되며, `setupCommands`에 `-enable-pretty-printing` / `set print pretty on` / `set print object on`이 포함되어 STL 및 Eigen 타입이 가독성 있게 표시됩니다.

```jsonc
// .vscode/launch.json (main 노드 + attach — 나머지 노드는 §3 표의 program/params 값을 같은 패턴에 대입)
{
  "version": "0.2.0",
  "configurations": [
    { "name": "C++: Launch integrated_rt_controller (Debug)", "type": "cppdbg", "request": "launch",
      "program": "${workspaceFolder}/../../install/integrated_bringup/lib/integrated_bringup/integrated_rt_controller",
      "args": ["--ros-args", "--params-file", "${workspaceFolder}/integrated_bringup/config/ur5e_hand/sim.yaml"],
      "cwd": "${workspaceFolder}/../..", "MIMode": "gdb", "preLaunchTask": "colcon: Build All (Debug)",
      "setupCommands": [
        { "text": "-enable-pretty-printing", "ignoreFailures": true },
        { "text": "set print pretty on" },
        { "text": "set print object on" } ] },
    { "name": "C++: Attach to Node (Pick Process)", "type": "cppdbg", "request": "attach",
      "program": "${workspaceFolder}/../../install/integrated_bringup/lib/integrated_bringup/integrated_rt_controller",
      "processId": "${command:pickProcess}", "MIMode": "gdb",
      "setupCommands": [ { "text": "-enable-pretty-printing", "ignoreFailures": true } ] }
  ]
}
```

> [!IMPORTANT]
> GDB 가 RT 노드의 공유 라이브러리를 찾으려면 **`source install/setup.bash` 된 터미널에서 VS Code 를 실행**하거나, launch 구성에 `environment` 로 `LD_LIBRARY_PATH` 를 주입해야 합니다 (ROS 환경 미설정 시 `error while loading shared libraries`).

### `extensions.json`

권장 확장: clangd, cpptools(디버거용), Python+debugpy+Pylance, CMake, YAML/XML/URDF, ROS. 아래 템플릿을 만들면 `Ctrl+Shift+P` → `Show Recommended Extensions`에서 일괄 설치 가능.

```jsonc
// .vscode/extensions.json
{
  "recommendations": [
    "llvm-vs-code-extensions.vscode-clangd",
    "ms-vscode.cpptools",
    "ms-python.python", "ms-python.debugpy",
    "ms-vscode.cmake-tools", "twxs.cmake",
    "redhat.vscode-yaml", "redhat.vscode-xml",
    "ms-iot.vscode-ros", "smilerobotics.urdf"
  ]
}
```

---

## 관련 파일

| 파일 | 역할 | 추적 |
|------|------|:--:|
| `.vscode/settings.json` | clangd, 에디터, Python, exclusion 설정 | 로컬 (gitignored) |
| `.vscode/tasks.json` | 빌드/테스트/런치 태스크 | 로컬 (gitignored) |
| `.vscode/launch.json` | GDB launch/attach 구성 | 로컬 (gitignored) |
| `.vscode/extensions.json` | 권장 확장 목록 | 로컬 (gitignored) |
| [.clangd](../.clangd) | clangd 컴파일 플래그 + `CompilationDatabase: .` (repo 루트 DB) | ✅ |
| [merge_compile_commands.py](../merge_compile_commands.py) | 패키지별 `compile_commands.json`을 repo 루트로 병합 | ✅ |
