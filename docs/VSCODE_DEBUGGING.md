# VS Code Debugging Guide

이 문서는 `rtc-framework` 프로젝트에서 VS Code + GDB를 사용하여 C++ 노드를 디버깅하는 방법을 설명합니다.

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

---

## 1. 사전 요구사항

### 필수 패키지 설치

```bash
sudo apt install gdb
```

### VS Code 확장 설치

VS Code에서 `Ctrl+Shift+X` → 다음 확장 설치 (또는 `Ctrl+Shift+P` → `Show Recommended Extensions`):

| 확장 | 역할 |
|------|------|
| `ms-vscode.cpptools` | C++ IntelliSense + GDB 디버거 |
| `ms-vscode.cpptools-extension-pack` | C++ 도구 묶음 |
| `ms-iot.vscode-ros` | ROS 2 통합 |

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
cd /home/user/ros2_ws/ur5e_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug \
               -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

> [!IMPORTANT]
> Release 빌드된 바이너리는 최적화(`-O2`/`-O3`)로 인해 **변수가 사라지거나 순서가 바뀌어** 디버깅이 부정확합니다. 반드시 Debug 빌드(`-g -O0`)로 사용하세요.

---

## 3. Launch 디버거 — 노드 직접 실행

VS Code가 프로세스를 직접 실행하면서 디버깅을 시작합니다.

### 3-1. `rt_controller` 노드 디버깅

1. **`F5`** 키 또는 사이드바 `Run and Debug` (▷ 아이콘) 클릭
2. 드롭다운에서 **`C++: Launch rt_controller (Debug)`** 선택
3. **`F5`** 눌러 시작

```
실행 흐름:
  preLaunchTask 실행 (colcon: Build All (Debug))
    → 빌드 완료
      → GDB가 rt_controller 프로세스 시작
        → Breakpoint에서 일시 정지
```

> [!NOTE]
> 이 설정은 `ros2 launch`를 우회하고 바이너리를 **직접** 실행합니다.
> 따라서 ROS 2 파라미터(YAML) 로딩이 되지 않을 수 있습니다.
> 파라미터가 필요하면 `"args"` 필드에 `--ros-args`, `-p` 등의 인수를 추가하세요.

**`launch.json` `args` 예시 (파라미터 전달):**

```json
"args": [
  "--ros-args",
  "--params-file", "/home/user/ros2_ws/ur5e_ws/src/rtc-framework/rtc_controller_manager/config/rt_controller_manager.yaml"
]
```

### 3-2. `mujoco_simulator_node` 디버깅

동일한 방법으로 **`C++: Launch mujoco_simulator_node (Debug)`** 선택 후 `F5`.

---

## 4. Attach 디버거 — 실행 중인 노드에 연결

이미 `ros2 launch`로 실행 중인 노드에 디버거를 붙이는 방법입니다.
**실시간 로봇 시스템**에서 프로세스를 재시작하지 않고 디버깅할 때 특히 유용합니다.

### 4-1. 노드 실행

먼저 터미널에서 노드를 정상적으로 실행합니다:

```bash
cd /home/user/ros2_ws/ur5e_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch ur5e_bringup robot.launch.py
```

### 4-2. VS Code에서 Attach

1. 사이드바 `Run and Debug` → **`C++: Attach to Node (Pick Process)`** 선택
2. **`F5`** 또는 ▷ 버튼 클릭
3. 프로세스 선택 팝업에서 **`rt_controller`** 검색 후 선택

```
Tip: 팝업에서 프로세스 이름을 타이핑하면 필터링됩니다.
예) "custom" 입력 → rt_controller 프로세스만 표시
```

### 4-3. 프로세스 ID로 직접 Attach (CLI)

프로세스 ID를 알고 있다면 터미널에서도 가능합니다:

```bash
# PID 확인
ros2 node list
ps aux | grep rt_controller

# GDB attach (참고용)
sudo gdb -p <PID>
```

> [!NOTE]
> Attach 모드는 프로세스가 이미 실행 중이므로, `preLaunchTask` (빌드)가 실행되지 않습니다.
> Attach 전에 노드가 **Debug 빌드**로 실행 중인지 확인하세요.

---

## 5. Breakpoint 사용법

### 기본 Breakpoint

소스 코드 줄 번호 왼쪽 여백을 클릭하면 빨간 원(●)이 생깁니다.

```
단축키:
  F9          — 현재 줄에 Breakpoint 토글
  F5          — 다음 Breakpoint까지 계속 실행 (Continue)
  F10         — 한 줄 실행, 함수 진입 안 함 (Step Over)
  F11         — 한 줄 실행, 함수 내부로 진입 (Step Into)
  Shift+F11   — 현재 함수에서 빠져나옴 (Step Out)
  Shift+F5    — 디버깅 종료
```

### 조건부 Breakpoint (Conditional)

특정 조건일 때만 멈추게 합니다.

1. Breakpoint 위에서 **우클릭** → `Edit Breakpoint`
2. 조건식 입력 예시:

```cpp
// 예: 100번째 루프에서만 멈춤
loop_count == 100

// 예: 특정 관절 각도 초과 시 멈춤
joint_pos[0] > 1.5

// 예: 포인터가 null일 때 멈춤
ptr == nullptr
```

### Hit Count Breakpoint

N번 실행된 후 처음으로 멈춥니다.

1. Breakpoint 우클릭 → `Edit Breakpoint` → `Hit Count` 선택
2. 숫자 입력 (예: `500` → 500번 실행 후 정지)

실시간 제어 루프(1kHz)에서 **특정 순간만 포착**할 때 유용합니다.

### Logpoint (실행을 멈추지 않고 로그 출력)

Breakpoint처럼 멈추지 않고, 대신 메시지를 출력합니다.

1. 줄 번호 여백 우클릭 → `Add Logpoint`
2. 메시지 입력: `"joint_pos[0] = {joint_pos[0]}, time = {t}"` (`{}` 안에 변수)

> [!TIP]
> 실시간 루프에서 Breakpoint는 루프 타이밍을 완전히 망가뜨립니다.
> 타이밍을 유지하면서 값을 관찰하려면 **Logpoint**를 사용하세요.

---

## 6. 변수 및 메모리 검사

디버거가 Breakpoint에서 멈췄을 때 사용할 수 있는 기능입니다.

### Variables 패널

왼쪽 `Run and Debug` 사이드바 상단에 자동으로 표시됩니다.

- **Locals**: 현재 함수의 지역 변수
- **Globals**: 전역 변수
- **Registers**: CPU 레지스터 값

### Watch 표현식

원하는 표현식을 등록해서 실시간으로 값을 추적합니다.

1. `Watch` 패널 → `+` 클릭
2. 표현식 입력:

```cpp
// 배열 전체 표시
joint_pos

// 특정 인덱스
joint_pos[2]

// 포인터 역참조
*controller_ptr

// 멤버 접근
state.q[0]

// 수식
state.q[0] * (180.0 / 3.14159)   // rad → degree 변환
```

### Hover로 즉시 확인

소스 코드에서 **변수 위에 마우스를 올리면** 현재 값이 팝업으로 표시됩니다.

### Call Stack

현재 함수가 어떤 호출 경로로 실행됐는지 보여줍니다.
`CALL STACK` 패널에서 이전 프레임을 클릭하면 해당 시점의 변수도 볼 수 있습니다.

---

## 7. GDB 콘솔 직접 사용

VS Code 하단 `DEBUG CONSOLE` 탭에서 GDB 명령어를 직접 실행할 수 있습니다.

> [!NOTE]
> VS Code의 DEBUG CONSOLE은 GDB MI(Machine Interface) 위에서 동작합니다.
> 순수 GDB 명령어를 쓰려면 앞에 `-exec`를 붙이거나 `-interpreter-exec console` 형식을 사용합니다.

### 자주 쓰는 DEBUG CONSOLE 명령

```
// 변수 출력
-exec print joint_pos[0]
-exec print *controller_ptr

// 타입 확인
-exec ptype state

// 메모리 덤프 (addr 기준 10개 double 출력)
-exec x/10g &joint_pos[0]

// 역참조 배열 출력
-exec print joint_pos[0]@7        // index 0부터 7개 출력

// 함수 호출
-exec call compute_torque(state)

// 현재 스레드 목록
-exec info threads

// 특정 스레드로 전환
-exec thread 2

// 백트레이스 (콜스택)
-exec bt

// 레지스터 값
-exec info registers
```

### Eigen / STL 컨테이너 보기

STL pretty-printer가 설정되어 있으므로 `std::vector`, `std::array` 등을 가독성 있게 표시합니다.

```
// std::vector 출력
-exec print my_vector

// Eigen 행렬 (pretty-printer 없으면 내부 구조 표시)
-exec print q.data()@7
```

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

### ❌ `rt_controller` 노드가 팝업 목록에 없음 (Attach 시)

노드가 실행 중이지 않거나 이름이 다릅니다.

```bash
# 실행 중인 프로세스 확인
ps aux | grep -E "rt_controller|mujoco_simulator"

# ROS 2 노드 목록
ros2 node list
```

---

### ❌ Breakpoint에서 멈추면 1kHz 루프 타이밍 깨짐

**실시간 루프** 디버깅 특이사항입니다. Breakpoint에서 멈추는 동안 ROS 타이머가 쌓이거나 WatchDog가 트리거될 수 있습니다.

**대안**:
1. **Logpoint** 사용 (멈추지 않고 로그 출력)
2. 코드에 `RCLCPP_DEBUG` 로그 추가 후 `ros2 run rqt_console rqt_console`로 관찰
3. `data_logger`를 활용해 파일로 기록 후 오프라인 분석

---

## 관련 파일

| 파일 | 역할 |
|------|------|
| [`.vscode/launch.json`](../.vscode/launch.json) | 디버그 실행 구성 |
| [`.vscode/tasks.json`](../.vscode/tasks.json) | 빌드/테스트 태스크 |
| [`.vscode/c_cpp_properties.json`](../.vscode/c_cpp_properties.json) | IntelliSense 설정 |
| [`.vscode/settings.json`](../.vscode/settings.json) | 워크스페이스 설정 |
