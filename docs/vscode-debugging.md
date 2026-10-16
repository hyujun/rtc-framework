# VS Code Debugging Guide

VS Code + GDB 로 `rtc-framework` 의 C++ 노드를 디버깅하는 방법이다. 설정 파일 템플릿은 [docs/vscode/](vscode/) 에 있다.

## 시작 전

`.vscode/` 는 `.gitignore` 대상이라 repo 에 체크인되지 않는다. 템플릿을 복사해서 쓴다:

```bash
cd ~/ros2_ws/rtc_ws/src/rtc-framework
cp -r docs/vscode .vscode
```

VS Code 로 **repo root (`src/rtc-framework`) 를 연다.** 그러면 `${workspaceFolder}` 가 repo root 이고 colcon `build/` · `install/` 은 `${workspaceFolder}/../../` (ws root `~/ros2_ws/rtc_ws`) 에 있다. 템플릿은 이 전제로 작성되어 있다.

| 파일 | 내용 |
|------|------|
| `settings.json` | clangd 를 IntelliSense 로, cpptools 는 디버거 전용. ROS distro (`jazzy`), 탐색 제외 |
| `tasks.json` | 빌드·테스트·launch task 11개 (아래 표) |
| `launch.json` | 노드 5종 launch + Attach + GTest (모두 `cppdbg`, GDB) |
| `extensions.json` | 권장 확장 |

## 준비

```bash
sudo apt install gdb clangd libomp-dev
```

`install.sh` 는 `--skip-debug` 를 주지 않는 한 셋 다 설치한다. `libomp-dev` 는 필수다: pinocchio/proxsuite 헤더가 `<omp.h>` 를 include 하는데, 없으면 clangd preamble 이 실패해서 `no member` · `aka int` 같은 연쇄 진단이 쏟아진다.

확장은 `Ctrl+Shift+P` → `Show Recommended Extensions` 로 일괄 설치한다. 핵심 둘은 역할이 다르다.

| 확장 | 역할 |
|------|------|
| `llvm-vs-code-extensions.vscode-clangd` | IntelliSense · 저장 시 포맷 |
| `ms-vscode.cpptools` | GDB 디버거 (IntelliSense 는 비활성) |

## Debug 빌드

디버거가 소스와 바이너리를 매핑하려면 **Debug 모드**로 빌드해야 한다. Release 는 최적화 때문에 변수가 `optimized out` 이 되거나 실행 순서가 바뀐다.

`Ctrl+Shift+B` → **`colcon: Build All (Debug)`** (내부적으로 `./build.sh -d`). 터미널에서는 repo root 에서 같은 명령을 쓴다:

```bash
./build.sh -d
```

`build.sh` 는 빌드가 끝나면 `merge_compile_commands.py` 를 자동 실행해 clangd 용 `compile_commands.json` 을 repo root 에 둔다. 테스트는 기본 빌드에 포함되지 않는다 — 단일 패키지는 `colcon: Build Selected Package` (`--tests` 포함) 로 먼저 빌드한다.

### Task 목록

| Label | 역할 |
|-------|------|
| `colcon: Build All (Debug)` | 전체 Debug 빌드 (기본 빌드, launch 의 `preLaunchTask`) |
| `colcon: Build All (Release)` | 전체 Release 빌드 |
| `colcon: Build Selected Package` | `./build.sh -d --tests -p <pkg>` |
| `colcon: Build Sim` / `Build Robot` | `./build.sh sim -d` / `./build.sh robot -d` |
| `colcon: Test All` / `Test Selected Package` | `colcon test` (ws root 에서, All 은 `--base-paths src/rtc-framework` 로 이 repo 만) |
| `rtc: Merge compile_commands.json` | 병합만 수동 실행 |
| `rtc: Check RT Setup` | `check_rt_setup.sh --summary` |
| `ros2 launch: sim` / `robot` | `sim_ur5e_p1a` / `robot_ur5e_p1a` launch (`robot_ip` 를 묻는다) |

## Launch 구성

`F5` (또는 `Run and Debug`) 로 구성을 고르면 VS Code 가 `preLaunchTask` 로 Debug 빌드를 한 뒤 GDB 로 바이너리를 **직접** 실행한다 (`ros2 launch` 를 거치지 않는다).

> [!IMPORTANT]
> **sim · robot 전체를 디버깅할 때는 `ros2 launch` 로 띄우고 [Attach](#attach--실행-중인-노드에-연결) 한다** (권장). launch 는 노드 간 lifecycle 순서 (mujoco → controller), cset shield 편입, 세션 디렉토리, launch 인자 override 를 맡는데 직접 실행은 그중 아무것도 하지 않는다. 직접 실행은 노드 하나의 `on_configure` · `on_activate` 를 처음부터 밟아 볼 때 쓴다.

모든 노드가 lifecycle 노드라 직접 띄우면 `unconfigured` 에 멈춘다. 다른 터미널에서 전환을 직접 건다 (controller 는 mujoco 가 active 가 된 뒤):

```bash
ros2 lifecycle nodes                              # 노드 이름 확인
ros2 lifecycle set /mujoco_simulator configure && ros2 lifecycle set /mujoco_simulator activate
ros2 lifecycle set /integrated_rt_controller configure && ros2 lifecycle set /integrated_rt_controller activate
```


| 구성 | 바이너리 (`install/<pkg>/lib/<pkg>/`) | `--params-file` (repo root 기준) |
|------|------|------|
| `C++: Launch integrated_rt_controller (Debug)` | `integrated_rt_controller` | `integrated_bringup/config/ur5e_p1a/_base.yaml` → `sim.yaml` (launch 와 같은 순서, `LD_PRELOAD=libmimalloc.so.2` 포함) |
| `C++: Launch mujoco_simulator_node (Debug)` | `mujoco_simulator_node` (`rtc_mujoco_sim`) | `rtc_mujoco_sim/config/solver_param.yaml` → `integrated_bringup/config/ur5e_p1a/mujoco_simulator.yaml` |
| `C++: Launch udp_hand_node (Debug)` | `udp_hand_node` (`udp_hand_driver`) | `udp_hand_driver/config/udp_hand_node.yaml` |
| `C++: Launch bt_coordinator_node (Debug)` | `bt_coordinator_node` (`ur5e_bt_coordinator`) | `ur5e_bt_coordinator/config/bt_coordinator_p1a.yaml` |
| `C++: Launch shape_estimation_node (Debug)` | `shape_estimation_node` (`shape_estimation`) | `shape_estimation/config/shape_estimation_node.yaml` |
| `C++: Attach to Node (Pick Process)` | 실행 중인 프로세스 선택 | — |
| `C++: Run GTest (Selected Package)` | `<ws>/build/` 기준 GTest 경로를 입력받아 실행 | — |

params 는 기본 로봇 `ur5e_p1a` 기준이다. 실로봇 설정이나 다른 로봇으로 디버깅하려면 `launch.json` 의 `args` 를 `config/<robot>/` 의 파일로 바꾼다. 어떤 파일을 어떤 순서로 겹치는지는 그 로봇의 launch 파일 ([integrated_bringup/launch/](../integrated_bringup/launch/)) 이 정한다.

GTest 는 `colcon: Build Selected Package` 로 먼저 빌드하고, 입력창에 `<ws>/build/` 아래 경로를 넣는다 (기본값 `rtc_base/test_seqlock`).

> [!IMPORTANT]
> GDB 가 공유 라이브러리를 찾으려면 **`setup_env.sh` 를 source 한 터미널에서 VS Code 를 실행**해야 한다 (`code .`). 안 그러면 `error while loading shared libraries` 가 난다.

## Attach — 실행 중인 노드에 연결

프로세스를 재시작하지 않고 붙이고 싶을 때 쓴다. 먼저 터미널에서 평소처럼 띄운다. 노드는 **Debug 빌드**여야 한다 (Attach 는 `preLaunchTask` 를 실행하지 않는다).

```bash
ros2 launch integrated_bringup robot_ur5e_p1a.launch.py
```

`Run and Debug` → **`C++: Attach to Node (Pick Process)`** → `F5` → 팝업에서 `integrated_rt_controller` 를 입력해 고른다.

### 변수 표시

모든 C++ 구성 (Attach 포함) 은 `setupCommands` 에 `-enable-pretty-printing` · `set print pretty on` · `set print object on` 을 넣어 두어서 `std::vector` · `std::array` · Eigen 타입이 읽기 좋게 표시된다. pretty-printer 가 안 붙는 raw 버퍼는 DEBUG CONSOLE 에서 `-exec print q.data()@7` (포인터@개수) 로 펼쳐 본다.

`c_cpp_properties.json` 은 일부러 만들지 않았다. clangd 가 `.clangd` 와 repo root 의 `compile_commands.json` 을 직접 읽기 때문이다.

## RT 루프 주의 · Logpoint

> [!WARNING]
> RT 루프 (`control_rate`) 에서 Breakpoint 로 멈추면 ROS 타이머가 쌓이고 WatchDog / E-STOP 이 걸리며, 재개 후 타이밍이 깨진다. 조건부 Breakpoint 나 Hit Count 보다 **Logpoint** 를 우선한다.

Logpoint 는 멈추지 않고 메시지만 출력한다. 줄 번호 여백 우클릭 → `Add Logpoint` → `"joint_pos[0] = {joint_pos[0]}"` 처럼 `{}` 안에 변수를 쓴다. 부족하면 `RCLCPP_DEBUG` + `rqt_console`, 또는 `data_logger` 로 파일에 남겨 오프라인 분석한다.

Breakpoint 가 필요한 코드는 RT 가 아닌 lifecycle · init · 테스트 경로에서 잡는다. DEBUG CONSOLE 에서는 GDB 명령 앞에 `-exec` 를 붙인다 (`-exec bt`, `-exec info threads`).

## 문제 해결

| 증상 | 해결 |
|------|------|
| `Unable to open 'x.cpp'`, `optimized out`, 빨간 원에 경고 표시 | Debug 로 다시 빌드한다 (`Ctrl+Shift+B`). Release 바이너리이거나 소스를 고치고 빌드하지 않은 경우다. 경로가 다르면 DEBUG CONSOLE 에서 `-exec set substitute-path <old> <new>` |
| Attach 시 `Operation not permitted` | ptrace 정책 때문이다. 확인: `cat /proc/sys/kernel/yama/ptrace_scope`. 세션 동안만 허용: `echo 0 \| sudo tee /proc/sys/kernel/yama/ptrace_scope` (재부팅 시 복구; 개발 PC 에서만) |
| Attach 팝업에 노드가 없음 | 노드가 떠 있는지 `ps aux \| grep integrated_rt_controller` 와 `ros2 node list` 로 확인한다 |
| clangd 가 있는 member 에 `no member` · `aka int` | API 문제가 아니라 preamble 실패의 연쇄 진단이다. 로그의 **첫** `file not found` 부터 해결한다 (대부분 `omp.h` → `libomp-dev`) |

clangd 가 새 header · target 을 못 보면 `./build.sh -d` (자동 merge) → `Ctrl+Shift+P` → **`clangd: Restart language server`** 순서로 갱신한다. `.cache/clangd/index` 삭제는 completion 만 stale 할 때의 마지막 수단이다. VS Code 없이 확인하려면 repo root 에서:

```bash
clangd --check=rtc_controller_manager/src/rt_controller_node.cpp
```

`pp_file_not_found` · `conflicting_types` · `no_member` · `aka int` 가 **없으면** 정상이다 (`clangd --check` 는 기능 self-test 실패도 error 로 세므로 error 수가 0 일 필요는 없다).

## 다음에 볼 곳

- 빌드 옵션 전체: `./build.sh --help` · [repo_scripts/README.md](../repo_scripts/README.md)
- 테스트 종류와 실행 순서: [testing.md](testing.md)
- RT 환경 (shield · 권한) 확인: `rtc: Check RT Setup` task

## 부록 — 핵심 키

| 키 | 동작 |
|----|------|
| `F5` | 시작 · Continue |
| `F9` | Breakpoint 토글 |
| `F10` / `F11` / `Shift+F11` | Step Over / Into / Out |
| `Ctrl+Shift+B` | 기본 빌드 (`Build All (Debug)`) |
| `Ctrl+Shift+P` → `Run Task` | task 목록 |
