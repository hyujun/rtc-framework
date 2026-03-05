# ur5e_mujoco_sim 파일 분리 계획

## 현황 분석

| 파일 | 줄 수 | 문제 |
|------|-------|------|
| `include/ur5e_mujoco_sim/mujoco_simulator.hpp` | **1451줄** | 모든 구현이 인라인으로 하나의 파일에 집중 |
| `src/mujoco_simulator_node.cpp` | 347줄 | 적정 수준 — 대규모 변경 불필요 |

`mujoco_simulator.hpp` 1451줄의 기능별 구성:

| 영역 | 줄 범위 | 줄 수 |
|------|---------|-------|
| 클래스 선언 (public API + private 멤버) | 1–371 | ~371 |
| 수명 주기 (ctor/dtor, Initialize, Start, Stop) | 377–488 | ~112 |
| 명령/상태 I/O (SetCommand, Get*, SetExternalForce 등) | 489–543 | ~55 |
| 물리 헬퍼 (ReadState, ApplyCommand, PreparePhysicsStep 등) | 544–725 | ~182 |
| SimLoopFreeRun + SimLoopSyncStep | 726–848 | ~123 |
| ViewerLoop (GLFW + 키보드/마우스 + 오버레이) | 849–1451 | **~603** |

---

## 목표 구조

헤더-전용 → **선언 헤더 + 기능별 `.cpp` 분리** 방식으로 전환합니다.

```
src/ur5e_mujoco_sim/
│
├── include/ur5e_mujoco_sim/
│   └── mujoco_simulator.hpp          ← 클래스 선언만 (inline 구현 전부 제거, ~370줄)
│
└── src/
    ├── mujoco_simulator.cpp          ← 수명 주기 + 명령·상태 I/O    (~170줄)
    ├── mujoco_sim_loop.cpp           ← 물리 헬퍼 + 시뮬 루프 2종    (~300줄)
    ├── mujoco_viewer.cpp             ← ViewerLoop (GLFW 전체)         (~620줄)
    └── mujoco_simulator_node.cpp     ← ROS2 노드 (기존 유지, 347줄)
```

---

## 각 파일별 상세 계획

### 1. `include/ur5e_mujoco_sim/mujoco_simulator.hpp` — 클래스 선언

**변경 사항:**
- 줄 377–1451의 모든 `inline` 구현 제거
- `inline` 키워드 제거 (→ 각 .cpp에서 일반 멤버 함수로 정의)
- `#include <GLFW/glfw3.h>` 제거 (viewer.cpp로 이동)
- 클래스 선언부(public API, private 멤버, 중첩 타입) 및 `kMjJointNames` 상수 배열만 유지
- `#include <mujoco/mujoco.h>` 유지 (mjModel*, mjData*, mjvPerturb 멤버 변수 타입에 필요)

**유지되는 것:**
- `SimMode` enum
- `Config` struct
- `SolverStats` struct
- `StateCallback` 타입 alias
- 모든 public 메서드 선언
- 모든 private 멤버 변수 선언
- 모든 private 메서드 선언
- 짧은 one-liner atomic setter/getter는 선언부에 `inline` 유지 가능
  (예: `SetIntegrator`, `GetIntegrator`, `SetSolverType`, `Pause`, `Resume`, `IsPaused`, `RequestReset`, `SetMaxRtf`, `EnableGravity`, 상태 accessor 등 — 1~3줄짜리 trivial 함수들)

**예상 줄 수: ~370줄**

---

### 2. `src/mujoco_simulator.cpp` — 수명 주기 + I/O

**담당 함수:**

| 함수 | 원본 줄 | 설명 |
|------|---------|------|
| `MuJoCoSimulator()` (ctor) | 377–381 | 초기화 |
| `~MuJoCoSimulator()` (dtor) | 383–387 | Stop() + mj_delete |
| `ResolveJointIndices()` | 389–405 | joint name → qpos/qvel 인덱스 |
| `Initialize()` | 407–461 | MJCF 로드, mj_makeData, 초기 자세 적용 |
| `Start()` | 463–474 | sim_thread_ + viewer_thread_ 생성 |
| `Stop()` | 476–487 | jthread request_stop + join |
| `SetCommand()` | 489–493 | pending_cmd_ 쓰기, cmd_pending_ set |
| `SetStateCallback()` | 495–497 | state_cb_ 등록 |
| `GetPositions()` | 499–501 | state_mutex_ 읽기 |
| `GetVelocities()` | 503–505 | state_mutex_ 읽기 |
| `GetEfforts()` | 507–511 | state_mutex_ 읽기 |
| `SetExternalForce()` | 514–523 | ext_xfrc_ 쓰기 (pert_mutex_) |
| `ClearExternalForce()` | 525–529 | ext_xfrc_ 초기화 |
| `UpdatePerturb()` | 531–535 | shared_pert_ 쓰기 (pert_mutex_) |
| `ClearPerturb()` | 537–542 | pert_active_ = false |

**인클루드:**
```cpp
#include "ur5e_mujoco_sim/mujoco_simulator.hpp"
#include <cstdio>
```

**예상 줄 수: ~170줄**

---

### 3. `src/mujoco_sim_loop.cpp` — 물리 헬퍼 + 시뮬 루프

**담당 함수:**

| 함수 | 원본 줄 | 설명 |
|------|---------|------|
| `ApplyCommand()` | 545–552 | pending_cmd_ → data_->ctrl |
| `ReadState()` | 554–563 | qpos/qvel/qfrc_actuator → latest_* |
| `ReadSolverStats()` | 565–577 | solver 통계 캡처 |
| `GetSolverStats()` | 579–582 | solver_stats_mutex_ 읽기 |
| `InvokeStateCallback()` | 584–594 | state_cb_ 호출 |
| `UpdateVizBuffer()` | 596–604 | viz_mutex_ try_lock → viz_qpos_ 복사 |
| `UpdateRtf()` | 606–617 | RTF 측정 (200스텝마다) |
| `ThrottleIfNeeded()` | 619–641 | max_rtf 초과 시 sleep |
| `PreparePhysicsStep()` | 644–682 | gravity/solver/contact/외력/perturbation 적용 |
| `ClearContactForces()` | 684–691 | xfrc_applied 초기화 |
| `HandleReset()` | 693–724 | initial_qpos로 재초기화 |
| `SimLoopFreeRun()` | 726–777 | 자유 실행 물리 루프 |
| `SimLoopSyncStep()` | 780–846 | 동기 스텝 물리 루프 |

**인클루드:**
```cpp
#include "ur5e_mujoco_sim/mujoco_simulator.hpp"
#include <cstring>
#include <chrono>
#include <thread>
```

**예상 줄 수: ~300줄**

---

### 4. `src/mujoco_viewer.cpp` — ViewerLoop (GLFW)

**담당 함수:**

| 함수 | 원본 줄 | 설명 |
|------|---------|------|
| `ViewerLoop()` | 849–1451 | GLFW 뷰어 전체 (초기화, 렌더링, 이벤트 처리) |

**내용 (ViewerLoop 내부):**

| 기능 | 설명 |
|------|------|
| GLFW 초기화 / 윈도우 생성 | `glfwInit`, `glfwCreateWindow` |
| MuJoCo 렌더링 초기화 | `mjv_makeScene`, `mjr_makeContext` |
| 키보드 핸들러 | Space/R/G/+/-/I/S/[/]/F1/F3/F4/C/F/V/T/Backspace/Escape |
| 마우스 핸들러 | orbit/pan/zoom/Ctrl+drag perturbation |
| 상태 오버레이 렌더링 | 우상단 상태 표시 |
| Help 오버레이 (F1) | 키 안내 + 현재 설정 표시 |
| RTF 프로파일러 그래프 (F3) | 200샘플 롤링 버퍼 막대그래프 |
| Solver 통계 오버레이 (F4) | improvement/gradient/iter/ncon |
| 렌더링 루프 | ~60Hz `mjr_render` |
| 정리 | `mjv_freeScene`, `mjr_freeContext`, `glfwDestroyWindow` |

**인클루드:**
```cpp
#include "ur5e_mujoco_sim/mujoco_simulator.hpp"
#ifdef MUJOCO_HAVE_GLFW
#include <GLFW/glfw3.h>
#endif
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>
```

**예상 줄 수: ~620줄**

---

### 5. `src/mujoco_simulator_node.cpp` — ROS2 노드 (기존 유지)

변경 없음. 347줄로 기능별 구성이 명확하게 되어 있어 추가 분리 불필요.

---

## CMakeLists.txt 변경

```cmake
# 현재
add_executable(mujoco_simulator_node
  src/mujoco_simulator_node.cpp
)

# 변경 후
add_executable(mujoco_simulator_node
  src/mujoco_simulator_node.cpp
  src/mujoco_simulator.cpp
  src/mujoco_sim_loop.cpp
  src/mujoco_viewer.cpp
)
```

`#ifdef MUJOCO_HAVE_GLFW` 조건부 컴파일은 `mujoco_viewer.cpp` 내부에서 처리되므로 CMakeLists.txt의 기존 GLFW 조건 로직은 그대로 유지됩니다.

---

## 분리 후 파일별 줄 수 요약

| 파일 | 예상 줄 수 | 역할 |
|------|-----------|------|
| `include/.../mujoco_simulator.hpp` | ~370줄 | 클래스 선언 (public API) |
| `src/mujoco_simulator.cpp` | ~170줄 | 수명 주기 + 명령/상태 I/O |
| `src/mujoco_sim_loop.cpp` | ~300줄 | 물리 헬퍼 + SimLoopFreeRun/SyncStep |
| `src/mujoco_viewer.cpp` | ~620줄 | ViewerLoop 전체 (GLFW) |
| `src/mujoco_simulator_node.cpp` | 347줄 | ROS2 노드 (변경 없음) |
| **합계** | **~1,807줄** | (5개 파일, 파일당 평균 361줄) |

---

## 구현 순서

1. **`mujoco_simulator.hpp` 수정** — 클래스 선언만 남기고 모든 `inline` 구현 제거, GLFW include 제거
2. **`mujoco_simulator.cpp` 생성** — 수명 주기 + I/O 함수 이식
3. **`mujoco_sim_loop.cpp` 생성** — 물리 헬퍼 + 시뮬 루프 이식
4. **`mujoco_viewer.cpp` 생성** — ViewerLoop 이식 + GLFW include 이동
5. **`CMakeLists.txt` 수정** — 3개 소스 파일 추가
6. **빌드 검증** — `colcon build --packages-select ur5e_mujoco_sim`
7. **커밋 & 푸시**

---

## 설계 원칙

- **네임스페이스 유지**: `ur5e_rt_controller` (변경 없음)
- **헤더 가드 유지**: `UR5E_MUJOCO_SIM_MUJOCO_SIMULATOR_HPP_`
- **noexcept 유지**: 모든 함수 시그니처 동일하게 유지
- **기능 변경 없음**: 순수 파일 분리만 수행
- **짧은 인라인 유지**: atomic getter/setter처럼 1~3줄짜리 trivial 함수는 헤더에 `inline`으로 유지 가능 (ODR 위반 없음)
- **GLFW 조건부 컴파일**: `#ifdef MUJOCO_HAVE_GLFW` 블록은 `mujoco_viewer.cpp` 내부로 완전 이동
