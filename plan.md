# CPU 격리 최적화 계획 — 자동 감지 기반 동적 전환

## 목표
- **빌드 시**: 자동으로 격리 감지 → 해제(또는 bypass) → 전체 코어로 컴파일
- **로봇 구동 시**: 자동으로 격리 상태 감지 → 미격리면 enable → RT 코어 격리 후 실행
- **시뮬레이션 구동 시**: 경량 격리 (RT-critical 코어만) + MuJoCo에 전용 코어 할당
- **8코어+ 시스템**: 티어별 격리로 RT 보호와 빌드/시뮬 성능 양립
- 수동 명령 불필요 — 모든 전환이 자동

## 현재 문제
- `isolcpus` GRUB 파라미터는 부팅 시 고정 → 컴파일 시에도 격리 유지
- 6코어 시스템에서 isolcpus=2-5 적용 후 colcon build → Core 0-1만 사용 (3-4배 느림)
- 개발 머신(NUC)에서 빈번한 빌드-테스트 사이클에 큰 불이익
- MuJoCo 시뮬레이터가 CPU affinity 없이 실행 → OS 코어에 몰려 성능 저하
- 8코어+ 시스템에서 Core 7이 단순 spare → 활용 미흡

---

## 코어 할당 전략: 모드별 3-tier 격리

### Tier 분류

```
┌──────────────────────────────────────────────────────────────────┐
│ Tier 1 (RT-critical)  — 항상 격리 (로봇/시뮬 모두)             │
│   Core 2: rt_control    SCHED_FIFO/90  500Hz 제어루프           │
│   Core 3: sensor_io     SCHED_FIFO/70  JointState 콜백          │
├──────────────────────────────────────────────────────────────────┤
│ Tier 2 (RT-support)   — 로봇 모드에서만 격리                    │
│   Core 4: udp_recv      SCHED_FIFO/65  Hand UDP 폴링            │
│   Core 5: logging       SCHED_OTHER    CSV drain                 │
│   Core 6: aux/monitor   SCHED_OTHER    E-STOP, status           │
├──────────────────────────────────────────────────────────────────┤
│ Tier 3 (Flexible)     — 격리하지 않음                           │
│   Core 7+: sim/monitoring/build                                  │
│   빌드 모드: 전체 코어 사용 가능                                │
│   시뮬 모드: MuJoCo sim_thread / viewer_thread 배치             │
└──────────────────────────────────────────────────────────────────┘
```

### 모드별 격리 전략

```
┌────────────┬────────────────────────────────────────────────────┐
│            │ Core 0-1   │ Core 2-3  │ Core 4-6  │ Core 7+     │
│            │ (OS/DDS)   │ (Tier 1)  │ (Tier 2)  │ (Tier 3)    │
├────────────┼────────────┼───────────┼───────────┼─────────────┤
│ 빌드 모드  │ 전체 코어 사용 가능 (격리 없음)                  │
│ ./build.sh │ make -j$(nproc --all)                             │
├────────────┼────────────┼───────────┼───────────┼─────────────┤
│ 시뮬 모드  │ OS/DDS     │ 격리      │ 비격리    │ MuJoCo      │
│ mujoco_sim │ viewer     │ rt_ctrl   │ logging   │ sim_thread  │
│            │ ROS2       │ sensor_io │ aux       │ monitoring  │
├────────────┼────────────┼───────────┼───────────┼─────────────┤
│ 로봇 모드  │ OS/DDS     │ 격리      │ 격리      │ monitoring  │
│ ur_control │ UR driver  │ rt_ctrl   │ udp/log   │ cyclictest  │
│            │ NIC IRQ    │ sensor_io │ aux       │ spare       │
└────────────┴────────────┴───────────┴───────────┴─────────────┘
```

### 시뮬레이션 vs 로봇 차이점

| 항목 | 로봇 모드 | 시뮬레이션 모드 |
|------|-----------|----------------|
| 격리 범위 | Tier 1 + Tier 2 (Core 2-6) | Tier 1만 (Core 2-3) |
| Hand UDP | 실제 UDP 통신 (Core 4, FIFO/65) | fake_hand_response (격리 불필요) |
| IRQ affinity | 필수 (NIC IRQ → Core 0-1) | 선택 (시뮬은 NIC 부하 낮음) |
| CPU governor | performance 필수 | 기본값 OK (성능 영향 미미) |
| MuJoCo sim_thread | 해당없음 | Core 7+ 할당 (물리 연산 집중) |
| MuJoCo viewer | 해당없음 | Core 0-1 (GPU/디스플레이 접근) |
| jitter 요구 | <50μs (실제 로봇 안전) | 관대함 (~1ms OK, 시뮬이므로) |

---

## 8코어+ 시스템 코어 할당 상세

### 현재 8코어 레이아웃 (thread_config.hpp)

```
Core 0-1: OS / DDS / NIC IRQ
Core 2:   rt_control   (SCHED_FIFO 90)
Core 3:   sensor_io    (SCHED_FIFO 70)
Core 4:   udp_recv     (SCHED_FIFO 65)
Core 5:   logging      (SCHED_OTHER nice -5)
Core 6:   aux/monitor  (SCHED_OTHER nice 0/-2)
Core 7:   spare        ← 활용 미흡
```

### 제안: 8코어 최적화 레이아웃

**로봇 모드 (8코어):**
```
Core 0-1: OS / DDS / UR driver / NIC IRQ          (비격리)
Core 2:   rt_control   SCHED_FIFO/90              (Tier 1 격리)
Core 3:   sensor_io    SCHED_FIFO/70              (Tier 1 격리)
Core 4:   udp_recv     SCHED_FIFO/65              (Tier 2 격리)
Core 5:   logging      SCHED_OTHER nice -5        (Tier 2 격리)
Core 6:   aux/monitor  SCHED_OTHER nice 0/-2      (Tier 2 격리)
Core 7:   monitoring   cyclictest/htop            (Tier 3 비격리)
```
- 격리: Core 2-6 (Tier 1 + Tier 2)
- `cpu_shield.sh on --robot` → `cset shield --cpu=2-6`

**시뮬 모드 (8코어):**
```
Core 0-1: OS / DDS / MuJoCo viewer (GLFW)         (비격리)
Core 2:   rt_control   SCHED_FIFO/90              (Tier 1 격리)
Core 3:   sensor_io    SCHED_FIFO/70              (Tier 1 격리)
Core 4:   logging      SCHED_OTHER                (비격리, Tier 2 완화)
Core 5:   aux/monitor  SCHED_OTHER                (비격리)
Core 6:   MuJoCo sim_thread (물리 연산)           (비격리, affinity 설정)
Core 7:   monitoring / spare                       (비격리)
```
- 격리: Core 2-3만 (Tier 1)
- `cpu_shield.sh on --sim` → `cset shield --cpu=2-3`
- MuJoCo sim_thread → Core 6 affinity (launch에서 taskset)

### 제안: 10-12코어 레이아웃

```
Core 0-1:  OS / DDS / NIC IRQ
Core 2:    rt_control   SCHED_FIFO/90   (Tier 1)
Core 3:    sensor_io    SCHED_FIFO/70   (Tier 1)
Core 4:    udp_recv     SCHED_FIFO/65   (Tier 2)
Core 5:    logging      SCHED_OTHER     (Tier 2)
Core 6:    aux/monitor  SCHED_OTHER     (Tier 2)
Core 7:    MuJoCo sim_thread            (Tier 3)
Core 8:    MuJoCo viewer / monitoring   (Tier 3)
Core 9+:   spare / build               (Tier 3)
```
- 로봇 모드 격리: Core 2-6
- 시뮬 모드 격리: Core 2-3
- 빌드 모드: 전체 해제
- 여분 코어(9+)는 항상 빌드/모니터링에 사용 가능

### 제안: 16코어+ 레이아웃 (서버급)

```
Core 0-3:  OS / DDS / NIC IRQ / UR driver (OS 풀 확장)
Core 4:    rt_control   SCHED_FIFO/90   (Tier 1)
Core 5:    sensor_io    SCHED_FIFO/70   (Tier 1)
Core 6:    udp_recv     SCHED_FIFO/65   (Tier 2)
Core 7:    logging      SCHED_OTHER     (Tier 2)
Core 8:    aux/monitor  SCHED_OTHER     (Tier 2)
Core 9:    MuJoCo sim_thread            (Tier 3)
Core 10:   MuJoCo viewer               (Tier 3)
Core 11+:  spare / build / monitoring   (Tier 3)
```
- OS 코어를 4개로 확대 (DDS, UR driver, NIC, 시스템 프로세스 분산)
- RT 코어 시작점을 Core 4로 이동
- 빌드 시 Core 11+가 항상 가용 → isolcpus 없어도 빌드 영향 최소화

### thread_config.hpp 확장 (코드 변경)

```cpp
// 기존: 4/6/8코어 고정 레이아웃
// 제안: 코어 수에 따른 동적 오프셋 계산

struct CoreLayout {
  int os_cores_end;    // OS 코어 마지막 인덱스
  int rt_core_start;   // RT 코어 시작 인덱스
  int sim_core;        // MuJoCo sim_thread 전용 (-1이면 미할당)
  int viewer_core;     // MuJoCo viewer 전용 (-1이면 OS 코어 공유)
};

constexpr CoreLayout SelectCoreLayout(int physical_cores) {
  if (physical_cores >= 16) {
    return {3, 4, 9, 10};   // OS 0-3, RT 4-8, Sim 9, Viewer 10
  } else if (physical_cores >= 10) {
    return {1, 2, 7, 8};    // OS 0-1, RT 2-6, Sim 7, Viewer 8
  } else if (physical_cores >= 8) {
    return {1, 2, 6, -1};   // OS 0-1, RT 2-6, Sim shared with Tier 3
  } else if (physical_cores >= 6) {
    return {1, 2, -1, -1};  // OS 0-1, RT 2-5, no dedicated sim core
  } else {
    return {0, 1, -1, -1};  // OS 0, RT 1-3, minimal
  }
}
```

---

## 핵심 전략: 자동 감지 + 자동 전환

```
┌─────────────────────────────────────────────────────────┐
│  ./build.sh  or  ./install.sh                           │
│                                                         │
│  1. detect_cpu_shield_status()                          │
│     └─ 격리 감지 (isolcpus / cset shield 확인)         │
│  2. 격리 활성이면:                                      │
│     ├─ cset shield → 자동 해제 (cpu_shield.sh off)     │
│     └─ isolcpus → bypass (nproc 조정 불가, 경고만)     │
│  3. 전체 코어로 colcon build / make -j                  │
│  4. 빌드 완료 후:                                       │
│     └─ 격리 자동 복원하지 않음 (로봇 시작 시 자동 ON)   │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│  ros2 launch ur5e_rt_controller ur_control.launch.py    │
│                                                         │
│  1. pre-launch: detect_cpu_shield_status()              │
│     └─ 격리 상태 확인                                   │
│  2. 격리 미활성이면:                                    │
│     ├─ cpu_shield.sh on --robot (Tier 1+2 격리)        │
│     ├─ setup_irq_affinity.sh (IRQ pinning)              │
│     └─ CPU governor → performance                       │
│  3. rt_controller 실행 (격리된 코어에서)                │
│  4. 종료 시:                                            │
│     └─ 격리 유지 (다음 빌드에서 자동 해제됨)            │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│  ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py       │
│                                                         │
│  1. pre-launch: detect_cpu_shield_status()              │
│     └─ 격리 상태 확인                                   │
│  2. 격리 미활성이면:                                    │
│     ├─ cpu_shield.sh on --sim (Tier 1만 격리)          │
│     └─ IRQ affinity / governor는 선택적                 │
│  3. MuJoCo sim_thread → Tier 3 코어 affinity 설정      │
│  4. rt_controller 실행 (Tier 1 코어에서)                │
│  5. 종료 시:                                            │
│     └─ 격리 유지                                        │
└─────────────────────────────────────────────────────────┘
```

---

## 구현 단계

### Phase 1: cpu_shield.sh 핵심 스크립트 (신규)

**파일: `ur5e_rt_controller/scripts/cpu_shield.sh`**

```bash
#!/bin/bash
# cpu_shield.sh — 동적 CPU 격리 관리 (모드별 tiered isolation)
#
# Usage:
#   sudo cpu_shield.sh on [--robot|--sim]  # 격리 활성화
#   sudo cpu_shield.sh off                 # 격리 해제
#   cpu_shield.sh status                   # 상태 확인

# on --robot (기본):
#   - Tier 1 + Tier 2 격리 (Core 2-6 on 8-core)
#   - IRQ affinity 설정
#   - CPU governor → performance
#
# on --sim:
#   - Tier 1만 격리 (Core 2-3)
#   - Tier 2는 해제 (MuJoCo/logging이 자유롭게 사용)
#   - IRQ affinity 선택적
#
# off:
#   - cset shield --reset
#   - IRQ affinity 기본값 복원
#
# status:
#   - 현재 격리 모드 (robot/sim/none) 표시
#   - 격리된 코어 / 사용 가능 코어 수
```

격리 범위 계산:
```bash
compute_shield_cores() {
  local mode="$1"       # robot or sim
  local phys_cores="$2" # 물리 코어 수

  if [[ "$mode" == "sim" ]]; then
    # Tier 1만: rt_control + sensor_io
    if [[ "$phys_cores" -le 4 ]]; then
      echo "1-2"      # 4코어: Core 1-2
    else
      echo "2-3"      # 6코어+: Core 2-3
    fi
  else
    # Tier 1 + Tier 2: 전체 RT 코어
    if [[ "$phys_cores" -le 4 ]]; then
      echo "1-3"      # 4코어: Core 1-3
    elif [[ "$phys_cores" -le 7 ]]; then
      echo "2-5"      # 6코어: Core 2-5
    elif [[ "$phys_cores" -le 9 ]]; then
      echo "2-6"      # 8코어: Core 2-6
    elif [[ "$phys_cores" -le 15 ]]; then
      echo "2-6"      # 10-15코어: Core 2-6 (7+는 Tier 3)
    else
      echo "4-8"      # 16코어+: Core 4-8 (OS 확대)
    fi
  fi
}
```

### Phase 2: build.sh / install.sh 자동 감지 + 자동 해제

**공통 함수 추가 (두 스크립트에 동일 적용):**

```bash
# ── CPU shield 자동 관리 (빌드 전) ──────────────────────
auto_release_cpu_shield() {
  local isolated
  isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null)

  if [[ -z "$isolated" ]]; then
    return 0  # 격리 없음 → 전체 코어 사용 가능
  fi

  local available=$(nproc)
  local total=$(nproc --all)
  warn "CPU 격리 감지: Core ${isolated} 격리 중 (${available}/${total} 코어 사용 가능)"

  # Case 1: cset shield 활성 → 자동 해제
  local SHIELD_SCRIPT="${INSTALL_SCRIPT_DIR}/ur5e_rt_controller/scripts/cpu_shield.sh"
  if command -v cset &>/dev/null && cset shield -s 2>/dev/null | grep -q "user"; then
    info "cset shield 감지 → 빌드를 위해 자동 해제 중..."
    if [[ -f "$SHIELD_SCRIPT" ]]; then
      sudo bash "$SHIELD_SCRIPT" off
    else
      sudo cset shield --reset
    fi
    success "CPU 격리 해제 완료 — 전체 ${total} 코어로 빌드합니다"
    return 0
  fi

  # Case 2: isolcpus (GRUB 고정) → 해제 불가, 경고만
  if grep -q "isolcpus=" /proc/cmdline 2>/dev/null; then
    warn "isolcpus GRUB 파라미터 감지 — 재부팅 없이 해제 불가"
    warn "빌드에 ${available}/${total} 코어만 사용됩니다"
    warn "권장: isolcpus를 GRUB에서 제거하고 cset shield 방식으로 전환하세요"
    return 1
  fi

  return 0
}
```

### Phase 3: 시뮬레이션 런치 — 경량 격리 + MuJoCo affinity

**mujoco_sim.launch.py 수정:**

```python
# ── [RT] 시뮬레이션 모드 CPU Shield (Tier 1만) ───────────
enable_sim_cpu_shield = ExecuteProcess(
    cmd=[
        'bash', '-c',
        'SCRIPT_DIR=$(ros2 pkg prefix ur5e_rt_controller)/share/ur5e_rt_controller/scripts && '
        'if [ -f "$SCRIPT_DIR/cpu_shield.sh" ]; then '
        '  sudo "$SCRIPT_DIR/cpu_shield.sh" on --sim; '
        'fi'
    ],
    output='screen',
)

# ── MuJoCo sim_thread를 Tier 3 코어에 pin ─────────────
# 물리 연산이 집중되는 sim_thread를 전용 코어에 배치
# 8코어+에서만 적용 (6코어 이하는 OS 코어에서 실행)
pin_mujoco_sim = TimerAction(
    period=2.0,
    actions=[
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'PHYS=$(lscpu -p=Core,Socket 2>/dev/null | grep -v "^#" | sort -u | wc -l); '
                'if [ "$PHYS" -ge 8 ]; then '
                '  PID=$(pgrep -nf mujoco_simulator_node); '
                '  if [ -n "$PID" ]; then '
                '    SIM_CORE=$((PHYS >= 10 ? 7 : 6)); '
                '    taskset -cp $SIM_CORE "$PID" && '
                '    echo "[SIM] mujoco_simulator (PID=$PID) pinned to Core $SIM_CORE"; '
                '  fi; '
                'fi'
            ],
            output='screen',
        )
    ]
)
```

**실행 순서:**
1. `enable_sim_cpu_shield` — Tier 1 격리 (Core 2-3만)
2. `mujoco_node` — MuJoCo 시뮬레이터 시작
3. `rt_controller_node` — RT 컨트롤러 시작
4. `pin_mujoco_sim` — MuJoCo를 Tier 3 코어에 pin (2초 지연)

### Phase 4: 로봇 런치 — 풀 격리

**ur_control.launch.py 수정:**

```python
# ── [RT] 로봇 모드 CPU Shield (Tier 1 + Tier 2) ─────────
enable_cpu_shield = ExecuteProcess(
    cmd=[
        'bash', '-c',
        'SCRIPT_DIR=$(ros2 pkg prefix ur5e_rt_controller)/share/ur5e_rt_controller/scripts && '
        'if [ -f "$SCRIPT_DIR/cpu_shield.sh" ]; then '
        '  ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null); '
        '  if [ -z "$ISOLATED" ]; then '
        '    echo "[RT] CPU shield 미활성 — 로봇 모드 자동 활성화 중..."; '
        '    sudo "$SCRIPT_DIR/cpu_shield.sh" on --robot; '
        '  else '
        '    echo "[RT] CPU shield 이미 활성: Core $ISOLATED 격리됨"; '
        '  fi; '
        'fi'
    ],
    output='screen',
    condition=IfCondition(LaunchConfiguration('use_cpu_affinity'))
)
```

### Phase 5: GRUB 파라미터 최적화

**setup_nvidia_rt.sh [3/11] GRUB 설정 수정:**

```bash
# isolcpus 제거 (cset shield로 대체), 나머지 유지
declare -A GRUB_PARAMS_WITH_VALUE=(
  ["nohz_full"]="${RT_CORES}"       # 유지: tickless
  ["rcu_nocbs"]="${RT_CORES}"       # 유지: RCU offload
  ["processor.max_cstate"]="1"      # 유지: C-state 비활성화
)
```

### Phase 6: check_rt_setup.sh 업데이트

**[2/8] CPU Isolation 수정 — 모드별 검증:**

```bash
check_cpu_isolation() {
  local isolated
  isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null)

  if [[ -n "$isolated" ]]; then
    if grep -q "isolcpus=" /proc/cmdline 2>/dev/null; then
      _pass "CPU 격리 (isolcpus): ${isolated}"
      _warn "isolcpus는 빌드 성능에 영향. cset shield 전환 권장"
    else
      _pass "CPU 격리 (cset shield): ${isolated}"
    fi
  else
    _warn "CPU 격리 미활성 — 로봇/시뮬 실행 시 자동 활성화됩니다"
    _fix "수동 활성화: sudo cpu_shield.sh on [--robot|--sim]"
  fi
}
```

### Phase 7: install.sh 통합

**cset 도구 설치 + cpu_shield.sh 테스트:**

```bash
install_cset_tools() {
  info "Installing cpuset tools for dynamic CPU isolation..."
  if command -v cset &>/dev/null; then
    success "cset already installed"
    return
  fi
  if sudo apt-get install -y python3-cpuset > /dev/null 2>&1; then
    success "cset installed via python3-cpuset"
  elif sudo apt-get install -y cpuset > /dev/null 2>&1; then
    success "cset installed"
  else
    warn "cset 설치 실패 — /sys/fs/cgroup/cpuset fallback 사용"
  fi
}
```

### Phase 8: thread_config.hpp 확장 (선택적)

MuJoCo 시뮬레이터 전용 코어 할당 정보 추가:

```cpp
/// Simulation core assignment (Tier 3, no RT scheduling)
/// Used by mujoco_sim.launch.py for taskset pinning
struct SimCoreLayout {
  int sim_thread_core;    // MuJoCo physics thread (-1 = no pinning)
  int viewer_core;        // GLFW viewer thread (-1 = OS cores)
};

constexpr SimCoreLayout GetSimCoreLayout(int physical_cores) {
  if (physical_cores >= 10) return {7, 8};
  if (physical_cores >= 8)  return {6, -1};  // viewer on OS cores
  return {-1, -1};  // 6코어 이하: pinning 없음
}
```

### Phase 9: 문서 업데이트

**docs/RT_OPTIMIZATION.md 추가:**
- 모드별 코어 할당 다이어그램 (빌드/시뮬/로봇)
- 8코어+ 시스템 최적화 가이드
- cset shield 마이그레이션 가이드

---

## 전체 워크플로우 시나리오

### 시나리오 1: 개발자 빌드-테스트 사이클

```
$ ./build.sh
  ⚠ CPU 격리 감지: Core 2-5 격리 중 (2/6 코어 사용 가능)
  ▶ cset shield 감지 → 빌드를 위해 자동 해제 중...
  ✔ CPU 격리 해제 완료 — 전체 6 코어로 빌드합니다
  ▶ Building: ur5e_msgs ur5e_rt_base ...
  ✔ Build complete [1.16s]

$ ros2 launch ur5e_rt_controller ur_control.launch.py
  [RT] CPU shield 미활성 — 로봇 모드 자동 활성화 중...
  [RT] Core 2-5 격리 완료 (Tier 1+2, cset shield)
  [RT] IRQ affinity 설정 완료 (Core 0-1)
  [RT] CPU governor → performance
  [INFO] Thread 'rt_control' configured: Core 2, SCHED_FIFO/90
```

### 시나리오 2: 시뮬레이션 실행 (8코어 NUC)

```
$ ros2 launch ur5e_mujoco_sim mujoco_sim.launch.py
  [SIM] Tier 1 경량 격리: Core 2-3 (cset shield --sim)
  [SIM] MuJoCo sim_thread → Core 6 (8코어 시스템)
  [INFO] Thread 'rt_control' configured: Core 2, SCHED_FIFO/90
  [INFO] Thread 'sensor_io' configured: Core 3, SCHED_FIFO/70
  [SIM] mujoco_simulator (PID=12345) pinned to Core 6
```

### 시나리오 3: isolcpus 마이그레이션 전 (하위 호환)

```
$ ./build.sh
  ⚠ isolcpus GRUB 파라미터 감지 — 재부팅 없이 해제 불가
  ⚠ 빌드에 2/6 코어만 사용됩니다
  ⚠ 권장: isolcpus를 GRUB에서 제거하고 cset shield 방식으로 전환
  ▶ Building with 2 parallel workers...
```

---

## 성능 비교

### 6코어 시스템

| 시나리오 | isolcpus (기존) | cset shield 자동 (제안) |
|----------|----------------|------------------------|
| RT 런타임 jitter | <50μs | <50μs (동등) |
| colcon build | **2코어** | **6코어** (자동 해제) |
| 시뮬 MuJoCo RTF | OS코어 경합 | 전용코어 없음 (6코어 한계) |

### 8코어 시스템

| 시나리오 | isolcpus (기존) | cset shield 자동 (제안) |
|----------|----------------|------------------------|
| RT 런타임 jitter | <50μs | <50μs (동등) |
| colcon build | **2코어** | **8코어** (자동 해제) |
| 시뮬 MuJoCo RTF | Core 7 미활용 | **Core 6 전용** (물리 연산) |
| 시뮬 격리 범위 | Core 2-6 전체 | **Core 2-3만** (나머지 해제) |

---

## 위험 요소

1. **cset vs isolcpus RT 성능 차이**:
   - `isolcpus`는 커널 레벨 완전 격리 (가장 강력)
   - `cset shield`는 cpuset cgroups 기반 (99%+ 동등)
   - `nohz_full` + `rcu_nocbs`는 GRUB 유지 → tick/RCU 간섭 차단됨

2. **cset 패키지 의존성**:
   - Ubuntu 22.04: `python3-cpuset`
   - Ubuntu 24.04: `cpuset` 또는 `python3-cpuset`
   - fallback: `/sys/fs/cgroup/cpuset` 직접 조작

3. **sudo 권한 필요**:
   - cpu_shield.sh on/off에 sudo 필요
   - 해결: `/etc/sudoers.d/cpu-shield`로 passwordless 설정

4. **시뮬 모드 Tier 1만 격리 시 sensor_io 간섭 가능성**:
   - MuJoCo fake sensor 응답은 DDS 토픽 경유 → sensor_io에 간섭 가능
   - 실제로는 시뮬이므로 jitter 수백μs도 무해
   - 필요 시 `--sim-strict` 옵션으로 Tier 1+2 격리 가능

## 수정 파일 목록

| 파일 | 작업 | 우선순위 |
|------|------|---------|
| `ur5e_rt_controller/scripts/cpu_shield.sh` | **신규** — on [--robot/--sim] / off / status | P0 |
| `build.sh` | 수정 — `auto_release_cpu_shield()` 추가 | P0 |
| `install.sh` | 수정 — 감지 로직 + `install_cset_tools()` | P0 |
| `ur5e_rt_controller/launch/ur_control.launch.py` | 수정 — `cpu_shield.sh on --robot` pre-launch | P0 |
| `ur5e_mujoco_sim/launch/mujoco_sim.launch.py` | 수정 — `cpu_shield.sh on --sim` + MuJoCo affinity | P0 |
| `ur5e_rt_controller/scripts/setup_nvidia_rt.sh` | 수정 — GRUB `isolcpus` 제외 | P1 |
| `ur5e_rt_controller/scripts/check_rt_setup.sh` | 수정 — cset shield 모드별 검증 | P1 |
| `ur5e_rt_controller/scripts/build_rt_kernel.sh` | 수정 — 빌드 시 코어 감지 | P1 |
| `ur5e_rt_base/include/.../thread_config.hpp` | 수정 — SimCoreLayout 추가 | P2 |
| `ur5e_rt_controller/src/rt_controller_main.cpp` | 수정 — 격리 상태 경고 | P2 |
| `ur5e_rt_controller/CMakeLists.txt` | 수정 — cpu_shield.sh 설치 | P1 |
| `docs/RT_OPTIMIZATION.md` | 수정 — 모드별 격리 + 8코어+ 가이드 | P2 |
