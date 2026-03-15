# CPU 격리 최적화 계획 — 자동 감지 기반 동적 전환

## 목표
- **빌드 시**: 자동으로 격리 감지 → 해제(또는 bypass) → 전체 코어로 컴파일
- **로봇 구동 시**: 자동으로 격리 상태 감지 → 미격리면 enable → RT 코어 격리 후 실행
- 수동 명령 불필요 — 모든 전환이 자동

## 현재 문제
- `isolcpus` GRUB 파라미터는 부팅 시 고정 → 컴파일 시에도 격리 유지
- 6코어 시스템에서 isolcpus=2-5 적용 후 colcon build → Core 0-1만 사용 (3-4배 느림)
- 개발 머신(NUC)에서 빈번한 빌드-테스트 사이클에 큰 불이익

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
│     ├─ cpu_shield.sh on  (cset shield 활성화)           │
│     ├─ setup_irq_affinity.sh (IRQ pinning)              │
│     └─ CPU governor → performance                       │
│  3. rt_controller 실행 (격리된 코어에서)                │
│  4. 종료 시:                                            │
│     └─ 격리 유지 (다음 빌드에서 자동 해제됨)            │
└─────────────────────────────────────────────────────────┘
```

## 구현 단계

### Phase 1: cpu_shield.sh 핵심 스크립트 (신규)

**파일: `ur5e_rt_controller/scripts/cpu_shield.sh`**

```bash
#!/bin/bash
# cpu_shield.sh — 동적 CPU 격리 관리
#
# Usage:
#   sudo cpu_shield.sh on       # RT 코어 격리 (로봇 구동 전)
#   sudo cpu_shield.sh off      # 격리 해제 (빌드 전)
#   cpu_shield.sh status         # 현재 상태 확인 (sudo 불필요)

# on:
#   - cset shield --cpu=${RT_CORES} --kthread=on
#   - setup_irq_affinity.sh 호출 (IRQ를 OS 코어로 제한)
#   - CPU governor → performance
#
# off:
#   - cset shield --reset
#   - IRQ affinity 기본값 복원
#
# status:
#   - /sys/devices/system/cpu/isolated 확인
#   - cset shield 상태 확인
#   - 사용 가능한 코어 수 표시
```

주요 구현 세부:
- `get_physical_cores()` — 기존 함수 재사용 (thread_config.hpp 레이아웃 매칭)
- SMT/HT 시블링 포함 격리 (6C/12T → isolate logical CPU 2-5,8-11)
- cset 미설치 시 fallback: `/sys/fs/cgroup/cpuset` 직접 조작
- 멱등성: 이미 on이면 skip, 이미 off이면 skip

### Phase 2: build.sh / install.sh 자동 감지 + 자동 해제

**공통 함수 추가 (두 스크립트에 동일 적용):**

```bash
# ── CPU shield 자동 관리 (빌드 전) ──────────────────────
# 격리 상태를 감지하고, cset shield이면 자동 해제하여
# 전체 코어를 빌드에 활용한다.
auto_release_cpu_shield() {
  local isolated
  isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null)

  if [[ -z "$isolated" ]]; then
    # 격리 없음 — 전체 코어 사용 가능
    return 0
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
    warn ""
    warn "권장: isolcpus를 GRUB에서 제거하고 cset shield 방식으로 전환하세요"
    warn "  1. sudo nano /etc/default/grub  →  isolcpus=... 제거"
    warn "  2. sudo update-grub && sudo reboot"
    warn "  3. 로봇 구동 시 자동으로 cset shield가 활성화됩니다"
    return 1  # 빌드는 계속 진행 (제한된 코어로)
  fi

  return 0
}
```

**build.sh 수정 위치:**
- `check_workspace_structure()` 직후, `colcon build` 직전에 `auto_release_cpu_shield()` 호출
- 반환값에 따라 `--parallel-workers` 자동 조정:
  - 해제 성공: `nproc --all` 전체 사용
  - isolcpus 고정: `nproc` (비격리 코어만) 사용

**install.sh 수정 위치:**
- `check_prerequisites()` 직후에 동일 함수 호출
- `build_package()` 내부의 병렬도 설정 시 사용 가능 코어 반영

**build_rt_kernel.sh 수정:**
- `make -j${BUILD_THREADS}` 전에 동일 감지
- `BUILD_THREADS`를 사용 가능 코어 수로 조정

### Phase 3: 로봇 구동 시 자동 격리 활성화

**ur_control.launch.py 수정:**

launch 파일에 pre-launch ExecuteProcess 추가:

```python
# ── [RT] CPU Shield 자동 활성화 ──────────────────────────
# RT 프로세스 시작 전 CPU 격리를 자동으로 활성화한다.
# 이미 격리 상태이면 skip (멱등성).
enable_cpu_shield = ExecuteProcess(
    cmd=[
        'bash', '-c',
        'SCRIPT_DIR=$(ros2 pkg prefix ur5e_rt_controller)/share/ur5e_rt_controller/scripts && '
        'if [ -f "$SCRIPT_DIR/cpu_shield.sh" ]; then '
        '  ISOLATED=$(cat /sys/devices/system/cpu/isolated 2>/dev/null); '
        '  if [ -z "$ISOLATED" ]; then '
        '    echo "[RT] CPU shield 미활성 — 자동 활성화 중..."; '
        '    sudo "$SCRIPT_DIR/cpu_shield.sh" on; '
        '  else '
        '    echo "[RT] CPU shield 이미 활성: Core $ISOLATED 격리됨"; '
        '  fi; '
        'else '
        '  echo "[RT] WARNING: cpu_shield.sh not found — CPU 격리 건너뜀"; '
        'fi'
    ],
    output='screen',
    condition=IfCondition(LaunchConfiguration('use_cpu_affinity'))
)
```

**실행 순서:**
1. `enable_cpu_shield` — CPU 격리 활성화 + IRQ affinity + governor
2. `ur_driver_launch_action` — UR 드라이버 시작
3. `pin_ur_driver` (기존) — UR 드라이버를 Core 0-1로 pin (3초 지연)
4. `rt_controller_node` — RT 컨트롤러 실행 (격리된 코어에서)

**rt_controller_main.cpp 수정 (선택적 추가):**

C++ 레벨에서도 격리 상태 검증 (경고만, 실행은 계속):

```cpp
// main() 초반, mlockall 이후
// CPU 격리 상태 확인 — 비격리 시 경고 (실행은 차단하지 않음)
{
  std::ifstream isolated("/sys/devices/system/cpu/isolated");
  std::string isolated_cpus;
  if (isolated.is_open()) {
    std::getline(isolated, isolated_cpus);
  }
  if (isolated_cpus.empty()) {
    fprintf(stderr,
      "[WARN] No CPU isolation detected. RT jitter may increase.\n"
      "       Run: sudo cpu_shield.sh on\n"
      "       Or launch with use_cpu_affinity:=true (default)\n");
  } else {
    fprintf(stdout, "[INFO] CPU isolation active: Core %s\n",
            isolated_cpus.c_str());
  }
}
```

### Phase 4: GRUB 파라미터 최적화

**setup_nvidia_rt.sh [3/11] GRUB 설정 수정:**

기존 GRUB_PARAMS_WITH_VALUE에서 `isolcpus` 제거:

```bash
# 기존:
declare -A GRUB_PARAMS_WITH_VALUE=(
  ["isolcpus"]="${RT_CORES}"        # ← 제거
  ["nohz_full"]="${RT_CORES}"
  ["rcu_nocbs"]="${RT_CORES}"
  ["processor.max_cstate"]="1"
)

# 변경:
declare -A GRUB_PARAMS_WITH_VALUE=(
  ["nohz_full"]="${RT_CORES}"       # 유지: tickless
  ["rcu_nocbs"]="${RT_CORES}"       # 유지: RCU offload
  ["processor.max_cstate"]="1"      # 유지: C-state 비활성화
)
```

이유:
- `isolcpus` → `cset shield`로 대체 (동적 전환)
- `nohz_full` → 유지 (cset과 독립, RT 코어에서 timer tick 제거)
- `rcu_nocbs` → 유지 (RCU 콜백을 OS 코어로 offload)
- 나머지 → 유지

### Phase 5: check_rt_setup.sh 업데이트

**[2/8] CPU Isolation 카테고리 수정:**

```bash
check_cpu_isolation() {
  # 방법 1: isolcpus (GRUB)
  # 방법 2: cset shield (동적)
  # 둘 중 하나라도 활성이면 PASS

  local isolated
  isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null)

  if [[ -n "$isolated" ]]; then
    # isolcpus 또는 cset shield에 의해 격리됨
    if grep -q "isolcpus=" /proc/cmdline 2>/dev/null; then
      _pass "CPU 격리 (isolcpus): ${isolated}"
      _warn "isolcpus는 빌드 성능에 영향. cset shield 전환 권장"
    else
      _pass "CPU 격리 (cset shield): ${isolated}"
    fi
  else
    # 격리 없음 — 빌드 모드이거나 설정 미완료
    _warn "CPU 격리 미활성 — RT 실행 시 자동 활성화됩니다"
    _fix "수동 활성화: sudo cpu_shield.sh on"
  fi
}
```

### Phase 6: install.sh RT 설정 통합

**install.sh의 `install_rt_permissions()` 이후 cset 설치 추가:**

```bash
install_cset_tools() {
  info "Installing cpuset tools for dynamic CPU isolation..."
  if command -v cset &>/dev/null; then
    success "cset already installed"
    return
  fi
  # Ubuntu 22.04: python3-cpuset (cset 명령)
  # Ubuntu 24.04: cpuset 패키지명이 다를 수 있음
  if sudo apt-get install -y python3-cpuset > /dev/null 2>&1; then
    success "cset installed via python3-cpuset"
  elif sudo apt-get install -y cpuset > /dev/null 2>&1; then
    success "cset installed"
  else
    warn "cset 설치 실패 — /sys/fs/cgroup/cpuset fallback 사용"
  fi
}
```

### Phase 7: 문서 업데이트

**docs/RT_OPTIMIZATION.md 추가 섹션:**

```markdown
## 동적 CPU 격리 (cset shield)

기존 `isolcpus` GRUB 파라미터 대신 `cset shield`를 사용하여
재부팅 없이 CPU 격리를 동적으로 전환할 수 있습니다.

### 자동 동작 (권장)
- `./build.sh` / `./install.sh` → 격리 자동 해제 → 전체 코어 빌드
- `ros2 launch ur5e_rt_controller ur_control.launch.py` → 격리 자동 활성화

### 수동 제어
  sudo cpu_shield.sh on      # RT 모드 (로봇 구동)
  sudo cpu_shield.sh off     # 빌드 모드 (전체 코어)
  cpu_shield.sh status       # 상태 확인

### isolcpus에서 마이그레이션
  1. sudo nano /etc/default/grub
     → GRUB_CMDLINE_LINUX_DEFAULT에서 isolcpus=... 제거
     → nohz_full, rcu_nocbs는 유지
  2. sudo update-grub && sudo reboot
  3. 이후 자동 동작으로 전환됨
```

## 전체 워크플로우 시나리오

### 시나리오 1: 개발자 빌드-테스트 사이클

```
$ ./build.sh
  ⚠ CPU 격리 감지: Core 2-5 격리 중 (2/6 코어 사용 가능)
  ▶ cset shield 감지 → 빌드를 위해 자동 해제 중...
  ✔ CPU 격리 해제 완료 — 전체 6 코어로 빌드합니다
  ▶ Building: ur5e_msgs ur5e_rt_base ...
  ... (6코어 병렬 빌드, ~1.2초) ...
  ✔ Build complete

$ ros2 launch ur5e_rt_controller ur_control.launch.py
  [RT] CPU shield 미활성 — 자동 활성화 중...
  [RT] Core 2-5 격리 완료 (cset shield)
  [RT] IRQ affinity 설정 완료 (Core 0-1)
  [RT] CPU governor → performance
  [INFO] Thread 'rt_control' configured: Core 2, SCHED_FIFO/90
  ...
```

### 시나리오 2: isolcpus 마이그레이션 전 (하위 호환)

```
$ ./build.sh
  ⚠ CPU 격리 감지: Core 2-5 격리 중 (2/6 코어 사용 가능)
  ⚠ isolcpus GRUB 파라미터 감지 — 재부팅 없이 해제 불가
  ⚠ 빌드에 2/6 코어만 사용됩니다
  ⚠ 권장: isolcpus를 GRUB에서 제거하고 cset shield 방식으로 전환하세요
  ▶ Building with 2 parallel workers...
```

### 시나리오 3: 격리 없는 상태에서 로봇 실행

```
$ ros2 launch ur5e_rt_controller ur_control.launch.py
  [RT] CPU shield 미활성 — 자동 활성화 중...
  [RT] ✔ Core 2-5 격리 완료
  [RT] ✔ IRQ affinity 설정 완료
  [INFO] CPU isolation active: Core 2-5
  ...
```

## 성능 비교 (6코어 시스템 기준)

| 시나리오 | isolcpus (기존) | cset shield 자동 (제안) |
|----------|----------------|------------------------|
| RT 런타임 jitter | <50μs | <50μs (동등) |
| colcon build 병렬도 | **2코어** (Core 0-1) | **6코어** (자동 해제) |
| make -j 커널빌드 | **2코어** | **6코어** (자동 해제) |
| 전환 방식 | 재부팅 필요 | **자동** (수동 개입 없음) |
| 개발자 경험 | 빌드 느림, 수동 전환 | **투명** — 빌드/실행만 하면 됨 |

## 위험 요소

1. **cset vs isolcpus RT 성능 차이**:
   - `isolcpus`는 커널 레벨 완전 격리 (가장 강력)
   - `cset shield`는 cpuset cgroups 기반 (99%+ 동등)
   - 실제 500Hz 제어루프에서 차이 미미 (타겟 <200μs, 실측 <50μs)
   - `nohz_full` + `rcu_nocbs`는 GRUB에 유지되므로 tick/RCU 간섭은 차단됨

2. **cset 패키지 의존성**:
   - Ubuntu 22.04: `python3-cpuset` 패키지
   - Ubuntu 24.04: `cpuset` 또는 `python3-cpuset`
   - 미설치 시 fallback: `/sys/fs/cgroup/cpuset` 직접 조작

3. **sudo 권한 필요**:
   - cpu_shield.sh on/off에 sudo 필요
   - build.sh에서 자동 해제 시 sudo 프롬프트 발생 가능
   - 해결: passwordless sudo for cset 설정 가능
     `echo "$USER ALL=(ALL) NOPASSWD: /path/to/cpu_shield.sh" | sudo tee /etc/sudoers.d/cpu-shield`

4. **빌드 중 로봇 실행 방지 없음**:
   - 격리 해제 상태에서 로봇 실행하면 launch에서 자동 활성화
   - 빌드와 로봇 실행이 동시에 되면 리소스 경합 (하지만 이는 기존에도 동일)

## 수정 파일 목록

| 파일 | 작업 | 우선순위 |
|------|------|---------|
| `ur5e_rt_controller/scripts/cpu_shield.sh` | **신규** — on/off/status 핵심 스크립트 | P0 |
| `build.sh` | 수정 — `auto_release_cpu_shield()` 추가 | P0 |
| `install.sh` | 수정 — 동일 감지 로직 + `install_cset_tools()` 추가 | P0 |
| `ur5e_rt_controller/launch/ur_control.launch.py` | 수정 — pre-launch `enable_cpu_shield` 추가 | P0 |
| `ur5e_rt_controller/scripts/setup_nvidia_rt.sh` | 수정 — GRUB에서 `isolcpus` 제외 | P1 |
| `ur5e_rt_controller/scripts/check_rt_setup.sh` | 수정 — cset shield 검증 추가 | P1 |
| `ur5e_rt_controller/scripts/build_rt_kernel.sh` | 수정 — 빌드 시 코어 감지 | P1 |
| `ur5e_rt_controller/src/rt_controller_main.cpp` | 수정 — 격리 상태 경고 출력 | P2 |
| `ur5e_rt_controller/CMakeLists.txt` | 수정 — cpu_shield.sh 설치 대상 추가 | P1 |
| `docs/RT_OPTIMIZATION.md` | 수정 — 동적 격리 섹션 추가 | P2 |
