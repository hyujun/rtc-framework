# CPU 격리 최적화 계획 — 컴파일 성능 고려

## 목표
- RT 런타임: Core 격리 유지 (500Hz 제어루프 jitter < 50μs)
- 컴파일 시: 모든 코어 활용 가능 (빌드 성능 극대화)
- 전환: 재부팅 없이 동적 전환

## 현재 문제
- `isolcpus` GRUB 파라미터는 부팅 시 고정 → 컴파일 시에도 격리 유지
- 6코어 시스템에서 isolcpus=2-5 적용 후 colcon build → Core 0-1만 사용 (3-4배 느림)
- 개발 머신(NUC)에서 빈번한 빌드-테스트 사이클에 큰 불이익

## 해결 전략: cset shield 기반 동적 격리

`isolcpus` 대신 `cset shield` (cpuset cgroups) 사용:
- 런타임: `cset shield --cpu 2-5` → RT 코어 격리 (isolcpus와 동등한 효과)
- 컴파일: `cset shield --reset` → 모든 코어 해제 → 전체 코어로 빌드
- 재부팅 불필요

## 구현 단계

### Phase 1: 동적 격리 스크립트 생성 (신규)

**파일: `ur5e_rt_controller/scripts/cpu_shield.sh`**

기능:
- `cpu_shield.sh on` — RT 모드: cset shield로 RT 코어 격리
  - 물리 코어 자동 감지 (4/6/8코어 레이아웃 매칭)
  - SMT/HT 시블링 포함 격리
  - IRQ affinity도 함께 설정 (setup_irq_affinity.sh 호출)
  - CPU governor → performance
- `cpu_shield.sh off` — 빌드 모드: 격리 해제, 모든 코어 사용 가능
  - cset shield reset
  - IRQ affinity 해제 (기본값 복원)
- `cpu_shield.sh status` — 현재 격리 상태 표시
  - 격리된 코어 목록
  - 사용 가능한 코어 수
  - IRQ affinity 상태

구현 세부:
```bash
# on 모드
sudo cset shield --cpu=${RT_CORES} --kthread=on
# cset shield는 "system" set(OS 코어)과 "user" set(RT 코어)으로 분리
# RT 프로세스는 cset shield --exec -- <command>로 실행

# off 모드
sudo cset shield --reset
```

### Phase 2: build.sh / install.sh 빌드 최적화

**build.sh 수정:**
1. 빌드 시작 전 격리 상태 감지:
   - `/sys/devices/system/cpu/isolated` 읽기
   - cset shield 상태 확인
2. 격리 상태이면 사용 가능한 코어 수 계산:
   - `nproc` (--all 없이) → 비격리 코어만 반환
   - `--parallel-workers`를 자동 조정
3. 사용자에게 경고 + 자동 해제 제안:
   ```
   ⚠ CPU 격리 활성 (isolcpus=2-5) — 빌드에 Core 0-1만 사용 가능
   ⚠ 빌드 전 격리 해제 권장: sudo cpu_shield.sh off
   ```

**install.sh 수정:**
- 동일한 감지 로직 추가
- build_rt_kernel.sh의 `make -j` 도 동일하게 조정

**구체적 함수: `detect_available_cores()`**
```bash
detect_available_cores() {
  local isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null)
  if [[ -n "$isolated" ]]; then
    # nproc (without --all) returns only non-isolated cores
    local available=$(nproc)
    local total=$(nproc --all)
    warn "CPU isolation active: ${isolated} isolated, ${available}/${total} cores available for build"
    echo "$available"
  else
    nproc --all
  fi
}
```

### Phase 3: GRUB 파라미터 최적화

**isolcpus 제거, 나머지 유지:**

기존:
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=2-5 nohz_full=2-5 rcu_nocbs=2-5 threadirqs nosoftlockup processor.max_cstate=1"
```

변경:
```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash nohz_full=2-5 rcu_nocbs=2-5 threadirqs nosoftlockup processor.max_cstate=1"
```

이유:
- `isolcpus` → cset shield로 대체 (동적 전환 가능)
- `nohz_full` → 유지 (tickless는 cset과 독립, 항상 도움됨)
- `rcu_nocbs` → 유지 (RCU 콜백 오프로드는 항상 도움됨)
- `threadirqs` → 유지 (IRQ를 스레드화하여 RT 선점 가능)
- `nosoftlockup` → 유지
- `processor.max_cstate=1` → 유지 (C-state 절전 비활성화)

**setup_nvidia_rt.sh 수정:**
- GRUB 파라미터 설정 시 `isolcpus` 제외
- 대신 cset shield 서비스 등록

### Phase 4: RT 실행 통합

**rt_controller 시작 시 자동 격리:**

`ur_control.launch.py` 또는 시작 스크립트에서:
1. RT 프로세스 시작 전 `cpu_shield.sh on` 자동 실행
2. RT 프로세스를 shield된 코어에서 실행:
   ```bash
   cset shield --exec -- ros2 launch ur5e_rt_controller ur_control.launch.py
   ```
3. 종료 시 선택적으로 `cpu_shield.sh off`

또는 systemd 서비스로:
```ini
[Service]
ExecStartPre=/path/to/cpu_shield.sh on
ExecStart=ros2 launch ...
ExecStopPost=/path/to/cpu_shield.sh off
```

### Phase 5: check_rt_setup.sh 업데이트

검증 카테고리 수정:
- [2/8] CPU Isolation: `isolcpus` OR `cset shield` 중 하나만 있으면 PASS
- 새 카테고리: "Dynamic Shield" 상태 표시
- `--fix` 모드에서 cset 기반 명령 제안

### Phase 6: 문서 및 도움말 업데이트

- RT_OPTIMIZATION.md에 동적 격리 섹션 추가
- build.sh --help에 격리 관련 안내 추가
- install.sh의 --skip-rt 옵션 설명 업데이트

## 영향 분석

### 성능 비교 (6코어 시스템 기준)

| 시나리오 | isolcpus (기존) | cset shield (제안) |
|----------|----------------|-------------------|
| RT 런타임 jitter | <50μs | <50μs (동등) |
| colcon build 병렬도 | 2코어 (Core 0-1) | 6코어 (전체) |
| make -j 커널빌드 | 2코어 | 6코어 |
| 전환 방식 | 재부팅 필요 | 명령 1줄 (즉시) |
| nohz_full 효과 | 적용됨 | 적용됨 (GRUB 유지) |
| rcu_nocbs 효과 | 적용됨 | 적용됨 (GRUB 유지) |

### 위험 요소

1. **cset vs isolcpus RT 성능 차이**:
   - `isolcpus`는 커널 레벨에서 완전 격리 (가장 강력)
   - `cset shield`는 cpuset cgroups 기반 (99%+ 동등, 극한 상황에서 미세 차이)
   - 실제 500Hz 제어루프에서는 차이 미미 (타겟 <200μs, 실측 <50μs)

2. **cset 패키지 의존성**:
   - Ubuntu 22.04/24.04: `sudo apt install cpuset` 또는 `cset` 필요
   - 없을 경우 fallback으로 직접 /sys/fs/cgroup/cpuset 조작

3. **잊고 빌드 모드로 RT 실행**:
   - cpu_shield.sh status를 rt_controller 시작 시 자동 체크
   - 격리 안 되어 있으면 경고 출력 (실행은 차단하지 않음)

## 수정 파일 목록

| 파일 | 작업 |
|------|------|
| `ur5e_rt_controller/scripts/cpu_shield.sh` | **신규** — 동적 격리 on/off/status |
| `build.sh` | 수정 — 격리 상태 감지, 사용 가능 코어 자동 조정 |
| `install.sh` | 수정 — 동일한 감지 로직 + RT 설정에서 cset 서비스 등록 |
| `ur5e_rt_controller/scripts/setup_nvidia_rt.sh` | 수정 — GRUB에서 isolcpus 제외, cset 서비스 등록 |
| `ur5e_rt_controller/scripts/check_rt_setup.sh` | 수정 — cset shield 상태 검증 추가 |
| `ur5e_rt_controller/scripts/build_rt_kernel.sh` | 수정 — 빌드 시 사용 가능 코어 감지 |
| `CMakeLists.txt` | 수정 — cpu_shield.sh를 ROS2 설치 대상에 추가 |
| `docs/RT_OPTIMIZATION.md` | 수정 — 동적 격리 섹션 추가 |
