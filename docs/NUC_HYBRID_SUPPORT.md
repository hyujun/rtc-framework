# NUC Hybrid CPU Support

> **Status**: Stage A landed. Stage B / Phase 2 / Phase 3 pending.
>
> **관련 문서**:
> - [`docs/nuc_hybrid_analysis.md`](nuc_hybrid_analysis.md) — v4 프롬프트 설계 분석 (Stage B 진입 전 gating 항목 §3.1~§4.6 참고)
> - [`rtc_base/include/rtc_base/threading/cpu_topology.hpp`](../rtc_base/include/rtc_base/threading/cpu_topology.hpp) — C++ 감지 entry point
> - [`rtc_scripts/scripts/lib/rt_common.sh`](../rtc_scripts/scripts/lib/rt_common.sh) — shell 감지 entry point

---

## 지원 현황

| 세대 | 대표 모델 | 지원 Phase | Stage A 동작 |
|---|---|---|---|
| Raptor Lake-P | NUC 13 Pro (i5-1340P · i7-1360P · i7-1370P) | **Phase 1 (진행중)** | 감지 OK, 레이아웃은 동종 tier 사용 |
| Meteor Lake | NUC 14 Pro (Core Ultra 7 155H / 165H) | Phase 2 (예정) | 감지 OK, 레이아웃은 동종 tier 사용, WARN 로그 |
| Arrow Lake-H | NUC 15 Pro+ (Core Ultra 7 265H / 9 285H) | Phase 3 (예정) | 감지 OK, 레이아웃은 동종 tier 사용, WARN 로그 |
| NUC 13 Pro (BIOS HT off) | — | FAIL — HT 활성화 요구 | `check_rt_setup.sh` FAIL |
| AMD / homogeneous Intel | — | 기존 로직 | regression 없음 |

**Stage A 핵심**: `SelectThreadConfigs()`는 아직 `GetPhysicalCpuCount()` 기반 tier dispatch를 사용한다. `CpuTopology`, `NucGeneration`, `DegradationMode` 타입과 감지 로직만 landed — **소비자는 없다**. Stage B에서 `Make_NucHybrid_SMT_Configs_*`가 도입되며 소비가 시작된다.

---

## BIOS 체크리스트 (NUC 13 Pro)

Stage A는 BIOS 상태를 감지하고 사용자에게 경고만 한다. Phase 1 Stage B에서 실제 레이아웃이 의존하게 되므로 아래 설정을 사전에 확인해두는 것이 안전하다.

- [ ] **Hyper-Threading**: Enabled — Stage B의 SMT sibling pairing 필수. `check_rt_setup.sh`가 disabled를 FAIL로 잡음.
- [ ] **Intel Turbo Boost**: Enabled — P-core 부스트로 MPC solve 지연 감소.
- [ ] **C-states**: C1/C2 이하로 제한하거나 GRUB에 `intel_idle.max_cstate=1 processor.max_cstate=1` 추가.
- [ ] **Speed Shift (HWP)**: Enabled — `intel_pstate=active` 활용.
- [ ] **Intel SpeedStep (EIST)**: 선택사항. `intel_pstate=active`일 때는 무관.
- [ ] **VT-d / IOMMU**: 가능한 Disabled (UDP/DDS 경로의 DMA 지연 최소화, 실측 후 결정).
- [ ] **E-core / LP E-core**: Enabled 유지 — OS/IRQ가 활용.

---

## 환경변수

### `RTC_FORCE_HYBRID_GENERATION`

컨테이너 환경이나 sysfs가 제한된 개발 워크스테이션에서 NUC 세대 감지를 강제 지정한다.

| 값 | 의미 |
|---|---|
| `raptor_lake_p` | NUC 13 Pro class 강제 |
| `meteor_lake` | NUC 14 Pro class 강제 |
| `arrow_lake_h` | NUC 15 Pro+ class 강제 |
| `raptor_lake_p_ht_off` | BIOS HT off 시나리오 재현 (테스트용) |
| `none` | NOT_NUC_HYBRID 강제 |

**중요**: 이 변수는 **세대 enum만 override**하며 `p_core_physical_ids` 등 ID 리스트는 여전히 sysfs에서 파생된다. AMD 하드웨어에 `raptor_lake_p`를 설정하면 enum은 바뀌지만 ID 리스트는 비어 있으므로 Stage B의 `ValidateSystemThreadConfigs`가 FAIL 처리하게 된다. **Production 이미지에는 반드시 unset**한다 — `check_rt_setup.sh`가 값이 남아있으면 WARN 경고한다.

### `RTC_SYSFS_ROOT` / `RTC_PROC_CPUINFO`

테스트 전용. 기본값 `/sys`, `/proc/cpuinfo`. 단위 테스트(`test_rt_common.sh`, `test_cpu_topology.cpp`)가 mock 디렉토리를 구성할 때만 사용.

---

## Stage A 검증 방법

```bash
# 1. rtc_base 단위 테스트 (C++ 감지 로직)
cd rtc_ws
colcon test --packages-select rtc_base --event-handlers console_direct+
# 예상: test_cpu_topology 7 passed (1 skipped on non-hybrid host)

# 2. rtc_scripts 단위 테스트 (shell 감지 로직)
colcon test --packages-select rtc_scripts --event-handlers console_direct+
# 예상: test_rt_common PASS

# 3. 실기 감지 결과 확인
src/rtc-framework/rtc_scripts/scripts/check_rt_setup.sh
# 섹션 [2.5/9] Hybrid CPU Detection 확인:
#   AMD / 비-hybrid Intel: "Homogeneous CPU" PASS
#   NUC 13 Pro i7-1360P: "NUC 13 Pro class: Raptor Lake-P (4P + 8E)" PASS
#   NUC 13 Pro BIOS HT off: FAIL — Hyper-Threading 비활성
```

---

## 벤치마크 결과 템플릿 (Stage B merge gating용)

Stage B merge 전 다음 벤치마크를 채워 Stage B PR에 첨부한다. 각 세션 10분, MuJoCo sim 또는 실기 grasp demo 동일 워크로드.

### cyclictest (RT 지터)

| 환경 | 구성 | p50 (µs) | p99 (µs) | max (µs) |
|---|---|---|---|---|
| AMD 개발 PC | 8-core tier | | | |
| NUC 13 i7-1360P (Stage A) | 12-core tier (E-core에 FIFO) | | | |
| NUC 13 i7-1360P (Stage B) | Raptor Lake-P 4P 레이아웃 | | | |

명령: `sudo cyclictest --mlockall --smp --priority=90 --interval=2000 --duration=3600`

### MPC solve timing (PR #2 관측 경로 사용)

`logging_data/YYMMDD_HHMM/controllers/demo_wbc_controller/mpc_solve_timing.csv`에서 수집 (writer는 `DemoWbcController` 자체 LifecycleNode의 1 Hz aux 타이머). `DemoWbcController::ComputeControl`이 phase-independent로 MPC state를 publish하므로 **kIdle** 구간도 20 Hz solve 측정이 가능하다 — kHold 워크로드와 별도 행으로 분리해 baseline vs active 차이를 구분한다. `count == 0` 행은 `DemoWbcController::GetMpcSolveStats` sentinel(`mpc.enabled=true`인데 solver가 publish 못한 상태 — dim-mismatch / solver error / 워밍업 구간) — 수집 스크립트에서 필터링 필요.

#### 수집 절차

```bash
ros2 launch ur5e_bringup sim.launch.py \
    initial_controller:=demo_wbc_controller enable_mpc:=true mpc_engine:=handler

# 워크로드별 5분씩:
# 1) kIdle — robot_target / grasp_cmd 미전송
# 2) kHold — grasp 시퀀스 수동 트리거 (gains[0]=1 + robot_target 전송)

# 세션 종료 후:
awk -F, 'NR>1 && $2>0 {print $6, $7, $8}' \
    logging_data/<SID>/controllers/demo_wbc_controller/mpc_solve_timing.csv \
    | sort -n | awk 'END{print "rows:", NR}'
# 컬럼 순서: p50_ns, p99_ns, max_ns (NS → MS는 1e-6 스케일).
```

| 환경 | 구성 | Workload | p50 (ms) | p99 (ms) | max (ms) |
|---|---|---|---|---|---|
| AMD 개발 PC | — | kIdle | | | |
| AMD 개발 PC | — | kHold | | | |
| NUC 13 i7-1360P (Stage A) | E-core worker 존재 | kIdle | | | |
| NUC 13 i7-1360P (Stage A) | E-core worker 존재 | kHold | | | |
| NUC 13 i7-1360P (Stage B) | P-core dedicated worker | kIdle | | | |
| NUC 13 i7-1360P (Stage B) | P-core dedicated worker | kHold | | | |

### UDP burst 중 sensor callback latency

측정 방법은 Phase 1 Stage B 별도 PR에서 도입 예정. Stage A 벤치마크 템플릿에는 자리만 둠.

---

## 변경 이력

| 날짜 | 변경 | PR |
|---|---|---|
| 2026-04-22 | Stage A: `CpuTopology` 감지 layer 추가. `check_rt_setup.sh` [2.5/9] 섹션 추가. 동작 변화 없음. | (이 PR) |

---

## 다음 단계

- **Stage B (예정)**: NUC 13 Pro 레이아웃 분기. `SelectThreadConfigs()` NucGeneration 기반 dispatch. 실측 벤치마크 필수.
- **Phase 2 (예정)**: NUC 14 Pro LP E-core 활용. `get_os_cores`가 LP-E 반환. cpu_shield.sh Core 0-1 재분류.
- **Phase 3 (예정)**: NUC 15 Pro+ SMT-off 레이아웃. `DegradationMode::SERIAL_MPC` 실사용 (Ultra 5 4P 등).
