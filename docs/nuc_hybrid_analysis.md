# NUC Hybrid CPU Core Allocation — 분석 자료 (v4 프롬프트 기준)

> **상태 (2026-04-22)**: Stage A 구현 완료 — [`docs/NUC_HYBRID_SUPPORT.md`](NUC_HYBRID_SUPPORT.md) 참조. 이 문서는 **Stage B 진입 전 선결 항목**(§3.1~§4.6)의 설계 결정·실측 근거 체크리스트로 계속 유효.
>
> **목적**: v4 프롬프트가 현재 코드베이스(2026-04-22 unified 10/12/14-core + monotonicity invariant)에 어떤 영향을 주는지, 어떤 설계 결정이 해소/잔존되었는지, 실패 확률이 높은 지점이 어디인지 정리. **Stage B 설계 gating 문서**.
>
> **Stage A에서 이미 baked 된 결정** (아래 §7 체크리스트에서 처리 완료):
> - §4.1 `RTC_FORCE_HYBRID_GENERATION` — 감지 힌트 전용(ID 리스트는 sysfs 유지) · 구현 [`cpu_topology.hpp::ApplyEnvHint`](../rtc_base/include/rtc_base/threading/cpu_topology.hpp)
> - §4.2 `GetCpuTopology()` 캐싱 — function-local static + `on_configure` prewarm 예정(Stage B RT thread 시작 전)
> - §4.4 `DegradationMode` — enum만 Phase 1에 land, Aligator runner 연동은 Stage C
> - §1.3 test coverage — 6 C++ mock + SMT invariant live-host + 27 shell assertions
>
> **관련 문서**:
> - [`docs/NUC_HYBRID_SUPPORT.md`](NUC_HYBRID_SUPPORT.md) — Stage A 산출물 요약 + Phase 2/3 로드맵 + 벤치마크 템플릿
> - [`docs/RT_OPTIMIZATION.md`](RT_OPTIMIZATION.md), [`docs/SHELL_SCRIPTS.md`](SHELL_SCRIPTS.md) — 현행 RT/shield/IRQ 설계
> - [`agent_docs/architecture.md`](../agent_docs/architecture.md) — threading model, E-STOP, SSoT 설명
> - [`rtc_base/test/test_mpc_thread_config.cpp`](../rtc_base/test/test_mpc_thread_config.cpp) — `TierIsolationMonotonicity`
> - memory: `project_core_allocation.md` (2026-04-22 unified layout, SSoT 4파일 목록)

---

## 0. 결론 요약 (TL;DR)

1. v4 프롬프트는 v2/v3 대비 **scope가 현실적으로 좁혀졌고** (NUC 13 Pilot 1종), 이전 분석에서 지적한 주요 설계 문제(`bool allocation_degraded`, 6 함수 중복, Phase 2/3 섞임, BIOS HT off silent regression) 4건이 해소되었다.
2. 단, **새로 도입된 설계 결정** 3건은 실측·벤치마크로만 검증 가능하므로 코드 merge 전에 결론을 낼 수 없다:
   - i7-1360P(4P)의 `mpc_main+mpc_worker_0` SMT sibling 결합이 Aligator parallel backward pass에 실제로 유리한가
   - i7-1370P(6P) Variant A vs B 선택
   - `sensor_io+udp_recv` SMT sibling 결합이 I/O burst 시 경쟁을 유발하는가
3. **즉시 merge 가능한 부분은 Stage A(Topology 감지)뿐**이다. Stage B 이후는 NUC 13 Pro i7-1360P 실기 벤치마크가 선행 조건이다.
4. 잔존 우려 3건은 plan 수립 시점에 결정해야 한다:
   - `DegradationMode::SERIAL_MPC` 경로를 Phase 1에서 **구현**하는지 **명세만** 확정하는지
   - `GetCpuTopology()` 캐싱 시점(`static` initializer vs 명시적 `Initialize()`)
   - 기존 `TierIsolationMonotonicity` 테스트를 hybrid 경로에 어떻게 확장할 것인가 (현재 테스트는 `NOT_NUC_HYBRID` 경로 전용 invariant를 가정)
5. 현재 `TierIsolationMonotonicity` 테스트는 동종 CPU 전제 하에서 "per-thread dedication never regresses as core count grows"를 보장한다. v4의 hybrid 분기가 추가되면 이 테스트의 전제 자체가 바뀌므로 **테스트 리팩토링 혹은 분리**가 Phase 1 필수 작업에 포함되어야 한다 (프롬프트 산출물 목록에 명시되지 않음).

---

## 1. 현재 상태 snapshot (2026-04-22 기준)

| 항목 | 현재 | 출처 |
|---|---|---|
| Tier dispatch 기준 | `GetPhysicalCpuCount()` 물리 코어 수 | [`thread_utils.hpp:506,723`](../rtc_base/include/rtc_base/threading/thread_utils.hpp) |
| 지원 tier | 4/6/8/10/12/14/16 (모두 `inline const ThreadConfig` 정적 상수) | [`thread_config.hpp:48,123,184,257,342,432,515`](../rtc_base/include/rtc_base/threading/thread_config.hpp) |
| 대표 FIFO 우선순위 | rt_control 90, sensor_io 70, udp_recv 65, mpc_main 60, mpc_worker_* 55 | [`architecture.md`](../agent_docs/architecture.md), [`thread_config.hpp`](../rtc_base/include/rtc_base/threading/thread_config.hpp) |
| Worker 개수 | 8코어 0 · 10코어 1 · 12/14/16코어 2 | [`thread_config.hpp:105,235,308,392,477,568`](../rtc_base/include/rtc_base/threading/thread_config.hpp) |
| SSoT 4파일 | `thread_config.hpp`, `thread_utils.hpp::SelectThreadConfigs`, `rt_common.sh`, `cpu_shield.sh`, `verify_rt_runtime.sh` | memory `project_core_allocation.md` |
| Invariant 테스트 | `TierIsolationMonotonicity` — 인접 tier 간 num_workers 단조 증가, distinct-core 단조 비감소, ≥10코어에서 mpc_main/udp_recv/logger 분리 | [`test_mpc_thread_config.cpp`](../rtc_base/test/test_mpc_thread_config.cpp) |
| Hybrid 인식 | 전무 — i7-1360P(4P+8E)는 "12 physical" 로만 보이고 12-core tier 선택 | `thread_utils.hpp::SelectThreadConfigs` |

**문제의 구체적 구현 경로** (v4 배경이 기술한 그대로 재확인):

`GetPhysicalCpuCount()`는 `/sys/.../core_cpus_list` unique count를 세므로 i7-1360P에서 12를 반환 → `SelectThreadConfigs`의 `>= 12` 분기 → `kRtControlConfig12Core` 적용 → `mpc_worker_1=Core 6`, `udp_recv=Core 7(FIFO 65)`, `logger=Core 8`, `aux/publish=Core 9` 순으로 E-core(CPU 8–15)에 SCHED_FIFO 스레드가 떨어진다. Aligator 병렬 backward pass는 slowest-worker barrier이므로 E-core worker 하나가 전체 solve 분포의 p95를 지배한다.

---

## 2. v4가 해소한 이전 분석 우려 (v2/v3 → v4)

| # | 이전 우려 | v4 해소 방식 | 완전 해소? |
|---|---|---|---|
| 1 | `bool allocation_degraded` 플래그에 소비자 명세가 없음 | `DegradationMode` enum + Aligator runner의 `linear_solver_choice = SERIAL` / `setNumThreads(1)` + `RCLCPP_WARN` 명세 | ✅ 명세 확정 |
| 2 | 6개 `Make_NUC{13,14,15}_{4P,6P}_Configs` 함수 중복 | Phase 1을 `Make_NucHybrid_SMT_Configs_{4P,6P}` 2개로 축소, NUC 14는 Phase 2에서 동일 함수에 `has_lp_e_cores` 분기 추가 | ✅ 구조적으로 해소 |
| 3 | BIOS HT off에서 silent fallback → AMD 12-core 레이아웃 적용 | HT off (`IS_HYBRID=1 && P_CORE_HAS_SMT=0`)를 별도 감지 + `check_rt_setup.sh`에서 FAIL | ✅ 명시적 차단 |
| 4 | Phase 2/3이 Phase 1과 섞여 risk 분산 어려움 | 독립 PR 3개로 분리, Phase 2/3 작업은 문서화만 | ✅ 분리 완료 |
| 5 | 컨테이너/Docker 환경 sysfs 접근 실패 대응 없음 | `/proc/cpuinfo flags hybrid` fallback + `RTC_FORCE_HYBRID_GENERATION` 환경변수 + WARN 로그 | ⚠️ 부분 해소 (아래 §4.1 참조) |
| 6 | `GetCpuTopology()` 캐싱 전략 부재 | `thread_utils.cpp` static initializer 방식으로 프로세스당 1회 감지 명시 | ⚠️ 부분 해소 (아래 §4.2 참조) |
| 7 | SMT sibling index 매칭(`p_core_physical_ids[i]` vs `p_core_sibling_ids[i]`가 같은 core_id인가) 보장 없음 | `detect_hybrid_capability` Step 5에서 `core_id` grouping으로 invariant 명시 + `TEST(Topology, SmtSiblingInvariant)` | ✅ 명세 확정 |

---

## 3. v4에서 새로 도입된 설계 결정 (실측 없이는 판단 불가)

### 3.1 i7-1360P(4P) 레이아웃의 `mpc_main + mpc_worker_0` SMT pair 결합

**v4 지시 (§2.2)**:
```
P-core 2: mpc_main    (p[2], FIFO 60)
         mpc_worker_0 (s[2], FIFO 55)   ← 같은 physical core의 SMT sibling
```

**근거 (프롬프트 주석)**: "Aligator cache 공유".

**분석**:
- Aligator parallel backward pass는 각 worker가 stage별 Riccati sweep을 분담하고 barrier에서 동기화한다. Worker 간 공유 메모리는 거의 없고, 각 worker가 독립적인 Riccati state를 읽고 쓴다.
- SMT sibling은 L1d/L1i/L2 cache를 공유하지만 ALU/FP unit은 교대 사용이다. 즉 `mpc_main`과 `mpc_worker_0`이 같은 physical core에서 돌면 Riccati 행렬곱(Eigen DGEMM)은 serialize된다.
- 반면 다른 physical P-core에 배치하면 L2 miss penalty가 크지만 ALU 병렬도를 확보.
- **어느 쪽이 유리한지는 실측이 필요**. 소형 문제(horizon 20, state 18)에서는 cache 공유가 유리할 수 있고, 대형 문제(horizon 50+)에서는 ALU 병렬도가 유리할 수 있다.

**실측 필요 항목**:
- i7-1360P에서 `mpc_main=p[2], mpc_worker_0=s[2]` 배치 vs `mpc_main=p[2], mpc_worker_0=p[3]` 배치의 Aligator solve 분포(median/p95/p99) 비교
- Variant B(6P)는 이 결정을 반복 — `mpc_main=p[3], mpc_worker_0=s[3]` 이므로 동일 이슈

**분석 결론**: v4는 "cache 공유"를 기본 선택으로 가정했으나, **실측 없이 기본값으로 두면 Phase 1 pilot의 주요 성능 지표가 가정에 의존**한다. Phase 1 벤치마크(§Task 5.1) 항목에 "SMT pair vs cross-core worker 배치 비교"를 명시적으로 추가해야 한다.

### 3.2 i7-1370P(6P) Variant A vs B 선택

**v4 지시 (§2.3)**: Variant A(worker 4개, aggressive SMT), Variant B(worker 2개, I/O dedicated) 양쪽 구현하고 벤치마크로 선택. CMake 옵션 `RTC_NUC_6P_VARIANT=A` 제공, 기본값 B.

**분석**:
- Variant A는 Aligator worker 수를 4개로 늘려 horizon 병렬도를 높이지만, 4개 중 2개는 SMT sibling 공유(`s[2], s[3]`)이므로 3.1과 동일한 ALU 경쟁 우려.
- Variant B는 worker 2개이므로 현재 12/14/16-core tier의 `num_workers=2`와 동일 — Aligator 병렬도는 동일하나 `sensor_io`와 `udp_recv`가 독립 P-core에 dedicated되어 I/O jitter가 낮다.
- **`TierIsolationMonotonicity` 관점**: 현재 테스트는 "num_workers가 tier 증가에 따라 단조 증가"를 강제한다. i7-1360P(4P, worker 2) → i7-1370P(6P, Variant B, worker 2)는 **증가가 아니라 동일**이므로 테스트는 통과하나, Variant A(worker 4)는 증가를 보장한다.
- 만약 사용자가 Variant B를 기본으로 채택하면 "6P가 4P보다 worker를 늘리지 않는다"는 의사결정을 문서화해야 함 (이유: I/O 격리가 worker 추가보다 가치 있음).

**실측 필요 항목**:
- i7-1370P 실기에서 Variant A/B의 Aligator solve + cyclictest + UDP burst latency 비교
- i7-1370P 실기가 **현재 없다면** Variant B 기본 채택의 위험은 낮음(기존 `num_workers=2`와 일치).

**분석 결론**: i7-1370P 실기 확보 전까지는 Variant B만 빌드/테스트 대상으로 두고, Variant A는 코드로만 존재(deadcode 경고 suppress) — CMake 옵션은 실기 확보 후에 활성화.

### 3.3 `sensor_io + udp_recv` SMT sibling 결합 (4P, 6P Variant A 공통)

**v4 지시 (§2.2, §2.3 Variant A)**:
```
P-core 1: sensor_io (p[1], FIFO 70)
         udp_recv  (s[1], FIFO 65)
```

**분석**:
- 둘 다 **I/O 콜백**이다. `sensor_io`는 ROS2 subscription executor, `udp_recv`는 UDP 소켓 recv 루프.
- SMT sibling 공유는 I/O interrupt 처리 중 L1d cache hit rate 향상에는 긍정적이지만, UDP burst(10kHz 이상) 시 두 스레드가 context switch 경쟁을 일으키면 **sensor callback p99가 현재(12-core tier, Core 3 dedicated)보다 나빠질 수 있다**.
- v4 §2.3 Variant B는 이 문제를 예방하기 위해 `sensor_io=p[1]` sibling 비움, `udp_recv=p[2]` dedicated로 설계.
- v4 §2.2 4P는 P-core 4개밖에 없어 dedicated 할 수 없음 — **구조적 제약이지 선택이 아니다**.

**실측 필요 항목**:
- i7-1360P에서 UDP burst(hand driver 10kHz)와 JointState subscription(500Hz) 동시 가동 시 sensor callback latency p99
- 현재 12-core tier(sensor Core 3, udp_recv Core 7)와의 비교

**분석 결론**: 4P는 구조적 제약이므로 수용하되, Phase 1 pilot 벤치마크에 "UDP burst 조건에서 sensor callback p99"를 추가. 만약 regression이 발견되면 대안은 `udp_recv`를 E-core로 이동 — 그러나 이는 v4 §금지사항("SCHED_FIFO 스레드는 절대 E-core에 배치하지 않는다")을 위반. 대안 모색이 어려우므로 4P에서는 실측 후 수용 여부 결정.

---

## 4. v4에도 여전히 남은 잔존 우려

### 4.1 컨테이너 fallback + `RTC_FORCE_HYBRID_GENERATION` 환경변수의 신뢰 경계

**v4 지시 (§Task 1.4)**:
```
export RTC_FORCE_HYBRID_GENERATION=raptor_lake_p
```
sysfs 접근 실패 시 이 변수로 강제 지정 가능.

**분석**:
- 환경변수는 **강제값**이므로 실제 하드웨어와 불일치하더라도 topology 감지가 그 값을 따른다. 개발 환경에서는 유용하지만, **Production docker 이미지에 실수로 값이 남으면 다른 하드웨어에서 잘못된 레이아웃이 적용**된다.
- `check_rt_setup.sh`가 이 변수의 사용을 FAIL 또는 WARN 처리하지 않으면 감지 실패가 silent가 된다.
- 또한 환경변수로 `raptor_lake_p`를 강제했는데 실제 CPU가 AMD Ryzen이면, `p_core_physical_ids` 등은 어디서 오는가? 프롬프트는 이를 명시하지 않음.

**결론**: Plan 수립 시 다음 중 하나를 결정:
- (A) `RTC_FORCE_HYBRID_GENERATION`은 **감지 힌트**로만 작동 — 실제 `p_core_physical_ids`는 여전히 sysfs 또는 cpuinfo_max_freq 분포에서 파생
- (B) 환경변수 전체 CpuTopology JSON 주입(`RTC_TOPOLOGY_OVERRIDE=/path/to/topo.json`)
- (C) 환경변수 지원을 Phase 1에서 제거 — Production 환경에서는 sysfs bind-mount 강제

v4 프롬프트는 (A)를 의도한 것으로 보이나 명시적이지 않다.

### 4.2 `GetCpuTopology()` static initializer 타이밍과 thread-safety

**v4 지시 (§Task 1.3)**:
```cpp
const CpuTopology& GetCpuTopology() noexcept {
    static const CpuTopology topo = []() { ... }();
    return topo;
}
```

**분석**:
- C++11 function-local static은 thread-safe 초기화가 보장되나, **RT thread에서 최초 호출이 일어나면 sysfs 읽기/파싱이 RT 경로에서 실행**된다. Sysfs 접근은 POSIX read로, 커널 seq_file 락을 경유하므로 RT-safe하지 않다.
- 따라서 `SelectThreadConfigs()` 혹은 그 전 시점(e.g., `on_configure` lifecycle callback)에서 반드시 최초 호출되어야 한다.
- 현재 `SelectThreadConfigs()`는 `RtControllerNode::on_activate`에서 호출된다 ([`architecture.md`](../agent_docs/architecture.md)). 이 시점은 RT 스레드 시작 이전이므로 안전.
- 단, `GetCpuTopology`가 다른 경로(예: logging thread, aux callback)에서도 호출될 수 있으면 최초 호출 경합 가능성. `[[nodiscard]] const CpuTopology&` 반환 + `noexcept` 서명만으로는 이를 강제할 수 없다.

**결론**: Plan에 다음 명시 필요:
- `GetCpuTopology()`는 `on_configure`에서 **반드시 prewarm** (명시적 `GetCpuTopology();` 호출)
- 혹은 `RtControllerMain`의 early phase에서 prewarm
- RT 경로에서 호출이 생기지 않도록 static analysis 규칙 또는 코드 리뷰 체크리스트 추가

### 4.3 `TierIsolationMonotonicity` 테스트의 hybrid 경로 확장 전략

**현재 테스트** ([`test_mpc_thread_config.cpp`](../rtc_base/test/test_mpc_thread_config.cpp)):
- 동종 CPU 4/6/8/10/12/14/16-core 배열을 정의하고 인접 쌍마다:
  1. `num_workers` 단조 증가
  2. distinct-core count 단조 비감소
  3. ≥10코어에서 mpc_main / udp_recv / logger 분리

**v4가 도입하는 변화**:
- `SelectThreadConfigs()`가 `NucGeneration`에 따라 완전히 다른 레이아웃 분기
- i7-1360P(12 physical, 4P+8E)의 FIFO 스레드는 CPU 0, 2, 4, 6, 1, 3, 5, 7 등 불연속 번호에 배치됨
- AMD 동종 12코어의 FIFO 스레드는 CPU 2–9 연속

**분석**:
- 현재 테스트는 "물리 코어 수 증가 → 자원 독점성 증가"를 가정한다. NUC 13 i7-1360P(12 physical)는 FIFO 스레드 6개가 P-core 4개의 SMT 8 logical에 겹쳐 배치 — `distinct-core count`는 AMD 12코어(각 스레드 dedicated)보다 **낮다**.
- 즉 "12 physical → num_workers=2" 불변은 유지되나 "distinct-core 단조 비감소"는 **전제 자체가 NucGeneration에 따라 달라진다**.

**대응 옵션**:
- **Option 1**: 테스트를 NucGeneration별로 분리 — `TierIsolationMonotonicity_Homogeneous`, `TierIsolationMonotonicity_RaptorLakeP`.
- **Option 2**: Monotonicity invariant를 "**동일 generation 내에서만**" 유지로 재정의. AMD 12 vs NUC 13 12 비교는 의미가 없음.
- **Option 3**: 새로운 hybrid-specific invariant 추가 (예: "모든 SCHED_FIFO가 P-core logical CPU에 속함", "rt_control sibling 비어 있음"). v4 §Task 2.5 "신규 검증"이 이를 언급하나 기존 테스트와의 관계는 불명확.

**결론**: Phase 1 산출물 목록(§산출물 7)에 `test_mpc_thread_config.cpp` 확장이 있으나 **기존 테스트를 어떻게 나눌 것인지**는 없다. Plan에서 Option 1 또는 2를 결정해야 한다. 권장은 Option 2 + Option 3 결합.

### 4.4 `DegradationMode::SERIAL_MPC` Phase 1 구현 범위

**v4 지시 (§Phase 1 산출물 4)**: `rtc_mpc/aligator_runner.cpp`에 `DegradationMode` 처리 로직 구현. Phase 1에서는 NONE만 사용되지만 SERIAL_MPC 명세 구현.

**분석**:
- Phase 1에서 SERIAL_MPC 경로는 **어디서도 트리거되지 않는다** (i7-1360P 최소 4P이므로 worker ≥ 1 확보).
- 그러나 코드가 존재하면 테스트가 필요하고, 테스트 없는 dead code는 향후 회귀 위험.
- 반대로 명세만 두고 구현을 Phase 3로 미루면 NUC 15 Ultra 5 도입 시 Aligator runner 대대적 수정이 필요.

**결론**: Plan에서 다음 중 결정:
- (A) Phase 1에 SERIAL_MPC 분기 구현 + unit test(`MockAligator`로 `setNumThreads(1)` 호출 여부만 확인)
- (B) Phase 1에서는 `DegradationMode` enum만 정의, Aligator runner 수정은 Phase 3에 묶기
- (C) Phase 1에 SERIAL_MPC 분기 구현 + **인위적으로** `RTC_FORCE_DEGRADATION=serial_mpc` 환경변수로 테스트 환경에서 경로 가동

(A)가 명세 완전성 측면에서 바람직하나 Phase 1 scope를 늘리므로 trade-off 존재.

### 4.5 `mujoco_sim.launch.py` SIM_CORE 하드코딩은 Phase 2로 미뤄도 되는가

**v4 지시 (§Phase 2 예정 작업 8)**: "mujoco_sim.launch.py의 SIM_CORE 동적 결정"을 Phase 2로 연기.

**분석 (memory project_core_allocation.md + ur5e_bringup/launch/mujoco_sim.launch.py:346)**:
- 현재 SIM_CORE는 14-core tier에서 Core 10을 하드코딩으로 pin.
- NUC 13 i7-1360P에서는 Core 10 = E-core. MuJoCo sim은 RT 스레드가 아니므로 E-core 배치가 성능에는 문제없음.
- 그러나 v4 §2.2 4P layout은 `aux=e[1]`로 E-core에 일반 스레드 배치 + `publish=e[0]`. Core 10/11은 아직 비어 있으므로 SIM_CORE=Core 10은 **우연히 충돌하지 않는다**.
- 하지만 이는 우연에 의존하는 정합성이며, Phase 1 머지 후 SIM과 함께 돌리는 pilot 테스트가 실패할 경우 원인 분석이 복잡해질 수 있다.

**결론**: Phase 1 pilot 벤치마크에 "MuJoCo sim 동시 가동 시 sim-to-rt sync latency" 항목을 추가하거나, SIM_CORE를 Phase 1에서 `get_sim_core()` helper로 빼는 것이 안전. 단, v4 산출물 목록에는 없음 — plan에서 결정.

### 4.6 `cpu_shield.sh`의 Core 0-1 재분류 Phase 2 연기 리스크

**v4 지시 (§Task 3.4)**: Phase 1에서 cpu_shield.sh 변경 없음.

**분석 (memory project_core_allocation.md)**:
- 현재 cpu_shield.sh는 `compute_shield_cores`가 tier별로 "system" cpuset 범위를 계산. NUC 13(12 physical) → 기존 12-core tier 경로 → shield 2–9.
- v4 Phase 1에서 `SelectThreadConfigs`가 NUC 13에서 새 레이아웃 반환 → FIFO 스레드는 CPU 0–7(P-core logical) 전체에 분산. Shield 2–9에 **CPU 0, 1이 포함되지 않음** (현재 shield는 cset이 "OS용"으로 지정한 CPU 0–1을 건드리지 않는 전제).
- 단, `rt_control=p[0]=CPU 0` (sysfs 표준 배치)일 경우 **shield 범위 밖**에 RT 스레드가 배치된다. 이는 v4 프롬프트의 SMT invariant(p[0] ∈ P-core logical)에는 부합하나 cpu_shield.sh 설계 가정과 충돌.
- v4 §Task 3.4는 "NUC 13이 LP E-core 없으므로 Core 0-1이 OS용으로 자연스럽게 유지됨"이라 명시하나, 이는 **`rt_control`이 CPU 0이 아닌 logical CPU에 배치된다**는 가정이다. 그러나 §2.2 pseudocode는 `p[0]`, `s[0]`을 사용하는데, sysfs는 보통 `/sys/.../intel_core/cpulist=0-7` (i7-1360P HT on 기준)이므로 `p[0]=0, s[0]=1`이 될 수 있다.

**결론**: i7-1360P의 실제 sysfs 출력을 확인해야 한다:
- `/sys/devices/system/cpu/types/intel_core/cpulist` 값
- `/sys/devices/system/cpu/cpu0/topology/core_id` 값
- 만약 P-core logical CPU에 0이 포함되면 **Phase 1에서 cpu_shield.sh 동작이 깨진다**. 최소한 `rt_control`을 `p[0]`에서 `p[1]`로 이동하거나 cpu_shield.sh를 Phase 1에 포함해야 한다.
- 이는 §4.3에서 언급한 "invariant 전제 변화"와 연결된 문제.

---

## 5. SSoT 4파일과의 정합성 검토

| SSoT 파일 | Phase 1 변경 필요 | v4 산출물 목록 | 일치? |
|---|---|---|---|
| `thread_config.hpp` | `CpuTopology/NucGeneration/DegradationMode` 추가, `Make_NucHybrid_SMT_Configs_*` 추가 | ✅ | ✅ |
| `thread_utils.hpp/cpp::SelectThreadConfigs` | NucGeneration 분기, `ValidateSystemThreadConfigs` 신규 | ✅ | ✅ |
| `rt_common.sh` | `detect_hybrid_capability`, `get_nuc_generation`, `get_mpc_cores/rt_cores` 분기 | ✅ | ✅ |
| `cpu_shield.sh::compute_shield_cores` | **변경 없음 (Phase 2로 연기)** | ❌ 명시적 제외 | ⚠️ §4.6에서 지적한 CPU 0-1 충돌 가능성 |
| `verify_rt_runtime.sh::build_expected_threads` | 카테고리 3.5 "Hybrid P-core Placement" 추가 | ✅ | ✅ |

**결론**: v4는 SSoT 5파일 중 4개를 Phase 1에 포함하나 `cpu_shield.sh`를 제외. §4.6에서 지적한 rt_control 배치와 shield 범위 충돌을 i7-1360P 실기 sysfs 출력으로 먼저 검증해야 한다.

---

## 6. Stage별 (A~E) 위험도 및 선행 조건

| Stage | 변경 범위 | 동작 변화 | Merge 전 필수 조건 | 위험도 |
|---|---|---|---|---|
| **A: Topology 감지** | `detect_hybrid_capability`, `GetCpuTopology()`, 관련 shell helper. `SelectThreadConfigs` 미변경 | **없음** (감지값을 소비하는 코드가 없음) | Unit test (NUC 13 mock, HT off, 컨테이너, AMD regression) | **낮음** — 독립 머지 가능 |
| **B: 레이아웃 분기** | `SelectThreadConfigs` NucGeneration 분기, `Make_NucHybrid_SMT_Configs_*`, `ValidateSystemThreadConfigs` | NUC 13 i7-1360P에서 레이아웃 완전 변경 | ①A 완료 ②AMD regression 테스트 ③i7-1360P 실기 cyclictest 1시간 ④§3.1, §3.3 SMT pair 결정 ⑤§4.3 TierIsolationMonotonicity 리팩토링 | **높음** — 최소 3가지 실측 선행 |
| **C: DegradationMode + Aligator 연동** | `rtc_mpc/aligator_runner.cpp` | NONE만 활성이므로 동작 변화 없음 (이론상) | Unit test로 SERIAL_MPC 경로 강제 검증 | **중간** — §4.4 결정 필요 |
| **D: 쉘 스크립트** | `rt_common.sh get_mpc_cores/rt_cores`, `check_rt_setup.sh`, `verify_rt_runtime.sh` | NUC 13에서 IRQ affinity, verify 결과 변화 | B 완료 + i7-1360P 실기 verify PASS | **중간** — B와 묶이는 게 자연스러움 |
| **E: 벤치마크 + merge** | 문서화, CHANGELOG | 없음 | 모든 벤치 통과 | **낮음** |

**권장 merge 순서**:
1. Stage A 단독 PR (topology 감지만, 동작 변화 없음)
2. i7-1360P 실기에서 §3.1, §3.3, §4.6 실측
3. `TierIsolationMonotonicity` 리팩토링 (§4.3) — A에 포함할지 B에 포함할지 결정
4. Stage B + D 통합 PR + Stage C 포함 (A-gated)
5. Stage E 문서화 PR

---

## 7. 실행 전 결정해야 할 사항 (plan 수립 체크리스트)

1. **i7-1370P 실기 확보 여부 확인** — 없으면 Variant B 기본값만 구현(Variant A 코드는 제외), 있으면 벤치마크 일정 잡기
2. **§3.1 SMT pair 기본값 결정**: `mpc_main+mpc_worker_0` SMT 결합을 기본으로 할지, cross-core를 기본으로 할지 — i7-1360P 벤치마크 후
3. **§3.3 4P에서 `sensor_io+udp_recv` SMT 결합** 실측 UDP burst 조건 하에서 sensor p99 regression 여부 확인
4. **§4.1 `RTC_FORCE_HYBRID_GENERATION`** 의미 확정 (감지 힌트인가, 전체 override인가)
5. **§4.2 `GetCpuTopology()` prewarm** 지점 명시 (`on_configure` 권장)
6. **§4.3 `TierIsolationMonotonicity` 리팩토링 방식** — Option 2 + Option 3 조합 권장
7. **§4.4 `DegradationMode::SERIAL_MPC`** Phase 1 구현 범위 결정
8. **§4.5 `mujoco_sim.launch.py`** SIM_CORE 동적화를 Phase 1에 넣을지 Phase 2로 미룰지
9. **§4.6 `cpu_shield.sh`** i7-1360P 실제 sysfs core_id 확인 후 Phase 1 포함 여부 결정
10. **현장 배포 NUC 모델 확정** — i7-1360P와 i7-1370P 중 실제 배포 대상이 무엇인가. Phase 1 벤치마크는 배포 대상에 집중.
11. **벤치마크 환경 표준화** — cyclictest 1시간은 `setup_env.sh` + `cpu_shield.sh` + `verify_rt_runtime.sh` PASS 상태에서 수행해야 수치가 의미가 있음. 벤치 절차 문서화 필요.

---

## 8. 권고 (Plan 수립 시 유의점)

1. **Stage A는 즉시 merge 가능한 저위험 작업으로 분리**. 토폴로지 감지만으로도 앞으로의 모든 작업에 진단 정보가 축적된다 (`check_rt_setup.sh`에서 NUC 세대 표시 등).
2. **Stage B 이전에 `TierIsolationMonotonicity` 재정의**. 현재 테스트는 hybrid 레이아웃이 추가되면 논리적으로 모순된다 — 이를 해결하지 않으면 Stage B에서 "테스트가 실패하므로 레이아웃을 바꿔야 한다" vs "테스트가 틀렸으므로 테스트를 바꿔야 한다"의 판단 부담이 merger에게 넘어간다.
3. **Phase 1 pilot 벤치마크 매트릭스 사전 고정**. §3.1, §3.3, §4.5, §4.6의 결정이 모두 실측 기반이므로, 무엇을 어떻게 잴지 plan 단계에서 체크리스트화. 실측 후 판단이 번복되지 않도록.
4. **i7-1370P 실기가 없다면 Variant A 코드 제외**. 벤치마크 없이 dead code 축적은 유지비용만 높아진다. 실기 확보 시점에 별도 PR.
5. **Phase 1에서 cpu_shield.sh 포함 여부를 §4.6 sysfs 조사 후 결정**. 실측 없이 plan에서 "Phase 2로 연기"를 확정하지 말 것.
6. **`DegradationMode` Phase 1 구현(Option 4.4-A)을 권장**. Phase 1 종료 시점에 enum과 runner가 모두 존재해야 Phase 3 작업이 "새 기능 추가"가 아닌 "분기 활성화"가 된다.

---

## 9. 분석 요약표

| 영역 | v2 → v4 개선 | v4 잔존 우려 | Phase 1 merge gating 여부 |
|---|---|---|---|
| Target 범위 | 6개 NUC 모델 → NUC 13 Pilot 1종 | — | 해소 |
| Degradation 명세 | bool flag → enum + Aligator runner 명세 | §4.4 Phase 1 구현 범위 | 권고: 구현 포함 |
| 함수 중복 | 6함수 → 2함수 (SMT-aware 공유) | — | 해소 |
| BIOS HT off | silent fallback | 명시적 FAIL | 해소 |
| SMT sibling invariant | 부재 | 명세 + 테스트 | 해소 |
| Caching | 미명시 | static initializer | 권고: prewarm 위치 확정 |
| 컨테이너 fallback | 부재 | env var + WARN | 권고: env var 의미론 확정 |
| SMT pair 성능 가정 | 없음 (E-core에 FIFO) | §3.1, §3.3 실측 미결 | **Stage B 차단** |
| i7-1370P Variant | — | §3.2 실기 없음 | 권고: A 제외 |
| `TierIsolationMonotonicity` | — | §4.3 전제 충돌 | **Stage B 차단** |
| cpu_shield.sh | — | §4.6 CPU 0-1 충돌 가능 | Stage B 차단 가능성 |
| mujoco_sim SIM_CORE | — | §4.5 우연한 비충돌 | 권고: 확인 후 결정 |

---

**작성자 의견**: v4는 "pilot 1종 narrowing" + "명세 명확화"로 이전 버전 대비 실행 가능성이 크게 높아졌으나, 남은 결정 7건은 모두 실측 또는 아키텍처 결정이 필요하다. Stage A만 선제 merge한 뒤, Stage B는 §3.1, §3.3, §4.3, §4.6의 4건을 해소한 후 착수하는 것이 잔업 위험을 최소화한다.
