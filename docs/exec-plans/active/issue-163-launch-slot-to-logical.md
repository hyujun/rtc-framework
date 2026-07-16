# issue-163: 런치 taskset 의 slot→logical 번역 누락 수정

Issue: #163 — `fix(rtc_tools): 런치 taskset 이 slot 을 logical CPU 로 오용 — e93d0d4 sweep 에서 Python mirror 누락`

## Goal

`ThreadConfig::cpu_core` 는 **slot 인덱스**이지 커널 logical CPU id 가 아니다. commit e93d0d4 가 이 correctness 수정을 bash 소비자 3곳(cpu_shield / setup_grub_rt / verify_rt_runtime)에 적용했으나 **Python 런치 mirror 가 누락**됐다. 런치가 raw slot 을 `taskset` 에 직행시켜, 제어 PC(NUC 13 Pro, 4P+8E)에서 DDS·드라이버 스레드가 의도와 다른 물리코어 — 하필 RT 클러스터 — 에 핀되고 있다. 이를 bash SSoT 위임으로 수정하고 회귀 방지 축을 세운다.

## Acceptance criteria

`[SPRINT]` (사용자 컨펌 완료):

1. NUC13 mock(4P+8E)에서 런치가 내보내는 taskset CPU 인자가 arm=10 / hand=11 / DDS=4 (현재 6 / 7 / 2) — 즉 `SlotToLogicalCpu(slot)` 과 일치
2. 신규 drift 테스트가 NUC13 · Ryzen SMT · SMT-off 3개 fixture 에서 "런치가 쓰는 logical == bash `slot_to_logical_cpu(slot)`" 를 고정하고, 어느 한쪽이 어긋나면 실패
3. `colcon test`: `rtc_tools` / `integrated_bringup` / `repo_scripts` 통과
4. `_pin_external_driver` 중복 정의 0건 (런치 5개가 단일 헬퍼 소비)

## Out of scope

- **`taskset -acp`** (Phase 4) — NUC13 실측 gate. 아래 "Constraints / pending human decisions" 참조
- **DDS pin race** — t=5s 타이머 이후 생성 DDS 스레드 미핀. 근본 해결은 RT 노드가 자기 DDS 스레드를 직접 핀 (thread model 변경 = CLAUDE.md §6 E-7). 별도 이슈
- **cset shield 실기 검증 / cpuset 상호작용** → #151
- **GRUB nohz slot→logical 통일** → #152 (NUC13 primaries-first 라 미발현, 검증 완료)
- 티어 테이블(`thread_config.hpp`) 값 자체는 변경하지 않음 — 레이아웃 변경은 E-7

## 이슈 진단 검증 결과 (착수 전 grep — 이슈 본문은 가설로 취급)

- **누락 실재 확인**: `git show --stat e93d0d4` → 7 files 전부 `repo_scripts/`. `thread_layout.py` 는 layout v4.1(1fba170) 이후 무수정.
- **Python 변환 부재 확인**: `grep -rn "slot_to_logical\|physical_core_slots\|SlotToLogical" --include=*.py .` → 결과 없음.
- **NUC13 매핑 확정** (저장소 helper 를 NUC13 sysfs mock 으로 실행; `test_rt_common.sh::test_physical_core_slots_nuc13_hybrid` 의 `PCS.NUC13.slots` fixture 와 일치):
  ```
  generation          : raptor_lake_p (Raptor Lake-P mobile)
  num_p_physical      : 4  / E: 8  → get_physical_cores = 12 → 12-core 티어 (티어 선택은 정상)
  PHYSICAL_CORE_SLOTS : 0 2 4 6 8 9 10 11 12 13 14 15
  get_rt_shield_cpus  : 2-9
  ```
- **오핀 3건 확정**: DDS slot 2 → logical **2**(rt_control, 기대 4) / arm slot 6 → logical **6**(mpc_main, 기대 10) / hand slot 7 → logical **7**(mpc_main HT sibling, 기대 11). 세 타깃 모두 shield 범위 `2-9` 안쪽.
- **테스트 공백 확인**: `test_thread_layout.py` 에 `slot`/`logical`/`hybrid` 미등장 — slot 테이블만 비교하므로 이 버그에 구조적으로 눈멂.
- **중복 확인**: `_pin_external_driver` 가 `robot_ur5e_p1a` / `robot_ur5e_p1b` 에 복제. `thread_layout` 소비 런치 5개 (robot p1a/p1b, sim p1a/p1b, sim_iiwa7_leap).
- **의존 사전 확인**: `integrated_bringup/package.xml` 에 `<exec_depend>repo_scripts</exec_depend>` + `<exec_depend>rtc_tools</exec_depend>` **이미 존재** → 신규 의존 0건, ARCH-2 무관. `rt_common.sh` 는 `lib/repo_scripts/lib/rt_common.sh` 로 이미 설치됨 (`repo_scripts/CMakeLists.txt` 두 번째 `install(PROGRAMS)`), install 트리 존재 확인.
- **P5 검색**: slot→logical 유틸이 C++(`SlotToLogicalCpu`) · bash(`slot_to_logical_cpu`) 에 이미 존재 — Python 재구현은 3번째 mirror 이므로 기각, bash SSoT 위임.

## Current state

**2026-07-16**: Phase 0·1·2 완료, `fix/163-launch-slot-to-logical` 브랜치에 2 커밋. **handoff 경계 도달** — Phase 3·4 는 NUC13 실측(사용자 소유)에 의존.

- [x] Phase 0 — #163 생성, #151 · #152 코멘트, 본 artifact 생성
- [x] Phase 1 — `rtc_tools/launch/pinning.py` 신설 + 런치 5개 전환 + `_pin_external_driver` 중복 제거 (commit a015ce3)
- [x] Phase 2 — `test_pinning_slot_to_logical.py` 하이브리드 축 drift 테스트 (commit a14f97f)
- [ ] **handoff 경계** — NUC13 실측 (사용자 소유). #151 코멘트의 명령 세트 실행
- [ ] Phase 3 — DDS pin 루프 이름 필터 (nrt yank 차단). 실측과 독립, 착수 가능
- [ ] Phase 4 — `taskset -acp` (실측 gate)

## Next action

두 갈래가 병렬 가능:
1. **NUC13 실측** (사용자) — #151 코멘트의 명령(`cset shield -s`, `Cpus_allowed_list`, `ps -eLo comm,psr,cls,rtprio`)을 NUC13 에서 실행. 목적: (a) Phase 1 이 실제로 slot→logical 을 고쳤는지 런타임 확인, (b) cset/cpuset EINVAL 가설 확정/반증, (c) Phase 4 `-acp` 의 E-core 부하 판단.
2. **Phase 3** (에이전트) — `robot_ur5e_p1a/p1b` 의 DDS pin 루프(현재 `pin_dds_threads_to_slot` 헬퍼로 이동됨)에서 SCHED_FIFO 필터를 RTC 소유 스레드 이름(`rt_control`/`rt_callback`/`mpc_main`/`mpc_worker_*`/`nrt_logging`/`nrt_callback`) 제외로 교체. 이름 목록은 `thread_config.hpp` `.name` 필드 mirror → Phase 2 테스트에 함께 고정.

## Phase 1·2 검증 결과 (실측 evidence)

- `ruff check` 5 launch + pinning.py + test: **All checks passed**
- `colcon build --packages-select rtc_tools integrated_bringup`: **2 packages finished**
- `colcon test rtc_tools -k "thread_layout or pinning"`: **18 passed** (기존 12 + 신규 5 + launch_imports 1), 0 failure
- 렌더된 스니펫을 NUC13 mock 에서 실행: slot 6→**logical 10**, 7→**11**, 2→**4** (Sprint 기준 1 충족)
- 이 dev 박스(Ryzen 6C identity)·SMT-off 8core mock: slot==logical (비-하이브리드 무회귀)
- negative check: prelude 를 raw-slot 으로 되돌리면 slot 6→6 → 테스트가 실제로 fail 판정 (회귀 감지 확인)
- 설치 환경에서 5개 런치 `generate_launch_description()` 전부 OK
- end-to-end: `rt_common_path()` → 설치된 `install/repo_scripts/lib/repo_scripts/lib/rt_common.sh`, 실기 slot 2→2 (identity)
- symlink-install: `build/rtc_tools/rtc_tools` 가 소스 심링크라 pinning.py 자동 노출 (별도 install 불필요)

## Decisions and rationale

- **slot→logical 을 bash SSoT 에 위임** (Python 재구현 / subprocess 왕복 기각). Python 은 tier→slot 소유(`thread_config.hpp` 대비 drift 테스트 존재), bash 는 slot→logical 소유(`cpu_topology.hpp` 대비 `test_rt_common.sh` fixture 로 이미 테스트됨) — **새 mirror 가 생기지 않는다.** 런치는 이미 `bash -c` 문자열을 생성하고 `repo_scripts` share dir 를 찾으므로 추가 배관 불필요.
- **`_pin_external_driver` 를 `rtc_tools/launch/pinning.py` 로 승격** — 2곳 복제 + 소비 런치 5개. 설계원칙 P5(fork 대신 일반화). 승격 안 하면 같은 수정을 5곳에 반복.
- **`-acp` 를 Phase 1 에서 분리** (사용자 결정) — 현재 "메인 스레드만 핀" 상태가 우연한 부하 분산으로 작동 중일 수 있어, slot 수정과 `-a` 를 겹치면 지터 회귀 시 원인 분리가 불가능.
- **Phase 3 은 이름 필터까지만** (사용자 결정) — race 근본 해결(E-7)은 별도 이슈.
- **#152 는 NUC13 블로커 아님** — NUC13 primaries-first 라 `get_rt_cores_with_siblings` 와 `get_rt_shield_cpus` 가 둘 다 `2-9`. 우선순위 유지 근거를 "실기에서 깨질 수 있다" → "현 하드웨어 미발현, 교체 시 발현" 으로 갱신.

## Evidence

```
$ git show --stat --oneline e93d0d4
e93d0d4 fix(repo_scripts): correct RT-safety sensor & isolation logic (Phase 1)
 7 files changed, 228 insertions(+), 107 deletions(-)   # 전부 repo_scripts/

$ grep -rn "slot_to_logical\|physical_core_slots\|SlotToLogical" --include=*.py .
(결과 없음)

$ grep -rn "cset shield --exec\|cset shield --shield\|cset proc\|cset set" --include=*.sh --include=*.py .
(결과 없음)

# NUC13 sysfs mock + repo helper (rt_common.sh)
get_rt_cores (slots)              : 1,2,3,4,5
get_rt_shield_cpus  [cpu_shield]  : 2-9
get_rt_cores_with_siblings [GRUB] : 2-9      ← #152 divergence 주장 반증
PHYSICAL_CORE_SLOTS               : 0 2 4 6 8 9 10 11 12 13 14 15

# dev 박스 (AMD Ryzen 5 5600X 6C/12T) — 제어 PC 아님
physical cores: 6 → 6-core degraded 티어, PHYSICAL_CORE_SLOTS 항등 → 이 버그 미발현
```

Phase 1·2 build/test evidence: **미수집** (착수 전).

## Failed approaches

없음 (Phase 0 종료 시점).

기각한 대안 (착수 전 검토): Python sysfs 재구현(3번째 mirror), Python→bash subprocess 왕복(launch import 시점 비용).

## Constraints / pending human decisions

- **Phase 4 gate — NUC13 실측 필요**: `[CONCERN] Warning` — `taskset -acp` 는 UR 드라이버 전 스레드를 E-core 1개(logical 10)에 몰아넣는다. `ur_ros2_driver` 는 500Hz RTDE 루프를 돌리므로 E-core 1개로 부족할 수 있다. 즉 "계획대로 고치는 것"이 성능을 악화시킬 수 있고, 현재 상태는 버그와 우연한 완화가 겹쳐 있다. E-core 사용률 · RTDE 지터 실측 후 결정.
- **Phase 3 잔여 race**: `[CONCERN] Warning` — 이름 필터는 nrt yank 는 막지만 t=5s 이후 생성 DDS 스레드는 여전히 미핀. 별도 이슈로 분리 (사용자 결정).
- **#151 미검증 가설** — cset shield 활성 시 프로세스가 `system` cpuset 에 갇혀 `pthread_setaffinity_np` 가 EINVAL → `thread_utils.hpp:163` 이 SCHED_FIFO 설정 **전에** `return false` → rt_control 이 pin 과 FIFO 90 을 모두 잃음. **실기 확인 전까지 미검증.** 이것이 참이면 Phase 1 수정도 런타임에서 무력화되므로 Phase 1 검증 자체가 이 데이터를 필요로 한다.

## Workspace

- branch: `fix/163-launch-slot-to-logical` (from `main` @ `002b8c9`), 아직 push/PR 안 함
- 커밋: `a015ce3` (Phase 1), `a14f97f` (Phase 2)
- `git status`: untracked `docs/WBC_CONTROLLER_IMPLEMENTATION.md` — **본 task 와 무관, 다른 소유자**. 스테이징/커밋에서 제외함.
- 미커밋 변경: 본 artifact 갱신분(Phase 1·2 완료 반영)

## Pointers

- #163 (본 이슈), #151 (cpu_shield 실기 검증), #152 (GRUB nohz 통일)
- commit e93d0d4 (bash 쪽 동일 수정), 1fba170 (layout v4.1)
- `rtc_tools/rtc_tools/launch/thread_layout.py` — tier→slot mirror (수정 대상 아님, 역할 유지)
- `rtc_tools/launch/pinning.py` — **신설 예정**
- `rtc_tools/test/test_thread_layout.py` — 하이브리드 축 추가 대상
- `rtc_base/include/rtc_base/threading/thread_utils.hpp` — `SlotToLogicalCpu` (:115), `ApplyThreadConfig` affinity 실패 early-return (:163), `SelectThreadConfigs` (:889)
- `rtc_base/include/rtc_base/threading/cpu_topology.hpp` — `physical_core_slots` 생성 (:616-654)
- `rtc_base/include/rtc_base/threading/thread_config.hpp` — 티어 테이블 (SSoT, 변경 금지)
- `repo_scripts/scripts/lib/rt_common.sh` — `slot_to_logical_cpu` (:747), `get_rt_shield_cpus` (:829), `get_rt_cores` (:642)
- `repo_scripts/test/test_rt_common.sh` — NUC13 fixture (:497-516)
- `integrated_bringup/launch/robot_ur5e_p1b.launch.py` — `_pin_external_driver` (:79), DDS pin 루프 (:439-469), shield 게이트 (:396)
- `integrated_bringup/launch/robot_ur5e_p1a.launch.py` — `_pin_external_driver` 복제본
- `rtc_controller_manager/src/rt_controller_main_impl.cpp` — nrt 스레드 생성 (:182-183)
- `agent_docs/anti-patterns.md:95` — 동일 계열 결함 기술
