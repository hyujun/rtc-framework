# NUC Hybrid CPU Support

> **Status**: hybrid 식별 + slot 매핑 landed. Layout v4.1 이 `CpuTopology::physical_core_slots` 추상화 (P/E/LP-E asc) 를 통해 hybrid 분기를 우회 — 별도 `Make_NucHybrid_*` config 없이 모든 tier 가 P-core primary logical 에 자동 핀 ([RT_OPTIMIZATION.md:117](RT_OPTIMIZATION.md#L117)).
>
> **Entry points**:
> - C++: [`rtc_base/include/rtc_base/threading/cpu_topology.hpp`](../rtc_base/include/rtc_base/threading/cpu_topology.hpp) (`SlotToLogicalCpu`, CPUID family.model fallback)
> - Shell: [`repo_scripts/scripts/lib/rt_common.sh`](../repo_scripts/scripts/lib/rt_common.sh) (`detect_hybrid_capability`, `get_nuc_generation`)

## 지원 현황

| 세대 | 대표 모델 | 동작 |
|---|---|---|
| Raptor Lake-P | NUC 13 Pro (i5-1340P · i7-1360P · i7-1370P) | 감지 OK, slot 매핑으로 P-core 자동 핀 |
| Meteor Lake | NUC 14 Pro (Core Ultra 7 155H / 165H) | 감지 OK, slot 매핑으로 P-core 자동 핀 |
| Arrow Lake-H | NUC 15 Pro+ (Core Ultra 7 265H / 9 285H) | 감지 OK, slot 매핑으로 P-core 자동 핀 |
| NUC 13 Pro (BIOS HT off) | — | `check_rt_setup.sh` FAIL — HT 활성화 필수 |
| AMD / homogeneous Intel | — | non-hybrid slot 매핑 (physical core first-logicals asc) |

## BIOS 체크리스트 (Intel NUC hybrid)

- [ ] **Hyper-Threading**: Enabled — slot 매핑이 P-core primary logical (sibling 회피) 에 의존. `check_rt_setup.sh` 가 disabled 를 FAIL.
- [ ] **Intel Turbo Boost**: Enabled — P-core 부스트로 MPC solve 지연 감소.
- [ ] **C-states**: C1/C2 이하 제한 또는 GRUB 에 `intel_idle.max_cstate=1 processor.max_cstate=1`.
- [ ] **Speed Shift (HWP)**: Enabled (`intel_pstate=active`).
- [ ] **VT-d / IOMMU**: 가능한 Disabled (실측 후 결정).
- [ ] **E-core / LP E-core**: Enabled 유지 — OS / IRQ 가 활용.

## 환경변수

### `RTC_FORCE_HYBRID_GENERATION`

컨테이너 / sysfs 제한 환경에서 세대 enum 만 override. ID 리스트 (`p_core_physical_ids` 등) 는 여전히 sysfs 에서 파생.

| 값 | 의미 |
|---|---|
| `raptor_lake_p` | NUC 13 Pro class |
| `meteor_lake` | NUC 14 Pro class |
| `arrow_lake_h` | NUC 15 Pro+ class |
| `raptor_lake_p_ht_off` | BIOS HT off 시나리오 재현 (테스트용) |
| `none` | NOT_NUC_HYBRID 강제 |

**Production 이미지에는 반드시 unset** — `check_rt_setup.sh` 가 값이 남으면 WARN.

### `RTC_SYSFS_ROOT` / `RTC_PROC_CPUINFO`

테스트 전용. 기본 `/sys`, `/proc/cpuinfo`. `test_rt_common.sh` / `test_cpu_topology.cpp` mock 디렉토리 구성 시만.

## 검증

```bash
cd ~/ros2_ws/rtc_ws
colcon test --packages-select rtc_base --event-handlers console_direct+      # test_cpu_topology
colcon test --packages-select repo_scripts --event-handlers console_direct+   # test_rt_common
src/rtc-framework/repo_scripts/scripts/check_rt_setup.sh                       # [2.5/9] Hybrid CPU Detection
```
