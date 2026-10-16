# RT 호스트 설정 — 근거·실측·이력

> **이 문서는 헌법이 아니다.** 절차는 [rt-optimization.md](../rt-optimization.md) 가, 규범은 [invariants.md](../../agent_docs/invariants.md#rt-host--runtime-preconditions) 의 RT-HOST-1~3 이 갖는다. 여기는 그 설정이 *왜* 이런 모양인지 · 무엇이 사고로 이어졌는지를 기록한다. 둘이 어긋나면 invariants.md 가 옳다. 수치·이름은 **기록 시점의 것**이고 다시 재야 한다.

## RT 코어 집합이 모든 tier 에서 1-3 인 이유

RT 스레드는 `rt_control` · `rt_callback` · `mpc_main` 세 개다. 한때 상위 슬롯 (10~11 코어는 4, 12 코어 이상은 5 까지) 이 RT 집합에 들어 있었는데, 거기 앉은 것은 `mpc_worker` 두 개뿐이었고 그 스레드들은 아무것도 실행하지 않았다 (#380). 그래서 실행하는 스레드가 없는 코어를 격리하고 `nohz_full` 로 tickless 로 만들고 있었다. 그 슬롯을 회수하자 코어 수와 무관하게 RT 집합이 같아졌다.

non-RT lane (`nrt_callback` · `nrt_publish` · `nrt_logging`) 은 aux slot 2 로 접혀 `rt_callback` 과 동거한다 (#349). FIFO 가 CFS 를 무조건 선점하므로 `rt_callback` 의 지연 계약은 유지되고, 반납된 슬롯은 system cpuset 으로 돌아간다. 실측 (NUC13 12 코어, `/proc/<tid>/stat` 차분) 에서 slot 2 의 `rt_callback` 은 약 1.5 %, non-RT 세 lane 은 합쳐 약 5.6 % 였다. 같은 슬롯에 둬도 여유가 충분하다. 반대로 CSV 의 timing scope 로 재면 약 6 배 낮게 나온다 — 콜백 본문만 재고 `rmw_wait` · take · deserialize 를 빼기 때문이다.

결과 (비-SMT 기준 slot 표기. 드라이버는 8 코어 이상에서 slot 4·5 로 전 tier 동일):

| 물리 코어 | `nohz_full` / `rcu_nocbs` | CM cpuset (RT 와 non-RT lane 의 합집합) | 격리 해제 영역 |
|---|---|---|---|
| 4 (degraded) | `1-3` | `1-3` (non-RT lane 은 OS slot 0 이라 제외) | slot 0 (OS · driver) |
| 6–7 | `1-3` | `1-3` | slot 0, 4–5 (4 는 driver) |
| 8–9 | `1-3` | `1-3` | slot 0, 4–7 (4·5 는 driver) |
| 10–11 | `1-3` | `1-3` | slot 0, 4–9 (4·5 는 driver) |
| 12–15 | `1-3` | `1-3` (NUC13 hybrid 의 logical 은 `2-7`) | slot 0, 4–11 (4·5 는 driver) |
| 16+ | `1-3` | `1-3` | slot 0, 4+ (4·5 는 driver) |

SMT 가 켜져 있으면 `setup_grub_rt.sh` 가 형제 logical CPU 를 합집합에 넣는다 (6C/12T 에서 `nohz_full=1-3,7-9`). 이 표는 `repo_scripts/test/test_rt_common.sh` 의 tier -> 기대값 테스트 (`test_get_rt_cores_with_siblings_non_smt`) 가 오라클로 삼는다. 10 코어 이상이 `1-4` · `1-5` 로 되돌아오면 아무도 실행하지 않는 코어를 다시 격리하고 있다는 뜻이다. 레이아웃 각 언어의 값 오라클은 일부러 손으로 쓴 리터럴이다. 생성물끼리 대조하면 잘못된 표가 세 언어에서 일치해 버려 잡히지 않기 때문이다.

## CM cpuset 은 RT 스레드 코어보다 넓다

cset 은 *프로세스* 를 cpuset 에 가두므로 (모든 스레드 포함), cpuset 은 RT 스레드 코어뿐 아니라 같은 프로세스의 non-RT lane 코어까지 덮어야 한다. 덮지 않으면 non-RT lane 의 self-pin 이 EINVAL 이 되고, `ApplyThreadConfig` 는 affinity 실패 시 SCHED_FIFO 설정 *전에* return 한다. 현재 레이아웃에서는 non-RT lane 이 RT 범위 안 (slot 2) 에 들어와 합집합이 RT 집합과 같지만, 그 포함은 **레이아웃의 우연이지 계산의 전제가 아니다.** 그래서 코드는 합집합을 유지한다 (`rt_common.sh` 의 `get_cm_shield_cpus` = RT 집합 합집합 non-RT lane, OS slot 제외).

cset exclusive cpuset 은 코어를 한 cpuset 에만 넣는다. non-RT lane 코어가 shield 에 들어가면 그 코어에 pin 하려는 *다른 프로세스* 의 스레드는 EINVAL 이다. 따라서 CM 밖 프로세스 (예: `udp_hand_driver` 의 failure detector) 의 보조 스레드는 non-RT lane 코어에 pin 하지 않고 `cpu_core = -1` 로 자기 프로세스 affinity 를 상속한다.

driver 프로세스 (`arm_driver` · `hand_driver`) 는 CM 스레드가 아니라서 shield 대상이 아니다. 그래서 driver 슬롯이 옮겨 가도 shield 범위가 변하지 않는다. `hand_driver` 는 프로세스가 스스로 자기 스레드를 핀하고, launch 는 그 self-pin 이 닿지 못하는 DDS 스레드만 밖에서 핀한다. 예전 `taskset -a` 스윕은 `-a` 때문에 보조 I/O 스레드를 도로 hand 코어로 끌어왔다 (#345).

## cset adopt 와 fail-closed

`cset shield --cpu` 는 기존 태스크를 `system` 으로 쓸어내고, 이후 런치되는 CM 은 `system` 을 상속해 RT 코어에 접근할 수 없다 (EINVAL). 그래서 shield 는 cpuset 을 *만들기만* 하고, CM 을 그 안으로 넣는 것은 런치의 몫이다 (#151). 런치는 configure 완료 후 · activate (=RT 스레드 생성) *전* 에 `cpu_shield.sh adopt <pid>` 로 CM 을 `user` cpuset 에 옮기고, ACTIVATE 를 그 프로세스의 **종료 코드** 에 게이트한다. 비동기 sudo 와 스레드 생성 사이의 race 를 없애려는 것이다. shield off · sudo 없음 · cset 미설치 런은 adopt 가 no-op (exit 0) 이고 기존 self-pin 경로를 탄다.

**adopt 가 필요했는데 실패하면 ACTIVATE 를 내지 않고 `inactive` 에 둔다 (#344).** 그대로 활성화하면 실기 500 Hz 모션 제어가 CFS 로, 핀 없이 돈다 — affinity 실패가 SCHED_FIFO 설정 전에 return 하기 때문이다. 그 상태가 조용했던 이유가 핵심이다. 스레드 이름 설정 (`pthread_setname_np`) 도 그 return 뒤에 있어 RT 스레드에 이름이 붙지 않았고, 이름으로 스레드를 찾는 `verify_rt_runtime.sh` 가 정책·affinity 검사를 통째로 건너뛰었다. 그래서 검증기도 함께 고쳐 **필수 스레드 미발견을 FAIL** 로 만들었다. "검증이 통과했다" 가 "검사할 대상이 없었다" 와 구분되지 않는 것이 이 사고의 구조다.

이 배선 (감지 · adopt · 게이트) 의 단일 구현은 `rtc_tools.launch.cpu_shield` 이고 real 2 + sim 3, 다섯 런치가 공유한다. 예전에는 런치마다 복사돼 있어서 #151 -> #233 -> cset 감지의 세 수정이 `robot_ur5e_p1b` 에만 들어갔다. 배선은 `integrated_bringup/test/test_launch_shield_wiring.py` 가 고정한다.

## RT-HOST-3 의 degraded 예외 (arm RT 루프)

`arm_driver` 프로세스는 SCHED_OTHER 이되 그 안의 `controller_manager` 제어 루프만 FIFO 50 으로 올라가 arm 코어에 핀된다. 그 핀은 `taskset` 이 아니라 launch 가 만드는 CM 파라미터가 건다 — `taskset` 은 main thread 밖을 못 옮기고, 500 Hz 루프는 main 이 아닌 별도 스레드이기 때문이다 (#343). arm 코어는 shield 밖 (`system` cpuset) 이라 이 루프는 affinity + FIFO 만 보장받고 다른 system task · IRQ 로부터 격리되지는 않는다.

cpuset 을 재설계해 arm 코어까지 보호하는 대신 **degraded 예외로 인정하고 실측 게이트를 두는 쪽을 택했다.** 이유는 둘이다. RTDE 는 어차피 커널 네트워크 스택과 소켓을 공유하므로 코어 격리만으로 닫히지 않는다. shield 를 arm 코어까지 넓히면 CM adopt 계약 (cset shield 는 user / system 2 분할) 과 충돌한다. 게이트는 NUC13 에서 `verify_rt_runtime.sh` 통과 + `cm_timing_log.csv` 지터 회귀 없음이다. 6 코어 tier 는 arm 과 hand 가 slot 4 를 공유하고 hand 의 UDP 수신 루프 (FIFO 65) 가 arm 루프 (FIFO 50) 보다 높으므로, 그 tier 의 측정은 반드시 **arm + hand 동시 구동** 에서 한다.

`verify_rt_runtime.sh` 가 arm 을 RT 루프 축으로 보는 이유도 같다. FIFO 요청이 거부돼도 affinity 는 적용되므로 위치만 보는 검사는 강등된 루프를 통과시킨다. 설정된 우선순위의 SCHED_FIFO 스레드가 없으면 FAIL 이다. 그 스레드는 pthread 이름이 없어 (전 TID 가 같은 `comm`) FIFO 여부로만 골라낼 수 있다.

## 격리에 `isolcpus` 를 쓰지 않는 이유

과거에는 `/etc/default/grub` 에 `isolcpus` · `nohz_full` · `rcu_nocbs` 를 같이 박았다. 세 가지 이유로 폐기했다. 빌드 중에도 코어가 격리되어 빌드가 느려진다. 재부팅 없이는 해제할 수 없다. non-RT lane 과 driver 까지 tickless 로 만들어 `nohz_full` 이 RT 스레드 코어만 대상으로 한다는 의도와 어긋난다. 지금은 부팅 옵션에 `nohz_full` · `rcu_nocbs` 만 두고 격리는 런타임 `cset shield` 가 맡는다.

## NVIDIA DKMS 가 RT 커널에서 막히는 세 층

NVIDIA 드라이버는 `CONFIG_PREEMPT_RT` 커널에서 빌드를 거부한다. 커스텀 RT 커널에서 모듈을 빌드하려면 세 층을 모두 우회해야 하고, `setup_nvidia_rt.sh` 와 `build_rt_kernel.sh --with-nvidia` 가 이를 처리한다.

| 차단 층 | 원인 | 우회 |
|---|---|---|
| `dkms autoinstall` 의 apport 검증 | 커스텀 커널 이름 (`linux-headers-*-rt-custom`) 거부 | `dkms build -m nvidia -v <ver> -k <kernel>` 직접 호출 |
| `conftest.sh` 의 RT 감지 | `CONFIG_PREEMPT_RT` 를 보고 빌드 거부 | `IGNORE_PREEMPT_RT_PRESENCE=1` 환경변수 |
| `dkms.conf` 의 `BUILD_EXCLUSIVE_CONFIG` | `!CONFIG_PREEMPT_RT` 조건 | 해당 줄 주석 처리 |

환경변수는 커널 갱신 때 DKMS 가 다시 빌드할 때도 필요하므로 `/etc/environment` 에 영구 등록한다. `check_rt_setup.sh` 의 NVIDIA 항목이 이 영구 설정을 확인한다.

## PREEMPT_RT 는 6.12 부터 mainline

PREEMPT_RT 는 Linux 6.12 에서 mainline 에 병합됐다 (x86_64 · arm64 · riscv). 6.12 이상은 외부 패치 없이 `CONFIG_PREEMPT_RT=y` 로 RT 가 켜진다. `build_rt_kernel.sh` 는 그래서 vanilla 6.12.x LTS 를 받아 kernel.org 서명을 검증하고, 6.12 미만 버전은 거부한다. 기본 버전을 스크립트에 pin 한 것은 두 머신 (데스크탑 · NUC) 의 재현성 때문이다. 올릴 때는 kernel.org 에서 최신 6.12.x 를 확인한다.

## CycloneDDS 의 affinity 제약

CycloneDDS 0.10.5 에는 CPU affinity 설정이 없다. `<Threads><Thread>` 는 있지만 `StackSize` 와 scheduling 만 담는다 (한때 "0.11 에서 제거됨" 이라고 적었으나 틀렸다, #164). 그래서 DDS 스레드의 고정은 launch 의 `taskset` 이 맡고, DDS 수신 스레드는 `rt_callback` 과 같은 slot 에 동거시킨다. SCHED_FIFO 가 CFS 를 무조건 선점하므로 RT 결정성에는 영향이 없다.
