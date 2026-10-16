# RT 호스트 설정 — 근거·실측·이력

> **이 문서는 헌법이 아니다.** 절차는 [rt-optimization.md](../rt-optimization.md) 가, 규범은 [invariants.md](../../agent_docs/invariants.md#rt-host--runtime-preconditions) 의 RT-HOST-1~3 이 갖는다. 여기는 그 설정이 *왜* 이런 모양인지 · 무엇이 사고로 이어졌는지를 기록한다. 둘이 어긋나면 invariants.md 가 옳다. 수치·이름은 **기록 시점의 것**이고 다시 재야 한다.

## RT 코어 집합이 모든 tier 에서 1-3 인 이유

RT 스레드는 `rt_control` · `rt_callback` · `mpc_main` 세 개다. 한때 상위 슬롯 (10~11 코어는 4, 12 코어 이상은 5 까지) 이 RT 집합에 들어 있었는데, 거기 앉은 것은 `mpc_worker` 두 개뿐이었고 그 스레드들은 `ApplyThreadConfig` 직후 반환해 아무것도 실행하지 않았다 (#380). 실행하는 스레드가 없는 코어를 격리하고 `nohz_full` 로 tickless 로 만들고 있었던 셈이다. 그 스레드를 쓸 solver 도 없었다 — Aligator 의 병렬화는 OpenMP 이고 그 런타임이 자기 스레드를 소유·생성하므로 외부 `std::jthread` 를 넘겨받는 API 자체가 없다. 슬롯을 회수하자 코어 수와 무관하게 RT 집합이 같아졌다. 같은 전제로 죽은 `DegradationMode` (#379) 의 이력은 `cpu_topology.hpp` 의 주석이 갖는다.

non-RT lane (`nrt_callback` · `nrt_publish` · `nrt_logging`) 은 aux slot 2 로 접혀 `rt_callback` 과 동거한다 (#349). FIFO 가 CFS 를 무조건 선점하므로 `rt_callback` 의 지연 계약은 유지되고, 반납된 슬롯은 system cpuset 으로 돌아간다. 실측치는 `repo_scripts/config/thread_layout.yaml` 의 v5 주석에 있다.

결과로 `nohz_full` / `rcu_nocbs` 는 4 코어 degraded 부터 16+ 까지 전부 `1-3` 이다 (SMT 면 `setup_grub_rt.sh` 가 형제 logical CPU 를 더한다). tier 별 `nohz_full` 기대값은 `repo_scripts/test/test_rt_common.sh` 의 `test_get_rt_cores_with_siblings_non_smt` 리터럴이, tier 별 thread × core 배치 (driver slot 포함) 는 [rtc_base/README.md](../../rtc_base/README.md) 의 생성 매트릭스가 갖는다. 10 코어 이상이 `1-4` · `1-5` 로 되돌아오면 아무도 실행하지 않는 코어를 다시 격리하고 있다는 뜻이다. 그 테스트의 기대값이 손으로 쓴 리터럴인 것은 일부러다 — 생성물끼리 대조하면 잘못된 표가 세 언어에서 일치해 버려 잡히지 않는다.

## CM cpuset 은 RT 스레드 코어보다 넓다

cset 은 *프로세스* 를 cpuset 에 가두므로, cpuset 은 RT 스레드 코어뿐 아니라 같은 프로세스의 non-RT lane 코어까지 덮어야 한다. 덮지 않으면 non-RT lane 의 self-pin 이 EINVAL 이 되고, `ApplyThreadConfig` 는 affinity 실패 시 SCHED_FIFO 설정 *전에* return 한다. 현재는 non-RT lane 이 slot 2 라 합집합이 RT 집합과 같지만 그 포함은 **레이아웃의 우연이지 계산의 전제가 아니다** — 그래서 `get_cm_shield_cpus` 는 합집합을 유지한다.

반대로 exclusive cpuset 은 코어를 한 cpuset 에만 넣으므로, shield 안 코어에 pin 하려는 *다른 프로세스* 의 스레드는 EINVAL 이다. `udp_hand_driver` 의 failure detector 가 그 예라서 `cpu_core = -1` 규칙 ([architecture.md](../../agent_docs/architecture.md) §Threading Model) 이 생겼다. driver 프로세스는 CM 스레드가 아니라 shield 대상이 아니며, `hand_driver` 는 스스로 자기 스레드를 핀하고 launch 는 DDS 스레드만 밖에서 핀한다. 예전 `taskset -a` 스윕은 보조 I/O 스레드를 도로 hand 코어로 끌어왔다 (#345).

## cset adopt 와 fail-closed

`cset shield --cpu` 는 기존 태스크를 `system` 으로 쓸어내므로 이후 런치되는 CM 은 RT 코어에 접근할 수 없다 (EINVAL). 그래서 shield 는 cpuset 만 만들고 CM 을 `user` 로 옮기는 adopt 는 런치의 몫이며 (#151), adopt 실패 시 ACTIVATE 를 내지 않는다 (#344). 이 순서·게이트·`verify_rt_runtime.sh` 가 사고를 놓친 경위는 `rtc_tools.launch.cpu_shield` 모듈 docstring 이 갖는다. 거기 없는 교훈 하나 — "검증이 통과했다" 가 "검사할 대상이 없었다" 와 구분되지 않는 것이 이 사고의 구조여서, 검증기는 **필수 스레드 미발견을 FAIL** 로 다룬다. 배선을 다섯 런치가 한 모듈로 공유하는 것은 예전에 복사본마다 수정 (#151 -> #233 -> cset 감지) 이 한 런치에만 들어갔기 때문이다.

## RT-HOST-3 의 degraded 예외 (arm RT 루프)

`arm_driver` 프로세스는 SCHED_OTHER 이되 그 안의 `controller_manager` 제어 루프만 FIFO 50 으로 올라가 arm 코어에 핀된다. 그 핀은 `taskset` 이 아니라 launch 가 만드는 CM 파라미터 파일 (upstream `cpu_affinity` · `thread_priority`) 이 건다 — `taskset` 은 main thread 밖을 못 옮기고, 500 Hz 루프는 별도 스레드이기 때문이다 (#343). arm 코어는 shield 밖 (`system` cpuset) 이라 이 루프는 affinity + FIFO 만 보장받고 다른 system task · IRQ 로부터 격리되지는 않는다.

cpuset 을 재설계해 arm 코어까지 보호하는 대신 **degraded 예외로 인정하고 실측 게이트를 두는 쪽을 택했다.** 이유는 둘이다. RTDE 는 어차피 커널 네트워크 스택과 소켓을 공유하므로 코어 격리만으로 닫히지 않는다. shield 를 arm 코어까지 넓히면 CM adopt 계약 (cset shield 는 user / system 2 분할) 과 충돌한다. 게이트는 NUC13 에서 `verify_rt_runtime.sh` 통과 + `cm_timing_log.csv` 지터 회귀 없음이다. 6 코어 tier 의 측정 조건 (arm + hand 동시 구동) 은 [rt-optimization.md](../rt-optimization.md) §4.3.

`verify_rt_runtime.sh` 가 arm 을 FIFO 우선순위로 찾는 것도 같은 이유다 — FIFO 요청이 거부돼도 affinity 는 적용되므로 위치만 보는 검사는 강등된 루프를 통과시킨다.

## 격리에 `isolcpus` 를 쓰지 않는 이유

과거에는 `/etc/default/grub` 에 `isolcpus` · `nohz_full` · `rcu_nocbs` 를 같이 박았다. 세 가지 이유로 폐기했다. 빌드 중에도 코어가 격리되어 빌드가 느려진다. 재부팅 없이는 해제할 수 없다. non-RT lane 과 driver 까지 tickless 로 만들어 `nohz_full` 이 RT 스레드 코어만 대상으로 한다는 의도와 어긋난다. 지금은 부팅 옵션에 `nohz_full` · `rcu_nocbs` 만 두고 격리는 런타임 `cset shield` 가 맡는다.

## CycloneDDS 의 affinity 제약

CycloneDDS 에는 CPU affinity 설정이 없다 (설치된 버전을 확인할 것). `<Threads><Thread>` 는 있지만 `StackSize` 와 scheduling 만 담는다 — 한때 "0.11 에서 제거됨" 이라고 적었으나 틀렸다 (#164). 그래서 DDS 스레드의 고정은 launch 의 `taskset` 이 맡고, DDS 수신 스레드는 `rt_callback` 과 같은 slot 에 동거시킨다. SCHED_FIFO 가 CFS 를 무조건 선점하므로 RT 결정성에는 영향이 없다.
