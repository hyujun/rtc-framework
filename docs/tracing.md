# Tracing — ros2_tracing (LTTng UST + kernel sched_switch)

CSV timing log 는 tick 의 총 시간만 알려 준다. 어느 thread 가 어느 core 에서 언제 돌았는지, 어떤 callback 이 시간을 쓰는지가 궁금하면 LTTng trace 를 캡처해 Perfetto 로 본다. 이 문서는 사용법만 다룬다. 규범은 [testing-debug.md](../agent_docs/testing-debug.md) §Tracing, 설계 근거는 [tracing-rationale.md](reference/tracing-rationale.md), sensor matrix 는 [testing.md](testing.md) 에 있다.

## 언제 쓰나

`perf record` 의 sampling 은 짧은 구간을 놓친다. `ros2_tracing` 은 event 기반이라 정확한 begin/end 를 남긴다.

* `ros2:callback_start/end` — callback 의 정확한 시작·끝
* `sched_switch` (kernel) — 어느 thread 가 어느 core 에서 언제 돌았는가
* `irq_handler_entry/exit` — RT core 로 새는 IRQ

기본 캡처는 callback **바깥 경계**만 본다. RT 제어 루프 (raw `clock_nanosleep` pthread) 와 그 안의 함수 단위 breakdown 은 `--tracing` 빌드의 `rtc:*` span 으로 본다 ([RT trace spans](#rt-trace-spans-rtc--rt-tick-내부-빌드-opt-in)).

## 한눈에 시작

```bash
./install.sh --tracing                                                  # 1회: 도구·커널 모듈 설치
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true   # 캡처 (Ctrl+C 로 종료)
./repo_scripts/scripts/timeline.sh                                      # 최신 trace → trace.json, 그 뒤 ui.perfetto.dev 에 drag-drop
```

## 설치·권한

`./install.sh --tracing` 이 필요한 패키지 (lttng-tools, lttng-modules-dkms, babeltrace2, python3-bt2, ros2trace 등) 를 설치한다. 목록의 SSoT 는 `repo_scripts/scripts/lib/install_dev.sh` 의 `install_tracing_tools` 다. 그룹 멤버십만 필요하면 다음을 실행하고, 적용은 logout/login 또는 `newgrp tracing`.

```bash
sudo usermod -a -G tracing "$USER"
```

검증: `./repo_scripts/scripts/check_rt_setup.sh --summary` 가 `Tracing  lttng=1 kmod=1 group=1 launch=1` 을 내면 된다.

`kmod=0` 이면 사유가 `kmod_block=…` 로 함께 나온다.

| `kmod_block` | 뜻 | 처방 |
|---|---|---|
| `not_built` | 모듈이 아직 설치·빌드되지 않음 | `./install.sh --tracing` (또는 `sudo dkms status lttng-modules`) |
| `unsigned` | `.ko` 는 있으나 Secure Boot 서명이 없음 | 아래 Secure Boot |
| `kernel_incompat` | 이 커널을 지원하는 `lttng-modules` 가 배포판에 없음 | [Kernel timeline 가용성](#kernel-timeline-가용성-커널--lttng-modules). `install.sh --tracing` 을 반복해도 같은 자리에서 실패한다 |

### Secure Boot 호스트

`modprobe lttng-tracer` 가 `Key was rejected by service` 로 실패하면, 서명되지 않은 DKMS module 을 Secure Boot 가 거부하는 상태다 (`kmod_block=unsigned`). MOK (Machine Owner Key) 를 등록해 푼다.

```bash
./repo_scripts/scripts/enroll_lttng_mok.sh    # 키 생성 + DKMS 서명 설정 + 재빌드 + mokutil import
```

reboot 후 UEFI 'Perform MOK management' 화면에서 Enroll MOK → Continue → Yes → (helper 의 password) → Reboot. 재부팅 뒤:

```bash
sudo modprobe lttng-tracer && lsmod | grep lttng
sudo systemctl restart lttng-sessiond
lttng list --kernel                           # 이벤트 목록이 보이면 성공
```

## Permissions

| 경로 | 요구 권한 |
|---|---|
| UST tracing (`ros2:*`, `rtc:*`) | 그룹 불필요 |
| Kernel tracing (`sched_switch` 등) | `tracing` 그룹 멤버십 **또는** sudo |
| lttng-modules-dkms 빌드 | `linux-headers-$(uname -r)` + `build-essential` |

kernel event 가 EPERM 으로 실패하면 `groups` 출력을 확인하고 `newgrp tracing` 또는 logout/login 한다.

## RT trace spans (`rtc:*` — RT-tick 내부, 빌드 opt-in)

ros2_tracing 은 rclcpp 콜백만 본다. RT 제어 루프와 그 안의 WBC sub-step 은 자동 트레이스에 안 잡힌다. `rtc_base` 의 `RTC_TRACE_SCOPE(name)` 로 계측한 구간을 보려면 **tracing 빌드**가 필요하다. 컴파일타임 게이트라 기본 빌드는 완전 no-op 이다 (RT 비용 0).

```bash
./build.sh sim --tracing                      # liblttng-ust-dev 필요 (install.sh --tracing 이 설치)
colcon build --cmake-args -DRTC_ENABLE_TRACING=ON   # 직접 빌드할 때 (colcon 은 ws root 에서)
```

빌드 후 `enable_tracing:=true` 로 캡처하면 `timeline.sh` 가 span 을 스레드 레인의 **중첩 flame 스택**으로 그린다. 계측된 주기 스레드는 다음과 같다.

| 스레드 (프로세스) | L1 root span | 하위에서 보이는 것 |
|---|---|---|
| `rt_control` (`integrated_rt_controller`) | `rt_control_tick` | `CM::ReadDeviceState` · `CM::Compute` (WBC sub-step 포함) · `CM::WriteCommand` · `CM::PublishHandoff` |
| `mpc_main` (같은 프로세스) | `mpc_tick` | `mpc_read_state` · `mpc_solve` · `mpc_publish` |
| `nrt_publish` (같은 프로세스) | `nrt_publish_drain` | `owned_topics_publish` |
| `sim_thread` (`mujoco_simulator_node`) | `sim_step` | `sim_wait_command` · `sim_substep` |
| `viewer` (같은 노드) | `viewer_frame` | `viewer_sync` · `viewer_render` · `viewer_swap` |
| CommLoop · `failure_detector` (`udp_hand_node`) | `hand_comm_tick` (손마다 하나 — 두 손이면 tick 당 둘) · `hand_detector_tick` | `hand_cycle_txn` (bulk 의 한 cycle 트랜잭션; CAN FD 는 그 안의 `hand_cycle_window`) · `hand_read_*` · `hand_write_echo` · `hand_detector_check` |

executor 콜백 본문에도 L2 span 이 있다. 새 계측점은 함수 첫 줄에 `RTC_TRACE_SCOPE("Name");` 한 줄이면 된다 (RAII 라 early-return 에도 닫힘). span 계층과 cadence 판정의 규칙은 [testing-debug.md](../agent_docs/testing-debug.md#tracing), 그 근거와 계측 제외 대상은 [tracing-rationale.md](reference/tracing-rationale.md).

`enable_tracing:=false` 로 실행하면 span 이 컴파일돼 있어도 기록되지 않는다.

> **빌드 전환 주의**: `RTC_ENABLE_TRACING` 을 ON↔OFF 로 바꿀 때 colcon 이 재컴파일을 건너뛸 수 있다. 확실히 바꾸려면 `--cmake-clean-first` 또는 `./build.sh ... --clean` 을 쓴다.

## 캡처

```bash
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true
ros2 launch integrated_bringup robot_ur5e_p1a.launch.py enable_tracing:=true    # 실기
```

출력은 `<ws>/logging_data/<YYMMDD_HHMM>/tracing/<session_name>/` (CTF 디렉토리) 이고 CSV timing log 와 같은 session_dir 에 들어간다. Ctrl+C 로 끝내면 flush 는 자동이다.

| LaunchArgument | 설명 |
|---|---|
| `enable_tracing:=true\|false` | 캡처 on/off (기본 false) |
| `trace_session_name:=<str>` | `tracing/` 아래 leaf 폴더 이름 (기본 `trace`). 한 run 에서 여러 번 캡처하려면 다르게 준다 |
| `trace_events_ust:=<콤마 목록>` | UST event. 비우면 ros2_tracing 기본 `ros2:*`. `rtc:*` span 은 항상 추가된다 |
| `trace_events_kernel:=<콤마 목록>` | 기본 `sched_switch,sched_waking,sched_wakeup,irq_handler_entry,irq_handler_exit`. **`none` = kernel 비활성 (UST only)** |

`trace_events_kernel` 은 `none` / `off` / `false` / `0` 을 받는다 (kernel module 이나 tracing 그룹이 없을 때 UST only 로 쓴다). 빈 값 (`trace_events_kernel:=`) 은 ros2launch 가 거부한다.

```bash
# callback timing 만 (register 를 빼면 이름이 주소로 나온다)
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true \
    trace_events_ust:=ros2:callback_start,ros2:callback_end,ros2:rclcpp_callback_register
```

launch 밖에서는 `ros2 trace start --session-name <이름> --ust ... --kernel ...` / `ros2 trace stop <이름>` 을 쓴다 (출력 `~/.ros/tracing/<이름>/`, launch action 과 동시 사용 금지).

## 보기

```bash
./repo_scripts/scripts/timeline.sh                       # 최신 logging_data/*/tracing/*/ 자동 선택
./repo_scripts/scripts/timeline.sh <trace_dir> [out.json]  # 명시 입력. 기본 출력은 <trace_dir>/trace.json
./repo_scripts/scripts/timeline.sh --max-duration-s 5    # 앞 5초만 변환 (대용량 캡처)
./repo_scripts/scripts/timeline.sh --all-threads         # 외부 프로세스 요약 해제
```

`-` 로 시작하는 플래그는 변환기 `ctf_to_chrome_trace` 로 그대로 전달된다 (`--help`). 진행은 stderr 에 찍힌다. `python3-bt2` 가 없으면 느린 `babeltrace2` CLI 를 쓴다.

https://ui.perfetto.dev 에 JSON 을 drag-drop 한다 (설치 불필요). swimlane 은 위에서 아래로 다음 순서다.

* **Workspace 프로세스** — `rtc:span_*` 을 낸 프로세스. thread 별 row 의 중첩 스택으로 RT tick·MPC 주기를 확인한다.
* **Cpus** — `Cpu 000` … 코어별 `sched_switch` timeline. RT thread 가 의도한 코어에 고정됐는지, migration 이 없는지 본다.
* **외부 프로세스 요약** — span 이 없는 UST 프로세스 (vendor driver 등) 는 async lane 1개로 축약된다. 펼치려면 `--all-threads`, 일부만 승격하려면 `--focus-proc <substr,...>`.
* **Cpu/IRQ** — IRQ handler entry/exit. RT core 로 새는 IRQ 를 본다.

span 이 하나도 없는 캡처 (tracing OFF 빌드) 는 전 프로세스를 per-thread 로 그린다. callback 이름은 `ros2:rclcpp_callback_register` 로 해석되며, 없으면 `callback@0x...` 로 나온다. Perfetto 에서 shift+드래그는 범위 측정, Ctrl+F 는 symbol 검색이다.

### JSON 크기 — Perfetto 가 안 열리거나 레인이 비어 보일 때

JSON 은 이벤트당 약 100 B 다. 프로세스·스레드 이름은 파일 앞에, 슬라이스는 뒤에 있어서 뷰어가 끝까지 못 읽으면 **그룹은 보이는데 레인이 빈** 상태가 되고, 이는 변환기 버그나 캡처 결손과 겉보기가 같다. 변환기는 250 MB 초과 시 경고한다. 뷰가 아니라 캡처를 좁혀 해결한다.

```bash
# 1순위: 창 잘라 보기 (모든 lane 유지, 295 MB/2m6s → 47 MB/21s)
./repo_scripts/scripts/timeline.sh --max-duration-s 5
# 2순위: 필요한 프로세스만
./repo_scripts/scripts/timeline.sh --focus-proc integrated_rt
```

3순위는 캡처 자체를 좁히는 것이다 (`rtc:*` span 만 필요하면 `trace_events_kernel:=none`).

## 문제 해결

### Event loss — 트레이서가 이벤트를 버린 경우

LTTng 의 per-CPU 링 버퍼는 discard 모드가 기본이다. 버퍼가 `lttng-consumerd` 의 비우는 속도보다 빨리 차면 이벤트를 **버리고**, 버린 개수만 packet header 에 남긴다. 살아남은 이벤트는 멀쩡해 보이므로 이 카운터가 유일한 신호다. 변환기가 유실을 stderr 에 경고한다 (총 개수, 비율, CPU 별 최다 유실).

```
[ctf_to_chrome] WARNING: the tracer DISCARDED 3,505,417 events (56.8% of 6,174,930) — ...
```

Cpus 레인은 `sched_switch` 를 재생해 그리므로 switch 하나가 유실되면 그 CPU 의 다음 switch 까지 color bar 가 사라진다. 그 스레드의 `rtc:*` span 은 정상이라 **"스레드는 연산 중인데 CPU 에는 아무것도 없다"** 로 보인다.

대응은 효과 순으로 다음과 같다.

1. **캡처를 좁힌다** — `trace_events_kernel:=sched_switch` 만으로도 Cpus 뷰가 재구성된다.
2. **런을 짧게** — 유실은 부하 구간에 몰린다.
3. **버퍼 확대** — `subbuffer_size_kernel` (기본 128 KB/CPU) 이지만 `make_trace_action` 이 아직 노출하지 않는다.

`--max-duration-s` 를 쓰면 유실 집계도 변환한 창 안만 센다. babeltrace2 CLI fallback 경로 (`python3-bt2` 미설치 또는 `--stdin`) 는 이 카운터를 읽지 못한다. 그 경로에서 경고가 없다는 것은 "유실 없음"이 아니라 "확인 안 됨"이다.

### Kernel timeline 가용성 (커널 × lttng-modules)

UST (`ros2:*` · `rtc:*`) 는 커널 모듈과 무관하게 항상 동작한다. 이 절은 kernel event (`sched_switch` · IRQ) 가 가능한 조합을 다룬다.

`lttng-modules` 는 커널 tracepoint 프로토타입을 재선언해서, 커널이 바꾸면 결정론적 컴파일 실패가 난다. 재시도·재부팅으로 안 풀린다. 배포판 `lttng-modules 2.14.0` 의 실측 경계:

| 커널 | 결과 | 비고 |
|---|---|---|
| 6.1.175+ · 6.6.141+ · **6.12.91+** · 6.18.33+ · 7.0.10+ | **빌드 실패** | `hrtimer_start` 에 인자가 추가된 stable 계열. `build_rt_kernel.sh` 기본 커널이 여기 든다 |
| 그 밖 (6.12.90 이하, 6.17.x 등) | 빌드 성공 | |

upstream 은 **v2.14.6** 에서 대응했다. `ppa:lttng/stable-2.14` 는 최신이 2.14.4 라 해결책이 아니다. `install.sh --tracing` 과 `check_rt_setup.sh` 는 `uname -r` 이 아니라 **커널 헤더의 버전** 으로 판정한다 (`rt_common.sh` 의 `lttng_modules_compat_verdict`). 배포판이 수정만 backport 했다면 `RTC_LTTNG_SKIP_COMPAT_GATE=1` 로 게이트를 우회한다.

#### 지원되지 않는 커널 — UST-only (현재 정책)

```bash
ros2 launch integrated_bringup sim_ur5e_p1a.launch.py enable_tracing:=true trace_events_kernel:=none
```

못 보는 것은 `Cpus` 레인 · IRQ · 비-UST 스레드 (DDS / rclcpp 내부) 다. `rtc:*` span 과 callback timing 은 영향이 없고, 스레드가 어느 코어에 있는지는 `verify_rt_runtime.sh` 가 `/proc` 로 읽는다.

#### 커널 타임라인이 필요해지면 — upstream lttng-modules 를 DKMS 로 등록

**필요해진 시점에 수행하는 절차이며 현재 설정이 아니다.** MOK 단계가 재부팅을 요구해 자동화하지 않는다.

필요한 최소 버전은 위 표가 아니라 도구가 답한다.

```bash
source repo_scripts/scripts/lib/rt_common.sh
lttng_modules_min_version_for_kernel "$(lttng_kernel_build_version "$(uname -r)")"   # 예: 2.14.6
```

1. **소스와 `dkms.conf` 를 합친다.** upstream git 에는 `dkms.conf` 가 없지만 배포판 deb 에는 완성본이 있다 (2.14.0 과 2.14.6 의 모듈 집합은 동일). 버전 문자열만 바꿔 재사용한다.

   ```bash
   VER=2.14.6
   git clone -b v${VER} --depth 1 https://github.com/lttng/lttng-modules.git /tmp/lttng-${VER}
   apt-get download lttng-modules-dkms && dpkg-deb -x lttng-modules-dkms_*.deb /tmp/lttng-deb
   cp /tmp/lttng-deb/usr/src/lttng-modules-*/dkms.conf /tmp/lttng-${VER}/
   sed -i "s/^PACKAGE_VERSION=.*/PACKAGE_VERSION=\"${VER}\"/" /tmp/lttng-${VER}/dkms.conf
   ```

2. **등록 전에 빌드가 되는지 확인한다** (시스템 변경 없음).

   ```bash
   ( cd /tmp/lttng-${VER} && KERNELDIR=/lib/modules/$(uname -r)/build make -j"$(nproc)" modules )
   ```

3. **DKMS 등록:**

   ```bash
   sudo cp -r /tmp/lttng-${VER} /usr/src/lttng-modules-${VER}
   sudo dkms add -m lttng-modules -v ${VER}
   sudo dkms build -m lttng-modules -v ${VER}
   sudo dkms install -m lttng-modules -v ${VER}
   ```

4. **Secure Boot 호스트면** 서명 없이는 load 가 거부되므로 위 Secure Boot 절의 `enroll_lttng_mok.sh` 를 이어서 수행한다.
5. 검증: `check_rt_setup.sh --summary` 가 `kmod=1` 로 바뀐다.

**유지 비용이 있다.** 커널이 오를 때마다 upstream 가드를 다시 확인하고 재빌드해야 한다.

### Event drop policy (JSON size)

*Event loss* 와 다르다. 이쪽은 변환기가 **의도적으로** 안 그리는 것이다. 기본 변환은 callback B/E + `sched_switch` + `irq_handler_*` 만 slice 로 내고 `sched_wakeup` · `ros2:rclcpp_publish` 등은 버린다 (JSON 이 2 배 가까이 커지기 때문). 무엇이 잘렸는지는 변환 끝의 `dropped N non-structured events ...` 요약에 나온다. 다시 보려면:

```bash
./repo_scripts/scripts/timeline.sh --keep-events ros2:rclcpp_publish,sched_wakeup   # 특정 종류만
./repo_scripts/scripts/timeline.sh --keep-all                                       # 전부 instant marker (큰 JSON)
```

## 그 밖의 도구

JSON 변환 없이 텍스트로 보려면 babeltrace2 를 직접 쓴다. 변환기에 파이프할 수도 있다 (`--stdin`, Event loss 검사는 안 된다).

```bash
babeltrace2 logging_data/<session>/tracing/<lttng_session>/ | head -50
babeltrace2 logging_data/<session>/tracing/<lttng_session>/ \
    | python3 -m rtc_tools.conversion.ctf_to_chrome_trace --stdin --output trace.json
```
