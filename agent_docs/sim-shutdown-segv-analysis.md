# Sim Shutdown SEGV — Investigation Notes (2026-04-26)

> **상태**: 부분 해결 (5초 hang + 15초 hang 모두 fix). 종료 시 `mpc_main` SEGV는 남음.
> **재개 시 시작점**: §6 "Remaining: mpc_main thread SEGV".

## 1. Symptom

`ros2 launch ur5e_bringup sim.launch.py` → Ctrl+C → `ur5e_rt_controller` process가:
- 초기엔 5초 후 SIGTERM, 그 다음 SIGSEGV (-11)로 죽음
- 두 번째 fix 후엔 15초 hang (5s SIGINT→SIGTERM, 10s SIGTERM→SIGKILL) → 결국 SEGV
- 마지막 fix 후엔 hang 사라지고 SIGINT 직후 즉시 -11

## 2. Stages of investigation

### Stage 1 — 5초 sim CV hang (fix 적용됨, 검증)

**원인**: `rtc_controller_manager/src/rt_controller_node_rt_loop.cpp` sim mode RT 루프가 `state_cv_.wait_for(lock, sim_timeout, ...)` 안에서 5초간 (== `sim_sync_timeout_sec` 기본값) 잠듦. predicate가 `state_fresh_ || !rt_loop_running_`만 체크하고 `rclcpp::ok()` 누락. SIGINT로 mujoco가 `/joint_states` 안 보내면 cv 안 깨어남, RT thread는 destructor의 `rt_loop_thread_.join()`을 5초간 막음.

**Fix** (commit 후보 1):
- CV wait predicate에 `!rclcpp::ok()` 추가
- `StopRtLoop()`에서 `state_cv_.notify_all()` 호출 — destructor / on_deactivate / on_shutdown 경로에서 RT thread 즉시 깨움
- `rtc_controller_manager/README.md` "RT 루프 종료" 섹션 추가

**검증**: 다음 sim에서 5초 hang 사라짐 확인.

### Stage 2 — 15초 executor wait hang (fix 적용됨, 검증)

**원인**: main()의 `t_aux.join()`이 `aux_executor.spin()` 안의 `rmw_fastrtps_shared_cpp::__rmw_wait` (FastDDS WaitSet) 에서 영원히 대기. SIGINT가 `rclcpp::ok()`를 false로 만들어도 rmw guard condition이 wait set을 깨우지 못해서 spin이 안 끝남. 핵심: `aux_executor.add_node(ctrl_node->get_node_base_interface())` 로 controller LifecycleNode들이 attach되면서 wait set 구조가 복잡해짐.

**진단 방법**: SIGTERM 후 SEGV가 떨어진 core (`/tmp/core.mpc_main.*`)의 thread bt → main thread frame이 `__GI___clock_nanosleep` (FastDDS DomainParticipantFactory dtor의 atexit) 또는 `__rmw_wait`. 명시적으로 `executor.cancel()`을 호출 안 한 게 문제.

**Fix** (commit 후보 2):
`rtc_controller_manager/src/rt_controller_main_impl.cpp` main():
1. `t_*.join()` 호출 *전*에 `while (rclcpp::ok()) sleep_for(50ms)` polling으로 SIGINT 대기
2. 깨어나면 `sensor_executor.cancel(); log_executor.cancel(); aux_executor.cancel();` 명시적 호출
3. 그 후 join — 이때 모든 spin이 즉시 반환
4. `aux_executor.remove_node()` 명시적으로 controller_nodes_ + main node 제거 → local destructor에서 race 방지

**검증**: 15초 hang 메시지 사라짐 (SIGTERM/SIGKILL 로그 없음). SIGINT 직후 빠르게 종료.

## 3. Remaining: mpc_main thread SEGV (UNRESOLVED)

### Symptom

마지막 fix 후에도 `[ERROR] process has died ... exit code -11`. dmesg는 `mpc_main[<pid>]: segfault at ... ip ... in ur5e_rt_controller[<offset>]`.

### Pattern of crashes

| 시각 | PID | IP offset | 단계 |
|------|-----|-----------|------|
| 19:10:20 | 46542 | 0x42eea0 | Stage 0 (5초 hang 전) |
| 19:17:38 | 50315 | 0x42f640 | Stage 0 |
| 19:20:49 | 50929 | 0x42f640 | Stage 0 |
| 19:21:37 | 51071 | 0x42f640 | Stage 0 |
| 20:00:02 | 52502 | 0x42f640 | Stage 1 fix 후 |
| 20:12 | 56105 | 0x42f640 | Stage 1 fix 후 |
| 20:14 | 56479 | 0x42f640 | Stage 1 fix 후 |
| 20:15 | 56833 | 0x42f640 | Stage 1 fix 후 |
| 20:23 | 62797 | (확인 필요) | Stage 1 fix 후 |
| 20:25 | 63011 | 0x42f640 | Stage 1 + node.cpp/seqlock 시도 후 (revert됨) |
| 20:26 | 63333 | **0x42f408** | 위와 동일 단계 |
| 20:28 | 66591 | **0x42f6d8** | **Stage 2 fix 후** |
| 20:31 | 67652 | **0x42f110** | **Stage 2 fix 후 (가장 최신)** |

> Stage 2 (executor cancel) fix 이후 IP offset이 `0x42f110/0x42f6d8`로 **달라짐** (이전엔 모두 `0x42f640` = `MPCSolutionManager::ReadState() const + 32` = SeqLock retry loop top). 즉 Stage 2 fix가 SeqLock spin hang은 잡았지만, **다른 use-after-free**가 노출됨.

### Confirmed facts (gdb 분석으로)

- SEGV thread는 `mpc_main` (per-controller MPC worker, [rtc_mpc/src/thread/mpc_thread.cpp](../rtc_mpc/src/thread/mpc_thread.cpp) 의 `RunMain`)
- Fault address (`r9` register)는 **mmap gap에 위치한 unmapped 영역** (예: `0x78003c53c8c0`은 두 .so 매핑 사이 빈 공간) — 즉 use-after-free 후 mimalloc이 페이지를 OS로 munmap한 영역.
- 첫 SEGV들 (offset 0x42f640): `MPCSolutionManager::ReadState() const + 32` = SeqLock `Load()`의 retry loop top (`mov (%r9), %eax`).
- 두 번째 SEGV들 (Stage 2 fix 후 0x42f110/0x42f6d8): 함수명 미확인. **재개 시 첫 일거리: `addr2line` 또는 `disas`로 새 SEGV가 어떤 함수의 어떤 line인지 확정**.

### Root-cause hypotheses (검증 안 됨)

H1. **`~DemoWbcController` 자체가 호출되지 않음**.
- 멤버 자동 destruct 순서로는 `mpc_thread_` (line 360)가 `mpc_manager_` (line 359)보다 먼저 destruct되어야 함. base `~MPCThread()`가 `Join()` 호출, main_thread join 시도.
- 그런데 mpc_main이 살아있다 = Join이 호출되지 않았거나 실패.

H2. **`HandlerMPCThread::~HandlerMPCThread() override = default` 가 위험**:
- Default destructor는 (1) derived 멤버 (`handler_`, `phase_manager_`, `pdata_`, scratch buffers) destruct → (2) base `~MPCThread()` → (3) `Join()`.
- (1) 시점에 main_thread가 살아있고 `Solve()` 안에서 derived 멤버 사용 중이면 use-after-free.
- 하지만 SEGV는 ReadState (Solve 직전)에서 났으므로 직접 원인은 아닐 수도.

H3. **SeqLock writer가 odd seq로 멈춰서 reader 무한 spin**:
- RT thread가 Store 도중 (seq → odd 직후, seq → even 직전)에 종료하면 seq 영원히 odd. Reader (mpc_main)는 retry loop 영원. 그동안 manager가 free → unmap → SEGV.
- Stage 2 fix 후에도 IP offset이 다르긴 하나, **`SeqLock::Load()` 안의 다른 명령어**일 수도 있음 (확정 필요).

### What was tried but did NOT fix

- **`SeqLock::Load()`에 `kMaxRetries=1024` 추가**: SEGV는 retry loop 안이어도 fault가 *다른 명령어*에서 났거나, retry는 영향 없는 다른 use-after-free였음. **revert됨**.
- **`~RtControllerNode`에 명시적 `controllers_.clear() + controller_nodes_.clear()` 추가**: `~DemoWbcController`가 자동 멤버 destruct 순서대로 호출되어야 하는데 SEGV 여전 → 자동 vs 명시적 clear는 차이 없음. **revert됨**.

### 다음에 시도할 것

1. **새 SEGV의 정확한 위치 확정**:
   ```bash
   gdb /home/junho/ros2_ws/rtc_ws/install/ur5e_bringup/lib/ur5e_bringup/ur5e_rt_controller \
       /tmp/core.mpc_main.<latest> \
       -batch -ex "thread 1" -ex "info registers" \
       -ex "disas /s \$pc, +32"
   addr2line -e .../ur5e_rt_controller -fC 0x42f110
   ```
2. 함수 확인 후 main_thread가 use-중인 객체와 그 객체의 owner 추적.
3. 가장 가능성 높은 구조적 fix:
   - `DemoWbcController`에 명시적 destructor 추가 → 첫 줄에서 `if (mpc_thread_) mpc_thread_.reset();`. 다른 멤버 destruct 시작 *전*에 thread join 보장.
   - `MPCThread::~MPCThread()`에 `Join()` 호출은 있지만, derived 멤버 destruct 후에 호출되니 base만으론 부족할 수 있음. **`HandlerMPCThread`에 명시적 destructor를 만들어 첫 줄에서 `Join()` 호출**, 그 후 derived 멤버 destruct.
4. Debug 빌드로 backtrace에 args/locals를 보여 `manager_` 포인터 값 + `this` 추적.

### Reference cores

`/tmp/core.mpc_main.*` 보관 중. PID/시각 별 매핑은 위 표 참조. 재개 시 `67594`/`67652` (Stage 2 fix 후 가장 최신) 우선 분석.

## 4. Files touched (검증된 변경만)

- `rtc_controller_manager/src/rt_controller_node_rt_loop.cpp` — CV wait predicate + StopRtLoop notify_all
- `rtc_controller_manager/src/rt_controller_main_impl.cpp` — executor cancel + remove_node
- `rtc_controller_manager/README.md` — RT 루프 종료 섹션

## 5. Validation matrix

| 테스트 | 결과 |
|--------|------|
| `colcon test --packages-select rtc_controller_manager` (gtest 직접) | lifecycle 9/9, switch 9/9, timing 17/17 PASS |
| `./build.sh full` | OK |
| sim Ctrl+C — 5초 SIGTERM 메시지 | 사라짐 ✓ |
| sim Ctrl+C — 15초 SIGKILL 메시지 | 사라짐 ✓ |
| sim Ctrl+C — exit code 0 | ✗ 여전히 -11 (mpc_main SEGV) |
